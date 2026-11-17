# -*- coding: utf-8 -*-
"""``laser_web.jobs`` 的单元测试：线程、状态、日志、取消、保留策略。

全部用独立的 ``JobRegistry(root=<临时目录>)``，不碰仓库根的 ``runs/``；
每个测试都 ``shutdown()`` 收尾，不留后台线程。

覆盖（对应设计文档 §7 的第 1、2、7 条）：

1. ``submit`` -> 轮询到 ``done``，``result`` 原样带出；
2. 任务内部抛异常 -> ``failed`` + ``error`` 有内容（不冒到调用方）；
3. ``request_cancel`` -> ``cancelled``（假任务自己看标志收工）；
4. 日志环形缓冲不超过 :data:`jobs.LOG_LIMIT` 行，且能按游标增量取；
5. ``runs/`` 超过 50 个时最旧的被删掉（直接造目录，不真跑 50 次）；
6. 收尾修复：``_JobStream`` 并发写不丢行、终态任务有上限、取消后删
   半成品运行目录。
"""

import os
import tempfile
import threading
import time
import unittest
from pathlib import Path

from laser_web import jobs

#: 轮询超时（秒）。本机任务都是毫秒级，10 s 已经非常宽松。
TIMEOUT = 10.0


def _wait(registry: jobs.JobRegistry, job_id: str,
          timeout: float = TIMEOUT) -> dict:
    """轮询到任务结束（或超时抛断言）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = registry.snapshot(job_id)
        if snapshot is not None and snapshot["state"] in jobs.TERMINAL_STATES:
            return snapshot
        time.sleep(0.01)
    raise AssertionError(f"任务 {job_id} 在 {timeout}s 内没有结束")


class JobRegistryTestCase(unittest.TestCase):
    """带临时产物目录的基类（每个测试一份 registry）。"""

    def setUp(self):
        """建临时目录与独立登记表。"""
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name) / "runs"
        self.registry = jobs.JobRegistry(self.root)

    def tearDown(self):
        """停掉工作线程并清临时目录。"""
        self.registry.shutdown()
        self._tmp.cleanup()


class SubmitTest(JobRegistryTestCase):
    """正常路径：跑完、结果、运行目录、状态迁移。"""

    def test_done_with_result(self):
        """``done`` 后快照里能拿到返回值，运行目录真的建了。"""

        def task(job):
            job.set_progress("干活", 0, 2)
            print("第一行")
            job.set_progress("干活", 2, 2)
            return {"ok": True, "value": 42}

        job_id = self.registry.submit("scan", task)
        self.assertRegex(job_id, r"^\d{8}-\d{6}-[0-9a-f]{4}$")
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "done")
        self.assertEqual(snapshot["result"], {"ok": True, "value": 42})
        self.assertIsNone(snapshot["error"])
        self.assertEqual(snapshot["kind"], "scan")
        run_dir = Path(snapshot["run_dir"])
        self.assertEqual(run_dir.name, job_id)
        self.assertTrue(run_dir.is_dir())
        self.assertEqual(run_dir.parent, self.root)
        self.assertIn("第一行", snapshot["log"])
        self.assertEqual(snapshot["progress"]["fraction"], 1.0)

    def test_unknown_job(self):
        """查不到的 id 返回 None / False，不抛异常。"""
        self.assertIsNone(self.registry.get("nope"))
        self.assertIsNone(self.registry.snapshot("nope"))
        self.assertFalse(self.registry.request_cancel("nope"))

    def test_cancel_after_done_is_rejected(self):
        """已结束的任务不再受理取消。"""
        job_id = self.registry.submit("scan", lambda job: {"ok": True})
        _wait(self.registry, job_id)
        self.assertFalse(self.registry.request_cancel(job_id))


class FailureTest(JobRegistryTestCase):
    """异常一律转成 ``failed``，不让 UI 线程见到 traceback。"""

    def test_exception_becomes_failed(self):
        """任务抛 ValueError -> failed + error 有内容。"""

        def task(job):
            print("开始")
            raise ValueError("模型文件坏了")

        job_id = self.registry.submit("scan", task)
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "failed")
        self.assertIn("ValueError", snapshot["error"])
        self.assertIn("模型文件坏了", snapshot["error"])
        self.assertIsNone(snapshot["result"])
        self.assertIn("开始", snapshot["log"])

    def test_system_exit_becomes_failed(self):
        """``strict`` 模式下算法会 ``sys.exit(2)``，也必须收成 failed。"""

        def task(job):
            raise SystemExit(2)

        job_id = self.registry.submit("scan", task)
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "failed")
        self.assertIn("SystemExit", snapshot["error"])


class CancelTest(JobRegistryTestCase):
    """取消：请求后状态必须是 cancelled，而不是 done。"""

    def test_cancel_running_job(self):
        """假任务自己检查标志，取消后状态变 ``cancelled``。"""

        def slow(job):
            for _ in range(200):
                if job.cancel.is_set():
                    raise jobs.JobCancelled("用户取消")
                time.sleep(0.01)
            return {"ok": True}

        job_id = self.registry.submit("scan", slow)
        deadline = time.monotonic() + TIMEOUT
        while self.registry.snapshot(job_id)["state"] == "pending":
            self.assertLess(time.monotonic(), deadline, "任务一直没开始")
            time.sleep(0.01)
        self.assertTrue(self.registry.request_cancel(job_id))
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "cancelled")
        self.assertIsNone(snapshot["error"])


class LogRingBufferTest(JobRegistryTestCase):
    """日志环形缓冲：最多 :data:`jobs.LOG_LIMIT` 行，可增量取。"""

    def test_ring_buffer_limit(self):
        """打印 1.5 倍上限的行数，缓冲区只留最后 200 行。"""

        def noisy(job):
            for index in range(jobs.LOG_LIMIT + 100):
                print(f"line {index}")
            return {"ok": True}

        job_id = self.registry.submit("scan", noisy)
        snapshot = _wait(self.registry, job_id)
        job = self.registry.get(job_id)
        self.assertEqual(len(job.log), jobs.LOG_LIMIT)
        self.assertEqual(job.log[0], f"line {jobs.LOG_LIMIT + 100 - jobs.LOG_LIMIT}")
        self.assertEqual(job.log[-1], f"line {jobs.LOG_LIMIT + 99}")
        self.assertEqual(snapshot["log_total"], jobs.LOG_LIMIT + 100)
        self.assertEqual(snapshot["dropped"], 100)
        self.assertEqual(len(snapshot["log"]), jobs.LOG_LIMIT)

    def test_log_cursor(self):
        """按游标取增量：两次调用拼起来等于全部日志。"""

        def task(job):
            for index in range(5):
                print(f"line {index}")
            return {"ok": True}

        job_id = self.registry.submit("scan", task)
        _wait(self.registry, job_id)
        first = self.registry.snapshot(job_id, 0)
        self.assertEqual(len(first["log"]), 5)
        second = self.registry.snapshot(job_id, first["log_from"])
        self.assertEqual(second["log"], [])
        self.assertEqual(second["log_from"], first["log_from"])


class RetentionTest(JobRegistryTestCase):
    """保留策略：``runs/`` 最多 50 个目录，超出删最旧的。"""

    def test_oldest_directories_are_pruned(self):
        """先造 60 个目录（mtime 递增），再提交任务触发清理。"""
        self.root.mkdir(parents=True, exist_ok=True)
        names = [f"20260101-0000{index:02d}-aaaa" for index in range(60)]
        for offset, name in enumerate(names):
            path = self.root / name
            path.mkdir()
            stamp = time.time() - (len(names) - offset) * 60.0
            os.utime(path, (stamp, stamp))

        job_id = self.registry.submit("scan", lambda job: {"ok": True})
        _wait(self.registry, job_id)

        remaining = sorted(path.name for path in self.root.iterdir())
        self.assertEqual(len(remaining), jobs.KEEP_RUNS)
        # 11 个最旧的被删（60 + 1 新建 - 50 保留）。
        for name in names[:11]:
            self.assertNotIn(name, remaining)
        for name in names[11:]:
            self.assertIn(name, remaining)
        self.assertIn(job_id, remaining)


class RunDirTest(unittest.TestCase):
    """运行目录与 id 的纯函数部分。"""

    def test_new_run_id_shape(self):
        """id 形状：时间戳-短随机。"""
        self.assertRegex(jobs.new_run_id(), r"^\d{8}-\d{6}-[0-9a-f]{4}$")

    def test_make_run_dir_creates_unique_dirs(self):
        """连续两次不重复，且都在给定的根目录下。"""
        with tempfile.TemporaryDirectory() as tmp:
            first_id, first = jobs.make_run_dir(tmp)
            second_id, second = jobs.make_run_dir(tmp)
            self.assertNotEqual(first_id, second_id)
            self.assertTrue(first.is_dir() and second.is_dir())
            self.assertEqual({first.parent, second.parent},
                             {Path(tmp).resolve()})


if __name__ == "__main__":
    unittest.main()


class PruneKeepsActiveRunsTest(unittest.TestCase):
    """清理不能删掉还在跑的任务目录（评审抓到的回归点）。"""

    def test_prune_skips_active_job_directory(self):
        import threading
        import time

        with tempfile.TemporaryDirectory() as tmp:
            registry = jobs.JobRegistry(tmp, keep=1)
            started = threading.Event()

            def slow(job):
                started.set()
                time.sleep(0.4)
                (job.run_dir / "scan.json").write_text("{}")
                return {"ok": True}

            first = registry.submit("scan", slow)
            self.assertTrue(started.wait(2.0))
            first_dir = registry.get(first).run_dir

            # 再提交两个任务：keep=1，若不做保护会把第一个（最旧）删掉
            for _ in range(2):
                registry.submit("scan", lambda job: {"ok": True})
            time.sleep(0.9)

            self.assertTrue(first_dir.is_dir(),
                            "正在跑的任务目录被 prune 删掉了")
            self.assertTrue((first_dir / "scan.json").exists())
            self.assertEqual(registry.get(first).state, "done")


def _stream(name: str) -> "jobs._JobStream":
    """建一个挂在裸 :class:`jobs.Job` 上的日志流（不启动工作线程）。"""
    job = jobs.Job(id=name, kind="scan", state="running", progress={},
                   log=[], result=None, error=None,
                   run_dir=Path("/nonexistent"), cancel=threading.Event())
    return jobs._JobStream(job)


class JobStreamConcurrencyTest(unittest.TestCase):
    """``_JobStream`` 的写入是**多线程并发**的（评审抓到 ``+=`` 缓冲无锁）。

    ``redirect_stdout`` 换的是进程级 ``sys.stdout``，任务线程与其它线程（UI 线程
    上的 ``core.preview_domain`` 之类）可能同时往里写；``str +=`` 是
    "读-拼-写"三步，被切线程就会丢字。
    """

    def test_write_takes_the_stream_lock(self):
        """锁真的被用上了：占住它，另一条线程的 ``write`` 必须等着。

        这是本类的**确定性**回归钉：原实现是裸的 ``self._buffer += text``，
        占锁对它没影响，写作线程会先把行追加进去。
        """
        stream = _stream("lock")
        stream._lock.acquire()
        try:
            worker = threading.Thread(target=stream.write, args=("占用锁期间\n",))
            worker.start()
            worker.join(0.3)
            self.assertTrue(worker.is_alive(), "write 没拿缓冲锁就写进去了")
            self.assertEqual(stream._job.log, [])
        finally:
            stream._lock.release()
        worker.join(2.0)
        self.assertEqual(stream._job.log, ["占用锁期间"])

    def test_concurrent_writes_do_not_lose_or_tear_lines(self):
        """8 条线程同时写 160 行：一行不丢、一行不残（压力测试）。"""
        stream = _stream("concurrency")
        threads, per_thread = 8, 20            # 160 行 < LOG_LIMIT，不会被挤掉
        barrier = threading.Barrier(threads)

        def writer(tag: int) -> None:
            barrier.wait()                     # 尽量让 8 条线程同时进 write
            for index in range(per_thread):
                stream.write(f"{tag:02d}-{index:03d}-" + "x" * 40 + "\n")

        workers = [threading.Thread(target=writer, args=(tag,))
                   for tag in range(threads)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        stream.flush()

        job = stream._job
        expected = {f"{tag:02d}-{index:03d}-" + "x" * 40
                    for tag in range(threads) for index in range(per_thread)}
        self.assertEqual(len(job.log), threads * per_thread)
        self.assertEqual(set(job.log), expected)
        self.assertEqual(job._log_total, threads * per_thread)

    def test_flush_keeps_partial_line(self):
        """没换行的尾巴由 ``flush`` 落进缓冲（不会丢到最后一行）。"""
        stream = _stream("flush")
        stream.write("没有换行")
        self.assertEqual(stream._job.log, [])
        stream.flush()
        self.assertEqual(stream._job.log, ["没有换行"])
        stream.flush()                         # 再 flush 一次不该重复追加
        self.assertEqual(stream._job.log, ["没有换行"])


class FinishedJobRetentionTest(JobRegistryTestCase):
    """登记表只保留最近若干条**已结束**任务，未结束的永不淘汰。"""

    def test_oldest_finished_jobs_are_evicted(self):
        registry = jobs.JobRegistry(self.root, keep=5, max_finished=2)
        try:
            ids = []
            for _ in range(4):
                job_id = registry.submit("scan", lambda job: {"ok": True})
                ids.append(job_id)
                _wait(registry, job_id)
            remaining = {job.id for job in registry.jobs()}
            self.assertNotIn(ids[0], remaining, "最旧的终态任务没被淘汰")
            self.assertEqual(len(remaining), 3)      # 2 条终态 + 刚提交的
            self.assertIn(ids[-1], remaining)
            self.assertIsNone(registry.snapshot(ids[0]))
        finally:
            registry.shutdown()

    def test_running_job_is_never_evicted(self):
        """``max_finished=0``（最激进的淘汰）也不能动未结束的任务。"""
        registry = jobs.JobRegistry(self.root, keep=5, max_finished=0)
        try:
            slow_id = registry.submit("scan",
                                      lambda job: (time.sleep(0.4),
                                                   {"ok": True})[1])
            pending_ids = [registry.submit("scan", lambda job: {"ok": True})
                           for _ in range(2)]
            self.assertIsNotNone(registry.get(slow_id))
            for job_id in pending_ids:
                _wait(registry, job_id)
            self.assertIsNotNone(registry.get(slow_id),
                                 "还在跑的任务被淘汰了")
            self.assertEqual(registry.get(slow_id).state, "done")
        finally:
            registry.shutdown()


class CancelCleanupTest(JobRegistryTestCase):
    """取消后的半成品运行目录要删；已落下 ``scan.json`` 的必须保留。"""

    def test_partial_run_dir_is_removed(self):
        def task(job):
            job.set_progress("出图", 1, 1)
            (job.run_dir / "plot.png").write_bytes(b"\x89PNG\r\n\x1a\n")
            raise jobs.JobCancelled("用户取消")

        job_id = self.registry.submit("scan", task)
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "cancelled")
        self.assertFalse(Path(snapshot["run_dir"]).exists(),
                         "只有 plot.png 的半成品目录没删掉")
        self.assertTrue(any("已删除半成品" in line for line in snapshot["log"]))

    def test_complete_run_dir_is_kept(self):
        def task(job):
            (job.run_dir / jobs.SCAN_ARTIFACT).write_text(
                "{}", encoding="utf-8")
            raise jobs.JobCancelled("取消发生在落盘之后")

        job_id = self.registry.submit("scan", task)
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "cancelled")
        run_dir = Path(snapshot["run_dir"])
        self.assertTrue((run_dir / jobs.SCAN_ARTIFACT).is_file())
        self.assertTrue(any("保留完整运行目录" in line for line in snapshot["log"]))

    def test_cancel_after_return_keeps_finished_dir(self):
        """结果算完但用户已点取消：状态 cancelled，产物完整，目录保留。"""

        def task(job):
            (job.run_dir / jobs.SCAN_ARTIFACT).write_text(
                "{}", encoding="utf-8")
            job.cancel.set()
            return {"ok": True}

        job_id = self.registry.submit("scan", task)
        snapshot = _wait(self.registry, job_id)
        self.assertEqual(snapshot["state"], "cancelled")
        self.assertIsNone(snapshot["result"])
        self.assertTrue((Path(snapshot["run_dir"])
                         / jobs.SCAN_ARTIFACT).is_file())


class ArtifactNameTest(unittest.TestCase):
    """``jobs.SCAN_ARTIFACT`` 必须与 ``core.ARTIFACT_SCAN_JSON`` 同名（防漂移）。"""

    def test_matches_core_constant(self):
        from laser_web import core

        self.assertEqual(jobs.SCAN_ARTIFACT, core.ARTIFACT_SCAN_JSON)


class MakeRunDirKeepTest(unittest.TestCase):
    """``make_run_dir`` 是"先建目录再 prune"，keep 至少按 1 算——新建的不会被删。"""

    def test_new_directory_survives_prune(self):
        with tempfile.TemporaryDirectory() as tmp:
            last = None
            for _ in range(5):
                last = jobs.make_run_dir(tmp, keep=1)[1]
                self.assertTrue(last.is_dir())
            self.assertTrue(last.is_dir(), "刚建的运行目录被 prune 删了")
            self.assertEqual([path.name for path in Path(tmp).iterdir()],
                             [last.name])

    def test_keep_zero_is_clamped_to_one(self):
        """``keep=0`` 也不会把刚建的删掉（``make_run_dir`` 里 max(1, keep)）。"""
        with tempfile.TemporaryDirectory() as tmp:
            _run_id, run_dir = jobs.make_run_dir(tmp, keep=0)
            self.assertTrue(run_dir.is_dir())
