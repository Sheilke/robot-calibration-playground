#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``jobs.Job`` 的订阅钩子（S3）：工作线程主动通知，网页版 ``snapshot`` 一字未改。

为什么这组测试放在 ``laser_desktop/tests`` 而不在 ``laser_web/tests``：钩子是给
桌面端加的那一条（设计文档 §3.3），但被测对象是 ``laser_web.jobs``。放这边
不会让网页版的测试集合多出"桌面需求"；网页版原有的 23 个 jobs 测试照旧跑，
本文件额外钉住"订阅"这一面。

四件事必须成立：

1. ``subscribe`` 之后，``append_log`` / ``set_progress`` / ``set_state`` 都会通知；
2. ``unsubscribe`` 之后不再通知；
3. 回调**在锁外**调用（回调里 ``snapshot()`` 不会死锁），回调抛异常不影响任务；
4. ``snapshot()`` 的字段与语义完全没变（与原来的轮询写法逐字段对齐）。
"""

import shutil
import tempfile
import threading
import time
import unittest
from pathlib import Path

from laser_web import jobs

#: 等任务结束的上限（秒）：假任务都是毫秒级，10 s 已经是异常兜底。
TIMEOUT = 10.0


def wait_terminal(job, timeout: float = TIMEOUT) -> None:
    """自旋等任务进入终态（不依赖 UI，纯 ``threading``）。"""
    deadline = time.monotonic() + timeout
    while job.state not in jobs.TERMINAL_STATES:
        if time.monotonic() > deadline:
            raise AssertionError(f"任务没在 {timeout}s 内结束：state={job.state}")
        time.sleep(0.005)


class SubscribeTest(unittest.TestCase):
    """订阅钩子的行为（假任务，不碰 IK）。"""

    def setUp(self):
        self.root = Path(tempfile.mkdtemp(prefix="jobs-subscribe-"))
        self.registry = jobs.JobRegistry(root=self.root)

    def tearDown(self):
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.root, ignore_errors=True)

    def _submit(self, fn) -> jobs.Job:
        """提交并取回 ``Job`` 对象（工作线程会立刻开跑）。"""
        job_id = self.registry.submit("scan", fn)
        return self.registry.get(job_id)

    def test_callback_sees_running_and_done(self):
        """订阅后至少要收到一次 running 与一次 done（进度更新时状态就是 running）。"""
        gate = threading.Event()
        seen = []

        def record(job):
            # 记下**通知发生那一刻**的状态：同一个 job 对象稍后会被改成 done
            seen.append((job.state, dict(job.progress)))

        def work(job):
            gate.wait(TIMEOUT)          # 等测试订阅完再动，避免错过通知
            job.set_progress("解算 IK", 1, 2)
            job.append_log("一行日志")
            return {"ok": True, "value": 7}

        job = self._submit(work)
        job.subscribe(record)
        gate.set()
        wait_terminal(job)

        states = [state for state, _ in seen]
        self.assertIn("running", states, f"没收到 running：{states}")
        self.assertIn("done", states, f"没收到 done：{states}")
        self.assertTrue(any(progress["stage"] == "解算 IK"
                            for state, progress in seen
                            if state == "running"))
        self.assertEqual(job.result, {"ok": True, "value": 7})

    def test_unsubscribe_stops_notifications(self):
        """注销之后不再收到任何回调。"""
        seen = []
        job = self._submit(lambda job: {"ok": True})
        wait_terminal(job)                       # 先让任务结束，避免竞态
        job.subscribe(seen.append)
        job.append_log("订阅前的一行")
        self.assertEqual(len(seen), 1)
        job.unsubscribe(seen.append)
        job.append_log("注销后的一行")
        job.set_progress("解算 IK", 1, 1)
        job.set_state("done", result={"ok": True})
        self.assertEqual(len(seen), 1, "unsubscribe 之后还在通知")

    def test_subscribe_is_idempotent(self):
        """同一个回调重复注册只生效一次（页面重建时不会越挂越多）。"""
        seen = []
        job = self._submit(lambda job: {"ok": True})
        wait_terminal(job)
        job.subscribe(seen.append)
        job.subscribe(seen.append)
        job.append_log("一行")
        self.assertEqual(len(seen), 1)

    def test_callback_runs_outside_the_lock(self):
        """回调里 ``snapshot()`` 必须能取到锁（否则就是持锁回调 → 死锁）。

        触发通知的线程单独开、主线程 ``join(timeout=5)``：将来真要是改成锁内
        回调，这里会**判失败**，而不是把整个测试会话挂在 ``append_log`` 里。
        """

        def read_snapshot(job):
            snapshot = job.snapshot(0)
            self.assertIsInstance(snapshot["log"], list)

        job = self._submit(lambda job: {"ok": True})
        wait_terminal(job)
        job.subscribe(read_snapshot)
        probe = threading.Thread(target=job.append_log,
                                 args=("这行不会把线程锁死",),
                                 name="subscribe-lock-probe", daemon=True)
        probe.start()
        probe.join(timeout=5)
        self.assertFalse(probe.is_alive(),
                         "回调在持锁状态下执行：snapshot() 死锁（5 s 未返回）")
        self.assertIn("这行不会把线程锁死", job.snapshot(0)["log"])

    def test_listener_exception_does_not_break_the_job(self):
        """监听器自己炸了不能拖垮任务（桌面端一个槽出错不该让解算失败）。

        但不能静默：要在日志里留下完整栈（S3 评审第 9 条）。
        """
        calls = []

        def boom(job):
            calls.append(job.state)
            raise RuntimeError("监听器的 bug")

        def work(job):
            job.append_log("先写一行")           # 这里会触发一次 boom
            return {"ok": True}

        with self.assertLogs("laser_web.jobs", level="WARNING") as logs:
            job = self._submit(work)
            job.subscribe(boom)
            wait_terminal(job)
            job.append_log("再来一行")           # 确保至少一次告警落在断言区间内
        self.assertEqual(job.state, "done")
        self.assertEqual(job.result, {"ok": True})
        self.assertTrue(calls)
        text = "\n".join(logs.output)
        self.assertIn("订阅回调异常", text)
        self.assertIn("监听器的 bug", text)       # exc_info=True：栈里有原因

    def test_notification_comes_from_the_worker_thread(self):
        """通知来自 ``jobs`` 的工作线程（桌面端据此只许 emit 信号）。"""
        names = []
        gate = threading.Event()

        def work(job):
            gate.wait(TIMEOUT)
            job.set_progress("解算 IK", 1, 2)
            return {"ok": True}

        job = self._submit(work)
        job.subscribe(lambda item: names.append(threading.current_thread().name))
        gate.set()
        wait_terminal(job)
        self.assertTrue(names)
        self.assertEqual(set(names), {"laser-web-jobs"},
                         f"通知线程不对：{set(names)}")

    def test_snapshot_shape_is_unchanged(self):
        """``snapshot()`` 的字段与游标语义完全没变（网页版仍按它轮询）。"""
        job = self._submit(lambda job: {"ok": True})
        wait_terminal(job)
        first = job.snapshot(0)
        self.assertEqual(
            set(first),
            {"id", "kind", "state", "progress", "log", "log_from", "log_total",
             "dropped", "result", "error", "run_dir"})
        self.assertEqual(first["log_from"], len(first["log"]))
        self.assertEqual(job.snapshot(first["log_from"])["log"], [])


if __name__ == "__main__":
    unittest.main()
