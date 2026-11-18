#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``JobBridge``（S3）：工作线程 → Qt 信号，进度/日志/终态/取消/接管。

用**真** ``JobRegistry`` + 假任务（sleep 几十毫秒），不跑 IK：这里要验的是
"信号有没有把工作线程的状态原样搬到 GUI 线程"，不是算法。真 IK 的那条在
``test_generate_scan.py``。

测试全部在 offscreen 下跑（建 QApplication 后转事件循环等信号）。
"""

import os
import re
import shutil
import tempfile
import threading
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import job_bridge
from laser_desktop.job_bridge import JobBridge, adopt_running_job
from laser_desktop.qt_compat import QtWidgets
from laser_web import jobs

#: 等信号的上限（秒）。
TIMEOUT = 30.0

_APP = None


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-bridge-tests"])
    return _APP


def wait_until(predicate, timeout: float = TIMEOUT) -> bool:
    """转事件循环直到 ``predicate()`` 为真（超时返回 False）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app().processEvents()
        if predicate():
            return True
        time.sleep(0.005)
    app().processEvents()
    return bool(predicate())


class BridgeTestBase(unittest.TestCase):
    """每条用例一个临时 ``runs/`` 根 + 一个桥（flush_ms 调小让用例快）。"""

    flush_ms = 50

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="bridge-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=self.flush_ms)
        self.events = {"progress": [], "log": [], "finished": []}
        self.bridge.progress.connect(self.events["progress"].append)
        self.bridge.log_line.connect(self.events["log"].append)
        self.bridge.finished.connect(self.events["finished"].append)

    def tearDown(self):
        self.bridge.shutdown()
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.root, ignore_errors=True)

    def wait_finished(self, timeout: float = TIMEOUT) -> dict:
        """等 ``finished`` 信号，返回它的 payload（超时直接失败）。"""
        ok = wait_until(lambda: bool(self.events["finished"]), timeout)
        self.assertTrue(ok, f"没等到 finished；进度={self.events['progress'][-1:]} "
                            f"日志={self.events['log'][-3:]}")
        return self.events["finished"][-1]


class DoneTest(BridgeTestBase):
    """正常结束：``finished`` 里的 state/result/run_id/run_dir 与登记表一致。"""

    def test_finished_carries_state_result_and_run_dir(self):
        expected = {"ok": True, "value": 42, "rows": [1, 2, 3]}

        def work(job):
            job.set_progress("解算 IK", 1, 3)
            print("第一行")
            time.sleep(0.05)
            return dict(expected)

        job_id = self.bridge.submit("scan", work)
        payload = self.wait_finished()

        self.assertEqual(payload["state"], "done")
        self.assertEqual(payload["result"], expected)
        self.assertIsNone(payload["error"])
        self.assertEqual(payload["run_id"], job_id)
        self.assertEqual(payload["run_dir"],
                         str(self.registry.get(job_id).run_dir))
        self.assertEqual(len(self.events["finished"]), 1, "finished 只能发一次")

    def test_log_lines_are_complete_and_not_duplicated(self):
        """30 行日志一行不漏、一行不重（游标语义与前缀去重）。"""
        lines = [f"日志行 {index}" for index in range(30)]

        def work(job):
            for line in lines:
                print(line)
            return {"ok": True}

        self.bridge.submit("scan", work)
        payload = self.wait_finished()
        self.assertEqual(payload["state"], "done")
        self.assertEqual(self.events["log"], lines)

    def test_dropped_log_lines_are_announced(self):
        """源缓冲挤掉旧行时必须插一行提示（S3 评审第 2 条）。

        写 ``LOG_LIMIT + 120`` 行：旧实现只默默给最后 200 行，看上去就像
        解算跑到一半断了。现在每丢一段就插一句“…已省略 N 行”，N 对得上。
        """
        total = jobs.LOG_LIMIT + 120

        def work(job):
            for index in range(total):
                job.append_log(f"line {index + 1}")
            return {"ok": True}

        self.bridge.submit("scan", work)
        self.wait_finished()
        notices = [line for line in self.events["log"] if "已省略" in line]
        self.assertGreaterEqual(len(notices), 1,
                                f"丢了 {total - jobs.LOG_LIMIT} 行却一声不响")
        omitted = [int(re.search(r"已省略 (\d+) 行", line).group(1))
                   for line in notices]
        self.assertEqual(sum(omitted), total - jobs.LOG_LIMIT,
                         f"提示的行数与实际丢弃对不上：{notices}")
        self.assertIn(str(job_bridge.LOG_LIMIT), notices[0])
        kept = [line for line in self.events["log"] if line.startswith("line ")]
        self.assertEqual(kept,
                         [f"line {index}" for index in
                          range(total - jobs.LOG_LIMIT + 1, total + 1)])

    def test_progress_payload_has_the_documented_keys(self):
        def work(job):
            job.set_progress("解算 IK", 2, 4)
            return {"ok": True}

        self.bridge.submit("scan", work)
        self.wait_finished()
        self.assertTrue(self.events["progress"], "没收到任何进度")
        for item in self.events["progress"]:
            self.assertTrue({"stage", "index", "total", "fraction"} <= set(item))
        fractions = [item["fraction"] for item in self.events["progress"]]
        self.assertEqual(fractions, sorted(fractions), "进度分数不单调")
        self.assertEqual(fractions[-1], 1.0)

    def test_progress_is_coalesced_not_flooded(self):
        """几百次进度更新要被合并成少数几次 emit（不然事件循环会被淹）。"""
        def work(job):
            for index in range(300):
                job.set_progress("解算 IK", index, 300)
                time.sleep(0.001)
            return {"ok": True}

        self.bridge.submit("scan", work)
        self.wait_finished()
        self.assertGreaterEqual(len(self.events["progress"]), 2)
        self.assertLess(len(self.events["progress"]), 60,
                        f"合并失效：收到 {len(self.events['progress'])} 次进度")

    def test_signals_arrive_on_the_gui_thread(self):
        """信号槽必须在 GUI（主）线程执行——跨线程碰控件的根因就在这里。"""
        received = []

        def work(job):
            print("来自工作线程的一行")
            return {"ok": True}

        self.bridge.log_line.connect(
            lambda _line: received.append(threading.current_thread()))
        self.bridge.finished.connect(
            lambda _payload: received.append(threading.current_thread()))
        self.bridge.submit("scan", work)
        self.wait_finished()
        self.assertTrue(received)
        self.assertEqual({thread is threading.main_thread() for thread in received},
                         {True}, "有信号在非 GUI 线程里被处理")


class FailureTest(BridgeTestBase):
    """任务抛异常：state=failed，error 带类型名，日志里有原文。"""

    def test_exception_becomes_failed(self):
        def work(job):
            print("开始干活")
            raise ValueError("解算炸了")

        self.bridge.submit("scan", work)
        payload = self.wait_finished()
        self.assertEqual(payload["state"], "failed")
        self.assertIsNone(payload["result"])
        self.assertIn("ValueError", payload["error"])
        self.assertIn("解算炸了", payload["error"])
        self.assertTrue(any("解算炸了" in line for line in self.events["log"]))


class CancelTest(BridgeTestBase):
    """取消：``cancel()`` → 终态 cancelled（不是 failed）。"""

    def test_cancel_ends_cancelled(self):
        def work(job):
            for _ in range(400):
                if job.cancel.is_set():
                    raise jobs.JobCancelled("用户取消")
                job.set_progress("解算 IK", 1, 2)
                time.sleep(0.005)
            return {"ok": True}

        self.bridge.submit("scan", work)
        self.assertTrue(wait_until(lambda: self.bridge.is_running))
        self.assertTrue(self.bridge.cancel(), "cancel 应该被受理")
        payload = self.wait_finished()
        self.assertEqual(payload["state"], "cancelled")
        self.assertFalse(self.bridge.is_running)

    def test_cancel_without_a_job_returns_false(self):
        self.assertFalse(self.bridge.cancel())
        self.assertFalse(self.bridge.is_running)


class AttachTest(BridgeTestBase):
    """接管：页面重建后按 job_id 接回在跑的任务 / 已经结束的任务。"""

    def test_attach_to_a_finished_job_emits_finished_immediately(self):
        marker = {"ok": True, "value": "已经跑完"}
        job_id = self.registry.submit("scan", lambda job: dict(marker))
        job = self.registry.get(job_id)
        deadline = time.monotonic() + TIMEOUT
        while job.state not in jobs.TERMINAL_STATES:
            self.assertLess(time.monotonic(), deadline, "任务没结束")
            time.sleep(0.005)

        fresh = JobBridge(registry=self.registry, flush_ms=self.flush_ms)
        try:
            received = []
            fresh.finished.connect(received.append)
            self.assertIs(fresh.attach(job_id), job)
            # attach 里会补一次现状：已经结束的任务立刻就能拿到终态
            app().processEvents()
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0]["state"], "done")
            self.assertEqual(received[0]["result"], marker)
            self.assertEqual(received[0]["run_id"], job_id)
        finally:
            fresh.shutdown()

    def test_attach_running_job_then_receives_finished(self):
        gate = threading.Event()

        def work(job):
            print("接管前写的日志")
            gate.wait(TIMEOUT)
            return {"ok": True, "value": "接管成功"}

        job_id = self.registry.submit("scan", work)
        fresh = JobBridge(registry=self.registry, flush_ms=self.flush_ms)
        try:
            received = {"log": [], "finished": []}
            fresh.log_line.connect(received["log"].append)
            fresh.finished.connect(received["finished"].append)
            fresh.attach(job_id)
            # 工作线程什么时候打印不由测试决定：等它真的进了桥（attach 的补读
            # 与刷新定时器都会把它带过来），不靠一次 processEvents 赌运气
            self.assertTrue(
                wait_until(lambda: "接管前写的日志" in received["log"]),
                f"接管后没收到之前写好的日志：{received['log']}")
            gate.set()
            self.assertTrue(wait_until(lambda: bool(received["finished"])),
                            "接管后没等到 finished")
            self.assertEqual(received["finished"][-1]["state"], "done")
            self.assertEqual(received["finished"][-1]["result"]["value"], "接管成功")
        finally:
            gate.set()
            fresh.shutdown()

    def test_detach_stops_the_bridge(self):
        gate = threading.Event()

        def work(job):
            gate.wait(TIMEOUT)
            print("detach 之后的一行")
            return {"ok": True}

        self.bridge.submit("scan", work)
        self.bridge.detach()
        self.assertFalse(self.bridge.is_running)
        gate.set()
        wait_until(lambda: False, timeout=0.3)   # 给工作线程时间把日志写完
        self.assertEqual(self.events["finished"], [])
        self.assertEqual(self.events["log"], [])

    def test_adopt_running_job_finds_the_unfinished_one(self):
        gate = threading.Event()

        def work(job):
            gate.wait(TIMEOUT)
            return {"ok": True}

        job_id = self.registry.submit("scan", work)
        adopted = adopt_running_job(self.registry, "scan")
        try:
            self.assertIsNotNone(adopted)
            self.assertEqual(adopted.id, job_id)
            self.assertIsNone(adopt_running_job(self.registry, "repair"))
        finally:
            gate.set()
            self.registry.shutdown(timeout=TIMEOUT)


class ShutdownTest(BridgeTestBase):
    """``shutdown()`` 之后不再有信号（窗口关掉后定时器不能再跑）。"""

    def test_shutdown_stops_signals(self):
        self.bridge.shutdown()
        self.assertFalse(self.bridge._timer.isActive())
        self.assertIsNone(self.bridge.job_id)
        self.assertFalse(self.bridge.cancel())


class StateTextTest(unittest.TestCase):
    """状态文案与 ``jobs.STATES`` 对得上（页面直接用）。"""

    def test_every_state_has_a_label(self):
        self.assertEqual(set(job_bridge.STATE_TEXT), set(jobs.STATES))


if __name__ == "__main__":
    unittest.main()
