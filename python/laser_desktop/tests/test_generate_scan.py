#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真跑一次 8 点小案子（S3 的线程安全冒烟）：真 IK、真信号、真结果表。

这是桌面版第一次真跑 ``core.run_scan``：参数是规格里给的固定小案子

* 域 ``900..1100 / -100..100 / 500..700``（六个轴边界）
* ``target_count 8``、``rpy 3 73 -167``、``euler xyz-intrinsic``
* ``q-seed 43,-30,-36,42,-89,-1``、``j5_min 20``、``seed 0``

实测 < 1 s，因此放在 pytest 里是安全的。断言三件事：

1. ``finished`` 收到 ``done`` 且 ``solved == 8``（真解算，不是造的）；
2. 结果表 8 行、统计标签与 ``stats`` 一致、``runs/<id>/scan.json`` 真的写出来了；
3. 全程没有跨线程异常：``sys.excepthook`` / ``threading.excepthook`` 抓到的
   异常为空，且所有信号槽都在主线程执行（跨线程碰控件会在这里炸掉）。
"""

import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import generate_page
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtWidgets
from laser_web import jobs

#: 真解算的超时（秒）：8 点实测 < 1 s，给足 3 倍余量（慢机器/CI）。
TIMEOUT = 180.0

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-scan-tests"])
    return _APP


def wait_until(predicate, timeout: float = TIMEOUT) -> bool:
    """转事件循环直到 ``predicate()`` 为真（超时返回 False）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app().processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    app().processEvents()
    return bool(predicate())


def wait(ms: float) -> None:
    """真跑 ``ms`` 毫秒事件循环（去抖定时器要靠它才响）。"""
    deadline = time.monotonic() + ms / 1000.0
    while True:
        app().processEvents()
        if time.monotonic() >= deadline:
            return
        time.sleep(0.005)


class StubPlot(QtWidgets.QWidget):
    """只记下被推过来的图（真 PlotView 要 WebEngine）。"""

    def __init__(self):
        super().__init__()
        self.figures = []

    def set_figure(self, figure):
        self.figures.append(figure)


def configure_small_case(page) -> None:
    """把表单改成规格里的 8 点小案子。"""
    fields = page.fields
    fields["domain_axes"].setChecked(True)
    for key, value in (("x_min", 900.0), ("x_max", 1100.0),
                       ("y_min", -100.0), ("y_max", 100.0),
                       ("z_min", 500.0), ("z_max", 700.0)):
        fields[key].setValue(value)
    fields["grid_count"].setChecked(True)
    fields["target_count"].setValue(8)
    fields["rpy_a"].setValue(3.0)
    fields["rpy_b"].setValue(73.0)
    fields["rpy_c"].setValue(-167.0)
    fields["euler"].setCurrentIndex(fields["euler"].findData("xyz-intrinsic"))
    fields["q_seed"].setText("43,-30,-36,42,-89,-1")
    fields["j5_min"].setValue(20.0)
    fields["seed"].setValue(0)


class RealScanTest(unittest.TestCase):
    """真跑 8 点：从点按钮到结果表、导出、scan.json 全链路。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="scan-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.plot = StubPlot()
        self.page = generate_page.GeneratePage(plot=self.plot, bridge=self.bridge)
        # 跨线程异常会进 excepthook（Qt 槽里抛异常在 PyQt5 会直接 abort，抓不到；
        # 能抓到的部分一个都不许有）
        self.caught = []
        self._old_excepthook = sys.excepthook
        self._old_thread_hook = threading.excepthook
        sys.excepthook = lambda *args: self.caught.append(args)
        threading.excepthook = lambda args: self.caught.append(args)

    def tearDown(self):
        sys.excepthook = self._old_excepthook
        threading.excepthook = self._old_thread_hook
        self.page.shutdown()
        self.page.deleteLater()
        self.plot.deleteLater()
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.root, ignore_errors=True)

    def _run_small_case(self):
        configure_small_case(self.page)
        self.assertEqual(self.page.total.count, 8,
                         f"表单没落到 8 点：{self.page.total}")
        job_id = self.page.start_scan()
        self.assertIsNotNone(job_id, "start_scan 没起任务")
        finished_on = []
        self.bridge.finished.connect(
            lambda _payload: finished_on.append(threading.current_thread()))
        self.assertTrue(
            wait_until(lambda: self.page.run_state in jobs.TERMINAL_STATES),
            f"解算没在 {TIMEOUT}s 内结束：state={self.page.run_state} "
            f"进度={self.page.progress_bar.format()}")
        return job_id, finished_on

    def test_eight_point_case_runs_to_done(self):
        job_id, finished_on = self._run_small_case()

        self.assertEqual(self.page.run_state, "done",
                         f"没跑完：{self.page.status_label.text()}")
        result = self.page.last_result
        self.assertIsNotNone(result, "没有结果")
        self.assertEqual(result["run_id"], job_id)
        stats = result["stats"]
        self.assertEqual(stats["solved"], 8)
        self.assertEqual(stats["total"], 8)

        # ---- 结果区 ----
        self.assertEqual(self.page.waypoint_table.rowCount(), 8)
        self.assertEqual(self.page.segment_table.rowCount(), 7)
        self.assertEqual(self.page.progress_bar.value(), 1000)
        self.assertIn("完成", self.page.progress_bar.format())
        self.assertIn("完成", self.page.status_label.text())
        self.assertTrue(self.page.cancel_button.isHidden())
        self.assertTrue(self.page.generate_button.isEnabled())
        self.assertTrue(all(button.isEnabled()
                            for button in self.page.export_buttons.values()))
        self.assertEqual(self.page.stat_labels["solved"].text(), "成功 8/8")
        self.assertEqual(
            self.page.stat_labels["dq_l2"].text(),
            f"ΣdqL2 {stats['dq_l2_sum']:.2f}")

        # 8 点全都解出来了 → 没有红底行
        red = generate_page.QtGui.QColor(generate_page._FAILED_BG).name()
        for row in range(self.page.waypoint_table.rowCount()):
            self.assertNotEqual(self.page.waypoint_table.item(row, 0)
                                .background().color().name(), red, f"GP{row + 1}")

        # ---- 产物与图 ----
        run_dir = Path(result["run_dir"])
        self.assertTrue((run_dir / "scan.json").is_file())
        payload = json.loads((run_dir / "scan.json").read_text(encoding="utf-8"))
        self.assertEqual(len(payload["waypoints"]), 8)
        self.assertEqual(sum(1 for step in payload["perturbed_ik"] if step["ok"]), 8)
        self.assertEqual(payload["singularity_guard"]["j5_abs_min_deg"],
                         stats["j5_abs_min"])
        self.assertEqual([row["pos"] for row in payload["waypoints"]],
                         [row["pos"] for row in result["waypoints"]])
        self.assertIn("joints.txt", result["artifacts"].values())
        # 只推过**一张**图，而且必须恰好是结果图（无名 trace 就是预览图）
        names = [str(getattr(figure.data[0], "name", ""))
                 for figure in self.plot.figures]
        self.assertEqual(len(self.plot.figures), 1,
                         f"推了 {len(self.plot.figures)} 张图：{names}")
        self.assertIs(self.plot.figures[-1], self.page.last_figure)
        good = self.page.last_figure.data[0]
        self.assertIn("成功", str(good.name))
        self.assertIn("按路径顺序", str(good.name))
        self.assertEqual(len(good.x), 8)

        # ---- 线程安全 ----
        self.assertEqual(self.caught, [], f"有异常：{self.caught}")
        self.assertEqual({thread is threading.main_thread()
                          for thread in finished_on}, {True})

    def test_result_figure_survives_stale_preview_debounce(self):
        """P1：小案子跑完后，陈旧预览去抖不许把结果图盖掉（连跑 3 次）。

        两段判据：

        1. 点按钮时排着的去抖定时器必须被 ``start_scan`` 杀掉（不依赖解算
           快慢）；
        2. 就算它后来又响了一次（直接 ``_debounce.start()`` 复现竞态，不赌
           解算时长），只许有 **1** 张图，且那张是无名 trace 的结果图。
        旧实现在这里会多推一张无名的预览图，并把 ``last_figure`` 改写成它。
        """
        for attempt in range(1, 4):
            self.plot.figures.clear()
            self.page._debounce.start()        # 模拟“点按钮前排着的定时器”
            self._run_small_case()
            self.assertFalse(self.page._debounce.isActive(),
                             f"第 {attempt} 次起任务时没杀掉陈旧去抖")
            wait(generate_page.DEBOUNCE_MS + 150)
            figures = self.plot.figures
            self.assertEqual(
                len(figures), 1,
                f"第 {attempt} 次推了 {len(figures)} 张图（结果图被预览图盖掉）："
                f"{[str(getattr(figure.data[0], 'name', '')) for figure in figures]}")
            self.assertIs(figures[-1], self.page.last_figure)
            name = str(figures[-1].data[0].name)
            self.assertEqual(name, "成功 8 点 · 按路径顺序",
                             f"第 {attempt} 次推的不是结果图：{name!r}")
            self.assertEqual(len(figures[-1].data[0].x), 8)
            # 二、把那个“陈旧的”定时器补响一次：不得拿预览图盖结果
            self.page._debounce.start()
            wait(generate_page.DEBOUNCE_MS + 150)
            self.assertEqual(len(self.plot.figures), 1,
                             f"第 {attempt} 次：去抖响一次就又推了一张图")
            self.assertIs(self.page.last_figure, figures[-1])

    def test_form_change_after_run_returns_to_preview(self):
        """跑完再改表单→图回到预览（P1 的修法不能把这条语义一起屏蔽掉）。"""
        self.plot.figures.clear()
        self._run_small_case()
        wait(generate_page.DEBOUNCE_MS + 150)
        self.assertEqual(len(self.plot.figures), 1)
        self.page.fields["rpy_a"].setValue(5.0)       # 不影响点数/合法性
        wait(generate_page.DEBOUNCE_MS + 150)
        self.assertEqual(len(self.plot.figures), 2, "改表单后没回到预览图")
        preview = self.plot.figures[-1]
        self.assertIs(preview, self.page.last_figure)
        self.assertTrue(str(preview.layout.title.text).startswith("路点预览"),
                        preview.layout.title.text)
        self.assertIn(preview.data[0].name, (None, ""),
                      "预览图不该带 trace 名（那是结果图的标志）")
        # 切换面板（不走 _on_change 的入口）也要算“表单变了”，得回到预览
        self.page.fields["grid_explicit"].setChecked(True)
        wait(generate_page.DEBOUNCE_MS + 150)
        self.assertEqual(len(self.plot.figures), 3, "切换面板后没回到预览图")
        self.assertIs(self.plot.figures[-1], self.page.last_figure)

    def test_progress_and_log_arrive_while_running(self):
        """跑的过程中进度真的动过、日志真的有内容（不是结束后一次性补）。"""
        configure_small_case(self.page)
        stages = []
        self.bridge.progress.connect(lambda item: stages.append(item["stage"]))
        self.page.start_scan()
        self.assertTrue(
            wait_until(lambda: self.page.run_state in jobs.TERMINAL_STATES),
            "解算没结束")
        self.assertEqual(self.page.run_state, "done")
        self.assertGreaterEqual(len(set(stages)), 2, f"进度只有 {set(stages)}")
        self.assertGreater(self.page.log_edit.blockCount(), 1,
                           "日志区是空的（stdout 没被收进 Job）")
        self.assertLessEqual(self.page.log_edit.blockCount(),
                             generate_page.LOG_WIDGET_LIMIT)

    def test_cancel_before_start_ends_cancelled(self):
        """取消路径不报红：state=cancelled，状态行是灰字。

        做法：先用一个慢任务占住工作线程（``JobRegistry`` 只有一条工作线程），
        于是本页提交的任务只能排队；此时取消→置标志，再放开工线程，排队的任务
        会在 ``_run_one`` 开头就被判为 cancelled（不会真的跑 IK）。
        """
        configure_small_case(self.page)
        blocker = threading.Event()
        self.registry.submit("scan", lambda job: blocker.wait(30))
        try:
            job_id = self.page.start_scan()
            self.assertIsNotNone(job_id)
            self.assertFalse(self.page.cancel_button.isHidden())
            self.assertTrue(self.bridge.cancel(), "cancel 应该被受理")
            blocker.set()                   # 放开工作线程，排队的任务立刻判取消
            self.assertTrue(
                wait_until(lambda: self.page.run_state in jobs.TERMINAL_STATES),
                "取消后没到终态")
            self.assertEqual(self.page.run_state, "cancelled")
            self.assertIn("已取消", self.page.status_label.text())
            # 灰字中性提示：不能报红（取消是现场工程师自己点的）
            self.assertIn("color:#6b7280", self.page.status_label.styleSheet())
            self.assertIsNone(self.page.last_result)
            self.assertEqual(self.page.waypoint_table.rowCount(), 0)
            self.assertTrue(self.page.cancel_button.isHidden())
        finally:
            blocker.set()


if __name__ == "__main__":
    unittest.main()
