#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 五点页真跑（S5）：一次 200 mm 立方体解算（走 ``JobBridge``）。

案子照站点参数缩到 200 mm 边长：域 ``925..1125 / -100..100 / 505..705``、
``rpy 3 73 -167``、``euler xyz-intrinsic``、``q-seed 43,-30,-36,42,-89,-1``。
本机实测**整次解算 0.4 s 左右**（5 点 IK + matplotlib 出图 + 写 JSON），远低于
60 s 的验收上限；耗时在测试里打印出来（报告里也写了）。

断言：

1. 终态 ``done``、``ok`` 为真、5 个点全可解；
2. 路点表 5 行、``|J5|min`` 有值、构型一致；
3. ``runs/<id>/`` 里真的有 ``iso.json`` 与 ``iso.png``，导出按钮点亮；
4. 线程安全沿用 S3/S4 的判据：``sys.excepthook`` / ``threading.excepthook``
   抓到的异常必须为空，且 ``finished`` 信号在**主线程**发出（跨线程碰控件会
   在这里炸）。
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

from laser_desktop import iso_page
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtWidgets
from laser_web import core, jobs

#: 真跑的上限（秒）：本机实测 0.4 s，慢机器/CI 也远用不到这么多。
TIMEOUT = 180.0

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-iso-run-tests"])
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


def configure_small_cube(page) -> None:
    """把表单改成 200 mm 立方体（站点位置，5 点全可解）。"""
    fields = page.fields
    fields["cube_bounds"].setChecked(True)
    for key, value in (("x0", 925.0), ("x1", 1125.0), ("y0", -100.0),
                       ("y1", 100.0), ("z0", 505.0), ("z1", 705.0)):
        fields[key].setValue(value)
    fields["layout"].setCurrentIndex(fields["layout"].findData("vertices"))
    fields["plane"].setCurrentIndex(fields["plane"].findData("xz"))
    fields["rpy_a"].setValue(3.0)
    fields["rpy_b"].setValue(73.0)
    fields["rpy_c"].setValue(-167.0)
    fields["euler"].setCurrentIndex(fields["euler"].findData("xyz-intrinsic"))
    fields["q_seed"].setText("43,-30,-36,42,-89,-1")
    fields["j5_min"].setValue(20.0)


class RealIsoRunTest(unittest.TestCase):
    """从「生成五点」到结果表、图、产物、导出的全链路。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="iso-run-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = iso_page.IsoPage(plot=None, bridge=self.bridge)
        self.errors = []
        self.finished_threads = []
        self._previous_excepthook = sys.excepthook
        self._previous_thread_hook = getattr(threading, "excepthook", None)
        sys.excepthook = lambda *info: self.errors.append(info)
        threading.excepthook = lambda info: self.errors.append(info)
        # 记录 finished 是在哪个线程发出的（Qt 的排队连接必须落在主线程）
        self.bridge.finished.connect(
            lambda _payload: self.finished_threads.append(
                threading.current_thread().name))

    def tearDown(self):
        sys.excepthook = self._previous_excepthook
        if self._previous_thread_hook is not None:
            threading.excepthook = self._previous_thread_hook
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=30)
        shutil.rmtree(self.root, ignore_errors=True)

    def test_real_run_produces_five_points(self):
        configure_small_cube(self.page)
        preview = self.page.refresh_preview()
        self.assertTrue(preview["ok"], preview.get("error"))
        self.assertEqual(preview["side"], 200.0)
        self.assertTrue(self.page.generate_button.isEnabled())

        started = time.monotonic()
        job_id = self.page.start_iso()
        self.assertTrue(job_id, "应当起任务")
        self.assertTrue(self.page.cancel_button.isVisibleTo(self.page))
        self.assertTrue(wait_until(
            lambda: self.page.run_state in ("done", "failed", "cancelled")))
        elapsed = time.monotonic() - started
        print(f"  [ISO 真跑] {elapsed:.2f} s（上限 {TIMEOUT:.0f} s）")

        result = self.page.last_result or {}
        self.assertEqual(self.page.run_state, "done", result.get("error"))
        self.assertTrue(result.get("ok"), result.get("error"))
        self.assertEqual(result["stats"]["total"], 5)
        self.assertEqual(result["stats"]["solved"], 5)
        j5_min = result["stats"]["j5_abs_min"]
        self.assertIsNotNone(j5_min)
        # 有值、而且是“真的算过”（不是 0 / nan）：默认模型下这组参数约 65°
        self.assertGreater(j5_min, 0.0)
        self.assertLess(j5_min, 90.0)
        self.assertTrue(all(point["j5_abs"] is not None
                            for point in result["points"]))
        self.assertTrue(result["stats"]["config_consistent"])

        # 结果区
        self.assertEqual(self.page.point_table.rowCount(), 5)
        self.assertEqual(self.page.stat_labels["solved"].text(), "IK 成功 5/5")
        self.assertNotEqual(self.page.stat_labels["j5"].text(), "|J5|min —")
        self.assertEqual(self.page.progress_bar.value(), 1000)

        # 产物：iso.json / iso.png 都在 run_dir 里
        run_dir = Path(result["run_dir"])
        self.assertTrue((run_dir / core.ARTIFACT_ISO_JSON).is_file())
        self.assertTrue((run_dir / core.ARTIFACT_ISO_PLOT).is_file())
        for key, button in self.page.export_buttons.items():
            self.assertTrue(button.isEnabled(), f"{key} 导出按钮应当点亮")

        # iso.json 与 scan.json 同构：5 条路点，export_gp 能直接吃
        payload = json.loads((run_dir / core.ARTIFACT_ISO_JSON)
                             .read_text(encoding="utf-8"))
        self.assertEqual(len(payload["waypoints"]), 5)
        self.assertEqual(payload["meta"]["motion"], "MOVL")

        # 线程安全：没有异常逃逸，finished 在主线程发出
        self.assertEqual(self.errors, [])
        self.assertTrue(self.finished_threads)
        self.assertEqual(set(self.finished_threads),
                         {threading.main_thread().name})
        self.assertFalse(self.bridge.is_running)
        self.assertTrue(self.page.generate_button.isEnabled(),
                        "任务结束后按钮要放开")

    def test_validation_error_is_shown_without_starting_job(self):
        """下界 > 上界：不起任务，报错原文红字写在卡片和状态行。"""
        self.page.fields["x0"].setValue(5000.0)
        job_id = self.page.start_iso()
        self.assertIsNone(job_id)
        self.assertIn("下界必须小于上界", self.page.error_label.text())
        self.assertIn("参数不合法", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertFalse(self.bridge.is_running)


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
