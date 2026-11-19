#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 五点页真算（S5）：点一次「生成五点」得到坐标与可达性（主线程）。

案子：200 mm 边长、站点位置 ``925..1125 / -100..100 / 505..705``、参考姿态
``rpy 3 73 -167``、``xyz-intrinsic``。这一页按用户要求**不显示关节量**，所以
这里断言的也是“坐标 + 可达性 + 没有 IK 展示”：

1. 算得完（主线程，实测 10 ms 量级）、``run_state=done``；
2. 五点全可达、路点表 5 行、统计行是 ``可达 5/5``（不是 ``IK 成功 5/5``）；
3. 表头**没有** ``q1`` 之类的关节列；
4. 导出的 ``iso.json`` 里每点是 X/Y/Z + 参考姿态，**没有** ``q`` / ``waypoints``；
5. 线程安全沿用 S3/S4 的判据：``sys.excepthook`` / ``threading.excepthook``
   抓到的异常必须为空（这一页现在连线程都不起）。
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
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import iso_page
from laser_desktop.qt_compat import QtWidgets
from laser_web import core, jobs

#: 算五点的上限（秒）：本机实测 0.01 s，慢机器也远用不到这么多。
TIMEOUT = 60.0

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-iso-run-tests"])
    return _APP


def configure_small_cube(page, bounds=None) -> None:
    """把表单改成 200 mm 立方体（默认站点位置，5 点全可解；不填种子）。"""
    fields = page.fields
    bounds = bounds or (925.0, 1125.0, -100.0, 100.0, 505.0, 705.0)
    for key, value in zip(("x0", "x1", "y0", "y1", "z0", "z1"), bounds):
        fields[key].setValue(value)
    fields["rpy_a"].setValue(3.0)
    fields["rpy_b"].setValue(73.0)
    fields["rpy_c"].setValue(-167.0)
    fields["euler"].setCurrentIndex(fields["euler"].findData("xyz-intrinsic"))


class RealIsoRunTest(unittest.TestCase):
    """从「生成五点」到结果表、图、产物的全链路（没有任务队列）。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="iso-run-"))
        self._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = str(self.root)
        self.page = iso_page.IsoPage(plot=None)
        self.errors = []
        self._previous_excepthook = sys.excepthook
        self._previous_thread_hook = getattr(threading, "excepthook", None)
        sys.excepthook = lambda *info: self.errors.append(info)
        threading.excepthook = lambda info: self.errors.append(info)

    def tearDown(self):
        sys.excepthook = self._previous_excepthook
        if self._previous_thread_hook is not None:
            threading.excepthook = self._previous_thread_hook
        if self._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = self._previous_runs
        self.page.shutdown()
        self.page.deleteLater()
        shutil.rmtree(self.root, ignore_errors=True)

    def test_real_run_produces_coordinates_and_reachability(self):
        configure_small_cube(self.page)
        preview = self.page.refresh_preview()
        self.assertTrue(preview["ok"], preview.get("error"))
        self.assertEqual(preview["side"], 200.0)
        self.assertTrue(self.page.generate_button.isEnabled())

        started = time.monotonic()
        result = self.page.start_iso()
        elapsed = time.monotonic() - started
        print(f"  [ISO 真算] {elapsed:.3f} s（上限 {TIMEOUT:.0f} s）")
        self.assertIsNotNone(result, "应当算出结果")
        self.assertLess(elapsed, TIMEOUT)
        # 主线程算完，不需要任务机制：实测 10 ms 量级（远低于 300 ms 的门槛）
        self.assertLess(elapsed, 0.3, "点击到出结果应当在 300 ms 内")
        self.assertEqual(self.page.run_state, "done")

        summary = result["summary"]
        self.assertEqual(summary, {"reachable": 5, "total": 5})
        for row in result["points"]:
            self.assertTrue(row["reachable"], row["name"])
            self.assertEqual(set(row), {"name", "pos", "reachable"},
                             "结果里不许出现关节量")

        # 结果区：5 行、可达 5/5、没有关节列
        table = self.page.point_table
        self.assertEqual(table.rowCount(), 5)
        headers = [table.horizontalHeaderItem(index).text()
                   for index in range(table.columnCount())]
        self.assertEqual(headers, list(iso_page.POINT_HEADERS))
        self.assertEqual(headers[-1], "可达")
        for header in headers:
            self.assertNotIn("q1", header)
        self.assertEqual(self.page.stat_labels["reach"].text(), "可达 5/5")
        self.assertNotIn("IK 成功", self.page.stat_labels["reach"].text())

        # 图：立方体 + 可达点（trace 名带点数，不含关节 hover）
        names = [str(trace.name) for trace in self.page.last_figure.data]
        self.assertTrue(any(name.startswith("可达 5/5") for name in names), names)

        # 导出：物化一次，iso.json 里只有坐标 + 参考姿态
        result = self.page.start_iso()
        self.assertIsNotNone(result)
        target = self.root / "iso-out.json"
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=(str(target), "")):
            self.page._on_export("iso_json")
        self.assertTrue(target.is_file(), self.page.status_label.text())
        payload = json.loads(target.read_text(encoding="utf-8"))
        self.assertEqual(len(payload["points"]), 5)
        self.assertEqual(payload["meta"]["point_count"], 5)
        self.assertEqual(payload["meta"]["rpy"], [3.0, 73.0, -167.0])
        for row in payload["points"]:
            self.assertEqual(row["rpy"], [3.0, 73.0, -167.0])
            self.assertNotIn("q", row)
        self.assertNotIn("waypoints", payload)
        self.assertNotIn("perturbed_ik", payload)

        # 线程安全：没有异常逃逸（这一页连线程都不起）
        self.assertEqual(self.errors, [])

    def test_numbering_is_p2p3_small_p4p5_large(self):
        """编号回归：P2/P3 在小 X 小 Z 一侧，P4/P5 在大 X 大 Z 一侧。

        真算一次 200 mm 立方体（中心 ``1000 0 600``），断言结果里的坐标就是
        ISO 9283 的编号——这条是给“编号被悄悄翻转”准备的（旧实现把两侧对调过）。
        """
        configure_small_cube(self.page,
                             bounds=(900.0, 1100.0, -100.0, 100.0,
                                     500.0, 700.0))
        preview = self.page.refresh_preview()
        self.assertTrue(preview["ok"], preview.get("error"))
        self.assertEqual(preview["center"], [1000.0, 0.0, 600.0])
        self.assertEqual(preview["side"], 200.0)

        result = self.page.start_iso()
        self.assertIsNotNone(result)
        points = {row["name"]: row["pos"] for row in result["points"]}
        center = (1000.0, 600.0)
        for name in ("P2", "P3"):
            self.assertLess(points[name][0], center[0], name)
            self.assertLess(points[name][2], center[1], name)
        for name in ("P4", "P5"):
            self.assertGreater(points[name][0], center[0], name)
            self.assertGreater(points[name][2], center[1], name)
        # 表里看到的也是同一份坐标（不是只对了返回的 dict）
        table = self.page.point_table
        self.assertEqual(table.rowCount(), 5)
        rows = {table.item(row, 0).text(): table.item(row, 1).text()
                for row in range(table.rowCount())}
        self.assertEqual(rows["P2"], "900.0")
        self.assertEqual(rows["P4"], "1100.0")

    def test_validation_error_is_shown_without_computing(self):
        """下界 > 上界：不算，报错原文红字写在卡片和状态行。"""
        self.page.fields["x0"].setValue(5000.0)
        self.assertIsNone(self.page.start_iso())
        self.assertIn("下界必须小于上界", self.page.error_label.text())
        self.assertIn("参数不合法", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertIsNone(self.page.last_result)


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
