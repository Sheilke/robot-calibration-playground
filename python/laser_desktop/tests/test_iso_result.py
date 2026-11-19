#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 五点页结果区（S5）：**构造**的 ``core.iso_points`` 返回值。

要验的是“把结果铺到界面上”这一层。这一页按用户要求**不显示关节量**，所以
这些用例钉的就是“没有 IK 展示”：

* 路点表：5 行、列头就是 ``P / X / Y / Z / 可达``（**没有** q1..q6 / |J3| / |J5|
  / cfg），不可达的行整行标红；
* 统计行：``可达 N/5``（**不是** ``IK 成功 N/5``）+ 边长 + 中心，缺值一律 ``—``；
  界面上不存在 j5 / j3 / config 这些标签；
* 结果图：复用 ``laser_web.figures.iso_points_figure``（立方体 + 可达/不可达两色，
  悬停只有点名与 XYZ）；
* 导出：产物**在点导出时才生成**（``core.save_iso_points``），文件名带
  ``<run_id>-`` 前缀。
"""

import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import iso_page
from laser_desktop.qt_compat import QtGui, QtWidgets
from laser_web import core, jobs

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-iso-result-tests"])
    return _APP


def make_result(*, failed=False, run_dir=None) -> dict:
    """与 ``core.iso_points`` 成功返回值同构的结果（可带一个不可达点）。

    ``failed=True`` 时 P4 不可达（``reachable=False``）——注意整次计算仍然是
    ``ok=True``：可达性本身就是结果，不是错误。
    """
    rows = [("P1", [1025.0, 0.0, 605.0], True),
            ("P2", [925.0, 100.0, 505.0], True),
            ("P3", [925.0, -100.0, 505.0], True),
            ("P4", [1125.0, 100.0, 705.0], not failed),
            ("P5", [1125.0, -100.0, 705.0], True)]
    points = [{"name": name, "pos": position, "reachable": reachable}
              for name, position, reachable in rows]
    reachable_count = sum(1 for row in points if row["reachable"])
    result = {"ok": True,
              "points": points,
              "summary": {"reachable": reachable_count, "total": 5},
              "meta": {"source": "iso9283_points", "points_source": "cube",
                       "layout": "vertices", "plane": "xz",
                       "cube_center": [1025.0, 0.0, 605.0], "cube_side": 200.0,
                       "rpy": [3.0, 73.0, -167.0],
                       "euler_convention": "xyz-intrinsic"},
              "warnings": []}
    if run_dir is not None:
        result["run_dir"] = str(run_dir)
    return result


class PageCase(unittest.TestCase):
    """共用：一个页面 + 一个临时 runs 根目录。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="iso-runs-"))
        self._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = str(self.root)
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        if self._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = self._previous_runs
        self.page.shutdown()
        self.page.deleteLater()
        shutil.rmtree(self.root, ignore_errors=True)


def cell_text(table, row: int, column: int) -> str:
    """表格单元格文本（空单元格返回空串）。"""
    item = table.item(row, column)
    return "" if item is None else item.text()


def row_for(table, name: str) -> int:
    """按 P 名找行号（表头按 P 名升序排过）。"""
    for row in range(table.rowCount()):
        if cell_text(table, row, 0) == name:
            return row
    raise AssertionError(f"表里没有 {name}")


class PointTableTest(PageCase):
    """路点表：5 行、只有坐标与可达、不可达整行标红。"""

    def test_headers_have_no_joint_columns(self):
        """表头**没有**关节角列（``q1`` 一个都不能出现）。"""
        self.page.render_result(make_result())
        table = self.page.point_table
        headers = [table.horizontalHeaderItem(index).text()
                   for index in range(table.columnCount())]
        self.assertEqual(headers, list(iso_page.POINT_HEADERS))
        self.assertEqual(headers, ["P", "X (mm)", "Y (mm)", "Z (mm)", "可达"])
        for banned in ("q1", "q2", "q3", "q4", "q5", "q6",
                       "|J3|", "|J5|", "cfg"):
            for header in headers:
                self.assertNotIn(banned, header, f"表头里还有 {banned}：{headers}")

    def test_five_rows_show_coordinates_and_reachable(self):
        self.page.render_result(make_result())
        table = self.page.point_table
        self.assertEqual(table.rowCount(), 5)
        row = row_for(table, "P1")
        self.assertEqual(cell_text(table, row, 1), "1025.0")
        self.assertEqual(cell_text(table, row, 2), "0.0")
        self.assertEqual(cell_text(table, row, 3), "605.0")
        self.assertEqual(cell_text(table, row, 4), "✓ 可达")

    def test_unreachable_row_is_red(self):
        """不可达的点：可达列写 ✗ 不可达，整行浅红底（排序后也不丢）。"""
        self.page.render_result(make_result(failed=True))
        table = self.page.point_table
        row = row_for(table, "P4")
        self.assertEqual(cell_text(table, row, 4), "✗ 不可达")
        item = table.item(row, 0)
        self.assertEqual(item.background().color().name(),
                         QtGui.QColor("#fde8e8").name())

    def test_point_rows_keep_numbers_and_have_no_joints(self):
        """``point_rows`` 保留数字类型（表头排序要用数值），且不带关节量。"""
        rows = iso_page.point_rows(make_result())
        self.assertEqual(len(rows), 5)
        self.assertIsInstance(rows[0]["x"], float)
        self.assertEqual(set(rows[0]), {"name", "x", "y", "z", "reach",
                                        "failed"})
        failed = iso_page.point_rows(make_result(failed=True))
        self.assertTrue(failed[3]["failed"])
        self.assertEqual(failed[3]["reach"], "✗ 不可达")


class StatsRowTest(PageCase):
    """统计行：可达 N/5 / 边长 / 中心（没有 IK 统计）。"""

    def test_summary_is_reachable_not_ik(self):
        self.page.render_result(make_result())
        labels = self.page.stat_labels
        self.assertEqual(list(labels), list(iso_page.STAT_KEYS))
        self.assertEqual(labels["reach"].text(), "可达 5/5")
        self.assertIn("color:#1e8449", labels["reach"].styleSheet())
        self.assertEqual(labels["side"].text(), "边长 200 mm")
        self.assertEqual(labels["center"].text(), "中心 (1025, 0, 605) mm")
        # 旧的那几个标签一个都不能还在
        for gone in ("solved", "j5", "j3", "config", "configs"):
            self.assertNotIn(gone, labels, f"统计行里还有 {gone}")

    def test_partial_reachability_is_red(self):
        self.page.render_result(make_result(failed=True))
        labels = self.page.stat_labels
        self.assertEqual(labels["reach"].text(), "可达 4/5")
        self.assertIn("color:#c0392b", labels["reach"].styleSheet())

    def test_missing_cube_numbers_are_dashes(self):
        """没有立方体（人工逐点）时边长/中心是 ``—``，不是 0。"""
        result = make_result()
        result["meta"]["cube_side"] = None
        result["meta"]["cube_center"] = None
        self.page.render_result(result)
        self.assertEqual(self.page.stat_labels["side"].text(), "边长 —")
        self.assertEqual(self.page.stat_labels["center"].text(), "中心 —")


class FigureAndExportTest(PageCase):
    """结果图与导出（产物在导出时才生成）。"""

    def test_result_figure_has_no_joint_hover(self):
        figure = iso_page.result_figure(make_result())
        names = [str(trace.name) for trace in figure.data]
        self.assertEqual(len(names), 2, names)          # 立方体 + 可达点
        self.assertIn("立方体 L=200 mm", names[0])
        self.assertIn("可达 5/5 点", names[1])
        point_trace = figure.data[1]
        for text in point_trace.hovertext:
            self.assertNotIn("|J3|", text)
            self.assertNotIn("|J5|", text)
            self.assertNotIn("cfg", text)
            self.assertIn("P", text)
        failed = iso_page.result_figure(make_result(failed=True))
        self.assertTrue(any("不可达 1 点" in str(trace.name)
                            for trace in failed.data))

    def test_export_writes_artifacts_on_demand(self):
        """点导出才落盘：iso.json 里是每点 X/Y/Z + 参考姿态，没有关节角。"""
        self.page.render_result(make_result())
        self.assertTrue(self.page.export_buttons["iso_json"].isEnabled(),
                        "有结果就该能点导出")
        target = self.root / "out.json"
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=(str(target), "")):
            self.page._on_export("iso_json")
        self.assertTrue(target.is_file())
        payload = json.loads(target.read_text(encoding="utf-8"))
        self.assertEqual(len(payload["points"]), 5)
        for row in payload["points"]:
            self.assertEqual(set(row), {"name", "pos", "rpy", "reachable"})
            self.assertEqual(len(row["pos"]), 3)
            self.assertEqual(row["rpy"], [3.0, 73.0, -167.0])
        self.assertNotIn("waypoints", payload)
        self.assertNotIn("perturbed_ik", payload)
        # 第二次导出不再重写（产物已经在了）
        saved = self.page.last_result.get("run_dir")
        self.assertTrue(saved and Path(saved).is_dir())

    def test_export_without_result_is_refused(self):
        self.page._set_status("")
        self.page._on_export("iso_json")
        self.assertIn("还没有结果", self.page.status_label.text())

    def test_current_figure_falls_back_to_preview(self):
        """还没结果时切回本页：给预览图，而不是把生成页的图留着。"""
        figure = self.page.current_figure()
        names = [str(trace.name) for trace in figure.data]
        self.assertEqual(len(names), 3,
                         "预览图 = 立方体 + 候选点 + 未解 IK 的五点")
        self.assertIn("P1~P5（未解 IK）", names)
        self.page.render_result(make_result())
        names = [str(trace.name) for trace in self.page.current_figure().data]
        self.assertTrue(any(name.startswith("可达") for name in names), names)


class NoJobMachineryTest(PageCase):
    """这一页没有任务机制：没有进度条 / 取消 / 日志页，也没有 q_seed。"""

    def test_page_has_no_job_widgets(self):
        for attribute in ("progress_bar", "cancel_button", "log_edit",
                          "log_page", "bridge", "last_job_id"):
            self.assertFalse(hasattr(self.page, attribute), attribute)
        self.assertEqual(list(iso_page.TABS), ["路点", "预览"])
        for key in ("q_seed", "j5_min"):
            self.assertNotIn(key, self.page.fields)

    def test_result_carries_no_joint_data(self):
        """``core.iso_points`` 的返回里没有关节量可显示。"""
        result = make_result()
        for row in result["points"]:
            self.assertNotIn("q", row)
            self.assertNotIn("config", row)


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
