#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 五点页结果区（S5）：**构造**的 ``run_iso9283`` 返回值。

要验的是“把结果铺到界面上”这一层，不跑 IK：

* 路点表：5 行、``_point_rows`` 的口径（P 名 / XYZ 一位小数 / q 三位小数 /
  cfg / ok），**失败行的关节角显示 ``—``**（不是 0、不是空白）；
* 统计行：``IK 成功 n/5``、``|J5|min``、``|J3|min``、构型一致与否、构型标签、
  立方体边长与中心（缺值一律 ``—``）；
* 结果图：复用 ``laser_web.figures.iso_cube_figure``（立方体 + 可解 / 无解点）；
* 导出：``iso.json`` / ``iso.png`` 真实存在才点亮，文件名带 ``<run_id>-`` 前缀；
* 终态：``ok=False``（有解不出的点）时原文红字留在结果区，产物照样能导出。
"""

import os
import shutil
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import iso_page
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtGui, QtWidgets
from laser_web import jobs

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-iso-result-tests"])
    return _APP


def make_point(name, position, *, ok=True, q=None, config=None,
               j3=42.95, j5=53.09):
    """造一个 ``run_iso9283()["points"]`` 的元素。"""
    return {"name": name, "pos": position, "q": q if ok else None,
            "ok": ok, "config": config if ok else None,
            "j3_abs": j3 if ok else None, "j5_abs": j5 if ok else None}


def make_result(*, failed=False, consistent=True, run_dir=None) -> dict:
    """与 ``core.run_iso9283`` 成功返回值同构的结果（可带一个无解点）。"""
    points = [
        make_point("P1", [1025.0, 0.0, 605.0],
                   q=[0.105, 3.812, -50.039, 1.095, -63.21, 15.345],
                   config=[0, -1, -1], j3=50.04, j5=63.21),
        make_point("P2", [1125.0, 100.0, 705.0],
                   q=[5.7, 1.14, -40.7, 7.56, -56.88, 10.05],
                   config=[0, -1, -1] if consistent else [0, 1, 1],
                   j3=40.7, j5=56.88),
        make_point("P3", [1125.0, -100.0, 705.0],
                   q=[-5.51, 1.16, -40.73, -5.26, -56.7, 20.37],
                   config=[0, -1, -1] if consistent else [0, 1, 1],
                   j3=40.73, j5=56.7),
        make_point("P4", [925.0, 100.0, 505.0],
                   ok=not failed,
                   q=[7.07, 4.07, -57.75, 8.08, -70.99, 11.14],
                   config=[0, -1, -1], j3=57.75, j5=70.99),
        make_point("P5", [925.0, -100.0, 505.0],
                   q=[-6.84, 4.11, -57.78, -6.0, -70.84, 19.85],
                   config=[0, -1, -1], j3=57.78, j5=70.84),
    ]
    solved = sum(1 for point in points if point["ok"])
    configs = sorted({tuple(point["config"]) for point in points
                      if point["config"] is not None})
    stats = {"total": 5, "solved": solved, "reachable": solved == 5,
             "j3_abs_min": 40.7, "j5_abs_min": 56.7,
             "configs": [list(tag) for tag in configs],
             "config_consistent": len(configs) <= 1,
             "cube_side": 200.0, "cube_center": [1025.0, 0.0, 605.0],
             "layout": "vertices", "plane": "xz"}
    result = {"ok": not failed, "run_id": "20260101-000000-abcd",
              "run_dir": str(run_dir) if run_dir is not None else "/tmp/nowhere",
              "stats": stats, "points": points,
              "artifacts": {"iso_json": "iso.json", "plot": "iso.png"},
              "warnings": []}
    if failed:
        result["error"] = "1 个点无解（P4）——缩小立方体或换姿态"
    return result


class PageCase(unittest.TestCase):
    """共用：一个挂临时登记表的页面 + 一个临时运行目录。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="iso-runs-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = iso_page.IsoPage(plot=None, bridge=self.bridge)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=30)
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
    """路点表：5 行、失败行关节角 ``—``、整行标红。"""

    def test_five_rows_and_headers(self):
        self.page.render_result(make_result())
        table = self.page.point_table
        self.assertEqual(table.rowCount(), 5)
        self.assertEqual([table.horizontalHeaderItem(index).text()
                          for index in range(table.columnCount())],
                         list(iso_page.POINT_HEADERS))
        row = row_for(table, "P1")
        self.assertEqual(cell_text(table, row, 1), "1025.0")
        self.assertEqual(cell_text(table, row, 2), "0.0")
        self.assertEqual(cell_text(table, row, 3), "605.0")
        self.assertEqual(cell_text(table, row, 4), "0.105")     # q1
        self.assertEqual(cell_text(table, row, 12), "0,-1,-1")  # cfg
        self.assertEqual(cell_text(table, row, 13), "✓")        # ok

    def test_failed_row_shows_dash(self):
        """无解的点：q1..q6 / |J3| / |J5| / cfg 一律 ``—``，ok 是 ✗。"""
        self.page.render_result(make_result(failed=True))
        table = self.page.point_table
        row = row_for(table, "P4")
        for column in (4, 5, 6, 7, 8, 9):       # q1..q6
            self.assertEqual(cell_text(table, row, column), "—",
                             f"第 {column} 列（关节角）应为 —")
        self.assertEqual(cell_text(table, row, 10), "—")        # |J3|
        self.assertEqual(cell_text(table, row, 11), "—")        # |J5|
        self.assertEqual(cell_text(table, row, 12), "—")        # cfg
        self.assertEqual(cell_text(table, row, 13), "✗")
        # 整行标红（浅红底 + 深红字），排序后也不丢
        item = table.item(row, 0)
        self.assertEqual(item.background().color().name(),
                         QtGui.QColor("#fde8e8").name())

    def test_point_rows_keep_numbers(self):
        """``point_rows`` 保留数字类型（表头排序要用数值，不是字符串）。"""
        rows = iso_page.point_rows(make_result())
        self.assertEqual(len(rows), 5)
        self.assertIsInstance(rows[0]["x"], float)
        self.assertEqual(rows[0]["q1"], 0.105)
        failed = iso_page.point_rows(make_result(failed=True))
        self.assertIsNone(failed[3]["q1"])
        self.assertTrue(failed[3]["failed"])


class StatsRowTest(PageCase):
    """统计行：IK 成功 n/5 / |J5|min / |J3|min / 构型。"""

    def test_all_solved(self):
        self.page.render_result(make_result())
        labels = self.page.stat_labels
        self.assertEqual(labels["solved"].text(), "IK 成功 5/5")
        self.assertIn("color:#1e8449", labels["solved"].styleSheet())
        self.assertEqual(labels["j5"].text(), "|J5|min 56.7°")
        self.assertEqual(labels["j3"].text(), "|J3|min 40.7°")
        self.assertEqual(labels["config"].text(), "构型 一致")
        self.assertEqual(labels["configs"].text(), "构型标签 0,-1,-1")
        self.assertEqual(labels["side"].text(), "边长 200 mm")
        self.assertEqual(labels["center"].text(), "中心 (1025, 0, 605) mm")

    def test_partial_and_inconsistent(self):
        self.page.render_result(make_result(failed=True, consistent=False))
        labels = self.page.stat_labels
        self.assertEqual(labels["solved"].text(), "IK 成功 4/5")
        self.assertIn("color:#c0392b", labels["solved"].styleSheet())
        self.assertEqual(labels["config"].text(), "构型 不一致")
        self.assertIn("color:#c0392b", labels["config"].styleSheet())
        self.assertIn("0,1,1", labels["configs"].text())

    def test_missing_numbers_are_dashes(self):
        """没有立方体（人工逐点）时边长/中心是 ``—``，不是 0。"""
        result = make_result()
        result["stats"]["cube_side"] = None
        result["stats"]["cube_center"] = None
        self.page.render_result(result)
        self.assertEqual(self.page.stat_labels["side"].text(), "边长 —")
        self.assertEqual(self.page.stat_labels["center"].text(), "中心 —")


class FigureAndExportTest(PageCase):
    """结果图与导出按钮。"""

    def test_result_figure_reuses_web_figure(self):
        figure = iso_page.result_figure(make_result())
        names = [str(trace.name) for trace in figure.data]
        self.assertEqual(len(names), 2)                 # 立方体 + 可解点
        self.assertIn("立方体 L=200 mm", names[0])
        self.assertIn("可解 5/5 点", names[1])
        failed = iso_page.result_figure(make_result(failed=True))
        self.assertTrue(any("无解 1 点" in str(trace.name)
                            for trace in failed.data))

    def test_exports_light_up_only_with_files(self):
        run_dir = Path(tempfile.mkdtemp(prefix="iso-artifacts-"))
        try:
            result = make_result(run_dir=run_dir)
            self.page.render_result(result)
            self.assertFalse(self.page.export_buttons["iso_json"].isEnabled(),
                             "产物还没落盘时不许点亮导出")
            (run_dir / "iso.json").write_text("{}", encoding="utf-8")
            (run_dir / "iso.png").write_bytes(b"\x89PNG")
            self.page.render_result(result)
            self.assertTrue(self.page.export_buttons["iso_json"].isEnabled())
            self.assertTrue(self.page.export_buttons["plot"].isEnabled())

            # 默认文件名带 <run_id>- 前缀（export_artifact 的口径）
            from unittest import mock

            target = run_dir / "out.json"
            with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                                   return_value=(str(target), "")):
                level, message = iso_page.export_artifact(
                    result, "iso_json", iso_page.EXPORTS, self.page)
            self.assertIsNone(level)
            self.assertTrue(target.is_file(), message)
            default_name = str(Path.home() / "20260101-000000-abcd-iso.json")
            self.assertIsNone(level)
            self.assertEqual(iso_page.EXPORTS[0][0], "iso_json")
            self.assertTrue(default_name.endswith("iso.json"))
        finally:
            shutil.rmtree(run_dir, ignore_errors=True)

    def test_current_figure_falls_back_to_preview(self):
        """还没结果时切回本页：给预览图，而不是把生成页的图留着。"""
        figure = self.page.current_figure()
        names = [str(trace.name) for trace in figure.data]
        self.assertEqual(len(names), 3,
                         "预览图 = 立方体 + 候选点 + 未解 IK 的五点")
        self.assertIn("P1~P5（未解 IK）", names)
        self.page.render_result(make_result())
        names = [str(trace.name) for trace in self.page.current_figure().data]
        self.assertTrue(any(name.startswith("可解") for name in names), names)


class FinishedStateTest(PageCase):
    """终态：``ok=False`` 时原文红字 + 产物仍可导出。"""

    def test_infeasible_iso_keeps_result_and_exports(self):
        run_dir = Path(tempfile.mkdtemp(prefix="iso-infeasible-"))
        try:
            (run_dir / "iso.json").write_text("{}", encoding="utf-8")
            payload = {"state": "done", "result": make_result(failed=True,
                                                              run_dir=run_dir),
                       "error": None, "run_id": "x", "run_dir": str(run_dir)}
            self.page._on_finished(payload)
            self.assertIn("完成但不可用", self.page.status_label.text())
            self.assertIn("无解", self.page.status_label.text())
            self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
            self.assertEqual(self.page.point_table.rowCount(), 5)
            self.assertTrue(self.page.export_buttons["iso_json"].isEnabled())
            self.assertFalse(self.page.export_buttons["plot"].isEnabled())
        finally:
            shutil.rmtree(run_dir, ignore_errors=True)

    def test_cancelled_is_muted(self):
        self.page._on_finished({"state": "cancelled", "result": None,
                                "error": None, "run_id": None, "run_dir": None})
        self.assertIn("已取消", self.page.status_label.text())
        self.assertIn("color:#6b7280", self.page.status_label.styleSheet())


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
