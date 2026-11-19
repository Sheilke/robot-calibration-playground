#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""结果区渲染与导出（S3）：**构造**的 ``run_scan`` 返回值，不跑 IK。

要验的是"把结果铺到界面上"这一层：

* 路点表行数 = 路点数、列头齐全、失败行**整行**红底（排序之后也不丢）；
* 段诊断表行数 = 段数、穿零标红；
* 统计标签上的数字与 ``stats`` 一致（成功 n/N、切换数、ΣdqL2、两个最小值、
  条件数、可行绿/红）；
* 结果图里失败点是单独一条红色 trace（复用网页版的 ``figures.waypoint_figure``）；
* 导出按钮：``QFileDialog`` 选完目标后**复制**，内容与源逐字节相同。
"""

import os
import shutil
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import generate_page, page_layout
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtGui, QtWidgets
from laser_web import jobs

#: 等信号的上限（秒）。
TIMEOUT = 30.0

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-result-tests"])
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


def make_result(run_dir: Path, *, feasible=True, artifacts=True) -> dict:
    """造一份与 ``core.run_scan`` 同构的返回值（字段逐个对齐）。"""
    waypoints = [
        {"gp": 1, "index": 0, "pos": [900.0, -100.0, 500.0],
         "rpy_pert": [0.0, 0.0, 0.0], "q": [10.0, -20.0, 30.0, 40.0, 50.0, 60.0],
         "j3_abs": 30.0, "j5_abs": 50.0, "config": [0, 1, 0],
         "dq_l2": 0.0, "ok": True},
        {"gp": 2, "index": 1, "pos": [1100.0, -100.0, 500.0],
         "rpy_pert": [1.0, 2.0, 3.0], "q": [11.5, -21.5, 31.5, 41.5, 51.5, 61.5],
         "j3_abs": 31.5, "j5_abs": 51.5, "config": [-1, 1, 0],
         "dq_l2": 12.25, "ok": True},
        # 这一行是"无解"：q / config / |J3| / |J5| 全为 None，要整行标红
        {"gp": 3, "index": 2, "pos": [1100.0, 100.0, 500.0],
         "rpy_pert": None, "q": None, "j3_abs": None, "j5_abs": None,
         "config": None, "dq_l2": None, "ok": False},
        {"gp": 4, "index": 3, "pos": [900.0, 100.0, 700.0],
         "rpy_pert": [2.0, 0.0, -1.0], "q": [12.0, -22.0, 32.0, 42.0, 52.0, 62.0],
         "j3_abs": 32.0, "j5_abs": 52.0, "config": [0, 1, 0],
         "dq_l2": 5.5, "ok": True},
    ]
    segments = [
        {"gp_from": 1, "gp_to": 2, "j3_abs_min": 30.0, "j5_abs_min": 50.0,
         "crosses": False, "worst_frac": 0.5,
         "worst_pos": [1000.0, -100.0, 500.0]},
        {"gp_from": 2, "gp_to": 3, "j3_abs_min": 0.5, "j5_abs_min": 4.0,
         "crosses": True, "worst_frac": 0.25,
         "worst_pos": [1100.0, 0.0, 500.0]},
        {"gp_from": 3, "gp_to": 4, "j3_abs_min": None, "j5_abs_min": None,
         "crosses": False, "worst_frac": None, "worst_pos": None},
    ]
    stats = {"solved": 3, "total": 4, "switches": 2, "dq_l2_sum": 1234.5678,
             "j5_abs_min": 4.25, "j3_abs_min": 0.5, "design_cond": 14.36525,
             "post_ik_cond": 18.477054, "seg_j5_min": 4.0, "seg_j3_min": 0.5,
             "j3_crossings": 1, "j5_crossings": 0, "guard_violations": 0,
             "feasible": feasible}
    result = {"ok": True, "run_id": "20260101-000000-abcd",
              "run_dir": str(run_dir), "stats": stats,
              "meta": {"skip_ik": False, "kind": "scan"},
              "waypoints": waypoints, "segments": segments,
              "warnings": ["1 个路点无解（未通过 FK 自校验）"],
              "artifacts": {}}
    if artifacts:
        names = {"scan_json": "scan.json", "joints": "joints.txt",
                 "plot": "plot.png"}
        for key, name in names.items():
            (run_dir / name).write_text(f"{key} 的内容\n", encoding="utf-8")
        result["artifacts"] = dict(names)
    return result


class ResultRenderTest(unittest.TestCase):
    """``render_result`` 把统计/表格/图铺好（不碰后台任务）。"""

    def setUp(self):
        app()
        self.run_dir = Path(tempfile.mkdtemp(prefix="result-"))
        # 桥也绑临时 runs/：默认桥会在仓库根建 ``runs/``（评审第 7 条）
        self.root = Path(tempfile.mkdtemp(prefix="result-runs-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.plot = StubPlot()
        self.page = generate_page.GeneratePage(plot=self.plot,
                                               bridge=self.bridge)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.plot.deleteLater()
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.run_dir, ignore_errors=True)
        shutil.rmtree(self.root, ignore_errors=True)

    def test_waypoint_table_has_one_row_per_waypoint(self):
        result = make_result(self.run_dir)
        self.page.render_result(result)
        table = self.page.waypoint_table
        self.assertEqual(table.rowCount(), len(result["waypoints"]))
        self.assertEqual(table.columnCount(), len(generate_page.WAYPOINT_HEADERS))
        headers = [table.horizontalHeaderItem(index).text()
                   for index in range(table.columnCount())]
        self.assertEqual(tuple(headers), generate_page.WAYPOINT_HEADERS)
        self.assertTrue(table.isSortingEnabled())
        # GP 默认升序：第一行 GP 1，失败行是 GP 3
        self.assertEqual(table.item(0, 0).text(), "1")
        self.assertEqual(table.item(2, 0).text(), "3")

    def test_failed_row_is_red_across_all_columns(self):
        self.page.render_result(make_result(self.run_dir))
        table = self.page.waypoint_table
        red = QtGui.QColor(generate_page._FAILED_BG).name()
        for column in range(table.columnCount()):
            item = table.item(2, column)          # GP 3 是无解行
            self.assertEqual(item.background().color().name(), red,
                             f"第 {column} 列没标红")
        for column in range(table.columnCount()):  # 成功行不标红
            item = table.item(0, column)
            self.assertNotEqual(item.background().color().name(), red)

    def test_failed_row_stays_red_after_sorting(self):
        """可排序（规格要求）——排完序红色必须跟着行走，不是留在行号上。"""
        self.page.render_result(make_result(self.run_dir))
        table = self.page.waypoint_table
        red = QtGui.QColor(generate_page._FAILED_BG).name()
        table.sortItems(10, generate_page.Qt.DescendingOrder)   # 按 |J3| 降序
        failed_rows = [row for row in range(table.rowCount())
                       if table.item(row, 10).text() == "—"]
        self.assertEqual(failed_rows, [0], "无解行（|J3| = —）应该排在最末")
        for column in range(table.columnCount()):
            self.assertEqual(table.item(failed_rows[0], column)
                             .background().color().name(), red)

    def test_stats_labels_match_stats(self):
        result = make_result(self.run_dir)
        stats = result["stats"]
        self.page.render_result(result)
        labels = self.page.stat_labels
        self.assertEqual(set(labels), set(generate_page.STAT_KEYS))
        self.assertEqual(labels["solved"].text(), "成功 3/4")
        self.assertIn("color:#c0392b", labels["solved"].styleSheet())  # 不满格 → 红
        self.assertEqual(labels["switches"].text(),
                         f"构型切换 {stats['switches']}")
        self.assertEqual(labels["dq_l2"].text(),
                         f"ΣdqL2 {stats['dq_l2_sum']:.2f}")
        self.assertEqual(labels["j5_min"].text(),
                         f"|J5|min {stats['j5_abs_min']:.1f}°")
        self.assertEqual(labels["j3_min"].text(),
                         f"|J3|min {stats['j3_abs_min']:.1f}°")
        self.assertEqual(labels["cond"].text(),
                         f"design {stats['design_cond']:.3f}"
                         f" / post_ik {stats['post_ik_cond']:.3f}")
        self.assertEqual(labels["feasible"].text(), "可行")

    def test_feasible_label_turns_red_when_infeasible(self):
        """不可行时判词要带**原因**：光写"不可行"会和"成功 n/n"自相矛盾。

        ``make_result(feasible=False)`` 里有一个点没解出来，所以原因应当是
        "有点无解"——即使参数再漂亮，先报"根本解不出来"才是现场该看到的那句。
        """
        self.page.render_result(make_result(self.run_dir, feasible=False))
        label = self.page.stat_labels["feasible"]
        self.assertEqual(label.text(), "不可行·有点无解")
        self.assertIn("color:#c0392b", label.styleSheet())

    def test_verdict_names_the_actual_reason(self):
        """四种不可行的原因各自成词，别都笼统写成"不可行"。"""
        cases = [
            ({"solved": 3, "total": 4}, "不可行·有点无解"),
            ({"guard_violations": 2}, "不可行·奇异裕度不足"),
            ({"j3_crossings": 1}, "不可行·段内奇异"),
            ({"design_cond": 14.4}, "不可行·激励不足"),
        ]
        for stats, expected in cases:
            merged = {"solved": 4, "total": 4, "feasible": False}
            merged.update(stats)
            self.assertEqual(generate_page.feasible_text(merged), expected, merged)

    def test_reason_is_absent_when_feasible(self):
        self.assertEqual(
            generate_page.feasible_text({"solved": 4, "total": 4,
                                         "design_cond": 4.0, "feasible": True}),
            "可行")
        self.assertEqual(generate_page.verdict_reason({"solved": 4, "total": 4,
                                                      "design_cond": 4.0}), "")

    def test_all_solved_turns_the_counter_green(self):
        result = make_result(self.run_dir)
        result["stats"]["solved"] = result["stats"]["total"]
        self.page.render_result(result)
        label = self.page.stat_labels["solved"]
        self.assertEqual(label.text(), "成功 4/4")
        self.assertIn("color:#1e8449", label.styleSheet())

    def test_segment_table_rows_and_crossing_highlight(self):
        result = make_result(self.run_dir)
        self.page.render_result(result)
        table = self.page.segment_table
        self.assertEqual(table.rowCount(), len(result["segments"]))
        self.assertEqual(table.columnCount(), len(generate_page.SEGMENT_HEADERS))
        red = QtGui.QColor(generate_page._FAILED_BG).name()
        # 默认按 |J3|min 降序：第一行是 30.0 那段（不穿零），穿零那段在中间
        crossing = [row for row in range(table.rowCount())
                    if table.item(row, 3).text() == "是"]
        self.assertEqual(len(crossing), 1)
        self.assertEqual(table.item(crossing[0], 0).background()
                         .color().name(), red)
        self.assertIn("→", table.item(crossing[0], 0).text())
        self.assertIn("t=", table.item(crossing[0], 4).text())

    def test_result_figure_has_a_red_bad_trace(self):
        result = make_result(self.run_dir)
        self.page.render_result(result)
        figure = self.page.last_figure
        self.assertIsNotNone(figure)
        names = [trace.name for trace in figure.data]
        self.assertTrue(any("无解" in name for name in names), names)
        self.assertTrue(any("成功" in name for name in names), names)
        self.assertEqual(self.plot.figures[-1], figure, "结果图没推给图控件")
        # hover 文案里有 GP 与裕度（网页版 figures.py 同一套）
        good = next(trace for trace in figure.data if "成功" in trace.name)
        self.assertIn("GP 1", good.hovertext[0])
        self.assertIn("|J3|", good.hovertext[0])

    def test_log_box_is_read_only_monospace_and_capped(self):
        """日志区：只读 + 等宽 + 最多 500 行（超出删头部，不是停住不写）。"""
        edit = self.page.log_edit
        self.assertTrue(edit.isReadOnly())
        self.assertTrue(edit.font().fixedPitch(), edit.font().family())
        self.assertEqual(edit.maximumBlockCount(), generate_page.LOG_WIDGET_LIMIT)
        total = generate_page.LOG_WIDGET_LIMIT + 60
        for index in range(total):
            self.page._on_log_line(f"第 {index} 行")
        self.assertEqual(edit.blockCount(), generate_page.LOG_WIDGET_LIMIT)
        self.assertEqual(edit.document().lastBlock().text(), f"第 {total - 1} 行")
        self.assertEqual(edit.document().firstBlock().text(),
                         f"第 {total - generate_page.LOG_WIDGET_LIMIT} 行")

    def test_infeasible_result_shows_the_reason_in_status(self):
        """成功但不可行：原因（core 拼好的 warnings）要写在结果区状态行。"""
        result = make_result(self.run_dir, feasible=False)
        result["warnings"] = ["姿态激励条件数未达标：门禁 14.365 > 阈值 9"]
        self.page._on_finished({"state": "done", "result": result})
        text = self.page.status_label.text()
        self.assertIn("完成", text)
        self.assertIn("不可行", text)
        self.assertIn("姿态激励条件数未达标：门禁 14.365 > 阈值 9", text)
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertFalse(self.page.status_label.isHidden())

    def test_feasible_result_status_has_no_error_colour(self):
        """可行时状态行仍是绿的完成，不背“不可行”锅。"""
        self.page._on_finished({"state": "done",
                                "result": make_result(self.run_dir)})
        text = self.page.status_label.text()
        self.assertIn("完成", text)
        self.assertNotIn("不可行", text)
        self.assertIn("color:#1e8449", self.page.status_label.styleSheet())

    def test_result_area_grows_so_the_table_is_visible(self):
        """出结果后结果区撑到 ≥ RESULT_MIN_HEIGHT 且能看 8 行，表回到第一行。

        S6 L1：右欄改成三段（预览卡 / 图 / 结果），所以这里看第 3 段。
        """
        page = self.page
        page.resize(1000, 1000)          # 给足高度：现场笔记本最大化就这个量级
        page.show()
        app().processEvents()
        before = page._viewer_splitter.sizes()
        self.assertEqual(len(before), 3)
        # S6c：预览卡按内容拿（150–220），图 ≥ 260——首开不再要求结果区 < 300
        self.assertGreaterEqual(before[0], page_layout.PREVIEW_DEFAULT_MIN,
                                before)
        self.assertGreaterEqual(before[1], page_layout.PLOT_MIN_HEIGHT, before)

        page.render_result(make_result(self.run_dir))
        app().processEvents()
        sizes = page._viewer_splitter.sizes()
        self.assertGreaterEqual(sizes[2], generate_page.RESULT_MIN_HEIGHT, sizes)
        self.assertGreaterEqual(sizes[1], 120, sizes)   # 图区不能压成一条缝
        self.assertEqual(page.waypoint_table.verticalScrollBar().value(), 0)
        page.hide()

    def test_log_caption_mentions_the_source_buffer_limit(self):
        """说明文字要对齐真实行为：源缓冲只留 LOG_LIMIT 行，不是 500。"""
        from laser_desktop import job_bridge

        captions = [label.text() for label in self.page.log_page.findChildren(
            QtWidgets.QLabel)]
        text = "\n".join(captions)
        self.assertIn(str(job_bridge.LOG_LIMIT), text)
        self.assertIn(str(generate_page.LOG_WIDGET_LIMIT), text)
        self.assertIn("已省略", text)

    def test_export_buttons_are_enabled_only_with_artifacts(self):
        empty = make_result(self.run_dir, artifacts=False)
        self.page.render_result(empty)
        self.assertFalse(any(button.isEnabled()
                             for button in self.page.export_buttons.values()))

        result = make_result(self.run_dir)
        self.page.render_result(result)
        self.assertTrue(all(button.isEnabled()
                            for button in self.page.export_buttons.values()))


class ExportTest(unittest.TestCase):
    """导出：``QFileDialog`` 选目标 → 复制（源文件必须还在）。"""

    def setUp(self):
        app()
        self.run_dir = Path(tempfile.mkdtemp(prefix="export-source-"))
        self.target_dir = Path(tempfile.mkdtemp(prefix="export-target-"))
        self.root = Path(tempfile.mkdtemp(prefix="export-runs-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = generate_page.GeneratePage(plot=None, bridge=self.bridge)
        self.page.render_result(make_result(self.run_dir))

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.run_dir, ignore_errors=True)
        shutil.rmtree(self.target_dir, ignore_errors=True)
        shutil.rmtree(self.root, ignore_errors=True)

    def _export(self, key: str, target: Path):
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=(str(target), "")):
            self.page._on_export(key)

    def test_each_artifact_is_copied_byte_for_byte(self):
        for key, name in (("scan_json", "scan.json"), ("joints", "joints.txt"),
                          ("plot", "plot.png")):
            target = self.target_dir / f"导出的-{name}"
            self._export(key, target)
            source = self.run_dir / name
            self.assertTrue(target.is_file(), f"{name} 没复制出来")
            self.assertEqual(target.read_bytes(), source.read_bytes())
            self.assertTrue(source.is_file(), f"{name} 被搬走了（runs/ 是归档）")
            self.assertIn("已导出", self.page.last_status)

    def test_cancelled_dialog_writes_nothing(self):
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=("", "")):
            self.page._on_export("scan_json")
        self.assertEqual(list(self.target_dir.iterdir()), [])
        self.assertNotIn("已导出", self.page.last_status)

    def test_missing_suffix_is_appended(self):
        target = self.target_dir / "没有扩展名"
        self._export("plot", target)
        self.assertTrue((self.target_dir / "没有扩展名.png").is_file())

    def test_export_without_artifacts_reports_status(self):
        page = generate_page.GeneratePage(plot=None, bridge=self.bridge)
        try:
            page._on_export("scan_json")          # 还没跑过任务
            self.assertIn("无法导出", page.last_status)
        finally:
            page.shutdown()
            page.deleteLater()

    def test_missing_source_file_reports_status(self):
        """源文件被 prune 掉：状态栏与结果区状态行说同一句话（评审第 4 条）。"""
        (self.run_dir / "scan.json").unlink()
        target = self.target_dir / "scan.json"
        self._export("scan_json", target)
        self.assertFalse(target.exists())
        self.assertIn("源文件不存在", self.page.last_status)
        self.assertIn("源文件不存在", self.page.status_label.text())
        self.assertIn("保留策略", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())

    def test_default_filename_carries_the_run_id(self):
        """默认文件名带运行号：连跑两次解算不会在 home 里互相覆盖。"""
        result = self.page.last_result
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=("", "")) as dialog:
            self.page._on_export("scan_json")
        default = Path(dialog.call_args[0][2]).name
        self.assertEqual(default, f"{result['run_id']}-scan.json")


class StubPlot(QtWidgets.QWidget):
    """只记下被推过来的图（真 PlotView 要 WebEngine）。"""

    def __init__(self):
        super().__init__()
        self.figures = []

    def set_figure(self, figure):
        self.figures.append(figure)


class StartScanGuardTest(unittest.TestCase):
    """起任务的守门：参数不合法 / 点数超上限 / 已有任务 → 一律不起任务。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="guard-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = generate_page.GeneratePage(plot=None, bridge=self.bridge)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.root, ignore_errors=True)

    def test_invalid_form_shows_the_error_and_submits_nothing(self):
        """起任务前用 ``preview_domain`` 再确认一次：不合法就把原文写进卡片。"""
        bad = {"ok": False, "error": "参数不合法：演示用", "warnings": []}
        with mock.patch.object(generate_page.core, "preview_domain",
                               return_value=bad):
            job_id = self.page.start_scan()
        self.assertIsNone(job_id)
        self.assertEqual(self.registry.jobs(), [], "不合法却起了任务")
        self.assertEqual(self.page.error_label.text(), "参数不合法：演示用")
        self.assertFalse(self.page.error_label.isHidden())
        self.assertIn("参数不合法", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertFalse(self.page._busy)
        self.assertTrue(self.page.cancel_button.isHidden())

    def test_over_the_point_limit_submits_nothing(self):
        """手填 21×21×21 + 20×20 = 9661 点，超 5000 上限：拒绝起任务。"""
        fields = self.page.fields
        fields["grid_explicit"].setChecked(True)
        for key in ("xs", "ys", "zs"):
            fields[key].setValue(21)
        fields["z_insert"].setValue(20)
        self.assertTrue(self.page.total.over, self.page.total)
        self.assertIsNone(self.page.start_scan())
        self.assertEqual(self.registry.jobs(), [])
        self.assertIn("上限", self.page.status_label.text())

    def test_a_running_task_blocks_a_second_submit(self):
        """已有任务在跑时再点一次：不起第二条（否则单消费者队列会排队）。"""
        blocker = threading.Event()
        job_id = self.registry.submit("scan", lambda job: blocker.wait(TIMEOUT))
        try:
            self.assertTrue(wait_until(
                lambda: self.registry.get(job_id).state == "running"))
            self.bridge.attach(job_id)
            self.assertIsNone(self.page.start_scan())
            self.assertEqual(len(self.registry.jobs()), 1, "起了第二条任务")
            self.assertIn("已有任务在跑", self.page.status_label.text())
        finally:
            blocker.set()

    def test_page_creation_adopts_a_running_job(self):
        """页面重建时把还在跑的任务接回来（否则它既看不到也取消不了）。"""
        blocker = threading.Event()
        job_id = self.registry.submit("scan", lambda job: blocker.wait(TIMEOUT))
        try:
            self.assertTrue(wait_until(
                lambda: self.registry.get(job_id).state == "running"))
            adopted = generate_page.GeneratePage(
                plot=None, bridge=JobBridge(registry=self.registry, flush_ms=50))
            try:
                self.assertTrue(adopted._busy)
                self.assertEqual(adopted.bridge.job_id, job_id)
                self.assertEqual(adopted.run_state, "running")
                self.assertFalse(adopted.cancel_button.isHidden())
                self.assertIn("接管", adopted.status_label.text())
            finally:
                adopted.shutdown()
                adopted.deleteLater()
        finally:
            blocker.set()

    def test_failed_task_shows_the_error_in_red_and_keeps_the_log(self):
        """failed：状态行红字原文 + 自动切到日志页（日志区保留）。"""
        def boom(job):
            print("先写一行日志")
            raise RuntimeError("演示炸了")

        self.bridge.submit("scan", boom)
        self.assertTrue(wait_until(
            lambda: self.page.run_state in jobs.TERMINAL_STATES),
            "任务没结束")
        self.assertEqual(self.page.run_state, "failed")
        self.assertIn("演示炸了", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertEqual(self.page.result_tabs.currentWidget(),
                         self.page.log_page)
        self.assertIn("先写一行日志", self.page.log_edit.toPlainText())
        self.assertTrue(self.page.generate_button.isEnabled())


if __name__ == "__main__":
    unittest.main()
