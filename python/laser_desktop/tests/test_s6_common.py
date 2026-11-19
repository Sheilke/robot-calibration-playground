#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨页面共性问题：L1–L4（布局/段表/一致性/日志）+ S6b 的 P1–P6。

这些断言针对的是"三页各自看都能用、合起来现场就骂人"的那类问题，所以**三页
一起测**，而且都钉住能客观判定的数字（像素高度、可见行数、行数上限、调用次数、
默认值），不写"看起来改好了"。

S6b（2026-11-19）：S6 第一版把窗口钉成 1180×780 来凑"一屏 5 行"，代价是左侧
表单被横向裁掉、图区只剩 120 px。这里按新规格测：

* **P1**：默认 1600×950 / 最小 1280×800 / 记住上次窗口几何；
* **P2**：右栏三段比例，默认窗口下图 ≥260、结果 ≥300，最小窗口图 ≥200；
* **P3**：「机器人模型」组的高度 = 内容高度（三页一致）；
* **P4**：1280×800 下表单不横向裁剪（横滚条 0、数值框 width ≥ sizeHint）；
* **P6**：默认窗口 8 行、最小窗口 5 行（``viewport().visibleRegion()`` 口径）。

L1/P1–P6 用的页面是 ``plot=None``（不建 WebEngine）：布局与"图占位"完全一致，
图控件在真窗口里由 ``plot_view.PlotHost`` 搬进来（它的最小高度同样由
``page_layout.PLOT_FLOOR_HEIGHT`` 决定）。真 WebEngine 下的同一组数字在
``smoke_offscreen`` 里再测一次。

**P5**（关闭时序）与「1000 点 ``Plotly.react`` 计时」需要真 WebEngine，分别放在
``PlotShutdownTest`` / ``ReactTimingTest`` 里：后者本机 60~160 ms，阈值放宽到
1.5 s 只为防 CI 抖动。
"""
import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

# 必须在建 QApplication 之前：pytest 命令行不一定带 QT_QPA_PLATFORM
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import generate_page, page_layout, paths, plot_view
from laser_desktop.qt_compat import BINDING, QtCore, QtWidgets, Signal
from laser_web import jobs

_APP = None


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-s6"])
    return _APP


def setUpModule():
    """任何 QWidget 之前先建 QApplication（否则 Qt 直接 abort）。"""
    app()


def wait(ms: int) -> None:
    """真跑 ``ms`` 毫秒事件循环（布局要至少一轮才能量尺寸）。"""
    deadline = time.monotonic() + ms / 1000.0
    while True:
        app().processEvents()
        if time.monotonic() >= deadline:
            return
        time.sleep(0.005)


def visible_rows(table) -> int:
    """表格视口里**完整**可见的行数。

    用 ``viewport().visibleRegion()`` 而不是只看视口高度：表格被父容器（页签、
    分割器）裁掉时，视口自己的高度仍是 200，``visualItemRect`` 也在那 200 里，
    可屏幕上只剩 1 行——S6 第一次"看起来改好了"就是这么骗过去的。
    """
    region = table.viewport().visibleRegion().boundingRect()
    count = 0
    for row in range(table.rowCount()):
        item = table.item(row, 0)
        if item is None:
            continue
        rect = table.visualItemRect(item)
        if rect.top() >= region.top() and rect.bottom() < region.bottom():
            count += 1
    return count


def run_js(view, script: str, timeout_s: float = 10.0):
    """同步执行一段壳页 JS 并取回结果（只有真 WebEngine 走得到这里）。"""
    box = {}

    def _done(raw) -> None:
        box["raw"] = raw

    view.page().runJavaScript(script, 0, _done)
    deadline = time.monotonic() + timeout_s
    while "raw" not in box and time.monotonic() < deadline:
        app().processEvents()
        time.sleep(0.01)
    return box.get("raw")


def wait_loaded(view, timeout_s: float = 15.0) -> bool:
    """等壳页加载完（无显示器环境下可能一直等不到 → 调用方 ``skipTest``）。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app().processEvents()
        time.sleep(0.01)
        if view.loaded:
            return True
    return False


def marker_sizes(view, expected_first=None, timeout_s: float = 5.0,
                 count: int = 0):
    """读壳页里每个 trace 的 ``marker.size``（等得起 ``Plotly.restyle`` 的异步）。

    ``restyle`` 返回 Promise，读完立刻断言会读到**改之前**的数组；所以给了
    ``expected_first`` 就轮询到它出现为止（超时返回最后一次读到的值，由调用方
    报错）。

    注意：还没 ``restyle`` 过的 trace，``marker.size`` 是**标量**（图里写的 5），
    所以这里统一展开成 ``count`` 个一份的数组（``[5]`` -> ``[5, 5, ...]``），
    两条路都能直接比。
    """
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        raw = run_js(view, "window.__markerSizes()", timeout_s=3.0)
        try:
            parsed = json.loads(str(raw or "[]"))
        except (TypeError, ValueError):
            parsed = None
        if parsed:
            last = [_expand_size(value, count) for value in parsed]
        if last and (expected_first is None
                     or list(last[0]) == list(expected_first)):
            return last
        app().processEvents()
        time.sleep(0.02)
    return last


def _expand_size(value, count: int):
    """``marker.size``（标量或数组）-> ``count`` 个一份的数组。"""
    if isinstance(value, (list, tuple)):
        return list(value)
    if count <= 0:
        return [value]
    return [value] * count


def scan_result(count: int = 8, *, fail_at=None) -> dict:
    """构造一份与 ``core.run_scan`` 同构的返回值（不跑 IK）。"""
    waypoints = []
    for index in range(count):
        ok = index != fail_at
        waypoints.append({
            "gp": index + 1, "index": index,
            "pos": [900.0 + 25.0 * index, -100.0, 500.0 + 10.0 * index],
            "rpy_pert": [0.0, 0.0, 0.0] if ok else None,
            "q": [10.0, -20.0, 30.0, 40.0, 50.0, 60.0] if ok else None,
            "j3_abs": 30.0 + index if ok else None,
            "j5_abs": 50.0 + index if ok else None,
            "config": [0, 1, 0] if ok else None,
            "dq_l2": 0.0, "ok": ok})
    segments = [{"gp_from": index + 1, "gp_to": index + 2,
                 "j3_abs_min": 30.0, "j5_abs_min": 50.0, "crosses": False,
                 "worst_frac": 0.5, "worst_pos": [1000.0, 0.0, 500.0]}
                for index in range(count - 1)]
    stats = {"solved": count - (0 if fail_at is None else 1), "total": count,
             "switches": 2, "dq_l2_sum": 1.0, "j5_abs_min": 50.0,
             "j3_abs_min": 30.0, "design_cond": 14.0, "post_ik_cond": 18.0,
             "seg_j5_min": 50.0, "seg_j3_min": 30.0, "j3_crossings": 0,
             "j5_crossings": 0, "guard_violations": 0, "feasible": True}
    return {"ok": True, "kind": "scan", "run_id": "20260101-000000-abcd",
            "run_dir": "/tmp/s6-none", "stats": stats,
            "meta": {"skip_ik": False}, "waypoints": waypoints,
            "segments": segments, "warnings": [], "artifacts": {}}


def iso_result(count: int = 5) -> dict:
    """构造一份与 ``core.run_iso9283`` 同构的返回值（P1..P5）。"""
    points = [{"name": "P%d" % (index + 1),
               "pos": [1025.0, 0.0, 605.0], "ok": True,
               "j3_abs": 30.0, "j5_abs": 50.0, "config": [0, 1, 0],
               "rpy_pert": [0.0, 0.0, 0.0],
               "q": [10.0, -20.0, 30.0, 40.0, 50.0, 60.0]}
              for index in range(count)]
    stats = {"solved": count, "total": count, "j5_abs_min": 50.0,
             "j3_abs_min": 30.0, "cube_side": 200.0,
             "cube_center": [1025.0, 0.0, 605.0], "layout": "vertices",
             "plane": "xz", "configs_consistent": True,
             "config_label": "0,1,0"}
    return {"ok": True, "kind": "iso", "run_id": "20260101-000000-abcd",
            "run_dir": "/tmp/s6-none", "stats": stats, "points": points,
            "meta": {}, "warnings": [], "artifacts": {}}


class StubPlot(QtWidgets.QWidget):
    """记录被推过来的图与选中键；还能让测试假装"图上点了一下"。"""

    pointClicked = Signal(object)

    def __init__(self):
        super().__init__()
        self.figures = []
        self.selected = []

    def set_figure(self, figure):
        self.figures.append(figure)

    def select_points(self, keys, notify: bool = False):
        self.selected.append(list(keys))


# ---------------------------------------------------------------------------
# L1 / P2 / P6：一屏看得到几行 + 右栏三段比例
# ---------------------------------------------------------------------------
class ResultRowsVisibleTest(unittest.TestCase):
    """P6：默认 1600×950 下三页都要把该看的行全露出来（≥8 行）。

    口径仍是 ``viewport().visibleRegion()``：只看 ``rowCount()`` 或表控件的
    高度会被"表高 200 但屏幕上只有 1 行"骗过去。
    """

    DEFAULT = page_layout.DEFAULT_WINDOW_SIZE
    MINIMUM = page_layout.MINIMUM_WINDOW_SIZE

    def _window(self, page, size):
        window = QtWidgets.QMainWindow()
        window.setCentralWidget(page)
        window.resize(*size)
        window.show()
        wait(160)
        return window

    def _build(self, kind):
        """造出三个页之一：返回 (page, 渲染回调, 显示表的回调, 表属性名)。"""
        if kind == "generate":
            page = generate_page.GeneratePage()
            return (page,
                    lambda: page.render_result(scan_result(8)),
                    lambda: page.result_tabs.setCurrentWidget(
                        page.waypoints_page),
                    "waypoint_table")
        if kind == "repair":
            from laser_desktop import repair_page

            page = repair_page.RepairPage()
            return (page,
                    lambda: page.render_plan(scan_result(8)),
                    lambda: page.result_tabs.setCurrentWidget(page.plan_page),
                    "waypoint_table")
        from laser_desktop import iso_page

        page = iso_page.IsoPage()
        return (page,
                lambda: page.render_result(iso_result(5)),
                lambda: page.result_tabs.setCurrentWidget(page.points_page),
                "point_table")

    def _check_rows(self, kind, label, want_rows, size):
        page, render, show_tab, attr = self._build(kind)
        window = self._window(page, size)
        render()
        wait(200)
        show_tab()
        wait(200)
        table = getattr(page, attr)
        rows = visible_rows(table)
        self.assertGreaterEqual(
            table.height(), page_layout.RESULT_TABLE_MIN_HEIGHT,
            f"{label}: 表高 {table.height()} 小于最小高度")
        self.assertGreaterEqual(
            rows, want_rows,
            f"{label}（{size[0]}×{size[1]}）: 只看到 {rows} 行（要求 {want_rows}；"
            f"表高 {table.height()}，viewport {table.viewport().height()}，"
            f"表里共 {table.rowCount()} 行）")
        self.assertEqual(window.width(), size[0],
                         f"{label}: 窗口被撑宽到 {window.width()}")
        self.assertEqual(window.height(), size[1],
                         f"{label}: 窗口被撑高到 {window.height()}")
        window.close()
        page.deleteLater()

    # ---- 默认窗口 1600×950：8 点全见（ISO 只有 5 点 = 5 行） ----
    def test_generate_page_shows_eight_rows_at_default_size(self):
        self._check_rows("generate", "生成页", 8, self.DEFAULT)

    def test_repair_page_shows_eight_rows_at_default_size(self):
        self._check_rows("repair", "修复页", 8, self.DEFAULT)

    def test_iso_page_shows_all_five_rows_at_default_size(self):
        self._check_rows("iso", "ISO 页", 5, self.DEFAULT)

    # ---- 最小窗口 1280×800：至少 5 行（P6 的下限） ----
    def test_generate_page_shows_five_rows_at_minimum_size(self):
        self._check_rows("generate", "生成页", 5, self.MINIMUM)

    def test_repair_page_shows_five_rows_at_minimum_size(self):
        self._check_rows("repair", "修复页", 5, self.MINIMUM)

    def test_iso_page_shows_five_rows_at_minimum_size(self):
        self._check_rows("iso", "ISO 页", 5, self.MINIMUM)

    def test_right_column_has_three_draggable_sections(self):
        """右栏是三段分割器，三段各自守自己的下限，且能拖宽结果区。

        断言**不再**去比 ``DEFAULT_SPLIT_RATIOS`` 的归一化值：S6c 之后首开高度
        是"预览卡按内容拿、剩下给图与结果"算出来的，与那个比例常量本就不相等
        （无结果时预览卡内容更矮，结果区是空的，实际比例自然偏移）。比常量只会
        把实现细节钉死，改一次布局就要改一次测试。

        真正要守住的是三件事：

        1. 是三段、不是两段或平分布局；
        2. 每段都不低于自己的最小值（否则结果表只剩表头）；
        3. 拖分割条能把结果区拉大（用户要能自己分配空间）。
        """
        page = generate_page.GeneratePage()
        window = self._window(page, self.DEFAULT)
        splitter = page._viewer_splitter
        self.assertIsInstance(splitter, page_layout.RightSplitter)
        self.assertEqual(splitter.count(), 3)
        sizes = splitter.sizes()
        self.assertEqual(sum(splitter.ratios), 1000,
                         f"比例未归一化：{splitter.ratios}")
        self.assertGreaterEqual(
            sizes[1], page_layout.PLOT_FLOOR_HEIGHT,
            f"图区 {sizes[1]} 低于硬下限 {page_layout.PLOT_FLOOR_HEIGHT}")
        self.assertGreater(sizes[2], 0, f"结果区被压成 0：{sizes}")
        before = splitter.sizes()
        splitter.setSizes([before[0] - 60, before[1] - 60, before[2] + 120])
        wait(40)
        self.assertGreater(splitter.sizes()[2], before[2],
                           "拖分割条没有把结果区拉大")
        window.close()
        page.deleteLater()

    # ---- P2：三段高度的下限（默认窗口 / 最小窗口） ----
    def _check_segments(self, kind, label):
        page, render, show_tab, _attr = self._build(kind)
        window = self._window(page, self.DEFAULT)
        render()
        wait(200)
        show_tab()
        wait(200)
        preview, plot, results = page._viewer_splitter.sizes()
        self.assertGreaterEqual(plot, page_layout.PLOT_MIN_HEIGHT,
                                f"{label}: 图区只有 {plot} px")
        self.assertGreaterEqual(results, page_layout.RESULT_MIN_HEIGHT,
                                f"{label}: 结果区只有 {results} px")
        self.assertGreaterEqual(preview, page_layout.PREVIEW_MIN_HEIGHT,
                                f"{label}: 预览卡只有 {preview} px")
        window.resize(*self.MINIMUM)
        wait(200)
        preview, plot, results = page._viewer_splitter.sizes()
        self.assertGreaterEqual(
            plot, page_layout.PLOT_FLOOR_HEIGHT,
            f"{label}: 1280×800 下图区只有 {plot} px（硬下限 "
            f"{page_layout.PLOT_FLOOR_HEIGHT}）")
        self.assertGreaterEqual(
            results, page_layout.RESULT_MIN_HEIGHT,
            f"{label}: 1280×800 下结果区只有 {results} px")
        window.close()
        page.deleteLater()

    def test_segments_at_default_and_minimum_size_generate(self):
        self._check_segments("generate", "生成页")

    def test_segments_at_default_and_minimum_size_repair(self):
        self._check_segments("repair", "修复页")

    def test_segments_at_default_and_minimum_size_iso(self):
        self._check_segments("iso", "ISO 页")


class SplitterSettingsTest(unittest.TestCase):
    """比例存 QSettings，重开窗口能读回（P2 的第三条验收）。"""

    def setUp(self):
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.settings = QtCore.QSettings(
            str(self.tmp / "layout.ini"), QtCore.QSettings.IniFormat)

    def test_ratios_survive_a_rebuild(self):
        size = page_layout.DEFAULT_WINDOW_SIZE
        first = generate_page.GeneratePage(settings=self.settings)
        first.resize(*size)
        first.show()
        wait(120)
        splitter = first._viewer_splitter
        total = sum(splitter.sizes())
        self.assertGreater(total, 0)
        splitter.setSizes([total // 2, total // 4,
                           total - total // 2 - total // 4])
        splitter.save_ratios()                   # 等价于用户拖完自动存
        saved = splitter.ratios                  # 设计比例（= 屏幕上的这一份）
        first.close()

        second = generate_page.GeneratePage(settings=self.settings)
        self.assertEqual(second._viewer_splitter.saved_ratios(), saved,
                         "QSettings 里读回来的比例与存下去的不一致")
        self.assertEqual(tuple(second._viewer_splitter.ratios), saved,
                         "读回来的比例没被当作设计比例")
        second.resize(*size)
        second.show()
        wait(120)
        # 屏幕上的真实比例会因"护图区/撑结果区"微调，但差别不该超过 15%
        for actual, expected in zip(second._viewer_splitter.live_ratios(),
                                    saved):
            self.assertAlmostEqual(actual / expected, 1.0, delta=0.15,
                                   msg=f"存 {saved} 读回 {second._viewer_splitter.live_ratios()}")
        second.close()

    def test_reset_returns_to_the_default_ratios(self):
        page = generate_page.GeneratePage(settings=self.settings)
        page.resize(*page_layout.DEFAULT_WINDOW_SIZE)
        page.show()
        wait(120)
        page._viewer_splitter.reset_ratios()
        wait(60)
        self.assertEqual(tuple(page._viewer_splitter.ratios),
                         page_layout._normalize(
                             page_layout.DEFAULT_SPLIT_RATIOS))
        page.close()


# ---------------------------------------------------------------------------
# L2：段诊断表行数上限
# ---------------------------------------------------------------------------
class _CountingTable(generate_page.ResultTable):
    """数 ``resizeColumnsToContents`` 被调用了几次（真表不能加属性）。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.resize_calls = 0

    def resizeColumnsToContents(self):          # noqa: N802 - Qt 覆写
        self.resize_calls += 1
        super().resizeColumnsToContents()


class SegmentTableLimitTest(unittest.TestCase):
    """5000 段：只显示 500 行、给出提示、不逐列量宽、200 ms 内出表。"""

    @staticmethod
    def _segments(count: int):
        return [{"gp_from": index + 1, "gp_to": index + 2,
                 "j3_abs_min": float(index % 17), "j5_abs_min": float(index % 13),
                 "crosses": index % 50 == 0, "worst_frac": 0.5,
                 "worst_pos": [1000.0, 0.0, 500.0]} for index in range(count)]

    def test_five_thousand_segments_are_capped(self):
        table = _CountingTable(0, len(generate_page.SEGMENT_HEADERS))
        table.setHorizontalHeaderLabels(list(generate_page.SEGMENT_HEADERS))
        caption = QtWidgets.QLabel("")
        started = time.perf_counter()
        shown = generate_page.fill_segment_table(
            table, self._segments(5000), caption=caption,
            caption_text="段诊断表")
        elapsed = (time.perf_counter() - started) * 1000.0
        self.assertEqual(shown, generate_page.SEGMENT_ROW_LIMIT)
        self.assertEqual(table.rowCount(), generate_page.SEGMENT_ROW_LIMIT)
        self.assertIn("共 5000 段", caption.text())
        self.assertIn(str(generate_page.SEGMENT_ROW_LIMIT), caption.text())
        self.assertEqual(table.resize_calls, 0,
                         "超过 200 行还逐列量宽（会卡住界面）")
        self.assertLess(elapsed, 200.0, f"5000 段出表用了 {elapsed:.0f} ms")

    def test_small_tables_still_autofit(self):
        table = _CountingTable(0, len(generate_page.SEGMENT_HEADERS))
        table.setHorizontalHeaderLabels(list(generate_page.SEGMENT_HEADERS))
        caption = QtWidgets.QLabel("段诊断表")
        generate_page.fill_segment_table(
            table, self._segments(12), caption=caption, caption_text="段诊断表")
        self.assertEqual(table.rowCount(), 12)
        self.assertEqual(table.resize_calls, 1)
        self.assertEqual(caption.text(), "段诊断表", "没截断就别改说明文字")


# ---------------------------------------------------------------------------
# L3：表单两份副本的一致性（自动化检查，不靠注释）
# ---------------------------------------------------------------------------
class FormConsistencyTest(unittest.TestCase):
    """两处（form_widgets 与实际控件）的默认值/上限必须一致。

    改任何一处默认值或上限，这里就会红：这正是 ``# SYNC:`` 清单要防的事。
    """

    def test_grid_limits_match_the_shared_constants(self):
        from laser_desktop import form_widgets

        fields = {}
        # 留住整组控件：只留 fields 的话 group 会被 GC，里面的 QSpinBox 跟着没
        group = form_widgets.build_grid_group(
            fields, generate_page.form_defaults(), lambda *_args: None)
        self.assertIsNotNone(group["group"])
        self.assertEqual(fields["xs"].maximum(), form_widgets.GRID_VERTICES_MAX)
        self.assertEqual(fields["ys"].maximum(), form_widgets.GRID_VERTICES_MAX)
        self.assertEqual(fields["zs"].maximum(), form_widgets.GRID_VERTICES_MAX)
        self.assertEqual(fields["z_insert"].maximum(),
                         form_widgets.Z_INSERT_MAX)
        self.assertEqual(fields["target_count"].maximum(),
                         generate_page.TARGET_COUNT_MAX)

    def test_defaults_match_form_defaults(self):
        from laser_desktop import form_widgets

        defaults = generate_page.form_defaults()
        fields = {}
        group = form_widgets.build_grid_group(fields, defaults,
                                              lambda *_args: None)
        self.assertIsNotNone(group["group"])
        self.assertEqual(fields["target_count"].value(),
                         int(defaults["target_count"]))
        for key in ("xs", "ys", "zs", "z_insert"):
            self.assertEqual(fields[key].value(), int(defaults[key]), key)

    def test_generate_and_repair_share_the_same_six_groups(self):
        """两页的六组字段：同名、同默认值、同上下限（只在 form_widgets 写一次）。"""
        from laser_desktop import repair_page

        generate = generate_page.GeneratePage()
        repair = repair_page.RepairPage()
        # ``seg_samples`` 两页**故意**不同（扫描 CLI 21 / 修复 CLI 11，
        # 见 core.repair_cli_defaults 的说明），不算"副本分叉"。
        shared = [key for key in generate.fields
                  if key in repair.fields and key != "seg_samples"]
        self.assertGreaterEqual(len(shared), 20, shared)
        for key in shared:
            mine, theirs = generate.fields[key], repair.fields[key]
            for attribute in ("minimum", "maximum"):
                left = getattr(mine, attribute, lambda: None)()
                right = getattr(theirs, attribute, lambda: None)()
                self.assertEqual(left, right, f"{key}.{attribute} 两页不一致")
            if hasattr(mine, "value") and hasattr(mine, "currentData"):
                self.assertEqual(mine.currentData(), theirs.currentData(),
                                 f"{key} 两页默认值不一致")
            elif hasattr(mine, "value"):
                self.assertEqual(mine.value(), theirs.value(),
                                 f"{key} 两页默认值不一致")
        generate.deleteLater()
        repair.deleteLater()

    def test_point_limits_track_the_algorithm(self):
        from excitation_trajectory import aabb_snake_scan as scan
        from laser_web import pages

        expected = ((scan.COUNT_GRID_MAX_PER_AXIS + 1) ** 3
                    + scan.COUNT_GRID_MAX_PER_AXIS
                    * scan.COUNT_GRID_MAX_INSERT)
        self.assertEqual(generate_page.TARGET_COUNT_MAX, expected)
        self.assertEqual(generate_page.POINT_COUNT_WARN, pages.POINT_COUNT_WARN)


# ---------------------------------------------------------------------------
# L4：日志源缓冲与控件一致，800 行的省略数正确
# ---------------------------------------------------------------------------
class LogLimitTest(unittest.TestCase):
    """源缓冲 500 = 控件 500；打 800 行后末行还在、省略数说对。"""

    def test_source_and_widget_limits_agree(self):
        from laser_desktop import job_bridge

        self.assertGreaterEqual(jobs.LOG_LIMIT, 500)
        self.assertEqual(jobs.LOG_LIMIT, generate_page.LOG_WIDGET_LIMIT)
        self.assertEqual(job_bridge.LOG_LIMIT, jobs.LOG_LIMIT)

    def test_eight_hundred_lines_keep_the_last_one_and_the_count(self):
        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        registry = jobs.JobRegistry(root=tmp)
        job = registry.get(registry.submit("scan", lambda active: active.cancel()))
        try:
            for index in range(800):
                job.append_log(f"line {index}")
            snapshot = job.snapshot(0)
            self.assertEqual(len(snapshot["log"]), jobs.LOG_LIMIT)
            self.assertEqual(snapshot["dropped"], 800 - jobs.LOG_LIMIT)

            # 页面收到的是：一行省略提示 + 源缓冲里剩下的 500 行
            page = generate_page.GeneratePage()
            page._on_log_line(
                f"…已省略 {snapshot['dropped']} 行（源缓冲只留最近 "
                f"{jobs.LOG_LIMIT} 行）")
            for line in snapshot["log"]:
                page._on_log_line(line)
            text = page.log_edit.toPlainText()
            self.assertTrue(text.rstrip().endswith("line 799"),
                            text[-80:])
            self.assertIn("line 799", text)
            self.assertNotIn("line 0\n", text)
            omitted = page.log_omission_label.text()
            self.assertIn(str(800 - jobs.LOG_LIMIT), omitted)
            self.assertTrue(page.log_omission_label.isVisibleTo(page) or omitted)
            page.deleteLater()
        finally:
            registry.shutdown()


# ---------------------------------------------------------------------------
# P1：表格 ↔ 图双向
# ---------------------------------------------------------------------------
class TablePlotLinkTest(unittest.TestCase):
    """三页各一条：选行 → 调 ``select_points``；图上点 → 选行。"""

    def _generate(self):
        plot = StubPlot()
        page = generate_page.GeneratePage(plot=plot)
        page.render_result(scan_result(8, fail_at=2))
        return page, plot, page.waypoint_table

    def _repair(self):
        from laser_desktop import repair_page

        plot = StubPlot()
        page = repair_page.RepairPage(plot=plot)
        page.render_plan(scan_result(8, fail_at=2))
        return page, plot, page.waypoint_table

    def _iso(self):
        from laser_desktop import iso_page

        plot = StubPlot()
        page = iso_page.IsoPage(plot=plot)
        page.render_result(iso_result(5))
        return page, plot, page.point_table

    def _check_page(self, build, key_at_row_four, expected_label):
        page, plot, table = build()
        table.setCurrentCell(4, 0)              # 第 5 行
        wait(30)
        self.assertEqual(plot.selected[-1], [key_at_row_four],
                         f"{type(page).__name__}: 选行没有把键推给图")
        self.assertIn(expected_label, page.last_status)

        plot.selected.clear()
        plot.pointClicked.emit([key_at_row_four])
        wait(30)
        self.assertEqual(table.currentRow(), 4,
                         f"{type(page).__name__}: 图上点选没有选中表格行")
        self.assertEqual(plot.selected[-1], [key_at_row_four])
        page.deleteLater()

    def test_generate_page(self):
        self._check_page(self._generate, "5", "GP0005")

    def test_repair_page(self):
        self._check_page(self._repair, "5", "GP0005")

    def test_iso_page(self):
        self._check_page(self._iso, "P5", "P5")

    def test_figures_carry_gp_keys_for_every_point(self):
        """``customdata[0]`` 一律是 GP：失败点那条 trace 也要有。"""
        page, _plot, _table = self._generate()
        figure = page.last_figure
        keys = []
        for trace in figure.data:
            keys.extend(list(trace.customdata or []))
        flat = [row[0] if isinstance(row, (list, tuple)) else row
                for row in keys]
        # trace 顺序是"成功点一条、失败点一条"，所以键不按序号连续；
        # 但每个 GP 必须恰好出现一次，失败点（GP3）也要在。
        self.assertEqual(sorted(flat), list(range(1, 9)))
        self.assertIn(3, flat)

    def test_preview_figure_keys_start_at_one(self):
        """预览图（生成页）也是 GP1..GPn，不是 0 基下标。"""
        figure = generate_page.build_preview_figure()
        custom = figure.data[0]["customdata"]
        self.assertEqual(custom[0][0], 1)
        self.assertEqual(custom[-1][0], len(custom))

    def test_iso_result_keys_are_point_names(self):
        page, _plot, _table = self._iso()
        keys = []
        for trace in page.last_figure.data:
            for row in (trace.customdata or []):
                keys.append(row[0] if isinstance(row, (list, tuple)) else row)
        self.assertEqual(keys, ["P1", "P2", "P3", "P4", "P5"])


# ---------------------------------------------------------------------------
# P1：窗口尺寸策略（默认 1600×950 / 最小 1280×800 / 记住上次几何）
# ---------------------------------------------------------------------------
class WindowStrategyTest(unittest.TestCase):
    """P1：窗口不再为"一屏 5 行"钉死在 1180×780。"""

    def setUp(self):
        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.settings = QtCore.QSettings(
            str(tmp / "window.ini"), QtCore.QSettings.IniFormat)

    def test_default_size_is_1600x950(self):
        from laser_desktop import main_window

        window = main_window.MainWindow(settings=self.settings)
        self.assertEqual(page_layout.DEFAULT_WINDOW_SIZE, (1600, 950))
        self.assertEqual((window.width(), window.height()), (1600, 950))
        self.assertEqual(page_layout.MINIMUM_WINDOW_SIZE, (1280, 800))
        self.assertEqual((window.minimumWidth(), window.minimumHeight()),
                         (1280, 800))
        window.close()

    def test_nav_is_about_92px(self):
        """P4：导航栏收窄（130 → 92），省下的 38 px 给表单和图。

        S7 起宽度 = ``max(NAV_WIDTH, 最长一项)``：92 px 是 96 dpi 下的量法
        （最长一项「奇异修复」74 px）；现场机 ``Xft.dpi=192``，同样四项要
        122 px，钉死 92 就把「奇异修复」切成「奇异修」。所以这里断言的是
        “下限仍是 92、且不比最长一项窄”，不再钉死等于 92。
        """
        from laser_desktop import main_window

        window = main_window.MainWindow(settings=self.settings)
        window.show()
        wait(80)
        self.assertGreaterEqual(window.nav.width(), page_layout.NAV_WIDTH)
        self.assertGreaterEqual(
            window.nav.width(), window.nav.sizeHintForColumn(0),
            "导航比最长的一项还窄——文字会被切")
        self.assertLessEqual(page_layout.NAV_WIDTH, 100)
        window.close()

    def test_layout_fits_the_minimum_window(self):
        """三段最小高度之和必须装得下 1280×800（否则窗口会被自己撑大）。"""
        from laser_desktop import main_window

        window = main_window.MainWindow(settings=self.settings)
        window.show()
        wait(160)
        hint = window.minimumSizeHint()
        self.assertLessEqual(
            hint.height(), page_layout.MINIMUM_WINDOW_SIZE[1],
            f"窗口最小高度 {hint.height()} 超过 800（三段装不下）")
        self.assertLessEqual(
            hint.width(), page_layout.MINIMUM_WINDOW_SIZE[0],
            f"窗口最小宽度 {hint.width()} 超过 1280")
        window.close()

    def test_last_window_size_is_remembered(self):
        """关窗时的尺寸写进 QSettings，下次开窗照它摆回来。"""
        from laser_desktop import main_window

        first = main_window.MainWindow(settings=self.settings)
        first.show()
        wait(80)
        first.resize(1480, 930)
        wait(120)
        first.close()

        second = main_window.MainWindow(settings=self.settings)
        self.assertEqual((second.width(), second.height()), (1480, 930),
                         "上次关窗时的尺寸没被记住")
        second.close()

    def test_tiny_saved_geometry_falls_back_to_the_default(self):
        """旧版本存过 1180×780：读回来比最小尺寸小，就用默认尺寸。"""
        from laser_desktop import main_window

        first = main_window.MainWindow(settings=self.settings)
        first.resize(1180, 780)          # 模拟旧版本的记录（无 minimum 约束）
        page_layout.save_window_geometry(first, self.settings)
        first.close()

        second = main_window.MainWindow(settings=self.settings)
        self.assertGreaterEqual(second.width(),
                                page_layout.MINIMUM_WINDOW_SIZE[0])
        self.assertGreaterEqual(second.height(),
                                page_layout.MINIMUM_WINDOW_SIZE[1])
        second.close()


# ---------------------------------------------------------------------------
# P3：「机器人模型」组的高度 = 内容高度（三页一致）
# ---------------------------------------------------------------------------
class RobotGroupHeightTest(unittest.TestCase):
    """P3：分组不许被拉伸，说明文字也不许白占一行。

    旧版两条说明各钉 44 px（"两行高"），三行内容被撑成 185 px——截图里就是
    "模型 / 路径 / 另有 5 个文件"之间的大段空白。
    """

    #: 旧版实测 185 px；修好后 147 px。180 是回归线。
    MAX_LEGACY_HEIGHT = 180

    def _check(self, build, label):
        page = build()
        window = QtWidgets.QMainWindow()
        window.setCentralWidget(page)
        window.resize(*page_layout.DEFAULT_WINDOW_SIZE)
        window.show()
        wait(160)
        group = page.robot_group
        hint = group.sizeHint().height()
        self.assertLessEqual(
            group.height(), hint * 1.35,
            f"{label}: 组高 {group.height()} 超过 sizeHint {hint} 的 1.35 倍"
            "（被外层布局拉伸了）")
        self.assertLessEqual(
            group.height(), self.MAX_LEGACY_HEIGHT,
            f"{label}: 组高 {group.height()} 还是旧版的\"拉散\"高度"
            f"（内容只有 {hint} px）")
        # 说明文字只占实际行数的高度（一行 ≈ lineSpacing，不再是 44 px）
        line = page.robot_path_label.fontMetrics().lineSpacing()
        self.assertLessEqual(
            page.robot_path_label.height(), line * 2 + 4,
            f"{label}: 路径说明高 {page.robot_path_label.height()} px，"
            f"一行只有 {line} px")
        self.assertEqual(
            group.sizePolicy().verticalPolicy(),
            QtWidgets.QSizePolicy.Maximum,
            f"{label}: 分组的垂直策略不是 Maximum（会被拉伸）")
        window.close()
        page.deleteLater()

    def test_generate_page(self):
        self._check(generate_page.GeneratePage, "生成页")

    def test_repair_page(self):
        from laser_desktop import repair_page

        self._check(repair_page.RepairPage, "修复页")

    def test_iso_page(self):
        from laser_desktop import iso_page

        self._check(iso_page.IsoPage, "ISO 页")

    def test_three_pages_agree(self):
        """三页同一份实现：组高一致（±4 px，字体/行数差异之外没有分叉）。"""
        from laser_desktop import iso_page, repair_page

        heights = []
        for build in (generate_page.GeneratePage, repair_page.RepairPage,
                      iso_page.IsoPage):
            page = build()
            page.resize(*page_layout.DEFAULT_WINDOW_SIZE)
            page.show()
            wait(120)
            heights.append(page.robot_group.sizeHint().height())
            page.close()
            page.deleteLater()
        self.assertLessEqual(max(heights) - min(heights), 4, heights)


# ---------------------------------------------------------------------------
# P4：左侧表单在 1280×800 下不许横向裁剪
# ---------------------------------------------------------------------------
class FormNoHorizontalClippingTest(unittest.TestCase):
    """P4：横滚条 maximum == 0、数值框 width ≥ sizeHint、按钮完整可见。"""

    MINIMUM = page_layout.MINIMUM_WINDOW_SIZE

    def _window(self, page):
        window = QtWidgets.QMainWindow()
        window.setCentralWidget(page)
        window.resize(*self.MINIMUM)
        window.show()
        wait(200)
        return window

    def _check_scroll(self, page, label):
        scroll = page.form_scroll
        bar = scroll.horizontalScrollBar()
        self.assertEqual(
            bar.maximum(), 0,
            f"{label}: 表单出现横向滚动条（maximum={bar.maximum()}，"
            f"视口 {scroll.viewport().width()} < 内容 "
            f"{scroll.widget().minimumSizeHint().width()}）")

    def _check_spins(self, page, keys, label):
        for key in keys:
            box = page.fields[key]
            self.assertIsNotNone(box, f"{label}: 没有 {key} 输入框")
            self.assertGreaterEqual(
                box.width(), box.sizeHint().width(),
                f"{label}: {key} 被压到 {box.width()} px（sizeHint "
                f"{box.sizeHint().width()}）——文字会被切掉")

    def test_generate_page_form_is_not_clipped(self):
        page = generate_page.GeneratePage()
        window = self._window(page)
        page.fields["domain_axes"].setChecked(True)      # 「六个轴边界」面板
        wait(120)
        self._check_scroll(page, "生成页")
        self._check_spins(page, ("y_min", "z_min"), "生成页")
        window.close()
        page.deleteLater()

    def test_repair_page_form_is_not_clipped(self):
        from laser_desktop import repair_page

        page = repair_page.RepairPage()
        window = self._window(page)
        page.fields["domain_axes"].setChecked(True)
        wait(120)
        self._check_scroll(page, "修复页")
        self._check_spins(page, ("y_min", "z_min"), "修复页")
        # 「删除本组」要完整露出来（旧版截图里只露一半）
        button = page.q_rows[0]["remove_button"]
        self.assertGreaterEqual(button.width(), button.sizeHint().width(),
                                "删除本组按钮被压窄")
        viewport = page.form_scroll.viewport()
        top_left = button.mapTo(page.form_scroll.widget(),
                                button.rect().topLeft())
        self.assertGreaterEqual(top_left.x(), 0,
                                "删除本组按钮被推到表单左侧之外")
        self.assertLessEqual(top_left.x() + button.width(),
                             page.form_scroll.widget().width() + 1,
                             "删除本组按钮超出表单内容宽度")
        self.assertGreater(viewport.width(), 0)
        window.close()
        page.deleteLater()

    def test_iso_page_form_is_not_clipped(self):
        from laser_desktop import iso_page

        page = iso_page.IsoPage()
        window = self._window(page)
        wait(120)
        self._check_scroll(page, "ISO 页")
        self._check_spins(page, ("y0", "z0"), "ISO 页")
        window.close()
        page.deleteLater()

    def test_spin_minimum_width_tracks_the_size_hint(self):
        """数值框的最小宽度 = max(下限, sizeHint)——这才是"不会看不全"的保证。"""
        box = generate_page._double_spin(-10000.0, 10000.0, 500.0)
        self.assertGreaterEqual(box.minimumWidth(), box.sizeHint().width())
        integer = generate_page._int_spin(1, 100000, 50)
        self.assertGreaterEqual(integer.minimumWidth(),
                                integer.sizeHint().width())


# ---------------------------------------------------------------------------
# P5：关闭时序（先删 page 再删 view；5 次开关不崩）
# ---------------------------------------------------------------------------
class PlotShutdownTest(unittest.TestCase):
    """P5：``QWebEnginePage`` 必须比 profile 先没，否则 Qt 报 "Expect troubles !"。"""

    @classmethod
    def setUpClass(cls):
        from laser_desktop import qt_compat

        plot_view.webchannel_js_text()          # 触发 QtWebChannel 资源注册
        cls.qt_compat = qt_compat
        cls.app = app()

    def test_shutdown_deletes_the_page_before_the_view(self):
        view = plot_view.PlotView()
        page = view.page()
        self.assertIsNotNone(page)
        view.shutdown()
        self.assertTrue(self.qt_compat.is_deleted(page),
                        "页面没被删掉：profile 释放时会报 not deleted")
        self.assertTrue(self.qt_compat.is_deleted(view), "视图没被删掉")

    def test_shutdown_is_idempotent(self):
        view = plot_view.PlotView()
        view.shutdown()
        view.shutdown()                          # 再调一次不许抛

    def test_five_open_close_cycles(self):
        """建 → shutdown → 再建，连做 5 次不崩、每次页面都被删掉。"""
        for index in range(5):
            view = plot_view.PlotView()
            view.resize(400, 300)
            view.show()
            page = view.page()
            self.app.processEvents()
            view.shutdown()
            self.assertTrue(self.qt_compat.is_deleted(page),
                            f"第 {index + 1} 次：页面没被删掉")
            self.assertTrue(self.qt_compat.is_deleted(view),
                            f"第 {index + 1} 次：视图没被删掉")

    def test_five_main_window_open_close_cycles(self):
        """关窗 5 次：每次 ``closeEvent`` 都要把共享视图拆掉（不崩、不残留）。"""
        from laser_desktop import main_window

        settings = QtCore.QSettings(
            str(Path(self.enterContext(tempfile.TemporaryDirectory()))
                / "win.ini"), QtCore.QSettings.IniFormat)
        for index in range(5):
            window = main_window.MainWindow(settings=settings)
            window.show()
            wait(60)
            view = window.plot.ensure()          # 真的建出 WebEngine 视图
            wait(60)
            window.close()
            wait(60)
            self.assertFalse(window.plot.created,
                             f"第 {index + 1} 次关窗后视图还挂着")
            self.assertTrue(self.qt_compat.is_deleted(view),
                            f"第 {index + 1} 次关窗后 QWebEngineView 没被删掉")

    def test_host_shutdown_clears_the_view(self):
        host = plot_view.PlotHost()
        view = host.ensure()
        self.assertTrue(host.created)
        host.shutdown()
        self.assertIsNone(host.view)
        self.assertFalse(host.created)
        self.assertTrue(self.qt_compat.is_deleted(view))
        # 关掉之后再要图：重新建一个（"关窗 → 再开"不会留下坏状态）
        again = host.ensure()
        self.assertIsNot(again, view)
        host.shutdown()

    def test_no_profile_release_warning_on_exit(self):
        """真起一个子进程：建 2 个视图 → shutdown → 退出，stderr 里不许有告警。"""
        import subprocess
        import sys

        script = (
            "import os, sys\n"
            "os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')\n"
            "from laser_desktop.qt_compat import QtWidgets\n"
            "from laser_desktop import plot_view\n"
            "app = QtWidgets.QApplication(['shutdown-probe'])\n"
            "for _ in range(2):\n"
            "    view = plot_view.PlotView()\n"
            "    view.resize(300, 200)\n"
            "    view.show()\n"
            "    app.processEvents()\n"
            "    view.shutdown()\n"
            "print('done')\n")
        env = dict(os.environ)
        root = str(Path(__file__).resolve().parents[3])
        env["PYTHONPATH"] = root + os.pathsep + os.path.join(root, "python")
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
        result = subprocess.run([sys.executable, "-c", script], env=env,
                                capture_output=True, text=True, timeout=180,
                                check=False)
        self.assertIn("done", result.stdout, result.stderr[-800:])
        self.assertNotIn("not deleted", result.stderr,
                         f"子进程 stderr 里还有告警：\n{result.stderr[-800:]}")


# ---------------------------------------------------------------------------
# P3：1000 点的 Plotly.react 计时（真 WebEngine，离屏）
# ---------------------------------------------------------------------------
class ReactTimingTest(unittest.TestCase):
    """1000 点换图 < 1.5 s（本机实测 60~170 ms；阈值只为防 CI 抖动）。"""

    BUDGET_MS = 1500.0

    @classmethod
    def setUpClass(cls):
        plot_view.webchannel_js_text()          # 触发 QtWebChannel 资源注册
        cls.app = app()

    def _wait_loaded(self, view, timeout_s: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.01)
            if view.loaded:
                return True
        return False

    def _points(self, count: int):
        from laser_web import figures

        rows = [{"gp": index + 1, "index": index,
                 "pos": [900.0 + index, -100.0, 500.0], "ok": True,
                 "j3_abs": 30.0, "j5_abs": 50.0, "config": [0, 1, 0]}
                for index in range(count)]
        return figures.waypoint_figure(rows, all_ok=True)

    def test_thousand_points_react_within_budget(self):
        view = plot_view.PlotView()
        view.resize(900, 600)
        view.show()
        self.addCleanup(view.shutdown)      # P5：先删 page 再删 view
        view.load_shell()
        if not self._wait_loaded(view):
            self.skipTest("WebEngine 壳页 15 s 内没加载完（无头环境限制）")
        measured = {}

        def done(ms):
            measured["ms"] = ms

        view.measure_react(self._points(1000), done, timeout_s=20.0)
        deadline = time.monotonic() + 25.0
        while "ms" not in measured and time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.01)
        self.assertIn("ms", measured, "壳页没有回报 react 耗时")
        self.assertIsNotNone(measured["ms"])
        print(f"\n[P3] 1000 点 Plotly.react = {measured['ms']:.1f} ms")
        self.assertLess(measured["ms"], self.BUDGET_MS)
        self.assertEqual(view.load_count, 1, "换图不该重载壳页")


# ---------------------------------------------------------------------------
# S6 评审 R1：图上高亮只增不减（restyle 就地改写 marker.size）
# ---------------------------------------------------------------------------
class PlotHighlightTest(unittest.TestCase):
    """R1：选 3 → 只有第 3 个点 12；再选 7 → 只有第 7 个 12；清空 → 全部回 5。

    为什么必须在真 WebEngine 里量：BUG 在 ``Plotly.restyle`` 会把
    ``gd.data[t].marker.size`` **就地**改掉，任何纯 Python 断言都看不见。所以
    这里读的是壳页里 ``marker.size`` 数组本身，不是 ``__selectedKeys``（那个
    只说明"以为选了谁"，也正是它让回显与画面不一致）。
    """

    BASE, SELECTED, COUNT = 5, plot_view.SELECT_SIZE, 8

    @classmethod
    def setUpClass(cls):
        plot_view.webchannel_js_text()      # 触发 QtWebChannel 资源注册
        cls.app = app()

    def _figure(self):
        """8 个点、marker.size=5（与 ``laser_web.figures`` 的结果图同口径）。"""
        import plotly.graph_objects as go

        return go.Figure(go.Scatter3d(
            x=[0.0] * self.COUNT, y=[0.0] * self.COUNT, z=[0.0] * self.COUNT,
            mode="markers", marker={"size": self.BASE},
            customdata=[[index + 1] for index in range(self.COUNT)]))

    def _open(self):
        view = plot_view.PlotView()
        view.resize(600, 400)
        view.show()
        self.addCleanup(view.shutdown)
        view.load_shell()
        if not wait_loaded(view):
            self.skipTest("WebEngine 壳页 15 s 内没加载完（无头环境限制）")
        view.set_figure(self._figure())
        sizes = marker_sizes(view, [self.BASE] * self.COUNT, count=self.COUNT)
        self.assertEqual(sizes[:1], [[self.BASE] * self.COUNT],
                         f"基准尺寸不对：{sizes}")
        return view

    def _expect(self, view, keys, wanted, label):
        expected = [self.SELECTED if str(index + 1) in wanted else self.BASE
                    for index in range(self.COUNT)]
        view.select_points(keys)
        sizes = marker_sizes(view, expected, count=self.COUNT)
        self.assertEqual(sizes[:1], [expected],
                         f"{label}：marker.size 数组={sizes[:1]}")
        shown = run_js(view, "window.__selectedKeys()")
        self.assertEqual(json.loads(str(shown or "[]")),
                         [str(key) for key in keys],
                         f"{label}：回显与选择不一致")

    def test_highlight_is_the_only_selected_point_and_clears(self):
        view = self._open()
        self._expect(view, ["3"], {"3"}, "只选 3")
        self._expect(view, ["7"], {"7"}, "改选 7")
        self._expect(view, [], set(), "清空")

    def test_many_selections_do_not_leave_stale_big_points(self):
        """选过 8 行之后只能剩下当前那一个 12（评审实测会剩 8 个）。"""
        view = self._open()
        for index in range(1, self.COUNT + 1):
            self._expect(view, [str(index)], {str(index)}, f"选 {index}")
        sizes = marker_sizes(view, [self.BASE] * self.COUNT, count=self.COUNT)
        self.assertEqual(sizes[0].count(self.SELECTED), 1,
                         f"不只一个点被放大：{sizes[0]}")


# ---------------------------------------------------------------------------
# S6 评审建议 3：壳页没加载时 measure_react 不能让调用方永久等待
# ---------------------------------------------------------------------------
class PendingMeasureTest(unittest.TestCase):
    """``measure_react`` 在加载前被调 → 加载完成后必须回调。"""

    @classmethod
    def setUpClass(cls):
        plot_view.webchannel_js_text()
        cls.app = app()

    def test_callback_arrives_even_if_called_before_load(self):
        import plotly.graph_objects as go

        view = plot_view.PlotView()
        self.addCleanup(view.shutdown)
        box = {}
        # 注意：**先** measure，**后** load_shell（S6 评审建议 3 的复现顺序）
        view.measure_react(go.Figure(go.Scatter3d(x=[0, 1], y=[0, 1], z=[0, 1])),
                           lambda ms: box.setdefault("ms", ms), timeout_s=15.0)
        self.assertFalse(view.loaded)
        view.load_shell()
        deadline = time.monotonic() + 25.0
        while "ms" not in box and time.monotonic() < deadline:
            app().processEvents()
            time.sleep(0.01)
        if not view.loaded:
            self.skipTest("WebEngine 壳页 15 s 内没加载完（无头环境限制）")
        self.assertIn("ms", box, "壳页加载后 callback 永远没来")
        self.assertIsNotNone(box["ms"], "回调给了 None（超时口径）")


# ---------------------------------------------------------------------------
# S6 评审建议 4：桥的载荷校验 + 槽里不能抛异常
# ---------------------------------------------------------------------------
class BridgePayloadTest(unittest.TestCase):
    """坏载荷只忽略、不崩；只留 ``str``/``int`` 键。"""

    def setUp(self):
        app()
        self.view = plot_view.PlotView()
        self.addCleanup(self.view.shutdown)
        self.clicked = []
        self.view.pointClicked.connect(self.clicked.append)

    def test_bad_payloads_are_ignored(self):
        for payload in ("not json", "null", "", "true", "{}", "[null, {}]"):
            self.view._on_bridge_selection(payload)
        self.assertEqual(self.clicked, [], "坏载荷不该产生点击")
        self.assertTrue(self.view.last_error, "坏载荷要记一句话")

    def test_only_str_and_int_keys_are_kept(self):
        self.view._on_bridge_selection(
            '[1, "2", {"x": 1}, null, true, [3], " ", 4.5]')
        self.assertEqual(self.clicked[-1], ["1", "2"],
                         f"过滤结果={self.clicked[-1]}")

    def test_exception_in_a_slot_does_not_break_the_link(self):
        """槽里抛异常：PyQt 自己打印并吃掉，桥不能停在半路。"""
        def boom(_keys):
            raise RuntimeError("页面槽炸了")

        self.view.pointClicked.connect(boom)
        self.view._on_bridge_selection('["5"]')     # 不抛 / 不崩
        self.assertEqual(self.view._selected, ["5"])
        self.view._on_bridge_selection('["6"]')     # 后续照常工作
        self.assertEqual(self.clicked[-1], ["6"])

    def test_unexpected_errors_are_recorded_not_raised(self):
        """连 JSON 解析都炸的时候，也不能把异常扔回 Qt。"""
        with mock.patch.object(plot_view.json, "loads",
                               side_effect=RuntimeError("解析炸了")):
            self.view._on_bridge_selection('["5"]')     # 不抛
        self.assertEqual(self.clicked, [])
        self.assertTrue(self.view.last_error, "异常要记一句话")

    def test_bridge_slot_never_raises_to_qt(self):
        """下游槽炸了，``selectFromPlot`` 也不能把异常交给 Qt。"""
        bridge = self.view._bridge
        bridge.selection.disconnect(self.view._on_bridge_selection)
        self.addCleanup(bridge.selection.connect,
                        self.view._on_bridge_selection)

        def boom(_payload):
            raise RuntimeError("桥下游炸了")

        bridge.selection.connect(boom)
        bridge.selectFromPlot('["5"]')      # 不抛（PyQt 打印后继续）
        self.assertEqual(self.view._selected, [],
                         "下游槽炸了却把状态写脏了")


# ---------------------------------------------------------------------------
# S6 评审 R2 / 建议 2：状态栏不残留、隐藏页不抢状态、切页补回选中
# ---------------------------------------------------------------------------
class _StubHost(QtWidgets.QWidget):
    """假 ``PlotHost``：记下落子的图/键，又不存 WebEngine。"""

    def __init__(self):
        super().__init__()
        self.figures = []
        self.selected = []
        self.created = True
        self.loaded = True
        self.webgl_ok = True
        self.webgl_renderer = "stub"
        self.link_warning = ""

    def set_figure(self, figure):
        self.figures.append(figure)

    def select_points(self, keys, notify: bool = False):
        self.selected.append(list(keys))


class StatusBarTest(unittest.TestCase):
    """R2：常驻信息是永久控件；页面消息带前缀；切页不残留。"""

    def setUp(self):
        from laser_desktop import main_window

        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.settings = QtCore.QSettings(
            str(tmp / "status.ini"), QtCore.QSettings.IniFormat)
        self.window = main_window.MainWindow(settings=self.settings)
        self.window.show()
        wait(120)
        self.addCleanup(self.window.close)

    def tearDown(self):
        self.window.close()
        wait(30)

    def test_resident_info_is_permanent_and_survives_timeouts(self):
        bar = self.window.statusBar()
        label = self.window._status_label
        self.assertIn(BINDING, label.text())
        self.assertIn("数据：", label.text())
        self.assertIn(str(paths.data_root()), label.toolTip())
        # 临时消息 150 ms 就过期（真实是 10 s，只是不能在这里真等）
        bar.showMessage("临时消息", 150)
        wait(400)
        self.assertEqual(bar.currentMessage(), "", "临时消息没有自己清掉")
        self.assertIn("数据：", label.text(),
                      "临时消息超时把常驻信息一起抹掉了")
        self.assertIn("WebGL", label.text())

    def test_page_messages_carry_the_page_prefix(self):
        page = self.window.generate
        page._report_status("已选中 GP0005（图上已放大）")
        self.assertEqual(self.window.statusBar().currentMessage(),
                         "生成：已选中 GP0005（图上已放大）")
        self.window.iso._report_status("ISO 五点完成：run-1")
        self.assertTrue(self.window.statusBar().currentMessage().startswith("ISO："))

    def test_switching_pages_does_not_keep_the_previous_page_message(self):
        window = self.window
        generate = window.generate
        generate.render_result(scan_result(8))
        generate.waypoint_table.setCurrentCell(4, 0)
        wait(60)
        self.assertIn("GP0005", window.statusBar().currentMessage())
        window.nav.setCurrentRow(2)                 # 切到 ISO
        wait(120)
        message = window.statusBar().currentMessage()
        self.assertNotIn("GP0005", message, f"上一页的状态栏残留：{message!r}")
        if message:
            self.assertTrue(message.startswith("ISO："), message)

    def test_tables_rows_are_restored_on_the_plot_when_switching_back(self):
        """R2c：切回来时表格当前行必须重新点亮到图上。"""
        window = self.window
        stub = _StubHost()
        window.plot = stub
        for page in (window.generate, window.repair, window.iso):
            page._plot = stub
        window.nav.setCurrentRow(0)
        page = window.generate
        page.render_result(scan_result(8))
        page.waypoint_table.setCurrentCell(2, 0)    # 第 3 行
        wait(60)
        self.assertEqual(stub.selected[-1], ["3"])
        stub.selected.clear()
        window.nav.setCurrentRow(1)                 # 切走
        window.nav.setCurrentRow(0)                 # 切回
        wait(60)
        self.assertEqual(stub.selected[-1], ["3"],
                         "切回本页后图上的高亮没补回来")
        self.assertEqual(page.waypoint_table.currentRow(), 2)


class HiddenPageSelectionTest(unittest.TestCase):
    """建议 2：隐藏页的 ``select_key`` 不图上画、也不改状态栏。"""

    def test_hidden_page_does_nothing(self):
        from laser_desktop import iso_page

        plot = StubPlot()
        page = iso_page.IsoPage(plot=plot)
        self.addCleanup(page.deleteLater)
        page.render_result(iso_result(5))
        page.set_active(False)
        page.last_status = ""
        plot.selected.clear()
        page.point_table.setCurrentCell(1, 0)       # P2
        wait(30)
        self.assertEqual(plot.selected, [], "隐藏页还在往图上推高亮")
        self.assertEqual(page.last_status, "", "隐藏页还在写状态栏")

        page.set_active(True)
        page.point_table.setCurrentCell(2, 0)       # P3
        wait(30)
        self.assertEqual(plot.selected[-1], ["P3"], "当前页应该照常联动")

    def test_one_click_runs_three_slots_but_only_the_visible_page_speaks(self):
        """三页 connect 同一个 ``pointClicked``：只有当前页能动。"""
        from laser_desktop import main_window

        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        settings = QtCore.QSettings(str(tmp / "click.ini"),
                                    QtCore.QSettings.IniFormat)
        window = main_window.MainWindow(settings=settings)
        self.addCleanup(window.close)
        window.show()
        wait(120)
        window.nav.setCurrentRow(0)
        window.generate.render_result(scan_result(8))
        window.repair.render_plan(scan_result(8))
        window.iso.render_result(iso_result(5))
        wait(60)
        window.iso.last_status = ""                 # 隐藏页本来没说话
        window.plot.pointClicked.emit(["5"])        # 三个槽都会跑
        wait(60)
        message = window.statusBar().currentMessage()
        self.assertTrue(message.startswith("生成："),
                        f"一次点击后状态栏={message!r}")
        self.assertIn("GP0005", message)
        # 评审的复现：隐藏页被直接调 ``select_key`` 不许改写可见页的状态栏
        before = window.statusBar().currentMessage()
        window.iso.select_key("P2")
        wait(60)
        self.assertEqual(window.iso.last_status, "",
                         "隐藏的 ISO 页被写上了状态")
        self.assertEqual(window.statusBar().currentMessage(), before,
                         "隐藏页改写了可见页的状态栏")


class FigurePushTest(unittest.TestCase):
    """建议 1：同一张图不重复推；空页推空图（壳页重新显示「还没有数据」）。"""

    def setUp(self):
        from laser_desktop import main_window

        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.settings = QtCore.QSettings(str(tmp / "push.ini"),
                                         QtCore.QSettings.IniFormat)
        self.window = main_window.MainWindow(settings=self.settings)
        self.stub = _StubHost()
        self.window.plot = self.stub
        for page in (self.window.generate, self.window.repair, self.window.iso):
            page._plot = self.stub
        self.window.show()
        wait(80)
        self.addCleanup(self.window.close)

    def tearDown(self):
        self.window.close()
        wait(30)

    def test_the_same_figure_is_pushed_only_once(self):
        window = self.window
        window.nav.setCurrentRow(0)
        window.generate.render_result(scan_result(8))
        window.nav.setCurrentRow(1)
        window.nav.setCurrentRow(0)
        window.nav.setCurrentRow(1)
        window.nav.setCurrentRow(0)
        self.stub.figures.clear()
        window._last_pushed = window.generate.current_figure()
        window._show_page_figure(window.generate)   # 同一张图：不该再推
        self.assertEqual(self.stub.figures, [],
                         "同一张图被重推了一次（react 会重置相机/高亮）")

    def test_a_page_without_figure_gets_an_empty_one(self):
        """没出过图的页要推空图，而不是把上一页的图留在那里。"""
        window = self.window
        window.nav.setCurrentRow(0)
        window.generate.last_figure = None          # 一张图都还没出过
        window._last_pushed = None
        self.stub.figures.clear()
        window._show_page_figure(window.generate)
        self.assertEqual(self.stub.figures, [page_layout.empty_figure()],
                         "没出过图的页没推空图")
        self.assertEqual(len(page_layout.empty_figure().data), 0)

    def test_plot_view_skips_the_same_figure(self):
        """判重做在 PlotView 里：页面的 ``_push_figure`` 也走这一道。"""
        view = plot_view.PlotView()
        self.addCleanup(view.shutdown)
        view.loaded = True                          # 不真起壳页
        calls = []
        view._run_figure = lambda figure, token: calls.append((figure, token))
        figure = page_layout.empty_figure()
        view.set_figure(figure)
        view.set_figure(figure)
        view.set_figure(page_layout.empty_figure())  # 同一个单例也算同一张
        self.assertEqual(len(calls), 1, f"同一张图推了 {len(calls)} 次")


class EmptyFigureShellTest(unittest.TestCase):
    """建议 1：空图推下去之后，壳页的「还没有数据」要重新看得见。"""

    @classmethod
    def setUpClass(cls):
        plot_view.webchannel_js_text()
        cls.app = app()

    def test_empty_figure_restores_the_hint_and_purges_the_plot(self):
        import plotly.graph_objects as go

        view = plot_view.PlotView()
        view.resize(500, 320)
        view.show()
        self.addCleanup(view.shutdown)
        view.load_shell()
        if not wait_loaded(view):
            self.skipTest("WebEngine 壳页 15 s 内没加载完（无头环境限制）")
        view.set_figure(go.Figure(go.Scatter3d(x=[0, 1], y=[0, 1], z=[0, 1],
                                              customdata=[[1], [2]])))
        self.assertEqual(str(run_js(view, "window.__emptyShown()")), "hidden")
        view.set_figure(page_layout.empty_figure())
        deadline = time.monotonic() + 5.0
        shown = ""
        while time.monotonic() < deadline:
            shown = str(run_js(view, "window.__emptyShown()"))
            if shown == "shown":
                break
            self.app.processEvents()
            time.sleep(0.02)
        self.assertEqual(shown, "shown", "空图之后「还没有数据」没回来")
        self.assertEqual(str(run_js(view, "window.__state()")), "empty")


if __name__ == "__main__":
    unittest.main()
