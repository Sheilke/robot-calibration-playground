#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S6c：主操作按钮的固定行 + 预览卡按内容拿高度（三页 × 两种窗口尺寸）。

现场的三张截图（1600×950 与 1280×800）暴露了两件事：

* **主操作按钮被裁**：修复页「定位段」「修复」被预览卡的上边缘切掉半截
  （生成页「生成并解算」也是半截），ISO 页「生成五点」在表单滚动区里滚出去；
* **预览卡内容看不到**：`边长` 之后，`格点 / 总点数 / 设计条件数 / 欧拉约定`
  全部要滚动才出现——而「设计条件数」正是判断"可行 / 不可行"的依据。

根因是同一个：按钮和数值都住在**可滚动的预览卡**里，卡片又被
``PREVIEW_MIN_HEIGHT=50`` 压到只剩一条。S6c 的改法：

* **A1**：三页的主操作按钮搬进 ``page.action_row``（滚动区之外、
  ``QSizePolicy.Fixed``、不设 ``maximumHeight``），1600×950 与 1280×800 下
  都必须 ``visibleRegion() == rect()`` 且 ``height() >= sizeHint()``；
* **A2**：预览卡首开高度 = 内容 ``sizeHint`` 夹在 [150, 220]
  （:func:`laser_desktop.page_layout.preview_default_height`），1600×950 下
  五项数值**无滚动**可见；
* **A3**：1600×950 图 ≥260 / 表 8 行；1280×800 图 ≥200 / 表 ≥5 行。

**为什么用真 ``MainWindow`` 量**：S6b 的教训是"单元测试里够、现场不够"。
``MainWindow`` 有菜单栏与状态栏，页面拿到的可用高度比"裸窗口装一页"少 30 px
（1600×950 实测：裸窗口 934 / 真窗口 904）——正是那 30 px 决定了 8 行表与
预览卡能不能同时成立。所以这里一律用 ``main_window.MainWindow`` 量，和现场
截图、冒烟脚本同一口径。
"""
import os
import tempfile
import time
import unittest
from pathlib import Path

# 必须在建 QApplication 之前：pytest 命令行不一定带 QT_QPA_PLATFORM
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import main_window, page_layout
from laser_desktop.qt_compat import QtCore, QtWidgets
from laser_desktop.tests.test_s6_common import iso_result, scan_result

_APP = None

#: 导航下标（``main_window.PAGES`` 的顺序）。
NAV_ROW = {"generate": 0, "repair": 1, "iso": 2}

#: 预览卡里的五项数值：生成 / 修复 = 边长…欧拉约定；ISO = 立方体…截面。
METRIC_KEYS = {
    "generate": ("edges", "grid", "points", "design", "euler"),
    "repair": ("edges", "grid", "points", "design", "euler"),
    "iso": ("cube", "apothem", "center", "layout", "plane"),
}

#: 1600×950 下每页要露出的行数（ISO 只有 5 个点）。
DEFAULT_ROWS = {"generate": 8, "repair": 8, "iso": 5}

#: 1280×800 下至少要露出的行数（P6 的下限）。
MINIMUM_ROWS = 5


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-s6c"])
    return _APP


def setUpModule():
    """任何 QWidget 之前先建 QApplication（否则 Qt 直接 abort）。"""
    app()


def wait(ms: int) -> None:
    """真跑 ``ms`` 毫秒事件循环（布局要至少两轮才能量尺寸）。"""
    deadline = time.monotonic() + ms / 1000.0
    while True:
        app().processEvents()
        if time.monotonic() >= deadline:
            return
        time.sleep(0.005)


def visible_rows(table) -> int:
    """表格视口里**完整**可见的行数（口径与 ``test_s6_common`` 一致）。"""
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


def fully_visible(widget) -> bool:
    """控件在屏幕上**完整**可见：可见 + 可见区域 == 整个 rect + 高度 ≥ sizeHint。"""
    if not widget.isVisible():
        return False
    region = widget.visibleRegion().boundingRect()
    return (region == widget.rect()
            and widget.height() >= widget.sizeHint().height())


def buttons_of(page) -> dict:
    """当前页的主操作按钮（S6c 只认这三个）。"""
    if hasattr(page, "locate_button"):
        return {"定位段": page.locate_button, "修复": page.repair_button}
    return {page.generate_button.text(): page.generate_button}


def metric_labels_of(page) -> dict:
    """预览卡里的五项数值。"""
    if hasattr(page, "preview_labels"):
        return dict(page.preview_labels)
    return dict(page.metric_labels)


def result_table_of(page):
    """当前页的结果表 + 应该点开的页签。"""
    if page is None:                       # pragma: no cover - 防御
        raise AssertionError("没有页面")
    if hasattr(page, "point_table"):
        return page.point_table, page.points_page
    if hasattr(page, "plan_page"):
        return page.waypoint_table, page.plan_page
    return page.waypoint_table, page.waypoints_page


def render_result_into(page, kind):
    """给页面灌一份与 ``core`` 同构的**结果**（含告警，跟真跑一次同一个路径）。

    修复页走 ``render_result`` 而不是 ``render_plan``：真跑一次之后「计划」页签
    除了表还有统计卡与 core 的告警行，那些行是高度预算里真实存在的一部分。
    只拿“最干净”的渲染去量，就会得出“测试里 8 行、现场 7 行”的结论。
    """
    if kind == "iso":
        page.render_result(iso_result(5))
        return
    result = scan_result(8)
    if kind == "repair":
        result["warnings"] = [
            "自动凑数: --target_count 8 -> xs=1 ys=1 zs=1 z_insert=0，"
            "格点 8 + 插点 0 = 8 点"]
        result["repair"] = {"mode": "none", "ok": True}
        page.render_result(result)
        return
    page.render_result(result)


class _MainWindowCase(unittest.TestCase):
    """公共夹具：临时 ``QSettings`` + 一个真 ``MainWindow``。"""

    def setUp(self):
        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.settings = QtCore.QSettings(
            str(tmp / "s6c.ini"), QtCore.QSettings.IniFormat)

    def open_page(self, kind, size, render: bool = True):
        """开一个 ``MainWindow``，切到 ``kind`` 页、按 ``size`` 摆好；返回 (窗口, 页)。"""
        window = main_window.MainWindow(settings=self.settings)
        window.resize(*size)
        window.show()
        wait(240)
        window.nav.setCurrentRow(NAV_ROW[kind])
        wait(160)
        page = window.stack.currentWidget()
        table, tab = result_table_of(page)
        page.result_tabs.setCurrentWidget(tab)
        wait(120)
        if render:
            render_result_into(page, kind)
            wait(300)
            # 渲染会把页签切到“最该看的”那一页（修复页是「定位」）：这里再切回
            # 带表的页签，否则量到的是隐藏表的 0 行。
            page.result_tabs.setCurrentWidget(tab)
            wait(300)
        return window, page

    def assert_window_size(self, window, size, label):
        """窗口没被布局撑大（撑大就说明三段/minimum 装不下这一档）。"""
        self.assertEqual((window.width(), window.height()), tuple(size),
                         f"{label}：窗口被撑到 {window.width()}×"
                         f"{window.height()}（要求 {size[0]}×{size[1]}）")

    def preview_scroll(self, page):
        """预览卡外面那层滚动区。"""
        return page._viewer_splitter.widget(0)


# ---------------------------------------------------------------------------
# A1：主操作按钮在固定操作行里，两种尺寸下都完整可见
# ---------------------------------------------------------------------------
class ActionRowTest(_MainWindowCase):
    """A1：按钮 ``visibleRegion() == rect()`` 且 ``height() >= sizeHint()``。"""

    def _check(self, kind, size, label):
        window, page = self.open_page(kind, size)
        self.assert_window_size(window, size, label)
        row = page.action_row
        self.assertEqual(row.objectName(), "action_row")
        self.assertFalse(
            self._inside_scroll_area(row),
            f"{label}：操作行还在滚动区里（会跟着滚出去）")
        self.assertEqual(
            row.sizePolicy().verticalPolicy(), QtWidgets.QSizePolicy.Fixed,
            f"{label}：操作行垂直策略不是 Fixed（会被压缩）")
        self.assertGreaterEqual(row.height(), row.sizeHint().height(),
                                f"{label}：操作行被压到 {row.height()} px")
        for text, button in buttons_of(page).items():
            self.assertTrue(button.isVisible(), f"{label}：{text} 不可见")
            self.assertGreaterEqual(
                button.height(), button.sizeHint().height(),
                f"{label}：{text} 高 {button.height()} < sizeHint "
                f"{button.sizeHint().height()}（被 setMaximumHeight 压扁了）")
            self.assertEqual(
                button.visibleRegion().boundingRect(), button.rect(),
                f"{label}：{text} 只露出 "
                f"{button.visibleRegion().boundingRect()}（整块 {button.rect()}）")
            self.assertTrue(fully_visible(button), f"{label}：{text} 不完整")
        window.close()

    @staticmethod
    def _inside_scroll_area(widget) -> bool:
        """沿父链找 ``QAbstractScrollArea``：按钮在滚动区里就会跟着滚。"""
        parent = widget.parentWidget()
        while parent is not None:
            if isinstance(parent, QtWidgets.QAbstractScrollArea):
                return True
            parent = parent.parentWidget()
        return False

    # ---- 1600×950 ----
    def test_generate_page_at_default_size(self):
        self._check("generate", page_layout.DEFAULT_WINDOW_SIZE, "生成页 1600×950")

    def test_repair_page_at_default_size(self):
        self._check("repair", page_layout.DEFAULT_WINDOW_SIZE, "修复页 1600×950")

    def test_iso_page_at_default_size(self):
        self._check("iso", page_layout.DEFAULT_WINDOW_SIZE, "ISO 页 1600×950")

    # ---- 1280×800 ----
    def test_generate_page_at_minimum_size(self):
        self._check("generate", page_layout.MINIMUM_WINDOW_SIZE, "生成页 1280×800")

    def test_repair_page_at_minimum_size(self):
        self._check("repair", page_layout.MINIMUM_WINDOW_SIZE, "修复页 1280×800")

    def test_iso_page_at_minimum_size(self):
        self._check("iso", page_layout.MINIMUM_WINDOW_SIZE, "ISO 页 1280×800")

    def test_no_page_keeps_the_action_button_inside_the_preview_card(self):
        """三页的预览卡里都不许再放主操作按钮（S6c 的问题根源）。"""
        window, page = self.open_page("generate", page_layout.DEFAULT_WINDOW_SIZE)
        for kind in ("generate", "repair", "iso"):
            other = getattr(window, kind)
            card = other._viewer_splitter.preview_card
            for button in buttons_of(other).values():
                self.assertFalse(card.isAncestorOf(button),
                                 f"{kind}：主操作按钮还在预览卡里")
        window.close()


# ---------------------------------------------------------------------------
# A2：预览卡首开高度 = 内容高度（150–220），1600×950 下五项数值不用滚
# ---------------------------------------------------------------------------
class PreviewCardHeightTest(_MainWindowCase):
    """A2：卡片放得下内容；「设计条件数」在首屏。"""

    def test_default_height_is_the_content_height_within_bounds(self):
        """``preview_default_height``：夹在 [150, 220]，不是 50。"""
        self.assertEqual(page_layout.PREVIEW_MIN_HEIGHT, 100)
        self.assertEqual(page_layout.PREVIEW_DEFAULT_MIN, 150)
        self.assertEqual(page_layout.PREVIEW_DEFAULT_MAX, 220)
        self.assertEqual(page_layout.preview_default_height(None), 150)

        class _Fake(QtWidgets.QWidget):
            def __init__(self, height):
                super().__init__()
                self._height = height

            def sizeHint(self):              # noqa: N802 - Qt 覆写
                return QtCore.QSize(200, self._height)

        self.assertEqual(page_layout.preview_default_height(_Fake(20)), 150)
        self.assertEqual(page_layout.preview_default_height(_Fake(183)), 183)
        self.assertEqual(page_layout.preview_default_height(_Fake(400)), 220)

    def test_first_open_uses_the_content_height(self):
        """首开（还没出结果）：预览卡拿到 150–220，不是 50。"""
        window, page = self.open_page("generate",
                                      page_layout.DEFAULT_WINDOW_SIZE,
                                      render=False)
        scroll = self.preview_scroll(page)
        self.assertGreaterEqual(scroll.height(),
                                page_layout.PREVIEW_DEFAULT_MIN,
                                f"首开预览卡只有 {scroll.height()} px")
        self.assertLessEqual(scroll.height(),
                             page_layout.PREVIEW_DEFAULT_MAX + 1)
        self.assertEqual(scroll.verticalScrollBar().maximum(), 0)
        window.close()

    def _check(self, kind, label):
        window, page = self.open_page(kind, page_layout.DEFAULT_WINDOW_SIZE)
        self.assert_window_size(window, page_layout.DEFAULT_WINDOW_SIZE, label)
        splitter = page._viewer_splitter
        scroll = self.preview_scroll(page)
        card = splitter.preview_card
        labels = metric_labels_of(page)
        # A2 的硬判据（两条）：五项数值整块可见 + 卡片不需要滚动。
        for key in METRIC_KEYS[kind]:
            widget = labels[key]
            self.assertEqual(
                widget.visibleRegion().boundingRect(), widget.rect(),
                f"{label}：{key} 没完整显示（要滚动才看得到）")
        self.assertEqual(
            scroll.verticalScrollBar().maximum(), 0,
            f"{label}：预览卡还要竖向滚 "
            f"{scroll.verticalScrollBar().maximum()} px"
            f"（卡片 {scroll.height()}，内容 {card.sizeHint().height()}）")
        self.assertEqual(
            scroll.horizontalScrollBar().maximum(), 0,
            f"{label}：预览卡要横向滚（内容最小宽 "
            f"{card.minimumSizeHint().width()} > {scroll.viewport().width()}）")
        # 卡片不能比内容矮：矮了就是藏东西
        self.assertGreaterEqual(
            scroll.height(), min(page_layout.PREVIEW_DEFAULT_MIN,
                                 card.minimumSizeHint().height()),
            f"{label}：预览卡 {scroll.height()} < 内容 "
            f"{card.minimumSizeHint().height()}（会藏东西）")
        self.assertGreaterEqual(scroll.height(), page_layout.PREVIEW_MIN_HEIGHT,
                                f"{label}：预览卡跌破硬下限")
        self.assertLessEqual(scroll.height(),
                             page_layout.PREVIEW_DEFAULT_MAX + 1,
                             f"{label}：预览卡超过上限 220")
        window.close()

    def test_generate_page_shows_all_five_metrics_without_scrolling(self):
        self._check("generate", "生成页 1600×950")

    def test_repair_page_shows_all_five_metrics_without_scrolling(self):
        self._check("repair", "修复页 1600×950")

    def test_iso_page_shows_all_five_metrics_without_scrolling(self):
        self._check("iso", "ISO 页 1600×950")

    def test_preview_card_may_scroll_at_the_minimum_window(self):
        """1280×800 允许预览卡滚动（这时按钮已经在操作行里，不受影响）。"""
        window, page = self.open_page("iso", page_layout.MINIMUM_WINDOW_SIZE)
        scroll = self.preview_scroll(page)
        self.assertGreaterEqual(scroll.height(),
                                page_layout.PREVIEW_MIN_HEIGHT)
        self.assertTrue(fully_visible(page.generate_button))
        window.close()


# ---------------------------------------------------------------------------
# A3：三段 + 表的可见行数，两种尺寸各钉一遍
# ---------------------------------------------------------------------------
class ThreeSegmentsTest(_MainWindowCase):
    """A3：1600×950 图 ≥260 / 表 8 行；1280×800 图 ≥200 / 表 ≥5 行。"""

    def test_default_ratios_fold_to_a_workable_split(self):
        """``DEFAULT_SPLIT_RATIOS`` 在 922 px 可用高度下三段都"够用"。"""
        window, page = self.open_page("generate",
                                      page_layout.DEFAULT_WINDOW_SIZE,
                                      render=False)
        splitter = page._viewer_splitter
        self.assertEqual(tuple(splitter.ratios),
                         page_layout._normalize(
                             page_layout.DEFAULT_SPLIT_RATIOS))
        # 1600×950 的裸窗口里分割器可用高度实测 934（934 - 2×6 条手柄 = 922）
        sizes = splitter.default_sizes(922 + 2 * splitter.handleWidth())
        self.assertGreaterEqual(sizes[0], page_layout.PREVIEW_DEFAULT_MIN, sizes)
        self.assertGreaterEqual(sizes[1], page_layout.PLOT_MIN_HEIGHT, sizes)
        self.assertGreaterEqual(sizes[2], page_layout.RESULT_MIN_HEIGHT, sizes)
        self.assertEqual(sum(sizes), 922, sizes)
        # 折算比例（16/38/46）与真实分配一致（±10 千分比）
        folded = [int(round(size * page_layout.RATIO_SCALE / 922))
                  for size in sizes]
        for actual, expected in zip(folded, splitter.ratios):
            self.assertAlmostEqual(float(actual), float(expected), delta=10.0,
                                   msg=f"{folded} vs {splitter.ratios}")
        window.close()

    def _check(self, kind, size, label, want_rows):
        window, page = self.open_page(kind, size)
        self.assert_window_size(window, size, label)
        preview, plot, results = page._viewer_splitter.sizes()
        table, _tab = result_table_of(page)
        rows = visible_rows(table)
        if size == page_layout.DEFAULT_WINDOW_SIZE:
            self.assertGreaterEqual(
                plot, page_layout.PLOT_MIN_HEIGHT,
                f"{label}：图区只有 {plot} px（分割 {[preview, plot, results]}）")
            self.assertGreaterEqual(
                results, page_layout.RESULT_MIN_HEIGHT,
                f"{label}：结果区只有 {results} px")
            self.assertGreaterEqual(
                preview, page_layout.PREVIEW_MIN_HEIGHT,
                f"{label}：预览卡只有 {preview} px")
        else:
            self.assertGreaterEqual(
                plot, page_layout.PLOT_FLOOR_HEIGHT,
                f"{label}：图区只有 {plot} px（硬下限 "
                f"{page_layout.PLOT_FLOOR_HEIGHT}，分割 "
                f"{[preview, plot, results]}）")
        self.assertGreaterEqual(
            rows, want_rows,
            f"{label}：只看到 {rows} 行（要求 {want_rows}；表高 "
            f"{table.height()}，分割 {[preview, plot, results]}）")
        # 两种尺寸下按钮都必须完整（A1 的第一优先级）
        for text, button in buttons_of(page).items():
            self.assertTrue(fully_visible(button),
                            f"{label}：{text} 不完整")
        window.close()

    # ---- 1600×950 ----
    def test_generate_at_default_size(self):
        self._check("generate", page_layout.DEFAULT_WINDOW_SIZE,
                    "生成页 1600×950", DEFAULT_ROWS["generate"])

    def test_repair_at_default_size(self):
        self._check("repair", page_layout.DEFAULT_WINDOW_SIZE,
                    "修复页 1600×950", DEFAULT_ROWS["repair"])

    def test_iso_at_default_size(self):
        self._check("iso", page_layout.DEFAULT_WINDOW_SIZE,
                    "ISO 页 1600×950", DEFAULT_ROWS["iso"])

    # ---- 1280×800 ----
    def test_generate_at_minimum_size(self):
        self._check("generate", page_layout.MINIMUM_WINDOW_SIZE,
                    "生成页 1280×800", MINIMUM_ROWS)

    def test_repair_at_minimum_size(self):
        self._check("repair", page_layout.MINIMUM_WINDOW_SIZE,
                    "修复页 1280×800", MINIMUM_ROWS)

    def test_iso_at_minimum_size(self):
        self._check("iso", page_layout.MINIMUM_WINDOW_SIZE,
                    "ISO 页 1280×800", MINIMUM_ROWS)

    def test_minimum_window_needs_no_extra_height(self):
        """1280×800 下三档同时成立：窗口最小高度 ≤ 800（装不下就会自己变大）。"""
        window, _page = self.open_page("repair",
                                       page_layout.MINIMUM_WINDOW_SIZE)
        hint = window.minimumSizeHint()
        self.assertLessEqual(
            hint.height(), page_layout.MINIMUM_WINDOW_SIZE[1],
            f"窗口最小高度 {hint.height()} 超过 800（三段装不下）")
        window.close()


class UserMovedSplitterTest(_MainWindowCase):
    """R3：用户拖过分割条之后，``protect_plot`` 不许再把高度抢回去。

    评审实测（1600×950）：置 ``_user_moved=True`` 再 ``setSizes([300,150,452])``，
    立刻变成 ``[216,260,416]``——图区抬回 260，预览卡被扣 84。
    """

    def test_protect_plot_respects_user_moved(self):
        window, page = self.open_page("repair", page_layout.DEFAULT_WINDOW_SIZE)
        splitter = page._viewer_splitter
        total = sum(splitter.sizes())
        results_min = max(page_layout.RESULT_MIN_HEIGHT,
                          splitter.widget(2).minimumSizeHint().height())
        # 图区 210：高于硬下限 200（PlotSlot 的最小高度），低于设计下限 260
        target = [total - results_min - 210, 210, results_min]
        splitter._user_moved = True           # 评审探针就是这么做的
        splitter.setSizes(target)
        wait(300)
        self.assertTrue(splitter.user_moved)
        sizes = splitter.sizes()
        self.assertLess(
            sizes[1], page_layout.PLOT_MIN_HEIGHT,
            f"用户拖完后图区被抬回 {sizes[1]}（分割 {sizes}，目标 {target}）")
        self.assertGreaterEqual(
            sizes[0], target[0] - 8,
            f"预览卡被扣掉高度（分割 {sizes}，目标 {target}）")
        self.assertEqual(splitter.plot_floor(), page_layout.PLOT_FLOOR_HEIGHT,
                         "拖过之后 plot_floor 还按 260 算")
        self.assertFalse(splitter.protect_plot(),
                         "拖过之后 protect_plot 还在抢高度")
        window.close()

    def test_menu_style_ratio_restore_still_protects_plot(self):
        """没拖过的分割器照旧护图区（R3 的修法不能把原行为也删了）。"""
        window, page = self.open_page("generate", page_layout.DEFAULT_WINDOW_SIZE)
        splitter = page._viewer_splitter
        self.assertFalse(splitter.user_moved)
        total = sum(splitter.sizes())
        splitter._applying = True             # 程序自己的 setSizes（不算用户拖）
        try:
            splitter.setSizes([150, 150, total - 300])
        finally:
            splitter._applying = False
        wait(300)
        self.assertFalse(splitter.user_moved)
        self.assertGreaterEqual(splitter.sizes()[1],
                                page_layout.PLOT_MIN_HEIGHT,
                                f"没拖过却没护图区：{splitter.sizes()}")
        window.close()


# ---------------------------------------------------------------------------
# 建议 6：FlowRow 的高度要按当前宽度算
# ---------------------------------------------------------------------------
class FlowRowHeightTest(unittest.TestCase):
    """``height_for_width`` 数换行数；两个 ``preview_*_height`` 用它。"""

    KEYS = tuple((f"k{index}", f"名称{index}") for index in range(6))

    def _grid(self):
        grid = page_layout.MetricGrid({}, self.KEYS)
        self.addCleanup(grid.deleteLater)
        return grid

    def test_height_for_width_counts_the_rows(self):
        grid = self._grid()
        grid.resize(600, 30)
        wide = grid.height_for_width(600)
        narrow = grid.height_for_width(90)
        self.assertEqual(wide, grid.ITEM_HEIGHT, "宽到一行就该只有一行高")
        self.assertGreater(narrow, wide, f"窄宽度没算出换行：{narrow}")
        self.assertGreaterEqual(narrow, len(self.KEYS) * grid.ITEM_HEIGHT)
        # 宽度为 0（还没排过）时按一行算，不能抛
        self.assertEqual(grid.height_for_width(0), grid.ITEM_HEIGHT)

    def test_preview_heights_use_the_current_width(self):
        box = QtWidgets.QGroupBox("预览")
        self.addCleanup(box.deleteLater)
        layout = QtWidgets.QVBoxLayout(box)
        grid = page_layout.MetricGrid({}, self.KEYS)
        layout.addWidget(grid)
        box.resize(600, 300)
        box.show()
        wait(60)
        wide = page_layout.preview_default_height(box)
        self.assertEqual(page_layout.flow_height_extra(box), 0,
                         "宽度够时不该有换行补偿")

        # 冻住 relayout：模拟“宽度变了、卡片还没按新宽度排完”的那一刻
        # （评审量到 181 > 156 就是这个中间态）。
        grid.relayout = lambda: 0
        box.resize(120, 300)
        wait(60)
        extra = page_layout.flow_height_extra(box)
        self.assertGreater(extra, 0, "窄宽度没算出换行补偿")
        self.assertEqual(
            page_layout.preview_default_height(box),
            max(page_layout.PREVIEW_DEFAULT_MIN,
                min(page_layout.PREVIEW_DEFAULT_MAX,
                    box.sizeHint().height() + extra)))
        self.assertGreater(page_layout.preview_default_height(box), wide)


if __name__ == "__main__":
    unittest.main()


# ---------------------------------------------------------------------------
# 导航列：能拖宽、不能拖没、宽度记得住
# ---------------------------------------------------------------------------
class NavWidthTest(_MainWindowCase):
    """用户要求：最左侧边栏要能拖宽；拖窄时不许整个消失。

    这一条踩过两次：先是 ``setFixedWidth(92)``（往右拖没反应、字体一大就切字），
    改成 ``setMinimumWidth`` 后默认宽度又变成 ``QListWidget`` 的 256 px（导航右
    边空出一条白、表单和图各少拿 164 px）。所以断言要同时钉住三件事：默认贴紧、
    可拖宽、拖到 0 仍停在最低宽度。
    """

    def _window(self):
        window = main_window.MainWindow(settings=self.settings)
        window.resize(1600, 950)
        window.show()
        wait(200)
        return window

    def test_default_width_fits_the_longest_item(self):
        """默认宽度 = ``max(NAV_WIDTH, 最长项宽)``，不是 QListWidget 的 256 px。

        注意不是「等于最长项」：96 dpi 下最长项只有 74，而下限是 92，所以实际
        取 92（与旧版视觉一致，不因这一项改动而变）。要守住的是「不小于最长项」
        ——小了就切字。
        """
        window = self._window()
        needed = max(
            window.nav.sizeHintForIndex(window.nav.model().index(row, 0)).width()
            for row in range(window.nav.count()))
        padding = 2 * int(window.nav.frameWidth())
        expected = max(int(page_layout.NAV_WIDTH), needed + padding)
        self.assertEqual(window.nav.width(), expected,
                         f"默认导航宽 {window.nav.width()} != 期望 {expected}"
                         "（说明 QListWidget 的 sizeHint 又生效了）")
        self.assertGreaterEqual(window.nav.width(), needed,
                                "导航比最长项还窄，会切字")
        self.assertEqual(window._nav_splitter.sizes()[0], window.nav.width(),
                         f"分割器槽位 {window._nav_splitter.sizes()} 与导航宽不符")
        window.close()

    def test_dragging_widens_the_nav(self):
        """往右拖要真的变宽，而且第二次还能继续拖（没被重新钉死）。"""
        window = self._window()
        splitter = window._nav_splitter
        for width in (200, 300):
            total = sum(splitter.sizes())
            splitter.setSizes([width, total - width])
            wait(60)
            window._on_nav_moved(width, 1)
            wait(60)
            self.assertEqual(window.nav.width(), width,
                             f"拖到 {width} 后导航宽 {window.nav.width()}")
        window.close()

    def test_cannot_be_dragged_away(self):
        """拖到 0 时停在最低宽度，控件仍可见（用户找得到怎么切回去）。"""
        window = self._window()
        splitter = window._nav_splitter
        floor = window.nav.minimumWidth()
        splitter.setSizes([0, sum(splitter.sizes())])
        wait(80)
        self.assertGreaterEqual(window.nav.width(), floor)
        self.assertTrue(window.nav.isVisible())
        window.close()

    def test_width_is_remembered(self):
        """拖宽后重开窗口，宽度还是上次那个。"""
        window = self._window()
        splitter = window._nav_splitter
        total = sum(splitter.sizes())
        splitter.setSizes([280, total - 280])
        wait(60)
        window._on_nav_moved(280, 1)
        wait(60)
        window.close()

        again = self._window()
        self.assertEqual(again.nav.width(), 280,
                         f"重开后导航宽 {again.nav.width()}（应为 280）")
        again.close()

    def test_too_small_stored_width_falls_back_to_floor(self):
        """换字体/DPI 后旧宽度可能比新下限还小 —— 那时用下限，不许切字。"""
        self.settings.setValue("nav_width", 10)
        window = self._window()
        self.assertGreaterEqual(window.nav.width(), window.nav.minimumWidth())
        window.close()
