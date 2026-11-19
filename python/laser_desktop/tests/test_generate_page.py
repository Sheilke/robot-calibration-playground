#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成页（S2）：表单 -> core 参数、总点数口径、默认值、超上限与非法输入。

纯逻辑（``form_to_params`` / ``total_points``）不需要 Qt；GUI 部分全部在
offscreen 下跑（建 ``QApplication`` 后直接实例化 ``GeneratePage``，不 ``show()``，
也不建 ``QWebEngineView``——``plot=None`` 时图只算好放在 ``last_figure`` 里）。

口径对齐的证据在这里：手填模式的"直算公式"与 ``core.preview_domain`` 的
``point_count`` 逐组比对（网页版 ``_total_points_text`` 用的是同一套规则）。
"""
import math
import os
import time
import unittest
from pathlib import Path
from unittest import mock

# 必须在建 QApplication 之前：pytest 命令行不一定带 QT_QPA_PLATFORM
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from excitation_trajectory import aabb_snake_scan as scan
from laser_desktop import generate_page, paths
from laser_desktop.qt_compat import QtCore, QtWidgets
from laser_web import core, ui_state

REPO = Path(__file__).resolve().parents[3]

_APP = None


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-tests"])
    return _APP


def wait(ms: int) -> None:
    """真跑 ``ms`` 毫秒事件循环（去抖测试用）。

    ``QTest.qWait`` 的等价替代：``laser_desktop.qt_compat`` 不导出 ``QtTest``
    （页面代码只认那一个入口），所以这里用 ``processEvents`` + ``sleep`` 自己转
    事件循环。offscreen 下两者行为一致：到期的 ``QTimer`` 照常触发。
    """
    deadline = time.monotonic() + ms / 1000.0
    while True:
        app().processEvents()
        if time.monotonic() >= deadline:
            return
        time.sleep(0.005)


class StubPlot(QtWidgets.QWidget):
    """只记下被推过来的图（真 PlotView 要 WebEngine，不在这里建）。"""

    def __init__(self):
        super().__init__()
        self.figures = []

    def set_figure(self, figure):
        self.figures.append(figure)


# ---------------------------------------------------------------------------
# 纯逻辑：表单 -> core 参数
# ---------------------------------------------------------------------------
class FormToParamsTest(unittest.TestCase):
    """``form_to_params`` 只搬运 + 类型归一，校验仍交给 core。"""

    def test_explicit_vertices_become_segments(self):
        """界面按格点数填，算法要份数：4 格点 -> 3 份，core 能接受。"""
        params = generate_page.form_to_params(
            {"grid_mode": "explicit", "xs": 4, "ys": 4, "zs": 4, "z_insert": 0})
        self.assertEqual((params["xs"], params["ys"], params["zs"]), (3, 3, 3))
        self.assertEqual(params["z_insert"], 0)
        self.assertNotIn("target_count", params)

        result = core.preview_domain(params)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["point_count"], 64)
        self.assertEqual(result["grid"],
                         {"xs": 3, "ys": 3, "zs": 3, "z_insert": 0})

    def test_target_count_mode_leaves_the_grid_to_core(self):
        """目标点数模式下不提交 xs/ys/zs，网格由 choose_grid 凑。"""
        params = generate_page.form_to_params(
            {"grid_mode": "count", "target_count": 50})
        self.assertEqual(params["target_count"], 50)
        for key in ("xs", "ys", "zs", "z_insert"):
            self.assertNotIn(key, params)

        result = core.preview_domain(params)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["point_count"], 50)
        # 实测值（CLI 打印"自动凑数: --target-count 50 -> xs=1 ys=4 zs=4"）
        self.assertEqual(result["grid"],
                         {"xs": 1, "ys": 4, "zs": 4, "z_insert": 0})

    def test_default_form_is_accepted_by_core(self):
        """网页版默认值（2×5×5 格点 = 50 点）必须原样能进 core。"""
        params = generate_page.form_to_params(ui_state.default_form_values())
        result = core.preview_domain(params)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["point_count"], 50)
        self.assertEqual(result["grid"],
                         {"xs": 1, "ys": 4, "zs": 4, "z_insert": 0})

    def test_only_the_active_domain_group_is_submitted(self):
        """两点/六数互斥：只提交当前激活那一组，否则 core 会报"不能同时给"。"""
        points = generate_page.form_to_params({"domain_mode": "points"})
        self.assertIn("point_min", points)
        self.assertIn("point_max", points)
        self.assertNotIn("aabb6", points)
        self.assertTrue(core.preview_domain(points)["ok"])

        axes = generate_page.form_to_params({"domain_mode": "axes"})
        self.assertIn("aabb6", axes)
        self.assertNotIn("point_min", axes)
        self.assertNotIn("point_max", axes)
        self.assertTrue(core.preview_domain(axes)["ok"])

    def test_guard_fields_reach_core(self):
        """守卫/段检查/q-seed 也要送到 core（S2 只搬运，S3 才用）。"""
        params = generate_page.form_to_params({"j5_min": 25.0,
                                               "j5_attempts": 8,
                                               "seg_mode": "joint",
                                               "q_seed": "1,2,3,4,5,6"})
        for key in ("j5_min", "j3_min", "j5_attempts", "seg_min", "seg_mode",
                    "seg_samples", "seed", "q_seed", "cond_max"):
            self.assertIn(key, params)
        self.assertEqual(params["j5_min"], 25.0)
        self.assertEqual(params["j5_attempts"], 8)
        self.assertEqual(params["seg_mode"], "joint")
        self.assertEqual(params["q_seed"], "1,2,3,4,5,6")
        # core 真的认这个键（不是搬到一半被 form_to_namespace 丢掉）
        self.assertEqual(core.form_to_namespace(params).j5_attempts, 8)

    def test_params_do_not_alias_the_input_lists(self):
        """``values()`` 里的 list 是"现场快照"，params 里必须是新 list。"""
        values = generate_page.form_defaults()
        params = generate_page.form_to_params(values)
        params["rpy"][0] = 123.0
        self.assertNotEqual(values["rpy"][0], 123.0)


# ---------------------------------------------------------------------------
# 纯逻辑：总点数（与网页版 _total_points_text 同口径）
# ---------------------------------------------------------------------------
class TotalPointsTest(unittest.TestCase):
    """手填模式直接按公式算，凑数模式读 core。"""

    def test_explicit_grid_formula(self):
        total = generate_page.total_points(
            {"grid_mode": "explicit", "xs": 4, "ys": 4, "zs": 4,
             "z_insert": 0})
        self.assertEqual(total.count, 64)
        self.assertFalse(total.over)
        self.assertIn("64 点", total.text)
        self.assertIn("4×4×4 格点", total.text)

    def test_insert_points_are_counted_per_layer(self):
        """4×4×4 格点 + 每层插 1 个 = 64 + 3（换层 3 次）。"""
        total = generate_page.total_points(
            {"grid_mode": "explicit", "xs": 4, "ys": 4, "zs": 4,
             "z_insert": 1})
        self.assertEqual(total.count, 67)
        self.assertIn("67 点", total.text)
        self.assertIn("+ 3 插点", total.text)

    def test_target_count_mode_reads_core(self):
        total = generate_page.total_points(
            {"grid_mode": "count", "target_count": 50})
        self.assertEqual(total.count, 50)
        self.assertIn("50 点", total.text)
        self.assertIn("2×5×5 格点", total.text)

    def test_target_count_that_core_cannot_reach_shows_dash(self):
        """凑不出来的目标只显示 —（不瞎猜一个数字），也谈不上超限。"""
        total = generate_page.total_points(
            {"grid_mode": "count", "target_count": 6000})
        self.assertIsNone(total.count)
        self.assertEqual(total.text, "—")
        self.assertFalse(total.over)

    def test_explicit_formula_equals_core_point_count(self):
        """口径一致：直算公式 == core 的 grid_point_count（网页版也读这个值）。"""
        for vertices in ((2, 5, 5), (4, 4, 4), (3, 7, 2), (1, 1, 1)):
            for insert in (0, 1, 3):
                values = {"grid_mode": "explicit", "xs": vertices[0],
                          "ys": vertices[1], "zs": vertices[2],
                          "z_insert": insert}
                total = generate_page.total_points(values)
                result = core.preview_domain(
                    generate_page.form_to_params(values))
                self.assertTrue(result["ok"], result)
                self.assertEqual(total.count, result["point_count"],
                                 f"{vertices} + 插点 {insert}")
                self.assertEqual(f"{total.count} 点", f"{result['point_count']} 点")

    def test_over_limit_is_flagged(self):
        total = generate_page.total_points(
            {"grid_mode": "explicit", "xs": 200, "ys": 200, "zs": 200,
             "z_insert": 0})
        self.assertEqual(total.count, 8_000_000)
        self.assertTrue(total.over)

    def test_point_count_warn_matches_the_web_page(self):
        """桌面版的上限常量必须与网页版一致（不 import nicegui，只读源码）。"""
        source = (REPO / "python" / "laser_web" / "pages" / "__init__.py"
                  ).read_text(encoding="utf-8")
        self.assertIn(f"POINT_COUNT_WARN = {generate_page.POINT_COUNT_WARN}",
                      source)


# ---------------------------------------------------------------------------
# GUI（offscreen，不 show()）
# ---------------------------------------------------------------------------
class PageTest(unittest.TestCase):
    """每个用例一个全新的页面（表单状态互不污染）。"""

    def setUp(self):
        app()
        self.page = generate_page.GeneratePage(plot=None)

    def tearDown(self):
        self.page.deleteLater()
        self.page = None


class WidgetDefaultsTest(PageTest):
    """控件默认值来自 ``ui_state.default_form_values()``，不是手抄的数字。"""

    def test_values_match_ui_state(self):
        defaults = ui_state.default_form_values()
        values = self.page.values()
        self.assertEqual(values["domain_mode"], defaults["domain_mode"])
        self.assertEqual(values["grid_mode"], defaults["grid_mode"])
        self.assertEqual(values["point_min"], list(defaults["point_min"]))
        self.assertEqual(values["point_max"], list(defaults["point_max"]))
        self.assertEqual(values["aabb6"], list(defaults["aabb6"]))
        self.assertEqual(values["rpy"], list(defaults["rpy"]))
        self.assertEqual(values["euler"], defaults["euler"])
        self.assertEqual(values["perturb_deg"], list(defaults["perturb_deg"]))
        self.assertEqual(values["cond_max"], defaults["cond_max"])
        self.assertEqual(values["target_count"], defaults["target_count"])
        self.assertEqual(values["robot"], defaults["robot"])

    def test_grid_boxes_hold_vertex_counts_not_segments(self):
        """界面填的是格点数：默认 2/5/5（份数是 1/4/4）。"""
        self.assertEqual(self.page.fields["xs"].value(), 2)
        self.assertEqual(self.page.fields["ys"].value(), 5)
        self.assertEqual(self.page.fields["zs"].value(), 5)
        self.assertEqual(self.page.fields["z_insert"].value(), 0)
        self.assertEqual(ui_state.vertices_to_segments(2),
                         scan.DEFAULT_GRID[0])

    def test_guard_defaults_come_from_the_cli(self):
        args = scan.build_arg_parser().parse_args([])
        values = self.page.values()
        self.assertEqual(values["j5_min"], args.j5_min)
        self.assertEqual(values["j3_min"], args.j3_min)
        self.assertEqual(values["j5_attempts"], args.j5_attempts)
        self.assertEqual(values["seg_min"], args.seg_min)
        self.assertEqual(values["seg_samples"], args.seg_samples)
        self.assertEqual(values["seed"], args.seed)
        self.assertEqual(values["q_seed"], args.q_seed)
        # 段模型刻意跟网页版走 MOVL，而不是 CLI 的 joint
        self.assertEqual(values["seg_mode"], generate_page.DEFAULT_SEG_MODE)

    def test_initial_preview_is_the_fifty_point_sample(self):
        self.assertEqual(self.page.total.count, 50)
        self.assertEqual(self.page.metric_labels["edges"].text(),
                         "0.550 × 1.200 × 0.670 m")
        self.assertEqual(self.page.metric_labels["grid"].text(), "2×5×5")
        self.assertTrue(self.page.generate_button.isEnabled())


class RobotComboTest(PageTest):
    """机器人模型下拉：只列 ``usable`` 的，其余折叠成一句提示。"""

    def test_lists_only_usable_models(self):
        combo = self.page.fields["robot"]
        listed = [combo.itemData(index) for index in range(combo.count())]
        # 页面用 paths.asset_root() 查（打包后也指得到模型）；源码运行时它与
        # core 的默认仓库根完全等价，这里把两边的结果都钉住
        for robots in (core.list_robots(paths.asset_root()), core.list_robots()):
            expected = sorted(robot["path"] for robot in robots
                              if robot["usable"])
            self.assertEqual(sorted(listed), expected)
        self.assertEqual(combo.currentData(), ui_state.default_robot())

    def test_skipped_files_are_summarised(self):
        robots = [
            {"label": "good.urdf", "path": "data/configs/good.urdf",
             "kind": "urdf", "usable": True, "reason": None},
            {"label": "dh.json", "path": "data/configs/dh.json",
             "kind": "json", "usable": False, "reason": "KeyError: Link&CoupleCoe"},
            {"label": "scan.json", "path": "data/configs/scan.json",
             "kind": "json", "usable": False, "reason": "KeyError: joints"},
        ]
        page = generate_page.GeneratePage(plot=None, robots=robots)
        self.assertEqual(page.fields["robot"].count(), 1)
        self.assertEqual(page.fields["robot"].itemData(0),
                         "data/configs/good.urdf")
        self.assertEqual(page.skipped_label.text(), "另有 2 个文件不能当模型用")
        self.assertIn("KeyError", page.skipped_label.toolTip())


class EulerComboTest(PageTest):
    """欧拉约定下拉：值仍是算法约定名，文案是中文「内旋」。"""

    def test_labels_are_chinese_and_values_are_algorithm_names(self):
        combo = self.page.fields["euler"]
        values = [combo.itemData(index) for index in range(combo.count())]
        labels = [combo.itemText(index) for index in range(combo.count())]
        self.assertEqual(tuple(values), tuple(scan.EULER_CONVENTIONS))
        for label in labels:
            self.assertNotIn("intrinsic", label)
            self.assertIn("内旋", label)
        self.assertEqual(combo.currentData(),
                         ui_state.default_form_values()["euler"])
        self.assertEqual(self.page.metric_labels["euler"].text(),
                         ui_state.euler_label(combo.currentData()))


class OverLimitTest(PageTest):
    """超过 ``POINT_COUNT_WARN``：数字变红 + 禁用「生成并解算」。"""

    def test_explicit_over_limit_disables_generate(self):
        page = self.page
        page.fields["grid_explicit"].setChecked(True)
        for key in ("xs", "ys", "zs"):
            page.fields[key].setValue(200)

        self.assertEqual(page.total.count, 8_000_000)
        self.assertTrue(page.total.over)
        self.assertFalse(page.generate_button.isEnabled())
        self.assertIn("5000", page.total_hint.text())
        self.assertFalse(page.total_hint.isHidden())
        self.assertIn("5000", page.over_label.text())
        self.assertIn("5000", page.metric_labels["points"].text()
                      + page.over_label.text())
        self.assertIn("c0392b", page.total_value.styleSheet())

    def test_target_count_spin_max_is_what_choose_grid_can_reach(self):
        """目标点数上限 = ``choose_grid`` 真能凑出的最大点数（不抄数字）。

        份数上限 12（13 个格点）+ 单次插点上限 10 决定公式：
        ``13³ + 12×10 = 2317``；再往上就是「保证报错」的区间。夹到上限之后
        这个值本身必须真能凑出来（``ok is True``），否则上限也就白设了。
        """
        page = self.page
        limit = generate_page.TARGET_COUNT_MAX
        self.assertEqual(
            limit,
            (scan.COUNT_GRID_MAX_PER_AXIS + 1) ** 3
            + scan.COUNT_GRID_MAX_PER_AXIS * scan.COUNT_GRID_MAX_INSERT)
        self.assertEqual(page.fields["target_count"].maximum(), limit)

        page.fields["grid_count"].setChecked(True)
        page.fields["target_count"].setValue(limit + 1000)   # 被夹到上限
        self.assertEqual(page.fields["target_count"].value(), limit)
        result = page.preview_result()
        self.assertIs(result["ok"], True, result)
        self.assertEqual(result["point_count"], limit)
        self.assertEqual(page.total.count, limit)
        self.assertTrue(page.generate_button.isEnabled())

    def test_back_under_the_limit_reenables_generate(self):
        page = self.page
        page.fields["xs"].setValue(200)
        page.fields["ys"].setValue(200)
        self.assertTrue(page.total.over)
        self.assertFalse(page.generate_button.isEnabled())
        page.fields["xs"].setValue(2)
        page.fields["ys"].setValue(5)
        self.assertTrue(page.generate_button.isEnabled())
        self.assertTrue(page.total_hint.isHidden())


class InvalidInputTest(PageTest):
    """非法输入：预览卡片显示 core 的报错原文，不弹窗、不抛异常。"""

    def test_inverted_bounds_show_the_core_error_verbatim(self):
        page = self.page
        page.fields["domain_axes"].setChecked(True)
        page.fields["x_min"].setValue(2000.0)          # 上界还是 1300

        result = page.preview_result()
        self.assertFalse(result["ok"])
        self.assertFalse(page.error_label.isHidden())
        self.assertEqual(page.error_label.text(), result["error"])
        self.assertIn("大于上界", page.error_label.text())
        self.assertFalse(page.generate_button.isEnabled())
        # 预览卡片的数值全部退回 —；网格组的"总点数"仍按公式直算（与网页版一致：
        # 域非法不影响"这组格点会生成多少点"这件事本身）
        for key in ("edges", "grid", "points", "design", "euler"):
            self.assertEqual(page.metric_labels[key].text(), "—", key)
        self.assertEqual(page.total.count, 50)
        self.assertEqual(page.total_value.text(), "50 点（2×5×5 格点）")

    def test_fixing_the_input_clears_the_error(self):
        page = self.page
        page.fields["domain_axes"].setChecked(True)
        page.fields["x_min"].setValue(2000.0)
        self.assertTrue(page.error_label.text())
        page.fields["x_min"].setValue(750.0)
        self.assertTrue(page.error_label.isHidden())
        self.assertTrue(page.generate_button.isEnabled())
        self.assertEqual(page.metric_labels["edges"].text(),
                         "0.550 × 1.200 × 0.670 m")

    def test_zero_perturbation_is_rejected_by_core(self):
        """扰动半宽 0 是界面填得出来的值，判定仍归 core（不在这里写规则）。"""
        page = self.page
        page.fields["perturb_a"].setValue(0.0)
        result = page.preview_result()
        self.assertFalse(result["ok"])
        self.assertIn("perturb-deg", result["error"])
        self.assertEqual(page.error_label.text(), result["error"])


class RefreshAndFigureTest(PageTest):
    """任何输入变化都要重算预览；图跟着表单走（只规划，不跑 IK）。"""

    def test_switching_domain_swaps_the_panel_and_the_params(self):
        page = self.page
        self.assertEqual(page.domain_stack.currentIndex(), 0)
        self.assertEqual(page.values()["domain_mode"], "points")
        page.fields["domain_axes"].setChecked(True)
        self.assertEqual(page.domain_stack.currentIndex(), 1)
        self.assertEqual(page.values()["domain_mode"], "axes")
        self.assertIn("aabb6", page.params())
        page.fields["domain_points"].setChecked(True)
        self.assertEqual(page.domain_stack.currentIndex(), 0)
        self.assertIn("point_min", page.params())

    def test_switching_grid_swaps_the_panel_and_the_params(self):
        page = self.page
        page.fields["grid_count"].setChecked(True)
        self.assertEqual(page.grid_stack.currentIndex(), 0)
        self.assertIn("target_count", page.params())
        page.fields["grid_explicit"].setChecked(True)
        self.assertEqual(page.grid_stack.currentIndex(), 1)
        self.assertIn("xs", page.params())

    def test_preview_recomputes_on_any_change(self):
        """改扰动半宽 -> 设计条件数必须变（证明信号真的连上了）。"""
        page = self.page
        before = page.metric_labels["design"].text()
        page.fields["perturb_a"].setValue(30.0)
        after = page.metric_labels["design"].text()
        self.assertNotEqual(before, after)
        self.assertEqual(
            after, f"{page.preview_result()['design_condition']:.3f}")

    def test_figure_follows_the_grid(self):
        page = self.page
        page.fields["xs"].setValue(4)
        page.fields["ys"].setValue(4)
        page.fields["zs"].setValue(4)
        figure = page.apply_figure()
        self.assertEqual(page.total.count, 64)
        self.assertEqual(len(figure.data[0].x), 64)
        self.assertIn("64 个点", figure.layout.title.text)
        self.assertIn("未跑 IK", figure.layout.title.text)

    def test_over_limit_figure_is_a_placeholder(self):
        """超上限时不去 plan_waypoints 里冒险（800 万点会吃穿内存）。"""
        page = self.page
        for key in ("xs", "ys", "zs"):
            page.fields[key].setValue(200)
        figure = page.apply_figure()
        self.assertEqual(len(figure.data), 0)
        self.assertIn("5000", figure.layout.title.text
                      + " ".join(str(item.text) for item in figure.layout.annotations))

    def test_invalid_params_figure_explains_the_error(self):
        page = self.page
        page.fields["domain_axes"].setChecked(True)
        page.fields["x_min"].setValue(2000.0)
        figure = page.apply_figure()
        self.assertEqual(len(figure.data), 0)
        self.assertIn("大于上界", figure.layout.annotations[0].text)

    def test_figure_update_is_debounced(self):
        """拖动 spinbox 时不能每个刻度都重规划一次：真跑事件循环验证。

        连续 5 次改值（间隔 50 ms < 300 ms 去抖窗口）→ 只应该多出一张图；
        而且最后那张图的点数必须等于**最终**表单值（不是中间某一次的）。
        """
        page = self.page
        self.assertEqual(generate_page.DEBOUNCE_MS, 300)
        self.assertEqual(page._debounce.interval(), 300)
        self.assertTrue(page._debounce.isSingleShot())

        plot = StubPlot()
        own_page = generate_page.GeneratePage(plot=plot)
        try:
            wait(generate_page.DEBOUNCE_MS + 100)      # 吃掉首次预览的定时器
            baseline = len(plot.figures)
            for value in (2, 3, 4, 5, 6):              # 最终 xs = 6
                own_page.fields["xs"].setValue(value)
                wait(50)
            wait(generate_page.DEBOUNCE_MS + 100)

            self.assertEqual(len(plot.figures) - baseline, 1,
                             f"去抖失效：等了 400 ms 却收到 "
                             f"{len(plot.figures) - baseline} 张图")
            # 6×5×5 = 150：最后一张图用的是最终表单值，不是中途的值
            self.assertEqual(len(plot.figures[-1].data[0].x), 150)
            self.assertEqual(own_page.total.count, 150)
        finally:
            own_page.deleteLater()
            plot.deleteLater()

    def test_figure_is_pushed_into_the_plot_widget(self):
        """挂了 PlotView 时 ``apply_figure`` 要把图推过去（冒烟脚本靠这条）。"""
        plot = StubPlot()
        page = generate_page.GeneratePage(plot=plot)
        try:
            page.apply_figure()
            self.assertEqual(len(plot.figures), 1)
            self.assertEqual(len(plot.figures[-1].data[0].x), 50)
        finally:
            page.deleteLater()
            plot.deleteLater()

    def test_generate_button_no_longer_advertises_s3(self):
        """S3 已接入：按钮文案不再是占位提示，结果区装备到位、任务未跑时为空闲。"""
        self.assertEqual(self.page.generate_button.text(), "生成并解算")
        self.assertTrue(self.page.cancel_button.isHidden())
        self.assertEqual(self.page.progress_bar.format(), "空闲")
        self.assertEqual(self.page.result_tabs.count(), 3)
        self.assertIsNone(self.page.last_result)


class PreviewFailureTest(PageTest):
    """R1：``core.preview_domain`` 抛异常不能打断 Qt 槽（界面不能停在旧状态）。

    根因（零长度轴让 ``choose_grid`` 的 ``max/min`` 除零）已在算法层单独修（
    ``aabb_snake_scan.py`` 的格距只统计 ``length > 0`` 的轴），所以这里用
    ``mock`` 注入同一个 ``ZeroDivisionError``，钉住界面层的兜底：异常 → 卡片红字
    → 按钮禁用 → **不抛**。同时保留一条真实零长度轴的用例，证明修好的算法现在
    能直接接受这种域。
    """

    def _zero_length_x_axis_with_target_count(self):
        """把表单改成：六个轴边界 + 目标点数 + X 轴 min == max。"""
        page = self.page
        page.fields["domain_axes"].setChecked(True)
        page.fields["grid_count"].setChecked(True)
        page.fields["x_max"].setValue(page.fields["x_min"].value())
        return page

    def test_exception_becomes_a_card_error_and_disables_generate(self):
        page = self._zero_length_x_axis_with_target_count()
        boom = ZeroDivisionError("float division by zero")
        with mock.patch.object(core, "preview_domain", side_effect=boom):
            result = page.refresh_preview()           # 不能抛

        self.assertIs(result["ok"], False)
        self.assertEqual(
            result["error"],
            "预览失败：ZeroDivisionError: float division by zero")
        self.assertEqual(page.error_label.text(), result["error"])
        self.assertFalse(page.error_label.isHidden())
        self.assertTrue(page.error_label.text())
        self.assertIs(page.preview_result()["ok"], False)
        self.assertFalse(page.generate_button.isEnabled())
        for key in ("edges", "grid", "points", "design", "euler"):
            self.assertEqual(page.metric_labels[key].text(), "—", key)
        self.assertIsNone(page.total.count)
        self.assertEqual(page.total.text, "—")

    def test_the_other_two_call_sites_are_guarded_too(self):
        """``total_points`` 与 ``build_preview_figure`` 里的调用也要包住。"""
        page = self._zero_length_x_axis_with_target_count()
        values = page.values()
        boom = ZeroDivisionError("float division by zero")
        with mock.patch.object(core, "preview_domain", side_effect=boom):
            total = generate_page.total_points(values)          # preview=None
            figure = generate_page.build_preview_figure(values)
            page.refresh_preview()
            plotted = page.apply_figure()

        self.assertIsNone(total.count)
        self.assertEqual(total.text, "—")
        self.assertIn("预览失败：ZeroDivisionError",
                      figure.layout.annotations[0].text)
        self.assertIn("预览失败：ZeroDivisionError",
                      plotted.layout.annotations[0].text)

    def test_fixing_the_domain_goes_back_to_normal(self):
        """对照：异常消失后改回合法域，卡片恢复、按钮重新可用。"""
        page = self._zero_length_x_axis_with_target_count()
        with mock.patch.object(core, "preview_domain",
                               side_effect=ZeroDivisionError("boom")):
            self.assertIs(page.refresh_preview()["ok"], False)
            self.assertFalse(page.generate_button.isEnabled())
        page.fields["x_max"].setValue(1300.0)            # 上界 > 下界：合法
        result = page.refresh_preview()
        self.assertIs(result["ok"], True, result)
        self.assertTrue(page.error_label.isHidden())
        self.assertTrue(page.generate_button.isEnabled())

    def test_zero_length_axis_is_accepted_after_the_algorithm_fix(self):
        """根因修好后，零长度轴 + 目标点数本身就是合法参数（不会抛、能出图）。"""
        page = self._zero_length_x_axis_with_target_count()
        result = page.preview_result()
        self.assertIs(result["ok"], True, result)
        self.assertEqual(result["grid"]["xs"], 0)        # X 轴退化成一条线
        self.assertTrue(page.generate_button.isEnabled())
        self.assertEqual(len(page.apply_figure().data), 1)


class SingularPoseTest(PageTest):
    """R3：奇异参考姿态的 ``inf`` 要显示成 ``∞`` 并说明原因。"""

    def test_singular_reference_pose_shows_infinity_with_a_reason(self):
        page = self.page
        page.fields["rpy_b"].setValue(90.0)              # 内旋 XYZ、pitch = 90°
        result = page.preview_result()
        self.assertIs(result["ok"], True, result)
        self.assertTrue(math.isinf(result["design_condition"]))
        text = page.metric_labels["design"].text()
        self.assertIn("∞", text)
        self.assertIn("奇异", text)                      # 明说不是程序 bug
        self.assertNotIn("inf", text)

    def test_format_condition(self):
        self.assertEqual(generate_page.format_condition(float("inf")),
                         "∞（该参考姿态奇异）")
        self.assertEqual(generate_page.format_condition(float("nan")), "—")
        self.assertEqual(generate_page.format_condition(12.3456), "12.346")


class J5AttemptsTest(PageTest):
    """R4：网页版有「重采上限」，桌面版也要把它送到 core。"""

    def test_default_comes_from_the_cli(self):
        args = scan.build_arg_parser().parse_args([])
        field = self.page.fields["j5_attempts"]
        self.assertEqual((field.minimum(), field.maximum()), (1, 100))
        self.assertEqual(field.value(), args.j5_attempts)

    def test_edited_value_reaches_form_to_params(self):
        page = self.page
        page.fields["j5_attempts"].setValue(7)
        self.assertEqual(page.values()["j5_attempts"], 7)
        self.assertEqual(page.params()["j5_attempts"], 7)
        self.assertEqual(
            generate_page.form_to_params(page.values())["j5_attempts"], 7)
        # 进 core 后确实变成 --j5-attempts（不是搬了个没人读的键）
        self.assertEqual(core.form_to_namespace(page.params()).j5_attempts, 7)
        self.assertIs(page.preview_result()["ok"], True, page.preview_result())


class PlanFailureReportingTest(PageTest):
    """R5：``plan_waypoints`` 抛异常不能只变成一张图，还要进日志/状态栏。"""

    def _boom(self):
        return mock.patch.object(generate_page, "waypoint_figure",
                                 side_effect=RuntimeError("规划炸了"))

    def test_failure_is_logged_with_a_stack(self):
        with self._boom():
            with self.assertLogs("laser_desktop.generate_page",
                                 level="ERROR") as logs:
                figure = self.page.apply_figure()
        self.assertIn("规划失败：RuntimeError: 规划炸了",
                      figure.layout.annotations[0].text)
        joined = "\n".join(logs.output)
        self.assertIn("路点规划失败", joined)
        self.assertIn("RuntimeError: 规划炸了", joined)
        self.assertIn("Traceback", joined)               # exception() 带了栈

    def test_failure_reaches_the_window_status_bar(self):
        window = QtWidgets.QMainWindow()
        page = generate_page.GeneratePage()
        window.setCentralWidget(page)
        try:
            with self._boom():
                with self.assertLogs("laser_desktop.generate_page",
                                     level="ERROR"):
                    page.apply_figure()
            message = window.statusBar().currentMessage()
            self.assertIn("路点规划失败", message)
            self.assertIn("RuntimeError: 规划炸了", message)
        finally:
            page.deleteLater()
            window.deleteLater()

    def test_failure_without_a_window_only_logs(self):
        """没有 QMainWindow（纯逻辑测试）时只记日志，不报错。"""
        with self._boom():
            with self.assertLogs("laser_desktop.generate_page",
                                 level="ERROR"):
                self.page.apply_figure()             # 不抛就是通过


class GridCaptionTest(PageTest):
    """R8：提示文案里的公式必须带 (Z 格点数−1) 这个因子。"""

    def test_caption_states_the_real_formula(self):
        caption = self.page.grid_caption.text()
        self.assertIn("Z 格点数−1", caption)
        self.assertIn("换层插点", caption)
        # 数字口径与文案一致：4×4×4 + (4−1)×1 = 67
        total = generate_page.total_points(
            {"grid_mode": "explicit", "xs": 4, "ys": 4, "zs": 4,
             "z_insert": 1})
        self.assertEqual(total.count, 67)


class BuildPreviewFigureTest(unittest.TestCase):
    """``build_preview_figure`` 是冒烟脚本用的入口，参数从表单来。"""

    def test_default_is_the_fifty_point_sample(self):
        figure = generate_page.build_preview_figure()
        self.assertEqual(len(figure.data[0].x), 50)

    def test_follows_the_given_form_values(self):
        values = generate_page.form_defaults()
        values.update({"xs": 3, "ys": 3, "zs": 3})
        figure = generate_page.build_preview_figure(values)
        self.assertEqual(len(figure.data[0].x), 27)

    def test_invalid_values_give_a_placeholder(self):
        values = generate_page.form_defaults()
        values["domain_mode"] = "axes"
        values["aabb6"] = [1300.0, 750.0, -600.0, 600.0, 270.0, 940.0]
        figure = generate_page.build_preview_figure(values)
        self.assertEqual(len(figure.data), 0)


if __name__ == "__main__":
    unittest.main()
