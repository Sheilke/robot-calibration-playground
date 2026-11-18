#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 五点页（S5）纯逻辑：表单 -> core 参数、两种立方体来源、布点联动。

对齐的基准是网页版 ``laser_web/pages/iso9283.py::collect()``：本文件把它的键
集合**逐个写死**（``WEB_COMMON_KEYS`` / ``WEB_BOUNDS_KEYS`` / ``WEB_AUTO_KEYS``），
再断言 :func:`laser_desktop.iso_page.form_to_params` 的产物与它**完全同集合**
——两种 ``cube_mode`` 只提交激活那一组（``cube`` XOR ``search_box`` +
``resolution`` + ``prefer_x_min``）。

另外钉死网页版踩过的那类坑（commit ``32136c6``）：初始化时写死一个不存在的
模式名，会让两组输入同时隐藏、页面看起来“没有输入框”。桌面版要求：

1. 初始化后**始终恰有一组可见**；
2. 两种模式各切 2 次，可见组仍然恰好 1 个；
3. 切走再切回，之前填的数值还在（隐藏 ≠ 清空）。
"""

import os
import sys
import unittest

from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import iso_page
from laser_desktop.qt_compat import QtWidgets
from laser_web import core, ui_state
from excitation_trajectory import iso9283_cube as iso

_APP = None


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-iso-form-tests"])
    return _APP


#: 网页版 ``collect()`` 两种模式都会提交的键。
WEB_COMMON_KEYS = frozenset({"robot", "cube_mode", "layout", "plane",
                             "square_frac", "rpy", "euler", "q_seed", "j5_min"})
#: ``cube_mode="bounds"``（网页版叫「给最大/最小点」）额外提交的键。
WEB_BOUNDS_KEYS = WEB_COMMON_KEYS | {"cube"}
#: ``cube_mode="auto"``（自动搜索）额外提交的键。
WEB_AUTO_KEYS = WEB_COMMON_KEYS | {"search_box", "resolution", "prefer_x_min"}


def visible_panels(page) -> list[int]:
    """当前可见的立方体来源面板下标（``isVisibleTo`` 忽略窗口是否显示）。"""
    stack = page.mode_stack
    return [index for index in range(stack.count())
            if stack.widget(index).isVisibleTo(stack)]


class FormToParamsTest(unittest.TestCase):
    """表单值 -> ``core.run_iso9283`` 的入参（两种 cube_mode 各一组）。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def test_defaults_come_from_core(self):
        """默认值全部来自 ``core.iso_defaults()``，界面不抄数字。"""
        defaults = core.iso_defaults()
        page_defaults = iso_page.iso_form_defaults()
        for key in ("layout", "plane", "square_frac", "rpy", "euler",
                    "q_seed", "j5_min", "resolution", "prefer_x_min"):
            self.assertEqual(page_defaults[key], defaults[key], key)
        # 两点输入的示例值来自网页版示例域（与「生成」页同源）
        self.assertEqual(page_defaults["cube"][0],
                         float(ui_state.SAMPLE_POINT_MIN[0]))
        self.assertEqual(page_defaults["cube"][1],
                         float(ui_state.SAMPLE_POINT_MAX[0]))
        # 打开页面时的实际表单值与默认值一致
        self.assertEqual(self.page.params(), iso_page.form_to_params(page_defaults))

    def test_bounds_params_key_set(self):
        """两点输入只提交 ``cube``，不带自动搜索的三个键。"""
        params = self.page.params()
        self.assertEqual(set(params), set(WEB_BOUNDS_KEYS))
        self.assertNotIn("search_box", params)
        self.assertNotIn("resolution", params)
        self.assertNotIn("prefer_x_min", params)
        self.assertEqual(params["cube_mode"], "bounds")
        self.assertEqual(params["cube"],
                         [float(value) for value in
                          iso_page.iso_form_defaults()["cube"]])

    def test_auto_params_key_set(self):
        """自动搜索只提交 ``search_box`` / ``resolution`` / ``prefer_x_min``。"""
        self.page.fields["cube_auto"].setChecked(True)
        params = self.page.params()
        self.assertEqual(set(params), set(WEB_AUTO_KEYS))
        self.assertNotIn("cube", params)
        self.assertEqual(params["cube_mode"], "auto")
        self.assertEqual(params["search_box"],
                         [float(value) for value in
                          iso_page.SAMPLE_SEARCH_BOX])
        self.assertEqual(params["resolution"],
                         float(core.iso_defaults()["resolution"]))
        self.assertIsNone(params["prefer_x_min"], "0 号位是「不限」= core 的 None")

    def test_params_values_match_widgets(self):
        """逐键核对：提交值就是控件里那一版（不是默认值）。"""
        fields = self.page.fields
        fields["cube_bounds"].setChecked(True)
        for key, value in (("x0", 900.0), ("x1", 1100.0), ("y0", -100.0),
                           ("y1", 100.0), ("z0", 500.0), ("z1", 700.0),
                           ("j5_min", 20.0)):
            fields[key].setValue(value)
        fields["layout"].setCurrentIndex(fields["layout"].findData("circle"))
        fields["plane"].setCurrentIndex(fields["plane"].findData("yz"))
        fields["rpy_a"].setValue(3.0)
        fields["rpy_b"].setValue(73.0)
        fields["rpy_c"].setValue(-167.0)
        fields["euler"].setCurrentIndex(fields["euler"].findData("zyx-intrinsic"))
        fields["q_seed"].setText("43,-30,-36,42,-89,-1")

        params = self.page.params()
        self.assertEqual(params["cube"], [900.0, 1100.0, -100.0, 100.0,
                                          500.0, 700.0])
        self.assertEqual(params["layout"], "circle")
        self.assertEqual(params["plane"], "yz")
        self.assertEqual(params["rpy"], [3.0, 73.0, -167.0])
        self.assertEqual(params["euler"], "zyx-intrinsic")
        self.assertEqual(params["q_seed"], "43,-30,-36,42,-89,-1")
        self.assertEqual(params["j5_min"], 20.0)
        self.assertEqual(params["square_frac"],
                         float(fields["square_frac"].value()))

    def test_prefer_x_min_is_nullable(self):
        """「中心 X 不小于」留空 = 不限（``None``）；0 是**合法值**，不当哨兵。

        早先把 ``0`` 当“不限”（``setSpecialValueText``），于是“中心 X ≥ 0”
        这个合法要求根本没法表达——那种用法现在得到 ``0.0``。
        """
        self.page.fields["cube_auto"].setChecked(True)
        field = self.page.fields["prefer_x_min"]
        field.setText("")
        self.assertIsNone(self.page.params()["prefer_x_min"], "空 = 不限")
        field.setText("0")
        self.assertEqual(self.page.params()["prefer_x_min"], 0.0,
                         "0 是合法下限（中心 X ≥ 0）")
        field.setText("900")
        self.assertEqual(self.page.params()["prefer_x_min"], 900.0)

    def test_prefer_x_min_garbage_is_reported_not_swallowed(self):
        """填了非数字：卡片红字报错（**不**静默当成“不限”）。"""
        self.page.fields["cube_auto"].setChecked(True)
        self.page.fields["prefer_x_min"].setText("abc")
        preview = self.page.refresh_preview()
        self.assertFalse(preview["ok"])
        self.assertIn("要填数字", self.page.error_label.text())
        self.assertFalse(self.page.generate_button.isEnabled())
        # 改回合法值就恢复
        self.page.fields["prefer_x_min"].setText("900")
        self.assertTrue(self.page.refresh_preview()["ok"])

    def test_only_active_group_reaches_core(self):
        """两种模式都真的能被 ``core`` 接受（报错原文说明它读到了对的键）。"""
        core_params = self.page.params()
        self.assertIn("cube", core_params)
        # 把 cube 换成下界 > 上界：core 的报错原文必须能出现在预览卡片里
        self.page.fields["x0"].setValue(2000.0)
        preview = self.page.refresh_preview()
        self.assertFalse(preview["ok"])
        self.assertIn("下界必须小于上界", preview["error"])
        self.assertIn("下界必须小于上界", self.page.error_label.text())
        self.assertFalse(self.page.generate_button.isEnabled())


class ModeSwitchTest(unittest.TestCase):
    """立方体来源切换：**永远恰好一组可见**，且隐藏不清空（钉死 32136c6）。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def test_exactly_one_group_visible_after_init(self):
        """初始化按 radio 的当前值切，不许出现“两组都隐藏”。"""
        self.assertEqual(len(visible_panels(self.page)), 1)
        self.assertEqual(self.page.mode_value(), "bounds")
        self.assertEqual(visible_panels(self.page), [0])
        self.assertTrue(self.page.bounds_panel.isVisibleTo(self.page.mode_stack))
        self.assertFalse(self.page.auto_panel.isVisibleTo(self.page.mode_stack))

    def test_two_switches_each_way_keep_one_group_visible(self):
        """两种模式各切 2 次，可见组始终恰好 1 个。"""
        for _round in range(2):
            self.page.fields["cube_auto"].setChecked(True)
            self.assertEqual(len(visible_panels(self.page)), 1,
                             "切到自动搜索后可见组数目")
            self.assertEqual(visible_panels(self.page), [1])
            self.assertEqual(self.page.mode_value(), "auto")
            self.page.fields["cube_bounds"].setChecked(True)
            self.assertEqual(len(visible_panels(self.page)), 1,
                             "切回两点输入后可见组数目")
            self.assertEqual(visible_panels(self.page), [0])
            self.assertEqual(self.page.mode_value(), "bounds")

    def test_values_survive_switching(self):
        """切走再切回，两组输入填过的值都还在（隐藏 ≠ 清空）。"""
        fields = self.page.fields
        fields["x0"].setValue(901.5)
        fields["box_x0"].setValue(705.0)
        fields["resolution"].setValue(25.0)
        fields["prefer_x_min"].setText("880")

        fields["cube_auto"].setChecked(True)
        self.assertEqual(fields["x0"].value(), 901.5)
        fields["cube_bounds"].setChecked(True)
        self.assertEqual(fields["box_x0"].value(), 705.0)
        self.assertEqual(fields["resolution"].value(), 25.0)
        self.assertEqual(fields["prefer_x_min"].text(), "880")

        # 提交时两组各自带自己那一份，不串台
        bounds = self.page.params()
        self.assertEqual(bounds["cube"][0], 901.5)
        fields["cube_auto"].setChecked(True)
        auto = self.page.params()
        self.assertEqual(auto["search_box"][0], 705.0)
        self.assertEqual(auto["resolution"], 25.0)
        self.assertEqual(auto["prefer_x_min"], 880.0)

    def test_switch_mode_accepts_explicit_value(self):
        """``switch_mode`` 也能被显式调用（初始化/程序化设置都走它）。"""
        self.page.switch_mode("auto")
        self.assertEqual(self.page.mode_value(), "auto")
        self.assertEqual(visible_panels(self.page), [1])
        self.page.switch_mode("bounds")
        self.assertEqual(self.page.mode_value(), "bounds")
        self.assertEqual(visible_panels(self.page), [0])
        # 未知模式名不许把两组都藏起来（这就是 32136c6 那个 bug）
        self.page.switch_mode("center")
        self.assertEqual(len(visible_panels(self.page)), 1)

    def test_switch_to_auto_recomputes_preview(self):
        """切到「自动搜索」：预览卡立刻换成搜索箱，不挂上一种来源的边长。"""
        self.assertEqual(self.page.preview["mode"], "bounds")
        self.assertEqual(self.page.preview["side"], 550.0)
        self.assertEqual(self.page.preview_table.rowCount(), 5)

        self.page.fields["cube_auto"].setChecked(True)

        self.assertEqual(self.page.preview["mode"], "auto")
        self.assertIsNone(self.page.preview["side"])
        self.assertEqual(self.page.preview_table.rowCount(), 0)
        self.assertEqual(self.page.preview_labels["cube"].text(),
                         "待真跑（自动搜索）", "不许还写着上一种来源的边长")
        self.assertEqual(self.page.preview_labels["apothem"].text(), "—")
        self.assertIn("搜索范围", self.page.preview_hint.text())
        # 面心 / 候选点文本块也不许留着上一个立方体的内容
        self.assertEqual(self.page.faces_edit.toPlainText(), "")
        self.assertEqual(self.page.candidates_edit.toPlainText(), "")
        # “三轴跨度不等”是 bounds 那组输入的提示，切走就不该还在
        self.assertFalse(
            any("跨度不等" in line
                for line in self.page.warning_label.text().splitlines()),
            self.page.warning_label.text())

    def test_switch_back_to_bounds_restores_geometry(self):
        """再切回「两点输入」：边长 / 五点表都回到 550 mm 那一版。"""
        self.page.fields["cube_auto"].setChecked(True)
        self.assertEqual(self.page.preview_table.rowCount(), 0)

        self.page.fields["cube_bounds"].setChecked(True)

        self.assertEqual(self.page.preview["mode"], "bounds")
        self.assertEqual(self.page.preview["side"], 550.0)
        self.assertEqual(self.page.preview_labels["cube"].text(), "边长 550 mm")
        self.assertEqual(self.page.preview_table.rowCount(), 5)


class LayoutGateTest(unittest.TestCase):
    """``square_frac`` 只对 ``center-square`` 生效。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def test_square_frac_only_enabled_for_center_square(self):
        fields = self.page.fields
        for layout, enabled in (("vertices", False), ("circle", False),
                                ("center-square", True),
                                ("vertices", False)):
            fields["layout"].setCurrentIndex(fields["layout"].findData(layout))
            self.assertEqual(fields["square_frac"].isEnabled(), enabled, layout)

    def test_layout_values_come_from_algorithm_module(self):
        """下拉框的取值集合就是 ``iso9283_cube.LAYOUTS`` / ``PLANES``。"""
        fields = self.page.fields
        self.assertEqual([fields["layout"].itemData(index)
                          for index in range(fields["layout"].count())],
                         list(iso.LAYOUTS))
        self.assertEqual([fields["plane"].itemData(index)
                          for index in range(fields["plane"].count())],
                         list(iso.PLANES))
        self.assertEqual(set(iso_page.LAYOUTS), set(iso.LAYOUTS))
        self.assertEqual(set(iso_page.PLANES), set(iso.PLANES))


class PreviewGeometryTest(unittest.TestCase):
    """预览（不跑 IK）：边长 / 边心距 / 6 个面心 / 9 个候选点 / 5 个点。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def small_case(self) -> dict:
        """200 mm 立方体（站点位置，5 点全可解）。"""
        values = iso_page.iso_form_defaults()
        values.update({"cube": [925.0, 1125.0, -100.0, 100.0, 505.0, 705.0],
                       "rpy": [3.0, 73.0, -167.0],
                       "euler": "xyz-intrinsic",
                       "q_seed": "43,-30,-36,42,-89,-1"})
        return values

    def test_geometry_matches_core(self):
        """预览里的中心/边长与 ``core`` 的取法一致（同一条通道）。"""
        preview = iso_page.preview_geometry(self.small_case())
        self.assertTrue(preview["ok"], preview.get("error"))
        self.assertEqual(preview["center"], [1025.0, 0.0, 605.0])
        self.assertEqual(preview["side"], 200.0)
        self.assertEqual(preview["apothem"], 100.0)
        self.assertEqual(len(preview["faces"]), 6)
        self.assertEqual(len(preview["candidates"]), 9)
        self.assertEqual([row["name"] for row in preview["points"]],
                         list(iso.POINT_NAMES))
        # 9 个候选点 = 8 个顶点 + 中心（与 cube_vertices 同一份几何）
        vertices = [[float(value) for value in vertex]
                    for vertex in iso.cube_vertices(preview["center"],
                                                    preview["side"])]
        self.assertEqual([row["pos"] for row in preview["candidates"][:8]],
                         vertices)
        self.assertEqual(preview["candidates"][8]["pos"], preview["center"])
        # 6 个面心到中心的距离 = 边心距
        for row in preview["faces"]:
            distance = sum((float(row["pos"][axis])
                            - preview["center"][axis]) ** 2
                           for axis in range(3)) ** 0.5
            self.assertAlmostEqual(distance, preview["apothem"], places=6)
        # 五个点与 ``iso.cube_points`` 逐点一致（预览不另算一份）
        expected = iso.cube_points(preview["center"], preview["side"],
                                   "vertices", 0.4, "xz")
        for row in preview["points"]:
            self.assertEqual(row["pos"],
                             [float(value) for value in expected[row["name"]]])

    def test_points_carry_layout_labels(self):
        """点 1..5 带 ``layout`` 标注（P1 总是中心）。"""
        preview = iso_page.preview_geometry(self.small_case())
        self.assertEqual(preview["points"][0]["label"], "立方体中心")
        self.assertIn("对角截面四角", preview["points"][1]["label"])
        values = self.small_case()
        values["layout"] = "center-square"
        values["square_frac"] = 0.5
        preview = iso_page.preview_geometry(values)
        self.assertIn("正方形角", preview["points"][1]["label"])
        self.assertIn("0.5", preview["points"][1]["label"])

    def test_auto_mode_has_no_geometry(self):
        """自动搜索：边长/中心要真跑才知道，预览只给搜索范围。"""
        values = self.small_case()
        values["cube_mode"] = "auto"
        preview = iso_page.preview_geometry(values)
        self.assertTrue(preview["ok"])
        self.assertIsNone(preview["side"])
        self.assertEqual(preview["search_box"],
                         [float(value) for value in iso_page.SAMPLE_SEARCH_BOX])
        self.assertTrue(preview["warnings"])

    def test_card_shows_preview_numbers(self):
        """预览卡片：边长 / 边心距 / 中心 / 布点 / 截面 + 面心与候选点文本块。"""
        self.page.fields["cube_bounds"].setChecked(True)
        for key, value in (("x0", 925.0), ("x1", 1125.0), ("y0", -100.0),
                           ("y1", 100.0), ("z0", 505.0), ("z1", 705.0)):
            self.page.fields[key].setValue(value)
        self.page.refresh_preview()
        self.assertEqual(self.page.preview_labels["cube"].text(), "边长 200 mm")
        self.assertEqual(self.page.preview_labels["apothem"].text(), "100 mm")
        self.assertIn("1025.0", self.page.preview_labels["center"].text())
        self.assertIn("对角截面四角", self.page.preview_labels["layout"].text())
        self.assertIn("XZ", self.page.preview_labels["plane"].text())
        self.assertEqual(self.page.preview_table.rowCount(), 5)
        self.assertEqual(len(self.page.faces_edit.toPlainText().splitlines()), 6)
        self.assertEqual(
            len(self.page.candidates_edit.toPlainText().splitlines()), 9)

    def test_preview_recomputes_after_debounce(self):
        """改任何参数都重算预览：数字即时更新，图交给 300 ms 去抖。"""
        self.assertEqual(iso_page.DEBOUNCE_MS, 300)
        self.assertEqual(self.page._debounce.interval(), 300)
        # 默认示例域：边长 = 最小跨度 550 mm
        self.assertEqual(self.page.preview["side"], 550.0)
        self.page.fields["x0"].setValue(925.0)
        self.assertEqual(self.page.preview["center"][0], 1112.5,
                         "中心跟着 X 下界动")
        self.assertTrue(self.page._debounce.isActive(),
                        "改动后应当排上 300 ms 的去抖重绘")
        # 换成 200 mm 立方体，再把 X 收成 150 mm：边长与图都要跟着走
        for key, value in (("x1", 1125.0), ("y0", -100.0), ("y1", 100.0),
                           ("z0", 505.0), ("z1", 705.0)):
            self.page.fields[key].setValue(value)
        self.assertEqual(self.page.preview["side"], 200.0)
        self.page.fields["x0"].setValue(975.0)
        self.assertEqual(self.page.preview["side"], 150.0)
        figure = self.page.apply_preview()
        self.assertIn("150", str(figure.data[0].name))

    def test_default_preset_reports_unequal_spans(self):
        """默认示例域三轴跨度不等：提示文字来自 ``core``（界面不另写判据）。"""
        preview = self.page.preview
        self.assertTrue(preview["ok"], preview.get("error"))
        self.assertTrue(any("跨度不等" in line for line in preview["warnings"]),
                        preview["warnings"])
        self.assertTrue(self.page.warning_label.isVisibleTo(self.page))

    def test_apothem_shows_dash_when_missing(self):
        """``apothem`` 缺值显示 ``—``（``or 0`` 会把它画成「0 mm」）。"""
        preview = dict(self.page.preview)
        self.assertEqual(self.page.preview_labels["apothem"].text(), "275 mm")
        preview["apothem"] = None
        self.page._update_card(preview)
        self.assertEqual(self.page.preview_labels["apothem"].text(), "—")
        preview["apothem"] = 275.0
        self.page._update_card(preview)
        self.assertEqual(self.page.preview_labels["apothem"].text(), "275 mm")


class StaleResultTest(unittest.TestCase):
    """跑完之后改参数：统计行 / 路点表仍是上一次运行的，界面必须点明。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def fake_result(self) -> dict:
        """最小可渲染结果（够铺统计行与 5 行路点表，不跑 IK）。"""
        points = [{"name": name, "pos": [1025.0, 0.0, 605.0],
                   "q": [0.0] * 6, "config": [1, 0, 0], "ok": True,
                   "j3_abs": 30.0, "j5_abs": 40.0 + index}
                  for index, name in enumerate(iso.POINT_NAMES)]
        return {"ok": True, "run_id": "20260101-000000-iso0",
                "run_dir": "/tmp/iso-stale", "artifacts": {},
                "stats": {"solved": 5, "total": 5, "j5_abs_min": 40.0,
                          "j3_abs_min": 30.0, "config_consistent": True,
                          "configs": [[1, 0, 0]] * 5, "cube_side": 550.0,
                          "cube_center": [1025.0, 0.0, 605.0],
                          "layout": "vertices", "plane": "xz"},
                "points": points, "warnings": []}

    def test_changed_params_mark_the_table_as_stale(self):
        """出结果后改参数：灰字说清“下表来自上一次运行”。"""
        self.page.render_result(self.fake_result())
        self.assertEqual(self.page.point_table.rowCount(), 5)
        self.assertFalse(self.page.stale_label.isVisibleTo(self.page),
                         "刚跑完的结果不该被标成陈旧")

        self.page.fields["x0"].setValue(925.0)

        self.assertTrue(self.page.stale_label.isVisibleTo(self.page))
        self.assertIn("上一次运行", self.page.stale_label.text())
        self.assertIn("20260101-000000-iso0", self.page.stale_label.text())
        self.assertIn("color:#6b7280", self.page.stale_label.styleSheet())
        # 表确实是旧的那一份（路点表不会因为改参数而重算）
        self.assertEqual(self.page.point_table.rowCount(), 5)
        self.assertEqual(self.page.last_result["run_id"],
                         "20260101-000000-iso0")

    def test_hint_clears_when_a_new_result_arrives(self):
        """下一次真跑出结果时，灰字自己退场。"""
        self.page.render_result(self.fake_result())
        self.page.fields["x0"].setValue(925.0)
        self.assertTrue(self.page.stale_label.isVisibleTo(self.page))
        self.page.render_result(self.fake_result())
        self.assertFalse(self.page.stale_label.isVisibleTo(self.page))


class PreviewStdoutIsolationTest(unittest.TestCase):
    """预览的 ``redirect_stdout`` 只在没有任务在跑时用。

    ``contextlib.redirect_stdout`` 换的是进程级的 ``sys.stdout``，而预览在
    ``_on_change`` 里同步跑：任务运行中拖一下 spinbox 就会把工作线程正在
    ``print`` 的日志行吞进预览的 buffer（日志页于是少几行）。
    """

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def has_core_hint(self, preview) -> bool:
        """预览的 ``warnings`` 里有没有 ``core._iso_cube`` 那句 ``print``。"""
        return any("跨度不等" in line
                   for line in (preview or {}).get("warnings") or [])

    def test_capture_is_skipped_while_a_job_runs(self):
        """任务在跑时不抓 stdout；空闲时仍然抓（提示只有一个来源）。"""
        self.assertTrue(self.has_core_hint(self.page.preview))
        before = sys.stdout
        self.page._busy = True
        try:
            running = self.page.refresh_preview()
        finally:
            self.page._busy = False
        self.assertTrue(running["ok"])
        self.assertFalse(self.has_core_hint(running),
                         "任务在跑时不许抓 stdout（会吞掉工作线程的日志）")
        self.assertIs(sys.stdout, before, "预览必须把 sys.stdout 还回去")
        self.assertTrue(self.has_core_hint(self.page.refresh_preview()),
                        "空闲时仍然抓：提示文字只有一个来源")

    def test_bridge_running_is_respected(self):
        """接管来的任务（``bridge.is_running``）同样不抓。"""
        with mock.patch.object(type(self.page.bridge), "is_running",
                               property(lambda _self: True)):
            preview = self.page.refresh_preview()
        self.assertTrue(preview["ok"])
        self.assertFalse(self.has_core_hint(preview))


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
