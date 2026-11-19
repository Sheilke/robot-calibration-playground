#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 五点页（S5）纯逻辑：表单 -> core 参数、只有一种立方体来源、五点编号。

对齐的基准是网页版 ``laser_web/pages/iso9283.py::collect()``：本文件把它的键
集合**逐个写死**（:data:`PAGE_KEYS`），再断言
:func:`laser_desktop.iso_page.form_to_params` 的产物与它**完全同集合**。

ISO 9283 只定义了一种五点取法，所以页面上没有“立方体来源 / 布点 / 截面”这类
切换控件（它们连同自动搜索整组一起删掉了）：

1. 只提交 ``cube``，**不**提交 ``search_box`` / ``resolution`` / ``prefer_x_min``；
2. ``layout`` / ``plane`` 仍然提交（``core`` 的参数约定不变），值固定
   ``vertices`` / ``xz``；
3. 五点编号：P2/P3 在小 X 小 Z 一侧，P4/P5 在大 X 大 Z 一侧（ISO 9283）；
4. 这一页不显示关节量：``q_seed`` / ``j5_min`` 两个输入也删了，``core`` 用默认值
   （可达性验算的种子固定，见 ``core.iso_points``）。
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


#: ISO 页**唯一**的提交键集合（网页版 ``collect()`` 与本页
#: ``form_to_params`` 逐键一致）：立方体只有一种给法，布点与截面固定，
#: 没有 ``q_seed`` / ``j5_min``（界面不暴露，``core`` 用自己的默认值）。
PAGE_KEYS = frozenset({"robot", "cube_mode", "cube", "layout", "plane",
                       "rpy", "euler"})
#: 界面上已经没有的键：自动搜索整组 + 布点参数 + IK 种子与 |J5| 阈值。
SEARCH_KEYS = ("search_box", "resolution", "prefer_x_min", "square_frac",
               "q_seed", "j5_min")
#: 界面上已经没有的控件（来源切换 / 自动搜索 / 布点 / 截面 / 种子 / 告警）。
REMOVED_FIELDS = ("cube_bounds", "cube_auto", "box_x0", "box_x1", "box_y0",
                  "box_y1", "box_z0", "box_z1", "resolution",
                  "prefer_x_min", "layout", "plane", "square_frac",
                  "q_seed", "j5_min")


class FormToParamsTest(unittest.TestCase):
    """表单值 -> ``core.run_iso9283`` 的入参（只有一组）。"""

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
        for key in ("rpy", "euler"):
            self.assertEqual(page_defaults[key], defaults[key], key)
        # 布点 / 截面不可选：写死 ISO 9283 的取法（与 core 的 CLI 默认一致）
        self.assertEqual(page_defaults["layout"], iso_page.FIXED_LAYOUT)
        self.assertEqual(page_defaults["plane"], iso_page.FIXED_PLANE)
        self.assertEqual(page_defaults["layout"], defaults["layout"])
        self.assertEqual(page_defaults["plane"], defaults["plane"])
        # 两点输入的示例值来自网页版示例域（与「生成」页同源）
        self.assertEqual(page_defaults["cube"][0],
                         float(ui_state.SAMPLE_POINT_MIN[0]))
        self.assertEqual(page_defaults["cube"][1],
                         float(ui_state.SAMPLE_POINT_MAX[0]))
        # 打开页面时的实际表单值与默认值一致
        self.assertEqual(self.page.params(), iso_page.form_to_params(page_defaults))

    def test_params_submit_only_the_cube(self):
        """只提交 ``cube``：搜索相关的键一个不能有。"""
        params = self.page.params()
        self.assertEqual(set(params), set(PAGE_KEYS))
        for key in SEARCH_KEYS:
            self.assertNotIn(key, params)
        self.assertEqual(params["cube_mode"], "bounds")
        self.assertEqual(params["cube"],
                         [float(value) for value in
                          iso_page.iso_form_defaults()["cube"]])
        # 布点 / 截面仍然提交（core 的参数约定不变）
        self.assertEqual(params["layout"], iso_page.FIXED_LAYOUT)
        self.assertEqual(params["plane"], iso_page.FIXED_PLANE)

    def test_form_has_no_source_or_layout_widgets(self):
        """界面上没有来源 / 布点 / 截面控件（连字段名都不存在）。"""
        for key in REMOVED_FIELDS:
            self.assertNotIn(key, self.page.fields, key)
        self.assertFalse(hasattr(self.page, "mode_stack"))
        self.assertFalse(hasattr(self.page, "mode_buttons"))
        for method in ("switch_mode", "switch_layout", "mode_value",
                       "prefer_min_value"):
            self.assertFalse(hasattr(self.page, method), method)

    def test_params_values_match_widgets(self):
        """逐键核对：提交值就是控件里那一版（不是默认值）。"""
        fields = self.page.fields
        for key, value in (("x0", 900.0), ("x1", 1100.0), ("y0", -100.0),
                           ("y1", 100.0), ("z0", 500.0), ("z1", 700.0)):
            fields[key].setValue(value)
        fields["rpy_a"].setValue(3.0)
        fields["rpy_b"].setValue(73.0)
        fields["rpy_c"].setValue(-167.0)
        fields["euler"].setCurrentIndex(fields["euler"].findData("zyx-intrinsic"))

        params = self.page.params()
        self.assertEqual(params["cube"], [900.0, 1100.0, -100.0, 100.0,
                                          500.0, 700.0])
        self.assertEqual(params["layout"], "vertices")
        self.assertEqual(params["plane"], "xz")
        self.assertEqual(params["rpy"], [3.0, 73.0, -167.0])
        self.assertEqual(params["euler"], "zyx-intrinsic")
        self.assertNotIn("q_seed", params)
        self.assertNotIn("j5_min", params)

    def test_parameters_reach_core(self):
        """参数真的能被 ``core`` 接受（报错原文说明它读到了对的键）。"""
        core_params = self.page.params()
        self.assertIn("cube", core_params)
        # X 下界抬到上界之上：core 的报错原文必须能出现在预览卡片里
        self.page.fields["x0"].setValue(2000.0)
        preview = self.page.refresh_preview()
        self.assertFalse(preview["ok"])
        self.assertIn("下界必须小于上界", preview["error"])
        self.assertIn("下界必须小于上界", self.page.error_label.text())
        self.assertFalse(self.page.generate_button.isEnabled())


class CartesianOnlyTest(unittest.TestCase):
    """这一页只算坐标：提示文案、没有种子/告警输入、没有任务机制。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def test_hint_matches_the_agreed_wording(self):
        self.assertEqual(
            iso_page.HINT,
            "只算笛卡尔坐标；姿态固定、不加扰动；系统会解一次 IK 判断每点是否"
            "可达，不输出关节角。")
        # 页面上真的能看到这句（左栏姿态组 + 预览页签）
        labels = [label.text() for label in self.page.findChildren(QtWidgets.QLabel)]
        self.assertTrue(any(iso_page.HINT in text for text in labels),
                        "页面上找不到那句提示")

    def test_no_job_machinery_and_no_joint_inputs(self):
        for attribute in ("bridge", "progress_bar", "cancel_button",
                          "log_edit", "log_page"):
            self.assertFalse(hasattr(self.page, attribute), attribute)
        for key in ("q_seed", "j5_min"):
            self.assertNotIn(key, self.page.fields)
        self.assertEqual(list(iso_page.TABS), ["路点", "预览"])

    def test_stat_keys_are_reachable_side_center(self):
        self.assertEqual(list(iso_page.STAT_KEYS), ["reach", "side", "center"])
        self.assertEqual(list(iso_page.STAT_KEYS), list(self.page.stat_labels))


class SingleCubeSourceTest(unittest.TestCase):
    """只有一种立方体来源：改数值就重算预览，没有可切的“另一组”。"""

    def setUp(self):
        app()
        self.page = iso_page.IsoPage(plot=None)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()

    def test_preview_recomputes_when_the_cube_changes(self):
        """改极值点：边长 / 中心 / 五点表都跟着走。"""
        self.assertEqual(self.page.preview["side"], 550.0)
        self.assertEqual(self.page.preview_table.rowCount(), 5)
        for key, value in (("x0", 925.0), ("x1", 1125.0), ("y0", -100.0),
                           ("y1", 100.0), ("z0", 505.0), ("z1", 705.0)):
            self.page.fields[key].setValue(value)
        preview = self.page.refresh_preview()
        self.assertEqual(preview["side"], 200.0)
        self.assertEqual(preview["center"], [1025.0, 0.0, 605.0])
        self.assertEqual(self.page.preview_table.rowCount(), 5)
        self.assertEqual(self.page.preview_labels["cube"].text(), "边长 200 mm")


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

    def test_points_carry_the_iso_numbering_labels(self):
        """点 1..5 的位置标注按 ISO 9283 编号（P1 中心、P2/P3 小 X 小 Z）。"""
        preview = iso_page.preview_geometry(self.small_case())
        labels = [row["label"] for row in preview["points"]]
        self.assertEqual(labels[0], "立方体中心")
        for name, label in zip(("P2", "P3"), labels[1:3]):
            self.assertIn("X 小", label, name)
            self.assertIn("Z 小", label, name)
        for name, label in zip(("P4", "P5"), labels[3:]):
            self.assertIn("X 大", label, name)
            self.assertIn("Z 大", label, name)

    def test_card_shows_preview_numbers(self):
        """预览卡片：边长 / 边心距 / 中心 + 面心与候选点文本块（没有布点/截面）。"""
        for key, value in (("x0", 925.0), ("x1", 1125.0), ("y0", -100.0),
                           ("y1", 100.0), ("z0", 505.0), ("z1", 705.0)):
            self.page.fields[key].setValue(value)
        self.page.refresh_preview()
        self.assertEqual(self.page.preview_labels["cube"].text(), "边长 200 mm")
        self.assertEqual(self.page.preview_labels["apothem"].text(), "100 mm")
        self.assertIn("1025.0", self.page.preview_labels["center"].text())
        # 布点 / 截面不再是卡片上的一行（也不再是可选参数）
        self.assertNotIn("layout", self.page.preview_labels)
        self.assertNotIn("plane", self.page.preview_labels)
        self.assertEqual(list(self.page.preview_labels),
                         [key for key, _title in iso_page.PREVIEW_METRICS])
        # 面心 6 个 / 候选点 9 个 / 五点 5 个：文案与行数都对得上
        hint = self.page.preview_hint.text()
        self.assertIn("面心 6 个", hint)
        self.assertIn("候选点 9 个", hint)
        self.assertIn("五点 5 个", hint)
        self.assertIn("P2/P3 在 X 小 Z 小一侧", hint)
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
        """最小可渲染结果（够铺统计行与 5 行路点表，没有关节量）。"""
        points = [{"name": name, "pos": [1025.0, 0.0, 605.0],
                   "reachable": True}
                  for name in iso.POINT_NAMES]
        return {"ok": True,
                "summary": {"reachable": 5, "total": 5},
                "meta": {"cube_side": 550.0,
                         "cube_center": [1025.0, 0.0, 605.0],
                         "layout": "vertices", "plane": "xz",
                         "rpy": [3.0, 73.0, -167.0]},
                "points": points, "warnings": []}

    def test_changed_params_mark_the_table_as_stale(self):
        """出结果后改参数：灰字说清“下表来自上一次运行”。"""
        self.page.render_result(self.fake_result())
        self.assertEqual(self.page.point_table.rowCount(), 5)
        self.assertFalse(self.page.stale_label.isVisibleTo(self.page),
                         "刚跑完的结果不该被标成陈旧")

        self.page.fields["x0"].setValue(925.0)

        self.assertTrue(self.page.stale_label.isVisibleTo(self.page))
        self.assertIn("上一次的结果", self.page.stale_label.text())
        self.assertIn("color:#6b7280", self.page.stale_label.styleSheet())
        # 表确实是旧的那一份（路点表不会因为改参数而重算）
        self.assertEqual(self.page.point_table.rowCount(), 5)
        self.assertEqual(self.page.last_result["summary"]["reachable"], 5)

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

    def test_capture_is_skipped_while_another_page_runs_a_job(self):
        """别的页有任务在跑就不抓 stdout；空闲时仍然抓（提示只有一个来源）。"""
        self.assertTrue(self.has_core_hint(self.page.preview))
        before = sys.stdout
        with mock.patch.object(iso_page.IsoPage, "_job_running_elsewhere",
                               staticmethod(lambda: True)):
            running = self.page.refresh_preview()
        self.assertTrue(running["ok"])
        self.assertFalse(self.has_core_hint(running),
                         "任务在跑时不许抓 stdout（会吞掉工作线程的日志）")
        self.assertIs(sys.stdout, before, "预览必须把 sys.stdout 还回去")
        self.assertTrue(self.has_core_hint(self.page.refresh_preview()),
                        "空闲时仍然抓：提示文字只有一个来源")

    def test_registry_probe_does_not_raise(self):
        """登记表探针在任何状态下都要给出布尔值（判不出来就当“没在跑”）。"""
        self.assertIn(iso_page.IsoPage._job_running_elsewhere(), (True, False))


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
