#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复页（S4）纯逻辑：表单 -> core 参数、默认值、动态读数行、现场样例。

对齐的基准是网页版 ``laser_web/pages/repair.py::collect()``：本文件把它的键
集合**逐个写死**（``WEB_COLLECT_KEYS``），再断言
:func:`laser_desktop.repair_page.form_to_params` 的产物是「那套键 + 两个桌面版
新增项」——新增项在 ``core._repair_namespace`` 里真的被读走，不是装饰。

布尔方向用 ``core._repair_namespace`` 反查（``allow_rpy=True`` ->
``no_rpy_change=False``），不靠界面自己再判断一次。
"""

import os
import shutil
import tempfile
import unittest
from pathlib import Path

# 必须在建 QApplication 之前：pytest 命令行不一定带 QT_QPA_PLATFORM
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import repair_page
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtWidgets
from laser_web import core, jobs
from excitation_trajectory import aabb_snake_scan as scan

_APP = None


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-repair-tests"])
    return _APP


#: 网页版 ``collect()`` 一定提交的键（与采样域 / 网格的互斥项无关）。
WEB_COMMON_KEYS = frozenset({
    "robot", "rpy", "euler", "perturb_deg", "cond_max", "j5_min", "j3_min",
    "seg_min", "seed", "q_seed",
    # ---- 修复流程专用 ----
    "singular_q", "stopped_at", "segment_window", "avoid_margin",
    "avoid_radius", "seg_samples", "allow_rpy", "allow_shrink",
})

#: 桌面版**多**提交的两项（S4 规格要求表单有这两个输入框；core 认这两个键，
#: 网页版页面没有输入框所以只吃 CLI 默认值）。
DESKTOP_EXTRA_KEYS = frozenset({"repair_attempts", "max_candidates"})

#: 8 点小案子（六个轴边界 + 目标点数 8）：校验能过，`core` 也解得起。
SMALL_CASE = {
    "domain_mode": "axes",
    "aabb6": [900.0, 1100.0, -100.0, 100.0, 500.0, 700.0],
    "grid_mode": "count",
    "target_count": 8,
    "rpy": [3.0, 73.0, -167.0],
    "euler": "xyz-intrinsic",
    "q_seed": "43,-30,-36,42,-89,-1",
    "j5_min": 20.0,
    "seed": 0,
}


def expected_keys(domain_mode: str, grid_mode: str) -> set:
    """网页版 ``collect()`` 在该模式组合下会产出的键 + 桌面版新增项。"""
    keys = set(WEB_COMMON_KEYS) | set(DESKTOP_EXTRA_KEYS)
    if domain_mode == "points":
        keys |= {"point_min", "point_max"}
    else:
        keys |= {"aabb6"}
    if grid_mode == "count":
        keys |= {"target_count"}
    else:
        keys |= {"xs", "ys", "zs", "z_insert"}
    return keys


def expected_params_value(params: dict, path):
    """从 params 里取一个值；``path`` 可以是 ``"rpy"`` 或 ``("rpy", 0)``。"""
    key, *indexes = path if isinstance(path, tuple) else (path,)
    value = params[key]
    for index in indexes:
        value = value[index]
    return value


class FormToParamsTest(unittest.TestCase):
    """``form_to_params`` 只搬运 + 类型归一；校验与语义仍由 core 决定。"""

    def test_key_set_matches_web_collect_in_all_four_modes(self):
        """四种互斥组合下，键集合都要与网页版 ``collect()`` + 两项新增对齐。"""
        for domain_mode in ("points", "axes"):
            for grid_mode in ("count", "explicit"):
                with self.subTest(domain_mode=domain_mode, grid_mode=grid_mode):
                    params = repair_page.form_to_params({
                        "domain_mode": domain_mode, "grid_mode": grid_mode})
                    self.assertEqual(set(params),
                                     expected_keys(domain_mode, grid_mode))
                    # 互斥项只提交当前激活的那一组（不能两个都塞进去）
                    if domain_mode == "points":
                        self.assertNotIn("aabb6", params)
                    else:
                        self.assertNotIn("point_min", params)
                    if grid_mode == "count":
                        self.assertNotIn("xs", params)
                    else:
                        self.assertNotIn("target_count", params)

    def test_keys_web_does_not_submit_are_absent(self):
        """网页版不提交、core 的修复路径也不用的键，这里一个都不许有。"""
        params = repair_page.form_to_params({})
        for key in ("seg_mode", "j5_attempts", "seg_samples_final",
                    "force_search", "no_rpy_change", "no_shrink"):
            self.assertNotIn(key, params, key)

    def test_singular_q_is_list_of_six_float_lists(self):
        """``singular_q`` 必须是 ``list[list[float]]``（不是字符串也不是扁平 6 个）。"""
        params = repair_page.form_to_params({
            "singular_q": [[0, 0, 0, 55.721, 0, -55.721],
                           [1, 2, 3, 4, 5, 6]]})
        self.assertIsInstance(params["singular_q"], list)
        self.assertEqual(len(params["singular_q"]), 2)
        for group in params["singular_q"]:
            self.assertIsInstance(group, list)
            self.assertEqual(len(group), 6)
            for value in group:
                self.assertIsInstance(value, float)
        self.assertEqual(params["singular_q"][0],
                         [0.0, 0.0, 0.0, 55.721, 0.0, -55.721])

    def test_stopped_at_is_none_or_int(self):
        """界面 0 = 没给 -> ``None``；正数 -> ``int``（不能是 float 或字符串）。"""
        self.assertIsNone(repair_page.form_to_params(
            {"stopped_at": 0})["stopped_at"])
        self.assertIsNone(repair_page.form_to_params(
            {"stopped_at": None})["stopped_at"])
        value = repair_page.form_to_params({"stopped_at": 41})["stopped_at"]
        self.assertIsInstance(value, int)
        self.assertEqual(value, 41)
        # 单独测归一函数：空串也算没给
        self.assertIsNone(repair_page.stopped_at_value(""))
        self.assertEqual(repair_page.stopped_at_value("7"), 7)

    def test_allow_flags_keep_their_polarity_through_core(self):
        """``allow_rpy`` / ``allow_shrink`` 的方向：True -> ``no_*_change`` False。"""
        values = {**SMALL_CASE, "allow_rpy": False, "allow_shrink": True,
                  "singular_q": [[0.0, 0.0, 0.0, 55.72, 0.0, -55.72]],
                  "stopped_at": 4}
        args, _hints = core._repair_namespace(repair_page.form_to_params(values))
        self.assertTrue(args.no_rpy_change, "allow_rpy=False 应该变成 no_rpy_change=True")
        self.assertFalse(args.no_shrink, "allow_shrink=True 应该变成 no_shrink=False")

        values.update({"allow_rpy": True, "allow_shrink": False})
        args, _hints = core._repair_namespace(repair_page.form_to_params(values))
        self.assertFalse(args.no_rpy_change)
        self.assertTrue(args.no_shrink, "allow_shrink=False 应该变成 no_shrink=True")

    def test_desktop_extras_reach_the_namespace(self):
        """``repair_attempts`` / ``max_candidates`` 真的进了 core 的 Namespace。"""
        values = {**SMALL_CASE,
                  "singular_q": [[0.0, 0.0, 0.0, 55.72, 0.0, -55.72]],
                  "stopped_at": 4, "repair_attempts": 25,
                  "max_candidates": 60}
        args, _hints = core._repair_namespace(repair_page.form_to_params(values))
        self.assertEqual(args.repair_attempts, 25)
        self.assertEqual(args.max_candidates, 60)

    def test_core_rejects_invalid_values_with_its_own_text(self):
        """非法 ``stopped_at`` 由 core 报原文（界面不另写规则）。"""
        values = {**SMALL_CASE,
                  "singular_q": [[0.0, 0.0, 0.0, 55.72, 0.0, -55.72]],
                  "stopped_at": 8}          # 8 点案子必须是 1..7
        result = core.locate_segment(repair_page.form_to_params(values))
        self.assertFalse(result["ok"])
        self.assertIn("1..7", result["error"])

    def test_defaults_come_from_the_repair_cli(self):
        """默认值问 CLI 要（尤其 ``seg_samples`` 两边故意不同）。"""
        defaults = repair_page.repair_defaults()
        cli = core.repair_cli_defaults()
        self.assertEqual(defaults["seg_samples"], cli["seg_samples"])
        self.assertEqual(defaults["segment_window"], cli["segment_window"])
        self.assertEqual(defaults["avoid_margin"], cli["avoid_margin"])
        self.assertEqual(defaults["avoid_radius"], cli["avoid_radius"])
        self.assertEqual(defaults["repair_attempts"], cli["repair_attempts"])
        self.assertEqual(defaults["max_candidates"], cli["max_candidates"])
        # 两个 CLI 的 --seg-samples 默値故意不同（扫描 21 / 修复 11），
        # 修复页必须跟修复 CLI，不能跟扫描页
        self.assertNotEqual(cli["seg_samples"],
                            scan.build_arg_parser().parse_args([]).seg_samples)
        # 两个开关默认都是「允许」
        self.assertTrue(defaults["allow_rpy"])
        self.assertTrue(defaults["allow_shrink"])
        # 现场样例与网页版一致
        self.assertEqual(defaults["stopped_at"], 41)
        self.assertEqual(defaults["singular_q"],
                         [[0.0, 0.0, 0.0, 55.721, 0.0, -55.721]])
        # 默认值真的能过 core 的校验
        self.assertTrue(core.preview_domain(
            repair_page.form_to_params(defaults))["ok"])


class SingularRowTest(unittest.TestCase):
    """奇异读数动态行：至少留一组、范围/小数位、删除与再加。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="repair-rows-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = repair_page.RepairPage(plot=None, bridge=self.bridge)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=30)
        shutil.rmtree(self.root, ignore_errors=True)

    def test_default_row_has_six_spins_with_the_expected_range(self):
        self.assertEqual(len(self.page.q_rows), 1)
        fields = self.page.q_rows[0]["fields"]
        self.assertEqual(len(fields), 6)
        for box in fields:
            self.assertEqual(box.minimum(), -repair_page.Q_LIMIT_DEG)
            self.assertEqual(box.maximum(), repair_page.Q_LIMIT_DEG)
            self.assertEqual(box.decimals(), repair_page.Q_DECIMALS)

    def test_add_and_remove_keep_at_least_one_row(self):
        self.page.add_q_row([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(len(self.page.q_rows), 2)
        self.assertEqual(self.page.values()["singular_q"][1],
                         [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        added = self.page.q_rows[-1]
        self.assertTrue(self.page.remove_q_row(added))
        self.assertEqual(len(self.page.q_rows), 1)
        self.assertFalse(self.page.remove_q_row(self.page.q_rows[0]),
                         "最后一组不许删")
        self.assertEqual(len(self.page.q_rows), 1)
        self.assertIn("至少保留一组", self.page.last_status)
        # 只剩一组时按钮也该灰掉（语义在 remove_q_row 里再兜一次）
        self.assertFalse(self.page.q_rows[0]["remove_button"].isEnabled())

    def test_stopped_at_zero_shows_dash(self):
        spin = self.page.fields["stopped_at"]
        self.assertEqual(spin.minimum(), 0)
        self.assertEqual(spin.maximum(), 9999)
        spin.setValue(0)
        self.assertEqual(spin.text(), "—（未给）")
        self.assertIsNone(self.page.params()["stopped_at"])

    def test_load_sample_fills_the_field_case(self):
        self.page.add_q_row()                       # 先弄出两行，看它怎么收拾
        self.page.load_sample()
        values = self.page.values()
        self.assertEqual(values["stopped_at"],
                         repair_page.FIELD_SAMPLE["stopped_at"])
        self.assertEqual(values["domain_mode"], "points")
        self.assertEqual(values["grid_mode"], "count")
        self.assertEqual(values["target_count"], 50)
        self.assertEqual(values["rpy"], repair_page.FIELD_SAMPLE["rpy"])
        self.assertEqual(len(values["singular_q"]), 1)
        for joint, value in enumerate(repair_page.FIELD_SAMPLE["singular_q"]):
            self.assertAlmostEqual(values["singular_q"][0][joint], value, places=2)
        self.assertEqual(values["robot"], repair_page.FIELD_SAMPLE["robot"])


class _PageCase(unittest.TestCase):
    """共用：一个挂临时任务登记表的修复页。"""

    prefix = "repair-preview-"

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix=self.prefix))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = repair_page.RepairPage(plot=None, bridge=self.bridge)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=30)
        shutil.rmtree(self.root, ignore_errors=True)


class PreviewGuardTest(_PageCase):
    """参数非法时「修复」按钮禁用、报错原文写在卡片里（红字，不弹窗）。"""

    def test_repair_form_has_no_inert_inputs(self):
        """修复流程不提交的键，表单里就不该有输入框（否则是骗人的控件）。

        * ``seg_mode``：段剖面固定 MOVL（``core._repair_context`` 不传它）；
        * ``j5_attempts``：网页版 ``collect()`` 不提交，缺项即用 CLI 默认值。
        """
        self.assertNotIn("seg_mode", self.page.fields)
        self.assertNotIn("j5_attempts", self.page.fields)

    def test_invalid_domain_shows_core_error_and_disables_repair(self):
        spin = self.page.fields["stopped_at"]
        spin.setValue(0)
        self.page.fields["domain_axes"].setChecked(True)
        for key, value in (("x_min", 1200.0), ("x_max", 900.0)):   # 上界 < 下界
            self.page.fields[key].setValue(value)
        result = self.page.preview_result()
        self.assertFalse(result.get("ok"))
        self.assertFalse(self.page.repair_button.isEnabled())
        self.assertFalse(self.page.error_label.isHidden())
        self.assertIn(result["error"], self.page.error_label.text())
        # 定位段仍然可点（点下去会把 core 的报错原文留在结果区）
        self.assertTrue(self.page.locate_button.isEnabled())
        located = self.page.locate()
        self.assertFalse(located["ok"])
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertIn(result["error"], self.page.status_label.text())


class PointLimitGateTest(_PageCase):
    """Q2：点数超上限时「修复」禁用 + 提交被硬拦住（与生成页同一判据）。

    实测过的现场情况：``xs/ys/zs = 200/200/20``（80 万点）时 ``preview ok=True``，
    旧代码只看 ``ok`` 所以按钮照旧可点，点下去真去 plan + IK 80 万点。
    """

    prefix = "repair-limit-"

    def test_over_limit_disables_repair_and_blocks_submit(self):
        page = self.page
        page.fields["grid_explicit"].setChecked(True)
        page.fields["z_insert"].setValue(0)
        # 评审实测那个组合：200/200/20 = 80 万点，preview 仍然 ok
        for key, value in (("xs", 200), ("ys", 200), ("zs", 20)):
            page.fields[key].setValue(value)
        self.assertEqual(page.total.count, 800_000)
        self.assertTrue(page.total.over)
        self.assertIs(page.preview_result()["ok"], True, "参数本身是合法的")
        self.assertFalse(page.grid_group["total_hint"].isHidden(),
                         "网格组的总点数提示要可见")
        self.assertFalse(page.repair_button.isEnabled(), "超上限不能点修复（Q2）")
        self.assertTrue(page.locate_button.isEnabled(), "定位段不受这条约束")

        # 硬守卫：就算按钮被别处打开，提交也要被拦住
        page.repair_button.setEnabled(True)
        self.assertIsNone(page.start_repair(), "硬守卫要拦住提交，不靠按钮置灰")
        # 详细原因在结果区状态行（红字），状态栏给一句话
        self.assertIn("800000", page.status_label.text())
        self.assertIn("上限", page.status_label.text())
        self.assertIn("color:#c0392b", page.status_label.styleSheet())
        self.assertIn("未提交修复", page.last_status)
        self.assertEqual(page.run_state, "idle", "没提交就不得进 running")

    def test_back_under_the_limit_reenables_repair(self):
        page = self.page
        page.fields["grid_explicit"].setChecked(True)
        page.fields["xs"].setValue(200)
        page.fields["ys"].setValue(200)
        self.assertTrue(page.total.over)
        self.assertFalse(page.repair_button.isEnabled())
        page.fields["xs"].setValue(2)
        page.fields["ys"].setValue(5)
        page.fields["zs"].setValue(5)
        self.assertEqual(page.total.count, 50)
        self.assertFalse(page.total.over)
        self.assertTrue(page.repair_button.isEnabled(), "改小以后要能重新点")


class ControlGatewayTest(unittest.TestCase):
    """S4 建议 7：控件 → params 的网关（把“控件成了摆设”挡在测试里）。

    逐个控件改值，断言变更真的落到了 :meth:`RepairPage.params` 的产物里。
    两个“按设计不会变”的：

    * ``perturb_a``：三轴在 params 里是**一个键** ``perturb_deg``，改 A 只动
      下标 0——所以要看的是 ``perturb_deg[0]``，而不是“有没有出现新键”；
    * ``target_count``：只在 ``grid_mode=count`` 时提交；手填格点模式下按设计
      忽略它（params 里连这个键都没有，与网页版 ``collect()`` 一致）。

    互斥的两组（两点/六数、目标点数/手填）分两轮跑；每轮末尾都回查一遍
    “``fields`` 里每个控件都在本测试里出现过”，新增控件没接线就会红。
    """

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="repair-gateway-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = repair_page.RepairPage(plot=None, bridge=self.bridge)
        self.covered = {"domain_points", "domain_axes", "grid_count",
                        "grid_explicit", "euler", "robot"}

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=30)
        shutil.rmtree(self.root, ignore_errors=True)

    def _apply(self, cases):
        """逐条改控件、对 params。"""
        for field, value, path in cases:
            with self.subTest(field=field):
                widget = self.page.fields[field]
                if isinstance(widget, QtWidgets.QCheckBox):
                    widget.setChecked(bool(value))
                elif isinstance(widget, QtWidgets.QLineEdit):
                    widget.setText(str(value))
                else:
                    widget.setValue(value)
                params = self.page.params()
                expected = value - 1 if field in ("xs", "ys", "zs") else value
                self.assertEqual(expected_params_value(params, path), expected,
                                 f"{field} 没进 params")
                self.covered.add(field)

    def test_every_control_reaches_params(self):
        page = self.page
        # ---- 第一轮：两点输入 + 手填格点数 ----
        page.fields["domain_points"].setChecked(True)
        page.fields["grid_explicit"].setChecked(True)
        self.assertIn("point_min", page.params())
        self.assertNotIn("aabb6", page.params())
        self.assertIn("xs", page.params())
        self.assertNotIn("target_count", page.params(),
                         "手填格点模式下 target_count 按设计不提交")
        self._apply([
            ("rpy_a", 11.5, ("rpy", 0)),
            ("rpy_b", 12.5, ("rpy", 1)),
            ("rpy_c", -13.5, ("rpy", 2)),
            ("perturb_a", 6.0, ("perturb_deg", 0)),
            ("perturb_b", 7.0, ("perturb_deg", 1)),
            ("perturb_c", 8.0, ("perturb_deg", 2)),
            ("seed", 7, "seed"),
            ("j5_min", 12.5, "j5_min"),
            ("j3_min", 6.5, "j3_min"),
            ("seg_min", 4.5, "seg_min"),
            ("cond_max", 12.25, "cond_max"),
            ("seg_samples", 17, "seg_samples"),
            ("q_seed", "1,2,3,4,5,6", "q_seed"),
            ("stopped_at", 6, "stopped_at"),
            ("allow_rpy", False, "allow_rpy"),
            ("allow_shrink", False, "allow_shrink"),
            ("avoid_margin", 12.5, "avoid_margin"),
            ("avoid_radius", 95.0, "avoid_radius"),
            ("repair_attempts", 33, "repair_attempts"),
            ("max_candidates", 77, "max_candidates"),
            ("segment_window", 5, "segment_window"),
            ("min_x", 800.0, ("point_min", 0)),
            ("min_y", -50.0, ("point_min", 1)),
            ("min_z", 400.0, ("point_min", 2)),
            ("max_x", 1400.0, ("point_max", 0)),
            ("max_y", 50.0, ("point_max", 1)),
            ("max_z", 900.0, ("point_max", 2)),
            ("xs", 4, "xs"),
            ("ys", 6, "ys"),
            ("zs", 3, "zs"),
            ("z_insert", 2, "z_insert"),
        ])

        # ---- 第二轮：六个轴边界 + 目标点数 ----
        page.fields["domain_axes"].setChecked(True)
        page.fields["grid_count"].setChecked(True)
        self.assertIn("aabb6", page.params())
        self.assertNotIn("point_min", page.params())
        self.assertIn("target_count", page.params())
        self.assertNotIn("xs", page.params())
        self._apply([
            ("x_min", 850.0, ("aabb6", 0)),
            ("x_max", 1150.0, ("aabb6", 1)),
            ("y_min", -80.0, ("aabb6", 2)),
            ("y_max", 80.0, ("aabb6", 3)),
            ("z_min", 480.0, ("aabb6", 4)),
            ("z_max", 720.0, ("aabb6", 5)),
            ("target_count", 37, "target_count"),
        ])

        # ---- 下拉框：欧拉约定 / 机器人模型 ----
        euler = page.fields["euler"]
        euler.setCurrentIndex((euler.currentIndex() + 1) % euler.count())
        self.assertEqual(page.params()["euler"], euler.currentData())
        robot = page.fields["robot"]
        if robot.count() > 1:
            robot.setCurrentIndex(robot.count() - 1)
        self.assertEqual(page.params()["robot"], robot.currentData())

        # ---- 动态行：6 个关节角 ----
        for joint in range(6):
            page.q_rows[0]["fields"][joint].setValue(float(joint) + 2.5)
        self.assertEqual(page.params()["singular_q"][0],
                         [2.5, 3.5, 4.5, 5.5, 6.5, 7.5])
        self.covered.add("singular_q")

        # ---- “没进 params 的控件 = 摆设”反查 ----
        missing = set(page.fields) - self.covered
        self.assertEqual(missing, set(), f"这些控件没进 params：{sorted(missing)}")

    def test_radio_buttons_switch_which_group_is_submitted(self):
        """四个互斥组：只提交当前激活的那一组（两个都塞 core 会报二选一）。"""
        page = self.page
        page.fields["domain_axes"].setChecked(True)
        page.fields["grid_explicit"].setChecked(True)
        self.assertEqual({"aabb6", "xs", "ys", "zs", "z_insert"}
                         - set(page.params()), set())
        page.fields["domain_points"].setChecked(True)
        page.fields["grid_count"].setChecked(True)
        params = page.params()
        self.assertEqual({"point_min", "point_max", "target_count"}
                         - set(params), set())
        self.assertNotIn("aabb6", params)
        self.assertNotIn("z_insert", params)


if __name__ == "__main__":
    unittest.main()
