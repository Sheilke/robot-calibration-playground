# -*- coding: utf-8 -*-
"""``laser_web.core`` 的单元测试：纯函数，不 import 任何 UI 库。

覆盖（与设计文档 §9 的“单测”一栏对应）：

1. ``list_robots`` 能找到已知模型，且不返回不存在的路径；
2. 两点式与六数式的预览结果完全一致；
3. ``target_count=50`` 的网格与 ``choose_grid`` 的真实返回值一致；
4. 非法输入返回 ``ok: False`` + 报错原文，不抛异常；
5. 体积与手算一致（界面上显示的是三边长度，体积仍留在结果里）；
6. ``form_to_namespace`` 的缺失项填的是 CLI 默认值。
"""

import unittest
from pathlib import Path

from excitation_trajectory.aabb_snake_scan import build_arg_parser, choose_grid
from laser_web import core

#: 仓库根（``<repo>/python/laser_web/tests/test_core.py`` 往上三级）。
REPO_ROOT = Path(__file__).resolve().parents[3]

#: 设计文档 §5.1 的示例采样域，550 x 1200 x 670 mm = 0.4422 m³。
POINT_MIN = [750.0, -600.0, 270.0]
POINT_MAX = [1300.0, 600.0, 940.0]
AABB6 = [750.0, 1300.0, -600.0, 600.0, 270.0, 940.0]


def _params(**overrides):
    """一份与生成页默认值等价的表单参数（每次返回新 dict）。"""
    params = {"robot": "data/configs/hyundai_general6s.urdf",
              "rpy": [180.0, 0.0, -90.0],
              "euler": "xyz-extrinsic",
              "perturb_deg": [5.0, 20.0, 15.0],
              "cond_max": 9.0}
    params.update(overrides)
    return params


def _point_params(**overrides):
    """两点式 + 目标点数 50。"""
    return _params(point_min=list(POINT_MIN), point_max=list(POINT_MAX),
                   target_count=50, **overrides)


def _axis_params(**overrides):
    """六数式 + 目标点数 50。"""
    return _params(aabb6=list(AABB6), target_count=50, **overrides)


class ListRobotsUsabilityTest(unittest.TestCase):
    """``list_robots`` 必须标出哪些文件真能当模型加载。

    ``data/configs`` 里混着 DH 覆盖文件、扫描记录和 robot_sim 仿真资产，它们
    缺 ``Link&CoupleCoe``，喂给 ``Nexbot6R`` 会 ``KeyError``；下拉框若把它们
    列进去，用户选了个跑不起来的文件要等到解算才炸。
    """

    def test_only_urdf_models_are_usable(self):
        robots = core.list_robots()
        by_label = {r["label"]: r for r in robots}
        for label in ("hyundai_general6s.urdf", "ch2000_origin.urdf"):
            self.assertIn(label, by_label)
            self.assertTrue(by_label[label]["usable"], label)
            self.assertIsNone(by_label[label]["reason"], label)

    def test_non_model_json_is_flagged_with_reason(self):
        by_label = {r["label"]: r for r in core.list_robots()}
        for label in ("hyundai_general6s_dh.json",
                      "hyundai_general6s_scan.json",
                      "robot_sim/ch2000_3sr_inverted_7axis_new.json"):
            self.assertIn(label, by_label)
            self.assertFalse(by_label[label]["usable"], label)
            self.assertTrue(by_label[label]["reason"], label)

    def test_probe_can_be_skipped(self):
        robots = core.list_robots(probe=False)
        self.assertTrue(all(r["usable"] for r in robots))
        self.assertTrue(all(r["reason"] is None for r in robots))


class ListRobotsTest(unittest.TestCase):
    def test_lists_known_models_and_existing_paths(self):
        """列出 data/configs 下的模型，路径全部真实存在，且按 path 稳定排序。"""
        robots = core.list_robots()
        paths = [robot["path"] for robot in robots]

        self.assertIn("data/configs/hyundai_general6s.urdf", paths)
        self.assertEqual(paths, sorted(paths), "path 排序必须稳定")
        for robot in robots:
            self.assertTrue((REPO_ROOT / robot["path"]).is_file(),
                            f"返回了不存在的路径: {robot['path']}")
            self.assertIn(robot["kind"], ("urdf", "json"))
            self.assertTrue(robot["label"])
        # 子目录（robot_sim/）里的模型也要列出来，label 带相对路径
        self.assertTrue(any(path.startswith("data/configs/robot_sim/")
                            for path in paths))
        self.assertTrue(any("/" in robot["label"]
                            for robot in robots
                            if robot["path"].startswith("data/configs/robot_sim/")))

    def test_repo_root_argument(self):
        """显式给 repo_root 等价于默认；指向没有 configs 的目录时返回空列表。"""
        self.assertEqual(core.list_robots(REPO_ROOT), core.list_robots())
        self.assertEqual(core.list_robots(REPO_ROOT / "python"), [])


class PreviewDomainTest(unittest.TestCase):
    def test_point_and_axis_inputs_agree(self):
        """两点式与六数式的 bounds / grid / point_count / design_condition 完全一致。"""
        points = core.preview_domain(_point_params())
        axes = core.preview_domain(_axis_params())

        self.assertTrue(points["ok"], points.get("error"))
        self.assertTrue(axes["ok"], axes.get("error"))
        for key in ("bounds", "grid", "point_count", "design_condition",
                    "warnings"):
            self.assertEqual(points[key], axes[key], key)
        self.assertEqual(points["bounds"], AABB6)

    def test_target_count_matches_choose_grid(self):
        """target_count=50 的网格就是 choose_grid(50, dims) 的真实返回值。"""
        result = core.preview_domain(_point_params())
        self.assertTrue(result["ok"], result.get("error"))

        dims = (AABB6[1] - AABB6[0], AABB6[3] - AABB6[2], AABB6[5] - AABB6[4])
        expected = choose_grid(50, dims)
        self.assertEqual(
            tuple(result["grid"][name] for name in ("xs", "ys", "zs", "z_insert")),
            expected)
        # 设计文档里的例子：2x5x5 = 50 点
        self.assertEqual(result["grid"],
                         {"xs": 1, "ys": 4, "zs": 4, "z_insert": 0})
        self.assertEqual(result["point_count"], 50)

    def test_volume_matches_hand_calculation(self):
        """550 x 1200 x 670 mm³ = 0.4422 m³（卡片按 3 位小数显示 0.442）。"""
        result = core.preview_domain(_axis_params())
        self.assertTrue(result["ok"], result.get("error"))
        expected = 550.0 * 1200.0 * 670.0 / 1e9
        self.assertAlmostEqual(result["volume_m3"], expected, delta=1e-3)
        self.assertAlmostEqual(result["volume_m3"], 0.4422, delta=1e-3)
        # 预览卡显示三边（米）而不是体积：现场拿卷尺量的是 X/Y/Z
        self.assertEqual(result["edges_m"], [0.55, 1.2, 0.67])

    def test_invalid_inputs_return_error_without_raising(self):
        """非法输入返回 ok=False 与 CLI 报错原文，绝不抛异常。"""
        cases = {
            "x 下界大于上界": (
                _params(aabb6=[1300.0, 750.0, -600.0, 600.0, 270.0, 940.0]),
                "大于上界"),
            "target_count 与 xs/ys/zs 互斥": (
                _point_params(xs=1, ys=4, zs=4), "互斥"),
            "六数与两点互斥": (
                _params(aabb6=list(AABB6), point_min=list(POINT_MIN),
                        point_max=list(POINT_MAX)), "不能同时给"),
            "没有任何边界": (_params(), "需要 6 个 AABB 数值"),
            "cond_max 小于 1": (
                _axis_params(cond_max=0.5), "cond-max 必须 >= 1"),
            "扰动半宽为 0": (
                _axis_params(perturb_deg=[0.0, 20.0, 15.0]), "必须为有限正数"),
            "rpy 有空项": (
                _axis_params(rpy=[180.0, None, -90.0]), "有空项"),
        }
        for name, (params, expected_text) in cases.items():
            with self.subTest(name):
                result = core.preview_domain(params)      # 不许抛异常
                self.assertIs(result["ok"], False)
                self.assertIn(expected_text, result["error"])
                self.assertIsInstance(result["warnings"], list)


class FormToNamespaceTest(unittest.TestCase):
    def test_missing_keys_fall_back_to_cli_defaults(self):
        """空表单拿到的就是 CLI 的默认值（不在这里另抄一份默认值）。"""
        args = core.form_to_namespace({})
        cli_defaults = build_arg_parser().parse_args([])
        self.assertEqual(sorted(vars(args)), sorted(vars(cli_defaults)))
        for name in ("rpy", "xs", "ys", "zs", "target_count", "z_insert",
                     "j5_min", "j3_min", "cond_max", "perturb_deg", "euler",
                     "urdf", "seed", "seg_mode", "seg_samples", "seg_min"):
            self.assertEqual(getattr(args, name), getattr(cli_defaults, name),
                             name)

    def test_form_values_are_mapped(self):
        """表单值落到对应的 CLI 字段上（含 robot -> urdf、euler、AABB）。"""
        args = core.form_to_namespace(
            _params(aabb6=list(AABB6), target_count=50,
                    robot="data/configs/ch2000_origin.urdf"))
        self.assertEqual(args.aabb, AABB6)
        self.assertEqual(args.target_count, 50)
        self.assertEqual(args.urdf, "data/configs/ch2000_origin.urdf")
        self.assertEqual(args.euler, "xyz-extrinsic")
        self.assertEqual(args.perturb_deg, [5.0, 20.0, 15.0])
        # 归一化（x_min..z_max / euler_convention）由 _validate_args 完成
        self.assertFalse(hasattr(args, "x_min"))

class PreviewStabilityTest(unittest.TestCase):
    """同一份表单值两次预览必须相等（页面按参数缓存预览结果的前提）。"""

    def test_same_params_give_equal_preview(self):
        first = core.preview_domain(_point_params())
        second = core.preview_domain(_point_params())
        self.assertTrue(first["ok"], first.get("error"))
        self.assertEqual(first, second)


class SummaryStatsTest(unittest.TestCase):
    """``_summary_stats``：守卫违规必须让 ``feasible`` 变 False。"""

    def _args(self, skip_ik: bool = False):
        args = build_arg_parser().parse_args([])
        args.skip_ik = skip_ik
        return args

    def _step(self, j5: float = 30.0):
        from excitation_trajectory import aabb_snake_scan as scan

        return scan.ChainStep(index=0, q=[0.0, 0.0, 20.0, 0.0, j5, 0.0],
                              n_sols=1, config=(1, 0, 0), dq_l2=0.0,
                              dq_l1=0.0, ok=True)

    def _guard(self, violations):
        return {"violations": violations, "resampled": [],
                "j5_abs_min_deg": 30.0, "j3_abs_min_deg": 20.0}

    def _conditioning(self):
        from excitation_trajectory import aabb_snake_scan as scan

        return scan._conditioning_to_json({})

    def test_no_violation_is_feasible(self):
        stats = core._summary_stats(self._args(), self._conditioning(),
                                    [self._step()], self._guard([]), None)
        self.assertEqual(stats["guard_violations"], 0)
        self.assertTrue(stats["feasible"])

    def test_violation_is_infeasible(self):
        """重采后仍低于阈值 = 算法判不可行，徽标不能是绿的。"""
        stats = core._summary_stats(
            self._args(), self._conditioning(), [self._step()],
            self._guard([{"index": 0}]), None)
        self.assertEqual(stats["guard_violations"], 1)
        self.assertFalse(stats["feasible"])

    def test_skip_ik_leaves_feasible_none(self):
        """``skip_ik`` 没跑 IK，``feasible`` 只能是 ``None``（不是 False）。"""
        stats = core._summary_stats(self._args(skip_ik=True),
                                    self._conditioning(), [], None, None)
        self.assertIsNone(stats["feasible"])
        self.assertEqual(stats["guard_violations"], 0)


if __name__ == "__main__":
    unittest.main()
