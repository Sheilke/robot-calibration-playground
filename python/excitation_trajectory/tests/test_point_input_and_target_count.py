#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两点式 AABB 输入、--target-count 自动凑数、--robot-config 别名。

三者都是 CLI 层面的便利性：不改变路点生成与 IK 的任何数值行为，
因此测试重点放在「归一化后的 AABB/网格与原写法完全等价」。
"""
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from excitation_trajectory.aabb_snake_scan import (
    COUNT_GRID_MAX_PER_AXIS,
    build_arg_parser,
    choose_grid,
    grid_point_count,
    main,
    plan_waypoints,
)

REFERENCE_RPY = (180.0, 0.0, -90.0)


def _parse(argv):
    """解析并归一化参数（main 内部走同一套校验，这里手动调一次）。"""
    parser = build_arg_parser()
    args = parser.parse_args([str(v) for v in argv])
    from excitation_trajectory.aabb_snake_scan import _validate_args
    _validate_args(parser, args)
    return args


class GridCountTest(unittest.TestCase):
    """点数公式：格点 (xs+1)(ys+1)(zs+1) + 换层插点 zs*z_insert。"""

    def test_formula_matches_plan_waypoints(self):
        for xs, ys, zs, ins in ((1, 1, 1, 0), (4, 4, 1, 0), (3, 3, 2, 1),
                                (2, 3, 3, 2), (0, 0, 0, 0)):
            waypoints = plan_waypoints(0, 200, 0, 200, 0, 200, xs, ys, zs,
                                       REFERENCE_RPY, z_insert=ins)
            self.assertEqual(len(waypoints), grid_point_count(xs, ys, zs, ins),
                             f"xs={xs} ys={ys} zs={zs} insert={ins}")

    def test_known_delivered_cases(self):
        # 交付那批：48 格点 + 2 个换层插点 = 50
        self.assertEqual(grid_point_count(3, 3, 2, 1), 50)
        # 纯网格凑 50：5*5*2
        self.assertEqual(grid_point_count(4, 4, 1), 50)


class ChooseGridTest(unittest.TestCase):
    """--target-count：必须精确命中，且优先纯网格。"""

    def test_hits_target_exactly(self):
        dims = (550.0, 1200.0, 670.0)
        for target in (8, 18, 20, 24, 27, 30, 36, 48, 50, 60, 64, 100):
            xs, ys, zs, ins = choose_grid(target, dims)
            self.assertEqual(grid_point_count(xs, ys, zs, ins), target,
                             f"target={target} 得到 {xs},{ys},{zs},{ins}")

    def test_prefers_pure_grid_over_inserts(self):
        """50 点有纯网格解（5x5x2 等），不应选中带插点的组合。"""
        xs, ys, zs, ins = choose_grid(50, (550.0, 1200.0, 670.0))
        self.assertEqual(ins, 0, f"应优先纯网格，实际 z_insert={ins}")

    def test_uses_inserts_when_no_pure_grid(self):
        """17 是素数、已超过单轴份数上限，无纯网格解；靠插点才能精确凑出。"""
        target = 17
        xs, ys, zs, ins = choose_grid(target, (500.0, 500.0, 500.0))
        self.assertEqual(grid_point_count(xs, ys, zs, ins), target)
        self.assertGreater(ins, 0, "17 无纯网格解，应出现插点")

    def test_single_point(self):
        self.assertEqual(choose_grid(1, (100.0, 100.0, 100.0)), (0, 0, 0, 0))

    def test_rejects_non_positive_target(self):
        with self.assertRaises(ValueError):
            choose_grid(0, (100.0, 100.0, 100.0))
        with self.assertRaises(ValueError):
            choose_grid(-5, (100.0, 100.0, 100.0))

    def test_rejects_impossible_target(self):
        # 素数且超过单轴上限 -> 无法精确凑出
        with self.assertRaises(ValueError):
            choose_grid(99991, (100.0, 100.0, 100.0))

    def test_respects_axis_cap(self):
        xs, ys, zs, ins = choose_grid(50, (550.0, 1200.0, 670.0))
        for seg in (xs, ys, zs):
            self.assertLessEqual(seg, COUNT_GRID_MAX_PER_AXIS)


class PointInputTest(unittest.TestCase):
    """--point-min/--point-max 与 6 个交错数值必须完全等价。"""

    def test_two_points_equal_six_numbers(self):
        by_axes = _parse(["750", "1300", "-600", "600", "270", "940"])
        by_points = _parse(["--point-min", "750", "-600", "270",
                            "--point-max", "1300", "600", "940"])
        self.assertEqual(
            (by_axes.x_min, by_axes.x_max, by_axes.y_min, by_axes.y_max,
             by_axes.z_min, by_axes.z_max),
            (by_points.x_min, by_points.x_max, by_points.y_min,
             by_points.y_max, by_points.z_min, by_points.z_max))
        self.assertEqual(by_axes.aabb_source, "axes")
        self.assertEqual(by_points.aabb_source, "points")

    def test_swapped_corners_are_normalised(self):
        args = _parse(["--point-min", "1300", "600", "940",
                       "--point-max", "750", "-600", "270"])
        self.assertEqual(
            (args.x_min, args.x_max, args.y_min, args.y_max,
             args.z_min, args.z_max),
            (750.0, 1300.0, -600.0, 600.0, 270.0, 940.0))

    def test_rejects_mixing_forms(self):
        for argv in (["1", "2", "1", "2", "1", "2", "--point-min", "1", "2", "3"],
                     ["--point-min", "1", "2", "3"]):
            with self.assertRaises(SystemExit):
                _parse(argv)

    def test_rejects_missing_aabb(self):
        with self.assertRaises(SystemExit):
            _parse([])

    def test_rejects_wrong_positional_count(self):
        with self.assertRaises(SystemExit):
            _parse(["1", "2", "3", "4", "5"])


class TargetCountCliTest(unittest.TestCase):
    """--target-count 的 CLI 行为与互斥规则。"""

    def test_cli_emits_exact_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "scan.json"
            buffer = io.StringIO()
            with redirect_stdout(buffer):
                code = main(["--point-min", "750", "-600", "270",
                             "--point-max", "1300", "600", "940",
                             "--target-count", "50", "--skip-ik",
                             "--no-cond-check",
                             "--out", str(out),
                             "--plot", str(Path(tmp) / "p.png")])
            self.assertEqual(code, 0)
            payload = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(len(payload["waypoints"]), 50)
            self.assertEqual(payload["meta"]["grid_source"], "target-count")
            self.assertEqual(payload["meta"]["target_count"], 50)
            self.assertEqual(payload["meta"]["aabb_source"], "points")
            self.assertIn("自动凑数", buffer.getvalue())

    def test_rejects_with_explicit_grid(self):
        for extra in (["--xs", "3"], ["--ys", "3"], ["--zs", "3"],
                      ["--z-insert", "1"]):
            with self.assertRaises(SystemExit):
                _parse(["1", "2", "1", "2", "1", "2",
                        "--target-count", "50"] + extra)


class RobotConfigAliasTest(unittest.TestCase):
    """--robot-config 是 --urdf 的等价别名。"""

    def test_alias_sets_same_destination(self):
        path = "data/configs/ch2000_origin.urdf"
        canonical = _parse(["1", "2", "1", "2", "1", "2", "--urdf", path])
        alias = _parse(["1", "2", "1", "2", "1", "2",
                        "--robot-config", path])
        self.assertEqual(canonical.urdf, alias.urdf)
        self.assertEqual(alias.urdf, path)


if __name__ == "__main__":
    unittest.main()
