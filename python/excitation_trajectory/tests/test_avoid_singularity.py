#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""奇异点驱动自动重规划：输入解析、分类、决策与端到端行为。

端到端用例刻意用 8 点（2x2x2）网格，保证测试秒级完成；50 点的完整流程与
``aabb_snake_scan`` 共用同一套函数，已由那边的测试覆盖。
"""
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from excitation_trajectory.aabb_snake_scan import (
    _expm_so3,
    check_segments_cartesian,
    interpolate_pose,
)
from excitation_trajectory.avoid_singularity import (
    _describe_change,
    repair_segment,
    segment_profiles,
    change_cost,
    classify_singularity,
    domain_center,
    domain_volume,
    failure_score,
    flatten_joint_angles,
    main,
    make_domain,
)
from robot_model.nexbot_ik import Nexbot6R

REPO = Path(__file__).resolve().parents[3]
ROBOT = REPO / "data" / "configs" / "hyundai_general6s.urdf"
Q_SEED = "43,-30,-36,42,-89,-1"
BOX = ["--point-min", "900", "-100", "500",
       "--point-max", "1100", "100", "700"]


def _robot():
    return Nexbot6R(str(ROBOT))


class FlattenJointAnglesTest(unittest.TestCase):
    """--singular-q 的两种写法必须等价，非法输入要报错。"""

    def test_six_numbers(self):
        self.assertEqual(flatten_joint_angles([[0, 0, 0, 55.721, 0, -55.721]]),
                         [[0.0, 0.0, 0.0, 55.721, 0.0, -55.721]])

    def test_comma_string(self):
        self.assertEqual(flatten_joint_angles([["0,0,0,55.721,0,-55.721"]]),
                         [[0.0, 0.0, 0.0, 55.721, 0.0, -55.721]])

    def test_multiple_groups(self):
        groups = flatten_joint_angles([["1,2,3,4,5,6"], [10, 20, 30, 40, 50, 60]])
        self.assertEqual(len(groups), 2)
        self.assertEqual(groups[1], [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])

    def test_rejects_wrong_count(self):
        for bad in ([["1,2,3"]], [[1, 2, 3, 4, 5]], [["1,2,3,4,5,6,7"]]):
            with self.assertRaises(ValueError):
                flatten_joint_angles(bad)

    def test_rejects_non_numeric_or_empty(self):
        for bad in ([["a,b,c,d,e,f"]], [[]], [[""]]):
            with self.assertRaises(ValueError):
                flatten_joint_angles(bad)

    def test_rejects_non_finite(self):
        with self.assertRaises(ValueError):
            flatten_joint_angles([["0,0,0,0,nan,0"]])


class ClassifyTest(unittest.TestCase):
    """奇异分类与有效阈值上抬（只上抬、不下调）。"""

    def test_wrist_singularity_raises_j5_floor(self):
        info = classify_singularity(_robot(), [0, -30, -60, 0, 15.0, 0],
                                    j5_min=20.0, j3_min=0.0,
                                    avoid_margin=10.0)
        self.assertEqual(info["kind"], "wrist")
        self.assertAlmostEqual(info["j5_abs"], 15.0, places=6)
        self.assertAlmostEqual(info["j5_floor"], 25.0, places=6)   # 15 + 10 > 20
        self.assertEqual(info["j3_floor"], 0.0)

    def test_wrist_floor_never_below_builtin_threshold(self):
        """实测裕度很小（3°+10°=13°）时，仍以内置 20° 为准。"""
        info = classify_singularity(_robot(), [0, -30, -60, 0, 3.0, 0],
                                    j5_min=20.0, j3_min=0.0,
                                    avoid_margin=10.0)
        self.assertEqual(info["kind"], "wrist")
        self.assertAlmostEqual(info["j5_floor"], 20.0, places=6)

    def test_elbow_singularity_raises_j3_floor(self):
        info = classify_singularity(_robot(), [0, -30, 2.0, 0, 60.0, 0],
                                    j5_min=20.0, j3_min=5.0,
                                    avoid_margin=10.0)
        self.assertEqual(info["kind"], "elbow")
        self.assertAlmostEqual(info["j3_floor"], 12.0, places=6)   # 2 + 10

    def test_threshold_never_lowered(self):
        """实测裕度很大时不能把阈值降到内置值以下。"""
        info = classify_singularity(_robot(), [0, -30, -60, 0, 60.0, 0],
                                    j5_min=20.0, j3_min=0.0,
                                    avoid_margin=10.0)
        self.assertEqual(info["kind"], "unexplained")
        self.assertEqual(info["j5_floor"], 20.0)
        self.assertEqual(info["j3_floor"], 0.0)

    def test_position_is_fk_of_reported_q(self):
        rbt = _robot()
        q = [10.0, -20.0, -40.0, 5.0, 50.0, 7.0]
        info = classify_singularity(rbt, q, j5_min=20.0, j3_min=0.0,
                                    avoid_margin=10.0)
        expected = rbt.fk(q)[:3, 3]
        for got, want in zip(info["position"], expected):
            self.assertAlmostEqual(got, float(want), places=6)


class FieldStopRegressionTest(unittest.TestCase):
    """2026-11-05 现场那次停机（约第 41 点、运动中）的回归。

    控制器停在 ``[0,0,0,55.721,0,-55.721]``：J5=0 时 J4/J6 共线，只有 J4+J6
    有意义，该姿态与零位 q=0 完全等价——不能据此定位、也不能判成腕奇异，
    真正越界的是插补途中的肘部（J3 穿 0）。这三条必须钉死。
    """

    STOP_Q = [0.0, 0.0, 0.0, 55.721, 0.0, -55.721]

    def test_stop_pose_equals_zero_pose(self):
        import numpy as np
        rbt = _robot()
        stop, zero = rbt.fk(self.STOP_Q), rbt.fk([0.0] * 6)
        np.testing.assert_allclose(stop[:3, 3], zero[:3, 3], atol=1e-9)
        np.testing.assert_allclose(stop[:3, :3], zero[:3, :3], atol=1e-12)

    def test_classified_degenerate_not_wrist(self):
        info = classify_singularity(_robot(), self.STOP_Q, j5_min=20.0,
                                    j3_min=0.0, avoid_margin=10.0)
        self.assertEqual(info["kind"], "degenerate")
        self.assertTrue(info["same_as_zero_pose"])
        # 退化读数必须同时收紧肘部裕度，而不是只提 |J5|
        self.assertGreaterEqual(info["j3_floor"], 10.0)
        self.assertIn("J4+J6", info["note"])

    def test_degenerate_does_not_create_forbidden_sphere(self):
        """退化位形的位置无意义，不能变成禁区球（否则会去躲零位）。"""
        with tempfile.TemporaryDirectory() as tmp:
            argv = ["--robot", str(ROBOT)] + BOX + [
                "--rpy", "3", "73", "-167", "--euler", "xyz-intrinsic",
                "--q-seed", Q_SEED, "--target-count", "8",
                "--singular-q"] + [f"{v}" for v in self.STOP_Q] + [
                "--out", str(Path(tmp) / "s.json"),
                "--report", str(Path(tmp) / "r.json"),
                "--j5-attempts", "10"]
            with redirect_stdout(io.StringIO()):
                code = main(argv)
            self.assertEqual(code, 0)
            report = json.loads((Path(tmp) / "r.json").read_text(encoding="utf-8"))
            self.assertEqual(report["forbidden_positions"], [])
            self.assertEqual(report["singularity_analysis"][0]["kind"],
                             "degenerate")

    def test_elbow_suspected_even_without_j3_min(self):
        """``--j3-min`` 未启用时，|J3| 贴 0 也要按肘奇异收紧。"""
        info = classify_singularity(_robot(), [0, -30, 1.5, 0, 60, 0],
                                    j5_min=20.0, j3_min=0.0,
                                    avoid_margin=10.0)
        self.assertEqual(info["kind"], "elbow")
        self.assertAlmostEqual(info["j3_floor"], 11.5, places=6)


class DomainTest(unittest.TestCase):
    """域的量度与改动代价。"""

    def test_volume_and_center(self):
        domain = make_domain([0, 100, 0, 200, 0, 300], [0, 0, 0], (1, 1, 1))
        self.assertAlmostEqual(domain_volume(domain), 100 * 200 * 300)
        self.assertEqual(domain_center(domain), [50.0, 100.0, 150.0])

    def test_cost_grows_with_change(self):
        base = make_domain([0, 100, 0, 100, 0, 100], [0, 0, 0], (1, 1, 1))
        near = make_domain([0, 100, 0, 100, 0, 100], [0, 0, 0], (1, 1, 1))
        far = make_domain([0, 100, 0, 100, 0, 100], [0, 5, 0], (1, 1, 1))
        self.assertAlmostEqual(change_cost(base, near), 0.0)
        self.assertGreater(change_cost(base, far), 0.0)

    def test_failure_score_orders_severity(self):
        mild = {"violations": 1, "j5_deficit": 1.0}
        bad = {"unsolved": 1}
        self.assertLess(failure_score(mild), failure_score(bad))
        self.assertEqual(failure_score({}), 0.0)

    def test_describe_change_recovers_translate(self):
        base = make_domain([0, 100, 0, 100, 0, 100], [0, 0, 0], (1, 1, 1))
        shifted = make_domain([0, 100, 50, 150, 0, 100], [0, 2, 0], (1, 1, 1))
        info = _describe_change(base, shifted)
        kinds = {(m["type"], m["axis"]) for m in info["moves"]}
        self.assertIn(("translate", "Y"), kinds)
        self.assertIn(("rpy", "B"), kinds)
        self.assertIn("Y 平移 +50 mm", info["label"])

    def test_describe_change_recovers_shrink(self):
        base = make_domain([0, 100, 0, 100, 0, 100], [0, 0, 0], (1, 1, 1))
        shrunk = make_domain([0, 100, 0, 80, 0, 100], [0, 0, 0], (1, 1, 1))
        info = _describe_change(base, shrunk)
        self.assertEqual(info["moves"],
                         [{"type": "shrink", "axis": "Y", "side": "max",
                           "delta_mm": -20.0}])
        self.assertIn("Y max 内收 20 mm", info["label"])


class MovlInterpolationTest(unittest.TestCase):
    """MOVL（笛卡尔直线）路径模型：位置线性 + 姿态测地线插补。"""

    @staticmethod
    def _pose(position, rpy):
        from robot_model.nexbot_ik import euler_to_rot
        transform = __import__("numpy").eye(4)
        transform[:3, :3] = euler_to_rot(rpy, order="XYZ", extrinsic=False)
        transform[:3, 3] = position
        return transform

    @staticmethod
    def _angle(R1, R2):
        import numpy as np
        return float(np.degrees(np.arccos(max(
            -1.0, min(1.0, (float(np.trace(R1.T @ R2)) - 1.0) / 2.0)))))

    def test_endpoints_match(self):
        import numpy as np
        start = self._pose((100, 0, 0), (10, 20, 30))
        end = self._pose((300, 100, 50), (-40, 25, 80))
        np.testing.assert_allclose(interpolate_pose(start, end, 0.0), start,
                                   atol=1e-12)
        np.testing.assert_allclose(interpolate_pose(start, end, 1.0), end,
                                   atol=1e-12)

    def test_position_is_linear(self):
        import numpy as np
        start = self._pose((100, 0, 0), (10, 20, 30))
        end = self._pose((300, 100, 50), (-40, 25, 80))
        mid = interpolate_pose(start, end, 0.5)
        np.testing.assert_allclose(mid[:3, 3], (start[:3, 3] + end[:3, 3]) / 2,
                                   atol=1e-12)

    def test_orientation_splits_the_geodesic(self):
        """frac=0.5 时两端张角相等；姿态保持正交、det=1。"""
        import numpy as np
        start = self._pose((0, 0, 0), (10, 20, 30))
        end = self._pose((0, 0, 0), (-40, 25, 80))
        mid = interpolate_pose(start, end, 0.5)
        R = mid[:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        self.assertAlmostEqual(self._angle(start[:3, :3], R),
                               self._angle(R, end[:3, :3]), places=9)

    def test_expm_matches_axis_rotation(self):
        import numpy as np
        from robot_model.nexbot_ik import rot_z
        np.testing.assert_allclose(_expm_so3([0, 0, np.pi / 2]), rot_z(np.pi / 2),
                                   atol=1e-12)


class CartesianSegmentCheckTest(unittest.TestCase):
    """MOVL 段检查的输出结构与可复现性。"""

    def _plan(self, bounds):
        from excitation_trajectory.aabb_snake_scan import (
            _sample_perturbations, plan_waypoints, solve_chain_guarded)
        rbt = _robot()
        x0, x1, y0, y1, z0, z1 = bounds
        waypoints = plan_waypoints(x0, x1, y0, y1, z0, z1, 1, 1, 1,
                                   (3.0, 73.0, -167.0))
        _sample_perturbations(waypoints, 0, (20, 20, 35))
        steps, _ = solve_chain_guarded(
            rbt, waypoints, [43, -30, -36, 42, -89, -1], j5_min=20.0,
            attempts=10, seed=0, amplitudes_deg=(20, 20, 35),
            order="XYZ", extrinsic=False)
        return rbt, waypoints, steps

    def test_reports_mode_and_margins(self):
        rbt, waypoints, steps = self._plan((900, 1100, -100, 100, 500, 700))
        report = check_segments_cartesian(rbt, waypoints, steps, samples=9,
                                          order="XYZ", extrinsic=False)
        self.assertEqual(report["mode"], "cartesian-movl")
        self.assertEqual(report["samples"], 9)
        self.assertIsNotNone(report["j5_abs_min"])
        self.assertEqual(report["ik_failures"], 0)
        self.assertEqual(len(report["segments"]), 7)   # 8 点 7 段
        for segment in report["segments"]:
            self.assertIn("max_jump_deg", segment)

    def test_deterministic(self):
        rbt, waypoints, steps = self._plan((900, 1100, -100, 100, 500, 700))
        first = check_segments_cartesian(rbt, waypoints, steps, samples=9,
                                         order="XYZ", extrinsic=False)
        second = check_segments_cartesian(rbt, waypoints, steps, samples=9,
                                          order="XYZ", extrinsic=False)
        self.assertEqual(first, second)


class SegmentLocateAndRepairTest(unittest.TestCase):
    """按"已到的点号"定位段，并做只换端点姿态的局部修复。"""

    def _plan(self, bounds=(750, 1300, -600, 600, 270, 940)):
        from excitation_trajectory.aabb_snake_scan import (
            _sample_perturbations, plan_waypoints, solve_chain_guarded)
        rbt = _robot()
        x0, x1, y0, y1, z0, z1 = bounds
        waypoints = plan_waypoints(x0, x1, y0, y1, z0, z1, 1, 4, 4,
                                   (3.0, 73.0, -167.0))
        _sample_perturbations(waypoints, 0, (20, 20, 35))
        steps, _ = solve_chain_guarded(
            rbt, waypoints, [43, -30, -36, 42, -89, -1], j5_min=20.0,
            attempts=30, seed=0, amplitudes_deg=(20, 20, 35),
            order="XYZ", extrinsic=False)
        return rbt, waypoints, steps

    def test_profiles_cover_window_and_report_gp_numbers(self):
        rbt, waypoints, steps = self._plan()
        profiles = segment_profiles(rbt, waypoints, steps, 40, window=2,
                                    order="XYZ", extrinsic=False, samples=9)
        # window=2 -> 以 40 为中心前后各 2 段，共 5 段
        self.assertEqual([p["index"] for p in profiles], [38, 39, 40, 41, 42])
        self.assertEqual(profiles[2]["gp_from"], 41)
        self.assertEqual(profiles[2]["gp_to"], 42)
        for profile in profiles:
            self.assertIsNotNone(profile["j3_abs_min"])
            self.assertEqual(profile["ik_failures"], 0)

    def test_repair_fixes_a_tampered_segment_without_moving_points(self):
        """把某点 J3 反号制造穿零，局部修复应只换该点姿态、位置不变。"""
        import copy
        rbt, waypoints, steps = self._plan()
        victim = 20
        from excitation_trajectory.aabb_snake_scan import ChainStep
        tampered = list(steps)
        flipped = list(steps[victim].q)
        flipped[2] = -flipped[2]
        tampered[victim] = ChainStep(index=steps[victim].index, q=flipped,
                                     n_sols=steps[victim].n_sols,
                                     config=steps[victim].config,
                                     dq_l2=steps[victim].dq_l2,
                                     dq_l1=steps[victim].dq_l1, ok=True)

        before = check_segments_cartesian(rbt, waypoints, tampered, samples=21,
                                          order="XYZ", extrinsic=False)
        self.assertGreater(before["j3_crossings"], 0, "人为反号应造成穿零")

        positions = [wp.pos for wp in waypoints]
        fixed, changed = repair_segment(
            rbt, waypoints, tampered, victim, order="XYZ", extrinsic=False,
            perturb_deg=(20, 20, 35), seed=0, attempts=80,
            j5_min=20.0, j3_min=10.0, seg_samples=21)
        self.assertIsNotNone(changed, "应能通过重采该点姿态修好")
        after = check_segments_cartesian(rbt, waypoints, fixed, samples=21,
                                         order="XYZ", extrinsic=False)
        self.assertEqual(after["j3_crossings"], 0)
        self.assertEqual([wp.pos for wp in waypoints], positions,
                         "局部修复不得移动任何点")
        self.assertEqual(len(fixed), len(steps))

    def test_repair_returns_none_when_nothing_helps(self):
        """阈值抬到不可能满足时，应老实返回 None 而不是假装成功。"""
        from excitation_trajectory.aabb_snake_scan import ChainStep
        rbt, waypoints, steps = self._plan()
        tampered = list(steps)
        flipped = list(steps[10].q)
        flipped[2] = -flipped[2]
        tampered[10] = ChainStep(index=steps[10].index, q=flipped,
                                 n_sols=steps[10].n_sols,
                                 config=steps[10].config,
                                 dq_l2=steps[10].dq_l2,
                                 dq_l1=steps[10].dq_l1, ok=True)
        _, changed = repair_segment(
            rbt, waypoints, tampered, 10, order="XYZ", extrinsic=False,
            perturb_deg=(20, 20, 35), seed=0, attempts=3,
            j5_min=1000.0, j3_min=0.0, seg_samples=9)
        self.assertIsNone(changed)


class EndToEndTest(unittest.TestCase):
    """CLI 端到端：已可行则不动，落进禁区则自己平移。"""

    def _run(self, extra, tmp):
        argv = ["--robot", str(ROBOT)] + BOX + [
            "--rpy", "3", "73", "-167", "--euler", "xyz-intrinsic",
            "--q-seed", Q_SEED, "--target-count", "8",
            "--out", str(Path(tmp) / "scan.json"),
            "--report", str(Path(tmp) / "rep.json"),
            "--j5-attempts", "10"] + extra
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = main(argv)
        return code, buffer.getvalue(), Path(tmp)

    def test_domain_already_ok_is_left_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, output, root = self._run(
                ["--singular-q", "0", "-30", "-2.0", "0", "60", "0"], tmp)
            self.assertEqual(code, 0, output)
            payload = json.loads((root / "scan.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["meta"]["repaired_domain"]["tier"], 0)
            self.assertEqual(payload["meta"]["aabb"][:2], [900.0, 1100.0])
            self.assertEqual(len(payload["waypoints"]), 8)
            self.assertIn("无需修改", output)

    def test_reported_singular_at_domain_corner_triggers_shift(self):
        # 用域角点的位形作为"奇异报告"：平移整箱即可拉开距离
        rbt = _robot()
        corner = (900.0, -100.0, 500.0)
        solutions = rbt.ik(rbt.pose_to_T(corner, (3.0, 73.0, -167.0),
                                        extrinsic=False))
        corner_q = solutions[0]
        with tempfile.TemporaryDirectory() as tmp:
            code, output, root = self._run(
                ["--singular-q"] + [f"{v:.6f}" for v in corner_q]
                + ["--avoid-radius", "100"], tmp)
            self.assertEqual(code, 0, output)
            payload = json.loads((root / "scan.json").read_text(encoding="utf-8"))
            self.assertNotEqual(payload["meta"]["repaired_domain"]["tier"], 0)
            self.assertTrue(payload["meta"]["repaired_domain"]["moves"])
            # 平移后所有路点都应离开禁区球
            shifts = [abs(a - b) for a, b in
                      zip(payload["meta"]["aabb"], [900, 1100, -100, 100, 500, 700])]
            self.assertGreater(max(shifts), 0.0)
            report = json.loads((root / "rep.json").read_text(encoding="utf-8"))
            self.assertIsNotNone(report["decision"])
            self.assertEqual(len(report["singularity_analysis"]), 1)

    def test_rejects_bad_joint_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            argv = ["--robot", str(ROBOT)] + BOX + [
                "--singular-q", "1,2,3",
                "--out", str(Path(tmp) / "s.json"),
                "--report", str(Path(tmp) / "r.json")]
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(argv), 2)


if __name__ == "__main__":
    unittest.main()
