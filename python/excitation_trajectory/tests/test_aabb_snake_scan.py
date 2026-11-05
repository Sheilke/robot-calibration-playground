"""连续 AABB 蛇形网格路点与姿态激励条件数的单元测试。

覆盖两类度量：
* ``design_condition_number`` —— 方案级解析设计条件数（与 seed 无关，
  轴对齐姿态下 ≈ a_max/a_min）；
* ``pose_condition_number`` —— 实测条件数（分块：姿态 rad / 位置 mm），
  合并 6D 值只作诊断。
门禁测试直接用 ``evaluate_conditioning``，不依赖 IK 求解。
"""

import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

from robot_model.nexbot_ik import Nexbot6R, euler_to_rot
from excitation_trajectory.aabb_snake_scan import (
    DEFAULT_COND_MAX,
    config_tag,
    PERTURB_AMPLITUDE_DEG,
    build_arg_parser,
    design_condition_number,
    evaluate_conditioning,
    main,
    plan_waypoints,
    pose_condition_number,
    rotation_vector,
)

REFERENCE_RPY = (180.0, 0.0, -90.0)


def _pose(abc, pos=(0.0, 0.0, 0.0)):
    """由 ABC 欧拉角（deg）与位置构造 4x4 位姿。"""
    transform = np.eye(4)
    transform[:3, :3] = euler_to_rot(abc)
    transform[:3, 3] = pos
    return transform


def _pose_from_rotvec(w, pos=(0.0, 0.0, 0.0)):
    """由旋转向量（轴*角，rad）经 Rodrigues 公式构造 4x4 位姿。"""
    w = np.asarray(w, dtype=float)
    angle = float(np.linalg.norm(w))
    transform = np.eye(4)
    if angle > 0.0:
        axis = w / angle
        skew = np.array([[0.0, -axis[2], axis[1]],
                         [axis[2], 0.0, -axis[0]],
                         [-axis[1], axis[0], 0.0]])
        transform[:3, :3] = (np.eye(3) + math.sin(angle) * skew
                             + (1.0 - math.cos(angle)) * (skew @ skew))
    transform[:3, 3] = pos
    return transform


def _perturbed_poses(amplitudes_deg, count=48, seed=0, positions=None):
    """绕三轴按给定半宽均匀扰动参考姿态，构造位姿集。"""
    rng = np.random.default_rng(seed)
    a_rx, a_ry, a_rz = amplitudes_deg
    poses = []
    for index in range(count):
        pos = (0.0, 0.0, 0.0) if positions is None else positions[index]
        poses.append(_pose((REFERENCE_RPY[0] + rng.uniform(-a_rx, a_rx),
                            REFERENCE_RPY[1] + rng.uniform(-a_ry, a_ry),
                            REFERENCE_RPY[2] + rng.uniform(-a_rz, a_rz)),
                           pos))
    return poses


class PlanWaypointsTest(unittest.TestCase):
    """验证网格覆盖和任意一步只沿一个网格轴移动。"""

    @staticmethod
    def _grid_indices(waypoints):
        """本测试用 0..N 的整数坐标将路点位置还原为网格下标。"""
        return [tuple(int(round(value)) for value in waypoint.pos)
                for waypoint in waypoints]

    def assert_adjacent_grid_steps(self, indices):
        for previous, current in zip(indices, indices[1:]):
            with self.subTest(previous=previous, current=current):
                self.assertEqual(
                    sum(abs(after - before)
                        for before, after in zip(previous, current)),
                    1,
                )

    def test_two_by_two_by_two_grid_is_unique_and_continuous(self):
        waypoints = plan_waypoints(
            0, 2, 0, 2, 0, 2, xs=2, ys=2, zs=2, rpy_ref=(0, 0, 0)
        )
        indices = self._grid_indices(waypoints)

        self.assertEqual(len(waypoints), 27)
        self.assertEqual(len(set(indices)), 27)
        self.assertEqual(indices[0], (0, 0, 0))
        self.assertEqual(indices[8], (2, 2, 0))
        self.assertEqual(indices[9], (2, 2, 1))
        self.assert_adjacent_grid_steps(indices)

    def test_one_by_one_by_one_grid_is_unique_and_continuous(self):
        waypoints = plan_waypoints(
            0, 1, 0, 1, 0, 1, xs=1, ys=1, zs=1, rpy_ref=(0, 0, 0)
        )
        indices = self._grid_indices(waypoints)

        self.assertEqual(len(waypoints), 8)
        self.assertEqual(len(set(indices)), 8)
        self.assertEqual(indices[0], (0, 0, 0))
        self.assert_adjacent_grid_steps(indices)


class DesignConditionNumberTest(unittest.TestCase):
    """解析设计条件数：只由三轴幅度比决定，与 seed / 采样次数无关。"""

    def assert_ratio(self, amplitudes, expected, rel_tol=0.01):
        info = design_condition_number(amplitudes, REFERENCE_RPY)
        self.assertLessEqual(
            abs(info["condition_number"] - expected) / expected, rel_tol,
            msg=f"{amplitudes} -> {info['condition_number']}, 期望 {expected}")

    def test_default_amplitudes_equal_max_over_min_ratio(self):
        """默认 (5, 20, 15) -> a_max/a_min = 4（±1%）。"""
        self.assertEqual(PERTURB_AMPLITUDE_DEG, (5.0, 20.0, 15.0))
        self.assert_ratio(PERTURB_AMPLITUDE_DEG, 4.0)

    def test_balanced_amplitudes_give_unit_condition_number(self):
        """均衡 (10, 10, 10) -> 1.0（条件数下限）。"""
        self.assert_ratio((10.0, 10.0, 10.0), 1.0)

    def test_smallest_axis_sets_condition_number(self):
        """(20, 15, 0.5) -> ≈40，由最小轴幅度决定。"""
        self.assert_ratio((20.0, 15.0, 0.5), 40.0)

    def test_axis_aligned_reference_poses_agree(self):
        """轴对齐参考姿态下 cond 与参考姿态取值无关（J 至多差一个反射）。"""
        first = design_condition_number(PERTURB_AMPLITUDE_DEG,
                                        (0.0, 0.0, 0.0))
        second = design_condition_number(PERTURB_AMPLITUDE_DEG,
                                         REFERENCE_RPY)
        self.assertAlmostEqual(first["condition_number"],
                               second["condition_number"], places=9)

    def test_missing_axis_is_infinite(self):
        """某轴零幅度 -> 缺轴激励，条件数应为 inf。"""
        info = design_condition_number((20.0, 15.0, 0.0), REFERENCE_RPY)
        self.assertEqual(info["condition_number"], float("inf"))
        self.assertEqual(info["convention"], "cond(X)=sigma_max/sigma_min")


class PoseConditionNumberTest(unittest.TestCase):
    """实测条件数：分块报告 + 合并 6D 仅诊断。"""

    threshold = DEFAULT_COND_MAX       # 与 CLI 默认 --cond-max 保持一致

    @staticmethod
    def _grid_positions(z_min, z_max, xs=2, ys=2, zs=2):
        """取 AABB 蛇形网格的路点坐标（用于位置块测试）。"""
        return [wp.pos for wp in
                plan_waypoints(0, 200, 0, 200, z_min, z_max, xs, ys, zs,
                               (0, 0, 0))]

    def test_identical_orientations_orientation_block_infinite(self):
        """姿态全同：姿态块秩亏 = inf，位置块（真 3D 网格）仍有限。"""
        positions = self._grid_positions(0, 200)
        poses = [_pose(REFERENCE_RPY, pos) for pos in positions]
        info = pose_condition_number(poses, include_position=True)
        self.assertEqual(info["blocks"]["orientation"]["condition_number"],
                         float("inf"))
        self.assertTrue(math.isfinite(
            info["blocks"]["position"]["condition_number"]))
        self.assertEqual(info["reference_source"], "first_pose")

    def test_explicit_reference_removes_order_dependency(self):
        """显式参考姿态后 reference_source=explicit，且与首点选择无关。"""
        poses = _perturbed_poses((20.0, 15.0, 3.0))
        reference = euler_to_rot(REFERENCE_RPY)
        info = pose_condition_number(poses, reference=reference)
        self.assertEqual(info["reference_source"], "explicit")
        self.assertEqual(info["convention"], "cond(X)=sigma_max/sigma_min")

    def test_position_block_condition_is_unit_invariant(self):
        """位置块条件数在 mm / m 输入下一致，且姿态块完全不受影响。"""
        positions_mm = self._grid_positions(0, 200)
        poses_mm = _perturbed_poses((20.0, 15.0, 3.0), count=len(positions_mm),
                                    seed=11, positions=positions_mm)
        poses_m = _perturbed_poses(
            (20.0, 15.0, 3.0), count=len(positions_mm), seed=11,
            positions=[tuple(v * 1e-3 for v in pos) for pos in positions_mm])
        in_mm = pose_condition_number(poses_mm, include_position=True)
        in_m = pose_condition_number(poses_m, include_position=True)

        self.assertAlmostEqual(in_mm["blocks"]["position"]["condition_number"],
                               in_m["blocks"]["position"]["condition_number"],
                               places=9)
        self.assertAlmostEqual(in_mm["blocks"]["orientation"]["rms"],
                               in_m["blocks"]["orientation"]["rms"], places=12)
        # 位置块 rms 是绝对量：mm -> m 应恰好缩小 1e-3
        self.assertAlmostEqual(in_m["blocks"]["position"]["rms"] * 1e3,
                               in_mm["blocks"]["position"]["rms"], places=9)

    def test_plane_grid_position_block_is_rank_deficient(self):
        """平面网格（zs=0）: 位置块秩亏 = inf，姿态块有限。"""
        positions = self._grid_positions(0, 0, xs=2, ys=2, zs=0)
        self.assertTrue(all(pos[2] == 0.0 for pos in positions))
        poses = _perturbed_poses((20.0, 15.0, 3.0), count=len(positions),
                                 seed=5, positions=positions)
        info = pose_condition_number(poses, include_position=True)
        self.assertEqual(info["blocks"]["position"]["condition_number"],
                         float("inf"))
        self.assertTrue(math.isfinite(
            info["blocks"]["orientation"]["condition_number"]))

    def test_combined_metric_is_diagnostic_only(self):
        """include_position 时合并 6D 值标注为等 RMS 合并、仅诊断。"""
        positions = self._grid_positions(0, 200)
        poses = _perturbed_poses((20.0, 15.0, 3.0), count=len(positions),
                                 positions=positions)
        info = pose_condition_number(poses, include_position=True)
        self.assertTrue(info["metric"].endswith("-equal-rms-combined"))
        self.assertTrue(info["combined_is_diagnostic_only"])
        # 门禁只用分块值：合并值不得低于姿态块（等 RMS 拼接的固有性质）
        self.assertGreaterEqual(info["condition_number"],
                                info["blocks"]["orientation"]["condition_number"]
                                - 1e-12)

    def test_single_axis_excitation_is_rank_deficient(self):
        rng = np.random.default_rng(1)
        poses = [_pose((REFERENCE_RPY[0] + rng.uniform(-20, 20),
                        REFERENCE_RPY[1], REFERENCE_RPY[2]))
                 for _ in range(48)]
        info = pose_condition_number(poses)
        self.assertEqual(info["condition_number"], float("inf"))

    def test_two_axis_excitation_is_rejected(self):
        """缺轴激励的实测条件数远超阈值（残留来自 BCH 二阶交换子）。"""
        rng = np.random.default_rng(2)
        poses = [_pose((REFERENCE_RPY[0] + rng.uniform(-20, 20),
                        REFERENCE_RPY[1] + rng.uniform(-15, 15),
                        REFERENCE_RPY[2]))
                 for _ in range(48)]
        self.assertGreater(pose_condition_number(poses)["condition_number"],
                           self.threshold)

    def test_condition_number_is_nearly_scale_invariant(self):
        """条件数度量方向分布而非幅度。

        注意：特征为「相对参考姿态」的旋转向量，其合成含 BCH 二阶交换子
        （``log(exp(a)exp(b)) = a - b + O([a,b])``），因此尺度不变性只是
        近似成立（相对偏差 ~1e-4），不是精确恒等。
        """
        rng = np.random.default_rng(7)
        # 旋转向量限制在 log 映射主值区间内（|θ| < π），否则 log(exp(w)) 会绕回
        axes = [0.1 * w / np.linalg.norm(w) for w in rng.normal(size=(48, 3))]
        wide = pose_condition_number([_pose_from_rotvec(w) for w in axes])
        tiny = pose_condition_number(
            [_pose_from_rotvec(1e-4 * w) for w in axes])
        a = wide["condition_number"]
        b = tiny["condition_number"]
        self.assertLess(abs(a - b) / a, 1e-3)

    def test_rotation_vector_log_map(self):
        """log 映射：Rz(30deg) -> [0, 0, pi/6]，近 180deg 时模长为 pi。"""
        np.testing.assert_allclose(rotation_vector(euler_to_rot((0, 0, 30))),
                                   [0.0, 0.0, math.pi / 6.0], atol=1e-12)
        near_pi = rotation_vector(euler_to_rot((0, 0, 179.999999)))
        self.assertAlmostEqual(float(np.linalg.norm(near_pi)), math.pi, places=6)

    def test_rotation_vector_small_angle_precision(self):
        """小角度区间（含 1e-8 rad）相对误差 < 1e-9。

        旧实现 ``acos((tr-1)/2)`` 在 1e-10~1e-6 rad 区间把 cos 的舍入误差
        放大成 ~1e-4 的角误差；现用 atan2 形式后整个区间都精确。
        """
        axis = np.array([0.3, -0.5, 0.81])
        axis = axis / np.linalg.norm(axis)
        for angle in (1e-8, 1e-7, 1e-6, 1e-5, 1e-4):
            with self.subTest(angle=angle):
                expected = axis * angle
                got = rotation_vector(_pose_from_rotvec(expected)[:3, :3])
                relative_error = (np.linalg.norm(got - expected)
                                  / np.linalg.norm(expected))
                self.assertLess(relative_error, 1e-9)

    def test_empty_poses_report_infinite_condition(self):
        info = pose_condition_number([])
        self.assertEqual(info["condition_number"], float("inf"))
        self.assertEqual(info["n_points"], 0)
        self.assertEqual(info["blocks"]["orientation"]["condition_number"],
                         float("inf"))


class ConditioningGateTest(unittest.TestCase):
    """一次性设计检查门禁：拒绝 / 只报告 / 不运行，均不循环重采样。"""

    @staticmethod
    def _args(argv_extra):
        parser = build_arg_parser()
        return parser.parse_args(
            ["0", "200", "0", "200", "0", "200"] + list(argv_extra))

    def _evaluate(self, args, amplitudes):
        waypoints = plan_waypoints(0, 200, 0, 200, 0, 200, 2, 2, 2,
                                   REFERENCE_RPY)
        poses = _perturbed_poses(amplitudes, count=len(waypoints), seed=0,
                                 positions=[wp.pos for wp in waypoints])
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            report = evaluate_conditioning(args, waypoints, REFERENCE_RPY,
                                           poses)
        return report, buffer.getvalue()

    def test_unequal_amplitudes_are_rejected_with_actionable_advice(self):
        """--perturb-deg 20 15 0.5 -> design cond 40 > 9 -> rejected（非 strict 不退出）。"""
        args = self._args(["--perturb-deg", "20", "15", "0.5"])
        report, output = self._evaluate(args, (20.0, 15.0, 0.5))

        self.assertEqual(report["status"], "rejected")
        self.assertGreater(report["design"]["condition_number"],
                           report["threshold"])
        self.assertGreater(report["gate"]["value"], report["threshold"])
        self.assertEqual(report["gate"]["source"], "design(orientation)")
        # 可执行建议：a_min >= a_max/cond_max = 20/9 = 2.22 deg（当前 0.5）
        self.assertIn("需要 a_min >= a_max/cond_max = 2.22 deg（当前 0.5）",
                      output)

    def test_gate_is_seed_independent(self):
        """门禁取设计值：同一方案不同 seed 必须给出相同的门禁值。

        若门禁改用实测（样本估计），同方案不同 seed 会在总体值附近波动，
        出现“有的 seed 过、有的不过”的不可复现行为。
        """
        args = self._args([])
        gates = []
        for seed in (0, 1, 2, 7, 11):
            waypoints = plan_waypoints(0, 200, 0, 200, 0, 200, 2, 2, 2,
                                       REFERENCE_RPY)
            poses = _perturbed_poses(PERTURB_AMPLITUDE_DEG,
                                     count=len(waypoints), seed=seed,
                                     positions=[wp.pos for wp in waypoints])
            with redirect_stdout(io.StringIO()):
                report = evaluate_conditioning(args, waypoints,
                                               REFERENCE_RPY, poses)
            self.assertEqual(report["status"], "accepted")
            gates.append(report["gate"]["value"])
        self.assertEqual(len(set(round(v, 12) for v in gates)), 1,
                         f"门禁值随 seed 变化: {gates}")

    def test_balanced_amplitudes_are_accepted(self):
        args = self._args(["--perturb-deg", "20", "15", "10"])
        report, _ = self._evaluate(args, (20.0, 15.0, 10.0))
        self.assertEqual(report["status"], "accepted")
        self.assertLessEqual(report["gate"]["value"], report["threshold"])

    def test_no_cond_check_reports_only(self):
        args = self._args(["--no-cond-check"])
        report, _ = self._evaluate(args, PERTURB_AMPLITUDE_DEG)
        self.assertEqual(report["status"], "reported_only")
        self.assertIsNotNone(report["realized"])

    def test_no_perturb_does_not_run(self):
        args = self._args(["--no-perturb"])
        report, output = self._evaluate(args, PERTURB_AMPLITUDE_DEG)
        self.assertEqual(report["status"], "not_run")
        self.assertIsNone(report["realized"])
        self.assertIsNone(report["gate"]["value"])
        # 设计值仍给出，但不打印实测数字
        self.assertAlmostEqual(report["design"]["condition_number"],
                               20.0 / 5.0, places=6)
        self.assertNotIn("实测", output)

    def test_single_point_grid_serializes_infinite_condition_as_null(self):
        """1 点网格：非 strict 运行可加载，conditioning 内无 Infinity。"""
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            out = Path(tmp) / "one_point.json"
            png = Path(tmp) / "one_point.png"
            with redirect_stdout(io.StringIO()):
                code = main(["800", "1200", "-200", "200", "800", "1000",
                             "--xs", "0", "--ys", "0", "--zs", "0",
                             "--out", str(out), "--plot", str(png)])
            self.assertEqual(code, 0)

            raw = out.read_text(encoding="utf-8")
            self.assertNotIn("Infinity", raw)
            self.assertNotIn("NaN", raw)
            payload = json.loads(raw)
            conditioning = payload["conditioning"]
            self.assertEqual(conditioning["status"], "rejected")
            self.assertIsNone(conditioning["realized"]["condition_number"])
            self.assertIsNone(
                conditioning["realized"]["blocks"]["orientation"]
                ["condition_number"])
            self.assertIsNone(conditioning["post_ik"]["condition_number"])
            self.assertIsNone(conditioning["gate"]["value"])

    def test_skip_ik_outputs_not_run_conditioning(self):
        """--skip-ik: 同构 conditioning（status=not_run）+ meta 三键。"""
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            out = Path(tmp) / "skip.json"
            png = Path(tmp) / "skip.png"
            with redirect_stdout(io.StringIO()):
                code = main(["0", "200", "0", "200", "0", "200",
                             "--xs", "2", "--ys", "2", "--zs", "2",
                             "--skip-ik", "--out", str(out),
                             "--plot", str(png)])
            self.assertEqual(code, 0)
            payload = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(payload["conditioning"]["status"], "not_run")
            self.assertIsNone(payload["conditioning"]["realized"])
            self.assertAlmostEqual(
                payload["conditioning"]["design"]["condition_number"],
                20.0 / 5.0, places=6)
            self.assertFalse(payload["meta"]["perturbed"])
            self.assertTrue(payload["meta"]["skip_ik"])
            self.assertIn("no_perturb", payload["meta"])


if __name__ == "__main__":
    unittest.main()


class EulerConventionOptionTest(unittest.TestCase):
    """--euler 选项：与 --intrinsic 等价、与默认（xyz-extrinsic）可区分。"""

    @staticmethod
    def _run(extra, out, plot):
        argv = ["800", "1200", "-200", "200", "800", "1000",
                "--xs", "1", "--ys", "1", "--zs", "0",
                "--seed", "3", "--out", str(out), "--plot", str(plot)] + extra
        with redirect_stdout(io.StringIO()):
            code = main(argv)
        assert code == 0
        return json.loads(Path(out).read_text(encoding="utf-8"))

    def test_euler_option_and_intrinsic_alias(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            root = Path(tmp)
            default = self._run([], root / "a.json", root / "a.png")
            extrinsic = self._run(["--euler", "xyz-extrinsic"],
                                  root / "b.json", root / "b.png")
            intrinsic_flag = self._run(["--intrinsic"], root / "c.json",
                                       root / "c.png")
            intrinsic_opt = self._run(["--euler", "xyz-intrinsic"],
                                      root / "d.json", root / "d.png")
            self.assertEqual(default["meta"]["euler_convention"],
                             "xyz-extrinsic")
            self.assertEqual(default, extrinsic)
            self.assertEqual(intrinsic_flag, intrinsic_opt)
            self.assertEqual(intrinsic_opt["meta"]["euler_convention"],
                             "xyz-intrinsic")
            # 两约定对同一扰动序列给出不同的 IK 解（rpy 数值本身与约定无关，
            # 约定只在 rpy -> 旋转矩阵时生效，因此差异体现在 perturbed_ik 的 q 上）
            self.assertNotEqual(default["perturbed_ik"],
                                intrinsic_opt["perturbed_ik"])

    def test_invalid_euler_choice_rejected(self):
        parser = build_arg_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args(["0", "200", "0", "200", "0", "200",
                               "--euler", "zyx-intrinsic"])


class ConfigTagTest(unittest.TestCase):
    """构型标签必须按 IK 的真实分支变量取值（曾是 q[5]=J6 的 off-by-one）。"""

    def _robot(self):
        return Nexbot6R(str(Path(__file__).resolve().parents[3]
                            / "data" / "configs" / "ch2000_origin.urdf"))

    def test_wrist_tag_uses_j5_not_j6(self):
        """J5 定分支、J6 随便变：标签必须只随 J5 翻。"""
        rbt = self._robot()
        pose = rbt.fk([0.0, 20.0, -20.0, 0.0, 60.0, 0.0])
        positive = config_tag(rbt, pose, [0.0, 20.0, -20.0, 0.0, 60.0, 30.0])
        negative = config_tag(rbt, pose, [0.0, 20.0, -20.0, 0.0, -60.0, 30.0])
        # J6 同为正，但 J5 反号 -> wrist 必须不同
        self.assertEqual(positive[2], 1)
        self.assertEqual(negative[2], -1)

    def test_j6_crossing_zero_does_not_change_wrist_tag(self):
        rbt = self._robot()
        pose = rbt.fk([0.0, 20.0, -20.0, 0.0, 60.0, 0.0])
        a = config_tag(rbt, pose, [0.0, 20.0, -20.0, 0.0, 60.0, -5.0])
        b = config_tag(rbt, pose, [0.0, 20.0, -20.0, 0.0, 60.0, 5.0])
        self.assertEqual(a[2], b[2])

    def test_elbow_tag_uses_j3(self):
        rbt = self._robot()
        pose = rbt.fk([0.0, 20.0, -20.0, 0.0, 60.0, 0.0])
        self.assertEqual(config_tag(rbt, pose, [0.0, 20.0, 20.0, 0.0, 60.0, 0.0])[1], 1)
        self.assertEqual(config_tag(rbt, pose, [0.0, 20.0, -20.0, 0.0, 60.0, 0.0])[1], -1)
