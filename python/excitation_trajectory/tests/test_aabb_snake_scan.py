"""连续 AABB 蛇形网格路点与姿态条件数的单元测试。"""

import math
import unittest

import numpy as np

from robot_model.nexbot_ik import euler_to_rot
from excitation_trajectory.aabb_snake_scan import (
    DEFAULT_COND_MAX,
    plan_waypoints,
    pose_condition_number,
    rotation_vector,
)


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


class PoseConditionNumberTest(unittest.TestCase):
    """验证 SVD 条件数能区分「充分激励」与「病态姿态集」。"""

    reference_abc = (180.0, 0.0, -90.0)
    threshold = DEFAULT_COND_MAX       # 与 CLI 默认 --cond-max 保持一致

    def _perturbed(self, amp=(20.0, 15.0, 3.0), count=48, seed=0):
        """构造绕三轴分别按 amp 幅度均匀扰动的位姿集。"""
        rng = np.random.default_rng(seed)
        ax, ay, az = self.reference_abc
        return [_pose((ax + rng.uniform(-amp[0], amp[0]),
                       ay + rng.uniform(-amp[1], amp[1]),
                       az + rng.uniform(-amp[2], amp[2])))
                for _ in range(count)]

    def test_identical_poses_are_rank_deficient(self):
        info = pose_condition_number([_pose(self.reference_abc) for _ in range(10)])
        self.assertEqual(info["condition_number"], float("inf"))
        self.assertEqual(info["n_points"], 10)

    def test_three_axis_excitation_passes_default_threshold(self):
        info = pose_condition_number(self._perturbed())
        self.assertLessEqual(info["condition_number"], self.threshold)
        self.assertEqual(len(info["singular_values"]), 3)

    def test_balanced_amplitudes_condition_better_than_unequal(self):
        """条件数由三轴**幅度比**决定：均衡幅度优于 20:15:3。"""
        balanced = pose_condition_number(
            self._perturbed(amp=(10.0, 10.0, 10.0), seed=3))
        unequal = pose_condition_number(
            self._perturbed(amp=(20.0, 15.0, 3.0), seed=3))
        self.assertLess(balanced["condition_number"], 2.0)
        self.assertGreater(unequal["condition_number"],
                           balanced["condition_number"])

    def test_single_axis_excitation_is_rejected(self):
        ax, ay, az = self.reference_abc
        rng = np.random.default_rng(1)
        poses = [_pose((ax + rng.uniform(-20, 20), ay, az)) for _ in range(48)]
        info = pose_condition_number(poses)
        self.assertGreater(info["condition_number"], self.threshold)

    def test_two_axis_excitation_is_rejected(self):
        ax, ay, az = self.reference_abc
        rng = np.random.default_rng(2)
        poses = [_pose((ax + rng.uniform(-20, 20),
                        ay + rng.uniform(-15, 15), az)) for _ in range(48)]
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

    def test_position_block_is_unit_invariant(self):
        """位置以 mm 或 m 输入，条件数必须一致（等 RMS 归一化）。"""
        def build(scale):
            base = self._perturbed(count=48, seed=11)
            for i, transform in enumerate(base):
                transform[:3, 3] = np.array([800.0 + (i % 4) * 130.0,
                                             -200.0 + (i // 4 % 3) * 200.0,
                                             800.0 + (i // 12) * 100.0]) * scale
            return base

        in_mm = pose_condition_number(build(1.0), include_position=True)
        in_m = pose_condition_number(build(1e-3), include_position=True)
        self.assertAlmostEqual(in_mm["condition_number"],
                              in_m["condition_number"], places=9)

    def test_rotation_vector_log_map(self):
        """log 映射：Rz(30deg) -> [0, 0, pi/6]，近 180deg 时模长为 pi。"""
        np.testing.assert_allclose(rotation_vector(euler_to_rot((0, 0, 30))),
                                   [0.0, 0.0, math.pi / 6.0], atol=1e-12)
        near_pi = rotation_vector(euler_to_rot((0, 0, 179.999999)))
        self.assertAlmostEqual(float(np.linalg.norm(near_pi)), math.pi, places=6)

    def test_empty_poses_report_infinite_condition(self):
        info = pose_condition_number([])
        self.assertEqual(info["condition_number"], float("inf"))
        self.assertEqual(info["n_points"], 0)


if __name__ == "__main__":
    unittest.main()
