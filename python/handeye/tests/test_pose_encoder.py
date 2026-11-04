# -*- coding: utf-8 -*-
"""handeye.pose_encoder 单元测试：欧拉约定（XYZ 内旋）与位姿字段往返。

关键回归点：C++ ``kXyzIntrinsic`` 对应 ``euler_to_rot(order="XYZ",
extrinsic=False)``，即 ``R = Rx·Ry·Rz``；若误用 ``extrinsic=True``
（``Rz·Ry·Rx``），第 3 个测试会立刻失败。
"""

import math
import unittest
from pathlib import Path

import numpy as np

from handeye.pose_encoder import (
    EULER_ORDER,
    EXTRINSIC,
    euler_intrinsic_to_rotation,
    matrix_to_pose_fields,
    pose_fields_to_matrix,
    rotation_to_euler_intrinsic,
    self_check,
)
from handeye.se3 import make_pose
from robot_model.nexbot_ik import Nexbot6R, euler_to_rot, rot_to_euler

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "ch2000_origin.urdf"


def _rotation_angle_deg(first, second) -> float:
    """两旋转的相对转角（deg），``atan2`` 形式（0 附近仍准确）。

    ``acos((tr-1)/2)`` 在 <1e-5 度时会被浮点下限吃掉（恒得 ~1.7e-6 度），
    无法验证 1e-9 度量级的一致性，因此这里换用 ``atan2``。
    """
    relative = np.asarray(first, dtype=float).T @ np.asarray(second, dtype=float)
    sin_part = 0.5 * float(np.linalg.norm([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ]))
    cos_part = 0.5 * (float(np.trace(relative)) - 1.0)
    return math.degrees(math.atan2(sin_part, cos_part))


def _random_rotations(count: int, seed: int = 0) -> list:
    """均匀采样的随机旋转矩阵（轴角表示 + Rodrigues）。"""
    rng = np.random.default_rng(seed)
    rotations = []
    for _ in range(count):
        axis = rng.normal(size=3)
        axis = axis / np.linalg.norm(axis)
        angle = rng.uniform(-math.pi, math.pi)
        skew = np.array([[0.0, -axis[2], axis[1]],
                         [axis[2], 0.0, -axis[0]],
                         [-axis[1], axis[0], 0.0]])
        rotations.append(np.eye(3) + math.sin(angle) * skew
                         + (1.0 - math.cos(angle)) * (skew @ skew))
    return rotations


class EulerConventionTest(unittest.TestCase):
    """约定本身：XYZ 内旋、与 C++ kXyzIntrinsic 对应。"""

    def test_constants(self):
        self.assertEqual(EULER_ORDER, "XYZ")
        self.assertFalse(EXTRINSIC)

    def test_roundtrip_two_hundred_random_poses(self):
        rotations = _random_rotations(200, seed=11)
        max_error = 0.0
        for rotation in rotations:
            euler = rotation_to_euler_intrinsic(rotation)
            back = euler_intrinsic_to_rotation(euler)
            max_error = max(max_error, float(np.max(np.abs(back - rotation))))
        self.assertLess(max_error, 1e-10)

    def test_matches_nexbot_intrinsic_euler(self):
        rng = np.random.default_rng(23)
        max_error = 0.0
        for rotation in _random_rotations(50, seed=5):
            euler = rotation_to_euler_intrinsic(rotation)
            if abs(float(rotation[0, 2])) > 1.0 - 1e-6:
                continue  # 万向锁附近单项角本身就病态，跳过（由 roundtrip 覆盖）
            expected = euler_to_rot(euler, order="XYZ", extrinsic=False)
            max_error = max(max_error, float(np.max(np.abs(expected - rotation))))
        self.assertLess(max_error, 1e-12)
        # 反向：由角度造矩阵应与库函数逐元素一致
        angles = rng.uniform(-180.0, 180.0, size=(50, 3))
        for rpy in angles:
            np.testing.assert_allclose(
                euler_intrinsic_to_rotation(rpy),
                euler_to_rot(rpy, order="XYZ", extrinsic=False), atol=1e-12)

    def test_differs_from_extrinsic_convention(self):
        rpy = [10.0, 20.0, 30.0]
        intrinsic = euler_intrinsic_to_rotation(rpy)
        extrinsic = euler_to_rot(rpy, order="XYZ", extrinsic=True)
        difference = float(np.max(np.abs(intrinsic - extrinsic)))
        self.assertGreater(difference, 1e-6)
        # 同一个矩阵用两种约定去分解，得到的三角度必须不同（符号也相反）
        as_intrinsic = rotation_to_euler_intrinsic(intrinsic)
        as_extrinsic = np.array(rot_to_euler(intrinsic, order="XYZ", extrinsic=True))
        self.assertGreater(float(np.max(np.abs(as_intrinsic - as_extrinsic))), 1e-6)

    def test_self_check_returns_small_error(self):
        self.assertLess(self_check(samples=200, seed=0), 1e-10)

    def test_gimbal_lock_still_reconstructs(self):
        for pitch in (90.0, -90.0):
            rotation = euler_to_rot([37.0, pitch, -21.0], order="XYZ",
                                    extrinsic=False)
            euler = rotation_to_euler_intrinsic(rotation)
            back = euler_intrinsic_to_rotation(euler)
            np.testing.assert_allclose(back, rotation, atol=1e-12)
            self.assertAlmostEqual(euler[1], pitch, places=12)


class PoseFieldsTest(unittest.TestCase):
    """4x4 <-> (position mm, orientation deg) 往返。"""

    def test_roundtrip_random_poses(self):
        rng = np.random.default_rng(31)
        for rotation in _random_rotations(100, seed=17):
            xyz = rng.uniform(-1500.0, 1500.0, size=3)
            matrix = make_pose(xyz, rotation)
            position, orientation = matrix_to_pose_fields(matrix)
            self.assertEqual(len(position), 3)
            self.assertEqual(len(orientation), 3)
            back = pose_fields_to_matrix(position, orientation)
            np.testing.assert_allclose(back[:3, 3], xyz, atol=1e-9, rtol=0.0)
            np.testing.assert_allclose(back[:3, :3], rotation, atol=1e-12)

    def test_nexbot_fk_roundtrip(self):
        robot = Nexbot6R(URDF)
        rng = np.random.default_rng(41)
        max_position_error = 0.0
        max_rotation_error = 0.0
        for _ in range(200):
            joints = [float(rng.uniform(low, high)) for low, high in robot.limits]
            matrix = robot.fk(joints)
            position, orientation = matrix_to_pose_fields(matrix)
            back = pose_fields_to_matrix(position, orientation)
            max_position_error = max(
                max_position_error, float(np.max(np.abs(back[:3, 3] - matrix[:3, 3]))))
            max_rotation_error = max(
                max_rotation_error, _rotation_angle_deg(back[:3, :3], matrix[:3, :3]))
        self.assertLess(max_position_error, 1e-9)
        self.assertLess(max_rotation_error, 1e-9)

    def test_known_pose_values(self):
        # 绕 X 90 度（内旋 XYZ 下 rx=90 表示 R = Rx(90)）
        position, orientation = matrix_to_pose_fields(
            make_pose([1.0, 2.0, 3.0],
                      euler_to_rot([90.0, 0.0, 0.0], order="XYZ", extrinsic=False)))
        np.testing.assert_allclose(position, [1.0, 2.0, 3.0], atol=0.0)
        np.testing.assert_allclose(orientation, [90.0, 0.0, 0.0], atol=1e-12)


class InvalidInputTest(unittest.TestCase):
    """非法输入统一抛 ValueError（库函数不吞错）。"""

    def test_pose_fields_to_matrix_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            pose_fields_to_matrix([1.0, 2.0], [0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            pose_fields_to_matrix([1.0, 2.0, 3.0], [0.0, 0.0])
        with self.assertRaises(ValueError):
            pose_fields_to_matrix([1.0, float("nan"), 3.0], [0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            pose_fields_to_matrix([1.0, 2.0, 3.0], [0.0, float("inf"), 0.0])
        with self.assertRaises(ValueError):
            pose_fields_to_matrix("abc", [0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            pose_fields_to_matrix(None, [0.0, 0.0, 0.0])

    def test_matrix_to_pose_fields_rejects_bad_input(self):
        good = make_pose([0.0, 0.0, 0.0], np.eye(3))
        with self.assertRaises(ValueError):
            matrix_to_pose_fields(np.eye(3))
        with self.assertRaises(ValueError):
            matrix_to_pose_fields(np.eye(5))
        broken = good.copy()
        broken[0, 3] = float("nan")
        with self.assertRaises(ValueError):
            matrix_to_pose_fields(broken)
        scaled = good.copy()
        scaled[:3, :3] = np.eye(3) * 1.5
        with self.assertRaises(ValueError):
            matrix_to_pose_fields(scaled)

    def test_rotation_helpers_reject_bad_input(self):
        with self.assertRaises(ValueError):
            rotation_to_euler_intrinsic(np.eye(4))
        with self.assertRaises(ValueError):
            euler_intrinsic_to_rotation([0.0, 0.0])
        with self.assertRaises(ValueError):
            euler_intrinsic_to_rotation([0.0, float("nan"), 0.0])
        with self.assertRaises(ValueError):
            rotation_to_euler_intrinsic(np.diag([1.0, 1.0, -1.0]))  # det = -1


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
