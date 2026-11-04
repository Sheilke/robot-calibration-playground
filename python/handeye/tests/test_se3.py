# -*- coding: utf-8 -*-
"""handeye.se3 单元测试。"""

import math
import unittest

import numpy as np

from handeye.se3 import (
    from_rpy,
    inverse,
    load_matrix16,
    make_pose,
    rotation_angle_between,
    to_matrix16,
    to_rpy,
    transform_point,
    transform_points,
    validate_rigid,
)


class Se3BasicsTest(unittest.TestCase):
    def test_rpy_roundtrip(self):
        xyz = (100.0, -50.0, 900.0)
        rpy = (180.0, 0.0, -90.0)
        pose = from_rpy(xyz, rpy)
        xyz_out, rpy_out = to_rpy(pose)
        np.testing.assert_allclose(xyz_out, xyz, atol=1e-9)
        np.testing.assert_allclose(rpy_out, rpy, atol=1e-9)

    def test_inverse(self):
        pose = from_rpy((10.0, 20.0, 30.0), (35.0, -20.0, 120.0))
        np.testing.assert_allclose(pose @ inverse(pose), np.eye(4), atol=1e-9)

    def test_transform_point_and_points_agree(self):
        pose = from_rpy((5.0, -6.0, 7.0), (10.0, 20.0, 30.0))
        points = np.array([[100.0, 0.0, 0.0], [0.0, 50.0, 20.0]])
        single = np.array([transform_point(pose, p) for p in points])
        np.testing.assert_allclose(transform_points(pose, points), single, atol=1e-9)

    def test_rotation_angle_between(self):
        a = from_rpy((0, 0, 0), (0.0, 0.0, 0.0))
        b = from_rpy((0, 0, 0), (0.0, 30.0, 0.0))
        self.assertAlmostEqual(rotation_angle_between(a[:3, :3], b[:3, :3]), 30.0,
                               places=9)


class Matrix16Test(unittest.TestCase):
    def _values(self, matrix):
        return [float(v) for v in np.asarray(matrix).reshape(-1)]

    def test_load_identity(self):
        matrix = load_matrix16(self._values(np.eye(4)), units="mm")
        np.testing.assert_allclose(matrix, np.eye(4))

    def test_load_converts_meters_to_mm(self):
        pose = from_rpy((0.10765, 0.004, 0.54442), (10.0, -20.0, 30.0))
        matrix = load_matrix16(self._values(pose), units="m", target_units="mm")
        np.testing.assert_allclose(matrix[:3, 3],
                                   [107.65, 4.0, 544.42], atol=1e-9)
        np.testing.assert_allclose(matrix[:3, :3], pose[:3, :3], atol=1e-12)

    def test_to_matrix16_roundtrip(self):
        pose = from_rpy((12.0, 34.0, -56.0), (5.0, 6.0, 7.0))
        values = to_matrix16(pose, units="m")
        back = load_matrix16(values, units="m", target_units="mm")
        np.testing.assert_allclose(back, pose, atol=1e-9)

    def test_rejects_wrong_count(self):
        with self.assertRaises(ValueError):
            load_matrix16([1.0] * 15)

    def test_rejects_non_finite(self):
        values = self._values(np.eye(4))
        values[3] = float("nan")
        with self.assertRaises(ValueError):
            load_matrix16(values)

    def test_rejects_bad_last_row(self):
        matrix = np.eye(4)
        matrix[3, 3] = 1.1
        with self.assertRaises(ValueError):
            load_matrix16(self._values(matrix))

    def test_rejects_non_orthonormal_rotation(self):
        matrix = np.eye(4)
        matrix[:3, :3] *= 1.01                      # 缩放破坏正交性
        with self.assertRaises(ValueError):
            load_matrix16(self._values(matrix))

    def test_rejects_reflection(self):
        matrix = np.eye(4)
        matrix[0, 0] = -1.0                         # det = -1（反射）
        with self.assertRaises(ValueError):
            load_matrix16(self._values(matrix))

    def test_rejects_unknown_units(self):
        with self.assertRaises(ValueError):
            load_matrix16(self._values(np.eye(4)), units="inch")

    def test_validate_rigid_accepts_near_orthogonal(self):
        # 容差 1e-4 内的正交化误差应通过（公司约定同容差）
        rng = np.random.default_rng(0)
        rot = from_rpy((0, 0, 0), tuple(rng.uniform(-180, 180, 3)))[:3, :3]
        rot = rot + 1e-6                            # 仍在容差内
        try:
            validate_rigid(make_pose((0, 0, 0), rot + np.eye(3) * 1e-7))
        except ValueError as error:
            self.fail(f"容差内的矩阵不应被拒绝: {error}")


if __name__ == "__main__":
    unittest.main()
