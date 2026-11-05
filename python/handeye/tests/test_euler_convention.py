# -*- coding: utf-8 -*-
"""欧拉约定选项（xyz-intrinsic / xyz-extrinsic）的行为测试。"""

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye.pose_encoder import (
    DEFAULT_EULER_CONVENTION,
    EULER_CONVENTIONS,
    convention_extrinsic,
    euler_intrinsic_to_rotation,
    euler_to_rotation,
    matrix_to_pose_fields,
    normalize_convention,
    pose_fields_to_matrix,
    rotation_to_euler,
    rotation_to_euler_intrinsic,
)
from handeye.session import SessionError, load_session, session_from_dict
from robot_model.nexbot_ik import euler_to_rot

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "ch2000_origin.urdf"
SESSION_EXAMPLE = (Path(__file__).resolve().parents[3]
                   / "data" / "configs" / "session_example.json")


class PoseEncoderConventionTest(unittest.TestCase):
    def test_normalize_convention(self):
        self.assertEqual(normalize_convention("xyz-intrinsic"), "xyz-intrinsic")
        self.assertEqual(normalize_convention(" XYZ-Intrinsic "), "xyz-intrinsic")
        self.assertEqual(normalize_convention("xyz-extrinsic"), "xyz-extrinsic")
        for bad in ("zyx-intrinsic", "intrinsic", 5, None):
            with self.assertRaises(ValueError):
                normalize_convention(bad)

    def test_convention_extrinsic_flag(self):
        self.assertFalse(convention_extrinsic("xyz-intrinsic"))
        self.assertTrue(convention_extrinsic("xyz-extrinsic"))

    def test_two_conventions_disagree(self):
        """两约定对同一组角度给出不同旋转（防止约定参数失效）。"""
        rpy = (180.0, 0.0, -90.0)
        inner = euler_to_rotation(rpy, "xyz-intrinsic")
        outer = euler_to_rotation(rpy, "xyz-extrinsic")
        self.assertGreater(float(np.max(np.abs(inner - outer))), 1e-6)

    def test_conventions_match_underlying_euler_to_rot(self):
        rng = np.random.default_rng(11)
        for _ in range(50):
            rpy = rng.uniform(-180, 180, 3)
            np.testing.assert_allclose(
                euler_to_rotation(rpy, "xyz-intrinsic"),
                euler_to_rot(rpy, extrinsic=False), atol=1e-12)
            np.testing.assert_allclose(
                euler_to_rotation(rpy, "xyz-extrinsic"),
                euler_to_rot(rpy, extrinsic=True), atol=1e-12)

    def test_roundtrip_both_conventions(self):
        rng = np.random.default_rng(5)
        for convention in EULER_CONVENTIONS:
            error = 0.0
            for _ in range(200):
                rot = euler_to_rot(tuple(rng.uniform(-179, 179, 3)),
                                   extrinsic=convention_extrinsic(convention))
                back = euler_to_rotation(
                    rotation_to_euler(rot, convention), convention)
                error = max(error, float(np.max(np.abs(back - rot))))
            self.assertLess(error, 1e-9, msg=convention)

    def test_matrix_to_pose_fields_respects_convention(self):
        """同一矩阵在两约定下解出的欧拉角能各自精确重构（且数值不同）。"""
        rng = np.random.default_rng(7)
        pose = euler_to_rot(tuple(rng.uniform(-170, 170, 3)), extrinsic=False)
        matrix = np.eye(4)
        matrix[:3, :3] = pose
        pos_i, ori_i = matrix_to_pose_fields(matrix, "xyz-intrinsic")
        pos_e, ori_e = matrix_to_pose_fields(matrix, "xyz-extrinsic")
        self.assertNotEqual(ori_i, ori_e)
        rebuilt_i = pose_fields_to_matrix(pos_i, ori_i, "xyz-intrinsic")
        rebuilt_e = pose_fields_to_matrix(pos_e, ori_e, "xyz-extrinsic")
        np.testing.assert_allclose(rebuilt_i, matrix, atol=1e-9)
        np.testing.assert_allclose(rebuilt_e, matrix, atol=1e-9)

    def test_default_unchanged_intrinsic(self):
        """默认仍是 XYZ 内旋（控制器链路），旧调用方零改动兼容。"""
        rng = np.random.default_rng(3)
        rot = euler_to_rot(tuple(rng.uniform(-170, 170, 3)), extrinsic=False)
        np.testing.assert_allclose(
            euler_to_rotation(rotation_to_euler(rot)), rot, atol=1e-12)
        self.assertEqual(DEFAULT_EULER_CONVENTION, "xyz-intrinsic")
        np.testing.assert_allclose(euler_to_rotation((10, 20, 30)),
                                   euler_intrinsic_to_rotation((10, 20, 30)))
        np.testing.assert_allclose(rotation_to_euler(np.eye(3)[:3, :3] @ euler_to_rot((0, 0, 0))),
                                   rotation_to_euler_intrinsic(np.eye(3)))


class SessionEulerConventionTest(unittest.TestCase):
    @staticmethod
    def _payload():
        return json.loads(SESSION_EXAMPLE.read_text(encoding="utf-8"))

    def test_default_is_intrinsic(self):
        session = load_session(SESSION_EXAMPLE)
        self.assertEqual(session.euler_convention, "xyz-intrinsic")

    def test_recorded_pose_decodes_differently_per_convention(self):
        """同一 orientation 字段在两约定下解出不同 4x4。"""
        q = [0.0, 20.0, -20.0, 0.0, 40.0, 0.0]
        orientation = [150.0, 30.0, -60.0]      # 两约定差异明显的一组角

        def build(euler_convention):
            payload = {
                "meta": {"robot_urdf": str(URDF),
                         "euler_convention": euler_convention},
                "reference": {"p_ball_laser_base": [800, 200, 650]},
                "handeye_matrix": None,
                "samples": [{"pose_id": 1, "q_deg": q,
                             "recorded_pose": {"position": [1, 2, 3],
                                               "orientation": orientation},
                             "frames": [{"target_points": {"7": [1, 2, 3]}}]}],
            }
            return session_from_dict(payload)

        inner = build("xyz-intrinsic").samples[0].recorded_pose
        outer = build("xyz-extrinsic").samples[0].recorded_pose
        self.assertGreater(float(np.max(np.abs(inner[:3, :3] - outer[:3, :3]))),
                           1e-3)
        # 各自忠实还原
        np.testing.assert_allclose(
            inner[:3, :3],
            euler_to_rot(orientation, extrinsic=False), atol=1e-9)
        np.testing.assert_allclose(
            outer[:3, :3],
            euler_to_rot(orientation, extrinsic=True), atol=1e-9)

    def test_invalid_convention_rejected(self):
        payload = self._payload()
        payload["meta"]["euler_convention"] = "zyx-intrinsic"
        with self.assertRaises(SessionError):
            session_from_dict(payload)

    def test_meta_recorded_in_session(self):
        payload = self._payload()
        payload["meta"]["euler_convention"] = "xyz-extrinsic"
        session = session_from_dict(payload)
        self.assertEqual(session.euler_convention, "xyz-extrinsic")


if __name__ == "__main__":
    unittest.main()
