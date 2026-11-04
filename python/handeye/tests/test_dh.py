# -*- coding: utf-8 -*-
"""handeye.dh 单元测试：基准等价、覆盖生效、FK 差异、单位换算与校验。"""

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye.dh import DH_KEYS, DhOverride, RobotKinematics, fk_difference
from robot_model.nexbot_ik import Nexbot6R

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "ch2000_origin.urdf"
ZERO_JOINTS = [0.0] * 6


def _random_joint_angles(count: int, seed: int = 0) -> list:
    """在 URDF 软限位内取 ``count`` 组随机关节角（度）。"""
    robot = Nexbot6R(URDF)
    rng = np.random.default_rng(seed)
    return [[float(rng.uniform(low, high)) for low, high in robot.limits]
            for _ in range(count)]


def _write_json(directory: str, payload: dict, name: str = "dh_override.json") -> Path:
    """把字典写成 JSON 文件并返回路径。"""
    path = Path(directory) / name
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


class BaseEquivalenceTest(unittest.TestCase):
    """基准覆盖（旧 DH = URDF 反解）必须与 Nexbot6R.fk 完全一致。"""

    @classmethod
    def setUpClass(cls):
        cls.robot = Nexbot6R(URDF)
        cls.base_override = DhOverride.from_base(URDF)
        cls.base_model = RobotKinematics(URDF, cls.base_override, label="old")
        cls.plain_model = RobotKinematics(URDF, label="plain")

    def test_from_base_fk_matches_nexbot_fk(self):
        for q in _random_joint_angles(10, seed=3):
            np.testing.assert_allclose(
                self.base_model.fk(q), self.robot.fk(q), atol=1e-9, rtol=0.0)

    def test_from_base_records_reference_parameters(self):
        np.testing.assert_allclose(self.base_model.dh["a"], self.robot.dh_a, atol=0.0)
        np.testing.assert_allclose(self.base_model.dh["d"], self.robot.dh_d, atol=0.0)
        np.testing.assert_allclose(
            self.base_model.dh["alpha"],
            np.degrees(self.robot.dh_alpha), atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(
            self.base_model.dh["theta_offset"],
            np.degrees(self.robot.dh_off), atol=1e-12, rtol=0.0)
        self.assertEqual(self.base_model.limits, self.robot.limits)

    def test_pose_rpy_agrees_with_fk(self):
        q = [10.0, 25.0, -20.0, 5.0, 40.0, -15.0]
        xyz, rpy = self.plain_model.pose_rpy(q)
        np.testing.assert_allclose(xyz, self.plain_model.fk(q)[:3, 3], atol=1e-12)
        self.assertEqual(np.asarray(rpy).shape, (3,))
        # RPY 反投影回矩阵应与 FK 的旋转块一致
        rebuilt = self.robot.pose_to_T(xyz, rpy)
        np.testing.assert_allclose(rebuilt, self.plain_model.fk(q), atol=1e-9)


class OverrideEffectTest(unittest.TestCase):
    """覆盖生效：替换（不是相加），且物理含义正确。"""

    @classmethod
    def setUpClass(cls):
        cls.plain_model = RobotKinematics(URDF)

    def test_a2_plus_ten_millimetres_moves_position(self):
        override = DhOverride.from_base(URDF)
        override.a[1] += 10.0
        model = RobotKinematics(URDF, override)
        deltas = [
            float(np.linalg.norm(model.fk(q)[:3, 3] - self.plain_model.fk(q)[:3, 3]))
            for q in _random_joint_angles(10, seed=7)
        ]
        self.assertGreater(max(deltas), 5.0)

    def test_d1_plus_ten_millimetres_shifts_along_base_z(self):
        override = DhOverride.from_base(URDF)
        override.d[0] += 10.0
        model = RobotKinematics(URDF, override)
        for q in [ZERO_JOINTS, [30.0, 20.0, -15.0, 10.0, 35.0, -60.0]]:
            delta = model.fk(q)[:3, 3] - self.plain_model.fk(q)[:3, 3]
            np.testing.assert_allclose(delta, [0.0, 0.0, 10.0], atol=1e-9)

    def test_override_replaces_instead_of_adding(self):
        override = DhOverride.from_base(URDF)
        override.d[0] = 600.0
        model = RobotKinematics(URDF, override)
        self.assertEqual(model.dh["d"][0], 600.0)
        self.assertEqual(model.dh["a"], self.plain_model.dh["a"])

    def test_label_follows_override_name(self):
        override = DhOverride.from_base(URDF, name="ch2000_new")
        self.assertEqual(RobotKinematics(URDF, override).label, "ch2000_new")
        self.assertEqual(RobotKinematics(URDF).label, "base")
        self.assertEqual(RobotKinematics(URDF, override, label="explicit").label, "explicit")


class FkDifferenceTest(unittest.TestCase):
    """fk_difference 的零差异与扰动同量级。"""

    @classmethod
    def setUpClass(cls):
        cls.reference = RobotKinematics(URDF, label="old")
        cls.q_samples = _random_joint_angles(8, seed=11)

    def test_zero_for_identical_model(self):
        candidate = RobotKinematics(URDF, DhOverride.from_base(URDF), label="new")
        report = fk_difference(self.reference, candidate, self.q_samples)
        self.assertEqual(report["n"], len(self.q_samples))
        self.assertEqual(report["position_max_mm"], 0.0)
        self.assertEqual(report["position_rms_mm"], 0.0)
        self.assertEqual(report["rotation_max_deg"], 0.0)
        self.assertEqual(report["rotation_rms_deg"], 0.0)
        self.assertEqual(len(report["per_sample"]), len(self.q_samples))
        self.assertEqual(report["per_sample"][0]["index"], 0)

    def test_translation_perturbation_is_reported(self):
        override = DhOverride.from_base(URDF)
        override.d[0] += 10.0
        report = fk_difference(
            self.reference, RobotKinematics(URDF, override), self.q_samples)
        self.assertAlmostEqual(report["position_max_mm"], 10.0, places=9)
        self.assertAlmostEqual(report["position_rms_mm"], 10.0, places=9)
        self.assertLess(report["rotation_max_deg"], 1e-6)

    def test_link_length_perturbation_is_same_order(self):
        override = DhOverride.from_base(URDF)
        override.a[1] += 10.0
        report = fk_difference(
            self.reference, RobotKinematics(URDF, override), self.q_samples)
        self.assertGreater(report["position_max_mm"], 5.0)
        self.assertLessEqual(report["position_max_mm"], 20.0)
        self.assertGreater(report["position_rms_mm"], 0.0)
        self.assertEqual(len(report["per_sample"]), len(self.q_samples))

    def test_empty_sample_list(self):
        report = fk_difference(self.reference, self.reference, [])
        self.assertEqual(report["n"], 0)
        self.assertEqual(report["position_max_mm"], 0.0)
        self.assertEqual(report["per_sample"], [])


class JsonLoadTest(unittest.TestCase):
    """dh_override.json 的单位换算与错误处理。"""

    @classmethod
    def setUpClass(cls):
        cls.base = DhOverride.from_base(URDF)

    def test_roundtrip_default_units(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = self.base.to_json_dict()
            payload["name"] = "roundtrip"
            loaded = DhOverride.from_json(_write_json(directory, payload))
        self.assertEqual(loaded.name, "roundtrip")
        np.testing.assert_allclose(loaded.a, self.base.a, atol=0.0)
        np.testing.assert_allclose(loaded.alpha, self.base.alpha, atol=0.0)
        np.testing.assert_allclose(loaded.d, self.base.d, atol=0.0)
        np.testing.assert_allclose(loaded.theta_offset, self.base.theta_offset, atol=0.0)

    def test_units_missing_defaults_to_mm_and_deg(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = self.base.as_dict()
            loaded = DhOverride.from_json(_write_json(directory, payload))
        np.testing.assert_allclose(loaded.a, self.base.a, atol=0.0)
        np.testing.assert_allclose(loaded.alpha, self.base.alpha, atol=0.0)

    def test_metres_are_converted_to_millimetres(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = {
                "name": "metres",
                "a": [value / 1000.0 for value in self.base.a],
                "alpha": list(self.base.alpha),
                "d": [value / 1000.0 for value in self.base.d],
                "theta_offset": list(self.base.theta_offset),
                "units": {"length": "m", "angle": "deg"},
            }
            loaded = DhOverride.from_json(_write_json(directory, payload))
        np.testing.assert_allclose(loaded.a, self.base.a, atol=1e-12)
        np.testing.assert_allclose(loaded.d, self.base.d, atol=1e-12)
        np.testing.assert_allclose(loaded.alpha, self.base.alpha, atol=0.0)

    def test_radians_are_converted_to_degrees(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = {
                "name": "radians",
                "a": list(self.base.a),
                "alpha": [math.radians(value) for value in self.base.alpha],
                "d": list(self.base.d),
                "theta_offset": [math.radians(value) for value in self.base.theta_offset],
                "units": {"length": "mm", "angle": "rad"},
            }
            loaded = DhOverride.from_json(_write_json(directory, payload))
        np.testing.assert_allclose(loaded.alpha, self.base.alpha, atol=1e-12)
        np.testing.assert_allclose(loaded.theta_offset, self.base.theta_offset, atol=1e-12)
        np.testing.assert_allclose(loaded.a, self.base.a, atol=0.0)

    def test_unknown_units_raise(self):
        for units in ({"length": "inch"}, {"angle": "grad"}, {"length": "m", "angle": "rad2"}):
            with tempfile.TemporaryDirectory() as directory:
                payload = self.base.to_json_dict()
                payload["units"] = units
                with self.assertRaises(ValueError):
                    DhOverride.from_json(_write_json(directory, payload))

    def test_missing_field_raises(self):
        for key in DH_KEYS:
            with tempfile.TemporaryDirectory() as directory:
                payload = self.base.to_json_dict()
                payload.pop(key)
                with self.assertRaises(ValueError):
                    DhOverride.from_json(_write_json(directory, payload))


class ValidationTest(unittest.TestCase):
    """非法的 DH 覆盖必须在 RobotKinematics 构造时报错并指出键与下标。"""

    def _assert_invalid(self, override: DhOverride):
        with self.assertRaises(ValueError) as context:
            RobotKinematics(URDF, override)
        return str(context.exception)

    def test_seven_values_raise(self):
        message = self._assert_invalid(
            DhOverride([0.0] * 7, [0.0] * 6, [0.0] * 6, [0.0] * 6))
        self.assertIn("a", message)

    def test_nan_raises_and_names_component(self):
        override = DhOverride.from_base(URDF)
        override.d[2] = float("nan")
        message = self._assert_invalid(override)
        self.assertIn("d", message)
        self.assertIn("[2]", message)

    def test_alpha_out_of_range_raises(self):
        override = DhOverride.from_base(URDF)
        override.alpha[4] = 400.0
        message = self._assert_invalid(override)
        self.assertIn("alpha", message)
        self.assertIn("[4]", message)

    def test_theta_offset_out_of_range_raises(self):
        override = DhOverride.from_base(URDF)
        override.theta_offset[0] = -400.0
        self.assertIn("theta_offset", self._assert_invalid(override))

    def test_length_out_of_range_raises(self):
        override = DhOverride.from_base(URDF)
        override.a[1] = 6000.0
        message = self._assert_invalid(override)
        self.assertIn("a", message)
        self.assertIn("[1]", message)

    def test_infinite_value_raises(self):
        override = DhOverride.from_base(URDF)
        override.a[0] = float("inf")
        self._assert_invalid(override)

    def test_override_not_mutated_by_model(self):
        override = DhOverride(a=[1.0] * 6, alpha=[0.0] * 6, d=[2.0] * 6,
                             theta_offset=[0.0] * 6, name="raw")
        model = RobotKinematics(URDF, override)
        self.assertIsNone(override.base)
        self.assertFalse(override.is_identity())
        self.assertIs(model.override.base is None, False)


class IdentitySemanticsTest(unittest.TestCase):
    """is_identity：与基准等价才算 identity，全 0 覆盖不算。"""

    def test_from_base_is_identity(self):
        self.assertTrue(DhOverride.from_base(URDF).is_identity())

    def test_tolerance(self):
        override = DhOverride.from_base(URDF)
        override.a[0] += 1e-12
        self.assertTrue(override.is_identity(tol=1e-9))
        self.assertFalse(override.is_identity(tol=1e-15))

    def test_modified_override_is_not_identity(self):
        override = DhOverride.from_base(URDF)
        override.d[0] += 1e-3
        self.assertFalse(override.is_identity())

    def test_zero_override_is_not_identity(self):
        zero = DhOverride([0.0] * 6, [0.0] * 6, [0.0] * 6, [0.0] * 6)
        self.assertFalse(zero.is_identity())
        self.assertFalse(zero.attach_base(URDF).is_identity())

    def test_unbound_override_is_not_identity(self):
        unbound = DhOverride(list(DhOverride.from_base(URDF).a),
                             list(DhOverride.from_base(URDF).alpha),
                             list(DhOverride.from_base(URDF).d),
                             list(DhOverride.from_base(URDF).theta_offset))
        self.assertFalse(unbound.is_identity())
        self.assertTrue(unbound.attach_base(URDF).is_identity())

    def test_model_attaches_base_without_mutating_caller(self):
        override = DhOverride.from_base(URDF)
        override.a[1] += 10.0
        model = RobotKinematics(URDF, override)
        self.assertFalse(model.override.is_identity())
        self.assertIsNot(model.override, override)
        np.testing.assert_allclose(model.override.a, override.a, atol=0.0)
        self.assertIsNone(RobotKinematics(URDF).override)


if __name__ == "__main__":
    unittest.main()
