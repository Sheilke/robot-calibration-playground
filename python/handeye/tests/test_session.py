# -*- coding: utf-8 -*-
"""handeye.session 单元测试：示例会话、参考点偏置、矩阵单位、非法输入、FK 一致性。"""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye.pose_encoder import matrix_to_pose_fields
from handeye.se3 import from_rpy
from handeye.session import (
    Frame,
    Sample,
    SessionError,
    check_fk_consistency,
    load_session,
    session_from_dict,
)

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "data" / "configs" / "ch2000_origin.urdf"
SESSION_EXAMPLE = ROOT / "data" / "configs" / "session_example.json"
DH_OVERRIDE_EXAMPLE = ROOT / "data" / "configs" / "dh_override_example.json"


def _payload() -> dict:
    """读入示例会话的深拷贝，供每个用例独立改写。"""
    return json.loads(SESSION_EXAMPLE.read_text(encoding="utf-8"))


def _fk_recorded_payload(offsets=None) -> dict:
    """把示例会话的 ``recorded_pose`` 填成 FK 结果（可对指定 pose 加平移）。"""
    payload = _payload()
    session = load_session(SESSION_EXAMPLE)
    offsets = offsets or {}
    for sample in payload["samples"]:
        pose_id = sample["pose_id"]
        model = session.sample_by_id(pose_id)
        matrix = session.kinematics.fk(model.q_deg)
        matrix[:3, 3] += np.asarray(offsets.get(pose_id, [0.0, 0.0, 0.0]), dtype=float)
        position, orientation = matrix_to_pose_fields(matrix)
        sample["recorded_pose"] = {
            "position": [float(value) for value in position],
            "orientation": [float(value) for value in orientation],
        }
    return payload


class ExampleSessionTest(unittest.TestCase):
    """示例会话必须能直接加载，结构与参考点符合预期。"""

    @classmethod
    def setUpClass(cls):
        cls.session = load_session(SESSION_EXAMPLE)

    def test_shape_and_frames(self):
        self.assertEqual(len(self.session.samples), 3)
        for sample in self.session.samples:
            self.assertEqual(len(sample.frames), 2)
            self.assertEqual(len(sample.q_deg), 6)
            self.assertEqual(sample.target_ids(), {"7"})
        self.assertEqual(self.session.all_target_ids(), {"7"})

    def test_reference_effective_without_delta(self):
        self.assertIsNone(self.session.delta_p_base)
        np.testing.assert_allclose(
            self.session.reference_effective, [800.0, 200.0, 650.0], atol=1e-12)

    def test_handeye_matrix_loaded(self):
        self.assertIsNotNone(self.session.handeye_matrix)
        expected = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
        np.testing.assert_allclose(self.session.handeye_matrix, expected, atol=1e-12)

    def test_source_path_recorded(self):
        self.assertEqual(self.session.source_path, SESSION_EXAMPLE)

    def test_kinematics_matches_urdf_limits(self):
        self.assertEqual(self.session.kinematics.urdf_path, URDF)
        self.assertEqual(len(self.session.kinematics.limits), 6)
        for sample in self.session.samples:
            for value, (low, high) in zip(
                    sample.q_deg, self.session.kinematics.limits):
                self.assertTrue(low <= value <= high)

    def test_sample_by_id_found_and_missing(self):
        self.assertEqual(self.session.sample_by_id(2).pose_id, 2)
        with self.assertRaises(KeyError):
            self.session.sample_by_id(99)


class ReferenceDeltaTest(unittest.TestCase):
    """``delta_p_base`` 非 null 时参考点要正确相加。"""

    def test_delta_is_added(self):
        payload = _payload()
        payload["reference"]["delta_p_base"] = [10.0, -5.0, 20.0]
        session = session_from_dict(payload)
        np.testing.assert_allclose(
            session.reference_effective, [810.0, 195.0, 670.0], atol=1e-12)

    def test_null_delta_is_zero(self):
        payload = _payload()
        payload["reference"]["delta_p_base"] = None
        session = session_from_dict(payload)
        np.testing.assert_allclose(
            session.reference_effective, session.reference_point, atol=0.0)


class HandeyeMatrixUnitTest(unittest.TestCase):
    """手眼矩阵可为 null；米制输入要换算成 mm。"""

    def test_null_matrix(self):
        payload = _payload()
        payload["handeye_matrix"] = None
        session = session_from_dict(payload)
        self.assertIsNone(session.handeye_matrix)

    def test_metre_matrix_converted_to_mm(self):
        payload = _payload()
        millimetres = np.array(
            payload["handeye_matrix"]["matrix16_rowmajor"], dtype=float).reshape(4, 4)
        converted = millimetres.copy()
        converted[:3, 3] /= 1000.0
        payload["handeye_matrix"] = {
            "matrix16_rowmajor": [float(value) for value in converted.reshape(-1)],
            "units": "m",
        }
        session = session_from_dict(payload)
        np.testing.assert_allclose(session.handeye_matrix, millimetres, atol=1e-9)


class InvalidSessionTest(unittest.TestCase):
    """逐字段的非法输入都必须抛 SessionError 且消息可定位。"""

    def _assert_invalid(self, name, mutate, fragment):
        payload = _payload()
        mutate(payload)
        with self.subTest(case=name):
            with self.assertRaises(SessionError) as context:
                session_from_dict(payload)
            self.assertIn(fragment, str(context.exception))

    def test_invalid_inputs(self):
        def set_q5(payload):
            payload["samples"][0]["q_deg"] = [0.0] * 5

        def set_q_over_limit(payload):
            payload["samples"][1]["q_deg"][4] = 130.0

        def duplicate_pose_id(payload):
            payload["samples"][2]["pose_id"] = payload["samples"][0]["pose_id"]

        def empty_frames(payload):
            payload["samples"][0]["frames"] = []

        def empty_target_points(payload):
            payload["samples"][0]["frames"][0]["target_points"] = {}

        def two_number_point(payload):
            payload["samples"][0]["frames"][0]["target_points"]["7"] = [1.0, 2.0]

        def nan_point(payload):
            payload["samples"][0]["frames"][0]["target_points"]["7"] = [
                1.0, float("nan"), 3.0]

        def bad_convention(payload):
            payload["meta"]["joint_convention"] = "controller"

        def bad_units(payload):
            payload["meta"]["units"] = "m"

        def drop_reference(payload):
            del payload["reference"]

        def drop_samples(payload):
            del payload["samples"]

        def empty_samples(payload):
            payload["samples"] = []

        def bad_matrix_units(payload):
            payload["handeye_matrix"]["units"] = "cm"

        cases = [
            ("q 只有 5 个", set_q5,
             "samples[0].q_deg 必须为 6 个数，当前 5 个"),
            ("q 超限位", set_q_over_limit, "第 5 轴限位"),
            ("pose_id 重复", duplicate_pose_id, "pose_id 重复：1"),
            ("frames 为空", empty_frames,
             "samples[0].frames 必须为非空数组"),
            ("target_points 为空 dict", empty_target_points,
             "samples[0].frames[0].target_points 不能为空"),
            ("点只有 2 个数", two_number_point,
             "samples[0].frames[0].target_points['7'] 必须为 3 个数，当前 2 个"),
            ("点含 nan", nan_point, "不是有限值"),
            ("joint_convention 非 model", bad_convention, "meta.joint_convention"),
            ("units 非 mm", bad_units, "meta.units"),
            ("缺 reference", drop_reference, "缺少必填字段 'reference'"),
            ("缺 samples", drop_samples, "缺少必填字段 'samples'"),
            ("samples 为空", empty_samples, "samples 不能为空"),
            ("手眼矩阵单位非法", bad_matrix_units, "handeye_matrix.units"),
        ]
        for name, mutate, fragment in cases:
            self._assert_invalid(name, mutate, fragment)

    def test_over_limit_message_contains_axis_limits_and_value(self):
        payload = _payload()
        limits = load_session(SESSION_EXAMPLE).kinematics.limits
        low, high = limits[1]
        payload["samples"][0]["q_deg"][1] = high + 1.0
        with self.assertRaises(SessionError) as context:
            session_from_dict(payload)
        message = str(context.exception)
        self.assertIn("第 2 轴限位", message)
        self.assertIn(f"{low:g}", message)
        self.assertIn(f"{high:g}", message)
        self.assertIn(f"{high + 1.0:g}", message)

    def test_pose_id_must_be_integer(self):
        payload = _payload()
        payload["samples"][0]["pose_id"] = "1"
        with self.assertRaises(SessionError):
            session_from_dict(payload)

    def test_load_session_rejects_missing_file(self):
        with self.assertRaises(SessionError):
            load_session(ROOT / "data" / "configs" / "does_not_exist.json")

    def test_load_session_rejects_bad_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text("{not json", encoding="utf-8")
            with self.assertRaises(SessionError):
                load_session(path)


class FkConsistencyTest(unittest.TestCase):
    """``check_fk_consistency`` 报告 FK 与 recorded_pose 的差异。"""

    def test_recorded_from_fk_is_zero(self):
        session = session_from_dict(_fk_recorded_payload())
        report = check_fk_consistency(session)
        self.assertEqual(report["n_compared"], 3)
        self.assertEqual(report["n_missing"], 0)
        self.assertLess(report["position_max_mm"], 1e-9)
        self.assertLess(report["position_rms_mm"], 1e-9)
        self.assertLess(report["rotation_max_deg"], 1e-5)
        self.assertLess(report["rotation_rms_deg"], 1e-5)
        self.assertIsNotNone(report["worst"])
        self.assertEqual(len(report["per_sample"]), 3)
        for entry in report["per_sample"]:
            self.assertLess(entry["position_mm"], 1e-9)

    def test_five_millimetre_offset_is_reported(self):
        session = session_from_dict(
            _fk_recorded_payload(offsets={2: [5.0, 0.0, 0.0]}))
        report = check_fk_consistency(session)
        self.assertAlmostEqual(report["position_max_mm"], 5.0, places=9)
        self.assertAlmostEqual(report["worst"]["position_mm"], 5.0, places=9)
        self.assertEqual(report["worst"]["pose_id"], 2)

    def test_missing_recorded_pose_counted(self):
        payload = _fk_recorded_payload()
        payload["samples"][2]["recorded_pose"] = None
        report = check_fk_consistency(session_from_dict(payload))
        self.assertEqual(report["n_compared"], 2)
        self.assertEqual(report["n_missing"], 1)
        self.assertEqual([entry["pose_id"] for entry in report["per_sample"]],
                         [1, 2])


class DataModelTest(unittest.TestCase):
    """Frame / Sample 的独立行为。"""

    def test_frame_len_and_ids_union(self):
        frame = Frame({"7": [1.0, 2.0, 3.0], "9": [4.0, 5.0, 6.0]})
        self.assertEqual(len(frame), 2)
        sample = Sample(1, [0.0] * 6, [
            {"target_points": {"7": [1.0, 2.0, 3.0]}},
            {"7": [1.0, 2.0, 3.0]},
            Frame({"9": [0.0, 0.0, 0.0]}),
        ])
        self.assertEqual(sample.target_ids(), {"7", "9"})
        self.assertEqual(len(sample.frames), 3)
        self.assertIsNone(sample.recorded_pose)

    def test_dh_override_example_matches_base(self):
        from handeye.dh import DhOverride

        override = DhOverride.from_json(DH_OVERRIDE_EXAMPLE)
        override.attach_base(URDF)
        self.assertTrue(override.is_identity())
        self.assertEqual(override.a, [150.0, 780.0, 250.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
