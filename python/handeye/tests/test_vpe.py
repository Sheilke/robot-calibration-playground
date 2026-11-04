# -*- coding: utf-8 -*-
"""handeye.vpe 单元测试：手算对照、无噪声、已知偏差、多帧/多 id、分位数、矩阵对比。"""

import math
import unittest
from pathlib import Path

import numpy as np

from handeye.se3 import from_rpy
from handeye.session import Sample, Session, load_session
from handeye.vpe import (
    VpeError,
    aggregate_sample_positions,
    compare_matrices,
    compute_vpe,
    vision_ball_base,
    vision_ball_base_frame,
)


class _StubKinematics:
    """按 ``q_deg[0]`` 返回预设 ``T_bh`` 的假运动学，便于用手算公式对照。"""

    def __init__(self, matrices):
        self.matrices = [np.asarray(matrix, dtype=float) for matrix in matrices]
        self.limits = None

    def fk(self, q_deg):
        return self.matrices[int(round(q_deg[0]))]


def _q(index: int) -> list:
    """把样本序号塞进第一个关节角，供 :class:`_StubKinematics` 选矩阵。"""
    return [float(index), 0.0, 0.0, 0.0, 0.0, 0.0]


def _session(matrices, handeye, reference, samples, delta=None):
    """用假运动学构造 Session（不依赖 URDF）。"""
    return Session(
        meta={},
        reference_point=reference,
        delta_p_base=delta,
        handeye_matrix=handeye,
        samples=samples,
        kinematics=_StubKinematics(matrices),
    )


def _project(t_bh, matrix, p_cam) -> np.ndarray:
    """测试侧独立公式：``(T_bh @ X @ [p_cam; 1])[:3]``（raw 4x4 乘法）。"""
    homogeneous = np.append(np.asarray(p_cam, dtype=float), 1.0)
    return (np.asarray(t_bh, dtype=float) @ np.asarray(matrix, dtype=float)
            @ homogeneous)[:3]


def _single_id_sample(pose_id: int, p_cam) -> Sample:
    """一个位姿、一帧、一个 id 的样本。"""
    return Sample(pose_id, _q(pose_id), [{"target_points": {"7": list(p_cam)}}])


class HandCalculationTest(unittest.TestCase):
    """三个已知样本：逐样本误差与全部指标都用独立公式重算。"""

    def setUp(self):
        self.matrix = from_rpy((40.0, -20.0, 90.0), (10.0, -5.0, 30.0))
        self.reference = np.array([800.0, 200.0, 650.0])
        self.t_bh = [
            from_rpy((900.0, 100.0, 1400.0), (5.0, -20.0, 40.0)),
            from_rpy((1000.0, -150.0, 1500.0), (-15.0, 10.0, -70.0)),
            from_rpy((700.0, 300.0, 1600.0), (30.0, 25.0, 120.0)),
        ]
        self.p_cam = [
            np.array([120.0, -40.0, 480.0]),
            np.array([-200.0, 30.0, 520.0]),
            np.array([60.0, 90.0, 610.0]),
        ]
        samples = [_single_id_sample(index, p_cam)
                   for index, p_cam in enumerate(self.p_cam)]
        self.session = _session(self.t_bh, self.matrix, self.reference, samples)

    def test_errors_and_metrics_match_hand_calculation(self):
        result = compute_vpe(self.session)

        positions = np.vstack([
            _project(t_bh, self.matrix, p_cam)
            for t_bh, p_cam in zip(self.t_bh, self.p_cam)])
        errors = positions - self.reference
        vpe = np.linalg.norm(errors, axis=1)
        expected_rmse = math.sqrt(float(np.mean(vpe ** 2)))
        expected_bias = errors.mean(axis=0)
        expected_p95 = float(np.percentile(vpe, 95))

        self.assertEqual(result["n_samples"], 3)
        self.assertEqual(result["n_frames"], 3)
        np.testing.assert_allclose(result["reference_mm"], self.reference, atol=0.0)
        np.testing.assert_allclose(result["matrix"], self.matrix, atol=0.0)

        for entry, position, error, value in zip(
                result["errors"], positions, errors, vpe):
            np.testing.assert_allclose(entry["position_vision"], position, atol=1e-9)
            np.testing.assert_allclose(entry["error"], error, atol=1e-9)
            self.assertAlmostEqual(entry["vpe_mm"], float(value), delta=1e-9)
            self.assertEqual(entry["n_frames"], 1)
            self.assertEqual(entry["n_ids"], 1)

        metrics = result["metrics"]
        self.assertAlmostEqual(metrics["vpe_rmse_mm"], expected_rmse, delta=1e-9)
        self.assertAlmostEqual(metrics["vpe_max_mm"], float(vpe.max()), delta=1e-9)
        self.assertAlmostEqual(metrics["vpe_mean_mm"], float(vpe.mean()), delta=1e-9)
        self.assertAlmostEqual(metrics["vpe_p95_mm"], expected_p95, delta=1e-9)
        np.testing.assert_allclose(metrics["bias"], expected_bias, atol=1e-9)
        self.assertAlmostEqual(
            metrics["bias_norm_mm"], float(np.linalg.norm(expected_bias)), delta=1e-9)
        for index, axis in enumerate(("x", "y", "z")):
            self.assertAlmostEqual(
                metrics["per_axis_rmse_mm"][axis],
                math.sqrt(float(np.mean(errors[:, index] ** 2))), delta=1e-9)
            self.assertAlmostEqual(
                metrics["per_axis_bias_mm"][axis],
                float(expected_bias[index]), delta=1e-9)


class PerfectDataTest(unittest.TestCase):
    """无噪声 + 真值 X：VPE 必须为零。"""

    def test_zero_vpe(self):
        matrix = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
        reference = np.array([800.0, 200.0, 650.0])
        t_bh = [
            from_rpy((950.0, 250.0, 1500.0), (0.0, -90.0, 0.0)),
            from_rpy((600.0, -100.0, 1200.0), (30.0, 20.0, 60.0)),
            from_rpy((1100.0, 400.0, 1700.0), (-45.0, 10.0, -120.0)),
        ]
        samples = []
        for index, pose in enumerate(t_bh):
            p_cam = np.linalg.inv(pose @ matrix) @ np.append(reference, 1.0)
            samples.append(_single_id_sample(index, p_cam[:3]))
        result = compute_vpe(_session(t_bh, matrix, reference, samples))
        metrics = result["metrics"]
        self.assertLess(metrics["vpe_rmse_mm"], 1e-9)
        self.assertLess(metrics["vpe_max_mm"], 1e-9)
        self.assertLess(metrics["bias_norm_mm"], 1e-9)


class KnownMatrixBiasTest(unittest.TestCase):
    """把 X 的平移 x 加 2 mm：bias 应恰好是 [2, 0, 0]。"""

    def test_bias_direction(self):
        true_matrix = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
        reference = np.array([800.0, 200.0, 650.0])
        # 纯平移的 T_bh（旋转=I）：X 平移 +2 mm 会原样传到 base 系
        t_bh = [from_rpy((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                from_rpy((120.0, -40.0, 60.0), (0.0, 0.0, 0.0))]
        samples = []
        for index, pose in enumerate(t_bh):
            p_cam = np.linalg.inv(pose @ true_matrix) @ np.append(reference, 1.0)
            samples.append(_single_id_sample(index, p_cam[:3]))

        bad_matrix = true_matrix.copy()
        bad_matrix[0, 3] += 2.0
        result = compute_vpe(
            _session(t_bh, true_matrix, reference, samples), bad_matrix)

        np.testing.assert_allclose(result["metrics"]["bias"], [2.0, 0.0, 0.0], atol=1e-9)
        self.assertAlmostEqual(result["metrics"]["bias_norm_mm"], 2.0, delta=1e-9)
        self.assertAlmostEqual(
            result["metrics"]["per_axis_bias_mm"]["x"], 2.0, delta=1e-9)
        self.assertAlmostEqual(
            result["metrics"]["per_axis_bias_mm"]["y"], 0.0, delta=1e-9)
        self.assertAlmostEqual(
            result["metrics"]["per_axis_rmse_mm"]["x"], 2.0, delta=1e-9)
        np.testing.assert_allclose(result["matrix"], bad_matrix, atol=0.0)
        for entry in result["errors"]:
            self.assertAlmostEqual(entry["vpe_mm"], 2.0, delta=1e-9)


class MultiFrameAggregationTest(unittest.TestCase):
    """同一位姿两帧同 id：spread 等于手工算的均值距离，n_frames 为 2。"""

    def setUp(self):
        self.matrix = from_rpy((40.0, -20.0, 90.0), (10.0, -5.0, 30.0))
        self.t_bh = from_rpy((900.0, 100.0, 1400.0), (5.0, -20.0, 40.0))
        self.p_cam = [np.array([120.0, -40.0, 480.0]),
                      np.array([121.0, -39.0, 481.5])]
        frames = [{"target_points": {"7": list(p)}} for p in self.p_cam]
        self.sample = Sample(1, _q(0), frames)
        self.session = _session([self.t_bh], self.matrix,
                                [800.0, 200.0, 650.0], [self.sample])

    def test_spread_and_frame_counts(self):
        positions = np.vstack([_project(self.t_bh, self.matrix, p)
                               for p in self.p_cam])
        mean = positions.mean(axis=0)
        expected_spread = float(np.mean(
            np.linalg.norm(positions - mean, axis=1)))

        aggregate = aggregate_sample_positions(self.session, self.sample)
        self.assertEqual(aggregate["ids"]["7"]["n"], 2)
        np.testing.assert_allclose(aggregate["ids"]["7"]["mean"], mean, atol=1e-9)
        self.assertAlmostEqual(
            aggregate["ids"]["7"]["spread_mm"], expected_spread, delta=1e-9)
        np.testing.assert_allclose(aggregate["combined"], mean, atol=1e-9)

        result = compute_vpe(self.session)
        self.assertEqual(result["n_frames"], 2)
        self.assertEqual(result["errors"][0]["n_frames"], 2)
        self.assertEqual(result["errors"][0]["n_ids"], 1)
        self.assertAlmostEqual(
            result["errors"][0]["spread_mm"], expected_spread, delta=1e-9)
        self.assertAlmostEqual(
            result["spread_mean_mm"], expected_spread, delta=1e-9)


class MultiIdTest(unittest.TestCase):
    """一帧两个 id：combined 是两者 mean 的算术平均。"""

    def setUp(self):
        self.matrix = from_rpy((40.0, -20.0, 90.0), (10.0, -5.0, 30.0))
        self.t_bh = from_rpy((850.0, 50.0, 1450.0), (-10.0, 15.0, 80.0))
        self.points = {"7": [120.0, -40.0, 480.0], "9": [90.0, -35.0, 520.0]}
        self.sample = Sample(1, _q(0), [{"target_points": self.points}])
        self.session = _session([self.t_bh], self.matrix,
                                [800.0, 200.0, 650.0], [self.sample])

    def test_combined_is_mean_of_id_means(self):
        projected = {target_id: _project(self.t_bh, self.matrix, point)
                     for target_id, point in self.points.items()}
        expected = (projected["7"] + projected["9"]) / 2.0

        aggregate = aggregate_sample_positions(self.session, self.sample)
        self.assertEqual(set(aggregate["ids"]), {"7", "9"})
        for target_id, point in projected.items():
            np.testing.assert_allclose(
                aggregate["ids"][target_id]["mean"], point, atol=1e-9)
            self.assertEqual(aggregate["ids"][target_id]["n"], 1)
            self.assertEqual(aggregate["ids"][target_id]["spread_mm"], 0.0)
        np.testing.assert_allclose(aggregate["combined"], expected, atol=1e-9)
        np.testing.assert_allclose(
            vision_ball_base(self.session, self.sample), expected, atol=1e-9)

        frame_result = vision_ball_base_frame(
            self.session, self.sample, self.sample.frames[0])
        self.assertIsInstance(frame_result, dict)
        for target_id, point in projected.items():
            np.testing.assert_allclose(frame_result[target_id], point, atol=1e-9)

    def test_single_id_frame_returns_array(self):
        sample = _single_id_sample(0, self.points["7"])
        session = _session([self.t_bh], self.matrix,
                           [800.0, 200.0, 650.0], [sample])
        frame_result = vision_ball_base_frame(session, sample, sample.frames[0])
        self.assertIsInstance(frame_result, np.ndarray)
        self.assertEqual(frame_result.shape, (3,))
        np.testing.assert_allclose(
            frame_result, _project(self.t_bh, self.matrix, self.points["7"]),
            atol=1e-9)


class PercentileTest(unittest.TestCase):
    """``vpe_p95_mm`` 必须等于 ``np.percentile(vpe, 95)``。"""

    def test_p95(self):
        matrix = from_rpy((10.0, 20.0, 30.0), (0.0, 0.0, 0.0))
        reference = np.array([800.0, 200.0, 650.0])
        identity = np.eye(4)
        samples = []
        for index in range(5):
            offset = np.array([index * 1.0, -index * 0.5, index * 0.25])
            p_cam = np.linalg.inv(matrix) @ np.append(reference + offset, 1.0)
            samples.append(_single_id_sample(index, p_cam[:3]))
        result = compute_vpe(
            _session([identity] * 5, matrix, reference, samples))
        values = np.array([entry["vpe_mm"] for entry in result["errors"]])
        self.assertAlmostEqual(
            result["metrics"]["vpe_p95_mm"],
            float(np.percentile(values, 95)), delta=1e-12)


class CompareMatricesTest(unittest.TestCase):
    """``compare_matrices`` 给出平移差（mm）与相对转角（deg）。"""

    def test_translation_and_rotation(self):
        first = from_rpy((10.0, 20.0, 30.0), (0.0, 0.0, 0.0))
        second = from_rpy((13.0, 24.0, 30.0), (0.0, 0.0, 90.0))
        result = compare_matrices(first, second)
        self.assertAlmostEqual(result["position_mm"], 5.0, places=9)
        self.assertAlmostEqual(result["rotation_deg"], 90.0, places=6)

    def test_identical_matrices(self):
        matrix = from_rpy((1.0, 2.0, 3.0), (10.0, 20.0, 30.0))
        result = compare_matrices(matrix, matrix.copy())
        self.assertAlmostEqual(result["position_mm"], 0.0, places=12)
        self.assertAlmostEqual(result["rotation_deg"], 0.0, places=9)


class MissingMatrixTest(unittest.TestCase):
    """matrix=None 且 session 也没有 X 时必须抛 VpeError。"""

    def test_missing_matrix_raises(self):
        matrix = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
        reference = np.array([800.0, 200.0, 650.0])
        t_bh = np.eye(4)
        p_cam = np.linalg.inv(matrix) @ np.append(reference, 1.0)
        sample = _single_id_sample(0, p_cam[:3])
        session = _session([t_bh], None, reference, [sample])

        with self.assertRaises(VpeError):
            compute_vpe(session)
        with self.assertRaises(VpeError):
            aggregate_sample_positions(session, sample)
        with self.assertRaises(VpeError):
            vision_ball_base(session, sample)
        with self.assertRaises(VpeError):
            vision_ball_base_frame(session, sample, sample.frames[0])


class EmptySamplesTest(unittest.TestCase):
    """空 samples：按 compute_vpe docstring 约定**抛 VpeError**。"""

    def test_empty_samples_raises(self):
        session = Session(
            meta={},
            reference_point=[800.0, 200.0, 650.0],
            delta_p_base=None,
            handeye_matrix=np.eye(4),
            samples=[],
            kinematics=_StubKinematics([]),
        )
        with self.assertRaises(VpeError) as context:
            compute_vpe(session)
        self.assertIn("samples 为空", str(context.exception))
        # 行为已写进 docstring
        self.assertIn("抛", compute_vpe.__doc__)


class ExampleSessionVpeTest(unittest.TestCase):
    """``data/configs/session_example.json`` 的 VPE 必须是亚毫米（示例数据自洽）。"""

    def test_example_session_is_sub_millimetre(self):
        example = (Path(__file__).resolve().parents[3]
                   / "data" / "configs" / "session_example.json")
        result = compute_vpe(load_session(example))
        self.assertEqual(result["n_samples"], 3)
        self.assertLess(result["metrics"]["vpe_rmse_mm"], 1.0)
        self.assertLess(result["spread_mean_mm"], 1.0)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
