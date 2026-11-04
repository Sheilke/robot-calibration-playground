# -*- coding: utf-8 -*-
"""handeye.offline_recompute 单元测试。

覆盖：protobuf wire format（varint / goal 编码 / 结果解析）、标定-验证划分、
「假求解器」端到端重算（含 recorded vs fk_old 两种旧侧位姿来源）、命令后端的
失败路径、markdown 报告与 CLI 退出码。

端到端用例依赖公司 proto（运行时由 protoc 生成），CI 缺失时整组跳过；
**任何用例都不调用 docker / ROS**：求解器统一用 ``CommandSolverBackend``
模板里「复制预设 ``result.bin``」的假命令。
"""

import contextlib
import io
import json
import math
import re
import shutil
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye.dh import DhOverride, RobotKinematics
from handeye.offline_recompute import (
    DEFAULT_SOLVER_CMD,
    CommandSolverBackend,
    SolverBackend,
    SolverError,
    encode_goal_payload,
    main,
    parse_calibration_result,
    render_report,
    run_offline_recompute,
    split_ids,
    varint_decode,
    varint_encode,
)
from handeye.pb_rewrite import compare_recorded_vs_fk
from handeye.pose_encoder import matrix_to_pose_fields
from handeye.proto_codec import load_pb_modules, write_recording, read_recording
from handeye.se3 import (
    from_rpy,
    inverse,
    rotation_angle_between,
    to_matrix16,
)
from handeye.session import session_from_dict
from handeye.vpe import compare_matrices

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "ch2000_origin.urdf"

#: 「假求解器」：把 ``{preset_bin}`` 指向的预设结果复制成 ``{output_bin}``。
FAKE_SOLVER = (
    "python3 -c \"import shutil,sys; shutil.copy(r'{preset_bin}', r'{output_bin}')\""
)

#: 6 个都在 URDF 限位内的位姿（度，模型轴角）。
Q_DEG = [
    [30.0, -30.0, -70.0, 95.0, -100.0, -270.0],
    [0.0, 90.0, -70.0, 5.0, 100.0, -80.0],
    [25.0, 50.0, -60.0, 165.0, -45.0, 355.0],
    [-40.0, -10.0, 45.0, -120.0, 60.0, 120.0],
    [70.0, 20.0, -20.0, 30.0, -90.0, -15.0],
    [-15.0, 60.0, 10.0, 140.0, 110.0, 200.0],
]

#: 激光跟踪仪实测的靶球中心（基座系，mm）。
REFERENCE = np.array([800.0, 200.0, 650.0])

#: 控制器回传位姿与 URDF FK 之间的细微差异（模拟「模型 ≠ 控制器」）。
CONTROLLER_OFFSET = from_rpy((0.25, -0.10, 0.05), (0.0, 0.0, 0.02))


def _bindings_available() -> bool:
    """protoc + 公司 proto 是否都可用（决定端到端用例能否跑）。"""
    if shutil.which("protoc") is None:
        return False
    try:
        load_pb_modules()
    except Exception:
        return False
    return True


BINDINGS_AVAILABLE = _bindings_available()

SKIP_REASON = ("公司 proto / protoc 不可用（需要 arc-common proto_msgs 与 "
               "protobuf-compiler），跳过依赖 .pb 的离线重算用例")


# ---------------------------------------------------------------------------
# 测试辅助：手工编码 EyeOnHandCalibrationResult（parse_calibration_result 的逆）
# ---------------------------------------------------------------------------


def encode_calibration_result(matrix16, *, success=True, compute_error=0.01,
                              packed=True, unknown=()):
    """手工编码 ``EyeOnHandCalibrationResult``（仅测试使用）。

    Args:
        matrix16: 16 个 double（row-major）。
        success: bool 字段。
        compute_error: double 字段（field 3）。
        packed: ``True`` 用 packed（wire type 2）编码 matrix，``False`` 逐个
            wire type 1。
        unknown: 追加的未知字段 ``[(字段号, 值)]``（varint）。

    Returns:
        ``bytes``。
    """
    parts = [b"\x08" + varint_encode(1 if success else 0)]
    payload = struct.pack("<16d", *[float(value) for value in matrix16])
    if packed:
        parts.append(b"\x12" + varint_encode(len(payload)) + payload)
    else:
        parts.extend(b"\x11" + struct.pack("<d", value)
                     for value in [float(v) for v in matrix16])
    parts.append(b"\x19" + struct.pack("<d", float(compute_error)))
    for field_number, value in unknown:
        key = (int(field_number) << 3) | 0
        parts.append(varint_encode(key) + varint_encode(int(value)))
    return b"".join(parts)


def _handmade_rmse(samples, kinematics, matrix, reference) -> float:
    """测试侧独立公式重算 VPE_RMSE：``sqrt(mean(||T_bh·X·p_cam − P||²))``。"""
    errors = []
    for sample in samples:
        for frame in sample.frames:
            for point in frame.target_points.values():
                homogeneous = np.append(np.asarray(point, dtype=float), 1.0)
                base = (np.asarray(kinematics.fk(sample.q_deg), dtype=float)
                        @ np.asarray(matrix, dtype=float) @ homogeneous)
                errors.append(float(np.linalg.norm(base[:3] - reference)))
    return math.sqrt(sum(value * value for value in errors) / len(errors))


class _ExplodingBackend(SolverBackend):
    """被调用就失败的假后端（用于证明 dry-run 不会触碰求解器）。"""

    name = "exploding"

    def solve(self, input_dir):  # pragma: no cover - 只应在出错时执行
        raise AssertionError(f"dry-run 不应调用求解器（input_dir={input_dir}）")


class _StubSample:
    """只有 ``pose_id`` 的假样本（供 ``split_ids`` 使用）。"""

    def __init__(self, pose_id):
        self.pose_id = pose_id


#: ``_run`` 的「未指定 backend」哨兵（区别于显式传 ``None``）。
_UNSET = object()


# ---------------------------------------------------------------------------
# 1-2. varint 与 goal 编码
# ---------------------------------------------------------------------------


class VarintTest(unittest.TestCase):
    """``varint_encode`` / ``varint_decode`` 的往返与非法输入。"""

    def test_roundtrip(self):
        for value in (0, 1, 127, 128, 300, 2 ** 20):
            encoded = varint_encode(value)
            decoded, offset = varint_decode(encoded)
            self.assertEqual(decoded, value, msg=f"value={value}")
            self.assertEqual(offset, len(encoded))

    def test_known_encodings(self):
        self.assertEqual(varint_encode(0), b"\x00")
        self.assertEqual(varint_encode(127), b"\x7f")
        self.assertEqual(varint_encode(128), b"\x80\x01")
        self.assertEqual(varint_encode(300), b"\xac\x02")
        self.assertEqual(varint_encode(2 ** 20), b"\x80\x80\x40")

    def test_decode_with_offset(self):
        buffer = b"\x08" + varint_encode(300) + b"\x19"
        value, offset = varint_decode(buffer, 1)
        self.assertEqual((value, offset), (300, 3))
        self.assertEqual(offset, 3)
        self.assertEqual(varint_decode(buffer, offset), (0x19, 4))

    def test_decode_accepts_bytes_like(self):
        encoded = varint_encode(300)
        self.assertEqual(varint_decode(bytearray(encoded))[0], 300)
        self.assertEqual(varint_decode(memoryview(encoded))[0], 300)

    def test_decode_truncated(self):
        for buffer in (b"", b"\x80", b"\x80\x80", b"\xff\xff"):
            with self.assertRaises(ValueError, msg=repr(buffer)):
                varint_decode(buffer)

    def test_decode_too_long(self):
        with self.assertRaises(ValueError):
            varint_decode(b"\x80" * 10)

    def test_decode_bad_offset_and_type(self):
        with self.assertRaises(ValueError):
            varint_decode(b"\x01", -1)
        with self.assertRaises(ValueError):
            varint_decode(b"\x01", 5)
        with self.assertRaises(ValueError):
            varint_decode("not bytes")

    def test_encode_rejects_invalid(self):
        with self.assertRaises(ValueError):
            varint_encode(-1)
        with self.assertRaises(ValueError):
            varint_encode("1")
        with self.assertRaises(ValueError):
            varint_encode(True)


class GoalPayloadTest(unittest.TestCase):
    """``EyeOnHandCalibrationGoal{input_directory}`` 的字节级契约。"""

    def test_known_payload(self):
        payload = encode_goal_payload("/tmp/x")
        self.assertEqual(payload, b"\x0a\x06/tmp/x")
        self.assertEqual(payload.hex(), "0a062f746d702f78")

    def test_long_path_uses_multi_byte_varint(self):
        path_value = "/data/aruco/" + "a" * 200
        payload = encode_goal_payload(Path(path_value))
        self.assertEqual(payload[0], 0x0A)
        length, offset = varint_decode(payload, 1)
        self.assertEqual(length, len(path_value.encode("utf-8")))
        self.assertGreater(offset, 2)  # 长度 > 127 -> varint 至少 2 字节
        self.assertEqual(payload[offset:].decode("utf-8"), path_value)

    def test_empty_rejected(self):
        for bad in ("", "   ", None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                encode_goal_payload(bad)


# ---------------------------------------------------------------------------
# 3. EyeOnHandCalibrationResult 解析
# ---------------------------------------------------------------------------


class ParseCalibrationResultTest(unittest.TestCase):
    """手写 wire format 解析：packed / 非 packed / 未知字段 / 非法输入。"""

    def setUp(self):
        self.matrix = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
        self.matrix16 = to_matrix16(self.matrix, units="mm")

    def test_packed(self):
        data = encode_calibration_result(
            self.matrix16, success=True, compute_error=0.0123, packed=True)
        parsed = parse_calibration_result(data)
        self.assertTrue(parsed["success"])
        self.assertEqual(parsed["matrix"], self.matrix16)
        self.assertAlmostEqual(parsed["compute_error"], 0.0123, delta=1e-15)
        self.assertEqual(parsed["unknown_fields"], [])
        np.testing.assert_allclose(
            np.asarray(parsed["matrix"]).reshape(4, 4), self.matrix, atol=0.0)

    def test_unpacked(self):
        data = encode_calibration_result(
            self.matrix16, success=True, compute_error=0.5, packed=False)
        parsed = parse_calibration_result(data)
        self.assertTrue(parsed["success"])
        self.assertEqual(parsed["matrix"], self.matrix16)
        self.assertAlmostEqual(parsed["compute_error"], 0.5, delta=1e-15)

    def test_mixed_packed_and_single(self):
        """前 4 个 double 逐个 wire type 1，其余 12 个 packed，也必须能解析。"""
        parts = [b"\x08\x01"]
        parts.extend(b"\x11" + struct.pack("<d", value)
                     for value in self.matrix16[:4])
        payload = struct.pack("<12d", *self.matrix16[4:])
        parts.append(b"\x12" + varint_encode(len(payload)) + payload)
        parts.append(b"\x19" + struct.pack("<d", 0.0))
        parsed = parse_calibration_result(b"".join(parts))
        self.assertEqual(parsed["matrix"], self.matrix16)

    def test_success_false(self):
        data = encode_calibration_result(self.matrix16, success=False)
        self.assertFalse(parse_calibration_result(data)["success"])

    def test_unknown_fields_are_skipped(self):
        base = bytearray(encode_calibration_result(self.matrix16))
        base += b"\x48" + varint_encode(7)                 # field 9, varint
        base += b"\x52" + varint_encode(3) + b"abc"        # field 10, length
        base += b"\x61" + struct.pack("<d", 1.0)           # field 12, fixed64
        base += b"\x6d" + struct.pack("<f", 1.0)           # field 13, fixed32
        parsed = parse_calibration_result(bytes(base))
        self.assertEqual(parsed["unknown_fields"], [9, 10, 12, 13])
        self.assertEqual(parsed["matrix"], self.matrix16)

    def test_wrong_matrix_length(self):
        for count in (0, 15, 17):
            data = b"".join(
                b"\x11" + struct.pack("<d", float(index))
                for index in range(count))
            with self.assertRaises(SolverError, msg=f"count={count}"):
                parse_calibration_result(data)

    def test_non_finite_rejected(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            matrix = list(self.matrix16)
            matrix[7] = bad
            with self.assertRaises(SolverError, msg=repr(bad)):
                parse_calibration_result(encode_calibration_result(matrix))

    def test_empty_bytes_rejected(self):
        with self.assertRaises(SolverError):
            parse_calibration_result(b"")

    def test_truncated_rejected(self):
        data = encode_calibration_result(self.matrix16)
        for cut in (1, 2, 4, len(data) - 1):
            with self.assertRaises(SolverError, msg=f"cut={cut}"):
                parse_calibration_result(data[:cut])

    def test_bad_input_type(self):
        for bad in ("", 123, None):
            with self.assertRaises(SolverError, msg=repr(bad)):
                parse_calibration_result(bad)


# ---------------------------------------------------------------------------
# 4. 标定 / 验证划分
# ---------------------------------------------------------------------------


class SplitIdsTest(unittest.TestCase):
    """``split_ids`` 的显式 / 半显式 / 比例分支与非法输入。"""

    def test_explicit_ids_are_sorted(self):
        samples = [_StubSample(index) for index in range(1, 7)]
        calib, valid = split_ids(
            samples, calib_ids=[3, 1, 2], valid_ids=[6, 4])
        self.assertEqual(calib, [1, 2, 3])
        self.assertEqual(valid, [4, 6])

    def test_only_calib_ids(self):
        samples = [_StubSample(index) for index in range(1, 7)]
        calib, valid = split_ids(samples, calib_ids=[1, 2, 3])
        self.assertEqual(calib, [1, 2, 3])
        self.assertEqual(valid, [4, 5, 6])

    def test_only_valid_ids(self):
        samples = [_StubSample(index) for index in range(1, 7)]
        calib, valid = split_ids(samples, valid_ids=[6])
        self.assertEqual(valid, [6])
        self.assertEqual(calib, [1, 2, 3, 4, 5])

    def test_accepts_dicts_and_integers(self):
        samples = [{"pose_id": index} for index in range(1, 7)]
        self.assertEqual(split_ids(samples, calib_ids=[1, 2, 3])[0], [1, 2, 3])
        self.assertEqual(split_ids(list(range(1, 7)), calib_ids=[1, 2, 3])[0],
                         [1, 2, 3])

    def test_ratio_is_deterministic(self):
        samples = [_StubSample(index) for index in range(1, 11)]
        first = split_ids(samples, split_ratio=0.7, seed=3)
        second = split_ids(samples, split_ratio=0.7, seed=3)
        self.assertEqual(first, second)
        self.assertEqual(len(first[0]), 7)
        self.assertEqual(len(first[1]), 3)
        self.assertEqual(first[0], sorted(first[0]))
        self.assertEqual(first[1], sorted(first[1]))

    def test_ratio_meets_minimum_samples(self):
        """6 个样本、默认 0.7 -> 4 标定 / 2 验证，且互不相交。"""
        samples = [_StubSample(index) for index in range(1, 7)]
        calib, valid = split_ids(samples, seed=0)
        self.assertGreaterEqual(len(calib), 3)
        self.assertGreaterEqual(len(valid), 1)
        self.assertEqual(len(calib) + len(valid), 6)
        self.assertFalse(set(calib) & set(valid))

    def test_different_seeds_can_differ(self):
        samples = [_StubSample(index) for index in range(1, 11)]
        outcomes = {
            tuple(split_ids(samples, seed=seed)[0]) for seed in range(6)}
        self.assertGreater(len(outcomes), 1)
        for outcome in outcomes:
            self.assertEqual(len(outcome), 7)

    def test_ratio_does_not_touch_global_random(self):
        import random as random_module

        state = random_module.getstate()
        split_ids([_StubSample(index) for index in range(1, 11)], seed=5)
        self.assertEqual(random_module.getstate(), state)

    def test_intersection_rejected(self):
        samples = [_StubSample(index) for index in range(1, 7)]
        with self.assertRaises(ValueError):
            split_ids(samples, calib_ids=[1, 2, 3], valid_ids=[3, 4])

    def test_unknown_and_duplicate_ids_rejected(self):
        samples = [_StubSample(index) for index in range(1, 7)]
        with self.assertRaises(ValueError):
            split_ids(samples, calib_ids=[1, 2, 99])
        with self.assertRaises(ValueError):
            split_ids(samples, calib_ids=[1, 1, 2])

    def test_too_few_samples_rejected(self):
        with self.assertRaises(ValueError):
            split_ids([_StubSample(index) for index in range(1, 4)])
        samples = [_StubSample(index) for index in range(1, 7)]
        with self.assertRaises(ValueError):
            split_ids(samples, calib_ids=[1, 2])
        with self.assertRaises(ValueError):
            split_ids(samples, calib_ids=[1, 2, 3, 4, 5, 6])

    def test_invalid_ratio_rejected(self):
        samples = [_StubSample(index) for index in range(1, 7)]
        for ratio in (0.0, 1.0, -0.5, 1.5, float("nan")):
            with self.assertRaises(ValueError, msg=repr(ratio)):
                split_ids(samples, split_ratio=ratio)


# ---------------------------------------------------------------------------
# 端到端数据集（.pb + 会话 + 预设 result.bin）
# ---------------------------------------------------------------------------


def _build_dataset(root: Path) -> dict:
    """在 ``root`` 下造 6 条 ``.pb``、会话 dict 与两个预设结果 ``.bin``。

    真值：``X_true`` 与旧 DH；每个样本的相机系靶点由
    ``p_cam = inverse(X_true) · inverse(T_bh_true) · [P; 1]`` 反算，
    因此 ``(θ_old, X_true)`` 组合的 VPE 为 0。``robot_pt`` 用
    ``T_bh(θ_old) · controller_offset`` 模拟控制器回传位姿。

    Returns:
        dict：``pb_dir`` / ``session_payload`` / ``x_true`` / ``x_old`` /
        ``preset_old`` / ``preset_new`` / ``model_old`` / ``override_new`` /
        ``q_deg`` / ``recorded_poses`` / ``p_cam``。
    """
    pb2, _ = load_pb_modules()
    pb_dir = root / "pb"
    pb_dir.mkdir(parents=True, exist_ok=True)

    model_old = RobotKinematics(URDF, label="old_dh")
    override_new = DhOverride.from_base(URDF, name="new_dh")
    override_new.d[0] += 2.0
    override_new.theta_offset[1] += 0.05

    x_true = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
    x_old = from_rpy((52.0, -30.5, 100.0), (5.3, -8.0, 120.0))

    samples = []
    recorded_poses = []
    p_cam_points = []
    for index, q_deg in enumerate(Q_DEG, start=1):
        t_bh_true = model_old.fk(q_deg)
        p_cam = (inverse(x_true) @ inverse(t_bh_true)
                 @ np.append(REFERENCE, 1.0))[:3]
        controller = t_bh_true @ CONTROLLER_OFFSET
        position, orientation = matrix_to_pose_fields(controller)

        message = pb2.ArucoRecordingResult()
        message.success = True
        message.output_image_path = f"/data/aruco/pose_{index:02d}.png"
        carrier = message.single_statics.robot_pt.base
        carrier.position.data[:] = position
        carrier.orientation.data[:] = orientation
        point = message.single_statics.board_statics.board_pts.add()
        point.id = 7
        point.x, point.y, point.z = [float(value) for value in p_cam]
        write_recording(message, pb_dir / f"pose_{index:02d}.pb")

        recorded_poses.append({"position": position, "orientation": orientation})
        p_cam_points.append([float(value) for value in p_cam])
        samples.append({
            "pose_id": index,
            "q_deg": q_deg,
            "recorded_pose": {"position": position, "orientation": orientation},
            "frames": [{"target_points": {"7": p_cam_points[-1]}}],
        })

    session_payload = {
        "meta": {
            "robot_urdf": str(URDF),
            "dh_override": None,
            "group": 4,
            "joint_convention": "model",
            "units": "mm",
        },
        "reference": {
            "p_ball_laser_base": REFERENCE.tolist(),
            "delta_p_base": None,
        },
        "handeye_matrix": None,
        "samples": samples,
    }

    preset_old = root / "preset_old.bin"
    preset_old.write_bytes(encode_calibration_result(
        to_matrix16(x_old, units="mm"), success=True, compute_error=0.02))
    preset_new = root / "preset_new.bin"
    preset_new.write_bytes(encode_calibration_result(
        to_matrix16(x_true, units="mm"), success=True, compute_error=0.01))

    return {
        "pb_dir": pb_dir,
        "session_payload": session_payload,
        "x_true": x_true,
        "x_old": x_old,
        "preset_old": preset_old,
        "preset_new": preset_new,
        "model_old": model_old,
        "override_new": override_new,
        "recorded_poses": recorded_poses,
        "p_cam": p_cam_points,
    }


def _fake_solver_command() -> str:
    """按输入目录名（``old_calib`` / ``new_calib``）选预设 bin 的假求解命令。"""
    return (
        "python3 -c \"import shutil,sys; src=r'{preset_old}' if 'new_calib' not in "
        "sys.argv[1] else r'{preset_new}'; shutil.copy(src, r'{output_bin}')\" "
        "{input_dir}"
    )


# ---------------------------------------------------------------------------
# 5-7. 端到端重算
# ---------------------------------------------------------------------------


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_REASON)
class OfflineRecomputeEndToEndTest(unittest.TestCase):
    """假求解器端到端：子集划分、改写、两套 X、4 种组合与手算对照。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="offline_recompute_"))
        cls.data = _build_dataset(cls.tempdir)
        cls.session = session_from_dict(cls.data["session_payload"])

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def _backend(self, work_dir):
        return CommandSolverBackend(
            _fake_solver_command(), timeout_s=60, work_dir=work_dir,
            extra_format={
                "preset_old": self.data["preset_old"],
                "preset_new": self.data["preset_new"],
            })

    def _run(self, *, backend=_UNSET, dry_run=False, **kwargs):
        work_dir = Path(tempfile.mkdtemp(dir=self.tempdir, prefix="work_"))
        result = run_offline_recompute(
            pb_dir=self.data["pb_dir"],
            session=self.session,
            dh_old=kwargs.pop("dh_old", self.data["model_old"]),
            dh_new=kwargs.pop("dh_new", self.data["override_new"]),
            backend=(self._backend(work_dir / "solver")
                     if backend is _UNSET else backend),
            calib_ids=kwargs.pop("calib_ids", [1, 2, 3, 4]),
            valid_ids=kwargs.pop("valid_ids", [5, 6]),
            work_dir=work_dir,
            dry_run=dry_run,
            **kwargs,
        )
        return result, work_dir

    def _valid_samples(self):
        return [sample for sample in self.session.samples
                if sample.pose_id in (5, 6)]

    def test_full_pipeline(self):
        result, work_dir = self._run()

        self.assertEqual(result["calib_ids"], [1, 2, 3, 4])
        self.assertEqual(result["valid_ids"], [5, 6])
        self.assertEqual(result["solver"]["old"]["backend"], "command")
        self.assertEqual(result["solver"]["new"]["backend"], "command")

        # 两套 X 等于预设 bin（假求解器原样复制）
        np.testing.assert_allclose(
            result["matrices"]["x_old"], self.data["x_old"], atol=1e-12)
        np.testing.assert_allclose(
            result["matrices"]["x_new"], self.data["x_true"], atol=1e-12)

        # matrix_delta 与预设矩阵的差异一致
        expected_delta = compare_matrices(self.data["x_old"], self.data["x_true"])
        self.assertAlmostEqual(result["matrix_delta"]["position_mm"],
                               expected_delta["position_mm"], delta=1e-6)
        self.assertAlmostEqual(result["matrix_delta"]["rotation_deg"],
                               expected_delta["rotation_deg"], delta=1e-6)
        self.assertAlmostEqual(
            result["matrix_delta"]["rotation_deg"],
            rotation_angle_between(self.data["x_old"][:3, :3],
                                   self.data["x_true"][:3, :3]),
            delta=1e-9)

        # 4 种组合、基线与主结果各一个
        combinations = result["combinations"]
        self.assertEqual(len(combinations), 4)
        self.assertEqual([entry["label"] for entry in combinations],
                         ["θ_old × X_old", "θ_new × X_new",
                          "θ_new × X_old", "θ_old × X_new"])
        self.assertEqual(sum(1 for entry in combinations
                             if entry["is_baseline"]), 1)
        self.assertEqual(sum(1 for entry in combinations
                             if entry["is_primary"]), 1)
        self.assertTrue(combinations[0]["is_baseline"])
        self.assertTrue(combinations[1]["is_primary"])
        self.assertEqual([entry["n_samples"] for entry in combinations],
                         [2, 2, 2, 2])

        # 逐组合与独立公式（raw 4x4 乘法）对照
        model_new = RobotKinematics(URDF, self.data["override_new"],
                                    label="new_dh")
        expected = {
            "θ_old × X_old": _handmade_rmse(
                self._valid_samples(), self.data["model_old"],
                self.data["x_old"], REFERENCE),
            "θ_new × X_new": _handmade_rmse(
                self._valid_samples(), model_new, self.data["x_true"], REFERENCE),
            "θ_new × X_old": _handmade_rmse(
                self._valid_samples(), model_new, self.data["x_old"], REFERENCE),
            "θ_old × X_new": _handmade_rmse(
                self._valid_samples(), self.data["model_old"],
                self.data["x_true"], REFERENCE),
        }
        for entry in combinations:
            self.assertAlmostEqual(entry["vpe_rmse_mm"],
                                   expected[entry["label"]], delta=1e-6,
                                   msg=entry["label"])

        # ΔVPE_RMSE = VPE_old - VPE_new（手算）
        expected_delta_vpe = expected["θ_old × X_old"] - expected["θ_new × X_new"]
        self.assertAlmostEqual(result["delta_vpe_rmse_mm"], expected_delta_vpe,
                               delta=1e-6)
        self.assertNotAlmostEqual(result["delta_vpe_rmse_mm"], 0.0, places=3)

        # 改写结果：旧一侧原样复制 4 个、新一侧改写 4 个
        self.assertEqual(result["rewrite"]["old"]["copied_unchanged"], 4)
        self.assertEqual(result["rewrite"]["new"]["changed"], 4)
        self.assertEqual(result["fk_consistency"]["n_compared"], 6)
        self.assertEqual(result["work_dir_temporary"], False)

        for subdir in ("old_calib", "new_calib"):
            names = sorted(path.name for path in (work_dir / subdir).glob("*.pb"))
            self.assertEqual(names,
                             [f"pose_{index:02d}.pb" for index in (1, 2, 3, 4)])
        self.assertEqual(result["config"]["n_calib_frames"], 4)
        self.assertEqual(result["config"]["n_valid_frames"], 2)

    def test_dry_run_skips_solver_and_writes_calib_dirs(self):
        result, work_dir = self._run(backend=_ExplodingBackend(), dry_run=True)
        self.assertEqual(result["combinations"], [])
        self.assertIsNone(result["matrices"]["x_old"])
        self.assertIsNone(result["matrices"]["x_new"])
        self.assertIsNone(result["matrix_delta"]["position_mm"])
        self.assertIsNone(result["delta_vpe_rmse_mm"])
        self.assertTrue(result["solver"]["old"]["skipped"])
        for subdir in ("old_calib", "new_calib"):
            self.assertEqual(len(list((work_dir / subdir).glob("*.pb"))), 4)

    def test_dry_run_allows_missing_backend(self):
        result, _ = self._run(backend=None, dry_run=True)
        self.assertEqual(result["combinations"], [])

    def test_missing_backend_rejected(self):
        with self.assertRaises(ValueError):
            self._run(backend=None, dry_run=False)

    def test_recorded_keeps_original_robot_pt(self):
        result, work_dir = self._run(pose_source_old="recorded")
        old_dir = work_dir / "old_calib"
        for path in sorted(old_dir.glob("*.pb")):
            self.assertEqual(path.read_bytes(),
                             (self.data["pb_dir"] / path.name).read_bytes(),
                             msg=f"{path.name} 不应被改写")
            index = int(path.stem.split("_")[-1]) - 1
            sample = self.session.samples[index]
            info = compare_recorded_vs_fk(
                read_recording(path),
                self.session.kinematics.fk(sample.q_deg))
            # 控制器位姿与模型有已知 0.25 mm 量级偏差，必须「一致」为 False
            self.assertFalse(info["consistent"])
        self.assertEqual(result["rewrite"]["old"]["changed"], 0)
        self.assertEqual(result["rewrite"]["old"]["copied_unchanged"], 4)

    def test_fk_old_rewrites_robot_pt(self):
        result, work_dir = self._run(pose_source_old="fk_old")
        old_dir = work_dir / "old_calib"
        changed = 0
        for name in sorted(old_dir.glob("*.pb")):
            self.assertNotEqual(name.read_bytes(),
                                (self.data["pb_dir"] / name.name).read_bytes(),
                                msg=f"{name.name} 应被 FK(θ_old) 改写")
            changed += 1
            index = int(name.stem.split("_")[-1]) - 1
            info = compare_recorded_vs_fk(
                read_recording(name),
                self.data["model_old"].fk(self.session.samples[index].q_deg))
            self.assertTrue(info["consistent"], msg=name.name)
        self.assertEqual(changed, 4)
        self.assertEqual(result["rewrite"]["old"]["changed"], 4)
        self.assertEqual(result["rewrite"]["old"]["copied_unchanged"], 0)

    def test_pb_count_mismatch_raises(self):
        short = Path(tempfile.mkdtemp(dir=self.tempdir, prefix="short_"))
        for path in sorted(self.data["pb_dir"].glob("*.pb"))[:5]:
            shutil.copyfile(path, short / path.name)
        with self.assertRaises(ValueError) as context:
            run_offline_recompute(
                pb_dir=short, session=self.session,
                dh_old=self.data["model_old"], dh_new=self.data["override_new"],
                backend=None, dry_run=True)
        message = str(context.exception)
        self.assertIn("5", message)
        self.assertIn("6", message)

    def test_kinematics_accepts_both_types(self):
        result, _ = self._run(dh_old=DhOverride.from_base(URDF, name="old_base"),
                              dh_new=RobotKinematics(
                                  URDF, self.data["override_new"], label="new_kin"))
        self.assertEqual(result["config"]["dh_old"], "old_base")
        self.assertEqual(result["config"]["dh_new"], "new_kin")

    def test_invalid_kinematics_type_rejected(self):
        with self.assertRaises(ValueError):
            self._run(dh_old="old.json", dry_run=True)

    def test_invalid_pose_source_rejected(self):
        with self.assertRaises(ValueError):
            self._run(pose_source_old="controller", dry_run=True)

    def test_ratio_split_is_deterministic(self):
        result, _ = self._run(calib_ids=None, valid_ids=None, split_ratio=0.7,
                              seed=4)
        self.assertEqual(len(result["calib_ids"]), 4)
        self.assertEqual(len(result["valid_ids"]), 2)
        self.assertEqual(result["config"]["split"]["mode"], "ratio")
        self.assertEqual(result["config"]["split"]["seed"], 4)

    def test_temporary_work_dir_is_cleaned(self):
        result = run_offline_recompute(
            pb_dir=self.data["pb_dir"], session=self.session,
            dh_old=self.data["model_old"], dh_new=self.data["override_new"],
            backend=self._backend(self.tempdir / "solver_autoclean"),
            calib_ids=[1, 2, 3, 4], valid_ids=[5, 6], work_dir=None)
        self.assertTrue(result["work_dir_temporary"])
        self.assertFalse(Path(result["work_dir"]).exists())


# ---------------------------------------------------------------------------
# 8. CommandSolverBackend 失败路径
# ---------------------------------------------------------------------------


class CommandSolverBackendTest(unittest.TestCase):
    """命令后端：非零退出 / 超时 / 结果缺失 / 缺占位符 / 结果非法。"""

    def setUp(self):
        self.tempdir = Path(tempfile.mkdtemp(prefix="solver_backend_"))
        self.input_dir = self.tempdir / "pb"
        self.input_dir.mkdir()
        (self.input_dir / "pose_01.pb").write_bytes(b"\x08\x01")
        self.work_dir = self.tempdir / "work"
        self.matrix = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
        self.preset = self.tempdir / "preset.bin"
        self.preset.write_bytes(encode_calibration_result(
            to_matrix16(self.matrix, units="mm"), compute_error=0.0123))

    def tearDown(self):
        shutil.rmtree(self.tempdir, ignore_errors=True)

    def _backend(self, command, **kwargs):
        kwargs.setdefault("work_dir", self.work_dir)
        kwargs.setdefault("timeout_s", 30)
        return CommandSolverBackend(command, **kwargs)

    def test_nonzero_exit_raises(self):
        backend = self._backend("python3 -c \"raise SystemExit(3)\"")
        with self.assertRaises(SolverError) as context:
            backend.solve(self.input_dir)
        self.assertIn("3", str(context.exception))

    def test_timeout_raises(self):
        backend = self._backend(
            "python3 -c \"import time; time.sleep(2)\"", timeout_s=0.2)
        with self.assertRaises(SolverError) as context:
            backend.solve(self.input_dir)
        self.assertIn("超时", str(context.exception))

    def test_missing_output_bin_raises(self):
        backend = self._backend("python3 -c \"pass\"")
        with self.assertRaises(SolverError) as context:
            backend.solve(self.input_dir)
        self.assertIn("未产出", str(context.exception))

    def test_missing_placeholder_raises_with_name(self):
        backend = self._backend(FAKE_SOLVER)
        with self.assertRaises(SolverError) as context:
            backend.solve(self.input_dir)
        self.assertIn("preset_bin", str(context.exception))

    def test_missing_input_dir_raises(self):
        backend = self._backend(FAKE_SOLVER,
                                extra_format={"preset_bin": self.preset})
        with self.assertRaises(SolverError):
            backend.solve(self.tempdir / "nope")

    def test_success_with_extra_format(self):
        backend = self._backend(FAKE_SOLVER,
                                extra_format={"preset_bin": self.preset})
        matrix, info = backend.solve(self.input_dir)
        np.testing.assert_allclose(matrix, self.matrix, atol=1e-12)
        self.assertEqual(info["backend"], "command")
        self.assertEqual(info["returncode"], 0)
        self.assertTrue(info["raw"]["success"])
        self.assertAlmostEqual(info["raw"]["compute_error"], 0.0123, delta=1e-15)
        self.assertEqual(len(info["raw"]["matrix"]), 16)

    def test_success_false_raises(self):
        failed = self.tempdir / "failed.bin"
        failed.write_bytes(encode_calibration_result(
            to_matrix16(self.matrix, units="mm"), success=False))
        backend = self._backend(FAKE_SOLVER, extra_format={"preset_bin": failed})
        with self.assertRaises(SolverError) as context:
            backend.solve(self.input_dir)
        self.assertIn("success=false", str(context.exception))

    def test_bad_matrix_in_result_raises(self):
        broken = self.tempdir / "broken.bin"
        broken.write_bytes(b"".join(
            b"\x11" + struct.pack("<d", float(index)) for index in range(3)))
        backend = self._backend(FAKE_SOLVER, extra_format={"preset_bin": broken})
        with self.assertRaises(SolverError):
            backend.solve(self.input_dir)

    def test_abstract_backend_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            SolverBackend().solve(self.input_dir)

    def test_invalid_constructor_arguments(self):
        with self.assertRaises(ValueError):
            CommandSolverBackend("")
        with self.assertRaises(ValueError):
            CommandSolverBackend("echo", timeout_s=0)
        with self.assertRaises(ValueError):
            CommandSolverBackend("echo", extra_format=[])
        with self.assertRaises(ValueError):
            CommandSolverBackend("echo", env=[])

    def test_default_solver_command_is_exposed(self):
        self.assertIn("{input_dir}", DEFAULT_SOLVER_CMD)
        self.assertIn("{output_bin}", DEFAULT_SOLVER_CMD)
        self.assertIn("ros2", DEFAULT_SOLVER_CMD)


# ---------------------------------------------------------------------------
# 9. markdown 报告
# ---------------------------------------------------------------------------


def _synthetic_result(*, delta=0.5, rmse_old=1.0, dry_run=False):
    """构造一份 ``run_offline_recompute`` 形状的结果（不依赖 .pb / 求解器）。"""
    matrix_old = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
    matrix_new = from_rpy((51.0, -30.0, 100.0), (5.2, -8.0, 120.0))
    rmse_new = rmse_old - delta
    specs = (
        ("θ_old × X_old", "old", "old", True, False, rmse_old),
        ("θ_new × X_new", "new", "new", False, True, rmse_new),
        ("θ_new × X_old", "new", "old", False, False, rmse_old * 0.9),
        ("θ_old × X_new", "old", "new", False, False, rmse_old * 1.1),
    )
    combinations = [
        {
            "label": label, "dh": dh, "x": x,
            "vpe_rmse_mm": rmse, "vpe_max_mm": rmse * 1.2,
            "vpe_p95_mm": rmse * 1.1, "bias_norm_mm": rmse * 0.8,
            "spread_mean_mm": 0.02, "n_samples": 6, "n_frames": 12,
            "is_baseline": baseline, "is_primary": primary,
        }
        for label, dh, x, baseline, primary, rmse in specs
    ]
    if dry_run:
        combinations = []
    return {
        "calib_ids": [1, 2, 3, 4],
        "valid_ids": [5, 6],
        "solver": {
            "old": {"backend": "command",
                    "raw": {"success": True, "compute_error": 0.021}},
            "new": {"backend": "command",
                    "raw": {"success": True, "compute_error": 0.009}},
        },
        "matrices": {"x_old": None if dry_run else matrix_old,
                     "x_new": None if dry_run else matrix_new},
        "matrix_delta": ({"position_mm": None, "rotation_deg": None}
                         if dry_run else compare_matrices(matrix_old, matrix_new)),
        "combinations": combinations,
        "delta_vpe_rmse_mm": None if dry_run else delta,
        "fk_consistency": {
            "n_compared": 6, "n_missing": 1,
            "position_max_mm": 0.42, "position_rms_mm": 0.31,
            "rotation_max_deg": 0.03, "rotation_rms_deg": 0.02,
            "worst": {"pose_id": 3, "position_mm": 0.42,
                      "rotation_deg": 0.03},
            "per_sample": [],
        },
        "rewrite": {"old": {"changed": 0}, "new": {"changed": 4}},
        "config": {
            # 故意用绝对路径，验证报告只显示相对路径/文件名
            "pb_dir": "/home/somebody/data/pb",
            "session": "/home/somebody/data/session.json",
            "dh_old": "old_dh", "dh_new": "new_dh",
            "pose_source_old": "recorded",
            "split": {"mode": "ratio", "ratio": 0.7, "seed": 0},
            "calib_ids": [1, 2, 3, 4], "valid_ids": [5, 6],
            "n_calib_samples": 4, "n_calib_frames": 8,
            "n_valid_samples": 2, "n_valid_frames": 4,
            "pb_files": [f"pose_{index:02d}.pb" for index in range(1, 7)],
        },
        "work_dir": "/tmp/handeye_recompute_example",
        "work_dir_temporary": True,
    }


class ReportTest(unittest.TestCase):
    """报告结构、可复现性与格式约定。"""

    SECTIONS = [
        "## 1. 实验配置",
        "## 2. 求解结果",
        "## 3. VPE 对比",
        "## 4. 结论",
        "## 5. 交叉组合诊断",
        "## 6. 运动学一致性自检",
        "## 7. 复现方式",
    ]

    def test_sections_in_order(self):
        text = render_report(_synthetic_result())
        headers = [line for line in text.splitlines() if line.startswith("## ")]
        self.assertEqual(headers, self.SECTIONS)

    def test_two_renders_are_byte_identical(self):
        first = render_report(_synthetic_result())
        second = render_report(_synthetic_result())
        self.assertEqual(first, second)
        self.assertTrue(first.endswith("\n"))

    def test_no_absolute_paths_or_timestamps(self):
        text = render_report(_synthetic_result())
        for marker in ("/home/", "/tmp/", "/var/", "/data/somebody"):
            self.assertNotIn(marker, text)
        self.assertIsNone(re.search(r"\d{4}-\d{2}-\d{2}", text))
        self.assertIsNone(re.search(r"\d{2}:\d{2}:\d{2}", text))
        self.assertNotIn("nan", text.lower())
        self.assertNotIn("inf", text.lower())

    def test_numbers_have_four_decimals(self):
        text = render_report(_synthetic_result())
        for number in re.findall(r"\d+\.\d+", text):
            self.assertEqual(len(number.split(".")[1]), 4, msg=number)

    def test_vpe_table_marks_baseline_and_primary(self):
        text = render_report(_synthetic_result())
        self.assertIn("| 组合 | VPE_RMSE (mm) | VPE_MAX (mm) | VPE_P95 (mm) | "
                      "‖bias‖ (mm) | spread (mm) | 样本数 |", text)
        bold_labels = [line for line in text.splitlines()
                       if line.startswith("| **") and "θ_" in line]
        self.assertEqual(len(bold_labels), 2)
        self.assertIn("θ_old × X_old", bold_labels[0])
        self.assertIn("θ_new × X_new", bold_labels[1])

    def test_cross_combination_caveat(self):
        text = render_report(_synthetic_result())
        self.assertIn("交叉组合**不具备物理一致性**，仅作诊断，不作最终性能结论", text)

    def test_verdict_thresholds(self):
        improved = render_report(_synthetic_result(delta=0.3, rmse_old=1.0))
        self.assertIn(
            "新 DH 使手眼定位精度改善超过 10%，支持采用标定后参数", improved)
        unchanged = render_report(_synthetic_result(delta=0.05, rmse_old=1.0))
        self.assertIn(
            "新旧 DH 对手眼定位精度影响在 ±10% 内，不足以支持结论，"
            "建议增加样本或复核标定残差", unchanged)
        worse = render_report(_synthetic_result(delta=-0.3, rmse_old=1.0))
        self.assertIn(
            "新 DH 使手眼定位精度变差，需检查 DH 标定结果与零点定义转换", worse)

    def test_delta_row_and_formula(self):
        text = render_report(_synthetic_result(delta=0.5, rmse_old=1.0))
        self.assertIn("ΔVPE_RMSE = VPE_old − VPE_new（正值 = 改善）", text)
        self.assertIn("0.5000 mm", text)
        self.assertIn("50.0000%", text)

    def test_dry_run_report(self):
        text = render_report(_synthetic_result(dry_run=True))
        headers = [line for line in text.splitlines() if line.startswith("## ")]
        self.assertEqual(headers, self.SECTIONS)
        self.assertIn("未执行求解", text)

    def test_invalid_input_rejected(self):
        with self.assertRaises(ValueError):
            render_report("not a result")


# ---------------------------------------------------------------------------
# 10. CLI
# ---------------------------------------------------------------------------


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_REASON)
class OfflineRecomputeCliTest(unittest.TestCase):
    """CLI：正常路径 / dry-run / 错误路径与退出码（不出现 traceback）。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="offline_cli_"))
        cls.data = _build_dataset(cls.tempdir)
        cls.session_path = cls.tempdir / "session.json"
        cls.session_path.write_text(
            json.dumps(cls.data["session_payload"], ensure_ascii=False),
            encoding="utf-8")
        cls.dh_old_path = cls.tempdir / "dh_old.json"
        cls.dh_old_path.write_text(
            json.dumps(DhOverride.from_base(URDF, name="old_dh").to_json_dict()),
            encoding="utf-8")
        cls.dh_new_path = cls.tempdir / "dh_new.json"
        cls.dh_new_path.write_text(
            json.dumps(cls.data["override_new"].to_json_dict()), encoding="utf-8")

        cls.solver_script = cls.tempdir / "fake_solver.py"
        cls.solver_script.write_text(
            "import shutil, sys\n"
            "input_dir, output_bin, preset_old, preset_new = sys.argv[1:5]\n"
            "shutil.copy(preset_old if 'new_calib' not in input_dir "
            "else preset_new, output_bin)\n",
            encoding="utf-8")
        cls.solver_cmd = (
            f"python3 {cls.solver_script} {{input_dir}} {{output_bin}} "
            f"{cls.data['preset_old']} {cls.data['preset_new']}")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def _invoke(self, argv):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = main(argv)
        return code, stdout.getvalue(), stderr.getvalue()

    def _base_argv(self, out, work_dir):
        return [
            "--pb-dir", str(self.data["pb_dir"]),
            "--session", str(self.session_path),
            "--dh-old", str(self.dh_old_path),
            "--dh-new", str(self.dh_new_path),
            "--calib-ids", "1,2,3,4",
            "--valid-ids", "5,6",
            "-o", str(out),
            "--work-dir", str(work_dir),
        ]

    def test_normal_run_writes_report_and_json(self):
        out = self.tempdir / "report_normal.md"
        json_out = self.tempdir / "report_normal.json"
        work = self.tempdir / "work_normal"
        argv = self._base_argv(out, work) + [
            "--solver-cmd", self.solver_cmd, "--json", str(json_out)]
        code, stdout, stderr = self._invoke(argv)
        self.assertEqual(code, 0, msg=stderr)
        self.assertTrue(out.is_file())
        self.assertIn("报告已写入", stdout)
        self.assertNotIn("Traceback", stderr)

        text = out.read_text(encoding="utf-8")
        headers = [line for line in text.splitlines() if line.startswith("## ")]
        self.assertEqual(headers, ReportTest.SECTIONS)
        self.assertNotIn("/home/", text)

        payload = json.loads(json_out.read_text(encoding="utf-8"))
        self.assertEqual(len(payload["combinations"]), 4)
        self.assertAlmostEqual(payload["delta_vpe_rmse_mm"],
                               payload["combinations"][0]["vpe_rmse_mm"]
                               - payload["combinations"][1]["vpe_rmse_mm"],
                               delta=1e-5)
        self.assertNotIn("NaN", json_out.read_text(encoding="utf-8"))

    def test_dry_run_writes_report(self):
        out = self.tempdir / "report_dry.md"
        work = self.tempdir / "work_dry"
        code, stdout, stderr = self._invoke(
            self._base_argv(out, work) + ["--dry-run"])
        self.assertEqual(code, 0, msg=stderr)
        self.assertTrue(out.is_file())
        self.assertIn("dry-run", stdout)
        self.assertIn("未执行求解", out.read_text(encoding="utf-8"))
        self.assertEqual(len(list((work / "new_calib").glob("*.pb"))), 4)

    def test_missing_pb_dir_returns_two(self):
        out = self.tempdir / "report_missing.md"
        work = self.tempdir / "work_missing"
        argv = self._base_argv(out, work)
        argv[argv.index("--pb-dir") + 1] = str(self.tempdir / "no_such_dir")
        code, _, stderr = self._invoke(argv)
        self.assertEqual(code, 2)
        self.assertIn("pb 目录不存在", stderr)
        self.assertNotIn("Traceback", stderr)

    def test_missing_session_returns_two(self):
        out = self.tempdir / "report_missing_session.md"
        work = self.tempdir / "work_missing_session"
        argv = self._base_argv(out, work)
        argv[argv.index("--session") + 1] = str(self.tempdir / "no_session.json")
        code, _, stderr = self._invoke(argv)
        self.assertEqual(code, 2)
        self.assertIn("会话文件不存在", stderr)
        self.assertNotIn("Traceback", stderr)

    def test_solver_failure_returns_two_without_traceback(self):
        out = self.tempdir / "report_solver_error.md"
        work = self.tempdir / "work_solver_error"
        code, _, stderr = self._invoke(
            self._base_argv(out, work)
            + ["--solver-cmd", "python3 -c \"raise SystemExit(9)\""])
        self.assertEqual(code, 2)
        self.assertIn("9", stderr)
        self.assertNotIn("Traceback", stderr)
        self.assertFalse(out.is_file())

    def test_bad_calib_ids_returns_two(self):
        out = self.tempdir / "report_bad_ids.md"
        work = self.tempdir / "work_bad_ids"
        argv = self._base_argv(out, work)
        argv[argv.index("--calib-ids") + 1] = "1,x,3"
        code, _, stderr = self._invoke(argv)
        self.assertEqual(code, 2)
        self.assertNotIn("Traceback", stderr)


if __name__ == "__main__":  # pragma: no cover - 直接运行本文件
    unittest.main()
