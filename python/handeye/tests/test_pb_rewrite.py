# -*- coding: utf-8 -*-
"""handeye.pb_rewrite 单元测试：改写 robot_pt、pose_type 语义、目录批量与 CLI。

依赖公司 proto（运行时由 protoc 生成），CI 缺失时整组跳过。
"""

import contextlib
import io
import json
import math
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye import pb_rewrite  # noqa: F401 - 确认模块可导入（CLI 入口依赖）
from handeye.dh import DhOverride, RobotKinematics
from handeye.pb_rewrite import (
    compare_recorded_vs_fk,
    main,
    rewrite_directory,
    rewrite_recording,
    rotation_delta_deg,
)
from handeye.pose_encoder import matrix_to_pose_fields, pose_fields_to_matrix
from handeye.proto_codec import (
    load_pb_modules,
    pose_position_orientation,
    pose_type_of,
    read_recording,
    write_recording,
)
from handeye.se3 import make_pose, rotation_angle_between
from robot_model.nexbot_ik import euler_to_rot

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "ch2000_origin.urdf"

JOINTS_A = [10.0, 20.0, -30.0, 15.0, 45.0, -60.0]
JOINTS_B = [-25.0, 35.0, -40.0, -10.0, 60.0, 80.0]


def _bindings_available() -> bool:
    """protoc + 公司 proto 是否都可用（决定整组测试能否跑）。"""
    if shutil.which("protoc") is None:
        return False
    try:
        load_pb_modules()
    except Exception:
        return False
    return True


BINDINGS_AVAILABLE = _bindings_available()

SKIP_REASON = ("公司 proto / protoc 不可用（需要 arc-common proto_msgs 与 "
               "protobuf-compiler），跳过 .pb 改写测试")


def _recording_mode(pb2, pose_type="base", matrix=None):
    """造一条 recording；``matrix=None`` 时用一个固定位姿。"""
    if matrix is None:
        matrix = make_pose([100.0, -50.0, 900.0],
                           euler_to_rot([170.0, -3.0, 85.0], order="XYZ",
                                        extrinsic=False))
    position, orientation = matrix_to_pose_fields(matrix)
    message = pb2.ArucoRecordingResult()
    message.success = True
    message.output_image_path = f"/data/aruco/{pose_type}_{position[0]:.1f}.png"
    pose = message.single_statics.robot_pt
    carrier = getattr(pose, pose_type)
    carrier.position.data[:] = position
    carrier.orientation.data[:] = orientation
    for index, point_xyz in enumerate([(10.0, -2.0, 480.0), (14.0, 3.0, 470.0)]):
        point = message.single_statics.board_statics.board_pts.add()
        point.id = index + 1
        point.x, point.y, point.z = point_xyz
    return message, matrix


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_REASON)
class RewriteRecordingTest(unittest.TestCase):
    """单条改写：位姿等于 FK、pose_type 语义、其它字段不动。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="pb_rewrite_"))
        cls.cache_dir = cls.tempdir / "gen"
        cls.pb2, _ = load_pb_modules(cache_dir=cls.cache_dir)
        cls.model = RobotKinematics(URDF, label="base")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def _roundtrip(self, message, name="case.pb"):
        path = self.tempdir / name
        write_recording(message, path)
        return read_recording(path, cache_dir=self.cache_dir)

    def test_rewrite_matches_fk(self):
        target = self.model.fk(JOINTS_A)
        message, _ = _recording_mode(self.pb2, "base")
        info = rewrite_recording(message, target)

        self.assertTrue(info["changed"])
        self.assertEqual(info["pose_type"], "base")
        loaded = self._roundtrip(message)

        position, orientation = pose_position_orientation(
            loaded.single_statics.robot_pt)
        matrix = pose_fields_to_matrix(position, orientation)
        self.assertLess(float(np.linalg.norm(matrix[:3, 3] - target[:3, 3])), 1e-6)
        self.assertLess(rotation_delta_deg(matrix[:3, :3], target[:3, :3]), 1e-6)
        # 其它字段必须原样保留
        self.assertTrue(loaded.success)
        self.assertEqual(loaded.output_image_path, message.output_image_path)
        self.assertEqual(len(loaded.single_statics.board_statics.board_pts), 2)
        self.assertEqual(loaded, message)

    def test_rewrite_reports_delta_against_previous_pose(self):
        original = make_pose([0.0, 0.0, 0.0], np.eye(3))
        message, _ = _recording_mode(self.pb2, "base", original)
        target = make_pose([30.0, 40.0, 0.0],
                           euler_to_rot([0.0, 0.0, 5.0], order="XYZ",
                                        extrinsic=False))
        info = rewrite_recording(message, target)
        self.assertAlmostEqual(info["position_delta_mm"], 50.0, places=9)
        self.assertAlmostEqual(info["rotation_delta_deg"], 5.0, places=9)

    def test_same_pose_is_not_reported_as_changed(self):
        target = self.model.fk(JOINTS_A)
        message, _ = _recording_mode(self.pb2, "base", target)
        info = rewrite_recording(message, target)
        self.assertFalse(info["changed"])
        self.assertAlmostEqual(info["position_delta_mm"], 0.0, places=12)
        self.assertLess(info["rotation_delta_deg"], 1e-9)

    def test_none_matrix_is_noop(self):
        message, _ = _recording_mode(self.pb2, "cart")
        before = message.SerializeToString()
        info = rewrite_recording(message, None)
        self.assertFalse(info["changed"])
        self.assertEqual(info["pose_type"], "cart")
        self.assertEqual(message.SerializeToString(), before)

    def test_pose_type_is_preserved_by_default(self):
        target = self.model.fk(JOINTS_B)
        for kind in ("base", "cart"):
            message, _ = _recording_mode(self.pb2, kind)
            info = rewrite_recording(message, target)
            self.assertEqual(info["pose_type"], kind)
            self.assertEqual(pose_type_of(message.single_statics.robot_pt), kind)
            position, orientation = pose_position_orientation(
                message.single_statics.robot_pt)
            matrix = pose_fields_to_matrix(position, orientation)
            self.assertLess(float(np.linalg.norm(matrix[:3, 3] - target[:3, 3])), 1e-6)

    def test_cart_keeps_normal_and_external_axis(self):
        message, _ = _recording_mode(self.pb2, "cart")
        message.single_statics.robot_pt.cart.normal.data.extend([0.0, 0.0, 1.0])
        message.single_statics.robot_pt.cart.external_axis.data.extend([120.0])
        target = self.model.fk(JOINTS_B)
        rewrite_recording(message, target)
        pose = message.single_statics.robot_pt
        self.assertEqual(pose_type_of(pose), "cart")
        self.assertEqual(list(pose.cart.normal.data), [0.0, 0.0, 1.0])
        self.assertEqual(list(pose.cart.external_axis.data), [120.0])

    def test_explicit_pose_type_switches_base_to_cart(self):
        target = self.model.fk(JOINTS_A)
        message, _ = _recording_mode(self.pb2, "base")
        info = rewrite_recording(message, target, pose_type="cart")
        self.assertEqual(info["pose_type"], "cart")
        pose = message.single_statics.robot_pt
        self.assertEqual(pose_type_of(pose), "cart")
        self.assertTrue(pose.HasField("cart"))
        self.assertFalse(pose.HasField("base"))
        position, orientation = pose_position_orientation(pose)
        matrix = pose_fields_to_matrix(position, orientation)
        self.assertLess(float(np.linalg.norm(matrix[:3, 3] - target[:3, 3])), 1e-6)

    def test_unset_pose_type_requires_explicit_choice(self):
        message = self.pb2.ArucoRecordingResult()
        message.single_statics.board_statics.board_pts.add().id = 1
        message.single_statics.robot_pt.Clear()
        with self.assertRaises(ValueError):
            rewrite_recording(message, self.model.fk(JOINTS_A))
        info = rewrite_recording(message, self.model.fk(JOINTS_A), pose_type="base")
        self.assertEqual(info["pose_type"], "base")
        self.assertEqual(pose_type_of(message.single_statics.robot_pt), "base")

    def test_invalid_pose_type_rejected(self):
        message, _ = _recording_mode(self.pb2, "base")
        with self.assertRaises(ValueError):
            rewrite_recording(message, self.model.fk(JOINTS_A), pose_type="quat")

    def test_invalid_matrix_rejected(self):
        message, _ = _recording_mode(self.pb2, "base")
        with self.assertRaises(ValueError):
            rewrite_recording(message, np.eye(3))
        bad = self.model.fk(JOINTS_A).copy()
        bad[0, 0] = float("nan")
        with self.assertRaises(ValueError):
            rewrite_recording(message, bad)


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_REASON)
class CompareRecordedVsFkTest(unittest.TestCase):
    """compare_recorded_vs_fk：同一位姿 0 差异、不同位姿给出正确 delta。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="pb_compare_"))
        cls.pb2, _ = load_pb_modules(cache_dir=cls.tempdir / "gen")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def test_identical_pose_has_zero_difference(self):
        matrix = make_pose([12.5, -3.25, 900.0],
                           euler_to_rot([179.0, 1.0, -87.0], order="XYZ",
                                        extrinsic=False))
        message, _ = _recording_mode(self.pb2, "base", matrix)
        report = compare_recorded_vs_fk(message, matrix)
        self.assertEqual(report["pose_type"], "base")
        self.assertEqual(report["position_delta_mm"], 0.0)
        self.assertLess(report["rotation_delta_deg"], 1e-9)
        self.assertTrue(report["consistent"])

    def test_different_pose_reports_correct_delta(self):
        recorded = make_pose([100.0, 0.0, 500.0], np.eye(3))
        target = make_pose([100.0, 30.0, 540.0],
                           euler_to_rot([0.0, 0.0, 10.0], order="XYZ",
                                        extrinsic=False))
        message, _ = _recording_mode(self.pb2, "base", recorded)
        report = compare_recorded_vs_fk(message, target)
        self.assertAlmostEqual(report["position_delta_mm"], 50.0, places=9)
        self.assertAlmostEqual(report["rotation_delta_deg"], 10.0, places=9)
        self.assertFalse(report["consistent"])
        # 与 se3 的角度度量在非零角度上一致（acos 版本在 >1e-5 度才可靠）
        self.assertAlmostEqual(
            report["rotation_delta_deg"],
            rotation_angle_between(recorded[:3, :3], target[:3, :3]), places=9)
        self.assertEqual(report["target_position"],
                         matrix_to_pose_fields(target)[0])

    def test_cart_branch_supported(self):
        matrix = make_pose([1.0, 2.0, 3.0], np.eye(3))
        message, _ = _recording_mode(self.pb2, "cart", matrix)
        report = compare_recorded_vs_fk(message, matrix)
        self.assertEqual(report["pose_type"], "cart")

    def test_non_cartesian_pose_rejected(self):
        message = self.pb2.ArucoRecordingResult()
        message.single_statics.robot_pt.axis.angle.data.extend([0.0] * 6)
        with self.assertRaises(ValueError):
            compare_recorded_vs_fk(message, np.eye(4))


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_REASON)
class RewriteDirectoryTest(unittest.TestCase):
    """目录批量：数量校验、dry-run、跳过、损坏文件。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="pb_rewrite_dir_"))
        cls.cache_dir = cls.tempdir / "gen"
        cls.pb2, _ = load_pb_modules(cache_dir=cls.cache_dir)
        cls.model = RobotKinematics(URDF, label="base")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def _source_dir(self, name: str, count: int = 2) -> Path:
        directory = self.tempdir / name
        directory.mkdir(parents=True, exist_ok=True)
        for index in range(count):
            message, _ = _recording_mode(self.pb2, "base")
            write_recording(message, directory / f"sample_{index:02d}.pb")
        return directory

    def test_count_mismatch_raises(self):
        source = self._source_dir("mismatch", count=2)
        matrices = [self.model.fk(JOINTS_A)]
        with self.assertRaises(ValueError) as context:
            rewrite_directory(source, self.tempdir / "out_mismatch", matrices)
        self.assertIn("数量", str(context.exception))

    def test_unknown_dict_key_raises(self):
        source = self._source_dir("unknown_key", count=1)
        matrices = {"not_there.pb": self.model.fk(JOINTS_A)}
        with self.assertRaises(ValueError):
            rewrite_directory(source, self.tempdir / "out_unknown", matrices)

    def test_dry_run_writes_nothing(self):
        source = self._source_dir("dry_run", count=2)
        output = self.tempdir / "out_dry_run"
        matrices = [self.model.fk(JOINTS_A), self.model.fk(JOINTS_B)]
        report = rewrite_directory(source, output, matrices, dry_run=True)
        self.assertFalse(output.exists())
        self.assertEqual(report["count"], 2)
        self.assertEqual(report["changed"], 2)
        self.assertEqual(report["skipped"], 0)
        self.assertGreater(report["max_position_delta_mm"], 0.0)
        self.assertEqual([entry["file"] for entry in report["per_file"]],
                         ["sample_00.pb", "sample_01.pb"])

    def test_rewrite_all_files_and_skip_missing_matrix(self):
        source = self._source_dir("partial", count=2)
        output = self.tempdir / "out_partial"
        matrices = {"sample_01.pb": self.model.fk(JOINTS_B)}
        report = rewrite_directory(source, output, matrices)
        self.assertEqual(report["count"], 2)
        self.assertEqual(report["changed"], 1)
        self.assertEqual(report["skipped"], 1)
        self.assertEqual(report["skipped_files"][0]["file"], "sample_00.pb")
        self.assertTrue((output / "sample_01.pb").is_file())
        self.assertFalse((output / "sample_00.pb").exists())

        target = self.model.fk(JOINTS_B)
        loaded = read_recording(output / "sample_01.pb", cache_dir=self.cache_dir)
        position, orientation = pose_position_orientation(
            loaded.single_statics.robot_pt)
        matrix = pose_fields_to_matrix(position, orientation)
        self.assertLess(float(np.linalg.norm(matrix[:3, 3] - target[:3, 3])), 1e-6)
        self.assertLess(rotation_delta_deg(matrix[:3, :3], target[:3, :3]), 1e-6)

    def test_corrupted_file_raises_and_writes_nothing(self):
        source = self._source_dir("corrupted", count=2)
        (source / "zz_broken.pb").write_bytes(b"\xff\xff\xff\xff")
        output = self.tempdir / "out_corrupted"
        matrices = [self.model.fk(JOINTS_A), self.model.fk(JOINTS_B),
                    self.model.fk(JOINTS_A)]
        with self.assertRaises(ValueError) as context:
            rewrite_directory(source, output, matrices, proto_root=None)
        self.assertIn("zz_broken.pb", str(context.exception))
        # 两阶段：解析全部成功后才写盘，出错时不留半成品
        self.assertFalse(output.exists())

    def test_missing_source_dir_raises(self):
        with self.assertRaises(ValueError):
            rewrite_directory(self.tempdir / "no_such_dir",
                              self.tempdir / "out_missing", [])


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_REASON)
class CliTest(unittest.TestCase):
    """``python -m handeye.pb_rewrite``：报告字段、跳过提示、退出码。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="pb_cli_"))
        cls.cache_dir = cls.tempdir / "gen"
        cls.pb2, _ = load_pb_modules(cache_dir=cls.cache_dir)
        cls.model = RobotKinematics(URDF, label="base")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def _fixture(self, name: str) -> tuple:
        """源目录 + q.json（只覆盖第一个文件，用来触发 skipped 提示）。"""
        source = self.tempdir / f"src_{name}"
        source.mkdir(parents=True, exist_ok=True)
        for index in range(2):
            message, _ = _recording_mode(self.pb2, "base")
            write_recording(message, source / f"pose_{index:02d}.pb")
        q_path = self.tempdir / f"q_{name}.json"
        q_path.write_text(json.dumps({
            "samples": [{"file": "pose_00.pb", "q_deg": JOINTS_A}]}), encoding="utf-8")
        return source, q_path

    def _run(self, argv):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = main(argv)
        return code, stdout.getvalue(), stderr.getvalue()

    def test_cli_writes_report_and_skips_missing_joint_samples(self):
        source, q_path = self._fixture("ok")
        output = self.tempdir / "out_ok"
        report_path = self.tempdir / "report_ok.json"
        code, stdout, stderr = self._run([
            str(source), str(output),
            "--urdf", str(URDF), "--dh-base",
            "--q-json", str(q_path),
            "--report", str(report_path),
        ])
        self.assertEqual(code, 0)
        report = json.loads(stdout)
        self.assertEqual(report["count"], 2)
        self.assertEqual(report["changed"], 1)
        self.assertEqual(report["skipped"], 1)
        self.assertEqual(report["dh_label"], "base")
        self.assertEqual(report["urdf"], str(URDF))
        self.assertEqual(report["q_source"], str(q_path))
        self.assertIn("pose_01.pb", stderr)  # 跳过必须显式提示
        self.assertTrue((output / "pose_00.pb").is_file())
        self.assertEqual(json.loads(report_path.read_text(encoding="utf-8")), report)

        target = self.model.fk(JOINTS_A)
        loaded = read_recording(output / "pose_00.pb", cache_dir=self.cache_dir)
        position, orientation = pose_position_orientation(
            loaded.single_statics.robot_pt)
        matrix = pose_fields_to_matrix(position, orientation)
        self.assertLess(float(np.linalg.norm(matrix[:3, 3] - target[:3, 3])), 1e-6)

    def test_cli_dry_run_creates_no_output(self):
        source, q_path = self._fixture("dry")
        output = self.tempdir / "out_cli_dry"
        code, stdout, _ = self._run([
            str(source), str(output), "--urdf", str(URDF), "--dh-base",
            "--q-json", str(q_path), "--dry-run",
        ])
        self.assertEqual(code, 0)
        self.assertFalse(output.exists())
        self.assertEqual(json.loads(stdout)["count"], 2)

    def test_cli_with_dh_override_json(self):
        source, q_path = self._fixture("override")
        override = DhOverride.from_base(URDF, name="cli_override")
        override.theta_offset[0] += 0.5  # 注入已知扰动
        override_path = self.tempdir / "dh_override.json"
        override_path.write_text(
            json.dumps(override.to_json_dict()), encoding="utf-8")
        output = self.tempdir / "out_override"
        code, stdout, _ = self._run([
            str(source), str(output), "--urdf", str(URDF),
            "--dh-override", str(override_path), "--q-json", str(q_path),
        ])
        self.assertEqual(code, 0)
        report = json.loads(stdout)
        self.assertEqual(report["dh_label"], "cli_override")
        self.assertEqual(report["changed"], 1)

    def test_cli_returns_2_on_bad_inputs(self):
        source, q_path = self._fixture("bad")
        cases = [
            # 源目录不存在
            [str(self.tempdir / "nope"), str(self.tempdir / "out_x"),
             "--urdf", str(URDF), "--dh-base", "--q-json", str(q_path)],
            # q-json 不存在
            [str(source), str(self.tempdir / "out_y"),
             "--urdf", str(URDF), "--dh-base",
             "--q-json", str(self.tempdir / "missing.json")],
            # URDF 不存在
            [str(source), str(self.tempdir / "out_z"),
             "--urdf", str(self.tempdir / "missing.urdf"), "--dh-base",
             "--q-json", str(q_path)],
        ]
        bad_joints = self.tempdir / "q_bad.json"
        bad_joints.write_text(json.dumps({"samples": [
            {"file": "pose_00.pb", "q_deg": [1.0, 2.0]}]}), encoding="utf-8")
        cases.append([str(source), str(self.tempdir / "out_w"),
                      "--urdf", str(URDF), "--dh-base", "--q-json", str(bad_joints)])

        for argv in cases:
            code, stdout, stderr = self._run(argv)
            self.assertEqual(code, 2, f"argv={argv}")
            self.assertEqual(stdout, "")
            self.assertIn("错误", stderr)

    def test_cli_requires_dh_source(self):
        with self.assertRaises(SystemExit) as context:
            with contextlib.redirect_stderr(io.StringIO()):
                main(["a", "b", "--urdf", str(URDF), "--q-json", "q.json"])
        self.assertEqual(context.exception.code, 2)


class RotationDeltaTest(unittest.TestCase):
    """rotation_delta_deg 与 se3.rotation_angle_between 在非零角度上一致。"""

    def test_matches_se3_for_large_angles(self):
        rng = np.random.default_rng(5)
        for _ in range(50):
            a = euler_to_rot(rng.uniform(-180.0, 180.0, 3), order="XYZ",
                             extrinsic=False)
            b = euler_to_rot(rng.uniform(-180.0, 180.0, 3), order="XYZ",
                             extrinsic=False)
            self.assertAlmostEqual(rotation_delta_deg(a, b),
                                   rotation_angle_between(a, b), places=6)

    def test_zero_for_identical(self):
        rotation = euler_to_rot([10.0, 20.0, 30.0], order="XYZ", extrinsic=False)
        self.assertEqual(rotation_delta_deg(rotation, rotation), 0.0)
        self.assertAlmostEqual(rotation_delta_deg(rotation, rotation), 0.0,
                               delta=math.radians(1e-9))


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
