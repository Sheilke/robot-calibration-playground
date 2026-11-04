# -*- coding: utf-8 -*-
"""handeye.proto_codec 单元测试：路径解析、protoc 生成、.pb 读写、Pose 语义。

proto 根目录来自公司 arc-common（不在仓库内），CI 上缺失时整组测试跳过。
"""

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from handeye import proto_codec
from handeye.proto_codec import (
    DEFAULT_PROTO_ROOT,
    ENV_PROTO_ROOT,
    ProtoUnavailableError,
    ensure_bindings,
    load_pb_modules,
    pose_position_orientation,
    pose_type_of,
    read_recording,
    resolve_proto_root,
    set_pose_position_orientation,
    write_recording,
)


def _default_root() -> Path:
    return Path(DEFAULT_PROTO_ROOT).expanduser()


def _protos_present() -> bool:
    """公司 proto 源文件是否齐全（决定整组测试能否跑）。"""
    root = _default_root()
    return all((root / relative).is_file() for relative in proto_codec.PROTO_SOURCES)


def _protoc_present() -> bool:
    return shutil.which("protoc") is not None


def _bindings_available() -> bool:
    """真实试一次 ``load_pb_modules()``（尊重环境变量解析结果）。"""
    try:
        load_pb_modules()
    except Exception:  # ProtoUnavailableError / ImportError 等
        return False
    return True


PROTOS_AVAILABLE = _protos_present()
PROTOC_AVAILABLE = _protoc_present()
BINDINGS_AVAILABLE = _bindings_available()

SKIP_PROTOS = (
    f"公司 proto 根目录不可用（{_default_root()} 缺少 calibration/common proto），跳过")
SKIP_PROTOC = "未找到 protoc（protobuf 编译器），跳过 .pb 生成相关测试"
SKIP_BINDINGS = (
    "无法生成/导入公司 proto 绑定（缺 proto 源文件、protoc 或 protobuf 运行时），"
    f"跳过；期望的默认根目录：{_default_root()}")


def _make_recording(calibration_pb2):
    """构造一条内容齐全的 recording（base 分支 + 两个靶球点）。"""
    message = calibration_pb2.ArucoRecordingResult()
    message.success = True
    message.output_image_path = "/tmp/images/frame_0001.png"
    pose = message.single_statics.robot_pt
    pose.base.position.data.extend([101.5, -22.25, 900.125])
    pose.base.orientation.data.extend([179.5, -1.25, 87.0])
    for index, (x, y, z) in enumerate([(12.5, -3.25, 410.75), (18.0, 4.5, 405.0)]):
        point = message.single_statics.board_statics.board_pts.add()
        point.id = index + 1
        point.x = x
        point.y = y
        point.z = z
    return message


class ResolveProtoRootTest(unittest.TestCase):
    """路径解析优先级：显式参数 -> 环境变量 -> 默认路径。"""

    def setUp(self):
        self.tempdir = Path(tempfile.mkdtemp(prefix="proto_root_"))
        self.addCleanup(shutil.rmtree, self.tempdir, True)

    def test_default_root_when_present(self):
        if not _default_root().is_dir():
            self.skipTest(f"默认 proto 根目录不存在：{_default_root()}，跳过")
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(ENV_PROTO_ROOT, None)
            self.assertEqual(resolve_proto_root(), _default_root().resolve())

    def test_explicit_argument_wins_over_environment(self):
        with mock.patch.dict(os.environ, {ENV_PROTO_ROOT: str(self.tempdir)}):
            self.assertEqual(resolve_proto_root(self.tempdir), self.tempdir.resolve())

    def test_environment_override(self):
        other = Path(tempfile.mkdtemp(prefix="proto_root_env_"))
        self.addCleanup(shutil.rmtree, other, True)
        with mock.patch.dict(os.environ, {ENV_PROTO_ROOT: str(other)}):
            self.assertEqual(resolve_proto_root(), other.resolve())

    def test_missing_explicit_path_raises(self):
        missing = self.tempdir / "does_not_exist"
        with self.assertRaises(ProtoUnavailableError) as context:
            resolve_proto_root(missing)
        self.assertIn(str(missing), str(context.exception))

    def test_missing_environment_path_raises(self):
        missing = self.tempdir / "nope"
        with mock.patch.dict(os.environ, {ENV_PROTO_ROOT: str(missing)}):
            with self.assertRaises(ProtoUnavailableError):
                resolve_proto_root()

    def test_missing_default_path_raises(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(ENV_PROTO_ROOT, None)
            with mock.patch.object(proto_codec, "DEFAULT_PROTO_ROOT",
                                   str(self.tempdir / "nowhere")):
                with self.assertRaises(ProtoUnavailableError) as context:
                    resolve_proto_root()
        self.assertIn("默认路径", str(context.exception))


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_BINDINGS)
class BindingsTest(unittest.TestCase):
    """protoc 生成 + 导入 + 消息类存在性。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="proto_gen_"))
        cls.cache_dir = cls.tempdir / "build" / "proto_gen"
        cls.calibration_pb2, cls.common_pb2 = load_pb_modules(cache_dir=cls.cache_dir)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def test_ensure_bindings_creates_expected_layout(self):
        cache = ensure_bindings(cache_dir=self.cache_dir)
        self.assertEqual(cache, self.cache_dir.resolve())
        self.assertTrue((cache / "proto" / "calibration" / "calibration_pb2.py").is_file())
        self.assertTrue((cache / "proto" / "common" / "common_pb2.py").is_file())
        self.assertTrue((cache / proto_codec.MANIFEST_NAME).is_file())
        # 生成物不得落进仓库受控区
        self.assertNotIn("python", str(cache))

    def test_generated_modules_expose_messages(self):
        for name in ("ArucoRecordingResult", "SingleCalibStatics", "BoardStatics",
                     "ArucoBoardPoint"):
            self.assertTrue(hasattr(self.calibration_pb2, name), name)
        for name in ("Pose", "PoseCart", "PoseBase", "PoseAxis", "PoseQuat", "VectorXd"):
            self.assertTrue(hasattr(self.common_pb2, name), name)
        # 跨包引用：SingleCalibStatics.robot_pt 的类型确实是 arc_s.common.Pose
        pose_field = self.calibration_pb2.SingleCalibStatics.DESCRIPTOR.fields_by_name[
            "robot_pt"]
        self.assertEqual(pose_field.message_type.full_name, "arc_s.common.Pose")
        self.assertEqual(self.common_pb2.Pose.DESCRIPTOR.full_name, "arc_s.common.Pose")

    def test_second_call_reuses_cache(self):
        cache = ensure_bindings(cache_dir=self.cache_dir)
        generated = cache / "proto" / "calibration" / "calibration_pb2.py"
        first_mtime = generated.stat().st_mtime_ns
        ensure_bindings(cache_dir=self.cache_dir)
        self.assertEqual(generated.stat().st_mtime_ns, first_mtime)

    def test_force_regenerates(self):
        cache = ensure_bindings(cache_dir=self.cache_dir, force=True)
        self.assertTrue((cache / "proto" / "calibration" / "calibration_pb2.py").is_file())
        # 强制重生成后仍能正常导入
        calibration_pb2, common_pb2 = load_pb_modules(cache_dir=self.cache_dir, force=True)
        self.assertTrue(hasattr(calibration_pb2, "ArucoRecordingResult"))
        self.assertTrue(hasattr(common_pb2, "Pose"))


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_BINDINGS)
class RecordingIoTests(unittest.TestCase):
    """.pb 读写与错误处理。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="proto_io_"))
        cls.cache_dir = cls.tempdir / "gen"
        cls.calibration_pb2, _ = load_pb_modules(cache_dir=cls.cache_dir)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def test_roundtrip_preserves_all_fields(self):
        original = _make_recording(self.calibration_pb2)
        path = self.tempdir / "nested" / "dir" / "sample.pb"
        write_recording(original, path)
        self.assertTrue(path.is_file())

        loaded = read_recording(path)
        self.assertTrue(loaded.success)
        self.assertEqual(loaded.output_image_path, original.output_image_path)
        self.assertEqual(pose_type_of(loaded.single_statics.robot_pt), "base")
        position, orientation = pose_position_orientation(loaded.single_statics.robot_pt)
        self.assertEqual(position, [101.5, -22.25, 900.125])
        self.assertEqual(orientation, [179.5, -1.25, 87.0])
        points = list(loaded.single_statics.board_statics.board_pts)
        self.assertEqual([point.id for point in points], [1, 2])
        self.assertEqual([point.x for point in points], [12.5, 18.0])
        self.assertEqual([point.y for point in points], [-3.25, 4.5])
        self.assertEqual([point.z for point in points], [410.75, 405.0])
        self.assertEqual(loaded, original)

    def test_read_rejects_non_protobuf_file(self):
        for name, payload in (
                # 纯文本：protobuf 会把它当作未知字段乱解，甚至只发 RuntimeWarning
                ("text.pb", b"definitely not a protobuf payload"),
                ("garbage.pb", b"\xff\xff\xff\xff"),
                ("empty.pb", b"")):
            broken = self.tempdir / name
            broken.write_bytes(payload)
            with self.assertRaises(ValueError, msg=name) as context:
                read_recording(broken)
            self.assertIn(str(broken), str(context.exception))

    def test_read_missing_file_raises_oserror(self):
        with self.assertRaises(OSError):
            read_recording(self.tempdir / "absent.pb")


@unittest.skipUnless(BINDINGS_AVAILABLE, SKIP_BINDINGS)
class PoseSemanticsTest(unittest.TestCase):
    """pose_type_of / pose_position_orientation / set_pose_position_orientation。"""

    @classmethod
    def setUpClass(cls):
        cls.tempdir = Path(tempfile.mkdtemp(prefix="pose_sem_"))
        cls.cache_dir = cls.tempdir / "gen"
        _, cls.common_pb2 = load_pb_modules(cache_dir=cls.cache_dir)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tempdir, ignore_errors=True)

    def _pose(self):
        return self.common_pb2.Pose()

    def test_pose_type_of(self):
        pose = self._pose()
        self.assertEqual(pose_type_of(pose), "unset")
        pose.cart.position.data.extend([1.0, 2.0, 3.0])
        self.assertEqual(pose_type_of(pose), "cart")
        other = self._pose()
        other.base.position.data.extend([1.0, 2.0, 3.0])
        self.assertEqual(pose_type_of(other), "base")

    def test_pose_position_orientation_supports_base_and_cart(self):
        pose = self._pose()
        pose.base.position.data.extend([1.0, 2.0, 3.0])
        pose.base.orientation.data.extend([4.0, 5.0, 6.0])
        self.assertEqual(pose_position_orientation(pose), ([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]))

        cart = self._pose()
        cart.cart.position.data.extend([7.0, 8.0, 9.0])
        cart.cart.orientation.data.extend([10.0, 11.0, 12.0])
        self.assertEqual(pose_position_orientation(cart),
                         ([7.0, 8.0, 9.0], [10.0, 11.0, 12.0]))

    def test_pose_position_orientation_rejects_other_branches(self):
        for branch in ("axis", "quat", "unset"):
            pose = self._pose()
            if branch == "axis":
                pose.axis.angle.data.extend([0.0] * 6)
            elif branch == "quat":
                pose.quat.quaternion.data.extend([0.0, 0.0, 0.0, 1.0])
            with self.assertRaises(ValueError):
                pose_position_orientation(pose)

    def test_set_keeps_pose_type_and_other_fields(self):
        pose = self._pose()
        pose.cart.position.data.extend([1.0, 2.0, 3.0])
        pose.cart.orientation.data.extend([4.0, 5.0, 6.0])
        pose.cart.normal.data.extend([0.0, 0.0, 1.0])
        pose.cart.weld_direction.data.extend([1.0, 0.0, 0.0])
        pose.cart.external_axis.data.extend([12.5, 0.0])

        set_pose_position_orientation(pose, [100.0, 200.0, 300.0], [10.0, 20.0, 30.0])

        self.assertEqual(pose_type_of(pose), "cart")
        self.assertEqual(pose_position_orientation(pose),
                         ([100.0, 200.0, 300.0], [10.0, 20.0, 30.0]))
        self.assertEqual(list(pose.cart.normal.data), [0.0, 0.0, 1.0])
        self.assertEqual(list(pose.cart.weld_direction.data), [1.0, 0.0, 0.0])
        self.assertEqual(list(pose.cart.external_axis.data), [12.5, 0.0])

    def test_set_keeps_base_branch(self):
        pose = self._pose()
        pose.base.position.data.extend([1.0, 2.0, 3.0])
        pose.base.orientation.data.extend([4.0, 5.0, 6.0])
        set_pose_position_orientation(pose, [7.0, 8.0, 9.0], [10.0, 11.0, 12.0])
        self.assertEqual(pose_type_of(pose), "base")
        self.assertEqual(pose_position_orientation(pose),
                         ([7.0, 8.0, 9.0], [10.0, 11.0, 12.0]))

    def test_set_rejects_wrong_shapes_and_unset(self):
        pose = self._pose()
        with self.assertRaises(ValueError):
            set_pose_position_orientation(pose, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0])
        cart = self._pose()
        cart.cart.position.data.extend([1.0, 2.0, 3.0])
        with self.assertRaises(ValueError):
            set_pose_position_orientation(cart, [1.0, 2.0], [0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            set_pose_position_orientation(cart, [1.0, 2.0, 3.0], [0.0, float("nan"), 0.0])


class UnavailableProtocTest(unittest.TestCase):
    """protoc 缺失时必须给出带安装指引的 ProtoUnavailableError（不需要真 proto）。"""

    def setUp(self):
        self.tempdir = Path(tempfile.mkdtemp(prefix="proto_missing_"))
        self.addCleanup(shutil.rmtree, self.tempdir, True)

    def test_missing_protoc_raises_with_guidance(self):
        if not _protos_present():
            self.skipTest(SKIP_PROTOS)
        # 显式指定默认根目录，避免环境变量把失败原因换掉；缓存目录故意为空
        with mock.patch.object(proto_codec.shutil, "which", return_value=None):
            with self.assertRaises(ProtoUnavailableError) as context:
                ensure_bindings(proto_root=_default_root(), cache_dir=self.tempdir / "gen")
        message = str(context.exception)
        self.assertIn("protoc", message)
        self.assertIn("protobuf-compiler", message)

    def test_missing_proto_sources_raises(self):
        empty_root = self.tempdir / "empty_root"
        empty_root.mkdir()
        with self.assertRaises(ProtoUnavailableError) as context:
            ensure_bindings(proto_root=empty_root, cache_dir=self.tempdir / "gen")
        self.assertIn("缺少必要文件", str(context.exception))

    def test_missing_protobuf_runtime_raises_with_guidance(self):
        with mock.patch.dict(sys.modules, {"google.protobuf": None}):
            with self.assertRaises(ProtoUnavailableError) as context:
                load_pb_modules(cache_dir=self.tempdir / "gen")
        message = str(context.exception)
        self.assertIn("protobuf", message)
        self.assertIn("pip install", message)
