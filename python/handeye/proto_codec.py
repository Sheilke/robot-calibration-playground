# -*- coding: utf-8 -*-
"""公司 calibration/common proto 的动态编译与 ``.pb`` 读写。

背景
----
现场采集结果是 ``calibration.ArucoRecordingResult``（protobuf 二进制 ``.pb``），
每条记录含「机器人笛卡尔位姿 ``robot_pt`` + 相机系靶球点 ``board_statics``」。
离线重算需要把 ``robot_pt`` 换成「关节角 + DH」重算的位姿后再喂给公司 C++
求解器（ROS2 action ``calibration/eye_on_hand_calibration``），因此必须先能
读写这种 ``.pb``。

公司 proto 属于内部资料，**不随本仓库分发**，只能在运行时用系统 ``protoc``
从公司 arc-common 源码树动态生成到本仓库的 ``build/proto_gen``（已被
``.gitignore`` 的 ``build/`` 规则覆盖）。本模块只做三件事：

1. 定位 proto 根目录（显式参数 -> 环境变量 -> 默认公司路径）；
2. 调 ``protoc --python_out`` 生成 ``*_pb2.py`` 并按 mtime+size 缓存；
3. 读写 ``.pb``、按 ``Pose`` 的 oneof 语义取/写 position+orientation。

单位契约
--------
``position`` 为 **mm**、``orientation`` 为 **deg**（Nexbot 控制链路，见
``common.proto`` 的 ``PoseCart/PoseBase`` 注释）；欧拉约定是 ``R = Rx·Ry·Rz``
（``kXyzIntrinsic``），矩阵↔字段换算见 :mod:`handeye.pose_encoder`。

生成布局
--------
``protoc -I <root> --python_out=<cache>`` 会在 ``<cache>`` 下按 proto 的相对
路径产出 ``proto/calibration/calibration_pb2.py`` 与
``proto/common/common_pb2.py``；生成代码内部用 ``from proto.common import
common_pb2`` 互相引用，所以导入方式是「把 ``<cache>`` 插到 ``sys.path``，再
``import proto.calibration.calibration_pb2``」。

依赖缺失
--------
``protobuf`` 运行时或 ``protoc`` 缺失时不会在 import 阶段崩，而是在调用
:func:`ensure_bindings` / :func:`load_pb_modules` 时抛
:class:`ProtoUnavailableError`，消息里给出安装指引。
"""

from __future__ import annotations

import importlib
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

#: 公司 arc-common proto 根目录的默认位置（相对 ``$HOME``）。
DEFAULT_PROTO_ROOT = "~/ws/3srobotics/arc-common/src/proto_msgs"

#: 覆盖 proto 根目录的环境变量名（优先于 :data:`DEFAULT_PROTO_ROOT`）。
ENV_PROTO_ROOT = "ARC_COMMON_PROTO_ROOT"

#: 需要的 proto 源文件（相对 proto 根目录）。
PROTO_SOURCES = (
    "proto/calibration/calibration.proto",
    "proto/common/common.proto",
)

#: 生成物默认落盘位置（仓库根下的 ``build/proto_gen``，被 ``build/`` 规则忽略）。
DEFAULT_CACHE_DIR = Path(__file__).resolve().parents[2] / "build" / "proto_gen"

#: 缓存清单文件名（记录 proto 源文件的 mtime/size，用于失效判断）。
MANIFEST_NAME = ".proto_gen.json"

#: 生成模块的完整模块名（供 ``sys.path`` 方案导入）。
_CALIBRATION_MODULE = "proto.calibration.calibration_pb2"
_COMMON_MODULE = "proto.common.common_pb2"
_GENERATED_MODULES = (_CALIBRATION_MODULE, _COMMON_MODULE)

_PROTOC_TIMEOUT_SEC = 120

#: ``load_pb_modules`` 的进程内缓存：{缓存目录: (生成物指纹, (cal, common))}
_MODULE_CACHE: dict[str, tuple] = {}


class ProtoUnavailableError(RuntimeError):
    """找不到 proto 根目录 / protoc 失败 / 生成的模块导入失败。"""


# ---------------------------------------------------------------------------
# 路径与校验
# ---------------------------------------------------------------------------


def _abs_path(value) -> Path:
    """``str`` / ``Path`` -> 展开 ``~`` 并解析为绝对路径（不做存在性检查）。"""
    return Path(value).expanduser().resolve()


def resolve_proto_root(explicit=None) -> Path:
    """定位公司 proto 根目录（含 ``proto/calibration/calibration.proto`` 的目录）。

    查找顺序：**显式参数** -> 环境变量 :data:`ENV_PROTO_ROOT` ->
    :data:`DEFAULT_PROTO_ROOT`。

    Args:
        explicit: 调用方显式给出的 proto 根目录；``None`` 表示按环境变量/默认值查找。

    Returns:
        绝对路径（``pathlib.Path``）。

    Raises:
        ProtoUnavailableError: 目标不是已存在的目录。消息会说明该路径来自
            显式参数 / 环境变量 / 默认值，便于排查。
    """
    if explicit is not None:
        candidate, origin = _abs_path(explicit), "显式参数 proto_root"
    elif os.environ.get(ENV_PROTO_ROOT):
        candidate = _abs_path(os.environ[ENV_PROTO_ROOT])
        origin = f"环境变量 {ENV_PROTO_ROOT}"
    else:
        candidate = _abs_path(DEFAULT_PROTO_ROOT)
        origin = f"默认路径 {DEFAULT_PROTO_ROOT}"

    if not candidate.is_dir():
        raise ProtoUnavailableError(
            f"proto 根目录不存在（来源：{origin}）：{candidate}\n"
            f"请用 --proto-root 显式指定，或设置环境变量 {ENV_PROTO_ROOT} 指向 "
            "公司 arc-common 的 proto_msgs 目录（其下应有 proto/calibration/"
            "calibration.proto）。")
    return candidate


def _source_paths(root: Path) -> list[tuple[str, Path]]:
    """检查 :data:`PROTO_SOURCES` 是否齐全，返回 ``[(相对路径, 绝对路径)]``。"""
    found: list[tuple[str, Path]] = []
    missing: list[str] = []
    for relative in PROTO_SOURCES:
        candidate = root / relative
        if candidate.is_file():
            found.append((relative, candidate))
        else:
            missing.append(str(candidate))
    if missing:
        raise ProtoUnavailableError(
            f"proto 根目录 {root} 缺少必要文件：\n  " + "\n  ".join(missing))
    return found


def _cache_dir_path(cache_dir) -> Path:
    """缓存目录：显式参数或 :data:`DEFAULT_CACHE_DIR`。"""
    return DEFAULT_CACHE_DIR if cache_dir is None else _abs_path(cache_dir)


def _generated_path(cache: Path, relative: str) -> Path:
    """proto 相对路径 -> 生成的 ``*_pb2.py`` 路径（保留 ``proto/`` 子目录布局）。"""
    source = Path(relative)
    return cache / source.parent / f"{source.stem}_pb2.py"


def _protoc_version(protoc: str) -> str:
    """``protoc --version`` 的输出（失败时返回 ``"unknown"``，不抛错）。"""
    try:
        completed = subprocess.run(
            [protoc, "--version"], capture_output=True, text=True,
            timeout=_PROTOC_TIMEOUT_SEC)
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    if completed.returncode != 0:
        return "unknown"
    return completed.stdout.strip() or "unknown"


def _signature(sources, protoc: str, version: str) -> dict:
    """缓存签名：proto 文件 mtime+size 以及 protoc 标识。"""
    return {
        "protoc": protoc,
        "protoc_version": version,
        "sources": _sources_signature(sources),
    }


def _sources_signature(sources) -> dict:
    """proto 源文件的 mtime+size 签名（与 protoc 无关的那一半）。"""
    files = {}
    for relative, path in sources:
        stat = path.stat()
        files[relative] = {"size": int(stat.st_size), "mtime_ns": int(stat.st_mtime_ns)}
    return files


def _read_manifest(cache: Path):
    """读取缓存清单；缺失或损坏返回 ``None``（触发重生成）。"""
    try:
        return json.loads((cache / MANIFEST_NAME).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _write_manifest(cache: Path, signature: dict) -> None:
    """写缓存清单；失败只影响下次缓存命中，不影响本次结果。"""
    try:
        (cache / MANIFEST_NAME).write_text(
            json.dumps(signature, ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8")
    except OSError:
        pass


# ---------------------------------------------------------------------------
# 生成与导入
# ---------------------------------------------------------------------------


def ensure_bindings(proto_root=None, cache_dir=None, force=False) -> Path:
    """调用 ``protoc`` 生成 ``calibration_pb2`` / ``common_pb2`` 并返回缓存目录。

    生成的 ``.py`` 落在 ``<cache_dir>/proto/calibration/`` 与
    ``<cache_dir>/proto/common/``（protoc 按 proto 的相对路径分目录输出）。
    缓存以「proto 源文件 mtime + size + protoc 版本」为签名写入
    ``<cache_dir>/.proto_gen.json``：签名一致且生成物齐全时直接复用，不重复
    调用 protoc。若缓存已完整且源文件未变，**没有 protoc 也能用**（方便
    CI/离线环境直接消费预生成的 ``build/proto_gen``）。

    Args:
        proto_root: proto 根目录；``None`` 时走
            :func:`resolve_proto_root` 的「环境变量 -> 默认值」顺序。
        cache_dir: 生成目录；``None`` 时为仓库内 ``build/proto_gen``
            （已被 ``.gitignore`` 的 ``build/`` 规则覆盖）。
        force: ``True`` 时忽略缓存强制重生成。

    Returns:
        生成目录的绝对路径（``pathlib.Path``），可直接插入 ``sys.path``。

    Raises:
        ProtoUnavailableError: proto 根目录/源文件缺失、找不到 ``protoc``、
            protoc 返回码非 0、或生成的 ``.py`` 缺失。错误消息包含实际执行的
            命令与 stderr。
    """
    root = resolve_proto_root(proto_root)
    sources = _source_paths(root)
    cache = _cache_dir_path(cache_dir)
    cache.mkdir(parents=True, exist_ok=True)

    outputs = [_generated_path(cache, relative) for relative, _ in sources]
    outputs_ready = all(path.is_file() for path in outputs)
    manifest = _read_manifest(cache)
    protoc = shutil.which("protoc")
    if protoc is None:
        # 没有 protoc 时仍允许复用已生成好的缓存（源文件未变且生成物齐全），
        # 方便 CI/离线环境直接使用预生成的 build/proto_gen。
        if not force and outputs_ready and manifest is not None \
                and manifest.get("sources") == _sources_signature(sources):
            return cache
        raise ProtoUnavailableError(
            "未找到 protoc（protobuf 编译器），且缓存目录中没有可用的生成物。\n"
            "请安装后重试，例如 Ubuntu：\n"
            "  sudo apt install -y protobuf-compiler\n"
            "或用环境变量 PATH 指向已有的 protoc。")

    version = _protoc_version(protoc)
    signature = _signature(sources, protoc, version)
    if not force and manifest == signature and outputs_ready:
        return cache

    command = [
        protoc,
        "-I", str(root),
        f"--python_out={cache}",
        *[relative for relative, _ in sources],
    ]
    try:
        completed = subprocess.run(
            command, cwd=str(root), capture_output=True, text=True,
            timeout=_PROTOC_TIMEOUT_SEC)
    except FileNotFoundError as error:
        raise ProtoUnavailableError(
            f"protoc 无法执行：{protoc}（{error}）\n命令：{shlex.join(command)}") from error
    except subprocess.TimeoutExpired as error:
        raise ProtoUnavailableError(
            f"protoc 超时（>{_PROTOC_TIMEOUT_SEC}s）\n命令：{shlex.join(command)}") from error

    if completed.returncode != 0:
        raise ProtoUnavailableError(
            f"protoc 失败（返回码 {completed.returncode}）\n"
            f"命令：{shlex.join(command)}\n"
            f"工作目录：{root}\n"
            f"stdout：{completed.stdout.strip()}\n"
            f"stderr：{completed.stderr.strip()}")

    missing = [str(path) for path in outputs if not path.is_file()]
    if missing:
        raise ProtoUnavailableError(
            "protoc 返回成功但未生成预期文件：\n  " + "\n  ".join(missing) +
            f"\n命令：{shlex.join(command)}\n"
            f"stdout：{completed.stdout.strip()}\n"
            f"stderr：{completed.stderr.strip()}")

    _write_manifest(cache, signature)
    return cache


def _generated_stamp(cache: Path) -> tuple:
    """生成物的 (路径, mtime_ns, size) 指纹，用于失效进程内模块缓存。"""
    stamp = []
    for module in _GENERATED_MODULES:
        relative = module.replace(".", "/")
        path = cache / f"{relative}.py"
        try:
            stat = path.stat()
            stamp.append((str(path), int(stat.st_mtime_ns), int(stat.st_size)))
        except OSError:
            stamp.append((str(path), None, None))
    return tuple(stamp)


def _import_generated(cache: Path, module_name: str):
    """按 ``<cache>`` 为前缀导入生成模块，失败抛出可读的 ProtoUnavailableError。"""
    try:
        return importlib.import_module(module_name)
    except Exception as error:  # ImportError / TypeError(描述符冲突) 等
        raise ProtoUnavailableError(
            f"导入生成模块 {module_name} 失败（缓存目录 {cache}）。\n"
            f"原因：{type(error).__name__}: {error}\n"
            "可尝试清理该缓存目录后重试，或检查 protobuf 运行时版本是否与 "
            "protoc 3.21 生成的代码兼容（pip install 'protobuf>=4.21,<5'）。"
        ) from error


def load_pb_modules(proto_root=None, cache_dir=None, force=False) -> tuple:
    """生成并导入 proto 绑定，返回 ``(calibration_pb2, common_pb2)``。

    内部先调用 :func:`ensure_bindings`，再把缓存目录插到 ``sys.path`` 首位，
    以 ``proto.calibration.calibration_pb2`` / ``proto.common.common_pb2``
    导入（protoc 按 proto 相对路径分目录输出，因此必须带 ``proto.`` 前缀）。
    同一缓存目录、同一批生成物在进程内只导入一次。

    Args:
        proto_root: proto 根目录；``None`` 时按环境变量/默认值查找。
        cache_dir: 生成目录；``None`` 时为 ``build/proto_gen``。
        force: ``True`` 时强制重生成并重新导入。

    Returns:
        ``(calibration_pb2, common_pb2)`` 两个模块对象。

    Raises:
        ProtoUnavailableError: 生成失败（见 :func:`ensure_bindings`）、
            ``protobuf`` 运行时缺失、或生成的模块导入失败。
    """
    try:
        import google.protobuf  # noqa: F401 - 仅探测运行时是否可用
    except ImportError as error:
        raise ProtoUnavailableError(
            "缺少 protobuf 运行时（Python 绑定）。请安装后重试：\n"
            "  python3 -m pip install 'protobuf>=4.21,<5'\n"
            f"原始错误：{error}") from error

    cache = ensure_bindings(proto_root, cache_dir, force)
    key = str(cache)
    stamp = _generated_stamp(cache)
    cached = _MODULE_CACHE.get(key)
    if cached is not None and cached[0] == stamp and not force:
        return cached[1]

    if key not in sys.path:
        sys.path.insert(0, key)

    # 需要（重新）导入：先清掉同名模块，避免拿到上一次缓存目录/旧版本的绑定。
    # 生成物内容未变时 protobuf 允许重复注册同一份描述符，重导入是安全的。
    for name in (*_GENERATED_MODULES, "proto.calibration", "proto.common"):
        sys.modules.pop(name, None)

    calibration_pb2 = _import_generated(cache, _CALIBRATION_MODULE)
    common_pb2 = _import_generated(cache, _COMMON_MODULE)
    _MODULE_CACHE[key] = (stamp, (calibration_pb2, common_pb2))
    return calibration_pb2, common_pb2


# ---------------------------------------------------------------------------
# .pb 读写
# ---------------------------------------------------------------------------


def read_recording(path, *, proto_root=None, cache_dir=None) -> "object":
    """读单个 ``.pb``，返回 ``calibration.ArucoRecordingResult`` 消息对象。

    除了捕获 ``DecodeError``，还把「解析过程产生 RuntimeWarning」与「空文件」
    视为解析失败：protobuf 对某些非 proto 字节（例如意外遇到 end-group tag）
    只发 ``RuntimeWarning: Unexpected end-group tag: Not all data was converted``
    而**不会报错**，真实的采集结果不会出现这种警告。

    Args:
        path: ``.pb`` 文件路径（``str`` / ``Path``）。
        proto_root: 可选的公司 proto 根目录（默认走环境变量/内置默认路径）。
        cache_dir: 可选的生成物缓存目录（默认 ``build/proto_gen``）。

    Returns:
        ``ArucoRecordingResult`` 实例。

    Raises:
        ValueError: 内容不是合法的 ``ArucoRecordingResult`` 二进制（消息里带路径）。
        OSError: 文件不存在或无法读取。
    """
    import warnings

    from google.protobuf.message import DecodeError

    path = Path(path).expanduser()
    calibration_pb2, _ = load_pb_modules(proto_root, cache_dir)
    message = calibration_pb2.ArucoRecordingResult()
    data = path.read_bytes()
    if not data:
        raise ValueError(f"无法解析 {path}：文件为空（0 字节）")
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        try:
            message.ParseFromString(data)
        except DecodeError as error:
            raise ValueError(
                f"无法把 {path} 解析为 calibration.ArucoRecordingResult"
                f"（{len(data)} 字节）：{error}") from error
    warnings_seen = [str(item.message) for item in caught
                     if issubclass(item.category, RuntimeWarning)]
    if warnings_seen:
        raise ValueError(
            f"无法把 {path} 解析为 calibration.ArucoRecordingResult"
            f"（{len(data)} 字节）：{'; '.join(warnings_seen)}")
    return message


def write_recording(message, path) -> None:
    """把消息序列化写回 ``.pb``；父目录不存在时自动创建。

    Args:
        message: ``calibration.ArucoRecordingResult`` 消息对象（就地序列化）。
        path: 目标路径（``str`` / ``Path``）。

    Raises:
        OSError: 目录无法创建或文件无法写入。
    """
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(message.SerializeToString())


# ---------------------------------------------------------------------------
# Pose 语义
# ---------------------------------------------------------------------------


def pose_type_of(pose) -> str:
    """返回 ``Pose`` 的 oneof 类型：``'base'`` / ``'cart'`` / ``'axis'`` / ``'quat'`` / ``'unset'``。

    Args:
        pose: ``arc_s.common.Pose`` 消息对象。

    Returns:
        oneof ``pose_type`` 的当前分支名；未设置时返回 ``'unset'``。
    """
    kind = pose.WhichOneof("pose_type")
    return "unset" if kind is None else str(kind)


def pose_position_orientation(pose) -> tuple:
    """取 ``Pose`` 的 ``(position, orientation)`` 两个列表。

    只支持笛卡尔分支 ``base`` / ``cart``（二者的字段同为 ``position`` +
    ``orientation``，单位 mm / deg）。``axis`` / ``quat`` / 未设置时无法给出
    笛卡尔位姿，抛 ``ValueError``。

    Args:
        pose: ``arc_s.common.Pose`` 消息对象。

    Returns:
        ``([x, y, z] mm, [rx, ry, rz] deg)``，元素为 ``float`` 的新列表。

    Raises:
        ValueError: 当前分支不是 ``base`` / ``cart``。
    """
    kind = pose_type_of(pose)
    if kind not in ("base", "cart"):
        raise ValueError(
            f"Pose 当前为 {kind!r}，只有 base/cart 分支带 position+orientation")
    carrier = getattr(pose, kind)
    return [float(value) for value in carrier.position.data], [
        float(value) for value in carrier.orientation.data]


def set_pose_position_orientation(pose, position, orientation) -> None:
    """就地替换 ``position`` / ``orientation``，**保留 pose_type 与其它字段**。

    只改 ``data`` 内容，不重建消息：``cart`` 分支上的 ``normal`` /
    ``weld_direction`` / ``external_axis`` 以及 oneof 分支本身都保持不变
    （与 C++ 侧「只更新机器人位姿」的语义一致）。

    Args:
        pose: ``arc_s.common.Pose`` 消息对象（就地修改）。
        position: 3 个位置分量（mm）。
        orientation: 3 个欧拉角分量（deg，XYZ 内旋）。

    Raises:
        ValueError: 当前分支不是 ``base`` / ``cart``，或分量不是 3 个有限数。
    """
    kind = pose_type_of(pose)
    if kind not in ("base", "cart"):
        raise ValueError(
            f"Pose 当前为 {kind!r}，不能直接写笛卡尔位姿；"
            "请先确定 base/cart 分支（见 handeye.pb_rewrite.rewrite_recording）")
    values = []
    for name, items in (("position", position), ("orientation", orientation)):
        try:
            floats = [float(value) for value in items]
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} 含无法转为浮点的值：{error}") from error
        if len(floats) != 3:
            raise ValueError(f"{name} 需恰好 3 个值，当前 {len(floats)} 个")
        if not all(math.isfinite(value) for value in floats):
            raise ValueError(f"{name} 含非有限值：{floats}")
        values.append(floats)

    carrier = getattr(pose, kind)
    carrier.position.data[:] = values[0]
    carrier.orientation.data[:] = values[1]
