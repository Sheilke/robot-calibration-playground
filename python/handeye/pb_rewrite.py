# -*- coding: utf-8 -*-
"""用 FK 重算的位姿改写 ``.pb`` 的 ``robot_pt``，产出可直接喂给 C++ 求解器的目录。

背景
----
公司手眼求解器（ROS2 action ``calibration/eye_on_hand_calibration``）只认
「目录里的一堆 ``.pb``」，每个 ``.pb`` 是 ``calibration.ArucoRecordingResult``：
``single_statics.robot_pt``（机器人笛卡尔位姿）+ ``single_statics.board_statics``
（相机系靶球点）。它按固定点搜索解 ``T_bh · X · p_cam ≈ P_fix``，**不知道关节角
也不知道 DH**。

因此「换一套 DH 离线重算」的前提是：拿到每个采样点的关节角 ``q``，用
:class:`handeye.dh.RobotKinematics` 的 FK 算出 4×4 位姿，再用本模块把它写回
``robot_pt``（位置 mm + 欧拉角 deg，XYZ 内旋，见 :mod:`handeye.pose_encoder`），
其余字段（``success`` / ``board_statics`` / ``output_image_path``）原样保留。
求解器代码一行不改。

pose_type 语义
--------------
驱动回传的 ``robot_pt`` 可能是 ``Pose.base`` 也可能是 ``Pose.cart``，本模块
**默认沿用原分支**（只在需要时用 ``pose_type`` 显式切换）；``cart`` 上的
``normal`` / ``weld_direction`` / ``external_axis`` 在同分支改写时保持不变，
切换分支时则随旧分支一起丢弃（那些字段在 ``PoseBase`` 上不存在）。

delta 口径
----------
``position_delta_mm`` 是平移向量差的模；``rotation_delta_deg`` 是相对转角，
用 ``atan2`` 形式计算（接近 0 时比 :func:`handeye.se3.rotation_angle_between`
的 ``acos`` 更稳，后者在 <1e-5 度时会被浮点下限吃掉；较大角度两者一致）。
原 ``robot_pt`` 不是 ``base``/``cart``（unset/axis/quat）时无法读出笛卡尔
位姿作对比，delta 记 ``0.0``。

CLI
---
::

    PYTHONPATH=python python3 -m handeye.pb_rewrite SRC OUT \\
        --urdf data/configs/ch2000_origin.urdf --dh-override dh.json \\
        --q-json q.json [--pose-type base|cart] [--proto-root PATH] \\
        [--dry-run] [--report report.json]

``--q-json`` 形如 ``{"samples": [{"file": "xxx.pb", "q_deg": [6 个]}, ...]}``。
报告（stdout + 可选 ``--report``）字段同 :func:`rewrite_directory`，另加
``urdf`` / ``dh_label`` / ``q_source``。退出码：成功 ``0``、参数或数据错误 ``2``。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from handeye.dh import DhOverride, RobotKinematics
from handeye.pose_encoder import matrix_to_pose_fields, pose_fields_to_matrix
from handeye.proto_codec import (
    ProtoUnavailableError,
    pose_position_orientation,
    pose_type_of,
    read_recording,
    set_pose_position_orientation,
    write_recording,
)
from handeye.se3 import validate_rigid

#: 允许显式指定的 ``pose_type``（axis/quat 不带笛卡尔位姿）。
POSE_TYPES = ("base", "cart")

#: 判断「数值是否真的变了」的容差（mm / deg）。
_VALUE_EPS = 1e-12


def _as_matrix(value, name: str = "matrix") -> np.ndarray:
    """任意 4×4 输入 -> 校验过的 ``np.ndarray``，不合法抛 ``ValueError``。"""
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 无法转为 4x4 浮点数组：{error}") from error
    try:
        validate_rigid(matrix)
    except ValueError as error:
        raise ValueError(f"{name} 不是合法的刚体齐次矩阵：{error}") from error
    return matrix


def rotation_delta_deg(first, second) -> float:
    """两个 3×3 旋转的相对转角（deg），``atan2`` 形式（接近 0 时仍准确）。

    与 :func:`handeye.se3.rotation_angle_between` 数学等价，但后者用
    ``acos``，角度 < 1e-5 度时会被浮点下限吃掉（实测恒为 ~1.7e-6 度），
    无法回答「两条链路是否完全一致」。

    Args:
        first: 3×3 旋转矩阵。
        second: 3×3 旋转矩阵。

    Returns:
        相对转角（deg，``[0, 180]``）。
    """
    relative = np.asarray(first, dtype=float).T @ np.asarray(second, dtype=float)
    sin_part = 0.5 * float(np.linalg.norm([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ]))
    cos_part = 0.5 * (float(np.trace(relative)) - 1.0)
    return math.degrees(math.atan2(sin_part, cos_part))


def _read_cartesian(pose):
    """读出 ``Pose`` 的 ``(position, orientation, 4×4)``；非 base/cart 返回 ``None``。"""
    if pose_type_of(pose) not in POSE_TYPES:
        return None
    position, orientation = pose_position_orientation(pose)
    return position, orientation, pose_fields_to_matrix(position, orientation)


def rewrite_recording(message, matrices_by_index, *, proto_root=None,
                      pose_type=None) -> dict:
    """就地改写一条 recording 的 ``robot_pt``（其它字段不动）。

    Args:
        message: ``calibration.ArucoRecordingResult`` 消息对象。
        matrices_by_index: 新的 4×4 位姿（mm）；``None`` 表示不改。
        proto_root: 预留参数：消息对象已构造，改写本身不需要 proto 绑定；
            保留是为了与 :func:`rewrite_directory` / CLI 的签名一致。
        pose_type: ``None`` = 沿用原 ``robot_pt`` 的分支；也可显式给
            ``'base'`` / ``'cart'``（原为 ``unset`` 时必须给，否则无法决定写哪个
            oneof 分支）。

    Returns:
        ``{"changed", "pose_type", "position_delta_mm", "rotation_delta_deg"}``：
        ``changed`` 表示位姿数值或分支真的变了；delta 为与改写前的对比（原
        ``robot_pt`` 不是 base/cart 时记 ``0.0``）。

    Raises:
        ValueError: ``matrices_by_index`` 不是合法 4×4、``pose_type`` 非法、
            或原分支未设置且没有显式 ``pose_type``。
    """
    pose = message.single_statics.robot_pt
    current = pose_type_of(pose)

    if pose_type is None:
        target = current
    else:
        target = str(pose_type)
        if target not in POSE_TYPES:
            raise ValueError(
                f"pose_type 只支持 {POSE_TYPES}，当前 {target!r}")

    if matrices_by_index is None:
        return {"changed": False, "pose_type": current,
                "position_delta_mm": 0.0, "rotation_delta_deg": 0.0}

    matrix = _as_matrix(matrices_by_index)
    if target not in POSE_TYPES:
        raise ValueError(
            "robot_pt 的 pose_type 未设置，无法决定写 base 还是 cart；"
            "请显式指定 pose_type='base' 或 pose_type='cart'")

    new_position, new_orientation = matrix_to_pose_fields(matrix)

    previous = _read_cartesian(pose)
    if previous is None:
        position_delta = 0.0
        rotation_delta = 0.0
    else:
        old_position, _, old_matrix = previous
        position_delta = float(np.linalg.norm(
            np.asarray(new_position, dtype=float) - np.asarray(old_position, dtype=float)))
        rotation_delta = rotation_delta_deg(old_matrix[:3, :3], matrix[:3, :3])

    if target != current:
        # 切换 oneof 分支：旧分支的附加字段（normal/external_axis 等）不再适用
        pose.Clear()
        carrier = getattr(pose, target)
        carrier.position.data[:] = new_position
        carrier.orientation.data[:] = new_orientation
    # 同分支改写走公共 setter：保留 normal / weld_direction / external_axis
    set_pose_position_orientation(pose, new_position, new_orientation)

    changed = (target != current
               or position_delta > _VALUE_EPS
               or rotation_delta > _VALUE_EPS)
    return {
        "changed": bool(changed),
        "pose_type": target,
        "position_delta_mm": position_delta,
        "rotation_delta_deg": rotation_delta,
    }


def _matrix_map(matrices, files) -> dict:
    """把 ``matrices``（list / dict）整理成 ``{文件名: 4×4 或 None}``。

    * ``dict``：键按文件名匹配（``Path(key).name``）；键在源目录中不存在 -> ``ValueError``；
      目录里有文件但 dict 未给出 -> 该文件被跳过（返回的映射里没有它）。
    * 序列：长度必须与 ``.pb`` 文件数一致，否则 ``ValueError``。
    * 元素为 ``None`` -> 该文件不做改写。
    """
    names = [path.name for path in files]

    if isinstance(matrices, dict):
        mapping = {}
        unknown = []
        for key, value in matrices.items():
            name = Path(str(key)).name
            if name not in names:
                unknown.append(str(key))
                continue
            mapping[name] = None if value is None else _as_matrix(value, f"矩阵[{name}]")
        if unknown:
            raise ValueError(
                f"matrices 中这些文件在源目录里不存在：{unknown}（源目录：{names}）")
        return mapping

    if isinstance(matrices, np.ndarray):
        if matrices.ndim == 3:
            matrices = list(matrices)
        elif matrices.ndim == 2:
            raise ValueError(
                "matrices 需为「矩阵序列」或「{文件名: 4x4}」，收到单个 4x4；"
                "请用 [matrix] 或 {文件名: matrix}")

    sequence = list(matrices)
    if len(sequence) != len(names):
        raise ValueError(
            f"矩阵数量 {len(sequence)} 与 .pb 文件数 {len(names)} 不匹配（按文件名排序）")
    return {
        name: (None if value is None else _as_matrix(value, f"矩阵[{name}]"))
        for name, value in zip(names, sequence)
    }


def rewrite_directory(source_dir, output_dir, matrices, *, proto_root=None,
                      pattern="*.pb", dry_run=False, pose_type=None) -> dict:
    """批量改写一个目录里的 ``.pb``，产出新目录（供 C++ 求解器直接读取）。

    先读完全部文件、算完全部结果，再统一写盘：中途遇到损坏文件会直接抛错，
    不会留下「写了一半」的输出目录。``dry_run=True`` 时完全不创建输出目录。

    Args:
        source_dir: 源目录（内含 ``*.pb``）。
        output_dir: 输出目录（``dry_run`` 时不创建）。
        matrices: ``list[np.ndarray]``（与按文件名排序后的 ``.pb`` 一一对应，
            元素可为 ``None`` 表示跳过该文件）或 ``dict{文件名: 4x4}``。
        proto_root: 传给 :func:`handeye.proto_codec.read_recording` 的 proto
            根目录（``None`` = 环境变量/默认路径）。
        pattern: 文件通配，默认 ``*.pb``。
        dry_run: 只计算差异、不写文件。
        pose_type: ``None`` = 沿用每条记录原有的分支；也可强制 ``'base'`` /
            ``'cart'``。

    Returns:
        ``{"count", "changed", "skipped", "per_file", "max_position_delta_mm",
        "max_rotation_delta_deg", "skipped_files"}``。``count`` 是匹配到的
        ``.pb`` 总数，``changed`` 是真正变化（含分支切换）的文件数，
        ``per_file`` 每项 ``{"file","pose_type","position_delta_mm",
        "rotation_delta_deg","changed"}``，``skipped_files`` 每项
        ``{"file","reason"}``。

    Raises:
        ValueError: 源目录不存在、矩阵数量/文件名不匹配、矩阵不是合法 4×4、
            某个 ``.pb`` 解析失败（消息里带文件名）。
        ProtoUnavailableError: proto 绑定不可用（protoc/protobuf 缺失）。
    """
    source = Path(source_dir).expanduser()
    if not source.is_dir():
        raise ValueError(f"源目录不存在：{source}")
    output = Path(output_dir).expanduser()

    files = sorted((path for path in source.glob(pattern) if path.is_file()),
                   key=lambda path: path.name)
    matrix_map = _matrix_map(matrices, files)

    per_file: list[dict] = []
    skipped: list[dict] = []
    pending: list[tuple] = []

    for path in files:
        matrix = matrix_map.get(path.name)
        if matrix is None:
            skipped.append({
                "file": path.name,
                "reason": "未提供对应位姿矩阵（关节角缺失或显式 None）",
            })
            continue
        message = read_recording(path, proto_root=proto_root)
        info = rewrite_recording(message, matrix, proto_root=proto_root,
                                 pose_type=pose_type)
        pending.append((path, message))
        per_file.append({
            "file": path.name,
            "pose_type": info["pose_type"],
            "position_delta_mm": info["position_delta_mm"],
            "rotation_delta_deg": info["rotation_delta_deg"],
            "changed": info["changed"],
        })

    if not dry_run and pending:
        output.mkdir(parents=True, exist_ok=True)
        for path, message in pending:
            write_recording(message, output / path.name)

    positions = [entry["position_delta_mm"] for entry in per_file]
    rotations = [entry["rotation_delta_deg"] for entry in per_file]
    return {
        "count": len(files),
        "changed": sum(1 for entry in per_file if entry["changed"]),
        "skipped": len(skipped),
        "per_file": per_file,
        "max_position_delta_mm": max(positions) if positions else 0.0,
        "max_rotation_delta_deg": max(rotations) if rotations else 0.0,
        "skipped_files": skipped,
    }


def compare_recorded_vs_fk(message, matrix, *, position_tolerance_mm=1e-6,
                           rotation_tolerance_deg=1e-6) -> dict:
    """把记录里的 ``robot_pt`` 与给定 ``T_bh`` 对比（校验 DH/约定/单位是否自洽）。

    典型用途：用旧 DH 重算位姿后与现场记录的控制器位姿对比——差异应该只有
    控制器 FK 与 URDF 模型的舍入误差；如果差出几十毫米或几十度，说明欧拉
    约定（内旋/外旋）、单位（mm/m）或 DH 参数有出入。

    Args:
        message: ``calibration.ArucoRecordingResult`` 消息对象。
        matrix: 待对比的 4×4 位姿（mm）。
        position_tolerance_mm: 判定 ``consistent`` 的位置容差，默认 1e-6 mm。
        rotation_tolerance_deg: 判定 ``consistent`` 的旋转容差，默认 1e-6 度。

    Returns:
        dict：``pose_type``、``recorded_position``、``recorded_orientation``、
        ``target_position``、``target_orientation``、``position_delta_mm``、
        ``rotation_delta_deg``、``consistent``（两项 delta 都在容差内）。

    Raises:
        ValueError: ``robot_pt`` 不是 base/cart，或 ``matrix`` 不是合法 4×4。
    """
    pose = message.single_statics.robot_pt
    kind = pose_type_of(pose)
    position, orientation = pose_position_orientation(pose)
    recorded = pose_fields_to_matrix(position, orientation)
    target = _as_matrix(matrix)
    target_position, target_orientation = matrix_to_pose_fields(target)

    position_delta = float(np.linalg.norm(target[:3, 3] - recorded[:3, 3]))
    rotation_delta = rotation_delta_deg(recorded[:3, :3], target[:3, :3])
    return {
        "pose_type": kind,
        "recorded_position": position,
        "recorded_orientation": orientation,
        "target_position": target_position,
        "target_orientation": target_orientation,
        "position_delta_mm": position_delta,
        "rotation_delta_deg": rotation_delta,
        "consistent": bool(position_delta <= position_tolerance_mm
                           and rotation_delta <= rotation_tolerance_deg),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    """构造 ``python -m handeye.pb_rewrite`` 的参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m handeye.pb_rewrite",
        description="用「关节角 + DH」重算机器人位姿并改写 .pb 的 robot_pt，"
                    "产出可直接喂给 C++ eye_on_hand_calibration 的目录。")
    parser.add_argument("source_dir", metavar="SYS_SOURCE_DIR", help="源目录（内含 *.pb）")
    parser.add_argument("output_dir", metavar="OUTPUT_DIR", help="输出目录（dry-run 时不创建）")
    parser.add_argument("--urdf", required=True, help="基准 URDF（反解基准 DH）")
    dh_group = parser.add_mutually_exclusive_group(required=True)
    dh_group.add_argument("--dh-override", help="DH 覆盖 JSON（新 DH）")
    dh_group.add_argument("--dh-base", action="store_true",
                          help="直接用 URDF 反解出的 DH（旧 DH = 基准）")
    parser.add_argument("--q-json", required=True,
                        help='关节角 JSON：{"samples":[{"file":"x.pb","q_deg":[6 个]}]}')
    parser.add_argument("--pose-type", choices=POSE_TYPES, default=None,
                        help="强制 robot_pt 的分支；默认沿用每条记录原有的分支")
    parser.add_argument("--proto-root", default=None,
                        help="公司 arc-common proto 根目录（默认环境变量 "
                             "ARC_COMMON_PROTO_ROOT 或内置默认路径）")
    parser.add_argument("--dry-run", action="store_true",
                        help="只统计差异，不写任何文件")
    parser.add_argument("--report", default=None, help="把 JSON 报告另存为文件")
    return parser


def _load_samples(path: Path) -> dict:
    """读 ``--q-json``，返回 ``{文件名: 关节角列表}``；格式错误抛 ``ValueError``。"""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"无法读取 --q-json 文件 {path}：{error}") from error
    except ValueError as error:
        raise ValueError(f"--q-json 文件 {path} 不是合法 JSON：{error}") from error

    if not isinstance(payload, dict):
        raise ValueError(f"--q-json 顶层须为对象，当前 {type(payload).__name__}")
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError('--q-json 需包含非空的 "samples" 数组')

    mapping: dict[str, list[float]] = {}
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"samples[{index}] 须为对象，当前 {type(sample).__name__}")
        name = sample.get("file")
        if not isinstance(name, str) or not name:
            raise ValueError(f"samples[{index}] 缺少非空的 'file' 字段")
        joints = sample.get("q_deg")
        if not isinstance(joints, list):
            raise ValueError(f"samples[{index}]（{name}）缺少 'q_deg' 数组")
        try:
            values = [float(value) for value in joints]
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"samples[{index}]（{name}）的 q_deg 含非数值：{error}") from error
        if len(values) != 6:
            raise ValueError(
                f"samples[{index}]（{name}）的 q_deg 需 6 个分量，当前 {len(values)} 个")
        mapping[Path(name).name] = values
    return mapping


def main(argv=None) -> int:
    """CLI 主入口，返回退出码（0 成功 / 2 参数或数据错误）。"""
    parser = build_parser()
    args = parser.parse_args(argv)

    source = Path(args.source_dir).expanduser()
    if not source.is_dir():
        print(f"错误：源目录不存在：{source}", file=sys.stderr)
        return 2
    urdf = Path(args.urdf).expanduser()
    if not urdf.is_file():
        print(f"错误：URDF 不存在：{urdf}", file=sys.stderr)
        return 2

    try:
        if args.dh_base:
            override = DhOverride.from_base(urdf)
        else:
            override_path = Path(args.dh_override).expanduser()
            if not override_path.is_file():
                print(f"错误：DH 覆盖文件不存在：{override_path}", file=sys.stderr)
                return 2
            override = DhOverride.from_json(override_path)
        model = RobotKinematics(urdf, override, label=override.name)
        q_source = Path(args.q_json).expanduser()
        samples = _load_samples(q_source)
        matrices = {name: model.fk(joints) for name, joints in samples.items()}
        result = rewrite_directory(
            source, args.output_dir, matrices, proto_root=args.proto_root,
            dry_run=args.dry_run, pose_type=args.pose_type)
    except (ValueError, OSError, ProtoUnavailableError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2

    for entry in result["skipped_files"]:
        print(f"警告：{entry['file']} 已跳过（{entry['reason']}）", file=sys.stderr)

    report = dict(result)
    report["urdf"] = str(urdf)
    report["dh_label"] = model.label
    report["q_source"] = str(q_source)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    if args.report:
        try:
            report_path = Path(args.report).expanduser()
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(text + "\n", encoding="utf-8")
        except OSError as error:
            print(f"错误：写报告失败 {args.report}：{error}", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI 入口
    sys.exit(main())
