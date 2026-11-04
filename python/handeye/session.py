# -*- coding: utf-8 -*-
"""会话 JSON 的装载与校验（一次手眼验证实验的全部输入）。

会话结构
--------
::

    {
      "meta": {"robot_urdf": "data/configs/ch2000_origin.urdf",
               "dh_override": null, "group": 4,
               "joint_convention": "model", "units": "mm", "note": "..."},
      "reference": {"p_ball_laser_base": [x, y, z], "delta_p_base": null},
      "handeye_matrix": null | {"matrix16_rowmajor": [16 值], "units": "mm"},
      "samples": [{"pose_id": 1, "q_deg": [6 值], "recorded_pose": null,
                   "frames": [{"target_points": {"7": [x, y, z]}}]}]
    }

约定
----
* 长度单位统一 **mm**（``target_points`` / ``p_ball_laser_base`` /
  ``delta_p_base`` / ``recorded_pose``）；本期只接受 ``meta.units = "mm"``。
* ``meta.joint_convention`` 本期只支持 ``"model"``（``q_deg`` 就是
  :class:`handeye.dh.RobotKinematics` 模型轴角度，非控制器的实际轴角）。
* ``meta.robot_urdf`` / ``meta.dh_override`` 的相对路径按 **仓库根** 解析
  （``python/handeye/session.py`` 向上两级）；显式传入 ``base_dir`` 时先试
  ``base_dir``，再回落到仓库根。
* ``samples[].frames`` 里同一 ``target_id`` 可以重复出现（同一位姿多帧），
  聚合与统计由 :mod:`handeye.vpe` 负责。

所有非法输入统一抛 :class:`SessionError`（``ValueError`` 子类），消息里带
JSON 路径（如 ``samples[3].q_deg 必须为 6 个数，当前 5 个``），便于直接定位。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from handeye.dh import DhOverride, RobotKinematics
from handeye.pose_encoder import pose_fields_to_matrix
from handeye.se3 import load_matrix16, rotation_angle_between, validate_rigid

#: 仓库根：``python/handeye/session.py`` 向上两级。
REPO_ROOT = Path(__file__).resolve().parents[2]

#: 关节数（本项目只支持 6 轴本体）。
JOINT_COUNT = 6

#: 本期支持的长度单位。
SUPPORTED_UNITS = ("mm",)

#: 本期支持的关节角约定。
SUPPORTED_JOINT_CONVENTIONS = ("model",)

#: 手眼矩阵允许的输入单位（装载后统一换算成 mm）。
_MATRIX_UNITS = ("mm", "m")


class SessionError(ValueError):
    """会话 JSON 内容不合法。"""


# ---------------------------------------------------------------------------
# 基础校验工具
# ---------------------------------------------------------------------------


def _as_finite_floats(values, where: str, length: int) -> list:
    """把任意序列转成 ``length`` 个有限浮点，不合法抛 :class:`SessionError`。

    Args:
        values: 待转换的序列（``list`` / ``tuple`` / ``np.ndarray``）。
        where: 报错时使用的字段路径，如 ``"samples[0].q_deg"``。
        length: 期望的元素个数。

    Returns:
        ``list[float]``，长度恰为 ``length``。

    Raises:
        SessionError: 不是序列 / 含无法转浮点的值 / 长度不符 / 含 nan、inf。
    """
    if isinstance(values, (str, bytes, dict)) or not hasattr(values, "__iter__"):
        raise SessionError(
            f"{where} 必须为 {length} 个数的数组，当前 {type(values).__name__}")
    try:
        items = [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise SessionError(f"{where} 含无法转为浮点的值：{error}") from error
    if len(items) != length:
        raise SessionError(f"{where} 必须为 {length} 个数，当前 {len(items)} 个")
    for index, value in enumerate(items):
        if not math.isfinite(value):
            raise SessionError(f"{where}[{index}] 不是有限值：{value!r}")
    return items


def _require_mapping(raw, where: str) -> dict:
    """要求 ``raw`` 是 JSON 对象，返回它；否则抛 :class:`SessionError`。"""
    if not isinstance(raw, dict):
        raise SessionError(f"{where} 必须为对象，当前 {type(raw).__name__}")
    return raw


def _require_key(mapping: dict, key: str, where: str):
    """取必填键，缺失时抛 :class:`SessionError`（而不是 ``KeyError``）。"""
    if key not in mapping:
        raise SessionError(f"{where} 缺少必填字段 {key!r}")
    return mapping[key]


def _as_int(value, where: str) -> int:
    """要求 ``value`` 是整数（拒绝 ``bool`` 与浮点），返回 ``int``。"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise SessionError(f"{where} 必须为整数，当前 {value!r}")
    return int(value)


def _frame_from(raw, where: str) -> "Frame":
    """把 ``Frame`` / ``{"target_points": {...}}`` / ``{id: [x, y, z]}`` 统一成 Frame。"""
    if isinstance(raw, Frame):
        return raw
    if (isinstance(raw, dict) and isinstance(raw.get("target_points"), dict)):
        return Frame(raw["target_points"], where=f"{where}.target_points")
    return Frame(raw, where=f"{where}.target_points")


def _posed_matrix(raw, where: str):
    """把 ``{"position": [3], "orientation": [3]}`` 转成 4x4；``None`` 原样返回。"""
    if raw is None:
        return None
    if isinstance(raw, np.ndarray) or (
            not isinstance(raw, dict) and hasattr(raw, "__len__")):
        try:
            matrix = np.asarray(raw, dtype=float)
        except (TypeError, ValueError) as error:
            raise SessionError(f"{where} 无法转为 4x4 矩阵：{error}") from error
        if matrix.shape != (4, 4):
            raise SessionError(f"{where} 需为 4x4 或 position/orientation 对象，"
                               f"当前形状 {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise SessionError(f"{where} 含非有限值")
        try:
            validate_rigid(matrix)
        except ValueError as error:
            raise SessionError(f"{where} 不是合法刚体矩阵：{error}") from error
        return matrix

    mapping = _require_mapping(raw, where)
    position = _as_finite_floats(
        _require_key(mapping, "position", where), f"{where}.position", 3)
    orientation = _as_finite_floats(
        _require_key(mapping, "orientation", where), f"{where}.orientation", 3)
    try:
        return pose_fields_to_matrix(position, orientation)
    except ValueError as error:
        raise SessionError(f"{where} 不合法：{error}") from error


# ---------------------------------------------------------------------------
# 数据模型
# ---------------------------------------------------------------------------


class Frame:
    """单帧观测：``{target_id(str) -> np.ndarray(3,)}``（相机系，mm）。

    Attributes:
        target_points: ``dict[str, np.ndarray]``，键统一转成字符串（与 JSON
            对象键一致），值为相机系下的 3D 点（mm）。
    """

    def __init__(self, target_points: dict, where: str = "target_points"):
        """校验并保存一帧的靶点。

        Args:
            target_points: ``{id: [x, y, z]}``，至少 1 个点，坐标单位 mm。
            where: 报错时使用的字段路径前缀（内部使用，默认 ``"target_points"``）。

        Raises:
            SessionError: 不是对象 / 空 dict / 坐标不是 3 个有限数 / id 归一化后重复。
        """
        if not isinstance(target_points, dict):
            raise SessionError(
                f"{where} 必须为对象（id -> [x, y, z]），当前 "
                f"{type(target_points).__name__}")
        if not target_points:
            raise SessionError(f"{where} 不能为空，至少需要 1 个靶点")

        normalized: dict = {}
        for key, value in target_points.items():
            target_id = str(key)
            if target_id in normalized:
                raise SessionError(f"{where} 的 id {target_id!r} 重复出现")
            normalized[target_id] = np.asarray(
                _as_finite_floats(value, f"{where}[{target_id!r}]", 3), dtype=float)
        self.target_points = normalized

    def __len__(self) -> int:
        """本帧靶点个数。"""
        return len(self.target_points)

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return f"Frame(ids={sorted(self.target_points)})"


class Sample:
    """单个机器人位姿下的采样（同一 ``q_deg`` 的多帧观测）。

    Attributes:
        pose_id: 位姿编号（整数，会话内唯一）。
        q_deg: 6 个关节角（度，模型轴角度）。
        frames: ``list[Frame]``，至少 1 帧。
        recorded_pose: 控制器回传的 4x4 笛卡尔位姿（mm），没有则为 ``None``。
    """

    def __init__(self, pose_id, q_deg, frames, recorded_pose=None,
                 where: str = "sample"):
        """校验并保存一个位姿的采样。

        Args:
            pose_id: 位姿编号（``int``）。
            q_deg: 6 个关节角（度）。
            frames: 非空序列，元素为 :class:`Frame` 或 ``{id: [x, y, z]}`` 字典。
            recorded_pose: ``None`` / 4x4 矩阵 / ``{"position","orientation"}``。
            where: 报错时使用的字段路径前缀（内部使用）。

        Raises:
            SessionError: 任一分量不合法。关节限位需要运动学模型，由
                :class:`Session` / :func:`session_from_dict` 在构造后检查。
        """
        self.pose_id = _as_int(pose_id, f"{where}.pose_id")
        self.q_deg = _as_finite_floats(q_deg, f"{where}.q_deg", JOINT_COUNT)
        if isinstance(frames, (str, bytes, dict)) or not hasattr(frames, "__iter__"):
            raise SessionError(
                f"{where}.frames 必须为非空数组，当前 {type(frames).__name__}")
        frames = list(frames)
        if not frames:
            raise SessionError(f"{where}.frames 必须为非空数组，当前 0 帧")
        self.frames = [
            _frame_from(frame, f"{where}.frames[{index}]")
            for index, frame in enumerate(frames)
        ]
        self.recorded_pose = _posed_matrix(recorded_pose, f"{where}.recorded_pose")

    def target_ids(self) -> set:
        """该位姿所有帧里出现过的目标 id 并集。"""
        ids: set = set()
        for frame in self.frames:
            ids.update(frame.target_points)
        return ids

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return (f"Sample(pose_id={self.pose_id!r}, frames={len(self.frames)}, "
                f"ids={sorted(self.target_ids())})")


class Session:
    """一次验证实验的全部输入。

    Attributes:
        meta: 会话元信息（原样保存的 ``dict``）。
        reference_point: 激光跟踪仪实测的靶球中心（基座系，mm）。
        delta_p_base: Tool0—靶球固定偏置 ``^B ΔP``（mm），无则为 ``None``。
        handeye_matrix: 手眼矩阵 X（flange←camera，4x4，mm），可为 ``None``。
        samples: ``list[Sample]``。
        kinematics: :class:`handeye.dh.RobotKinematics`（提供 ``fk`` / ``limits``）。
        source_path: 会话 JSON 路径（构造性会话为 ``None``）。
    """

    def __init__(self, meta, reference_point, delta_p_base, handeye_matrix,
                 samples, kinematics, source_path=None):
        """保存已校验的数据；这里再做一次形状/有限性校验。

        Args:
            meta: 元信息字典（可为 ``None``，表示为空）。
            reference_point: 3 个有限数（mm）。
            delta_p_base: ``None`` 或 3 个有限数（mm）。
            handeye_matrix: ``None`` 或 4x4 刚体矩阵（mm）。
            samples: ``list[Sample]``；允许为空（由 :mod:`handeye.vpe` 决定行为）。
            kinematics: 运动学对象，需实现 ``fk(q_deg) -> 4x4``。
            source_path: 会话 JSON 路径。

        Raises:
            SessionError: 形状/数值/重复 ``pose_id``/关节超限不合法。
        """
        if meta is None:
            meta = {}
        self.meta = dict(_require_mapping(meta, "meta"))
        self.reference_point = np.asarray(
            _as_finite_floats(reference_point, "reference_point", 3), dtype=float)
        self.delta_p_base = (
            None if delta_p_base is None
            else np.asarray(_as_finite_floats(delta_p_base, "delta_p_base", 3),
                            dtype=float))
        if handeye_matrix is None:
            self.handeye_matrix = None
        else:
            matrix = np.asarray(handeye_matrix, dtype=float)
            try:
                validate_rigid(matrix)
            except ValueError as error:
                raise SessionError(
                    f"handeye_matrix 不是合法刚体矩阵：{error}") from error
            self.handeye_matrix = matrix
        if isinstance(samples, (str, bytes, dict)) or not hasattr(samples, "__iter__"):
            raise SessionError(
                f"samples 必须为数组，当前 {type(samples).__name__}")
        self.samples = list(samples)
        seen: dict = {}
        for index, sample in enumerate(self.samples):
            if not isinstance(sample, Sample):
                raise SessionError(
                    f"samples[{index}] 必须为 Sample，当前 {type(sample).__name__}")
            if sample.pose_id in seen:
                raise SessionError(
                    f"samples[{index}].pose_id 重复：{sample.pose_id}"
                    f"（已在 samples[{seen[sample.pose_id]}] 出现）")
            seen[sample.pose_id] = index
            _check_joint_limits(sample.q_deg, kinematics, f"samples[{index}]")
        self._by_id = seen
        if kinematics is None or not hasattr(kinematics, "fk"):
            raise SessionError("kinematics 必须为提供 fk() 的运动学对象")
        self.kinematics = kinematics
        self.source_path = None if source_path is None else Path(source_path)

    @property
    def reference_effective(self) -> np.ndarray:
        """有效参考点 ``^B P_ball^laser + ^B ΔP``（``delta_p_base`` 为 ``None`` 视为 0）。"""
        if self.delta_p_base is None:
            return self.reference_point.copy()
        return self.reference_point + self.delta_p_base

    def sample_by_id(self, pose_id) -> Sample:
        """按 ``pose_id`` 取样本。

        Args:
            pose_id: 位姿编号（整数）。

        Returns:
            对应的 :class:`Sample`。

        Raises:
            KeyError: 会话里没有该 ``pose_id``。
        """
        if pose_id in self._by_id:
            return self.samples[self._by_id[pose_id]]
        known = sorted(self._by_id)
        raise KeyError(f"会话里不存在 pose_id={pose_id!r} 的样本（已有 {known}）")

    def all_target_ids(self) -> set:
        """会话内所有样本、所有帧出现过的目标 id 并集。"""
        ids: set = set()
        for sample in self.samples:
            ids.update(sample.target_ids())
        return ids


# ---------------------------------------------------------------------------
# JSON -> 模型
# ---------------------------------------------------------------------------


def _resolve_input_path(raw, base_dir, where: str) -> Path:
    """解析输入文件路径：绝对路径直接用，相对路径先试 ``base_dir`` 再试仓库根。"""
    if not isinstance(raw, str) or not raw.strip():
        raise SessionError(f"{where} 必须为非空路径字符串，当前 {raw!r}")
    candidate = Path(raw).expanduser()
    if candidate.is_absolute():
        if not candidate.is_file():
            raise SessionError(f"{where} 指向的文件不存在：{candidate}")
        return candidate

    tried = []
    if base_dir is not None:
        tried.append(Path(base_dir) / candidate)
    tried.append(REPO_ROOT / candidate)
    for option in tried:
        if option.is_file():
            return option
    locations = "、".join(str(item) for item in tried)
    raise SessionError(f"{where} 指向的文件不存在：{raw}（已尝试 {locations}）")


def _check_joint_limits(q_deg, kinematics, where: str) -> None:
    """检查 6 个关节角是否落在 ``kinematics.limits`` 内。"""
    limits = getattr(kinematics, "limits", None)
    if limits is None:
        return
    for index, (value, (low, high)) in enumerate(zip(q_deg, limits)):
        if not low <= value <= high:
            raise SessionError(
                f"{where}.q_deg[{index}]={value:g} 超出第 {index + 1} 轴限位 "
                f"[{low:g}, {high:g}] 度")


def _parse_sample(raw, index: int, kinematics) -> Sample:
    """解析 ``samples[index]`` 并检查关节限位。"""
    where = f"samples[{index}]"
    mapping = _require_mapping(raw, where)
    pose_id = _require_key(mapping, "pose_id", where)
    q_deg = _require_key(mapping, "q_deg", where)
    frames = _require_key(mapping, "frames", where)
    sample = Sample(pose_id, q_deg, frames,
                    recorded_pose=mapping.get("recorded_pose"), where=where)
    _check_joint_limits(sample.q_deg, kinematics, where)
    return sample


def session_from_dict(payload, *, base_dir=None, source_path=None) -> Session:
    """从已解析的 JSON 字典构造 :class:`Session`。

    Args:
        payload: 会话字典（见模块 docstring 的 schema）。
        base_dir: 相对路径的优先解析目录（通常是会话文件所在目录）；找不到
            时回落到仓库根。``None`` 表示只用仓库根。
        source_path: 会话文件路径，写进 ``Session.source_path``；同时作为
            ``base_dir`` 的默认值。

    Returns:
        校验通过的 :class:`Session`。

    Raises:
        SessionError: 顶层不是对象 / 缺必填键 / 类型或数值不合法 / URDF 或
            DH 覆盖无法装载 / ``pose_id`` 重复 / 关节超限。
    """
    if source_path is not None:
        source_path = Path(source_path)
        if base_dir is None:
            base_dir = source_path.parent
    if not isinstance(payload, dict):
        raise SessionError(
            f"会话 JSON 顶层必须为对象，当前 {type(payload).__name__}")

    # ---- meta ----
    meta_raw = _require_mapping(_require_key(payload, "meta", "会话"), "meta")
    urdf_path = _resolve_input_path(
        _require_key(meta_raw, "robot_urdf", "meta"), base_dir, "meta.robot_urdf")

    override_raw = meta_raw.get("dh_override")
    override_path = None
    if override_raw is not None:
        override_path = _resolve_input_path(
            override_raw, base_dir, "meta.dh_override")

    group_raw = meta_raw.get("group")
    if group_raw is not None:
        _as_int(group_raw, "meta.group")

    convention = meta_raw.get("joint_convention", "model")
    if convention not in SUPPORTED_JOINT_CONVENTIONS:
        raise SessionError(
            f"meta.joint_convention 必须为 "
            f"{SUPPORTED_JOINT_CONVENTIONS[0]!r}（本期只支持模型轴角度，"
            f"即 q_deg 直接喂给 RobotKinematics.fk），当前 {convention!r}")

    units = meta_raw.get("units", "mm")
    if units not in SUPPORTED_UNITS:
        raise SessionError(
            f"meta.units 必须为 {SUPPORTED_UNITS[0]!r}（长度单位，作用于 "
            f"target_points / p_ball_laser_base / delta_p_base / recorded_pose），"
            f"当前 {units!r}")

    try:
        override = None if override_path is None else DhOverride.from_json(override_path)
        kinematics = RobotKinematics(urdf_path, override)
    except (OSError, ValueError, KeyError) as error:
        raise SessionError(
            f"无法构造机器人运动学（meta.robot_urdf={str(urdf_path)!r}, "
            f"meta.dh_override={None if override_path is None else str(override_path)!r}）："
            f"{error}") from error

    # ---- reference ----
    reference_raw = _require_mapping(
        _require_key(payload, "reference", "会话"), "reference")
    reference_point = _as_finite_floats(
        _require_key(reference_raw, "p_ball_laser_base", "reference"),
        "reference.p_ball_laser_base", 3)
    delta_raw = reference_raw.get("delta_p_base")
    delta_p_base = (
        None if delta_raw is None
        else _as_finite_floats(delta_raw, "reference.delta_p_base", 3))

    # ---- handeye_matrix ----
    handeye_matrix = None
    handeye_raw = payload.get("handeye_matrix")
    if handeye_raw is not None:
        handeye_mapping = _require_mapping(handeye_raw, "handeye_matrix")
        matrix_units = handeye_mapping.get("units", "mm")
        if matrix_units not in _MATRIX_UNITS:
            raise SessionError(
                f"handeye_matrix.units 必须为 {' 或 '.join(repr(u) for u in _MATRIX_UNITS)}，"
                f"当前 {matrix_units!r}")
        values = _as_finite_floats(
            _require_key(handeye_mapping, "matrix16_rowmajor", "handeye_matrix"),
            "handeye_matrix.matrix16_rowmajor", 16)
        try:
            handeye_matrix = load_matrix16(
                values, units=matrix_units, target_units="mm")
        except ValueError as error:
            raise SessionError(
                f"handeye_matrix.matrix16_rowmajor 不合法：{error}") from error

    # ---- samples ----
    samples_raw = _require_key(payload, "samples", "会话")
    if isinstance(samples_raw, (str, bytes, dict)) or not isinstance(
            samples_raw, (list, tuple)):
        raise SessionError(
            f"samples 必须为数组，当前 {type(samples_raw).__name__}")
    if not samples_raw:
        raise SessionError("samples 不能为空，至少需要 1 个位姿")

    samples = []
    seen: dict = {}
    for index, raw in enumerate(samples_raw):
        sample = _parse_sample(raw, index, kinematics)
        if sample.pose_id in seen:
            raise SessionError(
                f"samples[{index}].pose_id 重复：{sample.pose_id}"
                f"（已在 samples[{seen[sample.pose_id]}] 出现）")
        seen[sample.pose_id] = index
        samples.append(sample)

    return Session(
        meta=meta_raw,
        reference_point=reference_point,
        delta_p_base=delta_p_base,
        handeye_matrix=handeye_matrix,
        samples=samples,
        kinematics=kinematics,
        source_path=source_path,
    )


def load_session(path, *, proto_root=None) -> Session:
    """从 JSON 文件装载会话。

    Args:
        path: 会话 JSON 路径。
        proto_root: 预留参数（当前纯 JSON 会话不含 protobuf；保留它是为了与
            ``handeye.proto_codec`` 的调用风格一致，值被忽略）。

    Returns:
        :class:`Session`。

    Raises:
        SessionError: 文件无法读取 / 不是合法 JSON / 内容不合法。
    """
    del proto_root  # 预留：本函数只消费 JSON，protobuf 由 proto_codec 负责
    path = Path(path)
    try:
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except OSError as error:
        raise SessionError(f"无法读取会话文件 {path}：{error}") from error
    except UnicodeDecodeError as error:
        raise SessionError(f"会话文件 {path} 不是 UTF-8 文本：{error}") from error
    except json.JSONDecodeError as error:
        raise SessionError(f"会话文件 {path} 不是合法 JSON：{error}") from error
    return session_from_dict(payload, source_path=path)


# ---------------------------------------------------------------------------
# FK 一致性报告
# ---------------------------------------------------------------------------


def _rms(values: list) -> float:
    """平方平均值开根；空列表返回 ``0.0``。"""
    if not values:
        return 0.0
    return float(math.sqrt(sum(value * value for value in values) / len(values)))


def check_fk_consistency(session) -> dict:
    """把每个样本的 ``recorded_pose`` 与 ``kinematics.fk(q_deg)`` 对比。

    用途：暴露 DH 参数、长度单位、欧拉约定（内旋/外旋）或
    ``joint_convention`` 与控制器不一致的问题。没有 ``recorded_pose`` 的
    样本计入 ``n_missing``。

    Args:
        session: :class:`Session`。

    Returns:
        dict：

        * ``n_compared``：参与对比的样本数；
        * ``n_missing``：没有 ``recorded_pose`` 的样本数；
        * ``position_max_mm`` / ``position_rms_mm``：位置差统计（mm）；
        * ``rotation_max_deg`` / ``rotation_rms_deg``：旋转差统计（deg，
          用 :func:`handeye.se3.rotation_angle_between`）；
        * ``worst``：位置差最大的样本 ``{"pose_id","position_mm","rotation_deg"}``，
          没有可比样本时为 ``None``；
        * ``per_sample``：逐样本``{"pose_id","position_mm","rotation_deg"}``。
    """
    positions: list = []
    rotations: list = []
    per_sample: list = []
    worst = None
    missing = 0
    for sample in session.samples:
        if sample.recorded_pose is None:
            missing += 1
            continue
        fk = session.kinematics.fk(sample.q_deg)
        position_mm = float(np.linalg.norm(
            fk[:3, 3] - sample.recorded_pose[:3, 3]))
        rotation_deg = float(rotation_angle_between(
            fk[:3, :3], sample.recorded_pose[:3, :3]))
        positions.append(position_mm)
        rotations.append(rotation_deg)
        per_sample.append({
            "pose_id": sample.pose_id,
            "position_mm": position_mm,
            "rotation_deg": rotation_deg,
        })
        if worst is None or position_mm > worst["position_mm"]:
            worst = {
                "pose_id": sample.pose_id,
                "position_mm": position_mm,
                "rotation_deg": rotation_deg,
            }

    return {
        "n_compared": len(per_sample),
        "n_missing": missing,
        "position_max_mm": max(positions) if positions else 0.0,
        "position_rms_mm": _rms(positions),
        "rotation_max_deg": max(rotations) if rotations else 0.0,
        "rotation_rms_deg": _rms(rotations),
        "worst": worst,
        "per_sample": per_sample,
    }
