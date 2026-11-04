# -*- coding: utf-8 -*-
"""URDF 基准运动学 + DH 参数覆盖，产出「旧 DH / 新 DH」两套 FK。

背景
----
``robot_model.nexbot_ik.Nexbot6R`` 从 URDF 反解出标准 DH 参数（``a``/``d`` 单位
mm，``alpha``/``theta`` 偏置在类内部以 **弧度** 存放）。手眼离线重算需要回答
一个问题：把现场 ``dh_override.json`` 里的「新 DH」写到模型上之后，标定位姿会
移动多少？本模块在 URDF 基准之上叠加一层 **替换式** DH 覆盖：

* 旧 DH：``RobotKinematics(urdf)`` —— 覆盖为空，等价于 URDF 反解结果；
* 新 DH：``RobotKinematics(urdf, override)`` —— 6 个分量按索引 **替换**。

覆盖语义与单位
--------------
覆盖是「按索引替换」（不是相加）：``effective[k] = override[k]``。对外单位统一为

* ``a`` / ``d``：mm
* ``alpha`` / ``theta_offset``：度

``theta_offset`` 进入 ``dh_matrix`` 前先转弧度，``alpha`` 同理。复用
``robot_model.nexbot_ik.dh_matrix`` 逐节累乘，**不调用** ``Nexbot6R.fk``
（后者只会用未覆盖参数）。

用法示例::

    from handeye.dh import DhOverride, RobotKinematics, fk_difference

    old = RobotKinematics("data/configs/ch2000_origin.urdf", label="old")
    new = RobotKinematics("data/configs/ch2000_origin.urdf",
                          DhOverride.from_json("dh_override.json"), label="new")
    report = fk_difference(old, new, [[0] * 6, [10, 20, -30, 0, 40, 0]])

``is_identity`` 的语义
----------------------
覆盖值等价于「没有覆盖」，当且仅当它与 **基准参数** 逐分量相等；因此
``DhOverride`` 会记录一份基准快照（``from_base`` 自带，``from_json`` 载入的
对象可用 ``attach_base(urdf)`` 绑定）。**全 0 的覆盖不是 identity**：那是一个
所有连杆长度/转角都为零的退化机器人，而不是「保持原样」。未绑定基准时无法
证明等价，``is_identity`` 返回 ``False``。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from handeye.se3 import rotation_angle_between, to_rpy
from robot_model.nexbot_ik import Nexbot6R, dh_matrix

#: DH 覆盖的 4 个分量名（与 ``dh_override.json`` 的字段一一对应）。
DH_KEYS = ("a", "alpha", "d", "theta_offset")

#: 关节数（本项目只支持 6 轴本体）。
JOINT_COUNT = 6

# 长度单位 -> mm 的换算系数；键即 JSON 允许的写法
_LENGTH_TO_MM = {"mm": 1.0, "m": 1000.0}
# 角度单位 -> 度的换算系数
_ANGLE_TO_DEG = {"deg": 1.0, "rad": math.degrees(1.0)}

# 覆盖值合法范围（与现场参数的量级匹配，用于拦截明显的单位错误）
_MAX_ANGLE_DEG = 360.0
_MAX_LENGTH_MM = 5000.0


def _as_float_list(values, key: str) -> list[float]:
    """任意序列 -> ``list[float]``；长度/数值合法性留给 :func:`validate_override`。"""
    try:
        return [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise ValueError(f"DH 覆盖 {key!r} 含无法转为浮点的值：{error}") from error


def validate_override(override: "DhOverride") -> None:
    """校验一组 DH 覆盖的规模与量级，不合法抛 ``ValueError``。

    规则（与 ``RobotKinematics.__init__`` 的校验一致）：

    * 每个键必须恰好 6 个值；
    * 每个值必须是有限数（``nan`` / ``inf`` 拒绝）；
    * ``alpha`` / ``theta_offset`` 必须落在 ``[-360, 360]`` 度内；
    * ``a`` / ``d`` 的绝对值必须小于 5000 mm。

    Args:
        override: 待校验的 :class:`DhOverride`。

    Raises:
        ValueError: 违反上述任一规则；消息会说明是哪个键、第几个分量。
    """
    for key in DH_KEYS:
        values = list(getattr(override, key))
        if len(values) != JOINT_COUNT:
            raise ValueError(
                f"DH 覆盖 {key!r} 需恰好 {JOINT_COUNT} 个值，当前 {len(values)} 个")
        for index, value in enumerate(values):
            if not math.isfinite(value):
                raise ValueError(
                    f"DH 覆盖 {key}[{index}] 不是有限值：{value!r}")
            if key in ("alpha", "theta_offset"):
                if abs(value) > _MAX_ANGLE_DEG:
                    raise ValueError(
                        f"DH 覆盖 {key}[{index}]={value} 超出 "
                        f"[-{_MAX_ANGLE_DEG:g}, {_MAX_ANGLE_DEG:g}] 度")
            elif abs(value) >= _MAX_LENGTH_MM:
                raise ValueError(
                    f"DH 覆盖 {key}[{index}]={value} mm 的绝对值须小于 "
                    f"{_MAX_LENGTH_MM:g} mm")


class DhOverride:
    """一组 DH 覆盖参数（标准 DH，单位 mm / 度）。

    Attributes:
        a: 6 个连杆长度（mm）。
        alpha: 6 个连杆扭角（**度**）。
        d: 6 个连杆偏距（mm）。
        theta_offset: 6 个关节零位偏置（**度**）。
        name: 覆盖集名称（写进报告，便于区分现场 ``dh_override.json``）。
        base: 可选的基准快照（``dict``，键同 :data:`DH_KEYS`，单位 mm/度）；
            由 :meth:`from_base` / :meth:`attach_base` 写入，供
            :meth:`is_identity` 判断「是否等价于没有覆盖」。
    """

    def __init__(self, a, alpha, d, theta_offset, name: str = "override"):
        """保存 4 组各 6 个分量；这里只做类型归一化，合法性由 ``RobotKinematics`` 校验。

        Args:
            a: 6 个连杆长度（mm）。
            alpha: 6 个连杆扭角（度）。
            d: 6 个连杆偏距（mm）。
            theta_offset: 6 个关节零位偏置（度）。
            name: 覆盖集名称，默认 ``"override"``。
        """
        self.a = _as_float_list(a, "a")
        self.alpha = _as_float_list(alpha, "alpha")
        self.d = _as_float_list(d, "d")
        self.theta_offset = _as_float_list(theta_offset, "theta_offset")
        self.name = str(name)
        self.base: dict | None = None

    # ---------------- 构造 ----------------

    @classmethod
    def from_json(cls, path) -> "DhOverride":
        """从 ``dh_override.json`` 读入覆盖参数（长度换算到 mm，角度换算到度）。

        最小合法文件只含 4 个键（各 6 个值）::

            {"name": "ch2000_dh_20261103",
             "a": [150, 780, 250, 0, 0, 0],
             "alpha": [90, 0, 90, 90, -90, 0],
             "d": [510.5, -47.5, 0, 1048.7, 0, 102.6],
             "theta_offset": [0, 90, 0, 0, 0, 0],
             "units": {"length": "mm", "angle": "deg"}}

        ``units`` 可省略（默认 mm/deg）；写成 ``"m"`` / ``"rad"`` 时按单位换算。

        Args:
            path: JSON 文件路径（``str`` / ``Path``）。

        Returns:
            新的 :class:`DhOverride`（未绑定基准，``is_identity`` 为 ``False``）。

        Raises:
            ValueError: JSON 不是对象 / 缺字段 / 分量不是数值 / 单位无法识别。
            OSError: 文件不存在或无法读取。
        """
        path = Path(path)
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"DH 覆盖 JSON 顶层须为对象，当前 {type(data).__name__}")

        units = data.get("units") or {}
        if not isinstance(units, dict):
            raise ValueError(f"DH 覆盖 JSON 的 units 须为对象，当前 {type(units).__name__}")
        length_unit = str(units.get("length", "mm"))
        angle_unit = str(units.get("angle", "deg"))
        if length_unit not in _LENGTH_TO_MM:
            raise ValueError(
                f"未知长度单位 {length_unit!r}，支持 {sorted(_LENGTH_TO_MM)}")
        if angle_unit not in _ANGLE_TO_DEG:
            raise ValueError(
                f"未知角度单位 {angle_unit!r}，支持 {sorted(_ANGLE_TO_DEG)}")
        length_scale = _LENGTH_TO_MM[length_unit]
        angle_scale = _ANGLE_TO_DEG[angle_unit]

        def pick(key: str) -> list[float]:
            if key not in data:
                raise ValueError(f"DH 覆盖 JSON 缺少 {key!r} 字段")
            raw = data[key]
            if isinstance(raw, (str, bytes)) or not hasattr(raw, "__len__"):
                raise ValueError(f"DH 覆盖 JSON 的 {key!r} 须为数组，当前 {type(raw).__name__}")
            scale = length_scale if key in ("a", "d") else angle_scale
            return [float(value) * scale for value in raw]

        name = data.get("name", path.stem)
        return cls(pick("a"), pick("alpha"), pick("d"), pick("theta_offset"), name=str(name))

    @classmethod
    def from_base(cls, urdf_path, name: str = "base") -> "DhOverride":
        """取 URDF 反解出的基准 DH 作为覆盖（用于「旧 DH = 基准」场景）。

        ``Nexbot6R`` 内部 ``alpha`` / ``theta`` 偏置是弧度，这里统一换成度，
        因此该覆盖与原模型逐位等价，:meth:`is_identity` 返回 ``True``。

        Args:
            urdf_path: URDF 路径（``.urdf``）。
            name: 覆盖集名称，默认 ``"base"``。

        Returns:
            绑定基准快照的 :class:`DhOverride`。
        """
        robot = Nexbot6R(urdf_path)
        override = cls(
            a=robot.dh_a,
            alpha=[math.degrees(value) for value in robot.dh_alpha],
            d=robot.dh_d,
            theta_offset=[math.degrees(value) for value in robot.dh_off],
            name=name,
        )
        override.base = {
            "a": list(override.a),
            "alpha": list(override.alpha),
            "d": list(override.d),
            "theta_offset": list(override.theta_offset),
        }
        return override

    # ---------------- 导出 ----------------

    def as_dict(self) -> dict:
        """导出 ``{"name","a","alpha","d","theta_offset"}``（mm / 度，浅拷贝）。"""
        return {
            "name": self.name,
            "a": list(self.a),
            "alpha": list(self.alpha),
            "d": list(self.d),
            "theta_offset": list(self.theta_offset),
        }

    def to_json_dict(self) -> dict:
        """导出可直接 ``json.dumps`` 的字典（附带显式单位 ``mm`` / ``deg``）。"""
        payload = self.as_dict()
        payload["units"] = {"length": "mm", "angle": "deg"}
        return payload

    # ---------------- 语义判断 ----------------

    def is_identity(self, tol: float = 1e-9) -> bool:
        """本覆盖是否等价于「没有覆盖」（即与基准 DH 逐分量相等，误差 ``tol``）。

        全 0 覆盖 **不是** identity：全零 DH 对应一个所有连杆为零的退化机器人。
        未绑定基准（``base is None``）时无法证明等价，返回 ``False``。

        Args:
            tol: 逐分量比较的绝对容差，默认 ``1e-9``。

        Returns:
            与基准等价返回 ``True``，否则 ``False``。
        """
        if self.base is None:
            return False
        for key in DH_KEYS:
            mine = getattr(self, key)
            reference = list(self.base.get(key, []))
            if len(reference) != len(mine):
                return False
            if any(abs(value - other) > tol for value, other in zip(mine, reference)):
                return False
        return True

    def attach_base(self, urdf_path) -> "DhOverride":
        """就地绑定 URDF 基准快照（供 :meth:`is_identity` 判断），返回自身。

        Args:
            urdf_path: URDF 路径（``.urdf``）。

        Returns:
            ``self``（便于链式调用）。
        """
        reference = DhOverride.from_base(urdf_path, name=self.name)
        self.base = {
            "a": list(reference.a),
            "alpha": list(reference.alpha),
            "d": list(reference.d),
            "theta_offset": list(reference.theta_offset),
        }
        return self

    def copy(self) -> "DhOverride":
        """返回一份深拷贝（``base`` 快照一并复制）。"""
        clone = DhOverride(self.a, self.alpha, self.d, self.theta_offset, name=self.name)
        clone.base = None if self.base is None else {
            key: list(values) for key, values in self.base.items()}
        return clone

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return f"DhOverride(name={self.name!r}, identity={self.is_identity()})"


class RobotKinematics:
    """URDF 基准模型 + 可选 DH 覆盖，提供 ``fk`` 与位姿输出。

    Attributes:
        label: 便于报告区分的标签（默认取 ``override.name``，无覆盖时为 ``"base"``）。
        urdf_path: 基准 URDF 路径。
        limits: 6 个关节软限位 ``[(lo, hi), ...]``（度）。
        dh: 生效的 DH 参数 ``{"a","alpha","d","theta_offset"}``（mm / 度）。
        override: 绑定基准快照的覆盖副本；无覆盖时为 ``None``。
    """

    def __init__(self, urdf_path, override: "DhOverride | None" = None, label=None):
        """加载基准 DH，校验并叠加覆盖。

        Args:
            urdf_path: URDF 路径（``.urdf``，由 ``Nexbot6R`` 反解 DH）。
            override: 可选的 DH 覆盖；``None`` 表示直接用基准参数（旧 DH）。
            label: 标签，默认取 ``override.name``；两者都为空时用 ``"base"``。

        Raises:
            ValueError: 覆盖的分量数不是 6 / 含非有限值 / ``alpha``、
                ``theta_offset`` 超出 ``[-360, 360]`` 度 / ``a``、``d`` 绝对值
                不小于 5000 mm。
        """
        self.urdf_path = Path(urdf_path)
        robot = Nexbot6R(self.urdf_path)
        self.limits = [(float(low), float(high)) for low, high in robot.limits]

        self.base_dh = {
            "a": [float(value) for value in robot.dh_a],
            "alpha": [math.degrees(value) for value in robot.dh_alpha],
            "d": [float(value) for value in robot.dh_d],
            "theta_offset": [math.degrees(value) for value in robot.dh_off],
        }

        if override is None:
            self.override = None
            self.dh = {key: list(values) for key, values in self.base_dh.items()}
        else:
            validate_override(override)
            # 不改动调用方的对象：复制一份并绑定基准快照，让 is_identity 有意义
            attached = override.copy()
            if attached.base is None:
                attached.base = {
                    key: list(values) for key, values in self.base_dh.items()}
            self.override = attached
            self.dh = {key: list(getattr(attached, key)) for key in DH_KEYS}

        if label is not None:
            self.label = str(label)
        elif self.override is not None:
            self.label = self.override.name
        else:
            self.label = "base"

        # 预先转成弧度，避免每次 fk 重复换算
        self._alpha_rad = [math.radians(value) for value in self.dh["alpha"]]
        self._theta_offset_rad = [math.radians(value) for value in self.dh["theta_offset"]]

    def fk(self, q_deg) -> np.ndarray:
        """正运动学：6 个关节角（度）-> base->flange 的 4x4 齐次矩阵（mm）。

        与 ``Nexbot6R.fk`` 的区别只在于参数来源：本方法用 **生效 DH**
        （基准 + 覆盖），内部仍是逐节乘 ``robot_model.nexbot_ik.dh_matrix``。

        Args:
            q_deg: 6 个关节角（度）。

        Returns:
            4x4 ``np.ndarray``，平移单位 mm。

        Raises:
            ValueError: 关节角数量不是 6。
        """
        q = [float(value) for value in q_deg]
        if len(q) != JOINT_COUNT:
            raise ValueError(f"关节角需 {JOINT_COUNT} 个，当前 {len(q)} 个")
        a = self.dh["a"]
        d = self.dh["d"]
        theta_offset_rad = self._theta_offset_rad
        alpha_rad = self._alpha_rad
        transform = np.eye(4, dtype=float)
        for index in range(JOINT_COUNT):
            transform = transform @ dh_matrix(
                math.radians(q[index]) + theta_offset_rad[index],
                d[index], a[index], alpha_rad[index])
        return transform

    def pose_rpy(self, q_deg) -> tuple:
        """正运动学 + 欧拉角分解。

        Args:
            q_deg: 6 个关节角（度）。

        Returns:
            ``(xyz, rpy)``：``xyz`` 为 mm 的 ``np.ndarray``，``rpy`` 为度（定轴
            XYZ，与示教器一致，见 :func:`handeye.se3.to_rpy`）。
        """
        transform = self.fk(q_deg)
        return to_rpy(transform)

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return f"RobotKinematics(label={self.label!r}, override={self.override!r})"


def fk_difference(reference: "RobotKinematics", candidate: "RobotKinematics",
                  q_samples) -> dict:
    """两组参数在同一批关节角下的 FK 差异（位置 mm / 旋转 度）。

    典型用法是把「旧 DH」当 ``reference``、「新 DH」当 ``candidate``，用于量化
    现场 DH 覆盖会带来多大的位姿移动；位置差是平移向量之差的模，旋转差用
    :func:`handeye.se3.rotation_angle_between`（相对转角）。

    Args:
        reference: 参考模型（通常是旧 DH）。
        candidate: 待比较模型（通常是新 DH）。
        q_samples: 关节角样本序列，每项 6 个角度（度）。

    Returns:
        dict，键为 ``n``（样本数）、``position_max_mm``、``position_rms_mm``、
        ``rotation_max_deg``、``rotation_rms_deg``，以及逐样本明细
        ``per_sample``（每项 ``{"index","position_mm","rotation_deg"}``）。
        样本为空时统计量全为 ``0.0``。

    Raises:
        ValueError: 某个样本的关节角数量不是 6（由 :meth:`RobotKinematics.fk` 抛出）。
    """
    positions: list[float] = []
    rotations: list[float] = []
    per_sample: list[dict] = []

    for index, q_deg in enumerate(q_samples):
        reference_pose = reference.fk(q_deg)
        candidate_pose = candidate.fk(q_deg)
        position_mm = float(np.linalg.norm(
            candidate_pose[:3, 3] - reference_pose[:3, 3]))
        rotation_deg = float(rotation_angle_between(
            reference_pose[:3, :3], candidate_pose[:3, :3]))
        positions.append(position_mm)
        rotations.append(rotation_deg)
        per_sample.append({
            "index": index,
            "position_mm": position_mm,
            "rotation_deg": rotation_deg,
        })

    def _rms(values: list[float]) -> float:
        if not values:
            return 0.0
        return float(math.sqrt(float(np.mean(np.square(values)))))

    return {
        "n": len(per_sample),
        "position_max_mm": max(positions) if positions else 0.0,
        "position_rms_mm": _rms(positions),
        "rotation_max_deg": max(rotations) if rotations else 0.0,
        "rotation_rms_deg": _rms(rotations),
        "per_sample": per_sample,
    }
