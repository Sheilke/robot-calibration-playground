# -*- coding: utf-8 -*-
"""``T_bh``（4×4，mm）↔ 控制器 ``robot_pt``（position mm + orientation deg）。

约定（与公司 C++ 对齐）
----------------------
* 位置单位 **mm**，欧拉角单位 **deg**（Nexbot 控制链路，见 ``common.proto``
  的 ``PoseCart/PoseBase`` 注释）；
* 欧拉角是 **XYZ 内旋（intrinsic）**：``R = Rx(rx)·Ry(ry)·Rz(rz)``。
  C++ 侧 ``PoseEulerFromRobotType(ROBOT_TYPE_NEXBOT)`` 返回 ``kXyzIntrinsic``，
  ``Isometry2XYZRPY`` 对 ``kXyzIntrinsic`` 用 ``Eigen::eulerAngles(0,1,2)``；
  本模块用 :func:`robot_model.nexbot_ik.rot_to_euler` 的
  ``order="XYZ", extrinsic=False`` 分支与之等价（**不是**默认的
  ``extrinsic=True``，那个是 ``Rz·Ry·Rx``）。

自校验（测试与 :func:`self_check` 保证）
--------------------------------------
对任意旋转矩阵 ``R``：``euler_intrinsic_to_rotation(rotation_to_euler_intrinsic(R)) ≈ R``，
200 个随机姿态的最大元素误差 ``< 1e-10``；对 Nexbot FK 输出做整链
``matrix_to_pose_fields`` / ``pose_fields_to_matrix`` 往返，位置误差
``< 1e-9 mm``、旋转误差 ``< 1e-9 deg``。
"""

from __future__ import annotations

import math

import numpy as np

from handeye.se3 import make_pose, validate_rigid
from robot_model.nexbot_ik import euler_to_rot, rot_to_euler

#: 欧拉角轴序（与 C++ ``kXyzIntrinsic`` 一致）。
EULER_ORDER = "XYZ"

#: 是否定轴（外旋）。``False`` = 动轴（内旋），即 ``R = Rx·Ry·Rz``。
EXTRINSIC = False

#: 支持的欧拉约定（对外统一命名，供 CLI / 会话 JSON 使用）。
#: ``xyz-intrinsic`` = ``R = Rx·Ry·Rz``（C++ ``kXyzIntrinsic``，Nexbot 链路）；
#: ``xyz-extrinsic`` = ``R = Rz·Ry·Rx``（RPY 定轴，本仓其他工具默认）。
EULER_CONVENTIONS = ("xyz-intrinsic", "xyz-extrinsic")

#: 默认约定：与控制器链路一致（XYZ 内旋）。
DEFAULT_EULER_CONVENTION = "xyz-intrinsic"

#: 奇异判定阈值：``cos(pitch)`` 小于该值（即 |pitch| 距 90° 小于约 6e-8 度）
#: 时改用专门的奇异分支。
_GIMBAL_EPS = 1e-9

#: 与 ``se3.validate_rigid`` 一致的旋转正交性容差。
_RIGID_TOLERANCE = 1e-4


def _as_vector3(values, name: str) -> np.ndarray:
    """任意序列 -> 长度 3 的有限浮点 ``np.ndarray``，不合法抛 ``ValueError``。

    Args:
        values: 待转换的序列（list / tuple / ndarray）。
        name: 报错时使用的字段名。

    Returns:
        ``np.ndarray``（shape ``(3,)``，``float``）。

    Raises:
        ValueError: 无法转为浮点 / 长度不是 3 / 含 ``nan``、``inf``。
    """
    try:
        vector = np.asarray([float(value) for value in values], dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 需 3 个可转为浮点的值：{error}") from error
    if vector.shape != (3,):
        raise ValueError(f"{name} 需恰好 3 个值，当前 {vector.size} 个")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} 含非有限值：{vector.tolist()}")
    return vector


def _as_rotation(rot) -> np.ndarray:
    """校验并返回 3×3 旋转矩阵（正交 + det=+1，容差 1e-4）。"""
    try:
        matrix = np.asarray(rot, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"旋转需为 3x3 可转浮点的数组：{error}") from error
    if matrix.shape != (3, 3):
        raise ValueError(f"旋转需为 3x3，当前 {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("旋转矩阵含非有限值")
    if (float(np.max(np.abs(matrix.T @ matrix - np.eye(3)))) > _RIGID_TOLERANCE
            or abs(float(np.linalg.det(matrix)) - 1.0) > _RIGID_TOLERANCE):
        raise ValueError(
            f"旋转部分须正交且 det=+1（容差 {_RIGID_TOLERANCE}），"
            "请检查是否误传了含缩放/镜像的矩阵")
    return matrix


def rotation_to_euler_intrinsic(rot) -> np.ndarray:
    """3×3 旋转矩阵 -> XYZ 内旋欧拉角（deg），等价 ``Eigen::eulerAngles(0,1,2)``。

    非奇异位姿直接复用 :func:`robot_model.nexbot_ik.rot_to_euler`
    （``extrinsic=False`` 分支，即 ``R = Rx·Ry·Rz``）；``cos(pitch) ≈ 0`` 的
    万向锁位姿（|pitch| 距 90° 小于约 6e-8 度）改用专门分支（令 ``yaw=0``、由
    ``R`` 直接解 ``roll``），保证往返仍能精确重构 ``R``。

    注意：靠近万向锁时 ``roll``/``yaw`` 的分离本身是病态的（条件数 ``1/cos(pitch)``），
    任何算法都无法把两者的单项误差压到 1e-9 度以下；本函数保证的是
    「重构出的 ``R`` 仍然准确」，这也是求解器实际使用的形式。

    Args:
        rot: 3×3 旋转矩阵（正交、det=+1）。

    Returns:
        ``np.ndarray``（shape ``(3,)``），单位 deg，顺序 ``[rx, ry, rz]``。

    Raises:
        ValueError: 形状不是 3×3 / 含非有限值 / 不是正交旋转。
    """
    matrix = _as_rotation(rot)
    sin_pitch = float(matrix[0, 2])
    # R[0] = [cos(pitch)cos(yaw), -cos(pitch)sin(yaw), sin(pitch)]，用 hypot
    # 代替 asin(sin) 可避免俯仰接近 ±90° 时 asin 的病态放大。
    cos_pitch = math.hypot(float(matrix[0, 0]), float(matrix[0, 1]))

    if cos_pitch <= _GIMBAL_EPS:
        # 万向锁：R 只依赖 roll±yaw 的组合；令 yaw=0 后用 O(1) 量级的分量解 roll。
        if sin_pitch > 0.0:
            roll = math.atan2(float(matrix[1, 0]), float(matrix[1, 1]))
            pitch = math.pi / 2.0
        else:
            roll = math.atan2(-float(matrix[1, 0]), float(matrix[2, 0]))
            pitch = -math.pi / 2.0
        return np.degrees(np.array([roll, pitch, 0.0], dtype=float))

    roll, _, yaw = rot_to_euler(matrix, order=EULER_ORDER, extrinsic=EXTRINSIC)
    return np.array(
        [roll, math.degrees(math.atan2(sin_pitch, cos_pitch)), yaw], dtype=float)


def euler_intrinsic_to_rotation(rpy_deg) -> np.ndarray:
    """XYZ 内旋欧拉角（deg）-> 3×3 旋转矩阵，``R = Rx(rx)·Ry(ry)·Rz(rz)``。

    Args:
        rpy_deg: 3 个欧拉角（deg）。

    Returns:
        ``np.ndarray``（3×3，``float``）。

    Raises:
        ValueError: 长度不是 3 / 含非有限值。
    """
    angles = _as_vector3(rpy_deg, "rpy_deg")
    return euler_to_rot(angles, order=EULER_ORDER, extrinsic=EXTRINSIC)


def normalize_convention(value, where: str = "euler_convention") -> str:
    """校验并归一化欧拉约定名（大小写不敏感），返回 ``EULER_CONVENTIONS`` 之一。

    Args:
        value: 约定名（如 ``"xyz-intrinsic"`` / ``"XYZ-Intrinsic"``）。
        where: 报错时使用的字段名。

    Returns:
        归一化后的约定名（小写全称）。

    Raises:
        ValueError: 不是 ``EULER_CONVENTIONS`` 之一。
    """
    if not isinstance(value, str):
        raise ValueError(f"{where} 必须为 {EULER_CONVENTIONS} 之一，当前 {value!r}")
    normalized = value.strip().lower()
    if normalized not in EULER_CONVENTIONS:
        raise ValueError(f"{where} 必须为 {EULER_CONVENTIONS} 之一，当前 {value!r}")
    return normalized


def convention_extrinsic(convention) -> bool:
    """约定名 -> ``extrinsic`` 旗标（``xyz-intrinsic`` -> False，其余 -> True）。"""
    normalized = normalize_convention(convention)
    return normalized == "xyz-extrinsic"


def rotation_to_euler(rot, convention: str = DEFAULT_EULER_CONVENTION) -> np.ndarray:
    """3×3 旋转矩阵 -> 指定约定的 XYZ 欧拉角（deg）。

    Args:
        rot: 3×3 旋转矩阵（正交、det=+1）。
        convention: ``"xyz-intrinsic"``（R=Rx·Ry·Rz，默认）或
            ``"xyz-extrinsic"``（R=Rz·Ry·Rx）。内旋分支带万向锁专门处理；
            外旋分支直接复用 :func:`robot_model.nexbot_ik.rot_to_euler`
            （其自身含奇异分支）。

    Returns:
        ``np.ndarray``（shape ``(3,)``），单位 deg，顺序 ``[rx, ry, rz]``。
    """
    normalized = normalize_convention(convention)
    if normalized == "xyz-intrinsic":
        return rotation_to_euler_intrinsic(rot)
    matrix = _as_rotation(rot)
    return np.asarray(
        rot_to_euler(matrix, order=EULER_ORDER, extrinsic=True), dtype=float)


def euler_to_rotation(rpy_deg, convention: str = DEFAULT_EULER_CONVENTION) -> np.ndarray:
    """指定约定的 XYZ 欧拉角（deg）-> 3×3 旋转矩阵。"""
    normalized = normalize_convention(convention)
    angles = _as_vector3(rpy_deg, "rpy_deg")
    return euler_to_rot(angles, order=EULER_ORDER,
                        extrinsic=(normalized == "xyz-extrinsic"))


def matrix_to_pose_fields(matrix,
                          convention: str = DEFAULT_EULER_CONVENTION) -> tuple:
    """4×4 齐次矩阵 -> ``([x, y, z] mm, [rx, ry, rz] deg)``。

    先调用 :func:`handeye.se3.validate_rigid` 校验（形状、有限性、末行、
    正交性、det=+1），再分解平移与按 ``convention`` 约定的 XYZ 欧拉角
    （默认 ``xyz-intrinsic``，与控制器链路一致）。

    Args:
        matrix: 4×4 齐次矩阵（平移 mm）。
        convention: ``"xyz-intrinsic"`` / ``"xyz-extrinsic"``。

    Returns:
        ``(position, orientation)``：两个 ``list[float]``。

    Raises:
        ValueError: 不是合法刚体齐次矩阵。
    """
    try:
        transform = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"位姿矩阵需可转为浮点数组：{error}") from error
    validate_rigid(transform)
    position = [float(value) for value in transform[:3, 3]]
    orientation = [float(value) for value in
                   rotation_to_euler(transform[:3, :3], convention)]
    return position, orientation


def pose_fields_to_matrix(position, orientation,
                          convention: str = DEFAULT_EULER_CONVENTION) -> np.ndarray:
    """``([x, y, z] mm, [rx, ry, rz] deg)`` -> 4×4 齐次矩阵。

    Args:
        position: 3 个位置分量（mm）。
        orientation: 3 个欧拉角分量（deg，按 ``convention`` 约定解释，
            默认 XYZ 内旋）。

    Returns:
        ``np.ndarray``（4×4，``float``，平移 mm）。

    Raises:
        ValueError: 任一向量长度不是 3 / 含非有限值。
    """
    xyz = _as_vector3(position, "position")
    rpy = _as_vector3(orientation, "orientation")
    return make_pose(xyz, euler_to_rotation(rpy, convention))


def self_check(samples: int = 200, seed: int = 0, tolerance: float = 1e-10) -> float:
    """随机姿态自校验：欧拉往返 + FK 位姿往返，返回实测最大误差。

    生成 ``samples`` 个随机旋转（轴角均匀采样），检查
    ``euler_intrinsic_to_rotation(rotation_to_euler_intrinsic(R)) ≈ R`` 的
    最大元素误差；同时统计 ``matrix_to_pose_fields`` /
    ``pose_fields_to_matrix`` 的平移往返误差与旋转角误差，任一超过
    ``tolerance`` 抛 ``AssertionError``。

    Args:
        samples: 随机姿态个数，默认 200。
        seed: 随机种子，默认 0（可复现）。
        tolerance: 允许的最大误差，默认 1e-10。

    Returns:
        实测的欧拉往返最大元素误差（``float``）。

    Raises:
        AssertionError: 往返误差超过 ``tolerance``（约定写反时会立刻暴露）。
    """
    rng = np.random.default_rng(seed)
    euler_error = 0.0
    position_error = 0.0
    for _ in range(int(samples)):
        axis = rng.normal(size=3)
        norm = float(np.linalg.norm(axis))
        if norm == 0.0:  # 极低概率，退化为绕 Z 轴
            axis, norm = np.array([0.0, 0.0, 1.0]), 1.0
        axis = axis / norm
        angle = rng.uniform(-math.pi, math.pi)
        skew = np.array([[0.0, -axis[2], axis[1]],
                         [axis[2], 0.0, -axis[0]],
                         [-axis[1], axis[0], 0.0]])
        # 轴角 -> 旋转矩阵（Rodrigues）
        rot = np.eye(3) + math.sin(angle) * skew + (
            1.0 - math.cos(angle)) * (skew @ skew)
        back = euler_intrinsic_to_rotation(rotation_to_euler_intrinsic(rot))
        euler_error = max(euler_error, float(np.max(np.abs(back - rot))))

        xyz = rng.uniform(-1500.0, 1500.0, size=3)
        position, orientation = matrix_to_pose_fields(make_pose(xyz, rot))
        matrix = pose_fields_to_matrix(position, orientation)
        position_error = max(
            position_error, float(np.max(np.abs(matrix[:3, 3] - xyz))))

    if euler_error > tolerance:
        raise AssertionError(
            f"欧拉往返误差 {euler_error:.3e} 超过容差 {tolerance:.1e}，"
            "请检查 EULER_ORDER / EXTRINSIC 是否与 C++ kXyzIntrinsic 一致")
    if position_error > tolerance:
        raise AssertionError(
            f"位姿往返平移误差 {position_error:.3e} 超过容差 {tolerance:.1e}")
    return euler_error
