# -*- coding: utf-8 -*-
"""SE(3) 基础运算与手眼矩阵装载/校验/单位换算。

手眼矩阵存储约定与公司仓库一致（arc-common ``hand_eye_utils.hpp``）：
16 值 row-major，末行 ``[0,0,0,1]``（容差 1e-6），旋转部分正交且
行列式 +1（容差 1e-4），**米制**。本仓库 FK 为 mm，故装载时按
``units`` / ``target_units`` 显式换算平移部分。
"""

from __future__ import annotations

import math

import numpy as np

from robot_model.nexbot_ik import euler_to_rot, rot_to_euler

# 与 arc-common hand_eye_utils.hpp 的校验容差保持一致
_RIGID_TOLERANCE = 1e-4
_LAST_ROW_TOLERANCE = 1e-6

# 每单位对应的 mm 数：换算 scale = factor[源单位] / factor[目标单位]
_UNIT_SCALES = {"mm": 1.0, "m": 1000.0}


def make_pose(xyz, rot: np.ndarray) -> np.ndarray:
    """平移 + 3x3 旋转 -> 4x4 齐次矩阵。"""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.asarray(rot, dtype=float)
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


def from_rpy(xyz, rpy_deg, order: str = "XYZ", extrinsic: bool = True) -> np.ndarray:
    """平移 + 欧拉角（deg）-> 4x4。默认定轴 XYZ（RPY），与示教器一致。"""
    return make_pose(xyz, euler_to_rot(rpy_deg, order=order, extrinsic=extrinsic))


def to_rpy(transform: np.ndarray, order: str = "XYZ",
           extrinsic: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """4x4 -> (xyz, rpy_deg)。"""
    transform = np.asarray(transform, dtype=float)
    return transform[:3, 3].copy(), np.array(
        rot_to_euler(transform[:3, :3], order=order, extrinsic=extrinsic), dtype=float)


def inverse(transform: np.ndarray) -> np.ndarray:
    """4x4 齐次矩阵求逆（R 正交，闭式）。"""
    transform = np.asarray(transform, dtype=float)
    rot = transform[:3, :3]
    result = np.eye(4, dtype=float)
    result[:3, :3] = rot.T
    result[:3, 3] = -rot.T @ transform[:3, 3]
    return result


def transform_point(transform: np.ndarray, point) -> np.ndarray:
    """4x4 作用到单个 3D 点。"""
    transform = np.asarray(transform, dtype=float)
    point = np.asarray(point, dtype=float)
    return transform[:3, :3] @ point + transform[:3, 3]


def transform_points(transform: np.ndarray, points) -> np.ndarray:
    """4x4 作用到 N x 3 点集。"""
    transform = np.asarray(transform, dtype=float)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotation_angle_between(first: np.ndarray, second: np.ndarray) -> float:
    """两个旋转矩阵的相对转角（deg）：acos(clip((tr(Ra^T Rb)-1)/2))。"""
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    relative = first.T @ second
    cosine = (float(np.trace(relative)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def validate_rigid(transform: np.ndarray, rigid_tolerance: float = _RIGID_TOLERANCE,
                   last_row_tolerance: float = _LAST_ROW_TOLERANCE) -> None:
    """校验 4x4 是刚体齐次矩阵，不合法抛 ``ValueError``。

    与公司 ``ValidateHandEyeMatrix`` 同容差：末行 [0,0,0,1]（1e-6）、
    旋转正交且 det=+1（1e-4）。
    """
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"矩阵形状需为 4x4，当前 {transform.shape}")
    if not np.all(np.isfinite(transform)):
        raise ValueError("矩阵含非有限值")
    expected_last_row = np.array([0.0, 0.0, 0.0, 1.0])
    if float(np.max(np.abs(transform[3] - expected_last_row))) > last_row_tolerance:
        raise ValueError(f"矩阵末行须为 [0,0,0,1]（容差 {last_row_tolerance}）")
    rot = transform[:3, :3]
    if (float(np.max(np.abs(rot.T @ rot - np.eye(3)))) > rigid_tolerance
            or abs(float(np.linalg.det(rot)) - 1.0) > rigid_tolerance):
        raise ValueError(f"旋转部分须正交且 det=+1（容差 {rigid_tolerance}）")


def load_matrix16(values, units: str = "mm", target_units: str = "mm") -> np.ndarray:
    """16 值 row-major -> 4x4（校验 + 平移单位换算）。

    Args:
        values: 16 个有限浮点（row-major）。
        units: 输入数据的长度单位（"mm" / "m"），公司 common_config 为 "m"。
        target_units: 目标单位，本仓库统一 "mm"。

    Raises:
        ValueError: 数量不是 16 / 含非有限值 / 末行或旋转不合法 / 单位未知。
    """
    values = [float(v) for v in values]
    if len(values) != 16:
        raise ValueError(f"matrix16 需恰好 16 个值（row-major），当前 {len(values)}")
    if not all(math.isfinite(v) for v in values):
        raise ValueError("matrix16 含非有限值")
    if units not in _UNIT_SCALES or target_units not in _UNIT_SCALES:
        raise ValueError(f"单位需为 {sorted(_UNIT_SCALES)}，当前 units={units!r} "
                         f"target_units={target_units!r}")
    matrix = np.array(values, dtype=float).reshape(4, 4)
    validate_rigid(matrix)
    scale = _UNIT_SCALES[units] / _UNIT_SCALES[target_units]
    if scale != 1.0:
        matrix[:3, 3] *= scale
    return matrix


def to_matrix16(transform: np.ndarray, units: str = "mm") -> list[float]:
    """4x4 -> 16 值 row-major（平移换算到 ``units``）。"""
    if units not in _UNIT_SCALES:
        raise ValueError(f"单位需为 {sorted(_UNIT_SCALES)}，当前 units={units!r}")
    transform = np.asarray(transform, dtype=float)
    scale = _UNIT_SCALES["mm"] / _UNIT_SCALES[units]
    out = transform.copy()
    out[:3, 3] *= scale
    return [float(v) for v in out.reshape(-1)]
