# -*- coding: utf-8 -*-
"""视觉链推算（``T_bh(q) · X · p_cam``）与 VPE 指标。

指标定义
--------
对每个位姿 ``i``，先把该位姿所有帧、所有 id 的视觉链结果聚合成一个代表点

* 同一 id 的多帧结果取算术平均（``mean``），``spread_mm`` 是该 id 各帧结果
  到均值的欧氏距离的算术平均（视觉重复性）；
* 多 id 时 ``combined`` 取各 id 均值的算术平均。

误差与指标（``P_reference_effective`` 见 :attr:`handeye.session.Session.reference_effective`）::

    e_i      = P_vision_i - P_reference_effective
    VPE_i    = ||e_i||
    RMSE     = sqrt(mean(VPE_i^2))
    bias     = mean(e_i)
    逐轴 RMSE = sqrt(mean(e_axis^2))

``vpe_p95_mm`` 直接用 ``np.percentile(VPE, 95)``（线性插值，不做自定义分位数）。

约定
----
* 长度单位统一 mm；``X`` 必须是 flange←camera 的 4x4 刚体矩阵（平移 mm）。
* 本模块**只消费**给定的 X；手眼求解器不在本仓库。
* 库函数不打印、不 ``sys.exit``，非法输入抛 :class:`VpeError`。
"""

from __future__ import annotations

import numpy as np

from handeye.se3 import rotation_angle_between, transform_point

#: 逐轴名称，顺序与 ``xyz`` 一致。
_AXES = ("x", "y", "z")


class VpeError(ValueError):
    """VPE 计算所需的数据不完整，或矩阵不合法。"""


def _as_matrix(matrix, name: str) -> np.ndarray:
    """校验并返回 4x4 有限矩阵（不检查旋转正交性，便于比较「坏」矩阵）。"""
    try:
        array = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError) as error:
        raise VpeError(f"{name} 需为可转浮点的 4x4 矩阵：{error}") from error
    if array.shape != (4, 4):
        raise VpeError(f"{name} 需为 4x4 矩阵，当前 {array.shape}")
    if not np.all(np.isfinite(array)):
        raise VpeError(f"{name} 含非有限值")
    return array


def _resolve_matrix(session, matrix=None) -> np.ndarray:
    """取手眼矩阵：显式 ``matrix`` 优先，否则用 ``session.handeye_matrix``。"""
    if matrix is not None:
        return _as_matrix(matrix, "matrix")
    stored = getattr(session, "handeye_matrix", None)
    if stored is None:
        raise VpeError(
            "未提供手眼矩阵 X：matrix 参数为 None，且 session.handeye_matrix 为 null")
    return _as_matrix(stored, "session.handeye_matrix")


def _project_frame(session, sample, frame, matrix: np.ndarray) -> dict:
    """把一帧的所有靶点投到 base 系，返回 ``{id: ndarray(3)}``。"""
    chain = session.kinematics.fk(sample.q_deg) @ matrix
    points = getattr(frame, "target_points", None)
    if not points:
        raise VpeError(f"pose_id={sample.pose_id} 的某一帧没有任何靶点，无法推算")
    return {target_id: transform_point(chain, point)
            for target_id, point in points.items()}


def vision_ball_base_frame(session, sample, frame, matrix=None) -> dict | np.ndarray:
    """指定帧的视觉链结果：``T_bh(q) @ X @ [p_cam; 1]``。

    Args:
        session: :class:`handeye.session.Session`。
        sample: 该帧所属的 :class:`handeye.session.Sample`（提供 ``q_deg``）。
        frame: 单个 :class:`handeye.session.Frame`。
        matrix: 手眼矩阵 X（4x4，mm）；``None`` 时用 ``session.handeye_matrix``。

    Returns:
        该帧只有 1 个 id 时返回 ``np.ndarray(3,)``；有多个 id 时返回
        ``dict{id: np.ndarray(3)}``（base 系球心坐标，mm）。

    Raises:
        VpeError: 没有可用的手眼矩阵 / 矩阵不是 4x4 / 帧里没有靶点。
    """
    resolved = _resolve_matrix(session, matrix)
    points = _project_frame(session, sample, frame, resolved)
    if len(points) == 1:
        return next(iter(points.values())).copy()
    return points


def vision_ball_base(session, sample, matrix=None) -> np.ndarray:
    """单个位姿的视觉链结果（多帧、多 id 全部投影后取算术平均）。

    与 :func:`vision_ball_base_frame` 的区别：本函数**不要求**该位姿只有一帧，
    而是把每一帧的每个靶点都按 ``T_bh(q) @ X @ p_cam`` 算一次，再对所有结果
    取算术平均。同一位姿里同 id 出现多次时逐帧各算一次；单 id 时它与
    :func:`aggregate_sample_positions` 的 ``combined`` 完全相同（多 id 且各 id
    帧数不等时，本函数按「点」平均，而 ``combined`` 先按 id 取均值再平均）。

    Args:
        session: :class:`handeye.session.Session`。
        sample: 待推算的 :class:`handeye.session.Sample`。
        matrix: 手眼矩阵 X（4x4，mm）；``None`` 时用 ``session.handeye_matrix``。

    Returns:
        ``np.ndarray(3,)``：base 系球心坐标（mm）。

    Raises:
        VpeError: 没有可用的手眼矩阵 / 该位姿没有任何帧或靶点。
    """
    resolved = _resolve_matrix(session, matrix)
    projected = [point for frame in sample.frames
                 for point in _project_frame(session, sample, frame, resolved).values()]
    if not projected:
        raise VpeError(f"pose_id={sample.pose_id} 没有任何帧，无法推算球心")
    return np.mean(np.vstack(projected), axis=0)


def aggregate_sample_positions(session, sample, matrix=None) -> dict:
    """把一个位姿的多帧、多 id 结果投影到 base 系并按 id 聚合。

    Args:
        session: :class:`handeye.session.Session`。
        sample: 待聚合的 :class:`handeye.session.Sample`。
        matrix: 手眼矩阵 X（4x4，mm）；``None`` 时用 ``session.handeye_matrix``。

    Returns:
        dict：

        * ``"ids"``：``{id: {"mean": ndarray(3), "n": int, "spread_mm": float}}``，
          ``mean`` 为该 id 各帧结果的算术平均（mm），``n`` 为该 id 的帧数，
          ``spread_mm`` 为各帧结果到均值的欧氏距离均值（单帧时为 ``0.0``）；
        * ``"combined"``：各 id 均值的算术平均（mm）；没有任何 id 时为 ``None``。

    Raises:
        VpeError: 没有可用的手眼矩阵 / 帧里没有靶点。
    """
    resolved = _resolve_matrix(session, matrix)
    buckets: dict = {}
    for frame in sample.frames:
        for target_id, point in _project_frame(session, sample, frame, resolved).items():
            buckets.setdefault(target_id, []).append(np.asarray(point, dtype=float))

    ids: dict = {}
    for target_id, points in buckets.items():
        stack = np.vstack(points)
        mean = stack.mean(axis=0)
        spread = (float(np.mean(np.linalg.norm(stack - mean, axis=1)))
                  if len(stack) > 1 else 0.0)
        ids[target_id] = {
            "mean": mean,
            "n": int(len(stack)),
            "spread_mm": spread,
        }

    combined = (np.mean(np.vstack([entry["mean"] for entry in ids.values()]), axis=0)
                if ids else None)
    return {"ids": ids, "combined": combined}


def compute_vpe(session, matrix=None) -> dict:
    """整段会话的 VPE。

    空会话（``samples`` 为空）**抛** :class:`VpeError`：没有任何样本时统计量
    无定义，返回全 ``nan`` 的 metrics 容易在报告/CLI 里被静默当成「0 误差」。

    Args:
        session: :class:`handeye.session.Session`。
        matrix: 手眼矩阵 X（4x4，mm）；``None`` 时用 ``session.handeye_matrix``。

    Returns:
        dict：

        * ``n_samples``：参与统计的样本数；
        * ``n_frames``：参与统计的帧总数；
        * ``errors``：逐样本 ``{"pose_id","position_vision","error","vpe_mm",
          "n_frames","n_ids","spread_mm"}``；
        * ``metrics``：``vpe_rmse_mm``、``vpe_max_mm``、``vpe_p95_mm``、
          ``vpe_mean_mm``、``bias``、``bias_norm_mm``、``per_axis_rmse_mm``、
          ``per_axis_bias_mm``；
        * ``reference_mm``：``session.reference_effective``；
        * ``matrix``：本次实际使用的 X（mm）；
        * ``spread_mean_mm``：所有 ``(样本, id)`` 的 ``spread_mm`` 平均。

    Raises:
        VpeError: 没有可用的手眼矩阵 / ``samples`` 为空 / 某个位姿没有靶点。
    """
    resolved = _resolve_matrix(session, matrix)
    samples = list(session.samples)
    if not samples:
        raise VpeError("会话没有任何样本（samples 为空），无法计算 VPE")
    reference = np.asarray(session.reference_effective, dtype=float)

    errors: list = []
    all_spreads: list = []
    for sample in samples:
        aggregate = aggregate_sample_positions(session, sample, resolved)
        position = aggregate["combined"]
        if position is None:
            raise VpeError(f"pose_id={sample.pose_id} 没有任何靶点，无法计算 VPE")
        delta = np.asarray(position, dtype=float) - reference
        spreads = [entry["spread_mm"] for entry in aggregate["ids"].values()]
        all_spreads.extend(spreads)
        errors.append({
            "pose_id": sample.pose_id,
            "position_vision": np.asarray(position, dtype=float),
            "error": delta,
            "vpe_mm": float(np.linalg.norm(delta)),
            "n_frames": len(sample.frames),
            "n_ids": len(aggregate["ids"]),
            "spread_mm": float(np.mean(spreads)) if spreads else 0.0,
        })

    vpe = np.array([entry["vpe_mm"] for entry in errors], dtype=float)
    deltas = np.vstack([entry["error"] for entry in errors])
    bias = deltas.mean(axis=0)

    metrics = {
        "vpe_rmse_mm": float(np.sqrt(np.mean(vpe ** 2))),
        "vpe_max_mm": float(np.max(vpe)),
        "vpe_p95_mm": float(np.percentile(vpe, 95)),
        "vpe_mean_mm": float(np.mean(vpe)),
        "bias": bias,
        "bias_norm_mm": float(np.linalg.norm(bias)),
        "per_axis_rmse_mm": {
            axis: float(np.sqrt(np.mean(deltas[:, index] ** 2)))
            for index, axis in enumerate(_AXES)},
        "per_axis_bias_mm": {
            axis: float(bias[index]) for index, axis in enumerate(_AXES)},
    }

    return {
        "n_samples": len(errors),
        "n_frames": int(sum(entry["n_frames"] for entry in errors)),
        "errors": errors,
        "metrics": metrics,
        "reference_mm": reference.copy(),
        "matrix": resolved.copy(),
        "spread_mean_mm": float(np.mean(all_spreads)) if all_spreads else 0.0,
    }


def compare_matrices(first, second) -> dict:
    """两个 4x4 位姿（或手眼矩阵）的差异，用于新旧 X 对比。

    Args:
        first: 4x4 矩阵（mm）。
        second: 4x4 矩阵（mm）。

    Returns:
        ``{"position_mm": float, "rotation_deg": float}``：平移差向量的模
        （mm）与旋转相对转角（deg）。

    Raises:
        VpeError: 任一矩阵不是 4x4 或含非有限值。
    """
    first_matrix = _as_matrix(first, "first")
    second_matrix = _as_matrix(second, "second")
    return {
        "position_mm": float(np.linalg.norm(
            first_matrix[:3, 3] - second_matrix[:3, 3])),
        "rotation_deg": float(rotation_angle_between(
            first_matrix[:3, :3], second_matrix[:3, :3])),
    }
