#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AABB 网格蛇形寻位路点规划工具（CH2000 六轴焊接机器人）。

迭代 1 —— 已交付:
    * Waypoint 数据类（一次定义齐全，后续批次直接复用）;
    * plan_waypoints(): AABB 网格蛇形路点生成（连续蛇形：每步只动一个轴，
      X 行内往返、Y 层内逐格、Z 层间直升，Y 方向逐层翻转）;
    * plot_path(): 3D 预览图（Agg 后端，无显示环境）;
    * CLI + ``--skip-ik`` 仅路点预览模式（路点表打印 / 预览图 / JSON 输出）。

迭代 2 —— 已交付:
    * ChainStep 数据类 + config_tag(): (arm, elbow, wrist) 构型标签;
    * solve_chain(): 链式 IK 选解引擎（度数域 score 复核 + 构型 tie-break
      + FK 复验；无解跳过 / strict 中止）;
    * ``--no-perturb`` 开发验证模式（参考姿态直接跑 IK 链）+ 成功/失败着色图。

迭代 3 —— 本批次范围（最终形态）:
    * 随机姿态扰动: 默认模式逐路点独立采样
      ``delta = (U(-20,20), U(-15,15), U(-3,3))`` deg（对应 rx/ry/rz），
      采样顺序与路点 index 一致（同 ``--seed`` 可复现），回填每个路点的
      ``delta_rpy`` 与 ``rpy_pert = rpy_ref + delta``（不 wrap）;
    * 默认流程只对扰动后路点求 IK（未扰动位姿不求）; ``--no-perturb``
      保持开发验证行为（参考姿态直接跑链，不采样扰动，扰动字段保持 None）;
    * JSON 最终结构: ``ik_chain`` 节改名 ``perturbed_ik``（字段不变）;
      ``meta`` 增加 ``perturbed`` 标记（默认 true / --no-perturb false）;
      同 seed 两次运行输出逐字节一致（浮点统一 round 6，无时间戳/随机序）。

迭代 4 —— 姿态可辨识性（条件数）前置校验:
    * 路点用于最小二乘参数辨识，姿态集需要充分激励才不会让辨识矩阵病态;
    * 对姿态集做 SVD 求条件数 ``sigma_max / sigma_min``，超过 ``--cond-max``
      则**重新采样扰动**（可设上限 ``--cond-attempts``），达标后才进入 IK;
    * 特征构造: 各姿态相对参考姿态的 SO(3) log 映射旋转向量（N x 3，去中心化）;
      ``--cond-position`` 时再并入位置（与姿态等 RMS 归一化）;
    * 校验在 IK **之前**：条件数只依赖位姿数据，重采样会作废全部 IK 结果，
      而解析 IK（8 分支 + 限位过滤 + FK 复验）是流程中最贵的一段;
    * ``--no-perturb`` / ``--skip-ik`` 下姿态完全相同，条件数退化，不做该校验。

蛇形规则（本项目核心需求）::

    自 (X_min, Y_min, Z_min) 起"一格一格走"：任意相邻两路点恰在一个轴
    上相差一格，无斜跳/飞移。

    Z 自下而上逐层 (z_min -> z_max):
      层内: Y 沿 y_dir 逐格前进（偶数层 y_min -> y_max，奇数层反向）:
        行内: 保持 Y, Z 不变, X 逐格扫描; 行方向逐行交替
              （全局连续，首行 X_min -> X_max）
        行间: 保持 X, Z 不变, Y 前进一格
      层末: 保持 X, Y 不变, 垂直上升一个 Z（无 XY 跳变），
            并翻转 y_dir，下一层自 Y 边界逐格返回。
    每层首行落点即该行端点，沿远离落点方向继续扫描。

用法示例（在仓库根目录）::

    PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \\
        0 200 0 200 0 200 --xs 2 --ys 2 --zs 2 --skip-ik \\
        --plot /tmp/p1.png --out /tmp/r1.json

    PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \\
        800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 --seed 7 \\
        --plot /tmp/p2.png --out /tmp/r2.json

    PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \\
        800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 --no-perturb \\
        --plot /tmp/p3.png --out /tmp/r3.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from robot_model.nexbot_ik import Nexbot6R, wrap_180

# ---------------------------------------------------------------------------
# 常量 / 默认值
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_RPY = (180.0, 0.0, -90.0)                                   # 参考姿态 deg
DEFAULT_URDF = _REPO_ROOT / "data" / "configs" / "ch2000_origin.urdf"
DEFAULT_Q_SEED = "0,20,-20,0,40,0"                                  # IK 种子关节角 deg
DEFAULT_SEED = 0                                                    # 随机种子
DEFAULT_OUT_JSON = "scan_result.json"
DEFAULT_OUT_PNG = "scan_path.png"
# 姿态条件数阈值。注意：该阈值与 PERTURB_AMPLITUDE_DEG 的幅度比耦合——
# 条件数度量的是三轴**相对**激励强度，幅度比为 20:15:3 时固有条件数约 4.6~8.4
# （300 seed 实测 p99 ≈ 7.8），重采样无法改善幅度比。取 9 可接受该幅度比，
# 同时仍能明确拒绝缺轴激励（双轴 ≈ 39，单轴/无激励 = inf）。
DEFAULT_COND_MAX = 9.0
DEFAULT_COND_ATTEMPTS = 20        # 条件数不达标时的最大重新采样次数

# 姿态扰动幅度（deg，对应 rx/ry/rz 的 ± 半宽）。
PERTURB_AMPLITUDE_DEG = (20.0, 15.0, 3.0)


# ---------------------------------------------------------------------------
# 数据类（一次定义齐全，后续批次复用）
# ---------------------------------------------------------------------------
@dataclass
class Waypoint:
    """单个寻位路点。"""

    index: int                      # 0-based 路径序
    pos: tuple                      # (x, y, z) mm
    rpy_ref: tuple                  # 参考姿态 deg（所有路点相同）
    rpy_pert: tuple | None = None   # 扰动后姿态 deg（迭代 3 填）
    delta_rpy: tuple | None = None  # 扰动量 deg（迭代 3 填）


@dataclass
class ChainStep:
    """链式 IK 选解引擎的单步结果（迭代 2）。"""

    index: int                 # 路点 0-based 序号
    q: list | None             # 选中解 deg（None=无解）
    n_sols: int                # ik 返回的解总数（无解为 0）
    config: tuple | None       # (arm, elbow, wrist) 构型标签，无解为 None
    dq_l2: float | None        # 相对上一选中解的 Σ wrap_180(Δq)²，无解为 None
    dq_l1: float | None        # Σ |wrap_180(Δq)|，无解为 None
    ok: bool                   # FK 复验通过（q 非空且 fk(q)≈T）


# ---------------------------------------------------------------------------
# 路点生成
# ---------------------------------------------------------------------------
def plan_waypoints(x_min: float, x_max: float,
                   y_min: float, y_max: float,
                   z_min: float, z_max: float,
                   xs: int, ys: int, zs: int,
                   rpy_ref: tuple) -> list[Waypoint]:
    """生成 AABB 网格蛇形寻位路点（连续蛇形，一格一格走）。

    顶点：各轴 ``np.linspace(min, max, 份数 + 1)``（含边界）。
    行走顺序：自 ``(x_min, y_min, z_min)`` 起，任意相邻两路点恰在一个轴
    上相差一格（无斜跳/飞移）：

    * 行内: 保持 Y, Z 不变, X 逐格扫描; 行方向逐行交替
      （全局连续，首行 X_min -> X_max，满足首行方向不变量）;
    * 行间: 保持 X, Z 不变, Y 沿 ``y_dir`` 前进一格;
    * 层末: 保持 X, Y 不变, 垂直上升一个 Z（无 XY 跳变），并翻转
      ``y_dir``，下一层自 Y 边界逐格返回；落点即新层首行端点，
      沿远离落点方向继续扫描（行方向同步取反，保持全局交替）。

    ``y_dir`` 每层翻转（偶数层 y_min -> y_max，奇数层反向）。

    Args:
        x_min/x_max/y_min/y_max/z_min/z_max: AABB 边界（mm）。
        xs/ys/zs: 各轴份数（顶点数 = 份数 + 1）。
        rpy_ref: 参考姿态 (roll, pitch, yaw) deg，赋给全部路点。

    Returns:
        按生成顺序的 Waypoint 列表，index 从 0 递增；总数
        ``(xs+1)*(ys+1)*(zs+1)``，每个网格顶点恰好出现一次。
    """
    xs_v = np.linspace(float(x_min), float(x_max), int(xs) + 1)
    ys_v = np.linspace(float(y_min), float(y_max), int(ys) + 1)
    zs_v = np.linspace(float(z_min), float(z_max), int(zs) + 1)
    rpy_ref = tuple(float(v) for v in rpy_ref)

    waypoints: list[Waypoint] = []

    def _emit(i: int, j: int, k: int) -> None:
        """把网格顶点 (i, j, k) 追加为下一个路点。"""
        waypoints.append(Waypoint(index=len(waypoints),
                                  pos=(float(xs_v[i]), float(ys_v[j]),
                                       float(zs_v[k])),
                                  rpy_ref=rpy_ref))

    # 光标式一格一格走：i/j/k 为当前顶点网格下标，x_dir/y_dir 为行/层方向
    i, j, k = 0, 0, 0                     # 起点 (x_min, y_min, z_min)
    _emit(i, j, k)
    x_dir = 1                             # 行内 X 方向（首行 X_min -> X_max）
    y_dir = 1                             # 层内 Y 方向（每层翻转）
    while True:
        # 1) 行内扫描: 保持 Y, Z 不变, X 沿 x_dir 逐格走到行边界
        while 0 <= i + x_dir <= int(xs):
            i += x_dir
            _emit(i, j, k)
        # 2) 行间前进: 保持 X, Z 不变, Y 沿 y_dir 前进一格; 行方向取反
        if 0 <= j + y_dir <= int(ys):
            j += y_dir
            _emit(i, j, k)
            x_dir = -x_dir
            continue
        # 3) 层间转换: 保持 X, Y 不变, 垂直上升一个 Z（关键：无 XY 跳变）;
        #    Y 方向翻转, 行方向同步取反（落点行沿远离落点方向扫描）
        if k + 1 <= int(zs):
            k += 1
            _emit(i, j, k)
            y_dir = -y_dir
            x_dir = -x_dir
            continue
        break                             # 三个方向全部到界，路径完成
    return waypoints


# ---------------------------------------------------------------------------
# 3D 预览图
# ---------------------------------------------------------------------------
def plot_path(points_xyz, ok_flags, out_png, title) -> None:
    """绘制 3D 路点路径预览图并保存 PNG（Agg 后端，无显示环境）。

    Args:
        points_xyz: Nx3 数组（mm），按路径 index 顺序。
        ok_flags: N 个 bool。True=成功点（按原始路径 index 从深灰到浅灰），
            False=失败点（红）。
        out_png: 输出 PNG 路径。
        title: 图标题（应含 ``成功/总数``，如 ``waypoints 27/27``）。
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pts = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
    flags = np.asarray(ok_flags, dtype=bool)
    if len(flags) != len(pts):
        raise ValueError("ok_flags 长度必须与 points_xyz 路点数一致")
    pts_ok = pts[flags]
    pts_bad = pts[~flags]

    # 成功点颜色按原始路径 index 单调上升：首点深灰、末点浅灰。
    # 必须以完整路径的 index 取色（而非成功点压缩后的序号），这样即使中间
    # 存在红色无解点，灰阶仍准确表示原始行走顺序。
    gray_by_index = np.linspace(0.20, 0.60, len(pts))
    ok_gray_rgb = np.repeat(gray_by_index[flags, np.newaxis], 3, axis=1)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    # 路径线：按 index 顺序连接全部点（含失败点）
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
            linewidth=0.6, c="#7f8c9a", alpha=0.8)
    # 成功点：按路点序号深灰 -> 浅灰；关闭 3D 深度阴影，避免相机距离
    # 再次篡改灰阶含义。失败点始终红色。
    if len(pts_ok):
        ax.scatter(pts_ok[:, 0], pts_ok[:, 1], pts_ok[:, 2],
                   c=ok_gray_rgb, s=28, depthshade=False,
                   edgecolors="#202020", linewidths=0.25)
    if len(pts_bad):
        ax.scatter(pts_bad[:, 0], pts_bad[:, 1], pts_bad[:, 2],
                   c="red", s=48, depthshade=False,
                   edgecolors="#7f0000", linewidths=0.25)
    # 三轴比例按绘图坐标范围设置。Z 的显示边界向上下各扩 20%，
    # 即 z_span 由输入跨度增至 140%，而非对视觉比例直接乘系数。
    xr = max(float(np.ptp(pts[:, 0])), 1e-9)
    yr = max(float(np.ptp(pts[:, 1])), 1e-9)
    z_data_min = float(np.min(pts[:, 2]))
    z_data_max = float(np.max(pts[:, 2]))
    z_data_span = z_data_max - z_data_min
    z_padding = 0.20 * z_data_span
    if z_padding == 0.0:
        z_padding = 1e-9
    z_plot_min = z_data_min - z_padding
    z_plot_max = z_data_max + z_padding
    z_plot_span = z_plot_max - z_plot_min
    ax.set_zlim(z_plot_min, z_plot_max)
    ax.set_box_aspect((xr, yr, z_plot_span))
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(title)
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 链式 IK 选解引擎（迭代 2）
# ---------------------------------------------------------------------------
def config_tag(rbt, T, q) -> tuple[int, int, int]:
    """计算解 ``q``（deg）在位姿 ``T`` 下的构型标签 ``(arm, elbow, wrist)``。

    * wrist: q5 > 0 记 +1，否则 -1（恰为 0 归 -1）;
    * elbow: q3 > 0 记 +1，否则 -1（恰为 0 归 -1）;
    * arm (0/1): 由腕点方位反解 J1 双解 ``phi+beta`` / ``phi+pi-beta``
      （W = p - d6*z_T，phi = atan2(Wy, Wx)，beta = asin(d2/rho)），
      与 q1（wrap 后角度差）更近的候选记 0/1。

    Args:
        rbt: robot_model.nexbot_ik.Nexbot6R 实例（用其 d2 / d6）。
        T: 4x4 目标位姿矩阵。
        q: 6 关节角（deg）。

    Returns:
        (arm, elbow, wrist)，各分量为 0/1 或 ±1。
    """
    T = np.asarray(T, dtype=float)
    q = [float(v) for v in q]

    wrist = 1 if q[5] > 0 else -1
    elbow = 1 if q[2] > 0 else -1

    W = T[:3, 3] - rbt.d6 * T[:3, :3][:, 2]      # 腕点（4/5/6 轴交点）
    phi = math.atan2(W[1], W[0])
    rho = math.hypot(W[0], W[1])
    if rho < 1e-9:
        beta = 0.0                               # 腕点近回转轴，退化
    else:
        beta = math.asin(min(1.0, max(-1.0, rbt.d2 / rho)))
    c0 = math.degrees(phi + beta)                # J1 候选解一
    c1 = math.degrees(phi + math.pi - beta)      # J1 候选解二
    arm = 0 if abs(wrap_180(c0 - q[0])) <= abs(wrap_180(c1 - q[0])) else 1
    return (arm, elbow, wrist)


def solve_chain(rbt, T_list, q_seed, strict: bool = False) -> list[ChainStep]:
    """对位姿序列做链式 IK 选解。

    链式规则：每步用上一**选中**解作 ``q_ref``（首步或连续无解时仍为
    ``q_seed``），对 ``rbt.ik`` 返回的全部解在**度数域**重算
    ``score = Σ wrap_180(Δq)²`` 复核排序（ik 内部在弧度域排序），取最小者；
    最小与前两名差 < 1e-9 时在并列者中选 ``config_tag`` 与上一选中解
    （无则与 q_seed 的 config_tag）相同分量数多者。

    无解路点：告警打印并跳过（``q_prev`` 保持不变），``strict=True`` 时
    打印错误后 ``sys.exit(2)``。选中解做 FK 复验（rot<1e-6, pos<1e-4mm）
    记入 ``ok``。

    Args:
        rbt: robot_model.nexbot_ik.Nexbot6R 实例。
        T_list: 4x4 位姿矩阵序列。
        q_seed: IK 链种子关节角（deg，长度 6）。
        strict: 无解即中止（退出码 2）。

    Returns:
        与 T_list 等长、同序的 ChainStep 列表。
    """
    q_prev = [float(v) for v in q_seed]
    # tie-break 参考构型：上一选中解的构型；尚无选中解时用 q_seed 的构型
    cfg_seed = config_tag(rbt, rbt.fk(q_seed), q_seed)
    last_cfg: tuple | None = None

    steps: list[ChainStep] = []
    for i, T in enumerate(T_list):
        T = np.asarray(T, dtype=float)
        sols = rbt.ik(T, q_ref=q_prev)
        n_sols = len(sols)

        if n_sols == 0:
            print(f"  waypoint #{i} 无解(跳过)")
            if strict:
                print(f"错误: strict 模式下路点 #{i} 无解，中止")
                sys.exit(2)
            steps.append(ChainStep(index=i, q=None, n_sols=0, config=None,
                                    dq_l2=None, dq_l1=None, ok=False))
            continue                       # q_prev / last_cfg 保持不变

        # 度数域 score 复核排序（不直接信任 ik 的弧度域排序）
        scores = [sum(wrap_180(a - b) ** 2 for a, b in zip(sol, q_prev))
                  for sol in sols]
        best = min(scores)
        tied = [k for k, sc in enumerate(scores) if sc - best < 1e-9]

        if len(tied) == 1:
            pick = tied[0]
        else:
            ref_cfg = cfg_seed if last_cfg is None else last_cfg

            def _n_match(sol) -> int:
                """与参考构型相同分量数（越大越好）。"""
                cfg = config_tag(rbt, T, sol)
                return sum(1 for x, y in zip(cfg, ref_cfg) if x == y)

            # 并列者中构型相同分量多者优先，其次 score 小者、先出现者
            pick = min(tied, key=lambda k: (-_n_match(sols[k]), scores[k]))

        q_sel = [float(v) for v in sols[pick]]
        dq_l2 = float(scores[pick])
        dq_l1 = float(sum(abs(wrap_180(a - b))
                          for a, b in zip(q_sel, q_prev)))
        cfg = config_tag(rbt, T, q_sel)

        # FK 复验：fk(q_sel) 是否回到目标位姿 T
        Tq = rbt.fk(q_sel)
        ok = bool(np.max(np.abs(Tq[:3, :3] - T[:3, :3])) < 1e-6 and
                  np.max(np.abs(Tq[:3, 3] - T[:3, 3])) < 1e-4)

        steps.append(ChainStep(index=i, q=q_sel, n_sols=n_sols, config=cfg,
                               dq_l2=dq_l2, dq_l1=dq_l1, ok=ok))
        q_prev = q_sel
        last_cfg = cfg

    return steps


# --------------------------------------------------------------------------
# 姿态可辨识性（条件数）校验
# ---------------------------------------------------------------------------
def rotation_vector(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> SO(3) log 映射的旋转向量（轴 * 角，rad）。

    对小角度扰动，旋转矩阵的对称部分是二阶小量（O(ω²)），直接展平 9 个元素
    做 SVD 会被数值噪声主导（实测条件数 10² 量级且无物理意义）；log 映射
    保留一阶项，得到的是真正反映「三轴激励程度」的 3 维特征。

    Args:
        R: 3x3 旋转矩阵。

    Returns:
        旋转向量 [wx, wy, wz]，模长为旋转角（rad）。
    """
    R = np.asarray(R, dtype=float)
    cos_angle = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.acos(cos_angle)
    skew = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]], dtype=float)
    if angle < 1e-10:
        return 0.5 * skew                       # 小角度: sin(angle) ≈ angle
    if abs(math.pi - angle) < 1e-6:
        # 近 180° 时 sin(angle) -> 0，用对称部分取轴
        sym = (R + np.eye(3)) / 2.0
        diag = np.sqrt(np.clip(np.diag(sym), 0.0, None))
        k = int(np.argmax(diag))
        if diag[k] < 1e-12:
            return np.zeros(3)
        axis = sym[:, k] / diag[k]
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12:
            return np.zeros(3)
        return (axis / norm) * angle
    return skew / (2.0 * math.sin(angle)) * angle


def rotation_vectors(poses, reference: np.ndarray | None = None) -> np.ndarray:
    """姿态序列 -> N x 3 旋转向量矩阵（相对参考姿态）。

    Args:
        poses: 4x4 位姿序列（只用旋转部分）。
        reference: 参考 3x3 旋转；默认取第一个位姿的旋转。

    Returns:
        N x 3 数组，每行为 ``log(R_i @ R_ref.T)``。
    """
    rotations = [np.asarray(T, dtype=float)[:3, :3] for T in poses]
    if not rotations:
        return np.zeros((0, 3))
    ref = rotations[0] if reference is None else np.asarray(reference, dtype=float)
    return np.array([rotation_vector(R @ ref.T) for R in rotations], dtype=float)


def _to_unit_rms(matrix: np.ndarray) -> np.ndarray:
    """按整体 RMS 归一化，使不同量纲的特征块权重可比。"""
    rms = float(np.sqrt(np.mean(matrix ** 2)))
    return matrix / rms if rms > 0.0 else matrix


def pose_condition_number(poses, include_position: bool = False) -> dict:
    """用 SVD 求姿态集（可含位置）的条件数，衡量激励是否充分。

    构造：各姿态相对参考姿态的旋转向量（N x 3）去中心化；
    ``include_position`` 时把位置（N x 3）去中心化后与姿态**等 RMS** 拼接，
    避免 mm 与 rad 的量纲差主导奇异值。

    条件数 ``cond = sigma_max / sigma_min``：接近 1 表示三个转动自由度被均匀
    激励；只绕单轴扰动时 sigma_min -> 0，cond 急剧变大。矩阵秩亏（如全部
    姿态相同）时返回 ``inf``。

    Args:
        poses: 4x4 位姿序列。
        include_position: 是否把位置也计入条件数。

    Returns:
        dict: ``condition_number`` / ``singular_values`` / ``n_points`` /
        ``metric`` / ``include_position``。
    """
    poses = list(poses)
    if not poses:
        return {"condition_number": float("inf"), "singular_values": [],
                "n_points": 0, "metric": "orientation-rotvec-centered",
                "include_position": bool(include_position)}

    feat = rotation_vectors(poses)
    feat = feat - feat.mean(axis=0)                       # 去中心化: 度量离散度
    if include_position:
        pos = np.array([np.asarray(T, dtype=float)[:3, 3] for T in poses])
        pos = pos - pos.mean(axis=0)
        feat = np.hstack([_to_unit_rms(feat), _to_unit_rms(pos)])

    singular = np.linalg.svd(feat, compute_uv=False)
    scale = float(singular[0]) if len(singular) else 0.0
    if scale <= 0.0 or float(singular[-1]) <= scale * 1e-12:
        cond = float("inf")                              # 秩亏 / 无激励
    else:
        cond = float(singular[0] / singular[-1])

    return {"condition_number": cond,
            "singular_values": [float(v) for v in singular],
            "n_points": len(poses),
            "metric": ("orientation+position-equal-rms-centered"
                       if include_position else "orientation-rotvec-centered"),
            "include_position": bool(include_position)}


def _sample_perturbations(waypoints, seed: int, attempt: int) -> None:
    """按给定尝试序号采样姿态扰动并回填路点（原地修改）。

    ``attempt == 0`` 时使用 ``default_rng(seed)``，与既有 ``--seed`` 行为逐位
    一致（保证脚本向后兼容）；后续尝试改用 ``default_rng([seed, attempt])``,
    使每次重采样互相独立且整体可复现。

    Args:
        waypoints: Waypoint 列表，回填 ``delta_rpy`` / ``rpy_pert``。
        seed: 用户随机种子。
        attempt: 0-based 尝试序号（0 = 首次采样）。
    """
    rng = (np.random.default_rng(seed) if attempt == 0
           else np.random.default_rng([int(seed), int(attempt)]))
    ax, ay, az = PERTURB_AMPLITUDE_DEG
    for wp in waypoints:
        delta = (rng.uniform(-ax, ax),
                 rng.uniform(-ay, ay),
                 rng.uniform(-az, az))
        wp.delta_rpy = delta
        wp.rpy_pert = tuple(r + d for r, d in zip(wp.rpy_ref, delta))


def _check_pose_conditioning(args: argparse.Namespace, waypoints,
                             build_poses) -> dict:
    """迭代 4：条件数不达标则重采样姿态扰动（步骤在 IK 之前）。

    Args:
        args: CLI 参数（``--cond-max`` / ``--cond-attempts`` 等）。
        waypoints: Waypoint 列表（会被重采样覆盖）。
        build_poses: 由路点构造位姿序列的可调用对象。

    Returns:
        dict: 条件数报告（含 ``attempts_used`` / ``accepted``）。
    """
    max_attempts = max(1, int(args.cond_attempts))
    best = None
    for attempt in range(max_attempts):
        _sample_perturbations(waypoints, args.seed, attempt)
        info = pose_condition_number(build_poses(waypoints),
                                     include_position=args.cond_position)
        passed = info["condition_number"] <= args.cond_max
        mark = "通过" if passed else "重采样"
        print(f"[条件数] 采样 {attempt + 1}/{max_attempts}: "
              f"cond={info['condition_number']:.3f} "
              f"(σ={np.array2string(np.array(info['singular_values']), precision=3)})"
              f" 阈值 {args.cond_max:g} -> {mark}")
        if best is None or info["condition_number"] < best[1]["condition_number"]:
            best = (attempt, info)
        if passed:
            return {"enabled": True, "metric": info["metric"],
                    "include_position": info["include_position"],
                    "threshold": float(args.cond_max),
                    "attempts_used": attempt + 1, "accepted": True,
                    "condition_number": info["condition_number"],
                    "singular_values": info["singular_values"]}

    # 全部尝试均未达标：保留最优结果并显式告警（绝不静默当作通过）
    attempt, info = best
    print(f"警告: {max_attempts} 次重采样均未达到条件数阈值 {args.cond_max:g}，"
          f"采用最优 cond={info['condition_number']:.3f}（第 {attempt + 1} 次采样）")
    if args.strict:
        print("错误: strict 模式下姿态条件数不达标，中止")
        sys.exit(2)
    _sample_perturbations(waypoints, args.seed, attempt)   # 回填最优采样
    return {"enabled": True, "metric": info["metric"],
            "include_position": info["include_position"],
            "threshold": float(args.cond_max),
            "attempts_used": max_attempts, "accepted": False,
            "condition_number": info["condition_number"],
            "singular_values": info["singular_values"]}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.aabb_snake_scan",
        description="AABB 网格蛇形寻位路点规划工具（CH2000 六轴焊接机器人）。"
                    "路点生成 + 随机姿态扰动（--seed 可复现）+ 姿态条件数 SVD "
                    "前置校验（超阈值则重采样）+ 只对扰动后路点链式 IK 选解；"
                    "--no-perturb 开发验证模式用参考姿态直接跑链。")
    parser.add_argument("x_min", type=float, metavar="X_MIN",
                        help="AABB X 下界 (mm)")
    parser.add_argument("x_max", type=float, metavar="X_MAX",
                        help="AABB X 上界 (mm)")
    parser.add_argument("y_min", type=float, metavar="Y_MIN",
                        help="AABB Y 下界 (mm)")
    parser.add_argument("y_max", type=float, metavar="Y_MAX",
                        help="AABB Y 上界 (mm)")
    parser.add_argument("z_min", type=float, metavar="Z_MIN",
                        help="AABB Z 下界 (mm)")
    parser.add_argument("z_max", type=float, metavar="Z_MAX",
                        help="AABB Z 上界 (mm)")
    parser.add_argument("--rpy", nargs=3, type=float, metavar=("A", "B", "C"),
                        default=list(DEFAULT_RPY),
                        help="参考姿态 roll pitch yaw (deg)，默认 180 0 -90")
    parser.add_argument("--xs", type=int, default=3,
                        help="X 方向份数（顶点数=份数+1），默认 3")
    parser.add_argument("--ys", type=int, default=3,
                        help="Y 方向份数，默认 3")
    parser.add_argument("--zs", type=int, default=3,
                        help="Z 方向份数，默认 3")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF),
                        help="机器人模型路径（.urdf / .json，IK 链使用）")
    parser.add_argument("--q-seed", default=DEFAULT_Q_SEED,
                        help="IK 链种子关节角，逗号分隔 6 个 deg，"
                             "默认 0,20,-20,0,40,0")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="随机种子（默认模式姿态扰动采样使用，"
                             "同 seed 两次运行输出逐字节一致）")
    parser.add_argument("--strict", action="store_true",
                        help="严格模式：路点无解即中止（退出码 2）")
    parser.add_argument("--no-perturb", action="store_true",
                        help="开发验证：用参考姿态（不加扰动）跑 IK 链")
    parser.add_argument("--skip-ik", action="store_true",
                        help="仅路点预览模式：不求解 IK，打印路点表并输出"
                             "预览图与 JSON")
    parser.add_argument("--intrinsic", action="store_true",
                        help="动轴欧拉角模式（extrinsic=False）")
    parser.add_argument("--cond-max", type=float, default=DEFAULT_COND_MAX,
                        help="姿态条件数阈值 σmax/σmin，超过则重新采样扰动，"
                             f"默认 {DEFAULT_COND_MAX:g}")
    parser.add_argument("--cond-attempts", type=int,
                        default=DEFAULT_COND_ATTEMPTS,
                        help="条件数不达标时的最大重采样次数，"
                             f"默认 {DEFAULT_COND_ATTEMPTS}")
    parser.add_argument("--cond-position", action="store_true",
                        help="条件数同时计入位置（与姿态等 RMS 归一化）")
    parser.add_argument("--no-cond-check", action="store_true",
                        help="关闭姿态条件数前置校验（不重采样，仅报告）")
    parser.add_argument("--out", default=DEFAULT_OUT_JSON,
                        help="JSON 输出路径，默认 scan_result.json")
    parser.add_argument("--plot", default=DEFAULT_OUT_PNG,
                        help="PNG 预览图输出路径，默认 scan_path.png")
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """参数校验（各模式共用）：边界大小关系与份数非负。失败则 parser.error 退出。"""
    for axis, lo, hi in (("X", args.x_min, args.x_max),
                         ("Y", args.y_min, args.y_max),
                         ("Z", args.z_min, args.z_max)):
        if lo > hi:
            parser.error(f"{axis} 轴下界 {lo} 大于上界 {hi}")
    for name, n in (("xs", args.xs), ("ys", args.ys), ("zs", args.zs)):
        if n < 0:
            parser.error(f"--{name} 份数需 >= 0，当前为 {n}")
    if args.cond_max < 1.0:
        parser.error(f"--cond-max 必须 >= 1（条件数定义），当前为 {args.cond_max}")
    if args.cond_attempts < 1:
        parser.error(f"--cond-attempts 必须 >= 1，当前为 {args.cond_attempts}")


def _fmt_g(value: float) -> str:
    """浮点紧凑显示：整数值不带小数点（180.0 -> '180'，22.5 -> '22.5'）。"""
    return f"{float(value):.6g}"


def _round6(seq) -> list:
    """把序列内浮点统一 round 到 6 位后输出 list（保证 JSON 可复现）。"""
    return [round(float(v), 6) for v in seq]


def _run_skip_ik(args: argparse.Namespace) -> int:
    """--skip-ik 模式：路点表打印 + 预览图 + JSON 输出。"""
    rpy_ref = tuple(float(v) for v in args.rpy)

    # ---- 路点生成 ----
    waypoints = plan_waypoints(args.x_min, args.x_max, args.y_min, args.y_max,
                               args.z_min, args.z_max,
                               args.xs, args.ys, args.zs, rpy_ref)

    # ---- 路点表打印（坐标 1 位小数；末尾打印总数） ----
    rpy_str = ", ".join(_fmt_g(v) for v in rpy_ref)
    for wp in waypoints:
        print(f"#{wp.index:03d} (x={wp.pos[0]:.1f}, y={wp.pos[1]:.1f}, "
              f"z={wp.pos[2]:.1f}) rpy=({rpy_str})")
    print(f"total waypoints: {len(waypoints)}")

    # ---- 预览图（--skip-ik 模式全部点视为成功） ----
    pts = np.array([wp.pos for wp in waypoints], dtype=float)
    ok_flags = [True] * len(waypoints)
    plot_path(pts, ok_flags, args.plot,
              title=f"waypoints {len(waypoints)}/{len(waypoints)}")

    # ---- JSON 输出（浮点统一 round 6 位，UTF-8） ----
    payload = {
        "meta": {
            "aabb": _round6([args.x_min, args.x_max,
                             args.y_min, args.y_max,
                             args.z_min, args.z_max]),
            "rpy": _round6(rpy_ref),
            "xs": int(args.xs),
            "ys": int(args.ys),
            "zs": int(args.zs),
            "seed": int(args.seed),
            "skip_ik": bool(args.skip_ik),
        },
        "waypoints": [
            {
                "index": wp.index,
                "pos": _round6(wp.pos),
                "rpy_ref": _round6(wp.rpy_ref),
                "rpy_pert": None if wp.rpy_pert is None else _round6(wp.rpy_pert),
                "delta_rpy": None if wp.delta_rpy is None else _round6(wp.delta_rpy),
            }
            for wp in waypoints
        ],
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return 0


def _parse_q_seed(parser: argparse.ArgumentParser, raw) -> list[float]:
    """解析 --q-seed：逗号分隔 6 个浮点（deg）。格式错则 parser.error。"""
    try:
        vals = [float(v) for v in str(raw).split(",")]
    except ValueError:
        parser.error(f"--q-seed 需逗号分隔 6 个浮点数（如 0,20,-20,0,40,0），"
                     f"当前为 {raw!r}")
    if len(vals) != 6:
        parser.error(f"--q-seed 需恰好 6 个浮点数（当前 {len(vals)} 个）: {raw!r}")
    return vals


def _run_ik_chain(args: argparse.Namespace,
                  parser: argparse.ArgumentParser) -> int:
    """IK 链主流程（迭代 3 最终形态，--skip-ik 之外的模式）。"""
    q_seed = _parse_q_seed(parser, args.q_seed)

    # ---- 1. 加载机器人模型（--urdf，链/DH 交叉校验在类内完成） ----
    rbt = Nexbot6R(args.urdf)

    # ---- 2. 路点生成（同迭代 1） ----
    rpy_ref = tuple(float(v) for v in args.rpy)
    waypoints = plan_waypoints(args.x_min, args.x_max, args.y_min, args.y_max,
                               args.z_min, args.z_max,
                               args.xs, args.ys, args.zs, rpy_ref)

    # ---- 3. 姿态扰动采样 + 姿态可辨识性（条件数）前置校验 ----
    def _poses_of(rpy_getter):
        """由路点姿态构造位姿序列（extrinsic 由 --intrinsic 控制）。"""
        return [rbt.pose_to_T(wp.pos, rpy_getter(wp),
                              extrinsic=not args.intrinsic)
                for wp in waypoints]

    def _ref_rpy(wp):
        return wp.rpy_ref

    def _pert_rpy(wp):
        return wp.rpy_pert

    conditioning = {"enabled": False, "metric": None,
                    "include_position": bool(args.cond_position),
                    "threshold": float(args.cond_max), "attempts_used": 0,
                    "accepted": None, "condition_number": None,
                    "singular_values": []}
    if args.no_perturb:
        # 开发验证模式：参考姿态直接建位姿跑链（不采样扰动，字段保持 None）
        poses = _poses_of(_ref_rpy)
        print("[条件数] 跳过: --no-perturb 下姿态全部相同，条件数退化")
    elif args.no_cond_check:
        # 关闭校验：仍采样一次并报告条件数，便于人工判断，但不重采样
        _sample_perturbations(waypoints, args.seed, 0)
        info = pose_condition_number(_poses_of(_pert_rpy),
                                     include_position=args.cond_position)
        print(f"[条件数] 已关闭校验: cond={info['condition_number']:.3f} "
              f"(σ={np.array2string(np.array(info['singular_values']), precision=3)})")
        conditioning.update(metric=info["metric"], attempts_used=1,
                            condition_number=info["condition_number"],
                            singular_values=info["singular_values"])
        poses = _poses_of(_pert_rpy)
    else:
        # 条件数只依赖位姿，必须先于 IK 校验：不达标就重采样扰动，
        # 避免把 8 分支解析 IK + 限位过滤 + FK 复验的算力浪费在废数据上。
        conditioning = _check_pose_conditioning(
            args, waypoints, lambda _wps: _poses_of(_pert_rpy))
        poses = _poses_of(_pert_rpy)

    # ---- 4. 链式 IK 选解 ----
    chain = solve_chain(rbt, poses, q_seed, strict=args.strict)

    # ---- 5. 链结果表 ----
    for s in chain:
        if s.q is None:
            print(f"#{s.index:03d} q=None cfg=None dqL2=None "
                  f"nSols={s.n_sols} [FAIL]")
        else:
            q_str = ", ".join(f"{v:8.3f}" for v in s.q)
            print(f"#{s.index:03d} q=[{q_str}] cfg={s.config} "
                  f"dqL2={s.dq_l2:.2f} nSols={s.n_sols}")

    # ---- 6. 汇总（成功率 / 构型切换 / 累计关节变化） ----
    ok_steps = [s for s in chain if s.ok]
    n_switch = sum(1 for a, b in zip(ok_steps, ok_steps[1:])
                   if a.config != b.config)
    sum_dq_l2 = float(sum(s.dq_l2 for s in ok_steps))
    print(f"成功 {len(ok_steps)}/{len(chain)}")
    print(f"构型切换次数 {n_switch}")
    print(f"Σ dqL2 = {sum_dq_l2:.2f}")
    if conditioning["condition_number"] is not None:
        if conditioning["accepted"] is None:
            state = "未校验"
        elif conditioning["accepted"]:
            state = "通过"
        else:
            state = "未达标"
        print(f"姿态条件数 cond = {conditioning['condition_number']:.3f} "
              f"(阈值 {conditioning['threshold']:g}, "
              f"重采样 {conditioning['attempts_used']} 次, {state})")

    # ---- 7. 预览图（成功深灰 / 失败红，迭代 1 的 plot_path 复用） ----
    pts = np.array([wp.pos for wp in waypoints], dtype=float)
    plot_path(pts, [s.ok for s in chain], args.plot,
              title=f"IK chain {len(ok_steps)}/{len(chain)}")

    # ---- 8. JSON 输出（最终结构：meta/waypoints/perturbed_ik；
    #      浮点统一 round 6，同 seed 两次运行逐字节一致） ----
    payload = {
        "meta": {
            "aabb": _round6([args.x_min, args.x_max,
                             args.y_min, args.y_max,
                             args.z_min, args.z_max]),
            "rpy": _round6(rpy_ref),
            "xs": int(args.xs),
            "ys": int(args.ys),
            "zs": int(args.zs),
            "seed": int(args.seed),
            "skip_ik": bool(args.skip_ik),
            "no_perturb": bool(args.no_perturb),
            "perturbed": not bool(args.no_perturb),
        },
        "waypoints": [
            {
                "index": wp.index,
                "pos": _round6(wp.pos),
                "rpy_ref": _round6(wp.rpy_ref),
                "rpy_pert": None if wp.rpy_pert is None else _round6(wp.rpy_pert),
                "delta_rpy": None if wp.delta_rpy is None else _round6(wp.delta_rpy),
            }
            for wp in waypoints
        ],
        "conditioning": {
            "enabled": bool(conditioning["enabled"]),
            "metric": conditioning["metric"],
            "include_position": bool(conditioning["include_position"]),
            "threshold": round(float(conditioning["threshold"]), 6),
            "attempts_used": int(conditioning["attempts_used"]),
            "accepted": conditioning["accepted"],
            "condition_number": (None if conditioning["condition_number"] is None
                                 else round(float(conditioning["condition_number"]), 6)),
            "singular_values": _round6(conditioning["singular_values"]),
        },
        "perturbed_ik": [
            {
                "index": s.index,
                "q": None if s.q is None else _round6(s.q),
                "config": None if s.config is None
                         else [int(c) for c in s.config],
                "dq_l2": None if s.dq_l2 is None else round(s.dq_l2, 6),
                "dq_l1": None if s.dq_l1 is None else round(s.dq_l1, 6),
                "n_sols": int(s.n_sols),
                "ok": bool(s.ok),
            }
            for s in chain
        ],
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return 0


def main(argv: list[str] | None = None) -> int:
    """CLI 入口。返回进程退出码。"""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)

    if args.skip_ik:
        # --skip-ik 仅路点预览模式（迭代 1，行为保持不变，优先级最高）
        return _run_skip_ik(args)

    # ===== [迭代 2+3] 链式 IK 选解（默认模式: 先按 --seed 采样姿态扰动，
    # ===== 只对扰动路点求 IK; --no-perturb: 参考姿态直接跑链）       =====
    return _run_ik_chain(args, parser)


if __name__ == "__main__":
    sys.exit(main())
