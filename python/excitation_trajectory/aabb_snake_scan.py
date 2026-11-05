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
      ``delta = (U(-5,5), U(-20,20), U(-15,15))`` deg（对应 rx/ry/rz），
      采样顺序与路点 index 一致（同 ``--seed`` 可复现），回填每个路点的
      ``delta_rpy`` 与 ``rpy_pert = rpy_ref + delta``（不 wrap）;
    * 默认流程只对扰动后路点求 IK（未扰动位姿不求）; ``--no-perturb``
      保持开发验证行为（参考姿态直接跑链，不采样扰动，扰动字段保持 None）;
    * JSON 最终结构: ``ik_chain`` 节改名 ``perturbed_ik``（字段不变）;
      ``meta`` 增加 ``perturbed`` 标记（默认 true / --no-perturb false）;
      同 seed 两次运行输出逐字节一致（浮点统一 round 6，无时间戳/随机序）。

迭代 4（修订）—— 姿态激励条件数（一次性设计检查）:
    * 这是**姿态覆盖度诊断**，度量 ``cond(X)=sigma_max/sigma_min``（X 为
      N x 3 激励设计矩阵）；不是 DH 参数辨识回归矩阵的条件数（后者依赖
      关节角，需在 IK 之后另行计算）;
    * ``design_condition_number()``: 方案级、**与随机种子无关**的设计条件数。
      各路点扰动独立均匀，切空间协方差
      ``Σ = J·diag(a²/3)·Jᵀ``（J 为旋转向量对 rpy 的雅可比），
      ``XᵀX ≈ N·Σ``，故 ``cond(X)=sqrt(λmax(Σ)/λmin(Σ))``；轴对齐姿态下
      ``≈ a_max/a_min``（默认 (5,20,15) -> ≈4.0，(10,10,10) -> 1.0）;
    * ``pose_condition_number()``: 实测值**分块**报告（姿态 rad / 位置 mm），
      每块给出各自条件数与去中心化后的绝对 RMS；``--cond-position`` 时附加
      的合并 6D 值经等 RMS 归一化，**只作诊断**，不作 6D 激励证明;
    * 门禁 = ``max(分块条件数)``；超过 ``--cond-max`` 时打印可执行建议，
      ``--strict`` 才 ``sys.exit(2)``。**不再**「超阈值就重采样扰动」——
      重采样不改变三轴幅度比，只在同一分布内挑样本噪声;
    * 参考姿态统一取标称 RPY（``--rpy``），消除「参考=第一个路点」的顺序依赖;
    * ``--perturb-deg A B C`` 显式指定三轴扰动半宽（deg）;
    * ``--no-perturb`` / ``--skip-ik`` 下姿态完全相同，实测度量退化，
      状态记 ``not_run``（设计值仍报告，不打印误导性数字）。

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

from robot_model.nexbot_ik import Nexbot6R, euler_to_rot, wrap_180

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
# 姿态激励条件数阈值 cond(X)=σmax/σmin。该值语义等价于
# cond(XᵀX)=81（约损失 1 位有效数字）。条件数由三轴**幅度比**决定，
# 轴对齐姿态下解析极限 ≈ a_max/a_min（默认 5:20:15 -> ≈4），
# 与随机种子无关；取 9 可接受该幅度比，同时仍能拒绝缺轴激励
# （双轴 ≈ 39，单轴/无激励 = inf）。
DEFAULT_COND_MAX = 9.0

# 姿态扰动幅度（deg，对应 rx/ry/rz 的 ± 半宽）。可由 --perturb-deg 覆盖。
PERTURB_AMPLITUDE_DEG = (5.0, 20.0, 15.0)


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

    数值处理：旋转角用 ``atan2(‖skew‖/2, (tr R - 1)/2)`` 求（小角度下相对
    误差 ~1e-16；``acos`` 形式在 1e-10~1e-6 rad 区间因 cos 的舍入误差
    相对误差可达 1e-4），再用 ``skew/2 * (θ/sinθ)`` 还原模长，故相对误差
    在整个 ``[0, π)`` 区间 ≤1e-9。近 π 时 ``sinθ -> 0`` 改用对称部分取轴。

    Args:
        R: 3x3 旋转矩阵。

    Returns:
        旋转向量 [wx, wy, wz]，模长为旋转角（rad）。
    """
    R = np.asarray(R, dtype=float)
    skew = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]], dtype=float)
    sin_angle = 0.5 * float(np.linalg.norm(skew))
    cos_angle = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.atan2(sin_angle, cos_angle)
    if angle < 1e-12 or sin_angle <= 0.0:
        return 0.5 * skew                       # θ/sinθ -> 1（一阶项）
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
    return 0.5 * skew * (angle / sin_angle)


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


def _condition_of(matrix: np.ndarray) -> tuple:
    """对特征矩阵求 ``(cond, 降序奇异值)``；秩亏 / 无激励 -> ``inf``。"""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.size == 0 or matrix.shape[0] == 0:
        return float("inf"), []
    singular = np.linalg.svd(matrix, compute_uv=False)
    values = [float(v) for v in singular]
    scale = values[0] if values else 0.0
    if (not math.isfinite(scale) or scale <= 0.0
            or values[-1] <= scale * 1e-12):
        return float("inf"), values
    return values[0] / values[-1], values


def _center(matrix: np.ndarray) -> np.ndarray:
    """按列去中心化（条件数只度量离散度，与均值无关）。"""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape[0] == 0:
        return matrix
    return matrix - matrix.mean(axis=0)


def _block_report(values: np.ndarray) -> dict:
    """单块（姿态或位置）的特征报告：绝对 RMS + 条件数 + 奇异值。

    ``rms`` 是去中心化后的**绝对** RMS（姿态 rad、位置 mm），用于暴露绝对
    幅度；等 RMS 归一化会抹掉这一信息，所以不能只看合并矩阵的条件数。
    """
    if np.asarray(values).size == 0:
        return {"rms": 0.0, "condition_number": float("inf"),
                "singular_values": []}
    centered = _center(np.asarray(values, dtype=float))
    rms = float(np.sqrt(np.mean(centered ** 2)))
    cond, singular = _condition_of(centered)
    return {"rms": rms, "condition_number": cond,
            "singular_values": singular}


def pose_condition_number(poses, include_position: bool = False,
                          reference: np.ndarray | None = None) -> dict:
    """用 SVD 求姿态集（可含位置）的条件数，衡量姿态覆盖度是否充分。

    分块报告：姿态块（相对参考姿态的旋转向量，rad）与位置块（mm）各自独立
    去中心化后求条件数，并给出**绝对 RMS** 以暴露绝对幅度。``include_position``
    时顶层条件数是「等 RMS 归一化拼接」的 6D 结果——归一化抹掉了绝对幅度，
    因此**只能当诊断**（``combined_is_diagnostic_only``），不作 6D 激励证明。

    条件数 ``cond = sigma_max / sigma_min``：接近 1 表示三个自由度被均匀
    激励；只绕单轴扰动时 sigma_min -> 0，cond 急剧变大。矩阵秩亏（如全部
    姿态相同）时返回 ``inf``。

    Args:
        poses: 4x4 位姿序列。
        include_position: 是否附加位置块与合并 6D 诊断值。
        reference: 参考 3x3 旋转（应传标称参考姿态）。None 时退回
            ``poses[0]`` 的旋转（旧行为，``reference_source="first_pose"``），
            以便调用方显式消除「参考=第一个路点」的顺序依赖。

    Returns:
        dict: ``condition_number`` / ``singular_values`` / ``n_points`` /
        ``metric`` / ``include_position`` / ``blocks`` /
        ``reference_source`` / ``convention`` / ``combined_is_diagnostic_only``。
    """
    poses = list(poses)
    reference_source = "first_pose" if reference is None else "explicit"
    blocks = {"orientation": {"rms": 0.0,
                              "condition_number": float("inf"),
                              "singular_values": []},
              "position": None}
    if not poses:
        return {"condition_number": float("inf"), "singular_values": [],
                "n_points": 0, "metric": "orientation-rotvec-centered",
                "include_position": bool(include_position),
                "blocks": blocks,
                "reference_source": reference_source,
                "convention": "cond(X)=sigma_max/sigma_min",
                "combined_is_diagnostic_only": bool(include_position)}

    rotvec = rotation_vectors(poses, reference)
    blocks["orientation"] = _block_report(rotvec)
    feature = _center(rotvec)
    if include_position:
        pos = np.array([np.asarray(T, dtype=float)[:3, 3] for T in poses])
        blocks["position"] = _block_report(pos)
        feature = np.hstack([_to_unit_rms(feature),
                             _to_unit_rms(_center(pos))])

    cond, singular = _condition_of(feature)
    return {"condition_number": cond,
            "singular_values": singular,
            "n_points": len(poses),
            "metric": ("orientation+position-equal-rms-combined"
                       if include_position else "orientation-rotvec-centered"),
            "include_position": bool(include_position),
            "blocks": blocks,
            "reference_source": reference_source,
            "convention": "cond(X)=sigma_max/sigma_min",
            "combined_is_diagnostic_only": bool(include_position)}


def design_condition_number(amplitudes_deg, reference_rpy,
                            order: str = "XYZ",
                            extrinsic: bool = True) -> dict:
    """方案级（与随机种子无关）的姿态激励设计条件数。

    原理：各路点扰动独立均匀 ``δ_i ~ U(-a_i, a_i)``，故
    ``diag(a_rx², a_ry², a_rz²) / 3`` 即切空间协方差；旋转向量对 rpy 的
    雅可比 ``J = ∂ω/∂rpy``（在 ``reference_rpy`` 处中心差分，步长 1e-4 deg，
    ``ω = log(R(rpy)·R_refᵀ)`` 为**相对**参考姿态的旋转向量）把该协方差
    映射到 ``Σ = J·diag(a²/3)·Jᵀ``。设计矩阵 X（N x 3）满足
    ``XᵀX ≈ N·Σ``，因此 ``cond(X) = sqrt(λmax(Σ)/λmin(Σ))``。

    必须用相对旋转向量（而不是全局 ``log R``）：默认参考姿态 (180,0,-90)
    本身就在 ``log`` 的 π 奇异点附近，全局形式会得到无意义的巨大奇异值。

    该值与 seed 及采样次数无关，只由三轴**幅度比**决定：轴对齐姿态下 J
    正交（至多差一个旋转/反射），故 ``cond ≈ a_max/a_min``，
    如默认 (5, 20, 15) -> ≈4.0、(10, 10, 10) -> 1.0；参考姿态不在轴对齐
    （如 pitch ≠ 0）时 rpy 三轴在切空间不正交，cond 会偏离该比值。

    Args:
        amplitudes_deg: 三轴扰动半宽 (a_rx, a_ry, a_rz)，deg。
        reference_rpy: 标称参考姿态 (roll, pitch, yaw)，deg。
        order: 欧拉角顺序（与 ``euler_to_rot`` 一致）。
        extrinsic: True=定轴 RPY（默认），False=动轴。

    Returns:
        dict: ``condition_number`` / ``singular_values``（sqrt(λmax..λmin)
        降序）/ ``amplitudes_deg`` / ``reference_rpy`` / ``convention``。
    """
    amp = np.array([float(v) for v in amplitudes_deg], dtype=float)
    rpy0 = np.array([float(v) for v in reference_rpy], dtype=float)
    ref_rot = euler_to_rot(rpy0, order=order, extrinsic=extrinsic)
    step = 1e-4                                    # deg（中心差分步长）
    jac = np.zeros((3, 3), dtype=float)
    for axis in range(3):
        delta = np.zeros(3, dtype=float)
        delta[axis] = step
        rot_plus = euler_to_rot(rpy0 + delta, order=order, extrinsic=extrinsic)
        rot_minus = euler_to_rot(rpy0 - delta, order=order, extrinsic=extrinsic)
        w_plus = rotation_vector(rot_plus @ ref_rot.T)      # 相对参考姿态
        w_minus = rotation_vector(rot_minus @ ref_rot.T)
        jac[:, axis] = (w_plus - w_minus) / (2.0 * step)

    cov = jac @ np.diag(amp ** 2 / 3.0) @ jac.T
    eigvals = np.clip(np.linalg.eigvalsh(cov), 0.0, None)
    singular = np.sqrt(np.sort(eigvals)[::-1])
    scale = float(singular[0]) if len(singular) else 0.0
    if (not math.isfinite(scale) or scale <= 0.0
            or float(singular[-1]) <= scale * 1e-9):
        cond = float("inf")                        # 缺轴激励 / 退化
    else:
        cond = float(singular[0] / singular[-1])
    return {"condition_number": cond,
            "singular_values": [float(v) for v in singular],
            "amplitudes_deg": [float(v) for v in amplitudes_deg],
            "reference_rpy": [float(v) for v in reference_rpy],
            "convention": "cond(X)=sigma_max/sigma_min"}


def _extrinsic_of(args: argparse.Namespace) -> bool:
    """解析后的欧拉约定 -> ``extrinsic`` 旗标（``--validate-args`` 已归一化）。"""
    return getattr(args, "euler_convention", "xyz-extrinsic") == "xyz-extrinsic"


def _reference_rotation(args: argparse.Namespace,
                        reference_rpy) -> np.ndarray:
    """标称参考姿态（``--rpy``）的旋转矩阵，与 ``--euler`` 约定一致。"""
    return euler_to_rot(tuple(float(v) for v in reference_rpy),
                        order="XYZ",
                        extrinsic=_extrinsic_of(args))


def _fmt_cond(value) -> str:
    """条件数显示：None -> 'n/a'，非有限 -> 'inf'，其余 3 位小数。"""
    if value is None:
        return "n/a"
    number = float(value)
    if not math.isfinite(number):
        return "inf" if number > 0 else "-inf"
    return f"{number:.3f}"


def _sample_perturbations(waypoints, seed: int, amplitudes_deg=None) -> None:
    """按三轴半宽采样姿态扰动并回填路点（原地修改）。

    对每个路点独立同分布采样::

        delta = (U(-a_rx, a_rx), U(-a_ry, a_ry), U(-a_rz, a_rz))  deg

    采样顺序与路点 index 一致，``np.random.default_rng(seed)`` 保证同一
    ``--seed`` 逐位可复现（与旧版首次采样完全一致）。

    Args:
        waypoints: Waypoint 列表，回填 ``delta_rpy`` / ``rpy_pert``。
        seed: 用户随机种子。
        amplitudes_deg: 三轴半宽 (a_rx, a_ry, a_rz) deg；None 时取模块常量
            ``PERTURB_AMPLITUDE_DEG``（保持旧行为）。
    """
    rng = np.random.default_rng(seed)
    ax, ay, az = (PERTURB_AMPLITUDE_DEG if amplitudes_deg is None
                  else tuple(float(v) for v in amplitudes_deg))
    for wp in waypoints:
        delta = (rng.uniform(-ax, ax),
                 rng.uniform(-ay, ay),
                 rng.uniform(-az, az))
        wp.delta_rpy = delta
        wp.rpy_pert = tuple(r + d for r, d in zip(wp.rpy_ref, delta))


def evaluate_conditioning(args: argparse.Namespace, waypoints, reference_rpy,
                          poses) -> dict:
    """一次性姿态激励条件数检查 + 门禁判定（一次性，无循环重试）。

    步骤：
      1. 解析设计 cond（``design_condition_number``，与 seed 无关）；
      2. 实测 cond（``pose_condition_number``，分块 + 位置块可选）；
      3. 门禁 = ``max(分块 cond)``：姿态块恒参与，``--cond-position`` 时
         位置块也参与；合并 6D 值只报告、不参与门禁（等 RMS 归一化抹掉
         绝对幅度，不能作为 6D 激励的证明）；
      4. 门禁 > ``--cond-max`` 时打印可执行建议，``--strict`` 才退出码 2；
         ``--no-cond-check`` 只报告（``status="reported_only"``）；
         ``--no-perturb`` 不运行实测（``status="not_run"``）。

    Args:
        args: CLI 参数（``cond_max`` / ``cond_position`` / ``no_cond_check`` /
            ``no_perturb`` / ``strict`` / ``intrinsic`` / ``perturb_deg``）。
        waypoints: Waypoint 列表（仅用于报告路点数）。
        reference_rpy: 标称参考姿态 (roll, pitch, yaw) deg。
        poses: 实测位姿序列（``--no-perturb`` 时可为参考姿态位姿）；
            ``None`` 表示无实测位姿（如 ``--skip-ik`` 预览模式），只报设计值。

    Returns:
        dict: ``status`` / ``convention`` / ``gate``（``value`` + ``source``）/
        ``threshold`` / ``design`` / ``realized`` / ``post_ik``。
        ``status`` 取 ``accepted|rejected|reported_only|disabled|not_run``
        （``disabled`` 保留给程序化调用显式关闭检查的场景）。
    """
    amplitudes = [float(v) for v in
                  (getattr(args, "perturb_deg", None) or PERTURB_AMPLITUDE_DEG)]
    reference_rpy = [float(v) for v in reference_rpy]
    threshold = float(args.cond_max)
    include_position = bool(args.cond_position)
    design = design_condition_number(amplitudes, reference_rpy,
                                     order="XYZ",
                                     extrinsic=_extrinsic_of(args))
    report = {"status": "not_run",
              "convention": "cond(X)=sigma_max/sigma_min",
              "gate": {"value": None, "source": "design(orientation)"},
              "threshold": threshold,
              "design": design,
              "realized": None,
              "post_ik": None}

    print(f"[条件数] 设计值（幅度比决定，与 seed 无关）: "
          f"cond={_fmt_cond(design['condition_number'])} "
          f"幅度 {amplitudes[0]:g}/{amplitudes[1]:g}/{amplitudes[2]:g} deg, "
          f"参考 rpy=({reference_rpy[0]:g}, {reference_rpy[1]:g}, "
          f"{reference_rpy[2]:g}), 路点 {len(waypoints)} 个")

    if getattr(args, "no_perturb", False):
        print("[条件数] 未运行: --no-perturb 下姿态全部相同，条件数退化")
        return report

    if poses is None:
        print("[条件数] 未运行: 无实测位姿（仅路点预览模式）")
        return report

    realized = pose_condition_number(
        poses, include_position=include_position,
        reference=_reference_rotation(args, reference_rpy))
    report["realized"] = realized

    # 门禁用与 seed **无关**的量：姿态块取解析设计值（总体条件数），位置块
    # （AABB 网格确定）取块内值。实测值只是样本估计，同一方案不同 seed 会
    # 在总体值附近波动，用它做门禁会导致“同方案有的 seed 过、有的不过”。
    # 但**秩亏**不是噪声：实测姿态块为 inf（路点过少 / 姿态全同）时
    # 设计值再小也不可接受，必须直接判失败。
    blocks = realized["blocks"]
    realized_orientation = blocks["orientation"]["condition_number"]
    gate_conds = [("design(orientation)", design["condition_number"])]
    position_block = blocks.get("position")
    if include_position and position_block is not None:
        gate_conds.append(("position_block", position_block["condition_number"]))
    gate_source, gate = max(gate_conds, key=lambda item: item[1])
    if not math.isfinite(realized_orientation):
        gate_source, gate = "realized_rank_deficient", float("inf")
    report["gate"]["value"] = gate
    report["gate"]["source"] = gate_source

    position_text = ""
    if position_block is not None:
        position_text = (f", 位置 cond={_fmt_cond(position_block['condition_number'])}"
                         f" (rms={position_block['rms']:.6g} mm)")
    print(f"[条件数] 实测（样本估计，仅诊断）: "
          f"姿态 cond={_fmt_cond(blocks['orientation']['condition_number'])}"
          f" (rms={blocks['orientation']['rms']:.6g} rad)"
          f"{position_text}"
          + ("" if not include_position else
             f"；合并 6D 诊断值 cond={_fmt_cond(realized['condition_number'])}"
             f"（等 RMS，仅诊断，不参与门禁）"))
    print(f"[条件数] 门禁 = {report['gate']['source']} = {_fmt_cond(gate)}，"
          f"阈值 {threshold:g}（与 seed 无关）")

    if getattr(args, "no_cond_check", False):
        report["status"] = "reported_only"
        print("[条件数] --no-cond-check: 仅报告，不作为门禁")
        return report

    if math.isfinite(gate) and gate <= threshold:
        report["status"] = "accepted"
        return report

    a_max = max(amplitudes)
    a_min = min(amplitudes)
    report["status"] = "rejected"
    print(f"[条件数] 未达标: 门禁 {_fmt_cond(gate)} > 阈值 {threshold:g}")
    if not math.isfinite(gate):
        if gate_source == "realized_rank_deficient":
            print(f"[条件数] 实测姿态块秩亏（cond=inf）：路点过少（当前 "
                  f"{len(waypoints)} 个）或姿态完全相同，设计值不适用")
        else:
            print("[条件数] 门禁为 inf（秩亏）：检查份数是否过少、"
                  "参考姿态是否使某轴退化；此时幅度比建议不适用")
    elif gate_source == "design(orientation)":
        print(f"需要 a_min >= a_max/cond_max = {a_max / threshold:.2f} deg"
              f"（当前 {a_min:g}）；可用 --perturb-deg 调整或提高 --cond-max")
    if getattr(args, "strict", False):
        print("错误: strict 模式下姿态激励条件数不达标，中止")
        sys.exit(2)
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.aabb_snake_scan",
        description="AABB 网格蛇形寻位路点规划工具（CH2000 六轴焊接机器人）。"
                    "路点生成 + 随机姿态扰动（--seed 可复现）+ 姿态激励条件数 "
                    "一次性设计检查（cond=σmax/σmin，分块报告，一次性判定）+ "
                    "只对扰动后路点链式 IK 选解；"
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
    parser.add_argument("--euler", choices=("xyz-extrinsic", "xyz-intrinsic"),
                        default=None, metavar="CONVENTION",
                        help="欧拉角约定：xyz-extrinsic=定轴 RPY（Rz·Ry·Rx，"
                             "默认）或 xyz-intrinsic=动轴（Rx·Ry·Rz，Nexbot "
                             "控制器链路）；显式给出时覆盖 --intrinsic")
    parser.add_argument("--intrinsic", action="store_true",
                        help="[已废弃] 等价于 --euler xyz-intrinsic；为保持"
                             "兼容保留，显式 --euler 优先")
    parser.add_argument("--cond-max", type=float, default=DEFAULT_COND_MAX,
                        help="姿态激励条件数阈值 cond(X)=σmax/σmin（门禁取"
                             " max(分块 cond)），超过则拒绝并给出可执行建议，"
                             f"默认 {DEFAULT_COND_MAX:g}；需有限且 >= 1")
    parser.add_argument("--perturb-deg", nargs=3, type=float,
                        metavar=("A_RX", "A_RY", "A_RZ"), default=None,
                        help="三轴姿态扰动半宽 (deg)，默认 "
                             f"{PERTURB_AMPLITUDE_DEG}")
    parser.add_argument("--cond-position", action="store_true",
                        help="额外报告位置块条件数与合并 6D 诊断值"
                             "（合并值经等 RMS 归一化，不参与门禁）")
    parser.add_argument("--no-cond-check", action="store_true",
                        help="条件数只报告不门禁（status=reported_only）")
    parser.add_argument("--out", default=DEFAULT_OUT_JSON,
                        help="JSON 输出路径，默认 scan_result.json")
    parser.add_argument("--plot", default=DEFAULT_OUT_PNG,
                        help="PNG 预览图输出路径，默认 scan_path.png")
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """参数校验（各模式共用）：边界/份数/欧拉约定/门禁阈值/扰动幅度。失败则 parser.error 退出。"""
    # 欧拉约定归一化：显式 --euler 优先，否则看 --intrinsic，默认定轴 RPY。
    if args.euler is not None:
        args.euler_convention = args.euler
    else:
        args.euler_convention = ("xyz-intrinsic" if args.intrinsic
                                 else "xyz-extrinsic")
    for axis, lo, hi in (("X", args.x_min, args.x_max),
                         ("Y", args.y_min, args.y_max),
                         ("Z", args.z_min, args.z_max)):
        if lo > hi:
            parser.error(f"{axis} 轴下界 {lo} 大于上界 {hi}")
    for name, n in (("xs", args.xs), ("ys", args.ys), ("zs", args.zs)):
        if n < 0:
            parser.error(f"--{name} 份数需 >= 0，当前为 {n}")
    if not math.isfinite(args.cond_max):
        parser.error(f"--cond-max 必须为有限值（拒绝 inf/nan），当前为 {args.cond_max}")
    if args.cond_max < 1.0:
        parser.error(f"--cond-max 必须 >= 1（条件数定义），当前为 {args.cond_max}")

    amplitudes = (list(PERTURB_AMPLITUDE_DEG) if args.perturb_deg is None
                  else [float(v) for v in args.perturb_deg])
    for name, value in zip(("A_RX", "A_RY", "A_RZ"), amplitudes):
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--perturb-deg {name} 必须为有限正数（deg），"
                         f"当前为 {value}")
    args.perturb_deg = amplitudes            # 归一化为 list[float]，下游直接用


def _fmt_g(value: float) -> str:
    """浮点紧凑显示：整数值不带小数点（180.0 -> '180'，22.5 -> '22.5'）。"""
    return f"{float(value):.6g}"


def _round6(seq) -> list:
    """把序列内浮点统一 round 到 6 位后输出 list（保证 JSON 可复现）。"""
    return [round(float(v), 6) for v in seq]


def _finite_or_none(value, ndigits: int = 6):
    """JSON 浮点：非有限值（inf/nan）一律写 None，绝不输出 Infinity。"""
    if value is None:
        return None
    number = float(value)
    return round(number, ndigits) if math.isfinite(number) else None


def _finite_list(values, ndigits: int = 6) -> list:
    """逐个套用 ``_finite_or_none``，保持原顺序。"""
    return [_finite_or_none(v, ndigits) for v in values]


def _block_to_json(block) -> dict | None:
    """单个特征块（姿态/位置）的 JSON 形式（非有限值 -> null）。"""
    if block is None:
        return None
    return {"rms": _finite_or_none(block.get("rms")),
            "condition_number": _finite_or_none(block.get("condition_number")),
            "singular_values": _finite_list(block.get("singular_values") or [])}


def _realized_to_json(report) -> dict | None:
    """实测条件数（``pose_condition_number`` 返回）的 JSON 形式。"""
    if report is None:
        return None
    blocks = report.get("blocks") or {}
    return {"condition_number": _finite_or_none(report.get("condition_number")),
            "singular_values": _finite_list(report.get("singular_values") or []),
            "n_points": int(report.get("n_points", 0)),
            "blocks": {"orientation": _block_to_json(blocks.get("orientation")),
                       "position": _block_to_json(blocks.get("position"))},
            "metric": report.get("metric"),
            "reference_source": report.get("reference_source")}


def _conditioning_to_json(report: dict) -> dict:
    """把 ``evaluate_conditioning`` 的返回整理成统一 JSON 节。

    所有非有限值一律写 ``null``（配合 ``json.dump(..., allow_nan=False)``，
    输出文件内不会出现 ``Infinity`` / ``NaN``）。
    """
    design = report.get("design") or {}
    gate = report.get("gate") or {}
    post_ik = report.get("post_ik")
    return {
        "status": report.get("status", "not_run"),
        "convention": report.get("convention", "cond(X)=sigma_max/sigma_min"),
        "gate": {"value": _finite_or_none(gate.get("value")),
                 "source": gate.get("source", "max(block_condition_numbers)")},
        "threshold": _finite_or_none(report.get("threshold")),
        "design": {
            "condition_number": _finite_or_none(design.get("condition_number")),
            "singular_values": _finite_list(design.get("singular_values") or []),
            "amplitudes_deg": _finite_list(design.get("amplitudes_deg") or []),
            "reference_rpy": _finite_list(design.get("reference_rpy") or []),
            "convention": design.get("convention",
                                     "cond(X)=sigma_max/sigma_min"),
        },
        "realized": _realized_to_json(report.get("realized")),
        "post_ik": (None if post_ik is None else {
            "condition_number": _finite_or_none(post_ik.get("condition_number")),
            "singular_values": _finite_list(post_ik.get("singular_values") or []),
            "n_points": int(post_ik.get("n_points", 0)),
            "blocks": {
                "orientation": _block_to_json(post_ik["blocks"].get("orientation")),
                "position": _block_to_json(post_ik["blocks"].get("position")),
            },
            "metric": post_ik.get("metric"),
        }),
    }


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

    # ---- 姿态激励条件数（预览模式无实测位姿，只报设计值） ----
    conditioning = _conditioning_to_json(
        evaluate_conditioning(args, waypoints, rpy_ref, None))

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
            "euler_convention": getattr(args, "euler_convention", "xyz-extrinsic"),
            "skip_ik": bool(args.skip_ik),
            "no_perturb": bool(args.no_perturb),
            "perturbed": False,          # 预览模式不采样扰动
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
        "conditioning": conditioning,
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2, allow_nan=False)
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

    # ---- 3. 姿态扰动采样 + 姿态激励条件数（一次性检查，非循环） ----
    def _poses_of(rpy_getter):
        """由路点姿态构造位姿序列（欧拉约定由 --euler 控制）。"""
        return [rbt.pose_to_T(wp.pos, rpy_getter(wp),
                              extrinsic=_extrinsic_of(args))
                for wp in waypoints]

    def _ref_rpy(wp):
        return wp.rpy_ref

    def _pert_rpy(wp):
        return wp.rpy_pert

    if args.no_perturb:
        # 开发验证模式：参考姿态直接建位姿跑链（不采样扰动，字段保持 None）
        poses = _poses_of(_ref_rpy)
    else:
        _sample_perturbations(waypoints, args.seed, args.perturb_deg)
        poses = _poses_of(_pert_rpy)

    # 条件数只依赖位姿（与 IK 无关）：先算一次设计/实测值并判定门禁，
    # 不做任何循环重试——三轴幅度比是设计参数，同分布内再采样改变不了它。
    conditioning = evaluate_conditioning(args, waypoints, rpy_ref, poses)

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

    # ---- 6b. IK 之后对**成功子集**重算实测条件数（只报告，不做门禁） ----
    ok_poses = [poses[s.index] for s in chain if s.ok]
    if ok_poses:
        info = pose_condition_number(
            ok_poses, include_position=args.cond_position,
            reference=_reference_rotation(args, rpy_ref))
        conditioning["post_ik"] = {
            "condition_number": info["condition_number"],
            "singular_values": info["singular_values"],
            "n_points": info["n_points"],
            "blocks": info["blocks"],
            "metric": info["metric"],
        }
    else:
        conditioning["post_ik"] = None

    conditioning_json = _conditioning_to_json(conditioning)
    post_ik = conditioning_json["post_ik"]
    if post_ik is None:
        post_ik_text = "post_ik=n/a"
    else:
        post_ik_cond = post_ik["condition_number"]
        post_ik_label = ("null" if post_ik_cond is None
                         else _fmt_cond(post_ik_cond))
        post_ik_text = f"post_ik={post_ik_label} (n={post_ik['n_points']})"
    print(f"姿态条件数: status={conditioning_json['status']} "
          f"门禁={_fmt_cond(conditioning_json['gate']['value'])} "
          f"阈值={conditioning_json['threshold']:g} "
          f"design={_fmt_cond(conditioning_json['design']['condition_number'])} "
          f"{post_ik_text}")

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
            "euler_convention": getattr(args, "euler_convention", "xyz-extrinsic"),
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
        "conditioning": conditioning_json,
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
        json.dump(payload, f, ensure_ascii=False, indent=2, allow_nan=False)
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
