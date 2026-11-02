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
      ``delta = (U(-6,6), U(-5,5), U(-3,3))`` deg（对应 rx/ry/rz），
      采样顺序与路点 index 一致（同 ``--seed`` 可复现），回填每个路点的
      ``delta_rpy`` 与 ``rpy_pert = rpy_ref + delta``（不 wrap）;
    * 默认流程只对扰动后路点求 IK（未扰动位姿不求）; ``--no-perturb``
      保持开发验证行为（参考姿态直接跑链，不采样扰动，扰动字段保持 None）;
    * JSON 最终结构: ``ik_chain`` 节改名 ``perturbed_ik``（字段不变）;
      ``meta`` 增加 ``perturbed`` 标记（默认 true / --no-perturb false）;
      同 seed 两次运行输出逐字节一致（浮点统一 round 6，无时间戳/随机序）。

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


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.aabb_snake_scan",
        description="AABB 网格蛇形寻位路点规划工具（CH2000 六轴焊接机器人）。"
                    "迭代 3/3 最终形态：路点生成 + 随机姿态扰动（默认模式，"
                    "--seed 可复现）+ 只对扰动后路点链式 IK 选解；"
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

    # ---- 3. 姿态扰动采样 + 位姿序列（迭代 3） ----
    if args.no_perturb:
        # 开发验证模式：参考姿态直接建位姿跑链（不采样扰动，字段保持 None）
        poses = [rbt.pose_to_T(wp.pos, wp.rpy_ref, extrinsic=not args.intrinsic)
                 for wp in waypoints]
    else:
        # 默认模式：逐路点依次独立采样 rx/ry/rz 扰动（顺序与 index 一致，
        # 同 seed 可复现）；只对扰动后路点求 IK，未扰动位姿不求
        rng = np.random.default_rng(args.seed)
        for wp in waypoints:
            delta = (rng.uniform(-6.0, 6.0),
                     rng.uniform(-5.0, 5.0),
                     rng.uniform(-3.0, 3.0))
            wp.delta_rpy = delta
            wp.rpy_pert = tuple(r + d for r, d in zip(wp.rpy_ref, delta))
        poses = [rbt.pose_to_T(wp.pos, wp.rpy_pert,
                               extrinsic=not args.intrinsic)
                 for wp in waypoints]

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
