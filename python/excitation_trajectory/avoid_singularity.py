#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""奇异点驱动的采样域自动重规划。

现场 50 点运行中报奇异时，把当时的关节角交给本工具，它会自己判断该改什么、
改多少，并输出一份与 ``aabb_snake_scan`` 同结构的扫描 JSON（可直接喂给
``export_gp``），不需要人工分析。

判定与决策流程（全部自动，确定性可复现）
----------------------------------------

1. **解析报告的奇异位形**：对每个 ``--singular-q`` 算 FK 位置与 ``|J3|``/``|J5|``。
2. **与内置判据对齐**，分两种情况：

   * ``|J5| < j5_min`` 或 ``|J3| < j3_min``
     → 与内置奇异判据一致。把该点实测裕度作为**经验阈值**上抬：
     ``j5_eff = max(j5_min, |J5| + avoid_margin)``（``j3_eff`` 同理），
     让重规划把这一类位形整体推开，而不只是躲开那一个点。
   * 两者都不满足（例如 ``|J5|=55.7°``）
     → 与内置判据不一致，说明控制器的报警条件本仓模型没覆盖。
     按**位置禁区**处理：以该点 FK 位置为球心、``avoid_radius`` 为半径，
     禁止任何路点落入。

3. **评估当前域**：生成 N 个路点 → 采样姿态扰动 → 链式 IK（带守卫）→
   段内穿零检查 → 设计条件数门禁。任一不满足即不可行。
4. **分层搜索**（tier 越小改动越轻，只在当前层全部失败才进下一层）：

   ======  ==========================================
   tier 1  整箱平移：±X/±Y/±Z × 若干步长
   tier 2  单侧收缩：6 个边界各自向内收若干步长
   tier 3  参考姿态微调：每轴 ±小角度（``--no-rpy-change`` 可禁用）
   tier 4  tier 1/2 的两步组合（贪心递进）
   ======  ==========================================

5. **在可行候选里挑改动最小的**：
   ``cost = volume_loss + center_shift/1000 + rpy_change/5``（越小越好，
   同分再比较 tier 与总位移），并强制：
   点数不变、关节在限位内、无段内穿零、设计条件数不超阈值。
6. 输出扫描 JSON + 报告 JSON，报告里写明**为什么这么改**、每个 tier 的失败
   原因、以及最终域的完整参数，便于现场复核。

用法::

    PYTHONPATH=python python3 -m excitation_trajectory.avoid_singularity \\
      --robot data/configs/hyundai_general6s.urdf \\
      --point-min 750 -600 270 --point-max 1300 600 940 \\
      --rpy 3 73 -167 --euler xyz-intrinsic \\
      --q-seed 43,-30,-36,42,-89,-1 \\
      --singular-q "0,0,0,55.721,0,-55.721" \\
      --out scan50_repaired.json --report avoid_report.json

退出码：0 = 已输出可行方案；2 = 所有候选都不可行（此时报告里给出各候选的
失败原因，不输出扫描 JSON）。
"""
from __future__ import annotations

import argparse
import io
import json
import math
import sys
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

from excitation_trajectory.aabb_snake_scan import (
    interpolate_pose,
    DEFAULT_EULER_CONVENTION,
    DEFAULT_GRID,
    DEFAULT_Q_SEED,
    EULER_CONVENTIONS,
    MOTION_ORDER,
    PERTURB_AMPLITUDE_DEG,
    _conditioning_to_json,
    _sample_perturbations,
    _select_solution,
    check_segments_cartesian,
    choose_grid,
    evaluate_conditioning,
    grid_point_count,
    plan_waypoints,
    solve_chain_guarded,
)
from robot_model.nexbot_ik import Nexbot6R

#: 判"肘部可疑"的角度阈值（deg）：``|J3|`` 小于它即按肘奇异处理，
#: 即使 ``--j3-min`` 未启用。现场停机常表现为 J3 贴 0。
ELBOW_SUSPECT_DEG = 5.0

#: 搜索时每个路点的重试次数（比最终输出的少，先粗筛再精算）。
SEARCH_ATTEMPTS = 5

#: tier 1/2 试探的位移步长（mm）。
SHIFT_MM = (200.0, 400.0, 600.0, 800.0)
#: 双轴平移的档位（mm）。
SHIFT_MM_PAIR = (400.0, 600.0, 800.0)

#: tier 3 试探的参考姿态微调（deg，逐轴）。
RPY_STEP_DEG = (5.0, 10.0)

#: 单侧收缩时，体积损失超过该比例就不再往 tier 2 之外扩（仅用于报告提示）。
VOLUME_LOSS_WARN = 0.5


# ---------------------------------------------------------------------------
# 输入解析
# ---------------------------------------------------------------------------
def parse_joint_angles(text, where: str = "--singular-q") -> list[float]:
    """解析 ``"a,b,c,d,e,f"`` 形式的关节角（deg）。"""
    try:
        values = [float(v) for v in str(text).replace(" ", "").split(",")]
    except ValueError as error:
        raise ValueError(f"{where} 需 6 个逗号分隔的角度（deg）：{error}") from error
    if len(values) != 6:
        raise ValueError(f"{where} 需恰好 6 个角度（当前 {len(values)} 个）")
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{where} 含非有限值")
    return values


def flatten_joint_angles(groups) -> list[list[float]]:
    """把 ``--singular-q`` 的多组写法统一成 ``[[6 个角度], ...]``。

    每个 group 既可以是 ``[0, 0, 0, 55.7, 0, -55.7]``（6 个 token），
    也可以是 ``["0,0,0,55.7,0,-55.7"]``（1 个逗号串）。

    Raises:
        ValueError: 展平后总数不是 6 的整数倍，或某个 token 无法解析。
    """
    flattened: list[float] = []
    for group in groups:
        for token in group:
            text = str(token).strip()
            pieces = text.split(",") if "," in text else [text]
            for piece in pieces:
                piece = piece.strip()
                if not piece:
                    continue
                try:
                    flattened.append(float(piece))
                except ValueError as error:
                    raise ValueError(f"无法解析关节角 {piece!r}：{error}") from error
    if not flattened:
        raise ValueError("--singular-q 未给出任何角度")
    if len(flattened) % 6 != 0:
        raise ValueError(f"--singular-q 展平后共 {len(flattened)} 个角度，"
                         f"应为 6 的整数倍")
    if not all(math.isfinite(v) for v in flattened):
        raise ValueError("--singular-q 含非有限值")
    return [flattened[i:i + 6] for i in range(0, len(flattened), 6)]


def classify_singularity(rbt: Nexbot6R, q_deg, *, j5_min: float, j3_min: float,
                         avoid_margin: float,
                         degenerate_eps: float = 1.0) -> dict:
    """把控制器报的关节角归类：腕奇异 / 肘奇异 / 退化读数 / 与模型判据不一致。

    现场停机姿态常常**不是**一个有效位姿：``J5 = 0`` 时 J4 与 J6 共线，只有
    ``J4+J6`` 有意义，控制器会把这对自由度随手分配（历史上的
    ``[0,0,0,55.721,0,-55.721]`` 就与零位 ``q=0`` 完全等价，FK 位置差 0.000e+00、
    旋转差 6e-17）。这种读数**不能**用来定位位置，也不能据此判定是哪一类奇异，
    否则会把肘奇异误认成腕奇异、并在零位附近造出一个无意义的禁区。

    因此这里先判退化，再判腕/肘；退化时两向都收紧，且不产生位置禁区。

    Args:
        rbt: 机器人模型。
        q_deg: 报告到的 6 个关节角（deg）。
        j5_min: 内置 ``|J5|`` 下限（deg）。
        j3_min: 内置 ``|J3|`` 下限（deg），0 表示未启用。
        avoid_margin: 经验阈值上抬量（deg）＝实测裕度之外再加的角度。
        degenerate_eps: 判定"接近零位/退化"的角度阈值（deg）。

    Returns:
        dict：``q_deg`` / ``position`` / ``j3_abs`` / ``j5_abs`` /
        ``kind``（``wrist`` / ``elbow`` / ``degenerate`` / ``unexplained``）/
        ``j5_floor`` / ``j3_floor`` / ``note``。
    """
    q = [float(v) for v in q_deg]
    transform = rbt.fk(q)
    position = [float(v) for v in transform[:3, 3]]
    j3_abs, j5_abs = abs(q[2]), abs(q[4])

    zero = rbt.fk([0.0] * 6)
    same_as_zero = (
        math.dist(position, [float(v) for v in zero[:3, 3]]) < 1e-6
        and float(np.max(np.abs(transform[:3, :3] - zero[:3, :3]))) < 1e-9)

    degenerate = (j5_abs < j5_min and j3_abs < degenerate_eps) or same_as_zero
    elbow_threshold = max(float(j3_min), ELBOW_SUSPECT_DEG)

    if degenerate:
        kind = "degenerate"
        j5_floor = max(float(j5_min), j5_abs + avoid_margin)
        j3_floor = max(float(j3_min), float(avoid_margin))
        note = ("J5≈0 时 J4/J6 共线、只有 J4+J6 有意义，控制器对这对自由度的"
                "分配是随机的；该读数不能定位位置，也不足以区分腕/肘奇异。"
                "按段内肘部裕度收紧处理。")
    elif j5_min > 0.0 and j5_abs < j5_min:
        kind = "wrist"
        j5_floor = max(float(j5_min), j5_abs + avoid_margin)
        j3_floor = float(j3_min)
        note = "端点 |J5| 低于阈值。"
    elif j3_abs < elbow_threshold:
        kind = "elbow"
        j5_floor = float(j5_min)
        j3_floor = max(float(j3_min), j3_abs + avoid_margin)
        note = "端点 |J3| 接近 0（肘伸展）。"
    else:
        kind = "unexplained"
        j5_floor = float(j5_min)
        j3_floor = float(j3_min)
        note = "与内置判据不一致，按位置禁区处理。"

    return {
        "q_deg": q,
        "position": position,
        "j3_abs": j3_abs,
        "j5_abs": j5_abs,
        "kind": kind,
        "j5_floor": j5_floor,
        "j3_floor": j3_floor,
        "same_as_zero_pose": bool(same_as_zero),
        "note": note,
    }


# ---------------------------------------------------------------------------
# 采样域
# ---------------------------------------------------------------------------
def make_domain(bounds, rpy, grid, z_insert: int = 0) -> dict:
    """构造一个采样域描述（bounds = [xmin,xmax,ymin,ymax,zmin,zmax]）。"""
    xs, ys, zs = grid
    return {
        "bounds": [float(v) for v in bounds],
        "rpy": [float(v) for v in rpy],
        "grid": (int(xs), int(ys), int(zs)),
        "z_insert": int(z_insert),
        "label": "原域",
        "tier": 0,
        "moves": [],
    }


def domain_volume(domain) -> float:
    """域体积（mm³）。"""
    b = domain["bounds"]
    return max(0.0, b[1] - b[0]) * max(0.0, b[3] - b[2]) * max(0.0, b[5] - b[4])


def domain_center(domain) -> list:
    """域中心（mm）。"""
    b = domain["bounds"]
    return [(b[0] + b[1]) / 2.0, (b[2] + b[3]) / 2.0, (b[4] + b[5]) / 2.0]


def clone_domain(domain, *, bounds=None, rpy=None, label=None, tier=None,
                 move=None) -> dict:
    """复制域并按需覆盖字段（``moves`` 记录改动链）。"""
    new = {
        "bounds": list(domain["bounds"] if bounds is None else bounds),
        "rpy": list(domain["rpy"] if rpy is None else rpy),
        "grid": domain["grid"],
        "z_insert": domain["z_insert"],
        "label": domain["label"] if label is None else label,
        "tier": domain["tier"] if tier is None else tier,
        "moves": list(domain["moves"]),
    }
    if move:
        new["moves"].append(move)
    return new


def point_count(domain) -> int:
    """该域生成的路点总数。"""
    xs, ys, zs = domain["grid"]
    return grid_point_count(xs, ys, zs, domain["z_insert"])


# ---------------------------------------------------------------------------
# 评估
# ---------------------------------------------------------------------------
def evaluate_domain(rbt: Nexbot6R, domain, *, target_count: int, seed: int,
                    q_seed, j5_min: float, j3_min: float, seg_min: float,
                    cond_max: float, perturb_deg, order: str,
                    extrinsic: bool, attempts: int,
                    forbidden: list, avoid_radius: float,
                    seg_samples: int = 11, j3_seg_min: float = 0.0,
                    steps_override=None, waypoints_override=None) -> dict:
    """评估一个采样域是否满足全部约束。

    Args:
        forbidden: 位置禁区球心列表（mm），来自 ``unexplained`` 类奇异位形。
        avoid_radius: 位置禁区半径（mm）。

    Returns:
        dict：``ok`` / ``reasons``（失败原因列表）/ ``stats``（裕度等）/
        ``waypoints`` / ``steps`` / ``guard`` / ``segments`` / ``conditioning``。
    """
    if point_count(domain) != target_count:
        bad = {"unsolved": target_count, "violations": 0, "j3_crossings": 0,
               "j5_crossings": 0, "j5_deficit": 0.0, "j3_deficit": 0.0,
               "j5_gap_sum": 0.0, "j3_gap_sum": 0.0,
               "seg_j5_gap": 0.0, "seg_j3_gap": 0.0}
        return {"ok": False,
                "reasons": [f"点数 {point_count(domain)} != {target_count}"],
                "metrics": bad, "score": failure_score(bad), "stats": {},
                "waypoints": None, "steps": None, "guard": None,
                "segments": None, "conditioning": None}

    b = domain["bounds"]
    rpy = tuple(domain["rpy"])
    xs, ys, zs = domain["grid"]
    if waypoints_override is not None:
        # 局部修复过的路点：姿态已被改写，不能重新采样覆盖
        waypoints = list(waypoints_override)
    else:
        waypoints = plan_waypoints(b[0], b[1], b[2], b[3], b[4], b[5],
                                   xs, ys, zs, rpy, z_insert=domain["z_insert"])
        _sample_perturbations(waypoints, seed, perturb_deg)

    reasons = []
    # 位置禁区：任何路点落在球内即判不可行
    if forbidden and avoid_radius > 0.0:
        for center in forbidden:
            for wp in waypoints:
                distance = math.dist(wp.pos, center)
                if distance < avoid_radius:
                    reasons.append(
                        f"路点 #{wp.index} 距奇异位形位置 {distance:.1f} mm "
                        f"< 禁区半径 {avoid_radius:.1f} mm")
                    break
            if reasons:
                break
    if reasons:
        bad = {"unsolved": 0, "violations": 0, "j3_crossings": 0,
               "j5_crossings": 0, "j5_deficit": 0.0, "j3_deficit": 0.0,
               "j5_gap_sum": 0.0, "j3_gap_sum": 0.0,
               "seg_j5_gap": 0.0, "seg_j3_gap": 0.0,
               "forbidden": len(reasons)}
        return {"ok": False, "reasons": reasons, "metrics": bad,
                "score": failure_score(bad), "stats": {},
                "waypoints": waypoints, "steps": None, "guard": None,
                "segments": None, "conditioning": None}

    # 条件数门禁（只算设计值，与 seed 无关，便宜）
    index = argparse.Namespace(cond_max=cond_max, cond_position=False,
                               perturb_deg=list(perturb_deg),
                               euler_convention=domain.get(
                                   "euler_convention", DEFAULT_EULER_CONVENTION))
    with redirect_stdout(io.StringIO()):
        conditioning = evaluate_conditioning(index, waypoints, rpy, None)
    gate = conditioning.get("gate") or {}
    if conditioning.get("status") == "rejected":
        reasons.append(f"姿态设计条件数 {gate.get('value')} > {cond_max}")

    # 链式 IK（带奇异守卫）；局部修复过的链直接沿用，不重解
    if steps_override is None:
        with redirect_stdout(io.StringIO()):
            steps, guard = solve_chain_guarded(
                rbt, waypoints, q_seed, j5_min=j5_min, j3_min=j3_min,
                attempts=attempts, seed=seed, amplitudes_deg=perturb_deg,
                order=order, extrinsic=extrinsic, strict=False)
    else:
        steps = list(steps_override)
        solved = [s for s in steps if s.ok and s.q is not None]
        guard = {
            "j5_min": float(j5_min), "j3_min": float(j3_min),
            "attempts": int(attempts), "resampled": [],
            "violations": [
                {"index": s.index, "q3_deg": float(s.q[2]),
                 "q5_deg": float(s.q[4]), "tries": 1}
                for s in solved
                if abs(s.q[4]) < j5_min
                or (j3_min > 0.0 and abs(s.q[2]) < j3_min)],
            "j5_abs_min_deg": (min(abs(s.q[4]) for s in solved)
                               if solved else None),
            "j3_abs_min_deg": (min(abs(s.q[2]) for s in solved)
                               if solved else None),
        }
    failed = [s.index for s in steps if not s.ok]
    if failed:
        reasons.append(f"{len(failed)} 个路点无解（首个 #{failed[0]}）")
    if guard.get("violations"):
        first = guard["violations"][0]
        reasons.append(f"{len(guard['violations'])} 个路点重采后仍低于奇异阈值"
                       f"（首个 #{first['index']}, "
                       f"|J5|={abs(first['q5_deg']):.2f}°）")

    # 现场用 MOVL：按笛卡尔直线插补重建路径，关节轨迹不是直线
    segments = check_segments_cartesian(rbt, waypoints, steps,
                                        samples=seg_samples, order=order,
                                        extrinsic=extrinsic)
    if segments.get("ik_failures"):
        reasons.append(f"MOVL 段内有 {segments['ik_failures']} 个采样点无解")
    if segments.get("j5_crossings") or segments.get("j3_crossings"):
        reasons.append(f"MOVL 段内穿零 J3={segments.get('j3_crossings')} "
                       f"J5={segments.get('j5_crossings')}")
    if segments.get("j5_abs_min") is not None and \
            segments["j5_abs_min"] < seg_min:
        reasons.append(f"段内 |J5| 最低 {segments['j5_abs_min']:.2f}° < {seg_min:.1f}°")
    if j3_seg_min > 0.0 and segments.get("j3_abs_min") is not None \
            and segments["j3_abs_min"] < j3_seg_min:
        reasons.append(f"MOVL 段内 |J3| 最低 {segments['j3_abs_min']:.2f}° "
                       f"< {j3_seg_min:.1f}°")

    violations = guard.get("violations") or []
    j5_seen = guard.get("j5_abs_min_deg")
    j3_seen = guard.get("j3_abs_min_deg")
    seg_j5 = segments.get("j5_abs_min")
    seg_j3 = segments.get("j3_abs_min")

    # 逐点裕度缺口之和：给爬山提供连续梯度（计数相同也能分出优劣）
    j5_gap_sum = 0.0
    j3_gap_sum = 0.0
    for step in steps:
        if not step.ok or step.q is None:
            continue
        if j5_min > 0.0:
            j5_gap_sum += max(0.0, j5_min - abs(step.q[4]))
        if j3_min > 0.0:
            j3_gap_sum += max(0.0, j3_min - abs(step.q[2]))

    metrics = {
        "unsolved": len(failed),
        "violations": len(violations),
        "j3_crossings": int(segments.get("j3_crossings") or 0),
        "j5_crossings": int(segments.get("j5_crossings") or 0),
        "ik_failures": int(segments.get("ik_failures") or 0),
        "j5_deficit": max(0.0, j5_min - j5_seen) if j5_seen is not None
                      else float(j5_min),
        "j3_deficit": (max(0.0, j3_min - j3_seen) if j3_min > 0.0
                       and j3_seen is not None else 0.0),
        "j5_gap_sum": j5_gap_sum,
        "j3_gap_sum": j3_gap_sum,
        "seg_j5_gap": max(0.0, seg_min - seg_j5) if seg_j5 is not None else 0.0,
        "seg_j3_gap": (max(0.0, j3_seg_min - seg_j3)
                       if (j3_seg_min > 0.0 and seg_j3 is not None) else 0.0),
    }
    return {
        "ok": not reasons,
        "reasons": reasons,
        "metrics": metrics,
        "score": failure_score(metrics),
        "stats": {
            "j5_abs_min": guard.get("j5_abs_min_deg"),
            "j3_abs_min": guard.get("j3_abs_min_deg"),
            "seg_j5_min": segments.get("j5_abs_min"),
            "seg_j3_min": segments.get("j3_abs_min"),
            "cond_design": (conditioning.get("design") or {}).get("condition_number"),
            "volume_mm3": domain_volume(domain),
            "center": domain_center(domain),
        },
        "waypoints": waypoints, "steps": steps, "guard": guard,
        "segments": segments, "conditioning": conditioning,
    }


# ---------------------------------------------------------------------------
# 候选生成 + 选择
# ---------------------------------------------------------------------------
def segment_profiles(rbt, waypoints, steps, start: int, *, window: int,
                     order: str, extrinsic: bool, samples: int) -> list:
    """给出 ``start`` 附近各段的 MOVL 剖面对比（含最差点位置）。

    Args:
        start: 0 基的段起点下标（即"已到的点"对应的 index）。
        window: 向前后各看多少段。

    Returns:
        list[dict]：``index``（段起点，0 基）/``gp_from``/``gp_to``（1 基 GP 号）/
        ``ok``/``j3_abs_min``/``j5_abs_min``/``crossings``/``worst_frac``/
        ``worst_pos``（最差采样点的法兰位置，mm）。
    """
    total = min(len(waypoints), len(steps))
    lo = max(0, int(start) - int(window))
    hi = min(total - 1, int(start) + int(window) + 1)
    profiles = []
    for index in range(lo, hi):
        first, second = steps[index], steps[index + 1]
        if not (first.ok and second.ok and first.q and second.q):
            continue
        wp0, wp1 = waypoints[index], waypoints[index + 1]
        T0 = rbt.pose_to_T(wp0.pos,
                           wp0.rpy_pert if wp0.rpy_pert is not None else wp0.rpy_ref,
                           order=order, extrinsic=extrinsic)
        T1 = rbt.pose_to_T(wp1.pos,
                           wp1.rpy_pert if wp1.rpy_pert is not None else wp1.rpy_ref,
                           order=order, extrinsic=extrinsic)
        q_ref = list(first.q)
        j3, j5, worst, worst_frac, worst_pos, ik_fail = [], [], None, 0.0, None, 0
        for step_index in range(max(2, int(samples))):
            frac = step_index / (max(2, int(samples)) - 1)
            if step_index == 0:
                q = list(first.q)
                position = [float(v) for v in T0[:3, 3]]
            elif step_index == max(2, int(samples)) - 1:
                q = list(second.q)
                position = [float(v) for v in T1[:3, 3]]
            else:
                pose = interpolate_pose(T0, T1, frac)
                solutions = rbt.ik(pose, q_ref=q_ref)
                if not solutions:
                    ik_fail += 1
                    break
                q = [float(v) for v in solutions[0]]
                position = [float(v) for v in pose[:3, 3]]
            margin = min(abs(q[2]), abs(q[4]))
            if worst is None or margin < worst:
                worst, worst_frac = margin, frac
                worst_pos = position
            j3.append(q[2])
            j5.append(q[4])
            q_ref = q
        profiles.append({
            "index": index,
            "gp_from": index + 1,
            "gp_to": index + 2,
            "j3_abs_min": min(abs(v) for v in j3) if j3 else None,
            "j5_abs_min": min(abs(v) for v in j5) if j5 else None,
            "crossings": int(len(j3) > 1 and (
                (min(j3) < 0.0 < max(j3)) or (min(j5) < 0.0 < max(j5)))),
            "ik_failures": ik_fail,
            "worst_frac": round(worst_frac, 3),
            "worst_pos": None if worst_pos is None
                         else [round(v, 1) for v in worst_pos],
        })
    return profiles


def repair_segment(rbt, waypoints, steps, index: int, *, order: str,
                   extrinsic: bool, perturb_deg, seed: int, attempts: int,
                   j5_min: float, j3_min: float, seg_samples: int,
                   neighbors: int = 1):
    """只重采该段两端点的姿态扰动，使这一段在 MOVL 下不撞奇异。

    与"整体挪箱"相比这是最小改动：位置、网格、点数都不变，只换端点姿态。
    依次尝试改**后端点**、再改**前端点**，每次都要求：

    * 该端点的端点裕度仍满足 ``|J5| >= j5_min``、``|J3| >= j3_min``；
    * 前后各 ``neighbors`` 段（含本段）在 MOVL 下都不穿零、裕度达标；
    * 构型标签与相邻点一致（不换肘上/肘下分支）。

    Returns:
        ``(steps, changed)``：``changed`` 为 ``None`` 表示没改成；
        否则 ``{"index": i, "delta_rpy": [...], "q": [...]}``。
    """
    count = min(len(waypoints), len(steps))
    if index < 0 or index + 1 >= count:
        return steps, None
    rng = np.random.default_rng(int(seed) + 0x5E9)
    amps = [float(v) for v in perturb_deg]
    lo = max(0, index - neighbors)
    hi = min(count - 1, index + 1 + neighbors)

    for target in (index + 1, index):      # 先换后端点（未到位的那点）
        if target < 0 or target >= count:
            continue
        previous = steps[target - 1] if target > 0 else None
        cfg_ref = previous.config if previous is not None and previous.config \
            else steps[target].config
        for _ in range(max(1, int(attempts))):
            delta = tuple(float(rng.uniform(-a, a)) for a in amps)
            waypoint = waypoints[target]
            rpy = tuple(r + d for r, d in zip(waypoint.rpy_ref, delta))
            pose = rbt.pose_to_T(waypoint.pos, rpy, order=order,
                                 extrinsic=extrinsic)
            q_sel, _, _, _, cfg, ok = _select_solution(
                rbt, pose, previous.q if previous is not None else q_sel_seed(steps),
                cfg_ref)
            if not ok or q_sel is None:
                continue
            if abs(q_sel[4]) < j5_min or (j3_min > 0.0 and abs(q_sel[2]) < j3_min):
                continue
            if previous is not None and previous.config and cfg != previous.config:
                continue

            trial_steps = list(steps)
            original_q = trial_steps[target].q
            trial_steps[target] = type(steps[target])(
                index=steps[target].index, q=q_sel,
                n_sols=steps[target].n_sols, config=cfg,
                dq_l2=steps[target].dq_l2, dq_l1=steps[target].dq_l1, ok=True)
            original_pose = (waypoint.delta_rpy, waypoint.rpy_pert)
            waypoint.delta_rpy = delta
            waypoint.rpy_pert = rpy

            sub_wp = waypoints[lo:hi + 1]
            sub_steps = trial_steps[lo:hi + 1]
            report = check_segments_cartesian(
                rbt, sub_wp, sub_steps, samples=seg_samples,
                order=order, extrinsic=extrinsic)
            good = (not report["ik_failures"]
                    and not report["j3_crossings"]
                    and not report["j5_crossings"]
                    and (report["j5_abs_min"] is None
                         or report["j5_abs_min"] >= j5_min)
                    and (j3_min <= 0.0 or report["j3_abs_min"] is None
                         or report["j3_abs_min"] >= j3_min))
            if good:
                return trial_steps, {"index": target,
                                     "delta_rpy": [float(v) for v in delta],
                                     "q": [float(v) for v in q_sel]}
            trial_steps[target] = type(steps[target])(
                index=steps[target].index, q=original_q,
                n_sols=steps[target].n_sols, config=steps[target].config,
                dq_l2=steps[target].dq_l2, dq_l1=steps[target].dq_l1, ok=True)
            waypoint.delta_rpy, waypoint.rpy_pert = original_pose
    return steps, None


def q_sel_seed(steps):
    """取链上第一个有解的点作为种子（用于首个路点的 q_ref）。"""
    for step in steps:
        if step.ok and step.q:
            return list(step.q)
    return [0.0] * 6


def ordered_candidates(base, *, allow_rpy: bool, allow_shrink: bool,
                       include_pairs: bool = False):
    """按**改动量从小到大**排序的候选域列表（确定性）。

    候选来源：

    * 单轴平移：``±{200,400,600,800} mm``
    * 单侧收缩：6 个边界 ``×{200,400,600,800} mm``（``allow_shrink``）
    * 参考姿态：每轴 ``±{5,10}°``（``allow_rpy``）
    * 双轴平移（``include_pairs``，仅在单轴全部失败后启用）

    排序键 ``(magnitude, kind_rank)``：先小改动、同量级优先平移（不动姿态
    约定）、再收缩、最后改参考姿态。``magnitude`` 以 mm 或 deg 的档位计。

    Returns:
        list[dict]，每项除域字段外含 ``magnitude`` 与 ``kind``。
    """
    b = base["bounds"]
    items = []

    def add(domain, magnitude, kind):
        domain["magnitude"] = float(magnitude)
        domain["kind"] = kind
        items.append(domain)

    for axis in range(3):
        for mag in SHIFT_MM:
            for sign in (1.0, -1.0):
                moved = list(b)
                moved[2 * axis] += sign * mag
                moved[2 * axis + 1] += sign * mag
                name = "XYZ"[axis]
                add(clone_domain(
                    base, bounds=moved, tier=1,
                    label=f"整箱沿 {name} {'+' if sign > 0 else '-'}{mag:g} mm",
                    move={"type": "translate", "axis": name,
                          "delta_mm": sign * mag}), mag, 0)

    if allow_shrink:
        for side in range(6):
            axis_name = "XYZ"[side // 2]
            for mag in SHIFT_MM:
                scaled = list(b)
                if side % 2 == 0:
                    scaled[side] += mag
                else:
                    scaled[side] -= mag
                if (scaled[1] - scaled[0]) <= 0 or (scaled[3] - scaled[2]) <= 0 \
                        or (scaled[5] - scaled[4]) <= 0:
                    continue
                add(clone_domain(
                    base, bounds=scaled, tier=2,
                    label=f"{axis_name} {'min' if side % 2 == 0 else 'max'} "
                          f"内收 {mag:g} mm",
                    move={"type": "shrink", "axis": axis_name,
                          "side": "min" if side % 2 == 0 else "max",
                          "delta_mm": mag}), mag, 1)

    if allow_rpy:
        for axis in range(3):
            for mag in RPY_STEP_DEG:
                for sign in (1.0, -1.0):
                    rpy = list(base["rpy"])
                    rpy[axis] += sign * mag
                    name = "ABC"[axis]
                    add(clone_domain(
                        base, rpy=rpy, tier=3,
                        label=f"参考姿态 {name} {'+' if sign > 0 else '-'}{mag:g}°",
                        move={"type": "rpy", "axis": name,
                              "delta_deg": sign * mag}), mag, 2)

    if include_pairs:
        for first, second in ((0, 1), (0, 2), (1, 2)):
            for mag in SHIFT_MM_PAIR:
                for sign_a in (1.0, -1.0):
                    for sign_b in (1.0, -1.0):
                        moved = list(b)
                        moved[2 * first] += sign_a * mag
                        moved[2 * first + 1] += sign_a * mag
                        moved[2 * second] += sign_b * mag
                        moved[2 * second + 1] += sign_b * mag
                        name_a, name_b = "XYZ"[first], "XYZ"[second]
                        add(clone_domain(
                            base, bounds=moved, tier=4,
                            label=f"整箱沿 {name_a} {'+' if sign_a > 0 else '-'}"
                                  f"{mag:g} mm 且 {name_b} "
                                  f"{'+' if sign_b > 0 else '-'}{mag:g} mm",
                            move={"type": "translate_pair"}), mag, 3)

    items.sort(key=lambda d: (d["magnitude"], d["kind"]))
    return items


def failure_score(metrics) -> float:
    """把不可行的程度压成一个标量（0 = 可行；越小越接近可行）。

    权重按"危害从大到小"排列：无解 >> 违反关节裕度 > 段内穿零 > 裕度缺口。
    用于爬山搜索比较候选优劣。
    """
    return (1e7 * float(metrics.get("forbidden", 0))
            + 1e7 * float(metrics.get("ik_failures", 0))
            + 1e6 * float(metrics.get("unsolved", 0))
            + 1e4 * float(metrics.get("violations", 0))
            + 1e3 * float(metrics.get("j3_crossings", 0)
                          + metrics.get("j5_crossings", 0))
            + 1e2 * float(metrics.get("j5_deficit", 0.0))
            + 1e2 * float(metrics.get("j3_deficit", 0.0))
            + 1e1 * float(metrics.get("j5_gap_sum", 0.0))
            + 1e1 * float(metrics.get("j3_gap_sum", 0.0))
            + 1e2 * float(metrics.get("seg_j5_gap", 0.0))
            + 1e2 * float(metrics.get("seg_j3_gap", 0.0)))


def change_cost(base, candidate) -> float:
    """候选相对原域的改动代价（越小越接近原意）。"""
    volume_base = domain_volume(base)
    volume = domain_volume(candidate)
    volume_loss = 0.0 if volume_base <= 0 else 1.0 - volume / volume_base
    shift = math.dist(domain_center(base), domain_center(candidate))
    rpy_change = max(abs(a - b) for a, b in zip(base["rpy"], candidate["rpy"]))
    return volume_loss + shift / 1000.0 + rpy_change / 5.0


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------
def scan_payload(domain, evaluation, *, seed: int, rpy, convention: str,
                 target_count) -> dict:
    """按 ``aabb_snake_scan`` 的 schema 组装扫描 JSON。"""
    steps = evaluation["steps"] or []
    conditioning = evaluation["conditioning"] or {}
    b = domain["bounds"]
    xs, ys, zs = domain["grid"]
    return {
        "meta": {
            "aabb": [round(v, 6) for v in b],
            "rpy": [round(float(v), 6) for v in rpy],
            "xs": int(xs), "ys": int(ys), "zs": int(zs),
            "z_insert": int(domain["z_insert"]),
            "motion_order": MOTION_ORDER,
            "grid_source": "avoid_singularity",
            "target_count": int(target_count),
            "aabb_source": "avoid_singularity",
            "seed": int(seed),
            "euler_convention": convention,
            "skip_ik": False,
            "no_perturb": False,
            "perturbed": True,
            "repaired_domain": {"label": domain["label"],
                                "tier": int(domain["tier"]),
                                "moves": domain["moves"]},
        },
        "waypoints": [
            {"index": wp.index,
             "pos": [round(float(v), 6) for v in wp.pos],
             "rpy_ref": [round(float(v), 6) for v in wp.rpy_ref],
             "rpy_pert": (None if wp.rpy_pert is None
                          else [round(float(v), 6) for v in wp.rpy_pert]),
             "delta_rpy": (None if wp.delta_rpy is None
                           else [round(float(v), 6) for v in wp.delta_rpy])}
            for wp in evaluation["waypoints"]
        ],
        "singularity_guard": evaluation["guard"],
        "segment_check": evaluation["segments"],
        "conditioning": _conditioning_to_json(conditioning),
        "perturbed_ik": [
            {"index": s.index,
             "q": None if s.q is None else [round(float(v), 6) for v in s.q],
             "config": None if s.config is None else [int(c) for c in s.config],
             "dq_l2": None if s.dq_l2 is None else round(s.dq_l2, 6),
             "dq_l1": None if s.dq_l1 is None else round(s.dq_l1, 6),
             "n_sols": int(s.n_sols),
             "ok": bool(s.ok)}
            for s in steps
        ],
    }


def build_arg_parser() -> argparse.ArgumentParser:
    """构建 CLI 参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.avoid_singularity",
        description="奇异点驱动的采样域自动重规划：给奇异时的关节角，程序自己"
                    "判断该平移/收缩/改参考姿态，并输出新的 N 点扫描 JSON。")
    parser.add_argument("--robot", "--urdf", "--robot-config", dest="robot",
                        required=True, help="机器人模型（.urdf 或控制器 .json）")
    parser.add_argument("--point-min", nargs=3, type=float,
                        metavar=("X", "Y", "Z"), required=True,
                        help="原始采样域最小点（mm）")
    parser.add_argument("--point-max", nargs=3, type=float,
                        metavar=("X", "Y", "Z"), required=True,
                        help="原始采样域最大点（mm）")
    parser.add_argument("--rpy", nargs=3, type=float, default=[180.0, 0.0, -90.0],
                        metavar=("A", "B", "C"), help="参考姿态（deg）")
    parser.add_argument("--euler", choices=EULER_CONVENTIONS, type=str.lower,
                        default=DEFAULT_EULER_CONVENTION, metavar="CONVENTION",
                        help=f"欧拉约定，默认 {DEFAULT_EULER_CONVENTION}")
    parser.add_argument("--q-seed", default=DEFAULT_Q_SEED,
                        help="IK 链种子关节角（逗号分隔 6 个 deg）")
    parser.add_argument("--target-count", type=int, default=50,
                        help="目标路点数（自动选网格凑满），默认 50")
    parser.add_argument("--singular-q", nargs="+", action="append",
                        required=True, metavar="DEG",
                        help="奇异发生时的 6 个关节角（deg）。两种写法等价："
                             "``--singular-q 0 0 0 55.7 0 -55.7``（6 个数）或 "
                             "``--singular-q=\"0,0,0,55.7,0,-55.7\"``（逗号串）；"
                             "可重复给多次以报告多个奇异位形")
    parser.add_argument("--stopped-at", type=int, default=None, metavar="N",
                        help="现场已走到的 GP 点号（1 基）。给了它就先定位 "
                             "N->N+1 这一段、先做局部修复，失败才整体挪箱")
    parser.add_argument("--segment-window", type=int, default=3, metavar="W",
                        help="--stopped-at 时向前后各看多少段找最危险段，默认 3")
    parser.add_argument("--force-search", action="store_true",
                        help="即使模型判定报告段安全，也强行做整体搜索")
    parser.add_argument("--repair-attempts", type=int, default=60, metavar="N",
                        help="局部修复时每端点的姿态重采次数，默认 60")
    parser.add_argument("--perturb-deg", nargs=3, type=float,
                        default=list(PERTURB_AMPLITUDE_DEG),
                        metavar=("A_RX", "A_RY", "A_RZ"),
                        help="姿态扰动三轴半宽（deg）")
    parser.add_argument("--j5-min", type=float, default=20.0, metavar="DEG",
                        help="|J5| 下限（deg），默认 20")
    parser.add_argument("--j3-min", type=float, default=0.0, metavar="DEG",
                        help="|J3| 下限（deg），默认 0（关闭）")
    parser.add_argument("--seg-samples", type=int, default=11, metavar="N",
                        help="MOVL 段检查每段采样点数，默认 11（搜索时求快；"
                             "最终复核用 --seg-samples-final）")
    parser.add_argument("--seg-samples-final", type=int, default=31, metavar="N",
                        help="最终复核时每段采样点数，默认 31")
    parser.add_argument("--seg-min", type=float, default=5.0, metavar="DEG",
                        help="段内 |J5|/|J3| 最小允许值（deg），默认 5")
    parser.add_argument("--avoid-margin", type=float, default=10.0, metavar="DEG",
                        help="与内置判据一致时，在实测裕度上再加的角度（deg），默认 10")
    parser.add_argument("--avoid-radius", type=float, default=80.0, metavar="MM",
                        help="与内置判据不一致时，把奇异位置当作禁区的半径（mm），默认 80")
    parser.add_argument("--cond-max", type=float, default=9.0, metavar="N",
                        help="姿态设计条件数阈值，默认 9")
    parser.add_argument("--no-rpy-change", action="store_true",
                        help="禁止微调参考姿态（只允许平移/收缩）")
    parser.add_argument("--no-shrink", action="store_true",
                        help="禁止收缩（只允许平移；体积保持不变）")
    parser.add_argument("--max-candidates", type=int, default=400, metavar="N",
                        help="最多评估多少个候选（默认 400）")
    parser.add_argument("--seed", type=int, default=0, help="姿态扰动随机种子")
    parser.add_argument("--j5-attempts", type=int, default=30, metavar="N",
                        help="最终确认时的单点重试上限，默认 30")
    parser.add_argument("--out", default="scan_repaired.json",
                        help="输出扫描 JSON（与 aabb_snake_scan 同结构）")
    parser.add_argument("--report", default="avoid_report.json",
                        help="输出决策报告 JSON")
    return parser


def run(args) -> int:
    """执行自动重规划，返回退出码。"""
    rbt = Nexbot6R(args.robot)
    order, _, sense = args.euler.partition("-")
    order, extrinsic = order.upper(), sense != "intrinsic"

    q_seed = [float(v) for v in str(args.q_seed).replace(" ", "").split(",")]
    if len(q_seed) != 6:
        print(f"错误: --q-seed 需 6 个角度，当前 {len(q_seed)} 个", file=sys.stderr)
        return 2

    bounds = [float(args.point_min[0]), float(args.point_max[0]),
              float(args.point_min[1]), float(args.point_max[1]),
              float(args.point_min[2]), float(args.point_max[2])]
    for lo, hi, axis in ((bounds[0], bounds[1], "X"), (bounds[2], bounds[3], "Y"),
                         (bounds[4], bounds[5], "Z")):
        if lo > hi:
            print(f"错误: {axis} 轴 min {lo} > max {hi}", file=sys.stderr)
            return 2

    try:
        xs, ys, zs, z_insert = choose_grid(args.target_count, [
            bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]])
    except ValueError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    base = make_domain(bounds, args.rpy, (xs, ys, zs), z_insert)
    base["euler_convention"] = args.euler

    # ---- 1) 分析报告的奇异位形 ----
    analyses = []
    try:
        reported = flatten_joint_angles(args.singular_q)
    except ValueError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2
    for q in reported:
        analyses.append(classify_singularity(
            rbt, q, j5_min=args.j5_min, j3_min=args.j3_min,
            avoid_margin=args.avoid_margin))

    j5_eff = max([args.j5_min] + [a["j5_floor"] for a in analyses])
    j3_eff = max([args.j3_min] + [a["j3_floor"] for a in analyses])
    elbow_required = any(a["kind"] in ("elbow", "degenerate") for a in analyses)
    # 报了肘/退化时，段内肘部也要有裕度（端点合格 ≠ 路径合格）
    j3_seg_min = max(args.seg_min, j3_eff) if elbow_required else 0.0
    forbidden = [a["position"] for a in analyses if a["kind"] == "unexplained"]

    print("奇异位形分析:")
    for a in analyses:
        print(f"  q={[round(v, 3) for v in a['q_deg']]}  "
              f"|J3|={a['j3_abs']:.2f}° |J5|={a['j5_abs']:.2f}°  "
              f"位置={[round(v, 1) for v in a['position']]}  -> {a['kind']}")
    print(f"有效阈值: |J5| >= {j5_eff:.2f}°, |J3| >= {j3_eff:.2f}°"
          + (f", 位置禁区 {len(forbidden)} 个（r={args.avoid_radius:g} mm）"
             if forbidden else ""))

    def _evaluate(domain, attempts, steps_override=None,
                  waypoints_override=None):
        return evaluate_domain(
            rbt, domain, target_count=args.target_count, seed=args.seed,
            q_seed=q_seed, j5_min=j5_eff, j3_min=j3_eff, seg_min=args.seg_min,
            cond_max=args.cond_max, perturb_deg=args.perturb_deg,
            order=order, extrinsic=extrinsic, attempts=attempts,
            forbidden=forbidden, avoid_radius=args.avoid_radius,
            seg_samples=args.seg_samples, j3_seg_min=j3_seg_min,
            steps_override=steps_override,
            waypoints_override=waypoints_override)

    # ---- 2) 先看原域 ----
    print("\n评估原域 ...")
    baseline = _evaluate(base, args.j5_attempts)
    segment_diagnostics = None
    local_change = None
    if args.stopped_at is not None:
        target_index = int(args.stopped_at) - 1
        if not 0 <= target_index < args.target_count - 1:
            print(f"错误: --stopped-at 需在 1..{args.target_count - 1} 之间"
                  f"（它表示已走到的点，后面还得有一点）", file=sys.stderr)
            return 2
        if not baseline.get("steps") or not baseline.get("segments"):
            # 原域因位置禁区早退时没有链可分段，直接说明原因（否则 len(None) 崩）
            print(f"  原域在生成链之前就判不可行，跳过段定位："
                  f"{'；'.join(baseline['reasons'][:2])}")
            profiles = []
        else:
            profiles = segment_profiles(
                rbt, baseline["waypoints"], baseline["steps"], target_index,
                window=args.segment_window, order=order, extrinsic=extrinsic,
                samples=args.seg_samples_final)
        segment_diagnostics = {"reported_gp": int(args.stopped_at),
                               "window": int(args.segment_window),
                               "profiles": profiles}
        print(f"\n段定位（已到 GP{args.stopped_at}，故障应在 "
              f"GP{args.stopped_at}->GP{args.stopped_at + 1}）:")
        for profile in profiles:
            if profile["j3_abs_min"] is None:
                continue
            flag = "  <== 报告段" if profile["index"] == target_index else ""
            print(f"  GP{profile['gp_from']:02d}->GP{profile['gp_to']:02d}  "
                  f"|J3|min={profile['j3_abs_min']:7.3f}  "
                  f"|J5|min={profile['j5_abs_min']:7.3f}  "
                  f"穿零={profile['crossings']}  无解={profile['ik_failures']}"
                  f"  最差点={profile['worst_pos']} (t={profile['worst_frac']})"
                  f"{flag}")
        worst_segment = min(
            [p for p in profiles if p["j3_abs_min"] is not None],
            key=lambda p: min(p["j3_abs_min"], p["j5_abs_min"]),
            default=None)
        if worst_segment is not None and worst_segment["index"] != target_index:
            print(f"  注意: 附近最危险的是 GP{worst_segment['gp_from']}->"
                  f"GP{worst_segment['gp_to']}，不是报告的那一段"
                  f"（现场点号可能有偏差）")

        def _profile_fails(profile) -> bool:
            """这一段是否真的越界（用于决定要不要动手）。"""
            if profile["j3_abs_min"] is None:
                return False
            if profile["crossings"] or profile["ik_failures"]:
                return True
            if profile["j5_abs_min"] < args.seg_min:
                return True
            return bool(j3_seg_min > 0.0
                        and profile["j3_abs_min"] < j3_seg_min)

        offenders = [p for p in profiles if _profile_fails(p)]
        by_index = {p["index"]: p for p in profiles}
        reported = by_index.get(target_index)
        if not offenders:
            if baseline["ok"]:
                print("  窗口内各段都满足阈值，且整链无越界：不动采样域。")
            else:
                segment_stats = baseline.get("segments") or {}
                print("  窗口内各段都满足阈值：这一点附近复现不出报警，"
                      "建议核对现场点号或控制器报警类型。")
                print(f"  但整条链另有 {segment_stats.get('j3_crossings', 0)} 处 "
                      f"J3 穿零 / {segment_stats.get('j5_crossings', 0)} 处 "
                      f"J5 穿零，接着做整体搜索。")
                bad = [f"GP{s['from'] + 1}->GP{s['to'] + 1}"
                       for s in (segment_stats.get("segments") or [])
                       if s["j3_crosses_zero"] or s["j5_crosses_zero"]]
                if bad:
                    print("  穿零段：" + "、".join(bad[:12])
                          + ("…" if len(bad) > 12 else ""))
        else:
            repair_index = (target_index if reported is not None
                            and _profile_fails(reported)
                            else min(offenders,
                                     key=lambda p: min(p["j3_abs_min"],
                                                       p["j5_abs_min"]))["index"])
            if repair_index != target_index:
                print(f"  报告段本身安全，改修最危险的 "
                      f"GP{repair_index + 1}->GP{repair_index + 2}")

        # 先试局部修复：只换端点姿态，不动网格与位置
        repaired_steps, changed = repair_segment(
            rbt, baseline["waypoints"], baseline["steps"],
            repair_index if offenders else -1,
            order=order, extrinsic=extrinsic, perturb_deg=args.perturb_deg,
            seed=args.seed, attempts=args.repair_attempts,
            j5_min=j5_eff, j3_min=j3_eff,
            seg_samples=args.seg_samples_final)
        if changed is not None:
            local_change = changed
            print(f"  局部修复成功：重采第 {changed['index'] + 1} 点"
                  f"（GP{changed['index'] + 1}）的姿态，"
                  f"delta_rpy={[round(v, 2) for v in changed['delta_rpy']]}")
            repaired_eval = _evaluate(
                base, args.j5_attempts, steps_override=repaired_steps,
                waypoints_override=baseline["waypoints"])
            if repaired_eval["ok"]:
                baseline = repaired_eval
                print("  全链复核通过（局部修复后整计划安全）")
            else:
                print("  局部修复后全链仍不满足：" +
                      "；".join(repaired_eval["reasons"][:2]))
                local_change = None
        elif offenders:
            print("  局部修复未成功（换端点姿态解决不了），转整体挪箱")
    if baseline["ok"]:
        print("原域已满足全部约束，无需修改。")
        chosen, chosen_eval = base, baseline
        search_log = [{"label": base["label"], "tier": 0, "ok": True,
                       "reasons": []}]
    else:
        print(f"原域不可行（评分 {baseline['score']:.1f}）："
              + "；".join(baseline["reasons"]))
        search_log = [{"label": base["label"], "tier": 0, "ok": False,
                       "reasons": baseline["reasons"]}]
        # ---- 3) 按改动量从小到大的一次性搜索 + 二分细化 ----
        # 候选按"改动量"升序排列，第一个可行的就已接近最小改动；
        # 找到后再二分收缩该改动，压到刚好可行，避免过度修改采样域。
        best = None                        # (cost, tier, domain, evaluation)
        evaluated = 0

        def _run_phase(include_pairs: bool, tag: str):
            """跑一轮候选；返回 (domain, evaluation, tier) 或 None。"""
            nonlocal evaluated
            candidates = ordered_candidates(
                base, allow_rpy=not args.no_rpy_change,
                allow_shrink=not args.no_shrink, include_pairs=include_pairs)
            for candidate in candidates:
                evaluated += 1
                if evaluated > args.max_candidates:
                    print(f"  已达候选上限 {args.max_candidates}，停止搜索")
                    return None
                result = _evaluate(candidate, SEARCH_ATTEMPTS)
                search_log.append({
                    "phase": tag, "label": candidate["label"],
                    "tier": candidate["tier"],
                    "magnitude": candidate["magnitude"],
                    "bounds": [round(v, 3) for v in candidate["bounds"]],
                    "rpy": [round(v, 3) for v in candidate["rpy"]],
                    "ok": result["ok"], "score": round(result["score"], 3),
                    "reasons": result["reasons"][:2]})
                if result["ok"]:
                    print(f"  可行：{candidate['label']}"
                          f"（改动量 {candidate['magnitude']:g}）")
                    return candidate, result, candidate["tier"]
            return None

        found = _run_phase(include_pairs=False, tag="single")
        if found is None:
            print("  单轴/单侧候选全部不可行，尝试双轴平移组合 ...")
            found = _run_phase(include_pairs=True, tag="pair")

        if found is not None:
            candidate, result, tier = found
            candidate, result = _refine(base, candidate, result, _evaluate,
                                        search_log)
            best = (change_cost(base, candidate), tier, candidate, result)

        if best is None:
            if forbidden:
                # 禁区球心落在域内部时，平移保持相对距离、收缩只会更近，
                # 单纯改采样域无法满足；直接说明，避免现场反复试。
                center = domain_center(base)
                nearest = min(math.dist(center, point) for point in forbidden)
                if nearest < args.avoid_radius:
                    print(f"\n提示: 有奇异位形的位置距采样域中心仅 {nearest:.0f} mm "
                          f"（禁区半径 {args.avoid_radius:g} mm）。禁区球心在域内时，"
                          f"平移不改变相对距离、收缩只会更近，改采样域无解；"
                          f"请调小 --avoid-radius、换参考姿态，或直接换采样区域。",
                          file=sys.stderr)
            print("\n所有候选都不可行。报告见 " + args.report, file=sys.stderr)
            write_report(args, analyses, base, baseline, search_log,
                         j5_eff, j3_eff, forbidden, chosen=None)
            return 2
        _, _, chosen, chosen_eval = best
        # 用完整重试次数复核，避免粗筛阶段的低重试次数造成假可行
        print(f"\n用完整重试次数复核：{chosen['label']}")
        original_samples = args.seg_samples
        args.seg_samples = args.seg_samples_final
        confirm = _evaluate(chosen, args.j5_attempts)
        args.seg_samples = original_samples
        if not confirm["ok"]:
            print("复核未通过：" + "；".join(confirm["reasons"]), file=sys.stderr)
            write_report(args, analyses, base, baseline, search_log,
                         j5_eff, j3_eff, forbidden, chosen=None)
            return 2
        chosen_eval = confirm
        print("复核通过。")

    # ---- 4) 输出 ----
    payload = scan_payload(chosen, chosen_eval, seed=args.seed,
                           rpy=chosen["rpy"], convention=args.euler,
                           target_count=args.target_count)
    Path(args.out).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8")
    write_report(args, analyses, base, baseline, search_log, j5_eff, j3_eff,
                 forbidden, chosen=chosen, chosen_eval=chosen_eval)

    print(f"\n已输出 {args.out}（{len(payload['waypoints'])} 点）与 {args.report}")
    if chosen["tier"] == 0:
        print("改动：无")
    else:
        print(f"改动：{chosen['label']}")
        print(f"  边界 {[round(v, 1) for v in chosen['bounds']]}")
        print(f"  参考姿态 {[round(v, 3) for v in chosen['rpy']]}")
    return 0


def _refine(base, candidate, result, evaluate, search_log,
            iterations: int = 4) -> tuple:
    """二分压缩已可行候选的改动量，压到"刚好可行"。

    基线（改动量 0）已知不可行、候选可行，于是在 ``[0, 1]`` 的缩放系数上二分：
    每次取中点重试，可行就把上界压到中点，不可行就抬下界。``iterations`` 次后
    改动量精度约 1/2**iterations（默认 4 次 ≈ 6%）。

    Returns:
        ``(domain, evaluation)``：压缩后的域及其评估结果（仍可行）。
    """
    moves = candidate["moves"]
    if len(moves) != 1:
        return candidate, result
    move = moves[0]
    kind = move.get("type")
    if kind not in ("translate", "shrink", "rpy"):
        return candidate, result

    def build(factor: float) -> dict:
        trial = clone_domain(candidate, tier=candidate["tier"],
                             label=candidate["label"])
        trial["moves"] = list(moves)
        if kind == "translate":
            axis = "XYZ".index(move["axis"])
            delta = move["delta_mm"] * factor
            bounds = list(base["bounds"])
            bounds[2 * axis] += delta
            bounds[2 * axis + 1] += delta
            trial["bounds"] = bounds
        elif kind == "shrink":
            side = 0 if move["side"] == "min" else 1
            index = 2 * "XYZ".index(move["axis"]) + side
            bounds = list(base["bounds"])
            bounds[index] += (move["delta_mm"] * factor
                              if side == 0 else -move["delta_mm"] * factor)
            trial["bounds"] = bounds
        else:                                   # rpy
            axis = "ABC".index(move["axis"])
            rpy = list(base["rpy"])
            rpy[axis] += move["delta_deg"] * factor
            trial["rpy"] = rpy
        return trial

    lo, hi = 0.0, 1.0                       # lo=不可行, hi=可行
    best_domain, best_result = candidate, result
    for _ in range(max(1, int(iterations))):
        mid = (lo + hi) / 2.0
        trial = build(mid)
        if (trial["bounds"][1] - trial["bounds"][0]) <= 0 or \
                (trial["bounds"][3] - trial["bounds"][2]) <= 0 or \
                (trial["bounds"][5] - trial["bounds"][4]) <= 0:
            break
        result_trial = evaluate(trial, SEARCH_ATTEMPTS)
        search_log.append({
            "phase": "refine", "label": trial["label"],
            "tier": trial["tier"], "ok": result_trial["ok"],
            "factor": round(mid, 4),
            "score": round(result_trial["score"], 3),
            "bounds": [round(v, 3) for v in trial["bounds"]],
            "rpy": [round(v, 3) for v in trial["rpy"]],
            "reasons": result_trial["reasons"][:2]})
        if result_trial["ok"]:
            hi, best_domain, best_result = mid, trial, result_trial
        else:
            lo = mid

    # 细化后改动量已变，重建 label/moves 以保持输出诚实
    final = _describe_change(base, best_domain)
    best_domain = clone_domain(best_domain, label=final["label"])
    best_domain["moves"] = final["moves"]
    if hi < 1.0:
        print(f"  细化：改动量压到 {hi:.0%} -> {final['label']}")
    return best_domain, best_result


def _describe_change(base, domain) -> dict:
    """由最终边界/姿态反推改动描述（label + moves），避免沿用粗筛时的旧标签。"""
    moves = []
    b, o = domain["bounds"], base["bounds"]
    for axis in range(3):
        name = "XYZ"[axis]
        delta = b[2 * axis] - o[2 * axis]
        if abs(delta) > 1e-9:
            moves.append({"type": "translate", "axis": name,
                          "delta_mm": round(delta, 3)})
    for side in range(6):
        axis_name = "XYZ"[side // 2]
        delta = b[side] - o[side]
        if abs(delta) > 1e-9 and not any(
                m["type"] == "translate" and m["axis"] == axis_name
                and abs(m["delta_mm"] - (b[2 * (side // 2)] -
                                         o[2 * (side // 2)])) < 1e-9
                for m in moves):
            moves.append({"type": "shrink", "axis": axis_name,
                          "side": "min" if side % 2 == 0 else "max",
                          "delta_mm": round(delta, 3)})
    for axis in range(3):
        delta = domain["rpy"][axis] - base["rpy"][axis]
        if abs(delta) > 1e-9:
            moves.append({"type": "rpy", "axis": "ABC"[axis],
                          "delta_deg": round(delta, 3)})
    parts = []
    for move in moves:
        if move["type"] == "translate":
            parts.append(f"{move['axis']} 平移 {move['delta_mm']:+.0f} mm")
        elif move["type"] == "shrink":
            parts.append(f"{move['axis']} {move['side']} 内收 "
                         f"{abs(move['delta_mm']):.0f} mm")
        else:
            parts.append(f"{move['axis']} 姿态 {move['delta_deg']:+.1f}°")
    return {"label": "；".join(parts) if parts else "无改动", "moves": moves}


def write_report(args, analyses, base, baseline, search_log, j5_eff, j3_eff,
                 forbidden, *, chosen, chosen_eval=None) -> None:
    """写出决策报告 JSON（含每个候选的结果与最终改动链）。"""
    report = {
        "tool": "avoid_singularity",
        "inputs": {
            "robot": str(args.robot),
            "point_min": list(args.point_min),
            "point_max": list(args.point_max),
            "rpy": list(args.rpy),
            "euler": args.euler,
            "target_count": int(args.target_count),
            "singular_q": [[float(v) for v in a["q_deg"]] for a in analyses],
            "perturb_deg": list(args.perturb_deg),
        },
        "singularity_analysis": analyses,
        "effective_thresholds": {"j5_min_deg": j5_eff, "j3_min_deg": j3_eff},
        "forbidden_positions": forbidden,
        "avoid_radius_mm": float(args.avoid_radius),
        "baseline": {"label": base["label"],
                     "bounds": [round(v, 3) for v in base["bounds"]],
                     "rpy": [round(v, 3) for v in base["rpy"]],
                     "ok": baseline["ok"], "reasons": baseline["reasons"]},
        "search": search_log,
        "decision": None if chosen is None else {
            "label": chosen["label"],
            "tier": int(chosen["tier"]),
            "bounds": [round(v, 3) for v in chosen["bounds"]],
            "rpy": [round(v, 3) for v in chosen["rpy"]],
            "moves": chosen["moves"],
            "cost": round(change_cost(base, chosen), 6),
            "stats": (chosen_eval or {}).get("stats"),
        },
    }
    Path(args.report).write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8")


def main(argv=None) -> int:
    """CLI 入口。"""
    args = build_arg_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
