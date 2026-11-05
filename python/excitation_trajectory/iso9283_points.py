#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 ISO 9283 从测试立方体生成 P1～P5 五个定位精度测点。

依据（对应《手眼 & 机器人精度 关联分析方案》1-E）：

* 在工作空间内划一个**内接立方体**（边长 L），P1～P5 取在该立方体的
  **对角线平面**内；
* **P1** 为立方体中心；
* **P2～P5** 位于该对角截面上**最大内切圆**的圆周上，互成 90°。该截面是
  ``L × L√2`` 的矩形，内切圆直径受短边限制，故半径 ``L/2``；
* 五个点的**姿态完全一致**，现场按**直线（LIN/L）**方式依次移动。

实现取对角平面 ``u = (X+Y)/√2``、``v = Z``，即截面绕竖直方向转 45°。
于是 P2～P5 相对中心偏移 ``(L/2)·(cos45°·û + sin45°·v̂)`` 及其 90° 旋转::

    P2 = c + (L/4, L/4, L·0.35355)
    P3 = c - (L/4, L/4, -L·0.35355)
    P4 = c - (L/4, L/4,  L·0.35355)
    P5 = c + (L/4, L/4, -L·0.35355)

    P1 = c（立方体中心）

用法（仓库根）::

    PYTHONPATH=python python3 -m excitation_trajectory.iso9283_points \\
        --cube 750 1420 -340 330 270 940 \\
        --urdf data/configs/hyundai_general6s.urdf \\
        --rpy 3 73 -167 --euler xyz-intrinsic \\
        --q-seed 43,-30,-36,42,-89,-1 --j5-min 20
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

from robot_model.nexbot_ik import Nexbot6R, euler_to_rot
from excitation_trajectory.aabb_snake_scan import config_tag

#: ISO 9283 的 5 个测点名称。
POINT_NAMES = ("P1", "P2", "P3", "P4", "P5")
#: 对角平面内 P2～P5 的角度（deg），45° 起每 90° 一个。
CORNER_ANGLES_DEG = (45.0, 135.0, 225.0, 315.0)


def cube_center(bounds) -> np.ndarray:
    """``(x0,x1,y0,y1,z0,z1)`` -> 立方体中心。"""
    x0, x1, y0, y1, z0, z1 = [float(v) for v in bounds]
    return np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0])


def iso9283_points(bounds, *, side=None) -> list:
    """按 ISO 9283 生成 P1～P5（基座系，mm）。

    Args:
        bounds: ``(x0,x1,y0,y1,z0,z1)``，立方体与坐标轴对齐。
        side: 立方体边长；``None`` 时取三边最小值（即最大内接立方体）。

    Returns:
        ``[{"name","pos","offset"}, ...]``，``offset`` 为相对中心的偏移。
    """
    x0, x1, y0, y1, z0, z1 = [float(v) for v in bounds]
    extents = (x1 - x0, y1 - y0, z1 - z0)
    length = min(extents) if side is None else float(side)
    if length <= 0:
        raise ValueError(f"立方体边长需为正，当前 {length}")
    for axis, extent in zip("XYZ", extents):
        if length > extent + 1e-9:
            raise ValueError(
                f"边长 {length} 超过 {axis} 方向可用范围 {extent}")

    center = cube_center(bounds)
    u_hat = np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)   # 对角平面水平方向
    v_hat = np.array([0.0, 0.0, 1.0])
    radius = length / 2.0

    points = [{"name": "P1", "pos": center.copy(),
               "offset": np.zeros(3)}]
    for name, angle in zip(POINT_NAMES[1:], CORNER_ANGLES_DEG):
        theta = math.radians(angle)
        offset = radius * (math.cos(theta) * u_hat + math.sin(theta) * v_hat)
        points.append({"name": name, "pos": center + offset,
                       "offset": offset.copy()})
    return points


def check_corners(rbt, bounds) -> list:
    """立方体 8 个角点在给定姿态下是否可达（用于判断边长是否过大）。"""
    x0, x1, y0, y1, z0, z1 = [float(v) for v in bounds]
    return list(itertools.product((x0, x1), (y0, y1), (z0, z1)))


def solve_points(rbt, points, rpy_deg, *, extrinsic: bool,
                 q_seed=None) -> list:
    """对 5 个点求 IK（同一姿态），返回带解的信息。"""
    rotation = euler_to_rot(rpy_deg, extrinsic=extrinsic)
    results = []
    q_ref = None if q_seed is None else [float(v) for v in q_seed]
    for point in points:
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = point["pos"]
        sols = rbt.ik(T, q_ref=q_ref)
        entry = {"name": point["name"], "pos": [float(v) for v in point["pos"]],
                 "offset": [float(v) for v in point["offset"]],
                 "n_sols": len(sols), "q_deg": None, "config": None,
                 "abs_j3_deg": None, "abs_j5_deg": None,
                 "fk_residual_mm": None}
        if sols:
            # 同一姿态下 5 点共享构型更利于直线移动：优先与上一点同构型
            if q_ref is not None:
                pick = min(sols, key=lambda q: float(np.sum(
                    (np.asarray(q) - np.asarray(q_ref)) ** 2)))
            else:
                pick = sols[0]
            entry["q_deg"] = [float(v) for v in pick]
            entry["config"] = list(config_tag(rbt, T, pick))
            entry["abs_j3_deg"] = abs(float(pick[2]))
            entry["abs_j5_deg"] = abs(float(pick[4]))
            back = rbt.fk(pick)
            entry["fk_residual_mm"] = float(
                np.linalg.norm(back[:3, 3] - point["pos"]))
            q_ref = [float(v) for v in pick]
        results.append(entry)
    return results


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.iso9283_points",
        description="按 ISO 9283 从测试立方体生成 P1～P5（对角线平面、同一姿态、"
                    "现场按直线方式移动），并给出每个点的 IK 与奇异余量。")
    parser.add_argument("--cube", nargs=6, type=float, required=True,
                        metavar=("X0", "X1", "Y0", "Y1", "Z0", "Z1"),
                        help="测试立方体的 AABB 边界（mm）")
    parser.add_argument("--side", type=float, default=None,
                        help="立方体边长（默认取三边最小值）")
    parser.add_argument("--urdf", required=True, help="机器人模型")
    parser.add_argument("--rpy", nargs=3, type=float, required=True,
                        metavar=("A", "B", "C"), help="5 点共用的姿态（deg）")
    parser.add_argument("--euler", choices=("xyz-extrinsic", "xyz-intrinsic"),
                        default="xyz-intrinsic", help="欧拉约定，默认 xyz-intrinsic")
    parser.add_argument("--q-seed", default=None, help="IK 参考关节角，逗号分隔 6 个 deg")
    parser.add_argument("--j5-min", type=float, default=20.0, metavar="DEG",
                        help="|J5| 下限（度），低于则告警，默认 20")
    parser.add_argument("--j3-min", type=float, default=10.0, metavar="DEG",
                        help="|J3| 下限（度），低于则告警，默认 10")
    parser.add_argument("--out", default=None, help="输出 JSON（含 5 点坐标与 IK）")
    parser.add_argument("--gp-template", default=None,
                        help="控制器 position_R1.json；给了就把 5 点写进 GP0001..GP0005")
    parser.add_argument("--gp-out", default=None, help="GP 文件输出路径（副本）")
    parser.add_argument("--gp-first", type=int, default=1,
                        help="起始 GP 编号，默认 1")
    args = parser.parse_args(argv)

    bounds = tuple(args.cube)
    for axis, (lo, hi) in zip("XYZ", ((bounds[0], bounds[1]),
                                      (bounds[2], bounds[3]),
                                      (bounds[4], bounds[5]))):
        if lo >= hi:
            parser.error(f"{axis} 下界需小于上界，当前 [{lo}, {hi}]")

    try:
        points = iso9283_points(bounds, side=args.side)
    except ValueError as error:
        parser.error(str(error))
    length = float(args.side) if args.side else min(
        bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])

    rbt = Nexbot6R(args.urdf)
    extrinsic = args.euler == "xyz-extrinsic"
    q_seed = None
    if args.q_seed:
        try:
            q_seed = [float(v) for v in str(args.q_seed).split(",")]
        except ValueError:
            parser.error(f"--q-seed 需逗号分隔 6 个浮点，当前 {args.q_seed!r}")
        if len(q_seed) != 6:
            parser.error(f"--q-seed 需恰好 6 个数，当前 {len(q_seed)}")

    results = solve_points(rbt, points, args.rpy, extrinsic=extrinsic,
                           q_seed=q_seed)

    print(f"测试立方体 {length:.0f} mm："
          f"X[{bounds[0]:.0f},{bounds[1]:.0f}] "
          f"Y[{bounds[2]:.0f},{bounds[3]:.0f}] "
          f"Z[{bounds[4]:.0f},{bounds[5]:.0f}]")
    center = cube_center(bounds)
    print(f"中心 {np.round(center,1).tolist()}  姿态 {args.rpy} ({args.euler})")
    print()
    print(f"{'点':>3} {'X':>9} {'Y':>9} {'Z':>9} {'解':>3} {'构型':>12} "
          f"{'|J3|':>7} {'|J5|':>7}")
    warnings = []
    for entry in results:
        cfg = str(tuple(entry["config"])) if entry["config"] else "—"
        j3 = entry["abs_j3_deg"]
        j5 = entry["abs_j5_deg"]
        print(f"{entry['name']:>3} {entry['pos'][0]:9.1f} {entry['pos'][1]:9.1f} "
              f"{entry['pos'][2]:9.1f} {entry['n_sols']:>3} {cfg:>12} "
              f"{j3 if j3 is None else round(j3,1):>7} "
              f"{j5 if j5 is None else round(j5,1):>7}")
        if entry["q_deg"] is None:
            warnings.append(f"{entry['name']} 在该姿态下不可达")
            continue
        if j3 is not None and j3 < args.j3_min:
            warnings.append(f"{entry['name']} |J3|={j3:.1f} < {args.j3_min:g}（肘部接近伸直）")
        if j5 is not None and j5 < args.j5_min:
            warnings.append(f"{entry['name']} |J5|={j5:.1f} < {args.j5_min:g}（腕奇异）")

    configs = {tuple(e["config"]) for e in results if e["config"]}
    if len(configs) > 1:
        warnings.append(f"5 点跨越 {len(configs)} 种构型 {sorted(configs)}，"
                        "直线移动会发生大幅重排")
    if warnings:
        print("\n警告：")
        for text in warnings:
            print(f"  - {text}")
    else:
        print("\n5 点同构型、可达、奇异余量充足 ✅")

    # ---- GP 导出：ISO 9283 走直线，必须给笛卡尔（BF），关节值同时写入 ----
    if args.gp_template:
        if not args.gp_out:
            parser.error("给了 --gp-template 就必须给 --gp-out")
        from excitation_trajectory.export_gp import make_gp_entry, write_gp_entries
        try:
            template = json.loads(Path(args.gp_template).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            print(f"错误: 读 GP 模板失败: {error}", file=sys.stderr)
            return 2
        rpy = [float(v) for v in args.rpy]
        points_for_gp = []
        for entry in results:
            if entry["q_deg"] is None:
                parser.error(f"{entry['name']} 无解，无法写入 GP")
            points_for_gp.append({"index": int(entry["name"][1:]),
                                  "q": entry["q_deg"],
                                  "cart": list(entry["pos"]) + rpy})
        # write_gp_entries 的 cartesian_of 只接 q，这里直接手工装配
        registers = template["globalPosition"]
        for offset, item in enumerate(points_for_gp):
            name = f"GP{args.gp_first + offset:04d}"
            registers[name] = make_gp_entry(item["q"], cart=item["cart"])
        Path(args.gp_out).write_text(
            json.dumps(template, ensure_ascii=False, indent=1) + "\n",
            encoding="utf-8")
        print(f"\nGP 已写入 {args.gp_first}..{args.gp_first + len(points_for_gp) - 1}"
              f" -> {args.gp_out}（笛卡尔 idx0..5 与关节 idx7..12 都已填）")
        print(f"  GP{args.gp_first:04d} = "
              f"{[round(v, 3) for v in registers[f'GP{args.gp_first:04d}']]}")

    if args.out:
        payload = {
            "iso": "ISO 9283",
            "cube": {"bounds": [float(v) for v in bounds], "side": length,
                     "center": [float(v) for v in center],
                     "diagonal_plane": "u=(X+Y)/sqrt2, v=Z"},
            "reference_pose": {"rpy_deg": [float(v) for v in args.rpy],
                               "euler_convention": args.euler},
            "points": results,
            "warnings": warnings,
            "note": "5 点姿态一致；现场按直线（LIN/L）方式移动，"
                    "每点到位后停下测量",
        }
        Path(args.out).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        print(f"\nJSON: {args.out}")
    return 0 if not any("不可达" in w for w in warnings) else 2


if __name__ == "__main__":
    raise SystemExit(main())
