#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 ISO 9283 从**测试立方体**生成 P1～P5 五个定位精度测点。

来源与改造说明
--------------
本模块由 ``~/Downloads/iso9283_cube_generator.py`` 改造而来。原脚本的思路
（采样工作空间 → 找最大内接立方体 → 5 点 IK 验证）保留，但做了四处实质性替换：

===================  ==========================  ==============================
原脚本               本模块                       原因
===================  ==========================  ==============================
硬编码 CH2000 DH     读 ``--robot``（URDF/JSON）   换机型必须改代码是不可接受的
scipy 数值 IK        仓库的解析 IK ``Nexbot6R``    少一个依赖，且解析解更准更快
``plt.show()`` 阻塞   ``--plot`` 存 PNG（Agg）      现场无显示环境
参数写死在 main       argparse + ``--layout``      三种布点可切换、可复现
===================  ==========================  ==============================

三种布点（``--layout``）
------------------------
``vertices``（默认，ISO 9283 的对角截面）
    五点在立方体的**对角截面**上：P1 是立方体中心，P2～P5 是该截面矩形的四个角。
    截面取 XZ 平面的对角面 ``x = z``（从 XZ 最小角倾斜到最大角），四个角正是
    立方体顶点 ``(h,h,h) (h,-h,h) (-h,h,-h) (-h,-h,-h)``（h = 边长/2）——
    三轴都用满 ±h，空间利用率最大。

``circle``
    P1 为中心，P2～P5 落在**同一对角截面**上的内切圆（半径 h，受截面短边 y 限制），
    四点互成 90°。dev_1.0 分支里的旧实现就是这种；圆周点到不了截面四角。

``center-square``
    复现原 Downloads 脚本的布点：P1 在立方体中心，P2～P5 是 ``z = -k·h`` 平面上
    边长 ``2k·h`` 的正方形四角（``k`` 由 ``--square-frac`` 给，原脚本 ``k=0.4``）。
    保留它是为了与旧结论对照——注意它**只用到一个高度**，Z 方向没有任何覆盖。

用法
----
直接给立方体（最常用）::

    PYTHONPATH=python python3 -m excitation_trajectory.iso9283_cube \\
        --robot data/configs/hyundai_general6s.urdf \\
        --cube 750 1300 -600 600 270 940 \\
        --rpy 3 73 -167 --euler xyz-intrinsic \\
        --q-seed 43,-30,-36,42,-89,-1 --out iso5.json --plot iso5.png

自动搜最大内接立方体（慢一些，默认 6 秒级）::

    PYTHONPATH=python python3 -m excitation_trajectory.iso9283_cube \\
        --robot data/configs/hyundai_general6s.urdf \\
        --search-box 700 1400 -600 600 200 1300 --resolution 100 \\
        --rpy 3 73 -167 --euler xyz-intrinsic

输出的 JSON 与 ``aabb_snake_scan`` 同构（``meta`` / ``waypoints`` /
``perturbed_ik``），所以可以直接接 ``export_gp`` 写 GP 文件——五个点的姿态
完全一致，现场按 MOVL 依次移动。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from excitation_trajectory.aabb_snake_scan import (
    EULER_CONVENTIONS,
    _split_convention,
    config_tag,
)
from robot_model.nexbot_ik import Nexbot6R

#: 布点方式（见模块 docstring）。
LAYOUTS = ("vertices", "circle", "center-square")

#: 可选的对角截面：键 -> (对角轴 a, 对角轴 b, 第三轴) 的下标。
#: 截面是 ``coord[a] == coord[b]`` 的平面，第三轴在截面内自由。
PLANES = {"xz": (0, 2, 1), "yz": (1, 2, 0), "xy": (0, 1, 2)}

#: 五个点的名字，顺序即现场执行顺序。
POINT_NAMES = ("P1", "P2", "P3", "P4", "P5")

#: 自动搜立方体时，边长二分的最少/最多次数。
_SEARCH_BISECT_STEPS = 8


def cube_points(center, side: float, layout: str = "vertices",
                square_frac: float = 0.4, plane: str = "xz") -> dict:
    """按指定布点方式算出 P1～P5 的位置（mm）。

    Args:
        center: 立方体中心 ``(x, y, z)``（mm）。
        side: 立方体边长 L（mm）。
        layout: ``"vertices"`` / ``"circle"`` / ``"center-square"``。
        square_frac: ``center-square`` 布点的 k（相对半边长 h 的比例）。
        plane: 对角截面 ``xz`` / ``yz`` / ``xy``（只影响 vertices 与 circle）。

    Returns:
        ``{"P1": ndarray(3), ..., "P5": ndarray(3)}``。

    Raises:
        ValueError: 边长非正，layout / plane 不认识。
    """
    cube_center = np.asarray(center, dtype=float).reshape(3)
    side = float(side)
    if not math.isfinite(side) or side <= 0.0:
        raise ValueError(f"立方体边长需为有限正数，当前 {side}")
    if layout not in LAYOUTS:
        raise ValueError(f"--layout 需为 {LAYOUTS} 之一，当前 {layout!r}")

    h = side / 2.0

    if layout in ("vertices", "circle"):
        # ISO 9283 的**对角截面**：``coord[a] == coord[b]``，第三轴在截面内自由。
        # 截面是 u∈±h√2（对角方向）× h 的矩形，四个角 = 立方体四个间隔顶点。
        first, second, third = PLANES[plane]
        sign_pairs = ((+1, +1), (+1, -1), (-1, +1), (-1, -1))
        # 对角方向分 h（vertices，取到截面四角）或 h/√2（circle，内切圆 45° 处）
        diagonal = h if layout == "vertices" else h * math.cos(math.pi / 4.0)
        # 直角坐标里对角两轴各占 u/√2；circle 时即 h/2，vertices 时即 h
        along = diagonal / math.sqrt(2.0) if layout == "circle" else h
        across = (h * math.sin(math.pi / 4.0) if layout == "circle" else h)
        offsets = {}
        for name, (sign_diag, sign_third) in zip(POINT_NAMES[1:], sign_pairs):
            offset = [0.0, 0.0, 0.0]
            offset[first] = sign_diag * along
            offset[second] = sign_diag * along
            offset[third] = sign_third * across
            offsets[name] = tuple(offset)
    else:                                   # center-square（原脚本布点，与 --plane 无关）
        k = float(square_frac)
        if not 0.0 < k <= 1.0:
            raise ValueError(f"--square-frac 需在 (0, 1]，当前 {k}")
        plane = -k * h
        offsets = {
            "P2": (-k * h, -k * h, plane),
            "P3": (+k * h, -k * h, plane),
            "P4": (+k * h, +k * h, plane),
            "P5": (-k * h, +k * h, plane),
        }

    points = {"P1": cube_center.copy()}
    for name, offset in offsets.items():
        points[name] = cube_center + np.asarray(offset, dtype=float)
    return points


def cube_vertices(center, side: float) -> list:
    """立方体的 8 个顶点（判断"内接"时用，比只查 5 个点稳）。"""
    cube_center = np.asarray(center, dtype=float).reshape(3)
    h = float(side) / 2.0
    return [cube_center + np.array([sx * h, sy * h, sz * h], dtype=float)
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def solve_point(rbt: Nexbot6R, position, rpy, *, order: str, extrinsic: bool,
                q_ref) -> dict:
    """在给定位置与姿态下求一组 IK（优先与 ``q_ref`` 最近的解）。

    Args:
        position: 法兰位置（mm）。
        rpy: 姿态（deg），五点是同一姿态。
        q_ref: 参考关节角；``None`` 表示无偏好（取第一个解）。

    Returns:
        ``{"ok": bool, "q": list|None, "n_sols": int, "config": list|None,
        "j3_abs": float|None, "j5_abs": float|None}``。
    """
    pose = rbt.pose_to_T(tuple(float(v) for v in position),
                         tuple(float(v) for v in rpy),
                         order=order, extrinsic=extrinsic)
    solutions = rbt.ik(pose, q_ref=q_ref)
    if not solutions:
        return {"ok": False, "q": None, "n_sols": 0, "config": None,
                "j3_abs": None, "j5_abs": None}
    best = [float(v) for v in solutions[0]]
    return {"ok": True, "q": best, "n_sols": len(solutions),
            "config": list(config_tag(rbt, pose, best)),
            "j3_abs": abs(best[2]), "j5_abs": abs(best[4])}


def solve_points(rbt: Nexbot6R, points: dict, rpy, *, order: str,
                 extrinsic: bool, q_seed) -> dict:
    """依次解 P1～P5，用上一解作参考以保持构型连续。

    Returns:
        ``{name: solve_point() 的结果 + "pos": [x,y,z]}``。
    """
    results = {}
    q_ref = None if q_seed is None else [float(v) for v in q_seed]
    for name in POINT_NAMES:
        position = points[name]
        solved = solve_point(rbt, position, rpy, order=order,
                            extrinsic=extrinsic, q_ref=q_ref)
        solved["pos"] = [float(v) for v in position]
        results[name] = solved
        if solved["ok"]:
            q_ref = solved["q"]
    return results


def cube_reachable(rbt: Nexbot6R, center, side: float, rpy, *, order: str,
                   extrinsic: bool, q_seed) -> tuple:
    """立方体是否"内接"：8 个顶点都要在给定姿态下可解。

    Returns:
        ``(all_ok, results)``：``results`` 是 8 个顶点各自的求解结果。
    """
    results = []
    q_ref = None if q_seed is None else [float(v) for v in q_seed]
    for vertex in cube_vertices(center, side):
        solved = solve_point(rbt, vertex, rpy, order=order,
                            extrinsic=extrinsic, q_ref=q_ref)
        solved["pos"] = [float(v) for v in vertex]
        results.append(solved)
        if solved["ok"]:
            q_ref = solved["q"]
    return all(item["ok"] for item in results), results


def find_largest_cube(rbt: Nexbot6R, box, rpy, *, order: str,
                      extrinsic: bool, q_seed, resolution: float = 100.0,
                      prefer_x_min=None) -> dict:
    """在给定搜索箱里找**最大轴对齐内接立方体**。

    做法：按 ``resolution`` 铺一层中心候选，每个中心用二分法求最大可行边长
    （判据：8 个顶点在固定姿态下都可解得 IK）。比原脚本"20 万点随机采样 +
    40mm 体素"更快、更确定，也不受采样密度影响。

    Args:
        box: ``[x0, x1, y0, y1, z0, z1]``（mm）搜索范围。
        prefer_x_min: 只接受中心 X 不小于该值的候选（避开机身后方）。

    Returns:
        ``{"ok": bool, "side": float, "center": [x,y,z], "samples": int,
        "largest_side_seen": float}``。
    """
    x0, x1, y0, y1, z0, z1 = (float(v) for v in box)
    if x0 > x1 or y0 > y1 or z0 > z1:
        raise ValueError(f"搜索箱下界大于上界：{box}")
    resolution = max(1.0, float(resolution))
    steps = [int(round((x1 - x0) / resolution)),
             int(round((y1 - y0) / resolution)),
             int(round((z1 - z0) / resolution))]
    samples = 0
    best = {"ok": False, "side": 0.0, "center": None, "samples": 0,
            "largest_side_seen": 0.0}

    for ix in range(steps[0] + 1):
        for iy in range(steps[1] + 1):
            for iz in range(steps[2] + 1):
                center = np.array([x0 + ix * resolution,
                                   y0 + iy * resolution,
                                   z0 + iz * resolution])
                if prefer_x_min is not None and center[0] < prefer_x_min:
                    continue
                # 上限：到搜索箱六个面的最小距离（立方体不能出箱）
                limit = 2.0 * min(center[0] - x0, x1 - center[0],
                                  center[1] - y0, y1 - center[1],
                                  center[2] - z0, z1 - center[2])
                if limit <= 1.0:
                    continue
                if limit <= best["side"]:
                    continue                # 不可能更好，跳过二分
                low, high = 0.0, limit
                for _ in range(_SEARCH_BISECT_STEPS):
                    middle = (low + high) / 2.0
                    samples += 1
                    ok, _results = cube_reachable(
                        rbt, center, middle, rpy, order=order,
                        extrinsic=extrinsic, q_seed=q_seed)
                    if ok:
                        low = middle
                    else:
                        high = middle
                best["largest_side_seen"] = max(best["largest_side_seen"], low)
                if low > best["side"]:
                    best = {"ok": low > 0.0, "side": float(low),
                            "center": [float(v) for v in center],
                            "samples": 0, "largest_side_seen": float(low)}
    best["samples"] = samples
    return best


def scan_payload(points: dict, results: dict, *, cube, layout: str,
                 side: float, rpy, convention: str, motion_hint: str,
                 plane: str = "xz") -> dict:
    """组装与 ``aabb_snake_scan`` 同构的 JSON，好让 ``export_gp`` 直接吃。"""
    waypoints = []
    chain = []
    for index, name in enumerate(POINT_NAMES):
        solved = results[name]
        waypoints.append({
            "index": index,
            "name": name,
            "pos": [round(float(v), 6) for v in solved["pos"]],
            "rpy_ref": [round(float(v), 6) for v in rpy],
            "rpy_pert": None,           # ISO 五点：姿态完全一致，不扰动
            "delta_rpy": None,
        })
        chain.append({
            "index": index,
            "q": (None if solved["q"] is None
                  else [round(float(v), 6) for v in solved["q"]]),
            "config": solved["config"],
            "dq_l2": None,
            "dq_l1": None,
            "n_sols": int(solved["n_sols"]),
            "ok": bool(solved["ok"]),
        })
    return {
        "meta": {
            "source": "iso9283_cube",
            "layout": layout,
            "plane": plane,
            "cube_center": [round(float(v), 6) for v in cube["center"]],
            "cube_side": round(float(side), 6),
            "aabb": [round(float(v), 6) for v in cube["bounds"]],
            "rpy": [round(float(v), 6) for v in rpy],
            "euler_convention": convention,
            "point_count": len(POINT_NAMES),
            "motion": motion_hint,
            "note": "P1..P5 姿态完全一致；现场按 MOVL 依次移动，"
                    "不要用 PTP（关节直线插补会改变实际路径）",
        },
        "waypoints": waypoints,
        "perturbed_ik": chain,
    }


def _cube_bounds(center, side: float) -> list:
    """立方体的 AABB（写进 meta，便于和扫描文件对照）。"""
    h = float(side) / 2.0
    c = [float(v) for v in center]
    return [c[0] - h, c[0] + h, c[1] - h, c[1] + h, c[2] - h, c[2] + h]


def _parse_triplet(values, name: str) -> list:
    values = [float(v) for v in values]
    if len(values) != 3:
        raise ValueError(f"{name} 需要 3 个数，当前 {len(values)} 个")
    return values


def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.iso9283_cube",
        description="按 ISO 9283 从测试立方体生成 P1~P5 五个定位精度测点"
                    "（输出与 aabb_snake_scan 同构，可直接接 export_gp）。")
    parser.add_argument("--robot", "--urdf", "--robot-config", dest="robot",
                        required=True, help="机器人模型（.urdf 或控制器 .json）")
    source = parser.add_argument_group("立方体来源（二选一）")
    source.add_argument("--cube", nargs=6, type=float, metavar="BOUND",
                        help="直接给立方体：x0 x1 y0 y1 z0 z1（mm）；"
                             "取各轴中点为中心、各轴跨度为边长（需近似各向等长）")
    source.add_argument("--center", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="直接给立方体中心（配合 --side）")
    parser.add_argument("--side", type=float, help="立方体边长 L（mm）")
    parser.add_argument("--auto-cube", action="store_true",
                        help="自动搜最大内接立方体（用 --search-box）")
    parser.add_argument("--search-box", nargs=6, type=float, metavar="BOUND",
                        help="自动搜索范围 x0 x1 y0 y1 z0 z1（mm）")
    parser.add_argument("--resolution", type=float, default=100.0, metavar="MM",
                        help="自动搜索的中心网格间距，默认 100")
    parser.add_argument("--prefer-x-min", type=float, default=None, metavar="MM",
                        help="自动搜索时只接受中心 X >= 该值（避开机身后方）")
    parser.add_argument("--layout", choices=LAYOUTS, default="vertices",
                        help="布点方式，默认 vertices（ISO 9283 对角截面四角）")
    parser.add_argument("--plane", choices=tuple(PLANES), default="xz",
                        help="对角截面：xz（x=z，默认）/ yz（y=z）/ xy（x=y）；"
                             "只影响 --layout vertices 与 circle")
    parser.add_argument("--square-frac", type=float, default=0.4, metavar="K",
                        help="center-square 布点的方形比例 k，默认 0.4（同原脚本）")
    parser.add_argument("--rpy", nargs=3, type=float, default=[180.0, 0.0, -90.0],
                        metavar=("A", "B", "C"), help="五点统一姿态（deg）")
    parser.add_argument("--euler", choices=EULER_CONVENTIONS, type=str.lower,
                        default="xyz-intrinsic", metavar="CONVENTION",
                        help="欧拉约定（**只支持内旋**）：xyz / zyx / zyz，"
                             "默认 xyz-intrinsic")
    parser.add_argument("--q-seed", default="0,20,-20,0,40,0",
                        help="IK 种子关节角，逗号分隔 6 个 deg")
    parser.add_argument("--j5-min", type=float, default=0.0, metavar="DEG",
                        help="|J5| 告警下限（deg），默认 0 只报告不判失败")
    parser.add_argument("--out", default="iso9283_p5.json",
                        help="输出 JSON（与 aabb_snake_scan 同构）")
    parser.add_argument("--plot", default=None,
                        help="可选：输出 3D 预览 PNG（Agg 后端，不弹窗）")
    return parser


def plot_points(points: dict, cube_center, side: float, out_png: str,
                title: str) -> None:
    """画立方体 + 五个点（Agg，写完即 close；无显示环境安全）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    c = np.asarray(cube_center, dtype=float)
    # 立方体 12 条棱
    corners = cube_vertices(c, side)
    edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
             (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
    for first, second in edges:
        a, b = corners[first], corners[second]
        axis.plot(*zip(a, b), color="0.6", linewidth=1.0)
    for name in POINT_NAMES:
        position = points[name]
        axis.scatter(*position, s=60, depthshade=False,
                     label=f"{name} ({position[0]:.0f},{position[1]:.0f},"
                           f"{position[2]:.0f})")
    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title(title)
    axis.legend(loc="upper left", fontsize=8)
    figure.tight_layout()
    figure.savefig(out_png, dpi=130)
    plt.close(figure)


def main(argv=None) -> int:
    """CLI 入口。返回退出码。"""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    order, extrinsic = _split_convention(argparse.Namespace(
        euler_convention=args.euler))

    try:
        q_seed = [float(v) for v in str(args.q_seed).replace(" ", "").split(",")]
        if len(q_seed) != 6:
            raise ValueError(f"--q-seed 需 6 个数，当前 {len(q_seed)}")
        rpy = _parse_triplet(args.rpy, "--rpy")
    except ValueError as error:
        parser.error(str(error))

    # 三种给法：--auto-cube / --cube / (--center 且 --side)
    if not args.auto_cube and args.cube is None:
        if args.center is None or args.side is None:
            parser.error("需要以下之一：--auto-cube --search-box …；"
                         "--cube x0 x1 y0 y1 z0 z1；或 --center X Y Z --side L")

    rbt = Nexbot6R(args.robot)

    # ---- 1. 定立方体 ----
    if args.auto_cube:
        box = args.search_box
        if box is None:
            parser.error("--auto-cube 需要 --search-box x0 x1 y0 y1 z0 z1")
        try:
            found = find_largest_cube(
                rbt, box, rpy, order=order, extrinsic=extrinsic,
                q_seed=q_seed, resolution=args.resolution,
                prefer_x_min=args.prefer_x_min)
        except ValueError as error:
            parser.error(str(error))
        if not found["ok"]:
            print("错误: 搜索范围内找不到任何可解立方体，"
                  "请扩大 --search-box 或降低 --prefer-x-min", file=sys.stderr)
            return 2
        center = found["center"]
        side = found["side"]
        # 对齐到 1mm，便于现场读数
        side = math.floor(side)
        print(f"自动搜索：边长 {side} mm，中心 "
              f"({center[0]:.0f}, {center[1]:.0f}, {center[2]:.0f}) mm，"
              f"评估 {found['samples']} 个候选")
    else:
        if args.center is not None and args.cube is not None:
            parser.error("--center 与 --cube 二选一")
        if args.cube is not None:
            x0, x1, y0, y1, z0, z1 = (float(v) for v in args.cube)
            spans = [x1 - x0, y1 - y0, z1 - z0]
            if min(spans) <= 0:
                parser.error(f"--cube 的下界必须小于上界：{args.cube}")
            side = min(spans)
            center = [(x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0]
            if max(spans) - side > 1e-6:
                print(f"提示: --cube 三轴跨度不等（{spans}），"
                      f"取最小跨度 {side:g} mm 为边长，中心用各轴中点")
        else:
            center = _parse_triplet(args.center, "--center")
            side = float(args.side)
            if not math.isfinite(side) or side <= 0:
                parser.error(f"--side 需为正数，当前 {side}")

    # ---- 2. 出点 + 解 IK ----
    try:
        points = cube_points(center, side, args.layout, args.square_frac,
                             args.plane)
    except ValueError as error:
        parser.error(str(error))

    results = solve_points(rbt, points, rpy, order=order,
                          extrinsic=extrinsic, q_seed=q_seed)
    failed = [name for name in POINT_NAMES if not results[name]["ok"]]
    warnings = []
    for name in POINT_NAMES:
        solved = results[name]
        if solved["ok"] and args.j5_min > 0.0 and solved["j5_abs"] < args.j5_min:
            warnings.append(f"{name} 的 |J5|={solved['j5_abs']:.2f}° "
                            f"低于 {args.j5_min:g}°（腕部接近奇异）")

    print(f"立方体：边长 {side:g} mm，中心 "
          f"({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}) mm，"
          f"布点 {args.layout}")
    print(f"姿态  ：{rpy} deg（{args.euler}），五点一致")
    print("五点：")
    for name in POINT_NAMES:
        solved = results[name]
        head = (f"  {name} pos=({solved['pos'][0]:7.1f},{solved['pos'][1]:7.1f},"
                f"{solved['pos'][2]:7.1f})")
        if not solved["ok"]:
            print(f"{head}  无解 [FAIL]")
            continue
        q_text = ", ".join(f"{v:7.2f}" for v in solved["q"])
        print(f"{head}  q=[{q_text}]  |J3|={solved['j3_abs']:6.2f} "
              f"|J5|={solved['j5_abs']:6.2f}  cfg={solved['config']}")

    configs = {tuple(results[name]["config"]) for name in POINT_NAMES
               if results[name]["ok"]}
    if len(configs) > 1:
        warnings.append(f"五点的构型标签不一致（{sorted(configs)}）："
                        f"现场按 MOVL 走点时可能换分支")
    for line in warnings:
        print(f"警告: {line}")

    cube = {"center": [float(v) for v in center],
            "bounds": _cube_bounds(center, side)}
    payload = scan_payload(points, results, cube=cube, layout=args.layout,
                           side=side, rpy=rpy, convention=args.euler,
                           motion_hint="MOVL", plane=args.plane)
    Path(args.out).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8")
    print(f"已输出 {args.out}（{len(POINT_NAMES)} 点）")

    if args.plot:
        plot_points(points, center, side, args.plot,
                    f"ISO 9283 P1~P5 ({args.layout}, L={side:g} mm)")
        print(f"已输出 {args.plot}")

    if failed:
        print(f"错误: {len(failed)} 个点无解（{','.join(failed)}）——"
              f"缩小立方体或换姿态", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
