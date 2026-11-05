#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 AABB 扫描结果导出为 9011(N9011) 点位下载 + 逐点行走报文。

背景（现场 DH 标定采点，方案 1-F/3-C）：

* 路点来自 ``aabb_snake_scan.py`` 的输出 ``scan_result.json``（含扰动姿态与
  链式 IK 选解）。用**轴坐标**（coord=1）下发可精确复现链式 IK 选定的构型，
  避免控制器自己解笛卡尔 IK 时跳构型。
* 下位 JOB（nexbot_controller_driver/robot_down/*/N9011.JBR，Hyundai 9011 同
  构）是 ENET 定长字符串协议：
  - ``G{N}`` + N 个点（每点 ``coord(1) + 6×8 字(2 位小数) + P 编号(4)``），
    报文上限 240 字符 → 6 轴点每包最多 4 个，本工具自动分包；
  - ``T{begin:04d}{end:04d}{movedata}`` 逐点走 P[begin..end]，
    movedata = ``{P|L}`` + ``speed(7,2 位小数)`` + ``acc(1)`` + ``tool(1)``
    + ``ratio(4, 百分数)``。
* **9011 的 T 循环没有停留参数**（N9011.JBR 的 WHILE 循环 MOVE 后直接下一
  点；Hyundai 9011.JOB 同）。逐点 6 秒暂停必须由**上位发送循环**实现：每发
  一条单点 T、等运动完成后 sleep ``--dwell`` 秒再发下一条。报文脚本里用
  注释行标注暂停点。

用法（仓库根）::

    PYTHONPATH=python python3 -m excitation_trajectory.export_9011 \\
        scan_result.json --out /tmp/calib_points.json \\
        --messages /tmp/calib_9011.txt --dwell 6.0

字段宽度与 C++ 驱动（``nexbot_protocol.cpp`` / ``hyundai_robot.cpp`` 的
``DoubleToStr``/``IntToStr``/``MoveDataToStr``）逐字符一致，可直接喂给
公司仓库的发送链路，也可按 messages 文本逐行发送。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

# 与 nexbot_protocol.hpp MaxMessageLength() 一致
MAX_MESSAGE_LENGTH = 240
# 与 robot.h kNexbotPathRegisterStart 一致（P 变量从 1 号开始）
SLOT_START = 1
DEFAULT_DWELL_S = 6.0
DEFAULT_SPEED = -1.0          # <0 → 百分比模式（N9011 的 MOVJ 固定 VJ=5%）
DEFAULT_ACC = 0
DEFAULT_TOOL = 0
DEFAULT_RATIO_PCT = 10


def _fixed_float(value: float, width: int = 8, decimals: int = 2) -> str:
    """C++ ``DoubleToStr`` 等价：负号单独拼接 + ``width-1`` 宽零填充。"""
    value = float(value)
    if value < 0:
        text = "-" + f"{-value:0{width - 1}.{decimals}f}"
    else:
        text = f"{value:0{width}.{decimals}f}"
    if len(text) != width:
        raise ValueError(f"数值 {value} 格式化后 {text!r} 超出 {width} 字宽")
    return text


def _fixed_int(value: int, width: int) -> str:
    """C++ ``IntToStr`` 等价：负号单独拼接 + ``width-1`` 宽零填充。"""
    value = int(value)
    if value < 0:
        text = "-" + f"{-value:0{width - 1}d}"
    else:
        text = f"{value:0{width}d}"
    if len(text) != width:
        raise ValueError(f"整数 {value} 格式化后 {text!r} 超出 {width} 字宽")
    return text


def position_to_str(coord: int, data, slot: int) -> str:
    """单点 wire 形式：coord(1) + data×8 字 + slot(4)。"""
    fields = [_fixed_int(coord, 1)]
    fields += [_fixed_float(v) for v in data]
    fields.append(_fixed_int(slot, 4))
    return "".join(fields)


def move_data_to_str(move_type: str = "P", speed: float = DEFAULT_SPEED,
                     acc: int = DEFAULT_ACC, tool: int = DEFAULT_TOOL,
                     ratio_pct: int = DEFAULT_RATIO_PCT) -> str:
    """运动参数 wire 形式：type(1) + speed(7) + acc(1) + tool(1) + ratio(4)。"""
    return (f"{move_type}{_fixed_float(speed, width=7)}"
            f"{_fixed_int(acc, 1)}{_fixed_int(tool, 1)}"
            f"{_fixed_int(ratio_pct, 4)}")


def group_messages(points, slot_start: int = SLOT_START) -> list:
    """点位打包为 G 报文（自动按 240 字符上限分包，与驱动一致）。

    Args:
        points: 每项为 (coord, data) ；data 为 6（或 6+外轴）个数值。
        slot_start: 首个 P 变量编号（默认 1）。

    Returns:
        ``[(slots, message_line), ...]``，message 含结尾换行。每包点数同时
        受两个限制：报文长度 ≤ 240（含换行），且点数 ≤ 9（JOB 侧计数仅 1 位）。
    """
    chunks, current, current_slots = [], [], []
    slot = slot_start
    for coord, data in points:
        single = position_to_str(coord, data, slot)
        would_be = 2 + sum(len(p) for p in current) + len(single) + 1
        if current and (would_be > MAX_MESSAGE_LENGTH or len(current) >= 9):
            chunks.append((list(current_slots),
                           f"G{len(current)}{''.join(current)}\n"))
            current, current_slots = [], []
        current.append(single)
        current_slots.append(slot)
        slot += 1
    if current:
        chunks.append((list(current_slots),
                       f"G{len(current)}{''.join(current)}\n"))
    return chunks


def walk_messages(slots, move_data: str, dwell_s: float) -> list:
    """逐点单步 T 报文 + 上位暂停标注（T 循环本身无停留参数）。"""
    lines = []
    for i, slot in enumerate(slots, start=1):
        lines.append(f"T{_fixed_int(slot, 4)}{_fixed_int(slot, 4)}{move_data}\n")
        if i < len(slots):
            lines.append(f"# [{i}/{len(slots)}] 等运动完成后 sleep {dwell_s:g} s 再发下一条\n")
    return lines


def load_scan_points(scan_path: Path) -> list:
    """从 scan_result.json 取 IK 成功的路点（模型轴角，度，路径序）。"""
    payload = json.loads(Path(scan_path).read_text(encoding="utf-8"))
    chain = payload.get("perturbed_ik") or payload.get("ik_chain")
    if not chain:
        raise SystemExit("scan_result.json 里没有 perturbed_ik/ik_chain 结果，"
                         "请先用不带 --skip-ik 的模式跑一遍")
    points = []
    for step in chain:
        if not step.get("ok") or step.get("q") is None:
            continue
        points.append({"index": int(step["index"]),
                       "q": [float(v) for v in step["q"]],
                       "config": step.get("config")})
    if not points:
        raise SystemExit("没有 IK 成功的路点")
    return points


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.export_9011",
        description="AABB 扫描结果 -> 9011(N9011) 点位下载与逐点行走报文。"
                    "轴坐标下发保持链式 IK 构型；逐点 6 秒暂停由上位发送循环实现。")
    parser.add_argument("scan", help="aabb_snake_scan 输出的 scan_result.json")
    parser.add_argument("--out", default="calib_points_9011.json",
                        help="结构化点位 JSON 输出路径")
    parser.add_argument("--messages", default="calib_9011_messages.txt",
                        help="可直接发送的报文文本（含暂停标注注释行）")
    parser.add_argument("--dwell", type=float, default=DEFAULT_DWELL_S,
                        help="逐点暂停秒数（默认 6），写进 JSON 与报文注释，"
                             "由上位发送循环执行")
    parser.add_argument("--move-type", choices=("P", "L"), default="P",
                        help="P=关节(MOVJ，N9011 固定 VJ=5%%)，L=直线(MOVL，"
                             "用 --speed)，默认 P")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                        help="mm/s；负值=百分比模式（默认 -1）")
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC,
                        help="平滑等级 PL（0-9），默认 0")
    parser.add_argument("--tool", type=int, default=DEFAULT_TOOL,
                        help="工具号（0=Tool0 法兰），默认 0")
    parser.add_argument("--ratio", type=int, default=DEFAULT_RATIO_PCT,
                        help="速度百分比（movedata 末 4 位），默认 10")
    parser.add_argument("--slot-start", type=int, default=SLOT_START,
                        help="首个 P 变量编号（默认 1，与驱动一致）")
    parser.add_argument("--ext-axis", nargs="*", type=float, default=None,
                        metavar="EXT",
                        help="7/8/9 轴站的外轴坐标（每点相同，逐个追加；"
                             "6 轴站不填）")
    args = parser.parse_args(argv)

    if not math.isfinite(args.dwell) or args.dwell <= 0:
        parser.error(f"--dwell 需为正数，当前 {args.dwell}")
    if not (0 <= args.acc <= 9):
        parser.error(f"--acc 需在 0..9，当前 {args.acc}")
    if not (0 <= args.ratio <= 9999):
        parser.error(f"--ratio 需在 0..9999，当前 {args.ratio}")

    scan_path = Path(args.scan)
    points = load_scan_points(scan_path)
    ext = [] if args.ext_axis is None else [float(v) for v in args.ext_axis]

    wire_points = []
    for item in points:
        data = list(item["q"]) + list(ext)
        wire_points.append((1, data))          # coord=1 轴坐标
    try:
        chunks = group_messages(wire_points, args.slot_start)
        move_data = move_data_to_str(args.move_type, args.speed, args.acc,
                                     args.tool, args.ratio)
    except ValueError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    slots = [s for chunk_slots, _ in chunks for s in chunk_slots]
    walk = walk_messages(slots, move_data, args.dwell)

    lines = ["# ---- 1) 点位下载（G 分包，逐行发送，等回包后再发下一条） ----\n"]
    lines += [msg for _, msg in chunks]
    lines.append(f"# ---- 2) 逐点行走（共 {len(slots)} 点；每条 T 只走 1 点） ----\n")
    lines += walk
    lines.append("# ---- 3) 结束退出 ----\nQ0\n")
    Path(args.messages).write_text("".join(lines), encoding="utf-8")

    payload = {
        "meta": {
            "source_scan": str(scan_path),
            "n_points": len(slots),
            "slot_start": args.slot_start,
            "coord": "axis",
            "dwell_s": args.dwell,
            "note": ("轴坐标=模型关节角(度)，方向映射 identity（如现场 Direction "
                     "非全 1 需先换算）；逐点暂停由上位在每条 T 后执行"),
            "move_data": {"type": args.move_type, "speed": args.speed,
                          "acc": args.acc, "tool": args.tool,
                          "ratio_pct": args.ratio},
        },
        "points": [
            {"slot": slot, "index": item["index"], "coord": 1,
             "data": data, "config": item["config"]}
            for slot, item, (_, data) in zip(slots, points, wire_points)
        ],
    }
    Path(args.out).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8")

    print(f"点位 {len(slots)} 个（P{slots[0]}..P{slots[-1]}），"
          f"G 报文 {len(chunks)} 包")
    print(f"JSON:   {args.out}")
    print(f"报文:   {args.messages}")
    print(f"行走:   每点一条单步 T，运动完成后 sleep {args.dwell:g} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
