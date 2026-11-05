#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 AABB 扫描结果导出为 9011(N9011) 点位下载 + 逐点行走报文。

背景（现场 DH 标定采点，方案 1-F/3-C）：

* 路点来自 ``aabb_snake_scan.py`` 的输出 ``scan_result.json``（含扰动姿态与
  链式 IK 选解）。用**轴坐标**（coord=1）下发可精确复现链式 IK 选定的构型，
  避免控制器自己解笛卡尔 IK 时跳构型。
* 下位 JOB（nexbot_controller_driver/robot_down/*/N9011.JBR，Hyundai 9011 同
  构）是 ENET 定长字符串协议：
  - ``M0`` + 单点（``coord(1) + 每轴 8 字(2 位小数) + slot(4)``）+ movedata：
    **每次报文自带目标点**，单次 MOVE，movedata = ``{P|L}`` + ``speed(7)``
    + ``acc(1)`` + ``tool(1)`` + ``ratio(4)``。默认用这个（无下载状态可失效）；
  - ``G{N}`` + N 个点下载到 P[slot]（每点 53 字，报文上限 240 字符 →
    6 轴每包最多 4 点，且 ``G`` 头计数只有 1 位），再 ``T{begin:04d}{end:04d}``
    引用已下载的 P 号逐点走（``--walk T``）。
* **不用 ``M1``**：subtype 1 是 J6 解缠专用，会（a）每点走**两次**
  （Hyundai S9 + S11；Nexbot MOVJ + MOVJ）、（b）把 J6 强缠进 ±180°
  （改掉 IK 给定的关节值）、（c）Hyundai 侧第二次移动写死 ``S=V12!mm/s``，
  百分比模式（speed<0）会变成负的 mm/s。``M2``/``M3`` 同理（把 J6 推到
  正/负极限）。
* **9011 的移动指令都没有停留参数**（N9011 的 WHILE / Hyundai 的 FOR 循环
  都是 MOVE 完直接下一点）。逐点 6 秒暂停必须由**上位发送循环**实现：每发
  一条单点报文、等运动完成后 sleep ``--dwell`` 秒再发下一条。报文脚本里用
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
#: 单点行走默认用 M0：报文自带目标点，不依赖之前的 G 下载；
#: M1/M2/M3 是 J6 解缠分支（每点走两次），采点不能用。
DEFAULT_WALK = "M0"
WALK_MODES = ("M0", "T")
#: M0 用的暂存 P 号。取 100 与公司驱动 ``kCalibrationPointStatus`` 一致，
#: 避开路径占用槽位 P1..PN。
DEFAULT_SCRATCH_SLOT = 100
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


def single_move_message(slot: int, coord: int, data, move_data: str) -> str:
    """``M0`` 单点报文：``M`` + subtype(1) + 点(53) + movedata(14) + 换行。

    与 ``HyundaiRobot::MovementToMsg`` / ``NexbotProtocol`` 的 M 分支一致：
    目标点随报文下发，**不依赖任何已下载的 P 数组**。
    """
    return f"M0{position_to_str(coord, data, slot)}{move_data}\n"


def walk_messages(slots, move_data: str, dwell_s: float, *,
                  mode: str = DEFAULT_WALK, coords_data=None) -> list:
    """生成逐点移动报文 + 上位暂停标注（下位移动指令无停留参数）。

    Args:
        slots: 每点的 P 编号（``T`` 模式下是下载槽号；``M0`` 模式下是暂存槽）。
        move_data: :func:`move_data_to_str` 的结果。
        dwell_s: 点到点之间的暂停秒数（写进注释，由上位执行）。
        mode: ``"M0"``（默认，自带坐标）或 ``"T"``（引用已下载 P）。
        coords_data: ``M0`` 模式必需，``[(coord, data), ...]`` 与 ``slots``
            一一对应。
    """
    if mode not in WALK_MODES:
        raise ValueError(f"mode 需为 {WALK_MODES} 之一，当前 {mode!r}")
    lines = []
    for i, slot in enumerate(slots, start=1):
        if mode == "M0":
            if coords_data is None:
                raise ValueError("M0 模式需要 coords_data（每点坐标）")
            coord, data = coords_data[i - 1]
            lines.append(single_move_message(slot, coord, data, move_data))
        else:
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
    parser.add_argument("--walk", choices=WALK_MODES, default=DEFAULT_WALK,
                        help="逐点移动的报文形式：M0=每次自带目标点（默认，"
                             "不依赖预下载，G 丢包也不会走到旧 P）；T=引用 G 下载"
                             "的 P 号。M1/M2/M3 是 J6 解缠分支，故不提供")
    parser.add_argument("--scratch-slot", type=int,
                        default=DEFAULT_SCRATCH_SLOT,
                        help=f"M0 模式用的暂存 P 号（默认 "
                             f"{DEFAULT_SCRATCH_SLOT}，避开路径槽位 P1..PN）")
    parser.add_argument("--slot-start", type=int, default=SLOT_START,
                        help="T 模式下首个 P 变量编号（默认 1，与驱动一致）")
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
    if not (1 <= args.scratch_slot <= 9999):
        parser.error(f"--scratch-slot 需在 1..9999，当前 {args.scratch_slot}")
    if args.walk == "T" and args.scratch_slot != DEFAULT_SCRATCH_SLOT:
        parser.error("--scratch-slot 仅用于 M0 模式，T 模式请用 --slot-start")

    scan_path = Path(args.scan)
    points = load_scan_points(scan_path)
    ext = [] if args.ext_axis is None else [float(v) for v in args.ext_axis]

    wire_points = []
    for item in points:
        data = list(item["q"]) + list(ext)
        wire_points.append((1, data))          # coord=1 轴坐标
    try:
        move_data = move_data_to_str(args.move_type, args.speed, args.acc,
                                     args.tool, args.ratio)
        if args.walk == "T":
            chunks = group_messages(wire_points, args.slot_start)
            slots = [s for chunk_slots, _ in chunks for s in chunk_slots]
        else:
            chunks = []
            slots = [args.scratch_slot] * len(wire_points)
        walk = walk_messages(slots, move_data, args.dwell, mode=args.walk,
                             coords_data=wire_points)
    except ValueError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    lines = []
    if chunks:
        lines.append("# ---- 1) 点位下载（G 分包，逐行发送，等回包后再发下一条） ----\n")
        lines += [msg for _, msg in chunks]
        lines.append(f"# ---- 2) 逐点行走（共 {len(slots)} 点；每条 T 只走 1 点） ----\n")
    else:
        lines.append(
            f"# ---- 逐点行走（共 {len(slots)} 点；每条 M0 自带目标点，"
            f"不依赖预下载的 P，movedata={move_data}） ----\n")
    lines += walk
    lines.append("# ---- 结束退出 ----\nQ0\n")
    Path(args.messages).write_text("".join(lines), encoding="utf-8")

    payload = {
        "meta": {
            "source_scan": str(scan_path),
            "n_points": len(slots),
            "walk_mode": args.walk,
            "slot_start": args.slot_start if args.walk == "T" else None,
            "scratch_slot": args.scratch_slot if args.walk == "M0" else None,
            "coord": "axis",
            "dwell_s": args.dwell,
            "note": ("轴坐标=模型关节角(度)，方向映射 identity（如现场 Direction "
                     "非全 1 需先换算）；逐点暂停由上位在每条移动报文后执行；"
                     "不用 M1/M2/M3（J6 解缠分支，每点走两次）"),
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

    if args.walk == "T":
        print(f"点位 {len(slots)} 个（P{slots[0]}..P{slots[-1]}），G 报文 {len(chunks)} 包")
    else:
        print(f"点位 {len(slots)} 个（每条 M0 自带坐标，暂存 P{args.scratch_slot}）")
    print(f"JSON:   {args.out}")
    print(f"报文:   {args.messages}")
    print(f"行走:   每点一条单步 {args.walk}，运动完成后 sleep {args.dwell:g} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
