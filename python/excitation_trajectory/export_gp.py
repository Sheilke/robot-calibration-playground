#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把扫描路点写成下位全局位置寄存器（GPxxxx）的存储文件。

控制器把 GP 变量存在 ``position_R1.json`` 之类的文件里，每个 GP 是 14 个
数值。字段布局由现场已创建的一条样例反推（用户确认「1 52 …6 deg 是 1 到
6 轴」）::

    idx 0..5   笛卡尔 x y z rx ry rz
    idx 6      标志位（样例为 0）
    idx 7..12  关节角 J1..J6（度）
    idx 13     外部轴（6 轴站为 0）

所以 ``[0,0,0,0,0,0, 0, 1,52,3,4,5,6, 0]`` 表示 J1..J6 = 1/52/3/4/5/6。

用法（在仓库根）::

    PYTHONPATH=python python3 -m excitation_trajectory.export_gp \\
        scan_result.json /home/infoli/Documents/backup/position_R1.json \\
        -o /tmp/position_R1_gp1_50.json --first 1

注意：写的是**副本**，模板文件本身不会被修改；先备份、再在示教器上抽查
一个 GP 确认关节值回读正确，最后再整体导入。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

#: 每个 GP 的数值个数。
GP_FIELD_COUNT = 14
#: 字段区段：笛卡尔 6 个、标志位 1 个、关节 6 个、外部轴 1 个。
CART_SLICE = slice(0, 6)
FLAG_INDEX = 6
JOINT_SLICE = slice(7, 13)
EXT_INDEX = 13
#: 标志位默认值：与现场已创建样例一致。
DEFAULT_FLAG = 0


def load_scan_points(scan_path) -> list:
    """从 ``aabb_snake_scan`` 输出里取 IK 成功的路点（按路点顺序）。"""
    payload = json.loads(Path(scan_path).read_text(encoding="utf-8"))
    chain = payload.get("perturbed_ik") or payload.get("ik_chain")
    if not chain:
        raise SystemExit("scan 文件里没有 perturbed_ik/ik_chain，"
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


def check_joint_range(q_deg, where: str) -> list:
    """关节角必须有限且落在 (-360, 360)（GP 里存的是绝对角度）。"""
    values = [float(v) for v in q_deg]
    if len(values) != 6:
        raise ValueError(f"{where} 需 6 个关节角，当前 {len(values)} 个")
    for axis, value in enumerate(values, 1):
        if not math.isfinite(value):
            raise ValueError(f"{where} J{axis} 非有限值：{value!r}")
        if not -360.0 < value < 360.0:
            raise ValueError(f"{where} J{axis}={value} 超出 (-360, 360)")
    return values


def make_gp_entry(q_deg, *, cart=(0.0,) * 6, flag: int = DEFAULT_FLAG,
                  ext: float = 0.0) -> list:
    """按 14 字段布局构造一个 GP 条目。"""
    joints = check_joint_range(q_deg, "q_deg")
    cart_values = [float(v) for v in cart]
    if len(cart_values) != 6:
        raise ValueError(f"cart 需 6 个分量，当前 {len(cart_values)} 个")
    entry = [0.0] * GP_FIELD_COUNT
    entry[CART_SLICE] = cart_values
    entry[FLAG_INDEX] = float(flag)
    entry[JOINT_SLICE] = joints
    entry[EXT_INDEX] = float(ext)
    if len(entry) != GP_FIELD_COUNT:
        raise ValueError("GP 条目长度错误")
    return entry


def write_gp_entries(template, points, *, first: int = 1,
                     flag: int = DEFAULT_FLAG,
                     cartesian_of=None) -> tuple:
    """在模板文件内容上写入 GP 条目，返回 (新内容, 报告)。

    Args:
        template: 已解析的 ``position_R1.json`` 内容（``dict``）。
        points: :func:`load_scan_points` 的结果。
        first: 起始 GP 编号（1 基）。
        flag: 标志位取值。
        cartesian_of: 可选 ``q_deg -> (x,y,z,rx,ry,rz)``；``None`` 时笛卡尔
            部分写 0（与现场样例一致）。
    """
    if not isinstance(template, dict):
        raise ValueError("模板文件顶层需为对象")
    registers = template.get("globalPosition")
    if not isinstance(registers, dict):
        raise ValueError("模板里没有 globalPosition 对象")

    first = int(first)
    if first < 0:
        raise ValueError(f"--first 需 >= 0，当前 {first}")
    written = []
    for offset, point in enumerate(points):
        slot = first + offset
        name = f"GP{slot:04d}"
        cart = (0.0,) * 6
        if cartesian_of is not None:
            cart = cartesian_of(point["q"])
        registers[name] = make_gp_entry(point["q"], cart=cart, flag=flag)
        written.append({"gp": name, "point_index": point["index"],
                        "q_deg": point["q"]})
    return template, {"count": len(written), "first": f"GP{first:04d}",
                      "last": f"GP{first + len(written) - 1:04d}",
                      "entries": written}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.export_gp",
        description="把扫描路点写入下位 GP 寄存器存储文件的副本（14 字段布局："
                    "笛卡尔 6 + 标志 1 + 关节 6 + 外轴 1）。")
    parser.add_argument("scan", help="aabb_snake_scan 输出的 scan_result.json")
    parser.add_argument("template", help="控制器现成的 position_R1.json（只读，不修改）")
    parser.add_argument("-o", "--out", required=True, help="输出文件（副本）")
    parser.add_argument("--first", type=int, default=1,
                        help="起始 GP 编号（1 基），默认 1")
    parser.add_argument("--flag", type=int, default=DEFAULT_FLAG,
                        help=f"14 字段布局里 idx 6 的标志位，默认 {DEFAULT_FLAG}")
    parser.add_argument("--cartesian", choices=("zero", "fk"), default="zero",
                        help="笛卡尔部分写 0（默认，与现场样例一致）或用模型 FK 填")
    parser.add_argument("--report", default=None, help="额外输出 JSON 报告")
    args = parser.parse_args(argv)

    points = load_scan_points(args.scan)
    template_path = Path(args.template)
    try:
        template = json.loads(template_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"错误: 读模板失败 {template_path}: {error}", file=sys.stderr)
        return 2

    cartesian_of = None
    if args.cartesian == "fk":
        from handeye.dh import DhOverride, RobotKinematics
        from handeye.pose_encoder import matrix_to_pose_fields
        urdf = Path(__file__).resolve().parents[2] / "data" / "configs" \
            / "hyundai_general6s.urdf"
        robot = RobotKinematics(urdf, DhOverride.from_base(urdf))

        def cartesian_of(q_deg):
            position, orientation = matrix_to_pose_fields(robot.fk(q_deg))
            return list(position) + list(orientation)

    try:
        updated, report = write_gp_entries(template, points, first=args.first,
                                           flag=args.flag, cartesian_of=cartesian_of)
    except ValueError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    out_path = Path(args.out)
    out_path.write_text(json.dumps(updated, ensure_ascii=False, indent=1) + "\n",
                        encoding="utf-8")
    if args.report:
        Path(args.report).write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"写入 {report['count']} 个 GP：{report['first']} .. {report['last']}")
    print(f"模板 {template_path.name}（未修改） -> 输出 {out_path}")
    first_entry = updated["globalPosition"][report["first"]]
    print(f"{report['first']} = {[round(v, 3) for v in first_entry]}")
    print("提示：先在示教器抽查一个 GP 回读的关节值，再整体导入控制器")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
