#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot_A.json (inexbot 控制器配置) -> 标准 URDF 转换器
====================================================

映射规则 (标准DH -> URDF 关节原点):
  DH:  A_i = Rz(off_i + q_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)
  URDF 链: T = O_1·Rz(q_1) · O_2·Rz(q_2) · ... · O_6·Rz(q_6) · O_tail

  令  O_1    = Rz(off_1)                                   (joint1 原点)
      O_{i+1} = Tz(d_i)·Tx(a_i)·Rx(alpha_i)·Rz(off_{i+1})   (joint2..6 原点)
      O_tail  = Tz(d_6)·Tx(a_6)·Rx(alpha_6)                 (link6->tool0 固定关节)
  则两条链逐项相等, FK 严格等价。

单位: URDF 标准 -- 长度 m, 角度 rad。

字段来源:
  Link&CoupleCoe.Link     -> DH (a/alpha/d/theta偏置)
  Joint[i].NegSWLimit/PosSWLimit -> <limit lower/upper>
  Joint[i].RatedVel (deg/s, 关节侧) -> <limit velocity> (rad/s)
  没有的信息一律留空/置 0:
    - effort 未知 -> 0
    - visual/collision 无 -> 不输出
    - 电机方向 Direction、5-6轴耦合系数 Couple_Coe_5_6、eulerAngle 约定
      无法在 URDF 中表达 -> 写入 XML 注释备查

用法（在仓库根目录）::

    PYTHONPATH=python python -m robot_model.json_to_urdf ROBOT_A_JSON [OUTPUT_URDF]

``ROBOT_A_JSON`` 为必填的控制器配置；未提供 ``OUTPUT_URDF`` 时，默认写入
仓库的 ``data/configs/ch2000_origin.urdf``。
"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


DEFAULT_OUTPUT_URDF = (
    Path(__file__).resolve().parents[2]
    / "data" / "configs" / "ch2000_origin.urdf"
)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def mat_to_rpy(R):
    """旋转矩阵 -> URDF rpy (roll,pitch,yaw), 约定 R = Rz(yaw)Ry(pitch)Rx(roll)。"""
    sp = min(1.0, max(-1.0, -R[2, 0]))
    p = math.asin(sp)
    if abs(abs(sp) - 1.0) < 1e-12:   # 俯仰 ±90° 奇异, 取代表解
        r = math.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    else:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


def origin_element(T, indent="      "):
    """4x4 -> <origin xyz rpy/>(米/弧度, 输入 T 为 mm/弧度)。"""
    xyz = T[:3, 3] / 1000.0
    r, p, y = mat_to_rpy(T[:3, :3])
    r, p, y = r + 0.0, p + 0.0, y + 0.0  # 消除 -0
    return (f'{indent}<origin xyz="{xyz[0]:.17g} {xyz[1]:.17g} {xyz[2]:.17g}" '
            f'rpy="{r:.17g} {p:.17g} {y:.17g}"/>')


def build_arg_parser() -> argparse.ArgumentParser:
    """构建 JSON -> URDF 转换器的命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="将 Nexbot/CH2000 Robot_A.json 转换为标准 URDF。")
    parser.add_argument("source", metavar="ROBOT_A_JSON",
                        help="必填：Robot_A.json 的显式本地路径")
    parser.add_argument("output", metavar="OUTPUT_URDF", nargs="?",
                        default=str(DEFAULT_OUTPUT_URDF),
                        help=("可选：输出 URDF 路径；默认 "
                              "data/configs/ch2000_origin.urdf"))
    return parser


def main(argv: list[str] | None = None):
    args = build_arg_parser().parse_args(argv)
    src = args.source
    dst = args.output

    with open(src, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    link = cfg["Link&CoupleCoe"]["Link"]
    joints = cfg["Joint"]
    cc = cfg["Link&CoupleCoe"].get("CoupleCoe", {})
    couple56 = cc.get("Couple_Coe_5_6", 0.0)
    directions = [j.get("Direction", 1) for j in joints]

    a = [float(l["a"]) for l in link]                    # mm
    alpha = [math.radians(float(l["alpha"])) for l in link]
    d = [float(l["d"]) for l in link]                    # mm
    off = [math.radians(float(l["theta"])) for l in link]

    # ---- 按分解规则构造各关节原点 (mm) ----
    def make_O(i):  # O_{i+1}, i=1..5 (1-based)
        T = np.eye(4)
        T[:3, 3] = [a[i - 1], 0.0, d[i - 1]]
        T[:3, :3] = rot_x(alpha[i - 1]) @ rot_z(off[i])
        return T

    O = [np.eye(4) for _ in range(6)]
    O[0][:3, :3] = rot_z(off[0])                          # O_1 = Rz(off1)
    for i in range(1, 6):
        O[i] = make_O(i)                                  # O_2..O_6
    tail = np.eye(4)                                      # link6 -> tool0
    tail[:3, 3] = [a[5], 0.0, d[5]]
    tail[:3, :3] = rot_x(alpha[5])

    # ---- 生成 URDF 文本 ----
    L = []
    L.append('<?xml version="1.0"?>')
    L.append(f'<robot name="nexbot_8_cantilever">')
    L.append('  <!-- 由 inexbot Robot_A.json 自动转换 (json_to_urdf.py)')
    L.append(f'       robotType={cfg.get("robotType")}  totalAxisNum={cfg.get("totalAxisNum")}'
             f'  mountMethod={cfg["Link&CoupleCoe"].get("mountMethod", 0)}')
    L.append('       标准 DH (mm/deg): a=' + str([v for v in a]) +
             ' alpha=' + str([float(l["alpha"]) for l in link]) +
             ' d=' + str(d) +
             ' thetaOff=' + str([float(l["theta"]) for l in link]))
    L.append('       注意: URDF 无法表达以下配置信息:')
    L.append(f'         * 电机方向 Direction={directions} (模型正方向约定, 需在应用层处理)')
    L.append(f'         * 5-6轴耦合系数 Couple_Coe_5_6={couple56}')
    L.append(f'         * eulerAngle 约定 = {cfg.get("eulerAngle")}')
    L.append('         * effort 未知 -> limit effort="0"')
    L.append('         * 无几何外观信息 -> 不含 visual/collision -->')

    L.append('  <link name="base_link"/>')
    for i in range(1, 7):
        L.append(f'  <link name="link{i}"/>')
    L.append('  <link name="tool0"/>')

    for i in range(6):
        lo, hi = joints[i]["NegSWLimit"], joints[i]["PosSWLimit"]
        vel = math.radians(joints[i].get("RatedVel", 0.0))  # deg/s -> rad/s
        parent = "base_link" if i == 0 else f"link{i}"
        child = f"link{i + 1}"
        L.append(f'  <joint name="joint{i + 1}" type="revolute">')
        L.append(f'    <!-- DH: 前级连杆 a={a[i-1] if i else 0}mm d={d[i-1] if i else 0}mm'
                 f' alpha={math.degrees(alpha[i-1]) if i else 0}deg; 本关节 theta 偏置='
                 f'{math.degrees(off[i])}deg; 限位(原始 deg)=[{lo}, {hi}] -->')
        L.append(origin_element(O[i], indent="    "))
        L.append(f'    <parent link="{parent}"/>')
        L.append(f'    <child link="{child}"/>')
        L.append(f'    <axis xyz="0 0 1"/>')
        L.append(f'    <limit lower="{math.radians(lo):.17g}" upper="{math.radians(hi):.17g}"'
                 f' effort="0" velocity="{vel:.17g}"/>')
        L.append('  </joint>')

    L.append('  <joint name="joint6-tool0" type="fixed">')
    L.append(f'    <!-- DH 末端: a6={a[5]}mm d6={d[5]}mm alpha6={math.degrees(alpha[5])}deg;'
             f' tool0 = DH 系6(法兰) -->')
    L.append(origin_element(tail, indent="    "))
    L.append('    <parent link="link6"/>')
    L.append('    <child link="tool0"/>')
    L.append('  </joint>')
    L.append('</robot>')

    with open(dst, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print(f"已生成: {dst}")

    # ---- 自校验: URDF 链 FK vs DH FK ----
    def dh_mat(theta, dd, aa, al):
        ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(al), math.sin(al)
        return np.array([[ct, -st * ca, st * sa, aa * ct],
                         [st, ct * ca, -ct * sa, aa * st],
                         [0, sa, ca, dd], [0, 0, 0, 1]], dtype=float)

    def rz4(theta):
        T = np.eye(4)
        T[:3, :3] = rot_z(theta)
        return T

    rng = np.random.default_rng(7)
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, 6)
        T_dh = np.eye(4)
        for k in range(6):
            T_dh = T_dh @ dh_mat(q[k] + off[k], d[k], a[k], alpha[k])
        T_urdf = np.eye(4)
        for k in range(6):
            T_urdf = T_urdf @ O[k] @ rz4(q[k])
        T_urdf = T_urdf @ tail
        worst = max(worst, float(np.max(np.abs(T_dh - T_urdf))))
    print(f"自校验 DH-FK vs URDF-FK (200 随机位姿): 最大误差 = {worst:.3e} mm/rad")
    assert worst < 1e-9, "URDF 与 DH 不等价!"
    print("校验通过 ✔  (FK 严格等价)")


if __name__ == "__main__":
    main()
