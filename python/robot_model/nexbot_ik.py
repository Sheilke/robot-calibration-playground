#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nexbot 6R 机械臂 解析逆运动学（IK）模块
=========================================

依据运动学模型构建, 支持两种输入:
  1) 标准 URDF (推荐, 由 json_to_urdf.py 从 Robot_A.json 转换生成)
  2) inexbot 控制器配置 Robot_A.json (原始格式)
(示例: nexbot_8_cantilever/Robot_A.json, robotType=R_GENERAL_6S, totalAxisNum=6)

URDF 加载流程: 解析 base_link->tool0 运动链 -> 反解出标准 DH 参数
  -> 用通用链 FK 与 DH FK 交叉校验 -> 检查球型腕结构 -> 启用解析 IK

从配置中提取并使用的字段 (JSON 路径; URDF 中对应 <joint> 字段):
  - Link&CoupleCoe.Link      : 6 组标准 DH 参数 (a, alpha, d, theta 偏置), 单位 mm / 度
  - Joint                    : 各轴软限位 NegSWLimit/PosSWLimit, 电机方向 Direction
  - CoupleCoe.Couple_Coe_5_6 : 5-6 轴耦合系数(0.11481), 提供选配补偿接口
  - eulerAngle               : "XYZ" 欧拉角约定(提供 定轴/动轴 两种解释)
  (URDF 无法表达 Direction/耦合系数/eulerAngle, 加载时取默认并在 dump 中标注)

结构说明:
  - a4 = d5 = a5 = 0  =>  4/5/6 轴交汇于腕点, 球型腕, 可用 Pieper 分解求封闭解
  - 一个可达位姿最多 8 组解: theta1(前后) x 肘(上下) x 腕(正反)
  - 每组返回解都经过 FK 正反馈验证, 保证 fk(ik(T)) == T

单位约定: 关节角/欧拉角 [度], 坐标 [mm]; 内部计算用弧度。

用法示例:
    from robot_model.nexbot_ik import Nexbot6R
    rbt = Nexbot6R("data/configs/ch2000_origin.urdf")  # 或显式 Robot_A.json
    T = rbt.fk([0, 30, -40, 0, 60, 0])              # 正解 -> 4x4
    sols = rbt.ik(T, q_ref=[0, 0, 0, 0, 30, 0])     # 逆解, 按与参考位姿距离排序
    pos, abc = rbt.T_to_pose(T)                     # 位姿 (RPY/定轴XYZ)
    T = rbt.pose_to_T([x, y, z], [A, B, C])

命令行（在仓库根目录）:
    PYTHONPATH=python python -m robot_model.nexbot_ik
    PYTHONPATH=python python -m robot_model.nexbot_ik --selftest 5000
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


DEFAULT_URDF = (
    Path(__file__).resolve().parents[2]
    / "data" / "configs" / "ch2000_origin.urdf"
)

# ---------------------------------------------------------------------------
# 基础旋转矩阵
# ---------------------------------------------------------------------------

def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """标准 DH 齐次变换矩阵 A = Rz(theta) Tz(d) Tx(a) Rx(alpha)。"""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def wrap_pi(a: float) -> float:
    """归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_180(deg: float) -> float:
    """角度差/角度归一化到 (-180, 180]（度）。"""
    return math.degrees(wrap_pi(math.radians(deg)))


def ang_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """逐元素角度差 a-b, 归一化到 (-pi, pi]。"""
    return np.array([wrap_pi(x - y) for x, y in zip(a, b)])


# ---------------------------------------------------------------------------
# 欧拉角 <-> 旋转矩阵
# ---------------------------------------------------------------------------

def euler_to_rot(abc_deg, order: str = "XYZ", extrinsic: bool = True) -> np.ndarray:
    """
    欧拉角 -> 旋转矩阵。

    order + extrinsic 的两种常用解释 (abc_deg = [A, B, C]):
      - extrinsic=True  (定轴/固定坐标系, 教学上常称 RPY):
            先绕固定 X 转 A, 再绕固定 Y 转 B, 再绕固定 Z 转 C,
            R = Rz(C) @ Ry(B) @ Rx(A)          # "XYZ" -> 逆序相乘
      - extrinsic=False (动轴/随体坐标系):
            先绕动 X 转 A, 再绕动 Y 转 B, 再绕动 Z 转 C,
            R = Rx(A) @ Ry(B) @ Rz(C)

    注: 配置文件 eulerAngle="XYZ" 在不同厂商处实现不一, 若与示教器显示不一致,
        请切换 extrinsic 旗标或 order。
    """
    axes = {"X": rot_x, "Y": None, "Z": rot_z}
    # 构建 Y 旋转
    def rot_y(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
    axes["Y"] = rot_y

    a, b, c = [math.radians(v) for v in abc_deg]
    mats = [axes[order[0]](a), axes[order[1]](b), axes[order[2]](c)]
    R = np.eye(3)
    if extrinsic:
        for M in mats:                # 定轴: 逆序相乘
            R = M @ R
    else:
        for M in mats:                # 动轴: 顺序相乘
            R = R @ M
    return R


def rot_to_euler(R: np.ndarray, order: str = "XYZ", extrinsic: bool = True):
    """旋转矩阵 -> 欧拉角 [A,B,C] (度)。与 euler_to_rot 互逆, 奇异位姿取代表解。"""
    R = np.asarray(R, dtype=float)
    if order == "XYZ" and extrinsic:
        # R = Rz(C) Ry(B) Rx(A)
        sp = min(1.0, max(-1.0, -R[2, 0]))
        B = math.asin(sp)
        if abs(abs(sp) - 1.0) < 1e-9:  # 俯仰奇异
            A = math.atan2(-R[1, 2], R[1, 1])
            C = 0.0
        else:
            A = math.atan2(R[2, 1], R[2, 2])
            C = math.atan2(R[1, 0], R[0, 0])
        return [math.degrees(A), math.degrees(B), math.degrees(C)]
    if order == "XYZ" and not extrinsic:
        # R = Rx(A) Ry(B) Rz(C)
        sb = min(1.0, max(-1.0, R[0, 2]))
        B = math.asin(sb)
        if abs(abs(sb) - 1.0) < 1e-9:
            A = math.atan2(-R[1, 2], R[1, 1])
            C = 0.0
        else:
            A = math.atan2(-R[1, 2], R[2, 2])
            C = math.atan2(-R[0, 1], R[0, 0])
        return [math.degrees(A), math.degrees(B), math.degrees(C)]
    raise NotImplementedError(f"暂只实现 XYZ 两种解释 (extrinsic={extrinsic})")


# ---------------------------------------------------------------------------
# URDF 解析 (运动链 -> 标准 DH 反解)
# ---------------------------------------------------------------------------

def _urdf_origin_to_T(joint_elem) -> np.ndarray:
    """URDF <joint> 的 <origin xyz rpy> -> 4x4 (mm/弧度)。约定 R = Rz(y)Ry(p)Rx(r)。"""
    o = joint_elem.find("origin")
    xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0")).split()]) * 1000.0
    r, p, y = [float(v) for v in (o.get("rpy", "0 0 0")).split()]  # 已是弧度
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def _urdf_chain(root):
    """返回从基座到末端 leaf 的关节元素列表 (按运动顺序)。"""
    joints = list(root.findall("joint"))
    links = [l.get("name") for l in root.findall("link")]
    child_of = {j.find("child").get("link"): j for j in joints}
    by_parent = {j.find("parent").get("link"): j for j in joints}
    roots = [l for l in links if l not in child_of]
    if not roots:
        raise ValueError("URDF 无基座 link (无父关节的 link)")
    base = "base_link" if "base_link" in roots else roots[0]
    chain, cur = [], base
    while cur in by_parent:
        j = by_parent[cur]
        chain.append(j)
        cur = j.find("child").get("link")
    return chain


def parse_urdf(path: str) -> dict:
    """
    解析标准 URDF (米/弧度) -> 运动学参数 dict (mm/度)。

    要求: 单链 6 个 revolute 关节, axis=(0,0,1), 且各关节原点满足
    标准 DH 可分解性 (由 json_to_urdf.py 生成的 URDF 天然满足):
        O_1     = Rz(off_1)
        O_{i+1} = Tz(d_i)Tx(a_i)Rx(alpha_i)Rz(off_{i+1})
        tail    = Tz(d_6)Tx(a_6)Rx(alpha_6)   (joint6 之后的所有 fixed 关节)
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(path).getroot()
    chain = _urdf_chain(root)

    # 组合相邻 fixed 关节, 得到 6 个 revolute 的等效原点 + 尾部固定变换
    O_list, tail, acc = [], np.eye(4), np.eye(4)
    for j in chain:
        T = _urdf_origin_to_T(j)
        if j.get("type") in ("revolute", "continuous"):
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None \
                else np.array([0.0, 0.0, 1.0])
            if not np.allclose(axis, [0, 0, 1], atol=1e-9):
                raise NotImplementedError(
                    f"关节 {j.get('name')} 的 axis={axis.tolist()} != (0,0,1), "
                    "本解析器仅支持标准 DH 型 URDF")
            O_list.append(acc @ T)
            acc = np.eye(4)
        elif j.get("type") == "fixed":
            acc = acc @ T
        else:
            raise NotImplementedError(f"不支持的关节类型: {j.get('type')}")
    tail = acc
    if len(O_list) != 6:
        raise NotImplementedError(f"期望 6 个旋转关节, 实际 {len(O_list)} 个")

    # ---- 反解 DH ----
    a = [0.0] * 6
    d = [0.0] * 6
    alpha = [0.0] * 6
    off = [0.0] * 6

    # O_1 = Rz(off1): 纯绕 Z
    O1 = O_list[0]
    if not (np.allclose(O1[:3, 3], 0, atol=1e-6) and
            abs(O1[0, 2]) < 1e-9 and abs(O1[1, 2]) < 1e-9 and
            abs(O1[2, 0]) < 1e-9 and abs(O1[2, 1]) < 1e-9):
        raise ValueError("joint1 原点不是纯 Rz 变换, 与标准 DH 分解不符")
    off[0] = math.atan2(O1[1, 0], O1[0, 0])

    def rx_rz_decompose(R):
        """R = Rx(alpha)@Rz(theta) -> (alpha, theta)。"""
        al = math.atan2(-R[1, 2], R[2, 2])
        sal = math.sin(al)
        if abs(sal) > 1e-8:
            th = math.atan2(R[2, 0] / sal, R[2, 1] / sal)
        else:
            th = math.atan2(R[1, 0], R[0, 0])
        return al, th

    # O_{i+1} -> (a_i, d_i, alpha_i, off_{i+1})
    for i in range(1, 6):
        O = O_list[i]
        if abs(O[1, 3]) > 1e-6:
            raise ValueError(f"joint{i + 1} 原点含 Y 向平移, 与标准 DH 分解不符")
        a[i - 1], d[i - 1] = O[0, 3], O[2, 3]
        al, th = rx_rz_decompose(O[:3, :3])
        alpha[i - 1], off[i] = al, th
        if not np.allclose(rot_x(al) @ rot_z(th), O[:3, :3], atol=1e-9):
            raise ValueError(f"joint{i + 1} 原点旋转无法分解为 Rx(alpha)Rz(theta)")

    # tail -> (a_6, d_6, alpha_6), 纯 Rx
    if abs(tail[1, 3]) > 1e-6:
        raise ValueError("末端固定关节含 Y 向平移")
    a[5], d[5] = tail[0, 3], tail[2, 3]
    alpha[5] = math.atan2(-tail[1, 2], tail[2, 2])
    if not np.allclose(rot_x(alpha[5]), tail[:3, :3], atol=1e-9):
        raise ValueError("末端固定关节旋转不是纯 Rx")

    # ---- 限位 (rad -> deg), 速度 (rad/s, 仅记录) ----
    limits = []
    velocities = []
    names = []
    for j in chain:
        if j.get("type") in ("revolute", "continuous"):
            lim = j.find("limit")
            lo = float(lim.get("lower", "-3.141592653589793")) if lim is not None \
                else -math.pi
            hi = float(lim.get("upper", "3.141592653589793")) if lim is not None \
                else math.pi
            limits.append((math.degrees(lo), math.degrees(hi)))
            velocities.append(float(lim.get("velocity", 0.0)) if lim is not None else 0.0)
            names.append(j.get("name"))

    return {
        "dh_a": a, "dh_alpha": alpha, "dh_d": d, "dh_off": off,
        "limits": limits, "directions": [1] * 6,   # URDF 不含电机方向
        "couple_5_6": 0.0,                          # URDF 不含耦合系数
        "euler_order": None,                        # URDF 不含欧拉角约定
        "robot_type": root.get("name", "?"),
        "source": f"URDF: {path}",
        "chain_O": O_list, "chain_tail": tail,
        "joint_names": names, "velocities": velocities,
    }


def parse_json(path: str) -> dict:
    """解析 inexbot Robot_A.json -> 运动学参数 dict。"""
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    links = cfg["Link&CoupleCoe"]["Link"]
    joints = cfg["Joint"]
    cc = cfg["Link&CoupleCoe"].get("CoupleCoe", {})
    return {
        "dh_a": [float(l["a"]) for l in links],
        "dh_alpha": [math.radians(float(l["alpha"])) for l in links],
        "dh_d": [float(l["d"]) for l in links],
        "dh_off": [math.radians(float(l["theta"])) for l in links],
        "limits": [(float(j["NegSWLimit"]), float(j["PosSWLimit"])) for j in joints],
        "directions": [int(j.get("Direction", 1)) for j in joints],
        "couple_5_6": float(cc.get("Couple_Coe_5_6", 0.0)),
        "euler_order": str(cfg.get("eulerAngle", "XYZ")),
        "robot_type": cfg.get("robotType", "?"),
        "source": f"JSON: {path}",
        "chain_O": None, "chain_tail": None,
        "joint_names": [f"joint{i + 1}" for i in range(6)],
        "velocities": [math.radians(float(j.get("RatedVel", 0.0))) for j in joints],
    }


# ---------------------------------------------------------------------------
# 机械臂: FK + 解析 IK
# ---------------------------------------------------------------------------

class Nexbot6R:
    def __init__(self, config_path: str):
        if str(config_path).lower().endswith(".urdf"):
            p = parse_urdf(config_path)
        else:
            p = parse_json(config_path)

        assert len(p["dh_a"]) == 6, "本模块针对 6 轴本体 (R_GENERAL_6S)"
        self.dh_a = [float(v) for v in p["dh_a"]]                    # mm
        self.dh_alpha = [float(v) for v in p["dh_alpha"]]           # rad
        self.dh_d = [float(v) for v in p["dh_d"]]                   # mm
        self.dh_off = [float(v) for v in p["dh_off"]]               # rad

        self.limits = [(float(lo), float(hi)) for lo, hi in p["limits"]]
        self.directions = [int(v) for v in p["directions"]]
        self.couple_5_6 = float(p["couple_5_6"])
        self.euler_order = p["euler_order"]
        self.mount_method = 0
        self.robot_type = p["robot_type"]
        self.total_axis_num = 6
        self.source = p["source"]
        self.joint_names = p["joint_names"]
        self.velocities = p["velocities"]
        self._chain_O = p["chain_O"]
        self._chain_tail = p["chain_tail"]

        # 解析解常用连杆参数
        (self.a1, self.a2, self.a3) = (self.dh_a[0], self.dh_a[1], self.dh_a[2])
        (self.d1, self.d2, self.d4, self.d6) = (
            self.dh_d[0], self.dh_d[1], self.dh_d[3], self.dh_d[5])
        self.alpha1 = self.dh_alpha[0]
        self.alpha3 = self.dh_alpha[2]
        self.alpha4 = self.dh_alpha[3]
        self.alpha5 = self.dh_alpha[4]

        self._check_analytic_structure()
        if self._chain_O is not None:
            self._verify_against_chain()

    # ---------------- 加载校验 ----------------

    def _check_analytic_structure(self):
        """封闭解适用条件: alpha=[90,0,90,90,-90,0], 球型腕 (a4=d5=a5=0)。"""
        tol = 1e-6
        expect_alpha = [math.pi / 2, 0.0, math.pi / 2,
                        math.pi / 2, -math.pi / 2, 0.0]
        for i, (ea, aa) in enumerate(zip(expect_alpha, self.dh_alpha)):
            if abs(aa - ea) > tol:
                raise NotImplementedError(
                    f"alpha[{i + 1}]={math.degrees(aa):.3f}deg 期望 {math.degrees(ea):.1f}deg, "
                    "封闭解仅支持当前结构型")
        for i in (3, 4):  # a4, a5 (0-based)
            if abs(self.dh_a[i]) > 1e-6:
                raise NotImplementedError("腕部非球型 (a4/a5 != 0), 无封闭解")
        if abs(self.dh_d[4]) > 1e-6:
            raise NotImplementedError("腕部非球型 (d5 != 0), 无封闭解")

    def fk_chain(self, q_deg) -> np.ndarray:
        """URDF 通用链 FK (不经 DH, 用于交叉校验)。"""
        if self._chain_O is None:
            raise RuntimeError("当前模型非 URDF 加载, 无链数据")
        T = np.eye(4)
        for O, q in zip(self._chain_O, [math.radians(v) for v in q_deg]):
            T = T @ O
            Rz4 = np.eye(4)
            Rz4[:3, :3] = rot_z(q)
            T = T @ Rz4
        return T @ self._chain_tail

    def _verify_against_chain(self):
        """DH-FK vs URDF链-FK 随机位姿交叉校验, 确认 DH 反解无误。"""
        rng = np.random.default_rng(0)
        for _ in range(50):
            q = [rng.uniform(lo, hi) for lo, hi in self.limits]
            if not np.allclose(self.fk(q), self.fk_chain(q), atol=1e-6):
                raise RuntimeError("URDF 链 FK 与反解 DH 的 FK 不一致, 解析结果不可信!")

    # ---------------- 正运动学 ----------------

    def fk(self, q_deg) -> np.ndarray:
        """6 个关节角(度) -> 基坐标到法兰 4x4 齐次矩阵 (mm)。"""
        q = [math.radians(float(v)) for v in q_deg]
        T = np.eye(4)
        for i in range(6):
            T = T @ dh_matrix(q[i] + self.dh_off[i], self.dh_d[i],
                              self.dh_a[i], self.dh_alpha[i])
        return T

    # ---------------- 逆运动学 (封闭解) ----------------

    def ik(self, T, q_ref=None, enforce_limits: bool = True):
        """
        位姿矩阵 -> 关节角列表(度)。

        步骤 (Pieper 分解, 球型腕):
          1) 腕点 W = p - d6*z_T
          2) theta1 : 由 (W - o2) 垂直于 z1 ->  Wx*s1 - Wy*c1 = d2, 两解
          3) theta2/theta3 : 臂平面内等效 2 连杆 (a2 与等效前臂 sqrt(a3^2+d4^2)),
             肘上/下两解;  q2 = theta2_dh - 90度偏置
          4) theta4/5/6 : Rw = R03^T R = Rz(4)Rx(90)Rz(5)Rx(-90)Rz(6)
             = ZYZ 型, 腕正/反两解
        => 最多 8 组解; 每组解先 FK 校验再返回。

        q_ref: 参考关节角(度), 提供时结果按关节距离平方升序排列(第一组最接近)。
        enforce_limits: 按配置软限位过滤。
        """
        T = np.asarray(T, dtype=float)
        p = T[:3, 3]
        R = T[:3, :3]
        if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
            raise ValueError("输入的旋转部分不是正交矩阵")

        W = p - self.d6 * R[:, 2]          # 腕点(4/5/6 轴交点)
        Wx, Wy = W[0], W[1]
        rho = math.hypot(Wx, Wy)
        if rho < abs(self.d2) + 1e-9:
            return []  # 腕点离基座回转轴太近, 无解/奇异

        s = min(1.0, max(-1.0, self.d2 / rho))
        beta = math.asin(s)
        phi = math.atan2(Wy, Wx)

        m = math.hypot(self.a3, self.d4)            # 等效前臂长
        gamma = math.atan2(-self.d4, self.a3)       # 等效前臂安装角

        sols = []
        for th1 in (phi + beta, phi + math.pi - beta):
            R1 = rot_z(th1) @ rot_x(self.alpha1)
            o1 = np.array([self.a1 * math.cos(th1),
                           self.a1 * math.sin(th1), self.d1])
            o2 = o1 + self.d2 * R1[:, 2]
            r1 = R1.T @ (W - o2)                    # 腕点在 1 号系下坐标
            rx, ry = r1[0], r1[1]
            if abs(r1[2]) > 1e-6 * max(1.0, np.max(np.abs(r1))):
                continue                             # 数值不一致, 跳过
            # 等效 2 连杆: r = a2*(cosθ2,sinθ2) + m*(cos(θ2+δ), sin(θ2+δ)),
            # 其中 δ = θ3 + γ 为等效肘角 (等效前臂与 x3 夹角 γ = atan2(-d4, a3))
            cd = (rx * rx + ry * ry - self.a2 ** 2 - m ** 2) / (2 * self.a2 * m)
            if abs(cd) > 1.0 + 1e-9:
                continue                             # 超出臂展, 该分支无解
            cd = min(1.0, max(-1.0, cd))
            for sg3 in (1.0, -1.0):                  # 肘上/肘下
                delta = sg3 * math.acos(cd)
                th3 = delta - gamma                  # 真实关节角 θ3
                th2 = math.atan2(ry, rx) - math.atan2(
                    m * math.sin(delta), self.a2 + m * math.cos(delta))  # θ2'(DH)
                R03 = R1 @ rot_z(th2) @ rot_z(th3) @ rot_x(self.alpha3)
                Rw = R03.T @ R                       # 腕部等效旋转 (ZYZ 型)
                c5 = min(1.0, max(-1.0, Rw[2, 2]))
                for sg5 in (1.0, -1.0):              # 腕正/腕反
                    th5 = sg5 * math.acos(c5)
                    s5 = math.sin(th5)
                    if abs(s5) < 1e-8:
                        # 腕奇异: theta4/6 不独立(仅和/差可定), 取 theta4=0
                        th4 = 0.0
                    else:
                        th4 = math.atan2(-sg5 * Rw[1, 2], -sg5 * Rw[0, 2])
                    R46 = rot_z(th4) @ rot_x(self.alpha4) @ \
                        rot_z(th5) @ rot_x(self.alpha5)
                    M = R46.T @ Rw                   # == Rz(th6)
                    th6 = math.atan2(M[1, 0], M[0, 0])
                    q = [wrap_pi(v) for v in
                         (th1, th2 - self.dh_off[1], th3, th4, th5, th6)]
                    sols.append(np.array(q))

        # 去重
        uniq = []
        for q in sols:
            if not any(np.max(np.abs(ang_diff(q, u))) < 1e-6 for u in uniq):
                uniq.append(q)

        # FK 校验: 只返回真正到达目标位姿的解
        ok = []
        for q in uniq:
            Tq = self.fk(np.degrees(q))
            if (np.max(np.abs(Tq[:3, :3] - R)) < 1e-6 and
                    np.max(np.abs(Tq[:3, 3] - p)) < 1e-4):
                ok.append(q)

        # 软限位过滤
        if enforce_limits:
            lim = [(math.radians(lo), math.radians(hi)) for lo, hi in self.limits]
            ok = [q for q in ok
                  if all(lo - 1e-9 <= q[i] <= hi + 1e-9
                         for i, (lo, hi) in enumerate(lim))]

        # 按与参考位姿的距离排序
        if q_ref is not None:
            qr = np.radians([float(v) for v in q_ref])
            ok.sort(key=lambda q: float(np.sum(ang_diff(q, qr) ** 2)))

        return [np.degrees(q) for q in ok]

    # ---------------- 位姿便捷接口 ----------------

    def pose_to_T(self, pos, abc, order: str = "XYZ", extrinsic: bool = True):
        """[x,y,z](mm) + 欧拉角(度) -> 4x4。默认定轴 XYZ(RPY)。"""
        T = np.eye(4)
        T[:3, :3] = euler_to_rot(abc, order=order, extrinsic=extrinsic)
        T[:3, 3] = pos
        return T

    def T_to_pose(self, T, order: str = "XYZ", extrinsic: bool = True):
        """4x4 -> ([x,y,z], [A,B,C])。默认定轴 XYZ(RPY)。"""
        T = np.asarray(T, dtype=float)
        return T[:3, 3].copy(), rot_to_euler(T[:3, :3], order=order, extrinsic=extrinsic)

    # ---------------- 电机轴映射 / 耦合 (选配) ----------------

    def motor_to_model(self, q_motor):
        """电机/编码器轴角 -> 模型关节角。假设 model = motor * Direction。
        (Direction 取自配置 Joint[i].Direction, 为 ±1; 具体符号约定请实机核对)"""
        return [float(q) * d for q, d in zip(q_motor, self.directions)]

    def model_to_motor(self, q_model):
        """模型关节角 -> 电机/编码器轴角 (Direction 为 ±1, 正反互逆)。"""
        return [float(q) * d for q, d in zip(q_model, self.directions)]

    def apply_couple_5_6(self, q_model, sign: float = -1.0):
        """5-6 轴耦合补偿(选配): 若控制器下发的 6 轴指令含 5 轴耦合项, 通常形如
              q6_cmd = q6 -/+ Couple_Coe_5_6 * q5
        返回补偿后的指令角。默认 sign=-1 (q6 - coe*q5)。
        耦合方向与系数含义请以控制器实际行为核对后再启用!"""
        q = list(q_model)
        q[5] = q[5] + sign * self.couple_5_6 * q[4]
        return q

    # ---------------- 信息打印 ----------------

    def dump(self):
        print(f"模型来源: {self.source}")
        print(f"robotType={self.robot_type}  totalAxisNum={self.total_axis_num}  "
              f"eulerAngle={self.euler_order if self.euler_order else '(URDF未含, 默认XYZ)'}")
        print("DH (标准DH, a/alpha/d 单位mm/度, theta 为关节零位偏置):")
        print("  i |     a | alpha |      d | theta_off")
        for i in range(6):
            print(f"  {i+1} | {self.dh_a[i]:5.1f} | {math.degrees(self.dh_alpha[i]) + 0.0:5.1f} | "
                  f"{self.dh_d[i]:6.1f} | {math.degrees(self.dh_off[i]) + 0.0:5.1f}")
        print("软限位(度) / 方向 / 关节速度:")
        for i, (lo, hi) in enumerate(self.limits):
            print(f"  J{i+1} ({self.joint_names[i]}): [{lo:g}, {hi:g}]  "
                  f"Direction={self.directions[i]}"
                  f"{' (URDF未含,默认1)' if self.directions[i] == 1 else ''}  "
                  f"vel={self.velocities[i]:.3f} rad/s")
        note = "" if self.couple_5_6 else " (URDF未含,默认0)"
        print(f"Couple_Coe_5_6 = {self.couple_5_6}{note}")


# ---------------------------------------------------------------------------
# 自测
# ---------------------------------------------------------------------------

def selftest(rbt: Nexbot6R, n: int = 2000, seed: int = 0):
    rng = np.random.default_rng(seed)
    fails = 0
    total_sols = 0
    n_empty = 0
    worst_pose_err = 0.0

    for k in range(n):
        q = [float(rng.uniform(lo, hi)) for lo, hi in rbt.limits]
        T = rbt.fk(q)
        sols = rbt.ik(T, q_ref=q, enforce_limits=True)
        if not sols:
            n_empty += 1
            print(f"[FAIL] 无解: q={np.round(q, 3).tolist()}")
            fails += 1
            continue
        total_sols += len(sols)
        # 每组解 FK 误差
        for s in sols:
            Ts = rbt.fk(s)
            err = max(np.max(np.abs(Ts[:3, :3] - T[:3, :3])) * 1000.0,   # rad->mrad
                      np.max(np.abs(Ts[:3, 3] - T[:3, 3])))              # mm
            worst_pose_err = max(worst_pose_err, err)
        # 原始解必须在返回值中
        if not any(all(abs(wrap_180(a - b)) < 1e-3 for a, b in zip(q, s))
                   for s in sols):
            fails += 1
            print(f"[FAIL] 原解丢失: q={np.round(q, 3).tolist()} "
                  f"返回 {len(sols)} 组")

    # 欧拉角往返 (两种解释; 中间角取非奇异区间)
    e_fails = 0
    for _ in range(200):
        abc = rng.uniform(-179, 179, 3)
        abc[1] = rng.uniform(-89, 89)  # 俯仰角避开 ±90° 万向锁
        for ext in (True, False):
            R = euler_to_rot(abc, extrinsic=ext)
            back = rot_to_euler(R, extrinsic=ext)
            if max(abs(wrap_180(a - b)) for a, b in zip(abc, back)) > 1e-6:
                e_fails += 1

    # 不可达位姿检测 (远超臂展)
    T_far = rbt.pose_to_T([9000.0, 0.0, 500.0], [0, 0, 0])
    far_sols = rbt.ik(T_far)

    print("-" * 64)
    print(f"自测: {n} 个限位内随机位姿")
    print(f"  失败(无解/原解丢失): {fails}")
    print(f"  平均解数(限位内): {total_sols / max(1, n - n_empty):.2f}")
    print(f"  最大 FK 复验误差: {worst_pose_err:.3e} (mrad / mm)")
    print(f"  欧拉角往返失败: {e_fails}")
    print(f"  远端不可达位姿解数: {len(far_sols)} (应为 0)")
    print("自测" + ("通过 ✔" if fails == 0 and e_fails == 0 and not far_sols else "未通过 ✘"))
    return fails == 0 and e_fails == 0 and not far_sols


def demo(rbt: Nexbot6R):
    rbt.dump()
    print("-" * 64)

    q_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    T0 = rbt.fk(q_home)
    pos0, rpy0 = rbt.T_to_pose(T0)
    print(f"FK@零位: pos={np.round(pos0, 2).tolist()} mm, "
          f"RPY(定轴XYZ)={np.round(rpy0, 3).tolist()} deg")

    # 演示目标位姿
    q_demo = [10.0, 35.0, -45.0, 20.0, 60.0, -30.0]
    T = rbt.fk(q_demo)
    pos, rpy = rbt.T_to_pose(T)
    print(f"\n演示: q={q_demo}")
    print(f"      FK -> pos={np.round(pos, 2).tolist()}, RPY={np.round(rpy, 2).tolist()}")

    sols = rbt.ik(T, q_ref=[0, 20, -20, 0, 40, 0])
    print(f"      IK -> 限位内 {len(sols)} 组解 (按接近参考 [0,20,-20,0,40,0] 排序):")
    for i, s in enumerate(sols):
        print(f"        #{i+1}: {np.round(s, 3).tolist()}")

    if sols:
        Ts = rbt.fk(sols[0])
        print(f"      取 #1 复验 FK 误差: "
              f"rot={np.max(np.abs(Ts[:3,:3]-T[:3,:3])):.2e}, "
              f"pos={np.max(np.abs(Ts[:3,3]-T[:3,3])):.2e} mm")


def main():
    ap = argparse.ArgumentParser(
        description="Nexbot 6R 解析 IK (支持 ch2000_origin.urdf 或 Robot_A.json)")
    ap.add_argument("--config", default=str(DEFAULT_URDF),
                    help="模型文件路径: .urdf 或 Robot_A.json")
    ap.add_argument("--selftest", type=int, metavar="N", default=0,
                    help="随机往返自测样本数")
    args = ap.parse_args()

    rbt = Nexbot6R(args.config)
    if args.selftest:
        ok = selftest(rbt, n=args.selftest)
        sys.exit(0 if ok else 1)
    demo(rbt)


if __name__ == "__main__":
    main()
