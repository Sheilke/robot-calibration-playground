# -*- coding: utf-8 -*-
"""表单默认值工厂。

每次调用 :func:`default_form_values` 都返回**全新的 dict**（含全新 list），
页面把它存在自己的局部变量里，因此多标签页/刷新不会串台；本模块**不保存
任何可变状态**，只有不可变常量。

默认值全部从 ``excitation_trajectory.aabb_snake_scan`` 的 CLI 常量取，
不在这里另抄一份数字，保证界面与命令行同默认。
"""

from __future__ import annotations

from pathlib import Path

from excitation_trajectory import aabb_snake_scan as scan

#: 采样域输入方式：两点对角点 / 六个轴边界。
DOMAIN_MODES = ("points", "axes")
#: 网格来源：目标点数自动凑数 / 手填各轴份数（与 CLI 的互斥规则一致）。
GRID_MODES = ("count", "explicit")
#: 表单里作为示例的采样域（设计文档 §5.1 的例子，550 x 1200 x 670 mm）。
SAMPLE_POINT_MIN = (750.0, -600.0, 270.0)
SAMPLE_POINT_MAX = (1300.0, 600.0, 940.0)


def repo_root() -> Path:
    """仓库根：由 CLI 默认模型路径反推（``<repo>/data/configs/<model>.urdf``）。"""
    return scan.DEFAULT_URDF.parents[2]


def default_robot() -> str:
    """CLI 默认模型的仓库相对路径（与 ``core.list_robots`` 的 ``path`` 同格式）。"""
    return scan.DEFAULT_URDF.relative_to(repo_root()).as_posix()


def euler_conventions() -> list[str]:
    """欧拉约定下拉框的选项（``xyz-intrinsic`` 等 3 种内旋），来自算法模块。"""
    return list(scan.EULER_CONVENTIONS)


def default_form_values() -> dict:
    """返回一份新的表单默认值 dict。

    键与 ``core.preview_domain`` 的入参一致，另外带三个界面状态：
    ``domain_mode``（两点/六数）、``grid_mode``（凑数/手填）、
    ``target_count`` 等非激活项的预填值（不激活时不会提交给 core）。

    Returns:
        dict: 全新构造的默认值，调用方可以随意改。
    """
    point_min, point_max = SAMPLE_POINT_MIN, SAMPLE_POINT_MAX
    return {
        "robot": default_robot(),
        "domain_mode": DOMAIN_MODES[0],
        "point_min": [float(v) for v in point_min],
        "point_max": [float(v) for v in point_max],
        "aabb6": [float(point_min[0]), float(point_max[0]),
                  float(point_min[1]), float(point_max[1]),
                  float(point_min[2]), float(point_max[2])],
        "rpy": [float(v) for v in scan.DEFAULT_RPY],
        "euler": scan.DEFAULT_EULER_CONVENTION,
        "perturb_deg": [float(v) for v in scan.PERTURB_AMPLITUDE_DEG],
        "grid_mode": GRID_MODES[1],
        "target_count": 50,
        "xs": int(scan.DEFAULT_GRID[0]),
        "ys": int(scan.DEFAULT_GRID[1]),
        "zs": int(scan.DEFAULT_GRID[2]),
        "z_insert": 0,
        "cond_max": float(scan.DEFAULT_COND_MAX),
    }
