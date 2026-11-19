# -*- coding: utf-8 -*-
"""路径解析：打包成 exe **之后**仍然能找到数据、写到可写目录。

为什么必须单独一层
------------------
现有代码（`laser_web/core.py`、`jobs.py`、`ui_state.py`、`aabb_snake_scan.py`、
`export_gp.py`、`nexbot_ik.py`）都用 ``Path(__file__).parents[2]`` 反推仓库根。
源码运行时没问题，但 PyInstaller 冻结后 ``__file__`` 落在 ``_internal/`` 里，
推出来是个不存在的路径——于是模型找不到、``runs/`` 也建不出来。

桌面版一律走这里的函数，不再自己数 ``parents``：

=====================  ==========================  =============================
用途                   源码运行                     冻结（PyInstaller）
=====================  ==========================  =============================
只读资产（模型配置）    仓库根/data/configs          ``_MEIPASS/assets/data/configs``
可写数据（runs/ 缓存）  仓库根/runs、仓库根/.cache    用户数据目录（Windows: %LOCALAPPDATA%）
=====================  ==========================  =============================

``LASER_HOME`` 环境变量可整体覆盖可写数据目录（便携部署/U 盘使用时有用）。
"""
from __future__ import annotations

import os
import platform
import sys
from pathlib import Path

#: 可写数据目录的环境变量名（给出即覆盖默认位置）。
ENV_HOME = "LASER_HOME"

#: 应用标识（决定 Windows 上 ``%LOCALAPPDATA%\<名字>`` 的最后一级）。
APP_ID = "LaserWaypoint"

#: 冻结包里只读资产的子目录名（PyInstaller ``--add-data`` 的目标）。
BUNDLE_ASSETS = "assets"


def is_frozen() -> bool:
    """当前是否跑在 PyInstaller/同类打包产物里。"""
    return bool(getattr(sys, "frozen", False))


def bundle_root() -> Path:
    """打包产物的临时解包目录（``_MEIPASS``）；源码运行时是仓库根。"""
    if is_frozen():
        return Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
    return repo_root()


def repo_root() -> Path:
    """仓库根（仅源码运行时有意义）。"""
    return Path(__file__).resolve().parents[2]


def asset_root() -> Path:
    """**只读**资产根：模型配置、随包的 plotly.min.js 等。

    转调 :func:`laser_web.paths.asset_root`：只读资产根只在那边定义一次。
    ``core`` 也要用它拼模型路径，两边各算一遂迟早算出两种答案（曾经就是
    这样：桌面端算对了， ``core._REPO_ROOT`` 却在冻结后指向安装目录）。
    """
    from laser_web import paths as web_paths

    return web_paths.asset_root()


def data_root() -> Path:
    """**可写**数据根：``runs/``、缓存、用户设置。

    优先级：``LASER_HOME`` → 冻结时用户数据目录 → 源码时仓库根。
    """
    override = os.environ.get(ENV_HOME)
    if override:
        return Path(override).expanduser().resolve()
    if not is_frozen():
        return repo_root()
    return _platform_data_dir()


def _platform_data_dir() -> Path:
    """各平台约定的用户数据目录（Windows 优先 ``LOCALAPPDATA``）。"""
    system = platform.system()
    if system == "Windows":
        base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA")
        if base:
            return Path(base) / APP_ID
        return Path.home() / "AppData" / "Local" / APP_ID
    if system == "Darwin":
        return Path.home() / "Library" / "Application Support" / APP_ID
    base = os.environ.get("XDG_DATA_HOME")
    if base:
        return Path(base) / APP_ID
    return Path.home() / ".local" / "share" / APP_ID


def config_dir() -> Path:
    """机器人模型目录（``data/configs``）——只读。"""
    return asset_root() / "data" / "configs"


def runs_root() -> Path:
    """运行产物目录（与网页版共用同一个 ``runs/``，便于两版对照）。"""
    return data_root() / "runs"


def cache_root() -> Path:
    """缓存目录（Plotly 壳页、plotly.min.js、临时文件）。"""
    return data_root() / ".cache"


def ensure_writable_roots() -> dict:
    """确保可写目录存在，返回它们的路径（启动时调一次）。"""
    result = {}
    for name, path in (("data", data_root()), ("runs", runs_root()),
                       ("cache", cache_root())):
        path.mkdir(parents=True, exist_ok=True)
        result[name] = path
    return result


def describe() -> dict:
    """把解析结果列出来（``--print-paths`` 与故障排查用）。"""
    return {"frozen": is_frozen(),
            "bundle": str(bundle_root()),
            "assets": str(asset_root()),
            "configs": str(config_dir()),
            "data": str(data_root()),
            "runs": str(runs_root()),
            "cache": str(cache_root())}
