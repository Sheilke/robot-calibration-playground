# -*- coding: utf-8 -*-
"""``laser_web`` 的路径解析：**只读资产根**在哪。

为什么需要这个模块
------------------
``core`` 里原本是 ``_REPO_ROOT = Path(__file__).resolve().parents[2]``。源码模式下
它确实是仓库根，因为文件在 ``<repo>/python/laser_web/core.py``。但 PyInstaller 把
模块打进归档后，``__file__`` 的形状变成 ``<_MEIPASS>/laser_web/core``（**没有
``.py`` 后缀**，loader 用 ``sys._MEIPASS`` 拼 origin），于是：

.. code-block:: text

    __file__                  <app>/_internal/laser_web/core
    parents[1]                <app>/_internal            ← _MEIPASS
    parents[2] = _REPO_ROOT   <app>                      ← 安装目录，不是资产根
    模型实际在                <app>/_internal/assets/data/configs/…

结果：下拉框里选得出模型，一点「生成并解算」就 ``FileNotFoundError``。这类故障
在源码模式下**永远复现不了**，只会在装完之后出现，所以路径不能靠 ``parents[N]``
数层级。

真值与假值
----------
"冻结"的唯一可信判据是 ``sys.frozen``（PyInstaller 会设），而不是 ``hasattr(sys,
'_MEIPASS')`` —— 后者在某些嵌入场景下也会出现，但资产并没有被打进 ``_MEIPASS``。
``_MEIPASS`` 只用来定位**打包后的**资源目录。

与 ``laser_desktop.paths`` 的关系
---------------------------------
桌面端的 ``laser_desktop/paths.py`` 还管"可写数据根"（``runs/``、缓存、用户设置，
Linux 上可能是仓库根、Windows 上是 ``%LOCALAPPDATA%``），那是另一件事。**只读资产
根**只在这里定义一次，桌面端转调本模块，避免两处各算一遍、冻结时算出两种答案。
"""

from __future__ import annotations

import sys
from pathlib import Path

#: 打包后资产相对 ``_MEIPASS`` 的位置（与 ``packaging/desktop.spec`` 的收集目标一致）。
BUNDLE_ASSETS = "assets"


def is_frozen() -> bool:
    """当前是否跑在 PyInstaller 冻结产物里。"""
    return bool(getattr(sys, "frozen", False))


def bundle_root() -> Path:
    """冻结产物里的 ``_MEIPASS``（onedir 模式下是 ``<app>/_internal``）。"""
    meipass = getattr(sys, "_MEIPASS", None)
    if not meipass:
        # 冻结了却没有 _MEIPASS：宁可指到可执行文件所在目录，也不要静默用 cwd。
        return Path(sys.executable).resolve().parent
    return Path(meipass)


def repo_root() -> Path:
    """源码模式下的仓库根（``<repo>/python/laser_web/paths.py`` 往上三级）。"""
    return Path(__file__).resolve().parents[2]


def asset_root() -> Path:
    """**只读**资产根：机器人模型、随包的 ``plotly.min.js``、模板等。

    冻结后是 ``<_MEIPASS>/assets``（``desktop.spec`` 把 ``data/configs`` 与
    ``data/assets/plotly`` 收在这里）；源码模式是仓库根，与历史行为一致。
    """
    if is_frozen():
        return bundle_root() / BUNDLE_ASSETS
    return repo_root()
