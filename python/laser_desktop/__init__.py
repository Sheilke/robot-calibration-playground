# -*- coding: utf-8 -*-
"""桌面版（PySide6 + QtWebEngine 嵌 Plotly）。

与网页版的关系：**共用同一套服务层** ``laser_web.core`` / ``jobs`` / ``figures``，
桌面版只负责窗口与控件，不重写任何算法。网页版保留作数值对照与临时救急入口。

模块：

* ``paths``        打包后仍能找到资产/写到可写目录（源码里到处数 ``parents`` 的替代）
* ``qt_compat``    Qt 绑定兼容层（交付 PySide6；Linux 验证退回 PyQt5）
* ``plot_view``    一次加载壳页、之后 ``Plotly.react`` 就地换图
* ``main_window``  左侧导航 + 四页
"""
from . import paths
from .qt_compat import BINDING, DEV_ONLY_BINDING

__all__ = ["paths", "BINDING", "DEV_ONLY_BINDING"]
