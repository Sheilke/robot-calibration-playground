# -*- coding: utf-8 -*-
"""Qt 绑定兼容层：**交付用 PySide6（LGPL）**，Linux 上退回 PyQt5 只为本地验证。

为什么需要它
------------
PySide6 的 Linux wheel **不含 QtWebEngine**（实测 55 个 Qt 模块里没有
``QtWebEngineWidgets``），只有 Windows 版带。所以：

* 交付代码按 PySide6 写、按 PySide6 打包；
* 在 Ubuntu 上验证时用系统 ``python3-pyqt5.qtwebengine``（GPL，**不进交付**）。

两个绑定的 API 几乎一致，差异集中在三处：信号名（``Signal`` vs ``pyqtSignal``）、
枚举访问（``Qt.AlignCenter`` 两边都能用）、以及 ``QWebEngineView`` 的 import 路径。
本模块把这三点抹平，页面代码只 import 这里。

**重要**：``QtWebEngineWidgets`` 必须在 ``QApplication`` 创建**之前**导入，否则
Qt5 会报 "QtWebEngineWidgets must be imported before a QCoreApplication instance
is created"。所以任何入口都要先 ``import laser_desktop.qt_compat`` 再建 app。
"""
from __future__ import annotations

import os

#: 实际使用的绑定名（"PySide6" 或 "PyQt5"），日志/关于页会显示它。
BINDING = ""

#: 是否处于"仅为本地验证"的模式（PyQt5 = GPL，不得交付）。
DEV_ONLY_BINDING = False


def _import_pyside6():
    """优先 PySide6（交付用）。"""
    global BINDING, DEV_ONLY_BINDING
    from PySide6 import QtCore, QtGui, QtWidgets
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtWebEngineCore import QWebEngineSettings
    BINDING, DEV_ONLY_BINDING = "PySide6", False
    return (QtCore, QtGui, QtWidgets, QWebEngineView, QWebEngineSettings,
            QtCore.Signal)


def _import_pyqt5():
    """退回 PyQt5（Linux 本地验证；GPL，不进交付）。"""
    global BINDING, DEV_ONLY_BINDING
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
    BINDING, DEV_ONLY_BINDING = "PyQt5", True
    return (QtCore, QtGui, QtWidgets, QWebEngineView, QWebEngineSettings,
            QtCore.pyqtSignal)


try:
    (QtCore, QtGui, QtWidgets, QWebEngineView, QWebEngineSettings,
     Signal) = _import_pyside6()
except ImportError:
    try:
        (QtCore, QtGui, QtWidgets, QWebEngineView, QWebEngineSettings,
         Signal) = _import_pyqt5()
    except ImportError as error:            # pragma: no cover - 环境问题
        raise ImportError(
            "既没有 PySide6 也没有 PyQt5 + QtWebEngine。\n"
            "  交付环境：pip install PySide6-Essentials PySide6-Addons\n"
            "  Linux 本地验证：sudo apt install python3-pyqt5.qtwebengine"
        ) from error

#: 简写（页面代码里 ``Qt.AlignCenter`` 这类用法两边都成立）。
Qt = QtCore.Qt


def apply_chinese_font(app) -> str:
    """给 Qt 设置一个带中文的字体，返回实际选中的族名。

    Windows 上用微软雅黑，Linux 上用 Noto Sans CJK；都找不到就保持默认（不静默
    改成无中文字体，否则界面会出现方块——matplotlib 那边已经踩过一次）。
    """
    import platform
    candidates = (["Microsoft YaHei UI", "Microsoft YaHei"] if platform.system() == "Windows"
                  else ["Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Micro Hei"])
    available = set(QtGui.QFontDatabase().families())
    for family in candidates:
        if family in available:
            font = app.font()
            font.setFamily(family)
            app.setFont(font)
            return family
    return app.font().family()


def configure_webengine_environment() -> None:
    """WebEngine 启动前的环境设置（必须在建 QApplication 之前调用）。

    **不要加 ``--disable-gpu``**：Plotly 的 3D 图（``Scatter3d``）依赖 WebGL，
    实测带 ``--disable-gpu`` 时 WebGL 直接不可用，图区只显示 Chromium 那句
    "WebGL is not supported by your browser"。默认（不传开关）在有核显的机器上
    走硬件加速，正常；无 GPU 的机器由 Chromium 自己退到软件渲染。

    只处理 sandbox：容器/提权环境下 setuid sandbox 会直接崩，给一个开关。
    """
    if os.environ.get("LASER_WEBENGINE_NO_SANDBOX", "1") == "1":
        os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
