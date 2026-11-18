# -*- coding: utf-8 -*-
"""主窗口：左侧导航 + 右侧四页（S3 起「生成」页真跑解算）。

S3 的定位：生成页能**从头跑到尾**——填域/网格/姿态，预览卡片与路点图跟着变，
点「生成并解算」就在后台线程跑 IK，进度/日志/取消/结果表/导出全部就位（
线程模型见 :mod:`laser_desktop.job_bridge`）。其余三页仍是占位说明，等 S4/S5
逐个填。

表单与预览的实现在 :mod:`laser_desktop.generate_page`；本模块只负责窗口骨架、
导航、状态栏，以及给冒烟脚本用的 :func:`build_preview_figure`。
"""
from __future__ import annotations

from . import paths
from .generate_page import GeneratePage, build_preview_figure
from .plot_view import PlotView
from .qt_compat import BINDING, DEV_ONLY_BINDING, QtWidgets

#: 导航顺序（与网页版一致：生成 / 奇异修复 / ISO 五点 / 工具）。
PAGES = ("生成", "奇异修复", "ISO 五点", "工具")

__all__ = ["MainWindow", "PAGES", "build_preview_figure"]


def _placeholder(title: str, note: str) -> QtWidgets.QWidget:
    """占位页：一句话说明这个页面以后放什么。"""
    widget = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(widget)
    heading = QtWidgets.QLabel(title)
    font = heading.font()
    font.setPointSize(font.pointSize() + 4)
    heading.setFont(font)
    layout.addWidget(heading)
    layout.addWidget(QtWidgets.QLabel(note))
    layout.addStretch(1)
    return widget


class MainWindow(QtWidgets.QMainWindow):
    """主窗口：左侧 ``QListWidget`` 导航 + 右侧 ``QStackedWidget``。"""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("标定路点工具")
        self.resize(1180, 780)

        self.nav = QtWidgets.QListWidget()
        self.nav.setFixedWidth(150)
        self.nav.addItems(PAGES)
        self.stack = QtWidgets.QStackedWidget()

        splitter = QtWidgets.QSplitter()
        splitter.addWidget(self.nav)
        splitter.addWidget(self.stack)
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        self.plot = PlotView()
        self.generate = GeneratePage(self.plot)
        self.stack.addWidget(self.generate)
        self.stack.addWidget(_placeholder(
            "奇异修复", "现场报奇异时，填「已走到第几点」和停机时的 6 个关节角，\n"
                        "程序定位到具体段并给出修复方案（S4）。"))
        self.stack.addWidget(_placeholder(
            "ISO 五点", "按 ISO 9283 生成 P1~P5：给最大/最小点，或自动搜索内接立方体（S5）。"))
        self.stack.addWidget(_placeholder(
            "工具", "GP 导出、控制器 JSON → URDF、运行目录清理（S5）。"))

        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.nav.setCurrentRow(0)

        self.statusBar().showMessage(self._status_text())
        self.plot.load_shell()
        self.plot.check_webgl(self._on_webgl)

    def _on_webgl(self, ok: bool, renderer: str) -> None:
        """WebGL 自检结果：可用就写进状态栏，不可用就把英文报错换成中文建议。"""
        self.statusBar().showMessage(self._status_text())
        if not ok:
            self.generate.show_webgl_warning()

    def _status_text(self) -> str:
        """状态栏：Qt 绑定 / 数据目录 / WebGL 渲染器（三者都是现场排查要看的）。"""
        renderer = self.plot.webgl_renderer or "检查中"
        if self.plot.webgl_ok is False:
            renderer = "不可用"
        text = (f"Qt 绑定：{BINDING}　|　数据目录：{paths.data_root()}"
                f"　|　WebGL：{renderer}")
        if DEV_ONLY_BINDING:
            text += "　|　⚠ PyQt5 仅供本地验证，交付用 PySide6"
        return text

    def closeEvent(self, event) -> None:            # noqa: N802 - Qt 回调命名
        """关闭前断开任务订阅 / 停定时器（任务本身在工作线程里，不强杀）。"""
        self.generate.shutdown()
        super().closeEvent(event)
