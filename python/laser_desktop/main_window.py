# -*- coding: utf-8 -*-
"""主窗口：左侧导航 + 右侧四页（生成 / 奇异修复 / ISO 五点 / 工具）。

S3 起「生成」页能从头跑到尾，S4 补上「奇异修复」页，S5 补上最后两页：

* **ISO 五点**（:mod:`laser_desktop.iso_page`）：立方体来源（两点输入 / 自动
  搜索）→ 五点 → 后台解算（``core.run_iso9283``）→ 统计 / 3D / 路点表 / 导出；
* **工具**（:mod:`laser_desktop.tools_page`）：GP 导出 / JSON → URDF / 运行
  目录清理 / 模板文件摘要（四块各自独立，全部走 ``laser_web.core``）。

四页里三个有图（生成 / 修复 / ISO）**复用同一个 PlotlyView**：切页时由主窗口
把该页当前该展示的图重推一次（:meth:`MainWindow._on_nav_changed`），而不是每页
各建一个 WebEngine。

表单与预览的实现在各页模块里；本模块只负责窗口骨架、导航、状态栏，以及给冒烟
脚本用的 :func:`build_preview_figure`。
"""
from __future__ import annotations

from . import paths
from .generate_page import GeneratePage, build_preview_figure
from .iso_page import IsoPage
from .plot_view import PlotView
from .qt_compat import BINDING, DEV_ONLY_BINDING, QtWidgets
from .repair_page import RepairPage
from .tools_page import ToolsPage

#: 导航顺序（与网页版一致：生成 / 奇异修复 / ISO 五点 / 工具）。
PAGES = ("生成", "奇异修复", "ISO 五点", "工具")

__all__ = ["MainWindow", "PAGES", "build_preview_figure"]


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
        self.repair = RepairPage(self.plot)
        self.iso = IsoPage(self.plot)
        # 工具页不发任务，但要能在清理运行目录时跳过**本进程正在跑**的那些
        # （桌面版每个页面一个 JobRegistry，``core.clean_runs`` 看不见它们）。
        self.tools = ToolsPage(active_dirs=self.active_run_dirs)
        self.stack.addWidget(self.generate)
        self.stack.addWidget(self.repair)
        self.stack.addWidget(self.iso)
        self.stack.addWidget(self.tools)

        self.nav.currentRowChanged.connect(self._on_nav_changed)
        self.nav.setCurrentRow(0)

        self.statusBar().showMessage(self._status_text())
        self.plot.load_shell()
        self.plot.check_webgl(self._on_webgl)

    def active_run_dirs(self) -> set[str]:
        """本进程里还没结束的任务目录（三个发任务的页面的登记表之并）。"""
        directories: set[str] = set()
        for page in (self.generate, self.repair, self.iso):
            registry = getattr(getattr(page, "bridge", None), "registry", None)
            if registry is not None:
                directories |= registry.active_dirs()
        return directories

    def _on_webgl(self, ok: bool, renderer: str) -> None:
        """WebGL 自检结果：可用就写进状态栏，不可用就把英文报错换成中文建议。"""
        self.statusBar().showMessage(self._status_text())
        if not ok:
            self.generate.show_webgl_warning()
            self.repair.show_webgl_warning()
            self.iso.show_webgl_warning()

    def _on_nav_changed(self, row: int) -> None:
        """切页：换栈内页，并把该页当前该展示的图重推给共用的 PlotView。

        三张图共用一个 ``QWebEngineView``（计划 §6 的验收项：切页 < 200 ms），
        所以切回来时必须主动重推一次，否则会看到另一页留下的图。
        """
        self.stack.setCurrentIndex(row)
        page = self.stack.currentWidget()
        current = getattr(page, "current_figure", None)
        if not callable(current):
            return
        figure = current()
        if figure is not None:
            self.plot.set_figure(figure)

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
        self.repair.shutdown()
        self.iso.shutdown()
        super().closeEvent(event)
