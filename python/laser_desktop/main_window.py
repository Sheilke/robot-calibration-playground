# -*- coding: utf-8 -*-
"""主窗口：左侧导航 + 右侧四页（生成 / 奇异修复 / ISO 五点 / 工具）。

S3 起「生成」页能从头跑到尾，S4 补上「奇异修复」页，S5 补上最后两页：

* **ISO 五点**（:mod:`laser_desktop.iso_page`）：立方体来源（两点输入 / 自动
  搜索）→ 五点 → 后台解算（``core.run_iso9283``）→ 统计 / 3D / 路点表 / 导出；
* **工具**（:mod:`laser_desktop.tools_page`）：GP 导出 / JSON → URDF / 运行
  目录清理 / 模板文件摘要（四块各自独立，全部走 ``laser_web.core``）。

四页里三个有图（生成 / 修复 / ISO）**复用同一个 PlotView**（S6：包在懒创建的
``PlotHost`` 里，第一次真要出图才启动 Chromium）。切页时由主窗口做四件事
（:meth:`MainWindow._on_nav_changed`）：把图控件**搬**进当前页的 ``PlotSlot``、
把该页该展示的图推过去（同一张图不重复推）、把该页表格里选中的行**补回图上**
（``Plotly.react`` 会把高亮重置掉），再把状态栏临时区换成该页最后一句（带页面
前缀）。

状态栏分两层（S6 评审 R2）：**常驻信息**（Qt 绑定 / 数据目录 / WebGL）用
``addPermanentWidget`` 挂一个 ``QLabel``，临时消息（“已选中 GP0005”）走
``showMessage``。不这么分的话，临时消息 10 s 超时后会把常驻信息一起抹掉、
并且不再回来（实测 150 ms 超时后状态栏空）。

表单与预览的实现在各页模块里；本模块只负责窗口骨架、导航、状态栏，以及给冒烟
脚本用的 :func:`build_preview_figure`。
"""
from __future__ import annotations

from pathlib import Path

from . import page_layout, paths
from .generate_page import GeneratePage, build_preview_figure
from .iso_page import IsoPage
from .plot_view import PlotHost
from .qt_compat import BINDING, DEV_ONLY_BINDING, Qt, QtCore, QtWidgets
from .repair_page import RepairPage
from .tools_page import ToolsPage

#: 导航顺序（与网页版一致：生成 / 奇异修复 / ISO 五点 / 工具）。
PAGES = ("生成", "奇异修复", "ISO 五点", "工具")

__all__ = ["MainWindow", "PAGES", "build_preview_figure"]


def _short_path(path) -> str:
    """把家目录缩成 ``~``（状态栏常驻区只有两三百 px，绝对路径会把消息挤没）。"""
    text = str(path)
    home = str(Path.home())
    if home and text.startswith(home) and text != home:
        return "~" + text[len(home):]
    return text


class _NavHolder(QtWidgets.QWidget):
    """导航列的容器：由它回答「我想多宽」，而不是让 ``QListWidget`` 回答。

    问题出在 ``QListWidget.sizeHint``：它对任何列表都报 256 px，而导航只需要
    92 px。``QSplitter`` 按子控件的 sizeHint 分初始槽位，于是导航右边白白空出
    164 px，表单和图各少拿 164 px。

    所以导航不直接进分割器，而是套一层：holder 的 ``sizeHint`` 报我们真正想要
    的宽度（最长项宽度，或用户上次拖到的宽度）。导航自己的 ``minimumWidth``
    守住「不许窄到切字」，而 ``maximumWidth`` **不设**——不设才能继续拖宽。
    """

    def __init__(self, nav: QtWidgets.QListWidget, width: int) -> None:
        super().__init__()
        self._width = max(0, int(width))
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(nav)

    def set_preferred_width(self, width: int) -> None:
        """记住新的「首选宽度」（用户拖完之后调用，下次打开就用它）。"""
        self._width = max(0, int(width))
        self.updateGeometry()

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt 接口
        hint = super().sizeHint()
        return QtCore.QSize(self._width, hint.height())


class MainWindow(QtWidgets.QMainWindow):
    """主窗口：左侧 ``QListWidget`` 导航 + 右侧 ``QStackedWidget``。

    窗口尺寸策略（S6b）：默认 :data:`page_layout.DEFAULT_WINDOW_SIZE`
    （1600×950），允许缩到 :data:`page_layout.MINIMUM_WINDOW_SIZE`（1280×800）
    并把上次关窗时的尺寸/位置记在 ``QSettings`` 里。S6 第一版把默认钉成
    1180×780 来凑"一屏 5 行"，代价是表单横向裁剪 + 图区只剩 120 px。
    """

    def __init__(self, settings=None) -> None:
        """建窗口。

        Args:
            settings: ``QSettings``（测试可注入临时文件）；``None`` 时用
                应用的默认设置（布局比例与窗口几何共用它）。
        """
        super().__init__()
        self.setWindowTitle("标定路点工具")
        self._settings = (page_layout.default_settings()
                          if settings is None else settings)
        self.setMinimumSize(*page_layout.MINIMUM_WINDOW_SIZE)
        self.resize(*page_layout.DEFAULT_WINDOW_SIZE)
        page_layout.restore_window_geometry(self, self._settings)

        self.nav = QtWidgets.QListWidget()
        self.nav.addItems(PAGES)
        # 宽度：**下限**是不切字的宽度，上限不限 —— 用户想拖宽就拖宽（见 _on_nav_moved）。
        # 不能 setFixedWidth：那会把侧边栏钉死，往右拖没反应，往左拖到极限时
        # 整块被压成 0（用户看到的就是"缩小时直接消失"）。
        fitted = self._nav_width()
        self.nav.setMinimumWidth(fitted)
        # 宽度下限就是按最长一项算的，横滚条只会在字体量不准时冒出来白占高度
        self.nav.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.stack = QtWidgets.QStackedWidget()

        # 导航不能直接 addWidget 进分割器：``QListWidget.sizeHint()`` 是 256 px，
        # 而 ``QSplitter`` 按 sizeHint 分初始槽位 —— 直接把导航放进去，实测槽位
        # 256 / 内容只有 92，导航右边空出 164 px，表单和图各少拿 164 px。
        # 套一层 holder，由它回答"想要多宽"（最长项或用户上次拖到的宽度）。
        self._nav_key = "nav_width"
        want = fitted
        stored = self._settings.value(self._nav_key)
        if stored is not None:
            try:
                width = int(stored)
            except (TypeError, ValueError):
                width = 0
            # 换过字体/DPI 后旧宽度可能比新下限还小 —— 那就用下限，别切字
            if width >= fitted:
                want = width
        nav_holder = _NavHolder(self.nav, want)

        splitter = QtWidgets.QSplitter()
        splitter.addWidget(nav_holder)
        splitter.addWidget(self.stack)
        # 多余宽度全给右侧页面，导航保持自己那份（不被 stretch 拉宽）
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        # 不允许被拖到 0：侧边栏一消失，用户找不到把页面切回去的地方
        splitter.setChildrenCollapsible(False)
        splitter.splitterMoved.connect(self._on_nav_moved)
        self.setCentralWidget(splitter)
        self._nav_splitter = splitter
        self._nav_holder = nav_holder

        self.plot = PlotHost(on_webgl=self._on_webgl)
        #: 上一次真推给图控件的那个 Figure（按**对象身份**判重）：切页时同一张图
        #: 不重复推，``Plotly.react`` 也就不会把相机/高亮重置一次。
        self._last_pushed = None
        self.generate = GeneratePage(self.plot, settings=self._settings)
        self.repair = RepairPage(self.plot, settings=self._settings)
        self.iso = IsoPage(self.plot, settings=self._settings)
        # 工具页不发任务，但要能在清理运行目录时跳过**本进程正在跑**的那些
        # （桌面版每个页面一个 JobRegistry，``core.clean_runs`` 看不见它们）。
        self.tools = ToolsPage(active_dirs=self.active_run_dirs)
        self.stack.addWidget(self.generate)
        self.stack.addWidget(self.repair)
        self.stack.addWidget(self.iso)
        self.stack.addWidget(self.tools)

        # 常驻信息放永久控件：临时消息（showMessage，10 s 超时）只会清掉临时区，
        # 不会把这三项现场排查要用的信息一起抹掉（S6 评审 R2a）。
        self._status_label = QtWidgets.QLabel(self)
        self._status_label.setObjectName("status_permanent")
        self._status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._status_label.setContentsMargins(0, 0, 8, 0)
        self.statusBar().addPermanentWidget(self._status_label)
        self.statusBar().setSizeGripEnabled(True)

        self.nav.currentRowChanged.connect(self._on_nav_changed)
        self.nav.setCurrentRow(0)

    def _on_nav_moved(self, _pos: int, _index: int) -> None:
        """用户拖完导航列：把宽度存下来（下次打开还是这么宽）。

        存像素而不是比例：导航是"够用就行"的窄条，跟着窗口一起缩放只会把表单
        挤窄。下限由 :meth:`_nav_width` 守着，所以存下来的值不会小到切字。

        **不**把最大宽度钉死 —— 钉了就等于又变成 ``setFixedWidth``，第二次拖动
        就没反应了。只告诉 holder "以后初始宽度是这个"，用户随时还能继续拖。
        """
        width = int(self.nav.width())
        if width <= 0:
            return
        self._nav_holder.set_preferred_width(width)
        self._settings.setValue(self._nav_key, width)

        self._refresh_status()

    def active_run_dirs(self) -> set[str]:
        """本进程里还没结束的任务目录（三个发任务的页面的登记表之并）。"""
        directories: set[str] = set()
        for page in (self.generate, self.repair, self.iso):
            registry = getattr(getattr(page, "bridge", None), "registry", None)
            if registry is not None:
                directories |= registry.active_dirs()
        return directories

    def _nav_width(self) -> int:
        """导航列宽度（px）：按最长的一项算，下限是 :data:`page_layout.NAV_WIDTH`。

        为什么不再直接 ``setFixedWidth(NAV_WIDTH)``：92 px 是在 96 dpi 下量的
        （那时最长一项「奇异修复」只有 74 px，92 还宽出 18）。S7 现场机器的
        ``Xft.dpi=192``：同样四项要 122 px，钉死的 92 就把「奇异修复」切成
        「奇异修」、「ISO 五点」切成「ISO 五」（截图 ``/tmp/ld-shot.png``）。

        宽度改成「最长一项」与「老下限」取大：96 dpi 下仍然等于 92（视觉与旧版
        完全一致，省下的 38 px 还给表单和图），字体一变就自己变宽、不再切字。
        ``sizeHintForColumn`` 含列表项的左、右内边距，但**不含**列表自己的
        边框：不补这 2 px，现场实测会冒一根最大 2 px 的横滚条（白吃 20 px 高，
        截图里导航底下多一条带箭头的槽）。

        Returns:
            int: 导航列宽度（px）。
        """
        padding = 2 * int(self.nav.frameWidth())
        return max(int(page_layout.NAV_WIDTH),
                   int(self.nav.sizeHintForColumn(0)) + padding)

    def _on_webgl(self, ok: bool, renderer: str) -> None:
        """WebGL 自检结果：可用就写进状态栏，不可用就把英文报错换成中文建议。"""
        self._refresh_status()
        if not ok:
            self.generate.show_webgl_warning()
            self.repair.show_webgl_warning()
            self.iso.show_webgl_warning()

    def _on_nav_changed(self, row: int) -> None:
        """切页：换栈内页，把共享的图搬进来，并把该页的图/选中/状态栏都对齐。

        三张图共用一个 ``PlotView``（计划 §6 的验收项：切页 < 200 ms、子进程
        ≤ 2），所以：

        1. 物理上把图控件挪到当前页（否则它会一直停在最后一次 ``addWidget``
           的那一页）；
        2. 把该页当前该展示的图推过去——**同一张图不重复推**（``Plotly.react``
           会把相机和高亮重置，白花 20~40 ms）；
        3. 该页还没出过图时推一张**空图**，让壳页重新显示「还没有数据」，
           而不是把上一页的图留在那里；
        4. 把该页表格当前行补回图上（react 会重置 marker 尺寸），并把状态栏
           临时区换成该页最后一句（带页面前缀）——否则上一页的
           “已选中 …（图上已放大）”会跨页残留，图上却早就不亮了。
        """
        self.stack.setCurrentIndex(row)
        page = self.stack.currentWidget()
        # 隐藏页不许再往共享视图推图（它们的预览去抖照样会到点），
        # 也不许再改状态栏（三页 connect 同一个 pointClicked）。
        for other in (self.generate, self.repair, self.iso):
            set_active = getattr(other, "set_active", None)
            if callable(set_active):
                set_active(other is page)
        slot = getattr(page, "plot_slot", None)
        if slot is not None:
            slot.adopt(self.plot)
        self._show_page_figure(page)
        self._restore_page_selection(page)
        self._refresh_status()
        self._restore_page_status(page)

    def _show_page_figure(self, page) -> None:
        """把该页该展示的图推给共享视图（同一张图不重复推）。"""
        current = getattr(page, "current_figure", None)
        if not callable(current):
            return
        figure = current()
        if figure is None:
            if not self.plot.created:
                # 懒创建：一张图都还没推过，就没有残留要清（启动时不许拉
                # Chromium，P5）。
                return
            figure = page_layout.empty_figure()
        if figure is self._last_pushed and self.plot.loaded:
            return                          # 同一张图：react 一遍只会白花钱
        self.plot.set_figure(figure)
        self._last_pushed = figure

    def _restore_page_selection(self, page) -> None:
        """把本页表格当前行重新点亮到图上（R2c：react 已经把高亮清了）。"""
        restore = getattr(page, "restore_plot_selection", None)
        if callable(restore):
            restore()

    def _restore_page_status(self, page) -> None:
        """状态栏临时区换成该页最后一句（带页面前缀）；没说过话就清空。

        同样给 10 s 超时：这是**临时**区，消息到点自己消失，常驻信息在永久
        控件里不受影响。
        """
        message = getattr(page, "status_message", None)
        text = message() if callable(message) else ""
        if text:
            self.statusBar().showMessage(str(text), 10_000)
        else:
            self.statusBar().clearMessage()

    def _refresh_status(self) -> None:
        """刷新常驻信息（永久控件；临时消息超时不会把它抹掉）。

        控件里放**简版**（绑定 · 数据目录 · WebGL），完整文字（含绝对路径、
        两条告警）放 ``tooltip``：状态栏同一条上还有临时消息，常驻区占了
        1400 px 的话页面消息就被挤成半句（实测）。
        """
        label = getattr(self, "_status_label", None)
        if label is None:
            return
        label.setText(self._status_brief())
        label.setToolTip(self._status_text())

    def _status_brief(self) -> str:
        """常驻信息的短版（永久控件里那一行）。"""
        text = (f"{BINDING}　|　数据：{_short_path(paths.data_root())}"
                f"　|　WebGL：{self._webgl_text(brief=True)}")
        if DEV_ONLY_BINDING:
            text += "　|　⚠ 仅本地验证"
        if getattr(self.plot, "link_warning", ""):
            text += "　|　⚠ 图→表联动不可用"
        return text

    def _webgl_text(self, brief: bool = False) -> str:
        """WebGL 一栏的文字（还没查完时写“检查中”）。

        Args:
            brief: 是否截短渲染器名（常驻控件里那一行；全名在 tooltip 里）。
        """
        renderer = self.plot.webgl_renderer or "检查中"
        if self.plot.webgl_ok is False:
            renderer = "不可用"
        if brief and len(renderer) > 22:
            renderer = renderer[:21] + "…"
        return renderer

    def _status_text(self) -> str:
        """常驻信息全文（tooltip / 日志 / 测试用）：绑定·数据目录·WebGL·告警。"""
        text = (f"Qt 绑定：{BINDING}　|　数据目录：{paths.data_root()}"
                f"　|　WebGL：{self._webgl_text()}")
        warning = getattr(self.plot, "link_warning", "")
        if warning:
            text += f"　|　⚠ {warning}"
        if DEV_ONLY_BINDING:
            text += "　|　⚠ PyQt5 仅供本地验证，交付用 PySide6"
        return text

    def closeEvent(self, event) -> None:            # noqa: N802 - Qt 回调命名
        """关闭前：记窗口尺寸 / 停三页订阅 / 拆掉共享的 WebEngine 视图。

        :meth:`PlotHost.shutdown` 只调一次（三页共用一个视图）。它先把
        ``QWebEnginePage`` 删掉再删视图——反过来的话，Qt 释放 profile 时会报
        "Release of profile requested but WebEnginePage still not deleted"，
        交付到 Windows 上就是关窗报错 / 退出崩。
        """
        page_layout.save_window_geometry(self, self._settings)
        self.generate.shutdown()
        self.repair.shutdown()
        self.iso.shutdown()
        self.plot.shutdown()
        super().closeEvent(event)
