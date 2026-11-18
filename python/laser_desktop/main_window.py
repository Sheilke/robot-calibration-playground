# -*- coding: utf-8 -*-
"""主窗口：左侧导航 + 右侧四页（S1 只把"生成"页做出真实内容）。

S1 的定位是**可跑的骨架**：窗口、导航、以及一张真实的 Plotly 图（50 个路点，
只做规划不跑 IK）。其余三页放占位说明，等 S2~S5 逐个填。
"""
from __future__ import annotations

from pathlib import Path

from . import paths
from .plot_view import PlotView
from .qt_compat import BINDING, DEV_ONLY_BINDING, Qt, QtWidgets

#: 导航顺序（与网页版一致：生成 / 奇异修复 / ISO 五点 / 工具）。
PAGES = ("生成", "奇异修复", "ISO 五点", "工具")

#: 与网页版「生成」页默认一致的示例参数（只规划路点，不跑 IK）。
SAMPLE_DOMAIN = {"x_min": 750.0, "x_max": 1300.0,
                 "y_min": -600.0, "y_max": 600.0,
                 "z_min": 270.0, "z_max": 940.0}
SAMPLE_RPY = (3.0, 73.0, -167.0)
SAMPLE_GRID = {"xs": 1, "ys": 4, "zs": 4, "z_insert": 0}


def build_preview_figure():
    """按示例参数生成一张路点图（不跑 IK，秒级返回）。

    复用算法模块的 ``plan_waypoints``，保证与网页版/命令行**同一套网格**；
    连不上核心模块时返回一个说明性空图，而不是让窗口起不来。
    """
    import plotly.graph_objects as go

    try:
        from excitation_trajectory.aabb_snake_scan import plan_waypoints
    except ImportError:
        figure = go.Figure()
        figure.add_annotation(text="找不到算法模块（检查 PYTHONPATH=python）",
                              showarrow=False, font={"size": 14})
        figure.update_layout(title="路点预览（不可用）")
        return figure

    bounds = SAMPLE_DOMAIN
    waypoints = plan_waypoints(
        bounds["x_min"], bounds["x_max"], bounds["y_min"], bounds["y_max"],
        bounds["z_min"], bounds["z_max"], SAMPLE_GRID["xs"], SAMPLE_GRID["ys"],
        SAMPLE_GRID["zs"], SAMPLE_RPY, z_insert=SAMPLE_GRID["z_insert"])
    positions = [waypoint.pos for waypoint in waypoints]
    figure = go.Figure(go.Scatter3d(
        x=[pos[0] for pos in positions],
        y=[pos[1] for pos in positions],
        z=[pos[2] for pos in positions],
        mode="markers+lines",
        marker={"size": 5, "color": list(range(len(positions))),
                "colorscale": "Viridis", "colorbar": {"title": "序号"}},
        line={"color": "rgba(120,120,120,0.35)", "width": 2},
        customdata=[[index] for index in range(len(positions))],
        hovertemplate="GP%{customdata[0]}<br>"
                      "X=%{x:.0f} Y=%{y:.0f} Z=%{z:.0f}<extra></extra>"))
    figure.update_layout(
        title=f"路点预览：{len(positions)} 个点（Y→X→Z 蛇形，未跑 IK）",
        scene={"xaxis_title": "X (mm)", "yaxis_title": "Y (mm)",
               "zaxis_title": "Z (mm)", "aspectmode": "data"},
        margin={"l": 0, "r": 0, "t": 40, "b": 0})
    return figure


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
        self.stack.addWidget(self._generate_page())
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
        if ok:
            self.statusBar().showMessage(
                f"{self._status_text()}　|　WebGL：{renderer or '可用'}")
            return
        self.banner.setText(
            "⚠ 这台机器的 WebGL 不可用，Plotly 3D 画不出来。\n"
            "　处理办法：① 更新显卡驱动；② 远程桌面下改在本机打开；"
            "③ 先用网页版（python -m laser_web）看图，功能与数值一致。")
        self.banner.setStyleSheet(
            "background:#fff4e5; color:#8a4b00; padding:8px; border:1px solid #f0c48a;")
        self.banner.show()

    def _generate_page(self) -> QtWidgets.QWidget:
        """生成页（S1 版）：只显示示例路点图 + 一句说明。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        self.banner = QtWidgets.QLabel("")
        self.banner.setWordWrap(True)
        self.banner.hide()
        layout.addWidget(self.banner)
        hint = QtWidgets.QLabel(
            "S1 骨架：这张图用的是示例参数（域 750..1300 / -600..600 / 270..940，"
            f"网格 2×5×5 = 50 点），只做规划、未跑 IK。\n"
            f"表单、后台任务与导出在 S2/S3 接入。当前 Qt 绑定：{BINDING}")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addWidget(self.plot, 1)
        return page

    def _status_text(self) -> str:
        text = f"Qt 绑定：{BINDING}　|　数据目录：{paths.data_root()}"
        if DEV_ONLY_BINDING:
            text += "　|　⚠ PyQt5 仅供本地验证，交付用 PySide6"
        return text
