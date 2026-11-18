# -*- coding: utf-8 -*-
"""生成页（S2）：完整表单 + 实时预览（只规划路点，**不跑 IK**）。

与网页版 ``laser_web/pages/generate.py`` 的关系
----------------------------------------------
* 字段、默认值、总点数口径**与网页版一致**：默认值来自
  :func:`laser_web.ui_state.default_form_values`（格点数是"份数 + 1"）和 CLI 的
  ``build_arg_parser()``，本模块不抄任何数字；
* 校验只有一套：任何输入变化都把表单收成 :func:`form_to_params` 的产物，
  交给 :func:`laser_web.core.preview_domain`（内部走 CLI 的 ``_validate_args``）；
  出错就把**报错原文**显示在预览卡片里，不弹窗、不在界面里重写规则；
* 总点数按网页版 ``_total_points_text`` 的口径算：手填模式**直接按公式**
  ``X×Y×Z 格点 + (Z 格点数−1) × 换层插点``（不为了显示一行数字去调 core），
  凑数模式读 ``preview_domain`` 的 ``point_count``；
* ``core.preview_domain`` 外面还有一层兜底：算法层万一抛异常（历史上的
  ``choose_grid`` 零长度轴 ``ZeroDivisionError``），一律转成预览卡片的报错，
  不让异常打断 Qt 槽、把界面停在旧状态；
* S2 只做预览：不跑 IK、不起后台任务、不导出（S3 才接那三件事）。

图只在参数合法且点数不超上限时才规划（``plan_waypoints`` 是毫秒级，但 800 万点
会直接把内存吃穿），并用 300 ms 去抖，避免拖动 spinbox 时反复重算。
"""

from __future__ import annotations

import logging
import math
from typing import NamedTuple

from excitation_trajectory import aabb_snake_scan as scan
from laser_web import core, ui_state

from . import paths
from .qt_compat import Qt, QtCore, QtWidgets

try:                                    # 缺算法模块时给说明图，而不是让窗口起不来
    from excitation_trajectory.aabb_snake_scan import plan_waypoints
except ImportError:                     # pragma: no cover - 环境问题
    plan_waypoints = None

#: 路点数的软上限：与 ``laser_web/pages/__init__.py`` 的 ``POINT_COUNT_WARN``
#: 保持一致（S 层不改服务层，所以这里是一个显式常量 + 单测钉住两边相等）。
#: 超过它就在预览卡片标红、禁用「生成并解算」——800 万点会在 ``plan_waypoints``
#: 里直接吃穿内存，而未来的取消只在阶段边界生效。
POINT_COUNT_WARN = 5000

#: 「目标点数」输入框的上限：``choose_grid`` 真能凑出来的最大点数。
#: 份数上限 ``COUNT_GRID_MAX_PER_AXIS``（每轴 <= 12，即 13 个格点）与单次插点
#: 上限 ``COUNT_GRID_MAX_INSERT`` 决定：``13³ + 12×10 = 2317``。再往上就是
#: 「保证报错」的区间，不该让用户填得出来——所以常量从算法模块算，不抄数字。
TARGET_COUNT_MAX = ((scan.COUNT_GRID_MAX_PER_AXIS + 1) ** 3
                    + scan.COUNT_GRID_MAX_PER_AXIS * scan.COUNT_GRID_MAX_INSERT)

#: 日志器（``preview_domain`` 抛异常、路点规划失败都记在这里）。
_LOGGER = logging.getLogger(__name__)

#: CLI 默认值的唯一来源（不手抄数字）。
_CLI_DEFAULTS = scan.build_arg_parser().parse_args([])

#: 段模型下拉框：值 -> 页面文案（与网页版一致：现场 JOB 走 MOVL）。
SEG_MODES = {"cartesian": "MOVL（笛卡尔直线）", "joint": "PTP（关节直线）"}

#: 段模型默认值：**故意与 CLI 不同**。CLI 默认 ``--seg-mode joint``（PTP，历史
#: 行为），而现场 JOB 走 MOVL；这里跟随网页版的默认，只影响段内检查。
DEFAULT_SEG_MODE = "cartesian"

#: 两点输入 / 六个轴边界；目标点数 / 手填格点数（与 ``ui_state`` 的取值一致）。
DOMAIN_MODES = ui_state.DOMAIN_MODES
GRID_MODES = ui_state.GRID_MODES

#: 单位后缀一律 ASCII（中文只出现在标签上）。
UNIT_MM = " mm"
UNIT_DEG = " deg"

#: 预览卡片里标红用的样式（超上限 / 报错）。
_RED = "color:#c0392b"
_RED_BOLD = "color:#c0392b; font-weight:bold"

#: 预览图去抖间隔（毫秒）：拖动 spinbox 时不必每个刻度都重规划一次。
DEBOUNCE_MS = 300


class TotalPoints(NamedTuple):
    """当前表单会生成多少点（与网页版 ``_total_points_text`` 同口径）。"""

    count: int | None
    """总点数；参数非法（凑数凑不出来）时为 ``None``。"""
    text: str
    """一行显示文本，如 ``"64 点（4×4×4 格点）"``。"""
    over: bool
    """是否超过 :data:`POINT_COUNT_WARN`。"""


def preview_failure(error: BaseException) -> dict:
    """把 ``core.preview_domain`` 抛出的异常收成它自己的失败格式。

    ``core.preview_domain`` 本该自己吞掉校验错误，但算法层仍可能抛出未预料的
    异常（例如 ``choose_grid`` 在零长度轴上的 ``ZeroDivisionError``）。界面不能
    让异常穿过 Qt 槽：那会让槽提前 return，卡片停在上一组数值上。

    Args:
        error: 捕获到的异常。

    Returns:
        dict: 与 ``preview_domain`` 失败时同构的 ``{"ok": False, "error",
        "warnings"}``，可直接交给 :meth:`GeneratePage._update_card`。
    """
    return {"ok": False,
            "error": f"预览失败：{type(error).__name__}: {error}",
            "warnings": []}


def format_condition(value) -> str:
    """设计条件数显示：奇异参考姿态的 ``inf`` 要说清是姿态问题，不是程序 bug。

    内旋 XYZ 下 pitch = ±90° 时 Jacobian 退化，``design_condition_number`` 返回
    ``inf``；原样打印 ``inf`` 会被当成故障。

    Args:
        value: ``preview_domain`` 的 ``design_condition``。

    Returns:
        str: ``"12.345"``、``"∞（该参考姿态奇异）"``，``nan`` 退成 ``"—"``。
    """
    number = float(value)
    if math.isinf(number):
        return "∞（该参考姿态奇异）"
    if math.isnan(number):
        return "—"
    return f"{number:.3f}"


# ---------------------------------------------------------------------------
# 纯逻辑（不碰 Qt，单测直接调）
# ---------------------------------------------------------------------------
def form_defaults() -> dict:
    """表单默认值：网页版的 :func:`ui_state.default_form_values` + 守卫项。

    守卫项（``j5_min`` / ``seg_mode`` / ``q_seed`` …）不在 ``ui_state`` 里，
    网页版是在 ``render()`` 里补的；这里同样**问 CLI 要**，只有 ``seg_mode``
    是界面默认（见 :data:`DEFAULT_SEG_MODE` 的说明）。

    Returns:
        dict: 全新构造的表单值，键与 :func:`form_to_params` 的入参一致。
    """
    values = ui_state.default_form_values()
    values.update({
        "j5_min": float(_CLI_DEFAULTS.j5_min),
        "j3_min": float(_CLI_DEFAULTS.j3_min),
        "j5_attempts": int(_CLI_DEFAULTS.j5_attempts),
        "seg_min": float(_CLI_DEFAULTS.seg_min),
        "seg_mode": DEFAULT_SEG_MODE,
        "seg_samples": int(_CLI_DEFAULTS.seg_samples),
        "seed": int(_CLI_DEFAULTS.seed),
        "q_seed": str(_CLI_DEFAULTS.q_seed),
    })
    return values


def form_to_params(values: dict) -> dict:
    """表单值 -> ``core.preview_domain`` 的入参（只提交**当前激活**的互斥项）。

    界面上的「格点数」在这里换算成算法要的「份数」：
    ``vertices_to_segments``（顶点数 = 份数 + 1）。互斥规则本身不在这里判断，
    仍然由 ``_validate_args`` 兜底——所以两处都填了也不会静默丢掉一个。

    Args:
        values: 表单值，缺省项按 :func:`form_defaults` 补齐（单测可以只给变动的键）。

    Returns:
        dict: 可直接喂 :func:`laser_web.core.preview_domain` 的参数。
    """
    values = {**form_defaults(), **(values or {})}
    params = {
        "robot": values["robot"],
        "rpy": [float(value) for value in values["rpy"]],
        "euler": str(values["euler"]),
        "perturb_deg": [float(value) for value in values["perturb_deg"]],
        "cond_max": float(values["cond_max"]),
        # ---- 守卫 / 段检查 / 随机种子（S2 只做搬运，S3 才算） ----
        "j5_min": float(values["j5_min"]),
        "j3_min": float(values["j3_min"]),
        "j5_attempts": int(values["j5_attempts"]),
        "seg_min": float(values["seg_min"]),
        "seg_mode": str(values["seg_mode"]),
        "seg_samples": int(values["seg_samples"]),
        "seed": int(values["seed"]),
        "q_seed": str(values["q_seed"]),
    }
    if values["domain_mode"] == DOMAIN_MODES[0]:        # 两点输入
        params["point_min"] = [float(value) for value in values["point_min"]]
        params["point_max"] = [float(value) for value in values["point_max"]]
    else:                                               # 六个轴边界
        params["aabb6"] = [float(value) for value in values["aabb6"]]
    if values["grid_mode"] == GRID_MODES[0]:             # 目标点数
        params["target_count"] = int(values["target_count"])
    else:                                                # 手填格点数
        for name in ("xs", "ys", "zs"):
            params[name] = ui_state.vertices_to_segments(values[name])
        params["z_insert"] = int(values["z_insert"])
    return params


def total_points(values: dict, preview: dict | None = None) -> TotalPoints:
    """当前表单会生成多少点：``(count, text, over)``。

    口径照抄网页版 ``generate.py::_total_points_text``：

    * **手填模式**直接按 ``X×Y×Z 格点 + (Z 格点 - 1) × 换层插点`` 算，
      不为了显示一行数字去调 core（要的是"边填边看"）；
    * **目标点数模式**只能读 ``core.preview_domain`` 的 ``point_count``
      （网格由 ``choose_grid`` 凑出来），凑不出来就显示 ``—``。

    Args:
        values: 表单值（缺省项按 :func:`form_defaults` 补齐）。
        preview: 已经算好的 ``core.preview_domain`` 结果；``None`` 时在本函数里算
            （目标点数模式下才需要）。

    Returns:
        TotalPoints: 总点数、显示文本、是否超过 :data:`POINT_COUNT_WARN`。
    """
    values = {**form_defaults(), **(values or {})}
    if values["grid_mode"] == GRID_MODES[0]:            # 目标点数：只能问 core
        if preview is None:
            try:
                result = core.preview_domain(form_to_params(values))
            except Exception as error:                  # noqa: BLE001 - 见 preview_failure
                _LOGGER.exception("core.preview_domain 抛异常（总点数显示 —）")
                result = preview_failure(error)
        else:
            result = preview
        if not result.get("ok"):
            return TotalPoints(None, "—", False)
        grid = result["grid"]
        count = int(result["point_count"])
        text = (f"{count} 点（{grid['xs'] + 1}×{grid['ys'] + 1}×"
                f"{grid['zs'] + 1} 格点）")
        return TotalPoints(count, text, count > POINT_COUNT_WARN)

    try:
        vertices = [max(1, int(values[key] or 1)) for key in ("xs", "ys", "zs")]
        insert = max(0, int(values["z_insert"] or 0))
    except (TypeError, ValueError):
        return TotalPoints(None, "—", False)
    segments = [value - 1 for value in vertices]
    count = (segments[0] + 1) * (segments[1] + 1) * (segments[2] + 1) \
        + segments[2] * insert
    text = (f"{count} 点（{'×'.join(str(value) for value in vertices)} 格点"
            + (f" + {segments[2] * insert} 插点" if insert else "") + "）")
    return TotalPoints(count, text, count > POINT_COUNT_WARN)


def waypoint_figure(bounds, grid: dict, rpy):
    """按 AABB 与网格出一张路点图（只规划，不跑 IK）。

    Args:
        bounds: ``[x_min, x_max, y_min, y_max, z_min, z_max]``（mm），已归一化。
        grid: ``{"xs": int, "ys": int, "zs": int, "z_insert": int}``（**份数**）。
        rpy: 参考姿态 ``(a, b, c)`` deg。

    Returns:
        plotly ``Figure``。
    """
    if plan_waypoints is None:                          # pragma: no cover - 环境问题
        return placeholder_figure("找不到算法模块（检查 PYTHONPATH=python）")
    waypoints = plan_waypoints(
        bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5],
        int(grid["xs"]), int(grid["ys"]), int(grid["zs"]), tuple(rpy),
        z_insert=int(grid["z_insert"]))
    positions = [waypoint.pos for waypoint in waypoints]

    import plotly.graph_objects as go

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


def placeholder_figure(reason: str, title: str = "路点预览（未出图）"):
    """出一张只有一句说明的空图（参数非法 / 点数超上限时用，不抛异常）。"""
    import plotly.graph_objects as go

    figure = go.Figure()
    figure.add_annotation(text=str(reason), showarrow=False,
                          font={"size": 13}, align="left")
    figure.update_layout(title=title, margin={"l": 0, "r": 0, "t": 40, "b": 0})
    return figure


def build_preview_figure(values: dict | None = None):
    """按表单值出图（默认 = 网页版示例域 2×5×5 格点，50 点）。

    这是 S1 ``build_preview_figure()`` 的 S2 版本：同一个函数，从"写死示例"改成
    "跟着表单走"。参数非法时返回说明图，不抛异常。

    Args:
        values: 表单值；``None`` 表示用 :func:`form_defaults`。

    Returns:
        plotly ``Figure``。
    """
    values = form_defaults() if values is None else values
    try:
        result = core.preview_domain(form_to_params(values))
    except Exception as error:                          # noqa: BLE001 - 别让按钮/冒烟崩
        _LOGGER.exception("core.preview_domain 抛异常（改出说明图）")
        result = preview_failure(error)
    if not result.get("ok"):
        return placeholder_figure(result.get("error") or "参数不可用")
    if int(result["point_count"]) > POINT_COUNT_WARN:
        return placeholder_figure(
            f"当前设置 {result['point_count']} 点，超过 {POINT_COUNT_WARN} 点上限，"
            f"先不画图。")
    return waypoint_figure(result["bounds"], result["grid"], values["rpy"])


# ---------------------------------------------------------------------------
# 控件小工具
# ---------------------------------------------------------------------------
def _double_spin(minimum: float, maximum: float, value: float,
                 suffix: str = UNIT_MM, decimals: int = 1,
                 step: float = 0.5) -> QtWidgets.QDoubleSpinBox:
    """带单位后缀的浮点输入（范围/精度按字段给）。"""
    box = QtWidgets.QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setSuffix(suffix)
    box.setValue(float(value))
    box.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    box.setMinimumWidth(88)
    return box


def _int_spin(minimum: int, maximum: int, value: int,
              suffix: str = "") -> QtWidgets.QSpinBox:
    """整数输入（格点数 / 点数 / 采样数）。"""
    box = QtWidgets.QSpinBox()
    box.setRange(minimum, maximum)
    box.setValue(int(value))
    if suffix:
        box.setSuffix(suffix)
    box.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    box.setMinimumWidth(76)
    return box


def _row(*widgets) -> QtWidgets.QWidget:
    """把几个控件横向排成一行（放进 QFormLayout 的一行里）。"""
    holder = QtWidgets.QWidget()
    layout = QtWidgets.QHBoxLayout(holder)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(6)
    for widget in widgets:
        layout.addWidget(widget)
    return holder


def _inline(items) -> QtWidgets.QWidget:
    """一行里并排几组「字段名 + 输入框」（列宽均分，高度紧凑）。

    Args:
        items: ``[(字段名, 控件), ...]``。

    Returns:
        可直接 ``addRow`` 的容器控件。
    """
    holder = QtWidgets.QWidget()
    layout = QtWidgets.QHBoxLayout(holder)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(6)
    for title, widget in items:
        column = QtWidgets.QVBoxLayout()
        column.setContentsMargins(0, 0, 0, 0)
        column.setSpacing(2)
        label = QtWidgets.QLabel(title)
        label.setStyleSheet("color:#6b7280")
        column.addWidget(label)
        column.addWidget(widget)
        layout.addLayout(column, 1)
    return holder


def _caption(text: str) -> QtWidgets.QLabel:
    """说明性小字（灰）。"""
    label = QtWidgets.QLabel(text)
    label.setWordWrap(True)
    label.setStyleSheet("color:#6b7280")
    return label


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class GeneratePage(QtWidgets.QWidget):
    """生成页：左侧可滚动表单 + 右上预览卡片 + 右下路点图。

    所有输入变化都会立刻重算预览卡片（``core.preview_domain`` 约 1 ms），
    路点图则交给 :data:`DEBOUNCE_MS` 的去抖定时器，避免拖动 spinbox 时卡顿。

    Attributes:
        fields: ``{键: 控件}``，取值与 :func:`form_to_params` 的入参同名。
        metric_labels: 预览卡片里的数值标签。
        generate_button: 「生成并解算」（S3 才真干活；参数非法或超上限时禁用）。
        total: 最近一次的 :class:`TotalPoints`（网格组与预览卡片共用同一口径）。
    """

    #: 预览图去抖间隔（毫秒）。
    DEBOUNCE_MS = DEBOUNCE_MS

    def __init__(self, plot=None, robots=None, parent=None) -> None:
        """建页面。

        Args:
            plot: ``PlotView``（或任何有 ``set_figure`` 的对象）；``None`` 表示
                不挂 WebEngine（纯逻辑测试用，图仍然会算好放在 ``last_figure``）。
            robots: :func:`laser_web.core.list_robots` 的返回值；``None`` 时现查。
            parent: 父控件。
        """
        super().__init__(parent)
        self._plot = plot
        self._robots = (core.list_robots(paths.asset_root())
                        if robots is None else list(robots))
        self._preview = None
        self.total = TotalPoints(None, "—", False)
        self.last_figure = None
        self.fields = {}
        self.metric_labels = {}
        self._debounce = QtCore.QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(self.DEBOUNCE_MS)
        self._debounce.timeout.connect(self.apply_figure)

        page_layout = QtWidgets.QVBoxLayout(self)
        page_layout.setContentsMargins(8, 8, 8, 8)
        page_layout.setSpacing(6)
        self.banner = QtWidgets.QLabel("")
        self.banner.setWordWrap(True)
        self.banner.hide()
        page_layout.addWidget(self.banner)

        body = QtWidgets.QWidget()
        outer = QtWidgets.QHBoxLayout(body)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)
        outer.addWidget(self._build_form(), 0)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(8)
        right.addWidget(self._build_preview_card(), 0)
        if self._plot is not None:
            right.addWidget(self._plot, 1)
        else:                                            # 纯逻辑测试/无图环境
            right.addWidget(_caption("路点图在这里（本环境未挂 WebEngine）"), 1)
        outer.addLayout(right, 1)
        page_layout.addWidget(body, 1)
        self.refresh_preview()

    # ------------------------------------------------------------------ 表单
    def _build_form(self) -> QtWidgets.QScrollArea:
        """左侧表单：六个分组塞进 ``QScrollArea``。"""
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setSpacing(8)
        for builder in (self._group_robot, self._group_domain, self._group_grid,
                        self._group_pose, self._group_perturb, self._group_guard):
            layout.addWidget(builder())
        layout.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(455)
        scroll.setMaximumWidth(560)
        return scroll

    def _group_robot(self) -> QtWidgets.QGroupBox:
        """机器人模型：只列 ``Nexbot6R`` 真能加载的；其余折叠成一句提示。"""
        group = QtWidgets.QGroupBox("机器人模型")
        form = QtWidgets.QFormLayout(group)
        usable = [robot for robot in self._robots if robot["usable"]]
        skipped = [robot for robot in self._robots if not robot["usable"]]

        combo = QtWidgets.QComboBox()
        for robot in usable:
            combo.addItem(robot["label"], robot["path"])
        if combo.count() == 0:                           # 一个都用不了时别空着
            combo.addItem(ui_state.default_robot(), ui_state.default_robot())
        default_path = ui_state.default_robot()
        index = combo.findData(default_path)
        combo.setCurrentIndex(index if index >= 0 else 0)
        combo.currentIndexChanged.connect(self._on_change)
        self.fields["robot"] = combo
        form.addRow("模型", combo)

        self.robot_path_label = _caption(f"路径：{combo.currentData()}")
        form.addRow(self.robot_path_label)
        combo.currentIndexChanged.connect(
            lambda *_args: self.robot_path_label.setText(
                f"路径：{combo.currentData()}"))

        self.skipped_label = _caption("")
        if skipped:
            detail = "；".join(f"{robot['label']}（{robot['reason']}）"
                              for robot in skipped)
            self.skipped_label.setText(
                f"另有 {len(skipped)} 个文件不能当模型用")
            self.skipped_label.setToolTip(detail)
        else:
            self.skipped_label.setText("data/configs 下的文件全部可以当模型用")
        form.addRow(self.skipped_label)
        return group

    def _group_domain(self) -> QtWidgets.QGroupBox:
        """采样域：两点对角点 / 六个轴边界（互斥，二选一）。"""
        defaults = form_defaults()
        group = QtWidgets.QGroupBox("采样域")
        form = QtWidgets.QFormLayout(group)

        points = QtWidgets.QRadioButton("两点输入")
        axes = QtWidgets.QRadioButton("六个轴边界")
        buttons = QtWidgets.QButtonGroup(group)
        buttons.addButton(points, 0)
        buttons.addButton(axes, 1)
        self.fields["domain_points"] = points
        self.fields["domain_axes"] = axes
        form.addRow(_row(points, axes))

        self.domain_stack = QtWidgets.QStackedWidget()
        points_panel = QtWidgets.QWidget()
        points_layout = QtWidgets.QVBoxLayout(points_panel)
        points_layout.setContentsMargins(0, 0, 0, 0)
        points_layout.setSpacing(4)
        lows, highs = [], []
        for axis in range(3):
            name = "XYZ"[axis]
            low = _double_spin(-10000.0, 10000.0, defaults["point_min"][axis])
            high = _double_spin(-10000.0, 10000.0, defaults["point_max"][axis])
            low.valueChanged.connect(self._on_change)
            high.valueChanged.connect(self._on_change)
            self.fields[f"min_{name.lower()}"] = low
            self.fields[f"max_{name.lower()}"] = high
            lows.append((f"{name} 最小", low))
            highs.append((f"{name} 最大", high))
        points_layout.addWidget(_inline(lows))
        points_layout.addWidget(_inline(highs))
        self.domain_stack.addWidget(points_panel)

        axes_panel = QtWidgets.QWidget()
        axes_layout = QtWidgets.QVBoxLayout(axes_panel)
        axes_layout.setContentsMargins(0, 0, 0, 0)
        axes_layout.setSpacing(4)
        lows, highs = [], []
        for axis in range(3):
            name = "XYZ"[axis]
            low = _double_spin(-10000.0, 10000.0, defaults["aabb6"][2 * axis])
            high = _double_spin(-10000.0, 10000.0, defaults["aabb6"][2 * axis + 1])
            low.valueChanged.connect(self._on_change)
            high.valueChanged.connect(self._on_change)
            self.fields[f"{name.lower()}_min"] = low
            self.fields[f"{name.lower()}_max"] = high
            lows.append((f"{name} 下界", low))
            highs.append((f"{name} 上界", high))
        axes_layout.addWidget(_inline(lows))
        axes_layout.addWidget(_inline(highs))
        self.domain_stack.addWidget(axes_panel)

        form.addRow(self.domain_stack)
        points.setChecked(defaults["domain_mode"] == DOMAIN_MODES[0])
        axes.setChecked(defaults["domain_mode"] == DOMAIN_MODES[1])
        self.domain_stack.setCurrentIndex(0 if points.isChecked() else 1)
        points.toggled.connect(self._on_domain_switch)
        axes.toggled.connect(self._on_domain_switch)
        return group

    def _group_grid(self) -> QtWidgets.QGroupBox:
        """网格：目标点数 / 手填格点数；总点数就地显示（不用只看右侧）。"""
        defaults = form_defaults()
        group = QtWidgets.QGroupBox("网格")
        form = QtWidgets.QFormLayout(group)

        count = QtWidgets.QRadioButton("目标点数")
        explicit = QtWidgets.QRadioButton("手填格点数")
        buttons = QtWidgets.QButtonGroup(group)
        buttons.addButton(count, 0)
        buttons.addButton(explicit, 1)
        self.fields["grid_count"] = count
        self.fields["grid_explicit"] = explicit
        form.addRow(_row(count, explicit))

        self.grid_stack = QtWidgets.QStackedWidget()
        count_panel = QtWidgets.QWidget()
        count_form = QtWidgets.QFormLayout(count_panel)
        target = _int_spin(1, TARGET_COUNT_MAX, defaults["target_count"])
        target.valueChanged.connect(self._on_change)
        self.fields["target_count"] = target
        count_form.addRow("目标点数", target)
        count_form.addRow(_caption(
            f"自动凑一组 X×Y×Z 格点，恰好凑出这个点数。上限 "
            f"{TARGET_COUNT_MAX} = {scan.COUNT_GRID_MAX_PER_AXIS + 1}³ + "
            f"{scan.COUNT_GRID_MAX_PER_AXIS}×{scan.COUNT_GRID_MAX_INSERT}"
            f"（单轴份数 / 单次插点的硬约束），再往上凑不出来；"
            f"凑不出来会显示 core 的报错原文。"))
        self.grid_stack.addWidget(count_panel)

        explicit_panel = QtWidgets.QWidget()
        explicit_layout = QtWidgets.QVBoxLayout(explicit_panel)
        explicit_layout.setContentsMargins(0, 0, 0, 0)
        explicit_layout.setSpacing(4)
        columns = []
        for key, title in (("xs", "X 格点数"), ("ys", "Y 格点数"),
                           ("zs", "Z 格点数"), ("z_insert", "换层插点")):
            maximum = 20 if key == "z_insert" else 200
            box = _int_spin(0 if key == "z_insert" else 1, maximum,
                            defaults[key])
            box.valueChanged.connect(self._on_change)
            self.fields[key] = box
            columns.append((title, box))
        explicit_layout.addWidget(_inline(columns))
        # 文案里的公式必须是真公式：插点按“换层次数”算，即 Z 格点数 − 1
        self.grid_caption = _caption(
            "格点数 = 该方向的点数（旧的“份数” + 1）；"
            "总点数 = X×Y×Z 格点 + (Z 格点数−1) × 换层插点。")
        explicit_layout.addWidget(self.grid_caption)
        self.grid_stack.addWidget(explicit_panel)

        form.addRow(self.grid_stack)

        # 总点数就地显示：填格点时要当场看到总数，不能只看右侧卡片
        self.total_value = QtWidgets.QLabel("—")
        self.total_hint = _caption(f"（超过 {POINT_COUNT_WARN} 点，无法生成）")
        form.addRow("总点数", _row(self.total_value, self.total_hint))

        count.setChecked(defaults["grid_mode"] == GRID_MODES[0])
        explicit.setChecked(defaults["grid_mode"] == GRID_MODES[1])
        self.grid_stack.setCurrentIndex(0 if count.isChecked() else 1)
        count.toggled.connect(self._on_grid_switch)
        explicit.toggled.connect(self._on_grid_switch)
        return group

    def _group_pose(self) -> QtWidgets.QGroupBox:
        """参考姿态：A/B/C 三个角 + 欧拉约定（显示「内旋」，值仍是算法约定名）。"""
        defaults = form_defaults()
        group = QtWidgets.QGroupBox("参考姿态")
        form = QtWidgets.QFormLayout(group)
        columns = []
        for index, (key, title) in enumerate(
                (("rpy_a", "A (roll)"), ("rpy_b", "B (pitch)"),
                 ("rpy_c", "C (yaw)"))):
            box = _double_spin(-360.0, 360.0, defaults["rpy"][index], UNIT_DEG)
            box.valueChanged.connect(self._on_change)
            self.fields[key] = box
            columns.append((title, box))
        form.addRow(_inline(columns))

        combo = QtWidgets.QComboBox()
        for value, label in ui_state.euler_convention_options().items():
            combo.addItem(label, value)
        index = combo.findData(defaults["euler"])
        combo.setCurrentIndex(index if index >= 0 else 0)
        combo.currentIndexChanged.connect(self._on_change)
        self.fields["euler"] = combo
        form.addRow("欧拉约定", combo)
        return group

    def _group_perturb(self) -> QtWidgets.QGroupBox:
        """姿态扰动：三轴半宽 + 随机种子。"""
        defaults = form_defaults()
        group = QtWidgets.QGroupBox("姿态扰动")
        form = QtWidgets.QFormLayout(group)
        columns = []
        for index, (key, title) in enumerate(
                (("perturb_a", "±A (roll)"), ("perturb_b", "±B (pitch)"),
                 ("perturb_c", "±C (yaw)"))):
            box = _double_spin(0.0, 90.0, defaults["perturb_deg"][index], UNIT_DEG)
            box.valueChanged.connect(self._on_change)
            self.fields[key] = box
            columns.append((title, box))
        form.addRow(_inline(columns))

        seed = _int_spin(0, 2_147_483_647, defaults["seed"])
        seed.valueChanged.connect(self._on_change)
        self.fields["seed"] = seed
        form.addRow("随机种子", seed)
        return group

    def _group_guard(self) -> QtWidgets.QGroupBox:
        """守卫：|J5| / |J3| / 段内 / 条件数 / 段模型 / 段采样 / 重采上限 / q-seed。

        这些阈值 S2 只负责送到 core（校验同一套规则），真正用它们是 S3 的解算。
        """
        defaults = form_defaults()
        group = QtWidgets.QGroupBox("守卫")
        form = QtWidgets.QFormLayout(group)
        columns = []
        for key, title in (("j5_min", "|J5| 下限"), ("j3_min", "|J3| 下限"),
                           ("seg_min", "段内下限")):
            box = _double_spin(0.0, 90.0, defaults[key], UNIT_DEG)
            box.valueChanged.connect(self._on_change)
            self.fields[key] = box
            columns.append((title, box))
        form.addRow(_inline(columns))

        cond = _double_spin(1.0, 1000.0, defaults["cond_max"], "",
                            decimals=2, step=0.5)
        cond.valueChanged.connect(self._on_change)
        self.fields["cond_max"] = cond

        mode = QtWidgets.QComboBox()
        for value, label in SEG_MODES.items():
            mode.addItem(label, value)
        index = mode.findData(defaults["seg_mode"])
        mode.setCurrentIndex(index if index >= 0 else 0)
        mode.currentIndexChanged.connect(self._on_change)
        self.fields["seg_mode"] = mode

        samples = _int_spin(2, 200, defaults["seg_samples"])
        samples.valueChanged.connect(self._on_change)
        self.fields["seg_samples"] = samples

        # 网页版有「重采上限」（laser_web/pages/generate.py 的 j5_attempts），
        # 它进 collect() 后成 --j5-attempts；桌面版不能少这一项。
        attempts = _int_spin(1, 100, defaults["j5_attempts"])
        attempts.valueChanged.connect(self._on_change)
        self.fields["j5_attempts"] = attempts
        form.addRow(_inline([("条件数上限", cond), ("段采样", samples),
                             ("重采上限", attempts)]))
        form.addRow("段模型", mode)

        q_seed = QtWidgets.QLineEdit(str(defaults["q_seed"]))
        q_seed.textChanged.connect(self._on_change)
        self.fields["q_seed"] = q_seed
        form.addRow("q-seed（6 个角）", q_seed)
        return group

    def _build_preview_card(self) -> QtWidgets.QGroupBox:
        """预览卡片：体积 / 格点 / 总点数 / 设计条件数 / 欧拉约定 + 报错原文。"""
        group = QtWidgets.QGroupBox("预览（不跑 IK）")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)

        grid = QtWidgets.QFormLayout()
        for key, title in (("volume", "体积"), ("grid", "格点 X×Y×Z"),
                           ("points", "总点数"), ("design", "设计条件数"),
                           ("euler", "欧拉约定")):
            value = QtWidgets.QLabel("—")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.metric_labels[key] = value
            grid.addRow(title, value)
        layout.addLayout(grid)

        self.over_label = QtWidgets.QLabel("")
        self.over_label.setWordWrap(True)
        self.over_label.setStyleSheet(_RED)
        self.over_label.hide()
        layout.addWidget(self.over_label)

        self.error_label = QtWidgets.QLabel("")
        self.error_label.setWordWrap(True)
        self.error_label.setStyleSheet(_RED)
        self.error_label.hide()
        layout.addWidget(self.error_label)

        self.warning_label = QtWidgets.QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet("color:#8a6d3b")
        self.warning_label.hide()
        layout.addWidget(self.warning_label)

        self.generate_button = QtWidgets.QPushButton("生成并解算（S3 接入）")
        self.generate_button.clicked.connect(self._on_generate_clicked)
        layout.addWidget(self.generate_button)
        layout.addWidget(_caption(
            "S2 只做预览：改任何一项都会重算上面这些数，并重画右侧路点图；"
            "解算、进度与导出在 S3 接入。"))
        return group

    # ------------------------------------------------------------------ 取值
    def values(self) -> dict:
        """当前表单值（键与 :func:`form_to_params` 的入参一致）。"""
        fields = self.fields
        return {
            "robot": fields["robot"].currentData() or ui_state.default_robot(),
            "domain_mode": (DOMAIN_MODES[0] if fields["domain_points"].isChecked()
                            else DOMAIN_MODES[1]),
            "point_min": [fields["min_x"].value(), fields["min_y"].value(),
                          fields["min_z"].value()],
            "point_max": [fields["max_x"].value(), fields["max_y"].value(),
                          fields["max_z"].value()],
            "aabb6": [fields["x_min"].value(), fields["x_max"].value(),
                      fields["y_min"].value(), fields["y_max"].value(),
                      fields["z_min"].value(), fields["z_max"].value()],
            "grid_mode": (GRID_MODES[0] if fields["grid_count"].isChecked()
                          else GRID_MODES[1]),
            "target_count": int(fields["target_count"].value()),
            "xs": int(fields["xs"].value()),
            "ys": int(fields["ys"].value()),
            "zs": int(fields["zs"].value()),
            "z_insert": int(fields["z_insert"].value()),
            "rpy": [fields["rpy_a"].value(), fields["rpy_b"].value(),
                    fields["rpy_c"].value()],
            "euler": fields["euler"].currentData(),
            "perturb_deg": [fields["perturb_a"].value(),
                            fields["perturb_b"].value(),
                            fields["perturb_c"].value()],
            "seed": int(fields["seed"].value()),
            "cond_max": float(fields["cond_max"].value()),
            "j5_min": float(fields["j5_min"].value()),
            "j3_min": float(fields["j3_min"].value()),
            "j5_attempts": int(fields["j5_attempts"].value()),
            "seg_min": float(fields["seg_min"].value()),
            "seg_mode": fields["seg_mode"].currentData(),
            "seg_samples": int(fields["seg_samples"].value()),
            "q_seed": fields["q_seed"].text(),
        }

    def params(self) -> dict:
        """当前表单值 -> ``core.preview_domain`` 的入参。"""
        return form_to_params(self.values())

    def preview_result(self) -> dict:
        """最近一次 ``core.preview_domain`` 的结果（原样，含报错原文）。"""
        return self._preview or {}

    # ------------------------------------------------------------ 刷新与出图
    def _on_change(self, *_args) -> None:
        """任何输入的改动都走这里：立刻重算预览，图延后去抖重画。"""
        self.refresh_preview()

    def _on_domain_switch(self, checked: bool) -> None:
        """两点 / 六数切换：换面板并重算（只在被选中的那个按钮上动一次）。"""
        if not checked:
            return
        self.domain_stack.setCurrentIndex(
            0 if self.fields["domain_points"].isChecked() else 1)
        self.refresh_preview()

    def _on_grid_switch(self, checked: bool) -> None:
        """目标点数 / 手填格点数切换。"""
        if not checked:
            return
        self.grid_stack.setCurrentIndex(
            0 if self.fields["grid_count"].isChecked() else 1)
        self.refresh_preview()

    def refresh_preview(self) -> dict:
        """重算预览卡片与总点数，并按结果启用/禁用「生成并解算」。

        ``core.preview_domain`` 外面包了兜底：算法层抛出的任何异常都转成
        ``preview_failure`` 的报错结果，走同一条错误显示路径（卡片红字、按钮禁用），
        不让异常穿过 Qt 槽（那会让槽提前 return，界面停在旧状态）。

        Returns:
            dict: ``core.preview_domain`` 的结果（``ok``/``error``/…）。
        """
        values = self.values()
        try:
            result = core.preview_domain(form_to_params(values))
        except Exception as error:                      # noqa: BLE001 - 别打断 Qt 槽
            _LOGGER.exception("core.preview_domain 抛异常（预览卡片显示失败原因）")
            result = preview_failure(error)
        self._preview = result
        self.total = total_points(values, result)
        self._update_total_row()
        self._update_card(result)
        self.generate_button.setEnabled(bool(result.get("ok"))
                                        and not self.total.over)
        self._debounce.start()
        return result

    def _update_total_row(self) -> None:
        """网格组里的总点数行（超上限时变红并给出说明）。"""
        total = self.total
        self.total_value.setText(total.text)
        self.total_value.setStyleSheet(_RED_BOLD if total.over else "")
        self.total_hint.setVisible(total.over)

    def _update_card(self, result: dict) -> None:
        """预览卡片：数值或 core 的报错原文（红字，不弹窗）。"""
        if not result.get("ok"):
            self.error_label.setText(str(result.get("error") or "预览失败"))
            self.error_label.show()
            self.over_label.hide()
            self.warning_label.hide()
            for label in self.metric_labels.values():
                label.setText("—")
                label.setStyleSheet("")
            return
        self.error_label.hide()
        grid = result["grid"]
        over = int(result["point_count"]) > POINT_COUNT_WARN
        self.metric_labels["volume"].setText(f"{result['volume_m3']:.3f} m³")
        insert = (f" + {grid['zs'] * grid['z_insert']} 插点"
                  if grid["z_insert"] else "")
        self.metric_labels["grid"].setText(
            f"{grid['xs'] + 1}×{grid['ys'] + 1}×{grid['zs'] + 1}{insert}")
        points = self.metric_labels["points"]
        points.setText(f"{result['point_count']} 点")
        points.setStyleSheet(_RED_BOLD if over else "")
        self.metric_labels["design"].setText(
            format_condition(result["design_condition"]))
        self.metric_labels["euler"].setText(
            ui_state.euler_label(result["convention"]))

        if over:
            self.over_label.setText(
                f"超过 {POINT_COUNT_WARN} 点：规划与解算会占满内存，且取消要等"
                f"当前阶段结束。请减少格点数或改用目标点数。")
        else:
            self.over_label.setText("")
        self.over_label.setVisible(over)

        warnings = result.get("warnings") or []
        self.warning_label.setText("\n".join(str(line) for line in warnings))
        self.warning_label.setVisible(bool(warnings))

    def apply_figure(self):
        """按当前预览结果重建路点图（只规划，不跑 IK）。

        参数非法或点数超上限时给说明图，不去 ``plan_waypoints`` 里冒险
        （800 万点会直接吃穿内存）。

        Returns:
            plotly ``Figure``（同时存进 ``last_figure``；挂了 PlotView 就推过去）。
        """
        result = self._preview or {}
        if not result.get("ok"):
            figure = placeholder_figure(
                str(result.get("error") or "参数不可用，先不画图"))
        elif self.total.over:
            figure = placeholder_figure(
                f"当前设置 {self.total.count} 点，超过 {POINT_COUNT_WARN} 点上限，"
                f"先不画图。")
        else:
            try:
                figure = waypoint_figure(result["bounds"], result["grid"],
                                         self.values()["rpy"])
            except Exception as error:                   # noqa: BLE001 - 别让窗口崩
                # 图可以退成说明图，但异常不能只在界面上消失：日志留完整栈，
                # 状态栏留一句话（用户不一定会翻日志）。
                _LOGGER.exception("路点规划失败（表单值=%r）", self.values())
                figure = placeholder_figure(
                    f"规划失败：{type(error).__name__}: {error}")
                self._report_status(
                    f"路点规划失败：{type(error).__name__}: {error}")
        self.last_figure = figure
        if self._plot is not None:
            self._plot.set_figure(figure)
        return figure

    def _report_status(self, message: str) -> None:
        """把一句话写到窗口状态栏（没有窗口的纯逻辑测试里就只留日志）。

        ``self.window()`` 对没有父控件的页面返回页面自己（它没有 ``statusBar``），
        所以这里用 ``getattr`` 探测，而不是假设总有 ``QMainWindow``。

        Args:
            message: 给用户看的一句话（异常类型 + 原文）。
        """
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(str(message), 10_000)

    def _on_generate_clicked(self) -> None:
        """「生成并解算」：S2 只解释按钮还没接活儿（S3 才起后台任务）。"""
        QtWidgets.QMessageBox.information(
            self, "S3 接入",
            "这个按钮在 S3 接入：起后台任务跑 IK、显示进度、写 runs/<id>/ 与导出。\n"
            "当前 S2 只做预览（右侧路点图与预览卡片）。")

    # ------------------------------------------------------------------ 横幅
    def show_webgl_warning(self) -> None:
        """WebGL 不可用时在页面顶部挂一条中文建议（图区只说英文的话没用）。"""
        self.banner.setText(
            "⚠ 这台机器的 WebGL 不可用，Plotly 3D 画不出来。\n"
            "　处理办法：① 更新显卡驱动；② 远程桌面下改在本机打开；"
            "③ 先用网页版（python -m laser_web）看图，功能与数值一致。")
        self.banner.setStyleSheet(
            "background:#fff4e5; color:#8a4b00; padding:8px; border:1px solid #f0c48a;")
        self.banner.show()
