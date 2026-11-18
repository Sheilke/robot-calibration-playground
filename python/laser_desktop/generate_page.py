# -*- coding: utf-8 -*-
"""生成页（S3）：完整表单 + 实时预览 + **真跑解算**（``core.run_scan`` + ``jobs``）。

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
  不让异常打断 Qt 槽、把界面停在旧状态。

S3 的"真跑"路径（线程模型见 :mod:`laser_desktop.job_bridge`）
--------------------------------------------------------------
1. 点「生成并解算」→ :meth:`GeneratePage.start_scan` 再向 ``core.preview_domain``
   要一次确认（非法就只把报错原文写在卡片里，**不起任务**）；
2. ``JobBridge.submit("scan", lambda job: core.run_scan(params, job=job,
   on_progress=job.set_progress))`` —— ``core`` 与 ``jobs`` 一行未改，解算跑在
   ``jobs`` 自己的那条工作线程里；
3. 进度 / 日志 / 终态全部通过 :class:`laser_desktop.job_bridge.JobBridge` 的
   信号回到 GUI 线程（页面从不轮询 ``snapshot()``）；
4. 终态：``done`` → 统计标签 + 路点表 + 段诊断表 + 结果图（失败点红叉）；
   ``cancelled`` → 灰字中性提示（不报红）；``failed`` → 红色报错原文 + 日志保留；
5. 导出（``scan.json`` / ``joints.txt`` / ``plot.png``）用 ``QFileDialog`` 选
   目标后从 ``run_dir`` **复制**（复制，不移动：``runs/`` 是归档）。

图只在参数合法且点数不超上限时才规划（``plan_waypoints`` 是毫秒级，但 800 万点
会直接把内存吃穿），并用 300 ms 去抖，避免拖动 spinbox 时反复重算。
"""

from __future__ import annotations

import logging
import math
import shutil
from pathlib import Path
from typing import NamedTuple

from excitation_trajectory import aabb_snake_scan as scan
from laser_web import core, ui_state

from . import paths
from .job_bridge import LOG_LIMIT as JOB_LOG_LIMIT
from .job_bridge import STATE_TEXT, JobBridge, adopt_running_job
from .qt_compat import Qt, QtCore, QtGui, QtWidgets

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

#: 日志区最多保留的行数（超出删头部；靠 ``QPlainTextEdit.setMaximumBlockCount``）。
LOG_WIDGET_LIMIT = 500

#: 出结果后结果区至少占多高（像素）：路点表要露出表头 + 几行，
#: 默认的 120 px 只够看表头（S3 评审核对过截图）。
RESULT_MIN_HEIGHT = 260

#: 进度条的量程：分数（0~1）乘它取整，比 0~100 细一个量级，又不用浮点。
PROGRESS_STEPS = 1000

#: 结果区统计标签的键（顺序即界面顺序；单测按这些键取值）。
STAT_KEYS = ("solved", "switches", "dq_l2", "j5_min", "j3_min", "cond",
             "feasible")

#: 路点表列头（与网页版一致，只是少了网页版的 dqL2 列，规格里没要求）。
WAYPOINT_HEADERS = ("GP", "X (mm)", "Y (mm)", "Z (mm)",
                    "q1", "q2", "q3", "q4", "q5", "q6",
                    "|J3|°", "|J5|°", "cfg", "ok")

#: 段诊断表列头（GP 起→止 / |J3|min / |J5|min / 穿零 / 最差点）。
SEGMENT_HEADERS = ("GP 起→止", "|J3|min °", "|J5|min °", "穿零",
                   "最差点 (mm, t)")

#: 导出按钮：``(artifacts 键, 按钮文案, 文件对话框过滤器)``。
EXPORTS = (("scan_json", "导出 scan.json", "JSON (*.json);;所有文件 (*)"),
           ("joints", "导出 joints.txt", "文本 (*.txt);;所有文件 (*)"),
           ("plot", "导出 plot.png", "PNG 图片 (*.png);;所有文件 (*)"))

#: 结果区状态行的颜色：红 / 绿 / 灰（取消、中性提示）/ 中性。
_STATUS_STYLES = {"error": "color:#c0392b", "ok": "color:#1e8449",
                  "muted": "color:#6b7280", "info": ""}

#: 成功计数的绿色（与 ``_RED`` / ``_RED_BOLD`` 对偶）。
_GREEN_BOLD = "color:#1e8449; font-weight:bold"

#: 失败行的底色 / 字色（浅红底 + 深红字，比纯红底易读）。
_FAILED_BG = "#fde8e8"
_FAILED_FG = "#b00020"


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


def format_metric(value, digits: int = 1) -> str:
    """结果区数字的显示格式（``None``/``nan`` → ``—``，``inf`` → ``∞``）。

    与 :func:`format_condition` 同一个道理：奇异参考姿态的 ``inf`` 是姿态问题，
    不是程序 bug，不能原样打 ``inf``；失败路点没有关节角，留空会被当成“值太小
    被吃掉了”。

    Args:
        value: ``stats`` / 行里的数字（可能是 ``None``）。
        digits: 小数位。

    Returns:
        str: 如 ``"123.46"``、``"—"``、``"∞"``。
    """
    if value is None:
        return "—"
    number = float(value)
    if math.isinf(number):
        return "∞"
    if math.isnan(number):
        return "—"
    return f"{number:.{digits}f}"


def sort_key(value):
    """表格排序键：缺值（``None`` / ``nan`` / ``inf``）一律排到末尾。

    返回的元组第一项是“有没有值”（0 有 1 无），第二项是数值。统一成元组，
    同一列里就不会既有 float 又有 str 比大小。

    Args:
        value: 单元格的原始数值。

    Returns:
        tuple: ``(rank, number)``。
    """
    if value is None:
        return (1, 0.0)
    number = float(value)
    if not math.isfinite(number):
        return (1, 0.0)
    return (0, number)


def result_figure(result: dict):
    """结果图：成功点按路径顺序着色、**失败点红叉**、段内最差点叠一层方块。

    直接复用网页版 :func:`laser_web.figures.waypoint_figure`（同一份代码，
    自然与网页版逐点一致）：失败点用红色 ``x``，hover 里有 GP / XYZ /
    ``|J3|`` / ``|J5|`` 与 ok。

    Args:
        result: ``core.run_scan`` 的返回值。

    Returns:
        plotly ``Figure``。
    """
    # 与 ``waypoint_figure`` 一样把 plotly 留到用时再 import：本模块在缺 plotly
    # 的环境里也能 import（S2 就是这样，别把依赖提前）
    from laser_web import figures

    meta = result.get("meta") or {}
    return figures.waypoint_figure(
        result.get("waypoints") or [],
        color_by="order",
        worsts=result.get("segments"),
        all_ok=bool(meta.get("skip_ik")))


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
        # ---- 守卫 / 段检查 / 随机种子（S3 的解算真的用它们） ----
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


#: 等宽字体候选（Linux / Windows 都覆盖；按顺序取第一个装了的）。
#: ``QFontDatabase.systemFont(FixedFont)`` 在国内机器上可能返回一个非等宽的
#: 中文字体（实测返回 Noto Sans CJK SC），所以还得自己点名几个真等宽族。
MONOSPACE_FAMILIES = ("Consolas", "DejaVu Sans Mono", "Liberation Mono",
                      "Noto Sans Mono", "Courier New", "Menlo", "Monaco")


def monospace_font() -> QtGui.QFont:
    """日志区用的等宽字体（候选族都找不到时退回系统定宽字体，并置位定宽标志）。

    ``QFontDatabase.systemFont(FixedFont)`` 在两个绑定上都可用；但 PySide6 的
    严格枚举写法是 ``QFontDatabase.FontType.FixedFont``，所以扁平名取不到时
    退回嵌套枚举，不写死任何一种。

    Returns:
        QtGui.QFont: 等宽字体。
    """
    database = QtGui.QFontDatabase
    fixed = getattr(database, "FixedFont", None)
    if fixed is None:                        # pragma: no cover - PySide6 严格枚举
        fixed = database.FontType.FixedFont
    font = database.systemFont(fixed)
    try:
        available = set(database().families())
    except Exception:                        # noqa: BLE001 - 无字形库也不该开不了窗口
        available = set()
    for family in MONOSPACE_FAMILIES:
        if family in available:
            font.setFamily(family)
            break
    font.setFixedPitch(True)
    return font


class SortItem(QtWidgets.QTableWidgetItem):
    """可按**数值**排序的单元格：显示文本自己格式化，排序键单独存。

    ``QTableWidgetItem`` 默认按文本比大小，``"10.0" < "2.0"`` 就错了；而把
    浮点数直接放进 DisplayRole 又会把小数位丢掉。``operator<`` 在 Qt 里是
    ``virtual``，所以可以在 Python 里重载（PyQt5 / PySide6 都支持）。
    """

    def __init__(self, key, text: str) -> None:
        """建单元格。

        Args:
            key: 排序键（``sort_key()`` 的元组，或纯文本）。
            text: 显示文本。
        """
        super().__init__(str(text))
        self.sort_key = key

    def __lt__(self, other) -> bool:         # noqa: D105 - 排序协议
        try:
            return self.sort_key < other.sort_key
        except (AttributeError, TypeError):
            return super().__lt__(other)


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class GeneratePage(QtWidgets.QWidget):
    """生成页：左侧可滚动表单 + 右上预览卡片 + 右下（图 + 结果区）。

    所有输入变化都会立刻重算预览卡片（``core.preview_domain`` 约 1 ms），
    路点图则交给 :data:`DEBOUNCE_MS` 的去抖定时器，避免拖动 spinbox 时卡顿。

    Attributes:
        fields: ``{键: 控件}``，取值与 :func:`form_to_params` 的入参同名。
        metric_labels: 预览卡片里的数值标签。
        generate_button: 「生成并解算」（参数非法 / 超上限 / 任务在跑时禁用）。
        total: 最近一次的 :class:`TotalPoints`（网格组与预览卡片共用同一口径）。
        bridge: :class:`laser_desktop.job_bridge.JobBridge`（提交/取消/接管）。
        progress_bar: 任务进度条（格式文本就是 ``阶段 索引/总数 · 状态``）。
        cancel_button: 「取消」（只在任务进行中可见）。
        stat_labels: 结果区统计标签，键见 :data:`STAT_KEYS`。
        waypoint_table: 路点表（可排序，失败行标红）。
        segment_table: 段诊断表。
        log_edit: 日志区（只读、等宽、最多 :data:`LOG_WIDGET_LIMIT` 行）。
        export_buttons: ``{artifacts 键: 按钮}``，有产物才可用。
        last_result: 最近一次建成的 ``core.run_scan`` 返回值（只看不跑）。
        last_status: 最近一次写到状态栏的文字（无主窗口时也能断言）。
    """

    #: 预览图去抖间隔（毫秒）。
    DEBOUNCE_MS = DEBOUNCE_MS

    def __init__(self, plot=None, robots=None, parent=None, bridge=None) -> None:
        """建页面。

        Args:
            plot: ``PlotView``（或任何有 ``set_figure`` 的对象）；``None`` 表示
                不挂 WebEngine（纯逻辑测试用，图仍然会算好放在 ``last_figure``）。
            robots: :func:`laser_web.core.list_robots` 的返回值；``None`` 时现查。
            parent: 父控件。
            bridge: 现成的 :class:`JobBridge`（测试注入假登记表用）；``None``
                时按 :func:`laser_desktop.paths.runs_root` 新建一个。
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
        self.stat_labels = {}
        self.export_buttons = {}
        self.last_result = None
        self.last_status = ""
        self.run_state = "idle"
        self.last_job_id = None
        self._busy = False
        # 表单改动序号 / 出结果时的改动序号：两者相等说明“结果渲染后表单没再变”，
        # 这时到点的预览去抖不许拿预览图盖掉结果图（见 apply_figure）。
        self._change_seq = 0
        self._result_seq = -1
        self.bridge = bridge if bridge is not None else JobBridge(parent=self)
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
        right.addWidget(self._build_viewer(), 1)
        outer.addLayout(right, 1)
        page_layout.addWidget(body, 1)

        # 信号连在控件建完之后：槽里直接碰控件（都在 GUI 线程，见 job_bridge）
        self.bridge.progress.connect(self._on_progress)
        self.bridge.log_line.connect(self._on_log_line)
        self.bridge.finished.connect(self._on_finished)
        self.refresh_preview()
        self._resume_running_job()

    def _build_viewer(self) -> QtWidgets.QWidget:
        """图 + 结果区：上下可拖的 ``QSplitter``（默认给图多一点高度）。"""
        if self._plot is not None:
            viewer = self._plot
        else:                                            # 纯逻辑测试/无图环境
            viewer = _caption("路点图在这里（本环境未挂 WebEngine）")
        result_area = self._build_result_area()
        splitter = QtWidgets.QSplitter(Qt.Vertical)
        splitter.addWidget(viewer)
        splitter.addWidget(result_area)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        # 初始尺寸：结果区的 sizeHint 比图大（表格），不指定的话图会被压成一条
        splitter.setSizes([420, 120])
        splitter.setChildrenCollapsible(False)
        self._viewer_splitter = splitter
        return splitter

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

        这些阈值一路送到 ``core.run_scan``：校验与用量都是同一套规则（S3 起真
        拿它们解算）。
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

        self.generate_button = QtWidgets.QPushButton("生成并解算")
        self.generate_button.setToolTip(
            "起一条后台任务：生成路点 → 解算 IK → 段检查 → 条件数 → 写 runs/<id>/")
        self.generate_button.clicked.connect(self._on_generate_clicked)
        layout.addWidget(self.generate_button)
        layout.addWidget(_caption(
            "改任何一项都会重算上面这些数，并重画右侧路点图；"
            "点「生成并解算」才真的跑 IK（在后台线程，可取消）。"))
        return group

    def _build_result_area(self) -> QtWidgets.QGroupBox:
        """结果区：状态行 + 进度/取消 + 统计标签 + 三页签（路点 / 段诊断 / 日志） + 导出。"""
        group = QtWidgets.QGroupBox("结果")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.status_label.hide()
        layout.addWidget(self.status_label)

        # 进度条本身的格式文本就是"阶段 索引/总数 · 状态"（规格要求的展示位）
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, PROGRESS_STEPS)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("空闲")
        self.cancel_button = QtWidgets.QPushButton("取消")
        self.cancel_button.setToolTip("只在阶段边界生效：当前阶段跑完就停，不会硬打断 IK")
        self.cancel_button.clicked.connect(self._on_cancel_clicked)
        self.cancel_button.hide()
        progress_row = QtWidgets.QHBoxLayout()
        progress_row.setContentsMargins(0, 0, 0, 0)
        progress_row.addWidget(self.progress_bar, 1)
        progress_row.addWidget(self.cancel_button, 0)
        layout.addLayout(progress_row)

        stats_row = QtWidgets.QHBoxLayout()
        stats_row.setContentsMargins(0, 0, 0, 0)
        stats_row.setSpacing(12)
        for key in STAT_KEYS:
            label = QtWidgets.QLabel("—")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.stat_labels[key] = label
            stats_row.addWidget(label)
        stats_row.addStretch(1)
        self.stats_holder = QtWidgets.QWidget()
        self.stats_holder.setLayout(stats_row)
        self.stats_holder.hide()        # 没结果时不占高度（预览时图可以高一点）
        layout.addWidget(self.stats_holder)

        self.result_tabs = QtWidgets.QTabWidget()
        self.waypoints_page = self._build_waypoints_page()
        self.segments_page = self._build_segments_page()
        self.log_page = self._build_log_page()
        self.result_tabs.addTab(self.waypoints_page, "路点")
        self.result_tabs.addTab(self.segments_page, "段诊断")
        self.result_tabs.addTab(self.log_page, "日志")
        self.result_tabs.hide()         # 起任务后才展开（预览时把高度让给图）
        layout.addWidget(self.result_tabs, 1)

        export_row = QtWidgets.QHBoxLayout()
        export_row.setContentsMargins(0, 0, 0, 0)
        export_row.addWidget(_caption("导出（从 runs/<id>/ 复制，不移动）："))
        for key, title, _filters in EXPORTS:
            button = QtWidgets.QPushButton(title)
            button.setEnabled(False)
            button.clicked.connect(
                lambda _checked=False, export_key=key: self._on_export(export_key))
            self.export_buttons[key] = button
            export_row.addWidget(button)
        export_row.addStretch(1)
        layout.addLayout(export_row)
        return group

    @staticmethod
    def _make_table(headers) -> QtWidgets.QTableWidget:
        """只读、整行选中、可排序的表格（填数据前记得先关排序）。"""
        table = QtWidgets.QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(list(headers))
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table.setAlternatingRowColors(True)
        table.verticalHeader().setVisible(False)
        table.setSortingEnabled(True)
        table.horizontalHeader().setStretchLastSection(True)
        return table

    def _build_waypoints_page(self) -> QtWidgets.QWidget:
        """「路点」页：一行一个 GP，失败行整行标红。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(_caption("点表头排序；无解 / 未通过自校验的行整行标红。"))
        self.waypoint_table = self._make_table(WAYPOINT_HEADERS)
        layout.addWidget(self.waypoint_table, 1)
        return page

    def _build_segments_page(self) -> QtWidgets.QWidget:
        """「段诊断」页：每段的最小裕度、是否穿零、最差点位置。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(_caption(
            "最差点 = 段内 |J3| 最小的位置（按关节直线解析求 t，坐标沿线插值）。"))
        self.segment_table = self._make_table(SEGMENT_HEADERS)
        layout.addWidget(self.segment_table, 1)
        return page

    def _build_log_page(self) -> QtWidgets.QWidget:
        """「日志」页：算法自己的 ``print`` 与 ``jobs`` 的提示，等价终端输出。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(_caption(
            f"等价终端输出。注意两层上限：后台任务源缓冲只留最近 {JOB_LOG_LIMIT} 行"
            f"（更早的行在源头就丢了，丢一段会插一行「…已省略 N 行」），本控件最多"
            f"保留 {LOG_WIDGET_LIMIT} 行（超出删头部）。"))
        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setFont(monospace_font())
        self.log_edit.setMaximumBlockCount(LOG_WIDGET_LIMIT)
        self.log_edit.setPlaceholderText("点「生成并解算」后，这里实时滚日志。")
        layout.addWidget(self.log_edit, 1)
        return page

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
        # 每重算一次预览就算“表单变了一版”（含两点/六数、目标/手填这种只走
        # ``_on_domain_switch`` / ``_on_grid_switch`` 的入口）：结果图的
        # 保护判据靠它区分“用户又改了”与“只是陈旧去抖到点了”。
        self._change_seq += 1
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
                                        and not self.total.over
                                        and not self._busy)
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
        if self._busy:
            # 解算期间不拿预览图盖住结果/进度；结束后用户再改表单才回到预览
            return self.last_figure
        if self._result_seq == self._change_seq and self.last_result:
            # 结果渲染之后表单没再变过：这是起任务前排下的陈旧去抖（任务在
            # DEBOUNCE_MS 内就结束了，``_busy`` 已经放开），不许拿预览图把刚
            # 推上去的结果图盖掉。改过表单（序号 +1）就照旧回到预览。
            return self.last_figure
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
        """把一句话写到窗口状态栏（没有窗口的纯逻辑测试里也能断言）。

        ``self.window()`` 对没有父控件的页面返回页面自己（它没有 ``statusBar``），
        所以这里用 ``getattr`` 探测，而不是假设总有 ``QMainWindow``；不管有没有
        状态栏，文字都存在 :attr:`last_status` 里，测试直接看它。

        Args:
            message: 给用户看的一句话。
        """
        self.last_status = str(message)
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(self.last_status, 10_000)

    # ------------------------------------------------------------ 起任务
    def _on_generate_clicked(self) -> None:
        """「生成并解算」按钮：起一条后台解算任务（详见 :meth:`start_scan`）。"""
        self.start_scan()

    def start_scan(self):
        """校验当前表单并起一次**真解算**。

        流程（与规格一致）：

        1. 先用当前表单值问一次 ``core.preview_domain``：不合法就只把报错原文
           写进预览卡片，**不起任务**；
        2. 清空结果区、显示进度条与「取消」、禁用「生成并解算」；
        3. ``bridge.submit("scan", lambda job: core.run_scan(params, job=job,
           on_progress=job.set_progress))``——``core`` 与 ``jobs`` 一行未改。

        Returns:
            str: 任务 id；未启动（参数非法 / 超上限 / 已有任务在跑）时为 ``None``。
        """
        if self.bridge.is_running:
            self._set_status("已有任务在跑：等它结束，或先点「取消」。", level="muted")
            self._report_status("已有任务在跑，未重复提交")
            return None
        values = self.values()
        params = form_to_params(values)
        try:
            preview = core.preview_domain(params)
        except Exception as error:                      # noqa: BLE001 - 别打断 Qt 槽
            _LOGGER.exception("core.preview_domain 抛异常（拒绝起任务）")
            preview = preview_failure(error)
        self._preview = preview
        self.total = total_points(values, preview)
        self._update_total_row()
        self._update_card(preview)
        if not preview.get("ok"):
            self._set_status(
                f"参数不合法，未启动解算：{preview.get('error') or '预览失败'}",
                level="error")
            self._report_status("参数不合法，未启动解算")
            return None
        if self.total.over:
            self._set_status(
                f"当前设置 {self.total.count} 点，超过 {POINT_COUNT_WARN} 点上限，"
                f"未启动解算。", level="error")
            self._report_status("点数超上限，未启动解算")
            return None

        self._reset_result_area()
        self._busy = True
        # 杀掉点按钮前排上的陈旧预览定时器：小案子 200~500 ms 就跑完，
        # 那个定时器会在 ``_busy`` 放开之后才响，正好把结果图盖成预览图。
        self._debounce.stop()
        self.run_state = "running"
        self.generate_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.cancel_button.show()
        self.result_tabs.show()         # 跑的时候把页签展开（先看日志）
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("排队 0/0 · 排队中")
        self.result_tabs.setCurrentWidget(self.log_page)   # 跑的时候先看日志
        self._set_status(f"已提交：{self.total.text}", level="info")
        try:
            job_id = self.bridge.submit(
                "scan",
                lambda job: core.run_scan(params, job=job,
                                          on_progress=job.set_progress))
        except Exception as error:                      # noqa: BLE001 - 别让槽抛出去
            # 提交本身也可能失败（runs/ 不可写、磁盘满）：不能让异常穿过 Qt 槽
            _LOGGER.exception("提交解算任务失败（params=%r）", params)
            self._busy = False
            self.run_state = "failed"
            self.cancel_button.hide()
            self.generate_button.setEnabled(True)
            self.progress_bar.setFormat("失败")
            self._set_status(f"提交任务失败：{type(error).__name__}: {error}",
                             level="error")
            self._report_status(f"提交任务失败：{type(error).__name__}: {error}")
            return None
        self.last_job_id = job_id
        self._report_status(f"已提交解算任务 {job_id}")
        return job_id

    def _resume_running_job(self) -> None:
        """启动时扫一遍**本桥登记表里**未结束的任务，有就接回来。

        网页版 ``adopt_running_job`` 同语义，但只在同一个 ``JobRegistry`` 内有效：
        另开窗口/进程是接不回来的（每个默认桥自建进程内登记表，见
        :func:`laser_desktop.job_bridge.adopt_running_job` 的边界说明）。
        """
        job = adopt_running_job(self.bridge.registry, "scan")
        if job is None:
            return
        if self.bridge.attach(job.id) is None:          # 刚好被清掉
            return
        self._busy = True
        self.run_state = "running"
        self.generate_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.cancel_button.show()
        self._set_status(
            f"接管本进程里还没跑完的任务 {job.id}"
            f"（当前 {STATE_TEXT.get(job.state, job.state)}）",
            level="info")

    def _on_cancel_clicked(self) -> None:
        """请求取消：只在阶段边界生效（不会硬打断 IK）。"""
        if not self.bridge.cancel():
            self.cancel_button.setEnabled(False)
            self._report_status("任务已结束，无需取消")
            return
        self.cancel_button.setEnabled(False)
        self._set_status("已请求取消：当前阶段跑完就停（不会硬打断 IK）。",
                         level="muted")
        self._report_status("已请求取消")

    # --------------------------------------------------------- 信号（GUI 线程）
    def _on_progress(self, progress: dict) -> None:
        """进度：进度条显示 ``阶段 索引/总数 · 状态``。"""
        stage = str(progress.get("stage") or "")
        index = int(progress.get("index") or 0)
        total = int(progress.get("total") or 0)
        fraction = float(progress.get("fraction") or 0.0)
        state = str(progress.get("state") or "")
        self.progress_bar.setValue(int(round(max(0.0, min(1.0, fraction))
                                            * PROGRESS_STEPS)))
        text = f"{stage} {index}/{total}"
        if state:
            text += f" · {STATE_TEXT.get(state, state)}"
        self.progress_bar.setFormat(text)

    def _on_log_line(self, line: str) -> None:
        """增量日志：直接追加（上限由 ``setMaximumBlockCount`` 管）。"""
        self.log_edit.appendPlainText(str(line))

    def _on_finished(self, payload: dict) -> None:
        """终态：done → 渲染结果；cancelled → 灰字；failed → 红字。"""
        state = str(payload.get("state") or "")
        result = payload.get("result")
        self._busy = False
        self.run_state = state
        self.cancel_button.hide()
        self.cancel_button.setEnabled(True)
        self.generate_button.setEnabled(
            bool((self._preview or {}).get("ok")) and not self.total.over)

        if state == "done" and isinstance(result, dict) and result.get("ok"):
            self.progress_bar.setValue(PROGRESS_STEPS)
            self.progress_bar.setFormat("完成 1/1 · 完成")
            self.render_result(result)
            stats = result.get("stats") or {}
            feasible = stats.get("feasible")
            # 出结果但不可行：原因已经在 core 的 warnings 里拼好了（“姿态激励
            # 条件数未达标：…”、“段内奇异…”、“N 个路点无解”），不能只把结论
            # 撂在这儿——现场得知道该改哪个参数。
            head = "完成" if feasible is not False else "完成（不可行）"
            text = (f"{head}：成功 {stats.get('solved')}/{stats.get('total')} 点，"
                    f"运行 {result.get('run_id')} → {result.get('run_dir')}")
            warnings = [str(line) for line in (result.get("warnings") or [])]
            if warnings:
                # 一行搞定：状态行每多一行，结果区就从图区多抢一点高度
                label = "原因" if feasible is False else "备注"
                text += f"\n{label}：" + "；".join(warnings)
            self._set_status(
                text, level="ok" if feasible is not False else "error")
            self.result_tabs.setCurrentWidget(self.waypoints_page)
            self._report_status(
                f"解算完成：{stats.get('solved')}/{stats.get('total')} 点，"
                f"{result.get('run_id')}"
                + ("" if feasible is not False
                   else "（不可行，原因见结果区）"))
            return

        if state == "cancelled":
            # 取消不是失败：现场工程师自己点的，用灰字而不是红字
            self.progress_bar.setFormat("已取消")
            self._set_status(
                "已取消：任务在当前阶段末尾停下；没落下结果 JSON 的半成品运行"
                "目录已删掉。", level="muted")
            self._report_status("解算已取消")
            return

        error = ((result or {}).get("error") if isinstance(result, dict) else None) \
            or payload.get("error") or "未知错误"
        warnings = (result or {}).get("warnings") if isinstance(result, dict) else None
        text = f"失败：{error}"
        if warnings:
            text += "\n" + "\n".join(f"· {line}" for line in warnings)
        self.progress_bar.setFormat("失败")
        self._set_status(text, level="error")
        self.result_tabs.setCurrentWidget(self.log_page)   # 日志区保留
        self._report_status(f"解算失败：{error}")

    # ------------------------------------------------------------- 结果渲染
    def render_result(self, result: dict) -> None:
        """把 ``core.run_scan`` 的返回值铺到结果区（统计 / 两张表 / 结果图）。

        参数是真的 ``run_scan`` 返回值还是构造的 dict 都行（单测就构造一份），
        这里只做展示：不重算、不碰算法。

        Args:
            result: ``core.run_scan`` 的成功返回值。
        """
        self.last_result = result
        # 记下“结果对应哪一版表单”：此后再没人改表单，预览去抖就该让路
        self._result_seq = self._change_seq
        self._fill_stats(result.get("stats") or {})
        self._fill_waypoints(result.get("waypoints") or [])
        self._fill_segments(result.get("segments") or [])
        self.waypoint_table.scrollToTop()      # 排序后别停在中间，从 GP1 看起
        self._update_exports(result)
        self.stats_holder.show()
        self.result_tabs.show()
        try:
            figure = result_figure(result)
        except Exception as error:                      # noqa: BLE001 - 别让窗口崩
            _LOGGER.exception("结果图渲染失败（运行 %r）", result.get("run_id"))
            figure = placeholder_figure(
                f"结果图渲染失败：{type(error).__name__}: {error}")
            self._report_status(f"结果图渲染失败：{type(error).__name__}: {error}")
        self.last_figure = figure
        if self._plot is not None:
            self._plot.set_figure(figure)
        self._expand_result_area()

    def _expand_result_area(self) -> None:
        """出结果后把结果区撑到 :data:`RESULT_MIN_HEIGHT`（默认 120 px 只够看表头）。

        只在结果区比下限矮时才动尺寸：用户自己拖过分隔条就尊重他的选择；
        图区让出空间但至少留 120 px（不然 3D 图就成了一条缝）。
        """
        splitter = getattr(self, "_viewer_splitter", None)
        if splitter is None:
            return
        sizes = splitter.sizes()
        if len(sizes) != 2 or sizes[1] >= RESULT_MIN_HEIGHT:
            return
        viewer = max(sizes[0] - (RESULT_MIN_HEIGHT - sizes[1]), 120)
        splitter.setSizes([viewer, RESULT_MIN_HEIGHT])

    def _fill_stats(self, stats: dict) -> None:
        """统计标签：成功 n/N、构型切换、ΣdqL2、|J5|/|J3| 最小值、条件数、可行。"""
        solved = int(stats.get("solved") or 0)
        total = int(stats.get("total") or 0)
        self.stat_labels["solved"].setText(f"成功 {solved}/{total}")
        self.stat_labels["solved"].setStyleSheet(
            _GREEN_BOLD if total and solved == total else _RED_BOLD)
        self.stat_labels["switches"].setText(
            f"构型切换 {int(stats.get('switches') or 0)}")
        self.stat_labels["dq_l2"].setText(
            f"ΣdqL2 {format_metric(stats.get('dq_l2_sum'), 2)}")
        self.stat_labels["j5_min"].setText(
            f"|J5|min {format_metric(stats.get('j5_abs_min'))}°")
        self.stat_labels["j3_min"].setText(
            f"|J3|min {format_metric(stats.get('j3_abs_min'))}°")
        self.stat_labels["cond"].setText(
            f"design {format_metric(stats.get('design_cond'), 3)}"
            f" / post_ik {format_metric(stats.get('post_ik_cond'), 3)}")
        feasible = stats.get("feasible")
        if feasible is None:
            self.stat_labels["feasible"].setText("—")
            self.stat_labels["feasible"].setStyleSheet("")
        else:
            self.stat_labels["feasible"].setText("可行" if feasible else "不可行")
            self.stat_labels["feasible"].setStyleSheet(
                _GREEN_BOLD if feasible else _RED_BOLD)

    def _fill_waypoints(self, rows) -> None:
        """路点表：GP / X / Y / Z / q1..q6 / |J3| / |J5| / cfg / ok；失败行红底。"""
        table = self.waypoint_table
        table.setSortingEnabled(False)
        table.clearContents()
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            ok = bool(row.get("ok"))
            position = list(row.get("pos") or [None, None, None])
            joints = list(row.get("q") or [None] * 6)
            joints += [None] * (6 - len(joints))
            config = row.get("config")
            config_text = ("—" if config is None
                           else ",".join(str(int(value)) for value in config))
            gp = row.get("gp")
            cells = [
                (sort_key(gp), "—" if gp is None else f"{int(gp)}", "right"),
                (sort_key(position[0]), format_metric(position[0], 1), "right"),
                (sort_key(position[1]), format_metric(position[1], 1), "right"),
                (sort_key(position[2]), format_metric(position[2], 1), "right"),
            ]
            cells += [(sort_key(joints[axis]), format_metric(joints[axis], 3),
                       "right") for axis in range(6)]
            cells += [
                (sort_key(row.get("j3_abs")),
                 format_metric(row.get("j3_abs"), 3), "right"),
                (sort_key(row.get("j5_abs")),
                 format_metric(row.get("j5_abs"), 3), "right"),
                ((0, config_text), config_text, "left"),
                ((0, 1 if ok else 0), "✓" if ok else "✗", "center"),
            ]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "center": Qt.AlignCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                if not ok:                      # 失败整行标红（表格可排序也不丢）
                    item.setBackground(QtGui.QColor(_FAILED_BG))
                    item.setForeground(QtGui.QColor(_FAILED_FG))
                table.setItem(row_index, column, item)
        table.setSortingEnabled(True)
        table.sortItems(0, Qt.AscendingOrder)
        if len(rows) <= 300:        # 几千行时逐列量宽会把界面卡住
            table.resizeColumnsToContents()

    def _fill_segments(self, segments) -> None:
        """段诊断表：GP 起→止 / |J3|min / |J5|min / 穿零 / 最差点坐标与 t。"""
        table = self.segment_table
        table.setSortingEnabled(False)
        table.clearContents()
        table.setRowCount(len(segments))
        for row_index, segment in enumerate(segments):
            start, stop = segment.get("gp_from"), segment.get("gp_to")
            span = ("—" if start is None or stop is None
                    else f"{int(start)} → {int(stop)}")
            position = segment.get("worst_pos")
            fraction = segment.get("worst_frac")
            if not position:
                worst = "—"
            else:
                worst = ", ".join(f"{float(value):.1f}" for value in position)
                if fraction is not None:
                    worst += f" (t={float(fraction):.2f})"
            cells = [
                ((0, span), span, "center"),
                (sort_key(segment.get("j3_abs_min")),
                 format_metric(segment.get("j3_abs_min"), 3), "right"),
                (sort_key(segment.get("j5_abs_min")),
                 format_metric(segment.get("j5_abs_min"), 3), "right"),
                ((0, 1 if segment.get("crosses") else 0),
                 "是" if segment.get("crosses") else "否", "center"),
                ((0, worst), worst, "left"),
            ]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "center": Qt.AlignCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                if segment.get("crosses"):      # 穿零的段整行标红（最危险）
                    item.setBackground(QtGui.QColor(_FAILED_BG))
                    item.setForeground(QtGui.QColor(_FAILED_FG))
                table.setItem(row_index, column, item)
        table.setSortingEnabled(True)
        if segments:
            # 升序 + “缺值排末尾”（见 sort_key）：裕度最小的段排最前面
            table.sortItems(1, Qt.AscendingOrder)
        table.resizeColumnsToContents()

    def _update_exports(self, result: dict) -> None:
        """按 ``result["artifacts"]`` 启用/禁用三个导出按钮（文件真的在才开）。"""
        artifacts = result.get("artifacts") or {}
        run_dir = result.get("run_dir")
        for key, button in self.export_buttons.items():
            name = artifacts.get(key)
            path = None if (name is None or run_dir is None) \
                else Path(run_dir) / str(name)
            available = path is not None and path.is_file()
            button.setEnabled(available)
            button.setToolTip("" if available else f"本次运行没有 {key} 产物")

    def _on_export(self, key: str) -> None:
        """导出按钮：选目标后**复制**（不移动，``runs/`` 是归档）。"""
        result = self.last_result or {}
        artifacts = result.get("artifacts") or {}
        run_dir = result.get("run_dir")
        name = artifacts.get(key)
        if not name or not run_dir:
            self._report_status(f"本次运行没有 {key} 产物，无法导出")
            return
        source = Path(str(run_dir)) / str(name)
        if not source.is_file():
            # 源文件可能在 `runs/` 的保留策略里被清掉了（KEEP_RUNS）：状态栏和
            # 结果区状态行说同一句话，不要只在一处提示。
            message = (f"源文件不存在：{source}"
                       "（可能已被 runs/ 保留策略清掉，或手删了运行目录）")
            self._set_status(message, level="error")
            self._report_status(message)
            return
        filters = next((item[2] for item in EXPORTS if item[0] == key), "")
        # 默认文件名带运行号：连跑两次解算时不会互相覆盖（run_id 就是 runs/<id>）
        run_id = str(result.get("run_id") or "")
        default_name = f"{run_id}-{name}" if run_id else str(name)
        target, _selected = QtWidgets.QFileDialog.getSaveFileName(
            self, f"导出 {name}", str(Path.home() / default_name), filters)
        if not target:                      # 用户取消
            return
        target_path = Path(str(target))
        if not target_path.suffix:          # 手打完删了扩展名：按源文件补回来
            target_path = target_path.with_suffix(source.suffix)
        try:
            shutil.copy2(source, target_path)
        except OSError as error:
            self._set_status(f"导出失败：{type(error).__name__}: {error}",
                             level="error")
            self._report_status(f"导出失败：{type(error).__name__}: {error}")
            return
        self._report_status(f"已导出 {name} → {target_path}")

    def _reset_result_area(self) -> None:
        """起新任务前清空结果区（上一次的表、图、日志不留在界面上冒充新结果）。"""
        self.last_result = None
        self.log_edit.clear()
        for table in (self.waypoint_table, self.segment_table):
            table.setSortingEnabled(False)
            table.clearContents()
            table.setRowCount(0)
            table.setSortingEnabled(True)
        for label in self.stat_labels.values():
            label.setText("—")
            label.setStyleSheet("")
        self.stats_holder.hide()        # 上一轮的统计不留在界面上冒充新结果
        for button in self.export_buttons.values():
            button.setEnabled(False)
            button.setToolTip("")
        self.status_label.clear()
        self.status_label.hide()

    def _set_status(self, text: str, level: str = "info") -> None:
        """结果区状态行：``ok`` 绿 / ``error`` 红 / ``muted`` 灰 / ``info`` 正常。"""
        message = str(text or "")
        self.status_label.setText(message)
        self.status_label.setStyleSheet(_STATUS_STYLES.get(level, ""))
        self.status_label.setVisible(bool(message.strip()))

    def shutdown(self) -> None:
        """窗口关闭时收尾：停去抖/合并定时器并注销任务订阅（不取消任务）。"""
        self._debounce.stop()
        self.bridge.shutdown()

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
