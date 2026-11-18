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

from . import page_layout, paths
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
#: 与源缓冲（``jobs.LOG_LIMIT``）相等：源头留多少，控件就能显示多少。
LOG_WIDGET_LIMIT = 500

#: 段诊断表最多显示多少行（S6 L2）：965 段时全量塞进去光建单元格就要几十
#: 毫秒、再加上逐列量宽会明显卡顿。
SEGMENT_ROW_LIMIT = 500

#: 行数不超过这个才逐列量宽：``resizeColumnsToContents`` 是 O(行×列) 的。
SEGMENT_AUTOFIT_LIMIT = 200

#: 超过 :data:`SEGMENT_AUTOFIT_LIMIT` 时的固定列宽（px，顺序同
#: :data:`SEGMENT_HEADERS`）。
SEGMENT_COLUMN_WIDTHS = (110, 90, 90, 60, 210)

#: 结果页签的最小高度（px）：``QTabWidget`` 只按**当前页**报最小尺寸，不定死
#: 的话切到「段诊断」时结果区会把表裁掉（见 :mod:`laser_desktop.page_layout`
#: 顶部的说明）。240 ≈ 页签条 + 表 200 + 少量边距。
TABS_MIN_HEIGHT = 246

#: 省略提示（``JobBridge`` 在源缓冲丢行时插的一行）的行首。
OMISSION_PREFIX = "…已省略"

#: 出结果后结果区至少占多高（像素）：表 200 + 页签条 + 统计 + 状态/进度/导出。
#: 与 :data:`laser_desktop.page_layout.RESULT_MIN_HEIGHT` 同一份（三页共用一个数）。
RESULT_MIN_HEIGHT = page_layout.RESULT_MIN_HEIGHT

#: 数值框的最小宽度下限（px）：真正的宽度取 ``max(这个数, sizeHint)``，
#: 见 :func:`_fit_width`（带 `` mm`` 后缀的浮点框实测 sizeHint=131）。
SPIN_MIN_WIDTH = 88

#: 整数框的最小宽度下限（px，同上）。
INT_SPIN_MIN_WIDTH = 76

#: 左侧表单列的宽度上限（px）：再宽就是从图和结果表那里硬拿。
#: 下限不写死：由 :func:`_form_scroll` 量内容的 ``minimumSizeHint``（不裁控件）。
FORM_MAX_WIDTH = 560

#: 进度条的量程：分数（0~1）乘它取整，比 0~100 细一个量级，又不用浮点。
PROGRESS_STEPS = 1000

#: 进度条的高度上限（px）：默认 33 px 太高，而结果区每多 1 px 都会顶高窗口。
PROGRESS_BAR_HEIGHT = 22

#: 进度条的固定宽度（px）：S6c 起它和状态文字同占一行，“阶段 2/3 · 解算 IK”
#: 这个宽度里不会被截断；状态文字拿剩下的宽度。
PROGRESS_BAR_WIDTH = 200

#: 导出按钮的高度上限（px）：同上，结果区最后一行也要省着用。
EXPORT_BUTTON_HEIGHT = 26

#: 出结果后希望一屏露出的行数（默认窗口 1600×950）：生成页的 8 个点要全见。
#: 实际拿到的行数还受结果区最小高度与其它段落限制（见 :func:`expand_result_area`）。
RESULT_ROW_TARGET = 8

#: 结果区“多了多少才算多”的阈值（px）：低于它的不折腾（避免每个像素都重排）。
SHRINK_SLACK = 8

#: 出结果后从预览卡上扣高度的下限（px）：再矮就只剩个标题，读不了。
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
#: 导出按钮：``(artifacts 键, 按钮文案, 文件对话框过滤器)``。
#: 按钮只写文件名（"导出"由行首的说明标签承担）：四个“导出 xxx.json”并排会把
#: 结果区的最小宽度顶到 570 px，1180 宽的窗口装不下（L1 实测）。
EXPORTS = (("scan_json", "scan.json", "JSON (*.json);;所有文件 (*)"),
           ("joints", "joints.txt", "文本 (*.txt);;所有文件 (*)"),
           ("plot", "plot.png", "PNG 图片 (*.png);;所有文件 (*)"))

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
        customdata=[[index + 1] for index in range(len(positions))],
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
def _export_button(title: str) -> QtWidgets.QPushButton:
    """导出按钮：统一压到 :data:`EXPORT_BUTTON_HEIGHT`（结果区最后一行要省着用）。"""
    button = QtWidgets.QPushButton(title)
    button.setMaximumHeight(EXPORT_BUTTON_HEIGHT)
    return button


def _compact_result_layout(layout) -> None:
    """结果区的布局缩紧（边距 6/6/6/4、行距 3）。

    结果区的最小高度直接变成窗口的最小高度：三段（预览 50 + 图 200 + 结果）
    加在一起必须装得下 1280×800 的窗口。默认的 9 px 边距与 6 px 行距在五六个
    控件上就是 ~20 px，刚好把窗口顶高就能把表的行挤掉。

    Args:
        layout: 结果区的 ``QVBoxLayout``。
    """
    layout.setContentsMargins(6, 6, 6, 4)
    layout.setSpacing(2)


def wanted_result_height(result_area, table,
                         rows: int = RESULT_ROW_TARGET) -> int:
    """要露出 ``rows`` 行，结果区得多高（按当前字体/样式实测，不拍脑袋）。

    算法：表的视口高度 = 行高 × 行数（``sizeHintForRow``），再加上表自己的
    "壳"（表头 + 边框 + 横向滚动条 = ``height - viewport.height()``）与结果区
    里除表以外的东西（页签条 / 统计 / 状态 / 进度 / 导出 / 标题）。两者都是
    现场量的，三页各不相同的部分（ISO 只有 5 个点、修复页多一行运行目录）
    不需要在这里写分支。

    Args:
        result_area: 结果区 ``QGroupBox``（:attr:`GeneratePage.result_area`）。
        table: 结果表（生成/修复 = 路点表，ISO = 点表）。
        rows: 想露出的行数（不超过表里实际行数）。

    Returns:
        int: 期望的结果区高度（px）。
    """
    if result_area is None or table is None:
        return RESULT_MIN_HEIGHT
    try:
        row_count = max(0, table.rowCount())
        section = int(table.verticalHeader().sectionSize(0) or 0)
        table_height = int(table.height())
        viewport_height = int(table.viewport().height())
        area_height = int(result_area.height())
    except RuntimeError:                   # 页面已被删：回到静态下限
        return RESULT_MIN_HEIGHT
    count = min(int(rows), row_count)
    # 行高用**表头段高**（``sizeHintForRow`` 报 26、实际行是 30 px，差 4 px
    # 足以把第 8 行切掉半截——实测就是靠多露 2 px 才从 7 行变 8 行）。
    row_height = section
    if row_height <= 0 and row_count:
        row_height = int(table.sizeHintForRow(0) or 0)
    if row_height <= 0:
        row_height = 30
    # 行间还有 1 px 网格线，最后再留 4 px 余量（字体换一下不至于又少一行）
    viewport = row_height * count + max(0, count - 1) + 4
    table_shell = max(0, table_height - viewport_height)
    wanted_table = viewport + table_shell
    extra = max(0, area_height - table_height)
    return int(wanted_table + extra)


def expand_result_area(page) -> None:
    """出结果后把结果区撑到"够看 :data:`RESULT_ROW_TARGET` 行"（三页共用）。

    只往大的方向动：用户自己拖大了就尊重；比例本身由
    :class:`page_layout.RightSplitter` 管。高度先从图区拿（但不低于
    :data:`page_layout.PLOT_MIN_HEIGHT`），不够再从预览卡上扣（保留
    :func:`page_layout.preview_keep_height`，即“放得下内容”的下限）——顺序是
    "预览卡 → 图 → 结果"的优先级反过来。

    下限按**内容**而不是常数算：1600×950 的真实窗口（含菜单栏/状态栏）只给
    分割器 ~892 px，8 行表 + 图 260 之后预览卡只剩 120–130 px；卡片内容本来只
    有 ~115 px，压到 115 仍然一眼全见，而写死 150 就会把 8 行表挤成 7 行。

    Args:
        page: 生成 / 修复 / ISO 页（要 ``_viewer_splitter``、``result_area``、
            ``waypoint_table`` 或 ``point_table``）。
    """
    splitter = getattr(page, "_viewer_splitter", None)
    if splitter is None:
        return
    table = getattr(page, "waypoint_table", None) or getattr(
        page, "point_table", None)
    try:
        target = max(RESULT_MIN_HEIGHT, wanted_result_height(
            getattr(page, "result_area", None), table))
        sizes = list(splitter.sizes())
    except RuntimeError:                       # 页面/控件已被删
        return
    if len(sizes) != 3:
        return
    if sizes[2] >= target:
        shrink_result_area(splitter, sizes, target)
        return
    keep = page_layout.preview_keep_height(
        getattr(splitter, "preview_card", None))
    # 图区这一档能守 260 还是只能守 200：1280×800 下是 200（见 plot_floor）
    plot_floor = page_layout.PLOT_MIN_HEIGHT
    floor_of = getattr(splitter, "plot_floor", None)
    if callable(floor_of):
        plot_floor = int(floor_of())
    want = target - sizes[2]
    take = min(want, max(0, sizes[1] - plot_floor))
    sizes[1] -= take
    sizes[2] += take
    left = want - take
    if left > 0:
        give = min(left, max(0, sizes[0] - keep))
        sizes[0] -= give
        sizes[2] += give
        left = want - take - give
    if left > 0 and sizes[1] >= plot_floor:
        # 图区的设计下限已经守住了，表还差几行：预览卡可以再让一点（不低
        # ``PREVIEW_MIN_HEIGHT``）。
        give = min(left, max(0, sizes[0] - page_layout.PREVIEW_MIN_HEIGHT))
        sizes[0] -= give
        sizes[2] += give
    try:
        splitter._apply_sizes(sizes)           # 不当作"用户拖过"写进设置
    except RuntimeError:                       # pragma: no cover - 已删
        return
    _recheck_result_area_later(page)


def shrink_result_area(splitter, sizes, target: int) -> bool:
    """结果区比它需要的高时，把多出来的还给图区（最多一次收干净）。

    为什么需要“缩”：出结果那一刻量到的 ``extra``（表以外占多少高）是**旧**
    几何——计划页的统计卡刚 ``show()``、页签刚切，表还没被挤小，于是目标偏高；
    而 :func:`expand_result_area` 只管涨不管降，多拿的 20–30 px 就再也回不去了
    （实测修复页：结果区 532，实际 509 就够 8 行，白亏 23 px 预览卡）。

    安全性：

    * 目标只依赖行高与 ``extra``，而 ``extra = 结果区 - 表`` 在缩放时不变（表
      跟着一起缩），所以收完就停，不会来回抽；
    * 用户亲手拖过（``splitter.user_moved``）就**不**动——那是他自己要的大小。

    Args:
        splitter: :class:`page_layout.RightSplitter`。
        sizes: 当前三段尺寸（会被本函数修改）。
        target: 结果区真正需要的高度。

    Returns:
        bool: 真的收过才 ``True``。
    """
    if getattr(splitter, "user_moved", False):
        return False
    surplus = int(sizes[2]) - int(target)
    if surplus < SHRINK_SLACK:
        return False
    sizes[2] -= surplus
    # 先把预览卡补回它的首开高度（出结果那一刻它被多拿走的），剩下的给图区。
    # 不补回来的话，预览卡会永远停在被挤扁那一刻的高度（实测修复页 100 px，
    # 而五项数值要 115 px，最后一项就挂在滚动区外面）。
    to_preview = min(surplus,
                     max(0, page_layout.preview_default_height(
                         getattr(splitter, "preview_card", None)) - sizes[0]))
    sizes[0] += to_preview
    sizes[1] += surplus - to_preview
    try:
        splitter._apply_sizes(sizes)          # 不当作"用户拖过"写进设置
    except RuntimeError:                      # pragma: no cover - 已删
        return False
    return True


def _recheck_result_area_later(page) -> None:
    """下一轮事件循环再量一次（页签/表那时才拿到真几何）。

    ``expand_result_area`` 用的是**实际像素**（表高、结果区高），刚出结果那一
    刻当前页签可能还是「日志」，路点表的几何是旧的——那样算出来的目标会偏小，
    屏幕上就只能露 5 行。只往大的方向重算，幂等。
    """
    if getattr(page, "_expand_pending", False):
        return
    page._expand_pending = True

    def _again() -> None:
        page._expand_pending = False
        try:
            expand_result_area(page)
        except RuntimeError:               # 页面/表已被删（测试收尾、关窗）
            return

    QtCore.QTimer.singleShot(0, _again)


def _fit_width(box, floor: int):
    """数值框的最小宽度 = max(下限, 自己的 ``sizeHint``)：表单里不会被压到看不全。

    现场症状（S6 截图）：修复页的 ``Y 下界`` 显示成 ``500.0 mr``，``Z 下界``
    半截被切。原因是 ``setMinimumWidth(88)`` 比 ``sizeHint``（带 `` mm`` 后缀
    实测 131 px）小——``QFormLayout`` 的列不够宽时就把控件压到 88，文字被
    内部行编辑裁掉。钉到 ``sizeHint`` 之后，布局要么给够宽度，要么把整个表单
    列撑宽（后者由 :func:`_form_scroll` 的 ``minimumWidth`` 兼底）。

    Args:
        box: ``QSpinBox`` / ``QDoubleSpinBox``。
        floor: 宽度下限（px）。

    Returns:
        同一个控件（方便链式调用）。
    """
    box.setMinimumWidth(max(int(floor), int(box.sizeHint().width())))
    return box


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
    return _fit_width(box, SPIN_MIN_WIDTH)


def _int_spin(minimum: int, maximum: int, value: int,
              suffix: str = "") -> QtWidgets.QSpinBox:
    """整数输入（格点数 / 点数 / 采样数）。"""
    box = QtWidgets.QSpinBox()
    box.setRange(minimum, maximum)
    box.setValue(int(value))
    if suffix:
        box.setSuffix(suffix)
    box.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    return _fit_width(box, INT_SPIN_MIN_WIDTH)


def _form_scroll(content, maximum_width: int) -> QtWidgets.QScrollArea:
    """把表单内容装进纵向滚动区，宽度不小于内容真正需要的最小宽度。

    为什么不能拍一个数字：三页的表单行宽差很多（修复页的「现场信息」有 6 个
    关节角 + 「删除本组」，是四页里最宽的）。写死一个 455 时它比内容窄，于是
    底部出现横向滚动条，右边的数值框/按钮被裁掉一半（S6 截图里的
    ``500.0 mr`` 与只露一半的「删除本组」）。这里量 ``minimumSizeHint``：
    **控件的最小宽度之和**才是"不裁"的边界，再补上纵向滚动条与边框。

    Args:
        content: 表单内容控件（已摆好布局）。
        maximum_width: 宽度上限（px）：再宽就是浪费图的横向空间。

    Returns:
        ``QScrollArea``（纵向可滚，横向不允许）。
    """
    scroll = QtWidgets.QScrollArea()
    scroll.setWidget(content)
    scroll.setWidgetResizable(True)
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    needed = (content.minimumSizeHint().width()
              + scroll.verticalScrollBar().sizeHint().width()
              + 2 * scroll.frameWidth() + 4)
    scroll.setMinimumWidth(min(int(needed), int(maximum_width)))
    scroll.setMaximumWidth(int(maximum_width))
    return scroll


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


def _caption(text: str, lines: int = 2) -> QtWidgets.QLabel:
    """说明性小字（灰）：自动换行，高度按**文字实际占几行**算（最多 ``lines`` 行）。

    早先这里把高度固定成 ``lines`` 行（默认 2 行 = 44 px）：这种标签大多只有
    一句话，每行都多出一行空白——「机器人模型」组里连着两条，看上去就是
    "行距被拉散、排版坏了"（S6b 截图）。现在交给
    :class:`laser_desktop.page_layout.HintLabel`：宽度一变就重算，**高度 = 内容
    高度**，但不超 ``lines`` 行（超出的部分全文一律放 tooltip，与
    ``page_layout.fit_message`` 同一口径）。

    Args:
        text: 文案。
        lines: 高度上限（行数）。

    Returns:
        ``QLabel``（灰色小字）。
    """
    label = page_layout.HintLabel(text, max_lines=lines)
    label.setStyleSheet("color:#6b7280")
    label.setToolTip(text)
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


class ResultTable(QtWidgets.QTableWidget):
    """结果表：``minimumSizeHint`` 不跟着列宽涨（否则会把整个窗口顶宽）。

    ``QTableWidget`` 的最小宽度会算上所有列的宽度：14 列的路点表在
    ``resizeColumnsToContents`` 之后能报到 1000+ px，于是 QMainWindow 的
    最小宽度变成 1400+（实测 1422），1180 的窗口根本撑不到。列宽是显示细节，
    该由表格自己的横向滚动条解决，不该变成窗口的下限。
    """

    def minimumSizeHint(self):                   # noqa: N802 - Qt 回调命名
        hint = super().minimumSizeHint()
        return QtCore.QSize(160, hint.height())


def make_table(headers) -> QtWidgets.QTableWidget:
    """只读、整行选中、可排序的表格（填数据前记得先关排序）。"""
    table = ResultTable(0, len(headers))
    table.setHorizontalHeaderLabels(list(headers))
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    table.setAlternatingRowColors(True)
    table.verticalHeader().setVisible(False)
    table.setSortingEnabled(True)
    table.horizontalHeader().setStretchLastSection(True)
    return table


def fill_stat_labels(labels: dict, stats: dict) -> None:
    """统计标签：成功 n/N、构型切换、ΣdqL2、|J5|/|J3| 最小值、条件数、可行。

    这是模块级函数（不是页面方法）**为了复用**：修复页的「计划」页签要把
    ``core.run_repair`` 的 ``stats`` 铺成同一排骨架标签，照同一个口径格式化
    （键见 :data:`STAT_KEYS`）。

    Args:
        labels: ``{STAT_KEYS 里的键: QLabel}``。
        stats: ``core.run_scan`` / ``core.run_repair`` 的 ``stats``。
    """
    solved = int(stats.get("solved") or 0)
    total = int(stats.get("total") or 0)
    labels["solved"].setText(f"成功 {solved}/{total}")
    labels["solved"].setStyleSheet(
        _GREEN_BOLD if total and solved == total else _RED_BOLD)
    labels["switches"].setText(f"构型切换 {int(stats.get('switches') or 0)}")
    labels["dq_l2"].setText(f"ΣdqL2 {format_metric(stats.get('dq_l2_sum'), 2)}")
    labels["j5_min"].setText(
        f"|J5|min {format_metric(stats.get('j5_abs_min'))}°")
    labels["j3_min"].setText(
        f"|J3|min {format_metric(stats.get('j3_abs_min'))}°")
    labels["cond"].setText(
        f"design {format_metric(stats.get('design_cond'), 3)}"
        f" / post_ik {format_metric(stats.get('post_ik_cond'), 3)}")
    feasible = stats.get("feasible")
    if feasible is None:
        labels["feasible"].setText("—")
        labels["feasible"].setStyleSheet("")
    else:
        labels["feasible"].setText("可行" if feasible else "不可行")
        labels["feasible"].setStyleSheet(
            _GREEN_BOLD if feasible else _RED_BOLD)
    # 文字一变，每张卡的宽度也变了：让容器重排一次（行数可能少一行）
    holder = labels["solved"].parentWidget() if labels.get("solved") else None
    relayout = getattr(holder, "relayout", None)
    if callable(relayout):
        relayout()


def fill_waypoint_table(table, rows) -> None:
    """路点表：GP / X / Y / Z / q1..q6 / |J3| / |J5| / cfg / ok；失败行红底。

    Args:
        table: 列头为 :data:`WAYPOINT_HEADERS` 的 ``QTableWidget``。
        rows: ``result["waypoints"]``。
    """
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


def fill_segment_table(table, segments, caption=None,
                       caption_text: str = "") -> int:
    """段诊断表：GP 起→止 / |J3|min / |J5|min / 穿零 / 最差点坐标与 t。

    S6 L2：段数过多时只显示前 :data:`SEGMENT_ROW_LIMIT` 行，并由调用方给的
    ``caption`` 标签说明“共 N 段，已显示前 M 段”——不静默截断。
    逐列量宽只在行数 <= :data:`SEGMENT_AUTOFIT_LIMIT` 时做。

    Args:
        table: 列头为 :data:`SEGMENT_HEADERS` 的 ``QTableWidget``。
        segments: ``result["segments"]``。
        caption: 可选的说明标签；给了就在这里更新截断提示。
        caption_text: 没有截断时的说明文字（各页自己的静态说明）。

    Returns:
        int: 实际显示的行数。
    """
    total = len(segments or [])
    shown = list(segments or [])[:SEGMENT_ROW_LIMIT]
    table.setSortingEnabled(False)
    table.clearContents()
    table.setRowCount(len(shown))
    for row_index, segment in enumerate(shown):
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
    if shown:
        # 升序 + “缺值排末尾”（见 sort_key）：裕度最小的段排最前面
        table.sortItems(1, Qt.AscendingOrder)
    if len(shown) <= SEGMENT_AUTOFIT_LIMIT:
        table.resizeColumnsToContents()
    else:
        # 几千行时逐列量宽会把界面卡住：用固定列宽（表头已够读懂）
        for column, width in enumerate(SEGMENT_COLUMN_WIDTHS):
            table.setColumnWidth(column, width)
    if caption is not None:
        if total > len(shown):
            text = (f"共 {total} 段，已显示前 {len(shown)} 段（按 |J3|min 排序只在"
                    f"这 {len(shown)} 段里做）")
        else:
            text = caption_text
        caption.setText(text)
        caption.setToolTip(text)
    return len(shown)


def export_artifact(result: dict, key: str, specs, parent=None):
    """把 ``result["artifacts"][key]`` 复制到用户选的位置（选完就复制，不移动）。

    ``runs/`` 是归档，导出只做复制；默认文件名带运行号，连跑两次不会互相覆盖。
    这是节点级函数（不是页面方法）为了让修复页复用同一套“缺产物 / 源文件被清 /
    手打漏扩展名 / 用户取消”的分支，而不是各页各写一份再漂移。

    Args:
        result: ``core.run_scan`` / ``core.run_repair`` 的返回值。
        key: ``artifacts`` 里的键（``scan_json`` / ``joints`` / ``plot`` / ``repair``）。
        specs: ``((键, 按钮文案, 文件对话框过滤器), ...)``，用于取过滤器。
        parent: ``QFileDialog`` 的父控件。

    Returns:
        ``(level, message)``：``message`` 是要显示的原文；``level`` 为 ``None``
        表示只是“正常提示”（只写状态栏），``"error"`` 表示结果区也要标红。
        用户取消返回 ``(None, "")``。
    """
    artifacts = (result or {}).get("artifacts") or {}
    run_dir = (result or {}).get("run_dir")
    name = artifacts.get(key)
    if not name or not run_dir:
        return None, f"本次运行没有 {key} 产物，无法导出"
    source = Path(str(run_dir)) / str(name)
    if not source.is_file():
        # 源文件可能在 `runs/` 的保留策略里被清掉了（KEEP_RUNS）
        return "error", (f"源文件不存在：{source}"
                          "（可能已被 runs/ 保留策略清掉，或手删了运行目录）")
    filters = next((item[2] for item in specs if item[0] == key), "")
    run_id = str((result or {}).get("run_id") or "")
    default_name = f"{run_id}-{name}" if run_id else str(name)
    target, _selected = QtWidgets.QFileDialog.getSaveFileName(
        parent, f"导出 {name}", str(Path.home() / default_name), filters)
    if not target:                          # 用户取消
        return None, ""
    target_path = Path(str(target))
    if not target_path.suffix:              # 手打完删了扩展名：按源文件补回来
        target_path = target_path.with_suffix(source.suffix)
    try:
        shutil.copy2(source, target_path)
    except OSError as error:
        return "error", f"导出失败：{type(error).__name__}: {error}"
    return None, f"已导出 {name} → {target_path}"


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class GeneratePage(QtWidgets.QWidget):
    """生成页：左侧可滚动表单 + 右侧三段（预览卡 / 图 / 结果区）。

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

    #: 状态栏里的页面前缀：三页共用一个窗口，不说清是哪一页写的就会张冠李戴
    #: （S6 评审 R2：隐藏页发的一句话把当前页的状态栏改了）。
    STATUS_PREFIX = "生成"

    def __init__(self, plot=None, robots=None, parent=None, bridge=None,
                 settings=None) -> None:
        """建页面。

        Args:
            plot: ``PlotView``（或任何有 ``set_figure`` 的对象）；``None`` 表示
                不挂 WebEngine（纯逻辑测试用，图仍然会算好放在 ``last_figure``）。
            robots: :func:`laser_web.core.list_robots` 的返回值；``None`` 时现查。
            parent: 父控件。
            bridge: 现成的 :class:`JobBridge`（测试注入假登记表用）；``None``
                时按 :func:`laser_desktop.paths.runs_root` 新建一个。
            settings: ``QSettings``（右栏三段分割的比例）；``None`` 时用应用默认。
        """
        super().__init__(parent)
        self._plot = plot
        #: 只有**当前页**才往共享的 PlotView 推图（主窗口切页时改这个）。
        #: 三页共用一个视图，隐藏页的去抖/预览如果照推，会把当前页的
        #: 结果图换成自己的预览图（S6 联调实测）。
        self._active = True
        self._settings = settings
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
        outer.addWidget(self._build_left_column(), 0)
        outer.addWidget(self._build_right_column(), 1)
        page_layout.addWidget(body, 1)

        # 信号连在控件建完之后：槽里直接碰控件（都在 GUI 线程，见 job_bridge）
        self.bridge.progress.connect(self._on_progress)
        self.bridge.log_line.connect(self._on_log_line)
        self.bridge.finished.connect(self._on_finished)
        self._connect_plot_link()
        self.refresh_preview()
        self._resume_running_job()

    def _build_left_column(self) -> QtWidgets.QWidget:
        """左栏：表单滚动区 + **主操作行**（S6c）。

        主操作按钮（「生成并解算」）必须在**滚动区之外**：S6b 把它放在预览卡片
        里，卡片是右栏第一段、窗口一变就被压矮，按钮跟着滚出视野。现在它和
        表单滚动区平级挂在这列底部（垂直策略 ``Fixed``），1600×950 与
        1280×800 下都按 ``sizeHint`` 拿到整高。

        为什么不放在右栏（比如紧贴预览卡下面）：右栏每多一行 39 px 都是从
        "图 260 + 表 8 行" 里硬拿的。放在左栏底部不占分割器的高度——1600×950
        下预览卡、图、表三段都是紧的（实测 892 px 可用：图 260 + 结果 490 +
        预览 130），多拿走 39 px 就得从第 8 行表或图区里扣。
        """
        column = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._build_form(), 1)
        layout.addWidget(self._build_actions(), 0)
        return column

    def _build_actions(self) -> QtWidgets.QWidget:
        """主操作行：「生成并解算」（不设 ``maximumHeight``，否则 height < sizeHint）。"""
        self.generate_button = QtWidgets.QPushButton("生成并解算")
        self.generate_button.setToolTip(
            "起一条后台任务：生成路点 → 解算 IK → 段检查 → 条件数 → 写 runs/<id>/")
        self.generate_button.clicked.connect(self._on_generate_clicked)
        self.action_row = page_layout.build_action_row(self.generate_button)
        return self.action_row

    def _build_right_column(self) -> QtWidgets.QWidget:
        """右栏三段：预览卡 / 图 / 结果区（``QSplitter``，比例可拖可记忆）。

        L1：之前是“预览卡固定高度 + 图/结果两分”，小窗口下结果区被压成表头
        + 一行。现在三段都在分割里，首开按 40/35/25 分配，结果表又定了最小高度
        （5 行）。**出结果之后**结果区的最小高度（表 + 页签 + 统计 + 状态/导出）
        会盖过 25% 的份额——这是"一屏 5 行"的代价，比例仍可拖、也会被记住。
        图本身由主窗口在切页时搬进 :attr:`plot_slot`（三页共用一个 ``PlotView``）。
        """
        self.plot_slot = page_layout.PlotSlot()
        if (self._plot is not None
                and not getattr(self._plot, "SHARED_ACROSS_PAGES", False)):
            self.plot_slot.adopt(self._plot)     # 单页/测试：图就是它自己
        splitter = page_layout.RightSplitter(
            "generate", self._build_preview_card(), self.plot_slot,
            self._build_result_area(), settings=self._settings)
        self._viewer_splitter = splitter
        self._right_splitter = splitter
        return splitter

    # ------------------------------------------------------------------ 表单
    def _build_form(self) -> QtWidgets.QScrollArea:
        """左侧表单：六个分组塞进 ``QScrollArea``。

        六组控件全部由 :mod:`laser_desktop.form_widgets` 建（与修复页共用同一
        份实现，S4 评审第 1 条：之前两页各存一份副本，改一处忘了另一处就分叉）。
        本方法只负责“摆”和把页面要用的属性接回来。
        """
        from . import form_widgets      # 延迟导入：本模块是 form_widgets 的依赖
        defaults = form_defaults()
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setSpacing(8)

        robot = form_widgets.build_robot_group(
            self.fields, self._robots, self._on_change, defaults["robot"])
        self.robot_path_label = robot["path_label"]
        self.skipped_label = robot["skipped_label"]
        self.robot_group = robot["group"]     # 冒烟/单测要量它高没高（P3）
        layout.addWidget(robot["group"])

        domain = form_widgets.build_domain_group(self.fields, defaults,
                                                 self._on_change)
        self.domain_stack = domain["stack"]
        domain["points"].toggled.connect(self._on_domain_switch)
        domain["axes"].toggled.connect(self._on_domain_switch)
        layout.addWidget(domain["group"])

        grid = form_widgets.build_grid_group(self.fields, defaults,
                                             self._on_change)
        self.grid_stack = grid["stack"]
        self.total_value = grid["total_value"]
        self.total_hint = grid["total_hint"]
        self.grid_caption = grid["caption"]
        grid["count"].toggled.connect(self._on_grid_switch)
        grid["explicit"].toggled.connect(self._on_grid_switch)
        layout.addWidget(grid["group"])

        layout.addWidget(form_widgets.build_pose_group(self.fields, defaults,
                                                       self._on_change))
        layout.addWidget(form_widgets.build_perturb_group(self.fields, defaults,
                                                          self._on_change))
        layout.addWidget(form_widgets.build_guard_group(self.fields, defaults,
                                                        self._on_change))
        layout.addStretch(1)
        scroll = _form_scroll(content, FORM_MAX_WIDTH)
        self.form_scroll = scroll             # 冒烟/单测要查横滚条（P4）
        return scroll

    def _build_preview_card(self) -> QtWidgets.QGroupBox:
        """预览卡片：体积 / 格点 / 总点数 / 设计条件数 / 欧拉约定 + 报错原文。

        S6c 两处收口：

        * **主操作按钮搬走了**（现在在左栏底部，见 :meth:`_build_left_column`）。
          卡片里放着它的时候，卡片被压矮就两道矛盾：数值滚出去、按钮被切半截。
        * 五项数值改成**按需换行**（:class:`page_layout.MetricGrid`）：五行
          ``QFormLayout`` 要 141 px，两行只要 ~45 px——「设计条件数」是判断
          可行/不可行的依据，1600×950 下必须一眼看到（不用滚）。
        """
        group = QtWidgets.QGroupBox("预览（不跑 IK）")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(3)
        layout.setContentsMargins(4, 2, 4, 2)

        self.metric_grid = page_layout.MetricGrid(
            self.metric_labels, page_layout.PREVIEW_METRICS)
        layout.addWidget(self.metric_grid)

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

        # 说明文字走 tooltip，不占行：1600×950 下右栏只能给预览卡 ~150 px
        # （图 260 + 8 行表 ~500 已占了 772）。多一行说明就是「欧拉约定」
        # 被顶出卡片、要多滚一下——卡片里只允许放**要看的东西**。
        group.setToolTip(
            "改任何一项都会重算上面这些数，并重画右侧路点图；"
            "点「生成并解算」才真的跑 IK（在后台线程，可取消）。")
        return group

    def _build_result_area(self) -> QtWidgets.QGroupBox:
        """结果区：状态行 + 进度/取消 + 统计标签 + 三页签（路点 / 段诊断 / 日志） + 导出。"""
        group = QtWidgets.QGroupBox("结果")
        self.result_area = group        # expand_result_area 要量"表以外占多高"
        layout = QtWidgets.QVBoxLayout(group)
        # 紧凑：结果区的最小高度直接决定窗口最小高度（1280×800 要装得下三段）
        _compact_result_layout(layout)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        # 长文案（"完成：成功 8/8 点，运行 … → /home/…"）自动换行后会把**最小**
        # 高度顶起来，窗口就再也缩不回最小尺寸（S6 冒烟实测 900）。垂直方向
        # 让布局随便给（Ignored = 有多少用多少、最小 0），有空间时照样整段显示。
        # 垂直方向 Ignored：不参与窗口的最小高度；高度由 ``fit_message`` 按
        # 文案的自然高度钉住（上限 4 行），所以既不会贪心吃满表格的高度，
        # 也不会把窗口的最小高度顶高。
        self.status_label.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                        QtWidgets.QSizePolicy.Ignored)
        # 进度条本身的格式文本就是"阶段 索引/总数 · 状态"（规格要求的展示位）
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, PROGRESS_STEPS)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("空闲")
        # 进度条固定矮一点：它连同取消按钮占 33 px，而结果区的最小高度直接
        # 决定窗口最小高度（1280×800 要装得下三段）。
        self.progress_bar.setMaximumHeight(PROGRESS_BAR_HEIGHT)
        self.progress_bar.setFixedWidth(PROGRESS_BAR_WIDTH)
        self.cancel_button = QtWidgets.QPushButton("取消")
        self.cancel_button.setToolTip("只在阶段边界生效：当前阶段跑完就停，不会硬打断 IK")
        self.cancel_button.clicked.connect(self._on_cancel_clicked)
        self.cancel_button.hide()
        # **状态文字 + 进度条 + 取消**同占一行（S6c）：它们本来就是"一句状态 +
        # 一条进度"，各占一行等于白送结果区 22 px——而真实窗口里分割器只有
        # ~892 px，8 行表 + 图 260 之后，22 px 就是预览卡能不能整段显示的差别。
        status_row = QtWidgets.QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.setSpacing(6)
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.progress_bar, 0)
        status_row.addWidget(self.cancel_button, 0)
        layout.addLayout(status_row)
        self.status_label.hide()

        # 统计卡：按需换行（1600×950 两行、1280×800 三行，都不切字）
        self.stats_holder = page_layout.StatsGrid(self.stat_labels, STAT_KEYS)
        self.stats_holder.hide()        # 没结果时不占高度（预览时图可以高一点）
        layout.addWidget(self.stats_holder)

        self.result_tabs = QtWidgets.QTabWidget()
        self.waypoints_page = self._build_waypoints_page()
        self.segments_page = self._build_segments_page()
        self.log_page = self._build_log_page()
        self.result_tabs.setMinimumHeight(TABS_MIN_HEIGHT)
        self.result_tabs.addTab(self.waypoints_page, "路点")
        self.result_tabs.addTab(self.segments_page, "段诊断")
        self.result_tabs.addTab(self.log_page, "日志")
        # 切页签会改变「结果区最少要多高」（QTabWidget 只算当前页）：切到路点表
        # 就把结果区撑到够看 8 行（出结果那一刻当前页签可能还是日志）。
        self.result_tabs.currentChanged.connect(
            lambda _index: self._expand_result_area())
        self.result_tabs.hide()         # 起任务后才展开（预览时把高度让给图）
        layout.addWidget(self.result_tabs, 1)

        export_row = QtWidgets.QHBoxLayout()
        export_row.setContentsMargins(0, 0, 0, 0)
        self.export_caption = _caption("导出：", lines=1)
        self.export_caption.setToolTip("从 runs/<id>/ 复制，原文件不动")
        export_row.addWidget(self.export_caption)
        for key, title, _filters in EXPORTS:
            button = _export_button(title)
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
        """只读、整行选中、可排序的表格（见 :func:`make_table`）。"""
        return make_table(headers)

    def _build_waypoints_page(self) -> QtWidgets.QWidget:
        """「路点」页：一行一个 GP，失败行整行标红。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        # 说明放进 tooltip：结果区每多一行，窗口的最小高度就要多长 20 px
        # （这一页要装"状态 + 统计 + 页签 + 表 + 导出"，一行都省不得）。
        self.waypoint_table = page_layout.mark_result_table(
            self._make_table(WAYPOINT_HEADERS))
        self.waypoint_table.setToolTip(
            "点表头排序；无解 / 未通过自校验的行整行标红；"
            "选一行，图上的那个点会放大。")
        layout.addWidget(self.waypoint_table, 1)
        return page

    def _build_segments_page(self) -> QtWidgets.QWidget:
        """「段诊断」页：每段的最小裕度、是否穿零、最差点位置。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        self.segment_caption_text = (
            "最差点 = 段内 |J3| 最小的位置（按关节直线解析求 t，坐标沿线插值）。")
        self.segment_caption = _caption(self.segment_caption_text, lines=1)
        layout.addWidget(self.segment_caption)
        self.segment_table = page_layout.mark_result_table(
            self._make_table(SEGMENT_HEADERS))
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
            f"保留 {LOG_WIDGET_LIMIT} 行（超出删头部；省略提示会留在上面单独一行，"
            f"不会被删）。"))
        self.log_omission_label = QtWidgets.QLabel("")
        self.log_omission_label.setWordWrap(True)
        self.log_omission_label.setStyleSheet(_STATUS_STYLES["muted"])
        self.log_omission_label.hide()
        layout.addWidget(self.log_omission_label)
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
            self._push_figure(figure)
        return figure

    # ------------------------------------------------------------ 表格 ↔ 图
    def _connect_plot_link(self) -> None:
        """把路点表与图接起来（S6 P1）：两边都用 GP 号作键。"""
        self.waypoint_table.itemSelectionChanged.connect(
            self._on_waypoint_selected)
        signal = getattr(self._plot, "pointClicked", None)
        if signal is not None:
            signal.connect(self._on_plot_clicked)

    def _on_waypoint_selected(self) -> None:
        """表格选中变化 → 图上放大那个点，状态栏说出 GP 号。"""
        key = page_layout.table_key(self.waypoint_table,
                                    self.waypoint_table.currentRow())
        if key:
            self.select_key(key)

    def select_key(self, key, report: bool = True) -> None:
        """选中一个 GP：图上放大、表格行选中、状态栏显示 ``GP0005``。

        **隐藏页一律不动作**（S6 评审建议 2）：三页的 ``table`` 都 ``connect``
        同一个 ``plot.pointClicked``，一次点击会跑三个槽；隐藏页如果也照着
        “选中 + 上报”，可见页的状态栏就会被它改写（实测：在生成页点一下，
        隐藏的 ISO 页把状态栏写成 “已选中 P2（图上已放大）”）。

        Args:
            key: GP 号（``"5"`` / ``5``）或 ``customdata[0]`` 的原值。
            report: 是否写状态栏（程序内部调用时可以不重复写）。
        """
        if not self._active:
            return
        label = page_layout.gp_label(key)
        if report:
            self._report_status(f"已选中 {label}（图上已放大）")
        select = getattr(self._plot, "select_points", None)
        if callable(select):
            select([key])
        row = page_layout.find_row(self.waypoint_table, key)
        if row >= 0 and row != self.waypoint_table.currentRow():
            self.waypoint_table.blockSignals(True)
            try:
                self.waypoint_table.setCurrentCell(row, 0)
                self.waypoint_table.scrollToItem(
                    self.waypoint_table.item(row, 0))
            finally:
                self.waypoint_table.blockSignals(False)

    def _on_plot_clicked(self, keys) -> None:
        """图上点了点 → 表格选中它那一行（P1 的反向）。"""
        for key in (keys or []):
            if page_layout.find_row(self.waypoint_table, key) >= 0:
                self.select_key(key)
                return

    def _report_status(self, message: str) -> None:
        """把一句话写到窗口状态栏（没有窗口的纯逻辑测试里也能断言）。

        常驻信息（绑定/数据目录/WebGL）现在是主窗口的**永久控件**，临时消息
        超时后只会清掉临时区，不会把它们一起抹掉（S6 评审 R2a）。这里的文字
        带页面前缀（``生成：…``），因为三页共用一个状态栏。

        Args:
            message: 给用户看的一句话。
        """
        self.last_status = str(message)
        self._show_status()

    def _show_status(self) -> None:
        """把 :attr:`last_status`（带页面前缀）写进状态栏临时区。"""
        text = self.status_message()
        if not text:
            return
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(text, 10_000)

    def status_message(self) -> str:
        """状态栏里该显示的本页最后一句（带页面前缀）；没说过话时空串。"""
        text = str(getattr(self, "last_status", "") or "").strip()
        if not text:
            return ""
        return f"{self.STATUS_PREFIX}：{text}"

    def restore_plot_selection(self) -> None:
        """切回本页时把表格当前行补回图上（S6 评审 R2c）。

        ``Plotly.react`` 会把 marker 尺寸重置成原值，切页后高亮就没了；不补
        的话“表里选着、图上不亮、状态栏却说（图上已放大）”三者对不上。
        """
        table = self.waypoint_table
        row = table.currentRow()
        if row < 0:
            return
        key = page_layout.table_key(table, row)
        if key:
            self.select_key(key, report=False)

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
        """增量日志：直接追加；省略提示另存一份到固定标签（L4）。

        源缓冲丢行时 :class:`JobBridge` 会插一行「…已省略 N 行」。日志控件有
        ``setMaximumBlockCount``，这一行本身也会被后面的行顶掉——用户就只能
        看到“最后 500 行”而不知道前面已经少了一截。所以这句另存到一个不会
        被裁剪的标签上。
        """
        text = str(line)
        if text.startswith(OMISSION_PREFIX):
            self.log_omission_label.setText(
                f"{text}（更早的行已不在源缓冲里；下面只剩最后"
                f" {LOG_WIDGET_LIMIT} 行）")
            self.log_omission_label.show()
        self.log_edit.appendPlainText(text)

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
            self._push_figure(figure)
        self._expand_result_area()

    def _expand_result_area(self) -> None:
        """出结果后把结果区撑到"够看 :data:`RESULT_ROW_TARGET` 行"。

        实现见模块级 :func:`expand_result_area`（三页共用同一份口径）。
        """
        expand_result_area(self)

    def _fill_stats(self, stats: dict) -> None:
        """统计标签（实现见 :func:`fill_stat_labels`，修复页复用同一份）。"""
        fill_stat_labels(self.stat_labels, stats)

    def _fill_waypoints(self, rows) -> None:
        """路点表（实现见 :func:`fill_waypoint_table`，修复页复用同一份）。"""
        fill_waypoint_table(self.waypoint_table, rows)
        self.waypoint_table.scrollToTop()  # 排序后别停在中间，从 GP1 看起

    def _fill_segments(self, segments) -> None:
        """段诊断表（实现见 :func:`fill_segment_table`，修复页复用同一份）。"""
        self.last_segment_rows = fill_segment_table(
            self.segment_table, segments, caption=self.segment_caption,
            caption_text=getattr(self, "segment_caption_text", ""))

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
        """导出按钮：选目标后**复制**（不移动，``runs/`` 是归档）。

        实现见 :func:`export_artifact`（修复页复用同一份分支：缺产物 /
        源文件被清 / 手打漏扩展名 / 用户取消）。

        Args:
            key: ``artifacts`` 里的键。
        """
        level, message = export_artifact(self.last_result or {}, key, EXPORTS,
                                         self)
        if not message:                     # 用户取消
            return
        if level == "error":
            self._set_status(message, level="error")
        self._report_status(message)

    def _reset_result_area(self) -> None:
        """起新任务前清空结果区（上一次的表、图、日志不留在界面上冒充新结果）。"""
        self.last_result = None
        self.log_edit.clear()
        self.log_omission_label.clear()
        self.log_omission_label.hide()
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
        page_layout.fit_message(self.status_label, message, max_height=26)
        self.status_label.setStyleSheet(_STATUS_STYLES.get(level, ""))
        self.status_label.setVisible(bool(message.strip()))

    def shutdown(self) -> None:
        """窗口关闭时收尾：停去抖/合并定时器并注销任务订阅（不取消任务）。"""
        self._debounce.stop()
        self.bridge.shutdown()

    def set_active(self, active: bool) -> None:
        """主窗口切页时告知本页是不是当前页（隐藏页不往共享视图推图）。"""
        self._active = bool(active)

    def _push_figure(self, figure) -> None:
        """把图推给共享的 ``PlotView``：不是当前页就不推（免得盖掉别页的图）。"""
        if self._plot is None or not self._active:
            return
        self._plot.set_figure(figure)

    def current_figure(self):
        """本页当前该展示的图（导航切回来时主窗口用它重推）。

        Returns:
            plotly ``Figure`` 或 ``None``（还没出过图）。
        """
        return self.last_figure

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
