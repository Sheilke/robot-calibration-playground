# -*- coding: utf-8 -*-
"""ISO 9283 五点页（S5）：立方体 / 参考姿态 → **笛卡尔坐标** → 表 / 3D / 导出。

用户 2026-11-16 当面提的要求：**这一页不显示关节量**——只算 P1~P5 的 X/Y/Z，
姿态固定为用户填的参考姿态、不加扰动；IK 只解一次用来判断“这一点到不到得了”。
所以界面上没有 q1..q6、没有 |J3| / |J5| / 构型标签，也没有 ``q_seed`` /
``j5_min`` 输入（``core`` 的默认值照旧生效）。行为基准是网页版
``laser_web/pages/iso9283.py``：字段、默认值、``point_rows`` 的口径、产物下载项
都照它搬。

本模块只做编排与展示：

1. **收参数**：:func:`form_to_params` 的键集合与网页版 ``collect()`` 逐键一致。
   立方体只有**一种**给法（最大点 + 最小点），布点与截面按 ISO 9283 固定为
   ``vertices`` / ``xz``——这两项仍然提交给 ``core``（core 的参数约定不变），
   界面上没有可选项；
2. **预览**：改任何参数都在 :data:`DEBOUNCE_MS`（300 ms）后重算几何
   （:func:`preview_geometry`）——立方体边长 / 边心距、6 个面心、9 个候选点
   （8 顶点 + 中心）、最终 5 点（带 ISO 9283 的编号标注）。**不跑 IK**；
3. **算五点**：:meth:`IsoPage.start_iso` 在主线程调 ``core.iso_points``
   （纯几何 + 5 次 IK，实测 10 ms 量级），**没有任务队列 / 进度条 / 取消 / 日志**；
4. **结果区**：统计行（可达 N/5 · 边长 · 中心）+ ``QTabWidget`` 两页签——路点
   （P / X / Y / Z / 可达）/ 预览（几何）；
5. **导出**：``iso.json``（每点 X/Y/Z + 参考姿态）/ ``iso.png`` **在点导出时才
   生成**（matplotlib 出图约 0.3 s，不拖慢“点一下出结果”这一步），然后从
   ``run_dir`` **复制**（文件名带 ``<run_id>-`` 前缀，与生成页同一个实现
   :func:`laser_desktop.generate_page.export_artifact`）。

校验与语义**全部**走 ``core``：本页不写任何判定规则，``{"ok": False,
"error": ...}`` 时把报错原文用红字写在结果区状态行，**不弹窗**。

关于预览用到的两个 ``core`` 私有函数（见报告「与规格的偏差」）
------------------------------------------------------------
``core`` 没有 public 的 ``preview_iso``。为了不把「立方体怎么定 / 参数怎么归一」
在界面里再写一遍，预览直接复用 ``core._iso_options``（参数归一与取值域检查）与
``core._iso_cube``（由两个极值点算中心/边长）；两者都是 ``core.iso_points``
内部走同一条路径的函数，界面只是提前调用、不复制逻辑。
"""

from __future__ import annotations

import contextlib
import io
import logging
import time
from pathlib import Path

from excitation_trajectory import iso9283_cube as iso
from laser_web import core, ui_state

from . import form_widgets, generate_page, page_layout, paths
from .generate_page import (
    SortItem,
    _caption,
    _compact_result_layout,
    _double_spin,
    _inline,
    _row,
    export_artifact,
    format_metric,
    make_table,
    monospace_font,
    placeholder_figure,
    sort_key,
)
from .qt_compat import Qt, QtCore, QtGui, QtWidgets

#: 日志器（``core`` 万一抛异常、出图失败都记在这里）。
_LOGGER = logging.getLogger(__name__)

#: 立方体来源（ISO 9283 页只留一种：给最大/最小点）。值给 ``core``，文案给人看。
CUBE_MODE = "bounds"
#: 布点与截面固定为 ISO 9283 的取法：XZ 对角截面的四个角。界面不提供选项，
#: 但**仍然提交**给 ``core``（core 的参数约定不变）。
FIXED_LAYOUT = "vertices"
FIXED_PLANE = "xz"
#: 布点 / 截面的中文说明（图标题、预览卡的 tooltip 用）。
#: 图上标题用的简短说明。**不要**在这里写“对角截面四角 / XZ 对角面”：
#: ISO 9283 没有这样的可选项，写出来容易被当成“还有别的截面可选”。
GEOMETRY_LABEL = "P1 中心 + P2~P5 截面四角"
#: 五点的编号说明（ISO 9283）：P2/P3 在小 X 小 Z 一侧，P4/P5 在大 X 大 Z 一侧。
NUMBERING_NOTE = ("ISO 9283 五点：P1 立方体中心；P2/P3 在 X 小 Z 小一侧、"
                  "P4/P5 在 X 大 Z 大一侧（同侧两点差在 Y）；五点姿态完全相同，"
                  "用 MOVL 走直线。")
#: 这一页到底算什么（一字不改地照用户给的说法；结果区也要写一遍）。
HINT = ("只算笛卡尔坐标；姿态固定、不加扰动；系统会解一次 IK 判断每点是否可达，"
        "不输出关节角。")

#: 预览卡片里的三项数值（键, 名称）：由 :class:`page_layout.MetricGrid` 按需换行。
#: 布点 / 截面已不可选（固定为 ISO 9283 的取法），所以不再占两行。
PREVIEW_METRICS = (("cube", "立方体"), ("apothem", "边心距 L/2"),
                   ("center", "中心"))

#: 路点表列头（**没有** q1..q6 / |J3| / |J5| / cfg：这一页不出关节量）。
POINT_HEADERS = ("P", "X (mm)", "Y (mm)", "Z (mm)", "可达")

#: 预览里 6 个面心的标签（顺序 = X−/X+/Y−/Y+/Z−/Z+）。
FACE_LABELS = ("X− 面心", "X+ 面心", "Y− 面心", "Y+ 面心",
               "Z− 面心", "Z+ 面心")

#: 导出按钮：``(artifacts 键, 按钮文案, 文件对话框过滤器)``。产物是
#: ``iso.json``（每点 X/Y/Z + 参考姿态）与 ``iso.png``（五点图），**没有**
#: ``joints.txt``——这一页不输出关节角。
EXPORTS = (("iso_json", "iso.json", "JSON (*.json);;所有文件 (*)"),
           ("plot", "iso.png", "PNG 图片 (*.png);;所有文件 (*)"))

#: 两个页签（规格：路点 / 预览；日志页随任务机制一起删了）。
TABS = ("路点", "预览")

#: 状态行样式（与生成页 / 修复页同一组颜色）。
_STATUS_STYLES = generate_page._STATUS_STYLES
_GREEN_BOLD = "color:#1e8449; font-weight:bold"
_RED_BOLD = "color:#c0392b; font-weight:bold"
_WARN = "color:#8a6d3b"
_MONO_STYLE = "font-family: monospace;"

#: 预览去抖间隔（毫秒）：与生成页同一个数量级，拖动 spinbox 时不必每格重算。
DEBOUNCE_MS = 300

#: 出结果后结果区至少占多高（像素）：三页统一走
#: :func:`laser_desktop.generate_page.expand_result_area`（按"露 8 行"实测算），
#: 这里保留一个显式下限常量只为文档/测试引用。
RESULT_MIN_HEIGHT_ISO = generate_page.RESULT_MIN_HEIGHT


def run_dir_text(run_dir) -> str:
    """``/…/runs/<id>`` -> ``runs/<id>``（空值给 ``—``）。"""
    if not run_dir:
        return "—"
    path = Path(str(run_dir))
    return f"{path.parent.name}/{path.name}"


#: 统计行标签的键（顺序即界面顺序；单测按这些键取值）。
#: 只有「可达 N/5 / 边长 / 中心」——IK 统计（|J3| / |J5| / 构型）已经不显示。
STAT_KEYS = ("reach", "side", "center")

#: ``core`` 里的表单异常类型（``_FormError`` 是私有的，取不到就退回 ValueError，
#: 保证 ``{"ok": False, "error": ...}`` 的语义不变——见模块 docstring）。
_FORM_ERROR = getattr(core, "_FormError", ValueError)


# ---------------------------------------------------------------------------
# 纯逻辑（不碰 Qt，单测直接调）
# ---------------------------------------------------------------------------
def iso_form_defaults() -> dict:
    """ISO 表单的默认值：``core.iso_defaults()``（CLI 默认值）+ 网页版示例域。

    「立方体」的六个输入取网页版的示例值：``ui_state.SAMPLE_POINT_MIN/MAX``
    （与「生成」页的示例域同源，550 mm 立方体就是站点参数
    ``center 1025 0 605 / side 550``）。其余项**一个数字都不抄**，全部问 ``core``。
    ``q_seed`` / ``j5_min`` 不在里面：界面没有这两个输入（``core`` 用自己的默认）。

    Returns:
        dict: 键与 :func:`form_to_params` 的入参一致。
    """
    defaults = core.iso_defaults()
    low, high = ui_state.SAMPLE_POINT_MIN, ui_state.SAMPLE_POINT_MAX
    return {
        "robot": ui_state.default_robot(),
        "cube_mode": CUBE_MODE,
        # 两点输入：最小点 + 最大点（x0 x1 y0 y1 z0 z1）
        "cube": [float(low[0]), float(high[0]), float(low[1]),
                 float(high[1]), float(low[2]), float(high[2])],
        # 布点与截面不可选：固定 ISO 9283 的取法，仍然提交给 core。
        "layout": FIXED_LAYOUT,
        "plane": FIXED_PLANE,
        "rpy": [float(value) for value in defaults["rpy"]],
        "euler": str(defaults["euler"]),
    }


def form_to_params(values: dict) -> dict:
    """表单值 -> ``core.iso_points`` 的入参（只有一组：最大/最小点）。

    键集合与网页版 ``pages/iso9283.py::collect()`` 逐一对应（单测里逐个写死）：
    ``robot`` / ``cube_mode`` / ``cube`` / ``layout`` / ``plane`` / ``rpy`` /
    ``euler``。**不提交** ``q_seed`` / ``j5_min``（界面没有这两个输入），也不提交
    ``square_frac``（布点固定，k 用不到）。立方体只有一种给法，布点与截面也不
    可选：``layout`` / ``plane`` 由 :data:`FIXED_LAYOUT` / :data:`FIXED_PLANE`
    填，**仍然提交**——``core`` 的参数约定不变，``meta`` 照旧落盘。

    Args:
        values: 表单值，缺省项按 :func:`iso_form_defaults` 补齐。

    Returns:
        dict: 可直接喂 ``core.iso_points``。
    """
    values = {**iso_form_defaults(), **(values or {})}
    return {
        "robot": values["robot"],
        "cube_mode": CUBE_MODE,
        "cube": [float(value) for value in values["cube"]],
        "layout": str(values.get("layout") or FIXED_LAYOUT),
        "plane": str(values.get("plane") or FIXED_PLANE),
        "rpy": [float(value) for value in values["rpy"]],
        "euler": str(values["euler"]),
    }


def point_annotation(name: str) -> str:
    """单个点的位置标注（预览表里“这个点在立方体的哪一侧”）。

    文案描述的是 ``iso9283_cube.cube_points`` docstring 里写明的 ISO 9283 编号，
    不参与任何计算：P1 永远是立方体中心，P2/P3 在小 X 小 Z 一侧，P4/P5 在大 X
    大 Z 一侧（同侧两点差在 Y）。布点已经不可选，所以不再报“哪种布点”。

    Args:
        name: ``P1`` ~ ``P5``。

    Returns:
        str: 给现场看的一句话。
    """
    if str(name) == iso.POINT_NAMES[0]:
        return "立方体中心"
    side = "小" if str(name) in iso.POINT_NAMES[1:3] else "大"
    return f"X {side} Z {side} 一侧（对角截面角点）"


def face_centers(center, side: float) -> list[dict]:
    """立方体 6 个面心（中心 ± 半边长，逐轴一个）。

    Args:
        center: 立方体中心 ``(x, y, z)``，mm。
        side: 边长 L（mm）；面心到中心的距离是 L/2（边心距）。

    Returns:
        list[dict]: 6 行 ``{"label", "pos"}``，顺序同 :data:`FACE_LABELS`。
    """
    values = [float(value) for value in center]
    half = float(side) / 2.0
    rows = []
    for axis, name in enumerate("XYZ"):
        for sign, suffix in ((-1.0, "−"), (1.0, "+")):
            position = list(values)
            position[axis] += sign * half
            rows.append({"label": f"{name}{suffix} 面心", "pos": position})
    return rows


def candidate_points(center, side: float) -> list[dict]:
    """立方体的 9 个候选点：8 个顶点 + 中心。

    顶点顺序直接取 ``iso9283_cube.cube_vertices``（与算法模块判定“内接”时
    用的是同一份几何，不在界面里重算 ±h 的组合）。

    Args:
        center: 立方体中心 ``(x, y, z)``，mm。
        side: 边长 L（mm）。

    Returns:
        list[dict]: 9 行 ``{"label", "pos"}``（顶点 1~8 + 中心）。
    """
    rows = []
    for index, vertex in enumerate(iso.cube_vertices(center, side), start=1):
        rows.append({"label": f"顶点 {index}",
                     "pos": [float(value) for value in vertex]})
    rows.append({"label": "中心", "pos": [float(value) for value in center]})
    return rows


def preview_geometry(values: dict, capture_stdout: bool = True) -> dict:
    """按表单值算几何预览（**不跑 IK**）。

    走的是 ``core`` 的同一条参数通道：``core._iso_options`` 归一 + 校验，
    ``core._iso_cube`` 由两个极值点定中心/边长，最后用
    ``iso9283_cube.cube_points`` 出五点。任何校验失败都转成
    ``{"ok": False, "error": 报错原文}``（原文来自 ``core``，界面不改写）。

    Args:
        values: 表单值（缺省项按 :func:`iso_form_defaults` 补齐）。
        capture_stdout: 是否把 ``core._iso_cube`` 的那句 ``print`` 提示抓进
            ``warnings``。**别的页有任务在跑时必须传 ``False``**：
            ``redirect_stdout`` 换的是进程级的 ``sys.stdout``，工作线程的日志
            会一起被吞掉。抓不到只是少一行提示，边长 / 中心 / 五点全部来自
            返回值，一个数都不受影响。

    Returns:
        dict: ``{"ok", "layout", "plane", "square_frac", "center", "side",
        "apothem", "faces", "candidates", "points", "warnings", "error"}``；
        ``points`` 的每一项是 ``{"name", "pos", "label"}``。
    """
    params = form_to_params(values)
    try:
        options = core._iso_options(params)
    except _FORM_ERROR as error:                    # 参数域错误：原文回显
        return {"ok": False, "error": str(error), "warnings": []}
    except (TypeError, ValueError) as error:
        return {"ok": False, "error": f"参数无法解析：{error}", "warnings": []}

    preview = {"ok": True,
               "layout": str(options["layout"]),
               "plane": str(options["plane"]),
               "square_frac": float(options["square_frac"]),
               "center": None, "side": None, "apothem": None,
               "faces": [], "candidates": [], "points": [],
               "warnings": [], "error": None}

    hint = ""
    try:
        # 只有「两个极值点」这一种来源：``core._iso_cube`` 的这个分支不碰
        # rbt / order / extrinsic，传 None 是安全的。
        # ``core._iso_cube`` 在“三轴跨度不等”时会 ``print`` 一句提示（CLI 的
        # 行为）；桌面版把那段 stdout 抓下来当预览提示——提示文字的来源仍然
        # 是 core，界面不另写一份判据。
        #
        # ``contextlib.redirect_stdout`` 换的是进程级的 ``sys.stdout``：别的页
        # 有任务在跑时抓，会把工作线程正在 ``print`` 的日志行吞进这个 buffer。
        # 所以只在**没有任务在跑**时才抓（判据由调用方传 ``capture_stdout``）。
        if capture_stdout:
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                center, side = core._iso_cube(options, None, order=None,
                                              extrinsic=None, q_seed=None)
            hint = buffer.getvalue().strip()
        else:
            center, side = core._iso_cube(options, None, order=None,
                                         extrinsic=None, q_seed=None)
    except _FORM_ERROR as error:
        return {"ok": False, "error": str(error), "warnings": []}
    if hint:
        preview["warnings"].append(hint)

    preview["center"] = [float(value) for value in center]
    preview["side"] = float(side)
    preview["apothem"] = float(side) / 2.0

    try:
        points = iso.cube_points(preview["center"], preview["side"],
                                 preview["layout"], preview["square_frac"],
                                 preview["plane"])
    except ValueError as error:                     # layout / square_frac 语义错
        return {"ok": False, "error": str(error),
                "warnings": preview["warnings"]}

    preview["faces"] = face_centers(preview["center"], preview["side"])
    preview["candidates"] = candidate_points(preview["center"], preview["side"])
    preview["points"] = [
        {"name": name,
         "pos": [float(value) for value in points[name]],
         "label": point_annotation(name)}
        for name in iso.POINT_NAMES]
    return preview


def point_rows(result: dict) -> list[dict]:
    """``core.iso_points()["points"]`` -> 表格行（保留数字类型，不可达行标红）。

    与网页版 ``_point_rows`` 同一约定：数值不转字符串（表头要按大小排序）；
    这一页**没有关节列**——只有 P / X / Y / Z / 可达。
    """
    rows = []
    for point in (result or {}).get("points") or []:
        reachable = bool(point.get("reachable", point.get("ok")))
        position = point.get("pos") or [None, None, None]
        rows.append({"name": str(point.get("name") or "?"),
                     "x": _round(position[0], 1),
                     "y": _round(position[1], 1),
                     "z": _round(position[2], 1),
                     "reach": "✓ 可达" if reachable else "✗ 不可达",
                     "failed": not reachable})
    return rows


def _round(value, digits: int):
    """保留位数（``None`` 原样返回）；表格排序键用数值，不用字符串。"""
    return None if value is None else round(float(value), digits)


def _coordinate_text(position) -> str:
    """一行坐标 ``( 1025.0,     0.0,   605.0)``（预览里的等宽对齐）。"""
    values = [float(value) for value in position]
    return "(" + ", ".join(f"{value:9.1f}" for value in values) + ")"


def block_text(rows) -> str:
    """预览文本块：``标签  (x, y, z)`` 逐行（面心 / 候选点共用）。"""
    return "\n".join(f"{row['label']:<8s}{_coordinate_text(row['pos'])}"
                     for row in rows or [])


def fill_point_table(table, rows) -> None:
    """五点表：P / X / Y / Z / 可达；不可达整行标红。

    Args:
        table: 列头为 :data:`POINT_HEADERS` 的 ``QTableWidget``。
        rows: :func:`point_rows` 的返回值。
    """
    table.setSortingEnabled(False)
    table.clearContents()
    table.setRowCount(len(rows))
    for row_index, row in enumerate(rows):
        ok = bool(row.get("reach") == "✓ 可达")
        cells = [
            ((0, row["name"]), row["name"], "center"),
            (sort_key(row["x"]), format_metric(row["x"], 1), "right"),
            (sort_key(row["y"]), format_metric(row["y"], 1), "right"),
            (sort_key(row["z"]), format_metric(row["z"], 1), "right"),
            ((0, 1 if ok else 0), row["reach"], "center"),
        ]
        for column, (key, text, align) in enumerate(cells):
            item = SortItem(key, text)
            item.setTextAlignment(
                {"right": Qt.AlignRight | Qt.AlignVCenter,
                 "center": Qt.AlignCenter,
                 "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
            if not ok:                       # 不可达整行标红（排序后也不丢）
                item.setBackground(QtGui.QColor(generate_page._FAILED_BG))
                item.setForeground(QtGui.QColor(generate_page._FAILED_FG))
            table.setItem(row_index, column, item)
    table.setSortingEnabled(True)
    if rows:
        table.sortItems(0, Qt.AscendingOrder)
    table.resizeColumnsToContents()


def fill_preview_table(table, preview) -> None:
    """预览表：P / X / Y / Z / 位置标注（5 行，不排序）。"""
    table.setSortingEnabled(False)
    table.clearContents()
    points = (preview or {}).get("points") or []
    table.setRowCount(len(points))
    for row_index, point in enumerate(points):
        position = point["pos"]
        cells = [((0, point["name"]), point["name"], "center")]
        cells += [(sort_key(position[axis]),
                   format_metric(position[axis], 1), "right")
                  for axis in range(3)]
        cells += [((0, point["label"]), point["label"], "left")]
        for column, (key, text, align) in enumerate(cells):
            item = SortItem(key, text)
            item.setTextAlignment(
                {"right": Qt.AlignRight | Qt.AlignVCenter,
                 "center": Qt.AlignCenter,
                 "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
            table.setItem(row_index, column, item)
    table.sortItems(0, Qt.AscendingOrder)
    table.resizeColumnsToContents()


def stats_view(result: dict) -> dict:
    """``core.iso_points`` 的结果 -> 统计行的口径（可达 / 边长 / 中心）。

    兼容旧形状（``stats.solved`` / ``stats.cube_side``）：一部分单测与冒烟用
    构造的 dict 喂 ``render_result``，这里只做字段归一，不改判定。

    Args:
        result: ``core.iso_points()`` 的返回值（或同构的构造 dict）。

    Returns:
        dict: ``{"reachable", "total", "cube_side", "cube_center"}``。
    """
    result = result or {}
    summary = result.get("summary") or {}
    meta = result.get("meta") or {}
    stats = result.get("stats") or {}
    reachable = summary.get("reachable", stats.get("solved"))
    total = summary.get("total", stats.get("total"))
    side = meta.get("cube_side", stats.get("cube_side"))
    center = meta.get("cube_center", stats.get("cube_center"))
    return {"reachable": reachable, "total": total,
            "cube_side": side, "cube_center": center}


def fill_iso_stats(labels, stats) -> None:
    """统计行：**可达 N/5** / 边长 / 中心（没有 |J3| / |J5| / 构型）。

    Args:
        labels: ``{STAT_KEYS 里的键: QLabel}``。
        stats: :func:`stats_view` 的返回值。
    """
    stats = stats or {}
    reachable = stats.get("reachable")
    total = stats.get("total")
    all_reachable = bool(total) and reachable == total
    labels["reach"].setText(
        "可达 —" if total is None
        else f"可达 {int(reachable or 0)}/{int(total)}")
    labels["reach"].setStyleSheet(_GREEN_BOLD if all_reachable else _RED_BOLD)
    side = stats.get("cube_side")
    center = stats.get("cube_center")
    labels["side"].setText(
        "边长 —" if side is None else f"边长 {format_metric(side, 0)} mm")
    labels["center"].setText(
        "中心 —" if not center
        else "中心 (" + ", ".join(f"{float(value):.0f}" for value in center) + ") mm")


def preview_figure(preview: dict):
    """预览图：立方体 12 条棱 + 9 个候选点 + 最终 5 点（**不跑 IK**）。

    与 ``laser_web.figures`` 的区别只有一处：那份按 ``ok``
    把点分成“可解 / 无解”两色，而预览还没有解过 IK，所以这里用中性配色
    （灰棱 / 灰点 / 蓝点 + 点名），不去断言“有解”。

    Args:
        preview: :func:`preview_geometry` 的返回值。

    Returns:
        plotly ``Figure``（几何不可用时是说明图）。
    """
    preview = preview or {}
    if not preview.get("ok"):
        return placeholder_figure(preview.get("error") or "参数不可用，先不画图")
    if not preview.get("points"):
        return placeholder_figure("还没有可以画的点",
                                  title="ISO 五点预览（未出图）")

    import plotly.graph_objects as go

    center = preview["center"]
    side = preview["side"]
    corners = [[float(value) for value in corner]
               for corner in iso.cube_vertices(center, side)]
    # 12 条棱：cube_vertices 的顺序是 sx, sy, sz 三层循环，下标差 1/2/4 即相邻
    edges = [(index, index | bit) for index in range(8)
             for bit in (1, 2, 4) if not index & bit]
    xs, ys, zs = [], [], []
    for first, second in edges:
        for point in (corners[first], corners[second]):
            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])
        xs.append(None)                             # 断开：一条 trace 画 12 条棱
        ys.append(None)
        zs.append(None)
    figure = go.Figure()
    figure.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs, mode="lines",
        line={"color": "rgba(140,140,140,0.85)", "width": 2},
        name=f"立方体 L={float(side):g} mm", hoverinfo="skip"))
    candidates = preview["candidates"]
    figure.add_trace(go.Scatter3d(
        x=[row["pos"][0] for row in candidates],
        y=[row["pos"][1] for row in candidates],
        z=[row["pos"][2] for row in candidates],
        mode="markers",
        marker={"size": 3, "color": "#9aa0a6"},
        name=f"候选点 {len(candidates)} 个（8 顶点 + 中心）",
        hovertext=[f"{row['label']} {_coordinate_text(row['pos'])}"
                   for row in candidates],
        hovertemplate="%{hovertext}<extra></extra>"))
    points = preview["points"]
    figure.add_trace(go.Scatter3d(
        x=[row["pos"][0] for row in points],
        y=[row["pos"][1] for row in points],
        z=[row["pos"][2] for row in points],
        mode="markers+text",
        marker={"size": 6, "color": "#1976d2",
                "line": {"color": "#20232a", "width": 0.5}},
        text=[row["name"] for row in points],
        customdata=[[str(row["name"])] for row in points],
        textposition="top center",
        textfont={"size": 11, "color": "#1976d2"},
        hovertext=[f"{row['name']} {_coordinate_text(row['pos'])}"
                   f"<br>{row['label']}" for row in points],
        hovertemplate="%{hovertext}<extra></extra>",
        name="P1~P5（未解 IK）"))
    figure.update_layout(
        title=f"ISO 9283 五点预览：{GEOMETRY_LABEL}",
        height=520,
        margin={"l": 0, "r": 0, "t": 36, "b": 0},
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.0,
                "xanchor": "left", "x": 0.0},
        scene={"xaxis": {"title": "X (mm)", "backgroundcolor": "#fafafa"},
               "yaxis": {"title": "Y (mm)", "backgroundcolor": "#fafafa"},
               "zaxis": {"title": "Z (mm)", "backgroundcolor": "#fafafa"},
               "aspectmode": "data"})
    return figure


def attach_point_keys(figure, points):
    """给 ISO 图的点 trace 补 ``customdata[0] = 点名``（P1 的联动键）。

    ``laser_web.figures`` 是网页版共用资产，它没写 ``customdata``；桌面版要
    “点图 → 选中表格行”，所以在桌面层按 trace 名补上。trace 只有三种：
    立方体棱线（没有键，点它不联动）、可达到的点、不可达的点。

    Args:
        figure: :func:`laser_web.figures.iso_points_figure` 的结果（就地改）。
        points: ``core.iso_points()["points"]``。

    Returns:
        plotly ``Figure``: 同一个对象（便于链式调用）。
    """
    rows = [row for row in (points or []) if row.get("pos")]
    good = [[str(row.get("name"))] for row in rows
            if row.get("reachable", row.get("ok"))]
    bad = [[str(row.get("name"))] for row in rows
           if not row.get("reachable", row.get("ok"))]
    for trace in figure.data or []:
        name = str(getattr(trace, "name", "") or "")
        if name.startswith("可达"):
            trace.customdata = good
        elif name.startswith("不可达"):
            trace.customdata = bad
    return figure


def result_figure(result: dict):
    """结果图：五点 3D（立方体 + 可达/不可达两色，**没有关节量**）。

    Args:
        result: ``core.iso_points`` 的返回值。

    Returns:
        plotly ``Figure``（点 trace 带 ``customdata[0] = P1..P5``）。
    """
    from laser_web import figures

    stats = stats_view(result)
    figure = figures.iso_points_figure(
        (result or {}).get("points") or [],
        cube_center=stats.get("cube_center"),
        side=stats.get("cube_side"),
        height=520)
    return attach_point_keys(figure, (result or {}).get("points") or [])


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class IsoPage(QtWidgets.QWidget):
    """ISO 五点页：左侧表单 + 右侧三段（预览卡 / 图 / 两页签结果区）。

    Attributes:
        fields: ``{键: 控件}``，取值与 :func:`form_to_params` 的入参同名。
        bounds_panel: 立方体的一组输入（最大点 + 最小点；没有第二种给法）。
        stat_labels: 结果区统计行（键见 :data:`STAT_KEYS`）。
        point_table: 路点表（P1~P5 的 X/Y/Z + 可达，不可达行标红）。
        preview_table: 预览页签里的 5 点表（带 ISO 9283 的位置标注）。
        faces_edit / candidates_edit: 预览页签的 6 个面心 / 9 个候选点文本块。
        export_buttons: ``{artifacts 键: 按钮}``，出结果后即可点（导出时生成）。
        preview: 最近一次的 :func:`preview_geometry` 结果。
        last_result: 最近一次 ``core.iso_points`` 的返回值（含 ``ok=False``）。
        last_status: 最近一次写到状态栏的文字（无主窗口时也能断言）。
        last_elapsed_ms: 最近一次算五点花了多少毫秒（报告/单测用）。
        run_state: ``idle`` / ``done`` / ``failed``（没有“running”：算得太快）。
    """

    #: 预览去抖间隔（毫秒）。
    DEBOUNCE_MS = DEBOUNCE_MS

    #: 状态栏里的页面前缀（三页共用一个状态栏）。
    STATUS_PREFIX = "ISO"

    def __init__(self, plot=None, robots=None, parent=None, bridge=None,
                 settings=None) -> None:
        """建页面。

        Args:
            plot: ``PlotView``（或任何有 ``set_figure`` 的对象）；``None`` 表示
                不挂 WebEngine（纯逻辑测试用，图仍然算好放在 ``last_figure``）。
            robots: :func:`laser_web.core.list_robots` 的返回值；``None`` 时现查。
            parent: 父控件。
            bridge: **已废弃**（这一页不再建任务）；保留形参只为调用方兼容，
                传什么都不会用它。
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
        self.fields = {}
        self.stat_labels = {}
        self.export_buttons = {}
        self.preview = None
        self.last_result = None
        self.last_figure = None
        self.last_status = ""
        self.last_elapsed_ms = None
        self.run_state = "idle"
        # 表单改动序号 / 出结果时的改动序号：相等说明“结果渲染后没人再改表单”，
        # 这时到点的预览去抖不许拿预览图盖掉结果图（与生成页同一道保护）。
        self._change_seq = 0
        self._result_seq = -1
        self._debounce = QtCore.QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(self.DEBOUNCE_MS)
        self._debounce.timeout.connect(self.apply_preview)

        column = QtWidgets.QVBoxLayout(self)
        column.setContentsMargins(8, 8, 8, 8)
        column.setSpacing(6)
        self.banner = QtWidgets.QLabel("")
        self.banner.setWordWrap(True)
        self.banner.hide()
        column.addWidget(self.banner)

        # 左右两栏之间是**横向**分割器（S6e，与生成页同一份实现）：宽度下限
        # 钉在表单/右栏自身的最小宽度上，比例存 ``body_splitter/iso``。
        body = page_layout.BodySplitter("iso", self._build_left_column(),
                                        self._build_right_column(),
                                        settings=self._settings)
        self._body_splitter = body
        column.addWidget(body, 1)

        self._connect_plot_link()
        self.refresh_preview()

    def _build_left_column(self) -> QtWidgets.QWidget:
        """左栏：表单滚动区 + **主操作行**（S6c）。

        「生成五点」原来在表单**最底下**（在滚动区里）：表单内容一多它就滚出
        视野，离屏实测 ``visibleRegion()`` 是空的（看不见）。现在它跟在表单
        滚动区外面，垂直策略 ``Fixed``，两种窗口尺寸下都按 ``sizeHint`` 拿到整高。
        """
        column = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        layout.addWidget(self._build_form(), 1)
        layout.addWidget(self._build_actions(), 0)
        return column

    def _build_actions(self) -> QtWidgets.QWidget:
        """主操作行：「生成五点」。"""
        self.generate_button = QtWidgets.QPushButton("生成五点")
        self.generate_button.setToolTip(
            "算 P1~P5 的 X/Y/Z 并验算一次可达性（主线程，毫秒级；不输出关节角）")
        self.generate_button.clicked.connect(self.start_iso)
        self.action_row = page_layout.build_action_row(self.generate_button)
        return self.action_row

    def _build_right_column(self) -> QtWidgets.QWidget:
        """右栏三段：预览卡 / 图 / 结果区（L1，与生成页同一份实现）。"""
        self.plot_slot = page_layout.PlotSlot(
            "ISO 五点图在这里（本环境未挂 WebEngine）")
        if (self._plot is not None
                and not getattr(self._plot, "SHARED_ACROSS_PAGES", False)):
            self.plot_slot.adopt(self._plot)
        splitter = page_layout.RightSplitter(
            "iso", self._build_preview_card(), self.plot_slot,
            self._build_result_area(), settings=self._settings)
        self._viewer_splitter = splitter
        self._right_splitter = splitter
        return splitter

    # ------------------------------------------------------------------ 表单
    def _build_form(self) -> QtWidgets.QScrollArea:
        """左侧表单：模型 / 立方体 / 参考姿态（没有种子与告警项）。"""
        defaults = iso_form_defaults()
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setSpacing(8)

        robot = form_widgets.build_robot_group(
            self.fields, self._robots, self._on_change, defaults["robot"])
        self.robot_path_label = robot["path_label"]
        self.skipped_label = robot["skipped_label"]
        self.robot_group = robot["group"]     # 冒烟/单测要量它高没高（P3）
        layout.addWidget(robot["group"])
        layout.addWidget(self._build_cube_group(defaults))
        layout.addWidget(self._build_pose_group(defaults))
        layout.addStretch(1)

        scroll = generate_page._form_scroll(content,
                                            generate_page.FORM_MAX_WIDTH)
        self.form_scroll = scroll                # 冒烟/单测要查横滚条（P4）
        return scroll

    def _build_cube_group(self, defaults) -> QtWidgets.QGroupBox:
        """立方体：**只有**「最大点 + 最小点」一种给法。

        另一条路（在搜索箱里自动搜内接立方体）与布点 / 截面选项一起删了：
        ISO 9283 没有“布点 / 截面”这种选法，页面也不应该自己搜立方体
        ——立方体由现场按工作空间给。
        """
        group = QtWidgets.QGroupBox("立方体（给最大/最小点）")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)
        self.bounds_panel = self._build_bounds_panel(defaults)
        layout.addWidget(self.bounds_panel)
        return group

    def _build_bounds_panel(self, defaults) -> QtWidgets.QWidget:
        """两点输入：最小点 + 最大点（各 X/Y/Z 三个数，整数 mm 步进）。"""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        cube = list(defaults["cube"])
        for row_label, keys, values in (
                ("最小点", ("x0", "y0", "z0"), cube[0:6:2]),
                ("最大点", ("x1", "y1", "z1"), cube[1:6:2])):
            columns = []
            for key, axis, value in zip(keys, "XYZ", values):
                box = _double_spin(-10000.0, 10000.0, value, decimals=1,
                                   step=1.0)
                box.valueChanged.connect(self._on_change)
                self.fields[key] = box
                columns.append((axis, box))
            layout.addWidget(_row(QtWidgets.QLabel(row_label), _inline(columns)))
        layout.addWidget(_caption(
            "只填这两个极值点；程序取各轴中点为中心、最小跨度为立方体边长"
            "（三轴跨度不等时预览会提示）。"))
        return panel

    def _build_pose_group(self, defaults) -> QtWidgets.QGroupBox:
        """参考姿态：五点都用它，**不加扰动**、也不再有 IK 种子。"""
        group = QtWidgets.QGroupBox("参考姿态")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)
        pose = form_widgets.build_pose_group(self.fields, defaults,
                                            self._on_change)
        layout.addWidget(pose)
        layout.addWidget(_caption(HINT, lines=3))
        return group

    # ------------------------------------------------------------ 预览卡片
    def _build_preview_card(self) -> QtWidgets.QGroupBox:
        """预览卡片：立方体 / 边心距 / 中心 + 报错原文 + 提示。

        三项用 :class:`laser_desktop.page_layout.MetricGrid`（按需换行，一行
        放不下就换行），**都不会横向裁字**。布点 / 截面已不可选（按 ISO 9283
        固定为对角截面四角、XZ 面），所以这两行拿掉了。

        「生成五点」在左栏底部的操作行里（S6c：它原来在表单**滚动区**最底下，
        内容一多就滚出视野——离屏实测 ``visibleRegion()`` 是空的）。说明文字走
        tooltip：``HINT`` 在左侧「参考姿态」组里已经完整写过一遍。
        """
        group = QtWidgets.QGroupBox("预览（不跑 IK）")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(3)
        layout.setContentsMargins(4, 2, 4, 2)

        self.preview_labels = {}
        self.metric_grid = page_layout.MetricGrid(
            self.preview_labels, PREVIEW_METRICS)
        layout.addWidget(self.metric_grid)

        self.error_label = QtWidgets.QLabel("")
        self.error_label.setWordWrap(True)
        self.error_label.setStyleSheet(_STATUS_STYLES["error"])
        self.error_label.hide()
        layout.addWidget(self.error_label)

        self.warning_label = QtWidgets.QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet(_WARN)
        self.warning_label.hide()
        layout.addWidget(self.warning_label)

        group.setToolTip(
            NUMBERING_NOTE + "改任何一项都会重算预览（面心 / 候选点 / 五点都在"
            "下面的「预览」页签里）。")
        return group

    def _build_result_area(self) -> QtWidgets.QGroupBox:
        """结果区：状态行 + 统计行 + 两页签（路点/预览）+ 导出。"""
        group = QtWidgets.QGroupBox("结果")
        self.result_area = group        # expand_result_area 要量"表以外占多高"
        layout = QtWidgets.QVBoxLayout(group)
        _compact_result_layout(layout)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.status_label.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                        QtWidgets.QSizePolicy.Ignored)
        layout.addWidget(self.status_label)
        self.status_label.hide()

        # 参数改完但没重跑：灰字点明“下表是上一次运行的”，就压在被提示的表上面
        self.stale_label = QtWidgets.QLabel("")
        self.stale_label.setWordWrap(True)
        self.stale_label.setStyleSheet(_STATUS_STYLES["muted"])
        self.stale_label.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                       QtWidgets.QSizePolicy.Ignored)
        self.stale_label.hide()
        layout.addWidget(self.stale_label)
        # 统计卡：按需换行（1600×950 一行、1280×800 两行，都不切字）
        self.stats_holder = page_layout.StatsGrid(self.stat_labels, STAT_KEYS)
        self.stats_holder.hide()        # 没结果时不占高度（预览时图可以高一点）
        layout.addWidget(self.stats_holder)

        self.result_tabs = QtWidgets.QTabWidget()
        self.points_page = self._build_points_page()
        self.preview_page = self._build_preview_page()
        self.result_tabs.addTab(self.points_page, TABS[0])
        self.result_tabs.addTab(self.preview_page, TABS[1])
        # 最小高度按**最高的那一页**算（见 page_layout.tabs_min_height）：表格的
        # 最小高度随行高变，写常数会让大字体下的表被页签裁掉半截。
        page_layout.mark_tabs_min_height(self.result_tabs)
        # 切页签后重新撑一次（口径见 generate_page.expand_result_area）
        self.result_tabs.currentChanged.connect(
            lambda _index: self._expand_result_area())
        layout.addWidget(self.result_tabs, 1)

        export_row = QtWidgets.QHBoxLayout()
        export_row.setContentsMargins(0, 0, 0, 0)
        self.export_caption = _caption("导出：", lines=1)
        self.export_caption.setToolTip(
            "点导出时才生成文件（出图约 0.3 s），再从 runs/<id>/ 复制，原文件不动")
        export_row.addWidget(self.export_caption)
        for key, title, _filters in EXPORTS:
            button = generate_page._export_button(title)
            button.setEnabled(False)
            button.clicked.connect(
                lambda _checked=False, export_key=key: self._on_export(export_key))
            self.export_buttons[key] = button
            export_row.addWidget(button)
        export_row.addStretch(1)
        layout.addLayout(export_row)
        return group

    def _build_points_page(self) -> QtWidgets.QWidget:
        """“路点”页：一行一个点（P1~P5 的 X/Y/Z + 可达），不可达行整行标红。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        # 一句话（1 行）：结果区每多一行说明，路点表就少露一行（1280×800 要
        # 露满 5 行）。完整编号说明在 tooltip 里（``HintLabel`` 自动带）。
        layout.addWidget(_caption(
            "只列坐标：P1 中心；P2/P3 小 X 小 Z；P4/P5 大 X 大 Z。"
            "不可达整行标红。", lines=1))
        self.point_table = page_layout.mark_result_table(
            make_table(POINT_HEADERS))
        layout.addWidget(self.point_table, 1)
        return page

    def _build_preview_page(self) -> QtWidgets.QScrollArea:
        """“预览”页：6 个面心 / 9 个候选点 / 最终 5 点（带位置标注）。"""
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.preview_hint = _caption(
            "几何预览（不跑 IK）：改任何参数 300 ms 后重算这里。")
        layout.addWidget(self.preview_hint)

        layout.addWidget(QtWidgets.QLabel("6 个面心"))
        self.faces_edit = self._make_text_block("还没有立方体")
        layout.addWidget(self.faces_edit)

        layout.addWidget(QtWidgets.QLabel("9 个候选点（8 顶点 + 中心）"))
        self.candidates_edit = self._make_text_block("还没有立方体")
        layout.addWidget(self.candidates_edit)

        layout.addWidget(QtWidgets.QLabel("最终 5 点（按 ISO 9283 编号）"))
        self.preview_table = make_table(("P", "X (mm)", "Y (mm)", "Z (mm)",
                                        "位置标注"))
        self.preview_table.setSortingEnabled(False)
        self.preview_table.setMinimumHeight(130)
        layout.addWidget(self.preview_table)

        layout.addWidget(_caption(HINT, lines=3))
        layout.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        return scroll

    def _make_text_block(self, placeholder: str) -> QtWidgets.QPlainTextEdit:
        """预览里的等宽只读文本块（面心 / 候选点）。"""
        edit = QtWidgets.QPlainTextEdit()
        edit.setReadOnly(True)
        edit.setFont(monospace_font())
        edit.setPlaceholderText(placeholder)
        edit.setMaximumBlockCount(64)
        edit.setMinimumHeight(96)
        edit.setMaximumHeight(140)
        return edit

    # ------------------------------------------------------------------ 取值
    def values(self) -> dict:
        """当前表单值（键与 :func:`form_to_params` 的入参一致）。

        ``cube_mode`` / ``layout`` / ``plane`` 不是控件值：立方体只有一种给法，
        布点与截面按 ISO 9283 固定（见 :data:`CUBE_MODE` / :data:`FIXED_LAYOUT`
        / :data:`FIXED_PLANE`），仍然跟着参数一起交给 ``core``。
        """
        fields = self.fields
        return {
            "robot": fields["robot"].currentData() or ui_state.default_robot(),
            "cube_mode": CUBE_MODE,
            "cube": [fields["x0"].value(), fields["x1"].value(),
                     fields["y0"].value(), fields["y1"].value(),
                     fields["z0"].value(), fields["z1"].value()],
            "layout": FIXED_LAYOUT,
            "plane": FIXED_PLANE,
            "rpy": [fields["rpy_a"].value(), fields["rpy_b"].value(),
                    fields["rpy_c"].value()],
            "euler": fields["euler"].currentData(),
        }

    def params(self) -> dict:
        """当前表单值 -> ``core.iso_points`` 的入参。"""
        return form_to_params(self.values())

    def preview_result(self) -> dict:
        """最近一次 :func:`preview_geometry` 的结果（原样，含报错原文）。"""
        return self.preview or {}

    # ------------------------------------------------------------ 刷新
    def _on_change(self, *_args) -> None:
        """任何输入的改动都走这里：立刻（去抖后）重算预览。"""
        self.refresh_preview()

    @staticmethod
    def _job_running_elsewhere() -> bool:
        """本进程里还有别的页的任务在跑？（决定预览能不能抓 stdout）"""
        try:
            registry = core.jobs.get_registry()
            return any(job.state not in core.jobs.TERMINAL_STATES
                       for job in registry.jobs())
        except Exception:                    # noqa: BLE001 - 判不出来就当没有
            return False

    def refresh_preview(self) -> dict:
        """重算几何预览（去抖 300 ms），并按结果启用/禁用「生成五点」。

        Returns:
            dict: :func:`preview_geometry` 的结果。
        """
        # 别的页有任务在跑时工作线程也在 ``print``，而 ``redirect_stdout``
        # 换的是进程级 ``sys.stdout``：这时抓会把它的日志行吞掉。
        capture = not self._job_running_elsewhere()
        # 每重算一次预览就算“表单变了一版”：结果图的保护判据靠它区分
        # “用户又改了”与“只是陈旧去抖到点了”。
        self._change_seq += 1
        try:
            preview = preview_geometry(self.values(), capture_stdout=capture)
        except Exception as error:          # noqa: BLE001 - 别打断 Qt 槽
            _LOGGER.exception("preview_geometry 抛异常（卡片显示失败原因）")
            preview = {"ok": False,
                       "error": f"预览失败：{type(error).__name__}: {error}",
                       "warnings": []}
        self.preview = preview
        self._update_card(preview)
        self._sync_buttons()
        self._update_stale_hint()
        self._debounce.start()
        return preview

    def _update_stale_hint(self) -> None:
        """参数改动后仍在显示上一次运行：灰字点明，别拿旧表对新立方体。

        跑完之后改参数只重算预览（图会换成新立方体），统计行 / 路点表却仍是
        上一次运行的那一份——现场最容易在这一刻看错。``_result_seq`` 是出结果
        时的 ``_change_seq``，两者不等就说明“结果之后表单又动过”。
        """
        result = self.last_result or {}
        stale = (bool(result) and self._result_seq >= 0
                 and self._result_seq != self._change_seq
                 and self.point_table.rowCount() > 0)
        if not stale:
            self.stale_label.hide()
            return
        page_layout.fit_message(
            self.stale_label,
            "参数已改动，下表来自上一次的结果；要更新请重新点「生成五点」。")
        self.stale_label.show()

    def _update_card(self, preview: dict) -> None:
        """预览卡片：数值或 ``core`` 的报错原文（红字，不弹窗）。"""
        preview = preview or {}
        if not preview.get("ok"):
            self.error_label.setText(str(preview.get("error") or "预览失败"))
            self.error_label.show()
            self.warning_label.hide()
            for label in self.preview_labels.values():
                label.setText("—")
            self.preview_table.setRowCount(0)
            self.faces_edit.setPlainText("")
            self.candidates_edit.setPlainText("")
            self.preview_hint.setText("参数不合法：先按左边的报错改。")
            return
        self.error_label.hide()
        side = preview.get("side")
        if side is None:                    # 只可能是预览没算出边长（不该发生）
            self.preview_labels["cube"].setText("—")
            self.preview_labels["apothem"].setText("—")
            self.preview_labels["center"].setText("—")
            self.preview_hint.setText("还没有立方体：先填最大点与最小点。")
            self.preview_table.setRowCount(0)
            self.faces_edit.setPlainText("")
            self.candidates_edit.setPlainText("")
            return
        self.preview_labels["cube"].setText(f"边长 {float(side):g} mm")
        # ``apothem`` 缺值要显示 ``—``：``or 0`` 会把它画成「0 mm」，
        # 看起来像边心距真的是 0。
        apothem = preview.get("apothem")
        self.preview_labels["apothem"].setText(
            "—" if apothem is None else f"{float(apothem):g} mm")
        center = preview.get("center") or []
        self.preview_labels["center"].setText(_coordinate_text(center)
                                             if center else "—")
        self.preview_hint.setText(
            f"立方体 {float(side):g} mm · 面心 {len(preview.get('faces') or [])}"
            f" 个 · 候选点 {len(preview.get('candidates') or [])} 个 · "
            f"五点 {len(preview.get('points') or [])} 个（不跑 IK）。"
            + NUMBERING_NOTE)
        self.faces_edit.setPlainText(block_text(preview.get("faces")))
        self.candidates_edit.setPlainText(
            block_text(preview.get("candidates")))
        fill_preview_table(self.preview_table, preview)
        warnings = [str(line) for line in (preview.get("warnings") or [])]
        self.warning_label.setText("\n".join(warnings))
        self.warning_label.setVisible(bool(warnings))

    def _sync_buttons(self) -> None:
        """按“参数合不合法”启用「生成五点」（没有任务，不存在“正在跑”）。"""
        self.generate_button.setEnabled(bool((self.preview or {}).get("ok")))

    def apply_preview(self):
        """按当前预览重建图（参数非法时给说明图）。

        Returns:
            plotly ``Figure``（同时存进 ``last_figure``；挂了 PlotView 就推过去）。
        """
        if self._result_seq == self._change_seq and self.last_result:
            # 结果渲染之后表单没再变过：不许拿预览图把刚推上去的结果图盖掉
            return self.last_figure
        try:
            figure = preview_figure(self.preview or {})
        except Exception as error:          # noqa: BLE001 - 图坏了也不能崩窗口
            _LOGGER.exception("ISO 预览图渲染失败（预览=%r）", self.preview)
            figure = placeholder_figure(
                f"预览图渲染失败：{type(error).__name__}: {error}")
            self._report_status(f"预览图渲染失败：{type(error).__name__}: {error}")
        self.last_figure = figure
        if self._plot is not None:
            self._push_figure(figure)
        return figure

    # ------------------------------------------------------------ 算五点
    def start_iso(self):
        """算 P1~P5 的坐标并验算一次可达性（主线程，毫秒级；不建任务）。

        流程：预览不合法就只把报错原文写在卡片里、**不计算**；合法就在本线程
        调 ``core.iso_points``（几何 + 5 次 IK，实测 10 ms 量级），然后铺结果区。
        参数非法 / 算不出来时返回 ``None``，成功时返回 ``core.iso_points`` 的
        结果 dict（调用方判真假即可）。

        Returns:
            dict | None: 结果；没算（参数非法 / 异常）时为 ``None``。
        """
        preview = self.refresh_preview()
        if not preview.get("ok"):
            self._set_status(
                f"参数不合法，没有计算：{preview.get('error') or '预览失败'}",
                level="error")
            self._report_status("参数不合法，没有计算")
            return None

        params = form_to_params(self.values())
        self._reset_result_area()
        self._debounce.stop()       # 算完立刻出结果：别让陈旧预览盖上去
        started = time.monotonic()
        try:
            result = core.iso_points(params)
        except Exception as error:                  # noqa: BLE001 - 别抛穿槽
            _LOGGER.exception("core.iso_points 抛异常（params=%r）", params)
            result = {"ok": False,
                      "error": f"算五点失败：{type(error).__name__}: {error}",
                      "warnings": []}
        self.last_elapsed_ms = (time.monotonic() - started) * 1000.0

        self.last_result = result
        if not result.get("ok"):
            self.run_state = "failed"
            self._sync_buttons()
            error = result.get("error") or "未知错误"
            self._set_status(f"算不了：{error}", level="error")
            self._report_status(f"ISO 五点算不了：{error}")
            return None

        self.run_state = "done"
        self.render_result(result)
        summary = result.get("summary") or {}
        text = (f"{summary.get('reachable')}/{summary.get('total')} 点可达"
                f" · {self.last_elapsed_ms:.0f} ms")
        warnings = [str(line) for line in (result.get("warnings") or [])]
        if warnings:
            text += "\n备注：" + "；".join(warnings)
        self._set_status(text, level="ok" if summary.get("reachable") ==
                         summary.get("total") else "error")
        self._report_status(
            f"ISO 五点：可达 {summary.get('reachable')}/{summary.get('total')}")
        return result

    # ------------------------------------------------------------- 结果渲染
    def render_result(self, result: dict) -> None:
        """把 ``core.iso_points`` 的返回值铺到结果区（统计 / 路点表 / 图）。

        Args:
            result: ``core.iso_points`` 的成功返回值（或同构的构造 dict）。
        """
        self.last_result = result
        self._result_seq = self._change_seq
        self.stale_label.hide()          # 这一次的结果就是按当前参数算的
        fill_iso_stats(self.stat_labels, stats_view(result))
        fill_point_table(self.point_table, point_rows(result))
        self._update_exports(result)
        self.stats_holder.show()
        self.result_tabs.show()
        try:
            figure = result_figure(result)
        except Exception as error:                      # noqa: BLE001 - 别崩窗口
            _LOGGER.exception("ISO 结果图渲染失败（结果=%r）", result)
            figure = placeholder_figure(
                f"结果图渲染失败：{type(error).__name__}: {error}")
            self._report_status(f"结果图渲染失败：{type(error).__name__}: {error}")
        self.last_figure = figure
        if self._plot is not None:
            self._push_figure(figure)
        self.result_tabs.setCurrentWidget(self.points_page)
        self._expand_result_area()

    def _expand_result_area(self) -> None:
        """出结果后把结果区撑到"够看 ``RESULT_ROW_TARGET`` 行"（三段版本）。

        与生成页同一口径（实现见 ``generate_page.expand_result_area``）。
        """
        generate_page.expand_result_area(self)

    def _update_exports(self, result: dict) -> None:
        """按“有没有结果”启用导出按钮：产物**在点导出时**才生成。

        文件不再提前落到 ``runs/``：这一页没有后台任务，产物只在你真的要导出
        时写一次（``core.save_iso_points``，matplotlib 出图约 0.3 s）。
        """
        ready = bool((result or {}).get("ok"))
        for key, button in self.export_buttons.items():
            button.setEnabled(ready)
            button.setToolTip("" if ready else "还没有结果")

    def _on_export(self, key: str) -> None:
        """导出按钮：先落盘（一次），再选目标后**复制**（不移动）。

        Args:
            key: ``artifacts`` 里的键（``iso_json`` / ``plot``）。
        """
        result = self.last_result or {}
        if not result.get("ok"):
            self._set_status("还没有结果，无法导出", level="error")
            return
        artifacts = result.get("artifacts") or {}
        if not artifacts:
            try:
                saved = core.save_iso_points(result)
            except (OSError, ValueError) as error:
                self._set_status(f"无法写出产物：{type(error).__name__}: {error}",
                                 level="error")
                self._report_status(f"无法写出产物：{error}")
                return
            result = {**result, **saved}
            self.last_result = result
            self._report_status(f"已写出产物：{run_dir_text(saved.get('run_dir'))}")
        level, message = export_artifact(self.last_result or {}, key, EXPORTS,
                                         self)
        if not message:                     # 用户取消
            return
        if level == "error":
            self._set_status(message, level="error")
        self._report_status(message)

    def _reset_result_area(self) -> None:
        """算新一轮前清空结果区（上一次的表、图不留在界面上冒充新结果）。"""
        self.last_result = None
        self.last_figure = None
        self.point_table.setSortingEnabled(False)
        self.point_table.clearContents()
        self.point_table.setRowCount(0)
        self.point_table.setSortingEnabled(True)
        for label in self.stat_labels.values():
            label.setText("—")
            label.setStyleSheet("")
        self.stats_holder.hide()
        for button in self.export_buttons.values():
            button.setEnabled(False)
            button.setToolTip("")
        self.status_label.clear()
        self.status_label.hide()
        self.stale_label.clear()
        self.stale_label.hide()
        if self._plot is not None:
            self._push_figure(placeholder_figure("正在算五点…"))

    def _set_status(self, text: str, level: str = "info") -> None:
        """结果区状态行：``ok`` 绿 / ``error`` 红 / ``muted`` 灰 / ``info`` 正常。"""
        message = str(text or "")
        page_layout.fit_message(self.status_label, message, max_height=26)
        self.status_label.setStyleSheet(_STATUS_STYLES.get(level, ""))
        self.status_label.setVisible(bool(message.strip()))

    # ------------------------------------------------------------ 表格 ↔ 图
    def _connect_plot_link(self) -> None:
        """把五点表与图接起来（S6 P1）：两边都用点名（P1..P5）作键。"""
        self.point_table.itemSelectionChanged.connect(
            self._on_point_selected)
        signal = getattr(self._plot, "pointClicked", None)
        if signal is not None:
            signal.connect(self._on_plot_clicked)

    def _on_point_selected(self) -> None:
        """表格选中变化 → 图上放大那个点，状态栏说出点名。"""
        key = page_layout.table_key(self.point_table,
                                    self.point_table.currentRow())
        if key:
            self.select_key(key)

    def select_key(self, key, report: bool = True) -> None:
        """选中一个点：图上放大、表格行选中、状态栏显示点名。

        隐藏页不动作：三页都 ``connect`` 同一个 ``plot.pointClicked``，隐藏页
        跟着“选中 + 上报”会改写可见页的状态栏（S6 评审建议 2）。
        """
        if not self._active:
            return
        label = page_layout.gp_label(key)
        if report:
            self._report_status(f"已选中 {label}（图上已放大）")
        select = getattr(self._plot, "select_points", None)
        if callable(select):
            select([key])
        row = page_layout.find_row(self.point_table, key)
        if row >= 0 and row != self.point_table.currentRow():
            self.point_table.blockSignals(True)
            try:
                self.point_table.setCurrentCell(row, 0)
                self.point_table.scrollToItem(self.point_table.item(row, 0))
            finally:
                self.point_table.blockSignals(False)

    def _on_plot_clicked(self, keys) -> None:
        """图上点了点 → 表格选中它那一行（P1 的反向）。"""
        for key in (keys or []):
            if page_layout.find_row(self.point_table, key) >= 0:
                self.select_key(key)
                return

    def _report_status(self, message: str) -> None:
        """把一句话写到窗口状态栏（没有主窗口的纯逻辑测试里也能断言）。"""
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
        """切回本页时把表格当前行补回图上（S6 评审 R2c）。"""
        table = self.point_table
        row = table.currentRow()
        if row < 0:
            return
        key = page_layout.table_key(table, row)
        if key:
            self.select_key(key, report=False)

    def shutdown(self) -> None:
        """窗口关闭时收尾：停预览的去抖定时器（本页没有任务要注销）。"""
        self._debounce.stop()

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
            plotly ``Figure``；没出过图时给一张预览/说明图，而不是把生成页的
            图留在同一个 PlotView 上。
        """
        if self.last_figure is not None:
            return self.last_figure
        # 同一个状态下同一个对象：主窗口按**对象身份**判“这张图推过了”，
        # 每次现建一个新 Figure 就永远命不中，切回本页就会白白 react 一遍
        # （相机与高亮都跟着被重置）。状态一变（preview 换对象 / run_state 变）
        # 才算新图。
        cached = getattr(self, "_current_figure", None)
        if (cached is not None and cached[0] is self.preview
                and cached[1] == self.run_state):
            return cached[2]
        try:
            figure = preview_figure(self.preview or {})
        except Exception:               # noqa: BLE001 - 说明图也不该崩
            figure = placeholder_figure("还没有 ISO 五点结果。",
                                        title="ISO 五点图（未出图）")
        self._current_figure = (self.preview, self.run_state, figure)
        return figure

    def show_webgl_warning(self) -> None:
        """WebGL 不可用时在页面顶部挂一条中文建议（与其它两页同一段文案）。"""
        self.banner.setText(
            "⚠ 这台机器的 WebGL 不可用，Plotly 3D 画不出来。\n"
            "　处理办法：① 更新显卡驱动；② 远程桌面下改在本机打开；"
            "③ 先用网页版（python -m laser_web）看图，功能与数值一致。")
        self.banner.setStyleSheet(
            "background:#fff4e5; color:#8a4b00; padding:8px; border:1px solid #f0c48a;")
        self.banner.show()
