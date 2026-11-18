# -*- coding: utf-8 -*-
"""ISO 9283 五点页（S5）：立方体来源 / 布点 / 姿态 → 后台解算 → 统计 / 3D / 表 / 导出。

行为基准是网页版 ``laser_web/pages/iso9283.py``（513 行）：字段、默认值、两种
立方体来源的互斥、``square_frac`` 的启用条件、``_point_rows`` 的口径、产物
下载项都**照它搬**。本模块只做编排与展示：

1. **收参数**：:func:`form_to_params` 的键集合与网页版 ``collect()`` 逐键一致
   （两种 ``cube_mode`` 各自只提交激活那一组的键）；
2. **预览**：改任何参数都在 :data:`DEBOUNCE_MS`（300 ms）后重算几何
   （:func:`preview_geometry`）——立方体边长 / 边心距、6 个面心、9 个候选点
   （8 顶点 + 中心）、最终 5 点与 ``layout`` 标注。**切换立方体来源也算一次
   改动**，卡片 / 预览页签当场换成新来源那一套（否则会挂着上一种来源的边长）。
   **不跑 IK**，所以解算结果不会因为预览而被“预演”一遍；
3. **解算**：``bridge.submit("iso", lambda job: core.run_iso9283(params, job=job,
   on_progress=job.set_progress))``——进度 / 日志 / 取消 / 终态全部走
   :class:`laser_desktop.job_bridge.JobBridge` 的三条信号（与 S3 / S4 同一套
   线程模型）；
4. **结果区**：``QTabWidget`` 三页签——路点（P1~P5 表）/ 预览（几何）/ 日志；
   上方统计行是「IK 是否全部成功 / |J5|min / 各点构型」；
5. **导出**：``iso.json`` / ``iso.png`` 从 ``run_dir`` **复制**（文件名带
   ``<run_id>-`` 前缀），与生成页 / 修复页同一个实现
   （:func:`laser_desktop.generate_page.export_artifact`）。

校验与语义**全部**走 ``core``：本页不写任何判定规则，``{"ok": False,
"error": ...}`` 时把报错原文用红字写在结果区状态行，**不弹窗**。

关于预览用到的两个 ``core`` 私有函数（见报告「与规格的偏差」）
------------------------------------------------------------
``core`` 只有 ``run_iso9283``（含 IK + 落盘），**没有** public 的
``preview_iso``。为了不把「立方体怎么定 / 参数怎么归一」在界面里再写一遍，
预览直接复用 ``core._iso_options``（参数归一与取值域检查）与
``core._iso_cube``（``bounds`` 模式的中心/边长）；两者都是 ``run_iso9283``
内部走同一条路径的函数，界面只是提前调用、不复制逻辑。``auto`` 模式必须
逐候选试 IK 才知道边长，所以预览只显示搜索范围，边长/中心由真跑给出。
"""

from __future__ import annotations

import contextlib
import io
import logging
from pathlib import Path

from excitation_trajectory import iso9283_cube as iso
from laser_web import core, ui_state

from . import form_widgets, generate_page, page_layout, paths
from .generate_page import (
    LOG_WIDGET_LIMIT,
    PROGRESS_BAR_HEIGHT,
    PROGRESS_BAR_WIDTH,
    PROGRESS_STEPS,
    STATE_TEXT,
    UNIT_DEG,
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
from .job_bridge import LOG_LIMIT as JOB_LOG_LIMIT
from .job_bridge import JobBridge, adopt_running_job
from .qt_compat import Qt, QtCore, QtGui, QtWidgets

#: 日志器（``core`` 万一抛异常、出图失败都记在这里）。
_LOGGER = logging.getLogger(__name__)

#: 立方体来源 toggle（**值与网页版 ``_CUBE_MODES`` 相同**，文案照抄）。
CUBE_MODES = {"bounds": "给最大/最小点", "auto": "自动搜索内接"}
#: 来源顺序 = 面板顺序（初始化与 ``switch_mode`` 都按它算下标）。
CUBE_MODE_ORDER = ("bounds", "auto")
#: 布点方式（键来自 ``iso9283_cube.LAYOUTS``，文案照抄网页版 ``_LAYOUTS``）。
LAYOUTS = {"vertices": "对角截面四角（ISO 标准）",
           "circle": "内切圆（旧实现）",
           "center-square": "原脚本布点"}
#: 对角截面（键来自 ``iso9283_cube.PLANES``，文案照抄网页版 ``_PLANES``）。
PLANES = {"xz": "XZ 对角面（x=z）", "yz": "YZ 对角面（y=z）",
          "xy": "XY 对角面（x=y）"}
#: 自动搜索的示例搜索箱（照抄网页版 ``_SAMPLE_SEARCH_BOX``：x 朝机身外侧留开）。
SAMPLE_SEARCH_BOX = (700.0, 1400.0, -600.0, 600.0, 200.0, 1300.0)

#: 预览卡片里的五项数值（键, 名称）：由 :class:`page_layout.MetricGrid` 按需换行。
PREVIEW_METRICS = (("cube", "立方体"), ("apothem", "边心距 L/2"),
                   ("center", "中心"), ("layout", "布点"),
                   ("plane", "截面"))

#: 五点的姿态完全相同，现场只能用直线运动连接（网页版同一句话）。
MOVL_NOTE = "这 5 点的姿态完全相同，控制器用 MOVL 走直线（不要用 PTP）。"
#: 网页版「工具」页的提示（``iso.json`` 与 ``scan.json`` 同构）。
GP_HINT = ("可到「工具」页用这个 JSON 直接导 GP：iso.json 与 scan.json 同构"
           "（meta / waypoints / perturbed_ik）。")

#: 路点表列头（与网页版 ``_POINT_COLUMNS`` 一致：点名 + XYZ + q1..q6 +
#: |J3| / |J5| / cfg / ok）。
POINT_HEADERS = ("P", "X (mm)", "Y (mm)", "Z (mm)",
                 "q1", "q2", "q3", "q4", "q5", "q6",
                 "|J3|°", "|J5|°", "cfg", "ok")

#: 预览里 6 个面心的标签（顺序 = X−/X+/Y−/Y+/Z−/Z+）。
FACE_LABELS = ("X− 面心", "X+ 面心", "Y− 面心", "Y+ 面心",
               "Z− 面心", "Z+ 面心")

#: 导出按钮：``(artifacts 键, 按钮文案, 文件对话框过滤器)``。键来自
#: ``core.run_iso9283`` 的 ``artifacts``——ISO 运行**只落** ``iso.json`` 与
#: ``iso.png``（``core`` 不给 ISO 写 ``joints.txt``，``iso5.json`` 也只是 CLI
#: 的 ``--out`` 默认名）。所以这里导出的是真实存在的两个产物，文件名带
#: ``<run_id>-`` 前缀（:func:`laser_desktop.generate_page.export_artifact`）。
EXPORTS = (("iso_json", "iso.json", "JSON (*.json);;所有文件 (*)"),
           ("plot", "iso.png", "PNG 图片 (*.png);;所有文件 (*)"))

#: 结果页签的最小高度（px）：不许随"当前点开哪个页签"变（见 ``page_layout`` 的
#: 说明）。260 ≈ 页签条 + 表 200 + 一行说明。
TABS_MIN_HEIGHT = 262

#: 三个页签（规格：路点 / 预览 / 日志）。
TABS = ("路点", "预览", "日志")

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

#: 结果区状态行里运行目录的简写（``runs/<id>``）：完整路径放进 tooltip，
#: 状态行只留一行——它每多一行，路点表就少露一行。
def run_dir_text(run_dir) -> str:
    """``/…/runs/<id>`` -> ``runs/<id>``（空值给 ``—``）。"""
    if not run_dir:
        return "—"
    path = Path(str(run_dir))
    return f"{path.parent.name}/{path.name}"


#: 统计行标签的键（顺序即界面顺序；单测按这些键取值）。
#: 边长与中心分成两个标签：合成一行会折行，把下面的路点表挤到只剩表头。
STAT_KEYS = ("solved", "j5", "j3", "config", "configs", "side", "center")

#: ``core`` 里的表单异常类型（``_FormError`` 是私有的，取不到就退回 ValueError，
#: 保证 ``{"ok": False, "error": ...}`` 的语义不变——见模块 docstring）。
_FORM_ERROR = getattr(core, "_FormError", ValueError)


# ---------------------------------------------------------------------------
# 纯逻辑（不碰 Qt，单测直接调）
# ---------------------------------------------------------------------------
def iso_form_defaults() -> dict:
    """ISO 表单的默认值：``core.iso_defaults()``（CLI 默认值）+ 网页版示例域。

    只有「立方体来源」的两组输入取网页版的示例值：``bounds`` 用
    ``ui_state.SAMPLE_POINT_MIN/MAX``（与「生成」页的示例域同源，550 mm 立方体
    就是站点参数 ``center 1025 0 605 / side 550``），``auto`` 用
    :data:`SAMPLE_SEARCH_BOX`。其余项**一个数字都不抄**，全部问 ``core``。

    Returns:
        dict: 键与 :func:`form_to_params` 的入参一致。
    """
    defaults = core.iso_defaults()
    low, high = ui_state.SAMPLE_POINT_MIN, ui_state.SAMPLE_POINT_MAX
    return {
        "robot": ui_state.default_robot(),
        "cube_mode": CUBE_MODE_ORDER[0],
        # 两点输入：最小点 + 最大点（x0 x1 y0 y1 z0 z1）
        "cube": [float(low[0]), float(high[0]), float(low[1]),
                 float(high[1]), float(low[2]), float(high[2])],
        "search_box": [float(value) for value in SAMPLE_SEARCH_BOX],
        "resolution": float(defaults["resolution"]),
        "prefer_x_min": defaults["prefer_x_min"],
        "layout": str(defaults["layout"]),
        "plane": str(defaults["plane"]),
        "square_frac": float(defaults["square_frac"]),
        "rpy": [float(value) for value in defaults["rpy"]],
        "euler": str(defaults["euler"]),
        "q_seed": str(defaults["q_seed"]),
        "j5_min": float(defaults["j5_min"]),
    }


def form_to_params(values: dict) -> dict:
    """表单值 -> ``core.run_iso9283`` 的入参（**只提交激活那一组**）。

    键集合与网页版 ``pages/iso9283.py::collect()`` 逐一对应（单测里逐个写死）：

    * 两种模式都提交：``robot`` / ``cube_mode`` / ``layout`` / ``plane`` /
      ``square_frac`` / ``rpy`` / ``euler`` / ``q_seed`` / ``j5_min``；
    * ``cube_mode="bounds"`` 额外提交 ``cube``（6 个数：x0 x1 y0 y1 z0 z1）；
    * ``cube_mode="auto"`` 额外提交 ``search_box`` / ``resolution`` /
      ``prefer_x_min``。

    互斥规则本身不在这里判断（``core._iso_options`` 才是唯一的校验入口），本
    函数只决定“这一组控件此刻算不算激活”。

    Args:
        values: 表单值，缺省项按 :func:`iso_form_defaults` 补齐。

    Returns:
        dict: 可直接喂 ``core.run_iso9283``。
    """
    values = {**iso_form_defaults(), **(values or {})}
    mode = str(values["cube_mode"])
    if mode not in CUBE_MODES:                  # 只可能是界面值出错：退回默认
        mode = CUBE_MODE_ORDER[0]
    params = {
        "robot": values["robot"],
        "cube_mode": mode,
        "layout": str(values["layout"]),
        "plane": str(values["plane"]),
        "square_frac": float(values["square_frac"]),
        "rpy": [float(value) for value in values["rpy"]],
        "euler": str(values["euler"]),
        "q_seed": str(values["q_seed"]),
        "j5_min": float(values["j5_min"]),
    }
    if mode == CUBE_MODE_ORDER[0]:              # 两点输入
        params["cube"] = [float(value) for value in values["cube"]]
    else:                                       # 自动搜索
        params["search_box"] = [float(value) for value in values["search_box"]]
        params["resolution"] = float(values["resolution"])
        prefer = values["prefer_x_min"]
        params["prefer_x_min"] = (None if prefer is None
                                  else float(prefer))
    return params


def layout_label(value) -> str:
    """布点方式 -> 中文文案（未知值原样显示，不吞信息）。"""
    return LAYOUTS.get(str(value), str(value))


def plane_label(value) -> str:
    """对角截面 -> 中文文案（未知值原样显示）。"""
    return PLANES.get(str(value), str(value))


def point_layout_label(name: str, layout: str, plane: str,
                       square_frac: float) -> str:
    """单个点的 ``layout`` 标注（预览表里“该点属于哪种布点”）。

    文案描述的是 ``iso9283_cube.cube_points`` docstring 里写明的几何，不参与
    任何计算：P1 永远是立方体中心，P2~P5 随布点方式不同。

    Args:
        name: ``P1`` ~ ``P5``。
        layout: ``vertices`` / ``circle`` / ``center-square``。
        plane: 对角截面（只影响 ``vertices`` / ``circle``）。
        square_frac: ``center-square`` 的 k。

    Returns:
        str: 给现场看的一句话。
    """
    if str(name) == iso.POINT_NAMES[0]:
        return "立方体中心"
    if str(layout) == "vertices":
        return f"对角截面四角 · {plane_label(plane)}"
    if str(layout) == "circle":
        return f"内切圆 90° · {plane_label(plane)}"
    if str(layout) == "center-square":
        return f"z = −{float(square_frac):g}·h 正方形角（k={float(square_frac):g}）"
    return str(layout)                  # 未知布点：原样显示，不编造


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
    ``core._iso_cube`` 定 ``bounds`` 模式的中心/边长，最后用
    ``iso9283_cube.cube_points`` 出五点。任何校验失败都转成
    ``{"ok": False, "error": 报错原文}``（原文来自 ``core``，界面不改写）。

    ``cube_mode="auto"`` 时边长/中心必须真跑（要逐个候选试 IK）才知道，所以
    只回搜索范围，其余几何字段留空。

    Args:
        values: 表单值（缺省项按 :func:`iso_form_defaults` 补齐）。
        capture_stdout: 是否把 ``core._iso_cube`` 的那句 ``print`` 提示抓进
            ``warnings``。**任务在跑时必须传 ``False``**：``redirect_stdout``
            换的是进程级的 ``sys.stdout``，工作线程的日志会一起被吞掉（详见
            下面 ``redirect_stdout`` 那段注释）。抓不到只是少一行提示，
            边长 / 中心 / 五点全部来自返回值，一个数都不受影响。

    Returns:
        dict: ``{"ok", "mode", "layout", "plane", "square_frac", "center",
        "side", "apothem", "faces", "candidates", "points", "warnings",
        "error"}``；``points`` 的每一项是 ``{"name", "pos", "label"}``。
    """
    params = form_to_params(values)
    try:
        options = core._iso_options(params)
    except _FORM_ERROR as error:                    # 参数域错误：原文回显
        return {"ok": False, "error": str(error), "warnings": []}
    except (TypeError, ValueError) as error:
        return {"ok": False, "error": f"参数无法解析：{error}", "warnings": []}

    preview = {"ok": True,
               "mode": str(options["cube_mode"]),
               "layout": str(options["layout"]),
               "plane": str(options["plane"]),
               "square_frac": float(options["square_frac"]),
               "center": None, "side": None, "apothem": None,
               "faces": [], "candidates": [], "points": [],
               "search_box": None, "resolution": None,
               "warnings": [], "error": None}
    if preview["mode"] == CUBE_MODE_ORDER[1]:       # 自动搜索：没有立方体可画
        cube = options["cube"]
        preview["search_box"] = [float(value) for value in cube["search_box"]]
        preview["resolution"] = float(cube["resolution"])
        preview["warnings"].append(
            "自动搜索：边长与中心要真跑一次才知道（逐个候选试 IK），"
            "预览只显示搜索范围。")
        return preview

    hint = ""
    try:
        # ``bounds`` 分支不碰 rbt / order / extrinsic，传 None 是安全的；
        # ``auto`` 分支已在上面提前返回。
        # ``core._iso_cube`` 在“三轴跨度不等”时会 ``print`` 一句提示（CLI 的
        # 行为）；桌面版把那段 stdout 抓下来当预览提示——提示文字的来源仍然
        # 是 core，界面不另写一份判据。
        #
        # ``contextlib.redirect_stdout`` 换的是进程级的 ``sys.stdout``，而预览
        # 在 ``_on_change`` 里同步跑：任务运行中拖一下 spinbox 就有概率把工作
        # 线程正在 ``print`` 的日志行吞进这个 buffer（日志页于是少几行）。所以
        # 只在**没有任务在跑**时才抓。更好的修法是 ``core`` 那侧返回告警而不是
        # ``print``，但那不在这一轮的改动范围里。
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
         "label": point_layout_label(name, preview["layout"], preview["plane"],
                                     preview["square_frac"])}
        for name in iso.POINT_NAMES]
    return preview


def point_rows(result: dict) -> list[dict]:
    """``run_iso9283()["points"]`` -> 表格行（保留数字类型，失败行标红）。

    与网页版 ``_point_rows`` 同一约定：数值不转字符串（表头要按大小排序），
    ``None`` 由 :func:`format_metric` 画成 ``—``。
    """
    rows = []
    for point in (result or {}).get("points") or []:
        joints = list(point.get("q") or [None] * 6)
        joints += [None] * (6 - len(joints))
        config = point.get("config")
        row = {"name": str(point.get("name") or "?"),
               "x": _round(point.get("pos", [None, None, None])[0], 1),
               "y": _round(point.get("pos", [None, None, None])[1], 1),
               "z": _round(point.get("pos", [None, None, None])[2], 1),
               "cfg": ("—" if config is None
                       else ",".join(str(int(tag)) for tag in config)),
               "ok": "✓" if point.get("ok") else "✗",
               "failed": not bool(point.get("ok")),
               "j3": _round(point.get("j3_abs"), 3),
               "j5": _round(point.get("j5_abs"), 3)}
        for index, key in enumerate(("q1", "q2", "q3", "q4", "q5", "q6")):
            row[key] = _round(joints[index], 3)
        rows.append(row)
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
    """五点表：P / X / Y / Z / q1..q6 / |J3| / |J5| / cfg / ok；失败行红底。

    Args:
        table: 列头为 :data:`POINT_HEADERS` 的 ``QTableWidget``。
        rows: :func:`point_rows` 的返回值。
    """
    table.setSortingEnabled(False)
    table.clearContents()
    table.setRowCount(len(rows))
    for row_index, row in enumerate(rows):
        ok = bool(row.get("ok") == "✓")
        cells = [
            ((0, row["name"]), row["name"], "center"),
            (sort_key(row["x"]), format_metric(row["x"], 1), "right"),
            (sort_key(row["y"]), format_metric(row["y"], 1), "right"),
            (sort_key(row["z"]), format_metric(row["z"], 1), "right"),
        ]
        cells += [(sort_key(row[f"q{axis}"]),
                   format_metric(row[f"q{axis}"], 3), "right")
                  for axis in range(1, 7)]
        cells += [
            (sort_key(row.get("j3")), format_metric(row.get("j3"), 3), "right"),
            (sort_key(row.get("j5")), format_metric(row.get("j5"), 3), "right"),
            ((0, row["cfg"]), row["cfg"], "left"),
            ((0, 1 if ok else 0), row["ok"], "center"),
        ]
        for column, (key, text, align) in enumerate(cells):
            item = SortItem(key, text)
            item.setTextAlignment(
                {"right": Qt.AlignRight | Qt.AlignVCenter,
                 "center": Qt.AlignCenter,
                 "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
            if not ok:                       # 失败整行标红（排序后也不丢）
                item.setBackground(QtGui.QColor(generate_page._FAILED_BG))
                item.setForeground(QtGui.QColor(generate_page._FAILED_FG))
            table.setItem(row_index, column, item)
    table.setSortingEnabled(True)
    if rows:
        table.sortItems(0, Qt.AscendingOrder)
    table.resizeColumnsToContents()


def fill_preview_table(table, preview) -> None:
    """预览表：P / X / Y / Z / ``layout`` 标注（5 行，不排序）。"""
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


def fill_iso_stats(labels, stats) -> None:
    """统计行：IK 成功 n/N / |J5|min / |J3|min / 构型一致 / 构型标签 / 立方体。

    Args:
        labels: ``{STAT_KEYS 里的键: QLabel}``。
        stats: ``core.run_iso9283()["stats"]``。
    """
    stats = stats or {}
    solved = int(stats.get("solved") or 0)
    total = int(stats.get("total") or 0)
    labels["solved"].setText(f"IK 成功 {solved}/{total}")
    labels["solved"].setStyleSheet(
        _GREEN_BOLD if total and solved == total else _RED_BOLD)
    labels["j5"].setText(f"|J5|min {format_metric(stats.get('j5_abs_min'))}°")
    labels["j3"].setText(f"|J3|min {format_metric(stats.get('j3_abs_min'))}°")
    consistent = stats.get("config_consistent")
    if consistent is None:
        labels["config"].setText("—")
        labels["config"].setStyleSheet("")
    else:
        labels["config"].setText("构型 一致" if consistent else "构型 不一致")
        labels["config"].setStyleSheet(
            _GREEN_BOLD if consistent else _RED_BOLD)
    configs = stats.get("configs") or []
    labels["configs"].setText(
        "构型标签 " + (" / ".join(",".join(str(int(tag)) for tag in row)
                                  for row in configs) if configs else "—"))
    side = stats.get("cube_side")
    center = stats.get("cube_center")
    labels["side"].setText(
        "边长 —" if side is None else f"边长 {format_metric(side, 0)} mm")
    labels["center"].setText(
        "中心 —" if not center
        else "中心 (" + ", ".join(f"{float(value):.0f}" for value in center) + ") mm")


def preview_figure(preview: dict):
    """预览图：立方体 12 条棱 + 9 个候选点 + 最终 5 点（**不跑 IK**）。

    与 ``laser_web.figures.iso_cube_figure`` 的区别只有一处：那份按 ``ok``
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
        reason = ("自动搜索：点「生成五点」后才知道立方体位置"
                  if preview.get("mode") == CUBE_MODE_ORDER[1]
                  else "还没有可以画的点")
        return placeholder_figure(reason, title="ISO 五点预览（未出图）")

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
        title=f"ISO 9283 五点预览：{layout_label(preview['layout'])} · "
              f"{plane_label(preview['plane'])}",
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

    ``laser_web.figures`` 是网页版共用资产（本片不改它），它没写 ``customdata``；
    桌面版要“点图 → 选中表格行”，所以在桌面层按 trace 名补上。trace 只有三
    种：立方体棱线（没有键，点它不联动）、可解点、无解点。

    Args:
        figure: :func:`laser_web.figures.iso_cube_figure` 的结果（就地改）。
        points: ``core.run_iso9283()["points"]``。

    Returns:
        plotly ``Figure``: 同一个对象（便于链式调用）。
    """
    rows = [row for row in (points or []) if row.get("pos")]
    good = [[str(row.get("name"))] for row in rows if row.get("ok")]
    bad = [[str(row.get("name"))] for row in rows if not row.get("ok")]
    for trace in figure.data or []:
        name = str(getattr(trace, "name", "") or "")
        if name.startswith("可解"):
            trace.customdata = good
        elif name.startswith("无解"):
            trace.customdata = bad
    return figure


def result_figure(result: dict):
    """结果图：复用网页版 :func:`laser_web.figures.iso_cube_figure`（同一份代码）。

    Args:
        result: ``core.run_iso9283`` 的返回值。

    Returns:
        plotly ``Figure``（点 trace 带 ``customdata[0] = P1..P5``）。
    """
    from laser_web import figures

    stats = (result or {}).get("stats") or {}
    figure = figures.iso_cube_figure(
        result.get("points") or [],
        cube_center=stats.get("cube_center"),
        side=stats.get("cube_side"),
        layout=str(stats.get("layout") or ""),
        plane=str(stats.get("plane") or ""),
        height=520)
    return attach_point_keys(figure, result.get("points") or [])


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class IsoPage(QtWidgets.QWidget):
    """ISO 五点页：左侧表单 + 右侧三段（预览卡 / 图 / 三页签结果区）。

    Attributes:
        fields: ``{键: 控件}``，取值与 :func:`form_to_params` 的入参同名。
        mode_stack: 立方体来源的两组输入（``bounds`` / ``auto``，永远恰好显示
            一组——初始化按 radio 的**当前值**切，不写死模式名）。
        stat_labels: 结果区统计行（键见 :data:`STAT_KEYS`）。
        point_table: 路点表（P1~P5，失败行标红）。
        preview_table: 预览页签里的 5 点表（含 ``layout`` 标注）。
        faces_edit / candidates_edit: 预览页签的 6 个面心 / 9 个候选点文本块。
        export_buttons: ``{artifacts 键: 按钮}``，有产物才可用。
        preview: 最近一次的 :func:`preview_geometry` 结果。
        last_result: 最近一次 ``core.run_iso9283`` 的返回值（含 ``ok=False``）。
        last_status: 最近一次写到状态栏的文字（无主窗口时也能断言）。
        run_state: ``idle`` / ``running`` / ``done`` / ``failed`` / ``cancelled``。
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
        self.fields = {}
        self.mode_buttons = []
        self.stat_labels = {}
        self.export_buttons = {}
        self.preview = None
        self.last_result = None
        self.last_figure = None
        self.last_status = ""
        self.run_state = "idle"
        self.last_job_id = None
        self._busy = False
        # 表单改动序号 / 出结果时的改动序号：相等说明“结果渲染后没人再改表单”，
        # 这时到点的预览去抖不许拿预览图盖掉结果图（与生成页同一道保护）。
        self._change_seq = 0
        self._result_seq = -1
        self.bridge = bridge if bridge is not None else JobBridge(parent=self)
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

        body = QtWidgets.QWidget()
        outer = QtWidgets.QHBoxLayout(body)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)
        outer.addWidget(self._build_left_column(), 0)
        outer.addWidget(self._build_right_column(), 1)
        column.addWidget(body, 1)

        # 信号连在控件建完之后（槽里直接碰控件，都在 GUI 线程，见 job_bridge）
        self.bridge.progress.connect(self._on_progress)
        self.bridge.log_line.connect(self._on_log_line)
        self.bridge.finished.connect(self._on_finished)
        self._connect_plot_link()
        self.refresh_preview()
        self._resume_running_job()

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
            "起一条后台任务：定立方体 → 生成五点 → 解算 IK → 出图 → 写 runs/<id>/")
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
        """左侧表单：模型 / 立方体来源 / 布点与截面 / 参考姿态 / 种子与告警。"""
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
        layout.addWidget(self._build_layout_group(defaults))
        layout.addWidget(form_widgets.build_pose_group(self.fields, defaults,
                                                      self._on_change))
        layout.addWidget(self._build_seed_group(defaults))
        layout.addStretch(1)

        scroll = generate_page._form_scroll(content,
                                            generate_page.FORM_MAX_WIDTH)
        self.form_scroll = scroll                # 冒烟/单测要查横滚条（P4）
        return scroll

    def _build_cube_group(self, defaults) -> QtWidgets.QGroupBox:
        """立方体来源：两点输入 / 自动搜索（两组互斥，永远恰好显示一组）。"""
        group = QtWidgets.QGroupBox("立方体来源")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)

        buttons = QtWidgets.QButtonGroup(group)
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        for index, mode in enumerate(CUBE_MODE_ORDER):
            button = QtWidgets.QRadioButton(CUBE_MODES[mode])
            buttons.addButton(button, index)
            self.mode_buttons.append(button)
            self.fields[f"cube_{mode}"] = button
            row.addWidget(button)
        row.addStretch(1)
        layout.addLayout(row)

        self.mode_stack = QtWidgets.QStackedWidget()
        self.bounds_panel = self._build_bounds_panel(defaults)
        self.auto_panel = self._build_auto_panel(defaults)
        self.mode_stack.addWidget(self.bounds_panel)
        self.mode_stack.addWidget(self.auto_panel)
        layout.addWidget(self.mode_stack)

        self.mode_buttons[CUBE_MODE_ORDER.index(defaults["cube_mode"])].setChecked(
            True)
        # 槽必须连在 ``setChecked`` **之后**：初始化那一下也会发 ``toggled``，而
        # 那一刻后面的分组（布点 / 姿态 / 种子）还没建出来，``_on_mode_switch``
        # 里的 ``refresh_preview`` 会 ``KeyError: 'layout'``。
        for button in self.mode_buttons:
            button.toggled.connect(self._on_mode_switch)
        return group

    def _build_bounds_panel(self, defaults) -> QtWidgets.QWidget:
        """「两点输入」：最小点 + 最大点（各 X/Y/Z 三个数，整数 mm 步进）。"""
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

    def _build_auto_panel(self, defaults) -> QtWidgets.QWidget:
        """「自动搜索」：搜索箱六个界 + 步长 + 中心 X 下限。"""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        box = list(defaults["search_box"])
        for index, axis in enumerate("XYZ"):
            low = _double_spin(-10000.0, 10000.0, box[2 * index], decimals=1,
                               step=1.0)
            high = _double_spin(-10000.0, 10000.0, box[2 * index + 1],
                                decimals=1, step=1.0)
            low.valueChanged.connect(self._on_change)
            high.valueChanged.connect(self._on_change)
            self.fields[f"box_{axis.lower()}0"] = low
            self.fields[f"box_{axis.lower()}1"] = high
            layout.addWidget(_inline([(f"搜索 {axis} 下界", low),
                                      (f"搜索 {axis} 上界", high)]))

        resolution = _double_spin(1.0, 1000.0, defaults["resolution"],
                                  decimals=1, step=10.0)
        resolution.valueChanged.connect(self._on_change)
        self.fields["resolution"] = resolution
        prefer = defaults["prefer_x_min"]
        # 可空文本框：**空 = 不限**（``None``）。早先用 spinbox 的 0 当哨兵，
        # 于是「中心 X ≥ 0」这个合法要求无法表达（0 被读成“不限”）。
        prefer_box = QtWidgets.QLineEdit(
            "" if prefer is None else f"{float(prefer):g}")
        prefer_box.setPlaceholderText("留空 = 不限（mm）")
        prefer_box.setToolTip("只接受中心 X 不小于该值的立方体；留空表示不设下限")
        prefer_box.textChanged.connect(self._on_change)
        self.fields["prefer_x_min"] = prefer_box
        layout.addWidget(_inline([("搜索步长 (mm)", resolution),
                                  ("中心 X 不小于 (mm)", prefer_box)]))
        layout.addWidget(_caption(
            "按 8 个顶点都可解来判可行，逐箱二分边长；步长越小越慢（默认 100 mm）。"))
        return panel

    def _build_layout_group(self, defaults) -> QtWidgets.QGroupBox:
        """布点与截面：``layout`` / ``plane`` / ``square_frac``（后者按布点门禁）。"""
        group = QtWidgets.QGroupBox("布点与截面")
        form = QtWidgets.QFormLayout(group)

        layout_box = QtWidgets.QComboBox()
        for value, label in LAYOUTS.items():
            layout_box.addItem(label, value)
        index = layout_box.findData(defaults["layout"])
        layout_box.setCurrentIndex(index if index >= 0 else 0)
        layout_box.currentIndexChanged.connect(
            lambda *_args: self.switch_layout())
        self.fields["layout"] = layout_box
        form.addRow("布点", layout_box)

        plane_box = QtWidgets.QComboBox()
        for value, label in PLANES.items():
            plane_box.addItem(label, value)
        index = plane_box.findData(defaults["plane"])
        plane_box.setCurrentIndex(index if index >= 0 else 0)
        plane_box.currentIndexChanged.connect(self._on_change)
        self.fields["plane"] = plane_box
        form.addRow("对角截面", plane_box)

        square = _double_spin(0.01, 1.0, defaults["square_frac"], suffix="",
                             decimals=2, step=0.1)
        square.valueChanged.connect(self._on_change)
        self.fields["square_frac"] = square
        form.addRow("center-square 的 k", square)

        # 初始化也要按**当前布点**门禁（不写死任何一种布点），但不能在这里
        # 触发整页刷新：此刻后面的分组（姿态 / 种子）还没建出来。
        self._apply_square_gate()
        return group

    def _build_seed_group(self, defaults) -> QtWidgets.QGroupBox:
        """IK 种子与 |J5| 告警：五点用同一姿态，种子只影响选哪组解。"""
        group = QtWidgets.QGroupBox("IK 种子与告警")
        form = QtWidgets.QFormLayout(group)
        seed = QtWidgets.QLineEdit(str(defaults["q_seed"]))
        seed.setPlaceholderText("6 个关节角，逗号分隔（度）；留空用 CLI 默认值")
        seed.textChanged.connect(self._on_change)
        self.fields["q_seed"] = seed
        form.addRow("q-seed", seed)

        j5_min = _double_spin(0.0, 90.0, defaults["j5_min"], UNIT_DEG,
                              decimals=1, step=1.0)
        j5_min.setToolTip("|J5| 低于它就在结果里告警（0 = 只报告不告警）")
        j5_min.valueChanged.connect(self._on_change)
        self.fields["j5_min"] = j5_min
        form.addRow("|J5| 告警下限", j5_min)
        form.addRow(_caption(MOVL_NOTE))
        return group

    # ------------------------------------------------------------ 预览卡片
    def _build_preview_card(self) -> QtWidgets.QGroupBox:
        """预览卡片：立方体 / 边心距 / 中心 / 布点 / 截面 + 报错原文 + 提示。

        五项用 :class:`laser_desktop.page_layout.MetricGrid`（按需换行，一行
        放不下就换行）：1600×950 下两行、1280×800 下三行，**都不会横向裁字**。
        S6b 是 ``QFormLayout`` + ``_inline``（一对占上下两行、一行 52 px），
        卡片要 289 px——压在图上边，就是图被挤掉的那一截。

        「生成五点」在左栏底部的操作行里（S6c：它原来在表单**滚动区**最底下，
        内容一多就滚出视野——离屏实测 ``visibleRegion()`` 是空的）。说明文字走
        tooltip：``MOVL_NOTE`` 在左侧「IK 种子与告警」组里已经完整写过一遍。
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

        # 说明走 tooltip（与生成页 / 修复页同一份做法）：1600×950 的真实窗口
        # 里预览卡只有 ~148 px，而 core 的提示行（三轴跨度不等时）还要占两行；
        # 再多一行说明就得滚一下才看得到「截面」。
        # ``MOVL_NOTE`` 在左侧「IK 种子与告警」组里已经完整写过。
        group.setToolTip(
            MOVL_NOTE + "改任何一项都会重算预览（面心 / 候选点 / 五点都在"
            "下面的「预览」页签里）。")
        return group

    def _build_result_area(self) -> QtWidgets.QGroupBox:
        """结果区：状态行 + 进度/取消 + 统计行 + 三页签（路点/预览/日志）+ 导出。"""
        group = QtWidgets.QGroupBox("结果")
        self.result_area = group        # expand_result_area 要量"表以外占多高"
        layout = QtWidgets.QVBoxLayout(group)
        _compact_result_layout(layout)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.status_label.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                        QtWidgets.QSizePolicy.Ignored)
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, PROGRESS_STEPS)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("空闲")
        # 进度条固定矮一点：它连同取消按钮占 33 px，而结果区的最小高度直接
        # 决定窗口最小高度（1280×800 要装得下三段）。
        self.progress_bar.setMaximumHeight(PROGRESS_BAR_HEIGHT)
        self.progress_bar.setFixedWidth(PROGRESS_BAR_WIDTH)
        self.cancel_button = QtWidgets.QPushButton("取消")
        self.cancel_button.setToolTip(
            "只在阶段边界生效：当前阶段跑完就停，不会硬打断 IK")
        self.cancel_button.clicked.connect(self._on_cancel_clicked)
        self.cancel_button.hide()
        # 状态 + 进度 + 取消同占一行（S6c，与生成页 / 修复页同一份做法）。
        status_row = QtWidgets.QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.setSpacing(6)
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.progress_bar, 0)
        status_row.addWidget(self.cancel_button, 0)
        layout.addLayout(status_row)
        self.status_label.hide()

        # 参数改完但没重跑：灰字点明“下表是上一次运行的”，就压在被提示的表上面
        self.stale_label = QtWidgets.QLabel("")
        self.stale_label.setWordWrap(True)
        self.stale_label.setStyleSheet(_STATUS_STYLES["muted"])
        self.stale_label.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                       QtWidgets.QSizePolicy.Ignored)
        self.stale_label.hide()
        layout.addWidget(self.stale_label)
        # 统计卡：按需换行（1600×950 两行、1280×800 三行，都不切字）
        self.stats_holder = page_layout.StatsGrid(self.stat_labels, STAT_KEYS)
        self.stats_holder.hide()        # 没结果时不占高度（预览时图可以高一点）
        layout.addWidget(self.stats_holder)

        self.result_tabs = QtWidgets.QTabWidget()
        self.points_page = self._build_points_page()
        self.preview_page = self._build_preview_page()
        self.log_page = self._build_log_page()
        self.result_tabs.setMinimumHeight(TABS_MIN_HEIGHT)
        self.result_tabs.addTab(self.points_page, TABS[0])
        self.result_tabs.addTab(self.preview_page, TABS[1])
        self.result_tabs.addTab(self.log_page, TABS[2])
        # 切页签后重新撑一次（口径见 generate_page.expand_result_area）
        self.result_tabs.currentChanged.connect(
            lambda _index: self._expand_result_area())
        layout.addWidget(self.result_tabs, 1)

        export_row = QtWidgets.QHBoxLayout()
        export_row.setContentsMargins(0, 0, 0, 0)
        self.export_caption = _caption("导出：", lines=1)
        self.export_caption.setToolTip("从 runs/<id>/ 复制，原文件不动")
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
        """「路点」页：一行一个点（P1~P5），失败行整行标红。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(_caption(
            "P1 是立方体中心，P2~P5 是截面的四个角；无解的行整行标红，"
            "关节角缺失显示「—」。", lines=1))
        self.point_table = page_layout.mark_result_table(
            make_table(POINT_HEADERS))
        layout.addWidget(self.point_table, 1)
        return page

    def _build_preview_page(self) -> QtWidgets.QScrollArea:
        """「预览」页：6 个面心 / 9 个候选点 / 最终 5 点（含 layout 标注）。"""
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

        layout.addWidget(QtWidgets.QLabel("最终 5 点（按 layout 布点）"))
        self.preview_table = make_table(("P", "X (mm)", "Y (mm)", "Z (mm)",
                                         "layout 标注"))
        self.preview_table.setSortingEnabled(False)
        self.preview_table.setMinimumHeight(130)
        layout.addWidget(self.preview_table)

        layout.addWidget(_caption(MOVL_NOTE))
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

    def _build_log_page(self) -> QtWidgets.QWidget:
        """「日志」页：算法自己的 ``print`` 与 ``jobs`` 的提示，等价终端输出。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(_caption(
            f"等价终端输出。后台任务源缓冲只留最近 {JOB_LOG_LIMIT} 行"
            f"（丢一段会插一行「…已省略 N 行」），本控件最多保留 "
            f"{LOG_WIDGET_LIMIT} 行（省略提示另存上面一行，不会被删）。"))
        self.log_omission_label = QtWidgets.QLabel("")
        self.log_omission_label.setWordWrap(True)
        self.log_omission_label.setStyleSheet(_STATUS_STYLES["muted"])
        self.log_omission_label.hide()
        layout.addWidget(self.log_omission_label)
        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setFont(monospace_font())
        self.log_edit.setMaximumBlockCount(LOG_WIDGET_LIMIT)
        self.log_edit.setPlaceholderText("点「生成五点」后，这里实时滚日志。")
        layout.addWidget(self.log_edit, 1)
        return page

    # ------------------------------------------------------------------ 取值
    def mode_value(self) -> str:
        """当前选中的立方体来源（``bounds`` / ``auto``），永远返回合法值。"""
        buttons = self.mode_buttons or []
        for index, button in enumerate(buttons):
            if button.isChecked():
                return CUBE_MODE_ORDER[index]
        return CUBE_MODE_ORDER[0]            # 没建/没选中：退回第一个（总有面板）

    def values(self) -> dict:
        """当前表单值（键与 :func:`form_to_params` 的入参一致）。"""
        fields = self.fields
        return {
            "robot": fields["robot"].currentData() or ui_state.default_robot(),
            "cube_mode": self.mode_value(),
            "cube": [fields["x0"].value(), fields["x1"].value(),
                     fields["y0"].value(), fields["y1"].value(),
                     fields["z0"].value(), fields["z1"].value()],
            "search_box": [fields["box_x0"].value(), fields["box_x1"].value(),
                           fields["box_y0"].value(), fields["box_y1"].value(),
                           fields["box_z0"].value(), fields["box_z1"].value()],
            "resolution": float(fields["resolution"].value()),
            # 空 = 不限（``None``）；0 是**合法值**（中心 X ≥ 0）
            "prefer_x_min": self.prefer_min_value(),
            "layout": fields["layout"].currentData(),
            "plane": fields["plane"].currentData(),
            "square_frac": float(fields["square_frac"].value()),
            "rpy": [fields["rpy_a"].value(), fields["rpy_b"].value(),
                    fields["rpy_c"].value()],
            "euler": fields["euler"].currentData(),
            "q_seed": fields["q_seed"].text(),
            "j5_min": float(fields["j5_min"].value()),
        }

    def params(self) -> dict:
        """当前表单值 -> ``core.run_iso9283`` 的入参。"""
        return form_to_params(self.values())

    def preview_result(self) -> dict:
        """最近一次 :func:`preview_geometry` 的结果（原样，含报错原文）。"""
        return self.preview or {}

    # ------------------------------------------------------------ 切换与刷新
    def _on_change(self, *_args) -> None:
        """任何输入的改动都走这里：立刻（去抖后）重算预览。"""
        self.refresh_preview()

    def _on_mode_switch(self, checked: bool) -> None:
        """「两点输入 / 自动搜索」切换：换面板**并重算预览**。

        只在**被选中**的那个按钮上动一次（``toggled`` 会同时通知旧按钮的
        ``False``），并且另一组只是隐藏——控件与填过的值都留着。

        换完面板必须自己重算一次：预览卡的每一项（边长 / 边心距 / 中心 /
        面心 / 候选点 / 五点 / 提示）都只由 ``preview`` 决定，而 ``preview``
        来自 ``values()["cube_mode"]``——不重算就一直是上一种来源的数字
        （实测：切到自动搜索后卡片还写着「边长 550 mm」和「三轴跨度不等」）。

        ``refresh_preview`` 必须放在**这里**、不能放进 :meth:`switch_mode`：
        初始化时 ``switch_mode`` 由 :meth:`_build_cube_group` 触发，那一刻
        ``rpy_a`` / ``q_seed`` 还没建出来，``values()`` 会 ``KeyError``（所以
        那儿的 ``toggled`` 也是等 ``setChecked`` 之后再连的）。
        """
        if not checked:
            return
        self.switch_mode(self.mode_value())
        self.refresh_preview()

    def switch_mode(self, mode: str) -> None:
        """切换立方体来源的面板（**永远恰好显示一组**，另一组的值不清空）。

        初始化时**必须用 radio 的当前值**：网页版曾把 ``"center"`` 写死在
        初始化里，而那个模式已经不存在 → 两组输入全被隐藏、页面看起来没有
        输入框（commit ``32136c6``）。这里用 ``QStackedWidget`` 从结构上保证
        “只有一个可见”，且仍然只按传入的模式切，模式名不重复写死。

        Args:
            mode: ``bounds`` / ``auto``；未知值按 ``bounds`` 处理（总有面板可见）。

        注意：这里只切面板，**不重算预览**。重算在 :meth:`_on_mode_switch`
        （用户点 radio 时），因为初始化那一下调本方法时后面的分组还没建出来。
        """
        index = (CUBE_MODE_ORDER.index(mode)
                 if str(mode) in CUBE_MODE_ORDER else 0)
        self.mode_stack.setCurrentIndex(index)
        button = self.mode_buttons[index]
        if not button.isChecked():
            button.blockSignals(True)
            button.setChecked(True)
            button.blockSignals(False)

    def switch_layout(self) -> None:
        """``square_frac`` 只对 ``center-square`` 生效（其余布点用不到它）。"""
        self._apply_square_gate()
        self.refresh_preview()

    def _apply_square_gate(self) -> None:
        """只改 ``square_frac`` 的启用状态（构建期用，不触发刷新）。"""
        layout = self.fields["layout"].currentData()
        self.fields["square_frac"].setEnabled(layout == "center-square")

    def refresh_preview(self) -> dict:
        """重算几何预览（去抖 300 ms），并按结果启用/禁用「生成五点」。

        Returns:
            dict: :func:`preview_geometry` 的结果。
        """
        # 任务在跑时工作线程也在 ``print``（日志页读的就是它），而
        # ``redirect_stdout`` 换的是进程级 ``sys.stdout``：这时抓会把它的日志
        # 行吞掉（见 :func:`preview_geometry`）。
        capture = not (self._busy or self.bridge.is_running)
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
            f"参数已改动，下表来自上一次运行（{result.get('run_id') or '—'}）；"
            "要更新请重新点「生成五点」。")
        self.stale_label.show()

    def prefer_min_value(self):
        """「中心 X 不小于」输入框 -> ``float`` 或 ``None``（**空 = 不限**）。

        早先用 ``0`` 当哨兵，于是「中心 X ≥ 0」这个合法取值根本没法表达；
        现在空串才是哨兵。填了非数字就抛 ``ValueError``——由
        :meth:`refresh_preview` 的兜底转成卡片里的红字，**不**静默当成“不限”。

        Returns:
            float | None: 下限（mm）；空输入时 ``None``。

        Raises:
            ValueError: 填的不是数字。
        """
        raw = str(self.fields["prefer_x_min"].text()).strip()
        if not raw:
            return None
        try:
            return float(raw)
        except ValueError as error:
            raise ValueError(
                f"「中心 X 不小于」要填数字或留空（当前 {raw!r}）") from error

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
        if side is None:                    # 自动搜索：真跑才知道边长/中心
            self.preview_labels["cube"].setText("待真跑（自动搜索）")
            self.preview_labels["apothem"].setText("—")
            self.preview_labels["center"].setText("—")
            box = preview.get("search_box") or []
            self.preview_hint.setText(
                "搜索范围 " + "，".join(f"{float(value):.0f}" for value in box)
                + f"（步长 {float(preview.get('resolution') or 0):g} mm）"
                "——边长与中心由真跑给出。")
            self.preview_table.setRowCount(0)
            self.faces_edit.setPlainText("")
            self.candidates_edit.setPlainText("")
        else:
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
                f"五点 {len(preview.get('points') or [])} 个（不跑 IK）。")
            self.faces_edit.setPlainText(block_text(preview.get("faces")))
            self.candidates_edit.setPlainText(
                block_text(preview.get("candidates")))
            fill_preview_table(self.preview_table, preview)
        self.preview_labels["layout"].setText(layout_label(preview.get("layout")))
        self.preview_labels["plane"].setText(plane_label(preview.get("plane")))
        warnings = [str(line) for line in (preview.get("warnings") or [])]
        self.warning_label.setText("\n".join(warnings))
        self.warning_label.setVisible(bool(warnings))

    def _sync_buttons(self) -> None:
        """按“参数合不合法 / 任务在不在跑”启用「生成五点」。"""
        running = self._busy or self.bridge.is_running
        valid = bool((self.preview or {}).get("ok"))
        self.generate_button.setEnabled(valid and not running)

    def apply_preview(self):
        """按当前预览重建图（参数非法 / 自动搜索时给说明图）。

        Returns:
            plotly ``Figure``（同时存进 ``last_figure``；挂了 PlotView 就推过去）。
        """
        if self._busy:
            # 解算期间不拿预览图盖住结果/进度
            return self.last_figure
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

    # ------------------------------------------------------------ 起任务
    def start_iso(self):
        """校验当前表单并起一次真解算（``core.run_iso9283``）。

        流程与生成页一致：预览不合法就只把报错原文写在卡片里、**不起任务**；
        合法则清空结果区、显示进度条与取消，再把 ``params`` 交给
        ``bridge.submit("iso", ...)``。

        Returns:
            str: 任务 id；未启动（参数非法 / 已有任务在跑）时为 ``None``。
        """
        if self.bridge.is_running:
            self._set_status("已有任务在跑：等它结束，或先点「取消」。",
                             level="muted")
            self._report_status("已有任务在跑，未重复提交")
            return None
        preview = self.refresh_preview()
        if not preview.get("ok"):
            self._set_status(
                f"参数不合法，未启动解算：{preview.get('error') or '预览失败'}",
                level="error")
            self._report_status("参数不合法，未启动解算")
            return None

        params = form_to_params(self.values())
        self._reset_result_area()
        self._busy = True
        self._debounce.stop()       # 小案子几百毫秒就跑完：别让陈旧预览盖结果
        self.run_state = "running"
        self.generate_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.cancel_button.show()
        self.result_tabs.setCurrentWidget(self.log_page)   # 跑的时候先看日志
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("排队 0/0 · 排队中")
        self._set_status(f"已提交：{layout_label(preview.get('layout'))}",
                         level="info")
        try:
            job_id = self.bridge.submit(
                "iso",
                lambda job: core.run_iso9283(params, job=job,
                                            on_progress=job.set_progress))
        except Exception as error:                      # noqa: BLE001 - 别抛穿槽
            _LOGGER.exception("提交 ISO 任务失败（params=%r）", params)
            self._busy = False
            self.run_state = "failed"
            self.cancel_button.hide()
            self._sync_buttons()
            self.progress_bar.setFormat("失败")
            self._set_status(f"提交任务失败：{type(error).__name__}: {error}",
                             level="error")
            self._report_status(f"提交任务失败：{type(error).__name__}: {error}")
            return None
        self.last_job_id = job_id
        self._report_status(f"已提交 ISO 任务 {job_id}")
        return job_id

    def _resume_running_job(self) -> None:
        """启动时扫一遍本桥登记表里未结束的 ``iso`` 任务，有就接回来。"""
        job = adopt_running_job(self.bridge.registry, "iso")
        if job is None:
            return
        if self.bridge.attach(job.id) is None:          # 刚好被清掉
            return
        self._busy = True
        self.run_state = "running"
        self.cancel_button.show()
        self.cancel_button.setEnabled(True)
        self._sync_buttons()
        self._set_status(
            f"接管本进程里还没跑完的任务 {job.id}"
            f"（当前 {STATE_TEXT.get(job.state, job.state)}）", level="info")

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

    # ------------------------------------------------------- 信号（GUI 线程）
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
        """增量日志：直接追加；省略提示另存一份到固定标签（L4）。"""
        text = str(line)
        if text.startswith(generate_page.OMISSION_PREFIX):
            self.log_omission_label.setText(
                f"{text}（更早的行已不在源缓冲里；下面只剩最后"
                f" {LOG_WIDGET_LIMIT} 行）")
            self.log_omission_label.show()
        self.log_edit.appendPlainText(text)

    def _on_finished(self, payload: dict) -> None:
        """终态：done → 渲染结果；cancelled → 灰字；failed → 红字。

        ``core.run_iso9283`` 在有解不出的点时返回 ``ok=False``（但 ``iso.json``
        与 ``iso.png`` 已经落盘）：那种情况仍然接管结果、点亮导出，并把
        ``error`` 原文写在状态行——否则现场看不到该往哪个方向缩立方体。
        """
        state = str(payload.get("state") or "")
        result = payload.get("result")
        self._busy = False
        self.run_state = state
        self.cancel_button.hide()
        self.cancel_button.setEnabled(True)
        self._sync_buttons()

        if state == "done" and isinstance(result, dict):
            self.last_result = result
            run_id = result.get("run_id")
            run_dir = result.get("run_dir")
            self._update_exports(result)
            if result.get("ok"):
                self.progress_bar.setValue(PROGRESS_STEPS)
                self.progress_bar.setFormat("完成 1/1 · 完成")
                self.render_result(result)
                stats = result.get("stats") or {}
                text = (f"完成：IK {stats.get('solved')}/{stats.get('total')} 点"
                        f" · {run_dir_text(run_dir)}")
                warnings = [str(line) for line in (result.get("warnings") or [])]
                if warnings:
                    text += "\n备注：" + "；".join(warnings)
                self._set_status(text, level="ok")
                self.status_label.setToolTip(str(run_dir or ""))
                self._report_status(f"ISO 五点完成：{run_id}")
                return
            # 有解不出的点：产物仍然在盘上，结果照样铺出来
            self.render_result(result)
            self.progress_bar.setFormat("完成（有解不出的点）")
            error = result.get("error") or "未知错误"
            warnings = [str(line) for line in (result.get("warnings") or [])]
            text = f"完成但不可用：{error}"
            if warnings:
                text += "\n" + "\n".join(f"· {line}" for line in warnings)
            self._set_status(text, level="error")
            self._report_status(f"ISO 五点有解不出的点：{error}")
            return

        if state == "cancelled":
            self.progress_bar.setFormat("已取消")
            carried = result if isinstance(result, dict) else {}
            self.last_result = carried or None
            self._set_status(
                "已取消：任务在当前阶段末尾停下；没来得及落盘的运行目录已删掉，"
                "已经写出结果的目录会保留。", level="muted")
            self._report_status("ISO 五点已取消")
            return

        error = ((result or {}).get("error") if isinstance(result, dict) else None) \
            or payload.get("error") or "未知错误"
        self.progress_bar.setFormat("失败")
        self._set_status(f"失败：{error}", level="error")
        self.result_tabs.setCurrentWidget(self.log_page)
        self._report_status(f"ISO 五点失败：{error}")

    # ------------------------------------------------------------- 结果渲染
    def render_result(self, result: dict) -> None:
        """把 ``core.run_iso9283`` 的返回值铺到结果区（统计 / 路点表 / 图）。

        Args:
            result: ``core.run_iso9283`` 的成功返回值（或同构的构造 dict）。
        """
        self.last_result = result
        self._result_seq = self._change_seq
        self.stale_label.hide()          # 这一次的结果就是按当前参数跑的
        fill_iso_stats(self.stat_labels, result.get("stats") or {})
        fill_point_table(self.point_table, point_rows(result))
        self._update_exports(result)
        self.stats_holder.show()
        self.result_tabs.show()
        try:
            figure = result_figure(result)
        except Exception as error:                      # noqa: BLE001 - 别崩窗口
            _LOGGER.exception("ISO 结果图渲染失败（运行 %r）", result.get("run_id"))
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
        """按 ``result["artifacts"]`` 启用/禁用导出按钮（文件真的在才开）。"""
        artifacts = (result or {}).get("artifacts") or {}
        run_dir = (result or {}).get("run_dir")
        for key, button in self.export_buttons.items():
            name = artifacts.get(key)
            path = None if (name is None or run_dir is None) \
                else Path(str(run_dir)) / str(name)
            available = path is not None and path.is_file()
            button.setEnabled(available)
            button.setToolTip("" if available else f"本次运行没有 {key} 产物")

    def _on_export(self, key: str) -> None:
        """导出按钮：选目标后**复制**（不移动，``runs/`` 是归档）。

        Args:
            key: ``artifacts`` 里的键（``iso_json`` / ``plot``）。
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
        self.last_figure = None
        self.log_edit.clear()
        self.log_omission_label.clear()
        self.log_omission_label.hide()
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
            self._push_figure(placeholder_figure("正在解算：新任务已提交"))

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
        """窗口关闭时收尾：停去抖定时器并注销任务订阅（不取消任务）。"""
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
        if self.run_state == "running":
            figure = placeholder_figure(
                "正在解算：任务在跑，结果出来会显示在这里。",
                title="ISO 五点图（进行中）")
        else:
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
