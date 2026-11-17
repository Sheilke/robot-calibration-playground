# -*- coding: utf-8 -*-
"""生成页：机器人模型 + 采样域表单 + 即时预览 + **后台解算与结果渲染**。

M1 只做「改动即刷新」的只读预览（体积 / 网格 / 点数 / 设计条件数，约 1 ms）；
M2 把两个按钮接上真活儿（设计文档 §5.1 / §8）：

* 「生成并解算」→ ``jobs.submit`` 起后台任务（``core.run_scan``），
  进度条 + 日志 + 取消都来自任务快照的轮询；
* 「预览路点」→ 同一个 ``run_scan`` 的 ``skip_ik=True`` 分支，只生成路点并出
  3D 图（不算 IK），不复制任何流程；
* 结果区：统计卡 / plotly 3D（可切着色依据）/ 路点表（可排序、失败行标红）/
  段诊断表 / 三个产物下载按钮。

表单状态与任务状态全部是 :func:`render` 的局部变量（每个客户端一份），没有
模块级可变对象；``jobs.get_registry()`` 是进程级单例，因为解算本来就该串行。
"""

from __future__ import annotations

from pathlib import Path

from nicegui import ui

from excitation_trajectory import aabb_snake_scan as scan

from .. import core, figures, jobs
from . import (POINT_COUNT_WARN, adopt_running_job, cancelled_result,
               download_button)
from ..ui_state import default_form_values, euler_conventions

#: M2 新增字段的默认值——**不抄数字**，全部问 CLI 的参数解析器要。
_CLI_DEFAULTS = scan.build_arg_parser().parse_args([])
#: 段模型默认值：**故意与 CLI 不同**。CLI 默认 ``--seg-mode joint``（PTP，
#: 历史行为），而现场 JOB 走 MOVL，所以界面默认按笛卡尔直线检查；这只影响
#: 段诊断，不改变 IK 结果（与 CLI 的唯一有意差异，见代码里的注释）。
_DEFAULT_SEG_MODE = "cartesian"
#: 段模型下拉框：值 -> 页面文案。
_SEG_MODES = {"cartesian": "MOVL（笛卡尔直线）", "joint": "PTP（关节直线）"}
#: 轮询间隔（秒）。IK 是阶段级的，0.3 s 足够跟手又不会刷爆事件循环。
POLL_INTERVAL = 0.3
#: 任务状态 -> 界面文案。
_STATE_TEXT = {"pending": "排队中", "running": "进行中", "done": "完成",
               "failed": "失败", "cancelled": "已取消"}

#: 路点表列（名字, 表头, 字段, 对齐）。
_WAYPOINT_COLUMNS = (
    ("gp", "GP", "gp", "right"),
    ("x", "X (mm)", "x", "right"),
    ("y", "Y (mm)", "y", "right"),
    ("z", "Z (mm)", "z", "right"),
    ("q1", "q1", "q1", "right"),
    ("q2", "q2", "q2", "right"),
    ("q3", "q3", "q3", "right"),
    ("q4", "q4", "q4", "right"),
    ("q5", "q5", "q5", "right"),
    ("q6", "q6", "q6", "right"),
    ("j3", "|J3|°", "j3", "right"),
    ("j5", "|J5|°", "j5", "right"),
    ("cfg", "cfg", "cfg", "left"),
    ("dq_l2", "dqL2", "dq_l2", "right"),
    ("ok", "ok", "ok", "center"),
)
#: 预览模式（``skip_ik``）只显示坐标列。
_PREVIEW_COLUMNS = tuple(column for column in _WAYPOINT_COLUMNS
                         if column[0] in ("gp", "x", "y", "z"))
#: 失败行标红的 Vue 模板（Quasar ``body-row`` 槽，保留排序与分页）。
#: 空单元格画 ``—``：失败路点没有关节角，留白看着像“值太小被吃掉了”。
_FAILED_ROW_SLOT = """
<q-tr :props="props" :class="props.row.failed ? 'bg-red-1 text-negative' : ''">
    <q-td v-for="col in props.cols" :key="col.name" :props="props">
        {{ col.value === null || col.value === undefined ? '—' : col.value }}
    </q-td>
</q-tr>
"""


def _fmt(value, digits: int = 3) -> str:
    """None 安全的数字显示（失败点没有关节角，写 ``—``）。"""
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def _round(value, digits: int):
    """保留位数（``None`` 原样返回），供表格排序用数值而不是字符串。"""
    return None if value is None else round(float(value), digits)


def _number(label: str, value, on_change, **kwargs) -> ui.number:
    """建一个占满一行的数字输入框（默认值来自 ``ui_state`` / CLI）。"""
    return ui.number(label, value=value, on_change=on_change,
                     **kwargs).classes("grow")


def _columns(spec) -> list[dict]:
    """列定义转成 ``ui.table`` 的 dict（全部可排序）。"""
    return [{"name": name, "label": label, "field": field, "align": align,
             "sortable": True} for name, label, field, align in spec]


def _waypoint_rows(result: dict) -> list[dict]:
    """``run_scan`` 的路点行 -> 表格行（数值保留 1~3 位，失败行标红）。

    数值**保持数字类型**（点表头排序要按大小排，变成字符串就成字典序了）；
    失败路点的关节角是 ``None``，由 ``_FAILED_ROW_SLOT`` 画成 ``—``——与
    :func:`_fmt` 的约定一致，但不在 Python 里转字符串。
    """
    preview = bool(result["meta"].get("skip_ik"))
    rows = []
    for row in result["waypoints"]:
        joints = row["q"] or [None] * 6
        config = row["config"]
        table_row = {"gp": row["gp"],
                     "x": _round(row["pos"][0], 1),
                     "y": _round(row["pos"][1], 1),
                     "z": _round(row["pos"][2], 1),
                     "cfg": "—" if config is None
                            else ",".join(str(int(c)) for c in config),
                     "ok": "预览" if preview else ("✓" if row["ok"] else "✗"),
                     "failed": False if preview else not row["ok"]}
        for index, key in enumerate(("q1", "q2", "q3", "q4", "q5", "q6")):
            table_row[key] = _round(joints[index], 3)
        table_row["j3"] = _round(row["j3_abs"], 3)
        table_row["j5"] = _round(row["j5_abs"], 3)
        table_row["dq_l2"] = _round(row["dq_l2"], 2)
        rows.append(table_row)
    return rows


def _result_mode(result) -> str:
    """结果区该画哪种卡片：``empty`` / ``cancelled`` / ``failed`` / ``ok``。

    取消**不是**失败：它是现场工程师主动点的，用灰色中性卡片而不是红字，
    否则看着像算法炸了（评审提的“取消被渲染成失败”）。
    """
    if result is None:
        return "empty"
    if result.get("cancelled"):
        return "cancelled"
    return "ok" if result.get("ok") else "failed"


def _segment_rows(result: dict) -> list[dict]:
    """段诊断行：GP 起→止 / |J3|min / |J5|min / 穿零 / 最差点。"""
    rows = []
    for segment in result.get("segments") or []:
        position = segment.get("worst_pos")
        rows.append({
            "gp": f"{segment['gp_from']} → {segment['gp_to']}",
            "j3": _round(segment["j3_abs_min"], 3),
            "j5": _round(segment["j5_abs_min"], 3),
            "crosses": "是" if segment["crosses"] else "否",
            "worst": ("—" if not position else
                      ", ".join(f"{float(v):.1f}" for v in position)
                      + f" (t={segment['worst_frac']:.2f})"),
            "crosses_flag": bool(segment["crosses"]),
        })
    return rows


def _figure_for(result: dict, color_by: str):
    """按当前着色依据重建 plotly figure（切换 toggle 时用）。"""
    return figures.waypoint_figure(
        result["waypoints"], color_by=color_by,
        worsts=result.get("segments"),
        all_ok=bool(result["meta"].get("skip_ik")))


def render() -> None:
    """渲染生成页（含 M2 的后台解算与结果区）。"""
    defaults = default_form_values()
    defaults.update({
        "j5_min": float(_CLI_DEFAULTS.j5_min),
        "j3_min": float(_CLI_DEFAULTS.j3_min),
        "j5_attempts": int(_CLI_DEFAULTS.j5_attempts),
        "seg_min": float(_CLI_DEFAULTS.seg_min),
        "seg_mode": _DEFAULT_SEG_MODE,
        "seg_samples": int(_CLI_DEFAULTS.seg_samples),
        "seed": int(_CLI_DEFAULTS.seed),
        "q_seed": str(_CLI_DEFAULTS.q_seed),
    })
    # 只列 Nexbot6R 真能加载的模型；dh.json / scan.json / robot_sim 资产会被探
    # 测判为不可用，选了也跑不起来，所以不进下拉。
    robots = core.list_robots()
    usable = [robot for robot in robots if robot["usable"]]
    skipped = [robot for robot in robots if not robot["usable"]]
    # ui.select 的 dict 选项在 NiceGUI 4.0 里是 {value: label}
    options = {robot["path"]: robot["label"] for robot in usable}
    default_robot = defaults["robot"]
    if default_robot not in options:      # CLI 默认模型被删掉时退回第一个
        default_robot = next(iter(options), None)

    fields = {}      # 表单控件（ui.number / ui.select / ui.toggle / ui.input）
    boxes = {}       # 需要整块显隐的容器（两点/六数、凑数/手填）
    state = {"job_id": None, "log_cursor": 0, "result": None,
             "color_by": "order", "preview": None}
    registry = jobs.get_registry()

    def collect() -> dict:
        """把当前表单值收成 ``core.preview_domain`` / ``core.run_scan`` 的入参。

        只提交**当前激活**的那一组互斥项（两点/六数、凑数/手填）；互斥
        规则本身仍由 ``_validate_args`` 判定，这里不做二次判断。
        """
        params = {
            "robot": fields["robot"].value,
            "rpy": [fields["rpy_roll"].value,
                    fields["rpy_pitch"].value,
                    fields["rpy_yaw"].value],
            "euler": fields["euler"].value,
            "perturb_deg": [fields["perturb_roll"].value,
                            fields["perturb_pitch"].value,
                            fields["perturb_yaw"].value],
            "cond_max": fields["cond_max"].value,
            # ---- M2：守卫 / 段检查 / 随机种子 ----
            "j5_min": fields["j5_min"].value,
            "j3_min": fields["j3_min"].value,
            "j5_attempts": fields["j5_attempts"].value,
            "seg_min": fields["seg_min"].value,
            "seg_mode": fields["seg_mode"].value,
            "seg_samples": fields["seg_samples"].value,
            "seed": fields["seed"].value,
            "q_seed": fields["q_seed"].value,
        }
        if fields["domain_mode"].value == "points":
            params["point_min"] = [fields["min_x"].value,
                                   fields["min_y"].value,
                                   fields["min_z"].value]
            params["point_max"] = [fields["max_x"].value,
                                   fields["max_y"].value,
                                   fields["max_z"].value]
        else:
            params["aabb6"] = [fields["x_min"].value, fields["x_max"].value,
                               fields["y_min"].value, fields["y_max"].value,
                               fields["z_min"].value, fields["z_max"].value]
        if fields["grid_mode"].value == "count":
            params["target_count"] = fields["target_count"].value
        else:
            params["xs"] = fields["xs"].value
            params["ys"] = fields["ys"].value
            params["zs"] = fields["zs"].value
            params["z_insert"] = fields["z_insert"].value
        return params

    def switch_domain(mode: str) -> None:
        """切换“两点 / 六数”输入，并立刻刷新预览。"""
        boxes["points"].set_visibility(mode == "points")
        boxes["axes"].set_visibility(mode == "axes")
        preview_card.refresh()

    def switch_grid(mode: str) -> None:
        """切换“目标点数 / 手填份数”，并立刻刷新预览。"""
        boxes["count"].set_visibility(mode == "count")
        boxes["explicit"].set_visibility(mode == "explicit")
        preview_card.refresh()

    def _preview() -> dict:
        """当前表单的预览结果（``preview_card`` 与点数守卫共用，按参数缓存）。

        ``core.preview_domain`` 走 ``_validated_namespace``，后者用
        ``redirect_stdout`` 收 CLI 的提示行——那是**进程级**的副作用：任务在
        跑的时候每调一次，都可能把工作线程这一瞬间的 ``print`` 吞进临时
        ``StringIO``（日志少一行）。所以只在参数变了才重新算，不每次重算。
        """
        params = collect()
        cached = state.get("preview")
        if cached is None or cached[0] != params:
            state["preview"] = (params, core.preview_domain(params))
        return state["preview"][1]

    @ui.refreshable
    def preview_card() -> None:
        """预览卡片：体积 / 网格 / 点数 / 设计条件数，或一行红字报错。"""
        result = _preview()
        with ui.card().classes("w-full gap-1"):
            ui.label("预览（不跑 IK）").classes("text-subtitle1")
            if not result["ok"]:
                ui.label(result["error"]).classes(
                    "text-negative text-body2 whitespace-pre-wrap")
                return
            grid = result["grid"]
            cells = f"{grid['xs'] + 1}×{grid['ys'] + 1}×{grid['zs'] + 1}"
            insert = (f" + {grid['zs'] * grid['z_insert']} 插点"
                      if grid["z_insert"] else "")
            with ui.grid(columns=2).classes("w-full gap-x-4"):
                ui.label("体积").classes("text-grey-7")
                ui.label(f"{result['volume_m3']:.3f} m³")
                ui.label("网格").classes("text-grey-7")
                ui.label(f"{cells}{insert}")
                ui.label("点数").classes("text-grey-7")
                over = result["point_count"] > POINT_COUNT_WARN
                ui.label(f"{result['point_count']} 点").classes(
                    "text-negative" if over else "")
                if over:
                    ui.label(f"超过 {POINT_COUNT_WARN} 点：规划与解算会占满内存，"
                             f"且取消要等当前阶段结束。请减少份数或改用"
                             f"目标点数。").classes(
                        "text-negative text-caption whitespace-pre-wrap")
                ui.label("设计条件数").classes("text-grey-7")
                ui.label(_fmt(result["design_condition"]))
            ui.label(f"欧拉约定 {result['convention']}").classes("text-caption")
            for line in result["warnings"]:
                ui.label(line).classes(
                    "text-warning text-caption whitespace-pre-wrap")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        # ---------------- 左：表单 ----------------
        with ui.column().classes("grow gap-3"):
            with ui.card().classes("w-full"):
                ui.label("机器人模型").classes("text-subtitle1")
                fields["robot"] = ui.select(
                    options, value=default_robot,
                    label="模型（data/configs 下，含 robot_sim/）",
                    on_change=preview_card.refresh).classes("w-full")
                ui.label().bind_text_from(
                    fields["robot"], "value",
                    lambda path: f"路径：{path}").classes("text-caption")
                ui.label("上传控制器 JSON 并在 M3 的“工具”页转 URDF").classes(
                    "text-caption text-grey-6")
                if skipped:
                    with ui.expansion(
                            f"另有 {len(skipped)} 个文件不能当模型用", icon="info"
                    ).classes("text-caption"):
                        for robot in skipped:
                            ui.label(f"{robot['label']} — {robot['reason']}").classes(
                                "text-caption text-grey-7")

            with ui.card().classes("w-full"):
                ui.label("采样域").classes("text-subtitle1")
                fields["domain_mode"] = ui.toggle(
                    {"points": "两点输入", "axes": "六个轴边界"},
                    value=defaults["domain_mode"],
                    on_change=lambda event: switch_domain(event.value)).props(
                        "spread")
                boxes["points"] = ui.column().classes("w-full gap-1")
                with boxes["points"]:
                    for row, keys in (
                            ("point_min", (("min_x", "min X"),
                                           ("min_y", "min Y"),
                                           ("min_z", "min Z"))),
                            ("point_max", (("max_x", "max X"),
                                           ("max_y", "max Y"),
                                           ("max_z", "max Z")))):
                        with ui.row().classes("w-full no-wrap gap-2"):
                            for index, (key, label) in enumerate(keys):
                                fields[key] = _number(label, defaults[row][index],
                                                      preview_card.refresh)
                boxes["axes"] = ui.column().classes("w-full gap-1")
                with boxes["axes"]:
                    for index, (low_key, high_key, axis) in enumerate(
                            (("x_min", "x_max", "X"),
                             ("y_min", "y_max", "Y"),
                             ("z_min", "z_max", "Z"))):
                        with ui.row().classes("w-full no-wrap gap-2"):
                            fields[low_key] = _number(
                                f"{axis} 下界", defaults["aabb6"][2 * index],
                                preview_card.refresh)
                            fields[high_key] = _number(
                                f"{axis} 上界", defaults["aabb6"][2 * index + 1],
                                preview_card.refresh)

            with ui.card().classes("w-full"):
                ui.label("姿态与扰动").classes("text-subtitle1")
                with ui.row().classes("w-full no-wrap gap-2"):
                    for index, (key, label) in enumerate(
                            (("rpy_roll", "参考 A (roll)"),
                             ("rpy_pitch", "参考 B (pitch)"),
                             ("rpy_yaw", "参考 C (yaw)"))):
                        fields[key] = _number(label, defaults["rpy"][index],
                                              preview_card.refresh)
                fields["euler"] = ui.select(
                    euler_conventions(), value=defaults["euler"],
                    label="欧拉约定",
                    on_change=preview_card.refresh).classes("w-full")
                with ui.row().classes("w-full no-wrap gap-2"):
                    for index, (key, label) in enumerate(
                            (("perturb_roll", "扰动 ±roll"),
                             ("perturb_pitch", "扰动 ±pitch"),
                             ("perturb_yaw", "扰动 ±yaw"))):
                        fields[key] = _number(label, defaults["perturb_deg"][index],
                                              preview_card.refresh)
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields["seed"] = _number("随机种子 seed", defaults["seed"],
                                             None, min=0, step=1)

            with ui.card().classes("w-full"):
                ui.label("网格与守卫").classes("text-subtitle1")
                fields["grid_mode"] = ui.toggle(
                    {"count": "目标点数", "explicit": "手填份数"},
                    value=defaults["grid_mode"],
                    on_change=lambda event: switch_grid(event.value)).props(
                        "spread")
                boxes["count"] = ui.column().classes("w-full gap-1")
                with boxes["count"]:
                    fields["target_count"] = _number(
                        "目标点数（自动凑网格，与手填份数互斥）",
                        defaults["target_count"], preview_card.refresh,
                        min=1, step=1)
                boxes["explicit"] = ui.column().classes("w-full gap-1")
                with boxes["explicit"]:
                    with ui.row().classes("w-full no-wrap gap-2"):
                        for key, label in (("xs", "X 份数"), ("ys", "Y 份数"),
                                           ("zs", "Z 份数"),
                                           ("z_insert", "换层插点")):
                            fields[key] = _number(label, defaults[key],
                                                  preview_card.refresh,
                                                  min=0, step=1)
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields["j5_min"] = _number(
                        "|J5| 下限（度）", defaults["j5_min"], None,
                        min=0.0, step=1.0)
                    fields["j3_min"] = _number(
                        "|J3| 下限（度）", defaults["j3_min"], None,
                        min=0.0, step=1.0)
                fields["cond_max"] = _number(
                    "条件数上限 cond_max", defaults["cond_max"],
                    preview_card.refresh, min=1.0, step=0.5)

            with ui.card().classes("w-full"):
                ui.label("段内检查与 IK 种子").classes("text-subtitle1")
                fields["seg_mode"] = ui.select(
                    _SEG_MODES, value=defaults["seg_mode"],
                    label="段模型").classes("w-full")
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields["seg_min"] = _number(
                        "段内 min（度）", defaults["seg_min"], None,
                        min=0.0, step=1.0)
                    fields["seg_samples"] = _number(
                        "每段采样点", defaults["seg_samples"], None,
                        min=2, step=1)
                    fields["j5_attempts"] = _number(
                        "重采上限", defaults["j5_attempts"], None,
                        min=1, step=1)
                fields["q_seed"] = ui.input(
                    "q-seed（6 个关节角，逗号分隔，度）",
                    value=defaults["q_seed"]).classes("w-full")

            with ui.row().classes("gap-2"):
                preview_button = ui.button(
                    "预览路点", icon="visibility",
                    on_click=lambda: start_job({"skip_ik": True}))
                solve_button = ui.button(
                    "生成并解算", icon="play_arrow",
                    on_click=lambda: start_job({"skip_ik": False}))
                solve_button.props("color=primary")

        # ---------------- 右：预览卡片 ----------------
        with ui.column().classes("w-80 gap-3"):
            preview_card()

    # ---------------- 进度与日志 ----------------
    progress_card = ui.card().classes("w-full")
    with progress_card:
        with ui.row().classes("w-full items-center gap-3 no-wrap"):
            ui.icon("settings_suggest").classes("text-primary")
            stage_label = ui.label("就绪").classes("text-body2 grow")
            cancel_button = ui.button("取消", icon="stop",
                                     on_click=lambda: on_cancel()).props(
                "flat dense color=negative")
        bar = ui.linear_progress(value=0.0, show_value=False).classes("w-full")
        with ui.expansion("日志（等价终端输出，最近 200 行）",
                          icon="terminal").classes("w-full text-caption"):
            log_view = ui.log(max_lines=jobs.LOG_LIMIT).classes(
                "w-full h-48 text-caption")
    progress_card.set_visibility(False)

    # ---------------- 结果区 ----------------
    @ui.refreshable
    def result_area() -> None:
        """结果区：统计卡 / 3D 图 / 路点表 / 段诊断 / 导出。"""
        result = state["result"]
        mode = _result_mode(result)
        if mode == "empty":
            with ui.card().classes("w-full"):
                ui.label("还没有结果").classes("text-subtitle1")
                ui.label("点「生成并解算」跑一次；只想看路径就点「预览路点」。"
                         ).classes("text-caption text-grey-7")
            return
        if mode == "cancelled":
            with ui.card().classes("w-full bg-grey-2"):
                ui.label("已取消").classes("text-subtitle1 text-grey-8")
                ui.label("任务在当前阶段末尾停止：没来得及落盘的运行目录已删掉，"
                         "已经写出 scan.json 的目录会保留。").classes(
                    "text-caption text-grey-7 whitespace-pre-wrap")
            return
        if mode == "failed":
            with ui.card().classes("w-full"):
                ui.label("失败").classes("text-subtitle1 text-negative")
                ui.label(str(result.get("error") or "未知错误")).classes(
                    "text-negative text-body2 whitespace-pre-wrap")
                for line in result.get("warnings") or []:
                    ui.label(line).classes(
                        "text-warning text-caption whitespace-pre-wrap")
            return

        preview = bool(result["meta"].get("skip_ik"))
        stats = result["stats"]
        with ui.card().classes("w-full gap-2"):
            with ui.row().classes("w-full items-center gap-2 flex-wrap"):
                if preview:
                    ui.chip(f"预览 {len(result['waypoints'])} 点（不跑 IK）",
                            icon="visibility", color="blue-grey")
                else:
                    ok = stats["solved"] == stats["total"]
                    ui.chip(f"成功 {stats['solved']}/{stats['total']}",
                            icon="check_circle" if ok else "error",
                            color="green" if ok else "red")
                ui.chip(f"构型切换 {stats['switches']}", icon="swap_horiz")
                if not preview:
                    ui.chip(f"ΣdqL2 {_fmt(stats['dq_l2_sum'], 2)}",
                            icon="timeline")
                    ui.chip(f"|J5|min {_fmt(stats['j5_abs_min'])}", icon="speed")
                    ui.chip(f"|J3|min {_fmt(stats['j3_abs_min'])}", icon="speed")
                ui.chip(f"design {_fmt(stats['design_cond'])}", icon="science")
                if not preview:
                    ui.chip(f"post_ik {_fmt(stats['post_ik_cond'])}",
                            icon="science")
                    feasible = stats.get("feasible")
                    ui.chip("可行" if feasible else "不可行",
                            icon="verified" if feasible else "report",
                            color="green" if feasible else "red")
            for line in result.get("warnings") or []:
                ui.label(f"· {line}").classes(
                    "text-warning text-caption whitespace-pre-wrap")
            ui.label(f"运行 {result['run_id']} · {result['run_dir']}").classes(
                "text-caption text-grey-7")

        # ---- 3D 图（切换着色依据时重建 figure，不重建元素） ----
        holder = {}

        def change_color(event) -> None:
            """切换着色依据：只更新 figure，不重画整个结果区。"""
            state["color_by"] = event.value
            if holder.get("plot") is not None:
                holder["plot"].update_figure(
                    _figure_for(state["result"], event.value))

        with ui.card().classes("w-full gap-1"):
            with ui.row().classes("w-full items-center gap-2 no-wrap"):
                ui.label("3D 路点").classes("text-subtitle1")
                ui.space()
                ui.toggle({"order": "按路径顺序", "margin": "按关节裕度"},
                          value=state["color_by"],
                          on_change=change_color).props("dense")
            holder["plot"] = ui.plotly(
                _figure_for(result, state["color_by"])).classes("w-full")

        # ---- 路点表 ----
        with ui.card().classes("w-full gap-1"):
            ui.label("路点表").classes("text-subtitle1")
            ui.label("点表头排序；失败行标红。").classes(
                "text-caption text-grey-7")
            waypoint_table = ui.table(
                columns=_columns(_PREVIEW_COLUMNS if preview
                                 else _WAYPOINT_COLUMNS),
                rows=_waypoint_rows(result),
                row_key="gp",
                pagination={"rowsPerPage": 20, "sortBy": "gp",
                            "descending": False},
            ).classes("w-full text-caption")
            waypoint_table.add_slot("body-row", _FAILED_ROW_SLOT)

        # ---- 段诊断表 ----
        with ui.card().classes("w-full gap-1"):
            ui.label("段诊断").classes("text-subtitle1")
            if not result.get("segments"):
                ui.label("预览模式不做段内检查。" if preview
                         else "没有任何相邻段（路点少于 2 个）。").classes(
                    "text-caption text-grey-7")
            else:
                ui.label("最差点 = 段内 |J3| 最小的位置（按关节直线解析求 t，"
                         "坐标沿线插值）。").classes("text-caption text-grey-7")
                ui.table(
                    columns=[{"name": "gp", "label": "GP 起→止", "field": "gp",
                              "align": "center", "sortable": True},
                             {"name": "j3", "label": "|J3|min °", "field": "j3",
                              "align": "right", "sortable": True},
                             {"name": "j5", "label": "|J5|min °", "field": "j5",
                              "align": "right", "sortable": True},
                             {"name": "crosses", "label": "穿零",
                              "field": "crosses", "align": "center",
                              "sortable": True},
                             {"name": "worst", "label": "最差点 (mm, t)",
                              "field": "worst", "align": "left"}],
                    rows=_segment_rows(result),
                    row_key="gp",
                    pagination={"rowsPerPage": 10}).classes(
                        "w-full text-caption")

        # ---- 导出 ----
        artifacts = result.get("artifacts") or {}
        run_dir = Path(result["run_dir"])
        labels = {"scan_json": "下载 scan.json", "joints": "下载 joints.txt",
                  "plot": "下载 plot.png"}
        with ui.row().classes("w-full items-center gap-2 flex-wrap"):
            ui.label("导出：").classes("text-caption text-grey-7")
            for key, button_label in labels.items():
                name = artifacts.get(key)
                if not name:
                    continue
                download_button(run_dir / name, name, button_label,
                                icon="download").props("flat dense")

    def _domains_too_big() -> bool:
        """当前表单算出来的点数是否超软上限（笔误保护，见 POINT_COUNT_WARN）。"""
        preview = _preview()
        return bool(preview.get("ok")
                    and preview["point_count"] > POINT_COUNT_WARN)

    def set_running(running: bool) -> None:
        """解算期间禁用两个按钮并显示取消；结束后按点数上限决定能否再点。"""
        blocked = (not running) and _domains_too_big()
        for button in (preview_button, solve_button):
            (button.disable if (running or blocked) else button.enable)()
        cancel_button.set_visibility(running)

    def poll() -> None:
        """轮询任务快照：刷进度、追加日志、完成后渲染结果。"""
        job_id = state["job_id"]
        if job_id is None:
            return
        snapshot = registry.snapshot(job_id, state["log_cursor"])
        if snapshot is None:               # 登记表被换掉/进程重启
            state["job_id"] = None
            poll_timer.deactivate()
            set_running(False)
            return
        state["log_cursor"] = snapshot["log_from"]
        for line in snapshot["log"]:
            log_view.push(line)
        progress = snapshot["progress"]
        bar.set_value(progress["fraction"])
        stage_text = progress["stage"]
        if progress["total"] > 0:
            stage_text += f" {progress['index']}/{progress['total']}"
        stage_label.set_text(f"{stage_text} · "
                             f"{_STATE_TEXT.get(snapshot['state'], snapshot['state'])}")
        if snapshot["state"] not in jobs.TERMINAL_STATES:
            return
        poll_timer.deactivate()
        state["job_id"] = None
        set_running(False)
        if snapshot["state"] == "done":
            bar.set_value(1.0)
            state["result"] = snapshot["result"]
            stage_label.set_text("完成")
            ui.notify("解算完成", type="positive")
        elif snapshot["state"] == "cancelled":
            state["result"] = cancelled_result()
            stage_label.set_text("已取消")
            ui.notify("任务已取消", type="warning")
        else:
            state["result"] = {"ok": False,
                              "error": snapshot["error"] or "任务失败",
                              "warnings": []}
            stage_label.set_text("失败")
            ui.notify("任务失败，见日志", type="negative")
        result_area.refresh()

    def start_job(extra: dict) -> None:
        """提交一次解算（``extra`` 决定走完整解算还是路点预览）。"""
        if state["job_id"] is not None:
            ui.notify("已有任务在跑，先等它结束或点取消", type="warning")
            return
        # 硬守卫：即使按钮因参数改变而没来得及重新禁用，也不放过超限提交
        if _domains_too_big():
            ui.notify(f"点数超过 {POINT_COUNT_WARN}：请减少份数或改用目标点数",
                      type="warning")
            return
        params = collect()
        params.update(extra)
        state["log_cursor"] = 0
        state["result"] = None
        state["color_by"] = "order"
        log_view.clear()
        bar.set_value(0.0)
        stage_label.set_text("排队")
        progress_card.set_visibility(True)
        set_running(True)
        result_area.refresh()

        def task(job: jobs.Job) -> dict:
            """在 jobs 的工作线程里跑：进度直接写回任务对象。"""
            return core.run_scan(params, job=job, on_progress=job.set_progress)

        try:
            job_id = registry.submit("scan", task)
        except OSError as error:           # runs/ 建不出来（磁盘满/无权限）
            ui.notify(f"无法创建运行目录：{error}", type="negative")
            stage_label.set_text("提交失败")
            set_running(False)
            return
        state["job_id"] = job_id
        poll_timer.activate()

    def on_cancel() -> None:
        """请求取消：阶段边界生效（IK 不会被打断）。"""
        job_id = state["job_id"]
        if job_id is None:
            return
        if registry.request_cancel(job_id):
            ui.notify("已请求取消，当前阶段结束后停止", type="info")
        else:
            ui.notify("任务已经结束", type="info")

    poll_timer = ui.timer(POLL_INTERVAL, poll, active=False)
    set_running(False)

    # 页面重建（F5 / 切页回来）时接管仍在跑的任务：否则任务不可见、取消不了，
    # 还会把新提交的任务堵在「排队中」（单消费者队列）。
    running_job = adopt_running_job(registry, "scan")
    if running_job is not None:
        state["job_id"] = running_job.id
        state["log_cursor"] = 0
        progress_card.set_visibility(True)
        set_running(True)
        stage_label.set_text("接管在跑的任务（上一次页面已关闭）")
        poll_timer.activate()

    result_area()
    switch_domain(defaults["domain_mode"])
    switch_grid(defaults["grid_mode"])
