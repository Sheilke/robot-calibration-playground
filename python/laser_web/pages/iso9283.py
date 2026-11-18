# -*- coding: utf-8 -*-
"""ISO 9283 五点页：立方体来源 + 布点 + 姿态 -> 后台解算 -> 统计 / 3D / 表格 / 导出。

页面只做三件事：收参数（``collect``）、把参数交给 ``core.run_iso9283`` 的后台
任务（``jobs.submit("iso", ...)``）、把结果画出来。立方体怎么定、五点怎么摆、
IK 怎么解全部在 ``excitation_trajectory.iso9283_cube`` 里，本页不复制任何公式。

自动搜索要在搜索箱里逐点二分边长（文档里的例子是 6 秒级），所以「生成五点」
**必须**走任务队列：进度条、日志、取消都来自任务快照的轮询，与「生成」页
同一套机制。
"""

from __future__ import annotations

from pathlib import Path

from nicegui import ui

from excitation_trajectory import iso9283_cube as iso

from .. import core, figures, jobs, ui_state
from . import adopt_running_job, cancelled_result, download_button
from .generate import (POLL_INTERVAL, _FAILED_ROW_SLOT, _STATE_TEXT, _columns,
                       _fmt, _number, _result_mode, _round)

#: 立方体来源 toggle（值给 ``core``，文案给人看）。
_CUBE_MODES = {"bounds": "给最大/最小点", "auto": "自动搜索内接"}
#: 布点方式（值来自 ``iso9283_cube.LAYOUTS``，这里只给中文文案）。
_LAYOUTS = {"vertices": "对角截面四角（ISO 标准）",
            "circle": "内切圆（旧实现）",
            "center-square": "原脚本布点"}
#: 对角截面（值来自 ``iso9283_cube.PLANES``）。
_PLANES = {"xz": "XZ 对角面（x=z）", "yz": "YZ 对角面（y=z）",
           "xy": "XY 对角面（x=y）"}
#: 「最大/最小点」的示例值：与「生成」页的示例采样域同源
#: （``ui_state.SAMPLE_POINT_*``）；立方体取各轴中点、最小跨度为边长。
#: 自动搜索的示例搜索箱（README「功能四」里跑过的那一组，x 朝机身外侧留开）。
_SAMPLE_SEARCH_BOX = (700.0, 1400.0, -600.0, 600.0, 200.0, 1300.0)
#: 「工具」页导 GP 的提示（``iso.json`` 与 ``scan.json`` 同构）。
_GP_HINT = ("可到「工具」页用这个 JSON 直接导 GP：iso.json 与 scan.json 同构"
            "（meta / waypoints / perturbed_ik），五个点姿态完全一致，"
            "现场按 MOVL 依次走。")
#: 五点表列（名字, 表头, 字段, 对齐）。
_POINT_COLUMNS = (
    ("name", "P", "name", "center"),
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
    ("ok", "ok", "ok", "center"),
)


def _point_rows(result: dict) -> list[dict]:
    """``run_iso9283()["points"]`` -> 表格行（保留数字类型，失败行标红）。

    与「生成」页的 ``_waypoint_rows`` 同一约定：数值不转字符串（点表头要按
    大小排序），``None`` 由 ``_FAILED_ROW_SLOT`` 画成 ``—``。
    """
    rows = []
    for point in result["points"]:
        joints = point.get("q") or [None] * 6
        config = point.get("config")
        row = {"name": point["name"],
               "x": _round(point["pos"][0], 1),
               "y": _round(point["pos"][1], 1),
               "z": _round(point["pos"][2], 1),
               "cfg": ("—" if config is None
                       else ",".join(str(int(tag)) for tag in config)),
               "ok": "✓" if point["ok"] else "✗",
               "failed": not point["ok"],
               "j3": _round(point.get("j3_abs"), 3),
               "j5": _round(point.get("j5_abs"), 3)}
        for index, key in enumerate(("q1", "q2", "q3", "q4", "q5", "q6")):
            row[key] = _round(joints[index], 3)
        rows.append(row)
    return rows


def _layout_label(value: str) -> str:
    """布点方式 -> 中文文案（未知值原样显示，不吞信息）。"""
    return _LAYOUTS.get(value, value)


def _plane_label(value: str) -> str:
    """对角截面 -> 中文文案（未知值原样显示）。"""
    return _PLANES.get(value, value)


def render() -> None:
    """渲染 ISO 五点页（表单 + 统计卡 + 3D + 表格 + 导出）。"""
    defaults = core.iso_defaults()
    # 只列 Nexbot6R 真能加载的模型（同「生成」页：dh.json / scan.json /
    # robot_sim 资产会在探测阶段被判为不可用）。
    robots = [robot for robot in core.list_robots() if robot["usable"]]
    options = {robot["path"]: robot["label"] for robot in robots}
    default_robot = ui_state.default_robot()
    if default_robot not in options:      # CLI 默认模型被删掉时退回第一个
        default_robot = next(iter(options), None)

    fields = {}       # 表单控件
    boxes = {}        # 需要整块显隐的容器（三种立方体来源）
    state = {"job_id": None, "log_cursor": 0, "result": None}
    registry = jobs.get_registry()

    def collect() -> dict:
        """把当前表单收成 ``core.run_iso9283`` 的入参（只提交激活的那一组）。"""
        params = {"robot": fields["robot"].value,
                  "cube_mode": fields["cube_mode"].value,
                  "layout": fields["layout"].value,
                  "plane": fields["plane"].value,
                  "square_frac": fields["square_frac"].value,
                  "rpy": [fields["rpy_roll"].value,
                          fields["rpy_pitch"].value,
                          fields["rpy_yaw"].value],
                  "euler": fields["euler"].value,
                  "q_seed": fields["q_seed"].value,
                  "j5_min": fields["j5_min"].value}
        mode = fields["cube_mode"].value
        if mode == "bounds":
            params["cube"] = [fields["x0"].value, fields["x1"].value,
                              fields["y0"].value, fields["y1"].value,
                              fields["z0"].value, fields["z1"].value]
        else:
            params["search_box"] = [fields["box_x0"].value,
                                    fields["box_x1"].value,
                                    fields["box_y0"].value,
                                    fields["box_y1"].value,
                                    fields["box_z0"].value,
                                    fields["box_z1"].value]
            params["resolution"] = fields["resolution"].value
            params["prefer_x_min"] = fields["prefer_x_min"].value
        return params

    def switch_mode(mode: str) -> None:
        """切换立方体来源（人工指定范围 / 自动搜索）：只显示当前那一组输入。"""
        for key, box in boxes.items():
            box.set_visibility(key == mode)

    def switch_layout(value: str) -> None:
        """``square_frac`` 只对 ``center-square`` 生效（其余布点用不到它）。"""
        (fields["square_frac"].enable if value == "center-square"
         else fields["square_frac"].disable)()

    # ---------------- 左：表单 ----------------
    with ui.row().classes("w-full no-wrap items-start gap-4"):
        with ui.column().classes("grow gap-3"):
            with ui.card().classes("w-full"):
                ui.label("机器人模型").classes("text-subtitle1")
                fields["robot"] = ui.select(
                    options, value=default_robot,
                    label="模型（data/configs 下，含 robot_sim/）"
                ).classes("w-full")
                ui.label().bind_text_from(
                    fields["robot"], "value",
                    lambda path: f"路径：{path}").classes("text-caption")

            with ui.card().classes("w-full"):
                ui.label("立方体来源").classes("text-subtitle1")
                fields["cube_mode"] = ui.toggle(
                    _CUBE_MODES, value="bounds",
                    on_change=lambda event: switch_mode(event.value)).props(
                        "spread")
                boxes["bounds"] = ui.column().classes("w-full gap-1")
                with boxes["bounds"]:
                    # 只要两个极值点：最小点 + 最大点（各 X/Y/Z 三个数）
                    for row_label, keys, values in (
                            ("最小点", ("x0", "y0", "z0"),
                             ui_state.SAMPLE_POINT_MIN),
                            ("最大点", ("x1", "y1", "z1"),
                             ui_state.SAMPLE_POINT_MAX)):
                        with ui.row().classes("w-full no-wrap gap-2 items-baseline"):
                            ui.label(row_label).classes("w-16 text-grey-7")
                            for key, axis, value in zip(keys, "XYZ", values):
                                fields[key] = _number(axis, value, None, step=1.0)
                    ui.label("只填这两个极值点；程序取各轴中点为中心、"
                             "最小跨度为立方体边长（三轴跨度不等时会提示）。"
                             ).classes("text-caption text-grey-7")
                boxes["auto"] = ui.column().classes("w-full gap-1")
                with boxes["auto"]:
                    for low_key, high_key, axis, values in (
                            ("box_x0", "box_x1", "X",
                             _SAMPLE_SEARCH_BOX[0:2]),
                            ("box_y0", "box_y1", "Y",
                             _SAMPLE_SEARCH_BOX[2:4]),
                            ("box_z0", "box_z1", "Z",
                             _SAMPLE_SEARCH_BOX[4:6])):
                        with ui.row().classes("w-full no-wrap gap-2"):
                            fields[low_key] = _number(
                                f"搜索 {axis} 下界", values[0], None, step=1.0)
                            fields[high_key] = _number(
                                f"搜索 {axis} 上界", values[1], None, step=1.0)
                    with ui.row().classes("w-full no-wrap gap-2"):
                        fields["resolution"] = _number(
                            "搜索步长（mm）", defaults["resolution"], None,
                            min=1.0, step=10.0)
                        fields["prefer_x_min"] = _number(
                            "中心 X 不小于（mm，可空）",
                            defaults["prefer_x_min"], None, step=10.0)
                    ui.label("按 8 个顶点都可解来判可行，逐箱二分边长；"
                             "步长越小越慢（默认 100 mm）。").classes(
                        "text-caption text-grey-7")

            with ui.card().classes("w-full"):
                ui.label("布点与截面").classes("text-subtitle1")
                fields["layout"] = ui.select(
                    _LAYOUTS, value=defaults["layout"], label="布点",
                    on_change=lambda event: switch_layout(event.value)
                ).classes("w-full")
                fields["plane"] = ui.select(
                    _PLANES, value=defaults["plane"],
                    label="对角截面（只影响 vertices / circle）"
                ).classes("w-full")
                fields["square_frac"] = _number(
                    "center-square 的 k（相对半边长）", defaults["square_frac"],
                    None, min=0.01, max=1.0, step=0.1)

            with ui.card().classes("w-full"):
                ui.label("姿态与 IK 种子").classes("text-subtitle1")
                ui.label("五点用同一姿态；现场按 MOVL 依次移动，不要用 PTP。"
                         ).classes("text-caption text-grey-7")
                with ui.row().classes("w-full no-wrap gap-2"):
                    for key, label, value in (
                            ("rpy_roll", "参考 A (roll)", defaults["rpy"][0]),
                            ("rpy_pitch", "参考 B (pitch)", defaults["rpy"][1]),
                            ("rpy_yaw", "参考 C (yaw)", defaults["rpy"][2])):
                        fields[key] = _number(label, value, None, step=1.0)
                fields["euler"] = ui.select(
                    ui_state.euler_convention_options(), value=defaults["euler"],
                    label="欧拉约定（内旋）").classes("w-full")
                fields["q_seed"] = ui.input(
                    "q-seed（6 个关节角，逗号分隔，度）",
                    value=defaults["q_seed"]).classes("w-full")
                fields["j5_min"] = _number(
                    "|J5| 告警下限（度，0 = 只报告不告警）",
                    defaults["j5_min"], None, min=0.0, step=1.0)

            with ui.row().classes("gap-2"):
                submit_button = ui.button(
                    "生成五点", icon="play_arrow",
                    on_click=lambda: start_job()).props("color=primary")

        # ---------------- 右：结果统计 ----------------
        with ui.column().classes("w-80 gap-3"):
            @ui.refreshable
            def stats_card() -> None:
                """右侧统计卡：这一次跑出来的关键数字。"""
                result = state["result"]
                mode = _result_mode(result)
                with ui.card().classes("w-full gap-1"):
                    ui.label("结果").classes("text-subtitle1")
                    if mode == "empty":
                        ui.label("点「生成五点」跑一次。").classes(
                            "text-caption text-grey-7")
                    elif mode == "cancelled":
                        ui.label("已取消").classes("text-body2 text-grey-8")
                    elif mode == "failed":
                        ui.label(str(result.get("error") or "未知错误")).classes(
                            "text-negative text-body2 whitespace-pre-wrap")
                    else:
                        stats = result["stats"]
                        with ui.grid(columns=2).classes("w-full gap-x-3"):
                            ui.label("总点数").classes("text-grey-7")
                            ui.label(f"{stats['total']} 点").classes("")
                            ui.label("可达").classes("text-grey-7")
                            ok = stats["solved"] == stats["total"]
                            ui.label(f"{stats['solved']}/{stats['total']}").classes(
                                "" if ok else "text-negative")
                            ui.label("构型").classes("text-grey-7")
                            ui.label("一致" if stats["config_consistent"]
                                     else "不一致").classes(
                                "" if stats["config_consistent"]
                                else "text-negative")
                            ui.label("|J3|min").classes("text-grey-7")
                            ui.label(f"{_fmt(stats['j3_abs_min'])}°")
                            ui.label("|J5|min").classes("text-grey-7")
                            ui.label(f"{_fmt(stats['j5_abs_min'])}°")
                            ui.label("边长").classes("text-grey-7")
                            ui.label("—" if stats["cube_side"] is None
                                     else f"{stats['cube_side']:.0f} mm")
                            ui.label("中心").classes("text-grey-7")
                            ui.label("—" if not stats["cube_center"] else
                                     "(" + ", ".join(
                                         f"{float(value):.0f}"
                                         for value in stats["cube_center"]) + ") mm")
                            ui.label("布点").classes("text-grey-7")
                            ui.label(_layout_label(stats["layout"]))
                            ui.label("截面").classes("text-grey-7")
                            ui.label(_plane_label(stats["plane"]))
                        if stats["configs"]:
                            ui.label("构型标签 "
                                     + " / ".join(
                                         ",".join(str(int(tag)) for tag in row)
                                         for row in stats["configs"])
                                     ).classes("text-caption text-grey-7")

            stats_card()

    # ---------------- 进度与日志 ----------------
    progress_card = ui.card().classes("w-full")
    with progress_card:
        with ui.row().classes("w-full items-center gap-3 no-wrap"):
            ui.icon("deployed_code").classes("text-primary")
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

    # ---------------- 结果区：3D / 五点表 / 导出 ----------------
    @ui.refreshable
    def detail_area() -> None:
        """3D 图（立方体 + 五点）、五点表、产物下载。"""
        result = state["result"]
        mode = _result_mode(result)
        if mode == "empty":
            with ui.card().classes("w-full"):
                ui.label("还没有结果").classes("text-subtitle1")
                ui.label("左边填参数 -> 点「生成五点」。自动搜索会在后台试几十个"
                         "候选中心，进度与日志在上面的卡片里。").classes(
                    "text-caption text-grey-7")
            return
        if mode == "cancelled":
            with ui.card().classes("w-full bg-grey-2"):
                ui.label("已取消").classes("text-subtitle1 text-grey-8")
                ui.label("任务在当前阶段末尾停止：没来得及落盘的运行目录已删掉，"
                         "已经写出结果的目录会保留。").classes(
                    "text-caption text-grey-7 whitespace-pre-wrap")
            return
        if mode == "failed":
            with ui.card().classes("w-full"):
                ui.label("失败").classes("text-subtitle1 text-negative")
                ui.label(str(result.get("error") or "未知错误")).classes(
                    "text-negative text-body2 whitespace-pre-wrap")
                for line in result.get("warnings") or []:
                    ui.label(f"· {line}").classes(
                        "text-warning text-caption whitespace-pre-wrap")
                if result.get("artifacts") and result.get("run_dir"):
                    # 有点无解时产物仍然落下（PNG 能看出该往哪个方向缩立方体）
                    ui.label(f"已落盘 {result['run_dir']}："
                             f"{', '.join(result['artifacts'].values())}").classes(
                        "text-caption text-grey-7")
            return

        stats = result["stats"]
        for line in result.get("warnings") or []:
            ui.label(f"· {line}").classes(
                "text-warning text-caption whitespace-pre-wrap")
        ui.label(f"运行 {result['run_id']} · {result['run_dir']}").classes(
            "text-caption text-grey-7")

        with ui.card().classes("w-full gap-1"):
            ui.label("3D 五点").classes("text-subtitle1")
            ui.label("灰线是立方体 12 条棱（人工指定五点时不画）；"
                     "绿圆 = 有解，红叉 = 无解。"
                     "hover 看点名 / XYZ / |J3| / |J5| / cfg / ok。").classes(
                "text-caption text-grey-7")
            ui.plotly(figures.iso_cube_figure(
                result["points"],
                cube_center=stats["cube_center"],
                side=stats["cube_side"],
                layout=stats["layout"],
                plane=stats["plane"])).classes("w-full")

        with ui.card().classes("w-full gap-1"):
            ui.label("五点表").classes("text-subtitle1")
            ui.label("P1 是立方体中心，P2~P5 是截面的四个角；失败行标红。"
                     ).classes("text-caption text-grey-7")
            table = ui.table(columns=_columns(_POINT_COLUMNS),
                             rows=_point_rows(result),
                             row_key="name",
                             pagination={"rowsPerPage": 5}).classes(
                "w-full text-caption")
            table.add_slot("body-row", _FAILED_ROW_SLOT)

        artifacts = result.get("artifacts") or {}
        run_dir = Path(result["run_dir"])
        labels = {"iso_json": "下载 iso.json", "plot": "下载 iso.png"}
        with ui.row().classes("w-full items-center gap-2 flex-wrap"):
            ui.label("导出：").classes("text-caption text-grey-7")
            for key, button_label in labels.items():
                name = artifacts.get(key)
                if not name:
                    continue
                download_button(run_dir / name, name, button_label,
                                icon="download").props("flat dense")
        ui.label(_GP_HINT).classes("text-caption text-grey-7")

    detail_area()

    def set_running(running: bool) -> None:
        """解算期间禁用提交按钮并显示取消。"""
        (submit_button.disable if running else submit_button.enable)()
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
            if state["result"].get("ok"):
                ui.notify("五点解算完成", type="positive")
            else:
                ui.notify("有解不出的点，见结果区", type="warning")
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
        stats_card.refresh()
        detail_area.refresh()

    def start_job() -> None:
        """提交一次解算（自动搜索可能几秒，一律走后台任务）。"""
        if state["job_id"] is not None:
            ui.notify("已有任务在跑，先等它结束或点取消", type="warning")
            return
        params = collect()
        state["log_cursor"] = 0
        state["result"] = None
        log_view.clear()
        bar.set_value(0.0)
        stage_label.set_text("排队")
        progress_card.set_visibility(True)
        set_running(True)
        stats_card.refresh()
        detail_area.refresh()

        def task(job: jobs.Job) -> dict:
            """在 jobs 的工作线程里跑：进度直接写回任务对象。"""
            return core.run_iso9283(params, job=job,
                                    on_progress=job.set_progress)

        try:
            job_id = registry.submit("iso", task)
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

    # 页面重建（F5 / 切页回来）时接管仍在跑的任务：否则任务不可见、取消不了。
    running_job = adopt_running_job(registry, "iso")
    if running_job is not None:
        state["job_id"] = running_job.id
        state["log_cursor"] = 0
        progress_card.set_visibility(True)
        set_running(True)
        stage_label.set_text("接管在跑的任务（上一次页面已关闭）")
        poll_timer.activate()

    # 初始化必须用 toggle 的**当前值**，不能写死模式名：写死一个已不存在的
    # 模式（曾经是 "center"）会让两组输入全部隐藏，页面看起来"没有输入框"。
    switch_mode(fields["cube_mode"].value)
    switch_layout(defaults["layout"])
