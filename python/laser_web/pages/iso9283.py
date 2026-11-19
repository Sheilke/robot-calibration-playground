# -*- coding: utf-8 -*-
"""ISO 9283 五点页：立方体（最大点 + 最小点）+ 参考姿态 -> P1~P5 笛卡尔坐标。

页面只做三件事：收参数（``collect``）、把参数交给 ``core.iso_points``、把结果
画出来。立方体怎么定、五点怎么摆、IK 怎么解全部在
``excitation_trajectory.iso9283_cube`` 里，本页不复制任何公式。

**这一页不显示关节量**（用户 2026-11-16 当面要求）：只算笛卡尔坐标，姿态固定
为用户填的参考姿态、不加扰动；IK 只解一次用来判断"这一点到不到得了"，关节角、
|J3| / |J5|、构型标签都不出现在界面上、也不写进 ``iso.json``。所以：

* 没有 ``q_seed`` / ``j5_min`` 输入（``core`` 的默认值照旧生效）；
* 不建后台任务：五点的坐标是纯几何（微秒级），可达性验算是 5 次 IK（毫秒级），
  点一下就算完，不需要进度条 / 取消 / 日志；
* 导出时才落盘（``iso.json`` + ``iso.png``，matplotlib 出图约 0.3 s，放在导出
  这条明确动作里，不阻塞"点一下出结果"）。

ISO 9283 只定义了一种五点摆法（立方体中心 + 对角截面四角），所以页面**不给**
“布点 / 截面”选项，也**不自己搜**立方体：立方体由现场按工作空间给两个极值点。
``layout`` / ``plane`` 仍然照旧提交给 ``core``（参数约定不变，``meta`` 里照旧落盘）。
"""

from __future__ import annotations

from pathlib import Path

from nicegui import ui

from .. import core, figures, ui_state
from .generate import _FAILED_ROW_SLOT, _columns, _number, _round

#: 立方体只有一种给法（最大点 + 最小点）；值给 ``core``，文案给人看。
_CUBE_MODE = "bounds"
#: 布点与截面按 ISO 9283 固定：XZ 对角截面的四个角。界面不给选项，但**仍然提交**。
_FIXED_LAYOUT = "vertices"
_FIXED_PLANE = "xz"
#: 五点编号说明（ISO 9283）：P2/P3 在小 X 小 Z 一侧，P4/P5 在大 X 大 Z 一侧。
_NUMBERING_NOTE = ("ISO 9283 五点：P1 立方体中心；P2/P3 在 X 小 Z 小一侧、"
                   "P4/P5 在 X 大 Z 大一侧（同侧两点差在 Y）；五点姿态完全相同，"
                   "用 MOVL 走直线。")
#: 页面自己的行为说明（一字不改地照用户给的说法）。
_HINT = ("只算笛卡尔坐标；姿态固定、不加扰动；系统会解一次 IK 判断每点是否可达，"
         "不输出关节角。")
#: 五点表列（名字, 表头, 字段, 对齐）：**没有** q1..q6 / |J3| / |J5| / cfg。
_POINT_COLUMNS = (
    ("name", "P", "name", "center"),
    ("x", "X (mm)", "x", "right"),
    ("y", "Y (mm)", "y", "right"),
    ("z", "Z (mm)", "z", "right"),
    ("reach", "可达", "reach", "center"),
)


def _point_rows(result: dict) -> list[dict]:
    """``core.iso_points()["points"]`` -> 表格行（保留数字类型，不可达行标红）。

    与「生成」页的 ``_waypoint_rows`` 同一约定：数值不转字符串（点表头要按
    大小排序），失败行由 ``_FAILED_ROW_SLOT`` 标红。
    """
    rows = []
    for point in (result or {}).get("points") or []:
        reachable = bool(point.get("reachable"))
        rows.append({"name": str(point["name"]),
                     "x": _round(point["pos"][0], 1),
                     "y": _round(point["pos"][1], 1),
                     "z": _round(point["pos"][2], 1),
                     "reach": "✓ 可达" if reachable else "✗ 不可达",
                     "failed": not reachable})
    return rows


def _summary_text(result: dict) -> str:
    """汇总行：``可达 N/5``（**不是** ``IK 成功 N/5``——这一页不报关节量）。"""
    summary = (result or {}).get("summary") or {}
    reachable = summary.get("reachable")
    total = summary.get("total")
    if reachable is None or total is None:
        return "可达 —"
    return f"可达 {int(reachable)}/{int(total)}"


def render() -> None:
    """渲染 ISO 五点页（表单 + 统计卡 + 3D + 五点表 + 导出）。"""
    defaults = core.iso_defaults()
    # 只列 Nexbot6R 真能加载的模型（同「生成」页：dh.json / scan.json /
    # robot_sim 资产会在探测阶段被判为不可用）。
    robots = [robot for robot in core.list_robots() if robot["usable"]]
    options = {robot["path"]: robot["label"] for robot in robots}
    default_robot = ui_state.default_robot()
    if default_robot not in options:      # CLI 默认模型被删掉时退回第一个
        default_robot = next(iter(options), None)

    fields = {}       # 表单控件
    state = {"result": None, "saved": None}

    def collect() -> dict:
        """把当前表单收成 ``core.iso_points`` 的入参。

        立方体只有一种给法（最大点 + 最小点），只提交 ``cube``；搜索相关的
        ``search_box`` / ``resolution`` / ``prefer_x_min`` 根本不存在。``layout``
        / ``plane`` 也不可选，但**照旧提交**（core 的参数约定不变）。
        ``q_seed`` / ``j5_min`` 不再提交——界面没有这两个输入，
        ``core`` 用自己的默认值（可达性验算的种子固定，见 ``core.iso_points``）。
        """
        return {"robot": fields["robot"].value,
                "cube_mode": _CUBE_MODE,
                "cube": [fields["x0"].value, fields["x1"].value,
                         fields["y0"].value, fields["y1"].value,
                         fields["z0"].value, fields["z1"].value],
                "layout": _FIXED_LAYOUT,
                "plane": _FIXED_PLANE,
                "rpy": [fields["rpy_roll"].value,
                        fields["rpy_pitch"].value,
                        fields["rpy_yaw"].value],
                "euler": fields["euler"].value}

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
                ui.label("立方体（给最大/最小点）").classes("text-subtitle1")
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

            with ui.card().classes("w-full"):
                ui.label("参考姿态").classes("text-subtitle1")
                ui.label(_HINT).classes("text-caption text-grey-7")
                with ui.row().classes("w-full no-wrap gap-2"):
                    for key, label, value in (
                            ("rpy_roll", "参考 A (roll)", defaults["rpy"][0]),
                            ("rpy_pitch", "参考 B (pitch)", defaults["rpy"][1]),
                            ("rpy_yaw", "参考 C (yaw)", defaults["rpy"][2])):
                        fields[key] = _number(label, value, None, step=1.0)
                fields["euler"] = ui.select(
                    ui_state.euler_convention_options(), value=defaults["euler"],
                    label="欧拉约定（内旋）").classes("w-full")

            with ui.row().classes("gap-2"):
                submit_button = ui.button(
                    "生成五点", icon="calculate",
                    on_click=lambda: start_job()).props("color=primary")

        # ---------------- 右：结果统计 ----------------
        with ui.column().classes("w-80 gap-3"):
            @ui.refreshable
            def stats_card() -> None:
                """右侧统计卡：可达 N/5 + 立方体边长/中心（没有关节量）。"""
                result = state["result"]
                saved = state["saved"] or {}
                with ui.card().classes("w-full gap-1"):
                    ui.label("结果").classes("text-subtitle1")
                    if result is None:
                        ui.label("点「生成五点」算一次。").classes(
                            "text-caption text-grey-7")
                        return
                    if not result.get("ok"):
                        ui.label(str(result.get("error") or "未知错误")).classes(
                            "text-negative text-body2 whitespace-pre-wrap")
                        return
                    summary = result.get("summary") or {}
                    meta = result.get("meta") or {}
                    reachable = int(summary.get("reachable") or 0)
                    total = int(summary.get("total") or 0)
                    with ui.grid(columns=2).classes("w-full gap-x-3"):
                        ui.label("可达").classes("text-grey-7")
                        ui.label(_summary_text(result)).classes(
                            "" if total and reachable == total else "text-negative")
                        ui.label("总点数").classes("text-grey-7")
                        ui.label(f"{total} 点")
                        ui.label("边长").classes("text-grey-7")
                        ui.label("—" if meta.get("cube_side") is None
                                 else f"{float(meta['cube_side']):.0f} mm")
                        ui.label("中心").classes("text-grey-7")
                        ui.label("—" if not meta.get("cube_center") else
                                 "(" + ", ".join(
                                     f"{float(value):.0f}"
                                     for value in meta["cube_center"]) + ") mm")
                        ui.label("姿态").classes("text-grey-7")
                        ui.label(" / ".join(f"{float(v):g}"
                                            for v in meta.get("rpy") or []) + "°")
                    if saved.get("run_dir"):
                        ui.label(f"已落盘 {saved['run_dir']}").classes(
                            "text-caption text-grey-7")

            stats_card()

    # ---------------- 结果区：3D / 五点表 / 导出 ----------------
    @ui.refreshable
    def detail_area() -> None:
        """3D 图（立方体 + 五点）、五点表、产物下载。"""
        result = state["result"]
        if result is None:
            with ui.card().classes("w-full"):
                ui.label("还没有结果").classes("text-subtitle1")
                ui.label("左边填两个极值点与参考姿态 -> 点「生成五点」。"
                         + _HINT).classes("text-caption text-grey-7")
            return
        if not result.get("ok"):
            with ui.card().classes("w-full"):
                ui.label("没法算").classes("text-subtitle1 text-negative")
                ui.label(str(result.get("error") or "未知错误")).classes(
                    "text-negative text-body2 whitespace-pre-wrap")
                for line in result.get("warnings") or []:
                    ui.label(f"· {line}").classes(
                        "text-warning text-caption whitespace-pre-wrap")
            return

        meta = result.get("meta") or {}
        for line in result.get("warnings") or []:
            ui.label(f"· {line}").classes(
                "text-warning text-caption whitespace-pre-wrap")

        with ui.card().classes("w-full gap-1"):
            ui.label("3D 五点").classes("text-subtitle1")
            ui.label("灰线是立方体 12 条棱；绿圆 = 可达，红叉 = 不可达。"
                     "hover 只看点名与 XYZ。").classes(
                "text-caption text-grey-7")
            ui.plotly(figures.iso_points_figure(
                result["points"],
                cube_center=meta.get("cube_center"),
                side=meta.get("cube_side"))).classes("w-full")

        with ui.card().classes("w-full gap-1"):
            ui.label("五点表").classes("text-subtitle1")
            ui.label(_NUMBERING_NOTE + "不可达的行标红。"
                     ).classes("text-caption text-grey-7")
            table = ui.table(columns=_columns(_POINT_COLUMNS),
                             rows=_point_rows(result),
                             row_key="name",
                             pagination={"rowsPerPage": 5}).classes(
                "w-full text-caption")
            table.add_slot("body-row", _FAILED_ROW_SLOT)

        with ui.row().classes("w-full items-center gap-2 flex-wrap"):
            ui.label("导出：").classes("text-caption text-grey-7")
            for key, label in (("iso_json", "下载 iso.json"),
                               ("plot", "下载 iso.png")):
                ui.button(label, icon="download",
                          on_click=lambda _event=None, wanted=key:
                          export(wanted)).props("flat dense")
        ui.label("iso.json 里是每点的 X/Y/Z + 参考姿态（五点姿态相同），"
                 "没有关节角；iso.png 是上面那张图。").classes(
            "text-caption text-grey-7")

    detail_area()

    def ensure_saved() -> dict:
        """导出前把 ``iso.json`` / ``iso.png`` 落到运行目录（只写一次）。"""
        result = state["result"] or {}
        if not result.get("ok"):
            return {}
        saved = state["saved"] or {}
        artifacts = saved.get("artifacts") or {}
        if (saved.get("run_dir") and artifacts
                and all((Path(saved["run_dir"]) / name).is_file()
                        for name in artifacts.values())):
            return saved
        try:
            saved = core.save_iso_points(result)
        except (OSError, ValueError) as error:
            ui.notify(f"无法写出产物：{error}", type="negative")
            return {}
        state["saved"] = saved
        stats_card.refresh()
        return saved

    def export(key: str) -> None:
        """下载一个产物：第一次点导出时才生成文件（出图约 0.3 s）。"""
        saved = ensure_saved()
        if not saved:
            return
        name = (saved.get("artifacts") or {}).get(key)
        target = Path(saved["run_dir"]) / str(name)
        if not name or not target.is_file():
            ui.notify(f"文件已不存在：{name or key}", type="negative")
            return
        ui.download.file(target, str(name))

    def start_job() -> None:
        """算一次：坐标是几何、可达性 5 次 IK，都在主线程算完（毫秒级）。"""
        params = collect()
        state["result"] = None
        state["saved"] = None
        submit_button.disable()
        try:
            result = core.iso_points(params)
        except Exception as error:          # noqa: BLE001 - 页面不能崩
            result = {"ok": False,
                      "error": f"算五点失败：{type(error).__name__}: {error}",
                      "warnings": []}
        finally:
            submit_button.enable()
        state["result"] = result
        stats_card.refresh()
        detail_area.refresh()
        if not result.get("ok"):
            ui.notify(result.get("error") or "算不了，见结果区", type="negative")
        elif (result.get("summary") or {}).get("reachable") == \
                (result.get("summary") or {}).get("total"):
            ui.notify(_summary_text(result), type="positive")
        else:
            ui.notify(_summary_text(result) + "：有不可达的点", type="warning")

    # 表单里不存在“来源 / 布点 / 截面”切换控件：立方体只有一种给法，
    # 布点与截面按 ISO 9283 固定（``_FIXED_LAYOUT`` / ``_FIXED_PLANE``）。
