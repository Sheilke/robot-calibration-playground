# -*- coding: utf-8 -*-
"""生成页（M1）：机器人模型 + 采样域表单 + 预览卡片；**不跑 IK**。

任何输入变化都会重算预览卡片（体积 / 网格 / 点数 / 设计条件数，本机实测
约 1 ms，远低于 100 ms）。校验失败时把 CLI 的报错原文红字显示在卡片里，
不弹窗。两个动作按钮在 M1 只占位，真正的路点预览与解算留给 M2。

表单状态全部是 :func:`render` 的局部变量（每个客户端一份），没有模块级
可变对象，因此多标签页/刷新不会串台。
"""

from __future__ import annotations

import math

from nicegui import ui

from .. import core
from ..ui_state import default_form_values, euler_conventions


def _fmt_condition(value) -> str:
    """条件数显示：非有限值写 inf，其余 3 位小数（与 CLI 的显示一致）。"""
    if value is None:
        return "n/a"
    number = float(value)
    if math.isinf(number):
        return "inf" if number > 0 else "-inf"
    if math.isnan(number):
        return "n/a"
    return f"{number:.3f}"


def _number(label: str, value, on_change, **kwargs) -> ui.number:
    """建一个占满一行的数字输入框（默认值来自 ``ui_state`` 的工厂）。"""
    return ui.number(label, value=value, on_change=on_change,
                     **kwargs).classes("grow")


def render() -> None:
    """渲染生成页。"""
    defaults = default_form_values()
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

    fields = {}      # 表单控件（ui.number / ui.select / ui.toggle），按名字索引
    boxes = {}       # 需要整块显隐的容器（两点/六数、凑数/手填）

    def collect() -> dict:
        """把当前表单值收成 ``core.preview_domain`` 的入参。

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

    @ui.refreshable
    def preview_card() -> None:
        """预览卡片：体积 / 网格 / 点数 / 设计条件数，或一行红字报错。"""
        result = core.preview_domain(collect())
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
                ui.label(f"{result['point_count']} 点")
                ui.label("设计条件数").classes("text-grey-7")
                ui.label(_fmt_condition(result["design_condition"]))
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
                fields["cond_max"] = _number(
                    "条件数上限 cond_max", defaults["cond_max"],
                    preview_card.refresh, min=1.0, step=0.5)

            with ui.row().classes("gap-2"):
                ui.button("预览路点",
                          on_click=lambda: ui.notify("M2 实现", type="info"))
                ui.button("生成并解算",
                          on_click=lambda: ui.notify("M2 实现", type="info"))

        # ---------------- 右：预览卡片 ----------------
        with ui.column().classes("w-80 gap-3"):
            preview_card()

    switch_domain(defaults["domain_mode"])
    switch_grid(defaults["grid_mode"])
