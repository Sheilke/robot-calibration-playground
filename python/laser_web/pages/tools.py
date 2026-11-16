# -*- coding: utf-8 -*-
"""工具页（占位，M3 实现）。

M1 只放说明与骨架：JSON -> URDF 与 GP 导出都在 M3，页面里没有上传控件。
"""

from __future__ import annotations

from nicegui import ui


def render() -> None:
    """渲染占位页。"""
    with ui.card().classes("w-full gap-2"):
        ui.label("工具（M3 实现）").classes("text-h6")
        ui.label("本页在 M1 只是骨架：没有上传/下载控件，也不会写任何文件。")
        ui.label("计划内容（M3）：").classes("text-subtitle2")
        ui.label(
            "· JSON -> URDF：上传控制器 JSON、填型号，下载 <原文件名>_<型号>.urdf，"
            "并显示 FK 自校验误差（robot_model.json_to_urdf）。")
        ui.label(
            "· GP 导出：选一次运行结果 + 上传 position_R1 模板，按 --cartesian "
            "zero|fk、--cart-units、--euler 生成 GP 文件（export_gp.write_gp_entries）。")
        ui.label("· 清理：清空产物目录与日志（保留最近 50 次运行）。")
