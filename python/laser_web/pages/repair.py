# -*- coding: utf-8 -*-
"""奇异修复页（占位，M4 实现）。

M1 只放说明与骨架：页面里没有任何按钮回调，避免让人误以为功能可用。
"""

from __future__ import annotations

from nicegui import ui


def render() -> None:
    """渲染占位页。"""
    with ui.card().classes("w-full gap-2"):
        ui.label("奇异修复（M4 实现）").classes("text-h6")
        ui.label("本页在 M1 只是骨架：没有输入框、没有按钮，也不会跑任何算法。")
        ui.label("计划内容（M4）：").classes("text-subtitle2")
        ui.label(
            "· 现场信息：已走到第几个 GP、停机时的关节角 q（可加多组）；"
            "采样域自动预填生成页上次的值。")
        ui.label(
            "· 段定位：用 avoid_singularity.segment_profiles 给出窗口内各段的 "
            "|J3|min / |J5|min、是否穿零、最差点位置与 t，并把报告段标到 3D 图上。")
        ui.label(
            "· 修复：先局部（允许改参考姿态 / 允许收缩边界），失败再整体搜索；"
            "结果给出改了哪个 GP 的 δrpy、相邻段变化、边界变化与体积损失。")
        ui.label(
            "· 退化读数（degenerate）单独给一句直白说明，不混在 wrist/elbow 里。"
        ).classes("text-caption")
