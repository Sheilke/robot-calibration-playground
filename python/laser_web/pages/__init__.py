# -*- coding: utf-8 -*-
"""页面包：生成页、奇异修复页、ISO 五点页、工具页。

每个页面模块只暴露一个 ``render()``，由 :mod:`laser_web.app` 放进各自的
页面容器里；页面只负责“收集参数 -> 调 core -> 渲染”，不出现算法公式。

这里另外放几个页面共用的 UI 小工具（下载守卫、任务接管、取消占位），
避免各页各写一份、行为不一致。
"""

from __future__ import annotations

from pathlib import Path

from nicegui import ui

from .. import jobs

#: 路点数的软上限：超过它就在预览卡片标红并禁用"生成"按钮。
#: 8M 点会在 ``plan_waypoints`` 里直接把内存吃穿（评审实测 xs=200 即 8120601 点），
#: 而取消只在阶段边界生效，一旦提交就只能等或杀进程。
POINT_COUNT_WARN = 5000


def cancelled_result() -> dict:
    """取消后的结果占位（两个页面共用，形状与 ``run_scan`` 的失败返回一致）。

    ``cancelled=True`` 是关键：取消是现场工程师主动点的，不是算法出错，页面
    据此画灰色中性卡片而不是红字"失败"。没有这个键的话，``{"ok": False}``
    只能被当成失败渲染。

    Returns:
        dict: ``{"ok": False, "cancelled": True, "error", "warnings"}``。
    """
    return {"ok": False, "cancelled": True, "error": "任务已取消",
            "warnings": []}


def download_button(path, filename: str, label: str | None = None, **kwargs):
    """下载按钮：文件不存在时给提示，而不是静默无反应。

    ``ui.download.file`` 在 ``single_use`` 路由里对缺失文件会抛
    ``FileNotFoundError``，异常只进服务端日志、浏览器端毫无反馈——现场工程师
    会以为按钮坏了。运行目录可能被"保留最近 50 次"的策略或工具页的清理删掉，
    所以每次点击都先查存在性。

    Args:
        path: 要下载的文件路径（``str`` 或 ``Path``）。
        filename: 浏览器端保存的文件名。
        label: 按钮文字；``None`` 时用"下载 <filename>"。
        **kwargs: 透传给 ``ui.button``（如 ``icon``）。

    Returns:
        ``ui.button`` 元素。
    """
    target = Path(path)

    def on_click() -> None:
        if not target.is_file():
            ui.notify(f"文件已不存在：{filename}（运行目录可能已被清理）",
                      type="negative")
            return
        try:
            ui.download.file(target, filename)
        except FileNotFoundError:
            ui.notify(f"文件已不存在：{filename}", type="negative")

    return ui.button(label or f"下载 {filename}", on_click=on_click, **kwargs)


def adopt_running_job(registry, kind: str):
    """找一个还在跑的同类型任务，供页面重建时接管。

    任务跑在 ``jobs`` 的工作线程里，与页面生命周期无关：修复任务要 15 s，期间
    按 F5 或切页面，页面局部的 ``job_id`` 就丢了，于是任务既看不到也取消不了，
    还会把新任务堵在"排队中"。页面渲染时调用本函数把第一个未结束的任务捡回来。

    Args:
        registry: ``jobs.JobRegistry``。
        kind: ``"scan"`` / ``"repair"`` / ``"iso"``。

    Returns:
        ``Job`` 或 ``None``（没有在跑的同类型任务）。
    """
    for job in registry.jobs():
        if job.kind == str(kind) and job.state not in jobs.TERMINAL_STATES:
            return job
    return None
