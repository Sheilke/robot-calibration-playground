# -*- coding: utf-8 -*-
"""左侧导航 + 三个页面容器 + ``ui.run``。

三个页面共用一套布局：顶部标题条 + 左侧固定导航 + 右侧内容区；每个页面的
内容由 :mod:`laser_web.pages` 里对应的 ``render()`` 填充。
"""

from __future__ import annotations

from nicegui import ui

from .pages import generate, repair, tools

#: 页面表：路径 -> (标题, 图标, 渲染函数)。
PAGES = (("/", "生成", "route", generate.render),
         ("/repair", "奇异修复", "build", repair.render),
         ("/tools", "工具", "construction", tools.render))


def _layout(active: str) -> None:
    """画页面外壳（标题条 + 左导航）；之后的元素落进内容区。"""
    with ui.header().classes("items-center"):
        ui.label("标定路点工具").classes("text-h6")
        ui.space()
        ui.label("仅本机 127.0.0.1").classes("text-caption")

    with ui.left_drawer(value=True, fixed=True).classes("bg-grey-1"):
        ui.label("页面").classes("text-caption text-grey-7")
        for path, title, icon, _render in PAGES:
            with ui.row().classes("w-full items-center gap-2 no-wrap"):
                ui.icon(icon).classes(
                    "text-primary" if path == active else "text-grey-6")
                ui.link(title, path).classes(
                    "text-primary no-underline" if path == active
                    else "text-grey-9 no-underline")


def build_pages() -> None:
    """注册三个页面（生成 / 奇异修复 / 工具）。"""
    for path, title, _icon, render in PAGES:
        def _page(active: str = path, body=render) -> None:
            """单个页面的内容函数（工厂内绑定，避免闭包串页）。"""
            _layout(active)
            with ui.column().classes("w-full p-4 gap-3"):
                body()

        ui.page(path, title=f"{title} · 标定路点工具")(_page)


def run(host: str = "127.0.0.1", port: int = 8765) -> None:
    """构建页面并启动本地服务（不自动开浏览器、不启用 reload）。"""
    build_pages()
    ui.run(host=host, port=port, title="标定路点工具", reload=False, show=False)
