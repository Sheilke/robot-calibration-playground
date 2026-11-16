# -*- coding: utf-8 -*-
"""工具页（M3）：GP 导出 / JSON -> URDF / 运行目录清理。

三张卡片各管一件事，页面只做「收集输入 -> 调 :mod:`laser_web.core` -> 渲染」，
算法与文件读取全部在服务层：

1. **GP 导出**：选一次运行（``core.list_runs()``，标出有没有 ``scan.json``）+
   上传 ``position_R1.json`` 模板 -> ``core.build_gp`` 写 GP 副本，显示写了几
   个、GP 号区间、模板 ``controllerID`` / ``version`` 未被改动，并给下载按钮；
2. **JSON -> URDF**：上传控制器 JSON + 填型号 -> ``core.convert_urdf``，
   显示 ``<原主名>_<型号>.urdf`` 与自校验误差（``4.547e-13`` 这类）；型号非法
   时显示 ``json_to_urdf`` 的原始报错；
3. **运行目录**：表格列出 ``runs/*/``（run_id / 时间 / 大小 / 产物标记），
   「清理（保留最近 50）」调 ``core.clean_runs`` 并刷新表格。

表单状态全部是 :func:`render` 的局部变量（每个客户端一份，见设计文档 §7.8）；
上传内容只存在内存里，脚本文件一律落到 ``core`` 指定的目录（运行目录或系统
临时目录），不在仓库里写任何东西。
"""

from __future__ import annotations

from datetime import datetime

from nicegui import ui

from .. import core, jobs

#: GP 导出表单的默认值（与 ``export_gp`` 的 CLI 同源，见 ``core.gp_defaults``）。
_GP_DEFAULTS = core.gp_defaults()
#: 运行下拉框为空时的提示。
_NO_RUN_HINT = "还没有运行记录：先在「生成」页跑一次，或点「刷新」"
#: 产物标记的显示顺序（键与 ``core.list_runs`` 的 ``has_*`` 对应）。
_ARTIFACTS = (("scan", "scan"), ("plot", "plot"), ("joints", "joints"),
              ("repair", "repair"))


def _fmt_size(size_bytes: int) -> str:
    """字节数 -> 人类可读（表格里只求一眼看出量级）。"""
    value = float(size_bytes)
    if value < 1024:
        return f"{value:.0f} B"
    value /= 1024
    if value < 1024:
        return f"{value:.1f} KB"
    return f"{value / 1024:.1f} MB"


def _fmt_time(created: float) -> str:
    """运行目录的 mtime -> ``YYYY-mm-dd HH:MM:SS``（也就等于它的创建时间）。"""
    return datetime.fromtimestamp(float(created)).strftime("%Y-%m-%d %H:%M:%S")


def _run_label(row: dict) -> str:
    """下拉框里一项的文案：``run_id · 时间 · 有没有 scan.json``。"""
    mark = "有 scan.json" if row["has_scan"] else "缺 scan.json"
    return f"{row['run_id']} · {_fmt_time(row['created'])} · {mark}"


def _artifact_marks(row: dict) -> str:
    """一行产物标记（✓ = 有，✗ = 没有）。"""
    return " ".join(f"{label}{'✓' if row[f'has_{key}'] else '✗'}"
                    for key, label in _ARTIFACTS)


def _download_button(path: str, filename: str, label: str | None = None) -> None:
    """一个下载按钮（``ui.download.file`` 是 NiceGUI 4.0 的写法）。"""
    ui.button(label or f"下载 {filename}", icon="download",
              on_click=lambda target=path, name=filename:
              ui.download.file(target, name)).props("unelevated")


def render() -> None:
    """渲染工具页（三张卡片）。"""
    # ---- 每个客户端一份状态：上传内容与上一次的返回 ----
    state: dict = {"template": None, "gp": None, "json": None, "urdf": None}

    @ui.refreshable
    def gp_result_area() -> None:
        """GP 导出结果（成功给下载按钮，失败给报错原文）。"""
        result = state["gp"]
        if not result:
            ui.label("尚未生成。模板只读：文件不会被改动，写的是运行目录下的"
                     "副本。").classes("text-caption text-grey-7")
            return
        if not result.get("ok"):
            ui.label(f"失败：{result.get('error')}").classes("text-negative")
        else:
            ui.label(f"已写入 {result['count']} 个 GP："
                     f"GP{result['first']:04d} .. GP{result['last']:04d}"
                     ).classes("text-subtitle2")
            meta = result.get("meta") or {}
            ui.label(f"模板 controllerID={meta.get('controllerID')} · "
                     f"version={meta.get('version')} —— 已回读核对，未改动"
                     ).classes("text-caption text-positive")
            ui.label(f"输出：{result['path']}").classes("text-caption text-grey-7")
            _download_button(result["path"], result["filename"])
        for text in result.get("warnings") or []:
            ui.label(f"· {text}").classes("text-caption text-orange-9")

    @ui.refreshable
    def urdf_result_area() -> None:
        """JSON -> URDF 结果（成功给文件名 + 自校验误差 + 下载按钮）。"""
        result = state["urdf"]
        if not result:
            ui.label("尚未转换。型号只允许字母数字与 . _ -，它会成为文件名"
                     "后缀。").classes("text-caption text-grey-7")
            return
        if not result.get("ok"):
            ui.label(f"失败：{result.get('error')}").classes("text-negative")
        else:
            ui.label(f"已生成 {result['filename']}").classes("text-subtitle2")
            error = result.get("self_check_error")
            text = "（没能从输出里解析出自校验误差）" if error is None \
                else f"自校验误差 {error:.3e} mm/rad（< 1e-9 视为 FK 等价）"
            ui.label(text).classes("text-caption text-positive" if error is not None
                                   and error < 1e-9 else "text-caption text-orange-9")
            ui.label(f"输出：{result['path']}").classes("text-caption text-grey-7")
            _download_button(result["path"], result["filename"])
        for text in result.get("warnings") or []:
            ui.label(f"· {text}").classes("text-caption text-orange-9")

    # ------------------------------------------------------------------ 卡片 1
    with ui.card().classes("w-full gap-2"):
        ui.label("GP 导出").classes("text-h6")
        ui.label("把一次扫描的关节角写进 position_R1 模板的副本：14 字段布局"
                 "（笛卡尔 6 + 标志 1 + 关节 6 + 外轴 1），与 "
                 "excitation_trajectory.export_gp 完全同一套。"
                 ).classes("text-caption text-grey-7")
        run_select = ui.select(options={}, label="运行（runs/<id>）",
                               with_input=True).classes("w-full")
        run_hint = ui.label("").classes("text-caption text-grey-7")
        template_upload = ui.upload(
            label="上传 GP 模板（position_R1.json）", auto_upload=True,
        ).props("accept=.json flat bordered").classes("w-full")
        template_hint = ui.label("尚未上传模板。").classes("text-caption text-grey-7")
        with ui.row().classes("w-full items-center gap-2 flex-wrap"):
            first_input = ui.number("起始 GP 号 --first",
                                    value=_GP_DEFAULTS["first"], format="%d",
                                    min=0, step=1).classes("w-32")
            flag_input = ui.number("标志位 --flag", value=_GP_DEFAULTS["flag"],
                                   format="%d", step=1).classes("w-32")
            cartesian_select = ui.select(list(core.GP_CARTESIAN_MODES),
                                         label="笛卡尔 --cartesian",
                                         value=_GP_DEFAULTS["cartesian"]
                                         ).classes("w-40")
            units_select = ui.select(list(core.GP_CART_UNITS),
                                     label="姿态单位 --cart-units",
                                     value=_GP_DEFAULTS["cart_units"]
                                     ).classes("w-40")
            euler_select = ui.select(core.gp_euler_conventions(),
                                     label="欧拉约定 --euler",
                                     value=_GP_DEFAULTS["euler"]).classes("w-48")
        build_button = ui.button("生成 GP", icon="playlist_add").props("unelevated")
        gp_result_area()

        def toggle_euler() -> None:
            """只有 ``--cartesian fk`` 才用欧拉约定（CLI 也是这个语义）。"""
            (euler_select.enable if cartesian_select.value == "fk"
             else euler_select.disable)()

        cartesian_select.on_value_change(lambda _event: toggle_euler())
        toggle_euler()

        async def on_template(event) -> None:
            """收模板文件：内容只留在内存里（模板本身不落地、不修改）。"""
            data = await event.file.read()
            state["template"] = {"name": event.file.name, "bytes": data}
            template_hint.set_text(
                f"模板：{event.file.name}（{len(data)} 字节，只读引用）"
            ).classes(remove="text-grey-7", add="text-positive")

        template_upload.on_upload(on_template)

        def generate_gp() -> None:
            """「生成 GP」：调 core.build_gp 并把结果交给结果区渲染。"""
            run_id = run_select.value
            if not run_id:
                ui.notify(_NO_RUN_HINT, type="warning")
                return
            template = state["template"]
            if template is None:
                ui.notify("先上传 GP 模板（position_R1.json）", type="warning")
                return
            row = next((item for item in core.list_runs()
                        if item["run_id"] == run_id), None)
            params = {"run_dir": row["dir"] if row else run_id,
                      "template": template["bytes"],
                      "template_name": template["name"],
                      "first": first_input.value,
                      "flag": flag_input.value,
                      "cartesian": cartesian_select.value,
                      "cart_units": units_select.value,
                      "euler": euler_select.value}
            state["gp"] = core.build_gp(params)
            gp_result_area.refresh()
            if state["gp"].get("ok"):
                ui.notify(f"已写入 {state['gp']['count']} 个 GP", type="positive")
            else:
                ui.notify("GP 导出失败，见下方报错", type="negative")

        build_button.on_click(generate_gp)

    # ------------------------------------------------------------------ 卡片 2
    with ui.card().classes("w-full gap-2"):
        ui.label("JSON → URDF").classes("text-h6")
        ui.label("控制器配置（Robot_A.json）转标准 URDF：自校验用 200 个随机位姿"
                 "比对 DH-FK 与 URDF-FK，误差 < 1e-9 才算通过。"
                 ).classes("text-caption text-grey-7")
        json_upload = ui.upload(label="上传控制器 JSON", auto_upload=True,
                                ).props("accept=.json flat bordered").classes("w-full")
        json_hint = ui.label("尚未上传控制器 JSON。").classes(
            "text-caption text-grey-7")
        with ui.row().classes("w-full items-center gap-2 flex-wrap"):
            model_input = ui.input("型号 --model（可空，会成为文件名后缀）"
                                   ).classes("grow")
            ui.button("转换", icon="transform",
                      on_click=lambda: convert_now()).props("unelevated")
        urdf_result_area()

        async def on_json(event) -> None:
            """收控制器 JSON：内容留在内存，转换时才落临时目录。"""
            data = await event.file.read()
            state["json"] = {"name": event.file.name, "bytes": data}
            json_hint.set_text(
                f"文件：{event.file.name}（{len(data)} 字节）"
            ).classes(remove="text-grey-7", add="text-positive")

        json_upload.on_upload(on_json)

        def convert_now() -> None:
            """「转换」：调 core.convert_urdf 并刷新结果区。"""
            payload = state["json"]
            if payload is None:
                ui.notify("先上传控制器 JSON", type="warning")
                return
            state["urdf"] = core.convert_urdf(payload["bytes"],
                                              model_input.value,
                                              filename=payload["name"])
            urdf_result_area.refresh()
            if state["urdf"].get("ok"):
                ui.notify(f"已生成 {state['urdf']['filename']}", type="positive")
            else:
                ui.notify("转换失败，见下方报错", type="negative")

    # ------------------------------------------------------------------ 卡片 3
    with ui.card().classes("w-full gap-2"):
        ui.label("运行目录").classes("text-h6")
        ui.label(f"runs/ 下的产物（每次运行一个目录），自动保留最近 "
                 f"{jobs.KEEP_RUNS} 个。删除只碰 runs/ 下的子目录。"
                 ).classes("text-caption text-grey-7")

        @ui.refreshable
        def runs_area() -> None:
            """运行目录表 + 清理按钮（清理后整块重建）。"""
            rows = core.list_runs()
            with ui.row().classes("w-full items-center gap-2"):
                ui.label(f"共 {len(rows)} 个运行目录"
                         ).classes("text-subtitle2 grow")
                ui.button("刷新", icon="refresh",
                          on_click=lambda: refresh_all()).props("flat dense")
                ui.button(f"清理（保留最近 {jobs.KEEP_RUNS}）", icon="delete_sweep",
                          on_click=lambda: clean_now()).props("flat dense")
            if not rows:
                ui.label(_NO_RUN_HINT).classes("text-caption text-grey-7")
                return
            ui.table(
                columns=[{"name": "run_id", "label": "运行", "field": "run_id",
                          "align": "left", "sortable": True},
                         {"name": "created", "label": "时间",
                          "field": "created", "align": "left", "sortable": True},
                         {"name": "size", "label": "大小", "field": "size",
                          "align": "right", "sortable": True},
                         {"name": "artifacts", "label": "产物 (scan/plot/joints/repair)",
                          "field": "artifacts", "align": "left"}],
                rows=[{"run_id": row["run_id"],
                       "created": _fmt_time(row["created"]),
                       "size": _fmt_size(row["size_bytes"]),
                       "size_bytes": row["size_bytes"],
                       "artifacts": _artifact_marks(row)}
                      for row in rows],
                row_key="run_id",
                pagination={"rowsPerPage": 10}).classes("w-full text-caption")

        def clean_now() -> None:
            """调 core.clean_runs（默认保留 50 个），回报删了几个并刷新。"""
            result = core.clean_runs(jobs.KEEP_RUNS)
            removed = len(result["removed"])
            ui.notify(f"已删除 {removed} 个运行目录，保留最近 {result['kept']} 个",
                      type="positive" if removed else "info")
            refresh_all()

        def refresh_all() -> None:
            """重建运行下拉框与运行目录表（新跑一次的目录要能立刻出现）。"""
            rows = core.list_runs()
            previous = run_select.value
            run_select.set_options({row["run_id"]: _run_label(row)
                                    for row in rows})
            if previous in run_select.options:
                run_select.value = previous
            elif rows:
                run_select.value = rows[0]["run_id"]
            else:
                run_select.value = None          # 目录被清空：别留一个失效的选择
            run_hint.set_text(f"可导出 {len(rows)} 次运行；只有带 scan.json 的才能写 GP"
                              if rows else _NO_RUN_HINT)
            runs_area.refresh()

        runs_area()
        refresh_all()
