# -*- coding: utf-8 -*-
"""奇异修复页（M4）：现场读数 → 判定 / 段定位 → 局部或整体修复。

页面只做编排，算法与校验全部在 :mod:`laser_web.core`：

1. **定位段**：同步调用 ``core.locate_segment``（50 点约 0.5 s），渲染判定徽标、
   有效阈值、窗口剖面表与一句话结论——这一步**不改任何参数、不落盘**；
2. **修复**：``jobs.submit("repair", ...)`` 起后台任务（``core.run_repair``），
   进度条 / 日志 / 取消沿用 M2 的那套轮询；
3. **结果区**：修复前后对比（模式、边界、体积损失、局部改的是哪个 GP）、候选
   日志（默认收起）、以及「修复后的计划」（统计卡 + 3D 图 + 路点表 + 段诊断）。

M2 的渲染器（路点行 / 段行 / 3D 图 / 表格列定义 / 失败行模板）直接从
:mod:`laser_web.pages.generate` 复用：``run_repair`` 的返回值与 ``run_scan``
同构就是为了这一点，复制一份只会让两个页面慢慢漂移。

表单状态全部是 :func:`render` 的局部变量（每个客户端一份，见设计文档 §7.8）；
``jobs`` 的登记表是进程级单例，因为解算本来就该串行。
"""

from __future__ import annotations

from pathlib import Path

from nicegui import ui

from excitation_trajectory import aabb_snake_scan as scan

from .. import core, jobs
from ..ui_state import default_form_values, euler_conventions
from . import adopt_running_job, download_button, generate

#: 扫描页 CLI 的默认值（采样域/守卫那部分两页同源）。
_CLI_DEFAULTS = scan.build_arg_parser().parse_args([])
#: 修复流程 CLI 的默认值（``--seg-samples`` 等与扫描 CLI 故意不同）。
_REPAIR_DEFAULTS = core.repair_cli_defaults()
#: 轮询间隔（秒）。
POLL_INTERVAL = 0.3
#: 任务状态 -> 界面文案（与生成页一致）。
_STATE_TEXT = {"pending": "排队中", "running": "进行中", "done": "完成",
               "failed": "失败", "cancelled": "已取消"}
#: 判定结论的徽标：颜色（Quasar 调色板）与中文名。
_KIND_COLOR = {"degenerate": "grey-6", "wrist": "blue-7",
               "elbow": "orange-8", "unexplained": "red-8"}
_KIND_LABEL = {"degenerate": "退化读数", "wrist": "腕奇异",
               "elbow": "肘奇异", "unexplained": "判据外位形"}
#: 每种判定的一句直白说明（``note`` 是算法给的，这里是给现场看的补充）。
_KIND_HINT = {
    "degenerate": ("退化读数：J5≈0 时 J4/J6 共线，控制器对这对自由度的分配是"
                   "随机的。这个读数不能用来定位位置，也不能区分腕/肘奇异，"
                   "所以按段内肘部裕度整体收紧，且不生成位置禁区。"),
    "wrist": "腕奇异：端点 |J5| 已低于阈值，把 |J5| 下限抬到实测裕度之上再重规划。",
    "elbow": "肘奇异：端点 |J3| 贴近 0（肘伸展），端点与 MOVL 段内都要留肘部裕度。",
    "unexplained": ("判据外位形：本仓模型的判据解释不了这次报警，"
                    "按位置禁区处理——禁区球内的路点一律判不可行。"),
}
#: 导出按钮（artifacts 键 -> 文案）。
_EXPORTS = (("scan_json", "下载 scan.json"), ("joints", "下载 joints.txt"),
            ("plot", "下载 plot.png"), ("repair", "下载 repair.json"))
#: 报告段底色 / 越界段的 Vue 模板（Quasar ``body-row`` 槽，保留排序与分页）。
_PROFILE_ROW_SLOT = """
<q-tr :props="props" :class="props.row.failed ? 'bg-red-1 text-negative'
    : (props.row.reported ? 'bg-amber-2' : '')">
    <q-td v-for="col in props.cols" :key="col.name" :props="props">
        {{ col.value }}
    </q-td>
</q-tr>
"""
#: 候选日志里失败行的模板。
_LOG_ROW_SLOT = """
<q-tr :props="props" :class="props.row.failed ? 'text-negative' : ''">
    <q-td v-for="col in props.cols" :key="col.name" :props="props">
        {{ col.value }}
    </q-td>
</q-tr>
"""
#: 边界对比里"有变化"的行标黄。
_BOUND_ROW_SLOT = """
<q-tr :props="props" :class="props.row.changed ? 'bg-amber-2' : ''">
    <q-td v-for="col in props.cols" :key="col.name" :props="props">
        {{ col.value }}
    </q-td>
</q-tr>
"""
#: 现场那次退化读数（2026-11-05）：点一次就把整份现场参数填回来。
_FIELD_SAMPLE = {
    "robot": "data/configs/hyundai_general6s.urdf",
    "point_min": [750.0, -600.0, 270.0],
    "point_max": [1300.0, 600.0, 940.0],
    "rpy": [3.0, 73.0, -167.0],
    "euler": "xyz-intrinsic",
    "q_seed": "43,-30,-36,42,-89,-1",
    "stopped_at": 41,
    "singular_q": [0.0, 0.0, 0.0, 55.721, 0.0, -55.721],
}


def _fmt(value, digits: int = 3) -> str:
    """``None`` 安全的数字显示（缺数据写 ``—``）。"""
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def _round(value, digits: int):
    """保留位数（``None`` 原样返回），供表格排序用数值而不是字符串。"""
    return None if value is None else round(float(value), digits)


def _number(label: str, value, on_change=None, **kwargs) -> ui.number:
    """建一个占满一行的数字输入框（默认值来自 ``ui_state`` / CLI）。"""
    return ui.number(label, value=value, on_change=on_change,
                     **kwargs).classes("grow")


def _offender_indexes(offenders) -> set[int]:
    """越界段下标集合（``locate_segment`` 给 dict，``run_repair`` 给 int）。"""
    indexes = set()
    for item in offenders or []:
        indexes.add(int(item["index"]) if isinstance(item, dict) else int(item))
    return indexes


def _profile_rows(repair_like: dict) -> list[dict]:
    """段定位表：GP 起→止 / |J3|min / |J5|min / 穿零 / 最差点 / worst_frac。

    Args:
        repair_like: ``locate_segment`` 的返回值，或 ``run_repair()["repair"]``
            （两者都有 ``profiles`` / ``offenders`` / ``stopped_at``）。
    """
    offenders = _offender_indexes(repair_like.get("offenders"))
    stopped = repair_like.get("stopped_at")
    reported = None if stopped is None else int(stopped) - 1
    rows = []
    for profile in repair_like.get("profiles") or []:
        position = profile.get("worst_pos")
        rows.append({
            "gp": f"GP{profile['gp_from']:02d} → GP{profile['gp_to']:02d}",
            "j3": _round(profile.get("j3_abs_min"), 3),
            "j5": _round(profile.get("j5_abs_min"), 3),
            "crosses": int(profile.get("crossings") or 0),
            "worst": ("—" if not position else
                      ", ".join(f"{float(v):.1f}" for v in position)),
            "frac": _round(profile.get("worst_frac"), 3),
            "reported": profile["index"] == reported,
            "failed": profile["index"] in offenders,
        })
    return rows


def _bound_rows(repair: dict) -> list[dict]:
    """边界 before → after 表（每轴一行，变化的高亮）。"""
    names = ("X 下界", "X 上界", "Y 下界", "Y 上界", "Z 下界", "Z 上界")
    before = repair.get("bounds_before") or [None] * 6
    after = repair.get("bounds_after") or [None] * 6
    rows = []
    for index, name in enumerate(names):
        old, new = before[index], after[index]
        delta = None if old is None or new is None else float(new) - float(old)
        rows.append({"axis": name, "before": _round(old, 1),
                     "after": _round(new, 1), "delta": _round(delta, 1),
                     "changed": delta is not None and abs(delta) > 1e-9})
    return rows


def _log_rows(repair: dict) -> list[dict]:
    """候选评估记录（label / ok / score / reasons；失败行标红）。"""
    rows = []
    # seq 是行 key：_refine 会产出多条同名 label（同一候选不同 factor），
    # 拿 label 当 key 会撞车，导致 Vue 复用/丢失行。
    for seq, entry in enumerate(repair.get("log") or []):
        rows.append({
            "seq": seq,
            "phase": entry.get("phase") or "原域",
            "label": entry.get("label") or "—",
            "ok": "✓" if entry.get("ok") else "✗",
            "score": _round(entry.get("score"), 2),
            "reasons": "；".join(entry.get("reasons") or []) or "—",
            "failed": not bool(entry.get("ok")),
        })
    return rows


def _stat_rows(repair: dict) -> list[dict]:
    """修复前 / 修复后的关键指标对照（缺数据的写 ``—``）。"""
    before = repair.get("before_stats") or {}
    after = repair.get("after_stats") or {}

    def pair(label, key, digits=3):
        return {"item": label,
                "before": _fmt(before.get(key), digits),
                "after": _fmt(after.get(key), digits)}

    return [{"item": "解出路点", "before": f"{before.get('solved', '—')}"
                            f"/{before.get('total', '—')}",
             "after": f"{after.get('solved', '—')}"
                      f"/{after.get('total', '—')}"},
            pair("|J5|min °", "j5_abs_min"),
            pair("|J3|min °", "j3_abs_min"),
            pair("段内 |J5|min °", "seg_j5_min"),
            pair("段内 |J3|min °", "seg_j3_min"),
            {"item": "穿零 J3 / J5",
             "before": f"{before.get('j3_crossings', '—')} / "
                       f"{before.get('j5_crossings', '—')}",
             "after": f"{after.get('j3_crossings', '—')} / "
                      f"{after.get('j5_crossings', '—')}"},
            pair("构型切换", "switches", 0),
            pair("设计条件数", "design_cond", 2),
            {"item": "可行", "before": ("是" if before.get("feasible") else "否"),
             "after": ("是" if after.get("feasible") else "否")}]


def render() -> None:
    """渲染奇异修复页（表单 + 定位 / 修复 + 结果区）。"""
    defaults = default_form_values()
    defaults.update({
        "j5_min": float(_CLI_DEFAULTS.j5_min),
        "j3_min": float(_CLI_DEFAULTS.j3_min),
        "seg_min": float(_CLI_DEFAULTS.seg_min),
        "seg_samples": int(_REPAIR_DEFAULTS["seg_samples"]),
        "segment_window": int(_REPAIR_DEFAULTS["segment_window"]),
        "avoid_margin": float(_REPAIR_DEFAULTS["avoid_margin"]),
        "avoid_radius": float(_REPAIR_DEFAULTS["avoid_radius"]),
        "seed": int(_REPAIR_DEFAULTS["seed"]),
        "q_seed": str(_CLI_DEFAULTS.q_seed),
    })
    robots = core.list_robots()
    options = {robot["path"]: robot["label"]
               for robot in robots if robot["usable"]}
    skipped = [robot for robot in robots if not robot["usable"]]
    default_robot = defaults["robot"]
    if default_robot not in options:      # CLI 默认模型被删掉时退回第一个
        default_robot = next(iter(options), None)

    fields: dict = {}          # 普通表单控件
    boxes: dict = {}           # 需要整块显隐的容器（两点/六数、凑数/手填）
    q_groups: list[dict] = []  # 奇异读数：每项 {"fields": [6 个 ui.number]}
    state = {"job_id": None, "log_cursor": 0, "result": None,
             "located": None, "color_by": "order"}
    registry = jobs.get_registry()

    def collect() -> dict:
        """把当前表单收成 ``core.locate_segment`` / ``core.run_repair`` 的入参。

        只提交**当前激活**的互斥项（两点/六数、凑数/手填）；互斥规则本身仍由
        ``_validate_args`` 判定，这里不做二次判断。
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
            "j5_min": fields["j5_min"].value,
            "j3_min": fields["j3_min"].value,
            "seg_min": fields["seg_min"].value,
            "seed": fields["seed"].value,
            "q_seed": fields["q_seed"].value,
            # ---- 修复流程专用 ----
            "singular_q": [[field.value for field in group["fields"]]
                           for group in q_groups],
            "stopped_at": fields["stopped_at"].value,
            "segment_window": fields["segment_window"].value,
            "avoid_margin": fields["avoid_margin"].value,
            "avoid_radius": fields["avoid_radius"].value,
            "seg_samples": fields["seg_samples"].value,
            "allow_rpy": bool(fields["allow_rpy"].value),
            "allow_shrink": bool(fields["allow_shrink"].value),
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
        """预览卡片：体积 / 网格 / 点数（复用 ``core.preview_domain``）。"""
        result = core.preview_domain(collect())
        with ui.card().classes("w-full gap-1"):
            ui.label("采样域预览（不跑 IK）").classes("text-subtitle1")
            if not result["ok"]:
                ui.label(result["error"]).classes(
                    "text-negative text-body2 whitespace-pre-wrap")
                return
            grid = result["grid"]
            with ui.grid(columns=2).classes("w-full gap-x-4"):
                ui.label("体积").classes("text-grey-7")
                ui.label(f"{result['volume_m3']:.3f} m³")
                ui.label("网格").classes("text-grey-7")
                ui.label(f"{grid['xs'] + 1}×{grid['ys'] + 1}×{grid['zs'] + 1}")
                ui.label("点数").classes("text-grey-7")
                ui.label(f"{result['point_count']} 点")
                ui.label("设计条件数").classes("text-grey-7")
                ui.label(_fmt(result["design_condition"]))
            ui.label(f"欧拉约定 {result['convention']}").classes("text-caption")

    with ui.row().classes("w-full no-wrap items-start gap-4"):
        # ---------------- 左：表单 ----------------
        with ui.column().classes("grow gap-3"):
            with ui.card().classes("w-full"):
                ui.label("机器人模型").classes("text-subtitle1")
                fields["robot"] = ui.select(
                    options, value=default_robot,
                    label="模型（data/configs 下，只列能加载的）",
                    on_change=preview_card.refresh).classes("w-full")
                if skipped:
                    with ui.expansion(
                            f"另有 {len(skipped)} 个文件不能当模型用", icon="info"
                    ).classes("text-caption"):
                        for robot in skipped:
                            ui.label(f"{robot['label']} — {robot['reason']}").classes(
                                "text-caption text-grey-7")

            with ui.card().classes("w-full"):
                ui.label("现场信息").classes("text-subtitle1")
                with ui.row().classes("w-full no-wrap gap-2 items-end"):
                    fields["stopped_at"] = _number(
                        "已走到 GP（可空：留空只判断整链）",
                        _FIELD_SAMPLE["stopped_at"], None, step=1,
                        precision=0)
                    ui.button("填入现场样例", icon="auto_fix_high",
                              on_click=lambda: load_sample()).props("flat dense")
                ui.label("奇异读数 q（deg，6 个关节角一组，可加多组）").classes(
                    "text-caption text-grey-7")
                groups_box = ui.column().classes("w-full gap-2")

                def remove_group(entry: dict) -> None:
                    """删掉一组读数（至少留一组，避免空提交）。"""
                    if len(q_groups) <= 1:
                        ui.notify("至少保留一组奇异读数", type="warning")
                        return
                    q_groups.remove(entry)
                    entry["row"].delete()

                def add_group(values=None) -> None:
                    """追加一组 6 个关节角输入框。"""
                    values = list(values or [0.0] * 6)
                    entry: dict = {"fields": []}
                    with groups_box:
                        entry["row"] = ui.row().classes(
                            "w-full no-wrap gap-1 items-end")
                        with entry["row"]:
                            for joint in range(6):
                                entry["fields"].append(ui.number(
                                    f"J{joint + 1}",
                                    value=values[joint]).classes("w-20"))
                            ui.button(icon="remove_circle_outline",
                                      on_click=lambda item=entry:
                                      remove_group(item)).props(
                                "flat dense round color=negative")
                    q_groups.append(entry)

                add_group(_FIELD_SAMPLE["singular_q"])
                ui.button("再加一组", icon="add",
                          on_click=lambda: add_group()).props("flat dense")

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
                                fields[key] = _number(label,
                                                      defaults[row][index],
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
                fields["grid_mode"] = ui.toggle(
                    {"count": "目标点数", "explicit": "手填份数"},
                    value=defaults["grid_mode"],
                    on_change=lambda event: switch_grid(event.value)).props(
                        "spread")
                boxes["count"] = ui.column().classes("w-full gap-1")
                with boxes["count"]:
                    fields["target_count"] = _number(
                        "目标点数（自动凑网格）", defaults["target_count"],
                        preview_card.refresh, min=1, step=1)
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
                        fields[key] = _number(label,
                                              defaults["perturb_deg"][index],
                                              preview_card.refresh)
                    fields["seed"] = _number("随机种子 seed", defaults["seed"],
                                             None, min=0, step=1)
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
                fields["q_seed"] = ui.input(
                    "q-seed（6 个关节角，逗号分隔，度）",
                    value=defaults["q_seed"]).classes("w-full")

            with ui.card().classes("w-full"):
                ui.label("修复策略").classes("text-subtitle1")
                with ui.row().classes("w-full no-wrap gap-4"):
                    fields["allow_rpy"] = ui.checkbox(
                        "允许改参考姿态", value=True)
                    fields["allow_shrink"] = ui.checkbox(
                        "允许收缩边界", value=True)
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields["avoid_margin"] = _number(
                        "avoid-margin（度）", defaults["avoid_margin"], None,
                        min=0.0, step=1.0)
                    fields["avoid_radius"] = _number(
                        "avoid-radius（mm）", defaults["avoid_radius"], None,
                        min=0.0, step=10.0)
                with ui.row().classes("w-full no-wrap gap-2"):
                    fields["seg_samples"] = _number(
                        "每段采样点（搜索）", defaults["seg_samples"], None,
                        min=2, step=1)
                    fields["segment_window"] = _number(
                        "前后各看几段", defaults["segment_window"], None,
                        min=0, step=1)
                    fields["seg_min"] = _number(
                        "段内 min（度）", defaults["seg_min"], None,
                        min=0.0, step=1.0)
                ui.label("段模型固定 MOVL（笛卡尔直线）——与 avoid_singularity "
                         "CLI 一致，修复流程不提供 PTP 分支。").classes(
                    "text-caption text-grey-7")

            with ui.row().classes("gap-2"):
                locate_button = ui.button(
                    "定位段", icon="my_location",
                    on_click=lambda: on_locate())
                repair_button = ui.button(
                    "修复", icon="build",
                    on_click=lambda: start_job())
                repair_button.props("color=primary")

        # ---------------- 右：预览卡片 ----------------
        with ui.column().classes("w-80 gap-3"):
            preview_card()

    def load_sample() -> None:
        """把 2026-11-05 现场那份参数填回表单（含退化读数）。"""
        if _FIELD_SAMPLE["robot"] in options:
            fields["robot"].value = _FIELD_SAMPLE["robot"]
        for key, value in (("rpy_roll", _FIELD_SAMPLE["rpy"][0]),
                           ("rpy_pitch", _FIELD_SAMPLE["rpy"][1]),
                           ("rpy_yaw", _FIELD_SAMPLE["rpy"][2])):
            fields[key].value = value
        fields["euler"].value = _FIELD_SAMPLE["euler"]
        fields["q_seed"].value = _FIELD_SAMPLE["q_seed"]
        fields["stopped_at"].value = _FIELD_SAMPLE["stopped_at"]
        for key, value in (("min_x", _FIELD_SAMPLE["point_min"][0]),
                           ("min_y", _FIELD_SAMPLE["point_min"][1]),
                           ("min_z", _FIELD_SAMPLE["point_min"][2]),
                           ("max_x", _FIELD_SAMPLE["point_max"][0]),
                           ("max_y", _FIELD_SAMPLE["point_max"][1]),
                           ("max_z", _FIELD_SAMPLE["point_max"][2])):
            fields[key].value = value
        fields["domain_mode"].value = "points"
        fields["grid_mode"].value = "count"
        fields["target_count"].value = 50
        while len(q_groups) > 1:            # 多余的组删掉，只留第一组
            entry = q_groups[-1]
            q_groups.remove(entry)
            entry["row"].delete()
        for joint, value in enumerate(_FIELD_SAMPLE["singular_q"]):
            q_groups[0]["fields"][joint].value = value
        switch_domain("points")
        switch_grid("count")
        ui.notify("已填入现场样例参数", type="info")

    # ---------------- 进度与日志 ----------------
    progress_card = ui.card().classes("w-full")
    with progress_card:
        with ui.row().classes("w-full items-center gap-3 no-wrap"):
            ui.icon("build").classes("text-primary")
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
        """结果区：判定徽标 / 段定位表 / 前后对比 / 候选日志 / 修复后的计划。"""
        located = state["located"]
        result = state["result"]
        if located is None and result is None:
            with ui.card().classes("w-full"):
                ui.label("还没有结果").classes("text-subtitle1")
                ui.label("先点「定位段」看窗口内哪一段最危险（不修改任何参数），"
                         "再点「修复」让它自己选局部换姿态还是整体挪箱。").classes(
                    "text-caption text-grey-7")
            return

        if result is not None and not result.get("ok"):
            with ui.card().classes("w-full"):
                ui.label("失败").classes("text-subtitle1 text-negative")
                ui.label(str(result.get("error") or "未知错误")).classes(
                    "text-negative text-body2 whitespace-pre-wrap")
                for line in result.get("warnings") or []:
                    ui.label(line).classes(
                        "text-warning text-caption whitespace-pre-wrap")
                repair = result.get("repair")
                if repair:
                    _log_card(repair)
            return

        # 修复结果自带判定信息；只做过定位时用定位结果。
        view = located
        if result is not None and result.get("ok"):
            view = result.get("repair") or located
        if view is not None:
            _verdict_card(view)

        if result is None or not result.get("ok"):
            return
        repair = result["repair"]
        _compare_card(repair)
        _log_card(repair)
        _plan_card(result)

    def _verdict_card(view: dict) -> None:
        """判定徽标 + 一句话结论 + 有效阈值 + 禁区数量 + 段定位表。"""
        with ui.card().classes("w-full gap-2"):
            ui.label("判定").classes("text-subtitle1")
            with ui.row().classes("w-full items-center gap-2 flex-wrap"):
                for row in view.get("classifications") or []:
                    kind = row["kind"]
                    ui.chip(_KIND_LABEL.get(kind, kind), icon="warning",
                            color=_KIND_COLOR.get(kind, "grey-6"))
                    ui.label(
                        f"q={[round(v, 3) for v in row['q_deg']]} · "
                        f"|J3|={row['j3_abs']:.2f}° · |J5|={row['j5_abs']:.2f}°"
                    ).classes("text-caption text-grey-8")
                thresholds = view.get("effective_thresholds") or {}
                ui.chip(f"有效阈值 |J5|≥{_fmt(thresholds.get('j5_min_deg'), 2)}°"
                        f" · |J3|≥{_fmt(thresholds.get('j3_min_deg'), 2)}°",
                        icon="straighten", color="blue-grey-6")
                if thresholds.get("j3_seg_min_deg"):
                    ui.chip("段内 |J3|≥"
                            f"{_fmt(thresholds['j3_seg_min_deg'], 2)}°",
                            icon="straighten", color="blue-grey-6")
                ui.chip(f"位置禁区 {len(view.get('forbidden_positions') or [])} 个",
                        icon="block", color="grey-7")
            for row in view.get("classifications") or []:
                hint = _KIND_HINT.get(row["kind"])
                if hint:
                    ui.label(hint).classes("text-caption text-grey-7")
                if row.get("note"):
                    ui.label(f"· {row['note']}").classes(
                        "text-caption text-grey-7 whitespace-pre-wrap")
            verdict = view.get("verdict")
            if verdict:
                color = ("text-positive" if "不动采样域" in verdict
                         else "text-warning")
                ui.label(verdict).classes(f"text-body1 {color}")

            profiles = view.get("profiles") or []
            if view.get("stopped_at") is not None:
                ui.label(f"段定位（已到 GP{view['stopped_at']}，故障应在 "
                         f"GP{view['stopped_at']}→GP{int(view['stopped_at']) + 1}）"
                         ).classes("text-subtitle2")
            if not profiles:
                ui.label("没有窗口剖面：留空「已走到 GP」时只判断整链。" if
                         view.get("stopped_at") is None else "窗口内没有可检查的段。"
                         ).classes("text-caption text-grey-7")
                return
            ui.label("报告段（黄底）与越界段（红底）；最差点 = 段内 "
                     "min(|J3|,|J5|) 最小的位置（MOVL 直线插补）。").classes(
                "text-caption text-grey-7")
            table = ui.table(
                columns=[{"name": "gp", "label": "GP 起→止", "field": "gp",
                          "align": "center", "sortable": True},
                         {"name": "j3", "label": "|J3|min °", "field": "j3",
                          "align": "right", "sortable": True},
                         {"name": "j5", "label": "|J5|min °", "field": "j5",
                          "align": "right", "sortable": True},
                         {"name": "crosses", "label": "穿零", "field": "crosses",
                          "align": "center", "sortable": True},
                         {"name": "worst", "label": "最差点 (mm)",
                          "field": "worst", "align": "left"},
                         {"name": "frac", "label": "worst_frac", "field": "frac",
                          "align": "right", "sortable": True}],
                rows=_profile_rows(view), row_key="gp",
                pagination={"rowsPerPage": 10}).classes("w-full text-caption")
            table.add_slot("body-row", _PROFILE_ROW_SLOT)

    def _compare_card(repair: dict) -> None:
        """修复前后对比：模式 / label / 边界 / 体积损失 / 局部改动。"""
        mode = repair.get("mode")
        mode_color = {"none": "grey-7", "local": "teal-7",
                      "domain": "indigo-7"}.get(mode, "grey-7")
        mode_text = {"none": "不动采样域", "local": "局部修复（只换端点姿态）",
                     "domain": "整体挪动采样域"}.get(mode, str(mode))
        with ui.card().classes("w-full gap-2"):
            with ui.row().classes("w-full items-center gap-2 flex-wrap"):
                ui.label("修复结果").classes("text-subtitle1")
                ui.chip(mode_text, icon="build", color=mode_color)
                ui.label(repair.get("label") or "—").classes("text-body1")
            ui.label(f"体积损失 {float(repair.get('volume_loss') or 0.0):.2%}"
                     ).classes("text-caption text-grey-7")
            ui.table(
                columns=[{"name": "axis", "label": "边界", "field": "axis",
                          "align": "left"},
                         {"name": "before", "label": "修复前 (mm)",
                          "field": "before", "align": "right"},
                         {"name": "after", "label": "修复后 (mm)",
                          "field": "after", "align": "right"},
                         {"name": "delta", "label": "变化 (mm)",
                          "field": "delta", "align": "right"}],
                rows=_bound_rows(repair), row_key="axis",
                pagination={"rowsPerPage": 6}).classes("w-full text-caption"
                                                       ).add_slot(
                "body-row", _BOUND_ROW_SLOT)
            rpy_before = repair.get("rpy_before") or []
            rpy_after = repair.get("rpy_after") or []
            if rpy_before and rpy_after:
                changed = any(abs(float(a) - float(b)) > 1e-9
                              for a, b in zip(rpy_before, rpy_after))
                ui.label(
                    "参考姿态 " + " / ".join(f"{float(v):.1f}" for v in rpy_before)
                    + ("  →  " + " / ".join(f"{float(v):.1f}" for v in rpy_after)
                       if changed else "（未改）")).classes(
                    "text-caption text-grey-8")

            local = repair.get("local")
            if local:
                ui.label(f"局部修复：重采 GP{local['gp']} 的姿态，位置与网格不变"
                         ).classes("text-subtitle2")
                with ui.grid(columns=2).classes("w-full gap-x-4 gap-y-1"):
                    ui.label("delta_rpy").classes("text-grey-7")
                    ui.label(", ".join(f"{float(v):+.2f}"
                                       for v in local["delta_rpy"]))
                    ui.label("q 修复前").classes("text-grey-7")
                    ui.label(", ".join(f"{float(v):.2f}"
                                       for v in (local.get("q_before")
                                                 or []) or ["—"]))
                    ui.label("q 修复后").classes("text-grey-7")
                    ui.label(", ".join(f"{float(v):.2f}"
                                       for v in local.get("q_after") or []))

            with ui.expansion("修复前 / 修复后的关键指标", icon="compare_arrows"
                              ).classes("w-full text-caption"):
                ui.table(
                    columns=[{"name": "item", "label": "指标", "field": "item",
                              "align": "left"},
                             {"name": "before", "label": "修复前",
                              "field": "before", "align": "right"},
                             {"name": "after", "label": "修复后",
                              "field": "after", "align": "right"}],
                    rows=_stat_rows(repair), row_key="item",
                    pagination={"rowsPerPage": 12}).classes(
                        "w-full text-caption")

    def _log_card(repair: dict) -> None:
        """候选评估记录（默认收起：几十行，展开才看）。"""
        rows = _log_rows(repair)
        total = repair.get("log_total", len(rows))
        with ui.card().classes("w-full gap-1"):
            with ui.expansion(f"候选日志（{len(rows)}/{total} 条，按改动量从小到大）",
                              icon="list_alt").classes("w-full text-caption"):
                if not rows:
                    ui.label("这次没有评估候选（原域已可行或没进入整体搜索）。"
                             ).classes("text-caption text-grey-7")
                    return
                table = ui.table(
                    columns=[{"name": "phase", "label": "阶段", "field": "phase",
                              "align": "center", "sortable": True},
                             {"name": "label", "label": "候选改动",
                              "field": "label", "align": "left",
                              "sortable": True},
                             {"name": "ok", "label": "可行", "field": "ok",
                              "align": "center", "sortable": True},
                             {"name": "score", "label": "评分",
                              "field": "score", "align": "right",
                              "sortable": True},
                             {"name": "reasons", "label": "失败原因（前两条）",
                              "field": "reasons", "align": "left"}],
                    rows=rows, row_key="seq",
                    pagination={"rowsPerPage": 15}).classes("w-full text-caption")
                table.add_slot("body-row", _LOG_ROW_SLOT)

    def _plan_card(result: dict) -> None:
        """修复后的计划：统计卡 + 3D + 路点表 + 段诊断（M2 渲染器复用）。"""
        stats = result["stats"]
        with ui.card().classes("w-full gap-2"):
            with ui.row().classes("w-full items-center gap-2 flex-wrap"):
                ui.label("修复后的计划").classes("text-subtitle1")
                ok = stats["solved"] == stats["total"]
                ui.chip(f"成功 {stats['solved']}/{stats['total']}",
                        icon="check_circle" if ok else "error",
                        color="green" if ok else "red")
                ui.chip(f"构型切换 {stats['switches']}", icon="swap_horiz")
                ui.chip(f"ΣdqL2 {_fmt(stats['dq_l2_sum'], 2)}", icon="timeline")
                ui.chip(f"|J5|min {_fmt(stats['j5_abs_min'])}", icon="speed")
                ui.chip(f"|J3|min {_fmt(stats['j3_abs_min'])}", icon="speed")
                ui.chip(f"design {_fmt(stats['design_cond'], 2)}", icon="science")
                if stats.get("post_ik_cond") is not None:
                    ui.chip(f"post_ik {_fmt(stats['post_ik_cond'], 2)}",
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

        holder = {}

        def change_color(event) -> None:
            """切换着色依据：只更新 figure，不重画整个结果区。"""
            state["color_by"] = event.value
            if holder.get("plot") is not None:
                holder["plot"].update_figure(
                    generate._figure_for(state["result"], event.value))

        with ui.card().classes("w-full gap-1"):
            with ui.row().classes("w-full items-center gap-2 no-wrap"):
                ui.label("3D 路点").classes("text-subtitle1")
                ui.space()
                ui.toggle({"order": "按路径顺序", "margin": "按关节裕度"},
                          value=state["color_by"],
                          on_change=change_color).props("dense")
            holder["plot"] = ui.plotly(
                generate._figure_for(result, state["color_by"])).classes("w-full")

        with ui.card().classes("w-full gap-1"):
            ui.label("路点表").classes("text-subtitle1")
            ui.label("点表头排序；失败行标红。").classes(
                "text-caption text-grey-7")
            waypoint_table = ui.table(
                columns=generate._columns(generate._WAYPOINT_COLUMNS),
                rows=generate._waypoint_rows(result), row_key="gp",
                pagination={"rowsPerPage": 20, "sortBy": "gp",
                            "descending": False},
            ).classes("w-full text-caption")
            waypoint_table.add_slot("body-row", generate._FAILED_ROW_SLOT)

        with ui.card().classes("w-full gap-1"):
            ui.label("段诊断").classes("text-subtitle1")
            if not result.get("segments"):
                ui.label("没有任何相邻段（路点少于 2 个）。").classes(
                    "text-caption text-grey-7")
            else:
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
                    rows=generate._segment_rows(result), row_key="gp",
                    pagination={"rowsPerPage": 10}).classes(
                        "w-full text-caption")

        artifacts = result.get("artifacts") or {}
        run_dir = Path(result["run_dir"])
        with ui.row().classes("w-full items-center gap-2 flex-wrap"):
            ui.label("导出：").classes("text-caption text-grey-7")
            for key, label in _EXPORTS:
                name = artifacts.get(key)
                if not name:
                    continue
                download_button(run_dir / name, name, label,
                                icon="download").props("flat dense")

    def set_running(running: bool) -> None:
        """修复期间禁用按钮并显示取消；结束后反过来。"""
        for button in (locate_button, repair_button):
            (button.disable if running else button.enable)()
        cancel_button.set_visibility(running)

    def on_locate() -> None:
        """「定位段」：同步跑 ``locate_segment``（约 0.5 s），结果直接渲染。"""
        if state["job_id"] is not None:
            ui.notify("修复任务在跑，等它结束再看定位", type="warning")
            return
        result = core.locate_segment(collect())
        state["located"] = result if result.get("ok") else None
        state["result"] = None
        if not result.get("ok"):
            ui.notify(result.get("error") or "定位失败", type="negative")
        else:
            ui.notify(result["verdict"], type="info")
        result_area.refresh()

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
            state["located"] = None    # 结果里自带判定信息，避免两份并存
            stage_label.set_text("完成")
            ui.notify("修复完成", type="positive")
        elif snapshot["state"] == "cancelled":
            state["result"] = {"ok": False, "error": "任务已取消",
                               "warnings": []}
            stage_label.set_text("已取消")
            ui.notify("任务已取消", type="warning")
        else:
            state["result"] = {"ok": False,
                               "error": snapshot["error"] or "任务失败",
                               "warnings": []}
            stage_label.set_text("失败")
            ui.notify("任务失败，见日志", type="negative")
        result_area.refresh()

    def start_job() -> None:
        """提交一次修复（``jobs`` 后台线程跑 ``core.run_repair``）。"""
        if state["job_id"] is not None:
            ui.notify("已有任务在跑，先等它结束或点取消", type="warning")
            return
        params = collect()
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
            return core.run_repair(params, job=job,
                                   on_progress=job.set_progress)

        try:
            job_id = registry.submit("repair", task)
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

    # 修复要十几秒：页面重建后接管在跑的任务，否则既看不到也取消不了
    running_job = adopt_running_job(registry, "repair")
    if running_job is not None:
        state["job_id"] = running_job.id
        state["log_cursor"] = 0
        stage_label.set_text("接管在跑的任务（上一次页面已关闭）")
        set_running(True)
        poll_timer.activate()

    result_area()
    switch_domain(defaults["domain_mode"])
    switch_grid(defaults["grid_mode"])
