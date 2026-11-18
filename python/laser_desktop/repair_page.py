# -*- coding: utf-8 -*-
"""奇异修复页（S4）：现场读数 → 判定 / 段定位 → 局部或整体修复。

行为基准是网页版 ``laser_web/pages/repair.py``（1024 行）：字段、默认值、
徽标文案、结果区结构、候选日志列、导出项都**照它搬**。本模块只做编排与展示：

1. **定位段**：同步调 ``core.locate_segment``（实测 0.3–0.5 s，可接受），
   渲染判定徽标、有效阈值、位置禁区数、窗口剖面表与一句话结论——这一步不改
   任何参数、不落盘；跑的时候换等待光标 + 状态栏写「定位中…」；
2. **修复**：``bridge.submit("repair", lambda job: core.run_repair(params,
   job=job, on_progress=job.set_progress))``，进度 / 日志 / 取消 / 终态全部走
   :class:`laser_desktop.job_bridge.JobBridge` 的三条信号（与 S3 同一套线程模型）；
3. **结果区**：``QTabWidget`` 四页签——定位（判定徽标 + 段定位表）/ 修复
   （模式、边界 before→after、体积损失、局部改动、前后指标、候选日志）/ 计划
   （统计行 + 路点表 + 段诊断表，复用生成页的渲染器）/ 日志（任务 stdout）；
4. **导出**：``scan.json`` / ``joints.txt`` / ``plot.png`` / ``repair.json`` 从
   ``run_dir`` **复制**（文件名带 ``<run_id>-`` 前缀），与生成页同一个实现
   （:func:`laser_desktop.generate_page.export_artifact`）。

校验与语义**全部**走 ``core``（``preview_domain`` / ``locate_segment`` /
``run_repair``）：界面不另写规则，``{"ok": False, "error": ...}`` 时把报错原文
用红字写在结果区状态行里，**不弹窗**（弹窗会丢掉原文，现场没法回看改哪一项）。

与网页版的一处**有意差异**（见报告）：网页版的 ``collect()`` 不提交
``repair_attempts`` / ``max_candidates``（用 CLI 默认值），而 S4 规格要求表单
有这两个输入框；``core._repair_namespace`` 认这两个键，所以这里提交它们。
``seg_mode`` 则相反——网页版不提交、``core`` 的修复路径也不用它（段剖面固定
MOVL），所以本页只保留一句说明，不做成会误导人的输入框。
"""

from __future__ import annotations

import logging
from pathlib import Path

from laser_web import core, ui_state

from . import form_widgets, generate_page, paths
from .generate_page import (
    LOG_WIDGET_LIMIT,
    POINT_COUNT_WARN,
    PROGRESS_STEPS,
    RESULT_MIN_HEIGHT,
    STAT_KEYS,
    STATE_TEXT,
    UNIT_DEG,
    SortItem,
    _caption,
    _double_spin,
    _inline,
    _int_spin,
    _row,
    export_artifact,
    fill_segment_table,
    fill_stat_labels,
    fill_waypoint_table,
    make_table,
    sort_key,
)
from .job_bridge import LOG_LIMIT as JOB_LOG_LIMIT
from .job_bridge import JobBridge, adopt_running_job
from .qt_compat import Qt, QtCore, QtGui, QtWidgets

#: 日志器（``core`` 万一抛异常、出图失败都记在这里）。
_LOGGER = logging.getLogger(__name__)

#: 判定结论的徽标：颜色与中文名（**照抄**网页版 ``_KIND_COLOR`` / ``_KIND_LABEL``；
#: Quasar 的 grey-6 / blue-7 / orange-8 / red-8 换成本页用的十六进制色值）。
KIND_COLOR = {"degenerate": "#9e9e9e", "wrist": "#1976d2",
              "elbow": "#ef6c00", "unexplained": "#c62828"}
KIND_LABEL = {"degenerate": "退化读数", "wrist": "腕奇异",
              "elbow": "肘奇异", "unexplained": "判据外位形"}
#: 判定区的一句引导语。**判定本身**的解释由算法给的 ``classifications[i]["note"]``
#: 提供（那一份带数据来源），页面不再复述一遁——S4 评审截图里两句话叠着说同一
#: 件事（“退化读数：J5≈0 时 J4/J6 共线…” + “• J5≈0 时 J4/J6 共线、只有
#: J4+J6 有意义…”）。这里只留一句短的、告诉现场接下来会怎么修的引导语。
VERDICT_GUIDE = ("判定只决定修复怎么走：优先局部换端点姿态，不行再整体挪采样域。"
                 "下面每条的说明文字来自算法（note），页面不另写一份。")

#: ``repair.mode`` 的徽标文案与颜色（**照抄**网页版 ``_compare_card`` 的
#: ``mode_text`` / ``mode_color``：none=grey-7、local=teal-7、domain=indigo-7）。
MODE_TEXT = {"none": "不动采样域", "local": "局部修复（只换端点姿态）",
             "domain": "整体挪动采样域"}
MODE_COLOR = {"none": "#616161", "local": "#00796b", "domain": "#303f9f"}

#: 报告段（黄底）/ 越界段（红底，复用生成页的失败行配色）/ 变化轴（黄底）。
_REPORTED_BG = "#ffecb3"          # Quasar bg-amber-2
_OFFENDER_BG = generate_page._FAILED_BG
_OFFENDER_FG = generate_page._FAILED_FG
_CHANGED_BG = "#ffecb3"
_CHIP_BG = "#546e7a"              # Quasar blue-grey-6（阈值 / 禁区徽标）
_THRESHOLD_OK = "color:#1e8449"
_THRESHOLD_WARN = "color:#8a6d3b"
_STATUS_STYLES = generate_page._STATUS_STYLES

#: 段定位表列头（网页版 ``_profile_rows`` 的列：GP 起→止 / |J3| / |J5| / 穿零 /
#: 最差点 / worst_frac）。
PROFILE_HEADERS = ("GP 起→止", "|J3|min °", "|J5|min °", "穿零",
                   "最差点 (mm)", "t (worst_frac)")

#: 边界对比表列头（网页版 ``_bound_rows``）。
BOUND_HEADERS = ("边界", "修复前 (mm)", "修复后 (mm)", "变化 (mm)")

#: 修复前/后指标对照表列头（网页版 ``_stat_rows``）。
COMPARE_HEADERS = ("指标", "修复前", "修复后")

#: 候选日志表列头（网页版 ``_log_rows``：阶段 / 候选改动 / 可行 / 评分 /
#: 失败原因）。行 key 用序号，``label`` 会重复（同一候选不同 factor）。
LOG_HEADERS = ("阶段", "候选改动", "可行", "评分", "失败原因（前两条）")

#: 四个产物在 ``run_dir`` 里的文件名（``_update_exports`` 按它们查文件到底在不在）。
#: 键与 :data:`EXPORTS` 一致；名字全部问 ``core`` 要，不手抄文件名。
ARTIFACT_NAMES = {"scan_json": core.ARTIFACT_SCAN_JSON,
                  "joints": core.ARTIFACT_JOINTS,
                  "plot": core.ARTIFACT_PLOT,
                  "repair": core.ARTIFACT_REPAIR}

#: ``mode=none`` 时边界表的替身文案（前后必然相同、6 行占位数字没有任何信息）。
BOUND_UNCHANGED_TEXT = "边界未改动（需改参考姿态或收缩时才会调整）"

#: 导出按钮（**照抄**网页版 ``_EXPORTS`` 的四个键，多一个 ``repair.json`` 的
#: 过滤串是为了文件对话框）。
EXPORTS = (("scan_json", "导出 scan.json", "JSON (*.json);;所有文件 (*)"),
           ("joints", "导出 joints.txt", "文本 (*.txt);;所有文件 (*)"),
           ("plot", "导出 plot.png", "PNG 图片 (*.png);;所有文件 (*)"),
           ("repair", "导出 repair.json", "JSON (*.json);;所有文件 (*)"))

#: 现场那次退化读数（2026-11-05）：点一次就把整份现场参数填回来。
#: **照抄**网页版 ``_FIELD_SAMPLE``（它是现场复盘的固定参数，不是界面默认值）。
FIELD_SAMPLE = {
    "robot": "data/configs/hyundai_general6s.urdf",
    "point_min": [750.0, -600.0, 270.0],
    "point_max": [1300.0, 600.0, 940.0],
    "rpy": [3.0, 73.0, -167.0],
    "euler": "xyz-intrinsic",
    "q_seed": "43,-30,-36,42,-89,-1",
    "stopped_at": 41,
    "singular_q": [0.0, 0.0, 0.0, 55.721, 0.0, -55.721],
}

#: 奇异读数的关节角范围（规格：−360..360，2 位小数）。
Q_LIMIT_DEG = 360.0
Q_DECIMALS = 2


# ---------------------------------------------------------------------------
# 纯逻辑（不碰 Qt，单测直接调）
# ---------------------------------------------------------------------------
def repair_defaults() -> dict:
    """修复页的表单默认值：生成页那套 + 修复流程专用项。

    采样域 / 网格 / 姿态 / 扰动 / 守卫的默认值与生成页**同源**
    （:func:`laser_desktop.generate_page.form_defaults`），这里只补修复流程自己的
    几项，且**全部问 CLI 要**（:func:`laser_web.core.repair_cli_defaults`）：

    * ``seg_samples`` 故意与扫描 CLI 不同（修复 11 / 扫描 21，修复要跑上百次候选）；
    * ``segment_window`` / ``avoid_margin`` / ``avoid_radius`` / ``repair_attempts``
      / ``max_candidates`` 只有修复 CLI 才有；
    * ``stopped_at`` 与 ``singular_q`` 的初值取现场样例（与网页版一致：页面一打开
      就有一组可跑的读数，而不是空表）。

    Returns:
        dict: 全新的表单值（含 list，调用方可以随意改）。
    """
    values = generate_page.form_defaults()
    defaults = core.repair_cli_defaults()
    values.update({
        "singular_q": [[float(value) for value in FIELD_SAMPLE["singular_q"]]],
        "stopped_at": int(FIELD_SAMPLE["stopped_at"]),
        "segment_window": int(defaults["segment_window"]),
        "avoid_margin": float(defaults["avoid_margin"]),
        "avoid_radius": float(defaults["avoid_radius"]),
        "repair_attempts": int(defaults["repair_attempts"]),
        "max_candidates": int(defaults["max_candidates"]),
        "seg_samples": int(defaults["seg_samples"]),
        "allow_rpy": True,
        "allow_shrink": True,
    })
    return values


def stopped_at_value(raw):
    """「已走到第几点」的界面值 -> ``core`` 要的 ``stopped_at``。

    界面用 ``QSpinBox``，``0`` 表示**没给**（显示成「—」），所以这里把 ``0`` /
    ``None`` / 空串都归一成 ``None``：``core._stopped_index`` 拿到 ``None`` 就只
    判整链，拿到 ``0`` 会按 CLI 原文报「需在 1..N 之间」。

    Args:
        raw: 表单值。

    Returns:
        int | None：``None`` 或 ``1..9999`` 的整数。
    """
    if raw is None or raw == "":
        return None
    value = int(raw)
    return None if value == 0 else value


def form_to_params(values: dict) -> dict:
    """表单值 -> ``core.locate_segment`` / ``core.run_repair`` 的入参。

    键集合照抄网页版 ``pages/repair.py::collect()``，两端互斥项只提交**当前激活**
    的那一组（两点 XOR 六数、目标点数 XOR 手填份数），互斥规则本身仍由
    ``core`` 里的 ``_validate_args`` 判定——这里不做二次判断。

    Args:
        values: 表单值，缺省项按 :func:`repair_defaults` 补齐。

    Returns:
        dict: 可直接喂 ``core.locate_segment`` / ``core.run_repair``。
    """
    values = {**repair_defaults(), **(values or {})}
    params = {
        "robot": values["robot"],
        "rpy": [float(value) for value in values["rpy"]],
        "euler": str(values["euler"]),
        "perturb_deg": [float(value) for value in values["perturb_deg"]],
        "cond_max": float(values["cond_max"]),
        "j5_min": float(values["j5_min"]),
        "j3_min": float(values["j3_min"]),
        "seg_min": float(values["seg_min"]),
        "seed": int(values["seed"]),
        "q_seed": str(values["q_seed"]),
        # ---- 修复流程专用 ----
        "singular_q": [[float(value) for value in group]
                       for group in values["singular_q"]],
        "stopped_at": stopped_at_value(values["stopped_at"]),
        "segment_window": int(values["segment_window"]),
        "avoid_margin": float(values["avoid_margin"]),
        "avoid_radius": float(values["avoid_radius"]),
        "seg_samples": int(values["seg_samples"]),
        "allow_rpy": bool(values["allow_rpy"]),
        "allow_shrink": bool(values["allow_shrink"]),
        # 规格要求这两个输入框（网页版没有，用 CLI 默认值）；core 认这两个键，
        # 分别进 ``repair_segment(attempts=...)`` 与 ``_search_domains(max_candidates)``。
        "repair_attempts": int(values["repair_attempts"]),
        "max_candidates": int(values["max_candidates"]),
    }
    if values["domain_mode"] == generate_page.DOMAIN_MODES[0]:      # 两点输入
        params["point_min"] = [float(value) for value in values["point_min"]]
        params["point_max"] = [float(value) for value in values["point_max"]]
    else:                                                          # 六个轴边界
        params["aabb6"] = [float(value) for value in values["aabb6"]]
    if values["grid_mode"] == generate_page.GRID_MODES[0]:          # 目标点数
        params["target_count"] = int(values["target_count"])
    else:                                                          # 手填格点数
        for name in ("xs", "ys", "zs"):
            params[name] = ui_state.vertices_to_segments(values[name])
        params["z_insert"] = int(values["z_insert"])
    return params


def _fmt(value, digits: int = 3) -> str:
    """``None`` 安全的数字显示（缺数据写 ``—``，与网页版 ``_fmt`` 一致）。"""
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def _round(value, digits: int):
    """保留位数（``None`` 原样返回）；表格里的排序键用数值，不用字符串。"""
    return None if value is None else round(float(value), digits)


def offender_indexes(offenders) -> set[int]:
    """越界段下标集合（``locate_segment`` 给 dict，``run_repair`` 给 int）。"""
    indexes = set()
    for item in offenders or []:
        indexes.add(int(item["index"]) if isinstance(item, dict) else int(item))
    return indexes


def profile_rows(repair_like: dict) -> list[dict]:
    """段定位表的行：GP 起→止 / |J3|min / |J5|min / 穿零 / 最差点 / worst_frac。

    照抄网页版 ``_profile_rows``：``reported`` 标报告段（黄底），``failed`` 标越界
    段（红底）；两者都为真时红底优先（与网页版的行模板优先级一致）。

    Args:
        repair_like: ``locate_segment`` 的返回值，或 ``run_repair()["repair"]``
            （两者都有 ``profiles`` / ``offenders`` / ``stopped_at``）。

    Returns:
        list[dict]：每行的显示文本与标记。
    """
    offenders = offender_indexes((repair_like or {}).get("offenders"))
    stopped = (repair_like or {}).get("stopped_at")
    reported = None if stopped is None else int(stopped) - 1
    rows = []
    for profile in (repair_like or {}).get("profiles") or []:
        position = profile.get("worst_pos")
        rows.append({
            "gp": f"GP{profile['gp_from']:02d} → GP{profile['gp_to']:02d}",
            "j3": _round(profile.get("j3_abs_min"), 3),
            "j5": _round(profile.get("j5_abs_min"), 3),
            "crosses": int(profile.get("crossings") or 0),
            "worst": ("—" if not position else
                      ", ".join(f"{float(value):.1f}" for value in position)),
            "frac": _round(profile.get("worst_frac"), 3),
            "reported": profile["index"] == reported,
            "failed": profile["index"] in offenders,
        })
    return rows


def bound_rows(repair: dict) -> list[dict]:
    """边界 before → after 表（每轴一行：X/Y/Z 的上下界，变化的高亮）。"""
    names = ("X 下界", "X 上界", "Y 下界", "Y 上界", "Z 下界", "Z 上界")
    before = (repair or {}).get("bounds_before") or [None] * 6
    after = (repair or {}).get("bounds_after") or [None] * 6
    rows = []
    for index, name in enumerate(names):
        old, new = before[index], after[index]
        delta = None if old is None or new is None else float(new) - float(old)
        rows.append({"axis": name, "before": _round(old, 1),
                     "after": _round(new, 1), "delta": _round(delta, 1),
                     "changed": delta is not None and abs(delta) > 1e-9})
    return rows


def log_rows(repair: dict) -> list[dict]:
    """候选评估记录（阶段 / label / ok / score / reasons；失败行标红）。"""
    rows = []
    # seq 是行 key：整体搜索会产出多条同名 label（同一候选不同 factor），
    # 拿 label 当 key 会撞车。
    for seq, entry in enumerate((repair or {}).get("log") or []):
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


def stat_rows(repair: dict) -> list[dict]:
    """修复前 / 修复后的关键指标对照（照抄网页版 ``_stat_rows``，缺数据写 ``—``）。"""
    before = (repair or {}).get("before_stats") or {}
    after = (repair or {}).get("after_stats") or {}

    def pair(label, key, digits=3):
        return {"item": label,
                "before": _fmt(before.get(key), digits),
                "after": _fmt(after.get(key), digits)}

    def solved(stats):
        return f"{stats.get('solved', '—')}/{stats.get('total', '—')}"

    def crossings(stats):
        return f"{stats.get('j3_crossings', '—')} / {stats.get('j5_crossings', '—')}"

    return [{"item": "解出路点",
             "before": solved(before), "after": solved(after)},
            pair("|J5|min °", "j5_abs_min"),
            pair("|J3|min °", "j3_abs_min"),
            pair("段内 |J5|min °", "seg_j5_min"),
            pair("段内 |J3|min °", "seg_j3_min"),
            {"item": "穿零 J3 / J5",
             "before": crossings(before), "after": crossings(after)},
            pair("构型切换", "switches", 0),
            pair("设计条件数", "design_cond", 2),
            {"item": "可行",
             "before": ("是" if before.get("feasible") else "否"),
             "after": ("是" if after.get("feasible") else "否")}]


def kind_text(kind) -> str:
    """判定 -> 徽标文字（未知判定原样显示，不吞信息）。"""
    return KIND_LABEL.get(kind, str(kind))


def mode_text(mode) -> str:
    """``repair.mode`` -> 徽标文字（未知值原样显示）。"""
    return MODE_TEXT.get(mode, str(mode))


def q_text(values) -> str:
    """6 个关节角 -> 一行显示文本（``None`` 系列写 ``—``）。"""
    if not values:
        return "—"
    return ", ".join(f"{float(value):.2f}" for value in values)


def _badge(text: str, background: str, color: str = "#ffffff"):
    """圆角色块（网页版 ``ui.chip`` 的等价物）。"""
    label = QtWidgets.QLabel(str(text))
    label.setStyleSheet(
        f"background:{background}; color:{color}; border-radius:8px;"
        "padding:2px 10px; font-weight:bold;")
    label.setAlignment(Qt.AlignCenter)
    label.setSizePolicy(QtWidgets.QSizePolicy.Maximum,
                        QtWidgets.QSizePolicy.Preferred)
    return label


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class RepairPage(QtWidgets.QWidget):
    """奇异修复页：左侧可滚动表单 + 右上预览卡片 + 右侧（图 + 四页签结果）。

    Attributes:
        fields: ``{键: 控件}``，取值与 :func:`form_to_params` 的入参同名。
        q_rows: 奇异读数的动态行（每项 ``{"fields": [6 个 QDoubleSpinBox], ...}``）。
        kind_badges: 当前判定徽标（``QLabel`` 列表，顺序同 ``classifications``）。
        threshold_labels: 有效阈值 / 禁区数量徽标。
        hint_labels: 判定区的说明行（一句引导语 + 算法给的 ``note``）。
        profile_table: 段定位表（报告段黄底、越界段红底）。
        mode_badge / label_value / volume_label / bound_table / rpy_label:
            修复前后对比块（``mode=none`` 时 ``bound_table`` 换成 ``bound_note``
            那句说明）。
        local_labels: 局部修复块（``gp`` / ``delta_rpy`` / ``q_before`` / ``q_after``）。
        stat_table: 修复前/后关键指标对照表。
        log_table: 候选日志表。
        stat_labels: 「计划」页签的统计标签（键见 ``STAT_KEYS``）。
        waypoint_table / segment_table: 修复后的路点表与段诊断表。
        export_buttons: ``{artifacts 键: 按钮}``，有产物才可用。
        total: 当前表单的总点数（``TotalPoints``；``over`` 时「修复」禁用）。
        last_located: 最近一次 ``core.locate_segment`` 的成功返回值。
        last_result: 最近一次 ``core.run_repair`` 的返回值（**含 ``ok=False``**：
            不可行或取消时也接管，因为那时 ``repair.json`` 往往已经在盘上）。
        run_fingerprint: 提交那一刻的关键参数指纹（修复期间表单不锁，终态靠它
            把结果对回到具体一版参数）。
        last_status: 最近一次写到状态栏的文字（无主窗口时也能断言）。
        run_state: ``idle`` / ``running`` / ``done`` / ``failed`` / ``cancelled``。
    """

    def __init__(self, plot=None, robots=None, parent=None, bridge=None) -> None:
        """建页面。

        Args:
            plot: ``PlotView``（或任何有 ``set_figure`` 的对象）；``None`` 表示不挂
                WebEngine（纯逻辑测试用，图仍然算好放在 ``last_figure``）。
            robots: :func:`laser_web.core.list_robots` 的返回值；``None`` 时现查。
            parent: 父控件。
            bridge: 现成的 :class:`JobBridge`（测试注入假登记表用）；``None`` 时
                按 :func:`laser_desktop.paths.runs_root` 新建一个。
        """
        super().__init__(parent)
        self._plot = plot
        self._robots = (core.list_robots(paths.asset_root())
                        if robots is None else list(robots))
        self.fields = {}
        self.q_rows = []
        self.kind_badges = []
        self.threshold_labels = []
        self.hint_labels = []
        self.detail_labels = []
        self.local_labels = {}
        self.stat_labels = {}
        self.export_buttons = {}
        self.preview_labels = {}
        self.metric_labels = self.preview_labels        # 与生成页同名，便于对照
        self.last_located = None
        self.locate_error = None
        self.last_result = None
        self.last_figure = None
        self.last_status = ""
        self.run_state = "idle"
        self.total = None                # ``refresh_preview`` 里填
        self.run_fingerprint = ""        # ``start_repair`` 提交时抓一版
        self.last_job_id = None
        self._preview = None
        self._busy = False
        self._locating = False
        self.bridge = bridge if bridge is not None else JobBridge(parent=self)

        page_layout = QtWidgets.QVBoxLayout(self)
        page_layout.setContentsMargins(8, 8, 8, 8)
        page_layout.setSpacing(6)
        self.banner = QtWidgets.QLabel("")
        self.banner.setWordWrap(True)
        self.banner.hide()
        page_layout.addWidget(self.banner)

        body = QtWidgets.QWidget()
        outer = QtWidgets.QHBoxLayout(body)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)
        outer.addWidget(self._build_form(), 0)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(8)
        right.addWidget(self._build_preview_card(), 0)
        right.addWidget(self._build_viewer(), 1)
        outer.addLayout(right, 1)
        page_layout.addWidget(body, 1)

        # 信号连在控件建完之后（槽里直接碰控件，都在 GUI 线程，见 job_bridge）
        self.bridge.progress.connect(self._on_progress)
        self.bridge.log_line.connect(self._on_log_line)
        self.bridge.finished.connect(self._on_finished)
        self.refresh_preview()
        self._resume_running_job()

    # ------------------------------------------------------------------ 表单
    def _build_form(self) -> QtWidgets.QScrollArea:
        """左侧表单：模型 / 现场信息 / 采样域 / 网格 / 姿态 / 扰动 / 守卫 / 修复策略。"""
        defaults = repair_defaults()
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setSpacing(8)
        layout.addWidget(form_widgets.build_robot_group(
            self.fields, self._robots, self._on_change,
            defaults["robot"])["group"])
        layout.addWidget(self._build_field_group(defaults))

        domain = form_widgets.build_domain_group(self.fields, defaults,
                                                 self._on_change)
        self.domain_group = domain
        domain["points"].toggled.connect(self._on_domain_switch)
        domain["axes"].toggled.connect(self._on_domain_switch)
        layout.addWidget(domain["group"])

        grid = form_widgets.build_grid_group(self.fields, defaults,
                                             self._on_change)
        self.grid_group = grid
        grid["count"].toggled.connect(self._on_grid_switch)
        grid["explicit"].toggled.connect(self._on_grid_switch)
        layout.addWidget(grid["group"])

        layout.addWidget(form_widgets.build_pose_group(self.fields, defaults,
                                                       self._on_change))
        layout.addWidget(form_widgets.build_perturb_group(self.fields, defaults,
                                                          self._on_change))
        layout.addWidget(form_widgets.build_guard_group(
            self.fields, defaults, self._on_change, with_attempts=False,
            with_seg_mode=False))
        layout.addWidget(self._build_strategy_group(defaults))
        layout.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        # 比生成页宽一点：6 个关节角要排在一行里（带单位后缀会撑不下）
        scroll.setMinimumWidth(520)
        scroll.setMaximumWidth(640)
        return scroll

    def _build_field_group(self, defaults) -> QtWidgets.QGroupBox:
        """现场信息：已走到第几点 + 奇异关节角（可加/删组，至少留一组）。"""
        group = QtWidgets.QGroupBox("现场信息")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)

        stopped = QtWidgets.QSpinBox()
        stopped.setRange(0, 9999)
        stopped.setValue(int(defaults["stopped_at"]))
        # 0 = 没给：显示成「—（未给）」，而不是让现场以为要填 0
        stopped.setSpecialValueText("—（未给）")
        stopped.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        stopped.setToolTip("已走到的 GP 点号；留 0 表示没给，只判断整链")
        stopped.valueChanged.connect(self._on_change)
        self.fields["stopped_at"] = stopped
        sample_button = QtWidgets.QPushButton("填入现场样例")
        sample_button.setToolTip("把 2026-11-05 那份退化读数与采样域填回表单")
        sample_button.clicked.connect(self.load_sample)
        layout.addWidget(_row(QtWidgets.QLabel("已走到第几点"), stopped,
                              sample_button))

        layout.addWidget(_caption(
            "奇异读数 q（deg，6 个关节角一组，可加多组；至少留一组）"))
        self.q_rows_box = QtWidgets.QWidget()
        self.q_rows_layout = QtWidgets.QVBoxLayout(self.q_rows_box)
        self.q_rows_layout.setContentsMargins(0, 0, 0, 0)
        self.q_rows_layout.setSpacing(4)
        layout.addWidget(self.q_rows_box)
        for values in defaults["singular_q"] or [[0.0] * 6]:
            self.add_q_row(values)

        add_button = QtWidgets.QPushButton("＋ 再加一组")
        add_button.clicked.connect(lambda: self.add_q_row())
        layout.addWidget(add_button)
        return group

    def _build_strategy_group(self, defaults) -> QtWidgets.QGroupBox:
        """修复策略：两个开关 + 裕度/禁区半径/尝试次数/候选上限。"""
        group = QtWidgets.QGroupBox("修复策略")
        form = QtWidgets.QFormLayout(group)

        self.fields["allow_rpy"] = QtWidgets.QCheckBox("允许改参考姿态")
        self.fields["allow_rpy"].setChecked(bool(defaults["allow_rpy"]))
        self.fields["allow_rpy"].toggled.connect(self._on_change)
        self.fields["allow_shrink"] = QtWidgets.QCheckBox("允许收缩边界")
        self.fields["allow_shrink"].setChecked(bool(defaults["allow_shrink"]))
        self.fields["allow_shrink"].toggled.connect(self._on_change)
        form.addRow(_row(self.fields["allow_rpy"], self.fields["allow_shrink"]))

        margin = _double_spin(0.0, 90.0, defaults["avoid_margin"], UNIT_DEG,
                              decimals=2, step=1.0)
        radius = _double_spin(0.0, 5000.0, defaults["avoid_radius"], " mm",
                              decimals=1, step=10.0)
        for box in (margin, radius):
            box.valueChanged.connect(self._on_change)
        self.fields["avoid_margin"] = margin
        self.fields["avoid_radius"] = radius
        form.addRow(_inline([("经验裕度 avoid_margin", margin),
                             ("位置禁区半径 avoid_radius", radius)]))

        attempts = _int_spin(1, 1000, int(defaults["repair_attempts"]))
        candidates = _int_spin(1, 100000, int(defaults["max_candidates"]))
        for box in (attempts, candidates):
            box.valueChanged.connect(self._on_change)
        self.fields["repair_attempts"] = attempts
        self.fields["max_candidates"] = candidates
        window = _int_spin(0, 50, int(defaults["segment_window"]))
        window.valueChanged.connect(self._on_change)
        self.fields["segment_window"] = window
        form.addRow(_inline([("局部修复尝试次数", attempts),
                            ("候选上限", candidates),
                            ("前后各看几段", window)]))
        form.addRow(_caption(
            "候选上限 = 整体搜索最多评估多少个候选（超出即停，结论里会说明）。"))
        form.addRow(_caption(
            "段模型固定 MOVL（笛卡尔直线）——与 avoid_singularity CLI 一致，"
            "修复流程不提供 PTP 分支（core 的修复路径不接受 seg_mode）。"))
        return group

    def add_q_row(self, values=None) -> dict:
        """追加一组 6 个关节角输入框。

        Args:
            values: 6 个角度（deg）；``None`` 表示全 0。

        Returns:
            dict: ``{"fields": [6 个 QDoubleSpinBox], "widget", "remove_button"}``。
        """
        values = list(values or [0.0] * 6)
        entry = {"fields": []}
        holder = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(holder)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        for joint in range(6):
            # 单位（deg）已写在分组说明里：6 个框要排进一行，后缀会把它撑爆
            box = _double_spin(-Q_LIMIT_DEG, Q_LIMIT_DEG, values[joint], "",
                               decimals=Q_DECIMALS, step=1.0)
            box.setMinimumWidth(70)
            box.valueChanged.connect(self._on_change)
            entry["fields"].append(box)
            column = QtWidgets.QVBoxLayout()
            column.setContentsMargins(0, 0, 0, 0)
            column.setSpacing(1)
            title = QtWidgets.QLabel(f"J{joint + 1}")
            title.setStyleSheet("color:#6b7280")
            column.addWidget(title)
            column.addWidget(box)
            row.addLayout(column, 1)
        remove = QtWidgets.QPushButton("删除本组")
        remove.setToolTip("至少保留一组奇异读数")
        remove.clicked.connect(lambda _checked=False, item=entry:
                               self.remove_q_row(item))
        entry["remove_button"] = remove
        row.addWidget(remove)
        entry["widget"] = holder
        self.q_rows_layout.addWidget(holder)
        self.q_rows.append(entry)
        self._sync_q_remove_buttons()
        return entry

    def remove_q_row(self, entry) -> bool:
        """删掉一组读数（至少留一组，避免空提交）。

        Args:
            entry: :meth:`add_q_row` 的返回值。

        Returns:
            bool: 真的删掉了才 ``True``。
        """
        if len(self.q_rows) <= 1:
            self._set_status("至少保留一组奇异读数", level="muted")
            return False
        if entry in self.q_rows:
            self.q_rows.remove(entry)
        entry["widget"].setParent(None)
        entry["widget"].deleteLater()
        self._sync_q_remove_buttons()
        self._on_change()
        return True

    def _sync_q_remove_buttons(self) -> None:
        """只剩一组时把「删除本组」按灰（语义在 :meth:`remove_q_row` 里再兜一次）。"""
        only = len(self.q_rows) <= 1
        for entry in self.q_rows:
            entry["remove_button"].setEnabled(not only)

    def load_sample(self) -> None:
        """把 2026-11-05 现场那份参数填回表单（含退化读数）。"""
        fields = self.fields
        index = fields["robot"].findData(FIELD_SAMPLE["robot"])
        if index >= 0:
            fields["robot"].setCurrentIndex(index)
        for key, value in (("rpy_a", FIELD_SAMPLE["rpy"][0]),
                           ("rpy_b", FIELD_SAMPLE["rpy"][1]),
                           ("rpy_c", FIELD_SAMPLE["rpy"][2])):
            fields[key].setValue(float(value))
        euler = fields["euler"].findData(FIELD_SAMPLE["euler"])
        if euler >= 0:
            fields["euler"].setCurrentIndex(euler)
        fields["q_seed"].setText(FIELD_SAMPLE["q_seed"])
        fields["stopped_at"].setValue(int(FIELD_SAMPLE["stopped_at"]))
        for key, value in (("min_x", FIELD_SAMPLE["point_min"][0]),
                           ("min_y", FIELD_SAMPLE["point_min"][1]),
                           ("min_z", FIELD_SAMPLE["point_min"][2]),
                           ("max_x", FIELD_SAMPLE["point_max"][0]),
                           ("max_y", FIELD_SAMPLE["point_max"][1]),
                           ("max_z", FIELD_SAMPLE["point_max"][2])):
            fields[key].setValue(float(value))
        fields["domain_points"].setChecked(True)
        fields["grid_count"].setChecked(True)
        fields["target_count"].setValue(50)
        while len(self.q_rows) > 1:           # 多余的组删掉，只留第一组
            entry = self.q_rows[-1]
            self.q_rows.remove(entry)
            entry["widget"].setParent(None)
            entry["widget"].deleteLater()
        for joint, value in enumerate(FIELD_SAMPLE["singular_q"]):
            self.q_rows[0]["fields"][joint].setValue(float(value))
        self._sync_q_remove_buttons()
        self._on_domain_switch(True)
        self._on_grid_switch(True)
        self.refresh_preview()
        self._report_status("已填入现场样例参数")

    # -------------------------------------------------------- 右侧：预览 + 结果
    def _build_preview_card(self) -> QtWidgets.QGroupBox:
        """预览卡片：体积 / 格点 / 总点数 / 设计条件数 / 欧拉约定 + 报错原文。"""
        group = QtWidgets.QGroupBox("预览（不跑 IK）")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(2)
        grid = QtWidgets.QFormLayout()
        for key, title in (("volume", "体积"), ("grid", "格点 X×Y×Z"),
                           ("points", "总点数"), ("design", "设计条件数"),
                           ("euler", "欧拉约定")):
            value = QtWidgets.QLabel("—")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.preview_labels[key] = value
            grid.addRow(title, value)
        layout.addLayout(grid)

        self.error_label = QtWidgets.QLabel("")
        self.error_label.setWordWrap(True)
        self.error_label.setStyleSheet(_STATUS_STYLES["error"])
        self.error_label.hide()
        layout.addWidget(self.error_label)

        self.warning_label = QtWidgets.QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet(_THRESHOLD_WARN)
        self.warning_label.hide()
        layout.addWidget(self.warning_label)

        buttons = QtWidgets.QHBoxLayout()
        buttons.setContentsMargins(0, 0, 0, 0)
        self.locate_button = QtWidgets.QPushButton("定位段")
        self.locate_button.setToolTip(
            "同步跑 core.locate_segment（约 0.3–0.5 s）：定位窗口内最危险的一段，"
            "不改任何参数")
        self.locate_button.clicked.connect(lambda: self.locate())
        self.repair_button = QtWidgets.QPushButton("修复")
        self.repair_button.setToolTip(
            "起一条后台任务：局部换端点姿态 → 不行再整体挪采样域 → 复核 → 落盘")
        self.repair_button.clicked.connect(self.start_repair)
        buttons.addWidget(self.locate_button)
        buttons.addWidget(self.repair_button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        layout.addWidget(_caption(
            "「定位段」只读不落盘；「修复」在后台线程跑，可取消（阶段边界生效）。"))
        return group

    def _build_viewer(self) -> QtWidgets.QWidget:
        """图 + 结果区：上下可拖的 ``QSplitter``（默认给图多一点高度）。"""
        if self._plot is not None:
            viewer = self._plot
        else:                                            # 纯逻辑测试/无图环境
            viewer = _caption("修复后的路点图在这里（本环境未挂 WebEngine）")
        result_area = self._build_result_area()
        splitter = QtWidgets.QSplitter(Qt.Vertical)
        splitter.addWidget(viewer)
        splitter.addWidget(result_area)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([420, 120])
        splitter.setChildrenCollapsible(False)
        self._viewer_splitter = splitter
        return splitter

    def _build_result_area(self) -> QtWidgets.QGroupBox:
        """结果区：状态行 + 进度/取消 + 四页签（定位/修复/计划/日志）+ 导出。"""
        group = QtWidgets.QGroupBox("结果")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.status_label.hide()
        layout.addWidget(self.status_label)

        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, PROGRESS_STEPS)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("空闲")
        self.cancel_button = QtWidgets.QPushButton("取消")
        self.cancel_button.setToolTip(
            "只在阶段边界生效：当前阶段跑完就停，不会硬打断 IK")
        self.cancel_button.clicked.connect(self._on_cancel_clicked)
        self.cancel_button.hide()
        progress_row = QtWidgets.QHBoxLayout()
        progress_row.setContentsMargins(0, 0, 0, 0)
        progress_row.addWidget(self.progress_bar, 1)
        progress_row.addWidget(self.cancel_button, 0)
        layout.addLayout(progress_row)

        self.result_tabs = QtWidgets.QTabWidget()
        self.locate_page = self._build_locate_page()
        self.compare_page = self._build_compare_page()
        self.plan_page = self._build_plan_page()
        self.log_page = self._build_log_page()
        self.result_tabs.addTab(self.locate_page, "定位")
        self.result_tabs.addTab(self.compare_page, "修复")
        self.result_tabs.addTab(self.plan_page, "计划")
        self.result_tabs.addTab(self.log_page, "日志")
        layout.addWidget(self.result_tabs, 1)

        export_row = QtWidgets.QHBoxLayout()
        export_row.setContentsMargins(0, 0, 0, 0)
        export_row.addWidget(QtWidgets.QLabel("导出："))
        for key, title, _filters in EXPORTS:
            button = QtWidgets.QPushButton(title)
            button.setEnabled(False)
            button.clicked.connect(
                lambda _checked=False, export_key=key: self._on_export(export_key))
            self.export_buttons[key] = button
            export_row.addWidget(button)
        export_row.addStretch(1)
        layout.addLayout(export_row)
        return group

    def _build_locate_page(self) -> QtWidgets.QScrollArea:
        """「定位」页：判定徽标 + 有效阈值 + 禁区数 + 段定位表。

        整页放在 ``QScrollArea`` 里：判定 + 一句说明 + ``note`` 的行数随现场读数
        变化，固定高度会把下面的段表挤到重叠（Qt 的布局在最小高度之和超过可用
        高度时不会报错，只是叠着画）。
        """
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.locate_empty_label = _caption(
            "还没有结果：先点「定位段」看窗口内哪一段最危险（不修改任何参数），"
            "再点「修复」让它自己选局部换姿态还是整体挪箱。")
        layout.addWidget(self.locate_empty_label)

        holder = QtWidgets.QWidget()
        self.verdict_layout = QtWidgets.QVBoxLayout(holder)
        self.verdict_layout.setContentsMargins(0, 0, 0, 0)
        self.verdict_layout.setSpacing(2)
        self.verdict_holder = holder
        self.verdict_holder.hide()
        layout.addWidget(self.verdict_holder)

        layout.addWidget(_caption(
            "报告段（黄底）与越界段（红底）；最差点 = 段内 min(|J3|,|J5|) 最小的"
            "位置（MOVL 直线插补），t 是段内位置 0..1。"))
        self.profile_table = make_table(PROFILE_HEADERS)
        self.profile_table.setSortingEnabled(False)   # 窗口顺序有意义，不排序
        # 表自己滚（段多时不把整页撑到几百像素）
        self.profile_table.setMinimumHeight(150)
        layout.addWidget(self.profile_table)
        layout.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        return scroll

    def _build_compare_page(self) -> QtWidgets.QScrollArea:
        """「修复」页：模式徽标 / 体积损失 / 边界表 / 局部改动 / 前后指标 / 候选日志。"""
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        head = QtWidgets.QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        title = QtWidgets.QLabel("修复结果")
        title.setStyleSheet("font-weight:bold")
        head.addWidget(title)
        self.mode_badge = _badge("—", MODE_COLOR["none"])
        self.mode_badge.hide()
        head.addWidget(self.mode_badge)
        self.label_value = QtWidgets.QLabel("—")
        head.addWidget(self.label_value)
        head.addStretch(1)
        layout.addLayout(head)

        self.volume_label = QtWidgets.QLabel("—")
        self.volume_label.setStyleSheet(_THRESHOLD_WARN)
        layout.addWidget(self.volume_label)

        self.bound_table = make_table(BOUND_HEADERS)
        self.bound_table.setSortingEnabled(False)
        self.bound_table.setMinimumHeight(150)
        layout.addWidget(self.bound_table)
        # ``mode=none`` 时代替上面那张表：6 行“前后一模一样”的数字是纯掎0位
        self.bound_note = _caption(BOUND_UNCHANGED_TEXT)
        self.bound_note.hide()
        layout.addWidget(self.bound_note)

        self.rpy_label = QtWidgets.QLabel("—")
        layout.addWidget(self.rpy_label)

        self.local_group = QtWidgets.QGroupBox("局部修复")
        local_form = QtWidgets.QFormLayout(self.local_group)
        for key, title_text in (("gp", "改了哪个 GP"),
                                ("delta_rpy", "delta_rpy"),
                                ("q_before", "q 修复前"),
                                ("q_after", "q 修复后")):
            value = QtWidgets.QLabel("—")
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.local_labels[key] = value
            local_form.addRow(title_text, value)
        self.local_group.hide()
        layout.addWidget(self.local_group)

        layout.addWidget(_caption("修复前 / 修复后的关键指标"))
        self.stat_table = make_table(COMPARE_HEADERS)
        self.stat_table.setSortingEnabled(False)
        self.stat_table.setMinimumHeight(200)
        layout.addWidget(self.stat_table)

        self.log_caption = _caption("候选日志（0/0 条，按改动量从小到大）")
        layout.addWidget(self.log_caption)
        self.log_table = make_table(LOG_HEADERS)
        self.log_table.setSortingEnabled(False)
        self.log_table.setMinimumHeight(180)
        layout.addWidget(self.log_table)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        return scroll

    def _build_plan_page(self) -> QtWidgets.QWidget:
        """「计划」页：统计行 + 路点表 + 段诊断表（渲染器复用生成页）。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        stats_holder = QtWidgets.QWidget()
        stats_row = QtWidgets.QHBoxLayout(stats_holder)
        stats_row.setContentsMargins(0, 0, 0, 0)
        stats_row.setSpacing(12)
        for key in STAT_KEYS:
            label = QtWidgets.QLabel("—")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.stat_labels[key] = label
            stats_row.addWidget(label)
        stats_row.addStretch(1)
        self.stats_holder = stats_holder
        self.stats_holder.hide()
        layout.addWidget(self.stats_holder)

        self.plan_status_label = QtWidgets.QLabel("")
        self.plan_status_label.setWordWrap(True)
        self.plan_status_label.hide()
        layout.addWidget(self.plan_status_label)

        self.run_label = _caption("尚未修复（还没有运行目录）")
        layout.addWidget(self.run_label)

        layout.addWidget(_caption("路点表（排序后失败行仍标红）"))
        self.waypoint_table = make_table(generate_page.WAYPOINT_HEADERS)
        layout.addWidget(self.waypoint_table, 3)
        layout.addWidget(_caption("段诊断表"))
        self.segment_table = make_table(generate_page.SEGMENT_HEADERS)
        layout.addWidget(self.segment_table, 2)
        return page

    def _build_log_page(self) -> QtWidgets.QWidget:
        """「日志」页：算法自身的 ``print`` 与 ``jobs`` 的提示（等价终端输出）。"""
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(_caption(
            f"等价终端输出。两层上限：后台任务源缓冲只留最近 {JOB_LOG_LIMIT} 行"
            f"（更早的在源头就丢了，丢一段会插一行「…已省略 N 行」），本控件最多"
            f"保留 {LOG_WIDGET_LIMIT} 行（超出删头部）。"))
        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setFont(generate_page.monospace_font())
        self.log_edit.setMaximumBlockCount(LOG_WIDGET_LIMIT)
        self.log_edit.setPlaceholderText("点「修复」后，这里实时滚日志。")
        layout.addWidget(self.log_edit, 1)
        return page

    # ------------------------------------------------------------------ 取值
    def values(self) -> dict:
        """当前表单值（键与生成页 ``values()`` 同源，另加修复流程的几项）。"""
        fields = self.fields
        return {
            "robot": fields["robot"].currentData() or ui_state.default_robot(),
            "domain_mode": (generate_page.DOMAIN_MODES[0]
                            if fields["domain_points"].isChecked()
                            else generate_page.DOMAIN_MODES[1]),
            "point_min": [fields["min_x"].value(), fields["min_y"].value(),
                          fields["min_z"].value()],
            "point_max": [fields["max_x"].value(), fields["max_y"].value(),
                          fields["max_z"].value()],
            "aabb6": [fields["x_min"].value(), fields["x_max"].value(),
                      fields["y_min"].value(), fields["y_max"].value(),
                      fields["z_min"].value(), fields["z_max"].value()],
            "grid_mode": (generate_page.GRID_MODES[0]
                          if fields["grid_count"].isChecked()
                          else generate_page.GRID_MODES[1]),
            "target_count": int(fields["target_count"].value()),
            "xs": int(fields["xs"].value()),
            "ys": int(fields["ys"].value()),
            "zs": int(fields["zs"].value()),
            "z_insert": int(fields["z_insert"].value()),
            "rpy": [fields["rpy_a"].value(), fields["rpy_b"].value(),
                    fields["rpy_c"].value()],
            "euler": fields["euler"].currentData(),
            "perturb_deg": [fields["perturb_a"].value(),
                            fields["perturb_b"].value(),
                            fields["perturb_c"].value()],
            "seed": int(fields["seed"].value()),
            "cond_max": float(fields["cond_max"].value()),
            "j5_min": float(fields["j5_min"].value()),
            "j3_min": float(fields["j3_min"].value()),
            "seg_min": float(fields["seg_min"].value()),
            "seg_samples": int(fields["seg_samples"].value()),
            "q_seed": fields["q_seed"].text(),
            # ---- 修复流程专用 ----
            "singular_q": [[float(box.value()) for box in row["fields"]]
                           for row in self.q_rows],
            "stopped_at": int(fields["stopped_at"].value()),
            "segment_window": int(fields["segment_window"].value()),
            "avoid_margin": float(fields["avoid_margin"].value()),
            "avoid_radius": float(fields["avoid_radius"].value()),
            "repair_attempts": int(fields["repair_attempts"].value()),
            "max_candidates": int(fields["max_candidates"].value()),
            "allow_rpy": bool(fields["allow_rpy"].isChecked()),
            "allow_shrink": bool(fields["allow_shrink"].isChecked()),
        }

    def params(self) -> dict:
        """当前表单值 -> ``core.locate_segment`` / ``core.run_repair`` 的入参。"""
        return form_to_params(self.values())

    def preview_result(self) -> dict:
        """最近一次 ``core.preview_domain`` 的结果（原样，含报错原文）。"""
        return self._preview or {}

    # ------------------------------------------------------------ 刷新与校验
    def _on_change(self, *_args) -> None:
        """任何输入的改动都走这里：重算预览卡片（``preview_domain`` 约 1 ms）。"""
        self.refresh_preview()

    def _on_domain_switch(self, checked: bool) -> None:
        """两点 / 六数切换：换面板并重算（只在被选中的那个按钮上动一次）。"""
        if not checked:
            return
        self.domain_group["stack"].setCurrentIndex(
            0 if self.fields["domain_points"].isChecked() else 1)
        self.refresh_preview()

    def _on_grid_switch(self, checked: bool) -> None:
        """目标点数 / 手填格点数切换。"""
        if not checked:
            return
        self.grid_group["stack"].setCurrentIndex(
            0 if self.fields["grid_count"].isChecked() else 1)
        self.refresh_preview()

    def refresh_preview(self) -> dict:
        """重算预览卡片、总点数，并按结果启用/禁用「修复」。

        校验只有一套：把表单收成 :func:`form_to_params` 的产物交给
        ``core.preview_domain``（内部走 CLI 的 ``_validate_args``）；出错就把
        **报错原文**写在卡片里（红字，不弹窗），不在界面里重写规则。

        Returns:
            dict: ``core.preview_domain`` 的结果。
        """
        values = self.values()
        try:
            result = core.preview_domain(form_to_params(values))
        except Exception as error:                      # noqa: BLE001 - 别打断 Qt 槽
            _LOGGER.exception("core.preview_domain 抛异常（预览卡片显示失败原因）")
            result = generate_page.preview_failure(error)
        self._preview = result
        self.total = generate_page.total_points(values, result)
        self._update_total_row()
        self._update_card(result)
        self._sync_buttons()
        return result

    def _update_total_row(self) -> None:
        """网格组里的总点数行（超上限时变红并给出说明）。"""
        total = getattr(self, "total", None)
        grid = getattr(self, "grid_group", None)
        if total is None or grid is None:
            return
        grid["total_value"].setText(total.text)
        grid["total_value"].setStyleSheet(
            generate_page._RED_BOLD if total.over else "")
        grid["total_hint"].setVisible(total.over)

    def _update_card(self, result: dict) -> None:
        """预览卡片：数值或 ``core`` 的报错原文（红字，不弹窗）。"""
        if not result.get("ok"):
            self.error_label.setText(str(result.get("error") or "预览失败"))
            self.error_label.show()
            self.warning_label.hide()
            for label in self.preview_labels.values():
                label.setText("—")
                label.setStyleSheet("")
            return
        self.error_label.hide()
        grid = result["grid"]
        self.preview_labels["volume"].setText(f"{result['volume_m3']:.3f} m³")
        insert = (f" + {grid['zs'] * grid['z_insert']} 插点"
                  if grid["z_insert"] else "")
        self.preview_labels["grid"].setText(
            f"{grid['xs'] + 1}×{grid['ys'] + 1}×{grid['zs'] + 1}{insert}")
        over = int(result["point_count"]) > generate_page.POINT_COUNT_WARN
        points = self.preview_labels["points"]
        points.setText(f"{result['point_count']} 点")
        points.setStyleSheet(generate_page._RED_BOLD if over else "")
        self.preview_labels["design"].setText(
            generate_page.format_condition(result["design_condition"]))
        self.preview_labels["euler"].setText(
            ui_state.euler_label(result["convention"]))
        warnings = [str(line) for line in (result.get("warnings") or [])]
        self.warning_label.setText("\n".join(warnings))
        self.warning_label.setVisible(bool(warnings))

    def _sync_buttons(self) -> None:
        """按“任务在不在跑 / 参数合不合法 / 点数超没超上限”启用两个按钮。

        点数上限这一条与生成页同一判据（``ok and not total.over and not busy``）：
        80 万点的修复会真去 plan + IK，而取消只在阶段边界生效——不能只因为
        ``preview ok`` 就让它点得下去（S4 评审 Q2）。
        """
        running = self._busy or self.bridge.is_running
        valid = bool((self._preview or {}).get("ok"))
        over = bool(getattr(self.total, "over", False))
        self.repair_button.setEnabled(valid and not over and not running)
        self.locate_button.setEnabled(not running)

    def param_fingerprint(self) -> str:
        """关键参数的一句话指纹（让现场把结果对回到具体一版参数）。

        修复期间表单**不锁**（锁了反倒改不动），所以指纹在**提交时**抖一次存
        进 :attr:`run_fingerprint`，终态再写回状态行——不然跑完再看表单，
        看到的可能是另一版参数。

        Returns:
            str: 形如 ``j5_min=20.0° · perturb=±5.0/5.0/5.0° · stopped_at=GP4``。
        """
        values = self.values()
        perturb = values["perturb_deg"]
        stopped = values["stopped_at"]
        return (f"j5_min={values['j5_min']:.1f}° · "
                f"perturb=±{perturb[0]:.1f}/{perturb[1]:.1f}/{perturb[2]:.1f}° · "
                f"stopped_at={'—' if not stopped else f'GP{int(stopped)}'}")

    # ------------------------------------------------------------ 定位（同步）
    def _begin_wait(self, message: str) -> None:
        """进等待态：等待光标 + 状态栏「定位中…」（规格要求的两件事）。"""
        QtWidgets.QApplication.setOverrideCursor(QtGui.QCursor(Qt.WaitCursor))
        self.last_status = str(message)
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(self.last_status)
        self._pump_events()

    def _end_wait(self) -> None:
        """退出等待态（恢复光标）。"""
        if QtWidgets.QApplication.overrideCursor() is not None:
            QtWidgets.QApplication.restoreOverrideCursor()

    @staticmethod
    def _pump_events() -> None:
        """处理非输入事件（让光标与状态栏真的画出来），屏蔽鼠标/键盘避免重入。"""
        try:
            flags = QtCore.QEventLoop.ExcludeUserInputEvents
        except AttributeError:                          # pragma: no cover - 严格枚举
            flags = QtCore.QEventLoop.ProcessEventsFlag.ExcludeUserInputEvents
        try:
            QtWidgets.QApplication.processEvents(flags)
        except TypeError:                               # pragma: no cover - 老绑定
            QtWidgets.QApplication.processEvents()

    def locate(self):
        """「定位段」：同步跑 ``core.locate_segment``（0.3–0.5 s），结果直接渲染。

        Returns:
            dict | None: ``core.locate_segment`` 的返回值；任务在跑 / 重入时 ``None``。
        """
        if self._locating:
            return None
        if self.bridge.is_running:
            self._set_status("修复任务在跑，等它结束再看定位。", level="muted")
            self._report_status("修复任务在跑，未定位")
            return None
        params = self.params()
        self._locating = True
        self._begin_wait("定位中…")
        try:
            result = core.locate_segment(params)
        except Exception as error:                      # noqa: BLE001 - core 不抛异常
            _LOGGER.exception("core.locate_segment 抛异常（params=%r）", params)
            result = {"ok": False,
                      "error": f"{type(error).__name__}: {error}",
                      "warnings": []}
        finally:
            self._end_wait()
            self._locating = False
        self.render_locate(result)
        return result

    def render_locate(self, result: dict) -> None:
        """把 ``core.locate_segment`` 的返回值铺到「定位」页签。

        ``{"ok": False, "error": ...}`` 时把报错原文写在结果区状态行（红字），
        并清掉上一次的判定/段表——**不弹窗**，现场要能回看改哪一项。

        Args:
            result: ``core.locate_segment`` 的返回值（真的或构造的都行）。
        """
        if not isinstance(result, dict):
            result = {"ok": False, "error": "定位返回了非 dict 结果", "warnings": []}
        if result.get("ok"):
            self.last_located = result
            self.locate_error = None
            self._render_verdict(result)
            self.result_tabs.setCurrentWidget(self.locate_page)
            verdict = str(result.get("verdict") or "")
            self._set_status(f"定位完成：{verdict}", level="ok")
            self._report_status(f"定位完成：{verdict}")
            return
        self.last_located = None
        self.locate_error = result
        form_widgets.clear_layout(self.verdict_layout)
        self.kind_badges = []
        self.threshold_labels = []
        self.hint_labels = []
        self.detail_labels = []
        self.verdict_holder.hide()
        self.locate_empty_label.show()
        self._fill_profile_table([])
        text = f"定位失败：{result.get('error') or '未知错误'}"
        warnings = [str(line) for line in (result.get("warnings") or [])]
        if warnings:
            text += "\n" + "\n".join(f"· {line}" for line in warnings)
        self._set_status(text, level="error")
        self.result_tabs.setCurrentWidget(self.locate_page)
        self._report_status(f"定位失败：{result.get('error') or '未知错误'}")

    # ------------------------------------------------- 判定徽标 + 段定位表
    def _render_verdict(self, view: dict) -> None:
        """判定徽标 + 一句话说明 + 有效阈值 + 禁区数量 + 段定位表。

        Args:
            view: ``locate_segment`` 的返回值，或 ``run_repair()["repair"]``。
        """
        form_widgets.clear_layout(self.verdict_layout)
        self.kind_badges = []
        self.threshold_labels = []
        self.hint_labels = []
        self.detail_labels = []

        chips_holder = QtWidgets.QWidget()
        chips_column = QtWidgets.QVBoxLayout(chips_holder)
        chips_column.setContentsMargins(0, 0, 0, 0)
        chips_column.setSpacing(4)
        # 第一行：判定徽标（一种判定一个）；第二行：有效阈值 + 禁区数量。
        # 挤成一行会在窄窗口里把最后一个徽标切掉。
        kind_row = QtWidgets.QWidget()
        chips = QtWidgets.QHBoxLayout(kind_row)
        chips.setContentsMargins(0, 0, 0, 0)
        chips.setSpacing(6)
        for row in view.get("classifications") or []:
            kind = row.get("kind")
            badge = _badge(kind_text(kind), KIND_COLOR.get(kind, "#9e9e9e"))
            badge.setToolTip(str(row.get("note") or ""))
            self.kind_badges.append(badge)
            chips.addWidget(badge)
        chips.addStretch(1)
        chips_column.addWidget(kind_row)

        thresholds_holder = QtWidgets.QWidget()
        threshold_row = QtWidgets.QHBoxLayout(thresholds_holder)
        threshold_row.setContentsMargins(0, 0, 0, 0)
        threshold_row.setSpacing(6)
        thresholds = view.get("effective_thresholds") or {}
        threshold_text = (f"有效阈值 |J5|≥{_fmt(thresholds.get('j5_min_deg'), 2)}°"
                          f" · |J3|≥{_fmt(thresholds.get('j3_min_deg'), 2)}°")
        threshold = _badge(threshold_text, _CHIP_BG)
        self.threshold_labels.append(threshold)
        threshold_row.addWidget(threshold)
        if thresholds.get("j3_seg_min_deg"):
            segment = _badge("段内 |J3|≥"
                             f"{_fmt(thresholds['j3_seg_min_deg'], 2)}°", _CHIP_BG)
            self.threshold_labels.append(segment)
            threshold_row.addWidget(segment)
        forbidden = _badge(
            f"位置禁区 {len(view.get('forbidden_positions') or [])} 个", _CHIP_BG)
        self.threshold_labels.append(forbidden)
        threshold_row.addWidget(forbidden)
        threshold_row.addStretch(1)
        chips_column.addWidget(thresholds_holder)
        self.verdict_layout.addWidget(chips_holder)

        # 一句引导语：判定的**解释**只由 ``note`` 说（那份带数据来源），
        # 页面不再复述一遍（S4 评审：两句话叠着说同一件事）。
        if view.get("classifications"):
            guide = _caption(VERDICT_GUIDE)
            self.hint_labels.append(guide)
            self.verdict_layout.addWidget(guide)

        for row in view.get("classifications") or []:
            detail = QtWidgets.QLabel(
                f"q={[round(float(value), 3) for value in row.get('q_deg') or []]}"
                f" · |J3|={float(row.get('j3_abs') or 0.0):.2f}°"
                f" · |J5|={float(row.get('j5_abs') or 0.0):.2f}°")
            detail.setWordWrap(True)
            detail.setStyleSheet("color:#374151")
            self.detail_labels.append(detail)
            self.verdict_layout.addWidget(detail)
            if row.get("note"):
                note = _caption(f"· {row['note']}")
                self.hint_labels.append(note)
                self.verdict_layout.addWidget(note)

        self.verdict_label = QtWidgets.QLabel(str(view.get("verdict") or "—"))
        self.verdict_label.setWordWrap(True)
        self.verdict_label.setStyleSheet(
            "font-weight:bold;" + (
                _THRESHOLD_OK if "不动采样域" in str(view.get("verdict") or "")
                else _THRESHOLD_WARN))
        self.verdict_layout.addWidget(self.verdict_label)

        stopped = view.get("stopped_at")
        profiles = view.get("profiles") or []
        if stopped is not None:
            self.verdict_layout.addWidget(QtWidgets.QLabel(
                f"段定位（已到 GP{stopped}，故障应在 "
                f"GP{stopped}→GP{int(stopped) + 1}）"))
        if not profiles:
            self.verdict_layout.addWidget(_caption(
                "没有窗口剖面：留空「已走到 GP」时只判断整链。"
                if stopped is None else "窗口内没有可检查的段。"))
        self._fill_profile_table(profile_rows(view) if profiles else [])
        self.verdict_holder.show()
        self.locate_empty_label.hide()

    def _fill_profile_table(self, rows) -> None:
        """段定位表：报告段黄底、越界段红底（红底优先，与网页版行模板一致）。"""
        table = self.profile_table
        table.setSortingEnabled(False)
        table.clearContents()
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            cells = [
                ((0, row["gp"]), row["gp"], "center"),
                (sort_key(row["j3"]), _fmt(row["j3"]), "right"),
                (sort_key(row["j5"]), _fmt(row["j5"]), "right"),
                ((0, row["crosses"]), str(row["crosses"]), "center"),
                ((0, row["worst"]), row["worst"], "left"),
                (sort_key(row["frac"]), _fmt(row["frac"]), "right"),
            ]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "center": Qt.AlignCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                if row.get("failed"):
                    item.setBackground(QtGui.QColor(_OFFENDER_BG))
                    item.setForeground(QtGui.QColor(_OFFENDER_FG))
                elif row.get("reported"):
                    item.setBackground(QtGui.QColor(_REPORTED_BG))
                table.setItem(row_index, column, item)
        table.resizeColumnsToContents()

    # ------------------------------------------------------------ 修复结果渲染
    def _render_compare(self, repair: dict) -> None:
        """模式徽标 / 体积损失 / 边界 before→after / 局部改动 / 前后指标。"""
        mode = repair.get("mode")
        self.mode_badge.setText(mode_text(mode))
        self.mode_badge.setStyleSheet(
            f"background:{MODE_COLOR.get(mode, MODE_COLOR['none'])}; color:#ffffff;"
            "border-radius:8px; padding:2px 10px; font-weight:bold;")
        self.mode_badge.show()
        self.label_value.setText(str(repair.get("label") or "—"))
        # 没有体积损失数据就写「—」，不把“没数据”说成“0 损失”。
        # 百分比要用 ``.2%``（0.1234 -> 12.34%），所以不能直接套 :func:`_fmt`
        # 的定点格式（那会把 0.1234 写成 0.12）。
        loss = repair.get("volume_loss")
        self.volume_label.setText(
            "体积损失 —" if loss is None else f"体积损失 {float(loss):.2%}")
        # ``mode=none`` = 一整份都没动：前后必然相同的 6 行数字没有任何信息，
        # 换成一句说明（其它模式照旧出表）
        unchanged = mode == "none"
        self._fill_bound_table([] if unchanged else bound_rows(repair))
        self.bound_table.setVisible(not unchanged)
        self.bound_note.setVisible(unchanged)

        rpy_before = repair.get("rpy_before") or []
        rpy_after = repair.get("rpy_after") or []
        if rpy_before and rpy_after:
            changed = any(abs(float(a) - float(b)) > 1e-9
                          for a, b in zip(rpy_before, rpy_after))
            text = ("参考姿态 " + " / ".join(f"{float(v):.1f}" for v in rpy_before)
                    + ("  →  " + " / ".join(f"{float(v):.1f}" for v in rpy_after)
                       if changed else "（未改）"))
            self.rpy_label.setText(text)
            self.rpy_label.setStyleSheet(_THRESHOLD_WARN if changed else "")
        else:
            self.rpy_label.setText("参考姿态 —")
            self.rpy_label.setStyleSheet("")

        local = repair.get("local")
        if local:
            self.local_labels["gp"].setText(
                f"GP{local.get('gp')}（位置与网格不变，只重采端点姿态）")
            # ``local`` 里的四个键都当可能缺：硬下标会把“有一项没回填”变成崩溃
            delta = local.get("delta_rpy") or []
            self.local_labels["delta_rpy"].setText(
                ", ".join(f"{float(value):+.2f}" for value in delta)
                if delta else "—")
            self.local_labels["q_before"].setText(q_text(local.get("q_before")))
            self.local_labels["q_after"].setText(q_text(local.get("q_after")))
            self.local_group.show()
        else:
            for label in self.local_labels.values():
                label.setText("—")
            self.local_group.hide()
        self._fill_compare_table(stat_rows(repair))

    def _fill_bound_table(self, rows) -> None:
        """边界表：变化（|Δ|>1e-9）的行标黄。"""
        table = self.bound_table
        table.setSortingEnabled(False)
        table.clearContents()
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            cells = [
                ((0, row["axis"]), row["axis"], "left"),
                (sort_key(row["before"]), _fmt(row["before"], 1), "right"),
                (sort_key(row["after"]), _fmt(row["after"], 1), "right"),
                (sort_key(row["delta"]), _fmt(row["delta"], 1), "right"),
            ]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                if row.get("changed"):
                    item.setBackground(QtGui.QColor(_CHANGED_BG))
                table.setItem(row_index, column, item)
        table.resizeColumnsToContents()

    def _fill_compare_table(self, rows) -> None:
        """修复前/后的关键指标对照表（缺数据写 ``—``）。"""
        table = self.stat_table
        table.setSortingEnabled(False)
        table.clearContents()
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            cells = [((0, row["item"]), row["item"], "left"),
                     ((0, row["before"]), row["before"], "right"),
                     ((0, row["after"]), row["after"], "right")]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                table.setItem(row_index, column, item)
        table.resizeColumnsToContents()

    def _fill_log_table(self, repair: dict) -> None:
        """候选日志表：行 key 用序号（``label`` 会重复），失败行红字。"""
        rows = log_rows(repair)
        # 键存在但为 ``None``（如整体搜索一条候选都没跑）时不能写出「1/None 条」
        total = repair.get("log_total") or len(rows)
        self.log_caption.setText(
            f"候选日志（{len(rows)}/{total} 条，按改动量从小到大；"
            "阶段=原域/single/pair/refine，可行 ✗ 行是失败候选）")
        table = self.log_table
        table.setSortingEnabled(False)
        table.clearContents()
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            cells = [((0, row["phase"]), row["phase"], "center"),
                     ((0, row["label"]), row["label"], "left"),
                     ((0, 1 if row["ok"] == "✓" else 0), row["ok"], "center"),
                     (sort_key(row["score"]), _fmt(row["score"], 2), "right"),
                     ((0, row["reasons"]), row["reasons"], "left")]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "center": Qt.AlignCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                if row.get("failed"):
                    item.setForeground(QtGui.QColor(_OFFENDER_FG))
                table.setItem(row_index, column, item)
        table.resizeColumnsToContents()

    def render_repair(self, repair: dict) -> None:
        """修复块（``result["repair"]``）：判定 + 前后对比 + 候选日志。

        Args:
            repair: ``core.run_repair()["repair"]``（真的或构造的都行）。
        """
        self._render_verdict(repair)
        self._render_compare(repair)
        self._fill_log_table(repair)

    def render_plan(self, result: dict) -> None:
        """「计划」页签：统计行 + 路点表 + 段诊断表 + 图 + 导出按钮。

        Args:
            result: ``core.run_repair`` 的成功返回值（与 ``run_scan`` 同构）。
        """
        stats = result.get("stats") or {}
        fill_stat_labels(self.stat_labels, stats)
        self.stats_holder.setVisible(bool(stats))
        fill_waypoint_table(self.waypoint_table, result.get("waypoints") or [])
        fill_segment_table(self.segment_table, result.get("segments") or [])
        self.waypoint_table.scrollToTop()
        warnings = [str(line) for line in (result.get("warnings") or [])]
        head = ""
        feasible = stats.get("feasible")
        if feasible is False:
            head = "修复后的计划不可行，原因："
        self.plan_status_label.setText(
            head + "\n".join(f"· {line}" for line in warnings))
        self.plan_status_label.setStyleSheet(
            _STATUS_STYLES["error"] if feasible is False else _THRESHOLD_WARN)
        self.plan_status_label.setVisible(bool(head or warnings))
        run_id = result.get("run_id")
        run_dir = result.get("run_dir")
        self.run_label.setText(
            f"运行 {run_id} · {run_dir}" if run_id or run_dir
            else "尚未修复（还没有运行目录）")
        self._update_exports(result)
        try:
            figure = generate_page.result_figure(result)
        except Exception as error:                      # noqa: BLE001 - 别让窗口崩
            _LOGGER.exception("修复后计划的出图失败（运行 %r）", run_id)
            figure = generate_page.placeholder_figure(
                f"结果图渲染失败：{type(error).__name__}: {error}")
            self._report_status(
                f"结果图渲染失败：{type(error).__name__}: {error}")
        self.last_figure = figure
        if self._plot is not None:
            self._plot.set_figure(figure)
        self._expand_result_area()

    def render_result(self, result: dict) -> None:
        """把 ``core.run_repair`` 的成功返回值铺到结果区（判定 / 对比 / 日志 / 计划）。

        Args:
            result: ``core.run_repair`` 的返回值（参数是真的还是构造的都行）。
        """
        self.last_result = result
        repair = result.get("repair")
        if isinstance(repair, dict) and repair:
            self.last_located = None        # 结果里自带判定信息，避免两份并存
            self.render_repair(repair)
            self.result_tabs.setCurrentWidget(self.compare_page)
        self.render_plan(result)

    def _expand_result_area(self) -> None:
        """出结果后把结果区撑到 :data:`RESULT_MIN_HEIGHT`（默认 120 px 只够看表头）。"""
        splitter = getattr(self, "_viewer_splitter", None)
        if splitter is None:
            return
        sizes = splitter.sizes()
        if len(sizes) != 2 or sizes[1] >= RESULT_MIN_HEIGHT:
            return
        viewer = max(sizes[0] - (RESULT_MIN_HEIGHT - sizes[1]), 120)
        splitter.setSizes([viewer, RESULT_MIN_HEIGHT])

    def _update_exports(self, result: dict) -> None:
        """按 ``result["artifacts"]`` 启用/禁用四个导出按钮（文件真的在才开）。"""
        artifacts = result.get("artifacts") or {}
        run_dir = result.get("run_dir")
        for key, button in self.export_buttons.items():
            name = artifacts.get(key)
            path = None if (name is None or run_dir is None) \
                else Path(run_dir) / str(name)
            available = path is not None and path.is_file()
            button.setEnabled(available)
            button.setToolTip("" if available else f"本次运行没有 {key} 产物")

    def _on_export(self, key: str) -> None:
        """导出按钮：选目标后**复制**（不移动）；实现与生成页共用同一份。"""
        level, message = export_artifact(self.last_result or {}, key, EXPORTS,
                                         self)
        if not message:                     # 用户取消
            return
        if level == "error":
            self._set_status(message, level="error")
        self._report_status(message)

    # ------------------------------------------------------------ 起任务
    def start_repair(self):
        """起一次**真修复**（``bridge`` 后台线程跑 ``core.run_repair``）。

        Returns:
            str | None: 任务 id；未启动（点数超上限 / 已有任务在跑 / 提交失败）
            时为 ``None``。
        """
        if self.bridge.is_running:
            self._set_status("已有任务在跑：等它结束，或先点「取消」。",
                             level="muted")
            self._report_status("已有任务在跑，未重复提交")
            return None
        # 硬守卫（不能只靠按钮置灰）：80 万点的修复会真去 plan + IK，
        # 取消只在阶段边界生效，提交下去就只能等。
        total = getattr(self, "total", None)
        if total is not None and total.over:
            message = (f"当前设置 {total.count} 点，超过 {POINT_COUNT_WARN} 点上限，"
                       f"先减少点数")
            self._set_status(message, level="error")
            self._report_status("点数超上限，未提交修复")
            return None
        params = self.params()
        # 表单不锁：把提交这一版的指纹先存下来，终态写回状态行
        self.run_fingerprint = self.param_fingerprint()
        self._reset_result_area()
        self._busy = True
        self.run_state = "running"
        self._sync_buttons()
        self.cancel_button.show()
        self.cancel_button.setEnabled(True)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("排队 0/0 · 排队中")
        self.result_tabs.setCurrentWidget(self.log_page)
        self._set_status("已提交修复任务：定位段 → 局部修复 → 整体搜索 → 复核 → 落盘\n"
                         f"参数（提交时）：{self.run_fingerprint}", level="info")
        try:
            job_id = self.bridge.submit(
                "repair",
                lambda job: core.run_repair(params, job=job,
                                            on_progress=job.set_progress))
        except Exception as error:                      # noqa: BLE001 - 别让槽抛出去
            _LOGGER.exception("提交修复任务失败（params=%r）", params)
            self._busy = False
            self.run_state = "failed"
            self.cancel_button.hide()
            self._sync_buttons()
            self.progress_bar.setFormat("失败")
            message = f"提交任务失败：{type(error).__name__}: {error}"
            self._set_status(message, level="error")
            self._report_status(message)
            return None
        self.last_job_id = job_id
        self._report_status(f"已提交修复任务 {job_id}")
        return job_id

    def _resume_running_job(self) -> None:
        """启动时扫一遍本桥登记表里未结束的 ``repair`` 任务，有就接回来。"""
        job = adopt_running_job(self.bridge.registry, "repair")
        if job is None:
            return
        if self.bridge.attach(job.id) is None:          # 刚好被清掉
            return
        self._busy = True
        self.run_state = "running"
        self._sync_buttons()
        self.cancel_button.show()
        self.cancel_button.setEnabled(True)
        self._set_status(
            f"接管本进程里还没跑完的任务 {job.id}"
            f"（当前 {STATE_TEXT.get(job.state, job.state)}）", level="info")

    def _on_cancel_clicked(self) -> None:
        """请求取消：只在阶段边界生效（不会硬打断 IK）。"""
        if not self.bridge.cancel():
            self.cancel_button.setEnabled(False)
            self._report_status("任务已结束，无需取消")
            return
        self.cancel_button.setEnabled(False)
        self._set_status("已请求取消：当前阶段跑完就停（不会硬打断 IK）。",
                         level="muted")
        self._report_status("已请求取消")

    # ------------------------------------------------------- 信号（GUI 线程）
    def _on_progress(self, progress: dict) -> None:
        """进度：进度条显示 ``阶段 索引/总数 · 状态``。"""
        stage = str(progress.get("stage") or "")
        index = int(progress.get("index") or 0)
        total = int(progress.get("total") or 0)
        fraction = float(progress.get("fraction") or 0.0)
        state = str(progress.get("state") or "")
        self.progress_bar.setValue(int(round(max(0.0, min(1.0, fraction))
                                            * PROGRESS_STEPS)))
        text = f"{stage} {index}/{total}"
        if state:
            text += f" · {STATE_TEXT.get(state, state)}"
        self.progress_bar.setFormat(text)

    def _on_log_line(self, line: str) -> None:
        """增量日志：直接追加（上限由 ``setMaximumBlockCount`` 管）。"""
        self.log_edit.appendPlainText(str(line))

    def _on_finished(self, payload: dict) -> None:
        """终态：done → 渲染结果；cancelled → 灰字；failed → 红字。

        任何终态都要把 ``result`` 接管过来（``last_result`` / ``run_label`` /
        导出按钮）：``ok=False``（所有候选都不可行）与 ``cancelled`` 时 ``core``
        往往已经把 ``repair.json`` 写在盘上了，错误原文里就写着“详见
        repair.json”——不给导出入口、还写“还没有运行目录”就是自相矛盾
        （S4 评审 Q1）。
        """
        state = str(payload.get("state") or "")
        result = payload.get("result")
        self._busy = False
        self.run_state = state
        self.cancel_button.hide()
        self.cancel_button.setEnabled(True)
        self._sync_buttons()
        # 修复期间表单没锁：状态行带上“提交那一刻”的参数指纹，避免跑完以后
        # 拿改过的表单去对旧结果。
        fingerprint = getattr(self, "run_fingerprint", "") or ""
        suffix = f"\n参数（提交时）：{fingerprint}" if fingerprint else ""

        if state == "done" and isinstance(result, dict) and result.get("ok"):
            self.progress_bar.setValue(PROGRESS_STEPS)
            self.progress_bar.setFormat("完成 1/1 · 完成")
            self.render_result(result)
            repair = result.get("repair") or {}
            mode = mode_text(repair.get("mode"))
            text = (f"完成：{mode}；运行 {result.get('run_id')} → "
                    f"{result.get('run_dir')}")
            warnings = [str(line) for line in (result.get("warnings") or [])]
            if warnings:
                text += "\n备注：" + "；".join(warnings)
            self._set_status(text + suffix, level="ok")
            self._report_status(
                f"修复完成：{mode}，{result.get('run_id')}")
            return

        if state == "done" and isinstance(result, dict):
            # core 返回 ok=False（参数非法 / 所有候选都不可行）：原文留在结果区，
            # 同时接管结果本身——不可行时 core 也落了 repair.json，那就要有入口。
            self.last_result = result
            run_id = result.get("run_id")
            run_dir = result.get("run_dir")
            if run_id or run_dir:
                self.run_label.setText(f"运行 {run_id} · {run_dir}")
            self._update_exports(result)    # 只会点亮真实存在的那几项
            repair = result.get("repair")
            if isinstance(repair, dict) and repair:
                self.render_repair(repair)      # 不可行也把对比与候选日志画出来
            error = result.get("error") or "未知错误"
            warnings = [str(line) for line in (result.get("warnings") or [])]
            text = f"失败：{error}"
            if warnings:
                text += "\n" + "\n".join(f"· {line}" for line in warnings)
            self.progress_bar.setFormat("失败")
            self._set_status(text + suffix, level="error")
            self.result_tabs.setCurrentWidget(self.log_page)
            self._report_status(f"修复失败：{error}")
            return

        if state == "cancelled":
            self.progress_bar.setFormat("已取消")
            # 取消也要接管：没来得及落盘的目录 core 会删，但已经写出
            # repair.json 的目录会保留——那就得能导出、并显示真实运行目录。
            carried = result if isinstance(result, dict) else {}
            run_id = carried.get("run_id") or payload.get("run_id")
            run_dir = carried.get("run_dir") or payload.get("run_dir")
            artifacts = carried.get("artifacts") or (
                dict(ARTIFACT_NAMES) if run_dir else {})
            self.last_result = {**carried, "run_id": run_id, "run_dir": run_dir,
                                "artifacts": artifacts}
            if run_id or run_dir:
                self.run_label.setText(f"运行 {run_id} · {run_dir}")
            self._update_exports(self.last_result)
            self._set_status(
                "已取消：任务在当前阶段末尾停下；没来得及落盘的运行目录已删掉，"
                "已经写出 repair.json 的目录会保留。" + suffix, level="muted")
            self._report_status("修复已取消")
            return

        error = ((result or {}).get("error") if isinstance(result, dict) else None) \
            or payload.get("error") or "未知错误"
        text = f"失败：{error}"
        self.progress_bar.setFormat("失败")
        self._set_status(text + suffix, level="error")
        self.result_tabs.setCurrentWidget(self.log_page)
        self._report_status(f"修复失败：{error}")

    # ------------------------------------------------------------ 状态与清理
    def _reset_result_area(self) -> None:
        """起新任务前清空结果区（上一次的表、图、日志不留在界面上冒充新结果）。

        图也算结果：``last_figure`` 不清、``_plot`` 不换，上一次成功的图会一直
        贴在画布上，切页回来还会被 :meth:`current_figure` 重推一次。
        """
        self.last_result = None
        self.last_located = None
        self.locate_error = None
        self.last_figure = None
        if self._plot is not None:
            self._plot.set_figure(
                generate_page.placeholder_figure("正在修复：新任务已提交"))
        self.log_edit.clear()
        form_widgets.clear_layout(self.verdict_layout)
        self.kind_badges = []
        self.threshold_labels = []
        self.hint_labels = []
        self.detail_labels = []
        self.verdict_holder.hide()
        self.locate_empty_label.show()
        for table in (self.profile_table, self.bound_table, self.stat_table,
                      self.log_table, self.waypoint_table, self.segment_table):
            table.setSortingEnabled(False)
            table.clearContents()
            table.setRowCount(0)
        self.mode_badge.hide()
        self.label_value.setText("—")
        self.volume_label.setText("—")
        self.rpy_label.setText("—")
        self.bound_table.show()
        self.bound_note.hide()
        self.local_group.hide()
        for label in self.local_labels.values():
            label.setText("—")
        self.log_caption.setText("候选日志（0/0 条，按改动量从小到大）")
        for label in self.stat_labels.values():
            label.setText("—")
            label.setStyleSheet("")
        self.stats_holder.hide()
        self.plan_status_label.clear()
        self.plan_status_label.hide()
        self.run_label.setText("尚未修复（还没有运行目录）")
        for button in self.export_buttons.values():
            button.setEnabled(False)
            button.setToolTip("")
        self.status_label.clear()
        self.status_label.hide()

    def _set_status(self, text: str, level: str = "info") -> None:
        """结果区状态行：``ok`` 绿 / ``error`` 红 / ``muted`` 灰 / ``info`` 正常。"""
        message = str(text or "")
        self.last_status = message
        self.status_label.setText(message)
        self.status_label.setStyleSheet(_STATUS_STYLES.get(level, ""))
        self.status_label.setVisible(bool(message.strip()))

    def _report_status(self, message: str) -> None:
        """把一句话写到窗口状态栏（没有主窗口的纯逻辑测试里也能断言）。"""
        self.last_status = str(message)
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(self.last_status, 10_000)

    def shutdown(self) -> None:
        """窗口关闭时收尾：注销任务订阅（不取消任务）。"""
        self.bridge.shutdown()

    def current_figure(self):
        """本页当前该展示的图（导航切回来时主窗口用它重推）。

        还没有修复结果时给一张说明图，而不是让生成页的图留在同一个 PlotView 上
        （那会让人以为修复页也有结果）；任务在跑时说明图直接写“正在修复”。

        Returns:
            plotly ``Figure``。
        """
        if self.last_figure is not None:
            return self.last_figure
        if self.run_state == "running":
            # 任务在跑：既不能拿旧图充新结果，也不该说“还没有结果”
            return generate_page.placeholder_figure(
                "正在修复：任务在跑，结果出来会显示在这里。",
                title="修复后的路点图（进行中）")
        return generate_page.placeholder_figure(
            "还没有修复结果：点「修复」后这里显示修复后的计划。",
            title="修复后的路点图（未出图）")

    def show_webgl_warning(self) -> None:
        """WebGL 不可用时在页面顶部挂一条中文建议（与生成页同一段文案）。"""
        self.banner.setText(
            "⚠ 这台机器的 WebGL 不可用，Plotly 3D 画不出来。\n"
            "　处理办法：① 更新显卡驱动；② 远程桌面下改在本机打开；"
            "③ 先用网页版（python -m laser_web）看图，功能与数值一致。")
        self.banner.setStyleSheet(
            "background:#fff4e5; color:#8a4b00; padding:8px;"
            "border:1px solid #f0c48a;")
        self.banner.show()
