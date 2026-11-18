# -*- coding: utf-8 -*-
"""工具页（S5）：GP 导出 / JSON → URDF / 运行目录 / 模板文件。

行为基准是网页版 ``laser_web/pages/tools.py``（359 行）：三张卡片各管一件事，
页面只做「收集输入 → 调 :mod:`laser_web.core` → 渲染」，算法与文件读取全部在
服务层。桌面版按 S5 规格把模板信息单独拆成第四块（只读摘要），所以是**四块
纵向排列、各自独立**：

1. **GP 导出**：选一次运行（``core.list_runs()``，标出有没有 ``scan.json``，
   没有的不能选）+ 选 ``position_R1.json`` 模板 → ``core.build_gp``；结果显示
   写了几个 GP、GP 号区间、模板 sha256 前后是否一致（**必须不变**；变了就是
   **硬失败**：红字 + 禁用下载 + 不写「已写入 N 个 GP」）、GP 副本路径与下载
   按钮；
2. **JSON → URDF**：选控制器 JSON + 填型号（``--model`` 后缀）→
   ``core.convert_urdf``；结果显示 URDF 路径、关节数、末端位姿与下载按钮；
3. **运行目录**：表格（运行号 / 时间 / 大小 / 产物标记）+「刷新」「清理（保留
   最近 50）」→ :meth:`ToolsPage.clean_runs`，回报删了几个；
4. **模板文件**：``position_R1_template.json`` 的 sha256 与字节数（只读）。
   它**只**从环境变量 / 可写数据目录 / ``data/configs`` 三处找——没有
   “顺手把 ``~/Documents/backup`` 里那份也拿来显示”的兜底：那样会在客户机上
   毫无标识地炫耀一份旧模板的 sha256，容易被当成交付基线抄走。

上传一律走 ``QFileDialog`` 同步读（桌面版不需要 ``await``），**读之前先判
大小**：两个上传口各 16 MB，超限直接红字拒绝、**不读进内存**
（:func:`read_limited` 先 ``stat()`` 再 ``read_bytes()``）。

两处与 ``core`` 的接口差异（报告里也写了）：

* ``core.clean_runs`` 没有 ``exclude`` 参数，它只排除
  ``jobs.get_registry()`` 里的活跃目录（网页版的进程级登记表）。桌面版每个
  页面自带一个 ``JobRegistry``（见 :mod:`laser_desktop.job_bridge` 的边界
  说明），于是桌面版在跑的任务目录它看不见——直接调会删掉正在写产物的目录。
  所以 :meth:`ToolsPage.clean_runs` 在没有活跃目录时调 ``core.clean_runs``
  （与网页版同一条路径），有活跃目录时调它**内部用的同一个**
  ``jobs.prune_runs`` 并把 ``exclude`` 一起传进去；
* ``core.convert_urdf`` 只回路径 / 文件名 / 自校验误差，规格要的「关节数、
  末端位姿」由 :func:`urdf_summary` 从生成的 URDF 里数 / 算（纯显示，不参与
  任何校验）。
"""

from __future__ import annotations

import hashlib
import logging
import os
import shutil
import xml.etree.ElementTree as ElementTree
from datetime import datetime
from pathlib import Path

from laser_web import core, jobs, ui_state

from . import paths
from .generate_page import (
    _caption,
    _int_spin,
    _inline,
    _row,
    make_table,
    monospace_font,
    SortItem,
)
from .qt_compat import Qt, QtWidgets

#: 日志器（``core`` 万一抛异常、URDF 摘要算不出来都记在这里）。
_LOGGER = logging.getLogger(__name__)

#: 两个上传口的大小上限（字节）。真实的模板 / 控制器 JSON 只有几十 KB，
#: 16 MB 已经非常宽松，又能拦住“顺手选了一个几百 MB 的日志”把内存吃穿。
#: **超限在读到内存之前就拒绝**（见 :func:`read_limited`）。
UPLOAD_MAX_BYTES = 16 * 1024 * 1024

#: GP 上传模板的默认文件名（与 ``core.GP_TEMPLATE_NAME`` 同一个名字）。
TEMPLATE_NAME = core.GP_TEMPLATE_NAME
#: 「模板文件」块要显示摘要的那一份：现场那份未被写过的模板（只读引用）。
TEMPLATE_FILE_NAME = "position_R1_template.json"
#: 覆盖模板搜索路径的环境变量（现场备份在别处时用）。
TEMPLATE_ENV = "LASER_GP_TEMPLATE"

#: 运行表里要标出来的产物（顺序照 S5 规格：scan / joints / plot / repair），
#: 文件名一律问 ``core.RUN_ARTIFACTS`` 要，不手抄。
ARTIFACT_MARKS = (("scan", "scan.json"), ("joints", "joints.txt"),
                  ("plot", "plot.png"), ("repair", "repair.json"))

#: ISO 五点目录里没有 ``scan.json``，但 ``iso.json`` 与它同构（``core`` 的
#: ``_gp_scan_path`` 两种名字都认），所以 GP 导出的候选判断要多看这一个文件。
ISO_ARTIFACT = core.ARTIFACT_ISO_JSON

#: 运行下拉框为空时的提示（照抄网页版 ``_NO_RUN_HINT``）。
NO_RUN_HINT = "还没有运行记录：先在「生成」页跑一次，或点「刷新」"
#: GP 卡片里「只有带 scan.json 的才能写 GP」的提示。
GP_HINT = "只有带 scan.json（或同构的 iso.json）的运行才能写 GP"

_STATUS_STYLES = {"error": "color:#c0392b", "ok": "color:#1e8449",
                  "muted": "color:#6b7280", "info": ""}
_WARN = "color:#8a6d3b"
_GREEN = "color:#1e8449"
_RED = "color:#c0392b"
_RED_BOLD = "color:#c0392b; font-weight:bold"


# ---------------------------------------------------------------------------
# 纯逻辑（不碰 Qt，单测直接调）
# ---------------------------------------------------------------------------
def fmt_size(size_bytes) -> str:
    """字节数 -> 人类可读（表格里只求一眼看出量级）。"""
    value = float(size_bytes or 0)
    if value < 1024:
        return f"{value:.0f} B"
    value /= 1024
    if value < 1024:
        return f"{value:.1f} KB"
    return f"{value / 1024:.1f} MB"


def fmt_time(created) -> str:
    """运行目录的 mtime -> ``YYYY-mm-dd HH:MM:SS``（也就等于它的创建时间）。"""
    return datetime.fromtimestamp(float(created)).strftime("%Y-%m-%d %H:%M:%S")


def gp_source(row: dict) -> str | None:
    """一次运行能不能写 GP：返回产物名（``scan.json`` / ``iso.json``）或 ``None``。

    ``core._gp_scan_path`` 对运行目录先找 ``scan.json`` 再找 ``iso.json``（两者
    同构），这里用**同一个顺序**判断，界面与 core 不会各判一套。

    Args:
        row: :func:`laser_web.core.list_runs` 的一行。

    Returns:
        str | None：可用的产物文件名；两个都没有时 ``None``（该运行不能选）。
    """
    if row.get("has_scan"):
        return core.ARTIFACT_SCAN_JSON
    run_dir = row.get("dir")
    if run_dir and (Path(str(run_dir)) / ISO_ARTIFACT).is_file():
        return ISO_ARTIFACT
    return None


def run_label(row: dict) -> str:
    """下拉框里一项的文案：``run_id · 时间 · 有没有 scan.json``。"""
    source = gp_source(row)
    mark = (f"有 {source}" if source else "缺 scan.json")
    return f"{row['run_id']} · {fmt_time(row['created'])} · {mark}"


def artifact_marks(row: dict) -> str:
    """一行产物标记（✓ = 有，✗ = 没有）。"""
    marks = [f"{label}{'✓' if row.get(f'has_{key}') else '✗'}"
             for key, label in ARTIFACT_MARKS]
    marks.append(f"{ISO_ARTIFACT}{'✓' if _has_iso(row) else '✗'}")
    return " ".join(marks)


def _has_iso(row: dict) -> bool:
    """运行目录里有没有 ``iso.json``（``list_runs`` 不给这个标记）。"""
    run_dir = row.get("dir")
    return bool(run_dir) and (Path(str(run_dir)) / ISO_ARTIFACT).is_file()


def read_limited(path, limit: int = UPLOAD_MAX_BYTES):
    """读一个上传文件：**先判大小**，超限直接拒绝，不读进内存。

    Args:
        path: 文件路径。
        limit: 上限（字节），默认 :data:`UPLOAD_MAX_BYTES`。

    Returns:
        ``(bytes, None)`` 成功；``(None, 报错原文)`` 超限或读不了。超限时返回
        的原文里带实际字节数与上限，直接显示成红字即可。
    """
    file_path = Path(path)
    try:
        size = file_path.stat().st_size
    except OSError as error:
        return None, f"读不了文件 {file_path}：{error}"
    if int(size) > int(limit):
        return None, (f"文件 {fmt_size(size)}（{int(size)} 字节）超过 "
                      f"{fmt_size(limit)} 上限，未读取。")
    try:
        return file_path.read_bytes(), None
    except OSError as error:
        return None, f"读不了文件 {file_path}：{error}"


def file_digest(path) -> dict | None:
    """文件的 ``{path, size_bytes, sha256}``；不存在/读不了返回 ``None``。

    sha256 用 1 MB 分块算：模板与 URDF 都可能有几十 MB，不整块读进内存。
    """
    file_path = Path(str(path))
    if not file_path.is_file():
        return None
    digest = hashlib.sha256()
    size = 0
    try:
        with open(file_path, "rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
                size += len(chunk)
    except OSError:
        return None
    return {"path": str(file_path), "size_bytes": size,
            "sha256": digest.hexdigest()}


def template_candidates() -> list[Path]:
    """``position_R1_template.json`` 的搜索路径（按优先级）。

    现场模板不在仓库里（它是控制器的运行时文件），所以只认三处：
    ``LASER_GP_TEMPLATE`` 环境变量（可整体覆盖）、
    ``<可写数据目录>/position_R1_template.json``（放到 runs/ 旁边最省事）、
    ``data/configs/``。

    **没有**“再到 ``~/Documents/backup/`` 找一份”的兜底：GP 导出只吃用户显式
    选中的字节（:meth:`ToolsPage.load_template`），所以兜底不会导出错的 GP，
    但本块（只读摘要）会在命中时**毫无标识**地展示一份旧模板的 sha256——
    客户机上很容易被当成交付基线抄走。要指就别处那份，用环境变量。
    """
    candidates: list[Path] = []
    override = os.environ.get(TEMPLATE_ENV)
    if override:
        candidates.append(Path(override).expanduser())
    candidates += [paths.data_root() / TEMPLATE_FILE_NAME,
                   paths.config_dir() / TEMPLATE_FILE_NAME]
    return candidates


def find_template() -> Path | None:
    """按 :func:`template_candidates` 找第一个存在的模板文件。"""
    for candidate in template_candidates():
        if candidate.is_file():
            return candidate
    return None


def template_info(path=None) -> dict:
    """模板文件的只读信息（``{"path", "size_bytes", "sha256", "exists"}``）。"""
    found = Path(str(path)) if path is not None else find_template()
    if found is None or not found.is_file():
        return {"path": str(found) if found is not None else "",
                "size_bytes": None, "sha256": None, "exists": False}
    digest = file_digest(found) or {}
    return {"path": str(found), "size_bytes": digest.get("size_bytes"),
            "sha256": digest.get("sha256"), "exists": bool(digest)}


def urdf_summary(path) -> dict:
    """从生成的 URDF 里读出**显示用**的两件事：关节数、零位末端位姿。

    ``core.convert_urdf`` 只回路径 / 文件名 / 自校验误差（S5 规格额外要关节数
    与末端位姿），所以这里补两件事，都是纯显示、不参与校验：

    * 关节数：数 URDF 里的 ``<joint>`` 元素（含固定关节，就是 URDF 声明的全部）；
    * 末端位姿：用 ``handeye`` 的 ``RobotKinematics`` + ``DhOverride.from_base``
      做一次零位 FK（与 ``core._gp_cartesian_of`` 同一套模型装载方式），取末端
      位置（mm）与 ``xyz-intrinsic`` 姿态（deg）。

    Args:
        path: URDF 路径。

    Returns:
        dict: ``{"joints": int|None, "ee_position": [x,y,z]|None,
        "ee_rpy": [a,b,c]|None, "error": str|None}``；算不出来时字段为 ``None``，
        界面显示 ``—``（不谎报数字）。
    """
    summary = {"joints": None, "ee_position": None, "ee_rpy": None,
               "error": None}
    file_path = Path(str(path))
    if not file_path.is_file():
        summary["error"] = "URDF 不存在"
        return summary
    try:
        root = ElementTree.parse(str(file_path)).getroot()
        summary["joints"] = len(root.findall("joint"))
    except (ElementTree.ParseError, OSError) as error:
        summary["error"] = f"URDF 解析失败：{error}"
        return summary
    try:
        from handeye.dh import DhOverride, RobotKinematics
        from handeye.pose_encoder import matrix_to_pose_fields

        robot = RobotKinematics(file_path, DhOverride.from_base(file_path))
        position, orientation = matrix_to_pose_fields(
            robot.fk([0.0] * 6), convention="xyz-intrinsic")
        summary["ee_position"] = [float(value) for value in position]
        summary["ee_rpy"] = [float(value) for value in orientation]
    except Exception as error:                      # noqa: BLE001 - 只影响显示
        summary["error"] = f"{type(error).__name__}: {error}"
        _LOGGER.warning("URDF 末端位姿算不出来（%s）：%s", file_path, error)
    return summary


def value_or_dash(value) -> str:
    """``None`` / 空串 -> ``—``（结果区里所有缺值都走这一个口径）。"""
    if value is None or (isinstance(value, str) and not value.strip()):
        return "—"
    return str(value)


def pose_text(summary: dict) -> str:
    """末端位姿的一行显示（``XYZ: … / RPY: …``，缺值写字 ``—``）。"""
    summary = summary or {}
    position = summary.get("ee_position")
    rpy = summary.get("ee_rpy")
    if not position or not rpy:
        return "—"
    return ("位置 (" + ", ".join(f"{float(v):.1f}" for v in position) + ") mm"
            " · RPY (" + ", ".join(f"{float(v):.2f}" for v in rpy) + ") deg")


def left_row(*widgets) -> QtWidgets.QWidget:
    """一行控件**左对齐**、右边留白（按钮不拉开成整行宽）。

    ``QHBoxLayout`` 默认把子控件拉平；四个块里按钮多，拉平后每个都占半屏，
    现场看着像是四个巨大的“主按钮”。``_row`` 用的是同一种布局，这里只是补一
    个 ``addStretch``。
    """
    holder = QtWidgets.QWidget()
    layout = QtWidgets.QHBoxLayout(holder)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(6)
    for widget in widgets:
        layout.addWidget(widget)
    layout.addStretch(1)
    return holder


def save_copy(source, suggested_name: str, parent=None,
              filters: str = "所有文件 (*)") -> tuple[str | None, str]:
    """把 ``source`` 复制到用户选的位置（选完就复制，不移动）。

    Args:
        source: 源文件路径。
        suggested_name: 建议文件名（``QFileDialog`` 的默认值）。
        parent: 对话框父控件。
        filters: 文件过滤器。

    Returns:
        ``(level, message)``；``level`` 为 ``"error"`` 时界面要标红，
        用户取消返回 ``(None, "")``。
    """
    src = Path(str(source))
    if not src.is_file():
        return "error", f"源文件不存在：{src}"
    target, _selected = QtWidgets.QFileDialog.getSaveFileName(
        parent, f"保存 {src.name}", str(Path.home() / suggested_name), filters)
    if not target:                                  # 用户取消
        return None, ""
    target_path = Path(str(target))
    if not target_path.suffix:                      # 手打时删了扩展名
        target_path = target_path.with_suffix(src.suffix)
    try:
        shutil.copy2(src, target_path)
    except OSError as error:
        return "error", f"保存失败：{type(error).__name__}: {error}"
    return None, f"已保存 {src.name} → {target_path}"


# ---------------------------------------------------------------------------
# 页面
# ---------------------------------------------------------------------------
class ToolsPage(QtWidgets.QWidget):
    """工具页：四块纵向排列（GP 导出 / JSON→URDF / 运行目录 / 模板文件）。

    Attributes:
        runs: 最近一次 ``core.list_runs`` 的结果（按 created 倒序）。
        gp_state / urdf_state: 最近一次 ``core.build_gp`` / ``core.convert_urdf``
            的返回值（**含 ``ok=False``**：报错原文要能回看）。
        gp_export_usable: 最近一次 GP 导出的输出能不能用（模板 sha256 变了就是
            ``False``——此时结果区禁用下载，界面不许写成“已写入 N 个 GP”）。
        template_state: 本次 GP 导出用的模板（``{"name", "bytes", "path",
            "sha256"}``，内容只在内存里，模板文件本身只读）。
        controller_state: 本次转换用的控制器 JSON（同上）。
        last_clean: 最近一次清理的返回值（``{"removed", "kept"}``）。
        last_status: 最近一次写到状态栏的文字（无主窗口时也能断言）。
    """

    #: 状态栏里的页面前缀（四页共用一个状态栏）。
    STATUS_PREFIX = "工具"

    def __init__(self, parent=None, runs_root=None, active_dirs=None) -> None:
        """建页面。

        Args:
            parent: 父控件。
            runs_root: 运行目录根（``None`` 走 :func:`laser_desktop.paths.runs_root`，
                与其它页面的 ``JobBridge`` 同一个 ``runs/``）；测试注入临时目录。
            active_dirs: 可调用对象，返回“桌面版自己还在跑的任务目录”（通常是
                三个页面的 ``bridge.registry.active_dirs()`` 之并）。清理时与
                ``jobs.get_registry()`` 的活跃目录一起排除，见
                :meth:`clean_runs`。
        """
        super().__init__(parent)
        self.runs_root = (paths.runs_root() if runs_root is None
                          else Path(str(runs_root)))
        self._active_dirs = active_dirs
        self.runs: list[dict] = []
        self.gp_state: dict | None = None
        self.gp_export_usable = False
        self.urdf_state: dict | None = None
        self.template_state: dict | None = None
        self.controller_state: dict | None = None
        self.template_file: dict = {}
        self.last_clean: dict | None = None
        self.last_status = ""

        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setSpacing(10)
        layout.addWidget(self._build_gp_group())
        layout.addWidget(self._build_urdf_group())
        layout.addWidget(self._build_runs_group())
        layout.addWidget(self._build_template_group())
        layout.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        self.scroll = scroll                 # 冒烟脚本要把视图滚到运行表那一段
        outer.addWidget(scroll, 1)

        self.refresh_all()

    # ------------------------------------------------------------ 卡片 1：GP
    def _build_gp_group(self) -> QtWidgets.QGroupBox:
        """GP 导出：运行下拉 + 模板 + 五个参数 + 结果区（含模板 sha256 核对）。"""
        group = QtWidgets.QGroupBox("GP 导出")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)
        layout.addWidget(_caption(
            "把一次扫描的关节角写进 position_R1 模板的副本：14 字段布局"
            "（笛卡尔 6 + 标志 1 + 关节 6 + 外轴 1），与 "
            "excitation_trajectory.export_gp 完全同一套。"))

        self.run_combo = QtWidgets.QComboBox()
        self.run_combo.setMinimumWidth(420)
        self.run_combo.currentIndexChanged.connect(self._on_run_changed)
        layout.addWidget(_row(QtWidgets.QLabel("运行"), self.run_combo))
        self.run_hint = _caption(NO_RUN_HINT)
        layout.addWidget(self.run_hint)

        self.template_button = QtWidgets.QPushButton("选择 GP 模板（position_R1.json）")
        self.template_button.setToolTip(
            f"上限 {fmt_size(UPLOAD_MAX_BYTES)}；超限直接拒绝，不读进内存")
        self.template_button.clicked.connect(self.load_template)
        layout.addWidget(left_row(self.template_button))
        self.template_hint = _caption("尚未上传模板。")
        layout.addWidget(self.template_hint)

        defaults = core.gp_defaults()
        self.first_spin = _int_spin(0, 9999, defaults["first"], " 号")
        self.flag_spin = _int_spin(-32768, 32767, defaults["flag"])
        self.cartesian_combo = QtWidgets.QComboBox()
        for value in core.GP_CARTESIAN_MODES:
            self.cartesian_combo.addItem(value, value)
        self.cartesian_combo.setCurrentIndex(
            max(0, self.cartesian_combo.findData(defaults["cartesian"])))
        self.cartesian_combo.currentIndexChanged.connect(self._toggle_gp_euler)
        self.units_combo = QtWidgets.QComboBox()
        for value in core.GP_CART_UNITS:
            self.units_combo.addItem(value, value)
        self.units_combo.setCurrentIndex(
            max(0, self.units_combo.findData(defaults["cart_units"])))
        self.euler_combo = QtWidgets.QComboBox()
        for value in core.gp_euler_conventions():
            self.euler_combo.addItem(ui_state.euler_label(value), value)
        self.euler_combo.setCurrentIndex(
            max(0, self.euler_combo.findData(defaults["euler"])))
        layout.addWidget(_inline([
            ("起始 GP 号", self.first_spin), ("标志位", self.flag_spin),
            ("笛卡尔部分", self.cartesian_combo),
            ("笛卡尔姿态单位", self.units_combo),
            ("欧拉约定（内旋）", self.euler_combo)]))
        self._toggle_gp_euler()

        self.generate_gp_button = QtWidgets.QPushButton("生成 GP")
        self.generate_gp_button.clicked.connect(self.generate_gp)
        layout.addWidget(left_row(self.generate_gp_button))

        self.gp_count_label = QtWidgets.QLabel("尚未生成。")
        self.gp_count_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.gp_count_label)
        self.gp_meta_label = QtWidgets.QLabel("")
        self.gp_meta_label.setWordWrap(True)
        layout.addWidget(self.gp_meta_label)
        self.gp_template_label = QtWidgets.QLabel("")
        self.gp_template_label.setWordWrap(True)
        self.gp_template_label.setFont(monospace_font())
        layout.addWidget(self.gp_template_label)
        self.gp_error_label = QtWidgets.QLabel("")
        self.gp_error_label.setWordWrap(True)
        self.gp_error_label.hide()
        layout.addWidget(self.gp_error_label)
        self.gp_warnings_label = QtWidgets.QLabel("")
        self.gp_warnings_label.setWordWrap(True)
        self.gp_warnings_label.setStyleSheet(_WARN)
        self.gp_warnings_label.hide()
        layout.addWidget(self.gp_warnings_label)
        self.gp_path_label = QtWidgets.QLabel("—")
        self.gp_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.gp_path_label.setWordWrap(True)
        layout.addWidget(_row(QtWidgets.QLabel("GP 副本"),
                              self.gp_path_label))
        self.gp_download_button = QtWidgets.QPushButton("下载 GP 副本")
        self.gp_download_button.setEnabled(False)
        self.gp_download_button.clicked.connect(self._download_gp)
        layout.addWidget(left_row(self.gp_download_button))
        layout.addWidget(_caption(
            "模板只读：文件不会被改动，写的是运行目录下的副本。"))
        return group

    def _toggle_gp_euler(self, *_args) -> None:
        """只有 ``--cartesian fk`` 才用欧拉约定（CLI 也是这个语义）。"""
        fk = self.cartesian_combo.currentData() == "fk"
        self.euler_combo.setEnabled(fk)

    # ---------------------------------------------------------- 卡片 2：URDF
    def _build_urdf_group(self) -> QtWidgets.QGroupBox:
        """JSON → URDF：控制器 JSON + 型号 + 结果（路径 / 关节数 / 末端位姿）。"""
        group = QtWidgets.QGroupBox("JSON → URDF")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)
        layout.addWidget(_caption(
            "控制器配置（Robot_A.json）转标准 URDF：自校验用 200 个随机位姿比对 "
            "DH-FK 与 URDF-FK，误差 < 1e-9 才算通过。"))

        self.json_button = QtWidgets.QPushButton("选择控制器 JSON")
        self.json_button.setToolTip(
            f"上限 {fmt_size(UPLOAD_MAX_BYTES)}；超限直接拒绝，不读进内存")
        self.json_button.clicked.connect(self.load_controller)
        self.json_hint = _caption("尚未上传控制器 JSON。")
        layout.addWidget(left_row(self.json_button))
        layout.addWidget(self.json_hint)

        self.model_edit = QtWidgets.QLineEdit()
        self.model_edit.setPlaceholderText("型号 --model（可空，会成为文件名后缀）")
        self.convert_button = QtWidgets.QPushButton("转换")
        self.convert_button.clicked.connect(self.convert)
        layout.addWidget(_row(QtWidgets.QLabel("型号"), self.model_edit,
                              self.convert_button))

        self.urdf_name_label = QtWidgets.QLabel("尚未转换。")
        self.urdf_name_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.urdf_name_label)
        self.urdf_joints_label = QtWidgets.QLabel("关节数 —")
        layout.addWidget(self.urdf_joints_label)
        self.urdf_pose_label = QtWidgets.QLabel("末端位姿 —")
        self.urdf_pose_label.setWordWrap(True)
        self.urdf_pose_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.urdf_pose_label)
        self.urdf_error_label = QtWidgets.QLabel("")
        self.urdf_error_label.setWordWrap(True)
        self.urdf_error_label.hide()
        layout.addWidget(self.urdf_error_label)
        self.urdf_warnings_label = QtWidgets.QLabel("")
        self.urdf_warnings_label.setWordWrap(True)
        self.urdf_warnings_label.setStyleSheet(_WARN)
        self.urdf_warnings_label.hide()
        layout.addWidget(self.urdf_warnings_label)
        self.urdf_path_label = QtWidgets.QLabel("—")
        self.urdf_path_label.setWordWrap(True)
        self.urdf_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(_row(QtWidgets.QLabel("输出"), self.urdf_path_label))
        self.urdf_download_button = QtWidgets.QPushButton("下载 URDF")
        self.urdf_download_button.setEnabled(False)
        self.urdf_download_button.clicked.connect(self._download_urdf)
        layout.addWidget(left_row(self.urdf_download_button))
        return group

    # -------------------------------------------------------- 卡片 3：运行目录
    def _build_runs_group(self) -> QtWidgets.QGroupBox:
        """运行目录：表格 + 「刷新」「清理（保留最近 N）」。"""
        group = QtWidgets.QGroupBox("运行目录")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)
        layout.addWidget(_caption(
            f"runs/ 下的产物（每次运行一个目录），自动保留最近 "
            f"{jobs.KEEP_RUNS} 个。删除只碰 runs/ 下的子目录，正在跑的任务目录"
            f"不会动。"))

        self.runs_count_label = QtWidgets.QLabel("共 0 个运行目录")
        self.refresh_button = QtWidgets.QPushButton("刷新")
        self.refresh_button.clicked.connect(self.refresh_all)
        self.clean_button = QtWidgets.QPushButton(f"清理（保留最近 {jobs.KEEP_RUNS}）")
        self.clean_button.clicked.connect(self.clean_now)
        head = QtWidgets.QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        head.addWidget(self.runs_count_label)
        head.addStretch(1)
        head.addWidget(self.refresh_button)
        head.addWidget(self.clean_button)
        layout.addLayout(head)

        self.runs_table = make_table(("运行", "时间", "大小", "产物"))
        self.runs_table.setMinimumHeight(180)
        layout.addWidget(self.runs_table)
        self.clean_status_label = QtWidgets.QLabel("")
        self.clean_status_label.setWordWrap(True)
        layout.addWidget(self.clean_status_label)
        return group

    # ---------------------------------------------------------- 卡片 4：模板
    def _build_template_group(self) -> QtWidgets.QGroupBox:
        """模板文件：``position_R1_template.json`` 的 sha256 与字节数（只读）。"""
        group = QtWidgets.QGroupBox("模板文件")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(4)
        layout.addWidget(_caption(
            f"{TEMPLATE_FILE_NAME} 的摘要：本页只读它算 sha256，不写这个文件。"
            f"找不到时用环境变量 {TEMPLATE_ENV} 指过去。"))
        self.template_path_label = QtWidgets.QLabel("—")
        self.template_path_label.setWordWrap(True)
        self.template_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.template_size_label = QtWidgets.QLabel("字节数 —")
        self.template_sha_label = QtWidgets.QLabel("sha256 —")
        self.template_sha_label.setFont(monospace_font())
        self.template_sha_label.setWordWrap(True)
        self.template_sha_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.template_reload_button = QtWidgets.QPushButton("重新读取")
        self.template_reload_button.clicked.connect(self.refresh_template_block)
        layout.addWidget(_row(QtWidgets.QLabel("路径"),
                              self.template_path_label))
        layout.addWidget(_row(self.template_size_label, self.template_sha_label))
        layout.addWidget(left_row(self.template_reload_button))
        return group

    # ------------------------------------------------------------------ 刷新
    def refresh_all(self) -> None:
        """重建运行下拉框、运行表与模板摘要（新跑一次的目录要能立刻出现）。"""
        self.refresh_runs()
        self.refresh_template_block()

    def refresh_runs(self) -> list[dict]:
        """重建运行下拉框与运行表；返回 ``core.list_runs`` 的结果。

        没有 ``scan.json`` / ``iso.json`` 的运行会**灰掉、不能选**（``QStandardItem``
        的 ``enabled=False``），但仍在列表里——现场要能看见“这一跑没有产物”。
        """
        rows = core.list_runs(self.runs_root)
        self.runs = rows
        previous = self.run_combo.currentData()
        self.run_combo.blockSignals(True)
        self.run_combo.clear()
        for row in rows:
            source = gp_source(row)
            self.run_combo.addItem(
                ("✓ " if source else "✗ ") + run_label(row), row["run_id"])
            index = self.run_combo.count() - 1
            if source is None:
                item = self._combo_item(index)
                if item is not None:
                    item.setEnabled(False)
                    item.setToolTip(f"该运行没有 {core.ARTIFACT_SCAN_JSON}，"
                                    f"不能写 GP")
        self.run_combo.blockSignals(False)
        wanted = previous if any(row["run_id"] == previous for row in rows) \
            else next((row["run_id"] for row in rows if gp_source(row)), None)
        if wanted is not None:
            self.run_combo.setCurrentIndex(self.run_combo.findData(wanted))
        self.run_hint.setText(f"可导出 {len(rows)} 次运行；{GP_HINT}"
                              if rows else NO_RUN_HINT)
        self._fill_runs_table(rows)
        self._on_run_changed()          # 没运行时按钮不能亮着
        return rows

    def _combo_item(self, index: int):
        """取 ``QComboBox`` 某一项（默认模型是 ``QStandardItemModel``）。"""
        model = self.run_combo.model()
        getter = getattr(model, "item", None)
        return getter(index) if callable(getter) else None

    def _fill_runs_table(self, rows) -> None:
        """运行表：运行号 / 时间 / 大小 / 产物标记（大小按数值排序）。"""
        self.runs_table.setSortingEnabled(False)
        self.runs_table.clearContents()
        self.runs_table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            cells = [
                ((0, row["run_id"]), row["run_id"], "left"),
                ((0, fmt_time(row["created"])), fmt_time(row["created"]), "left"),
                (float(row["size_bytes"]), fmt_size(row["size_bytes"]), "right"),
                ((0, artifact_marks(row)), artifact_marks(row), "left"),
            ]
            for column, (key, text, align) in enumerate(cells):
                item = SortItem(key, text)
                item.setTextAlignment(
                    {"right": Qt.AlignRight | Qt.AlignVCenter,
                     "center": Qt.AlignCenter,
                     "left": Qt.AlignLeft | Qt.AlignVCenter}[align])
                self.runs_table.setItem(row_index, column, item)
        self.runs_table.setSortingEnabled(True)
        self.runs_table.resizeColumnsToContents()
        self.runs_count_label.setText(f"共 {len(rows)} 个运行目录")

    def refresh_template_block(self) -> dict:
        """重新算模板文件的只读摘要（路径 / 字节数 / sha256）。"""
        info = template_info()
        self.template_file = info
        if not info["exists"]:
            self.template_path_label.setText(
                "—（找不到 " + "；".join(str(path) for path in template_candidates())
                + f"；可用环境变量 {TEMPLATE_ENV} 指定）")
            self.template_size_label.setText("字节数 —")
            self.template_sha_label.setText("sha256 —")
            self.template_sha_label.setStyleSheet(_WARN)
            return info
        self.template_path_label.setText(info["path"])
        self.template_size_label.setText(f"字节数 {info['size_bytes']}")
        self.template_sha_label.setText(f"sha256 {info['sha256']}")
        self.template_sha_label.setStyleSheet("")
        return info

    # ------------------------------------------------------------ GP 导出
    def current_run(self) -> dict | None:
        """当前选中的运行行（没有运行/没选时 ``None``）。"""
        run_id = self.run_combo.currentData()
        return next((row for row in self.runs if row["run_id"] == run_id), None)

    def _on_run_changed(self, *_args) -> None:
        """选运行：把该运行能不能写 GP 说清楚（不能的按钮直接禁用）。"""
        row = self.current_run()
        source = gp_source(row) if row else None
        if row is None:
            self.run_hint.setText(NO_RUN_HINT)
        elif source is None:
            self.run_hint.setText(
                f"该运行没有 {core.ARTIFACT_SCAN_JSON}，不能写 GP")
            self.run_hint.setStyleSheet(_RED)
        else:
            self.run_hint.setText(f"{row['run_id']} 用 {source} 写 GP")
            self.run_hint.setStyleSheet(_GREEN)
        self._sync_generate_button()

    def _sync_generate_button(self) -> None:
        """「生成 GP」的可用性：只跟“选中的运行有没有可读产物”有关。

        模板另管：没上传时 ``generate_gp`` 自己拦（红字「先选 GP 模板」），
        上传被拒时 :meth:`_reject_template` 会额外把它禁掉。
        """
        row = self.current_run()
        source = gp_source(row) if row is not None else None
        self.generate_gp_button.setEnabled(row is not None and source is not None)

    def load_template(self):
        """选模板文件：**先判大小**，超限红字拒绝、不读进内存。

        Returns:
            dict | None：``{"name", "bytes", "path", "sha256"}``；取消/超限时
            ``None``（超限时状态写进 ``template_hint``，红字）。
        """
        selected, _filters = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择 GP 模板（position_R1.json）", str(Path.home()),
            "JSON (*.json);;所有文件 (*)")
        if not selected:
            return None
        data, error = read_limited(selected)
        if error is not None:
            self._reject_template(error)
            return None
        state = {"name": Path(selected).name, "bytes": data,
                 "path": str(selected),
                 "sha256": hashlib.sha256(data).hexdigest()}
        self.set_template(state)
        return state

    def _reject_template(self, error: str) -> None:
        """超限/读不了：**先清掉上一次的模板**，再红字两行写进提示。

        只写红字是不够的：``template_state`` 会留着上一次成功上传的内容，而
        「生成 GP」仍然可用——一点就用**上一个**模板静默导出。所以这里同时做
        三件事：清状态（恢复“尚未上传”）、把“当前装的是哪个模板”重新写回
        第一行、禁用「生成 GP」。
        """
        self.set_template(None)                      # 恢复「尚未上传模板。」
        self.template_hint.setText(
            "当前模板：尚未上传（上一次上传的模板已清空）\n" + str(error))
        self.template_hint.setStyleSheet(_RED_BOLD)
        self.generate_gp_button.setEnabled(False)

    def set_template(self, state: dict | None) -> None:
        """设置本次 GP 导出用的模板（测试/上传共用；内容只在内存里）。

        Args:
            state: ``{"name", "bytes", "path", "sha256"}`` 或 ``None``（清空）。
        """
        self.template_state = state
        if state is None:
            self.template_hint.setText("尚未上传模板。")
            self.template_hint.setStyleSheet("")
            return
        self.template_hint.setText(
            f"模板：{state['name']}（{len(state['bytes'])} 字节，只读引用，"
            f"sha256 {state.get('sha256') or '—'}）")
        self.template_hint.setStyleSheet(_GREEN)
        self._sync_generate_button()

    def generate_gp(self) -> dict | None:
        """「生成 GP」：调 ``core.build_gp``，并把模板 sha256 前后对比显示出来。

        Returns:
            dict | None: ``core.build_gp`` 的返回值；缺运行/缺模板时 ``None``。
        """
        row = self.current_run()
        if row is None or gp_source(row) is None:
            self._set_gp_error(NO_RUN_HINT, level="error")
            return None
        template = self.template_state
        if template is None:
            self._set_gp_error("先选 GP 模板（position_R1.json）", level="error")
            return None
        before = self._template_digests()
        params = {"run_dir": row["dir"],
                  "template": template["bytes"],
                  "template_name": template["name"],
                  "first": int(self.first_spin.value()),
                  "flag": int(self.flag_spin.value()),
                  "cartesian": self.cartesian_combo.currentData(),
                  "cart_units": self.units_combo.currentData(),
                  "euler": self.euler_combo.currentData()}
        try:
            result = core.build_gp(params)
        except Exception as error:              # noqa: BLE001 - core 不抛异常
            _LOGGER.exception("core.build_gp 抛异常（params=%r）", params)
            result = {"ok": False, "error": f"{type(error).__name__}: {error}",
                      "warnings": []}
        after = self._template_digests()
        self.gp_state = result
        self.render_gp(result, before, after)
        return result

    def _template_digests(self) -> dict:
        """当前模板的两份指纹：内存里的字节 + （若来自文件）磁盘上的那份。"""
        state = self.template_state or {}
        data = state.get("bytes") or b""
        digests = {"bytes": hashlib.sha256(data).hexdigest() if data else None,
                   "file": None}
        if state.get("path"):
            info = file_digest(state["path"]) or {}
            digests["file"] = info.get("sha256")
        return digests

    def _set_gp_error(self, message: str, level: str = "error") -> None:
        """GP 结果区的报错（红字，不弹窗）。"""
        self.gp_error_label.setText(str(message))
        self.gp_error_label.setStyleSheet(_STATUS_STYLES.get(level, ""))
        self.gp_error_label.setVisible(bool(str(message).strip()))

    def render_gp(self, result: dict, before: dict, after: dict) -> None:
        """铺 GP 导出结果：写了几个、模板 sha256、副本路径、下载按钮。

        Args:
            result: ``core.build_gp`` 的返回值。
            before / after: :meth:`_template_digests` 的前后指纹。
        """
        result = result or {}
        self.gp_warnings_label.hide()
        self.gp_download_button.setEnabled(False)
        if not result.get("ok"):
            self.gp_export_usable = False
            self._set_gp_error(f"失败：{result.get('error') or '未知错误'}")
            self.gp_count_label.setText("尚未生成。")
            self.gp_count_label.setStyleSheet("")
            self.gp_meta_label.setText("")
            self.gp_template_label.setText("")
            self.gp_path_label.setText("—")
            warnings = [str(line) for line in (result.get("warnings") or [])]
            self.gp_warnings_label.setText("\n".join(f"· {line}"
                                                     for line in warnings))
            self.gp_warnings_label.setVisible(bool(warnings))
            return
        self.gp_error_label.hide()
        unchanged = before.get("bytes") == after.get("bytes")
        if before.get("file") is not None or after.get("file") is not None:
            unchanged = unchanged and before.get("file") == after.get("file")
        meta = result.get("meta") or {}
        self.gp_path_label.setText(str(result.get("path") or "—"))
        warnings = [str(line) for line in (result.get("warnings") or [])]
        self.gp_warnings_label.setText("\n".join(f"· {line}" for line in warnings))
        self.gp_warnings_label.setVisible(bool(warnings))
        if not unchanged:
            # 模板必须不变：变了就是**硬失败**——只画红字不够，那时“已写入 N 个
            # GP”还是绿的、下载按钮还是亮的，现场会直接拿走一份不可用的输出。
            # 两个指纹都摆出来（不许只说“有变化”）。
            self.gp_export_usable = False
            self.gp_count_label.setText(
                "已作废：模板 sha256 变了，本次输出不能用于现场。")
            self.gp_count_label.setStyleSheet(_RED_BOLD)
            self.gp_meta_label.setText("")
            self.gp_meta_label.setStyleSheet("")
            self.gp_template_label.setText(
                "⚠ 模板 sha256 变了（模板必须保持不变）！\n"
                f"  导出前：{before.get('bytes')}\n"
                f"  导出后：{after.get('bytes')}"
                + ("" if before.get("file") == after.get("file")
                   else f"\n  磁盘前：{before.get('file')}\n"
                        f"  磁盘后：{after.get('file')}"))
            self.gp_template_label.setStyleSheet(_RED_BOLD)
            self._set_gp_error(
                "模板 sha256 变了（模板必须保持不变）：本次输出已作废，"
                "不要下载、不要写进控制器。")
            self.gp_download_button.setEnabled(False)
            self.refresh_runs()
            return
        self.gp_export_usable = True
        self.gp_count_label.setText(
            f"已写入 {result['count']} 个 GP："
            f"GP{int(result['first']):04d} .. GP{int(result['last']):04d}")
        self.gp_count_label.setStyleSheet("")
        # 缺值一律写「—」（模板里本来就没有这一项时不许显示 None）
        self.gp_meta_label.setText(
            f"模板 controllerID={value_or_dash(meta.get('controllerID'))} · "
            f"version={value_or_dash(meta.get('version'))} —— 已回读核对，未改动")
        self.gp_meta_label.setStyleSheet(_GREEN)
        self.gp_template_label.setText(
            f"模板 sha256 未变：{before.get('bytes')}"
            + ("" if before.get("file") is None
               else f"\n磁盘文件 sha256 未变：{before.get('file')}"))
        self.gp_template_label.setStyleSheet(_GREEN)
        self.gp_download_button.setEnabled(
            bool(result.get("path")) and Path(str(result["path"])).is_file())
        self.refresh_runs()             # 输出文件落在运行目录里：标记要立刻更新

    def _download_gp(self) -> None:
        """下载 GP 副本（复制到用户选的位置）。"""
        result = self.gp_state or {}
        if not result.get("ok") or not self.gp_export_usable:
            return
        level, message = save_copy(result["path"], result["filename"], self,
                                   "JSON (*.json);;所有文件 (*)")
        if not message:
            return
        self._set_gp_error(message, level=level or "info")
        self._report_status(message)

    # ---------------------------------------------------------- JSON → URDF
    def load_controller(self):
        """选控制器 JSON：先判大小，超限红字拒绝、不读进内存。"""
        selected, _filters = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择控制器 JSON", str(paths.config_dir()),
            "JSON (*.json);;所有文件 (*)")
        if not selected:
            return None
        data, error = read_limited(selected)
        if error is not None:
            self._reject_controller(error)
            return None
        state = {"name": Path(selected).name, "bytes": data,
                 "path": str(selected)}
        self.set_controller(state)
        return state

    def _reject_controller(self, error: str) -> None:
        """超限/读不了：清掉上一次的控制器 JSON 并禁用「转换」（理由同模板）。"""
        self.set_controller(None)                    # 恢复「尚未上传控制器 JSON。」
        self.json_hint.setText(
            "当前控制器 JSON：尚未上传（上一次上传的内容已清空）\n"
            + str(error))
        self.json_hint.setStyleSheet(_RED_BOLD)
        self.convert_button.setEnabled(False)

    def set_controller(self, state: dict | None) -> None:
        """设置本次转换用的控制器 JSON（测试/上传共用）。"""
        self.controller_state = state
        if state is None:
            self.json_hint.setText("尚未上传控制器 JSON。")
            self.json_hint.setStyleSheet("")
            return
        self.json_hint.setText(
            f"文件：{state['name']}（{len(state['bytes'])} 字节）")
        self.json_hint.setStyleSheet(_GREEN)
        self.convert_button.setEnabled(True)

    def convert(self) -> dict | None:
        """「转换」：调 ``core.convert_urdf`` 并铺结果。

        Returns:
            dict | None: ``core.convert_urdf`` 的返回值；没上传时 ``None``。
        """
        payload = self.controller_state
        if payload is None:
            self._set_urdf_error("先上传控制器 JSON")
            return None
        try:
            result = core.convert_urdf(payload["bytes"],
                                       self.model_edit.text().strip() or None,
                                       filename=payload["name"])
        except Exception as error:              # noqa: BLE001 - core 不抛异常
            _LOGGER.exception("core.convert_urdf 抛异常")
            result = {"ok": False, "error": f"{type(error).__name__}: {error}",
                      "warnings": []}
        self.urdf_state = result
        self.render_urdf(result)
        return result

    def _set_urdf_error(self, message: str) -> None:
        """URDF 结果区的报错（红字，不弹窗）。"""
        self.urdf_error_label.setText(str(message))
        self.urdf_error_label.setStyleSheet(_STATUS_STYLES["error"])
        self.urdf_error_label.show()

    def render_urdf(self, result: dict) -> None:
        """铺 JSON → URDF 的结果：文件名 / 关节数 / 末端位姿 / 路径 / 下载。"""
        result = result or {}
        self.urdf_warnings_label.hide()
        self.urdf_download_button.setEnabled(False)
        if not result.get("ok"):
            self._set_urdf_error(f"失败：{result.get('error') or '未知错误'}")
            self.urdf_name_label.setText("尚未转换。")
            self.urdf_joints_label.setText("关节数 —")
            self.urdf_pose_label.setText("末端位姿 —")
            self.urdf_path_label.setText("—")
            warnings = [str(line) for line in (result.get("warnings") or [])]
            self.urdf_warnings_label.setText(
                "\n".join(f"· {line}" for line in warnings))
            self.urdf_warnings_label.setVisible(bool(warnings))
            return
        self.urdf_error_label.hide()
        self.urdf_name_label.setText(f"已生成 {result['filename']}")
        error = result.get("self_check_error")
        if error is None:
            self.urdf_name_label.setStyleSheet(_WARN)
            self.urdf_name_label.setText(
                f"已生成 {result['filename']}（没能从输出里解析出自校验误差）")
        else:
            self.urdf_name_label.setStyleSheet(_GREEN)
            self.urdf_name_label.setText(
                f"已生成 {result['filename']} · 自校验误差 {error:.3e} mm/rad")
        summary = urdf_summary(result["path"])
        self.urdf_joints_label.setText(
            "关节数 " + ("—" if summary.get("joints") is None
                          else str(int(summary["joints"]))))
        self.urdf_pose_label.setText("末端位姿 " + pose_text(summary))
        if summary.get("error"):
            self.urdf_pose_label.setToolTip(str(summary["error"]))
        self.urdf_path_label.setText(str(result.get("path") or "—"))
        self.urdf_download_button.setEnabled(
            bool(result.get("path")) and Path(str(result["path"])).is_file())
        warnings = [str(line) for line in (result.get("warnings") or [])]
        self.urdf_warnings_label.setText(
            "\n".join(f"· {line}" for line in warnings))
        self.urdf_warnings_label.setVisible(bool(warnings))

    def _download_urdf(self) -> None:
        """下载生成的 URDF（复制到用户选的位置）。"""
        result = self.urdf_state or {}
        if not result.get("ok"):
            return
        level, message = save_copy(result["path"], result["filename"], self,
                                   "URDF (*.urdf);;所有文件 (*)")
        if not message:
            return
        self._set_urdf_error(message)
        self._report_status(message)

    # ------------------------------------------------------------- 清理
    def active_run_dirs(self) -> set[str]:
        """还在跑的任务目录：``jobs.get_registry()`` + 桌面版各页面的登记表。"""
        active = set(jobs.get_registry().active_dirs())
        provider = self._active_dirs
        if provider is not None:
            try:
                active |= {str(Path(item).resolve()) for item in provider() or ()}
            except Exception:               # noqa: BLE001 - 清理不该被上游拖垮
                _LOGGER.exception("取活跃任务目录失败（清理只按 core 的口径）")
        return active

    def clean_runs(self, keep: int | None = None) -> dict:
        """按保留策略清理运行目录，返回 ``{"removed": [...], "kept": n}``。

        ``core.clean_runs`` 的 ``exclude`` 只来自 ``jobs.get_registry()``（网页版
        的进程级登记表），桌面版每个页面自带一个 ``JobRegistry``，所以桌面版
        正在跑的任务目录它看不见——直接调会把它们删掉。这里：

        * 没有额外活跃目录 → 直接调 ``core.clean_runs``（与网页版同一条路径）；
        * 有额外活跃目录   → 调 ``core.clean_runs`` **内部用的同一个**
          ``jobs.prune_runs``，把 ``exclude`` 一起传进去（保留策略、排序键、
          ``removed``/``kept`` 口径完全一致）。

        Args:
            keep: 保留最近多少个（``None`` 走 ``jobs.KEEP_RUNS`` = 50）。

        Returns:
            dict: ``{"removed": [被删目录的绝对路径, ...], "kept": 剩余目录数}``。
        """
        keep = jobs.KEEP_RUNS if keep is None else max(0, int(keep))
        base = jobs.runs_root(self.runs_root)
        extra = self.active_run_dirs() - jobs.get_registry().active_dirs()
        if not extra:
            return core.clean_runs(keep, root=base)
        removed = [str(path) for path in
                   jobs.prune_runs(base, keep,
                                   exclude=jobs.get_registry().active_dirs()
                                   | extra)]
        return {"removed": removed, "kept": len(core.list_runs(base))}

    def clean_now(self, keep: int | None = None) -> dict:
        """「清理（保留最近 N）」：清理并刷新表格，回报删了几个。"""
        result = self.clean_runs(keep)
        removed = len(result["removed"])
        self.last_clean = result
        self.clean_status_label.setText(
            f"已删除 {removed} 个运行目录，保留最近 {result['kept']} 个")
        self.clean_status_label.setStyleSheet(_GREEN if removed else _STATUS_STYLES["muted"])
        self._report_status(f"已删除 {removed} 个运行目录")
        self.refresh_all()
        return result

    # ------------------------------------------------------------------ 杂项
    def _report_status(self, message: str) -> None:
        """把一句话写到窗口状态栏（没有主窗口的纯逻辑测试里也能断言）。"""
        self.last_status = str(message)
        text = self.status_message()
        if not text:
            return
        status_bar = getattr(self.window(), "statusBar", None)
        if callable(status_bar):
            status_bar().showMessage(text, 10_000)

    def status_message(self) -> str:
        """状态栏里该显示的本页最后一句（带页面前缀）；没说过话时空串。"""
        text = str(getattr(self, "last_status", "") or "").strip()
        if not text:
            return ""
        return f"{self.STATUS_PREFIX}：{text}"
