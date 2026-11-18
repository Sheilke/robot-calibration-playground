# -*- coding: utf-8 -*-
"""表单构件：机器人模型 / 采样域 / 网格 / 参考姿态 / 扰动 / 守卫。

为什么有这一层
--------------
生成页（S3）与修复页（S4）的这六组字段**同源**：默认值来自
:func:`laser_desktop.generate_page.form_defaults`（它再往下问
``laser_web.ui_state`` 与两个 CLI 的解析器），控件小工具（``_double_spin`` /
``_int_spin`` / ``_row`` / ``_inline`` / ``_caption``）与上限常量
（:data:`laser_desktop.generate_page.TARGET_COUNT_MAX` /
:data:`laser_desktop.generate_page.POINT_COUNT_WARN`）也都在生成页里。

所以**两页共用这一份**：本模块是“这六组表单怎么摆”的唯一实现。S4 评审第 1
条指的就是之前的样子——两页各存一份副本、文件头写着“不复制”，实际上改一处
忘了另一处就会悄悄分叉（``maximum = 20 if key == "z_insert" else 200`` 当时
就是两份）。现在生成页的 ``_build_form`` 也只调这里。

约定：

* ``fields`` 的键名两页**完全一致**（``min_x`` / ``x_min`` / ``rpy_a`` /
  ``j5_min`` / ...），因此修复页可以直接复用
  :func:`laser_desktop.generate_page.total_points` 的口径；
* 每个 ``build_*_group`` 只负责建控件并写进调用方给的 ``fields`` dict，取值、
  校验、参数组装仍由页面与 :mod:`laser_web.core` 负责（本模块不碰 core）；
* 输入框的上下限、默认值与说明文字只在这里写一次；
* 两页的差异用**参数**表达、不用复制：修复页不给「段模型」与「重采上限」
  （见 :func:`build_guard_group`），生成页把网格组的说明小字挂在
  :func:`build_grid_group` 返回值的 ``caption`` 上。

# SYNC: 与别处必须一致的东西（S6 L3 的自动化检查在
# ``tests/test_s6_common.py::FormConsistencyTest``，改下面任何一项都会红）

* ``GRID_VERTICES_MAX`` / ``Z_INSERT_MAX`` —— 手填格点数 / 换层插点的输入上限；
  实际控件由 :func:`build_grid_group` 建，测试比对控件的 ``maximum()``；
* ``target_count`` 的上限 = :data:`laser_desktop.generate_page.TARGET_COUNT_MAX`
  （由 ``scan.COUNT_GRID_MAX_PER_AXIS`` / ``COUNT_GRID_MAX_INSERT`` 算出，不手抄）；
* 所有默认值 = :func:`laser_desktop.generate_page.form_defaults`
  （它再问 ``ui_state`` 与两个 CLI 的解析器）；修复页的 ``seg_samples`` 例外——
  它跟随修复 CLI 的 11，扫描 CLI 是 21，两边都写在自己的 ``defaults()`` 里；
* 生成页与修复页的六组字段：同名、同默认值、同上下限（测试逐项比）。
"""

from __future__ import annotations

from excitation_trajectory import aabb_snake_scan as scan
from laser_web import ui_state

from .generate_page import (
    DEFAULT_SEG_MODE,
    DOMAIN_MODES,
    GRID_MODES,
    POINT_COUNT_WARN,
    SEG_MODES,
    TARGET_COUNT_MAX,
    UNIT_DEG,
    _caption,
    _double_spin,
    _inline,
    _int_spin,
    _row,
)
from .qt_compat import QtWidgets

__all__ = ["build_robot_group", "build_domain_group", "build_grid_group",
           "build_pose_group", "build_perturb_group", "build_guard_group",
           "clear_layout", "GRID_VERTICES_MAX", "Z_INSERT_MAX"]

#: 手填格点数（X/Y/Z）输入框的上限：桌面版自己夹的，网页版只给了 ``min=0``。
#: 真正的硬约束在 ``_validate_args`` 的份数上限里，这里只是别让输入框能转到
#: 天文数字（转到几百万点数会直接吃穿内存）。
GRID_VERTICES_MAX = 200

#: 换层插点的输入上限（同上；一次插这么多点已经远超 CLI 的
#: ``COUNT_GRID_MAX_INSERT``，只是输入框的护栏）。
Z_INSERT_MAX = 20


def _compact_group(group):
    """分组高度 = 内容高度（``sizeHint``）：不允许被外层布局拉伸。

    不设的话，表单内容比窗口矮时 ``QVBoxLayout`` 会把多出来的高度均摊给各组
    （``Preferred`` 可以长），组里的 ``QFormLayout`` 再把这点高度摊到行距上：
    现场看到的就是"模型 / 路径 / 提示"之间的大段空白（S6b 截图）。

    Args:
        group: ``QGroupBox``。

    Returns:
        同一个分组。
    """
    group.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                        QtWidgets.QSizePolicy.Maximum)
    return group


def clear_layout(layout) -> None:
    """清空一个布局（含嵌套布局），子控件一律 ``deleteLater``。

    结果区要反复重画（判定徽标一行几个是运行时才知道的），所以每次先清再建；
    只 ``takeAt`` 不删控件会让旧的徽标留在父控件上继续画。

    Args:
        layout: ``QLayout``。
    """
    while layout.count():
        item = layout.takeAt(0)
        widget = item.widget()
        if widget is not None:
            widget.setParent(None)
            widget.deleteLater()
            continue
        child = item.layout()
        if child is not None:
            clear_layout(child)
            child.setParent(None)
            child.deleteLater()


def build_robot_group(fields, robots, on_change, default_path=None):
    """机器人模型：只列 ``Nexbot6R`` 真能加载的，其余折叠成一句提示。

    Args:
        fields: ``{键: 控件}``，会写入 ``"robot"``。
        robots: :func:`laser_web.core.list_robots` 的返回值。
        on_change: 任何输入变化都调它（``*args`` 兼容信号签名）。
        default_path: 默认选中的模型路径；``None`` 用 ``ui_state.default_robot()``。

    Returns:
        dict: ``{"group", "path_label", "skipped_label"}``（页面若要在别处读
        模型路径 / 跳过的文件提示，从这里取控件，不另建一份）。
    """
    group = QtWidgets.QGroupBox("机器人模型")
    form = QtWidgets.QFormLayout(group)
    usable = [robot for robot in robots if robot["usable"]]
    skipped = [robot for robot in robots if not robot["usable"]]

    combo = QtWidgets.QComboBox()
    for robot in usable:
        combo.addItem(robot["label"], robot["path"])
    if combo.count() == 0:                    # 一个都用不了时别空着
        combo.addItem(ui_state.default_robot(), ui_state.default_robot())
    wanted = default_path or ui_state.default_robot()
    index = combo.findData(wanted)
    combo.setCurrentIndex(index if index >= 0 else 0)
    combo.currentIndexChanged.connect(on_change)
    fields["robot"] = combo
    form.addRow("模型", combo)

    path_label = _caption(f"路径：{combo.currentData()}")
    combo.currentIndexChanged.connect(
        lambda *_args: path_label.setText(f"路径：{combo.currentData()}"))
    form.addRow(path_label)

    if skipped:
        detail = "；".join(f"{robot['label']}（{robot['reason']}）"
                          for robot in skipped)
        note = _caption(f"另有 {len(skipped)} 个文件不能当模型用")
        note.setToolTip(detail)
    else:
        note = _caption("data/configs 下的文件全部可以当模型用")
    form.addRow(note)
    # 三行之间不留空行：说明文字自己只有一行高（HintLabel 按文字实际行数算），
    # 分组也不许被拉伸——否则截图里就是"排版坏了"（见 _compact_group）
    form.setVerticalSpacing(4)
    return {"group": _compact_group(group), "path_label": path_label,
            "skipped_label": note}


def build_domain_group(fields, defaults, on_change):
    """采样域：两点对角点 / 六个轴边界（互斥，二选一）。

    Args:
        fields: ``{键: 控件}``，会写入 ``domain_points`` / ``domain_axes`` /
            ``min_x``~``max_z`` / ``x_min``~``z_max``。
        defaults: 表单默认值（键见 :func:`laser_desktop.generate_page.form_defaults`）。
        on_change: 输入变化回调。

    Returns:
        dict: ``{"group", "stack", "points", "axes"}``（页面据此切面板）。
    """
    group = QtWidgets.QGroupBox("采样域")
    form = QtWidgets.QFormLayout(group)

    points = QtWidgets.QRadioButton("两点输入")
    axes = QtWidgets.QRadioButton("六个轴边界")
    buttons = QtWidgets.QButtonGroup(group)
    buttons.addButton(points, 0)
    buttons.addButton(axes, 1)
    fields["domain_points"] = points
    fields["domain_axes"] = axes
    form.addRow(_row(points, axes))

    stack = QtWidgets.QStackedWidget()
    points_panel = QtWidgets.QWidget()
    points_layout = QtWidgets.QVBoxLayout(points_panel)
    points_layout.setContentsMargins(0, 0, 0, 0)
    points_layout.setSpacing(4)
    lows, highs = [], []
    for axis in range(3):
        name = "XYZ"[axis]
        low = _double_spin(-10000.0, 10000.0, defaults["point_min"][axis])
        high = _double_spin(-10000.0, 10000.0, defaults["point_max"][axis])
        low.valueChanged.connect(on_change)
        high.valueChanged.connect(on_change)
        fields[f"min_{name.lower()}"] = low
        fields[f"max_{name.lower()}"] = high
        lows.append((f"{name} 最小", low))
        highs.append((f"{name} 最大", high))
    points_layout.addWidget(_inline(lows))
    points_layout.addWidget(_inline(highs))
    stack.addWidget(points_panel)

    axes_panel = QtWidgets.QWidget()
    axes_layout = QtWidgets.QVBoxLayout(axes_panel)
    axes_layout.setContentsMargins(0, 0, 0, 0)
    axes_layout.setSpacing(4)
    lows, highs = [], []
    for axis in range(3):
        name = "XYZ"[axis]
        low = _double_spin(-10000.0, 10000.0, defaults["aabb6"][2 * axis])
        high = _double_spin(-10000.0, 10000.0, defaults["aabb6"][2 * axis + 1])
        low.valueChanged.connect(on_change)
        high.valueChanged.connect(on_change)
        fields[f"{name.lower()}_min"] = low
        fields[f"{name.lower()}_max"] = high
        lows.append((f"{name} 下界", low))
        highs.append((f"{name} 上界", high))
    axes_layout.addWidget(_inline(lows))
    axes_layout.addWidget(_inline(highs))
    stack.addWidget(axes_panel)

    form.addRow(stack)
    points.setChecked(defaults["domain_mode"] == DOMAIN_MODES[0])
    axes.setChecked(defaults["domain_mode"] == DOMAIN_MODES[1])
    stack.setCurrentIndex(0 if points.isChecked() else 1)
    return {"group": _compact_group(group), "stack": stack, "points": points,
            "axes": axes}


def build_grid_group(fields, defaults, on_change):
    """网格：目标点数 / 手填格点数；总点数就地显示。

    Args:
        fields: ``{键: 控件}``，会写入 ``grid_count`` / ``grid_explicit`` /
            ``target_count`` / ``xs`` / ``ys`` / ``zs`` / ``z_insert``。
        defaults: 表单默认值。
        on_change: 输入变化回调。

    Returns:
        dict: ``{"group", "stack", "count", "explicit", "total_value",
        "total_hint"}``。
    """
    group = QtWidgets.QGroupBox("网格")
    form = QtWidgets.QFormLayout(group)

    count = QtWidgets.QRadioButton("目标点数")
    explicit = QtWidgets.QRadioButton("手填格点数")
    buttons = QtWidgets.QButtonGroup(group)
    buttons.addButton(count, 0)
    buttons.addButton(explicit, 1)
    fields["grid_count"] = count
    fields["grid_explicit"] = explicit
    form.addRow(_row(count, explicit))

    stack = QtWidgets.QStackedWidget()
    count_panel = QtWidgets.QWidget()
    count_form = QtWidgets.QFormLayout(count_panel)
    target = _int_spin(1, TARGET_COUNT_MAX, defaults["target_count"])
    target.valueChanged.connect(on_change)
    fields["target_count"] = target
    count_form.addRow("目标点数", target)
    count_form.addRow(_caption(
        f"自动凑一组 X×Y×Z 格点，恰好凑出这个点数。上限 "
        f"{TARGET_COUNT_MAX} = {scan.COUNT_GRID_MAX_PER_AXIS + 1}³ + "
        f"{scan.COUNT_GRID_MAX_PER_AXIS}×{scan.COUNT_GRID_MAX_INSERT}"
        f"（单轴份数 / 单次插点的硬约束），再往上凑不出来；"
        f"凑不出来会显示 core 的报错原文。"))
    stack.addWidget(count_panel)

    explicit_panel = QtWidgets.QWidget()
    explicit_layout = QtWidgets.QVBoxLayout(explicit_panel)
    explicit_layout.setContentsMargins(0, 0, 0, 0)
    explicit_layout.setSpacing(4)
    columns = []
    for key, title in (("xs", "X 格点数"), ("ys", "Y 格点数"),
                       ("zs", "Z 格点数"), ("z_insert", "换层插点")):
        insert = key == "z_insert"
        box = _int_spin(0 if insert else 1,
                        Z_INSERT_MAX if insert else GRID_VERTICES_MAX,
                        defaults[key])
        box.valueChanged.connect(on_change)
        fields[key] = box
        columns.append((title, box))
    explicit_layout.addWidget(_inline(columns))
    # 文案里的公式必须是真公式：插点按“换层次数”算，即 Z 格点数 − 1
    caption = _caption(
        "格点数 = 该方向的点数（旧的“份数” + 1）；"
        "总点数 = X×Y×Z 格点 + (Z 格点数−1) × 换层插点。")
    explicit_layout.addWidget(caption)
    stack.addWidget(explicit_panel)
    form.addRow(stack)

    total_value = QtWidgets.QLabel("—")
    total_hint = _caption(f"（超过 {POINT_COUNT_WARN} 点，无法生成）")
    form.addRow("总点数", _row(total_value, total_hint))

    count.setChecked(defaults["grid_mode"] == GRID_MODES[0])
    explicit.setChecked(defaults["grid_mode"] == GRID_MODES[1])
    stack.setCurrentIndex(0 if count.isChecked() else 1)
    return {"group": _compact_group(group), "stack": stack, "count": count,
            "explicit": explicit, "total_value": total_value,
            "total_hint": total_hint, "caption": caption}


def build_pose_group(fields, defaults, on_change):
    """参考姿态：A/B/C 三个角 + 欧拉约定（显示「内旋」，值仍是算法约定名）。"""
    group = QtWidgets.QGroupBox("参考姿态")
    form = QtWidgets.QFormLayout(group)
    columns = []
    for index, (key, title) in enumerate(
            (("rpy_a", "A (roll)"), ("rpy_b", "B (pitch)"),
             ("rpy_c", "C (yaw)"))):
        box = _double_spin(-360.0, 360.0, defaults["rpy"][index], UNIT_DEG)
        box.valueChanged.connect(on_change)
        fields[key] = box
        columns.append((title, box))
    form.addRow(_inline(columns))

    combo = QtWidgets.QComboBox()
    for value, label in ui_state.euler_convention_options().items():
        combo.addItem(label, value)
    index = combo.findData(defaults["euler"])
    combo.setCurrentIndex(index if index >= 0 else 0)
    combo.currentIndexChanged.connect(on_change)
    fields["euler"] = combo
    form.addRow("欧拉约定", combo)
    return _compact_group(group)


def build_perturb_group(fields, defaults, on_change):
    """姿态扰动：三轴半宽 + 随机种子。"""
    group = QtWidgets.QGroupBox("姿态扰动")
    form = QtWidgets.QFormLayout(group)
    columns = []
    for index, (key, title) in enumerate(
            (("perturb_a", "±A (roll)"), ("perturb_b", "±B (pitch)"),
             ("perturb_c", "±C (yaw)"))):
        box = _double_spin(0.0, 90.0, defaults["perturb_deg"][index], UNIT_DEG)
        box.valueChanged.connect(on_change)
        fields[key] = box
        columns.append((title, box))
    form.addRow(_inline(columns))

    seed = _int_spin(0, 2_147_483_647, defaults["seed"])
    seed.valueChanged.connect(on_change)
    fields["seed"] = seed
    form.addRow("随机种子", seed)
    return _compact_group(group)


def build_guard_group(fields, defaults, on_change, with_attempts: bool = True,
                      with_seg_mode: bool = True):
    """守卫：|J5| / |J3| / 段内 / 条件数 / 段模型 / 段采样 / 重采上限 / q-seed。

    Args:
        fields: ``{键: 控件}``。
        defaults: 表单默认值。
        on_change: 输入变化回调。
        with_attempts: 是否给「重采上限」（``j5_attempts``）。生成页有这一项；
            修复页**不给**——网页版 ``pages/repair.py`` 的 ``collect()`` 不提交它，
            ``core._repair_namespace`` 也没有对应字段（缺项即用 CLI 默认值）。
        with_seg_mode: 是否给「段模型」下拉框。修复页**不给**：修复流程的段剖面
            固定 MOVL（``avoid_singularity.segment_profiles`` 没有 PTP 分支，
            ``core._repair_context`` 也不传 ``seg_mode``），给一个不生效的下拉框
            只会误导现场——那里改成一句说明（与网页版一致）。

    Returns:
        ``QGroupBox``。
    """
    group = QtWidgets.QGroupBox("守卫")
    form = QtWidgets.QFormLayout(group)
    columns = []
    for key, title in (("j5_min", "|J5| 下限"), ("j3_min", "|J3| 下限"),
                       ("seg_min", "段内下限")):
        box = _double_spin(0.0, 90.0, defaults[key], UNIT_DEG)
        box.valueChanged.connect(on_change)
        fields[key] = box
        columns.append((title, box))
    form.addRow(_inline(columns))

    cond = _double_spin(1.0, 1000.0, defaults["cond_max"], "",
                        decimals=2, step=0.5)
    cond.valueChanged.connect(on_change)
    fields["cond_max"] = cond

    mode = None
    if with_seg_mode:
        mode = QtWidgets.QComboBox()
        for value, label in SEG_MODES.items():
            mode.addItem(label, value)
        index = mode.findData(defaults.get("seg_mode", DEFAULT_SEG_MODE))
        mode.setCurrentIndex(index if index >= 0 else 0)
        mode.currentIndexChanged.connect(on_change)
        fields["seg_mode"] = mode

    samples = _int_spin(2, 200, defaults["seg_samples"])
    samples.valueChanged.connect(on_change)
    fields["seg_samples"] = samples

    row = [("条件数上限", cond), ("段采样", samples)]
    if with_attempts:
        attempts = _int_spin(1, 100, defaults["j5_attempts"])
        attempts.valueChanged.connect(on_change)
        fields["j5_attempts"] = attempts
        row.append(("重采上限", attempts))
    form.addRow(_inline(row))
    if mode is not None:
        form.addRow("段模型", mode)

    q_seed = QtWidgets.QLineEdit(str(defaults["q_seed"]))
    q_seed.textChanged.connect(on_change)
    fields["q_seed"] = q_seed
    form.addRow("q-seed（6 个角）", q_seed)
    return _compact_group(group)
