# -*- coding: utf-8 -*-
"""VPE 结果的 markdown 报告渲染。

报告结构
--------
小节标题与顺序固定（便于逐字节 diff 与机器解析）::

    # 手眼验证（VPE）报告
    ## 1. 会话概要
    ## 2. VPE 指标总览
    ## 3. 逐轴误差
    ## 4. 逐位姿明细
    ## 5. 偏置方向诊断
    ## 6. 手眼矩阵 X（flange←camera）
    ## 7. 运动学一致性自检
    ## 8. 复现方式
    ## 9. 备注              # 仅当 extra_notes 非空时出现

复现性约定
----------
* 输出只依赖入参：不写时间戳、不写绝对路径、不含随机内容；
* 长度统一 mm、角度统一 deg，所有浮点一律 ``f"{v:.4f}"``（角度加 ``°``）；
* ``abs(v) < 5e-5`` 的数值统一输出 ``0.0000``，避免 ``-0.0000``；
* 会话里的绝对路径只显示「仓库内相对路径」或「文件名」，报告可直接外发；
* 库函数只返回字符串/路径，不打印、不 ``sys.exit``；写盘由调用方负责。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from handeye.se3 import to_matrix16
from handeye.session import REPO_ROOT

#: 报告结构版本号：小节标题/表格列发生不兼容变化时递增。
REPORT_FORMAT_VERSION = 1

#: 逐轴统计的顺序。
_AXES = ("x", "y", "z")

#: FK 一致性自检的判读阈值（mm，位置最大偏差）。
_FK_SELF_CONSISTENT_MM = 1.0

#: 视为「零」的数值下限（用于压制 ``-0.0000``）。
_ZERO_EPSILON = 5e-5

#: ``bias_ratio`` 低于该值时判为位姿相关误差（与 ``bias_ratio_threshold`` 独立）。
_POSE_DEPENDENT_RATIO = 0.3

#: 固定偏置主导的默认比例阈值。
_DEFAULT_BIAS_RATIO_THRESHOLD = 0.5


# ---------------------------------------------------------------------------
# 格式化工具
# ---------------------------------------------------------------------------


def _format_number(value, suffix: str = "") -> str:
    """把数值格式化为 4 位小数（非有限值输出 ``—``）。

    Args:
        value: 任意可转 ``float`` 的数值。
        suffix: 紧跟数值的单位后缀，如 ``" mm"`` / ``"°"``。

    Returns:
        ``f"{v:.4f}{suffix}"``；``abs(v) < 5e-5`` 时固定输出 ``0.0000``。
    """
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(number):
        return "—"
    if abs(number) < _ZERO_EPSILON:
        number = 0.0
    rounded = round(number, 4)
    if rounded == 0.0:
        rounded = 0.0  # 消除 -0.0
    return f"{rounded:.4f}{suffix}"


def _format_vector(vector, unit: str = "mm") -> str:
    """把 3 维向量格式化为 ``(x, y, z) unit``；长度不符或非有限值给 ``—``。"""
    try:
        values = np.asarray(vector, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return "—"
    if values.size != 3 or not np.all(np.isfinite(values)):
        return "—"
    body = ", ".join(_format_number(value) for value in values)
    return f"({body}) {unit}" if unit else f"({body})"


def _display_path(value) -> str:
    """把路径转成不含绝对路径的展示文本。

    仓库内的绝对路径显示为相对仓库根的 POSIX 路径，仓库外的绝对路径只显示
    文件名；``None`` / 空串显示 ``—``。报告因此可以安全外发，且同一会话在
    不同工作目录下渲染结果一致。
    """
    if value is None:
        return "—"
    text = str(value)
    if not text.strip():
        return "—"
    candidate = Path(text)
    if not candidate.is_absolute():
        return candidate.as_posix()
    try:
        return candidate.resolve().relative_to(REPO_ROOT).as_posix()
    except (OSError, ValueError):
        return candidate.name


def _meta_text(value) -> str:
    """把 ``meta`` 里的任意 JSON 值转成稳定的单行文本。"""
    if value is None:
        return "—"
    if isinstance(value, str):
        return value if value.strip() else "—"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        return _format_number(value)
    try:
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    except (TypeError, ValueError):  # pragma: no cover - meta 来自 JSON，正常不会触发
        return str(value)


def _sorted_target_ids(ids) -> list:
    """靶球 id 排序：纯数字按数值升序，其余按字符串排在后面。"""
    def key(item: str):
        try:
            return (0, int(item), "")
        except (TypeError, ValueError):
            return (1, 0, item)

    return sorted((str(item) for item in ids), key=key)


def _table(headers, rows) -> list:
    """生成标准 markdown 管道表格（列对齐用 ``---``）。"""
    lines = [
        "| " + " | ".join(str(header) for header in headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend(
        "| " + " | ".join(str(cell) for cell in row) + " |" for row in rows)
    return lines


# ---------------------------------------------------------------------------
# 偏置方向诊断
# ---------------------------------------------------------------------------


def bias_diagnosis(vpe_result, *, bias_ratio_threshold=0.5) -> dict:
    """判断误差是固定偏置主导还是随位姿变化主导。

    判据（严格按此顺序，先判 ``systematic`` 再判 ``pose-dependent``）::

        bias_ratio = ‖bias‖ / VPE_RMSE        （RMSE≈0 时取 0）
        bias_ratio >= bias_ratio_threshold    -> "systematic"
        bias_ratio < 0.3                      -> "pose-dependent"
        其它                                  -> "mixed"

    Args:
        vpe_result: :func:`handeye.vpe.compute_vpe` 的返回值（至少含
            ``metrics.vpe_rmse_mm``、``metrics.bias_norm_mm``、``metrics.bias``）。
        bias_ratio_threshold: 判定为固定偏置主导的比例阈值，默认 0.5。

    Returns:
        dict：

        * ``dominant``：``"systematic"`` / ``"pose-dependent"`` / ``"mixed"``；
        * ``bias_norm_mm`` / ``rmse_mm`` / ``bias_ratio``：本次判据用到的数值；
        * ``dominant_axis``：``bias`` 绝对值最大的轴（``"x"``/``"y"``/``"z"``，
          并列时取靠前的轴）；
        * ``message``：中文结论（含判据数值与优先排查方向）。

    Raises:
        ValueError: ``vpe_result`` 缺少必要字段，或数值不是有限值。
    """
    try:
        metrics = vpe_result["metrics"]
        bias = np.asarray(metrics["bias"], dtype=float).reshape(-1)
        rmse = float(metrics["vpe_rmse_mm"])
        bias_norm = float(metrics.get("bias_norm_mm", np.linalg.norm(bias)))
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise ValueError(f"vpe_result 缺少偏置诊断所需字段：{error}") from error
    if bias.size != 3:
        raise ValueError(f"metrics.bias 必须为 3 个数，当前 {bias.size} 个")
    if (not np.all(np.isfinite(bias)) or not math.isfinite(rmse)
            or not math.isfinite(bias_norm)):
        raise ValueError("metrics.bias / vpe_rmse_mm / bias_norm_mm 含非有限值")

    threshold = float(bias_ratio_threshold)
    if not math.isfinite(threshold):
        raise ValueError("bias_ratio_threshold 必须为有限值")

    ratio = 0.0 if rmse <= 1e-12 else bias_norm / rmse
    axis_index = int(np.argmax(np.abs(bias)))
    dominant_axis = _AXES[axis_index]

    norm_text = _format_number(bias_norm)
    ratio_text = _format_number(ratio)
    threshold_text = _format_number(threshold)
    axis_bias_text = _format_number(bias[axis_index])

    if ratio >= threshold:
        dominant = "systematic"
        message = (
            f"误差以固定偏置为主（‖bias‖={norm_text} mm，"
            f"bias_ratio={ratio_text} ≥ 阈值 {threshold_text}，"
            f"主偏置轴 {dominant_axis}={axis_bias_text} mm）："
            f"优先怀疑 Tool0—靶球偏置 ΔP、手眼平移或跟踪仪建站偏置。")
    elif ratio < _POSE_DEPENDENT_RATIO:
        dominant = "pose-dependent"
        message = (
            f"误差随位姿变化（‖bias‖={norm_text} mm，"
            f"bias_ratio={ratio_text} < {_format_number(_POSE_DEPENDENT_RATIO)}，"
            f"主偏置轴 {dominant_axis}={axis_bias_text} mm）："
            f"优先怀疑运动学模型（DH/零点）、手眼旋转或视觉位姿估计。")
    else:
        dominant = "mixed"
        message = (
            f"固定偏置与位姿相关误差并存（‖bias‖={norm_text} mm，"
            f"{_format_number(_POSE_DEPENDENT_RATIO)} ≤ bias_ratio={ratio_text}"
            f" < 阈值 {threshold_text}，主偏置轴 {dominant_axis}="
            f"{axis_bias_text} mm）：先校准 Tool0—靶球偏置 ΔP/手眼平移，"
            f"再复核运动学模型与手眼旋转。")

    return {
        "dominant": dominant,
        "bias_norm_mm": float(bias_norm),
        "rmse_mm": float(rmse),
        "bias_ratio": float(ratio),
        "dominant_axis": dominant_axis,
        "message": message,
    }


# ---------------------------------------------------------------------------
# markdown 渲染
# ---------------------------------------------------------------------------


def _session_section(session, vpe_result) -> list:
    """``## 1. 会话概要`` 的两列表格。"""
    meta = session.meta if isinstance(session.meta, dict) else {}
    dh_override = meta.get("dh_override")
    target_ids = _sorted_target_ids(session.all_target_ids())
    reference = vpe_result.get("reference_mm", session.reference_effective)
    robot_urdf = meta.get("robot_urdf")
    rows = [
        ("组别", _meta_text(meta.get("group"))),
        ("机器人模型", _display_path(robot_urdf)
         if isinstance(robot_urdf, str) else _meta_text(robot_urdf)),
        ("DH 覆盖", "基准" if dh_override is None
         else (_display_path(dh_override) if isinstance(dh_override, str)
               else _meta_text(dh_override))),
        ("关节约定", _meta_text(meta.get("joint_convention"))),
        ("欧拉约定", getattr(session, "euler_convention", "xyz-intrinsic")),
        ("坐标单位", _meta_text(meta.get("units"))),
        ("样本数", str(int(vpe_result["n_samples"]))),
        ("总帧数", str(int(vpe_result["n_frames"]))),
        ("靶球 id 列表", ", ".join(target_ids) if target_ids else "—"),
        ("跟踪仪实测球心", _format_vector(session.reference_point)),
        ("Tool0—靶球偏置 ΔP",
         "—" if session.delta_p_base is None
         else _format_vector(session.delta_p_base)),
        ("参与计算的参考点", _format_vector(reference)),
    ]
    return ["## 1. 会话概要", ""] + _table(("项目", "值"), rows)


def _overview_section(vpe_result) -> list:
    """``## 2. VPE 指标总览``。"""
    metrics = vpe_result["metrics"]
    rows = [
        ("VPE_RMSE", _format_number(metrics["vpe_rmse_mm"], " mm")),
        ("VPE_MAX", _format_number(metrics["vpe_max_mm"], " mm")),
        ("VPE_P95", _format_number(metrics["vpe_p95_mm"], " mm")),
        ("VPE_MEAN", _format_number(metrics["vpe_mean_mm"], " mm")),
        ("‖bias‖", _format_number(metrics["bias_norm_mm"], " mm")),
        ("视觉帧间重复性 (spread)",
         _format_number(vpe_result.get("spread_mean_mm", 0.0), " mm")),
    ]
    return ["## 2. VPE 指标总览", ""] + _table(("指标", "数值"), rows)


def _axis_section(vpe_result) -> list:
    """``## 3. 逐轴误差``。"""
    metrics = vpe_result["metrics"]
    rows = [
        (axis,
         _format_number(metrics["per_axis_rmse_mm"][axis]),
         _format_number(metrics["per_axis_bias_mm"][axis]))
        for axis in _AXES
    ]
    return (["## 3. 逐轴误差", ""]
            + _table(("轴", "RMSE (mm)", "Bias (mm)"), rows))


def _per_pose_section(vpe_result) -> list:
    """``## 4. 逐位姿明细``。"""
    rows = []
    for entry in vpe_result["errors"]:
        error = np.asarray(entry["error"], dtype=float).reshape(-1)
        rows.append((
            entry["pose_id"],
            int(entry["n_frames"]),
            int(entry["n_ids"]),
            _format_number(entry["vpe_mm"]),
            _format_number(error[0]),
            _format_number(error[1]),
            _format_number(error[2]),
            _format_number(entry.get("spread_mm", 0.0)),
        ))
    headers = ("pose_id", "帧数", "靶球数", "VPE (mm)",
               "e_x", "e_y", "e_z", "spread (mm)")
    return ["## 4. 逐位姿明细", ""] + _table(headers, rows)


def _diagnosis_section(vpe_result) -> list:
    """``## 5. 偏置方向诊断``。"""
    diagnosis = bias_diagnosis(vpe_result)
    metrics = vpe_result["metrics"]
    bias = np.asarray(metrics["bias"], dtype=float).reshape(-1)
    axis = diagnosis["dominant_axis"]
    axis_index = _AXES.index(axis)
    lines = [
        "## 5. 偏置方向诊断",
        "",
        f"- ‖bias‖ = {_format_number(diagnosis['bias_norm_mm'], ' mm')}，"
        f"VPE_RMSE = {_format_number(diagnosis['rmse_mm'], ' mm')}，"
        f"bias_ratio = {_format_number(diagnosis['bias_ratio'])}",
        f"- 主偏置轴：`{axis}`（分量 {_format_number(bias[axis_index], ' mm')}）",
        f"- 结论：`{diagnosis['dominant']}`",
        "",
        diagnosis["message"],
        "",
        f"判据（`bias_ratio = ‖bias‖ / VPE_RMSE`，本报告阈值 "
        f"`{_format_number(_DEFAULT_BIAS_RATIO_THRESHOLD)}`，即 "
        f"`bias_ratio_threshold` 默认值）：",
        "",
        f"- `bias_ratio ≥ {_format_number(_DEFAULT_BIAS_RATIO_THRESHOLD)}` → "
        f"`systematic`：误差以固定偏置为主；",
        f"- `bias_ratio < {_format_number(_POSE_DEPENDENT_RATIO)}` → "
        f"`pose-dependent`：误差随位姿变化为主；",
        "- 其余 → `mixed`：固定偏置与位姿相关误差并存。",
        "",
        "规则按上述顺序判断：先判 `systematic`，再判 `pose-dependent`。",
    ]
    return lines


def _matrix_section(vpe_result) -> list:
    """``## 6. 手眼矩阵 X（flange←camera）``。"""
    matrix = np.asarray(vpe_result["matrix"], dtype=float).reshape(4, 4)
    rows = []
    for row_index in range(4):
        rows.append(
            [row_index + 1]
            + [_format_number(matrix[row_index, column]) for column in range(4)])
    matrix16 = to_matrix16(matrix, units="mm")
    matrix16_line = ", ".join(_format_number(value) for value in matrix16)
    lines = [
        "## 6. 手眼矩阵 X（flange←camera）",
        "",
        "语义：`p_base = T_bh(q) · X · p_cam`；`X` 为 flange←camera，"
        "表中平移单位为 **mm**（旋转部分无量纲）。",
        "",
    ]
    lines += _table(("行", "第 1 列", "第 2 列", "第 3 列", "第 4 列"), rows)
    lines += [
        "",
        "**`matrix16_rowmajor`（mm，row-major，可直接粘到验证脚本/配置备份）**：",
        "",
        "```text",
        matrix16_line,
        "```",
        "",
        "> 注意：**common_config 期望米制（meters）**，粘贴前须把上述 16 值中的"
        "平移分量除以 1000；本节表格与 16 值行都是 mm。",
    ]
    return lines


def _fk_section(fk_consistency) -> list:
    """``## 7. 运动学一致性自检``。"""
    lines = ["## 7. 运动学一致性自检", ""]
    if fk_consistency is None:
        lines.append(
            "未提供：调用方未传入 `fk_consistency`（会话缺少 "
            "`recorded_pose` 时也无法计算），本节跳过。")
        return lines

    n_compared = int(fk_consistency.get("n_compared", 0))
    n_missing = int(fk_consistency.get("n_missing", 0))
    position_max = float(fk_consistency.get("position_max_mm", 0.0))
    position_rms = float(fk_consistency.get("position_rms_mm", 0.0))
    rotation_max = float(fk_consistency.get("rotation_max_deg", 0.0))
    rotation_rms = float(fk_consistency.get("rotation_rms_deg", 0.0))
    worst = fk_consistency.get("worst")
    worst_id = "—" if worst is None else str(worst.get("pose_id", "—"))

    rows = [
        ("比较样本数", str(n_compared)),
        ("缺失数", str(n_missing)),
        ("位置最大偏差 (mm)", _format_number(position_max)),
        ("位置 RMS 偏差 (mm)", _format_number(position_rms)),
        ("旋转最大偏差 (deg)", _format_number(rotation_max, "°")),
        ("旋转 RMS 偏差 (deg)", _format_number(rotation_rms, "°")),
        ("最差 pose_id", worst_id),
    ]
    lines += _table(("项目", "值"), rows)
    lines.append("")
    if n_compared == 0:
        lines.append(
            f"判读：没有参与比较的样本（缺失 {n_missing} 个），"
            "无法判断 FK 与控制器回传位姿是否自洽。")
    elif position_max <= _FK_SELF_CONSISTENT_MM:
        lines.append(
            f"判读：位置最大偏差 {_format_number(position_max, ' mm')} ≤ "
            f"{_format_number(_FK_SELF_CONSISTENT_MM)} mm，"
            "FK 与控制器回传位姿自洽。")
    else:
        lines.append(
            f"判读：位置最大偏差 {_format_number(position_max, ' mm')} > "
            f"{_format_number(_FK_SELF_CONSISTENT_MM)} mm，FK 与控制器回传位姿"
            "不自洽；建议检查 DH 参数/零点、欧拉角约定、长度单位与 "
            "`meta.joint_convention`。")
    return lines


def _reproduce_section(session) -> list:
    """``## 8. 复现方式``。"""
    session_arg = _display_path(session.source_path)
    if session_arg == "—":
        session_arg = "<SESSION.json>"
    lines = [
        "## 8. 复现方式",
        "",
        "在仓库根执行（`SESSION.json` 为会话文件；报告只写相对路径/文件名，"
        "避免绝对路径）：",
        "",
        "```bash",
        f"PYTHONPATH=python python3 -m handeye.verify {session_arg} "
        "-o vpe_report.md",
        f"PYTHONPATH=python python3 -m handeye.verify {session_arg} "
        "-o vpe_report.md --json vpe_report.json",
        "```",
    ]
    return lines


def _notes_section(extra_notes) -> list:
    """``## 9. 备注``（仅当 ``extra_notes`` 非空）。"""
    if isinstance(extra_notes, str):
        extra_notes = [extra_notes]
    notes = [str(note).strip() for note in extra_notes if str(note).strip()]
    if not notes:
        return []
    return ["## 9. 备注", ""] + [f"- {note}" for note in notes]


def render_markdown(session, vpe_result, *, fk_consistency=None,
                    title="手眼验证（VPE）报告", extra_notes=None) -> str:
    """渲染 markdown 报告并返回字符串（调用方负责写盘）。

    Args:
        session: :class:`handeye.session.Session`（提供 meta/参考点/靶球 id/
            会话路径）。
        vpe_result: :func:`handeye.vpe.compute_vpe` 的返回值。
        fk_consistency: :func:`handeye.session.check_fk_consistency` 的返回值；
            ``None`` 时第 7 节写「未提供」。
        title: 一级标题文字，默认「手眼验证（VPE）报告」。
        extra_notes: ``list[str]``；非空时追加 ``## 9. 备注`` 条目。

    Returns:
        UTF-8 兼容的 markdown 文本，以换行结尾；同输入逐字节可复现。

    Raises:
        ValueError: ``vpe_result`` 缺少必要字段或数值非法。
    """
    try:
        matrix = np.asarray(vpe_result["matrix"], dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(f"vpe_result.matrix 需为 4x4，当前 {matrix.shape}")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("vpe_result.matrix 含非有限值")
        # 提前触发指标缺失错误，避免只渲染出一半报告。
        _ = vpe_result["metrics"]["vpe_rmse_mm"]
        _ = vpe_result["n_samples"]
        _ = vpe_result["n_frames"]
        _ = vpe_result["errors"]
    except KeyError as error:
        raise ValueError(f"vpe_result 缺少必要字段：{error}") from error

    lines = [f"# {title}", ""]
    lines += _session_section(session, vpe_result)
    lines += [""]
    lines += _overview_section(vpe_result)
    lines += [""]
    lines += _axis_section(vpe_result)
    lines += [""]
    lines += _per_pose_section(vpe_result)
    lines += [""]
    lines += _diagnosis_section(vpe_result)
    lines += [""]
    lines += _matrix_section(vpe_result)
    lines += [""]
    lines += _fk_section(fk_consistency)
    lines += [""]
    lines += _reproduce_section(session)
    notes = _notes_section(extra_notes or [])
    if notes:
        lines += [""]
        lines += notes
    return "\n".join(lines).rstrip("\n") + "\n"


def write_markdown(path, text) -> Path:
    """把报告写成 UTF-8 文本（父目录不存在则创建）。

    Args:
        path: 输出路径（``str`` / :class:`pathlib.Path`）。
        text: :func:`render_markdown` 的返回值。

    Returns:
        实际写入的 :class:`pathlib.Path`。

    Raises:
        OSError: 目录无法创建或文件无法写入。
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return target


__all__ = [
    "REPORT_FORMAT_VERSION",
    "bias_diagnosis",
    "render_markdown",
    "write_markdown",
]
