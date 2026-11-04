# -*- coding: utf-8 -*-
"""VPE 验证 CLI：会话 JSON -> markdown 报告 + 可选机器可读 JSON。

用法::

    python -m handeye.verify SESSION.json [选项]

    -o, --out PATH            markdown 报告输出路径（默认 vpe_report.md）
        --json PATH           额外输出机器可读 JSON（可选）
        --matrix16 V1..V16    用命令行给的 16 个值覆盖 session 内的 X
        --matrix-units {mm,m} --matrix16 的长度单位（默认 mm）
        --no-report           不写 markdown（只打印摘要 + 可选 JSON）
        --fk-tolerance-mm F   FK 一致性告警阈值（默认 1.0）
        --strict              FK 一致性超阈值时以退出码 3 结束
        --quiet               只输出一行摘要

退出码：

* ``0``：成功（``--strict`` 且 FK 未超阈值也返回 0）；
* ``2``：参数/数据错误（会话无法加载、缺少 X、矩阵非法、VPE 无法计算、
  写盘失败）——只在 stderr 给提示，不抛 traceback；
* ``3``：仅 ``--strict`` 且 FK 一致性位置最大偏差超过
  ``--fk-tolerance-mm`` 时返回（报告/JSON 仍然会按参数写出，便于排查）。

实现约定：``main(argv=None) -> int`` 负责全部打印与返回码；核心计算复用
:mod:`handeye.session` / :mod:`handeye.vpe` / :mod:`handeye.report`。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from handeye.report import (
    REPORT_FORMAT_VERSION,
    bias_diagnosis,
    render_markdown,
    write_markdown,
)
from handeye.se3 import load_matrix16, to_matrix16
from handeye.session import SessionError, check_fk_consistency, load_session
from handeye.vpe import VpeError, compute_vpe

#: 视为「零」的数值下限（与 report 的格式约定一致）。
_ZERO_EPSILON = 5e-5


class _UsageError(Exception):
    """argparse 的参数错误，``main`` 捕获后转换为退出码 2。"""


class _ArgumentParser(argparse.ArgumentParser):
    """不直接 ``sys.exit`` 的 ArgumentParser，便于测试直接调用 ``main``。"""

    def error(self, message):
        raise _UsageError(message)


def _format_number(value, suffix: str = "") -> str:
    """与报告一致的 4 位小数格式（非有限值输出 ``nan`` 占位）。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "nan"
    if not math.isfinite(number):
        return "nan"
    if abs(number) < _ZERO_EPSILON:
        number = 0.0
    rounded = round(number, 4)
    if rounded == 0.0:
        rounded = 0.0
    return f"{rounded:.4f}{suffix}"


def _build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器（不读取 ``sys.argv``）。"""
    parser = _ArgumentParser(
        prog="python -m handeye.verify",
        description="手眼验证（VPE）报告生成器：读取会话 JSON，输出 markdown/JSON。",
    )
    parser.add_argument("session", help="会话 JSON 路径")
    parser.add_argument(
        "-o", "--out", default="vpe_report.md",
        help="markdown 报告输出路径（默认 vpe_report.md）")
    parser.add_argument(
        "--json", dest="json_path", default=None,
        help="额外输出机器可读 JSON（可选）")
    parser.add_argument(
        "--matrix16", nargs=16, type=float, default=None, metavar="V",
        help="用命令行给出的 16 个值（row-major）覆盖会话内的 handeye_matrix")
    parser.add_argument(
        "--matrix-units", choices=("mm", "m"), default="mm",
        help="--matrix16 的长度单位（默认 mm）")
    parser.add_argument(
        "--no-report", action="store_true",
        help="不写 markdown（只打印摘要与可选 JSON）")
    parser.add_argument(
        "--fk-tolerance-mm", type=float, default=1.0,
        help="FK 一致性告警阈值（mm，默认 1.0）")
    parser.add_argument(
        "--strict", action="store_true",
        help="FK 一致性超阈值时以退出码 3 结束")
    parser.add_argument(
        "--quiet", action="store_true",
        help="只输出一行摘要（抑制报告/JSON 路径与分项输出）")
    return parser


def _json_ready(value):
    """把 numpy/嵌套结构转成可 JSON 序列化的值。

    float 统一 ``round(..., 6)``；非有限值写 ``None``（配合
    ``allow_nan=False`` 保证输出里不会出现 ``NaN`` / ``Infinity``）。
    """
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, np.ndarray):
        return [_json_ready(item) for item in value.tolist()]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        number = float(value)
        if not math.isfinite(number):
            return None
        rounded = round(number, 6)
        return 0.0 if rounded == 0.0 else rounded
    return value


def _build_json_payload(session, vpe_result, fk_consistency, diagnosis, matrix):
    """按约定结构组装机器可读 JSON。"""
    meta = dict(session.meta) if isinstance(session.meta, dict) else {}
    meta.update({
        "n_samples": vpe_result["n_samples"],
        "n_frames": vpe_result["n_frames"],
        "reference_mm": vpe_result["reference_mm"],
        "spread_mean_mm": vpe_result["spread_mean_mm"],
        "report_format_version": REPORT_FORMAT_VERSION,
    })
    return {
        "meta": _json_ready(meta),
        "metrics": _json_ready(vpe_result["metrics"]),
        "errors": _json_ready(vpe_result["errors"]),
        "fk_consistency": _json_ready(fk_consistency),
        "bias_diagnosis": _json_ready(diagnosis),
        "matrix16_mm": _json_ready(to_matrix16(matrix, units="mm")),
    }


def _write_json(path, payload) -> Path:
    """写 UTF-8 JSON（父目录不存在则创建），返回路径。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    return target


def _print_summary(session, vpe_result, diagnosis, *, quiet: bool) -> None:
    """打印人读摘要（``quiet`` 时只有一行）。"""
    metrics = vpe_result["metrics"]
    n_samples = int(vpe_result["n_samples"])
    n_frames = int(vpe_result["n_frames"])
    rmse = _format_number(metrics["vpe_rmse_mm"], " mm")
    maximum = _format_number(metrics["vpe_max_mm"], " mm")
    p95 = _format_number(metrics["vpe_p95_mm"], " mm")
    bias_norm = _format_number(metrics["bias_norm_mm"], " mm")
    spread = _format_number(vpe_result["spread_mean_mm"], " mm")
    ratio = _format_number(diagnosis["bias_ratio"])

    if quiet:
        print(
            f"VPE n_samples={n_samples} n_frames={n_frames} RMSE={rmse} "
            f"MAX={maximum} P95={p95} bias_norm={bias_norm} spread={spread} "
            f"diagnosis={diagnosis['dominant']}"
            f"(axis={diagnosis['dominant_axis']},ratio={ratio})")
        return

    source = "—" if session.source_path is None else str(session.source_path)
    print(f"会话: {source}")
    print(f"样本数: {n_samples}  帧数: {n_frames}")
    print(f"VPE_RMSE: {rmse}")
    print(f"VPE_MAX: {maximum}")
    print(f"VPE_P95: {p95}")
    print(f"VPE_MEAN: {_format_number(metrics['vpe_mean_mm'], ' mm')}")
    print(f"‖bias‖: {bias_norm}")
    print(f"视觉帧间重复性 (spread): {spread}")
    print(
        f"诊断: {diagnosis['dominant']}（主偏置轴 "
        f"{diagnosis['dominant_axis']}，bias_ratio={ratio}）")
    print(f"  {diagnosis['message']}")


def main(argv=None) -> int:
    """CLI 入口（可被测试直接调用）。

    Args:
        argv: 参数列表；``None`` 时读 ``sys.argv[1:]``。

    Returns:
        退出码：0 成功；2 参数/数据错误；3 仅 ``--strict`` 且 FK 超阈值。
    """
    parser = _build_parser()
    try:
        args = parser.parse_args(argv)
    except _UsageError as error:
        print(f"参数错误：{error}", file=sys.stderr)
        print(parser.format_usage().rstrip(), file=sys.stderr)
        return 2
    except SystemExit as exit_code:  # --help / --version
        return int(exit_code.code or 0)

    tolerance = float(args.fk_tolerance_mm)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        print("参数错误：--fk-tolerance-mm 必须为有限非负数", file=sys.stderr)
        return 2

    # 1. 加载会话（SessionError 统一转 stderr + 退出码 2，不抛 traceback）
    try:
        session = load_session(args.session)
    except SessionError as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2

    # 2. 解析手眼矩阵：--matrix16 优先，否则用会话内的 X
    if args.matrix16 is not None:
        try:
            matrix = load_matrix16(args.matrix16, units=args.matrix_units)
        except ValueError as error:
            print(f"错误：--matrix16 不合法：{error}", file=sys.stderr)
            return 2
    else:
        matrix = session.handeye_matrix
        if matrix is None:
            print(
                "错误：会话未提供 handeye_matrix，请用 --matrix16 或先求解。",
                file=sys.stderr)
            return 2

    # 3. FK 一致性检查：超阈值时醒目告警（不中断，除非 --strict）
    fk_consistency = check_fk_consistency(session)
    fk_exceeded = (
        int(fk_consistency["n_compared"]) > 0
        and float(fk_consistency["position_max_mm"]) > tolerance)
    if fk_exceeded:
        print(
            "WARNING: FK 一致性超阈值：位置最大偏差 "
            f"{_format_number(fk_consistency['position_max_mm'], ' mm')} > "
            f"阈值 {_format_number(tolerance, ' mm')}"
            f"（RMS {_format_number(fk_consistency['position_rms_mm'], ' mm')}，"
            f"旋转最大 {_format_number(fk_consistency['rotation_max_deg'], '°')}，"
            f"样本 {int(fk_consistency['n_compared'])} 个）。"
            "建议检查 DH 参数/零点、欧拉角约定、长度单位与关节角约定；"
            "确认数据无误后可用 --fk-tolerance-mm 放宽阈值。",
            file=sys.stderr)

    # 4. VPE
    try:
        vpe_result = compute_vpe(session, matrix=matrix)
        diagnosis = bias_diagnosis(vpe_result)
    except (VpeError, ValueError) as error:
        print(f"错误：无法计算 VPE：{error}", file=sys.stderr)
        return 2

    # 5. 摘要
    _print_summary(session, vpe_result, diagnosis, quiet=args.quiet)

    # 6. markdown 报告
    if not args.no_report:
        try:
            text = render_markdown(
                session, vpe_result, fk_consistency=fk_consistency)
        except ValueError as error:
            print(f"错误：无法生成 markdown 报告：{error}", file=sys.stderr)
            return 2
        try:
            written = write_markdown(args.out, text)
        except OSError as error:
            print(f"错误：无法写入 markdown 报告：{error}", file=sys.stderr)
            return 2
        if not args.quiet:
            print(f"报告已写入: {written}")

    # 7. 可选 JSON
    if args.json_path is not None:
        try:
            payload = _build_json_payload(
                session, vpe_result, fk_consistency, diagnosis, matrix)
            written_json = _write_json(args.json_path, payload)
        except (OSError, TypeError, ValueError) as error:
            print(f"错误：无法写入 JSON：{error}", file=sys.stderr)
            return 2
        if not args.quiet:
            print(f"JSON 已写入: {written_json}")

    # 8. 退出码：--strict 且 FK 超阈值 -> 3
    return 3 if (args.strict and fk_exceeded) else 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
