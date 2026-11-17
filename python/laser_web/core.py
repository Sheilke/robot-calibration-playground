# -*- coding: utf-8 -*-
"""``laser_web`` 的服务层：包装现有算法，纯函数，**不 import 任何 UI 库**。

三条硬约束（设计文档 §6 / §7）：

1. 算法一律 **import 复用**，本模块不复制任何公式；
2. 参数校验只有一套——把表单值塞进 ``argparse.Namespace`` 后交给
   ``aabb_snake_scan._validate_args``（CLI 用的同一个函数），把它的报错原文
   回显给前端，因此界面与 CLI 不会漂移；
3. 输入只吃 dict / 基础类型，输出 dict；**不抛异常**，错误走 ``ok: False``，
   这样 ``tests/test_core.py`` 可以脱离 UI 单测。

M1 只提供 ``list_robots`` / ``preview_domain`` / ``form_to_namespace`` 三个入口；
M2 增加 ``run_scan``（完整解算 + 产物落盘，含 ``skip_ik`` 轻量预览分支）；
M3 增加 ``build_gp``（扫描结果 + GP 模板 -> GP 文件）、``convert_urdf``
（控制器 JSON -> URDF）与 ``list_runs`` / ``clean_runs``（运行目录盘点与清理）；
M4 增加 ``locate_segment``（窗口段剖面 + 判定结论）与 ``run_repair``
（先局部换端点姿态、失败再整体挪箱，产物与 ``run_scan`` 同构）。
"""

from __future__ import annotations

import argparse
import contextlib
import functools
import hashlib
import io
import json
import math
import re
import tempfile
from pathlib import Path

import numpy as np

from excitation_trajectory import aabb_snake_scan as scan
from excitation_trajectory import avoid_singularity as avoid

from . import jobs

#: 仓库根（``<repo>/python/laser_web/core.py`` 往上两级）。
_REPO_ROOT = Path(__file__).resolve().parents[2]
#: 机器人模型目录（相对仓库根）。
CONFIG_DIR = Path("data") / "configs"
#: 体积换算：1 m³ = 1e9 mm³。
_MM3_PER_M3 = 1e9
#: ``data/configs`` 下被当作模型的扩展名 -> kind。
_MODEL_SUFFIXES = ((".urdf", "urdf"), (".json", "json"))


class _FormError(Exception):
    """表单参数非法；异常消息就是 CLI ``parser.error`` 收到的报错原文。"""


def _resolve_model_path(path) -> str:
    """模型路径按仓库根解析，避免服务从别的目录启动时 ``FileNotFoundError``。"""
    candidate = Path(str(path)).expanduser()
    if candidate.is_absolute():
        return str(candidate)
    return str((_REPO_ROOT / candidate).resolve())


def _as_float_list(values, length: int, name: str) -> list[float]:
    """把表单里的序列转成 ``length`` 个浮点数（不是校验规则，只是类型归一）。

    Args:
        values: 任意可迭代对象（页面传的是 ``ui.number`` 的 ``value`` 列表）。
        length: 期望的元素个数（3 或 6）。
        name: 出错时回显的字段名。

    Returns:
        list[float]，长度恰为 ``length``。

    Raises:
        ValueError: 有空项或个数不对（类型问题，不是 AABB 语义校验）。
    """
    items = list(values)
    if len(items) != length:
        raise ValueError(f"{name} 需要 {length} 个数值，当前 {len(items)} 个")
    if any(item is None for item in items):
        raise ValueError(f"{name} 有空项")
    return [float(item) for item in items]


def form_to_namespace(params: dict) -> argparse.Namespace:
    """把表单 dict 转成 ``aabb_snake_scan`` 认的 Namespace（供 ``_validate_args`` 用）。

    缺失项填该 CLI 的默认值：默认值不是在这里抄的，而是让真正的 CLI
    ``build_arg_parser().parse_args([])`` 生成，避免与 CLI 的默认值漂移。

    本函数**不做语义校验**（互斥、上下界、阈值范围等全部留给
    ``_validate_args``），只做「表单键 -> Namespace 字段」的搬运与类型转换。

    Args:
        params: 表单值，键与生成页一致：``robot`` / ``point_min`` /
            ``point_max`` / ``aabb6`` / ``rpy`` / ``euler`` / ``perturb_deg`` /
            ``target_count`` / ``xs`` / ``ys`` / ``zs`` / ``z_insert`` /
            ``cond_max``，以及 M2 的守卫与段检查项：``j5_min`` / ``j3_min`` /
            ``j5_attempts`` / ``seg_min`` / ``seg_mode``（``"joint"`` 或
            ``"cartesian"``）/ ``seg_samples`` / ``seed`` / ``q_seed``（逗号串
            或 6 个数列表）/ ``no_perturb`` / ``skip_ik`` / ``strict`` /
            ``cond_position`` / ``no_cond_check``。值为 ``None`` 的键视为
            「没填」，保留 CLI 默认值。

    Returns:
        Namespace，字段与 ``aabb_snake_scan`` 的 CLI 参数一一对应。
    """
    args = scan.build_arg_parser().parse_args([])
    params = params or {}

    if params.get("aabb6") is not None:
        args.aabb = _as_float_list(params["aabb6"], 6, "六个轴边界 aabb6")
    if params.get("point_min") is not None:
        args.point_min = _as_float_list(params["point_min"], 3, "最小点 point_min")
    if params.get("point_max") is not None:
        args.point_max = _as_float_list(params["point_max"], 3, "最大点 point_max")
    if params.get("rpy") is not None:
        args.rpy = _as_float_list(params["rpy"], 3, "参考姿态 rpy")
    if params.get("perturb_deg") is not None:
        args.perturb_deg = _as_float_list(params["perturb_deg"], 3,
                                          "扰动半宽 perturb_deg")
    if params.get("euler") is not None:
        args.euler = str(params["euler"])
    if params.get("cond_max") is not None:
        args.cond_max = float(params["cond_max"])
    if params.get("target_count") is not None:
        args.target_count = int(params["target_count"])
    for name in ("xs", "ys", "zs"):
        if params.get(name) is not None:
            setattr(args, name, int(params[name]))
    if params.get("z_insert") is not None:
        args.z_insert = int(params["z_insert"])
    if params.get("robot") is not None:
        args.urdf = str(params["robot"])
    # ---- M2 新增：守卫 / 段检查 / 随机种子 / 预览开关（纯搬运 + 类型归一） ----
    if params.get("j5_min") is not None:
        args.j5_min = float(params["j5_min"])
    if params.get("j3_min") is not None:
        args.j3_min = float(params["j3_min"])
    if params.get("j5_attempts") is not None:
        args.j5_attempts = int(params["j5_attempts"])
    if params.get("seg_min") is not None:
        args.seg_min = float(params["seg_min"])
    if params.get("seg_mode") is not None:
        args.seg_mode = str(params["seg_mode"])
    if params.get("seg_samples") is not None:
        args.seg_samples = int(params["seg_samples"])
    if params.get("seed") is not None:
        args.seed = int(params["seed"])
    if params.get("q_seed") is not None:
        args.q_seed = _q_seed_text(params["q_seed"])
    if params.get("no_perturb") is not None:
        args.no_perturb = bool(params["no_perturb"])
    if params.get("skip_ik") is not None:
        args.skip_ik = bool(params["skip_ik"])
    if params.get("strict") is not None:
        args.strict = bool(params["strict"])
    if params.get("cond_position") is not None:
        args.cond_position = bool(params["cond_position"])
    if params.get("no_cond_check") is not None:
        args.no_cond_check = bool(params["no_cond_check"])
    return args


def _q_seed_text(raw) -> str:
    """把 ``q_seed`` 的两种写法（逗号串 / 6 个数）归一成 CLI 的逗号串。

    只做类型归一，**不校验**个数与数值——那一步交给真正的
    ``aabb_snake_scan._parse_q_seed``（见 :func:`_parsed_q_seed`）。

    Args:
        raw: ``"43,-30,-36,42,-89,-1"`` 或 ``[43, -30, -36, 42, -89, -1]``。

    Returns:
        str: 逗号分隔文本（列表元素用 ``repr`` 级精度，不丢位）。
    """
    if isinstance(raw, str):
        return raw.strip()
    return ",".join(str(float(value)) for value in raw)


def _validated_namespace(params: dict) -> tuple[argparse.Namespace, list[str]]:
    """归一化 + 校验表单参数（唯一一套校验：CLI 的 ``_validate_args``）。

    ``_validate_args`` 收到的 parser 与 CLI 完全相同（``prog`` / 报错措辞
    一致），只是把 ``parser.error`` 从「打印 usage 后 ``SystemExit(2)``」
    换成抛 :class:`_FormError`，异常消息仍是 argparse 拿到的那段原文。

    Args:
        params: 表单值，见 :func:`form_to_namespace`。

    Returns:
        ``(namespace, hints)``：namespace 里的 ``x_min/x_max/y_min/y_max/
        z_min/z_max``、``xs/ys/zs/z_insert``、``rpy``、``perturb_deg``、
        ``euler_convention`` 均已归一化；``hints`` 是校验过程中 CLI 打到
        stdout 的提示行（如“自动凑数: --target-count 50 -> …”、
        “警告: … 轴只有 1 个取值”）。

    Raises:
        _FormError: 参数非法，异常消息即 CLI 的报错原文。
    """
    args = form_to_namespace(params)
    parser = scan.build_arg_parser()

    def _raise_error(message: str) -> None:
        raise _FormError(str(message))

    parser.error = _raise_error  # type: ignore[method-assign]

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        scan._validate_args(parser, args)
    hints = [line.strip() for line in buffer.getvalue().splitlines()
             if line.strip()]
    return args, hints


def _failure(message: str) -> dict:
    """校验失败的标准返回（形状固定，前端只认这三种键）。"""
    return {"ok": False, "error": message, "warnings": []}


def preview_domain(params: dict) -> dict:
    """只算网格与点数，不跑 IK。

    流程：``form_to_namespace`` → ``_validate_args``（内含 ``_resolve_aabb``
    归一化与 ``_resolve_grid``；后者在给 ``target_count`` 时调 ``choose_grid``）
    → ``grid_point_count`` → 体积 → ``design_condition_number``。

    Args:
        params: 表单值（键与生成页一致）：

            * ``point_min`` / ``point_max``：两点式，各 ``[x, y, z]``（mm）；
            * ``aabb6``：六数式，``[x_min, x_max, y_min, y_max, z_min, z_max]``；
              与两点式互斥（由 ``_validate_args`` 判定）；
            * ``rpy``：参考姿态 ``[a, b, c]``（deg）；
            * ``euler``：欧拉约定，取值见 ``scan.EULER_CONVENTIONS``；
            * ``perturb_deg``：三轴扰动半宽 ``[a, b, c]``（deg）；
            * ``target_count``：目标点数；与 ``xs/ys/zs/z_insert`` 互斥；
            * ``xs`` / ``ys`` / ``zs`` / ``z_insert``：手填网格份数；
            * ``cond_max``：条件数上限（有限且 >= 1）；
            * ``robot``：机器人模型（本阶段只作回显，不加载）。

    Returns:
        dict: 成功时
        ``{"ok": True, "bounds": [x_min, x_max, y_min, y_max, z_min, z_max],
        "volume_m3": float, "grid": {"xs": int, "ys": int, "zs": int,
        "z_insert": int}, "point_count": int, "design_condition": float,
        "convention": str, "warnings": [str, ...]}``；
        ``convention`` 是本次实际使用的欧拉约定名（如 ``xyz-extrinsic``），
        ``warnings`` 是校验阶段 CLI 的提示行。
        校验失败返回 ``{"ok": False, "error": 报错原文, "warnings": []}``，
        不抛异常。
    """
    try:
        args, hints = _validated_namespace(params or {})
    except _FormError as error:
        return _failure(str(error))
    except SystemExit:                 # 兜底：error 已被替换，正常走不到
        return _failure("参数校验失败（argparse 提前退出）")
    except (TypeError, ValueError) as error:
        return _failure(f"参数无法解析：{error}")

    # _resolve_aabb 已把两种输入归一化到 x_min..z_max，这里是唯一来源。
    bounds = [float(args.x_min), float(args.x_max),
              float(args.y_min), float(args.y_max),
              float(args.z_min), float(args.z_max)]
    volume_mm3 = ((args.x_max - args.x_min)
                  * (args.y_max - args.y_min)
                  * (args.z_max - args.z_min))
    # _resolve_grid 已归一化网格；target_count 模式下 choose_grid 已在其中执行。
    grid = {"xs": int(args.xs), "ys": int(args.ys), "zs": int(args.zs),
            "z_insert": int(args.z_insert or 0)}
    point_count = scan.grid_point_count(grid["xs"], grid["ys"], grid["zs"],
                                        grid["z_insert"])
    design = scan.design_condition_number(
        args.perturb_deg, args.rpy,
        order=scan._order_of(args), extrinsic=scan._extrinsic_of(args))

    return {"ok": True,
            "bounds": bounds,
            "volume_m3": round(volume_mm3 / _MM3_PER_M3, 3),
            "grid": grid,
            "point_count": int(point_count),
            "design_condition": float(design["condition_number"]),
            "convention": args.euler_convention,
            "warnings": hints}


def list_robots(repo_root=None, *, probe: bool = True) -> list[dict]:
    """列出 ``data/configs`` 下的候选模型，并标记**能否真的被加载**。

    递归扫描（``robot_sim/`` 之类的子目录也列，label 里带相对路径），按
    扩展名标 ``kind``；不硬编码任何文件名。

    ``data/configs`` 里的文件并不都能当模型用：``*_dh.json`` 是 DH 覆盖文件、
    ``*_scan.json`` 是扫描记录、``robot_sim/*.json`` 是仿真器资产，它们都缺少
    ``Link&CoupleCoe``，喂给 ``Nexbot6R`` 会 ``KeyError``。所以这里用
    ``Nexbot6R`` 逐个试加载，结果写进 ``usable`` / ``reason``——下拉框只列
    可用的，其余在页面上注明，避免选了个跑不起来的文件到 M2 才炸。

    Args:
        repo_root: 仓库根；``None`` 表示本文件所在的仓库。
        probe: 是否真加载一次做可用性判定（``False`` 时 ``usable`` 恒为
            ``True``，仅供只想要文件清单的场合）。

    Returns:
        list[dict]: ``[{"label": 展示名, "path": 相对仓库根的路径,
        "kind": "urdf"|"json", "usable": bool, "reason": str|None}, ...]``，
        按 ``path`` 稳定排序；``label`` 是相对 ``data/configs`` 的路径。
    """
    root = _REPO_ROOT if repo_root is None else Path(repo_root).expanduser().resolve()
    config_dir = root / CONFIG_DIR
    robots: list[dict] = []
    for suffix, kind in _MODEL_SUFFIXES:
        for path in config_dir.rglob(f"*{suffix}"):
            if not path.is_file():
                continue
            usable, reason = True, None
            if probe:
                usable, reason = _probe_model(path)
            robots.append({"label": path.relative_to(config_dir).as_posix(),
                           "path": path.relative_to(root).as_posix(),
                           "kind": kind, "usable": usable, "reason": reason})
    robots.sort(key=lambda item: item["path"])
    return robots


def _probe_model(path: Path) -> tuple[bool, str | None]:
    """试加载一次，返回 ``(是否可当模型, 失败原因)``。"""
    from robot_model.nexbot_ik import Nexbot6R

    try:
        Nexbot6R(str(path))
    except Exception as error:          # noqa: BLE001 - 任意失败都只算"不可用"
        return False, f"{type(error).__name__}: {error}"
    return True, None


# ---------------------------------------------------------------------------
# M2：完整解算（plan -> perturb -> IK -> 段检查 -> 条件数 -> 落盘）
# ---------------------------------------------------------------------------
#: 进度阶段名（顺序即执行顺序，页面进度条直接用这些中文短语）。
STAGES = ("生成路点", "采样姿态", "解算 IK", "段内检查", "条件数", "出图", "写文件")
#: 段内奇异的插补模型（"cartesian" 对应现场 MOVL，CLI 的 --seg-mode）。
SEG_MODES = ("cartesian", "joint")
#: 产物文件名（写进 ``runs/<id>/``，也是 ``result["artifacts"]`` 的值）。
ARTIFACT_SCAN_JSON = "scan.json"
ARTIFACT_JOINTS = "joints.txt"
ARTIFACT_PLOT = "plot.png"


class _StageReporter:
    """进度回调的薄包装：没传回调时是个空壳，调用处不用写 ``if``。"""

    def __init__(self, callback=None) -> None:
        """Args:
            callback: ``on_progress(stage, index, total)`` 或 ``None``。
        """
        self._callback = callback

    def __call__(self, stage: str, index: int = 1, total: int = 1) -> None:
        """上报一个阶段进度（异常由调用方/上层处理，这里不吞）。"""
        if self._callback is not None:
            self._callback(str(stage), int(index), int(total))


def _parsed_q_seed(args: argparse.Namespace) -> list[float]:
    """解析 IK 种子关节角（复用 CLI 的 ``_parse_q_seed``）。

    Args:
        args: 已归一化的 Namespace（``args.q_seed`` 是逗号串）。

    Returns:
        list[float]: 6 个 deg 值。

    Raises:
        _FormError: 个数/格式不对，消息即 CLI 的报错原文。
    """
    parser = scan.build_arg_parser()

    def _raise_error(message: str) -> None:
        raise _FormError(str(message))

    parser.error = _raise_error          # type: ignore[method-assign]
    return scan._parse_q_seed(parser, args.q_seed)


def _check_seg_mode(args: argparse.Namespace) -> None:
    """``seg_mode`` 只认 ``cartesian`` / ``joint``（其余是调用方的类型错误）。"""
    if args.seg_mode not in SEG_MODES:
        raise _FormError(f"seg_mode 需为 {' 或 '.join(SEG_MODES)}，"
                         f"当前为 {args.seg_mode!r}")


def _check_cancel(job) -> None:
    """阶段边界检查取消标志；已取消则抛 :class:`jobs.JobCancelled`。

    IK 内部不检查（那要么复制 ``solve_chain_guarded`` 的循环、要么改算法），
    所以取消的最坏延迟是一个阶段——这与设计文档 §7.7「阶段边界检查」一致。
    """
    if job is not None and job.cancel.is_set():
        raise jobs.JobCancelled(f"任务 {job.id} 已取消")


def _line_min_frac(first: float, second: float) -> float:
    """线段上 ``|值|`` 最小的参数 ``t``（0..1，含端点）。

    Args:
        first: 起点值（如 J3 起点关节角）。
        second: 终点值。

    Returns:
        float: ``t``；两端同号时取绝对值小的一端，异号时取穿零点。
    """
    if first == second:
        return 0.0
    if (first < 0.0 < second) or (second < 0.0 < first):
        return first / (first - second)
    return 0.0 if abs(first) <= abs(second) else 1.0


def _worst_point(waypoints, steps_by_index, segment) -> tuple:
    """段内 ``|J3|`` 最小的位置（给段诊断表的「最差点」列）。

    算法只回报段内最小值、不回报取到最小值的位置（``check_segments`` /
    ``check_segments_cartesian`` 的返回值里没有 t 与坐标），但又不能为了这个
    数字去复制一遍采样：这里只在**关节线性插值**这条线上解一元问题得到
    ``t``，再按位置线性插值得到坐标。

    * MOVL（cartesian）模式：位置本来就是线性插值，坐标就是段上的点；
    * PTP（joint）模式：位置是同一参数下的线性近似，仅用于在图上指个大概。

    Args:
        waypoints: 路点列表（按下标取 ``pos``）。
        steps_by_index: ``{index: ChainStep}``。
        segment: ``check_segments*`` 返回的单段 dict（含 ``from`` / ``to``）。

    Returns:
        ``(t, [x, y, z])``；两端路点缺解时返回 ``(None, None)``。
    """
    first = steps_by_index.get(int(segment["from"]))
    second = steps_by_index.get(int(segment["to"]))
    if first is None or second is None or first.q is None or second.q is None:
        return None, None
    frac = _line_min_frac(float(first.q[2]), float(second.q[2]))
    start = waypoints[int(segment["from"])].pos
    end = waypoints[int(segment["to"])].pos
    position = [float(a) + (float(b) - float(a)) * frac
                for a, b in zip(start, end)]
    return frac, position


def _write_joints(path: Path, chain) -> int:
    """写 ``joints.txt``：每行 6 个关节角（deg，**空格分隔**，3 位小数，GP 顺序）。

    分隔符与已交付的 ``position_R1_scan50_joints.txt`` 保持一致（空格），
    这样两份文件可以直接 diff。

    只写有解的点——GP 程序里没法下发一个「无解」行，缺的点在 ``warnings``
    里单独报。

    Args:
        path: 输出路径。
        chain: ``ChainStep`` 序列（GP 顺序）。

    Returns:
        int: 写出的行数（= 有解路点数）。
    """
    lines = [" ".join(f"{float(value):.3f}" for value in step.q)
             for step in chain if step.q is not None]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    return len(lines)


def _scan_payload(args: argparse.Namespace, waypoints, guard_report, segment_report,
                  conditioning_json, chain) -> dict:
    """组装 ``scan.json`` 的内容（字段与 CLI 输出逐项一致）。

    本函数只是把算法返回值搬进 CLI ``_run_ik_chain`` 的同一套结构：``meta`` /
    ``waypoints`` / ``singularity_guard`` / ``segment_check`` / ``conditioning``
    / ``perturbed_ik``，浮点统一 ``round(6)``，所以同参数下与 CLI 写出的
    JSON 逐字节相同（``--skip-ik`` 时多三个空键，见模块注释）。

    Args:
        args: 归一化后的 Namespace。
        waypoints: ``Waypoint`` 序列。
        guard_report: ``solve_chain_guarded`` 的守卫报告或 ``None``。
        segment_report: 段检查报告或 ``None``（预览模式）。
        conditioning_json: ``_conditioning_to_json`` 的结果。
        chain: ``ChainStep`` 序列（预览模式为空列表）。

    Returns:
        dict: 可直接 ``json.dump`` 的字典。
    """
    skip_ik = bool(args.skip_ik)
    return {
        "meta": {
            "aabb": scan._round6([args.x_min, args.x_max,
                                  args.y_min, args.y_max,
                                  args.z_min, args.z_max]),
            "rpy": scan._round6(args.rpy),
            "xs": int(args.xs),
            "ys": int(args.ys),
            "zs": int(args.zs),
            "z_insert": int(getattr(args, "z_insert", 0)),
            "motion_order": scan.MOTION_ORDER,
            "grid_source": getattr(args, "grid_source", "explicit"),
            "target_count": getattr(args, "target_count", None),
            "aabb_source": getattr(args, "aabb_source", "axes"),
            "seed": int(args.seed),
            "euler_convention": getattr(args, "euler_convention",
                                        "xyz-extrinsic"),
            "skip_ik": skip_ik,
            "no_perturb": bool(args.no_perturb),
            "perturbed": (not bool(args.no_perturb)) and not skip_ik,
        },
        "waypoints": [
            {
                "index": wp.index,
                "pos": scan._round6(wp.pos),
                "rpy_ref": scan._round6(wp.rpy_ref),
                "rpy_pert": None if wp.rpy_pert is None
                            else scan._round6(wp.rpy_pert),
                "delta_rpy": None if wp.delta_rpy is None
                             else scan._round6(wp.delta_rpy),
            }
            for wp in waypoints
        ],
        "singularity_guard": guard_report,
        "segment_check": segment_report,
        "conditioning": conditioning_json,
        "perturbed_ik": [
            {
                "index": step.index,
                "q": None if step.q is None else scan._round6(step.q),
                "config": None if step.config is None
                          else [int(c) for c in step.config],
                "dq_l2": None if step.dq_l2 is None else round(step.dq_l2, 6),
                "dq_l1": None if step.dq_l1 is None else round(step.dq_l1, 6),
                "n_sols": int(step.n_sols),
                "ok": bool(step.ok),
            }
            for step in chain
        ],
    }


def _waypoint_rows(waypoints, chain) -> list[dict]:
    """把路点与选解并成前端表格/3D 图用的行。"""
    steps = {step.index: step for step in chain}
    rows = []
    for waypoint in waypoints:
        step = steps.get(waypoint.index)
        q = None if step is None or step.q is None else [float(v) for v in step.q]
        rows.append({
            "gp": int(waypoint.index) + 1,
            "index": int(waypoint.index),
            "pos": [float(v) for v in waypoint.pos],
            "rpy_pert": (None if waypoint.rpy_pert is None
                         else [float(v) for v in waypoint.rpy_pert]),
            "q": q,
            "j3_abs": None if q is None else abs(q[2]),
            "j5_abs": None if q is None else abs(q[4]),
            "config": (None if step is None or step.config is None
                       else [int(c) for c in step.config]),
            "dq_l2": None if step is None or step.dq_l2 is None else float(step.dq_l2),
            "ok": bool(step is not None and step.ok),
        })
    return rows


def _segment_rows(waypoints, chain, segment_report) -> list[dict]:
    """把段检查的每段转成前端表格用的行（GP 从 1 起算）。

    算法返回值里没有「最差点」，位置由 :func:`_worst_point` 按段内 ``|J3|``
    最小解析求出（见该函数的说明）。
    """
    if not segment_report:
        return []
    steps = {step.index: step for step in chain}
    rows = []
    for segment in segment_report.get("segments", []):
        frac, position = _worst_point(waypoints, steps, segment)
        rows.append({
            "gp_from": int(segment["from"]) + 1,
            "gp_to": int(segment["to"]) + 1,
            "j3_abs_min": segment.get("j3_abs_min"),
            "j5_abs_min": segment.get("j5_abs_min"),
            "crosses": bool(segment.get("j3_crosses_zero")
                            or segment.get("j5_crosses_zero")),
            "worst_frac": frac,
            "worst_pos": position,
        })
    return rows


def _scan_warnings(args: argparse.Namespace, warnings: list[str], chain,
                   guard_report, segment_report, conditioning_json) -> None:
    """把 CLI 打印的告警整理成列表（前端逐条显示，措辞与 CLI 对齐）。"""
    ok_steps = [step for step in chain if step.ok]
    if args.skip_ik:
        warnings.append("预览模式：不跑 IK（q / 统计留空，等价 CLI --skip-ik）")
        return
    missing = len(chain) - len(ok_steps)
    if missing:
        warnings.append(f"{missing} 个路点无解（未通过 FK 自校验）")
    if guard_report and guard_report["violations"]:
        bad = ", ".join(f"#{item['index']}" for item in guard_report["violations"])
        warnings.append(f"重采后仍低于奇异阈值: {bad}")
    if segment_report is None:
        return
    if segment_report["j3_crossings"] or segment_report["j5_crossings"]:
        warnings.append(
            f"段内奇异: J3 穿 0 的段 {segment_report['j3_crossings']} 处，"
            f"J5 穿 0 的段 {segment_report['j5_crossings']} 处")
    elif segment_report["j3_abs_min"] is not None:
        lowest_values = [value for value in (segment_report["j3_abs_min"],
                                            segment_report["j5_abs_min"])
                         if value is not None]
        if lowest_values and min(lowest_values) < args.seg_min:
            warnings.append(f"段内最小值 {min(lowest_values):.3f} 低于 "
                            f"seg_min {args.seg_min:g}")
    if conditioning_json["status"] == "rejected":
        warnings.append(f"姿态激励条件数未达标：门禁 "
                        f"{conditioning_json['gate']['value']} > 阈值 "
                        f"{conditioning_json['threshold']:g}")


def _summary_stats(args: argparse.Namespace, conditioning_json, chain,
                   guard_report, segment_report) -> dict:
    """把解算结果压成前端统计卡的字段（``run_scan`` 与 ``run_repair`` 共用）。

    M4 的修复页要并排展示「修复前 / 修复后」两份统计，所以这段计算从
    :func:`_execute` 提出来做成纯函数；逻辑与 CLI 的汇总一致。

    Args:
        args: 归一化后的 Namespace（用 ``skip_ik`` 判断要不要算 ``feasible``）。
        conditioning_json: ``_conditioning_to_json`` 的结果。
        chain: ``ChainStep`` 序列。
        guard_report: ``solve_chain_guarded`` 的守卫报告或 ``None``。
        segment_report: 段检查报告或 ``None``。

    Returns:
        dict: ``solved`` / ``total`` / ``switches`` / ``dq_l2_sum`` /
        ``j5_abs_min`` / ``j3_abs_min`` / ``design_cond`` / ``post_ik_cond`` /
        ``seg_j5_min`` / ``seg_j3_min`` / ``j3_crossings`` / ``j5_crossings`` /
        ``guard_violations``（守卫重采后仍低于阈值的点数）/ ``feasible``
        （``skip_ik`` 时为 ``None``，表示「没跑 IK 就没有可行性」）。
    """
    skip_ik = bool(getattr(args, "skip_ik", False))
    ok_steps = [step for step in chain if step.ok]
    switches = sum(1 for first, second in zip(ok_steps, ok_steps[1:])
                   if first.config != second.config)
    dq_l2_sum = float(sum(step.dq_l2 for step in ok_steps))
    if guard_report is not None:
        j5_abs_min = guard_report["j5_abs_min_deg"]
        j3_abs_min = guard_report["j3_abs_min_deg"]
    else:
        j5_abs_min = min((abs(step.q[4]) for step in ok_steps), default=None)
        j3_abs_min = min((abs(step.q[2]) for step in ok_steps), default=None)
    design = conditioning_json["design"]["condition_number"]
    post_ik = conditioning_json["post_ik"]
    post_ik_cond = None if post_ik is None else post_ik["condition_number"]
    seg_j5_min = None if segment_report is None else segment_report["j5_abs_min"]
    seg_j3_min = None if segment_report is None else segment_report["j3_abs_min"]
    j3_crossings = 0 if segment_report is None else segment_report["j3_crossings"]
    j5_crossings = 0 if segment_report is None else segment_report["j5_crossings"]
    violations = 0 if guard_report is None else len(
        guard_report.get("violations") or [])
    feasible = None
    if not skip_ik:
        # 判据必须与 avoid_singularity.evaluate_domain 一致：守卫重采后仍低于
        # 阈值的点、段内裕度不足、段内穿零、条件数被拒，任一条都算不可行。
        # 早先只看「有解 + 无穿零 + 条件数」，会出现卡片绿"可行"而算法判不可行。
        feasible = (bool(ok_steps) and len(ok_steps) == len(chain)
                    and violations == 0
                    and j3_crossings == 0 and j5_crossings == 0
                    and conditioning_json["status"] != "rejected")
    return {"solved": len(ok_steps), "total": len(chain),
            "switches": switches, "dq_l2_sum": dq_l2_sum,
            "j5_abs_min": j5_abs_min, "j3_abs_min": j3_abs_min,
            "design_cond": design, "post_ik_cond": post_ik_cond,
            "seg_j5_min": seg_j5_min, "seg_j3_min": seg_j3_min,
            "j3_crossings": j3_crossings, "j5_crossings": j5_crossings,
            "guard_violations": violations, "feasible": feasible}


def _execute(args: argparse.Namespace, q_seed: list[float], job,
             progress: _StageReporter, warnings: list[str]) -> dict:
    """``run_scan`` 的主体（校验已通过，产物目录在这里建）。

    **界面阶段**（``STAGES``）的顺序与设计文档 §8 的 M2 任务项一致：规划路点 ->
    采样扰动 -> 链式 IK -> 段内检查 -> 条件数 -> 出图 -> 写文件；每两个阶段
    之间检查一次取消标志。**计算**顺序按 CLI：设计条件数在链式 IK **之前**
    算（见下方 2.5 的注释），它的阶段名仍在段检查之后上报。
    """
    from robot_model.nexbot_ik import Nexbot6R

    run_id, run_dir = ((str(job.id), Path(job.run_dir)) if job is not None
                       else jobs.make_run_dir())
    order = scan._order_of(args)
    extrinsic = scan._extrinsic_of(args)
    rpy_ref = tuple(float(value) for value in args.rpy)
    skip_ik = bool(args.skip_ik)

    rbt = Nexbot6R(_resolve_model_path(args.urdf))

    # ---- 1. 路点生成 ----
    _check_cancel(job)
    progress(STAGES[0], 0, 1)
    waypoints = scan.plan_waypoints(
        args.x_min, args.x_max, args.y_min, args.y_max,
        args.z_min, args.z_max, args.xs, args.ys, args.zs, rpy_ref,
        z_insert=int(getattr(args, "z_insert", 0)))
    progress(STAGES[0], 1, 1)

    def _poses(getter):
        """由路点姿态构造位姿序列（欧拉约定由表单决定）。"""
        return [rbt.pose_to_T(waypoint.pos, getter(waypoint), order=order,
                              extrinsic=extrinsic) for waypoint in waypoints]

    # ---- 2. 姿态扰动采样（预览模式不采样，字段保持 None） ----
    poses = None
    if not skip_ik:
        if args.no_perturb:
            poses = _poses(lambda waypoint: waypoint.rpy_ref)
        else:
            _check_cancel(job)
            progress(STAGES[1], 0, 1)
            scan._sample_perturbations(waypoints, args.seed, args.perturb_deg)
            progress(STAGES[1], 1, 1)
            poses = _poses(lambda waypoint: waypoint.rpy_pert)

    # ---- 2.5 姿态激励条件数（设计值 + 实测值） ----
    # CLI 的顺序是「采样 -> evaluate_conditioning -> 链式 IK」，这里的 realized
    # 必须用**重采之前**的位姿；守卫重采后再算会得到另一批 poses，scan.json 的
    # conditioning.realized 就与 CLI 不一致（守卫没重采时恰好看不出来）。
    # 这一步没有同步的阶段进度：它一毫秒级就完事，而 STAGES[4]（条件数）在
    # 阶段表里排在段检查之后，此时上报会让进度条先跳到「条件数」再退回
    # 「解算 IK」。stage 名只用于显示，计算顺序按 CLI。
    _check_cancel(job)
    conditioning = scan.evaluate_conditioning(args, waypoints, rpy_ref, poses)

    # ---- 3. 链式 IK 选解（带奇异守卫） ----
    chain: list = []
    guard_report = None
    if not skip_ik:
        _check_cancel(job)
        total_waypoints = len(waypoints)
        progress(STAGES[2], 0, total_waypoints)
        if args.j5_min > 0.0 or args.j3_min > 0.0:
            chain, guard_report = scan.solve_chain_guarded(
                rbt, waypoints, q_seed, j5_min=args.j5_min, j3_min=args.j3_min,
                attempts=args.j5_attempts, seed=args.seed,
                amplitudes_deg=args.perturb_deg, order=order,
                extrinsic=extrinsic, strict=args.strict)
            # 守卫可能重采过路点姿态：位姿必须以最终姿态重建（CLI 同样处理）。
            poses = _poses(lambda waypoint: waypoint.rpy_pert)
        else:
            chain = scan.solve_chain(rbt, poses, q_seed, strict=args.strict)
        progress(STAGES[2], total_waypoints, total_waypoints)

    # ---- 4. 段内奇异检查（端点安全 != 路径安全） ----
    segment_report = None
    if not skip_ik:
        _check_cancel(job)
        segment_total = max(0, len(chain) - 1)
        progress(STAGES[3], 0, segment_total)
        if args.seg_mode == "cartesian":
            segment_report = scan.check_segments_cartesian(
                rbt, waypoints, chain, samples=args.seg_samples,
                order=order, extrinsic=extrinsic)
        else:
            segment_report = scan.check_segments(chain)
        progress(STAGES[3], segment_total, segment_total)

    # ---- 5. IK 后的实测条件数（只用成功子集，CLI 同样只报告） ----
    _check_cancel(job)
    progress(STAGES[4], 0, 2)
    ok_poses = [poses[step.index] for step in chain if step.ok]
    if ok_poses:
        info = scan.pose_condition_number(
            ok_poses, include_position=args.cond_position,
            reference=scan._reference_rotation(args, rpy_ref))
        conditioning["post_ik"] = {
            "condition_number": info["condition_number"],
            "singular_values": info["singular_values"],
            "n_points": info["n_points"],
            "blocks": info["blocks"],
            "metric": info["metric"],
        }
    conditioning_json = scan._conditioning_to_json(conditioning)
    progress(STAGES[4], 2, 2)

    # ---- 6. 预览图（matplotlib Agg；画完立刻 close） ----
    _check_cancel(job)
    progress(STAGES[5], 0, 1)
    ok_steps = [step for step in chain if step.ok]
    plot_target = run_dir / ARTIFACT_PLOT
    if skip_ik:
        ok_flags = [True] * len(waypoints)
        title = f"waypoints {len(waypoints)}/{len(waypoints)}"
    else:
        ok_flags = [step.ok for step in chain]
        title = f"IK chain {len(ok_steps)}/{len(chain)}"
    scan.plot_path([waypoint.pos for waypoint in waypoints], ok_flags,
                   str(plot_target), title=title)
    import matplotlib.pyplot as plt
    plt.close("all")
    progress(STAGES[5], 1, 1)

    # ---- 7. 落盘：scan.json / joints.txt ----
    _check_cancel(job)
    progress(STAGES[6], 0, 2)
    payload = _scan_payload(args, waypoints, guard_report, segment_report,
                            conditioning_json, chain)
    with open(run_dir / ARTIFACT_SCAN_JSON, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2,
                  allow_nan=False)
    progress(STAGES[6], 1, 2)
    artifacts = {"scan_json": ARTIFACT_SCAN_JSON, "plot": ARTIFACT_PLOT}
    if chain and any(step.q is not None for step in chain):
        written = _write_joints(run_dir / ARTIFACT_JOINTS, chain)
        artifacts["joints"] = ARTIFACT_JOINTS
        if written < len(chain):
            warnings.append(f"joints.txt 只写了 {written}/{len(chain)} 行"
                            f"（无解路点无法下发）")
    progress(STAGES[6], 2, 2)

    # ---- 8. 统计 / 行数据 / 收尾 ----
    _scan_warnings(args, warnings, chain, guard_report, segment_report,
                   conditioning_json)
    stats = _summary_stats(args, conditioning_json, chain, guard_report,
                           segment_report)
    return {"ok": True,
            "run_id": run_id,
            "run_dir": str(run_dir),
            "stats": stats,
            "meta": payload["meta"],
            "waypoints": _waypoint_rows(waypoints, chain),
            "segments": _segment_rows(waypoints, chain, segment_report),
            "artifacts": artifacts,
            "warnings": warnings}


def run_scan(params: dict, job=None, on_progress=None) -> dict:
    """生成路点并解算（M2 的主入口，**不抛异常**，错误走 ``ok: False``）。

    与 CLI ``python -m excitation_trajectory.aabb_snake_scan`` 走**同一条**算法
    路径：参数归一化与校验交给 ``_validate_args``，路点、扰动、IK、段检查、
    条件数、出图全部 import 复用，本函数只负责串流程、报进度、落盘。同参数下
    ``runs/<id>/scan.json`` 与 CLI 写出的 JSON 逐字节一致。

    Args:
        params: 表单值（键见 :func:`form_to_namespace`）。M2 新增
            ``j5_min`` / ``j3_min`` / ``seg_min`` / ``seg_mode`` /
            ``seg_samples`` / ``seed`` / ``q_seed``（逗号串或 6 个数列表）/
            ``j5_attempts`` / ``no_perturb``；另有 ``skip_ik``（只出预览图）、
            ``strict`` / ``cond_position`` / ``no_cond_check``。
        job: ``jobs.Job``（可选）。给出时用它已有 ``run_dir`` / ``id``，并在
            每个阶段边界检查 ``job.cancel``；``None`` 时自己新建
            ``runs/<id>/``。
        on_progress: ``on_progress(stage, index, total)`` 或 ``None``；阶段名见
            :data:`STAGES`（``生成路点`` / ``采样姿态`` / ``解算 IK`` /
            ``段内检查`` / ``条件数`` / ``出图`` / ``写文件``）。

    Returns:
        dict: 成功时 ``{"ok": True, "run_id", "run_dir", "stats", "meta",
        "waypoints", "segments", "artifacts", "warnings"}``；校验失败或 strict
        中止时 ``{"ok": False, "error": 报错原文, "warnings": [...]}``。
        程序性错误（代码 bug）不在这里吞，交给 ``jobs`` 层记 ``failed``。
    """
    warnings: list[str] = []
    try:
        args, hints = _validated_namespace(params or {})
        _check_seg_mode(args)
        q_seed = _parsed_q_seed(args)
    except _FormError as error:
        return _failure(str(error))
    except SystemExit:                 # 兜底：error 已被替换，正常走不到
        return _failure("参数校验失败（argparse 提前退出）")
    except (TypeError, ValueError) as error:
        return _failure(f"参数无法解析：{error}")
    warnings.extend(hints)
    try:
        return _execute(args, q_seed, job, _StageReporter(on_progress), warnings)
    except SystemExit as exit_error:   # strict 模式下算法自己 sys.exit(2)
        return {"ok": False,
                "error": f"strict 模式中止（退出码 {exit_error.code}），"
                         f"详见任务日志",
                "warnings": warnings}


# ---------------------------------------------------------------------------
# M4：奇异修复（定位段 -> 局部修复 -> 整体搜索 -> 复核）
# ---------------------------------------------------------------------------
#: 修复流程在 M2 阶段名之外补充的阶段（``run_repair`` 的 on_progress 用）。
REPAIR_STAGES = ("定位段", "局部修复", "整体搜索", "复核")
#: 修复页独有的产物名（决策报告，字段对齐 CLI ``--report``）。
ARTIFACT_REPAIR = "repair.json"
#: 候选日志保留条数（前端表格不撑爆；超出的记在 ``log_total`` 里）。
REPAIR_LOG_LIMIT = 60
#: 修复专用字段（``avoid_singularity`` 的 CLI 才有，扫描页没有）。
_REPAIR_KEYS = ("seg_samples", "seg_samples_final", "segment_window",
                "avoid_margin", "avoid_radius", "repair_attempts",
                "max_candidates", "j5_attempts", "seg_min", "cond_max",
                "target_count", "seed", "perturb_deg")


@functools.lru_cache(maxsize=1)
def repair_cli_defaults() -> dict:
    """修复流程的默认值：**问修复 CLI 要**，不在界面里另抄数字。

    ``aabb_snake_scan`` 的解析器里没有 ``--avoid-margin`` / ``--avoid-radius``
    / ``--repair-attempts`` / ``--seg-samples-final``；而 ``--seg-samples`` 两个
    CLI 的默认值**故意不同**（扫描 21，修复 11，因为修复要跑上百次候选）。
    修复页要跟的是修复 CLI，所以这里让真正的解析器解析一份哑参数，取回它自己
    的默认值——与 :func:`form_to_namespace` 的做法一致。

    Returns:
        dict: ``{dest: default}``，键见 :data:`_REPAIR_KEYS`。
    """
    argv = ["--robot", "unused.urdf", "--point-min", "0", "0", "0",
            "--point-max", "1", "1", "1",
            "--singular-q", "0", "0", "0", "0", "0", "0"]
    args = avoid.build_arg_parser().parse_args(argv)
    return {key: getattr(args, key) for key in _REPAIR_KEYS}


def _singular_groups(raw) -> list[list[float]]:
    """把表单里的奇异读数归一成 ``[[6 个角度], ...]``（deg）。

    接受三种写法：``[[6 个数], ...]``（页面）、``[6 个数]``（扁平列表）、
    ``"0,0,0,55.7,0,-55.7"`` 或 ``["0,0,0,..."]``（逗号串）。归一后交给
    :func:`avoid_singularity.flatten_joint_angles`，报错原文直接回显。

    Args:
        raw: 表单值（``None`` 表示没填）。

    Returns:
        list[list[float]]：每组 6 个 deg。

    Raises:
        _FormError: 没给、个数不是 6 的整数倍、或含非有限值。
    """
    if raw is None:
        raise _FormError("需要至少一组奇异读数（singular_q，6 个关节角/组）")
    if isinstance(raw, str):
        groups = [raw]
    else:
        items = list(raw)
        if not items:
            raise _FormError("需要至少一组奇异读数（singular_q，6 个关节角/组）")
        if all(isinstance(item, (int, float)) for item in items):
            groups = [items]                    # 扁平 6 个数 = 一组
        else:
            groups = [[item] if isinstance(item, str) else item
                      for item in items]
    try:
        return avoid.flatten_joint_angles(groups)
    except (TypeError, ValueError) as error:
        raise _FormError(str(error)) from error


def _stopped_at_value(raw):
    """把「已走到 GP 几」归一成 ``int`` 或 ``None``。

    Raises:
        _FormError: 不是整数（``41.5``、``"41a"`` 之类）。
    """
    if raw is None or raw == "":
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError) as error:
        raise _FormError(f"已走到的 GP 点号需为整数（当前 {raw!r}）") from error
    if not math.isfinite(value) or value != int(value):
        raise _FormError(f"已走到的 GP 点号需为整数（当前 {raw!r}）")
    return int(value)


def _repair_namespace(params: dict) -> tuple[argparse.Namespace, list[str]]:
    """修复流程的参数：``_validated_namespace`` + 修复专用字段。

    采样域 / 姿态 / 守卫那套走的是扫描页的同一条校验（``_validate_args``），
    这里只补修复流程独有的字段，并把两个页面复选框映射成 CLI 的开关：

    * ``allow_rpy``（默认 ``True``）→ ``no_rpy_change = not allow_rpy``；
    * ``allow_shrink``（默认 ``True``）→ ``no_shrink = not allow_shrink``。

    Args:
        params: 表单值，键见 :func:`form_to_namespace`，另加 ``singular_q`` /
            ``stopped_at`` / ``segment_window`` / ``avoid_margin`` /
            ``avoid_radius`` / ``repair_attempts`` / ``seg_samples_final`` /
            ``max_candidates`` / ``allow_rpy`` / ``allow_shrink``。

    Returns:
        ``(namespace, hints)``。

    Raises:
        _FormError: 参数非法（CLI 报错原文，或修复专用字段的明确说明）。
    """
    args, hints = _validated_namespace(params or {})
    defaults = repair_cli_defaults()
    values = params or {}

    def number(key: str, *, integer: bool, minimum, label: str):
        """取一个修复专用数值（缺项/空串 -> CLI 默认值）。"""
        raw = values.get(key)
        if raw is None or raw == "":
            raw = defaults[key]
        try:
            value = int(raw) if integer else float(raw)
        except (TypeError, ValueError) as error:
            kind = "整数" if integer else "数值"
            raise _FormError(f"{label} 需为{kind}（当前 {values.get(key)!r}）"
                             ) from error
        if not math.isfinite(float(value)):
            raise _FormError(f"{label} 需为有限值（当前 {value}）")
        if minimum is not None and value < minimum:
            raise _FormError(f"{label} 需 >= {minimum}（当前 {value}）")
        return value

    args.seg_samples = number("seg_samples", integer=True, minimum=2,
                              label="每段采样点 seg_samples")
    args.seg_samples_final = number("seg_samples_final", integer=True,
                                    minimum=2,
                                    label="复核采样点 seg_samples_final")
    args.segment_window = number("segment_window", integer=True, minimum=0,
                                 label="前后各看几段 segment_window")
    args.avoid_margin = number("avoid_margin", integer=False, minimum=0.0,
                               label="经验裕度附加 avoid_margin")
    args.avoid_radius = number("avoid_radius", integer=False, minimum=0.0,
                               label="位置禁区半径 avoid_radius")
    args.repair_attempts = number("repair_attempts", integer=True, minimum=1,
                                  label="局部修复尝试次数 repair_attempts")
    args.max_candidates = number("max_candidates", integer=True, minimum=1,
                                 label="候选上限 max_candidates")
    args.force_search = bool(values.get("force_search", False))
    allow_rpy = values.get("allow_rpy")
    args.no_rpy_change = ((not bool(allow_rpy)) if allow_rpy is not None
                          else bool(values.get("no_rpy_change", False)))
    allow_shrink = values.get("allow_shrink")
    args.no_shrink = ((not bool(allow_shrink)) if allow_shrink is not None
                      else bool(values.get("no_shrink", False)))
    args.stopped_at = _stopped_at_value(values.get("stopped_at"))
    args.singular_q = _singular_groups(values.get("singular_q"))
    return args, hints


def _repair_target_count(args: argparse.Namespace) -> int:
    """本次参与修复的路点总数（网格已在 ``_resolve_grid`` 里定下来）。"""
    return int(scan.grid_point_count(int(args.xs), int(args.ys), int(args.zs),
                                     int(getattr(args, "z_insert", 0) or 0)))


def _stopped_index(args: argparse.Namespace, target_count: int):
    """``stopped_at`` -> 0 基段起点下标；越界按 CLI 原文报错。

    Args:
        args: 归一化后的 Namespace（``stopped_at`` 可为 ``None``）。
        target_count: 路点总数。

    Returns:
        int | None：``None`` 表示没有现场点号，只做整链判断。

    Raises:
        _FormError: 点号落在 ``1..target_count-1`` 之外（提示与 CLI 一致）。
    """
    if args.stopped_at is None:
        return None
    if not 1 <= int(args.stopped_at) <= target_count - 1:
        raise _FormError(
            f"--stopped-at 需在 1..{target_count - 1} 之间"
            f"（它表示已走到的点，后面还得有一点；当前 {args.stopped_at}）")
    return int(args.stopped_at) - 1


def _repair_context(args: argparse.Namespace, q_seed: list[float]) -> dict:
    """构造修复流程的上下文：模型 + 原域 + 有效阈值 + 评估器。

    对应 CLI ``avoid_singularity.run()`` 的第 1、2 步（分类读数 -> 有效阈值 ->
    位置禁区），所有判定都调用 ``avoid_singularity`` 的函数，不重写。

    Args:
        args: :func:`_repair_namespace` 的结果。
        q_seed: 6 个 IK 种子关节角（deg）。

    Returns:
        dict：``rbt`` / ``order`` / ``extrinsic`` / ``base``（原域）/
        ``analyses``（每个奇异读数的分类）/ ``j5_eff`` / ``j3_eff`` /
        ``j3_seg_min`` / ``forbidden`` / ``target_count`` / ``evaluate``
        （``evaluate(domain, attempts, steps_override=None,
        waypoints_override=None, samples=None)``）。
    """
    from robot_model.nexbot_ik import Nexbot6R

    rbt = Nexbot6R(_resolve_model_path(args.urdf))
    order, extrinsic = scan._order_of(args), scan._extrinsic_of(args)
    bounds = [float(args.x_min), float(args.x_max),
              float(args.y_min), float(args.y_max),
              float(args.z_min), float(args.z_max)]
    base = avoid.make_domain(bounds, args.rpy,
                             (int(args.xs), int(args.ys), int(args.zs)),
                             int(getattr(args, "z_insert", 0) or 0))
    base["euler_convention"] = args.euler_convention

    analyses = [avoid.classify_singularity(
        rbt, q, j5_min=args.j5_min, j3_min=args.j3_min,
        avoid_margin=args.avoid_margin) for q in args.singular_q]
    j5_eff = max([float(args.j5_min)] + [a["j5_floor"] for a in analyses])
    j3_eff = max([float(args.j3_min)] + [a["j3_floor"] for a in analyses])
    # 报了肘/退化时，段内肘部也要有裕度（端点合格 != 路径合格），与 CLI 一致。
    elbow_required = any(a["kind"] in ("elbow", "degenerate") for a in analyses)
    j3_seg_min = max(float(args.seg_min), j3_eff) if elbow_required else 0.0
    forbidden = [a["position"] for a in analyses
                 if a["kind"] == "unexplained"]
    target_count = _repair_target_count(args)

    def evaluate(domain, attempts, steps_override=None,
                 waypoints_override=None, samples=None):
        """评估一个域（等价 CLI 里的 ``_evaluate``）。

        Args:
            domain: 采样域描述（``avoid.make_domain`` / ``clone_domain``）。
            attempts: 单点重试次数（粗筛用 ``SEARCH_ATTEMPTS``，复核用
                ``args.j5_attempts``）。
            steps_override: 已解算的链（局部修复后复核用，不重解）。
            waypoints_override: 局部修复后的路点（姿态已改写，不能重采样）。
            samples: 每段采样点数；``None`` 用 ``args.seg_samples``
                （复核时显式传 ``args.seg_samples_final``）。

        Returns:
            dict：``avoid_singularity.evaluate_domain`` 的返回值。
        """
        return avoid.evaluate_domain(
            rbt, domain, target_count=target_count, seed=args.seed,
            q_seed=q_seed, j5_min=j5_eff, j3_min=j3_eff,
            seg_min=args.seg_min, cond_max=args.cond_max,
            perturb_deg=args.perturb_deg, order=order, extrinsic=extrinsic,
            attempts=attempts, forbidden=forbidden,
            avoid_radius=args.avoid_radius,
            seg_samples=(args.seg_samples if samples is None else samples),
            j3_seg_min=j3_seg_min, steps_override=steps_override,
            waypoints_override=waypoints_override)

    return {"rbt": rbt, "order": order, "extrinsic": extrinsic,
            "base": base, "analyses": analyses, "j5_eff": j5_eff,
            "j3_eff": j3_eff, "j3_seg_min": j3_seg_min,
            "forbidden": forbidden, "target_count": target_count,
            "evaluate": evaluate}


def _profile_fails(profile: dict, args: argparse.Namespace,
                   j3_seg_min: float) -> bool:
    """这一段是否真的越界（CLI 里的 ``_profile_fails``，决定要不要动手）。"""
    if profile.get("j3_abs_min") is None or profile.get("j5_abs_min") is None:
        return False
    if profile.get("crossings") or profile.get("ik_failures"):
        return True
    if profile["j5_abs_min"] < args.seg_min:
        return True
    return bool(j3_seg_min > 0.0 and profile["j3_abs_min"] < j3_seg_min)


def _window_profiles(ctx: dict, baseline: dict, target_index: int,
                     args: argparse.Namespace) -> tuple[list[dict], list[dict]]:
    """窗口内各段的 MOVL 剖面，并标出 ``ok``（不越界）。

    Returns:
        ``(profiles, offenders)``：``offenders`` 是 ``profiles`` 里真的越界的
        那几段（CLI 用它决定修哪一段）。
    """
    # 原域可能因「位置禁区」在 evaluate_domain 里早退（steps/segments 为 None），
    # 此时没有任何链可做段剖面——直接返回空，让 verdict/warnings 讲清原因，
    # 而不是抛 "NoneType has no len()" 这种把真实原因埋掉的类型错误。
    if not baseline.get("steps") or not baseline.get("segments"):
        return [], []

    profiles = [dict(profile) for profile in avoid.segment_profiles(
        ctx["rbt"], baseline["waypoints"], baseline["steps"], target_index,
        window=args.segment_window, order=ctx["order"],
        extrinsic=ctx["extrinsic"], samples=args.seg_samples_final)]
    for profile in profiles:
        profile["ok"] = not _profile_fails(profile, args, ctx["j3_seg_min"])
    return profiles, [profile for profile in profiles if not profile["ok"]]


def _repair_index(profiles: list[dict], offenders: list[dict],
                  target_index: int):
    """要修哪一段：优先报告段，报告段安全就修窗口里最危险的（CLI 同规则）。"""
    if not offenders:
        return None
    reported = next((profile for profile in profiles
                     if profile["index"] == target_index), None)
    if reported is not None and not reported["ok"]:
        return target_index
    return min(offenders,
               key=lambda profile: min(profile["j3_abs_min"],
                                       profile["j5_abs_min"]))["index"]


def _chain_summary(segment_report) -> dict:
    """整链穿零概览（判定结论里要报"整条链另有 N 处穿零"）。"""
    if not segment_report:
        return {"j3_crossings": 0, "j5_crossings": 0, "bad_segments": [],
                "seg_j3_min": None, "seg_j5_min": None}
    bad = [f"GP{int(segment['from']) + 1}->GP{int(segment['to']) + 1}"
           for segment in segment_report.get("segments") or []
           if segment.get("j3_crosses_zero") or segment.get("j5_crosses_zero")]
    return {"j3_crossings": int(segment_report.get("j3_crossings") or 0),
            "j5_crossings": int(segment_report.get("j5_crossings") or 0),
            "bad_segments": bad,
            "seg_j3_min": segment_report.get("j3_abs_min"),
            "seg_j5_min": segment_report.get("j5_abs_min")}


def _verdict(baseline: dict, profiles: list[dict], offenders: list[dict],
             target_index, chain: dict) -> str:
    """一句话结论（措辞与 CLI 一致；无现场点号时另给一句）。

    四种情形：原域根本没生成链（位置禁区早退） / 整链无越界 / 窗口内无越界 /
    报告段或窗口内最危险段越界。
    """
    if not baseline.get("steps"):
        # 原域在出链之前就判不可行（典型是位置禁区）。此时没有任何段可定位，
        # 结论必须直说原因，而不是「窗口内各段都满足阈值」（那就成误导了）。
        reasons = "；".join(baseline.get("reasons") or []) or "未知原因"
        return f"原域在生成链之前就判不可行（{reasons}），无法定位到具体段。"
    if target_index is None:
        if baseline["ok"]:
            return "整链无越界：不动采样域。"
        worst = chain.get("bad_segments") or []
        where = f"（最近的穿零段：{worst[0]}）" if worst else ""
        return (f"整链有 {chain['j3_crossings']} 处 J3 穿零 / "
                f"{chain['j5_crossings']} 处 J5 穿零{where}；"
                f"没给已走到的 GP 点号，无法定位到具体段。")
    if not offenders:
        if baseline["ok"]:
            return "窗口内各段都满足阈值，且整链无越界：不动采样域。"
        return ("窗口内各段都满足阈值：这一点附近复现不出报警，"
                "建议核对现场点号或控制器报警类型。")
    index = _repair_index(profiles, offenders, target_index)
    reported = next((profile for profile in profiles
                     if profile["index"] == target_index), None)
    if index != target_index:
        return (f"报告段本身安全，最危险的是 GP{index + 1}->GP{index + 2}。")
    j3 = reported["j3_abs_min"] if reported else None
    j5 = reported["j5_abs_min"] if reported else None
    return (f"报告段 GP{index + 1}->GP{index + 2} 越界："
            f"|J3|min={_fmt_value(j3)}、|J5|min={_fmt_value(j5)}。")


def _fmt_value(value, digits: int = 3) -> str:
    """``None`` 安全的数字显示（结论句里用）。"""
    return "n/a" if value is None else f"{float(value):.{digits}f}°"


def _classification_rows(analyses) -> list[dict]:
    """``classify_singularity`` 的结果 -> 前端用的扁平行（数值转 float）。

    定位页与修复结果都要展示这几个徽标，所以组装只留一份。

    Args:
        analyses: ``classify_singularity`` 返回的 dict 列表。

    Returns:
        list[dict]：``q_deg`` / ``kind`` / ``j3_abs`` / ``j5_abs`` /
        ``j5_floor`` / ``j3_floor`` / ``same_as_zero_pose`` / ``note`` /
        ``position``。
    """
    return [{"q_deg": [float(v) for v in analysis["q_deg"]],
             "kind": analysis["kind"],
             "j3_abs": float(analysis["j3_abs"]),
             "j5_abs": float(analysis["j5_abs"]),
             "j5_floor": float(analysis["j5_floor"]),
             "j3_floor": float(analysis["j3_floor"]),
             "same_as_zero_pose": bool(analysis["same_as_zero_pose"]),
             "note": analysis["note"],
             "position": [float(v) for v in analysis["position"]]}
            for analysis in analyses]


def _evaluation_stats(args: argparse.Namespace, evaluation: dict) -> dict:
    """把一次 ``evaluate_domain`` 的结果压成统计卡字段（修复前/后各一份）。

    与 ``run_scan`` 的 ``stats`` 同构，只有 ``post_ik_cond`` 恒为 ``None``：
    修复流程只算设计条件数（与 CLI 的搜索一致），不做逐位姿实测。
    """
    conditioning_json = scan._conditioning_to_json(
        evaluation.get("conditioning") or {})
    return _summary_stats(args, conditioning_json,
                          evaluation.get("steps") or [],
                          evaluation.get("guard"),
                          evaluation.get("segments"))


def locate_segment(params: dict) -> dict:
    """定位「已走到第几点」附近最危险的一段，并给出一句话结论（M4 的只读入口）。

    流程与 CLI 完全一致：``_validate_args`` 校验 → 解算原域 → 对每个报告位形
    ``classify_singularity`` → ``segment_profiles`` 取窗口剖面 → 用同一套判据
    找出真正越界的段。**不改任何东西**，也不落盘。

    Args:
        params: 表单值（键见 :func:`_repair_namespace`），其中
            ``singular_q`` 必填（``[[6 个角度], ...]``，deg），
            ``stopped_at`` 可为 ``None``（那就只报整链概览）。

    Returns:
        dict: 成功时 ``{"ok": True, "classifications", "effective_thresholds",
        "forbidden_positions", "stopped_at", "profiles", "offenders",
        "verdict", "unmodified", "chain", "warnings"}``：

        * ``classifications``：每个报告位形的 ``kind``（``degenerate`` /
          ``wrist`` / ``elbow`` / ``unexplained``）、``|J3|``/``|J5|``、
          两个下限、``same_as_zero_pose``、``note``、FK 位置；
        * ``effective_thresholds``：``j5_min_deg`` / ``j3_min_deg`` /
          ``j3_seg_min_deg``（由 ``classify_singularity`` 抬起来的阈值）；
        * ``profiles``：窗口内每段的 ``gp_from``/``gp_to``/``|J3|min``/
          ``|J5|min``/``crossings``/``worst_frac``/``worst_pos``/``ok``；
        * ``offenders``：``profiles`` 里真的越界的子集；
        * ``verdict``：一句话中文结论；
        * ``unmodified``：原域自身是否满足全部约束（``ok`` + ``reasons``）。

        校验失败或算法异常返回 ``{"ok": False, "error": 说明, "warnings": []}``，
        不抛异常。
    """
    warnings: list[str] = []
    try:
        args, hints = _repair_namespace(params or {})
        q_seed = _parsed_q_seed(args)
        target_count = _repair_target_count(args)
        target_index = _stopped_index(args, target_count)
    except _FormError as error:
        return _failure(str(error))
    except SystemExit:
        return _failure("参数校验失败（argparse 提前退出）")
    except (TypeError, ValueError) as error:
        return _failure(f"参数无法解析：{error}")
    warnings.extend(hints)

    try:
        ctx = _repair_context(args, q_seed)
        baseline = ctx["evaluate"](ctx["base"], args.j5_attempts)
        profiles: list[dict] = []
        offenders: list[dict] = []
        if target_index is not None:
            profiles, offenders = _window_profiles(ctx, baseline, target_index,
                                                   args)
        chain = _chain_summary(baseline.get("segments"))
        if not baseline["ok"]:
            warnings.append("原域不可行：" +
                            "；".join(baseline["reasons"][:2]))
        if chain["bad_segments"]:
            shown = "、".join(chain["bad_segments"][:12])
            warnings.append(f"整链有 {chain['j3_crossings']} 处 J3 穿零 / "
                            f"{chain['j5_crossings']} 处 J5 穿零：{shown}"
                            + ("…" if len(chain["bad_segments"]) > 12 else ""))
        for profile in profiles:
            if profile.get("ik_failures"):
                warnings.append(f"GP{profile['gp_from']}->GP{profile['gp_to']} "
                                f"段内有 {profile['ik_failures']} 个采样点无解")
        return {"ok": True,
                "classifications": _classification_rows(ctx["analyses"]),
                "effective_thresholds": {
                    "j5_min_deg": float(ctx["j5_eff"]),
                    "j3_min_deg": float(ctx["j3_eff"]),
                    "j3_seg_min_deg": float(ctx["j3_seg_min"])},
                "forbidden_positions": [[float(v) for v in point]
                                        for point in ctx["forbidden"]],
                "stopped_at": (None if args.stopped_at is None
                               else int(args.stopped_at)),
                "profiles": profiles,
                "offenders": offenders,
                "verdict": _verdict(baseline, profiles, offenders,
                                    target_index, chain),
                "unmodified": {"ok": bool(baseline["ok"]),
                               "reasons": list(baseline["reasons"])},
                "chain": chain,
                "warnings": warnings}
    except jobs.JobCancelled:
        raise
    except Exception as error:          # noqa: BLE001 - 一律转 ok:False
        return {"ok": False,
                "error": f"{type(error).__name__}: {error}",
                "warnings": warnings}


def _json_safe(value):
    """递归把非有限浮点换成 ``None``（``allow_nan=False`` 落盘才不炸）。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    return value


def _repair_report(args: argparse.Namespace, ctx: dict, baseline: dict,
                   profiles: list[dict], offenders: list[dict],
                   search_log: list[dict], chosen, chosen_eval,
                   repair: dict, succeeded: bool = True) -> dict:
    """``repair.json`` 的内容（骨架对齐 CLI ``write_report``）。

    ``succeeded=False`` 时 ``decision`` 写 ``null``——与 CLI 失败时的报告一致，
    否则读者会把"原域"当成最终决定。
    """
    base = ctx["base"]
    bounds = list(base["bounds"])
    return _json_safe({
        "tool": "laser_web.run_repair",
        "inputs": {
            "robot": str(args.urdf),
            "point_min": bounds[0::2],
            "point_max": bounds[1::2],
            "rpy": [float(v) for v in args.rpy],
            "euler": args.euler_convention,
            "target_count": int(ctx["target_count"]),
            "singular_q": [[float(v) for v in analysis["q_deg"]]
                           for analysis in ctx["analyses"]],
            "perturb_deg": [float(v) for v in args.perturb_deg],
            "stopped_at": None if args.stopped_at is None
                          else int(args.stopped_at),
            "segment_window": int(args.segment_window),
        },
        "singularity_analysis": [
            {key: analysis[key] for key in
             ("q_deg", "position", "j3_abs", "j5_abs", "kind", "j5_floor",
              "j3_floor", "same_as_zero_pose", "note")}
            for analysis in ctx["analyses"]],
        "effective_thresholds": {
            "j5_min_deg": ctx["j5_eff"], "j3_min_deg": ctx["j3_eff"],
            "j3_seg_min_deg": ctx["j3_seg_min"]},
        "forbidden_positions": ctx["forbidden"],
        "avoid_radius_mm": float(args.avoid_radius),
        "segment_diagnostics": (None if args.stopped_at is None else
                                {"reported_gp": int(args.stopped_at),
                                 "window": int(args.segment_window),
                                 "profiles": profiles,
                                 "offenders": [item["index"]
                                               for item in offenders]}),
        "baseline": {"label": base["label"],
                     "bounds": [round(float(v), 3) for v in base["bounds"]],
                     "rpy": [round(float(v), 3) for v in base["rpy"]],
                     "ok": bool(baseline["ok"]),
                     "reasons": list(baseline["reasons"])},
        "search": search_log,
        "decision": None if not succeeded else {"label": chosen["label"],
                     "tier": int(chosen["tier"]),
                     "bounds": [round(float(v), 3) for v in chosen["bounds"]],
                     "rpy": [round(float(v), 3) for v in chosen["rpy"]],
                     "moves": chosen["moves"],
                     "cost": round(avoid.change_cost(base, chosen), 6),
                     "stats": chosen_eval.get("stats")},
        "repair": repair,
    })


def _execute_repair(args: argparse.Namespace, q_seed: list[float],
                    target_index, job, progress, warnings: list[str]) -> dict:
    """``run_repair`` 的主体：解算 → 定位 → 局部修复 → 整体搜索 → 复核 → 落盘。

    顺序固定，与 CLI ``avoid_singularity.run()`` 一一对应；每一步只做编排，
    定位/修复/搜索/细化全部调用 ``avoid_singularity`` 的函数。
    """
    import matplotlib.pyplot as plt

    run_id, run_dir = ((str(job.id), Path(job.run_dir)) if job is not None
                       else jobs.make_run_dir())

    # ---- 1. 解算原域 ----
    _check_cancel(job)
    progress("解算 IK", 0, 1)
    ctx = _repair_context(args, q_seed)
    base, evaluate = ctx["base"], ctx["evaluate"]
    baseline = evaluate(base, args.j5_attempts)
    before_stats = _evaluation_stats(args, baseline)
    progress("解算 IK", 1, 1)
    if not baseline["ok"]:
        warnings.append("原域不可行（评分 "
                        f"{baseline['score']:.1f}）："
                        + "；".join(baseline["reasons"][:3]))

    # ---- 2. 段定位 + 3. 局部修复（只换端点姿态，不动网格与位置） ----
    profiles: list[dict] = []
    offenders: list[dict] = []
    local_change = None
    repair_mode = "none"
    chain_summary = _chain_summary(baseline.get("segments"))
    verdict = _verdict(baseline, profiles, offenders, target_index,
                       chain_summary)
    if chain_summary["bad_segments"]:
        shown = "、".join(chain_summary["bad_segments"][:12])
        warnings.append(f"整链另有 {chain_summary['j3_crossings']} 处 J3 穿零 / "
                        f"{chain_summary['j5_crossings']} 处 J5 穿零：{shown}"
                        + ("…" if len(chain_summary["bad_segments"]) > 12
                           else ""))
    if target_index is not None:
        _check_cancel(job)
        progress("定位段", 0, 1)
        profiles, offenders = _window_profiles(ctx, baseline, target_index, args)
        # 结论句用**动手之前**的整链状态（CLI 也是先判后改）
        verdict = _verdict(baseline, profiles, offenders, target_index,
                           chain_summary)
        progress("定位段", 1, 1)

        _check_cancel(job)
        progress("局部修复", 0, 1)
        index = _repair_index(profiles, offenders, target_index)
        if baseline.get("steps") and index is not None:
            repaired_steps, changed = avoid.repair_segment(
                ctx["rbt"], baseline["waypoints"], baseline["steps"], index,
                order=ctx["order"], extrinsic=ctx["extrinsic"],
                perturb_deg=args.perturb_deg, seed=args.seed,
                attempts=args.repair_attempts, j5_min=ctx["j5_eff"],
                j3_min=ctx["j3_eff"], seg_samples=args.seg_samples_final)
        else:
            # 原域在出链之前就判不可行（位置禁区）或没有越界段：没有链可改，
            # 直接去整体搜索。不能把 steps=None 传给 repair_segment——
            # 它第一行就 len(steps)，会抛出一个埋掉真实原因的 TypeError。
            repaired_steps, changed = baseline["steps"], None
        progress("局部修复", 1, 1)
        if changed is not None:
            q_before = None
            if baseline["steps"][changed["index"]].q is not None:
                q_before = [float(v) for v
                            in baseline["steps"][changed["index"]].q]
            repaired_eval = evaluate(base, args.j5_attempts,
                                     steps_override=repaired_steps,
                                     waypoints_override=baseline["waypoints"])
            if repaired_eval["ok"]:
                baseline = repaired_eval
                local_change = {
                    "index": int(changed["index"]),
                    "gp": int(changed["index"]) + 1,
                    "delta_rpy": [float(v) for v in changed["delta_rpy"]],
                    "q_before": q_before,
                    "q_after": [float(v) for v in changed["q"]]}
                repair_mode = "local"
            else:
                warnings.append("局部修复后全链仍不满足：" +
                                "；".join(repaired_eval["reasons"][:2]))
        elif index is not None:
            warnings.append("局部修复未成功（换端点姿态解决不了），转整体挪箱")

    # ---- 4. 整体搜索（按改动量从小到大，第一个可行再二分压缩） ----
    chosen, chosen_eval = base, baseline
    search_log: list[dict] = []
    if baseline["ok"]:
        search_log = [{"label": base["label"], "tier": 0, "ok": True,
                       "score": round(float(baseline["score"]), 3),
                       "reasons": []}]
    else:
        search_log = [{"label": base["label"], "tier": 0, "ok": False,
                       "score": round(float(baseline["score"]), 3),
                       "reasons": list(baseline["reasons"][:2])}]
        found = _search_domains(base, evaluate, args, search_log, job, progress,
                                warnings)
        # ---- 5. 复核（完整重试次数 + 更密的段采样） ----
        _check_cancel(job)
        progress("复核", 0, 1)
        candidate, confirmed = None, None
        if found is not None:
            candidate, _result = found
            confirmed = evaluate(candidate, args.j5_attempts,
                                 samples=args.seg_samples_final)
        progress("复核", 1, 1)
        if confirmed is not None and confirmed["ok"]:
            chosen, chosen_eval = candidate, confirmed
            repair_mode = "domain"
        else:
            reasons = [] if confirmed is None else confirmed["reasons"][:3]
            repair = _build_repair(repair_mode, local_change, base, chosen,
                                   search_log, before_stats,
                                   _evaluation_stats(args, chosen_eval),
                                   profiles, offenders, target_index, args, ctx,
                                   verdict)
            report_path = run_dir / ARTIFACT_REPAIR
            report_path.write_text(json.dumps(
                _repair_report(args, ctx, baseline, profiles, offenders,
                               search_log, chosen, chosen_eval, repair,
                               succeeded=False),
                ensure_ascii=False, indent=2, allow_nan=False),
                encoding="utf-8")
            return {"ok": False,
                    "error": ("所有候选都不可行"
                              + ("：" + "；".join(reasons) if reasons else "")
                              + f"（详见 {ARTIFACT_REPAIR}）"),
                    "repair": repair,
                    "run_id": run_id,
                    "run_dir": str(run_dir),
                    "artifacts": {"repair": ARTIFACT_REPAIR},
                    "warnings": warnings}

    # ---- 6. 出图 ----
    _check_cancel(job)
    progress("出图", 0, 1)
    waypoints = chosen_eval["waypoints"]
    chain = chosen_eval["steps"] or []
    ok_flags = [step.ok for step in chain]
    plot_target = run_dir / ARTIFACT_PLOT
    scan.plot_path([tuple(waypoint.pos) for waypoint in waypoints], ok_flags,
                   str(plot_target),
                   title=f"repair {repair_mode} · IK {sum(ok_flags)}/{len(chain)}")
    plt.close("all")
    progress("出图", 1, 1)

    # ---- 7. 落盘：scan.json / joints.txt / repair.json ----
    _check_cancel(job)
    progress("写文件", 0, 3)
    payload = avoid.scan_payload(chosen, chosen_eval, seed=args.seed,
                                 rpy=chosen["rpy"],
                                 convention=args.euler_convention,
                                 target_count=ctx["target_count"])
    with open(run_dir / ARTIFACT_SCAN_JSON, "w", encoding="utf-8") as handle:
        json.dump(_json_safe(payload), handle, ensure_ascii=False, indent=2,
                  allow_nan=False)
    progress("写文件", 1, 3)
    artifacts = {"scan_json": ARTIFACT_SCAN_JSON, "plot": ARTIFACT_PLOT,
                 "repair": ARTIFACT_REPAIR}
    if any(step.q is not None for step in chain):
        written = _write_joints(run_dir / ARTIFACT_JOINTS, chain)
        artifacts["joints"] = ARTIFACT_JOINTS
        if written < len(chain):
            warnings.append(f"joints.txt 只写了 {written}/{len(chain)} 行"
                            f"（无解路点无法下发）")
    progress("写文件", 2, 3)

    after_stats = _evaluation_stats(args, chosen_eval)
    repair = _build_repair(repair_mode, local_change, base, chosen, search_log,
                           before_stats, after_stats, profiles, offenders,
                           target_index, args, ctx, verdict)
    report = _repair_report(args, ctx, baseline, profiles, offenders,
                            search_log, chosen, chosen_eval, repair)
    (run_dir / ARTIFACT_REPAIR).write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8")
    progress("写文件", 3, 3)

    _scan_warnings(args, warnings, chain, chosen_eval.get("guard"),
                   chosen_eval.get("segments"),
                   scan._conditioning_to_json(chosen_eval.get("conditioning")
                                              or {}))
    if repair_mode == "local":
        warnings.append(f"局部修复：GP{local_change['gp']} 换成 "
                        f"delta_rpy="
                        f"{[round(v, 2) for v in local_change['delta_rpy']]}"
                        "（位置不变，只换端点姿态）")
    return {"ok": True,
            "run_id": run_id,
            "run_dir": str(run_dir),
            "stats": after_stats,
            "meta": payload["meta"],
            "waypoints": _waypoint_rows(waypoints, chain),
            "segments": _segment_rows(waypoints, chain,
                                      chosen_eval.get("segments")),
            "artifacts": artifacts,
            "repair": repair,
            "warnings": warnings}


def _build_repair(repair_mode: str, local_change, base, chosen,
                  search_log: list[dict], before_stats: dict,
                  after_stats: dict, profiles: list[dict],
                  offenders: list[dict], target_index, args, ctx,
                  verdict: str) -> dict:
    """组装返回给前端的 ``repair`` 节（模式 / 改动摘要 / 前后对比 / 日志）。"""
    volume_before = avoid.domain_volume(base)
    volume_after = avoid.domain_volume(chosen)
    volume_loss = (0.0 if volume_before <= 0
                   else max(0.0, 1.0 - volume_after / volume_before))
    if repair_mode == "local":
        label = f"重采 GP{local_change['gp']} 的姿态"
    elif repair_mode == "domain":
        label = chosen["label"]
    else:
        label = "无改动"
    return {"mode": repair_mode,
            "label": label,
            "bounds_before": [float(v) for v in base["bounds"]],
            "bounds_after": [float(v) for v in chosen["bounds"]],
            "volume_loss": float(volume_loss),
            "rpy_before": [float(v) for v in base["rpy"]],
            "rpy_after": [float(v) for v in chosen["rpy"]],
            "moves": [dict(move) for move in chosen["moves"]],
            "local": local_change,
            "log": search_log[:REPAIR_LOG_LIMIT],
            "log_total": len(search_log),
            "before_stats": before_stats,
            "after_stats": after_stats,
            # 下面几项是页面用的额外信息（判定徽标 / 段定位表），
            # 规格里的 repair 键列没写它们，但省一次重复解算。
            "verdict": verdict,
            "classifications": _classification_rows(ctx["analyses"]),
            "forbidden_positions": [[float(v) for v in point]
                                    for point in ctx["forbidden"]],
            "profiles": profiles,
            "offenders": [profile["index"] for profile in offenders],
            "stopped_at": None if target_index is None
                          else int(target_index) + 1,
            "effective_thresholds": {
                "j5_min_deg": float(ctx["j5_eff"]),
                "j3_min_deg": float(ctx["j3_eff"]),
                "j3_seg_min_deg": float(ctx["j3_seg_min"])}}


def _search_domains(base, evaluate, args: argparse.Namespace,
                    search_log: list[dict], job, progress,
                    warnings: list[str]):
    """按改动量从小到大搜候选（单轴/单侧 -> 双轴），再二分压缩。

    Returns:
        ``(candidate, evaluation)`` 或 ``None``（全部不可行 / 到候选上限）。
    """
    state = {"evaluated": 0}

    def phase(include_pairs: bool, tag: str):
        """跑一轮候选；可行就立刻返回（与 CLI 的 ``_run_phase`` 等价）。"""
        candidates = avoid.ordered_candidates(
            base, allow_rpy=not args.no_rpy_change,
            allow_shrink=not args.no_shrink, include_pairs=include_pairs)
        total = len(candidates)
        for candidate in candidates:
            state["evaluated"] += 1
            if state["evaluated"] > args.max_candidates:
                warnings.append(f"已达候选上限 {args.max_candidates}，停止搜索")
                return None
            _check_cancel(job)
            progress("整体搜索", state["evaluated"], total)
            result = evaluate(candidate, avoid.SEARCH_ATTEMPTS)
            search_log.append({
                "phase": tag, "label": candidate["label"],
                "tier": int(candidate["tier"]),
                "magnitude": float(candidate["magnitude"]),
                "ok": bool(result["ok"]),
                "score": round(float(result["score"]), 3),
                "reasons": list(result["reasons"][:2])})
            if result["ok"]:
                return candidate, result
        return None

    found = phase(False, "single")
    if found is None:
        found = phase(True, "pair")
    if found is None:
        return None
    candidate, result = found
    refined, refined_result = avoid._refine(base, candidate, result, evaluate,
                                            search_log)
    progress("整体搜索", state["evaluated"], state["evaluated"])
    return refined, refined_result


def run_repair(params: dict, job=None, on_progress=None) -> dict:
    """修复奇异点：局部优先，失败再整体挪箱（M4 的主入口，**不抛异常**）。

    与 CLI ``python -m excitation_trajectory.avoid_singularity`` 走**同一条**
    算法路径：校验用 ``_validate_args``，分类/定位/局部修复/候选搜索/二分细化
    全部 import ``avoid_singularity``，本函数只负责串流程、报进度、落盘。

    顺序（与 CLI ``run()`` 一致）：

    1. 解算原域并评估（``evaluate_domain``，含 MOVL 段检查）；
    2. 给了 ``stopped_at`` 就 ``segment_profiles`` 定位窗口，有 offender 才
       ``repair_segment``（只换端点姿态，位置与网格不变）；
    3. 局部结果用 ``steps_override`` + ``waypoints_override`` 复核全链，通过即采用；
    4. 否则 ``ordered_candidates`` 按改动量搜索 + ``_refine`` 压缩，再用完整
       重试次数与更密的段采样复核；
    5. 复核通过后落盘（``scan.json`` / ``joints.txt`` / ``plot.png`` /
       ``repair.json``）。

    Args:
        params: 表单值（键见 :func:`_repair_namespace`）。
        job: ``jobs.Job``（可选）；给了就用它的 ``run_dir`` 并在阶段边界检查取消。
        on_progress: ``on_progress(stage, index, total)``；阶段名是
            :data:`STAGES` 加 :data:`REPAIR_STAGES`。

    Returns:
        dict: 成功时与 ``run_scan`` **同构**：``{"ok", "run_id", "run_dir",
        "stats", "meta", "waypoints", "segments", "artifacts", "warnings"}``，
        另外多一个 ``repair``：``{"mode"（none/local/domain）, "label",
        "bounds_before", "bounds_after", "volume_loss", "rpy_before",
        "rpy_after", "moves", "local", "log", "before_stats", "after_stats"}``
        （``local`` 是 ``{"index", "gp", "delta_rpy", "q_before", "q_after"}``
        或 ``None``，``log`` 是候选评估记录，最多 60 条）。

        校验失败返回 ``{"ok": False, "error": 报错原文, "warnings": [...]}``；
        所有候选都不可行时也返回 ``ok: False``（附 ``repair`` 与
        ``repair.json`` 供复盘）。不抛异常。
    """
    warnings: list[str] = []
    try:
        args, hints = _repair_namespace(params or {})
        q_seed = _parsed_q_seed(args)
        target_count = _repair_target_count(args)
        target_index = _stopped_index(args, target_count)
    except _FormError as error:
        return _failure(str(error))
    except SystemExit:
        return _failure("参数校验失败（argparse 提前退出）")
    except (TypeError, ValueError) as error:
        return _failure(f"参数无法解析：{error}")
    warnings.extend(hints)
    try:
        return _execute_repair(args, q_seed, target_index, job,
                               _StageReporter(on_progress), warnings)
    except jobs.JobCancelled:
        raise                          # 交给 jobs 层归为 cancelled
    except SystemExit as exit_error:   # 算法自己 sys.exit 时也不炸 UI
        return {"ok": False,
                "error": f"算法中止（退出码 {exit_error.code}），详见任务日志",
                "warnings": warnings}
    except Exception as error:          # noqa: BLE001 - 一律转 ok:False
        return {"ok": False,
                "error": f"{type(error).__name__}: {error}",
                "warnings": warnings}


# ---------------------------------------------------------------------------
# M3：工具页（GP 导出 / JSON -> URDF / 运行目录盘点与清理）
# ---------------------------------------------------------------------------
#: GP 模板里必须保持原样的元数据字段（写完回读比对，作为"模板未改动"的证据）。
GP_TEMPLATE_META_KEYS = ("controllerID", "version")
#: 上传字节里拿不到文件名时的兜底（页面会显式传 ``template_name``）。
GP_TEMPLATE_NAME = "position_R1.json"
#: ``--cartesian`` 的全部取值（与 ``export_gp`` 的 CLI 一致）。
GP_CARTESIAN_MODES = ("zero", "fk")
#: ``--cart-units`` 的全部取值（与 ``export_gp`` 的 CLI 一致）。
GP_CART_UNITS = ("rad", "deg")
#: ``--first`` 的默认值（与 ``export_gp`` 的 CLI 一致）。
GP_DEFAULT_FIRST = 1
#: ``--cart-units`` 的默认值（与 ``export_gp`` 的 CLI 一致：控制器读数是弧度）。
GP_DEFAULT_CART_UNITS = "rad"
#: ``--euler`` 的默认值（与 ``export_gp`` 的 CLI 一致；等价于
#: ``handeye.pose_encoder.DEFAULT_EULER_CONVENTION``，测试里核对）。
GP_DEFAULT_EULER = "xyz-intrinsic"
#: ``export_gp.main`` 在 ``--cartesian fk`` 时写死使用的模型（照抄同一路径）。
GP_FK_URDF = CONFIG_DIR / "hyundai_general6s.urdf"
#: 上传文件名/型号进路径前的清洗白名单（设计文档 §7.6：上传统一落 ASCII 名）。
_SAFE_NAME = re.compile(r"[^A-Za-z0-9._-]+")
#: JSON -> URDF 的工作目录（系统临时目录下，供下载按钮持续读取）。
URDF_WORKDIR_NAME = "laser_web_urdf"
#: 上传名清洗后为空时的兜底主名。
URDF_DEFAULT_STEM = "controller"
#: ``json_to_urdf`` 自校验那一行的误差值：``… 最大误差 = 4.547e-13 mm/rad``。
_SELF_CHECK_PATTERN = re.compile(
    r"最大误差\s*=\s*([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)")
#: 运行目录里被 :func:`list_runs` 逐个检查的产物（键 -> 文件名，键即 ``has_*``）。
RUN_ARTIFACTS = {"scan": ARTIFACT_SCAN_JSON, "plot": ARTIFACT_PLOT,
                 "joints": ARTIFACT_JOINTS, "repair": ARTIFACT_REPAIR}


def _bytes_of(raw) -> bytes:
    """把上传内容归一成 ``bytes``（``str`` 按 UTF-8 编码）。

    Args:
        raw: ``bytes`` / ``bytearray`` / ``memoryview`` / ``str``。

    Returns:
        bytes: 内容本身（不做编码猜测，模板与控制器 JSON 都是 UTF-8）。

    Raises:
        TypeError: 既不是字节串也不是字符串。
    """
    if isinstance(raw, bytes):
        return raw
    if isinstance(raw, (bytearray, memoryview)):
        return bytes(raw)
    if isinstance(raw, str):
        return raw.encode("utf-8")
    raise TypeError(f"需要字节串，当前为 {type(raw).__name__}")


def _safe_filename(name, default: str) -> tuple[str, bool]:
    """把上传名清洗成可安全落盘的 ASCII 文件名（保留扩展名）。

    Args:
        name: 上传时的原始文件名（可能带目录、中文、空格）。
        default: 清洗后为空时的兜底名。

    Returns:
        ``(文件名, 是否被改过)``。
    """
    candidate = Path(str(name or "")).name.strip()
    cleaned = _SAFE_NAME.sub("_", candidate).strip("._")
    if not cleaned:
        return default, candidate != ""
    return cleaned, cleaned != candidate


def gp_euler_conventions() -> list[str]:
    """``--euler`` 下拉框的选项（算法模块里的 4 种约定，页面不另抄一份）。"""
    from handeye.pose_encoder import EULER_CONVENTIONS

    return list(EULER_CONVENTIONS)


def gp_defaults() -> dict:
    """GP 导出表单的默认值（与 ``export_gp`` 的 CLI 默认值一一对应）。"""
    from excitation_trajectory import export_gp

    return {"first": GP_DEFAULT_FIRST,
            "flag": int(export_gp.DEFAULT_FLAG),
            "cartesian": "zero",
            "cart_units": GP_DEFAULT_CART_UNITS,
            "euler": GP_DEFAULT_EULER}


def _gp_scan_path(params: dict) -> Path:
    """解析 ``run_dir`` / ``scan`` 两种入参，返回 ``scan.json`` 的绝对路径。

    Args:
        params: GP 导出入参；``run_dir`` 指向 ``runs/<id>/``（自动补
            ``scan.json``），``scan`` 直接给 ``scan.json`` 路径（优先）。

    Returns:
        Path: 已确认存在的 ``scan.json`` 绝对路径。

    Raises:
        _FormError: 两个键都没给，或文件不存在。
    """
    for key in ("scan", "run_dir"):
        value = params.get(key)
        if value is None or str(value).strip() == "":
            continue
        path = Path(str(value)).expanduser()
        if not path.is_absolute():
            path = jobs.repo_root() / path
        path = path.resolve()
        if path.is_dir():
            path = path / ARTIFACT_SCAN_JSON
        if not path.is_file():
            raise _FormError(f"{key} 里没有 {ARTIFACT_SCAN_JSON}：{path}")
        return path
    raise _FormError(f"需要 run_dir（runs/<id>/）或 scan（{ARTIFACT_SCAN_JSON} 路径）")


def _gp_options(params: dict) -> dict:
    """归一化 GP 导出选项（取值合法性判定与 ``export_gp`` 的 CLI 一致）。"""
    cartesian = str(params.get("cartesian") or "zero").strip().lower()
    if cartesian not in GP_CARTESIAN_MODES:
        raise _FormError(f"--cartesian 需为 {' 或 '.join(GP_CARTESIAN_MODES)}，"
                         f"当前为 {cartesian!r}")
    units = str(params.get("cart_units") or GP_DEFAULT_CART_UNITS).strip().lower()
    if units not in GP_CART_UNITS:
        raise _FormError(f"--cart-units 需为 {' 或 '.join(GP_CART_UNITS)}，"
                         f"当前为 {units!r}")
    euler = str(params.get("euler") or GP_DEFAULT_EULER).strip().lower()
    first = GP_DEFAULT_FIRST if params.get("first") is None else int(params["first"])
    flag = params.get("flag")
    from excitation_trajectory import export_gp

    flag = int(export_gp.DEFAULT_FLAG if flag is None else flag)
    return {"scan": _gp_scan_path(params or {}),
            "first": first, "flag": flag, "cartesian": cartesian,
            "cart_units": units, "euler": euler,
            "template_name": params.get("template_name")}


def _gp_cartesian_of(euler: str, cart_units: str):
    """构造 ``--cartesian fk`` 的笛卡尔填充函数。

    与 ``excitation_trajectory.export_gp.main`` 的 ``--cartesian fk`` 分支**同一套**：
    同一个模型（``data/configs/hyundai_general6s.urdf``）、同一个
    ``DhOverride.from_base``、同一个 ``matrix_to_pose_fields``。本函数只负责
    把这三者接起来，公式一律来自 ``handeye``（不在这里复制）。

    Args:
        euler: 欧拉约定（``xyz-intrinsic`` 等，见 :func:`gp_euler_conventions`）。
        cart_units: ``rad`` 或 ``deg``（与 ``--cart-units`` 同义）。

    Returns:
        ``q_deg -> [x, y, z, rx, ry, rz]`` 的可调用对象。

    Raises:
        _FormError: ``euler`` 不在 ``EULER_CONVENTIONS`` 里（报错措辞同 CLI）。
    """
    from handeye.dh import DhOverride, RobotKinematics
    from handeye.pose_encoder import EULER_CONVENTIONS, matrix_to_pose_fields

    if euler not in EULER_CONVENTIONS:
        raise _FormError(f"--euler 必须为 {EULER_CONVENTIONS} 之一，"
                         f"当前为 {euler!r}")
    urdf = _REPO_ROOT / GP_FK_URDF
    robot = RobotKinematics(urdf, DhOverride.from_base(urdf))

    def cartesian_of(q_deg):
        position, orientation = matrix_to_pose_fields(robot.fk(q_deg),
                                                      convention=euler)
        angles = list(orientation)
        if cart_units == "rad":
            angles = [math.radians(value) for value in angles]
        return list(position) + angles

    return cartesian_of


def build_gp(params: dict, template_bytes=None) -> dict:
    """把一次扫描的关节角写进 GP 模板的副本（CLI ``export_gp`` 的等价入口）。

    与 ``python -m excitation_trajectory.export_gp`` 走**同一条**路径：
    ``load_scan_points`` 取点、``check_joint_range`` 做写前限位校验、
    ``write_gp_entries`` 填 14 字段；``--cartesian fk`` 用同一个模型 +
    ``handeye.pose_encoder.matrix_to_pose_fields``（见 :func:`_gp_cartesian_of`）。
    模板**只读**：上传的字节先落到临时目录，写完把这份副本再读回来比对
    sha256 与 ``controllerID`` / ``version``。

    Args:
        params: 入参：

            * ``run_dir``：某次运行的 ``runs/<id>/``（内含 ``scan.json``），
              或 ``scan``：直接给 ``scan.json`` 路径（优先）；
            * ``template``：GP 模板文件字节（**必填**，也可用
              ``template_bytes`` 参数传，二者等价，参数优先）；
            * ``template_name``：模板原始文件名（决定输出文件名，
              缺省 ``position_R1.json``）；
            * ``first`` / ``flag``：起始 GP 号（默认 1）/ 标志位（默认 0）；
            * ``cartesian``：``"zero"``（默认）或 ``"fk"``；
            * ``cart_units``：``"rad"``（默认）或 ``"deg"``；
            * ``euler``：``--cartesian fk`` 的欧拉约定，默认 ``xyz-intrinsic``。
        template_bytes: 模板字节（覆盖 ``params["template"]``）。

    Returns:
        dict: 成功时 ``{"ok": True, "path": 绝对路径, "filename": 文件名,
        "count": int, "first": int, "last": int,
        "meta": {"controllerID": ..., "version": ...}, "warnings": [...]}``；
        失败时 ``{"ok": False, "error": 报错原文, "warnings": [...]}``，不抛异常。
    """
    warnings: list[str] = []
    from excitation_trajectory import export_gp

    try:
        options = _gp_options(params or {})
        raw = (template_bytes if template_bytes is not None
               else (params or {}).get("template"))
        if raw is None:
            raise _FormError("需要上传 GP 模板文件（position_R1.json 的字节）")
        data = _bytes_of(raw)
        if not data:
            raise _FormError("GP 模板文件是空的")
        name, renamed = _safe_filename(options["template_name"], GP_TEMPLATE_NAME)
        if renamed:
            warnings.append(f"模板文件名被清洗为 {name}（原 "
                            f"{options['template_name']!r}）")
        run_dir = options["scan"].parent
        out_path = run_dir / name
    except _FormError as error:
        return _failure(str(error))
    except (TypeError, ValueError) as error:
        return _failure(f"参数无法解析：{error}")

    if out_path.exists():
        warnings.append(f"输出文件 {name} 已存在，已覆盖")

    with tempfile.TemporaryDirectory(prefix="laser_web_gp-") as tmp:
        # 模板只读：先落一份临时副本，流程里只碰副本，写完再读回来核对。
        template_path = Path(tmp) / name
        try:
            template_path.write_bytes(data)
            before_digest = hashlib.sha256(template_path.read_bytes()).hexdigest()
            template = json.loads(template_path.read_text(encoding="utf-8"))
            if not isinstance(template, dict):
                raise ValueError("模板文件顶层需为对象")
            before_meta = {key: template.get(key) for key in GP_TEMPLATE_META_KEYS}
            points = export_gp.load_scan_points(options["scan"])
            for point in points:                       # CLI 的写前限位校验
                export_gp.check_joint_range(point["q"], f"路点 {point['index']}")
            cartesian_of = None
            if options["cartesian"] == "fk":
                cartesian_of = _gp_cartesian_of(options["euler"],
                                                options["cart_units"])
            updated, report = export_gp.write_gp_entries(
                template, points, first=options["first"], flag=options["flag"],
                cartesian_of=cartesian_of)
        except _FormError as error:
            return {"ok": False, "error": str(error), "warnings": warnings}
        except SystemExit as exit_error:            # load_scan_points 用 SystemExit 报错
            return {"ok": False, "error": str(exit_error.code), "warnings": warnings}
        except (OSError, ValueError, UnicodeDecodeError) as error:
            return {"ok": False, "error": str(error), "warnings": warnings}
        except Exception as error:                  # noqa: BLE001 - 一律转 ok:False
            return {"ok": False, "error": f"{type(error).__name__}: {error}",
                    "warnings": warnings}

        try:
            with open(out_path, "w", encoding="utf-8") as handle:
                handle.write(json.dumps(updated, ensure_ascii=False, indent=1) + "\n")
        except OSError as error:
            return {"ok": False, "error": f"写 GP 文件失败：{error}",
                    "warnings": warnings}

        # 回读模板副本：字节与 controllerID/version 都必须原样。
        try:
            after_digest = hashlib.sha256(template_path.read_bytes()).hexdigest()
            after_meta = {key: json.loads(template_path.read_text(encoding="utf-8"))
                          .get(key) for key in GP_TEMPLATE_META_KEYS}
        except (OSError, ValueError) as error:
            return {"ok": False, "error": f"回读模板失败：{error}",
                    "warnings": warnings}

    if after_digest != before_digest or after_meta != before_meta:
        return {"ok": False,
                "error": "模板在导出过程中被改动（sha256 或 controllerID/version "
                         "不一致），已中止",
                "warnings": warnings}
    for key in GP_TEMPLATE_META_KEYS:
        if before_meta[key] is None:
            warnings.append(f"模板里没有 {key}，无法核对这一项是否被改动")

    first = int(options["first"])
    count = int(report["count"])
    return {"ok": True,
            "path": str(out_path),
            "filename": name,
            "count": count,
            "first": first,
            "last": first + count - 1,
            "meta": before_meta,
            "warnings": warnings}


def urdf_workdir() -> Path:
    """JSON -> URDF 的工作目录：``<系统临时目录>/laser_web_urdf``（自动创建）。"""
    path = Path(tempfile.gettempdir()) / URDF_WORKDIR_NAME
    path.mkdir(parents=True, exist_ok=True)
    return path


def _urdf_stem(filename) -> tuple[str, bool]:
    """上传文件名 -> 输出主名（去目录、去扩展名、清洗成 ASCII）。"""
    name = Path(str(filename or "")).name
    stem = Path(name).stem if name else ""
    cleaned = _SAFE_NAME.sub("_", stem).strip("._")
    if not cleaned:
        return URDF_DEFAULT_STEM, bool(name)
    return cleaned, cleaned != stem


def _cli_error_text(stderr_text: str) -> str:
    """从被捕获的 stderr 里取 argparse / 算法的原始报错（末行，去掉 usage/prog）。"""
    lines = [line.strip() for line in str(stderr_text or "").splitlines()
             if line.strip()]
    if not lines:
        return ""
    last = lines[-1]
    marker = "error: "
    if marker in last:
        last = last.split(marker, 1)[1]
    return last.strip()


def convert_urdf(json_bytes, model=None, filename=None) -> dict:
    """控制器 JSON -> URDF（CLI ``python -m robot_model.json_to_urdf`` 的等价入口）。

    直接调 ``robot_model.json_to_urdf.main``：上传的 JSON 落到临时工作目录，
    输出 ``<原主名>_<型号>.urdf``（型号为空时不追加后缀，与 CLI 不带
    ``--model`` 一致）。自校验误差由 ``main`` 打印在 stdout 上，这里用正则
    抓出来给页面显示。

    Args:
        json_bytes: 控制器 JSON 的字节（``str`` 按 UTF-8 处理）。
        model: 型号名（进文件名后缀）；``None`` 或空串表示不追加后缀。
        filename: 上传时的原始文件名（决定输出主名）；缺省 ``controller.json``。

    Returns:
        dict: 成功时 ``{"ok": True, "path": 绝对路径, "filename": 文件名,
        "self_check_error": float|None, "warnings": [...]}``；失败时
        ``{"ok": False, "error": 报错原文, "warnings": [...]}``（型号非法时
        错误原文来自 ``json_to_urdf`` 的 argparse），不抛异常。
    """
    from robot_model import json_to_urdf

    warnings: list[str] = []
    try:
        data = _bytes_of(json_bytes)
    except TypeError as error:
        return _failure(str(error))
    if not data:
        return _failure("需要上传控制器 JSON 文件")
    model_name = str(model).strip() if model is not None else ""
    stem, renamed = _urdf_stem(filename)
    if renamed:
        warnings.append(f"上传文件名被清洗为 {stem}（原 "
                        f"{str(filename or '')!r}）")

    try:
        workdir = urdf_workdir()
        source = workdir / f"{stem}.json"
        target = workdir / f"{stem}.urdf"
        source.write_bytes(data)
    except OSError as error:
        return _failure(f"写临时文件失败：{error}")

    argv = [str(source), str(target)]
    if model_name:
        argv += ["--model", model_name]
    stdout, stderr = io.StringIO(), io.StringIO()
    try:
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = json_to_urdf.main(argv)
    except SystemExit as exit_error:          # argparse 报错（型号非法）
        message = _cli_error_text(stderr.getvalue())
        return {"ok": False,
                "error": message or f"json_to_urdf 中止（退出码 {exit_error.code}）",
                "warnings": warnings}
    except AssertionError as error:           # 自校验断言失败
        return {"ok": False, "error": f"自校验失败：{error}", "warnings": warnings}
    except (OSError, ValueError, KeyError, TypeError) as error:
        return {"ok": False, "error": f"{type(error).__name__}: {error}",
                "warnings": warnings}
    if code not in (None, 0):
        return _failure(f"json_to_urdf 返回退出码 {code}")

    produced = json_to_urdf.output_path_for(source, target, model_name or None)
    if not produced.is_file():
        return _failure(f"json_to_urdf 没有生成 {produced.name}")

    self_check = None
    match = _SELF_CHECK_PATTERN.search(stdout.getvalue())
    if match is not None:
        try:
            self_check = float(match.group(1))
        except ValueError:                    # 正则已限定数字，这里只兜底
            self_check = None
    if self_check is None:
        warnings.append("没能从 json_to_urdf 的输出里解析出自校验误差")
    elif self_check >= 1e-9:
        warnings.append(f"自校验误差 {self_check:.3e} 超过 1e-9，"
                        "URDF 与 DH 可能不等价")

    return {"ok": True,
            "path": str(produced),
            "filename": produced.name,
            "self_check_error": self_check,
            "warnings": warnings}


def _run_row(path: Path) -> dict:
    """一个运行目录 -> 一行盘点数据（``list_runs`` 的元素）。

    Args:
        path: ``runs/<id>/`` 目录。

    Returns:
        dict: ``run_id`` / ``dir`` / ``created``（目录 mtime，与
        ``jobs.prune_runs`` 的排序键同源）/ 四个 ``has_*`` 产物标记 /
        ``size_bytes``（目录下所有文件的字节数）。
    """
    stat = path.stat()
    files = set()
    size = 0
    for item in path.rglob("*"):
        try:
            if not item.is_file():
                continue
            files.add(item.name)
            size += item.stat().st_size
        except OSError:                       # 正在被删的目录：跳过即可
            continue
    row = {"run_id": path.name, "dir": str(path),
           "created": float(stat.st_mtime), "size_bytes": int(size)}
    for key, name in RUN_ARTIFACTS.items():
        row[f"has_{key}"] = name in files
    return row


def list_runs(root=None) -> list[dict]:
    """列出 ``runs/`` 下的运行目录（按 ``created`` 倒序，最新在前）。

    Args:
        root: 产物根目录（``None`` 走 ``jobs.runs_root()``：环境变量
            ``LASER_WEB_RUNS`` > ``<repo>/runs``）。

    Returns:
        list[dict]: ``[{"run_id", "dir", "created", "has_scan", "has_plot",
        "has_joints", "has_repair", "size_bytes"}, ...]``；目录不存在或里面
        没有子目录时返回空列表。
    """
    base = jobs.runs_root(root)
    if not base.is_dir():
        return []
    rows = []
    for path in base.iterdir():
        try:
            if not path.is_dir():
                continue
            rows.append(_run_row(path))
        except OSError:                       # 目录在扫描期间消失：忽略
            continue
    rows.sort(key=lambda row: (row["created"], row["run_id"]), reverse=True)
    return rows


def clean_runs(keep: int = jobs.KEEP_RUNS, root=None) -> dict:
    """按保留策略清理运行目录（调 ``jobs.prune_runs``，不自己删目录）。

    Args:
        keep: 保留最近多少个运行目录（默认 ``jobs.KEEP_RUNS`` = 50；负数按 0
            处理，与 ``jobs.prune_runs`` 的归一规则一致）。
        root: 产物根目录（``None`` 走 ``jobs.runs_root()``）。

    Returns:
        dict: ``{"removed": [被删目录的绝对路径, ...], "kept": 剩余目录数}``。
    """
    base = jobs.runs_root(root)
    # 跳过活跃任务目录：工具页的"清理"可能点在一个任务正在跑的时候
    active = jobs.get_registry().active_dirs()
    removed = [str(path) for path in
               jobs.prune_runs(base, max(0, int(keep)), exclude=active)]
    return {"removed": removed, "kept": len(list_runs(base))}
