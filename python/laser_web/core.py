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
``build_gp`` / ``convert_urdf`` / ``locate_segment`` / ``run_repair`` 留给 M3 ~ M4。
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
from pathlib import Path

import numpy as np

from excitation_trajectory import aabb_snake_scan as scan

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
    """写 ``joints.txt``：每行 6 个关节角（deg，逗号分隔，3 位小数，GP 顺序）。

    只写有解的点——GP 程序里没法下发一个「无解」行，缺的点在 ``warnings``
    里单独报。

    Args:
        path: 输出路径。
        chain: ``ChainStep`` 序列（GP 顺序）。

    Returns:
        int: 写出的行数（= 有解路点数）。
    """
    lines = [",".join(f"{float(value):.3f}" for value in step.q)
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


def _execute(args: argparse.Namespace, q_seed: list[float], job,
             progress: _StageReporter, warnings: list[str]) -> dict:
    """``run_scan`` 的主体（校验已通过，产物目录在这里建）。

    顺序与设计文档 §8 的 M2 任务项一致：规划路点 -> 采样扰动 -> 链式 IK ->
    段内检查 -> 条件数 -> 出图 -> 写文件；每两个阶段之间检查一次取消标志。
    """
    from robot_model.nexbot_ik import Nexbot6R

    run_id, run_dir = ((str(job.id), Path(job.run_dir)) if job is not None
                       else jobs.make_run_dir())
    order = scan._order_of(args)
    extrinsic = scan._extrinsic_of(args)
    rpy_ref = tuple(float(value) for value in args.rpy)
    skip_ik = bool(args.skip_ik)

    rbt = Nexbot6R(args.urdf)

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

    # ---- 5. 姿态激励条件数（设计值 + 实测值 + IK 后实测值） ----
    _check_cancel(job)
    progress(STAGES[4], 0, 2)
    conditioning = scan.evaluate_conditioning(args, waypoints, rpy_ref, poses)
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
    feasible = None
    if not skip_ik:
        feasible = (bool(ok_steps) and len(ok_steps) == len(chain)
                    and j3_crossings == 0 and j5_crossings == 0
                    and conditioning_json["status"] != "rejected")
    _scan_warnings(args, warnings, chain, guard_report, segment_report,
                   conditioning_json)
    stats = {"solved": len(ok_steps), "total": len(chain),
             "switches": switches, "dq_l2_sum": dq_l2_sum,
             "j5_abs_min": j5_abs_min, "j3_abs_min": j3_abs_min,
             "design_cond": design, "post_ik_cond": post_ik_cond,
             "seg_j5_min": seg_j5_min, "seg_j3_min": seg_j3_min,
             "j3_crossings": j3_crossings, "j5_crossings": j5_crossings,
             "feasible": feasible}
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
