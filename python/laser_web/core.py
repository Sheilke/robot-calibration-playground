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
``run_scan`` / ``build_gp`` / ``convert_urdf`` / ``locate_segment`` / ``run_repair``
留给 M2 ~ M4。
"""

from __future__ import annotations

import argparse
import contextlib
import io
from pathlib import Path

from excitation_trajectory import aabb_snake_scan as scan

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
            ``cond_max``。值为 ``None`` 的键视为「没填」，保留 CLI 默认值。

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
    return args


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
