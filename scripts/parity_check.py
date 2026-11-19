#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三端数值一致：命令行 / 网页 core / 桌面 core 产出的结果文件逐字节相同。

    python scripts/parity_check.py --quick     # 8 点小域，秒级（提交前跑这个）
    python scripts/parity_check.py             # 站点 50 点（发布前跑这个）
    python scripts/parity_check.py --keep      # 失败留档时不删临时目录

脚本自己把 ``python/`` 放进 ``sys.path``，所以不必先设 ``PYTHONPATH``；只有
子进程（真命令行入口）会显式带上它。

三个入口分别怎么装配参数
------------------------
* **命令行**：``subprocess`` 跑 ``python -m excitation_trajectory.aabb_snake_scan``
  （以及 ``iso9283_cube``）。这是现场与脚本真正用的那条路：参数要过 argparse、
  校验、再进算法，``meta`` 里的 ``aabb_source`` / ``grid_source`` 也由它记录。
* **网页 core**：``ui_state.default_form_values()`` + ``render()`` 补的守卫默认值，
  再按 ``pages/generate.py::collect()`` 的口径收成 ``core.run_scan`` 的入参。
  ``collect()`` 是 ``render()`` 里的闭包、import 不到，所以这里**逐键照抄**一份
  （:func:`web_scan_params`）——改网页表单时这份要一起改，否则这项检查会当场变红，
  这正是它该做的事。
* **桌面 core**：建真的 ``GeneratePage`` / ``IsoPage``（``plot=None``，不碰
  WebEngine），把同一批数值填进控件，再走 ``GeneratePage.values()`` →
  ``form_to_params()``。也就是说桌面这条走的是**真控件**，不是另抄一份字典。

比对什么
--------
1. **参数**：网页入口与桌面入口的参数字典逐键相等（不等就直接报是哪几个键，
   这比"sha 不一样"有用得多）；
2. **产物**：三端各自的 ``scan.json`` / ``iso.json`` 的 sha256 相同（``meta`` 里
   没有时间戳与 run_id，所以同参数下**可以**逐字节比）；
3. **六个统计量**：``solved`` / ``total`` / ``switches`` / ``dq_l2_sum`` /
   ``j5_abs_min`` / ``j3_abs_min``（界面统计卡上那几项）。网页/桌面取
   ``core`` 返回的 ``stats``；命令行没有 ``stats`` 对象，从它写的 JSON 反推
   （:func:`summary_from_payload`，口径照抄 ``core._summary_stats``）。
   JSON 里的浮点是 ``round(x, 6)``，所以反推值与未取整值最多差 ``1e-6``
   ——不是模糊匹配，是"同一个数按 6 位小数落盘"的必然结果。

**这套检查证明的是什么（务必别读多）**
------------------------------------
三个入口**共用** ``laser_web.core`` 这一份算法层：网页与桌面都直接调
``core.run_scan`` / ``core.run_iso9283``，命令行虽然走子进程，但 ``core`` 内部
import 的也是 ``excitation_trajectory.aabb_snake_scan`` 的同一批函数。

所以这项检查证明的是「**三个入口装配出同样的参数**」——表单控件 → 参数字典 →
argparse 名称空间这一路的映射、默认值的来源、互斥项的取舍。它**不**证明
「三套独立实现算出同样结果」；要证明那个得有第二份独立实现，本仓库没有。
这条也印在每次输出里，免得下一份报告把它写成"三端算法互相印证"。

桌面项跳过时
------------
``GeneratePage.values()`` 需要 ``QApplication``：这里用
``QT_QPA_PLATFORM=offscreen`` 离屏建。装不上 Qt（或没有 QtWebEngine 绑定）时
**跳过桌面这一端**并明说跳过了什么、还剩几端在比——不装作三端都跑过。

退出码
------
0：全部一致（含"桌面端跳过、其余两端一致"）；1：任何一项不一致或 CLI 跑失败。
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PYTHON_SRC = REPO / "python"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

#: ``scan.json`` / ``iso.json`` 的文件名（与 ``laser_web.core`` 的产物名一致）。
SCAN_JSON = "scan.json"
ISO_JSON = "iso.json"

#: 逐项比对的六个统计量（与界面统计卡同源，顺序即输出顺序）。
STAT_KEYS = ("solved", "total", "switches", "dq_l2_sum",
             "j5_abs_min", "j3_abs_min")

#: 统计量的浮点容差：JSON 里是 ``round(x, 6)``，反推值最多差半个末位多一点。
STATS_TOL = 1e-6
#: 每次输出都要带上的那句话（别让读者把这项检查当成"三套实现互相印证"）。
LIMITATION = (
    "局限：三个入口共用 laser_web.core 的算法层——本检查证明的是"
    "「三个入口装配出同样的参数」，\n"
    "      不是「三套独立实现算出同样结果」。")

#: 命令行子进程的超时（秒）：50 点实测 20 s 量级，给足余量。
CLI_TIMEOUT = 900


# ---------------------------------------------------------------------------
# 用例
# ---------------------------------------------------------------------------
def _cli_defaults():
    """扫描 CLI 的默认参数（守卫项与 ``q_seed`` 的真来源）。"""
    from excitation_trajectory import aabb_snake_scan as scan

    return scan.build_arg_parser().parse_args([])


def quick_scan_values() -> dict:
    """``--quick``：8 点小域（冒烟脚本与单测用的同一个盒子）。

    Returns:
        dict: 表单值（键与 ``pages/generate.py::collect()`` 一致）。
    """
    return {
        "domain_mode": "axes",
        "aabb6": [900.0, 1100.0, -100.0, 100.0, 500.0, 700.0],
        "grid_mode": "count",
        "target_count": 8,
    }


def site_scan_values() -> dict:
    """默认用例：站点 50 点——**网页/桌面表单的默认值**，不改任何一项。

    Returns:
        dict: 空 dict（= 让两个入口各用自己的默认值）。
    """
    return {}


def iso_values() -> dict:
    """ISO 五点用例：两端口径的默认表单（示例域 550 mm 立方体，5 点全可解）。

    Returns:
        dict: 空 dict（= 让两个入口各用自己的默认值）。
    """
    return {}


# ---------------------------------------------------------------------------
# 网页入口：表单默认值 + collect() 口径
# ---------------------------------------------------------------------------
def web_form_values() -> dict:
    """网页「生成」页打开时表单里的值（``pages/generate.py::render`` 的 defaults）。

    Returns:
        dict: 全部表单值（含未激活的互斥项，``collect()`` 才会挑）。
    """
    from laser_web import ui_state

    defaults = ui_state.default_form_values()
    cli = _cli_defaults()
    defaults.update({
        "j5_min": float(cli.j5_min),
        "j3_min": float(cli.j3_min),
        "j5_attempts": int(cli.j5_attempts),
        "seg_min": float(cli.seg_min),
        # 与网页版同一个有意差异：CLI 默认 PTP，界面默认 MOVL（见
        # pages/generate.py 的 _DEFAULT_SEG_MODE）。
        "seg_mode": "cartesian",
        "seg_samples": int(cli.seg_samples),
        "seed": int(cli.seed),
        "q_seed": str(cli.q_seed),
    })
    return defaults


def web_scan_params(values: dict | None = None) -> dict:
    """网页入口的 ``core.run_scan`` 入参（逐键照抄 ``pages/generate.py::collect``）。

    Args:
        values: 覆盖项（``--quick`` 的采样域/网格）；``None`` 表示全用默认值。

    Returns:
        dict: 只含**当前激活**那一组互斥项的参数字典。
    """
    from laser_web import ui_state

    form = {**web_form_values(), **(values or {})}
    params = {
        "robot": form["robot"],
        "rpy": [form["rpy"][0], form["rpy"][1], form["rpy"][2]],
        "euler": form["euler"],
        "perturb_deg": list(form["perturb_deg"]),
        "cond_max": form["cond_max"],
        "j5_min": form["j5_min"],
        "j3_min": form["j3_min"],
        "j5_attempts": form["j5_attempts"],
        "seg_min": form["seg_min"],
        "seg_mode": form["seg_mode"],
        "seg_samples": form["seg_samples"],
        "seed": form["seed"],
        "q_seed": form["q_seed"],
    }
    if form["domain_mode"] == "points":
        params["point_min"] = list(form["point_min"])
        params["point_max"] = list(form["point_max"])
    else:
        params["aabb6"] = list(form["aabb6"])
    if form["grid_mode"] == "count":
        params["target_count"] = form["target_count"]
    else:
        params["xs"] = ui_state.vertices_to_segments(form["xs"])
        params["ys"] = ui_state.vertices_to_segments(form["ys"])
        params["zs"] = ui_state.vertices_to_segments(form["zs"])
        params["z_insert"] = form["z_insert"]
    return params


def web_iso_params() -> dict:
    """网页入口的 ``core.run_iso9283`` 入参（照抄 ``pages/iso9283.py::collect``）。

    Returns:
        dict: 表单默认值收成的参数（立方体来源只有 ``bounds`` 一种，布点与截面
        固定 ``vertices`` / ``xz``，示例域立方体）。**没有** ``q_seed`` /
        ``j5_min``：ISO 页不显示关节量，界面也不再暴露这两项。
    """
    from laser_web import core, ui_state

    defaults = core.iso_defaults()
    low, high = ui_state.SAMPLE_POINT_MIN, ui_state.SAMPLE_POINT_MAX
    return {
        "robot": ui_state.default_robot(),
        "cube_mode": "bounds",
        "cube": [float(low[0]), float(high[0]), float(low[1]),
                 float(high[1]), float(low[2]), float(high[2])],
        "layout": "vertices",
        "plane": "xz",
        "rpy": [float(value) for value in defaults["rpy"]],
        "euler": str(defaults["euler"]),
    }


# ---------------------------------------------------------------------------
# 桌面入口：真控件 -> values() -> form_to_params()
# ---------------------------------------------------------------------------
class DesktopUnavailable(RuntimeError):
    """没有可用的 Qt 绑定（跳过桌面这一端时抛）。"""


#: 进程内唯一那个 ``QApplication``。**必须留住引用**：Qt 的 app 是 Python
#: 对象持有的，表达式里建完就没人引用时会被回收，紧接着建控件就是
#: ``QWidget: Must construct a QApplication before a QWidget``。
_QT_APP = None


def _qt_app():
    """离屏 ``QApplication``（整个进程共用一个）。

    Raises:
        DesktopUnavailable: 没有 Qt 绑定或没有 QtWebEngine 绑定。
    """
    global _QT_APP
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    try:
        from laser_desktop.qt_compat import QtWidgets
    except ImportError as error:            # 没装 PySide6 / PyQt5+WebEngine
        raise DesktopUnavailable(f"没有可用的 Qt 绑定（{error}）") from error
    if _QT_APP is None:
        _QT_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["parity-check"])
    return _QT_APP


def _set_choice(combo, value, label: str) -> None:
    """按 ``data`` 选中下拉项（找不到就抛，别静默用一个别的值跑）。"""
    index = combo.findData(str(value))
    if index < 0:
        raise DesktopUnavailable(f"{label}下拉里没有 {value!r}（界面与参数表不一致？）")
    combo.setCurrentIndex(index)


def _fill_generate_form(page, params: dict) -> None:
    """把参数字典填进生成页的真控件（``params`` 是 ``collect()`` 口径）。"""
    from laser_web import ui_state

    fields = page.fields
    _set_choice(fields["robot"], params["robot"], "机器人模型")
    if params.get("aabb6") is not None:
        fields["domain_axes"].setChecked(True)
        for key, value in zip(("x_min", "x_max", "y_min", "y_max",
                               "z_min", "z_max"), params["aabb6"]):
            fields[key].setValue(float(value))
    else:
        fields["domain_points"].setChecked(True)
        for key, value in zip(("min_x", "min_y", "min_z"), params["point_min"]):
            fields[key].setValue(float(value))
        for key, value in zip(("max_x", "max_y", "max_z"), params["point_max"]):
            fields[key].setValue(float(value))
    if params.get("target_count") is not None:
        fields["grid_count"].setChecked(True)
        fields["target_count"].setValue(int(params["target_count"]))
    else:
        fields["grid_explicit"].setChecked(True)
        for key in ("xs", "ys", "zs"):
            # 控件按「格点数」输入，参数里是「份数」——界面自己的换算，照用。
            fields[key].setValue(
                ui_state.segments_to_vertices(int(params[key])))
        fields["z_insert"].setValue(int(params.get("z_insert") or 0))
    for key, value in zip(("rpy_a", "rpy_b", "rpy_c"), params["rpy"]):
        fields[key].setValue(float(value))
    _set_choice(fields["euler"], params["euler"], "欧拉约定")
    for key, value in zip(("perturb_a", "perturb_b", "perturb_c"),
                          params["perturb_deg"]):
        fields[key].setValue(float(value))
    fields["seed"].setValue(int(params["seed"]))
    fields["cond_max"].setValue(float(params["cond_max"]))
    fields["j5_min"].setValue(float(params["j5_min"]))
    fields["j3_min"].setValue(float(params["j3_min"]))
    fields["j5_attempts"].setValue(int(params["j5_attempts"]))
    fields["seg_min"].setValue(float(params["seg_min"]))
    _set_choice(fields["seg_mode"], params["seg_mode"], "段模型")
    fields["seg_samples"].setValue(int(params["seg_samples"]))
    fields["q_seed"].setText(str(params["q_seed"]))


def desktop_scan_params(params: dict) -> dict:
    """桌面入口：把参数填进真表单，再走 ``values()`` → ``form_to_params()``。

    Args:
        params: 参数字典（``collect()`` 口径，用来填控件）。

    Returns:
        dict: 桌面页自己装配出来的 ``core.run_scan`` 入参。

    Raises:
        DesktopUnavailable: 没有 Qt，或控件里没有对应的值。
    """
    _qt_app()
    from laser_desktop import generate_page

    page = generate_page.GeneratePage(plot=None)
    _fill_generate_form(page, params)
    return generate_page.form_to_params(page.values())


def desktop_iso_params(params: dict) -> dict:
    """桌面入口：把 ISO 参数填进真表单，再走 ``values()`` -> ``form_to_params()``。

    Args:
        params: ``core.run_iso9283`` 的入参（用来填控件）。

    Returns:
        dict: 桌面 ISO 页自己装配出来的参数。

    Raises:
        DesktopUnavailable: 没有 Qt，或表单不提供这种立方体来源。
    """
    _qt_app()
    from laser_desktop import iso_page

    mode = str(params["cube_mode"])
    if mode != "bounds":
        # 桌面表单只有一种立方体来源（最大/最小点）；其余给法只在命令行里有。
        raise DesktopUnavailable(
            f"桌面 ISO 表单不提供 cube_mode={mode!r}（只有 bounds）")
    if (str(params.get("layout")) != iso_page.FIXED_LAYOUT
            or str(params.get("plane")) != iso_page.FIXED_PLANE):
        # 布点与截面在界面上不可选（固定为 ISO 9283 的取法）。
        raise DesktopUnavailable(
            "桌面 ISO 表单固定 layout=/plane="
            f"{iso_page.FIXED_LAYOUT}/{iso_page.FIXED_PLANE}，"
            f"参数表里是 {params.get('layout')!r}/{params.get('plane')!r}")
    page = iso_page.IsoPage(plot=None)
    fields = page.fields
    _set_choice(fields["robot"], params["robot"], "机器人模型")
    for key, value in zip(("x0", "x1", "y0", "y1", "z0", "z1"),
                          params["cube"]):
        fields[key].setValue(float(value))
    for key, value in zip(("rpy_a", "rpy_b", "rpy_c"), params["rpy"]):
        fields[key].setValue(float(value))
    _set_choice(fields["euler"], params["euler"], "欧拉约定")
    return iso_page.form_to_params(page.values())


# ---------------------------------------------------------------------------
# 命令行入口：参数 -> argv -> 子进程
# ---------------------------------------------------------------------------
def _num(value) -> str:
    """数值 -> 命令行文本（整数值写成 ``8`` 而不是 ``8.0``，int 型选项才收）。"""
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, int):
        return str(value)
    number = float(value)
    return str(int(number)) if number.is_integer() else repr(number)


def scan_argv(params: dict, out_json: Path, out_png: Path) -> list[str]:
    """参数字典 -> 真命令行的 argv（``aabb_snake_scan``）。

    采样域与网格的**写法**决定 ``meta`` 里的 ``aabb_source`` / ``grid_source``，
    所以这里照参数里的实际形状翻：给 ``aabb6`` 就写 6 个位置参数（``axes``），
    给 ``point_min/point_max`` 就写那两个选项（``points``）；网格同理。

    Returns:
        list[str]: ``[python, "-m", ...]`` 形式，可直接喂 ``subprocess``。
    """
    argv = [sys.executable, "-m", "excitation_trajectory.aabb_snake_scan",
            "--urdf", str(params["robot"])]
    if params.get("aabb6") is not None:
        argv += [_num(value) for value in params["aabb6"]]
    else:
        argv += ["--point-min", *[_num(value) for value in params["point_min"]],
                 "--point-max", *[_num(value) for value in params["point_max"]]]
    if params.get("target_count") is not None:
        argv += ["--target-count", _num(params["target_count"])]
    else:
        for key in ("xs", "ys", "zs"):
            argv += [f"--{key}", _num(params[key])]
        if params.get("z_insert"):
            argv += ["--z-insert", _num(params["z_insert"])]
    argv += ["--rpy", *[_num(value) for value in params["rpy"]],
             "--euler", str(params["euler"]),
             "--perturb-deg", *[_num(value) for value in params["perturb_deg"]],
             "--seed", _num(params["seed"]),
             "--cond-max", _num(params["cond_max"]),
             "--j5-min", _num(params["j5_min"]),
             "--j3-min", _num(params["j3_min"]),
             "--j5-attempts", _num(params["j5_attempts"]),
             "--seg-min", _num(params["seg_min"]),
             "--seg-mode", str(params["seg_mode"]),
             "--seg-samples", _num(params["seg_samples"]),
             "--q-seed", str(params["q_seed"]),
             "--out", str(out_json), "--plot", str(out_png)]
    return argv


def iso_argv(params: dict, out_json: Path, out_png: Path) -> list[str]:
    """参数字典 -> 真命令行的 argv（``iso9283_cube``）。

    Returns:
        list[str]: ``[python, "-m", ...]`` 形式，可直接喂 ``subprocess``。
    """
    argv = [sys.executable, "-m", "excitation_trajectory.iso9283_cube",
            "--robot", str(params["robot"])]
    mode = str(params["cube_mode"])
    if mode == "bounds":
        argv += ["--cube", *[_num(value) for value in params["cube"]]]
    elif mode == "auto":
        argv += ["--auto-cube",
                 "--search-box", *[_num(value) for value in params["search_box"]],
                 "--resolution", _num(params["resolution"])]
        if params.get("prefer_x_min") is not None:
            argv += ["--prefer-x-min", _num(params["prefer_x_min"])]
    else:
        raise DesktopUnavailable(
            f"命令行入口与界面口径不同：cube_mode={mode!r} 没有对应写法")
    argv += ["--layout", str(params["layout"]),
             "--plane", str(params["plane"]),
             "--rpy", *[_num(value) for value in params["rpy"]],
             "--euler", str(params["euler"])]
    # 页面只提交坐标与姿态（没有 q_seed / j5_min）：命令行这边不传就是它自己的
    # 默认值，两边算出来的 ``iso.json`` 仍然逐字节相同。
    if params.get("q_seed") is not None:
        argv += ["--q-seed", str(params["q_seed"])]
    if params.get("j5_min") is not None:
        argv += ["--j5-min", _num(params["j5_min"])]
    argv += ["--out", str(out_json), "--plot", str(out_png)]
    return argv


def run_cli(argv: list[str]) -> tuple[int, str]:
    """跑一次命令行入口。

    Args:
        argv: :func:`scan_argv` / :func:`iso_argv` 的返回值。

    Returns:
        ``(returncode, 输出尾部)``；超时按 ``-1`` 返回。
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = str(PYTHON_SRC) + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    try:
        proc = subprocess.run(argv, cwd=str(REPO), env=env, timeout=CLI_TIMEOUT,
                              capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return -1, f"超时（{CLI_TIMEOUT} s）"
    tail = "\n".join((proc.stdout or "").strip().splitlines()[-6:])
    if proc.stderr.strip():
        tail += "\n" + "\n".join(proc.stderr.strip().splitlines()[-6:])
    return proc.returncode, tail


# ---------------------------------------------------------------------------
# 比对
# ---------------------------------------------------------------------------
def sha256_of(path: Path) -> str:
    """文件内容的 sha256（十六进制小写）；文件不存在返回空串。"""
    if not path.is_file():
        return ""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summary_from_payload(payload: dict) -> dict:
    """从 ``scan.json`` 反推六个统计量（口径照抄 ``core._summary_stats``）。

    Args:
        payload: ``scan.json`` 的内容。

    Returns:
        dict: :data:`STAT_KEYS` 六项；没有解算结果时值为 ``None``。
    """
    steps = payload.get("perturbed_ik") or []
    ok_steps = [step for step in steps if step.get("ok")]
    guard = payload.get("singularity_guard") or {}
    if guard:
        j5_abs_min = guard.get("j5_abs_min_deg")
        j3_abs_min = guard.get("j3_abs_min_deg")
    else:
        j5_abs_min = min((abs(step["q"][4]) for step in ok_steps
                          if step.get("q")), default=None)
        j3_abs_min = min((abs(step["q"][2]) for step in ok_steps
                          if step.get("q")), default=None)
    return {
        "solved": len(ok_steps),
        "total": len(steps),
        "switches": sum(1 for first, second in zip(ok_steps, ok_steps[1:])
                        if first.get("config") != second.get("config")),
        "dq_l2_sum": float(sum((step.get("dq_l2") or 0.0) for step in ok_steps)),
        "j5_abs_min": j5_abs_min,
        "j3_abs_min": j3_abs_min,
    }


def _same_number(left, right, tolerance: float = STATS_TOL) -> bool:
    """两个统计数是否相等（``None`` 只等于 ``None``；浮点按 ``tolerance``）。"""
    if left is None or right is None:
        return left is None and right is None
    try:
        return abs(float(left) - float(right)) <= tolerance
    except (TypeError, ValueError):
        return left == right


def _tolerance(key: str, stats: dict) -> float:
    """某一项的容差：求和项随项数放大，其余单项就是 1e-6。

    ``dq_l2_sum`` 是**累加**出来的，而 JSON 里每个 ``dq_l2`` 都单独
    ``round(6)``：n 项最多累计 ``0.5e-6 × n``。50 点实测差 2.5e-6（3.5 倍单项
    容差），对得上这个界。其余项（solved / total / switches / 最小值）要么是
    整数，要么是单个已取整的值，不需要放大。
    """
    if key == "dq_l2_sum":
        return STATS_TOL * max(1, int(stats.get("total") or 1))
    return STATS_TOL


def stats_report(stats: dict) -> str:
    """六个统计量 -> 一行给人看的摘要（``solved/total`` + 其余四项）。"""
    def number(key, digits=3):
        value = stats.get(key)
        return "—" if value is None else f"{float(value):.{digits}f}"

    return (f"stats {stats.get('solved')}/{stats.get('total')}"
            f" · 切换 {stats.get('switches')}"
            f" · ΣdqL2 {number('dq_l2_sum')}"
            f" · |J5|min {number('j5_abs_min', 2)}"
            f" · |J3|min {number('j3_abs_min', 2)}")


def params_diff(left: dict, right: dict) -> list[str]:
    """两份参数字典的差异（只报键与两边的值，别打印整个字典）。

    Returns:
        list[str]: 每条形如 ``键：网页={...} 桌面={...}``；空列表 = 逐键相同。
    """
    lines = []
    for key in sorted(set(left) | set(right)):
        if left.get(key) != right.get(key):
            lines.append(f"{key}：网页={left.get(key)!r} 桌面={right.get(key)!r}")
    return lines


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
class Entry:
    """一个入口的一次产出（名字 / scan.json / iso.json / stats）。"""

    def __init__(self, name: str):
        self.name = name
        self.scan_path: Path | None = None
        self.iso_path: Path | None = None
        self.stats: dict = {}
        self.skipped: str = ""

    def line(self) -> str:
        """三行结果里的那一行（sha 前 8 位 + 六个统计量）。"""
        scan = sha256_of(self.scan_path) if self.scan_path else ""
        iso = sha256_of(self.iso_path) if self.iso_path else ""
        return (f"{self.name:9s} {SCAN_JSON} {scan[:8] or '—':8s} "
                f"{ISO_JSON} {iso[:8] or '—':8s} "
                f"{stats_report(self.stats) if self.stats else 'stats —'}")


def _run_quietly(fn):
    """跑一个会往 stdout/stderr 打算法日志的函数，把日志收起来。

    ``core.run_scan`` / ``core.run_iso9283`` 会把算法自己的打印（条件数、五点表）
    直接写 stdout——那本来是给网页任务日志用的。这里收进缓冲区，只在失败时
    把它拼进错误信息，免得三行结果被几十行日志冲散。

    Args:
        fn: 无参可调用对象。

    Returns:
        ``(返回值, 被吃掉的输出文本)``。
    """
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
        outcome = fn()
    return outcome, buffer.getvalue()


def _run_core(entry: Entry, folder: Path, scan_params: dict | None,
              iso_params: dict | None, errors: list[str]) -> None:
    """跑一次 core（网页 / 桌面入口共用）：把产物路径与 stats 记进 entry。"""
    from laser_web import core

    if scan_params is not None:
        result, output = _run_quietly(lambda: core.run_scan(scan_params))
        if not result.get("ok"):
            errors.append(f"{entry.name} scan：{result.get('error')}\n{output}")
        else:
            entry.scan_path = Path(result["run_dir"]) / SCAN_JSON
            entry.stats = dict(result.get("stats") or {})
    if iso_params is not None:
        result, output = _run_quietly(lambda: core.run_iso9283(iso_params))
        if not result.get("ok"):
            errors.append(f"{entry.name} iso：{result.get('error')}\n{output}")
        else:
            entry.iso_path = Path(result["run_dir"]) / ISO_JSON


def main(argv=None) -> int:
    """入口。返回退出码（0 一致 / 1 不一致或跑失败）。"""
    parser = argparse.ArgumentParser(
        description="三端（命令行 / 网页 core / 桌面 core）数值一致性检查")
    parser.add_argument("--quick", action="store_true",
                        help="只跑 8 点小域（秒级）；默认跑站点 50 点")
    parser.add_argument("--keep", action="store_true",
                        help="保留临时目录（默认成功即删、失败留档）")
    args = parser.parse_args(argv)

    work = Path(tempfile.mkdtemp(prefix="parity-check-"))
    # 三个入口的 runs/ 都落在这里：不往仓库 runs/ 里堆测试产物，也不受上一轮
    # 遗留目录影响（jobs 的保留策略会删最旧的，混用会误删别人的记录）。
    os.environ["LASER_WEB_RUNS"] = str(work / "runs")
    cli_dir = work / "cli"
    cli_dir.mkdir(parents=True, exist_ok=True)

    case = "8 点小域（--quick）" if args.quick else "站点 50 点（表单默认值）"
    print(f"用例：{case}")
    print(f"仓库：{REPO}")

    errors: list[str] = []
    web_values = quick_scan_values() if args.quick else site_scan_values()
    scan_params = web_scan_params(web_values)
    iso_params = web_iso_params()

    # ---- 1. 参数：网页 vs 桌面（不一致就直说哪几个键） ----
    desktop_ready = True
    desktop_scan = None
    desktop_iso = None
    try:
        desktop_scan = desktop_scan_params(scan_params)
        desktop_iso = desktop_iso_params(iso_params)
    except DesktopUnavailable as error:
        desktop_ready = False
        skipped_reason = str(error)
    if desktop_ready:
        differences = (params_diff(scan_params, desktop_scan)
                       + [f"iso/{line}" for line in
                          params_diff(iso_params, desktop_iso)])
        if differences:
            print("参数：网页入口与桌面入口**不一致**——")
            for line in differences:
                print(f"  {line}")
            errors.append("参数装配不一致（见上）")
        else:
            print(f"参数：网页入口与桌面入口逐键相同"
                  f"（scan {len(scan_params)} 项 / iso {len(iso_params)} 项）")

    # ---- 2. 三端跑一遍 ----
    entries = [Entry("命令行"), Entry("网页 core"), Entry("桌面 core")]
    cli_scan = cli_dir / "cli" / SCAN_JSON
    cli_scan.parent.mkdir(parents=True, exist_ok=True)
    code, tail = run_cli(scan_argv(scan_params, cli_scan,
                                   cli_scan.with_suffix(".png")))
    if code != 0 or not cli_scan.is_file():
        errors.append(f"命令行 scan 跑失败（退出码 {code}）：\n{tail}")
    else:
        entries[0].scan_path = cli_scan
        entries[0].stats = summary_from_payload(
            json.loads(cli_scan.read_text(encoding="utf-8")))
    cli_iso = cli_dir / "iso" / ISO_JSON
    cli_iso.parent.mkdir(parents=True, exist_ok=True)
    code, tail = run_cli(iso_argv(iso_params, cli_iso,
                                  cli_iso.with_suffix(".png")))
    if code != 0 or not cli_iso.is_file():
        errors.append(f"命令行 iso 跑失败（退出码 {code}）：\n{tail}")
    else:
        entries[0].iso_path = cli_iso

    _run_core(entries[1], work / "web", scan_params, iso_params, errors)
    if desktop_ready:
        _run_core(entries[2], work / "desktop", desktop_scan, desktop_iso, errors)
    else:
        entries[2].skipped = skipped_reason

    for entry in entries:
        if entry.skipped:
            # 跳过的原因可能自带多行安装提示：压成一行，否则"三行结果"会被打断
            reason = " ".join(str(entry.skipped).split())
            print(f"{entry.name:9s} 跳过：{reason}")
        else:
            print(entry.line())

    # ---- 3. 结论 ----
    compared = [entry for entry in entries if not entry.skipped]
    if len(compared) >= 2:
        scan_shas = {sha256_of(entry.scan_path) for entry in compared
                     if entry.scan_path}
        iso_shas = {sha256_of(entry.iso_path) for entry in compared
                    if entry.iso_path}
        if len(scan_shas) != 1:
            errors.append("scan.json 不是逐字节相同："
                          + "、".join(sorted(sha[:8] or "—" for sha in scan_shas)))
        if len(iso_shas) != 1:
            errors.append("iso.json 不是逐字节相同："
                          + "、".join(sorted(sha[:8] or "—" for sha in iso_shas)))
        base_stats = compared[0].stats
        for entry in compared[1:]:
            for key in STAT_KEYS:
                if not _same_number(base_stats.get(key), entry.stats.get(key),
                                    _tolerance(key, base_stats)):
                    errors.append(f"stats.{key} 不一致："
                                  f"{compared[0].name}={base_stats.get(key)!r} "
                                  f"{entry.name}={entry.stats.get(key)!r}")

    print()
    if errors:
        print("结论：**不一致**（退出码 1）")
        for line in errors:
            print(f"  · {line}")
        print(f"临时目录已保留：{work}")
        print(LIMITATION)
        return 1

    ends = "三端一致" if desktop_ready else f"{len(compared)} 端一致（桌面入口跳过）"
    print(f"结论：{ends} —— {SCAN_JSON} / {ISO_JSON} 逐字节相同，stats 六项相同。")
    print(LIMITATION)
    if args.keep:
        print(f"临时目录已保留：{work}")
    else:
        shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
