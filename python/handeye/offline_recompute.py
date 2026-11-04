# -*- coding: utf-8 -*-
"""旧/新 DH 离线重算：同一批采样、只换运动学模型，比较手眼 VPE。

背景
----
现场用「激光跟踪仪 + 相机看固定标准靶球」验证手眼标定精度。手眼矩阵 ``X``
（flange←camera）由公司 C++ 求解器（ROS2 action
``calibration/eye_on_hand_calibration``）算出，它的输入只有 **笛卡尔位姿**，
不知道关节角、也不知道 DH。于是「换一套 DH 是否让手眼精度变好」这件事可以
做成一次 **离线重算实验**：同一批采样（同样的图像、同样的靶球点、同样的关节
读数），只换运动学模型，各解一次 X，再在 **未参与求解** 的验证样本上比 VPE::

    X_old = Solve(poses_from(θ_old))     # 旧 DH
    X_new = Solve(poses_from(θ_new))     # 新 DH（DH 标定结果）
    VPE(θ_old, X_old) vs VPE(θ_new, X_new)     # 只在验证子集上比较

整条链完全复用仓库已有能力，**求解器一行不改**：

``session.json(q_deg)`` --dh.py--> ``T_bh`` --pb_rewrite.py--> 改写后的 ``.pb``
目录 --> 公司动作 ``calibration/eye_on_hand_calibration`` -->
``EyeOnHandCalibrationResult{matrix[16], compute_error}`` --> ``vpe.py`` 算 VPE。

公司 C++ 侧契约（本模块按此实现，不需要在容器里装 protobuf）
-----------------------------------------------------------
* 动作：``calibration/eye_on_hand_calibration``，类型
  ``arc_msgs/action/ArcAction``；Goal 的 ``data`` 字段是序列化的
  ``calibration.EyeOnHandCalibrationGoal``，只有一个字段
  ``string input_directory = 1``，因此 goal 字节可手工编码
  （见 :func:`encode_goal_payload`）：``0x0a`` + varint 长度 + 路径字节。
* Result 的 ``data`` 字段是序列化的
  ``calibration.EyeOnHandCalibrationResult{bool success=1; repeated double
  matrix=2; double compute_error=3;}``；``matrix`` 是 **16 个 double、
  row-major、平移单位 mm**（整个求解过程用 robot_pt 的 mm 计算）。
  本模块用 :func:`parse_calibration_result` 手写 wire format 解析它。
* 目录里的 ``.pb`` 会被 **全部** 读取并参与求解，所以喂给求解器的目录只放
  **标定子集**（验证子集必须留在外面，否则就是「用验证数据求解」）。

关键约定
--------
* 旧一侧的位姿来源可选：``recorded``（默认）保留控制器回传的 ``robot_pt``
  不改写——生产上的 X_old 就是用控制器位姿算的；``fk_old`` 则用
  FK(θ_old) 改写，用于「模型 vs 模型」的纯离线对照。
* 4 种组合里 ``(θ_old, X_old)`` 是基线、``(θ_new, X_new)`` 是主结果；另外两
  个交叉组合只作误差来源诊断，**没有物理一致性**，不作最终性能结论。
* 组合的 VPE 用 :func:`handeye.vpe.compute_vpe` 在「只含验证子集、且运动学
  为指定 DH」的 **浅拷贝视图** 上计算（见 :func:`_subset_session`），不重新
  读 JSON、不改动调用方的 Session。
* ``work_dir`` 为 ``None`` 时用 :func:`tempfile.mkdtemp` 建临时目录并在结束时
  自动清理；调用方显式给 ``work_dir`` 时不删除，便于事后检查中间 ``.pb``。

CLI
---
::

    python -m handeye.offline_recompute --pb-dir DIR --session SESSION.json \\
        --dh-old OLD.json --dh-new NEW.json [选项]

默认求解命令见 :data:`DEFAULT_SOLVER_CMD`（需要容器内可用的 ``ros2``；本仓库
**不自动执行 docker**，请按环境替换 ``--solver-cmd``）。退出码：成功 ``0``，
参数/数据/求解错误 ``2``（只在 stderr 给提示，不抛 traceback）。

库函数不打印、不 ``sys.exit``；打印与返回码由 :func:`main` 负责。
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

from handeye.dh import DhOverride, RobotKinematics
from handeye.pb_rewrite import rewrite_directory
from handeye.proto_codec import ProtoUnavailableError
from handeye.report import write_markdown
from handeye.se3 import load_matrix16, to_matrix16
from handeye.session import (
    REPO_ROOT,
    SessionError,
    check_fk_consistency,
    load_session,
)
from handeye.vpe import VpeError, compare_matrices, compute_vpe

#: 默认求解命令模板（占位符由 :class:`CommandSolverBackend` 填充）。
#:
#: 该模板假设「容器内有可用的 ``ros2``」，并由容器内的一个小工具把
#: ``EyeOnHandCalibrationGoal`` 发到动作 ``/calibration/eye_on_hand_calibration``、
#: 把序列化后的 ``EyeOnHandCalibrationResult`` 落到 ``{output_bin}``。
#: 本仓库不自动执行 docker；现场请用 ``--solver-cmd`` 换成自己的调用方式
#: （模板里的字面花括号须写成 ``{{`` / ``}}``）。
DEFAULT_SOLVER_CMD = (
    "docker exec arc-eye-on-hand bash -lc \"ros2 run calibration_py "
    "eye_on_hand_calibration_solver --input-dir '{input_dir}' "
    "--output-bin '{output_bin}' --output-json '{output_json}'\""
)

#: 旧 DH 一侧的位姿来源：保留控制器回传位姿 / 用 FK(θ_old) 改写。
POSE_SOURCES_OLD = ("recorded", "fk_old")

#: ``CommandSolverBackend`` 的后端名（写进结果），便于报告区分。
_COMMAND_BACKEND_NAME = "command"

#: 标定子集最少样本数（求解器需要解 6 自由度，3 个位姿是下限）。
MIN_CALIB_SAMPLES = 3

#: 验证子集最少样本数（至少要有一个未参与求解的样本才能谈泛化）。
MIN_VALID_SAMPLES = 1

#: 视为「零」的数值下限（与 ``handeye.report`` 的格式约定一致）。
_ZERO_EPSILON = 5e-5

#: 4 种组合的固定顺序与标记：``(标签, DH 侧, X 侧, 基线, 主结果)``。
COMBINATION_SPECS = (
    ("θ_old × X_old", "old", "old", True, False),
    ("θ_new × X_new", "new", "new", False, True),
    ("θ_new × X_old", "new", "old", False, False),
    ("θ_old × X_new", "old", "new", False, False),
)

#: wire type（protobuf 编码的 3 位类型字段）。
_WIRE_VARINT = 0
_WIRE_FIXED64 = 1
_WIRE_LENGTH_DELIMITED = 2
_WIRE_FIXED32 = 5


# ---------------------------------------------------------------------------
# 求解器后端
# ---------------------------------------------------------------------------


class SolverError(RuntimeError):
    """求解器调用失败（命令非零退出、结果缺失、结果不合法）。"""


class SolverBackend:
    """求解手眼的抽象后端。

    输入一个「只含标定子集 ``.pb``」的目录，返回 4×4 矩阵（mm）。子类只需
    实现 :meth:`solve`；本基类不可直接使用（:meth:`solve` 抛
    ``NotImplementedError``）。

    Attributes:
        name: 后端名称（写进结果 ``solver[*]["backend"]``，便于报告区分）。
    """

    #: 后端名称（子类覆盖）。
    name: str = "abstract"

    def solve(self, input_dir) -> tuple[np.ndarray, dict]:
        """求解 ``input_dir`` 里的 ``.pb``，返回 ``(matrix 4x4 mm, info dict)``。

        Args:
            input_dir: 只含标定子集 ``*.pb`` 的目录。

        Returns:
            ``(np.ndarray(4, 4), info)``；``info`` 至少含 ``{"backend": str,
            "raw": ...}``，``raw`` 为后端原始结果（本仓库里是
            :func:`parse_calibration_result` 的返回值）。

        Raises:
            NotImplementedError: 基类未实现。
        """
        raise NotImplementedError(
            "SolverBackend.solve 是抽象方法，请使用 CommandSolverBackend "
            "或在测试里实现一个假后端")


def _absolute_path(value) -> Path:
    """``str`` / ``Path`` -> 绝对路径（展开 ``~``，不解析符号链接、不要求存在）。"""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return (Path.cwd() / path).absolute()


def _tail(text, limit: int = 2000) -> str:
    """取文本末尾 ``limit`` 个字符（命令输出可能很长，避免撑爆报告/异常）。"""
    if not text:
        return ""
    text = str(text)
    return text if len(text) <= limit else "…" + text[-limit:]


class CommandSolverBackend(SolverBackend):
    """用外部命令求解（对接公司 ``docker + ros2`` 调用，也便于测试）。

    命令模板支持占位符 ``{input_dir}`` / ``{output_bin}`` / ``{output_json}``，
    以及调用方通过 ``extra_format`` 提供的任意附加占位符（例如测试用的
    ``{preset_bin}``）。命令执行后从 ``output_bin`` 读取 **序列化的
    ``calibration.EyeOnHandCalibrationResult``** 并用
    :func:`parse_calibration_result` 解析。

    模板替换 **不自动加引号**：路径可能含空格时请在模板里自行加引号；模板里
    的字面花括号要写成 ``{{`` / ``}}``。

    Attributes:
        name: 固定为 ``"command"``。
        command: 命令模板。
        timeout_s: 单次命令超时（秒）。
        work_dir: 结果落盘根目录；``None`` 表示每次调用新建临时目录并在成功
            读取结果后删除（失败时保留，路径写进异常消息便于排查）。
        env: 追加/覆盖到当前进程环境变量之上的映射；``None`` 表示不修改。
        extra_format: 附加占位符取值。
    """

    name = _COMMAND_BACKEND_NAME

    def __init__(self, command, *, timeout_s=600, work_dir=None, env=None,
                 extra_format=None):
        """保存命令模板与运行参数。

        Args:
            command: 非空命令模板字符串（``str.format`` 语法）。
            timeout_s: 超时秒数，必须为有限正数。
            work_dir: 结果落盘根目录（``None`` = 每次调用用临时目录）。
            env: 额外环境变量映射（在 ``os.environ`` 副本之上覆盖）。
            extra_format: 附加占位符取值（``dict``）；模板里出现但这里没有
                的占位符会抛带占位符名字的 :class:`SolverError`。

        Raises:
            ValueError: 模板为空 / 超时为非有限正数 / ``extra_format`` 不是
                ``dict`` / ``env`` 不是 ``dict``。
        """
        if not isinstance(command, str) or not command.strip():
            raise ValueError("command 必须为非空命令模板字符串")
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError(f"timeout_s 必须为有限正数，当前 {timeout_s!r}")
        if env is not None and not isinstance(env, dict):
            raise ValueError(
                f"env 必须为 dict 或 None，当前 {type(env).__name__}")
        if extra_format is not None and not isinstance(extra_format, dict):
            raise ValueError(
                f"extra_format 必须为 dict 或 None，当前 "
                f"{type(extra_format).__name__}")
        self.command = command
        self.timeout_s = timeout
        self.work_dir = None if work_dir is None else Path(work_dir).expanduser()
        self.env = None if env is None else {
            str(key): str(value) for key, value in env.items()}
        self.extra_format = {
            str(key): str(value) for key, value in (extra_format or {}).items()}
        self._calls = 0

    def _render(self, values: dict) -> str:
        """按 ``values`` 渲染命令模板，缺键/花括号错误统一转 :class:`SolverError`。"""
        try:
            return self.command.format(**values)
        except KeyError as error:
            missing = error.args[0] if error.args else "?"
            raise SolverError(
                f"求解命令模板含未知占位符 {{{missing}}}；"
                f"请通过 extra_format 提供取值（已提供：{sorted(values)}）") from error
        except (IndexError, ValueError) as error:
            raise SolverError(
                f"求解命令模板无法格式化（字面花括号需写成 {{{{ }}}}）：{error}"
            ) from error

    def solve(self, input_dir) -> tuple[np.ndarray, dict]:
        """执行一次求解命令并解析结果。

        Args:
            input_dir: 只含标定子集 ``*.pb`` 的目录。

        Returns:
            ``(matrix 4x4 mm, info)``，``info`` 含 ``backend``、``command``、
            ``returncode``、``stdout``、``stderr``、``result_bin`` 与 ``raw``
            （:func:`parse_calibration_result` 的返回值）。

        Raises:
            SolverError: 输入目录不存在 / 模板缺占位符 / 命令无法执行或超时 /
                返回码非 0 / 结果文件缺失 / 结果解析失败 / ``success=false`` /
                矩阵不是合法刚体矩阵。
        """
        source = _absolute_path(input_dir)
        if not source.is_dir():
            raise SolverError(f"求解输入目录不存在：{source}")

        self._calls += 1
        temporary = self.work_dir is None
        run_dir = (Path(tempfile.mkdtemp(prefix="handeye_solver_"))
                   if temporary else self.work_dir / f"call_{self._calls:02d}")
        run_dir.mkdir(parents=True, exist_ok=True)

        output_bin = run_dir / "result.bin"
        output_json = run_dir / "result.json"
        values = {
            "input_dir": str(source),
            "output_bin": str(output_bin),
            "output_json": str(output_json),
        }
        values.update(self.extra_format)
        rendered = self._render(values)

        env = None
        if self.env is not None:
            env = {**os.environ, **self.env}

        try:
            completed = subprocess.run(
                rendered, shell=True, cwd=str(run_dir), env=env,
                capture_output=True, text=True, timeout=self.timeout_s)
        except subprocess.TimeoutExpired as error:
            raise SolverError(
                f"求解命令超时（>{self.timeout_s:g} s）：{rendered}\n"
                f"结果目录：{run_dir}") from error
        except OSError as error:
            raise SolverError(
                f"无法执行求解命令：{rendered}（{error}）") from error

        if completed.returncode != 0:
            raise SolverError(
                f"求解命令返回码 {completed.returncode}：{rendered}\n"
                f"stdout：{_tail(completed.stdout)}\n"
                f"stderr：{_tail(completed.stderr)}\n"
                f"结果目录：{run_dir}")
        if not output_bin.is_file():
            raise SolverError(
                f"求解命令返回成功（0）但未产出结果文件：{output_bin}\n"
                f"命令：{rendered}\n结果目录：{run_dir}")

        try:
            data = output_bin.read_bytes()
        except OSError as error:
            raise SolverError(f"无法读取求解结果文件 {output_bin}：{error}") from error

        parsed = parse_calibration_result(data)
        if not parsed["success"]:
            raise SolverError(
                f"求解器返回 success=false（compute_error="
                f"{parsed['compute_error']!r}）：{rendered}\n"
                f"结果文件：{output_bin}")
        try:
            matrix = load_matrix16(parsed["matrix"], units="mm")
        except ValueError as error:
            raise SolverError(f"求解结果不是合法刚体矩阵：{error}") from error

        info = {
            "backend": self.name,
            "command": rendered,
            "returncode": int(completed.returncode),
            "stdout": _tail(completed.stdout),
            "stderr": _tail(completed.stderr),
            "result_bin": str(output_bin) if not temporary else output_bin.name,
            "raw": parsed,
        }
        if temporary:
            shutil.rmtree(run_dir, ignore_errors=True)
        return matrix, info


# ---------------------------------------------------------------------------
# protobuf wire format（手写，不依赖 protobuf 运行时）
# ---------------------------------------------------------------------------


def varint_encode(value: int) -> bytes:
    """把非负整数编码成标准 base-128 小端变长字节串。

    Args:
        value: 非负整数（``bool`` 与非整数拒绝）。

    Returns:
        ``bytes``；``0`` / ``127`` 编码为 1 字节，``128`` 起为 2 字节……

    Raises:
        ValueError: ``value`` 不是整数或为负数。
    """
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"varint 只接受整数，当前 {type(value).__name__}")
    value = int(value)
    if value < 0:
        raise ValueError(f"varint 不支持负数，当前 {value}")
    out = bytearray()
    while True:
        bits = value & 0x7F
        value >>= 7
        if value:
            out.append(bits | 0x80)
        else:
            out.append(bits)
            return bytes(out)


def varint_decode(buffer, offset: int = 0) -> tuple[int, int]:
    """从 ``buffer[offset:]`` 解码一个 base-128 变长整数。

    Args:
        buffer: ``bytes`` / ``bytearray`` / ``memoryview``。
        offset: 起始字节偏移，默认 0。

    Returns:
        ``(value, next_offset)``：``value`` 为解码出的整数，``next_offset`` 为
        紧随其后的字节偏移（便于顺序解析多个字段）。

    Raises:
        ValueError: ``buffer`` 类型不支持 / ``offset`` 越界 / 变长整数被截断
            （没有终止字节）或超过 10 字节。
    """
    if isinstance(buffer, (bytes, bytearray, memoryview)):
        data = bytes(buffer)
    else:
        raise ValueError(
            f"varint_decode 只接受 bytes/bytearray/memoryview，"
            f"当前 {type(buffer).__name__}")
    if isinstance(offset, bool) or not isinstance(offset, (int, np.integer)):
        raise ValueError(f"offset 必须为整数，当前 {type(offset).__name__}")
    offset = int(offset)
    if offset < 0 or offset > len(data):
        raise ValueError(f"offset {offset} 超出范围 [0, {len(data)}]")

    value = 0
    shift = 0
    index = offset
    while True:
        if index >= len(data):
            raise ValueError(
                f"varint 截断：偏移 {offset} 起没有终止字节"
                f"（缓冲区长度 {len(data)}）")
        byte = data[index]
        index += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, index
        shift += 7
        if shift >= 70:
            raise ValueError(f"varint 过长（超过 10 字节），起始偏移 {offset}")


def encode_goal_payload(input_directory) -> bytes:
    """编码 ``calibration.EyeOnHandCalibrationGoal{input_directory}``。

    该 message 只有 ``string input_directory = 1``，所以字节就是
    ``0x0a``（字段 1 + wire type 2）+ varint 长度 + UTF-8 路径。

    Args:
        input_directory: 输入目录（``str`` / ``Path``）。

    Returns:
        ``bytes``：可直接塞进 ``ArcAction`` goal 的 ``data`` 字段。

    Raises:
        ValueError: 路径为空。
    """
    if input_directory is None:
        raise ValueError("input_directory 不能为 None")
    text = str(input_directory)
    if not text.strip():
        raise ValueError("input_directory 不能为空字符串")
    payload = text.encode("utf-8")
    return b"\x0a" + varint_encode(len(payload)) + payload


def _parse_error(message: str) -> SolverError:
    """构造带统一前缀的解析错误。"""
    return SolverError(f"解析 EyeOnHandCalibrationResult 失败：{message}")


def _skip_field(data: bytes, offset: int, wire_type: int) -> int:
    """跳过未知字段，返回下一个字段的偏移。

    Args:
        data: 完整字节串。
        offset: 值（payload）的起始偏移。
        wire_type: 字段的 wire type。

    Returns:
        下一个字段的偏移。

    Raises:
        SolverError: 未知 wire type（含已废弃的 group）或长度越界。
    """
    total = len(data)
    if wire_type == _WIRE_VARINT:
        _, next_offset = varint_decode(data, offset)
        return next_offset
    if wire_type == _WIRE_FIXED64:
        if offset + 8 > total:
            raise _parse_error(f"固定 64 位字段被截断（偏移 {offset}）")
        return offset + 8
    if wire_type == _WIRE_LENGTH_DELIMITED:
        length, start = varint_decode(data, offset)
        end = start + length
        if end > total:
            raise _parse_error(
                f"长度前缀字段越界（偏移 {offset} 起声明 {length} 字节，"
                f"缓冲区长度 {total}）")
        return end
    if wire_type == _WIRE_FIXED32:
        if offset + 4 > total:
            raise _parse_error(f"固定 32 位字段被截断（偏移 {offset}）")
        return offset + 4
    raise _parse_error(f"不支持的 wire type {wire_type}（偏移 {offset}）")


def parse_calibration_result(data_bytes) -> dict:
    """解析序列化的 ``calibration.EyeOnHandCalibrationResult``。

    手写 wire format 解析，**不依赖 protobuf 运行时**：

    * ``field 1`` = ``bool success``（wire type 0 / varint）；
    * ``field 2`` = ``repeated double matrix``，同时支持 packed（wire type 2，
      ``8N`` 字节的定长串）与逐个 wire type 1（每个 8 字节小端 double）；
    * ``field 3`` = ``double compute_error``（wire type 1）；
    * 其它字段按 wire type 跳过，字段号收集进 ``unknown_fields``。

    重复出现的 ``field 1`` / ``field 3`` 按 protobuf 语义取 **最后一次**；
    缺失时分别取默认值 ``False`` / ``0.0``。

    Args:
        data_bytes: 结果字节串（``bytes`` / ``bytearray`` / ``memoryview``）。

    Returns:
        ``{"success": bool, "matrix": list[16 float], "compute_error": float,
        "unknown_fields": [int, ...]}``。

    Raises:
        SolverError: 输入为空 / 不是字节串 / wire format 非法或截断 /
            matrix 不是 16 个 double / 含非有限值。
    """
    if isinstance(data_bytes, (bytes, bytearray, memoryview)):
        data = bytes(data_bytes)
    else:
        raise _parse_error(
            f"data_bytes 需为 bytes，当前 {type(data_bytes).__name__}")
    if not data:
        raise _parse_error("输入为空（0 字节）")

    success = False
    doubles: list = []
    compute_error = 0.0
    unknown_fields: list = []
    total = len(data)
    offset = 0

    while offset < total:
        try:
            key, offset = varint_decode(data, offset)
        except ValueError as error:
            raise _parse_error(str(error)) from error
        field_number, wire_type = key >> 3, key & 0x07
        if field_number == 0:
            raise _parse_error("字段号为 0（wire format 非法）")

        if field_number == 1:
            if wire_type != _WIRE_VARINT:
                raise _parse_error(
                    f"field 1 (success) 需为 varint（wire type 0），"
                    f"当前 wire type {wire_type}")
            try:
                raw, offset = varint_decode(data, offset)
            except ValueError as error:
                raise _parse_error(f"field 1 (success)：{error}") from error
            success = bool(raw)
        elif field_number == 2:
            if wire_type == _WIRE_LENGTH_DELIMITED:
                try:
                    length, start = varint_decode(data, offset)
                except ValueError as error:
                    raise _parse_error(f"field 2 (matrix) 长度：{error}") from error
                end = start + length
                if end > total:
                    raise _parse_error(
                        f"field 2 (matrix) 越界：偏移 {offset} 起声明 {length} "
                        f"字节，缓冲区长度 {total}")
                if length % 8:
                    raise _parse_error(
                        f"field 2 (matrix) packed 长度 {length} 不是 8 的整数倍")
                doubles.extend(struct.unpack(f"<{length // 8}d", data[start:end]))
                offset = end
            elif wire_type == _WIRE_FIXED64:
                if offset + 8 > total:
                    raise _parse_error(
                        f"field 2 (matrix) 固定 64 位值被截断（偏移 {offset}）")
                doubles.append(struct.unpack("<d", data[offset:offset + 8])[0])
                offset += 8
            else:
                raise _parse_error(
                    f"field 2 (matrix) 需为 packed（wire type 2）或 double"
                    f"（wire type 1），当前 wire type {wire_type}")
        elif field_number == 3:
            if wire_type != _WIRE_FIXED64:
                raise _parse_error(
                    f"field 3 (compute_error) 需为 double（wire type 1），"
                    f"当前 wire type {wire_type}")
            if offset + 8 > total:
                raise _parse_error(
                    f"field 3 (compute_error) 固定 64 位值被截断（偏移 {offset}）")
            compute_error = struct.unpack("<d", data[offset:offset + 8])[0]
            offset += 8
        else:
            unknown_fields.append(int(field_number))
            try:
                offset = _skip_field(data, offset, wire_type)
            except ValueError as error:
                raise _parse_error(str(error)) from error

    if len(doubles) != 16:
        raise _parse_error(f"matrix 需 16 个 double，当前 {len(doubles)} 个")
    if not all(math.isfinite(value) for value in doubles):
        raise _parse_error("matrix 含非有限值（nan / inf）")
    if not math.isfinite(compute_error):
        raise _parse_error(f"compute_error 非有限值：{compute_error!r}")

    return {
        "success": bool(success),
        "matrix": [float(value) for value in doubles],
        "compute_error": float(compute_error),
        "unknown_fields": unknown_fields,
    }


# ---------------------------------------------------------------------------
# 标定 / 验证子集划分
# ---------------------------------------------------------------------------


def _pose_id_of(sample) -> int:
    """从 ``Sample`` / ``dict`` / 整数取出 ``pose_id``（整数）。"""
    if isinstance(sample, bool):
        raise ValueError("pose_id 不能是 bool")
    if isinstance(sample, (int, np.integer)):
        return int(sample)
    if isinstance(sample, dict):
        if "pose_id" not in sample:
            raise ValueError(f"样本字典缺少 'pose_id'：{sample!r}")
        return _pose_id_of(sample["pose_id"])
    if hasattr(sample, "pose_id"):
        return _pose_id_of(getattr(sample, "pose_id"))
    raise ValueError(f"无法从 {type(sample).__name__} 取出 pose_id")


def _normalize_ids(values, name: str, known: set) -> list:
    """把显式 id 列表规范化：转 ``int``、查重、检查存在性，按 ``pose_id`` 升序返回。

    Args:
        values: ``None`` 或 id 序列（``int`` / ``np.integer``）。
        name: 报错时使用的参数名（如 ``"calib_ids"``）。
        known: 会话里实际存在的 ``pose_id`` 集合。

    Returns:
        排序后的 ``list[int]``；``values is None`` 时返回 ``None``。

    Raises:
        ValueError: 不是可迭代序列 / 元素不是整数 / 重复 / 会话里不存在。
    """
    if values is None:
        return None
    if isinstance(values, (str, bytes)) or not hasattr(values, "__iter__"):
        raise ValueError(
            f"{name} 必须为整数序列，当前 {type(values).__name__}")
    result: list = []
    for item in values:
        if isinstance(item, bool) or not isinstance(item, (int, np.integer)):
            raise ValueError(f"{name} 只接受整数 pose_id，当前 {item!r}")
        value = int(item)
        if value in result:
            raise ValueError(f"{name} 含重复 pose_id {value}")
        if value not in known:
            raise ValueError(
                f"{name} 含会话里不存在的 pose_id {value}"
                f"（已有 {sorted(known)}）")
        result.append(value)
    return sorted(result)


def split_ids(samples, *, calib_ids=None, valid_ids=None, split_ratio=0.7,
              seed=0) -> tuple[list[int], list[int]]:
    """决定哪些 ``pose_id`` 参与求解（标定）、哪些用于验证。

    规则（按顺序判断）：

    1. 显式给 ``calib_ids`` 与 ``valid_ids``：按给的来，要求互不相交；
    2. 只给 ``calib_ids``：``valid`` 取其余样本；
    3. 只给 ``valid_ids``：``calib`` 取其余样本；
    4. 都不给：把 ``pose_id`` 升序排列后，用 ``random.Random(seed)``（**不是**
       ``np.random``，可复现且不污染全局随机状态）洗牌，前
       ``round(n * split_ratio)``（夹在 ``[3, n-1]``）个当标定、其余当验证。

    任何分支都要求标定至少 :data:`MIN_CALIB_SAMPLES` 个、验证至少
    :data:`MIN_VALID_SAMPLES` 个；两个返回值都按 ``pose_id`` 升序，便于报告
    逐字节复现。

    Args:
        samples: 样本序列（``Sample`` / ``{"pose_id": ...}`` / 整数均可）。
        calib_ids: 参与求解的 ``pose_id`` 序列；``None`` 表示不指定。
        valid_ids: 用于验证的 ``pose_id`` 序列；``None`` 表示不指定。
        split_ratio: 未指定 id 时标定子集占比，必须落在 ``(0, 1)``。
        seed: 洗牌随机种子（整数）。

    Returns:
        ``(calib_ids, valid_ids)``：两个按 ``pose_id`` 升序的 ``list[int]``。

    Raises:
        ValueError: 没有样本 / ``pose_id`` 重复 / id 不在会话里 / 两个列表
            相交 / 样本不足 / 比例或种子非法。
    """
    ids = [_pose_id_of(sample) for sample in samples]
    if not ids:
        raise ValueError("samples 为空，无法划分标定/验证子集")
    if len(set(ids)) != len(ids):
        raise ValueError("samples 里存在重复的 pose_id，无法划分")
    known = set(ids)

    selected_calib = _normalize_ids(calib_ids, "calib_ids", known)
    selected_valid = _normalize_ids(valid_ids, "valid_ids", known)

    if selected_calib is not None and selected_valid is not None:
        conflict = sorted(set(selected_calib) & set(selected_valid))
        if conflict:
            raise ValueError(
                f"calib_ids 与 valid_ids 不能相交，重复的 pose_id：{conflict}")
        chosen_calib, chosen_valid = selected_calib, selected_valid
    elif selected_calib is not None:
        chosen_calib = list(selected_calib)
        chosen_valid = sorted(known - set(selected_calib))
    elif selected_valid is not None:
        chosen_valid = list(selected_valid)
        chosen_calib = sorted(known - set(selected_valid))
    else:
        ratio = float(split_ratio)
        if not math.isfinite(ratio) or not 0.0 < ratio < 1.0:
            raise ValueError(f"split_ratio 需落在 (0, 1)，当前 {split_ratio!r}")
        if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)):
            raise ValueError(f"seed 必须为整数，当前 {seed!r}")
        ordered = sorted(known)
        n_total = len(ordered)
        n_calib = int(round(n_total * ratio))
        n_calib = min(max(MIN_CALIB_SAMPLES, n_calib),
                      n_total - MIN_VALID_SAMPLES)
        shuffled = list(ordered)
        random.Random(int(seed)).shuffle(shuffled)
        chosen_calib = sorted(shuffled[:n_calib])
        chosen_valid = sorted(shuffled[n_calib:])

    if len(chosen_calib) < MIN_CALIB_SAMPLES:
        raise ValueError(
            f"标定子集至少需要 {MIN_CALIB_SAMPLES} 个 pose_id，"
            f"当前 {len(chosen_calib)} 个（{chosen_calib}）")
    if len(chosen_valid) < MIN_VALID_SAMPLES:
        raise ValueError(
            f"验证子集至少需要 {MIN_VALID_SAMPLES} 个 pose_id，"
            f"当前 {len(chosen_valid)} 个；请增加样本或调整 calib_ids/valid_ids")
    return chosen_calib, chosen_valid


# ---------------------------------------------------------------------------
# 运动学解析与会话视图
# ---------------------------------------------------------------------------


def _session_urdf_path(session, role: str) -> Path:
    """从 ``session.meta["robot_urdf"]`` 解析 URDF 路径（相对路径先试仓库根）。"""
    meta = getattr(session, "meta", None)
    raw = meta.get("robot_urdf") if isinstance(meta, dict) else None
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError(
            f"session.meta.robot_urdf 缺失，无法用 {role} 的 DhOverride 构造运动学")
    candidate = Path(raw).expanduser()
    if candidate.is_absolute():
        return candidate
    for base in (REPO_ROOT, Path.cwd()):
        option = base / candidate
        if option.is_file():
            return option
    return candidate


def _resolve_kinematics(session, spec, *, role: str) -> RobotKinematics:
    """把 ``DhOverride`` / ``RobotKinematics`` 统一成 :class:`RobotKinematics`。

    Args:
        session: :class:`handeye.session.Session`（提供 ``meta.robot_urdf``）。
        spec: ``DhOverride`` 或已构造好的 ``RobotKinematics``。
        role: 报错用的角色名（``"dh_old"`` / ``"dh_new"``）。

    Returns:
        :class:`RobotKinematics`：``robot_urdf`` 缺失时抛 ``ValueError``。

    Raises:
        ValueError: ``spec`` 类型不支持 / URDF 路径缺失 / URDF 无法装载。
    """
    if isinstance(spec, RobotKinematics):
        return spec
    if isinstance(spec, DhOverride):
        urdf = _session_urdf_path(session, role)
        try:
            return RobotKinematics(urdf, spec, label=spec.name)
        except (OSError, ValueError, KeyError) as error:
            raise ValueError(
                f"无法用 {role}={spec.name!r} 构造运动学（URDF={urdf}）：{error}"
            ) from error
    raise ValueError(
        f"{role} 需为 DhOverride 或 RobotKinematics，当前 {type(spec).__name__}")


def _subset_session(session, ids, kinematics):
    """构造「只含 ``ids`` 的样本 + 指定运动学」的 **浅拷贝视图**。

    实现方式是 ``copy.copy(session)`` 后替换 ``.samples`` 与 ``.kinematics``
    两个普通属性，其它属性（``meta`` / ``reference_point`` /
    ``delta_p_base`` / ``handeye_matrix`` / ``source_path``）保持原对象的引用，
    **不重新读 JSON、不改动调用方 Session**。逐样本对象也是原对象（``Sample``
    本身不会被复制或修改）。

    .. warning::
       ``Session`` 的私有索引 ``_by_id`` 仍指向 **原** ``samples`` 列表的下标，
       因此视图上的 :meth:`handeye.session.Session.sample_by_id` 不可靠；
       :func:`handeye.vpe.compute_vpe` 只遍历 ``.samples``，不受影响。

    Args:
        session: 原 :class:`handeye.session.Session`。
        ids: 要保留的 ``pose_id`` 序列（顺序即视图的样本顺序）。
        kinematics: 视图使用的运动学对象（实现 ``fk(q_deg)``）。

    Returns:
        浅拷贝的 :class:`handeye.session.Session` 视图。

    Raises:
        KeyError: ``ids`` 里出现会话中不存在的 ``pose_id``。
    """
    by_id = {sample.pose_id: sample for sample in session.samples}
    view = copy.copy(session)
    view.samples = [by_id[pose_id] for pose_id in ids]
    view.kinematics = kinematics
    return view


def _frame_count(samples) -> int:
    """样本序列的总帧数。"""
    return sum(len(sample.frames) for sample in samples)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def run_offline_recompute(*, pb_dir, session, dh_old, dh_new, backend,
                          calib_ids=None, valid_ids=None, split_ratio=0.7,
                          seed=0, work_dir=None, pose_source_old="recorded",
                          proto_root=None, dry_run=False) -> dict:
    """完整离线重算：改写标定子集 -> 两次求解 -> 验证子集上比 4 种组合的 VPE。

    步骤：

    1. 用 ``session.samples`` 的 ``q_deg`` 与 ``dh_old`` / ``dh_new`` 分别算
       ``T_bh``；
    2. 把 ``pb_dir`` 里的 ``.pb`` 按 **文件名排序** 与 ``session.samples`` 按
       **列表顺序** 一一对应（数量不等 -> ``ValueError``）；
    3. :func:`split_ids` 划分标定 / 验证子集；
    4. 把标定子集分别改写到 ``<work_dir>/old_calib`` 与 ``<work_dir>/new_calib``：
       ``pose_source_old="recorded"`` 时旧一侧 **原样复制**、保留控制器回传的
       ``robot_pt``，``"fk_old"`` 时用 FK(θ_old) 改写；
    5. 用 ``backend`` 分别求解 -> ``X_old`` / ``X_new``；
    6. 在验证子集上算 4 种组合的 VPE：``(θ_old, X_old)`` 基线、
       ``(θ_new, X_new)`` 主结果、以及两个交叉组合（仅诊断）；
    7. 汇总返回。

    ``work_dir`` 下只清理本函数自己的 ``old_calib`` / ``new_calib`` 两个子目录
    （避免上一次运行残留的 ``.pb`` 混进求解输入）；``work_dir`` 本身不删。

    Args:
        pb_dir: 原始 ``.pb`` 目录（每条记录含 ``board_statics`` 与 ``robot_pt``）。
        session: :class:`handeye.session.Session`（提供 ``q_deg`` / 靶点 /
            参考点；其 ``handeye_matrix`` 不参与本流程）。
        dh_old: 旧 DH，``DhOverride`` 或 ``RobotKinematics``。
        dh_new: 新 DH，同上。
        backend: :class:`SolverBackend` 实例（``dry_run=True`` 时可为 ``None``）。
        calib_ids: 显式标定 ``pose_id``（``None`` = 按比例划分）。
        valid_ids: 显式验证 ``pose_id``（``None`` = 其余样本）。
        split_ratio: 未指定 id 时的标定占比。
        seed: 划分随机种子。
        work_dir: 中间产物目录；``None`` = 临时目录（结束后自动删除）。
        pose_source_old: ``"recorded"`` 或 ``"fk_old"``。
        proto_root: 传给 :func:`handeye.pb_rewrite.rewrite_directory` 的 proto
            根目录（``None`` = 环境变量/默认路径）。
        dry_run: ``True`` 时只做规划与 ``.pb`` 改写，不调用求解器。

    Returns:
        dict（键固定，另含 ``config`` / ``work_dir`` / ``work_dir_temporary``）：

        * ``calib_ids`` / ``valid_ids``：两个 ``list[int]``；
        * ``solver``：``{"old": info, "new": info}``；
        * ``matrices``：``{"x_old": ndarray(4, 4), "x_new": ndarray(4, 4)}``
          （``dry_run`` 时两者为 ``None``）；
        * ``matrix_delta``：``{"position_mm", "rotation_deg"}``（``dry_run``
          时为 ``None``）；
        * ``combinations``：4 项，每项含 ``label`` / ``dh`` / ``x`` /
          ``vpe_rmse_mm`` / ``vpe_max_mm`` / ``vpe_p95_mm`` / ``bias_norm_mm`` /
          ``spread_mean_mm`` / ``n_samples`` / ``n_frames`` / ``is_baseline`` /
          ``is_primary``（``dry_run`` 时为空列表）；
        * ``delta_vpe_rmse_mm``：``VPE_old - VPE_new``（正值 = 改善）；
        * ``fk_consistency``：:func:`handeye.session.check_fk_consistency`
          对 **原会话** 的结果；
        * ``rewrite``：``{"old": {...}, "new": {...}}``，
          :func:`handeye.pb_rewrite.rewrite_directory` 的返回值（旧一侧另加
          ``pose_source`` / ``copied_unchanged``）；
        * ``config``：报告所需的实验配置（pb 目录、会话、DH 标签、划分方式、
          样本/帧数等）。

    Raises:
        ValueError: 目录不存在 / 文件数与样本数不匹配 / 划分非法 /
            ``pose_source_old`` 非法 / 缺少后端 / DH 类型不支持。
        handeye.pb_rewrite.ProtoUnavailableError: proto 绑定不可用。
        SolverError: 求解失败。
        handeye.vpe.VpeError: VPE 无法计算。
    """
    pose_source = str(pose_source_old)
    if pose_source not in POSE_SOURCES_OLD:
        raise ValueError(
            f"pose_source_old 只支持 {POSE_SOURCES_OLD}，当前 {pose_source!r}")
    if not dry_run and (
            backend is None or not callable(getattr(backend, "solve", None))):
        raise ValueError(
            "缺少求解器后端：backend 需实现 solve(input_dir) -> (matrix, info)；"
            "只做规划与改写请传 dry_run=True")

    source = Path(pb_dir).expanduser()
    if not source.is_dir():
        raise ValueError(f"pb 目录不存在：{source}")
    samples = list(session.samples)
    files = sorted(
        (path for path in source.glob("*.pb") if path.is_file()),
        key=lambda path: path.name)
    if len(files) != len(samples):
        raise ValueError(
            f".pb 文件数 {len(files)} 与 session.samples 数 {len(samples)} "
            f"不匹配（要求按文件名排序与 samples 列表顺序一一对应；"
            f"pb 目录：{source}）")

    kinematics_old = _resolve_kinematics(session, dh_old, role="dh_old")
    kinematics_new = _resolve_kinematics(session, dh_new, role="dh_new")
    chosen_calib, chosen_valid = split_ids(
        samples, calib_ids=calib_ids, valid_ids=valid_ids,
        split_ratio=split_ratio, seed=seed)

    index_by_id = {sample.pose_id: index for index, sample in enumerate(samples)}
    calib_samples = [samples[index_by_id[pose_id]] for pose_id in chosen_calib]
    valid_samples = [samples[index_by_id[pose_id]] for pose_id in chosen_valid]
    calib_names = [files[index_by_id[pose_id]].name for pose_id in chosen_calib]

    temporary = work_dir is None
    work_root = (Path(tempfile.mkdtemp(prefix="handeye_recompute_"))
                 if temporary else Path(work_dir).expanduser())
    work_root.mkdir(parents=True, exist_ok=True)
    old_dir = work_root / "old_calib"
    new_dir = work_root / "new_calib"
    # 清掉上一次运行的残留，保证求解器只看到本次标定子集
    for stale in (old_dir, new_dir):
        shutil.rmtree(stale, ignore_errors=True)

    try:
        old_matrices = {}
        new_matrices = {}
        for pose_id in chosen_calib:
            index = index_by_id[pose_id]
            name = files[index].name
            sample = samples[index]
            old_matrices[name] = (
                kinematics_old.fk(sample.q_deg) if pose_source == "fk_old" else None)
            new_matrices[name] = kinematics_new.fk(sample.q_deg)

        rewrite_old = dict(rewrite_directory(
            source, old_dir, old_matrices, proto_root=proto_root))
        if pose_source == "recorded":
            # 旧一侧保留控制器回传位姿：逐字节原样复制标定子集
            old_dir.mkdir(parents=True, exist_ok=True)
            for name in calib_names:
                shutil.copyfile(source / name, old_dir / name)
            calib_name_set = set(calib_names)
            skipped = [entry for entry in rewrite_old["skipped_files"]
                       if entry["file"] not in calib_name_set]
            rewrite_old["skipped_files"] = skipped
            rewrite_old["skipped"] = len(skipped)
            rewrite_old["copied_unchanged"] = len(calib_names)
        else:
            rewrite_old["copied_unchanged"] = 0
        rewrite_old["pose_source"] = pose_source

        rewrite_new = dict(rewrite_directory(
            source, new_dir, new_matrices, proto_root=proto_root))
        rewrite_new["pose_source"] = "fk_new"
        rewrite_new["copied_unchanged"] = 0

        matrices = {"x_old": None, "x_new": None}
        solver_info = {"old": None, "new": None}
        combinations: list = []
        matrix_delta = {"position_mm": None, "rotation_deg": None}
        delta_vpe = None

        if dry_run:
            solver_info = {
                "old": {"backend": getattr(backend, "name", "dry-run"),
                        "skipped": True, "reason": "dry_run"},
                "new": {"backend": getattr(backend, "name", "dry-run"),
                        "skipped": True, "reason": "dry_run"},
            }
        else:
            x_old, info_old = backend.solve(old_dir)
            x_new, info_new = backend.solve(new_dir)
            matrices = {"x_old": x_old, "x_new": x_new}
            solver_info = {"old": info_old, "new": info_new}
            matrix_delta = compare_matrices(x_old, x_new)

            views = {
                "old": _subset_session(session, chosen_valid, kinematics_old),
                "new": _subset_session(session, chosen_valid, kinematics_new),
            }
            for label, dh_key, x_key, is_baseline, is_primary in COMBINATION_SPECS:
                matrix = matrices[f"x_{x_key}"]
                vpe_result = compute_vpe(views[dh_key], matrix=matrix)
                metrics = vpe_result["metrics"]
                combinations.append({
                    "label": label,
                    "dh": dh_key,
                    "x": x_key,
                    "vpe_rmse_mm": float(metrics["vpe_rmse_mm"]),
                    "vpe_max_mm": float(metrics["vpe_max_mm"]),
                    "vpe_p95_mm": float(metrics["vpe_p95_mm"]),
                    "bias_norm_mm": float(metrics["bias_norm_mm"]),
                    "spread_mean_mm": float(vpe_result["spread_mean_mm"]),
                    "n_samples": int(vpe_result["n_samples"]),
                    "n_frames": int(vpe_result["n_frames"]),
                    "is_baseline": bool(is_baseline),
                    "is_primary": bool(is_primary),
                })
            baseline = next(entry for entry in combinations
                            if entry["is_baseline"])
            primary = next(entry for entry in combinations
                           if entry["is_primary"])
            delta_vpe = float(
                baseline["vpe_rmse_mm"] - primary["vpe_rmse_mm"])

        split_mode = ("explicit"
                      if calib_ids is not None or valid_ids is not None
                      else "ratio")
        config = {
            "pb_dir": str(source),
            "session": (None if session.source_path is None
                        else str(session.source_path)),
            "dh_old": kinematics_old.label,
            "dh_new": kinematics_new.label,
            "pose_source_old": pose_source,
            "split": {
                "mode": split_mode,
                "ratio": float(split_ratio),
                "seed": int(seed),
            },
            "calib_ids": list(chosen_calib),
            "valid_ids": list(chosen_valid),
            "n_calib_samples": len(calib_samples),
            "n_valid_samples": len(valid_samples),
            "n_calib_frames": _frame_count(calib_samples),
            "n_valid_frames": _frame_count(valid_samples),
            "pb_files": [path.name for path in files],
        }

        return {
            "calib_ids": list(chosen_calib),
            "valid_ids": list(chosen_valid),
            "solver": solver_info,
            "matrices": matrices,
            "matrix_delta": matrix_delta,
            "combinations": combinations,
            "delta_vpe_rmse_mm": delta_vpe,
            "fk_consistency": check_fk_consistency(session),
            "rewrite": {"old": rewrite_old, "new": rewrite_new},
            "config": config,
            "work_dir": str(work_root),
            "work_dir_temporary": bool(temporary),
        }
    finally:
        if temporary:
            shutil.rmtree(work_root, ignore_errors=True)


# ---------------------------------------------------------------------------
# markdown 报告
# ---------------------------------------------------------------------------


def _format_number(value, suffix: str = "") -> str:
    """把数值格式化为 4 位小数；``None`` / 非有限值输出 ``—``。"""
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


def _format_ratio(ratio) -> str:
    """把比例格式化为百分比（4 位小数）；非有限值输出 ``—``。"""
    if ratio is None:
        return "—"
    try:
        number = float(ratio)
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(number):
        return "—"
    return f"{_format_number(number * 100.0)}%"


def _display_path(value) -> str:
    """路径展示：仓库内绝对路径转相对仓库根，仓库外只显示文件名，``None`` 给 ``—``。"""
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


def _ids_text(ids) -> str:
    """``pose_id`` 列表 -> ``"1, 2, 3"``（空列表给 ``—``）。"""
    values = list(ids or [])
    return ", ".join(str(int(value)) for value in values) if values else "—"


def _table(headers, rows) -> list:
    """生成标准 markdown 管道表格。"""
    lines = [
        "| " + " | ".join(str(header) for header in headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend(
        "| " + " | ".join(str(cell) for cell in row) + " |" for row in rows)
    return lines


def _matrix16_text(matrix) -> str:
    """4×4 矩阵 -> ``matrix16_rowmajor``（mm）的逗号分隔单行文本。"""
    values = to_matrix16(np.asarray(matrix, dtype=float), units="mm")
    return ", ".join(_format_number(value) for value in values)


def _section_config(config) -> list:
    """``## 1. 实验配置``。"""
    split = config.get("split") or {}
    if split.get("mode") == "ratio":
        split_text = (
            f"按比例 split_ratio={_format_number(split.get('ratio'))}，"
            f"seed={split.get('seed')}（random.Random，可复现）")
    else:
        split_text = (
            f"显式指定 --calib-ids / --valid-ids（split_ratio="
            f"{_format_number(split.get('ratio'))}、seed={split.get('seed')} 未参与）")
    source_text = (
        "recorded（旧一侧保留控制器回传 robot_pt，不改写）"
        if config.get("pose_source_old") == "recorded"
        else "fk_old（旧一侧用 FK(θ_old) 改写 robot_pt）")
    rows = [
        ("pb 目录", _display_path(config.get("pb_dir"))),
        ("会话", _display_path(config.get("session"))),
        ("旧 DH 标签", str(config.get("dh_old", "—"))),
        ("新 DH 标签", str(config.get("dh_new", "—"))),
        ("旧侧位姿来源", source_text),
        ("划分方式", split_text),
        ("标定 pose_id", _ids_text(config.get("calib_ids"))),
        ("验证 pose_id", _ids_text(config.get("valid_ids"))),
        ("标定样本数 / 帧数",
         f"{int(config.get('n_calib_samples', 0))} / "
         f"{int(config.get('n_calib_frames', 0))}"),
        ("验证样本数 / 帧数",
         f"{int(config.get('n_valid_samples', 0))} / "
         f"{int(config.get('n_valid_frames', 0))}"),
    ]
    return ["## 1. 实验配置", ""] + _table(("项目", "值"), rows)


def _section_solve(result) -> list:
    """``## 2. 求解结果``。"""
    lines = ["## 2. 求解结果", ""]
    matrices = result.get("matrices") or {}
    x_old = matrices.get("x_old")
    x_new = matrices.get("x_new")
    if x_old is None or x_new is None:
        lines.append(
            "未执行求解（`--dry-run` 只做规划与 `.pb` 改写），本节无结果。")
        return lines

    solver = result.get("solver") or {}
    rows = []
    for key, title in (("old", "X_old（旧 DH）"), ("new", "X_new（新 DH）")):
        info = solver.get(key) or {}
        raw = info.get("raw") or {}
        rows.append((
            title,
            str(info.get("backend", "—")),
            "true" if raw.get("success") else "false",
            _format_number(raw.get("compute_error")),
        ))
    lines += _table(("结果", "后端", "success", "compute_error"), rows)
    lines += [
        "",
        "**X_old `matrix16_rowmajor`（mm，row-major）**：",
        "",
        "```text",
        _matrix16_text(x_old),
        "```",
        "",
        "**X_new `matrix16_rowmajor`（mm，row-major）**：",
        "",
        "```text",
        _matrix16_text(x_new),
        "```",
    ]
    delta = result.get("matrix_delta") or {}
    lines += ["", "**两套 X 的差异**：", ""]
    lines += _table(("指标", "数值"), [
        ("平移差 (mm)", _format_number(delta.get("position_mm"), " mm")),
        ("旋转差 (deg)", _format_number(delta.get("rotation_deg"), "°")),
    ])
    return lines


def _section_vpe(result) -> list:
    """``## 3. VPE 对比``（核心表：4 种组合）。"""
    lines = ["## 3. VPE 对比", ""]
    combinations = result.get("combinations") or []
    if not combinations:
        lines.append("未执行求解（`--dry-run`），没有可比较的 VPE。")
        return lines

    rows = []
    for entry in combinations:
        cells = [
            str(entry.get("label", "—")),
            _format_number(entry.get("vpe_rmse_mm")),
            _format_number(entry.get("vpe_max_mm")),
            _format_number(entry.get("vpe_p95_mm")),
            _format_number(entry.get("bias_norm_mm")),
            _format_number(entry.get("spread_mean_mm")),
            str(int(entry.get("n_samples", 0))),
        ]
        if entry.get("is_baseline") or entry.get("is_primary"):
            cells = [f"**{cell}**" for cell in cells]
        rows.append(cells)
    headers = ("组合", "VPE_RMSE (mm)", "VPE_MAX (mm)", "VPE_P95 (mm)",
               "‖bias‖ (mm)", "spread (mm)", "样本数")
    lines += _table(headers, rows)
    lines += [
        "",
        "说明：全部组合只在 **验证子集**（未参与求解）上统计；粗体行为 "
        "`is_baseline`（θ_old × X_old，生产基线）与 `is_primary`"
        "（θ_new × X_new，新 DH 主结果）。",
    ]
    return lines


def _vpe_ratio(delta, rmse_old):
    """``ΔVPE_RMSE / VPE_RMSE_old``；基线 RMSE≈0 时按符号返回 ``±inf``。"""
    if delta is None or rmse_old is None:
        return None
    delta = float(delta)
    rmse_old = float(rmse_old)
    if not math.isfinite(delta) or not math.isfinite(rmse_old):
        return None
    if rmse_old <= 1e-12:
        if delta > 0.0:
            return math.inf
        if delta < 0.0:
            return -math.inf
        return 0.0
    return delta / rmse_old


def _verdict_text(ratio) -> str:
    """按 ``Δ/RMSE_old`` 给出中文判读（阈值 ±10%）。"""
    if ratio is None:
        return "无法给出结论：缺少 VPE 对比数据。"
    if ratio >= 0.10:
        return "新 DH 使手眼定位精度改善超过 10%，支持采用标定后参数"
    if -0.10 < ratio < 0.10:
        return ("新旧 DH 对手眼定位精度影响在 ±10% 内，不足以支持结论，"
                "建议增加样本或复核标定残差")
    return "新 DH 使手眼定位精度变差，需检查 DH 标定结果与零点定义转换"


def _section_verdict(result) -> list:
    """``## 4. 结论``。"""
    lines = ["## 4. 结论", ""]
    combinations = result.get("combinations") or []
    baseline = next(
        (entry for entry in combinations if entry.get("is_baseline")), None)
    primary = next(
        (entry for entry in combinations if entry.get("is_primary")), None)
    if baseline is None or primary is None:
        lines.append("未执行求解（`--dry-run`），无法给出结论。")
        return lines

    delta = result.get("delta_vpe_rmse_mm")
    rmse_old = baseline.get("vpe_rmse_mm")
    rmse_new = primary.get("vpe_rmse_mm")
    ratio = _vpe_ratio(delta, rmse_old)
    rows = [
        ("VPE_RMSE 基线（θ_old × X_old）", _format_number(rmse_old, " mm")),
        ("VPE_RMSE 主结果（θ_new × X_new）", _format_number(rmse_new, " mm")),
        ("ΔVPE_RMSE = VPE_old − VPE_new（正值 = 改善）",
         _format_number(delta, " mm")),
        ("Δ / VPE_RMSE_old", _format_ratio(ratio)),
    ]
    lines += _table(("项目", "数值"), rows)
    lines += ["", f"**判读：{_verdict_text(ratio)}**"]
    if ratio is not None and not math.isfinite(ratio):
        lines += [
            "",
            "注：基线 `VPE_RMSE` 为 0（或低于 1e-12 mm），`Δ/RMSE_old` 无定义，"
            "此处按 Δ 的符号给出判读。",
        ]
    return lines


def _section_cross(result) -> list:
    """``## 5. 交叉组合诊断``。"""
    combinations = {
        entry.get("label"): entry for entry in (result.get("combinations") or [])}
    baseline = next(
        (entry for entry in (result.get("combinations") or [])
         if entry.get("is_baseline")), None)
    lines = [
        "## 5. 交叉组合诊断",
        "",
        "两个交叉组合用来定位误差来源（VPE 只在验证子集上统计）：",
        "",
        "- `θ_new × X_old`：保留旧手眼矩阵、只换运动学模型。VPE 的变化只来自 "
        "DH 改动，用于判断「新 DH 本身」对视觉链的影响；",
        "- `θ_old × X_new`：保留旧运动学、只用新求解出的 X。VPE 的变化只来自"
        "手眼矩阵，用于判断新 X 是否吸收了旧 DH 的模型误差；",
    ]
    cross_new = combinations.get("θ_new × X_old")
    cross_old = combinations.get("θ_old × X_new")
    if baseline is not None and cross_new is not None and cross_old is not None:
        lines += [
            f"- 本次 `θ_new × X_old` 的 VPE_RMSE 相对基线变化 "
            f"{_format_number(cross_new['vpe_rmse_mm'] - baseline['vpe_rmse_mm'], ' mm')}；",
            f"- 本次 `θ_old × X_new` 的 VPE_RMSE 相对基线变化 "
            f"{_format_number(cross_old['vpe_rmse_mm'] - baseline['vpe_rmse_mm'], ' mm')}。",
        ]
    lines += [
        "",
        "交叉组合**不具备物理一致性**，仅作诊断，不作最终性能结论。",
    ]
    return lines


def _section_fk(fk_consistency) -> list:
    """``## 6. 运动学一致性自检``。"""
    lines = ["## 6. 运动学一致性自检", ""]
    if fk_consistency is None:
        lines.append("未提供 `fk_consistency`，本节跳过。")
        return lines
    worst = fk_consistency.get("worst")
    rows = [
        ("比较样本数", str(int(fk_consistency.get("n_compared", 0)))),
        ("缺失数", str(int(fk_consistency.get("n_missing", 0)))),
        ("位置最大偏差 (mm)",
         _format_number(fk_consistency.get("position_max_mm"))),
        ("位置 RMS 偏差 (mm)",
         _format_number(fk_consistency.get("position_rms_mm"))),
        ("旋转最大偏差 (deg)",
         _format_number(fk_consistency.get("rotation_max_deg"), "°")),
        ("旋转 RMS 偏差 (deg)",
         _format_number(fk_consistency.get("rotation_rms_deg"), "°")),
        ("最差 pose_id", "—" if worst is None else str(worst.get("pose_id"))),
    ]
    lines += _table(("项目", "值"), rows)
    lines += [
        "",
        "自检口径：会话自带的运动学（装载 `SESSION.json` 时的 DH）对每个样本"
        "重算 `T_bh`，与 `recorded_pose` 比对。",
    ]
    return lines


def _section_reproduce(config) -> list:
    """``## 7. 复现方式``。"""
    pb_dir = _display_path(config.get("pb_dir"))
    session = _display_path(config.get("session"))
    if session == "—":
        session = "<SESSION.json>"
    calib = ",".join(str(int(value)) for value in (config.get("calib_ids") or []))
    valid = ",".join(str(int(value)) for value in (config.get("valid_ids") or []))
    command = [
        "PYTHONPATH=python python3 -m handeye.offline_recompute \\",
        f"    --pb-dir {pb_dir} --session {session} \\",
        "    --dh-old '<DH_OLD>.json' --dh-new '<DH_NEW>.json' \\",
    ]
    if calib:
        command.append(f"    --calib-ids {calib} --valid-ids {valid} \\")
    command.append(
        f"    --pose-source-old {config.get('pose_source_old', 'recorded')} \\")
    command.append("    -o offline_recompute_report.md")
    return [
        "## 7. 复现方式",
        "",
        "在仓库根执行（报告只写相对路径/文件名，可直接外发）：",
        "",
        "```bash",
        *command,
        "```",
        "",
        "- 路径展示规则：仓库内路径为相对仓库根的路径，仓库外只显示文件名；",
        "- `--dh-old` / `--dh-new` 换成实际的 DH 覆盖 JSON（标签见第 1 节）；",
        "- 求解器默认用内置模板（需要容器内 `ros2`，本仓库不自动执行 "
        "docker），请按环境用 `--solver-cmd` 替换，模板占位符为 "
        "`{input_dir}` / `{output_bin}` / `{output_json}`；",
        "- 划分固定为显式 `--calib-ids` / `--valid-ids`，与本次实验完全一致"
        "（若原实验用 `--split` + `--seed`，同一份 `pose_id` 排序可复现）。",
    ]


def render_report(result, *, title="手眼离线重算报告（旧 DH vs 新 DH）") -> str:
    """渲染离线重算的 markdown 报告（调用方负责写盘）。

    小节标题与顺序固定（便于逐字节 diff）：``## 1. 实验配置``、
    ``## 2. 求解结果``、``## 3. VPE 对比``、``## 4. 结论``、
    ``## 5. 交叉组合诊断``、``## 6. 运动学一致性自检``、``## 7. 复现方式``。

    复现性约定与 :mod:`handeye.report` 一致：只依赖入参、无时间戳、无绝对
    路径、所有浮点 4 位小数、非有限值输出 ``—``。

    Args:
        result: :func:`run_offline_recompute` 的返回值。
        title: 一级标题文字。

    Returns:
        UTF-8 兼容的 markdown 文本，以换行结尾。

    Raises:
        ValueError: ``result`` 不是字典。
    """
    if not isinstance(result, dict):
        raise ValueError(f"result 需为 dict，当前 {type(result).__name__}")
    config = result.get("config") or {}
    lines = [f"# {title}", ""]
    for section in (
            _section_config(config),
            _section_solve(result),
            _section_vpe(result),
            _section_verdict(result),
            _section_cross(result),
            _section_fk(result.get("fk_consistency")),
            _section_reproduce(config),
    ):
        lines += section
        lines += [""]
    return "\n".join(lines).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


class _UsageError(Exception):
    """argparse 的参数错误，``main`` 捕获后转成退出码 2。"""


class _ArgumentParser(argparse.ArgumentParser):
    """不直接 ``sys.exit`` 的 ArgumentParser，便于测试直接调用 ``main``。"""

    def error(self, message):
        raise _UsageError(message)


def build_parser() -> argparse.ArgumentParser:
    """构造 ``python -m handeye.offline_recompute`` 的参数解析器。"""
    parser = _ArgumentParser(
        prog="python -m handeye.offline_recompute",
        description="旧/新 DH 离线重算：同一批采样换运动学模型，各解一次手眼，"
                    "在未参与求解的验证样本上比较 VPE。")
    parser.add_argument("--pb-dir", required=True,
                        help="原始 .pb 目录（每条记录含 board_statics 与 robot_pt）")
    parser.add_argument("--session", required=True, help="会话 JSON 路径")
    parser.add_argument("--dh-old", required=True, help="旧 DH 覆盖 JSON")
    parser.add_argument("--dh-new", required=True, help="新 DH 覆盖 JSON")
    parser.add_argument("-o", "--out", default="offline_recompute_report.md",
                        help="markdown 报告路径（默认 offline_recompute_report.md）")
    parser.add_argument("--json", dest="json_path", default=None,
                        help="机器可读 JSON 输出路径（可选）")
    parser.add_argument("--calib-ids", default=None,
                        help="参与求解的 pose_id，逗号分隔（如 1,2,3）")
    parser.add_argument("--valid-ids", default=None,
                        help="用于验证的 pose_id，逗号分隔（如 4,5,6）")
    parser.add_argument("--split", dest="split_ratio", type=float, default=0.7,
                        help="未指定 id 时标定子集占比（默认 0.7）")
    parser.add_argument("--seed", type=int, default=0,
                        help="划分随机种子（默认 0）")
    parser.add_argument("--pose-source-old", choices=list(POSE_SOURCES_OLD),
                        default="recorded",
                        help="旧 DH 一侧的位姿来源（默认 recorded：保留控制器位姿）")
    parser.add_argument("--solver-cmd", default=DEFAULT_SOLVER_CMD,
                        help="外部求解命令模板（占位符 {input_dir} {output_bin} "
                             "{output_json}）")
    parser.add_argument("--solver-timeout", type=float, default=600.0,
                        help="求解命令超时秒数（默认 600）")
    parser.add_argument("--work-dir", default=None,
                        help="中间产物目录（默认临时目录并自动清理）")
    parser.add_argument("--proto-root", default=None,
                        help="公司 arc-common proto 根目录（默认环境变量 "
                             "ARC_COMMON_PROTO_ROOT 或内置默认路径）")
    parser.add_argument("--dry-run", action="store_true",
                        help="只做规划与改写，不调求解器")
    return parser


def _parse_id_list(text, name: str):
    """解析 ``"1,2,3"`` -> ``[1, 2, 3]``；``None`` 原样返回。"""
    if text is None:
        return None
    if not isinstance(text, str):
        raise ValueError(f"{name} 需为逗号分隔的字符串，当前 {type(text).__name__}")
    items = [chunk.strip() for chunk in text.split(",")]
    values = []
    for index, chunk in enumerate(items):
        if not chunk:
            raise ValueError(f"{name} 第 {index + 1} 项为空（示例：1,2,3）")
        try:
            values.append(int(chunk))
        except ValueError as error:
            raise ValueError(
                f"{name} 第 {index + 1} 项 {chunk!r} 不是整数") from error
    return values


def _load_dh_override(path, name: str) -> DhOverride:
    """读 DH 覆盖 JSON，失败时抛带参数名的 ``ValueError``。"""
    target = Path(path).expanduser()
    if not target.is_file():
        raise ValueError(f"{name} 指向的文件不存在：{target}")
    try:
        return DhOverride.from_json(target)
    except (OSError, ValueError, KeyError) as error:
        raise ValueError(f"{name} 无法装载 DH 覆盖（{target}）：{error}") from error


def _json_ready(value):
    """把 numpy / 嵌套结构转成可 JSON 序列化的值（非有限值写 ``None``）。"""
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


def _write_json(path, payload) -> Path:
    """写 UTF-8 JSON（父目录不存在则创建），返回路径。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2,
                  allow_nan=False)
        handle.write("\n")
    return target


def _print_summary(result, *, pb_dir, session_path) -> None:
    """打印人读摘要（只由 CLI 调用）。"""
    config = result.get("config") or {}
    print(f"pb 目录: {pb_dir}")
    print(f"会话: {session_path}")
    print(f"旧 DH: {config.get('dh_old')}  新 DH: {config.get('dh_new')}"
          f"  旧侧位姿来源: {config.get('pose_source_old')}")
    print(f"标定 pose_id: {_ids_text(result.get('calib_ids'))}"
          f"  验证 pose_id: {_ids_text(result.get('valid_ids'))}")
    combinations = result.get("combinations") or []
    baseline = next(
        (entry for entry in combinations if entry.get("is_baseline")), None)
    primary = next(
        (entry for entry in combinations if entry.get("is_primary")), None)
    if baseline is None or primary is None:
        print("dry-run：已完成规划与 .pb 改写，未调用求解器。")
        return
    delta = result.get("delta_vpe_rmse_mm")
    ratio = _vpe_ratio(delta, baseline["vpe_rmse_mm"])
    print(f"VPE_RMSE: 基线 {_format_number(baseline['vpe_rmse_mm'], ' mm')}"
          f"  主结果 {_format_number(primary['vpe_rmse_mm'], ' mm')}")
    print(f"ΔVPE_RMSE: {_format_number(delta, ' mm')}（正值 = 改善，"
          f"Δ/RMSE_old={_format_ratio(ratio)}）")
    print(f"判读: {_verdict_text(ratio)}")


def main(argv=None) -> int:
    """CLI 入口（可被测试直接调用）。

    Args:
        argv: 参数列表；``None`` 时读 ``sys.argv[1:]``。

    Returns:
        退出码：``0`` 成功；``2`` 参数/数据/求解错误。
    """
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except _UsageError as error:
        print(f"参数错误：{error}", file=sys.stderr)
        print(parser.format_usage().rstrip(), file=sys.stderr)
        return 2
    except SystemExit as exit_code:  # --help / --version
        return int(exit_code.code or 0)

    pb_dir = Path(args.pb_dir).expanduser()
    if not pb_dir.is_dir():
        print(f"错误：pb 目录不存在：{pb_dir}", file=sys.stderr)
        return 2
    session_path = Path(args.session).expanduser()
    if not session_path.is_file():
        print(f"错误：会话文件不存在：{session_path}", file=sys.stderr)
        return 2

    try:
        calib_ids = _parse_id_list(args.calib_ids, "--calib-ids")
        valid_ids = _parse_id_list(args.valid_ids, "--valid-ids")
        dh_old = _load_dh_override(args.dh_old, "--dh-old")
        dh_new = _load_dh_override(args.dh_new, "--dh-new")
    except (OSError, ValueError, KeyError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2

    try:
        session = load_session(session_path)
    except (SessionError, OSError) as error:
        print(f"错误：无法装载会话：{error}", file=sys.stderr)
        return 2

    backend = None
    if not args.dry_run:
        try:
            backend = CommandSolverBackend(
                args.solver_cmd, timeout_s=args.solver_timeout,
                work_dir=args.work_dir)
        except ValueError as error:
            print(f"错误：--solver-cmd 不合法：{error}", file=sys.stderr)
            return 2

    try:
        result = run_offline_recompute(
            pb_dir=pb_dir,
            session=session,
            dh_old=dh_old,
            dh_new=dh_new,
            backend=backend,
            calib_ids=calib_ids,
            valid_ids=valid_ids,
            split_ratio=args.split_ratio,
            seed=args.seed,
            work_dir=args.work_dir,
            pose_source_old=args.pose_source_old,
            proto_root=args.proto_root,
            dry_run=bool(args.dry_run),
        )
    except (ValueError, OSError, SessionError, SolverError, VpeError,
            ProtoUnavailableError) as error:
        print(f"错误：离线重算失败：{error}", file=sys.stderr)
        return 2

    _print_summary(result, pb_dir=pb_dir, session_path=session_path)

    try:
        text = render_report(result)
        written = write_markdown(args.out, text)
    except (OSError, ValueError) as error:
        print(f"错误：无法写出 markdown 报告：{error}", file=sys.stderr)
        return 2
    print(f"报告已写入: {written}")

    if args.json_path is not None:
        try:
            written_json = _write_json(args.json_path, _json_ready(result))
        except (OSError, TypeError, ValueError) as error:
            print(f"错误：无法写出 JSON：{error}", file=sys.stderr)
            return 2
        print(f"JSON 已写入: {written_json}")
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI 入口
    raise SystemExit(main())
