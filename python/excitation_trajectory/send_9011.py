#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 ``export_9011.py`` 产出的点位逐点发给 Nexbot 控制器，每点之间暂停。

协议事实（均已从公司仓库下位 JOB 与驱动核实）
--------------------------------------------
* **PC 是 TCP 服务端**：``NexbotRobot`` 在 ``listen_ip:port`` 上 listen，控制器侧
  ``ENET1.OPEN 1`` 作为客户端连过来。所以本工具只能**先监听、等控制器连上**，
  不能主动去连机器人；下位 JOB 里的 ``ENET1.IP`` 必须填 **本机**地址，
  ``ENET1.RPORT`` 必须等于本工具的 ``--port``。
  本现场：控制器 ``192.168.3.13``（客户端），本机/TCP 服务端 ``192.168.3.11``。
* 报文是 ``\\n`` 结尾的定长 ASCII 行；单点移动用 ``M0``（自带坐标）。
* **每条指令有握手应答**：JOB 走完 ``MOVE`` 后落到 ``LABEL [$2$]``，发
  ``done``，``TIMER T = 0.1``，再发 ``ready``，然后回到 ``$Recv$``。
  所以「运动完成」是可观测的：收到 ``done`` 即运动结束，收到 ``ready``
  才可发下一条。这比按路程估算时间可靠得多。
* 出错时下位会回 ``error,...``，必须当失败处理，不能等 ``done`` 蒙过去。
* ``Q0`` 会让 JOB 跳到 ``$END$`` 结束，**之后必须重新运行 9011**；
  因此默认不发 ``Q0``（``--quit`` 才发）。

用法（仓库根）::

    # 1) 先生成点位（M0 自带坐标，无 G 下载）
    PYTHONPATH=python python3 -m excitation_trajectory.export_9011 \\
        scan_result.json --out calib_points_9011.json --dwell 6

    # 2) 控制器侧运行 9011/N9011，本机起监听并逐点发送
    PYTHONPATH=python python3 -m excitation_trajectory.send_9011 \\
        calib_points_9011.json --host 192.168.3.11 --port 12345 --dwell 6

    # 只发第 5..12 点；发完退出 JOB
    PYTHONPATH=python python3 -m excitation_trajectory.send_9011 \\
        calib_points_9011.json --start 5 --end 12 --quit
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

DEFAULT_PORT = 12345
DEFAULT_DWELL_S = 6.0
DEFAULT_TIMEOUT_S = 120.0
#: 下位 ``done`` 与 ``ready`` 之间的 ``TIMER T = 0.1``
HANDSHAKE_GAP_S = 0.1

#: 帧结束符。Nexbot 的 ``OPENMSG`` 分隔/帧尾是**控制器侧设置**（N9018.JBR
#: 注释写「分隔/帧尾用 ``;``（按现场控制器设置）」），而 C++ 驱动只发 ``\n``；
#: 两者不一致时表现为“报文发出去了但机器人不动”。
TERMINATORS = {
    "lf": "\n",
    "crlf": "\r\n",
    "semicolon": ";",
    "lf-semicolon": "\n;",
}
DEFAULT_TERMINATOR = "lf"
#: 读应答时把这些字符都当分隔（兼容控制器任意帧方式）
_RECV_SEPARATORS = "\r\n;"


def _terminator_of(name: str) -> str:
    """结束符名 -> 实际字节串。"""
    try:
        return TERMINATORS[name]
    except KeyError as error:
        raise ProtocolError(
            f"--terminator 需为 {sorted(TERMINATORS)} 之一，当前 {name!r}") from error


def local_ipv4_addresses(include_loopback: bool = False) -> list:
    """列出本机 IPv4 地址（不依赖第三方库，不发送任何报文）。"""
    found = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       family=socket.AF_INET):
            found.add(info[4][0])
    except OSError:
        pass
    try:                                  # UDP connect 不会发包，只求路由选中的本机地址
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            found.add(probe.getsockname()[0])
        finally:
            probe.close()
    except OSError:
        pass
    if include_loopback:
        return sorted(found)
    return sorted(addr for addr in found if not addr.startswith("127."))


def _is_loopback(host: str) -> bool:
    return host == "localhost" or host.startswith("127.")


def check_bind_address(host: str) -> None:
    """预检：``--host`` 必须是本机已配置的 IPv4，否则机器人永远连不上。"""
    if host in ("", "0.0.0.0") or _is_loopback(host):
        return
    if host in local_ipv4_addresses(include_loopback=True):
        return
    available = local_ipv4_addresses()
    raise ProtocolError(
        f"--host {host} 不是本机已配置的 IPv4（本机有: "
        f"{', '.join(available) if available else '未检测到'}）。"
        "控制器会连 ENET1.IP 指定的地址；绑错地址会“监听成功但永远连不上”。"
        "若本站应使用 192.168.3.11，先用 ip addr 确认该地址配在本机网卡上。")


class ProtocolError(RuntimeError):
    """下位回 ``error,...``、超时或断线。"""


def move_data_str(spec: dict) -> str:
    """由 ``export_9011`` 的 ``meta.move_data`` 复现 movedata（14 字）。"""
    move_type = str(spec.get("type", "P"))
    speed = float(spec.get("speed", -1.0))
    acc = int(spec.get("acc", 0))
    tool = int(spec.get("tool", 0))
    ratio = int(spec.get("ratio_pct", 10))

    def fixed_float(value: float, width: int = 7) -> str:
        text = ("-" + f"{-value:0{width - 1}.2f}") if value < 0 else f"{value:0{width}.2f}"
        if len(text) != width:
            raise ProtocolError(f"速度 {value} 超出 {width} 字宽")
        return text

    return (f"{move_type}{fixed_float(speed)}"
            f"{acc:01d}{tool:01d}{ratio:04d}")


def point_to_message(point: dict, move_data: str,
                     terminator: str = DEFAULT_TERMINATOR) -> str:
    """单个点 -> ``M0`` 报文（含结束符；默认 ``\n``）。"""
    coord = int(point.get("coord", 1))
    data = [float(v) for v in point["data"]]
    slot = int(point["slot"])
    fields = [f"{coord:1d}"]
    for value in data:
        text = ("-" + f"{-value:07.2f}") if value < 0 else f"{value:08.2f}"
        if len(text) != 8:
            raise ProtocolError(f"坐标 {value} 超出 8 字宽")
        fields.append(text)
    fields.append(f"{slot:04d}")
    return f"M0{''.join(fields)}{move_data}{_terminator_of(terminator)}"


def build_plan(points_path, *, start=None, end=None, dwell=None,
               terminator: str = DEFAULT_TERMINATOR) -> dict:
    """读 ``calib_points_9011.json`` 并生成待发报文序列。"""
    payload = json.loads(Path(points_path).read_text(encoding="utf-8"))
    meta = payload.get("meta", {})
    points = payload.get("points") or []
    if not points:
        raise ProtocolError(f"{points_path} 里没有 points")
    if meta.get("walk_mode", "M0") != "M0":
        raise ProtocolError(
            f"该文件是 walk_mode={meta.get('walk_mode')!r} 生成的报文（T 模式引用 "
            "预下载的 P 号），本工具只发自带坐标的 M0，请用 --walk M0 重新导出")

    lo = 1 if start is None else int(start)
    hi = len(points) if end is None else int(end)
    if not (1 <= lo <= hi <= len(points)):
        raise ProtocolError(f"--start/--end 需满足 1 <= start <= end <= {len(points)}，"
                            f"当前 {lo}..{hi}")

    move_data = move_data_str(meta.get("move_data", {}))
    selected = points[lo - 1:hi]
    lines = [point_to_message(point, move_data, terminator)
             for point in selected]
    return {
        "lines": lines,
        "points": selected,
        "dwell_s": float(meta.get("dwell_s", DEFAULT_DWELL_S) if dwell is None
                         else dwell),
        "move_data": move_data,
        "total": len(points),
        "range": (lo, hi),
        "terminator": terminator,
    }


class RobotLink:
    """TCP 服务端：监听端口，等控制器（客户端）连上来。"""

    def __init__(self, host: str, port: int, *, timeout_s: float = DEFAULT_TIMEOUT_S,
                 accept_timeout_s: float = 300.0, on_listening=None,
                 expect_peer: str | None = None):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.accept_timeout_s = accept_timeout_s
        #: ``listen()`` 完成、``accept()`` 之前回调（测试用它启动假控制器；
        #: 传 ``port=0`` 时可用 ``link.port`` 拿到内核分配的实际端口）。
        self.on_listening = on_listening
        #: 仅接受来自该 IP 的连接（机器人地址），拒绍其它来源。
        self.expect_peer = expect_peer
        self.peer = None
        self._server = None
        self._conn = None
        self._buffer = b""

    def __enter__(self) -> "RobotLink":
        check_bind_address(self.host)
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        server.settimeout(self.accept_timeout_s)
        self._server = server
        self.port = server.getsockname()[1]
        print(f"[监听] {self.host}:{self.port} 等待控制器连接"
              f"（超时 {self.accept_timeout_s:g} s）...")
        if self.on_listening is not None:
            self.on_listening(self)
        try:
            conn, peer = server.accept()
        except socket.timeout as error:
            self._close_all()
            raise ProtocolError(
                f"{self.accept_timeout_s:g} s 内没有控制器连上来：确认下位 9011/N9011 "
                f"已运行、JOB 里 ENET1.IP 指向本机、RPORT={self.port}") from error
        conn.settimeout(self.timeout_s)
        self._conn = conn
        self.peer = peer[0]
        print(f"[连接] 控制器已接入 {peer[0]}:{peer[1]}")
        if self.expect_peer is not None and peer[0] != self.expect_peer:
            self._close_all()
            raise ProtocolError(
                f"接入方 {peer[0]} 不是预期的机器人地址 {self.expect_peer}，"
                "已拒绝（核对 ENET1.IP / 网络是否串了）")
        return self

    def __exit__(self, *_exc) -> None:
        self._close_all()

    def _close_all(self) -> None:
        for sock in (self._conn, self._server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._conn = None
        self._server = None

    def send(self, line: str) -> None:
        if self._conn is None:
            raise ProtocolError("未连接控制器")
        self._conn.sendall(line.encode("ascii"))

    def read_line(self, timeout_s: float | None = None) -> str:
        """读一条应答；``\\n`` / ``\\r`` / ``;`` 都当分隔符（兼容控制器帧设置）。"""
        if self._conn is None:
            raise ProtocolError("未连接控制器")
        self._conn.settimeout(self.timeout_s if timeout_s is None else timeout_s)
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while True:
            cuts = [self._buffer.find(ch.encode())
                    for ch in _RECV_SEPARATORS]
            cuts = [pos for pos in cuts if pos >= 0]
            if cuts:
                cut = min(cuts)
                line, self._buffer = self._buffer[:cut], self._buffer[cut + 1:]
                text = line.decode("ascii", errors="replace").strip()
                if text:
                    return text
                continue                     # 连续分隔符产生的空 token 跳过
            if deadline is not None and time.monotonic() > deadline:
                raise ProtocolError(f"{timeout_s:g} s 内未收到应答")
            try:
                chunk = self._conn.recv(4096)
            except socket.timeout as error:
                raise ProtocolError(
                    f"{self.timeout_s:g} s 内未收到应答（下一条指令前必须收到 "
                    "done/ready；若报文已发但无任何回包，检查控制器 OPENMSG 的帧"
                    "分隔/结束符是否与本工具 --terminator 一致）") from error
            if not chunk:
                raise ProtocolError("控制器断开连接")
            self._buffer += chunk

    def wait_handshake(self) -> tuple:
        """等到 ``done`` 之后再等到 ``ready``；返回 (done 行, ready 行)。

        与驱动 ``RobotDriverBase::StateCheck`` 同语义：``done`` 出现在
        ``ready`` 之前；先到的孤立 ``ready``（上一条命令残留）跳过。
        """
        seen_done = None
        while True:
            line = self.read_line()
            if not line:
                continue
            if line.startswith("error"):
                raise ProtocolError(f"下位报错: {line}")
            if line == "done" and seen_done is None:
                seen_done = line
                continue
            if line == "ready":
                if seen_done is None:
                    continue          # 上一条命令晚到的 ready，跳过
                return seen_done, line
            # 其它输出（如 P 回读）忽略


def run_plan(link, plan, *, handshake: bool = True, log=print,
             sleep=time.sleep) -> list:
    """逐点发送：发一条 -> 等 done/ready -> 暂停 dwell 秒（最后一点不停）。"""
    results = []
    lines = plan["lines"]
    dwell = plan["dwell_s"]
    for index, line in enumerate(lines, start=1):
        slot = plan["points"][index - 1]["slot"]
        started = time.monotonic()
        link.send(line)
        if handshake:
            link.wait_handshake()
        motion_s = time.monotonic() - started
        is_last = index == len(lines)
        log(f"[{index}/{len(lines)}] P{slot:04d} 到位 {motion_s:.1f}s"
            + ("（最后一点，不暂停）" if is_last else f"，暂停 {dwell:g}s 等测量"))
        if not is_last:
            sleep(dwell)
        results.append({"index": index, "slot": slot, "motion_s": motion_s,
                        "dwell_s": 0.0 if is_last else dwell})
    return results


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m excitation_trajectory.send_9011",
        description="读 calib_points_9011.json，逐点发 M0 给 Nexbot 控制器，"
                    "每点之间按 done/ready 握手确认运动完成后暂停指定秒数。")
    parser.add_argument("points", help="export_9011.py 产出的 calib_points_9011.json")
    parser.add_argument("--host", default="0.0.0.0",
                        help="监听地址（默认 0.0.0.0）。指定为具体 IPv4 时会预检"
                             "它是否属于本机；本现场 PC 为 192.168.3.11")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"监听端口（默认 {DEFAULT_PORT}，须与 JOB 的 ENET1.RPORT 一致）")
    parser.add_argument("--expect-peer", default=None, metavar="ROBOT_IP",
                        help="只接受来自该 IP 的连接（机器人地址，本现场 "
                             "192.168.3.13）；不填则不校验")
    parser.add_argument("--dwell", type=float, default=None,
                        help="每点到位后暂停秒数（默认取 JSON 的 meta.dwell_s）")
    parser.add_argument("--start", type=int, default=None, help="起始点序号（1 基）")
    parser.add_argument("--end", type=int, default=None, help="结束点序号（1 基，含）")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"单条指令应答超时秒数（默认 {DEFAULT_TIMEOUT_S:g}）")
    parser.add_argument("--no-handshake", action="store_true",
                        help="不等 done/ready（不推荐：无法确认运动是否完成）")
    parser.add_argument("--quit", action="store_true",
                        help="结束时发 Q0（注意：Q0 会让 JOB 结束，之后必须重新运行 9011）")
    parser.add_argument("--terminator", choices=sorted(TERMINATORS),
                        default=DEFAULT_TERMINATOR,
                        help="帧结束符（默认 lf）。Nexbot 的 OPENMSG 用控制器侧"
                             "分隔/帧尾设置；若注释写「分隔/帧尾用 ;」而本工具发 \\n，"
                             "报文会不被分发：此时用 semicolon 或 lf-semicolon")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印将要发送的报文，不监听端口")
    args = parser.parse_args(argv)

    try:
        plan = build_plan(args.points, start=args.start, end=args.end,
                          dwell=args.dwell, terminator=args.terminator)
    except (OSError, KeyError, ValueError, ProtocolError) as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    if not (plan["dwell_s"] >= 0):
        print(f"错误: --dwell 需 >= 0，当前 {plan['dwell_s']}", file=sys.stderr)
        return 2
    lo, hi = plan["range"]
    print(f"[计划] 共 {plan['total']} 点，本次发第 {lo}..{hi} 点"
          f"（{len(plan['lines'])} 条），movedata={plan['move_data']}，"
          f"每点暂停 {plan['dwell_s']:g}s，结束符={plan['terminator']!r}")
    print(f"[网络] 本机监听 {args.host}:{args.port}"
          + (f"，只接受 {args.expect_peer} 的连接" if args.expect_peer else "")
          + f"；本机 IPv4: {', '.join(local_ipv4_addresses()) or '未检测到'}")
    print(f"[下位] JOB 需满足 ENET1.IP=\"<本机地址>\"、ENET1.RPORT={args.port}")

    if args.dry_run:
        for line in plan["lines"]:
            print(line.rstrip("\n"))
        return 0

    try:
        check_bind_address(args.host)
    except ProtocolError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    try:
        with RobotLink(args.host, args.port, timeout_s=args.timeout,
                       expect_peer=args.expect_peer) as link:
            run_plan(link, plan, handshake=not args.no_handshake)
            if args.quit:
                link.send("Q0\n")
                print("[退出] 已发 Q0 —— JOB 已结束，下次采点需重新运行 9011")
            else:
                print("[完成] 未发 Q0，JOB 仍在等待指令（可直接再跑一次本工具）")
    except ProtocolError as error:
        print(f"错误: {error}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("\n[中止] 收到 Ctrl+C", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
