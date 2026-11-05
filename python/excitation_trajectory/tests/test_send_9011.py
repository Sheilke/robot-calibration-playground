# -*- coding: utf-8 -*-
"""``send_9011`` 的单元测试：用假控制器（TCP 客户端）仿真下位握手。

下位行为（N9011.JBR 231-234 行）：每条指令执行完后发 ``done``，等 0.1 s
再发 ``ready``，然后回到 ``$Recv$``。本测试按同样时序回包，验证发送端
「发一条 → 等 done/ready → 暂停」的节奏与报文内容。
"""

import io
import json
import socket
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from excitation_trajectory.export_9011 import main as export_main
from excitation_trajectory.send_9011 import (
    HANDSHAKE_GAP_S,
    check_bind_address,
    ProtocolError,
    RobotLink,
    build_plan,
    main,
    move_data_str,
    point_to_message,
    run_plan,
)

SCAN_STUB = {"meta": {}, "waypoints": [], "perturbed_ik": [
    {"index": i, "ok": True,
     "q": [float(i), 10.0 + i, -20.0 - i, 0.5, 40.0, -30.0 + i],
     "config": [1, 1, 1]} for i in range(4)]}


class FakeController(threading.Thread):
    """仿真下位：接受连接，每收到一行回 done/ready，记录收到的报文。"""

    def __init__(self, port, *, reply_error_at=None, silent_at=None,
                 gap_s=HANDSHAKE_GAP_S):
        super().__init__(daemon=True)
        self.port = port
        self.reply_error_at = reply_error_at
        self.silent_at = silent_at
        self.gap_s = gap_s
        self.received = []
        self.connected = threading.Event()
        self.failure = None

    def run(self):
        try:
            time.sleep(0.15)
            with socket.create_connection(("127.0.0.1", self.port), timeout=5) as sock:
                self.connected.set()
                sock.settimeout(5)
                buffer = b""
                while True:
                    chunk = sock.recv(4096)
                    if not chunk:
                        return
                    buffer += chunk
                    while b"\n" in buffer:
                        line, _, buffer = buffer.partition(b"\n")
                        text = line.decode("ascii")
                        self.received.append(text)
                        index = len(self.received)
                        if self.silent_at == index:
                            return                      # 不回包，模拟卡死
                        if self.reply_error_at == index:
                            sock.sendall(b"error,999\n")
                            continue
                        sock.sendall(b"done\n")
                        time.sleep(self.gap_s)
                        sock.sendall(b"ready\n")
        except Exception as error:                      # noqa: BLE001 - 测试线程
            self.failure = error
            self.connected.set()


def _make_points(tmpdir, *, walk_mode="M0"):
    scan = Path(tmpdir) / "scan.json"
    scan.write_text(json.dumps(SCAN_STUB), encoding="utf-8")
    out = Path(tmpdir) / "points.json"
    argv = [str(scan), "--out", str(out), "--dwell", "0.05"]
    if walk_mode == "T":
        argv += ["--walk", "T"]
    with redirect_stdout(io.StringIO()):
        assert export_main(argv) == 0
    return out


class MessageFormatTest(unittest.TestCase):
    def test_move_data_str(self):
        self.assertEqual(move_data_str({"type": "P", "speed": -1.0, "acc": 0,
                                        "tool": 0, "ratio_pct": 10}),
                         "P-001.00000010")
        self.assertEqual(move_data_str({"type": "L", "speed": 150.0, "acc": 1,
                                        "tool": 0, "ratio_pct": 100}),
                         "L0150.00100100")

    def test_point_to_message(self):
        point = {"slot": 1, "coord": 1,
                 "data": [1.0, -2.5, 0.0, 3.25, -4.0, 5.0]}
        text = point_to_message(point, "P-001.00000010")
        self.assertTrue(text.startswith("M01"))
        self.assertTrue(text.endswith("\n"))
        self.assertEqual(len(text.rstrip("\n")), 69)
        self.assertEqual(text[2:11], "100001.00")   # coord + 首轴
        self.assertEqual(text[11:19], "-0002.50")

    def test_unsupported_walk_mode_rejected(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            points = _make_points(tmp, walk_mode="T")
            with self.assertRaises(ProtocolError):
                build_plan(points)


class PlanTest(unittest.TestCase):
    def test_plan_matches_export(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            points = _make_points(tmp)
            plan = build_plan(points)
            self.assertEqual(len(plan["lines"]), 4)
            self.assertEqual(plan["dwell_s"], 0.05)
            payload = json.loads(points.read_text(encoding="utf-8"))
            for line, point in zip(plan["lines"], payload["points"]):
                self.assertEqual(line, point_to_message(
                    point, move_data_str(payload["meta"]["move_data"])))

    def test_range_selection(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            points = _make_points(tmp)
            plan = build_plan(points, start=2, end=3)
            self.assertEqual(plan["range"], (2, 3))
            self.assertEqual(len(plan["lines"]), 2)

    def test_range_validation(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            points = _make_points(tmp)
            for start, end in ((0, 2), (3, 2), (1, 99)):
                with self.assertRaises(ProtocolError):
                    build_plan(points, start=start, end=end)


class SendPlanTest(unittest.TestCase):
    def _run(self, **fake_kwargs):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            plan = build_plan(_make_points(tmp))
        holder = {}

        def start_fake(link):
            controller = FakeController(link.port, **fake_kwargs)
            controller.start()
            holder["controller"] = controller

        with RobotLink("127.0.0.1", 0, timeout_s=3, accept_timeout_s=5,
                       on_listening=start_fake) as link:
            results = run_plan(link, plan, log=lambda *_: None,
                               sleep=lambda _s: None)
        controller = holder["controller"]
        controller.join(5)
        if controller.failure:
            raise controller.failure
        return plan, results, controller

    def test_sends_all_points_and_waits_for_handshake(self):
        plan, results, controller = self._run()
        self.assertEqual(len(controller.received), len(plan["lines"]))
        self.assertEqual(controller.received, [l.rstrip("\n") for l in plan["lines"]])
        self.assertEqual([r["index"] for r in results], [1, 2, 3, 4])
        self.assertEqual([r["dwell_s"] for r in results], [0.05, 0.05, 0.05, 0.0])

    def test_error_line_aborts(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            plan = build_plan(_make_points(tmp))
        holder = {}

        def start_fake(link):
            holder["c"] = FakeController(link.port, reply_error_at=2)
            holder["c"].start()

        with RobotLink("127.0.0.1", 0, timeout_s=3, accept_timeout_s=5,
                       on_listening=start_fake) as link:
            with self.assertRaises(ProtocolError) as ctx:
                run_plan(link, plan, log=lambda *_: None, sleep=lambda _s: None)
        self.assertIn("error", str(ctx.exception))
        holder["c"].join(5)

    def test_timeout_when_controller_silent(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            plan = build_plan(_make_points(tmp))
        holder = {}

        def start_fake(link):
            holder["c"] = FakeController(link.port, silent_at=1)
            holder["c"].start()

        with RobotLink("127.0.0.1", 0, timeout_s=0.3, accept_timeout_s=5,
                       on_listening=start_fake) as link:
            with self.assertRaises(ProtocolError):
                run_plan(link, plan, log=lambda *_: None, sleep=lambda _s: None)
        holder["c"].join(5)


class CliTest(unittest.TestCase):
    def test_dry_run_prints_messages(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            points = _make_points(tmp)
            out = io.StringIO()
            with redirect_stdout(out):
                code = main([str(points), "--dry-run"])
            self.assertEqual(code, 0)
            lines = [l for l in out.getvalue().splitlines() if l.startswith("M0")]
            self.assertEqual(len(lines), 4)

    def test_missing_file(self):
        err = io.StringIO()
        with redirect_stderr(err):
            code = main(["/tmp/does-not-exist-9011.json", "--dry-run"])
        self.assertEqual(code, 2)
        self.assertIn("错误", err.getvalue())

    def test_accept_timeout_reports_actionable_error(self):
        link = RobotLink("127.0.0.1", 0, timeout_s=1, accept_timeout_s=0.3)
        with self.assertRaises(ProtocolError) as ctx:
            link.__enter__()
        message = str(ctx.exception)
        self.assertIn("ENET1.IP", message)
        self.assertIn("9011", message)


if __name__ == "__main__":
    unittest.main()


class NetworkPreflightTest(unittest.TestCase):
    """IP 方向易搞反：控制器是客户端，PC 是服务端；绑错地址会“监听成功但连不上”。"""

    def test_loopback_is_allowed(self):
        for host in ("127.0.0.1", "localhost", "0.0.0.0", ""):
            check_bind_address(host)          # 不应抛异常

    def test_foreign_ip_rejected_with_local_list(self):
        with self.assertRaises(ProtocolError) as ctx:
            check_bind_address("192.0.2.77")   # TEST-NET-1，几乎不可能在本机
        message = str(ctx.exception)
        self.assertIn("192.0.2.77", message)
        self.assertIn("IPv4", message)

    def test_dry_run_does_not_require_local_ip(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            points = _make_points(tmp)
            out = io.StringIO()
            with redirect_stdout(out):
                code = main([str(points), "--dry-run", "--host", "192.0.2.77"])
            self.assertEqual(code, 0)
            self.assertEqual(len([l for l in out.getvalue().splitlines()
                                  if l.startswith("M0")]), 4)

    def test_expect_peer_rejects_unexpected_client(self):
        def start_fake(link):
            threading.Thread(
                target=lambda: (time.sleep(0.1),
                                socket.create_connection(("127.0.0.1", link.port),
                                                         timeout=5)),
                daemon=True).start()

        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            plan = build_plan(_make_points(tmp))
        link = RobotLink("127.0.0.1", 0, timeout_s=3, accept_timeout_s=5,
                         on_listening=start_fake, expect_peer="192.168.3.13")
        with self.assertRaises(ProtocolError) as ctx:
            link.__enter__()
        self.assertIn("192.168.3.13", str(ctx.exception))

    def test_expect_peer_accepts_matching_client(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            plan = build_plan(_make_points(tmp))
        holder = {}

        def start_fake(link):
            holder["c"] = FakeController(link.port)
            holder["c"].start()

        with RobotLink("127.0.0.1", 0, timeout_s=3, accept_timeout_s=5,
                       on_listening=start_fake, expect_peer="127.0.0.1") as link:
            self.assertEqual(link.peer, "127.0.0.1")
            run_plan(link, plan, log=lambda *_: None, sleep=lambda _s: None)
        holder["c"].join(5)


class TerminatorTest(unittest.TestCase):
    """OPENMSG 的帧分隔/帧尾是控制器侧设置，与报文结束符必须一致。"""

    def test_terminator_variants(self):
        point = {"slot": 100, "coord": 1, "data": [0.0] * 6}
        for name, expected in (("lf", "\n"), ("crlf", "\r\n"),
                               ("semicolon", ";"), ("lf-semicolon", "\n;")):
            with self.subTest(terminator=name):
                text = point_to_message(point, "P-001.00000010", name)
                self.assertTrue(text.endswith(expected))
                self.assertEqual(len(text), 69 + len(expected))

    def test_unknown_terminator(self):
        with self.assertRaises(ProtocolError):
            point_to_message({"slot": 1, "coord": 1, "data": [0.0] * 6},
                             "P-001.00000010", "nul")

    def test_semicolon_framing_round_trip(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            plan = build_plan(_make_points(tmp), terminator="semicolon")
            self.assertTrue(all(line.endswith(";") for line in plan["lines"]))
            self.assertEqual(plan["terminator"], "semicolon")

    def test_reader_accepts_semicolon_framed_replies(self):
        """下位若回 ``done;ready;``，读取端也必须能拆开。"""
        holder = {}

        def start_fake(link):
            def client():
                time.sleep(0.1)
                with socket.create_connection(("127.0.0.1", link.port),
                                              timeout=5) as sock:
                    sock.sendall(b"done;ready;")     # 分号分帧，一次到齐
                    time.sleep(0.5)
            threading.Thread(target=client, daemon=True).start()

        with RobotLink("127.0.0.1", 0, timeout_s=3, accept_timeout_s=5,
                       on_listening=start_fake) as link:
            done, ready = link.wait_handshake()
        self.assertEqual((done, ready), ("done", "ready"))
