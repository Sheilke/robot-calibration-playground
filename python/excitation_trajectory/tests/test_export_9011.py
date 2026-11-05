# -*- coding: utf-8 -*-
"""9011 点位/报文导出（``export_9011``）的单元测试。

关键约束来自下位 JOB 的定宽字符串切片：

* 点位记录 ``coord(1) + 每轴 8 字(2 位小数) + P 编号(4)``，6 轴即 53 字；
* 报文上限 240 字符（``NexbotProtocol::MaxMessageLength``），且 ``G`` 头的
  点数是 1 位数字 → 单包最多 9 点；
* 行走记录 ``T + begin(4) + end(4) + type(1) + speed(7) + acc(1) + tool(1)
  + ratio(4)`` = 23 字；
* ``T`` 循环内**没有停留指令**，6 秒暂停必须由上位发送循环执行。
"""

import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from excitation_trajectory.export_9011 import (
    MAX_MESSAGE_LENGTH,
    _fixed_float,
    _fixed_int,
    group_messages,
    main,
    move_data_to_str,
    position_to_str,
    walk_messages,
)

ARM_AXES = 6


def _decode_point(text: str) -> tuple:
    """独立解码一条点位记录（不复用被测的编码函数）。"""
    assert len(text) == 53, f"记录长度应为 53，实际 {len(text)}"
    coord = int(text[0])
    values = [float(text[1 + i * 8: 9 + i * 8]) for i in range(6)]
    slot = int(text[49:53])
    return coord, values, slot


class FixedWidthTest(unittest.TestCase):
    def test_fixed_float_matches_cpp_double_to_str(self):
        # C++: std::format("{:0{}.{}f}", num, fill, accuracy) / 负数走 "-" + fill-1
        self.assertEqual(_fixed_float(16.20491), "00016.20")
        self.assertEqual(_fixed_float(-16.20491), "-0016.20")
        self.assertEqual(_fixed_float(0.0), "00000.00")
        self.assertEqual(_fixed_float(-0.005), "-0000.01")
        self.assertEqual(_fixed_float(99999.99), "99999.99")   # 8 字宽上限

    def test_fixed_float_rejects_overflow(self):
        with self.assertRaises(ValueError):
            _fixed_float(123456.78)             # 9 字，超宽
        with self.assertRaises(ValueError):
            _fixed_float(-123456.78)

    def test_fixed_int_matches_cpp_int_to_str(self):
        self.assertEqual(_fixed_int(1, 1), "1")
        self.assertEqual(_fixed_int(0, 1), "0")
        self.assertEqual(_fixed_int(46, 4), "0046")
        self.assertEqual(_fixed_int(-3, 4), "-003")

    def test_move_data_width_is_fourteen(self):
        text = move_data_to_str()               # P, -1, acc 0, tool 0, 10%
        self.assertEqual(text, "P-001.00000010")
        self.assertEqual(len(text), 14)

    def test_position_record_width_is_53_for_six_axes(self):
        text = position_to_str(1, [0.0] * ARM_AXES, 1)
        self.assertEqual(len(text), 1 + 8 * ARM_AXES + 4)
        self.assertEqual(text, "100000.0000000.0000000.0000000.0000000.0000000.000001")


class GroupMessageTest(unittest.TestCase):
    def _points(self, count):
        return [(1, [float(i), -float(i), 0.0, 1.5, -2.5, 3.5]) for i in range(count)]

    def test_chunk_size_and_header(self):
        chunks = group_messages(self._points(48), slot_start=1)
        self.assertEqual(len(chunks), 12)       # 48 / 4
        for slots, message in chunks:
            self.assertTrue(message.startswith(f"G{len(slots)}"))
            self.assertTrue(message.endswith("\n"))
            self.assertLessEqual(len(message), MAX_MESSAGE_LENGTH)
            self.assertLessEqual(len(slots), 9)  # G 头的计数只有 1 位
        flat = [slot for slots, _ in chunks for slot in slots]
        self.assertEqual(flat, list(range(1, 49)))

    def test_round_trip_decode(self):
        points = self._points(48)
        chunks = group_messages(points, slot_start=1)
        decoded = []
        for slots, message in chunks:
            body = message.strip()[2:]          # 去掉 "G{n}" 与换行
            self.assertEqual(len(body), 53 * len(slots))
            for index in range(len(slots)):
                decoded.append(_decode_point(body[index * 53:(index + 1) * 53]))
        self.assertEqual(len(decoded), len(points))
        for position, ((coord, data, slot),
                       (ref_coord, ref_data)) in enumerate(zip(decoded, points), start=1):
            self.assertEqual(coord, ref_coord)
            self.assertEqual(slot, position)     # 槽号从 slot_start 连续递增
            self.assertEqual([round(v, 2) for v in data],
                             [round(v, 2) for v in ref_data])

    def test_slot_start_is_respected(self):
        chunks = group_messages(self._points(5), slot_start=11)
        self.assertEqual(chunks[0][0], [11, 12, 13, 14])
        self.assertEqual(chunks[1][0], [15])

    def test_single_point(self):
        chunks = group_messages(self._points(1), slot_start=1)
        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0][1][:2], "G1")


class WalkMessageTest(unittest.TestCase):
    def test_one_t_per_point_with_pause_comments(self):
        lines = walk_messages([1, 2, 3], move_data_to_str(), 6.0)
        moves = [line for line in lines if line.startswith("T")]
        pauses = [line for line in lines if line.startswith("#")]
        self.assertEqual(moves, ["T00010001P-001.00000010\n",
                                 "T00020002P-001.00000010\n",
                                 "T00030003P-001.00000010\n"])
        self.assertEqual(len(pauses), 2)        # 最后一点后不停留
        self.assertIn("sleep 6 s", pauses[0])
        for move in moves:
            self.assertTrue(move.endswith("\n"))
            self.assertEqual(len(move.rstrip("\n")), 23)

    def test_single_point_has_no_pause(self):
        lines = walk_messages([7], move_data_to_str(), 6.0)
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith("T"))


class CliTest(unittest.TestCase):
    @staticmethod
    def _scan_payload(count=5):
        steps = [{"index": i, "ok": True,
                  "q": [float(i), 10.0 + i, -20.0 - i, 0.5, 40.0, -30.0 + i],
                  "config": [1, 1, 1]} for i in range(count)]
        steps.append({"index": count, "ok": False, "q": None, "config": None})
        return {"meta": {"seed": 0}, "waypoints": [],
                "perturbed_ik": steps}

    def _run(self, argv):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = main(argv)
        return code, out.getvalue(), err.getvalue()

    def test_end_to_end(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            root = Path(tmp)
            scan = root / "scan.json"
            scan.write_text(json.dumps(self._scan_payload(5)), encoding="utf-8")
            out_json = root / "points.json"
            out_msg = root / "messages.txt"
            argv = [str(scan), "--out", str(out_json),
                    "--messages", str(out_msg), "--dwell", "6"]
            code, stdout, _ = self._run(argv)
            self.assertEqual(code, 0, msg=stdout)
            self.assertIn("点位 5 个", stdout)

            payload = json.loads(out_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["meta"]["n_points"], 5)
            self.assertEqual(payload["meta"]["dwell_s"], 6.0)
            self.assertEqual(payload["meta"]["coord"], "axis")
            self.assertEqual([p["slot"] for p in payload["points"]], [1, 2, 3, 4, 5])
            # 失败的 lot 被跳过
            self.assertEqual([p["index"] for p in payload["points"]], [0, 1, 2, 3, 4])

            text = out_msg.read_text(encoding="utf-8")
            self.assertIn("点位下载", text)
            self.assertIn("逐点行走", text)
            self.assertTrue(text.rstrip().endswith("Q0"))
            self.assertEqual(text.count("sleep 6 s"), 4)   # 5 点 → 4 处暂停

    def test_axis_coordinates_round_trip_to_scan(self):
        """报文里的轴角必须与 scan_result 的 q 一致（量化到 0.01 度）。"""
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            root = Path(tmp)
            payload = self._scan_payload(3)
            scan = root / "scan.json"
            scan.write_text(json.dumps(payload), encoding="utf-8")
            out_msg = root / "m.txt"
            code, _, _ = self._run([str(scan), "--out", str(root / "p.json"),
                                    "--messages", str(out_msg)])
            self.assertEqual(code, 0)
            first_line = next(line for line in out_msg.read_text(
                encoding="utf-8").splitlines() if line.startswith("G"))
            body = first_line.strip()[2:]
            decoded = [_decode_point(body[i * 53:(i + 1) * 53])
                       for i in range(len(body) // 53)]
            self.assertEqual(len(decoded), 3)
            for (coord, values, slot), step in zip(decoded,
                                                   payload["perturbed_ik"][:3]):
                self.assertEqual(coord, 1)
                self.assertEqual(slot, step["index"] + 1)
                self.assertEqual(values, [round(v, 2) for v in step["q"]])

    def test_invalid_dwell(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            scan = Path(tmp) / "scan.json"
            scan.write_text(json.dumps(self._scan_payload(2)), encoding="utf-8")
            for dwell in ("0", "-3"):
                with self.assertRaises(SystemExit):
                    main([str(scan), "--out", str(Path(tmp) / "p.json"),
                          "--messages", str(Path(tmp) / "m.txt"),
                          "--dwell", dwell])

    def test_missing_ik_results(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            scan = Path(tmp) / "scan.json"
            scan.write_text(json.dumps({"meta": {}, "waypoints": []}),
                            encoding="utf-8")
            with self.assertRaises(SystemExit):
                main([str(scan), "--out", str(Path(tmp) / "p.json"),
                      "--messages", str(Path(tmp) / "m.txt")])

    def test_all_points_failed(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            scan = Path(tmp) / "scan.json"
            scan.write_text(json.dumps({"perturbed_ik": [
                {"index": 0, "ok": False, "q": None}]}), encoding="utf-8")
            with self.assertRaises(SystemExit):
                main([str(scan), "--out", str(Path(tmp) / "p.json"),
                      "--messages", str(Path(tmp) / "m.txt")])


if __name__ == "__main__":
    unittest.main()
