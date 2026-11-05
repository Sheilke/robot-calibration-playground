# -*- coding: utf-8 -*-
"""GP 寄存器导出（14 字段布局）的单元测试。"""

import json
import tempfile
import unittest
from pathlib import Path

from excitation_trajectory.export_gp import (
    FLAG_INDEX,
    GP_FIELD_COUNT,
    JOINT_SLICE,
    load_scan_points,
    make_gp_entry,
    main,
    write_gp_entries,
)

#: 现场已创建并被用户确认为正确的样例：J1..J6 = 1/52/3/4/5/6
SAMPLE_GP = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 52.0, 3.0, 4.0, 5.0, 6.0, 0.0]


def _scan_payload(count=3):
    steps = [{"index": i, "ok": True,
              "q": [float(i + 1), 52.0 - i, 3.0 + i, 4.0, 5.0, 6.0 - i],
              "config": [0, -1, -1]} for i in range(count)]
    steps.append({"index": count, "ok": False, "q": None, "config": None})
    return {"meta": {}, "waypoints": [], "perturbed_ik": steps}


def _template():
    regs = {f"GP{i:04d}": [0] * 14 for i in range(1, 101)}
    regs["GP0001"] = list(SAMPLE_GP)
    return {"controllerID": "ABC", "globalPosition": regs,
            "note": {"GP0001": ""}, "version": "1.0.5"}


class LayoutTest(unittest.TestCase):
    def test_sample_round_trip(self):
        """用样例的关节值重建，必须逐位等于现场样例。"""
        entry = make_gp_entry([1, 52, 3, 4, 5, 6])
        self.assertEqual(entry, SAMPLE_GP)
        self.assertEqual(len(entry), GP_FIELD_COUNT)
        self.assertEqual(entry[FLAG_INDEX], 0.0)
        self.assertEqual(entry[JOINT_SLICE], [1.0, 52.0, 3.0, 4.0, 5.0, 6.0])

    def test_cartesian_and_flag(self):
        entry = make_gp_entry([1, 2, 3, 4, 5, 6], cart=(10, 20, 30, 0.5, 0.25, 0.1),
                              flag=1, ext=None or 0.0)
        self.assertEqual(entry[0:6], [10.0, 20.0, 30.0, 0.5, 0.25, 0.1])
        self.assertEqual(entry[FLAG_INDEX], 1.0)
        self.assertEqual(entry[JOINT_SLICE], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_invalid_joint_values(self):
        for bad in ([1, 2, 3, 4, 5], [1, 2, 3, 4, 5, float("nan")],
                    [1, 2, 3, 4, 5, 360.0], [1, 2, 3, 4, 5, -400]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                make_gp_entry(bad)

    def test_invalid_cartesian_length(self):
        with self.assertRaises(ValueError):
            make_gp_entry([1, 2, 3, 4, 5, 6], cart=(0, 0, 0))


class WriteTest(unittest.TestCase):
    def setUp(self):
        self.points = [{"index": i, "q": [float(i), 10.0, -20.0, 0.0, 30.0, -5.0],
                        "config": [0, -1, -1]} for i in range(4)]

    def test_writes_requested_slots_and_preserves_rest(self):
        template = _template()
        updated, report = write_gp_entries(template, self.points, first=1)
        self.assertEqual(report["count"], 4)
        self.assertEqual(report["first"], "GP0001")
        self.assertEqual(report["last"], "GP0004")
        for i, point in enumerate(self.points, start=1):
            self.assertEqual(updated["globalPosition"][f"GP{i:04d}"][JOINT_SLICE],
                             point["q"])
        # 5..100 原样
        for i in range(5, 101):
            self.assertEqual(updated["globalPosition"][f"GP{i:04d}"], [0] * 14)
        self.assertEqual(updated["controllerID"], "ABC")

    def test_first_offset(self):
        updated, report = write_gp_entries(_template(), self.points, first=47)
        self.assertEqual(report["first"], "GP0047")
        self.assertEqual(report["last"], "GP0050")
        self.assertEqual(updated["globalPosition"]["GP0047"][JOINT_SLICE],
                         self.points[0]["q"])

    def test_overwrites_the_sample_entry(self):
        """GP0001 原本是现场样例，写入后应被新值替换（不是拒绝）。"""
        updated, _ = write_gp_entries(_template(), self.points, first=1)
        self.assertNotEqual(updated["globalPosition"]["GP0001"], SAMPLE_GP)

    def test_rejects_missing_global_position(self):
        with self.assertRaises(ValueError):
            write_gp_entries({"controllerID": "X"}, self.points)


class CliTest(unittest.TestCase):
    def test_end_to_end(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            root = Path(tmp)
            scan = root / "scan.json"
            scan.write_text(json.dumps(_scan_payload(50)), encoding="utf-8")
            tpl = root / "position_R1.json"
            tpl.write_text(json.dumps(_template()), encoding="utf-8")
            out = root / "out.json"
            code = main([str(scan), str(tpl), "-o", str(out)])
            self.assertEqual(code, 0)
            payload = json.loads(out.read_text(encoding="utf-8"))
            filled = [k for k, v in payload["globalPosition"].items()
                      if any(v)]
            self.assertEqual(len(filled), 50)
            self.assertIn("GP0050", filled)
            self.assertNotIn("GP0051", filled)
            # 模板文件必须保持原状
            self.assertEqual(json.loads(tpl.read_text(encoding="utf-8")),
                             _template())

    def test_missing_ik_results(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            root = Path(tmp)
            scan = root / "scan.json"
            scan.write_text(json.dumps({"meta": {}}), encoding="utf-8")
            tpl = root / "p.json"
            tpl.write_text(json.dumps(_template()), encoding="utf-8")
            with self.assertRaises(SystemExit):
                main([str(scan), str(tpl), "-o", str(root / "o.json")])

    def test_load_scan_points_skips_failures(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            scan = Path(tmp) / "s.json"
            scan.write_text(json.dumps(_scan_payload(3)), encoding="utf-8")
            points = load_scan_points(scan)
            self.assertEqual([p["index"] for p in points], [0, 1, 2])


if __name__ == "__main__":
    unittest.main()
