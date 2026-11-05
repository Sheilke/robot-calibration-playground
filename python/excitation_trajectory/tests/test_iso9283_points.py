# -*- coding: utf-8 -*-
"""ISO 9283 P1～P5 生成的单元测试（几何关系 + 可达性 + CLI）。"""

import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

from excitation_trajectory.iso9283_points import (
    CORNER_ANGLES_DEG,
    cube_center,
    iso9283_points,
    main,
    solve_points,
)
from robot_model.nexbot_ik import Nexbot6R

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "hyundai_general6s.urdf"
CUBE = (750.0, 1420.0, -345.0, 325.0, 460.0, 1130.0)   # 边长 670


class GeometryTest(unittest.TestCase):
    def test_side_defaults_to_shortest_edge(self):
        points = iso9283_points(CUBE)
        center = cube_center(CUBE)
        np.testing.assert_allclose(points[0]["pos"], center)

    def test_p1_is_center_and_others_symmetric(self):
        points = iso9283_points(CUBE)
        center = cube_center(CUBE)
        np.testing.assert_allclose(points[0]["pos"], center, atol=1e-9)
        # P2..P5 相对中心对称，和为零
        offsets = [p["offset"] for p in points[1:]]
        np.testing.assert_allclose(np.sum(offsets, axis=0), np.zeros(3), atol=1e-9)

    def test_corner_radius_is_half_side(self):
        side = 670.0
        points = iso9283_points(CUBE, side=side)
        for entry in points[1:]:
            self.assertAlmostEqual(float(np.linalg.norm(entry["offset"])),
                                   side / 2.0, places=9)

    def test_corners_span_90_degrees(self):
        side = 670.0
        points = iso9283_points(CUBE, side=side)
        offsets = [np.asarray(p["offset"]) for p in points[1:]]
        for first, second in zip(offsets, offsets[1:] + offsets[:1]):
            cosine = float(np.dot(first, second) /
                           (np.linalg.norm(first) * np.linalg.norm(second)))
            self.assertAlmostEqual(cosine, 0.0, places=9)   # 相邻 90°
        self.assertEqual(len(CORNER_ANGLES_DEG), 4)

    def test_square_side_matches_diagonal_formula(self):
        """P2..P5 构成边长 L/√2 的正方形（ISO 9283 对角平面内切圆）。"""
        side = 670.0
        points = iso9283_points(CUBE, side=side)
        positions = [np.asarray(p["pos"]) for p in points[1:]]
        for first, second in zip(positions, positions[1:] + positions[:1]):
            self.assertAlmostEqual(float(np.linalg.norm(second - first)),
                                   side / math.sqrt(2.0), places=6)

    def test_all_points_inside_cube(self):
        side = 670.0
        x0, x1, y0, y1, z0, z1 = CUBE
        for entry in iso9283_points(CUBE, side=side):
            x, y, z = entry["pos"]
            for value, low, high in ((x, x0, x1), (y, y0, y1), (z, z0, z1)):
                self.assertGreaterEqual(value, low - 1e-9)
                self.assertLessEqual(value, high + 1e-9)

    def test_side_too_large_rejected(self):
        with self.assertRaises(ValueError):
            iso9283_points(CUBE, side=2000.0)


class SolveTest(unittest.TestCase):
    def test_all_reachable_and_same_configuration(self):
        rbt = Nexbot6R(str(URDF))
        points = iso9283_points(CUBE)
        results = solve_points(rbt, points, (3.0, 73.0, -167.0),
                              extrinsic=False,
                              q_seed=[43, -30, -36, 42, -89, -1])
        for entry in results:
            self.assertIsNotNone(entry["q_deg"], msg=entry["name"])
            self.assertLess(entry["fk_residual_mm"], 1e-6)
            self.assertGreater(entry["abs_j5_deg"], 20.0, msg=entry["name"])
            self.assertGreater(entry["abs_j3_deg"], 10.0, msg=entry["name"])
        configs = {tuple(e["config"]) for e in results}
        self.assertEqual(len(configs), 1, msg=f"5 点构型不一致: {configs}")


class CliTest(unittest.TestCase):
    def _run(self, extra):
        out, = (io.StringIO(),)
        with redirect_stdout(out):
            code = main(["--cube", *[str(v) for v in CUBE],
                         "--urdf", str(URDF), "--rpy", "3", "73", "-167",
                         "--euler", "xyz-intrinsic",
                         "--q-seed", "43,-30,-36,42,-89,-1"] + extra)
        return code, out.getvalue()

    def test_reports_success(self):
        code, text = self._run([])
        self.assertEqual(code, 0)
        self.assertIn("5 点同构型", text)
        for name in ("P1", "P2", "P3", "P4", "P5"):
            self.assertIn(name, text)

    def test_writes_json(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            path = Path(tmp) / "iso5.json"
            code, _ = self._run(["--out", str(path)])
            self.assertEqual(code, 0)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual([p["name"] for p in payload["points"]],
                             ["P1", "P2", "P3", "P4", "P5"])
            self.assertEqual(payload["cube"]["side"], 670.0)
            self.assertEqual(payload["reference_pose"]["euler_convention"],
                             "xyz-intrinsic")

    def test_gp_export_fills_cartesian_and_joints(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            root = Path(tmp)
            template = root / "position_R1.json"
            template.write_text(json.dumps({
                "controllerID": "X", "version": "1.0.5",
                "globalPosition": {f"GP{i:04d}": [0] * 14 for i in range(1, 101)},
                "note": {},
            }), encoding="utf-8")
            out = root / "gp.json"
            code, text = self._run(["--gp-template", str(template),
                                    "--gp-out", str(out)])
            self.assertEqual(code, 0, msg=text)
            payload = json.loads(out.read_text(encoding="utf-8"))
            for i, point in enumerate(
                    iso9283_points(CUBE), start=1):
                entry = payload["globalPosition"][f"GP{i:04d}"]
                np.testing.assert_allclose(entry[0:3], point["pos"], atol=1e-6)
                self.assertEqual(entry[3:6], [3.0, 73.0, -167.0])
            self.assertEqual(payload["globalPosition"]["GP0006"], [0] * 14)

    def test_invalid_cube_rejected(self):
        with self.assertRaises(SystemExit):
            main(["--cube", "100", "50", "0", "100", "0", "100",
                  "--urdf", str(URDF), "--rpy", "0", "0", "0"])


if __name__ == "__main__":
    unittest.main()
