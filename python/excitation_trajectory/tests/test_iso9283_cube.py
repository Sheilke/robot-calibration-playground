#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISO 9283 五点生成：布点几何、立方体求解、与 export_gp 的衔接。"""
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from excitation_trajectory.iso9283_cube import (
    LAYOUTS,
    POINT_NAMES,
    build_arg_parser,
    cube_points,
    cube_vertices,
    find_largest_cube,
    main,
    scan_payload,
    solve_points,
)
from robot_model.nexbot_ik import Nexbot6R

REPO = Path(__file__).resolve().parents[3]
ROBOT = REPO / "data" / "configs" / "hyundai_general6s.urdf"
RPY = (3.0, 73.0, -167.0)
Q_SEED = [43.0, -30.0, -36.0, 42.0, -89.0, -1.0]


def _robot():
    return Nexbot6R(str(ROBOT))


class CubePointsGeometryTest(unittest.TestCase):
    """三种布点的几何必须与文档一致（这是容易被悄悄改错的地方）。"""

    CENTER = (1000.0, 0.0, 600.0)
    SIDE = 400.0

    def test_vertices_uses_full_half_side_on_all_axes(self):
        points = cube_points(self.CENTER, self.SIDE, "vertices")
        h = self.SIDE / 2.0
        self.assertEqual(sorted(points), sorted(POINT_NAMES))
        np.testing.assert_allclose(points["P1"], self.CENTER)
        for name in POINT_NAMES[1:]:
            offset = np.asarray(points[name]) - np.asarray(self.CENTER)
            # 四个顶点：每个分量的绝对值都等于 h（三轴都用满）
            np.testing.assert_allclose(np.abs(offset), [h, h, h])
            self.assertAlmostEqual(float(np.linalg.norm(offset)), h * math.sqrt(3))
        # 四点两两不重合，且都落在立方体顶点上
        corners = {tuple(np.round(v, 6)) for v in cube_vertices(self.CENTER, self.SIDE)}
        for name in POINT_NAMES[1:]:
            self.assertIn(tuple(np.round(points[name], 6)), corners)

    def test_vertices_four_points_are_coplanar_on_the_diagonal_plane(self):
        """四个 P2..P5 应在同一对角截面上（这是 ISO 9283 的取法）。"""
        points = cube_points(self.CENTER, self.SIDE, "vertices")
        normals = []
        for name in POINT_NAMES[1:]:
            offset = np.asarray(points[name]) - np.asarray(self.CENTER)
            normals.append(offset / np.linalg.norm(offset))
        # 对角截面的四个角在 ±(1,1,1)/√3 方向上交替取号，平面法向与 (1,-1,1) 等垂直
        # 更直接：四个点都在平面 x - y + z = const 之类上，用秩判共面
        matrix = np.array([np.asarray(points[name]) for name in POINT_NAMES[1:]])
        matrix = matrix - matrix.mean(axis=0)
        self.assertLessEqual(int(np.linalg.matrix_rank(matrix, tol=1e-9)), 2)

    def test_circle_radius_is_half_side(self):
        points = cube_points(self.CENTER, self.SIDE, "circle")
        h = self.SIDE / 2.0
        for name in POINT_NAMES[1:]:
            offset = np.asarray(points[name]) - np.asarray(self.CENTER)
            self.assertAlmostEqual(float(np.linalg.norm(offset)), h, places=6)

    def test_center_square_matches_original_script(self):
        """center-square 必须与 Downloads 原脚本逐点一致（k=0.4）。"""
        points = cube_points(self.CENTER, self.SIDE, "center-square", 0.4)
        c = np.asarray(self.CENTER)
        h = self.SIDE / 2.0
        expected = {
            "P1": c,
            "P2": c + np.array([-0.4 * h, -0.4 * h, -0.4 * h]),
            "P3": c + np.array([0.4 * h, -0.4 * h, -0.4 * h]),
            "P4": c + np.array([0.4 * h, 0.4 * h, -0.4 * h]),
            "P5": c + np.array([-0.4 * h, 0.4 * h, -0.4 * h]),
        }
        for name in POINT_NAMES:
            np.testing.assert_allclose(points[name], expected[name], atol=1e-9)

    def test_rejects_bad_input(self):
        for side in (0.0, -1.0, float("inf")):
            with self.assertRaises(ValueError):
                cube_points(self.CENTER, side, "vertices")
        with self.assertRaises(ValueError):
            cube_points(self.CENTER, self.SIDE, "nope")
        with self.assertRaises(ValueError):
            cube_points(self.CENTER, self.SIDE, "center-square", 0.0)


class SolvePointsTest(unittest.TestCase):
    """真实模型上解五点：全部可达、构型一致、姿态不被扰动。"""

    def test_five_points_solve_in_one_configuration(self):
        rbt = _robot()
        points = cube_points((1025.0, 0.0, 605.0), 550.0, "vertices")
        results = solve_points(rbt, points, RPY, order="XYZ", extrinsic=False,
                               q_seed=Q_SEED)
        self.assertEqual(sorted(results), sorted(POINT_NAMES))
        for name in POINT_NAMES:
            self.assertTrue(results[name]["ok"], name)
            self.assertIsNotNone(results[name]["q"])
            self.assertEqual(len(results[name]["q"]), 6)
        configs = {tuple(results[name]["config"]) for name in POINT_NAMES}
        self.assertEqual(len(configs), 1, f"五点构型应一致，实际 {configs}")

    def test_unreachable_point_is_reported_not_raised(self):
        rbt = _robot()
        far = cube_points((8000.0, 0.0, 600.0), 100.0, "vertices")
        results = solve_points(rbt, far, RPY, order="XYZ", extrinsic=False,
                               q_seed=Q_SEED)
        self.assertFalse(any(results[name]["ok"] for name in POINT_NAMES))


class FindLargestCubeTest(unittest.TestCase):
    """自动搜立方体：可行性判据看 8 个顶点，结果单调且确定。"""

    def test_finds_a_cube_and_is_deterministic(self):
        rbt = _robot()
        box = [800, 1200, -300, 300, 400, 900]
        first = find_largest_cube(rbt, box, RPY, order="XYZ", extrinsic=False,
                                  q_seed=Q_SEED, resolution=200.0)
        second = find_largest_cube(rbt, box, RPY, order="XYZ", extrinsic=False,
                                   q_seed=Q_SEED, resolution=200.0)
        self.assertTrue(first["ok"])
        self.assertGreater(first["side"], 100.0)
        self.assertEqual(first["center"], second["center"])
        self.assertEqual(first["side"], second["side"])

    def test_all_eight_vertices_reachable_at_reported_side(self):
        from excitation_trajectory.iso9283_cube import cube_reachable
        rbt = _robot()
        box = [800, 1200, -300, 300, 400, 900]
        found = find_largest_cube(rbt, box, RPY, order="XYZ", extrinsic=False,
                                  q_seed=Q_SEED, resolution=200.0)
        ok, results = cube_reachable(rbt, found["center"], found["side"], RPY,
                                     order="XYZ", extrinsic=False,
                                     q_seed=Q_SEED)
        self.assertTrue(ok, [i for i, r in enumerate(results) if not r["ok"]])

    def test_prefer_x_min_filters_candidates(self):
        rbt = _robot()
        box = [600, 1400, -200, 200, 400, 900]
        found = find_largest_cube(rbt, box, RPY, order="XYZ", extrinsic=False,
                                  q_seed=Q_SEED, resolution=200.0,
                                  prefer_x_min=1200.0)
        if found["ok"]:
            self.assertGreaterEqual(found["center"][0], 1200.0)

    def test_rejects_inverted_box(self):
        rbt = _robot()
        with self.assertRaises(ValueError):
            find_largest_cube(rbt, [1200, 800, -300, 300, 400, 900], RPY,
                              order="XYZ", extrinsic=False, q_seed=Q_SEED)


class ScanPayloadTest(unittest.TestCase):
    """输出必须能被 export_gp 直接消费。"""

    def test_payload_shape_and_export_gp_roundtrip(self):
        from excitation_trajectory.export_gp import load_scan_points

        rbt = _robot()
        points = cube_points((1025.0, 0.0, 605.0), 550.0, "vertices")
        results = solve_points(rbt, points, RPY, order="XYZ", extrinsic=False,
                               q_seed=Q_SEED)
        payload = scan_payload(points, results,
                               cube={"center": [1025.0, 0.0, 605.0],
                                     "bounds": [750, 1300, -275, 275, 330, 880]},
                               layout="vertices", side=550.0, rpy=RPY,
                               convention="xyz-intrinsic", motion_hint="MOVL")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "iso.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            loaded = load_scan_points(path)        # export_gp 的入口
        self.assertEqual(len(loaded), 5)
        for index, point in enumerate(loaded):
            self.assertEqual(point["index"], index)
            self.assertEqual(len(point["q"]), 6)

    def test_meta_records_layout_and_motion(self):
        rbt = _robot()
        points = cube_points((1025.0, 0.0, 605.0), 300.0, "vertices")
        results = solve_points(rbt, points, RPY, order="XYZ", extrinsic=False,
                               q_seed=Q_SEED)
        payload = scan_payload(points, results,
                               cube={"center": [1025.0, 0.0, 605.0],
                                     "bounds": [875, 1175, -150, 150, 455, 755]},
                               layout="vertices", side=300.0, rpy=RPY,
                               convention="xyz-intrinsic", motion_hint="MOVL")
        meta = payload["meta"]
        self.assertEqual(meta["source"], "iso9283_cube")
        self.assertEqual(meta["layout"], "vertices")
        self.assertEqual(meta["motion"], "MOVL")
        self.assertEqual(meta["point_count"], 5)
        for waypoint in payload["waypoints"]:
            self.assertIsNone(waypoint["rpy_pert"])   # 姿态一致，不扰动


class CliTest(unittest.TestCase):
    """CLI 三种给法与错误路径。"""

    def test_cube_form_writes_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "iso.json"
            code = main(["--robot", str(ROBOT), "--cube", "750", "1300",
                         "-600", "600", "270", "940",
                         "--rpy", "3", "73", "-167", "--euler", "xyz-intrinsic",
                         "--q-seed", "43,-30,-36,42,-89,-1",
                         "--out", str(out)])
            self.assertEqual(code, 0)
            payload = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(len(payload["waypoints"]), 5)
            self.assertEqual(payload["meta"]["cube_side"], 550.0)
            self.assertEqual(payload["meta"]["cube_center"], [1025.0, 0.0, 605.0])

    def test_center_side_form_with_plot(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "iso.json"
            png = Path(tmp) / "iso.png"
            code = main(["--robot", str(ROBOT), "--center", "1000", "0", "600",
                         "--side", "400", "--rpy", "3", "73", "-167",
                         "--euler", "xyz-intrinsic",
                         "--q-seed", "43,-30,-36,42,-89,-1",
                         "--layout", "circle",
                         "--out", str(out), "--plot", str(png)])
            self.assertEqual(code, 0)
            self.assertTrue(png.exists())
            self.assertGreater(png.stat().st_size, 4096)

    def test_missing_side_is_rejected(self):
        # 校验在 main 里做（parser 本身只负责解析），所以这里调 main
        with self.assertRaises(SystemExit):
            main(["--robot", str(ROBOT), "--center", "1000", "0", "600"])

    def test_unreachable_cube_exits_with_2(self):
        with tempfile.TemporaryDirectory() as tmp:
            code = main(["--robot", str(ROBOT), "--center", "8000", "0", "600",
                         "--side", "100", "--rpy", "3", "73", "-167",
                         "--euler", "xyz-intrinsic",
                         "--q-seed", "43,-30,-36,42,-89,-1",
                         "--out", str(Path(tmp) / "x.json")])
            self.assertEqual(code, 2)


if __name__ == "__main__":
    unittest.main()
