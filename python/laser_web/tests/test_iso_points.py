# -*- coding: utf-8 -*-
"""ISO 五点页的**只算坐标**这条路：``core.iso_points`` / ``iso_points_payload``
/ ``save_iso_points`` / ``figures.iso_points_figure``。

用户 2026-11-16 当面要求：ISO 页只算笛卡尔坐标，IK 只解一次判“可达”，**不输出
关节角**、不加姿态扰动。这里的断言就是钉这条：

1. 结果里没有任何关节量（``q`` / ``config`` / ``j3_abs`` / ``j5_abs``）；
2. 汇总只有 ``可达 N/5``；不可达不算失败（``ok`` 仍为真，行上标红）；
3. 落盘的 ``iso.json`` 是每点 X/Y/Z + 参考姿态（**没有** ``waypoints`` /
   ``perturbed_ik`` 那种带 ``q`` 的结构）；
4. 图的悬浮文案里没有 ``|J3|`` / ``|J5|`` / ``cfg``。

站点参数（``center 1025 0 605 / side 550`` 等）在本机实测整次 ``iso_points``
约 10 ms；产物目录用 ``LASER_WEB_RUNS`` 指向临时目录，不往仓库 ``runs/`` 丢垃圾。
"""

import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from laser_web import core, figures, jobs

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT = "data/configs/hyundai_general6s.urdf"
RPY = [3.0, 73.0, -167.0]


def _params(**overrides) -> dict:
    """一份完整的入参（只有页面真正提交的键：没有 q_seed / j5_min）。"""
    params = {"robot": ROBOT,
              "cube_mode": "bounds",
              "cube": [925.0, 1125.0, -100.0, 100.0, 505.0, 705.0],
              "layout": "vertices",
              "plane": "xz",
              "rpy": list(RPY),
              "euler": "xyz-intrinsic"}
    params.update(overrides)
    return params


class IsoPointsTest(unittest.TestCase):
    """``core.iso_points``：坐标 + 可达性，没有关节量。"""

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = cls._tmp.name
        previous = os.getcwd()
        os.chdir(REPO_ROOT)
        try:
            cls.result = core.iso_points(_params())
        finally:
            os.chdir(previous)

    @classmethod
    def tearDownClass(cls):
        if cls._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = cls._previous_runs
        cls._tmp.cleanup()

    def test_points_carry_only_positions_and_reachability(self):
        self.assertTrue(self.result["ok"], self.result.get("error"))
        self.assertEqual([row["name"] for row in self.result["points"]],
                         list(core.iso.POINT_NAMES))
        for row in self.result["points"]:
            self.assertEqual(set(row), {"name", "pos", "reachable"}, row["name"])
            self.assertEqual(len(row["pos"]), 3)
            self.assertIsInstance(row["reachable"], bool)

    def test_summary_and_meta(self):
        self.assertEqual(self.result["summary"], {"reachable": 5, "total": 5})
        meta = self.result["meta"]
        self.assertEqual(meta["source"], "iso9283_points")
        self.assertEqual(meta["points_source"], "cube")
        self.assertEqual(meta["cube_side"], 200.0)
        self.assertEqual(meta["cube_center"], [1025.0, 0.0, 605.0])
        self.assertEqual(meta["rpy"], RPY)
        self.assertEqual(meta["layout"], "vertices")
        self.assertEqual(meta["plane"], "xz")
        self.assertIn("不输出关节角", meta["note"])
        self.assertEqual(self.result["warnings"], [])

    def test_numbering_is_p2p3_small_p4p5_large(self):
        """编号按 ISO 9283：P2/P3 小 X 小 Z，P4/P5 大 X 大 Z。"""
        points = {row["name"]: row["pos"] for row in self.result["points"]}
        center = self.result["meta"]["cube_center"]
        for name in ("P2", "P3"):
            self.assertLess(points[name][0], center[0], name)
            self.assertLess(points[name][2], center[2], name)
        for name in ("P4", "P5"):
            self.assertGreater(points[name][0], center[0], name)
            self.assertGreater(points[name][2], center[2], name)

    def test_unreachable_points_are_reported_not_raised(self):
        """立方体在够不到的地方：``ok`` 仍为真，只是每点都不可达。"""
        result = core.iso_points(_params(cube=[7900.0, 8100.0, -100.0, 100.0,
                                               500.0, 700.0]))
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["summary"]["reachable"], 0)
        self.assertTrue(all(not row["reachable"] for row in result["points"]))
        self.assertTrue(result["warnings"])
        self.assertIn("不可达", result["warnings"][0])

    def test_bad_cube_is_a_parameter_error(self):
        result = core.iso_points(_params(cube=[1200.0, 1000.0, -100.0, 100.0,
                                               500.0, 700.0]))
        self.assertFalse(result["ok"])
        self.assertIn("下界必须小于上界", result["error"])

    def test_reachability_seed_is_the_cli_default(self):
        """可达性验算用固定种子（与 CLI 的默认 ``--q-seed`` 同一个数）。"""
        self.assertEqual(core.iso_reachability_seed(),
                         core._iso_numbers(core.iso_defaults()["q_seed"], 6,
                                           "q_seed"))
        self.assertEqual(core.iso_reachability_seed(), [0, 20, -20, 0, 40, 0])


class IsoPointsArtifactsTest(unittest.TestCase):
    """落盘的 ``iso.json``：每点 X/Y/Z + 参考姿态，没有关节角。"""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="iso-points-"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_payload_shape(self):
        previous = os.getcwd()
        os.chdir(REPO_ROOT)
        try:
            result = core.iso_points(_params())
        finally:
            os.chdir(previous)
        payload = core.iso_points_payload(result)
        self.assertEqual(payload["meta"]["point_count"], 5)
        self.assertEqual(len(payload["points"]), 5)
        for row in payload["points"]:
            self.assertEqual(set(row), {"name", "pos", "rpy", "reachable"})
            self.assertEqual(row["rpy"], RPY, "五点姿态完全相同")
        self.assertNotIn("waypoints", payload)
        self.assertNotIn("perturbed_ik", payload)
        self.assertNotIn("q", json.dumps(payload))

    def test_save_writes_json_and_plot_without_joints(self):
        previous = os.getcwd()
        try:
            os.chdir(REPO_ROOT)
            result = core.iso_points(_params())
        finally:
            os.chdir(previous)
        saved = core.save_iso_points(result, run_dir=self.tmp)
        self.assertEqual(saved["run_dir"], str(self.tmp))
        json_path = self.tmp / core.ARTIFACT_ISO_JSON
        png_path = self.tmp / core.ARTIFACT_ISO_PLOT
        self.assertTrue(json_path.is_file())
        self.assertTrue(png_path.is_file())
        self.assertGreater(png_path.stat().st_size, 4096, "PNG 应当有内容")
        payload = json.loads(json_path.read_text(encoding="utf-8"))
        self.assertEqual(len(payload["points"]), 5)
        self.assertNotIn('"q"', json_path.read_text(encoding="utf-8"))

    def test_save_refuses_a_failed_result(self):
        with self.assertRaises(ValueError):
            core.save_iso_points({"ok": False, "error": "x"}, run_dir=self.tmp)


class IsoPointsFigureTest(unittest.TestCase):
    """``figures.iso_points_figure``：两色 + 悬停只有点名与 XYZ。"""

    def points(self):
        return [{"name": "P1", "pos": [1025.0, 0.0, 605.0], "reachable": True},
                {"name": "P2", "pos": [925.0, 100.0, 505.0], "reachable": False}]

    def test_two_colored_traces_without_joint_hover(self):
        figure = figures.iso_points_figure(
            self.points(), cube_center=[1025.0, 0.0, 605.0], side=200.0)
        names = [str(trace.name) for trace in figure.data]
        self.assertEqual(len(names), 3, names)
        self.assertIn("立方体 L=200 mm", names[0])
        self.assertIn("可达 1/2", names[1])
        self.assertIn("不可达 1", names[2])
        for trace in figure.data[1:]:
            for text in trace.hovertext:
                self.assertNotIn("|J3|", text)
                self.assertNotIn("|J5|", text)
                self.assertNotIn("cfg", text)
                self.assertRegex(text, r"X .* mm · Y .* mm · Z .* mm")

    def test_empty_points_give_an_empty_figure(self):
        figure = figures.iso_points_figure([], cube_center=None, side=None)
        self.assertEqual(list(figure.data), [])

    def test_old_figure_still_shows_joints(self):
        """``iso_cube_figure``（命令行/GP 那条路）保持原样，不受这次收窄影响。"""
        rows = [{"name": "P1", "pos": [1025.0, 0.0, 605.0], "ok": True,
                 "j3_abs": 50.04, "j5_abs": 63.21, "config": [0, -1, -1]}]
        figure = figures.iso_cube_figure(rows, cube_center=[1025.0, 0.0, 605.0],
                                         side=550.0, layout="vertices",
                                         plane="xz")
        text = "".join(figure.data[1].hovertext)
        self.assertIn("|J5|", text)


if __name__ == "__main__":
    unittest.main()
