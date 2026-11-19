# -*- coding: utf-8 -*-
"""``laser_web.core.run_iso9283`` 与 ``laser_web.figures.iso_cube_figure`` 的单元测试。

不起 UI，只跑算法与小案子：站点参数（``center 1025 0 605 / side 550``）单次解算
在本机实测约 0.3 s，整组测试 1 s 内跑完；产物目录用 ``LASER_WEB_RUNS`` 指向临时
目录，绝不往仓库根的 ``runs/`` 里丢垃圾。

覆盖：

1. 中心+边长（站点参数）：5/5 可达、构型一致，且 **P1 的 q 与 CLI 实测值逐分量
   一致（±0.01°）**——钉死「页面路径与 CLI 走同一套」，以后谁在 core 里另写一份
   五点/IK 逻辑，这条就先炸；
2. 范围模式：三轴跨度不等时取最小跨度为边长、各轴中点为心；
3. 自动搜索：搜到的边长必须比 100 mm 大（否则说明二分/判定退化了）；
4. 产物：``iso.json`` 能被 ``export_gp.load_scan_points`` 解析出 5 条；
   ``iso.png`` 真的画出来了（> 4 KB 才算有内容，空白图的 PNG 只有几百字节）；
5. 失败路径：无解与缺参数都返回 ``ok: False`` 且不抛异常；
6. 图形：``iso_cube_figure`` 的 trace 数 ≤ 4，空点集不抛异常。
"""

import os
import tempfile
import unittest
from pathlib import Path

from laser_web import core, figures, jobs

#: 站点参数（与 README「功能四」的 CLI 验收例子逐项相同）。
SITE_PARAMS = {"robot": "data/configs/hyundai_general6s.urdf",
               "cube_mode": "center",
               "center": [1025.0, 0.0, 605.0],
               "side": 550.0,
               "layout": "vertices",
               "plane": "xz",
               "rpy": [3.0, 73.0, -167.0],
               "euler": "xyz-intrinsic",
               "q_seed": [43.0, -30.0, -36.0, 42.0, -89.0, -1.0]}
#: CLI 实测的 P1 关节角（``iso9283_cube`` 的 main 打出来的那一行，6 位小数）。
CLI_P1_Q = [0.105, 3.812, -50.039, 1.095, -63.21, 15.345]


def _params(**overrides):
    """一份完整的入参（每次返回新 dict，列表也是新的）。"""
    params = {key: (list(value) if isinstance(value, list) else value)
              for key, value in SITE_PARAMS.items()}
    params.update(overrides)
    return params


class IsoRunTest(unittest.TestCase):
    """跑一次站点参数的解算，大部分断言都读这一份结果。"""

    @classmethod
    def setUpClass(cls):
        """把产物根目录挪到临时目录，跑一次中心+边长的解算。"""
        cls._tmp = tempfile.TemporaryDirectory()
        cls._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = cls._tmp.name
        cls.stages = []
        cls.result = core.run_iso9283(
            _params(), on_progress=lambda stage, index, total:
            cls.stages.append((stage, index, total)))
        cls.run_dir = Path(cls.result["run_dir"])

    @classmethod
    def tearDownClass(cls):
        """恢复环境变量并清临时目录。"""
        if cls._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = cls._previous_runs
        cls._tmp.cleanup()

    def test_site_params_are_reachable(self):
        """站点参数 5/5 可解、构型一致，五个点都 ok。"""
        result = self.result
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["stats"]["total"], 5)
        self.assertEqual(result["stats"]["solved"], 5)
        self.assertTrue(result["stats"]["reachable"])
        self.assertTrue(result["stats"]["config_consistent"])
        self.assertEqual(len(result["points"]), 5)
        self.assertEqual([row["name"] for row in result["points"]],
                         ["P1", "P2", "P3", "P4", "P5"])
        for row in result["points"]:
            self.assertTrue(row["ok"], row)
            self.assertEqual(len(row["q"]), 6, row)
            self.assertEqual(len(row["pos"]), 3)
            self.assertAlmostEqual(row["j3_abs"], abs(row["q"][2]), places=6)
            self.assertAlmostEqual(row["j5_abs"], abs(row["q"][4]), places=6)
        self.assertEqual(result["stats"]["j3_abs_min"],
                         min(row["j3_abs"] for row in result["points"]))
        self.assertEqual(result["stats"]["j5_abs_min"],
                         min(row["j5_abs"] for row in result["points"]))

    def test_p1_matches_cli(self):
        """P1 的 q 必须与 CLI 实测值逐分量相同（页面与 CLI 同一套算法）。"""
        p1 = self.result["points"][0]
        self.assertEqual(p1["name"], "P1")
        self.assertEqual(len(p1["q"]), 6)
        for index, (actual, expected) in enumerate(zip(p1["q"], CLI_P1_Q)):
            self.assertAlmostEqual(actual, expected, delta=0.01,
                                   msg=f"q{index + 1}: {actual} vs {expected}")

    def test_progress_stages(self):
        """进度回调按阶段上报（顺序即 ISO_STAGES）。"""
        seen = []
        for stage, _index, _total in self.stages:
            if not seen or seen[-1] != stage:
                seen.append(stage)
        self.assertEqual(seen, list(core.ISO_STAGES))

    def test_result_shape(self):
        """返回结构齐全，meta 与 iso.json 的 meta 同一份。"""
        result = self.result
        for key in ("ok", "run_id", "run_dir", "stats", "points", "meta",
                    "artifacts", "warnings"):
            self.assertIn(key, result)
        self.assertEqual(result["artifacts"],
                         {"iso_json": "iso.json", "plot": "iso.png"})
        self.assertEqual(result["meta"]["source"], "iso9283_cube")
        self.assertEqual(result["meta"]["layout"], "vertices")
        self.assertEqual(result["meta"]["plane"], "xz")
        self.assertEqual(result["meta"]["cube_side"], 550.0)
        self.assertEqual(result["meta"]["cube_center"], [1025.0, 0.0, 605.0])
        self.assertEqual(result["stats"]["cube_side"], 550.0)
        self.assertEqual(result["stats"]["cube_center"], [1025.0, 0.0, 605.0])
        # 五点姿态完全一致，所以没有扰动字段
        self.assertTrue(all(row["rpy_pert"] is None
                            for row in _scan_json(self.run_dir)["waypoints"]))

    def test_artifacts(self):
        """iso.json 能被 export_gp 解析出 5 条；iso.png 有内容（> 4 KB）。"""
        from excitation_trajectory import export_gp

        json_path = self.run_dir / self.result["artifacts"]["iso_json"]
        png_path = self.run_dir / self.result["artifacts"]["plot"]
        self.assertTrue(json_path.is_file(), json_path)
        self.assertTrue(png_path.is_file(), png_path)
        points = export_gp.load_scan_points(json_path)
        self.assertEqual(len(points), 5)
        self.assertEqual([point["index"] for point in points],
                         [0, 1, 2, 3, 4])
        for point in points:
            self.assertEqual(len(point["q"]), 6)
        self.assertGreater(png_path.stat().st_size, 4 * 1024)


class IsoCubeModeTest(unittest.TestCase):
    """另外两种立方体来源（范围 / 自动搜索）。"""

    def setUp(self):
        """每次用新的临时产物目录。"""
        self._tmp = tempfile.TemporaryDirectory()
        self._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = self._tmp.name

    def tearDown(self):
        """恢复环境变量并清临时目录。"""
        if self._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = self._previous_runs
        self._tmp.cleanup()

    def test_bounds_mode_takes_min_span(self):
        """``cube`` 三轴跨度不等：边长取最小跨度、中心取各轴中点。"""
        result = core.run_iso9283(_params(cube_mode="bounds", center=None,
                                          side=None,
                                          cube=[750, 1300, -600, 600, 270, 940]))
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["stats"]["cube_side"], 550.0)
        self.assertEqual(result["stats"]["cube_center"], [1025.0, 0.0, 605.0])
        # meta 里的 aabb 是**实际立方体**的边界（中心 ± 半边长），不是输入的搜索箱
        self.assertEqual(result["meta"]["aabb"], [750.0, 1300.0, -275.0, 275.0,
                                                  330.0, 880.0])

    def test_auto_mode_finds_a_cube(self):
        """自动搜索：找得到，且边长不是退化的几毫米。"""
        result = core.run_iso9283(_params(cube_mode="auto", center=None,
                                          side=None,
                                          search_box=[800, 1200, -300, 300,
                                                      400, 900],
                                          resolution=200.0))
        self.assertTrue(result["ok"], result.get("error"))
        self.assertGreater(result["stats"]["cube_side"], 100.0)
        center = result["stats"]["cube_center"]
        for value, low, high in zip(center, (800, -300, 400), (1200, 300, 900)):
            self.assertGreaterEqual(value, low)
            self.assertLessEqual(value, high)


class IsoFailureTest(unittest.TestCase):
    """失败一律走 ``ok: False``，不抛异常（否则 jobs 层会记 failed）。"""

    def setUp(self):
        """每次用新的临时产物目录。"""
        self._tmp = tempfile.TemporaryDirectory()
        self._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = self._tmp.name

    def tearDown(self):
        """恢复环境变量并清临时目录。"""
        if self._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = self._previous_runs
        self._tmp.cleanup()

    def test_unreachable_cube_is_a_failure_not_a_crash(self):
        """远在机器人够不到的地方：5 个点都无解 -> ok False + 非空 error。"""
        result = core.run_iso9283(_params(center=[8000.0, 0.0, 600.0],
                                          side=100.0))
        self.assertFalse(result["ok"])
        self.assertTrue(result["error"])
        self.assertEqual(result["stats"]["solved"], 0)
        self.assertFalse(result["stats"]["reachable"])

    def test_missing_side_is_a_parameter_error(self):
        """``cube_mode="center"`` 但没给 side：报参数错，不抛异常。"""
        result = core.run_iso9283({"robot": SITE_PARAMS["robot"],
                                   "cube_mode": "center",
                                   "center": [1025.0, 0.0, 605.0]})
        self.assertFalse(result["ok"])
        self.assertIn("side", result["error"])
        self.assertEqual(result["warnings"], [])

    def test_bad_layout_and_euler_are_rejected(self):
        """取值域外的东西一律在进算法之前挡下（消息里带上合法取值）。"""
        layout = core.run_iso9283(_params(layout="grid"))
        self.assertFalse(layout["ok"])
        self.assertIn("vertices", layout["error"])
        euler = core.run_iso9283(_params(euler="zyz-extrinsic"))
        self.assertFalse(euler["ok"])
        self.assertIn("xyz-intrinsic", euler["error"])

    def test_missing_robot_is_a_parameter_error(self):
        """没有模型就没有 IK，直接报参数错。"""
        result = core.run_iso9283({})
        self.assertFalse(result["ok"])
        self.assertTrue(result["error"])


class IsoFigureTest(unittest.TestCase):
    """``figures.iso_cube_figure``：trace 预算与空点集的退化行为。"""

    def _points(self, ok: bool = True) -> list[dict]:
        """一份最小的五点行（只有画图要用的键）。"""
        rows = []
        for index, name in enumerate(("P1", "P2", "P3", "P4", "P5")):
            rows.append({"name": name,
                         "pos": [1000.0 + index * 10.0, -5.0 + index, 600.0],
                         "q": [1.0] * 6 if ok else None,
                         "ok": ok,
                         "config": [0, -1, -1] if ok else None,
                         "j3_abs": 50.0 if ok else None,
                         "j5_abs": 60.0 if ok else None})
        return rows

    def test_trace_budget(self):
        """立方体棱 + 可解点（+ 无解点）最多 4 个 trace。"""
        figure = figures.iso_cube_figure(self._points(), cube_center=[1025, 0, 605],
                                         side=550.0, layout="vertices",
                                         plane="xz")
        self.assertLessEqual(len(figure.data), 4)
        self.assertGreaterEqual(len(figure.data), 2)
        self.assertEqual(figure.layout.scene.aspectmode, "data")
        self.assertEqual(figure.layout.scene.xaxis.title.text, "X (mm)")

    def test_hover_mentions_name_and_margins(self):
        """hover 至少含点名、XYZ、|J3|、|J5|、ok。"""
        figure = figures.iso_cube_figure(self._points(), cube_center=[1025, 0, 605],
                                         side=550.0, layout="vertices",
                                         plane="xz")
        hover = " ".join(str(text) for trace in figure.data
                         for text in (trace.hovertext or ()))
        for token in ("P1", "X ", "Y ", "Z ", "|J3|", "|J5|", "ok"):
            self.assertIn(token, hover)

    def test_mixed_and_empty_points(self):
        """失败点单独一个 trace；空点集返回空 figure 而不是抛异常。"""
        rows = self._points(ok=True)
        rows[2] = dict(rows[2], ok=False, q=None, config=None,
                       j3_abs=None, j5_abs=None)
        figure = figures.iso_cube_figure(rows, cube_center=[1025, 0, 605],
                                         side=550.0, layout="vertices",
                                         plane="xz")
        self.assertEqual(len(figure.data), 3)
        empty = figures.iso_cube_figure([], cube_center=[1025, 0, 605],
                                        side=550.0, layout="vertices",
                                        plane="xz")
        self.assertEqual(len(empty.data), 0)
        none = figures.iso_cube_figure(None, cube_center=None, side=None,
                                       layout="vertices", plane="xz")
        self.assertEqual(len(none.data), 0)


def _scan_json(run_dir: Path) -> dict:
    """读 ``runs/<id>/iso.json``（断言产物内容用）。"""
    import json

    return json.loads((run_dir / core.ARTIFACT_ISO_JSON).read_text("utf-8"))


if __name__ == "__main__":
    unittest.main()


class ManualPointsModeTest(unittest.TestCase):
    """「人工指定五点」：逐点给坐标，不要立方体。

    现场经常已经有一组坐标（来自旧 JOB 或别的表），要求能直接填进来算关节角，
    而不是被迫先反推一个立方体。
    """

    BASE = {"robot": "data/configs/hyundai_general6s.urdf",
            "cube_mode": "points", "rpy": [3, 73, -167],
            "euler": "xyz-intrinsic",
            "q_seed": [43, -30, -36, 42, -89, -1], "j5_min": 20}
    #: 与 ``cube_points`` 的 ISO 9283 编号一致：P2/P3 小 X 小 Z、P4/P5 大 X 大 Z。
    POINTS = [[1025, 0, 605], [750, 275, 330], [750, -275, 330],
              [1300, 275, 880], [1300, -275, 880]]

    def test_manual_points_solve_without_a_cube(self):
        result = core.run_iso9283(dict(self.BASE, points=self.POINTS))
        self.assertTrue(result["ok"], result.get("error"))
        stats = result["stats"]
        self.assertEqual(stats["solved"], 5)
        self.assertEqual(stats["total"], 5)
        self.assertIsNone(stats["cube_side"])       # 没有立方体，不造假数字
        self.assertIsNone(stats["cube_center"])
        self.assertEqual(result["meta"]["points_source"], "manual")
        self.assertIsNone(result["meta"]["cube_center"])
        # 坐标必须原样回显（人工填什么就算什么）
        for row, expected in zip(result["points"], self.POINTS):
            self.assertEqual(row["pos"], [float(v) for v in expected])

    def test_manual_matches_cube_mode_on_the_same_coordinates(self):
        """同一批坐标：人工指定 与 立方体模式 解出的关节角必须完全一致。"""
        manual = core.run_iso9283(dict(self.BASE, points=self.POINTS))
        cube = core.run_iso9283({"robot": self.BASE["robot"],
                                 "cube_mode": "bounds",
                                 "cube": [750, 1300, -600, 600, 270, 940],
                                 "rpy": self.BASE["rpy"],
                                 "euler": self.BASE["euler"],
                                 "q_seed": self.BASE["q_seed"],
                                 "j5_min": self.BASE["j5_min"]})
        self.assertTrue(manual["ok"] and cube["ok"])
        for got, want in zip(manual["points"], cube["points"]):
            self.assertEqual(got["q"], want["q"], got["name"])

    def test_rejects_wrong_point_count(self):
        result = core.run_iso9283(dict(self.BASE, points=self.POINTS[:4]))
        self.assertFalse(result["ok"])
        self.assertIn("5", result["error"])

    def test_rejects_missing_points(self):
        params = dict(self.BASE)
        params.pop("cube_mode")
        params["cube_mode"] = "points"
        result = core.run_iso9283(params)
        self.assertFalse(result["ok"])
        self.assertIn("points", result["error"])
