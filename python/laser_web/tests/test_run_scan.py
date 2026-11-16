# -*- coding: utf-8 -*-
"""``laser_web.core.run_scan`` 的单元测试：不起 UI，只跑算法与小案子。

案子刻意小（``--target-count 8``，8 个点、7 段），单次解算在本机实测约
0.4 s，整组测试 1~2 s 内跑完；产物目录用 ``LASER_WEB_RUNS`` 指向临时目录，
绝不往仓库根的 ``runs/`` 里丢垃圾。

覆盖（对应设计文档 §9 的单测一栏）：

1. 返回结构：``ok`` / ``stats`` / ``waypoints`` / ``segments`` / ``artifacts``；
2. ``scan.json`` 落盘且字段与 CLI 输出一致（``meta`` / ``waypoints`` /
   ``singularity_guard`` / ``segment_check`` / ``conditioning`` /
   ``perturbed_ik``）；
3. ``joints.txt`` 8 行、每行 6 个以空格分隔的 3 位小数、GP 顺序；
4. ``plot.png`` 真的画出来了；
5. ``skip_ik=True`` 时不解 IK（``q`` 全 ``None``、``solved == 0``）但仍出图；
6. 校验失败走 ``ok: False``，不抛异常；
7. 进度回调按阶段上报。
"""

import json
import os
import tempfile
import unittest
from pathlib import Path

from laser_web import core, jobs

#: 8 点小案子：域 200 x 200 x 200 mm，够小到 1 s 内跑完。
SMALL_DOMAIN = {"point_min": [900.0, -100.0, 500.0],
                "point_max": [1100.0, 100.0, 700.0]}
#: 与站点一致的姿态/种子参数（换小域不改变这些）。
BASE_PARAMS = {"robot": "data/configs/ch2000_origin.urdf",
               "rpy": [3.0, 73.0, -167.0],
               "euler": "xyz-intrinsic",
               "perturb_deg": [20.0, 20.0, 35.0],
               "target_count": 8,
               "q_seed": "43,-30,-36,42,-89,-1",
               "j5_min": 20.0,
               "seed": 0}


def _params(**overrides):
    """一份完整的入参（每次返回新 dict，列表也是新的）。"""
    params = dict(BASE_PARAMS)
    params.update({key: (list(value) if isinstance(value, list) else value)
                   for key, value in SMALL_DOMAIN.items()})
    params.update(overrides)
    return params


class RunScanTest(unittest.TestCase):
    """跑一次完整解算，全部断言都读这一份结果。"""

    @classmethod
    def setUpClass(cls):
        """把产物根目录挪到临时目录，跑一次完整解算。"""
        cls._tmp = tempfile.TemporaryDirectory()
        cls._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = cls._tmp.name
        cls.stages = []
        cls.result = core.run_scan(
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

    def test_result_shape(self):
        """返回结构齐全，8 点全部解出。"""
        result = self.result
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["stats"]["solved"], 8)
        self.assertEqual(result["stats"]["total"], 8)
        self.assertEqual(len(result["waypoints"]), 8)
        for row in result["waypoints"]:
            self.assertEqual(len(row["q"]), 6, row)
            self.assertTrue(row["ok"], row)
            self.assertEqual(len(row["pos"]), 3)
            self.assertAlmostEqual(row["j3_abs"], abs(row["q"][2]))
            self.assertAlmostEqual(row["j5_abs"], abs(row["q"][4]))
        self.assertEqual(result["stats"]["j5_abs_min"],
                         min(row["j5_abs"] for row in result["waypoints"]))
        # GP 从 1 起算，index 从 0 起算（生成 GP 时直接按 gp 排序）。
        self.assertEqual([row["gp"] for row in result["waypoints"]],
                         list(range(1, 9)))
        self.assertTrue(result["stats"]["feasible"])

    def test_segments(self):
        """段诊断：8 点 -> 7 段，GP 号与最差点都在。"""
        segments = self.result["segments"]
        self.assertEqual(len(segments), 7)
        for index, segment in enumerate(segments):
            self.assertEqual(segment["gp_from"], index + 1)
            self.assertEqual(segment["gp_to"], index + 2)
            self.assertIsNotNone(segment["j3_abs_min"])
            self.assertIsNotNone(segment["j5_abs_min"])
            self.assertFalse(segment["crosses"])
            self.assertTrue(0.0 <= segment["worst_frac"] <= 1.0)
            self.assertEqual(len(segment["worst_pos"]), 3)

    def test_scan_json(self):
        """scan.json 落盘，字段与 CLI 输出逐项对齐。"""
        payload = json.loads((self.run_dir / "scan.json").read_text("utf-8"))
        for key in ("meta", "waypoints", "singularity_guard", "segment_check",
                    "conditioning", "perturbed_ik"):
            self.assertIn(key, payload)
        for key in ("aabb", "rpy", "xs", "ys", "zs", "z_insert",
                    "motion_order", "grid_source", "target_count",
                    "aabb_source", "seed", "euler_convention", "skip_ik",
                    "no_perturb", "perturbed"):
            self.assertIn(key, payload["meta"])
        self.assertEqual(len(payload["waypoints"]), 8)
        self.assertEqual(len(payload["perturbed_ik"]), 8)
        self.assertEqual(len(payload["segment_check"]["segments"]), 7)
        self.assertFalse(payload["meta"]["skip_ik"])
        self.assertEqual(payload["meta"]["target_count"], 8)
        self.assertEqual(payload["meta"]["euler_convention"], "xyz-intrinsic")
        self.assertIn("design", payload["conditioning"])
        self.assertIn("post_ik", payload["conditioning"])
        self.assertEqual(payload["singularity_guard"]["j5_min"], 20.0)

    def test_artifacts_exist(self):
        """三个产物都写出来了，joints 行数=路点数，图片不是空文件。"""
        artifacts = self.result["artifacts"]
        self.assertEqual(artifacts["scan_json"], "scan.json")
        self.assertEqual(artifacts["joints"], "joints.txt")
        self.assertEqual(artifacts["plot"], "plot.png")
        for name in artifacts.values():
            self.assertTrue((self.run_dir / name).is_file(), name)
        self.assertGreater((self.run_dir / "plot.png").stat().st_size, 1024)

    def test_joints_txt(self):
        """joints.txt：8 行、每行 6 个数、空格分隔、<=3 位小数、GP 顺序。"""
        lines = (self.run_dir / "joints.txt").read_text("utf-8").splitlines()
        self.assertEqual(len(lines), 8)
        for line, row in zip(lines, self.result["waypoints"]):
            parts = line.split()
            self.assertEqual(len(parts), 6, line)
            for part, expected in zip(parts, row["q"]):
                self.assertLessEqual(len(part.split(".")[-1]), 3, part)
                self.assertAlmostEqual(float(part), expected, places=3)

    def test_progress_stages(self):
        """进度回调按阶段上报，且同一阶段内 index 不减。"""
        stages = [stage for stage, _index, _total in self.stages]
        for name in core.STAGES:
            if name in ("采样姿态", "段内检查"):
                continue        # 这两段也可能被跳过（无扰动/无相邻段）
            self.assertIn(name, stages)
        ik = [(index, total) for stage, index, total in self.stages
              if stage == "解算 IK"]
        self.assertEqual(ik[0][0], 0)
        self.assertEqual(ik[-1], (8, 8))
        self.assertTrue(all(index <= total
                            for _stage, index, total in self.stages))

    def test_warnings_carry_cli_hints(self):
        """校验阶段的 CLI 提示（自动凑数）要透传到 warnings。"""
        joined = " ".join(self.result["warnings"])
        self.assertIn("自动凑数", joined)


class RunScanPreviewTest(unittest.TestCase):
    """``skip_ik=True``：等价 CLI ``--skip-ik``，只出路点与预览图。"""

    @classmethod
    def setUpClass(cls):
        """临时产物目录 + 一次预览。"""
        cls._tmp = tempfile.TemporaryDirectory()
        cls._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = cls._tmp.name
        cls.result = core.run_scan(_params(skip_ik=True))
        cls.run_dir = Path(cls.result["run_dir"])

    @classmethod
    def tearDownClass(cls):
        """恢复环境变量并清临时目录。"""
        if cls._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = cls._previous_runs
        cls._tmp.cleanup()

    def test_no_ik(self):
        """预览不解 IK：q 全 None、solved=0，但路点还在。"""
        self.assertTrue(self.result["ok"], self.result.get("error"))
        self.assertEqual(len(self.result["waypoints"]), 8)
        self.assertEqual(self.result["stats"]["solved"], 0)
        self.assertTrue(self.result["meta"]["skip_ik"])
        self.assertFalse(self.result["meta"]["perturbed"])
        for row in self.result["waypoints"]:
            self.assertIsNone(row["q"])
            self.assertIsNone(row["j3_abs"])
            self.assertFalse(row["ok"])

    def test_artifacts(self):
        """预览也出图出 JSON，但没有 joints.txt（没有关节角可写）。"""
        self.assertTrue((self.run_dir / "plot.png").is_file())
        self.assertGreater((self.run_dir / "plot.png").stat().st_size, 1024)
        payload = json.loads((self.run_dir / "scan.json").read_text("utf-8"))
        self.assertTrue(payload["meta"]["skip_ik"])
        self.assertEqual(payload["perturbed_ik"], [])
        self.assertIsNone(payload["segment_check"])
        self.assertEqual(set(self.result["artifacts"]),
                         {"scan_json", "plot"})


class RunScanFailureTest(unittest.TestCase):
    """失败路径：报错原文回显、不抛异常、不建半个结果。"""

    def test_invalid_domain(self):
        """六数式下界大于上界 -> ok: False + CLI 报错原文。

        注意两点式（``point_min`` / ``point_max``）会把顺序相反的轴按分量
        取小/取大归一化并打一行提示，那不是错误，所以这里用六数式。
        """
        result = core.run_scan(_params(point_min=None, point_max=None,
                                       aabb6=[1100.0, 900.0, -100.0, 100.0,
                                              500.0, 700.0]))
        self.assertFalse(result["ok"])
        self.assertIn("大于上界", result["error"])
        self.assertEqual(result["warnings"], [])

    def test_bad_cond_max(self):
        """条件数上限 < 1 -> 仍走 ``_validate_args`` 的原文报错。"""
        result = core.run_scan(_params(cond_max=0.5))
        self.assertFalse(result["ok"])
        self.assertIn("cond-max", result["error"])

    def test_bad_q_seed(self):
        """q-seed 个数不对 -> 仍是 ok: False（走 CLI 的 ``_parse_q_seed``）。"""
        result = core.run_scan(_params(q_seed="1,2,3"))
        self.assertFalse(result["ok"])
        self.assertIn("6 个浮点数", result["error"])

    def test_bad_seg_mode(self):
        """未知段模型直接拒绝，而不是悄悄退回 joint。"""
        result = core.run_scan(_params(seg_mode="movl"))
        self.assertFalse(result["ok"])
        self.assertIn("seg_mode", result["error"])


if __name__ == "__main__":
    unittest.main()


class GuardParityTest(unittest.TestCase):
    """守卫真的重采时，scan.json 仍必须与 CLI 逐字段一致。

    这是评审抓到的回归点：早先 ``evaluate_conditioning`` 放在 ``solve_chain_guarded``
    之后，拿的是**重采后**的位姿，于是 ``conditioning.realized`` 与 CLI 不同；守卫
    没触发时（例如站点参数）恰好一致，所以人工核对发现不了。
    """

    PARAMS = {
        "robot": "data/configs/hyundai_general6s.urdf",
        "point_min": [900, -100, 500], "point_max": [1100, 100, 700],
        "rpy": [3, 73, -167], "euler": "xyz-intrinsic",
        "perturb_deg": [20, 20, 35], "target_count": 8,
        "seg_mode": "cartesian", "seg_samples": 11,
        "q_seed": [43, -30, -36, 42, -89, -1],
        "j5_min": 88, "j3_min": 0, "seg_min": 5, "cond_max": 99, "seed": 0,
    }

    def test_scan_matches_cli_while_guard_resamples(self):
        import json
        import subprocess
        import sys
        from pathlib import Path

        result = core.run_scan(dict(self.PARAMS))
        self.assertTrue(result["ok"], result)
        web = json.loads((Path(result["run_dir"]) / "scan.json").read_text("utf-8"))
        self.assertGreater(len(web["singularity_guard"]["resampled"]), 0,
                           "本用例必须真的触发重采，否则测不到回归")

        cli_out = Path("/tmp") / "laser_web_guard_parity.json"
        subprocess.run(
            [sys.executable, "-m", "excitation_trajectory.aabb_snake_scan",
             "900", "1100", "-100", "100", "500", "700",
             "--target-count", "8",
             "--urdf", "data/configs/hyundai_general6s.urdf",
             "--rpy", "3", "73", "-167", "--euler", "xyz-intrinsic",
             "--perturb-deg", "20", "20", "35", "--j5-min", "88",
             "--q-seed", "43,-30,-36,42,-89,-1", "--seed", "0",
             "--cond-max", "99", "--seg-mode", "cartesian",
             "--seg-samples", "11",
             "--out", str(cli_out), "--plot", "/tmp/laser_web_guard_parity.png"],
            check=True, capture_output=True)
        cli = json.loads(cli_out.read_text("utf-8"))
        for key in ("conditioning", "perturbed_ik", "singularity_guard",
                    "segment_check", "waypoints"):
            self.assertEqual(web[key], cli[key], f"{key} 与 CLI 不一致")
        # meta 除 aabb_source 外应完全一致：界面用两点式输入，CLI 这条命令用
        # 六个数，如实记录来源是设计如此（两者归一化后的边界相同）。
        web_meta = dict(web["meta"], aabb_source=None)
        cli_meta = dict(cli["meta"], aabb_source=None)
        self.assertEqual(web_meta, cli_meta, "meta 与 CLI 不一致")
        self.assertEqual(web["meta"]["aabb_source"], "points")

    def test_feasible_is_false_when_guard_violations_remain(self):
        """有守卫违规时不能报"可行"——判据要与 evaluate_domain 一致。"""
        result = core.run_scan(dict(self.PARAMS))
        self.assertTrue(result["ok"], result)
        self.assertGreater(result["stats"]["guard_violations"], 0)
        self.assertFalse(result["stats"]["feasible"])
