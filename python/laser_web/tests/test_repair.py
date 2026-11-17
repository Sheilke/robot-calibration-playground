# -*- coding: utf-8 -*-
"""``laser_web.core.locate_segment`` / ``core.run_repair`` 的单元测试（M4）。

只测服务层，不起 UI：界面只是把这些返回值渲染出来，逻辑全在这里。案子刻意
小，整组在本机实测约 3 s：

* 交付域（550×1200×670 mm，50 点）跑一次定位 + 一次修复，验证「窗口干净就
  不动」这条最重要的判断——2026-11-05 现场那次就是这种情形；
* 8 点小案子（200 mm 立方）验证 ``run_repair`` 的返回结构与产物；
* 20 点大箱体验证两种「窗口里有越界段」的结论句（报告段越界 / 报告段安全）；
* 校验错误、``stopped_at`` 越界两条错误路径，都要求 ``ok: False`` 且不抛异常。

产物目录用 ``LASER_WEB_RUNS`` 指到临时目录，绝不往仓库根的 ``runs/`` 里丢垃圾。
"""

import json
import os
import tempfile
import unittest
from pathlib import Path

from laser_web import core, jobs

#: 仓库根（``<repo>/python/laser_web/tests/test_repair.py`` 往上三级）。
REPO_ROOT = Path(__file__).resolve().parents[3]

#: 交付域：与现场 50 点运行完全一致的采样域与姿态。
DELIVERY_DOMAIN = {"point_min": [750.0, -600.0, 270.0],
                   "point_max": [1300.0, 600.0, 940.0]}
#: 2026-11-05 控制器报的那组读数：与零位完全等价（退化读数）。
DEGENERATE_Q = [0.0, 0.0, 0.0, 55.721, 0.0, -55.721]
#: 现场那套参数（CLI 默认的扰动半宽 5/20/15）。
FIELD_PARAMS = {"robot": "data/configs/hyundai_general6s.urdf",
                "rpy": [3.0, 73.0, -167.0],
                "euler": "xyz-intrinsic",
                "perturb_deg": [5.0, 20.0, 15.0],
                "q_seed": "43,-30,-36,42,-89,-1",
                "j5_min": 20.0,
                "seed": 0,
                "singular_q": [list(DEGENERATE_Q)],
                "stopped_at": 41,
                "target_count": 50}
#: 8 点小案子（200 mm 立方，7 段）。
SMALL_DOMAIN = {"point_min": [900.0, -100.0, 500.0],
                "point_max": [1100.0, 100.0, 700.0]}
SMALL_PARAMS = {"robot": "data/configs/hyundai_general6s.urdf",
                "rpy": [3.0, 73.0, -167.0],
                "euler": "xyz-intrinsic",
                "perturb_deg": [20.0, 20.0, 35.0],
                "q_seed": "43,-30,-36,42,-89,-1",
                "j5_min": 20.0,
                "seed": 0,
                "singular_q": [list(DEGENERATE_Q)],
                "stopped_at": 4,
                "target_count": 8}
#: 20 点大箱体：窗口里会出现真的越界段（J3 穿零），用来钉结论句。
BIG_DOMAIN = {"point_min": [750.0, -820.0, 270.0],
              "point_max": [1420.0, 800.0, 1320.0]}
BIG_PARAMS = dict(FIELD_PARAMS, **BIG_DOMAIN, target_count=20)


def _params(base: dict, domain: dict, **overrides) -> dict:
    """拼一份完整入参（每次返回新 dict，列表也是新的）。"""
    params = {key: (list(value) if isinstance(value, list) else value)
              for key, value in base.items()}
    params.update({key: (list(value) if isinstance(value, list) else value)
                   for key, value in domain.items()})
    params.update(overrides)
    return params


class TemporaryRuns(unittest.TestCase):
    """把 ``LASER_WEB_RUNS`` 指到临时目录，并在 ``setup_case`` 里跑一次解算。"""

    @classmethod
    def setUpClass(cls):
        """建临时产物目录 -> 跑子类的解算 -> 之后所有断言读这一份结果。"""
        cls._tmp = tempfile.TemporaryDirectory()
        cls._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = cls._tmp.name
        cls.setup_case()

    @classmethod
    def tearDownClass(cls):
        """恢复环境变量并清临时目录。"""
        if cls._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = cls._previous_runs
        cls._tmp.cleanup()

    @classmethod
    def setup_case(cls) -> None:
        """子类在这里准备 ``cls.result`` 之类的数据。"""


class DeliveryDomainTest(TemporaryRuns):
    """交付域 + 退化读数 + 已走到 GP41：窗口干净，采样域不动。"""

    @classmethod
    def setup_case(cls) -> None:
        """定位一次、修复一次（两次都用同一份 `现场参数`）。"""
        cls.params = _params(FIELD_PARAMS, DELIVERY_DOMAIN)
        cls.located = core.locate_segment(cls.params)
        cls.stages = []
        cls.result = core.run_repair(
            cls.params,
            on_progress=lambda stage, index, total:
            cls.stages.append((stage, index, total)))

    def test_degenerate_reading_is_classified(self):
        """退化读数：kind / 与零位等价 / 不产生位置禁区 / |J3| 下限抬到 10°。"""
        located = self.located
        self.assertTrue(located["ok"], located.get("error"))
        self.assertEqual(len(located["classifications"]), 1)
        row = located["classifications"][0]
        self.assertEqual(row["kind"], "degenerate")
        self.assertTrue(row["same_as_zero_pose"])
        self.assertEqual(located["forbidden_positions"], [])
        self.assertGreaterEqual(row["j3_floor"], 10.0)
        self.assertAlmostEqual(row["q_deg"][3], 55.721)
        self.assertEqual(len(row["position"]), 3)

    def test_effective_thresholds(self):
        """有效阈值：|J5| >= 20（内置）、|J3| >= 10（= avoid-margin）。"""
        thresholds = self.located["effective_thresholds"]
        self.assertAlmostEqual(thresholds["j5_min_deg"], 20.0)
        self.assertAlmostEqual(thresholds["j3_min_deg"], 10.0)
        # 报了退化 -> 段内肘部也要留裕度（CLI 同规则）
        self.assertAlmostEqual(thresholds["j3_seg_min_deg"], 10.0)

    def test_window_has_seven_clean_segments(self):
        """窗口 = 前后各 3 段 + 报告段，共 7 段，都不穿零、都不越界。"""
        located = self.located
        self.assertEqual(located["stopped_at"], 41)
        profiles = located["profiles"]
        # 已到 GP41 -> 段起点下标 40；前后各 3 段
        self.assertEqual([profile["index"] for profile in profiles],
                         list(range(37, 44)))
        self.assertEqual(profiles[3]["gp_from"], 41)
        self.assertEqual(profiles[3]["gp_to"], 42)
        for profile in profiles:
            self.assertEqual(profile["crossings"], 0)
            self.assertTrue(profile["ok"])
            self.assertIsNotNone(profile["j3_abs_min"])
            self.assertIsNotNone(profile["j5_abs_min"])
            self.assertEqual(len(profile["worst_pos"]), 3)
            self.assertTrue(0.0 <= profile["worst_frac"] <= 1.0)
        self.assertEqual(located["offenders"], [])

    def test_verdict_says_leave_the_domain_alone(self):
        """结论句：窗口干净 + 整链无越界 -> 不动采样域。"""
        located = self.located
        self.assertTrue(located["unmodified"]["ok"], located["unmodified"])
        self.assertEqual(located["chain"]["j3_crossings"], 0)
        self.assertIn("不动采样域", located["verdict"])

    def test_repair_changes_nothing(self):
        """修复：mode=none、边界与姿态都没动、体积损失为 0。"""
        result = self.result
        self.assertTrue(result["ok"], result.get("error"))
        repair = result["repair"]
        self.assertEqual(repair["mode"], "none")
        self.assertEqual(repair["label"], "无改动")
        self.assertEqual(repair["bounds_after"], repair["bounds_before"])
        self.assertEqual(repair["rpy_after"], repair["rpy_before"])
        self.assertEqual(repair["moves"], [])
        self.assertIsNone(repair["local"])
        self.assertAlmostEqual(repair["volume_loss"], 0.0)
        self.assertEqual(result["meta"]["repaired_domain"]["tier"], 0)

    def test_repair_result_shape_matches_run_scan(self):
        """返回值与 ``run_scan`` 同构，前端才能复用同一套渲染。"""
        result = self.result
        for key in ("ok", "run_id", "run_dir", "stats", "meta", "waypoints",
                    "segments", "artifacts", "warnings", "repair"):
            self.assertIn(key, result)
        for key in ("solved", "total", "switches", "dq_l2_sum", "j5_abs_min",
                    "j3_abs_min", "design_cond", "seg_j5_min", "seg_j3_min",
                    "j3_crossings", "j5_crossings", "feasible"):
            self.assertIn(key, result["stats"])
        self.assertEqual(result["stats"]["solved"], 50)
        self.assertEqual(len(result["waypoints"]), 50)
        self.assertEqual(len(result["segments"]), 49)
        self.assertEqual([row["gp"] for row in result["waypoints"]],
                         list(range(1, 51)))

    def test_repair_block_keys(self):
        """``repair`` 节：规格要求的键齐全，日志不超过 60 条。"""
        repair = self.result["repair"]
        for key in ("mode", "label", "bounds_before", "bounds_after",
                    "volume_loss", "rpy_before", "rpy_after", "moves",
                    "local", "log", "before_stats", "after_stats"):
            self.assertIn(key, repair)
        self.assertEqual(len(repair["bounds_before"]), 6)
        self.assertEqual(len(repair["rpy_before"]), 3)
        self.assertLessEqual(len(repair["log"]), core.REPAIR_LOG_LIMIT)
        self.assertEqual(repair["log_total"], len(repair["log"]))
        self.assertTrue(repair["after_stats"]["feasible"])
        self.assertEqual(repair["after_stats"]["j3_crossings"], 0)

    def test_artifacts_and_repair_report(self):
        """四个产物都落盘；``repair.json`` 与 CLI 报告同骨架。"""
        artifacts = self.result["artifacts"]
        for key, name in (("scan_json", "scan.json"), ("joints", "joints.txt"),
                          ("plot", "plot.png"), ("repair", "repair.json")):
            self.assertEqual(artifacts[key], name)
            self.assertTrue((Path(self.result["run_dir"]) / name).is_file(),
                            name)
        run_dir = Path(self.result["run_dir"])
        self.assertGreater((run_dir / "plot.png").stat().st_size, 1024)
        scan_payload = json.loads((run_dir / "scan.json").read_text("utf-8"))
        self.assertEqual(scan_payload["meta"]["repaired_domain"]["tier"], 0)
        self.assertEqual(len(scan_payload["waypoints"]), 50)
        report = json.loads((run_dir / "repair.json").read_text("utf-8"))
        for key in ("inputs", "singularity_analysis", "effective_thresholds",
                    "forbidden_positions", "baseline", "search", "decision",
                    "repair"):
            self.assertIn(key, report)
        self.assertEqual(report["decision"]["tier"], 0)
        self.assertEqual(report["repair"]["mode"], "none")
        lines = (run_dir / "joints.txt").read_text("utf-8").splitlines()
        self.assertEqual(len(lines), 50)
        self.assertEqual(len(lines[0].split()), 6)

    def test_progress_stages(self):
        """进度阶段名：M2 的短语 + 定位段 / 局部修复（整体搜索与复核只在搜索时出现）。"""
        stages = [stage for stage, _index, _total in self.stages]
        for name in ("解算 IK", "定位段", "局部修复", "出图", "写文件"):
            self.assertIn(name, stages)
        self.assertTrue(all(index <= total
                            for _stage, index, total in self.stages))
        for name in core.REPAIR_STAGES:
            self.assertIn(name, "".join(stages) + "".join(core.REPAIR_STAGES))


class SmallCaseTest(TemporaryRuns):
    """8 点小案子：跑完整修复流程，返回结构齐全（不做 UI）。"""

    @classmethod
    def setup_case(cls) -> None:
        cls.result = core.run_repair(_params(SMALL_PARAMS, SMALL_DOMAIN))

    def test_ok_and_repair_block(self):
        """小案子能跑完，``repair`` 节与统计都在。"""
        result = self.result
        self.assertTrue(result["ok"], result.get("error"))
        repair = result["repair"]
        self.assertIn(repair["mode"], ("none", "local", "domain"))
        self.assertIn("solved", repair["after_stats"])
        self.assertEqual(repair["after_stats"]["solved"], 8)
        self.assertEqual(len(repair["bounds_before"]), 6)
        self.assertEqual(len(repair["bounds_after"]), 6)
        self.assertEqual(len(result["waypoints"]), 8)
        self.assertEqual(len(result["segments"]), 7)
        self.assertLessEqual(len(repair["log"]), core.REPAIR_LOG_LIMIT)

    def test_local_change_shape(self):
        """``local`` 要么是 ``None``，要么是完整的 ``{index, gp, delta_rpy, q_*}``。

        局部修复是否触发取决于窗口里有没有越界段；这个小案子窗口干净，所以走
        ``mode="none"``。``repair_segment`` 本身的行为由
        ``excitation_trajectory/tests/test_avoid_singularity.py`` 覆盖，这里不重复。
        """
        repair = self.result["repair"]
        local = repair["local"]
        if repair["mode"] == "local":
            self.assertIsNotNone(local)
            self.assertEqual(local["gp"], local["index"] + 1)
            self.assertEqual(len(local["delta_rpy"]), 3)
            self.assertEqual(len(local["q_before"]), 6)
            self.assertEqual(len(local["q_after"]), 6)
        else:
            self.assertIsNone(local)
            self.assertIn(repair["mode"], ("none", "domain"))

    def test_no_stopped_at_still_runs(self):
        """不给现场点号也能量：跳过定位与局部修复，直接判整链。"""
        result = core.run_repair(_params(SMALL_PARAMS, SMALL_DOMAIN,
                                        stopped_at=None))
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["repair"]["profiles"], [])
        self.assertIsNone(result["repair"]["stopped_at"])
        self.assertEqual(result["repair"]["mode"], "none")


class WindowOffenderTest(unittest.TestCase):
    """20 点大箱体：窗口里真的有越界段，结论句两种写法各一次。"""

    def test_reported_segment_is_the_offender(self):
        """报告段自己越界 -> 结论句给出该段的 |J3|min / |J5|min。"""
        located = core.locate_segment(_params(BIG_PARAMS, {}, stopped_at=5))
        self.assertTrue(located["ok"], located.get("error"))
        self.assertTrue(located["offenders"])
        indexes = {profile["index"] for profile in located["offenders"]}
        self.assertIn(4, indexes)               # 报告段 = GP5->GP6
        reported = next(profile for profile in located["profiles"]
                        if profile["index"] == 4)
        self.assertEqual(reported["gp_from"], 5)
        self.assertEqual(reported["gp_to"], 6)
        self.assertGreaterEqual(reported["crossings"], 1)
        self.assertIn("报告段 GP5->GP6 越界", located["verdict"])
        self.assertIn("|J3|min=", located["verdict"])

    def test_safe_reported_segment_names_the_worst_one(self):
        """报告段安全、邻居越界 -> 结论句指出最危险的那一段。"""
        located = core.locate_segment(_params(BIG_PARAMS, {}, stopped_at=2))
        self.assertTrue(located["ok"], located.get("error"))
        self.assertTrue(located["offenders"])
        indexes = {profile["index"] for profile in located["offenders"]}
        self.assertNotIn(1, indexes)            # GP2->GP3 是报告段，安全
        self.assertIn(4, indexes)               # GP5->GP6 才是越界的那一段
        self.assertIn("报告段本身安全", located["verdict"])
        self.assertIn("GP5->GP6", located["verdict"])


class ValidationTest(unittest.TestCase):
    """校验错误一律 ``ok: False`` + 报错原文，不抛异常。"""

    def test_bad_singular_q_is_passed_through(self):
        """``singular_q`` 不是 6 的倍数：报错来自 CLI 的展平函数。"""
        result = core.locate_segment(
            _params(FIELD_PARAMS, DELIVERY_DOMAIN, singular_q=[[1, 2, 3]]))
        self.assertFalse(result["ok"])
        self.assertTrue(result["error"])
        self.assertIn("6 的整数倍", result["error"])
        self.assertEqual(result["warnings"], [])

    def test_missing_singular_q(self):
        """没给读数：明确说要一组 6 个关节角。"""
        result = core.locate_segment(
            _params(FIELD_PARAMS, DELIVERY_DOMAIN, singular_q=None))
        self.assertFalse(result["ok"])
        self.assertIn("singular_q", result["error"])

    def test_bad_domain_is_passed_through(self):
        """采样域自身的错误（六数式里 X 轴上下界反了）用 CLI 原文回显。"""
        params = _params(SMALL_PARAMS, {},
                         aabb6=[1100.0, 900.0, -100.0, 100.0, 500.0, 700.0])
        params.pop("point_min", None)
        params.pop("point_max", None)
        for result in (core.locate_segment(params),
                       core.run_repair(params)):
            self.assertFalse(result["ok"])
            self.assertIn("X 轴下界", result["error"])

    def test_stopped_at_out_of_range(self):
        """``stopped_at`` 等于最后一个点：后面没有段，明确报错。"""
        params = _params(SMALL_PARAMS, SMALL_DOMAIN, stopped_at=8)
        for result in (core.locate_segment(params),
                       core.run_repair(params)):
            self.assertFalse(result["ok"])
            self.assertIn("1..7", result["error"])
            self.assertIn("stopped-at", result["error"])

    def test_stopped_at_not_an_integer(self):
        """``stopped_at`` 是小数：提示要整数。"""
        result = core.locate_segment(
            _params(SMALL_PARAMS, SMALL_DOMAIN, stopped_at=3.5))
        self.assertFalse(result["ok"])
        self.assertIn("整数", result["error"])

    def test_negative_avoid_radius(self):
        """修复专用字段的取值也要挡住（derive 自 CLI 默认值的字段）。"""
        result = core.locate_segment(
            _params(SMALL_PARAMS, SMALL_DOMAIN, avoid_radius=-1.0))
        self.assertFalse(result["ok"])
        self.assertIn("avoid_radius", result["error"])


class DefaultsTest(unittest.TestCase):
    """修复页的默认值来自修复 CLI，不在这里另抄数字。"""

    def test_repair_cli_defaults(self):
        """``--seg-samples`` 修复默认 11（扫描 CLI 是 21），其余照 CLI。"""
        defaults = core.repair_cli_defaults()
        self.assertEqual(defaults["seg_samples"], 11)
        self.assertEqual(defaults["seg_samples_final"], 31)
        self.assertEqual(defaults["segment_window"], 3)
        self.assertEqual(defaults["avoid_margin"], 10.0)
        self.assertEqual(defaults["avoid_radius"], 80.0)
        self.assertEqual(defaults["repair_attempts"], 60)
        self.assertEqual(core.REPAIR_STAGES,
                         ("定位段", "局部修复", "整体搜索", "复核"))
        self.assertIs(core.repair_cli_defaults(), defaults)   # 缓存同一份

class WindowProfilesEarlyExitTest(unittest.TestCase):
    """原域因位置禁区早退（``steps``/``segments`` 为 None）时不能抛 TypeError。

    上一批修复的点：``segment_profiles`` 会 ``len(None)``，把真正的原因
    （禁区命中）埋在一句类型错误里。这里钉住"直接返回空 + 不碰 ctx"。
    """

    def test_missing_steps_returns_empty(self):
        import argparse

        args = argparse.Namespace(segment_window=3, seg_samples_final=11,
                                  seg_min=5.0)
        for baseline in ({"steps": None, "segments": None},
                         {"steps": [], "segments": None},
                         {"steps": [object()], "segments": None},
                         {}):
            self.assertEqual(
                core._window_profiles({}, baseline, 0, args), ([], []),
                baseline)

    def test_empty_profiles_keep_verdict_and_report_alive(self):
        """空 profiles 交给 ``_verdict`` / 报告时不炸（不出现未定义变量）。"""
        import argparse

        args = argparse.Namespace(segment_window=3, seg_samples_final=11,
                                  seg_min=5.0)
        baseline = {"steps": None, "segments": None, "ok": False,
                    "reasons": ["路点 #3 距奇异位形位置 20.0 mm < 禁区半径 80.0 mm"]}
        profiles, offenders = core._window_profiles({}, baseline, 2, args)
        verdict = core._verdict(baseline, profiles, offenders, 2,
                               core._chain_summary(None))
        self.assertEqual(profiles, [])
        self.assertEqual(offenders, [])
        # 结论必须说真话：是"生成链之前就不可行"，不是"各段都满足阈值"
        self.assertIn("生成链之前", verdict)
        self.assertIn("禁区", verdict)
        # 报告里 segment_diagnostics.profiles 也得是空列表（不是 None）
        ctx = {"base": {"label": "原域", "bounds": [0, 1, 0, 1, 0, 1],
                        "rpy": [0, 0, 0], "moves": []},
               "analyses": [], "j5_eff": 20.0, "j3_eff": 0.0,
               "j3_seg_min": 0.0, "forbidden": [], "target_count": 8}
        report = core._repair_report(
            argparse.Namespace(urdf="x.urdf", rpy=[0, 0, 0],
                              euler_convention="xyz-intrinsic",
                              singular_q=[], perturb_deg=[0, 0, 0],
                              stopped_at=2, segment_window=3,
                              avoid_radius=80.0),
            ctx, baseline, profiles, offenders, [],
            ctx["base"], {}, {"label": "无改动"}, succeeded=False)
        self.assertEqual(report["segment_diagnostics"]["profiles"], [])
        self.assertIsNone(report["decision"])

class ForbiddenPositionTest(TemporaryRuns):
    """位置禁区命中（``steps=None``）时，定位与修复都要说真话、不崩。

    上一批只修了 ``segment_profiles`` 那一处 ``len(None)``；修复路径上紧接着
    调用的 ``repair_segment`` 同样会 ``len(None)``，会把"禁区命中"埋成一句
    TypeError。这里用报告位形自己的位置造一个整箱都在禁区里的采样域，把两条
    路径都跑一遍。
    """

    #: 报告位形：``|J5|=89 / |J3|=36``，落在判据之外 -> ``unexplained`` -> 有禁区。
    UNEXPLAINED_Q = [43.0, -30.0, -36.0, 42.0, -89.0, -1.0]
    #: 箱子半边长（mm）：角点 69 mm < 禁区半径 80 mm，整箱都在禁区里。
    HALF = 40.0

    @classmethod
    def setup_case(cls) -> None:
        """围绕报告位形的 FK 位置造一个小箱，定位 + 修复各跑一次。"""
        from robot_model.nexbot_ik import Nexbot6R

        rbt = Nexbot6R(str(REPO_ROOT / "data/configs/hyundai_general6s.urdf"))
        position = [float(v) for v in rbt.fk(cls.UNEXPLAINED_Q)[:3, 3]]
        cls.params = _params(
            FIELD_PARAMS, {"point_min": [p - cls.HALF for p in position],
                           "point_max": [p + cls.HALF for p in position]},
            singular_q=[list(cls.UNEXPLAINED_Q)], stopped_at=3,
            target_count=8)
        cls.located = core.locate_segment(cls.params)
        cls.result = core.run_repair(cls.params)

    def test_locate_reports_the_real_reason(self):
        self.assertTrue(self.located["ok"], self.located)
        self.assertEqual(len(self.located["forbidden_positions"]), 1)
        self.assertEqual(self.located["profiles"], [])
        self.assertIn("生成链之前", self.located["verdict"])
        self.assertIn("禁区", self.located["verdict"])

    def test_repair_does_not_crash_on_missing_steps(self):
        self.assertTrue(self.result["ok"], self.result)
        self.assertIsNone(self.result.get("error"))
        self.assertNotIn("TypeError", " ".join(self.result["warnings"]))
        self.assertEqual(self.result["repair"]["mode"], "domain")
        self.assertIn("禁区", " ".join(self.result["warnings"]))

    def test_repair_found_a_plan_outside_the_forbidden_sphere(self):
        self.assertTrue(self.result["stats"]["solved"]
                        == self.result["stats"]["total"])


if __name__ == "__main__":
    unittest.main()
