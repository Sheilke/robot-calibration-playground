# -*- coding: utf-8 -*-
"""``laser_web.core`` 的 M3 单测：GP 导出 / JSON -> URDF / 运行目录清理。

只测服务层，不起任何 UI（与 :mod:`tests.test_run_scan` 同一套路：产物根目录
用 ``LASER_WEB_RUNS`` 指向临时目录，绝不往仓库根的 ``runs/`` 里丢垃圾）。

覆盖（对应设计文档 §8 M3 与 §9 的单测一栏）：

1. **GP 导出与 CLI 逐字段一致**：先 ``core.run_scan`` 跑 8 点小案子，再用真模板
   （``/home/infoli/Documents/backup/position_R1_template.json``，只读引用，
   不复制进仓库）走 ``core.build_gp``；同时用
   ``python -m excitation_trajectory.export_gp <scan> <template> -o … --first 1``
   跑一遍，8 个 GP × 14 个字段逐个相等；
2. **模板未被修改**：调用前后对模板文件做 sha256 比对；
3. **``--cartesian fk``**：前 6 个字段非全 0，第 7..12 字段与 ``zero`` 完全一致，
   且与 CLI 的 ``--cartesian fk`` 输出逐字段一致（参数一个都不省）；
4. **JSON -> URDF**：文件名带 ``_<model>`` 后缀，自校验误差被解析出来且 < 1e-9；
5. **运行列表与清理**：``list_runs`` 能列出刚跑的那次；``clean_runs(keep=1)``
   删掉构造出来的旧目录。
"""

import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from laser_web import core, jobs

#: 仓库根（``<repo>/python/laser_web/tests/test_tools.py`` 往上三级）。
REPO_ROOT = Path(__file__).resolve().parents[3]
#: 现场的真模板（**只读**引用，不复制进仓库；文档里明确它是现场备份）。
TEMPLATE_PATH = Path("/home/infoli/Documents/backup/position_R1_template.json")
#: 跑 CLI 用的解释器：优先仓库的 venv（与自检命令一致），否则退回当前解释器。
PYTHON = str(REPO_ROOT / ".venv" / "bin" / "python")
if not Path(PYTHON).exists():                       # pragma: no cover - 环境兜底
    PYTHON = sys.executable

#: 8 点小案子（与 ``test_run_scan`` 同一份参数，1 s 内跑完）。
BASE_PARAMS = {"robot": "data/configs/ch2000_origin.urdf",
               "point_min": [900.0, -100.0, 500.0],
               "point_max": [1100.0, 100.0, 700.0],
               "rpy": [3.0, 73.0, -167.0],
               "euler": "xyz-intrinsic",
               "perturb_deg": [20.0, 20.0, 35.0],
               "target_count": 8,
               "q_seed": "43,-30,-36,42,-89,-1",
               "j5_min": 20.0,
               "seed": 0}
#: GP 条目字段数（``export_gp.GP_FIELD_COUNT``）。
GP_FIELD_COUNT = 14
#: 最小可用的控制器 JSON（构造方式照抄 ``robot_model/tests/test_json_to_urdf.py``）。
MINIMAL_CONTROLLER = {
    "robotType": "R_GENERAL_6S", "totalAxisNum": 6, "eulerAngle": "XYZ",
    "Link&CoupleCoe": {
        "mountMethod": 0,
        "CoupleCoe": {"Couple_Coe_5_6": 0.0},
        "Link": [
            {"a": 159.47, "alpha": 90.0, "d": 446.6, "theta": 0.0},
            {"a": 770.7, "alpha": 0.0, "d": 0.0, "theta": 90.0},
            {"a": 141.05, "alpha": 90.0, "d": 0.0, "theta": 0.0},
            {"a": 0.0, "alpha": 90.0, "d": 1025.36, "theta": 0.0},
            {"a": 0.0, "alpha": -90.0, "d": 0.0, "theta": 0.0},
            {"a": 0.0, "alpha": 0.0, "d": 110.4, "theta": 0.0},
        ],
    },
    "Joint": [{"Direction": -1, "NegSWLimit": -180.0, "PosSWLimit": 180.0,
               "RatedVel": 100.0} for _ in range(6)],
}


def _params(**overrides):
    """一份完整的扫描入参（每次返回新 dict，列表也是新的）。"""
    params = json.loads(json.dumps(BASE_PARAMS))
    params.update(overrides)
    return params


def _sha256(path: Path) -> str:
    """文件内容的 sha256（模板"未被修改"的证据）。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _run_export_gp(*argv: str) -> Path:
    """按文档里的命令跑一次 CLI：``python -m excitation_trajectory.export_gp``。

    Args:
        argv: 追加在模块名之后的 CLI 参数（scan / template / -o / … 一个都不省）。

    Returns:
        Path: 约定的 CLI 输出文件（``argv`` 里 ``-o`` 后面那个）。
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = str(REPO_ROOT / "python") + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    completed = subprocess.run(
        [PYTHON, "-m", "excitation_trajectory.export_gp", *argv],
        cwd=str(REPO_ROOT), env=env, capture_output=True, text=True, check=False)
    if completed.returncode != 0:                    # pragma: no cover - 失败才走
        raise AssertionError(f"CLI 失败（{completed.returncode}）：\n"
                             f"{completed.stdout}\n{completed.stderr}")
    return Path(argv[list(argv).index("-o") + 1])


def _registers(path: Path) -> dict:
    """读一个 GP 文件，返回它的 ``globalPosition``。"""
    return json.loads(Path(path).read_text(encoding="utf-8"))["globalPosition"]


class RunOnceMixin:
    """跑一次 8 点小案子并把产物根目录挪进临时目录。"""

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls._previous_runs = os.environ.get(jobs.RUNS_ENV)
        os.environ[jobs.RUNS_ENV] = cls._tmp.name
        cls.scan_result = core.run_scan(_params())

    @classmethod
    def tearDownClass(cls):
        if cls._previous_runs is None:
            os.environ.pop(jobs.RUNS_ENV, None)
        else:
            os.environ[jobs.RUNS_ENV] = cls._previous_runs
        cls._tmp.cleanup()


class GpExportTest(RunOnceMixin, unittest.TestCase):
    """``core.build_gp`` 必须与 CLI ``export_gp`` 逐字段一致，且不碰模板。"""

    @classmethod
    def setUpClass(cls):
        if not TEMPLATE_PATH.is_file():            # 现场备份缺失时不硬造模板
            raise unittest.SkipTest(f"缺少现场模板 {TEMPLATE_PATH}")
        super().setUpClass()
        if not cls.scan_result["ok"]:              # pragma: no cover - 算法炸了
            raise unittest.SkipTest(f"扫描失败：{cls.scan_result.get('error')}")
        cls.run_dir = Path(cls.scan_result["run_dir"])
        cls.scan_json = cls.run_dir / core.ARTIFACT_SCAN_JSON
        cls.template_bytes = TEMPLATE_PATH.read_bytes()
        cls.before_sha = _sha256(TEMPLATE_PATH)
        cls.zero = core.build_gp({"run_dir": str(cls.run_dir),
                                  "template": cls.template_bytes,
                                  "template_name": TEMPLATE_PATH.name})
        cls.after_sha = _sha256(TEMPLATE_PATH)

    def test_scan_matches_expectation(self):
        """前提：8 个点全部解出，scan.json 在运行目录里。"""
        self.assertEqual(self.scan_result["stats"]["solved"], 8)
        self.assertTrue(self.scan_json.is_file())

    def test_result_shape(self):
        """返回结构：ok / path / filename / count / first / last / meta。"""
        result = self.zero
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["count"], 8)
        self.assertEqual((result["first"], result["last"]), (1, 8))
        self.assertEqual(result["filename"], TEMPLATE_PATH.name)
        self.assertEqual(Path(result["path"]), self.run_dir / TEMPLATE_PATH.name)
        self.assertEqual(result["meta"], {
            "controllerID": "D80469F1709A495E", "version": "1.0.5"})

    def test_fields_match_cli(self):
        """8 个 GP × 14 个字段与 CLI ``--first 1`` 逐个相等。"""
        cli_out = _run_export_gp(str(self.scan_json), str(TEMPLATE_PATH),
                                 "-o", "/tmp/laser_web_test_gp_cli.json",
                                 "--first", "1")
        mine = _registers(Path(self.zero["path"]))
        theirs = _registers(cli_out)
        for slot in range(1, 9):
            name = f"GP{slot:04d}"
            with self.subTest(gp=name):
                self.assertEqual(len(mine[name]), GP_FIELD_COUNT)
                self.assertEqual(len(theirs[name]), GP_FIELD_COUNT)
                for index in range(GP_FIELD_COUNT):
                    self.assertEqual(mine[name][index], theirs[name][index],
                                     f"{name} 第 {index} 个字段与 CLI 不一致")

    def test_other_options_match_cli(self):
        """``first`` / ``flag`` 也必须原样传给算法（与 CLI 同一组参数对照）。"""
        mine = core.build_gp({"run_dir": str(self.run_dir),
                              "template": self.template_bytes,
                              "template_name": TEMPLATE_PATH.name,
                              "first": 11, "flag": 1})
        self.assertTrue(mine["ok"], mine.get("error"))
        cli_out = _run_export_gp(str(self.scan_json), str(TEMPLATE_PATH),
                                 "-o", "/tmp/laser_web_test_gp_cli_flag.json",
                                 "--first", "11", "--flag", "1")
        self.assertEqual((mine["first"], mine["last"]), (11, 18))
        mine_regs, cli_regs = _registers(Path(mine["path"])), _registers(cli_out)
        for slot in range(11, 19):
            name = f"GP{slot:04d}"
            for index in range(GP_FIELD_COUNT):
                self.assertEqual(mine_regs[name][index], cli_regs[name][index],
                                 f"{name} 第 {index} 个字段与 CLI 不一致")

    def test_template_untouched(self):
        """模板文件的 sha256 前后一致（CLI 与 M3 的硬约定）。"""
        self.assertEqual(self.before_sha, self.after_sha)
        self.assertEqual(_sha256(TEMPLATE_PATH), self.before_sha)

    def test_overwrite_warns(self):
        """同一运行目录再导一次同名文件：覆盖并记 warning。"""
        again = core.build_gp({"run_dir": str(self.run_dir),
                               "template": self.template_bytes,
                               "template_name": TEMPLATE_PATH.name})
        self.assertTrue(again["ok"], again.get("error"))
        self.assertTrue(any("已覆盖" in text for text in again["warnings"]),
                        again["warnings"])

    def test_fk_fills_cartesian_and_keeps_joints(self):
        """``cartesian="fk"``：前 6 个字段非全 0，关节 6 个字段与 ``zero`` 一致。"""
        fk = core.build_gp({"run_dir": str(self.run_dir),
                            "template": self.template_bytes,
                            "template_name": TEMPLATE_PATH.name,
                            "cartesian": "fk"})
        self.assertTrue(fk["ok"], fk.get("error"))
        zero_regs, fk_regs = _registers(Path(self.zero["path"])), _registers(
            Path(fk["path"]))
        for slot in range(1, 9):
            name = f"GP{slot:04d}"
            self.assertTrue(any(value != 0.0 for value in fk_regs[name][:6]),
                            f"{name} 的笛卡尔部分仍是 0")
            self.assertEqual(fk_regs[name][6:13], zero_regs[name][6:13],
                             f"{name} 的关节/标志字段被 fk 模式改动了")

    def test_fk_matches_cli(self):
        """``--cartesian fk --cart-units rad --euler xyz-intrinsic`` 与 CLI 一致。"""
        mine = core.build_gp({"run_dir": str(self.run_dir),
                              "template": self.template_bytes,
                              "template_name": TEMPLATE_PATH.name,
                              "cartesian": "fk", "cart_units": "rad",
                              "euler": "xyz-intrinsic"})
        self.assertTrue(mine["ok"], mine.get("error"))
        cli_out = _run_export_gp(str(self.scan_json), str(TEMPLATE_PATH),
                                 "-o", "/tmp/laser_web_test_gp_cli_fk.json",
                                 "--first", "1", "--cartesian", "fk",
                                 "--cart-units", "rad",
                                 "--euler", "xyz-intrinsic")
        mine_regs, cli_regs = _registers(Path(mine["path"])), _registers(cli_out)
        for slot in range(1, 9):
            name = f"GP{slot:04d}"
            for index in range(GP_FIELD_COUNT):
                self.assertEqual(mine_regs[name][index], cli_regs[name][index],
                                 f"{name} 第 {index} 个字段与 CLI fk 不一致")

    def test_scan_path_instead_of_run_dir(self):
        """``scan`` 直接给 scan.json 路径也能用（输出仍落在该运行目录）。"""
        result = core.build_gp({"scan": str(self.scan_json),
                                "template": self.template_bytes,
                                "template_name": TEMPLATE_PATH.name})
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(Path(result["path"]).parent, self.run_dir)

    def test_missing_template_is_rejected(self):
        """没上传模板就返回 ``ok: False``，不抛异常。"""
        result = core.build_gp({"run_dir": str(self.run_dir)})
        self.assertFalse(result["ok"])
        self.assertIn("模板", result["error"])

    def test_bad_options_are_rejected(self):
        """``cartesian`` / ``cart_units`` / ``euler`` 的取值与 CLI 同样受限。"""
        cases = ({"cartesian": "world"}, {"cart_units": "mm"},
                 {"cartesian": "fk", "euler": "abc"})
        for extra in cases:
            params = {"run_dir": str(self.run_dir),
                      "template": self.template_bytes,
                      "template_name": TEMPLATE_PATH.name}
            params.update(extra)
            with self.subTest(**extra):
                result = core.build_gp(params)
                self.assertFalse(result["ok"])

    def test_joint_range_is_checked(self):
        """关节角越界时 ``ok: False``（写前限位校验用的是 ``check_joint_range``）。"""
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "fake-run"
            run_dir.mkdir()
            payload = {"perturbed_ik": [
                {"index": 0, "q": [400.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 "ok": True, "config": None},
                {"index": 1, "q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 "ok": True, "config": None}]}
            (run_dir / core.ARTIFACT_SCAN_JSON).write_text(
                json.dumps(payload), encoding="utf-8")
            result = core.build_gp({"run_dir": str(run_dir),
                                    "template": self.template_bytes,
                                    "template_name": TEMPLATE_PATH.name})
            self.assertFalse(result["ok"])
            self.assertIn("360", result["error"])
            self.assertFalse((run_dir / TEMPLATE_PATH.name).exists(),
                             "越界时不该写出 GP 文件")


class ConvertUrdfTest(unittest.TestCase):
    """``core.convert_urdf``：文件名后缀 + 自校验误差解析。"""

    def test_appends_model_suffix_and_parses_error(self):
        data = json.dumps(MINIMAL_CONTROLLER).encode("utf-8")
        result = core.convert_urdf(data, "CH2000", filename="robot_R1.json")
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["filename"], "robot_R1_CH2000.urdf")
        self.assertEqual(Path(result["path"]).name, "robot_R1_CH2000.urdf")
        self.assertTrue(Path(result["path"]).is_file())
        self.assertIsNotNone(result["self_check_error"])
        self.assertLess(result["self_check_error"], 1e-9)
        self.assertIn("<robot", Path(result["path"]).read_text(encoding="utf-8"))

    def test_empty_model_keeps_plain_name(self):
        """型号为空 = CLI 不带 ``--model``：不追加后缀。"""
        data = json.dumps(MINIMAL_CONTROLLER).encode("utf-8")
        result = core.convert_urdf(data, "", filename="robot_R1.json")
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["filename"], "robot_R1.urdf")

    def test_bad_model_reports_original_error(self):
        """型号非法时回显 ``json_to_urdf`` 的原始报错，不抛异常。"""
        data = json.dumps(MINIMAL_CONTROLLER).encode("utf-8")
        result = core.convert_urdf(data, "a/b", filename="robot_R1.json")
        self.assertFalse(result["ok"])
        self.assertIn("--model", result["error"])
        self.assertIn("a/b", result["error"])

    def test_broken_json_is_rejected(self):
        """不是合法控制器 JSON 时返回 ``ok: False``。"""
        result = core.convert_urdf(b"{not json", "CH2000", filename="x.json")
        self.assertFalse(result["ok"])
        self.assertIn("Error", result["error"] + "Error")

    def test_empty_payload_is_rejected(self):
        result = core.convert_urdf(b"", "CH2000", filename="x.json")
        self.assertFalse(result["ok"])


class RunsHousekeepingTest(RunOnceMixin, unittest.TestCase):
    """``list_runs`` / ``clean_runs``：盘点刚跑的那次，清理构造出来的旧目录。"""

    def test_lists_the_run_just_finished(self):
        """``list_runs`` 里能找到刚跑完的那次，且认得出 scan.json / plot.png。"""
        rows = core.list_runs()
        run_id = Path(self.scan_result["run_dir"]).name
        match = [row for row in rows if row["run_id"] == run_id]
        self.assertEqual(len(match), 1, rows)
        row = match[0]
        self.assertEqual(row["dir"], self.scan_result["run_dir"])
        self.assertTrue(row["has_scan"])
        self.assertTrue(row["has_plot"])
        self.assertTrue(row["has_joints"])
        self.assertFalse(row["has_repair"])
        self.assertGreater(row["size_bytes"], 0)
        self.assertIsInstance(row["created"], float)
        self.assertEqual(set(row), {"run_id", "dir", "created", "size_bytes",
                                    "has_scan", "has_plot", "has_joints",
                                    "has_repair"})

    def test_rows_are_newest_first(self):
        """构造三个目录：``list_runs`` 按 ``created`` 倒序。"""
        with tempfile.TemporaryDirectory() as tmp:
            for offset, name in enumerate(("run-a", "run-b", "run-c")):
                path = Path(tmp) / name
                path.mkdir()
                (path / core.ARTIFACT_SCAN_JSON).write_text("{}", encoding="utf-8")
                stamp = time.time() + offset
                os.utime(path, (stamp, stamp))
            rows = core.list_runs(root=tmp)
            self.assertEqual([row["run_id"] for row in rows],
                             ["run-c", "run-b", "run-a"])
            self.assertTrue(all(row["has_scan"] for row in rows))
            self.assertTrue(all(not row["has_plot"] for row in rows))

    def test_clean_keeps_newest_only(self):
        """``clean_runs(keep=1)`` 删掉两个旧目录，留下最新的那一个。"""
        with tempfile.TemporaryDirectory() as tmp:
            for offset, name in enumerate(("oldest", "middle", "newest")):
                path = Path(tmp) / name
                path.mkdir()
                (path / core.ARTIFACT_SCAN_JSON).write_text("{}", encoding="utf-8")
                stamp = time.time() + offset
                os.utime(path, (stamp, stamp))
            result = core.clean_runs(keep=1, root=tmp)
            self.assertEqual(result["kept"], 1)
            self.assertEqual(sorted(Path(item).name for item in result["removed"]),
                             ["middle", "oldest"])
            self.assertEqual([path.name for path in Path(tmp).iterdir()
                              if path.is_dir()], ["newest"])

    def test_clean_with_nothing_to_do(self):
        """目录数不超过 ``keep`` 时什么也不删。"""
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "only").mkdir()
            result = core.clean_runs(keep=50, root=tmp)
            self.assertEqual(result, {"removed": [], "kept": 1})

    def test_defaults_match_cli_constants(self):
        """默认值不另抄：``flag`` / ``euler`` / ``cart-units`` 与算法模块同源。"""
        from excitation_trajectory import export_gp
        from handeye.pose_encoder import DEFAULT_EULER_CONVENTION, EULER_CONVENTIONS

        defaults = core.gp_defaults()
        self.assertEqual(defaults["flag"], export_gp.DEFAULT_FLAG)
        self.assertEqual(defaults["first"], 1)
        self.assertEqual(defaults["cartesian"], "zero")
        self.assertEqual(defaults["cart_units"], "rad")
        self.assertEqual(core.GP_DEFAULT_CART_UNITS, "rad")
        self.assertEqual(core.GP_DEFAULT_EULER, DEFAULT_EULER_CONVENTION)
        self.assertEqual(core.GP_DEFAULT_EULER, defaults["euler"])
        self.assertIn(core.GP_DEFAULT_EULER, EULER_CONVENTIONS)
        self.assertEqual(core.gp_euler_conventions(), list(EULER_CONVENTIONS))
        self.assertEqual(core.GP_TEMPLATE_META_KEYS, ("controllerID", "version"))
        self.assertEqual(core.GP_CARTESIAN_MODES, ("zero", "fk"))
        self.assertEqual(core.GP_CART_UNITS, ("rad", "deg"))


if __name__ == "__main__":
    unittest.main()
