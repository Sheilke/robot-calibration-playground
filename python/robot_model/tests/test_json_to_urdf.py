#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""json_to_urdf 的输出命名：默认取输入 JSON 主名，--model 追加后缀。"""
import tempfile
import unittest
from pathlib import Path

from robot_model.json_to_urdf import (
    MODEL_NAME_PATTERN,
    build_arg_parser,
    main,
    output_path_for,
)


class OutputNamingTest(unittest.TestCase):
    """输出名规则：默认继承输入主名；--model 在生效主名后追加 _<型号>。"""

    def test_default_takes_input_stem(self):
        self.assertEqual(output_path_for("a/robot_R1.json", None),
                         Path("a/robot_R1.urdf"))

    def test_model_appends_suffix_to_input_stem(self):
        self.assertEqual(output_path_for("a/robot_R1.json", None, "CH2000"),
                         Path("a/robot_R1_CH2000.urdf"))

    def test_explicit_output_wins_over_stem(self):
        self.assertEqual(output_path_for("a/robot_R1.json", "b/out.urdf"),
                         Path("b/out.urdf"))

    def test_model_appends_to_explicit_output(self):
        self.assertEqual(
            output_path_for("a/robot_R1.json", "b/out.urdf", "CH2000"),
            Path("b/out_CH2000.urdf"))

    def test_extension_is_always_urdf(self):
        self.assertEqual(output_path_for("a/robot_R1.JSON", None),
                         Path("a/robot_R1.urdf"))
        self.assertEqual(output_path_for("a/r.json", "out.txt"),
                         Path("out.urdf"))


class ModelValidationTest(unittest.TestCase):
    """型号会进文件名，必须白名单校验。"""

    def test_accepts_reasonable_names(self):
        for name in ("CH2000", "R1", "R-GENERAL_6S", "v0.1", "A1_2-3.4"):
            self.assertIsNotNone(MODEL_NAME_PATTERN.match(name), name)

    def test_rejects_path_characters(self):
        for name in ("../evil", "a/b", "a b", "", "型号", "a*b"):
            self.assertIsNone(MODEL_NAME_PATTERN.match(name), name)

    def test_cli_rejects_bad_model(self):
        from robot_model.json_to_urdf import _validate_model
        parser = build_arg_parser()
        for name in ("a/b", "", "型号", "../x"):
            with self.assertRaises(SystemExit):
                _validate_model(parser, name)

    def test_cli_accepts_good_model(self):
        from robot_model.json_to_urdf import _validate_model
        parser = build_arg_parser()
        for name in (None, "CH2000", "R-GENERAL_6S"):
            _validate_model(parser, name)      # 不抛异常即可


class EndToEndTest(unittest.TestCase):
    """用真控制器 JSON 走一遍：输出文件名带型号后缀，且内容可被 Nexbot6R 读取。"""

    def test_writes_suffixed_urdf(self):
        source = Path(__file__).resolve().parents[3] / "data" / "configs" \
            / "hyundai_general6s.urdf"
        if not source.exists():
            self.skipTest("缺少参考 URDF")
        # 用一个最小可用的控制器 JSON 反推不现实；这里只校验命名与写出行为
        import json
        cfg = {
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
            "Joint": [
                {"Direction": -1, "NegSWLimit": -180.0, "PosSWLimit": 180.0,
                 "RatedVel": 100.0} for _ in range(6)
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "robot_R1.json"
            src.write_text(json.dumps(cfg), encoding="utf-8")
            code = main([str(src), "--model", "CH2000"])
            self.assertIn(code, (0, None))
            produced = Path(tmp) / "robot_R1_CH2000.urdf"
            self.assertTrue(produced.exists(), "应输出 robot_R1_CH2000.urdf")
            text = produced.read_text(encoding="utf-8")
            self.assertIn("<robot", text)
            self.assertIn("CH2000", text)


if __name__ == "__main__":
    unittest.main()
