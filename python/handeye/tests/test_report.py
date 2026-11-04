# -*- coding: utf-8 -*-
"""handeye.report 单元测试：小节顺序、可复现、无绝对路径/时间戳、数值格式、
偏置诊断三档判定、FK 自检、备注与手眼矩阵节。"""

import re
import unittest
from pathlib import Path

import numpy as np

from handeye.report import (
    REPORT_FORMAT_VERSION,
    bias_diagnosis,
    render_markdown,
    write_markdown,
)
from handeye.se3 import from_rpy, inverse, to_matrix16
from handeye.session import Sample, Session, check_fk_consistency, load_session
from handeye.vpe import compute_vpe

ROOT = Path(__file__).resolve().parents[3]
SESSION_EXAMPLE = ROOT / "data" / "configs" / "session_example.json"

#: 报告必须包含的 9 个标题（H1 + 8 个固定编号小节），顺序即渲染顺序。
HEADINGS = [
    "# 手眼验证（VPE）报告",
    "## 1. 会话概要",
    "## 2. VPE 指标总览",
    "## 3. 逐轴误差",
    "## 4. 逐位姿明细",
    "## 5. 偏置方向诊断",
    "## 6. 手眼矩阵 X（flange←camera）",
    "## 7. 运动学一致性自检",
    "## 8. 复现方式",
]


class _StubKinematics:
    """按 ``q_deg[0]`` 返回预设 ``T_bh`` 的假运动学（不依赖 URDF）。"""

    def __init__(self, matrices):
        self.matrices = [np.asarray(matrix, dtype=float) for matrix in matrices]
        self.limits = None

    def fk(self, q_deg):
        return self.matrices[int(round(q_deg[0]))]


def _perfect_session():
    """构造「视觉链恰好命中参考点」的无噪声会话（bias 应为 0）。"""
    matrix = from_rpy((50.0, -30.0, 100.0), (5.0, -8.0, 120.0))
    reference = np.array([800.0, 200.0, 650.0])
    tbhs = [
        from_rpy((900.0, 100.0, 1400.0), (5.0, -20.0, 40.0)),
        from_rpy((1000.0, -150.0, 1500.0), (-15.0, 10.0, -70.0)),
        from_rpy((700.0, 300.0, 1600.0), (30.0, 25.0, 120.0)),
    ]
    samples = []
    for index, t_bh in enumerate(tbhs):
        chain = t_bh @ matrix
        p_cam = inverse(chain)[:3, :3] @ reference + inverse(chain)[:3, 3]
        samples.append(Sample(
            index,
            [float(index), 0.0, 0.0, 0.0, 0.0, 0.0],
            [{"target_points": {"7": [float(value) for value in p_cam]}}],
            recorded_pose=t_bh,
        ))
    return Session(
        meta={
            "robot_urdf": "data/configs/ch2000_origin.urdf",
            "dh_override": None,
            "group": 4,
            "joint_convention": "model",
            "units": "mm",
        },
        reference_point=reference,
        delta_p_base=None,
        handeye_matrix=matrix,
        samples=samples,
        kinematics=_StubKinematics(tbhs),
    )


def _vpe_result(bias, rmse):
    """只含偏置诊断所需字段的最小 ``vpe_result``。"""
    bias = np.asarray(bias, dtype=float)
    return {
        "metrics": {
            "vpe_rmse_mm": float(rmse),
            "bias": bias,
            "bias_norm_mm": float(np.linalg.norm(bias)),
        },
    }


def _example():
    """示例会话 + 计算好的 VPE 结果。"""
    session = load_session(SESSION_EXAMPLE)
    return session, compute_vpe(session)


class SectionStructureTest(unittest.TestCase):
    """小节标题必须固定、齐全、按序。"""

    def test_headings_exist_in_order(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result)
        positions = [text.index(heading) for heading in HEADINGS]
        self.assertEqual(positions, sorted(positions))
        self.assertEqual(len(set(positions)), len(HEADINGS))
        self.assertEqual(REPORT_FORMAT_VERSION, 1)

    def test_render_is_byte_identical(self):
        session, vpe_result = _example()
        first = render_markdown(session, vpe_result)
        second = render_markdown(session, vpe_result)
        self.assertEqual(first, second)
        self.assertEqual(first.encode("utf-8"), second.encode("utf-8"))

    def test_no_timestamp_or_absolute_path(self):
        session, vpe_result = _example()
        text = render_markdown(
            session, vpe_result, fk_consistency=check_fk_consistency(session))
        self.assertNotIn("/home/", text)
        self.assertNotIn(str(ROOT), text)
        self.assertIsNone(re.search(r"20\d\d-", text))
        self.assertIsNone(re.search(r"\d{4}-\d{2}-\d{2}", text))

    def test_session_overview_rows(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result)
        self.assertIn("| 组别 | 4 |", text)
        self.assertIn("| DH 覆盖 | 基准 |", text)
        self.assertIn("| 关节约定 | model |", text)
        self.assertIn("| 坐标单位 | mm |", text)
        self.assertIn("| 样本数 | 3 |", text)
        self.assertIn("| 总帧数 | 6 |", text)
        self.assertIn("| 靶球 id 列表 | 7 |", text)
        self.assertIn("(800.0000, 200.0000, 650.0000) mm", text)


class NumberFormatTest(unittest.TestCase):
    """数值统一 4 位小数，且不出现 -0.0000。"""

    def test_rmse_has_four_decimals(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result)
        rmse = vpe_result["metrics"]["vpe_rmse_mm"]
        line = next(line for line in text.splitlines() if "VPE_RMSE" in line)
        self.assertRegex(line, r"\d+\.\d{4}")
        self.assertIn(f"{rmse:.4f}", line)

    def test_all_metric_rows_have_four_decimals(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result)
        for label in ("VPE_MAX", "VPE_P95", "VPE_MEAN", "‖bias‖",
                      "视觉帧间重复性 (spread)"):
            line = next(line for line in text.splitlines() if label in line)
            self.assertRegex(line, r"\d+\.\d{4}")

    def test_negative_zero_never_rendered(self):
        session = _perfect_session()
        vpe_result = compute_vpe(session)
        # 无噪声会话的 bias 是舍入误差量级（可能带负号），报告不得写成 -0.0000
        self.assertLess(vpe_result["metrics"]["bias_norm_mm"], 1e-9)
        text = render_markdown(
            session, vpe_result, fk_consistency=check_fk_consistency(session))
        self.assertNotIn("-0.0000", text)
        self.assertIn("0.0000", text)


class BiasDiagnosisTest(unittest.TestCase):
    """systematic / pose-dependent / mixed 三档与主偏置轴。"""

    def test_systematic(self):
        diagnosis = bias_diagnosis(_vpe_result([0.0, 0.0, 0.6], 0.75))
        self.assertEqual(diagnosis["dominant"], "systematic")
        self.assertEqual(diagnosis["dominant_axis"], "z")
        self.assertAlmostEqual(diagnosis["bias_ratio"], 0.8)
        self.assertIn("固定偏置为主", diagnosis["message"])
        self.assertIn("Tool0—靶球偏置 ΔP", diagnosis["message"])
        self.assertIn("跟踪仪建站偏置", diagnosis["message"])

    def test_pose_dependent(self):
        diagnosis = bias_diagnosis(_vpe_result([0.1, -0.3, 0.0], 2.0))
        self.assertEqual(diagnosis["dominant"], "pose-dependent")
        self.assertEqual(diagnosis["dominant_axis"], "y")
        self.assertIn("随位姿变化", diagnosis["message"])
        self.assertIn("运动学模型（DH/零点）", diagnosis["message"])
        self.assertIn("视觉位姿估计", diagnosis["message"])

    def test_mixed(self):
        diagnosis = bias_diagnosis(_vpe_result([-0.2, 0.35, 0.1], 1.0))
        self.assertEqual(diagnosis["dominant"], "mixed")
        self.assertEqual(diagnosis["dominant_axis"], "y")
        self.assertIn("并存", diagnosis["message"])
        self.assertIn("Tool0—靶球偏置 ΔP", diagnosis["message"])
        self.assertIn("运动学模型", diagnosis["message"])

    def test_threshold_boundary_is_systematic(self):
        # ratio 恰为 0.5：>= threshold 必须判 systematic
        diagnosis = bias_diagnosis(_vpe_result([0.3, 0.4, 0.0], 1.0))
        self.assertAlmostEqual(diagnosis["bias_ratio"], 0.5)
        self.assertEqual(diagnosis["dominant"], "systematic")

    def test_zero_rmse_ratio_is_zero(self):
        diagnosis = bias_diagnosis(_vpe_result([0.0, 0.0, 0.0], 0.0))
        self.assertEqual(diagnosis["bias_ratio"], 0.0)
        self.assertEqual(diagnosis["dominant"], "pose-dependent")

    def test_missing_fields_raise_value_error(self):
        with self.assertRaises(ValueError):
            bias_diagnosis({})


class FkSectionTest(unittest.TestCase):
    """第 7 节：None 写未提供，真实 dict 渲染数值。"""

    def test_none_renders_not_provided(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result, fk_consistency=None)
        section = text[text.index("## 7."):text.index("## 8.")]
        self.assertIn("未提供", section)

    def test_real_consistency_renders_numbers(self):
        session = _perfect_session()
        vpe_result = compute_vpe(session)
        consistency = check_fk_consistency(session)
        self.assertEqual(consistency["n_compared"], 3)
        text = render_markdown(
            session, vpe_result, fk_consistency=consistency)
        section = text[text.index("## 7."):text.index("## 8.")]
        self.assertIn("| 比较样本数 | 3 |", section)
        self.assertIn("| 缺失数 | 0 |", section)
        self.assertIn("| 位置最大偏差 (mm) | 0.0000 |", section)
        self.assertIn("| 旋转最大偏差 (deg) | 0.0000° |", section)
        self.assertIn("自洽", section)


class MatrixAndNotesTest(unittest.TestCase):
    """第 6 节 16 值行与米制提醒；第 9 节备注可选。"""

    def test_matrix16_line_and_metre_reminder(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result)
        section = text[text.index("## 6."):text.index("## 7.")]
        self.assertIn("米制", section)
        self.assertIn("meters", section)
        match = re.search(r"```text\n(.+)\n```", section)
        self.assertIsNotNone(match)
        values = [float(value) for value in match.group(1).split(",")]
        self.assertEqual(len(values), 16)
        np.testing.assert_allclose(
            values, to_matrix16(vpe_result["matrix"], units="mm"), atol=5e-5)

    def test_extra_notes_appended_as_section_nine(self):
        session, vpe_result = _example()
        text = render_markdown(
            session, vpe_result,
            extra_notes=["补充：X 来自 2024 批标定。", "第二条备注"])
        self.assertIn("## 9. 备注", text)
        self.assertLess(text.index("## 8. 复现方式"), text.index("## 9. 备注"))
        self.assertIn("- 补充：X 来自 2024 批标定。", text)
        self.assertIn("- 第二条备注", text)
        self.assertTrue(text.rstrip().endswith("第二条备注"))

    def test_empty_notes_omit_section(self):
        session, vpe_result = _example()
        text = render_markdown(session, vpe_result, extra_notes=[])
        self.assertNotIn("## 9. 备注", text)


class WriteMarkdownTest(unittest.TestCase):
    """write_markdown：创建父目录、UTF-8、返回路径。"""

    def test_writes_utf8_and_creates_parent(self):
        import tempfile

        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            target = Path(directory) / "nested" / "report.md"
            written = write_markdown(target, "# 标题\n")
            self.assertEqual(written, target)
            self.assertEqual(written.read_text(encoding="utf-8"), "# 标题\n")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
