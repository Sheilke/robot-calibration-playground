# -*- coding: utf-8 -*-
"""handeye.verify CLI 端到端测试：正常路径、X 覆盖、错误退出码、quiet、
--no-report、JSON、FK 一致性告警与 --strict。"""

import io
import json
import re
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import numpy as np

from handeye.pose_encoder import matrix_to_pose_fields
from handeye.se3 import to_matrix16
from handeye.session import load_session
from handeye.verify import main

ROOT = Path(__file__).resolve().parents[3]
SESSION_EXAMPLE = ROOT / "data" / "configs" / "session_example.json"


def _run(argv):
    """调用 ``main`` 并捕获 stdout/stderr。"""
    stdout, stderr = io.StringIO(), io.StringIO()
    with redirect_stdout(stdout), redirect_stderr(stderr):
        code = main(argv)
    return code, stdout.getvalue(), stderr.getvalue()


def _payload():
    """读入示例会话的深拷贝，供每个用例独立改写。"""
    return json.loads(SESSION_EXAMPLE.read_text(encoding="utf-8"))


def _write_session(directory, payload, name="session.json") -> Path:
    """把 payload 写成临时会话 JSON，返回路径。"""
    path = Path(directory) / name
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def _parse_rmse(stdout: str) -> float:
    """从摘要里取 RMSE（兼容 ``RMSE=`` 与 ``VPE_RMSE:`` 两种写法）。"""
    match = re.search(r"RMSE[=:\s]+([0-9]+(?:\.[0-9]+)?)", stdout)
    if match is None:
        raise AssertionError(f"stdout 中没有 RMSE：{stdout!r}")
    return float(match.group(1))


def _fk_payload(offset_mm=5.0):
    """把示例会话的 recorded_pose 填成 FK 结果并整体平移 ``offset_mm``。"""
    payload = _payload()
    session = load_session(SESSION_EXAMPLE)
    for sample in payload["samples"]:
        model = session.sample_by_id(sample["pose_id"])
        matrix = session.kinematics.fk(model.q_deg).copy()
        matrix[:3, 3] += np.array([offset_mm, 0.0, 0.0])
        position, orientation = matrix_to_pose_fields(matrix)
        sample["recorded_pose"] = {
            "position": [float(value) for value in position],
            "orientation": [float(value) for value in orientation],
        }
    return payload


class VerifyCliTest(unittest.TestCase):
    """CLI 行为与退出码矩阵。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(dir="/tmp")
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)

    # ---- 1. 正常路径 ----
    def test_normal_path_writes_report(self):
        out = self.tmp / "report.md"
        code, stdout, stderr = _run(
            [str(SESSION_EXAMPLE), "-o", str(out)])
        self.assertEqual(code, 0)
        self.assertEqual(stderr, "")
        self.assertTrue(out.is_file())
        self.assertTrue(out.read_text(encoding="utf-8").strip())
        self.assertIn("RMSE", stdout)
        self.assertIn("报告已写入", stdout)
        self.assertGreater(_parse_rmse(stdout), 0.0)

    # ---- 2. --matrix16 覆盖 ----
    def test_matrix16_override_increases_rmse(self):
        session = load_session(SESSION_EXAMPLE)
        baseline_code, baseline_out, _ = _run(
            [str(SESSION_EXAMPLE), "--no-report", "--quiet"])
        self.assertEqual(baseline_code, 0)
        baseline = _parse_rmse(baseline_out)

        values = to_matrix16(session.handeye_matrix, units="mm")
        values[3] += 2.0  # 平移 x 加 2 mm
        code, stdout, stderr = _run(
            [str(SESSION_EXAMPLE),
             "--matrix16", *[repr(value) for value in values],
             "--no-report", "--quiet"])
        self.assertEqual(code, 0, stderr)
        overridden = _parse_rmse(stdout)
        self.assertGreater(overridden, baseline + 1.0)

    def test_matrix16_metre_units_match_millimetres(self):
        session = load_session(SESSION_EXAMPLE)
        baseline_code, baseline_out, _ = _run(
            [str(SESSION_EXAMPLE), "--no-report", "--quiet"])
        self.assertEqual(baseline_code, 0)
        metres = to_matrix16(session.handeye_matrix, units="m")
        code, stdout, stderr = _run(
            [str(SESSION_EXAMPLE),
             "--matrix16", *[repr(value) for value in metres],
             "--matrix-units", "m", "--no-report", "--quiet"])
        self.assertEqual(code, 0, stderr)
        self.assertAlmostEqual(
            _parse_rmse(stdout), _parse_rmse(baseline_out), places=4)

    # ---- 3. 会话无 X ----
    def test_missing_handeye_matrix_returns_two_without_report(self):
        payload = _payload()
        payload["handeye_matrix"] = None
        session_path = _write_session(self.tmp, payload, "no_matrix.json")
        out = self.tmp / "should_not_exist.md"
        code, _, stderr = _run([str(session_path), "-o", str(out)])
        self.assertEqual(code, 2)
        self.assertIn("handeye_matrix", stderr)
        self.assertIn("--matrix16", stderr)
        self.assertFalse(out.exists())

    # ---- 4. 会话路径不存在 ----
    def test_missing_session_returns_two(self):
        out = self.tmp / "should_not_exist.md"
        code, _, stderr = _run(
            [str(self.tmp / "missing.json"), "-o", str(out)])
        self.assertEqual(code, 2)
        self.assertNotEqual(stderr.strip(), "")
        self.assertNotIn("Traceback", stderr)
        self.assertFalse(out.exists())

    # ---- 5. 数据非法：q 只有 5 个 ----
    def test_invalid_q_deg_returns_two_with_hint(self):
        payload = _payload()
        payload["samples"][0]["q_deg"] = [0.0, 0.0, 0.0, 0.0, 0.0]
        session_path = _write_session(self.tmp, payload, "bad_q.json")
        out = self.tmp / "should_not_exist.md"
        code, _, stderr = _run([str(session_path), "-o", str(out)])
        self.assertEqual(code, 2)
        self.assertIn("q_deg", stderr)
        self.assertNotIn("Traceback", stderr)
        self.assertFalse(out.exists())

    # ---- 6. --quiet 只输出一行 ----
    def test_quiet_prints_single_line(self):
        out = self.tmp / "quiet.md"
        code, stdout, stderr = _run(
            [str(SESSION_EXAMPLE), "-o", str(out), "--quiet"])
        self.assertEqual(code, 0, stderr)
        self.assertEqual(len(stdout.strip().splitlines()), 1)
        self.assertIn("RMSE", stdout)
        self.assertIn("VPE_RMSE", out.read_text(encoding="utf-8"))

    # ---- 7. --no-report ----
    def test_no_report_skips_file(self):
        out = self.tmp / "not_written.md"
        code, stdout, _ = _run(
            [str(SESSION_EXAMPLE), "-o", str(out), "--no-report"])
        self.assertEqual(code, 0)
        self.assertFalse(out.exists())
        self.assertIn("RMSE", stdout)

    # ---- 8. --json ----
    def test_json_output_is_valid_and_complete(self):
        json_path = self.tmp / "result.json"
        code, _, stderr = _run(
            [str(SESSION_EXAMPLE), "--no-report", "--json", str(json_path),
             "--quiet"])
        self.assertEqual(code, 0, stderr)
        raw = json_path.read_text(encoding="utf-8")
        self.assertNotIn("Infinity", raw)
        self.assertNotIn("NaN", raw)
        payload = json.loads(raw)
        self.assertEqual(
            set(payload),
            {"meta", "metrics", "errors", "fk_consistency",
             "bias_diagnosis", "matrix16_mm"})
        self.assertEqual(len(payload["matrix16_mm"]), 16)
        self.assertEqual(len(payload["errors"]), 3)
        for entry in payload["errors"]:
            self.assertEqual(
                set(entry),
                {"pose_id", "position_vision", "error", "vpe_mm",
                 "n_frames", "n_ids", "spread_mm"})
            self.assertEqual(len(entry["position_vision"]), 3)
            self.assertEqual(len(entry["error"]), 3)
        self.assertEqual(payload["bias_diagnosis"]["dominant"], "systematic")
        self.assertEqual(
            set(payload["metrics"]),
            {"vpe_rmse_mm", "vpe_max_mm", "vpe_p95_mm", "vpe_mean_mm",
             "bias", "bias_norm_mm", "per_axis_rmse_mm",
             "per_axis_bias_mm"})

    # ---- 9. FK 一致性告警 / --strict ----
    def test_fk_warning_and_strict_exit_code(self):
        session_path = _write_session(
            self.tmp, _fk_payload(offset_mm=5.0), "fk_off.json")

        code, stdout, stderr = _run(
            [str(session_path), "--no-report", "--fk-tolerance-mm", "1"])
        self.assertEqual(code, 0)
        combined = stdout + stderr
        self.assertIn("WARNING", combined)
        self.assertIn("5.0000", combined)

        strict_code, strict_stdout, strict_stderr = _run(
            [str(session_path), "--no-report", "--fk-tolerance-mm", "1",
             "--strict"])
        self.assertEqual(strict_code, 3)
        self.assertIn("WARNING", strict_stdout + strict_stderr)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
