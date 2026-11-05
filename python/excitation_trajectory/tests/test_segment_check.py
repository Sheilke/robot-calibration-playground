# -*- coding: utf-8 -*-
"""段内奇异检查：端点安全不代表 PTP 路径安全。"""

import unittest

from excitation_trajectory.aabb_snake_scan import ChainStep, check_segments


def _step(index, j3, j5, ok=True):
    return ChainStep(index=index, q=[0.0, 0.0, j3, 0.0, j5, 0.0],
                     n_sols=1, config=(0, 1, 1), dq_l2=0.0, dq_l1=0.0, ok=ok)


class SegmentCheckTest(unittest.TestCase):
    def test_detects_elbow_crossing(self):
        """两端 J3 各 8° 但反号 -> 段内必过 0（正是现场停机的成因）。"""
        report = check_segments([_step(0, 8.0, -40.0), _step(1, -8.0, -40.0)])
        self.assertEqual(report["j3_crossings"], 1)
        self.assertLess(report["j3_abs_min"], 0.5)
        self.assertFalse(report["segments"][0]["j5_crosses_zero"])

    def test_detects_wrist_crossing(self):
        report = check_segments([_step(0, -30.0, 10.0), _step(1, -30.0, -10.0)])
        self.assertEqual(report["j5_crossings"], 1)
        self.assertLess(report["j5_abs_min"], 0.5)

    def test_safe_segment_reports_minimum(self):
        report = check_segments([_step(0, -40.0, -60.0), _step(1, -20.0, -30.0)])
        self.assertEqual(report["j3_crossings"], 0)
        self.assertEqual(report["j5_crossings"], 0)
        self.assertAlmostEqual(report["j3_abs_min"], 20.0, places=6)
        self.assertAlmostEqual(report["j5_abs_min"], 30.0, places=6)

    def test_skips_failed_steps(self):
        steps = [_step(0, -40.0, -60.0),
                 ChainStep(index=1, q=None, n_sols=0, config=None,
                           dq_l2=None, dq_l1=None, ok=False),
                 _step(2, -35.0, -50.0)]
        report = check_segments(steps)
        self.assertEqual(len(report["segments"]), 1)
        self.assertEqual(report["segments"][0]["from"], 0)
        self.assertEqual(report["segments"][0]["to"], 2)

    def test_empty_and_single(self):
        self.assertIsNone(check_segments([])["j3_abs_min"])
        single = check_segments([_step(0, -40.0, -60.0)])
        self.assertEqual(single["segments"], [])
        self.assertIsNone(single["j3_abs_min"])


if __name__ == "__main__":
    unittest.main()
