# -*- coding: utf-8 -*-
"""奇异守卫（逐点重采姿态）的单元测试。"""

import io
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

from excitation_trajectory.aabb_snake_scan import (
    DEFAULT_J5_MIN_DEG,
    _sample_perturbations,
    plan_waypoints,
    solve_chain,
    solve_chain_guarded,
)
from robot_model.nexbot_ik import Nexbot6R

URDF = Path(__file__).resolve().parents[3] / "data" / "configs" / "hyundai_general6s.urdf"
RPY = (3.0, 73.0, -167.0)
SEED_Q = [43.0, -30.0, -36.0, 42.0, -89.0, -1.0]
AMPS = (15.0, 20.0, 15.0)


def _waypoints(x_max=1200, y=(-200, 200), z=(700, 900), z_insert=0):
    return plan_waypoints(800, x_max, y[0], y[1], z[0], z[1],
                          2, 2, 1, RPY, z_insert=z_insert)


class GuardOffTest(unittest.TestCase):
    def test_threshold_zero_matches_plain_chain(self):
        """守卫关闭时必须与 solve_chain 完全一致（向后兼容）。"""
        rbt = Nexbot6R(str(URDF))
        self.addCleanup(setattr, rbt, "_dummy", None)
        wps_a = _waypoints()
        wps_b = _waypoints()
        _sample_perturbations(wps_a, 0, AMPS)
        _sample_perturbations(wps_b, 0, AMPS)
        with redirect_stdout(io.StringIO()):
            guarded, report = solve_chain_guarded(rbt, wps_a, SEED_Q, j5_min=0.0,
                                                  j3_min=0.0, attempts=5, seed=0,
                                                  amplitudes_deg=AMPS,
                                                  extrinsic=False)
        poses = [rbt.pose_to_T(w.pos, w.rpy_pert, extrinsic=False) for w in wps_b]
        with redirect_stdout(io.StringIO()):
            plain = solve_chain(rbt, poses, SEED_Q)
        self.assertEqual([s.q for s in guarded], [s.q for s in plain])
        self.assertEqual(report["resampled"], [])


class GuardOnTest(unittest.TestCase):
    def test_threshold_is_respected(self):
        rbt = Nexbot6R(str(URDF))
        wps = _waypoints()
        _sample_perturbations(wps, 0, AMPS)
        with redirect_stdout(io.StringIO()):
            steps, report = solve_chain_guarded(
                rbt, wps, SEED_Q, j5_min=20.0, attempts=40, seed=0,
                amplitudes_deg=AMPS, extrinsic=False)
        ok = [s for s in steps if s.ok]
        self.assertEqual(len(ok), len(wps))
        self.assertGreaterEqual(min(abs(s.q[4]) for s in ok), 20.0 - 1e-9)
        self.assertEqual(report["violations"], [])

    def test_high_threshold_reports_violations_instead_of_crashing(self):
        """阈值高到做不到时，不崩、不静默：记录 violations 并给出最好结果。"""
        rbt = Nexbot6R(str(URDF))
        wps = _waypoints()
        _sample_perturbations(wps, 0, AMPS)
        with redirect_stdout(io.StringIO()):
            steps, report = solve_chain_guarded(
                rbt, wps, SEED_Q, j5_min=89.0, attempts=6, seed=0,
                amplitudes_deg=AMPS, extrinsic=False)
        self.assertEqual(len([s for s in steps if s.ok]), len(wps))
        self.assertTrue(report["violations"] or
                        min(abs(s.q[4]) for s in steps) >= 89.0 - 1e-9)

    def test_resampling_is_deterministic(self):
        rbt = Nexbot6R(str(URDF))
        runs = []
        for _ in range(2):
            wps = _waypoints()
            _sample_perturbations(wps, 0, AMPS)
            with redirect_stdout(io.StringIO()):
                steps, report = solve_chain_guarded(
                    rbt, wps, SEED_Q, j5_min=20.0, attempts=10, seed=7,
                    amplitudes_deg=AMPS, extrinsic=False)
            runs.append(([s.q for s in steps],
                         [r["index"] for r in report["resampled"]]))
        self.assertEqual(runs[0], runs[1])

    def test_positions_never_change(self):
        """守卫只动姿态，位置必须原样。"""
        rbt = Nexbot6R(str(URDF))
        wps = _waypoints()
        before = [w.pos for w in wps]
        _sample_perturbations(wps, 0, AMPS)
        with redirect_stdout(io.StringIO()):
            solve_chain_guarded(rbt, wps, SEED_Q, j5_min=20.0, attempts=20,
                                seed=0, amplitudes_deg=AMPS, extrinsic=False)
        self.assertEqual([w.pos for w in wps], before)

    def test_resampled_perturbation_within_amplitude(self):
        rbt = Nexbot6R(str(URDF))
        wps = _waypoints()
        _sample_perturbations(wps, 0, AMPS)
        with redirect_stdout(io.StringIO()):
            solve_chain_guarded(rbt, wps, SEED_Q, j5_min=20.0, attempts=20,
                                seed=0, amplitudes_deg=AMPS, extrinsic=False)
        for w in wps:
            for axis, amp in zip(range(3), AMPS):
                self.assertLessEqual(abs(w.delta_rpy[axis]), amp + 1e-12)

    def test_default_threshold_is_documented_value(self):
        self.assertEqual(DEFAULT_J5_MIN_DEG, 20.0)


if __name__ == "__main__":
    unittest.main()
