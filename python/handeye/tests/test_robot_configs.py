# -*- coding: utf-8 -*-
"""仓库内机器人配置的一致性回归测试。

`data/configs/` 下的 URDF 由控制器 JSON 转换而来，`*_dh.json` 是同一组 DH 的
显式副本（供 ``--dh-old/--dh-new`` 与会话 ``meta.dh_override`` 使用）。
两者一旦漂移，离线重算会用错基准却不报错，所以在这里钉死：

* URDF 能被解析，且满足解析 IK 的结构前提（alpha 模式 + 球型腕）；
* ``*_dh.json`` 与该 URDF 的基准 DH 逐分量相等、FK 完全一致；
* 零位姿态与“限位内可达性”自测通过。
"""

import math
import unittest
from pathlib import Path

import numpy as np

from handeye.dh import DhOverride, RobotKinematics, fk_difference
from robot_model.nexbot_ik import Nexbot6R

CONFIG_DIR = Path(__file__).resolve().parents[3] / "data" / "configs"

#: (URDF 文件名, DH 副本文件名, 说明)
ROBOT_CONFIGS = (
    ("ch2000_origin.urdf", None, "CH2000（昌宏）"),
    ("hyundai_general6s.urdf", "hyundai_general6s_dh.json", "Hyundai R_GENERAL_6S"),
)


class RobotConfigTest(unittest.TestCase):
    def _samples(self, robot, count=20, seed=0):
        rng = np.random.default_rng(seed)
        return [[float(rng.uniform(lo, hi)) for lo, hi in robot.limits]
                for _ in range(count)]

    def test_urdf_parses_and_has_spherical_wrist(self):
        for urdf_name, _dh_name, label in ROBOT_CONFIGS:
            with self.subTest(robot=label):
                robot = Nexbot6R(str(CONFIG_DIR / urdf_name))
                self.assertEqual(len(robot.limits), 6)
                # 解析 IK 的结构前提（类构造时已校验，这里显式断言便于定位）
                expect_alpha = [90.0, 0.0, 90.0, 90.0, -90.0, 0.0]
                got_alpha = [round(math.degrees(v), 9) for v in robot.dh_alpha]
                self.assertEqual(got_alpha, expect_alpha, msg=label)
                self.assertAlmostEqual(robot.dh_a[3], 0.0, places=9)
                self.assertAlmostEqual(robot.dh_a[4], 0.0, places=9)
                self.assertAlmostEqual(robot.dh_d[4], 0.0, places=9)

    def test_dh_copy_matches_urdf_baseline(self):
        for urdf_name, dh_name, label in ROBOT_CONFIGS:
            if dh_name is None:
                continue
            with self.subTest(robot=label):
                urdf_path = CONFIG_DIR / urdf_name
                override = DhOverride.from_json(CONFIG_DIR / dh_name)
                baseline = DhOverride.from_base(urdf_path)
                for key in ("a", "alpha", "d", "theta_offset"):
                    np.testing.assert_allclose(
                        getattr(override, key), getattr(baseline, key),
                        atol=1e-9, err_msg=f"{label}.{key}")

                plain = RobotKinematics(urdf_path)
                with_override = RobotKinematics(urdf_path, override)
                report = fk_difference(plain, with_override, self._samples(plain))
                self.assertEqual(report["position_max_mm"], 0.0, msg=label)
                # 旋转差用 acos 求，在接近单位阵时相对误差被放大（本例实测
                # 1.7e-06 度，对应位置差为 0）；按“远低于任何真实偏差”取容差。
                self.assertLess(report["rotation_max_deg"], 1e-4, msg=label)

    def test_fk_ik_roundtrip_within_limits(self):
        for urdf_name, _dh_name, label in ROBOT_CONFIGS:
            with self.subTest(robot=label):
                robot = Nexbot6R(str(CONFIG_DIR / urdf_name))
                checked = 0
                for q in self._samples(robot, count=12, seed=3):
                    pose = robot.fk(q)
                    solutions = robot.ik(pose, q_ref=q)
                    self.assertTrue(solutions, msg=f"{label}: {np.round(q, 2)} 无反解")
                    best = min(solutions,
                               key=lambda s: float(np.sum(
                                   (np.asarray(s, dtype=float)
                                    - np.asarray(q, dtype=float)) ** 2)))
                    back = robot.fk(best)
                    np.testing.assert_allclose(back[:3, 3], pose[:3, 3], atol=1e-6,
                                               err_msg=label)
                    checked += 1
                self.assertEqual(checked, 12)

    def test_zero_pose_is_finite(self):
        for urdf_name, _dh_name, label in ROBOT_CONFIGS:
            with self.subTest(robot=label):
                pose = Nexbot6R(str(CONFIG_DIR / urdf_name)).fk([0.0] * 6)
                self.assertTrue(np.all(np.isfinite(pose)))
                self.assertTrue(np.allclose(pose[3], [0, 0, 0, 1]))


if __name__ == "__main__":
    unittest.main()
