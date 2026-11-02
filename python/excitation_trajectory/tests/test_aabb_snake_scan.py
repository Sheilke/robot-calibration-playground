"""连续 AABB 蛇形网格路点的单元测试。"""

import unittest

from excitation_trajectory.aabb_snake_scan import plan_waypoints


class PlanWaypointsTest(unittest.TestCase):
    """验证网格覆盖和任意一步只沿一个网格轴移动。"""

    @staticmethod
    def _grid_indices(waypoints):
        """本测试用 0..N 的整数坐标将路点位置还原为网格下标。"""
        return [tuple(int(round(value)) for value in waypoint.pos)
                for waypoint in waypoints]

    def assert_adjacent_grid_steps(self, indices):
        for previous, current in zip(indices, indices[1:]):
            with self.subTest(previous=previous, current=current):
                self.assertEqual(
                    sum(abs(after - before)
                        for before, after in zip(previous, current)),
                    1,
                )

    def test_two_by_two_by_two_grid_is_unique_and_continuous(self):
        waypoints = plan_waypoints(
            0, 2, 0, 2, 0, 2, xs=2, ys=2, zs=2, rpy_ref=(0, 0, 0)
        )
        indices = self._grid_indices(waypoints)

        self.assertEqual(len(waypoints), 27)
        self.assertEqual(len(set(indices)), 27)
        self.assertEqual(indices[0], (0, 0, 0))
        self.assertEqual(indices[8], (2, 2, 0))
        self.assertEqual(indices[9], (2, 2, 1))
        self.assert_adjacent_grid_steps(indices)

    def test_one_by_one_by_one_grid_is_unique_and_continuous(self):
        waypoints = plan_waypoints(
            0, 1, 0, 1, 0, 1, xs=1, ys=1, zs=1, rpy_ref=(0, 0, 0)
        )
        indices = self._grid_indices(waypoints)

        self.assertEqual(len(waypoints), 8)
        self.assertEqual(len(set(indices)), 8)
        self.assertEqual(indices[0], (0, 0, 0))
        self.assert_adjacent_grid_steps(indices)


if __name__ == "__main__":
    unittest.main()
