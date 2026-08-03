import unittest

import numpy as np

from modules.waypoint_adapter import WaypointAdapter, WaypointAdapterConfig


class WaypointAdapterTest(unittest.TestCase):
    def test_nonholonomic_adapter_anchors_and_stops(self):
        adapter = WaypointAdapter(WaypointAdapterConfig(holonomic=False))
        adapter.set_waypoints(np.array([[1.0, 0.2, 0.0]]), np.array([0.0, 0.0, 0.0]))

        command = adapter.compute_command(np.array([0.0, 0.0, 0.0]), dt_s=0.02)
        self.assertGreater(command[0], 0.0)
        self.assertEqual(command[1], 0.0)
        self.assertGreater(command[2], 0.0)

        stopped = adapter.compute_command(np.array([1.0, 0.2, 0.0]), dt_s=0.02)
        np.testing.assert_allclose(stopped, np.zeros(3))
        self.assertTrue(adapter.goal_reached)

    def test_behind_target_rotates_without_forward_motion(self):
        adapter = WaypointAdapter(WaypointAdapterConfig(holonomic=False))
        adapter.set_waypoints(np.array([[-1.0, 0.0, np.pi]]), np.array([0.0, 0.0, 0.0]))
        command = adapter.compute_command(np.array([0.0, 0.0, 0.0]), dt_s=0.1)
        self.assertAlmostEqual(command[0], 0.0)
        self.assertNotEqual(command[2], 0.0)

    def test_multi_waypoint_advances_after_reaching_first(self):
        adapter = WaypointAdapter(WaypointAdapterConfig(holonomic=True))
        adapter.set_waypoints(
            np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
            np.array([0.0, 0.0, 0.0]),
        )
        adapter.compute_command(np.array([1.0, 0.0, 0.0]), dt_s=0.02)
        self.assertEqual(adapter.target_index, 1)
        command = adapter.compute_command(np.array([1.0, 0.0, 0.0]), dt_s=0.02)
        self.assertGreater(command[0], 0.0)

    def test_valid_mask_filters_waypoints(self):
        adapter = WaypointAdapter(WaypointAdapterConfig(holonomic=True))
        adapter.set_waypoints(
            np.array([[1.0, 0.0, 0.0], [5.0, 0.0, 0.0]]),
            np.array([0.0, 0.0, 0.0]),
            valid_mask=np.array([True, False]),
        )
        self.assertEqual(len(adapter._world_waypoints), 1)
        np.testing.assert_allclose(adapter._world_waypoints[0], [1.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
