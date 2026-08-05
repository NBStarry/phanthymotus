import unittest

from g1_nav2.readiness import evaluate_readiness


class ReadinessTest(unittest.TestCase):
    def _ready(self, **overrides):
        values = {
            "now_monotonic": 100.0,
            "max_age_sec": 0.5,
            "odom_status": {
                "state": "ready",
                "source_age_sec": 0.05,
                "timestamp_source": "driver",
            },
            "odom_status_received_at": 99.9,
            "scan_received_at": 99.9,
            "scan_source_age_sec": 0.05,
            "lifecycle_states": {
                "controller_server": 3,
                "velocity_smoother": 3,
                "planner_server": 3,
                "bt_navigator": 3,
            },
            "action_server_ready": True,
            "map_ready": True,
            "map_to_base_ready": True,
        }
        values.update(overrides)
        return evaluate_readiness(**values)

    def test_all_live_dependencies_are_ready(self):
        result = self._ready()

        self.assertTrue(result["n3_ready"])
        self.assertTrue(result["navigation_ready"])
        self.assertEqual(result["readiness_blockers"], [])

    def test_adapter_receive_timestamp_uses_fresh_status_delivery(self):
        result = self._ready(
            odom_status={
                "state": "ready",
                "source_age_sec": None,
                "timestamp_source": "adapter_receive",
            }
        )

        self.assertTrue(result["n3_ready"])
        self.assertNotIn("odom_source_stamp_stale", result["readiness_blockers"])

    def test_stale_source_and_inactive_lifecycle_fail_closed(self):
        result = self._ready(
            odom_status={
                "state": "ready",
                "source_age_sec": 2.0,
                "timestamp_source": "driver",
            },
            scan_source_age_sec=2.0,
            lifecycle_states={"controller_server": 2},
        )

        self.assertFalse(result["n3_ready"])
        self.assertIn("odom_source_stamp_stale", result["readiness_blockers"])
        self.assertIn("scan_source_stamp_stale", result["readiness_blockers"])
        self.assertIn(
            "lifecycle_not_active:controller_server",
            result["readiness_blockers"],
        )

    def test_unknown_timestamp_source_fails_closed(self):
        result = self._ready(
            odom_status={"state": "ready", "source_age_sec": 0.01}
        )

        self.assertIn("odom_source_stamp_stale", result["readiness_blockers"])

    def test_map_and_tf_only_block_navigation(self):
        result = self._ready(map_ready=False, map_to_base_ready=False)

        self.assertTrue(result["n3_ready"])
        self.assertFalse(result["navigation_ready"])
        self.assertEqual(
            result["navigation_blockers"][-2:],
            ["map_not_ready", "map_to_base_unavailable"],
        )


if __name__ == "__main__":
    unittest.main()
