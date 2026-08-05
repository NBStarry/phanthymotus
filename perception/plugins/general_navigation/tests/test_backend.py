import threading
import unittest

from plugins.general_navigation.backend import RosTopicNavigationBackend
from plugins.general_navigation.core import NavigationBackendError


class NavigationBackendStopConfirmationTest(unittest.TestCase):
    @staticmethod
    def backend(stop_response):
        backend = object.__new__(RosTopicNavigationBackend)
        backend._condition = threading.Condition()
        backend._closed = False
        backend._navigation = {}
        backend._request = lambda action, args, nav_id: dict(stop_response)
        return backend

    def test_stall_requires_confirmed_terminal_stop(self):
        backend = self.backend({"status": "stopping", "terminal_confirmed": False})

        with self.assertRaises(NavigationBackendError) as caught:
            backend._wait_navigation("nav-1", 0.0)

        self.assertEqual(
            caught.exception.code, "navigation_stalled_stop_unconfirmed"
        )

    def test_stall_returns_timeout_only_after_stop_confirmation(self):
        backend = self.backend({"status": "stopped", "terminal_confirmed": True})

        result = backend._wait_navigation("nav-1", 0.0)

        self.assertEqual(result["status"], "timeout")
        self.assertTrue(result["terminal_confirmed"])


class NavigationBackendRuntimeSwitchTest(unittest.TestCase):
    @staticmethod
    def backend(responses):
        backend = object.__new__(RosTopicNavigationBackend)
        backend._responses_for_test = list(responses)
        backend._requests_for_test = []
        backend._waits_for_test = []

        def request(action, args, *, nav_id):
            backend._requests_for_test.append((action, dict(args), nav_id))
            return backend._responses_for_test.pop(0)

        backend._request = request
        backend._wait_for_runtime = lambda mode, map_name="": (
            backend._waits_for_test.append((mode, map_name))
        )
        return backend

    def test_start_mapping_switches_then_retries_same_action(self):
        backend = self.backend(
            [
                {
                    "status": "switching",
                    "map_name": "office",
                    "mode_switch_required": True,
                    "next_runtime_mode": "mapping",
                    "retry_action_after_switch": True,
                },
                {"status": "mapping", "map_name": "office"},
            ]
        )

        result = backend.execute(
            "start_mapping", {"map_name": "office"}, nav_id=None
        )

        self.assertEqual(result["status"], "mapping")
        self.assertEqual(backend._waits_for_test, [("mapping", "office")])
        self.assertEqual(len(backend._requests_for_test), 2)
        self.assertEqual(
            backend._requests_for_test[0], backend._requests_for_test[1]
        )

    def test_stop_mapping_waits_until_saved_map_is_localized(self):
        backend = self.backend(
            [
                {
                    "status": "saved",
                    "map_name": "office",
                    "mode_switch_required": True,
                    "next_runtime_mode": "localization",
                    "retry_action_after_switch": False,
                }
            ]
        )

        result = backend.execute("stop_mapping", {}, nav_id=None)

        self.assertEqual(backend._waits_for_test, [("localization", "office")])
        self.assertEqual(result["runtime_mode"], "localization")
        self.assertTrue(result["automatic_mode_switch"])
        self.assertFalse(result["mode_switch_required"])


if __name__ == "__main__":
    unittest.main()
