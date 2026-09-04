import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "gazebo-nav/phanthymotus_sim_nav/goal_result.py"
SPEC = importlib.util.spec_from_file_location("goal_result", MODULE_PATH)
goal_result = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(goal_result)


class NavigationGoalResultTest(unittest.TestCase):
    def test_success_clears_stale_feedback_distance(self):
        self.assertEqual(
            goal_result.terminal_goal_update(4, cancel_requested=False),
            {"state": "succeeded", "error": "", "distance_remaining": 0.0},
        )

    def test_user_cancel_is_not_reported_as_an_error(self):
        self.assertEqual(
            goal_result.terminal_goal_update(5, cancel_requested=True),
            {"state": "canceled", "error": "", "cancel_reason": "user_requested"},
        )

    def test_non_user_cancel_remains_distinguishable(self):
        self.assertEqual(
            goal_result.terminal_goal_update(5, cancel_requested=False),
            {"state": "canceled", "error": "", "cancel_reason": "nav2_canceled"},
        )

    def test_other_terminal_status_remains_a_failure(self):
        self.assertEqual(
            goal_result.terminal_goal_update(6, cancel_requested=False),
            {"state": "failed", "error": "nav2_status_6"},
        )


if __name__ == "__main__":
    unittest.main()
