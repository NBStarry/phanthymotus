from __future__ import annotations


# action_msgs/msg/GoalStatus values from ROS 2 Humble.
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5


def terminal_goal_update(status: int, *, cancel_requested: bool) -> dict:
    """Map a Nav2 action terminal status to the card's public state."""
    if status == STATUS_SUCCEEDED:
        return {
            "state": "succeeded",
            "error": "",
            "distance_remaining": 0.0,
        }
    if status == STATUS_CANCELED:
        return {
            "state": "canceled",
            "error": "",
            "cancel_reason": "user_requested" if cancel_requested else "nav2_canceled",
        }
    return {
        "state": "failed",
        "error": f"nav2_status_{status}",
    }
