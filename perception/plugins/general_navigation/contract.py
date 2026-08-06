"""Public MCP contract for the General Navigation Perception card."""

from __future__ import annotations

from copy import deepcopy


GENERAL_NAVIGATION_ACTIONS = (
    "start_mapping",
    "stop_mapping",
    "tag_place",
    "untag_place",
    "list_tags",
    "list_maps",
    "delete_map",
    "load_map",
    "navigate_to_tag",
    "navigate_to_pose",
    "wait_navigation_done",
    "pause_nav",
    "resume_nav",
    "stop_nav",
)

GENERAL_NAVIGATION_ACTION_PARAMS = {
    "start_mapping": {
        "params": ["map_name"],
        "description": "Start SLAM mapping with given map name",
    },
    "stop_mapping": {
        "params": [],
        "description": "Stop mapping and save the map",
    },
    "tag_place": {
        "params": ["name", "description"],
        "description": "Tag current position with a semantic name",
    },
    "untag_place": {
        "params": ["name"],
        "description": "Remove a place tag",
    },
    "list_tags": {
        "params": [],
        "description": "List all tags in current map with relative positions",
    },
    "list_maps": {"params": [], "description": "List all saved maps"},
    "delete_map": {
        "params": ["map_name"],
        "description": "Delete a map and its associated data",
    },
    "load_map": {
        "params": ["map_name"],
        "description": "Load a map (robot must be at map origin)",
    },
    "navigate_to_tag": {
        "params": ["tag_name", "speed", "mode"],
        "description": (
            "Navigate to a tagged place (non-blocking). mode: "
            "0=detour (the only supported mode). "
            "MUST be followed by a "
            "separate wait_navigation_done call in the same turn to wait for "
            "arrival before proceeding."
        ),
    },
    "navigate_to_pose": {
        "params": ["x", "y", "yaw", "speed", "mode"],
        "description": (
            "Navigate to coordinates (non-blocking). mode: "
            "0=detour (the only supported mode). "
            "MUST be followed by a "
            "separate wait_navigation_done call in the same turn to wait for "
            "arrival before proceeding."
        ),
    },
    "wait_navigation_done": {
        "params": ["stall_timeout"],
        "description": (
            "Block until the previous navigate_to_tag or navigate_to_pose "
            "completes. Returns on arrival, timeout, or error. Always call "
            "after navigate_to_tag/navigate_to_pose."
        ),
    },
    "pause_nav": {"params": [], "description": "Pause navigation"},
    "resume_nav": {"params": [], "description": "Resume navigation"},
    "stop_nav": {"params": [], "description": "Stop and cancel navigation"},
}


def _root(namespace: str) -> str:
    normalized = namespace.strip("/")
    return f"/{normalized}" if normalized else ""


def general_navigation_tool_definition(namespace: str) -> dict:
    """Return an isolated tool definition for one robot namespace."""

    root = _root(namespace)
    tool = {
        "name": "navigation",
        "displayName": "Navigation 2",
        "type": "processor",
        "multiInstance": False,
        "description": (
            "Navigation 2 — mapping, saved-map localization, semantic "
            "place tags and Nav2 navigation. This Perception card only emits "
            "bounded velocity proposals; an explicitly authorized Driver loco "
            "actuator owns any physical execution."
        ),
        "x-execution-control": {
            "version": 1,
            "proposal_schema": "phanthy.navigation.velocity_proposal.v1",
            "output_port": "velocity_proposal",
            "target_tool": "loco",
            "lease_argument": "_control_nav_id",
            "start_actions": ["navigate_to_tag", "navigate_to_pose"],
            "wait_actions": ["wait_navigation_done"],
            "stop_actions": ["stop_nav"],
            "pause_actions": ["pause_nav"],
            "resume_actions": ["resume_nav"],
            "terminal_statuses": [
                "arrived",
                "succeeded",
                "cancelled",
                "stopped",
                "timeout",
                "error",
                "aborted",
                "rejected",
            ],
        },
        "x-topic-actions": [
            {
                "port": "goal_pose",
                "action": "navigate_to_pose",
                "wait_action": "wait_navigation_done",
                "stop_action": "stop_nav",
                "schema": "phanthy.navigation.goal.v1",
                "id_field": "goal_id",
                "allowed_fields": ["x", "y", "yaw", "speed", "mode"],
            }
        ],
        "topic_in": [
            {
                "port": "loco_state",
                "topic": f"{root}/loco/state",
                "format": "data/json",
                "ros_type": "std_msgs/msg/String",
                "qos": "BEST_EFFORT + KEEP_LAST(depth=10) + VOLATILE",
                "schema": "unitree.g1.loco_state.legacy",
                "compatible_schemas": ["phanthy.g1.loco_state.v2"],
                "rate_hz": 10,
                "timestamp": (
                    "adapter receive time; released Driver payload has no source timestamp"
                ),
                "frame_id": "odom_source (adapter contract, absent from payload)",
                "axes": "ROS REP-103 right-handed: x forward, y left, z up",
                "units": "position=m, velocity=m/s, yaw_speed=rad/s",
                "max_age_ms": 500,
                "desc": (
                    "Released Driver locomotion JSON; the adapter labels its "
                    "receive-time and frame assumptions explicitly before "
                    "converting it to odom -> base_link"
                ),
            },
            {
                "port": "lidar",
                "topic": f"{root}/lidar/cloud",
                "format": "sensor/pointcloud",
                "ros_type": "std_msgs/msg/UInt8MultiArray",
                "qos": "RELIABLE + KEEP_LAST(depth=10) + VOLATILE",
                "schema": "unitree.g1.pointcloud.legacy",
                "compatible_schemas": ["phanthy.sensor.pointcloud.v2"],
                "rate_hz": 10,
                "timestamp": (
                    "adapter receive time; released Driver envelope has no source timestamp"
                ),
                "frame_id": "livox_frame (adapter launch contract, absent from payload)",
                "axes": "ROS REP-103 right-handed: x forward, y left, z up",
                "units": "x/y/z=float32 meters",
                "max_age_ms": 500,
                "desc": (
                    "Released Driver MID360 envelope: uint32 point_step, "
                    "uint32 point_count, raw PointCloud2 bytes; exact size is "
                    "validated before rebuilding ROS PointCloud2"
                ),
            },
            {
                "port": "goal_pose",
                "topic": f"{root}/navigation/goal_pose",
                "format": "data/json",
                "ros_type": "std_msgs/msg/String",
                "qos": "RELIABLE + KEEP_LAST(depth=10) + VOLATILE",
                "schema": "phanthy.navigation.goal.v1",
                "required": False,
                "frame_id": "map",
                "axes": "ROS REP-103 right-handed: x forward, y left",
                "units": "x/y=m, yaw=rad, speed=m/s",
                "desc": (
                    "Optional target input. Each JSON message needs a unique "
                    "goal_id plus x/y/yaw and is executed through the same "
                    "Agent Core Driver lease as navigate_to_pose."
                ),
            },
        ],
        "topic_out": [
            {
                "port": "velocity_proposal",
                "topic": f"{root}/navigation/nav2/velocity_proposal",
                "format": "data/json",
                "ros_type": "std_msgs/msg/String",
                "qos": "RELIABLE + KEEP_LAST(depth=10) + VOLATILE",
                "schema": "phanthy.navigation.velocity_proposal.v1",
                "rate_hz": 20,
                "timestamp": "issued_at_unix_ms; TTL uses Driver receive monotonic time",
                "frame_id": "base_link",
                "axes": "ROS REP-103: x forward, y left, yaw counter-clockwise",
                "units": "linear=m/s, angular=rad/s",
                "max_age_ms": 250,
                "desc": (
                    "Structured Nav2 proposal for the existing Driver loco "
                    "actuator; never a physical command by itself"
                ),
            },
        ],
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": list(GENERAL_NAVIGATION_ACTIONS),
                    "description": "Action to perform",
                },
                "map_name": {
                    "type": "string",
                    "description": "Map name (for start_mapping, delete_map, load_map)",
                },
                "name": {"type": "string", "description": "POI tag name"},
                "description": {
                    "type": "string",
                    "description": "POI description",
                },
                "tag_name": {
                    "type": "string",
                    "description": "Target tag name for navigation",
                },
                "x": {
                    "type": "number",
                    "description": "Target X coordinate (meters)",
                },
                "y": {
                    "type": "number",
                    "description": "Target Y coordinate (meters)",
                },
                "yaw": {
                    "type": "number",
                    "description": "Target yaw (radians)",
                },
                "speed": {
                    "type": "number",
                    "minimum": 0.2,
                    "maximum": 0.8,
                    "default": 0.5,
                    "description": "Navigation speed 0.2-0.8 m/s (default 0.5)",
                },
                "mode": {
                    "type": "integer",
                    "enum": [0],
                    "default": 0,
                    "description": "Obstacle mode: 0=detour (only supported mode)",
                },
                "stall_timeout": {
                    "type": "number",
                    "minimum": 1.0,
                    "maximum": 3600.0,
                    "default": 90.0,
                    "description": (
                        "Seconds without movement before declaring timeout (default 90)"
                    ),
                },
            },
            "required": ["action"],
            "x-action-params": GENERAL_NAVIGATION_ACTION_PARAMS,
        },
    }
    return deepcopy(tool)
