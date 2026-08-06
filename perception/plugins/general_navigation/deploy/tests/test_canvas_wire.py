import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).with_name("canvas_wire.py")
SPEC = importlib.util.spec_from_file_location("navigation2_canvas_wire", SCRIPT)
canvas_wire = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(canvas_wire)


def endpoint(*, topic_in=None, topic_out=None):
    return {
        "topic_in": topic_in or [],
        "topic_out": topic_out or [],
    }


class Navigation2CanvasMigrationTest(unittest.TestCase):
    def test_legacy_card_is_migrated_in_place_and_keeps_goal_link(self):
        tools = {
            "loco_state": (
                {"id": "mcp-driver", "name": "G1"},
                endpoint(
                    topic_out=[
                        {"topic": canvas_wire.STATE_TOPIC, "format": "data/json"}
                    ]
                ),
            ),
            "lidar_cloud": (
                {"id": "mcp-driver", "name": "G1"},
                endpoint(
                    topic_out=[
                        {"topic": canvas_wire.LIDAR_TOPIC, "format": "sensor/pointcloud"}
                    ]
                ),
            ),
            "navigation2": (
                {"id": "mcp-perception", "name": "Perception Stack"},
                endpoint(
                    topic_in=[
                        {"port": "loco_state", "topic": canvas_wire.STATE_TOPIC},
                        {"port": "lidar", "topic": canvas_wire.LIDAR_TOPIC},
                        {"port": "goal_pose", "topic": "/ubuntu/navigation/goal_pose"},
                    ],
                    topic_out=[
                        {
                            "port": "velocity_proposal",
                            "topic": canvas_wire.PROPOSAL_TOPIC,
                            "format": "data/json",
                        }
                    ],
                ),
            ),
            "loco": (
                {"id": "mcp-driver", "name": "G1"},
                endpoint(
                    topic_in=[
                        {
                            "port": "velocity_proposal",
                            "topic": canvas_wire.PROPOSAL_TOPIC,
                        }
                    ]
                ),
            ),
        }
        layout = {
            "cards": [
                {"id": "state", "toolName": "loco_state", "x": 0, "y": 0},
                {"id": "lidar", "toolName": "lidar_cloud", "x": 0, "y": 100},
                {
                    "id": "navigation",
                    "toolName": "general_navigation",
                    "x": 100,
                    "y": 50,
                    "topicIn": [
                        {"port": "loco_state"},
                        {"port": "lidar"},
                        {"port": "goal_pose"},
                    ],
                },
                {"id": "loco", "toolName": "loco", "x": 200, "y": 50},
                {
                    "id": "goal-source",
                    "toolName": "goal_source",
                    "x": -100,
                    "y": 50,
                    "topicOut": [
                        {
                            "topic": "/planner/goal",
                            "format": "data/json",
                            "schema": canvas_wire.GOAL_SCHEMA,
                        }
                    ],
                },
            ],
            "connections": [
                {
                    "id": "goal-link",
                    "fromCardId": "goal-source",
                    "fromPortIdx": 0,
                    "toCardId": "navigation",
                    "toPortIdx": 2,
                    "fromTopic": "/planner/goal",
                }
            ],
            "execConnections": [],
        }

        migrated = canvas_wire.build_layout(layout, tools)
        canvas_wire.validate(migrated)

        navigation_cards = [
            card
            for card in migrated["cards"]
            if card.get("toolName") == "navigation2"
        ]
        self.assertEqual(len(navigation_cards), 1)
        self.assertEqual(navigation_cards[0]["id"], "navigation")
        self.assertFalse(
            any(
                card.get("toolName") == "general_navigation"
                for card in migrated["cards"]
            )
        )
        goal_links = [
            connection
            for connection in migrated["connections"]
            if connection.get("id") == "goal-link"
        ]
        self.assertEqual(len(goal_links), 1)
        self.assertEqual(goal_links[0]["toCardId"], "navigation")


if __name__ == "__main__":
    unittest.main()
