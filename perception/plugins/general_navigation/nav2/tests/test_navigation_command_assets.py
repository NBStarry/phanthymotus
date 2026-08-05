import ast
import unittest
from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "g1_nav2"


class NavigationCommandAssetsTest(unittest.TestCase):
    def test_command_bridge_is_installed_and_launched(self):
        setup_text = (PACKAGE / "setup.py").read_text()
        launch_text = (PACKAGE / "launch" / "g1_nav2.launch.py").read_text()
        smoke_text = (ROOT / "scripts" / "smoke-test.sh").read_text()
        audit_text = (ROOT / "scripts" / "audit-shadow.sh").read_text()

        self.assertIn("navigation_command_bridge", setup_text)
        self.assertIn("runtime_supervisor", setup_text)
        self.assertIn('executable="navigation_command_bridge"', launch_text)
        self.assertIn('"enforce_shadow_isolation": True', launch_text)
        self.assertIn('"supported_mode": 0', launch_text)
        self.assertIn('"max_shadow_speed": 0.15', launch_text)
        self.assertIn('"proposal_ttl_ms": 250', launch_text)
        self.assertIn("/ubuntu/navigation/nav2/velocity_proposal", launch_text)
        self.assertIn('"runtime_mode": mode', launch_text)
        self.assertIn('"startup_map_name": map_name', launch_text)
        self.assertIn('"runtime_switch_topic"', launch_text)
        self.assertIn("/ubuntu/navigation/nav2/status", smoke_text)
        self.assertIn("/ubuntu/navigation/nav2/status", audit_text)
        self.assertIn("/ubuntu/navigation/nav2/velocity_proposal", audit_text)
        self.assertIn('"physical_execution":false', smoke_text)
        self.assertIn('"physical_execution\\\":false', audit_text)
        self.assertIn("navigation_command_probe.py", smoke_text)
        self.assertIn("G1_NAV2_COMMAND_BRIDGE=PASS", smoke_text)
        self.assertIn("G1_NAV2_N5_PROPOSAL=PASS", smoke_text)

    def test_command_bridge_has_required_runtime_dependencies(self):
        document = ElementTree.parse(PACKAGE / "package.xml")
        dependencies = {
            element.text for element in document.findall("exec_depend")
        }

        self.assertIn("action_msgs", dependencies)
        self.assertIn("nav2_msgs", dependencies)
        self.assertIn("lifecycle_msgs", dependencies)
        self.assertIn("std_msgs", dependencies)
        self.assertIn("rclpy", dependencies)
        self.assertIn("slam_toolbox", dependencies)
        self.assertIn("tf2_ros", dependencies)

    def test_bridge_source_is_valid_and_contains_shadow_gates(self):
        source_path = PACKAGE / "g1_nav2" / "navigation_command_node.py"
        source = source_path.read_text()

        ast.parse(source, filename=str(source_path))
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")', source)
        self.assertIn("get_subscriptions_info_by_topic(self._shadow_topic)", source)
        self.assertIn("self._publish_velocity_proposal", source)
        self.assertIn("foreign_subscribers", source)
        self.assertIn("/slam_toolbox/save_map", source)
        self.assertIn("/slam_toolbox/serialize_map", source)
        self.assertIn("/map_server/load_map", source)
        self.assertIn("self._map_store.finalize_mapping", source)
        self.assertIn("self._request_runtime_switch", source)
        self.assertIn('"retry_action_after_switch": True', source)
        self.assertNotIn("n3_not_ready", source)
        self.assertIn("evaluate_readiness", source)
        self.assertIn("self._require_navigation_ready", source)
        self.assertNotIn('"n3_ready": True', source)
        self.assertIn("cancel_terminal_unconfirmed", source)
        self.assertIn("self._state_changed.notify_all()", source)
        self.assertLess(source.index("save_response ="), source.index("pause_response ="))
        self.assertLess(
            source.index("pause_response ="), source.index("serialize_response =")
        )
        self.assertIn('"physical_execution": False', source)
        self.assertNotIn("unitree_sdk", source)
        self.assertNotIn("SmartMotion", source)

    def test_python_and_ros_package_versions_match(self):
        setup_tree = ast.parse((PACKAGE / "setup.py").read_text())
        setup_version = next(
            keyword.value.value
            for node in ast.walk(setup_tree)
            if isinstance(node, ast.Call)
            for keyword in node.keywords
            if keyword.arg == "version" and isinstance(keyword.value, ast.Constant)
        )
        package_version = ElementTree.parse(PACKAGE / "package.xml").findtext("version")

        self.assertEqual(setup_version, package_version)
        self.assertEqual(package_version, "0.5.0")

    def test_n5_protocol_is_ros_independent_and_driver_owned(self):
        source_path = PACKAGE / "g1_nav2" / "execution_protocol.py"
        source = source_path.read_text()

        ast.parse(source, filename=str(source_path))
        self.assertIn("class ExecutorGate", source)
        self.assertIn("class VelocityProposal", source)
        self.assertIn("proposal_ttl_expired", source)
        self.assertIn("sequence_replay", source)
        self.assertIn("heartbeat_timeout", source)
        self.assertIn("ESTOPPED", source)
        self.assertNotIn("import rclpy", source)
        self.assertNotIn("unitree_sdk", source)


if __name__ == "__main__":
    unittest.main()
