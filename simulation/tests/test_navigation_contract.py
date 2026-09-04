import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class NavigationContractTest(unittest.TestCase):
    def test_world_and_map_geometry_are_parseable(self):
        world = ET.parse(ROOT / "gazebo-nav/worlds/synthetic_room.sdf")
        sensors_system = world.find(".//world/plugin[@name='ignition::gazebo::systems::Sensors']")
        lidar = world.find(".//sensor[@name='lidar']")
        self.assertIsNotNone(sensors_system)
        self.assertEqual(sensors_system.findtext("render_engine"), "ogre2")
        self.assertIsNotNone(lidar)
        self.assertEqual(lidar.attrib["type"], "gpu_lidar")
        left_wheel = world.find(".//model[@name='planar_base']/link[@name='left_wheel']")
        right_wheel = world.find(".//model[@name='planar_base']/link[@name='right_wheel']")
        self.assertEqual(left_wheel.findtext("pose").split()[3], "-1.5707963")
        self.assertEqual(right_wheel.findtext("pose").split()[3], "-1.5707963")
        for name in ("left_wheel_joint", "right_wheel_joint"):
            joint = world.find(f".//model[@name='planar_base']/joint[@name='{name}']")
            self.assertEqual(joint.findtext("axis/xyz"), "0 0 1")
        diff_drive = world.find(".//model[@name='planar_base']/plugin[@name='ignition::gazebo::systems::DiffDrive']")
        self.assertEqual(diff_drive.findtext("left_joint"), "left_wheel_joint")
        self.assertEqual(diff_drive.findtext("right_joint"), "right_wheel_joint")
        spec = importlib.util.spec_from_file_location("generate_map", ROOT / "gazebo-nav/tools/generate_map.py")
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "map.pgm"
            subprocess.run(
                [sys.executable, str(ROOT / "gazebo-nav/tools/generate_map.py"), str(target)],
                check=True,
            )
            tokens = target.read_text().split()
        self.assertEqual(tokens[:4], ["P2", "100", "80", "255"])
        self.assertEqual(len(tokens[4:]), 8000)
        self.assertEqual(tokens[4 + 40 * 100 + 50], "254")

    def test_contract_keeps_algorithm_topics_separate_from_canvas(self):
        source = (ROOT / "gazebo-nav/phanthymotus_sim_nav/navigation_node.py").read_text()
        self.assertIn('f"{ROOT}/mapping"', source)
        self.assertIn('"/scan"', source)
        self.assertIn('"/odom"', source)
        self.assertIn('"sensor/mapping"', source)
        self.assertIn("not G1 biped locomotion", source)
        self.assertIn('create_client(GetState, "/bt_navigator/get_state")', source)
        self.assertIn("State.PRIMARY_STATE_ACTIVE", source)
        self.assertIn('result["localization_ready"]', source)
        self.assertIn('"gazebo_ground_truth_odom"', source)
        self.assertIn('"amcl_laser_scan"', source)
        self.assertIn('map_to_odom.child_frame_id = "odom"', source)
        self.assertIn('"/amcl_pose"', source)
        self.assertIn('create_client(GetState, "/amcl/get_state")', source)
        self.assertIn('"/world/synthetic_room/dynamic_pose/info"', source)
        self.assertIn('transform.child_frame_id != "planar_base"', source)

    def test_nav2_behavior_tree_plugins_cover_default_trees(self):
        params = (ROOT / "gazebo-nav/config/nav2_params.yaml").read_text()
        launch = (ROOT / "gazebo-nav/launch/gazebo_nav.launch.py").read_text()
        self.assertIn("nav2_compute_path_through_poses_action_bt_node", params)
        self.assertIn("nav2_remove_passed_goals_action_bt_node", params)
        self.assertIn("nav2_navigate_to_pose_action_bt_node", params)
        self.assertIn("max_speed_xy: 0.45", params)
        self.assertIn("xy_goal_tolerance: 0.25", params)
        self.assertIn("max_vel_y: 0.0", params)
        self.assertIn("vy_samples: 5", params)
        self.assertIn("RotateToGoal.lookahead_time: -1.0", params)
        self.assertIn("amcl:", params)
        self.assertIn("set_initial_pose: true", params)
        self.assertIn("base_frame_id: base_link", params)
        self.assertIn('executable="map_server"', launch)
        self.assertIn('"node_names": ["map_server"]', launch)
        self.assertIn('"localization_launch.py"', launch)
        self.assertIn('"navigation_launch.py"', launch)
        self.assertNotIn('"bringup_launch.py"', launch)

    def test_p4_enables_amcl_without_changing_p3_default(self):
        compose = (ROOT / "compose.p4.yaml").read_text()
        launch = (ROOT / "gazebo-nav/launch/gazebo_nav.launch.py").read_text()
        source = (ROOT / "gazebo-nav/phanthymotus_sim_nav/navigation_node.py").read_text()
        self.assertIn("LOCALIZATION_MODE: amcl", compose)
        self.assertIn('os.environ.get("LOCALIZATION_MODE", "ground_truth")', launch)
        self.assertIn('os.environ.get("LOCALIZATION_MODE", "ground_truth")', source)
        self.assertIn('if LOCALIZATION_MODE == "ground_truth"', source)
        self.assertIn('self._amcl_lifecycle["id"] == State.PRIMARY_STATE_ACTIVE', source)
        self.assertIn('not self.snapshot()["localization_ready"]', source)

    def test_p5_adds_bounded_odometry_drift_and_relocalization(self):
        compose = (ROOT / "compose.p5.yaml").read_text()
        source = (ROOT / "gazebo-nav/phanthymotus_sim_nav/navigation_node.py").read_text()
        package = (ROOT / "gazebo-nav/package.xml").read_text()
        self.assertIn("ODOMETRY_MODE: deterministic_scale", compose)
        self.assertIn('os.environ.get("ODOMETRY_MODE", "ideal")', source)
        self.assertIn('create_client(Empty, "/reinitialize_global_localization")', source)
        self.assertIn('create_publisher(Twist, "/cmd_vel", 10)', source)
        self.assertIn("RELOCALIZATION_SCAN_SECONDS", compose)
        self.assertIn("_drive_relocalization_scan", source)
        self.assertIn('"relocalize"', source)
        self.assertIn('"localization_state": "relocalizing"', source)
        self.assertIn('"odometry_drift_error_m"', source)
        self.assertIn("<exec_depend>std_srvs</exec_depend>", package)

    def test_p3_preflight_checks_live_dependencies_without_relocking_p0(self):
        source = (ROOT / "scripts/p3-remote.sh").read_text()
        preflight = source.split("preflight(){", 1)[1].split("\nbuild(){", 1)[0]
        live_check = source.split("verify_existing_stack(){", 1)[1].split("\npreflight(){", 1)[0]
        self.assertNotIn("p0-remote.sh", preflight)
        self.assertNotIn("p2-remote.sh", preflight)
        self.assertIn("verify_existing_stack", preflight)
        self.assertIn("phanthymotus-sim-p0-agent-core", live_check)
        self.assertIn("phanthymotus-sim-p0-perception", live_check)
        self.assertIn("phanthymotus-sim-p2-g1-driver", live_check)
        self.assertIn("ROS_DOMAIN_ID=83", live_check)
        self.assertIn("mujoco_g1_29dof", live_check)
        self.assertIn("PYTHONPYCACHEPREFIX=/tmp/phanthymotus-sim-p3-pycache", preflight)
        self.assertIn('mktemp /tmp/phanthymotus-sim-p3-map.XXXXXX', preflight)

    def test_p3_verification_propagates_each_critical_failure(self):
        source = (ROOT / "scripts/p3-remote.sh").read_text()
        verify = source.split("verify(){", 1)[1].split("\ndeploy_and_verify(){", 1)[0]
        probe = (ROOT / "gazebo-nav/tools/probe_drive_signs.py").read_text()
        self.assertIn("probe_drive_signs.py", verify)
        self.assertNotIn("docker restart phanthymotus-sim-p3-gazebo-nav", verify)
        self.assertIn("P3 probe pose restoration PASS", verify)
        self.assertIn('"linear.x"', probe)
        self.assertIn('"angular.z"', probe)
        self.assertIn("restore_linear_position", probe)
        self.assertIn("restore_yaw", probe)
        self.assertIn("linear pose restoration timeout", probe)
        self.assertIn("yaw restoration timeout", probe)
        self.assertIn("cmd_vel sign mismatch", probe)
        self.assertIn("linear_ground_truth", probe)
        self.assertIn("angular_ground_truth", probe)
        self.assertIn('"/world/synthetic_room/dynamic_pose/info"', probe)
        self.assertIn('docker cp "$RUNTIME_ROOT/scripts/p3_acceptance.py"', verify)
        self.assertIn("/tmp/p3_acceptance.py || return $?", verify)
        self.assertIn("Gazebo Navigation isolation PASS", verify)
        self.assertGreaterEqual(verify.count("|| return $?"), 3)

    def test_p4_verification_rejects_goals_without_amcl_and_recovers(self):
        source = (ROOT / "scripts/p3-remote.sh").read_text()
        wrapper = (ROOT / "scripts/p4-remote.sh").read_text()
        verify = source.split("verify(){", 1)[1].split("\ndeploy_and_verify(){", 1)[0]
        self.assertIn("SIM_STAGE=p4", wrapper)
        self.assertIn("EXPECTED_LOCALIZATION_MODE=amcl_laser_scan", source)
        self.assertIn("ros2 lifecycle set /amcl deactivate", verify)
        self.assertIn("p4_localization_check.py unavailable", verify)
        self.assertIn("ros2 lifecycle set /amcl activate", verify)
        self.assertIn("p4_localization_check.py ready", verify)
        deactivate = verify.index("ros2 lifecycle set /amcl deactivate")
        unavailable = verify.index("p4_localization_check.py unavailable")
        activate = verify.index("ros2 lifecycle set /amcl activate")
        ready = verify.index("p4_localization_check.py ready")
        self.assertLess(deactivate, unavailable)
        self.assertLess(unavailable, activate)
        self.assertLess(activate, ready)

    def test_p5_verification_teleports_then_recovers_before_navigation(self):
        source = (ROOT / "scripts/p3-remote.sh").read_text()
        wrapper = (ROOT / "scripts/p5-remote.sh").read_text()
        check = (ROOT / "scripts/p5_localization_check.py").read_text()
        verify = source.split("verify(){", 1)[1].split("\ndeploy_and_verify(){", 1)[0]
        self.assertIn("SIM_STAGE=p5", wrapper)
        self.assertIn("ODOMETRY_MODE: deterministic_scale", (ROOT / "compose.p5.yaml").read_text())
        self.assertIn("/world/synthetic_room/set_pose", verify)
        self.assertIn("position {x: 0.0 y: 2.5", verify)
        self.assertIn("p5_localization_check.py", verify)
        self.assertLess(verify.index("/world/synthetic_room/set_pose"), verify.index("p5_localization_check.py"))
        self.assertIn('call("relocalize")', check)
        self.assertIn("scan_seen", check)
        self.assertIn("max_scan_rotation > 2.5", check)
        self.assertIn("navigation_not_ready", check)
        self.assertIn('navigation"]["state"] == "succeeded"', check)

    def test_failed_navigation_deploy_restores_previous_mode_and_image(self):
        source = (ROOT / "scripts/p3-remote.sh").read_text()
        deploy = source.split("deploy_and_verify(){", 1)[1].split("\ncase ", 1)[0]
        self.assertIn('previous_image="$(docker inspect', deploy)
        self.assertIn('previous_mode="$(docker inspect', deploy)
        self.assertIn('previous_odometry_mode="$(docker inspect', deploy)
        self.assertIn('[[ "$previous_mode" == amcl ]]', deploy)
        self.assertIn('[[ "$previous_odometry_mode" == deterministic_scale ]]', deploy)
        self.assertIn('GAZEBO_NAV_IMAGE="$previous_image" docker compose', deploy)
        self.assertIn("restored Gazebo Navigation", deploy)
        self.assertIn("ROLLBACK FAILED", deploy)


if __name__ == "__main__": unittest.main()
