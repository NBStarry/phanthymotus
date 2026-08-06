from pathlib import Path
import unittest

import yaml


NAV2_DIR = Path(__file__).resolve().parents[1]


class Nav2AssetsTest(unittest.TestCase):
    def test_yaml_configs_parse(self):
        for path in (
            NAV2_DIR / "g1_nav2/config/nav2_params.yaml",
            NAV2_DIR / "g1_nav2/config/slam_toolbox.yaml",
            NAV2_DIR / "compose.nav2-shadow.yml",
        ):
            with self.subTest(path=path):
                self.assertIsInstance(
                    yaml.safe_load(path.read_text(encoding="utf-8")), dict
                )

    def test_runtime_is_nav2_only_and_shadow_output_isolated(self):
        runtime = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (
                NAV2_DIR / "g1_nav2/launch/g1_nav2.launch.py",
                NAV2_DIR / "g1_nav2/config/nav2_params.yaml",
                NAV2_DIR / "compose.nav2-shadow.yml",
            )
        ).lower()
        self.assertNotIn("fast-livo", runtime)
        self.assertNotIn("ego_planner", runtime)
        self.assertIn("/ubuntu/navigation/nav2/cmd_vel_raw", runtime)
        self.assertIn("/ubuntu/navigation/nav2/cmd_vel_shadow", runtime)
        self.assertIn("/ubuntu/navigation/nav2/velocity_proposal", runtime)
        self.assertIn('setremap(src="cmd_vel_smoothed"', runtime)
        self.assertIn("/ubuntu/loco/state", runtime)
        self.assertIn("/ubuntu/lidar/cloud", runtime)
        self.assertIn("canvas_pointcloud_bridge", runtime)
        self.assertIn("/ubuntu/navigation/nav2/cloud", runtime)

    def test_dwb_is_holonomic_and_speed_limited(self):
        params = yaml.safe_load(
            (NAV2_DIR / "g1_nav2/config/nav2_params.yaml").read_text()
        )
        follow = params["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(follow["plugin"], "dwb_core::DWBLocalPlanner")
        self.assertGreater(follow["max_vel_y"], 0.0)
        self.assertLess(follow["min_vel_y"], 0.0)
        self.assertLessEqual(follow["max_vel_x"], 0.2)

    def test_localization_uses_the_frozen_mapping_origin_policy(self):
        params = yaml.safe_load(
            (NAV2_DIR / "g1_nav2/config/nav2_params.yaml").read_text()
        )
        amcl = params["amcl"]["ros__parameters"]
        self.assertTrue(amcl["set_initial_pose"])
        self.assertEqual(
            amcl["initial_pose"],
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        )

    def test_container_is_unprivileged_and_uses_mirrors(self):
        compose = yaml.safe_load(
            (NAV2_DIR / "compose.nav2-shadow.yml").read_text()
        )["services"]["nav2-shadow"]
        self.assertTrue(compose["read_only"])
        self.assertEqual(compose["cap_drop"], ["ALL"])
        self.assertNotIn("privileged", compose)
        self.assertIn("UDPv4", compose["environment"]["FASTDDS_BUILTIN_TRANSPORTS"])
        dockerfile = (NAV2_DIR / "Dockerfile").read_text()
        self.assertIn("mirrors.tuna.tsinghua.edu.cn", dockerfile)
        self.assertIn("ros:humble@sha256:", dockerfile)
        self.assertIn("ros-humble-navigation2=${ROS_NAVIGATION2_VERSION}", dockerfile)
        self.assertIn("python3-pytest=${PYTHON_PYTEST_VERSION}", dockerfile)
        self.assertTrue((NAV2_DIR / "THIRD_PARTY.md").is_file())

    def test_readiness_script_contains_no_robot_writes(self):
        forbidden = (
            "docker restart",
            "docker run",
            "docker compose up",
            "systemctl restart",
            "systemctl stop",
            "systemctl start",
            "rsync",
            "scp ",
            "rm -",
            "mv ",
        )
        for script_path in (
            NAV2_DIR / "scripts/g1-readiness.sh",
            NAV2_DIR / "scripts/audit-shadow.sh",
            NAV2_DIR / "scripts/loco-integration-readiness.sh",
        ):
            script = script_path.read_text()
            for token in forbidden:
                with self.subTest(script=script_path.name, token=token):
                    self.assertNotIn(token, script)

    def test_owner_deploy_requires_explicit_authorization_and_live_odom(self):
        script = (NAV2_DIR / "scripts/owner-deploy-shadow.sh").read_text()
        self.assertIn("I_AM_G1_OWNER", script)
        self.assertIn("PREFLIGHT_ONLY", script)
        self.assertIn("/ubuntu/loco/state produced no UDPv4 sample", script)
        self.assertIn("FASTDDS_BUILTIN_TRANSPORTS=UDPv4", script)
        self.assertIn("docker save", script)
        self.assertIn("--cap-drop ALL", script)
        self.assertNotIn("embodied-unitree-g1", script)
        self.assertIn("audit-shadow.sh", script)
        self.assertIn("no Driver executor is connected", script)

        audit = (NAV2_DIR / "scripts/audit-shadow.sh").read_text()
        self.assertIn("/ubuntu/navigation/nav2/cmd_vel_shadow", audit)
        self.assertIn("G1_NAV2_N5_PROPOSAL_ISOLATED=PASS", audit)
        self.assertIn("ros2 topic info --verbose", audit)
        self.assertIn("geometry_msgs/msg/Twist", audit)
        self.assertIn("g1_nav2_navigation_command", audit)
        self.assertIn("phanthy_bus_bridge", audit)
        self.assertIn("G1_NAV2_SHADOW_BUS_OBSERVERS", audit)
        self.assertIn("G1_NAV2_PROPOSAL_BUS_OBSERVERS", audit)
        self.assertIn("PROPOSAL_DRIVER_NODE", audit)
        self.assertIn("G1_NAV2_PROPOSAL_DRIVER_SUBSCRIBERS", audit)
        self.assertIn("PROPOSAL_DRIVER_STANDBY", audit)
        self.assertIn("expected_driver_subscribers", audit)
        self.assertIn("the expected loco proposal subscriber is present", audit)
        self.assertIn("for shadow_attempt in {1..10}", audit)
        self.assertIn("G1_NAV2_SHADOW_ENDPOINTS=waiting", audit)
        self.assertIn("twist_publishers=${shadow_twist_publishers}", audit)
        self.assertIn("G1_NAV2_SHADOW_AUDIT=PASS", audit)
        self.assertIn("native odom is not ready", audit)
        self.assertIn("no lidar scan sample", audit)
        self.assertIn(
            "LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT", audit
        )
        self.assertIn(
            "G1_NAV2_SCAN_STATUS=missing_legacy_input_allowed_for_card5_upgrade",
            audit,
        )
        self.assertIn(
            "G1_NAV2_LEGACY_RUNTIME_STATUS=degraded_source_allowed_for_card5_upgrade",
            audit,
        )
        self.assertIn("lifecycle_deadline=$((SECONDS + 60))", audit)
        self.assertIn("query_lifecycle_once", audit)
        self.assertIn('[[ "${lifecycle_state}" == *"active [3]"* ]]', audit)
        self.assertIn("for attempt in 1 2 3", audit)
        self.assertIn("active response accepted despite cli_rc", audit)
        self.assertIn("--field info /map", audit)
        self.assertNotIn("/map_metadata", audit)
        self.assertIn(
            "set -eo pipefail\n  source /opt/ros/humble/setup.bash\n  set -u",
            audit,
        )

    def test_owner_upgrade_is_guarded_and_keeps_a_rollback_container(self):
        script = (NAV2_DIR / "scripts/owner-upgrade-shadow.sh").read_text()
        self.assertIn("I_AM_G1_OWNER", script)
        self.assertIn("PREFLIGHT_ONLY", script)
        self.assertIn("G1_NAV2_EXISTING_SHADOW_ISOLATED=PASS", script)
        self.assertIn("docker save", script)
        self.assertIn("docker stop --time 10", script)
        self.assertIn("docker rename", script)
        self.assertIn("rollback_needed=1", script)
        self.assertIn("audit-shadow.sh", script)
        self.assertIn("G1_NAV2_SHADOW_UPGRADE=PASS", script)
        self.assertNotIn("docker rm ${backup_name}", script)

    def test_owner_n3_upgrade_and_acceptance_are_staged_and_guarded(self):
        upgrade = (NAV2_DIR / "scripts/owner-upgrade-n3.sh").read_text()
        acceptance = (NAV2_DIR / "scripts/owner-n3-acceptance.sh").read_text()
        probe = (NAV2_DIR / "tests/n3_owner_probe.py").read_text()
        source_lock = (NAV2_DIR / "source-lock.env").read_text()

        self.assertIn("I_AM_G1_OWNER", upgrade)
        self.assertIn("PREFLIGHT_ONLY", upgrade)
        self.assertIn("NAV2_N3_IMAGE", upgrade)
        self.assertIn("NAV2_N3_IMAGE", acceptance)
        self.assertIn(
            "NAV2_N3_IMAGE=phanthy-nav2:g1-humble-nav2card2", source_lock
        )
        self.assertIn("phanthy-nav2-shadow-card1-rollback", upgrade)
        self.assertIn("rollback_needed=1", upgrade)
        self.assertIn("G1_NAV2_N3_RUNTIME=PASS", upgrade)
        self.assertIn('--group-add "${remote_map_gid}"', upgrade)
        self.assertNotIn("embodied-unitree-g1", upgrade)

        for phase in (
            "preflight",
            "begin",
            "save",
            "localize",
            "globalize",
            "verify",
        ):
            self.assertIn(phase, acceptance)
        self.assertIn("I_AM_G1_OWNER", acceptance)
        self.assertIn("phanthy-nav2-shadow-n3-mapping-rollback", acceptance)
        self.assertIn("mode:=localization", acceptance)
        self.assertIn("--group-add ${remote_map_gid}", acceptance)
        self.assertIn("test -w /maps", acceptance)
        self.assertIn("audit-shadow.sh", acceptance)
        self.assertIn("no Driver executor is connected", acceptance)
        self.assertIn('"start_mapping"', probe)
        self.assertIn('"stop_mapping"', probe)
        self.assertIn('"load_map"', probe)
        self.assertIn('"/reinitialize_global_localization"', probe)
        self.assertIn("request_global_localization", probe)
        self.assertIn('"navigate_to_tag"', probe)

        repair = (NAV2_DIR / "scripts/owner-repair-map-access.sh").read_text()
        self.assertIn("I_AM_G1_OWNER", repair)
        self.assertIn("PREFLIGHT_ONLY", repair)
        self.assertIn("phanthy-nav2-shadow-card2-map-access-rollback", repair)
        self.assertIn('--group-add "${remote_map_gid}"', repair)
        self.assertIn("test -w /maps", repair)
        self.assertIn("G1_NAV2_MAP_ACCESS_REPAIR=PASS", repair)
        self.assertNotIn("chmod 777", repair)

    def test_owner_n5_protocol_upgrade_is_guarded_and_preserves_runtime(self):
        script = (
            NAV2_DIR / "scripts/owner-upgrade-n5-protocol.sh"
        ).read_text()
        source_lock = (NAV2_DIR / "source-lock.env").read_text()
        compose = (NAV2_DIR / "compose.nav2-shadow.yml").read_text()

        self.assertIn(
            "NAV2_N5_IMAGE=phanthy-nav2:g1-humble-nav2card3", source_lock
        )
        self.assertIn("${NAV2_IMAGE:-phanthy-nav2:g1-humble-nav2card5}", compose)
        self.assertIn("I_AM_G1_OWNER", script)
        self.assertIn("PREFLIGHT_ONLY", script)
        self.assertIn("expected_current=", script)
        self.assertIn("NAV2_N3_IMAGE", script)
        self.assertIn("phanthy-nav2-shadow-card2-n5-rollback", script)
        self.assertIn("runtime_mode", script)
        self.assertIn("runtime_map", script)
        self.assertIn("starting|navigating|paused", script)
        self.assertIn('test -s "/maps/${runtime_map}/${filename}"', script)
        self.assertIn("docker save", script)
        self.assertIn("docker stop --time 10", script)
        self.assertIn("--group-add ${remote_map_gid}", script)
        self.assertIn("--restart no", script)
        self.assertIn("REQUIRE_N5_PROTOCOL=1", script)
        self.assertIn('"n5_protocol_ready":true', script)
        self.assertIn('"proposal_subscribers":', script)
        self.assertIn("G1_NAV2_N5_PROTOCOL_UPGRADE=PASS", script)
        self.assertIn("proposal-only and no Driver command was issued", script)
        self.assertIn("exec ros2 run g1_nav2 runtime_supervisor", script)
        self.assertIn("--env NAV2_MODE=${runtime_mode}", script)
        self.assertNotIn("embodied-unitree-g1", script)
        self.assertNotIn("/cmd_vel\n", script)

    def test_owner_runtime_switch_is_card5_only_guarded_and_rollback_capable(self):
        script = (
            NAV2_DIR / "scripts/owner-switch-runtime-mode.sh"
        ).read_text()

        self.assertIn("NAV2_TARGET_MODE", script)
        self.assertIn('"mapping"', script)
        self.assertIn('"localization"', script)
        self.assertIn("I_AM_G1_OWNER", script)
        self.assertIn("PREFLIGHT_ONLY", script)
        self.assertIn("quoted_container_format", script)
        self.assertIn('current_image}" != "${NAV2_IMAGE}', script)
        self.assertIn("starting|navigating|paused", script)
        self.assertIn("stop active mapping", script)
        self.assertIn("phanthy-nav2-shadow-rollback", script)
        self.assertIn('docker rm --force "${backup_name}"', script)
        self.assertIn("docker stop --time 10", script)
        self.assertIn("docker rename", script)
        self.assertIn("rollback_needed=1", script)
        self.assertIn("--group-add ${remote_map_gid}", script)
        self.assertIn("FASTDDS_BUILTIN_TRANSPORTS=UDPv4", script)
        self.assertIn("NAV2_LIDAR_X", script)
        self.assertIn("exec ros2 run g1_nav2 runtime_supervisor", script)
        self.assertIn("--env NAV2_MODE=${target_mode}", script)
        self.assertIn("REQUIRE_N5_PROTOCOL=1", script)
        self.assertIn("audit-shadow.sh", script)
        self.assertIn("G1_NAV2_RUNTIME_SWITCH=PASS", script)
        self.assertNotIn("embodied-unitree-g1", script)

    def test_owner_canvas_input_upgrade_targets_card4_and_preserves_card3(self):
        wrapper = (
            NAV2_DIR / "scripts/owner-upgrade-canvas-inputs.sh"
        ).read_text()
        upgrade = (
            NAV2_DIR / "scripts/owner-upgrade-n5-protocol.sh"
        ).read_text()
        source_lock = (NAV2_DIR / "source-lock.env").read_text()

        self.assertIn(
            "NAV2_CANVAS_IMAGE=phanthy-nav2:g1-humble-nav2card4", source_lock
        )
        self.assertIn("NAV2_UPGRADE_STAGE=canvas-inputs", wrapper)
        self.assertIn("phanthy-nav2-shadow-card3-canvas-inputs-rollback", upgrade)
        self.assertIn("NAV2_N5_IMAGE", upgrade)
        self.assertIn("g1_canvas_pointcloud_bridge", upgrade)
        self.assertIn("G1_NAV2_CANVAS_POINTCLOUD_ADAPTER=PASS", upgrade)
        self.assertIn("G1_NAV2_CANVAS_INPUTS_UPGRADE=PASS", upgrade)

    def test_driver_contract_compatibility_is_card5_and_uses_audited_urdf_extrinsic(self):
        source_lock = (NAV2_DIR / "source-lock.env").read_text()
        compose = (NAV2_DIR / "compose.nav2-shadow.yml").read_text()
        launch = (NAV2_DIR / "g1_nav2/launch/g1_nav2.launch.py").read_text()

        self.assertIn("NAV2_IMAGE=phanthy-nav2:g1-humble-nav2card5", source_lock)
        self.assertIn("NAV2_LIDAR_X=-0.00368", source_lock)
        self.assertIn("NAV2_LIDAR_Y=0.00003", source_lock)
        self.assertIn("NAV2_LIDAR_Z=0.46018", source_lock)
        self.assertIn("NAV2_LIDAR_PITCH=0.04014257279586953", source_lock)
        self.assertIn("driver-main-cfb8efe-g1_model.urdf", source_lock)
        for name in ("X", "Y", "Z", "ROLL", "PITCH", "YAW"):
            self.assertIn(f"NAV2_LIDAR_{name}: ${{NAV2_LIDAR_{name}:?", compose)
        self.assertNotIn('DeclareLaunchArgument("lidar_x", default_value=', launch)
        self.assertIn('DeclareLaunchArgument(\n                "lidar_x", description=', launch)

        wrapper = (NAV2_DIR / "scripts/owner-upgrade-driver-inputs.sh").read_text()
        upgrade = (NAV2_DIR / "scripts/owner-upgrade-n5-protocol.sh").read_text()
        self.assertIn("NAV2_UPGRADE_STAGE=driver-inputs", wrapper)
        self.assertIn("phanthy-nav2-shadow-rollback", upgrade)
        self.assertIn("current_image_id", upgrade)
        self.assertIn('docker rm --force "${backup_name}"', upgrade)
        self.assertIn("expected_current_alt", upgrade)
        self.assertIn("REQUIRE_DRIVER_INPUT_CONTRACT", upgrade)
        self.assertIn(
            'if [[ "${upgrade_stage}" == "driver-inputs" ]]', upgrade
        )
        self.assertIn(
            "LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT", upgrade
        )
        self.assertIn("NAV2_LIDAR_X", upgrade)
        self.assertIn("NAV2_LIDAR_SOURCE", upgrade)
        self.assertIn("G1_NAV2_DRIVER_INPUTS_UPGRADE=PASS", upgrade)

    def test_readiness_uses_released_canvas_cloud(self):
        readiness = (NAV2_DIR / "scripts/g1-readiness.sh").read_text()
        input_probe = (
            NAV2_DIR / "tests/driver_input_contract_probe.py"
        ).read_text()
        cloud_bridge = (
            NAV2_DIR / "g1_nav2/g1_nav2/canvas_pointcloud_node.py"
        ).read_text()
        self.assertIn('lidar_line="$(printf', readiness)
        self.assertIn("/ubuntu/lidar/cloud", readiness)
        self.assertIn('grep -Fq "std_msgs/msg/UInt8MultiArray"', readiness)
        self.assertIn("does not advertise the UInt8MultiArray envelope", readiness)
        self.assertIn("docker exec embodied-unitree-g1", readiness)
        self.assertIn("driver_input_contract_probe.py", readiness)
        self.assertIn("REQUIRE_DRIVER_INPUT_CONTRACT", readiness)
        self.assertGreaterEqual(
            readiness.count("FASTDDS_BUILTIN_TRANSPORTS=UDPv4"), 2
        )
        self.assertNotIn("lidar_fast_livo", readiness)
        self.assertEqual(input_probe.count("qos_profile_sensor_data"), 3)
        self.assertNotIn("ReliabilityPolicy.RELIABLE", input_probe)
        self.assertEqual(cloud_bridge.count("qos_profile_sensor_data"), 3)
        self.assertNotIn("ReliabilityPolicy.RELIABLE", cloud_bridge)

    def test_odom_health_receipt_is_faster_than_the_stale_gate(self):
        odom = (
            NAV2_DIR / "g1_nav2/g1_nav2/loco_odom_node.py"
        ).read_text()

        self.assertIn("self.create_timer(0.1, self._publish_status)", odom)
        self.assertIn('self.declare_parameter("source_timeout", 0.5)', odom)

    def test_mapping_occupancy_save_has_a_bounded_retry(self):
        command = (
            NAV2_DIR / "g1_nav2/g1_nav2/navigation_command_node.py"
        ).read_text()

        self.assertIn("for attempt in range(3):", command)
        self.assertIn("SLAM occupancy save returned result=", command)
        self.assertIn("if attempt < 2:", command)

    def test_smoke_inputs_exercise_the_canvas_cloud_decoder(self):
        inputs = (NAV2_DIR / "tests/synthetic_inputs.py").read_text()
        self.assertIn("UInt8MultiArray", inputs)
        self.assertIn("/ubuntu/lidar/cloud", inputs)
        self.assertIn('"<4sHHIIqH"', inputs)
        self.assertIn('"source_stamp_ns": time.time_ns()', inputs)
        self.assertIn('"frame_id": "odom_source"', inputs)
        self.assertNotIn("LaserScan", inputs)
        self.assertNotIn("/ubuntu/navigation/nav2/scan", inputs)

    def test_smoke_test_rejects_root_cmd_vel(self):
        script = (NAV2_DIR / "scripts/smoke-test.sh").read_text()
        self.assertIn("NAV2_IMAGE_OVERRIDE", script)
        self.assertIn("ERROR=root_cmd_vel_present", script)
        self.assertIn("/ubuntu/navigation/nav2/cmd_vel_raw", script)
        self.assertIn("/ubuntu/navigation/nav2/cmd_vel_shadow", script)
        self.assertIn("/ubuntu/navigation/nav2/velocity_proposal", script)
        self.assertIn("/ubuntu/navigation/nav2/cloud", script)
        self.assertIn("G1_NAV2_CANVAS_CLOUD_BEGIN", script)
        self.assertIn("--field info /map", script)
        self.assertIn("G1_NAV2_N5_PROPOSAL=PASS", script)
        self.assertIn('grep -Fq "G1_NAV2_N3_${phase_upper}=PASS"', script)
        self.assertIn("audit_runtime mapping", script)
        self.assertIn("audit_runtime localization", script)
        self.assertIn("N3_OWNER_PHASE=globalize", script)
        self.assertIn("n3_owner_probe.py", script)
        self.assertIn('map_name="smoke-map"', script)
        self.assertIn('start_runtime localization "/maps/${map_name}/map.yaml"', script)

    def test_owner_shadow_goal_is_guarded_and_cancelled(self):
        owner = (NAV2_DIR / "scripts/owner-shadow-goal-test.sh").read_text()
        probe = (NAV2_DIR / "scripts/shadow_goal_test.py").read_text()
        self.assertIn("I_AM_G1_OWNER", owner)
        self.assertIn("DRY_RUN", owner)
        self.assertIn("audit-shadow.sh", owner)
        self.assertIn("docker exec -i", owner)
        self.assertIn("get_subscriptions_info_by_topic", probe)
        self.assertIn("g1_nav2_navigation_command", probe)
        self.assertIn("/navigate_to_pose", probe)
        self.assertIn("cancel_goal_async", probe)
        self.assertIn("G1_NAV2_SHADOW_GOAL_TEST=PASS", probe)
        self.assertIn("no Driver executor was connected", probe)

    def test_owner_card_command_test_is_guarded_and_uses_json_bridge(self):
        owner = (NAV2_DIR / "scripts/owner-card-command-test.sh").read_text()
        probe = (NAV2_DIR / "tests/navigation_command_probe.py").read_text()
        self.assertIn("I_AM_G1_OWNER", owner)
        self.assertIn("DRY_RUN", owner)
        self.assertIn("audit-shadow.sh", owner)
        self.assertIn("docker exec -i", owner)
        self.assertIn("G1_NAV2_G1_CARD_COMMAND_TEST=PASS", owner)
        self.assertIn("/ubuntu/navigation/nav2/command", probe)
        self.assertIn('"start_mapping"', probe)
        self.assertIn('"stop_mapping"', probe)
        self.assertIn('"load_map"', probe)
        self.assertIn('"navigate_to_tag"', probe)
        self.assertIn('"stop_nav"', probe)
        self.assertIn("G1_NAV2_COMMAND_BRIDGE=PASS", probe)

    def test_owner_n5_acceptance_uses_mcp_and_stops_the_shadow_goal(self):
        owner = (
            NAV2_DIR / "scripts/owner-n5-shadow-acceptance.sh"
        ).read_text()
        probe = (NAV2_DIR / "tests/n5_mcp_acceptance_probe.py").read_text()
        self.assertIn("I_AM_G1_OWNER", owner)
        self.assertIn("DRY_RUN", owner)
        self.assertIn("REQUIRE_N5_PROTOCOL=1", owner)
        self.assertIn("/tests/mcp_probe.py", owner)
        self.assertIn("source /nav2_ws/install/setup.bash", owner)
        self.assertIn("N5_DRY_RUN", owner)
        self.assertIn("n5_mcp_acceptance_probe.py", owner)
        self.assertIn("GENERAL_NAVIGATION_N5_G1_SHADOW_ACCEPTANCE=PASS", owner)
        self.assertIn('"name": "navigation2"', probe)
        self.assertIn('"action": "navigate_to_pose"', probe)
        self.assertIn('"action": "stop_nav"', probe)
        self.assertIn("select_goal", probe)
        self.assertIn("GENERAL_NAVIGATION_N5_GOAL_DRY_RUN=PASS", probe)
        self.assertIn("require_nonzero=True", probe)
        self.assertIn("VelocityProposal.from_payload", probe)
        self.assertIn("after_sequence=motion.sequence", probe)
        self.assertIn("terminal.velocity.is_zero", probe)
        self.assertIn("GENERAL_NAVIGATION_N5_MCP_ACCEPTANCE=PASS", probe)

    def test_loco_link_readiness_verifies_registry_canvas_and_ros_endpoint(self):
        readiness = (
            NAV2_DIR / "scripts/loco-integration-readiness.sh"
        ).read_text()

        self.assertIn("LOCO_PROPOSAL_NODE", readiness)
        self.assertIn("PROPOSAL_DRIVER_NODE", readiness)
        self.assertIn("PROPOSAL_DRIVER_STANDBY=1", readiness)
        self.assertIn("EXPECT_CANVAS_WIRED=1", readiness)
        self.assertIn('"${perception_container}" python3 -', readiness)
        self.assertIn('${deploy_dir}/tests/mcp_probe.py', readiness)
        self.assertIn("core_registry_probe.py", readiness)
        self.assertIn("loco_registry_probe.py", readiness)
        self.assertIn("loco_runtime_probe.py", readiness)
        self.assertIn("REQUIRE_DRIVER_INPUT_CONTRACT=1", readiness)
        self.assertNotIn("REQUIRE_DRIVER_AUTHORIZATION", readiness)
        self.assertIn("GENERAL_NAVIGATION_LOCO_LINK_PREFLIGHT=PASS", readiness)
        self.assertNotIn("I_AM_G1_OWNER", readiness)

    def test_loco_card_acceptance_is_owner_gated_and_uses_agent_core(self):
        owner = (
            NAV2_DIR / "scripts/owner-loco-card-acceptance.sh"
        ).read_text()
        probe = (NAV2_DIR / "tests/n5_mcp_acceptance_probe.py").read_text()

        self.assertIn("STAGE must be preflight or move", owner)
        self.assertIn("I_AM_G1_OWNER", owner)
        self.assertIn("I_HAVE_G1_REMOTE", owner)
        self.assertIn("N5_COMMAND_BACKEND=agent_core", owner)
        self.assertIn("AGENT_CORE_ACCESS_TOKEN", owner)
        self.assertIn("loco-integration-readiness.sh", owner)
        self.assertIn("N5_PHYSICAL_E2E", owner)
        self.assertIn("GENERAL_NAVIGATION_LOCO_CARD_ACCEPTANCE=PASS", owner)
        self.assertNotIn("SmartMotion", owner)
        self.assertNotIn("StopMove", owner)
        self.assertNotIn("SetVelocity", owner)

        self.assertIn('"name": "navigation2"', probe)
        self.assertIn('"action": "navigate_to_pose"', probe)
        self.assertIn('"action": "wait_navigation_done"', probe)
        self.assertIn("require_framework_canvas", probe)
        self.assertIn("Agent Core project is not running", probe)
        self.assertIn("navigation2 -> loco", probe)
        self.assertIn("wait_for_physical_arrival", probe)
        self.assertIn("max_planar_m", probe)
        self.assertIn("final_target_distance_m", probe)
        self.assertIn("MAX_GOAL_DISTANCE_M = 5.0", probe)
        self.assertIn("ComputePathToPose", probe)
        self.assertIn('"/compute_path_to_pose"', probe)
        self.assertIn("node.compute_path_length", probe)
        self.assertNotIn("CANDIDATE_SAMPLE_SPACING_M", probe)
        self.assertNotIn("score_candidate", probe)
        self.assertIn('"goal_yaw": round(target_yaw, 3)', probe)
        self.assertIn("GENERAL_NAVIGATION_LOCO_E2E_ACCEPTANCE=PASS", probe)
        self.assertNotIn("unitree_sdk", probe)
        self.assertNotIn("SmartMotion", probe)

    def test_n3_map_store_is_persistent_and_path_safe(self):
        source = (
            NAV2_DIR / "g1_nav2/g1_nav2/map_store.py"
        ).read_text(encoding="utf-8")
        compose = yaml.safe_load(
            (NAV2_DIR / "compose.nav2-shadow.yml").read_text()
        )["services"]["nav2-shadow"]

        self.assertIn("os.replace", source)
        self.assertIn("MAP_FILES", source)
        self.assertIn("map.posegraph", source)
        self.assertIn("unsafe_map_path", source)
        self.assertIn("${NAV2_MAP_DIR:-./maps}:/maps", compose["volumes"])
        self.assertEqual(compose["group_add"], ["${NAV2_MAP_GID:-1000}"])
        self.assertIn("NAV2_MAP_NAME", compose["environment"])


if __name__ == "__main__":
    unittest.main()
