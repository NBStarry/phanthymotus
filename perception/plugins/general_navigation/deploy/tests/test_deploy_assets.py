import pathlib
import unittest


DEPLOY_DIR = pathlib.Path(__file__).resolve().parents[1]
PERCEPTION_DIR = DEPLOY_DIR.parents[2]
FORMAL_DEPLOY_DIR = PERCEPTION_DIR / "deploy"


def read_env(path: pathlib.Path) -> dict[str, str]:
    result = {}
    for line in path.read_text().splitlines():
        if line and not line.startswith("#"):
            key, value = line.split("=", 1)
            result[key] = value
    return result


def top_level_keys(source: str) -> list[str]:
    return [
        line[:-1]
        for line in source.splitlines()
        if line and not line[0].isspace() and line.endswith(":")
    ]


class NavigationPerceptionDeployAssetsTest(unittest.TestCase):
    def test_runtime_config_is_navigation_only_and_shadow_only(self):
        config = (DEPLOY_DIR / "config.yaml").read_text()

        self.assertIn("ws_enabled: false\n", config)
        self.assertIn("  navigation2:\n", config)
        self.assertNotIn("  asr:\n", config)
        self.assertNotIn("  tts:\n", config)
        self.assertIn("    enabled: true\n", config)
        self.assertIn("    namespace: ubuntu\n", config)
        self.assertIn("    backend: ros_topic\n", config)
        self.assertIn("    shadow_only: true\n", config)
        self.assertIn("    request_timeout_sec: 30.0\n", config)
        self.assertIn("    runtime_switch_timeout_sec: 120.0\n", config)

    def test_service_fragment_has_no_robot_or_host_privilege(self):
        service = (DEPLOY_DIR / "service.yml").read_text()

        self.assertEqual(top_level_keys(service), ["perception", "nav2"])
        self.assertIn("  image: __IMAGE__\n", service)
        self.assertIn("  container_name: embodied-perception\n", service)
        self.assertIn("  depends_on:\n    - nav2\n", service)
        self.assertIn("  read_only: false\n", service)
        self.assertIn("  cap_drop:\n    - ALL\n", service)
        self.assertIn("  restart: unless-stopped\n", service)
        self.assertNotIn("  privileged:", service)
        self.assertNotIn("  devices:", service)
        self.assertNotIn("  pid:", service)
        self.assertNotIn("  ipc:", service)
        self.assertIn("    - FASTDDS_BUILTIN_TRANSPORTS=UDPv4\n", service)
        self.assertIn(
            "    - CONFIG_PATH=/config/general-navigation.yaml\n", service
        )
        self.assertIn("  image: phanthy-nav2:g1-humble-nav2card5\n", service)
        self.assertIn("  container_name: phanthy-nav2-shadow\n", service)
        self.assertIn("  read_only: true\n", service)
        self.assertIn("    - /home/unitree/phanthy-nav2/maps:/maps\n", service)
        self.assertIn("    - NAV2_MODE=mapping\n", service)

    def test_formal_perception_compose_owns_nav2_companion(self):
        service = (FORMAL_DEPLOY_DIR / "service.yml").read_text()

        self.assertEqual(top_level_keys(service), ["perception", "nav2"])
        self.assertIn("  depends_on:\n    - nav2\n", service)
        self.assertIn("    - FASTDDS_BUILTIN_TRANSPORTS=UDPv4\n", service)
        self.assertIn("  image: phanthy-nav2:g1-humble-nav2card5\n", service)
        self.assertIn("  container_name: phanthy-nav2-shadow\n", service)
        self.assertIn("    - /home/unitree/phanthy-nav2/maps:/maps\n", service)

        config = (PERCEPTION_DIR / "config.yaml").read_text()
        self.assertIn("  navigation2:\n", config)
        self.assertIn("    enabled: true\n", config)
        self.assertIn('    namespace: "ubuntu"\n', config)

        jetson_dockerfile = (PERCEPTION_DIR / "Dockerfile.jetson").read_text()
        self.assertIn("COPY perception/deploy/     /deploy/", jetson_dockerfile)

    def test_core_deploy_starts_declared_companions(self):
        source = (PERCEPTION_DIR.parent / "agent-core/src/api/drivers.py").read_text()

        self.assertIn("Compose owns", source)
        self.assertNotIn(
            "'up', '-d', '--no-deps', '--force-recreate', service_name", source
        )
        self.assertIn(
            "'up', '-d', '--force-recreate', service_name", source
        )

    def test_dockerfile_is_pinned_and_installs_no_extra_packages(self):
        dockerfile = (DEPLOY_DIR / "Dockerfile").read_text()
        locked = read_env(DEPLOY_DIR / "source-lock.env")

        self.assertIn(locked["ROS_BASE_IMAGE"], dockerfile)
        self.assertNotIn("apt-get", dockerfile)
        self.assertNotIn("pip install", dockerfile)
        self.assertNotIn("COPY perception/plugins/ ", dockerfile)
        self.assertIn("general_navigation/plugin.py", dockerfile)
        self.assertIn("/deploy/service.yml", dockerfile)

    def test_perception_entrypoint_keeps_ws_default_and_allows_opt_out(self):
        source = (PERCEPTION_DIR / "main.py").read_text()

        self.assertIn('cfg.get("ws_enabled", True) is True', source)
        self.assertIn("if ws_enabled:", source)
        self.assertIn("WebSocket ASR server disabled by config", source)

    def test_owner_deploy_is_guarded_audited_and_rollback_capable(self):
        owner = (DEPLOY_DIR / "scripts" / "owner-deploy-g1.sh").read_text()

        gate = owner.index('I_AM_G1_OWNER:-0')
        local_image_read = owner.index('docker image inspect "${GENERAL_NAVIGATION_IMAGE}"')
        remote_image_write = owner.index('docker save "${GENERAL_NAVIGATION_IMAGE}"')
        preflight_exit = owner.index('GENERAL_NAVIGATION_G1_PREFLIGHT=PASS')
        self.assertLess(gate, local_image_read)
        self.assertLess(preflight_exit, remote_image_write)
        self.assertIn('G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"', owner)
        self.assertIn('cp -p "${compose_file}" "${backup_file}"', owner)
        self.assertIn(
            "docker exec ${core_container} cp -p ${backup_file} ${compose_file}",
            owner,
        )
        self.assertIn("os.chown(t,st.st_uid,st.st_gid)", owner)
        self.assertIn("s['perception']['read_only']=False", owner)
        self.assertIn("compose ownership changed", owner)
        self.assertIn("deployment failed; restoring compose backup", owner)
        self.assertIn("MCP_STARTUP_TIMEOUT=30", owner)
        self.assertIn("EXPECT_BRIDGE_SUBSCRIBER=1", owner)
        self.assertIn("core_registry_probe.py", owner)

    def test_mcp_probe_has_bounded_startup_retry(self):
        probe = (DEPLOY_DIR / "tests" / "mcp_probe.py").read_text()

        self.assertIn('MCP_STARTUP_TIMEOUT", "0"', probe)
        self.assertIn("0.0 <= startup_timeout <= 60.0", probe)
        self.assertIn("except OSError:", probe)

    def test_mcp_probe_allows_other_perception_tools_to_coexist(self):
        probe = (DEPLOY_DIR / "tests" / "mcp_probe.py").read_text()

        self.assertIn(
            'tool.get("name") == "navigation2"', probe
        )
        self.assertIn("assert len(matches) == 1", probe)
        self.assertNotIn("assert len(tools) == 1", probe)

    def test_loco_registry_probe_freezes_existing_actuator_input(self):
        probe = (DEPLOY_DIR / "tests" / "loco_registry_probe.py").read_text()

        self.assertIn('tool.get("name") == "loco"', probe)
        self.assertIn('tool.get("type") == "actuator"', probe)
        self.assertIn('entry.get("port") == "velocity_proposal"', probe)
        self.assertIn("/ubuntu/navigation/nav2/velocity_proposal", probe)
        self.assertIn("std_msgs/msg/String", probe)
        self.assertIn("phanthy.navigation.velocity_proposal.v1", probe)
        self.assertNotIn("x-navigation-execution", probe)
        self.assertIn("GENERAL_NAVIGATION_LOCO_REGISTRY=PASS", probe)

    def test_loco_runtime_probe_requires_released_driver_standby(self):
        probe = (DEPLOY_DIR / "tests" / "loco_runtime_probe.py").read_text()

        self.assertIn('info.get("state") == "ready"', probe)
        self.assertIn('info.get("connected") is False', probe)
        self.assertIn('info.get("armed") is False', probe)
        self.assertIn('info.get("expected_nav_id") is None', probe)
        self.assertIn("GENERAL_NAVIGATION_DRIVER_RUNTIME_STANDBY=PASS", probe)
        self.assertIn("refusing non-local Driver MCP URL", probe)

    def test_owner_upgrade_is_guarded_and_preserves_compose_metadata(self):
        owner = (DEPLOY_DIR / "scripts/owner-upgrade-g1.sh").read_text()
        source_lock = read_env(DEPLOY_DIR / "source-lock.env")

        gate = owner.index("I_AM_G1_OWNER:-0")
        preflight = owner.index("GENERAL_NAVIGATION_G1_UPGRADE_PREFLIGHT=PASS")
        image_write = owner.index('docker save "${GENERAL_NAVIGATION_IMAGE}"')
        self.assertLess(gate, preflight)
        self.assertLess(preflight, image_write)
        self.assertEqual(
            source_lock["GENERAL_NAVIGATION_IMAGE"],
            "phanthy-perception:g1-general-navigation5",
        )
        self.assertIn("before-general-navigation5", owner)
        self.assertIn("phanthy-perception:g1-general-navigation4", owner)
        self.assertIn("owner must start Agent Core and rerun preflight", owner)
        self.assertNotIn("docker start ${core_container}", owner)
        self.assertIn("os.chown(t,st.st_uid,st.st_gid)", owner)
        self.assertIn("s['perception']['read_only']=False", owner)
        self.assertIn('"${current_read_only}" == "false"', owner)
        self.assertIn("compose ownership changed", owner)
        self.assertIn("upgrade failed; restoring compose backup", owner)
        self.assertIn('compose_service_state="orphan"', owner)
        self.assertIn("refusing unknown orphan Perception container", owner)
        self.assertIn("com.docker.compose.project.config_files", owner)
        self.assertIn('candidate_fragment="/opt/phanthy-motus/', owner)
        self.assertIn("docker rename", owner)
        self.assertIn('"${perception_container}" "${rollback_container}"', owner)
        self.assertIn("GENERAL_NAVIGATION_ROLLBACK_CONTAINER", owner)
        self.assertIn("MCP_STARTUP_TIMEOUT=30", owner)
        self.assertIn("EXPECT_BRIDGE_SUBSCRIBER=1", owner)
        self.assertIn("core_registry_probe.py", owner)
        self.assertIn("audit-shadow.sh", owner)
        self.assertIn("REQUIRE_N5_PROTOCOL=1", owner)
        self.assertIn("no Driver command was issued", owner)
        self.assertIn("released legacy Driver input adapter", owner)
        self.assertNotIn("timestamped Driver input contract", owner)

    def test_driver_main_upgrade_restores_the_released_actuator_fail_closed(self):
        owner = (
            DEPLOY_DIR / "scripts" / "owner-upgrade-driver-main-g1.sh"
        ).read_text()
        source_lock = read_env(DEPLOY_DIR / "source-lock.env")

        self.assertEqual(
            source_lock["GENERAL_NAVIGATION_DRIVER_IMAGE"],
            "phanthy-g1-driver:main-cfb8efe",
        )
        self.assertEqual(
            source_lock["GENERAL_NAVIGATION_DRIVER_ROLLBACK_IMAGE"],
            "bj-warehouse.tencentcloudcr.com/phanthy-motus/drivers/unitree/g1:release.260708.57ef78e",
        )
        gate = owner.index("I_AM_G1_OWNER:-0")
        preflight = owner.index("G1_DRIVER_MAIN_UPGRADE_PREFLIGHT=PASS")
        image_write = owner.index(
            'docker save "${GENERAL_NAVIGATION_DRIVER_IMAGE}"'
        )
        self.assertLess(gate, preflight)
        self.assertLess(preflight, image_write)
        self.assertIn("project_stopped_probe.py", owner)
        self.assertIn("remote_container_inspect()", owner)
        self.assertIn("printf -v quoted_format '%q'", owner)
        self.assertIn("printf -v quoted_container '%q'", owner)
        self.assertIn(
            "'{{.State.Running}}|{{.Config.Image}}'", owner
        )
        self.assertIn("velocity_proposal_allowed_fsm_ids", owner)
        self.assertIn("hostname':'ubuntu'", owner)
        self.assertIn("s['unitree-g1']", owner)
        self.assertIn("loco_registry_probe.py", owner)
        self.assertIn("loco_runtime_probe.py", owner)
        self.assertIn("REQUIRE_DRIVER_INPUT_CONTRACT=1", owner)
        self.assertIn("UNARMED", owner)
        self.assertIn("restoring prior stopped Driver", owner)
        self.assertIn("driver-main-rollback.tmp", owner)
        self.assertIn("quoted_restore_program", owner)
        self.assertIn(
            '"${compose_image}" != "${GENERAL_NAVIGATION_DRIVER_ROLLBACK_IMAGE}"',
            owner,
        )
        self.assertNotIn("docker rename", owner)

    def test_canvas_wire_is_owner_gated_and_preserves_unrelated_cards(self):
        owner = (DEPLOY_DIR / "scripts" / "owner-wire-canvas-g1.sh").read_text()
        wire = (DEPLOY_DIR / "tests" / "canvas_wire.py").read_text()

        self.assertIn("STAGE must be preflight or wire", owner)
        self.assertIn("I_AM_G1_OWNER", owner)
        self.assertIn("CANVAS_APPLY", owner)
        self.assertIn("project remains stopped", owner)
        self.assertIn("loco_state", wire)
        self.assertIn("lidar_cloud", wire)
        self.assertIn('NAVIGATION_TOOL = "navigation2"', wire)
        self.assertIn('LEGACY_NAVIGATION_TOOL = "general_navigation"', wire)
        self.assertIn("velocity_proposal", wire)
        self.assertIn("goal_pose", wire)
        self.assertIn("phanthy.navigation.goal.v1", wire)
        self.assertIn("preserved_goal_connections", wire)
        self.assertIn("preserved_unrelated_cards", wire)
        self.assertIn("canvas/claim-edit", wire)
        self.assertIn("navigation-canvas-backups", wire)
        self.assertIn("GENERAL_NAVIGATION_CANVAS_WIRE=PASS", wire)

    def test_owner_recovery_is_hash_locked_and_compose_only(self):
        recovery = (DEPLOY_DIR / "scripts" / "owner-recover-g1.sh").read_text()

        gate = recovery.index("I_AM_G1_OWNER:-0")
        restore = recovery.index('cp -p "${backup_file}" "${compose_file}"')
        preflight = recovery.index("GENERAL_NAVIGATION_RECOVERY_PREFLIGHT=PASS")
        self.assertLess(gate, restore)
        self.assertLess(preflight, restore)
        self.assertIn("RECOVERY_CURRENT_SHA256", recovery)
        self.assertIn("RECOVERY_BACKUP_SHA256", recovery)
        self.assertIn("refusing compose-only recovery", recovery)
        self.assertIn("assert set(bs)=={'agent-core'}", recovery)
        self.assertNotIn("docker compose restart", recovery)
        self.assertNotIn("docker compose up", recovery)


if __name__ == "__main__":
    unittest.main()
