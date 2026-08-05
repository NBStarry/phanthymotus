import pathlib
import unittest


DEPLOY_DIR = pathlib.Path(__file__).resolve().parents[1]
CORE_DIR = DEPLOY_DIR.parent


def read_env() -> dict[str, str]:
    result = {}
    for line in (DEPLOY_DIR / "source-lock.env").read_text().splitlines():
        if line and not line.startswith("#"):
            key, value = line.split("=", 1)
            result[key] = value
    return result


class NavigationCoreDeployAssetsTest(unittest.TestCase):
    def test_build_is_arm64_overlay_on_the_released_core(self):
        locked = read_env()
        dockerfile = (DEPLOY_DIR / "Dockerfile.navigation").read_text()
        build = (DEPLOY_DIR / "scripts/build.sh").read_text()

        self.assertEqual(
            locked["AGENT_CORE_IMAGE"],
            "phanthy-motus/core:g1-general-navigation1",
        )
        self.assertEqual(locked["TARGET_PLATFORM"], "linux/arm64")
        self.assertIn("phanthy-motus/core@sha256:", locked["AGENT_CORE_BASE_IMAGE"])
        self.assertIn("FROM ${AGENT_CORE_BASE_IMAGE}", dockerfile)
        self.assertIn("src/navigation_execution.py", dockerfile)
        self.assertIn("src/topic_action_routing.py", dockerfile)
        self.assertIn("src/ros2_bridge.py", dockerfile)
        self.assertIn("web/js/canvas.js /work/web/js/canvas.js", dockerfile)
        self.assertNotIn("apt-get", dockerfile)
        self.assertIn("AGENT_CORE_BASE_IMAGE", build)
        self.assertIn("Dockerfile.navigation", build)

    def test_owner_upgrade_is_guarded_scoped_and_rollback_capable(self):
        owner = (DEPLOY_DIR / "scripts/owner-upgrade-g1.sh").read_text()

        gate = owner.index("I_AM_G1_OWNER:-0")
        preflight = owner.index("AGENT_CORE_NAVIGATION_UPGRADE_PREFLIGHT=PASS")
        write = owner.index('docker save "${AGENT_CORE_IMAGE}"')
        self.assertLess(gate, preflight)
        self.assertLess(preflight, write)
        self.assertIn("project_stopped_probe.py", owner)
        self.assertIn("runtime_probe.py", owner)
        self.assertIn("remote_container_inspect()", owner)
        self.assertIn("printf -v quoted_format '%q'", owner)
        self.assertIn(
            "'{{.State.Running}}|{{.Config.Image}}'", owner
        )
        self.assertIn("current_image_id", owner)
        self.assertIn("s['agent-core']['image']=sys.argv[2]", owner)
        self.assertIn("os.chown(t,st.st_uid,st.st_gid)", owner)
        self.assertIn("restoring compose backup", owner)
        self.assertIn("navigation-core-rollback.tmp", owner)
        self.assertIn("--mount type=bind,source=/opt/phanthy-motus", owner)
        self.assertNotIn(
            "rollback_script=\"set -e; cp -p", owner
        )
        self.assertIn("up --detach --no-deps agent-core", owner)
        self.assertNotIn("unitree-g1", owner)
        self.assertNotIn("embodied-perception", owner)

    def test_runtime_probe_requires_new_code_and_stopped_project(self):
        probe = (DEPLOY_DIR / "tests/runtime_probe.py").read_text()

        self.assertIn("call_with_execution_lease", probe)
        self.assertIn("resolve_topic_action_routes", probe)
        self.assertIn("fieldSchema.type === 'integer'", probe)
        self.assertIn("field?.style.display === 'none'", probe)
        self.assertIn('project == {"running": False}', probe)
        self.assertIn("AGENT_CORE_NAVIGATION_RUNTIME=PASS", probe)

    def test_project_gate_explains_the_manual_ui_action(self):
        probe = (DEPLOY_DIR / "tests/project_stopped_probe.py").read_text()

        self.assertIn("Agent Core canvas project is running", probe)
        self.assertIn("stop it manually in", probe)
        self.assertNotIn("assert payload", probe)

    def test_smoke_provides_ephemeral_resource_storage(self):
        smoke = (DEPLOY_DIR / "scripts/smoke-test.sh").read_text()

        self.assertIn("--read-only", smoke)
        self.assertIn("--tmpfs /work/resource:", smoke)
        self.assertIn("fieldSchema.type ===", smoke)
        self.assertIn("field?.style.display ===", smoke)
        self.assertIn("topic_action_routing", smoke)


if __name__ == "__main__":
    unittest.main()
