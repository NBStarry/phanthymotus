import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_renderer():
    path = ROOT / "scripts" / "render-local-services.py"
    spec = importlib.util.spec_from_file_location("render_local_services", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CoreDockerManagementContractTest(unittest.TestCase):
    def test_core_has_production_equivalent_docker_authority(self):
        compose = yaml.safe_load((ROOT / "compose.p0.yaml").read_text())
        core = compose["services"]["agent-core"]
        self.assertNotIn("privileged", core)
        self.assertIn("/var/run/docker.sock:/var/run/docker.sock", core["volumes"])
        self.assertIn("core-compose:/opt/phanthy-motus", core["volumes"])
        self.assertIn(
            "./state:/etc/phanthymotus/local-services:ro",
            core["volumes"],
        )
        self.assertEqual(core["environment"]["COMPOSE_DIR"], "/opt/phanthy-motus")
        self.assertEqual(
            core["environment"]["LOCAL_SERVICES_MANIFEST"],
            "/etc/phanthymotus/local-services/local-services.json",
        )

    def test_docker_binaries_are_downloaded_on_remote_with_mirrors(self):
        dockerfile = (ROOT / "docker" / "agent-core.Dockerfile").read_text()
        self.assertIn("ARG DOCKER_VERSION=27.5.1", dockerfile)
        self.assertIn("docker-${DOCKER_VERSION}.tgz", dockerfile)
        self.assertIn("mirrors.tuna.tsinghua.edu.cn", dockerfile)
        self.assertIn("docker-compose-linux-x86_64", dockerfile)
        self.assertIn("ghfast.top/https://github.com", dockerfile)
        self.assertNotIn("COPY artifacts/", dockerfile)

    def test_renderer_exposes_only_exact_simulation_containers(self):
        renderer = load_renderer()
        names = {item["container_name"] for item in renderer.SERVICE_SPECS}
        self.assertEqual(names, {
            "phanthymotus-sim-p0-agent-core",
            "phanthymotus-sim-p0-perception",
            "phanthymotus-sim-p2-g1-driver",
            "phanthymotus-sim-p3-gazebo-nav",
        })

        def fake_inspect(name):
            return {
                "Config": {"Image": f"local/{name}:test"},
                "State": {"Status": "running"},
            }

        with mock.patch.object(renderer, "inspect_container", side_effect=fake_inspect):
            manifest = renderer.build_manifest(now=123)
        self.assertEqual(len(manifest), 4)
        self.assertTrue(all(item["last_deploy"]["ts"] == 123 for item in manifest))

        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "state" / "local-services.json"
            renderer.write_atomic(output, manifest)
            self.assertEqual(json.loads(output.read_text()), manifest)
            self.assertEqual([path for path in output.parent.iterdir() if path != output], [])

    def test_lifecycle_refreshes_manifest_but_preflight_stays_read_only(self):
        p0 = (ROOT / "scripts" / "p0-remote.sh").read_text()
        preflight = p0.split("preflight() {", 1)[1].split("build_core_image() {", 1)[0]
        self.assertNotIn("render_local_services", preflight)
        self.assertIn("deploy-core-and-verify", p0)
        self.assertIn("render_local_services && verify", p0)

        for name in ("p1-remote.sh", "p2-remote.sh", "p3-remote.sh"):
            script = (ROOT / "scripts" / name).read_text()
            self.assertIn("refresh_local_services", script, name)
            self.assertIn("render-local-services.py", script, name)


if __name__ == "__main__":
    unittest.main()
