import tempfile
import unittest
from pathlib import Path

from g1_nav2.runtime_process import build_launch_command


LIDAR_ENV = {
    "NAV2_LIDAR_X": "0.1",
    "NAV2_LIDAR_Y": "0.0",
    "NAV2_LIDAR_Z": "0.4",
    "NAV2_LIDAR_ROLL": "0.0",
    "NAV2_LIDAR_PITCH": "0.04",
    "NAV2_LIDAR_YAW": "0.0",
}


class RuntimeSupervisorTest(unittest.TestCase):
    def test_mapping_command_has_locked_launch_arguments(self):
        command = build_launch_command(mode="mapping", environ=LIDAR_ENV)

        self.assertEqual(command[:4], ["ros2", "launch", "g1_nav2", "g1_nav2.launch.py"])
        self.assertIn("mode:=mapping", command)
        self.assertNotIn("map:=", " ".join(command))

    def test_localization_requires_existing_saved_map(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaisesRegex(ValueError, "saved map is missing"):
                build_launch_command(
                    mode="localization",
                    map_name="office",
                    maps_root=str(root),
                    environ=LIDAR_ENV,
                )
            directory = root / "office"
            directory.mkdir()
            (directory / "map.yaml").write_text("image: map.pgm\n")

            command = build_launch_command(
                mode="localization",
                map_name="office",
                maps_root=str(root),
                environ=LIDAR_ENV,
            )

        self.assertIn("mode:=localization", command)
        self.assertIn("map_name:=office", command)
        self.assertIn(f"map:={directory / 'map.yaml'}", command)

    def test_invalid_mode_map_and_missing_extrinsic_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "mode"):
            build_launch_command(mode="unsafe", environ=LIDAR_ENV)
        with self.assertRaisesRegex(ValueError, "NAV2_LIDAR_X"):
            build_launch_command(mode="mapping", environ={})
        invalid = dict(LIDAR_ENV, NAV2_LIDAR_X="nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            build_launch_command(mode="mapping", environ=invalid)
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(ValueError, "plain name"):
                build_launch_command(
                    mode="localization",
                    map_name="../escape",
                    maps_root=temporary,
                    environ=LIDAR_ENV,
                )


if __name__ == "__main__":
    unittest.main()
