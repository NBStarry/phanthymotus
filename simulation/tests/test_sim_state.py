import math
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim-driver"))

from state import SimulationState  # noqa: E402


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class SimulationStateTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.state = SimulationState(clock=self.clock, seed=23)

    def test_move_is_clamped_and_integrated(self):
        result = self.state.command_move(4.0, -4.0, 9.0)
        self.assertEqual(result["velocity"], [1.0, -1.0, 2.0])
        self.clock.advance(0.1)
        snap = self.state.step()
        self.assertAlmostEqual(snap["pose"]["x"], 0.1)
        self.assertAlmostEqual(snap["pose"]["y"], -0.1)
        self.assertAlmostEqual(snap["pose"]["yaw"], 0.2)

    def test_timed_move_stops(self):
        self.state.command_move(0.5, 0.0, 0.0, duration=0.2)
        self.clock.advance(0.1)
        self.state.step()
        self.clock.advance(0.11)
        snap = self.state.step()
        self.assertEqual(snap["velocity"], {"vx": 0.0, "vy": 0.0, "vyaw": 0.0})

    def test_pause_and_freeze_fault_hold_pose(self):
        self.state.command_move(0.5, 0.0, 0.0)
        self.state.set_paused(True)
        self.clock.advance(0.2)
        self.assertEqual(self.state.step()["pose"]["x"], 0.0)
        self.state.set_paused(False)
        self.state.set_fault("freeze_motion")
        self.clock.advance(0.2)
        self.assertEqual(self.state.step()["pose"]["x"], 0.0)

    def test_reset_is_deterministic(self):
        self.state.command_move(0.5, 0.1, 0.2)
        self.clock.advance(0.2)
        self.state.step()
        reset = self.state.reset(seed=9)
        self.assertEqual(reset["seed"], 9)
        self.assertEqual(reset["pose"], {"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(reset["sequence"], 0)

    def test_joint_names_and_quaternion_are_stable(self):
        self.state.command_move(0.5, 0.0, 0.4)
        self.clock.advance(0.1)
        self.state.step()
        joints = self.state.joints_snapshot(["left_hip_pitch_joint", "right_knee_joint"])
        self.assertEqual([item["name"] for item in joints["joints"]], ["left_hip_pitch_joint", "right_knee_joint"])
        quat = joints["imu_quat"]
        self.assertAlmostEqual(sum(value * value for value in quat), 1.0, places=5)
        self.assertTrue(math.isfinite(joints["joints"][0]["q"]))

    def test_protocol_snapshot_does_not_fabricate_physical_telemetry(self):
        snap = self.state.loco_snapshot()
        self.assertEqual(snap["robot_morphology"], "humanoid_biped")
        self.assertEqual(snap["simulation_backend"], "protocol_only_no_physics")
        self.assertFalse(snap["physical_telemetry"]["valid"])
        self.assertIn("do not infer balance", snap["physical_telemetry"]["reason"])
        self.assertIsNone(snap["foot_force"])
        self.assertFalse(snap["foot_force_valid"])
        self.assertEqual(snap["mode"], 0)

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            self.state.command_move(float("nan"), 0.0, 0.0)
        with self.assertRaises(ValueError):
            self.state.set_fault("unknown")


if __name__ == "__main__":
    unittest.main()
