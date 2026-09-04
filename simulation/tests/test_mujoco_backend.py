import os
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim-driver"))
MODEL_PATH = os.environ.get("SIM_MUJOCO_TEST_MODEL", "")


@unittest.skipUnless(MODEL_PATH and Path(MODEL_PATH).is_file(), "locked MuJoCo G1 model unavailable")
class MujocoSimulationStateTest(unittest.TestCase):
    class FakeClock:
        def __init__(self):
            self.value = 100.0

        def __call__(self):
            return self.value

        def advance(self, seconds: float):
            self.value += seconds

    def setUp(self):
        from mujoco_backend import MujocoSimulationState

        self.clock = self.FakeClock()
        self.state = MujocoSimulationState(MODEL_PATH, clock=self.clock, seed=23)

    def advance(self, seconds: float):
        steps = int(round(seconds / 0.05))
        for _ in range(steps):
            self.clock.advance(0.05)
            self.state.step()
        return self.state.loco_snapshot()

    def test_stand_semantic_wave_fall_and_reset(self):
        standing = self.advance(3.0)
        self.assertEqual(standing["simulation_backend"], "mujoco_g1_29dof")
        self.assertEqual(standing["balance"]["state"], "stable")
        self.assertTrue(standing["physical_telemetry"]["valid"])
        self.assertTrue(standing["physical_telemetry"]["balance_assist"])
        self.assertFalse(standing["physical_telemetry"]["autonomous_balance"])
        self.assertFalse(standing["gait_valid"])
        self.assertEqual(
            standing["control_mode"],
            "joint_position_servo_with_virtual_base_stabilization",
        )
        self.assertGreater(standing["contact_forces_n"]["left_foot"], 50.0)
        self.assertGreater(standing["contact_forces_n"]["right_foot"], 50.0)

        tracked_names = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_wrist_roll_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_roll_joint",
            "waist_yaw_joint",
        ]
        positions = {name: [] for name in tracked_names}
        phases = set()
        balance_states = []
        wave = self.state.command_wave(duration=5.0)
        self.assertEqual(wave["control_mode"], "mujoco_joint_position_servo")
        self.assertEqual(wave["motion_semantics"], "raise_left_arm_wave_wrist_then_lower")
        for _ in range(110):
            self.clock.advance(0.05)
            self.state.step()
            state = self.state.snapshot()
            phases.add(state["gesture_phase"])
            balance_states.append(self.state.loco_snapshot()["balance"]["state"])
            joints = self.state.joints_snapshot(tracked_names)["joints"]
            for joint in joints:
                self.assertTrue(joint["valid"])
                positions[joint["name"]].append(joint["q"])
        self.assertGreater(max(positions["left_shoulder_roll_joint"]), 1.2)
        self.assertGreater(
            max(positions["left_wrist_yaw_joint"])
            - min(positions["left_wrist_yaw_joint"]),
            0.9,
        )
        self.assertLess(
            max(positions["right_shoulder_roll_joint"])
            - min(positions["right_shoulder_roll_joint"]),
            0.15,
        )
        self.assertLess(
            max(positions["waist_yaw_joint"])
            - min(positions["waist_yaw_joint"]),
            0.15,
        )
        self.assertTrue({"raising", "waving", "lowering"} <= phases, phases)
        self.assertTrue(all(state == "stable" for state in balance_states), balance_states)
        self.assertEqual(self.state.snapshot()["gesture"], "idle")
        self.assertAlmostEqual(positions["left_shoulder_pitch_joint"][-1], 0.2, delta=0.15)
        self.assertAlmostEqual(positions["left_shoulder_roll_joint"][-1], 0.0, delta=0.15)
        self.assertAlmostEqual(positions["left_wrist_yaw_joint"][-1], 0.0, delta=0.15)

        self.state.reset(seed=23)
        self.advance(1.0)
        self.state.set_balance_assist(False)
        self.state.apply_push(400.0, 0.0, duration=0.3)
        fallen = self.advance(4.0)
        self.assertTrue(fallen["balance"]["fallen"])
        self.assertEqual(fallen["balance"]["state"], "fallen")

        reset = self.state.reset(seed=23)
        self.assertEqual(reset["pose"], {"x": 0.0, "y": 0.0, "yaw": 0.0})
        recovered = self.advance(3.0)
        self.assertEqual(recovered["balance"]["state"], "stable")
        self.assertTrue(recovered["physical_telemetry"]["balance_assist"])


if __name__ == "__main__":
    unittest.main()
