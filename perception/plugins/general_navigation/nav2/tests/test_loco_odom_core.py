import json
import math
import unittest

from g1_nav2.loco_odom_core import InvalidLocoState, OriginNormalizer


def state(x=10.0, y=20.0, yaw=1.0, vx=0.2, vy=-0.1, wz=0.3):
    return {
        "schema_version": 2,
        "source_stamp_ns": 5_000_000_000,
        "frame_id": "odom_source",
        "position": [x, y, 0.0],
        "velocity": [vx, vy, 0.0],
        "yaw_speed": wz,
        "imu": {
            "rpy": [0.0, 0.0, yaw],
            "quaternion": [1.0, 0.0, 0.0, 0.0],
        },
    }


class OriginNormalizerTest(unittest.TestCase):
    def test_first_sample_becomes_origin(self):
        converter = OriginNormalizer(reset_origin=True)
        output = converter.convert(json.dumps(state()))
        self.assertEqual(output.source_stamp_ns, 5_000_000_000)
        self.assertEqual(output.source_frame, "odom_source")
        self.assertEqual(output.timestamp_source, "driver")
        self.assertEqual(output.frame_source, "driver_payload")
        self.assertEqual(output.source_schema, "phanthy.g1.loco_state.v2")
        self.assertAlmostEqual(output.x, 0.0)
        self.assertAlmostEqual(output.y, 0.0)
        self.assertAlmostEqual(output.yaw, 0.0)
        self.assertAlmostEqual(output.vx, 0.2)
        self.assertAlmostEqual(output.vy, -0.1)
        self.assertAlmostEqual(output.wz, 0.3)

    def test_translation_is_rotated_into_initial_heading(self):
        converter = OriginNormalizer(reset_origin=True)
        converter.convert(state(x=0.0, y=0.0, yaw=math.pi / 2))
        output = converter.convert(
            state(x=0.0, y=1.0, yaw=math.pi / 2)
        )
        self.assertAlmostEqual(output.x, 1.0, places=6)
        self.assertAlmostEqual(output.y, 0.0, places=6)

    def test_yaw_wraps_across_pi(self):
        converter = OriginNormalizer(reset_origin=True)
        converter.convert(state(yaw=math.pi - 0.1))
        output = converter.convert(state(yaw=-math.pi + 0.1))
        self.assertAlmostEqual(output.yaw, 0.2, places=6)

    def test_odom_velocity_can_be_rotated_to_body(self):
        converter = OriginNormalizer(
            reset_origin=False, velocity_frame="odom"
        )
        output = converter.convert(
            state(x=0.0, y=0.0, yaw=math.pi / 2, vx=0.0, vy=1.0)
        )
        self.assertAlmostEqual(output.vx, 1.0, places=6)
        self.assertAlmostEqual(output.vy, 0.0, places=6)

    def test_invalid_payload_fails_closed(self):
        converter = OriginNormalizer()
        with self.assertRaises(InvalidLocoState):
            converter.convert({"position": [1.0]})

    def test_released_legacy_payload_uses_explicit_adapter_metadata(self):
        payload = state()
        for field in ("schema_version", "source_stamp_ns", "frame_id"):
            payload.pop(field)
        converter = OriginNormalizer()

        output = converter.convert(payload, receive_stamp_ns=7_000_000_000)

        self.assertEqual(output.source_stamp_ns, 7_000_000_000)
        self.assertEqual(output.source_frame, "odom_source")
        self.assertEqual(output.timestamp_source, "adapter_receive")
        self.assertEqual(output.frame_source, "adapter_contract")
        self.assertEqual(output.source_schema, "unitree.g1.loco_state.legacy")

    def test_legacy_requires_receive_stamp_and_unknown_versions_fail_closed(self):
        converter = OriginNormalizer()
        for payload in (
            {"position": [1.0, 2.0]},
            {**state(), "source_stamp_ns": 0},
            {**state(), "frame_id": "unknown"},
            {**state(), "schema_version": 3},
        ):
            with self.subTest(payload=payload):
                with self.assertRaises(InvalidLocoState):
                    converter.convert(payload)


if __name__ == "__main__":
    unittest.main()
