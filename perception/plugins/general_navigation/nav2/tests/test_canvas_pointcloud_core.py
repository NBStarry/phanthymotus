import struct
import unittest

from g1_nav2.canvas_pointcloud_core import (
    InvalidCanvasPointCloud,
    decode_canvas_pointcloud,
)


class CanvasPointCloudCoreTest(unittest.TestCase):
    @staticmethod
    def envelope(point_step, point_count, points, *, stamp=123456789, frame="livox_frame"):
        frame_bytes = frame.encode("utf-8")
        return (
            struct.pack(
                "<4sHHIIqH",
                b"PCV2",
                2,
                0,
                point_step,
                point_count,
                stamp,
                len(frame_bytes),
            )
            + frame_bytes
            + points
        )

    def test_decodes_the_timestamped_lidar_cloud_envelope(self):
        point_step = 16
        points = b"".join(
            struct.pack("<fffI", x, y, z, intensity)
            for x, y, z, intensity in (
                (1.0, 2.0, 3.0, 4),
                (-1.0, -2.0, -3.0, 5),
            )
        )

        cloud = decode_canvas_pointcloud(self.envelope(point_step, 2, points))

        self.assertEqual(cloud.source_stamp_ns, 123456789)
        self.assertEqual(cloud.frame_id, "livox_frame")
        self.assertEqual(cloud.timestamp_source, "driver")
        self.assertEqual(cloud.frame_source, "driver_payload")
        self.assertEqual(cloud.source_schema, "phanthy.sensor.pointcloud.v2")
        self.assertEqual(cloud.point_step, point_step)
        self.assertEqual(cloud.point_count, 2)
        self.assertEqual(cloud.data, points)

    def test_rejects_truncated_or_unsafe_envelopes(self):
        invalid = (
            b"",
            self.envelope(16, 1, b"\x00" * 16, stamp=0),
            self.envelope(16, 1, b"\x00" * 16, frame="/unsafe"),
            struct.pack("<II", 8, 1) + b"\x00" * 8,
            self.envelope(12, 2, b"\x00" * 12),
        )
        for payload in invalid:
            with self.subTest(payload=payload):
                with self.assertRaises(InvalidCanvasPointCloud):
                    decode_canvas_pointcloud(payload)

    def test_decodes_released_driver_legacy_envelope_with_explicit_metadata(self):
        points = struct.pack("<fff", 1.0, 2.0, 3.0)
        payload = struct.pack("<II", 12, 1) + points

        cloud = decode_canvas_pointcloud(
            payload,
            receive_stamp_ns=987654321,
            legacy_frame_id="livox_frame",
        )

        self.assertEqual(cloud.source_stamp_ns, 987654321)
        self.assertEqual(cloud.frame_id, "livox_frame")
        self.assertEqual(cloud.timestamp_source, "adapter_receive")
        self.assertEqual(cloud.frame_source, "adapter_contract")
        self.assertEqual(cloud.source_schema, "unitree.g1.pointcloud.legacy")
        self.assertEqual(cloud.data, points)

    def test_legacy_requires_receive_timestamp_frame_and_exact_size(self):
        valid = struct.pack("<II", 12, 1) + b"\x00" * 12
        for kwargs in (
            {},
            {"receive_stamp_ns": 1},
            {"legacy_frame_id": "livox_frame"},
            {"receive_stamp_ns": 1, "legacy_frame_id": "/unsafe"},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(InvalidCanvasPointCloud):
                    decode_canvas_pointcloud(valid, **kwargs)
        with self.assertRaises(InvalidCanvasPointCloud):
            decode_canvas_pointcloud(
                valid[:-1], receive_stamp_ns=1, legacy_frame_id="livox_frame"
            )


if __name__ == "__main__":
    unittest.main()
