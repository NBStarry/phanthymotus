import json
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "g1_nav2"
sys.path.insert(0, str(PACKAGE_ROOT))

from g1_nav2.map_store import MAP_FILES, MapStore, MapStoreError  # noqa: E402


def _write_map_files(directory: Path, *, image: str = "map.pgm") -> None:
    (directory / "map.yaml").write_text(
        "\n".join(
            (
                f"image: {image}",
                "mode: trinary",
                "resolution: 0.05",
                "origin: [-1, -1, 0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            )
        ),
        encoding="utf-8",
    )
    for filename in MAP_FILES[1:]:
        (directory / filename).write_bytes(f"content:{filename}".encode())


class MapStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "maps"
        self.store = MapStore(self.root)

    def tearDown(self):
        self.temporary.cleanup()

    def _ready_map(self, name: str = "上海展厅"):
        session = self.store.begin_mapping(name)
        _write_map_files(session.path)
        summary = self.store.finalize_mapping(session)
        return session, summary

    def test_finalize_is_atomic_and_records_four_artifacts(self):
        session = self.store.begin_mapping("上海展厅")
        tag = self.store.put_tag(
            session.path,
            session.map_name,
            "起点",
            "展厅入口",
            {"x": 0.0, "y": 0.0, "yaw": 0.25},
        )
        _write_map_files(session.path)

        summary = self.store.finalize_mapping(session)

        self.assertFalse(session.path.exists())
        self.assertEqual(summary["map_name"], "上海展厅")
        self.assertEqual(summary["tag_count"], 1)
        self.assertEqual(tag["name"], "起点")
        manifest = json.loads(
            (self.root / "上海展厅" / "manifest.json").read_text()
        )
        self.assertEqual(manifest["state"], "ready")
        self.assertEqual(set(manifest["files"]), set(MAP_FILES))
        self.assertTrue(all(item["sha256"] for item in manifest["files"].values()))

    def test_tags_survive_finalize_and_support_update_and_remove(self):
        _, summary = self._ready_map("office")
        directory = Path(summary["map_yaml"]).parent

        first = self.store.put_tag(
            directory,
            "office",
            "desk",
            "first",
            {"x": 1, "y": 2, "yaw": 0.3},
        )
        second = self.store.put_tag(
            directory,
            "office",
            "desk",
            "updated",
            {"x": 3, "y": 4, "yaw": -0.2},
        )

        self.assertEqual(first["created_at"], second["created_at"])
        self.assertEqual(self.store.get_tag(directory, "office", "desk")["x"], 3.0)
        self.assertEqual(self.store.list_tags(directory, "office")[0]["description"], "updated")
        removed = self.store.remove_tag(directory, "office", "desk")
        self.assertEqual(removed["status"], "removed")
        self.assertEqual(self.store.list_tags(directory, "office"), [])

    def test_active_map_is_persisted_and_cleared_on_delete(self):
        self._ready_map("office")
        self.store.set_active_map("office")
        self.assertEqual(self.store.active_map(), "office")

        result = self.store.delete_map("office")

        self.assertEqual(result, {"map_name": "office", "status": "deleted"})
        self.assertIsNone(self.store.active_map())
        self.assertEqual(self.store.list_maps(), [])

    def test_list_maps_surfaces_corrupt_visible_directories(self):
        corrupt = self.root / "broken"
        corrupt.mkdir()

        maps = self.store.list_maps()

        self.assertEqual(maps[0]["map_name"], "broken")
        self.assertEqual(maps[0]["status"], "corrupt")
        self.assertEqual(maps[0]["error_code"], "map_incomplete")

    def test_rejects_paths_hidden_names_controls_and_symlink_maps(self):
        for value in ("../escape", ".hidden", "bad/name", "bad\\name", "bad\nname"):
            with self.subTest(value=value), self.assertRaises(MapStoreError):
                self.store.begin_mapping(value)

        outside = Path(self.temporary.name) / "outside"
        outside.mkdir()
        (self.root / "linked").symlink_to(outside, target_is_directory=True)
        with self.assertRaises(MapStoreError) as captured:
            self.store.map_summary("linked")
        self.assertEqual(captured.exception.code, "map_not_found")

    def test_incomplete_or_external_image_never_becomes_ready(self):
        session = self.store.begin_mapping("unsafe")
        _write_map_files(session.path, image="../outside.pgm")

        with self.assertRaises(MapStoreError) as captured:
            self.store.finalize_mapping(session)

        self.assertEqual(captured.exception.code, "invalid_map_yaml")
        self.assertTrue(session.path.exists())
        self.assertFalse((self.root / "unsafe").exists())


if __name__ == "__main__":
    unittest.main()
