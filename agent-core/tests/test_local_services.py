"""Sim-only local service manifest and lifecycle regression tests.

Run inside the Agent Core runtime, which already contains FastAPI and docker-py:
python3 -m unittest agent-core/tests/test_local_services.py -v
"""

import json
import os
import sys
import tempfile
import unittest
from unittest import mock

_TEST_DIR = tempfile.mkdtemp(prefix='phanthymotus-local-services-')
os.environ['DB_PATH'] = os.path.join(_TEST_DIR, 'data.db')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from api import drivers  # noqa: E402


class LocalServicesTest(unittest.TestCase):
    def test_local_manifest_overrides_db_without_persisting(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'services.json')
            with open(path, 'w', encoding='utf-8') as f:
                json.dump([{
                    'id': 'sim-gazebo',
                    'name': 'Simulated Navigation (Gazebo)',
                    'category': 'driver',
                    'image': 'phanthymotus-sim/gazebo-nav:test',
                    'container_name': 'phanthymotus-sim-p3-gazebo-nav',
                }], f)
            fake_config = {
                'drivers': [{'id': 'sim-gazebo', 'image': 'stale:image'},
                            {'id': 'published', 'image': 'registry:image'}],
            }
            with mock.patch.dict(os.environ, {'LOCAL_SERVICES_MANIFEST': path}), \
                    mock.patch.object(drivers._config, 'main', fake_config):
                manifest = drivers._load_manifest()
                by_id = {item['id']: item for item in manifest}
                self.assertEqual(
                    by_id['sim-gazebo']['image'],
                    'phanthymotus-sim/gazebo-nav:test',
                )
                self.assertIs(by_id['sim-gazebo']['local_managed'], True)

                drivers._save_manifest(manifest)
                self.assertEqual(drivers._config.main['drivers'], [
                    {'id': 'published', 'image': 'registry:image'},
                ])

    def test_invalid_local_manifest_fails_loudly(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'services.json')
            with open(path, 'w', encoding='utf-8') as f:
                f.write('{}')
            with mock.patch.dict(os.environ, {'LOCAL_SERVICES_MANIFEST': path}):
                with self.assertRaisesRegex(RuntimeError, 'must contain a list'):
                    drivers._load_local_manifest()

    def test_stopped_exact_image_container_starts_without_pull(self):
        class Container:
            status = 'exited'
            attrs = {'Config': {'Image': 'phanthymotus-sim/gazebo-nav:test'}}
            started = False

            def start(self):
                self.started = True

        container = Container()

        class Containers:
            @staticmethod
            def get(name):
                self.assertEqual(name, 'phanthymotus-sim-p3-gazebo-nav')
                return container

        class Api:
            @staticmethod
            def pull(*args, **kwargs):
                raise AssertionError('an exact local image must not be pulled')

        client = type('Client', (), {'containers': Containers(), 'api': Api()})()
        with mock.patch.object(drivers, '_docker', return_value=client):
            result = drivers._deploy_sync({
                'id': 'sim-gazebo',
                'image': 'phanthymotus-sim/gazebo-nav:test',
                'container_name': 'phanthymotus-sim-p3-gazebo-nav',
            })
        self.assertIs(container.started, True)
        self.assertIs(result['reused'], True)
        self.assertEqual(result['container_name'], 'phanthymotus-sim-p3-gazebo-nav')


if __name__ == '__main__':
    unittest.main()
