#!/usr/bin/env python3
"""Bounded real ROS integration: room camera -> VOP/face; speech -> ASR -> TTS.

Run inside Perception, after sourcing ROS. Uses isolated test instances, not the
shared canvas. No direct TTS speak call: audio must come from the ASR topic.
"""
import argparse
import json
import ssl
import time
from urllib.request import Request, urlopen

import rclpy
from audio_msgs.msg import AudioChunk
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver-id', required=True)
    parser.add_argument('--perception-id', required=True)
    parser.add_argument('--expected-text', default='椅子', help='Required ASR text from the room speech fixture')
    args = parser.parse_args()
    if not args.expected_text.strip():
        parser.error('--expected-text must not be empty')
    context = ssl._create_unverified_context()  # Loopback development Core only.
    base = 'https://127.0.0.1:15678'

    def request(path, body=None):
        req = Request(base + path, None if body is None else json.dumps(body).encode(),
                      {'Content-Type': 'application/json'})
        with urlopen(req, context=context, timeout=35) as response:
            return json.load(response)

    def call(mid, tool, action, **extra):
        result = request('/api/mcp/' + mid + '/call',
                         {'tool': tool, 'arguments': {'action': action, **extra}})
        assert result.get('code') == 200, result
        data = json.loads(result['data'][0]['text'])
        assert not (data.get('error') or data.get('error_code') or data.get('state') == 'error'), data
        print(tool, action, data.get('state'), flush=True)
        return data

    audio = '/phanthymotus_sim_g1/mic/audio'
    image = '/phanthymotus_sim_g1/camera/rgb'
    cards = [('tts', audio + '/asr'), ('asr', audio), ('vop', image), ('face_recognition', image)]
    assert request('/api/config/project-running').get('running') is False
    for sensor in ('mic', 'camera_rgb'):
        data = call(args.driver_id, sensor, 'info')
        assert data.get('simulation') and data.get('state') == 'idle', data
        if sensor == 'camera_rgb':
            assert data.get('input_source', {}).get('source') == 'mujoco_render', data
    for card, topic in cards:
        assert call(args.perception_id, card, 'info').get('state') == 'idle'

    rclpy.init()
    node = rclpy.create_node('sim_perception_stack_acceptance')
    evidence = {'asr': False, 'vop': False, 'face_empty': False, 'tts_bytes': 0, 'tts_nonzero': False, 'tts_eof': False}

    def receive(kind, msg):
        if kind == 'tts':
            payload = bytes(msg.data)
            if payload == b'\x01\x00\xff\xff\x01\x00\xff\xff':
                evidence['tts_eof'] = True
            else:
                evidence['tts_bytes'] += len(payload)
                evidence['tts_nonzero'] |= any(payload)
            return
        data = json.loads(msg.data)
        if kind == 'asr':
            evidence['asr'] |= args.expected_text in data.get('text', '')
        elif kind == 'vop':
            evidence['vop'] |= any(o.get('name') == 'chair' for o in data.get('objects', []))
        else:
            evidence['face_empty'] |= data.get('count') == 0 and data.get('faces') == []

    subscriptions = []
    for kind, topic in [('asr', audio + '/asr'), ('vop', image + '/objects'),
                        ('face', image + '/face'), ('tts', audio + '/asr/tts')]:
        subscriptions.append(node.create_subscription(
            AudioChunk if kind == 'tts' else String, topic,
            lambda msg, kind=kind: receive(kind, msg), qos_profile_sensor_data))
    cleanup = []
    try:
        for sensor in ('mic', 'camera_rgb'):
            cleanup.append((args.driver_id, sensor, {}))
            assert call(args.driver_id, sensor, 'start').get('state') == 'running'
        for card, topic in cards:
            extra = {'instance_id': 'sim_stack_' + card, 'input_topic': topic}
            cleanup.append((args.perception_id, card, extra))
            assert call(args.perception_id, card, 'start', **extra).get('state') in ('running', 'loading')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
            if all(evidence.values()) and evidence['tts_bytes'] >= 32000:
                print('PUBLIC_STACK_ROS_PASS', json.dumps(evidence), flush=True)
                break
        else:
            raise RuntimeError('Missing joint output: ' + json.dumps(evidence))
    finally:
        errors = []
        for mid, tool, extra in reversed(cleanup):
            try:
                assert call(mid, tool, 'stop', **extra).get('state') == 'idle'
            except Exception as exc:
                errors.append(f'{tool}: {exc}')
        node.destroy_node()
        rclpy.shutdown()
        if errors:
            raise RuntimeError('Cleanup failed: ' + '; '.join(errors))


if __name__ == '__main__':
    main()
