#!/usr/bin/env python3
"""Isolated real ROS camera test. No Core registration or running Driver writes."""
from collections import deque
from io import BytesIO
import json
import math
import time

import mujoco
import numpy as np
from PIL import Image
import rclpy
from sensor_msgs.msg import CompressedImage

from main import LOW_LATENCY_QOS, TOPICS, SimPublisherNode, SimG1Bundle
from mujoco_backend import MujocoSimulationState


def main():
    rclpy.init()
    state = MujocoSimulationState('/work/resource/mujoco/g1/scene_29dof.xml',
                                  camera_enabled=True)
    node = SimPublisherNode(state, [])
    observer = rclpy.create_node('isolated_camera_probe')
    bundle = SimG1Bundle(node, state, '<robot name="test"/>')
    frames = deque(maxlen=30)
    observer.create_subscription(CompressedImage, TOPICS['camera_rgb'],
                                 frames.append, LOW_LATENCY_QOS)

    def wait_for(predicate, seconds=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.1)
            if predicate():
                return
        raise AssertionError('Camera condition timed out')

    try:
        assert bundle.dispatch('camera_rgb', {'action': 'info'})['state'] == 'idle'
        bundle.dispatch('camera_rgb', {'action': 'start'})
        wait_for(lambda: len(frames) >= 3)
        before = np.asarray(Image.open(BytesIO(bytes(frames[-1].data)))).copy()
        assert before.shape == (240, 320, 3)
        assert frames[-1].header.frame_id == 'sim_torso_camera'
        stamps = [(f.header.stamp.sec, f.header.stamp.nanosec) for f in frames]
        assert stamps == sorted(set(stamps)), stamps
        # This state belongs only to this temporary test, never a live service.
        with state._lock:
            state._data.qpos[3:7] = [math.cos(math.pi/8), 0, 0, math.sin(math.pi/8)]
            mujoco.mj_forward(state._model, state._data)
        frames.clear()
        wait_for(lambda: len(frames) >= 3)
        after = np.asarray(Image.open(BytesIO(bytes(frames[-1].data))))
        change = float(np.mean(np.any(before != after, axis=2)))
        assert change > .01, change
        bundle.dispatch('camera_rgb', {'action': 'stop'})
        count = node.metrics('camera_rgb')['published']
        time.sleep(.6)
        assert node.metrics('camera_rgb')['published'] == count
        state.set_fault('drop_camera')
        bundle.dispatch('camera_rgb', {'action': 'start'})
        time.sleep(.6)
        assert node.metrics('camera_rgb')['published'] == count
        state.set_fault('none')
        wait_for(lambda: node.metrics('camera_rgb')['published'] > count)
        def fail_render():
            raise RuntimeError('injected render failure')
        state.render_camera = fail_render
        wait_for(lambda: bool(node.metrics('camera_rgb')['last_error']))
        result = bundle.dispatch('camera_rgb', {'action': 'info'})
        assert result['state'] == 'error' and 'injected render failure' in result['last_error']
        try:
            bundle.dispatch('camera_rgb', {'action': 'start'})
        except RuntimeError:
            pass
        else:
            raise AssertionError('Failed camera start must not succeed')
        print(json.dumps({'camera_ros': 'PASS', 'changed_pixel_ratio': change,
                          'published': result['published'],
                          'stop_fault_and_error': 'PASS'}))
    finally:
        node.set_active('camera_rgb', False)
        node.stop_camera_worker()
        node.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
