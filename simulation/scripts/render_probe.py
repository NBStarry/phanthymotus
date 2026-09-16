#!/usr/bin/env python3
"""Isolated RGB/depth rendering check; no ROS, network, or running robot state.

Run with MUJOCO_GL=osmesa in the render candidate image. This checks rendering
capability, not the Driver camera integration or learned perception accuracy.
"""
import argparse
import json
import math
import time

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--g1-model', help='Also verify a torso-mounted camera on the actual G1 MJCF')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_string('''
    <mujoco>
      <worldbody>
        <light pos="0 0 4"/>
        <camera name="sensor" pos="0 -3 1" xyaxes="1 0 0 0 0 1"/>
        <geom type="plane" size="5 5 .1" rgba=".3 .3 .3 1"/>
        <body pos="0 0 1"><freejoint/>
          <geom type="box" size=".4 .4 .4" rgba="1 .1 .1 1"/>
        </body>
      </worldbody>
    </mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    started = time.monotonic()
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data, camera='sensor')
        rgb = renderer.render().copy()
        assert rgb.shape == (240, 320, 3) and rgb.dtype == np.uint8
        assert np.ptp(rgb) > 50, 'RGB lacks scene contrast'
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        assert depth.shape == (240, 320) and np.isfinite(depth).all()
        assert 2.4 < depth[120, 160] < 2.8, depth[120, 160]
        renderer.disable_depth_rendering()
        # Change the actual simulated body pose, not the rendered image pixels.
        data.qpos[0] += 1.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera='sensor')
        moved = renderer.render().copy()
        changed = float(np.mean(np.any(rgb != moved, axis=2)))
        assert changed > .01, f'Body displacement did not change camera: {changed}'
    print(json.dumps({'render_probe': 'PASS', 'mujoco': mujoco.__version__,
                      'center_depth_m': float(depth[120, 160]),
                      'changed_pixel_ratio': changed,
                      'elapsed_seconds': round(time.monotonic() - started, 3)}))
    if args.g1_model:
        spec = mujoco.MjSpec.from_file(args.g1_model)
        spec.body('torso_link').add_camera(
            name='sim_rgb', pos=[.12, 0, .35],
            xyaxes=[0, -1, 0, 0, 0, 1], fovy=65)
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX,
                                pos=[2, 0, 1.2], size=[.3, .3, .3],
                                rgba=[1, .05, .05, 1])
        model = spec.compile()
        assert model.nu == 29
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        before_position = data.cam_xpos.copy()
        with mujoco.Renderer(model, height=240, width=320) as renderer:
            renderer.update_scene(data, camera='sim_rgb')
            before = renderer.render().copy()
            # Rotate the simulated root 45 degrees. Camera follows the robot.
            data.qpos[3:7] = [math.cos(math.pi / 8), 0, 0, math.sin(math.pi / 8)]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera='sim_rgb')
            after = renderer.render().copy()
            changed = float(np.mean(np.any(before != after, axis=2)))
            assert changed > .05, changed
            assert not np.allclose(before_position, data.cam_xpos)
        print(json.dumps({'g1_mounted_camera': 'PASS', 'actuators': model.nu,
                          'changed_pixel_ratio': changed,
                          'camera_position': data.cam_xpos.tolist()}))


if __name__ == '__main__':
    main()
