ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

COPY simulation/artifacts/python-wheels/pillow-11.3.0-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl /tmp/python-wheels/
COPY simulation/artifacts/python-wheels-p2/ /tmp/python-wheels/
COPY simulation/sim-driver/requirements.txt /tmp/sim-driver-requirements.txt
COPY simulation/sim-driver/requirements.p2.txt /tmp/sim-driver-p2-requirements.txt
RUN python3 -m pip install --no-index --find-links /tmp/python-wheels \
      --require-hashes -r /tmp/sim-driver-requirements.txt && \
    python3 -m pip install --no-deps --no-index --find-links /tmp/python-wheels \
      --require-hashes -r /tmp/sim-driver-p2-requirements.txt && \
    python3 -c 'import mujoco, numpy; assert mujoco.__version__ == "3.3.6"; print("MuJoCo", mujoco.__version__, "NumPy", numpy.__version__)' && \
    rm -rf /tmp/python-wheels /tmp/sim-driver-requirements.txt /tmp/sim-driver-p2-requirements.txt

WORKDIR /work
COPY simulation/sim-driver/ /work/
COPY simulation/tests/ /work/tests/
COPY simulation/src/phanthymotus-driver/common/ /work/common/
COPY simulation/src/phanthymotus-driver/unitree/g1/resource/g1_model.urdf /work/resource/g1_model.urdf
COPY simulation/src/unitree_mujoco/LICENSE /work/resource/mujoco/LICENSE
COPY simulation/src/unitree_mujoco/unitree_robots/g1/ /work/resource/mujoco/g1/
RUN SIM_MUJOCO_TEST_MODEL=/work/resource/mujoco/g1/scene_29dof.xml \
    python3 -m unittest discover -s /work/tests -p 'test_mujoco_backend.py' -v

ARG SOURCE_REVISION=unknown
ARG UNITREE_MUJOCO_REVISION=unknown
LABEL org.opencontainers.image.revision=${SOURCE_REVISION} \
      phanthymotus.sim.unitree-mujoco-revision=${UNITREE_MUJOCO_REVISION}
ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    MCP_PORT=15730 \
    SIM_NAMESPACE=phanthymotus_sim_g1 \
    SIMULATION_BACKEND=mujoco \
    SIM_MUJOCO_MODEL=/work/resource/mujoco/g1/scene_29dof.xml
EXPOSE 15730
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
