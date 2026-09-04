ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

COPY simulation/artifacts/python-wheels/pillow-11.3.0-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl /tmp/python-wheels/
COPY simulation/sim-driver/requirements.txt /tmp/sim-driver-requirements.txt
RUN python3 -m pip install --no-index --find-links /tmp/python-wheels \
      --require-hashes -r /tmp/sim-driver-requirements.txt && \
    rm -rf /tmp/python-wheels /tmp/sim-driver-requirements.txt

WORKDIR /work
COPY simulation/sim-driver/ /work/
COPY simulation/src/phanthymotus-driver/common/ /work/common/
COPY simulation/src/phanthymotus-driver/unitree/g1/resource/g1_model.urdf /work/resource/g1_model.urdf

ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.revision=${SOURCE_REVISION}
ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    MCP_PORT=15730 \
    SIM_NAMESPACE=phanthymotus_sim_g1
EXPOSE 15730
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
