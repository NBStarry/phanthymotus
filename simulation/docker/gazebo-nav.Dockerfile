ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

ARG ROS2_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
RUN printf '%s\n' "deb [arch=amd64 signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS2_MIRROR} jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get -o Acquire::Retries=3 update && \
    apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      ros-humble-ros-gz=0.244.25-1jammy.20260804.223901 \
      ros-humble-navigation2=1.1.20-1jammy.20260804.223401 \
      ros-humble-nav2-bringup=1.1.20-1jammy.20260804.225407 \
      ros-humble-slam-toolbox=2.6.10-1jammy.20260804.222728 && \
    rm -rf /var/lib/apt/lists/*

COPY simulation/gazebo-nav/ /sim_ws/src/phanthymotus_sim_nav/
RUN python3 /sim_ws/src/phanthymotus_sim_nav/tools/generate_map.py && \
    . /opt/ros/humble/setup.sh && \
    cd /sim_ws && colcon build --packages-select phanthymotus_sim_nav --cmake-args -DCMAKE_BUILD_TYPE=Release

ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.revision=${SOURCE_REVISION} \
      phanthymotus.sim.backend=gazebo-fortress-nav2
ENV PYTHONUNBUFFERED=1 ROS_DOMAIN_ID=83 MCP_PORT=15731
EXPOSE 15731
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /sim_ws/install/setup.bash && ros2 launch phanthymotus_sim_nav gazebo_nav.launch.py"]
