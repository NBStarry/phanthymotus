ARG ROS_BASE_IMAGE=ros:humble@sha256:5417e56962ff6e15d4cf9b2f78a71a78f3901f47cd78696b575b7eecdb54eb78
FROM ${ROS_BASE_IMAGE}

ARG APT_UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG APT_ROS_MIRROR=mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ARG PYTHON_COLCON_VERSION=0.3.0-100
ARG PYTHON_PYTEST_VERSION=6.2.5-1ubuntu2
ARG ROS_NAV2_BRINGUP_VERSION=1.1.20-1jammy.20260613.010543
ARG ROS_NAVIGATION2_VERSION=1.1.20-1jammy.20260613.005009
ARG ROS_POINTCLOUD_TO_LASERSCAN_VERSION=2.0.1-3jammy.20260607.112335
ARG ROS_RMW_FASTRTPS_CPP_VERSION=6.2.10-1jammy.20260605.135747
ARG ROS_SLAM_TOOLBOX_VERSION=2.6.10-1jammy.20260613.004837

RUN sed -i "s|http://ports.ubuntu.com/ubuntu-ports|https://${APT_UBUNTU_MIRROR}/ubuntu-ports|g" /etc/apt/sources.list && \
    sed -i "s|http://packages.ros.org/ros2/ubuntu|https://${APT_ROS_MIRROR}|g; s|^Types: deb deb-src$|Types: deb|" /etc/apt/sources.list.d/ros2.sources

# Docker Desktop arm64 emulation makes apt's `_apt` sandbox fail detached
# signature verification although direct gpgv succeeds. Keep signature checks
# enabled, but perform build-time acquire/verify as root.
RUN apt-get -o APT::Sandbox::User=root update && \
    apt-get -o APT::Sandbox::User=root install -y --no-install-recommends \
      python3-colcon-common-extensions=${PYTHON_COLCON_VERSION} \
      python3-pytest=${PYTHON_PYTEST_VERSION} \
      ros-humble-nav2-bringup=${ROS_NAV2_BRINGUP_VERSION} \
      ros-humble-navigation2=${ROS_NAVIGATION2_VERSION} \
      ros-humble-pointcloud-to-laserscan=${ROS_POINTCLOUD_TO_LASERSCAN_VERSION} \
      ros-humble-rmw-fastrtps-cpp=${ROS_RMW_FASTRTPS_CPP_VERSION} \
      ros-humble-slam-toolbox=${ROS_SLAM_TOOLBOX_VERSION} && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /nav2_ws
COPY g1_nav2/ /nav2_ws/src/g1_nav2/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select g1_nav2 --symlink-install && \
    dpkg-query -W \
      python3-colcon-common-extensions \
      python3-pytest \
      ros-humble-nav2-bringup \
      ros-humble-navigation2 \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-slam-toolbox \
      > /opt/g1-nav2-package-lock.txt && \
    test "$(wc -l < /opt/g1-nav2-package-lock.txt)" -eq 7

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

LABEL org.opencontainers.image.title="Phanthy Motus G1 Nav2 shadow"
LABEL org.opencontainers.image.description="Nav2-only G1 mapping and navigation shadow"

CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source /nav2_ws/install/setup.bash && exec ros2 run g1_nav2 runtime_supervisor"]
