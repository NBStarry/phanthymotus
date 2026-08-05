ARG ROS_BASE_IMAGE=ros:humble@sha256:5417e56962ff6e15d4cf9b2f78a71a78f3901f47cd78696b575b7eecdb54eb78
FROM ${ROS_BASE_IMAGE}

ENV CONFIG_PATH=/config/general-navigation.yaml \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /work
RUN mkdir -p /work/plugins/general_navigation /config /deploy /tests

COPY perception/main.py /work/main.py
COPY perception/plugins/general_navigation/__init__.py \
     perception/plugins/general_navigation/backend.py \
     perception/plugins/general_navigation/contract.py \
     perception/plugins/general_navigation/core.py \
     perception/plugins/general_navigation/plugin.py \
     /work/plugins/general_navigation/
COPY perception/plugins/general_navigation/deploy/config.yaml /config/general-navigation.yaml
COPY perception/plugins/general_navigation/deploy/service.yml /deploy/service.yml
COPY perception/plugins/general_navigation/deploy/tests/mcp_probe.py /tests/mcp_probe.py

EXPOSE 15720

LABEL org.opencontainers.image.title="Phanthy Motus General Navigation Perception"
LABEL org.opencontainers.image.description="G1 Nav2 shadow-only Perception card"

CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
