ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

ARG PYPI_MIRROR=https://nexus.4pd.io/repository/pypi-all/simple/
RUN env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" PyYAML websockets

WORKDIR /work
COPY perception/main.py /work/main.py
COPY perception/plugins/ /work/plugins/
COPY perception/utils/ /work/utils/
COPY perception/deploy/ /deploy/
COPY simulation/config/perception-p0.yaml /work/config.yaml

# 上游生产容器使用 host network，因此默认注册地址是 localhost。
# 仿真栈使用隔离 bridge，仅在本镜像覆盖为可配置的容器 DNS 地址。
RUN sed -i 's|"url":  f"http://localhost:{mcp_port}/mcp"|"url":  os.environ.get("MCP_ADVERTISE_URL", f"http://localhost:{mcp_port}/mcp")|' /work/main.py && \
    grep -F 'MCP_ADVERTISE_URL' /work/main.py

ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    CONFIG_PATH=/work/config.yaml
EXPOSE 15720 15721
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
