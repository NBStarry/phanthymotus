ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

ARG PYPI_MIRROR=https://nexus.4pd.io/repository/pypi-all/simple/
RUN env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" PyYAML requests

WORKDIR /work
COPY actucore/main.py /work/main.py
COPY actucore/plugins/ /work/plugins/
COPY actucore/config.yaml /work/config.yaml
COPY actucore/deploy/ /deploy/
COPY perception/utils/logsafe.py /work/logsafe.py

# Production uses host networking. The simulation bridge needs a reachable
# advertised address while keeping the upstream application source unchanged.
RUN sed -i 's|"url":  f"http://localhost:{mcp_port}/mcp"|"url":  os.environ.get("MCP_ADVERTISE_URL", f"http://localhost:{mcp_port}/mcp")|' /work/main.py && \
    grep -F 'MCP_ADVERTISE_URL' /work/main.py

ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.revision=${SOURCE_REVISION}
ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    CONFIG_PATH=/work/config.yaml
EXPOSE 15730
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
