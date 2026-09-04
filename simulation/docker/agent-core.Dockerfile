ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

ARG PYPI_MIRROR=https://nexus.4pd.io/repository/pypi-all/simple/
ARG DOCKER_VERSION=27.5.1
ARG DOCKER_STATIC_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/static/stable/x86_64
ARG DOCKER_COMPOSE_VERSION=2.37.0
ARG GITHUB_DOWNLOAD_MIRROR=https://ghfast.top/https://github.com
RUN env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" uv

# 与正式版 Agent Core 一样具备 Docker CLI、Compose plugin 和 Docker socket
# 管理能力。二进制均在 wlcb-23 构建期间直接下载，不经 Mac 中转。
RUN curl -fL --retry 5 --retry-delay 2 \
      "${DOCKER_STATIC_MIRROR}/docker-${DOCKER_VERSION}.tgz" \
      -o /tmp/docker.tgz && \
    tar -xzf /tmp/docker.tgz -C /tmp && \
    install -m 0755 /tmp/docker/docker /usr/local/bin/docker && \
    mkdir -p /usr/local/lib/docker/cli-plugins && \
    curl -fL --retry 5 --retry-delay 2 \
      "${GITHUB_DOWNLOAD_MIRROR}/docker/compose/releases/download/v${DOCKER_COMPOSE_VERSION}/docker-compose-linux-x86_64" \
      -o /usr/local/lib/docker/cli-plugins/docker-compose && \
    chmod 0755 /usr/local/lib/docker/cli-plugins/docker-compose && \
    docker --version && docker compose version && \
    rm -rf /tmp/docker /tmp/docker.tgz

WORKDIR /work
COPY agent-core/pyproject.toml agent-core/uv.lock /work/
# 与官方发布 Dockerfile 保持一致：pyproject.toml 是依赖声明的权威来源，构建时重新生成锁。
# 所有依赖均在 wlcb-23 远端下载；不从 Mac 上传 wheel 或其他大文件。
RUN UV_HTTP_TIMEOUT=120 uv lock && \
    UV_HTTP_TIMEOUT=120 uv sync --no-dev --no-install-project && \
    uv pip check --python .venv/bin/python && \
    .venv/bin/python -c "import importlib.metadata as m; import lark_oapi as lark; import lark_oapi.ws.client; from lark_oapi.api.im.v1 import CreateMessageRequest; assert hasattr(lark.ws, 'Client'); print('Feishu Channel SDK import PASS version=' + m.version('lark-oapi'))"

RUN echo "/opt/ros/humble/lib/python3.10/site-packages" >> .venv/lib/python3.10/site-packages/ros2.pth && \
    echo "/opt/ros/humble/local/lib/python3.10/dist-packages" >> .venv/lib/python3.10/site-packages/ros2.pth && \
    echo "/ros_ws/install/audio_msgs/local/lib/python3.10/dist-packages" >> .venv/lib/python3.10/site-packages/ros2.pth && \
    echo "/usr/lib/python3/dist-packages" >> .venv/lib/python3.10/site-packages/system.pth

COPY agent-core/web/ /work/web/
COPY agent-core/src/ /work/src/
COPY agent-core/resource/ /work/resource/
COPY agent-core/deploy/ /deploy/
COPY agent-core/resource/memory/defaults/ /opt/defaults/memory/
COPY agent-core/tests/test_local_services.py /work/tests/test_local_services.py

ARG SOURCE_REVISION=unknown
ENV IMAGE_TAG=${SOURCE_REVISION}-amd64
RUN printf '%s-amd64\n' "${SOURCE_REVISION}" > /work/VERSION

ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15678
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && .venv/bin/python src/start.py"]
