ARG ROS_BASE_IMAGE=phanthymotus-sim/ros-base:humble-amd64
FROM ${ROS_BASE_IMAGE}

ARG PYPI_MIRROR=https://nexus.4pd.io/repository/pypi-all/simple/
RUN apt-get update -o Acquire::Retries=3 && \
    apt-get install -y --no-install-recommends ffmpeg libgl1 libglib2.0-0 git && \
    rm -rf /var/lib/apt/lists/*
RUN env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" \
    PyYAML websockets webrtcvad==2.0.10 \
    numpy==1.26.4 onnxruntime==1.20.1 opencv-python==4.11.0.86 \
    torch==2.2.2 torchvision==0.17.2 ftfy regex
# Fetch only missing pinned wheels from the verified public mirror.
ARG PUBLIC_PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple/
RUN python3 -m pip download --no-cache-dir --no-deps -d /tmp/perception-wheels \
    -i "${PUBLIC_PYPI_MIRROR}" sherpa-onnx==1.13.6 sherpa-onnx-core==1.13.6 \
    ultralytics==8.4.144 ultralytics-thop==2.1.6 && \
    env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" \
    --find-links /tmp/perception-wheels sherpa-onnx==1.13.6 ultralytics==8.4.144 \
    numpy==1.26.4 torch==2.2.2 torchvision==0.17.2 opencv-python==4.11.0.86 && \
    rm -rf /tmp/perception-wheels && python3 -m pip check
# Same pinned CLIP revision already verified by the wlcb-23 VOP runtime.
RUN env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" setuptools==75.8.0 packaging==24.2 wheel tqdm && \
    python3 -m pip install --no-cache-dir --no-deps --no-build-isolation \
    'git+https://ghfast.top/https://github.com/ultralytics/CLIP.git@a13192f8cb767260d7dfd98c843b0716593169e7' && \
    python3 -c 'import sherpa_onnx, onnxruntime, torch, torchvision, ultralytics, cv2, clip; print("perception AMD64 dependency imports PASS")'

WORKDIR /work
COPY perception/main.py /work/main.py
COPY perception/plugins/ /work/plugins/
COPY perception/utils/ /work/utils/
COPY perception/deploy/ /deploy/
COPY perception/config.yaml /work/config.yaml
COPY perception/tools/ /work/tools/

ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.revision=${SOURCE_REVISION}
ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    CONFIG_PATH=/work/config.yaml
EXPOSE 15720 15721
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
