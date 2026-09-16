# Optional CPU offscreen rendering layer; public application images are untouched.
ARG SIM_DRIVER_IMAGE
FROM ${SIM_DRIVER_IMAGE}
RUN sed -i 's|http://mirrors.tuna.tsinghua.edu.cn|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    apt-get update -o Acquire::Retries=3 -o APT::Update::Error-Mode=any && \
    apt-get install -y --no-install-recommends libosmesa6 && \
    rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa
COPY sim-driver/ /work/
