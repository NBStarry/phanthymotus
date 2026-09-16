# Opt-in OCR PR layer; Core/ActuCore and other Perception business files stay unchanged.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
ARG PYPI_MIRROR=https://nexus.4pd.io/repository/pypi-all/simple/
ARG PUBLIC_PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple/
COPY rapidocr-3.9.1-py3-none-any.whl /tmp/
RUN echo '600885e4e94e0b427abad394fccb0ec1d3c9118a215ca435bf7680aeae0e292b  /tmp/rapidocr-3.9.1-py3-none-any.whl' | sha256sum -c - && \
    env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
    python3 -m pip install --no-cache-dir -i "${PYPI_MIRROR}" \
    /tmp/rapidocr-3.9.1-py3-none-any.whl cffi==1.17.1 numpy==1.26.4 opencv-python==4.11.0.86
COPY pyvips_binary-8.18.6-cp37-abi3-manylinux_2_28_x86_64.whl /tmp/
RUN echo '61409b3784e762398ab6cca9d6f2e1b4bcce73ebc3ec1e821d8039e856915bee  /tmp/pyvips_binary-8.18.6-cp37-abi3-manylinux_2_28_x86_64.whl' | sha256sum -c - && \
    python3 -m pip install --no-cache-dir --no-build-isolation --no-deps \
    -i "${PUBLIC_PYPI_MIRROR}" pyvips==3.1.0 /tmp/pyvips_binary-8.18.6-cp37-abi3-manylinux_2_28_x86_64.whl
COPY ocr-cpu-only.patch /tmp/ocr-cpu-only.patch
RUN cd /work && git apply -p2 --check /tmp/ocr-cpu-only.patch && git apply -p2 /tmp/ocr-cpu-only.patch
COPY perception/tools/verify_ocr_cpu.py /work/tools/verify_ocr_cpu.py
LABEL phanthymotus.ocr.source-revision="7f87c173cee45354686527d9a684fbcbef67fe2c"
