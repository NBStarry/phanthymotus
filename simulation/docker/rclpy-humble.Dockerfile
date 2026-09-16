# Dependency-only candidate; public application files are inherited unchanged.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
COPY docker/rclpy-humble-executor.patch /tmp/rclpy-humble-executor.patch
# Local adaptation of ros2/rclpy#1150 plus a nested QoS handle guard.
# This is not an official released backport.
# Refuse a different dependency implementation rather than apply a fuzzy patch.
RUN echo 'e854591961c38e95e465c2040400c18111b8b3ce445cae195345cabbaaf6c010  /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py' | sha256sum -c - && \
    patch --batch --fuzz=0 -d /opt/ros/humble/local/lib/python3.10/dist-packages -p1 < /tmp/rclpy-humble-executor.patch && \
    rm /tmp/rclpy-humble-executor.patch
LABEL phanthymotus.sim.dependency-fix="rclpy-humble-invalidhandle-local-backport-v2"
