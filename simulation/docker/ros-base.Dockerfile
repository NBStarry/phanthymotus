ARG ROS_BASE_IMAGE=local/phanthy-motus/ros-base:humble-amd64-c124798-v3
FROM ${ROS_BASE_IMAGE}

COPY deploy/ros-base/audio_msgs/ /ros_ws/src/audio_msgs/
RUN rm -rf /ros_ws/build /ros_ws/install /ros_ws/log && \
    cd /ros_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select audio_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN sed -i 's|exec "$@"|source /ros_ws/install/setup.bash\nexec "$@"|' /ros_entrypoint.sh
