FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/workspace
ENV ROS_LOG_DIR=/tmp/ros-log
ENV PIP_DISABLE_PIP_VERSION_CHECK=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/thesis_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/thesis_requirements.txt

WORKDIR ${ROS_WS}
COPY thesis_ws/src ./src

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "decision_layer", "docker_pipeline.launch.py"]
