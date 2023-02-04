ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-dev

RUN mkdir -p ~/ros2_ws/src/ndt
WORKDIR /ros2_ws

COPY . ./src/ndt

RUN source /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y

COPY entrypoint.sh .
RUN ["chmod", "+x", "entrypoint.sh"]

ENTRYPOINT ["./entrypoint.sh"]
CMD ["/bin/bash"]
