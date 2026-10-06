ARG ROS_DISTRO=humble
ARG ROS_BASE=ros:${ROS_DISTRO}-ros-base
FROM ${ROS_BASE} AS runtime
ARG PRISM_APT_MIRROR=tsinghua
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
RUN case "$ROS_DISTRO" in humble|jazzy) ;; *) exit 2;; esac \
    && test "$(dpkg --print-architecture)" = arm64
COPY docker/use_tsinghua_mirrors.sh /usr/local/bin/prism-use-tsinghua-mirrors
RUN bash /usr/local/bin/prism-use-tsinghua-mirrors \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-diagnostic-msgs ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-launch-ros \
      ros-${ROS_DISTRO}-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

FROM runtime AS builder
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-builtin-interfaces ros-${ROS_DISTRO}-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
COPY --from=prism_sdk include /opt/prism-sdk/include
COPY --from=prism_sdk cmake/PrismRkLocalSdk.cmake /opt/prism-sdk/cmake/PrismRkLocalSdk.cmake
COPY --from=prism_sdk runtime/linux-arm64/libprism_rklocal_sdk.a /opt/prism-sdk/runtime/linux-arm64/libprism_rklocal_sdk.a
COPY common /opt/src/prism-ros-adapter/common
COPY ros2_ws/src /opt/src/prism-ros-adapter/ros2_ws/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /opt/src/prism-ros-adapter/ros2_ws \
    && colcon build --merge-install --executor sequential \
      --install-base /opt/prism-ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DPRISM_TRANSPORT=rklocal -DPRISM_RKLOCAL_SDK_ROOT=/opt/prism-sdk

FROM runtime
COPY --from=builder /opt/prism-ros2 /opt/prism-ros2
COPY docker/entrypoint_ros2.sh /prism_entrypoint.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /opt/prism-ros2/setup.bash \
    && ldd /opt/prism-ros2/lib/prism_ros_driver/prism_ros_driver_node > /tmp/prism-ldd.txt \
    && ! grep 'not found' /tmp/prism-ldd.txt
ENTRYPOINT ["/bin/bash", "/prism_entrypoint.sh"]
CMD ["ros2", "launch", "prism_ros_driver", "prism.launch.py"]
