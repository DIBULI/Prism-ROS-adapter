ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential \
      libssl-dev \
      libusb-1.0-0-dev \
      pkg-config \
      ros-${ROS_DISTRO}-diagnostic-msgs \
      ros-${ROS_DISTRO}-launch-ros \
      ros-${ROS_DISTRO}-rosidl-default-generators \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY --from=prism_sdk . /tmp/prism-usb-sdk
RUN cmake -S /tmp/prism-usb-sdk -B /tmp/prism-usb-sdk-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/prism-sdk \
      -DPRISM_USB_SDK_BUILD_EXAMPLES=OFF \
      -DBUILD_TESTING=OFF \
    && cmake --build /tmp/prism-usb-sdk-build --parallel \
    && cmake --install /tmp/prism-usb-sdk-build \
    && rm -rf /tmp/prism-usb-sdk /tmp/prism-usb-sdk-build

COPY . /opt/src/prism-ros-adapter
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /opt/src/prism-ros-adapter/ros2_ws \
    && colcon build --merge-install \
         --install-base /opt/prism-ros2 \
         --cmake-args \
           -DCMAKE_BUILD_TYPE=Release \
           -DPrismUsbSdk_DIR=/opt/prism-sdk/lib/cmake/PrismUsbSdk

COPY docker/entrypoint_ros2.sh /prism_entrypoint.sh
ENTRYPOINT ["/prism_entrypoint.sh"]
CMD ["ros2", "launch", "prism_ros_driver", "prism.launch.py"]
