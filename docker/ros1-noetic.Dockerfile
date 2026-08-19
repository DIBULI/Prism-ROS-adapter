FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential \
      libssl-dev \
      libusb-1.0-0-dev \
      pkg-config \
      python3-pip \
      ros-noetic-diagnostic-msgs \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-sensor-msgs \
    && pip3 install --no-cache-dir \
         -i https://mirrors.aliyun.com/pypi/simple cmake==3.28.4 \
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
RUN source /opt/ros/noetic/setup.bash \
    && cd /opt/src/prism-ros-adapter/ros1_ws \
    && catkin_make \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/opt/prism-ros1 \
         -DPrismUsbSdk_DIR=/opt/prism-sdk/lib/cmake/PrismUsbSdk \
    && catkin_make install \
         -DCMAKE_INSTALL_PREFIX=/opt/prism-ros1 \
         -DPrismUsbSdk_DIR=/opt/prism-sdk/lib/cmake/PrismUsbSdk

COPY docker/entrypoint_ros1.sh /prism_entrypoint.sh
ENTRYPOINT ["/prism_entrypoint.sh"]
CMD ["roslaunch", "prism_ros_driver", "prism.launch"]
