cmake_minimum_required(VERSION 3.10)
project(prism_ros_driver)

find_package(ament_cmake REQUIRED)
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  # Prime Humble's generated ament library resolver. On AArch64/QEMU its
  # generic _lib lookup can otherwise cache a false NOTFOUND result even when
  # the exported library exists in the ROS prefix.
  get_filename_component(PRISM_ROS_PREFIX
    "${ament_cmake_DIR}/../../.." ABSOLUTE)
  find_library(PRISM_HUMBLE_BUILTIN_INTERFACES_GENERATOR_C
    NAMES builtin_interfaces__rosidl_generator_c
    PATHS "${PRISM_ROS_PREFIX}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
    REQUIRED)
  mark_as_advanced(PRISM_HUMBLE_BUILTIN_INTERFACES_GENERATOR_C)
endif()
find_package(diagnostic_msgs REQUIRED)
find_package(prism_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(PRISM_TRANSPORT "usb" CACHE STRING "Prism transport: usb or rklocal")
set_property(CACHE PRISM_TRANSPORT PROPERTY STRINGS usb rklocal)
if(PRISM_TRANSPORT STREQUAL "rklocal")
  set(PRISM_RKLOCAL_SDK_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/Prism-SDK"
    CACHE PATH "Prism-SDK 1.2.0 distribution root")
  include("${PRISM_RKLOCAL_SDK_ROOT}/cmake/PrismRkLocalSdk.cmake")
  set(PRISM_TRANSPORT_TARGET Prism::RkLocal)
elseif(PRISM_TRANSPORT STREQUAL "usb")
# Locate the bundled binary SDK prefix automatically so a bare `colcon build`
# works out of the box. Precedence: explicit PrismUsbSdk_DIR / CMAKE_PREFIX_PATH
# from the caller > PRISM_USB_SDK_PREFIX env (documented in README) > newest
# matching in-repo prefix under third_party/Prism-SDK/runtime/ros/.
if(NOT PrismUsbSdk_DIR)
  if(DEFINED ENV{PRISM_USB_SDK_PREFIX})
    list(APPEND CMAKE_PREFIX_PATH "$ENV{PRISM_USB_SDK_PREFIX}")
  else()
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      set(_prism_arch_tag "arm64")
    else()
      set(_prism_arch_tag "x64")
    endif()
    set(_prism_sdk_prefix
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/Prism-SDK/runtime/ros/linux-${_prism_arch_tag}")
    if(EXISTS "${_prism_sdk_prefix}/lib/cmake/PrismUsbSdk")
      list(APPEND CMAKE_PREFIX_PATH "${_prism_sdk_prefix}")
      message(STATUS "Prism USB SDK prefix (bundled): ${_prism_sdk_prefix}")
    endif()
  endif()
endif()
find_package(PrismUsbSdk 1.2.0 EXACT CONFIG REQUIRED)
  set(PRISM_TRANSPORT_TARGET Prism::UsbSdk)
else()
  message(FATAL_ERROR "PRISM_TRANSPORT must be usb or rklocal")
endif()
message(STATUS "Prism ROS transport: ${PRISM_TRANSPORT}")
find_package(Threads REQUIRED)

add_executable(prism_ros_driver_node
  src/prism_ros2_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/lidar_frame_accumulator.cpp
)
target_compile_features(prism_ros_driver_node PRIVATE cxx_std_17)
if(PRISM_TRANSPORT STREQUAL "rklocal")
  target_compile_definitions(prism_ros_driver_node PRIVATE PRISM_ROS_RKLOCAL=1)
endif()
target_include_directories(prism_ros_driver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include
)
if(COMMAND ament_target_dependencies)
  # Humble/Jazzy keep the compatibility helper. Kilted deprecates it.
  ament_target_dependencies(prism_ros_driver_node
    diagnostic_msgs prism_ros_msgs rclcpp sensor_msgs std_msgs)
  target_link_libraries(prism_ros_driver_node ${PRISM_TRANSPORT_TARGET} Threads::Threads)
else()
  # Lyrical and newer use only modern imported CMake targets.
  target_link_libraries(prism_ros_driver_node
    ${diagnostic_msgs_TARGETS}
    ${prism_ros_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    ${PRISM_TRANSPORT_TARGET} Threads::Threads
  )
endif()
if(NOT MSVC)
  target_compile_options(prism_ros_driver_node PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

install(TARGETS prism_ros_driver_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
