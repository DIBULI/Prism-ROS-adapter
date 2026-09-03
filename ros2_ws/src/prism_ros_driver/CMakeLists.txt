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
find_package(PrismUsbSdk 1.0.0 EXACT CONFIG REQUIRED)

add_executable(prism_ros_driver_node
  src/prism_ros2_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/lidar_frame_accumulator.cpp
)
target_compile_features(prism_ros_driver_node PRIVATE cxx_std_17)
target_include_directories(prism_ros_driver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include
)
if(COMMAND ament_target_dependencies)
  # Humble/Jazzy keep the compatibility helper. Kilted deprecates it.
  ament_target_dependencies(prism_ros_driver_node
    diagnostic_msgs prism_ros_msgs rclcpp sensor_msgs std_msgs)
  target_link_libraries(prism_ros_driver_node Prism::UsbSdk)
else()
  # Lyrical and newer use only modern imported CMake targets.
  target_link_libraries(prism_ros_driver_node
    ${diagnostic_msgs_TARGETS}
    ${prism_ros_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    Prism::UsbSdk
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
