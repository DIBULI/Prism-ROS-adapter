cmake_minimum_required(VERSION 3.10)
project(prism_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  prism_ros_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(PrismUsbSdk 1.0.0 EXACT CONFIG REQUIRED)

catkin_package(CATKIN_DEPENDS
  diagnostic_msgs prism_ros_msgs roscpp sensor_msgs std_msgs)

add_executable(prism_ros_driver_node
  src/prism_ros1_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/lidar_frame_accumulator.cpp
)
target_compile_features(prism_ros_driver_node PRIVATE cxx_std_17)
target_include_directories(prism_ros_driver_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include
)
target_link_libraries(prism_ros_driver_node
  ${catkin_LIBRARIES}
  Prism::UsbSdk
)
add_dependencies(prism_ros_driver_node
  ${catkin_EXPORTED_TARGETS}
  ${prism_ros_msgs_EXPORTED_TARGETS}
)
if(NOT MSVC)
  target_compile_options(prism_ros_driver_node PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

install(TARGETS prism_ros_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
