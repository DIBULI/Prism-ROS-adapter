cmake_minimum_required(VERSION 3.10)
project(prism_ros_driver)

include("${CMAKE_CURRENT_SOURCE_DIR}/../../../common/cmake/InstallLicenses.cmake")

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  prism_ros_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(PrismUsbSdk 1.3.0 EXACT CONFIG REQUIRED)
find_package(Threads REQUIRED)

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
  Prism::UsbSdk Threads::Threads
)
add_dependencies(prism_ros_driver_node
  ${catkin_EXPORTED_TARGETS}
  ${prism_ros_msgs_EXPORTED_TARGETS}
)
if(NOT MSVC)
  target_compile_options(prism_ros_driver_node PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

get_target_property(_sdk_type Prism::UsbSdk TYPE)
if(_sdk_type STREQUAL "SHARED_LIBRARY")
  install(FILES $<TARGET_FILE:Prism::UsbSdk> DESTINATION lib)
  set_property(TARGET prism_ros_driver_node APPEND PROPERTY INSTALL_RPATH "$ORIGIN/..")
endif()
install(TARGETS prism_ros_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
