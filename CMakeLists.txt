cmake_minimum_required(VERSION 3.16)
project(prism_ros_adapter_common VERSION 1.2.0 LANGUAGES CXX)

include(CTest)

if(BUILD_TESTING)
  set(PRISM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Prism-SDK"
      CACHE PATH "Pinned SDK 1.2.0 root")
  add_executable(prism-lidar-speed-control-test common/tests/lidar_speed_control_test.cpp)
  target_compile_features(prism-lidar-speed-control-test PRIVATE cxx_std_17)
  target_include_directories(prism-lidar-speed-control-test PRIVATE common/include "${PRISM_SDK_ROOT}/include")
  if(NOT MSVC)
    target_compile_options(prism-lidar-speed-control-test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME lidar-speed-control COMMAND prism-lidar-speed-control-test)
  add_executable(prism-receiver-model-test common/tests/receiver_model_test.cpp)
  target_compile_features(prism-receiver-model-test PRIVATE cxx_std_17)
  target_include_directories(prism-receiver-model-test PRIVATE common/include "${PRISM_SDK_ROOT}/include")
  if(NOT MSVC)
    target_compile_options(prism-receiver-model-test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME receiver-model COMMAND prism-receiver-model-test)
  add_executable(prism-lidar-conversion-test common/tests/lidar_conversion_test.cpp common/src/lidar_frame_accumulator.cpp)
  target_compile_features(prism-lidar-conversion-test PRIVATE cxx_std_17)
  target_include_directories(prism-lidar-conversion-test PRIVATE common/include "${PRISM_SDK_ROOT}/include")
  if(NOT MSVC)
    target_compile_options(prism-lidar-conversion-test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME lidar-conversion COMMAND prism-lidar-conversion-test)
  add_executable(prism-navigation-test common/tests/navigation_test.cpp)
  target_compile_features(prism-navigation-test PRIVATE cxx_std_17)
  target_include_directories(prism-navigation-test PRIVATE common/include)
  if(NOT MSVC)
    target_compile_options(prism-navigation-test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME navigation COMMAND prism-navigation-test)
  add_test(NAME ros-message-parity COMMAND diff -ru
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_ws/src/prism_ros_msgs/msg
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/prism_ros_msgs/msg)
  add_test(NAME ros-service-parity COMMAND diff -ru
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_ws/src/prism_ros_msgs/srv
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/prism_ros_msgs/srv)

  add_executable(prism-device-time-resolver-test
    common/tests/device_time_resolver_test.cpp)
  target_compile_features(prism-device-time-resolver-test PRIVATE cxx_std_17)
  target_include_directories(prism-device-time-resolver-test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include)
  if(NOT MSVC)
    target_compile_options(prism-device-time-resolver-test PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME device-time-resolver COMMAND prism-device-time-resolver-test)

  add_executable(prism-lidar-frame-accumulator-test
    common/tests/lidar_frame_accumulator_test.cpp
    common/src/lidar_frame_accumulator.cpp)
  target_compile_features(prism-lidar-frame-accumulator-test PRIVATE cxx_std_17)
  target_include_directories(prism-lidar-frame-accumulator-test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include)
  if(NOT MSVC)
    target_compile_options(prism-lidar-frame-accumulator-test PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME lidar-frame-accumulator
    COMMAND prism-lidar-frame-accumulator-test)
endif()
