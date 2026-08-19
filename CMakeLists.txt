cmake_minimum_required(VERSION 3.16)
project(prism_ros_adapter_common LANGUAGES CXX)

include(CTest)

if(BUILD_TESTING)
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
endif()
