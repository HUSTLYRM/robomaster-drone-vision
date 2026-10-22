# 轻量双目 VO（C++17 + OpenCV，无 ROS）。只用 OpenCV 4.2 起就有的接口，便于在机载板（Ubuntu 20.04 apt 版 OpenCV）上编译。
#   cmake -S stereo/vo -B stereo/vo/build && cmake --build stereo/vo/build -j4
#   ctest --test-dir stereo/vo/build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(stereo_vo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video imgcodecs)

add_library(vo_core src/calib.cpp src/vo.cpp)
target_include_directories(vo_core PUBLIC src)
target_link_libraries(vo_core PUBLIC ${OpenCV_LIBS})

add_executable(stereo_vo src/main.cpp src/v4l2_stereo.cpp)
target_link_libraries(stereo_vo PRIVATE vo_core)

option(VO_BUILD_TESTS "构建 gtest 单测" ON)
if(VO_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  add_executable(test_vo tests/test_vo.cpp)
  target_link_libraries(test_vo PRIVATE vo_core GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_vo)
endif()
