# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: CMakeLists.txt
# Brief: Plain-CMake build for rtk_driver -- NO rclcpp, NO ament, NO colcon
#
# Description:
# rtk_driver is a non-ROS C++ process (CLAUDE.md 0.2): it publishes to the RT-
# plane Zenoh, not to ROS topics, so it must NOT link rclcpp. That is enforced
# HERE, not by the directory (0.2 verbatim): this file finds no ROS package and
# links no ROS target, so a stray rclcpp include would fail to compile -- and it
# does, because the ORIN has no ROS2 / colcon installed at all, which is exactly
# why a plain-CMake package is the right (and only) way to build this today.
# Build:  cd ros2_ws/sensor && cmake -B build && cmake --build build && ctest --test-dir build
# common/ is header-only (INTERFACE): consumed by include path, nothing to link.

cmake_minimum_required(VERSION 3.16)
project(rtk_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)          # 13 CPP-1: exactly C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)       # no GNU extensions (block accidental C++20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

set(COMMON_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include)

# The ROS-free, zenoh-free core: parse -> resolve -> GnssHeading -> envelope.
add_library(rtk_driver_core STATIC
  src/nmea_parser.cc
  src/gnss_heading.cc
  src/gnss_fix.cc
  src/clock_status.cc
  src/heading_resolver.cc
  src/rtk_driver.cc
  src/rtk_config.cc
)
target_include_directories(rtk_driver_core PUBLIC include ${COMMON_INCLUDE})

# The process binary: config -> serial -> RtkDriver -> RT-plane ZenohSink @20Hz.
# It links zenoh-c (installed at /usr/local by the Z1 step). zenoh lives ONLY in
# zenoh_sink.cc (pImpl), so rtk_driver_core and the offline tests never see it.
# The link is OPTIONAL: on a host without libzenohc (e.g. x86 CI) the core lib and
# all offline tests still build; only this executable is skipped.
find_library(ZENOHC_LIB zenohc PATHS /usr/local/lib)
if(ZENOHC_LIB)
  add_executable(rtk_driver src/main.cc src/zenoh_sink.cc)
  target_include_directories(rtk_driver PRIVATE /usr/local/include)
  target_link_libraries(rtk_driver PRIVATE rtk_driver_core ${ZENOHC_LIB} pthread)
else()
  message(STATUS "libzenohc not found -- skipping rtk_driver executable (offline tests still build)")
endif()

# Offline tests: plain executables, no ROS / no hardware, registered with CTest.
enable_testing()
foreach(t nmea_parser gnss_heading gnss_fix clock_status heading_resolver rtk_driver yaml_lite rtk_config)
  add_executable(test_${t} test/test_${t}.cc)
  target_link_libraries(test_${t} PRIVATE rtk_driver_core)
  add_test(NAME test_${t} COMMAND test_${t})
endforeach()
