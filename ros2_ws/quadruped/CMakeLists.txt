# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: CMakeLists.txt
# Brief: Plain-CMake build for quadruped -- core lib + optional rclcpp/CycloneDDS
#
# Description:
# Why plain CMake and not ament, when 13 says quadruped uses rclcpp. Because
# the dependency is one-directional and late: the core (config, tx seam, and in
# later batches the CHS-A codec and Tier 1) has NO ROS in it and must stay
# buildable and testable on a machine with no ROS at all -- that is where the
# unit tests run, and where a reviewer checks them. rclcpp enters only at the
# uplink edge (13 PB-5 limits the rclcpp coupling to three call sites:
# create context, publish Odometry, publish tf). So this file builds:
#
#   quadruped_core   static, ROS-free and DDS-free: always built, always tested
#   quadruped_m20    the process binary: core + entry point; always built
#   (later batches)  the rclcpp uplink and the domain-0 CycloneDDS reader are
#                    added as OPTIONAL targets, guarded by find_package so a
#                    host without them still builds and tests the core
#
# The optional guards are written now, not later, for one reason: a build that
# silently drops a target when a dependency is missing looks identical to a
# build that never had the target. Each guard prints what it did.
#
# Build:   cmake -S ros2_ws/quadruped -B ros2_ws/quadruped/build
#          cmake --build ros2_ws/quadruped/build
#          ctest --test-dir ros2_ws/quadruped/build --output-on-failure
# Install: cmake --install ros2_ws/quadruped/build \
#            --prefix /opt/xbrain_v6/data/install/quadruped
#          (DEC-15 / 99 U83: install root is data/install; the layout
#           <pkg>/lib/<pkg>/<exe> matches what the systemd unit expects)
#
# The one thing NOT to change without reading 13 CPP-1 / PB-5: the standard is
# exactly C++17 and there are no distribution macros anywhere. Both are audited
# by scripts/ci/cxx_discipline_audit.py (rules PB-5a / PB-5b), so a "helpful"
# bump to 20 fails CI rather than surprising the platform decision.

cmake_minimum_required(VERSION 3.16)
project(quadruped LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)          # 13 CPP-1: exactly 17, not "at least"
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)       # no GNU extensions (blocks accidental C++20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# The installed binary must find its shared libraries without help from the
# unit's environment. CMake strips RPATH on install by default, which would
# leave a data/install copy that runs from the build tree and fails under
# systemd -- a difference that only shows up in deployment.
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# common/ is header-only from this package's point of view (config reader, tx
# guard, envelope, units): consumed by include path, nothing to link.
set(COMMON_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include)

# The vendored JSON reader (99 U85 (6)). Header-only, and PRIVATE on purpose:
# it is an implementation detail of the report parser and no public header of
# this package includes it. Making it PUBLIC would put a third-party type in
# the include path of every consumer, and the first one to use it in a
# signature would make the vendoring decision irreversible.
#
# It is used ONLY on the chs_a_rx thread (13 S9.1). Nothing it touches may be
# reached from ctrl or rt_safety: it allocates, and QD-7 forbids allocation on
# the realtime path.
set(THIRD_PARTY_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../common/third_party)

# ---------------------------------------------------------------------------
# Core: no ROS, no DDS, no socket. Everything that can be tested without a
# chassis lives here, and that is deliberate -- the parts that need hardware
# are the parts that cannot be unit tested, so the boundary is drawn to keep
# that set as small as possible.
# ---------------------------------------------------------------------------
add_library(quadruped_core STATIC
  src/quadruped_config.cc
  src/tx_owner.cc
  src/chs_a_codec.cc
  src/chs_a_framer.cc
  src/chs_a_reports.cc
  src/chs_a_session.cc
  src/tier1.cc
)
target_include_directories(quadruped_core PUBLIC include ${COMMON_INCLUDE})
target_include_directories(quadruped_core PRIVATE ${THIRD_PARTY_INCLUDE})
target_link_libraries(quadruped_core PUBLIC pthread)

# ---------------------------------------------------------------------------
# The process binary. Today it is the B0 self-check (see src/main.cc for why it
# refuses to run as a service); the channels land in later batches and attach
# here without changing the layout.
# ---------------------------------------------------------------------------
add_executable(quadruped_m20 src/main.cc)
target_link_libraries(quadruped_m20 PRIVATE quadruped_core)

# DEC-15 (99 U83): install root = /opt/xbrain_v6/data/install, supplied as the
# per-package prefix. DESTINATION lib/${PROJECT_NAME} mirrors the layout every
# other C++ unit already expects (<pkg>/lib/<pkg>/<exe>), which is the path in
# deploy/systemd/xbrain-quadruped.service.
install(TARGETS quadruped_m20 RUNTIME DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Optional: rclcpp uplink (channel three, ROS 2 domain 42 -- batch B8).
# Guarded rather than required so the core still builds on a host without ROS.
# The message is not decoration: a missing target and a skipped target look the
# same in a build log unless one of them says so.
# ---------------------------------------------------------------------------
find_package(rclcpp QUIET)
find_package(nav_msgs QUIET)
find_package(tf2_ros QUIET)
if(rclcpp_FOUND AND nav_msgs_FOUND AND tf2_ros_FOUND)
  message(STATUS "quadruped: rclcpp/nav_msgs/tf2_ros found -- uplink target will "
                 "be built when batch B8 adds its sources")
else()
  message(STATUS "quadruped: rclcpp/nav_msgs/tf2_ros NOT found -- skipping the "
                 "ROS 2 uplink target; core and tests still build")
endif()

# ---------------------------------------------------------------------------
# Optional: chassis DDS domain 0 (channel two -- batch B7). CycloneDDS is used
# through its bare C API, never through rclcpp (13 DDS-1/PB-6): the domain id is
# passed explicitly so the two domains cannot collapse into one via an
# environment variable.
# ---------------------------------------------------------------------------
find_package(CycloneDDS QUIET)
if(CycloneDDS_FOUND)
  message(STATUS "quadruped: CycloneDDS found -- domain-0 reader target will be "
                 "built when batch B7 adds its sources")
else()
  message(STATUS "quadruped: CycloneDDS NOT found -- skipping the domain-0 "
                 "reader; core and tests still build")
endif()

# ---------------------------------------------------------------------------
# Offline tests: plain executables, no ROS, no hardware, registered with CTest.
# The config test needs a writable directory for its fixtures, so it gets the
# build dir as argv[1] rather than writing next to the source.
# ---------------------------------------------------------------------------
enable_testing()
add_executable(test_quadruped_config test/test_quadruped_config.cc)
target_link_libraries(test_quadruped_config PRIVATE quadruped_core)
add_test(NAME test_quadruped_config
         COMMAND test_quadruped_config ${CMAKE_CURRENT_BINARY_DIR})

add_executable(test_tx_owner test/test_tx_owner.cc)
target_link_libraries(test_tx_owner PRIVATE quadruped_core)
add_test(NAME test_tx_owner COMMAND test_tx_owner)

# The two CHS-A tests read frames captured off the real chassis. The path is
# passed as argv[1] rather than resolved relative to the working directory:
# ctest runs from the build tree, and a relative path would silently find
# nothing -- which, for a file-driven test, means every case passes on an empty
# input. Both tests fail loudly when the file is missing or parses to zero
# vectors, for the same reason.
set(CHS_A_GOLDEN ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/chs_a_frames.txt)

add_executable(test_chs_a_codec test/test_chs_a_codec.cc)
target_link_libraries(test_chs_a_codec PRIVATE quadruped_core)
add_test(NAME test_chs_a_codec COMMAND test_chs_a_codec ${CHS_A_GOLDEN})

add_executable(test_chs_a_framer test/test_chs_a_framer.cc)
target_link_libraries(test_chs_a_framer PRIVATE quadruped_core)
add_test(NAME test_chs_a_framer COMMAND test_chs_a_framer ${CHS_A_GOLDEN})

# The report parser is checked against the same capture: the four payloads it
# reads are what the machine actually sent on 2026-09-15, including the Gait 0
# that 13 S5.3's table does not contain.
add_executable(test_chs_a_reports test/test_chs_a_reports.cc)
target_link_libraries(test_chs_a_reports PRIVATE quadruped_core)
add_test(NAME test_chs_a_reports COMMAND test_chs_a_reports ${CHS_A_GOLDEN})

# The session test needs no fixture at all: it injects its dialler and its
# clock, so a three-second timeout and a five-second backoff rung are exercised
# in microseconds. Nothing in it sleeps.
add_executable(test_chs_a_session test/test_chs_a_session.cc)
target_link_libraries(test_chs_a_session PRIVATE quadruped_core)
add_test(NAME test_chs_a_session COMMAND test_chs_a_session)

# Tier 1. Needs no fixture and no golden file: the clock is injected, so the
# 200 ms command timeout is exercised in microseconds. This is the one test in
# the package that must never be quarantined for flakiness, which is why it is
# built not to be flaky rather than trusted not to be.
add_executable(test_tier1 test/test_tier1.cc)
target_link_libraries(test_tier1 PRIVATE quadruped_core)
add_test(NAME test_tier1 COMMAND test_tier1)
