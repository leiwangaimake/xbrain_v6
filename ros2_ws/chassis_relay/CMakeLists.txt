# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: CMakeLists.txt
# Brief: Plain-CMake build for chassis_relay -- core lib + zenoh-guarded binary
#
# Description:
# Same split as quadruped's build and for the same reason: everything that can
# be tested without a router lives in a ROS-free, zenoh-free static core
# (relay_core here), always built and always tested; the transport edge
# (relay_session.cc + main.cc) needs zenoh-c and is guarded, so a host without
# zenoh still builds and runs the whole offline suite. The guard PRINTS what it
# did -- a build that silently drops a target when a dependency is missing
# looks identical to a build that never had the target.
#
# This package never links rclcpp or any ROS type: it sits on the emergency
# stop path (CLAUDE.md 5.3 names chassis_relay as the reason common/ must stay
# ROS-free). colcon can build it as a plain CMake package; the "no rclcpp"
# property is enforced by what this file links, not by which directory it
# lives in.
#
# Build:   cmake -S ros2_ws/chassis_relay -B ros2_ws/chassis_relay/build
#          cmake --build ros2_ws/chassis_relay/build
#          ctest --test-dir ros2_ws/chassis_relay/build --output-on-failure
# Install: cmake --install ros2_ws/chassis_relay/build \
#            --prefix /opt/xbrain_v6/data/install/chassis_relay
#          (DEC-15 / 99 U83: install root is data/install; the layout
#           <pkg>/lib/<pkg>/<exe> is the ExecStart path of
#           deploy/systemd/xbrain-chassis-relay.service.)
#
# The one thing NOT to change without reading 13 CPP-1 / PB-5: the standard is
# exactly C++17 and there are no distribution macros anywhere.

cmake_minimum_required(VERSION 3.16)
project(chassis_relay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)          # 13 CPP-1: exactly 17, not "at least"
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)       # no GNU extensions (blocks accidental C++20)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# The installed binary must find libzenohc without help from the unit's
# environment; CMake strips RPATH on install by default (same note and same
# fix as quadruped and rtk_driver).
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# common/ is header-only from this package's point of view (yaml_lite, the
# frozen QoS table, the lock-free slot): consumed by include path, nothing to
# link. The vendored JSON reader is TEST-only here: the forward path parses
# nothing (envelope_rebuild.cc is a hand scanner), so nlohmann never enters a
# non-test target of this package.
set(COMMON_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include)
set(THIRD_PARTY_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../common/third_party)

# ---------------------------------------------------------------------------
# Core: no zenoh, no ROS, no threads. The whitelist table, the envelope
# rebuild, the forward step, the config loader, the session config document.
# ---------------------------------------------------------------------------
add_library(chassis_relay_core STATIC
  src/relay_keys.cc
  src/envelope_rebuild.cc
  src/relay_core.cc
  src/relay_config.cc
  src/relay_session_config.cc
)
target_include_directories(chassis_relay_core PUBLIC include ${COMMON_INCLUDE})

# ---------------------------------------------------------------------------
# The binary, guarded on zenoh-c. Unlike quadruped there is no degraded mode:
# a relay that cannot open sessions has no job at all, so a missing zenoh
# skips the TARGET loudly instead of shipping a binary that cannot run.
# ---------------------------------------------------------------------------
find_library(ZENOHC_LIBRARY NAMES zenohc HINTS /usr/local/lib)
find_path(ZENOHC_INCLUDE_DIR NAMES zenoh.h HINTS /usr/local/include)
if(ZENOHC_LIBRARY AND ZENOHC_INCLUDE_DIR)
  message(STATUS "chassis_relay: zenoh-c found -- building the process binary")
  add_executable(chassis_relay src/relay_session.cc src/main.cc)
  target_include_directories(chassis_relay PRIVATE include ${COMMON_INCLUDE}
                             ${ZENOHC_INCLUDE_DIR})
  target_link_libraries(chassis_relay PRIVATE chassis_relay_core
                        ${ZENOHC_LIBRARY} pthread)
  # DEC-15 (99 U83): <pkg>/lib/<pkg>/<exe> under the per-package prefix --
  # exactly the ConditionPathExists / ExecStart path of the systemd unit.
  install(TARGETS chassis_relay RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "chassis_relay: zenoh-c NOT found -- skipping the process "
                 "binary; the core and every offline test still build")
endif()

# ---------------------------------------------------------------------------
# Offline tests: plain executables, no zenoh, no router, registered with
# CTest. Arguments follow the repo conventions: the contract path for the key
# table (a missing file FAILS, never skips), the build dir for the config
# test's fixtures.
# ---------------------------------------------------------------------------
enable_testing()
set(CONTRACT_MD ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/11-接口契约.md)

add_executable(test_relay_keys test/test_relay_keys.cc)
target_link_libraries(test_relay_keys PRIVATE chassis_relay_core)
add_test(NAME test_relay_keys COMMAND test_relay_keys ${CONTRACT_MD})

add_executable(test_envelope_rebuild test/test_envelope_rebuild.cc)
target_link_libraries(test_envelope_rebuild PRIVATE chassis_relay_core)
target_include_directories(test_envelope_rebuild PRIVATE ${THIRD_PARTY_INCLUDE})
add_test(NAME test_envelope_rebuild COMMAND test_envelope_rebuild)

add_executable(test_relay_core test/test_relay_core.cc)
target_link_libraries(test_relay_core PRIVATE chassis_relay_core)
target_include_directories(test_relay_core PRIVATE ${THIRD_PARTY_INCLUDE})
add_test(NAME test_relay_core COMMAND test_relay_core)

add_executable(test_relay_config test/test_relay_config.cc)
target_link_libraries(test_relay_config PRIVATE chassis_relay_core)
add_test(NAME test_relay_config
         COMMAND test_relay_config ${CMAKE_CURRENT_BINARY_DIR})
