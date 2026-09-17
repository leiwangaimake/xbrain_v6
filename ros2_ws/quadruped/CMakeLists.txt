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
# C is enabled alongside C++ for ONE reason: idlc emits C, and the generated
# type support has to be compiled. Without it CMake has no language for a .c
# file and drops it SILENTLY -- the library builds, and the link of anything
# that uses a descriptor fails with an undefined reference that names the
# symbol and not the cause.
#
# This does not relax CPP-1: the C++ standard below is still exactly 17, and no
# hand-written C is added to this package.
project(quadruped LANGUAGES C CXX)

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
  src/rt_keys.cc
  src/rt_payloads.cc
  src/mode_machine.cc
  src/odometry.cc
  src/dds_names.cc
  src/chassis_socket.cc
  src/process.cc
  src/rt_parse.cc
  src/rt_bridge.cc
)
target_include_directories(quadruped_core PUBLIC include ${COMMON_INCLUDE})
target_include_directories(quadruped_core PRIVATE ${THIRD_PARTY_INCLUDE})
target_link_libraries(quadruped_core PUBLIC pthread)

# ---------------------------------------------------------------------------
# The process binary. It now RUNS: main.cc loads the resolved snapshot, builds
# the assembly in process.h and starts the two threads. What it still does not
# start is the two OPTIONAL targets below (the domain-0 reader and the rclcpp
# uplink) -- they are separate libraries precisely so the core stays buildable
# without ROS, and linking them in unconditionally would undo that.
# ---------------------------------------------------------------------------
add_executable(quadruped_m20 src/main.cc)
target_link_libraries(quadruped_m20 PRIVATE quadruped_core)

# The RT plane is linked in when zenoh-c is present. The guard is deferred to
# the bottom of this file, after quadruped_rt is defined -- CMake needs the
# target to exist before it can be linked, and defining it earlier would put
# the optional dependency ahead of the core it depends on.

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
find_package(geometry_msgs QUIET)
if(rclcpp_FOUND AND nav_msgs_FOUND AND tf2_ros_FOUND AND geometry_msgs_FOUND)
  message(STATUS "quadruped: rclcpp/nav_msgs/tf2_ros found -- building the B8 "
                 "uplink target")
  # A SEPARATE library, so the ROS dependency stops here. quadruped_core, its
  # tests and every other consumer keep building on a machine with no ROS --
  # 13 S5.3 requires that, because chassis_relay is on the emergency-stop path
  # and must never pull rclcpp in.
  add_library(quadruped_uplink STATIC src/uplink.cc)
  target_include_directories(quadruped_uplink PUBLIC include ${COMMON_INCLUDE})
  # The PLAIN signature on both calls. ament_target_dependencies uses the
  # non-keyword form internally, and CMake refuses to mix the two on one
  # target -- the error names the target but not the cause, so it is worth
  # saying here rather than rediscovering it.
  ament_target_dependencies(quadruped_uplink rclcpp nav_msgs tf2_ros geometry_msgs)
  target_link_libraries(quadruped_uplink quadruped_core)

  # Registered only when ROS is present, so a machine without it still runs the
  # whole offline suite. The test brings the context up on the CONFIGURED
  # domain (42) rather than a test-only one: a test that quietly used a
  # different domain would pass on an implementation that reads ROS_DOMAIN_ID
  # from the environment, which is the mistake DDS-1 cares about most.
  #
  # It proves the publish call returned. It does NOT prove a packet left the
  # machine -- that is T-ODOM-1 and needs a second process.
  enable_testing()
  add_executable(test_uplink test/test_uplink.cc)
  ament_target_dependencies(test_uplink rclcpp nav_msgs tf2_ros geometry_msgs)
  target_link_libraries(test_uplink quadruped_uplink)
  add_test(NAME test_uplink COMMAND test_uplink)

  # The allocation measurement behind 13 V-69. It replaces the global operator
  # new, which is why it is a SEPARATE binary: linked into test_uplink it would
  # count that test's own allocations as well, and the number would stop
  # meaning "the cost of one control period".
  add_executable(test_uplink_alloc test/test_uplink_alloc.cc)
  ament_target_dependencies(test_uplink_alloc rclcpp nav_msgs tf2_ros geometry_msgs)
  target_link_libraries(test_uplink_alloc quadruped_uplink)
  add_test(NAME test_uplink_alloc COMMAND test_uplink_alloc)
else()
  message(STATUS "quadruped: rclcpp/nav_msgs/tf2_ros NOT found -- skipping the "
                 "ROS 2 uplink target; core and tests still build")
endif()

# ---------------------------------------------------------------------------
# Optional: the RT-plane Zenoh session (11 S1.1.2). Guarded like the other two
# so the core and its offline tests still build on a host without zenoh-c.
#
# The CONFIG test is registered unconditionally below, outside this guard: it
# compares this package's config string with session_factory.py's validator and
# needs neither zenoh nor a router. That separation is the point -- the half
# that fails silently is the half that must be checkable everywhere.
# ---------------------------------------------------------------------------
find_library(ZENOHC_LIBRARY NAMES zenohc HINTS /usr/local/lib)
find_path(ZENOHC_INCLUDE_DIR NAMES zenoh.h HINTS /usr/local/include)
if(ZENOHC_LIBRARY AND ZENOHC_INCLUDE_DIR)
  message(STATUS "quadruped: zenoh-c found -- building the RT-plane session")
  add_library(quadruped_rt STATIC src/rt_session.cc src/rt_session_config.cc
                                  src/rt_runtime.cc)
  target_include_directories(quadruped_rt PUBLIC include ${COMMON_INCLUDE}
                             ${ZENOHC_INCLUDE_DIR})
  target_link_libraries(quadruped_rt PUBLIC quadruped_core ${ZENOHC_LIBRARY})
else()
  message(STATUS "quadruped: zenoh-c NOT found -- skipping the RT-plane "
                 "session; core and tests still build")
endif()

# ---------------------------------------------------------------------------
# Optional: chassis DDS domain 0 (channel two -- batch B7). CycloneDDS is used
# through its bare C API, never through rclcpp (13 DDS-1/PB-6): the domain id is
# passed explicitly so the two domains cannot collapse into one via an
# environment variable.
# ---------------------------------------------------------------------------
find_package(CycloneDDS QUIET)
find_program(IDLC_EXECUTABLE idlc)
if(CycloneDDS_FOUND AND IDLC_EXECUTABLE)
  message(STATUS "quadruped: CycloneDDS + idlc found -- building the B7 "
                 "domain-0 reader")
  # The type support is GENERATED, from an IDL this package owns rather than
  # from the one that ships with sensor_msgs. The stock IDL produces the type
  # name "sensor_msgs::msg::Imu", while ROS 2 puts "sensor_msgs::msg::dds_::Imu_"
  # on the wire -- a reader built from the stock name matches nothing, and the
  # symptom is silence (CLAUDE.md 5.4, measured 2026-09-15). The IDL carries the
  # full reasoning.
  set(CHS_B_IDL ${CMAKE_CURRENT_SOURCE_DIR}/idl/chassis_dds_types.idl)
  set(CHS_B_GEN_C ${CMAKE_CURRENT_BINARY_DIR}/chassis_dds_types.c)
  set(CHS_B_GEN_H ${CMAKE_CURRENT_BINARY_DIR}/chassis_dds_types.h)
  add_custom_command(
    OUTPUT ${CHS_B_GEN_C} ${CHS_B_GEN_H}
    COMMAND ${IDLC_EXECUTABLE} ${CHS_B_IDL}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${CHS_B_IDL}
    COMMENT "idlc: generating the domain-0 type support")

  add_library(quadruped_chs_b STATIC src/chs_b.cc ${CHS_B_GEN_C})
  target_include_directories(quadruped_chs_b PUBLIC include ${COMMON_INCLUDE}
                             ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(quadruped_chs_b PUBLIC quadruped_core CycloneDDS::ddsc)
  # The generated C is not ours to hold to our warning settings.
  set_source_files_properties(${CHS_B_GEN_C} PROPERTIES COMPILE_OPTIONS "-w")

  # The one test that can verify DDS-1: it sets ROS_DOMAIN_ID to the WRONG
  # value and requires the participant to have joined the configured one
  # anyway. An implementation that read the environment would put this
  # participant on the same domain as the uplink, and the result is a reader
  # that never receives anything with no error anywhere.
  enable_testing()
  add_executable(test_chs_b test/test_chs_b.cc)
  target_link_libraries(test_chs_b quadruped_chs_b)
  add_test(NAME test_chs_b COMMAND test_chs_b)
else()
  message(STATUS "quadruped: CycloneDDS or idlc NOT found -- skipping the "
                 "domain-0 reader; core and tests still build")
endif()

# ---------------------------------------------------------------------------
# The binary's RT plane, attached now that quadruped_rt exists.
#
# *** QUADRUPED_HAVE_RT IS NOT A DISTRIBUTION MACRO, and 13 PB-5 is about those.
# PB-5 forbids branching on humble-vs-jazzy because D-45 is undecided and a
# macro written today would have to be torn out when it lands. This one asks a
# different question -- "is the optional dependency present in THIS build" --
# which is the same question the find_package guards above already ask, and it
# has an answer that does not depend on any pending decision.
#
# The binary is still built without zenoh, and it SAYS SO at startup rather than
# coming up quietly with no way to receive a command. A process that looks alive
# while nothing can reach it is the failure CLAUDE.md 3.2 puts first.
if(TARGET quadruped_rt)
  target_link_libraries(quadruped_m20 PRIVATE quadruped_rt)
  target_compile_definitions(quadruped_m20 PRIVATE QUADRUPED_HAVE_RT=1)
  message(STATUS "quadruped: quadruped_m20 links the RT plane")
else()
  message(WARNING "quadruped: quadruped_m20 is being built WITHOUT the RT "
                  "plane -- it will run, drive the chassis and hold it safe, "
                  "and it will not be able to RECEIVE a velocity command. "
                  "Install zenoh-c to get a complete binary.")
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

# The key table is checked against the CONTRACT ITSELF: every suffix must appear
# verbatim in docs/11. That is the only assertion that catches a mistyped key,
# which otherwise produces a process that publishes happily and is heard by
# nobody. The path is passed in rather than derived, and a missing file FAILS
# rather than skipping -- a comparison that cannot run is not a comparison.
set(CONTRACT_MD ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/11-接口契约.md)
add_executable(test_rt_keys test/test_rt_keys.cc)
target_link_libraries(test_rt_keys PRIVATE quadruped_core)
add_test(NAME test_rt_keys COMMAND test_rt_keys ${CONTRACT_MD})

# The published data objects. Every case PARSES the produced text rather than
# substring-matching it: invalid JSON on state/robot makes the consumer see
# nothing at all, which reads as the robot having stopped reporting.
add_executable(test_rt_payloads test/test_rt_payloads.cc)
target_link_libraries(test_rt_payloads PRIVATE quadruped_core)
target_include_directories(test_rt_payloads PRIVATE ${THIRD_PARTY_INCLUDE})
add_test(NAME test_rt_payloads COMMAND test_rt_payloads ${CHS_A_GOLDEN})

# The mode triple. Clock injected, so the 5 s switch timeout and the 3.5 s
# external-transition hold are exercised in microseconds.
add_executable(test_mode_machine test/test_mode_machine.cc)
target_link_libraries(test_mode_machine PRIVATE quadruped_core)
add_test(NAME test_mode_machine COMMAND test_mode_machine)

# Dead reckoning. The headline cases assert 13 S4.4's four baseline rows as
# NUMBERS -- that table is T-ODOM-2's acceptance basis, and "it grows" is
# satisfied by any rate at all including the 2.45x-optimistic one the document
# used to carry.
add_executable(test_odometry test/test_odometry.cc)
target_link_libraries(test_odometry PRIVATE quadruped_core)
add_test(NAME test_odometry COMMAND test_odometry)

# The ROS-to-DDS name mapping. Tested WITHOUT a DDS stack on purpose: a wrong
# mapping produces a participant that comes up, a topic that exists and not one
# sample, so there is nothing to observe at run time.
add_executable(test_dds_names test/test_dds_names.cc)
target_link_libraries(test_dds_names PRIVATE quadruped_core)
add_test(NAME test_dds_names COMMAND test_dds_names)

# Real sockets against an in-process loopback listener, not a mock. EAGAIN,
# EINPROGRESS and the two different meanings of a zero-length read are kernel
# behaviour; a test double would encode what the author believed instead.
add_executable(test_chassis_socket test/test_chassis_socket.cc)
target_link_libraries(test_chassis_socket PRIVATE quadruped_core)
add_test(NAME test_chassis_socket COMMAND test_chassis_socket)

# The assembly. It drives CtrlTick and RxPump DIRECTLY, with no threads running
# and the clock supplied per call, and it feeds RxPump the frames captured off
# the real chassis on 2026-09-15 rather than frames this repository invented.
#
# Both choices are load-bearing. Without the first, a 200 ms Tier 1 timeout
# would take 200 ms of wall time and the ordering inside one control period --
# which is where the defect found while writing this test actually was -- would
# be unobservable. Without the second, the test would agree with the parser
# about a report format that the chassis does not send.
# The RT plane's inbound parsing. No zenoh, no router, no robot: the safety
# rule of 11 S3.0.1 lives in the decoder, and keeping it testable without a
# transport is what lets "a truncated estop still stops" be a case at all.
# The RT-plane session CONFIG, compared against the other implementation of it.
# No zenoh and no router: the assertions are about the bytes handed to zenoh,
# and 11 S1.1.2's json5 block disagrees with them (see the test's header). The
# python path is passed in rather than derived, and a missing file FAILS rather
# than skipping -- a comparison that cannot run is not a comparison.
set(SESSION_FACTORY_PY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../xbrain/common/zenoh/session_factory.py)
add_executable(test_rt_session test/test_rt_session.cc src/rt_session_config.cc)
target_include_directories(test_rt_session PRIVATE include ${COMMON_INCLUDE})
add_test(NAME test_rt_session COMMAND test_rt_session ${SESSION_FACTORY_PY})

# The routing: which message causes which call, and what gets acked. No router,
# no chassis, no threads -- the handlers are called with bytes and the acks are
# captured, so "a truncated estop still stops AND is still acked" is a case
# rather than an intention.
add_executable(test_rt_bridge test/test_rt_bridge.cc)
target_link_libraries(test_rt_bridge PRIVATE quadruped_core)
target_include_directories(test_rt_bridge PRIVATE ${THIRD_PARTY_INCLUDE})
add_test(NAME test_rt_bridge COMMAND test_rt_bridge)

add_executable(test_rt_parse test/test_rt_parse.cc)
target_link_libraries(test_rt_parse PRIVATE quadruped_core)
target_include_directories(test_rt_parse PRIVATE ${THIRD_PARTY_INCLUDE})
add_test(NAME test_rt_parse COMMAND test_rt_parse)

add_executable(test_process test/test_process.cc)
target_link_libraries(test_process PRIVATE quadruped_core)
add_test(NAME test_process COMMAND test_process ${CHS_A_GOLDEN})
