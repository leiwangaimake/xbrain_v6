# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: CMakeLists.txt
# Brief: Builds the minimal no-ROS translation unit against xbrain::common and
#        writes out the resolved build facts for the Python driver to assert on
#
# Description:
# What problem this solves. CFG-CM-14 asks for a translation unit that links
# common/ while linking no ROS library, and for four properties of the build
# itself: standard exactly 17, standard required, extensions off, and the four
# warning flags. The first three are CMake target properties -- they do not all
# show up on a compiler command line, and CXX_STANDARD_REQUIRED in particular is
# invisible on a toolchain that happens to support C++17 anyway. Asserting them
# by grepping xbrain/common/CMakeLists.txt would be a test of the text rather
# than of the build, which is the "定义式冒充实测结论" shape in CLAUDE.md S3.2.
# So this project asks CMake for the values it actually resolved and writes them
# to build_facts.txt at generate time; the driver reads that file.
#
# Which sections this serves: 13 S9.3 CPP-1 (exactly C++17), 13 S2.0 PB-5 (no
# distribution macro), 19 S1.1 (warning flags and the debug assertions), 19 S1.2
# and 11 S1.1.6 CRL-4 / CRL-5 (nothing in common/ may drag in ROS, because
# chassis_relay is on the emergency-stop path and could then not use it).
#
# What is deliberately absent, and why each absence is the point:
#   * no find_package(ament_cmake), no find_package(rclcpp), no
#     find_package(rosidl_*). Nothing here puts a ROS prefix on the include path
#     or a ROS library on the link line, which is precisely the condition the
#     gate needs.
#   * no if() on the ROS distribution or the OS version. PB-5 forbids it while
#     the D-45 baseline is undecided.
#   * no compile flag of its own. Every flag on the target comes from
#     xbrain_common_apply_cxx_baseline(), so this file cannot pass by setting the
#     flags locally while the shared baseline is broken -- a mutation that empties
#     the baseline function has to show up here.
#
# The trap this layout exists to catch. add_subdirectory() runs
# xbrain/common/CMakeLists.txt in its own directory scope, so the CMAKE_CXX_*
# variables it sets do NOT reach this directory. A target defined here without
# the baseline call would be built with the compiler default -- gnu++17 on GCC
# 11, meaning extensions back ON -- and the build would stay green. The driver's
# mutation that deletes the baseline call below is the test of that claim.
#
# Build products land wherever the caller points -B; the driver uses a pytest
# temporary directory so nothing is written into the repository. A developer
# building this by hand should target data/ per CLAUDE.md S0.2.

cmake_minimum_required(VERSION 3.16)
project(xbrain_link_no_ros LANGUAGES CXX)

# Set here rather than left to the caller so the driver's assertions on the real
# compiler command line cannot be silently skipped by a missing -D on the cmake
# invocation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Derived from this file's location, so a copied tree resolves to its own copy of
# the baseline and its own copy of the headers. The mutation tests rely on that:
# they copy three directories into a sandbox and mutate the copy.
get_filename_component(XBRAIN_V6_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." REALPATH)

add_subdirectory("${XBRAIN_V6_ROOT}/xbrain/common"
                 "${CMAKE_CURRENT_BINARY_DIR}/xbrain_common")

add_executable(link_no_ros main.cc)

# The only link dependency. xbrain::common is an INTERFACE target, so this adds
# an include directory and nothing else -- which is the shape the claim needs: if
# the resulting binary shows a ROS shared object in ldd, it can only have come
# from a header.
target_link_libraries(link_no_ros PRIVATE xbrain::common)

xbrain_common_apply_cxx_baseline(link_no_ros)

# Written at generate time, so these are the values CMake resolved for the target
# and not a re-statement of what was requested. INCLUDE_DIRECTORIES and
# LINK_LIBRARIES are included because "no ROS" has to be checked on the resolved
# lists, not on the source text; COMPILE_DEFINITIONS carries the generator
# expression result and is how the driver sees _GLIBCXX_ASSERTIONS appear in a
# Debug build and stay absent otherwise.
file(GENERATE
     OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/build_facts.txt"
     CONTENT
"cxx_standard=$<TARGET_PROPERTY:link_no_ros,CXX_STANDARD>
cxx_standard_required=$<TARGET_PROPERTY:link_no_ros,CXX_STANDARD_REQUIRED>
cxx_extensions=$<TARGET_PROPERTY:link_no_ros,CXX_EXTENSIONS>
compile_options=$<JOIN:$<TARGET_PROPERTY:link_no_ros,COMPILE_OPTIONS>,|>
compile_definitions=$<JOIN:$<TARGET_PROPERTY:link_no_ros,COMPILE_DEFINITIONS>,|>
include_directories=$<JOIN:$<TARGET_PROPERTY:link_no_ros,INCLUDE_DIRECTORIES>,|>
link_libraries=$<JOIN:$<TARGET_PROPERTY:link_no_ros,LINK_LIBRARIES>,|>
")
