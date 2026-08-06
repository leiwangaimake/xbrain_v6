# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: CMakeLists.txt
# Brief: C++17 build baseline for the shared foundation library, and the target
#        that exposes its deployed headers
#
# Description:
# What problem this solves. The foundation library under common/ is consumed by
# chassis_relay, quadruped, perception and rtk_driver. 11 S1.1.6 puts
# chassis_relay on the emergency-stop path and constrains it with CRL-4 (C++, no
# dynamic allocation, no blocking logging) and CRL-5 (single hop under 200 us).
# 19 S1.2 states the consequence in one line: if anything in common/ pulls in
# rclcpp, that process simply cannot use it. Nothing enforced that until now --
# there was no CMakeLists.txt anywhere in the tree, so "common/ links without
# ROS" was a sentence in a document with no build behind it. This file is the
# build half, and tests/common/link_no_ros/ is the evaluation half.
#
# Which sections this implements:
#   * 13 S9.3 CPP-1 -- the language standard is EXACTLY C++17: CXX_STANDARD 17
#     plus CXX_STANDARD_REQUIRED ON plus CXX_EXTENSIONS OFF. The last one is not
#     decoration; without it GCC gives gnu++17 and GNU extensions leak in.
#   * 13 S2.0 PB-5 -- no distribution detection macro of any kind. The platform
#     baseline D-45 (humble/22.04 against Jazzy/24.04) is NOT decided, and a
#     macro written today has to be torn out when it is. There is deliberately
#     no if(ROS_DISTRO ...) and no CMAKE_SYSTEM_VERSION test below.
#   * 19 S1.1 -- warning flags -Wall -Wextra -Werror -Wpedantic, and a debug
#     build that supports -D_GLIBCXX_ASSERTIONS.
#
# Where the files live, because the split is easy to get wrong. CLAUDE.md S0.2
# puts DEPLOYMENT artefacts (binary libraries and headers) under common/ and
# forbids source there; the shared layer's own sources live under xbrain/common/.
# A build script is source, so it lives here, and it reaches sideways to
# include for the headers it publishes. Do not move this file into
# common/ to be next to the headers -- that directory takes no source.
#
# What this file does NOT do, so logic does not get pushed in here:
#   * it declares no install() rule. The deployment location IS
#     include and the headers are already there, so an install rule
#     would copy a directory onto itself and would have no consumer today
#     (CLAUDE.md S9.3 forbids hooks kept for later).
#   * it does not scan the headers for a rclcpp include. A grep here would report
#     its own verdict; the gate that matters is that a translation unit with no
#     ROS include directory and no ROS library on its link line still builds,
#     which is what tests/common/link_no_ros/ actually does.
#   * it says nothing about CRL-4's no-allocation rule. The header published here
#     allocates through std::string by design and documents where it may be
#     called from; whether chassis_relay allocates on its hop is a property of
#     chassis_relay's own sources, not of this build baseline.
#
# The two ways to use this that look right and are not:
#
#   1. set(CMAKE_CXX_STANDARD 17) here and expecting it to reach the parent.
#      Those three variables are directory scoped. A parent project that does
#      add_subdirectory() on this directory and then defines its own targets gets
#      the COMPILER default, which on GCC 11 is gnu++17 -- extensions silently
#      back ON, and nothing says so. That is why the baseline is also published
#      as a function, and why every consumer must call it per target.
#
#   2. target_compile_features(... INTERFACE cxx_std_17). A compile feature means
#      "at least C++17", so a consumer building at C++20 satisfies it. CPP-1 says
#      EXACTLY C++17. The feature is therefore deliberately absent: it would make
#      the C++20 mutation in tests/common/test_link_no_ros.py compile.

cmake_minimum_required(VERSION 3.16)

# Only claim a project when this directory is the top of the build. Consumers
# reach this file through add_subdirectory() and have already called project();
# a second call there would open a nested project scope for no reason.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(xbrain_common LANGUAGES CXX)
endif()

# 13 S9.3 CPP-1, verbatim form. These cover any target defined in this directory
# or below it. See trap 1 in the header: they do NOT cover the parent, which is
# what xbrain_common_apply_cxx_baseline() exists for.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# The baseline, as a function so it survives a directory boundary.
#
# The flag list is written inside the function body on purpose. A CMake function
# runs in a scope whose parent is the CALL SITE, not the definition site, so a
# variable set next to this definition is invisible when the function is called
# from another directory -- the flags would expand to nothing and every consumer
# would build with warnings off while still appearing to have called the
# baseline. Keeping the literal list inside the body removes that failure.
# ---------------------------------------------------------------------------
function(xbrain_common_apply_cxx_baseline target)
  set_target_properties(${target} PROPERTIES
    CXX_STANDARD 17
    # ON, and this one is load bearing in a way its name does not suggest.
    # Measured on cmake 3.22 with GCC 11: with it OFF, CMake decides the
    # compiler's DEFAULT already provides C++17 and therefore emits no -std flag
    # at all. GCC's default is gnu++17, so the target silently builds with GNU
    # extensions enabled -- CXX_EXTENSIONS OFF two lines below is defeated
    # without a word. The documented effect (fail instead of decaying to an older
    # standard) is the smaller half; on this toolchain OFF is simply how you lose
    # -std=c++17 from the command line.
    CXX_STANDARD_REQUIRED ON
    # OFF gives -std=c++17; ON gives -std=gnu++17. Under gnu++17 constructs like
    # typeof compile, and 13 S2.0 PB-5 warns that GNU extensions are the quiet
    # route by which C++20 spellings arrive.
    CXX_EXTENSIONS OFF)

  # 19 S1.1 compile switches. -Werror is the one that makes the other three mean
  # something: a warning nobody reads is not a gate.
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Werror -Wpedantic)

  # 19 S1.1 asks the debug build to support -D_GLIBCXX_ASSERTIONS, and 19 S11
  # PMT-40 relies on it: the mutation that lets a ROS 2 publish hold a loaned
  # sample is only caught because the debug build traps it. Bound to the Debug
  # configuration through a generator expression so a release build is not
  # silently paying for the checks, and so no separate opt-in switch exists to be
  # forgotten.
  target_compile_definitions(${target} PRIVATE
    $<$<CONFIG:Debug>:_GLIBCXX_ASSERTIONS>)
endfunction()

# ---------------------------------------------------------------------------
# The deployed headers.
# ---------------------------------------------------------------------------

# Derived from this file's own location rather than written as an absolute path,
# so a copy of the tree (which is how the mutation tests build a mutated
# common/) resolves to its own headers and not back to the original.
get_filename_component(XBRAIN_COMMON_DEPLOY_INCLUDE
                       "${CMAKE_CURRENT_LIST_DIR}/include" REALPATH)

# Fail at configure time rather than emitting a target whose include directory
# does not exist. The latter produces a "no such file" on the first consumer
# instead of here, and the message points at the consumer.
if(NOT IS_DIRECTORY "${XBRAIN_COMMON_DEPLOY_INCLUDE}")
  message(FATAL_ERROR
    "deployed header directory is missing: ${XBRAIN_COMMON_DEPLOY_INCLUDE}")
endif()

# INTERFACE, because everything published today is header only. That is not a
# weaker test than a compiled library would be: the consumer still has to
# preprocess and compile every header it includes, which is where a rclcpp
# include would surface.
add_library(xbrain_common INTERFACE)
add_library(xbrain::common ALIAS xbrain_common)

# ---------------------------------------------------------------------------
# The aggregate header, so the no-ROS gate covers every deployed header rather
# than the ones somebody remembered.
#
# Why this is generated and not a hand-written list. The gate in
# tests/common/link_no_ros/ only reaches headers its translation unit actually
# includes. With a hand-kept list, a header added under common/include and not
# added to the list sits OUTSIDE the scan surface while the suite still reports
# green -- CLAUDE.md S3.2 form 6, a number quoted without saying what it was
# measured over. That is not hypothetical: a second header appeared under
# common/include while this item was being written. Globbing makes the surface
# complete by construction, and CONFIGURE_DEPENDS re-runs the glob at build time
# so a header added after the last configure is still picked up.
#
# Being precise about what "by construction" is worth: it removes the human step,
# it does not make the enumeration self-verifying. A glob pattern that matched
# only .hpp would silently drop every .h and still look complete. So
# tests/common/test_link_no_ros.py walks the directory in Python, independently
# of CMake, and compares the two enumerations.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE XBRAIN_COMMON_DEPLOY_HEADERS CONFIGURE_DEPENDS
     RELATIVE "${XBRAIN_COMMON_DEPLOY_INCLUDE}"
     "${XBRAIN_COMMON_DEPLOY_INCLUDE}/*.h"
     "${XBRAIN_COMMON_DEPLOY_INCLUDE}/*.hpp")

# An empty glob is the classic silent failure: the aggregate would compile, the
# gate would pass, and it would be covering nothing at all.
if(XBRAIN_COMMON_DEPLOY_HEADERS STREQUAL "")
  message(FATAL_ERROR
    "no deployed headers found under ${XBRAIN_COMMON_DEPLOY_INCLUDE}")
endif()

# Sorted so the generated file is byte-stable across machines; an unsorted glob
# would rewrite the file on some filesystems and force needless rebuilds.
list(SORT XBRAIN_COMMON_DEPLOY_HEADERS)

set(XBRAIN_COMMON_AGGREGATE_BODY "")
foreach(header IN LISTS XBRAIN_COMMON_DEPLOY_HEADERS)
  string(APPEND XBRAIN_COMMON_AGGREGATE_BODY "#include \"${header}\"\n")
endforeach()

set(XBRAIN_COMMON_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(WRITE
  "${XBRAIN_COMMON_GENERATED_DIR}/xbrain/common/all_deployed_headers.h"
"/* Generated by common/CMakeLists.txt. Do not edit, do not commit.
 * One include per header under common/include, so the no-ROS gate in
 * tests/common/link_no_ros/ covers all of them. */
#ifndef HACHIST_XBRAIN_V6_COMMON_ALL_DEPLOYED_HEADERS_H_
#define HACHIST_XBRAIN_V6_COMMON_ALL_DEPLOYED_HEADERS_H_

${XBRAIN_COMMON_AGGREGATE_BODY}
#endif  // HACHIST_XBRAIN_V6_COMMON_ALL_DEPLOYED_HEADERS_H_
")

# Plain include, not SYSTEM. Marking it SYSTEM would suppress warnings coming
# from these headers, and -Werror would then stop applying to exactly the code
# this baseline exists to keep clean.
target_include_directories(xbrain_common INTERFACE
                           "${XBRAIN_COMMON_DEPLOY_INCLUDE}"
                           "${XBRAIN_COMMON_GENERATED_DIR}")
