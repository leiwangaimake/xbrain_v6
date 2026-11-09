# Host build for formal service configuration tests (no camera/SDK/model/network).
cmake_minimum_required(VERSION 3.16)
project(perception_service_host LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(RNS_UPSTREAM_COMMON "" CACHE PATH "Verified upstream contract_r6/common/include")
option(SERVICE_SANITIZERS "address/undefined sanitizers" ON)
if(NOT EXISTS "${RNS_UPSTREAM_COMMON}/xbrain/config/yaml_lite.h")
  message(FATAL_ERROR "RNS_UPSTREAM_COMMON must point at the verified upstream common include")
endif()
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp)  # yaml-cpp 0.7 (Ubuntu 22.04) exports only the plain name
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
find_package(Threads REQUIRED)
get_filename_component(PERCEPTION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_executable(test_service_config
  "${PERCEPTION_ROOT}/tests/service/test_service_config.cpp"
  "${PERCEPTION_ROOT}/src/service/service_config.cpp"
  "${PERCEPTION_ROOT}/src/common/resolved_yaml.cpp"
  "${PERCEPTION_ROOT}/src/common/config.cpp")
target_include_directories(test_service_config PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_compile_definitions(test_service_config PRIVATE FIXTURE_DIR="${PERCEPTION_ROOT}/tests/service/fixtures")
target_link_libraries(test_service_config PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(test_service_config PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_service_config PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_service_config PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME service_config COMMAND test_service_config)
add_executable(test_calibration_record
  "${PERCEPTION_ROOT}/tests/service/test_calibration_record.cpp"
  "${PERCEPTION_ROOT}/src/service/calibration_record.cpp"
  "${PERCEPTION_ROOT}/src/common/resolved_yaml.cpp"
  "${PERCEPTION_ROOT}/src/common/config.cpp")
target_include_directories(test_calibration_record PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_calibration_record PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(test_calibration_record PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_calibration_record PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_calibration_record PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME calibration_record COMMAND test_calibration_record)
add_executable(test_geometry_setup
  "${PERCEPTION_ROOT}/tests/service/test_geometry_setup.cpp"
  "${PERCEPTION_ROOT}/src/service/geometry_setup.cpp"
  "${PERCEPTION_ROOT}/src/service/calibration_record.cpp"
  "${PERCEPTION_ROOT}/src/service/service_config.cpp"
  "${PERCEPTION_ROOT}/src/common/rns_geometry_config.cpp"
  "${PERCEPTION_ROOT}/src/common/resolved_yaml.cpp"
  "${PERCEPTION_ROOT}/src/common/config.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp")
target_include_directories(test_geometry_setup PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_geometry_setup PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(test_geometry_setup PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_geometry_setup PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_geometry_setup PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME geometry_setup COMMAND test_geometry_setup)
add_executable(test_object_geometry_setup
  "${PERCEPTION_ROOT}/tests/service/test_object_geometry_setup.cpp"
  "${PERCEPTION_ROOT}/src/service/object_geometry_setup.cpp"
  "${PERCEPTION_ROOT}/src/common/rns_geometry_config.cpp"
  "${PERCEPTION_ROOT}/src/common/resolved_yaml.cpp"
  "${PERCEPTION_ROOT}/src/modules/rgb_depth_registration.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_object_geometry.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_reason_edges.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp")
target_include_directories(test_object_geometry_setup PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_object_geometry_setup PRIVATE Threads::Threads)
target_compile_options(test_object_geometry_setup PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_object_geometry_setup PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_object_geometry_setup PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME object_geometry_setup COMMAND test_object_geometry_setup)
add_executable(test_semantic_slot
  "${PERCEPTION_ROOT}/tests/service/test_semantic_slot.cpp"
  "${PERCEPTION_ROOT}/src/service/semantic_slot.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic_kernel.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
  "${PERCEPTION_ROOT}/src/modules/traversability_kernel.cpp")
target_include_directories(test_semantic_slot PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_semantic_slot PRIVATE Threads::Threads)
target_compile_options(test_semantic_slot PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_semantic_slot PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_semantic_slot PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME semantic_slot COMMAND test_semantic_slot)
# The Objects callback's delivery step: real semantic slot, real paced release and Objects channel.
add_executable(test_objects_delivery
  "${PERCEPTION_ROOT}/tests/service/test_objects_delivery.cpp"
  "${PERCEPTION_ROOT}/src/service/semantic_slot.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_objects_release.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publication_journal.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic_kernel.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
  "${PERCEPTION_ROOT}/src/modules/traversability_kernel.cpp")
target_include_directories(test_objects_delivery PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_objects_delivery PRIVATE Threads::Threads)
target_compile_options(test_objects_delivery PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_objects_delivery PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_objects_delivery PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME objects_delivery COMMAND test_objects_delivery)
add_executable(test_t_mask_slot
  "${PERCEPTION_ROOT}/tests/service/test_t_mask_slot.cpp"
  "${PERCEPTION_ROOT}/src/service/t_mask_slot.cpp")
target_include_directories(test_t_mask_slot PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_compile_definitions(test_t_mask_slot PRIVATE T_MASK_SLOT_TEST_HOOK)
target_link_libraries(test_t_mask_slot PRIVATE Threads::Threads)
target_compile_options(test_t_mask_slot PRIVATE -Wall -Wextra -O2 -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_t_mask_slot PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_t_mask_slot PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME t_mask_slot COMMAND test_t_mask_slot)
add_executable(test_t_projection_setup
  "${PERCEPTION_ROOT}/tests/service/test_t_projection_setup.cpp"
  "${PERCEPTION_ROOT}/src/service/t_projection_setup.cpp"
  "${PERCEPTION_ROOT}/src/modules/t_color_projection.cpp"
  "${PERCEPTION_ROOT}/src/modules/rgb_depth_registration.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_traversability.cpp"
  "${PERCEPTION_ROOT}/src/modules/traversability_kernel.cpp"
  "${PERCEPTION_ROOT}/src/common/rns_geometry_config.cpp"
  "${PERCEPTION_ROOT}/src/common/resolved_yaml.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_object_geometry.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp")
target_include_directories(test_t_projection_setup PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_t_projection_setup PRIVATE Threads::Threads)
target_compile_options(test_t_projection_setup PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_t_projection_setup PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_t_projection_setup PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME t_projection_setup COMMAND test_t_projection_setup)
add_executable(test_t_status
  "${PERCEPTION_ROOT}/tests/service/test_t_status.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_reason_edges.cpp")
target_include_directories(test_t_status PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_t_status PRIVATE Threads::Threads)
target_compile_options(test_t_status PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_t_status PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_t_status PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME t_status COMMAND test_t_status)
add_executable(test_psc4_golden
  "${PERCEPTION_ROOT}/tests/service/test_psc4_golden.cpp"
  "${PERCEPTION_ROOT}/src/service/psc4_golden.cpp"
  "${PERCEPTION_ROOT}/src/common/config.cpp")
target_include_directories(test_psc4_golden PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
target_link_libraries(test_psc4_golden PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(test_psc4_golden PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_psc4_golden PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_psc4_golden PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME psc4_golden COMMAND test_psc4_golden)
add_executable(test_bench_calibration
  "${PERCEPTION_ROOT}/tests/service/test_bench_calibration.cpp"
  "${PERCEPTION_ROOT}/src/service/bench_calibration.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp")
target_include_directories(test_bench_calibration PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_bench_calibration PRIVATE Threads::Threads)
target_compile_options(test_bench_calibration PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_bench_calibration PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_bench_calibration PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME bench_calibration COMMAND test_bench_calibration)
add_executable(test_profile_reason_state
  "${PERCEPTION_ROOT}/tests/service/test_profile_reason_state.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_reason_edges.cpp")
target_include_directories(test_profile_reason_state PRIVATE "${PERCEPTION_ROOT}/include")
target_compile_options(test_profile_reason_state PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_profile_reason_state PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_profile_reason_state PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME profile_reason_state COMMAND test_profile_reason_state)
add_executable(test_latency_histogram "${PERCEPTION_ROOT}/tests/service/test_latency_histogram.cpp")
target_include_directories(test_latency_histogram PRIVATE "${PERCEPTION_ROOT}/include")
target_compile_options(test_latency_histogram PRIVATE -Wall -Wextra -UNDEBUG)
add_test(NAME latency_histogram COMMAND test_latency_histogram)
# Diagnostic thread scheduler probe behind the SUP_FRAME deltas: the schedstat parser and, on
# Linux, the live counters against the thread CPU clock.
add_executable(test_thread_sched_probe "${PERCEPTION_ROOT}/tests/service/test_thread_sched_probe.cpp")
target_include_directories(test_thread_sched_probe PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_thread_sched_probe PRIVATE Threads::Threads)
target_compile_options(test_thread_sched_probe PRIVATE -Wall -Wextra -UNDEBUG)
add_test(NAME thread_sched_probe COMMAND test_thread_sched_probe)
add_executable(test_config_env_host
  "${PERCEPTION_ROOT}/tests/test_config_env.cpp"
  "${PERCEPTION_ROOT}/src/common/config.cpp")
target_include_directories(test_config_env_host PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_config_env_host PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(test_config_env_host PRIVATE -Wall -Wextra -UNDEBUG)
add_test(NAME config_env COMMAND test_config_env_host WORKING_DIRECTORY "${PERCEPTION_ROOT}")
add_executable(test_gen_notices
  "${PERCEPTION_ROOT}/tests/service/test_gen_notices.cpp"
  "${PERCEPTION_ROOT}/src/service/gen_notices.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp")
target_include_directories(test_gen_notices PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_gen_notices PRIVATE yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(test_gen_notices PRIVATE -Wall -Wextra -UNDEBUG)
if(SERVICE_SANITIZERS)
  target_compile_options(test_gen_notices PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_gen_notices PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME gen_notices COMMAND test_gen_notices)
# Development-only additions (our harness-driven logger checks); absent from the delivered package.
include(${CMAKE_CURRENT_SOURCE_DIR}/dev_extras.cmake OPTIONAL)
