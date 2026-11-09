cmake_minimum_required(VERSION 3.16)
project(perception_rns_host LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(RNS_SANITIZERS "Run host contract tests under address/undefined sanitizers" OFF)
set(RNS_UPSTREAM_COMMON "" CACHE PATH "Verified upstream common include path (helper differential and R6 setup test)")
set(RNS_ZENOH_ROOT "" CACHE PATH "Isolated zenoh-c 1.9.0 standalone SDK; empty disables native/network test")
set(RNS_ORBBEC_INCLUDE "" CACHE PATH "Optional real Orbbec SDK include directory; compile-only bridge check, no device access")
find_package(Threads REQUIRED)
get_filename_component(PERCEPTION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
foreach(name contract cadence envelope publisher system_clock capture_estimate)
  add_executable(test_rns_${name}
    "${PERCEPTION_ROOT}/src/output/rns_${name}.cpp"
    "${PERCEPTION_ROOT}/tests/test_rns_${name}.cpp")
  target_include_directories(test_rns_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_rns_${name} PRIVATE Threads::Threads)
  target_compile_options(test_rns_${name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_${name} COMMAND test_rns_${name})
  set_tests_properties(rns_${name} PROPERTIES TIMEOUT 30)
endforeach()
add_executable(test_native_depth
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/tests/test_native_depth.cpp")
target_include_directories(test_native_depth PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_native_depth PRIVATE Threads::Threads)
target_compile_options(test_native_depth PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_native_depth PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_native_depth PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME native_depth COMMAND test_native_depth)
set_tests_properties(native_depth PROPERTIES TIMEOUT 30)

# Pure normal-plane/evidence kernels. No prior fallback, SDK, full Profile,
# geometry publication or inferred production calibration is enabled.
foreach(name ground_fit native_geometry)
  add_executable(test_${name}
    "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
    "${PERCEPTION_ROOT}/tests/test_${name}.cpp")
  target_include_directories(test_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_${name} PRIVATE Threads::Threads)
  target_compile_options(test_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME ${name} COMMAND test_${name})
  set_tests_properties(${name} PROPERTIES TIMEOUT 60)
endforeach()
target_sources(test_native_geometry PRIVATE
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")

foreach(name grid_traversal normal_grid)
  add_executable(test_${name}
    "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
    "${PERCEPTION_ROOT}/tests/test_${name}.cpp")
  target_include_directories(test_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_${name} PRIVATE Threads::Threads)
  target_compile_options(test_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME ${name} COMMAND test_${name})
  set_tests_properties(${name} PROPERTIES TIMEOUT 60)
endforeach()
target_sources(test_normal_grid PRIVATE
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")

# NormalGrid against its frozen parent copy (tests/reference): the ticket-cell decision
# (in_cell) at every cell edge, and every output of process() over synthetic scenes. Built
# with -O2: 216 native frames through the geometry and two grids.
add_executable(test_normal_grid_ticket_cell
  "${PERCEPTION_ROOT}/tests/test_normal_grid_ticket_cell.cpp"
  "${PERCEPTION_ROOT}/tests/reference/normal_grid_parent.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")
target_include_directories(test_normal_grid_ticket_cell PRIVATE
  "${PERCEPTION_ROOT}/include" "${PERCEPTION_ROOT}/tests/reference")
target_link_libraries(test_normal_grid_ticket_cell PRIVATE Threads::Threads)
target_compile_options(test_normal_grid_ticket_cell PRIVATE -Wall -Wextra -Wpedantic -Werror -O2 -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_normal_grid_ticket_cell PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_normal_grid_ticket_cell PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME normal_grid_ticket_cell COMMAND test_normal_grid_ticket_cell)
set_tests_properties(normal_grid_ticket_cell PROPERTIES TIMEOUT 240)

# NativeGeometry against its frozen parent copy (tests/reference): every pixel record, bin, raw
# count and ground point after the fast-path scan stopped writing the record (2026-10-14), on the
# fast and the general path, plus a fresh geometry per frame (no stale record). Built with -O2.
add_executable(test_native_geometry_pixel_record
  "${PERCEPTION_ROOT}/tests/test_native_geometry_pixel_record.cpp"
  "${PERCEPTION_ROOT}/tests/reference/native_geometry_parent.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")
target_include_directories(test_native_geometry_pixel_record PRIVATE
  "${PERCEPTION_ROOT}/include" "${PERCEPTION_ROOT}/tests/reference")
target_link_libraries(test_native_geometry_pixel_record PRIVATE Threads::Threads)
target_compile_options(test_native_geometry_pixel_record PRIVATE -Wall -Wextra -Wpedantic -Werror -O2 -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_native_geometry_pixel_record PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_native_geometry_pixel_record PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME native_geometry_pixel_record COMMAND test_native_geometry_pixel_record)
set_tests_properties(native_geometry_pixel_record PROPERTIES TIMEOUT 240)

# Raw T -> current ground cells -> bounded metric-neighbourhood erosion.
# No SDK distortion/calibration hookup, production Profile or Q4 enabled.
foreach(name traversability_kernel normal_traversability)
  add_executable(test_${name}
    "${PERCEPTION_ROOT}/src/modules/traversability_kernel.cpp"
    "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
    "${PERCEPTION_ROOT}/tests/test_${name}.cpp")
  target_include_directories(test_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_${name} PRIVATE Threads::Threads)
  target_compile_options(test_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME ${name} COMMAND test_${name})
  set_tests_properties(${name} PROPERTIES TIMEOUT 90)
endforeach()
target_sources(test_normal_traversability PRIVATE
  "${PERCEPTION_ROOT}/src/modules/normal_traversability.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")

# Current native geometry -> normal Profile -> injected put. No SDK/network.
foreach(profile_test normal_profile semantic_profile withdrawn_evidence native_profile a19_src_golden)
add_executable(test_rns_${profile_test}
  "${PERCEPTION_ROOT}/tests/test_rns_${profile_test}.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_normal_profile.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic_kernel.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
  "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
  "${PERCEPTION_ROOT}/src/modules/traversability_kernel.cpp"
  "${PERCEPTION_ROOT}/src/modules/normal_traversability.cpp")
target_include_directories(test_rns_${profile_test} PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_rns_${profile_test} PRIVATE Threads::Threads)
target_compile_options(test_rns_${profile_test} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_${profile_test} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_${profile_test} PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_${profile_test} COMMAND test_rns_${profile_test})
set_tests_properties(rns_${profile_test} PROPERTIES TIMEOUT 120)
endforeach()

# Measured base_link object geometry (19 section 4.1). No SDK/OpenCV/camera.
add_executable(test_rgb_depth_registration
  "${PERCEPTION_ROOT}/tests/test_rgb_depth_registration.cpp"
  "${PERCEPTION_ROOT}/src/modules/rgb_depth_registration.cpp")
target_include_directories(test_rgb_depth_registration PRIVATE "${PERCEPTION_ROOT}/include")
target_compile_options(test_rgb_depth_registration PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
add_test(NAME rgb_depth_registration COMMAND test_rgb_depth_registration)
set_tests_properties(rgb_depth_registration PROPERTIES TIMEOUT 120)

add_executable(test_rns_object_geometry
  "${PERCEPTION_ROOT}/tests/test_rns_object_geometry.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_object_geometry.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp")
target_include_directories(test_rns_object_geometry PRIVATE "${PERCEPTION_ROOT}/include")
target_compile_options(test_rns_object_geometry PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_object_geometry PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_object_geometry PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_object_geometry COMMAND test_rns_object_geometry)
set_tests_properties(rns_object_geometry PROPERTIES TIMEOUT 120)

# ObjectGeometryBuilder against its frozen original (tests/reference): every result bit for bit
# over mixed call orders, and one allocation per Located build once the scratch has grown.
add_executable(test_rns_object_geometry_scratch
  "${PERCEPTION_ROOT}/tests/test_rns_object_geometry_scratch.cpp"
  "${PERCEPTION_ROOT}/tests/reference/rns_object_geometry_original.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_object_geometry.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp")
target_include_directories(test_rns_object_geometry_scratch PRIVATE
  "${PERCEPTION_ROOT}/include" "${PERCEPTION_ROOT}/tests/reference")
target_compile_options(test_rns_object_geometry_scratch PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_object_geometry_scratch PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_object_geometry_scratch PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_object_geometry_scratch COMMAND test_rns_object_geometry_scratch)
set_tests_properties(rns_object_geometry_scratch PROPERTIES TIMEOUT 120)

# Strict resolved-config adapter, using actual pinned common YAML code. Pure
# config -> synthetic geometry -> Profile test; no SDK/camera or runtime launch.
if(RNS_UPSTREAM_COMMON)
  add_executable(test_rns_geometry_config
    "${PERCEPTION_ROOT}/tests/test_rns_geometry_config.cpp"
    "${PERCEPTION_ROOT}/src/common/rns_geometry_config.cpp"
    "${PERCEPTION_ROOT}/src/common/resolved_yaml.cpp"
    "${PERCEPTION_ROOT}/src/modules/native_depth.cpp"
    "${PERCEPTION_ROOT}/src/modules/ground_fit.cpp"
    "${PERCEPTION_ROOT}/src/modules/native_geometry.cpp"
    "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp"
    "${PERCEPTION_ROOT}/src/modules/normal_grid.cpp"
    "${PERCEPTION_ROOT}/src/modules/traversability_kernel.cpp"
    "${PERCEPTION_ROOT}/src/modules/normal_traversability.cpp")
  foreach(component normal_profile semantic semantic_kernel contract publisher envelope cadence)
    target_sources(test_rns_geometry_config PRIVATE "${PERCEPTION_ROOT}/src/output/rns_${component}.cpp")
  endforeach()
  target_include_directories(test_rns_geometry_config PRIVATE
    "${PERCEPTION_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
  target_link_libraries(test_rns_geometry_config PRIVATE Threads::Threads)
  target_compile_options(test_rns_geometry_config PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_geometry_config PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_geometry_config PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_geometry_config COMMAND test_rns_geometry_config)
  set_tests_properties(rns_geometry_config PROPERTIES TIMEOUT 120)
endif()

target_sources(test_rns_native_profile PRIVATE
  "${PERCEPTION_ROOT}/src/service/t_mask_slot.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_native_profile.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_withdrawn_evidence.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_withdrawn_profile.cpp")

# A19-TIME-2: the service's own steady Profile loop (src/service/profile_steady_loop.cpp) on three
# real threads and CLOCK_MONOTONIC. It measures real time, so it is always built with -O2 and never
# with the sanitizers (a step there is about 8x slower at -O0 and the camera would outrun it).
add_executable(test_rns_profile_time2
  "${PERCEPTION_ROOT}/tests/test_rns_profile_time2.cpp"
  "${PERCEPTION_ROOT}/src/service/profile_steady_loop.cpp"
  "${PERCEPTION_ROOT}/src/service/t_mask_slot.cpp"
  "${PERCEPTION_ROOT}/src/service/semantic_slot.cpp")
foreach(component native_profile normal_profile withdrawn_profile withdrawn_evidence semantic semantic_kernel
                  contract publisher envelope cadence capture_estimate system_clock unconfigured_profile)
  target_sources(test_rns_profile_time2 PRIVATE "${PERCEPTION_ROOT}/src/output/rns_${component}.cpp")
endforeach()
foreach(module native_depth ground_fit native_geometry grid_traversal normal_grid traversability_kernel
               normal_traversability)
  target_sources(test_rns_profile_time2 PRIVATE "${PERCEPTION_ROOT}/src/modules/${module}.cpp")
endforeach()
target_include_directories(test_rns_profile_time2 PRIVATE "${PERCEPTION_ROOT}/include" "${PERCEPTION_ROOT}/tests")
target_link_libraries(test_rns_profile_time2 PRIVATE Threads::Threads)
target_compile_options(test_rns_profile_time2 PRIVATE -Wall -Wextra -Wpedantic -Werror -O2 -UNDEBUG)
add_test(NAME rns_profile_time2 COMMAND test_rns_profile_time2)
set_tests_properties(rns_profile_time2 PROPERTIES TIMEOUT 120 LABELS realtime)

# The Profile thread's timing lines through ProfileTimingHandoff: the four line kinds byte for byte
# against a frozen copy of the old formatter, the handoff rules (full, close, concurrent take),
# and both real Profile loops for 31 s (three 10 s marks) with the Profile threads' heap calls
# counted at malloc (glibc) or operator new (elsewhere). Real time and a replaced malloc: -O2,
# never with the sanitizers. Same sources as the TIME-2 test.
get_target_property(RNS_PROFILE_THREAD_SOURCES test_rns_profile_time2 SOURCES)
list(REMOVE_ITEM RNS_PROFILE_THREAD_SOURCES "${PERCEPTION_ROOT}/tests/test_rns_profile_time2.cpp")
add_executable(test_rns_profile_timing_log
  "${PERCEPTION_ROOT}/tests/test_rns_profile_timing_log.cpp" ${RNS_PROFILE_THREAD_SOURCES})
target_include_directories(test_rns_profile_timing_log PRIVATE "${PERCEPTION_ROOT}/include" "${PERCEPTION_ROOT}/tests")
target_link_libraries(test_rns_profile_timing_log PRIVATE Threads::Threads)
target_compile_options(test_rns_profile_timing_log PRIVATE -Wall -Wextra -Wpedantic -Werror -O2 -UNDEBUG)
add_test(NAME rns_profile_timing_log COMMAND test_rns_profile_timing_log)
set_tests_properties(rns_profile_timing_log PROPERTIES TIMEOUT 150 LABELS realtime)

# Explicit rawdepth -> UNKNOWN mode: no geometry/grid/model/SDK source linkage.
add_executable(test_rns_unconfigured_profile
  "${PERCEPTION_ROOT}/tests/test_rns_unconfigured_profile.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")
foreach(component unconfigured_profile depth_health source_journal publisher contract envelope cadence)
  target_sources(test_rns_unconfigured_profile PRIVATE "${PERCEPTION_ROOT}/src/output/rns_${component}.cpp")
endforeach()
target_include_directories(test_rns_unconfigured_profile PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_rns_unconfigured_profile PRIVATE Threads::Threads)
target_compile_options(test_rns_unconfigured_profile PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_unconfigured_profile PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_unconfigured_profile PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_unconfigured_profile COMMAND test_rns_unconfigured_profile)
set_tests_properties(rns_unconfigured_profile PROPERTIES TIMEOUT 45)

target_sources(test_rns_withdrawn_evidence PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_withdrawn_evidence.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_withdrawn_profile.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_status_heartbeat.cpp")

add_executable(test_rns_status_heartbeat
  "${PERCEPTION_ROOT}/tests/test_rns_status_heartbeat.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_status_heartbeat.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp")
target_include_directories(test_rns_status_heartbeat PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_rns_status_heartbeat PRIVATE Threads::Threads)
target_compile_options(test_rns_status_heartbeat PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_status_heartbeat PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_status_heartbeat PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_status_heartbeat COMMAND test_rns_status_heartbeat)
set_tests_properties(rns_status_heartbeat PROPERTIES TIMEOUT 60)

# A19-ENC-1: every accepted Profile/Objects/Status text and every final packet through the strict
# JSON validator and inf/nan token scan of tests/strict_json.hpp, plus the non-finite negative
# matrix. Pure host test: no SDK, network or device.
add_executable(test_rns_enc_strict
  "${PERCEPTION_ROOT}/tests/test_rns_enc_strict.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp")
target_include_directories(test_rns_enc_strict PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_rns_enc_strict PRIVATE Threads::Threads)
target_compile_options(test_rns_enc_strict PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_enc_strict PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_enc_strict PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_enc_strict COMMAND test_rns_enc_strict)
set_tests_properties(rns_enc_strict PROPERTIES TIMEOUT 60)

# Source-backed health statistics/ROI with explicit unknowns, no production hook.
foreach(name health_window depth_health health_fields)
  add_executable(test_rns_${name}
    "${PERCEPTION_ROOT}/tests/test_rns_${name}.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_${name}.cpp")
  target_include_directories(test_rns_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_rns_${name} PRIVATE Threads::Threads)
  target_compile_options(test_rns_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_${name} COMMAND test_rns_${name})
  set_tests_properties(rns_${name} PROPERTIES TIMEOUT 60)
endforeach()
target_sources(test_rns_depth_health PRIVATE "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")
target_sources(test_rns_health_fields PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_health_window.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_depth_health.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_status_heartbeat.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")

foreach(name reason_edges reason_flow)
  add_executable(test_rns_${name}
    "${PERCEPTION_ROOT}/tests/test_rns_${name}.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_reason_edges.cpp")
  target_include_directories(test_rns_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_rns_${name} PRIVATE Threads::Threads)
  target_compile_options(test_rns_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_${name} COMMAND test_rns_${name})
  set_tests_properties(rns_${name} PROPERTIES TIMEOUT 60)
endforeach()
target_sources(test_rns_reason_flow PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_health_fields.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_health_window.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_depth_health.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_status_heartbeat.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")

add_executable(test_rns_semantic_kernel
  "${PERCEPTION_ROOT}/tests/test_rns_semantic_kernel.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_semantic_kernel.cpp"
  "${PERCEPTION_ROOT}/src/modules/grid_traversal.cpp")
target_include_directories(test_rns_semantic_kernel PRIVATE "${PERCEPTION_ROOT}/include")
target_compile_options(test_rns_semantic_kernel PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_semantic_kernel PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_semantic_kernel PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_semantic_kernel COMMAND test_rns_semantic_kernel)
set_tests_properties(rns_semantic_kernel PROPERTIES TIMEOUT 60)

add_executable(test_rns_gen_events
  "${PERCEPTION_ROOT}/tests/test_rns_gen_events.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_gen_events.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_reason_edges.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp")
target_include_directories(test_rns_gen_events PRIVATE "${PERCEPTION_ROOT}/include")
target_link_libraries(test_rns_gen_events PRIVATE Threads::Threads)
target_compile_options(test_rns_gen_events PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RNS_SANITIZERS)
  target_compile_options(test_rns_gen_events PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_rns_gen_events PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME rns_gen_events COMMAND test_rns_gen_events)
set_tests_properties(rns_gen_events PROPERTIES TIMEOUT 60)

# Uses actual SDK declarations, but deliberately does not link its platform
# library or create a Pipeline. This is NOT a camera/runtime acceptance test.
if(RNS_ORBBEC_INCLUDE)
  if(NOT EXISTS "${RNS_ORBBEC_INCLUDE}/libobsensor/ObSensor.hpp")
    message(FATAL_ERROR "RNS_ORBBEC_INCLUDE must point to real Orbbec SDK headers")
  endif()
  add_library(orbbec_native_depth_header_check OBJECT
    "${PERCEPTION_ROOT}/src/modules/orbbec_native_depth.cpp")
  target_include_directories(orbbec_native_depth_header_check PRIVATE "${PERCEPTION_ROOT}/include")
  target_include_directories(orbbec_native_depth_header_check SYSTEM PRIVATE "${RNS_ORBBEC_INCLUDE}")
  target_compile_options(orbbec_native_depth_header_check PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
target_sources(test_rns_publisher PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp")
if(RNS_UPSTREAM_COMMON)
  target_include_directories(test_rns_envelope PRIVATE "${RNS_UPSTREAM_COMMON}")
  target_compile_definitions(test_rns_envelope PRIVATE RNS_UPSTREAM_COMMON=1)
endif()

# Deliberately opt-in: normal host tests do not open sockets or load Zenoh.
# Never search the legacy vendored 1.2.1 SDK or a system library as fallback.
if(RNS_ZENOH_ROOT)
  if(NOT EXISTS "${RNS_ZENOH_ROOT}/include/zenoh_configure.h" OR
     NOT EXISTS "${RNS_ZENOH_ROOT}/lib/libzenohc.a")
    message(FATAL_ERROR "RNS_ZENOH_ROOT must contain the standalone SDK header and static library")
  endif()
  file(READ "${RNS_ZENOH_ROOT}/include/zenoh_configure.h" ZENOH_CONFIG)
  if(NOT ZENOH_CONFIG MATCHES "#define ZENOH_C \"1\\.9\\.0\"")
    message(FATAL_ERROR "R4 requires Zenoh 1.9.0; old/system versions are not substituted")
  endif()
  add_executable(test_rns_zenoh_loopback
    "${PERCEPTION_ROOT}/tests/test_rns_zenoh_loopback.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_zenoh_transport.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_system_clock.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp")
  target_include_directories(test_rns_zenoh_loopback PRIVATE
    "${PERCEPTION_ROOT}/include" "${RNS_ZENOH_ROOT}/include")
  target_compile_options(test_rns_zenoh_loopback PRIVATE -Wall -Wextra -Werror -UNDEBUG)
  target_link_libraries(test_rns_zenoh_loopback PRIVATE
    "${RNS_ZENOH_ROOT}/lib/libzenohc.a" Threads::Threads)
  if(APPLE)
    target_link_libraries(test_rns_zenoh_loopback PRIVATE
      "-framework Security" "-framework CoreFoundation" "-framework SystemConfiguration" resolv)
  elseif(UNIX)
    target_link_libraries(test_rns_zenoh_loopback PRIVATE dl m rt)
  endif()
  if(RNS_SANITIZERS)
    # Instruments our C++ only; this does not recompile the vendor Rust library.
    target_compile_options(test_rns_zenoh_loopback PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_zenoh_loopback PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_zenoh_loopback COMMAND test_rns_zenoh_loopback)
  set_tests_properties(rns_zenoh_loopback PROPERTIES TIMEOUT 30)
  add_executable(test_rns_gen_loopback
    "${PERCEPTION_ROOT}/tests/test_rns_gen_loopback.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_gen_transport.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_gen_events.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_reason_edges.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_zenoh_transport.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_system_clock.cpp")
  target_include_directories(test_rns_gen_loopback PRIVATE "${PERCEPTION_ROOT}/include" "${RNS_ZENOH_ROOT}/include")
  target_compile_options(test_rns_gen_loopback PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  target_link_libraries(test_rns_gen_loopback PRIVATE "${RNS_ZENOH_ROOT}/lib/libzenohc.a" Threads::Threads)
  if(APPLE)
    target_link_libraries(test_rns_gen_loopback PRIVATE "-framework Security" "-framework CoreFoundation" "-framework SystemConfiguration" resolv)
  elseif(UNIX)
    target_link_libraries(test_rns_gen_loopback PRIVATE dl m rt)
  endif()
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_gen_loopback PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_gen_loopback PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_gen_loopback COMMAND test_rns_gen_loopback)
  set_tests_properties(rns_gen_loopback PROPERTIES TIMEOUT 30)

  # Actual generated r6 bodies -> isolated TCP. Reuse production component
  # source lists, never the other test mains. Single controlled owner only;
  # synthetic frames/objects are not camera, model or cadence acceptance.
  get_target_property(RNS_GENERATED_PROFILE_SOURCES test_rns_native_profile SOURCES)
  get_target_property(RNS_GENERATED_HEALTH_SOURCES test_rns_health_fields SOURCES)
  set(RNS_GENERATED_SOURCES ${RNS_GENERATED_PROFILE_SOURCES} ${RNS_GENERATED_HEALTH_SOURCES})
  list(FILTER RNS_GENERATED_SOURCES EXCLUDE REGEX "/tests/")
  list(REMOVE_DUPLICATES RNS_GENERATED_SOURCES)
  add_executable(test_rns_zenoh_generated
    "${PERCEPTION_ROOT}/tests/test_rns_zenoh_generated.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_zenoh_transport.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_system_clock.cpp"
    ${RNS_GENERATED_SOURCES})
  target_include_directories(test_rns_zenoh_generated PRIVATE
    "${PERCEPTION_ROOT}/include" "${RNS_ZENOH_ROOT}/include")
  target_compile_options(test_rns_zenoh_generated PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  target_link_libraries(test_rns_zenoh_generated PRIVATE "${RNS_ZENOH_ROOT}/lib/libzenohc.a" Threads::Threads)
  if(APPLE)
    target_link_libraries(test_rns_zenoh_generated PRIVATE
      "-framework Security" "-framework CoreFoundation" "-framework SystemConfiguration" resolv)
  elseif(UNIX)
    target_link_libraries(test_rns_zenoh_generated PRIVATE dl m rt)
  endif()
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_zenoh_generated PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_zenoh_generated PRIVATE -fsanitize=address,undefined)
  endif()
  # Keep this opt-in network test inside the existing loopback exclusion
  # pattern used by test_rns_zenoh.py's separately labelled pure CTest run.
  add_test(NAME rns_zenoh_loopback_generated COMMAND test_rns_zenoh_generated)
  set_tests_properties(rns_zenoh_loopback_generated PROPERTIES TIMEOUT 30)

  # Three independent owners sharing only the process envelope/session.
  # The Objects pause is before publish, NOT inside the SDK. Synthetic DTOs
  # do not certify camera, live health-window handoff or publishing cadence.
  add_executable(test_rns_zenoh_owners
    "${PERCEPTION_ROOT}/tests/test_rns_zenoh_owners.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_zenoh_transport.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_system_clock.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
    "${PERCEPTION_ROOT}/src/output/rns_status_heartbeat.cpp")
  target_include_directories(test_rns_zenoh_owners PRIVATE
    "${PERCEPTION_ROOT}/include" "${RNS_ZENOH_ROOT}/include")
  target_compile_options(test_rns_zenoh_owners PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  target_link_libraries(test_rns_zenoh_owners PRIVATE "${RNS_ZENOH_ROOT}/lib/libzenohc.a" Threads::Threads)
  if(APPLE)
    target_link_libraries(test_rns_zenoh_owners PRIVATE
      "-framework Security" "-framework CoreFoundation" "-framework SystemConfiguration" resolv)
  elseif(UNIX)
    target_link_libraries(test_rns_zenoh_owners PRIVATE dl m rt)
  endif()
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_zenoh_owners PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_zenoh_owners PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_zenoh_loopback_owners COMMAND test_rns_zenoh_owners)
  set_tests_properties(rns_zenoh_loopback_owners PROPERTIES TIMEOUT 30)

  # A19-ALLOC-1: heap calls per put on the publishing thread, ours and zenoh-c's in separate
  # columns. It replaces glibc's malloc in the executable and wraps the put path's zenoh-c entry
  # points at link time, so Linux/glibc only and never under sanitizers (they replace malloc
  # themselves). Only the transport is linked: the measured path is ZenohRtTransport alone.
  # Named inside the loopback exclusion pattern: it opens a local TCP pair like the others.
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT RNS_SANITIZERS)
    add_executable(test_rns_zenoh_alloc
      "${PERCEPTION_ROOT}/tests/test_rns_zenoh_alloc.cpp"
      "${PERCEPTION_ROOT}/src/output/rns_zenoh_transport.cpp")
    target_include_directories(test_rns_zenoh_alloc PRIVATE
      "${PERCEPTION_ROOT}/include" "${RNS_ZENOH_ROOT}/include")
    target_compile_options(test_rns_zenoh_alloc PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    foreach(entry z_bytes_from_buf z_publisher_put z_bytes_drop z_internal_bytes_null
                  z_publisher_put_options_default z_publisher_loan)
      target_link_options(test_rns_zenoh_alloc PRIVATE "LINKER:--wrap=${entry}")
    endforeach()
    target_link_libraries(test_rns_zenoh_alloc PRIVATE
      "${RNS_ZENOH_ROOT}/lib/libzenohc.a" Threads::Threads dl m rt)
    add_test(NAME rns_zenoh_loopback_alloc COMMAND test_rns_zenoh_alloc)
    set_tests_properties(rns_zenoh_loopback_alloc PROPERTIES TIMEOUT 120)
  endif()
endif()

# Publication attempt/completion SPSC handoff. Pure/injected-clock tests only;
# these do not certify the SDK's internal blocking or a live publish cadence.
# packet_slot_pool: the RT transport's packet pool alone (A19-ALLOC-1), no SDK.
foreach(name health_pending publication_journal publication_handoff source_journal source_hooks source_handoff health_source_identity declaration declaration_handoff objects_release packet_slot_pool)
  add_executable(test_rns_${name} "${PERCEPTION_ROOT}/tests/test_rns_${name}.cpp")
  target_include_directories(test_rns_${name} PRIVATE "${PERCEPTION_ROOT}/include")
  target_link_libraries(test_rns_${name} PRIVATE Threads::Threads)
  target_compile_options(test_rns_${name} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
  if(RNS_SANITIZERS)
    target_compile_options(test_rns_${name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(test_rns_${name} PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME rns_${name} COMMAND test_rns_${name})
  set_tests_properties(rns_${name} PROPERTIES TIMEOUT 60)
endforeach()
target_sources(test_rns_health_pending PRIVATE "${PERCEPTION_ROOT}/src/output/rns_health_window.cpp")
target_sources(test_rns_health_source_identity PRIVATE "${PERCEPTION_ROOT}/src/output/rns_health_window.cpp")
target_sources(test_rns_declaration PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp")
target_sources(test_rns_publication_journal PRIVATE "${PERCEPTION_ROOT}/src/output/rns_publication_journal.cpp")
target_sources(test_rns_source_journal PRIVATE "${PERCEPTION_ROOT}/src/output/rns_source_journal.cpp")
target_sources(test_rns_source_hooks PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_source_journal.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_depth_health.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")
target_sources(test_rns_publication_handoff PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_publication_handoff.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_health_window.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_health_fields.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_depth_health.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_status_heartbeat.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_contract.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_cadence.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_envelope.cpp"
  "${PERCEPTION_ROOT}/src/output/rns_publisher.cpp"
  "${PERCEPTION_ROOT}/src/modules/native_depth.cpp")
get_target_property(RNS_HANDOFF_SOURCES test_rns_publication_handoff SOURCES)
list(REMOVE_ITEM RNS_HANDOFF_SOURCES "${PERCEPTION_ROOT}/tests/test_rns_publication_handoff.cpp")
target_sources(test_rns_source_handoff PRIVATE ${RNS_HANDOFF_SOURCES})
target_sources(test_rns_declaration_handoff PRIVATE ${RNS_HANDOFF_SOURCES})
# Paced Objects release: its real thread and slot in front of a real channel, journal, cadence
# ledger and Status handoff, with injected clocks and transports.
target_sources(test_rns_objects_release PRIVATE
  "${PERCEPTION_ROOT}/src/output/rns_objects_release.cpp" ${RNS_HANDOFF_SOURCES})
set_tests_properties(rns_objects_release PROPERTIES TIMEOUT 300)

# The optional journal is a PublisherChannel dependency even when a caller
# leaves it disabled. Include it in all existing source-based publisher tests.
get_property(RNS_ALL_HOST_TARGETS DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(rns_target IN LISTS RNS_ALL_HOST_TARGETS)
  get_target_property(rns_sources ${rns_target} SOURCES)
  if("${rns_sources}" MATCHES "/src/output/rns_publisher.cpp")
    target_sources(${rns_target} PRIVATE "${PERCEPTION_ROOT}/src/output/rns_publication_journal.cpp")
  endif()
  if("${rns_sources}" MATCHES "/src/output/rns_native_profile.cpp")
    target_sources(${rns_target} PRIVATE
      "${PERCEPTION_ROOT}/src/output/rns_source_journal.cpp"
      "${PERCEPTION_ROOT}/src/output/rns_depth_health.cpp")
  endif()
  if("${rns_sources}" MATCHES "/src/output/rns_publication_handoff.cpp")
    target_sources(${rns_target} PRIVATE "${PERCEPTION_ROOT}/src/output/rns_source_journal.cpp")
  endif()
endforeach()
