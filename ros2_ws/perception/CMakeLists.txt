# Copyright (c) 2026 Hachist Robotics
# Author: diaofanghaiyang@hachist.com
# 上海哈船智能船舶技术有限公司
# File: CMakeLists.txt
# Brief: ament_cmake build of the formal R6 perception service and its installation tools.
#
# Description:
# The defaults build the delivered product: perception_node (formal R6 service, 19 section 1.1,
# installed as lib/perception/perception_node for the contract systemd unit), the installation
# tools perception_engine_build and perception_psc4_seal (19 section 13.2 target-machine engines
# and PSC-4 seal, built on request) and perception_bench_calib (19 section 7.1 plane step),
# plus the unit tests of the delivered code.
# Orbbec capture, TensorRT and the frame-sync SEI are ON because the R6 service refuses any other
# combination. The platform common/ beside the package is used as
# is (XBRAIN_COMMON_ROOT). Development-only targets of our own repo live in cmake/dev_extras.cmake,
# included OPTIONAL and absent from the delivered package (harness/export_delivery.py).

cmake_minimum_required(VERSION 3.16)
project(perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(CTest)

# ---------------- ROS2 / ament ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------- core deps ----------------
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(realsense2 QUIET CONFIG PATHS /opt/librealsense)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(GSTREAMER gstreamer-1.0)
pkg_check_modules(GST_APP gstreamer-app-1.0)
pkg_check_modules(GST_RTSP gstreamer-rtsp-1.0)
pkg_check_modules(GST_RTSP_SERVER gstreamer-rtsp-server-1.0)

set(HAS_REALSENSE OFF)
if(realsense2_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/modules/d435_capture.cpp")
    set(HAS_REALSENSE ON)
    add_compile_definitions(HAS_REALSENSE)
    message(STATUS "Intel RealSense support enabled: ${realsense2_VERSION}")
else()
    message(STATUS "Intel RealSense not found; d435_capture and its smoke test will not be built")
endif()

option(
    ENABLE_ORBBEC_CAPTURE
    "Enable Orbbec Gemini 338Le capture support"
    ON)
# Platform common/ root (e.g. /opt/xbrain_v6/common on the target). When set, its
# vendor Orbbec SDK (include/libobsensor + lib/libobsensor with extensions/) and
# its xbrain/ headers are used AS IS -- common/ is shared and never modified.
set(XBRAIN_COMMON_ROOT "" CACHE PATH "Platform common/ root; use its Orbbec SDK and xbrain headers unchanged")
# Deployed as /opt/xbrain_v6/ros2_ws/perception the platform common/ sits two
# levels up -- the same relative path the other ros2_ws packages use
# (chassis_relay / quadruped / sensor: ${CMAKE_CURRENT_SOURCE_DIR}/../../common).
if(NOT XBRAIN_COMMON_ROOT AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../common/include/xbrain")
    get_filename_component(XBRAIN_COMMON_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../common" ABSOLUTE)
    message(STATUS "Platform common/ beside the package: ${XBRAIN_COMMON_ROOT}")
endif()
set(HAS_ORBBEC OFF)
if(ENABLE_ORBBEC_CAPTURE)
    if(XBRAIN_COMMON_ROOT)
        # The SDK's own CMake config derives its prefix from the vendor package
        # layout (<sdk>/lib/*.cmake -> <sdk>/{lib,include}); common/ keeps the
        # library one level deeper, so that config resolves to paths that do not
        # exist. Declare the same imported target directly instead.
        set(XBRAIN_ORBBEC_LIB_DIR "${XBRAIN_COMMON_ROOT}/lib/libobsensor")
        if(NOT EXISTS "${XBRAIN_ORBBEC_LIB_DIR}/libOrbbecSDK.so.2.9.3" OR
           NOT EXISTS "${XBRAIN_COMMON_ROOT}/include/libobsensor/ObSensor.hpp" OR
           NOT IS_DIRECTORY "${XBRAIN_ORBBEC_LIB_DIR}/extensions")
            message(FATAL_ERROR "XBRAIN_COMMON_ROOT has no Orbbec SDK 2.9.3 (lib/libobsensor with extensions/, include/libobsensor)")
        endif()
        add_library(ob::OrbbecSDK SHARED IMPORTED)
        set_target_properties(ob::OrbbecSDK PROPERTIES
            IMPORTED_LOCATION "${XBRAIN_ORBBEC_LIB_DIR}/libOrbbecSDK.so.2.9.3"
            IMPORTED_SONAME "libOrbbecSDK.so.2"
            INTERFACE_INCLUDE_DIRECTORIES "${XBRAIN_COMMON_ROOT}/include")
        set(OrbbecSDK_DIR "${XBRAIN_ORBBEC_LIB_DIR}")
        message(STATUS "Orbbec SDK from platform common/: ${XBRAIN_ORBBEC_LIB_DIR}")
    else()
        find_package(OrbbecSDK CONFIG REQUIRED)
    endif()
    if(NOT TARGET ob::OrbbecSDK)
        message(FATAL_ERROR "OrbbecSDK found without required target ob::OrbbecSDK")
    endif()
    set(HAS_ORBBEC ON)
    add_compile_definitions(HAS_ORBBEC)
    message(STATUS "Orbbec Gemini 338Le capture support enabled")
else()
    message(STATUS "Orbbec capture support disabled by ENABLE_ORBBEC_CAPTURE=OFF")
endif()

if(GSTREAMER_FOUND AND GST_APP_FOUND AND GST_RTSP_FOUND AND GST_RTSP_SERVER_FOUND)
    add_compile_definitions(HAS_GST_RTSP_SERVER)
    include_directories(${GSTREAMER_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS} ${GST_RTSP_SERVER_INCLUDE_DIRS})
    link_directories(${GSTREAMER_LIBRARY_DIRS} ${GST_APP_LIBRARY_DIRS} ${GST_RTSP_SERVER_LIBRARY_DIRS})
    message(STATUS "GStreamer RTSP Server found: ${GST_RTSP_SERVER_VERSION}")
else()
    message(STATUS "GStreamer RTSP Server not found; building RTSP proxy stub")
endif()

option(ENABLE_TENSORRT "Enable the TensorRT inference backend when dependencies are available" ON)
option(
    ENABLE_FRAME_SYNC_SEI
    "Embed exact source frame_id in HEVC user_data_unregistered SEI"
    ON)
if(ENABLE_FRAME_SYNC_SEI AND
   NOT (GSTREAMER_FOUND AND GST_APP_FOUND AND GST_RTSP_FOUND AND GST_RTSP_SERVER_FOUND))
    message(FATAL_ERROR "ENABLE_FRAME_SYNC_SEI requires the GStreamer RTSP stack")
endif()
set(HAS_TENSORRT OFF)
if(ENABLE_TENSORRT)
    find_package(CUDAToolkit QUIET)
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS /usr/include/aarch64-linux-gnu /usr/include /usr/local/include)
    find_library(TENSORRT_LIBRARY nvinfer
        HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
    find_library(TENSORRT_ONNX_PARSER_LIBRARY nvonnxparser
        HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)

    if(CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND TENSORRT_ONNX_PARSER_LIBRARY)
        set(HAS_TENSORRT ON)
        message(STATUS "TensorRT backend enabled: ${TENSORRT_LIBRARY}")
    else()
        message(STATUS "TensorRT/CUDA development files not found; building inference stub")
    endif()
else()
    message(STATUS "TensorRT backend disabled by ENABLE_TENSORRT=OFF; building inference stub")
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVCODEC_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWSCALE_INCLUDE_DIRS}
)

set(COMMON_LIBS
    ${OpenCV_LIBS}
    yaml-cpp
    Threads::Threads
    ${AVFORMAT_LIBRARIES}
    ${AVCODEC_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWSCALE_LIBRARIES}
    OpenMP::OpenMP_CXX
)
if(HAS_TENSORRT)
    include_directories(${TENSORRT_INCLUDE_DIR})
    list(APPEND COMMON_LIBS
        ${TENSORRT_LIBRARY}
        ${TENSORRT_ONNX_PARSER_LIBRARY}
        CUDA::cudart)
endif()
if(GSTREAMER_FOUND AND GST_APP_FOUND AND GST_RTSP_FOUND AND GST_RTSP_SERVER_FOUND)
    list(APPEND COMMON_LIBS ${GSTREAMER_LIBRARIES} ${GST_APP_LIBRARIES} ${GST_RTSP_LIBRARIES} ${GST_RTSP_SERVER_LIBRARIES})
endif()
if(HAS_REALSENSE)
    list(APPEND COMMON_LIBS realsense2::realsense2)
endif()
if(HAS_ORBBEC)
    list(APPEND COMMON_LIBS ob::OrbbecSDK)
endif()

set(COMMON_SOURCES
    src/common/logger.cpp
    src/common/config.cpp
    src/common/odom_transform.cpp
    src/common/track_velocity.cpp
    src/common/timer.cpp
    src/common/preprocess.cpp
    src/common/postprocess.cpp
    src/common/dual_model_merge.cpp
    src/modules/module_base.cpp
    src/modules/rgbd_capture_factory.cpp
    src/modules/bytetrack.cpp
    src/modules/depth_sampler.cpp
    src/modules/rgb_depth_registration.cpp
    src/modules/hevc_encoder.cpp
    src/modules/odom_buffer.cpp
    src/modules/inference_module.cpp
    src/modules/int8_plan.cpp
    src/output/rns_source_journal.cpp
    src/output/rns_fused_objects.cpp
    src/output/rns_contract.cpp
    src/supervisor/supervisor.cpp
)
# The standalone bit-level normalization contract excludes fast-math in every
# target compiling this dispatcher; legacy/fused remain in the same binary.
set_source_files_properties(src/common/preprocess.cpp PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
if(HAS_REALSENSE)
    list(APPEND COMMON_SOURCES src/modules/d435_capture.cpp)
endif()
if(HAS_ORBBEC)
    list(APPEND COMMON_SOURCES
        src/modules/orbbec_capture.cpp
        src/modules/orbbec_native_depth.cpp
        src/modules/native_depth.cpp
        src/modules/hw_mjpeg_decoder.cpp
        src/modules/jpeg_ycc.cpp)
endif()
if(ENABLE_FRAME_SYNC_SEI)
    list(APPEND COMMON_SOURCES
        src/common/frame_sync_gst.cpp
        src/common/frame_sync_sei.cpp)
endif()

# ================= Formal R6 service =================
# ENABLE_R6_SERVICE=ON: lib/perception/perception_node is the formal R6 service
# (19 section 1.1, contract systemd ExecStart). The legacy V5 ROS2 JSON node is no
# longer built (level-2 trim 2026-09-28; history: tag pre-level2-trim-20260928).
option(ENABLE_R6_SERVICE "Build/install the formal R6 perception service as perception_node" ON)
set(RNS_UPSTREAM_COMMON "" CACHE PATH "Verified upstream contract_r6/common/include (yaml_lite)")
if(NOT RNS_UPSTREAM_COMMON AND XBRAIN_COMMON_ROOT)
    set(RNS_UPSTREAM_COMMON "${XBRAIN_COMMON_ROOT}/include")
endif()

set(RNS_ZENOH_ROOT "" CACHE PATH "Pinned isolated zenoh-c 1.9.0 SDK")

if(ENABLE_R6_SERVICE)
    if(NOT HAS_ORBBEC OR NOT HAS_TENSORRT OR NOT ENABLE_FRAME_SYNC_SEI)
        message(FATAL_ERROR "R6 service needs Orbbec/TRT/SEI")
    endif()
    if(NOT EXISTS "${RNS_ZENOH_ROOT}/include/zenoh_configure.h" OR NOT EXISTS "${RNS_ZENOH_ROOT}/lib/libzenohc.a")
        message(FATAL_ERROR "R6 service requires the pinned zenoh-c 1.9.0 SDK (RNS_ZENOH_ROOT)")
    endif()
    file(READ "${RNS_ZENOH_ROOT}/include/zenoh_configure.h" R6_ZENOH_CONFIG)
    if(NOT R6_ZENOH_CONFIG MATCHES "#define ZENOH_C_MAJOR 1" OR NOT R6_ZENOH_CONFIG MATCHES "#define ZENOH_C_MINOR 9" OR
       NOT R6_ZENOH_CONFIG MATCHES "#define ZENOH_C_PATCH 0")
        message(FATAL_ERROR "R6 service pins zenoh-c 1.9.0")
    endif()
    if(NOT EXISTS "${RNS_UPSTREAM_COMMON}/xbrain/config/yaml_lite.h")
        message(FATAL_ERROR "R6 service requires RNS_UPSTREAM_COMMON with xbrain/config/yaml_lite.h")
    endif()
    # Gemini-only service (PSC-3): the D435/RealSense capture is not compiled or
    # linked, so the installed service has no librealsense runtime dependency.
    set(R6_COMMON_SOURCES ${COMMON_SOURCES})
    list(REMOVE_ITEM R6_COMMON_SOURCES src/modules/d435_capture.cpp)
    set(R6_COMMON_LIBS ${COMMON_LIBS})
    list(REMOVE_ITEM R6_COMMON_LIBS realsense2::realsense2)
    set(R6_SERVICE_SOURCES
        src/service/perception_service_main.cpp
        src/service/profile_steady_loop.cpp
        src/service/service_config.cpp
        src/service/gen_notices.cpp
        src/service/ros_bridge.cpp
        src/service/calibration_record.cpp
        src/service/geometry_setup.cpp
        src/common/resolved_yaml.cpp
        src/common/rns_geometry_config.cpp
        src/modules/ground_fit.cpp
        src/modules/native_geometry.cpp
        src/modules/grid_traversal.cpp
        src/modules/normal_grid.cpp
        src/modules/traversability_kernel.cpp
        src/modules/normal_traversability.cpp
        src/modules/t_color_projection.cpp
        src/modules/orbbec_native_rays.cpp
        src/service/object_geometry_setup.cpp
        src/service/semantic_slot.cpp
        src/service/t_mask_slot.cpp
        src/service/t_projection_setup.cpp
        src/service/t_producer.cpp
        src/service/psc4_golden.cpp
        src/service/psc4_run.cpp
        ${R6_COMMON_SOURCES})
    foreach(component zenoh_transport system_clock cadence envelope publisher status_heartbeat
        publication_journal publication_handoff objects_release health_fields health_window depth_health
        unconfigured_profile capture_estimate gen_events gen_transport reason_edges
        native_profile normal_profile withdrawn_profile withdrawn_evidence semantic semantic_kernel object_geometry)
        list(APPEND R6_SERVICE_SOURCES src/output/rns_${component}.cpp)
    endforeach()
    add_executable(perception_node ${R6_SERVICE_SOURCES})
    ament_target_dependencies(perception_node rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs)
    target_include_directories(perception_node SYSTEM PRIVATE "${RNS_ZENOH_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
    target_compile_definitions(perception_node PRIVATE HAS_TENSORRT PERCEPTION_FRAME_SYNC_SEI=1)
    target_link_libraries(perception_node ${R6_COMMON_LIBS} "${RNS_ZENOH_ROOT}/lib/libzenohc.a" ${CMAKE_DL_LIBS} m rt)
    target_compile_options(perception_node PRIVATE -Wall -Wextra -O2 -UHAS_REALSENSE)
    # A19-LINT-1: the static scans (no wall clock in a timestamp, no perception data on a ROS
    # topic, no hardcoded endpoint) run over exactly what is compiled into perception_node. The
    # list is written from the target's own sources here, so a unit added above is scanned
    # without being listed anywhere else; the script follows the project headers they include.
    string(REPLACE ";" "\n" R6_SERVICE_SOURCE_LINES "${R6_SERVICE_SOURCES}")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/perception_node_sources.txt" "${R6_SERVICE_SOURCE_LINES}\n")
    find_program(PERCEPTION_PYTHON3 python3)
    if(PERCEPTION_PYTHON3)
        add_test(NAME lint_scan COMMAND ${PERCEPTION_PYTHON3} "${CMAKE_CURRENT_SOURCE_DIR}/tests/lint/lint_scan.py"
            "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/perception_node_sources.txt"
            "${CMAKE_CURRENT_BINARY_DIR}/lint_scan.json" --self-test)
    else()
        # Without an interpreter the scan cannot run; that is a failed test, not a skipped one.
        add_test(NAME lint_scan COMMAND ${CMAKE_COMMAND} -E false)
    endif()
    # Installed layout (prefix = /opt/xbrain_v6/data/install/perception in the
    # contract unit): the Orbbec SDK runtime (with its extensions/ tree) is
    # bundled next to the service; system CUDA/TensorRT/OpenCV stay external.
    if(XBRAIN_COMMON_ROOT)
        # The platform keeps the SDK (with extensions/) in common/: run it from
        # there instead of bundling a second copy next to the service.
        set_target_properties(perception_node PROPERTIES
            INSTALL_RPATH "${XBRAIN_ORBBEC_LIB_DIR}:/usr/local/cuda/lib64:/usr/local/lib:/opt/ros/humble/lib")
        install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
    else()
        set_target_properties(perception_node PROPERTIES
            INSTALL_RPATH "\$ORIGIN/../orbbec_sdk/lib:/usr/local/cuda/lib64:/usr/local/lib:/opt/ros/humble/lib")
        install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
        get_filename_component(R6_ORBBEC_LIB_DIR "${OrbbecSDK_DIR}" ABSOLUTE)
        if(NOT EXISTS "${R6_ORBBEC_LIB_DIR}/libOrbbecSDK.so.2" OR NOT IS_DIRECTORY "${R6_ORBBEC_LIB_DIR}/extensions")
            message(FATAL_ERROR "R6 install needs OrbbecSDK_DIR pointing at the SDK lib/ directory with extensions/")
        endif()
        install(DIRECTORY "${R6_ORBBEC_LIB_DIR}/" DESTINATION lib/orbbec_sdk/lib USE_SOURCE_PERMISSIONS
            PATTERN "*.cmake" EXCLUDE PATTERN "*.md" EXCLUDE)
    endif()
    install(FILES deploy/perception/systemd/10-perception-r6.conf DESTINATION share/${PROJECT_NAME}/deploy/systemd)
    install(FILES deploy/perception/config/perception.pipeline.dev_candR.example.yaml DESTINATION share/${PROJECT_NAME}/config)

    # Bench extrinsic tool (19 section 7.1, reply Q13): isolated bench use, NOT installed.
    add_executable(perception_bench_calib
        src/service/bench_calib_main.cpp
        src/service/bench_calibration.cpp
        src/service/calibration_record.cpp
        src/service/service_config.cpp
        src/common/resolved_yaml.cpp
        src/modules/ground_fit.cpp
        src/modules/native_geometry.cpp
        src/modules/orbbec_native_rays.cpp
        ${R6_COMMON_SOURCES})
    target_include_directories(perception_bench_calib SYSTEM PRIVATE "${RNS_ZENOH_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
    target_compile_definitions(perception_bench_calib PRIVATE HAS_TENSORRT PERCEPTION_FRAME_SYNC_SEI=1)
    target_link_libraries(perception_bench_calib ${R6_COMMON_LIBS} "${RNS_ZENOH_ROOT}/lib/libzenohc.a" ${CMAKE_DL_LIBS} m rt)
    target_compile_options(perception_bench_calib PRIVATE -Wall -Wextra -O2 -UHAS_REALSENSE)

    # PSC-4 sealing tool: one-off, at installation / candidate time (diagnostic,
    # NOT installed, built only on request).
    add_executable(perception_psc4_seal EXCLUDE_FROM_ALL
        src/service/psc4_seal_main.cpp
        src/service/psc4_golden.cpp
        src/service/psc4_run.cpp
        src/service/service_config.cpp
        src/common/resolved_yaml.cpp
        src/common/rns_geometry_config.cpp
        src/service/calibration_record.cpp
        src/modules/native_geometry.cpp
        src/modules/ground_fit.cpp
        src/modules/normal_grid.cpp
        src/modules/grid_traversal.cpp
        ${R6_COMMON_SOURCES})
    target_include_directories(perception_psc4_seal SYSTEM PRIVATE "${RNS_ZENOH_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
    target_compile_definitions(perception_psc4_seal PRIVATE HAS_TENSORRT PERCEPTION_FRAME_SYNC_SEI=1)
    target_link_libraries(perception_psc4_seal ${R6_COMMON_LIBS} "${RNS_ZENOH_ROOT}/lib/libzenohc.a" ${CMAKE_DL_LIBS} m rt)
    target_compile_options(perception_psc4_seal PRIVATE -Wall -Wextra -O2 -UHAS_REALSENSE)

    # Target-machine engine build with a recorded build environment (19 section
    # 13.2; installation tool, NOT installed, built only on request).
    add_executable(perception_engine_build EXCLUDE_FROM_ALL
        src/service/engine_build_main.cpp
        src/service/psc4_golden.cpp
        src/service/psc4_run.cpp
        src/service/service_config.cpp
        src/common/resolved_yaml.cpp
        src/common/rns_geometry_config.cpp
        src/service/calibration_record.cpp
        src/modules/native_geometry.cpp
        src/modules/ground_fit.cpp
        src/modules/normal_grid.cpp
        src/modules/grid_traversal.cpp
        ${R6_COMMON_SOURCES})
    target_include_directories(perception_engine_build SYSTEM PRIVATE "${RNS_ZENOH_ROOT}/include" "${RNS_UPSTREAM_COMMON}")
    target_compile_definitions(perception_engine_build PRIVATE HAS_TENSORRT PERCEPTION_FRAME_SYNC_SEI=1)
    target_link_libraries(perception_engine_build ${R6_COMMON_LIBS} "${RNS_ZENOH_ROOT}/lib/libzenohc.a" ${CMAKE_DL_LIBS} m rt)
    target_compile_options(perception_engine_build PRIVATE -Wall -Wextra -O2 -UHAS_REALSENSE)

endif()

# ================= unit tests (dev, not installed) =================
add_executable(test_orbbec_frame_output tests/test_orbbec_frame_output.cpp)
target_link_libraries(test_orbbec_frame_output PRIVATE ${OpenCV_LIBS})
target_compile_options(test_orbbec_frame_output PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_orbbec_frame_output COMMAND test_orbbec_frame_output)
foreach(name jit_schedule jit_fresh_bind jit_pace persistent_task_pair)
    add_executable(test_${name} tests/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE Threads::Threads)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
    add_test(NAME test_${name} COMMAND test_${name})
endforeach()
add_executable(test_jit_second_chance tests/test_jit_second_chance.cpp src/output/rns_source_journal.cpp)
target_link_libraries(test_jit_second_chance PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_jit_second_chance PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_jit_second_chance COMMAND test_jit_second_chance)
foreach(name depth_codesort fixed_two_lane_depth_sampler depth_scan_fast_identity)
    add_executable(test_${name} tests/test_${name}.cpp src/modules/depth_sampler.cpp)
    target_link_libraries(test_${name} PRIVATE ${OpenCV_LIBS} Threads::Threads)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
    add_test(NAME test_${name} COMMAND test_${name})
endforeach()
add_executable(test_active_mask_parallel tests/test_active_mask_parallel.cpp
    tests/reference/active_mask_parent_postprocess.cpp
    src/common/postprocess.cpp src/common/preprocess.cpp)
target_include_directories(test_active_mask_parallel PRIVATE tests/reference)
target_link_libraries(test_active_mask_parallel PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_active_mask_parallel PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_active_mask_parallel COMMAND test_active_mask_parallel)
add_executable(test_active12_rect_decode tests/test_active12_rect_decode.cpp
    src/common/postprocess.cpp src/common/preprocess.cpp)
target_link_libraries(test_active12_rect_decode PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_active12_rect_decode PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_active12_rect_decode COMMAND test_active12_rect_decode)
add_executable(test_linear_bbox_resize tests/test_linear_bbox_resize.cpp
    src/common/postprocess.cpp src/common/preprocess.cpp)
target_link_libraries(test_linear_bbox_resize PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_linear_bbox_resize PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_linear_bbox_resize COMMAND test_linear_bbox_resize)
add_executable(test_jpeg_ycc tests/test_jpeg_ycc.cpp src/modules/jpeg_ycc.cpp)
target_compile_options(test_jpeg_ycc PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_jpeg_ycc COMMAND test_jpeg_ycc)
# Needs the Jetson hardware decoder (nvv4l2decoder); built where GStreamer and gst-app exist.
if(GSTREAMER_FOUND AND GST_APP_FOUND)
    add_executable(test_hw_mjpeg_decoder tests/test_hw_mjpeg_decoder.cpp src/modules/hw_mjpeg_decoder.cpp
        src/modules/jpeg_ycc.cpp)
    target_include_directories(test_hw_mjpeg_decoder PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})
    target_link_libraries(test_hw_mjpeg_decoder PRIVATE ${OpenCV_LIBS} ${GSTREAMER_LIBRARIES} ${GST_APP_LIBRARIES})
    target_compile_options(test_hw_mjpeg_decoder PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
    add_test(NAME test_hw_mjpeg_decoder COMMAND test_hw_mjpeg_decoder)
endif()
add_executable(test_t_producer tests/test_t_producer.cpp src/service/t_producer.cpp src/service/t_mask_slot.cpp
    src/common/dual_model_merge.cpp src/common/postprocess.cpp src/common/preprocess.cpp)
target_link_libraries(test_t_producer PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_t_producer PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_t_producer COMMAND test_t_producer)
foreach(name execution_cpu_trace capture_affinity_control sdk_stream_thread_placement profile_thread_placement
        parallel_dispatch_trace post_role_dispatch top_class mask_area_window declared_keys camera_identity
        capture_clock capture_time_state scene_luma_window)
    add_executable(test_${name} tests/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE Threads::Threads)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
    add_test(NAME test_${name} COMMAND test_${name})
endforeach()
add_executable(test_execution_cpu_postprocess tests/test_execution_cpu_postprocess.cpp
    src/common/postprocess.cpp src/common/preprocess.cpp src/common/dual_model_merge.cpp)
target_link_libraries(test_execution_cpu_postprocess PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_execution_cpu_postprocess PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_execution_cpu_postprocess COMMAND test_execution_cpu_postprocess)
add_executable(test_orbbec_inference_handoff tests/test_orbbec_inference_handoff.cpp)
target_link_libraries(test_orbbec_inference_handoff PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_orbbec_inference_handoff PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_orbbec_inference_handoff COMMAND test_orbbec_inference_handoff)
add_executable(test_rns_jit_source tests/test_rns_jit_source.cpp src/output/rns_source_journal.cpp)
target_link_libraries(test_rns_jit_source PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_rns_jit_source PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_rns_jit_source COMMAND test_rns_jit_source)

add_executable(test_rns_formal_window tests/test_rns_formal_window.cpp)
target_link_libraries(test_rns_formal_window PRIVATE ${OpenCV_LIBS})
target_compile_options(test_rns_formal_window PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_rns_formal_window COMMAND test_rns_formal_window)
add_executable(test_rns_fused_objects tests/test_rns_fused_objects.cpp
    src/output/rns_fused_objects.cpp src/output/rns_contract.cpp
    src/output/rns_source_journal.cpp src/modules/native_depth.cpp
    src/modules/bytetrack.cpp src/common/config.cpp src/common/logger.cpp)
target_link_libraries(test_rns_fused_objects PRIVATE ${OpenCV_LIBS} yaml-cpp Threads::Threads)
target_compile_options(test_rns_fused_objects PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_rns_fused_objects COMMAND test_rns_fused_objects)
add_executable(test_supervisor_rns_context tests/test_supervisor_rns_context.cpp ${COMMON_SOURCES})
target_link_libraries(test_supervisor_rns_context PRIVATE ${COMMON_LIBS})
target_compile_options(test_supervisor_rns_context PRIVATE -Wall -Wextra -O2 -UNDEBUG)
if(HAS_TENSORRT)
    target_compile_definitions(test_supervisor_rns_context PRIVATE HAS_TENSORRT)
endif()
if(ENABLE_FRAME_SYNC_SEI)
    target_compile_definitions(test_supervisor_rns_context PRIVATE PERCEPTION_FRAME_SYNC_SEI=1)
endif()
add_test(NAME test_supervisor_rns_context COMMAND test_supervisor_rns_context)

# Coasting tracks and the object geometry (2026-10-13). The hook test calls the service's own
# measure_object_geometry(); the sequence test drives the real Supervisor::process_loop() from a
# test queue with two independent states (geometry built for every track / the service's hook).
add_executable(test_object_geometry_hook tests/test_object_geometry_hook.cpp
    src/output/rns_object_geometry.cpp src/output/rns_contract.cpp src/modules/rgb_depth_registration.cpp)
target_link_libraries(test_object_geometry_hook PRIVATE ${OpenCV_LIBS} yaml-cpp Threads::Threads)
target_compile_options(test_object_geometry_hook PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
add_test(NAME test_object_geometry_hook COMMAND test_object_geometry_hook)
add_executable(test_supervisor_coast_sequence tests/test_supervisor_coast_sequence.cpp
    src/output/rns_object_geometry.cpp ${COMMON_SOURCES})
target_link_libraries(test_supervisor_coast_sequence PRIVATE ${COMMON_LIBS})
target_compile_options(test_supervisor_coast_sequence PRIVATE -Wall -Wextra -O2 -UNDEBUG)
if(HAS_TENSORRT)
    target_compile_definitions(test_supervisor_coast_sequence PRIVATE HAS_TENSORRT)
endif()
if(ENABLE_FRAME_SYNC_SEI)
    target_compile_definitions(test_supervisor_coast_sequence PRIVATE PERCEPTION_FRAME_SYNC_SEI=1)
endif()
add_test(NAME test_supervisor_coast_sequence COMMAND test_supervisor_coast_sequence)

add_executable(test_queue tests/test_queue.cpp)
target_link_libraries(test_queue PRIVATE Threads::Threads)
target_compile_options(test_queue PRIVATE -Wall -Wextra -O2)

add_executable(test_result_handoff_ledger tests/test_result_handoff_ledger.cpp)
target_link_libraries(test_result_handoff_ledger PRIVATE Threads::Threads)
target_compile_options(test_result_handoff_ledger PRIVATE -Wall -Wextra -O2)

add_executable(test_preprocess tests/test_preprocess.cpp src/common/preprocess.cpp)
target_link_libraries(test_preprocess PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_preprocess PRIVATE -Wall -Wextra -O2 -UNDEBUG)

foreach(name fused_preprocess preprocess_dispatch neon_preprocess window_preprocess)
    add_executable(test_${name} tests/test_${name}.cpp src/common/preprocess.cpp)
    target_include_directories(test_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/common)
    target_link_libraries(test_${name} PRIVATE ${OpenCV_LIBS} Threads::Threads)
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG -fno-fast-math)
    add_test(NAME test_${name} COMMAND test_${name})
endforeach()

add_executable(test_postprocess tests/test_postprocess.cpp src/common/postprocess.cpp src/common/preprocess.cpp)
target_link_libraries(test_postprocess PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_postprocess PRIVATE -Wall -Wextra -O2)
# Work counters of the last decode per thread (2026-10-16): one run per COCO80 mask mode, because
# both mode switches are read once per process.
add_executable(test_post_work_counts tests/test_post_work_counts.cpp src/common/postprocess.cpp src/common/preprocess.cpp)
target_link_libraries(test_post_work_counts PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_post_work_counts PRIVATE -Wall -Wextra -Werror -O2 -UNDEBUG)
foreach(mode roi2 roi full)
    add_test(NAME test_post_work_counts_${mode} COMMAND test_post_work_counts ${mode})
endforeach()

add_executable(test_postprocess_mask_ownership
    tests/test_postprocess_mask_ownership.cpp
    src/common/postprocess.cpp
    src/common/preprocess.cpp)
target_link_libraries(test_postprocess_mask_ownership PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_postprocess_mask_ownership PRIVATE -Wall -Wextra -O2 -UNDEBUG)
add_test(NAME test_postprocess_mask_ownership COMMAND test_postprocess_mask_ownership)

add_executable(test_dual_model_merge
    tests/test_dual_model_merge.cpp
    src/common/dual_model_merge.cpp
    src/common/postprocess.cpp
    src/common/preprocess.cpp)
target_link_libraries(test_dual_model_merge PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(test_dual_model_merge PRIVATE -Wall -Wextra -O2)

add_executable(test_config_env tests/test_config_env.cpp src/common/config.cpp)
target_link_libraries(test_config_env PRIVATE yaml-cpp Threads::Threads)
target_compile_options(test_config_env PRIVATE -Wall -Wextra -O2)

add_executable(test_int8_plan tests/test_int8_plan.cpp src/modules/int8_plan.cpp src/common/config.cpp)
target_link_libraries(test_int8_plan PRIVATE yaml-cpp Threads::Threads)
target_compile_options(test_int8_plan PRIVATE -Wall -Wextra -Werror -O2)

add_executable(test_rgbd_capture_factory
    tests/test_rgbd_capture_factory.cpp
    src/modules/rgbd_capture_factory.cpp
    src/modules/module_base.cpp
    src/common/config.cpp)
target_link_libraries(test_rgbd_capture_factory PRIVATE
    ${OpenCV_LIBS}
    yaml-cpp
    Threads::Threads)
target_compile_options(test_rgbd_capture_factory PRIVATE -Wall -Wextra -O2)
if(HAS_REALSENSE)
    target_sources(test_rgbd_capture_factory PRIVATE src/modules/d435_capture.cpp)
    target_link_libraries(test_rgbd_capture_factory PRIVATE realsense2::realsense2)
endif()
if(HAS_ORBBEC)
    target_sources(test_rgbd_capture_factory PRIVATE
        src/output/rns_source_journal.cpp
        src/modules/orbbec_capture.cpp
        src/modules/rgb_depth_registration.cpp
        src/modules/orbbec_native_depth.cpp
        src/modules/native_depth.cpp
        src/modules/hw_mjpeg_decoder.cpp
        src/modules/jpeg_ycc.cpp)
    target_link_libraries(test_rgbd_capture_factory PRIVATE ob::OrbbecSDK ${GSTREAMER_LIBRARIES} ${GST_APP_LIBRARIES})
endif()

add_executable(test_time_utils tests/test_time_utils.cpp)
target_compile_options(test_time_utils PRIVATE -Wall -Wextra -O2)

add_executable(test_odom_buffer tests/test_odom_buffer.cpp src/modules/odom_buffer.cpp src/common/config.cpp)
target_link_libraries(test_odom_buffer PRIVATE yaml-cpp Threads::Threads)
target_compile_options(test_odom_buffer PRIVATE -Wall -Wextra -O2)

add_executable(test_bytetrack tests/test_bytetrack.cpp src/modules/bytetrack.cpp src/common/config.cpp)
target_link_libraries(test_bytetrack PRIVATE ${OpenCV_LIBS} yaml-cpp)
target_compile_options(test_bytetrack PRIVATE -Wall -Wextra -O2)

add_executable(test_depth_sampler tests/test_depth_sampler.cpp src/modules/depth_sampler.cpp)
target_link_libraries(test_depth_sampler PRIVATE ${OpenCV_LIBS})
target_compile_options(test_depth_sampler PRIVATE -Wall -Wextra -O2)

add_executable(test_track_velocity tests/test_track_velocity.cpp src/common/track_velocity.cpp)
target_link_libraries(test_track_velocity PRIVATE ${OpenCV_LIBS})
target_compile_options(test_track_velocity PRIVATE -Wall -Wextra -O2)

add_executable(test_frame_sync_sei
    tests/test_frame_sync_sei.cpp
    src/common/frame_sync_sei.cpp)
target_compile_options(test_frame_sync_sei PRIVATE -Wall -Wextra -O2)

# Keep the existing lightweight test executables usable through CTest. Release
# builds define NDEBUG, which would otherwise compile out their assert() checks
# and turn successful exits into false positives.
set(PERCEPTION_UNIT_TEST_TARGETS
    test_queue
    test_result_handoff_ledger
    test_preprocess
    test_postprocess
    test_dual_model_merge
    test_config_env
    test_int8_plan
    test_rgbd_capture_factory
    test_time_utils
    test_odom_buffer
    test_bytetrack
    test_depth_sampler
    test_track_velocity
    test_frame_sync_sei)

if(BUILD_TESTING)
    foreach(test_target IN LISTS PERCEPTION_UNIT_TEST_TARGETS)
        add_test(NAME ${test_target} COMMAND $<TARGET_FILE:${test_target}>)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${test_target} PRIVATE -UNDEBUG)
        endif()
    endforeach()
endif()


# Development-only targets; absent from the delivered package (harness/export_delivery.py).
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dev_extras.cmake OPTIONAL)

ament_package()
