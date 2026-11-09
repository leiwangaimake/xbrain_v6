cmake_minimum_required(VERSION 3.16)
project(perception_orbbec_rays_offline LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(ORBBEC_SDK_ROOT "" CACHE PATH "Pinned real Orbbec SDK 2.9.3")
option(RAYS_SANITIZERS "Check adapter/test code with ASan and UBSan" OFF)
if(NOT EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/hpp/Utils.hpp" OR
   NOT EXISTS "${ORBBEC_SDK_ROOT}/lib/libOrbbecSDK.so")
  message(FATAL_ERROR "Need real SDK headers and library; no substitute/stub")
endif()
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_executable(test_orbbec_native_rays
  "${REPO}/tests/test_orbbec_native_rays.cpp"
  "${REPO}/src/modules/orbbec_native_rays.cpp"
  "${REPO}/src/modules/native_geometry.cpp"
  "${REPO}/src/modules/native_depth.cpp"
  "${REPO}/src/modules/ground_fit.cpp")
target_include_directories(test_orbbec_native_rays PRIVATE "${REPO}/include")
target_include_directories(test_orbbec_native_rays SYSTEM PRIVATE "${ORBBEC_SDK_ROOT}/include")
target_link_libraries(test_orbbec_native_rays PRIVATE "${ORBBEC_SDK_ROOT}/lib/libOrbbecSDK.so" Threads::Threads)
set_target_properties(test_orbbec_native_rays PROPERTIES BUILD_RPATH "${ORBBEC_SDK_ROOT}/lib")
target_compile_options(test_orbbec_native_rays PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
if(RAYS_SANITIZERS)
  target_compile_options(test_orbbec_native_rays PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(test_orbbec_native_rays PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME orbbec_native_rays_offline COMMAND test_orbbec_native_rays)
set_tests_properties(orbbec_native_rays_offline PROPERTIES TIMEOUT 60)
add_test(NAME rays_driver_process_cleanup COMMAND ${Python3_EXECUTABLE}
  -B "${REPO}/tests/test_rns_rays_cleanup.py")
set_tests_properties(rays_driver_process_cleanup PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
