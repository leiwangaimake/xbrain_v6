/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rgbd_capture_factory.cpp
 * Brief: M1 RGB-D driver selection and fail-closed regression tests.
 *
 * Description:
 * What it checks. The capture factory picks the driver named in the configuration. The tests cover
 * the interface and the default driver, that an unknown driver fails closed, an explicit D435
 * selection, and that Gemini and D435 availability match what the build actually contains (the
 * delivered build has no D435). Every config carries the bundled sealed dual-model contracts,
 * because dual_model.enabled is required and must be true.
 */

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>

#include "common/config.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/d435_capture.hpp"
#include "modules/rgbd_capture.hpp"
#include "modules/rgbd_capture_factory.hpp"
#include "modules/orbbec_capture.hpp"

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

// dual_model.enabled is required and must be true (level-3 trim): every config
// below carries the bundled sealed contracts by absolute path.
std::string sealed_dual_model_yaml() {
    std::filesystem::path source_file(__FILE__);
    if (source_file.is_relative()) {
        source_file = std::filesystem::absolute(source_file);
    }
    const std::filesystem::path root = source_file.parent_path().parent_path();
    return "dual_model:\n"
           "  enabled: true\n"
           "  runtime_config_path: \"" + (root / "config/dual_model_runtime_config.json").string() + "\"\n"
           "  class_schema_path: \"" + (root / "config/class_schema_94.json").string() + "\"\n";
}

yolo::Config load_config(const std::string& path, const std::string& yaml) {
    std::ofstream file(path);
    require(file.good(), "failed to open temporary camera config");
    file << yaml << sealed_dual_model_yaml();
    file.close();

    yolo::Config config;
    require(config.load(path), "failed to load temporary camera config");
    return config;
}

std::unique_ptr<perception::RgbdCapture> create(
    const yolo::Config& config,
    std::string& error) {
    auto inference_queue =
        std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(2U);
    auto encode_queue =
        std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(2U);
    return perception::create_rgbd_capture(
        config,
        std::move(inference_queue),
        std::move(encode_queue),
        error);
}

// Both camera classes implement RgbdCapture, and a config without camera.driver means d435.
void test_interface_and_default_driver() {
    static_assert(
        std::is_base_of<perception::RgbdCapture, perception::D435Capture>::value,
        "D435Capture must implement the camera-independent RgbdCapture contract");
    static_assert(
        std::is_base_of<perception::RgbdCapture, perception::OrbbecCapture>::value,
        "OrbbecCapture must implement the camera-independent RgbdCapture contract");

    const yolo::Config config = load_config(
        "/tmp/xbrain_rgbd_driver_default.yaml",
        "log:\n  level: INFO\n");
    require(config.camera_driver() == "d435", "missing camera.driver did not preserve D435 default");
}

// An unknown camera.driver creates no capture, and the error names the rejected value.
void test_unknown_driver_fails_closed() {
    const yolo::Config config = load_config(
        "/tmp/xbrain_rgbd_driver_unknown.yaml",
        "camera:\n  driver: definitely_unknown\n");
    std::string error;
    const auto capture = create(config, error);
    require(!capture, "unknown camera.driver unexpectedly created a capture module");
    require(
        error.find("unsupported camera.driver='definitely_unknown'") != std::string::npos,
        "unknown driver error did not identify the rejected value");
}

// An explicit d435 driver is kept.
void test_explicit_d435_driver() {
    const yolo::Config config = load_config(
        "/tmp/xbrain_rgbd_driver_explicit_d435.yaml",
        "camera:\n  driver: d435\n");
    require(config.camera_driver() == "d435", "explicit D435 driver was not preserved");
}

// The Gemini driver constructs OrbbecCapture only in an Orbbec build; otherwise it fails with an
// explicit reason.
void test_gemini_availability_matches_build() {
    const yolo::Config config = load_config(
        "/tmp/xbrain_rgbd_driver_gemini_m2.yaml",
        "hevc:\n"
        "  enable: true\n"
        "  sink: file_rtsp\n"
        "  rtsp_listen: rtsp://0.0.0.0:8555/rgb\n"
        "  width: 1920\n"
        "  height: 1080\n"
        "  fps: 30\n"
        "  bitrate: 4000000\n"
        "  codec: nvv4l2h265enc\n"
        "  gop: 30\n"
        "camera:\n"
        "  driver: orbbec_gemini338le\n"
        "  serial: CHL5663000DS\n"
        "  align_to_color: true\n"
        "  color:\n"
        "    width: 1280\n"
        "    height: 720\n"
        "    fps: 30\n"
        "    device_format: mjpg\n"
        "  depth:\n"
        "    width: 848\n"
        "    height: 530\n"
        "    fps: 30\n"
        "    device_format: y16\n"
        "  output:\n"
        "    width: 1920\n"
        "    height: 1080\n"
        "    fps: 30\n");
    std::string error;
    const auto capture = create(config, error);
#ifdef HAS_ORBBEC
    require(capture != nullptr, "Orbbec build failed to construct OrbbecCapture");
    require(error.empty(), "successful Orbbec construction returned an error");
    require(capture->name() == "OrbbecCapture", "Orbbec module identity changed");
#else
    require(!capture, "non-Orbbec build unexpectedly constructed OrbbecCapture");
    require(
        error.find("no OrbbecSDK support") != std::string::npos,
        "missing OrbbecSDK support did not fail with an explicit reason");
#endif
}

// The d435 driver constructs D435Capture only in a RealSense build; otherwise it fails with an
// explicit reason.
void test_d435_availability_matches_build() {
    const yolo::Config config = load_config(
        "/tmp/xbrain_rgbd_driver_d435.yaml",
        "camera:\n  driver: d435\n");
    std::string error;
    const auto capture = create(config, error);
#ifdef HAS_REALSENSE
    require(capture != nullptr, "RealSense build failed to construct D435Capture");
    require(error.empty(), "successful D435 construction returned an error");
    require(capture->name() == "D435Capture", "D435 module identity changed");
#else
    require(!capture, "non-RealSense build unexpectedly constructed D435Capture");
    require(
        error.find("no Intel RealSense support") != std::string::npos,
        "missing RealSense support did not fail with an explicit reason");
#endif
}

}  // namespace

int main() {
    test_interface_and_default_driver();
    test_explicit_d435_driver();
    test_unknown_driver_fails_closed();
    test_gemini_availability_matches_build();
    test_d435_availability_matches_build();
    std::cout << "PASS: RGB-D capture factory contract\n";
    return 0;
}
