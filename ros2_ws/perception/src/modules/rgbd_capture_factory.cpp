/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rgbd_capture_factory.cpp
 * Brief: Fail-closed RGB-D camera driver factory implementation.
 *
 * Description:
 * Implements rgbd_capture_factory.hpp. The d435 branch compiles only with HAS_REALSENSE,
 * which R6 builds undefine, so in R6 it reports the missing support.
 */

#include "modules/rgbd_capture_factory.hpp"

#include <memory>
#include <string>
#include <utility>

#ifdef HAS_REALSENSE
#include "modules/d435_capture.hpp"
#endif
#ifdef HAS_ORBBEC
#include "modules/orbbec_capture.hpp"
#endif

namespace perception {

std::unique_ptr<RgbdCapture> create_rgbd_capture(
    const yolo::Config& config,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue,
    std::string& error,
    std::shared_ptr<native_depth::Mailbox> native_depth_output,
    rns::SourceJournal* health_arrivals) {
    error.clear();
    const std::string driver = config.camera_driver();
    if ((native_depth_output || health_arrivals) &&
        (driver != "orbbec_gemini338le" || !native_depth_output)) {
        error = "native R6 hooks require an explicit Gemini native mailbox";
        return nullptr;
    }
#ifndef HAS_ORBBEC
    (void)native_depth_output;
    (void)health_arrivals;
#endif
#if !defined(HAS_REALSENSE) && !defined(HAS_ORBBEC)
    // Queue ownership is intentionally untouched when this build cannot create
    // the only M1 implementation.
    (void)output_queue;
    (void)encode_queue;
#endif

    if (driver == "d435") {
#ifdef HAS_REALSENSE
        return std::make_unique<D435Capture>(
            config,
            std::move(output_queue),
            std::move(encode_queue));
#else
        error = "camera.driver=d435 requested, but this build has no Intel RealSense support";
        return nullptr;
#endif
    }

    if (driver == "orbbec_gemini338le") {
#ifdef HAS_ORBBEC
        return std::make_unique<OrbbecCapture>(
            config,
            std::move(output_queue),
            std::move(encode_queue),
            std::move(native_depth_output), health_arrivals);
#else
        error =
            "camera.driver=orbbec_gemini338le requested, but this build has no OrbbecSDK support";
        return nullptr;
#endif
    }

    error = "unsupported camera.driver='" + driver +
            "' (expected d435 or orbbec_gemini338le)";
    return nullptr;
}

}  // namespace perception
