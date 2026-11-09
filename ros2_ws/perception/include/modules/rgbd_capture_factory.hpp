/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rgbd_capture_factory.hpp
 * Brief: Fail-closed construction of the configured RGB-D camera driver.
 *
 * Description:
 * The key camera.driver names exactly one driver; an unknown name, a driver this build lacks, or R6
 * native hooks without a Gemini mailbox return nullptr with the reason, never a fallback.
 * R6 builds have Orbbec only (HAS_REALSENSE is undefined; the D435 driver is not delivered).
 */

#pragma once

#include <memory>
#include <string>

#include "common/config.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/rgbd_capture.hpp"

namespace perception {
namespace native_depth { class Mailbox; }
namespace rns { class SourceJournal; }

/**
 * @brief Creates the exact driver named by camera.driver.
 *
 * On failure, returns nullptr and writes a human-readable reason to error.
 * The factory never falls back to another camera implementation.
 */
std::unique_ptr<RgbdCapture> create_rgbd_capture(
    const yolo::Config& config,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue,
    std::string& error,
    std::shared_ptr<native_depth::Mailbox> native_depth_output = nullptr,
    rns::SourceJournal* health_arrivals = nullptr);

}  // namespace perception
