/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_capture.hpp
 * Brief: Orbbec Gemini 338Le synchronized RGB-D capture module.
 *
 * Description:
 * The only camera driver of R6 (rgbd_capture_factory). One capture thread (cap_loop) feeds the
 * inference queue, the MED-2 encoder queue and, in native SW-D2C mode, the Profile mailbox of
 * pre-alignment depth. See orbbec_capture.cpp for the per-frame steps and the failure policy.
 */

#pragma once

#include <optional>

#include <mutex>

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "common/config.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/rgbd_capture.hpp"

namespace perception {
namespace native_depth { class Mailbox; }
namespace rns { class SourceJournal; }

/** @brief Captures Gemini 338Le color and explicitly selected aligned depth frames. */
class OrbbecCapture final : public RgbdCapture {
public:
    OrbbecCapture(
        const yolo::Config& config,
        std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue,
        std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue = nullptr,
        std::shared_ptr<native_depth::Mailbox> native_depth_output = nullptr,
        rns::SourceJournal* health_arrivals = nullptr);
    ~OrbbecCapture() override;

    bool start() override;
    void stop() override;
    // Lifecycle-owner audit, read together only after start/stop returns.
    // Worker/mailbox shutdown is separate from SDK stop acknowledgement.
    // Cumulative across restarts; these do not claim hardware power-off or
    // change the existing exception/cleanup behaviour of the capture path.
    std::uint64_t sdk_stop_attempts() const noexcept { return sdk_stop_attempts_.load(); }
    std::uint64_t sdk_stop_successes() const noexcept { return sdk_stop_successes_.load(); }
    std::uint64_t sdk_stop_failures() const noexcept { return sdk_stop_failures_.load(); }
    // SW-D2C only: colour intrinsics/distortion and native depth->colour
    // extrinsic reported by the SDK for the current stream start.
    std::optional<yolo::RgbdCalibration> rgbd_calibration() const override;

private:
    struct Impl;
    mutable std::mutex calibration_mutex_;
    std::optional<yolo::RgbdCalibration> calibration_;

    bool config_is_valid() const;
    void capture_loop();

    const yolo::Config& config_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue_;
    // Opt-in pre-D2C latest slot. Default nullptr preserves legacy operation.
    // Only a SW-D2C capture with native 640x400 input may enable this output.
    std::shared_ptr<native_depth::Mailbox> native_depth_output_;
    // Optional native-mode raw-depth observations, before RGB/D2C validity.
    // Caller owns this SPSC producer's lifetime through stop().
    rns::SourceJournal* health_arrivals_;
    std::thread worker_;
    std::atomic<std::uint64_t> frame_counter_{0};
    std::atomic<std::uint64_t> sdk_stop_attempts_{0}, sdk_stop_successes_{0}, sdk_stop_failures_{0};
    std::unique_ptr<Impl> impl_;
};

}  // namespace perception
