/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: d435_capture.hpp
 * Brief: Intel RealSense D435 RGB and aligned-depth capture module (V5; not built in R6).
 *
 * Description: Intel RealSense D435 RGB and aligned-depth capture module.
 * R6 builds undefine HAS_REALSENSE, so rgbd_capture_factory.cpp never includes this header,
 * and its implementation (d435_capture.cpp) is not part of the delivered package.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "common/config.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/rgbd_capture.hpp"

namespace perception {

/** @brief Captures synchronized D435 color/depth frames for the perception pipeline. */
class D435Capture final : public RgbdCapture {
public:
    D435Capture(
        const yolo::Config& config,
        std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue,
        std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue = nullptr);
    ~D435Capture() override;

    bool start() override;
    void stop() override;

private:
    struct Impl;

    bool config_is_valid() const;
    void capture_loop();

    const yolo::Config& config_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue_;
    std::thread worker_;
    std::atomic<std::uint64_t> frame_counter_{0};
    std::unique_ptr<Impl> impl_;
};

}  // namespace perception
