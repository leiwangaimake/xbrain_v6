/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rgbd_capture.hpp
 * Brief: Camera-independent RGB-D capture lifecycle contract.
 *
 * Description:
 * The Supervisor owns a capture through this base class (start/stop/is_running from
 * ModuleBase) and reads the stream calibration from it. R6 has one implementation,
 * OrbbecCapture.
 */

#pragma once

#include <optional>

#include <string>
#include <utility>

#include "modules/module_base.hpp"

namespace perception {

/**
 * @brief Common lifecycle boundary for synchronized color and aligned-depth capture.
 *
 * Concrete drivers keep ownership of their SDK resources and must publish the
 * existing yolo::FrameData contract to the queues supplied at construction.
 */
class RgbdCapture : public yolo::ModuleBase {
public:
    ~RgbdCapture() override = default;
    // Calibration of the current stream start; nullopt when the driver does not
    // provide it or the stream has not started. Thread-safe.
    virtual std::optional<yolo::RgbdCalibration> rgbd_calibration() const { return std::nullopt; }

protected:
    explicit RgbdCapture(std::string module_name)
        : yolo::ModuleBase(std::move(module_name)) {}
};

}  // namespace perception
