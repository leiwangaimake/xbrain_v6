/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_native_depth.hpp
 * Brief: Optional pre-D2C SDK bridge; never changes filters or camera settings.
 *
 * Description:
 * The capture thread calls offer_orbbec() for every complete RGB-D frameset, BEFORE alignment,
 * decode or resize, to hand the native depth raster to the Profile mailbox. It only reads the
 * SDK frames; a rejected frame disables this tap until the next stream start and never touches
 * the RGB-D path the inference uses.
 */
#pragma once
#include "modules/native_depth.hpp"
#include <libobsensor/ObSensor.hpp>
#include <memory>

namespace perception::native_depth {
// What the capture knows about this frameset beyond the SDK frames.
struct TapContext {
    bool pre_d2c = false;
    std::uint32_t expected_width = 0, expected_height = 0;
    std::uint64_t max_color_depth_skew_us = 0;
    std::uint64_t stream_id = 0, sequence = 0;
    std::int64_t host_ready_mono_ns = -1;
    // The frameset's capture time from the capture clock; a frameset without one is not offered.
    std::int64_t capture_mono_ns = -1;
    bool capture_estimated = false;
};
enum class TapError { None, NotNative, MissingFrame, Layout, Profile, Timestamp, SdkRead };
struct TapResult {
    Offer outcome = Offer::Invalid;
    TapError error = TapError::None;
    SourceId source;
};

// Borrowed SDK frames only during this call; Mailbox copies raster bytes before
// return. The reported intrinsic/distortion is retained, NOT asserted rectified
// and NOT replaced by color/aligned calibration. No device-to-host mapping or
// base_link extrinsic is fabricated. Exceptions become explicit SdkRead errors.
TapResult offer_orbbec(Mailbox&, const std::shared_ptr<ob::ColorFrame>&,
                     const std::shared_ptr<ob::DepthFrame>&, const TapContext&) noexcept;
}  // namespace perception::native_depth
