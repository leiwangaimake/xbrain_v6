/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_native_depth.cpp
 * Brief: offer_orbbec(): read one native depth frame from the SDK and offer it to the mailbox.
 *
 * Description:
 * Implements orbbec_native_depth.hpp. Every SDK property the geometry relies on is checked here
 * (size, Y16 format, data size, stream intrinsics of the same size, colour/depth timestamp skew)
 * so the mailbox only ever holds a frame the geometry can interpret.
 */
#include "modules/orbbec_native_depth.hpp"
#include <cmath>
#include <limits>

namespace perception::native_depth {
TapResult offer_orbbec(Mailbox& mailbox, const std::shared_ptr<ob::ColorFrame>& color,
                     const std::shared_ptr<ob::DepthFrame>& depth, const TapContext& context) noexcept {
    TapResult result;
    if (!context.pre_d2c) { result.error = TapError::NotNative; return result; }
    if (!color || !depth) { result.error = TapError::MissingFrame; return result; }
    try {
        // Layout: the configured native size, Y16, tightly packed.
        const auto width = depth->getWidth(), height = depth->getHeight();
        if (!width || !height || width != context.expected_width || height != context.expected_height ||
            depth->getFormat() != OB_FORMAT_Y16 || !depth->getData() ||
            static_cast<std::uint64_t>(width) * height > std::numeric_limits<std::size_t>::max() / sizeof(std::uint16_t) ||
            depth->getDataSize() != static_cast<std::size_t>(width) * height * sizeof(std::uint16_t)) {
            result.error = TapError::Layout; return result;
        }
        // The stream's own intrinsics, for exactly this raster size.
        auto base_profile = depth->getStreamProfile();
        if (!base_profile) { result.error = TapError::Profile; return result; }
        auto profile = base_profile->as<ob::VideoStreamProfile>();
        if (!profile) { result.error = TapError::Profile; return result; }
        const auto intrinsic = profile->getIntrinsic();
        if (intrinsic.width <= 0 || intrinsic.height <= 0 ||
            static_cast<std::uint32_t>(intrinsic.width) != width ||
            static_cast<std::uint32_t>(intrinsic.height) != height) {
            result.error = TapError::Profile; return result;
        }
        const auto distortion = profile->getDistortion();
        // Colour and depth of one frameset must be close in device time.
        const auto color_us = color->getTimeStampUs(), depth_us = depth->getTimeStampUs();
        const auto skew = color_us > depth_us ? color_us - depth_us : depth_us - color_us;
        // A capture time is part of the offer: exposure comes before the pickup.
        if (!color_us || !depth_us || skew > context.max_color_depth_skew_us ||
            context.host_ready_mono_ns <= 0 || context.capture_mono_ns < 0 ||
            context.capture_mono_ns > context.host_ready_mono_ns) {
            result.error = TapError::Timestamp; return result;
        }
        // The borrowed view; try_offer copies the pixels before returning.
        View view;
        view.pre_d2c = true;
        view.metadata.source = {context.stream_id, context.sequence, color->getIndex(), depth->getIndex(),
                                color_us, depth_us, context.host_ready_mono_ns,
                                context.capture_mono_ns, context.capture_estimated};
        view.metadata.intrinsics = {intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy, width, height};
        view.metadata.distortion = {true, static_cast<int>(distortion.model),
                                   {distortion.k1, distortion.k2, distortion.k3, distortion.k4,
                                    distortion.k5, distortion.k6, distortion.p1, distortion.p2}};
        // SDK2.9.3 getValueScale(): raw * scale gives millimetres, not metres.
        view.metadata.unit_m = static_cast<float>(depth->getValueScale() * 0.001);
        view.pixels = depth->getData();
        view.bytes = depth->getDataSize();
        result.source = view.metadata.source;
        result.outcome = mailbox.try_offer(view);
        return result;
    } catch (...) {
        result.error = TapError::SdkRead;
        return result;
    }
}
}  // namespace perception::native_depth
