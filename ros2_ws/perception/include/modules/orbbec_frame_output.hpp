/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_frame_output.hpp
 * Brief: Owned output from borrowed Gemini color/depth frame views.
 *
 * Description:
 * The capture wraps SDK frame memory in cv::Mat views; this helper turns a view into an
 * independent image before the SDK frame is released. In R6 colour and output sizes are equal,
 * so it is one copy; depth is copied only when no registration is active.
 */
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace perception::orbbec_capture_detail {

// SDK frame views do not retain their backing ob::Frame. Always return an
// independent allocation before capture releases/reuses the SDK frames. A
// same-size output needs only a copy; actual scaling keeps the caller's
// established interpolation and pixel-centre convention.
inline cv::Mat copy_or_resize_owned(
    const cv::Mat& source, const cv::Size& output_size, int interpolation) {
    if (source.size() == output_size) {
        return source.clone();
    }
    cv::Mat output;
    cv::resize(source, output, output_size, 0.0, 0.0, interpolation);
    return output;
}

}  // namespace perception::orbbec_capture_detail
