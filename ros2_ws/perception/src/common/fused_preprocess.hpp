/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: fused_preprocess.hpp
 * Brief: Declaration of the fused preprocessing implementation (preprocess_fused.inc).
 *
 * Description:
 * Selected by preprocess_impl: fused, which the production runtime set uses. The yolo_prototype
 * namespace is left from its candidate days. Internal to src/common (preprocess.cpp).
 */

#pragma once

#include "common/preprocess.hpp"

namespace yolo_prototype {

// CV_8UC3 takes the fused path (the production setting, see the file header). Other image types
// delegate to the unmodified legacy implementation, preserving its existing acceptance/rejection
// behavior (not accelerated).
void preprocess_into_fused(const cv::Mat& image, int input_size,
                         yolo::LetterboxInfo& info, float* output_data,
                         std::size_t output_count);

}  // namespace yolo_prototype
