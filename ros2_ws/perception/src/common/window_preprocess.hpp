/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: window_preprocess.hpp
 * Brief: Declarations of the window preprocessing implementations (preprocess_window.inc).
 *
 * Description:
 * A window input is rows [window_top, window_top + height) of the width x width letterbox tensor
 * (preprocess.hpp, preprocess_window_into). Two implementations with the same contract: the
 * reference runs the frozen legacy square path and copies the rows; the one-pass version writes
 * only the window. Internal to src/common (preprocess.cpp); the unit tests call them directly.
 */

#pragma once

#include "common/preprocess.hpp"

namespace yolo {

// By definition: the legacy square tensor's rows. Slow (the whole square is produced first).
void preprocess_window_into_reference(const cv::Mat& image, ModelInputShape shape, int window_top,
                                      LetterboxInfo& info, float* output_data, std::size_t output_count);

// One resize, each output element written once; CV_8UC3 only, other image types go to the
// reference (as the fused square path hands them to legacy). Bit for bit equal to the reference.
void preprocess_window_into_fused(const cv::Mat& image, ModelInputShape shape, int window_top,
                                  LetterboxInfo& info, float* output_data, std::size_t output_count);

}  // namespace yolo
