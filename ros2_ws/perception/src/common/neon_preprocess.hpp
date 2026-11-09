/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: neon_preprocess.hpp
 * Brief: Declaration of the NEON letterbox preprocessing (preprocess_neon.inc).
 *
 * Description:
 * Selected by preprocess_impl: neon. For the sizes it supports (an interior linear downscale between
 * 1x and 2x, or exactly 2x) it reproduces the fused path bit for bit with its own resize; every
 * other call is handed to the fused path unchanged.
 * Internal to src/common (preprocess.cpp); the detail function is for the unit test.
 */

#pragma once

#include "common/preprocess.hpp"

#include <string>

namespace yolo {

// Same contract, outputs and exceptions as preprocess_into_fused.
void preprocess_into_neon(const cv::Mat& image, int input_size, LetterboxInfo& info,
                          float* output_data, std::size_t output_count);

// Same contract, outputs and exceptions as preprocess_window_into_fused (window_preprocess.hpp);
// it runs the square input's plan on the window's rows.
void preprocess_window_into_neon(const cv::Mat& image, ModelInputShape shape, int window_top, LetterboxInfo& info,
                                 float* output_data, std::size_t output_count);

namespace neon_preprocess_detail {
// The plan for this source size and model input, built (and self-checked) on first use.
struct PlanStatus {
    bool fast = false;          // true: the NEON path runs; false: every call goes to fused
    std::string reason;         // "ok" or why not (not_aarch64, scale, width, edge, block, selfcheck,
                                // plan_cache_full)
    std::string kind;           // "linear", "area2" (exactly 2x, OpenCV's INTER_AREA shortcut) or "none"
};
PlanStatus plan_status(int source_width, int source_height, int input_size);
}  // namespace neon_preprocess_detail

}  // namespace yolo
