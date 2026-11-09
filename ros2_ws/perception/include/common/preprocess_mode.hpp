/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: preprocess_mode.hpp
 * Brief: Selection of the preprocessing implementation (PERCEPTION_PREPROCESS_IMPL).
 *
 * Description:
 * Three implementations of the same letterbox preprocessing (preprocess.hpp):
 *   legacy  the reference (OpenCV resize, cvtColor, pad, convertTo, split); also used when unset;
 *   fused   one pass that writes every output element once; the production runtime set uses it;
 *   neon    fused, except that sizes it supports take an own NEON fixed-point resize with the
 *           conversion in the same pass (preprocess_neon.inc, 2026-10-06); same bits as fused;
 *   shadow  diagnostic: runs both, compares every float bit and the letterbox metadata, returns
 *           legacy, and throws on any difference (the frame is then not submitted).
 * fused must equal legacy bit for bit; tests/test_fused_preprocess.cpp checks it, and
 * tests/test_neon_preprocess.cpp checks neon against fused and legacy.
 */

#pragma once
#include "common/preprocess.hpp"
#include <cstdint>
#include <limits>
namespace yolo {
enum class PreprocessMode { Legacy, Fused, Shadow, Neon };
// Exact values only. Unset means legacy; explicitly empty/unknown is invalid.
PreprocessMode parse_preprocess_mode(const char* value);
const char* preprocess_mode_name(PreprocessMode mode) noexcept;
// First call freezes success OR error. Integration invokes this before hardware.
PreprocessMode initialize_preprocess_mode_from_environment();
namespace preprocess_detail {
struct Comparison {
    std::size_t elements = 0, different_elements = 0;
    std::size_t first_difference = std::numeric_limits<std::size_t>::max();
    std::uint32_t old_bits = 0, new_bits = 0;
    bool metadata_equal = false;
};
// Diagnostic comparator only; old/new conversion never share this arithmetic.
Comparison compare(const float* reference, const float* candidate, std::size_t count,
                   const LetterboxInfo& old_info, const LetterboxInfo& new_info);
void require_equal(const Comparison& comparison);
}  // namespace preprocess_detail
}  // namespace yolo
