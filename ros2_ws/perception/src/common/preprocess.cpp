/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: preprocess.cpp
 * Brief: Preprocessing dispatcher: legacy, fused (production) or shadow, per preprocess_mode.hpp.
 *
 * Description:
 * What problem this solves. Implements preprocess.hpp. The mode is read once from
 * PERCEPTION_PREPROCESS_IMPL (the service sets it from the runtime section) and frozen, an invalid
 * value included: every later call then throws instead of silently falling back.
 *
 * Why the two .inc files look different from the rest of the code. preprocess_legacy.inc and
 * preprocess_fused.inc are frozen copies of the implementations that were proven equal for the
 * cand_R set, kept byte-identical as that evidence. They therefore keep their original camelCase
 * names (mapped below by macros) and their original file headers. preprocess_neon.inc is not a
 * frozen copy; it is included here for the same build reasons (see its header), and so is
 * preprocess_window.inc (model inputs that are a window of the square letterbox).
 */

#include "common/preprocess.hpp"
#include "common/preprocess_mode.hpp"
#include "common/execution_cpu_trace.hpp"
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/imgproc.hpp>

// Frozen implementation includes remain byte-identical. Token substitution
// only renames old symbols and points the fused non-8U fallback to the oracle.
// Geometry/resize/conversion bodies are neither rewritten nor shared. The .inc files keep
// their original camelCase names; these macros map them onto the snake_case declarations.
#define preprocessInto preprocess_into_legacy
#define preprocess preprocess_legacy
#include "preprocess_legacy.inc"
#undef preprocess
#define preprocessIntoFused preprocess_into_fused
#include "preprocess_fused.inc"
#undef preprocessIntoFused
#undef preprocessInto
#include "preprocess_neon.inc"
#include "preprocess_window.inc"

namespace yolo {
namespace {
std::uint32_t float_bits(float value) {
    std::uint32_t bits; std::memcpy(&bits, &value, sizeof(bits)); return bits;
}
// The mode as read once per process; an invalid value is frozen too, as an error.
struct FrozenMode {
    PreprocessMode mode = PreprocessMode::Legacy;
    bool valid = false;
    FrozenMode() {
        const char* raw = std::getenv("PERCEPTION_PREPROCESS_IMPL");
        const bool present = raw != nullptr;
        const std::string owned = present ? raw : "";
        try { mode = parse_preprocess_mode(present ? owned.c_str() : nullptr); valid = true; }
        catch (const std::invalid_argument&) { /* Invalid selection is frozen too. */ }
    }
};
std::atomic<std::uint64_t> shadow_sequence{0};
// FNV-1a over the input pixels: shadow mode proves neither implementation wrote into its input.
std::uint64_t image_hash(const cv::Mat& image) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (int y = 0; y < image.rows; ++y) {
        const auto* row = image.ptr<unsigned char>(y);
        for (std::size_t x = 0; x < static_cast<std::size_t>(image.cols) * image.elemSize(); ++x) {
            hash = (hash ^ row[x]) * 1099511628211ULL;
        }
    }
    return hash;
}
struct ShadowTrace {
    std::uint64_t call = 0, input_hash_before = 0, input_hash_after = 0;
    perception::ExecutionCpuStamp begin, compare_begin, compare_end, end;
};
void shadow_log(const ShadowTrace& trace, const cv::Mat& image, int input_size,
               const preprocess_detail::Comparison& comparison, bool completed,
               const char* result) {
    const bool clock_valid = trace.begin.valid && trace.end.valid &&
        trace.begin.tid == trace.end.tid && trace.end.wall_ns >= trace.begin.wall_ns;
    const bool compare_clock_valid = trace.compare_begin.valid && trace.compare_end.valid &&
        trace.compare_begin.tid == trace.compare_end.tid &&
        trace.compare_end.wall_ns >= trace.compare_begin.wall_ns;
    std::ostringstream line;
    line << "PREPROCESS_SHADOW schema=1 call=" << trace.call << " tid=" << trace.begin.tid
         << " clock_valid=" << static_cast<int>(clock_valid)
         << " call_begin_ns=" << trace.begin.wall_ns << " call_end_ns=" << trace.end.wall_ns
         << " compare_clock_valid=" << static_cast<int>(compare_clock_valid)
         << " compare_begin_ns=" << trace.compare_begin.wall_ns
         << " compare_end_ns=" << trace.compare_end.wall_ns
         << " input_size=" << input_size << " original_width=" << image.cols
         << " original_height=" << image.rows << " input_type=" << image.type()
         << " input_hash_fnv1a64=" << trace.input_hash_before
         << " input_hash_after_fnv1a64=" << trace.input_hash_after
         << " elements=" << comparison.elements
         << " comparison_complete=" << static_cast<int>(completed)
         << " different_elements=" << comparison.different_elements
         << " metadata_equal=" << static_cast<int>(comparison.metadata_equal)
         << " first_difference=";
    if (comparison.first_difference == std::numeric_limits<std::size_t>::max()) { line << -1; }
    else { line << comparison.first_difference; }
    line << " old_bits=" << comparison.old_bits << " new_bits=" << comparison.new_bits
         << " result=" << result << " return_source=legacy";
    const auto text = line.str();
    // One stdio call serializes lines across both prepare lanes on stderr.
    if (std::fprintf(stderr, "%s\n", text.c_str()) < 0) {
        throw std::runtime_error("preprocess shadow audit write failed");
    }
}
}  // namespace

PreprocessMode parse_preprocess_mode(const char* value) {
    if (value == nullptr || std::strcmp(value, "legacy") == 0) { return PreprocessMode::Legacy; }
    if (std::strcmp(value, "fused") == 0) { return PreprocessMode::Fused; }
    if (std::strcmp(value, "shadow") == 0) { return PreprocessMode::Shadow; }
    if (std::strcmp(value, "neon") == 0) { return PreprocessMode::Neon; }
    throw std::invalid_argument("PERCEPTION_PREPROCESS_IMPL must be unset, legacy, fused, shadow or neon");
}
const char* preprocess_mode_name(PreprocessMode mode) noexcept {
    switch (mode) {
        case PreprocessMode::Legacy: return "legacy";
        case PreprocessMode::Fused: return "fused";
        case PreprocessMode::Shadow: return "shadow";
        case PreprocessMode::Neon: return "neon";
    }
    return "invalid";
}
PreprocessMode initialize_preprocess_mode_from_environment() {
    static const FrozenMode frozen;
    if (!frozen.valid) {
        throw std::invalid_argument("PERCEPTION_PREPROCESS_IMPL frozen invalid; restart with a valid value");
    }
    return frozen.mode;
}
namespace preprocess_detail {
Comparison compare(const float* reference, const float* candidate, std::size_t count,
                   const LetterboxInfo& a, const LetterboxInfo& b) {
    if ((reference == nullptr || candidate == nullptr) && count != 0U) {
        throw std::invalid_argument("shadow comparison buffer must not be null");
    }
    Comparison result; result.elements = count;
    result.metadata_equal = float_bits(a.scale) == float_bits(b.scale) && a.pad_left == b.pad_left &&
        a.pad_top == b.pad_top && a.new_width == b.new_width && a.new_height == b.new_height &&
        a.original_width == b.original_width && a.original_height == b.original_height;
    for (std::size_t i = 0; i < count; ++i) {
        const auto old_bits = float_bits(reference[i]), new_bits = float_bits(candidate[i]);
        if (old_bits != new_bits) {
            if (result.different_elements == 0U) {
                result.first_difference = i; result.old_bits = old_bits; result.new_bits = new_bits;
            }
            ++result.different_elements;
        }
    }
    return result;
}
void require_equal(const Comparison& comparison) {
    if (comparison.different_elements != 0U || !comparison.metadata_equal) {
        throw std::runtime_error("preprocess shadow mismatch; input must not be submitted");
    }
}
}  // namespace preprocess_detail

// Dispatch on the frozen mode. Shadow fills the caller's buffer with legacy, runs fused into a
// scratch buffer, compares, logs one PREPROCESS_SHADOW line and throws on any difference.
void preprocess_into(const cv::Mat& image, int input_size, LetterboxInfo& info,
                    float* output_data, std::size_t output_count) {
    const auto mode = initialize_preprocess_mode_from_environment();
    if (mode == PreprocessMode::Legacy) {
        preprocess_into_legacy(image, input_size, info, output_data, output_count); return;
    }
    if (mode == PreprocessMode::Fused) {
        yolo_prototype::preprocess_into_fused(image, input_size, info, output_data, output_count); return;
    }
    if (mode == PreprocessMode::Neon) {
        preprocess_into_neon(image, input_size, info, output_data, output_count); return;
    }
    ShadowTrace trace;
    trace.call = shadow_sequence.fetch_add(1, std::memory_order_relaxed) + 1U;
    trace.begin = perception::execution_cpu_stamp();
    preprocess_detail::Comparison comparison;
    bool comparison_complete = false;
    try {
        // Old validation precedes candidate allocation; old caller output and
        // metadata remain authoritative. Never return after comparison failure.
        trace.input_hash_before = image_hash(image);
        preprocess_into_legacy(image, input_size, info, output_data, output_count);
        std::vector<float> candidate(output_count);
        LetterboxInfo candidate_info;
        yolo_prototype::preprocess_into_fused(image, input_size, candidate_info,
                                         candidate.data(), candidate.size());
        trace.compare_begin = perception::execution_cpu_stamp();
        comparison = preprocess_detail::compare(output_data, candidate.data(), output_count,
                                               info, candidate_info);
        trace.compare_end = perception::execution_cpu_stamp();
        comparison_complete = true;
        trace.input_hash_after = image_hash(image);
        preprocess_detail::require_equal(comparison);
        if (trace.input_hash_before != trace.input_hash_after) {
            throw std::runtime_error("preprocess shadow source pixels changed");
        }
    } catch (...) {
        trace.end = perception::execution_cpu_stamp();
        shadow_log(trace, image, input_size, comparison, comparison_complete,
                  comparison_complete ? "mismatch" : "exception");
        throw;
    }
    trace.end = perception::execution_cpu_stamp();
    shadow_log(trace, image, input_size, comparison, true, "equal");
}
// The window form of the dispatcher. Legacy is the reference (the legacy square tensor's rows);
// fused is the one-pass version and neon its NEON form; shadow fills the caller's buffer with the reference,
// runs the one-pass version into a scratch buffer, logs one PREPROCESS_SHADOW_WINDOW line and
// throws on any difference.
void preprocess_window_into(const cv::Mat& image, ModelInputShape shape, int window_top, LetterboxInfo& info,
                           float* output_data, std::size_t output_count) {
    if (window_top == 0 && shape.width == shape.height) {
        preprocess_into(image, shape.width, info, output_data, output_count);
        return;
    }
    const auto mode = initialize_preprocess_mode_from_environment();
    if (mode == PreprocessMode::Legacy) {
        preprocess_window_into_reference(image, shape, window_top, info, output_data, output_count); return;
    }
    if (mode == PreprocessMode::Fused) {
        preprocess_window_into_fused(image, shape, window_top, info, output_data, output_count); return;
    }
    if (mode == PreprocessMode::Neon) {
        preprocess_window_into_neon(image, shape, window_top, info, output_data, output_count); return;
    }
    const std::uint64_t call = shadow_sequence.fetch_add(1, std::memory_order_relaxed) + 1U;
    preprocess_detail::Comparison comparison;
    const char* result = "exception";
    try {
        preprocess_window_into_reference(image, shape, window_top, info, output_data, output_count);
        std::vector<float> candidate(output_count);
        LetterboxInfo candidate_info;
        preprocess_window_into_fused(image, shape, window_top, candidate_info, candidate.data(), candidate.size());
        comparison = preprocess_detail::compare(output_data, candidate.data(), output_count, info, candidate_info);
        result = "mismatch";
        preprocess_detail::require_equal(comparison);
        result = "equal";
    } catch (...) {
        std::fprintf(stderr, "PREPROCESS_SHADOW_WINDOW schema=1 call=%llu input=%dx%d window_top=%d original=%dx%d "
                     "elements=%zu different_elements=%zu metadata_equal=%d result=%s return_source=legacy\n",
                     static_cast<unsigned long long>(call), shape.width, shape.height, window_top, image.cols, image.rows,
                     comparison.elements, comparison.different_elements, static_cast<int>(comparison.metadata_equal), result);
        throw;
    }
    if (std::fprintf(stderr, "PREPROCESS_SHADOW_WINDOW schema=1 call=%llu input=%dx%d window_top=%d original=%dx%d "
                     "elements=%zu different_elements=%zu metadata_equal=%d result=%s return_source=legacy\n",
                     static_cast<unsigned long long>(call), shape.width, shape.height, window_top, image.cols, image.rows,
                     comparison.elements, comparison.different_elements, static_cast<int>(comparison.metadata_equal),
                     result) < 0) {
        throw std::runtime_error("preprocess shadow audit write failed");
    }
}
std::vector<float> preprocess(const cv::Mat& image, int input_size, LetterboxInfo& info) {
    if (input_size <= 0) {
        throw std::invalid_argument("preprocess input_size must be positive");
    }
    const std::size_t plane_size =
        static_cast<std::size_t>(input_size) * static_cast<std::size_t>(input_size);
    std::vector<float> output(3U * plane_size);
    preprocess_into(image, input_size, info, output.data(), output.size());
    return output;
}
}  // namespace yolo
