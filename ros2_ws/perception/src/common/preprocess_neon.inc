/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: preprocess_neon.inc
 * Brief: NEON letterbox preprocessing (preprocess_impl: neon): OpenCV's 8-bit resize (fixed-point
 *        INTER_LINEAR, or its INTER_AREA shortcut at exactly 2x) and the float conversion in one
 *        pass, bit for bit equal to the fused path.
 *
 * Description:
 * What problem this solves. In the fused path the Active12 input (1280 x 800 -> 896 x 560) costs
 * about 5.4 ms of cv::resize and 1.8 ms of conversion per frame on one core of 1.8 (2026-10-06,
 * single thread p50); this file does both in about 3.5 ms (in service par_helper -3.8 ms per
 * frame, n1ab1006). The COCO80 input (-> 640 x 400) takes 1.8 ms, here 0.8 ms. Dense scenes are
 * CPU bound, so every millisecond per frame counts.
 *
 * How it stays identical. cv::resize (OpenCV 4.5.4, resizeGeneric) on 8-bit input:
 *   - per output column d: f = float((d + 0.5) * scale - 0.5), sx = floor(f), f -= sx, weights
 *     a0 = saturate_cast<short>((1 - f) * 2048), a1 = saturate_cast<short>(f * 2048); rows alike;
 *   - horizontal pass per source row, exact in int: h = src[sx] * a0 + src[sx + 1] * a1;
 *   - vertical pass with its SIMD formula: ((((h0 >> 4) * b0) >> 16) + (((h1 >> 4) * b1) >> 16) + 2) >> 2,
 *     saturated to 0..255. The scalar tail of OpenCV uses another rounding, so the fast path only
 *     takes output widths that are a multiple of 16 (3 * width a multiple of 16: no scalar tail).
 * At exactly 2x in both directions cv::resize switches INTER_LINEAR to its INTER_AREA fast path:
 * each output sample is (a + b + c + d + 2) >> 2 of its 2 x 2 source block (the "area2" plan).
 * The NEON code computes exactly these integers, then value * (1 / 255) in float like fused.
 * Linear plans take only interior downscales strictly between 1x and 2x; area2 plans need an
 * output width that is a multiple of 8. Anything else, and any call the fused path would reject,
 * goes to the fused path. Each
 * plan is self-checked on a pseudo-random image against the fused path before its first use, so
 * another OpenCV build that rounds differently falls back instead of changing the model input.
 *
 * Window inputs (preprocess_window_into: a band of rows of the square letterbox, 2026-10-10). The
 * plan and its tables belong to the square input; both row loops take the first row and the row
 * count to write, the whole square being (0, input_size). A window therefore runs the same code
 * on the same tables as the self-checked square plan, and needs no plan of its own.
 *
 * Not a frozen copy: unlike preprocess_legacy.inc and preprocess_fused.inc this is maintained code.
 * It is an .inc so that every target that compiles preprocess.cpp gets it, with that file's
 * -fno-fast-math.
 */

#include "neon_preprocess.hpp"
#include "fused_preprocess.hpp"
#include "window_preprocess.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <opencv2/imgproc.hpp>

namespace yolo {
namespace {

constexpr float kNeonByteScale = static_cast<float>(1.0 / 255.0);
constexpr float kNeonPad = 114.0F * kNeonByteScale;
constexpr int kCoefficientScale = 2048;  // OpenCV INTER_RESIZE_COEF_SCALE
constexpr int kBlock = 16;               // output columns per horizontal block
constexpr int kBlockSpan = 32;           // source bytes one block may read (two NEON registers)
constexpr std::size_t kMaxPlans = 8;     // one per (camera size, model input); more is a misuse

enum class PlanKind { None, Linear, Area2 };

struct NeonPlan {
    int source_width = 0, source_height = 0, input_size = 0;
    LetterboxInfo geometry;
    bool fast = false;
    PlanKind kind = PlanKind::None;
    std::string reason;
    std::vector<int> row;                       // per output row: source rows row and row + 1
    std::vector<std::int16_t> row_w0, row_w1;   // per output row: vertical weights
    std::vector<int> block_base;                // per block of 16 output columns: first source column
    std::vector<std::uint8_t> tap0, tap1;       // per output column: sx - base and sx + 1 - base
    std::vector<std::uint16_t> col_w0, col_w1;  // per output column: horizontal weights
};

// The same expressions as preprocess_fused.inc.
void letterbox_geometry(int img_w, int img_h, int input_size, LetterboxInfo& info) {
    info.original_width = img_w;
    info.original_height = img_h;
    info.scale = std::min(static_cast<float>(input_size) / static_cast<float>(img_w),
                          static_cast<float>(input_size) / static_cast<float>(img_h));
    info.new_width = std::max(1, static_cast<int>(std::round(static_cast<float>(img_w) * info.scale)));
    info.new_height = std::max(1, static_cast<int>(std::round(static_cast<float>(img_h) * info.scale)));
    info.pad_left = (input_size - info.new_width) / 2;
    info.pad_top = (input_size - info.new_height) / 2;
}

// resizeGeneric's INTER_LINEAR table, fixed point; false if a tap would need OpenCV's edge handling.
[[maybe_unused]] bool linear_taps(int source, int output, std::vector<int>& index, std::vector<std::int16_t>& w0,
                 std::vector<std::int16_t>& w1) {
    const double scale = 1.0 / (static_cast<double>(output) / source);
    bool interior = true;
    for (int d = 0; d < output; ++d) {
        float f = static_cast<float>((d + 0.5) * scale - 0.5);
        const int s = cvFloor(f);
        f -= static_cast<float>(s);
        if (s < 0 || s + 1 >= source) interior = false;
        index.push_back(s);
        w0.push_back(cv::saturate_cast<short>((1.0F - f) * kCoefficientScale));
        w1.push_back(cv::saturate_cast<short>(f * kCoefficientScale));
    }
    return interior;
}

#if defined(__aarch64__)
// Per-thread scratch: the deinterleaved source row and the two horizontal rows in use.
struct NeonWork {
    std::vector<std::uint8_t> planes;  // B, G, R planes of one source row, kBlockSpan bytes slack each
    std::vector<std::int16_t> rows;    // two horizontal rows, 3 channel planes of new_width each
    int stride = 0;
};

// Horizontal pass of one source row into (h >> 4) per channel plane, as OpenCV packs it.
void horizontal_row(const std::uint8_t* source, const NeonPlan& p, NeonWork& w, std::int16_t* out) {
    std::uint8_t* const plane_b = w.planes.data();
    std::uint8_t* const plane_g = plane_b + w.stride;
    std::uint8_t* const plane_r = plane_g + w.stride;
    int x = 0;
    for (; x + 16 <= p.source_width; x += 16) {
        const uint8x16x3_t bgr = vld3q_u8(source + 3 * x);
        vst1q_u8(plane_b + x, bgr.val[0]);
        vst1q_u8(plane_g + x, bgr.val[1]);
        vst1q_u8(plane_r + x, bgr.val[2]);
    }
    for (; x < p.source_width; ++x) {
        plane_b[x] = source[3 * x];
        plane_g[x] = source[3 * x + 1];
        plane_r[x] = source[3 * x + 2];
    }
    const int width = p.geometry.new_width;
    for (int c = 0; c < 3; ++c) {
        const std::uint8_t* const plane = plane_b + c * w.stride;
        std::int16_t* const h = out + c * width;
        for (int b = 0; b * kBlock < width; ++b) {
            const int d = b * kBlock;
            const std::uint8_t* const s = plane + p.block_base[static_cast<std::size_t>(b)];
            const uint8x16x2_t span = {{vld1q_u8(s), vld1q_u8(s + 16)}};
            const uint8x16_t left = vqtbl2q_u8(span, vld1q_u8(&p.tap0[static_cast<std::size_t>(d)]));
            const uint8x16_t right = vqtbl2q_u8(span, vld1q_u8(&p.tap1[static_cast<std::size_t>(d)]));
            const uint16x8_t a0_lo = vld1q_u16(&p.col_w0[static_cast<std::size_t>(d)]);
            const uint16x8_t a0_hi = vld1q_u16(&p.col_w0[static_cast<std::size_t>(d) + 8]);
            const uint16x8_t a1_lo = vld1q_u16(&p.col_w1[static_cast<std::size_t>(d)]);
            const uint16x8_t a1_hi = vld1q_u16(&p.col_w1[static_cast<std::size_t>(d) + 8]);
            const uint16x8_t l_lo = vmovl_u8(vget_low_u8(left)), l_hi = vmovl_high_u8(left);
            const uint16x8_t r_lo = vmovl_u8(vget_low_u8(right)), r_hi = vmovl_high_u8(right);
            // At most 255 * 2049 >> 4 = 32655: fits int16 like OpenCV's saturating pack.
            const uint32x4_t s0 = vmlal_u16(vmull_u16(vget_low_u16(l_lo), vget_low_u16(a0_lo)), vget_low_u16(r_lo), vget_low_u16(a1_lo));
            const uint32x4_t s1 = vmlal_high_u16(vmull_high_u16(l_lo, a0_lo), r_lo, a1_lo);
            const uint32x4_t s2 = vmlal_u16(vmull_u16(vget_low_u16(l_hi), vget_low_u16(a0_hi)), vget_low_u16(r_hi), vget_low_u16(a1_hi));
            const uint32x4_t s3 = vmlal_high_u16(vmull_high_u16(l_hi, a0_hi), r_hi, a1_hi);
            vst1q_s16(h + d, vreinterpretq_s16_u16(vcombine_u16(vshrn_n_u32(s0, 4), vshrn_n_u32(s1, 4))));
            vst1q_s16(h + d + 8, vreinterpretq_s16_u16(vcombine_u16(vshrn_n_u32(s2, 4), vshrn_n_u32(s3, 4))));
        }
    }
}

inline void store_floats(float* destination, uint8x8_t values, float32x4_t scale) {
    const uint16x8_t wide = vmovl_u8(values);
    vst1q_f32(destination, vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_low_u16(wide))), scale));
    vst1q_f32(destination + 4, vmulq_f32(vcvtq_f32_u32(vmovl_high_u16(wide)), scale));
}

// Rows [row_begin, row_begin + row_count) of the letterbox tensor (the whole tensor: 0 and
// input_size): pad rows, and per resized row the vertical pass straight into floats.
void run_neon(const cv::Mat& image, const NeonPlan& p, float* output, int row_begin, int row_count) {
    thread_local NeonWork work;
    const int width = p.geometry.new_width;
    work.stride = p.source_width + kBlockSpan;
    if (work.planes.size() < static_cast<std::size_t>(3 * work.stride)) work.planes.assign(static_cast<std::size_t>(3 * work.stride), 0);
    if (work.rows.size() < static_cast<std::size_t>(6 * width)) work.rows.assign(static_cast<std::size_t>(6 * width), 0);
    const int input = p.input_size;
    const std::size_t plane = static_cast<std::size_t>(input) * static_cast<std::size_t>(row_count);
    const float32x4_t scale = vdupq_n_f32(kNeonByteScale);
    std::int16_t* row_a = work.rows.data();
    std::int16_t* row_b = row_a + 3 * width;
    int source_a = -1, source_b = -1;
    for (int y = row_begin; y < row_begin + row_count; ++y) {
        // Source channel c (B, G, R) goes to output plane 2 - c (R, G, B first).
        float* const red = output + static_cast<std::size_t>(y - row_begin) * static_cast<std::size_t>(input);
        float* const planes[3] = {red + 2 * plane, red + plane, red};
        const int dy = y - p.geometry.pad_top;
        if (dy < 0 || dy >= p.geometry.new_height) {
            for (float* o : planes) std::fill_n(o, input, kNeonPad);
            continue;
        }
        const int r0 = p.row[static_cast<std::size_t>(dy)];
        if (source_b == r0) { std::swap(row_a, row_b); std::swap(source_a, source_b); }
        if (source_a != r0) { horizontal_row(image.ptr<std::uint8_t>(r0), p, work, row_a); source_a = r0; }
        if (source_b != r0 + 1) { horizontal_row(image.ptr<std::uint8_t>(r0 + 1), p, work, row_b); source_b = r0 + 1; }
        const int16x8_t b0 = vdupq_n_s16(p.row_w0[static_cast<std::size_t>(dy)]);
        const int16x8_t b1 = vdupq_n_s16(p.row_w1[static_cast<std::size_t>(dy)]);
        for (int c = 0; c < 3; ++c) {
            float* const o = planes[c];
            std::fill_n(o, p.geometry.pad_left, kNeonPad);
            const std::int16_t* const h0 = row_a + c * width;
            const std::int16_t* const h1 = row_b + c * width;
            for (int x = 0; x < width; x += 8) {
                const int16x8_t v0 = vld1q_s16(h0 + x), v1 = vld1q_s16(h1 + x);
                // OpenCV v_mul_hi: (a * b) >> 16; saturating add; v_rshr_pack_u<2>: (v + 2) >> 2 to 0..255.
                const int16x8_t t0 = vcombine_s16(vshrn_n_s32(vmull_s16(vget_low_s16(v0), vget_low_s16(b0)), 16),
                                                  vshrn_n_s32(vmull_high_s16(v0, b0), 16));
                const int16x8_t t1 = vcombine_s16(vshrn_n_s32(vmull_s16(vget_low_s16(v1), vget_low_s16(b1)), 16),
                                                  vshrn_n_s32(vmull_high_s16(v1, b1), 16));
                store_floats(o + p.geometry.pad_left + x, vqrshrun_n_s16(vqaddq_s16(t0, t1), 2), scale);
            }
            std::fill(o + p.geometry.pad_left + width, o + input, kNeonPad);
        }
    }
}

// OpenCV's INTER_AREA fast path at exactly 2x: (a + b + c + d + 2) >> 2 per channel, 8 outputs
// (16 source pixels of two rows) at a time, written straight as floats.
void run_area2(const cv::Mat& image, const NeonPlan& p, float* output, int row_begin, int row_count) {
    const int input = p.input_size, width = p.geometry.new_width;
    const std::size_t plane = static_cast<std::size_t>(input) * static_cast<std::size_t>(row_count);
    const float32x4_t scale = vdupq_n_f32(kNeonByteScale);
    for (int y = row_begin; y < row_begin + row_count; ++y) {
        float* const red = output + static_cast<std::size_t>(y - row_begin) * static_cast<std::size_t>(input);
        float* const planes[3] = {red + 2 * plane, red + plane, red};
        const int dy = y - p.geometry.pad_top;
        if (dy < 0 || dy >= p.geometry.new_height) {
            for (float* o : planes) std::fill_n(o, input, kNeonPad);
            continue;
        }
        for (float* o : planes) std::fill_n(o, p.geometry.pad_left, kNeonPad);
        const std::uint8_t* const top = image.ptr<std::uint8_t>(2 * dy);
        const std::uint8_t* const bottom = image.ptr<std::uint8_t>(2 * dy + 1);
        for (int x = 0; x < width; x += 8) {
            const uint8x16x3_t a = vld3q_u8(top + 6 * x);
            const uint8x16x3_t b = vld3q_u8(bottom + 6 * x);
            for (int c = 0; c < 3; ++c) {
                const uint16x8_t sum = vaddq_u16(vpaddlq_u8(a.val[c]), vpaddlq_u8(b.val[c]));
                store_floats(planes[c] + p.geometry.pad_left + x, vrshrn_n_u16(sum, 2), scale);
            }
        }
        for (float* o : planes) std::fill(o + p.geometry.pad_left + width, o + input, kNeonPad);
    }
}

void run_plan(const cv::Mat& image, const NeonPlan& p, float* output, int row_begin, int row_count) {
    if (p.kind == PlanKind::Area2) {
        run_area2(image, p, output, row_begin, row_count);
    } else {
        run_neon(image, p, output, row_begin, row_count);
    }
}

// Pseudo-random image of the source size for the self-check (every value and neighbour mix occurs).
cv::Mat selfcheck_image(int width, int height) {
    cv::Mat image(height, width, CV_8UC3);
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;
    for (int y = 0; y < height; ++y) {
        std::uint8_t* row = image.ptr<std::uint8_t>(y);
        for (int x = 0; x < 3 * width; ++x) {
            state ^= state << 13U;
            state ^= state >> 7U;
            state ^= state << 17U;
            row[x] = static_cast<std::uint8_t>(state >> 32U);
        }
    }
    return image;
}
#endif

#if defined(__aarch64__)
// Self-check against the fused path (cv::resize) before the plan is used for a real frame.
std::shared_ptr<const NeonPlan> self_checked(std::shared_ptr<NeonPlan> p) {
    const cv::Mat probe = selfcheck_image(p->source_width, p->source_height);
    const std::size_t count = 3U * static_cast<std::size_t>(p->input_size) * static_cast<std::size_t>(p->input_size);
    std::vector<float> reference(count), candidate(count, -1.0F);
    LetterboxInfo reference_info;
    yolo_prototype::preprocess_into_fused(probe, p->input_size, reference_info, reference.data(), count);
    run_plan(probe, *p, candidate.data(), 0, p->input_size);
    if (std::memcmp(reference.data(), candidate.data(), count * sizeof(float)) != 0) {
        p->reason = "selfcheck";
        return p;
    }
    p->fast = true;
    p->reason = "ok";
    return p;
}
#endif

std::shared_ptr<const NeonPlan> build_plan(int source_width, int source_height, int input_size) {
    auto p = std::make_shared<NeonPlan>();
    p->source_width = source_width;
    p->source_height = source_height;
    p->input_size = input_size;
    letterbox_geometry(source_width, source_height, input_size, p->geometry);
#if !defined(__aarch64__)
    p->reason = "not_aarch64";
    return p;
#else
    const int width = p->geometry.new_width, height = p->geometry.new_height;
    const double scale_x = 1.0 / (static_cast<double>(width) / source_width);
    const double scale_y = 1.0 / (static_cast<double>(height) / source_height);
    if (2 * width == source_width && 2 * height == source_height) {
        // Exactly 2x: OpenCV's INTER_AREA shortcut.
        if (width % 8 != 0) { p->reason = "width"; return p; }
        p->kind = PlanKind::Area2;
        return self_checked(std::move(p));
    }
    if (!(scale_x > 1.0 && scale_x < 2.0 && scale_y > 1.0 && scale_y < 2.0)) { p->reason = "scale"; return p; }
    if (width % kBlock != 0) { p->reason = "width"; return p; }
    std::vector<int> column;
    std::vector<std::int16_t> w0, w1;
    if (!linear_taps(source_width, width, column, w0, w1) ||
        !linear_taps(source_height, height, p->row, p->row_w0, p->row_w1)) {
        p->reason = "edge";
        return p;
    }
    for (int b = 0; b * kBlock < width; ++b) {
        const int base = column[static_cast<std::size_t>(b * kBlock)];
        p->block_base.push_back(base);
        for (int j = 0; j < kBlock; ++j) {
            const std::size_t d = static_cast<std::size_t>(b * kBlock + j);
            const int offset = column[d] - base;
            // The block loads kBlockSpan bytes from base; the plane buffer has that much slack.
            if (offset < 0 || offset + 1 >= kBlockSpan) {
                p->reason = "block";
                return p;
            }
            p->tap0.push_back(static_cast<std::uint8_t>(offset));
            p->tap1.push_back(static_cast<std::uint8_t>(offset + 1));
            p->col_w0.push_back(static_cast<std::uint16_t>(w0[d]));
            p->col_w1.push_back(static_cast<std::uint16_t>(w1[d]));
        }
    }
    p->kind = PlanKind::Linear;
    return self_checked(std::move(p));
#endif
}

const char* plan_kind_name(PlanKind kind) {
    switch (kind) {
        case PlanKind::Linear: return "linear";
        case PlanKind::Area2: return "area2";
        case PlanKind::None: break;
    }
    return "none";
}

// Plans are built once per (source size, model input) under the lock, then shared read-only.
std::shared_ptr<const NeonPlan> plan_for(int source_width, int source_height, int input_size) {
    static std::mutex mutex;
    static std::vector<std::shared_ptr<const NeonPlan>> plans;
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& p : plans) {
        if (p->source_width == source_width && p->source_height == source_height && p->input_size == input_size) return p;
    }
    if (plans.size() >= kMaxPlans) {
        auto p = std::make_shared<NeonPlan>();
        p->source_width = source_width;
        p->source_height = source_height;
        p->input_size = input_size;
        p->reason = "plan_cache_full";
        return p;
    }
    const auto begin = std::chrono::steady_clock::now();
    auto p = build_plan(source_width, source_height, input_size);
    const auto build_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
    std::fprintf(stderr, "PREPROCESS_NEON_PLAN schema=2 source=%dx%d input=%d resized=%dx%d fast=%d kind=%s reason=%s build_us=%lld\n",
                 source_width, source_height, input_size, p->geometry.new_width, p->geometry.new_height,
                 static_cast<int>(p->fast), plan_kind_name(p->kind), p->reason.c_str(), static_cast<long long>(build_us));
    plans.push_back(p);
    return p;
}

}  // namespace

void preprocess_into_neon(const cv::Mat& image, int input_size, LetterboxInfo& info,
                          float* output_data, std::size_t output_count) {
    // Whatever the fused path would delegate or reject goes there unchanged (same exceptions).
    const std::size_t expected = input_size > 0
        ? 3U * static_cast<std::size_t>(input_size) * static_cast<std::size_t>(input_size) : 0U;
    if (image.type() != CV_8UC3 || image.empty() || input_size <= 0 || output_data == nullptr ||
        output_count != expected) {
        yolo_prototype::preprocess_into_fused(image, input_size, info, output_data, output_count);
        return;
    }
    const auto plan = plan_for(image.cols, image.rows, input_size);
    if (!plan->fast) {
        yolo_prototype::preprocess_into_fused(image, input_size, info, output_data, output_count);
        return;
    }
    info = plan->geometry;
#if defined(__aarch64__)
    run_plan(image, *plan, output_data, 0, input_size);
#endif
}

void preprocess_window_into_neon(const cv::Mat& image, ModelInputShape shape, int window_top, LetterboxInfo& info,
                                 float* output_data, std::size_t output_count) {
    // Whatever the one-pass window version would delegate or reject goes there unchanged (same
    // exceptions): other image types, a bad window or buffer, content the window would cut.
    const bool window_valid = shape.width > 0 && shape.height > 0 && window_top >= 0 &&
                              window_top + shape.height <= shape.width;
    const std::size_t expected = window_valid
        ? 3U * static_cast<std::size_t>(shape.width) * static_cast<std::size_t>(shape.height) : 0U;
    if (image.type() != CV_8UC3 || image.empty() || !window_valid || output_data == nullptr ||
        output_count != expected) {
        preprocess_window_into_fused(image, shape, window_top, info, output_data, output_count);
        return;
    }
    // Checked before a plan is looked up, so a source the window cannot hold does not take one of
    // the plan slots.
    LetterboxInfo square;
    letterbox_geometry(image.cols, image.rows, shape.width, square);
    if (square.pad_top < window_top || square.pad_top + square.new_height > window_top + shape.height) {
        preprocess_window_into_fused(image, shape, window_top, info, output_data, output_count);
        return;
    }
    // The square input's plan: its tables do not depend on which rows are written.
    const auto plan = plan_for(image.cols, image.rows, shape.width);
    if (!plan->fast) {
        preprocess_window_into_fused(image, shape, window_top, info, output_data, output_count);
        return;
    }
    info = plan->geometry;
    info.pad_top -= window_top;
#if defined(__aarch64__)
    run_plan(image, *plan, output_data, window_top, shape.height);
#endif
}

namespace neon_preprocess_detail {
PlanStatus plan_status(int source_width, int source_height, int input_size) {
    const auto plan = plan_for(source_width, source_height, input_size);
    return PlanStatus{plan->fast, plan->reason, plan_kind_name(plan->kind)};
}
}  // namespace neon_preprocess_detail

}  // namespace yolo
