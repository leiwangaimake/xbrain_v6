/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: jpeg_ycc.cpp
 * Brief: libjpeg-equivalent 4:2:2 YCbCr to BGR conversion for the hardware MJPG decode path.
 *
 * Description:
 * Two steps per row, both with libjpeg's integer arithmetic:
 * 1. widen Cb and Cr to full width with the h2v1 triangle filter (3/4 of the nearer sample plus
 *    1/4 of the farther one; the two edge samples are copied), jdsample.c h2v1_fancy_upsample;
 * 2. convert with the fixed-point terms of jdcolor.c build_ycc_rgb_table and clamp to 0..255.
 * On ARM (NEON) both steps run 8 samples at a time with the same integer formulas: the terms
 * are computed in 32-bit lanes, the clamp is the saturating narrow to 0..255, and the result is
 * bit-identical to the table path (test_jpeg_ycc checks this against an independent reference).
 * Edge samples and row tails use the table path. Measured on 1.8 in the service, the scalar
 * version took about 7.7 ms per 1280x800 frame on the capture core.
 */

#include "modules/jpeg_ycc.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#define PERCEPTION_JPEG_YCC_NEON 1
#endif

namespace perception {
namespace {

// jdcolor.c: SCALEBITS 16, ONE_HALF = 1 << 15, FIX(x) = x * 65536 rounded.
constexpr int kScaleBits = 16;
constexpr std::int32_t kHalf = 1 << (kScaleBits - 1);
std::int32_t fix(double v) { return static_cast<std::int32_t>(v * 65536.0 + 0.5); }

struct YccTables {
    std::int32_t fix_cr_r = fix(1.40200), fix_cb_b = fix(1.77200);
    std::int32_t fix_cb_g = fix(0.34414), fix_cr_g = fix(0.71414);
    std::array<int, 256> cr_r{}, cb_b{};
    std::array<std::int32_t, 256> cr_g{}, cb_g{};
    YccTables() {
        for (int i = 0; i < 256; ++i) {
            const std::int32_t x = i - 128;
            cr_r[i] = static_cast<int>((fix_cr_r * x + kHalf) >> kScaleBits);
            cb_b[i] = static_cast<int>((fix_cb_b * x + kHalf) >> kScaleBits);
            cr_g[i] = -fix_cr_g * x;
            cb_g[i] = -fix_cb_g * x + kHalf;
        }
    }
};

const YccTables& tables() {
    static const YccTables t;
    return t;
}

inline std::uint8_t clamp_sample(int v) { return static_cast<std::uint8_t>(std::min(255, std::max(0, v))); }

// jdsample.c h2v1_fancy_upsample for one row of n >= 2 chroma samples into 2 * n outputs.
void widen_row(const std::uint8_t* in, int n, std::uint8_t* out) {
    out[0] = in[0];
    out[1] = static_cast<std::uint8_t>((in[0] * 3 + in[1] + 2) >> 2);
    int i = 1;
#ifdef PERCEPTION_JPEG_YCC_NEON
    // Interior samples i .. i + 7 read in[i - 1 .. i + 8], so i + 8 <= n - 1.
    for (; i + 9 <= n; i += 8) {
        const uint16x8_t nearer = vmull_u8(vld1_u8(in + i), vdup_n_u8(3));
        const uint8x8_t even = vshrn_n_u16(vaddq_u16(vaddw_u8(nearer, vld1_u8(in + i - 1)), vdupq_n_u16(1)), 2);
        const uint8x8_t odd = vshrn_n_u16(vaddq_u16(vaddw_u8(nearer, vld1_u8(in + i + 1)), vdupq_n_u16(2)), 2);
        const uint8x8x2_t pair{{even, odd}};
        vst2_u8(out + 2 * i, pair);
    }
#endif
    for (; i < n - 1; ++i) {
        const int nearer = in[i] * 3;
        out[2 * i] = static_cast<std::uint8_t>((nearer + in[i - 1] + 1) >> 2);
        out[2 * i + 1] = static_cast<std::uint8_t>((nearer + in[i + 1] + 2) >> 2);
    }
    out[2 * n - 2] = static_cast<std::uint8_t>((in[n - 1] * 3 + in[n - 2] + 1) >> 2);
    out[2 * n - 1] = in[n - 1];
}

// Converts pixels [x, width) of one row with the tables.
void convert_tail(const YccTables& t, const std::uint8_t* luma, const std::uint8_t* wide_cb,
                  const std::uint8_t* wide_cr, int x, int width, std::uint8_t* out) {
    for (; x < width; ++x) {
        const int l = luma[x], b = wide_cb[x], c = wide_cr[x];
        out[3 * x] = clamp_sample(l + t.cb_b[b]);
        // Arithmetic shift of a possibly negative sum, as libjpeg's RIGHT_SHIFT (GCC/Clang).
        out[3 * x + 1] = clamp_sample(l + static_cast<int>((t.cb_g[b] + t.cr_g[c]) >> kScaleBits));
        out[3 * x + 2] = clamp_sample(l + t.cr_r[c]);
    }
}

#ifdef PERCEPTION_JPEG_YCC_NEON
// (fix * v + ONE_HALF) >> 16 for 8 signed samples, as 32-bit lanes, narrowed to 16 bits.
inline int16x8_t term(int16x8_t v, std::int32_t fixed) {
    const int32x4_t half = vdupq_n_s32(kHalf);
    const int32x4_t lo = vshrq_n_s32(vmlaq_n_s32(half, vmovl_s16(vget_low_s16(v)), fixed), kScaleBits);
    const int32x4_t hi = vshrq_n_s32(vmlaq_n_s32(half, vmovl_s16(vget_high_s16(v)), fixed), kScaleBits);
    return vcombine_s16(vmovn_s32(lo), vmovn_s32(hi));
}

// The green term (ONE_HALF - FIX(0.34414) b - FIX(0.71414) c) >> 16 for 8 samples.
inline int16x8_t green_term(int16x8_t b, int16x8_t c, std::int32_t fix_b, std::int32_t fix_c) {
    const int32x4_t half = vdupq_n_s32(kHalf);
    int32x4_t lo = vmlaq_n_s32(half, vmovl_s16(vget_low_s16(b)), -fix_b);
    int32x4_t hi = vmlaq_n_s32(half, vmovl_s16(vget_high_s16(b)), -fix_b);
    lo = vmlaq_n_s32(lo, vmovl_s16(vget_low_s16(c)), -fix_c);
    hi = vmlaq_n_s32(hi, vmovl_s16(vget_high_s16(c)), -fix_c);
    return vcombine_s16(vmovn_s32(vshrq_n_s32(lo, kScaleBits)), vmovn_s32(vshrq_n_s32(hi, kScaleBits)));
}
#endif

}  // namespace

void jfif_ycc422_to_bgr(const std::uint8_t* y, int y_stride, const std::uint8_t* cb,
                        const std::uint8_t* cr, int c_stride, int width, int height,
                        std::uint8_t* bgr, int bgr_stride) {
    if (!y || !cb || !cr || !bgr) throw std::invalid_argument("jfif_ycc422_to_bgr: null plane");
    if (width < 4 || width % 2 != 0 || height < 1)
        throw std::invalid_argument("jfif_ycc422_to_bgr: width must be even and >= 4, height >= 1");
    if (y_stride < width || c_stride < width / 2 || bgr_stride < 3 * width)
        throw std::invalid_argument("jfif_ycc422_to_bgr: stride shorter than a row");
    const auto& t = tables();
    thread_local std::vector<std::uint8_t> wide_cb, wide_cr;
    wide_cb.resize(static_cast<std::size_t>(width));
    wide_cr.resize(static_cast<std::size_t>(width));
    for (int r = 0; r < height; ++r) {
        widen_row(cb + static_cast<std::ptrdiff_t>(r) * c_stride, width / 2, wide_cb.data());
        widen_row(cr + static_cast<std::ptrdiff_t>(r) * c_stride, width / 2, wide_cr.data());
        const std::uint8_t* luma = y + static_cast<std::ptrdiff_t>(r) * y_stride;
        std::uint8_t* out = bgr + static_cast<std::ptrdiff_t>(r) * bgr_stride;
        int x = 0;
#ifdef PERCEPTION_JPEG_YCC_NEON
        const int16x8_t centre = vdupq_n_s16(128);
        for (; x + 8 <= width; x += 8) {
            const int16x8_t l = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(luma + x)));
            const int16x8_t b = vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(vld1_u8(wide_cb.data() + x))), centre);
            const int16x8_t c = vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(vld1_u8(wide_cr.data() + x))), centre);
            uint8x8x3_t px;
            px.val[0] = vqmovun_s16(vaddq_s16(l, term(b, t.fix_cb_b)));
            px.val[1] = vqmovun_s16(vaddq_s16(l, green_term(b, c, t.fix_cb_g, t.fix_cr_g)));
            px.val[2] = vqmovun_s16(vaddq_s16(l, term(c, t.fix_cr_r)));
            vst3_u8(out + 3 * x, px);
        }
#endif
        convert_tail(t, luma, wide_cb.data(), wide_cr.data(), x, width, out);
    }
}

}  // namespace perception
