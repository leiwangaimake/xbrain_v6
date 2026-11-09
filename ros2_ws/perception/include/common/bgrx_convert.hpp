/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: bgrx_convert.hpp
 * Brief: BGR (3 bytes) to BGRx (4 bytes, x = 255) row conversion, NEON on aarch64.
 *
 * Description:
 * The MED-2 stream (hevc_encoder) feeds BGRx into its appsrc. This produces the exact bytes
 * GStreamer videoconvert gave for the same frame (tests/claude/test_bgrx_equivalence.cpp compares
 * with a real videoconvert pipeline), replacing that element.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace perception {

inline void convert_bgr_row_to_bgrx(const std::uint8_t* src, std::uint8_t* dst, int width) noexcept {
    int x = 0;
#if defined(__aarch64__)
    const uint8x16_t alpha = vdupq_n_u8(255U);
    for (; x + 16 <= width; x += 16) {
        const uint8x16x3_t bgr = vld3q_u8(src);
        uint8x16x4_t bgrx;
        bgrx.val[0] = bgr.val[0];
        bgrx.val[1] = bgr.val[1];
        bgrx.val[2] = bgr.val[2];
        bgrx.val[3] = alpha;
        vst4q_u8(dst, bgrx);
        src += 48;
        dst += 64;
    }
#endif
    for (; x < width; ++x) {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        dst[3] = 255U;
        src += 3;
        dst += 4;
    }
}

}  // namespace perception
