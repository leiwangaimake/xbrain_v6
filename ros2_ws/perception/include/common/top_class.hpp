/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: top_class.hpp
 * Brief: Per-anchor top class of a YOLO score block, one contiguous pass per class.
 *
 * Description:
 * The COCO80 decoder picks, for each of 8400 anchors, the class with the highest score. The
 * scores are stored class-major ([classes][anchors]), so scanning the classes of one anchor
 * jumps num_anchors floats per class: 80 interleaved streams that the prefetcher cannot follow
 * (about 1 ms per frame on 1.8). top_class_per_anchor walks each class row once instead and
 * keeps a running maximum per anchor. It is exactly the anchor-major scan: every anchor sees
 * its classes in order 0..classes-1 with the same strict '>' (the first maximum wins, a NaN
 * never replaces a score and a NaN first score is never replaced, +0 and -0 do not replace
 * each other). On ARM the comparison and the two selects run four anchors at a time with NEON
 * (vcgtq_f32 is the same IEEE ordered '>'); elsewhere, and for the tail, the scalar loop runs.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace yolo {

// scores: `classes` rows of `anchors` floats. top_class/top_score: `anchors` outputs.
inline void top_class_per_anchor(const float* scores, std::int64_t anchors, int classes, int* top_class,
                                 float* top_score) {
    if (anchors <= 0 || classes <= 0) return;
    std::memcpy(top_score, scores, static_cast<std::size_t>(anchors) * sizeof(float));
    std::fill(top_class, top_class + anchors, 0);
    for (int k = 1; k < classes; ++k) {
        const float* row = scores + static_cast<std::int64_t>(k) * anchors;
        std::int64_t i = 0;
#if defined(__ARM_NEON)
        const int32x4_t class_k = vdupq_n_s32(k);
        for (; i + 4 <= anchors; i += 4) {
            const float32x4_t score = vld1q_f32(row + i);
            const float32x4_t best = vld1q_f32(top_score + i);
            const uint32x4_t greater = vcgtq_f32(score, best);
            vst1q_f32(top_score + i, vbslq_f32(greater, score, best));
            vst1q_s32(top_class + i, vbslq_s32(greater, class_k, vld1q_s32(top_class + i)));
        }
#endif
        for (; i < anchors; ++i) {
            if (row[i] > top_score[i]) {
                top_score[i] = row[i];
                top_class[i] = k;
            }
        }
    }
}

}  // namespace yolo
