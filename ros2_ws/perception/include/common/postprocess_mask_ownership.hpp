/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: postprocess_mask_ownership.hpp
 * Brief: Final ownership of the decoded masks (PERCEPTION_MASK_OUTPUT_COPY: clone or retain).
 *
 * Description:
 * clone (the production runtime set) deep-copies every mask once more; retain keeps the
 * decoder's own allocations, which are already independent (see below). The full-image mask is
 * dropped except for Active12's first class-92 instance.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "common/types.hpp"

namespace perception {

enum class MaskOutputCopyMode { Clone, Retain };

inline MaskOutputCopyMode parse_mask_output_copy_mode(const char* value) {
    if (value == nullptr || std::strcmp(value, "clone") == 0) {
        return MaskOutputCopyMode::Clone;
    }
    if (std::strcmp(value, "retain") == 0) {
        return MaskOutputCopyMode::Retain;
    }
    throw std::invalid_argument("PERCEPTION_MASK_OUTPUT_COPY must be clone or retain");
}

inline const char* mask_output_copy_mode_name(MaskOutputCopyMode mode) {
    switch (mode) {
        case MaskOutputCopyMode::Clone: return "clone";
        case MaskOutputCopyMode::Retain: return "retain";
    }
    throw std::invalid_argument("invalid postprocess mask output copy mode");
}

// postprocess_seg already returns independently allocated CV_8UC1 instance
// masks and, for Active12's first valid class 92, an independent canvas. The
// decoder's temporary float matrices and TensorRT inputs are not their owners.
// Call only after all decoder lanes have joined. Retain removes an additional
// deep copy; it is not a facility for extending the lifetime of borrowed data.
// COCO keeps its existing bbox-local contract in both modes.
// The return value counts clone destination bytes, not total memory traffic.
inline std::uint64_t finalize_postprocess_mask_ownership(
    std::vector<yolo::Detection>& detections,
    MaskOutputCopyMode mode,
    bool keep_full_image_mask) {
    // Reject an invalid enum before mutating even the first detection.
    (void)mask_output_copy_mode_name(mode);
    std::uint64_t copied_bytes = 0;
    const auto clone_mask = [&](cv::Mat& mask) {
        if (!mask.empty()) {
            const auto bytes = static_cast<std::uint64_t>(mask.total()) * mask.elemSize();
            mask = mask.clone();
            copied_bytes += bytes;
        }
    };
    for (auto& detection : detections) {
        if (mode == MaskOutputCopyMode::Clone) {
            clone_mask(detection.mask);
        }
        if (keep_full_image_mask) {
            if (mode == MaskOutputCopyMode::Clone) {
                clone_mask(detection.full_image_mask);
            }
        } else {
            detection.full_image_mask.release();
        }
    }
    return copied_bytes;
}

}  // namespace perception
