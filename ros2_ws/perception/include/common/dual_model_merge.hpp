/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: dual_model_merge.hpp
 * Brief: Pure dual-model fusion utilities.
 *
 * Description:
 *   1. Preserve both model branches without generic cross-model NMS.
 *   2. Audit vehicle/heavy-machinery overlaps without suppressing either side.
 *   3. Split map-like traversable output from ordinary tracker detections.
 *   4. Union the raw class-92 masks: the T-channel input, read by the T producer since plan 2f.
 */

#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "common/types.hpp"

namespace yolo {

// Audit threshold only: a COCO vehicle and an Active12 heavy-machinery box overlapping at least
// this much are recorded as a conflict; neither is removed.
struct DualModelMergeConfig {
    float vehicle_heavy_conflict_iou = 0.30F;
};

struct DualModelMergeOutput {
    // Ordered COCO results followed by active12 results. No generic
    // cross-model NMS is applied.
    std::vector<Detection> merged_detections;
    // Ordinary instances only; class 92 is never included.
    std::vector<Detection> tracker_detections;
    // Map-like outputs currently containing global class 92.
    std::vector<Detection> map_detections;
    std::vector<CrossModelConflictAudit> conflicts;
};

/**
 * @brief Standard rectangle IoU used by the audit-only conflict inventory.
 */
float box_iou(const cv::Rect& left, const cv::Rect& right);

/**
 * @brief Union of every global class-92 (traversable area) mask.
 *
 * Returns the RAW traversable segmentation as an original-image CV_8UC1 mask
 * (255 inside): no obstacle is subtracted and nothing is eroded. Empty when no
 * class-92 detection carries a mask. The first class-92 mask must be
 * full-image; a bbox-local one fails closed (std::invalid_argument).
 *
 * The plan 2f input: the T channel takes raw class 92 (11 section 3.1B.1),
 * never the former safe_traversable product. The T producer
 * (t_producer.cpp) calls it for every T sample.
 */
cv::Mat union_traversable_mask(const std::vector<Detection>& detections);

/**
 * @brief Separate ordinary instances from map-like class 92 output.
 */
void partition_tracker_detections(
    const std::vector<Detection>& detections,
    std::vector<Detection>& tracker_detections,
    std::vector<Detection>& map_detections);

/**
 * @brief Validate, audit, and combine one decoded result from each branch.
 *
 * Inputs must already be in global class IDs and original-image coordinates.
 * active12 local/global mapping is revalidated against the sealed contract.
 */
DualModelMergeOutput merge_dual_model_detections(
    const std::vector<Detection>& coco80_detections,
    const std::vector<Detection>& active12_detections,
    const DualModelMergeConfig& config = DualModelMergeConfig{});

}  // namespace yolo
