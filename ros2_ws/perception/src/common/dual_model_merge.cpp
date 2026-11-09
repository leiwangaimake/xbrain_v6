/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: dual_model_merge.cpp
 * Brief: Merge the COCO80 and Active12 detections of one frame without cross-model suppression.
 *
 * Description:
 * What problem this solves. Implements dual_model_merge.hpp. Two models see the same frame: the
 * general COCO80 detector and the site-specific Active12 segmenter. Their outputs are concatenated
 * (COCO first), their class ids re-validated against the sealed schema, vehicle/heavy-machinery
 * overlaps recorded for audit only, and the result split into tracker detections and the map-like
 * class 92 (traversable area), which must never become a tracked object (11 section 3.1B.2).
 *
 * Class-92 union. union_traversable_mask() ORs every raw class-92 mask onto the original-image
 * canvas. It is the T-channel input of plan 2f, read by the T producer (t_producer.cpp) since T
 * was wired on 2026-09-30.
 * The former safe_traversable_mask builder (class 92 minus obstacles, eroded; off in the
 * delivered configurations) left in level-3 trim; history at tag pre-level3-trim-20260929.
 */

#include "common/dual_model_merge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "common/postprocess.hpp"

namespace yolo {
namespace {

// Global class ids of the merged 94-class schema (COCO 0..79, Active12 80..93).
constexpr int kHeavyMachineryId = 89;
constexpr int kTraversableAreaId = 92;
constexpr std::array<int, 3> kCocoVehicleIds = {2, 5, 7};  // car, bus, truck

template <std::size_t Size>
bool contains(const std::array<int, Size>& values, int value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

void validate_configuration(const DualModelMergeConfig& config) {
    if (!std::isfinite(config.vehicle_heavy_conflict_iou) ||
        config.vehicle_heavy_conflict_iou < 0.0F ||
        config.vehicle_heavy_conflict_iou > 1.0F) {
        throw std::invalid_argument("vehicle/heavy conflict IoU must be finite in [0, 1]");
    }
}

// COCO detections keep their own ids (0..79) and source.
void validate_coco_detection(const Detection& detection) {
    if (detection.source != DetectionSource::coco80) {
        throw std::invalid_argument("COCO merge input has a non-COCO source");
    }
    if (detection.class_id < 0 || detection.class_id > 79) {
        throw std::invalid_argument("COCO merge input class ID must be in [0, 79]");
    }
    if (detection.original_class_id != detection.class_id) {
        throw std::invalid_argument("COCO local/global class IDs must remain identical");
    }
}

// Active12 local ids must map to the sealed global ids; reserved 87/88 stay
// empty and weak-detection admission stays off.
void validate_active12_detection(const Detection& detection) {
    if (detection.source != DetectionSource::active12) {
        throw std::invalid_argument("active12 merge input has a non-active12 source");
    }
    const ModelSpec& spec = active12_seg_model_spec();
    if (detection.original_class_id < 0 ||
        detection.original_class_id >= spec.num_classes) {
        throw std::invalid_argument("active12 local class ID is outside the sealed schema");
    }
    const int expected_global_id =
        spec.local_to_global_class_ids[
            static_cast<std::size_t>(detection.original_class_id)];
    if (detection.class_id != expected_global_id) {
        throw std::invalid_argument("active12 local/global class mapping mismatch");
    }
    if (detection.class_id == 87 || detection.class_id == 88) {
        throw std::invalid_argument("reserved global classes 87/88 must remain empty");
    }
    if (detection.low_confidence_candidate) {
        throw std::invalid_argument("active12 weak-detection admission is sealed off");
    }
}

// OR one detection's mask into the full-image target; accepts a full-image
// mask or a bbox-local one.
void apply_detection_mask(
    cv::Mat& target,
    const Detection& detection) {
    const cv::Size image_size = target.size();
    if (target.type() != CV_8UC1 ||
        image_size.width <= 0 || image_size.height <= 0) {
        throw std::invalid_argument("invalid traversable canvas size");
    }

    cv::Mat source;
    cv::Rect target_roi(0, 0, image_size.width, image_size.height);
    if (!detection.full_image_mask.empty()) {
        if (detection.full_image_mask.size() != image_size) {
            throw std::invalid_argument("full-image masks must share original-image dimensions");
        }
        source = detection.full_image_mask;
    } else if (!detection.mask.empty() && detection.mask.size() == image_size) {
        source = detection.mask;
    } else if (!detection.mask.empty() &&
               detection.mask.size() == detection.bbox.size()) {
        const cv::Rect clipped =
            detection.bbox & cv::Rect(0, 0, image_size.width, image_size.height);
        if (clipped.empty()) {
            return;
        }
        const cv::Rect local_roi(
            clipped.x - detection.bbox.x,
            clipped.y - detection.bbox.y,
            clipped.width,
            clipped.height);
        source = detection.mask(local_roi);
        target_roi = clipped;
    } else {
        throw std::invalid_argument(
            "fusion mask must be full-image or match its bbox");
    }

    if (source.channels() != 1) {
        throw std::invalid_argument("fusion masks must be single-channel");
    }
    cv::Mat binary;
    cv::compare(source, 0, binary, cv::CMP_GT);
    cv::Mat target_view = target(target_roi);
    cv::bitwise_or(target_view, binary, target_view);
}

// Canvas size from the first class-92 detection's full-image mask (empty: none).
cv::Size traversable_canvas_size(const std::vector<Detection>& detections) {
    for (const Detection& detection : detections) {
        if (detection.class_id != kTraversableAreaId) {
            continue;
        }
        if (!detection.full_image_mask.empty()) {
            return detection.full_image_mask.size();
        }
        if (!detection.mask.empty()) {
            // A class-92 map output is required to carry full-image geometry.
            // Accepting a bbox-local class-92 mask would silently create a
            // wrong navigation canvas, so fail closed below.
            if (detection.mask.size() == detection.bbox.size()) {
                throw std::invalid_argument(
                    "traversable class 92 requires an original-image mask");
            }
            return detection.mask.size();
        }
    }
    return cv::Size();
}

// The detection's mask when it already covers the whole canvas (empty otherwise).
cv::Mat full_canvas_mask(
    const Detection& detection,
    const cv::Size& canvas_size) {
    cv::Mat source;
    if (!detection.full_image_mask.empty()) {
        if (detection.full_image_mask.size() != canvas_size) {
            throw std::invalid_argument(
                "full-image masks must share original-image dimensions");
        }
        source = detection.full_image_mask;
    } else if (!detection.mask.empty() && detection.mask.size() == canvas_size) {
        source = detection.mask;
    }
    if (!source.empty() && source.channels() != 1) {
        throw std::invalid_argument("fusion masks must be single-channel");
    }
    return source;
}

}  // namespace

float box_iou(const cv::Rect& left, const cv::Rect& right) {
    const cv::Rect intersection = left & right;
    const float intersection_area = static_cast<float>(std::max(0, intersection.area()));
    const float union_area =
        static_cast<float>(std::max(0, left.area()) + std::max(0, right.area())) -
        intersection_area;
    return union_area > 0.0F ? intersection_area / union_area : 0.0F;
}

// Plan 2f input (raw class 92, 11 section 3.1B.1): this is the class-92 union
// that was step 1 of the former safe traversable mask builder, kept without its
// obstacle subtraction and erosion.
cv::Mat union_traversable_mask(const std::vector<Detection>& detections) {
    const cv::Size canvas_size = traversable_canvas_size(detections);
    if (canvas_size.empty()) {
        return cv::Mat();
    }

    // Seed the canvas from the first full-image class-92 mask, avoiding a full
    // zero canvas plus a second full-image OR; then OR every other class-92
    // mask. The result equals a zero canvas ORed with every class-92 mask.
    cv::Mat traversable;
    const Detection* seeded_detection = nullptr;
    for (const Detection& detection : detections) {
        if (detection.class_id != kTraversableAreaId) {
            continue;
        }
        const cv::Mat source = full_canvas_mask(detection, canvas_size);
        if (source.empty()) {
            continue;
        }
        cv::compare(source, 0, traversable, cv::CMP_GT);
        seeded_detection = &detection;
        break;
    }
    if (traversable.empty()) {
        traversable = cv::Mat::zeros(canvas_size, CV_8UC1);
    }
    for (const Detection& detection : detections) {
        if (detection.class_id != kTraversableAreaId ||
            &detection == seeded_detection) {
            continue;
        }
        apply_detection_mask(traversable, detection);
    }
    return traversable;
}

// Class 92 goes to the map side; everything else to the tracker.
void partition_tracker_detections(
    const std::vector<Detection>& detections,
    std::vector<Detection>& tracker_detections,
    std::vector<Detection>& map_detections) {
    tracker_detections.clear();
    map_detections.clear();
    tracker_detections.reserve(detections.size());
    map_detections.reserve(detections.size());
    for (const Detection& detection : detections) {
        if (detection.class_id == kTraversableAreaId) {
            map_detections.push_back(detection);
        } else {
            tracker_detections.push_back(detection);
        }
    }
}

DualModelMergeOutput merge_dual_model_detections(
    const std::vector<Detection>& coco80_detections,
    const std::vector<Detection>& active12_detections,
    const DualModelMergeConfig& config) {
    validate_configuration(config);
    for (const Detection& detection : coco80_detections) {
        validate_coco_detection(detection);
    }
    for (const Detection& detection : active12_detections) {
        validate_active12_detection(detection);
    }

    DualModelMergeOutput output;
    output.merged_detections.reserve(
        coco80_detections.size() + active12_detections.size());
    output.merged_detections.insert(
        output.merged_detections.end(),
        coco80_detections.begin(),
        coco80_detections.end());
    output.merged_detections.insert(
        output.merged_detections.end(),
        active12_detections.begin(),
        active12_detections.end());

    // There is deliberately no generic cross-model NMS. Only record sealed
    // vehicle-vs-heavy overlap evidence at IoU >= 0.30; suppress nothing.
    for (std::size_t active_index = 0;
         active_index < active12_detections.size();
         ++active_index) {
        const Detection& active_detection = active12_detections[active_index];
        if (active_detection.class_id != kHeavyMachineryId) {
            continue;
        }
        for (std::size_t old_index = 0;
             old_index < coco80_detections.size();
             ++old_index) {
            const Detection& old_detection = coco80_detections[old_index];
            if (!contains(kCocoVehicleIds, old_detection.class_id)) {
                continue;
            }
            const float overlap =
                box_iou(old_detection.bbox, active_detection.bbox);
            if (overlap < config.vehicle_heavy_conflict_iou) {
                continue;
            }
            CrossModelConflictAudit audit;
            audit.old_index = old_index;
            audit.active_index = active_index;
            audit.old_class_id = old_detection.class_id;
            audit.active_class_id = active_detection.class_id;
            audit.box_iou = overlap;
            // Sealed mode is audit_only: neither branch is ever suppressed.
            audit.mode = "audit_only";
            audit.old_suppressed = false;
            output.conflicts.push_back(std::move(audit));
        }
    }

    partition_tracker_detections(
        output.merged_detections,
        output.tracker_detections,
        output.map_detections);
    return output;
}

}  // namespace yolo
