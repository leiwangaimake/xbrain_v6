/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_dual_model_merge.cpp
 * Brief: Unit tests for merging the COCO and active12 detections into one result.
 *
 * Description:
 * What it checks. Eight numbered cases pin the merge policy: the fixed active12 class mapping
 * (IDs 87/88 stay empty), audit mode keeping both a COCO truck and active heavy machinery, no
 * generic cross-model NMS even for identical boxes, the inclusive 0.30 IoU threshold, class 92 as
 * a map output while pit 93 stays a tracked instance, fail-closed handling of wrong mappings, and
 * the raw class-92 union kept for plan 2f: every class-92 instance, obstacles not subtracted, no
 * erosion, empty without a class-92 mask and fail-closed on a bbox-local first mask.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>

#include "common/dual_model_merge.hpp"
#include "common/postprocess.hpp"

namespace {

yolo::Detection coco_detection(
    int class_id,
    const cv::Rect& bbox = cv::Rect(0, 0, 10, 10),
    const cv::Mat& full_mask = cv::Mat()) {
    yolo::Detection detection;
    detection.bbox = bbox;
    detection.confidence = 0.90F;
    detection.class_id = class_id;
    detection.original_class_id = class_id;
    detection.source = yolo::DetectionSource::coco80;
    detection.full_image_mask = full_mask;
    return detection;
}

yolo::Detection active_detection(
    int local_class_id,
    const cv::Rect& bbox = cv::Rect(0, 0, 10, 10),
    const cv::Mat& full_mask = cv::Mat()) {
    const yolo::ModelSpec& spec = yolo::active12_seg_model_spec();
    yolo::Detection detection;
    detection.bbox = bbox;
    detection.confidence = 0.90F;
    detection.original_class_id = local_class_id;
    detection.class_id =
        spec.local_to_global_class_ids[static_cast<std::size_t>(local_class_id)];
    detection.class_name =
        spec.class_names[static_cast<std::size_t>(local_class_id)];
    detection.source = yolo::DetectionSource::active12;
    detection.full_image_mask = full_mask;
    return detection;
}

cv::Mat binary_canvas(int rows, int cols, const cv::Rect& on) {
    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    mask(on).setTo(255);
    return mask;
}

}  // namespace

int main() {
    // 1. Fixed active12 mapping covers all delivered classes and leaves 87/88 empty.
    const yolo::ModelSpec& spec = yolo::active12_seg_model_spec();
    assert((spec.local_to_global_class_ids ==
            std::vector<int>{80, 81, 82, 83, 84, 85, 86, 89, 90, 91, 92, 93}));

    // 2. Audit mode preserves both a COCO truck and active heavy machinery.
    const auto conflict_output = yolo::merge_dual_model_detections(
        {coco_detection(7, cv::Rect(0, 0, 100, 100))},
        {active_detection(7, cv::Rect(0, 0, 100, 100))});
    assert(conflict_output.merged_detections.size() == 2U);
    assert(conflict_output.tracker_detections.size() == 2U);
    assert(conflict_output.conflicts.size() == 1U);
    assert(conflict_output.conflicts[0].old_class_id == 7);
    assert(conflict_output.conflicts[0].active_class_id == 89);
    assert(conflict_output.conflicts[0].mode == "audit_only");
    assert(!conflict_output.conflicts[0].old_suppressed);

    // 3. No generic cross-model NMS even for exactly overlapping boxes.
    const auto no_generic_nms = yolo::merge_dual_model_detections(
        {coco_detection(0)},
        {active_detection(4)});
    assert(no_generic_nms.merged_detections.size() == 2U);
    assert(no_generic_nms.conflicts.empty());

    // 4. Formal IoU threshold is inclusive at 0.30.
    yolo::DualModelMergeConfig threshold_config;
    threshold_config.vehicle_heavy_conflict_iou = 0.30F;
    // Intersection 60x100, union 140x100 => 3/7, safely above 0.30.
    const auto threshold_audit = yolo::merge_dual_model_detections(
        {coco_detection(2, cv::Rect(0, 0, 100, 100))},
        {active_detection(7, cv::Rect(40, 0, 100, 100))},
        threshold_config);
    assert(threshold_audit.conflicts.size() == 1U);
    assert(threshold_audit.conflicts[0].box_iou >= 0.30F);

    // 5. Class 92 is a map output; pit 93 remains an ordinary tracker instance.
    const cv::Mat traversable(20, 20, CV_8UC1, cv::Scalar(255));
    const auto partitioned = yolo::merge_dual_model_detections(
        {},
        {
            active_detection(10, cv::Rect(0, 0, 20, 20), traversable),
            active_detection(11),
        });
    assert(partitioned.map_detections.size() == 1U);
    assert(partitioned.map_detections[0].class_id == 92);
    assert(partitioned.tracker_detections.size() == 1U);
    assert(partitioned.tracker_detections[0].class_id == 93);

    // 6. Wrong/reserved active mapping fails closed instead of leaking ID 87/88.
    yolo::Detection invalid = active_detection(0);
    invalid.class_id = 87;
    bool invalid_mapping_threw = false;
    try {
        (void)yolo::merge_dual_model_detections({}, {invalid});
    } catch (const std::invalid_argument&) {
        invalid_mapping_threw = true;
    }
    assert(invalid_mapping_threw);

    // 7. The raw class-92 union (plan 2f input) covers every class-92 instance,
    //    including disjoint full-image and bbox-local masks (binarized to 255);
    //    an overlapping active pit and COCO person are not subtracted and
    //    nothing is eroded. Both class-92 masks carry the value 1, not 255, so
    //    a union that copied a mask instead of binarizing it fails below.
    cv::Mat left_traversable = cv::Mat::zeros(20, 20, CV_8UC1);
    left_traversable(cv::Rect(1, 1, 4, 4)).setTo(1);
    yolo::Detection right_traversable_detection =
        active_detection(10, cv::Rect(15, 15, 4, 4));
    right_traversable_detection.mask =
        cv::Mat(4, 4, CV_8UC1, cv::Scalar(1));
    const cv::Mat pit = binary_canvas(20, 20, cv::Rect(2, 2, 2, 2));
    const cv::Mat person = binary_canvas(20, 20, cv::Rect(15, 15, 2, 2));
    const auto union_output = yolo::merge_dual_model_detections(
        {coco_detection(0, cv::Rect(15, 15, 2, 2), person)},
        {
            active_detection(10, cv::Rect(1, 1, 4, 4), left_traversable),
            right_traversable_detection,
            active_detection(11, cv::Rect(2, 2, 2, 2), pit),
        });
    assert(union_output.map_detections.size() == 2U);
    const cv::Mat raw_traversable =
        yolo::union_traversable_mask(union_output.merged_detections);
    assert(raw_traversable.size() == cv::Size(20, 20));
    assert(raw_traversable.type() == CV_8UC1);
    assert(raw_traversable.at<std::uint8_t>(2, 2) == 255U);
    assert(raw_traversable.at<std::uint8_t>(16, 16) == 255U);
    assert(raw_traversable.at<std::uint8_t>(10, 10) == 0U);
    assert(cv::countNonZero(raw_traversable) == 32);
    // Only class 92 contributes: the map detections alone give the same union.
    assert(cv::norm(
               raw_traversable,
               yolo::union_traversable_mask(union_output.map_detections),
               cv::NORM_INF) == 0.0);

    // 8. Without a class-92 mask the union is empty. A bbox-local first
    //    class-92 mask fails closed in the union instead of inventing a canvas,
    //    while the merge itself still routes it to the map side.
    assert(yolo::union_traversable_mask({}).empty());
    assert(yolo::union_traversable_mask(
               {active_detection(11, cv::Rect(2, 2, 2, 2), pit)})
               .empty());
    yolo::Detection local_only = active_detection(10, cv::Rect(15, 15, 4, 4));
    local_only.mask = cv::Mat(4, 4, CV_8UC1, cv::Scalar(255));
    bool local_first_threw = false;
    try {
        (void)yolo::union_traversable_mask({local_only});
    } catch (const std::invalid_argument&) {
        local_first_threw = true;
    }
    assert(local_first_threw);
    const auto local_merge = yolo::merge_dual_model_detections({}, {local_only});
    assert(local_merge.map_detections.size() == 1U);

    std::cout << "All dual-model merge tests passed!" << std::endl;
    return 0;
}
