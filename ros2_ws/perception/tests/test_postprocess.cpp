/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_postprocess.cpp
 * Brief: Unit tests for the YOLO postprocess: decode, thresholds, the weak tier, NMS and masks.
 *
 * Description:
 * What it checks. Detection and segmentation decode (boxes, classes, bbox-local masks); the raw-top
 * class threshold and the weak (low-confidence) tier, which never relabels a class or bypasses a
 * normal threshold; weak proxies against normal candidates in overlap suppression; pure IoU NMS on
 * the un-clipped letterbox boxes; only class 92 keeps a full-image canvas; mask cropping and the
 * prototype ROI as in Ultralytics 8.4.66, with the older C++ behaviour rejected explicitly; and the
 * sealed class mapping and output tensor contract failing closed.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/postprocess.hpp"

namespace {

constexpr int64_t kNumBoxes = 8400;
constexpr int kChannels = 84;
constexpr int kSegChannels = 116;
constexpr int kMaskChannels = 32;
constexpr int kMaskHeight = 160;
constexpr int kMaskWidth = 160;
constexpr int64_t kActiveNumBoxes = 16464;
constexpr int kActiveChannels = 48;
constexpr int kActiveMaskHeight = 224;
constexpr int kActiveMaskWidth = 224;

void set_box(
    std::vector<float>& output,
    int64_t box_idx,
    float cx,
    float cy,
    float w,
    float h,
    int class_id,
    float conf) {
    output[box_idx] = cx;
    output[kNumBoxes + box_idx] = cy;
    output[(2 * kNumBoxes) + box_idx] = w;
    output[(3 * kNumBoxes) + box_idx] = h;
    output[((4 + class_id) * kNumBoxes) + box_idx] = conf;
}

void set_seg_box(
    std::vector<float>& output,
    int64_t box_idx,
    float cx,
    float cy,
    float w,
    float h,
    int class_id,
    float conf,
    float first_mask_coef) {
    output[box_idx] = cx;
    output[kNumBoxes + box_idx] = cy;
    output[(2 * kNumBoxes) + box_idx] = w;
    output[(3 * kNumBoxes) + box_idx] = h;
    output[((4 + class_id) * kNumBoxes) + box_idx] = conf;
    output[(84 * kNumBoxes) + box_idx] = first_mask_coef;
}

void set_active_seg_box(
    std::vector<float>& output,
    int64_t box_idx,
    float cx,
    float cy,
    float w,
    float h,
    int local_class_id,
    float conf,
    float first_mask_coef) {
    output[box_idx] = cx;
    output[kActiveNumBoxes + box_idx] = cy;
    output[(2 * kActiveNumBoxes) + box_idx] = w;
    output[(3 * kActiveNumBoxes) + box_idx] = h;
    output[((4 + local_class_id) * kActiveNumBoxes) + box_idx] = conf;
    output[(16 * kActiveNumBoxes) + box_idx] = first_mask_coef;
}

int differing_pixels(const cv::Mat& lhs, const cv::Mat& rhs) {
    assert(lhs.type() == rhs.type());
    assert(lhs.size() == rhs.size());
    cv::Mat difference;
    cv::compare(lhs, rhs, difference, cv::CMP_NE);
    return cv::countNonZero(difference);
}

}  // namespace

int main() {
    yolo::LetterboxInfo info;
    info.scale = 1.0F;
    info.pad_left = 0;
    info.pad_top = 80;
    info.new_width = 640;
    info.new_height = 480;
    info.original_width = 640;
    info.original_height = 480;

    std::vector<float> output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    const std::vector<int64_t> shape = {1, kChannels, kNumBoxes};

    set_box(output, 0, 150.0F, 170.0F, 100.0F, 80.0F, 0, 0.90F);
    set_box(output, 1, 152.0F, 172.0F, 100.0F, 80.0F, 0, 0.80F);
    set_box(output, 2, 420.0F, 280.0F, 60.0F, 40.0F, 2, 0.10F);
    set_box(output, 3, 320.0F, 300.0F, 40.0F, 60.0F, 2, 0.70F);

    const std::vector<yolo::Detection> detections =
        yolo::postprocess(output.data(), shape, info, 0.25F, 0.45F, 640);

    assert(detections.size() == 2);

    assert(detections[0].class_id == 0);
    assert(detections[0].class_name == "person");
    assert(detections[0].bbox.x == 100);
    assert(detections[0].bbox.y == 50);
    assert(detections[0].bbox.width == 100);
    assert(detections[0].bbox.height == 80);

    assert(detections[1].class_id == 2);
    assert(detections[1].class_name == "car");
    assert(detections[1].bbox.x == 300);
    assert(detections[1].bbox.y == 190);
    assert(detections[1].bbox.width == 40);
    assert(detections[1].bbox.height == 60);
    assert(!detections[0].low_confidence_candidate);
    assert(!detections[1].low_confidence_candidate);

    std::vector<float> seg_output(static_cast<std::size_t>(kSegChannels * kNumBoxes), 0.0F);
    std::vector<float> proto(static_cast<std::size_t>(kMaskChannels * kMaskHeight * kMaskWidth), 0.0F);
    std::fill(proto.begin(), proto.begin() + (kMaskHeight * kMaskWidth), 1.0F);
    const std::vector<int64_t> seg_shape = {1, kSegChannels, kNumBoxes};
    const std::vector<int64_t> proto_shape = {1, kMaskChannels, kMaskHeight, kMaskWidth};

    set_seg_box(seg_output, 0, 150.0F, 170.0F, 100.0F, 80.0F, 0, 0.90F, 10.0F);

    const std::vector<yolo::Detection> seg_detections = yolo::postprocess_seg(
        seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        0.45F,
        0.5F,
        640);

    assert(seg_detections.size() == 1);
    assert(seg_detections[0].bbox.x == 100);
    assert(seg_detections[0].bbox.y == 50);
    assert(seg_detections[0].mask.type() == CV_8UC1);
    assert(seg_detections[0].mask.cols == seg_detections[0].bbox.width);
    assert(seg_detections[0].mask.rows == seg_detections[0].bbox.height);
    assert(cv::countNonZero(seg_detections[0].mask) > 0);
    // Legacy overload parity: do not allocate one full-frame mask per COCO
    // detection (COCO80 masks stay bbox-local).
    assert(seg_detections[0].full_image_mask.empty());
    assert(!seg_detections[0].low_confidence_candidate);

    std::vector<float> per_class_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(per_class_output, 0, 100.0F, 140.0F, 40.0F, 40.0F, 0, 0.50F);
    set_box(per_class_output, 1, 200.0F, 180.0F, 40.0F, 40.0F, 2, 0.30F);
    const std::map<int, float> per_class_conf = {{0, 0.60F}, {2, 0.20F}};
    const std::vector<yolo::Detection> per_class_detections = yolo::postprocess(
        per_class_output.data(),
        shape,
        info,
        0.25F,
        per_class_conf,
        0.45F,
        640);
    assert(per_class_detections.size() == 1U);
    assert(per_class_detections[0].class_id == 2);

    // Compatibility: with the weak path disabled, choose the raw top class
    // first and reject the whole anchor when that class misses its threshold.
    std::vector<float> threshold_aware_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(threshold_aware_output, 0, 240.0F, 220.0F, 80.0F, 60.0F, 0, 0.50F);
    set_box(threshold_aware_output, 0, 240.0F, 220.0F, 80.0F, 60.0F, 2, 0.30F);
    const std::vector<yolo::Detection> threshold_aware_detections = yolo::postprocess(
        threshold_aware_output.data(),
        shape,
        info,
        0.25F,
        per_class_conf,
        0.45F,
        640);
    assert(threshold_aware_detections.empty());
    const auto explicit_floor_zero = yolo::postprocess(
        threshold_aware_output.data(),
        shape,
        info,
        0.25F,
        per_class_conf,
        0.0F,
        8,
        0.45F,
        640);
    assert(explicit_floor_zero.empty());

    // Enabling weak candidates must not let a lower-ranked normal class bypass
    // the strict raw-top person threshold either.
    const auto weak_enabled_strict_normal = yolo::postprocess(
        threshold_aware_output.data(),
        shape,
        info,
        0.25F,
        per_class_conf,
        0.10F,
        8,
        0.45F,
        640);
    assert(weak_enabled_strict_normal.empty());

    std::vector<float> strict_person_pass_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(strict_person_pass_output, 0, 240.0F, 220.0F, 80.0F, 60.0F, 0, 0.65F);
    set_box(strict_person_pass_output, 0, 240.0F, 220.0F, 80.0F, 60.0F, 2, 0.30F);
    const auto strict_person_pass = yolo::postprocess(
        strict_person_pass_output.data(),
        shape,
        info,
        0.25F,
        per_class_conf,
        0.10F,
        8,
        0.45F,
        640);
    assert(strict_person_pass.size() == 1U);
    assert(strict_person_pass[0].class_id == 0);
    assert(!strict_person_pass[0].low_confidence_candidate);

    const std::vector<yolo::Detection> fallback_detections = yolo::postprocess(
        per_class_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.45F,
        640);
    assert(fallback_detections.size() == 2U);

    std::vector<float> nested_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(nested_output, 0, 250.0F, 250.0F, 200.0F, 200.0F, 62, 0.80F);
    set_box(nested_output, 1, 250.0F, 250.0F, 100.0F, 100.0F, 62, 0.40F);
    const std::vector<yolo::Detection> nested_detections = yolo::postprocess(
        nested_output.data(),
        shape,
        info,
        0.25F,
        0.45F,
        640);
    assert(nested_detections.size() == 1U);
    assert(nested_detections[0].class_id == 62);
    assert(nested_detections[0].bbox.width == 200);

    // Formal_003-like partial nesting is intentionally left for the temporal
    // publication layer. IoU is about 0.16 and IOSA about 0.82, below detector
    // NMS's strict containment boundary, so both raw candidates must survive.
    std::vector<float> partial_nested_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(
        partial_nested_output,
        0,
        181.0F,
        302.5F,
        262.0F,
        245.0F,
        56,
        0.80F);
    set_box(
        partial_nested_output,
        1,
        150.5F,
        209.5F,
        141.0F,
        91.0F,
        56,
        0.70F);
    const auto partial_nested_detections = yolo::postprocess(
        partial_nested_output.data(),
        shape,
        info,
        0.25F,
        0.45F,
        640);
    assert(partial_nested_detections.size() == 2U);
    assert(partial_nested_detections[0].class_id == 56);
    assert(partial_nested_detections[1].class_id == 56);

    std::vector<float> per_class_seg_output(static_cast<std::size_t>(kSegChannels * kNumBoxes), 0.0F);
    set_seg_box(per_class_seg_output, 0, 150.0F, 170.0F, 100.0F, 80.0F, 0, 0.50F, 10.0F);
    set_seg_box(per_class_seg_output, 1, 300.0F, 220.0F, 80.0F, 80.0F, 2, 0.30F, 10.0F);
    const std::vector<yolo::Detection> per_class_seg_detections = yolo::postprocess_seg(
        per_class_seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        per_class_conf,
        0.45F,
        0.5F,
        640);
    assert(per_class_seg_detections.size() == 1U);
    assert(per_class_seg_detections[0].class_id == 2);

    std::vector<float> threshold_aware_seg_output(
        static_cast<std::size_t>(kSegChannels * kNumBoxes),
        0.0F);
    set_seg_box(threshold_aware_seg_output, 0, 240.0F, 220.0F, 80.0F, 60.0F, 0, 0.50F, 10.0F);
    set_seg_box(threshold_aware_seg_output, 0, 240.0F, 220.0F, 80.0F, 60.0F, 2, 0.30F, 10.0F);
    const std::vector<yolo::Detection> threshold_aware_seg_detections = yolo::postprocess_seg(
        threshold_aware_seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        per_class_conf,
        0.45F,
        0.5F,
        640);
    assert(threshold_aware_seg_detections.empty());

    const auto weak_enabled_strict_seg = yolo::postprocess_seg(
        threshold_aware_seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        per_class_conf,
        0.10F,
        8,
        0.45F,
        0.5F,
        640);
    assert(weak_enabled_strict_seg.empty());

    std::vector<float> nonperson_strict_seg_output(
        static_cast<std::size_t>(kSegChannels * kNumBoxes),
        0.0F);
    set_seg_box(
        nonperson_strict_seg_output,
        0,
        240.0F,
        220.0F,
        80.0F,
        60.0F,
        72,
        0.49F,
        10.0F);
    set_seg_box(
        nonperson_strict_seg_output,
        0,
        240.0F,
        220.0F,
        80.0F,
        60.0F,
        2,
        0.40F,
        10.0F);
    const auto nonperson_strict_weak_seg = yolo::postprocess_seg(
        nonperson_strict_seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        {{72, 0.50F}},
        0.10F,
        8,
        0.45F,
        0.5F,
        640);
    assert(nonperson_strict_weak_seg.size() == 1U);
    assert(nonperson_strict_weak_seg[0].class_id == 72);
    assert(nonperson_strict_weak_seg[0].low_confidence_candidate);
    assert(cv::countNonZero(nonperson_strict_weak_seg[0].mask) > 0);

    std::vector<float> nested_seg_output(static_cast<std::size_t>(kSegChannels * kNumBoxes), 0.0F);
    set_seg_box(nested_seg_output, 0, 250.0F, 250.0F, 200.0F, 200.0F, 62, 0.80F, 10.0F);
    set_seg_box(nested_seg_output, 1, 250.0F, 250.0F, 100.0F, 100.0F, 62, 0.40F, 10.0F);
    const std::vector<yolo::Detection> nested_seg_detections = yolo::postprocess_seg(
        nested_seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        0.45F,
        0.5F,
        640);
    assert(nested_seg_detections.size() == 1U);
    assert(nested_seg_detections[0].class_id == 62);
    assert(nested_seg_detections[0].bbox.width == 200);

    // The legacy overload keeps the weak path disabled, preserving the exact
    // threshold behavior unless the new floor is explicitly configured.
    std::vector<float> weak_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(weak_output, 0, 120.0F, 160.0F, 60.0F, 80.0F, 28, 0.18F);
    const auto weak_default_off =
        yolo::postprocess(weak_output.data(), shape, info, 0.25F, 0.45F, 640);
    assert(weak_default_off.empty());

    const auto weak_admitted = yolo::postprocess(
        weak_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.10F,
        8,
        0.45F,
        640);
    assert(weak_admitted.size() == 1U);
    assert(weak_admitted[0].class_id == 28);
    assert(weak_admitted[0].low_confidence_candidate);

    // Normal admission remains raw-top-only when weak mode is enabled. A
    // lower-ranked class with a permissive override must not turn the anchor
    // into a normal detection.
    std::vector<float> weak_runner_up_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(weak_runner_up_output, 0, 150.0F, 180.0F, 70.0F, 90.0F, 28, 0.12F);
    set_box(weak_runner_up_output, 0, 150.0F, 180.0F, 70.0F, 90.0F, 56, 0.11F);
    const auto weak_runner_up = yolo::postprocess(
        weak_runner_up_output.data(),
        shape,
        info,
        0.25F,
        {{56, 0.10F}},
        0.10F,
        8,
        0.45F,
        640);
    assert(weak_runner_up.size() == 1U);
    assert(weak_runner_up[0].class_id == 28);
    assert(weak_runner_up[0].low_confidence_candidate);

    // A raw-top person is never relabeled through the weak path, even if a
    // lower-scored non-person class on the same anchor clears the weak floor.
    std::vector<float> weak_person_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(weak_person_output, 0, 180.0F, 180.0F, 80.0F, 100.0F, 0, 0.20F);
    set_box(weak_person_output, 0, 180.0F, 180.0F, 80.0F, 100.0F, 2, 0.18F);
    const auto rejected_weak_person = yolo::postprocess(
        weak_person_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.10F,
        8,
        0.45F,
        640);
    assert(rejected_weak_person.empty());

    // A non-person class keeps its stricter normal threshold but may degrade
    // to the generic weak tier. The lower-ranked class still cannot replace
    // the raw-top class or become a normal semantic detection.
    std::vector<float> strict_class_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(strict_class_output, 0, 220.0F, 200.0F, 90.0F, 120.0F, 72, 0.22F);
    set_box(strict_class_output, 0, 220.0F, 200.0F, 90.0F, 120.0F, 2, 0.18F);
    const auto degraded_strict_class = yolo::postprocess(
        strict_class_output.data(),
        shape,
        info,
        0.25F,
        {{72, 0.50F}},
        0.10F,
        8,
        0.45F,
        640);
    assert(degraded_strict_class.size() == 1U);
    assert(degraded_strict_class[0].class_id == 72);
    assert(degraded_strict_class[0].class_name == "refrigerator");
    assert(degraded_strict_class[0].low_confidence_candidate);
    assert(std::abs(degraded_strict_class[0].confidence - 0.22F) < 1e-6F);

    std::vector<float> strict_boundary_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(strict_boundary_output, 0, 220.0F, 200.0F, 90.0F, 120.0F, 72, 0.49F);
    set_box(strict_boundary_output, 0, 220.0F, 200.0F, 90.0F, 120.0F, 2, 0.40F);
    const auto strict_boundary_weak = yolo::postprocess(
        strict_boundary_output.data(),
        shape,
        info,
        0.25F,
        {{72, 0.50F}},
        0.10F,
        8,
        0.45F,
        640);
    assert(strict_boundary_weak.size() == 1U);
    assert(strict_boundary_weak[0].class_id == 72);
    assert(strict_boundary_weak[0].low_confidence_candidate);

    const auto strict_boundary_weak_disabled = yolo::postprocess(
        strict_boundary_output.data(),
        shape,
        info,
        0.25F,
        {{72, 0.50F}},
        0.0F,
        8,
        0.45F,
        640);
    assert(strict_boundary_weak_disabled.empty());

    std::vector<float> strict_boundary_normal_output = strict_boundary_output;
    set_box(
        strict_boundary_normal_output,
        0,
        220.0F,
        200.0F,
        90.0F,
        120.0F,
        72,
        0.50F);
    const auto strict_boundary_normal = yolo::postprocess(
        strict_boundary_normal_output.data(),
        shape,
        info,
        0.25F,
        {{72, 0.50F}},
        0.10F,
        8,
        0.45F,
        640);
    assert(strict_boundary_normal.size() == 1U);
    assert(strict_boundary_normal[0].class_id == 72);
    assert(!strict_boundary_normal[0].low_confidence_candidate);

    std::vector<float> person_boundary_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(person_boundary_output, 0, 220.0F, 200.0F, 90.0F, 120.0F, 0, 0.39F);
    assert(yolo::postprocess(
               person_boundary_output.data(),
               shape,
               info,
               0.25F,
               {{0, 0.40F}},
               0.02F,
               8,
               0.45F,
               640)
               .empty());
    set_box(person_boundary_output, 0, 220.0F, 200.0F, 90.0F, 120.0F, 0, 0.40F);
    const auto person_boundary_normal = yolo::postprocess(
        person_boundary_output.data(),
        shape,
        info,
        0.25F,
        {{0, 0.40F}},
        0.02F,
        8,
        0.45F,
        640);
    assert(person_boundary_normal.size() == 1U);
    assert(person_boundary_normal[0].class_id == 0);
    assert(!person_boundary_normal[0].low_confidence_candidate);

    // Weak proxies ignore unstable semantic labels for overlap suppression.
    std::vector<float> weak_overlap_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(weak_overlap_output, 0, 280.0F, 250.0F, 120.0F, 160.0F, 28, 0.20F);
    set_box(weak_overlap_output, 1, 282.0F, 252.0F, 120.0F, 160.0F, 56, 0.19F);
    const auto weak_overlap = yolo::postprocess(
        weak_overlap_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.10F,
        8,
        0.45F,
        640);
    assert(weak_overlap.size() == 1U);
    assert(weak_overlap[0].class_id == 28);
    assert(weak_overlap[0].low_confidence_candidate);

    std::vector<float> weak_cap_output(static_cast<std::size_t>(kChannels * kNumBoxes), 0.0F);
    set_box(weak_cap_output, 0, 80.0F, 200.0F, 40.0F, 60.0F, 28, 0.20F);
    set_box(weak_cap_output, 1, 180.0F, 200.0F, 40.0F, 60.0F, 56, 0.19F);
    set_box(weak_cap_output, 2, 280.0F, 200.0F, 40.0F, 60.0F, 39, 0.18F);
    const auto weak_capped = yolo::postprocess(
        weak_cap_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.10F,
        2,
        0.45F,
        640);
    assert(weak_capped.size() == 2U);

    // Normal candidates always win overlap against weak proxies.
    std::vector<float> normal_weak_overlap_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(normal_weak_overlap_output, 0, 340.0F, 250.0F, 120.0F, 160.0F, 56, 0.30F);
    set_box(normal_weak_overlap_output, 1, 342.0F, 252.0F, 120.0F, 160.0F, 28, 0.20F);
    const auto normal_weak_overlap = yolo::postprocess(
        normal_weak_overlap_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.10F,
        8,
        0.45F,
        640);
    assert(normal_weak_overlap.size() == 1U);
    assert(normal_weak_overlap[0].class_id == 56);
    assert(!normal_weak_overlap[0].low_confidence_candidate);

    // The new path must not alter legacy cross-class behavior for two normal
    // detections that happen to occupy the same region.
    std::vector<float> normal_cross_class_output(
        static_cast<std::size_t>(kChannels * kNumBoxes),
        0.0F);
    set_box(normal_cross_class_output, 0, 400.0F, 250.0F, 120.0F, 160.0F, 2, 0.30F);
    set_box(normal_cross_class_output, 1, 402.0F, 252.0F, 120.0F, 160.0F, 56, 0.31F);
    const auto normal_cross_class = yolo::postprocess(
        normal_cross_class_output.data(),
        shape,
        info,
        0.25F,
        {},
        0.10F,
        8,
        0.45F,
        640);
    assert(normal_cross_class.size() == 2U);
    assert(!normal_cross_class[0].low_confidence_candidate);
    assert(!normal_cross_class[1].low_confidence_candidate);

    std::vector<float> weak_seg_output(static_cast<std::size_t>(kSegChannels * kNumBoxes), 0.0F);
    set_seg_box(weak_seg_output, 0, 460.0F, 240.0F, 80.0F, 100.0F, 28, 0.18F, 10.0F);
    const auto weak_seg_detections = yolo::postprocess_seg(
        weak_seg_output.data(),
        seg_shape,
        proto.data(),
        proto_shape,
        info,
        0.25F,
        {},
        0.10F,
        8,
        0.45F,
        0.5F,
        640);
    assert(weak_seg_detections.size() == 1U);
    assert(weak_seg_detections[0].low_confidence_candidate);
    assert(cv::countNonZero(weak_seg_detections[0].mask) > 0);

    const yolo::ModelSpec& active_spec = yolo::active12_seg_model_spec();
    assert(active_spec.input_width == 896 && active_spec.input_height == 896);
    assert(active_spec.num_classes == 12);
    assert(active_spec.mask_coefficient_offset == 16);
    assert(active_spec.expected_detection_channels == 48);
    assert(active_spec.expected_num_boxes == kActiveNumBoxes);
    assert(active_spec.expected_proto_height == kActiveMaskHeight);
    assert(active_spec.expected_proto_width == kActiveMaskWidth);
    assert(active_spec.max_detections == 300);
    assert((active_spec.local_to_global_class_ids ==
            std::vector<int>{80, 81, 82, 83, 84, 85, 86, 89, 90, 91, 92, 93}));

    yolo::LetterboxInfo active_info;
    active_info.scale = 1.0F;
    active_info.pad_left = 0;
    active_info.pad_top = 0;
    active_info.new_width = 896;
    active_info.new_height = 896;
    active_info.original_width = 896;
    active_info.original_height = 896;
    const std::vector<int64_t> active_shape = {
        1, kActiveChannels, kActiveNumBoxes};
    const std::vector<int64_t> active_proto_shape = {
        1, kMaskChannels, kActiveMaskHeight, kActiveMaskWidth};
    std::vector<float> active_output(
        static_cast<std::size_t>(kActiveChannels * kActiveNumBoxes),
        0.0F);
    std::vector<float> active_proto(
        static_cast<std::size_t>(
            kMaskChannels * kActiveMaskHeight * kActiveMaskWidth),
        0.0F);
    std::fill(
        active_proto.begin(),
        active_proto.begin() + (kActiveMaskHeight * kActiveMaskWidth),
        1.0F);

    // Pure IoU NMS (without legacy containment suppression): the nested rock
    // has IoU 0.25 at a 0.70 threshold, so both same-class candidates survive.
    set_active_seg_box(
        active_output, 0, 300.0F, 300.0F, 200.0F, 200.0F, 1, 0.90F, 10.0F);
    set_active_seg_box(
        active_output, 1, 300.0F, 300.0F, 100.0F, 100.0F, 1, 0.80F, 10.0F);
    // An exactly overlapping heavy-machinery candidate is a different class
    // and must also survive class-aware NMS.
    set_active_seg_box(
        active_output, 2, 300.0F, 300.0F, 200.0F, 200.0F, 7, 0.85F, 10.0F);
    // Same-class exact overlap is suppressed by pure IoU NMS.
    set_active_seg_box(
        active_output, 3, 300.0F, 300.0F, 200.0F, 200.0F, 1, 0.70F, 10.0F);
    const auto active_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        active_info,
        active_spec,
        0.25F,
        0.70F,
        0.50F,
        896);
    assert(active_detections.size() == 3U);
    assert(active_detections[0].class_id == 81);
    assert(active_detections[0].original_class_id == 1);
    assert(active_detections[0].source == yolo::DetectionSource::active12);
    assert(active_detections[0].class_name == "rock");
    assert(!active_detections[0].low_confidence_candidate);
    assert(active_detections[0].mask.size() ==
           active_detections[0].bbox.size());
    assert(active_detections[0].full_image_mask.empty());
    assert(active_detections[1].class_id == 89);
    assert(active_detections[1].full_image_mask.empty());
    assert(active_detections[2].class_id == 81);
    assert(active_detections[2].full_image_mask.empty());

    // Only map-like traversable class 92 retains a full-image canvas. This
    // keeps the 300-detection upper bound from allocating hundreds of MiB.
    std::fill(active_output.begin(), active_output.end(), 0.0F);
    set_active_seg_box(
        active_output, 0, 280.0F, 448.0F, 300.0F, 700.0F, 10, 0.90F, 10.0F);
    set_active_seg_box(
        active_output, 1, 616.0F, 448.0F, 300.0F, 700.0F, 10, 0.80F, 10.0F);
    const auto traversable_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        active_info,
        active_spec,
        0.25F,
        0.70F,
        0.50F,
        896);
    assert(traversable_detections.size() == 2U);
    assert(traversable_detections[0].class_id == 92);
    assert(traversable_detections[0].full_image_mask.size() ==
           cv::Size(896, 896));
    assert(traversable_detections[1].class_id == 92);
    assert(traversable_detections[1].full_image_mask.empty());

    // NMS must see the un-clipped 896-letterbox boxes. These same-class boxes
    // have raw IoU 0.50, so both survive the sealed 0.70 threshold. Clipping
    // them to the left image edge before NMS makes them identical (IoU 1.0)
    // and would incorrectly discard the lower-confidence candidate.
    std::fill(active_output.begin(), active_output.end(), 0.0F);
    set_active_seg_box(
        active_output, 0, 0.0F, 200.0F, 200.0F, 200.0F, 1, 0.90F, 10.0F);
    set_active_seg_box(
        active_output, 1, 50.0F, 200.0F, 100.0F, 200.0F, 1, 0.80F, 10.0F);
    const auto edge_nms_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        active_info,
        active_spec,
        0.25F,
        0.70F,
        0.50F,
        896);
    assert(edge_nms_detections.size() == 2U);
    assert(edge_nms_detections[0].bbox == cv::Rect(0, 100, 100, 200));
    assert(edge_nms_detections[1].bbox == cv::Rect(0, 100, 100, 200));

    // Padding-anchor guard (2026-10-10). The camera frame is 1280x800: resized to 896x560 and
    // centred, with 168 rows of grey above and below. Anchor centres are (column + 0.5,
    // row + 0.5) * stride; level offsets are 0 (stride 8, 112 wide), 12544 (stride 16, 56 wide)
    // and 15680 (stride 32, 28 wide).
    yolo::LetterboxInfo padded_info;
    padded_info.scale = 0.7F;
    padded_info.pad_left = 0;
    padded_info.pad_top = 168;
    padded_info.new_width = 896;
    padded_info.new_height = 560;
    padded_info.original_width = 1280;
    padded_info.original_height = 800;
    std::fill(active_output.begin(), active_output.end(), 0.0F);
    // In the top bar, the stride-32 anchor the INT8 engine was seen to fire at (row 2, column
    // 14, centre y 80): a wide traversable_area box that reaches into the image.
    set_active_seg_box(
        active_output, 15680 + (2 * 28) + 14, 464.0F, 140.0F, 800.0F, 120.0F, 10, 0.60F, 10.0F);
    // In the image: stride 8, row 50, column 50 (centre 404, 404).
    set_active_seg_box(
        active_output, (50 * 112) + 50, 404.0F, 404.0F, 100.0F, 100.0F, 1, 0.90F, 10.0F);
    // The last padding row and the first image row at the top edge (centre y 164 and 172).
    set_active_seg_box(
        active_output, (20 * 112) + 60, 484.0F, 300.0F, 60.0F, 60.0F, 4, 0.80F, 10.0F);
    set_active_seg_box(
        active_output, (21 * 112) + 70, 564.0F, 300.0F, 60.0F, 60.0F, 5, 0.70F, 10.0F);
    // The last image row and the first padding row at the bottom edge (centre y 724 and 732).
    set_active_seg_box(
        active_output, (90 * 112) + 80, 644.0F, 600.0F, 60.0F, 60.0F, 6, 0.65F, 10.0F);
    set_active_seg_box(
        active_output, (91 * 112) + 90, 724.0F, 600.0F, 60.0F, 60.0F, 7, 0.55F, 10.0F);
    const auto run_padded = [&](yolo::Active12PaddingGuard* guard) {
        return yolo::postprocess_seg(
            active_output.data(),
            active_shape,
            active_proto.data(),
            active_proto_shape,
            padded_info,
            active_spec,
            0.25F,
            0.70F,
            0.50F,
            896,
            yolo::Active12MaskImplementation::Serial,
            nullptr,
            nullptr,
            yolo::Active12MaskBinaryImplementation::Legacy,
            false,
            guard);
    };
    // Without the guard all six candidates are decoded, the padding ones included.
    const auto unguarded_detections = run_padded(nullptr);
    assert(unguarded_detections.size() == 6U);
    // With it the three padding anchors are dropped and counted; the image anchors are kept
    // with unchanged values.
    yolo::Active12PaddingGuard padding_guard;
    padding_guard.dropped = 99U;
    const auto guarded_detections = run_padded(&padding_guard);
    assert(padding_guard.dropped == 3U);
    assert(guarded_detections.size() == 3U);
    assert(guarded_detections[0].class_id == 81 && guarded_detections[0].class_name == "rock");
    assert(guarded_detections[1].original_class_id == 5);
    assert(guarded_detections[2].original_class_id == 6);
    for (const auto& kept : guarded_detections) {
        bool found = false;
        for (const auto& before : unguarded_detections) {
            found = found || (before.original_class_id == kept.original_class_id &&
                              before.bbox == kept.bbox && before.confidence == kept.confidence);
        }
        assert(found);
    }
    // A frame without padding loses nothing and reports zero.
    padding_guard.dropped = 99U;
    const auto square_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        active_info,
        active_spec,
        0.25F,
        0.70F,
        0.50F,
        896,
        yolo::Active12MaskImplementation::Serial,
        nullptr,
        nullptr,
        yolo::Active12MaskBinaryImplementation::Legacy,
        false,
        &padding_guard);
    assert(padding_guard.dropped == 0U);
    assert(square_detections.size() == 6U);

    // v8.4.66 crop_mask changes its CPU implementation at 50 detections. The
    // vectorized branch keeps float comparisons, equivalent to ceil() bounds;
    // it must not silently keep the <50 round-to-even crop.
    yolo::LetterboxInfo dense_active_info;
    dense_active_info.scale = 896.0F / 100.0F;
    dense_active_info.pad_left = 0;
    dense_active_info.pad_top = 0;
    dense_active_info.new_width = 896;
    dense_active_info.new_height = 896;
    dense_active_info.original_width = 100;
    dense_active_info.original_height = 100;
    std::fill(active_output.begin(), active_output.end(), 0.0F);
    for (std::int64_t index = 0; index < 50; ++index) {
        const float left = 1.1F + (12.0F * static_cast<float>(index % 8));
        const float top = 1.1F + (12.0F * static_cast<float>(index / 8));
        constexpr float width = 4.0F;
        constexpr float height = 4.0F;
        set_active_seg_box(
            active_output,
            index,
            (left + (width * 0.5F)) * dense_active_info.scale,
            (top + (height * 0.5F)) * dense_active_info.scale,
            width * dense_active_info.scale,
            height * dense_active_info.scale,
            1,
            0.99F - (0.001F * static_cast<float>(index)),
            10.0F);
    }
    const auto dense_crop_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        dense_active_info,
        active_spec,
        0.25F,
        0.70F,
        0.50F,
        896);
    assert(dense_crop_detections.size() == 50U);
    assert(dense_crop_detections[0].bbox == cv::Rect(2, 2, 4, 4));
    assert(dense_crop_detections[0].mask.size() == cv::Size(4, 4));
    assert(cv::countNonZero(dense_crop_detections[0].mask) == 16);

    // Ultralytics 8.4.66 process_mask_native regression. A 641x480 frame gives
    // asymmetric 112/113 input padding. At prototype resolution its sealed
    // scale_masks ROI is [x=0..224, y=28..196): note the last row, which the
    // older float-pad + int()-truncation path drops ([28..195)).
    yolo::LetterboxInfo odd_active_info;
    odd_active_info.scale = 896.0F / 641.0F;
    odd_active_info.pad_left = 0;
    odd_active_info.pad_top = 112;
    odd_active_info.new_width = 896;
    odd_active_info.new_height = 671;
    odd_active_info.original_width = 641;
    odd_active_info.original_height = 480;

    std::fill(active_output.begin(), active_output.end(), 0.0F);
    set_active_seg_box(
        active_output,
        0,
        448.0F,
        447.5F,
        896.0F,
        671.0F,
        10,
        0.90F,
        1.0F);
    std::fill(active_proto.begin(), active_proto.end(), 0.0F);
    cv::Mat mask_logits(
        kActiveMaskHeight,
        kActiveMaskWidth,
        CV_32F,
        active_proto.data());
    mask_logits.setTo(1.0F);
    const auto probability_070_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        odd_active_info,
        active_spec,
        0.25F,
        0.70F,
        0.70F,
        896);
    const auto probability_075_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        odd_active_info,
        active_spec,
        0.25F,
        0.70F,
        0.75F,
        896);
    const auto probability_zero_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        odd_active_info,
        active_spec,
        0.25F,
        0.70F,
        0.0F,
        896);
    const auto probability_one_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        odd_active_info,
        active_spec,
        0.25F,
        0.70F,
        1.0F,
        896);
    assert(probability_070_detections.size() == 1U);
    assert(probability_075_detections.size() == 1U);
    assert(probability_zero_detections.size() == 1U);
    assert(probability_one_detections.size() == 1U);
    const int full_odd_image_pixels =
        odd_active_info.original_width * odd_active_info.original_height;
    assert(cv::countNonZero(probability_070_detections[0].mask) ==
           full_odd_image_pixels);
    assert(cv::countNonZero(probability_075_detections[0].mask) == 0);
    assert(cv::countNonZero(probability_zero_detections[0].mask) ==
           full_odd_image_pixels);
    assert(cv::countNonZero(probability_one_detections[0].mask) == 0);

    mask_logits.setTo(-8.0F);
    mask_logits(cv::Rect(112, 28, 112, 168)).setTo(1.0F);
    mask_logits.row(195).setTo(8.0F);

    const auto native_mask_detections = yolo::postprocess_seg(
        active_output.data(),
        active_shape,
        active_proto.data(),
        active_proto_shape,
        odd_active_info,
        active_spec,
        0.25F,
        0.70F,
        0.50F,
        896);
    assert(native_mask_detections.size() == 1U);
    assert(native_mask_detections[0].class_id == 92);
    assert(native_mask_detections[0].bbox == cv::Rect(0, 0, 641, 480));
    assert(native_mask_detections[0].mask.size() == cv::Size(641, 480));

    cv::Mat native_restored_logits;
    cv::resize(
        mask_logits(cv::Rect(0, 28, 224, 168)),
        native_restored_logits,
        cv::Size(641, 480),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    cv::Mat native_reference;
    cv::threshold(
        native_restored_logits,
        native_reference,
        0.0,
        255.0,
        cv::THRESH_BINARY);
    native_reference.convertTo(native_reference, CV_8UC1);
    assert(differing_pixels(native_mask_detections[0].mask, native_reference) == 0);
    assert(cv::countNonZero(native_reference) > 0);
    assert(cv::countNonZero(native_reference) < native_reference.rows * native_reference.cols);

    // Explicitly reject the previous C++ behavior: sigmoid before interpolation
    // plus 224->896->original double resampling.
    cv::Mat negative_exponential;
    cv::exp(-mask_logits, negative_exponential);
    cv::Mat mask_probability = 1.0 / (1.0 + negative_exponential);
    cv::Mat legacy_mask_input;
    cv::resize(
        mask_probability,
        legacy_mask_input,
        cv::Size(896, 896),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    cv::Mat legacy_mask_probability;
    cv::resize(
        legacy_mask_input(cv::Rect(0, 112, 896, 671)),
        legacy_mask_probability,
        cv::Size(641, 480),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    cv::Mat legacy_mask_binary;
    cv::threshold(
        legacy_mask_probability,
        legacy_mask_binary,
        0.5,
        255.0,
        cv::THRESH_BINARY);
    legacy_mask_binary.convertTo(legacy_mask_binary, CV_8UC1);
    assert(differing_pixels(native_reference, legacy_mask_binary) > 0);

    // Explicitly reject the pre-8.4.66 float padding + int() crop. For this
    // fixture it produces a 167-row ROI and loses prototype row 195.
    cv::Mat old_int_restored_logits;
    cv::resize(
        mask_logits(cv::Rect(0, 28, 224, 167)),
        old_int_restored_logits,
        cv::Size(641, 480),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    cv::Mat old_int_binary;
    cv::threshold(
        old_int_restored_logits,
        old_int_binary,
        0.0,
        255.0,
        cv::THRESH_BINARY);
    old_int_binary.convertTo(old_int_binary, CV_8UC1);
    assert(differing_pixels(native_reference, old_int_binary) > 0);

    // Sealed mapping and exact output tensor contract fail closed.
    yolo::ModelSpec invalid_mapping = active_spec;
    invalid_mapping.local_to_global_class_ids[0] = 87;
    bool invalid_mapping_threw = false;
    try {
        (void)yolo::postprocess_seg(
            active_output.data(),
            active_shape,
            active_proto.data(),
            active_proto_shape,
            active_info,
            invalid_mapping,
            0.25F,
            0.70F,
            0.50F,
            896);
    } catch (const std::invalid_argument&) {
        invalid_mapping_threw = true;
    }
    assert(invalid_mapping_threw);

    std::vector<int64_t> wrong_active_shape = active_shape;
    wrong_active_shape[1] = 47;
    bool wrong_shape_threw = false;
    try {
        (void)yolo::postprocess_seg(
            active_output.data(),
            wrong_active_shape,
            active_proto.data(),
            active_proto_shape,
            active_info,
            active_spec,
            0.25F,
            0.70F,
            0.50F,
            896);
    } catch (const std::invalid_argument&) {
        wrong_shape_threw = true;
    }
    assert(wrong_shape_threw);

    std::cout << "All postprocess tests passed!" << std::endl;
    return 0;
}
