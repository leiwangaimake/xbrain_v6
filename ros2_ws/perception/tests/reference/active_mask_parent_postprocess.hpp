/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: active_mask_parent_postprocess.hpp
 * Brief: Declarations of the frozen parent YOLO postprocess, the oracle for the active mask tests.
 *
 * Description:
 * Used only by test_active_mask_parallel (namespace yolo_active_mask_parent). Contents:
 *   1. Decodes YOLOv11 [1, 84, 8400] tensors into detections.
 *   2. Restores letterboxed coordinates and applies confidence filtering plus NMS.
 */

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "common/preprocess.hpp"
#include "common/types.hpp"

namespace yolo_active_mask_parent {
using ::yolo::Detection;
using ::yolo::DetectionSource;
using ::yolo::LetterboxInfo;

extern const std::array<const char*, 80> COCO_CLASSES;

/**
 * @brief Sealed tensor and class contract for one segmentation branch.
 *
 * local_to_global_class_ids is deliberately part of the model specification,
 * rather than runtime-tunable state. A mismatch must fail closed before any
 * result is admitted to the shared tracker.
 */
struct ModelSpec {
    DetectionSource source = DetectionSource::coco80;
    int input_size = 0;
    int num_classes = 0;
    int mask_coefficient_count = 0;
    int mask_coefficient_offset = 0;
    int expected_detection_channels = 0;
    std::int64_t expected_num_boxes = 0;
    int expected_proto_height = 0;
    int expected_proto_width = 0;
    int max_detections = 0;
    std::vector<std::string> class_names;
    std::vector<int> local_to_global_class_ids;
};

/**
 * @brief Sealed active12 YOLO11n-seg 896 contract.
 */
const ModelSpec& active12_seg_model_spec();

/**
 * @brief Decode YOLOv11 model output, restore coordinates, and apply greedy NMS.
 */
std::vector<Detection> postprocess(
    const float* output_data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info,
    float conf_threshold,
    float iou_threshold,
    int input_size);

/**
 * @brief Decode YOLOv11 output with an optional generic weak-obstacle path.
 *
 * A low_confidence_floor <= 0 disables the weak path and preserves the legacy
 * result exactly. Weak candidates are marked for tracker/depth gating.
 */
std::vector<Detection> postprocess(
    const float* output_data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info,
    float conf_threshold,
    const std::map<int, float>& per_class_conf,
    float low_confidence_floor,
    int low_confidence_max_candidates,
    float iou_threshold,
    int input_size);

/**
 * @brief Decode YOLOv11 model output with optional class-specific confidence thresholds.
 */
std::vector<Detection> postprocess(
    const float* output_data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info,
    float conf_threshold,
    const std::map<int, float>& per_class_conf,
    float iou_threshold,
    int input_size);

/**
 * @brief Decode YOLOv11-seg outputs, restore boxes, apply NMS, and decode instance masks.
 */
std::vector<Detection> postprocess_seg(
    const float* detection_data,
    const std::vector<int64_t>& detection_shape,
    const float* prototype_data,
    const std::vector<int64_t>& prototype_shape,
    const LetterboxInfo& info,
    float conf_threshold,
    float iou_threshold,
    float mask_threshold,
    int input_size);

/**
 * @brief Decode YOLOv11-seg outputs with optional class-specific confidence thresholds.
 */
std::vector<Detection> postprocess_seg(
    const float* detection_data,
    const std::vector<int64_t>& detection_shape,
    const float* prototype_data,
    const std::vector<int64_t>& prototype_shape,
    const LetterboxInfo& info,
    float conf_threshold,
    const std::map<int, float>& per_class_conf,
    float iou_threshold,
    float mask_threshold,
    int input_size);

/**
 * @brief Decode YOLOv11-seg output with an optional generic weak-obstacle path.
 */
std::vector<Detection> postprocess_seg(
    const float* detection_data,
    const std::vector<int64_t>& detection_shape,
    const float* prototype_data,
    const std::vector<int64_t>& prototype_shape,
    const LetterboxInfo& info,
    float conf_threshold,
    const std::map<int, float>& per_class_conf,
    float low_confidence_floor,
    int low_confidence_max_candidates,
    float iou_threshold,
    float mask_threshold,
    int input_size);

/**
 * @brief Decode a segmentation branch described by a sealed ModelSpec.
 *
 * This path uses pure class-aware IoU NMS and never admits weak detections.
 * It produces both the legacy bbox-local mask and an original-image mask for
 * downstream dual-model fusion.
 */
std::vector<Detection> postprocess_seg(
    const float* detection_data,
    const std::vector<int64_t>& detection_shape,
    const float* prototype_data,
    const std::vector<int64_t>& prototype_shape,
    const LetterboxInfo& info,
    const ModelSpec& model_spec,
    float conf_threshold,
    float iou_threshold,
    float mask_threshold,
    int input_size);

}  // namespace yolo
