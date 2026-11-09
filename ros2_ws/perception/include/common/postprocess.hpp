/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: postprocess.hpp
 * Brief: YOLO11-seg output decoding for the two inference branches: boxes, classes, instance masks.
 *
 * Description:
 * What problem this solves. The engines return raw tensors: per anchor a box, class scores and 32
 * mask coefficients, plus 32 prototype masks. This file turns them into Detections in original
 * image coordinates, each with a bbox-local CV_8UC1 mask (the contract DepthSampler and the
 * tracker read). Two decoders, one per branch:
 *   - COCO80 (the postprocess_seg overload with low_confidence_floor): per-class thresholds and the
 *     generic weak-obstacle tier, NMS with containment; masks from sigmoid probabilities.
 *   - Active12 (the ModelSpec overload): the sealed 12-class contract, reproducing Ultralytics
 *     8.4.66 bit for bit (pure class-aware NMS, logits resized straight to the original image,
 *     threshold at the logit of mask_threshold); the first class-92 instance also carries an
 *     original-image mask for the traversable-area union (dual_model_merge).
 *
 * Boundaries. No merging of the branches (dual_model_merge), no tracking, no depth.
 *
 * Pitfalls. The PSC-4 golden digest depends on every bit of these outputs, so the fast paths
 * (ROI resizes, the column-limited logits kernel, the exact bbox resize) are only taken where
 * they provably give the same values, and fall back otherwise. The bbox resize plan is calibrated
 * per OpenCV thread count; prewarm_active12_mask_kernels does that at startup, otherwise the first
 * decode under a new count pays for the calibration.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "common/preprocess.hpp"
#include "common/types.hpp"
#include "common/execution_cpu_trace.hpp"

namespace perception {
class PersistentTaskPair;
}

namespace yolo {

extern const std::array<const char*, 80> COCO_CLASSES;

// Instance mask decoding of Active12: Serial (one thread), Parallel2 (two dedicated lanes, same
// output and order), Shadow (diagnostic: both, compared, Serial returned).
enum class Active12MaskImplementation { Serial, Parallel2, Shadow };
// Binarisation: Legacy (threshold then convertTo), Compare (one cv::compare, the same bytes),
// Shadow (diagnostic: both, compared, Legacy returned).
enum class Active12MaskBinaryImplementation { Legacy, Compare, Shadow };

Active12MaskBinaryImplementation parse_active12_mask_binary_implementation(const std::string& value);
const char* active12_mask_binary_implementation_name(Active12MaskBinaryImplementation implementation);

// Per-instance stage times of one Active12 mask decode (inference_profile only).
struct Active12MaskTaskProfile {
    perception::ExecutionCpuRecord execution_cpu;
    std::size_t index = 0;
    std::int64_t bbox_pixels = 0;
    std::int64_t gemm_ns = 0;
    std::int64_t resize_ns = 0;
    std::int64_t threshold_convert_ns = 0;
    std::int64_t canvas_ns = 0;
    std::int64_t total_ns = 0;
    int lane = 0;
};

struct Active12MaskProfile {
    std::size_t instances = 0;
    // Host wall durations: task stages sum to task total. Task totals are
    // nested inside batch_wall_ns and total_ns, not additive to either.
    std::int64_t batch_wall_ns = 0;
    std::int64_t total_ns = 0;
    bool parallel_used = false;
    bool shadow_checked = false;
    // Independent of Serial/Parallel2 shadow: only Legacy output is returned.
    bool binary_shadow_checked = false;
    std::vector<Active12MaskTaskProfile> tasks;
};

/**
 * @brief Padding-anchor guard of the Active12 decoder (2026-10-10).
 *
 * The model input is the camera image resized and centred, with grey bars filling the rest
 * (letterbox). An anchor whose centre lies in those bars has no image content under it; the FP16
 * model never reports from there. An INT8 engine with quantized first layers does: it reports
 * traversable_area at a few fixed padding anchors (measured 2026-10-10: 232 such predictions in
 * the 579 validation images, none of them a true positive, against 0 for the FP16 engine). With
 * a guard object the decoder drops every candidate above the threshold whose anchor centre is in
 * the padding, before NMS, and writes how many it dropped. nullptr leaves the decoder unchanged.
 */
struct Active12PaddingGuard {
    std::size_t dropped = 0;
};

/**
 * @brief Sealed tensor and class contract for one segmentation branch.
 *
 * local_to_global_class_ids is deliberately part of the model specification,
 * rather than runtime-tunable state. A mismatch must fail closed before any
 * result is admitted to the shared tracker.
 */
struct ModelSpec {
    // Detection channels are [cx, cy, w, h, class scores..., mask coefficients...], so
    // mask_coefficient_offset = 4 + num_classes; global ids follow config/class_schema_94.json.
    DetectionSource source = DetectionSource::coco80;
    // Model input [1, 3, input_height, input_width]. It is rows [input_window_top,
    // input_window_top + input_height) of the input_width x input_width letterbox the image is
    // centred in; a square input is the whole letterbox (0).
    int input_width = 0;
    int input_height = 0;
    int input_window_top = 0;
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
 * @brief Sealed active12 YOLO11n-seg contract of the square 896 x 896 input.
 */
const ModelSpec& active12_seg_model_spec();

/**
 * @brief Sealed active12 contract of one input shape: 896 x 896, or 896 wide and 608 high.
 *
 * Same network, weights and classes; only the anchor count and the prototype size follow the
 * input. The 608-high input is rows 160..767 of the square letterbox: the 1280 x 800 camera
 * image keeps 8 padding rows above and 40 below instead of 168 on either side. 160 is a multiple
 * of every stride, so each anchor cell sees the same image patch as in the square input, and the
 * 40 rows below are needed (measured 2026-10-10 on the Orin: with 8 rows below, low-score
 * targets at the lower image edge are lost; with the content half a stride-32 cell off, more).
 * Throws std::invalid_argument for any other shape: an input shape is usable only with its
 * sealed counts.
 */
const ModelSpec& active12_seg_model_spec(ModelInputShape input_shape);

// The detection-only postprocess() overloads and the two shorter postprocess_seg() overloads are
// thin wrappers (no per-class thresholds, weak tier off) kept for tests and tools; the service
// calls the low_confidence_floor postprocess_seg (COCO80) and the ModelSpec one (Active12).

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

// Work counters of the last postprocess_seg() call on the calling thread (2026-10-16, Fable question
// 13.2: which work grows when the Objects post time does). Diagnostic only: read right after the
// call, on the same thread; another call on that thread replaces them, a call that throws leaves
// them zero. candidates: anchors above the confidence gate, before NMS; selected: kept after NMS.
// The COCO80 overload also counts the mask path each kept instance took: roi2 (only the prototype
// rows its resizes read), roi (the whole prototype plane and the integer-scale ROI resize) and full
// (the whole plane and the full-frame resize, the fallback); an instance with an empty box or ROI
// is in none of them. The Active12 overloads set candidates and selected only.
struct PostWorkCounts {
    std::uint32_t candidates = 0, selected = 0, mask_roi2 = 0, mask_roi = 0, mask_full = 0;
};
PostWorkCounts last_post_work_counts() noexcept;

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
 * input_shape must be the model_spec's own input (the letterbox in info was made for it); a
 * mismatch throws std::invalid_argument. For an input that is a window of the square letterbox
 * the letterbox must be the window's, and the image content must lie inside the window.
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
    ModelInputShape input_shape,
    Active12MaskImplementation implementation = Active12MaskImplementation::Serial,
    perception::PersistentTaskPair* mask_lanes = nullptr,
    Active12MaskProfile* profile = nullptr,
    Active12MaskBinaryImplementation binary_implementation = Active12MaskBinaryImplementation::Legacy,
    bool execution_cpu_trace = false,
    Active12PaddingGuard* padding_guard = nullptr);

/**
 * @brief The same decoder for a square input: input_size x input_size.
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
    int input_size,
    Active12MaskImplementation implementation = Active12MaskImplementation::Serial,
    perception::PersistentTaskPair* mask_lanes = nullptr,
    Active12MaskProfile* profile = nullptr,
    Active12MaskBinaryImplementation binary_implementation = Active12MaskBinaryImplementation::Legacy,
    bool execution_cpu_trace = false,
    Active12PaddingGuard* padding_guard = nullptr);

/**
 * @brief Runs the Active12 mask kernels' one-time calibrations (prototype-rectangle logits and the
 * bbox resize for this prototype and image size) outside the realtime path; the first decode
 * would otherwise pay for them. Sizes: prototype width/height and the original image size.
 * The resize is calibrated for OpenCV thread counts 1 and 2 (and the current one), so this call
 * sets the process-wide count temporarily and restores it: call it only while no other thread
 * uses OpenCV (InferenceModule::start(), before its worker and before the capture starts).
 */
void prewarm_active12_mask_kernels(int proto_width, int proto_height, int original_width, int original_height);

}  // namespace yolo
