/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: active_mask_parent_postprocess.cpp
 * Brief: Frozen copy of the parent YOLO postprocess, the oracle for the active mask tests.
 *
 * Description:
 * Built only into test_active_mask_parallel (namespace yolo_active_mask_parent), which compares
 * every mode of the production postprocess against it. Not production code. Contents:
 *   1. Parses channel-major YOLOv11 tensors and maps COCO class names.
 *   2. Applies coordinate restoration, clipping, confidence filtering, and NMS.
 */

#include "active_mask_parent_postprocess.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/logger.hpp"

namespace yolo_active_mask_parent {

const std::array<const char*, 80> COCO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"};

const ModelSpec& active12_seg_model_spec() {
    static const ModelSpec spec{
        DetectionSource::active12,
        896,
        12,
        32,
        16,
        48,
        16464,
        224,
        224,
        300,
        {
            "tree_trunk",
            "rock",
            "dirt_pile",
            "bush",
            "pole",
            "pillar",
            "pallet",
            "heavy_machinery",
            "traffic_cone",
            "barrier",
            "traversable_area",
            "pit",
        },
        {80, 81, 82, 83, 84, 85, 86, 89, 90, 91, 92, 93},
    };
    return spec;
}

namespace {

enum class CocoMaskResizeMode {
    full,
    roi,
};

CocoMaskResizeMode coco_mask_resize_mode() {
    static const CocoMaskResizeMode mode = []() {
        const char* value = std::getenv("PERCEPTION_COCO_MASK_RESIZE");
        if (value == nullptr) {
            return CocoMaskResizeMode::roi;
        }
        const std::string text(value);
        if (text == "roi" || text == "1") {
            return CocoMaskResizeMode::roi;
        }
        return CocoMaskResizeMode::full;
    }();
    return mode;
}

bool resize_linear_integer_scale_roi(
    const cv::Mat& source,
    const cv::Size& full_output_size,
    const cv::Rect& output_roi,
    cv::Mat& output) {
    constexpr int kMinimumOptimizedDimension = 32;
    constexpr std::int64_t kMinimumOptimizedArea = 4096;
    output.release();
    if (source.empty() || source.type() != CV_32FC1 ||
        full_output_size.width <= 0 || full_output_size.height <= 0 ||
        output_roi.empty() ||
        output_roi.width < kMinimumOptimizedDimension ||
        output_roi.height < kMinimumOptimizedDimension ||
        static_cast<std::int64_t>(output_roi.width) * output_roi.height <
            kMinimumOptimizedArea ||
        output_roi.x < 0 || output_roi.y < 0 ||
        output_roi.x + output_roi.width > full_output_size.width ||
        output_roi.y + output_roi.height > full_output_size.height ||
        full_output_size.width % source.cols != 0 ||
        full_output_size.height % source.rows != 0) {
        return false;
    }

    const int scale_x = full_output_size.width / source.cols;
    const int scale_y = full_output_size.height / source.rows;
    if (scale_x <= 0 || scale_x != scale_y) {
        return false;
    }
    const int scale = scale_x;

    // Keep a one-source-pixel halo so the cropped resize has the same linear
    // interpolation neighbours as the full-image resize. The local crop is
    // phase-aligned to the integer output scale before extracting output_roi.
    const int source_left = std::max(0, (output_roi.x / scale) - 1);
    const int source_top = std::max(0, (output_roi.y / scale) - 1);
    const int output_right = output_roi.x + output_roi.width;
    const int output_bottom = output_roi.y + output_roi.height;
    const int source_right = std::min(
        source.cols,
        ((output_right + scale - 1) / scale) + 1);
    const int source_bottom = std::min(
        source.rows,
        ((output_bottom + scale - 1) / scale) + 1);
    const cv::Rect source_roi(
        source_left,
        source_top,
        source_right - source_left,
        source_bottom - source_top);
    if (source_roi.empty()) {
        return false;
    }

    cv::Mat enlarged;
    cv::resize(
        source(source_roi),
        enlarged,
        cv::Size(source_roi.width * scale, source_roi.height * scale),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    const cv::Rect local_roi(
        output_roi.x - (source_left * scale),
        output_roi.y - (source_top * scale),
        output_roi.width,
        output_roi.height);
    if (local_roi.x < 0 || local_roi.y < 0 ||
        local_roi.x + local_roi.width > enlarged.cols ||
        local_roi.y + local_roi.height > enlarged.rows) {
        return false;
    }
    output = enlarged(local_roi);
    return true;
}

struct Candidate {
    cv::Rect2f bbox;
    float conf = 0.0F;
    int class_id = -1;
    int original_class_id = -1;
    std::int64_t source_index = -1;
    std::array<float, 32> mask_coef {};
    bool low_confidence_candidate = false;
};

float intersection_over_union(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.width, b.x + b.width);
    const float y2 = std::min(a.y + a.height, b.y + b.height);
    const float inter_w = std::max(0.0F, x2 - x1);
    const float inter_h = std::max(0.0F, y2 - y1);
    const float inter_area = inter_w * inter_h;
    const float union_area = a.area() + b.area() - inter_area;
    if (union_area <= 0.0F) {
        return 0.0F;
    }
    return inter_area / union_area;
}

float intersection_over_smaller_area(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.width, b.x + b.width);
    const float y2 = std::min(a.y + a.height, b.y + b.height);
    const float inter_w = std::max(0.0F, x2 - x1);
    const float inter_h = std::max(0.0F, y2 - y1);
    const float smaller_area = std::min(a.area(), b.area());
    if (smaller_area <= 0.0F) {
        return 0.0F;
    }
    return (inter_w * inter_h) / smaller_area;
}

bool overlaps_as_duplicate(const Candidate& candidate, const Candidate& kept, float iou_threshold) {
    constexpr float CONTAINMENT_THRESHOLD = 0.90F;
    return intersection_over_union(candidate.bbox, kept.bbox) > iou_threshold ||
           intersection_over_smaller_area(candidate.bbox, kept.bbox) >= CONTAINMENT_THRESHOLD;
}

bool should_suppress(const Candidate& candidate, const Candidate& kept, float iou_threshold) {
    // Legacy normal candidates remain class-aware. A weak candidate is only a
    // generic obstacle proxy, so overlap suppression must ignore its unstable
    // semantic label when compared with either normal or weak candidates.
    if (!candidate.low_confidence_candidate && !kept.low_confidence_candidate &&
        candidate.class_id != kept.class_id) {
        return false;
    }
    return overlaps_as_duplicate(candidate, kept, iou_threshold);
}

int rounded_non_negative(float value) {
    return std::max(0, static_cast<int>(std::round(value)));
}

int python_round_non_negative(double value) {
    if (!std::isfinite(value) || value <= 0.0) {
        return 0;
    }

    const double lower = std::floor(value);
    const double fraction = value - lower;
    if (fraction < 0.5) {
        return static_cast<int>(lower);
    }
    if (fraction > 0.5) {
        return static_cast<int>(lower + 1.0);
    }

    // Python and torch round exact half-way values to the nearest even integer.
    const auto lower_integer = static_cast<std::int64_t>(lower);
    return static_cast<int>((lower_integer % 2 == 0) ? lower_integer : lower_integer + 1);
}

cv::Rect rect_within_image(const cv::Rect& rect, int width, int height) {
    if (width <= 0 || height <= 0) {
        return cv::Rect();
    }
    return rect & cv::Rect(0, 0, width, height);
}

cv::Rect to_int_rect(const cv::Rect2f& rect) {
    const int x = rounded_non_negative(rect.x);
    const int y = rounded_non_negative(rect.y);
    const int right = rounded_non_negative(rect.x + rect.width);
    const int bottom = rounded_non_negative(rect.y + rect.height);
    return cv::Rect(x, y, std::max(0, right - x), std::max(0, bottom - y));
}

cv::Rect to_ultralytics_cpu_crop_rect(
    const cv::Rect2f& rect,
    std::size_t detection_count) {
    const auto convert = [detection_count](double value) {
        const double non_negative = std::max(0.0, value);
        if (detection_count < 50U) {
            // Ultralytics 8.4.66 CPU prediction loop:
            // boxes.clamp(min=0).round().int().
            return python_round_non_negative(non_negative);
        }
        // Its >=50 vectorized crop_mask branch retains the float inequalities
        // r >= x1 and r < x2, which are equivalent to ceil() pixel bounds.
        return static_cast<int>(std::ceil(non_negative));
    };
    const int x = convert(static_cast<double>(rect.x));
    const int y = convert(static_cast<double>(rect.y));
    const int right =
        convert(static_cast<double>(rect.x) + static_cast<double>(rect.width));
    const int bottom =
        convert(static_cast<double>(rect.y) + static_cast<double>(rect.height));
    return cv::Rect(x, y, std::max(0, right - x), std::max(0, bottom - y));
}

cv::Rect ultralytics_v8466_native_mask_roi(
    int prototype_width,
    int prototype_height,
    int original_width,
    int original_height) {
    if (prototype_width <= 0 || prototype_height <= 0 ||
        original_width <= 0 || original_height <= 0) {
        return cv::Rect();
    }

    // Ultralytics 8.4.66 scale_masks(..., padding=True): recompute the
    // letterbox padding at prototype resolution, crop logits once, then resize
    // directly to the original image. The round +/- 0.1 convention is
    // deliberately not the older int()-truncation implementation.
    const double gain = std::min(
        static_cast<double>(prototype_height) / static_cast<double>(original_height),
        static_cast<double>(prototype_width) / static_cast<double>(original_width));
    const double pad_width =
        static_cast<double>(
            prototype_width -
            python_round_non_negative(static_cast<double>(original_width) * gain)) /
        2.0;
    const double pad_height =
        static_cast<double>(
            prototype_height -
            python_round_non_negative(static_cast<double>(original_height) * gain)) /
        2.0;

    const int top = python_round_non_negative(pad_height - 0.1);
    const int left = python_round_non_negative(pad_width - 0.1);
    const int bottom =
        prototype_height - python_round_non_negative(pad_height + 0.1);
    const int right =
        prototype_width - python_round_non_negative(pad_width + 0.1);
    if (right <= left || bottom <= top) {
        return cv::Rect();
    }
    return rect_within_image(
        cv::Rect(left, top, right - left, bottom - top),
        prototype_width,
        prototype_height);
}

float probability_to_logit_threshold(float probability) {
    if (probability == 0.5F) {
        // Keep the sealed active12 threshold exactly equivalent to logits > 0.
        return 0.0F;
    }
    return static_cast<float>(
        std::log(static_cast<double>(probability)) -
        std::log1p(-static_cast<double>(probability)));
}

cv::Mat make_full_image_mask(
    const cv::Mat& bbox_local_mask,
    const cv::Rect& bbox,
    int image_width,
    int image_height) {
    cv::Mat full_image_mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);
    const cv::Rect clipped = rect_within_image(bbox, image_width, image_height);
    if (bbox_local_mask.empty() || clipped.empty()) {
        return full_image_mask;
    }

    if (clipped == bbox && bbox_local_mask.size() == bbox.size()) {
        bbox_local_mask.copyTo(full_image_mask(clipped));
        return full_image_mask;
    }

    const cv::Rect local_roi(
        clipped.x - bbox.x,
        clipped.y - bbox.y,
        clipped.width,
        clipped.height);
    if (local_roi.x >= 0 && local_roi.y >= 0 &&
        local_roi.x + local_roi.width <= bbox_local_mask.cols &&
        local_roi.y + local_roi.height <= bbox_local_mask.rows) {
        bbox_local_mask(local_roi).copyTo(full_image_mask(clipped));
    }
    return full_image_mask;
}

void validate_model_spec(const ModelSpec& spec) {
    if (spec.input_size <= 0 || spec.num_classes <= 0 || spec.mask_coefficient_count <= 0 ||
        spec.mask_coefficient_count > 32 || spec.mask_coefficient_offset < 4 ||
        spec.expected_detection_channels <= 0 || spec.expected_num_boxes <= 0 ||
        spec.expected_proto_height <= 0 || spec.expected_proto_width <= 0 ||
        spec.max_detections <= 0) {
        throw std::invalid_argument("invalid segmentation ModelSpec dimensions");
    }
    if (spec.mask_coefficient_offset != 4 + spec.num_classes ||
        spec.expected_detection_channels !=
            spec.mask_coefficient_offset + spec.mask_coefficient_count) {
        throw std::invalid_argument("ModelSpec channel offsets do not match class/mask counts");
    }
    if (spec.class_names.size() != static_cast<std::size_t>(spec.num_classes) ||
        spec.local_to_global_class_ids.size() !=
            static_cast<std::size_t>(spec.num_classes)) {
        throw std::invalid_argument("ModelSpec class names/mapping size mismatch");
    }
    for (const int global_id : spec.local_to_global_class_ids) {
        if (global_id < 0 || global_id > 93 || global_id == 87 || global_id == 88) {
            throw std::invalid_argument("ModelSpec maps to invalid or reserved global class ID");
        }
    }

    if (spec.source == DetectionSource::active12) {
        const ModelSpec& sealed = active12_seg_model_spec();
        if (spec.input_size != sealed.input_size ||
            spec.num_classes != sealed.num_classes ||
            spec.mask_coefficient_count != sealed.mask_coefficient_count ||
            spec.mask_coefficient_offset != sealed.mask_coefficient_offset ||
            spec.expected_detection_channels != sealed.expected_detection_channels ||
            spec.expected_num_boxes != sealed.expected_num_boxes ||
            spec.expected_proto_height != sealed.expected_proto_height ||
            spec.expected_proto_width != sealed.expected_proto_width ||
            spec.max_detections != sealed.max_detections ||
            spec.class_names != sealed.class_names ||
            spec.local_to_global_class_ids != sealed.local_to_global_class_ids) {
            throw std::invalid_argument("active12 ModelSpec differs from sealed deployment contract");
        }
    }
}

std::vector<Candidate> select_pure_class_aware_candidates(
    std::vector<Candidate> candidates,
    float iou_threshold,
    int max_detections) {
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& lhs, const Candidate& rhs) {
            if (lhs.conf != rhs.conf) {
                return lhs.conf > rhs.conf;
            }
            return lhs.source_index < rhs.source_index;
        });

    std::vector<Candidate> selected;
    selected.reserve(
        std::min(candidates.size(), static_cast<std::size_t>(max_detections)));
    for (const Candidate& candidate : candidates) {
        bool suppressed = false;
        for (const Candidate& kept : selected) {
            if (candidate.class_id == kept.class_id &&
                intersection_over_union(candidate.bbox, kept.bbox) > iou_threshold) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }
        selected.push_back(candidate);
        if (selected.size() >= static_cast<std::size_t>(max_detections)) {
            break;
        }
    }
    return selected;
}

void select_raw_top_class(
    const float* class_data,
    int64_t num_boxes,
    int64_t box_idx,
    int& selected_class_id,
    float& selected_conf) {
    selected_class_id = 0;
    selected_conf = class_data[box_idx];
    for (int class_id = 1; class_id < 80; ++class_id) {
        const float class_conf = class_data[(class_id * num_boxes) + box_idx];
        if (class_conf > selected_conf) {
            selected_class_id = class_id;
            selected_conf = class_conf;
        }
    }
}

float threshold_for_class(
    float global_threshold,
    const std::map<int, float>& per_class_conf,
    int class_id) {
    const auto threshold = per_class_conf.find(class_id);
    return threshold == per_class_conf.end() ? global_threshold : threshold->second;
}

bool is_weak_fallback_forbidden_class(int class_id) {
    constexpr int PERSON_CLASS_ID = 0;
    return class_id == PERSON_CLASS_ID;
}

enum class CandidateTier {
    rejected,
    normal,
    weak,
};

CandidateTier select_candidate_class(
    const float* class_data,
    int64_t num_boxes,
    int64_t box_idx,
    float global_threshold,
    const std::map<int, float>& per_class_conf,
    float low_confidence_floor,
    int& selected_class_id,
    float& selected_conf) {
    int raw_top_class_id = -1;
    float raw_top_conf = 0.0F;
    select_raw_top_class(
        class_data,
        num_boxes,
        box_idx,
        raw_top_class_id,
        raw_top_conf);
    const float raw_top_threshold =
        threshold_for_class(global_threshold, per_class_conf, raw_top_class_id);

    if (raw_top_conf >= raw_top_threshold) {
        selected_class_id = raw_top_class_id;
        selected_conf = raw_top_conf;
        return CandidateTier::normal;
    }

    if (low_confidence_floor <= 0.0F ||
        is_weak_fallback_forbidden_class(raw_top_class_id)) {
        // Compatibility and safety gate: only the raw top class may enter the
        // normal path. A failed raw-top person anchor cannot be relabeled weak.
        // Other class overrides remain normal-tier thresholds only; when their
        // raw-top confidence clears the weak floor they may degrade to the
        // existing generic-obstacle tier without changing semantic thresholds.
        return CandidateTier::rejected;
    }

    selected_class_id = raw_top_class_id;
    selected_conf = raw_top_conf;
    return raw_top_conf >= low_confidence_floor ? CandidateTier::weak : CandidateTier::rejected;
}

std::vector<Candidate> select_candidates(
    std::vector<Candidate> candidates,
    float iou_threshold,
    int low_confidence_max_candidates) {
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.low_confidence_candidate != rhs.low_confidence_candidate) {
            return !lhs.low_confidence_candidate;
        }
        return lhs.conf > rhs.conf;
    });

    std::vector<Candidate> selected;
    selected.reserve(candidates.size());
    int weak_count = 0;
    const int weak_limit = std::max(0, low_confidence_max_candidates);
    for (const Candidate& candidate : candidates) {
        bool suppressed = false;
        for (const Candidate& kept : selected) {
            if (should_suppress(candidate, kept, iou_threshold)) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }
        if (candidate.low_confidence_candidate) {
            if (weak_count >= weak_limit) {
                continue;
            }
            ++weak_count;
        }
        selected.push_back(candidate);
    }
    return selected;
}

}  // namespace

std::vector<Detection> postprocess(
    const float* output_data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info,
    float conf_threshold,
    float iou_threshold,
    int input_size) {
    return postprocess(output_data, shape, info, conf_threshold, {}, iou_threshold, input_size);
}

std::vector<Detection> postprocess(
    const float* output_data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info,
    float conf_threshold,
    const std::map<int, float>& per_class_conf,
    float iou_threshold,
    int input_size) {
    return postprocess(
        output_data,
        shape,
        info,
        conf_threshold,
        per_class_conf,
        0.0F,
        8,
        iou_threshold,
        input_size);
}

std::vector<Detection> postprocess(
    const float* output_data,
    const std::vector<int64_t>& shape,
    const LetterboxInfo& info,
    float conf_threshold,
    const std::map<int, float>& per_class_conf,
    float low_confidence_floor,
    int low_confidence_max_candidates,
    float iou_threshold,
    int input_size) {
    if (output_data == nullptr) {
        throw std::invalid_argument("postprocess received null output_data");
    }
    if (shape.size() != 3 || shape[0] != 1 || shape[1] != 84 || shape[2] <= 0) {
        throw std::invalid_argument("postprocess expects output shape [1, 84, num_boxes]");
    }
    if (info.scale <= 0.0F || input_size <= 0) {
        throw std::invalid_argument("postprocess received invalid letterbox metadata");
    }

    const int64_t num_boxes = shape[2];
    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<std::size_t>(num_boxes));

    for (int64_t box_idx = 0; box_idx < num_boxes; ++box_idx) {
        int best_class_id = -1;
        float best_conf = 0.0F;
        const CandidateTier tier = select_candidate_class(
            output_data + (4 * num_boxes),
            num_boxes,
            box_idx,
            conf_threshold,
            per_class_conf,
            low_confidence_floor,
            best_class_id,
            best_conf);
        if (tier == CandidateTier::rejected) {
            continue;
        }
        const bool low_confidence_candidate = tier == CandidateTier::weak;

        const float cx = output_data[box_idx];
        const float cy = output_data[num_boxes + box_idx];
        const float w = output_data[(2 * num_boxes) + box_idx];
        const float h = output_data[(3 * num_boxes) + box_idx];

        float x1 = (cx - (w * 0.5F) - static_cast<float>(info.pad_left)) / info.scale;
        float y1 = (cy - (h * 0.5F) - static_cast<float>(info.pad_top)) / info.scale;
        float x2 = (cx + (w * 0.5F) - static_cast<float>(info.pad_left)) / info.scale;
        float y2 = (cy + (h * 0.5F) - static_cast<float>(info.pad_top)) / info.scale;

        x1 = std::max(0.0F, x1);
        y1 = std::max(0.0F, y1);
        x2 = std::max(0.0F, x2);
        y2 = std::max(0.0F, y2);
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        Candidate candidate;
        candidate.bbox = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
        candidate.conf = best_conf;
        candidate.class_id = best_class_id;
        candidate.low_confidence_candidate = low_confidence_candidate;
        candidates.push_back(std::move(candidate));
    }

    const std::vector<Candidate> selected =
        select_candidates(std::move(candidates), iou_threshold, low_confidence_max_candidates);

    std::vector<Detection> detections;
    detections.reserve(selected.size());
    for (const Candidate& candidate : selected) {
        Detection detection;
        detection.bbox = to_int_rect(candidate.bbox);
        detection.confidence = candidate.conf;
        detection.class_id = candidate.class_id;
        detection.class_name = COCO_CLASSES[static_cast<std::size_t>(candidate.class_id)];
        detection.low_confidence_candidate = candidate.low_confidence_candidate;
        detection.source = DetectionSource::coco80;
        detection.original_class_id = candidate.class_id;
        detections.push_back(std::move(detection));
    }

    return detections;
}

std::vector<Detection> postprocess_seg(
    const float* detection_data,
    const std::vector<int64_t>& detection_shape,
    const float* prototype_data,
    const std::vector<int64_t>& prototype_shape,
    const LetterboxInfo& info,
    float conf_threshold,
    float iou_threshold,
    float mask_threshold,
    int input_size) {
    return postprocess_seg(
        detection_data,
        detection_shape,
        prototype_data,
        prototype_shape,
        info,
        conf_threshold,
        {},
        iou_threshold,
        mask_threshold,
        input_size);
}

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
    int input_size) {
    return postprocess_seg(
        detection_data,
        detection_shape,
        prototype_data,
        prototype_shape,
        info,
        conf_threshold,
        per_class_conf,
        0.0F,
        8,
        iou_threshold,
        mask_threshold,
        input_size);
}

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
    int input_size) {
    if (detection_data == nullptr) {
        throw std::invalid_argument("postprocessSeg received null detection_data");
    }
    if (prototype_data == nullptr) {
        throw std::invalid_argument("postprocessSeg received null prototype_data");
    }
    if (detection_shape.size() != 3 || detection_shape[0] != 1 || detection_shape[1] != 116 ||
        detection_shape[2] <= 0) {
        throw std::invalid_argument("postprocessSeg expects detection output shape [1, 116, num_boxes]");
    }
    if (prototype_shape.size() != 4 || prototype_shape[0] != 1 || prototype_shape[1] != 32 ||
        prototype_shape[2] <= 0 || prototype_shape[3] <= 0) {
        throw std::invalid_argument("postprocessSeg expects prototype output shape [1, 32, mask_h, mask_w]");
    }
    if (info.scale <= 0.0F || input_size <= 0 || info.original_width <= 0 || info.original_height <= 0) {
        throw std::invalid_argument("postprocessSeg received invalid letterbox metadata");
    }

    const auto mask_decode_start = std::chrono::steady_clock::now();
    const int64_t num_boxes = detection_shape[2];
    const int proto_channels = static_cast<int>(prototype_shape[1]);
    const int proto_h = static_cast<int>(prototype_shape[2]);
    const int proto_w = static_cast<int>(prototype_shape[3]);
    const int proto_plane = proto_h * proto_w;
    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<std::size_t>(num_boxes));

    for (int64_t box_idx = 0; box_idx < num_boxes; ++box_idx) {
        int best_class_id = -1;
        float best_conf = 0.0F;
        const CandidateTier tier = select_candidate_class(
            detection_data + (4 * num_boxes),
            num_boxes,
            box_idx,
            conf_threshold,
            per_class_conf,
            low_confidence_floor,
            best_class_id,
            best_conf);
        if (tier == CandidateTier::rejected) {
            continue;
        }
        const bool low_confidence_candidate = tier == CandidateTier::weak;

        const float cx = detection_data[box_idx];
        const float cy = detection_data[num_boxes + box_idx];
        const float w = detection_data[(2 * num_boxes) + box_idx];
        const float h = detection_data[(3 * num_boxes) + box_idx];

        float x1 = (cx - (w * 0.5F) - static_cast<float>(info.pad_left)) / info.scale;
        float y1 = (cy - (h * 0.5F) - static_cast<float>(info.pad_top)) / info.scale;
        float x2 = (cx + (w * 0.5F) - static_cast<float>(info.pad_left)) / info.scale;
        float y2 = (cy + (h * 0.5F) - static_cast<float>(info.pad_top)) / info.scale;

        x1 = std::max(0.0F, std::min(x1, static_cast<float>(info.original_width)));
        y1 = std::max(0.0F, std::min(y1, static_cast<float>(info.original_height)));
        x2 = std::max(0.0F, std::min(x2, static_cast<float>(info.original_width)));
        y2 = std::max(0.0F, std::min(y2, static_cast<float>(info.original_height)));
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        Candidate candidate;
        candidate.bbox = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
        candidate.conf = best_conf;
        candidate.class_id = best_class_id;
        candidate.low_confidence_candidate = low_confidence_candidate;
        for (int coef_idx = 0; coef_idx < proto_channels; ++coef_idx) {
            candidate.mask_coef[static_cast<std::size_t>(coef_idx)] =
                detection_data[((84 + coef_idx) * num_boxes) + box_idx];
        }
        candidates.push_back(candidate);
    }

    const std::vector<Candidate> selected =
        select_candidates(std::move(candidates), iou_threshold, low_confidence_max_candidates);

    cv::Mat proto(proto_channels, proto_plane, CV_32F, const_cast<float*>(prototype_data));
    std::vector<Detection> detections;
    detections.reserve(selected.size());
    std::int64_t total_mask_nonzero = 0;
    const CocoMaskResizeMode mask_resize_mode = coco_mask_resize_mode();

    for (const Candidate& candidate : selected) {
        Detection detection;
        detection.bbox = rect_within_image(to_int_rect(candidate.bbox), info.original_width, info.original_height);
        if (detection.bbox.empty()) {
            continue;
        }
        detection.confidence = candidate.conf;
        detection.class_id = candidate.class_id;
        detection.class_name = COCO_CLASSES[static_cast<std::size_t>(candidate.class_id)];
        detection.low_confidence_candidate = candidate.low_confidence_candidate;

        cv::Mat coef_mat(1, proto_channels, CV_32F, const_cast<float*>(candidate.mask_coef.data()));
        cv::Mat mask_raw = coef_mat * proto;
        mask_raw = mask_raw.reshape(1, proto_h);

        cv::Mat mask_exp;
        cv::exp(-mask_raw, mask_exp);
        cv::Mat mask_sigmoid = 1.0 / (1.0 + mask_exp);

        cv::Mat mask_input;
        cv::resize(mask_sigmoid, mask_input, cv::Size(input_size, input_size), 0.0, 0.0, cv::INTER_LINEAR);

        const cv::Rect letterbox_roi =
            rect_within_image(cv::Rect(info.pad_left, info.pad_top, info.new_width, info.new_height), input_size, input_size);
        if (letterbox_roi.empty()) {
            LOG_WARN_M("Postprocess", "empty letterbox ROI while decoding mask");
            continue;
        }

        cv::Mat mask_unpadded = mask_input(letterbox_roi);
        cv::Mat mask_original;
        cv::Mat mask_cropped;
        bool roi_supported = false;

        if (mask_resize_mode == CocoMaskResizeMode::roi) {
            roi_supported = resize_linear_integer_scale_roi(
                mask_unpadded,
                cv::Size(info.original_width, info.original_height),
                detection.bbox,
                mask_cropped);
        }

        if (mask_resize_mode != CocoMaskResizeMode::roi || !roi_supported) {
            cv::resize(
                mask_unpadded,
                mask_original,
                cv::Size(info.original_width, info.original_height),
                0.0,
                0.0,
                cv::INTER_LINEAR);
            mask_cropped = mask_original(detection.bbox);
        }

        cv::Mat mask_binary;
        cv::threshold(mask_cropped, mask_binary, mask_threshold, 255.0, cv::THRESH_BINARY);
        mask_binary.convertTo(detection.mask, CV_8UC1);
        detection.source = DetectionSource::coco80;
        detection.original_class_id = candidate.class_id;

        const int nonzero = cv::countNonZero(detection.mask);
        total_mask_nonzero += nonzero;
        LOG_DEBUG_M("Postprocess",
                    "mask instance class=" << detection.class_name << " bbox=[" << detection.bbox.x << ","
                                           << detection.bbox.y << "," << detection.bbox.width << ","
                                           << detection.bbox.height << "] size=" << detection.mask.cols << "x"
                                           << detection.mask.rows << " nonzero=" << nonzero);

        detections.push_back(std::move(detection));
    }

    const auto mask_decode_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::steady_clock::now() - mask_decode_start)
                                    .count() /
                                1000.0;
    LOG_DEBUG_M("Postprocess",
                "mask_decode_ms=" << mask_decode_ms << " detections=" << detections.size()
                                  << " mask_proto=" << proto_w << "x" << proto_h
                                  << " total_mask_nonzero=" << total_mask_nonzero);

    return detections;
}

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
    int input_size) {
    validate_model_spec(model_spec);
    if (detection_data == nullptr) {
        throw std::invalid_argument("ModelSpec postprocessSeg received null detection_data");
    }
    if (prototype_data == nullptr) {
        throw std::invalid_argument("ModelSpec postprocessSeg received null prototype_data");
    }
    if (detection_shape.size() != 3 || detection_shape[0] != 1 ||
        detection_shape[1] != model_spec.expected_detection_channels ||
        detection_shape[2] != model_spec.expected_num_boxes) {
        throw std::invalid_argument("segmentation detection tensor differs from ModelSpec");
    }
    if (prototype_shape.size() != 4 || prototype_shape[0] != 1 ||
        prototype_shape[1] != model_spec.mask_coefficient_count ||
        prototype_shape[2] != model_spec.expected_proto_height ||
        prototype_shape[3] != model_spec.expected_proto_width) {
        throw std::invalid_argument("segmentation prototype tensor differs from ModelSpec");
    }
    if (input_size != model_spec.input_size || info.scale <= 0.0F ||
        info.original_width <= 0 || info.original_height <= 0 ||
        info.new_width <= 0 || info.new_height <= 0) {
        throw std::invalid_argument("ModelSpec postprocessSeg received invalid input/letterbox metadata");
    }
    if (!std::isfinite(conf_threshold) || conf_threshold < 0.0F ||
        conf_threshold > 1.0F || !std::isfinite(iou_threshold) ||
        iou_threshold < 0.0F || iou_threshold > 1.0F ||
        !std::isfinite(mask_threshold) || mask_threshold < 0.0F ||
        mask_threshold > 1.0F) {
        throw std::invalid_argument("ModelSpec postprocessSeg thresholds must be finite in [0, 1]");
    }

    const std::int64_t num_boxes = detection_shape[2];
    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<std::size_t>(num_boxes));

    for (std::int64_t box_idx = 0; box_idx < num_boxes; ++box_idx) {
        int best_local_class_id = -1;
        float best_conf = -std::numeric_limits<float>::infinity();
        for (int local_class_id = 0; local_class_id < model_spec.num_classes;
             ++local_class_id) {
            const float score =
                detection_data[((4 + local_class_id) * num_boxes) + box_idx];
            if (std::isfinite(score) && score > best_conf) {
                best_conf = score;
                best_local_class_id = local_class_id;
            }
        }
        if (best_local_class_id < 0 || best_conf < conf_threshold) {
            continue;
        }

        const float cx = detection_data[box_idx];
        const float cy = detection_data[num_boxes + box_idx];
        const float width = detection_data[(2 * num_boxes) + box_idx];
        const float height = detection_data[(3 * num_boxes) + box_idx];
        if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) ||
            !std::isfinite(height) || width <= 0.0F || height <= 0.0F) {
            continue;
        }

        Candidate candidate;
        // Keep the raw xyxy box in the 896 letterbox coordinate system until
        // after pure class-aware NMS. Ultralytics/NCNN NMS operates in this
        // space; clipping first can turn two distinct edge boxes into an exact
        // overlap and suppress the wrong candidate.
        candidate.bbox = cv::Rect2f(
            cx - (width * 0.5F),
            cy - (height * 0.5F),
            width,
            height);
        candidate.conf = best_conf;
        candidate.original_class_id = best_local_class_id;
        candidate.class_id =
            model_spec.local_to_global_class_ids[static_cast<std::size_t>(best_local_class_id)];
        candidate.source_index = box_idx;
        candidate.low_confidence_candidate = false;
        for (int coef_idx = 0; coef_idx < model_spec.mask_coefficient_count;
             ++coef_idx) {
            const float coefficient =
                detection_data[
                    ((model_spec.mask_coefficient_offset + coef_idx) * num_boxes) +
                    box_idx];
            candidate.mask_coef[static_cast<std::size_t>(coef_idx)] =
                std::isfinite(coefficient) ? coefficient : 0.0F;
        }
        candidates.push_back(std::move(candidate));
    }

    const std::vector<Candidate> selected = select_pure_class_aware_candidates(
        std::move(candidates),
        iou_threshold,
        model_spec.max_detections);

    const int proto_channels = static_cast<int>(prototype_shape[1]);
    const int proto_height = static_cast<int>(prototype_shape[2]);
    const int proto_width = static_cast<int>(prototype_shape[3]);
    const int proto_plane = proto_height * proto_width;
    cv::Mat proto(
        proto_channels,
        proto_plane,
        CV_32F,
        const_cast<float*>(prototype_data));

    const cv::Rect native_mask_roi = ultralytics_v8466_native_mask_roi(
        proto_width,
        proto_height,
        info.original_width,
        info.original_height);
    if (native_mask_roi.empty()) {
        throw std::invalid_argument("ModelSpec postprocessSeg has empty native mask ROI");
    }

    std::vector<Candidate> mapped;
    mapped.reserve(selected.size());
    for (Candidate candidate : selected) {
        float x1 =
            (candidate.bbox.x - static_cast<float>(info.pad_left)) / info.scale;
        float y1 =
            (candidate.bbox.y - static_cast<float>(info.pad_top)) / info.scale;
        float x2 =
            (candidate.bbox.x + candidate.bbox.width -
             static_cast<float>(info.pad_left)) /
            info.scale;
        float y2 =
            (candidate.bbox.y + candidate.bbox.height -
             static_cast<float>(info.pad_top)) /
            info.scale;
        x1 = std::max(0.0F, std::min(x1, static_cast<float>(info.original_width)));
        y1 = std::max(0.0F, std::min(y1, static_cast<float>(info.original_height)));
        x2 = std::max(0.0F, std::min(x2, static_cast<float>(info.original_width)));
        y2 = std::max(0.0F, std::min(y2, static_cast<float>(info.original_height)));
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }
        candidate.bbox = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
        mapped.push_back(std::move(candidate));
    }

    std::vector<Detection> detections;
    detections.reserve(mapped.size());
    bool traversable_canvas_emitted = false;
    for (const Candidate& candidate : mapped) {
        Detection detection;
        detection.bbox = rect_within_image(
            to_ultralytics_cpu_crop_rect(candidate.bbox, mapped.size()),
            info.original_width,
            info.original_height);
        if (detection.bbox.empty()) {
            continue;
        }
        detection.confidence = candidate.conf;
        detection.class_id = candidate.class_id;
        detection.original_class_id = candidate.original_class_id;
        detection.class_name =
            model_spec.class_names[static_cast<std::size_t>(candidate.original_class_id)];
        detection.low_confidence_candidate = false;
        detection.source = model_spec.source;

        cv::Mat coefficient_row(
            1,
            proto_channels,
            CV_32F,
            const_cast<float*>(candidate.mask_coef.data()));
        cv::Mat mask_raw = coefficient_row * proto;
        mask_raw = mask_raw.reshape(1, proto_height);

        // Match Ultralytics 8.4.66 process_mask_native: interpolate logits, not
        // sigmoid probabilities, and perform one resize from the cropped
        // prototype directly to original resolution.
        const cv::Mat mask_unpadded_logits = mask_raw(native_mask_roi);
        cv::Mat mask_original_logits;
        cv::resize(
            mask_unpadded_logits,
            mask_original_logits,
            cv::Size(info.original_width, info.original_height),
            0.0,
            0.0,
            cv::INTER_LINEAR);

        const cv::Mat bbox_mask_logits = mask_original_logits(detection.bbox);
        cv::Mat bbox_mask_binary;
        if (mask_threshold <= 0.0F) {
            bbox_mask_binary =
                cv::Mat(bbox_mask_logits.size(), CV_8UC1, cv::Scalar(255));
        } else if (mask_threshold >= 1.0F) {
            bbox_mask_binary =
                cv::Mat::zeros(bbox_mask_logits.size(), CV_8UC1);
        } else {
            cv::threshold(
                bbox_mask_logits,
                bbox_mask_binary,
                probability_to_logit_threshold(mask_threshold),
                255.0,
                cv::THRESH_BINARY);
        }
        bbox_mask_binary.convertTo(detection.mask, CV_8UC1);
        // Only the map-like traversable class needs a persistent full-image
        // canvas. Obstacle instances keep their established bbox-local masks;
        // the merge stage projects them transiently when subtracting from 92.
        // This bounds memory even when max_detections=300.
        if (detection.class_id == 92 && !traversable_canvas_emitted) {
            detection.full_image_mask = make_full_image_mask(
                detection.mask,
                detection.bbox,
                info.original_width,
                info.original_height);
            traversable_canvas_emitted = true;
        }
        detections.push_back(std::move(detection));
    }

    return detections;
}

}  // namespace yolo
