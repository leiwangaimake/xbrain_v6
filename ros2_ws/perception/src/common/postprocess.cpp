/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: postprocess.cpp
 * Brief: YOLO output postprocessing implementation.
 *
 * Description:
 * Implements postprocess.hpp; read its header first. Layout of this file:
 *   - COCO mask fast paths: resize_linear_integer_scale_roi (crop of an integer-scale resize with
 *     a halo) and decode_coco_mask_roi2 (only the prototype rows those resizes read);
 *   - Active12 kernels: logits_rectangle (column-limited dot products, used only after a
 *     calibration shows it equals cv::gemm), and detail::resize_linear_roi with its calibration
 *     (the bbox of a full INTER_LINEAR resize computed on the bbox only, rounding chosen per column
 *     to match OpenCV);
 *   - candidate selection (tiers, NMS), the COCO decoder, the Active12 decoder and its dispatch.
 * Every fast path either reproduces the reference values exactly or returns false and lets the
 * caller take the reference path.
 */

#include "common/postprocess.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/linear_bbox_resize.hpp"
#include "common/logger.hpp"
#include "common/persistent_task_pair.hpp"
#include "common/time_utils.hpp"
#include "common/top_class.hpp"

namespace yolo {

namespace {
// Per thread, see PostWorkCounts.
thread_local PostWorkCounts last_work_counts;
}  // namespace

PostWorkCounts last_post_work_counts() noexcept { return last_work_counts; }

// Index = COCO80 class id = global id 0..79.
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

namespace {

// The sealed Active12 contract of one input shape: 12 classes, 32 mask coefficients at channel 16
// (= 4 + 12) of 48, at most 300 detections. Global ids 80..93 skip 87 and 88 (carton,
// storage_rack): reserved in config/class_schema_94.json and not produced by this model.
// The anchor count and the prototype size are sealed literals, and are also derived here from the
// three-level head (one anchor per cell at strides 8, 16 and 32; prototypes at stride 4): a
// literal that disagrees with the head is a program error, not a model to accept. The same holds
// for a window that does not start on a stride-32 row or leaves the square letterbox.
ModelSpec make_active12_spec(int input_width, int input_height, int input_window_top, std::int64_t num_boxes,
                             int proto_height, int proto_width) {
    std::int64_t derived_boxes = 0;
    for (const int stride : {8, 16, 32}) {
        derived_boxes += static_cast<std::int64_t>(input_width / stride) * (input_height / stride);
    }
    if (input_width % 32 != 0 || input_height % 32 != 0 || derived_boxes != num_boxes ||
        proto_height != input_height / 4 || proto_width != input_width / 4 || input_window_top < 0 ||
        input_window_top % 32 != 0 || input_window_top + input_height > input_width) {
        throw std::logic_error("sealed Active12 literals disagree with the stride 8/16/32 head layout");
    }
    return ModelSpec{
        DetectionSource::active12,
        input_width,
        input_height,
        input_window_top,
        12,
        32,
        16,
        48,
        num_boxes,
        proto_height,
        proto_width,
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
}

// The sealed spec of an input shape, nullptr if there is none.
//   896 x 896: 16464 anchors (112^2 + 56^2 + 28^2), 224 x 224 prototypes;
//   896 wide, 608 high, rows 160..767 of the square letterbox: 11172 anchors (112 x 76 + 56 x 38 +
//   28 x 19), prototypes 152 high and 224 wide (rows 40..191 of the square prototype plane).
const ModelSpec* sealed_active12_spec(int input_width, int input_height) {
    static const ModelSpec square = make_active12_spec(896, 896, 0, 16464, 224, 224);
    static const ModelSpec wide = make_active12_spec(896, 608, 160, 11172, 152, 224);
    if (input_width == 896 && input_height == 896) return &square;
    if (input_width == 896 && input_height == 608) return &wide;
    return nullptr;
}

}  // namespace

const ModelSpec& active12_seg_model_spec() {
    return *sealed_active12_spec(896, 896);
}

const ModelSpec& active12_seg_model_spec(ModelInputShape input_shape) {
    const ModelSpec* spec = sealed_active12_spec(input_shape.width, input_shape.height);
    if (spec == nullptr) {
        throw std::invalid_argument("no sealed active12 ModelSpec for input " + std::to_string(input_shape.width) +
                                    " x " + std::to_string(input_shape.height));
    }
    return *spec;
}

namespace {

enum class CocoMaskResizeMode {
    full,
    roi,
};

// PERCEPTION_COCO_MASK_RESIZE, read once: unset, "roi" or "1" = roi (resize only what the box
// needs, same values), anything else = the full-image resize.
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

// The halo source window of an integer-scale ROI resize (see resize_linear_integer_scale_roi):
// false when the optimized crop does not apply. Split out so a caller that already holds exactly
// that window (decode_coco_mask_roi2) can resize it without first pasting it into a zero canvas.
bool plan_integer_scale_roi(
    const cv::Size& source_size,
    const cv::Size& full_output_size,
    const cv::Rect& output_roi,
    cv::Rect& source_roi,
    int& scale) {
    constexpr int kMinimumOptimizedDimension = 32;
    constexpr std::int64_t kMinimumOptimizedArea = 4096;
    if (source_size.width <= 0 || source_size.height <= 0 ||
        full_output_size.width <= 0 || full_output_size.height <= 0 ||
        output_roi.empty() ||
        output_roi.width < kMinimumOptimizedDimension ||
        output_roi.height < kMinimumOptimizedDimension ||
        static_cast<std::int64_t>(output_roi.width) * output_roi.height <
            kMinimumOptimizedArea ||
        output_roi.x < 0 || output_roi.y < 0 ||
        output_roi.x + output_roi.width > full_output_size.width ||
        output_roi.y + output_roi.height > full_output_size.height ||
        full_output_size.width % source_size.width != 0 ||
        full_output_size.height % source_size.height != 0) {
        return false;
    }

    const int scale_x = full_output_size.width / source_size.width;
    const int scale_y = full_output_size.height / source_size.height;
    if (scale_x <= 0 || scale_x != scale_y) {
        return false;
    }
    scale = scale_x;

    // Keep a one-source-pixel halo so the cropped resize has the same linear
    // interpolation neighbours as the full-image resize. The local crop is
    // phase-aligned to the integer output scale before extracting output_roi.
    const int source_left = std::max(0, (output_roi.x / scale) - 1);
    const int source_top = std::max(0, (output_roi.y / scale) - 1);
    const int output_right = output_roi.x + output_roi.width;
    const int output_bottom = output_roi.y + output_roi.height;
    const int source_right = std::min(
        source_size.width,
        ((output_right + scale - 1) / scale) + 1);
    const int source_bottom = std::min(
        source_size.height,
        ((output_bottom + scale - 1) / scale) + 1);
    source_roi = cv::Rect(
        source_left,
        source_top,
        source_right - source_left,
        source_bottom - source_top);
    return !source_roi.empty();
}

// Resize the planned window (`window` holds exactly source_roi of the source) and cut output_roi.
bool resize_integer_scale_window(
    const cv::Mat& window,
    const cv::Rect& source_roi,
    int scale,
    const cv::Rect& output_roi,
    cv::Mat& output) {
    cv::Mat enlarged;
    cv::resize(
        window,
        enlarged,
        cv::Size(source_roi.width * scale, source_roi.height * scale),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    const cv::Rect local_roi(
        output_roi.x - (source_roi.x * scale),
        output_roi.y - (source_roi.y * scale),
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

bool resize_linear_integer_scale_roi(
    const cv::Mat& source,
    const cv::Size& full_output_size,
    const cv::Rect& output_roi,
    cv::Mat& output) {
    output.release();
    if (source.empty() || source.type() != CV_32FC1) {
        return false;
    }
    cv::Rect source_roi;
    int scale = 0;
    if (!plan_integer_scale_roi(source.size(), full_output_size, output_roi, source_roi, scale)) {
        return false;
    }
    return resize_integer_scale_window(source(source_roi), source_roi, scale, output_roi, output);
}

// PERCEPTION_COCO_MASK_ROI2=1 (read once): try decode_coco_mask_roi2 before the plane path.
bool coco_mask_roi2_enabled() {
    static const bool enabled = []() {
        const char* value = std::getenv("PERCEPTION_COCO_MASK_ROI2");
        return value != nullptr && std::string(value) == "1";
    }();
    return enabled;
}

// ROI-limited COCO mask decode. Computes gemm/exp/sigmoid only on the complete
// prototype rows that the two integer-scale ROI resizes read (rows keep the
// same SIMD partition as the full plane: width multiple of 16), then applies
// the existing halo-aligned ROI resize twice. Returns false (caller uses the
// original full path) whenever any precondition is not met.
bool decode_coco_mask_roi2(
    const cv::Mat& proto,
    const float* coefficients,
    int proto_channels,
    int proto_h,
    int proto_w,
    const LetterboxInfo& info,
    int input_size,
    const cv::Rect& bbox,
    cv::Mat& mask_cropped) {
    if (proto.rows != proto_channels || proto.cols != proto_h * proto_w || proto.type() != CV_32F ||
        proto_w % 16 != 0 || info.new_width <= 0 || info.new_height <= 0 ||
        info.original_width % info.new_width != 0 || info.original_height % info.new_height != 0 ||
        info.original_width / info.new_width != info.original_height / info.new_height ||
        input_size % proto_w != 0 || input_size % proto_h != 0 || input_size / proto_w != input_size / proto_h ||
        info.pad_left < 0 || info.pad_top < 0 || info.pad_left + info.new_width > input_size ||
        info.pad_top + info.new_height > input_size) {
        return false;
    }
    const int s2 = info.original_width / info.new_width;
    const int s1 = input_size / proto_w;
    if (s2 <= 0 || s1 <= 0) return false;
    // Stage-2 source window in unpadded coordinates (same arithmetic as the helper).
    const int sl2 = std::max(0, (bbox.x / s2) - 1);
    const int st2 = std::max(0, (bbox.y / s2) - 1);
    const int sr2 = std::min(info.new_width, ((bbox.x + bbox.width + s2 - 1) / s2) + 1);
    const int sb2 = std::min(info.new_height, ((bbox.y + bbox.height + s2 - 1) / s2) + 1);
    if (sr2 <= sl2 || sb2 <= st2) return false;
    const cv::Rect s2_roi(sl2, st2, sr2 - sl2, sb2 - st2);
    const cv::Rect s2_in_input(s2_roi.x + info.pad_left, s2_roi.y + info.pad_top, s2_roi.width, s2_roi.height);
    if (s2_in_input.x < 0 || s2_in_input.y < 0 || s2_in_input.x + s2_in_input.width > input_size ||
        s2_in_input.y + s2_in_input.height > input_size) {
        return false;
    }
    // Stage-1 source rows (full rows).
    const int st1 = std::max(0, (s2_in_input.y / s1) - 1);
    const int sb1 = std::min(proto_h, ((s2_in_input.y + s2_in_input.height + s1 - 1) / s1) + 1);
    if (sb1 <= st1) return false;
    const int rows = sb1 - st1;
    // A column-range view, not a copy: the product of the non-continuous view is bit-identical
    // to the product of its clone (OpenCV 4.5.4 on 1.8 and OpenCV 5 on the Mac, checked
    // 2026-09-29), and the copy was a whole band of all 32 channels per instance.
    const cv::Mat sub_proto = proto.colRange(st1 * proto_w, sb1 * proto_w);
    cv::Mat coef_mat(1, proto_channels, CV_32F, const_cast<float*>(coefficients));
    cv::Mat raw = coef_mat * sub_proto;
    raw = raw.reshape(1, rows);
    cv::Mat raw_exp;
    cv::exp(-raw, raw_exp);
    cv::Mat sigmoid_rows = 1.0 / (1.0 + raw_exp);
    // Stage 1 reads the halo window of the zero-padded sigmoid plane; its rows are exactly
    // [st1, sb1) (same arithmetic), which sigmoid_rows already holds. Resize that window
    // directly unless the plan differs; the canvas path stays as the fallback.
    cv::Mat stage1;
    cv::Rect stage1_source;
    int stage1_scale = 0;
    if (plan_integer_scale_roi(cv::Size(proto_w, proto_h), cv::Size(input_size, input_size), s2_in_input,
                               stage1_source, stage1_scale) &&
        stage1_source.y == st1 && stage1_source.y + stage1_source.height == sb1) {
        if (!resize_integer_scale_window(sigmoid_rows.colRange(stage1_source.x, stage1_source.x + stage1_source.width),
                                         stage1_source, stage1_scale, s2_in_input, stage1)) {
            return false;
        }
    } else {
        cv::Mat sigmoid_full = cv::Mat::zeros(proto_h, proto_w, CV_32F);
        sigmoid_rows.copyTo(sigmoid_full.rowRange(st1, sb1));
        if (!resize_linear_integer_scale_roi(sigmoid_full, cv::Size(input_size, input_size), s2_in_input, stage1)) {
            return false;
        }
    }
    // Stage 2 reads exactly the window s2_roi of the unpadded canvas, which is stage1 itself (same
    // halo arithmetic, checked here). Resize stage1 directly instead of zero-filling the whole
    // new_width x new_height canvas (1 MB at 640x400) and pasting it back; any other plan keeps
    // the canvas path, so the output is identical by construction.
    cv::Rect stage2_source;
    int stage2_scale = 0;
    mask_cropped.release();
    if (plan_integer_scale_roi(cv::Size(info.new_width, info.new_height),
                               cv::Size(info.original_width, info.original_height), bbox, stage2_source,
                               stage2_scale) &&
        stage2_source == s2_roi && stage1.size() == s2_roi.size()) {
        return resize_integer_scale_window(stage1, stage2_source, stage2_scale, bbox, mask_cropped);
    }
    cv::Mat unpadded = cv::Mat::zeros(info.new_height, info.new_width, CV_32F);
    stage1.copyTo(unpadded(s2_roi));
    return resize_linear_integer_scale_roi(
        unpadded, cv::Size(info.original_width, info.original_height), bbox, mask_cropped);
}

// PERCEPTION_ACTIVE12_MASK_ROWS=1 (read once): compute only the logits the bbox resize reads.
bool active12_mask_rows_enabled() {
    static const bool enabled = []() {
        const char* value = std::getenv("PERCEPTION_ACTIVE12_MASK_ROWS");
        return value != nullptr && std::string(value) == "1";
    }();
    return enabled;
}

// The mask kernels below are plain per-element loops. GCC 11 (the target compiler) does not
// auto-vectorize at -O2, and vectorizing them keeps every element's operation order (no
// reduction is reordered; the double dot product only adds exact float products), so it is
// enabled for this section only. Measured on 1.8: 1.7x (full rows) to 8x (narrow boxes)
// faster than cv::gemm, and bit-identical. Clang vectorizes at -O2 already.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("tree-vectorize")
#endif

// Logits of a prototype rectangle as a per-element dot product: channels 0..31 in order, float
// products widened to double (exact), summed in double, rounded to float once. OpenCV's own
// product of a 1x32 row with a 32-channel band gives exactly these values (checked 2026-09-29 on
// OpenCV 4.5.4 on 1.8 and OpenCV 5 on the Mac); unlike the matrix product this can be limited to
// the columns the resize reads. It is used only if a calibration against cv::gemm matches.
// Channel-outer order: each channel's rows are read as contiguous runs and the double sums stay
// in one per-thread buffer (the parallel lanes each have their own), which is what makes it
// faster than a row-outer loop that jumps between the 32 channel planes.
void logits_rectangle(const cv::Mat& proto, const float* coefficients, int proto_channels, int proto_width,
                      int first_row, int last_row, int first_col, int last_col, cv::Mat& logits,
                      int out_row0, int out_col0) {
    const int rows = last_row - first_row + 1;
    const int cols = last_col - first_col + 1;
    static thread_local std::vector<double> sums;
    sums.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), 0.0);
    for (int c = 0; c < proto_channels; ++c) {
        const double k = coefficients[c];
        const float* plane = proto.ptr<float>(c);
        for (int r = 0; r < rows; ++r) {
            const float* p = plane + static_cast<std::size_t>(first_row + r) * proto_width + first_col;
            double* sum = sums.data() + static_cast<std::size_t>(r) * cols;
            for (int x = 0; x < cols; ++x) sum[x] += k * static_cast<double>(p[x]);
        }
    }
    for (int r = 0; r < rows; ++r) {
        const double* sum = sums.data() + static_cast<std::size_t>(r) * cols;
        // logits(0, 0) is prototype element (out_row0, out_col0).
        float* out = logits.ptr<float>(first_row + r - out_row0) + (first_col - out_col0);
        for (int x = 0; x < cols; ++x) out[x] = static_cast<float>(sum[x]);
    }
}

// Once per process: does logits_rectangle reproduce cv::gemm on this OpenCV build?
bool logits_rectangle_exact() {
    static const bool exact = []() {
        constexpr int kChannels = 32, kWidth = 224, kRows = 24;
        cv::Mat proto(kChannels, kWidth * kRows, CV_32F);
        std::uint32_t state = 0x85EBCA6BU;
        for (int c = 0; c < kChannels; ++c) {
            float* p = proto.ptr<float>(c);
            for (int i = 0; i < proto.cols; ++i) {
                state = state * 1664525U + 1013904223U;
                p[i] = static_cast<float>(state >> 8) / 16777216.0F * 2.0F - 1.0F;
            }
        }
        std::array<float, kChannels> coefficients {};
        for (float& v : coefficients) {
            state = state * 1664525U + 1013904223U;
            v = static_cast<float>(state >> 8) / 16777216.0F - 0.5F;
        }
        const cv::Mat coefficient_row(1, kChannels, CV_32F, coefficients.data());
        const cv::Mat product = coefficient_row * proto;
        const cv::Mat reference = product.reshape(1, kRows);
        cv::Mat mine = cv::Mat::zeros(kRows, kWidth, CV_32F);
        logits_rectangle(proto, coefficients.data(), kChannels, kWidth, 0, kRows - 1, 0, kWidth - 1, mine, 0, 0);
        bool same = true;
        for (int y = 0; y < kRows && same; ++y) {
            same = std::memcmp(mine.ptr(y), reference.ptr(y), sizeof(float) * kWidth) == 0;
        }
        LOG_INFO_M("Postprocess", "ACTIVE12_LOGITS kernel=" << (same ? "column_limited_dot" : "opencv_gemm_rows"));
        return same;
    }();
    return exact;
}

// One instance's logits: either a prototype plane (proto_height x proto_width; rows the resize
// does not read may be zero), or only the prototype rectangle the bbox resize reads, with the
// plane coordinates of its first element.
struct InstanceLogits {
    cv::Mat values;
    int row0 = 0, col0 = 0;
    bool rectangle = false;
    int plane_rows = 0, plane_cols = 0;
};

InstanceLogits whole_plane(cv::Mat plane) {
    InstanceLogits logits;
    logits.plane_rows = plane.rows;
    logits.plane_cols = plane.cols;
    logits.values = std::move(plane);
    return logits;
}

// Active12 logits for one instance, in one of three shapes: the whole plane (row limiting off, or
// no source row left to read); only the prototype rectangle the bbox resize reads (the usual
// case); or the whole plane with only the rows the resize can read computed (conservative +/-2
// row margin, full rows keep the SIMD partition: width multiple of 16) and the other rows zero,
// when the column kernel does not reproduce cv::gemm on this machine. Every element the resize
// reads equals (coef x proto); zero rows only feed output rows outside the bbox (cropped away).
InstanceLogits active12_logits_for_bbox(
    const cv::Mat& proto,
    const float* coefficients,
    int proto_channels,
    int proto_height,
    int proto_width,
    const cv::Rect& native_mask_roi,
    const LetterboxInfo& info,
    const cv::Rect& bbox) {
    cv::Mat coefficient_row(1, proto_channels, CV_32F, const_cast<float*>(coefficients));
    if (!active12_mask_rows_enabled() || proto_width % 16 != 0 || info.original_height <= 0 ||
        proto.cols != proto_height * proto_width) {
        cv::Mat mask_raw = coefficient_row * proto;
        return whole_plane(mask_raw.reshape(1, proto_height));
    }
    const double scale_y = static_cast<double>(native_mask_roi.height) / static_cast<double>(info.original_height);
    int first = static_cast<int>(std::floor((bbox.y + 0.5) * scale_y - 0.5)) - 2;
    int last = static_cast<int>(std::floor((bbox.y + bbox.height - 1 + 0.5) * scale_y - 0.5)) + 3;
    first = std::max(0, first) + native_mask_roi.y;
    last = std::min(native_mask_roi.height - 1, last) + native_mask_roi.y;
    if (last < first) {
        cv::Mat mask_raw = coefficient_row * proto;
        return whole_plane(mask_raw.reshape(1, proto_height));
    }
    if (proto.type() == CV_32F && proto.rows == proto_channels && logits_rectangle_exact()) {
        // Only the columns the bbox resize reads (same +/-2 margin as the rows).
        const double scale_x = static_cast<double>(native_mask_roi.width) / static_cast<double>(info.original_width);
        int first_col = static_cast<int>(std::floor((bbox.x + 0.5) * scale_x - 0.5)) - 2;
        int last_col = static_cast<int>(std::floor((bbox.x + bbox.width - 1 + 0.5) * scale_x - 0.5)) + 3;
        first_col = std::max(0, first_col) + native_mask_roi.x;
        last_col = std::min(native_mask_roi.width - 1, last_col) + native_mask_roi.x;
        if (first_col <= last_col) {
            // Only the rectangle is returned: no zeroed 224x224 plane per instance (a 200 KB
            // allocation and memset; about 11 us per small instance on 1.8). resize_active12_logits
            // pastes it into a zero plane when the kernel cannot read it in place.
            InstanceLogits logits;
            logits.values.create(last - first + 1, last_col - first_col + 1, CV_32F);
            logits.row0 = first;
            logits.col0 = first_col;
            logits.rectangle = true;
            logits.plane_rows = proto_height;
            logits.plane_cols = proto_width;
            logits_rectangle(proto, coefficients, proto_channels, proto_width, first, last, first_col, last_col,
                             logits.values, first, first_col);
            return logits;
        }
    }
    const int rows = last - first + 1;
    // View, not clone: bit-identical product without copying the 32-channel band (the copy was
    // about 1.2 ms of the ~4.6 ms kernel-lab split of one instance with a bbox of ~41% of the image
    // on 1.8, before the rectangle path above existed).
    const cv::Mat sub_proto = proto.colRange(first * proto_width, (last + 1) * proto_width);
    cv::Mat sub_logits = coefficient_row * sub_proto;
    cv::Mat logits = cv::Mat::zeros(proto_height, proto_width, CV_32F);
    sub_logits.reshape(1, rows).copyTo(logits.rowRange(first, last + 1));
    return whole_plane(std::move(logits));
}

// ---- Exact bbox-only INTER_LINEAR resize (Active12 masks) ----
// The Active12 decode resizes the unpadded prototype logits to the whole original image and
// keeps only the bbox (Ultralytics process_mask_native), so every instance paid for a full
// 1280x800 float image (about 2 ms on 1.8). The COCO halo crop (resize_linear_integer_scale_roi)
// cannot be reused: 1280/224 = 40/7 is not an integer and OpenCV derives each interpolation
// weight from the ABSOLUTE output coordinate, rounded to float at that magnitude, so a resized
// crop has different weights. The kernel below computes the bbox of the full resize directly,
// with OpenCV's own tables and passes; see common/linear_bbox_resize.hpp for the calibration.
}  // namespace

namespace detail {

namespace {

constexpr LinearRounding kRoundings[] = {LinearRounding::Plain, LinearRounding::FuseFirst, LinearRounding::FuseSecond};

// s0*w0 + s1*w1 in one of the three roundings. Plain keeps both products rounded: volatile stops
// the compiler from contracting them into an fma (GCC contracts across statements by default).
template <LinearRounding R>
inline float weighted_sum(float s0, float w0, float s1, float w1) {
    if constexpr (R == LinearRounding::FuseFirst) {
        return std::fma(s0, w0, s1 * w1);
    } else if constexpr (R == LinearRounding::FuseSecond) {
        return std::fma(s1, w1, s0 * w0);
    } else {
        volatile float p = s0 * w0;
        volatile float q = s1 * w1;
        return p + q;
    }
}

// `s` is source row data whose element 0 is source column `col0`; h[0] is output column roi_x.
template <LinearRounding H>
void horizontal_run(const float* s, int col0, const ColumnTaps& tx, int from, int to, int roi_x, float* h) {
    for (int dx = from; dx < to; ++dx) {
        const std::size_t i = static_cast<std::size_t>(dx);
        const int sx = tx.index[i] - col0;
        h[dx - roi_x] = dx < tx.limit ? weighted_sum<H>(s[sx], tx.w0[i], s[sx + 1], tx.w1[i]) : s[sx];
    }
}

template <LinearRounding V>
void vertical_run(const float* h0, const float* h1, float b0, float b1, int from, int to, float* d) {
    for (int x = from; x < to; ++x) d[x] = weighted_sum<V>(h0[x], b0, h1[x], b1);
}

// The run that contains column x (the runs cover the output width in order).
std::vector<RoundingRun>::const_iterator run_at(const std::vector<RoundingRun>& runs, int x) {
    return std::upper_bound(runs.begin(), runs.end(), x,
                            [](int column, const RoundingRun& run) { return column < run.end; });
}

// Horizontal pass of one source row over output columns [roi_x, roi_end).
void horizontal_pass(const float* s, int col0, const ColumnTaps& tx, const std::vector<RoundingRun>& runs,
                     int roi_x, int roi_end, float* h) {
    for (auto run = run_at(runs, roi_x); run != runs.end() && run->begin < roi_end; ++run) {
        const int from = std::max(run->begin, roi_x), to = std::min(run->end, roi_end);
        switch (run->rounding) {
            case LinearRounding::FuseFirst: horizontal_run<LinearRounding::FuseFirst>(s, col0, tx, from, to, roi_x, h); break;
            case LinearRounding::FuseSecond: horizontal_run<LinearRounding::FuseSecond>(s, col0, tx, from, to, roi_x, h); break;
            case LinearRounding::Plain: horizontal_run<LinearRounding::Plain>(s, col0, tx, from, to, roi_x, h); break;
        }
    }
}

// Vertical pass of one output row over columns [roi_x, roi_end); h0, h1 and d start at roi_x.
void vertical_pass(const float* h0, const float* h1, float b0, float b1, const std::vector<RoundingRun>& runs,
                   int roi_x, int roi_end, float* d) {
    for (auto run = run_at(runs, roi_x); run != runs.end() && run->begin < roi_end; ++run) {
        const int from = std::max(run->begin, roi_x) - roi_x, to = std::min(run->end, roi_end) - roi_x;
        switch (run->rounding) {
            case LinearRounding::FuseFirst: vertical_run<LinearRounding::FuseFirst>(h0, h1, b0, b1, from, to, d); break;
            case LinearRounding::FuseSecond: vertical_run<LinearRounding::FuseSecond>(h0, h1, b0, b1, from, to, d); break;
            case LinearRounding::Plain: vertical_run<LinearRounding::Plain>(h0, h1, b0, b1, from, to, d); break;
        }
    }
}

}  // namespace

cv::Mat resize_linear_roi(const cv::Mat& part, int row0, int col0, const LinearBboxPlan& plan, const cv::Rect& roi) {
    const RowTaps& ty = plan.rows;
    const int roi_end = roi.x + roi.width;
    const int first_row = ty.row0[static_cast<std::size_t>(roi.y)];
    const int last_row = ty.row1[static_cast<std::size_t>(roi.y + roi.height - 1)];
    // Horizontal pass once per source row the bbox reads.
    cv::Mat rows(last_row - first_row + 1, roi.width, CV_32F);
    for (int r = first_row; r <= last_row; ++r) {
        horizontal_pass(part.ptr<float>(r - row0), col0, plan.columns, plan.horizontal, roi.x, roi_end,
                        rows.ptr<float>(r - first_row));
    }
    // Vertical pass over the two (clipped) rows of every output row.
    cv::Mat out(roi.height, roi.width, CV_32F);
    for (int y = 0; y < roi.height; ++y) {
        const std::size_t dy = static_cast<std::size_t>(roi.y + y);
        vertical_pass(rows.ptr<float>(ty.row0[dy] - first_row), rows.ptr<float>(ty.row1[dy] - first_row), ty.w0[dy],
                      ty.w1[dy], plan.vertical, roi.x, roi_end, out.ptr<float>(y));
    }
    return out;
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

const char* linear_rounding_name(LinearRounding rounding) {
    switch (rounding) {
        case LinearRounding::FuseFirst: return "fuse_first";
        case LinearRounding::FuseSecond: return "fuse_second";
        case LinearRounding::Plain: default: return "plain";
    }
}

cv::Rect linear_bbox_reads(const LinearBboxPlan& plan, const cv::Rect& roi) {
    const int first_row = plan.rows.row0[static_cast<std::size_t>(roi.y)];
    const int last_row = plan.rows.row1[static_cast<std::size_t>(roi.y + roi.height - 1)];
    const int last_x = roi.x + roi.width - 1;
    const int first_col = plan.columns.index[static_cast<std::size_t>(roi.x)];
    const int last_col = plan.columns.index[static_cast<std::size_t>(last_x)] + (last_x < plan.columns.limit ? 1 : 0);
    return {first_col, first_row, last_col - first_col + 1, last_row - first_row + 1};
}

namespace {

// resizeGeneric's horizontal table for INTER_LINEAR: float fx from the double
// (d + 0.5) * scale - 0.5, floor; a negative sx becomes (0, fx = 0); from xmax on (sx + 1 past the
// last column) OpenCV copies the sample, with sx clamped to the last column.
ColumnTaps column_taps(int source, int output) {
    const double scale = 1.0 / (static_cast<double>(output) / source);
    ColumnTaps taps;
    taps.index.resize(static_cast<std::size_t>(output));
    taps.w0.resize(static_cast<std::size_t>(output));
    taps.w1.resize(static_cast<std::size_t>(output));
    taps.limit = output;
    for (int d = 0; d < output; ++d) {
        float f = static_cast<float>((d + 0.5) * scale - 0.5);
        int s = cvFloor(f);
        f -= static_cast<float>(s);
        if (s < 0) { f = 0.0F; s = 0; }
        if (s + 1 >= source) {
            taps.limit = std::min(taps.limit, d);
            if (s >= source - 1) { f = 0.0F; s = source - 1; }
        }
        taps.index[static_cast<std::size_t>(d)] = s;
        taps.w0[static_cast<std::size_t>(d)] = 1.0F - f;
        taps.w1[static_cast<std::size_t>(d)] = f;
    }
    return taps;
}

// resizeGeneric's vertical table: sy and fy are NOT clamped; the invoker clips the two row indices
// (clip(sy, 0, h), clip(sy + 1, 0, h)), so the first and last output rows blend the edge row with
// itself with weights (1 - fy, fy).
RowTaps row_taps(int source, int output) {
    const double scale = 1.0 / (static_cast<double>(output) / source);
    RowTaps taps;
    taps.row0.resize(static_cast<std::size_t>(output));
    taps.row1.resize(static_cast<std::size_t>(output));
    taps.w0.resize(static_cast<std::size_t>(output));
    taps.w1.resize(static_cast<std::size_t>(output));
    for (int d = 0; d < output; ++d) {
        float f = static_cast<float>((d + 0.5) * scale - 0.5);
        const int s = cvFloor(f);
        f -= static_cast<float>(s);
        taps.row0[static_cast<std::size_t>(d)] = std::clamp(s, 0, source - 1);
        taps.row1[static_cast<std::size_t>(d)] = std::clamp(s + 1, 0, source - 1);
        taps.w0[static_cast<std::size_t>(d)] = 1.0F - f;
        taps.w1[static_cast<std::size_t>(d)] = f;
    }
    return taps;
}

// Calibration sample: random sign, a full 24-bit mantissa and a power of two in [2^-12, 2^12], so
// that neighbouring samples often nearly cancel and the rounding of each product shows in the sum.
// Never zero, never subnormal (the single-pass references below rely on that).
float calibration_sample(std::uint64_t& state) {
    state ^= state << 13U;
    state ^= state >> 7U;
    state ^= state << 17U;
    const float mantissa = 1.0F + static_cast<float>(state & 0x7FFFFFU) / 8388608.0F;
    const float value = std::ldexp(mantissa, static_cast<int>((state >> 23U) % 25U) - 12);
    return ((state >> 40U) & 1U) != 0U ? -value : value;
}

cv::Mat calibration_plane(const cv::Size& size, std::uint64_t& state) {
    cv::Mat plane(size, CV_32F);
    for (int y = 0; y < plane.rows; ++y) {
        float* p = plane.ptr<float>(y);
        for (int x = 0; x < plane.cols; ++x) p[x] = calibration_sample(state);
    }
    return plane;
}

// Logits-like plane: 32-term dot products of uniform [-1, 1) values scaled by `scale`, summed in
// double and rounded once, like the decoder's product of coefficients and prototypes.
cv::Mat logits_plane(const cv::Size& size, std::uint64_t& state, float scale) {
    cv::Mat plane(size, CV_32F);
    for (int y = 0; y < plane.rows; ++y) {
        float* p = plane.ptr<float>(y);
        for (int x = 0; x < plane.cols; ++x) {
            double sum = 0.0;
            for (int k = 0; k < 32; ++k) {
                state ^= state << 13U;
                state ^= state >> 7U;
                state ^= state << 17U;
                const float a = static_cast<float>(state & 0xFFFFFFU) / 8388608.0F - 1.0F;
                const float b = static_cast<float>((state >> 24U) & 0xFFFFFFU) / 8388608.0F - 1.0F;
                sum += static_cast<double>(a) * static_cast<double>(b * scale);
            }
            p[x] = static_cast<float>(sum);
        }
    }
    return plane;
}

// Infinities, NaN and -0 next to the border, where OpenCV's vertical table reads the edge row twice
// and its horizontal table clamps (left) or copies (right).
void add_edge_specials(cv::Mat& plane) {
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const int h = plane.rows, w = plane.cols;
    for (int x = 0; x < w; x += 7) {
        plane.at<float>(1, x) = inf;
        plane.at<float>(h - 1, (x + 3) % w) = -inf;
        if (h >= 3) plane.at<float>(h - 2, (x + 5) % w) = nan;
    }
    for (int y = 2; y < h; y += 5) {
        plane.at<float>(y, 1) = inf;
        plane.at<float>(y, w - 1) = nan;
    }
    for (int x = std::min(10, w - 1); x < std::min(20, w); ++x) plane.at<float>(0, x) = -0.0F;
}

// Equal bits, or both NaN (the masks only compare against a threshold).
bool same_value(float a, float b) {
    return std::memcmp(&a, &b, sizeof(float)) == 0 || (std::isnan(a) && std::isnan(b));
}

bool same_values(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size() || a.type() != CV_32F || b.type() != CV_32F) return false;
    for (int y = 0; y < a.rows; ++y) {
        const float* p = a.ptr<float>(y);
        const float* q = b.ptr<float>(y);
        for (int x = 0; x < a.cols; ++x) {
            if (!same_value(p[x], q[x])) return false;
        }
    }
    return true;
}

// Clears rounding `r` of every column whose candidate values differ from the reference in any row.
void eliminate(std::vector<std::uint8_t>& mask, int r, const cv::Mat& candidate, const cv::Mat& reference) {
    const auto bit = static_cast<std::uint8_t>(1U << static_cast<unsigned>(r));
    for (int y = 0; y < reference.rows; ++y) {
        const float* c = candidate.ptr<float>(y);
        const float* e = reference.ptr<float>(y);
        for (int x = 0; x < reference.cols; ++x) {
            if ((mask[static_cast<std::size_t>(x)] & bit) != 0U && !same_value(c[x], e[x])) {
                mask[static_cast<std::size_t>(x)] = static_cast<std::uint8_t>(mask[static_cast<std::size_t>(x)] & ~bit);
            }
        }
    }
}

// Per column the rounding left after elimination. When several are left (they gave the same values
// on every plane) the column takes its left neighbour's, so runs stay long. False if a column has none.
bool choose_runs(const std::vector<std::uint8_t>& mask, std::vector<RoundingRun>& runs, int& ambiguous) {
    runs.clear();
    ambiguous = 0;
    int previous = -1;
    for (std::size_t x = 0; x < mask.size(); ++x) {
        const unsigned m = mask[x];
        if (m == 0U) return false;
        if ((m & (m - 1U)) != 0U) ++ambiguous;
        int r = 0;
        if (previous >= 0 && ((m >> static_cast<unsigned>(previous)) & 1U) != 0U) {
            r = previous;
        } else {
            while (((m >> static_cast<unsigned>(r)) & 1U) == 0U) ++r;
        }
        const int column = static_cast<int>(x);
        if (!runs.empty() && runs.back().rounding == kRoundings[r]) {
            runs.back().end = column + 1;
        } else {
            runs.push_back({column, column + 1, kRoundings[r]});
        }
        previous = r;
    }
    return true;
}

constexpr int kHorizontalPlanes = 32;  // x (source rows) samples per output column
constexpr int kVerticalPlanes = 6;     // x (output rows) samples per output column
constexpr int kCheckPlanes = 6;

// Returns why the kernel cannot be used, or "" when the plan reproduced the reference everywhere.
std::string calibrate_into(LinearBboxPlan& plan, const ResizeReference& reference) {
    const cv::Size source = plan.source, output = plan.output;
    // Only a strict upscale runs OpenCV's two blending passes on every output element (an equal
    // size is copied, and a downscale can take the INTER_AREA path).
    if (source.width < 2 || source.height < 1 || output.width <= source.width || output.height <= source.height) {
        return "not_an_upscale";
    }
    plan.columns = column_taps(source.width, output.width);
    plan.rows = row_taps(source.height, output.height);
    const int width = output.width;
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;
    // Horizontal pass alone: resizing to (output width, source height) runs the same horizontal
    // pass (its tables and SIMD/scalar split depend on the widths only) and a vertical pass with
    // weights (1, 0), which leaves the finite nonzero values here unchanged.
    std::vector<std::uint8_t> horizontal(static_cast<std::size_t>(width), 0x7U);
    const cv::Size wide(output.width, source.height);
    cv::Mat candidate(wide, CV_32F);
    for (int p = 0; p < kHorizontalPlanes; ++p) {
        const cv::Mat probe = calibration_plane(source, state);
        const cv::Mat expected = reference(probe, wide);
        if (expected.size() != wide || expected.type() != CV_32F) return "reference_shape";
        for (int r = 0; r < 3; ++r) {
            const std::vector<RoundingRun> uniform{{0, width, kRoundings[r]}};
            for (int y = 0; y < source.height; ++y) {
                horizontal_pass(probe.ptr<float>(y), 0, plan.columns, uniform, 0, width, candidate.ptr<float>(y));
            }
            eliminate(horizontal, r, candidate, expected);
        }
    }
    // Vertical pass alone: resizing a (output width, source height) plane to the output runs the
    // same vertical pass and a horizontal pass with weights (1, 0), again value-preserving here.
    std::vector<std::uint8_t> vertical(static_cast<std::size_t>(width), 0x7U);
    cv::Mat blended(output, CV_32F);
    for (int p = 0; p < kVerticalPlanes; ++p) {
        const cv::Mat probe = calibration_plane(wide, state);
        const cv::Mat expected = reference(probe, output);
        if (expected.size() != output || expected.type() != CV_32F) return "reference_shape";
        for (int r = 0; r < 3; ++r) {
            const std::vector<RoundingRun> uniform{{0, width, kRoundings[r]}};
            for (int y = 0; y < output.height; ++y) {
                const auto dy = static_cast<std::size_t>(y);
                vertical_pass(probe.ptr<float>(plan.rows.row0[dy]), probe.ptr<float>(plan.rows.row1[dy]),
                              plan.rows.w0[dy], plan.rows.w1[dy], uniform, 0, width, blended.ptr<float>(y));
            }
            eliminate(vertical, r, blended, expected);
        }
    }
    if (!choose_runs(horizontal, plan.horizontal, plan.horizontal_ambiguous)) return "horizontal_rounding_unmatched";
    if (!choose_runs(vertical, plan.vertical, plan.vertical_ambiguous)) return "vertical_rounding_unmatched";
    // The whole plan against the full resize: logits-like planes at three scales, calibration
    // samples, and infinities, NaN and -0 next to the border.
    const cv::Rect whole(0, 0, output.width, output.height);
    for (int p = 0; p < kCheckPlanes; ++p) {
        cv::Mat plane = p % 2 == 0 ? logits_plane(source, state, p == 0 ? 1.0F : (p == 2 ? 8.0F : 64.0F))
                                   : calibration_plane(source, state);
        if (p >= 3) add_edge_specials(plane);
        if (!same_values(resize_linear_roi(plane, 0, 0, plan, whole), reference(plane, output))) return "check_mismatch";
    }
    return "";
}

cv::Mat opencv_linear_resize(const cv::Mat& source, const cv::Size& output) {
    cv::Mat resized;
    cv::resize(source, resized, output, 0.0, 0.0, cv::INTER_LINEAR);
    return resized;
}

}  // namespace

LinearBboxPlan calibrate_linear_bbox(const cv::Size& source, const cv::Size& output, const ResizeReference& reference) {
    const auto start = std::chrono::steady_clock::now();
    LinearBboxPlan plan;
    plan.source = source;
    plan.output = output;
    plan.reason = calibrate_into(plan, reference);
    plan.exact = plan.reason.empty();
    if (!plan.exact) {
        plan.horizontal.clear();
        plan.vertical.clear();
    }
    plan.calibration_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return plan;
}

LinearBboxPlan uniform_linear_bbox_plan(const cv::Size& source, const cv::Size& output, LinearRounding horizontal,
                                        LinearRounding vertical) {
    LinearBboxPlan plan;
    plan.source = source;
    plan.output = output;
    plan.columns = column_taps(source.width, output.width);
    plan.rows = row_taps(source.height, output.height);
    plan.horizontal = {{0, output.width, horizontal}};
    plan.vertical = {{0, output.width, vertical}};
    plan.exact = true;
    return plan;
}

std::string describe_linear_bbox_plan(const LinearBboxPlan& plan) {
    std::ostringstream text;
    text << "source=" << plan.source.width << "x" << plan.source.height << " output=" << plan.output.width << "x"
         << plan.output.height << " kernel=" << (plan.exact ? "bbox_exact" : "full_resize_fallback");
    if (!plan.exact) text << " reason=" << plan.reason;
    text << " calibration_ms=" << plan.calibration_ms;
    if (plan.exact) {
        text << " ambiguous_columns=" << plan.horizontal_ambiguous << "/" << plan.vertical_ambiguous << " horizontal";
        for (const RoundingRun& run : plan.horizontal) {
            text << " [" << run.begin << "," << run.end << ")=" << linear_rounding_name(run.rounding);
        }
        text << " vertical";
        for (const RoundingRun& run : plan.vertical) {
            text << " [" << run.begin << "," << run.end << ")=" << linear_rounding_name(run.rounding);
        }
    }
    return text.str();
}

// Keyed by the OpenCV thread count too: OpenCV 5 resizes an 8x upscale with another algorithm on
// one thread (found 2026-09-30 on the Mac), so a plan is only trusted for the count it was
// calibrated under. The service starts with its process count and decodes under the capture's
// count (set when the capture starts), so prewarm_active12_mask_kernels calibrates both.
namespace {
std::atomic<int> linear_bbox_calibrations{0};
}  // namespace

int linear_bbox_calibration_count() { return linear_bbox_calibrations.load(); }

const LinearBboxPlan& linear_bbox_plan(const cv::Size& source, const cv::Size& output) {
    struct Entry {
        int opencv_threads = 0;
        LinearBboxPlan plan;
    };
    static std::mutex mutex;
    static std::vector<std::unique_ptr<Entry>> plans;  // never erased: references stay valid
    const int opencv_threads = cv::getNumThreads();
    const Entry* entry = nullptr;
    bool created = false;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& known : plans) {
            if (known->plan.source == source && known->plan.output == output && known->opencv_threads == opencv_threads) {
                entry = known.get();
            }
        }
        if (entry == nullptr) {
            plans.push_back(std::make_unique<Entry>(Entry{opencv_threads, calibrate_linear_bbox(source, output, opencv_linear_resize)}));
            entry = plans.back().get();
            created = true;
            linear_bbox_calibrations.fetch_add(1);
        }
    }
    if (created) {
        LOG_INFO_M("Postprocess", "ACTIVE12_ROI_RESIZE opencv_threads=" << opencv_threads << " "
                   << describe_linear_bbox_plan(entry->plan));
    }
    return entry->plan;
}

bool resize_linear_bbox_part(const cv::Mat& part, int row0, int col0, const cv::Size& source,
                             const cv::Size& output, const cv::Rect& bbox, cv::Mat& out) {
    if (part.type() != CV_32FC1 || bbox.x < 0 || bbox.y < 0 || bbox.width <= 0 || bbox.height <= 0 ||
        bbox.x + bbox.width > output.width || bbox.y + bbox.height > output.height) {
        return false;
    }
    const LinearBboxPlan& plan = linear_bbox_plan(source, output);
    if (!plan.exact) return false;
    const cv::Rect held(col0, row0, part.cols, part.rows);
    const cv::Rect reads = linear_bbox_reads(plan, bbox);
    if ((reads & held) != reads) return false;
    out = resize_linear_roi(part, row0, col0, plan, bbox);
    return true;
}

cv::Mat resize_linear_bbox(const cv::Mat& source, const cv::Size& output, const cv::Rect& bbox) {
    cv::Mat out;
    if (resize_linear_bbox_part(source, 0, 0, source.size(), output, bbox, out)) return out;
    cv::Mat full;
    cv::resize(source, full, output, 0.0, 0.0, cv::INTER_LINEAR);
    return full(bbox);
}

}  // namespace detail

namespace {

// The bbox of the full-image resize of one instance's logits. A rectangle is resized in place when
// the calibrated kernel applies and every source element it reads lies inside the rectangle;
// otherwise the rectangle goes back into a zero plane and the plane path runs, so every branch
// yields the values of resizing the whole plane and cropping the bbox.
cv::Mat resize_active12_logits(const InstanceLogits& logits, const cv::Rect& native_mask_roi,
                               const cv::Size& output, const cv::Rect& bbox) {
    if (!logits.rectangle) return detail::resize_linear_bbox(logits.values(native_mask_roi), output, bbox);
    cv::Mat out;
    if (detail::resize_linear_bbox_part(logits.values, logits.row0 - native_mask_roi.y,
                                        logits.col0 - native_mask_roi.x, native_mask_roi.size(), output, bbox, out)) {
        return out;
    }
    cv::Mat plane = cv::Mat::zeros(logits.plane_rows, logits.plane_cols, CV_32F);
    logits.values.copyTo(plane(cv::Rect(logits.col0, logits.row0, logits.values.cols, logits.values.rows)));
    return detail::resize_linear_bbox(plane(native_mask_roi), output, bbox);
}

// One anchor that passed its class threshold. bbox is in letterbox (Active12, until after NMS) or
// original-image coordinates (COCO80); source_index is the anchor index (NMS tie-break).
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

// COCO80 duplicate rule: IoU above the threshold, or one box at least 90% inside the other.
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

// Half away from zero (std::round), clamped at 0: the COCO80 box edges.
int rounded_non_negative(float value) {
    return std::max(0, static_cast<int>(std::round(value)));
}

// Python/torch round(): half to even, clamped at 0 (non-finite gives 0).
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

// COCO80 box to pixels: each edge rounded on its own.
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

// sigmoid(x) > p  <=>  x > log(p / (1 - p)); the Active12 masks are thresholded as logits.
float probability_to_logit_threshold(float probability) {
    if (probability == 0.5F) {
        // Keep the sealed active12 threshold exactly equivalent to logits > 0.
        return 0.0F;
    }
    return static_cast<float>(
        std::log(static_cast<double>(probability)) -
        std::log1p(-static_cast<double>(probability)));
}

// The bbox-local mask pasted into a zero original-image canvas (clipped to the image).
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

// Structural checks of any ModelSpec; an Active12 spec must also equal the sealed one of its
// input shape exactly (a shape without a sealed spec is rejected the same way).
void validate_model_spec(const ModelSpec& spec) {
    if (spec.input_width <= 0 || spec.input_height <= 0 || spec.input_window_top < 0 || spec.num_classes <= 0 ||
        spec.mask_coefficient_count <= 0 ||
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
        const ModelSpec* sealed = sealed_active12_spec(spec.input_width, spec.input_height);
        if (sealed == nullptr ||
            spec.input_window_top != sealed->input_window_top ||
            spec.num_classes != sealed->num_classes ||
            spec.mask_coefficient_count != sealed->mask_coefficient_count ||
            spec.mask_coefficient_offset != sealed->mask_coefficient_offset ||
            spec.expected_detection_channels != sealed->expected_detection_channels ||
            spec.expected_num_boxes != sealed->expected_num_boxes ||
            spec.expected_proto_height != sealed->expected_proto_height ||
            spec.expected_proto_width != sealed->expected_proto_width ||
            spec.max_detections != sealed->max_detections ||
            spec.class_names != sealed->class_names ||
            spec.local_to_global_class_ids != sealed->local_to_global_class_ids) {
            throw std::invalid_argument("active12 ModelSpec differs from sealed deployment contract");
        }
    }
}

// Active12 NMS as Ultralytics: confidence order (ties by anchor index), class-aware IoU only,
// at most max_detections kept.
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

// Argmax over the 80 COCO80 class scores of one anchor (the first index wins a tie).
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

// A person anchor below its threshold is rejected, never relabelled as a weak generic obstacle.
bool is_weak_fallback_forbidden_class(int class_id) {
    constexpr int PERSON_CLASS_ID = 0;
    return class_id == PERSON_CLASS_ID;
}

enum class CandidateTier {
    rejected,
    normal,
    weak,
};

// The tier of an anchor whose raw top class and score are known (see select_candidate_class):
// normal when the score reaches its class threshold; weak when it is below that but reaches
// low_confidence_floor (floor > 0, not a person); otherwise rejected. The class is always the raw
// top class, never a runner-up that happens to pass its own threshold.
CandidateTier candidate_tier_for_top(
    int raw_top_class_id,
    float raw_top_conf,
    float global_threshold,
    const std::map<int, float>& per_class_conf,
    float low_confidence_floor,
    int& selected_class_id,
    float& selected_conf) {
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
    return candidate_tier_for_top(raw_top_class_id, raw_top_conf, global_threshold, per_class_conf,
                                  low_confidence_floor, selected_class_id, selected_conf);
}
// COCO80 NMS: normal candidates first, each tier by confidence. Two normal candidates suppress
// each other only within one class; a pair with a weak candidate ignores the (proxy) class. At
// most low_confidence_max_candidates weak candidates survive.
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

void prewarm_active12_mask_kernels(int proto_width, int proto_height, int original_width, int original_height) {
    if (proto_width <= 0 || proto_height <= 0 || original_width <= 0 || original_height <= 0) return;
    const cv::Rect native_mask_roi =
        ultralytics_v8466_native_mask_roi(proto_width, proto_height, original_width, original_height);
    if (native_mask_roi.empty()) return;
    if (active12_mask_rows_enabled()) (void)logits_rectangle_exact();
    // The resize plan is keyed by the OpenCV thread count. The service decodes under the count the
    // capture sets when it starts, which is after this call, so every count the service allows
    // (1 and 2, service_config.cpp) is calibrated now, plus the current one; the current count is
    // restored. Called from InferenceModule::start(), before the worker and the capture use OpenCV.
    const int previous = cv::getNumThreads();
    std::vector<int> counts{1, 2};
    if (previous != 1 && previous != 2) counts.push_back(previous);
    const cv::Size source = native_mask_roi.size();
    const cv::Size output(original_width, original_height);
    for (const int threads : counts) {
        cv::setNumThreads(threads);
        (void)detail::linear_bbox_plan(source, output);
    }
    cv::setNumThreads(previous);
}

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

// The COCO80 decoder used by the service. Per anchor: tier (see candidate_tier_for_top), box
// restored from the letterbox and clamped to the image, the 32 mask coefficients from channel 84
// (= 4 + 80). After NMS each survivor gets a bbox-local mask: the ROI path when enabled and
// applicable, else sigmoid of (coefficients x prototypes), resized to the input size, letterbox
// padding cropped, resized to the original image (ROI crop or full) and thresholded as a
// probability.
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
    last_work_counts = PostWorkCounts{};
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
    PostWorkCounts work;
    const int64_t num_boxes = detection_shape[2];
    const int proto_channels = static_cast<int>(prototype_shape[1]);
    const int proto_h = static_cast<int>(prototype_shape[2]);
    const int proto_w = static_cast<int>(prototype_shape[3]);
    const int proto_plane = proto_h * proto_w;
    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<std::size_t>(num_boxes));
    // Raw top class of every anchor in one class-major pass (same result as select_raw_top_class
    // per anchor, see top_class.hpp); the per-anchor tier logic below is unchanged.
    static thread_local std::vector<int> top_classes;
    static thread_local std::vector<float> top_scores;
    top_classes.resize(static_cast<std::size_t>(num_boxes));
    top_scores.resize(static_cast<std::size_t>(num_boxes));
    top_class_per_anchor(detection_data + (4 * num_boxes), num_boxes, 80, top_classes.data(), top_scores.data());

    for (int64_t box_idx = 0; box_idx < num_boxes; ++box_idx) {
        int best_class_id = -1;
        float best_conf = 0.0F;
        const CandidateTier tier = candidate_tier_for_top(
            top_classes[static_cast<std::size_t>(box_idx)],
            top_scores[static_cast<std::size_t>(box_idx)],
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

    work.candidates = static_cast<std::uint32_t>(candidates.size());
    const std::vector<Candidate> selected =
        select_candidates(std::move(candidates), iou_threshold, low_confidence_max_candidates);
    work.selected = static_cast<std::uint32_t>(selected.size());

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

        if (mask_resize_mode == CocoMaskResizeMode::roi && coco_mask_roi2_enabled()) {
            cv::Mat roi2_cropped;
            if (decode_coco_mask_roi2(proto, candidate.mask_coef.data(), proto_channels, proto_h, proto_w,
                                   info, input_size, detection.bbox, roi2_cropped)) {
                cv::Mat roi2_binary;
                cv::threshold(roi2_cropped, roi2_binary, mask_threshold, 255.0, cv::THRESH_BINARY);
                roi2_binary.convertTo(detection.mask, CV_8UC1);
                detection.source = DetectionSource::coco80;
                detection.original_class_id = candidate.class_id;
                const int nonzero = cv::countNonZero(detection.mask);
                total_mask_nonzero += nonzero;
                detections.push_back(std::move(detection));
                ++work.mask_roi2;
                continue;
            }
        }
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
            ++work.mask_full;
        } else {
            ++work.mask_roi;
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

    last_work_counts = work;
    return detections;
}

namespace {

std::int64_t mask_profile_now_ns() {
    const auto value = perception::monotonic_now_ns();
    if (value <= 0) throw std::runtime_error("Active12 mask profile clock unavailable");
    return value;
}

// Is the centre of anchor box_idx outside the resized image inside the letterbox? Anchor order is
// the three-level head's: stride 8, then 16, then 32, each level row by row (index in the level =
// row * grid width + column), centre = (column + 0.5, row + 0.5) * stride. The caller has checked
// that the three grids add up to the candidate count.
bool anchor_centre_in_padding(std::int64_t box_idx, ModelInputShape input_shape, const LetterboxInfo& info) noexcept {
    for (const int stride : {8, 16, 32}) {
        const std::int64_t grid_width = input_shape.width / stride;
        const std::int64_t grid_height = input_shape.height / stride;
        if (box_idx < grid_width * grid_height) {
            const float cx = (static_cast<float>(box_idx % grid_width) + 0.5F) * static_cast<float>(stride);
            const float cy = (static_cast<float>(box_idx / grid_width) + 0.5F) * static_cast<float>(stride);
            return cx < static_cast<float>(info.pad_left) ||
                   cx >= static_cast<float>(info.pad_left + info.new_width) ||
                   cy < static_cast<float>(info.pad_top) ||
                   cy >= static_cast<float>(info.pad_top + info.new_height);
        }
        box_idx -= grid_width * grid_height;
    }
    return false;
}

// The Active12 decoder, Ultralytics 8.4.66 semantics: per anchor the argmax over the finite class
// scores, threshold, the raw box kept in letterbox space through pure class-aware NMS, then
// mapped to the original image and clipped. Masks: logits (not sigmoid) of the prototype ROI,
// resized straight to the original image, cropped to the box and compared with the logit of
// mask_threshold; the crop rounding depends on the detection count (to_ultralytics_cpu_crop_rect).
// With mask_lanes the instances are split even/odd over two dedicated lanes; output order and
// values equal the serial loop below.
// padding_dropped (optional): the padding-anchor guard, see Active12PaddingGuard; the number of
// dropped candidates is written (not added) there.
std::vector<Detection> postprocess_active12_impl(
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
    perception::PersistentTaskPair* mask_lanes,
    Active12MaskProfile* profile,
    Active12MaskBinaryImplementation binary_implementation,
    bool execution_cpu_trace,
    std::size_t* padding_dropped) {
    if (profile) *profile = Active12MaskProfile{};
    if (padding_dropped) *padding_dropped = 0;
    const auto total_start_ns = profile ? mask_profile_now_ns() : 0;
    validate_model_spec(model_spec);
    last_work_counts = PostWorkCounts{};
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
    if (input_shape.width != model_spec.input_width || input_shape.height != model_spec.input_height ||
        info.scale <= 0.0F ||
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
    // An input that is a window of the square letterbox: the letterbox must be the centred square
    // one seen through that window, with the whole image content inside it.
    if (model_spec.input_window_top > 0 &&
        (info.pad_left != (model_spec.input_width - info.new_width) / 2 ||
         info.pad_top != ((model_spec.input_width - info.new_height) / 2) - model_spec.input_window_top ||
         info.pad_top < 0 || info.pad_top + info.new_height > model_spec.input_height)) {
        throw std::invalid_argument("ModelSpec postprocessSeg letterbox does not match the model input window");
    }

    const std::int64_t num_boxes = detection_shape[2];
    // The guard needs the anchor grid: refuse a candidate count that is not the three-level
    // layout of this input shape rather than guard the wrong anchors.
    if (padding_dropped) {
        std::int64_t expected = 0;
        for (const int stride : {8, 16, 32}) {
            expected += static_cast<std::int64_t>(input_shape.width / stride) * (input_shape.height / stride);
        }
        if (input_shape.width % 32 != 0 || input_shape.height % 32 != 0 || expected != num_boxes) {
            throw std::invalid_argument(
                "Active12 padding-anchor guard needs the stride 8/16/32 anchor layout");
        }
    }
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
        if (padding_dropped && anchor_centre_in_padding(box_idx, input_shape, info)) {
            ++*padding_dropped;
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
        // Keep the raw xyxy box in the letterbox coordinate system of the model input until
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

    PostWorkCounts work;
    work.candidates = static_cast<std::uint32_t>(candidates.size());
    const std::vector<Candidate> selected = select_pure_class_aware_candidates(
        std::move(candidates),
        iou_threshold,
        model_spec.max_detections);
    work.selected = static_cast<std::uint32_t>(selected.size());
    last_work_counts = work;  // before the mask decode, whose lanes may run on other threads

    const int proto_channels = static_cast<int>(prototype_shape[1]);
    const int proto_height = static_cast<int>(prototype_shape[2]);
    const int proto_width = static_cast<int>(prototype_shape[3]);
    const int proto_plane = proto_height * proto_width;
    cv::Mat proto(
        proto_channels,
        proto_plane,
        CV_32F,
        const_cast<float*>(prototype_data));

    // Ultralytics computes the prototype crop for a centred letterbox. For a window input that is
    // the crop on the square prototype plane, moved up by the window's first row (one prototype
    // row per 4 input rows); for a square input both lines are the call as it always was.
    const int window_proto_rows = model_spec.input_window_top / 4;
    cv::Rect native_mask_roi = ultralytics_v8466_native_mask_roi(
        proto_width,
        window_proto_rows > 0 ? proto_width : proto_height,
        info.original_width,
        info.original_height);
    if (native_mask_roi.empty()) {
        throw std::invalid_argument("ModelSpec postprocessSeg has empty native mask ROI");
    }
    native_mask_roi.y -= window_proto_rows;
    if (native_mask_roi.y < 0 || native_mask_roi.y + native_mask_roi.height > proto_height) {
        throw std::invalid_argument("ModelSpec postprocessSeg image content lies outside the model input window");
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

    const auto batch_start_ns = profile ? mask_profile_now_ns() : 0;
    if (mask_lanes) {
        std::vector<Detection> detections;
        detections.reserve(mapped.size());
        std::vector<const Candidate*> jobs;
        jobs.reserve(mapped.size());
        std::size_t first_traversable = std::numeric_limits<std::size_t>::max();
        // Decide output order, crop-count rounding and first-92 ownership
        // serially. Neither vector changes size while the two lanes run.
        for (const Candidate& candidate : mapped) {
            Detection detection;
            detection.bbox = rect_within_image(
                to_ultralytics_cpu_crop_rect(candidate.bbox, mapped.size()),
                info.original_width,
                info.original_height);
            if (detection.bbox.empty()) continue;
            detection.confidence = candidate.conf;
            detection.class_id = candidate.class_id;
            detection.original_class_id = candidate.original_class_id;
            detection.class_name =
                model_spec.class_names[static_cast<std::size_t>(candidate.original_class_id)];
            detection.low_confidence_candidate = false;
            detection.source = model_spec.source;
            if (detection.class_id == 92 && first_traversable == std::numeric_limits<std::size_t>::max()) {
                first_traversable = detections.size();
            }
            detections.push_back(std::move(detection));
            jobs.push_back(&candidate);
        }
        if (profile) profile->tasks.resize(jobs.size());
        const auto decode_lane = [&](std::size_t lane) {
            for (std::size_t index = lane; index < jobs.size(); index += 2U) {
                const Candidate& candidate = *jobs[index];
                Detection& detection = detections[index];
                const auto started_ns = profile ? mask_profile_now_ns() : 0;
                perception::ExecutionCpuRecord execution_cpu;
                perception::ExecutionCpuScope execution_scope(execution_cpu,
                    execution_cpu_trace && profile != nullptr);
                const InstanceLogits mask_raw = active12_logits_for_bbox(proto, candidate.mask_coef.data(),
                    proto_channels, proto_height, proto_width, native_mask_roi, info, detection.bbox);
                const auto gemm_done_ns = profile ? mask_profile_now_ns() : 0;

                // Same values as resizing the whole image and cropping the bbox (exact bbox kernel).
                const cv::Mat bbox_mask_logits = resize_active12_logits(mask_raw, native_mask_roi,
                    cv::Size(info.original_width, info.original_height), detection.bbox);
                const auto resize_done_ns = profile ? mask_profile_now_ns() : 0;

                if (binary_implementation == Active12MaskBinaryImplementation::Compare) {
                    if (mask_threshold <= 0.0F) {
                        detection.mask = cv::Mat(bbox_mask_logits.size(), CV_8UC1, cv::Scalar(255));
                    } else if (mask_threshold >= 1.0F) {
                        detection.mask = cv::Mat::zeros(bbox_mask_logits.size(), CV_8UC1);
                    } else {
                        cv::compare(bbox_mask_logits, probability_to_logit_threshold(mask_threshold),
                                    detection.mask, cv::CMP_GT);
                    }
                } else {
                    // Default Legacy retains the original two-step operation,
                    // including convertTo after either endpoint branch.
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
                }
                const auto threshold_done_ns = profile ? mask_profile_now_ns() : 0;
                if (index == first_traversable) {
                    detection.full_image_mask = make_full_image_mask(
                        detection.mask,
                        detection.bbox,
                        info.original_width,
                        info.original_height);
                }
                if (profile) {
                    execution_scope.finish();
                    const auto done_ns = mask_profile_now_ns();
                    Active12MaskTaskProfile task;
                    task.execution_cpu = execution_cpu;
                    task.index = index;
                    task.bbox_pixels = static_cast<std::int64_t>(detection.bbox.width) * detection.bbox.height;
                    task.gemm_ns = gemm_done_ns - started_ns;
                    task.resize_ns = resize_done_ns - gemm_done_ns;
                    task.threshold_convert_ns = threshold_done_ns - resize_done_ns;
                    task.canvas_ns = done_ns - threshold_done_ns;
                    task.total_ns = done_ns - started_ns;
                    task.lane = static_cast<int>(lane);
                    profile->tasks[index] = task;
                }
            }
        };
        if (jobs.size() >= 2U) {
            if (profile) profile->parallel_used = true;
            // This dedicated pair is never the outer COCO/Active or submit
            // pair. Both lanes finish before returning or rethrowing.
            mask_lanes->run([&]() { decode_lane(1); }, [&]() { decode_lane(0); });
        } else {
            decode_lane(0);
        }
        if (profile) {
            const auto done_ns = mask_profile_now_ns();
            profile->instances = detections.size();
            profile->batch_wall_ns = done_ns - batch_start_ns;
            profile->total_ns = done_ns - total_start_ns;
        }
        return detections;
    }

    std::vector<Detection> detections;
    detections.reserve(mapped.size());
    if (profile) profile->tasks.reserve(mapped.size());
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

        const auto started_ns = profile ? mask_profile_now_ns() : 0;
        perception::ExecutionCpuRecord execution_cpu;
        perception::ExecutionCpuScope execution_scope(execution_cpu,
            execution_cpu_trace && profile != nullptr);
        const InstanceLogits mask_raw = active12_logits_for_bbox(proto, candidate.mask_coef.data(),
            proto_channels, proto_height, proto_width, native_mask_roi, info, detection.bbox);
        const auto gemm_done_ns = profile ? mask_profile_now_ns() : 0;

        // Match Ultralytics 8.4.66 process_mask_native: interpolate logits, not
        // sigmoid probabilities, and perform one resize from the cropped
        // prototype directly to original resolution.
        // The bbox of that full-resolution resize, computed on the bbox only (exact kernel above).
        const cv::Mat bbox_mask_logits = resize_active12_logits(mask_raw, native_mask_roi,
            cv::Size(info.original_width, info.original_height), detection.bbox);

        const auto resize_done_ns = profile ? mask_profile_now_ns() : 0;
        if (binary_implementation == Active12MaskBinaryImplementation::Compare) {
            if (mask_threshold <= 0.0F) {
                detection.mask = cv::Mat(bbox_mask_logits.size(), CV_8UC1, cv::Scalar(255));
            } else if (mask_threshold >= 1.0F) {
                detection.mask = cv::Mat::zeros(bbox_mask_logits.size(), CV_8UC1);
            } else {
                cv::compare(bbox_mask_logits, probability_to_logit_threshold(mask_threshold),
                            detection.mask, cv::CMP_GT);
            }
        } else {
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
        }
        const auto threshold_done_ns = profile ? mask_profile_now_ns() : 0;
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
        if (profile) {
            execution_scope.finish();
            const auto done_ns = mask_profile_now_ns();
            Active12MaskTaskProfile task;
            task.execution_cpu = execution_cpu;
            task.index = detections.size();
            task.bbox_pixels = static_cast<std::int64_t>(detection.bbox.width) * detection.bbox.height;
            task.gemm_ns = gemm_done_ns - started_ns;
            task.resize_ns = resize_done_ns - gemm_done_ns;
            task.threshold_convert_ns = threshold_done_ns - resize_done_ns;
            task.canvas_ns = done_ns - threshold_done_ns;
            task.total_ns = done_ns - started_ns;
            profile->tasks.push_back(task);
        }
        detections.push_back(std::move(detection));
    }

    if (profile) {
        const auto done_ns = mask_profile_now_ns();
        profile->instances = detections.size();
        profile->batch_wall_ns = done_ns - batch_start_ns;
        profile->total_ns = done_ns - total_start_ns;
    }
    return detections;
}

bool same_float_bits(float left, float right) {
    std::uint32_t left_bits = 0;
    std::uint32_t right_bits = 0;
    std::memcpy(&left_bits, &left, sizeof(left_bits));
    std::memcpy(&right_bits, &right, sizeof(right_bits));
    return left_bits == right_bits;
}

// Shadow modes: every field of both results must be identical, else this throws and the frame
// fails (the JIT loop then stops); a near match is still a failure.
void compare_active12_masks(
    const std::vector<Detection>& old_result,
    const std::vector<Detection>& parallel_result) {
    if (old_result.size() != parallel_result.size()) {
        throw std::runtime_error("ACTIVE12_MASK_SHADOW_MISMATCH active12_mask_shadow_mismatch detection_count");
    }
    for (std::size_t index = 0; index < old_result.size(); ++index) {
        const Detection& left = old_result[index];
        const Detection& right = parallel_result[index];
        const auto require = [index](bool equal, const char* field) {
            if (!equal) {
                throw std::runtime_error("ACTIVE12_MASK_SHADOW_MISMATCH active12_mask_shadow_mismatch index=" +
                    std::to_string(index) + " field=" + field);
            }
        };
        require(left.bbox == right.bbox, "bbox");
        require(same_float_bits(left.confidence, right.confidence), "confidence");
        require(left.class_id == right.class_id, "class_id");
        require(left.class_name == right.class_name, "class_name");
        require(left.low_confidence_candidate == right.low_confidence_candidate, "low_confidence_candidate");
        require(left.source == right.source, "source");
        require(left.original_class_id == right.original_class_id, "original_class_id");
        const auto compare_mat = [&](const cv::Mat& a, const cv::Mat& b, const char* field) {
            require(a.dims == b.dims && a.rows == b.rows && a.cols == b.cols &&
                a.type() == b.type() && a.empty() == b.empty(), field);
            if (a.empty()) return;
            require(a.dims == 2, field);
            const auto row_bytes = static_cast<std::size_t>(a.cols) * a.elemSize();
            for (int row = 0; row < a.rows; ++row) {
                require(std::memcmp(a.ptr(row), b.ptr(row), row_bytes) == 0, field);
            }
        };
        compare_mat(left.mask, right.mask, "mask");
        compare_mat(left.full_image_mask, right.full_image_mask, "full_image_mask");
    }
}

}  // namespace

Active12MaskBinaryImplementation parse_active12_mask_binary_implementation(const std::string& value) {
    if (value == "legacy") return Active12MaskBinaryImplementation::Legacy;
    if (value == "compare") return Active12MaskBinaryImplementation::Compare;
    if (value == "shadow") return Active12MaskBinaryImplementation::Shadow;
    throw std::invalid_argument("PERCEPTION_ACTIVE12_MASK_BINARY_IMPL must be legacy, compare or shadow");
}

const char* active12_mask_binary_implementation_name(Active12MaskBinaryImplementation implementation) {
    switch (implementation) {
        case Active12MaskBinaryImplementation::Legacy: return "legacy";
        case Active12MaskBinaryImplementation::Compare: return "compare";
        case Active12MaskBinaryImplementation::Shadow: return "shadow";
    }
    throw std::invalid_argument("invalid Active12 mask binary implementation");
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
    ModelInputShape input_shape,
    Active12MaskImplementation implementation,
    perception::PersistentTaskPair* mask_lanes,
    Active12MaskProfile* profile,
    Active12MaskBinaryImplementation binary_implementation,
    bool execution_cpu_trace,
    Active12PaddingGuard* padding_guard) {
    // Dispatch on the two switches: Serial or Parallel2 run once; Shadow runs serial and parallel
    // and compares; binary Shadow runs Legacy and Compare (each in the chosen mode) and returns
    // Legacy. The padding guard applies to every run alike (each writes the same count).
    if (profile) *profile = Active12MaskProfile{};
    if (implementation != Active12MaskImplementation::Serial &&
        implementation != Active12MaskImplementation::Parallel2 &&
        implementation != Active12MaskImplementation::Shadow) {
        throw std::invalid_argument("invalid Active12 mask implementation");
    }
    if (implementation != Active12MaskImplementation::Serial &&
        (mask_lanes == nullptr || model_spec.source != DetectionSource::active12)) {
        throw std::invalid_argument("parallel Active12 mask path requires its dedicated CPU lanes and Active12 ModelSpec");
    }
    if (binary_implementation != Active12MaskBinaryImplementation::Legacy &&
        binary_implementation != Active12MaskBinaryImplementation::Compare &&
        binary_implementation != Active12MaskBinaryImplementation::Shadow) {
        throw std::invalid_argument("invalid Active12 mask binary implementation");
    }
    if (binary_implementation != Active12MaskBinaryImplementation::Legacy &&
        model_spec.source != DetectionSource::active12) {
        throw std::invalid_argument("non-legacy Active12 mask binary path requires Active12 ModelSpec");
    }
    const auto sample = [&](perception::PersistentTaskPair* lanes, Active12MaskProfile* timing,
                            Active12MaskBinaryImplementation binary) {
        return postprocess_active12_impl(detection_data, detection_shape, prototype_data,
            prototype_shape, info, model_spec, conf_threshold, iou_threshold,
            mask_threshold, input_shape, lanes, timing, binary, execution_cpu_trace,
            padding_guard ? &padding_guard->dropped : nullptr);
    };
    const auto execute = [&](Active12MaskBinaryImplementation binary, Active12MaskProfile* timing) {
        if (implementation == Active12MaskImplementation::Serial) return sample(nullptr, timing, binary);
        if (implementation == Active12MaskImplementation::Parallel2) return sample(mask_lanes, timing, binary);
        auto serial_result = sample(nullptr, timing, binary);
        const auto parallel_result = sample(mask_lanes, nullptr, binary);
        compare_active12_masks(serial_result, parallel_result);
        if (timing) timing->shadow_checked = true;
        return serial_result;
    };
    if (binary_implementation != Active12MaskBinaryImplementation::Shadow) {
        return execute(binary_implementation, profile);
    }
    // Full sequential calls own separate outputs/profiles. Binary shadow never
    // substitutes Compare output and never publishes either call by itself.
    Active12MaskProfile legacy_profile;
    Active12MaskProfile compare_profile;
    auto legacy_result = execute(Active12MaskBinaryImplementation::Legacy, &legacy_profile);
    const auto compare_result = execute(Active12MaskBinaryImplementation::Compare, &compare_profile);
    compare_active12_masks(legacy_result, compare_result);
    if (profile) {
        *profile = std::move(legacy_profile);
        profile->binary_shadow_checked = true;
    }
    return legacy_result;
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
    int input_size,
    Active12MaskImplementation implementation,
    perception::PersistentTaskPair* mask_lanes,
    Active12MaskProfile* profile,
    Active12MaskBinaryImplementation binary_implementation,
    bool execution_cpu_trace,
    Active12PaddingGuard* padding_guard) {
    return postprocess_seg(detection_data, detection_shape, prototype_data, prototype_shape, info, model_spec,
                           conf_threshold, iou_threshold, mask_threshold, ModelInputShape{input_size, input_size},
                           implementation, mask_lanes, profile, binary_implementation, execution_cpu_trace,
                           padding_guard);
}

}  // namespace yolo
