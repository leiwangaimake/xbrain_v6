/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: linear_bbox_resize.hpp
 * Brief: The bbox of a full-image INTER_LINEAR upscale, computed on the bbox only.
 *
 * Description:
 * The Active12 mask decode resizes the prototype logits to the whole original image and keeps
 * only the bbox. This kernel computes that bbox directly, with the interpolation tables of
 * OpenCV's resizeGeneric and its two passes (horizontal per source row, then vertical). What
 * differs between OpenCV builds, and between the SIMD body and the scalar tail of a pass, is how
 * s0*w0 + s1*w1 is rounded: two rounded products (plain) or one product fused into the sum.
 * The rounding of every output column is therefore calibrated per pass against the machine's
 * cv::resize, and the whole plan is then checked against cv::resize on further planes (with
 * infinities, NaN and -0 at the edges). The kernel is used only when all of that matches bit for
 * bit; otherwise the full resize stays. These declarations are internal to the decoder; they are
 * public so that tests/test_linear_bbox_resize.cpp can check the calibration against a simulated
 * OpenCV and the kernel against this machine's cv::resize.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace yolo::detail {

enum class LinearRounding : std::uint8_t { Plain, FuseFirst, FuseSecond };

const char* linear_rounding_name(LinearRounding rounding);

// Output columns [begin, end) that share one rounding in one pass.
struct RoundingRun {
    int begin = 0, end = 0;
    LinearRounding rounding = LinearRounding::Plain;
};

// resizeGeneric's horizontal table: source column and weights per output column. From `limit`
// (xmax) on, OpenCV copies the source sample instead of blending.
struct ColumnTaps {
    std::vector<int> index;
    std::vector<float> w0, w1;
    int limit = 0;
};

// resizeGeneric's vertical table: the two source rows each output row blends (already clipped to
// the image, so both can be the same edge row) and their weights.
struct RowTaps {
    std::vector<int> row0, row1;
    std::vector<float> w0, w1;
};

struct LinearBboxPlan {
    cv::Size source, output;
    bool exact = false;
    ColumnTaps columns;
    RowTaps rows;
    std::vector<RoundingRun> horizontal, vertical;
    // Columns whose rounding the calibration could not narrow to one (all remaining roundings gave
    // the same values on every plane); they take their left neighbour's rounding.
    int horizontal_ambiguous = 0, vertical_ambiguous = 0;
    double calibration_ms = 0.0;
    std::string reason;  // why the kernel is not used (empty when exact)
};

// The reference the plan is calibrated against: cv::resize(source, dst, output, INTER_LINEAR).
using ResizeReference = std::function<cv::Mat(const cv::Mat& source, const cv::Size& output)>;

LinearBboxPlan calibrate_linear_bbox(const cv::Size& source, const cv::Size& output, const ResizeReference& reference);

// A plan with one rounding for every column of each pass (the test's simulated OpenCV builds).
LinearBboxPlan uniform_linear_bbox_plan(const cv::Size& source, const cv::Size& output, LinearRounding horizontal,
                                        LinearRounding vertical);

// The bbox `roi` of the resize under `plan` (roi in output coordinates). `part` holds the source
// from (row0, col0) on and must cover linear_bbox_reads(plan, roi).
cv::Mat resize_linear_roi(const cv::Mat& part, int row0, int col0, const LinearBboxPlan& plan, const cv::Rect& roi);
cv::Rect linear_bbox_reads(const LinearBboxPlan& plan, const cv::Rect& roi);

// Process-wide plan per size pair and OpenCV thread count, calibrated against cv::resize on first
// use (logged once as ACTIVE12_ROI_RESIZE). The decoder warms it before the first frame.
const LinearBboxPlan& linear_bbox_plan(const cv::Size& source, const cv::Size& output);
// Number of plans calibrated in this process (each size pair and OpenCV thread count once).
int linear_bbox_calibration_count();
std::string describe_linear_bbox_plan(const LinearBboxPlan& plan);

// The bbox of cv::resize(source, output, INTER_LINEAR) for a CV_32FC1 source: the kernel when its
// plan is exact, else the full resize and a view of it.
cv::Mat resize_linear_bbox(const cv::Mat& source, const cv::Size& output, const cv::Rect& bbox);

// The same bbox from `part`, which holds the source (full size `source`) from (row0, col0) on.
// False, with `out` untouched, when the plan is not exact or the bbox reads outside `part`.
bool resize_linear_bbox_part(const cv::Mat& part, int row0, int col0, const cv::Size& source,
                             const cv::Size& output, const cv::Rect& bbox, cv::Mat& out);

}  // namespace yolo::detail
