/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_linear_bbox_resize.cpp
 * Brief: The bbox resize kernel equals cv::resize + crop value for value.
 *
 * Description:
 * Its calibration finds the rounding of every column.
 * Part 1 (any machine): OpenCV builds are simulated by resizes with a known rounding per column
 * (uniform for each of the nine pass pairs, and the layouts seen on the Mac and on 1.8). The
 * calibration must yield a plan that reproduces the simulated build on fresh logits-like planes;
 * a plan with one wrong column must be caught by the same planes (so the check can see a
 * one-column error), and an unrepresentable build must fall back.
 * Part 2 (this machine): the plan against the real cv::resize must be exact for the production
 * size (224x140 -> 1280x800), and the kernel must equal cv::resize + crop on logits-like planes,
 * calibration samples and planes with infinities, NaN and -0 next to the border, for edge,
 * one-pixel and random bboxes, calibrated with 1 or 2 OpenCV threads and used with 1 or 2 (OpenCV
 * 5 takes another path for an 8x upscale on one thread; other sizes may therefore fall back).
 * Values compare by bits, NaN equal to NaN (the masks only compare against a threshold). Same size
 * and downscale must fall back to cv::resize.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/linear_bbox_resize.hpp"
#include "common/postprocess.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {

using yolo::detail::LinearBboxPlan;
using yolo::detail::LinearRounding;
using yolo::detail::RoundingRun;

int checks = 0;

void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

bool same_value(float a, float b) {
    return std::memcmp(&a, &b, sizeof(float)) == 0 || (std::isnan(a) && std::isnan(b));
}

// Number of differing elements (bits, NaN == NaN); -1 on a size mismatch.
long differing(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size()) return -1;
    long count = 0;
    for (int y = 0; y < a.rows; ++y) {
        for (int x = 0; x < a.cols; ++x) {
            if (!same_value(a.at<float>(y, x), b.at<float>(y, x))) ++count;
        }
    }
    return count;
}

// Logits-like values: 32-term dot products of uniform values, summed in double, rounded once.
cv::Mat logits_plane(const cv::Size& size, std::mt19937& random, float scale) {
    std::uniform_real_distribution<float> uniform(-1.0F, 1.0F);
    cv::Mat plane(size, CV_32F);
    for (int y = 0; y < size.height; ++y) {
        for (int x = 0; x < size.width; ++x) {
            double sum = 0.0;
            for (int k = 0; k < 32; ++k) sum += static_cast<double>(uniform(random)) * (uniform(random) * scale);
            plane.at<float>(y, x) = static_cast<float>(sum);
        }
    }
    return plane;
}

// Random sign, full mantissa, exponent in [-12, 12].
cv::Mat spread_plane(const cv::Size& size, std::mt19937& random) {
    std::uniform_int_distribution<std::uint32_t> bits;
    cv::Mat plane(size, CV_32F);
    for (int y = 0; y < size.height; ++y) {
        for (int x = 0; x < size.width; ++x) {
            const std::uint32_t b = bits(random);
            const float mantissa = 1.0F + static_cast<float>(b & 0x7FFFFFU) / 8388608.0F;
            const float value = std::ldexp(mantissa, static_cast<int>((b >> 23U) % 25U) - 12);
            plane.at<float>(y, x) = (b >> 31U) != 0U ? -value : value;
        }
    }
    return plane;
}

// Infinities, NaN and -0 on the two rows and columns at each border.
void add_specials(cv::Mat& plane) {
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const int h = plane.rows, w = plane.cols;
    for (int x = 0; x < w; x += 9) {
        plane.at<float>(1, x) = inf;
        plane.at<float>(0, (x + 4) % w) = nan;
        plane.at<float>(h - 1, (x + 2) % w) = -inf;
        plane.at<float>(h - 2, (x + 6) % w) = nan;
    }
    for (int y = 0; y < h; y += 4) {
        plane.at<float>(y, 0) = -inf;
        plane.at<float>(y, 1) = nan;
        plane.at<float>((y + 2) % h, w - 1) = inf;
        plane.at<float>((y + 1) % h, w - 2) = nan;
    }
    // -0 runs at the top and bottom edge rows (the edge rows are blended with themselves).
    for (int x = 20; x < std::min(w, 40); ++x) {
        plane.at<float>(0, x) = -0.0F;
        plane.at<float>(h - 1, x) = -0.0F;
    }
}

cv::Mat opencv_bbox(const cv::Mat& source, const cv::Size& output, const cv::Rect& bbox) {
    cv::Mat full;
    cv::resize(source, full, output, 0.0, 0.0, cv::INTER_LINEAR);
    return full(bbox).clone();
}

// ---- Part 1: simulated OpenCV builds ----

// A simulated build: the rounding of every column of each pass, as a function of the output
// width and the horizontal copy limit (xmax).
struct SimulatedBuild {
    std::string name;
    std::vector<RoundingRun> (*horizontal)(int width, int limit);
    std::vector<RoundingRun> (*vertical)(int width, int limit);
};

template <LinearRounding R>
std::vector<RoundingRun> uniform_runs(int width, int) {
    return {{0, width, R}};
}

// The Mac (OpenCV 5, clang): plain in the 4-lane body, fused in the scalar tail before xmax.
std::vector<RoundingRun> mac_horizontal(int width, int limit) {
    const int body = limit & ~3;
    std::vector<RoundingRun> runs{{0, body, LinearRounding::Plain}};
    if (limit > body) runs.push_back({body, limit, LinearRounding::FuseFirst});
    if (width > limit) runs.push_back({limit, width, LinearRounding::Plain});
    return runs;
}

cv::Mat simulated_resize(const SimulatedBuild& build, const cv::Mat& source, const cv::Size& output) {
    LinearBboxPlan plan = yolo::detail::uniform_linear_bbox_plan(source.size(), output, LinearRounding::Plain,
                                                                 LinearRounding::Plain);
    plan.horizontal = build.horizontal(output.width, plan.columns.limit);
    plan.vertical = build.vertical(output.width, plan.columns.limit);
    return yolo::detail::resize_linear_roi(source, 0, 0, plan, cv::Rect(0, 0, output.width, output.height));
}

void simulated_builds(const cv::Size& source, const cv::Size& output, const std::vector<SimulatedBuild>& builds) {
    std::mt19937 random(20260930U);
    const cv::Rect whole(0, 0, output.width, output.height);
    for (const SimulatedBuild& build : builds) {
        const auto reference = [&build](const cv::Mat& s, const cv::Size& o) { return simulated_resize(build, s, o); };
        const LinearBboxPlan plan = yolo::detail::calibrate_linear_bbox(source, output, reference);
        const std::string label = build.name + " " + yolo::detail::describe_linear_bbox_plan(plan);
        check(plan.exact, "simulated build not calibrated: " + label);
        long total = 0;
        for (int p = 0; p < 12; ++p) {
            const cv::Mat plane = p < 8 ? logits_plane(source, random, p < 4 ? 1.0F : 16.0F) : spread_plane(source, random);
            total += differing(yolo::detail::resize_linear_roi(plane, 0, 0, plan, whole), reference(plane, output));
        }
        check(total == 0, "calibrated plan differs from the simulated build: " + label + " differing=" + std::to_string(total));
        std::cout << "simulated " << build.name << " " << source.width << "x" << source.height << "->" << output.width
                  << "x" << output.height << " exact runs=" << plan.horizontal.size() << "/" << plan.vertical.size()
                  << " ambiguous=" << plan.horizontal_ambiguous << "/" << plan.vertical_ambiguous
                  << " calibration_ms=" << plan.calibration_ms << "\n";
    }
}

// The fresh planes above must be able to see one wrong column: on the 1.8-like build (fused
// everywhere), a plan with plain rounding in one column whose weight is about 0.9875 (column 48 of
// 224 -> 1280, the case the first calibration got wrong) must differ.
void one_wrong_column_is_visible() {
    const cv::Size source(224, 140), output(1280, 800);
    const SimulatedBuild fused{"fused", uniform_runs<LinearRounding::FuseFirst>, uniform_runs<LinearRounding::FuseFirst>};
    LinearBboxPlan wrong = yolo::detail::uniform_linear_bbox_plan(source, output, LinearRounding::FuseFirst,
                                                                  LinearRounding::FuseFirst);
    wrong.horizontal = {{0, 48, LinearRounding::FuseFirst}, {48, 49, LinearRounding::Plain},
                        {49, output.width, LinearRounding::FuseFirst}};
    std::mt19937 random(7U);
    long total = 0;
    for (int p = 0; p < 12; ++p) {
        const cv::Mat plane = p < 8 ? logits_plane(source, random, p < 4 ? 1.0F : 16.0F) : spread_plane(source, random);
        total += differing(yolo::detail::resize_linear_roi(plane, 0, 0, wrong, cv::Rect(0, 0, 1280, 800)),
                           simulated_resize(fused, plane, output));
    }
    check(total > 0, "a plain column 48 on a fused build is not visible on the check planes");
    std::cout << "one wrong column (48, plain on fused): differing=" << total << " over 12 planes\n";
}

// A build that blends in double (not one of the three roundings) must fall back.
void unrepresentable_build_falls_back() {
    const cv::Size source(224, 140), output(1280, 800);
    const auto reference = [](const cv::Mat& s, const cv::Size& o) {
        cv::Mat d;
        s.convertTo(d, CV_64F);
        cv::Mat r;
        cv::resize(d, r, o, 0.0, 0.0, cv::INTER_LINEAR);
        cv::Mat f;
        r.convertTo(f, CV_32F);
        return f;
    };
    const LinearBboxPlan plan = yolo::detail::calibrate_linear_bbox(source, output, reference);
    check(!plan.exact, "a double-precision build was accepted: " + yolo::detail::describe_linear_bbox_plan(plan));
    std::cout << "double-precision build: " << yolo::detail::describe_linear_bbox_plan(plan) << "\n";
}

// ---- Part 2: this machine's cv::resize ----

std::vector<cv::Rect> test_boxes(const cv::Size& output, std::mt19937& random) {
    const int w = output.width, h = output.height;
    std::vector<cv::Rect> boxes{{0, 0, w, h}, {0, 0, 1, 1}, {w - 1, h - 1, 1, 1}, {0, 0, w, 3}, {0, h - 3, w, 3},
                                {0, 0, 3, h}, {w - 3, 0, 3, h}, {w - 5, h - 5, 5, 5}, {0, h - 2, 2, 2}, {w - 2, 0, 2, 2}};
    std::uniform_int_distribution<int> any_x(0, w - 1), any_y(0, h - 1);
    for (int i = 0; i < 60; ++i) {
        const int x = any_x(random), y = any_y(random);
        std::uniform_int_distribution<int> bw(1, w - x), bh(1, h - y);
        boxes.emplace_back(x, y, bw(random), bh(random));
    }
    return boxes;
}

cv::Mat opencv_resize(const cv::Mat& source, const cv::Size& output) {
    cv::Mat resized;
    cv::resize(source, resized, output, 0.0, 0.0, cv::INTER_LINEAR);
    return resized;
}

// A plan calibrated with `calibration_threads` OpenCV threads, used with `use_threads` (the service
// warms the plan at start and decodes under the capture's thread limit). `required`: the production
// size, where the kernel must be in use; elsewhere a fallback is accepted (and then equals
// cv::resize by construction).
void real_resize(const cv::Size& source, const cv::Size& output, int calibration_threads, int use_threads,
                 bool required) {
    cv::setNumThreads(calibration_threads);
    const LinearBboxPlan plan = yolo::detail::calibrate_linear_bbox(source, output, opencv_resize);
    const std::string label = yolo::detail::describe_linear_bbox_plan(plan);
    std::cout << "real " << source.width << "x" << source.height << "->" << output.width << "x" << output.height
              << " calibrated_with_threads=" << calibration_threads << " used_with_threads=" << use_threads;
    if (!plan.exact) {
        check(!required, "the kernel is not in use on this machine: " + label);
        std::cout << " | " << label << "\n";
        return;
    }
    cv::setNumThreads(use_threads);
    std::mt19937 random(static_cast<std::uint32_t>(source.width * 131 + output.width + 7 * calibration_threads + use_threads));
    long compared = 0;
    for (int p = 0; p < 10; ++p) {
        cv::Mat plane = p < 4 ? logits_plane(source, random, p < 2 ? 1.0F : 32.0F) : spread_plane(source, random);
        if (p >= 6) add_specials(plane);
        const cv::Mat full = opencv_resize(plane, output);
        for (const cv::Rect& box : test_boxes(output, random)) {
            const long diff = differing(yolo::detail::resize_linear_roi(plane, 0, 0, plan, box), full(box));
            check(diff == 0, "kernel differs from cv::resize: plane " + std::to_string(p) + " box " +
                                 std::to_string(box.x) + "," + std::to_string(box.y) + "," + std::to_string(box.width) +
                                 "x" + std::to_string(box.height) + " differing=" + std::to_string(diff) + " | " + label);
            compared += static_cast<long>(box.area());
        }
    }
    std::cout << " compared_values=" << compared << " | " << label << "\n";
}

// For the production size OpenCV's own result must not depend on how it is called (1 or 2 threads,
// or nested in a parallel region, as a concurrent call can be in OpenCV 4.5): else "equal to
// cv::resize" would not be one value.
void opencv_path_is_stable(const cv::Size& source, const cv::Size& output) {
    std::mt19937 random(5U);
    for (int p = 0; p < 3; ++p) {
        const cv::Mat plane = logits_plane(source, random, 8.0F);
        cv::setNumThreads(1);
        const cv::Mat one = opencv_resize(plane, output);
        cv::setNumThreads(2);
        const cv::Mat two = opencv_resize(plane, output);
        cv::Mat nested;
        cv::parallel_for_(cv::Range(0, 1), [&](const cv::Range&) { nested = opencv_resize(plane, output); });
        check(differing(one, two) == 0 && differing(one, nested) == 0,
              "cv::resize depends on the thread count or nesting for the production size");
    }
    std::cout << "opencv path stable " << source.width << "x" << source.height << "->" << output.width << "x"
              << output.height << " (1 thread = 2 threads = nested)\n";
}

// The decoder's entry points (process-wide plan): bbox from the whole source, and from a part that
// holds only what the bbox reads (the logits rectangle), which must be refused when it misses a row.
void public_entry_points(const cv::Size& source, const cv::Size& output) {
    const LinearBboxPlan& plan = yolo::detail::linear_bbox_plan(source, output);
    check(plan.exact, "the process-wide plan is not exact: " + yolo::detail::describe_linear_bbox_plan(plan));
    std::mt19937 random(11U);
    for (int p = 0; p < 4; ++p) {
        cv::Mat plane = logits_plane(source, random, 4.0F);
        if (p >= 2) add_specials(plane);
        const cv::Mat full = opencv_resize(plane, output);
        for (const cv::Rect& box : test_boxes(output, random)) {
            check(differing(yolo::detail::resize_linear_bbox(plane, output, box), full(box)) == 0,
                  "resize_linear_bbox differs from cv::resize");
            const cv::Rect reads = yolo::detail::linear_bbox_reads(plan, box);
            const cv::Mat part = plane(reads).clone();
            cv::Mat out;
            check(yolo::detail::resize_linear_bbox_part(part, reads.y, reads.x, source, output, box, out),
                  "kernel refused a part that covers its reads");
            check(differing(out, full(box)) == 0, "part kernel differs from cv::resize");
            if (reads.height > 1) {
                const cv::Mat short_part = plane(cv::Rect(reads.x, reads.y + 1, reads.width, reads.height - 1)).clone();
                check(!yolo::detail::resize_linear_bbox_part(short_part, reads.y + 1, reads.x, source, output, box, out),
                      "kernel accepted a part that misses a row it reads");
            }
        }
    }
    std::cout << "public entry points " << source.width << "x" << source.height << "->" << output.width << "x"
              << output.height << " ok\n";
}

// The service warms the plan at start under its process count (2) and decodes under the capture's
// count (1): after the prewarm, neither count may calibrate again in the decode (found 2026-09-30
// on 1.8: a 210 ms calibration in the first decoded frame), and the thread count is restored.
void prewarm_covers_both_thread_counts() {
    cv::setNumThreads(2);
    yolo::prewarm_active12_mask_kernels(224, 224, 1280, 800);
    check(cv::getNumThreads() == 2, "prewarm did not restore the OpenCV thread count");
    const int after_prewarm = yolo::detail::linear_bbox_calibration_count();
    for (const int threads : {1, 2}) {
        cv::setNumThreads(threads);
        const LinearBboxPlan& plan = yolo::detail::linear_bbox_plan(cv::Size(224, 140), cv::Size(1280, 800));
        check(plan.exact, "prewarmed plan is not exact under " + std::to_string(threads) + " threads");
    }
    check(yolo::detail::linear_bbox_calibration_count() == after_prewarm,
          "a decode after the prewarm calibrated again");
    // The 896 x 608 Active12 input has 152 x 224 prototypes; for the same camera image its
    // prototype crop is again 224 x 140, so its prewarm must find the plan above and not calibrate.
    yolo::prewarm_active12_mask_kernels(224, 152, 1280, 800);
    check(yolo::detail::linear_bbox_calibration_count() == after_prewarm,
          "the 152 x 224 prototypes calibrated a second plan for the same 224x140 -> 1280x800 resize");
    check(cv::getNumThreads() == 2, "second prewarm did not restore the OpenCV thread count");
    std::cout << "prewarm covers opencv_threads 1 and 2 (calibrations so far " << after_prewarm << ")\n";
}

// Equal size and downscales are not what the kernel models: they must go to cv::resize.
void non_upscale_falls_back() {
    std::mt19937 random(3U);
    for (const auto& pair : {std::make_pair(cv::Size(224, 140), cv::Size(224, 140)),
                             std::make_pair(cv::Size(224, 140), cv::Size(112, 70)),
                             std::make_pair(cv::Size(224, 140), cv::Size(100, 60)),
                             std::make_pair(cv::Size(224, 140), cv::Size(1280, 140))}) {
        const LinearBboxPlan& plan = yolo::detail::linear_bbox_plan(pair.first, pair.second);
        check(!plan.exact && plan.reason == "not_an_upscale",
              "not an upscale but planned: " + yolo::detail::describe_linear_bbox_plan(plan));
        cv::Mat plane = spread_plane(pair.first, random);
        add_specials(plane);
        const cv::Rect box(1, 1, pair.second.width - 2, pair.second.height - 2);
        check(differing(yolo::detail::resize_linear_bbox(plane, pair.second, box), opencv_bbox(plane, pair.second, box)) == 0,
              "fallback differs from cv::resize");
    }
}

}  // namespace

int main() {
    try {
        using R = LinearRounding;
        std::vector<SimulatedBuild> builds{
            {"plain/plain", uniform_runs<R::Plain>, uniform_runs<R::Plain>},
            {"plain/fuse_first", uniform_runs<R::Plain>, uniform_runs<R::FuseFirst>},
            {"plain/fuse_second", uniform_runs<R::Plain>, uniform_runs<R::FuseSecond>},
            {"fuse_first/plain", uniform_runs<R::FuseFirst>, uniform_runs<R::Plain>},
            {"fuse_first/fuse_first(1.8)", uniform_runs<R::FuseFirst>, uniform_runs<R::FuseFirst>},
            {"fuse_first/fuse_second", uniform_runs<R::FuseFirst>, uniform_runs<R::FuseSecond>},
            {"fuse_second/plain", uniform_runs<R::FuseSecond>, uniform_runs<R::Plain>},
            {"fuse_second/fuse_first", uniform_runs<R::FuseSecond>, uniform_runs<R::FuseFirst>},
            {"fuse_second/fuse_second", uniform_runs<R::FuseSecond>, uniform_runs<R::FuseSecond>},
            {"mac_layout/fuse_first", mac_horizontal, uniform_runs<R::FuseFirst>},
        };
        simulated_builds(cv::Size(224, 140), cv::Size(1280, 800), builds);
        simulated_builds(cv::Size(160, 100), cv::Size(1280, 800), {builds[4], builds[9]});
        simulated_builds(cv::Size(224, 126), cv::Size(1280, 720), {builds[4], builds[9]});
        one_wrong_column_is_visible();
        unrepresentable_build_falls_back();
        opencv_path_is_stable(cv::Size(224, 140), cv::Size(1280, 800));
        // Across thread counts only where OpenCV's path was shown stable (the production size); other
        // sizes under the count they were calibrated with, as the process-wide cache keys them.
        for (const int calibration_threads : {1, 2}) {
            for (const int use_threads : {1, 2}) {
                real_resize(cv::Size(224, 140), cv::Size(1280, 800), calibration_threads, use_threads, true);
            }
            real_resize(cv::Size(160, 100), cv::Size(1280, 800), calibration_threads, calibration_threads, false);
            real_resize(cv::Size(224, 126), cv::Size(1280, 720), calibration_threads, calibration_threads, false);
        }
        prewarm_covers_both_thread_counts();
        cv::setNumThreads(1);
        public_entry_points(cv::Size(224, 140), cv::Size(1280, 800));
        non_upscale_falls_back();
        std::cout << "test_linear_bbox_resize: PASS checks=" << checks << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_linear_bbox_resize: FAIL " << e.what() << "\n";
        return 1;
    }
}
