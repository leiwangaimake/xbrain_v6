/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_depth_codesort.cpp
 * Brief: Unit tests for the code-sort depth sampling path against the standard-sort reference.
 *
 * Description:
 * What it checks. The code-sort implementation must give bit-identical samples to the std::sort
 * path: exhaustive depth codes and scales, modes, shapes and multisets, masks with clipping and
 * non-contiguous memory, early returns with scratch reuse, the rounding environment and
 * independent per-thread scratch.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "common/logger.hpp"
#include "modules/depth_sampler.hpp"

namespace {

using Sampler = perception::DepthSampler;
using Sort = Sampler::SortImplementation;
using Profile = perception::DepthSampleProfile;
using Sample = perception::DepthSample;

void same_bits(float first, float second) {
    assert(std::memcmp(&first, &second, sizeof(float)) == 0);
}

void same_vector(const std::vector<float>& first, const std::vector<float>& second) {
    assert(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) same_bits(first[i], second[i]);
}

// Do not memcmp a struct containing padding or std::string. Every public
// DepthSample field is checked explicitly, including all floating point bits.
void same_sample(const Sample& first, const Sample& second) {
    same_bits(first.depth_m, second.depth_m);
    for (std::size_t i = 0; i < first.centroid_3d.size(); ++i) {
        same_bits(first.centroid_3d[i], second.centroid_3d[i]);
    }
    assert(first.sample_region_pixel_count == second.sample_region_pixel_count);
    assert(first.valid_pixel_count == second.valid_pixel_count);
    assert(first.selected_pixel_count == second.selected_pixel_count);
    assert(first.required_cluster_pixel_count == second.required_cluster_pixel_count);
    same_bits(first.valid_fraction, second.valid_fraction);
    same_bits(first.selected_fraction, second.selected_fraction);
    same_bits(first.selected_min_m, second.selected_min_m);
    same_bits(first.selected_median_m, second.selected_median_m);
    same_bits(first.selected_max_m, second.selected_max_m);
    same_bits(first.selected_stddev_m, second.selected_stddev_m);
    assert(first.sample_source == second.sample_source);
    assert(first.selection_status == second.selection_status);
    assert(first.selection_fallback == second.selection_fallback);
    assert(first.valid == second.valid);
}

struct Oracle {
    std::vector<float> sorted;
    std::uint64_t roi_pixels = 0;
    std::uint64_t region_pixels = 0;
};

// This oracle deliberately does not call any sampler conversion/reconstruction
// helper. The reference expression is the original float(raw) * float(scale),
// followed by the original finite/range predicate and std::sort.
Oracle reference(const cv::Mat& depth, const yolo::DepthIntrinsics& intrinsics,
                 float scale, const yolo::Detection& detection) {
    Oracle result;
    if (depth.empty() || intrinsics.fx <= 0.0F || intrinsics.fy <= 0.0F || scale <= 0.0F ||
        (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) return result;
    auto roi = detection.bbox & cv::Rect(0, 0, depth.cols, depth.rows);
    if (roi.empty()) return result;
    const bool use_mask = !detection.mask.empty() && detection.mask.size() == detection.bbox.size();
    if (!use_mask) {
        const int margin_x = roi.width / 4;
        const int margin_y = roi.height / 4;
        roi.x += margin_x;
        roi.y += margin_y;
        roi.width -= 2 * margin_x;
        roi.height -= 2 * margin_y;
    }
    result.roi_pixels = static_cast<std::uint64_t>(roi.width) *
                        static_cast<std::uint64_t>(roi.height);
    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            if (use_mask && detection.mask.at<std::uint8_t>(
                    y - detection.bbox.y, x - detection.bbox.x) == 0U) continue;
            ++result.region_pixels;
            const float converted = depth.type() == CV_16UC1
                ? static_cast<float>(depth.at<std::uint16_t>(y, x)) * scale
                : depth.at<float>(y, x) * scale;
            if (std::isfinite(converted) && converted >= 0.3F && converted <= 10.0F) {
                result.sorted.push_back(converted);
            }
        }
    }
    std::sort(result.sorted.begin(), result.sorted.end());
    return result;
}

constexpr std::array<std::uint64_t Profile::*, 10> stages{
    &Profile::setup_ns, &Profile::scan_ns, &Profile::depth_copy_ns,
    &Profile::sort_ns, &Profile::cluster_selection_ns, &Profile::select_pixels_ns,
    &Profile::median_u_ns, &Profile::median_v_ns, &Profile::statistics_ns,
    &Profile::finalize_ns};

Profile poisoned_profile() {
    Profile profile;
    for (const auto stage : stages) profile.*stage = std::numeric_limits<std::uint64_t>::max();
    profile.start_ns = profile.end_ns = profile.total_wall_ns = profile.thread_cpu_ns =
        profile.roi_pixels = profile.region_pixels = profile.valid_pixels =
        profile.selected_pixels = std::numeric_limits<std::uint64_t>::max();
    profile.wall_clock_valid = profile.cpu_clock_valid = true;
    profile.code_sort_used = profile.sort_shadow_checked = true;
    return profile;
}

void check_profile(const Profile& profile, const Sample& sample, const Oracle& oracle) {
    assert(profile.wall_clock_valid);
    assert(profile.start_ns > 0 && profile.end_ns >= profile.start_ns);
    assert(profile.total_wall_ns == profile.end_ns - profile.start_ns);
    std::uint64_t sum = 0;
    for (const auto stage : stages) {
        assert(profile.*stage <= profile.total_wall_ns);
        sum += profile.*stage;
    }
    assert(sum == profile.total_wall_ns);
    assert(profile.roi_pixels == oracle.roi_pixels);
    assert(profile.region_pixels == oracle.region_pixels);
    assert(profile.valid_pixels == oracle.sorted.size());
    assert(profile.roi_pixels >= profile.region_pixels);
    assert(profile.region_pixels >= profile.valid_pixels);
    assert(profile.valid_pixels >= profile.selected_pixels);
    assert(profile.region_pixels == static_cast<std::uint64_t>(sample.sample_region_pixel_count));
    assert(profile.valid_pixels == static_cast<std::uint64_t>(sample.valid_pixel_count));
    assert(profile.selected_pixels == static_cast<std::uint64_t>(sample.selected_pixel_count));
    if (!profile.cpu_clock_valid) assert(profile.thread_cpu_ns == 0);
#if !defined(__linux__)
    assert(!profile.cpu_clock_valid);
#endif
    if (sample.selection_status == "invalid_input" || sample.selection_status == "bbox_outside") {
        assert(profile.setup_ns == profile.total_wall_ns);
        for (std::size_t i = 1; i < stages.size(); ++i) assert(profile.*stages[i] == 0);
    } else if (sample.selection_status == "no_valid_depth") {
        assert(profile.setup_ns + profile.scan_ns == profile.total_wall_ns);
        for (std::size_t i = 2; i < stages.size(); ++i) assert(profile.*stages[i] == 0);
    }
}

yolo::DepthIntrinsics intrinsics_for(const cv::Mat& depth) {
    yolo::DepthIntrinsics intrinsics;
    intrinsics.fx = 107.25F;
    intrinsics.fy = 98.5F;
    intrinsics.cx = 6.25F;
    intrinsics.cy = 5.75F;
    intrinsics.width = depth.cols;
    intrinsics.height = depth.rows;
    return intrinsics;
}

yolo::Detection full_mask(const cv::Mat& depth) {
    yolo::Detection detection;
    detection.bbox = cv::Rect(0, 0, depth.cols, depth.rows);
    if (!depth.empty()) detection.mask = cv::Mat::ones(depth.size(), CV_8UC1);
    return detection;
}

struct Measured {
    Sample sample;
    Profile profile;
    std::vector<float> sorted;
};

Measured measure(const Sampler& sampler, const cv::Mat& depth,
                 const yolo::DepthIntrinsics& intrinsics, float scale,
                 const yolo::Detection& detection, const Oracle& oracle) {
    Measured result;
    result.profile = poisoned_profile();
    result.sorted = {123.0F, -456.0F}; // Must be cleared even by early returns.
    result.sample = sampler.sample(depth, intrinsics, scale, detection, &result.profile, &result.sorted);
    check_profile(result.profile, result.sample, oracle);
    same_vector(result.sorted, oracle.sorted);
    return result;
}

void compare_samplers(const Sampler& old_sampler, const Sampler& code_sampler,
                     const Sampler& shadow_sampler, const cv::Mat& depth,
                     const yolo::DepthIntrinsics& intrinsics, float scale,
                     const yolo::Detection& detection) {
    const int rounding_before = std::fegetround();
    const auto oracle = reference(depth, intrinsics, scale, detection);
    const auto old_before = measure(old_sampler, depth, intrinsics, scale, detection, oracle);
    auto code = measure(code_sampler, depth, intrinsics, scale, detection, oracle);
    const auto old_after = measure(old_sampler, depth, intrinsics, scale, detection, oracle);
    const auto shadow = measure(shadow_sampler, depth, intrinsics, scale, detection, oracle);
    same_sample(old_before.sample, code.sample);
    same_sample(old_before.sample, old_after.sample);
    same_sample(old_before.sample, shadow.sample);
    assert(!old_before.profile.code_sort_used && !old_before.profile.sort_shadow_checked);
    assert(!old_after.profile.code_sort_used && !old_after.profile.sort_shadow_checked);
    assert(code.profile.code_sort_used == (depth.type() == CV_16UC1 && !oracle.sorted.empty()));
    assert(!code.profile.sort_shadow_checked);
    assert(!shadow.profile.code_sort_used && shadow.profile.sort_shadow_checked);

    // Opt-in profile and diagnostic copies cannot change sampling results.
    for (const Sampler* sampler : {&old_sampler, &code_sampler, &shadow_sampler}) {
        std::vector<float> unprofiled_sorted{999.0F};
        const auto unprofiled = sampler->sample(
            depth, intrinsics, scale, detection, nullptr, &unprofiled_sorted);
        same_sample(old_before.sample, unprofiled);
        same_vector(unprofiled_sorted, oracle.sorted);
        const auto no_diagnostics = sampler->sample(depth, intrinsics, scale, detection);
        same_sample(old_before.sample, no_diagnostics);
    }
    // All snapshots own their storage, including across two sampler instances
    // using the same thread-local scratch. Mutating one must not affect others.
    if (!code.sorted.empty()) code.sorted.front() = -123.0F;
    same_vector(old_before.sorted, oracle.sorted);
    same_vector(old_after.sorted, oracle.sorted);
    same_vector(shadow.sorted, oracle.sorted);
    assert(std::fegetround() == rounding_before);
}

void run_case(const cv::Mat& depth, float scale, const yolo::Detection& detection,
             const std::string& mode = "median",
             const yolo::DepthIntrinsics* custom_intrinsics = nullptr) {
    Sampler::Params params;
    params.sample_mode = mode;
    params.sort_impl = Sort::StdSort;
    const Sampler old_sampler(params);
    params.sort_impl = Sort::CodeSort;
    const Sampler code_sampler(params);
    params.sort_impl = Sort::Shadow;
    const Sampler shadow_sampler(params);
    const auto intrinsics = custom_intrinsics ? *custom_intrinsics : intrinsics_for(depth);
    compare_samplers(old_sampler, code_sampler, shadow_sampler, depth, intrinsics, scale, detection);
}

cv::Mat all_codes() {
    cv::Mat depth(256, 256, CV_16UC1);
    for (std::uint32_t code = 0; code <= 65535U; ++code) {
        depth.at<std::uint16_t>(static_cast<int>(code / 256U), static_cast<int>(code % 256U)) =
            static_cast<std::uint16_t>(code);
    }
    return depth;
}

void exhaustive_codes_and_scales() {
    const auto depth = all_codes();
    const auto detection = full_mask(depth);
    const std::array<float, 9> scales{
        0.001F, std::nextafter(0.001F, 0.0F),
        std::nextafter(0.001F, std::numeric_limits<float>::infinity()),
        0.0001F, 0.01F, 0.3F / 65535.0F, 10.0F / 65535.0F,
        std::numeric_limits<float>::denorm_min(), std::numeric_limits<float>::max()};
    for (const float scale : scales) run_case(depth, scale, detection);
    const auto max_code_oracle = reference(depth, intrinsics_for(depth), 0.3F / 65535.0F, detection);
    assert(!max_code_oracle.sorted.empty());
    same_bits(max_code_oracle.sorted.back(), 65535.0F * (0.3F / 65535.0F));
    assert(depth.at<std::uint16_t>(0, 0) == 0U);
    assert(depth.at<std::uint16_t>(255, 255) == 65535U);
}

void modes_shapes_and_multisets() {
    const std::array<std::size_t, 12> sizes{1, 2, 3, 7, 16, 31, 127, 1000, 4096, 16384, 65536, 1000000};
    std::mt19937 random(0xD435U);
    std::uniform_int_distribution<unsigned> codes(0U, 65535U);
    for (const auto size : sizes) {
        cv::Mat depth(1, static_cast<int>(size), CV_16UC1);
        for (std::size_t i = 0; i < size; ++i) {
            depth.at<std::uint16_t>(0, static_cast<int>(i)) =
                static_cast<std::uint16_t>(i % 4U == 0U ? 1500U : codes(random));
        }
        const auto detection = full_mask(depth);
        // Avoid a scale x shape x million-pixel Cartesian product.
        for (const std::string mode : {"median", "percentile", "nearest_cluster"}) {
            run_case(depth, 0.001F, detection, mode);
        }
    }
    auto ordered = all_codes();
    run_case(ordered, 0.0001F, full_mask(ordered), "percentile");
    std::reverse(ordered.ptr<std::uint16_t>(), ordered.ptr<std::uint16_t>() + ordered.total());
    run_case(ordered, 0.0001F, full_mask(ordered), "nearest_cluster");
    for (const auto code : {0U, 299U, 300U, 301U, 9999U, 10000U, 10001U, 65535U}) {
        cv::Mat repeated(7, 9, CV_16UC1, cv::Scalar(code));
        run_case(repeated, 0.001F, full_mask(repeated), "nearest_cluster");
    }
    cv::Mat boundaries(1, 10, CV_32FC1);
    const std::array<float, 10> values{
        0.0F, std::nextafter(0.3F, 0.0F), 0.3F,
        std::nextafter(0.3F, 1.0F), std::nextafter(10.0F, 0.0F), 10.0F,
        std::nextafter(10.0F, 11.0F), -1.0F,
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
    std::copy(values.begin(), values.end(), boundaries.ptr<float>());
    for (const std::string mode : {"median", "percentile", "nearest_cluster"}) {
        run_case(boundaries, 1.0F, full_mask(boundaries), mode);
    }
}

void masks_clipping_and_noncontiguous() {
    cv::Mat storage(29, 37, CV_16UC1);
    for (int y = 0; y < storage.rows; ++y) {
        for (int x = 0; x < storage.cols; ++x) {
            storage.at<std::uint16_t>(y, x) = static_cast<std::uint16_t>((y * 1999 + x * 43) % 12000);
        }
    }
    const cv::Mat depth = storage(cv::Rect(3, 4, 23, 19));
    assert(!depth.isContinuous());
    for (const std::string mode : {"median", "percentile", "nearest_cluster"}) {
        auto detection = full_mask(depth);
        run_case(depth, 0.001F, detection, mode);
        detection.mask.release();
        run_case(depth, 0.001F, detection, mode);
        detection.mask = cv::Mat::ones(2, 2, CV_8UC1);
        run_case(depth, 0.001F, detection, mode);
        detection.bbox = cv::Rect(-5, -3, 31, 25);
        cv::Mat mask_storage(29, 37, CV_8UC1, cv::Scalar(0));
        detection.mask = mask_storage(cv::Rect(2, 2, 31, 25));
        assert(!detection.mask.isContinuous());
        for (int y = 0; y < detection.mask.rows; ++y) {
            for (int x = 0; x < detection.mask.cols; ++x) {
                detection.mask.at<std::uint8_t>(y, x) = (x + 3 * y) % 5 == 0 ? 255U : 0U;
            }
        }
        run_case(depth, 0.001F, detection, mode);
        detection.mask.setTo(0);
        run_case(depth, 0.001F, detection, mode);
        detection.mask.release();
        run_case(depth, 0.001F, detection, mode);
    }
    cv::Mat float_storage;
    storage.convertTo(float_storage, CV_32FC1);
    const cv::Mat float_depth = float_storage(cv::Rect(3, 4, 23, 19));
    assert(!float_depth.isContinuous());
    run_case(float_depth, 0.001F, full_mask(float_depth));
}

void early_returns_and_scratch_reuse() {
    cv::Mat depth(11, 13, CV_16UC1, cv::Scalar(65535));
    const auto detection = full_mask(depth);
    const auto intrinsics = intrinsics_for(depth);
    Sampler::Params params;
    params.sample_mode = "median";
    const Sampler old_sampler(params);
    params.sort_impl = Sort::CodeSort;
    Sampler code_sampler(params);
    params.sort_impl = Sort::Shadow;
    const Sampler shadow_sampler(params);
    const std::array<float, 9> scales{
        0.0001F, 0.0F, -0.001F, std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
        0.3F / 65535.0F, 0.001F, 10.0F / 65535.0F};
    for (int repeat = 0; repeat < 3; ++repeat) {
        for (const float scale : scales) {
            compare_samplers(old_sampler, code_sampler, shadow_sampler, depth, intrinsics, scale, detection);
        }
        run_case(cv::Mat{}, 0.001F, detection);
        run_case(cv::Mat(11, 13, CV_8UC1, cv::Scalar(1)), 0.001F, detection);
        auto outside = detection;
        outside.bbox = cv::Rect(100, 100, 13, 11);
        run_case(depth, 0.001F, outside);
        outside.bbox = cv::Rect(0, 0, 0, 0);
        outside.mask.release();
        run_case(depth, 0.001F, outside);
        for (int invalid_axis = 0; invalid_axis < 2; ++invalid_axis) {
            auto invalid = intrinsics;
            (invalid_axis == 0 ? invalid.fx : invalid.fy) = 0.0F;
            run_case(depth, 0.0001F, detection, "median", &invalid);
        }
        depth.setTo(repeat % 2 == 0 ? 300 : 10000);
        compare_samplers(old_sampler, code_sampler, shadow_sampler, depth, intrinsics, 0.001F, detection);
        // Alternate the implementation on the same object as well as instances.
        auto changed = code_sampler.params();
        changed.sort_impl = Sort::StdSort;
        code_sampler.set_params(changed);
        const auto old_result = code_sampler.sample(depth, intrinsics, 0.001F, detection);
        changed.sort_impl = Sort::CodeSort;
        code_sampler.set_params(changed);
        same_sample(old_result, code_sampler.sample(depth, intrinsics, 0.001F, detection));
        depth.setTo(65535);
    }
}

void rounding_environment() {
    const int original_rounding = std::fegetround();
    assert(original_rounding != -1);
    const auto depth = all_codes();
    for (const int rounding : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
        assert(std::fesetround(rounding) == 0);
        run_case(depth, 0.001F, full_mask(depth));
        assert(std::fegetround() == rounding);
    }
    assert(std::fesetround(original_rounding) == 0);
}

void independent_thread_scratch() {
    Sampler::Params params;
    params.sample_mode = "percentile";
    const Sampler old_sampler(params);
    params.sort_impl = Sort::CodeSort;
    const Sampler code_sampler(params);
    params.sort_impl = Sort::Shadow;
    const Sampler shadow_sampler(params);
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    auto work = [&](int thread_number) {
        cv::Mat depth(64, 67, CV_16UC1);
        const auto detection = full_mask(depth);
        const auto intrinsics = intrinsics_for(depth);
        ready.fetch_add(1, std::memory_order_release);
        while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int iteration = 0; iteration < 12; ++iteration) {
            for (int y = 0; y < depth.rows; ++y) {
                for (int x = 0; x < depth.cols; ++x) {
                    depth.at<std::uint16_t>(y, x) = static_cast<std::uint16_t>(
                        (x * 197 + y * 313 + iteration * 997 + thread_number * 16381) % 65536);
                }
            }
            const float scale = (iteration + thread_number) % 2 == 0 ? 0.001F : 0.0001F;
            compare_samplers(old_sampler, code_sampler, shadow_sampler, depth, intrinsics, scale, detection);
        }
    };
    std::thread first(work, 0);
    std::thread second(work, 1);
    while (ready.load(std::memory_order_acquire) != 2) std::this_thread::yield();
    go.store(true, std::memory_order_release);
    first.join();
    second.join();
}

} // namespace

int main() {
    yolo::Logger::instance().set_level(yolo::LogLevel::ERROR);
    assert(Sampler{}.params().sort_impl == Sort::StdSort);
    std::cout << "codesort: exhaustive codes and scales\n";
    exhaustive_codes_and_scales();
    std::cout << "codesort: modes, multisets through 1000000 pixels, and boundaries\n";
    modes_shapes_and_multisets();
    std::cout << "codesort: masks, clipping, and noncontiguous storage\n";
    masks_clipping_and_noncontiguous();
    std::cout << "codesort: early returns and reused scratch\n";
    early_returns_and_scratch_reuse();
    std::cout << "codesort: preserved rounding environment\n";
    rounding_environment();
    std::cout << "codesort: two independent thread-local scratch instances\n";
    independent_thread_scratch();
    std::cout << "test_depth_codesort: PASS\n";
}
