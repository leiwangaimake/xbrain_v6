/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_depth_sampler.cpp
 * Brief: Unit tests for per-detection depth sampling (nearest dense surface and its confirmation).
 *
 * Description:
 * What it checks. Reused destinations are initialised even on early returns; the original profile
 * scenarios run against a genuinely separate sorting implementation with owning snapshots; the
 * nearest-cluster fallback is forced without changing the algorithm; a 7-pixel near patch below
 * the 8 percent / 8-pixel threshold is not accepted as the nearest dense surface; after the gate a
 * robust near-surface quantile is reported with the median still observable; returning to the
 * accepted band clears a pending outlier and alternating distant candidates do not confirm.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include "modules/depth_sampler.hpp"
#include "common/logger.hpp"

namespace {

void require_same_bits(float first, float second) {
    assert(std::memcmp(&first, &second, sizeof(float)) == 0);
}

void require_same_sample(const perception::DepthSample& first,
                       const perception::DepthSample& second) {
    require_same_bits(first.depth_m, second.depth_m);
    for (std::size_t i = 0; i != first.centroid_3d.size(); ++i) {
        require_same_bits(first.centroid_3d[i], second.centroid_3d[i]);
    }
    assert(first.sample_region_pixel_count == second.sample_region_pixel_count);
    assert(first.valid_pixel_count == second.valid_pixel_count);
    assert(first.selected_pixel_count == second.selected_pixel_count);
    assert(first.required_cluster_pixel_count == second.required_cluster_pixel_count);
    require_same_bits(first.valid_fraction, second.valid_fraction);
    require_same_bits(first.selected_fraction, second.selected_fraction);
    require_same_bits(first.selected_min_m, second.selected_min_m);
    require_same_bits(first.selected_median_m, second.selected_median_m);
    require_same_bits(first.selected_max_m, second.selected_max_m);
    require_same_bits(first.selected_stddev_m, second.selected_stddev_m);
    assert(first.sample_source == second.sample_source);
    assert(first.selection_status == second.selection_status);
    assert(first.selection_fallback == second.selection_fallback);
    assert(first.valid == second.valid);
}

void require_same_sorted(const std::vector<float>& first, const std::vector<float>& second) {
    assert(first.size() == second.size());
    for (std::size_t i = 0; i != first.size(); ++i) require_same_bits(first[i], second[i]);
}

void require_profile(const perception::DepthSampleProfile& profile,
                    const perception::DepthSample& measured, const cv::Mat& depth,
                    const yolo::Detection& detection) {
    assert(profile.wall_clock_valid);
    assert(profile.start_ns > 0);
    assert(profile.end_ns >= profile.start_ns);
    const auto stage_sum = profile.setup_ns + profile.scan_ns + profile.depth_copy_ns +
        profile.sort_ns + profile.cluster_selection_ns + profile.select_pixels_ns +
        profile.median_u_ns + profile.median_v_ns + profile.statistics_ns + profile.finalize_ns;
    assert(profile.total_wall_ns == profile.end_ns - profile.start_ns);
    assert(stage_sum == profile.total_wall_ns);
    assert(profile.roi_pixels >= profile.region_pixels);
    assert(profile.region_pixels >= profile.valid_pixels);
    assert(profile.valid_pixels >= profile.selected_pixels);
    assert(profile.region_pixels == static_cast<std::uint64_t>(measured.sample_region_pixel_count));
    assert(profile.valid_pixels == static_cast<std::uint64_t>(measured.valid_pixel_count));
    assert(profile.selected_pixels == static_cast<std::uint64_t>(measured.selected_pixel_count));
    if (!profile.cpu_clock_valid) assert(profile.thread_cpu_ns == 0);
#if !defined(__linux__)
    assert(!profile.cpu_clock_valid);
#endif
    if (measured.selection_status == "invalid_input" || measured.selection_status == "bbox_outside") {
        assert(profile.roi_pixels == 0);
        assert(profile.setup_ns == profile.total_wall_ns);
    } else {
        cv::Rect roi = detection.bbox & cv::Rect(0, 0, depth.cols, depth.rows);
        const bool mask = !detection.mask.empty() && detection.mask.size() == detection.bbox.size();
        if (!mask) {
            roi.width -= 2 * (roi.width / 4);
            roi.height -= 2 * (roi.height / 4);
        }
        assert(profile.roi_pixels == static_cast<std::uint64_t>(roi.width) *
                                     static_cast<std::uint64_t>(roi.height));
    }
    if (measured.selection_status == "no_valid_depth") {
        assert(profile.setup_ns + profile.scan_ns == profile.total_wall_ns);
        assert(profile.depth_copy_ns == 0 && profile.finalize_ns == 0);
    }
}

void profile_case(const perception::DepthSampler& sampler, const cv::Mat& depth,
                 const yolo::DepthIntrinsics& intrinsics, float scale,
                 const yolo::Detection& detection) {
    std::vector<float> before_sorted{123.0F};
    std::vector<float> measured_sorted{456.0F};
    std::vector<float> after_sorted{789.0F};
    const auto before = sampler.sample(depth, intrinsics, scale, detection, nullptr, &before_sorted);
    perception::DepthSampleProfile profile;
    // Reused destinations must be initialized even when sampling returns early.
    profile.sort_ns = profile.finalize_ns = profile.thread_cpu_ns =
        profile.roi_pixels = profile.region_pixels = profile.valid_pixels =
        profile.selected_pixels = std::numeric_limits<std::uint64_t>::max();
    const auto measured = sampler.sample(depth, intrinsics, scale, detection, &profile, &measured_sorted);
    const auto after = sampler.sample(depth, intrinsics, scale, detection, nullptr, &after_sorted);
    require_same_sample(before, measured);
    require_same_sample(measured, after);
    require_same_sorted(before_sorted, measured_sorted);
    require_same_sorted(measured_sorted, after_sorted);
    require_profile(profile, measured, depth, detection);
    assert(!profile.code_sort_used && !profile.sort_shadow_checked);

    // Keep every original profile scenario, and exercise a genuinely separate
    // sorting implementation with an owning snapshot across old/new/old calls.
    using Sort = perception::DepthSampler::SortImplementation;
    for (const auto implementation : {Sort::CodeSort, Sort::Shadow}) {
        auto params = sampler.params();
        params.sort_impl = implementation;
        perception::DepthSampler candidate(params);
        perception::DepthSampleProfile candidate_profile;
        candidate_profile.code_sort_used = candidate_profile.sort_shadow_checked = true;
        std::vector<float> candidate_sorted{999.0F};
        const auto candidate_sample = candidate.sample(
            depth, intrinsics, scale, detection, &candidate_profile, &candidate_sorted);
        require_same_sample(measured, candidate_sample);
        require_same_sorted(measured_sorted, candidate_sorted);
        require_profile(candidate_profile, candidate_sample, depth, detection);
        assert(candidate_profile.sort_shadow_checked == (implementation == Sort::Shadow));
        assert(candidate_profile.code_sort_used ==
            (implementation == Sort::CodeSort && depth.type() == CV_16UC1 &&
             !measured_sorted.empty()));
        std::vector<float> unprofiled_sorted{888.0F};
        const auto unprofiled = candidate.sample(
            depth, intrinsics, scale, detection, nullptr, &unprofiled_sorted);
        require_same_sample(measured, unprofiled);
        require_same_sorted(measured_sorted, unprofiled_sorted);
        const auto old_again = sampler.sample(depth, intrinsics, scale, detection);
        require_same_sample(measured, old_again);
        // Subsequent calls reuse scratch but cannot modify earlier snapshots.
        require_same_sorted(before_sorted, candidate_sorted);
        require_same_sorted(before_sorted, after_sorted);
    }
}

// In every mode, for 16-bit and float depth, every mask shape, out-of-image boxes, invalid inputs
// and a forced fallback, profiling changes nothing, and CodeSort and Shadow give the same sample
// and sorted values as StdSort.
void test_optional_profile_parity() {
    yolo::DepthIntrinsics intrinsics;
    intrinsics.fx = 107.25F;
    intrinsics.fy = 98.5F;
    intrinsics.cx = 6.25F;
    intrinsics.cy = 5.75F;
    intrinsics.width = 13;
    intrinsics.height = 11;
    cv::Mat depth16(11, 13, CV_16UC1);
    for (int y = 0; y != depth16.rows; ++y) {
        for (int x = 0; x != depth16.cols; ++x) {
            depth16.at<std::uint16_t>(y, x) =
                static_cast<std::uint16_t>(1000 + ((y * 13 + x) * 37) % 1300);
        }
    }
    depth16.at<std::uint16_t>(0, 0) = 0;
    depth16.at<std::uint16_t>(1, 1) = 299;
    depth16.at<std::uint16_t>(2, 2) = 300;
    depth16.at<std::uint16_t>(3, 3) = 10000;
    depth16.at<std::uint16_t>(4, 4) = 10001;
    cv::Mat depth32;
    depth16.convertTo(depth32, CV_32FC1);
    depth32.at<float>(2, 3) = std::numeric_limits<float>::quiet_NaN();
    depth32.at<float>(3, 2) = std::numeric_limits<float>::infinity();
    depth32.at<float>(4, 2) = -1.0F;

    for (const std::string mode : {"nearest_cluster", "percentile", "median"}) {
        perception::DepthSampler sampler({mode, 0.2F, 0.12F, 0.08F, 8, 0.1F});
        for (const cv::Mat& depth : {depth16, depth32}) {
            yolo::Detection detection;
            detection.bbox = cv::Rect(0, 0, depth.cols, depth.rows);
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
            detection.mask = cv::Mat::ones(detection.bbox.size(), CV_8UC1);
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
            for (int y = 0; y != detection.mask.rows; ++y) {
                for (int x = 0; x != detection.mask.cols; ++x) {
                    detection.mask.at<std::uint8_t>(y, x) = (x + y) % 3 == 0 ? 255U : 0U;
                }
            }
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
            detection.mask.setTo(0);
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
            detection.mask = cv::Mat::ones(2, 2, CV_8UC1); // Mismatch uses bbox-center.
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
            detection.bbox = cv::Rect(-2, -1, 10, 9);
            detection.mask = cv::Mat::ones(detection.bbox.size(), CV_8UC1);
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
            detection.bbox = cv::Rect(90, 90, 3, 3);
            detection.mask.release();
            profile_case(sampler, depth, intrinsics, 0.001F, detection);
        }
        yolo::Detection detection;
        detection.bbox = cv::Rect(0, 0, 13, 11);
        auto invalid_intrinsics = intrinsics;
        invalid_intrinsics.fx = 0.0F;
        profile_case(sampler, depth16, invalid_intrinsics, 0.001F, detection);
        invalid_intrinsics = intrinsics;
        invalid_intrinsics.fy = 0.0F;
        profile_case(sampler, depth16, invalid_intrinsics, 0.001F, detection);
        profile_case(sampler, depth16, intrinsics, 0.0F, detection);
        profile_case(sampler, depth16, intrinsics, -0.001F, detection);
        profile_case(sampler, cv::Mat{}, intrinsics, 0.001F, detection);
        profile_case(sampler, cv::Mat(11, 13, CV_8UC1, cv::Scalar(1)), intrinsics, 0.001F, detection);
        profile_case(sampler, cv::Mat(11, 13, CV_16UC1, cv::Scalar(0)), intrinsics, 0.001F, detection);
        profile_case(sampler, cv::Mat(11, 13, CV_32FC1,
            cv::Scalar(std::numeric_limits<float>::quiet_NaN())), intrinsics, 0.001F, detection);
        // Force the nearest-cluster fallback without altering the algorithm.
        perception::DepthSampler fallback({mode, 0.2F, 0.02F, 1.0F, 1000, 0.1F});
        profile_case(fallback, depth32, intrinsics, 0.001F, detection);
    }
}

} // namespace

int main() {
    yolo::Logger::instance().set_level(yolo::LogLevel::ERROR);
    test_optional_profile_parity();
    perception::DepthSampler sampler;
    assert(std::fabs(sampler.params().cluster_min_fraction - 0.08F) < 1e-6F);
    assert(
        std::fabs(sampler.params().nearest_cluster_output_percentile - 0.10F) <
        1e-6F);
    yolo::DepthIntrinsics intrinsics;
    intrinsics.fx = 100.0F;
    intrinsics.fy = 100.0F;
    intrinsics.cx = 2.0F;
    intrinsics.cy = 2.0F;
    intrinsics.width = 5;
    intrinsics.height = 5;

    cv::Mat depth(5, 5, CV_16UC1, cv::Scalar(2000));
    depth.at<std::uint16_t>(2, 2) = 0;
    depth.at<std::uint16_t>(1, 1) = 12000;
    yolo::Detection detection;
    detection.bbox = cv::Rect(0, 0, 5, 5);
    const perception::DepthSample sample = sampler.sample(depth, intrinsics, 0.001F, detection);
    assert(sample.valid);
    assert(sample.sample_region_pixel_count == 9);
    assert(sample.valid_pixel_count == 7);
    assert(sample.selected_pixel_count == 7);
    assert(std::fabs(sample.valid_fraction - 7.0F / 9.0F) < 1e-5F);
    assert(sample.sample_source == "bbox_center");
    assert(sample.selection_status == "percentile_fallback");
    assert(sample.selection_fallback);
    assert(std::fabs(sample.depth_m - 2.0F) < 1e-5F);
    assert(std::fabs(sample.centroid_3d[0]) < 1e-5F);
    assert(std::fabs(sample.centroid_3d[1]) < 1e-5F);
    assert(std::fabs(sample.centroid_3d[2] - 2.0F) < 1e-5F);

    cv::Mat masked_depth(5, 5, CV_16UC1, cv::Scalar(4000));
    detection.bbox = cv::Rect(1, 1, 3, 3);
    detection.mask = cv::Mat::zeros(detection.bbox.size(), CV_8UC1);
    for (int local_y = 0; local_y < detection.mask.rows; ++local_y) {
        detection.mask.at<std::uint8_t>(local_y, 0) = 255U;
        masked_depth.at<std::uint16_t>(detection.bbox.y + local_y, detection.bbox.x) = 1000U;
    }
    const perception::DepthSample masked = sampler.sample(masked_depth, intrinsics, 0.001F, detection);
    assert(masked.valid);
    assert(masked.sample_region_pixel_count == 3);
    assert(masked.valid_pixel_count == 3);
    assert(masked.selected_pixel_count == 3);
    assert(std::fabs(masked.valid_fraction - 1.0F) < 1e-5F);
    assert(masked.sample_source == "mask");
    assert(masked.selection_status == "percentile_fallback");
    assert(masked.selection_fallback);
    assert(std::fabs(masked.depth_m - 1.0F) < 1e-5F);

    cv::Mat hollow_depth(10, 10, CV_16UC1, cv::Scalar(2500));
    detection.bbox = cv::Rect(0, 0, 10, 10);
    detection.mask = cv::Mat::ones(detection.bbox.size(), CV_8UC1);
    hollow_depth.at<std::uint16_t>(0, 0) = 500U;  // Isolated invalid-near speckle.
    for (int x = 1; x <= 15; ++x) {
        hollow_depth.at<std::uint16_t>(x / 10, x % 10) = 1900U;
    }
    const perception::DepthSample hollow = sampler.sample(hollow_depth, intrinsics, 0.001F, detection);
    assert(hollow.valid);
    assert(hollow.sample_region_pixel_count == 100);
    assert(hollow.valid_pixel_count == 100);
    assert(hollow.selected_pixel_count == 15);
    assert(hollow.required_cluster_pixel_count == 8);
    assert(std::fabs(hollow.selected_fraction - 0.15F) < 1e-5F);
    assert(std::fabs(hollow.valid_fraction - 1.0F) < 1e-5F);
    assert(hollow.selection_status == "nearest_cluster");
    assert(!hollow.selection_fallback);
    assert(std::fabs(hollow.depth_m - 1.9F) < 1e-5F);
    assert(std::fabs(hollow.centroid_3d[2] - hollow.depth_m) < 1e-5F);
    assert(std::fabs(hollow.selected_min_m - 1.9F) < 1e-5F);
    assert(std::fabs(hollow.selected_median_m - 1.9F) < 1e-5F);
    assert(std::fabs(hollow.selected_max_m - 1.9F) < 1e-5F);
    assert(std::fabs(hollow.selected_stddev_m) < 1e-5F);

    // A seven-pixel near patch is below the deployed 8% / 8-pixel threshold.
    // It must not be accepted as the nearest dense surface.
    cv::Mat weak_near_depth(10, 10, CV_16UC1, cv::Scalar(2500));
    for (int index = 0; index < 7; ++index) {
        weak_near_depth.at<std::uint16_t>(index / 10, index % 10) = 1390U;
    }
    const perception::DepthSample weak_near =
        sampler.sample(weak_near_depth, intrinsics, 0.001F, detection);
    assert(weak_near.valid);
    assert(weak_near.required_cluster_pixel_count == 8);
    assert(weak_near.selected_pixel_count == 93);
    assert(!weak_near.selection_fallback);
    assert(std::fabs(weak_near.depth_m - 2.5F) < 1e-5F);

    perception::DepthSampler percentile_sampler(
        perception::DepthSampler::Params{"percentile", 0.20F, 0.12F, 0.03F, 8});
    const perception::DepthSample percentile =
        percentile_sampler.sample(hollow_depth, intrinsics, 0.001F, detection);
    assert(percentile.valid);
    assert(percentile.selection_status == "percentile");
    assert(!percentile.selection_fallback);
    assert(std::fabs(percentile.depth_m - 2.5F) < 1e-5F);

    perception::DepthSampler median_sampler(
        perception::DepthSampler::Params{"median", 0.20F, 0.12F, 0.03F, 8});
    const perception::DepthSample median_sample =
        median_sampler.sample(hollow_depth, intrinsics, 0.001F, detection);
    assert(median_sample.valid);
    assert(median_sample.selection_status == "median");
    assert(!median_sample.selection_fallback);
    assert(std::fabs(median_sample.depth_m - 2.5F) < 1e-5F);
    assert(median_sample.selected_pixel_count == 100);

    // Once a dense band has passed the 8% gate, report a robust near-surface
    // quantile instead of the band median. Keep the actual median observable.
    cv::Mat sloped_band(10, 10, CV_16UC1);
    detection.bbox = cv::Rect(0, 0, 10, 10);
    detection.mask = cv::Mat::ones(detection.bbox.size(), CV_8UC1);
    for (int index = 0; index < 100; ++index) {
        sloped_band.at<std::uint16_t>(index / 10, index % 10) =
            static_cast<std::uint16_t>(1000 + index);
    }
    const perception::DepthSample near_surface =
        sampler.sample(sloped_band, intrinsics, 0.001F, detection);
    assert(near_surface.valid);
    assert(near_surface.required_cluster_pixel_count == 8);
    assert(near_surface.selected_pixel_count == 100);
    assert(std::fabs(near_surface.depth_m - 1.009F) < 1e-5F);
    assert(std::fabs(near_surface.selected_min_m - 1.0F) < 1e-5F);
    assert(std::fabs(near_surface.selected_median_m - 1.05F) < 1e-5F);
    assert(std::fabs(near_surface.selected_max_m - 1.099F) < 1e-5F);

    auto make_sample = [](float depth_m) {
        perception::DepthSample value;
        value.depth_m = depth_m;
        value.centroid_3d[2] = depth_m;
        value.selected_min_m = depth_m - 0.01F;
        value.selected_median_m = depth_m;
        value.selected_max_m = depth_m + 0.01F;
        value.valid = true;
        return value;
    };
    perception::TrackDepthHysteresis hysteresis(
        perception::TrackDepthHysteresis::Params{0.20F, 0.12F, 2});
    assert(std::fabs(hysteresis.update(7, make_sample(1.0F)).depth_m - 1.0F) < 1e-5F);
    const perception::DepthSample held = hysteresis.update(7, make_sample(2.0F));
    assert(std::fabs(held.depth_m - 1.0F) < 1e-5F);
    assert(std::fabs(held.selected_median_m - 1.0F) < 1e-5F);
    // Returning to the accepted band clears the pending outlier immediately.
    assert(std::fabs(hysteresis.update(7, make_sample(1.02F)).depth_m - 1.02F) < 1e-5F);
    assert(std::fabs(hysteresis.update(7, make_sample(2.0F)).depth_m - 1.02F) < 1e-5F);
    assert(std::fabs(hysteresis.update(7, make_sample(2.04F)).depth_m - 2.04F) < 1e-5F);
    // Alternating distant candidates do not satisfy consecutive-band confirmation.
    assert(std::fabs(hysteresis.update(7, make_sample(1.0F)).depth_m - 2.04F) < 1e-5F);
    assert(std::fabs(hysteresis.update(7, make_sample(1.4F)).depth_m - 2.04F) < 1e-5F);
    hysteresis.retain({});
    // Removing a missing track also removes its stale accepted depth.
    assert(std::fabs(hysteresis.update(7, make_sample(3.0F)).depth_m - 3.0F) < 1e-5F);
    assert(std::fabs(hysteresis.update(7, make_sample(4.0F)).depth_m - 3.0F) < 1e-5F);
    perception::DepthSample invalid_gap;
    assert(!hysteresis.update(7, invalid_gap).valid);
    assert(std::fabs(hysteresis.update(7, make_sample(4.0F)).depth_m - 3.0F) < 1e-5F);
    assert(std::fabs(hysteresis.update(7, make_sample(4.01F)).depth_m - 4.01F) < 1e-5F);

    cv::Mat invalid(2, 2, CV_16UC1, cv::Scalar(0));
    detection.bbox = cv::Rect(0, 0, 2, 2);
    detection.mask.release();
    const perception::DepthSample empty = sampler.sample(invalid, intrinsics, 0.001F, detection);
    assert(!empty.valid);
    assert(empty.sample_region_pixel_count == 4);
    assert(empty.valid_pixel_count == 0);
    assert(std::fabs(empty.valid_fraction) < 1e-5F);
    assert(empty.sample_source == "bbox_center");
    assert(empty.selection_status == "no_valid_depth");
    return 0;
}
