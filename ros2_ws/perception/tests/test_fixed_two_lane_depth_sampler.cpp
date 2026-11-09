/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_fixed_two_lane_depth_sampler.cpp
 * Brief: Unit tests for the fixed two-lane parallel depth sampler used by the Supervisor.
 *
 * Description:
 * What it checks. Depth sampling for the detections of one frame is split over two fixed lanes.
 * The tests require bit-identical samples to the serial sampler across repeated construction,
 * batch shapes and profiles, reject invalid sizes, and check that an exception in one lane is
 * isolated and the sampler recovers for the next batch. Selected-pixel lists (2026-10-06): each
 * index's list equals the serial sampler's list for the same detection, on both lanes, and a
 * list vector of the wrong size is refused. Every test runs for both splits (fixed even/odd and,
 * since 2026-10-08, the shared largest-first split); a stress test draws 3000 random batches and
 * checks the outputs, the per-lane item counts and the withdrawal of an unclaimed batch; switching
 * the split between batches (diagnostic "alt") changes only the split, never an output.
 */

#include "supervisor/fixed_two_lane_depth_sampler.hpp"

#include <array>
#include <random>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using perception::DepthSample;
using perception::DepthSampleProfile;
using perception::internal::FixedTwoLaneDepthSampler;
using perception::internal::RawDepthSampleTask;
using perception::internal::TwoLaneSplit;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

auto sample_fields(const DepthSample& value) {
    return std::tie(value.depth_m, value.centroid_3d,
                    value.sample_region_pixel_count, value.valid_pixel_count,
                    value.selected_pixel_count, value.required_cluster_pixel_count,
                    value.valid_fraction, value.selected_fraction,
                    value.selected_min_m, value.selected_median_m,
                    value.selected_max_m, value.selected_stddev_m,
                    value.sample_source, value.selection_status,
                    value.selection_fallback, value.valid);
}

void require_same_sample(const DepthSample& actual, const DepthSample& expected) {
    require(sample_fields(actual) == sample_fields(expected),
            "two-lane sample differs from serial reference");
}

struct Fixture {
    static constexpr float scale = 0.001F;
    perception::DepthSampler sampler;
    cv::Mat depth{64, 128, CV_16UC1, cv::Scalar(0)};
    yolo::DepthIntrinsics intrinsics{100.0F, 100.0F, 64.0F, 32.0F, 128, 64};
    std::vector<RawDepthSampleTask> tasks;
    std::vector<DepthSample> expected;

    Fixture() {
        int x = 0;
        for (int i = 0; i != 6; ++i) {
            yolo::Detection detection;
            detection.bbox = cv::Rect(x, i, 9 + 3 * i, 7 + i);
            detection.mask = cv::Mat::ones(detection.bbox.size(), CV_8UC1);
            depth(detection.bbox).setTo(cv::Scalar(800 + 500 * i));
            expected.push_back(sampler.sample(depth, intrinsics, scale, detection));
            require(expected.back().valid, "invalid serial fixture");
            tasks.push_back({&sampler, std::move(detection)});
            x += 9 + 3 * i;
        }
    }

    std::vector<RawDepthSampleTask> prefix(std::size_t count) const {
        return {tasks.begin(), tasks.begin() + static_cast<std::ptrdiff_t>(count)};
    }
};

void require_profile(const DepthSampleProfile& profile, const DepthSample& sample,
                    const RawDepthSampleTask& task) {
    require(profile.wall_clock_valid, "profile wall clock was not recorded");
    require(profile.start_ns > 0 && profile.end_ns >= profile.start_ns,
            "profile endpoints are not ordered");
    require(profile.total_wall_ns == profile.end_ns - profile.start_ns,
            "profile total does not match its endpoints");
    require(profile.roi_pixels == static_cast<std::uint64_t>(task.detection.bbox.area()),
            "profile ROI count lost its task index");
    require(profile.region_pixels == static_cast<std::uint64_t>(sample.sample_region_pixel_count),
            "profile region count lost its task index");
    require(profile.valid_pixels == static_cast<std::uint64_t>(sample.valid_pixel_count),
            "profile valid count lost its task index");
    require(profile.selected_pixels == static_cast<std::uint64_t>(sample.selected_pixel_count),
            "profile selected count lost its task index");
    if (!profile.cpu_clock_valid) {
        require(profile.thread_cpu_ns == 0, "unavailable CPU clock was reported as measured");
    }
}

// The worker can be created and destroyed 128 times, with or without running a batch.
void test_repeated_construction(TwoLaneSplit split) {
    Fixture fixture;
    for (int iteration = 0; iteration != 128; ++iteration) {
        FixedTwoLaneDepthSampler worker(split);
        if (iteration % 2 == 0) continue; // Immediate destruction before any batch.
        const auto tasks = fixture.prefix(1);
        std::vector<DepthSample> outputs(1);
        std::vector<std::exception_ptr> exceptions(1);
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                      outputs, exceptions);
        require(!exceptions[0], "single-task batch failed");
        require_same_sample(outputs[0], fixture.expected[0]);
    }
}

// Batches of 0 to 6 tasks give the single-thread results, with or without profiles; a later
// unprofiled batch never touches the freed profile vector.
void test_batch_shapes_and_profiles(TwoLaneSplit split) {
    Fixture fixture;
    FixedTwoLaneDepthSampler worker(split);
    constexpr std::array<std::size_t, 10> counts{0, 1, 2, 3, 4, 5, 6, 1, 0, 6};
    for (int repeat = 0; repeat != 8; ++repeat) {
        for (const auto count : counts) {
            const auto tasks = fixture.prefix(count);
            std::vector<DepthSample> outputs(count);
            std::vector<std::exception_ptr> exceptions(count);
            {
                std::vector<DepthSampleProfile> profiles(count);
                for (auto& profile : profiles) {
                    profile.region_pixels = std::numeric_limits<std::uint64_t>::max();
                }
                worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                              outputs, exceptions, &profiles);
                // Reads deliberately happen only after the batch join returns.
                for (std::size_t i = 0; i != count; ++i) {
                    require(!exceptions[i], "profiled batch failed");
                    require_same_sample(outputs[i], fixture.expected[i]);
                    require_profile(profiles[i], outputs[i], tasks[i]);
                }
            }
            // The prior profile vector is gone. The default-null path must not
            // retain or access its storage on either lane in the next batch.
            worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                          outputs, exceptions);
            for (std::size_t i = 0; i != count; ++i) {
                require(!exceptions[i], "unprofiled batch failed after profiled batch");
                require_same_sample(outputs[i], fixture.expected[i]);
            }
        }
    }
}

template <typename Function>
void require_invalid_argument(Function&& function, const char* expected) {
    bool caught = false;
    try { function(); }
    catch (const std::invalid_argument& error) {
        caught = std::string(error.what()) == expected;
    }
    require(caught, "incorrect size validation or exception");
}

// Mismatched task, output, exception or profile sizes throw before any work, and the worker stays
// usable.
void test_size_validation(TwoLaneSplit split) {
    Fixture fixture;
    FixedTwoLaneDepthSampler worker(split);
    const auto tasks = fixture.prefix(2);
    std::vector<DepthSample> outputs(2);
    std::vector<std::exception_ptr> exceptions(2);
    std::vector<DepthSampleProfile> profiles(1);
    require_invalid_argument([&]() {
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                      outputs, exceptions, &profiles);
    }, "depth sample task/profile size mismatch");
    outputs.resize(1);
    require_invalid_argument([&]() {
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                      outputs, exceptions);
    }, "depth sample task/output/exception size mismatch");
    outputs.resize(2);
    exceptions.resize(1);
    require_invalid_argument([&]() {
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                      outputs, exceptions);
    }, "depth sample task/output/exception size mismatch");

    const auto empty = fixture.prefix(0);
    outputs.clear();
    exceptions.clear();
    require_invalid_argument([&]() {
        worker.sample(empty, fixture.depth, fixture.intrinsics, Fixture::scale,
                      outputs, exceptions, &profiles);
    }, "depth sample task/profile size mismatch");
    // All rejected calls must leave the worker idle and usable.
    outputs.resize(2);
    exceptions.resize(2);
    profiles.resize(2);
    worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                  outputs, exceptions, &profiles);
    for (std::size_t i = 0; i != tasks.size(); ++i) {
        require(!exceptions[i], "worker failed after rejected size");
        require_same_sample(outputs[i], fixture.expected[i]);
        require_profile(profiles[i], outputs[i], tasks[i]);
    }
}

// A failing task records its own exception and leaves its output and profile untouched without
// affecting the others; the next batch starts clean.
void test_exception_isolation_and_recovery(TwoLaneSplit split) {
    Fixture fixture;
    FixedTwoLaneDepthSampler worker(split);
    auto tasks = fixture.tasks;
    tasks[0].sampler = nullptr;
    tasks[3].sampler = nullptr;
    std::vector<DepthSample> outputs(tasks.size());
    std::vector<std::exception_ptr> exceptions(tasks.size());
    std::vector<DepthSampleProfile> profiles(tasks.size());
    for (auto& output : outputs) output.depth_m = -1.0F;
    worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                  outputs, exceptions, &profiles);
    for (std::size_t i = 0; i != tasks.size(); ++i) {
        if (i == 0 || i == 3) {
            require(static_cast<bool>(exceptions[i]), "missing per-index exception");
            bool correct_error = false;
            try { std::rethrow_exception(exceptions[i]); }
            catch (const std::logic_error& error) {
                correct_error = std::string(error.what()) == "null depth sampler task";
            }
            require(correct_error, "per-index exception type changed");
            require(outputs[i].depth_m == -1.0F, "failed task overwrote its output");
            // No DepthSampler call occurred, so this profile is not a sample.
            require(profiles[i].start_ns == 0, "failed task produced a false profile");
        } else {
            require(!exceptions[i], "exception leaked to a different task");
            require_same_sample(outputs[i], fixture.expected[i]);
            require_profile(profiles[i], outputs[i], tasks[i]);
        }
    }

    tasks[0].sampler = &fixture.sampler;
    tasks[3].sampler = &fixture.sampler;
    worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale,
                  outputs, exceptions, &profiles);
    for (std::size_t i = 0; i != tasks.size(); ++i) {
        require(!exceptions[i], "next batch retained a prior exception");
        require_same_sample(outputs[i], fixture.expected[i]);
        require_profile(profiles[i], outputs[i], tasks[i]);
    }
}

} // namespace

// Per-index selected-pixel lists from both lanes equal the serial sampler's lists; stale
// contents are replaced; a list vector of the wrong size throws before any work.
void test_selected_pixel_lists(TwoLaneSplit split) {
    Fixture fixture;
    FixedTwoLaneDepthSampler worker(split);
    for (std::size_t count = 2; count <= fixture.tasks.size(); ++count) {
        const auto tasks = fixture.prefix(count);
        std::vector<DepthSample> outputs(count);
        std::vector<std::exception_ptr> exceptions(count);
        std::vector<std::vector<std::uint32_t>> lists(count, std::vector<std::uint32_t>{1U, 2U, 3U});
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale, outputs, exceptions, nullptr,
                      &lists);
        for (std::size_t i = 0; i < count; ++i) {
            require(!exceptions[i], "unexpected lane exception");
            require_same_sample(outputs[i], fixture.expected[i]);
            std::vector<std::uint32_t> serial;
            fixture.sampler.sample(fixture.depth, fixture.intrinsics, Fixture::scale, tasks[i].detection, nullptr,
                                   nullptr, &serial);
            require(lists[i] == serial, "lane selected-pixel list differs from the serial sampler");
            require(lists[i].size() == static_cast<std::size_t>(outputs[i].selected_pixel_count),
                    "selected-pixel list size differs from selected_pixel_count");
        }
    }
    const auto tasks = fixture.prefix(2);
    std::vector<DepthSample> outputs(2);
    std::vector<std::exception_ptr> exceptions(2);
    std::vector<std::vector<std::uint32_t>> wrong(1);
    require_invalid_argument([&]() {
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale, outputs, exceptions, nullptr, &wrong);
    }, "depth sample task/selected-index size mismatch");
}

// 3000 random batches (0 to 12 tasks drawn from the fixture, repeats allowed, any order): every
// output equals the serial sample of its own task; the lanes together take each index once; a
// fixed split gives the caller the even indices; a shared batch the helper never claimed was
// done entirely by the caller.
void test_random_batches(TwoLaneSplit split) {
    Fixture fixture;
    FixedTwoLaneDepthSampler worker(split);
    std::mt19937 rng(split == TwoLaneSplit::Shared ? 2026U : 1008U);
    std::size_t claimed = 0, withdrawn = 0;
    for (int iteration = 0; iteration != 3000; ++iteration) {
        const std::size_t count = rng() % 13U;
        std::vector<RawDepthSampleTask> tasks;
        std::vector<std::size_t> source;
        for (std::size_t i = 0; i != count; ++i) {
            source.push_back(rng() % fixture.tasks.size());
            tasks.push_back(fixture.tasks[source.back()]);
        }
        std::vector<DepthSample> outputs(count);
        std::vector<std::exception_ptr> exceptions(count);
        std::vector<std::vector<std::uint32_t>> lists(count);
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale, outputs, exceptions, nullptr, &lists);
        for (std::size_t i = 0; i != count; ++i) {
            require(!exceptions[i], "random batch failed");
            require_same_sample(outputs[i], fixture.expected[source[i]]);
            require(lists[i].size() == static_cast<std::size_t>(outputs[i].selected_pixel_count),
                    "random batch list size differs");
        }
        if (count == 0) continue;
        const auto& stats = worker.last_stats();
        require(stats.caller_items + stats.helper_items == count, "lanes did not take every index once");
        require(stats.join_ns >= stats.caller_done_ns && stats.caller_done_ns >= 0, "batch timings out of order");
        if (split == TwoLaneSplit::Fixed) {
            require(stats.helper_claimed && stats.caller_items == (count + 1) / 2, "fixed split changed");
        } else if (stats.helper_claimed) {
            ++claimed;
            require(stats.helper_start_ns >= 0, "claimed batch without a helper start");
        } else {
            ++withdrawn;
            require(stats.caller_items == count && stats.helper_items == 0, "withdrawn batch not done by the caller");
        }
    }
    if (split == TwoLaneSplit::Shared)
        std::cout << "shared split: helper claimed " << claimed << " batches, " << withdrawn << " withdrawn\n";
}

// The diagnostic alternation: set_split between batches switches the split of the next batch only
// (the helper takes it with the batch); outputs never change, and the stats name the split used.
void test_split_switching() {
    Fixture fixture;
    FixedTwoLaneDepthSampler worker;
    std::mt19937 rng(338U);
    for (int iteration = 0; iteration != 2000; ++iteration) {
        const auto split = iteration % 2 ? TwoLaneSplit::Shared : TwoLaneSplit::Fixed;
        worker.set_split(split);
        const std::size_t count = 2U + rng() % 9U;
        std::vector<RawDepthSampleTask> tasks;
        std::vector<std::size_t> source;
        for (std::size_t i = 0; i != count; ++i) {
            source.push_back(rng() % fixture.tasks.size());
            tasks.push_back(fixture.tasks[source.back()]);
        }
        std::vector<DepthSample> outputs(count);
        std::vector<std::exception_ptr> exceptions(count);
        worker.sample(tasks, fixture.depth, fixture.intrinsics, Fixture::scale, outputs, exceptions);
        for (std::size_t i = 0; i != count; ++i) {
            require(!exceptions[i], "alternating batch failed");
            require_same_sample(outputs[i], fixture.expected[source[i]]);
        }
        const auto& stats = worker.last_stats();
        require(stats.split == split, "stats name the wrong split");
        require(stats.caller_items + stats.helper_items == count, "alternating lanes did not take every index once");
        if (split == TwoLaneSplit::Fixed) require(stats.caller_items == (count + 1) / 2, "fixed split changed");
    }
}

int main() {
    try {
        for (const auto split : {TwoLaneSplit::Fixed, TwoLaneSplit::Shared}) {
            test_selected_pixel_lists(split);
            test_repeated_construction(split);
            test_batch_shapes_and_profiles(split);
            test_size_validation(split);
            test_exception_isolation_and_recovery(split);
            test_random_batches(split);
        }
        test_split_switching();
        std::cout << "fixed two-lane depth sampler tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "fixed two-lane depth sampler test failed: " << error.what() << '\n';
        return 1;
    }
}
