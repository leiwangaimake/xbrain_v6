/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: fixed_two_lane_depth_sampler.hpp
 * Brief: Raw depth sampling of one frame's tracks on two fixed lanes, results index-aligned.
 *
 * Description:
 * What problem this solves. Depth sampling costs time per object on the sup_process thread, and
 * the samples of one frame's objects are independent of each other, so they can run in parallel.
 * How. The calling thread takes the even track indices and one persistent helper thread
 * (depth_lane2) the odd ones; results land at their own index, so the caller can then apply
 * the order-dependent steps (hysteresis, geometry, fusion) serially in the original track
 * order. Started once per process loop; no thread is created per frame. Production: runtime
 * switch parallel_depth_sample "1", used for frames with two or more tracks (the "canary"
 * wording in some log lines is historical). Optional per-index selected pixel lists (2026-10-06)
 * are filled the same way, for the object geometry that runs after the batch.
 * Shared split (2026-10-08, parallel_depth_sample "2"): instead of even/odd, both lanes take the
 * next index from one shared counter over the tracks ordered by box area (largest first), so a
 * lane that is late (the helper not yet scheduled, or either thread preempted) leaves its share to
 * the other; when the caller has taken every index and the helper has not even picked the batch
 * up, the batch is withdrawn from the helper and the caller returns without waiting. Every index
 * is still sampled exactly once by the same DepthSampler call, so the outputs are bit-identical.
 * Per-batch timings (last_stats) feed the diagnostic SUP_FRAME line. set_split() changes the split
 * between batches (diagnostic "alt": the two splits on alternate frames of one run, same scene).
 */

#pragma once

#include "common/thread_name.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "modules/depth_sampler.hpp"

namespace perception::internal {

// One track's input: the sampler to use (default or per-class override) and a detection built
// from the track (raw box, mask, class).
struct RawDepthSampleTask {
    const DepthSampler* sampler = nullptr;
    yolo::Detection detection;
};

// How the indices of a batch are divided between the two lanes.
enum class TwoLaneSplit {
    Fixed,   // caller even indices, helper odd indices (parallel_depth_sample "1")
    Shared,  // one shared counter over the tracks by box area, largest first ("2")
};

// Timings of the last batch (steady clock, ns from the batch start), for the diagnostic log.
struct TwoLaneBatchStats {
    TwoLaneSplit split = TwoLaneSplit::Fixed;  // the split this batch ran with
    std::size_t caller_items = 0, helper_items = 0;
    bool helper_claimed = false;         // the helper picked the batch up (false: withdrawn)
    std::int64_t helper_start_ns = -1;   // the helper picked the batch up
    std::int64_t caller_done_ns = 0;     // the caller finished its own items
    std::int64_t join_ns = 0;            // the batch was joined (sample() returns)
};

// Fixed split: the caller owns even indices and this single persistent helper owns odd indices,
// each lane serially. Shared split: see the file header. A batch is always joined (or withdrawn
// from a helper that never picked it up) before the caller can reuse its inputs or consume its
// index-aligned outputs, optional profiles and per-index exceptions. Profile storage belongs to
// the caller; an index with an exception has no usable output or profile.
class FixedTwoLaneDepthSampler {
public:
    explicit FixedTwoLaneDepthSampler(TwoLaneSplit split = TwoLaneSplit::Fixed) : split_(split) {
        // Every mutex, condition variable and batch-state member must already
        // exist before helper_loop can access them on the new thread.
        helper_thread_ = std::thread(&FixedTwoLaneDepthSampler::helper_loop, this);
    }

    ~FixedTwoLaneDepthSampler() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        work_cv_.notify_one();
        if (helper_thread_.joinable()) {
            helper_thread_.join();
        }
    }

    // Caller thread, between batches only: the split of the following batches. The helper takes
    // the split with each batch under the mutex.
    void set_split(TwoLaneSplit split) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (batch_pending_ || batch_running_ || batch_done_) {
            throw std::logic_error("depth sample split change during a batch");
        }
        split_ = split;
    }

    FixedTwoLaneDepthSampler(const FixedTwoLaneDepthSampler&) = delete;
    FixedTwoLaneDepthSampler& operator=(const FixedTwoLaneDepthSampler&) = delete;

    void sample(
        const std::vector<RawDepthSampleTask>& tasks,
        const cv::Mat& aligned_depth,
        const yolo::DepthIntrinsics& intrinsics,
        float depth_scale,
        std::vector<DepthSample>& outputs,
        std::vector<std::exception_ptr>& exceptions,
        std::vector<DepthSampleProfile>* profiles = nullptr,
        std::vector<std::vector<std::uint32_t>>* selected_indices = nullptr) {
        stats_ = {};
        stats_.split = split_;
        if (tasks.size() != outputs.size() || tasks.size() != exceptions.size()) {
            throw std::invalid_argument(
                "depth sample task/output/exception size mismatch");
        }
        if (profiles != nullptr && profiles->size() != tasks.size()) {
            throw std::invalid_argument("depth sample task/profile size mismatch");
        }
        if (selected_indices != nullptr && selected_indices->size() != tasks.size()) {
            throw std::invalid_argument("depth sample task/selected-index size mismatch");
        }
        if (tasks.empty()) {
            return;
        }

        const auto start = std::chrono::steady_clock::now();
        const auto since_start = [start]() {
            return static_cast<std::int64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count());
        };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (batch_pending_ || batch_running_ || batch_done_) {
                throw std::logic_error("depth sample helper batch overlap");
            }
            for (auto& exception : exceptions) {
                exception = nullptr;
            }
            if (split_ == TwoLaneSplit::Shared) {
                // Largest box first (ties by index), so the last indices taken are the short ones.
                order_.resize(tasks.size());
                for (std::size_t i = 0; i < tasks.size(); ++i) order_[i] = i;
                std::stable_sort(order_.begin(), order_.end(), [&tasks](std::size_t a, std::size_t b) {
                    return tasks[a].detection.bbox.area() > tasks[b].detection.bbox.area();
                });
                next_.store(0, std::memory_order_relaxed);
                helper_items_ = 0;
            }
            batch_start_ = start;
            tasks_ = &tasks;
            aligned_depth_ = &aligned_depth;
            intrinsics_ = &intrinsics;
            depth_scale_ = depth_scale;
            outputs_ = &outputs;
            exceptions_ = &exceptions;
            profiles_ = profiles;
            selected_indices_ = selected_indices;
            batch_pending_ = true;
        }
        work_cv_.notify_one();

        if (split_ == TwoLaneSplit::Shared) {
            for (std::size_t k = next_.fetch_add(1, std::memory_order_relaxed); k < tasks.size();
                 k = next_.fetch_add(1, std::memory_order_relaxed)) {
                const std::size_t i = order_[k];
                try {
                    sample_one(tasks, aligned_depth, intrinsics, depth_scale, outputs, profiles, selected_indices, i);
                } catch (...) {
                    exceptions[i] = std::current_exception();
                }
                ++stats_.caller_items;
            }
        } else {
            for (std::size_t i = 0; i < tasks.size(); i += 2U) {
                try {
                    sample_one(tasks, aligned_depth, intrinsics, depth_scale, outputs, profiles, selected_indices, i);
                } catch (...) {
                    exceptions[i] = std::current_exception();
                }
                ++stats_.caller_items;
            }
        }
        stats_.caller_done_ns = since_start();

        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (split_ == TwoLaneSplit::Shared && batch_pending_) {
                // Every index is taken and the helper never picked the batch up: withdraw it, so
                // the caller does not wait for a thread that is not even running.
                batch_pending_ = false;
            } else {
                done_cv_.wait(lock, [this]() { return batch_done_; });
                stats_.helper_claimed = true;
                stats_.helper_start_ns = helper_start_ns_;
                stats_.helper_items = helper_items_;
            }
            tasks_ = nullptr;
            aligned_depth_ = nullptr;
            intrinsics_ = nullptr;
            outputs_ = nullptr;
            exceptions_ = nullptr;
            profiles_ = nullptr;
            selected_indices_ = nullptr;
            batch_done_ = false;
        }
        stats_.join_ns = since_start();
        if (split_ == TwoLaneSplit::Fixed) stats_.helper_items = tasks.size() - stats_.caller_items;
    }

    // Timings of the last sample() call (diagnostic; caller thread only).
    const TwoLaneBatchStats& last_stats() const noexcept { return stats_; }

private:
    static void sample_one(
        const std::vector<RawDepthSampleTask>& tasks,
        const cv::Mat& aligned_depth,
        const yolo::DepthIntrinsics& intrinsics,
        float depth_scale,
        std::vector<DepthSample>& outputs,
        std::vector<DepthSampleProfile>* profiles,
        std::vector<std::vector<std::uint32_t>>* selected_indices,
        std::size_t index) {
        if (tasks[index].sampler == nullptr) {
            throw std::logic_error("null depth sampler task");
        }
        outputs[index] = tasks[index].sampler->sample(
            aligned_depth, intrinsics, depth_scale, tasks[index].detection,
            profiles != nullptr ? &(*profiles)[index] : nullptr, nullptr,
            selected_indices != nullptr ? &(*selected_indices)[index] : nullptr);
    }

    // Helper thread: wait for a batch, sample the odd indices, mark it done; exit on stop when no
    // batch is pending. A failing index stores its exception instead of ending the thread.
    void helper_loop() noexcept {
        perception::set_current_thread_name("depth_lane2");
        for (;;) {
            const std::vector<RawDepthSampleTask>* tasks = nullptr;
            const cv::Mat* aligned_depth = nullptr;
            const yolo::DepthIntrinsics* intrinsics = nullptr;
            float depth_scale = 0.0F;
            std::vector<DepthSample>* outputs = nullptr;
            std::vector<std::exception_ptr>* exceptions = nullptr;
            std::vector<DepthSampleProfile>* profiles = nullptr;
            std::vector<std::vector<std::uint32_t>>* selected_indices = nullptr;
            TwoLaneSplit split = TwoLaneSplit::Fixed;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                work_cv_.wait(lock, [this]() { return stopping_ || batch_pending_; });
                if (stopping_ && !batch_pending_) {
                    return;
                }
                tasks = tasks_;
                aligned_depth = aligned_depth_;
                intrinsics = intrinsics_;
                depth_scale = depth_scale_;
                outputs = outputs_;
                exceptions = exceptions_;
                profiles = profiles_;
                selected_indices = selected_indices_;
                split = split_;
                batch_pending_ = false;
                batch_running_ = true;
                helper_start_ns_ = static_cast<std::int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - batch_start_).count());
            }

            std::size_t items = 0;
            if (split == TwoLaneSplit::Shared) {
                for (std::size_t k = next_.fetch_add(1, std::memory_order_relaxed); k < tasks->size();
                     k = next_.fetch_add(1, std::memory_order_relaxed)) {
                    const std::size_t i = order_[k];
                    try {
                        sample_one(*tasks, *aligned_depth, *intrinsics, depth_scale, *outputs, profiles,
                                   selected_indices, i);
                    } catch (...) {
                        (*exceptions)[i] = std::current_exception();
                    }
                    ++items;
                }
            } else {
                for (std::size_t i = 1; i < tasks->size(); i += 2U) {
                    try {
                        sample_one(
                            *tasks,
                            *aligned_depth,
                            *intrinsics,
                            depth_scale,
                            *outputs,
                            profiles,
                            selected_indices,
                            i);
                    } catch (...) {
                        (*exceptions)[i] = std::current_exception();
                    }
                    ++items;
                }
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                helper_items_ = items;
                batch_running_ = false;
                batch_done_ = true;
            }
            done_cv_.notify_one();
        }
    }

    TwoLaneSplit split_;  // written by the caller under mutex_ between batches
    std::thread helper_thread_;
    std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable done_cv_;
    bool stopping_ = false;
    bool batch_pending_ = false;
    bool batch_running_ = false;
    bool batch_done_ = false;
    const std::vector<RawDepthSampleTask>* tasks_ = nullptr;
    const cv::Mat* aligned_depth_ = nullptr;
    const yolo::DepthIntrinsics* intrinsics_ = nullptr;
    float depth_scale_ = 0.0F;
    std::vector<DepthSample>* outputs_ = nullptr;
    std::vector<std::exception_ptr>* exceptions_ = nullptr;
    std::vector<DepthSampleProfile>* profiles_ = nullptr;
    std::vector<std::vector<std::uint32_t>>* selected_indices_ = nullptr;
    // Shared split: the visiting order (written before the batch is published under mutex_) and
    // the next position in it.
    std::vector<std::size_t> order_;
    std::atomic<std::size_t> next_{0};
    std::chrono::steady_clock::time_point batch_start_{};
    std::int64_t helper_start_ns_ = -1;  // under mutex_
    std::size_t helper_items_ = 0;       // under mutex_
    TwoLaneBatchStats stats_;            // caller thread only
};

}  // namespace perception::internal
