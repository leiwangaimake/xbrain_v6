/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_execution_cpu_postprocess.cpp
 * Brief: Unit tests that CPU tracing leaves the real postprocess and merge results unchanged.
 *
 * Description:
 * What it checks. Postprocess, role dispatch, mask ownership and dual-model merge are run with and
 * without the execution CPU trace on the persistent task pair; masks, detections and the merged
 * result must be identical, so the trace is observational only.
 */

#include "common/execution_cpu_trace.hpp"
#include "common/persistent_task_pair.hpp"
#include "common/postprocess.hpp"
#include "common/post_role_dispatch.hpp"
#include "common/postprocess_mask_ownership.hpp"
#include "common/dual_model_merge.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
int checks = 0;
void check(bool value) { ++checks; if (!value) throw std::runtime_error("post CPU trace check " + std::to_string(checks)); }
bool same_mat(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty()) return a.empty() == b.empty();
    if (a.size() != b.size() || a.type() != b.type()) return false;
    for (int row = 0; row < a.rows; ++row)
        if (std::memcmp(a.ptr(row), b.ptr(row), static_cast<std::size_t>(a.cols) * a.elemSize()) != 0) return false;
    return true;
}
void same(const std::vector<yolo::Detection>& a, const std::vector<yolo::Detection>& b) {
    check(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        check(a[i].bbox == b[i].bbox && a[i].class_id == b[i].class_id &&
              a[i].original_class_id == b[i].original_class_id && a[i].class_name == b[i].class_name &&
              a[i].source == b[i].source && a[i].low_confidence_candidate == b[i].low_confidence_candidate);
        check(std::memcmp(&a[i].confidence, &b[i].confidence, sizeof(float)) == 0);
        check(same_mat(a[i].mask, b[i].mask)); check(same_mat(a[i].full_image_mask, b[i].full_image_mask));
    }
}
void fixture(int width, int height, int count, yolo::Active12MaskImplementation mode) {
    constexpr std::int64_t anchors = 16464;
    std::vector<float> output(48 * anchors, 0.0F), proto(32 * 224 * 224, 0.0F);
    yolo::LetterboxInfo info;
    info.original_width = width; info.original_height = height;
    info.scale = std::min(896.0F / width, 896.0F / height);
    info.new_width = static_cast<int>(std::round(width * info.scale));
    info.new_height = static_cast<int>(std::round(height * info.scale));
    info.pad_left = (896 - info.new_width) / 2; info.pad_top = (896 - info.new_height) / 2;
    for (int i = 0; i < 224 * 224; ++i) proto[i] = (i % 7 == 0) ? -2.0F : 2.0F;
    for (int i = 0; i < count; ++i) {
        const float x = 0.05F + 0.20F * i, y = 0.07F + 0.14F * i;
        const float w = width * 0.35F * info.scale, h = height * 0.42F * info.scale;
        output[i] = info.pad_left + width * x * info.scale + w / 2;
        output[anchors + i] = info.pad_top + height * y * info.scale + h / 2;
        output[2 * anchors + i] = w; output[3 * anchors + i] = h;
        output[(4 + (i == 0 ? 10 : i)) * anchors + i] = 0.95F - i * 0.05F;
        output[16 * anchors + i] = 1.0F;
    }
    const auto input_before = output, proto_before = proto;
    perception::PersistentTaskPair lanes;
    yolo::Active12MaskProfile off, on;
    auto decode = [&](bool enabled, yolo::Active12MaskProfile* profile) {
        return yolo::postprocess_seg(output.data(), {1,48,anchors}, proto.data(), {1,32,224,224},
            info, yolo::active12_seg_model_spec(), 0.25F, 0.7F, 0.5F, 896, mode, &lanes, profile,
            yolo::Active12MaskBinaryImplementation::Legacy, enabled);
    };
    const auto old = decode(false, &off), measured = decode(true, &on);
    same(old, measured); same(measured, decode(false, nullptr));
    check(output == input_before && proto == proto_before);
    check(old.size() == static_cast<std::size_t>(count) && on.tasks.size() == old.size());
    std::set<std::int64_t> tids;
    for (std::size_t i = 0; i < on.tasks.size(); ++i) {
        const auto& a = off.tasks[i]; const auto& b = on.tasks[i];
        check(!a.execution_cpu.enabled && !a.execution_cpu.valid && a.execution_cpu.cpu_ns == -1);
        check(b.index == i && b.lane == (mode == yolo::Active12MaskImplementation::Parallel2 ? static_cast<int>(i % 2) : 0));
        check(b.execution_cpu.enabled);
#ifdef __linux__
        check(b.execution_cpu.valid && b.execution_cpu.cpu_ns > 0 && b.execution_cpu.wall_ns > 0);
        check(b.total_ns >= b.execution_cpu.wall_ns);
        tids.insert(b.execution_cpu.tid);
#else
        check(!b.execution_cpu.valid && b.execution_cpu.cpu_ns == -1 &&
              std::string(b.execution_cpu.reason) == "unsupported_platform");
#endif
        check(b.gemm_ns + b.resize_ns + b.threshold_convert_ns + b.canvas_ns == b.total_ns);
    }
#ifdef __linux__
    const auto expected_tids = count == 0 ? 0U : mode == yolo::Active12MaskImplementation::Parallel2 && count >= 2 ? 2U : 1U;
    check(tids.size() == expected_tids);
#endif
}

struct RawSeg {
    std::vector<float> output, proto;
    yolo::LetterboxInfo info;
    std::int64_t anchors;
    int input, classes, proto_size;
    RawSeg(int width, int height, int count, bool active)
        : anchors(active ? 16464 : 8400), input(active ? 896 : 640),
          classes(active ? 12 : 80), proto_size(input / 4) {
        output.assign((4 + classes + 32) * anchors, 0.0F);
        proto.assign(32 * proto_size * proto_size, 0.0F);
        info.original_width = width; info.original_height = height;
        info.scale = std::min(static_cast<float>(input) / width, static_cast<float>(input) / height);
        info.new_width = static_cast<int>(std::round(width * info.scale));
        info.new_height = static_cast<int>(std::round(height * info.scale));
        info.pad_left = (input - info.new_width) / 2; info.pad_top = (input - info.new_height) / 2;
        for (int i = 0; i < proto_size * proto_size; ++i) proto[i] = i % 7 == 0 ? -2.0F : 2.0F;
        for (int i = 0; i < count; ++i) {
            const float w = width * .35F * info.scale, h = height * .42F * info.scale;
            output[i] = info.pad_left + width * (.05F + .20F * i) * info.scale + w / 2;
            output[anchors + i] = info.pad_top + height * (.07F + .14F * i) * info.scale + h / 2;
            output[2 * anchors + i] = w; output[3 * anchors + i] = h;
            const int cls = active ? (i == 0 ? 10 : i == 1 ? 9 : i) : (i == 1 ? 2 : i);
            output[(4 + cls) * anchors + i] = .95F - i * .05F;
            output[(4 + classes) * anchors + i] = 1.0F;
        }
    }
};

void same_merged(const yolo::DualModelMergeOutput& a, const yolo::DualModelMergeOutput& b) {
    same(a.merged_detections, b.merged_detections);
    same(a.tracker_detections, b.tracker_detections);
    same(a.map_detections, b.map_detections);
    check(a.conflicts.size() == b.conflicts.size());
    for (std::size_t i = 0; i < a.conflicts.size(); ++i) {
        const auto& x = a.conflicts[i]; const auto& y = b.conflicts[i];
        check(x.kind == y.kind && x.old_index == y.old_index && x.active_index == y.active_index &&
              x.old_class_id == y.old_class_id && x.active_class_id == y.active_class_id &&
              x.mode == y.mode && x.old_suppressed == y.old_suppressed);
        check(std::memcmp(&x.box_iou, &y.box_iou, sizeof(float)) == 0);
    }
}

void role_fixture(int width, int height, int count) {
    const RawSeg coco(width, height, count, false), active(width, height, count, true);
    const auto coco_before = coco.output, coco_proto_before = coco.proto;
    const auto active_before = active.output, active_proto_before = active.proto;
    perception::ParallelTaskWorker worker;
    perception::PersistentTaskPair mask_lanes;
    auto decode_coco = [&] {
        auto result = yolo::postprocess_seg(coco.output.data(), {1,116,coco.anchors}, coco.proto.data(),
            {1,32,coco.proto_size,coco.proto_size}, coco.info, .25F, {}, 0.0F, 0, .7F, .5F, coco.input);
        (void)perception::finalize_postprocess_mask_ownership(result, perception::MaskOutputCopyMode::Clone, false);
        return result;
    };
    auto decode_active = [&](yolo::Active12MaskProfile* profile, bool cpu) {
        auto result = yolo::postprocess_seg(active.output.data(), {1,48,active.anchors}, active.proto.data(),
            {1,32,active.proto_size,active.proto_size}, active.info, yolo::active12_seg_model_spec(), .25F, .7F, .5F,
            active.input, yolo::Active12MaskImplementation::Parallel2, &mask_lanes, profile,
            yolo::Active12MaskBinaryImplementation::Legacy, cpu);
        (void)perception::finalize_postprocess_mask_ownership(result, perception::MaskOutputCopyMode::Clone, true);
        return result;
    };
    const auto reference_coco = decode_coco(), reference_active = decode_active(nullptr, false);
    check(reference_coco.size() == static_cast<std::size_t>(count));
    check(reference_active.size() == static_cast<std::size_t>(count));
    const auto reference_merged = yolo::merge_dual_model_detections(reference_coco, reference_active);
    const auto caller = std::this_thread::get_id();
    // A/B/A exercises role reuse with the same persistent outer worker and
    // nested mask lanes, immutable host arrays, clone policy and merge order.
    for (bool swap : {false, true, false}) {
        std::vector<yolo::Detection> coco_result, active_result;
        perception::ExecutionCpuRecord coco_cpu, active_cpu, join;
        yolo::Active12MaskProfile mask_profile;
        std::thread::id coco_tid, active_tid;
        perception::run_post_tasks(worker, swap, [&] {
            perception::ExecutionCpuScope scope(coco_cpu, true);
            coco_tid = std::this_thread::get_id();
            coco_result = decode_coco();
        }, [&] {
            perception::ExecutionCpuScope scope(active_cpu, true);
            active_tid = std::this_thread::get_id();
            active_result = decode_active(&mask_profile, true);
        }, &join);
        same(reference_coco, coco_result); same(reference_active, active_result);
        same_merged(reference_merged, yolo::merge_dual_model_detections(coco_result, active_result));
        check(coco.output == coco_before && coco.proto == coco_proto_before &&
              active.output == active_before && active.proto == active_proto_before);
        check(coco_tid != std::thread::id() && coco_tid != active_tid && (swap ? coco_tid : active_tid) == caller);
#ifdef __linux__
        // Kernel thread IDs: the join wait and the caller-role task ran on this
        // thread, the helper-role task on another one.
        const auto caller_kernel_tid = perception::execution_cpu_stamp().tid;
        check(coco_cpu.valid && active_cpu.valid && join.valid && join.tid == caller_kernel_tid);
        check((swap ? coco_cpu.tid : active_cpu.tid) == caller_kernel_tid && coco_cpu.tid != active_cpu.tid);
        for (const auto& task : mask_profile.tasks) {
            check(task.execution_cpu.valid);
            check(task.lane == 0 ? task.execution_cpu.tid == active_cpu.tid :
                                  task.execution_cpu.tid != active_cpu.tid && task.execution_cpu.tid != coco_cpu.tid);
        }
#else
        check(!coco_cpu.valid && !active_cpu.valid && !join.valid);
#endif
    }
}
}
int main() {
    try {
        cv::setNumThreads(1);
        for (auto mode : {yolo::Active12MaskImplementation::Serial, yolo::Active12MaskImplementation::Parallel2,
                          yolo::Active12MaskImplementation::Shadow})
            for (int count : {0,1,4}) {
                fixture(1280,800,count,mode); fixture(1279,799,count,mode);
            }
        for (int count : {0,1,4}) { role_fixture(1280,800,count); role_fixture(1279,799,count); }
        std::cout << "{\"status\":\"PASS\",\"fixtures\":18,\"checks\":" << checks
                  << ",\"timing_off_on_pixels_and_order_equal\":true,\"real_postprocess\":true,\"model_execution\":false"
                  << ",\"post_role_fixtures\":6,\"post_role_ABA_calls\":18,\"role_pixels_order_and_merge_equal\":true}\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
