/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_jit_source.cpp
 * Brief: Unit tests for binding each JIT inference completion to its own camera source.
 *
 * Description:
 * What it checks. Interleaved slots keep their own sources; no completion comes from an unbound,
 * wrong, failed or abandoned slot; calibration frames and an empty scene are normal completions;
 * a missing source is never invented, and a journal failure does not change a success.
 * Uses the real production source-binding helper, result objects and source journal with synthetic
 * metadata only; does not execute TensorRT, Orbbec, ROS or RNS sends.
 */

#include "common/inference_source_binding.hpp"

#include <cstdlib>
#include <iostream>

namespace {
using perception::InferenceSourceBinding;
using perception::rns::SourceJournal;
using perception::rns::SourceKind;
using perception::rns::SourceEvent;
namespace nd = perception::native_depth;
int checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; std::abort(); } } while (false)

struct Clock {
    std::int64_t now = 2000000000LL;
    int reads = 0;
    static std::int64_t read(void* p) noexcept {
        auto& c = *static_cast<Clock*>(p); ++c.reads; return c.now++;
    }
    perception::rns::SourceClock source_clock() noexcept { return {this, read}; }
};

yolo::FrameData frame(std::uint64_t id, std::uint64_t sequence) {
    yolo::FrameData f;
    f.frame_id = id;
    f.native_depth_source = {71, sequence, 100 + sequence, 200 + sequence,
        3000000 + sequence, 3000020 + sequence,
        1000000000LL + static_cast<std::int64_t>(sequence)};
    // Profiling is deliberately invalid/zero: source lifetime is independent.
    CHECK(!f.pipeline_timing.valid && f.pipeline_timing.capture_ready_ns == 0);
    return f;
}

bool same(const nd::SourceId& a, const nd::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
void take(SourceJournal& journal, const nd::SourceId& expected) {
    SourceEvent event;
    CHECK(journal.pop(event) && event.kind == SourceKind::InferenceComplete);
    CHECK(same(event.source, expected));
}
template<class F> void rejects(F&& f) {
    bool rejected = false;
    try { f(); } catch (const std::logic_error&) { rejected = true; }
    CHECK(rejected);
}

void interleaved_slots_own_their_sources() {
    Clock clock;
    SourceJournal journal{SourceKind::InferenceComplete, 8, clock.source_clock()};
    InferenceSourceBinding slots[2];
    auto a = frame(11, 1), b = frame(15, 2), c = frame(20, 3);
    const auto as = a.native_depth_source, bs = b.native_depth_source, cs = c.native_depth_source;
    slots[0].bind(a);
    slots[1].bind(b); // Next input may be prepared/submitted before current post.
    a.native_depth_source = {}; b.native_depth_source = {}; // Bound copies must survive.
    yolo::DetectionResult ra; ra.frame_id = 11;
    slots[0].complete(ra, true, &journal);
    CHECK(ra.inference_completed && ra.detections.empty() && same(ra.native_depth_source, as));
    take(journal, as);
    rejects([&] { slots[0].complete(ra, true, &journal); });

    slots[0].bind(c); // Reuse completed slot while the other current result completes.
    yolo::DetectionResult rb; rb.frame_id = 15;
    slots[1].complete(rb, true, &journal);
    CHECK(rb.inference_completed && same(rb.native_depth_source, bs));
    take(journal, bs);
    yolo::DetectionResult rc; rc.frame_id = 20;
    slots[0].complete(rc, true, &journal);
    CHECK(rc.inference_completed && same(rc.native_depth_source, cs));
    take(journal, cs);
    CHECK(journal.valid() && journal.published() == 3 && clock.reads == 3);
}

void no_completion_from_unbound_wrong_failed_or_abandoned() {
    Clock clock;
    SourceJournal journal{SourceKind::InferenceComplete, 8, clock.source_clock()};
    InferenceSourceBinding slot;
    yolo::DetectionResult output; output.frame_id = 5;
    rejects([&] { slot.complete(output, true, &journal); });
    auto f = frame(5, 1); slot.bind(f);
    output.frame_id = 6;
    rejects([&] { slot.complete(output, true, &journal); });
    CHECK(!output.inference_completed && output.native_depth_source.stream_id == 0);
    output.frame_id = 5;
    slot.complete(output, false, &journal);
    CHECK(!output.inference_completed && same(output.native_depth_source, f.native_depth_source));
    {
        InferenceSourceBinding prepared_or_drained;
        prepared_or_drained.bind(frame(9, 2));
        // Actual helper destruction has no completion effect. The JIT normal
        // stop/error drain does not call complete for unpublished slots.
    }
    CHECK(journal.valid() && journal.published() == 0 && clock.reads == 0);
}

void calibration_frames_and_empty_scene_are_normal_completions() {
    Clock clock;
    SourceJournal journal{SourceKind::InferenceComplete, 2, clock.source_clock()};
    InferenceSourceBinding slot;
    // Same production helper for 120 warmup + 300 calibration completions;
    // there is no log-phase filter in the production completion call.
    for (std::uint64_t i = 1; i <= 420; ++i) {
        auto f = frame(1000 + i, i);
        slot.bind(f);
        yolo::DetectionResult result; result.frame_id = f.frame_id;
        slot.complete(result, true, &journal);
        CHECK(result.inference_completed && result.detections.empty());
        CHECK(same(result.native_depth_source, f.native_depth_source));
        take(journal, f.native_depth_source);
    }
    CHECK(journal.valid() && journal.published() == 420 && clock.reads == 420);
}

void missing_source_is_not_invented_and_journal_failure_does_not_change_success() {
    Clock clock;
    SourceJournal journal{SourceKind::InferenceComplete, 2, clock.source_clock()};
    InferenceSourceBinding slot;
    yolo::FrameData f; f.frame_id = 3;
    slot.bind(f);
    yolo::DetectionResult result; result.frame_id = 3;
    slot.complete(result, true, &journal);
    CHECK(result.inference_completed && same(result.native_depth_source, nd::SourceId{}));
    CHECK(!journal.valid() && journal.published() == 0 && journal.fault_count() == 1);
    auto normal = frame(4, 2); slot.bind(normal); result.frame_id = 4;
    slot.complete(result, true, nullptr);
    CHECK(result.inference_completed && same(result.native_depth_source, normal.native_depth_source));
    CHECK(clock.reads == 1);
}
}  // namespace
int main() {
    interleaved_slots_own_their_sources();
    no_completion_from_unbound_wrong_failed_or_abandoned();
    calibration_frames_and_empty_scene_are_normal_completions();
    missing_source_is_not_invented_and_journal_failure_does_not_change_success();
    std::cout << "PASS rns_jit_source checks=" << checks
              << " production helper; synthetic source only, no device/backend execution\n";
}
