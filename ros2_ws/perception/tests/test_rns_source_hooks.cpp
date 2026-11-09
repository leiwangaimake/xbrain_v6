/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_source_hooks.cpp
 * Brief: Actual source hook/journal components with explicit synthetic events.
 *
 * Description:
 * This does NOT execute Orbbec, TensorRT, D2C or a production output queue.
 */

#include "output/rns_source_hooks.hpp"
#include "common/time_utils.hpp"
#include <array>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

using namespace perception::rns;
namespace nd = perception::native_depth;
namespace {
std::size_t checks = 0, groups = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __FILE__ << ':' << __LINE__ \
    << ": " #x "\n"; std::abort(); } } while (false)

struct Clock {
    std::int64_t now = 2000000000LL;
    std::size_t reads = 0;
    static std::int64_t read(void* context) noexcept {
        auto& self = *static_cast<Clock*>(context); ++self.reads; return self.now;
    }
    SourceClock source_clock() noexcept { return {this, read}; }
};
nd::SourceId source(std::uint64_t sequence = 1) {
    return {17, sequence, 100 + sequence, 200 + sequence,
        3000000 + sequence, 3000020 + sequence, 1000000000LL + static_cast<std::int64_t>(sequence)};
}
bool same(const nd::SourceId& a, const nd::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
SourceEvent take(SourceJournal& j, SourceKind kind, const nd::SourceId& expected, std::int64_t time) {
    SourceEvent event; CHECK(j.pop(event));
    CHECK(event.kind == kind && same(event.source, expected) && event.observed_ns == time);
    return event;
}

void exclusions_and_null() {
    Clock clock; SourceJournal journal{SourceKind::InferenceComplete, 8, clock.source_clock()};
    auto s = source(); const auto original = s;
    // The helper has no result-buffer argument. These sentinels attest only
    // to this caller's supplied state; they are NOT captured model outputs.
    const std::array<float, 4> output{1, 2, -3, 0}; const auto before = output;
    CHECK(!record_inference_completion(nullptr, true, s));
    CHECK(!record_inference_completion(nullptr, false, s));
    CHECK(clock.reads == 0 && journal.published() == 0 && output == before && same(s, original));
    // Both an inference failure and the no-TensorRT branch supply false.
    for (int excluded = 0; excluded < 2; ++excluded)
        CHECK(!record_inference_completion(&journal, false, s));
    // Invalid/missing source is irrelevant when the operation is excluded:
    // no clock call and no sticky source fault should be manufactured.
    CHECK(!record_inference_completion(&journal, false, {}));
    CHECK(clock.reads == 0 && journal.generation() == 0 && journal.valid());
    CHECK(journal.fault_count() == 0 && journal.published() == 0 && output == before);
    for (const auto kind : {SourceKind::DepthArrival, SourceKind::DepthProcessed}) {
        Clock c; SourceJournal wrong{kind, 2, c.source_clock()};
        CHECK(!record_inference_completion(&wrong, true, s));
        CHECK(c.reads == 0 && wrong.published() == 0 && wrong.generation() == 0 && wrong.valid());
    }
    ++groups;
}

void completion_and_missing_source() {
    Clock clock; SourceJournal journal{SourceKind::InferenceComplete, 8, clock.source_clock()};
    const auto s = source();
    // There is intentionally NO detection-count parameter: an empty successful
    // scene and a populated successful scene traverse exactly the same hook.
    const std::array<int, 0> empty_detections{};
    CHECK(empty_detections.empty());
    CHECK(record_inference_completion(&journal, true, s));
    CHECK(clock.reads == 1 && journal.published() == 1 && journal.generation() == 2);
    const auto event = take(journal, SourceKind::InferenceComplete, s, clock.now);
    CHECK(event.depth.code == DepthHealthCode::InvalidFrame && !event.depth.invalid_ratio);
    CHECK(event.depth.roi_pixels == 0 && event.depth.invalid_pixels == 0);
    CHECK(journal.consumed() == 1); SourceEvent absent; CHECK(!journal.pop(absent));
    CHECK(!record_inference_completion(&journal, true, s));
    CHECK(!journal.valid() && journal.fault_count() == 1 && journal.published() == 1);
    CHECK(!(journal.generation() & 1U));

    for (int missing = 0; missing < 5; ++missing) {
        Clock c; SourceJournal j{SourceKind::InferenceComplete, 2, c.source_clock()}; auto bad = source();
        if (missing == 0) bad = {};
        if (missing == 1) bad.stream_id = 0;
        if (missing == 2) bad.sequence = 0;
        if (missing == 3) bad.color_device_us = 0;
        if (missing == 4) bad.depth_device_us = 0;
        CHECK(!record_inference_completion(&j, true, bad));
        CHECK(!j.valid() && j.fault_count() == 1 && j.published() == 0 && c.reads == 1);
        CHECK(!(j.generation() & 1U));
    }
    ++groups;
}

void depth_arrival_survives_synthetic_downstream_failure() {
    Clock arrival_clock, inference_clock, processed_clock;
    SourceJournal arrival{SourceKind::DepthArrival, 8, arrival_clock.source_clock()};
    SourceJournal inference{SourceKind::InferenceComplete, 8, inference_clock.source_clock()};
    SourceJournal processed{SourceKind::DepthProcessed, 8, processed_clock.source_clock()};
    enum class SyntheticFailure { MissingRgb, D2cFailed, MailboxBusy };
    std::uint64_t seq = 0;
    for (const auto failure : {SyntheticFailure::MissingRgb, SyntheticFailure::D2cFailed, SyntheticFailure::MailboxBusy}) {
        auto s = source(++seq);
        if (failure == SyntheticFailure::MissingRgb) { s.color_index = 0; s.color_device_us = 0; s.depth_index = 0; }
        CHECK(arrival.record(s));
        const auto count_after_arrival = arrival.published();
        // Simulates later control flow ONLY: no actual SDK alignment, mailbox
        // contention or physical missing-RGB frame is claimed by this test.
        const bool inference_succeeded = false;
        CHECK(!record_inference_completion(&inference, inference_succeeded, s));
        CHECK(arrival.published() == count_after_arrival);
        const auto e = take(arrival, SourceKind::DepthArrival, s, arrival_clock.now);
        CHECK(e.depth.code == DepthHealthCode::InvalidFrame && !e.depth.invalid_ratio);
    }
    CHECK(arrival.valid() && arrival.published() == 3 && arrival_clock.reads == 3);
    CHECK(inference.valid() && inference.published() == 0 && inference_clock.reads == 0);
    CHECK(processed.valid() && processed.published() == 0 && processed_clock.reads == 0);
    // Arrival's absent colour allowance does NOT admit a missing depth time.
    Clock bad_clock; SourceJournal bad_arrival{SourceKind::DepthArrival, 2, bad_clock.source_clock()};
    auto bad = source(); bad.depth_device_us = 0;
    CHECK(!bad_arrival.record(bad) && !bad_arrival.valid() && bad_arrival.fault_count() == 1);
    ++groups;
}

void independent_kinds_and_real_roi_scan() {
    Clock ca, ci, cp; ca.now = 1100000000LL; ci.now = 1300000000LL; cp.now = 1400000000LL;
    SourceJournal arrival{SourceKind::DepthArrival, 8, ca.source_clock()};
    SourceJournal inference{SourceKind::InferenceComplete, 8, ci.source_clock()};
    SourceJournal processed{SourceKind::DepthProcessed, 8, cp.source_clock()};
    nd::Mailbox mailbox{8}; nd::Frame frame{8}; const std::array<std::uint16_t, 8> all_invalid{};
    auto s = source(); s.stream_id = mailbox.begin_stream();
    nd::View view; view.metadata.source = s; view.metadata.intrinsics = {1, 1, 0, 0, 2, 4};
    view.metadata.distortion.reported = true; view.metadata.unit_m = .001F;
    view.pixels = all_invalid.data(); view.bytes = sizeof(all_invalid); view.pre_d2c = true;
    CHECK(arrival.record(s));
    CHECK(inference.published() == 0 && processed.published() == 0);
    CHECK(mailbox.try_offer(view) == nd::Offer::Accepted && mailbox.try_take(frame) == nd::Take::Taken);
    // Compute, do not handwrite, the ROI evidence using the production pure
    // raster routine. Raster/source itself is explicitly synthetic.
    const auto observation = observe_depth_health(frame, {});
    CHECK(observation.code == DepthHealthCode::Ready && observation.roi_pixels == 4 &&
        observation.invalid_pixels == 4 && observation.invalid_ratio == 1);
    CHECK(processed.record(observation));
    CHECK(arrival.published() == 1 && inference.published() == 0 && processed.published() == 1);
    CHECK(record_inference_completion(&inference, true, s));
    const auto a = take(arrival, SourceKind::DepthArrival, s, ca.now);
    const auto i = take(inference, SourceKind::InferenceComplete, s, ci.now);
    const auto p = take(processed, SourceKind::DepthProcessed, s, cp.now);
    CHECK(a.observed_ns < i.observed_ns && i.observed_ns < p.observed_ns);
    CHECK(p.observed_ns != s.host_ready_mono_ns);
    CHECK(!a.depth.invalid_ratio && !i.depth.invalid_ratio && p.depth.invalid_ratio == 1);
    CHECK(p.depth.code == observation.code && same(p.depth.source, s) && p.depth.roi_kind == observation.roi_kind);
    CHECK(p.depth.roi_pixels == 4 && p.depth.invalid_pixels == 4 && cp.reads == 1);
    CHECK(arrival.valid() && inference.valid() && processed.valid());
    // Overload misuse must not relabel unprocessed arrival as a measured ratio.
    Clock c; SourceJournal wrong{SourceKind::DepthProcessed, 2, c.source_clock()};
    CHECK(!wrong.record(s) && !wrong.valid() && wrong.published() == 0);
    ++groups;
}

struct ProfileEnvironment {
    std::optional<std::string> previous;
    ProfileEnvironment() {
        if (const char* s = std::getenv("PERCEPTION_INFERENCE_PROFILE")) previous = s;
        CHECK(::setenv("PERCEPTION_INFERENCE_PROFILE", "0", 1) == 0);
    }
    ~ProfileEnvironment() {
        if (previous) (void)::setenv("PERCEPTION_INFERENCE_PROFILE", previous->c_str(), 1);
        else (void)::unsetenv("PERCEPTION_INFERENCE_PROFILE");
    }
};
void profiling_off_and_all_fields() {
    ProfileEnvironment restore;
    CHECK(!perception::inference_profile_enabled());
    Clock clock; SourceJournal journal{SourceKind::InferenceComplete, 8, clock.source_clock()};
    const auto s = source(); const auto saved = s;
    // No PipelineTiming object is passed into this real hook. It must use its
    // supplied SourceId and clock even when the actual profiling switch is off.
    CHECK(record_inference_completion(&journal, true, s));
    const auto event = take(journal, SourceKind::InferenceComplete, saved, clock.now);
    CHECK(event.source.stream_id == 17 && event.source.sequence == 1);
    CHECK(event.source.color_index == 101 && event.source.depth_index == 201);
    CHECK(event.source.color_device_us == 3000001 && event.source.depth_device_us == 3000021);
    CHECK(event.source.host_ready_mono_ns == 1000000001LL && same(s, saved));
    CHECK(event.observed_ns == 2000000000LL && clock.reads == 1 && journal.valid());
    ++groups;
}
} // namespace
int main() {
    exclusions_and_null(); completion_and_missing_source();
    depth_arrival_survives_synthetic_downstream_failure(); independent_kinds_and_real_roi_scan();
    profiling_off_and_all_fields();
    std::cout << "PASS source_hooks groups=" << groups << " checks=" << checks
        << " synthetic component-only; no Orbbec/TensorRT/D2C/production execution\n";
}
