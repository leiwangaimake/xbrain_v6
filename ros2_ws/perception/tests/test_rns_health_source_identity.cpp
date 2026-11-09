/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_health_source_identity.cpp
 * Brief: Native rate identity regression for the health window; synthetic values, no devices or network.
 *
 * Description:
 * What it checks. Health rates must count each native source frame exactly once. The tests cover
 * same-millisecond and same-nanosecond arrivals, exact replays versus conflicting identities,
 * fresh ordering and rejected identities, stream changes and independent rate streams, basic
 * source and time faults, window expiry and the global clock, capacity and lifetime overflow, and
 * the split publication path beside native rates without allocation.
 */

#include "output/rns_health_window.hpp"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#ifdef NDEBUG
#error "Native health identity tests require enabled assertions"
#endif
static std::atomic<std::size_t> ordinary_news{0}, aligned_news{0};
void* operator new(std::size_t n) {
    ++ordinary_news; if (void* p = std::malloc(n ? n : 1)) return p; throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    try { return ::operator new(n); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept { return ::operator new(n, std::nothrow); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t alignment) {
    ++aligned_news; void* p = nullptr;
    if (posix_memalign(&p, static_cast<std::size_t>(alignment), n ? n : 1) == 0) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n, std::align_val_t a) { return ::operator new(n, a); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    try { return ::operator new(n, a); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return ::operator new(n, a, std::nothrow);
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }

namespace perception::rns {
struct HealthSourceIdentityTestAccess {
    static Millis capture(const HealthWindow& w, std::size_t stream, std::size_t index) {
        const auto& s = w.streams_[stream];
        return s.events[(s.begin + index) % s.events.size()].capture;
    }
    static native_depth::SourceId last(const HealthWindow& w, std::size_t index) {
        return *w.streams_[index].last_source;
    }
    static void total(HealthWindow& w, std::size_t index, std::uint64_t value) {
        w.streams_[index].total_events = value;
    }
};
}
using namespace perception::rns;
namespace nd = perception::native_depth;
namespace {
std::size_t groups = 0;
void pass(const char* name) { ++groups; std::cout << "PASS " << name << '\n'; }
template<class F> auto no_allocation(F fn) {
    const auto a = ordinary_news.load(), b = aligned_news.load(); const auto result = fn();
    assert(ordinary_news == a && aligned_news == b); return result;
}
nd::SourceId source(std::uint64_t seq = 1, std::int64_t ns = 10000001) {
    nd::SourceId s; s.stream_id = 7; s.sequence = seq;
    s.color_index = seq - 1; s.depth_index = seq - 1;
    s.color_device_us = seq + 20; s.depth_device_us = seq + 30;
    s.host_ready_mono_ns = ns; return s;
}
HealthEventCode observe(HealthWindow& w, bool inference, const nd::SourceId& s,
                        Millis event, Millis ingest) {
    return no_allocation([&] { return inference ? w.observe_inference(s, event, ingest)
                                              : w.observe_depth(s, event, ingest); });
}
HealthWindowReport snapshot(HealthWindow& w, Millis now) {
    return no_allocation([&] { return w.snapshot(now); });
}
PublishAudit audit(Millis capture, Millis before, std::uint64_t seq = 1) {
    PublishAudit a; a.attempted = true; a.clock_valid = true; a.seq = seq;
    a.capture_ms = capture; a.put_before_ms = before; a.put_return_ms = before + 1;
    a.packet_bytes = 100; a.receipt = {PutOutcome::Accepted, 0}; return a;
}
void same_millisecond_and_nanosecond() {
    HealthWindow w(0, 8);
    auto first = source(9000000), second = source(9000001, 10999999), third = source(9000002, 10999999);
    assert(observe(w, false, first, 12, 12) == HealthEventCode::Accepted);
    assert(observe(w, false, second, 12, 12) == HealthEventCode::Accepted);
    assert(observe(w, false, third, 12, 12) == HealthEventCode::Accepted);
    for (std::size_t i = 0; i < 3; ++i)
        assert(HealthSourceIdentityTestAccess::capture(w, 0, i) == 10); // Real clock, not sequence.
    assert(observe(w, false, third, 12, 13) == HealthEventCode::Duplicate);
    auto s = snapshot(w, 13);
    assert(s.valid && s.depth.window_frames == 3 && s.depth.total_frames == 3 && s.depth.fps == .3);
    assert(!s.inference.ever_observed && s.diagnostics.duplicates == 1);
    // Independent rate streams may accept exactly the same source.
    assert(observe(w, true, first, 13, 13) == HealthEventCode::Accepted);
    assert(observe(w, true, second, 13, 13) == HealthEventCode::Accepted);
    assert(observe(w, true, third, 13, 13) == HealthEventCode::Accepted);
    s = snapshot(w, 13); assert(s.valid && s.inference.total_frames == 3 && s.inference.fps == .3);
    pass("same_millisecond_and_same_nanosecond_distinct_real_sources_count_independently");
}
void exact_replay_and_conflict() {
    for (bool inference : {false, true}) {
        HealthWindow w(0, 8); const auto first = source(10);
        assert(observe(w, inference, first, 20, 20) == HealthEventCode::Accepted);
        // Replayed event may be older than the last event, but still coherent
        // with its capture; it is not a fresh per-stream clock regression.
        assert(observe(w, inference, first, 19, 21) == HealthEventCode::Duplicate);
        assert(observe(w, inference, source(9, 9000000), 19, 22) == HealthEventCode::OutOfOrder);
        assert(observe(w, inference, first, 20, 23) == HealthEventCode::Duplicate);
        const auto s = snapshot(w, 23);
        assert(s.valid && s.diagnostics.duplicates == 2 && s.diagnostics.out_of_order == 1);
        assert((inference ? s.inference : s.depth).total_frames == 1);
    }
    for (unsigned field = 0; field < 5; ++field) {
        HealthWindow w(0, 8); const auto first = source();
        assert(observe(w, false, first, 12, 12) == HealthEventCode::Accepted);
        auto conflict = first;
        switch (field) {
            case 0: ++conflict.color_index; break;
            case 1: ++conflict.depth_index; break;
            case 2: ++conflict.color_device_us; break;
            case 3: ++conflict.depth_device_us; break;
            default: ++conflict.host_ready_mono_ns; break;
        }
        assert(observe(w, false, conflict, 12, 12) == HealthEventCode::Invalid);
        assert(observe(w, false, first, 12, 12) == HealthEventCode::Duplicate);
        assert(HealthSourceIdentityTestAccess::last(w, 0).sequence == 1);
        const auto s = snapshot(w, 12);
        assert(!s.valid && s.depth.total_frames == 1 && s.diagnostics.invalid == 1);
    }
    pass("full_last_identity_replay_conflicting_fields_and_old_sequence_without_history_set");
}
void fresh_order_and_rejected_identity() {
    for (unsigned n = 0; n < 7; ++n) {
        HealthWindow w(0, 8); const auto first = source(10, 10500000);
        assert(observe(w, false, first, 20, 20) == HealthEventCode::Accepted);
        auto next = source(11, 10500001); Millis event = 20;
        switch (n) {
            case 0: next.depth_index = first.depth_index; break;
            case 1: next.depth_index = first.depth_index - 1; break;
            case 2: next.depth_device_us = first.depth_device_us - 1; break;
            case 3: next.host_ready_mono_ns = first.host_ready_mono_ns - 1; break;
            case 4: event = 19; break;
            case 5: event = 22; break; // Future event at ingest 21.
            default: next.host_ready_mono_ns = 22000000; break; // True capture after event.
        }
        assert(observe(w, false, next, event, 21) == HealthEventCode::Invalid);
        assert(HealthSourceIdentityTestAccess::last(w, 0).sequence == 10);
        // The failed higher identity must not poison the following valid source.
        next = source(11, 10500001);
        assert(observe(w, false, next, 21, 21) == HealthEventCode::Accepted);
        const auto s = snapshot(w, 21);
        assert(!s.valid && s.depth.total_frames == 2 && s.diagnostics.invalid == 1);
    }
    HealthWindow w(0, 8); auto a = source(1);
    assert(observe(w, true, a, 15, 15) == HealthEventCode::Accepted);
    auto skip = source(99); skip.depth_device_us = a.depth_device_us;
    assert(observe(w, true, skip, 15, 15) == HealthEventCode::Accepted);
    assert(snapshot(w, 15).valid); // Same ns/us with increasing indices and sequence gap.
    pass("seven_fresh_order_faults_keep_last_identity_and_sequence_gaps_are_valid");
}
void stream_change_and_independent_streams() {
    HealthWindow w(0, 12);
    auto a = source(99); assert(observe(w, false, a, 20, 20) == HealthEventCode::Accepted);
    auto b = source(1); b.stream_id = 2; b.depth_device_us = 1;
    assert(observe(w, false, b, 20, 20) == HealthEventCode::Accepted);
    b.stream_id = 100; assert(observe(w, false, b, 20, 20) == HealthEventCode::Accepted);
    assert(observe(w, true, source(), 10, 20) == HealthEventCode::Accepted); // The inference stream is independent.
    auto c = source(1); c.stream_id = 4; --c.host_ready_mono_ns;
    assert(observe(w, false, c, 20, 20) == HealthEventCode::Invalid);
    assert(HealthSourceIdentityTestAccess::last(w, 0).stream_id == 100);
    c.host_ready_mono_ns = 10000001;
    assert(observe(w, false, c, 20, 20) == HealthEventCode::Accepted);
    const auto s = snapshot(w, 20);
    assert(!s.valid && s.depth.total_frames == 4 && s.inference.total_frames == 1);
    pass("stream_ids_not_numerically_ordered_host_clock_still_monotonic_streams_independent");
}
void basic_source_and_time_faults() {
    for (bool inference : {false, true}) {
        for (unsigned n = 0; n < 9; ++n) {
            HealthWindow w(5, 8); auto bad = source(); Millis event = 12, ingest = 12;
            switch (n) {
                case 0: bad.stream_id = 0; break;
                case 1: bad.sequence = 0; break;
                case 2: bad.depth_device_us = 0; break;
                case 3: bad.host_ready_mono_ns = 0; break;
                case 4: bad.host_ready_mono_ns = -1; break;
                case 5: event = 4; break;
                case 6: ingest = -1; break;
                case 7: ingest = std::numeric_limits<Millis>::max(); break;
                default: event = 9; break;
            }
            assert(observe(w, inference, bad, event, ingest) == HealthEventCode::Invalid);
            assert(observe(w, inference, source(), 12, 12) == HealthEventCode::Accepted);
            const auto s = snapshot(w, 12);
            assert(!s.valid && (inference ? s.inference : s.depth).total_frames == 1);
        }
    }
    HealthWindow w(0, 8); auto raw = source(); raw.color_index = 0; raw.color_device_us = 0;
    assert(observe(w, false, raw, 10, 10) == HealthEventCode::Accepted);
    assert(observe(w, true, raw, 10, 10) == HealthEventCode::Invalid);
    assert(observe(w, true, source(), 10, 10) == HealthEventCode::Accepted);
    // Positive sub-ms capture floors to 0 but remains a valid real source.
    HealthWindow sub(0, 2); auto tiny = source(1, 1);
    assert(observe(sub, false, tiny, 0, 0) == HealthEventCode::Accepted);
    assert(HealthSourceIdentityTestAccess::capture(sub, 0, 0) == 0);
    pass("eighteen_source_time_negatives_raw_depth_without_rgb_and_true_submillisecond_capture");
}
void window_expiry_and_global_clock() {
    HealthWindow w(0, 8); const auto a = source(), b = source(2);
    assert(observe(w, false, a, 12, 20) == HealthEventCode::Accepted);
    assert(observe(w, false, b, 12, 20) == HealthEventCode::Accepted);
    assert(observe(w, true, a, 13, 20) == HealthEventCode::Accepted);
    auto s = snapshot(w, 10011);
    assert(s.valid && s.full_window && s.depth.window_frames == 2 && s.inference.window_frames == 1);
    s = snapshot(w, 10012);
    assert(s.valid && s.depth.window_frames == 0 && s.depth.total_frames == 2 && s.depth.ever_observed);
    assert(s.inference.window_frames == 1 && s.inference.fps == .1);
    assert(observe(w, false, b, 12, 10012) == HealthEventCode::Duplicate);
    assert(observe(w, false, source(3), 12, 10012) == HealthEventCode::Invalid); // Expired new event cannot revive.
    assert(HealthSourceIdentityTestAccess::last(w, 0).sequence == 2);
    assert(observe(w, false, source(3, 10012000000), 10012, 10012) == HealthEventCode::Accepted);
    s = snapshot(w, 10013); assert(s.depth.window_frames == 1 && s.depth.total_frames == 3);
    assert(s.inference.window_frames == 0 && s.inference.total_frames == 1);
    // Snapshot and both native streams still share one ingest clock.
    assert(observe(w, true, source(2), 10013, 10012) == HealthEventCode::Invalid);
    assert(HealthSourceIdentityTestAccess::last(w, 1).sequence == 1);
    assert(observe(w, true, source(2), 10013, 10013) == HealthEventCode::Accepted);
    s = snapshot(w, 10012); assert(!s.valid && s.depth.window_frames == 0 && s.inference.window_frames == 0);
    pass("open_left_window_exact_totals_expired_duplicate_and_unified_ingest_snapshot_clock");
}
void capacity_and_lifetime_overflow() {
    HealthWindow w(0, 1);
    assert(observe(w, false, source(), 12, 12) == HealthEventCode::Accepted);
    assert(observe(w, false, source(2), 12, 12) == HealthEventCode::CapacityExceeded);
    assert(HealthSourceIdentityTestAccess::last(w, 0).sequence == 1);
    assert(observe(w, false, source(2, 10012000000), 10012, 10012) == HealthEventCode::Accepted);
    auto s = snapshot(w, 10012);
    assert(!s.valid && s.depth.total_frames == 2 && s.depth.window_frames == 1 && s.diagnostics.capacity_exceeded == 1);
    HealthWindow max(0, 2); constexpr auto limit = std::numeric_limits<std::uint64_t>::max();
    assert(observe(max, true, source(), 12, 12) == HealthEventCode::Accepted);
    HealthSourceIdentityTestAccess::total(max, 1, limit);
    assert(observe(max, true, source(2), 12, 12) == HealthEventCode::Invalid);
    s = snapshot(max, 12);
    assert(!s.valid && s.inference.total_frames == limit && s.inference.window_frames == 1);
    assert(HealthSourceIdentityTestAccess::last(max, 1).sequence == 1);
    pass("bounded_capacity_and_native_lifetime_saturation_cannot_replace_last_identity");
}
void publication_beside_native_rates_and_allocation() {
    HealthWindow native(0, 8);
    assert(observe(native, false, source(), 12, 12) == HealthEventCode::Accepted);
    assert(observe(native, false, source(2), 12, 12) == HealthEventCode::Accepted);
    no_allocation([&] {
        assert(native.observe_publication_attempt(Topic::Profile, 1, 10, 13, 14) == HealthEventCode::Accepted);
        assert(native.resolve_publication(Topic::Profile, audit(10, 13, 1), 14) == HealthEventCode::Accepted);
        // Same capture again: a replayed put is Duplicate even under a new seq.
        assert(native.observe_publication_attempt(Topic::Profile, 2, 10, 14, 15) == HealthEventCode::Duplicate);
        assert(native.observe_publication_attempt(Topic::Objects, 123, 10, 15, 15) == HealthEventCode::Accepted);
        assert(native.resolve_publication(Topic::Objects, audit(10, 15, 123), 16) == HealthEventCode::Accepted);
        return true;
    });
    const auto n = snapshot(native, 16);
    assert(n.valid && n.depth.total_frames == 2 && n.diagnostics.duplicates == 1);
    assert(n.profile.total.attempted == 1 && n.profile.total.accepted == 1 && n.profile.latency_p99_ms == 3);
    assert(n.objects.total.accepted == 1 && n.objects.latency_p99_ms == 5);
    HealthWindow loop(0, 32);
    no_allocation([&] {
        for (std::uint64_t i = 1; i <= 2000; ++i) {
            const auto event = static_cast<Millis>(i * 1000);
            const auto s = source(i, event * 1000000);
            assert(loop.observe_depth(s, event, event) == HealthEventCode::Accepted);
            assert(loop.observe_inference(s, event, event) == HealthEventCode::Accepted);
            assert(loop.observe_depth(s, event, event) == HealthEventCode::Duplicate);
            const auto out = loop.snapshot(event);
            assert(out.valid && out.depth.total_frames == i && out.inference.total_frames == i);
            assert(out.depth.window_frames <= 10 && out.inference.window_frames <= 10);
        }
        return true;
    });
    pass("split_publication_beside_native_rates_and_2000_native_rate_snapshots_zero_heap_allocations");
}
}
int main() {
    same_millisecond_and_nanosecond(); exact_replay_and_conflict(); fresh_order_and_rejected_identity();
    stream_change_and_independent_streams(); basic_source_and_time_faults();
    window_expiry_and_global_clock(); capacity_and_lifetime_overflow(); publication_beside_native_rates_and_allocation();
    std::cout << "Health SourceId PASS: " << groups << " groups; no devices/network.\n";
}
