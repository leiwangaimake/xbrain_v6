/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_health_window.cpp
 * Brief: Synthetic host health windows with exact clocks; no devices or network.
 *
 * Description:
 * What it checks. The health window feeds the Status rates and percentiles. Cases cover the empty
 * startup and its boundary, independent delayed streams, duplicates and out-of-order input,
 * timestamp faults, publication receipts, exact nearest-rank percentiles, capacity pruning and
 * expiry, the publication window without warm-up exclusion, numeric limits and configuration,
 * allocation and wraparound stress, and gaps from actual publications.
 * How it feeds. Through the service's own interfaces (SourceId rates, split attempt/resolve
 * puts) via rns_health_feed.hpp; invalid split audits are covered by test_rns_health_pending.cpp.
 */

#include "output/rns_health_window.hpp"
#include "rns_health_feed.hpp"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>

#ifdef NDEBUG
#error "Health window tests require enabled assertions"
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

using namespace perception::rns;
namespace {
std::size_t groups = 0, event_calls = 0, snapshot_calls = 0;
template <class F> auto no_allocation(F fn) {
    const auto ordinary = ordinary_news.load(), aligned = aligned_news.load();
    const auto result = fn();
    assert(ordinary_news == ordinary && aligned_news == aligned);
    return result;
}
HealthEventCode depth(HealthWindow& w, Millis capture, Millis event, Millis ingest) {
    ++event_calls; return no_allocation([&] { return test_feed::depth(w, capture, event, ingest); });
}
HealthEventCode infer(HealthWindow& w, Millis capture, Millis event, Millis ingest) {
    ++event_calls; return no_allocation([&] { return test_feed::inference(w, capture, event, ingest); });
}
HealthEventCode publish(HealthWindow& w, Topic topic, const PublishAudit& a, Millis ingest) {
    ++event_calls; return no_allocation([&] { return test_feed::publish(w, topic, a, ingest); });
}
HealthWindowReport snapshot(HealthWindow& w, Millis now) {
    ++snapshot_calls; return no_allocation([&] { return w.snapshot(now); });
}
PublishAudit audit(Millis capture, Millis before, Millis after, PutOutcome outcome = PutOutcome::Accepted) {
    PublishAudit a; // publish() issues the seq.
    a.attempted = true; a.clock_valid = true;
    a.capture_ms = capture; a.put_before_ms = before; a.put_return_ms = after;
    a.packet_bytes = 256; a.receipt = {outcome, outcome == PutOutcome::Rejected ? 17 : 0};
    return a;
}
void pass(const char* name) { ++groups; std::cout << "PASS " << name << '\n'; }
void assert_empty_publication(const HealthPublicationReport& p) {
    assert(p.window.attempted == 0 && p.window.accepted == 0 && p.window.rejected == 0 && p.window.unknown == 0);
    assert(!p.latency_p50_ms && !p.latency_p99_ms);
}
void startup_empty_and_boundary() {
    HealthWindow w(100, 8);
    auto s = snapshot(w, 100);
    assert(s.valid && !s.full_window && s.now_ms == 100);
    assert(!s.depth.ever_observed && !s.inference.ever_observed && !s.profile.ever_observed && !s.objects.ever_observed);
    assert(s.depth.window_frames == 0 && s.depth.fps == 0 && s.depth.total_frames == 0);
    assert_empty_publication(s.profile); assert_empty_publication(s.objects);
    assert(depth(w, 99, 100, 100) == HealthEventCode::Accepted);
    assert(infer(w, 99, 100, 100) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Profile, audit(99, 100, 100), 100) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Objects, audit(99, 100, 100), 100) == HealthEventCode::Accepted);
    s = snapshot(w, 100);
    assert(s.depth.fps == .1 && s.inference.fps == .1 && !s.full_window);
    assert(s.profile.latency_p50_ms == 1 && s.objects.latency_p99_ms == 1);
    s = snapshot(w, 10099);
    assert(s.valid && !s.full_window && s.depth.window_frames == 1 && s.profile.window.attempted == 1);
    s = snapshot(w, 10100); // Left boundary is OPEN; elapsed exactly 10000 is excluded.
    assert(s.valid && s.full_window && s.depth.window_frames == 0 && s.inference.window_frames == 0);
    assert(s.depth.ever_observed && s.inference.ever_observed && s.depth.total_frames == 1);
    assert(s.profile.ever_observed && s.objects.ever_observed && s.profile.total.attempted == 1);
    assert_empty_publication(s.profile); assert_empty_publication(s.objects);
    assert(depth(w, 10100, 10100, 10100) == HealthEventCode::Accepted); // Right boundary closed.
    assert(snapshot(w, 10100).depth.fps == .1);
    pass("startup_partial_fixed_denominator_and_open_left_closed_right");
}
void independent_delayed_streams() {
    HealthWindow w(0, 8);
    assert(depth(w, 10, 1000, 1100) == HealthEventCode::Accepted);
    assert(infer(w, 10, 500, 1101) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Profile, audit(20, 900, 1000), 1102) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Objects, audit(20, 300, 305), 1103) == HealthEventCode::Accepted);
    assert(infer(w, 11, 600, 1104) == HealthEventCode::Accepted);
    auto s = snapshot(w, 1104);
    assert(s.valid && s.depth.window_frames == 1 && s.inference.window_frames == 2);
    assert(s.depth.fps == .1 && s.inference.fps == .2);
    assert(s.profile.latency_p50_ms == 880 && s.objects.latency_p50_ms == 280);
    assert(s.profile.window.attempted == 1 && s.objects.window.attempted == 1);
    assert(depth(w, 11, 1000, 1104) == HealthEventCode::Accepted); // Equal event ms, distinct frame.
    assert(infer(w, 12, 599, 1105) == HealthEventCode::Invalid); // Fresh identity with actual per-stream rollback.
    s = snapshot(w, 1105);
    assert(!s.valid && s.inference.total_frames == 2 && s.diagnostics.invalid == 1);
    pass("independent_stream_delayed_handoff_and_per_stream_event_order");
}
void duplicate_and_out_of_order() {
    HealthWindow w(0, 8);
    assert(depth(w, 100, 200, 200) == HealthEventCode::Accepted);
    assert(depth(w, 100, 200, 201) == HealthEventCode::Duplicate);
    assert(depth(w, 99, 199, 202) == HealthEventCode::OutOfOrder);
    assert(depth(w, 100, 203, 203) == HealthEventCode::Duplicate); // New observation time cannot refresh source.
    assert(infer(w, 100, 204, 204) == HealthEventCode::Accepted); // Same source is valid in another stream.
    assert(publish(w, Topic::Profile, audit(100, 205, 205), 205) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Profile, audit(100, 206, 206), 206) == HealthEventCode::Duplicate);
    assert(publish(w, Topic::Profile, audit(99, 204, 205), 207) == HealthEventCode::OutOfOrder);
    auto s = snapshot(w, 207);
    assert(s.valid && s.depth.total_frames == 1 && s.profile.total.attempted == 1);
    assert(s.diagnostics.duplicates == 3 && s.diagnostics.out_of_order == 2 && s.diagnostics.invalid == 0);
    assert(depth(w, 101, 199, 208) == HealthEventCode::Invalid);
    assert(depth(w, 102, 209, 209) == HealthEventCode::Accepted);
    s = snapshot(w, 209);
    assert(!s.valid && s.depth.total_frames == 2 && s.diagnostics.invalid == 1); // Sticky; no implicit recovery.
    pass("per_stream_identity_dedup_outoforder_and_sticky_fault");
}
void timestamp_faults() {
    HealthWindow w(100, 8);
    assert(depth(w, 0, 100, 100) == HealthEventCode::Accepted);
    assert(snapshot(w, 200).valid);
    auto s = snapshot(w, 199);
    assert(!s.valid && !s.full_window && s.now_ms == 199);
    assert(s.depth.ever_observed && s.depth.total_frames == 1 && s.depth.window_frames == 0);
    assert(s.depth.fps == 0); assert_empty_publication(s.profile);
    assert(infer(w, 0, 150, 199) == HealthEventCode::Invalid); // Global ingest must not roll back after snapshot.
    assert(infer(w, 0, 150, 200) == HealthEventCode::Accepted); // Older cross-stream event is allowed.
    assert(depth(w, -1, 201, 201) == HealthEventCode::Invalid);
    assert(depth(w, 1, 99, 202) == HealthEventCode::Invalid); // Event before process startup.
    assert(depth(w, 1, 204, 203) == HealthEventCode::Invalid); // Future event.
    assert(depth(w, 300, 204, 204) == HealthEventCode::Invalid); // Future capture.
    assert(depth(w, 1, 205, -1) == HealthEventCode::Invalid);
    assert(depth(w, 1, 205, std::numeric_limits<Millis>::max()) == HealthEventCode::Invalid);
    s = snapshot(w, 205);
    assert(!s.valid && s.depth.total_frames == 1 && s.inference.total_frames == 1 && s.diagnostics.invalid == 8);
    pass("snapshot_and_ingest_epoch_rollback_invalid_times_no_recovery");
}
void publication_receipts() {
    HealthWindow w(0, 32);
    assert(publish(w, Topic::Objects, audit(0, 10, 12), 12) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Objects, audit(1, 20, 22, PutOutcome::Rejected), 22) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Objects, audit(2, 30, 32, PutOutcome::Unknown), 32) == HealthEventCode::Accepted);
    auto thrown = audit(3, 40, 42, PutOutcome::Unknown); thrown.threw = true;
    assert(publish(w, Topic::Objects, thrown, 42) == HealthEventCode::Accepted);
    auto s = snapshot(w, 42);
    assert(s.valid && s.objects.window.attempted == 4 && s.objects.total.attempted == 4);
    assert(s.objects.window.accepted == 1 && s.objects.window.rejected == 1 && s.objects.window.unknown == 2);
    assert(s.objects.latency_p50_ms == 19 && s.objects.latency_p99_ms == 37); // put-before, never return.
    s = snapshot(w, 100);
    assert(s.valid && s.objects.total.attempted == 4 && s.objects.window.attempted == 4 && !s.profile.ever_observed);
    s = snapshot(w, 10040);
    assert_empty_publication(s.objects);
    assert(s.objects.ever_observed && s.objects.total.accepted == 1 && s.objects.total.rejected == 1 && s.objects.total.unknown == 2);
    pass("attempt_receipt_separation_and_unknown_outcomes");
}
void exact_nearest_rank() {
    HealthWindow w(0, 128);
    for (Millis i = 0; i < 100; ++i) {
        const Millis capture = i * 100, latency = 100 - i;
        const auto outcome = i % 3 == 0 ? PutOutcome::Accepted : (i % 3 == 1 ? PutOutcome::Rejected : PutOutcome::Unknown);
        assert(publish(w, Topic::Profile, audit(capture, capture + latency, capture + latency, outcome), capture + latency)
               == HealthEventCode::Accepted);
    }
    for (Millis i = 0; i < 100; ++i) {
        const Millis capture = 100 + i * 90, latency = 2 * (100 - i);
        assert(publish(w, Topic::Objects, audit(capture, capture + latency, capture + latency, PutOutcome::Unknown), 10000)
               == HealthEventCode::Accepted);
    }
    const auto s = snapshot(w, 10000);
    assert(s.valid && s.full_window);
    assert(s.profile.latency_p50_ms == 50 && s.profile.latency_p99_ms == 99);
    assert(s.objects.latency_p50_ms == 100 && s.objects.latency_p99_ms == 198);
    assert(s.profile.window.attempted == 100 && s.profile.window.accepted == 34 &&
           s.profile.window.rejected == 33 && s.profile.window.unknown == 33);
    assert(s.objects.window.unknown == 100 && s.objects.total.attempted == 100);
    assert(snapshot(w, 10100).profile.window.attempted == 99);
    HealthWindow even(0, 4);
    const Millis values[] = {5, 10, 25, 50};
    for (Millis i = 0; i < 4; ++i)
        assert(publish(even, Topic::Profile, audit(i * 100, i * 100 + values[i], i * 100 + values[i]), 1000)
               == HealthEventCode::Accepted);
    const auto e = snapshot(even, 1000);
    assert(e.profile.latency_p50_ms == 10 && e.profile.latency_p99_ms == 50); // No interpolation/round-to-nearest rank.
    assert_empty_publication(e.objects);
    pass("exact_nearest_rank_100_samples_even_small_n_and_stream_separation");
}
void capacity_pruning_and_expiry() {
    HealthWindow w(0, 2);
    assert(w.capacity_per_stream() == 2);
    assert(depth(w, 1, 1, 1) == HealthEventCode::Accepted);
    assert(depth(w, 2, 2, 2) == HealthEventCode::Accepted);
    assert(depth(w, 3, 3, 3) == HealthEventCode::CapacityExceeded);
    assert(infer(w, 3, 3, 3) == HealthEventCode::Accepted);
    auto s = snapshot(w, 3);
    assert(!s.valid && s.depth.window_frames == 2 && s.depth.total_frames == 2 && s.inference.window_frames == 1);
    assert(s.diagnostics.capacity_exceeded == 1);
    assert(depth(w, 4, 10001, 10001) == HealthEventCode::Accepted); // Prunes exact-left-bound event before capacity check.
    s = snapshot(w, 10001);
    assert(!s.valid && s.depth.window_frames == 2 && s.depth.total_frames == 3);
    assert(snapshot(w, 10002).depth.window_frames == 1);
    assert(depth(w, 5, 10000, 20000) == HealthEventCode::Invalid); // Already expired; never revived.
    assert(depth(w, 6, 19999, 20000) == HealthEventCode::Accepted);
    s = snapshot(w, 20000);
    assert(!s.valid && s.depth.window_frames == 2 && s.depth.total_frames == 4);
    HealthWindow old(0, 1);
    assert(depth(old, 1, 1, 10001) == HealthEventCode::Invalid);
    s = snapshot(old, 10001);
    assert(!s.valid && !s.depth.ever_observed && s.depth.total_frames == 0);
    HealthWindow replay(0, 1);
    assert(depth(replay, 1, 1, 1) == HealthEventCode::Accepted);
    assert(depth(replay, 1, 1, 10001) == HealthEventCode::Duplicate);
    s = snapshot(replay, 10001);
    assert(s.valid && s.depth.ever_observed && s.depth.window_frames == 0 && s.depth.total_frames == 1);
    pass("capacity_never_overwrites_prune_first_and_expired_never_revives");
}
void publication_window_and_no_warmup_exclusion() {
    HealthWindow w(0, 8);
    assert(publish(w, Topic::Objects, audit(0, 1, 2, PutOutcome::Rejected), 2) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Objects, audit(1, 2, 3, PutOutcome::Unknown), 3) == HealthEventCode::Accepted);
    assert(publish(w, Topic::Objects, audit(2, 3, 4), 4) == HealthEventCode::Accepted);
    auto s = snapshot(w, 4); // Counted during cadence's separate 30-second warmup.
    assert(s.valid && !s.full_window && s.objects.total.attempted == 3 && s.objects.window.attempted == 3);
    s = snapshot(w, 10001);
    assert(s.objects.window.attempted == 2 && s.objects.window.rejected == 0 && s.objects.window.unknown == 1);
    assert(s.objects.total.rejected == 1 && s.objects.total.unknown == 1 && s.objects.total.accepted == 1);
    assert(snapshot(w, 10002).objects.window.attempted == 1);
    s = snapshot(w, 10003);
    assert_empty_publication(s.objects);
    assert(s.objects.ever_observed && s.objects.total.attempted == 3);
    assert(!s.profile.ever_observed && s.profile.total.attempted == 0);
    pass("publication_left_edge_outcome_expiry_no_cadence_exemption");
}
template <class F> void invalid_constructor(F fn) {
    bool threw = false;
    try { fn(); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
}
void numeric_limits_and_configuration() {
    invalid_constructor([] { HealthWindow w(-1, 1); });
    invalid_constructor([] { HealthWindow w(std::numeric_limits<Millis>::max(), 1); });
    invalid_constructor([] { HealthWindow w(0, 0); });
    invalid_constructor([] { HealthWindow w(0, HealthWindow::kMaxCapacityPerStream + 1); });
    invalid_constructor([] { HealthWindow w(0, std::numeric_limits<std::size_t>::max()); });
    constexpr auto max = ProcessEnvelope::kMaxMonoMs;
    HealthWindow late(max, 1);
    // A SourceId cannot carry a host ready time of kMaxMonoMs ms; the event and ingest can.
    assert(depth(late, 0, max, max) == HealthEventCode::Accepted);
    assert(publish(late, Topic::Profile, audit(0, max, max), max) == HealthEventCode::Accepted);
    auto s = snapshot(late, max);
    assert(s.valid && !s.full_window && s.depth.fps == .1);
    assert(s.profile.latency_p50_ms == max && s.profile.latency_p99_ms == max);
    HealthWindow aged(0, 1);
    assert(depth(aged, 0, 0, 0) == HealthEventCode::Accepted);
    s = snapshot(aged, max);
    assert(s.valid && s.full_window && s.depth.window_frames == 0 && s.depth.total_frames == 1);
    assert(depth(aged, 1, max, max) == HealthEventCode::Accepted);
    assert(snapshot(aged, max).depth.window_frames == 1);
    pass("constructor_limits_and_int64_edges_without_addition_overflow");
}
void allocation_and_wraparound_stress() {
    const auto ordinary = ordinary_news.load(), aligned = aligned_news.load();
    auto* a = ::operator new(1); ::operator delete(a);
    auto* b = ::operator new(64, std::align_val_t{64}); ::operator delete(b, std::align_val_t{64});
    assert(ordinary_news == ordinary + 1 && aligned_news == aligned + 1);
    HealthWindow w(0, 100);
    for (Millis i = 0; i < 2000; ++i) {
        const Millis now = i * 100;
        assert(depth(w, now, now, now) == HealthEventCode::Accepted);
        assert(infer(w, now, now, now) == HealthEventCode::Accepted);
        const auto outcome = i % 2 ? PutOutcome::Rejected : PutOutcome::Unknown;
        const auto a = audit(now, now, now, outcome);
        assert(publish(w, Topic::Profile, a, now) == HealthEventCode::Accepted);
        assert(publish(w, Topic::Objects, a, now) == HealthEventCode::Accepted);
        const auto s = snapshot(w, now);
        const auto count = static_cast<std::size_t>(i < 99 ? i + 1 : 100);
        assert(s.valid && s.depth.window_frames == count && s.inference.window_frames == count);
        assert(s.depth.fps == static_cast<double>(count) / 10.0);
        assert(s.profile.window.attempted == count && s.objects.window.attempted == count);
        assert(s.profile.latency_p50_ms == 0 && s.objects.latency_p99_ms == 0);
        assert(s.profile.total.attempted == static_cast<std::uint64_t>(i + 1));
    }
    const auto s = snapshot(w, 209900);
    assert(s.valid && s.depth.window_frames == 0 && s.depth.total_frames == 2000);
    assert_empty_publication(s.profile); assert_empty_publication(s.objects);
    assert(s.objects.total.rejected == 1000 && s.objects.total.unknown == 1000 && s.objects.total.accepted == 0);
    pass("all_four_ring_wraparound_and_ordinary_aligned_hotpath_zero_allocation");
}
void actual_publication_gaps() {
    HealthWindow w{0, 4};
    auto s = snapshot(w, 30000);
    assert(!s.objects.ever_observed && !s.objects.gap_p99_ms && !s.objects.gap_max_ms && !s.objects.open_gap_ms);
    assert(publish(w, Topic::Objects, audit(30000, 30001, 30001), 30001) == HealthEventCode::Accepted);
    s = snapshot(w, 30010);
    assert(s.objects.gap_p99_ms == 9 && s.objects.gap_max_ms == 9 && s.objects.closed_gap_samples == 0);
    assert(publish(w, Topic::Objects, audit(30001, 30041, 30041), 30041) == HealthEventCode::Accepted);
    s = snapshot(w, 30050);
    assert(s.objects.closed_gap_samples == 1 && s.objects.gap_p99_ms == 40 && s.objects.gap_max_ms == 40);
    const auto again = snapshot(w, 30050);
    assert(again.objects.closed_gap_samples == 1 && again.objects.window.attempted == 2);
    s = snapshot(w, 40041);
    assert(s.objects.window.attempted == 0 && !s.objects.latency_p50_ms && !s.objects.latency_p99_ms);
    assert(s.objects.open_gap_ms == 10000 && s.objects.gap_max_ms == 10000 && s.objects.gap_p99_ms == 10000);
    assert(!s.profile.open_gap_ms); // No fake gap based on process startup.
    HealthWindow warm{0, 4};
    assert(publish(warm, Topic::Objects, audit(0, 1, 1), 1) == HealthEventCode::Accepted);
    assert(publish(warm, Topic::Objects, audit(1, 42, 42), 42) == HealthEventCode::Accepted);
    s = snapshot(warm, 100); // actual health samples survive acceptance warmup
    assert(s.objects.closed_gap_samples == 1 && s.objects.gap_max_ms == 58 && s.objects.gap_p99_ms == 58);
    pass("actual_unique_publish_gap_no_startup_fiction_no_open_resampling_expired_latency");
}
}  // namespace
int main() {
    startup_empty_and_boundary(); independent_delayed_streams(); duplicate_and_out_of_order(); timestamp_faults();
    publication_receipts(); exact_nearest_rank(); capacity_pruning_and_expiry();
    publication_window_and_no_warmup_exclusion(); numeric_limits_and_configuration(); allocation_and_wraparound_stress();
    actual_publication_gaps();
    std::cout << "rns_health_window: " << groups << " groups PASS; " << event_calls << " events, "
              << snapshot_calls << " snapshots, ordinary/aligned hot-path allocations = 0/0\n";
}
