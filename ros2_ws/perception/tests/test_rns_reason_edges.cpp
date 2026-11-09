/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_reason_edges.cpp
 * Brief: Bounded local reason-edge tests; no GEN, network or Status fabrication.
 *
 * Description:
 * What it checks. Reason edges turn degraded-reason bits into add/remove events. Cases cover the
 * initial state with no replay, a mixed single entry, independent directions at the exact
 * boundary, independent causes and severity, overflow that preserves Status and older entries,
 * invalid clocks with atomic bit updates, all 64 bits, limits and configuration, counter
 * saturation, and allocation and ring stress.
 */

#include "output/rns_reason_edges.hpp"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>

#ifdef NDEBUG
#error "Reason edge tests require enabled assertions"
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
void* operator new(std::size_t n, std::align_val_t a) {
    ++aligned_news; void* p = nullptr;
    if (posix_memalign(&p, static_cast<std::size_t>(a), n ? n : 1) == 0) return p;
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
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept { return ::operator new(n, a, std::nothrow); }
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }

namespace perception::rns {
struct ReasonEdgesTestAccess {
    static void counters(ReasonEdges& edges, const ReasonEdgeCounters& counters) { edges.counters_ = counters; }
};
}  // namespace perception::rns
using namespace perception::rns;
namespace {
std::size_t groups = 0, updates = 0, pops = 0;
template <class F> auto no_allocation(F fn) {
    const auto ordinary = ordinary_news.load(), aligned = aligned_news.load();
    const auto result = fn();
    assert(ordinary_news == ordinary && aligned_news == aligned);
    return result;
}
ReasonEdgeUpdate update(ReasonEdges& edges, std::uint64_t active, std::int64_t now) {
    ++updates; return no_allocation([&] { return edges.update(active, now); });
}
bool pop(ReasonEdges& edges, ReasonEdge& out) {
    ++pops; return no_allocation([&] { return edges.pop(out); });
}
ReasonEdge take(ReasonEdges& edges) {
    ReasonEdge edge; assert(pop(edges, edge)); return edge;
}
void empty(ReasonEdges& edges) {
    ReasonEdge edge;
    edge.raw_added = 999; edge.active_after = 999; edge.observed_ms = 999;
    assert(!pop(edges, edge));
    assert(edge.raw_added == 0 && edge.raw_removed == 0 && edge.eligible_added == 0 && edge.eligible_removed == 0);
    assert(edge.suppressed_added == 0 && edge.suppressed_removed == 0 && edge.active_after == 0 && edge.observed_ms == -1);
}
void pass(const char* name) { ++groups; std::cout << "PASS " << name << '\n'; }
void initial_unchanged_and_no_replay() {
    ReasonEdges edges(50, 4, 8);
    assert(edges.valid() && edges.active() == 0 && edges.queued() == 0 && edges.capacity() == 8);
    assert(update(edges, 0, 50).code == ReasonEdgeCode::Unchanged); empty(edges);
    const auto added = update(edges, 1, 50);
    assert(added.code == ReasonEdgeCode::Accepted && added.added == 1 && added.removed == 0);
    assert(added.enqueued_edges == 1 && added.lost_edges == 0 && added.active_after == 1);
    assert(update(edges, 1, 100000).code == ReasonEdgeCode::Unchanged && edges.queued() == 1);
    const auto edge = take(edges);
    assert(edge.severity == ReasonEdgeSeverity::Warn && edge.raw_added == 1 && edge.eligible_added == 1);
    assert(edge.raw_removed == 0 && edge.eligible_removed == 0 && edge.active_after == 1 && edge.observed_ms == 50);
    empty(edges); empty(edges);
    assert(update(edges, 1, 110000).code == ReasonEdgeCode::Unchanged); empty(edges);
    assert(update(edges, 0, 110000).enqueued_edges == 1);
    const auto recovered = take(edges);
    assert(recovered.severity == ReasonEdgeSeverity::Info && recovered.eligible_removed == 1 && recovered.active_after == 0);
    assert(edges.valid() && edges.counters().enqueued_edges == 2 && edges.counters().popped_edges == 2);
    pass("startup_unchanged_sets_and_pop_never_periodically_replay");
}
void mixed_one_entry() {
    ReasonEdges edges(0, 4, 4);
    assert(update(edges, 3, 0).enqueued_edges == 1);
    const auto mixed = update(edges, 12, 1);
    assert(mixed.added == 12 && mixed.removed == 3 && mixed.eligible_added == 12 && mixed.eligible_removed == 3);
    assert(mixed.enqueued_edges == 1 && edges.queued() == 2); // Never split a changed-set observation.
    const auto first = take(edges);
    assert(first.raw_added == 3 && first.active_after == 3 && first.observed_ms == 0);
    const auto edge = take(edges);
    assert(edge.severity == ReasonEdgeSeverity::Warn && edge.raw_added == 12 && edge.raw_removed == 3);
    assert(edge.eligible_added == 12 && edge.eligible_removed == 3 && edge.active_after == 12 && edge.observed_ms == 1);
    assert(edge.suppressed_added == 0 && edge.suppressed_removed == 0);
    assert(update(edges, 0, 2).enqueued_edges == 1);
    const auto info = take(edges);
    assert(info.severity == ReasonEdgeSeverity::Info && info.raw_removed == 12 && info.eligible_removed == 12);
    assert(edges.counters().added_bits == 4 && edges.counters().removed_bits == 4);
    empty(edges); pass("one_mixed_edge_keeps_both_directions_and_full_active_set");
}
void independent_directions_exact_boundary() {
    ReasonEdges edges(0, 1, 8);
    assert(update(edges, 1, 0).enqueued_edges == 1); // Warn anchor 0.
    assert(update(edges, 0, 1).enqueued_edges == 1); // Info anchor 1, not suppressed by Warn.
    auto r = update(edges, 1, 2);
    assert(r.enqueued_edges == 0 && r.suppressed_added == 1 && edges.active() == 1);
    r = update(edges, 0, 3);
    assert(r.enqueued_edges == 0 && r.suppressed_removed == 1 && edges.active() == 0);
    assert(update(edges, 1, 9999).suppressed_added == 1);
    assert(update(edges, 1, 10000).code == ReasonEdgeCode::Unchanged); // No deferred edge when deadline passes.
    assert(update(edges, 0, 10000).suppressed_removed == 1); // Info elapsed 9999.
    assert(update(edges, 1, 10000).enqueued_edges == 1); // Warn elapsed EXACT 10000.
    assert(update(edges, 0, 10001).enqueued_edges == 1); // Info elapsed EXACT 10000.
    assert(edges.queued() == 4);
    const std::int64_t expected_times[] = {0, 1, 10000, 10001};
    for (std::size_t i = 0; i < 4; ++i) {
        const auto edge = take(edges);
        assert(edge.observed_ms == expected_times[i]);
        assert(edge.severity == (i % 2 ? ReasonEdgeSeverity::Info : ReasonEdgeSeverity::Warn));
    }
    const auto c = edges.counters();
    assert(c.suppressed_added_bits == 2 && c.suppressed_removed_bits == 2 && c.enqueued_edges == 4);
    assert(c.added_bits == 4 && c.removed_bits == 4 && c.lost_edges == 0 && edges.valid());
    empty(edges); pass("warn_info_warn_independent_9999_10000_no_sliding_suppression_anchor");
}
void independent_causes_and_severity() {
    ReasonEdges edges(0, 2, 8);
    assert(update(edges, 1, 0).enqueued_edges == 1);
    assert(update(edges, 0, 1).enqueued_edges == 1);
    assert(update(edges, 2, 2).enqueued_edges == 1); // Cause 1 is independent of cause 0's Warn.
    const auto mixed = update(edges, 1, 3);
    assert(mixed.added == 1 && mixed.removed == 2 && mixed.suppressed_added == 1 && mixed.suppressed_removed == 0);
    assert(mixed.eligible_added == 0 && mixed.eligible_removed == 2 && mixed.enqueued_edges == 1);
    (void)take(edges); (void)take(edges); (void)take(edges);
    const auto edge = take(edges);
    assert(edge.severity == ReasonEdgeSeverity::Info); // Raw suppressed addition cannot upgrade recovery to Warn.
    assert(edge.raw_added == 1 && edge.raw_removed == 2 && edge.suppressed_added == 1 && edge.eligible_removed == 2);
    assert(edge.active_after == 1);
    assert(update(edges, 3, 4).suppressed_added == 2);
    const auto suppressed = update(edges, 0, 5);
    assert(suppressed.suppressed_removed == 3 && suppressed.enqueued_edges == 0);
    assert(update(edges, 3, 10002).enqueued_edges == 1);
    const auto both = take(edges);
    assert(both.severity == ReasonEdgeSeverity::Warn && both.eligible_added == 3);
    assert(edges.valid()); pass("different_causes_independent_and_mixed_eligible_severity");
}
void overflow_preserves_status_and_old_entries() {
    ReasonEdges edges(0, 2, 1);
    assert(update(edges, 1, 0).enqueued_edges == 1);
    const auto failed = update(edges, 2, 1);
    assert(failed.code == ReasonEdgeCode::Overflow && failed.enqueued_edges == 0 && failed.lost_edges == 1);
    assert(failed.eligible_added == 2 && failed.eligible_removed == 1 && failed.active_after == 2);
    assert(!edges.valid() && edges.active() == 2 && edges.queued() == 1);
    assert(update(edges, 2, 1).code == ReasonEdgeCode::Unchanged && edges.counters().lost_edges == 1);
    const auto old = take(edges);
    assert(old.observed_ms == 0 && old.raw_added == 1 && old.raw_removed == 0 && old.active_after == 1);
    assert(!edges.valid()); empty(edges);
    assert(update(edges, 0, 2).enqueued_edges == 1); (void)take(edges);
    assert(update(edges, 2, 3).enqueued_edges == 1); // Lost Warn(cause1) at 1 never advanced its anchor.
    const auto retry_transition = take(edges);
    assert(retry_transition.eligible_added == 2 && retry_transition.observed_ms == 3);
    const auto suppressed = update(edges, 1, 4);
    assert(suppressed.suppressed_added == 1 && suppressed.suppressed_removed == 2 && suppressed.enqueued_edges == 0);
    assert(update(edges, 0, 5).enqueued_edges == 1); // Lost Info(cause0) at 1 never advanced its anchor.
    assert(take(edges).eligible_removed == 1);
    const auto c = edges.counters();
    assert(c.lost_edges == 1 && c.lost_bits == 2 && c.overflow_updates == 1);
    assert(c.enqueued_edges == 4 && c.popped_edges == 4 && !edges.valid());
    empty(edges); assert(update(edges, 0, 100000).code == ReasonEdgeCode::Unchanged); empty(edges);
    pass("overflow_atomic_single_entry_no_overwrite_status_still_updates_no_retry");
}
void invalid_clock_and_bits_atomic() {
    ReasonEdges edges(100, 2, 8);
    assert(update(edges, 1, 100).enqueued_edges == 1);
    assert(update(edges, 2, 99).code == ReasonEdgeCode::Invalid);
    assert(update(edges, 8, 1000).code == ReasonEdgeCode::Invalid);
    assert(edges.active() == 1 && edges.queued() == 1 && !edges.valid());
    assert(update(edges, 3, 101).enqueued_edges == 1); // Invalid high timestamp did not mutate accepted clock.
    assert(update(edges, 0, -1).code == ReasonEdgeCode::Invalid);
    assert(update(edges, 0, std::numeric_limits<std::int64_t>::max()).code == ReasonEdgeCode::Invalid);
    assert(update(edges, 3, 102).code == ReasonEdgeCode::Unchanged);
    assert(update(edges, 0, 101).code == ReasonEdgeCode::Invalid); // Even unchanged inputs advance accepted clock.
    assert(edges.active() == 3 && edges.queued() == 2 && edges.counters().invalid_updates == 5);
    assert(take(edges).active_after == 1 && take(edges).active_after == 3);
    empty(edges); assert(!edges.valid());
    pass("invalid_time_unknown_bits_and_rollback_leave_state_queue_anchors_unchanged");
}
void all64_bits() {
    constexpr auto all = std::numeric_limits<std::uint64_t>::max();
    constexpr auto high = std::uint64_t{1} << 63;
    ReasonEdges edges(0, 64, 4);
    assert(update(edges, all, 0).eligible_added == all);
    const auto all_edge = take(edges);
    assert(all_edge.raw_added == all && all_edge.active_after == all);
    assert(update(edges, high, 1).eligible_removed == (all ^ high));
    assert(take(edges).raw_removed == (all ^ high));
    assert(update(edges, 0, 2).eligible_removed == high);
    assert(take(edges).eligible_removed == high);
    assert(update(edges, high, 3).suppressed_added == high);
    assert(edges.valid() && edges.counters().added_bits == 65 && edges.counters().removed_bits == 64);
    assert(edges.counters().suppressed_added_bits == 1);
    ReasonEdges only63(0, 63, 1);
    assert(update(only63, high, 0).code == ReasonEdgeCode::Invalid && only63.active() == 0);
    ReasonEdges only1(0, 1, 1);
    assert(update(only1, 2, 0).code == ReasonEdgeCode::Invalid);
    pass("bit63_and_full_uint64_without_shift64_unknown_bit_rejection");
}
template <class F> void rejects_constructor(F fn) {
    bool threw = false; try { fn(); } catch (const std::invalid_argument&) { threw = true; } assert(threw);
}
void limits_and_configuration() {
    rejects_constructor([] { ReasonEdges e(-1, 1, 1); });
    rejects_constructor([] { ReasonEdges e(std::numeric_limits<std::int64_t>::max(), 1, 1); });
    rejects_constructor([] { ReasonEdges e(0, 0, 1); });
    rejects_constructor([] { ReasonEdges e(0, 65, 1); });
    rejects_constructor([] { ReasonEdges e(0, std::numeric_limits<std::size_t>::max(), 1); });
    rejects_constructor([] { ReasonEdges e(0, 1, 0); });
    rejects_constructor([] { ReasonEdges e(0, 1, ReasonEdges::kMaxQueueCapacity + 1); });
    rejects_constructor([] { ReasonEdges e(0, 1, std::numeric_limits<std::size_t>::max()); });
    constexpr auto max = ProcessEnvelope::kMaxMonoMs;
    ReasonEdges near(max - 10000, 1, 4);
    assert(update(near, 1, max - 10000).enqueued_edges == 1);
    assert(update(near, 0, max - 9999).enqueued_edges == 1);
    assert(update(near, 1, max - 1).suppressed_added == 1);
    assert(update(near, 0, max - 1).suppressed_removed == 1);
    assert(update(near, 1, max).enqueued_edges == 1);
    assert(update(near, 0, max).suppressed_removed == 1);
    assert(near.valid());
    ReasonEdges end(max, 1, 2);
    assert(update(end, 1, max).enqueued_edges == 1);
    assert(update(end, 0, max).enqueued_edges == 1);
    assert(update(end, 1, max).suppressed_added == 1 && end.valid());
    ReasonEdges maximum_storage(0, 64, ReasonEdges::kMaxQueueCapacity);
    assert(maximum_storage.capacity() == ReasonEdges::kMaxQueueCapacity && maximum_storage.valid());
    pass("constructor_storage_limits_int64_max_exact_dedupe_without_add_overflow");
}
void counter_saturation() {
    ReasonEdges edges(0, 2, 1);
    constexpr auto max = std::numeric_limits<std::uint64_t>::max();
    ReasonEdgeCounters counters;
    counters.added_bits = counters.invalid_updates = counters.lost_edges = counters.lost_bits = counters.overflow_updates = max - 1;
    ReasonEdgesTestAccess::counters(edges, counters);
    assert(update(edges, 3, 0).enqueued_edges == 1); // +2 saturates raw bit count, never wraps.
    assert(update(edges, 1, 1).code == ReasonEdgeCode::Overflow);
    assert(update(edges, 0, 2).code == ReasonEdgeCode::Overflow);
    assert(update(edges, 8, 3).code == ReasonEdgeCode::Invalid);
    assert(update(edges, 8, 4).code == ReasonEdgeCode::Invalid);
    counters = edges.counters();
    assert(counters.added_bits == max && counters.invalid_updates == max && counters.lost_edges == max);
    assert(counters.lost_bits == max && counters.overflow_updates == max && counters.saturated && !edges.valid());
    assert(edges.active() == 0 && edges.queued() == 1);
    assert(take(edges).raw_added == 3); empty(edges); assert(!edges.valid());
    pass("failure_and_raw_bit_counters_saturate_never_wrap");
}
void allocation_and_ring_stress() {
    const auto ordinary = ordinary_news.load(), aligned = aligned_news.load();
    auto* a = ::operator new(1); ::operator delete(a);
    auto* b = ::operator new(64, std::align_val_t{64}); ::operator delete(b, std::align_val_t{64});
    assert(ordinary_news == ordinary + 1 && aligned_news == aligned + 1);
    ReasonEdges edges(0, 1, 3);
    for (std::int64_t i = 0; i < 5000; ++i) {
        const auto active = i % 2 ? std::uint64_t{0} : std::uint64_t{1};
        const auto result = update(edges, active, i * 10000);
        assert(result.code == ReasonEdgeCode::Accepted && result.enqueued_edges == 1);
        if (i >= 2) {
            const auto edge = take(edges);
            assert(edge.observed_ms == (i - 2) * 10000);
            assert(edge.active_after == ((i - 2) % 2 ? 0U : 1U));
        }
        assert(edges.valid() && edges.queued() <= 3);
    }
    assert(take(edges).observed_ms == 4998 * 10000);
    assert(take(edges).observed_ms == 4999 * 10000);
    empty(edges);
    const auto c = edges.counters();
    assert(c.enqueued_edges == 5000 && c.popped_edges == 5000 && c.lost_edges == 0 && c.invalid_updates == 0);
    assert(c.suppressed_added_bits == 0 && c.suppressed_removed_bits == 0 && edges.valid());
    pass("bounded_ring_wraparound_and_ordinary_aligned_new_zero_hotpath_allocation");
}
}  // namespace
int main() {
    initial_unchanged_and_no_replay(); mixed_one_entry(); independent_directions_exact_boundary(); independent_causes_and_severity();
    overflow_preserves_status_and_old_entries(); invalid_clock_and_bits_atomic(); all64_bits(); limits_and_configuration();
    counter_saturation(); allocation_and_ring_stress();
    std::cout << "rns_reason_edges: " << groups << " groups PASS; " << updates << " updates, " << pops
              << " pops, ordinary/aligned hot-path allocations = 0/0\n";
}
