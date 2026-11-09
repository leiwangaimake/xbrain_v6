/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_source_handoff.cpp
 * Brief: Five-source certified cuts with injected clocks and immutable synthetic observations.
 *
 * Description:
 * What it checks. The publication handoff certifies a cut across the five sources before any
 * mutation. Cases cover independent counts and windows, exact boundaries with no observations,
 * certification of all five before mutation (a source clock draining while record owns the odd
 * generation), partial milliseconds and clock bounds, late and expired entries with full identity,
 * invalid roles, overflow and evidence, the old mode without allocation, and a bounded consumer
 * audit.
 * Scope. No camera, SDK, network, production Status or cadence claim.
 */

#include "output/rns_publication_handoff.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>

static std::atomic<std::size_t> allocations{0};
void* operator new(std::size_t n) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    if (auto* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace perception::rns;
using perception::native_depth::SourceId;
namespace {
std::size_t checks = 0, groups = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __FILE__ << ':' << __LINE__ \
    << ": " #x "\n"; std::abort(); } } while (false)
constexpr std::int64_t ns(Millis ms) { return ms * 1000000; }
struct Clock : PublishClock {
    Millis now = 0;
    Millis mono_millis() noexcept override { return now; }
    Millis wall_millis() noexcept override { return 1700000000000LL; }
};
struct SourceTime {
    std::int64_t now = 0;
    PublicationHealthHandoff* nested = nullptr;
    PublishClock* clock = nullptr;
    HandoffCode nested_code = HandoffCode::Invalid;
    static std::int64_t sample(void* context) noexcept {
        auto& t = *static_cast<SourceTime*>(context);
        if (t.nested) t.nested_code = t.nested->drain(*t.clock).code;
        return t.now;
    }
    SourceClock callback() noexcept { return {this, sample}; }
};
SourceId source(std::uint64_t seq, std::int64_t ready_ns, std::uint64_t stream = 1) {
    SourceId s;
    s.stream_id = stream; s.sequence = seq; s.depth_index = seq - 1; s.color_index = seq - 1;
    s.depth_device_us = seq * 33333; s.color_device_us = seq * 33333;
    s.host_ready_mono_ns = ready_ns; return s;
}
DepthHealthObservation observation(SourceId s, std::size_t pixels = 10, std::size_t invalid = 5) {
    DepthHealthObservation d;
    d.code = DepthHealthCode::Ready; d.source = s;
    d.roi_kind = HealthRoiKind::LowerHalfUncalibrated;
    d.roi_pixels = pixels; d.invalid_pixels = invalid;
    d.invalid_ratio = static_cast<double>(invalid) / pixels; return d;
}
void publication(PublicationJournal& j, std::uint64_t seq, Millis capture, Millis at) {
    PublishAudit a; a.attempted = true; a.seq = seq; a.capture_ms = capture;
    a.put_before_ms = at; a.packet_bytes = 100;
    CHECK(j.reserve()); j.begin_attempt(); j.record_attempt(a);
    a.put_return_ms = at; a.clock_valid = true; a.receipt = {PutOutcome::Accepted, 0};
    j.begin_completion(); j.record_completion(a); CHECK(j.valid());
}
struct Rig {
    Clock clock;
    SourceTime at, it, dt;
    PublicationJournal p{Topic::Profile, 32}, o{Topic::Objects, 32};
    HealthWindow health{0, 32}; ObjectsCadence cadence{0, 32}; DepthHealthWindow depth{0, 32};
    SourceJournal arrival{SourceKind::DepthArrival, 32, at.callback()};
    SourceJournal inference{SourceKind::InferenceComplete, 32, it.callback()};
    SourceJournal processed{SourceKind::DepthProcessed, 32, dt.callback()};
    PublicationHealthHandoff h{0, p, o, health, cadence, arrival, inference, processed, depth};
};
PublicationCut drain(Rig& r, Millis now) { r.clock.now = now; return r.h.drain(r.clock); }

void independent_counts_and_windows() {
    Rig r;
    const auto s1 = source(1, ns(900)), s2 = source(2, ns(1000)), s3 = source(3, ns(1100));
    r.at.now = ns(1000); CHECK(r.arrival.record(s1));
    r.at.now = ns(1100); CHECK(r.arrival.record(s2));
    r.at.now = ns(1200); CHECK(r.arrival.record(s3));
    r.it.now = ns(1200); CHECK(r.inference.record(s1));
    r.dt.now = ns(1100); CHECK(r.processed.record(observation(s1, 10, 10)));
    r.dt.now = ns(1200); CHECK(r.processed.record(observation(s2, 30, 0)));
    publication(r.p, 1, 900, 1100); publication(r.o, 2, 900, 1200);
    const auto c = drain(r, 1201);
    CHECK(c.code == HandoffCode::Ready && c.observed_ms == 1200);
    CHECK((c.events == std::array<std::size_t, 2>{2, 2}));
    CHECK((c.source_events == std::array<std::size_t, 3>{3, 1, 2}));
    const auto w = r.health.snapshot(c.observed_ms); const auto d = r.depth.snapshot(c.observed_ms);
    CHECK(w.valid && d.valid && w.now_ms == d.now_ms);
    CHECK(w.depth.window_frames == 3 && w.inference.window_frames == 1);
    CHECK(w.depth.fps == .3 && w.inference.fps == .1);
    CHECK(w.profile.total.accepted == 1 && w.objects.total.accepted == 1);
    CHECK(d.processed_frames == 2 && d.roi_pixels == 40 && d.invalid_pixels == 10 && d.invalid_ratio == .25);
    CHECK(r.cadence.snapshot(c.observed_ms).excluded_results == 1); // Real warmup, not discarded health.
    ++groups;
}
void exact_boundaries_and_no_observations() {
    Rig r;
    const auto s = source(1, ns(1900));
    r.at.now = r.it.now = r.dt.now = ns(2000);
    CHECK(r.arrival.record(s)); CHECK(r.inference.record(s));
    CHECK(r.processed.record(observation(s, 20, 20)));
    publication(r.o, 1, 1900, 2000);
    CHECK(drain(r, 2001).code == HandoffCode::Ready);
    CHECK(r.depth.snapshot(2000).invalid_ratio == 1);
    CHECK(drain(r, 3000).observed_ms == 2999);
    CHECK(r.depth.snapshot(2999).processed_frames == 1);
    CHECK(drain(r, 3001).observed_ms == 3000);
    const auto empty = r.depth.snapshot(3000);
    CHECK(empty.valid && empty.ever_observed && !empty.invalid_ratio && !empty.latest_source);
    CHECK(empty.processed_frames == 0 && empty.roi_pixels == 0 && empty.invalid_pixels == 0);
    CHECK(r.health.snapshot(3000).depth.window_frames == 1);
    CHECK(drain(r, 12000).observed_ms == 11999);
    CHECK(r.health.snapshot(11999).inference.window_frames == 1);
    CHECK(drain(r, 12001).observed_ms == 12000);
    const auto w = r.health.snapshot(12000);
    CHECK(w.valid && w.depth.window_frames == 0 && w.depth.total_frames == 1 && w.depth.fps == 0);
    CHECK(w.inference.window_frames == 0 && !w.objects.latency_p99_ms && w.objects.open_gap_ms == 10000);
    Rig never; CHECK(drain(never, 1001).code == HandoffCode::Ready);
    const auto nd = never.depth.snapshot(1000); const auto nw = never.health.snapshot(1000);
    CHECK(nd.valid && !nd.ever_observed && !nd.invalid_ratio && !nw.objects.open_gap_ms);
    ++groups;
}
struct ChangingClock final : Clock {
    SourceJournal* journal = nullptr; SourceTime* time = nullptr; SourceId id;
    bool fire = true;
    Millis mono_millis() noexcept override {
        if (fire) { fire = false; CHECK(journal->record(id)); }
        return now;
    }
};
void all_five_certification_before_any_mutation() {
    // A source clock calls drain while record owns the odd generation. The
    // consumer must leave BOTH queued publications and every window untouched.
    for (int role = 0; role < 3; ++role) {
        Rig r; publication(r.o, 1, 30900, 31000); r.clock.now = 31002;
        SourceTime* times[] = {&r.at, &r.it, &r.dt};
        SourceJournal* journals[] = {&r.arrival, &r.inference, &r.processed};
        auto& t = *times[role]; t.now = ns(31001); t.nested = &r.h; t.clock = &r.clock;
        const auto s = source(1, ns(30900));
        CHECK(role == 2 ? journals[role]->record(observation(s)) : journals[role]->record(s));
        CHECK(t.nested_code == HandoffCode::Busy && r.h.last_cut() == -1);
        CHECK(r.o.consumed() == 0 && journals[role]->consumed() == 0);
        CHECK(r.health.snapshot(0).objects.total.attempted == 0);
        CHECK(r.cadence.snapshot(0).unique_results == 0);
        CHECK(r.depth.snapshot(0).processed_frames == 0);
        t.nested = nullptr;
        const auto c = r.h.drain(r.clock);
        CHECK(c.code == HandoffCode::Ready && c.events[1] == 2 && c.source_events[role] == 1);
    }
    // Even->even changes during the shared clock read must also reject the
    // whole cut, not certify publications first and discover a source later.
    Rig r; publication(r.p, 1, 90, 100); r.at.now = ns(100);
    ChangingClock clock; clock.now = 101; clock.journal = &r.arrival; clock.id = source(1, ns(90));
    CHECK(r.h.drain(clock).code == HandoffCode::Busy);
    CHECK(r.p.consumed() == 0 && r.arrival.consumed() == 0 && r.h.last_cut() == -1);
    CHECK(r.health.snapshot(0).profile.total.attempted == 0);
    CHECK(r.h.drain(clock).code == HandoffCode::Ready);
    CHECK(r.p.consumed() == 2 && r.arrival.consumed() == 1);
    ++groups;
}
void partial_millisecond_and_clock_bounds() {
    Rig r; r.at.now = ns(100) + 999999;
    CHECK(r.arrival.record(source(1, ns(99))));
    const auto first = drain(r, 100);
    CHECK(first.code == HandoffCode::Ready && first.observed_ms == 99 && first.source_events[0] == 0);
    CHECK(r.arrival.consumed() == 0 && r.h.valid());
    CHECK(drain(r, 101).source_events[0] == 1);
    CHECK(r.health.snapshot(100).depth.window_frames == 1);
    Rig future; future.at.now = ns(101);
    CHECK(future.arrival.record(source(1, ns(90))));
    CHECK(drain(future, 100).code == HandoffCode::Invalid && !future.h.valid());
    CHECK(future.arrival.consumed() == 0 && future.health.snapshot(99).depth.total_frames == 0);
    Rig large; const auto max = ProcessEnvelope::kMaxMonoMs;
    CHECK(drain(large, max).code == HandoffCode::Ready); // No now*1e6 overflow.
    Rig rollback; CHECK(drain(rollback, 100).code == HandoffCode::Ready);
    CHECK(drain(rollback, 99).code == HandoffCode::Invalid);
    ++groups;
}
void late_expired_and_full_identity() {
    for (int role = 0; role < 3; ++role) {
        Rig r; CHECK(drain(r, 1001).code == HandoffCode::Ready);
        SourceTime* times[] = {&r.at, &r.it, &r.dt};
        SourceJournal* journals[] = {&r.arrival, &r.inference, &r.processed};
        times[role]->now = ns(1000); const auto s = source(1, ns(900));
        CHECK(role == 2 ? journals[role]->record(observation(s)) : journals[role]->record(s));
        CHECK(drain(r, 1002).code == HandoffCode::Invalid && journals[role]->consumed() == 0);
    }
    for (int role = 0; role < 3; ++role) {
        Rig r; SourceTime* times[] = {&r.at, &r.it, &r.dt};
        SourceJournal* journals[] = {&r.arrival, &r.inference, &r.processed};
        times[role]->now = ns(1000); const auto s = source(1, ns(900));
        CHECK(role == 2 ? journals[role]->record(observation(s)) : journals[role]->record(s));
        CHECK(drain(r, role == 2 ? 2001 : 11001).code == HandoffCode::Invalid);
        CHECK(journals[role]->consumed() == 0 && !r.h.valid());
    }
    for (int role = 0; role < 2; ++role) {
        Rig r; auto& j = role ? r.inference : r.arrival; auto& t = role ? r.it : r.at;
        t.now = ns(100); CHECK(j.record(source(1, ns(90) + 1)));
        t.now = ns(101); CHECK(j.record(source(2, ns(90) + 999999)));
        t.now = ns(101); CHECK(j.record(source(3, ns(90) + 999999)));
        CHECK(j.valid()); // Distinct source identity even when host ns also matches.
        CHECK(drain(r, 102).code == HandoffCode::Ready && r.h.valid());
        const auto w = r.health.snapshot(101);
        CHECK(w.diagnostics.duplicates == 0 && j.consumed() == 3);
        CHECK((role ? w.inference.total_frames : w.depth.total_frames) == 3);
    }
    ++groups;
}
void invalid_roles_overflow_and_evidence() {
    Rig r;
    PublicationHealthHandoff wrong{0, r.p, r.o, r.health, r.cadence,
        r.inference, r.arrival, r.processed, r.depth};
    CHECK(!wrong.valid() && wrong.drain(r.clock).code == HandoffCode::Invalid);
    PublicationHealthHandoff alias{0, r.p, r.o, r.health, r.cadence,
        r.arrival, r.arrival, r.processed, r.depth};
    CHECK(!alias.valid());
    SourceJournal broken{SourceKind::DepthArrival, 0, r.at.callback()};
    PublicationHealthHandoff invalid{0, r.p, r.o, r.health, r.cadence,
        broken, r.inference, r.processed, r.depth}; CHECK(!invalid.valid());
    Rig full; SourceJournal one{SourceKind::DepthArrival, 1, full.at.callback()};
    PublicationHealthHandoff overflow{0, full.p, full.o, full.health, full.cadence,
        one, full.inference, full.processed, full.depth};
    publication(full.o, 1, 90, 100);
    full.at.now = ns(100); CHECK(one.record(source(1, ns(90))));
    full.at.now = ns(101); CHECK(!one.record(source(2, ns(91))) && one.overflow_count() == 1);
    full.clock.now = 102;
    CHECK(overflow.drain(full.clock).code == HandoffCode::Invalid && full.o.consumed() == 0);
    CHECK(one.consumed() == 0 && one.published() == 1); // Actual committed evidence retained.
    Rig bad; bad.dt.now = ns(100); auto d = observation(source(1, ns(90))); d.invalid_ratio = .9;
    CHECK(bad.processed.record(d)); // Journal transports, does not repair the evidence.
    CHECK(drain(bad, 101).code == HandoffCode::Invalid && bad.processed.consumed() == 0);
    CHECK(!bad.depth.snapshot(100).valid);
    ++groups;
}
void old_mode_and_allocation_free() {
    Rig r; PublicationHealthHandoff old{0, r.p, r.o, r.health, r.cadence};
    r.at.now = -1; CHECK(!r.arrival.record(source(1, ns(90))));
    publication(r.o, 1, 90, 100); r.clock.now = 101;
    const auto old_cut = old.drain(r.clock);
    CHECK(old_cut.code == HandoffCode::Ready && old_cut.events[1] == 2);
    CHECK((old_cut.source_events == std::array<std::size_t, 3>{0, 0, 0}));
    CHECK(r.health.snapshot(100).depth.total_frames == 0); // Opt-in, no implicit sources.
    Rig hot; const auto before = allocations.load();
    for (std::uint64_t i = 1; i <= 1000; ++i) {
        const Millis at = static_cast<Millis>(i) * 11000;
        const auto s = source(i, ns(at - 1));
        hot.at.now = hot.it.now = hot.dt.now = ns(at);
        CHECK(hot.arrival.record(s)); CHECK(hot.inference.record(s));
        CHECK(hot.processed.record(observation(s)));
        const auto c = drain(hot, at + 1); CHECK(c.code == HandoffCode::Ready);
        const auto w = hot.health.snapshot(c.observed_ms); const auto d = hot.depth.snapshot(c.observed_ms);
        CHECK(w.valid && d.valid && w.depth.total_frames == i && w.inference.total_frames == i);
        CHECK(w.depth.window_frames == 1 && d.processed_frames == 1 && d.invalid_ratio == .5);
    }
    CHECK(allocations.load() == before);
    ++groups;
}
void bounded_consumer_audit() {
    struct Audit {
        std::array<SourceEvent, 4> rows{};
        std::array<Millis, 4> cuts{};
        std::size_t count = 0;
        bool reject = false;
        static bool record(void* p, const SourceEvent& e, Millis cut) noexcept {
            auto& a = *static_cast<Audit*>(p);
            if (a.count >= a.rows.size()) return false;
            a.rows[a.count] = e; a.cuts[a.count++] = cut;
            return !a.reject;
        }
        SourceDrainAudit sink() { return {this, record}; }
    } a;
    Rig r; r.at.now = ns(100) + 7; r.dt.now = ns(100) + 8;
    const auto s = source(1, ns(90));
    CHECK(r.arrival.record(s)); CHECK(r.processed.record(observation(s, 20, 3)));
    r.clock.now = 100;
    CHECK(r.h.drain(r.clock, a.sink()).code == HandoffCode::Ready && a.count == 0);
    CHECK(r.h.drain(r.clock, a.sink()).code == HandoffCode::NotAdvanced && a.count == 0);
    r.clock.now = 101; const auto before = allocations.load();
    const auto c = r.h.drain(r.clock, a.sink());
    CHECK(allocations.load() == before && c.code == HandoffCode::Ready && a.count == 2);
    CHECK(a.rows[0].kind == SourceKind::DepthArrival && a.rows[0].observed_ns == ns(100) + 7);
    CHECK(a.rows[1].kind == SourceKind::DepthProcessed && a.rows[1].depth.invalid_pixels == 3);
    CHECK(a.rows[0].source.sequence == s.sequence && a.rows[1].source.stream_id == s.stream_id);
    CHECK(a.rows[0].source.host_ready_mono_ns == s.host_ready_mono_ns && a.cuts[0] == 100 && a.cuts[1] == 100);
    CHECK(r.arrival.consumed() == 1 && r.processed.consumed() == 1);
    Rig failed; Audit rejected; rejected.reject = true; failed.at.now = ns(100);
    CHECK(failed.arrival.record(s)); failed.clock.now = 101;
    const auto bad = failed.h.drain(failed.clock, rejected.sink());
    CHECK(bad.code == HandoffCode::Invalid && !failed.h.valid() && bad.source_events[0] == 1);
    CHECK(rejected.count == 1 && failed.arrival.consumed() == 1 && failed.h.last_cut() == -1);
    CHECK(failed.health.snapshot(100).depth.total_frames == 1);
    CHECK(failed.h.drain(failed.clock, rejected.sink()).code == HandoffCode::Invalid && rejected.count == 1);
    Rig future; Audit not_called; future.at.now = ns(102); CHECK(future.arrival.record(s));
    future.clock.now = 101;
    CHECK(future.h.drain(future.clock, not_called.sink()).code == HandoffCode::Invalid && not_called.count == 0);
    ++groups;
}
}
int main() {
    independent_counts_and_windows(); exact_boundaries_and_no_observations();
    all_five_certification_before_any_mutation(); partial_millisecond_and_clock_bounds();
    late_expired_and_full_identity(); invalid_roles_overflow_and_evidence(); old_mode_and_allocation_free();
    bounded_consumer_audit();
    std::cout << "source_handoff groups=" << groups << " checks=" << checks
        << " allocation_free_iterations=1000 scope=synthetic_no_device_no_network\n";
}
