/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_declaration.cpp
 * Brief: Unit tests for the ordered Objects owner declarations, with injected clocks and transport.
 *
 * Description:
 * What it checks. Order within one millisecond, a full queue and cancellation, invalid clocks,
 * enums and topic, the journal protocol and its exhaustion, pending and timing re-entry, history
 * resume without a journal, and the preparation clock and allocation.
 * No real reconnect, network, sensor, native SDK or production acceptance.
 */

#include "output/rns_publication_journal.hpp"
#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>

static std::atomic<std::size_t> allocations{0};
void* operator new(std::size_t n) {
    ++allocations;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) {
    ++allocations;
    void* p = nullptr;
    if (posix_memalign(&p, static_cast<std::size_t>(a), n ? n : 1) == 0) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n, std::align_val_t a) { return ::operator new(n, a); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

using namespace perception::rns;
namespace perception::rns {
struct PublicationJournalTestAccess {
    static void positions(PublicationJournal& j, std::uint64_t p) {
        j.head_.store(p); j.tail_.store(p);
    }
    static void generation(PublicationJournal& j, std::uint64_t g) { j.generation_.store(g); }
};
}
namespace {
std::size_t groups = 0, checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; std::abort(); } } while (false)
using Journal = PublicationJournal;
using Event = Journal::Event;
using Kind = Journal::Kind;
Objects objects(Millis capture) { Objects o; o.t_capture_mono_ms = capture; return o; }
struct Clock final : PublishClock {
    Millis now = 30000, wall = 1700000000000;
    unsigned mono_reads = 0, wall_reads = 0;
    Journal* inspect = nullptr;
    bool require_odd = false;
    void* hook_context = nullptr;
    void (*hook)(void*) noexcept = nullptr;
    Millis mono_millis() noexcept override {
        ++mono_reads;
        if (require_odd) CHECK(inspect && (inspect->generation() & 1U));
        if (hook) hook(hook_context);
        return now;
    }
    Millis wall_millis() noexcept override { ++wall_reads; return wall; }
};
struct Sink final : RnsTransport {
    explicit Sink(Clock& c) : clock(c) {}
    Clock& clock;
    unsigned calls = 0;
    Millis delay = 0;
    bool prepare_reject = false;
    PutOutcome outcome = PutOutcome::Accepted;
    void* hook_context = nullptr;
    void (*hook)(void*) noexcept = nullptr;
    PutReceipt put(std::string_view, std::string_view, PublishQos) override {
        ++calls;
        if (hook) hook(hook_context);
        clock.now += delay;
        return {outcome, 0};
    }
    PutReceipt put_finalized(std::string_view k, char* p, std::size_t n,
                           PublishQos q, PacketFinalizer& f) override {
        if (prepare_reject) return {PutOutcome::Rejected, 1};
        if (!f.finalize(p, n)) return {PutOutcome::Rejected, 2};
        return put(k, {p, n}, q);
    }
};
struct Fixture {
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    Clock clock;
    Sink sink{clock};
    ObjectsCadence cadence{0, 128};
    Journal journal;
    PublisherChannel channel;
    explicit Fixture(std::size_t capacity = 16, bool with_journal = true)
        : journal(Topic::Objects, capacity),
          channel(Topic::Objects, envelope, clock, sink, 1024, 2048, &cadence,
                  with_journal ? &journal : nullptr) {}
};
bool default_audit(const PublishAudit& a) {
    return !a.attempted && !a.threw && !a.clock_valid && a.seq == 0 &&
        a.capture_ms == -1 && a.put_before_ms == -1 && a.put_return_ms == -1 &&
        a.packet_bytes == 0 && a.receipt.outcome == PutOutcome::Unknown &&
        a.receipt.native_code == 0 && a.cadence_event == CadenceEvent::Invalid;
}
PublishAudit attempt(std::uint64_t seq, Millis now) {
    PublishAudit a;
    a.attempted = true; a.seq = seq; a.capture_ms = now - 1;
    a.put_before_ms = now; a.packet_bytes = 200;
    return a;
}
void ordered_same_millisecond() {
    Fixture f;
    f.clock.inspect = &f.journal; f.clock.require_odd = true;
    const auto start = f.channel.declare_exclusion(Exclusion::Reconnecting);
    CHECK(start && start.declaration == Exclusion::Reconnecting && start.declared_ms == 30000);
    CHECK(f.clock.mono_reads == 1 && f.clock.wall_reads == 0 && f.sink.calls == 0);
    CHECK(f.channel.counters().attempted == 0 && f.journal.generation() == 2);
    f.clock.require_odd = false; // put-return sampling is also odd with a journal
    const auto a = f.channel.publish(objects(29900));
    CHECK(a && a.audit.seq == 1 && a.audit.cadence_event == CadenceEvent::Excluded);
    const auto end = f.channel.declare_exclusion(Exclusion::None);
    CHECK(end && end.declared_ms == 30000);
    const auto b = f.channel.publish(objects(29901));
    CHECK(b && b.audit.seq == 2 && b.audit.cadence_event == CadenceEvent::New);
    const std::array<Kind, 6> kinds{{Kind::Declaration, Kind::Attempt, Kind::Completion,
                                    Kind::Declaration, Kind::Attempt, Kind::Completion}};
    Event e;
    for (std::size_t i = 0; i < kinds.size(); ++i) {
        CHECK(f.journal.pop(e) && e.kind == kinds[i] && e.topic == Topic::Objects);
        if (e.kind == Kind::Declaration) {
            CHECK(e.declared_ms == 30000 && default_audit(e.audit));
            CHECK(e.declaration == (i == 0 ? Exclusion::Reconnecting : Exclusion::None));
        } else {
            CHECK(e.declared_ms == -1 && e.declaration == Exclusion::None);
            CHECK(e.audit.put_before_ms == 30000 && e.audit.seq == (i < 3 ? 1U : 2U));
        }
    }
    CHECK(!f.journal.pop(e) && f.journal.valid() && f.journal.generation() == 12);
    const auto stats = f.channel.counters();
    CHECK(stats.attempted == 2 && stats.accepted == 2 && stats.preflight_failed == 0);
    ++groups;
}
void full_queue_and_cancellation() {
    Fixture f(2);
    CHECK(f.channel.declare_exclusion(Exclusion::Reconnecting));
    CHECK(f.channel.declare_exclusion(Exclusion::Reconfiguring));
    const auto reads = f.clock.mono_reads;
    const auto full = f.channel.declare_exclusion(Exclusion::None);
    CHECK(full.error == PublishError::TelemetryUnavailable && full.declared_ms == -1);
    CHECK(f.clock.mono_reads == reads && f.journal.generation() == 4 && f.journal.valid());
    CHECK(f.journal.counters().admission_rejected == 1);
    CHECK(f.cadence.snapshot(30000).declared_exclusion == Exclusion::Reconfiguring);
    Event e; CHECK(f.journal.pop(e));
    CHECK(f.channel.declare_exclusion(Exclusion::None)); // one slot is enough
    CHECK(f.journal.pop(e) && e.declaration == Exclusion::Reconfiguring);
    CHECK(f.journal.pop(e) && e.declaration == Exclusion::None);
    CHECK(f.channel.declare_exclusion(Exclusion::None));
    const auto rejected = f.channel.publish(objects(29900)); // only one slot left
    CHECK(rejected.error == PublishError::TelemetryUnavailable && !rejected.audit.attempted);
    CHECK(f.sink.calls == 0 && f.channel.counters().attempted == 0);
    CHECK(f.journal.pop(e));
    CHECK(f.channel.publish(objects(29900)).audit.seq == 1); // same capture may retry
    CHECK(f.journal.pop(e) && f.journal.pop(e));
    CHECK(f.journal.begin_declaration() && (f.journal.generation() & 1U));
    f.journal.cancel();
    CHECK(f.journal.valid() && !(f.journal.generation() & 1U) && !f.journal.pop(e));
    CHECK(f.journal.begin_declaration() && f.journal.record_declaration(Exclusion::None, 30001));
    ++groups;
}
void invalid_clocks_enums_and_topic() {
    Fixture f;
    CHECK(f.channel.declare_exclusion(Exclusion::Reconnecting));
    for (const auto bad : {Millis{-1}, Millis{29999}, std::numeric_limits<Millis>::max()}) {
        f.clock.now = bad;
        const auto before = f.journal.published();
        CHECK(f.channel.declare_exclusion(Exclusion::None).error == PublishError::ClockBefore);
        CHECK(f.journal.valid() && !(f.journal.generation() & 1U) && f.journal.published() == before);
    }
    f.clock.now = 29999; // a failed negative clock did not lower last_clock
    CHECK(f.channel.publish(objects(29900)).error == PublishError::ClockBefore);
    CHECK(f.journal.valid() && !(f.journal.generation() & 1U));
    const auto reads = f.clock.mono_reads;
    CHECK(f.channel.declare_exclusion(static_cast<Exclusion>(99)).error == PublishError::InvalidExclusion);
    CHECK(f.clock.mono_reads == reads);
    f.clock.now = 30000;
    CHECK(f.cadence.snapshot(30000).declared_exclusion == Exclusion::Reconnecting);
    CHECK(f.channel.declare_exclusion(Exclusion::None));
    CHECK(f.channel.publish(objects(29900)).audit.seq == 1);
    CHECK(f.sink.calls == 1);
    for (const auto topic : {Topic::Profile, Topic::Status}) {
        PublisherChannel other(topic, f.envelope, f.clock, f.sink, 1024, 2048);
        const auto n = f.clock.mono_reads;
        CHECK(other.declare_exclusion(Exclusion::Reconnecting).error == PublishError::WrongTopic);
        CHECK(f.clock.mono_reads == n);
    }
    PublisherChannel unconfigured(Topic::Objects, f.envelope, f.clock, f.sink, 1024, 2048);
    CHECK(unconfigured.declare_exclusion(Exclusion::None).error == PublishError::Configuration);
    // A cadence observed ahead of this owner's clock cannot accept a backdate.
    Fixture ahead;
    CHECK(ahead.cadence.snapshot(31000).valid);
    CHECK(ahead.channel.declare_exclusion(Exclusion::Reconnecting).error == PublishError::CadenceRejected);
    CHECK(ahead.journal.published() == 0 && !(ahead.journal.generation() & 1U));
    ++groups;
}
void journal_protocol_and_exhaustion() {
    for (unsigned variant = 0; variant < 8; ++variant) {
        Journal j(variant == 0 ? Topic::Profile : Topic::Objects, 4);
        if (variant == 0) CHECK(!j.begin_declaration());
        else if (variant == 1) CHECK(!j.record_declaration(Exclusion::None, 30000));
        else if (variant == 2) { CHECK(j.reserve()); CHECK(!j.begin_declaration()); }
        else if (variant == 3) {
            CHECK(j.reserve()); j.begin_attempt(); j.record_attempt(attempt(1, 30000));
            CHECK(!j.begin_declaration());
            Event e; CHECK(j.pop(e) && e.kind == Kind::Attempt && e.audit.seq == 1);
        } else {
            CHECK(j.begin_declaration());
            if (variant == 4) CHECK(!j.begin_declaration());
            if (variant == 5) CHECK(!j.record_declaration(static_cast<Exclusion>(99), 30000));
            if (variant == 6) CHECK(!j.record_declaration(Exclusion::None, -1));
            if (variant == 7) CHECK(!j.record_declaration(Exclusion::None, std::numeric_limits<Millis>::max()));
        }
        CHECK(!j.valid() && j.counters().protocol_faults == 1);
        if (variant >= 5) CHECK(!(j.generation() & 1U));
    }
    constexpr auto max = std::numeric_limits<std::uint64_t>::max();
    Journal pos(Topic::Objects, 2);
    PublicationJournalTestAccess::positions(pos, max - 1);
    CHECK(pos.begin_declaration() && pos.record_declaration(Exclusion::None, 0));
    Event e; CHECK(pos.pop(e) && pos.published() == max && pos.consumed() == max);
    CHECK(!pos.begin_declaration() && !pos.valid());
    Journal generation(Topic::Objects, 2);
    PublicationJournalTestAccess::generation(generation, max - 3);
    CHECK(generation.begin_declaration() && generation.record_declaration(Exclusion::Reconnecting, 0));
    CHECK(generation.generation() == max - 1);
    CHECK(!generation.begin_declaration() && !generation.valid());
    ++groups;
}
struct Reentry {
    Fixture* f;
    unsigned callbacks = 0;
    bool expect_attempt;
    static void call(void* context) noexcept {
        auto& r = *static_cast<Reentry*>(context); ++r.callbacks;
        auto& f = *r.f;
        const auto head = f.journal.published(), generation = f.journal.generation();
        const auto reads = f.clock.mono_reads;
        Event e;
        if (r.expect_attempt) CHECK(f.journal.peek(e) && e.kind == Kind::Attempt);
        else CHECK(generation & 1U);
        CHECK(f.channel.declare_exclusion(Exclusion::Reconnecting).error == PublishError::Reentrant);
        CHECK(f.channel.publish(objects(29901)).error == PublishError::Reentrant);
        CHECK(f.journal.valid() && f.journal.published() == head && f.journal.generation() == generation);
        CHECK(f.clock.mono_reads == reads);
    }
};
void pending_and_timing_reentry() {
    Fixture f;
    Reentry r{&f, 0, true};
    f.sink.hook = &Reentry::call; f.sink.hook_context = &r;
    const auto first = f.channel.publish(objects(29900));
    CHECK(first && r.callbacks == 1 && first.audit.seq == 1);
    CHECK(f.channel.counters().attempted == 1 && f.journal.published() == 2);
    Event e; CHECK(f.journal.pop(e) && e.kind == Kind::Attempt);
    CHECK(f.journal.pop(e) && e.kind == Kind::Completion && e.audit.seq == 1);
    CHECK(f.cadence.snapshot(30000).declared_exclusion == Exclusion::None);
    f.sink.hook = nullptr;
    r.expect_attempt = false;
    f.clock.hook = &Reentry::call; f.clock.hook_context = &r;
    CHECK(f.channel.declare_exclusion(Exclusion::Reconfiguring));
    CHECK(r.callbacks == 2 && f.journal.generation() == 6);
    f.clock.hook = nullptr;
    CHECK(f.channel.declare_exclusion(Exclusion::None));
    CHECK(f.channel.publish(objects(29901)).audit.seq == 2);
    ++groups;
}
void history_resume_and_no_journal() {
    Fixture f;
    CHECK(f.cadence.snapshot(29999).excluded);
    CHECK(f.cadence.snapshot(29999).declared_exclusion == Exclusion::None);
    CHECK(f.channel.publish(objects(29900)));
    f.clock.now = 30200;
    CHECK(f.channel.declare_exclusion(Exclusion::Reconnecting));
    auto s = f.cadence.snapshot(30200);
    CHECK(s.excluded && s.declared_exclusion == Exclusion::Reconnecting);
    CHECK(s.historical_max_gap_ms == 200 && s.infer_gap && s.infer_rate_low);
    f.clock.now = 33000;
    CHECK(f.channel.publish(objects(32900)).audit.cadence_event == CadenceEvent::Excluded);
    f.clock.now = 35000;
    CHECK(f.channel.declare_exclusion(Exclusion::None));
    CHECK(f.channel.publish(objects(32900)).error == PublishError::Duplicate);
    CHECK(f.channel.publish(objects(32899)).error == PublishError::OutOfOrder);
    s = f.cadence.snapshot(35000);
    CHECK(!s.excluded && s.awaiting_new_segment && s.declared_excluded_ms == 4800);
    CHECK(s.declared_exclusion == Exclusion::None && s.historical_max_gap_ms == 200);
    CHECK(s.infer_gap && s.infer_rate_low);
    f.clock.now = 35001;
    CHECK(f.channel.publish(objects(35000)).audit.cadence_event == CadenceEvent::New);
    s = f.cadence.snapshot(35001);
    CHECK(!s.awaiting_new_segment && s.closed_gaps == 0 && s.historical_max_gap_ms == 200);
    Fixture direct(2, false);
    CHECK(direct.channel.declare_exclusion(Exclusion::Reconnecting));
    CHECK(direct.channel.publish(objects(29900)).audit.seq == 1);
    CHECK(direct.channel.declare_exclusion(Exclusion::None));
    CHECK(direct.channel.publish(objects(29901)).audit.seq == 2);
    CHECK(direct.journal.published() == 0 && direct.journal.generation() == 0);
    ++groups;
}
void preparation_clock_and_allocation() {
    Fixture f(4);
    const auto news = allocations.load();
    Event e;
    for (unsigned i = 0; i < 100; ++i) {
        f.clock.now = 30000 + static_cast<Millis>(i) * 40;
        CHECK(f.channel.declare_exclusion(Exclusion::Reconnecting));
        CHECK(f.journal.pop(e) && e.kind == Kind::Declaration);
        CHECK(f.channel.declare_exclusion(Exclusion::None));
        CHECK(f.journal.pop(e) && e.kind == Kind::Declaration);
        f.sink.prepare_reject = true;
        CHECK(f.channel.publish(objects(f.clock.now - 1)).error == PublishError::PrepareRejected);
        CHECK(f.journal.published() == f.journal.consumed() && !(f.journal.generation() & 1U));
        f.sink.prepare_reject = false;
        const auto before = f.clock.now;
        f.clock.now = -1;
        CHECK(f.channel.publish(objects(before - 1)).error == PublishError::ClockBefore);
        CHECK(f.journal.valid() && !(f.journal.generation() & 1U));
        f.clock.now = before;
        CHECK(f.channel.publish(objects(before - 1)).audit.seq == i + 1);
        CHECK(f.journal.pop(e) && e.kind == Kind::Attempt);
        CHECK(f.journal.pop(e) && e.kind == Kind::Completion);
    }
    CHECK(allocations.load() == news);
    CHECK(f.sink.calls == 100 && f.channel.counters().attempted == 100);
    ++groups;
}
} // namespace
int main() {
    ordered_same_millisecond(); full_queue_and_cancellation(); invalid_clocks_enums_and_topic();
    journal_protocol_and_exhaustion(); pending_and_timing_reentry(); history_resume_and_no_journal();
    preparation_clock_and_allocation();
    std::cout << "PASS declaration groups=" << groups << " checks=" << checks
              << " injected clocks/transport; no reconnect or production execution\n";
}
