/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_status_heartbeat.cpp
 * Brief: Deterministic host scheduler and actual Status packets; no device or network.
 *
 * Description:
 * What it checks. The Status owner publishes at 1 Hz from a snapshot. The tests cover the initial
 * packet, pausing without a catch-up burst, transport failures, snapshot rejections, the guard on
 * the actual put, invalid metrics and bounded strings, clock edges, channel isolation and
 * configuration, that a capacity failure still consumes its slot, and an allocation stress.
 */

#include "output/rns_status_heartbeat.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>

#ifdef NDEBUG
#error "Heartbeat tests require enabled assertions"
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
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
    return ::operator new(n, std::nothrow);
}
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
std::size_t polls = 0, groups = 0;
struct FakeClock final : PublishClock {
    Millis wall = 1700000000123, mono = 0;
    unsigned wall_reads = 0, mono_reads = 0;
    Millis wall_millis() noexcept override { ++wall_reads; return wall; }
    Millis mono_millis() noexcept override { ++mono_reads; return mono; }
};
struct Sink final : RnsTransport {
    explicit Sink(FakeClock& c) : clock(c) {}
    FakeClock& clock;
    std::array<char, 65536> packet{};
    std::array<char, 100> key{};
    std::size_t calls = 0, bytes = 0;
    Millis delay = 0;
    PutOutcome outcome = PutOutcome::Accepted;
    PublishQos qos{Priority::RealTime, false, true};
    bool throws = false;
    PutReceipt put(std::string_view k, std::string_view p, PublishQos q) override {
        ++calls; assert(k.size() < key.size() && p.size() < packet.size());
        std::memcpy(key.data(), k.data(), k.size()); key[k.size()] = '\0';
        std::memcpy(packet.data(), p.data(), p.size()); packet[p.size()] = '\0';
        bytes = p.size(); qos = q;
        assert(delay <= 0 || clock.mono <= std::numeric_limits<Millis>::max() - delay);
        clock.mono += delay;
        if (throws) throw 7;
        return {outcome, outcome == PutOutcome::Rejected ? 17 : 0};
    }
};
Status measured_status() {
    Status s;
    s.fps_depth = 29.5; s.fps_infer = 12.25;
    s.latency_ms_p50 = 41; s.latency_ms_p99 = 120;
    s.infer_gap_ms_p99 = 90; s.infer_gap_ms_max = 350;
    s.invalid_pixel_ratio = .125; s.ground_seg_level = 2;
    s.extrinsic_calibrated = false; s.traversable_seg_available = true;
    s.degraded_reasons = {"extrinsic_uncalibrated", "explicit caller observation"};
    return s;
}
struct Fixture {
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    FakeClock clock;
    Sink sink{clock};
    PublisherChannel channel{Topic::Status, envelope, clock, sink, 32768, 65536};
    StatusHeartbeat heartbeat;
    Status status = measured_status();
    explicit Fixture(Millis startup = 0, Millis max_age = 100)
        : heartbeat(channel, startup, max_age) {}
};
HeartbeatResult checked(StatusHeartbeat& heartbeat, Millis now, const Status* status, Millis observed) {
    const auto before = ordinary_news.load(), aligned_before = aligned_news.load();
    const auto result = heartbeat.poll(now, status, observed);
    assert(ordinary_news == before && aligned_news == aligned_before);
    ++polls; return result;
}
HeartbeatResult at(Fixture& f, Millis now, Millis observed = -2) {
    f.clock.mono = now;
    return checked(f.heartbeat, now, &f.status, observed == -2 ? now : observed);
}
void pass(const char* name) { ++groups; std::cout << "PASS " << name << '\n'; }
void initial_and_packet() {
    Fixture f;
    assert(f.heartbeat.next_deadline_ms() == 1000 && f.heartbeat.next_eligible_ms() == 1000);
    assert(at(f, 0).code == HeartbeatCode::NotDue);
    assert(at(f, 999).code == HeartbeatCode::NotDue);
    assert(f.clock.mono_reads == 0 && f.clock.wall_reads == 0 && f.sink.calls == 0);
    const auto r = at(f, 1000, 925);
    assert(r.code == HeartbeatCode::Published && r.due && r.publisher_called);
    assert(r.scheduled_ms == 1000 && r.lateness_ms == 0 && r.snapshot_age_ms == 75);
    assert(r.dispatch_delay_ms == 0 && r.publication.audit.seq == 1);
    assert(r.publication.audit.put_before_ms == 1000 && r.publication.audit.put_return_ms == 1000);
    assert(r.publication.audit.capture_ms == -1 && r.publication.audit.clock_valid);
    assert(r.publication.audit.packet_bytes == f.sink.bytes);
    assert(std::string_view(f.sink.key.data()) == "xbrain/robot_1/rt/perception/status");
    assert(f.sink.qos.priority == Priority::DataHigh && f.sink.qos.reliable && !f.sink.qos.express && f.sink.qos.drop);
    const std::string_view packet(f.sink.packet.data());
    for (const auto field : {"\"schema\":\"perception_status_v1\"", "\"fps_depth\":29.5",
        "\"fps_infer\":12.25", "\"latency_ms_p50\":41", "\"latency_ms_p99\":120",
        "\"infer_gap_ms_p99\":90", "\"infer_gap_ms_max\":350", "\"invalid_pixel_ratio\":0.125",
        "\"ground_seg_level\":2", "\"extrinsic_calibrated\":false", "\"traversable_seg_available\":true",
        "\"t_publish_mono_ms\":1000", "\"mono\":1.000"})
        assert(packet.find(field) != std::string_view::npos);
    assert(f.status.t_publish_mono_ms == -1 && f.status.degraded_reasons.size() == 2);
    assert(at(f, 1000).code == HeartbeatCode::NotDue && f.sink.calls == 1);
    assert(at(f, 1999).code == HeartbeatCode::NotDue);
    assert(at(f, 2000).code == HeartbeatCode::Published);
    const auto counters = f.heartbeat.counters();
    assert(counters.due_beats == 2 && counters.missed_deadlines == 0 && counters.accepted == 2);
    pass("initial_999_1000_actual_packet");
}
void pause_and_no_burst() {
    Fixture f;
    assert(at(f, 1000).code == HeartbeatCode::Published);
    const auto late = at(f, 5500);
    assert(late.code == HeartbeatCode::Published && late.missed_deadlines == 3);
    assert(late.scheduled_ms == 5000 && late.lateness_ms == 500);
    assert(f.heartbeat.next_deadline_ms() == 6000 && f.heartbeat.next_eligible_ms() == 6500);
    assert(at(f, 5500).code == HeartbeatCode::NotDue);
    assert(at(f, 6000).code == HeartbeatCode::NotDue);
    assert(at(f, 6499).code == HeartbeatCode::NotDue);
    assert(at(f, 6500).code == HeartbeatCode::Published && f.sink.calls == 3);
    assert(at(f, 6999).code == HeartbeatCode::NotDue);
    assert(at(f, 7500).code == HeartbeatCode::Published);
    assert(f.heartbeat.counters().missed_deadlines == 3);
    Fixture offset(123);
    assert(at(offset, 1122).code == HeartbeatCode::NotDue);
    assert(at(offset, 1123).scheduled_ms == 1123);
    Fixture edge;
    assert(at(edge, 3999).missed_deadlines == 2);
    assert(at(edge, 4000).code == HeartbeatCode::NotDue);
    assert(at(edge, 4999).code == HeartbeatCode::Published);
    pass("pause_coalesce_no_backlog_burst");
}
void transport_failures() {
    Fixture f;
    f.sink.outcome = PutOutcome::Rejected;
    const auto rejected = at(f, 1000);
    assert(rejected.code == HeartbeatCode::PublisherFailure && rejected.publication.audit.attempted);
    assert(rejected.publication.error == PublishError::PutRejected && rejected.publication.audit.receipt.native_code == 17);
    assert(at(f, 1000).code == HeartbeatCode::NotDue && f.sink.calls == 1);
    f.sink.outcome = PutOutcome::Unknown;
    assert(at(f, 2000).publication.error == PublishError::PutUnknown);
    assert(at(f, 2999).code == HeartbeatCode::NotDue && f.sink.calls == 2);
    f.sink.throws = true;
    const auto thrown = at(f, 3000);
    assert(thrown.publication.error == PublishError::PutUnknown && thrown.publication.audit.threw);
    assert(at(f, 3000).code == HeartbeatCode::NotDue);
    f.sink.throws = false; f.sink.outcome = PutOutcome::Accepted;
    assert(at(f, 4000).code == HeartbeatCode::Published);
    f.sink.delay = -1;
    const auto after_bad = at(f, 5000);
    assert(after_bad.code == HeartbeatCode::PublisherFailure && after_bad.publication.error == PublishError::ClockAfter);
    assert(after_bad.publication.audit.receipt.outcome == PutOutcome::Accepted);
    assert(!after_bad.publication.audit.clock_valid && at(f, 5000).code == HeartbeatCode::NotDue);
    f.sink.delay = 0;
    assert(at(f, 6000).code == HeartbeatCode::Published);
    const auto c = f.heartbeat.counters();
    assert(c.due_beats == 6 && c.put_attempts == 6 && c.accepted == 3 && c.rejected == 1 && c.unknown == 2);
    pass("rejected_unknown_throw_clockafter_no_retry");
}
void snapshot_rejections() {
    Fixture f;
    assert(at(f, 1000).code == HeartbeatCode::Published);
    assert(checked(f.heartbeat, 2000, nullptr, 2000).code == HeartbeatCode::MissingSnapshot);
    assert(at(f, 2500).code == HeartbeatCode::NotDue && f.sink.calls == 1);
    assert(at(f, 3000, -1).code == HeartbeatCode::InvalidSnapshot);
    assert(at(f, 4000, 4001).code == HeartbeatCode::FutureSnapshot);
    assert(at(f, 5000, 4899).code == HeartbeatCode::StaleSnapshot);
    assert(f.sink.calls == 1);
    assert(at(f, 6000, 5900).code == HeartbeatCode::Published); // inclusive max-age edge
    f.status.fps_depth = std::numeric_limits<double>::quiet_NaN();
    const auto nan = at(f, 7000);
    assert(nan.code == HeartbeatCode::InvalidSnapshot && nan.publication.error == PublishError::InvalidBody);
    assert(!nan.publication.audit.attempted && f.sink.calls == 2);
    f.status = Status{}; // Explicit measured zero DTO is valid; adapter did not fabricate it.
    assert(at(f, 8000).code == HeartbeatCode::Published);
    assert(f.heartbeat.counters().snapshot_rejections == 5);
    Fixture zero_age(0, 0);
    assert(at(zero_age, 1000, 999).code == HeartbeatCode::StaleSnapshot);
    assert(at(zero_age, 2000, 2000).code == HeartbeatCode::Published);
    pass("missing_future_stale_nan_never_replay_metrics");
}
void actual_put_guard() {
    Fixture f;
    f.clock.mono = 999;
    auto r = checked(f.heartbeat, 1000, &f.status, 950);
    assert(r.code == HeartbeatCode::SnapshotRejectedAtPut && !r.publication.audit.attempted);
    assert(r.publication.error == PublishError::SnapshotInvalid && f.sink.calls == 0);
    assert(r.publication.audit.put_before_ms == -1 && r.dispatch_delay_ms == -1);
    assert(at(f, 1000).code == HeartbeatCode::NotDue);
    assert(at(f, 2000).publication.audit.seq == 1); // guard rejection did not consume sequence
    f.clock.mono = 3101;
    r = checked(f.heartbeat, 3000, &f.status, 3000);
    assert(r.code == HeartbeatCode::SnapshotRejectedAtPut && f.sink.calls == 1);
    f.clock.mono = 4020; f.sink.delay = 5;
    r = checked(f.heartbeat, 4000, &f.status, 3990);
    assert(r.code == HeartbeatCode::Published && r.dispatch_delay_ms == 20);
    assert(r.publication.audit.seq == 2 && r.publication.audit.put_return_ms == 4025);
    assert(f.heartbeat.next_eligible_ms() == 5020);
    assert(checked(f.heartbeat, 4024, &f.status, 4024).code == HeartbeatCode::ClockRollback);
    assert(at(f, 5000).code == HeartbeatCode::NotDue && at(f, 5019).code == HeartbeatCode::NotDue);
    f.sink.delay = 0;
    assert(at(f, 5020).code == HeartbeatCode::Published);
    assert(f.heartbeat.counters().snapshot_rejections == 2);
    // Legacy unguarded status publication remains available, independent of scheduler.
    Fixture direct;
    direct.clock.mono = 10;
    assert(direct.channel.publish(direct.status).audit.attempted);
    pass("actual_before_notbefore_freshness_and_cooldown");
}
void invalid_metrics_and_bounded_strings() {
    Fixture f;
    f.status.invalid_pixel_ratio = 1.1;
    assert(at(f, 1000).code == HeartbeatCode::InvalidSnapshot);
    f.status = measured_status(); f.status.latency_ms_p50 = 121;
    assert(at(f, 2000).code == HeartbeatCode::InvalidSnapshot);
    f.status = measured_status(); f.status.infer_gap_ms_p99 = 351;
    assert(at(f, 3000).code == HeartbeatCode::InvalidSnapshot);
    f.status = measured_status(); f.status.fps_infer = std::numeric_limits<double>::infinity();
    assert(at(f, 4000).code == HeartbeatCode::InvalidSnapshot);
    f.status = measured_status(); f.status.degraded_reasons = {std::string(1, static_cast<char>(0xff))};
    assert(at(f, 5000).code == HeartbeatCode::InvalidSnapshot);
    f.status = measured_status(); f.status.degraded_reasons.assign(StatusHeartbeat::kMaxReasons + 1, "");
    assert(!at(f, 6000).publisher_called);
    f.status.degraded_reasons = {std::string(StatusHeartbeat::kMaxReasonBytes + 1, 'x')};
    assert(at(f, 7000).code == HeartbeatCode::InvalidSnapshot);
    f.status.degraded_reasons = {std::string(StatusHeartbeat::kMaxReasonBytes / 2, 'x'),
                                std::string(StatusHeartbeat::kMaxReasonBytes / 2 + 1, 'y')};
    assert(at(f, 8000).code == HeartbeatCode::InvalidSnapshot && f.sink.calls == 0);
    f.status.degraded_reasons = {std::string(StatusHeartbeat::kMaxReasonBytes, 'x')};
    assert(at(f, 9000).code == HeartbeatCode::Published);
    f.status.degraded_reasons.assign(StatusHeartbeat::kMaxReasons, "");
    assert(at(f, 10000).code == HeartbeatCode::Published);
    pass("metric_validation_utf8_and_explicit_support_bounds");
}
void clock_edges() {
    Fixture f;
    assert(checked(f.heartbeat, -1, &f.status, -1).code == HeartbeatCode::InvalidClock);
    assert(at(f, 999).code == HeartbeatCode::NotDue);
    assert(at(f, 998).code == HeartbeatCode::ClockRollback);
    assert(f.heartbeat.next_deadline_ms() == 1000);
    assert(at(f, 1000).code == HeartbeatCode::Published);
    assert(at(f, 999).code == HeartbeatCode::ClockRollback);
    constexpr auto max = ProcessEnvelope::kMaxMonoMs;
    assert(checked(f.heartbeat, std::numeric_limits<Millis>::max(), &f.status, 0).code == HeartbeatCode::InvalidClock);
    assert(f.heartbeat.counters().clock_rejections == 4);
    Fixture end(max - 1000, max);
    assert(at(end, max - 1).code == HeartbeatCode::NotDue);
    const auto last = at(end, max, 0);
    assert(last.code == HeartbeatCode::Published && last.snapshot_age_ms == max);
    assert(end.heartbeat.next_deadline_ms() == -1 && end.heartbeat.next_eligible_ms() == -1);
    assert(at(end, max).code == HeartbeatCode::Exhausted && end.sink.calls == 1);
    Fixture huge(0, max);
    const auto paused = at(huge, max, 0);
    assert(paused.code == HeartbeatCode::Published);
    assert(paused.missed_deadlines == static_cast<std::uint64_t>((max - 1000) / 1000));
    assert(huge.heartbeat.counters().missed_deadlines == paused.missed_deadlines);
    assert(!huge.heartbeat.counters().saturated && at(huge, max).code == HeartbeatCode::Exhausted);
    pass("clock_rollback_huge_pause_and_last_representable_beat");
}
template <class F> void rejects_constructor(F fn) {
    bool threw = false;
    try { fn(); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
}
void channel_isolation_and_configuration() {
    Fixture f(30000); // Exercise the independent cadence after its documented warmup.
    ObjectsCadence cadence(0, 32);
    PublisherChannel objects(Topic::Objects, f.envelope, f.clock, f.sink, 32768, 65536, &cadence);
    PublisherChannel profile(Topic::Profile, f.envelope, f.clock, f.sink, 32768, 65536);
    rejects_constructor([&] { StatusHeartbeat wrong(objects, 0, 100); });
    rejects_constructor([&] { StatusHeartbeat wrong(profile, 0, 100); });
    rejects_constructor([&] { StatusHeartbeat bad(f.channel, -1, 100); });
    rejects_constructor([&] { StatusHeartbeat bad(f.channel, ProcessEnvelope::kMaxMonoMs - 999, 100); });
    rejects_constructor([&] { StatusHeartbeat bad(f.channel, 0, -1); });
    rejects_constructor([&] { StatusHeartbeat bad(f.channel, 0, std::numeric_limits<Millis>::max()); });
    PublisherChannel no_buffer(Topic::Status, f.envelope, f.clock, f.sink, 0, 0);
    rejects_constructor([&] { StatusHeartbeat bad(no_buffer, 0, 100); });
    assert(objects.counters().attempted == 0 && objects.counters().preflight_failed == 0);
    assert(profile.counters().attempted == 0 && profile.counters().preflight_failed == 0);
    assert(f.clock.mono_reads == 0 && f.clock.wall_reads == 0 && f.sink.calls == 0);
    assert(at(f, 31000).publication.audit.seq == 1);
    Objects obj; obj.t_capture_mono_ms = 31500; f.clock.mono = 31500;
    assert(objects.publish(obj).audit.seq == 2);
    for (Millis now = 32000; now <= 42000; now += 1000) assert(at(f, now).code == HeartbeatCode::Published);
    const auto gap = cadence.snapshot(42000);
    assert(gap.valid && gap.unique_results == 1 && gap.open_gap_ms == 10500);
    assert(gap.max_gap_ms == 10500 && !gap.pass);
    assert(objects.counters().attempted == 1 && profile.counters().attempted == 0);
    assert(f.heartbeat.counters().accepted == 12 && f.heartbeat.counters().missed_deadlines == 0);
    pass("wrong_topic_reject_and_independent_silent_fast_slow_paths");
}
void capacity_failure_still_consumes() {
    Fixture f;
    PublisherChannel tiny(Topic::Status, f.envelope, f.clock, f.sink, 2, 32);
    StatusHeartbeat heartbeat(tiny, 0, 100);
    for (Millis now : {1000, 2000, 3000}) {
        const auto r = checked(heartbeat, now, &f.status, now);
        assert(r.code == HeartbeatCode::PublisherFailure && r.publication.error == PublishError::Capacity);
        assert(!r.publication.audit.attempted && r.due);
        assert(checked(heartbeat, now, &f.status, now).code == HeartbeatCode::NotDue);
    }
    assert(f.sink.calls == 0 && heartbeat.counters().due_beats == 3);
    assert(heartbeat.counters().publisher_calls == 3 && heartbeat.counters().put_attempts == 0);
    pass("capacity_preflight_consumes_each_due_beat");
}
void allocation_stress() {
    const auto before = ordinary_news.load(), aligned_before = aligned_news.load();
    auto* a = ::operator new(1); ::operator delete(a);
    auto* b = ::operator new(64, std::align_val_t{64}); ::operator delete(b, std::align_val_t{64});
    assert(ordinary_news == before + 1 && aligned_news == aligned_before + 1);
    Fixture f;
    for (Millis now = 0; now <= 200000; now += 10) {
        const auto r = at(f, now);
        assert(r.code == ((now != 0 && now % 1000 == 0) ? HeartbeatCode::Published : HeartbeatCode::NotDue));
    }
    assert(f.sink.calls == 200 && f.heartbeat.counters().accepted == 200);
    assert(f.heartbeat.counters().missed_deadlines == 0 && !f.heartbeat.counters().saturated);
    pass("ordinary_and_aligned_new_hooks_all_polls_zero_allocation");
}
void print_fixture(const char* name, Fixture& f, Millis now, Millis observed) {
    const auto r = checked(f.heartbeat, now, &f.status, observed);
    assert(r.code == HeartbeatCode::Published);
    std::cout << "{\"case\":\"" << name << "\",\"key\":\"" << f.sink.key.data() << "\",\"packet\":" << f.sink.packet.data()
              << ",\"before\":" << r.publication.audit.put_before_ms
              << ",\"after\":" << r.publication.audit.put_return_ms
              << ",\"capture\":-1,\"outcome\":\"accepted\",\"qos\":{\"drop\":true,\"reliable\":true,"
                 "\"express\":false,\"priority\":\"data_high\"},\"heartbeat\":{\"poll_now_ms\":" << now
              << ",\"observed_ms\":" << observed << ",\"max_age_ms\":100,\"scheduled_ms\":" << r.scheduled_ms
              << ",\"dispatch_delay_ms\":" << r.dispatch_delay_ms << "}}\n";
}
void fixture() {
    Fixture f;
    assert(f.envelope.note_clock_status(true, 999));
    f.clock.mono = 1007; f.sink.delay = 2;
    print_fixture("normal", f, 1000, 975);
    // Explicit synthetic observations; scheduler does not estimate these values.
    f.status.fps_depth = 0; f.status.fps_infer = 0;
    f.status.traversable_seg_available = false;
    f.status.infer_gap_ms_p99 = 1100; f.status.infer_gap_ms_max = 1100;
    f.status.degraded_reasons = {"reconnecting"};
    f.clock.mono = 2012;
    print_fixture("reconnecting_silent", f, 2010, 2000);
    f.status = measured_status(); f.clock.mono = 3021;
    print_fixture("recovered", f, 3020, 3010);
}
}  // namespace
int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--emit-fixtures") { fixture(); return 0; }
    initial_and_packet(); pause_and_no_burst(); transport_failures(); snapshot_rejections();
    actual_put_guard(); invalid_metrics_and_bounded_strings(); clock_edges();
    channel_isolation_and_configuration(); capacity_failure_still_consumes(); allocation_stress();
    std::cout << "rns_status_heartbeat: " << groups << " groups PASS; " << polls
              << " polls, ordinary/aligned process allocations = 0/0\n";
}
