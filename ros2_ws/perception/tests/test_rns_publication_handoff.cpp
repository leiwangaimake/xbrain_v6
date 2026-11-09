/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_publication_handoff.cpp
 * Brief: Certified publication cuts with injected clocks/transports only.
 *
 * Description:
 * No camera, SDK, network, production source, scheduling or 20 Hz claim.
 */

#include "output/rns_publication_handoff.hpp"
#include "output/rns_health_fields.hpp"
#include "output/rns_status_heartbeat.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <new>
#include <thread>

static std::atomic<std::size_t> allocation_count{0};
void* operator new(std::size_t n) {
    allocation_count.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace perception::rns;
namespace {
std::size_t checks = 0, groups = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __FILE__ << ':' << __LINE__ \
    << ": " #x "\n"; std::abort(); } } while (false)

struct Clock final : PublishClock {
    std::atomic<Millis>& now;
    explicit Clock(std::atomic<Millis>& n) : now(n) {}
    Millis wall_millis() noexcept override { return 1700000000000LL; }
    Millis mono_millis() noexcept override { return now.load(std::memory_order_seq_cst); }
};
struct Sink : RnsTransport {
    std::array<char, 16384> bytes{};
    std::size_t size = 0;
    std::atomic<std::size_t> calls{0};
    PutReceipt put(std::string_view, std::string_view p, PublishQos) override {
        if (p.size() >= bytes.size()) std::abort();
        std::memcpy(bytes.data(), p.data(), p.size());
        size = p.size(); bytes[size] = 0;
        calls.fetch_add(1, std::memory_order_release);
        return {PutOutcome::Accepted, 0};
    }
};
struct BlockingSink final : Sink {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false, release = false;
    PutReceipt put(std::string_view k, std::string_view p, PublishQos q) override {
        const auto receipt = Sink::put(k, p, q);
        std::unique_lock<std::mutex> lock(mutex);
        entered = true; cv.notify_all();
        if (!cv.wait_for(lock, std::chrono::seconds(10), [&] { return release; })) std::abort();
        return receipt;
    }
    void wait_entered() {
        std::unique_lock<std::mutex> lock(mutex);
        CHECK(cv.wait_for(lock, std::chrono::seconds(10), [&] { return entered; }));
    }
    void unblock() {
        std::lock_guard<std::mutex> lock(mutex); release = true; cv.notify_all();
    }
};
Objects object(Millis capture) { Objects o; o.t_capture_mono_ms = capture; return o; }
Profile profile(Millis capture) {
    Profile p; p.t_capture_mono_ms = capture; p.z_pass_m = .75F; p.range_max_m = 6;
    p.blind_near_m.reset(); p.angle_min_rad = -.01F; p.angle_step_rad = .01F;
    p.bins.resize(3);
    return p;
}
PublishAudit attempt(std::uint64_t seq, Millis before) {
    PublishAudit a; a.attempted = true; a.seq = seq; a.capture_ms = before - 1;
    a.put_before_ms = before; a.packet_bytes = 200; return a;
}
void commit_attempt(PublicationJournal& j, const PublishAudit& a) {
    CHECK(j.reserve()); j.begin_attempt(); j.record_attempt(a); CHECK(j.valid());
}
void complete(PublicationJournal& j, PublishAudit a, Millis after) {
    a.put_return_ms = after; a.clock_valid = true; a.receipt = {PutOutcome::Accepted, 0};
    j.begin_completion(); j.record_completion(a); CHECK(j.valid());
}
struct Rig {
    std::atomic<Millis> now{0}; Clock clock{now};
    PublicationJournal p{Topic::Profile, 64}, o{Topic::Objects, 64};
    HealthWindow health{0, 64}; ObjectsCadence cadence{0, 64};
    PublicationHealthHandoff handoff{0, p, o, health, cadence};
};
void emit_status(const char* name, const Sink& sink, const PublisherChannel& channel,
        const PublicationCut& cut, const HealthWindowReport& w,
        const DepthHealthWindowReport& d, const PublishAudit& audit, bool blocked) {
    const auto counts = [](const HealthPutCounts& c) {
        std::cout << "{\"attempted\":" << c.attempted << ",\"accepted\":" << c.accepted
            << ",\"rejected\":" << c.rejected << ",\"unknown\":" << c.unknown
            << ",\"pending\":" << c.pending << '}';
    };
    std::cout << "{\"case\":\"" << name << "\",\"key\":\"" << channel.key()
        << "\",\"packet\":" << sink.bytes.data()
        << ",\"scope\":\"injected_clock_and_transport_no_camera_no_network\""
        << ",\"sampled_now_ms\":" << cut.sampled_now_ms << ",\"cut_ms\":" << cut.observed_ms
        << ",\"health_observed_ms\":" << w.now_ms << ",\"health_valid\":" << (w.valid ? "true" : "false")
        << ",\"objects_native_return_blocked\":" << (blocked ? "true" : "false")
        << ",\"objects_receipt_returned\":" << (blocked ? "false" : "true")
        << ",\"objects_window\":"; counts(w.objects.window);
    std::cout << ",\"objects_total\":"; counts(w.objects.total);
    std::cout << ",\"profile_total\":"; counts(w.profile.total);
    std::cout << ",\"objects_attempt\":{\"capture_ms\":30960,\"put_before_ms\":31000}"
        << ",\"open_gap_ms\":" << *w.objects.open_gap_ms
        << ",\"processed_depth_frames\":" << d.processed_frames
        << ",\"roi_pixels\":" << d.roi_pixels << ",\"invalid_pixels\":" << d.invalid_pixels
        << ",\"put_before_ms\":" << audit.put_before_ms << ",\"put_return_ms\":" << audit.put_return_ms
        << ",\"packet_bytes\":" << sink.size << "}\n";
}

void blocked_return_and_independent_status(bool emit) {
    Rig r; r.now = 31000;
    ProcessEnvelope env{"robot_1", "abcdef01", 5000};
    BlockingSink object_sink;
    PublishResult object_result;
    std::atomic<bool> returned{false};
    // Objects' channel/cadence/clock are constructed and used ONLY by its owner.
    std::thread object_owner([&] {
        Clock owner_clock{r.now}; ObjectsCadence owner_cadence{0, 64};
        PublisherChannel channel{Topic::Objects, env, owner_clock, object_sink,
            4096, 8192, &owner_cadence, &r.o};
        object_result = channel.publish(object(30960));
        returned.store(true, std::memory_order_release);
    });
    object_sink.wait_entered();
    CHECK(!returned.load(std::memory_order_acquire));
    CHECK(r.o.generation() == 2 && r.o.published() == 1);
    Sink profile_sink; PublishResult pr;
    r.now = 31500;
    std::thread profile_owner([&] {
        Clock owner_clock{r.now};
        PublisherChannel channel{Topic::Profile, env, owner_clock, profile_sink,
            4096, 8192, nullptr, &r.p};
        pr = channel.publish(profile(31460));
    });
    profile_owner.join();
    CHECK(pr && pr.audit.attempted && pr.audit.seq == 2);
    r.now = 31501; const auto first = r.handoff.drain(r.clock);
    CHECK(first.code == HandoffCode::Ready && first.observed_ms == 31500);
    CHECK(first.events[0] == 2 && first.events[1] == 1);
    auto w = r.health.snapshot(first.observed_ms);
    CHECK(w.valid && w.objects.window.attempted == 1 && w.objects.window.pending == 1);
    CHECK(w.objects.total.unknown == 0 && w.profile.total.accepted == 1);
    CHECK(w.objects.latency_p50_ms == 40 && w.objects.open_gap_ms == 500);
    CHECK(r.cadence.snapshot(first.observed_ms).unique_results == 1);

    DepthHealthWindow depth{0, 64}; // REAL empty component, not a fabricated ratio DTO.
    HealthFieldContext context; context.extrinsic_calibrated = false; context.ground_seg_level = 0;
    Status status; status.degraded_reasons = {"roi_fallback", "no_traversable_seg", "infer_gap", "infer_rate_low"};
    Sink status_sink;
    PublisherChannel status_channel{Topic::Status, env, r.clock, status_sink, 4096, 8192};
    StatusHeartbeat heartbeat{status_channel, 31001, 10};
    constexpr const char* names[] = {"blocked_pending_1", "blocked_pending_2", "blocked_pending_3",
        "blocked_attempt_expired", "late_completion_total_only"};
    std::size_t beat = 0;
    const auto status_beat = [&](Millis now, bool blocked) {
        r.now = now; const auto cut = r.handoff.drain(r.clock);
        CHECK(cut.code == HandoffCode::Ready && cut.observed_ms == now - 1);
        const auto health = r.health.snapshot(cut.observed_ms);
        const auto cadence = r.cadence.snapshot(cut.observed_ms);
        const auto depth_report = depth.snapshot(cut.observed_ms);
        CHECK(health.valid && cadence.valid && !depth_report.invalid_ratio && !depth_report.ever_observed);
        CHECK(apply_health_fields(health, cadence, cut.observed_ms, depth_report, context, status) == HealthFieldsCode::Ready);
        CHECK(status.fps_depth == 0 && status.fps_infer == 0 && !status.invalid_pixel_ratio);
        CHECK(health.objects.total.attempted == 1 && health.objects.total.unknown == 0);
        CHECK(health.objects.total.pending == (blocked ? 1U : 0U));
        CHECK(health.objects.open_gap_ms == cut.observed_ms - 31000);
        if (cut.observed_ms >= 41000) {
            CHECK(health.objects.window.attempted == 0 && !status.latency_ms_p50 && !status.latency_ms_p99);
        } else CHECK(health.objects.window.pending == 1 && status.latency_ms_p50 == 40);
        CHECK(returned.load(std::memory_order_acquire) == !blocked);
        const auto h = heartbeat.poll(now, &status, cut.observed_ms);
        CHECK(h.code == HeartbeatCode::Published && h.publication.audit.attempted);
        CHECK(h.publication.audit.put_before_ms == now && h.publication.audit.put_return_ms == now);
        if (emit) emit_status(names[beat], status_sink, status_channel, cut, health, depth_report, h.publication.audit, blocked);
        ++beat;
    };
    for (const Millis now : {32001, 33001, 34001}) status_beat(now, true);
    r.now = 41000; const auto just_live = r.handoff.drain(r.clock);
    CHECK(just_live.code == HandoffCode::Ready && just_live.observed_ms == 40999);
    CHECK(r.health.snapshot(40999).objects.window.pending == 1);
    CHECK(r.cadence.snapshot(40999).unique_results == 1);
    r.now = 41001; const auto exactly_expired = r.handoff.drain(r.clock);
    CHECK(exactly_expired.code == HandoffCode::Ready && exactly_expired.observed_ms == 41000);
    const auto expired = r.health.snapshot(41000);
    CHECK(expired.objects.window.attempted == 0 && expired.objects.total.pending == 1 &&
        !expired.objects.latency_p99_ms && expired.objects.open_gap_ms == 10000);
    CHECK(r.cadence.snapshot(41000).unique_results == 1);
    status_beat(42001, true);
    CHECK(status_sink.calls.load() == 4 && !returned.load());
    r.now = 43000; object_sink.unblock(); object_owner.join();
    CHECK(returned.load() && object_result && object_result.audit.put_before_ms == 31000);
    CHECK(object_result.audit.put_return_ms == 43000 && object_sink.calls.load() == 1);
    r.now = 43001; const auto completion = r.handoff.drain(r.clock);
    CHECK(completion.code == HandoffCode::Ready && completion.events[1] == 1);
    w = r.health.snapshot(completion.observed_ms);
    CHECK(w.objects.total.accepted == 1 && w.objects.total.pending == 0 && w.objects.total.attempted == 1);
    CHECK(w.objects.window.attempted == 0 && !w.objects.latency_p99_ms && w.objects.open_gap_ms == 12000);
    CHECK(r.cadence.snapshot(completion.observed_ms).unique_results == 1);
    status_beat(44001, false); CHECK(beat == 5);
    ++groups;
}

void false_future_and_startup() {
    // Claiming a valid timestamp beyond the clock sampled AFTER the commit is
    // a protocol fault, not an event that may remain queued through Ready cuts.
    {
        Rig r; commit_attempt(r.o, attempt(1, 200)); r.now = 101;
        const auto c = r.handoff.drain(r.clock);
        CHECK(c.code == HandoffCode::Invalid && !r.handoff.valid() && r.o.consumed() == 0);
        CHECK(r.health.snapshot(100).objects.total.attempted == 0);
    }
    {
        Rig r; const auto a = attempt(1, 100); commit_attempt(r.o, a); complete(r.o, a, 200);
        r.now = 101; const auto c = r.handoff.drain(r.clock);
        CHECK(c.code == HandoffCode::Invalid && !r.handoff.valid() && r.o.consumed() == 1);
        const auto w = r.health.snapshot(100);
        CHECK(w.objects.total.attempted == 1 && w.objects.total.pending == 1 && w.objects.total.unknown == 0);
    }
    {
        std::atomic<Millis> now{100}; Clock clock{now};
        PublicationJournal p{Topic::Profile, 2}, o{Topic::Objects, 2};
        HealthWindow health{100, 8}; ObjectsCadence cadence{100, 8};
        PublicationHealthHandoff h{100, p, o, health, cadence};
        CHECK(h.drain(clock).code == HandoffCode::NotAdvanced && h.last_cut() == 99);
        now = 101; CHECK(h.drain(clock).code == HandoffCode::Ready && h.last_cut() == 100);
        CHECK(h.drain(clock).code == HandoffCode::NotAdvanced && h.valid());
        now = 100; CHECK(h.drain(clock).code == HandoffCode::Invalid && !h.valid());
    }
    ++groups;
}

struct GenerationClock final : PublishClock {
    PublicationJournal& journal; PublishAudit audit; Millis now; bool fire = true;
    GenerationClock(PublicationJournal& j, PublishAudit a, Millis n) : journal(j), audit(a), now(n) {}
    Millis wall_millis() noexcept override { return 1700000000000LL; }
    Millis mono_millis() noexcept override {
        if (fire) {
            fire = false;
            if (!journal.reserve()) std::abort();
            journal.begin_attempt(); journal.record_attempt(audit);
        }
        return now;
    }
};
void cuts_and_busy() {
    Rig r;
    CHECK(r.handoff.drain(r.clock).code == HandoffCode::NotAdvanced && r.handoff.last_cut() == -1);
    r.now = 100; const auto a = attempt(1, 100); commit_attempt(r.o, a); complete(r.o, a, 100);
    auto c = r.handoff.drain(r.clock);
    CHECK(c.code == HandoffCode::Ready && c.observed_ms == 99 && c.events[1] == 0);
    CHECK(r.o.consumed() == 0 && r.o.published() == 2);
    CHECK(r.health.snapshot(99).objects.total.attempted == 0);
    CHECK(r.cadence.snapshot(99).unique_results == 0);
    CHECK(r.handoff.drain(r.clock).code == HandoffCode::NotAdvanced);
    r.now = 101; c = r.handoff.drain(r.clock);
    CHECK(c.code == HandoffCode::Ready && c.observed_ms == 100 && c.events[1] == 2);
    CHECK(r.health.snapshot(100).objects.total.accepted == 1);
    CHECK(r.cadence.snapshot(100).historical_max_gap_ms == 0);

    CHECK(r.o.reserve()); r.o.begin_attempt(); r.now = 501;
    const auto head = r.o.published(), tail = r.o.consumed();
    CHECK(r.handoff.drain(r.clock).code == HandoffCode::Busy);
    CHECK(r.handoff.last_cut() == 100 && r.o.published() == head && r.o.consumed() == tail);
    CHECK(r.health.snapshot(100).objects.total.attempted == 1);
    CHECK(r.cadence.snapshot(100).historical_max_gap_ms == 0);
    r.o.cancel(); CHECK(r.o.valid() && !(r.o.generation() & 1U));
    // Independent replay: the event occurs DURING the clock read. A/B mismatch
    // must not snapshot cadence at 200, then retrospectively insert time 200.
    Rig changed; GenerationClock gc{changed.o, attempt(1, 200), 201};
    CHECK(changed.handoff.drain(gc).code == HandoffCode::Busy);
    CHECK(changed.handoff.last_cut() == -1 && changed.o.consumed() == 0);
    CHECK(changed.health.snapshot(0).objects.total.attempted == 0);
    CHECK(changed.cadence.snapshot(0).historical_max_gap_ms == 0);
    const auto restored = changed.handoff.drain(gc);
    CHECK(restored.code == HandoffCode::Ready && restored.observed_ms == 200 && restored.events[1] == 1);
    const auto hw = changed.health.snapshot(200); const auto cw = changed.cadence.snapshot(200);
    CHECK(hw.valid && hw.objects.total.pending == 1 && hw.objects.open_gap_ms == 0);
    CHECK(cw.valid && cw.historical_max_gap_ms == 0 && cw.unique_results + cw.excluded_results == 1);
    complete(changed.o, gc.audit, 201); gc.now = 202;
    CHECK(changed.handoff.drain(gc).code == HandoffCode::Ready);
    CHECK(changed.health.snapshot(201).objects.total.accepted == 1);
    ++groups;
}

struct PrepareSink final : Sink {
    enum Mode { Good, Reject, Throw, BadCopy } mode = Good;
    PutReceipt put_finalized(std::string_view k, char* p, std::size_t n, PublishQos q,
            PacketFinalizer& f) override {
        if (mode == Reject) return {PutOutcome::Rejected, 7};
        if (mode == Throw) throw 9;
        if (mode == BadCopy) { (void)f.finalize(nullptr, n); return {PutOutcome::Rejected, 8}; }
        return RnsTransport::put_finalized(k, p, n, q, f);
    }
};
void admission_and_cancellation() {
    std::atomic<Millis> now{100}; Clock clock{now}; Sink sink;
    ProcessEnvelope env{"robot_1", "abcdef01", 5000};
    PublicationJournal p{Topic::Profile, 2}, o{Topic::Objects, 2};
    ObjectsCadence own{0, 64}, mirror{0, 64}; HealthWindow health{0, 64};
    PublicationHealthHandoff handoff{0, p, o, health, mirror};
    PublisherChannel channel{Topic::Objects, env, clock, sink, 4096, 8192, &own, &o};
    const auto first = channel.publish(object(90)); CHECK(first && first.audit.seq == 1);
    now = 101; const auto rejected = channel.publish(object(91));
    CHECK(rejected.error == PublishError::TelemetryUnavailable && !rejected.audit.attempted);
    CHECK(sink.calls.load() == 1 && o.published() == 2 && o.counters().admission_rejected == 1);
    CHECK(o.valid() && !(o.generation() & 1U));
    CHECK(handoff.drain(clock).code == HandoffCode::Ready && o.consumed() == 2);
    const auto retry = channel.publish(object(91));
    CHECK(retry && retry.audit.seq == 2 && retry.audit.capture_ms == 91 && sink.calls.load() == 2);

    for (const auto mode : {PrepareSink::Reject, PrepareSink::Throw, PrepareSink::BadCopy, PrepareSink::Good}) {
        PublicationJournal journal{Topic::Objects, 2}; ObjectsCadence cadence{0, 8};
        ProcessEnvelope e{"robot_1", "abcdef01", 5000}; PrepareSink prep; prep.mode = mode;
        PublisherChannel pub{Topic::Objects, e, clock, prep, 4096, 8192, &cadence, &journal};
        now = mode == PrepareSink::Good ? 89 : 100;
        const auto failure = pub.publish(object(90));
        CHECK(!failure.audit.attempted && prep.calls.load() == 0);
        CHECK(failure.error == (mode == PrepareSink::Reject ? PublishError::PrepareRejected :
            mode == PrepareSink::Throw ? PublishError::PrepareUnknown :
            mode == PrepareSink::BadCopy ? PublishError::Internal : PublishError::ClockBefore));
        CHECK(journal.valid() && !(journal.generation() & 1U) && journal.published() == 0);
        prep.mode = PrepareSink::Good; now = 101;
        const auto good = pub.publish(object(90)); CHECK(good && good.audit.seq == 1 && prep.calls.load() == 1);
        CHECK(journal.published() == 2 && !(journal.generation() & 1U));
    }
    PublicationJournal wrong{Topic::Profile, 2}; ObjectsCadence local{0, 8};
    PublisherChannel wrong_topic{Topic::Objects, env, clock, sink, 4096, 8192, &local, &wrong};
    CHECK(!wrong_topic.valid() && wrong_topic.publish(object(99)).error == PublishError::Configuration);
    PublisherChannel wrong_status{Topic::Status, env, clock, sink, 4096, 8192, nullptr, &wrong};
    CHECK(!wrong_status.valid());
    PublicationHealthHandoff swapped{0, o, p, health, mirror}; CHECK(!swapped.valid());
    ++groups;
}

struct ReturnClock final : PublishClock {
    unsigned mono_reads = 0; Millis after;
    explicit ReturnClock(Millis value) : after(value) {}
    Millis wall_millis() noexcept override { return 1700000000000LL; }
    Millis mono_millis() noexcept override { return ++mono_reads == 1 ? 100 : after; }
};
void invalid_completion_clock() {
    for (const Millis after : {Millis{-1}, Millis{99}, Millis{ProcessEnvelope::kMaxMonoMs + 1}}) {
        Rig r; ReturnClock bad_clock{after}; Sink sink;
        ProcessEnvelope env{"robot_1", "abcdef01", 5000}; ObjectsCadence own{0, 8};
        PublisherChannel channel{Topic::Objects, env, bad_clock, sink, 4096, 8192, &own, &r.o};
        const auto result = channel.publish(object(90));
        CHECK(result.error == PublishError::ClockAfter && result.audit.attempted && !result.audit.clock_valid);
        CHECK(result.audit.receipt.outcome == PutOutcome::Accepted && sink.calls.load() == 1);
        CHECK(r.o.valid() && !(r.o.generation() & 1U));
        r.now = 101; const auto cut = r.handoff.drain(r.clock);
        CHECK(cut.code == HandoffCode::Invalid && !r.handoff.valid());
        CHECK(cut.events[1] == 2 && r.o.consumed() == 2);
        const auto w = r.health.snapshot(100);
        CHECK(!w.valid && w.diagnostics.clock_faults == 1);
        CHECK(w.objects.total.attempted == 1 && w.objects.total.accepted == 1 &&
            w.objects.total.pending == 0 && w.objects.total.unknown == 0);
        CHECK(w.objects.window.attempted == 1 && w.objects.latency_p99_ms == 10);
        CHECK(r.handoff.drain(r.clock).code == HandoffCode::Invalid);
        CHECK(r.health.snapshot(100).objects.total.accepted == 1);
    }
    ++groups;
}
void no_allocation_hot_path() {
    Rig r; ProcessEnvelope env{"robot_1", "abcdef01", 5000}; Sink sink;
    ObjectsCadence owner{0, 64};
    PublisherChannel objects{Topic::Objects, env, r.clock, sink, 4096, 8192, &owner, &r.o};
    PublisherChannel status_channel{Topic::Status, env, r.clock, sink, 4096, 8192};
    DepthHealthWindow depth{0, 64}; HealthFieldContext context;
    context.extrinsic_calibrated = false; context.ground_seg_level = 0;
    Status status; status.degraded_reasons = {"roi_fallback", "no_traversable_seg", "infer_gap", "infer_rate_low"};
    StatusHeartbeat heartbeat{status_channel, 0, 10};
    Objects body = object(90);
    const auto before = allocation_count.load(std::memory_order_relaxed);
    for (Millis i = 0; i < 20; ++i) {
        const Millis now = 100 + 20 * i; r.now = now; body.t_capture_mono_ms = now - 10;
        CHECK(objects.publish(body)); r.now = now + 1;
        const auto cut = r.handoff.drain(r.clock); CHECK(cut.code == HandoffCode::Ready);
        const auto w = r.health.snapshot(cut.observed_ms); const auto c = r.cadence.snapshot(cut.observed_ms);
        const auto d = depth.snapshot(cut.observed_ms);
        CHECK(apply_health_fields(w, c, cut.observed_ms, d, context, status) == HealthFieldsCode::Ready);
    }
    r.now = 1001; const auto cut = r.handoff.drain(r.clock);
    const auto w = r.health.snapshot(cut.observed_ms); const auto c = r.cadence.snapshot(cut.observed_ms);
    const auto d = depth.snapshot(cut.observed_ms);
    CHECK(apply_health_fields(w, c, cut.observed_ms, d, context, status) == HealthFieldsCode::Ready);
    CHECK(heartbeat.poll(1001, &status, cut.observed_ms).code == HeartbeatCode::Published);
    CHECK(allocation_count.load(std::memory_order_relaxed) == before);
    ++groups;
}
} // namespace
int main(int argc, char** argv) {
    const bool emit = argc == 2 && std::string_view(argv[1]) == "--emit-fixtures";
    if (argc > 2 || (argc == 2 && !emit)) return 2;
    blocked_return_and_independent_status(emit);
    cuts_and_busy(); false_future_and_startup(); admission_and_cancellation();
    invalid_completion_clock(); no_allocation_hot_path();
    if (!emit) std::cout << "PASS publication_handoff groups=" << groups << " checks=" << checks
        << " injected-only; no device/network/production/Hz claim\n";
}
