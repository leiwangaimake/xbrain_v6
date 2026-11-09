/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_objects_delivery.cpp
 * Brief: Host test of the Objects callback's delivery step with and without the paced release.
 *
 * Description:
 * The callback's order per inference frame is: S evidence into the semantic slot, the Objects
 * body on (deliver_objects), then the T mask. Checked here with the real SemanticSlot, the real
 * ObjectsRelease and a real Objects channel behind an injected clock and transport:
 *   - without a release the body is published at once and the publish's own result is returned;
 *   - with one the call returns without waiting: the frame's S snapshot is readable, with the
 *     frame's own capture stamp, while its Objects body still waits, and the step after the
 *     delivery (the T mask in the service) runs before anything is on the wire;
 *   - an Objects body that is overwritten, rejected or of unknown outcome leaves its S snapshot
 *     in place; the slot's own refusals (another clock epoch) do not depend on the release;
 *   - a body refused because the service is stopping is not a failure; a faulted release, a
 *     frame without a ready time and a failed synchronous publish are.
 * (An S snapshot that expires between the Profile's check and its put is covered by
 * test_rns_native_profile and test_rns_semantic_profile; that code is not touched here.)
 * Stop and faults (2026-10-11), with the functions the service's main program itself calls
 * (objects_delivery.hpp): the owner's counting for every publish outcome; a contract fault on
 * the last frame seen by the main loop's poll without another frame; a put in progress at a
 * normal stop that returns a contract fault makes the exit non-zero, one that returns a
 * transport outcome leaves it clean; a stop between the publication commitment and the channel
 * entry; the conservations of a stopped run.
 */

#include "service/objects_delivery.hpp"
#include "service/semantic_slot.hpp"
#include "output/rns_publication_journal.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace perception::rns;
using perception::service::SemanticSlot;
using perception::service::deliver_objects;
using perception::service::join_objects_release;
using perception::service::merge_release_fault;
using perception::service::publish_counted;
using perception::service::release_fault;
using perception::service::transport_outcome;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

constexpr std::int64_t kMs = 1000000;

struct Clock final : PublishClock {
    std::atomic<std::int64_t>& now_ns;
    explicit Clock(std::atomic<std::int64_t>& n) : now_ns(n) {}
    Millis wall_millis() noexcept override { return 1700000000000LL; }
    Millis mono_millis() noexcept override { return now_ns.load() / kMs; }
};
// Counts puts; the first put can be held, and the receipt chosen.
struct Sink final : RnsTransport {
    std::atomic<std::size_t> calls{0};
    std::atomic<PutOutcome> outcome{PutOutcome::Accepted};
    std::mutex gate;
    std::condition_variable cv;
    bool hold_first = false, entered = false, release = false;
    std::function<void()> before_return;  // runs inside every put, after a held one was let go
    PutReceipt put(std::string_view, std::string_view, PublishQos) override {
        const bool first = calls.fetch_add(1) == 0;
        if (first && hold_first) {
            std::unique_lock<std::mutex> lock(gate);
            entered = true;
            cv.notify_all();
            cv.wait_for(lock, std::chrono::seconds(20), [&] { return release; });
        }
        if (before_return) before_return();
        return {outcome.load(), 0};
    }
    void wait_entered() {
        std::unique_lock<std::mutex> lock(gate);
        cv.wait_for(lock, std::chrono::seconds(20), [&] { return entered; });
    }
    void unblock() {
        std::lock_guard<std::mutex> lock(gate);
        release = true;
        cv.notify_all();
    }
};

// One located object: enough for an S snapshot and a valid Objects body.
Objects body(Millis capture, std::int64_t track) {
    Objects m;
    m.t_capture_mono_ms = capture;
    Object o;
    o.track_id = track;
    o.class_id = 3;
    o.class_name = "chair";
    o.confidence = 0.8F;
    o.semantic_status = "confirmed";
    o.geometry.emplace(std::vector<Point>{{0.6F, 0.0F}, {0.7F, 0.0F}, {0.7F, 0.1F}, {0.6F, 0.1F}}, 0.0F, 0.5F);
    o.velocity_frame = "raw";
    o.velocity_status = "warming_up";
    o.depth_quality = "good";
    o.first_seen_mono_ms = capture;
    o.last_seen_mono_ms = capture;
    m.objects.push_back(o);
    return m;
}
perception::geometry::CaptureStamp stamp_of(std::uint64_t sequence, Millis capture_ms, std::uint64_t epoch = 1) {
    perception::geometry::CaptureStamp s;
    s.source.stream_id = 7;
    s.source.sequence = sequence;
    s.clock_epoch = epoch;
    s.capture_mono_ns = capture_ms * kMs;
    return s;
}

// The service's wiring around the delivery step, with the real slot, release and channel.
struct Rig {
    std::atomic<std::int64_t> now{1000 * kMs};
    Clock clock{now};
    Sink sink;
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublicationJournal journal{Topic::Objects, 256};
    ObjectsCadence cadence{0, 256};
    PublisherChannel channel{Topic::Objects, envelope, clock, sink, 8192, 16384, &cadence, &journal};
    SemanticSlot slot;
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<ReleaseRecord> records;
    // The Objects owner's counters and its one publish (ObjectsOwner::publish_result in the service).
    perception::service::ObjectsPublishCounters owner;
    std::function<void()> on_clock;  // inside every read of the release's clock; set before start()
    ObjectsRelease release{43 * kMs, [this] { if (on_clock) on_clock(); return now.load(); },
                           [this](const Objects& b) { return publish_counted(channel, owner, b); },
                           [this](const ReleaseRecord& r) {
                               std::lock_guard<std::mutex> lock(mutex);
                               records.push_back(r);
                               cv.notify_all();
                           }};
    int after_delivery = 0;  // the callback's next step (the T mask in the service)
    // One Objects callback in the service's order.
    bool callback(ObjectsRelease* paced, Millis capture, std::uint64_t frame, std::int64_t ready_ns) {
        const Objects b = body(capture, static_cast<std::int64_t>(frame));
        slot.publish(b, stamp_of(frame, capture), perception::service::kSemanticBaseRevision);
        // Without a release: the owner's synchronous publish (ObjectsOwner::publish in the service).
        const bool delivered = deliver_objects(
            paced, [this](const Objects& x) { return transport_outcome(publish_counted(channel, owner, x)); }, b, frame,
            ready_ns);
        ++after_delivery;
        return delivered;
    }
    void set(std::int64_t ns) {
        now.store(ns);
        release.wake();
    }
    std::vector<ReleaseRecord> wait_records(std::size_t n) {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait_for(lock, std::chrono::seconds(20), [&] { return records.size() >= n; });
        return records;
    }
    // The main loop's poll (perception_service_main.cpp), for up to two seconds.
    bool poll_fault(std::string& reason) {
        for (int i = 0; i < 2000; ++i) {
            if (release_fault(&release, reason)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }
    // The conservations of a stopped run (after join_objects_release).
    void conserved(const ReleaseCounters& c) {
        CHECK(c.offered == c.admitted + c.refused_closed + c.refused_faulted + c.refused_invalid + c.refused_copy);
        CHECK(c.unaccounted() == 0 && c.pending == 0 && c.in_flight == 0);
        CHECK(c.committed == c.published + c.channel_refused);
        // Every committed call is one call of the channel by the owner; the channel attempted a
        // put for the published ones only, and the journal holds an attempt and a completion each.
        CHECK(owner.channel_calls.load() == c.published + c.channel_refused);
        CHECK(c.published == c.accepted + c.rejected + c.unknown && channel.counters().attempted == c.published);
        CHECK(journal.published() == 2 * c.published && journal.generation() % 2 == 0);
    }
    // What a Profile frame captured at depth_ms would be offered from the slot.
    const SemanticFrame* latest(SemanticSlot::Snapshot& into, Millis depth_ms, std::uint64_t epoch = 1) {
        return slot.latest(into, stamp_of(900, depth_ms, epoch));
    }
};
}  // namespace

int main() {
    // ---- without a release: published at once, the publish's own result is returned ----
    {
        Rig r;
        r.set(1010 * kMs);
        CHECK(r.callback(nullptr, 960, 1, 1000 * kMs));
        CHECK(r.sink.calls.load() == 1 && r.after_delivery == 1 && r.release.counters().offered == 0);
        // A repeated capture is a local fault of the synchronous publish: reported as a failure.
        CHECK(!r.callback(nullptr, 960, 2, 1000 * kMs));
        CHECK(r.sink.calls.load() == 1 && r.channel.counters().duplicates == 1);
        // A frame without a ready time is fine on this path: nothing reads it.
        CHECK(r.callback(nullptr, 993, 3, 0) && r.sink.calls.load() == 2);
    }
    // ---- paced: the call does not wait; S is readable while the Objects body waits ----
    {
        Rig r;
        CHECK(r.release.start());
        r.set(1010 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));
        // Returned at the same clock value with nothing on the wire, and the next step ran.
        CHECK(r.now.load() == 1010 * kMs && r.sink.calls.load() == 0 && r.after_delivery == 1);
        auto c = r.release.counters();
        CHECK(c.admitted == 1 && c.pending == 1 && c.published == 0);
        // A Profile frame captured at 1015 ms gets this frame's S evidence, stamped with the
        // inference frame's own capture (960 ms), not with the hand-over or the release time.
        SemanticSlot::Snapshot snapshot;
        const auto* frame = r.latest(snapshot, 1015);
        CHECK(frame && frame->count == 1 && frame->stamp.source.sequence == 1);
        CHECK(frame->stamp.capture_mono_ns == 960 * kMs && r.sink.calls.load() == 0);
        r.set(1043 * kMs);
        const auto records = r.wait_records(1);
        CHECK(records.size() == 1 && records[0].end == ReleaseEnd::Published && records[0].result.audit.put_before_ms == 1043);
        // The Objects publish did not touch the slot.
        frame = r.latest(snapshot, 1050);
        CHECK(frame && frame->stamp.capture_mono_ns == 960 * kMs && r.slot.published() == 1);
        // Another clock epoch is refused by the slot itself, whatever the release is doing.
        CHECK(r.latest(snapshot, 1050, 2) == nullptr && r.slot.refused_epoch() == 1);
        r.release.close();
        r.release.join();
    }
    // ---- an overwritten Objects body keeps its S evidence; so do rejected and unknown puts ----
    {
        Rig r;
        r.sink.hold_first = true;
        CHECK(r.release.start());
        r.set(1050 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));  // late: enters the put, which is held
        r.sink.wait_entered();
        r.set(1060 * kMs);
        CHECK(r.callback(&r.release, 993, 2, 1033 * kMs));
        SemanticSlot::Snapshot snapshot;
        const auto* frame = r.latest(snapshot, 1065);
        CHECK(frame && frame->stamp.source.sequence == 2);  // S of frame 2 is there...
        r.set(1070 * kMs);
        CHECK(r.callback(&r.release, 1026, 3, 1066 * kMs));
        auto c = r.release.counters();
        CHECK(c.overwritten == 1 && c.in_flight == 1 && c.pending == 1);  // ...its Objects body never goes out
        frame = r.latest(snapshot, 1075);
        CHECK(frame && frame->stamp.source.sequence == 3 && r.slot.published() == 3 && r.after_delivery == 3);
        r.sink.outcome = PutOutcome::Rejected;
        r.sink.unblock();
        r.set(1109 * kMs);
        const auto records = r.wait_records(3);
        CHECK(records.size() == 3 && r.sink.calls.load() == 2);
        c = r.release.counters();
        CHECK(c.published == 2 && c.rejected == 2 && c.overwritten == 1 && c.unaccounted() == 0 && !r.release.faulted());
        // Two rejected puts and an overwritten body later the slot still holds frame 3's evidence.
        frame = r.latest(snapshot, 1110);
        CHECK(frame && frame->stamp.source.sequence == 3 && frame->stamp.capture_mono_ns == 1026 * kMs);
        r.sink.outcome = PutOutcome::Unknown;
        r.set(1200 * kMs);
        CHECK(r.callback(&r.release, 1100, 4, 1150 * kMs));
        (void)r.wait_records(4);
        frame = r.latest(snapshot, 1201);
        CHECK(frame && frame->stamp.source.sequence == 4 && r.release.counters().unknown == 1 && !r.release.faulted());
        r.release.close();
        r.release.join();
    }
    // ---- stopping is not a failure; S of that frame is still written ----
    {
        Rig r;
        CHECK(r.release.start());
        r.release.close();
        r.set(1010 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));
        const auto c = r.release.counters();
        CHECK(c.offered == 1 && c.refused_closed == 1 && c.admitted == 0 && r.sink.calls.load() == 0);
        CHECK(r.slot.published() == 1 && r.after_delivery == 1 && !r.release.faulted());
        r.release.join();
    }
    // ---- local faults are failures: no ready time, then every later frame ----
    {
        Rig r;
        CHECK(r.release.start());
        r.set(1010 * kMs);
        CHECK(!r.callback(&r.release, 960, 1, 0));
        CHECK(r.release.faulted() && r.release.counters().refused_invalid == 1);
        CHECK(!r.callback(&r.release, 993, 2, 1005 * kMs));
        CHECK(r.release.counters().refused_faulted == 1 && r.sink.calls.load() == 0);
        // S was written for both frames before the delivery step failed (the callback's order).
        CHECK(r.slot.published() == 2);
        r.release.join();
    }
    // ---- the owner's counting without a release (delay "0"): every outcome as it always was ----
    {
        Rig r;
        r.set(1010 * kMs);
        CHECK(r.callback(nullptr, 960, 1, 0));                      // accepted
        r.sink.outcome = PutOutcome::Rejected;
        r.set(1043 * kMs);
        CHECK(r.callback(nullptr, 993, 2, 0));                      // rejected: counted, not a failure
        r.sink.outcome = PutOutcome::Unknown;
        r.set(1076 * kMs);
        CHECK(r.callback(nullptr, 1026, 3, 0));                     // unknown: counted, not a failure
        r.sink.outcome = PutOutcome::Accepted;
        r.set(1109 * kMs);
        CHECK(!r.callback(nullptr, 1026, 4, 0));                    // repeated capture: a contract fault
        CHECK(r.owner.accepted.load() == 1 && r.owner.rejected.load() == 1 && r.owner.unknown.load() == 1);
        // Four calls of the channel, three put attempts: channel_calls is not the attempt count.
        CHECK(r.owner.channel_calls.load() == 4 && r.channel.counters().attempted == 3 && r.sink.calls.load() == 3);
        // The rule itself, on the channel's own results.
        bool fault = true;
        r.set(1142 * kMs);
        auto result = publish_counted(r.channel, r.owner, body(1100, 5), &fault);
        CHECK(result && !fault && transport_outcome(result) && r.owner.accepted.load() == 2);
        result = publish_counted(r.channel, r.owner, body(1100, 6), &fault);
        CHECK(!result && fault && !transport_outcome(result) && result.error == PublishError::Duplicate);
        CHECK(r.owner.channel_calls.load() == 6 && r.owner.accepted.load() == 2);
        // No release: nothing to poll, nothing to merge at a stop.
        std::string reason = "signal 15";
        CHECK(!release_fault(nullptr, reason) && reason == "signal 15");
        const auto summary = join_objects_release(nullptr);
        CHECK(!summary.present && !summary.faulted);
        CHECK(merge_release_fault(0, reason, summary, 70) == 0 && reason == "signal 15");
    }
    // ---- the last frame is a contract fault and no frame follows: the main loop still sees it ----
    {
        Rig r;
        CHECK(r.release.start());
        r.set(1050 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));
        (void)r.wait_records(1);
        std::string reason;
        CHECK(!release_fault(&r.release, reason) && reason.empty());
        r.set(1100 * kMs);
        CHECK(r.callback(&r.release, 960, 2, 1050 * kMs));  // handed over; its capture repeats frame 1's
        CHECK(r.poll_fault(reason));                        // no further callback
        CHECK(reason.find("publish contract fault") != std::string::npos && reason.find("frame_id=2") != std::string::npos);
        // The service's stop after such a fault: the exit code and the reason stay the main loop's.
        int exit_code = 70;
        r.release.close();
        const auto summary = join_objects_release(&r.release);
        CHECK(summary.present && summary.faulted && summary.reason == reason);
        const std::string before = reason;
        CHECK(merge_release_fault(exit_code, reason, summary, 70) == 70 && reason == before);
        CHECK(summary.counters.contract_faults == 1 && summary.counters.channel_refused == 1 && summary.counters.published == 1);
        r.conserved(summary.counters);
        // A later callback is a failure (the release is faulted), as the Supervisor expects.
        CHECK(!r.callback(&r.release, 1059, 3, 1090 * kMs));
    }
    // ---- a put in progress at a normal stop returns a contract fault: the exit is not clean ----
    {
        Rig r;
        r.sink.hold_first = true;
        r.sink.before_return = [&] { r.now.store(900 * kMs); };  // the clock steps back inside the put
        CHECK(r.release.start());
        r.set(1050 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));
        r.sink.wait_entered();
        // The main loop saw no fault and left on a signal.
        std::string reason;
        CHECK(!release_fault(&r.release, reason));
        reason = "signal 15";
        int exit_code = 0;
        r.release.close();                                       // stop, first step
        CHECK(r.callback(&r.release, 993, 2, 1033 * kMs));       // the pipeline's last frame: refused as Closed, not a failure
        r.sink.unblock();                                        // the put returns during the stop
        const auto summary = join_objects_release(&r.release);  // stop, second step
        CHECK(summary.present && summary.faulted);
        CHECK(summary.reason.find("publish contract fault") != std::string::npos);
        exit_code = merge_release_fault(exit_code, reason, summary, 70);
        CHECK(exit_code == 70);
        CHECK(reason.find("signal 15") == 0 && reason.find("objects release fault at stop") != std::string::npos &&
              reason.find(summary.reason) != std::string::npos);
        const auto records = r.wait_records(1);
        CHECK(records.size() == 1 && records[0].end == ReleaseEnd::Published && records[0].result.error == PublishError::ClockAfter);
        CHECK(summary.counters.published == 1 && summary.counters.contract_faults == 1 && summary.counters.refused_closed == 1);
        r.conserved(summary.counters);
        // Merging twice does not repeat the reason; an earlier exit code is kept.
        const std::string once = reason;
        CHECK(merge_release_fault(78, reason, summary, 70) == 78 && reason == once);
    }
    // ---- the same stop with a put that returns a transport outcome: a clean exit ----
    for (const PutOutcome outcome : {PutOutcome::Accepted, PutOutcome::Rejected, PutOutcome::Unknown}) {
        Rig r;
        r.sink.hold_first = true;
        r.sink.outcome = outcome;
        CHECK(r.release.start());
        r.set(1050 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));
        r.sink.wait_entered();
        r.set(1060 * kMs);
        CHECK(r.callback(&r.release, 993, 2, 1033 * kMs));       // pending: discarded by the stop
        std::string reason = "signal 15";
        r.release.close();
        CHECK(r.callback(&r.release, 1026, 3, 1066 * kMs));      // refused as Closed: not a failure
        r.set(1070 * kMs);
        r.sink.unblock();
        const auto summary = join_objects_release(&r.release);
        CHECK(summary.present && !summary.faulted && summary.reason.empty());
        CHECK(merge_release_fault(0, reason, summary, 70) == 0 && reason == "signal 15");
        const auto& c = summary.counters;
        CHECK(c.offered == 3 && c.admitted == 2 && c.refused_closed == 1 && c.published == 1 && c.stop_discarded == 1);
        CHECK(c.accepted == (outcome == PutOutcome::Accepted) && c.rejected == (outcome == PutOutcome::Rejected) &&
              c.unknown == (outcome == PutOutcome::Unknown));
        CHECK(r.owner.accepted.load() == c.accepted && r.owner.rejected.load() == c.rejected && r.owner.unknown.load() == c.unknown);
        r.conserved(c);
        // Every frame's S evidence was written, whatever became of its Objects body.
        CHECK(r.slot.published() == 3 && r.after_delivery == 3);
    }
    // ---- stop between the publication commitment and the channel entry ----
    {
        Rig r;
        std::mutex gate;
        std::condition_variable cv;
        bool reached = false, opened = false;
        const auto producer = std::this_thread::get_id();
        std::atomic<int> release_reads{0};
        // The release thread reads its clock for the deadline check (under its lock) and then,
        // after the commitment, for the call time: hold it there, before the channel.
        r.on_clock = [&] {
            if (std::this_thread::get_id() == producer || release_reads.fetch_add(1) + 1 != 2) return;
            std::unique_lock<std::mutex> lock(gate);
            reached = true;
            cv.notify_all();
            cv.wait_for(lock, std::chrono::seconds(20), [&] { return opened; });
        };
        CHECK(r.release.start());
        r.set(1050 * kMs);
        CHECK(r.callback(&r.release, 960, 1, 1000 * kMs));
        {
            std::unique_lock<std::mutex> lock(gate);
            CHECK(cv.wait_for(lock, std::chrono::seconds(20), [&] { return reached; }));
        }
        CHECK(r.sink.calls.load() == 0 && r.owner.channel_calls.load() == 0);
        std::string reason = "signal 15";
        r.release.close();                                       // returns: the committed call is not waited for
        CHECK(r.callback(&r.release, 993, 2, 1033 * kMs));       // Closed has its own account, no failure
        auto c = r.release.counters();
        CHECK(c.in_flight == 1 && c.refused_closed == 1 && c.stop_discarded == 0 && r.sink.calls.load() == 0);
        {
            std::lock_guard<std::mutex> lock(gate);
            opened = true;
            cv.notify_all();
        }
        const auto summary = join_objects_release(&r.release);
        CHECK(!summary.faulted && merge_release_fault(0, reason, summary, 70) == 0 && reason == "signal 15");
        // The one committed call was made after close() had returned, once.
        CHECK(r.sink.calls.load() == 1 && summary.counters.published == 1 && summary.counters.accepted == 1);
        CHECK(summary.counters.offered == 2 && summary.counters.refused_closed == 1);
        r.conserved(summary.counters);
    }
    if (failures) {
        std::cerr << "test_objects_delivery: " << failures << " of " << checks << " checks failed\n";
        return 1;
    }
    std::cout << "test_objects_delivery: PASS checks=" << checks << "\n";
    return 0;
}
