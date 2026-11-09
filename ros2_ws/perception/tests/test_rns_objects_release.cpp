/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_objects_release.cpp
 * Brief: Paced Objects release through its real thread, slot, channel, journal and cadence.
 *
 * Description:
 * Injected clock and transports only: no camera, network or timing claim. Every case runs the
 * real release thread against a real PublisherChannel. What is checked:
 *   release rule   r + D; late bodies; latest wins (19 P19-2, 2026-10-16): a body not selected
 *                  yet is replaced by the next hand-over, also while the release thread is held
 *                  with no put in progress (GPT question 18.1's case: only the newest goes out),
 *                  and the newcomer takes the replaced body's due over, so a fast host is not
 *                  starved (every body out when the due comes first, every other body when the
 *                  next hand-over comes first); Late is judged against the due a body carries;
 *   times          offer_begin_ns at the start of offer(), handoff_ns under the lock at the
 *                  hand-over itself, also when the clock crosses the due in between; a clock that
 *                  fails or goes back at the hand-over is refused;
 *   stop           close() itself ends what has no commitment (the pending body); a committed
 *                  body gets its one call even when close() returns
 *                  before the channel is entered; a put in progress returns and is accounted
 *                  once; offers after close are refused;
 *   faults         a channel refusal or a broken return clock is latched without any further
 *                  input; rejected, unknown and throwing transports are counted, not retried;
 *   ownership      the wire bytes equal those of a direct publish of the same body at the same
 *                  clock although the source body is changed and destroyed before the release,
 *                  up to the 512-object limit; no allocation for bodies of one shape;
 *   Status         a body waiting for its release shows as an open gap and the journal is not
 *                  busy; a blocked put shows as a pending attempt; the journal is busy only while
 *                  the release thread is inside the channel's stamp-and-commit step;
 *   accounting     every admitted body has exactly one end, under a concurrent producer, a slow
 *                  transport and a stop from a third thread.
 */

#include "output/rns_objects_release.hpp"
#include "output/rns_health_window.hpp"
#include "output/rns_publication_handoff.hpp"
#include "output/rns_publication_journal.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

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
std::atomic<std::size_t> checks{0};
#define CHECK(x) do { checks.fetch_add(1, std::memory_order_relaxed); if (!(x)) { std::cerr << __FILE__ << ':' << __LINE__ \
    << ": " #x "\n"; std::abort(); } } while (false)

constexpr std::int64_t kMs = 1000000;
constexpr std::int64_t kDelay = 43 * kMs;
// The service's Objects buffers (perception_service_main.cpp).
constexpr std::size_t kPacketCapacity = 262144, kBodyCapacity = kPacketCapacity - 4096;

struct Clock final : PublishClock {
    std::atomic<std::int64_t>& now_ns;
    std::function<void()> on_mono;  // runs inside every monotonic read (set before the thread starts)
    explicit Clock(std::atomic<std::int64_t>& n) : now_ns(n) {}
    Millis wall_millis() noexcept override { return 1700000000000LL; }
    Millis mono_millis() noexcept override {
        if (on_mono) on_mono();
        return now_ns.load(std::memory_order_seq_cst) / kMs;
    }
};

// Keeps every packet. outcome / throws choose the receipt; before_return runs inside the put.
struct Sink : RnsTransport {
    std::mutex mutex;
    std::vector<std::string> packets;
    std::atomic<std::size_t> calls{0};
    std::atomic<PutOutcome> outcome{PutOutcome::Accepted};
    std::atomic<bool> throws{false};
    std::function<void()> before_return;
    PutReceipt put(std::string_view, std::string_view p, PublishQos) override {
        {
            std::lock_guard<std::mutex> lock(mutex);
            packets.emplace_back(p);
        }
        calls.fetch_add(1, std::memory_order_release);
        if (before_return) before_return();
        if (throws.load()) throw std::runtime_error("transport");
        return {outcome.load(), 0};
    }
    std::string packet(std::size_t i) {
        std::lock_guard<std::mutex> lock(mutex);
        return i < packets.size() ? packets[i] : std::string();
    }
};
// The first put blocks until unblock(); later puts return at once.
struct BlockingSink final : Sink {
    std::mutex gate;
    std::condition_variable cv;
    bool entered = false, release = false;
    PutReceipt put(std::string_view k, std::string_view p, PublishQos q) override {
        const auto receipt = Sink::put(k, p, q);
        std::unique_lock<std::mutex> lock(gate);
        entered = true;
        cv.notify_all();
        if (!cv.wait_for(lock, std::chrono::seconds(20), [&] { return release; })) std::abort();
        return receipt;
    }
    void wait_entered() {
        std::unique_lock<std::mutex> lock(gate);
        CHECK(cv.wait_for(lock, std::chrono::seconds(20), [&] { return entered; }));
    }
    void unblock() {
        std::lock_guard<std::mutex> lock(gate);
        release = true;
        cv.notify_all();
    }
};

// A valid body of n objects in the three shapes the contract allows: located with a velocity,
// located without, unlocatable. name_bytes lengthens the class names.
Objects body(Millis capture, std::size_t n = 2, std::size_t name_bytes = 0) {
    Objects m;
    m.t_capture_mono_ms = capture;
    m.extrinsic_calibrated = true;
    for (std::size_t i = 0; i < n; ++i) {
        Object o;
        o.track_id = static_cast<std::int64_t>(i) + 1;
        o.class_id = static_cast<int>(i % 80);
        o.class_name = "class_" + std::to_string(i) + std::string(name_bytes, static_cast<char>('a' + i % 26));
        o.confidence = 0.25F + 0.001F * static_cast<float>(i % 500);
        o.semantic_status = i % 2 ? "proxy" : "confirmed";
        o.velocity_frame = "raw";
        o.depth_quality = "good";
        o.stable_frames = i;
        o.first_seen_mono_ms = capture > 100 ? capture - 100 : 0;
        o.last_seen_mono_ms = capture;
        const float x = 1.0F + static_cast<float>(i % 7);
        if (i % 3 == 0) {
            o.geometry = ObjectGeometry{{{x, -1}, {x + 1, -1}, {x + 1, 1}, {x, 1}}, 0.1F, 1.5F};
            o.velocity_xy = std::array<float, 2>{0.5F, -0.125F};
            o.velocity_frame = "ego_removed";
            o.velocity_status = "moving";
            o.velocity_valid = true;
        } else if (i % 3 == 1) {
            o.geometry = ObjectGeometry{{{x, 0}, {x + 1, 0}, {x, 1}}, -0.2F, 0.3F};
            o.velocity_status = "warming_up";
            o.depth_quality = "fair";
        } else {
            o.velocity_status = "timestamp_gap";
            o.depth_quality = "poor";
        }
        m.objects.push_back(std::move(o));
    }
    return m;
}

// Real channel, journal and cadence behind a real release thread; the clock is set by the test.
struct Rig {
    std::atomic<std::int64_t> now{0};
    Clock clock{now};
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublicationJournal journal{Topic::Objects, 4096};
    ObjectsCadence cadence{0, 1024};
    PublisherChannel channel;
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<ReleaseRecord> records;
    std::function<void(const ReleaseRecord&)> on_record;  // runs on the release thread, before the record is stored
    // Runs inside every read of the release's own clock, before the value is taken, on whichever
    // thread reads (offer(): twice, the second time with the release's lock held; the release
    // thread: with the lock held for a deadline check, without it for call and return times).
    // Set before start().
    std::function<void()> on_clock;
    ObjectsRelease release;
    explicit Rig(RnsTransport& sink, std::int64_t start_ns = 1000 * kMs, std::int64_t delay_ns = kDelay)
        : channel{Topic::Objects, envelope, clock, sink, kBodyCapacity, kPacketCapacity, &cadence, &journal},
          release{delay_ns, [this] { if (on_clock) on_clock(); return now.load(std::memory_order_seq_cst); },
                  [this](const Objects& b) { return channel.publish(b); },
                  [this](const ReleaseRecord& r) {
                      if (on_record) on_record(r);
                      std::lock_guard<std::mutex> lock(mutex);
                      records.push_back(r);
                      cv.notify_all();
                  }} {
        now = start_ns;
        CHECK(channel.valid() && release.valid());
    }
    void start() { CHECK(release.start()); }
    void set(std::int64_t ns) {
        now.store(ns, std::memory_order_seq_cst);
        release.wake();
    }
    // The first n records, once n ends were reported.
    std::vector<ReleaseRecord> wait_records(std::size_t n) {
        std::unique_lock<std::mutex> lock(mutex);
        CHECK(cv.wait_for(lock, std::chrono::seconds(20), [&] { return records.size() >= n; }));
        return records;
    }
    std::size_t record_count() {
        std::lock_guard<std::mutex> lock(mutex);
        return records.size();
    }
    void stop() {
        release.close();
        release.join();
    }
};

// Gives a wrongly eager release thread the time to act on the clock just set.
void let_it_run() { std::this_thread::sleep_for(std::chrono::milliseconds(30)); }

bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

// A gate a test thread opens: block() parks the calling thread until open().
struct Gate {
    std::mutex mutex;
    std::condition_variable cv;
    bool reached = false, opened = false;
    void block() {
        std::unique_lock<std::mutex> lock(mutex);
        reached = true;
        cv.notify_all();
        if (!cv.wait_for(lock, std::chrono::seconds(20), [&] { return opened; })) std::abort();
    }
    void wait_reached() {
        std::unique_lock<std::mutex> lock(mutex);
        CHECK(cv.wait_for(lock, std::chrono::seconds(20), [&] { return reached; }));
    }
    void open() {
        std::lock_guard<std::mutex> lock(mutex);
        opened = true;
        cv.notify_all();
    }
};

void balanced(const ReleaseCounters& c) {
    CHECK(c.unaccounted() == 0);
    CHECK(c.offered == c.admitted + c.refused_closed + c.refused_faulted + c.refused_invalid + c.refused_copy);
    // Every commitment is one call, returned or still in flight, and one selection (Deadline or
    // Late); a selection is the commitment itself, so the two counts are equal.
    CHECK(c.committed == c.published + c.channel_refused + c.in_flight);
    CHECK(c.by_deadline + c.late == c.committed && c.inherited <= c.committed);
    CHECK(c.in_flight <= 1 && c.pending <= 1);
    CHECK(c.accepted + c.rejected + c.unknown == c.published);
}

// r + D: nothing goes out one nanosecond early, and the publish stamp is the release time.
void released_at_ready_plus_delay() {
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1010 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    r.set(1043 * kMs - 1);
    let_it_run();
    CHECK(sink.calls.load() == 0 && r.record_count() == 0);
    auto c = r.release.counters();
    CHECK(c.admitted == 1 && c.pending == 1 && c.in_flight == 0 && c.published == 0);
    balanced(c);
    r.set(1043 * kMs);
    const auto records = r.wait_records(1);
    const auto& a = records[0];
    CHECK(a.end == ReleaseEnd::Published && a.reason == ReleaseReason::Deadline && a.frame_id == 1);
    CHECK(a.ready_ns == 1000 * kMs && a.offer_begin_ns == 1010 * kMs && a.handoff_ns == 1010 * kMs &&
          a.deadline_ns == 1043 * kMs && a.due_ns == 1043 * kMs);
    // The claim carries the clock value it was made at: an early claim would show it.
    CHECK(a.claim_ns == 1043 * kMs && a.call_ns == 1043 * kMs && a.return_ns == 1043 * kMs);
    CHECK(a.capture_ms == 960 && a.objects == 2 && a.result && a.result.audit.attempted);
    CHECK(a.result.audit.put_before_ms == 1043 && a.result.audit.capture_ms == 960);
    CHECK(sink.calls.load() == 1 && has(sink.packet(0), "\"t_capture_mono_ms\":960"));
    CHECK(has(sink.packet(0), "\"t_publish_mono_ms\":1043"));
    c = r.release.counters();
    CHECK(c.published == 1 && c.accepted == 1 && c.by_deadline == 1 && c.pending == 0 && c.in_flight == 0);
    CHECK(c.committed == 1 && c.inherited == 0);
    balanced(c);
    r.stop();
    CHECK(!r.release.faulted() && r.channel.counters().attempted == 1 && r.channel.counters().accepted == 1);
}

// A body handed over after its r + D is published at once.
void late_body_goes_out_at_once() {
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 7, 1000 * kMs) == OfferResult::Admitted);
    const auto records = r.wait_records(1);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].reason == ReleaseReason::Late);
    CHECK(records[0].claim_ns == 1050 * kMs && records[0].result.audit.put_before_ms == 1050);
    // Exactly at r + D counts as late as well: there was nothing left to wait for.
    r.set(1076 * kMs);
    CHECK(r.release.offer(body(993), 8, 1033 * kMs) == OfferResult::Admitted);
    CHECK(r.wait_records(2)[1].reason == ReleaseReason::Late);
    const auto c = r.release.counters();
    CHECK(c.late == 2 && c.by_deadline == 0 && c.published == 2);
    balanced(c);
    r.stop();
}

// Latest wins (19 P19-2): a body handed over before the waiting one was selected replaces it. The
// newcomer takes over the release time of the body it replaced, so the replacement does not push
// the publication back: frame 2 goes out at frame 1's r + D, not at its own.
void successor_replaces_the_waiting_body() {
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1008 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    r.set(1041 * kMs);  // two milliseconds before frame 1's r + D
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
    auto records = r.wait_records(1);
    CHECK(records[0].frame_id == 1 && records[0].end == ReleaseEnd::Overwritten && records[0].replaced_by == 2);
    CHECK(records[0].claim_ns == 0 && records[0].call_ns == 0);
    auto c = r.release.counters();
    CHECK(c.overwritten == 1 && c.pending == 1 && c.published == 0 && c.committed == 0);
    r.set(1043 * kMs - 1);
    let_it_run();
    CHECK(sink.calls.load() == 0);
    r.set(1043 * kMs);
    records = r.wait_records(2);
    const auto& a = records[1];
    CHECK(a.frame_id == 2 && a.end == ReleaseEnd::Published && a.reason == ReleaseReason::Deadline);
    CHECK(a.due_ns == 1043 * kMs && a.deadline_ns == 1076 * kMs && a.claim_ns == 1043 * kMs);
    CHECK(sink.calls.load() == 1 && has(sink.packet(0), "\"t_capture_mono_ms\":993"));
    c = r.release.counters();
    CHECK(c.published == 1 && c.overwritten == 1 && c.inherited == 1 && c.by_deadline == 1);
    balanced(c);
    r.stop();
}

// A fast host: every body is handed over 8 ms after its r, the next r 33 ms later, so the next
// hand-over comes 41 ms after a body's r. A replacement passes its due on, so the output never stops
// however the delay compares with that:
//   D < 41 ms  (due before the next hand-over): every body goes out at its own r + D, 33 ms apart;
//   D > 41 ms  (due after the next hand-over): every other body is replaced before its due and the
//              newest one goes out at the replaced body's r + D, 66 ms apart. That is what latest
//              wins means; nothing older than the newest body is published.
// The test steps the clock to each hand-over and, when the waiting body's due comes first, to
// that due, so the release thread sees exactly the order of events the comment describes.
void fast_host_does_not_starve(std::int64_t delay_ns, bool every_other) {
    Sink sink;
    Rig r(sink, 1000 * kMs, delay_ns);
    r.start();
    constexpr int frames = 60;
    std::size_t ends = 0, published = 0;
    std::uint64_t pending = 0;
    std::int64_t due = 0;
    std::vector<std::pair<std::uint64_t, std::int64_t>> expected;  // frame, publish time
    for (int i = 0; i < frames; ++i) {
        const std::int64_t ready = (1000 + 33 * i) * kMs;
        const std::int64_t handoff = ready + 8 * kMs;
        if (pending != 0 && due <= handoff) {
            r.set(due);
            expected.emplace_back(pending, due);
            ++published;
            (void)r.wait_records(++ends);
            pending = 0;
        }
        r.set(handoff);
        CHECK(r.release.offer(body(960 + 33 * i), static_cast<std::uint64_t>(i) + 1, ready) == OfferResult::Admitted);
        if (pending != 0) {
            ++ends;  // the overwrite of the waiting body
            due = std::min(due, ready + delay_ns);
        } else {
            due = ready + delay_ns;
        }
        pending = static_cast<std::uint64_t>(i) + 1;
    }
    r.set(due);
    expected.emplace_back(pending, due);
    ++published;
    const auto records = r.wait_records(++ends);
    std::vector<ReleaseRecord> out;
    for (const auto& record : records)
        if (record.end == ReleaseEnd::Published) out.push_back(record);
    CHECK(out.size() == expected.size() && out.size() == published);
    CHECK(published == (every_other ? frames / 2 : frames));
    for (std::size_t k = 0; k < out.size(); ++k) {
        CHECK(out[k].frame_id == expected[k].first && out[k].claim_ns == expected[k].second);
        CHECK(out[k].result.audit.put_before_ms == expected[k].second / kMs);
        if (k > 0) CHECK(out[k].result.audit.put_before_ms - out[k - 1].result.audit.put_before_ms == (every_other ? 66 : 33));
        CHECK(out[k].claim_ns <= out[k].deadline_ns && out[k].claim_ns == out[k].due_ns);
    }
    const auto c = r.release.counters();
    CHECK(c.published == published && c.overwritten == frames - published);
    CHECK(c.inherited == (every_other ? published : 0));
    balanced(c);
    r.stop();
    CHECK(sink.calls.load() == published);
}

// While a put is in progress and a body is already pending, a new body overwrites the pending
// one. The body inside the channel is not replaced, and each body is attempted once. When the put
// returns the newest body goes out at once: the replaced body's due had passed meanwhile.
void overwrite_while_a_put_is_in_progress() {
    BlockingSink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);  // late: enters the put
    sink.wait_entered();
    r.set(1060 * kMs);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
    auto c = r.release.counters();
    CHECK(c.in_flight == 1 && c.committed == 1 && c.pending == 1 && c.overwritten == 0);
    r.set(1070 * kMs);
    CHECK(r.release.offer(body(1026), 3, 1066 * kMs) == OfferResult::Admitted);
    c = r.release.counters();
    CHECK(c.admitted == 3 && c.in_flight == 1 && c.pending == 1 && c.overwritten == 1 && c.published == 0);
    balanced(c);
    CHECK(sink.calls.load() == 1);
    r.set(1080 * kMs);
    sink.unblock();
    const auto records = r.wait_records(3);
    // The overwrite was recorded first (at the hand-over), the put's end second.
    CHECK(records[0].end == ReleaseEnd::Overwritten && records[0].frame_id == 2 && records[0].replaced_by == 3);
    CHECK(records[0].call_ns == 0 && records[0].claim_ns == 0);
    CHECK(records[1].end == ReleaseEnd::Published && records[1].frame_id == 1 && records[1].return_ns == 1080 * kMs);
    CHECK(records[1].result.audit.put_before_ms == 1050 && records[1].result.audit.put_return_ms == 1080);
    // Frame 3 carries frame 2's due (1076) instead of its own (1109): handed over before it, so a
    // Deadline selection, made at 1080.
    CHECK(records[2].end == ReleaseEnd::Published && records[2].frame_id == 3 && records[2].reason == ReleaseReason::Deadline);
    CHECK(records[2].due_ns == 1076 * kMs && records[2].deadline_ns == 1109 * kMs && records[2].claim_ns == 1080 * kMs);
    CHECK(has(sink.packet(0), "\"t_capture_mono_ms\":960") && has(sink.packet(1), "\"t_capture_mono_ms\":1026"));
    c = r.release.counters();
    CHECK(c.published == 2 && c.overwritten == 1 && c.stop_discarded == 0 && c.inherited == 1);
    balanced(c);
    r.stop();
    CHECK(sink.calls.load() == 2 && r.channel.counters().attempted == 2);
}

// Stop while a body waits for its release: it is not published to drain the slot.
void stop_discards_a_waiting_body() {
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1010 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    r.release.close();
    r.release.close();  // idempotent
    r.release.join();
    const auto records = r.wait_records(1);
    CHECK(records[0].end == ReleaseEnd::StopDiscarded && records[0].frame_id == 1 && records[0].call_ns == 0);
    CHECK(sink.calls.load() == 0 && r.channel.counters().attempted == 0 && r.journal.published() == 0);
    // Hand-overs after the stop are refused, and that is not a fault.
    r.set(1100 * kMs);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Closed);
    const auto c = r.release.counters();
    CHECK(c.offered == 2 && c.admitted == 1 && c.refused_closed == 1 && c.stop_discarded == 1 && c.published == 0);
    balanced(c);
    CHECK(!r.release.faulted() && r.release.fault_reason().empty());
    CHECK(!r.release.start());  // no restart after a stop
}

// Stop while a put is in progress: that put returns and is accounted once; the pending body is
// discarded by close() itself, which does not wait for the put.
void stop_lets_a_put_in_progress_finish() {
    BlockingSink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    sink.wait_entered();
    r.set(1060 * kMs);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
    r.release.close();  // returns although the put is blocked
    CHECK(r.release.offer(body(1026), 3, 1058 * kMs) == OfferResult::Closed);
    auto c = r.release.counters();
    // When close() has returned only the committed body is still inside.
    CHECK(c.in_flight == 1 && c.pending == 0 && c.stop_discarded == 1 && c.published == 0);
    balanced(c);
    r.set(1090 * kMs);
    sink.unblock();
    r.release.join();
    const auto records = r.wait_records(2);
    // The discard was recorded by close(), before the put returned.
    CHECK(records[0].end == ReleaseEnd::StopDiscarded && records[0].frame_id == 2 && records[0].call_ns == 0);
    CHECK(records[1].end == ReleaseEnd::Published && records[1].frame_id == 1 && records[1].result);
    CHECK(records[1].result.audit.put_return_ms == 1090);
    c = r.release.counters();
    CHECK(c.admitted == 2 && c.published == 1 && c.accepted == 1 && c.stop_discarded == 1 && c.refused_closed == 1);
    balanced(c);
    // One attempt and one completion in the journal, no second put.
    CHECK(sink.calls.load() == 1 && r.journal.published() == 2 && r.journal.generation() % 2 == 0);
    PublicationJournal::Event event;
    CHECK(r.journal.pop(event) && event.kind == PublicationJournal::Kind::Attempt && event.audit.capture_ms == 960);
    CHECK(r.journal.pop(event) && event.kind == PublicationJournal::Kind::Completion && event.audit.put_return_ms == 1090);
    CHECK(!r.journal.pop(event) && !r.release.faulted());
}

// Stop while the release thread is held in its observer: frame 2 was replaced by frame 3, which
// waits in the pending place without a commitment. close() discards it before it returns.
void stop_while_the_release_thread_is_held() {
    Sink sink;
    Rig r(sink);
    Gate observer;
    r.on_record = [&](const ReleaseRecord& record) {
        if (record.frame_id == 1) observer.block();
    };
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    observer.wait_reached();
    r.set(1060 * kMs);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
    r.set(1070 * kMs);
    CHECK(r.release.offer(body(1026), 3, 1066 * kMs) == OfferResult::Admitted);  // replaces frame 2
    auto c = r.release.counters();
    CHECK(c.pending == 1 && c.in_flight == 0 && c.overwritten == 1 && c.committed == 1);
    balanced(c);
    r.release.close();
    // The release thread is still held in the observer: close() ended frame 3 itself.
    c = r.release.counters();
    CHECK(c.stop_discarded == 1 && c.pending == 0 && c.in_flight == 0 && c.committed == 1);
    balanced(c);
    observer.open();
    r.release.join();
    const auto records = r.wait_records(3);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].frame_id == 1);
    CHECK(records[1].end == ReleaseEnd::Overwritten && records[1].frame_id == 2 && records[1].replaced_by == 3);
    CHECK(records[2].end == ReleaseEnd::StopDiscarded && records[2].frame_id == 3 && records[2].claim_ns == 0 &&
          records[2].call_ns == 0);
    c = r.release.counters();
    CHECK(c.published == 1 && c.stop_discarded == 1 && c.overwritten == 1);
    balanced(c);
    CHECK(sink.calls.load() == 1 && r.channel.counters().attempted == 1);
}

// GPT question 18.1's counterexample, now with the required outcome. Frame 0's put has returned
// and the release thread is held in its observer, with no put in progress: frames 1, 2 and 3 are
// handed over one after the other. Each replaces the one before (nothing was selected), and when
// the release thread runs again it publishes frame 3 only. Before 2026-10-16 frame 1 was reserved
// by frame 2's hand-over and went out first, then frame 3.
void paused_release_thread_publishes_the_latest() {
    Sink sink;
    Rig r(sink);
    Gate observer;
    r.on_record = [&](const ReleaseRecord& record) {
        if (record.frame_id == 100) observer.block();
    };
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(950), 100, 1000 * kMs) == OfferResult::Admitted);  // late: published at once
    observer.wait_reached();
    CHECK(sink.calls.load() == 1);
    auto c = r.release.counters();
    CHECK(c.published == 1 && c.in_flight == 0 && c.pending == 0);
    r.set(1060 * kMs);
    CHECK(r.release.offer(body(993), 1, 1033 * kMs) == OfferResult::Admitted);
    r.set(1070 * kMs);
    CHECK(r.release.offer(body(1026), 2, 1066 * kMs) == OfferResult::Admitted);  // replaces frame 1
    r.set(1080 * kMs);
    CHECK(r.release.offer(body(1059), 3, 1075 * kMs) == OfferResult::Admitted);  // replaces frame 2
    c = r.release.counters();
    CHECK(c.admitted == 4 && c.in_flight == 0 && c.pending == 1 && c.overwritten == 2);
    CHECK(c.committed == 1 && c.published == 1);
    balanced(c);
    CHECK(sink.calls.load() == 1 && r.channel.counters().attempted == 1);
    r.set(1090 * kMs);
    observer.open();
    const auto records = r.wait_records(4);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].frame_id == 100);
    CHECK(records[1].end == ReleaseEnd::Overwritten && records[1].frame_id == 1 && records[1].replaced_by == 2);
    CHECK(records[2].end == ReleaseEnd::Overwritten && records[2].frame_id == 2 && records[2].replaced_by == 3);
    const auto& last = records[3];
    // Frame 3 carries frame 1's due (1076); it was handed over after it (1080), so it is Late,
    // selected when the release thread ran again (1090).
    CHECK(last.end == ReleaseEnd::Published && last.frame_id == 3 && last.reason == ReleaseReason::Late);
    CHECK(last.due_ns == 1076 * kMs && last.deadline_ns == 1118 * kMs && last.claim_ns == 1090 * kMs && last.call_ns == 1090 * kMs);
    CHECK(has(sink.packet(1), "\"t_capture_mono_ms\":1059"));
    r.set(1200 * kMs);
    let_it_run();
    c = r.release.counters();
    CHECK(c.published == 2 && c.overwritten == 2 && c.committed == 2 && c.stop_discarded == 0 && c.inherited == 1);
    balanced(c);
    r.stop();
    CHECK(sink.calls.load() == 2 && !r.release.faulted());
}

// Late is judged against the due a body carries. Frame 1 is handed over after its own r + D; frame 2,
// handed over before its own r + D, replaces it and takes its due over, so frame 2 is Late too.
void late_is_judged_against_the_due() {
    Sink sink;
    Rig r(sink);
    Gate observer;
    r.on_record = [&](const ReleaseRecord& record) {
        if (record.frame_id == 100) observer.block();
    };
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(950), 100, 1000 * kMs) == OfferResult::Admitted);
    observer.wait_reached();
    r.set(1080 * kMs);
    CHECK(r.release.offer(body(993), 1, 1033 * kMs) == OfferResult::Admitted);  // 4 ms after its r + D
    r.set(1085 * kMs);
    CHECK(r.release.offer(body(1026), 2, 1066 * kMs) == OfferResult::Admitted);  // replaces frame 1
    auto c = r.release.counters();
    CHECK(c.pending == 1 && c.overwritten == 1 && c.late == 1 && c.by_deadline == 0);
    observer.open();
    const auto records = r.wait_records(3);
    CHECK(records[1].frame_id == 1 && records[1].end == ReleaseEnd::Overwritten && records[1].replaced_by == 2);
    CHECK(records[2].frame_id == 2 && records[2].end == ReleaseEnd::Published && records[2].reason == ReleaseReason::Late);
    CHECK(records[2].handoff_ns == 1085 * kMs && records[2].due_ns == 1076 * kMs && records[2].deadline_ns == 1109 * kMs);
    r.stop();
    c = r.release.counters();
    CHECK(c.late == 2 && c.by_deadline == 0 && c.published == 2 && c.overwritten == 1 && c.inherited == 1);
    balanced(c);
}

// offer() reads the clock twice: at its start and, with the lock held, at the hand-over. The
// second read is the hand-over time. Here the clock moves between the two reads.
void handoff_time_is_read_under_the_lock() {
    const auto producer = std::this_thread::get_id();
    {
        // Across the body's own r + D: the offer starts 3 ms before it and the hand-over happens
        // 7 ms after it, so the body is Late although the call began in time.
        Sink sink;
        Rig r(sink);
        int reads = 0;
        r.on_clock = [&] {
            if (std::this_thread::get_id() == producer && ++reads == 2) r.now.store(1050 * kMs, std::memory_order_seq_cst);
        };
        r.start();
        r.now.store(1040 * kMs, std::memory_order_seq_cst);
        CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
        CHECK(reads == 2);
        const auto records = r.wait_records(1);
        const auto& a = records[0];
        CHECK(a.end == ReleaseEnd::Published && a.reason == ReleaseReason::Late);
        CHECK(a.offer_begin_ns == 1040 * kMs && a.handoff_ns == 1050 * kMs && a.deadline_ns == 1043 * kMs);
        CHECK(a.claim_ns == 1050 * kMs && a.result.audit.put_before_ms == 1050);
        const auto c = r.release.counters();
        CHECK(c.late == 1 && c.by_deadline == 0 && c.inherited == 0);
        balanced(c);
        r.stop();
    }
    {
        // A replacement is judged by its hand-over time, not the start of its offer: frame 1 is
        // pending (due 1043), frame 2's offer starts at 1041 and hands over at 1045. Frame 2 takes
        // frame 1's due over and is Late, because its hand-over came after that due.
        Sink sink;
        Rig r(sink);
        int reads = 0;
        bool armed = false;
        r.on_clock = [&] {
            if (armed && std::this_thread::get_id() == producer && ++reads == 2)
                r.now.store(1045 * kMs, std::memory_order_seq_cst);
        };
        // No release thread yet: nothing can select frame 1 while the clock moves.
        r.now.store(1008 * kMs, std::memory_order_seq_cst);
        CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
        r.now.store(1041 * kMs, std::memory_order_seq_cst);
        armed = true;
        CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
        CHECK(reads == 2);
        auto c = r.release.counters();
        CHECK(c.pending == 1 && c.overwritten == 1 && c.committed == 0);
        r.start();
        const auto records = r.wait_records(2);
        CHECK(records[0].frame_id == 1 && records[0].end == ReleaseEnd::Overwritten && records[0].replaced_by == 2);
        CHECK(records[1].frame_id == 2 && records[1].reason == ReleaseReason::Late);
        CHECK(records[1].offer_begin_ns == 1041 * kMs && records[1].handoff_ns == 1045 * kMs);
        CHECK(records[1].due_ns == 1043 * kMs && records[1].deadline_ns == 1076 * kMs && records[1].claim_ns == 1045 * kMs);
        c = r.release.counters();
        CHECK(c.late == 1 && c.inherited == 1);
        balanced(c);
        r.stop();
    }
}

// The clock fails, or reads earlier than at the start of the call, at the hand-over itself: the
// body is not admitted with a made-up time; the fault is latched.
void clock_failure_at_the_handoff_is_refused() {
    const auto producer = std::this_thread::get_id();
    for (const std::int64_t second : {std::int64_t{0}, 1049 * kMs, std::int64_t{-7}}) {
        Sink sink;
        Rig r(sink);
        int reads = 0;
        r.on_clock = [&] {
            if (std::this_thread::get_id() == producer && ++reads == 2) r.now.store(second, std::memory_order_seq_cst);
        };
        r.start();
        r.now.store(1050 * kMs, std::memory_order_seq_cst);
        CHECK(r.release.offer(body(960), 4, 1000 * kMs) == OfferResult::Invalid);
        CHECK(r.release.faulted() && has(r.release.fault_reason(), "frame 4") && has(r.release.fault_reason(), "hand-over"));
        r.release.join();
        const auto c = r.release.counters();
        CHECK(c.offered == 1 && c.admitted == 0 && c.refused_invalid == 1 && c.pending == 0);
        balanced(c);
        CHECK(sink.calls.load() == 0);
    }
    // The same clock value at both reads is fine (the usual case in these tests).
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    r.stop();
}

// The publication commitment. The release thread has committed frame 1 (under the lock, not
// closed) and is held just before it enters the channel, at its call-time clock read. close()
// returns meanwhile, discards the pending frame 2 and refuses frame 3. The committed call is
// still made, exactly once, and is accounted as published; nothing is discarded twice.
void close_after_the_commitment_still_makes_the_one_call() {
    Sink sink;
    Rig r(sink);
    Gate before_call;
    const auto producer = std::this_thread::get_id();
    std::atomic<int> release_reads{0};
    // With nothing pending the release thread sleeps without reading the clock. After the late
    // offer below its reads are: the deadline check (under the lock) and then the call time
    // (without the lock, after the commitment).
    r.on_clock = [&] {
        if (std::this_thread::get_id() != producer && release_reads.fetch_add(1) + 1 == 2) before_call.block();
    };
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    before_call.wait_reached();
    auto c = r.release.counters();
    CHECK(c.in_flight == 1 && c.committed == 1 && c.published == 0);
    CHECK(sink.calls.load() == 0 && r.channel.counters().attempted == 0);  // not in the channel yet
    r.now.store(1060 * kMs, std::memory_order_seq_cst);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
    r.release.close();  // returns: it does not wait for the committed call
    c = r.release.counters();
    CHECK(c.in_flight == 1 && c.pending == 0 && c.stop_discarded == 1 && c.published == 0);
    balanced(c);
    CHECK(sink.calls.load() == 0);
    CHECK(r.release.offer(body(1026), 3, 1058 * kMs) == OfferResult::Closed);
    r.now.store(1070 * kMs, std::memory_order_seq_cst);
    before_call.open();
    r.release.join();
    const auto records = r.wait_records(2);
    CHECK(records[0].end == ReleaseEnd::StopDiscarded && records[0].frame_id == 2 && records[0].call_ns == 0);
    CHECK(records[1].end == ReleaseEnd::Published && records[1].frame_id == 1 && records[1].reason == ReleaseReason::Late);
    // The call began after close() had returned.
    CHECK(records[1].claim_ns == 1050 * kMs && records[1].call_ns == 1070 * kMs && records[1].result);
    c = r.release.counters();
    CHECK(c.admitted == 2 && c.committed == 1 && c.published == 1 && c.accepted == 1 && c.stop_discarded == 1 &&
          c.refused_closed == 1 && c.in_flight == 0);
    balanced(c);
    CHECK(sink.calls.load() == 1 && r.channel.counters().attempted == 1 && !r.release.faulted());
    // One attempt and one completion in the journal.
    PublicationJournal::Event event;
    CHECK(r.journal.published() == 2 && r.journal.pop(event) && event.kind == PublicationJournal::Kind::Attempt);
    CHECK(r.journal.pop(event) && event.kind == PublicationJournal::Kind::Completion && !r.journal.pop(event));
}

// A fault latched by offer() (a frame without a usable ready time) while a body waits in the
// pending place (frame 2, which replaced frame 1): it has no commitment, so the release thread
// discards it; nothing is published after a fault, and the thread ends without a stop.
void fault_discards_what_is_not_committed() {
    Sink sink;
    Rig r(sink);
    Gate observer;
    r.on_record = [&](const ReleaseRecord& record) {
        if (record.frame_id == 100) observer.block();
    };
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(950), 100, 1000 * kMs) == OfferResult::Admitted);
    observer.wait_reached();
    r.set(1060 * kMs);
    CHECK(r.release.offer(body(993), 1, 1033 * kMs) == OfferResult::Admitted);
    r.set(1070 * kMs);
    CHECK(r.release.offer(body(1026), 2, 1066 * kMs) == OfferResult::Admitted);  // replaces frame 1
    CHECK(r.release.offer(body(1059), 3, 0) == OfferResult::Invalid);            // no ready time: fault
    CHECK(r.release.faulted());
    auto c = r.release.counters();
    CHECK(c.pending == 1 && c.overwritten == 1 && c.stop_discarded == 0 && c.refused_invalid == 1);
    observer.open();
    r.release.join();  // ends by itself: the fault closed the release
    const auto records = r.wait_records(3);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].frame_id == 100);
    CHECK(records[1].end == ReleaseEnd::Overwritten && records[1].frame_id == 1 && records[1].replaced_by == 2);
    CHECK(records[2].end == ReleaseEnd::StopDiscarded && records[2].frame_id == 2 && records[2].claim_ns == 0);
    c = r.release.counters();
    CHECK(c.published == 1 && c.committed == 1 && c.stop_discarded == 1 && c.overwritten == 1 && c.pending == 0);
    balanced(c);
    CHECK(sink.calls.load() == 1 && r.channel.counters().attempted == 1);
    CHECK(r.release.offer(body(1092), 4, 1069 * kMs) == OfferResult::Faulted);
}

// Without a release thread close() itself ends what was handed over.
void close_without_a_thread_accounts_the_slot() {
    Sink sink;
    Rig r(sink);
    r.set(1010 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    r.release.close();
    r.release.join();
    const auto c = r.release.counters();
    CHECK(c.admitted == 1 && c.stop_discarded == 1 && c.pending == 0);
    balanced(c);
    CHECK(sink.calls.load() == 0);
}

bool wait_faulted(const ObjectsRelease& release) {
    for (int i = 0; i < 2000 && !release.faulted(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return release.faulted();
}

// A channel refusal on the very first body: the fault is visible without another frame, the
// thread ends by itself and later hand-overs are refused.
void first_frame_channel_refusal_is_latched() {
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    Objects bad = body(960);
    bad.objects[0].class_name.clear();  // contract violation: InvalidBody, no put
    CHECK(r.release.offer(bad, 1, 1000 * kMs) == OfferResult::Admitted);
    CHECK(wait_faulted(r.release));
    r.release.join();  // no close(): the thread ended on the fault
    const auto records = r.wait_records(1);
    CHECK(records[0].end == ReleaseEnd::ChannelRefused && records[0].result.error == PublishError::InvalidBody);
    CHECK(!records[0].result.audit.attempted && sink.calls.load() == 0);
    CHECK(has(r.release.fault_reason(), "contract fault") && has(r.release.fault_reason(), "frame_id=1") &&
          has(r.release.fault_reason(), "error=" + std::to_string(static_cast<int>(PublishError::InvalidBody))));
    r.set(1100 * kMs);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Faulted);
    const auto c = r.release.counters();
    CHECK(c.channel_refused == 1 && c.contract_faults == 1 && c.published == 0 && c.refused_faulted == 1);
    balanced(c);
}

// The last body before the input ends is refused (same capture as its predecessor): latched with
// no further frame. It has no successor, waits for its own due and is refused by the channel
// without a put; a hand-over after that is refused as Faulted.
void last_frame_channel_refusal_is_latched() {
    BlockingSink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    sink.wait_entered();
    sink.unblock();
    (void)r.wait_records(1);
    r.set(1060 * kMs);
    CHECK(r.release.offer(body(960), 2, 1033 * kMs) == OfferResult::Admitted);
    r.set(1076 * kMs - 1);
    let_it_run();
    CHECK(!r.release.faulted() && sink.calls.load() == 1);
    r.set(1076 * kMs);
    CHECK(wait_faulted(r.release));
    r.release.join();
    const auto c = r.release.counters();
    CHECK(c.published == 1 && c.channel_refused == 1 && c.contract_faults == 1 && c.stop_discarded == 0);
    balanced(c);
    CHECK(sink.calls.load() == 1 && has(r.release.fault_reason(), "frame_id=2"));
    const auto records = r.wait_records(2);
    CHECK(records[1].end == ReleaseEnd::ChannelRefused && records[1].frame_id == 2 &&
          records[1].result.error == PublishError::Duplicate && !records[1].result.audit.attempted);
    CHECK(records[1].reason == ReleaseReason::Deadline && records[1].claim_ns == 1076 * kMs);
    CHECK(r.release.offer(body(1026), 3, 1066 * kMs) == OfferResult::Faulted);
}

// A put that was attempted but whose return clock is broken: published once, and a fault.
void broken_return_clock_is_a_fault_after_the_attempt() {
    Sink sink;
    Rig r(sink);
    sink.before_return = [&] { r.now.store(900 * kMs); };  // the clock steps back inside the put
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
    CHECK(wait_faulted(r.release));
    r.release.join();
    const auto records = r.wait_records(1);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].result.audit.attempted);
    CHECK(records[0].result.error == PublishError::ClockAfter);
    const auto c = r.release.counters();
    CHECK(c.published == 1 && c.accepted == 1 && c.contract_faults == 1 && c.channel_refused == 0);
    balanced(c);
    CHECK(sink.calls.load() == 1);
}

// Rejected, unknown and throwing puts are transport outcomes: counted, never retried, no fault.
void transport_outcomes_are_counted_not_retried() {
    Sink sink;
    Rig r(sink);
    r.start();
    const auto publish_one = [&](int i) {
        const std::int64_t ready = (1000 + 33 * i) * kMs;
        r.set(ready + 50 * kMs);
        CHECK(r.release.offer(body(960 + 33 * i), static_cast<std::uint64_t>(i) + 1, ready) == OfferResult::Admitted);
        return r.wait_records(static_cast<std::size_t>(i) + 1)[static_cast<std::size_t>(i)];
    };
    sink.outcome = PutOutcome::Rejected;
    auto a = publish_one(0);
    CHECK(a.end == ReleaseEnd::Published && a.result.error == PublishError::PutRejected && a.result.audit.attempted);
    sink.outcome = PutOutcome::Unknown;
    a = publish_one(1);
    CHECK(a.end == ReleaseEnd::Published && a.result.error == PublishError::PutUnknown);
    sink.outcome = PutOutcome::Accepted;
    sink.throws = true;
    a = publish_one(2);
    CHECK(a.end == ReleaseEnd::Published && a.result.error == PublishError::PutUnknown && a.result.audit.threw);
    sink.throws = false;
    a = publish_one(3);
    CHECK(a.end == ReleaseEnd::Published && a.result);
    const auto c = r.release.counters();
    CHECK(c.published == 4 && c.accepted == 1 && c.rejected == 1 && c.unknown == 2 && c.contract_faults == 0);
    balanced(c);
    CHECK(!r.release.faulted() && sink.calls.load() == 4);
    r.stop();
    const auto channel = r.channel.counters();
    CHECK(channel.attempted == 4 && channel.accepted == 1 && channel.rejected == 1 && channel.unknown == 2);
}

// A frame without a usable ready time cannot be paced; "now" is never used in its place.
void unusable_ready_time_is_refused_and_latched() {
    for (int variant = 0; variant < 4; ++variant) {
        Sink sink;
        Rig r(sink);
        r.start();
        r.set(variant == 3 ? 0 : 1050 * kMs);  // variant 3: the clock itself is unavailable
        const std::int64_t ready = variant == 0 ? 0 : variant == 1 ? -5 : variant == 2 ? 1050 * kMs + 1 : 1000 * kMs;
        CHECK(r.release.offer(body(960), 9, ready) == OfferResult::Invalid);
        CHECK(r.release.faulted() && has(r.release.fault_reason(), "frame 9") && has(r.release.fault_reason(), "ready"));
        r.release.join();
        const auto c = r.release.counters();
        CHECK(c.offered == 1 && c.admitted == 0 && c.refused_invalid == 1);
        balanced(c);
        CHECK(sink.calls.load() == 0);
        r.set(1100 * kMs);
        CHECK(r.release.offer(body(993), 10, 1050 * kMs) == OfferResult::Faulted);
    }
    // r exactly equal to now is usable.
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1050 * kMs) == OfferResult::Admitted);
    r.stop();
    // An unusable configuration is reported by valid() and refuses to start.
    std::atomic<std::int64_t> now{1};
    const auto clock = [&] { return now.load(); };
    const auto publish = [](const Objects&) { return PublishResult{}; };
    for (const std::int64_t delay : {std::int64_t{0}, std::int64_t{-1}, ObjectsRelease::kMaxDelayNs + 1}) {
        ObjectsRelease bad(delay, clock, publish);
        CHECK(!bad.valid() && !bad.start());
        CHECK(bad.offer(body(960), 1, 1) == OfferResult::Invalid && bad.faulted());
    }
    ObjectsRelease no_clock(kDelay, {}, publish), no_publish(kDelay, clock, {});
    CHECK(!no_clock.valid() && !no_publish.valid() && !no_clock.start() && !no_publish.start());
    ObjectsRelease longest(ObjectsRelease::kMaxDelayNs, clock, publish);
    CHECK(longest.valid());
}

// The release publishes an owned copy: the wire bytes are those of a direct publish of the same
// body at the same clock, although the source was changed and destroyed before the release.
void wire_bytes_equal_a_direct_publish(std::size_t objects, std::size_t name_bytes) {
    std::string direct;
    {
        Sink sink;
        std::atomic<std::int64_t> now{1043 * kMs};
        Clock clock{now};
        ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
        PublicationJournal journal{Topic::Objects, 64};
        ObjectsCadence cadence{0, 64};
        PublisherChannel channel{Topic::Objects, envelope, clock, sink, kBodyCapacity, kPacketCapacity, &cadence, &journal};
        const auto result = channel.publish(body(960, objects, name_bytes));
        CHECK(result && sink.calls.load() == 1);
        direct = sink.packet(0);
    }
    Sink sink;
    Rig r(sink);
    r.start();
    r.set(1010 * kMs);
    {
        auto source = std::make_unique<Objects>(body(960, objects, name_bytes));
        CHECK(r.release.offer(*source, 1, 1000 * kMs) == OfferResult::Admitted);
        // What the producer does next: the adapter reuses its body for the following frame.
        for (auto& o : source->objects) {
            o.class_name.assign(o.class_name.size(), '#');
            o.semantic_status = "proxy";
            if (o.geometry) o.geometry->footprint_xy.assign(o.geometry->footprint_xy.size(), Point{-9, -9});
            o.velocity_xy.reset();
            o.geometry.reset();
        }
        source->t_capture_mono_ms = 5;
        source->objects.clear();
        source.reset();
    }
    r.set(1043 * kMs);
    const auto records = r.wait_records(1);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].result && records[0].objects == objects);
    const std::string paced = sink.packet(0);
    CHECK(paced.size() == direct.size() && paced == direct);
    CHECK(has(paced, "\"t_publish_mono_ms\":1043") && !has(paced, "###"));
    r.stop();
}

// Bodies of one shape: after the three buffers were filled once, a hand-over and its release
// allocate nothing.
void same_shape_bodies_do_not_allocate() {
    struct FixedSink final : RnsTransport {
        std::atomic<std::size_t> calls{0};
        PutReceipt put(std::string_view, std::string_view, PublishQos) override {
            calls.fetch_add(1, std::memory_order_release);
            return {PutOutcome::Accepted, 0};
        }
    } sink;
    std::atomic<std::int64_t> now{1000 * kMs};
    Clock clock{now};
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublicationJournal journal{Topic::Objects, 4096};
    ObjectsCadence cadence{0, 1024};
    PublisherChannel channel{Topic::Objects, envelope, clock, sink, kBodyCapacity, kPacketCapacity, &cadence, &journal};
    std::atomic<std::size_t> ended{0};
    ObjectsRelease release{kDelay, [&] { return now.load(); }, [&](const Objects& b) { return channel.publish(b); },
                           [&](const ReleaseRecord& record) { if (record.end == ReleaseEnd::Published) ended.fetch_add(1); }};
    CHECK(release.start());
    std::vector<Objects> bodies;
    constexpr int warm = 6, measured = 60;
    for (int i = 0; i < warm + measured; ++i) bodies.push_back(body(960 + 33 * i, 10, 24));
    // The journal is not read here; its ring holds every event of this case.
    std::size_t before = 0;
    for (int i = 0; i < warm + measured; ++i) {
        if (i == warm) before = allocation_count.load();
        const std::int64_t ready = (1000 + 33 * i) * kMs;
        now.store(ready + 50 * kMs);
        CHECK(release.offer(bodies[static_cast<std::size_t>(i)], static_cast<std::uint64_t>(i) + 1, ready) ==
              OfferResult::Admitted);
        for (int spin = 0; spin < 20000 && ended.load() != static_cast<std::size_t>(i) + 1; ++spin)
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        CHECK(ended.load() == static_cast<std::size_t>(i) + 1);
    }
    const std::size_t after = allocation_count.load();
    release.close();
    release.join();
    CHECK(after == before);
    CHECK(sink.calls.load() == warm + measured && !release.faulted());
}

// A body waiting for its release is, for Status, an open publication gap like any other: the
// journal is even (not busy), nothing is excluded, and the release closes the gap.
void waiting_shows_as_an_open_gap_to_status() {
    Sink sink;
    Rig r(sink, 40000 * kMs);  // past the cadence ledger's 30 s warm-up
    PublicationJournal profile_journal{Topic::Profile, 64};
    HealthWindow health{0, 256};
    ObjectsCadence status_cadence{0, 256};
    PublicationHealthHandoff handoff{0, profile_journal, r.journal, health, status_cadence};
    CHECK(handoff.valid());
    r.start();
    r.set(40043 * kMs);
    CHECK(r.release.offer(body(39960), 1, 40000 * kMs) == OfferResult::Admitted);
    (void)r.wait_records(1);
    // Frame 2 waits from 40070 to its r + D = 40109.
    r.set(40070 * kMs);
    CHECK(r.release.offer(body(39993), 2, 40066 * kMs) == OfferResult::Admitted);
    r.set(40100 * kMs);
    let_it_run();
    CHECK(r.journal.generation() % 2 == 0);
    auto cut = handoff.drain(r.clock);
    CHECK(cut.code == HandoffCode::Ready && cut.observed_ms == 40099 && cut.events[1] == 2);
    auto window = health.snapshot(cut.observed_ms);
    CHECK(window.valid && window.objects.total.attempted == 1 && window.objects.total.pending == 0);
    CHECK(window.objects.open_gap_ms == 40099 - 40043);
    auto mirror = status_cadence.snapshot(cut.observed_ms);
    CHECK(mirror.valid && mirror.unique_results == 1 && mirror.open_gap_ms == 40099 - 40043 && !mirror.excluded);
    r.set(40109 * kMs);
    const auto records = r.wait_records(2);
    CHECK(records[1].reason == ReleaseReason::Deadline && records[1].result.audit.put_before_ms == 40109);
    r.set(40120 * kMs);
    cut = handoff.drain(r.clock);
    CHECK(cut.code == HandoffCode::Ready && cut.events[1] == 2);
    window = health.snapshot(cut.observed_ms);
    CHECK(window.objects.total.attempted == 2 && window.objects.open_gap_ms == 40119 - 40109);
    mirror = status_cadence.snapshot(cut.observed_ms);
    // The 66 ms between the two publications is one closed interval above 50 ms: the wait counts.
    CHECK(mirror.unique_results == 2 && mirror.closed_gaps == 1 && mirror.over50 == 1 && mirror.over100 == 0);
    r.stop();
    CHECK(handoff.valid());
}

// A transport that refuses (or throws) while preparing, before the packet is stamped: the
// channel reports a prepare failure without an attempt. That is a local fault, not a transport
// outcome: the body is refused by the channel, nothing reaches the journal, the fault is latched.
void prepare_failure_is_a_refusal_without_an_attempt() {
    struct PrepareSink final : RnsTransport {
        bool throws = false;
        std::atomic<std::size_t> puts{0}, prepares{0};
        PutReceipt put(std::string_view, std::string_view, PublishQos) override {
            puts.fetch_add(1);
            return {PutOutcome::Accepted, 0};
        }
        PutReceipt put_finalized(std::string_view, char*, std::size_t, PublishQos, PacketFinalizer&) override {
            prepares.fetch_add(1);
            if (throws) throw std::runtime_error("prepare");
            return {PutOutcome::Rejected, 91};
        }
    };
    for (const bool throws : {false, true}) {
        PrepareSink sink;
        sink.throws = throws;
        Rig r(sink);
        r.start();
        r.set(1050 * kMs);
        CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
        CHECK(wait_faulted(r.release));
        r.release.join();
        const auto records = r.wait_records(1);
        CHECK(records[0].end == ReleaseEnd::ChannelRefused && !records[0].result.audit.attempted);
        CHECK(records[0].result.error == (throws ? PublishError::PrepareUnknown : PublishError::PrepareRejected));
        const auto c = r.release.counters();
        CHECK(c.channel_refused == 1 && c.published == 0 && c.contract_faults == 1 && c.rejected == 0 && c.unknown == 0);
        balanced(c);
        CHECK(sink.prepares.load() == 1 && sink.puts.load() == 0);
        // Nothing was stamped: no journal event, the capture is not consumed.
        CHECK(r.journal.published() == 0 && r.journal.generation() % 2 == 0 && r.journal.valid());
        CHECK(r.channel.counters().attempted == 0 && r.channel.counters().preflight_failed == 1);
    }
}

// A put that blocks inside the transport while another body waits: Status takes a certified cut
// with the attempt pending, the open gap runs from that attempt, and the waiting body changes
// nothing in the journal until it is released.
void status_sees_a_blocked_put_with_a_waiting_body() {
    BlockingSink sink;
    Rig r(sink, 40000 * kMs);
    Clock status_clock{r.now};
    PublicationJournal profile_journal{Topic::Profile, 64};
    HealthWindow health{0, 256};
    ObjectsCadence status_cadence{0, 256};
    PublicationHealthHandoff handoff{0, profile_journal, r.journal, health, status_cadence};
    r.start();
    r.set(40050 * kMs);
    CHECK(r.release.offer(body(39960), 1, 40000 * kMs) == OfferResult::Admitted);  // late: enters the put
    sink.wait_entered();
    r.set(40070 * kMs);
    CHECK(r.release.offer(body(39993), 2, 40066 * kMs) == OfferResult::Admitted);  // waits behind it
    r.set(40100 * kMs);
    auto cut = handoff.drain(status_clock);
    CHECK(cut.code == HandoffCode::Ready && cut.observed_ms == 40099 && cut.events[1] == 1);
    auto window = health.snapshot(cut.observed_ms);
    CHECK(window.valid && window.objects.total.attempted == 1 && window.objects.total.pending == 1);
    CHECK(window.objects.total.accepted == 0 && window.objects.open_gap_ms == 40099 - 40050);
    auto c = r.release.counters();
    CHECK(c.in_flight == 1 && c.pending == 1 && c.published == 0);
    balanced(c);
    // The put returns at 40120; frame 2's r + D (40109) has passed meanwhile: it goes out at once.
    r.set(40120 * kMs);
    sink.unblock();
    const auto records = r.wait_records(2);
    CHECK(records[0].frame_id == 1 && records[0].result.audit.put_return_ms == 40120);
    CHECK(records[1].frame_id == 2 && records[1].reason == ReleaseReason::Deadline && records[1].claim_ns == 40120 * kMs &&
          records[1].deadline_ns == 40109 * kMs && records[1].result.audit.put_before_ms == 40120);
    r.set(40130 * kMs);
    cut = handoff.drain(status_clock);
    CHECK(cut.code == HandoffCode::Ready && cut.events[1] == 3);
    window = health.snapshot(cut.observed_ms);
    CHECK(window.objects.total.attempted == 2 && window.objects.total.accepted == 2 && window.objects.total.pending == 0);
    CHECK(status_cadence.snapshot(cut.observed_ms).unique_results == 2);
    r.stop();
    CHECK(handoff.valid() && r.journal.valid());
}

// The journal is odd (Status gets Busy, never a made-up healthy cut) exactly while the release
// thread is inside the channel's stamp-and-commit step; bodies handed over and overwritten in
// that time are accounted as usual, and the next cut is certified again.
void status_is_busy_only_inside_the_commit_step() {
    Sink sink;
    Rig r(sink, 40000 * kMs);
    std::mutex gate;
    std::condition_variable cv;
    bool armed = true, inside = false, release = false;
    // Block the release thread in its first put-before clock read: after begin_attempt, before
    // the attempt is recorded.
    r.clock.on_mono = [&] {
        std::unique_lock<std::mutex> lock(gate);
        if (!armed) return;
        armed = false;
        inside = true;
        cv.notify_all();
        if (!cv.wait_for(lock, std::chrono::seconds(20), [&] { return release; })) std::abort();
    };
    Clock status_clock{r.now};
    PublicationJournal profile_journal{Topic::Profile, 64};
    HealthWindow health{0, 256};
    ObjectsCadence status_cadence{0, 256};
    PublicationHealthHandoff handoff{0, profile_journal, r.journal, health, status_cadence};
    r.start();
    r.set(40030 * kMs);
    CHECK(r.release.offer(body(39960), 1, 40000 * kMs) == OfferResult::Admitted);
    // While it only waits for its release the journal is even and a cut is certified.
    let_it_run();
    CHECK(r.journal.generation() % 2 == 0 && handoff.drain(status_clock).code == HandoffCode::Ready);
    r.set(40043 * kMs);
    {
        std::unique_lock<std::mutex> lock(gate);
        CHECK(cv.wait_for(lock, std::chrono::seconds(20), [&] { return inside; }));
    }
    CHECK(r.journal.generation() % 2 == 1);
    r.now.store(40060 * kMs);
    const auto busy = handoff.drain(status_clock);
    CHECK(busy.code == HandoffCode::Busy && busy.events[1] == 0 && handoff.valid());
    // Two more bodies arrive meanwhile: one waits, the next overwrites it.
    CHECK(r.release.offer(body(39993), 2, 40033 * kMs) == OfferResult::Admitted);
    CHECK(r.release.offer(body(40026), 3, 40059 * kMs) == OfferResult::Admitted);
    auto c = r.release.counters();
    CHECK(c.admitted == 3 && c.in_flight == 1 && c.pending == 1 && c.overwritten == 1);
    balanced(c);
    {
        std::lock_guard<std::mutex> lock(gate);
        release = true;
        cv.notify_all();
    }
    auto records = r.wait_records(2);
    CHECK(records[0].end == ReleaseEnd::Overwritten && records[0].frame_id == 2);
    CHECK(records[1].end == ReleaseEnd::Published && records[1].frame_id == 1 && records[1].result.audit.put_before_ms == 40060);
    r.set(40102 * kMs);
    records = r.wait_records(3);
    // Frame 3 replaced frame 2 and carries its due (40033 + 43 = 40076), still ahead of the clock
    // when the put returned (40060); it goes out when the clock reaches it.
    CHECK(records[2].frame_id == 3 && records[2].reason == ReleaseReason::Deadline && records[2].claim_ns == 40102 * kMs &&
          records[2].due_ns == 40076 * kMs);
    r.set(40110 * kMs);
    const auto cut = handoff.drain(status_clock);
    CHECK(cut.code == HandoffCode::Ready && cut.events[1] == 4 && r.journal.generation() % 2 == 0);
    const auto window = health.snapshot(cut.observed_ms);
    CHECK(window.objects.total.attempted == 2 && window.objects.total.accepted == 2);
    c = r.release.counters();
    CHECK(c.published == 2 && c.overwritten == 1);
    balanced(c);
    r.stop();
}

std::int64_t real_now_ns() {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return static_cast<std::int64_t>(ts.tv_sec) * INT64_C(1000000000) + ts.tv_nsec;
}

// Concurrent producer, a transport that is sometimes slow and a stop from a third thread, on the
// real clock. Every admitted body has exactly one end; published captures strictly increase;
// puts, channel attempts and the release's own count agree. (Also the data-race target: build
// this file with -fsanitize=thread.)
void concurrent_producer_transport_and_stop(unsigned seed) {
    struct SlowSink final : RnsTransport {
        std::mt19937 random;
        std::atomic<std::size_t> calls{0};
        explicit SlowSink(unsigned seed) : random(seed) {}
        PutReceipt put(std::string_view, std::string_view, PublishQos) override {
            calls.fetch_add(1, std::memory_order_release);
            const unsigned draw = random() % 16U;
            if (draw == 0) std::this_thread::sleep_for(std::chrono::microseconds(600));
            return {draw == 1 ? PutOutcome::Rejected : draw == 2 ? PutOutcome::Unknown : PutOutcome::Accepted, 0};
        }
    } sink(seed);
    struct RealClock final : PublishClock {
        Millis wall_millis() noexcept override { return 1700000000000LL; }
        Millis mono_millis() noexcept override { return real_now_ns() / kMs; }
    } clock;
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublicationJournal journal{Topic::Objects, 16384};
    ObjectsCadence cadence{0, 4096};
    PublisherChannel channel{Topic::Objects, envelope, clock, sink, kBodyCapacity, kPacketCapacity, &cadence, &journal};
    std::vector<ReleaseRecord> records;  // release thread only until it is joined
    records.reserve(8192);
    const std::int64_t delay = 250000;  // 0.25 ms
    ObjectsRelease release{delay, [] { return real_now_ns(); }, [&](const Objects& b) { return channel.publish(b); },
                           [&](const ReleaseRecord& record) { records.push_back(record); }};
    CHECK(release.start());
    constexpr int frames = 1500;
    std::atomic<int> offered{0};
    std::vector<OfferResult> results(frames, OfferResult::Invalid);
    std::thread producer([&] {
        std::mt19937 random(seed * 7919U + 1U);
        // Capture times are milliseconds: one per frame, strictly increasing, well in the past.
        const Millis base = real_now_ns() / kMs / 2;
        for (int i = 0; i < frames; ++i) {
            const std::int64_t ready = real_now_ns() - static_cast<std::int64_t>(random() % 150000U);
            results[static_cast<std::size_t>(i)] =
                release.offer(body(base + i, 1 + random() % 4U), static_cast<std::uint64_t>(i) + 1, ready);
            offered.store(i + 1, std::memory_order_release);
            const unsigned pause = random() % 500U;
            if (pause > 60) std::this_thread::sleep_for(std::chrono::microseconds(pause));
        }
    });
    std::thread stopper([&] {
        // Close while bodies are still being handed over.
        const int at = 1000 + static_cast<int>(seed * 37U % 300U);
        while (offered.load(std::memory_order_acquire) < at) std::this_thread::sleep_for(std::chrono::microseconds(50));
        release.close();
    });
    producer.join();
    stopper.join();
    release.join();
    const auto c = release.counters();
    CHECK(!release.faulted() && c.contract_faults == 0 && c.refused_invalid == 0 && c.refused_copy == 0);
    CHECK(c.offered == frames && c.pending == 0 && c.in_flight == 0);
    CHECK(c.committed == c.published + c.channel_refused);
    balanced(c);
    std::size_t admitted = 0, closed = 0;
    for (const auto result : results) {
        CHECK(result == OfferResult::Admitted || result == OfferResult::Closed);
        (result == OfferResult::Admitted ? admitted : closed) += 1;
    }
    CHECK(admitted == c.admitted && closed == c.refused_closed && closed > 0);
    // Exactly one end per admitted frame, and the ends agree with the counters.
    CHECK(c.records_lost == 0 && records.size() == c.admitted);
    std::vector<int> ends(static_cast<std::size_t>(frames) + 1, 0);
    std::size_t published = 0, overwritten = 0, discarded = 0;
    Millis last_capture = -1;
    for (const auto& record : records) {
        CHECK(record.frame_id >= 1 && record.frame_id <= static_cast<std::uint64_t>(frames));
        CHECK(results[static_cast<std::size_t>(record.frame_id) - 1] == OfferResult::Admitted);
        ++ends[static_cast<std::size_t>(record.frame_id)];
        if (record.end == ReleaseEnd::Published) {
            ++published;
            CHECK(record.capture_ms > last_capture);
            last_capture = record.capture_ms;
            CHECK(record.ready_ns <= record.offer_begin_ns && record.offer_begin_ns <= record.handoff_ns);
            CHECK(record.claim_ns >= record.handoff_ns && record.call_ns >= record.claim_ns && record.return_ns >= record.call_ns);
            // A due is never later than the body's own r + D; a Deadline selection is never early;
            // a Late body was handed over at or after its due.
            CHECK(record.due_ns <= record.deadline_ns && record.claim_ns >= record.due_ns);
            if (record.reason == ReleaseReason::Deadline) CHECK(record.handoff_ns < record.due_ns);
            if (record.reason == ReleaseReason::Late) CHECK(record.handoff_ns >= record.due_ns);
            CHECK(record.reason == ReleaseReason::Deadline || record.reason == ReleaseReason::Late);
        } else if (record.end == ReleaseEnd::Overwritten) {
            ++overwritten;
            CHECK(record.replaced_by > record.frame_id && record.call_ns == 0);
        } else {
            CHECK(record.end == ReleaseEnd::StopDiscarded && record.call_ns == 0);
            ++discarded;
        }
    }
    for (const int n : ends) CHECK(n <= 1);
    CHECK(published == c.published && overwritten == c.overwritten && discarded == c.stop_discarded);
    CHECK(sink.calls.load() == c.published && channel.counters().attempted == c.published);
    CHECK(channel.counters().accepted == c.accepted && channel.counters().rejected == c.rejected &&
          channel.counters().unknown == c.unknown);
    CHECK(journal.valid() && journal.generation() % 2 == 0);
}
// GPT question 18 supplement: a stream of records must not keep a due body from being selected.
// The producer hands over two bodies (the second replaces the first, so one more record waits)
// every time the observer is entered, and the observer returns only after that. All dues lie in the
// past. Before 2026-10-16 the release thread went back to the records after every batch and never
// reached the selection while the stream lasted; now bodies are published during the stream, and
// each published body is the newest one at its selection (the second of its round).
void observer_backlog_does_not_block_the_release() {
    Sink sink;
    Rig r(sink);
    std::mutex m;
    std::condition_variable cv;
    int entries = 0, rounds_done = 0;
    bool streaming = true;
    std::vector<std::uint64_t> published_while_streaming;
    r.on_record = [&](const ReleaseRecord& record) {
        std::unique_lock<std::mutex> lock(m);
        if (!streaming) return;
        if (record.end == ReleaseEnd::Published && record.frame_id != 1) published_while_streaming.push_back(record.frame_id);
        const int entry = ++entries;
        cv.notify_all();
        if (!cv.wait_for(lock, std::chrono::seconds(20), [&] { return rounds_done >= entry || !streaming; })) std::abort();
    };
    r.start();
    r.set(5000 * kMs);
    CHECK(r.release.offer(body(3000), 1, 4000 * kMs) == OfferResult::Admitted);  // late: published, enters the observer
    constexpr int kRounds = 30;
    for (int round = 1; round <= kRounds; ++round) {
        {
            std::unique_lock<std::mutex> lock(m);
            CHECK(cv.wait_for(lock, std::chrono::seconds(20), [&] { return entries >= round; }));
        }
        const auto first = static_cast<std::uint64_t>(10 * round), second = first + 1;
        CHECK(r.release.offer(body(3000 + 2 * round), first, 4000 * kMs + round) == OfferResult::Admitted);
        CHECK(r.release.offer(body(3001 + 2 * round), second, 4000 * kMs + round) == OfferResult::Admitted);
        std::lock_guard<std::mutex> lock(m);
        rounds_done = round;
        cv.notify_all();
    }
    std::vector<std::uint64_t> during;
    {
        std::lock_guard<std::mutex> lock(m);
        during = published_while_streaming;
        streaming = false;
        cv.notify_all();
    }
    // Progress while the stream lasted (one selection per observer batch; every record makes the
    // observer wait for one more round, so batches grow and 30 rounds give about 4 to 5 selections;
    // the old loop gave none), and only the newest body of a round was ever selected.
    CHECK(during.size() >= 3);
    for (const auto id : during) CHECK(id % 10 == 1);
    for (int i = 0; i < 2000; ++i) {
        const auto c = r.release.counters();
        if (c.pending == 0 && c.in_flight == 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    r.stop();
    const auto c = r.release.counters();
    CHECK(c.admitted == 1 + 2 * kRounds && c.pending == 0 && c.in_flight == 0 && c.published >= 1 + during.size());
    balanced(c);
    CHECK(!r.release.faulted());
}

// Fable question 13.1: a hand-over in the gap between the selection (under the lock) and the call
// (without it). The release thread is held at its call-time clock read, after the commitment: the
// new body goes to the pending place, the selected one is published unchanged, then the new one.
void handoff_between_selection_and_call() {
    Sink sink;
    Rig r(sink);
    Gate before_call;
    const auto producer = std::this_thread::get_id();
    std::atomic<int> release_reads{0};
    r.on_clock = [&] {
        if (std::this_thread::get_id() != producer && release_reads.fetch_add(1) + 1 == 2) before_call.block();
    };
    r.start();
    r.set(1050 * kMs);
    CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);  // late: selected at once
    before_call.wait_reached();
    auto c = r.release.counters();
    CHECK(c.in_flight == 1 && c.committed == 1 && c.pending == 0 && sink.calls.load() == 0);
    r.now.store(1060 * kMs, std::memory_order_seq_cst);
    CHECK(r.release.offer(body(993), 2, 1033 * kMs) == OfferResult::Admitted);
    c = r.release.counters();
    CHECK(c.in_flight == 1 && c.pending == 1 && c.overwritten == 0);
    before_call.open();
    auto records = r.wait_records(1);
    CHECK(records[0].end == ReleaseEnd::Published && records[0].frame_id == 1 && records[0].call_ns == 1060 * kMs);
    CHECK(has(sink.packet(0), "\"t_capture_mono_ms\":960"));
    r.set(1076 * kMs);
    records = r.wait_records(2);
    CHECK(records[1].end == ReleaseEnd::Published && records[1].frame_id == 2 && records[1].reason == ReleaseReason::Deadline);
    CHECK(records[1].due_ns == 1076 * kMs && records[1].claim_ns == 1076 * kMs && has(sink.packet(1), "\"t_capture_mono_ms\":993"));
    c = r.release.counters();
    CHECK(c.published == 2 && c.overwritten == 0 && c.inherited == 0);
    balanced(c);
    r.stop();
}

// Fable question 13.1: a hand-over at exactly the due of the pending body, while the release thread
// is woken for that due. Both orders are legal and the test accepts either:
//   the release thread first: frame 1 selected at its due (Deadline), frame 2 pending on its own due;
//   the hand-over first: frame 1 replaced, frame 2 carries its due and is Late (handed over at it).
void handoff_exactly_at_the_due() {
    for (int run = 0; run < 20; ++run) {
        Sink sink;
        Rig r(sink);
        r.start();
        r.set(1010 * kMs);
        CHECK(r.release.offer(body(960), 1, 1000 * kMs) == OfferResult::Admitted);
        r.set(1043 * kMs);  // frame 1's due; the release thread may or may not run first
        CHECK(r.release.offer(body(993), 2, 1040 * kMs) == OfferResult::Admitted);
        const auto first = r.wait_records(1)[0];
        if (first.end == ReleaseEnd::Published) {
            CHECK(first.frame_id == 1 && first.reason == ReleaseReason::Deadline && first.claim_ns == 1043 * kMs);
            r.set(1083 * kMs);
            const auto second = r.wait_records(2)[1];
            CHECK(second.frame_id == 2 && second.end == ReleaseEnd::Published && second.due_ns == 1083 * kMs &&
                  second.reason == ReleaseReason::Deadline);
        } else {
            CHECK(first.end == ReleaseEnd::Overwritten && first.frame_id == 1 && first.replaced_by == 2);
            const auto second = r.wait_records(2)[1];
            CHECK(second.frame_id == 2 && second.end == ReleaseEnd::Published && second.due_ns == 1043 * kMs &&
                  second.reason == ReleaseReason::Late && second.handoff_ns == 1043 * kMs);
        }
        balanced(r.release.counters());
        r.stop();
        CHECK(!r.release.faulted());
    }
}

}  // namespace

int main() {
    released_at_ready_plus_delay();
    late_body_goes_out_at_once();
    successor_replaces_the_waiting_body();
    // Period 33 ms, ready -> hand-over 8 ms: a replacement exactly when D > 41 ms.
    fast_host_does_not_starve(20 * kMs, false);
    fast_host_does_not_starve(30 * kMs, false);
    fast_host_does_not_starve(40 * kMs, false);
    fast_host_does_not_starve(42 * kMs, true);
    fast_host_does_not_starve(43 * kMs, true);
    overwrite_while_a_put_is_in_progress();
    paused_release_thread_publishes_the_latest();
    late_is_judged_against_the_due();
    observer_backlog_does_not_block_the_release();
    handoff_between_selection_and_call();
    handoff_exactly_at_the_due();
    handoff_time_is_read_under_the_lock();
    clock_failure_at_the_handoff_is_refused();
    stop_discards_a_waiting_body();
    stop_lets_a_put_in_progress_finish();
    stop_while_the_release_thread_is_held();
    close_after_the_commitment_still_makes_the_one_call();
    fault_discards_what_is_not_committed();
    close_without_a_thread_accounts_the_slot();
    first_frame_channel_refusal_is_latched();
    last_frame_channel_refusal_is_latched();
    broken_return_clock_is_a_fault_after_the_attempt();
    transport_outcomes_are_counted_not_retried();
    unusable_ready_time_is_refused_and_latched();
    wire_bytes_equal_a_direct_publish(7, 0);
    wire_bytes_equal_a_direct_publish(12, 300);   // long class names
    wire_bytes_equal_a_direct_publish(kMaxObjects, 0);  // the contract's storage limit
    same_shape_bodies_do_not_allocate();
    waiting_shows_as_an_open_gap_to_status();
    prepare_failure_is_a_refusal_without_an_attempt();
    status_sees_a_blocked_put_with_a_waiting_body();
    status_is_busy_only_inside_the_commit_step();
    for (unsigned seed = 1; seed <= 12; ++seed) concurrent_producer_transport_and_stop(seed);
    std::cout << "test_rns_objects_release: PASS checks=" << checks.load() << "\n";
    return 0;
}
