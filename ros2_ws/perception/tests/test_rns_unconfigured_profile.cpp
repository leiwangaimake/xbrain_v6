/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_unconfigured_profile.cpp
 * Brief: Unit tests for the geometry-unconfigured Profile owner (no extrinsic: FREE withheld, health kept).
 *
 * Description:
 * What it checks. Without an extrinsic the service still publishes a Profile carrying raw depth
 * health and no geometric FREE. The tests cover taking the latest frame, empty frames and raw
 * health; source rejection and stream restart; malformed frames and capacity; capture mapping and
 * ordering; clock stages and their brackets; put failures that are never retried; the processed-
 * health handoff to Status; and configuration errors.
 */

/* Synthetic clocks/raw rasters ONLY. No camera, network, calibrated geometry,
 * real exposure mapping, per-bin G evidence or 20 Hz acceptance. */
#include "output/rns_unconfigured_profile.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>
#include <string_view>

namespace {
std::atomic<std::size_t> allocations{0};
void* alloc(std::size_t n) {
    void* p = std::malloc(n ? n : 1);
    if (!p) throw std::bad_alloc();
    ++allocations; return p;
}
void* aligned(std::size_t n, std::size_t a) {
    void* p = nullptr;
    if (posix_memalign(&p, a, n ? n : 1)) throw std::bad_alloc();
    ++allocations; return p;
}
[[noreturn]] void fail(const char* expr, int line) {
    std::cerr << "unconfigured Profile CHECK " << line << ": " << expr << '\n';
    std::abort();
}
#define CHECK(x) do { if (!(x)) fail(#x, __LINE__); } while (false)
}
void* operator new(std::size_t n) { return alloc(n); }
void* operator new[](std::size_t n) { return alloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) { return aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::rns;
namespace nd = perception::native_depth;
using Tick = UnconfiguredProfileTick;
using State = UnconfiguredProfileStatus;
std::size_t steps = 0;
constexpr std::size_t pixels = 12;
UnconfiguredProfileDomain domain() { return {.5F, -.25F, .125F, 4.F, 3, 42}; }

struct Clock final : PublishClock {
    mutable std::int64_t ns = 10000000000LL;
    mutable std::size_t calls = 0;
    std::size_t bad_at = 0, rollback_at = 0, stop_at = 0;
    nd::Mailbox* mailbox = nullptr;
    perception::geometry::CaptureStamp expected;
    int mutation = 0;
    bool fail_next_publish_sample = false;
    Millis publish_offset_ms = 0;
    Millis wall_millis() noexcept override { return 1700000000123; }
    Millis mono_millis() noexcept override {
        if (fail_next_publish_sample) { fail_next_publish_sample = false; return -1; }
        return ns / 1000000 + publish_offset_ms;
    }
    static std::int64_t now(const void* p) noexcept {
        const auto& c = *static_cast<const Clock*>(p);
        ++c.calls;
        if (c.stop_at == c.calls) c.mailbox->end_stream();
        if (c.bad_at == c.calls) return -1;
        if (c.rollback_at == c.calls) return c.ns - 1000000000;
        c.ns += 100000;
        return c.ns;
    }
    static std::int64_t health_now(void* p) noexcept { return now(p); }
    static bool resolve(const void* p, const nd::Metadata& m,
                        perception::geometry::CaptureStamp& out) noexcept {
        const auto& c = *static_cast<const Clock*>(p);
        CHECK(m.source.sequence == c.expected.source.sequence);
        out = c.expected; // frozen synthetic mapping; NOT a camera clock formula
        switch (c.mutation) {
        case 1: return false;
        case 2: ++out.source.stream_id; break;
        case 3: ++out.source.sequence; break;
        case 4: ++out.source.color_index; break;
        case 5: ++out.source.depth_index; break;
        case 6: ++out.source.color_device_us; break;
        case 7: ++out.source.depth_device_us; break;
        case 8: ++out.source.host_ready_mono_ns; break;
        case 9: ++out.clock_epoch; break;
        case 10: out.capture_mono_ns = m.source.host_ready_mono_ns + 1; break;
        case 11: out.capture_mono_ns = -1; break;
        }
        return true;
    }
    NativeProfileClock config() { return {this, 42, now, resolve}; }
};
struct Sink final : RnsTransport {
    std::array<char, 65536> packet{};
    std::size_t calls = 0, size = 0;
    PutOutcome outcome = PutOutcome::Accepted;
    Clock* fail_return_clock = nullptr;
    Clock* delay_clock = nullptr;
    SourceJournal* required_health = nullptr;
    bool throw_put = false;
    PutReceipt put(std::string_view key, std::string_view bytes, PublishQos q) override {
        CHECK(key == "xbrain/robot_1/rt/perception/profile");
        CHECK(q.priority == Priority::RealTime && q.drop && q.express && !q.reliable);
        CHECK(bytes.size() < packet.size());
        if (required_health) {
            SourceEvent e;
            CHECK(required_health->peek(e) && e.depth.code == DepthHealthCode::Ready);
            CHECK(e.kind == SourceKind::DepthProcessed && e.observed_ns <= delay_clock->ns);
            delay_clock->ns += 2000000000LL; // controlled synthetic slow put
        }
        ++calls; size = bytes.size();
        std::memcpy(packet.data(), bytes.data(), size); packet[size] = 0;
        if (fail_return_clock) fail_return_clock->fail_next_publish_sample = true;
        if (throw_put) throw 7; // no C++ operator-new allocation in this fixture
        return {outcome, 17};
    }
    std::string_view view() const { return {packet.data(), size}; }
};
struct Fixture {
    std::array<std::uint16_t, pixels> raster{};
    nd::Mailbox mailbox{pixels};
    Clock clock;
    Sink sink;
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublisherChannel channel;
    SourceJournal depth_health;
    UnconfiguredProfileConsumer consumer;
    std::uint64_t stream = 0, seq = 0;
    explicit Fixture(bool health = false, std::size_t journal_capacity = 4,
                     std::size_t body_capacity = 32768)
        : channel(Topic::Profile, envelope, clock, sink, body_capacity, 65536),
          depth_health(SourceKind::DepthProcessed, journal_capacity, {&clock, Clock::health_now}),
          consumer(mailbox, pixels, domain(), clock.config(), channel, health ? &depth_health : nullptr) {
        stream = mailbox.begin_stream(); clock.mailbox = &mailbox;
        raster.fill(1024);
    }
    nd::View next() {
        clock.ns += 10000000;
        nd::View v;
        v.metadata.intrinsics = {2, 3, 1, 1, 4, 3};
        v.metadata.distortion.reported = true;
        v.metadata.unit_m = 1.F / 1024;
        auto& s = v.metadata.source;
        s.stream_id = stream; s.sequence = ++seq;
        s.color_index = seq - 1; s.depth_index = seq - 1;
        s.color_device_us = seq * 1000; s.depth_device_us = seq * 1000 + 1;
        s.host_ready_mono_ns = clock.ns - 1000000;
        clock.expected = {s, 42, clock.ns - 2000000};
        v.pre_d2c = true; v.pixels = raster.data(); v.bytes = sizeof(raster);
        return v;
    }
    nd::Offer offer() { return mailbox.try_offer(next()); }
    Tick step() {
        const auto before = allocations.load();
        auto r = consumer.step(); ++steps;
        CHECK(allocations.load() == before);
        CHECK(r.mode == UnconfiguredProfileMode::GeometryUnconfigured);
        CHECK(r.reason == "geometry_unconfigured" && !r.geometry_ran && !r.grid_ran);
        return r;
    }
    void restart() { mailbox.end_stream(); stream = mailbox.begin_stream(); seq = 0; }
};
void success(const Tick& r) {
    CHECK(r.status == State::PutFinished && r.take_called && r.take == nd::Take::Taken);
    CHECK(r.depth_health_ran && r.capture_mapping_ran && r.build_ran);
    CHECK(r.put.dispatched && r.put.publish && r.put.publish.audit.attempted);
    CHECK(r.capture.capture_mono_ns <= r.source.host_ready_mono_ns);
    CHECK(r.source.host_ready_mono_ns <= r.taken_ns && r.begin_ns <= r.taken_ns);
    CHECK(r.taken_ns <= r.health_end_ns && r.health_end_ns <= r.capture_checked_ns);
    CHECK(r.capture_checked_ns <= r.build_end_ns && r.build_end_ns <= r.end_ns);
    CHECK(r.put.publish.audit.capture_ms == r.capture.capture_mono_ns / 1000000);
    CHECK(r.publish_clock_bracket_valid);
}
void unknown(const Sink& sink) {
    const auto p = sink.view();
    for (const auto field : {"\"d_free\":[null,null,null]", "\"d_block\":[null,null,null]",
            "\"h_block\":[null,null,null]", "\"slope_deg\":[null,null,null]",
            "\"src\":[0,0,0]", "\"conf\":[0,0,0]", "\"terrain\":[0,0,0]",
            "\"blind_near_m\":null", "\"t_seg_mono_ms\":null", "\"pose_used\":null",
            "\"extrinsic_calibrated\":false", "\"n_bins\":3", "\"range_max_m\":4"})
        CHECK(p.find(field) != std::string_view::npos);
    CHECK(p.find("ground_fit_fallback") == std::string_view::npos);
}
// Only the newest frame is published, as an all-UNKNOWN Profile with raw lower-half depth health
// (all-zero depth counts as fully invalid, not unobserved); other raw metadata never creates G
// bins.
void test_latest_empty_and_raw_health(bool emit = false) {
    Fixture f;
    auto r = f.step();
    CHECK(r.status == State::Idle && !r.depth_health_ran && !r.put.dispatched);
    CHECK(!r.depth_health.invalid_ratio && r.source.stream_id == 0 && f.sink.calls == 0);
    CHECK(f.offer() == nd::Offer::Accepted && f.offer() == nd::Offer::Replaced);
    r = f.step(); success(r); unknown(f.sink);
    CHECK(r.source.sequence == 2 && r.depth_health.source.sequence == 2);
    CHECK(r.depth_health.roi_kind == HealthRoiKind::LowerHalfUncalibrated);
    CHECK(r.depth_health.roi_pixels == 8 && r.depth_health.invalid_ratio == 0);
    CHECK(!r.depth_health_recorded && f.depth_health.published() == 0);
    CHECK(r.put.publish.audit.capture_ms != r.source.host_ready_mono_ns / 1000000);
    CHECK(f.consumer.storage_bytes() == pixels * sizeof(std::uint16_t) + 3 * sizeof(Bin));
    if (emit) std::cout << "{\"case\":\"geometry_unconfigured_nonzero\",\"packet\":"
        << f.sink.view() << ",\"camera\":false,\"network\":false}\n";
    r = f.step();
    CHECK(r.status == State::Idle && !r.build_ran && !r.put.dispatched && f.sink.calls == 1);
    CHECK(r.source.stream_id == 0 && r.capture.capture_mono_ns == -1);
    f.raster.fill(0); CHECK(f.offer() == nd::Offer::Accepted);
    r = f.step(); success(r); unknown(f.sink);
    CHECK(r.depth_health.roi_pixels == 8 && r.depth_health.invalid_pixels == 8);
    CHECK(r.depth_health.invalid_ratio == 1); // observed invalid, not no observation
    if (emit) std::cout << "{\"case\":\"geometry_unconfigured_allzero\",\"packet\":"
        << f.sink.view() << ",\"camera\":false,\"network\":false}\n";
    f.raster[4] = 100; f.raster[11] = 65535;
    CHECK(f.offer() == nd::Offer::Accepted); r = f.step(); success(r); unknown(f.sink);
    CHECK(r.depth_health.invalid_ratio == .75);
    // Different raw profile metadata cannot create calibrated rays or G bins.
    auto v = f.next(); v.metadata.intrinsics = {5, 7, 0, 2, 2, 6};
    v.metadata.unit_m = .001F; v.metadata.distortion.model = 99;
    CHECK(f.mailbox.try_offer(v) == nd::Offer::Accepted);
    r = f.step(); success(r); unknown(f.sink);
    CHECK(r.depth_health.roi_pixels == 6);
    const auto c = f.consumer.counts();
    CHECK(c.taken == 4 && c.health_scanned == 4 && c.accepted == 4 && c.attempted == 4);
}
// Old or repeated frames are refused; after a restart an old capture time cannot become a new wire
// identity; a stop before dispatch sends nothing.
void test_source_rejection_and_restart() {
    Fixture f;
    const auto first = f.next(); CHECK(f.mailbox.try_offer(first) == nd::Offer::Accepted);
    const auto initial = f.step(); success(initial);
    CHECK(f.mailbox.try_offer(first) == nd::Offer::Stale);
    CHECK(f.step().status == State::Idle && f.sink.calls == 1);
    auto stale = f.next(); stale.metadata.source.depth_index = first.metadata.source.depth_index;
    CHECK(f.mailbox.try_offer(stale) == nd::Offer::Stale);
    CHECK(f.step().status == State::Idle && f.sink.calls == 1);
    f.restart();
    CHECK(f.mailbox.try_offer(first) == nd::Offer::Stale);
    CHECK(f.offer() == nd::Offer::Accepted); auto r = f.step(); success(r);
    CHECK(r.source.stream_id == 2 && r.source.sequence == 1 && r.source.depth_index == 0);
    // Restart cannot make an old MONOTONIC capture a new wire identity.
    f.restart(); CHECK(f.offer() == nd::Offer::Accepted);
    f.clock.expected.capture_mono_ns = initial.capture.capture_mono_ns;
    r = f.step(); CHECK(r.status == State::ClockRejected && !r.build_ran && f.sink.calls == 2);
    CHECK(f.step().status == State::Idle && f.sink.calls == 2);
    CHECK(f.offer() == nd::Offer::Accepted); success(f.step());
    // Stopping while the consumer is active is forbidden by the ownership
    // contract; guards still catch stop before processing or before dispatch.
    for (std::size_t stage : {2, 3, 4, 5}) {
        CHECK(f.offer() == nd::Offer::Accepted);
        f.clock.stop_at = f.clock.calls + stage;
        r = f.step(); CHECK(r.status == State::SourceUnavailable && !r.put.dispatched);
        f.clock.stop_at = 0;
        CHECK(f.step().status == State::SourceUnavailable);
        f.restart();
    }
    CHECK(f.offer() == nd::Offer::Accepted); success(f.step());
    CHECK(f.sink.calls == 4);
}
// Malformed frames are refused by the mailbox and nothing is processed; a too-small consumer fails
// the take without allocating or consuming the pending frame.
void test_malformed_frames_and_capacity() {
    Fixture f;
    for (int mutation = 0; mutation < 8; ++mutation) {
        auto v = f.next();
        if (mutation == 0) v.pixels = nullptr;
        if (mutation == 1) v.bytes = 0;
        if (mutation == 2) v.metadata.intrinsics.width = 0;
        if (mutation == 3) v.metadata.intrinsics.height = 0;
        if (mutation == 4) v.metadata.unit_m = 0;
        if (mutation == 5) v.metadata.distortion.reported = false;
        if (mutation == 6) v.pre_d2c = false;
        if (mutation == 7) v.metadata.source.depth_device_us = 0;
        CHECK(f.mailbox.try_offer(v) == nd::Offer::Invalid);
        const auto r = f.step();
        CHECK(r.status == State::Idle && !r.depth_health_ran && !r.build_ran);
    }
    CHECK(f.sink.calls == 0);
    UnconfiguredProfileConsumer wrong_capacity(f.mailbox, pixels - 1, domain(), f.clock.config(), f.channel);
    CHECK(f.offer() == nd::Offer::Accepted);
    const auto before = allocations.load();
    const auto bad = wrong_capacity.step();
    CHECK(allocations.load() == before);
    CHECK(bad.status == State::SourceUnavailable && bad.take == nd::Take::Capacity && !bad.put.dispatched);
    success(f.step()); // failed take did not consume the real pending frame
}
// An invalid capture mapping, a capture time not in a newer millisecond than the last put, or a
// host-ready time in the future rejects the frame and sends nothing.
void test_capture_mapping_and_ordering() {
    Fixture f(true, 32);
    CHECK(f.offer() == nd::Offer::Accepted); const auto first = f.step(); success(first);
    for (int mutation = 1; mutation <= 11; ++mutation) {
        f.clock.mutation = mutation; CHECK(f.offer() == nd::Offer::Accepted);
        const auto r = f.step();
        CHECK(r.status == State::ClockRejected && r.capture_mapping_ran && !r.build_ran);
        CHECK(r.depth_health_recorded && !r.put.dispatched && f.sink.calls == 1);
        CHECK(f.step().status == State::Idle && f.sink.calls == 1);
    }
    f.clock.mutation = 0;
    for (const auto capture : {first.capture.capture_mono_ns, first.capture.capture_mono_ns - 1,
                              first.capture.capture_mono_ns + 1}) {
        CHECK(f.offer() == nd::Offer::Accepted);
        f.clock.expected.capture_mono_ns = capture;
        const auto r = f.step();
        CHECK(r.status == State::ClockRejected && !r.build_ran && !r.put.dispatched);
    }
    CHECK(f.offer() == nd::Offer::Accepted); success(f.step());
    auto future_ready = f.next();
    future_ready.metadata.source.host_ready_mono_ns = f.clock.ns + 100000000;
    f.clock.expected.source = future_ready.metadata.source;
    CHECK(f.mailbox.try_offer(future_ready) == nd::Offer::Accepted);
    const auto bad = f.step();
    CHECK(bad.status == State::ClockRejected && !bad.depth_health_ran && !bad.put.dispatched);
    CHECK(f.consumer.counts().clock_rejected == 15);
}
// A bad clock read at any stage rejects the frame (before the take the frame stays pending; after
// the put the receipt stays visible); a rollback, a publish clock from another domain or a failed
// before/after sample is never reported as success.
void test_clock_stages_and_brackets() {
    for (std::size_t stage = 1; stage <= 6; ++stage) {
        Fixture f; CHECK(f.offer() == nd::Offer::Accepted);
        f.clock.bad_at = f.clock.calls + stage;
        const auto r = f.step(); CHECK(r.status == State::ClockRejected);
        CHECK(r.put.dispatched == (stage == 6));
        if (stage == 6) {
            CHECK(r.put.publish.audit.receipt.outcome == PutOutcome::Accepted);
            CHECK(f.consumer.counts().accepted == 1);
        }
        f.clock.bad_at = 0;
        if (stage == 1) success(f.step()); // pre-take error leaves current frame pending
        else CHECK(f.step().status == State::Idle && f.sink.calls == (stage == 6 ? 1U : 0U));
    }
    Fixture rollback;
    CHECK(rollback.offer() == nd::Offer::Accepted); success(rollback.step());
    CHECK(rollback.offer() == nd::Offer::Accepted);
    rollback.clock.rollback_at = rollback.clock.calls + 3;
    CHECK(rollback.step().status == State::ClockRejected && rollback.sink.calls == 1);
    rollback.clock.rollback_at = 0;
    CHECK(rollback.step().status == State::Idle && rollback.sink.calls == 1);
    Fixture offset; CHECK(offset.offer() == nd::Offer::Accepted);
    offset.clock.publish_offset_ms = 1000;
    auto r = offset.step();
    CHECK(r.status == State::ClockRejected && r.put.publish && r.put.dispatched);
    CHECK(!r.publish_clock_bracket_valid && offset.consumer.counts().accepted == 1);
    CHECK(offset.step().status == State::Idle && offset.sink.calls == 1);
    Fixture after; CHECK(after.offer() == nd::Offer::Accepted);
    after.sink.fail_return_clock = &after.clock;
    r = after.step();
    CHECK(r.status == State::ClockRejected && r.put.publish.error == PublishError::ClockAfter);
    CHECK(r.put.publish.audit.receipt.outcome == PutOutcome::Accepted);
    CHECK(after.consumer.counts().accepted == after.channel.counters().accepted);
    CHECK(after.consumer.counts().accepted == 1 && after.sink.calls == 1);
    after.sink.fail_return_clock = nullptr;
    CHECK(after.step().status == State::Idle && after.sink.calls == 1);
    Fixture before; CHECK(before.offer() == nd::Offer::Accepted);
    before.clock.fail_next_publish_sample = true;
    r = before.step();
    CHECK(r.put.dispatched && !r.put.publish.audit.attempted && before.sink.calls == 0);
    CHECK(r.put.publish.error == PublishError::ClockBefore && before.consumer.counts().accepted == 0);
    CHECK(before.step().status == State::Idle && before.sink.calls == 0);
}
// Rejected, unknown or throwing puts are counted and never retried; a body too large for the
// channel is not attempted.
void test_put_failures_never_retry() {
    Fixture f;
    for (const auto outcome : {PutOutcome::Rejected, PutOutcome::Unknown}) {
        f.sink.outcome = outcome; CHECK(f.offer() == nd::Offer::Accepted);
        const auto r = f.step();
        CHECK(r.status == State::PutFinished && r.put.dispatched && r.put.publish.audit.attempted);
        CHECK(!r.put.publish && r.put.publish.audit.receipt.outcome == outcome);
        CHECK(r.publish_clock_bracket_valid);
        const auto calls = f.sink.calls;
        CHECK(f.step().status == State::Idle && f.sink.calls == calls);
    }
    f.sink.throw_put = true; CHECK(f.offer() == nd::Offer::Accepted);
    const auto thrown = f.step();
    CHECK(thrown.put.publish.error == PublishError::PutUnknown && thrown.put.publish.audit.threw);
    CHECK(f.step().status == State::Idle && f.sink.calls == 3);
    f.sink.throw_put = false; f.sink.outcome = PutOutcome::Accepted;
    CHECK(f.offer() == nd::Offer::Accepted); success(f.step()); unknown(f.sink);
    const auto c = f.consumer.counts();
    CHECK(c.dispatches == 4 && c.attempted == 4 && c.accepted == 1 && c.rejected == 1 && c.unknown == 2);
    CHECK(c.accepted == f.channel.counters().accepted);
    Fixture small(false, 4, 16); CHECK(small.offer() == nd::Offer::Accepted);
    const auto r = small.step();
    CHECK(r.put.dispatched && r.put.publish.error == PublishError::Capacity && !r.put.publish.audit.attempted);
    CHECK(small.consumer.counts().dispatches == 1 && small.consumer.counts().attempted == 0);
    CHECK(small.step().status == State::Idle && small.sink.calls == 0);
}
// Health is recorded once per processed frame (the newest only) before the put, all-invalid depth
// included; a full health queue shows the overflow but the Profile still goes out.
void test_processed_health_handoff() {
    Fixture f(true);
    CHECK(f.offer() == nd::Offer::Accepted && f.offer() == nd::Offer::Replaced);
    f.sink.required_health = &f.depth_health; f.sink.delay_clock = &f.clock;
    const auto r = f.step(); success(r);
    CHECK(r.depth_health_recorded && f.depth_health.published() == 1);
    SourceEvent e; CHECK(f.depth_health.pop(e));
    CHECK(e.source.sequence == 2 && e.source.depth_index == r.source.depth_index);
    CHECK(e.depth.source.sequence == r.depth_health.source.sequence);
    CHECK(e.observed_ns >= r.taken_ns && e.observed_ns <= r.health_end_ns);
    CHECK(e.observed_ns < r.capture_checked_ns && r.end_ns - e.observed_ns >= 2000000000LL);
    CHECK(e.depth.invalid_ratio == 0 && e.depth.roi_pixels == 8);
    f.sink.required_health = nullptr;
    CHECK(f.step().status == State::Idle && f.depth_health.published() == 1);
    f.raster.fill(0); CHECK(f.offer() == nd::Offer::Accepted); success(f.step());
    CHECK(f.depth_health.pop(e) && e.depth.invalid_ratio == 1 && e.depth.invalid_pixels == 8);
    Fixture full(true, 1);
    CHECK(full.offer() == nd::Offer::Accepted); success(full.step());
    CHECK(full.offer() == nd::Offer::Accepted); const auto overflow = full.step(); success(overflow);
    CHECK(overflow.depth_health_ran && !overflow.depth_health_recorded);
    CHECK(!full.depth_health.valid() && full.depth_health.overflow_count() == 1);
    CHECK(full.sink.calls == 2 && full.consumer.counts().health_recorded == 1);
}
// Invalid domain, clock, capacity, channel or journal settings are refused at construction,
// without sampling a clock or putting.
void test_configuration() {
    Fixture f;
    const auto reject = [&](UnconfiguredProfileDomain d, NativeProfileClock c,
                            std::size_t capacity, PublisherChannel& channel, SourceJournal* journal = nullptr) {
        bool caught = false;
        try { UnconfiguredProfileConsumer bad(f.mailbox, capacity, d, c, channel, journal); }
        catch (const std::invalid_argument&) { caught = true; }
        CHECK(caught);
    };
    reject({}, f.clock.config(), pixels, f.channel);
    for (int mutation = 0; mutation < 14; ++mutation) {
        auto d = domain();
        if (mutation == 0) d.z_pass_m = 0;
        if (mutation == 1) d.z_pass_m = std::numeric_limits<float>::infinity();
        if (mutation == 2) d.angle_min_rad = std::numeric_limits<float>::quiet_NaN();
        if (mutation == 3) d.angle_step_rad = 0;
        if (mutation == 4) d.angle_step_rad = -1;
        if (mutation == 5) d.angle_step_rad = std::numeric_limits<float>::infinity();
        if (mutation == 6) d.range_max_m = 0;
        if (mutation == 7) d.range_max_m = 6.01F;
        if (mutation == 8) d.range_max_m = std::numeric_limits<float>::quiet_NaN();
        if (mutation == 9) d.n_bins = 0;
        if (mutation == 10) d.n_bins = kMaxBins + 1;
        if (mutation == 11) d.clock_epoch = 0;
        if (mutation == 12) d.clock_epoch = 43;
        if (mutation == 13) d.angle_step_rad = std::numeric_limits<float>::max();
        reject(d, f.clock.config(), pixels, f.channel);
    }
    for (int mutation = 0; mutation < 4; ++mutation) {
        auto c = f.clock.config();
        if (mutation == 0) c.context = nullptr;
        if (mutation == 1) c.epoch = 0;
        if (mutation == 2) c.now_ns = nullptr;
        if (mutation == 3) c.capture = nullptr;
        reject(domain(), c, pixels, f.channel);
    }
    reject(domain(), f.clock.config(), 0, f.channel);
    reject(domain(), f.clock.config(), nd::kMaxPixels + 1, f.channel);
    PublisherChannel wrong(Topic::Status, f.envelope, f.clock, f.sink, 4096, 8192);
    PublisherChannel invalid(Topic::Profile, f.envelope, f.clock, f.sink, 0, 8192);
    reject(domain(), f.clock.config(), pixels, wrong);
    reject(domain(), f.clock.config(), pixels, invalid);
    SourceJournal arrival(SourceKind::DepthArrival, 2, {&f.clock, Clock::health_now});
    SourceJournal no_clock(SourceKind::DepthProcessed, 2, {});
    reject(domain(), f.clock.config(), pixels, f.channel, &arrival);
    reject(domain(), f.clock.config(), pixels, f.channel, &no_clock);
    CHECK(f.clock.calls == 0 && f.sink.calls == 0); // cold validation never samples/puts
}
}
int main(int argc, char** argv) {
    const bool emit = argc == 2 && std::string_view(argv[1]) == "--fixtures";
    test_latest_empty_and_raw_health(emit);
    if (emit) return 0;
    test_source_rejection_and_restart(); test_malformed_frames_and_capacity();
    test_capture_mapping_and_ordering(); test_clock_stages_and_brackets();
    test_put_failures_never_retry(); test_processed_health_handoff(); test_configuration();
    std::cout << "GeometryUnconfigured consumer 8/8 groups PASS; " << steps
        << " steps C++ allocation delta=0; all UNKNOWN, zero geometry/grid owners; "
        << "synthetic clocks/rawdepth; NOT camera/network/20Hz\n";
}
