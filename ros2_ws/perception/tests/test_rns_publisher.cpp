/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_publisher.cpp
 * Brief: Publisher channel tests with injected clocks and transport; no device or network.
 *
 * Description:
 * What it checks. PublisherChannel encodes a body, stamps the envelope and puts it once. Cases
 * cover the serialized fixtures, failures and timing, negative inputs and capacity, allocation and
 * separate owners, and the preparation boundary.
 * Scope. No receive-side PASS is implied.
 */

#include "output/rns_publisher.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <thread>

static std::atomic<std::size_t> allocations{0};
void* operator new(std::size_t n) { ++allocations; if (auto* p = std::malloc(n ? n : 1)) return p; throw std::bad_alloc(); }
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
using namespace perception::rns;
namespace perception::rns {
struct ProcessEnvelopeTestAccess {
    static void sequence(ProcessEnvelope& e, std::uint64_t n) { e.seq_.store(n); }
};
}

struct FakeClock final : PublishClock {
    Millis wall = 1700000000123, mono = 30000;
    unsigned wall_reads = 0, mono_reads = 0;
    Millis wall_millis() noexcept override { ++wall_reads; return wall; }
    Millis mono_millis() noexcept override { ++mono_reads; return mono; }
};
struct Sink final : RnsTransport {
    explicit Sink(FakeClock& c) : clock(c) {}
    FakeClock& clock;
    std::array<char, 65536> bytes{};
    std::array<char, 100> key{};
    PublishQos last_qos{Priority::DataHigh, true, false};
    std::size_t size = 0, calls = 0;
    Millis delay = 2, entry = -1;
    PutOutcome outcome = PutOutcome::Accepted;
    bool throws = false;
    std::atomic<bool>* entered = nullptr;
    std::atomic<bool>* release = nullptr;
    PutReceipt put(std::string_view k, std::string_view p, PublishQos qos) override {
        ++calls; entry = clock.mono; last_qos = qos;
        assert(p.size() < bytes.size() && k.size() < key.size());
        std::memcpy(bytes.data(), p.data(), p.size()); bytes[p.size()] = 0; size = p.size();
        std::memcpy(key.data(), k.data(), k.size()); key[k.size()] = 0;
        if (entered) {
            entered->store(true, std::memory_order_release);
            while (!release->load(std::memory_order_acquire)) std::this_thread::yield();
        }
        clock.mono += delay;
        if (throws) throw 7;
        return {outcome, outcome == PutOutcome::Rejected ? 12 : 0};
    }
};
struct PreparedSink final : RnsTransport {
    explicit PreparedSink(FakeClock& clock) : sink(clock) {}
    Sink sink;
    std::array<char, 65536> owned{};
    Millis prepare_delay = 37;
    bool reject = false, throw_prepare = false, active = false;
    unsigned prepared = 0, cleaned = 0;
    PutReceipt put(std::string_view k, std::string_view p, PublishQos q) override { return sink.put(k, p, q); }
    PutReceipt put_finalized(std::string_view k, char* p, std::size_t n,
                           PublishQos q, PacketFinalizer& finalizer) override {
        assert(!active && n <= owned.size());
        active = true; ++prepared;
        struct Cleanup { PreparedSink& s; ~Cleanup() { s.active = false; ++s.cleaned; } } cleanup{*this};
        std::memcpy(owned.data(), p, n);
        sink.clock.mono += prepare_delay;
        if (throw_prepare) throw 8;
        if (reject) return {PutOutcome::Rejected, 91};
        if (!finalizer.finalize(owned.data(), n)) return {PutOutcome::Rejected, 92};
        return sink.put(k, {owned.data(), n}, q);
    }
};
Profile sample_profile() {
    Profile p;
    p.t_capture_mono_ms = 29900; // publish deliberately unset: stamped by adapter
    p.z_pass_m = .75F; p.range_max_m = 6; p.blind_near_m = .5F;
    p.angle_min_rad = -.01F; p.angle_step_rad = .01F; p.bins.resize(3);
    p.bins[0].src = kS; p.bins[0].d_block = 2.5F; p.bins[2].src = kG;
    return p;
}
Objects sample_objects(Millis capture) { Objects b; b.t_capture_mono_ms = capture; return b; }
Status sample_status() { Status b; b.fps_depth = 30; b.fps_infer = 25; return b; }
void print_fixture(const Sink& sink, const PublishResult& r) {
    std::cout << "{\"key\":\"" << sink.key.data() << "\",\"packet\":" << sink.bytes.data()
              << ",\"before\":" << r.audit.put_before_ms << ",\"after\":" << r.audit.put_return_ms
              << ",\"capture\":" << r.audit.capture_ms << ",\"outcome\":\"accepted\""
              << ",\"qos\":{\"drop\":" << (sink.last_qos.drop ? "true" : "false")
              << ",\"reliable\":" << (sink.last_qos.reliable ? "true" : "false")
              << ",\"express\":" << (sink.last_qos.express ? "true" : "false")
              << ",\"priority\":\"" << (sink.last_qos.priority == Priority::RealTime ? "real_time" : "data_high")
              << "\"}}\n";
}
void fixtures() {
    ProcessEnvelope env("robot_1", "abcdef01", 5000); assert(env.note_clock_status(true, 29999));
    FakeClock clock; Sink sink(clock); ObjectsCadence cadence(0, 512);
    PublisherChannel p(Topic::Profile, env, clock, sink, 32768, 65536);
    PublisherChannel o(Topic::Objects, env, clock, sink, 32768, 65536, &cadence);
    PublisherChannel s(Topic::Status, env, clock, sink, 32768, 65536);
    auto pr = p.publish(sample_profile()); assert(pr); print_fixture(sink, pr);
    clock.mono = 30040; auto ob = o.publish(sample_objects(29940)); assert(ob); print_fixture(sink, ob);
    clock.mono = 31000; auto st = s.publish(sample_status()); assert(st); print_fixture(sink, st);
}
void failures_and_timing() {
    ProcessEnvelope env("robot_1", "abcdef01", 5000);
    FakeClock clock; Sink sink(clock); ObjectsCadence cadence(0, 512);
    PublisherChannel channel(Topic::Objects, env, clock, sink, 32768, 65536, &cadence);
    auto body = sample_objects(29900);
    auto first = channel.publish(body);
    assert(first && first.audit.seq == 1 && first.audit.capture_ms == 29900);
    assert(first.audit.put_before_ms == 30000 && sink.entry == 30000 && first.audit.put_return_ms == 30002);
    assert(first.audit.clock_valid && first.audit.packet_bytes == sink.size && first.audit.cadence_event == CadenceEvent::New);
    assert(body.t_publish_mono_ms == -1 && clock.wall_reads == 1 && clock.mono_reads == 2);
    assert(std::string_view(sink.key.data()) == "xbrain/robot_1/rt/perception/objects");
    assert(std::string_view(sink.bytes.data()).find("\"mono\":30.000") != std::string_view::npos);
    assert(std::string_view(sink.bytes.data()).find("\"ts\":1700000000.123") != std::string_view::npos);
    assert(std::string_view(sink.bytes.data()).find("\"ts_sync\":false") != std::string_view::npos);
    assert(sink.last_qos.drop && sink.last_qos.express && !sink.last_qos.reliable);
    assert(channel.publish(body).error == PublishError::Duplicate);
    assert(channel.publish(sample_objects(29899)).error == PublishError::OutOfOrder);
    assert(channel.publish(sample_status()).error == PublishError::WrongTopic);
    assert(sink.calls == 1);
    clock.mono = 30101;
    assert(cadence.snapshot(clock.mono).open_gap_ms == 101); // no duplicate refresh
    clock.mono = 30110; sink.outcome = PutOutcome::Rejected;
    auto rejected = channel.publish(sample_objects(30100));
    assert(rejected.error == PublishError::PutRejected && rejected.audit.attempted && rejected.audit.seq == 2);
    assert(rejected.audit.receipt.native_code == 12);
    assert(channel.publish(sample_objects(30100)).error == PublishError::Duplicate); // NO retry
    assert(cadence.snapshot(clock.mono).max_gap_ms == 110);
    clock.mono = 30150; sink.throws = true;
    auto uncertain = channel.publish(sample_objects(30140));
    assert(uncertain.error == PublishError::PutUnknown && uncertain.audit.threw && uncertain.audit.attempted);
    assert(uncertain.audit.receipt.outcome == PutOutcome::Unknown);
    sink.throws = false; sink.outcome = PutOutcome::Unknown; clock.mono = 30190;
    assert(channel.publish(sample_objects(30180)).error == PublishError::PutUnknown);
    sink.outcome = PutOutcome::Accepted; clock.mono = 30230; sink.delay = -1;
    auto bad_after = channel.publish(sample_objects(30220));
    assert(bad_after.error == PublishError::ClockAfter && bad_after.audit.attempted && !bad_after.audit.clock_valid);
    const auto calls = sink.calls;
    assert(channel.publish(sample_objects(30221)).error == PublishError::ClockBefore);
    assert(sink.calls == calls);
    clock.mono = 30300; sink.delay = 2;
    assert(channel.publish(sample_objects(30301)).error == PublishError::ClockBefore);
    assert(channel.publish(sample_objects(-1)).error == PublishError::InvalidBody);
    const auto stats = channel.counters();
    assert(stats.attempted == 5 && stats.accepted == 2 && stats.rejected == 1 && stats.unknown == 2);
    assert(stats.clock_faults == 3 && stats.duplicates == 2 && stats.out_of_order == 1);
    // Unknown transport outcome must survive a simultaneous return-clock fault.
    clock.mono = 30400; sink.throws = true; sink.delay = -1;
    const auto both = channel.publish(sample_objects(30390));
    assert(both.error == PublishError::ClockAfter && both.audit.threw && both.audit.attempted);
    assert(both.audit.receipt.outcome == PutOutcome::Unknown && !both.audit.clock_valid);
    assert(channel.counters().unknown == 3 && channel.counters().clock_faults == 4);
}
void negative_inputs_and_capacity() {
    ProcessEnvelope env("robot_1", "abcdef01", 5000);
    FakeClock clock; Sink sink(clock);
    auto body = sample_profile();
    PublisherChannel p(Topic::Profile, env, clock, sink, 32768, 65536);
    auto bad = body; bad.bins[0].d_free = 2;
    assert(p.publish(bad).error == PublishError::InvalidBody && sink.calls == 0);
    PublisherChannel body_small(Topic::Profile, env, clock, sink, 2, 65536);
    assert(body_small.publish(body).error == PublishError::Capacity && sink.calls == 0);
    PublisherChannel wire_small(Topic::Profile, env, clock, sink, 32768, 2);
    assert(wire_small.publish(body).error == PublishError::Capacity && sink.calls == 0);
    PublisherChannel no_buffer(Topic::Profile, env, clock, sink, 0, 0);
    assert(no_buffer.publish(body).error == PublishError::Configuration && sink.calls == 0);
    PublisherChannel no_cadence(Topic::Objects, env, clock, sink, 32768, 65536);
    assert(!no_cadence.valid());
    ProcessEnvelope invalid("BAD", "abcdef01", 5000);
    PublisherChannel bad_rid(Topic::Profile, invalid, clock, sink, 32768, 65536);
    assert(!bad_rid.valid());
    auto r = p.publish(body); assert(r && r.audit.seq == 1); // failed preflights didn't allocate seq
    const auto wire_size = sink.size;
    PublisherChannel exact(Topic::Profile, env, clock, sink, 32768, wire_size + 1);
    assert(exact.publish(body));
    PublisherChannel short_one(Topic::Profile, env, clock, sink, 32768, wire_size);
    assert(short_one.publish(body).error == PublishError::Capacity);
    // Saturated time fields exercise full slot widths (no narrowing or overrun).
    clock.mono = ProcessEnvelope::kMaxMonoMs; clock.wall = std::numeric_limits<Millis>::max(); sink.delay = 0;
    PublisherChannel large(Topic::Status, env, clock, sink, 32768, 65536);
    auto large_result = large.publish(sample_status()); assert(large_result);
    assert(large.publish(sample_status()).error == PublishError::Duplicate);
    assert(std::string_view(sink.bytes.data()).find("9223372036854775.807") != std::string_view::npos);
    assert(std::string_view(sink.bytes.data()).find("9223372036854775.806") != std::string_view::npos);
}
void allocation_and_separate_owners() {
    ProcessEnvelope env(std::string(32, 'a'), "abcdef01", 5000);
    FakeClock clock; Sink sink(clock); ObjectsCadence cadence(0, 512);
    PublisherChannel channel(Topic::Objects, env, clock, sink, 32768, 65536, &cadence);
    auto body = sample_objects(29900);
    const auto allocated = allocations.load();
    for (int i = 0; i <= 250; ++i) {
        clock.mono = 30000 + 40 * i; body.t_capture_mono_ms = clock.mono - 100;
        assert(env.note_clock_status(true, clock.mono));
        assert(channel.publish(body));
    }
    auto c = cadence.snapshot(clock.mono); assert(c.pass);
    assert(allocations.load() == allocated); // includes transport copy, stamping, encoding, cadence
    assert(channel.counters().accepted == 251); // still no receiver counter

    // One stalled slow-line sink cannot hold a shared envelope lock against a
    // different fast-line owner. This is injected transport, NOT GEN/Zenoh proof.
    FakeClock slow_clock, fast_clock;
    Sink slow_sink(slow_clock), fast_sink(fast_clock);
    ObjectsCadence slow_cadence(0, 512);
    PublisherChannel slow(Topic::Objects, env, slow_clock, slow_sink, 32768, 65536, &slow_cadence);
    PublisherChannel fast(Topic::Profile, env, fast_clock, fast_sink, 32768, 65536);
    std::atomic<bool> entered{false}, release{false};
    slow_sink.entered = &entered; slow_sink.release = &release;
    PublishResult slow_result;
    auto pb = sample_profile(); auto ob = sample_objects(29900);
    std::thread thread([&] { slow_result = slow.publish(ob); });
    while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
    const auto fast_result = fast.publish(pb);
    release.store(true, std::memory_order_release); thread.join();
    assert(fast_result && slow_result && fast_result.audit.seq > slow_result.audit.seq);
}
void preparation_boundary() {
    ProcessEnvelope env("robot_1", "abcdef01", 5);
    FakeClock clock; PreparedSink prepared(clock); ObjectsCadence cadence(0, 512);
    PublisherChannel channel(Topic::Objects, env, clock, prepared, 32768, 65536, &cadence);
    assert(env.note_clock_status(true, clock.mono));
    auto body = sample_objects(29900);
    auto result = channel.publish(body);
    assert(result && result.audit.put_before_ms == 30037 && result.audit.seq == 1);
    assert(prepared.sink.entry == 30037 && result.audit.put_return_ms == 30039);
    auto view = std::string_view(prepared.sink.bytes.data());
    assert(view.find("\"mono\":30.037") != view.npos);
    assert(view.find("\"t_publish_mono_ms\":30037") != view.npos);
    assert(view.find("\"ts_sync\":false") != view.npos); // expired DURING prepare
    assert(channel.publish(body).error == PublishError::Duplicate && prepared.prepared == 1);

    prepared.reject = true;
    result = channel.publish(sample_objects(29901));
    assert(result.error == PublishError::PrepareRejected && !result.audit.attempted);
    assert(result.audit.receipt.native_code == 91 && !prepared.active);
    prepared.reject = false; prepared.throw_prepare = true;
    result = channel.publish(sample_objects(29901));
    assert(result.error == PublishError::PrepareUnknown && result.audit.threw && !result.audit.attempted);
    prepared.throw_prepare = false;
    result = channel.publish(sample_objects(29901));
    assert(result && result.audit.seq == 2); // same identity retried only before stamp
    prepared.sink.throws = true;
    result = channel.publish(sample_objects(29902));
    assert(result.error == PublishError::PutUnknown && result.audit.attempted && result.audit.seq == 3);
    assert(channel.publish(sample_objects(29902)).error == PublishError::Duplicate);
    prepared.sink.throws = false;

    ProcessEnvelopeTestAccess::sequence(env, UINT64_MAX);
    result = channel.publish(sample_objects(29903));
    assert(result.error == PublishError::StampUnavailable && !result.audit.attempted && !prepared.active);
    assert(prepared.prepared == prepared.cleaned);
    // Test-only reset permits proving cancellation did not poison a transport.
    ProcessEnvelopeTestAccess::sequence(env, 3);
    assert(channel.publish(sample_objects(29903)).audit.seq == 4);
    assert(channel.counters().attempted == 4 && channel.counters().unknown == 1);

    PublisherChannel status_channel(Topic::Status, env, clock, prepared, 32768, 65536);
    const auto observed = clock.mono;
    const auto calls = prepared.sink.calls;
    result = status_channel.publish_status_snapshot(sample_status(), observed, 10, observed);
    assert(result.error == PublishError::SnapshotInvalid && !result.audit.attempted);
    assert(prepared.sink.calls == calls && prepared.prepared == prepared.cleaned);
    prepared.prepare_delay = 0;
    result = status_channel.publish_status_snapshot(sample_status(), clock.mono, 10, clock.mono);
    assert(result && result.audit.seq == 5); // stale cancellation did not consume seq
    const auto allocs = allocations.load();
    for (int i = 0; i < 5; ++i) {
        ++clock.mono;
        assert(channel.publish(sample_objects(29910 + i)));
    }
    assert(allocations.load() == allocs && !prepared.active && prepared.prepared == prepared.cleaned);
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--emit-fixtures") { fixtures(); return 0; }
    failures_and_timing(); negative_inputs_and_capacity(); allocation_and_separate_owners(); preparation_boundary();
    std::cout << "RNS typed put/envelope, same-frame audit, failure, isolation and no-allocation checks PASS\n";
}
