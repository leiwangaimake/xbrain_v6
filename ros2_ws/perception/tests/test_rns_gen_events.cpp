/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_gen_events.cpp
 * Brief: Unit tests for the GEN event encoder and its queue.
 *
 * Description:
 * What it checks. GEN events (fault, warn and info notices) are encoded against fixtures; the
 * tests cover failures, constructors, no allocation on the hot path, concurrency, and that a
 * blocked GEN side never blocks the RT publishing path.
 */

#include "output/rns_gen_events.hpp"
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <new>
#include <stdexcept>
#include <thread>

static std::atomic<std::size_t> ordinary{0}, aligned{0};
void* operator new(std::size_t n) { ++ordinary; if (auto* p = std::malloc(n ? n : 1)) return p; throw std::bad_alloc(); }
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) {
    ++aligned; void* p = nullptr;
    if (!posix_memalign(&p, static_cast<std::size_t>(a), n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n, std::align_val_t a) { return ::operator new(n, a); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

using namespace perception::rns;
namespace perception::rns {
struct GenEventsTestAccess { static void exhaust(GenEvents& g) { g.offered_.accepted = UINT64_MAX; } };
struct ProcessEnvelopeTestAccess { static void exhaust(ProcessEnvelope& e) { e.seq_.store(UINT64_MAX); } };
}
namespace {
constexpr auto nonce = "0123456789abcdef0123456789abcdef";
struct Clock : PublishClock {
    Millis now = 1000000;
    Millis wall_millis() noexcept override { return 1700000000123; }
    Millis mono_millis() noexcept override { return now; }
};
struct Sink : RnsTransport {
    char bytes[32768]{}; std::size_t size = 0, calls = 0;
    std::string_view last_key;
    PutOutcome outcome = PutOutcome::Accepted;
    bool throws = false;
    PublishQos last_qos = GenEvents::qos();
    PutReceipt put(std::string_view key, std::string_view packet, PublishQos qos) override {
        ++calls; assert(packet.size() <= sizeof(bytes)); std::memcpy(bytes, packet.data(), packet.size());
        size = packet.size(); last_key = key; last_qos = qos;
        if (throws) throw std::runtime_error("injected ambiguous API error");
        return {outcome, -7};
    }
    std::string_view packet() const { return {bytes, size}; }
};
ReasonEdge edge(bool warn, Millis observed) {
    ReasonEdge e; e.observed_ms = observed;
    e.severity = warn ? ReasonEdgeSeverity::Warn : ReasonEdgeSeverity::Info;
    if (warn) e.raw_added = e.eligible_added = e.active_after = 3;
    else e.raw_removed = e.eligible_removed = 3;
    return e;
}
template<class F> auto no_new(F fn) {
    const auto a = ordinary.load(), b = aligned.load(); auto result = fn();
    assert(ordinary == a && aligned == b); return result;
}
void fixtures(bool emit) {
    ProcessEnvelope envelope("gen_probe", "abcdef01", 5000); Clock clock; Sink sink;
    GenEvents events(envelope, clock, sink, sink, nonce, {"infer_gap", "infer_rate_low"}, 4, 8192);
    ReasonEdges journal(0, 2, 4); ReasonEdge e;
    assert(envelope.note_clock_status(true, 900));
    assert(journal.update(3, 1000).enqueued_edges == 1 && journal.pop(e));
    auto accepted = no_new([&] { return events.offer(e, 1700000000123, 1001); });
    assert(accepted.code == GenOfferCode::Accepted && accepted.seq == 1);
    // Competing RT stamps share one sequence, without sharing network locks.
    EnvelopeStamp rt; assert(envelope.stamp(1700000000222, 1002, rt) && rt.seq == 2);
    assert(journal.update(0, 11000).enqueued_edges == 1 && journal.pop(e));
    assert(events.offer(e, 1700000010123, 11001).seq == 3);
    for (const bool warn : {true, false}) {
        clock.now = warn ? 12000 : 13000;
        const auto result = no_new([&] { return events.send_one(); });
        assert(result.code == GenSendCode::Accepted && result.attempted && result.clock_valid);
        assert(result.stamp.mono_ms == (warn ? 1001 : 11001));
        assert(result.stamp.seq == (warn ? 1 : 3) && result.edge.observed_ms == (warn ? 1000 : 11000));
        assert(result.stamp.ts_sync == warn); // frozen at creation, not at send
        assert(sink.last_key == events.key(warn ? ReasonEdgeSeverity::Warn : ReasonEdgeSeverity::Info));
        assert(sink.last_qos.priority == Priority::Data && sink.last_qos.reliable && !sink.last_qos.drop && !sink.last_qos.express);
        if (emit) std::cout << "{\"packet\":" << sink.packet() << ",\"key\":\"" << sink.last_key
            << "\",\"before\":" << result.put_before_ms << ",\"after\":" << result.put_return_ms
            << ",\"api\":\"accepted\",\"received\":null}\n";
    }
    assert(events.send_one().code == GenSendCode::Empty && events.healthy() && sink.calls == 2);
}
void failures() {
    ProcessEnvelope env("test", "abcdef01", 100); Clock clock; Sink sink;
    GenEvents g(env, clock, sink, sink, nonce, {"infer_gap", "infer_rate_low"}, 1, 8192);
    assert(g.offer(edge(true, 0), 0, 0).code == GenOfferCode::Accepted);
    assert(no_new([&] { return g.offer(edge(false, 1), 1, 1); }).code == GenOfferCode::Full);
    assert(!g.healthy() && g.queue_depth() == 1 && g.offer_counters().full == 1);
    assert(g.send_one().edge.raw_added == 3); // no overwrite by lost recovery
    assert(g.send_one().code == GenSendCode::Empty);
    for (auto outcome : {PutOutcome::Rejected, PutOutcome::Unknown}) {
        sink.outcome = outcome;
        assert(g.offer(edge(true, 2), 2, 2).code == GenOfferCode::Accepted);
        auto sent = g.send_one(); assert(sent.attempted && sent.receipt.outcome == outcome);
        assert(g.send_one().code == GenSendCode::Empty); // no unknown replay
    }
    sink.throws = true; assert(g.offer(edge(false, 3), 3, 3).code == GenOfferCode::Accepted);
    auto thrown = g.send_one(); assert(thrown.threw && thrown.code == GenSendCode::Unknown);
    sink.throws = false;
    assert(g.send_counters().accepted == 1 && g.send_counters().rejected == 1 && g.send_counters().unknown == 2);
    auto bad = edge(true, 4); bad.raw_added = 8;
    assert(g.offer(bad, 4, 4).code == GenOfferCode::Invalid);
    assert(g.offer(edge(true, 4), -1, 4).code == GenOfferCode::Invalid);
    assert(g.offer(edge(true, 4), 4, 3).code == GenOfferCode::Invalid);
    assert(g.offer(edge(true, 2), 4, 4).code == GenOfferCode::Invalid);
    assert(g.offer_counters().invalid == 4 && g.queue_depth() == 0);
    assert(g.offer(edge(true, 5), 5, 5).code == GenOfferCode::Accepted);
    clock.now = 4; auto unsent = g.send_one(); assert(!unsent.attempted && unsent.code == GenSendCode::ClockInvalid);
    assert(g.send_counters().unsent == 1 && g.queue_depth() == 0);
    GenEventsTestAccess::exhaust(g);
    assert(g.offer(edge(true, 6), 6, 6).code == GenOfferCode::Exhausted);
    GenEvents other(env, clock, sink, sink, nonce, {"infer_gap", "infer_rate_low"}, 1, 8192);
    ProcessEnvelopeTestAccess::exhaust(env);
    assert(other.offer(edge(true, 7), 7, 7).code == GenOfferCode::StampUnavailable && !other.healthy());
}
void constructors() {
    ProcessEnvelope env("test", "abcdef01", 100); Clock clock; Sink sink;
    for (int variant = 0; variant < 7; ++variant) {
        bool threw = false;
        try {
            GenEvents g(env, clock, sink, sink,
                variant == 0 ? "bad" : variant == 1 ? std::string(32, '0') : nonce,
                variant == 2 ? std::vector<std::string>{"duplicate", "duplicate"} :
                variant == 3 ? std::vector<std::string>{"quote\""} : std::vector<std::string>{"valid"},
                variant == 4 ? 0 : variant == 5 ? 65537 : 1, variant == 6 ? 8 : 8192);
        } catch (const std::invalid_argument&) { threw = true; }
        assert(threw);
    }
}
void no_allocation() {
    ProcessEnvelope env("test", "abcdef01", 100); Clock clock; Sink sink;
    GenEvents g(env, clock, sink, sink, nonce, {"infer_gap", "infer_rate_low"}, 1, 8192);
    for (int i = 0; i < 2000; ++i) {
        assert(no_new([&] { return g.offer(edge(i % 2 == 0, i), i, i); }).code == GenOfferCode::Accepted);
        assert(no_new([&] { return g.send_one(); }).code == GenSendCode::Accepted);
    }
    assert(g.healthy());
}
void concurrency() {
    ProcessEnvelope env("test", "abcdef01", 100); Clock clock; Sink sink;
    GenEvents g(env, clock, sink, sink, nonce, {"infer_gap", "infer_rate_low"}, 32768, 8192);
    constexpr std::uint64_t total = 20000;
    std::thread producer([&] {
        for (std::uint64_t i = 1; i <= total; ++i)
            assert(g.offer(edge(i % 2 == 1, i), i, i).code == GenOfferCode::Accepted);
    });
    std::thread consumer([&] {
        for (std::uint64_t n = 1; n <= total;) {
            const auto result = g.send_one();
            if (result.code == GenSendCode::Empty) { std::this_thread::yield(); continue; }
            assert(result.code == GenSendCode::Accepted && result.stamp.seq == n && result.edge.observed_ms == static_cast<Millis>(n));
            ++n;
        }
    });
    producer.join(); consumer.join();
    assert(g.healthy() && g.queue_depth() == 0 && g.offer_counters().accepted == total && g.send_counters().accepted == total);
}
void blocked_gen_does_not_block_rt() {
    struct Blocking : RnsTransport {
        std::mutex mutex; std::condition_variable cv; bool entered = false, release = false;
        PutReceipt put(std::string_view, std::string_view, PublishQos) override {
            std::unique_lock<std::mutex> lock(mutex); entered = true; cv.notify_all();
            cv.wait(lock, [&] { return release; }); return {PutOutcome::Accepted, 0};
        }
    } blocked;
    ProcessEnvelope env("test", "abcdef01", 100); Clock gc, rc; Sink rt_sink;
    GenEvents gen(env, gc, blocked, blocked, nonce, {"infer_gap", "infer_rate_low"}, 2, 8192);
    ObjectsCadence cadence(0, 1024); PublisherChannel rt(Topic::Objects, env, rc, rt_sink, 4096, 8192, &cadence);
    assert(gen.offer(edge(true, 1), 1, 1).code == GenOfferCode::Accepted);
    GenSendResult result; std::thread sender([&] { result = gen.send_one(); });
    {
        std::unique_lock<std::mutex> lock(blocked.mutex);
        assert(blocked.cv.wait_for(lock, std::chrono::seconds(5), [&] { return blocked.entered; }));
    }
    // Actual blocked send owns no queue slot and no RT lock. Fill remaining
    // bounded storage and explicitly observe overload; fast puts still proceed.
    assert(gen.offer(edge(false, 2), 2, 2).code == GenOfferCode::Accepted);
    assert(gen.offer(edge(true, 3), 3, 3).code == GenOfferCode::Accepted);
    assert(gen.offer(edge(false, 4), 4, 4).code == GenOfferCode::Full);
    for (int i = 0; i < 100; ++i) {
        rc.now = 40000 + i * 40; Objects object; object.t_capture_mono_ms = rc.now - 10;
        assert(rt.publish(object));
    }
    assert(rt_sink.calls == 100);
    { std::lock_guard<std::mutex> lock(blocked.mutex); blocked.release = true; }
    blocked.cv.notify_all(); sender.join();
    assert(result.code == GenSendCode::Accepted && gen.send_one().code == GenSendCode::Accepted);
    assert(gen.send_one().code == GenSendCode::Accepted && gen.send_one().code == GenSendCode::Empty);
    assert(!gen.healthy()); // draining backlog never hides prior lost event
}
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--emit-fixtures") { fixtures(true); return 0; }
    fixtures(false); failures(); constructors(); no_allocation(); concurrency(); blocked_gen_does_not_block_rt();
    std::cout << "PASS GEN 6 groups: creation stamps, failure/no-retry, invalid configuration, 2000 zero-new offer/send pairs, 20000 SPSC FIFO events, blocked GEN + 100 RT puts; NO native delivery or real-time claim\n";
}
