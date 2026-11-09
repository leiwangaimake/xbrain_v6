/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_envelope.cpp
 * Brief: Clock boundaries, bounded allocation, atomic sequence and upstream parity of the envelope.
 *
 * Description:
 * What it checks. The process envelope stamps ts, mono, boot, seq and ts_sync on every message.
 * Cases cover configuration, clock boundaries and wall steps, invalid clocks and overflow, no
 * allocation, an atomic sequence shared by concurrent topic producers, concurrent clock snapshots,
 * and parity with the platform's own envelope_writer.h.
 * Trap. The platform header computes the 5 s CLK-A3 boundary in double seconds and rounds under at
 * 7.001 s -> 12.001 s; ours uses integer milliseconds and is exact. The test asserts that one
 * divergence explicitly instead of hiding it.
 */

#include "output/rns_envelope.hpp"

#ifdef RNS_UPSTREAM_COMMON
#include <xbrain/envelope/envelope_writer.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
std::atomic<bool> measure_allocations{false};
std::atomic<std::size_t> allocation_count{0};

void record_allocation() noexcept {
    if (measure_allocations.load(std::memory_order_relaxed))
        allocation_count.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace

// Exercise the entire stamp/note call, including accidental std::string copies.
void* operator new(std::size_t size) {
    record_allocation();
    if (auto* p = std::malloc(size == 0 ? 1 : size)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t size, std::align_val_t alignment) {
    record_allocation();
    void* p = nullptr;
    if (posix_memalign(&p, static_cast<std::size_t>(alignment), size == 0 ? 1 : size) == 0)
        return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
    return ::operator new(size, alignment);
}
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace perception::rns {
struct ProcessEnvelopeTestAccess {
    static void seed_sequence(ProcessEnvelope& envelope, std::uint64_t sequence) noexcept {
        envelope.seq_.store(sequence, std::memory_order_relaxed);
    }
};
}  // namespace perception::rns

namespace {
using perception::rns::EnvelopeStamp;
using perception::rns::ProcessEnvelope;
using perception::rns::ProcessEnvelopeTestAccess;

static_assert(!std::is_copy_constructible_v<ProcessEnvelope>);
static_assert(!std::is_move_constructible_v<ProcessEnvelope>);
static_assert(noexcept(std::declval<ProcessEnvelope&>().stamp(
    0, 0, std::declval<EnvelopeStamp&>())));
static_assert(noexcept(std::declval<ProcessEnvelope&>().note_clock_status(true, 0)));

void expect_unchanged(const EnvelopeStamp& actual, const EnvelopeStamp& expected) {
    assert(actual.seq == expected.seq && actual.wall_ms == expected.wall_ms &&
           actual.mono_ms == expected.mono_ms && actual.ts_sync == expected.ts_sync);
}

// Only a valid rid (lower-case letters, digits, '-' or '_', bounded length), an 8-hex-digit boot
// id and a positive timeout make a valid envelope; an invalid one stamps nothing.
void test_configuration() {
    for (const auto& rid : {std::string("r"), std::string("dog_01-abc"),
                            std::string(ProcessEnvelope::kMaxRidLength, 'x')}) {
        ProcessEnvelope envelope(rid, "0123abcd", 5000);
        assert(envelope.valid() && envelope.rid() == rid && envelope.boot() == "0123abcd");
    }
    for (const auto& rid : {std::string(), std::string("bad/id"), std::string("Dog"), std::string("*"),
                            std::string("?"), std::string("a b"), std::string("a\nb"),
                            std::string("\xc3\xa9"), std::string("a\0b", 3),
                            std::string(ProcessEnvelope::kMaxRidLength + 1, 'x')}) {
        ProcessEnvelope invalid(rid, "0123abcd", 5000);
        EnvelopeStamp stamp;
        assert(!invalid.valid() && invalid.rid().empty() && invalid.boot().empty());
        assert(!invalid.note_clock_status(true, 0) && !invalid.stamp(0, 0, stamp));
        expect_unchanged(stamp, EnvelopeStamp{});
    }
    for (const auto& boot : {"", "0123abc", "0123abcde", "0123ABCd", "0123abcg", "0123-abd"}) {
        ProcessEnvelope invalid("dog-01", boot, 5000);
        assert(!invalid.valid());
    }
    for (const std::int64_t timeout : {std::int64_t{0}, std::int64_t{-1}}) {
        ProcessEnvelope invalid("dog-01", "0123abcd", timeout);
        assert(!invalid.valid());
    }
}

// ts_sync is true only while the last true ClockStatus is younger than 5000 ms on the monotonic
// clock (exactly 5000 ms is stale); wall-clock steps never change that; a false status clears it
// at once.
void test_clock_boundaries_and_wall_steps() {
    ProcessEnvelope envelope("dog-01", "0123abcd", 5000);
    EnvelopeStamp stamp;
    assert(envelope.stamp(1700000000000, 0, stamp));
    assert(stamp.seq == 1 && !stamp.ts_sync);
    assert(envelope.note_clock_status(true, 1000));
    assert(envelope.stamp(1700000000001, 5999, stamp));
    assert(stamp.seq == 2 && stamp.ts_sync);  // Age 4999 ms is fresh.
    assert(envelope.stamp(1700000000002, 6000, stamp));
    assert(stamp.seq == 3 && !stamp.ts_sync);  // Age 5000 ms is stale.
    assert(envelope.stamp(1700000000003, 6001, stamp) && !stamp.ts_sync);

    assert(envelope.note_clock_status(true, 7000));
    assert(envelope.stamp(1700000000100, 7000, stamp) && stamp.ts_sync);
    assert(envelope.stamp(1, 7001, stamp) && stamp.ts_sync);  // Positive wall steps backward.
    assert(stamp.wall_ms == 1 && stamp.mono_ms == 7001);
    assert(envelope.stamp(std::numeric_limits<std::int64_t>::max(), 7002, stamp));
    assert(stamp.ts_sync);  // Even a huge wall step cannot expire ClockStatus.
    assert(envelope.stamp(0, 12000, stamp) && !stamp.ts_sync);

    assert(envelope.note_clock_status(false, 12001));
    assert(envelope.stamp(10, 12001, stamp) && !stamp.ts_sync);
    assert(envelope.note_clock_status(true, 12001));  // Same millisecond is permitted.
    assert(envelope.stamp(11, 12001, stamp) && stamp.ts_sync);
    assert(envelope.note_clock_status(false, 12001));
    assert(envelope.stamp(12, 12001, stamp) && !stamp.ts_sync);
}

// Negative or out-of-range clock values are refused without touching the output; an older receipt
// never replaces a newer one, a future receipt is never fresh, and the sequence stops at its
// maximum instead of wrapping.
void test_invalid_clock_and_overflow() {
    ProcessEnvelope envelope("dog-01", "0123abcd", 5000);
    EnvelopeStamp stamp{17, 18, 19, true};
    const auto untouched = stamp;
    assert(!envelope.note_clock_status(true, -1));
    assert(!envelope.note_clock_status(true, std::numeric_limits<std::int64_t>::max()));
    assert(!envelope.stamp(-1, 0, stamp));
    assert(!envelope.stamp(0, -1, stamp));
    assert(!envelope.stamp(0, std::numeric_limits<std::int64_t>::max(), stamp));
    expect_unchanged(stamp, untouched);
    assert(envelope.stamp(0, 0, stamp) && stamp.seq == 1 && !stamp.ts_sync);

    assert(envelope.note_clock_status(true, 100));
    assert(!envelope.note_clock_status(false, 99));  // Older receipt cannot replace current state.
    assert(envelope.stamp(1, 100, stamp) && stamp.ts_sync);
    assert(envelope.note_clock_status(false, 200));
    assert(!envelope.note_clock_status(true, 199));  // Older true cannot undo latest false.
    assert(envelope.stamp(2, 200, stamp) && !stamp.ts_sync);
    assert(envelope.note_clock_status(true, 10000));
    assert(envelope.stamp(3, 9999, stamp) && !stamp.ts_sync);  // Future receipt is never fresh.
    assert(envelope.stamp(4, 10000, stamp) && stamp.ts_sync);

    ProcessEnvelope extremes("dog-01", "0123abcd", std::numeric_limits<std::int64_t>::max());
    assert(extremes.note_clock_status(true, 0));
    assert(extremes.stamp(0, ProcessEnvelope::kMaxMonoMs, stamp) && stamp.ts_sync);
    assert(extremes.note_clock_status(true, ProcessEnvelope::kMaxMonoMs));
    assert(extremes.stamp(0, ProcessEnvelope::kMaxMonoMs - 1, stamp) && !stamp.ts_sync);
    assert(extremes.stamp(0, ProcessEnvelope::kMaxMonoMs, stamp) && stamp.ts_sync);

    ProcessEnvelope exhausted("dog-01", "0123abcd", 5000);
    ProcessEnvelopeTestAccess::seed_sequence(exhausted,
        std::numeric_limits<std::uint64_t>::max() - 1);
    assert(exhausted.stamp(0, 0, stamp));
    assert(stamp.seq == std::numeric_limits<std::uint64_t>::max());
    const auto last = stamp;
    assert(!exhausted.stamp(1, 1, stamp) && !exhausted.stamp(2, 2, stamp));
    expect_unchanged(stamp, last);  // No wrapped sequence, no partially written output.
}

// 20000 stamps and status updates allocate nothing.
void test_no_allocation() {
    ProcessEnvelope envelope(std::string(ProcessEnvelope::kMaxRidLength, 'r'), "0123abcd", 5000);
    EnvelopeStamp stamp;
    allocation_count.store(0, std::memory_order_relaxed);
    measure_allocations.store(true, std::memory_order_relaxed);
    for (std::int64_t i = 0; i < 20000; ++i) {
        assert(envelope.note_clock_status((i % 2) == 0, i));
        assert(envelope.stamp(i, i, stamp));
        assert(stamp.seq == static_cast<std::uint64_t>(i + 1));
        assert(stamp.ts_sync == ((i % 2) == 0));
        assert(envelope.rid().size() == ProcessEnvelope::kMaxRidLength);
    }
    measure_allocations.store(false, std::memory_order_relaxed);
    assert(allocation_count.load(std::memory_order_relaxed) == 0);
}

// Eight producers sharing one envelope get unique, gap-free sequence numbers (a stamp refused
// under contention consumes none).
void test_concurrent_sequence() {
    constexpr std::size_t workers = 8, attempts_per_worker = 5000;
    ProcessEnvelope envelope("dog-01", "0123abcd", 5000);
    // Each worker represents a topic producer sharing the same process object.
    // Preallocation keeps collection separate from the stamp implementation.
    std::vector<std::uint64_t> sequences(workers * attempts_per_worker, 0);
    std::vector<std::thread> threads;
    threads.reserve(workers);
    std::atomic<bool> start{false};
    for (std::size_t worker = 0; worker < workers; ++worker) {
        threads.emplace_back([&, worker] {
            while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
            for (std::size_t i = 0; i < attempts_per_worker; ++i) {
                EnvelopeStamp stamp;
                if (envelope.stamp(1000, 1000, stamp)) {
                    assert(!stamp.ts_sync);
                    sequences[worker * attempts_per_worker + i] = stamp.seq;
                }
                // false under contention is allowed and consumes no sequence.
            }
        });
    }
    start.store(true, std::memory_order_release);
    for (auto& thread : threads) thread.join();
    std::sort(sequences.begin(), sequences.end());
    const auto first = std::upper_bound(sequences.begin(), sequences.end(), std::uint64_t{0});
    const auto successes = static_cast<std::uint64_t>(sequences.end() - first);
    assert(successes >= attempts_per_worker);
    std::uint64_t expected = 1;
    for (auto it = first; it != sequences.end(); ++it) assert(*it == expected++);
    EnvelopeStamp last;
    assert(envelope.stamp(1001, 1001, last) && last.seq == successes + 1);
    // No assertion about topic order or transport/receiver ordering is made.
}

// A stamp never mixes an old true status with a newer receipt while another thread keeps writing
// false.
void test_concurrent_clock_snapshot() {
    ProcessEnvelope envelope("dog-01", "0123abcd", 5000);
    assert(envelope.note_clock_status(true, 0));
    std::atomic<bool> start{false};
    std::atomic<std::size_t> successes{0};
    std::array<std::thread, 4> readers;
    for (auto& reader : readers) {
        reader = std::thread([&] {
            while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
            for (int i = 0; i < 5000; ++i) {
                EnvelopeStamp stamp;
                if (envelope.stamp(5000, 5000, stamp)) {
                    // The old true status is stale; every newer status is false.
                    // Mixing the new receipt with old true would violate this.
                    assert(!stamp.ts_sync);
                    successes.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }
    std::thread writer([&] {
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (std::int64_t receipt = 1; receipt < 5000; ++receipt)
            assert(envelope.note_clock_status(false, receipt));
    });
    start.store(true, std::memory_order_release);
    for (auto& reader : readers) reader.join();
    writer.join();
    assert(successes.load(std::memory_order_relaxed) > 0);
    assert(!envelope.note_clock_status(true, 4998));
    EnvelopeStamp last;
    assert(envelope.stamp(5000, 5000, last) && !last.ts_sync);
}

#ifdef RNS_UPSTREAM_COMMON
// Our stamps equal the platform's envelope_writer.h field by field, except the one asserted case
// where its double-second arithmetic keeps an exactly 5 s old status fresh.
void test_actual_upstream_parity() {
    // This is the supplied implementation, not a second handwritten oracle.
    // Since common commit f005aa1 (11 S3.0) the supplied writer takes and stamps
    // SECONDS as double; ours keeps integer milliseconds internally and writes
    // seconds on the wire. Feed it seconds and compare in seconds: passing the
    // millisecond integers straight through still compiles (implicit int64 ->
    // double) and, with a 5000 "second" window, agreed with us by accident.
    using hachist::xbrain::envelope::EnvelopeWriter;
    const auto s = [](std::int64_t ms) { return static_cast<double>(ms) / 1000.0; };
    ProcessEnvelope bounded("dog-01", "0123abcd", 5000);
    EnvelopeWriter upstream("dog-01", "perception", "0123abcd", s(5000));
    const auto compare = [&](std::int64_t wall, std::int64_t mono) {
        EnvelopeStamp actual;
        assert(bounded.stamp(wall, mono, actual));
        const auto expected = upstream.stamp(s(wall), s(mono));
        assert(ProcessEnvelope::kVersion == expected.v && bounded.rid() == expected.rid);
        assert(bounded.boot() == expected.boot && ProcessEnvelope::kSource == expected.src);
        // Unit guard: the supplied writer must have been fed seconds.
        assert(expected.ts == s(wall) && expected.mono == s(mono));
        assert(s(actual.wall_ms) == expected.ts && s(actual.mono_ms) == expected.mono &&
               actual.seq == expected.seq && actual.ts_sync == expected.ts_sync);
    };
    const auto note = [&](bool sync, std::int64_t mono) {
        assert(bounded.note_clock_status(sync, mono));
        upstream.note_clock_status(sync, s(mono));
    };
    compare(1700000000000, 0);
    note(true, 1000);
    compare(1700000000001, 1000);
    compare(1, 5999);  // Positive wall-clock backstep retains upstream semantics.
    compare(1700000000002, 6000);
    compare(1700000000003, 6001);
    note(false, 6002);
    compare(1700000000004, 6002);
    note(true, 7000);
    compare(1700000000005, 7000);
    compare(1700000000006, 11999);
    compare(1700000000007, 12000);
    // Boundaries that are not whole seconds (ours: integer ms; theirs: double s).
    // CLK-A3 says an age of >= 5 s is stale. The supplied writer subtracts two
    // doubles, so at exactly 5000 ms it can see 4.999999999999999 s and keep
    // ts_sync true (received 7.001 s, now 12.001 s). Ours stays exact. That one
    // divergence is asserted explicitly here instead of being skipped: ours must
    // follow the contract; every other field (and every other case) stays in parity.
    const auto boundary = [&](std::int64_t wall, std::int64_t received, std::int64_t mono) {
        const bool upstream_rounds_under = s(mono) - s(received) < 5.0;
        if (!upstream_rounds_under) { compare(wall, mono); return; }
        EnvelopeStamp actual;
        assert(bounded.stamp(wall, mono, actual));
        const auto expected = upstream.stamp(s(wall), s(mono));
        assert(expected.ts == s(wall) && expected.mono == s(mono));
        assert(s(actual.wall_ms) == expected.ts && s(actual.mono_ms) == expected.mono &&
               actual.seq == expected.seq);
        assert(!actual.ts_sync && expected.ts_sync);  // contract-exact vs 1-ulp rounding
    };
    for (const std::int64_t received : {7001LL, 123457LL, 86400123LL, 987654321LL}) {
        note(true, received);
        compare(1700000000100, received + 4999);
        boundary(1700000000101, received, received + 5000);
        compare(1700000000102, received + 5001);
    }

    note(true, 999999990);
    EnvelopeStamp guarded;
    assert(bounded.stamp(1700000000008, 999999989, guarded) && !guarded.ts_sync);
    assert(upstream.stamp(s(1700000000008), s(999999989)).ts_sync);
    // The stricter negative-age behavior above is deliberately outside parity.
}
#endif

void emit_fixtures() {
    ProcessEnvelope envelope("dog-01", "0123abcd", 5000);
    bool first = true;
    std::cout << "[\n";
    const auto emit = [&](const char* name, std::int64_t wall, std::int64_t mono) {
        EnvelopeStamp stamp;
        assert(envelope.stamp(wall, mono, stamp));
        if (!first) std::cout << ",\n";
        first = false;
        std::cout << "  {\"case\":\"" << name << "\",\"v\":" << ProcessEnvelope::kVersion
                  << ",\"rid\":\"" << envelope.rid() << "\",\"wall_ms\":" << stamp.wall_ms
                  << ",\"mono_ms\":" << stamp.mono_ms << ",\"boot\":\"" << envelope.boot()
                  << "\",\"seq\":" << stamp.seq << ",\"src\":\"" << ProcessEnvelope::kSource
                  << "\",\"ts_sync\":" << (stamp.ts_sync ? "true" : "false") << '}';
    };
    emit("initial_false", 1700000000000, 0);
    assert(envelope.note_clock_status(true, 1000));
    emit("fresh_4999", 1700000000001, 5999);
    emit("stale_5000", 1700000000002, 6000);
    assert(envelope.note_clock_status(true, 7000));
    emit("wall_backstep", 1, 7000);
    assert(envelope.note_clock_status(false, 7001));
    emit("copied_false", 1700000000003, 7001);
    assert(envelope.note_clock_status(true, 8000));
    emit("future_receipt", 1700000000004, 7999);
    emit("fresh_zero_age", 1700000000005, 8000);
    std::cout << "\n]\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--emit-fixtures") {
        emit_fixtures();
        return 0;
    }
    if (argc != 1) {
        std::cerr << "usage: test_rns_envelope [--emit-fixtures]\n";
        return 2;
    }
    test_configuration();
    test_clock_boundaries_and_wall_steps();
    test_invalid_clock_and_overflow();
    test_no_allocation();
    test_concurrent_sequence();
    test_concurrent_clock_snapshot();
#ifdef RNS_UPSTREAM_COMMON
    test_actual_upstream_parity();
    std::cout << "RNS envelope actual-upstream equivalence checks PASS\n";
#else
    std::cout << "RNS envelope actual-upstream equivalence checks NOT RUN (RNS_UPSTREAM_COMMON unset)\n";
#endif
    std::cout << "RNS envelope identity, clock, sequence, allocation and concurrency checks PASS\n";
}
