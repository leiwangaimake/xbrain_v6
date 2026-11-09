/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_system_clock.cpp
 * Brief: Conversion/error boundaries and direct POSIX-clock bracketing of the publish clock, no sleeps.
 *
 * Description:
 * What it checks. SystemPublishClock turns CLOCK_REALTIME and CLOCK_MONOTONIC into the envelope's
 * millisecond fields. The tests cover the conversions at their numeric limits, read errors and
 * which clock each field uses, that a failed read neither latches a fault nor turns a valid epoch
 * value of 0 into an error, bracketing of the live clocks between two direct POSIX reads, and that
 * reading allocates nothing.
 */

#include "output/rns_system_clock.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <type_traits>
#include <utility>

namespace {
std::atomic<bool> measure_allocations{false};
std::atomic<std::size_t> allocation_count{0};
void record_allocation() noexcept {
    if (measure_allocations.load(std::memory_order_relaxed))
        allocation_count.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace

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

namespace {
using perception::rns::Millis;
using perception::rns::SystemPublishClock;
using perception::rns::timespec_to_millis;

static_assert(std::is_final_v<SystemPublishClock>);
static_assert(noexcept(std::declval<SystemPublishClock&>().wall_millis()));
static_assert(noexcept(std::declval<SystemPublishClock&>().mono_millis()));
static_assert(noexcept(timespec_to_millis(std::declval<const timespec&>())));

// timespec converts to milliseconds by truncation; negative or out-of-range fields and values
// beyond the millisecond limit give -1.
void test_conversion() {
    assert(timespec_to_millis(timespec{0, 0}) == 0);
    assert(timespec_to_millis(timespec{0, 999999}) == 0);
    assert(timespec_to_millis(timespec{0, 1000000}) == 1);
    assert(timespec_to_millis(timespec{0, 999999999}) == 999);
    assert(timespec_to_millis(timespec{1, 999999999}) == 1999);
    assert(timespec_to_millis(timespec{-1, 0}) == -1);
    assert(timespec_to_millis(timespec{0, -1}) == -1);
    assert(timespec_to_millis(timespec{0, 1000000000}) == -1);
    assert(timespec_to_millis(timespec{0, std::numeric_limits<long>::max()}) == -1);

    constexpr auto maximum = std::numeric_limits<Millis>::max();
    constexpr auto last_second = maximum / 1000;
    constexpr auto last_fraction = maximum % 1000;
    // macOS and 64-bit Linux use a time_t wide enough for the conversion limit.
    if constexpr (std::numeric_limits<time_t>::max() >= last_second) {
        const auto seconds = static_cast<time_t>(last_second);
        const auto fraction = static_cast<long>(last_fraction * 1000000);
        assert(timespec_to_millis(timespec{seconds, fraction}) == maximum);
        assert(timespec_to_millis(timespec{seconds, fraction + 999999}) == maximum);
        assert(timespec_to_millis(timespec{seconds, fraction + 1000000}) == -1);
        assert(timespec_to_millis(timespec{static_cast<time_t>(last_second + 1), 0}) == -1);
        assert(timespec_to_millis(timespec{std::numeric_limits<time_t>::max(), 0}) == -1);
    }
}

timespec injected_sample{};
int injected_status = 0;
int injected_calls = 0;
clockid_t observed_clock = CLOCK_REALTIME;
int injected_read(clockid_t id, timespec* sample) noexcept {
    ++injected_calls;
    observed_clock = id;
    *sample = injected_sample;
    return injected_status;
}

// Wall time reads CLOCK_REALTIME and mono reads CLOCK_MONOTONIC; a failed or invalid read gives -1
// without latching, and wall time may step backwards.
void test_read_errors_and_clock_choice() {
    SystemPublishClock clock(&injected_read);
    injected_sample = timespec{12, 345678901};
    injected_status = 0;
    assert(clock.wall_millis() == 12345 && observed_clock == CLOCK_REALTIME);
    assert(clock.mono_millis() == 12345 && observed_clock == CLOCK_MONOTONIC);
    for (const int status : {-1, 1}) {
        injected_status = status;
        assert(clock.wall_millis() == -1 && clock.mono_millis() == -1);
    }
    injected_status = 0;
    for (const timespec sample : {timespec{-1, 0}, timespec{0, -1}, timespec{0, 1000000000}}) {
        injected_sample = sample;
        assert(clock.wall_millis() == -1 && clock.mono_millis() == -1);
    }
    assert(injected_calls == 12);  // Every call used the selected source; no fallback read.
    constexpr auto last_second = std::numeric_limits<Millis>::max() / 1000;
    if constexpr (std::numeric_limits<time_t>::max() > last_second) {
        injected_sample = timespec{static_cast<time_t>(last_second + 1), 0};
        assert(clock.wall_millis() == -1 && clock.mono_millis() == -1);
    }
    // A failed read must not latch a fault or turn the valid epoch value 0 into
    // an error. Every later call independently converts the selected sample.
    injected_sample = timespec{0, 0};
    const auto calls_before_recovery = injected_calls;
    assert(clock.wall_millis() == 0 && clock.mono_millis() == 0);
    assert(injected_calls == calls_before_recovery + 2);
    injected_sample = timespec{7, 0};
    assert(clock.wall_millis() == 7000);
    injected_sample = timespec{2, 0};
    assert(clock.wall_millis() == 2000);  // Wall time is allowed to step backwards.
    SystemPublishClock absent(nullptr);
    assert(absent.wall_millis() == -1 && absent.mono_millis() == -1);
}

// Real readings fall between two direct clock_gettime calls, and monotonic readings never
// decrease.
void test_live_clock_brackets() {
    SystemPublishClock clock;
    Millis previous_mono = -1;
    for (int i = 0; i < 1000; ++i) {
        timespec before{}, after{};
        assert(::clock_gettime(CLOCK_MONOTONIC, &before) == 0);
        const auto current = clock.mono_millis();
        assert(::clock_gettime(CLOCK_MONOTONIC, &after) == 0);
        assert(current >= 0 && current >= previous_mono);
        assert(timespec_to_millis(before) <= current && current <= timespec_to_millis(after));
        previous_mono = current;

        assert(::clock_gettime(CLOCK_REALTIME, &before) == 0);
        const auto wall = clock.wall_millis();
        assert(::clock_gettime(CLOCK_REALTIME, &after) == 0);
        const auto before_wall = timespec_to_millis(before), after_wall = timespec_to_millis(after);
        // A wall-clock step is permitted; it is never used as the monotonic ledger.
        assert(wall >= std::min(before_wall, after_wall) && wall <= std::max(before_wall, after_wall));
    }
}

// 10000 reads and conversions allocate nothing.
void test_no_allocation() {
    SystemPublishClock clock;
    SystemPublishClock injected(&injected_read);
    allocation_count.store(0, std::memory_order_relaxed);
    measure_allocations.store(true, std::memory_order_relaxed);
    for (int i = 0; i < 10000; ++i) {
        assert(clock.wall_millis() >= 0);
        assert(clock.mono_millis() >= 0);
        assert(timespec_to_millis(timespec{i, 123456789}) == static_cast<Millis>(i) * 1000 + 123);
        injected_sample = timespec{i, 123456789};
        injected_status = (i % 2) == 0 ? 0 : -1;
        const Millis expected = injected_status == 0 ? static_cast<Millis>(i) * 1000 + 123 : -1;
        assert(injected.wall_millis() == expected && injected.mono_millis() == expected);
    }
    measure_allocations.store(false, std::memory_order_relaxed);
    assert(allocation_count.load(std::memory_order_relaxed) == 0);
}

}  // namespace

int main() {
    test_conversion();
    test_read_errors_and_clock_choice();
    test_live_clock_brackets();
    test_no_allocation();
    std::cout << "RNS system clock conversion, error, monotonic bracket and allocation checks PASS\n";
}
