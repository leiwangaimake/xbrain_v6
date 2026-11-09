/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_jit_pace.cpp
 * Brief: Unit test for the minimum submit interval of the JIT loop (infer_min_interval_ms).
 *
 * Description:
 * What it checks. pace_wait_ns: nothing to wait with the cap off, before the first submit, exactly
 * on time and late; the rest of the interval when early; a clock that runs backwards throws. The
 * switch text: "0" or absent is off, 1..49 in one spelling, everything else (50 and more too)
 * throws. jit_pace_wait
 * on a fake clock: no sleep without a wait, slices of at most 5 ms that end on the due time, and a
 * stop that ends the wait after the slice in progress. No CUDA, TensorRT or camera.
 */

#include "common/jit_schedule.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {
constexpr std::int64_t kMs = 1000000;

bool parse_throws(const char* text) {
    try {
        (void)perception::parse_infer_min_interval_ms(text);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

// A clock that only moves when the waiter sleeps.
struct FakeClock {
    std::int64_t now_ns = 0;
    std::vector<std::int64_t> sleeps;
    int keep_waiting_calls = 0;
    int stop_after_calls = -1;  // -1: never stopped
};
}  // namespace

int main() {
    using perception::pace_wait_ns;
    const std::int64_t last = 1000 * kMs;
    const std::int64_t interval = 45 * kMs;

    // pace_wait_ns
    assert(pace_wait_ns(0, 5 * kMs, interval) == 0);                 // first submit
    assert(pace_wait_ns(last, last + 41 * kMs, interval) == 4 * kMs);  // early: the rest
    assert(pace_wait_ns(last, last, interval) == interval);           // right after a submit
    assert(pace_wait_ns(last, last + interval, interval) == 0);       // exactly on time
    assert(pace_wait_ns(last, last + interval - 1, interval) == 1);
    assert(pace_wait_ns(last, last + 60 * kMs, interval) == 0);       // late
    assert(pace_wait_ns(last, last + 1 * kMs, 0) == 0);               // cap off
    assert(pace_wait_ns(last, last + 1 * kMs, -1) == 0);
    bool threw = false;
    try { (void)pace_wait_ns(last, last - 1, interval); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    // With the cap off nothing is checked, not even the clock order.
    assert(pace_wait_ns(last, last - 1, 0) == 0);

    // The switch text (the same rule as the service's switch table).
    assert(perception::parse_infer_min_interval_ms(nullptr) == 0);
    assert(perception::parse_infer_min_interval_ms("0") == 0);
    assert(perception::parse_infer_min_interval_ms("1") == 1);
    assert(perception::parse_infer_min_interval_ms("45") == 45);
    assert(perception::parse_infer_min_interval_ms("49") == 49);
    assert(perception::valid_infer_min_interval_text("49") && perception::valid_infer_min_interval_text("0"));
    for (const char* bad : {"", "50", "51", "99", "100", "101", "-1", "4.5", "abc", "045", "00", "1000", "45ms",
                            " 45", "45 ", "+45", "0x2d", "4e1"}) {
        assert(parse_throws(bad));
        assert(!perception::valid_infer_min_interval_text(bad));
    }

    const auto run = [](FakeClock& clock, std::int64_t last_submit_ns, std::int64_t interval_ns) {
        return perception::jit_pace_wait(last_submit_ns, interval_ns,
            [&]() { return clock.now_ns; },
            [&](std::int64_t ns) { clock.sleeps.push_back(ns); clock.now_ns += ns; },
            [&]() {
                ++clock.keep_waiting_calls;
                return clock.stop_after_calls < 0 || clock.keep_waiting_calls <= clock.stop_after_calls;
            });
    };

    // Early by 4 ms: one sleep, submit exactly on the due time.
    {
        FakeClock clock;
        clock.now_ns = last + 41 * kMs;
        assert(run(clock, last, interval) == 4 * kMs);
        assert(clock.sleeps.size() == 1 && clock.sleeps[0] == 4 * kMs);
        assert(clock.now_ns == last + interval);
    }
    // Right after a submit with a 12 ms cap: slices of 5, 5 and 2 ms.
    {
        FakeClock clock;
        clock.now_ns = last;
        assert(run(clock, last, 12 * kMs) == 12 * kMs);
        assert((clock.sleeps == std::vector<std::int64_t>{5 * kMs, 5 * kMs, 2 * kMs}));
    }
    // A sleep that overshoots ends the wait at the next clock read; the wait reports what passed.
    {
        FakeClock clock;
        clock.now_ns = last + 41 * kMs;
        const auto waited = perception::jit_pace_wait(last, interval,
            [&]() { return clock.now_ns; },
            [&](std::int64_t ns) { clock.sleeps.push_back(ns); clock.now_ns += ns + 300000; },
            []() { return true; });
        assert(waited == 4 * kMs + 300000 && clock.sleeps.size() == 1);
    }
    // No wait: first submit, on time, late, cap off. No sleep and no stop check.
    const std::vector<std::array<std::int64_t, 3>> no_wait = {
        {0, 5 * kMs, interval}, {last, last + interval, interval},
        {last, last + 60 * kMs, interval}, {last, last + 1 * kMs, 0}};
    for (const auto& [last_submit_ns, now_ns, interval_ns] : no_wait) {
        FakeClock clock;
        clock.now_ns = now_ns;
        assert(run(clock, last_submit_ns, interval_ns) == 0);
        assert(clock.sleeps.empty() && clock.keep_waiting_calls == 0);
    }
    // A stop during a 45 ms wait: the slice in progress ends, then the wait returns early.
    {
        FakeClock clock;
        clock.now_ns = last;
        clock.stop_after_calls = 2;  // running for two checks, stopped at the third
        assert(run(clock, last, interval) == 10 * kMs);
        assert(clock.sleeps.size() == 2 && clock.keep_waiting_calls == 3);
    }
    // Already stopped: no sleep at all.
    {
        FakeClock clock;
        clock.now_ns = last;
        clock.stop_after_calls = 0;
        assert(run(clock, last, interval) == 0);
        assert(clock.sleeps.empty() && clock.keep_waiting_calls == 1);
    }
    // The longest wait the switch allows is still sliced: never more than 5 ms between stop checks.
    {
        FakeClock clock;
        clock.now_ns = last;
        static_assert(perception::JIT_MIN_INTERVAL_MAX_MS == 49, "the 50 ms line of 19 section 6.3");
        assert(run(clock, last, perception::JIT_MIN_INTERVAL_MAX_MS * kMs) == 49 * kMs);
        assert(clock.sleeps.size() == 10 && clock.sleeps.back() == 4 * kMs);
        for (const auto ns : clock.sleeps) assert(ns <= perception::JIT_PACE_SLICE_NS);
    }
    return 0;
}
