/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_jit_fresh_bind.cpp
 * Brief: Unit tests for the JIT fresh-bind wait on the camera frame queue (try_pop_after_update_until).
 *
 * Description:
 * What it checks. With fresh-bind on, inference may wait a bounded time for a newer camera frame
 * instead of binding a stale one. The tests pin that contract: strict default-off parsing and
 * policy boundaries; an empty queue, a fresh candidate or an expired deadline never waits; a push
 * (copy or move) that replaces the queued frame wakes the waiter; a timeout returns the original
 * candidate; a late decision never renews the deadline; clear() cancels the wait even when a new
 * frame follows; a throwing callback neither consumes nor locks the queue; move-only payloads keep
 * the legacy queue operations.
 * Why. A wait that renews itself or misses a wakeup becomes an inference stall (r6 cadence), so
 * the deadline and wakeup cases are asserted directly; the watchdog is a generous deadlock guard,
 * not a latency gate.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/jit_schedule.hpp"
#include "common/threadsafe_queue.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

struct Frame {
    std::uint64_t id = 0;
    int payload = -1;
};

using Queue = yolo::ThreadsafeQueue<Frame>;

struct Result {
    bool popped = false;
    Frame frame{999, -999};
    yolo::QueueUpdateWaitInfo info{true, true, true};
};

template <typename T>
T finish(std::future<T>& future) {
    // This is a generous deadlock watchdog, not a millisecond latency gate.
    assert(future.wait_for(5s) == std::future_status::ready);
    return future.get();
}

void reached(std::future<void>& future) {
    assert(future.wait_for(5s) == std::future_status::ready);
    future.get();
}

template <typename Function>
void invalid_argument(Function&& function) {
    bool caught = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    assert(caught);
}

// Only "0" and "1" parse (unset is off); the deadline is target + 2 ms with overflow refused; a
// wait needs time before the deadline and a queued frame at least 25 ms old.
void test_strict_default_off_and_policy_boundaries() {
    assert(!perception::parse_jit_fresh_bind_enabled(nullptr));
    assert(!perception::parse_jit_fresh_bind_enabled("0"));
    assert(perception::parse_jit_fresh_bind_enabled("1"));
    for (const char* text : {"", "true", "false", " 1", "1 ", "01", "+1", "2", "-1", "1\n"}) {
        invalid_argument([&]() { (void)perception::parse_jit_fresh_bind_enabled(text); });
    }
    assert(perception::JIT_FRESH_BIND_BUDGET_NS == 2'000'000);
    assert(perception::JIT_FRESH_BIND_MIN_QUEUE_AGE_NS == 25'000'000);
    constexpr std::int64_t target = 100'000'000;
    const auto deadline = perception::jit_fresh_bind_deadline_ns(target);
    assert(deadline == 102'000'000);
    assert(perception::jit_fresh_bind_should_wait(target, deadline, 75'000'000));
    assert(perception::jit_fresh_bind_should_wait(target, deadline, 74'999'999));
    assert(!perception::jit_fresh_bind_should_wait(target, deadline, 75'000'001));
    assert(!perception::jit_fresh_bind_should_wait(target, deadline, target));
    assert(!perception::jit_fresh_bind_should_wait(deadline, deadline, 1));
    assert(!perception::jit_fresh_bind_should_wait(deadline + 1, deadline, 1));
    assert(perception::jit_fresh_bind_deadline_ns(target) == deadline); // No now+budget extension.
    invalid_argument([]() { (void)perception::jit_fresh_bind_deadline_ns(0); });
    invalid_argument([]() { (void)perception::jit_fresh_bind_deadline_ns(-1); });
    constexpr auto maximum = std::numeric_limits<std::int64_t>::max();
    assert(perception::jit_fresh_bind_deadline_ns(maximum - 2'000'000) == maximum);
    invalid_argument([&]() { (void)perception::jit_fresh_bind_deadline_ns(maximum - 1'999'999); });
    invalid_argument([&]() { (void)perception::jit_fresh_bind_deadline_ns(maximum); });
    for (const std::int64_t invalid : {std::int64_t{0}, std::int64_t{-1}}) {
        invalid_argument([&]() { (void)perception::jit_fresh_bind_should_wait(invalid, deadline, 1); });
        invalid_argument([&]() { (void)perception::jit_fresh_bind_should_wait(target, invalid, 1); });
        invalid_argument([&]() { (void)perception::jit_fresh_bind_should_wait(target, deadline, invalid); });
    }
    invalid_argument([&]() { (void)perception::jit_fresh_bind_should_wait(target, deadline, target + 1); });
}

// An empty queue returns at once without calling either callback.
void test_empty_never_waits_or_reads_candidate() {
    Queue queue(1);
    std::atomic<int> predicate_calls{0};
    std::atomic<int> key_calls{0};
    auto consumer = std::async(std::launch::async, [&]() {
        Result result;
        result.popped = queue.try_pop_after_update_until(
            result.frame, Clock::now() + 30s,
            [&](const Frame&) { ++predicate_calls; return true; },
            [&](const Frame& frame) { ++key_calls; return frame.id; }, result.info);
        return result;
    });
    const auto result = finish(consumer);
    assert(!result.popped);
    assert(!result.info.wait_requested && !result.info.updated && !result.info.cleared);
    assert(result.frame.id == 999 && result.frame.payload == -999);
    assert(predicate_calls == 0 && key_calls == 0);
    assert(queue.empty() && queue.dropped_count() == 0);
}

// A fresh candidate, or a deadline already passed, pops the queued frame at once without waiting.
void test_fresh_candidate_and_expired_deadline_never_wait() {
    for (const bool expired : {false, true}) {
        Queue queue(1);
        queue.push(Frame{10, 1010});
        std::atomic<int> predicate_calls{0};
        auto consumer = std::async(std::launch::async, [&]() {
            Result result;
            const auto deadline = expired ? Clock::now() - 1s : Clock::now() + 30s;
            result.popped = queue.try_pop_after_update_until(
                result.frame, deadline,
                [&](const Frame& frame) {
                    ++predicate_calls;
                    assert(frame.id == 10 && frame.payload == 1010);
                    return expired; // Would wait, except its deadline has passed.
                }, [](const Frame& frame) { return frame.id; }, result.info);
            return result;
        });
        const auto result = finish(consumer);
        assert(result.popped && result.frame.id == 10 && result.frame.payload == 1010);
        assert(!result.info.wait_requested && !result.info.updated && !result.info.cleared);
        assert(predicate_calls == 1);
        assert(queue.empty() && queue.dropped_count() == 0);
    }
}

// A push (copy or move) that replaces the old frame being waited on wakes the waiter, which takes
// the new frame.
void test_push_can_replace_queued_old_frame_and_wake() {
    // Exercise both push overloads; both must wake the condition-variable wait.
    for (const bool copy_push : {false, true}) {
        Queue queue(1);
        queue.push(Frame{10, 1010});
        std::promise<void> observed_old;
        auto observed = observed_old.get_future();
        auto consumer = std::async(std::launch::async, [&]() {
            Result result;
            result.popped = queue.try_pop_after_update_until(
                result.frame, Clock::now() + 30s,
                [&](const Frame& frame) {
                    assert(frame.id == 10 && frame.payload == 1010);
                    // Signalled under the queue mutex. The producer cannot
                    // change the queue until wait_until releases that mutex.
                    observed_old.set_value();
                    return true;
                }, [](const Frame& frame) { return frame.id; }, result.info);
            return result;
        });
        reached(observed);
        assert(queue.size() == 1); // Old frame has not been prematurely popped.
        auto producer = std::async(std::launch::async, [&]() {
            if (copy_push) {
                const Frame newer{11, 1111};
                queue.push(newer);
            } else {
                queue.push(Frame{11, 1111});
            }
        });
        reached(producer);
        const auto result = finish(consumer);
        assert(result.popped && result.frame.id == 11 && result.frame.payload == 1111);
        assert(result.info.wait_requested && result.info.updated && !result.info.cleared);
        assert(queue.empty());
        assert(queue.dropped_count() == 1); // Original max-size-one drop policy.
    }
}

// With no newer frame the wait ends at the absolute deadline and returns the original frame.
void test_timeout_returns_original_candidate() {
    Queue queue(1);
    queue.push(Frame{20, 2020});
    Result result;
    const auto deadline = Clock::now() + 150ms;
    result.popped = queue.try_pop_after_update_until(
        result.frame, deadline, [](const Frame&) { return true; },
        [](const Frame& frame) { return frame.id; }, result.info);
    const auto observed_end = Clock::now();
    // OS scheduling may oversleep. We assert neither exact 2 ms timing nor an
    // upper timing bound; the absolute deadline is not reset on wakeup.
    assert(observed_end >= deadline);
    assert(result.popped && result.frame.id == 20 && result.frame.payload == 2020);
    assert(!result.info.updated && !result.info.cleared);
    assert(queue.empty() && queue.dropped_count() == 0);
}

// A decision that runs past the deadline gets no extra wait; the original frame is returned.
void test_decision_overrun_does_not_renew_deadline() {
    Queue queue(1);
    queue.push(Frame{25, 2525});
    Result result;
    const auto deadline = Clock::now() + 150ms;
    result.popped = queue.try_pop_after_update_until(
        result.frame, deadline,
        [&](const Frame& frame) {
            assert(frame.id == 25);
            // A synthetic slow decision models scheduling delay before the CV
            // wait. Oversleep is allowed; no additional wait budget is granted.
            std::this_thread::sleep_until(deadline);
            return true;
        }, [](const Frame& frame) { return frame.id; }, result.info);
    assert(Clock::now() >= deadline);
    assert(result.popped && result.frame.id == 25 && result.frame.payload == 2525);
    assert(!result.info.wait_requested && !result.info.updated && !result.info.cleared);
    assert(queue.empty() && queue.dropped_count() == 0);
}

// clear() cancels the wait (nothing popped, not counted as a drop) even when a new frame is pushed
// right after.
void test_clear_cancels_even_if_new_frame_follows() {
    // Exercise the clear/new-push race repeatedly without assuming which
    // thread reacquires the mutex first. Either schedule must cancel the wait.
    for (int iteration = 0; iteration < 32; ++iteration) {
        Queue queue(1);
        queue.push(Frame{30, 3030});
        std::promise<void> observed_old;
        auto observed = observed_old.get_future();
        auto consumer = std::async(std::launch::async, [&]() {
            Result result;
            result.popped = queue.try_pop_after_update_until(
                result.frame, Clock::now() + 30s,
                [&](const Frame& frame) {
                    assert(frame.id == 30);
                    observed_old.set_value();
                    return true;
                }, [](const Frame& frame) { return frame.id; }, result.info);
            return result;
        });
        reached(observed);
        queue.clear();
        const bool push_after_clear = iteration != 0;
        if (push_after_clear) queue.push(Frame{31, 3131});
        const auto result = finish(consumer);
        assert(!result.popped);
        assert(result.info.wait_requested && result.info.cleared && !result.info.updated);
        assert(result.frame.id == 999 && result.frame.payload == -999);
        assert(queue.dropped_count() == 0); // clear is cancellation, not overflow.
        Frame next;
        assert(queue.try_pop(next) == push_after_clear);
        if (push_after_clear) assert(next.id == 31 && next.payload == 3131);
        assert(queue.empty());
    }
}

// A throwing callback leaves the frame in the queue and the queue usable.
void test_callback_failure_does_not_consume_or_lock_queue() {
    for (const bool fail_predicate : {false, true}) {
        Queue queue(1);
        queue.push(Frame{40, 4040});
        Result result;
        invalid_argument([&]() {
            (void)queue.try_pop_after_update_until(
                result.frame, Clock::now() + 30s,
                [&](const Frame&) {
                    if (fail_predicate) throw std::invalid_argument("invalid timestamp");
                    return true;
                }, [&](const Frame& frame) {
                    if (!fail_predicate) throw std::invalid_argument("invalid key");
                    return frame.id;
                }, result.info);
        });
        Frame retained;
        assert(queue.try_pop(retained));
        assert(retained.id == 40 && retained.payload == 4040);
        queue.push(Frame{41, 4141});
        assert(queue.try_pop(retained) && retained.id == 41);
        assert(queue.dropped_count() == 0);
    }
}

// Move-only payloads work with the wait and with the ordinary push, try_pop and pop.
void test_move_only_ownership_and_legacy_queue_operations() {
    yolo::ThreadsafeQueue<std::unique_ptr<Frame>> queue(1);
    queue.push(std::make_unique<Frame>(Frame{50, 5050}));
    std::unique_ptr<Frame> output;
    yolo::QueueUpdateWaitInfo info{true, true, true};
    assert(queue.try_pop_after_update_until(
        output, Clock::now() + 30s,
        [](const std::unique_ptr<Frame>&) { return false; },
        [](const std::unique_ptr<Frame>& frame) { return frame->id; }, info));
    assert(output && output->id == 50 && output->payload == 5050);
    assert(!info.wait_requested && !info.updated && !info.cleared);
    queue.push(std::make_unique<Frame>(Frame{51, 5151}));
    queue.push(std::make_unique<Frame>(Frame{52, 5252}));
    assert(queue.dropped_count() == 1);
    assert(queue.try_pop(output) && output->id == 52);
    assert(!queue.try_pop(output));
    queue.push(std::make_unique<Frame>(Frame{53, 5353}));
    assert(queue.pop(output, 100ms) && output->id == 53);
}

} // namespace

int main() {
    test_strict_default_off_and_policy_boundaries();
    test_empty_never_waits_or_reads_candidate();
    test_fresh_candidate_and_expired_deadline_never_wait();
    test_push_can_replace_queued_old_frame_and_wake();
    test_timeout_returns_original_candidate();
    test_decision_overrun_does_not_renew_deadline();
    test_clear_cancels_even_if_new_frame_follows();
    test_callback_failure_does_not_consume_or_lock_queue();
    test_move_only_ownership_and_legacy_queue_operations();
    std::cout << "test_jit_fresh_bind: PASS\n";
}
