/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_queue.cpp
 * Brief: ThreadsafeQueue unit tests.
 *
 * Description:
 *   1. Verifies bounded queue behavior, timeout, and non-blocking pop.
 *   2. Exercises one producer and one consumer from separate threads.
 *   3. Latest-wins swap policy (19 P19-2): overwrite, empty-replaces-non-empty,
 *      no lost wakeup, no torn read under a fast writer, and every pushed id
 *      accounted for exactly once as popped / overwritten / cleared / pending.
 */

#include "common/threadsafe_queue.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

namespace {

// A pushed value pops back and the queue is then empty.
void test_basic_push_pop() {
    yolo::ThreadsafeQueue<int> queue(5);

    queue.push(10);
    int value = 0;
    assert(queue.pop(value, std::chrono::milliseconds(50)));
    assert(value == 10);
    assert(queue.empty());
}

// A full FIFO drops its oldest item to take a new one.
void test_drop_oldest_when_full() {
    yolo::ThreadsafeQueue<int> queue(5);

    for (int value = 1; value <= 6; ++value) {
        queue.push(value);
    }

    int first_remaining_value = 0;
    assert(queue.size() == 5);
    assert(queue.pop(first_remaining_value, std::chrono::milliseconds(50)));
    assert(first_remaining_value == 2);
}

// pop on an empty queue returns false after its timeout.
void test_timeout_returns_false() {
    yolo::ThreadsafeQueue<int> queue(5);

    int value = 0;
    assert(!queue.pop(value, std::chrono::milliseconds(50)));
}

// try_pop on an empty queue returns false at once.
void test_try_pop_empty_returns_false() {
    yolo::ThreadsafeQueue<int> queue(5);

    int value = 0;
    assert(!queue.try_pop(value));
}

// One producer and one consumer thread pass 1000 items with none lost or repeated (the sum
// matches).
void test_single_producer_single_consumer_sum() {
    constexpr int item_count = 1000;
    constexpr int expected_sum = item_count * (item_count + 1) / 2;

    // Use enough capacity to make this test verify concurrency, not drop policy.
    yolo::ThreadsafeQueue<int> queue(item_count);
    int actual_sum = 0;

    std::thread producer([&queue]() {
        for (int value = 1; value <= item_count; ++value) {
            queue.push(value);
        }
    });

    std::thread consumer([&queue, &actual_sum]() {
        for (int count = 0; count < item_count; ++count) {
            int value = 0;
            const bool popped = queue.pop(value, std::chrono::milliseconds(1000));
            assert(popped);
            actual_sum += value;
        }
    });

    producer.join();
    consumer.join();

    assert(actual_sum == expected_sum);
    assert(queue.empty());
}

using yolo::QueueDiscard;
using yolo::QueuePolicy;
using yolo::ThreadsafeQueue;

// A payload whose consistency can be checked: every body element repeats id.
struct Result {
    std::uint64_t id = 0;
    std::vector<std::uint64_t> body;
};

Result make(std::uint64_t id, std::size_t n) { return Result{id, std::vector<std::uint64_t>(n, id)}; }

template <typename T>
void expect_balanced(const ThreadsafeQueue<T>& q) {
    assert(q.pushed_count() == q.popped_count() + q.dropped_count() + q.cleared_count() + q.size());
}

// Latest-wins keeps only the newest item and reports each overwritten one to the discard observer.
void test_latest_wins_overwrite() {
    ThreadsafeQueue<Result> q(10, QueuePolicy::kLatestWinsSwap);  // capacity is forced to one
    std::vector<std::uint64_t> dropped;
    q.set_discard_observer([&](const Result& r, QueueDiscard why) {
        assert(why == QueueDiscard::kOverwritten);
        dropped.push_back(r.id);
    });
    q.push(make(1, 3));
    q.push(make(2, 3));
    q.push(make(3, 3));
    assert(q.size() == 1U && q.dropped_count() == 2U && (dropped == std::vector<std::uint64_t>{1, 2}));
    Result out;
    assert(q.pop(out, std::chrono::milliseconds(10)) && out.id == 3 && out.body.size() == 3U);
    assert(q.empty() && !q.try_pop(out));
    assert(q.pushed_count() == 3U && q.popped_count() == 1U);
    expect_balanced(q);
}

// A newer empty result replaces an older one that had objects.
void test_latest_wins_empty_replaces_non_empty() {
    ThreadsafeQueue<Result> q(1, QueuePolicy::kLatestWinsSwap);
    q.push(make(7, 5));   // an older result with objects
    q.push(Result{8, {}}); // a newer complete result with none
    Result out;
    assert(q.try_pop(out) && out.id == 8 && out.body.empty());
    expect_balanced(q);
}

// A push wakes a waiting reader promptly (no lost wakeup), 200 times over.
void test_latest_wins_wakes_waiting_reader() {
    ThreadsafeQueue<int> q(1, QueuePolicy::kLatestWinsSwap);
    for (int i = 0; i < 200; ++i) {
        std::thread writer([&q, i] {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            q.push(i);
        });
        int out = -1;
        // A lost wakeup would leave the reader asleep until the 2 s bound.
        const auto t0 = std::chrono::steady_clock::now();
        assert(q.pop(out, std::chrono::milliseconds(2000)) && out == i);
        assert(std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(1000));
        writer.join();
    }
    expect_balanced(q);
}

// Under a fast writer the reader never sees a torn or older result, the newest is always
// delivered, and every id is popped or overwritten exactly once.
void test_latest_wins_concurrent_accounting_and_no_tear() {
    constexpr std::uint64_t kItems = 100000;
    ThreadsafeQueue<Result> q(1, QueuePolicy::kLatestWinsSwap);
    std::vector<std::uint64_t> dropped;  // written only by the writer thread (observer)
    q.set_discard_observer([&](const Result& r, QueueDiscard why) {
        assert(why == QueueDiscard::kOverwritten);
        dropped.push_back(r.id);
    });
    std::vector<std::uint64_t> popped;
    std::atomic<bool> done{false};
    std::thread reader([&] {
        Result out;
        std::uint64_t last = 0;
        while (!done.load(std::memory_order_acquire) || !q.empty()) {
            if (!q.pop(out, std::chrono::milliseconds(5))) continue;
            assert(!out.body.empty());
            assert(std::all_of(out.body.begin(), out.body.end(), [&](std::uint64_t v) { return v == out.id; }));
            assert(out.id > last);  // never an older result after a newer one
            last = out.id;
            popped.push_back(out.id);
        }
    });
    for (std::uint64_t id = 1; id <= kItems; ++id) q.push(make(id, 1 + id % 7));
    done.store(true, std::memory_order_release);
    reader.join();
    // Reconcile by id: every pushed id is popped or overwritten, never both, never missing.
    std::vector<std::uint64_t> all(popped);
    all.insert(all.end(), dropped.begin(), dropped.end());
    std::sort(all.begin(), all.end());
    assert(all.size() == kItems);
    for (std::uint64_t i = 0; i < kItems; ++i) assert(all[i] == i + 1);
    assert(q.dropped_count() == dropped.size() && q.popped_count() == popped.size());
    assert(!popped.empty() && popped.back() == kItems);  // the newest is always delivered
    expect_balanced(q);
}

// clear() reports each discarded item once as cleared, for both policies; clearing an empty queue
// reports nothing.
void test_clear_accounting() {
    std::vector<int> cleared;
    const auto observe = [&](const int& v, QueueDiscard why) {
        assert(why == QueueDiscard::kCleared);
        cleared.push_back(v);
    };
    ThreadsafeQueue<int> latest(1, QueuePolicy::kLatestWinsSwap);
    latest.set_discard_observer(observe);
    latest.push(1);
    latest.clear();
    latest.clear();  // an empty clear reports nothing
    assert(latest.empty() && latest.cleared_count() == 1U && (cleared == std::vector<int>{1}));
    expect_balanced(latest);
    cleared.clear();
    ThreadsafeQueue<int> fifo(4);
    fifo.set_discard_observer(observe);
    for (int i = 0; i < 3; ++i) fifo.push(i);
    fifo.clear();
    assert(fifo.empty() && fifo.cleared_count() == 3U && (cleared == std::vector<int>{0, 1, 2}));
    expect_balanced(fifo);
}

// The FIFO discard observer sees the oldest items as they are dropped.
void test_fifo_drop_observer_sees_oldest() {
    ThreadsafeQueue<int> q(2);
    std::vector<int> dropped;
    q.set_discard_observer([&](const int& v, QueueDiscard why) {
        assert(why == QueueDiscard::kOverwritten);
        dropped.push_back(v);
    });
    for (int i = 1; i <= 4; ++i) q.push(i);
    assert((dropped == std::vector<int>{1, 2}) && q.dropped_count() == 2U && q.size() == 2U);
    int out = 0;
    assert(q.try_pop(out) && out == 3);
    expect_balanced(q);
}

}  // namespace

int main() {
    test_basic_push_pop();
    test_drop_oldest_when_full();
    test_timeout_returns_false();
    test_try_pop_empty_returns_false();
    test_single_producer_single_consumer_sum();
    test_latest_wins_overwrite();
    test_latest_wins_empty_replaces_non_empty();
    test_latest_wins_wakes_waiting_reader();
    test_latest_wins_concurrent_accounting_and_no_tear();
    test_clear_accounting();
    test_fifo_drop_observer_sees_oldest();

    std::cout << "All queue tests passed!" << std::endl;
    return 0;
}
