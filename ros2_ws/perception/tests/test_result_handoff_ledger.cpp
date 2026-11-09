/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_result_handoff_ledger.cpp
 * Brief: ResultHandoffLedger with the latest-wins result queue (B8-4).
 *
 * Description:
 *   1. Every pushed frame_id is popped, overwritten or cleared exactly once,
 *      checked by identity against the ledger and against the queue counters.
 *   2. The identity checks fire on out-of-order and duplicate ids and stay at
 *      zero for one in-order producer and one consumer.
 *   3. Window lists are bounded and drained; cumulative totals are not.
 */

#include "common/threadsafe_queue.hpp"
#include "supervisor/result_handoff_ledger.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

namespace {

using perception::ResultHandoffLedger;
using perception::ResultHandoffWindow;
using yolo::QueueDiscard;
using yolo::QueuePolicy;
using yolo::ThreadsafeQueue;

struct Item {
    std::uint64_t frame_id = 0;
};

void wire(ThreadsafeQueue<Item>& q, ResultHandoffLedger& ledger) {
    q.set_discard_observer([&ledger](const Item& item, QueueDiscard why) {
        ledger.on_discarded(item.frame_id, why == QueueDiscard::kCleared);
    });
}

// Each result is counted exactly once as popped, overwritten or cleared, with its id listed in the
// window; a drain keeps the totals and resets the window.
void test_sequential_fates() {
    ThreadsafeQueue<Item> q(1, QueuePolicy::kLatestWinsSwap);
    ResultHandoffLedger ledger;
    wire(q, ledger);
    Item out;
    q.push({0});  // frame 0 is a real capture frame
    assert(q.try_pop(out) && out.frame_id == 0);
    ledger.on_popped(out.frame_id);
    q.push({1});
    q.push({2});  // overwrites 1
    assert(q.try_pop(out) && out.frame_id == 2);
    ledger.on_popped(out.frame_id);
    q.push({3});
    q.clear();    // stop discards 3
    const ResultHandoffWindow w = ledger.drain();
    assert(w.popped == 2 && w.overwritten == 1 && w.cleared == 1);
    assert((w.overwritten_ids == std::vector<std::uint64_t>{1}));
    assert((w.cleared_ids == std::vector<std::uint64_t>{3}));
    assert(w.window_popped == 2 && w.window_first_popped_id == 0 && w.window_last_popped_id == 2);
    assert(w.order_violations == 0 && w.identity_conflicts == 0 && w.ids_not_listed == 0);
    assert(q.pushed_count() == w.popped + w.overwritten + w.cleared + q.size());

    const ResultHandoffWindow next = ledger.drain();  // totals stay, window resets
    assert(next.popped == 2 && next.overwritten == 1 && next.cleared == 1);
    assert(next.window_popped == 0 && next.overwritten_ids.empty() && next.cleared_ids.empty());
}

// Out-of-order pops, an id both delivered and discarded, a double discard and backwards
// overwritten ids are all counted.
void test_checks_fire() {
    ResultHandoffLedger ledger;
    ledger.on_popped(5);
    ledger.on_popped(4);          // older after newer
    assert(ledger.drain().order_violations == 1);
    ledger.on_discarded(4, false);  // already delivered, now also discarded
    ResultHandoffWindow w = ledger.drain();
    assert(w.identity_conflicts == 1);
    ledger.on_discarded(9, false);
    ledger.on_discarded(9, true);   // discarded twice
    ledger.on_discarded(8, false);  // overwritten ids go backwards
    w = ledger.drain();
    assert(w.identity_conflicts == 2 && w.order_violations == 2);
    ledger.on_popped(9);            // delivered after being discarded
    assert(ledger.drain().identity_conflicts == 3);
}

// The per-window id list is capped; ids beyond the cap are counted as not listed.
void test_window_bound() {
    ResultHandoffLedger ledger;
    const std::uint64_t n = ResultHandoffLedger::kWindowIds + 5;
    for (std::uint64_t id = 1; id <= n; ++id) ledger.on_discarded(id, false);
    ResultHandoffWindow w = ledger.drain();
    assert(w.overwritten == n && w.overwritten_ids.size() == ResultHandoffLedger::kWindowIds);
    assert(w.ids_not_listed == 5 && w.overwritten_ids.front() == 1);
    w = ledger.drain();
    assert(w.ids_not_listed == 0 && w.overwritten_ids.empty() && w.overwritten == n);
}

// With a fast writer, a reader and a frequent drainer, the ledger totals equal the queue counters,
// with no order or identity errors.
void test_concurrent_reconciliation() {
    constexpr std::uint64_t kItems = 50000;
    ThreadsafeQueue<Item> q(1, QueuePolicy::kLatestWinsSwap);
    ResultHandoffLedger ledger;
    wire(q, ledger);
    std::atomic<bool> done{false};
    std::vector<std::uint64_t> seen;  // ids listed by the windows, all kinds
    std::uint64_t listed_popped = 0;
    std::thread reader([&] {
        Item out;
        while (!done.load(std::memory_order_acquire) || !q.empty()) {
            if (q.pop(out, std::chrono::milliseconds(5))) ledger.on_popped(out.frame_id);
        }
    });
    std::thread drainer([&] {  // like the 10 s service line, but often
        while (!done.load(std::memory_order_acquire)) {
            const ResultHandoffWindow w = ledger.drain();
            seen.insert(seen.end(), w.overwritten_ids.begin(), w.overwritten_ids.end());
            listed_popped += w.window_popped;
            assert(w.ids_not_listed == 0 || w.overwritten_ids.size() == ResultHandoffLedger::kWindowIds);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });
    for (std::uint64_t id = 0; id < kItems; ++id) q.push({id});
    done.store(true, std::memory_order_release);
    reader.join();
    drainer.join();
    q.push({kItems});
    q.clear();  // one result left at stop
    const ResultHandoffWindow w = ledger.drain();
    listed_popped += w.window_popped;
    assert(w.order_violations == 0 && w.identity_conflicts == 0);
    assert(w.popped == q.popped_count() && w.overwritten == q.dropped_count() && w.cleared == q.cleared_count());
    assert(w.popped + w.overwritten + w.cleared == kItems + 1 && q.pushed_count() == kItems + 1);
    assert(listed_popped == w.popped);
    assert((w.cleared_ids == std::vector<std::uint64_t>{kItems}));
}

}  // namespace

int main() {
    test_sequential_fates();
    test_checks_fire();
    test_window_bound();
    test_concurrent_reconciliation();
    std::cout << "All result handoff ledger tests passed!" << std::endl;
    return 0;
}
