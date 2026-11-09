/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_persistent_task_pair.cpp
 * Brief: Unit tests for the persistent helper/caller task pair used by inference postprocess.
 *
 * Description:
 * What it checks. The helper thread persists across frames and is initialised once; overlapping
 * tasks join and exceptions are joined before being rethrown; stop waits for the helper and an
 * initialisation failure is reported; the optional fixed delay is applied as configured.
 */

#include "common/persistent_task_pair.hpp"
#include "common/jit_schedule.hpp"

#include <cassert>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

template <typename Function>
void require_error(Function function, const char* expected) {
    bool caught = false;
    try { function(); }
    catch (const std::exception& error) {
        caught = std::string(error.what()) == expected;
    }
    assert(caught);
}

// The initializer runs once on the background thread, which then runs every background task; run()
// returns only after both tasks finish; after stop() (repeatable) run() throws.
void test_persistence_and_initialization() {
    std::thread::id initialized_thread;
    int initialization_count = 0;
    perception::PersistentTaskPair worker([&]() {
        initialized_thread = std::this_thread::get_id();
        ++initialization_count;
    });
    assert(initialization_count == 1);
    assert(initialized_thread != std::this_thread::get_id());
    std::vector<std::thread::id> visits;
    int output = -1;
    for (int i = 0; i != 40; ++i) {
        worker.run([&]() { visits.push_back(std::this_thread::get_id()); output = i; }, []() {});
        assert(output == i); // Deliberately non-atomic: run supplies happens-before.
    }
    assert(visits.size() == 40);
    for (const auto& id : visits) assert(id == initialized_thread);
    worker.stop();
    worker.stop();
    require_error([&]() { worker.run([]() {}, []() {}); }, "persistent task pair is not idle");
}

// Both tasks run at the same time; an exception from either is rethrown only after both have
// finished (the foreground one if both throw), and the pair stays usable.
void test_overlap_and_exception_join() {
    perception::PersistentTaskPair worker;
    std::promise<void> background_started;
    std::promise<void> foreground_started;
    auto bg = background_started.get_future();
    auto fg = foreground_started.get_future();
    worker.run([&]() { background_started.set_value(); fg.get(); },
               [&]() { bg.get(); foreground_started.set_value(); });

    bool background_completed = false;
    std::promise<void> release;
    auto released = release.get_future();
    require_error([&]() {
        worker.run([&]() { released.get(); background_completed = true; }, [&]() {
            release.set_value();
            throw std::runtime_error("foreground");
        });
    }, "foreground");
    assert(background_completed);

    bool foreground_completed = false;
    require_error([&]() {
        worker.run([]() { throw std::runtime_error("background"); },
                   [&]() { foreground_completed = true; });
    }, "background");
    assert(foreground_completed);

    background_completed = false;
    require_error([&]() {
        worker.run([&]() {
            background_completed = true;
            throw std::runtime_error("background");
        }, []() { throw std::runtime_error("foreground"); });
    }, "foreground");
    assert(background_completed);
    worker.run([]() {}, []() {}); // No abandoned callbacks after either failure.
}

// A throwing initializer fails construction; stop() waits for a running batch to finish.
void test_stop_waits_and_initialization_failure() {
    require_error([]() {
        perception::PersistentTaskPair worker([]() { throw std::runtime_error("initialize"); });
    }, "initialize");

    perception::PersistentTaskPair worker;
    std::promise<void> started;
    std::promise<void> release;
    auto start = started.get_future();
    auto released = release.get_future();
    bool completed = false;
    std::thread caller([&]() {
        worker.run([&]() { started.set_value(); released.get(); completed = true; }, []() {});
    });
    start.get();
    std::thread stopper([&]() { worker.stop(); assert(completed); });
    release.set_value();
    stopper.join();
    caller.join();
    assert(completed);
}

// The JIT bind delay accepts plain numbers (exponent allowed) from 0 to the cap; spaces, units,
// NaN/inf, negatives, values above the cap, under- or overflow, hex and malformed text are
// refused.
void test_fixed_delay() {
    assert(perception::parse_jit_bind_delay_ms("35.74915263", 100.0) == 35.74915263);
    assert(perception::parse_jit_bind_delay_ms("0", 100.0) == 0.0);
    assert(perception::parse_jit_bind_delay_ms("1e2", 100.0) == 100.0);
    for (const std::string invalid : {"", " ", " 1", "1 ", "1ms", "NaN", "nan", "inf",
         "-0.1", "100.0001", "1e9999", "1e-9999", "0x1p2", "+", ".", "1.2.3", "1e"}) {
        bool rejected = false;
        try { (void)perception::parse_jit_bind_delay_ms(invalid, 100.0); }
        catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
    }
}

} // namespace

int main() {
    test_persistence_and_initialization();
    test_overlap_and_exception_join();
    test_stop_waits_and_initialization_failure();
    test_fixed_delay();
}
