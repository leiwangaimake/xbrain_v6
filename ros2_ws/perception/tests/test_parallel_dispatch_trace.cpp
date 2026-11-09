/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_parallel_dispatch_trace.cpp
 * Brief: Unit tests for the inference helper worker (ParallelTaskWorker) and its join-wait CPU record.
 *
 * Description:
 * What it checks. ParallelTaskWorker, extracted verbatim from InferenceModule, runs one task on a
 * persistent helper while the caller runs the other. With real threads the tests check that the two
 * tasks run on different threads with the caller task on the calling thread, that run() returns only
 * after both finish in either order, that a re-entrant run() while the helper is busy is refused,
 * that the caller exception wins and the worker stays reusable, and that the optional join-wait
 * execution CPU record (PERCEPTION_EXECUTION_CPU_TRACE) is taken on the calling thread inside run().
 */

#include "common/parallel_dispatch_trace.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using perception::ParallelTaskWorker;
namespace {
int checks = 0;
void check(bool value) {
    if (!value) throw std::runtime_error("parallel task worker check " + std::to_string(checks + 1));
    ++checks;
}
void wait_flag(const std::atomic<bool>& flag) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!flag.load()) {
        if (std::chrono::steady_clock::now() > deadline) throw std::runtime_error("test handshake timeout");
        std::this_thread::yield();
    }
}
void real_thread_tests() {
    ParallelTaskWorker worker;
    std::atomic<int> tasks{0};
    worker.run([&] { ++tasks; }, [&] { ++tasks; }, nullptr);
    worker.run([&] { ++tasks; }, [&] { ++tasks; });
    check(tasks.load() == 4);

    const auto caller_id = std::this_thread::get_id();
    for (bool helper_finishes_first : {true, false}) {
        std::atomic<bool> permit{false};
        bool helper_done = false, caller_done = false;
        std::thread::id helper_id, task_caller_id;
        worker.run([&] {
            helper_id = std::this_thread::get_id();
            if (helper_finishes_first) permit.store(true);
            else { wait_flag(permit); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
            helper_done = true;
        }, [&] {
            task_caller_id = std::this_thread::get_id();
            if (helper_finishes_first) { wait_flag(permit); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
            else permit.store(true);
            caller_done = true;
        });
        check(helper_done && caller_done);
        check(task_caller_id == caller_id && helper_id != std::thread::id() && helper_id != caller_id);
    }

    bool busy_rejected = false;
    // Force helper still-running without adding any production hook.
    std::atomic<bool> release_helper{false};
    worker.run([&] { wait_flag(release_helper); }, [&] {
        try { worker.run([] {}, [] {}); }
        catch (const std::logic_error&) { busy_rejected = true; }
        release_helper.store(true);
    });
    check(busy_rejected);

    // Both callbacks always finish; caller exception wins when both throw.
    for (int scenario : {1, 2, 3}) {
        bool helper_ran = false, caller_ran = false;
        std::string error;
        try {
            worker.run([&] {
                helper_ran = true;
                if (scenario & 1) throw std::runtime_error("helper");
            }, [&] {
                caller_ran = true;
                if (scenario & 2) throw std::runtime_error("caller");
            });
        } catch (const std::runtime_error& e) { error = e.what(); }
        check(helper_ran && caller_ran);
        check(error == (scenario & 2 ? "caller" : "helper"));
        worker.run([&] { ++tasks; }, [&] { ++tasks; });
    }
    check(tasks.load() == 10);
    for (int i = 0; i < 100; ++i) {
        int helper_value = -1, caller_value = -1;
        worker.run([&] { helper_value = i; }, [&] { caller_value = i; });
        check(helper_value == i && caller_value == i);
    }

    // The join-wait record is taken on the calling thread, inside run(), and
    // spans the wait for a helper that is still running.
    perception::ExecutionCpuRecord join;
#ifdef __linux__
    const auto before = perception::execution_cpu_stamp();
    worker.run([] { std::this_thread::sleep_for(std::chrono::milliseconds(20)); }, [] {}, &join);
    const auto after = perception::execution_cpu_stamp();
    check(before.valid && after.valid && join.enabled && join.valid);
    check(join.tid == before.tid && before.wall_ns <= join.begin_ns && join.end_ns <= after.wall_ns);
    check(join.wall_ns >= 1000000);
#else
    worker.run([] {}, [] {}, &join);
    check(join.enabled && !join.valid && std::string(join.reason) == "unsupported_platform");
#endif
}
} // namespace

int main() {
    real_thread_tests();
    std::cout << "PASS parallel_dispatch_trace checks=" << checks
              << " production_worker_shared=1 device_used=0\n";
}
