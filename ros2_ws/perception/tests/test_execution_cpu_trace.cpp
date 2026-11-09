/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_execution_cpu_trace.cpp
 * Brief: Unit tests for the per-task execution CPU trace (wall and thread-CPU clocks) of the task pair.
 *
 * Description:
 * What it checks. The trace records wall time and CLOCK_THREAD_CPUTIME_ID for tasks run on the
 * persistent helper/caller pair. A deterministic rendezvous makes the background finish only after
 * the caller callback exits, so that waiting time is shown not to count as caller CPU service
 * time; unavailable clocks are reported as unknown (-1).
 */

#include "common/execution_cpu_trace.hpp"
#include "common/persistent_task_pair.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {
int checks = 0, reads = 0;
void check(bool value) { ++checks; if (!value) throw std::runtime_error("cpu trace check " + std::to_string(checks)); }
perception::ExecutionCpuStamp fail_clock() noexcept { ++reads; return {}; }
void unknown(const perception::ExecutionCpuRecord& value, const char* reason) {
    check(!value.valid && value.wall_ns == -1 && value.cpu_ns == -1);
    check(std::string(value.reason) == reason);
}
}
int main() {
    try {
        using namespace perception;
        check(!parse_execution_cpu_trace(nullptr)); check(!parse_execution_cpu_trace("0"));
        check(parse_execution_cpu_trace("1"));
        for (const char* text : {"", "true", "01", "2", " 1", "1 "}) {
            bool rejected = false;
            try { (void)parse_execution_cpu_trace(text); } catch (const std::invalid_argument&) { rejected = true; }
            check(rejected);
        }
        ExecutionCpuRecord disabled;
        { ExecutionCpuScope scope(disabled, false, fail_clock); }
        check(reads == 0); unknown(disabled, "disabled");
        ExecutionCpuRecord unavailable;
        { ExecutionCpuScope scope(unavailable, true, fail_clock); scope.finish(); scope.finish(); }
        check(reads == 2); unknown(unavailable, "clock_unavailable");
        ExecutionCpuStamp a{100, 5, 7, true, "ok"}, b{200, 15, 7, true, "ok"};
        auto delta = execution_cpu_difference(a, b);
        check(delta.valid && delta.wall_ns == 100 && delta.cpu_ns == 10 && delta.tid == 7);
        b.tid = 8; unknown(execution_cpu_difference(a, b), "cross_thread");
        b.tid = 7; b.cpu_ns = 4; unknown(execution_cpu_difference(a, b), "invalid_clock_order");
        b.cpu_ns = 15; b.wall_ns = 99; unknown(execution_cpu_difference(a, b), "invalid_clock_order");
        b.valid = false; b.reason = "clock_unavailable";
        unknown(execution_cpu_difference(a, b), "clock_unavailable");
        errno = EDOM; (void)execution_cpu_stamp(); check(errno == EDOM);

        ExecutionCpuRecord work;
        { ExecutionCpuScope scope(work, true); volatile std::uint64_t sum = 0;
          for (std::uint64_t i = 0; i < 200000; ++i) {
              sum += i;
          }
          (void)sum;
        }
#ifdef __linux__
        check(work.valid && work.cpu_ns > 0 && work.wall_ns > 0);
        auto begin = execution_cpu_stamp(); ExecutionCpuStamp other;
        std::thread crossing([&] { other = execution_cpu_stamp(); }); crossing.join();
        unknown(execution_cpu_difference(begin, other), "cross_thread");
        ExecutionCpuRecord sleeper;
        { ExecutionCpuScope scope(sleeper, true); std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
        check(sleeper.valid && sleeper.wall_ns >= 20000000 && sleeper.wall_ns > sleeper.cpu_ns);
        // Deterministic rendezvous: the background cannot finish until the
        // caller callback has exited. Its time is not caller CPU service time.
        PersistentTaskPair pair; std::atomic<bool> caller_done{false};
        ExecutionCpuRecord callback, outer;
        { ExecutionCpuScope wrapper(outer, true);
          pair.run([&] { while (!caller_done.load()) std::this_thread::yield();
                        std::this_thread::sleep_for(std::chrono::milliseconds(25)); },
                   [&] { { ExecutionCpuScope scope(callback, true); }
                         caller_done.store(true); }); }
        check(callback.valid && outer.valid && outer.wall_ns > callback.wall_ns);
        check(outer.end_ns - callback.end_ns >= 20000000);
#else
        unknown(work, "unsupported_platform");
#endif
        ExecutionCpuRecord exception;
        try { ExecutionCpuScope scope(exception, true); throw std::runtime_error("injected"); }
        catch (const std::runtime_error&) {}
#ifdef __linux__
        check(exception.valid);
#else
        unknown(exception, "unsupported_platform");
#endif
        std::cout << "{\"status\":\"PASS\",\"checks\":" << checks
                  << ",\"linux_cpu_clock_valid\":" << (work.valid ? "true" : "false")
                  << ",\"unsupported_is_unknown\":true,\"synthetic_only\":true}\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
