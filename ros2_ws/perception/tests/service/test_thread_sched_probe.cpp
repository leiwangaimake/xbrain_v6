/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_thread_sched_probe.cpp
 * Brief: Host test of the diagnostic thread scheduler probe behind the SUP_FRAME deltas.
 *
 * Description:
 * What it checks. parse_schedstat accepts exactly "<run> <wait> <slices>" with an optional final
 * newline and refuses everything else (missing or extra fields, signs, other separators,
 * overflow); the span helpers give -1 for an invalid end or a counter that went backwards; a
 * read allocates nothing. On Linux the live probe must agree with the kernel: counters never go
 * backwards, run time and the thread CPU clock grow together over a busy loop, sleeping adds a
 * voluntary switch and no comparable run time, touching fresh pages adds minor faults, and a
 * probe made on another thread reads that thread's counters, not this one's. Off Linux the probe
 * reads as invalid and only the parser and helpers are checked.
 */

#include "common/thread_sched_probe.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <new>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

namespace {
std::atomic<std::size_t> allocations{0};
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

[[maybe_unused]] std::int64_t thread_cpu_ns() {
    timespec ts{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
// Spin on this thread until it has used `ns` of its own CPU time.
[[maybe_unused]] void burn_cpu(std::int64_t ns) {
    const std::int64_t until = thread_cpu_ns() + ns;
    volatile std::uint64_t sink = 0;
    while (thread_cpu_ns() < until)
        for (int i = 0; i < 20000; ++i) sink = sink + static_cast<std::uint64_t>(i);
}
bool parses(const std::string& text, std::uint64_t run = 0, std::uint64_t wait = 0, std::uint64_t slices = 0) {
    std::uint64_t a = 111, b = 222, c = 333;
    if (!perception::parse_schedstat(text.data(), text.size(), a, b, c)) {
        // A refused text must leave the outputs alone.
        return a == 111 && b == 222 && c == 333 ? false : (++failures, false);
    }
    return a == run && b == wait && c == slices;
}
}  // namespace

void* operator new(std::size_t n) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    void* p = std::malloc(n == 0 ? 1 : n);
    if (!p) throw std::bad_alloc();
    return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

int main() {
    using perception::ThreadSchedCounters;
    using perception::ThreadSchedProbe;

    // ---- the schedstat text ----
    CHECK(parses("12345 678 9\n", 12345, 678, 9));
    CHECK(parses("12345 678 9", 12345, 678, 9));
    CHECK(parses("0 0 0\n", 0, 0, 0));
    CHECK(parses("18446744073709551615 18446744073709551615 18446744073709551615\n", UINT64_MAX, UINT64_MAX, UINT64_MAX));
    CHECK(parses("007 08 9\n", 7, 8, 9));
    for (const char* bad : {"", "\n", "1", "1 2", "1 2\n", "1 2 3 4\n", "1 2 3 \n", " 1 2 3\n", "1  2 3\n", "1\t2 3\n",
                            "1 2 3\n\n", "1 2 3\nx", "-1 2 3\n", "+1 2 3\n", "1 2 3x\n", "1 2 x\n", "a b c\n", "1,2,3\n",
                            "18446744073709551616 2 3\n", "1 2 99999999999999999999999\n", "1 2 3\r\n", "0x10 2 3\n"})
        CHECK(!parses(bad));
    {
        // Only `size` bytes are read: digits after the given size are not part of the text.
        const char text[] = "12 34 56789";
        std::uint64_t a = 0, b = 0, c = 0;
        CHECK(perception::parse_schedstat(text, 8, a, b, c) && a == 12 && b == 34 && c == 56);
        CHECK(!perception::parse_schedstat(nullptr, 0, a, b, c));
        CHECK(!perception::parse_schedstat(text, 0, a, b, c));
    }

    // ---- span helpers ----
    CHECK(perception::sched_span_ms(true, 1000000, true, 3500000) == 2.5);
    CHECK(perception::sched_span_ms(true, 5, true, 5) == 0.0);
    CHECK(perception::sched_span_ms(false, 1, true, 2) == -1.0 && perception::sched_span_ms(true, 1, false, 2) == -1.0);
    CHECK(perception::sched_span_ms(true, 3, true, 2) == -1.0);
    CHECK(perception::sched_span_count(true, 4, true, 9) == 5 && perception::sched_span_count(true, 9, true, 4) == -1);
    CHECK(perception::sched_span_count(false, 4, true, 9) == -1 && perception::sched_span_count(true, 4, false, 9) == -1);
    CHECK(perception::usage_span_count(true, 4, true, 9) == 5 && perception::usage_span_count(true, 9, true, 4) == -1);
    CHECK(perception::usage_span_count(false, 4, true, 9) == -1 && perception::usage_span_count(true, 4, false, 9) == -1);

    // ---- the live probe ----
    ThreadSchedProbe probe;
    const ThreadSchedCounters first = probe.read();
    {
        const auto before = allocations.load(std::memory_order_relaxed);
        for (int i = 0; i < 100; ++i) (void)probe.read();
        CHECK(allocations.load(std::memory_order_relaxed) == before);
    }
    bool kernel_schedstat = false;
#ifdef __linux__
    {
        // The probe may only be invalid where the kernel itself gives no schedstat.
        std::ifstream file("/proc/thread-self/schedstat");
        std::string line;
        kernel_schedstat = file.good() && static_cast<bool>(std::getline(file, line)) && !line.empty();
    }
    CHECK(probe.schedstat_open() == kernel_schedstat);
    CHECK(first.sched_valid == kernel_schedstat);
    CHECK(first.usage_valid);
    if (first.usage_valid) {
        // Busy for 60 ms of this thread's CPU time.
        const ThreadSchedCounters a = probe.read();
        const std::int64_t cpu_a = thread_cpu_ns();
        burn_cpu(60000000);
        const std::int64_t cpu_b = thread_cpu_ns();
        const ThreadSchedCounters b = probe.read();
        CHECK(b.usage_valid && b.minor_faults >= a.minor_faults && b.major_faults >= a.major_faults);
        CHECK(b.voluntary_switches >= a.voluntary_switches && b.involuntary_switches >= a.involuntary_switches);
        if (kernel_schedstat) {
            CHECK(a.sched_valid && b.sched_valid);
            CHECK(b.run_ns >= a.run_ns && b.wait_ns >= a.wait_ns && b.slices >= a.slices);
            // The kernel's run time and the thread CPU clock describe the same 60 ms; the kernel
            // adds run time at ticks and switches, so allow 20 ms either way.
            const double run_ms = perception::sched_span_ms(true, a.run_ns, true, b.run_ns);
            const double cpu_ms = static_cast<double>(cpu_b - cpu_a) / 1.0e6;
            CHECK(run_ms > cpu_ms - 20.0 && run_ms < cpu_ms + 20.0);
            std::cout << "busy: thread_cpu_ms=" << cpu_ms << " sched_run_ms=" << run_ms
                      << " sched_wait_ms=" << perception::sched_span_ms(true, a.wait_ns, true, b.wait_ns)
                      << " slices=" << perception::sched_span_count(true, a.slices, true, b.slices)
                      << " nivcsw=" << b.involuntary_switches - a.involuntary_switches << '\n';
        }
        // Asleep for 80 ms: a voluntary switch, and far less run time than the sleep.
        const ThreadSchedCounters c = probe.read();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        const ThreadSchedCounters d = probe.read();
        CHECK(d.voluntary_switches > c.voluntary_switches);
        if (kernel_schedstat) {
            CHECK(d.sched_valid && d.run_ns >= c.run_ns && d.slices > c.slices);
            CHECK(perception::sched_span_ms(true, c.run_ns, true, d.run_ns) < 40.0);
        }
        // Fresh pages: 256 pages touched for the first time are at least 100 minor faults
        // (the allocator may hand back a few already-mapped ones).
        const ThreadSchedCounters e = probe.read();
        {
            const std::size_t bytes = 256U * 4096U;
            char* fresh = static_cast<char*>(std::malloc(bytes));
            CHECK(fresh != nullptr);
            if (fresh) {
                for (std::size_t i = 0; i < bytes; i += 4096) fresh[i] = static_cast<char>(i);
                volatile char sink = fresh[bytes - 1];
                (void)sink;
                std::free(fresh);
            }
        }
        const ThreadSchedCounters f = probe.read();
        CHECK(f.minor_faults - e.minor_faults >= 100);
        // A probe belongs to its thread: one made on a busy helper counts the helper's run time,
        // while this thread, asleep meanwhile, gains almost none in its own probe.
        const ThreadSchedCounters g = probe.read();
        ThreadSchedCounters helper_begin, helper_end;
        std::thread helper([&]() {
            ThreadSchedProbe own;
            helper_begin = own.read();
            burn_cpu(50000000);
            helper_end = own.read();
        });
        helper.join();
        const ThreadSchedCounters h = probe.read();
        CHECK(helper_begin.usage_valid && helper_end.usage_valid);
        if (kernel_schedstat) {
            CHECK(helper_begin.sched_valid && helper_end.sched_valid);
            CHECK(perception::sched_span_ms(true, helper_begin.run_ns, true, helper_end.run_ns) > 30.0);
            CHECK(perception::sched_span_ms(true, g.run_ns, true, h.run_ns) < 25.0);
        }
    }
#else
    CHECK(!probe.schedstat_open() && !first.sched_valid && !first.usage_valid);
#endif
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"thread_sched_probe_host\",\"schedstat\":" << (kernel_schedstat ? "true" : "false")
              << ",\"usage\":" << (first.usage_valid ? "true" : "false") << "}" << std::endl;
    return failures ? 1 : 0;
}
