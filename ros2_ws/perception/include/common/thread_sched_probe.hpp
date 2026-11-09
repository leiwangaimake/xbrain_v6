/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: thread_sched_probe.hpp
 * Brief: Diagnostic scheduler and page-fault counters of the calling thread (SUP_FRAME deltas).
 *
 * Description:
 * What problem this solves. A span's wall time minus the thread's CPU time is everything the
 * thread did not spend running: waiting on a run queue for a CPU, sleeping on a lock or on a
 * helper, clock overhead. The kernel counts the first of these by itself:
 * /proc/thread-self/schedstat holds this thread's time on a CPU (ns), its time runnable but
 * waiting for one (ns) and the number of timeslices it ran; getrusage(RUSAGE_THREAD) holds its
 * page faults and context switches. ThreadSchedProbe reads both, so that a per-frame log line
 * can carry the differences between two points of that same frame.
 * Where it is used. Supervisor's SUP_FRAME line, only with PERCEPTION_SUP_FRAME_LOG=1; with the
 * log off no probe exists and nothing is opened or read.
 * Limits. The probe belongs to the thread that constructed it (thread-self is resolved when the
 * file is opened). The kernel adds run time at scheduler events (tick, switch), so run_ns can lag
 * the thread CPU clock by up to one tick inside a timeslice; wait_ns is complete whenever the
 * thread is running, which it is while it reads. A counter that could not be read is reported
 * invalid, never guessed. Not available off Linux (both parts read as invalid).
 */

#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __linux__
#include <fcntl.h>
#include <sys/resource.h>
#include <unistd.h>
#endif

namespace perception {

struct ThreadSchedCounters {
    // run_ns, wait_ns and slices were read and parsed.
    bool sched_valid = false;
    std::uint64_t run_ns = 0, wait_ns = 0, slices = 0;
    // The four counters below were read.
    bool usage_valid = false;
    std::int64_t minor_faults = 0, major_faults = 0, voluntary_switches = 0, involuntary_switches = 0;
};

// The schedstat text "<run_ns> <wait_ns> <slices>\n": exactly three unsigned decimal fields
// separated by single spaces, an optional final newline, nothing else, no overflow.
inline bool parse_schedstat(const char* text, std::size_t size, std::uint64_t& run_ns, std::uint64_t& wait_ns,
                            std::uint64_t& slices) noexcept {
    if (text == nullptr) return false;
    std::uint64_t field[3] = {0, 0, 0};
    std::size_t i = 0;
    for (int k = 0; k < 3; ++k) {
        if (k > 0) {
            if (i >= size || text[i] != ' ') return false;
            ++i;
        }
        const std::size_t first = i;
        while (i < size && text[i] >= '0' && text[i] <= '9') {
            const std::uint64_t digit = static_cast<std::uint64_t>(text[i] - '0');
            if (field[k] > (UINT64_MAX - digit) / 10) return false;
            field[k] = field[k] * 10 + digit;
            ++i;
        }
        if (i == first) return false;
    }
    if (i < size && text[i] == '\n') ++i;
    if (i != size) return false;
    run_ns = field[0]; wait_ns = field[1]; slices = field[2];
    return true;
}

// b - a in milliseconds, or -1 when either end is invalid or the counter went backwards.
inline double sched_span_ms(bool a_valid, std::uint64_t a, bool b_valid, std::uint64_t b) noexcept {
    return a_valid && b_valid && b >= a ? static_cast<double>(b - a) / 1.0e6 : -1.0;
}
// b - a as a count, or -1 under the same conditions.
inline std::int64_t sched_span_count(bool a_valid, std::uint64_t a, bool b_valid, std::uint64_t b) noexcept {
    return a_valid && b_valid && b >= a ? static_cast<std::int64_t>(b - a) : -1;
}
inline std::int64_t usage_span_count(bool a_valid, std::int64_t a, bool b_valid, std::int64_t b) noexcept {
    return a_valid && b_valid && b >= a ? b - a : -1;
}

class ThreadSchedProbe {
public:
    // Opens the calling thread's schedstat. Use the probe on that thread only.
    ThreadSchedProbe() noexcept {
#ifdef __linux__
        fd_ = ::open("/proc/thread-self/schedstat", O_RDONLY | O_CLOEXEC);
#endif
    }
    ~ThreadSchedProbe() {
#ifdef __linux__
        if (fd_ >= 0) ::close(fd_);
#endif
    }
    ThreadSchedProbe(const ThreadSchedProbe&) = delete;
    ThreadSchedProbe& operator=(const ThreadSchedProbe&) = delete;

    bool schedstat_open() const noexcept { return fd_ >= 0; }

    // One read of each source (two system calls). No allocation, no exception.
    ThreadSchedCounters read() const noexcept {
        ThreadSchedCounters out;
#ifdef __linux__
        if (fd_ >= 0) {
            char text[96];
            const ssize_t got = ::pread(fd_, text, sizeof(text), 0);
            // A full buffer could be a cut line: three 20-digit fields need 63 characters.
            if (got > 0 && static_cast<std::size_t>(got) < sizeof(text))
                out.sched_valid = parse_schedstat(text, static_cast<std::size_t>(got), out.run_ns, out.wait_ns, out.slices);
        }
        rusage usage{};
        if (::getrusage(RUSAGE_THREAD, &usage) == 0) {
            out.usage_valid = true;
            out.minor_faults = usage.ru_minflt;
            out.major_faults = usage.ru_majflt;
            out.voluntary_switches = usage.ru_nvcsw;
            out.involuntary_switches = usage.ru_nivcsw;
        }
#endif
        return out;
    }

private:
    int fd_ = -1;
};

}  // namespace perception
