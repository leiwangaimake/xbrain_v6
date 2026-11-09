/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: execution_cpu_trace.hpp
 * Brief: Diagnostic thread wall/CPU stamps around inference stages (PERCEPTION_EXECUTION_CPU_TRACE).
 *
 * Description:
 * A diagnostic switch; the service clears it at startup. When off, a scope reads no clock.
 */

#pragma once

#include <cstdint>
#include <cerrno>
#include <cstring>
#include <ostream>
#include <stdexcept>
#include <time.h>
#ifdef __linux__
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace perception {

// Observational only. No allocation, logging, throwing clocks or global state.
// wall-cpu is NOT a scheduler-delay counter: it also contains blocking, locks,
// device/child waits, and clock-read overhead. Never sum nested spans.
inline bool parse_execution_cpu_trace(const char* value) {
    if (!value || std::strcmp(value, "0") == 0) return false;
    if (std::strcmp(value, "1") == 0) return true;
    throw std::invalid_argument("PERCEPTION_EXECUTION_CPU_TRACE must be exactly 0 or 1");
}

struct ExecutionCpuStamp {
    std::int64_t wall_ns = -1;
    std::int64_t cpu_ns = -1;
    std::int64_t tid = -1;
    bool valid = false;
    const char* reason = "clock_unavailable";
};

inline ExecutionCpuStamp execution_cpu_stamp() noexcept {
    struct PreserveErrno { int saved = errno; ~PreserveErrno() noexcept { errno = saved; } } preserve_errno;
    ExecutionCpuStamp value;
#if defined(__linux__) && defined(CLOCK_THREAD_CPUTIME_ID)
    timespec wall{}, cpu{};
    value.tid = static_cast<std::int64_t>(::syscall(SYS_gettid));
    if (value.tid <= 0 || ::clock_gettime(CLOCK_MONOTONIC, &wall) != 0 ||
        ::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu) != 0) return value;
    value.wall_ns = static_cast<std::int64_t>(wall.tv_sec) * 1000000000LL + wall.tv_nsec;
    value.cpu_ns = static_cast<std::int64_t>(cpu.tv_sec) * 1000000000LL + cpu.tv_nsec;
    value.valid = value.wall_ns > 0 && value.cpu_ns >= 0;
    value.reason = value.valid ? "ok" : "clock_unavailable";
#else
    value.reason = "unsupported_platform";
#endif
    return value;
}

struct ExecutionCpuRecord {
    bool enabled = false;
    bool valid = false;
    const char* reason = "disabled";
    std::int64_t tid = -1;
    std::int64_t begin_ns = -1;
    std::int64_t end_ns = -1;
    std::int64_t wall_ns = -1;
    std::int64_t cpu_ns = -1;
};

inline ExecutionCpuRecord execution_cpu_difference(
    const ExecutionCpuStamp& begin, const ExecutionCpuStamp& end) noexcept {
    ExecutionCpuRecord result;
    result.enabled = true;
    result.reason = !begin.valid ? begin.reason : !end.valid ? end.reason : "invalid_clock_order";
    if (!begin.valid || !end.valid) return result;
    if (begin.tid <= 0 || begin.tid != end.tid) {
        result.reason = "cross_thread";
        return result;
    }
    if (begin.wall_ns <= 0 || begin.cpu_ns < 0 || end.wall_ns < begin.wall_ns ||
        end.cpu_ns < begin.cpu_ns) return result;
    result.tid = begin.tid;
    result.begin_ns = begin.wall_ns;
    result.end_ns = end.wall_ns;
    result.wall_ns = end.wall_ns - begin.wall_ns;
    result.cpu_ns = end.cpu_ns - begin.cpu_ns;
    result.valid = true;
    result.reason = "ok";
    return result;
}

class ExecutionCpuScope {
public:
    using Reader = ExecutionCpuStamp (*)() noexcept;
    ExecutionCpuScope(ExecutionCpuRecord& result, bool enabled,
                      Reader reader = execution_cpu_stamp) noexcept
        : result_(result), enabled_(enabled), reader_(reader) {
        result_ = ExecutionCpuRecord{};
        if (enabled_) begin_ = reader_();
    }
    ~ExecutionCpuScope() noexcept { finish(); }
    ExecutionCpuScope(const ExecutionCpuScope&) = delete;
    ExecutionCpuScope& operator=(const ExecutionCpuScope&) = delete;
    void finish() noexcept {
        if (enabled_ && !finished_) result_ = execution_cpu_difference(begin_, reader_());
        finished_ = true;
    }
private:
    ExecutionCpuRecord& result_;
    bool enabled_, finished_ = false;
    Reader reader_;
    ExecutionCpuStamp begin_;
};

// Formatting only at joined caller logging boundaries, never inside task lanes.
inline std::ostream& operator<<(std::ostream& out, const ExecutionCpuRecord& value) {
    return out << " cpu_trace_enabled=" << static_cast<int>(value.enabled)
        << " cpu_trace_valid=" << static_cast<int>(value.valid)
        << " cpu_trace_reason=" << value.reason << " cpu_trace_tid=" << value.tid
        << " cpu_trace_begin_ns=" << value.begin_ns << " cpu_trace_end_ns=" << value.end_ns
        << " cpu_trace_wall_ns=" << value.wall_ns << " cpu_trace_cpu_ns=" << value.cpu_ns;
}

} // namespace perception
