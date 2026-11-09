/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_system_clock.hpp
 * Brief: POSIX clock adapter with checked, allocation-free millisecond conversion.
 *
 * Description:
 * What problem this solves. Every r6 message carries two times in its envelope (11 section 3.0):
 * ts, the Unix wall time, and mono, the host CLOCK_MONOTONIC time that every age and timeout is
 * computed from. The publisher asks a PublishClock for both; this is the production
 * implementation on clock_gettime.
 *
 * Boundaries. A read error or an unrepresentable value is returned as -1 and never replaced by a
 * guess: the publisher refuses to stamp a message with an invalid time, which is safer than a
 * plausible wrong one. Nothing here allocates, so it may run on the RT publishing path.
 *
 * Trap. Only mono_millis() may be used for age or timeout arithmetic. Wall time can step in either
 * direction (NTP, manual set), so an age computed from it can go negative or jump.
 */
#pragma once

#include "output/rns_publisher.hpp"
#include <time.h>

namespace perception::rns {

// Truncate sub-millisecond precision. Negative seconds, nanoseconds outside
// [0, 1000000000), or a result exceeding int64 milliseconds return -1.
// Pure conversion: neither reads a clock nor allocates.
Millis timespec_to_millis(const timespec& sample) noexcept;

// Default construction uses the actual host clock_gettime. The optional plain
// function pointer permits deterministic error-path tests without changing the
// host clock; it must obey the clock_gettime contract and must not throw.
// Only mono_millis is suitable for local age/timeout arithmetic. Wall time is
// Unix epoch time for the envelope/logging, and can step in either direction.
class SystemPublishClock final : public PublishClock {
public:
    // A plain function pointer rather than std::function: it cannot allocate or
    // throw, which keeps the clock usable on the allocation-free publish path.
    using ReadClock = int (*)(clockid_t, timespec*);
    explicit SystemPublishClock(ReadClock read_clock = &::clock_gettime) noexcept;
    // CLOCK_REALTIME in milliseconds, -1 on any error (envelope ts only).
    Millis wall_millis() noexcept override;
    // CLOCK_MONOTONIC in milliseconds, -1 on any error (all age arithmetic).
    Millis mono_millis() noexcept override;

private:
    // One read plus conversion; an error is reported as -1, never latched, so a
    // transient failure does not poison later reads (see test_rns_system_clock).
    Millis read_millis(clockid_t id) noexcept;
    const ReadClock read_clock_;
};

}  // namespace perception::rns
