/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_system_clock.cpp
 * Brief: Checked CLOCK_REALTIME/CLOCK_MONOTONIC reads; errors stay explicit.
 *
 * Description:
 * What problem this solves. Implements SystemPublishClock: the envelope's wall and monotonic
 * milliseconds read straight from clock_gettime (11 section 3.0 envelope fields).
 *
 * Why the checks are explicit. timespec is signed and platform-sized; converting it without
 * range checks could wrap into a valid-looking time. Every path that cannot produce an exact
 * millisecond count returns -1, which the publisher treats as "no valid time" and refuses to
 * stamp, instead of publishing a message with a fabricated timestamp.
 */
#include "output/rns_system_clock.hpp"

#include <cstdint>
#include <limits>

namespace perception::rns {

Millis timespec_to_millis(const timespec& sample) noexcept {
    // A negative second or an out-of-range nanosecond field is not a valid POSIX
    // time; reject it rather than normalise, because normalising would hide a
    // broken clock source behind a plausible value.
    if (sample.tv_sec < 0 || sample.tv_nsec < 0 || sample.tv_nsec >= 1000000000L)
        return -1;
    // Unsigned arithmetic from here on: the sign was checked above, and unsigned
    // overflow is well defined, so the bound check below is exact.
    const auto seconds = static_cast<std::uintmax_t>(sample.tv_sec);
    const auto fraction_ms = static_cast<std::uintmax_t>(sample.tv_nsec / 1000000L);
    constexpr auto maximum = static_cast<std::uintmax_t>(std::numeric_limits<Millis>::max());
    // Check before multiplication/addition, including the last partial second.
    if (seconds > (maximum - fraction_ms) / 1000U) return -1;
    return static_cast<Millis>(seconds * 1000U + fraction_ms);
}

SystemPublishClock::SystemPublishClock(ReadClock read_clock) noexcept : read_clock_(read_clock) {}

Millis SystemPublishClock::read_millis(clockid_t id) noexcept {
    timespec sample{};
    // A missing reader or a failed call is an error for this read only; nothing
    // is cached, so the next read tries the clock again.
    if (read_clock_ == nullptr || read_clock_(id, &sample) != 0) return -1;
    return timespec_to_millis(sample);
}

// LINT-ALLOW(clock): the envelope ts field is wall-clock BY CONTRACT (11 section 3.0); mono is the monotonic one.
Millis SystemPublishClock::wall_millis() noexcept { return read_millis(CLOCK_REALTIME); }

// CLOCK_MONOTONIC never steps backwards and is the clock every r6 age, cadence
// interval and staleness check is computed on (CLK-C1).
Millis SystemPublishClock::mono_millis() noexcept { return read_millis(CLOCK_MONOTONIC); }

}  // namespace perception::rns
