/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: time_utils.hpp
 * Brief: Timestamp conversion helpers shared by ROS2 wrappers and tests.
 *
 * Description:
 *   1. Converts sec/nanosec message stamps to seconds.
 *   2. Selects explicit fallback timestamps only for zero message stamps.
 * R6 uses: monotonic_now_ns() for every timing stamp, the PERCEPTION_INFERENCE_PROFILE switch
 * (production "1"), are_positive_nondecreasing() for the timing ledger, system_time_ns() for
 * the one wall-clock sample, and stamp_to_sec() in the ROS bridge. is_zero_stamp() and
 * message_stamp_or_fallback() have no R6 reader (V5 ROS wrapper).
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <string_view>
#include <time.h>

namespace perception {

/** @brief Returns the current host monotonic clock in integer nanoseconds. */
inline std::int64_t monotonic_now_ns() {
    timespec value{};
    if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        return 0;
    }
    return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL +
           static_cast<std::int64_t>(value.tv_nsec);
}

/** @brief Returns CLOCK_MONOTONIC_RAW in integer nanoseconds (0 on failure): the host clock the
 *  camera SDK stamps on with clock type MONOTONIC; the host's clock discipline does not reach it. */
inline std::int64_t monotonic_raw_now_ns() {
    timespec value{};
    if (::clock_gettime(CLOCK_MONOTONIC_RAW, &value) != 0) {
        return 0;
    }
    return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL +
           static_cast<std::int64_t>(value.tv_nsec);
}

/** @brief Converts an existing system-clock sample to integer epoch nanoseconds. */
// LINT-ALLOW(clock): a conversion, not a clock read; its one caller samples at a marked site.
inline std::int64_t system_time_ns(const std::chrono::system_clock::time_point& value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               value.time_since_epoch())
        .count();
}

/** @brief Enables the latency ledger only for the exact opt-in value "1". */
inline bool inference_profile_enabled(const char* value) noexcept {
    return value != nullptr && std::string_view(value) == "1";
}

inline bool inference_profile_enabled() noexcept {
    return inference_profile_enabled(std::getenv("PERCEPTION_INFERENCE_PROFILE"));
}

/** @brief Validates a non-empty sequence of positive monotonic endpoints. */
inline bool are_positive_nondecreasing(
    std::initializer_list<std::int64_t> endpoints) noexcept {
    if (endpoints.size() == 0U) {
        return false;
    }
    std::int64_t previous = 0;
    for (const std::int64_t endpoint : endpoints) {
        if (endpoint <= 0 || endpoint < previous) {
            return false;
        }
        previous = endpoint;
    }
    return true;
}

inline double stamp_to_sec(std::int32_t sec, std::uint32_t nanosec) {
    return static_cast<double>(sec) + (static_cast<double>(nanosec) / 1000000000.0);
}

inline bool is_zero_stamp(std::int32_t sec, std::uint32_t nanosec) {
    return sec == 0 && nanosec == 0U;
}

inline double message_stamp_or_fallback(
    std::int32_t sec,
    std::uint32_t nanosec,
    double fallback_stamp_sec,
    bool& used_fallback) {
    used_fallback = is_zero_stamp(sec, nanosec);
    if (used_fallback) {
        return fallback_stamp_sec;
    }
    return stamp_to_sec(sec, nanosec);
}

}  // namespace perception
