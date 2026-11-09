/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_time_utils.cpp
 * Brief: Unit tests for message timestamp conversion helpers.
 *
 * Description:
 *   1. Verifies header stamps are used instead of fallback receive time.
 *   2. Verifies zero header stamps degrade to an explicit fallback.
 */

#include "common/time_utils.hpp"

#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_near(double actual, double expected, double epsilon, const char* message) {
    if (std::fabs(actual - expected) >= epsilon) {
        throw std::runtime_error(message);
    }
}

// A non-zero message stamp is used (seconds plus nanoseconds) and the fallback is not.
void test_header_stamp_wins_over_fallback() {
    bool used_fallback = true;
    const double stamp = perception::message_stamp_or_fallback(1780000000, 250000000U, 42.0, used_fallback);

    require(!used_fallback, "non-zero message stamp used fallback");
    require_near(stamp, 1780000000.25, 1e-9, "message stamp conversion mismatch");
}

// A zero message stamp uses the fallback and says so.
void test_zero_stamp_uses_fallback() {
    bool used_fallback = false;
    const double stamp = perception::message_stamp_or_fallback(0, 0U, 99.5, used_fallback);

    require(used_fallback, "zero message stamp did not report fallback");
    require_near(stamp, 99.5, 1e-9, "zero message stamp fallback mismatch");
}

// The monotonic clock is positive and non-decreasing, and system time converts exactly to
// nanoseconds.
void test_host_clock_helpers() {
    const std::int64_t first = perception::monotonic_now_ns();
    const std::int64_t second = perception::monotonic_now_ns();
    require(first > 0, "monotonic clock returned a non-positive timestamp");
    require(second >= first, "monotonic clock regressed");

    const auto epoch = std::chrono::system_clock::now();
    const auto expected_epoch_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            epoch.time_since_epoch())
            .count();
    require(
        perception::system_time_ns(epoch) == expected_epoch_ns,
        "system clock nanosecond conversion mismatch");
}

// Only "1" enables the inference profile; endpoint lists must be non-empty, positive and non-
// decreasing.
void test_profile_flag_and_endpoint_validation() {
    require(!perception::inference_profile_enabled(nullptr), "unset profile enabled");
    require(!perception::inference_profile_enabled(""), "empty profile enabled");
    require(!perception::inference_profile_enabled("0"), "zero profile enabled");
    require(!perception::inference_profile_enabled("false"), "false profile enabled");
    require(!perception::inference_profile_enabled("typo"), "typo profile enabled");
    require(perception::inference_profile_enabled("1"), "explicit profile disabled");

    require(
        perception::are_positive_nondecreasing({1, 1, 2, 3}),
        "valid monotonic endpoints were rejected");
    require(
        !perception::are_positive_nondecreasing({}),
        "empty endpoint sequence was accepted");
    require(
        !perception::are_positive_nondecreasing({1, 0, 2}),
        "zero endpoint was accepted");
    require(
        !perception::are_positive_nondecreasing({1, 3, 2}),
        "regressing endpoint was accepted");
}

}  // namespace

int main() {
    test_header_stamp_wins_over_fallback();
    test_zero_stamp_uses_fallback();
    test_host_clock_helpers();
    test_profile_flag_and_endpoint_validation();

    std::cout << "test_time_utils passed" << std::endl;
    return 0;
}
