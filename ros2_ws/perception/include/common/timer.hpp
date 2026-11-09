/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: timer.hpp
 * Brief: Steady-clock timer utility for profiling pipeline stages.
 *
 * Description:
 *   1. Measures elapsed wall-clock time in milliseconds and microseconds.
 *      (Elapsed real time on the monotonic steady clock, not the system wall clock.)
 *   2. Supports start, stop, and reset for scoped performance checks.
 */

#pragma once

#include <chrono>
#include <cstdint>

namespace yolo {

/**
 * @brief Small steady_clock based timer used by inference and diagnostics.
 */
class Timer {
public:
    /** @brief Creates a stopped timer with zero elapsed time. */
    Timer();

    /** @brief Starts or resumes timing. */
    void start();

    /** @brief Stops timing and preserves accumulated elapsed time. */
    void stop();

    /** @brief Clears elapsed time and returns the timer to stopped state. */
    void reset();

    /** @brief Returns elapsed time in milliseconds. */
    double elapsed_ms() const;

    /** @brief Returns elapsed time in microseconds. */
    std::int64_t elapsed_us() const;

private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::microseconds accumulated_;
    bool running_;
};

}  // namespace yolo
