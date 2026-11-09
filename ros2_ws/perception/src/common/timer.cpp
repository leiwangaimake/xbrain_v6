/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: timer.cpp
 * Brief: Steady-clock timer implementation.
 *
 * Description:
 *   1. Tracks accumulated elapsed time across start/stop intervals.
 *   2. Reports elapsed time in milliseconds and microseconds.
 */

#include "common/timer.hpp"

namespace yolo {

Timer::Timer() : start_time_(), accumulated_(0), running_(false) {}

void Timer::start() {
    if (!running_) {
        start_time_ = std::chrono::steady_clock::now();
        running_ = true;
    }
}

void Timer::stop() {
    if (running_) {
        const auto now = std::chrono::steady_clock::now();
        accumulated_ += std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
        running_ = false;
    }
}

void Timer::reset() {
    accumulated_ = std::chrono::microseconds(0);
    running_ = false;
    start_time_ = std::chrono::steady_clock::time_point();
}

double Timer::elapsed_ms() const {
    return static_cast<double>(elapsed_us()) / 1000.0;
}

std::int64_t Timer::elapsed_us() const {
    auto total = accumulated_;
    if (running_) {
        const auto now = std::chrono::steady_clock::now();
        total += std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    }

    return total.count();
}

}  // namespace yolo
