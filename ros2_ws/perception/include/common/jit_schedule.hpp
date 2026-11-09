/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: jit_schedule.hpp
 * Brief: JIT loop helpers: bind-delay parsing, the delay calibration, the fresh-bind option and
 *        the minimum submit interval.
 *
 * Description:
 * Used by InferenceModule::infer_loop_jit (inference_module.cpp describes the schedule).
 *   parse_jit_bind_delay_ms   the fixed D of the production runtime set (jit_bind_delay_ms);
 *   JitScheduleCalibration    120 warm-up and 300 measured frames, then
 *                             D = max(0, pair median - prepare p99 - 2 ms);
 *   jit_fresh_bind_*          an option that waits briefly for a newer frame; off in the
 *                             production set (jit_fresh_bind: "0");
 *   pace_wait_ns / jit_pace_wait
 *                             the minimum interval between two device submissions
 *                             (infer_min_interval_ms; "0" = off, submits back to back as before).
 */

#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace perception {

inline constexpr std::int64_t JIT_FRESH_BIND_BUDGET_NS = 2000000;
inline constexpr std::int64_t JIT_FRESH_BIND_MIN_QUEUE_AGE_NS = 25000000;

inline bool parse_jit_fresh_bind_enabled(const char* value) {
    if (value == nullptr || std::string(value) == "0") return false;
    if (std::string(value) == "1") return true;
    throw std::invalid_argument("PERCEPTION_JIT_FRESH_BIND must be exactly 0 or 1");
}

inline std::int64_t jit_fresh_bind_deadline_ns(std::int64_t bind_target_ns) {
    if (bind_target_ns <= 0 ||
        bind_target_ns > std::numeric_limits<std::int64_t>::max() - JIT_FRESH_BIND_BUDGET_NS) {
        throw std::invalid_argument("invalid fresh-bind monotonic target");
    }
    return bind_target_ns + JIT_FRESH_BIND_BUDGET_NS;
}

inline bool jit_fresh_bind_should_wait(
    std::int64_t now_ns, std::int64_t deadline_ns, std::int64_t push_begin_ns) {
    if (now_ns <= 0 || deadline_ns <= 0 || push_begin_ns <= 0 || push_begin_ns > now_ns) {
        throw std::invalid_argument("invalid fresh-bind monotonic endpoints");
    }
    return now_ns < deadline_ns &&
        now_ns - push_begin_ns >= JIT_FRESH_BIND_MIN_QUEUE_AGE_NS;
}

// Reject whitespace, suffixes, hex, NaN/Inf and range errors rather than
// silently truncating a malformed diagnostic environment override.
inline double parse_jit_bind_delay_ms(const std::string& text, double timeout_ms) {
    if (text.empty() || !std::isfinite(timeout_ms) || timeout_ms < 0.0 ||
        text.find_first_not_of("0123456789+-.eE") != std::string::npos) {
        throw std::invalid_argument("invalid fixed JIT bind delay");
    }
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (errno == ERANGE || end == text.c_str() ||
        end != text.c_str() + text.size() || !std::isfinite(value) ||
        value < 0.0 || value > timeout_ms) {
        throw std::invalid_argument("fixed JIT bind delay must be finite and within [0, timeout]");
    }
    return value;
}

// Minimum interval between two device submissions (runtime switch infer_min_interval_ms,
// PERCEPTION_INFER_MIN_INTERVAL_MS, 2026-10-20). Back to back the DLA pair runs at about 24.4 Hz;
// a cap above the pair time leaves DLA, GPU and CPU idle for the rest and so releases that time
// to the platform. The cost: once the cap binds, an Objects interval is the cap plus the change
// of the host time after ready (post, merge, Supervisor, release) from one frame to the next, so
// each ms of cap above the pair time removes one ms of host-jitter margin under the 50 ms line of
// 19 section 6.3. A value is a knob to be accepted per measurement window (the >50 ms share, the
// infer_gap / infer_rate_low counts, Objects age), never by this reasoning; the default is "0".
// 50 or more is refused: almost every interval would be at least 50 ms by construction.
// The cap only delays a submit; the frame already prepared is submitted as is. Age cost (needs
// jit_async_submit "1", see below): a frame ages by its own wait; on the overlap path the next
// submit, wait included, runs on the submit lane next to this frame's post and the push of this
// frame waits for both, so it also ages by max(0, next wait - post wall). The serial-next path
// (jit_async_submit "0") would submit, wait included, before publishing the current frame and
// hold every result back by the whole wait; the configuration and start() refuse it.
inline constexpr int JIT_MIN_INTERVAL_MAX_MS = 49;
// Longest single sleep of the wait; the stop request is checked between two.
inline constexpr std::int64_t JIT_PACE_SLICE_NS = 5000000;

// One spelling per value: "0" (off) or a whole number of milliseconds 1..49, no leading zero.
inline bool valid_infer_min_interval_text(const std::string& text) noexcept {
    if (text == "0") return true;
    if (text.empty() || text.size() > 2 || text[0] == '0') return false;
    for (const char c : text)
        if (c < '0' || c > '9') return false;
    const long value = std::strtol(text.c_str(), nullptr, 10);
    return value >= 1 && value <= JIT_MIN_INTERVAL_MAX_MS;
}

// The switch as start() reads it: absent is off (0); a malformed value throws.
inline int parse_infer_min_interval_ms(const char* text) {
    if (text == nullptr) return 0;
    if (!valid_infer_min_interval_text(text)) {
        throw std::invalid_argument("must be 0 or a whole number of milliseconds, 1..49");
    }
    return static_cast<int>(std::strtol(text, nullptr, 10));
}

// What is left to wait before the next submit so that it starts at least interval_ns after the
// previous one (last_submit_ns, of either slot). 0 with the cap off (interval_ns <= 0), before the
// first submit (last_submit_ns <= 0) and once the interval has passed (also exactly on time).
inline std::int64_t pace_wait_ns(std::int64_t last_submit_ns, std::int64_t now_ns, std::int64_t interval_ns) {
    if (interval_ns <= 0 || last_submit_ns <= 0) return 0;
    if (now_ns < last_submit_ns) {
        throw std::invalid_argument("invalid pace monotonic endpoints");
    }
    const std::int64_t elapsed_ns = now_ns - last_submit_ns;
    return elapsed_ns >= interval_ns ? 0 : interval_ns - elapsed_ns;
}

// The wait itself: sleeps in slices of at most JIT_PACE_SLICE_NS until pace_wait_ns() is 0 or
// keep_waiting() turns false (a stop), and returns how long it waited (0 without a wait). now()
// is the monotonic clock in ns, sleep(ns) one bounded sleep.
template <typename Now, typename Sleep, typename KeepWaiting>
std::int64_t jit_pace_wait(std::int64_t last_submit_ns, std::int64_t interval_ns,
                           Now&& now, Sleep&& sleep, KeepWaiting&& keep_waiting) {
    const std::int64_t begin_ns = now();
    std::int64_t observed_ns = begin_ns;
    std::int64_t remaining_ns = pace_wait_ns(last_submit_ns, begin_ns, interval_ns);
    while (remaining_ns > 0 && keep_waiting()) {
        sleep(std::min(remaining_ns, JIT_PACE_SLICE_NS));
        observed_ns = now();
        remaining_ns = pace_wait_ns(last_submit_ns, observed_ns, interval_ns);
    }
    return observed_ns - begin_ns;
}

// Fixed before the measuring phase of the JIT loop (production since the cand_R set; the name
// "probe" in the logs is historical). These are host spans, not GPU clock values or sums of
// independently running model durations.
class JitScheduleCalibration {
public:
    static constexpr std::size_t warmup_count = 120;
    static constexpr std::size_t sample_count = 300;

    void add(double prepare_and_clone_ms, double submit_to_ready_ms) {
        if (frozen_ || !std::isfinite(prepare_and_clone_ms) ||
            !std::isfinite(submit_to_ready_ms) || prepare_and_clone_ms < 0.0 ||
            submit_to_ready_ms < 0.0) {
            throw std::logic_error("invalid or late JIT calibration sample");
        }
        ++seen_;
        if (seen_ <= warmup_count) {
            return;
        }
        preparation_.push_back(prepare_and_clone_ms);
        pair_.push_back(submit_to_ready_ms);
        if (pair_.size() == sample_count) {
            pair_median_ = quantile(pair_, 0.50);
            prepare_p99_ = quantile(preparation_, 0.99);
            delay_ms_ = std::max(0.0, pair_median_ - prepare_p99_ - 2.0);
            frozen_ = true;
        }
    }

    bool frozen() const noexcept { return frozen_; }
    std::size_t seen() const noexcept { return seen_; }
    double delay_ms() const { require_frozen(); return delay_ms_; }
    double pair_median_ms() const { require_frozen(); return pair_median_; }
    double prepare_p99_ms() const { require_frozen(); return prepare_p99_; }

private:
    static double quantile(std::vector<double> values, double probability) {
        std::sort(values.begin(), values.end());
        const double index = probability * static_cast<double>(values.size() - 1U);
        const auto lo = static_cast<std::size_t>(std::floor(index));
        const auto hi = static_cast<std::size_t>(std::ceil(index));
        return values[lo] + (values[hi] - values[lo]) * (index - lo);
    }
    void require_frozen() const {
        if (!frozen_) {
            throw std::logic_error("JIT delay requested before fixed calibration");
        }
    }
    std::size_t seen_ = 0;
    bool frozen_ = false;
    double pair_median_ = 0.0;
    double prepare_p99_ = 0.0;
    double delay_ms_ = 0.0;
    std::vector<double> preparation_;
    std::vector<double> pair_;
};

}  // namespace perception
