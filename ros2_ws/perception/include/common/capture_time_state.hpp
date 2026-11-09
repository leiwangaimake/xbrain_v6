/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: capture_time_state.hpp
 * Brief: What the capture thread did with each frameset's capture time, for Status and the logs.
 *
 * Description:
 * The capture thread stamps every frameset with the capture clock (capture_clock.hpp) and hands
 * on only stamped ones. Three things live here so that they can be tested without a camera:
 *   - the runtime switch capture_min_path_us (PERCEPTION_CAPTURE_MIN_PATH_US): the calibrated
 *     shortest "camera frame timestamp -> pickup" of this camera, link and SDK in microseconds,
 *     the one calibration the capture clock needs. It is a measurement of the installation
 *     (30767 on the Gemini 338Le over Ethernet, SDK 2.9.3, 2026-10-13), not the contract's
 *     perception.capture.latency_fallback_ms, which no longer takes part in any timestamp;
 *   - process-wide counters the capture thread adds to and the Status thread reads;
 *   - the two Status reasons made from them. A reason is up while the capture thread's latest
 *     verdict says so, and for at least kCaptureTimeReasonHoldMs after its last occurrence. It
 *     clears only when a LATER frameset was stamped (for t_capture_estimated: stamped in the
 *     measured grade), never just because time passed: frames that stop coming are not a
 *     recovery (GPT question 16.4, 2026-10-14). The Status thread looks every 10 ms and
 *     publishes once a second, so with regular polling and a heartbeat that goes out, a
 *     single withheld frameset is on at least one published Status, and a run of them is one
 *     episode, not an edge per frameset:
 *       t_capture_estimated        (fixed literal, 11 section 3.1B.3) framesets were stamped
 *                                  from the host-side envelope alone, the SDK's mapping of the
 *                                  camera clock was not usable;
 *       capture_time_unavailable   (free diagnostic text) framesets got no capture time and
 *                                  were not handed on, so nothing was published for them. That
 *                                  pause is NOT a declared exclusion: it counts in infer_gap /
 *                                  infer_rate_low like any other pause (19 section 9: no silent
 *                                  stop; user decision 2026-10-14);
 *   - the run of framesets without a capture time as the capture thread sees it, and its limit
 *     (user decisions 2026-10-14, after GPT question 16.3 and Fable question 11.2): once the
 *     first second of a device stream (Settling) is over, framesets that keep coming without a
 *     capture time for kCaptureTimeWithheldLimitNs (5 s, like five consecutive 1 s frame
 *     timeouts) end the capture loop like any other capture failure, and the service stops; a
 *     restart sets the SDK's time mapping up again. The limit is a deadline on host time: it is
 *     checked on every withheld frameset, on every frame wait that times out, and on the
 *     stamped frameset that would end the run -- one that comes after the deadline does not
 *     clear it. Settling framesets open the run (for the log lines) but do not start the
 *     deadline. The host time here is CLOCK_MONOTONIC (the pickup sample); the capture clock
 *     itself works on CLOCK_MONOTONIC_RAW.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace perception {

inline constexpr std::int64_t kCaptureMinPathUsLow = 10000, kCaptureMinPathUsHigh = 100000;

// One spelling per value: digits only, no leading zero, 10000..100000 microseconds.
inline bool valid_capture_min_path_text(const std::string& text) noexcept {
    if (text.size() < 5 || text.size() > 6 || text[0] == '0') return false;
    for (const char c : text)
        if (c < '0' || c > '9') return false;
    const long value = std::strtol(text.c_str(), nullptr, 10);
    return value >= kCaptureMinPathUsLow && value <= kCaptureMinPathUsHigh;
}

// The switch as the capture thread reads it. No default: a missing or malformed value throws.
inline std::int64_t parse_capture_min_path_us(const char* text) {
    if (text == nullptr || !valid_capture_min_path_text(text))
        throw std::invalid_argument("PERCEPTION_CAPTURE_MIN_PATH_US must be a whole number of microseconds, 10000..100000");
    return std::strtol(text, nullptr, 10);
}

// Written by the capture thread only; read by the Status thread. Relaxed: counts, and two
// verdicts about the latest frameset that have a single writer.
struct CaptureTimeCounters {
    std::atomic<std::uint64_t> stamped{0};       // framesets handed on with a capture time
    std::atomic<std::uint64_t> estimated{0};     // of those: from the envelope alone
    std::atomic<std::uint64_t> withheld{0};      // framesets without a capture time, not handed on
    std::atomic<std::uint64_t> settling{0};      // of those: in a device stream's first second
    std::atomic<bool> withheld_now{false};       // the latest frameset was withheld (false again only once one is stamped)
    std::atomic<bool> estimated_now{false};      // the latest stamped frameset was in the estimated grade
};
inline CaptureTimeCounters& capture_time_counters() noexcept {
    static CaptureTimeCounters counters;
    return counters;
}

// How long a reason stays up after its counter last moved: longer than the 1 s Status period.
inline constexpr std::int64_t kCaptureTimeReasonHoldMs = 1500;

// The Status side: which of the two reasons hold at now_ms (any monotonic millisecond clock;
// a time that goes back is treated as "no time has passed").
struct CaptureTimeReasons {
    bool estimated = false, unavailable = false;
};
class CaptureTimeReasonWatch {
public:
    CaptureTimeReasons poll(const CaptureTimeCounters& counters, std::int64_t now_ms) noexcept {
        const std::uint64_t estimated = counters.estimated.load(std::memory_order_relaxed);
        const std::uint64_t withheld = counters.withheld.load(std::memory_order_relaxed);
        if (estimated != seen_estimated_) { seen_estimated_ = estimated; estimated_at_ms_ = now_ms; estimated_seen_ = true; }
        if (withheld != seen_withheld_) { seen_withheld_ = withheld; withheld_at_ms_ = now_ms; withheld_seen_ = true; }
        // Up while the latest verdict says so (a later stamped frameset is the only way down),
        // and for at least the hold after the last occurrence.
        return {counters.estimated_now.load(std::memory_order_relaxed) ||
                    (estimated_seen_ && now_ms - estimated_at_ms_ <= kCaptureTimeReasonHoldMs),
                counters.withheld_now.load(std::memory_order_relaxed) ||
                    (withheld_seen_ && now_ms - withheld_at_ms_ <= kCaptureTimeReasonHoldMs)};
    }

private:
    std::uint64_t seen_estimated_ = 0, seen_withheld_ = 0;
    std::int64_t estimated_at_ms_ = 0, withheld_at_ms_ = 0;
    bool estimated_seen_ = false, withheld_seen_ = false;
};

// Framesets in a row without a capture time, on host time.
inline constexpr std::int64_t kCaptureTimeWithheldLimitNs = INT64_C(5000000000);

class CaptureTimeWithheldRun {
public:
    // What one frameset did to the run.
    enum class Step : std::uint8_t {
        Stamped,    // stamped, no run was open
        Resumed,    // stamped, and it ends a run (log it)
        Started,    // withheld, and it opens a run (log it)
        Continues,  // withheld inside an open run
        TooLong     // the run's deadline has passed (withheld or stamped frameset alike): end the capture loop
    };
    // A frameset without a capture time; settling: in a device stream's first second.
    Step withheld(std::int64_t now_ns, bool settling) noexcept {
        Step step = Step::Continues;
        if (framesets_++ == 0) {
            since_ns_ = now_ns;
            step = Step::Started;
        }
        if (!settling && limit_from_ns_ < 0) limit_from_ns_ = now_ns;
        return overdue(now_ns) ? Step::TooLong : step;
    }
    // A stamped frameset. It ends an open run, unless the run's deadline has already passed.
    Step stamped(std::int64_t now_ns) noexcept {
        if (framesets_ == 0) return Step::Stamped;
        if (overdue(now_ns)) return Step::TooLong;
        ended_framesets_ = framesets_;
        framesets_ = 0;
        limit_from_ns_ = -1;
        return Step::Resumed;
    }
    // Also asked on every frame wait that times out: the deadline runs on host time, frames or not.
    bool overdue(std::int64_t now_ns) const noexcept {
        return framesets_ != 0 && limit_from_ns_ >= 0 && now_ns - limit_from_ns_ >= kCaptureTimeWithheldLimitNs;
    }
    std::uint64_t framesets() const noexcept { return framesets_; }               // the open run
    std::uint64_t ended_framesets() const noexcept { return ended_framesets_; }   // the run Resumed ended
    std::int64_t since_ns() const noexcept { return since_ns_; }                  // first frameset of the (last) run
    std::int64_t limit_from_ns() const noexcept { return limit_from_ns_; }        // first non-settling one; -1 = not started

private:
    std::uint64_t framesets_ = 0, ended_framesets_ = 0;
    std::int64_t since_ns_ = 0, limit_from_ns_ = -1;
};

}  // namespace perception
