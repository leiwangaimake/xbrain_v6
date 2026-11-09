/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: capture_clock.hpp
 * Brief: Capture time of one frameset from the camera's own frame timestamp (19 section 2.4, PD-11).
 *
 * Description:
 * What problem this solves. t_capture is the middle of the depth exposure on CLOCK_MONOTONIC and
 * every age downstream is computed from it. "Pickup time minus a constant" cannot see a frameset
 * that waited before it was picked up: measured on 2026-10-13/14, framesets 2..19 after every
 * start are 350..50 ms old when taken, and on two cores framesets queue for up to 0.34 s, all
 * stamped as fresh by a constant. This component stamps each frameset from its camera timestamp
 * instead. It is pure arithmetic on numbers the capture thread hands in: no SDK, no clock is
 * read here, nothing is allocated, one thread.
 *
 * Two candidates per frameset, both on CLOCK_MONOTONIC_RAW (the SDK's time domain with clock
 * type MONOTONIC; the host's clock discipline does not reach it):
 *   G  the SDK's global timestamp minus half the exposure: the SDK's mapping of the camera
 *      clock to the host clock. Usable when present, moving with the camera timestamp, not in
 *      the future and not older than max_latency_ns.
 *   F  camera timestamp + lower envelope of (pickup - camera timestamp) - min_path - half the
 *      exposure. The envelope drops at once and rises at most envelope_rise_ppm; min_path is the
 *      calibrated shortest "camera timestamp -> pickup". F is a candidate only once it is
 *      ANCHORED: in the current device stream it agreed with a usable G within anchor_agree_us
 *      for anchor_hold_ns without a break. Having seen some framesets anchors nothing: a stream
 *      that starts backlogged has no fresh frameset to anchor the envelope on.
 * The stamp is the OLDER of the usable candidates minus guard_us. No agreement threshold decides
 * between them. Neither usable: the frameset gets no capture time (NoTrustedTime) and the caller
 * does not publish an observation for it; no constant stands in.
 *   While G is not usable, F stands alone ("estimated") and is moved back by what its envelope
 *   may have risen since it last touched a frameset: (envelope_rise_ppm + holdover_drift_ppm)
 *   times that time. Otherwise a queue that never empties lets F creep newer.
 *
 * Device stream. A camera timestamp earlier than the last one by more than device_back_us, or
 * ahead by more than device_jump_us while the host moved less than device_jump_host_ns, starts
 * a new device stream: envelope, anchor and G history are dropped. A timestamp equal to the last
 * one (or earlier within device_back_us) is the same frame again: Duplicate, no state changes.
 * The host epoch and the last published time are never reset.
 *
 * Settling. For stream_settle_ns after the first frameset of a device stream nothing is stamped
 * (Settling); envelope, G history and anchor are built meanwhile. Reason, measured on 16 starts
 * (2026-10-14, evidence/reconnect_20261014): the SDK asks the camera for its time four times
 * within 150 ms when the device is created and next 0.23 s after the first frameset; until then
 * G is that 150 ms fit carried 0.8..1 s ahead and was off by -5.1..+3.0 ms, beyond the guard on
 * the new side in 4 of the 16. In the product a device stream starts only when the service
 * starts (a link loss ends the SDK's device and the capture loop with it).
 *
 * To CLOCK_MONOTONIC. The caller reads MONOTONIC, RAW, MONOTONIC around the pickup and passes
 * RAW, one MONOTONIC and the width of the bracket; wider than max_bracket_ns is refused
 * (BracketTooWide: read again). capture = pickup_mono - (pickup_raw - stamp_raw): only the age
 * of the frameset, tens to hundreds of ms, crosses between the two clocks.
 * Published value. Whole milliseconds, rounded down; a frameset whose millisecond is not later
 * than the last published one is NotLater and is not published, never moved forward.
 *
 * Watchdogs. Reported in health(), about once a second, never used for the stamp: the median
 * of G - F over fast_window_ns beyond fast_limit_us; the median over slow_window_ns away from
 * the first such median of the stream by more than slow_shift_limit_us, or beyond
 * slow_abs_limit_us; no comparable frameset for incomparable_limit_ns. They run on host time,
 * not on frame counts.
 *
 * What it cannot see. Everything is relative to the camera timestamp: which end of the exposure
 * it marks, and a delay in front of both candidates that grows slowly, move both together.
 * guard_us and the watchdog limits are values that held on the recordings of 2026-10-13/14, not
 * bounds.
 * What makes it worse than it could be, on the old side. The older candidate wins also when it
 * is the wrong one: a min_path calibrated too long, or a camera clock slower than the host by
 * more than envelope_rise_ppm (the envelope then cannot follow; measured: 29..35 ppm FAST), make
 * F too old and the stamp with it. The watchdogs report both.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace perception {

struct CaptureClockOptions {
    std::int64_t min_path_us = -1;                       // required: > 0
    std::int64_t guard_us = 2000;
    double envelope_rise_ppm = 50.0;
    double holdover_drift_ppm = 50.0;
    std::int64_t global_step_tolerance_us = 3000;
    std::int64_t anchor_agree_us = 4000;
    std::int64_t anchor_hold_ns = INT64_C(1000000000);
    std::int64_t device_back_us = 1000;
    std::int64_t device_jump_us = 1000000;
    std::int64_t device_jump_host_ns = INT64_C(500000000);
    // No stamp this long after a device stream's first frameset. It is an empirical isolation (16
    // warm starts, 2026-10-14), not a proof that the SDK's mapping is right afterwards (GPT
    // question 16.3 has a delayed-correction counterexample). It equals anchor_hold_ns on purpose:
    // the envelope cannot be anchored before then anyway; change the two together.
    std::int64_t stream_settle_ns = INT64_C(1000000000);
    std::int64_t max_bracket_ns = 200000;
    std::int64_t max_half_exposure_us = 100000;
    std::int64_t max_latency_ns = INT64_C(5000000000);
    std::int64_t fast_window_ns = INT64_C(2000000000);
    std::int64_t fast_limit_us = 3000;
    std::int64_t slow_window_ns = INT64_C(60000000000);
    std::int64_t slow_shift_limit_us = 1500;
    std::int64_t slow_abs_limit_us = 2500;
    std::int64_t incomparable_limit_ns = INT64_C(2000000000);
};

inline bool valid_capture_clock_options(const CaptureClockOptions& o) noexcept {
    return o.min_path_us > 0 && o.guard_us >= 0 && o.envelope_rise_ppm >= 0 && o.holdover_drift_ppm >= 0 &&
           o.global_step_tolerance_us > 0 && o.anchor_agree_us > 0 && o.anchor_hold_ns > 0 && o.device_back_us >= 0 &&
           o.device_jump_us > 0 && o.device_jump_host_ns > 0 && o.stream_settle_ns >= 0 && o.max_bracket_ns > 0 && o.max_half_exposure_us > 0 &&
           o.max_latency_ns > 0 && o.fast_window_ns > 0 && o.fast_limit_us > 0 && o.slow_window_ns > o.fast_window_ns &&
           o.slow_shift_limit_us > 0 && o.slow_abs_limit_us > 0 && o.incomparable_limit_ns > 0;
}

// What the capture thread knows when it has taken one frameset.
struct CaptureClockSample {
    std::int64_t pickup_raw_ns = -1;    // CLOCK_MONOTONIC_RAW right after the frameset was taken
    std::int64_t pickup_mono_ns = -1;   // CLOCK_MONOTONIC read with it
    std::int64_t bracket_ns = -1;       // width of the MONOTONIC / RAW / MONOTONIC bracket
    std::uint64_t device_us = 0;        // depth frame timestamp, camera clock; 0 = none
    std::int64_t half_exposure_us = -1; // frame timestamp - sensor timestamp (metadata); < 0 = none
    std::optional<std::uint64_t> global_us;  // SDK global timestamp of the depth frame, RAW clock
};

// Only Stamped carries a capture time; for every other status the caller publishes nothing for the frameset.
enum class CaptureStampStatus : std::uint8_t { Stamped, NoTrustedTime, Duplicate, NotLater, BracketTooWide, InvalidInput, Settling };
enum class CaptureStampSource : std::uint8_t { None, Global, Envelope };

struct CaptureStampResult {
    CaptureStampStatus status = CaptureStampStatus::InvalidInput;
    CaptureStampSource source = CaptureStampSource::None;
    bool estimated = false;             // stamped by F while G was not usable
    std::int64_t capture_mono_ns = -1;  // Stamped only: mid-exposure on CLOCK_MONOTONIC, guard taken off
    std::int64_t capture_mono_ms = -1;  // Stamped only: the published value
    std::int64_t latency_ns = -1;       // Stamped only: pickup - capture
    std::uint32_t device_stream = 0;    // counts the device streams seen, from 1
    bool global_usable = false;
    bool envelope_anchored = false;
    bool comparable = false;            // G usable on this frameset: global_minus_envelope_us is set
    std::int64_t global_minus_envelope_us = 0;
    std::int64_t arrival_lag_us = 0;    // pickup - (camera timestamp + envelope): how late this frameset came
};

struct CaptureClockHealth {
    bool fast_alarm = false, slow_alarm = false, incomparable = false, slow_locked = false;
    std::int64_t fast_median_us = 0, slow_median_us = 0, slow_locked_us = 0;
    std::uint64_t stamped_global = 0, stamped_envelope = 0, no_trusted_time = 0, duplicates = 0, not_later = 0,
        bracket_too_wide = 0, invalid_input = 0, device_streams = 0, anchors = 0, settling = 0;
};

class CaptureClock {
public:
    explicit CaptureClock(const CaptureClockOptions& options) noexcept
        : options_(options), valid_(valid_capture_clock_options(options)) {}
    bool valid() const noexcept { return valid_; }
    const CaptureClockHealth& health() const noexcept { return health_; }
    // True while pickup_raw_ns lies in the first stream_settle_ns of the current device stream, by
    // host time, whatever this frameset's own status is: the capture loop's 5 s deadline exempts that
    // window as a time interval (GPT 16 supplement: a BracketTooWide or InvalidInput frameset inside
    // the first second must not start the deadline early). Before any stream has started there is
    // no window, so an early invalid input still starts the deadline at once.
    bool in_stream_settle(std::int64_t pickup_raw_ns) const noexcept {
        return stream_start_raw_ns_ >= 0 && pickup_raw_ns >= stream_start_raw_ns_ &&
               pickup_raw_ns - stream_start_raw_ns_ < options_.stream_settle_ns;
    }

    CaptureStampResult step(const CaptureClockSample& s) noexcept {
        CaptureStampResult r;
        if (!valid_ || s.pickup_raw_ns < 0 || s.pickup_mono_ns < 0 || s.bracket_ns < 0 || s.device_us == 0 ||
            s.device_us > static_cast<std::uint64_t>(INT64_MAX / 1000) || s.half_exposure_us < 0 ||
            s.half_exposure_us > options_.max_half_exposure_us || (seen_ && s.pickup_raw_ns <= last_pickup_raw_ns_)) {
            ++health_.invalid_input;
            return r;
        }
        r.device_stream = stream_;
        if (s.bracket_ns > options_.max_bracket_ns) {
            ++health_.bracket_too_wide;
            r.status = CaptureStampStatus::BracketTooWide;
            return r;
        }
        const auto device_us = static_cast<std::int64_t>(s.device_us);
        if (seen_) {
            const std::int64_t step_us = device_us - last_device_us_;
            if (step_us <= 0 && step_us >= -options_.device_back_us) {
                ++health_.duplicates;
                r.status = CaptureStampStatus::Duplicate;
                return r;
            }
            if (step_us < 0 || (step_us > options_.device_jump_us &&
                                s.pickup_raw_ns - last_pickup_raw_ns_ < options_.device_jump_host_ns))
                new_stream();
        } else {
            new_stream();
        }
        r.device_stream = stream_;
        if (stream_start_raw_ns_ < 0) stream_start_raw_ns_ = s.pickup_raw_ns;

        // F: lower envelope of pickup - camera timestamp, in ns on the RAW clock.
        const double raw_ns = static_cast<double>(s.pickup_raw_ns) - static_cast<double>(device_us) * 1000.0;
        if (!envelope_set_) {
            envelope_ns_ = raw_ns;
            envelope_touch_raw_ns_ = s.pickup_raw_ns;
            envelope_set_ = true;
        } else {
            const double allowed = envelope_ns_ + options_.envelope_rise_ppm * 1e-6 *
                                                      static_cast<double>(s.pickup_raw_ns - last_pickup_raw_ns_);
            if (raw_ns <= allowed) {
                envelope_ns_ = raw_ns;
                envelope_touch_raw_ns_ = s.pickup_raw_ns;
            } else {
                envelope_ns_ = allowed;
            }
        }
        const std::int64_t half_ns = s.half_exposure_us * 1000;
        const std::int64_t arrival_lag_ns = static_cast<std::int64_t>(raw_ns - envelope_ns_);
        r.arrival_lag_us = arrival_lag_ns / 1000;
        // F's mid-exposure on RAW: the pickup moved back by the lag, the shortest path and half the exposure.
        const std::int64_t envelope_mid_ns = s.pickup_raw_ns - arrival_lag_ns - options_.min_path_us * 1000 - half_ns;

        // G: present, moving with the camera timestamp, neither in the future nor absurdly old.
        std::int64_t global_mid_ns = 0;
        bool global_usable = false;
        if (s.global_us && *s.global_us != 0 && *s.global_us <= static_cast<std::uint64_t>(INT64_MAX / 1000)) {
            const auto global_us = static_cast<std::int64_t>(*s.global_us);
            global_mid_ns = global_us * 1000 - half_ns;
            global_usable = global_mid_ns <= s.pickup_raw_ns && s.pickup_raw_ns - global_mid_ns <= options_.max_latency_ns;
            if (global_usable && global_seen_) {
                const std::int64_t disagreement = (global_us - last_global_us_) - (device_us - last_global_device_us_);
                global_usable = disagreement <= options_.global_step_tolerance_us &&
                                disagreement >= -options_.global_step_tolerance_us;
            }
            last_global_us_ = global_us;
            last_global_device_us_ = device_us;
            global_seen_ = true;
        }
        r.global_usable = global_usable;

        // Anchor: F agrees with a usable G for anchor_hold_ns without a break, once per device stream.
        if (global_usable) {
            const std::int64_t difference_us = (global_mid_ns - envelope_mid_ns) / 1000;
            r.comparable = true;
            r.global_minus_envelope_us = difference_us;
            last_comparable_raw_ns_ = s.pickup_raw_ns;
            if (difference_us <= options_.anchor_agree_us && difference_us >= -options_.anchor_agree_us) {
                if (agree_since_raw_ns_ < 0) agree_since_raw_ns_ = s.pickup_raw_ns;
                if (!anchored_ && s.pickup_raw_ns - agree_since_raw_ns_ >= options_.anchor_hold_ns) {
                    anchored_ = true;
                    ++health_.anchors;
                }
            } else {
                agree_since_raw_ns_ = -1;
            }
            watch_push(s.pickup_raw_ns, difference_us);
        } else {
            agree_since_raw_ns_ = -1;
        }
        r.envelope_anchored = anchored_;
        watch_evaluate(s.pickup_raw_ns);
        last_device_us_ = device_us;
        last_pickup_raw_ns_ = s.pickup_raw_ns;
        seen_ = true;

        // A young device stream: everything above was updated, but no stamp yet (see "Settling").
        if (s.pickup_raw_ns - stream_start_raw_ns_ < options_.stream_settle_ns) {
            ++health_.settling;
            r.status = CaptureStampStatus::Settling;
            return r;
        }

        // The older of the usable candidates.
        std::int64_t stamp_raw_ns = 0;
        if (global_usable) {
            stamp_raw_ns = global_mid_ns;
            r.source = CaptureStampSource::Global;
            if (anchored_ && envelope_mid_ns < global_mid_ns) {
                stamp_raw_ns = envelope_mid_ns;
                r.source = CaptureStampSource::Envelope;
            }
        } else if (anchored_) {
            // F alone: what the envelope may have risen since it last touched a frameset comes off.
            const double may_have_risen_ns = (options_.envelope_rise_ppm + options_.holdover_drift_ppm) * 1e-6 *
                                             static_cast<double>(s.pickup_raw_ns - envelope_touch_raw_ns_);
            stamp_raw_ns = envelope_mid_ns - static_cast<std::int64_t>(may_have_risen_ns);
            r.source = CaptureStampSource::Envelope;
            r.estimated = true;
        } else {
            ++health_.no_trusted_time;
            r.status = CaptureStampStatus::NoTrustedTime;
            return r;
        }
        stamp_raw_ns -= options_.guard_us * 1000;
        const std::int64_t latency_ns = s.pickup_raw_ns - stamp_raw_ns;
        const std::int64_t capture_mono_ns = s.pickup_mono_ns - latency_ns;
        if (latency_ns < 0 || capture_mono_ns < 0) {
            ++health_.invalid_input;
            r.source = CaptureStampSource::None;
            r.estimated = false;
            return r;
        }
        const std::int64_t capture_mono_ms = capture_mono_ns / 1000000;
        if (published_ && capture_mono_ms <= last_published_ms_) {
            ++health_.not_later;
            r.status = CaptureStampStatus::NotLater;
            r.source = CaptureStampSource::None;
            r.estimated = false;
            return r;
        }
        published_ = true;
        last_published_ms_ = capture_mono_ms;
        r.status = CaptureStampStatus::Stamped;
        r.capture_mono_ns = capture_mono_ns;
        r.capture_mono_ms = capture_mono_ms;
        r.latency_ns = latency_ns;
        ++(r.source == CaptureStampSource::Global ? health_.stamped_global : health_.stamped_envelope);
        return r;
    }

private:
    static constexpr std::size_t kWatchCapacity = 4096;  // 60 s at up to 68 framesets per second

    void new_stream() noexcept {
        ++stream_;
        ++health_.device_streams;
        envelope_set_ = false;
        anchored_ = false;
        global_seen_ = false;
        agree_since_raw_ns_ = -1;
        last_comparable_raw_ns_ = -1;
        stream_start_raw_ns_ = -1;
        watch_count_ = 0;
        watch_next_ = 0;
        watch_evaluated_raw_ns_ = -1;
        health_.fast_alarm = health_.slow_alarm = health_.incomparable = health_.slow_locked = false;
        health_.fast_median_us = health_.slow_median_us = health_.slow_locked_us = 0;
    }
    void watch_push(std::int64_t raw_ns, std::int64_t difference_us) noexcept {
        watch_time_[watch_next_] = raw_ns;
        watch_value_[watch_next_] = static_cast<std::int32_t>(std::clamp<std::int64_t>(difference_us, -2000000000, 2000000000));
        watch_next_ = (watch_next_ + 1) % kWatchCapacity;
        watch_count_ = std::min(watch_count_ + 1, kWatchCapacity);
    }
    // Median of the differences newer than `since`; false when there are fewer than `least`.
    bool watch_median(std::int64_t since_raw_ns, std::size_t least, std::int64_t& median) noexcept {
        std::size_t n = 0;
        for (std::size_t k = 0; k < watch_count_; ++k) {
            const std::size_t i = (watch_next_ + kWatchCapacity - 1 - k) % kWatchCapacity;
            if (watch_time_[i] < since_raw_ns) break;
            scratch_[n++] = watch_value_[i];
        }
        if (n < least) return false;
        std::nth_element(scratch_.begin(), scratch_.begin() + static_cast<std::ptrdiff_t>(n / 2),
                         scratch_.begin() + static_cast<std::ptrdiff_t>(n));
        median = scratch_[n / 2];
        return true;
    }
    void watch_evaluate(std::int64_t now_raw_ns) noexcept {
        if (stream_start_raw_ns_ < 0) stream_start_raw_ns_ = now_raw_ns;
        if (watch_evaluated_raw_ns_ >= 0 && now_raw_ns - watch_evaluated_raw_ns_ < INT64_C(1000000000)) return;
        watch_evaluated_raw_ns_ = now_raw_ns;
        const std::int64_t reference = last_comparable_raw_ns_ >= 0 ? last_comparable_raw_ns_ : stream_start_raw_ns_;
        health_.incomparable = now_raw_ns - reference > options_.incomparable_limit_ns;
        std::int64_t median = 0;
        if (watch_median(now_raw_ns - options_.fast_window_ns, 3, median)) {
            health_.fast_median_us = median;
            health_.fast_alarm = median > options_.fast_limit_us || median < -options_.fast_limit_us;
        } else {
            health_.fast_alarm = false;
        }
        // The slow median needs a window that is really full: comparable framesets as old as the window.
        const bool window_full = watch_count_ > 0 &&
            watch_time_[(watch_next_ + kWatchCapacity - watch_count_) % kWatchCapacity] <= now_raw_ns - options_.slow_window_ns * 9 / 10;
        if (anchored_ && window_full && watch_median(now_raw_ns - options_.slow_window_ns, 30, median)) {
            health_.slow_median_us = median;
            if (!health_.slow_locked) {
                health_.slow_locked = true;
                health_.slow_locked_us = median;
            }
            const std::int64_t shift = median - health_.slow_locked_us;
            health_.slow_alarm = shift > options_.slow_shift_limit_us || shift < -options_.slow_shift_limit_us ||
                                 median > options_.slow_abs_limit_us || median < -options_.slow_abs_limit_us;
        }
    }

    CaptureClockOptions options_;
    bool valid_ = false;
    CaptureClockHealth health_{};
    bool seen_ = false, envelope_set_ = false, anchored_ = false, global_seen_ = false, published_ = false;
    std::uint32_t stream_ = 0;
    double envelope_ns_ = 0;
    std::int64_t envelope_touch_raw_ns_ = -1, last_pickup_raw_ns_ = -1, last_device_us_ = 0, last_global_us_ = 0,
        last_global_device_us_ = 0, agree_since_raw_ns_ = -1, last_comparable_raw_ns_ = -1, stream_start_raw_ns_ = -1,
        last_published_ms_ = -1, watch_evaluated_raw_ns_ = -1;
    std::array<std::int64_t, kWatchCapacity> watch_time_{};
    std::array<std::int32_t, kWatchCapacity> watch_value_{};
    std::array<std::int32_t, kWatchCapacity> scratch_{};
    std::size_t watch_count_ = 0, watch_next_ = 0;
};

}  // namespace perception
