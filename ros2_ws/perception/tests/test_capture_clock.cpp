/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_capture_clock.cpp
 * Brief: The capture clock on synthetic camera streams whose true exposure times are known.
 *
 * Description:
 * How. A small world model produces framesets with a known mid-exposure on the host RAW clock:
 * a camera clock that drifts against the host, a shortest delivery path with jitter, a consumer
 * that may be busy (so framesets queue and the pickup times stay monotonic, as on a real host),
 * an SDK global timestamp with a controllable error, and CLOCK_MONOTONIC running at its own
 * rate. Every case states what the stamp must be against that truth. "Error" below is the
 * stamp minus the true mid-exposure on CLOCK_MONOTONIC; negative is older. With the 2 ms guard
 * a correct stamp sits near -2.0 ms (taken from the envelope) or -1.5 ms (from a global
 * timestamp that is 0.5 ms new, the default here).
 *
 * The cases follow the three regression groups agreed after the second adviser round
 * (2026-10-14):
 *   source validity    steady state; backlog after the start; no global timestamp (nothing is
 *                      stamped, no constant); a stream that starts backlogged; a global timestamp
 *                      that is lost, lost while framesets queue, or drifts; a camera clock that
 *                      restarts; the same frame delivered twice.
 *   selection, alarms  a global timestamp that turns 3.5 ms new is never followed; the alarms
 *                      run on host time, not on frame counts; below the limit there is no alarm;
 *                      a slow drift is reported by the shift rule, a constant offset by the
 *                      absolute one.
 *   final time domain  the bracket of clock readings, invalid inputs, whole milliseconds that
 *                      must increase, the last published time across a camera restart, the two
 *                      host clocks at different rates, no allocation.
 * One case records a limit instead of a guarantee: a delay in front of both candidates that
 * grows slowly moves both, and nothing here notices (test_known_limit_common_delay).
 *
 * Settling (added with the wiring, 2026-10-14). The cases above run with stream_settle_ns = 0, so
 * that each still states its rule from the first frameset on. The settling rule has its own
 * cases with the default second: nothing is stamped in a device stream's first second, measured
 * on host time; a global timestamp that starts several milliseconds new, as on 4 of 16 recorded
 * starts, never reaches a stamp (and does without the rule: the case proves it can fail); a
 * camera restart settles again.
 */

#include "common/capture_clock.hpp"
#include "common/capture_time_state.hpp"

#include <cstdlib>
#include <functional>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::size_t allocations = 0;
}
void* operator new(std::size_t n) {
    ++allocations;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {
using perception::CaptureClock;
using perception::CaptureClockOptions;
using perception::CaptureClockSample;
using perception::CaptureStampResult;
using Status = perception::CaptureStampStatus;
using Source = perception::CaptureStampSource;

int checks = 0;
void check(bool ok, int line) {
    ++checks;
    if (!ok) throw std::runtime_error("capture clock line " + std::to_string(line));
}
#define CHECK(x) check(static_cast<bool>(x), __LINE__)

constexpr std::int64_t kT0 = INT64_C(2890000000000000);        // host RAW at the first exposure, ns
constexpr std::int64_t kOffset0 = INT64_C(720000000000000);    // RAW - camera clock at kT0, ns
constexpr double kCameraDriftPpm = -35.0;                      // d(RAW - camera clock)/dt
constexpr std::int64_t kMonoMinusRaw0 = INT64_C(82700000000);  // CLOCK_MONOTONIC - RAW at kT0, ns
constexpr std::int64_t kHalfUs = 15054, kMinPathUs = 30767, kGuardUs = 2000;
// Between two framesets that touch the envelope (every seventh in these scenarios, 0.2 s) it rises
// 50 ppm while the camera clock moves 35 ppm the other way: the envelope candidate is up to
// 17 us newer than the truth, plus rounding. kEnv is that allowance on the "new" side.
constexpr std::int64_t kEnv = 25;
// A frameset that is 0.35 s old crosses between the two host clocks with that age: at the default
// 28.6 ppm it lands 10 us newer. kOld is that allowance for the cases with a backlog.
constexpr std::int64_t kOld = 12;

// The rule cases: no settling, so every case speaks from the first frameset on.
CaptureClockOptions options() {
    CaptureClockOptions o;
    o.min_path_us = kMinPathUs;
    o.stream_settle_ns = 0;
    return o;
}
// The product's options: the default second of settling.
CaptureClockOptions product_options() {
    CaptureClockOptions o;
    o.min_path_us = kMinPathUs;
    return o;
}

struct Scenario {
    int frames = 3000;
    std::int64_t period_ns = 33366700;
    double mono_ppm = 28.6;                                    // CLOCK_MONOTONIC against RAW
    double camera_ppm = kCameraDriftPpm;                       // d(RAW - camera clock)/dt; the measured sign is negative
    std::int64_t first_pickup_delay_ns = 0;                    // the consumer is not ready for the first frameset
    std::int64_t bracket_ns = 2000;
    std::function<std::int64_t(int)> global_error_us = [](int) { return INT64_C(500); };
    std::function<bool(int)> has_global = [](int) { return true; };
    std::function<std::int64_t(int)> busy_ns = [](int) { return INT64_C(5000000); };   // after taking frameset k
    // Extra delivery time over the shortest path; every seventh frameset comes on the shortest path.
    std::function<std::int64_t(int)> path_extra_us = [](int k) { return k % 7 == 0 ? INT64_C(0) : INT64_C(20) + (k * 37) % 300; };
    std::function<std::int64_t(int)> camera_restart_us = [](int) { return INT64_C(0); };  // taken off the camera timestamp
};

struct Out {
    CaptureStampResult r;
    std::int64_t error_us = 0;    // Stamped only
    double t_s = 0;               // host time of the pickup since the first exposure
    bool fast_alarm = false, slow_alarm = false, incomparable = false;
};

std::int64_t mono_of(const Scenario& s, std::int64_t raw_ns) {
    return raw_ns + kMonoMinusRaw0 + static_cast<std::int64_t>(s.mono_ppm * 1e-6 * static_cast<double>(raw_ns - kT0));
}

CaptureClockSample sample_of(const Scenario& s, int k, std::int64_t pickup_raw_ns) {
    const std::int64_t mid = kT0 + k * s.period_ns, end = mid + kHalfUs * 1000;
    const std::int64_t offset = kOffset0 + static_cast<std::int64_t>(s.camera_ppm * 1e-6 * static_cast<double>(end - kT0));
    CaptureClockSample x;
    x.pickup_raw_ns = pickup_raw_ns;
    x.pickup_mono_ns = mono_of(s, pickup_raw_ns);
    x.bracket_ns = s.bracket_ns;
    x.device_us = static_cast<std::uint64_t>((end - offset) / 1000 - s.camera_restart_us(k));
    x.half_exposure_us = kHalfUs;
    if (s.has_global(k)) x.global_us = static_cast<std::uint64_t>(end / 1000 + s.global_error_us(k));
    return x;
}

// Runs the scenario: framesets become available on their path, the consumer takes them in order
// when it is free. Host time only moves forward.
std::vector<Out> run(const Scenario& s, CaptureClock& clock) {
    std::vector<Out> out;
    out.reserve(static_cast<std::size_t>(s.frames));
    std::int64_t free_at = kT0 + s.first_pickup_delay_ns;
    for (int k = 0; k < s.frames; ++k) {
        const std::int64_t mid = kT0 + k * s.period_ns;
        const std::int64_t available = mid + (kHalfUs + kMinPathUs + s.path_extra_us(k)) * 1000;
        const std::int64_t pickup = std::max(available, free_at);
        free_at = pickup + s.busy_ns(k);
        Out o;
        o.r = clock.step(sample_of(s, k, pickup));
        o.t_s = static_cast<double>(pickup - kT0) / 1e9;
        if (o.r.status == Status::Stamped) o.error_us = (o.r.capture_mono_ns - mono_of(s, mid)) / 1000;
        o.fast_alarm = clock.health().fast_alarm;
        o.slow_alarm = clock.health().slow_alarm;
        o.incomparable = clock.health().incomparable;
        out.push_back(o);
    }
    return out;
}

// Every stamped frameset of [from, to) lies in [low, high] microseconds of the truth.
void within(const std::vector<Out>& out, int from, int to, std::int64_t low, std::int64_t high, int line) {
    for (int k = from; k < to; ++k) {
        const auto& o = out[static_cast<std::size_t>(k)];
        if (o.r.status != Status::Stamped) continue;
        if (o.error_us < low || o.error_us > high) {
            std::cerr << "frameset " << k << " error " << o.error_us << " us, expected " << low << ".." << high << "\n";
            check(false, line);
        }
    }
    check(true, line);
}
int count(const std::vector<Out>& out, int from, int to, Status status) {
    int n = 0;
    for (int k = from; k < to; ++k) n += out[static_cast<std::size_t>(k)].r.status == status;
    return n;
}
int first_frame(const std::vector<Out>& out, const std::function<bool(const Out&)>& f) {
    for (std::size_t k = 0; k < out.size(); ++k) if (f(out[k])) return static_cast<int>(k);
    return -1;
}

// ---- source validity ----

// Steady state. The first second is stamped from the global timestamp alone (0.5 ms new: error
// -1.5 ms); once the envelope is anchored the older candidate, the envelope, is taken (-2.0 ms).
void test_steady_state() {
    CaptureClock clock(options());
    CHECK(clock.valid());
    const auto out = run(Scenario{}, clock);
    CHECK(count(out, 0, 3000, Status::Stamped) == 3000);
    within(out, 0, 3000, -kGuardUs - 30, -kGuardUs + 500 + 5, __LINE__);
    const int anchored = first_frame(out, [](const Out& o) { return o.r.envelope_anchored; });
    CHECK(anchored >= 29 && anchored <= 32);     // one second of agreement, 30 framesets per second
    for (int k = 0; k < anchored; ++k) CHECK(out[static_cast<std::size_t>(k)].r.source == Source::Global);
    for (int k = anchored; k < 3000; ++k) {
        const auto& o = out[static_cast<std::size_t>(k)];
        CHECK(o.r.source == Source::Envelope && !o.r.estimated && o.r.comparable);   // the older one, with G present
        CHECK(o.r.global_minus_envelope_us > 400 && o.r.global_minus_envelope_us < 560);
    }
    within(out, anchored, 3000, -kGuardUs - 30, -kGuardUs + kEnv, __LINE__);
    const auto& h = clock.health();
    CHECK(h.anchors == 1 && h.device_streams == 1 && h.no_trusted_time == 0 && h.not_later == 0 && h.duplicates == 0);
    CHECK(!h.fast_alarm && !h.slow_alarm && !h.incomparable && h.slow_locked);
    CHECK(h.slow_locked_us > 400 && h.slow_locked_us < 560);
}

// A camera clock on the other side of the host clock (35 ppm slower instead of faster): now
// "pickup - camera timestamp" climbs by itself and the envelope has to follow it upwards. It may
// rise 50 ppm, so it does, and the stamp does not fall behind over 100 s. (A camera clock slower
// by MORE than the rise limit would leave the envelope behind, on the old side, and the slow
// watchdog would report the shift; the measured clock is 29..35 ppm fast.)
void test_camera_clock_slower_than_the_host() {
    Scenario s;
    s.camera_ppm = 35.0;
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 3000, Status::Stamped) == 3000);
    within(out, 40, 3000, -kGuardUs - 30, -kGuardUs + kEnv, __LINE__);
    CHECK(!clock.health().slow_alarm && !clock.health().fast_alarm && clock.health().slow_locked);
    Scenario far;
    far.camera_ppm = 120.0;                                   // beyond the rise limit: the envelope is left behind
    CaptureClock clock2(options());
    const auto out2 = run(far, clock2);
    CHECK(out2[2999].r.source == Source::Envelope && out2[2999].error_us < -kGuardUs - 5000);   // 7 ms too old after 100 s ...
    CHECK(clock2.health().slow_alarm);                                                        // ... and reported
}

// After the first frameset the consumer is busy for 310 ms (the service's first-frame work,
// measured 2026-10-14): framesets 1.. are picked up 300..0 ms late. Their stamps are still right
// and their latency says how old they are; "pickup - constant" would be 0.3 s new.
void test_backlog_after_the_start() {
    Scenario s;
    s.frames = 300;
    s.busy_ns = [](int k) { return k == 0 ? INT64_C(310000000) : INT64_C(15000000); };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 300, Status::Stamped) == 300);
    within(out, 0, 300, -kGuardUs - 30, -kGuardUs + 500 + kOld, __LINE__);
    CHECK(out[1].r.latency_ns > INT64_C(300000000) && out[1].r.arrival_lag_us > 250000);
    CHECK(out[0].r.latency_ns < INT64_C(50000000));
    const int caught_up = first_frame(out, [](const Out& o) { return o.r.arrival_lag_us < 1000 && o.t_s > 0.2; });
    CHECK(caught_up > 10 && caught_up < 25);
}

// No global timestamp, ever: the envelope has nothing to be anchored on, so no frameset gets a
// capture time, and no constant stands in. When the global timestamp appears it is used at once.
void test_no_global_timestamp_stamps_nothing() {
    Scenario s;
    s.frames = 400;
    s.has_global = [](int k) { return k >= 150; };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 150, Status::NoTrustedTime) == 150);
    for (int k = 0; k < 150; ++k) CHECK(out[static_cast<std::size_t>(k)].r.capture_mono_ns == -1 && out[static_cast<std::size_t>(k)].r.source == Source::None);
    CHECK(out[150].r.status == Status::Stamped && out[150].r.source == Source::Global);
    CHECK(count(out, 150, 400, Status::Stamped) == 250);
    within(out, 150, 400, -kGuardUs - 30, -kGuardUs + 500 + 5, __LINE__);
    CHECK(clock.health().no_trusted_time == 150 && clock.health().anchors == 1);
    CHECK(out[149].incomparable);     // nothing to compare for more than two seconds is reported
}

// The stream starts backlogged and there is no global timestamp (GPT, question 15): the first
// pickup comes 300 ms late and the consumer never catches up. Twenty framesets, or two hundred,
// anchor nothing: an envelope that has only seen late framesets would be 0.3 s new.
void test_backlogged_start_without_global_timestamp() {
    Scenario s;
    s.frames = 200;
    s.first_pickup_delay_ns = INT64_C(350000000);
    s.busy_ns = [&s](int) { return s.period_ns; };          // never catches up
    s.has_global = [](int) { return false; };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 200, Status::NoTrustedTime) == 200);
    CHECK(clock.health().anchors == 0 && clock.health().stamped_envelope == 0);
    // What the envelope alone would have said, for the record: about 0.3 s too new.
    CHECK(out[150].r.arrival_lag_us < 2000);   // it believes this frameset came on the shortest path
}

// The same backlogged start with a global timestamp: stamped from it all along; the envelope is
// inflated by the backlog, disagrees by 0.3 s and is not anchored until framesets come on the
// shortest path again and it has agreed for a second.
void test_backlogged_start_with_global_timestamp() {
    Scenario s;
    s.frames = 300;
    s.first_pickup_delay_ns = INT64_C(350000000);
    s.busy_ns = [](int) { return INT64_C(15000000); };      // catches up at about 18 ms per frameset
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 300, Status::Stamped) == 300);
    within(out, 0, 300, -kGuardUs - 30, -kGuardUs + 500 + kOld, __LINE__);
    CHECK(out[0].r.source == Source::Global && !out[0].r.envelope_anchored);
    CHECK(out[1].r.comparable && out[1].r.global_minus_envelope_us < -200000);   // the envelope is 0.2 s and more too new
    const int caught_up = first_frame(out, [](const Out& o) { return o.r.comparable && o.r.global_minus_envelope_us > -4000; });
    const int anchored = first_frame(out, [](const Out& o) { return o.r.envelope_anchored; });
    CHECK(caught_up > 10 && anchored >= caught_up + 29 && anchored <= caught_up + 33);
    for (int k = 0; k < anchored; ++k) CHECK(out[static_cast<std::size_t>(k)].r.source == Source::Global);
}

// The global timestamp is lost for ten seconds in steady state: the anchored envelope carries
// on, marked as estimated; the loss is reported; the global timestamp is used again when back.
void test_global_timestamp_lost() {
    Scenario s;
    s.frames = 1500;
    s.has_global = [](int k) { return k < 600 || k >= 900; };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 1500, Status::Stamped) == 1500);
    for (int k = 600; k < 900; ++k) {
        const auto& o = out[static_cast<std::size_t>(k)];
        CHECK(o.r.source == Source::Envelope && o.r.estimated && !o.r.comparable && !o.r.global_usable);
    }
    within(out, 600, 900, -kGuardUs - 60, -kGuardUs + kEnv, __LINE__);
    CHECK(!out[640].incomparable && out[720].incomparable && !out[960].incomparable);
    CHECK(out[900].r.global_usable && !out[900].r.estimated);
    within(out, 900, 1500, -kGuardUs - 30, -kGuardUs + kEnv, __LINE__);
    CHECK(clock.health().anchors == 1);
}

// The global timestamp is lost WHILE framesets queue for 18 s (the two-core case of 2026-10-13;
// GPT, question 15): no frameset touches the envelope, which rises 50 ppm while the camera clock
// runs 35 ppm the other way, so the bare envelope ends 1.5 ms new. Standing alone it is moved
// back by what it may have risen, and no stamp is newer than the guard line.
void test_global_timestamp_lost_while_framesets_queue() {
    Scenario s;
    s.frames = 1800;
    const int from = 600, to = 1140;
    s.has_global = [=](int k) { return k < from || k >= to; };
    s.busy_ns = [=, &s](int k) { return k >= from && k < to ? s.period_ns + INT64_C(530000) : INT64_C(5000000); };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(out[static_cast<std::size_t>(to - 1)].r.arrival_lag_us > 250000);          // the queue did build up: 0.25 s and more
    for (int k = from + 5; k < to; ++k) CHECK(out[static_cast<std::size_t>(k)].r.estimated);
    within(out, from, to, -kGuardUs - 400, -kGuardUs + kEnv, __LINE__);
    // Afterwards the global timestamp is back while the queue drains: stamped from it (the bare
    // envelope is 1.5 ms new by then and is not the older one).
    CHECK(out[static_cast<std::size_t>(to)].r.source == Source::Global && out[static_cast<std::size_t>(to)].r.global_minus_envelope_us < -900);
    within(out, 0, 1800, -kGuardUs - 400, -kGuardUs + 500 + kOld, __LINE__);
}

// The SDK's mapping goes stale: the global timestamp is 0.5 ms new, then drifts newer by
// 35 us per second for two minutes. The stamp never follows it (the envelope is older), and the
// slow watchdog reports the shift.
void test_global_timestamp_drifts() {
    Scenario s;
    s.frames = 4500;                                         // 150 s
    s.global_error_us = [&s](int k) {
        const double t = static_cast<double>(k) * static_cast<double>(s.period_ns) / 1e9;
        return INT64_C(500) + (t > 20 ? static_cast<std::int64_t>(35.0 * (t - 20)) : 0);
    };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 4500, Status::Stamped) == 4500);
    within(out, 40, 4500, -kGuardUs - 30, -kGuardUs + kEnv, __LINE__);
    // The 60 s median is locked at about 55 s (0.76 ms: the median of what the window has seen)
    // and has moved 1.5 ms from there at about 100 s; the absolute limit of 2.5 ms would only be
    // passed at 107 s. So an alarm between 95 and 105 s is the SHIFT rule at work.
    const int slow = first_frame(out, [](const Out& o) { return o.slow_alarm; });
    CHECK(slow > 0 && out[static_cast<std::size_t>(slow)].t_s > 95 && out[static_cast<std::size_t>(slow)].t_s < 105);
    CHECK(clock.health().slow_locked_us > 600 && clock.health().slow_locked_us < 900);
    CHECK(clock.health().slow_alarm && clock.health().fast_alarm);   // by the end it is 4.5 ms off
}

// A disagreement that is there from the first frameset and never changes, 2.8 ms: it is inside
// the anchoring tolerance and below the fast limit, and the 60 s median never SHIFTS -- the
// absolute limit of the slow watchdog is what reports it, as soon as the first median exists.
void test_constant_disagreement_is_reported() {
    Scenario s;
    s.frames = 2400;
    s.global_error_us = [](int) { return INT64_C(2800); };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    within(out, 40, 2400, -kGuardUs - 30, -kGuardUs + kEnv, __LINE__);          // the older one, the envelope, is used
    const int slow = first_frame(out, [](const Out& o) { return o.slow_alarm; });
    CHECK(slow > 0 && out[static_cast<std::size_t>(slow)].t_s > 50 && out[static_cast<std::size_t>(slow)].t_s < 62);
    CHECK(first_frame(out, [](const Out& o) { return o.fast_alarm; }) == -1);
    CHECK(clock.health().slow_locked_us > 2700 && clock.health().slow_locked_us < 2850);
}

// The camera clock restarts and the SDK has no mapping for two seconds: a new device stream.
// The old envelope and its anchor are gone and nothing vouches for the time, so those framesets
// are not stamped (no constant); the global timestamp, back, is used at once and the envelope is
// anchored again a second later.
void test_camera_clock_restarts() {
    Scenario s;
    s.frames = 1200;
    s.camera_restart_us = [](int k) { return k >= 600 ? INT64_C(2000000000) : INT64_C(0); };
    s.has_global = [](int k) { return k < 600 || k >= 660; };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(count(out, 0, 600, Status::Stamped) == 600);
    CHECK(count(out, 600, 660, Status::NoTrustedTime) == 60);
    CHECK(out[600].r.device_stream == 2 && !out[600].r.envelope_anchored && out[599].r.device_stream == 1);
    CHECK(out[660].r.status == Status::Stamped && out[660].r.source == Source::Global);
    CHECK(count(out, 660, 1200, Status::Stamped) == 540);
    within(out, 660, 1200, -kGuardUs - 30, -kGuardUs + 500 + 5, __LINE__);
    const int anchored = first_frame(out, [](const Out& o) { return o.r.device_stream == 2 && o.r.envelope_anchored; });
    CHECK(anchored >= 689 && anchored <= 693);
    CHECK(clock.health().device_streams == 2 && clock.health().anchors == 2 && clock.health().not_later == 0);
    // A camera clock that jumps AHEAD by more than a second within one frame period is a new stream too.
    Scenario ahead;
    ahead.frames = 700;
    ahead.camera_restart_us = [](int k) { return k >= 600 ? -INT64_C(5000000) : INT64_C(0); };
    CaptureClock clock2(options());
    const auto out2 = run(ahead, clock2);
    CHECK(out2[600].r.device_stream == 2 && out2[599].r.device_stream == 1 && clock2.health().device_streams == 2);
}

// The same frame delivered twice (equal camera timestamp), and a camera timestamp 0.4 ms EARLIER
// than the last one: neither is a restart. No state changes, the envelope stays anchored.
void test_duplicate_frame_is_not_a_restart() {
    CaptureClock clock(options());
    Scenario s;
    std::int64_t free_at = kT0;
    CaptureStampResult last;
    for (int k = 0; k < 700; ++k) {
        const std::int64_t available = kT0 + k * s.period_ns + (kHalfUs + kMinPathUs + s.path_extra_us(k)) * 1000;
        const std::int64_t pickup = std::max(available, free_at);
        free_at = pickup + 5000000;
        last = clock.step(sample_of(s, k, pickup));
        CHECK(last.status == Status::Stamped);
        if (k == 400) {
            auto again = sample_of(s, k, pickup + 1000000);          // the same frame, a millisecond later
            const auto r = clock.step(again);
            CHECK(r.status == Status::Duplicate && r.capture_mono_ns == -1 && r.device_stream == 1);
            again = sample_of(s, k, pickup + 2000000);
            again.device_us -= 400;                                   // 0.4 ms earlier: still not a restart
            if (again.global_us) *again.global_us -= 400;
            CHECK(clock.step(again).status == Status::Duplicate);
            free_at = pickup + 3000000;
        }
    }
    CHECK(last.envelope_anchored && last.device_stream == 1);
    CHECK(clock.health().duplicates == 2 && clock.health().device_streams == 1 && clock.health().anchors == 1);
}

// ---- selection and alarms ----

// The global timestamp turns 3.5 ms new for ten seconds (the counterexample to the first draft
// of the rule, which followed it without an alarm). The stamp stays on the envelope, and the
// fast watchdog reports it within a few seconds and clears afterwards. 2.5 ms: no alarm.
void test_global_timestamp_turns_new() {
    for (const std::int64_t extra : {INT64_C(3500), INT64_C(2000)}) {
        Scenario s;
        s.frames = 1500;
        s.global_error_us = [=](int k) { return k >= 600 && k < 900 ? INT64_C(500) + extra : INT64_C(500); };
        CaptureClock clock(options());
        const auto out = run(s, clock);
        within(out, 40, 1500, -kGuardUs - 30, -kGuardUs + kEnv, __LINE__);                     // never newer than the guard line
        for (int k = 602; k < 900; ++k) CHECK(out[static_cast<std::size_t>(k)].r.source == Source::Envelope);
        const int alarm = first_frame(out, [](const Out& o) { return o.fast_alarm; });
        if (extra == 3500) {
            CHECK(alarm > 600 && out[static_cast<std::size_t>(alarm)].t_s - out[600].t_s < 3.5);
            CHECK(!out[1050].fast_alarm && !clock.health().fast_alarm);                       // cleared after the fault
        } else {
            CHECK(alarm == -1);                                                               // 2.5 ms is under the limit
        }
        CHECK(!clock.health().slow_alarm);
    }
}

// The alarm runs on host time: with one frameset every 33 ms or every 100 ms the same fault is
// reported after about the same time, not after the same number of framesets.
void test_alarm_runs_on_time_not_on_frames() {
    double raised_after[2] = {0, 0};
    int frames_until[2] = {0, 0};
    int i = 0;
    for (const std::int64_t period : {INT64_C(33366700), INT64_C(100100100)}) {
        Scenario s;
        s.period_ns = period;
        s.frames = static_cast<int>(INT64_C(20000000000) / period);
        const int onset = static_cast<int>(INT64_C(8000000000) / period);
        s.global_error_us = [=](int k) { return k >= onset ? INT64_C(5500) : INT64_C(500); };
        CaptureClock clock(options());
        const auto out = run(s, clock);
        const int alarm = first_frame(out, [](const Out& o) { return o.fast_alarm; });
        CHECK(alarm > onset);
        raised_after[i] = out[static_cast<std::size_t>(alarm)].t_s - out[static_cast<std::size_t>(onset)].t_s;
        frames_until[i] = alarm - onset;
        ++i;
    }
    CHECK(raised_after[0] > 0.9 && raised_after[0] < 3.3 && raised_after[1] > 0.9 && raised_after[1] < 3.3);
    CHECK(frames_until[0] > 2 * frames_until[1]);
}

// A LIMIT, not a guarantee (GPT, question 15, built from the SDK's formula): from 20 s on, the
// delivery path in front of BOTH candidates grows by 20 us per second -- the envelope follows it
// (it may rise 50) and the SDK's global timestamp carries the same slow term. Both end 3.6 ms
// new, 0.5 ms apart as always, and no watchdog fires. This case exists so that nobody reads
// "the two candidates agree" as "the time is right"; it must be rewritten if that ever changes.
void test_known_limit_common_delay() {
    Scenario s;
    s.frames = 6000;                                         // 200 s
    const auto grown_us = [&s](int k) {
        const double t = static_cast<double>(k) * static_cast<double>(s.period_ns) / 1e9;
        return t > 20 ? static_cast<std::int64_t>(20.0 * (t - 20)) : INT64_C(0);
    };
    s.path_extra_us = [=](int k) { return grown_us(k) + (k % 7 == 0 ? INT64_C(0) : INT64_C(20) + (k * 37) % 300); };
    s.global_error_us = [=](int k) { return INT64_C(500) + grown_us(k); };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(out[5999].r.status == Status::Stamped && out[5999].error_us > 1400 && out[5999].error_us < 1700);   // 1.6 ms NEW after the guard
    CHECK(first_frame(out, [](const Out& o) { return o.fast_alarm || o.slow_alarm; }) == -1);
}

// ---- the final time domain ----

void test_inputs_and_bracket() {
    Scenario s;
    CaptureClock clock(options());
    const std::int64_t pickup = kT0 + (kHalfUs + kMinPathUs) * 1000;
    auto x = sample_of(s, 0, pickup);
    x.bracket_ns = 250000;                                   // the three clock readings took 0.25 ms: read again
    CHECK(clock.step(x).status == Status::BracketTooWide && clock.health().bracket_too_wide == 1);
    x.bracket_ns = 2000;
    auto bad = x; bad.device_us = 0;            CHECK(clock.step(bad).status == Status::InvalidInput);
    bad = x; bad.half_exposure_us = -1;         CHECK(clock.step(bad).status == Status::InvalidInput);
    bad = x; bad.half_exposure_us = 200000;     CHECK(clock.step(bad).status == Status::InvalidInput);
    bad = x; bad.pickup_raw_ns = -1;            CHECK(clock.step(bad).status == Status::InvalidInput);
    bad = x; bad.pickup_mono_ns = -1;           CHECK(clock.step(bad).status == Status::InvalidInput);
    bad = x; bad.bracket_ns = -1;               CHECK(clock.step(bad).status == Status::InvalidInput);
    CHECK(clock.health().invalid_input == 6 && clock.health().device_streams == 0);
    CHECK(clock.step(x).status == Status::Stamped);          // none of the refused samples left a trace
    auto next = sample_of(s, 1, pickup);                     // the host clock did not move
    CHECK(clock.step(next).status == Status::InvalidInput);
    // A global timestamp later than the pickup, or five seconds and more before it, is not
    // believed. Each on a clock of its own and as its first frameset, so that nothing but this
    // rule can turn it down (there is no earlier global timestamp to compare the step with).
    {
        CaptureClock fresh(options());
        auto future = sample_of(s, 0, pickup);
        *future.global_us += 100000;
        const auto r = fresh.step(future);
        CHECK(r.status == Status::NoTrustedTime && !r.global_usable && fresh.health().invalid_input == 0);
    }
    {
        CaptureClock fresh(options());
        auto ancient = sample_of(s, 0, pickup);
        *ancient.global_us -= 6000000;
        const auto r = fresh.step(ancient);
        CHECK(r.status == Status::NoTrustedTime && !r.global_usable && fresh.health().no_trusted_time == 1);
        auto old_but_plausible = sample_of(s, 1, pickup + s.period_ns);
        *old_but_plausible.global_us -= 4000000;              // four seconds: within max_latency_ns, used
        CaptureClock other(options());
        CHECK(other.step(old_but_plausible).status == Status::Stamped);
    }
    // Options without the calibrated shortest path are refused, and such a clock stamps nothing.
    CaptureClockOptions o;
    CHECK(!perception::valid_capture_clock_options(o));
    CaptureClock unset(o);
    CHECK(!unset.valid() && unset.step(x).status == Status::InvalidInput);
    o = options(); o.slow_window_ns = o.fast_window_ns;
    CHECK(!perception::valid_capture_clock_options(o));
}

// Whole milliseconds that increase: a frameset 0.3 ms after the last one falls into the same
// published millisecond and is not published; nothing is moved forward to make room.
void test_published_milliseconds_increase() {
    Scenario s;
    CaptureClock clock(options());
    std::int64_t last_ms = -1;
    for (int k = 0; k < 100; ++k) {
        const std::int64_t pickup = kT0 + k * s.period_ns + (kHalfUs + kMinPathUs) * 1000;
        const auto r = clock.step(sample_of(s, k, pickup));
        CHECK(r.status == Status::Stamped && r.capture_mono_ms > last_ms && r.capture_mono_ms == r.capture_mono_ns / 1000000);
        last_ms = r.capture_mono_ms;
        if (k == 50) {
            auto close = sample_of(s, k, pickup + 300000);
            close.device_us += 300;
            *close.global_us += 300;
            const bool same_ms = (r.capture_mono_ns + 300000) / 1000000 == r.capture_mono_ms;
            const auto again = clock.step(close);
            CHECK(again.status == (same_ms ? Status::NotLater : Status::Stamped));
            if (!same_ms) last_ms = again.capture_mono_ms;
            CHECK(same_ms);                                   // the scenario is built to hit the same millisecond
        }
    }
    CHECK(clock.health().not_later == 1);
}

// The last published time survives a camera restart. Right after the restart the SDK's mapping
// is 80 ms old for three framesets: two of them would be earlier than what was already published
// and are not published.
void test_published_time_survives_a_restart() {
    Scenario s;
    s.frames = 700;
    s.camera_restart_us = [](int k) { return k >= 600 ? INT64_C(2000000000) : INT64_C(0); };
    s.global_error_us = [](int k) { return k >= 600 && k < 603 ? -INT64_C(80000) : INT64_C(500); };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(out[600].r.status == Status::NotLater && out[601].r.status == Status::NotLater);
    CHECK(out[602].r.status == Status::Stamped);
    CHECK(out[603].r.status == Status::NoTrustedTime);        // the mapping jumped back by 80 ms: not believed for this frameset
    CHECK(out[604].r.status == Status::Stamped);
    std::int64_t last_ms = -1;
    for (const auto& o : out) {
        if (o.r.status != Status::Stamped) continue;
        CHECK(o.r.capture_mono_ms > last_ms);
        last_ms = o.r.capture_mono_ms;
    }
    CHECK(clock.health().not_later == 2 && clock.health().device_streams == 2);
}

// CLOCK_MONOTONIC running 85 ppm against RAW (seen on 2026-10-14) while a frameset is 0.35 s
// old: only that age crosses between the clocks, 0.03 ms.
void test_two_host_clocks() {
    Scenario s;
    s.frames = 300;
    s.mono_ppm = 85.0;
    s.busy_ns = [](int k) { return k == 0 ? INT64_C(350000000) : INT64_C(15000000); };
    CaptureClock clock(options());
    const auto out = run(s, clock);
    CHECK(out[1].r.latency_ns > INT64_C(340000000));
    within(out, 0, 300, -kGuardUs - 30, -kGuardUs + 500 + 40, __LINE__);
    CHECK(out[1].error_us > -kGuardUs + 500 + 20 && out[1].error_us < -kGuardUs + 500 + 40);   // the 0.03 ms is there, and no more
}

// ---- settling ----

// The default second. Nothing is stamped before the device stream is one second old; the state
// is built meanwhile, so the first stamp already has an anchored envelope beside the global
// timestamp. The second is host time: at 15 framesets per second it is 15 framesets, not 30.
void test_new_stream_settles_for_a_second() {
    CHECK(product_options().stream_settle_ns == INT64_C(1000000000));
    for (const std::int64_t period_ns : {INT64_C(33366700), INT64_C(66733400)}) {
        Scenario s;
        s.period_ns = period_ns;
        s.frames = 600;
        CaptureClock clock(product_options());
        const auto out = run(s, clock);
        const int first = first_frame(out, [](const Out& o) { return o.r.status == Status::Stamped; });
        CHECK(first > 0);
        for (int k = 0; k < first; ++k) {
            const auto& o = out[static_cast<std::size_t>(k)];
            CHECK(o.r.status == Status::Settling && o.r.capture_mono_ns < 0 && o.r.source == Source::None && !o.r.estimated);
        }
        const double first_s = out[static_cast<std::size_t>(first)].t_s - out[0].t_s;
        CHECK(first_s >= 1.0 && first_s < 1.0 + 1.5 * static_cast<double>(period_ns) / 1e9);
        CHECK(count(out, first, s.frames, Status::Stamped) == s.frames - first);
        CHECK(out[static_cast<std::size_t>(first)].r.global_usable);
        // One second of agreement was collected while settling: anchored within two framesets of the first stamp.
        const int anchored = first_frame(out, [](const Out& o) { return o.r.envelope_anchored; });
        CHECK(anchored >= 0 && anchored <= first + 2);
        within(out, first, s.frames, -kGuardUs - 30, -kGuardUs + 500 + 5, __LINE__);
        CHECK(clock.health().settling == static_cast<std::uint64_t>(first) && clock.health().no_trusted_time == 0);
    }
}

// What the second is for. As recorded on 2026-10-14: until the SDK's next time query, 0.23 s after
// the first frameset, its global timestamp is a short fit carried ahead, here 5 ms new and
// growing. With the rule no stamp is newer than the truth; without it the first framesets are,
// by about 3 ms, which is the failure the rule removes.
void test_global_timestamp_wrong_at_the_start() {
    Scenario s;
    s.frames = 600;
    s.global_error_us = [](int k) { return k < 8 ? INT64_C(5000) + 150 * k : INT64_C(500); };
    {
        CaptureClock clock(product_options());
        const auto out = run(s, clock);
        CHECK(count(out, 0, 8, Status::Settling) == 8);
        within(out, 0, s.frames, -kGuardUs - 30, -kGuardUs + 500 + 5, __LINE__);
        CHECK(count(out, 40, s.frames, Status::Stamped) == s.frames - 40);
    }
    {
        CaptureClock clock(options());   // no settling
        const auto out = run(s, clock);
        int newer = 0;
        for (int k = 0; k < 8; ++k) newer += out[static_cast<std::size_t>(k)].r.status == Status::Stamped && out[static_cast<std::size_t>(k)].error_us > 2500;
        CHECK(newer == 8);
    }
}

// A camera whose clock starts again is a new device stream: it settles again, on host time from
// its own first frameset, and the published time still only moves forward.
void test_camera_restart_settles_again() {
    Scenario s;
    s.frames = 900;
    s.camera_restart_us = [](int k) { return k >= 450 ? INT64_C(600000000) : INT64_C(0); };
    CaptureClock clock(product_options());
    const auto out = run(s, clock);
    CHECK(clock.health().device_streams == 2);
    const int first = first_frame(out, [](const Out& o) { return o.r.status == Status::Stamped; });
    CHECK(count(out, first, 450, Status::Stamped) == 450 - first);
    int again = -1;
    for (int k = 450; k < s.frames; ++k)
        if (out[static_cast<std::size_t>(k)].r.status == Status::Stamped) { again = k; break; }
    CHECK(again > 450 && count(out, 450, again, Status::Settling) == again - 450);
    const double settle_s = out[static_cast<std::size_t>(again)].t_s - out[450].t_s;
    CHECK(settle_s >= 1.0 && settle_s < 1.06);
    CHECK(out[static_cast<std::size_t>(again)].r.device_stream == 2);
    CHECK(out[static_cast<std::size_t>(again)].r.capture_mono_ms > out[449].r.capture_mono_ms);
    within(out, 0, s.frames, -kGuardUs - 30, -kGuardUs + 500 + 5, __LINE__);
}

// A negative settling time is not a valid option; zero switches the rule off.
void test_settle_option() {
    auto o = product_options();
    o.stream_settle_ns = -1;
    CHECK(!perception::valid_capture_clock_options(o) && !CaptureClock(o).valid());
    o.stream_settle_ns = 0;
    CHECK(perception::valid_capture_clock_options(o));
}

// The capture loop's 5 s deadline exempts a device stream's first second as a time interval
// (GPT 16 supplement): a frameset refused for another reason inside that second (here a bracket too
// wide at 0.1 s) must not start the deadline early. No global timestamp ever, so nothing is
// stamped and the loop ends on the deadline: not before 1 s + 5 s after the first frameset, and
// within two frames after. Feeding the deadline the status instead (the T5 code) ended at 5.1 s.
// Before any stream has started there is no window: an invalid first frameset starts it at once.
void test_startup_exemption_is_a_time_window() {
    Scenario s;
    s.has_global = [](int) { return false; };
    const auto o = product_options();
    CaptureClock clock(o);
    perception::CaptureTimeWithheldRun run;
    std::int64_t first_mono = -1, too_long_ms = -1;
    for (int k = 0; k < 400 && too_long_ms < 0; ++k) {
        auto x = sample_of(s, k, kT0 + k * s.period_ns + (kHalfUs + kMinPathUs + 100) * 1000);
        if (k == 3) x.bracket_ns = o.max_bracket_ns + 1;
        const auto r = clock.step(x);
        CHECK(r.status != Status::Stamped);
        CHECK(k != 3 || r.status == Status::BracketTooWide);
        if (first_mono < 0) first_mono = x.pickup_mono_ns;
        if (run.withheld(x.pickup_mono_ns, clock.in_stream_settle(x.pickup_raw_ns)) ==
            perception::CaptureTimeWithheldRun::Step::TooLong)
            too_long_ms = (x.pickup_mono_ns - first_mono) / 1000000;
    }
    CHECK(too_long_ms >= 6000 && too_long_ms <= 6000 + 2 * s.period_ns / 1000000);
    // No stream yet: an invalid first frameset is outside any window and starts the deadline.
    CaptureClock fresh(o);
    auto bad = sample_of(s, 0, kT0 + (kHalfUs + kMinPathUs) * 1000);
    bad.device_us = 0;
    CHECK(fresh.step(bad).status == Status::InvalidInput && !fresh.in_stream_settle(bad.pickup_raw_ns));
    perception::CaptureTimeWithheldRun early;
    early.withheld(bad.pickup_mono_ns, fresh.in_stream_settle(bad.pickup_raw_ns));
    CHECK(early.limit_from_ns() == bad.pickup_mono_ns);
}

void test_no_allocation() {
    Scenario s;
    CaptureClock clock(options());
    std::vector<CaptureClockSample> samples;
    for (int k = 0; k < 2500; ++k)
        samples.push_back(sample_of(s, k, kT0 + k * s.period_ns + (kHalfUs + kMinPathUs + s.path_extra_us(k)) * 1000));
    const std::size_t before = allocations;
    std::size_t stamped = 0;
    for (const auto& x : samples) stamped += clock.step(x).status == Status::Stamped;
    CHECK(allocations == before && stamped == 2500 && clock.health().slow_locked);
}
}  // namespace

int main() {
    try {
        test_steady_state();
        test_camera_clock_slower_than_the_host();
        test_backlog_after_the_start();
        test_no_global_timestamp_stamps_nothing();
        test_backlogged_start_without_global_timestamp();
        test_backlogged_start_with_global_timestamp();
        test_global_timestamp_lost();
        test_global_timestamp_lost_while_framesets_queue();
        test_global_timestamp_drifts();
        test_constant_disagreement_is_reported();
        test_camera_clock_restarts();
        test_duplicate_frame_is_not_a_restart();
        test_global_timestamp_turns_new();
        test_alarm_runs_on_time_not_on_frames();
        test_known_limit_common_delay();
        test_inputs_and_bracket();
        test_published_milliseconds_increase();
        test_published_time_survives_a_restart();
        test_two_host_clocks();
        test_new_stream_settles_for_a_second();
        test_global_timestamp_wrong_at_the_start();
        test_camera_restart_settles_again();
        test_settle_option();
        test_startup_exemption_is_a_time_window();
        test_no_allocation();
        std::cout << "capture clock: 25 cases, " << checks << " checks PASS (synthetic streams with known exposure times)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
