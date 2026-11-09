/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_profile_time2.cpp
 * Brief: A19-TIME-2 on real threads and a real clock: a stalled slow line leaves the Profile rate unchanged.
 *
 * Description:
 * What it checks (19 A19-TIME-2, P19-1, section 2.4; host level is formal evidence, user decision
 * 2026-09-30, B9-13 (a)). Three real threads on CLOCK_MONOTONIC. The main thread is the camera:
 * one frame of a flat floor every kCameraPeriodMs into the real mailbox. A slow-line thread is the
 * inference: every kSlowPeriodMs it publishes an empty Objects snapshot into the real S slot and a
 * full mask into the real T slot, stamped with the frame offered two frames earlier; at 3 s it is
 * stuck for 5 s (it sleeps inside its loop, so neither slot changes), then goes on. A fast-line
 * thread runs service::run_profile_steady(), the service's own steady Profile loop with its slot
 * adapters and its wait, over the real consumer, T owner (zero margin, seg_max_age 300 ms) and S
 * projector (objects_stale 100 ms, guard 15 ms). Puts go to a recording transport. The floor is
 * the light one of tests/rns_profile_thread.hpp (1/16 of the pixels, same routes, T backing and
 * S): with the full floor a step took about 51 ms on the 1.8 and the camera outran it (2026-10-15).
 * Before the timed run the loop is measured closed-loop for 30 frames with fresh snapshots. While
 * the loop runs its log lines are captured (and printed after it): the stop group carries the T
 * stage the loop measures itself (PROFILE_T_TIMING). Gates:
 *   G0  that measured step's p90 is at most 3/4 of the camera period: the machine can show
 *       whether the fast line follows the camera. It runs through the loop's own adapters, so a
 *       fast line that waits fails it too (the contract's mutant does); a red G0 is never skipped;
 *   G1  before the stall, published / offered >= 0.9: the fast line follows the camera, not the
 *       slow line (only visible because the camera is at least 1.5x faster than the slow line);
 *   G2  during the stall the publication rate is at least 0.9x the rate before it, and
 *       published / offered >= 0.9;
 *   G2b the median frame latency (put - host ready) during the stall is within 10 ms of the
 *       median before it, either way (no waiting added while the masks are stale, nor while
 *       they are fresh: with a light step such a wait fits into P and costs no frame);
 *   G3  every frame captured more than 300 ms after the last mask before the stall and put before
 *       the slow line resumes has t_seg null and no src bit 0, and there are at least
 *       0.9 x (5000 - 300 - 2P) / P such frames;
 *   G4  every non-null t_seg is at most 300 ms older than its frame;
 *   G5  before the stall at least 95 % of the frames carry t_seg and bit 0 bins;
 *   G6  after the slow line resumes, a frame with t_seg appears within 2 slow periods + 2 P;
 *   G7  the test itself: the stall lasted >= 5 s with no publication into either slot, the loop
 *       returned without an exception or an abnormal frame, the T ready flag was set, every put
 *       was recorded, and the T slot saw no torn, future or out-of-order read;
 *   G8  no wait of a few ms in the T access: at most 10 % of the T stages took more than 5 ms.
 *       The T stage is grid end -> T end on the consumer's own stamps (the loop's T adapter,
 *       TMaskSlot::latest() with its 1 MB copy, the T owner's work); the count is the loop's own,
 *       stage_over5 / n of its PROFILE_T_TIMING when=stop line, and n must equal its T outcome
 *       counters. Light floor, -O2, development Mac, 19 runs: p50 0.3-0.5 ms, p90 0.4-0.8 ms,
 *       0-5 of about 317 over 5 ms (stale masks about 0.01 ms); every core busy and nice 19, 10
 *       runs: p90 1.9-2.4 ms, 0-1 over 5 ms. Expected on the 1.8 (about 3x slower; not measured
 *       yet): p90 about 1.5-2.5 ms, a few % over 5 ms at most.
 * Mutants that must turn it red (each done once, see the commits' change lists): the fast line
 * waiting for a newer mask in its T adapter (M1), in its idle wait (M2), inside the consumer step
 * (M3) or inside TMaskSlot::latest() (M4); waiting only while the mask is stale (M5); no age check
 * in the T owner (M6); waiting up to 20 ms only while the mask is fresh (M7); since 2026-10-15 the
 * T adapter sleeping 12 ms on every call (W12), 6 ms on every call (W6), 12 ms only while its mask
 * is stale or missing (W12s), 6 ms only while it is fresh (W6f).
 * What it can and cannot catch (2026-10-15). Each gate sees one symptom of waiting. The sizes
 * below are what those mutants did on the development Mac (M1-M7 also on the 1.8): an empirical
 * record for them, not a general guarantee that a shorter or rarer wait is absent.
 *   - a wait for a newer mask that costs frames: G1/G2 (M1-M4);
 *   - a wait in one phase only (fresh masks, or stale ones) that moves the median latency by more
 *     than 10 ms: G2b (M5, M7, W12s);
 *   - a fixed wait inside the T stage, in either phase or both, on more than 10 % of the frames
 *     and long enough for the stage to pass 5 ms: G8 (W12, W6, W12s, W6f; M1, M3-M5 and M7 too). On
 *     the Mac a 4 ms sleep_for (about 5 ms real: macOS oversleeps) was caught and a 3 ms one (about
 *     4 ms real, 3-8 % of the stages over 5 ms) was not; on the 1.8 the stage's own time is larger
 *     and sleeps are closer to what is asked, so from about 3.5-4 ms (expected, not measured).
 *     G0-G7 cannot see a wait common to both phases that fits into P: G0 allows 3/4 P, and G2b
 *     compares the two phases, so a common wait cancels (GPT's counterexample of 2026-10-15, W12:
 *     step p90 4.0 -> 19.2 ms, G0-G7 all passed; now red at G8 alone);
 *   - no age check in the T owner (M6, not a wait): G2/G3;
 *   - not caught: a T-stage wait shorter than that or on fewer than 10 % of the frames; a wait
 *     common to both phases outside the T stage (the S adapter, the other stages) until the
 *     step's p90 passes 3/4 P (G0) or frames are lost (the S adapter sleeping 12 ms on every call
 *     passed every gate); a one-phase wait outside the T stage under 10 ms.
 * --step-cost runs the same measurement alone, for the light floor and the full one, with fresh
 * snapshots and with stale ones, and says how the light floor's p90 sits against 33 and 50 ms;
 * the loop's PROFILE_T_TIMING when=stop line it prints is the T stage to hold against G8.
 * What it does not check. Zenoh (a fake transport; Profile, Objects and Status share one RT
 * session in the service), the capture thread (the test is the camera), a slow line stuck halfway
 * through a slot write, the first-frame setup, and the waits listed as not caught above. G8 needs
 * the synchronous logger (PERCEPTION_ASYNC_LOG unset, as in ctest): its line is read from the log.
 * The 10 s segmentation flag is test_t_status's.
 * Build at -O2, no sanitizers (tests/rns_host/CMakeLists.txt): it measures real time.
 */

#include "rns_profile_thread.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {
using namespace perception::test_profile;

// ---- Rates and gates. ----
// The camera period P. One step of the fast line (-O2, the light floor, a fresh mask) must fit in it
// with room to spare on the machine that runs the test (G0: p90 at most 3/4 of P). Light floor,
// development Mac, closed loop: p50 2.9 ms, p90 3.3 ms; the full floor took 18.4 ms there and
// about 51 ms on the 1.8, so the light floor's step there should be near 9 ms (not measured yet:
// --step-cost under the regression chain's conditions, nice 19 on cores 2-3).
constexpr std::int64_t kCameraPeriodMs = 33;
// The slow line (inference) period. A fast line paced by the slow line looks normal unless the
// camera is at least 1.5x faster than the slow line (G1).
constexpr std::int64_t kSlowPeriodMs = 50;
static_assert(2 * kSlowPeriodMs >= 3 * kCameraPeriodMs, "the camera must be at least 1.5x the slow line (G1)");
// Timeline from the first frame: warm-up (not judged), the stall order, its length, the end.
constexpr std::int64_t kWarmupMs = 1000, kStallAtMs = 3000, kStallMs = 5000, kEndMs = 9500;
static_assert(kEndMs >= kStallAtMs + kSlowPeriodMs + kStallMs + 1200, "at least 1.2 s after the stall");
// seg_max_age_ms (19 section 2.4 suggests 300), objects_stale_ms and the publish guard of the
// service, and the capture lag of each frame (capture time = host ready - lag).
constexpr std::int64_t kSegMaxAgeMs = 300, kObjectsStaleMs = 100, kPublishGuardMs = 15, kCaptureLagMs = 30;
constexpr double kMinFollow = 0.9, kMinRateKept = 0.9, kMaxExtraLatencyMs = 10, kMinBacked = 0.95;
constexpr double kMaxStepShare = 0.75;  // G0: step p90 / P
// G8: the share of T stages over 5 ms, the first mark of the loop's own T line (stage_over5,
// ProfileTiming::enable_t; an exact count). A share, not a maximum: a T stage now and then stretched
// by preemption or a page fault is not a wait (development Mac, idle: 0-5 of about 317 over 5 ms,
// the longest up to 33 ms). 10 % still sees a wait in either phase: fresh and stale masks are each
// about half of the frames.
constexpr double kMaxTOver5Share = 0.1;
constexpr std::size_t kCalibrationFrames = 30;
constexpr std::int64_t kMs = 1000000;

double median(std::vector<double> v) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
double quantile(std::vector<double> v, double q) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(q * static_cast<double>(v.size() - 1))];
}

// The number after `key` (a field of a timing line, with its leading space) in `line`; -1 when missing.
double field(const std::string& line, std::string_view key) {
    const auto at = line.find(key);
    if (at == std::string::npos) return -1;
    char* end = nullptr;
    const char* const from = line.c_str() + at + key.size();
    const double value = std::strtod(from, &end);
    return end == from ? -1 : value;
}

// The T stage as the loop logged it at stop: n, stage_over5, p50, p99 and max of its PROFILE_T_TIMING
// line, and p10 / p90 of the normal route's PROFILE_STAGE_TIMING line (-1 when missing). lines: how
// many PROFILE_T_TIMING when=stop lines the log has (one per loop run).
struct TStage {
    std::size_t lines = 0;
    double n = -1, over5 = -1, p50 = -1, p99 = -1, max = -1, p10 = -1, p90 = -1;
};
TStage t_stage_of(const std::string& log) {
    TStage t;
    std::istringstream text(log);
    for (std::string line; std::getline(text, line);) {
        if (line.find("PROFILE_T_TIMING when=stop ") != std::string::npos) {
            ++t.lines;
            t.n = field(line, " n=");
            t.over5 = field(line, " stage_over5=");
            t.p50 = field(line, " stage_p50_le_ms=");
            t.p99 = field(line, " stage_p99_le_ms=");
            t.max = field(line, " stage_max_ms=");
        } else if (line.find("PROFILE_STAGE_TIMING when=stop route=normal ") != std::string::npos) {
            t.p10 = field(line, " t_p10_le_ms=");
            t.p90 = field(line, " t_p90_le_ms=");
        }
    }
    return t;
}

// The slow line: S then T (the order of the service's Objects callback), every kSlowPeriodMs, with
// the stamp of the frame offered two frames ago; once told, stuck for kStallMs.
struct SlowLine {
    ProfileOwners& o;
    std::mutex mutex;
    std::vector<geo::CaptureStamp> offered;        // guarded by mutex
    std::atomic<bool> stop{false}, stall{false};
    std::vector<std::int64_t> mask_capture_ns;    // slow thread only until joined
    std::int64_t stall_begin_ns = -1, stall_end_ns = -1, resume_ns = -1;
    std::uint64_t t_published_at_stall = 0, t_published_after_stall = 0, s_published_at_stall = 0,
        s_published_after_stall = 0;
    explicit SlowLine(ProfileOwners& owners) : o(owners) {
        offered.reserve(4096);
        mask_capture_ns.reserve(4096);
    }
    void run() {
        const rns::Objects empty;
        auto next = std::chrono::steady_clock::now();
        bool stalled = false;
        while (!stop.load(std::memory_order_acquire)) {
            next += std::chrono::milliseconds(kSlowPeriodMs);
            std::this_thread::sleep_until(next);
            if (stall.load(std::memory_order_acquire) && !stalled) {
                stalled = true;
                t_published_at_stall = o.t_slot.counters().published;
                s_published_at_stall = o.s_slot.published();
                stall_begin_ns = mono_ns();
                std::this_thread::sleep_for(std::chrono::milliseconds(kStallMs));
                stall_end_ns = mono_ns();
                t_published_after_stall = o.t_slot.counters().published;
                s_published_after_stall = o.s_slot.published();
                next = std::chrono::steady_clock::now();
            }
            geo::CaptureStamp stamp;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (offered.size() < 3) continue;
                stamp = offered[offered.size() - 3];
            }
            if (stalled && resume_ns < 0) resume_ns = mono_ns();  // before the first publication after it
            o.s_slot.publish(empty, stamp, svc::kSemanticBaseRevision);
            if (o.t_slot.publish(o.mask.data(), 1280, 800, 1280, stamp, svc::kTProjectionRevision))
                mask_capture_ns.push_back(stamp.capture_mono_ns);
        }
    }
};

// One run of the service's steady loop on its own thread; the exception text, if any.
struct FastLine {
    ProfileOwners& o;
    std::atomic<bool> finish{false};
    std::string error;
    bool returned = false;
    explicit FastLine(ProfileOwners& owners) : o(owners) {}
    void run() {
        try {
            svc::run_profile_steady(finish, o.inputs(), o.counters);
            returned = true;
        } catch (const std::exception& e) {
            error = e.what();
        }
    }
};

bool wait_ready(const ProfileOwners& o) {
    for (int i = 0; i < 500 && !o.t_ready.load(std::memory_order_acquire); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return o.t_ready.load(std::memory_order_acquire);
}

// The step of the running loop, closed-loop: the next frame as soon as the previous one is put, so
// the thread never idles and the CPU keeps its working clock. A Busy mailbox (the consumer held it
// for a moment) is offered again: this camera loses no frame. Fresh: S and T snapshots of the very
// frame published right after it is offered (a step that took the frame first uses the previous
// frame's, still fresh); stale: none, after seg_max_age. put - host ready per frame is the step
// plus the doorbell wake-up.
struct StepCost {
    std::vector<double> ms;
    std::size_t backed = 0, seg_null = 0;
    bool stuck = false;  // a frame not put within 2 s
};
StepCost measure_step(ProfileOwners& o, std::size_t frames, bool fresh) {
    StepCost c;
    const rns::Objects empty;
    if (!fresh) std::this_thread::sleep_for(std::chrono::milliseconds(kSegMaxAgeMs + 50));  // the last mask goes stale
    for (std::size_t k = 0; k < frames; ++k) {
        const std::size_t before = o.transport.size();
        geo::CaptureStamp stamp;
        nd::Offer offer = nd::Offer::Busy;
        for (int i = 0; i < 1000 && offer == nd::Offer::Busy; ++i) {
            offer = o.offer(mono_ns(), kCaptureLagMs * kMs, stamp);
            if (offer == nd::Offer::Busy) std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        if (offer != nd::Offer::Accepted && offer != nd::Offer::Replaced) { c.stuck = true; break; }
        if (fresh) {
            o.s_slot.publish(empty, stamp, svc::kSemanticBaseRevision);
            (void)o.t_slot.publish(o.mask.data(), 1280, 800, 1280, stamp, svc::kTProjectionRevision);
        }
        // Wait for this frame's put (0.2 ms polls), at most 2 s.
        for (int i = 0; o.transport.size() == before; ++i) {
            if (i == 10000) { c.stuck = true; break; }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        if (c.stuck) break;
        const auto& r = o.transport[o.transport.size() - 1];
        c.ms.push_back(static_cast<double>(r.put_end_ns - (r.capture_ms + kCaptureLagMs) * kMs) / 1e6);
        c.backed += r.bit0_bins > 0;
        c.seg_null += r.seg_ms == -1;
    }
    return c;
}

int time2() {
    ProfileOwners o(kSegMaxAgeMs * kMs, kObjectsStaleMs * kMs, kPublishGuardMs * kMs);
    o.light_floor();
    SlowLine slow(o);
    FastLine fast(o);
    // Only the loop's logger writes to stdout until the loop is joined (G8 reads its stop group).
    std::ostringstream loop_log;
    auto* const saved_cout = std::cout.rdbuf(loop_log.rdbuf());
    std::thread fast_thread([&] { fast.run(); });
    const bool ready = wait_ready(o);
    // G0: can this machine show whether the fast line follows the camera at all?
    const StepCost calibration = measure_step(o, kCalibrationFrames, true);
    const double step_p50 = quantile(calibration.ms, .5), step_p90 = quantile(calibration.ms, .9);
    std::thread slow_thread([&] { slow.run(); });
    // The camera, on this thread: absolute deadlines, so a late frame does not shift the others.
    std::vector<std::int64_t> offer_ns;
    offer_ns.reserve(4096);
    const auto t0 = std::chrono::steady_clock::now();
    const std::int64_t t0_ns = mono_ns();
    for (std::int64_t k = 0;; ++k) {
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(kCameraPeriodMs * k));
        const std::int64_t now = mono_ns();
        if (now - t0_ns >= kStallAtMs * kMs) slow.stall.store(true, std::memory_order_release);
        if (now - t0_ns >= kEndMs * kMs) break;
        geo::CaptureStamp stamp;
        const auto offer = o.offer(now, kCaptureLagMs * kMs, stamp);
        if (offer == nd::Offer::Accepted || offer == nd::Offer::Replaced) {
            std::lock_guard<std::mutex> lock(slow.mutex);
            slow.offered.push_back(stamp);
            offer_ns.push_back(now);
        }
    }
    fast.finish.store(true, std::memory_order_release);
    fast_thread.join();
    std::cout.rdbuf(saved_cout);
    std::cout << loop_log.str();
    slow.stop.store(true, std::memory_order_release);
    slow_thread.join();

    // ---- Phases by put time: before [warm-up, stall begin), during [stall begin, stall end). ----
    const auto& rec = o.transport;
    const std::int64_t pre_begin = t0_ns + kWarmupMs * kMs, pre_end = slow.stall_begin_ns;
    const std::int64_t stall_begin = slow.stall_begin_ns, stall_end = slow.stall_end_ns;
    const auto offered_in = [&](std::int64_t a, std::int64_t b) {
        std::size_t n = 0;
        for (const auto t : offer_ns) n += t >= a && t < b;
        return n;
    };
    const auto latency_ms = [](const PutRecord& r) {
        return static_cast<double>(r.put_end_ns - (r.capture_ms + kCaptureLagMs) * kMs) / 1e6;
    };
    std::size_t pre_n = 0, stall_n = 0, pre_backed = 0, seg_age_over = 0, must_null = 0, must_null_bad = 0;
    std::vector<double> pre_latency, stall_latency, step_free;
    std::int64_t last_mask_before_stall = -1;
    for (const auto c : slow.mask_capture_ns)
        if (c < stall_begin) last_mask_before_stall = std::max(last_mask_before_stall, c);
    std::int64_t first_backed_after_resume = -1;
    for (std::size_t i = 0; i < rec.size(); ++i) {
        const auto& r = rec[i];
        const bool backed = r.seg_ms >= 0 && r.bit0_bins > 0;
        if (r.seg_ms >= 0 && r.capture_ms - r.seg_ms > kSegMaxAgeMs) ++seg_age_over;
        if (r.put_begin_ns >= pre_begin && r.put_begin_ns < pre_end) {
            ++pre_n;
            pre_backed += backed;
            pre_latency.push_back(latency_ms(r));
        }
        if (r.put_begin_ns >= stall_begin && r.put_begin_ns < stall_end) {
            ++stall_n;
            stall_latency.push_back(latency_ms(r));
        }
        // G3: the frame is more than seg_max_age (+1 ms of millisecond truncation) newer than the
        // last mask, and was put before the slow line published again.
        if (last_mask_before_stall >= 0 && r.capture_ms * kMs > last_mask_before_stall + (kSegMaxAgeMs + 1) * kMs &&
            slow.resume_ns >= 0 && r.put_begin_ns < slow.resume_ns) {
            ++must_null;
            must_null_bad += r.seg_ms != -1 || r.bit0_bins != 0;
        }
        if (slow.resume_ns >= 0 && r.put_begin_ns >= slow.resume_ns && r.seg_ms >= 0 && first_backed_after_resume < 0)
            first_backed_after_resume = r.put_begin_ns;
    }
    const std::size_t pre_offered = offered_in(pre_begin, pre_end), stall_offered = offered_in(stall_begin, stall_end);
    const double pre_s = static_cast<double>(pre_end - pre_begin) / 1e9, stall_s = static_cast<double>(stall_end - stall_begin) / 1e9;
    const double pre_rate = pre_s > 0 ? static_cast<double>(pre_n) / pre_s : 0;
    const double stall_rate = stall_s > 0 ? static_cast<double>(stall_n) / stall_s : 0;
    const double pre_follow = pre_offered ? static_cast<double>(pre_n) / static_cast<double>(pre_offered) : 0;
    const double stall_follow = stall_offered ? static_cast<double>(stall_n) / static_cast<double>(stall_offered) : 0;
    const double extra_latency = median(stall_latency) - median(pre_latency);
    const double must_null_min = 0.9 * static_cast<double>(kStallMs - kSegMaxAgeMs - 2 * kCameraPeriodMs) /
                                 static_cast<double>(kCameraPeriodMs);
    const std::int64_t resume_bound = (2 * kSlowPeriodMs + 2 * kCameraPeriodMs) * kMs;
    const auto t = o.t_slot.counters();
    const TStage ts = t_stage_of(loop_log.str());
    const double t_counted = static_cast<double>(o.counters.t_fresh.load() + o.counters.t_missing.load() +
        o.counters.t_stale.load() + o.counters.t_geometry_unavailable.load() + o.counters.t_other.load());

    const bool g0 = !calibration.stuck && calibration.ms.size() == kCalibrationFrames &&
        step_p90 <= kMaxStepShare * static_cast<double>(kCameraPeriodMs);
    const bool g1 = pre_follow >= kMinFollow;
    const bool g2 = stall_rate >= kMinRateKept * pre_rate && stall_follow >= kMinFollow;
    // Both ways: a fast line waiting only while its masks are fresh shows as a higher median before
    // the stall; with a light step such a wait need not cost a frame (it fits into P).
    const bool g2b = !pre_latency.empty() && !stall_latency.empty() && extra_latency <= kMaxExtraLatencyMs &&
        extra_latency >= -kMaxExtraLatencyMs;
    const bool g3 = must_null_bad == 0 && static_cast<double>(must_null) >= must_null_min;
    const bool g4 = seg_age_over == 0;
    const bool g5 = pre_n > 0 && static_cast<double>(pre_backed) >= kMinBacked * static_cast<double>(pre_n);
    const bool g6 = first_backed_after_resume >= 0 && first_backed_after_resume - slow.resume_ns <= resume_bound;
    const bool g7 = ready && stall_begin > 0 && stall_end - stall_begin >= kStallMs * kMs &&
        slow.t_published_after_stall == slow.t_published_at_stall &&
        slow.s_published_after_stall == slow.s_published_at_stall && fast.returned && fast.error.empty() &&
        o.counters.abnormal.load() == 0 && o.counters.puts.load() == rec.size() && rec.overflow() == 0 &&
        t.torn_giveup == 0 && t.refused_future == 0 && t.refused_order == 0;
    // The loop's own T line of this run (its n is the loop's T outcomes), few T stages over 5 ms.
    const bool g8 = ts.lines == 1 && ts.n > 0 && ts.n == t_counted && ts.over5 >= 0 &&
        ts.over5 <= kMaxTOver5Share * ts.n;

    std::cout << "time2 step (light floor, closed loop, fresh, " << calibration.ms.size() << " frames) p50="
              << step_p50 << "ms p90=" << step_p90 << "ms (G0 max " << kMaxStepShare * static_cast<double>(kCameraPeriodMs)
              << "ms) stuck=" << calibration.stuck << "\n";
    std::cout << "time2 P=" << kCameraPeriodMs << "ms slow=" << kSlowPeriodMs << "ms offered=" << offer_ns.size()
              << " puts=" << rec.size() << " masks=" << slow.mask_capture_ns.size()
              << " stall_s=" << stall_s << " resume_after_stall_ms=" << static_cast<double>(slow.resume_ns - stall_end) / 1e6 << "\n"
              << "  rate_hz pre=" << pre_rate << " stall=" << stall_rate << "  follow pre=" << pre_follow
              << " (" << pre_n << "/" << pre_offered << ") stall=" << stall_follow << " (" << stall_n << "/" << stall_offered << ")\n"
              << "  latency_ms median pre=" << median(pre_latency) << " stall=" << median(stall_latency)
              << " extra=" << extra_latency << " p90 pre=" << quantile(pre_latency, .9) << " stall=" << quantile(stall_latency, .9) << "\n"
              << "  t_seg_null_required=" << must_null << " (min " << must_null_min << ") violations=" << must_null_bad
              << " seg_age_over_300=" << seg_age_over << " pre_backed=" << pre_backed << "/" << pre_n
              << " first_backed_after_resume_ms="
              << (first_backed_after_resume < 0 ? -1.0 : static_cast<double>(first_backed_after_resume - slow.resume_ns) / 1e6)
              << " (max " << static_cast<double>(resume_bound) / 1e6 << ")\n"
              << "  self: ready=" << ready << " returned=" << fast.returned << " error=\"" << fast.error << "\" abnormal="
              << o.counters.abnormal.load() << " counted_puts=" << o.counters.puts.load() << " t_fresh=" << o.counters.t_fresh.load()
              << " t_stale=" << o.counters.t_stale.load() << " slot_torn_giveup=" << t.torn_giveup
              << " slot_refused_future=" << t.refused_future << "\n"
              << "  t_stage_ms (grid end -> T end, the loop's PROFILE_T_TIMING when=stop): n=" << ts.n << " (T outcomes "
              << t_counted << ") over5=" << ts.over5 << " (max " << kMaxTOver5Share * ts.n << ") p10<=" << ts.p10
              << " p50<=" << ts.p50 << " p90<=" << ts.p90 << " p99<=" << ts.p99 << " max=" << ts.max << "\n";
    const char* names[] = {"G0", "G1", "G2", "G2b", "G3", "G4", "G5", "G6", "G7", "G8"};
    const bool gates[] = {g0, g1, g2, g2b, g3, g4, g5, g6, g7, g8};
    std::string failed;
    for (std::size_t i = 0; i < 10; ++i) {
        std::cout << (i ? " " : "  ") << names[i] << "=" << (gates[i] ? "pass" : "FAIL");
        if (!gates[i]) failed += std::string(failed.empty() ? "" : ",") + names[i];
    }
    std::cout << "\n";
    if (!g0)
        std::cout << "  G0: one step took too long (p90 " << step_p90 << " ms for P=" << kCameraPeriodMs
                  << " ms): a busy or slow machine, or a fast line that waits (the steps were measured "
                  << "through the same adapters); compare --step-cost of an unchanged build on free cores\n";
    if (!g8)
        std::cout << "  G8: " << (ts.lines != 1 || ts.n != t_counted
            ? "the loop's PROFILE_T_TIMING when=stop line was not read exactly once for this run: the loop did not "
              "reach its stop group (an exception, see G7) or the logger is asynchronous (PERCEPTION_ASYNC_LOG set)"
            : "more than 10 % of the T stages took over 5 ms: a wait in the T access (adapter, slot) or a T owner "
              "grown slower; compare the PROFILE_T_TIMING when=stop line of --step-cost on an unchanged build") << "\n";
    if (!g1)
        std::cout << "  G1: before the stall the fast line did not keep up with the camera (" << pre_follow
                  << "): either this machine is too busy or slow for P=" << kCameraPeriodMs
                  << " ms (see G0 and --step-cost), or the fast line is paced by something else\n";
    if (failed.empty()) {
        std::cout << "A19-TIME-2 host PASS: the stalled slow line left the Profile rate unchanged and t_seg/bit0 dropped\n";
        return 0;
    }
    std::cout << "A19-TIME-2 host FAIL " << failed << "\n";
    return 1;
}

// How a fresh-mask step p90 sits against a camera period: a frame is lost only when a step plus
// its wake-up outlasts the period, and G1 allows 10 % of them; half the period leaves room for a
// busy machine, three quarters is the least that is still workable.
const char* verdict(double p90_ms, std::int64_t period_ms) {
    if (p90_ms < 0) return "no data";
    if (p90_ms <= 0.5 * static_cast<double>(period_ms)) return "comfortable (p90 <= P/2)";
    if (p90_ms <= 0.75 * static_cast<double>(period_ms)) return "workable (p90 <= 3P/4)";
    return "too slow (p90 > 3P/4)";
}

// --step-cost: measure_step() alone, with fresh snapshots and then stale ones, for the light floor
// (the one the test uses) and then for the full floor; the loop's own PROFILE_STAGE_TIMING
// when=stop lines (printed by the logger) have the step's wall and, on Linux, CPU quantiles.
int step_cost() {
    constexpr std::size_t kFrames = 80;
    int failures = 0;
    double light_p90 = -1;
    for (const bool light : {true, false}) {
        ProfileOwners o(kSegMaxAgeMs * kMs, kObjectsStaleMs * kMs, kPublishGuardMs * kMs);
        if (light) o.light_floor();
        FastLine fast(o);
        std::thread fast_thread([&] { fast.run(); });
        StepCost fresh, stale;
        if (wait_ready(o)) {
            fresh = measure_step(o, kFrames, true);
            if (!fresh.stuck) stale = measure_step(o, kFrames, false);
        } else {
            fresh.stuck = true;
        }
        fast.finish = true;
        fast_thread.join();
        const bool ok = !fresh.stuck && !stale.stuck && fast.error.empty() && fresh.ms.size() == kFrames &&
                        stale.ms.size() == kFrames;
        failures += !ok;
        if (light) light_p90 = quantile(fresh.ms, .9);
        std::cout << (light ? "light floor" : "full floor") << " (put - host ready, ms; the test target builds at -O2):\n"
                  << "  fresh mask: n=" << fresh.ms.size() << " backed=" << fresh.backed << " p50="
                  << quantile(fresh.ms, .5) << " p90=" << quantile(fresh.ms, .9) << " max=" << quantile(fresh.ms, 1)
                  << "\n  stale mask: n=" << stale.ms.size() << " t_seg_null=" << stale.seg_null << " p50="
                  << quantile(stale.ms, .5) << " p90=" << quantile(stale.ms, .9) << " max=" << quantile(stale.ms, 1)
                  << "\n  loop: error=\"" << fast.error << "\" abnormal=" << o.counters.abnormal.load()
                  << " stuck=" << (fresh.stuck || stale.stuck) << (ok ? "" : " INCOMPLETE") << "\n";
    }
    std::cout << "verdict (light floor, fresh p90 " << light_p90 << " ms) for P=33 / slow 50: " << verdict(light_p90, 33)
              << "; for P=50 / slow 75: " << verdict(light_p90, 50) << "; this build: P=" << kCameraPeriodMs
              << ", G0 needs p90 <= " << kMaxStepShare * static_cast<double>(kCameraPeriodMs) << " ms\n";
    return failures ? 1 : 0;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--step-cost") return step_cost();
    if (argc != 1) {
        std::cerr << "usage: test_rns_profile_time2 [--step-cost]\n";
        return 2;
    }
    return time2();
}
