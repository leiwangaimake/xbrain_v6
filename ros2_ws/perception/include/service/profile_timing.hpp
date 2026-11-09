/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: profile_timing.hpp
 * Brief: The Profile owner thread's latency ledger, the figures frozen from it, and their log lines.
 *
 * Description:
 * Both Profile loops of the service (profile_steady_loop.cpp) keep one ProfileTiming. Moved
 * unchanged out of perception_service_main.cpp on 2026-10-14 (A19-TIME-2); only the rns::
 * prefixes the service file's using-directive used to supply were added. No Zenoh type: the put
 * audit is the plain struct of output/rns_native_put_audit.hpp.
 * Since 2026-10-14 (second change of that day) the ledger no longer logs itself. Every 10 s and at
 * stop the Profile thread freezes what the lines print into ProfileTimingFigures (a fixed copy,
 * quantiles computed there as before), and the lines are formatted from the figures by another
 * thread (service/profile_timing_handoff.hpp): the fast line formats and logs nothing in its loop
 * (19 P19-3 / A19-ALLOC-1). The four line kinds are byte for byte what ProfileTiming::log()
 * printed; PROFILE_TIMING_OBSERVED, in front of each group, is new.
 */
#pragma once

#include "output/rns_native_profile.hpp"
#include "output/rns_native_put_audit.hpp"
#include "service/latency_histogram.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace perception::service {

// Owner-thread latency ledger. sdk_frameset_wait_return_to_profile_put_return:
// host MONOTONIC sample right after the SDK complete RGB-D FrameSet wait returns
// -> native put return. NOT asserted equal to 19 section 2.2 G-P1 ("depth
// callback return"): earlier waiting inside/before the FrameSet is not covered
// (reply batch 3, B3-5). put is the native z_publisher_put bracket. processing
// is frame take -> body built.
struct ProfileTiming {
    static constexpr std::int64_t kMs = 1000000;
    service::LatencyHistogram gp1{100000, 1000, {14 * kMs, 60 * kMs}};
    service::LatencyHistogram put{10000, 1000, {1 * kMs}};
    service::LatencyHistogram processing{100000, 1000, {14 * kMs}};
    // Thread CPU time of the owner thread across the step that processed the
    // frame (includes that step's mailbox take, not any wait). Wall minus CPU
    // is not by itself a measure of contention.
    service::LatencyHistogram processing_cpu{100000, 1000, {14 * kMs}};
    // S snapshot ages (19 section 3.4A), only with an S owner:
    //   capture = depth capture - object capture (the inflation term)
    //   check   = the staleness gate input (check instant - object capture)
    // The thresholds are the configured objects_stale_ms and twice it, so the
    // log says how far the real pipeline sits from the configured bound.
    std::optional<service::LatencyHistogram> semantic_age_capture, semantic_age_check;
    std::uint64_t semantic_age_unavailable = 0;
    // S check instant -> publish entry, the interval the publish guard has to cover: over all
    // frames that reached the publish entry, and over those whose snapshot was Fresh at the check.
    std::optional<service::LatencyHistogram> semantic_check_to_publish, semantic_fresh_check_to_publish;
    // The uniform footprint margin the projector actually applied (19 section
    // 3.4A: (v_ego_max + omega_max*r_far)*(t_capture - t_obj) + m_jitter).
    double semantic_margin_max_m = 0, semantic_margin_sum_m = 0;
    std::uint64_t semantic_margin_n = 0;
    std::uint64_t audit_mismatch = 0, cpu_unavailable = 0;
    // Only with a T owner (plan 2f): the T step duration (grid end -> T end: slot
    // copy, mask check, projection, erosion) and the mask age at capture.
    std::optional<service::LatencyHistogram> t_stage, t_age;
    void enable_t(std::int64_t max_age_ns) {
        t_stage.emplace(100000, 1000, std::vector<std::int64_t>{5 * kMs, 14 * kMs});
        t_age.emplace(100000, 10000, std::vector<std::int64_t>{max_age_ns, 2 * max_age_ns});
    }
    void observe_t(std::int64_t stage_ns, std::int64_t age_ns) noexcept {
        if (!t_stage) return;
        if (stage_ns >= 0) t_stage->add(stage_ns);
        if (age_ns >= 0) t_age->add(age_ns);
    }
    // Only with an S owner: the two S age histograms.
    void enable_semantic(std::int64_t stale_ns) {
        semantic_age_capture.emplace(100000, 10000, std::vector<std::int64_t>{stale_ns, 2 * stale_ns});
        semantic_age_check.emplace(100000, 10000, std::vector<std::int64_t>{stale_ns, 2 * stale_ns});
        semantic_check_to_publish.emplace(100000, 1000, std::vector<std::int64_t>{5 * kMs, 15 * kMs});
        semantic_fresh_check_to_publish.emplace(100000, 1000, std::vector<std::int64_t>{5 * kMs, 15 * kMs});
    }
    void observe_semantic_publish(std::int64_t check_ns, std::int64_t publish_entry_ns, bool fresh) noexcept {
        if (!semantic_check_to_publish || check_ns < 0 || publish_entry_ns < 0) return;
        semantic_check_to_publish->add(publish_entry_ns - check_ns);
        if (fresh) semantic_fresh_check_to_publish->add(publish_entry_ns - check_ns);
    }
    void observe_semantic_age(std::int64_t at_capture_ns, std::int64_t at_check_ns, double margin_m) noexcept {
        if (!semantic_age_capture) return;
        if (at_capture_ns < 0 || at_check_ns < 0) { ++semantic_age_unavailable; return; }
        semantic_age_capture->add(at_capture_ns);
        semantic_age_check->add(at_check_ns);
        if (margin_m > 0) {
            semantic_margin_max_m = std::max(semantic_margin_max_m, margin_m);
            semantic_margin_sum_m += margin_m;
            ++semantic_margin_n;
        }
    }
    // One dispatched frame: FrameSet-return -> put-return, put duration and
    // processing time, only when the put audit really belongs to this frame.
    void observe(std::int64_t host_ready_ns, std::int64_t taken_ns, std::int64_t build_end_ns,
                 const rns::NativePutAudit& a) noexcept {
        if (!a.clock_valid || a.put_begin_ns < build_end_ns || a.put_end_ns < a.put_begin_ns || host_ready_ns <= 0) {
            ++audit_mismatch;  // not this frame's native put bracket: never guessed
            return;
        }
        gp1.add(a.put_end_ns - host_ready_ns);
        put.add(a.put_end_ns - a.put_begin_ns);
        processing.add(build_end_ns - taken_ns);
    }
    void observe_cpu(std::int64_t cpu_ns) noexcept {
        if (cpu_ns < 0) ++cpu_unavailable;
        else processing_cpu.add(cpu_ns);
    }
    // Wall time of each stage of a published frame, from the consumer's own stamps (2026-10-13,
    // diagnostic; 10 us bins up to 100 ms), kept apart per route, because the routes run different
    // stages: normal, no ground reference (no FREE, still a grid) and failed fit (withdrawal, no
    // grid). The frames counted are the ones PROFILE_TIMING counts: put finished and attempted.
    // Wall, not CPU: a stage during which the thread was preempted is longer than its work. For
    // the whole step the thread CPU time is kept beside it (cpu): step wall minus cpu at the same
    // quantile is the waiting. A stage's low quantile (p10) is the one to hold against the
    // isolated bench (perception_geometry_bench), and it is an upper bound of the work, not the
    // work; p50 and p90 show where the frame's waiting falls. take: mailbox take; health:
    // depth-health scan and capture stamp; geometry: ground fit and radial geometry; grid; t and
    // semantic only with their owners; build: body; publish: publish entry to the end of the step
    // (encode, put, receipt); step: all.
    static constexpr std::size_t kStages = 9, kRouteGroups = 3, kPerGroup = kStages + 1;
    static constexpr std::array<const char*, kStages> kStageNames{
        "take", "health", "geometry", "grid", "t", "semantic", "build", "publish", "step"};
    static constexpr std::array<const char*, kRouteGroups> kRouteGroupNames{"normal", "no_ground", "fit_failed"};
    // Index: route group * kPerGroup + stage; the last one of a group is the step's thread CPU.
    // Empty until enable_stages() (the unconfigured loop has no such stages).
    std::vector<service::LatencyHistogram> stages;
    void enable_stages() {
        stages.assign(kPerGroup * kRouteGroups, service::LatencyHistogram{10000, 10000});
    }
    void observe_stages(const rns::NativeProfileTick& t, std::int64_t cpu_ns) noexcept {
        if (stages.empty()) return;
        const std::size_t group = t.route == rns::NativeProfileRoute::Normal ? 0
            : t.route == rns::NativeProfileRoute::NoGroundReference ? 1 : 2;
        // -1 (a stage that did not run, or stamps out of order) is not a sample.
        const auto span = [](std::int64_t from, std::int64_t to) { return from >= 0 && to >= from ? to - from : -1; };
        const std::int64_t before_build = t.semantic_end_ns >= 0 ? t.semantic_end_ns
            : t.t_end_ns >= 0 ? t.t_end_ns : t.grid_end_ns >= 0 ? t.grid_end_ns : t.geometry_end_ns;
        const std::array<std::int64_t, kPerGroup> ns{
            span(t.begin_ns, t.taken_ns), span(t.taken_ns, t.capture_checked_ns),
            span(t.capture_checked_ns, t.geometry_end_ns), span(t.geometry_end_ns, t.grid_end_ns),
            t.t_ran ? span(t.grid_end_ns, t.t_end_ns) : -1,
            span(t.semantic_check_ns, t.semantic_end_ns), span(before_build, t.build_end_ns),
            span(t.build_end_ns, t.end_ns), span(t.begin_ns, t.end_ns), cpu_ns};
        for (std::size_t i = 0; i < kPerGroup; ++i)
            if (ns[i] >= 0) stages[group * kPerGroup + i].add(ns[i]);
    }
};

// When a group of timing lines was frozen, and by which of the two Profile loops.
enum class ProfileTimingWhen : std::uint8_t { Periodic, Stop };
enum class ProfileTimingOwner : std::uint8_t { Configured, Unconfigured };

// What one group of the timing lines prints, frozen by the Profile thread at the instant the
// lines used to be logged: the counts, threshold counts, maxima and quantile upper edges (ns; -1
// for none, printed as -1) of ProfileTiming's histograms, and its scalars. Cumulative since the
// loop started, exactly as the ledger itself (not a 10 s window). Fixed size, no pointer into the
// ledger: whoever formats it never reads the live histograms (service/profile_timing_handoff.hpp).
struct ProfileTimingFigures {
    // One histogram of PROFILE_TIMING, PROFILE_SEMANTIC_AGE or PROFILE_T_TIMING: n, the two
    // threshold counts, max, and the quantiles its line prints (the others stay -1).
    struct Histogram {
        std::uint64_t n = 0, over0 = 0, over1 = 0;
        std::int64_t p50 = -1, p99 = -1, max = -1;
    };
    // One stage (or the step's CPU) of PROFILE_STAGE_TIMING.
    struct Stage {
        std::uint64_t n = 0;
        std::int64_t p10 = -1, p50 = -1, p90 = -1;
    };
    // The observation (PROFILE_TIMING_OBSERVED): its number among this loop's observations
    // (dropped ones count too), how many were dropped before it, the CLOCK_MONOTONIC instant the
    // Profile thread froze it, that thread, and the loop.
    std::uint64_t seq = 0, dropped = 0;
    std::int64_t observed_mono_ns = -1;
    std::thread::id owner_thread;
    ProfileTimingWhen when = ProfileTimingWhen::Periodic;
    ProfileTimingOwner owner = ProfileTimingOwner::Configured;
    // Which optional lines the ledger has: stage lines (configured loop), S ages, T timing.
    bool stages = false, semantic = false, t = false;
    // PROFILE_TIMING
    Histogram gp1, put, processing, processing_cpu;
    std::uint64_t cpu_unavailable = 0, audit_mismatch = 0;
    // PROFILE_STAGE_TIMING, [route group][stage]; the last of a group is the step's thread CPU.
    std::array<std::array<Stage, ProfileTiming::kPerGroup>, ProfileTiming::kRouteGroups> stage{};
    // PROFILE_SEMANTIC_AGE
    Histogram age_capture, age_check, check_to_publish, fresh_check_to_publish;
    double margin_max_m = 0, margin_sum_m = 0;
    std::uint64_t margin_n = 0, age_unavailable = 0;
    // PROFILE_T_TIMING
    Histogram t_stage, t_age;
};
// One slot of the handoff: 1528 bytes measured on macOS arm64 (clang and GCC 16); plain LP64
// layout, the target's should be the same (the assertion is the bound that matters).
static_assert(sizeof(ProfileTimingFigures) <= 2048, "the frozen timing figures should stay a small fixed copy");

// Fills `f` from `timing` on the ledger's owner thread: the same quantile scans the log lines
// always made there (bounded, no allocation), no formatting. The observation fields are not
// touched (ProfileTimingHandoff sets them).
inline void freeze_profile_timing(const ProfileTiming& timing, ProfileTimingFigures& f) noexcept {
    const auto histogram = [](const service::LatencyHistogram& h, bool p50) {
        ProfileTimingFigures::Histogram x;
        x.n = h.count();
        x.over0 = h.over(0);
        x.over1 = h.over(1);
        if (p50) x.p50 = h.quantile_upper_ns(0.5);
        x.p99 = h.quantile_upper_ns(0.99);
        x.max = h.max_ns();
        return x;
    };
    f.gp1 = histogram(timing.gp1, true);
    f.put = histogram(timing.put, false);
    f.processing = histogram(timing.processing, true);
    f.processing_cpu = histogram(timing.processing_cpu, true);
    f.cpu_unavailable = timing.cpu_unavailable;
    f.audit_mismatch = timing.audit_mismatch;
    f.stages = !timing.stages.empty();
    for (std::size_t group = 0; group < ProfileTiming::kRouteGroups; ++group)
        for (std::size_t i = 0; i < ProfileTiming::kPerGroup; ++i) {
            auto& x = f.stage[group][i];
            x = {};
            if (!f.stages) continue;
            const auto& h = timing.stages[group * ProfileTiming::kPerGroup + i];
            x.n = h.count();
            x.p10 = h.quantile_upper_ns(0.1);
            x.p50 = h.quantile_upper_ns(0.5);
            x.p90 = h.quantile_upper_ns(0.9);
        }
    f.semantic = timing.semantic_age_check.has_value();
    f.age_capture = f.age_check = f.check_to_publish = f.fresh_check_to_publish = {};
    if (f.semantic) {
        f.age_capture = histogram(*timing.semantic_age_capture, true);
        f.age_check = histogram(*timing.semantic_age_check, true);
        f.check_to_publish = histogram(*timing.semantic_check_to_publish, true);
        f.fresh_check_to_publish = histogram(*timing.semantic_fresh_check_to_publish, true);
    }
    f.margin_max_m = timing.semantic_margin_max_m;
    f.margin_sum_m = timing.semantic_margin_sum_m;
    f.margin_n = timing.semantic_margin_n;
    f.age_unavailable = timing.semantic_age_unavailable;
    f.t = timing.t_stage.has_value();
    f.t_stage = f.t_age = {};
    if (f.t) {
        f.t_stage = histogram(*timing.t_stage, true);
        f.t_age = histogram(*timing.t_age, true);
    }
}

// Appends the text of the PROFILE_TIMING, PROFILE_STAGE_TIMING (one per route group with frames),
// PROFILE_SEMANTIC_AGE and PROFILE_T_TIMING lines, in that order: byte for byte what
// ProfileTiming::log() printed until 2026-10-14 for the same ledger (tests/test_rns_profile_timing_log.cpp
// compares them with a frozen copy of it). Allocates; never called on the Profile thread's loop.
inline void append_profile_timing_lines(const ProfileTimingFigures& f, std::vector<std::string>& lines) {
    using Stages = ProfileTiming;
    const auto ms = [](std::int64_t ns) { return ns < 0 ? -1.0 : static_cast<double>(ns) / 1e6; };
    const char* const when = f.when == ProfileTimingWhen::Stop ? "stop" : "periodic";
    {
        // Not labelled G-P1: the start is the SDK FrameSet wait return (complete
        // RGB-D set), whose mapping to "depth callback return" is unconfirmed (B3-5).
        std::ostringstream line;
        line << "PROFILE_TIMING when=" << when << " frameset_return_to_put_return_n=" << f.gp1.n
            << " frameset_return_to_put_return_p50_le_ms=" << ms(f.gp1.p50)
            << " frameset_return_to_put_return_p99_le_ms=" << ms(f.gp1.p99)
            << " frameset_return_to_put_return_max_ms=" << ms(f.gp1.max)
            << " frameset_return_to_put_return_over14=" << f.gp1.over0 << " frameset_return_to_put_return_over60=" << f.gp1.over1
            << " put_n=" << f.put.n << " put_p99_le_ms=" << ms(f.put.p99) << " put_max_ms="
            << ms(f.put.max) << " put_over1ms=" << f.put.over0 << " proc_p50_le_ms=" << ms(f.processing.p50)
            << " proc_p99_le_ms=" << ms(f.processing.p99) << " proc_max_ms=" << ms(f.processing.max)
            << " proc_over14=" << f.processing.over0 << " proc_cpu_n=" << f.processing_cpu.n
            << " proc_cpu_p50_le_ms=" << ms(f.processing_cpu.p50)
            << " proc_cpu_p99_le_ms=" << ms(f.processing_cpu.p99)
            << " proc_cpu_max_ms=" << ms(f.processing_cpu.max) << " proc_cpu_unavailable=" << f.cpu_unavailable
            << " audit_mismatch=" << f.audit_mismatch;
        lines.push_back(line.str());
    }
    // One line per route that had frames; fixed notation, so a parser never meets an exponent.
    for (std::size_t group = 0; group < Stages::kRouteGroups && f.stages; ++group) {
        const auto& h = f.stage[group];
        if (h[Stages::kStages - 1].n == 0) continue;
        std::ostringstream line;
        line << std::fixed << std::setprecision(3);
        line << "PROFILE_STAGE_TIMING when=" << when << " route=" << Stages::kRouteGroupNames[group] << " clock=wall";
        for (std::size_t i = 0; i < Stages::kStages; ++i)
            line << ' ' << Stages::kStageNames[i] << "_n=" << h[i].n
                 << ' ' << Stages::kStageNames[i] << "_p10_le_ms=" << ms(h[i].p10)
                 << ' ' << Stages::kStageNames[i] << "_p50_le_ms=" << ms(h[i].p50)
                 << ' ' << Stages::kStageNames[i] << "_p90_le_ms=" << ms(h[i].p90);
        line << " cpu_n=" << h[Stages::kStages].n << " cpu_p10_le_ms=" << ms(h[Stages::kStages].p10)
             << " cpu_p50_le_ms=" << ms(h[Stages::kStages].p50)
             << " cpu_p90_le_ms=" << ms(h[Stages::kStages].p90);
        lines.push_back(line.str());
    }
    if (f.semantic) {
        std::ostringstream line;
        line << "PROFILE_SEMANTIC_AGE when=" << when
            << " n=" << f.age_check.n
            << " at_capture_p50_le_ms=" << ms(f.age_capture.p50)
            << " at_capture_p99_le_ms=" << ms(f.age_capture.p99)
            << " at_capture_max_ms=" << ms(f.age_capture.max)
            << " at_check_p50_le_ms=" << ms(f.age_check.p50)
            << " at_check_p99_le_ms=" << ms(f.age_check.p99)
            << " at_check_max_ms=" << ms(f.age_check.max)
            << " at_check_over_stale=" << f.age_check.over0
            << " at_check_over_2x_stale=" << f.age_check.over1
            << " margin_n=" << f.margin_n
            << " margin_mean_m=" << (f.margin_n ? f.margin_sum_m / static_cast<double>(f.margin_n) : 0.0)
            << " margin_max_m=" << f.margin_max_m
            << " unavailable=" << f.age_unavailable
            << " check_to_publish_n=" << f.check_to_publish.n
            << " check_to_publish_p50_le_ms=" << ms(f.check_to_publish.p50)
            << " check_to_publish_p99_le_ms=" << ms(f.check_to_publish.p99)
            << " check_to_publish_max_ms=" << ms(f.check_to_publish.max)
            << " check_to_publish_over5=" << f.check_to_publish.over0
            << " check_to_publish_over15=" << f.check_to_publish.over1
            << " fresh_check_to_publish_n=" << f.fresh_check_to_publish.n
            << " fresh_check_to_publish_p50_le_ms=" << ms(f.fresh_check_to_publish.p50)
            << " fresh_check_to_publish_p99_le_ms=" << ms(f.fresh_check_to_publish.p99)
            << " fresh_check_to_publish_max_ms=" << ms(f.fresh_check_to_publish.max);
        lines.push_back(line.str());
    }
    if (f.t) {
        std::ostringstream line;
        line << "PROFILE_T_TIMING when=" << when << " n=" << f.t_stage.n
            << " stage_p50_le_ms=" << ms(f.t_stage.p50)
            << " stage_p99_le_ms=" << ms(f.t_stage.p99)
            << " stage_max_ms=" << ms(f.t_stage.max) << " stage_over5=" << f.t_stage.over0
            << " stage_over14=" << f.t_stage.over1 << " age_n=" << f.t_age.n
            << " age_p50_le_ms=" << ms(f.t_age.p50)
            << " age_p99_le_ms=" << ms(f.t_age.p99)
            << " age_max_ms=" << ms(f.t_age.max) << " age_over_max=" << f.t_age.over0;
        lines.push_back(line.str());
    }
}

// The line in front of each group, new on 2026-10-14: which observation the group is, so the
// moment a background thread logged it (the logger's time and thread) is not taken for the
// moment it was measured. Its tag is not "PROFILE_TIMING when=", so readers of the four lines
// above are not affected.
inline std::string profile_timing_observed_line(const ProfileTimingFigures& f) {
    std::ostringstream line;
    line << "PROFILE_TIMING_OBSERVED when=" << (f.when == ProfileTimingWhen::Stop ? "stop" : "periodic")
         << " seq=" << f.seq << " owner=" << (f.owner == ProfileTimingOwner::Configured ? "configured" : "unconfigured")
         << " owner_thread=" << f.owner_thread << " observed_mono_ms="
         << (f.observed_mono_ns < 0 ? -1 : f.observed_mono_ns / 1000000) << " dropped=" << f.dropped;
    return line.str();
}

}  // namespace perception::service
