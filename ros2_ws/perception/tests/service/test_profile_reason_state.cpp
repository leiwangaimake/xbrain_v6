/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_profile_reason_state.cpp
 * Brief: Host test (reply batch 3, B3-3): frame-level Profile reasons through the Status owner's ReasonEdges.
 *
 * Description:
 * The four required cases: A failure -> heartbeat -> no new frame: no pseudo recovery; B persistent failure across heartbeats: never withdrawn and re-raised; C failure then normal frame in the same window: real recovery edge, window history still shows the failure; D a failed heartbeat loses no window ledger and claims no recovery. Plus the regression shape of the replaced implementation (clear-on-beat).
 */

#include "service/profile_reason_state.hpp"

#include "output/rns_reason_edges.hpp"

#include <iostream>

using perception::rns::ReasonEdge;
using perception::rns::ReasonEdges;
using perception::rns::ReasonEdgeSeverity;
using perception::service::ProfileReasonState;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

constexpr std::uint64_t kFixed = 1U << 0;      // e.g. t_capture_estimated
constexpr std::uint64_t kFitFallback = 1U << 1;
constexpr std::uint64_t kWithdrawn = 1U << 2;
constexpr std::uint64_t kFrameBits = kFitFallback | kWithdrawn;

// Status owner loop body as in the service: 10 ms polls, heartbeat each 1000 ms.
struct StatusSim {
    ProfileReasonState& state;
    ReasonEdges edges{0, 8, 64};
    ProfileReasonState::Snapshot window_start;
    std::int64_t now = 0;
    std::size_t warns = 0, infos = 0;
    std::uint64_t window_fit_frames = 0, window_processed = 0;
    explicit StatusSim(ProfileReasonState& s) : state(s), window_start(s.snapshot()) {}
    void poll() {
        now += 10;
        const auto active = kFixed | state.current();
        (void)edges.update(active, now);
        ReasonEdge e;
        while (edges.pop(e)) (e.severity == ReasonEdgeSeverity::Warn ? warns : infos) += 1;
    }
    // Heartbeat: the window ledger is taken only when the beat is published.
    void heartbeat(bool published) {
        if (!published) return;
        const auto s = state.snapshot();
        window_fit_frames = s.frames_with[1] - window_start.frames_with[1];
        window_processed = s.processed_frames - window_start.processed_frames;
        window_start = s;
    }
};
}  // namespace

int main() {
    // ---- A: failure -> heartbeat -> no new frame ----
    {
        ProfileReasonState state(kFrameBits);
        StatusSim sim(state);
        sim.poll();  // fixed reason appears
        CHECK(sim.warns == 1 && sim.infos == 0);
        CHECK(state.commit_frame(kFitFallback | kWithdrawn));
        sim.poll();
        CHECK(sim.warns == 2);
        for (int beat = 0; beat < 5; ++beat) {
            for (int i = 0; i < 100; ++i) sim.poll();
            sim.heartbeat(true);
        }
        CHECK(sim.infos == 0 && (sim.edges.active() & kFrameBits) == kFrameBits);
        CHECK(sim.window_processed == 0 && sim.window_fit_frames == 0);  // later windows saw no frames
    }
    // ---- B: persistent failure across heartbeats ----
    {
        ProfileReasonState state(kFrameBits);
        StatusSim sim(state);
        for (int beat = 0; beat < 30; ++beat) {
            for (int i = 0; i < 100; ++i) {
                if (i % 3 == 0) CHECK(state.commit_frame(kFitFallback | kWithdrawn));
                sim.poll();
            }
            sim.heartbeat(true);
            CHECK(sim.window_fit_frames == 34 && sim.window_processed == 34);
        }
        CHECK(sim.warns == 1 && sim.infos == 0);  // fixed + frame reasons raised together once, never withdrawn
    }
    // ---- C: failure then normal frame in the same window ----
    {
        ProfileReasonState state(kFrameBits);
        StatusSim sim(state);
        sim.poll();
        CHECK(state.commit_frame(kFitFallback | kWithdrawn));
        sim.poll();
        for (int i = 0; i < 40; ++i) sim.poll();
        CHECK(state.commit_frame(0));  // explicit recovery evidence
        sim.poll();
        CHECK(sim.infos == 1 && (sim.edges.active() & kFrameBits) == 0);
        for (int i = 0; i < 57; ++i) sim.poll();
        sim.heartbeat(true);
        CHECK(sim.window_processed == 2 && sim.window_fit_frames == 1);  // history kept separately
        CHECK(state.current() == 0);
    }
    // ---- D: failed heartbeat keeps the window ledger and claims nothing ----
    {
        ProfileReasonState state(kFrameBits);
        StatusSim sim(state);
        sim.poll();
        CHECK(state.commit_frame(kFitFallback | kWithdrawn));
        for (int i = 0; i < 100; ++i) sim.poll();
        sim.heartbeat(false);  // publish failed: no window taken
        CHECK(sim.infos == 0 && state.current() == (kFitFallback | kWithdrawn));
        CHECK(state.commit_frame(kFitFallback | kWithdrawn));
        for (int i = 0; i < 100; ++i) sim.poll();
        sim.heartbeat(true);
        CHECK(sim.window_fit_frames == 2 && sim.window_processed == 2);  // nothing lost across the failed beat
        CHECK(sim.infos == 0);
    }
    // ---- refusal and regression shape ----
    {
        ProfileReasonState state(kFrameBits);
        CHECK(!state.commit_frame(1U << 5) && state.snapshot().processed_frames == 0);
        // Replaced behaviour: OR since last beat, cleared at each beat. With a
        // failure followed by NO frames, the clear produced a recovery edge.
        ReasonEdges edges(0, 8, 64);
        std::uint64_t pending = kFitFallback;
        std::size_t infos = 0;
        ReasonEdge e;
        for (int t = 10; t <= 3000; t += 10) {
            (void)edges.update(kFixed | pending, t);
            while (edges.pop(e)) infos += e.severity == ReasonEdgeSeverity::Info;
            if (t % 1000 == 0) pending = 0;  // clear-on-beat
        }
        CHECK(infos == 1);  // the defect the new state removes (case A shows 0)
    }
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"profile_reason_state_host\"}" << std::endl;
    return failures ? 1 : 0;
}
