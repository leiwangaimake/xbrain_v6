/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: profile_steady_loop.hpp
 * Brief: The steady state of the service's configured Profile loop (the fast line), as a plain function.
 *
 * Description:
 * What problem this solves. 19 A19-TIME-2 asks that a stalled slow line (inference) leaves the
 * fast line's publication rate unchanged, and the bug it is meant to catch ("the fast line
 * quietly waits for the slow one", 19 section 1.1 P19-1) can only sit in three places: the two
 * slot adapters the fast line reads through and the wait between steps. Those used to be inside
 * configured_loop in perception_service_main.cpp, in the anonymous namespace of the file with
 * main(), where no test could reach them. run_profile_steady() is that code, moved unchanged
 * (2026-10-14); the service calls it after its first-frame setup, and
 * tests/test_rns_profile_time2.cpp runs the same function on real threads and a real clock.
 *
 * What stays in the service. The first native frame, the geometry, the S projector, the T colour
 * projection and the T owner (they need the device's metadata, the configuration and the
 * Supervisor's calibration), and the RT transport: the loop reads the put audit through
 * PutAuditReader, so this unit has no Zenoh type.
 * The unconfigured loop (no transform) lives here too since the second change of 2026-10-14, and
 * both loops hand their timing figures to service/profile_timing_handoff.hpp instead of logging
 * them (tests/test_rns_profile_timing_log.cpp counts their heap calls across 10 s marks).
 */
#pragma once

#include "modules/native_depth.hpp"
#include "output/rns_native_profile.hpp"
#include "output/rns_native_put_audit.hpp"
#include "output/rns_unconfigured_profile.hpp"
#include "service/profile_reason_state.hpp"
#include "service/semantic_slot.hpp"
#include "service/t_mask_slot.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace perception::service {

class ProfileTimingHandoff;  // service/profile_timing_handoff.hpp

// The Profile thread's counters, read by the main thread's SERVICE_SUMMARY, SERVICE_T_SUMMARY and
// SERVICE_STOPPED lines (moved from the service's ProfileShared, which now derives from this).
struct ProfileCounters {
    std::atomic<std::uint64_t> puts{0}, accepted{0}, rejected_or_unknown{0}, abnormal{0};
    std::atomic<std::uint64_t> setup_frames{0}, route_normal{0}, route_no_ground{0}, route_withdrawn{0};
    std::atomic<std::uint64_t> clock_rejected{0}, geometry_unavailable{0}, grid_unavailable{0}, body_unavailable{0};
    // Per-frame S outcome. no_ground_refused counts NoGroundReference frames that
    // were NOT dispatched; since reply B6-2 such frames are published, so a
    // non-zero value is worth a look.
    std::atomic<std::uint64_t> semantic_fresh{0}, semantic_missing{0}, semantic_stale{0}, semantic_other{0},
        semantic_bins{0}, semantic_frames_with_bins{0}, no_ground_refused{0};
    // semantic_stale_guard: Stale only because of the publish guard (a subset of semantic_stale).
    // semantic_expired_publish: Fresh at the check, past objects_stale_ms at the publish entry,
    // so the whole body was refused (a subset of body_unavailable); the guard exists to make
    // this zero.
    std::atomic<std::uint64_t> semantic_stale_guard{0}, semantic_expired_publish{0};
    // T channel (19 section 3.3, plan 2f).
    std::atomic<std::uint64_t> t_fresh{0}, t_missing{0}, t_stale{0}, t_geometry_unavailable{0}, t_other{0},
        t_near_support_missing{0}, t_bins{0}, t_frames_with_bins{0};
};

// The native put audit of the Profile publisher's last put. In the service: the RT transport and
// a function returning its last_audit(); same "context + function pointer" form as
// rns::NativeProfileClock. Called on the Profile thread after a put, never null.
struct PutAuditReader {
    const void* context = nullptr;
    rns::NativePutAudit (*read)(const void*) noexcept = nullptr;
};

// Everything the service's first-frame setup built for the steady state. All of it must outlive
// the call; the loop is the only user while it runs.
struct ProfileSteadyInputs {
    native_depth::Mailbox& mailbox;
    geometry::NativeGeometry& geometry;
    geometry::NormalGrid& grid;
    rns::HealthRoiView health_roi;
    bool calibrated;                          // the calibration verdict (19 section 7.3)
    rns::SourceJournal* processed;            // processed-depth journal for Status
    rns::NativeProfileClock clock;            // the capture clock of the Profile thread
    rns::PublisherChannel& channel;           // the Profile channel
    PutAuditReader put_audit;
    // S channel (19 section 3.4A): the projector (null: no S owner) and the slot it reads (null:
    // no S slot at all, the S counters stay untouched). semantic_options are the ones the
    // projector was built with; PROFILE_SEMANTIC_READY prints them.
    rns::SemanticProjector* semantic;
    SemanticSlot* semantic_slot;
    rns::SemanticOptions semantic_options;
    // T channel (19 section 3.3, plan 2f): the T owner (null: no T this run), the slot it reads,
    // and the producer's ready flag (TProducerState::ready), set once the consumer exists.
    geometry::NormalTraversability* t_owner;
    TMaskSlot* t_slot;
    std::atomic<bool>* t_ready;
    std::int64_t t_max_age_ns;                // seg_max_age_ms in ns (PROFILE_T_TIMING thresholds)
    // Frame-level reasons for Status (reply B3-3) and their two bits in its active-reason mask.
    ProfileReasonState& reasons;
    std::uint64_t ground_fit_fallback, ground_free_withdrawn;
    // Where the timing figures go every 10 s and at stop (another thread formats and logs them).
    ProfileTimingHandoff& timing_handoff;
};

// The fast line's steady state (19 section 1.1 (2), P19-1): the S and T slot adapters, the
// consumer over the given owners, the T ready flag, then one step per frame with the counters and
// the timing ledger, its figures offered to in.timing_handoff every 10 s, until `finish`; then
// the handoff is closed and its last lines (when=stop) logged from this thread. Throws like the
// service's loop always did (90 consecutive abnormal frames, a reason outside the tracked set);
// the caller turns that into an owner fault.
void run_profile_steady(const std::atomic<bool>& finish, const ProfileSteadyInputs& in, ProfileCounters& s);

// The Profile loop without a configured transform: the explicit geometry-unconfigured Profile (no
// geometric claims), still at frame rate. Moved unchanged from the service (unconfigured_loop) on
// 2026-10-14 so that its periodic timing figures can be tested like the steady loop's.
struct ProfileUnconfiguredInputs {
    native_depth::Mailbox& mailbox;
    std::size_t frame_pixels;                 // the native depth raster (640 x 400)
    rns::UnconfiguredProfileDomain domain;
    rns::NativeProfileClock clock;
    rns::PublisherChannel& channel;
    rns::SourceJournal* processed;
    PutAuditReader put_audit;
    ProfileTimingHandoff& timing_handoff;
};
void run_profile_unconfigured(const std::atomic<bool>& finish, const ProfileUnconfiguredInputs& in,
                              ProfileCounters& s);

}  // namespace perception::service
