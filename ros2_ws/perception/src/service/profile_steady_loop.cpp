/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: profile_steady_loop.cpp
 * Brief: The service's two Profile loops: the configured steady state and the unconfigured loop.
 *
 * Description:
 * Moved unchanged from configured_loop in perception_service_main.cpp on 2026-10-14 (A19-TIME-2,
 * see profile_steady_loop.hpp): the S slot adapter, the T slot adapter, the consumer, the T ready
 * flag, the timing ledger with PROFILE_SEMANTIC_READY, and the loop. The only edits are the names
 * the service's locals had (s.mailbox -> in.mailbox, transport.last_audit() -> in.put_audit, the
 * finish flag, the T ready flag, objects_stale_ms and publish_guard_ms read back from the
 * semantic options they were turned into). Same order, same waits, same exceptions.
 * Second change of 2026-10-14: unconfigured_loop moved here as run_profile_unconfigured(), also
 * unchanged but for the names, and in both loops the 10 s and stop timing lines go through
 * ProfileTimingHandoff (service/profile_timing_handoff.hpp): offer() every 10 s, close() at stop.
 */

#include "service/profile_steady_loop.hpp"

#include "common/logger.hpp"
#include "service/profile_timing.hpp"
#include "service/profile_timing_handoff.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

namespace perception::service {
namespace {
using Steady = std::chrono::steady_clock;

// Thread CPU clock of the calling thread, or -1 when unavailable.
std::int64_t thread_cpu_ns() noexcept {
#if defined(__linux__) && defined(CLOCK_THREAD_CPUTIME_ID)
    timespec ts{};
    if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
#else
    return -1;
#endif
}
}  // namespace

void run_profile_steady(const std::atomic<bool>& finish, const ProfileSteadyInputs& in, ProfileCounters& s) {
    using namespace perception::rns;
    // The snapshot storage lives on this thread; latest() copies into it.
    struct SemanticContext {
        service::SemanticSlot* slot;
        service::SemanticSlot::Snapshot snapshot;
    } semantic_context{in.semantic_slot, {}};
    const rns::SemanticFrameSource semantic_source{
        [](void* context, const geometry::CaptureStamp& depth) noexcept -> const rns::SemanticFrame* {
            auto* c = static_cast<SemanticContext*>(context);
            return c->slot ? c->slot->latest(c->snapshot, depth) : nullptr;
        },
        &semantic_context};
    // The mask storage lives on this thread (1 MB, allocated once); latest() copies into it.
    struct TContext {
        service::TMaskSlot* slot;
        service::TMaskSlot::Snapshot snapshot;
    };
    std::unique_ptr<TContext> t_context;
    if (in.t_owner) t_context = std::make_unique<TContext>(TContext{in.t_slot, {}});
    const rns::TFrameSource t_source{
        [](void* context, const geometry::CaptureStamp& depth) noexcept -> const geometry::RawTMask* {
            auto* c = static_cast<TContext*>(context);
            return c->slot->latest(c->snapshot, depth, service::kTProjectionRevision);
        },
        t_context.get()};
    // With a T owner the T constructor; otherwise exactly the previous one.
    auto make_consumer = [&]() {
        if (in.t_owner)
            return NativeProfileConsumer(in.mailbox, in.geometry, in.grid, in.clock, in.channel,
                                         in.t_owner, t_source, in.semantic, semantic_source, in.processed,
                                         in.health_roi, in.calibrated);
        return NativeProfileConsumer(in.mailbox, in.geometry, in.grid, in.clock, in.channel,
                                     in.semantic, semantic_source, in.processed, in.health_roi, in.calibrated);
    };
    NativeProfileConsumer consumer = make_consumer();
    // From here the Objects side may publish masks (it publishes nothing before).
    if (in.t_owner && in.t_ready) in.t_ready->store(true, std::memory_order_release);
    ProfileTiming timing;
    timing.enable_stages();
    if (in.t_owner) timing.enable_t(in.t_max_age_ns);
    if (in.semantic) {
        timing.enable_semantic(in.semantic_options.max_age_ns);
        LOG_INFO_M("Service", "PROFILE_SEMANTIC_READY stale_ms=" << in.semantic_options.max_age_ns / 1000000
            << " publish_guard_ms=" << in.semantic_options.publish_guard_ns / 1000000
            << " v_ego_max_mps=" << in.semantic_options.v_max_mps << " omega_max_rps=" << in.semantic_options.omega_max_rps
            << " m_jitter_m=" << in.semantic_options.jitter_m << " base_revision=" << service::kSemanticBaseRevision
            << " bytes=" << in.semantic->storage_bytes()
            << "; a NoGroundReference frame keeps publishing (FREE withdrawn, independent S kept)");
    }
    const auto ground_fit_fallback = in.ground_fit_fallback, ground_free_withdrawn = in.ground_free_withdrawn;
    auto next_log = Steady::now() + std::chrono::seconds(10);
    std::uint64_t consecutive_abnormal = 0;
    while (!finish.load(std::memory_order_acquire)) {
        const auto before = in.mailbox.accepted_count();
        const auto cpu_before = thread_cpu_ns();
        const auto& tick = consumer.step();
        const auto cpu_after = thread_cpu_ns();
        // The tick's reason names -> this frame's reason bits; a name outside the tracked set
        // would never reach Status, so it is treated as a programming error.
        std::uint64_t reasons = 0;
        for (const auto reason : tick.reasons) {
            if (reason == "ground_fit_fallback") reasons |= ground_fit_fallback;
            else if (reason == "ground_free_withdrawn") reasons |= ground_free_withdrawn;
            else if (!reason.empty()) throw std::logic_error("unregistered Profile reason");
        }
        // S bookkeeping for frames that built a body: snapshot ages and margin for the
        // PROFILE_SEMANTIC_AGE line, the outcome counters for SERVICE_SUMMARY.
        if (in.semantic_slot && tick.build_ran) {
            if (in.semantic)
                timing.observe_semantic_age(in.semantic->report().age_at_capture_ns,
                                          in.semantic->report().age_at_check_ns,
                                          in.semantic->report().maximum_margin_m);
            switch (tick.semantic_status) {
            case SemanticStatus::Fresh: s.semantic_fresh.fetch_add(1, std::memory_order_relaxed); break;
            case SemanticStatus::Missing: s.semantic_missing.fetch_add(1, std::memory_order_relaxed); break;
            case SemanticStatus::Stale: s.semantic_stale.fetch_add(1, std::memory_order_relaxed); break;
            case SemanticStatus::NotProcessed: break;
            default: s.semantic_other.fetch_add(1, std::memory_order_relaxed); break;
            }
            if (tick.semantic_bins) {
                s.semantic_bins.fetch_add(tick.semantic_bins, std::memory_order_relaxed);
                s.semantic_frames_with_bins.fetch_add(1, std::memory_order_relaxed);
            }
            if (tick.semantic_stale_by_guard) s.semantic_stale_guard.fetch_add(1, std::memory_order_relaxed);
            if (tick.semantic_expired_at_publish) s.semantic_expired_publish.fetch_add(1, std::memory_order_relaxed);
            timing.observe_semantic_publish(tick.semantic_check_ns, tick.publish_entry_ns,
                                            tick.semantic_status == SemanticStatus::Fresh);
            if (tick.route == NativeProfileRoute::NoGroundReference && !tick.put.dispatched)
                s.no_ground_refused.fetch_add(1, std::memory_order_relaxed);
        }
        // T bookkeeping: outcome counters for SERVICE_T_SUMMARY, and the T step time measured on
        // the consumer's own stamps (grid end -> T end) with the mask age at capture.
        if (tick.t_ran) {
            switch (tick.t_status) {
            case geometry::TStatus::Fresh: s.t_fresh.fetch_add(1, std::memory_order_relaxed); break;
            case geometry::TStatus::Missing: s.t_missing.fetch_add(1, std::memory_order_relaxed); break;
            case geometry::TStatus::Stale: s.t_stale.fetch_add(1, std::memory_order_relaxed); break;
            case geometry::TStatus::GeometryUnavailable: s.t_geometry_unavailable.fetch_add(1, std::memory_order_relaxed); break;
            default: s.t_other.fetch_add(1, std::memory_order_relaxed); break;
            }
            if (tick.t_near_support_missing) s.t_near_support_missing.fetch_add(1, std::memory_order_relaxed);
            if (tick.t_bins) {
                s.t_bins.fetch_add(tick.t_bins, std::memory_order_relaxed);
                s.t_frames_with_bins.fetch_add(1, std::memory_order_relaxed);
            }
            timing.observe_t(tick.t_end_ns >= 0 && tick.grid_end_ns >= 0 ? tick.t_end_ns - tick.grid_end_ns : -1,
                             tick.t_age_ns);
        }
        // Per-status bookkeeping; same wait and 90-frame fault rule as the unconfigured loop.
        switch (tick.status) {
        case NativeProfileStatus::PutFinished:
            // A processed frame (built and dispatched): its COMPLETE reason set,
            // 0 for a normal frame, is the new current state (B3-3).
            if (tick.put.dispatched && !in.reasons.commit_frame(reasons))
                throw std::logic_error("Profile reason outside the tracked set");
            s.puts.fetch_add(1, std::memory_order_relaxed);
            if (tick.put.dispatched && tick.put.publish) s.accepted.fetch_add(1, std::memory_order_relaxed);
            else s.rejected_or_unknown.fetch_add(1, std::memory_order_relaxed);
            if (tick.route == NativeProfileRoute::Normal) s.route_normal.fetch_add(1, std::memory_order_relaxed);
            else if (tick.route == NativeProfileRoute::NoGroundReference) s.route_no_ground.fetch_add(1, std::memory_order_relaxed);
            else if (tick.route == NativeProfileRoute::FitFailedWithdrawal) s.route_withdrawn.fetch_add(1, std::memory_order_relaxed);
            if (tick.put.dispatched && tick.put.publish.audit.attempted) {
                timing.observe(tick.source.host_ready_mono_ns, tick.taken_ns, tick.build_end_ns,
                               in.put_audit.read(in.put_audit.context));
                timing.observe_cpu(cpu_before < 0 || cpu_after < 0 ? -1 : cpu_after - cpu_before);
                timing.observe_stages(tick, cpu_before < 0 || cpu_after < 0 ? -1 : cpu_after - cpu_before);
            }
            consecutive_abnormal = 0;
            break;
        case NativeProfileStatus::Idle:
        case NativeProfileStatus::SourceUnavailable:
            (void)in.mailbox.wait_for_accepted_after(before, std::chrono::milliseconds(5));
            break;
        default:
            s.abnormal.fetch_add(1, std::memory_order_relaxed);
            if (tick.status == NativeProfileStatus::ClockRejected) s.clock_rejected.fetch_add(1, std::memory_order_relaxed);
            else if (tick.status == NativeProfileStatus::GeometryUnavailable) s.geometry_unavailable.fetch_add(1, std::memory_order_relaxed);
            else if (tick.status == NativeProfileStatus::GridUnavailable) s.grid_unavailable.fetch_add(1, std::memory_order_relaxed);
            else s.body_unavailable.fetch_add(1, std::memory_order_relaxed);
            if (++consecutive_abnormal >= 90)
                throw std::runtime_error("configured Profile rejected 90 consecutive frames (last status " +
                                         std::to_string(static_cast<int>(tick.status)) + " geometry " +
                                         std::to_string(static_cast<int>(tick.geometry_status)) + ")");
        }
        // Every 10 s the figures of the timing lines are frozen into the handoff; the Status thread
        // formats and logs them (service/profile_timing_handoff.hpp). Nothing is formatted here.
        if (Steady::now() >= next_log) {
            next_log += std::chrono::seconds(10);
            (void)in.timing_handoff.offer(timing, ProfileTimingOwner::Configured);
        }
    }
    // Stopped: groups the Status thread has not taken yet, then the final figures (when=stop),
    // logged from this thread now that the loop is over.
    std::vector<std::string> lines;
    in.timing_handoff.close(timing, ProfileTimingOwner::Configured, lines);
    for (const auto& line : lines) LOG_INFO_M("Service", line);
}

void run_profile_unconfigured(const std::atomic<bool>& finish, const ProfileUnconfiguredInputs& in,
                              ProfileCounters& s) {
    using namespace perception::rns;
    UnconfiguredProfileConsumer consumer(in.mailbox, in.frame_pixels, in.domain, in.clock, in.channel, in.processed);
    ProfileTiming timing;
    auto next_log = Steady::now() + std::chrono::seconds(10);
    std::uint64_t consecutive_abnormal = 0;
    while (!finish.load(std::memory_order_acquire)) {
        const auto before = in.mailbox.accepted_count();
        const auto& tick = consumer.step();
        if (tick.status == UnconfiguredProfileStatus::PutFinished) {
            s.puts.fetch_add(1, std::memory_order_relaxed);
            if (tick.put.dispatched && tick.put.publish) s.accepted.fetch_add(1, std::memory_order_relaxed);
            else s.rejected_or_unknown.fetch_add(1, std::memory_order_relaxed);
            if (tick.put.dispatched && tick.put.publish.audit.attempted)
                timing.observe(tick.source.host_ready_mono_ns, tick.taken_ns, tick.build_end_ns,
                               in.put_audit.read(in.put_audit.context));
            consecutive_abnormal = 0;
        } else if (tick.status == UnconfiguredProfileStatus::Idle ||
                   tick.status == UnconfiguredProfileStatus::SourceUnavailable) {
            // Nothing new: wait for the next accepted frame (5 ms cap, so the
            // finish flag is seen promptly).
            (void)in.mailbox.wait_for_accepted_after(before, std::chrono::milliseconds(5));
        } else {
            // 90 consecutive rejected frames (about 3 s at 30 fps) is a fault,
            // not a transient: stop instead of publishing nothing forever.
            s.abnormal.fetch_add(1, std::memory_order_relaxed);
            if (++consecutive_abnormal >= 90)
                throw std::runtime_error("profile consumer rejected 90 consecutive frames (clock/profile)");
        }
        if (Steady::now() >= next_log) {
            next_log += std::chrono::seconds(10);
            (void)in.timing_handoff.offer(timing, ProfileTimingOwner::Unconfigured);
        }
    }
    std::vector<std::string> lines;
    in.timing_handoff.close(timing, ProfileTimingOwner::Unconfigured, lines);
    for (const auto& line : lines) LOG_INFO_M("Service", line);
}

}  // namespace perception::service
