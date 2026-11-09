/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: perception_service_main.cpp
 * Brief: Formal R6 perception service entry (installed as lib/perception/perception_node).
 *
 * Description:
 * One process (19 section 1.1): capture/inference/tracking via the existing
 * Supervisor; Profile owner (fast line, never waits for inference); Objects put
 * from the Supervisor result owner, or with objects_release_delay_ms from the paced
 * release thread (rns_objects_release.hpp); independent 1 Hz Status owner with reason
 * edges; GEN owner for reason edges, the startup config digest and RT router
 * loss/restore. Two Zenoh sessions (RT, GEN) with endpoints from the resolved
 * product. No test audit receiver, no matching-subscriber wait, no frame limit.
 *
 * Exit codes: 0 clean stop on SIGTERM/SIGINT; 78 startup self-check refused
 * (configuration/identity/environment); 69 dependency unavailable at startup
 * (router session, camera/model init); 70 runtime fault (owner/pipeline).
 *
 * Profile: no transform -> explicit geometry-unconfigured mode; a configured
 * transform (certified or not, reply Q12) -> native geometry from the FIRST
 * native frame (PSC-2/3/7 refusals exit 78) and the NativeProfileConsumer, with
 * the S owner (19 section 3.4A) fed from the Objects footprints of this process.
 * extrinsic_calibrated on all three topics comes only from the calibration
 * verdict (19 section 7.3).
 *
 * T evidence (19 section 3.3, plan 2f) is wired: the raw class-92 union of each inference frame
 * goes through TMaskSlot to the Profile owner's T consumer, and Status derives
 * traversable_seg_available / no_traversable_seg from the slot's publications. Still open
 * (reply 2026-09-17, Q3): the MED-2 media parameter table. PSC-4 runs one real golden
 * frame against sealed assets at startup.
 */

#include "service/calibration_record.hpp"
#include "service/gen_notices.hpp"
#include "service/geometry_setup.hpp"
#include "service/latency_histogram.hpp"
#include "service/object_geometry_hook.hpp"
#include "service/object_geometry_setup.hpp"
#include "service/objects_delivery.hpp"
#include "service/profile_steady_loop.hpp"
#include "service/profile_timing_handoff.hpp"
#include "service/psc4_run.hpp"
#include "service/semantic_slot.hpp"
#include "service/t_mask_slot.hpp"
#include "service/t_producer.hpp"
#include "service/t_status.hpp"
#include "service/t_projection_setup.hpp"
#include "service/profile_reason_state.hpp"
#include "service/ros_bridge.hpp"
#include "service/service_config.hpp"

#include "common/capture_time_state.hpp"
#include "common/config.hpp"
#include "common/logger.hpp"
#include "common/preprocess_mode.hpp"
#include "common/profile_thread_placement.hpp"
#include "common/sdk_stream_thread_placement.hpp"
#include "common/thread_name.hpp"
#include "modules/native_depth.hpp"
#include "common/rns_geometry_config.hpp"
#include "modules/rgb_depth_registration.hpp"
#include "modules/orbbec_native_rays.hpp"
#include "output/rns_capture_estimate.hpp"
#include "output/rns_fused_objects.hpp"
#include "output/rns_gen_events.hpp"
#include "output/rns_gen_transport.hpp"
#include "output/rns_health_fields.hpp"
#include "output/rns_object_geometry.hpp"
#include "output/rns_objects_release.hpp"
#include "output/rns_publication_handoff.hpp"
#include "output/rns_reason_edges.hpp"
#include "output/rns_status_heartbeat.hpp"
#include "output/rns_system_clock.hpp"
#include "output/rns_unconfigured_profile.hpp"
#include "output/rns_zenoh_transport.hpp"
#include "supervisor/supervisor.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <optional>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {
using namespace perception;
using namespace perception::rns;
namespace nd = perception::native_depth;
using Steady = std::chrono::steady_clock;

constexpr int kExitConfig = 78, kExitUnavailable = 69, kExitRuntime = 70;
constexpr std::uint64_t kEpoch = 1;              // one process, one host CLOCK_MONOTONIC
constexpr std::size_t kPixels = 640U * 400U;     // native depth raster (PSC-3 profile)
constexpr std::size_t kPacketCapacity = 262144;  // bounded wire packet buffer
constexpr std::size_t kSourceCapacity = 2048;
constexpr std::size_t kJournalCapacity = 1024;
// 11 T-11 / CLK-A3 window. No ClockStatus subscription is declared (reply Q1),
// so ts_sync stays false regardless of this value.
constexpr std::int64_t kClockStatusTimeoutMs = 5000;

// Signal handler only records the signal; the main loop does the stopping.
std::atomic<int> g_signal{0};

// Stop-time line of an RT transport's packet pool (A19-ALLOC-1): fallbacks counts packets that
// found every slot still held by zenoh-c and took the malloc path (0 in a healthy run);
// bad_releases must stay 0; held is an instantaneous reading, the slots zenoh-c had not returned
// at the moment of the call (not a total, and it may be 0 a moment later). Called by the owning
// thread after its last put, or by another thread once the owner has been joined (the counters
// are the owner's, not atomics). Only on an orderly stop: an owner that leaves through its catch
// path (a fault) never reaches its call, so a run can end with fewer than three RT_POOL lines.
void log_pool(const char* topic, const ZenohRtTransport& transport) {
    const PacketPoolCounters c = transport.pool_counters();
    LOG_INFO_M("Service", "RT_POOL topic=" << topic << " slots=" << ZenohRtTransport::kPoolSlots << " fallbacks="
        << c.fallbacks << " bad_releases=" << c.bad_releases << " held=" << c.held);
}
void on_signal(int signo) { g_signal.store(signo, std::memory_order_relaxed); }

// The one service clock (CLOCK_MONOTONIC ns); -1 on error, never a guess.
std::int64_t monotonic_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0 || ts.tv_sec < 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * INT64_C(1000000000) + ts.tv_nsec;
}
std::int64_t source_now(void*) noexcept { return monotonic_ns(); }

// Per-instance nonce for GEN event ids (eid = p-<nonce>-<seq>), from the kernel
// UUID source, so ids never repeat across restarts.
std::string random_nonce() {
    std::ifstream in("/proc/sys/kernel/random/uuid");
    std::string uuid, hex;
    std::getline(in, uuid);
    for (const char c : uuid) if (c != '-') hex += c;
    if (hex.size() != 32 || hex.find_first_not_of("0123456789abcdef") != std::string::npos)
        throw std::runtime_error("cannot read a process instance UUID from /proc/sys/kernel/random/uuid");
    return hex;
}

// Fault latch shared by owner threads and the main thread.
struct OwnerControl {
    std::atomic<bool> finish{false}, faulted{false}, refused{false};
    std::mutex mutex;
    std::string reason, refused_check, refused_key;
    void fault(const std::string& why) noexcept {
        try {
            std::lock_guard<std::mutex> lock(mutex);
            if (reason.empty()) reason = why;
        } catch (...) {}
        faulted.store(true, std::memory_order_release);
    }
    // A startup self-check that can only run once the device delivers data
    // (PSC-3/PSC-7 on the first native frame): the process exits 78.
    void refuse(const service::StartupCheckError& e) noexcept {
        try {
            std::lock_guard<std::mutex> lock(mutex);
            if (reason.empty()) {
                reason = e.what();
                refused_check = e.check();
                refused_key = e.key();
                refused.store(true, std::memory_order_release);
            }
        } catch (...) {}
        faulted.store(true, std::memory_order_release);
    }
};

// ---- Objects publication (Supervisor result owner thread, or the paced release thread) ----
// Objects are Q1 RT data (11 section 2.4): real-time priority, best effort,
// express; the channel also feeds the Objects cadence and publication journal.
// One thread publishes for the whole run: the Supervisor's with objects_release_delay_ms "0",
// the release thread otherwise. The counters are atomics for the main thread's summary.
struct ObjectsOwner {
    SystemPublishClock clock;
    ObjectsCadence cadence;
    ZenohRtTransport transport;
    PublisherChannel channel;
    // channel_calls / accepted / rejected / unknown (objects_delivery.hpp). The summary lines
    // print channel_calls under their old key objects_attempts: it is the number of calls of the
    // channel, not of put attempts.
    service::ObjectsPublishCounters counters;
    ObjectsOwner(ProcessEnvelope& envelope, ZenohRtSession& session, const std::string& key,
                 Millis startup, PublicationJournal& journal)
        : cadence(startup, 1024),
          transport(session, key, {Priority::RealTime, false, true, true}, kPacketCapacity),
          channel(Topic::Objects, envelope, clock, transport, kPacketCapacity - 4096, kPacketCapacity,
                  &cadence, &journal) {
        if (!channel.valid()) throw std::runtime_error("Objects publisher channel invalid");
    }
    // One publish with the channel's own result (the paced release keeps it per frame).
    // Transport outcomes (Rejected/Unknown) are counted, never retried and do not stop the
    // pipeline; any other publisher error is a local contract fault (service::publish_counted).
    PublishResult publish_result(const Objects& body) noexcept {
        bool contract_fault = false;
        const auto result = service::publish_counted(channel, counters, body, &contract_fault);
        if (contract_fault) LOG_ERROR_M("Service", "Objects publish contract fault error=" << static_cast<int>(result.error));
        return result;
    }
    bool publish(const Objects& body) noexcept { return service::transport_outcome(publish_result(body)); }
};

// Text forms for the OBJ_RELEASE line.
const char* release_end_name(ReleaseEnd end) noexcept {
    switch (end) {
    case ReleaseEnd::Published: return "published";
    case ReleaseEnd::ChannelRefused: return "channel_refused";
    case ReleaseEnd::Overwritten: return "overwritten";
    case ReleaseEnd::StopDiscarded: return "stop_discarded";
    }
    return "unknown";
}
const char* release_reason_name(ReleaseReason reason) noexcept {
    switch (reason) {
    case ReleaseReason::None: return "none";
    case ReleaseReason::Deadline: return "deadline";
    case ReleaseReason::Late: return "late";
    }
    return "unknown";
}
// The counters of the paced release for the summary lines; unaccounted must be 0.
std::string release_fields(const ReleaseCounters& c) {
    std::ostringstream out;
    out << "offered=" << c.offered << " admitted=" << c.admitted << " refused_closed=" << c.refused_closed
        << " refused_faulted=" << c.refused_faulted << " refused_invalid=" << c.refused_invalid
        << " refused_copy=" << c.refused_copy << " published=" << c.published << " channel_refused=" << c.channel_refused
        << " overwritten=" << c.overwritten << " stop_discarded=" << c.stop_discarded << " pending=" << c.pending
        << " in_flight=" << c.in_flight << " committed=" << c.committed
        << " by_deadline=" << c.by_deadline << " late=" << c.late << " inherited=" << c.inherited << " accepted=" << c.accepted << " rejected=" << c.rejected << " unknown=" << c.unknown
        << " contract_faults=" << c.contract_faults << " records_lost=" << c.records_lost
        << " unaccounted=" << c.unaccounted();
    return out.str();
}

// B8-4 handoff line (input queue, result handoff, result frame_id ledger).
// A live line reads the counters without a common lock and can be off by a
// push in flight; the line after stop is exact.
// Comma list of frame ids for the handoff log line, "-" when empty.
std::string join_ids(const std::vector<std::uint64_t>& ids) {
    if (ids.empty()) return "-";
    std::ostringstream out;
    for (std::size_t i = 0; i < ids.size(); ++i) out << (i ? "," : "") << ids[i];
    return out.str();
}

std::string result_handoff_fields(const perception::ResultHandoffSnapshot& h) {
    const auto& l = h.ledger;
    std::ostringstream out;
    out << "available=" << h.available
        << " result_policy=" << (h.result_latest_wins ? "latest_wins_swap" : "drop_oldest_fifo")
        << " input_pushed=" << h.input.pushed << " input_popped=" << h.input.popped
        << " input_overwritten=" << h.input.dropped << " input_cleared=" << h.input.cleared
        << " input_pending=" << h.input.pending
        << " result_pushed=" << h.result.pushed << " result_popped=" << h.result.popped
        << " result_overwritten=" << h.result.dropped << " result_cleared=" << h.result.cleared
        << " result_pending=" << h.result.pending
        << " ledger_popped=" << l.popped << " ledger_overwritten=" << l.overwritten << " ledger_cleared=" << l.cleared
        << " order_violations=" << l.order_violations << " identity_conflicts=" << l.identity_conflicts
        << " window_popped=" << l.window_popped << " window_first_popped_id=" << l.window_first_popped_id
        << " window_last_popped_id=" << l.window_last_popped_id
        << " overwritten_ids=" << join_ids(l.overwritten_ids) << " cleared_ids=" << join_ids(l.cleared_ids)
        << " ids_not_listed=" << l.ids_not_listed;
    return out.str();
}

// Object geometry: ObjectGeometryShared and measure_object_geometry() live in
// service/object_geometry_hook.hpp (moved 2026-10-13 so a test calls the same function).
using service::ObjectGeometryShared;
using service::measure_object_geometry;
// At least 5 measured points, 1 cm x-buckets for the footprint hull, 2 cm half width for a
// degenerate (line-like) point set; runtime switch object_geometry_points "float" adds the
// single-precision path with a 1 mm margin (rns_object_geometry.hpp).
constexpr ObjectGeometryOptions kObjectGeometryOptions{5, 0.01, 0.02};
constexpr double kObjectGeometryFloatMargin = 0.001;
ObjectGeometryOptions object_geometry_options() {
    ObjectGeometryOptions options = kObjectGeometryOptions;
    const char* points = std::getenv("PERCEPTION_OBJECT_GEOMETRY_POINTS");
    if (points != nullptr && std::strcmp(points, "float") == 0) options.float_margin_m = kObjectGeometryFloatMargin;
    return options;
}

// ---- Status reasons ----
// The degraded_reasons this service can raise; the index is the bit in the
// active-reason mask and the order of the published list.
const std::vector<std::string>& reason_names() {
    static const std::vector<std::string> names = {
        "infer_rate_low", "infer_gap", "no_traversable_seg", "roi_fallback", "ground_fit_fallback",
        "ground_free_withdrawn", "t_capture_estimated", "reconnecting", "reconfiguring",
        "geometry_unconfigured", "med2_not_configured",
        // Free diagnostic reason (19 section 5.1; batch-4 reply B4-3): configured
        // extrinsics but no Objects geometry capability for the latest Objects frame.
        "object_geometry_unavailable",
        // Free diagnostic reason (issues.md 3.3, agreed 2026-10-08): T is switched on but its
        // colour projection could not be set up (calibration or registration gate).
        "t_projection_unavailable",
        // Free diagnostic reason (issues.md 9.1, 2026-10-13): the placement of the camera SDK's
        // stream threads (runtime switch capture_sdk_threads_cpu) failed its self-check at start
        // or at the last check: not exactly the expected threads born at the start (too few,
        // too many, one born on other CPUs), one that ended, one on other CPUs, or one that
        // cannot be read. It says "not verified", which need not mean "wrong".
        "capture_threads_misplaced",
        // Free diagnostic reason (user decision 2026-10-14; capture_time_state.hpp): framesets got no
        // capture time from the capture clock and were not handed on, so nothing was published for
        // them. Not a declared exclusion: the pause counts in infer_gap / infer_rate_low.
        "capture_time_unavailable"};
    return names;
}
// Mask bit of a reason; an unknown name is a programming error.
std::uint64_t bit(const char* name) {
    const auto& names = reason_names();
    const auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) throw std::logic_error("unknown reason");
    return std::uint64_t{1} << static_cast<std::size_t>(it - names.begin());
}

// T channel producer state (plan 2f, service/t_producer.hpp), shared by the Objects callback
// (writer), the Profile thread (sets ready once its T owner exists) and Status (reads ready).
using service::TProducerState;

// Everything the Status owner reads: the journals written by the other owners,
// the GEN event queue for reason edges, and the fixed facts of this start.
struct StatusShared {
    ProcessEnvelope& envelope;
    ZenohRtSession& session;
    std::string key;
    Millis startup;
    PublicationJournal& profile;
    PublicationJournal& objects;
    SourceJournal& arrival;
    SourceJournal& inference;
    SourceJournal& processed;
    GenEvents& gen;
    bool geometry_unconfigured;
    bool med2_unconfigured;
    bool extrinsic_calibrated;                     // CalibrationVerdict::calibrated()
    service::ProfileReasonState& profile_reasons;  // latest processed Profile frame (B3-3)
    service::ProfileReasonState& objects_reasons;  // latest built Objects frame (B4-3)
    std::atomic<std::uint64_t> beats{0}, beat_failures{0}, edges_offered{0}, edges_lost{0};
    // T slot and producer (plan 2f); null without configured geometry or with traversable_seg off.
    const service::TMaskSlot* t_slot = nullptr;
    TProducerState* t_state = nullptr;
    // The Profile thread's timing figures: this thread formats and logs them (the PROFILE_TIMING
    // lines), so the Profile thread does not (service/profile_timing_handoff.hpp).
    service::ProfileTimingHandoff* profile_timing = nullptr;
    // Placement of the Profile thread on its own core (profile_thread_placement.hpp); null when
    // profile_thread_cpu is absent or "off".
    ProfileThreadPlacement* profile_placement = nullptr;
};

// Status owner thread (11 section 3.1B.3): every 10 ms drain the journals into
// the 10 s health windows, derive the active reasons, turn reason changes into
// GEN edge events, and publish the 1 Hz heartbeat. Five consecutive failed
// beats stop the service (a silent Status would look healthy to nobody). Since 2026-10-14 it
// also formats and logs the Profile thread's 10 s timing lines (profile_timing_handoff.hpp).
void status_owner(OwnerControl& ctl, StatusShared& s) noexcept {
    try {
        perception::set_current_thread_name("svc_status");
        SystemPublishClock clock;
        HealthWindow health(s.startup, kSourceCapacity);
        DepthHealthWindow depth(s.startup, kSourceCapacity);
        ObjectsCadence cadence(s.startup, 1024);
        PublicationHealthHandoff handoff(s.startup, s.profile, s.objects, health, cadence, s.arrival,
                                         s.inference, s.processed, depth);
        if (!handoff.valid()) throw std::runtime_error("status handoff invalid");
        // Status is Q2 (11 section 2.4): data_high priority, reliable, drop.
        ZenohRtTransport transport(s.session, s.key, {Priority::DataHigh, true, false, true}, kPacketCapacity);
        PublisherChannel channel(Topic::Status, s.envelope, clock, transport, 4096, kPacketCapacity);
        StatusHeartbeat heartbeat(channel, s.startup, 100);
        ReasonEdges edges(s.startup, reason_names().size(), 256);
        Status body;
        body.degraded_reasons.reserve(reason_names().size());
        std::uint64_t consecutive_failures = 0;
        // roi_fallback: uncalibrated extrinsics use the lower-half ROI (11 section 3.1B.3).
        // no_traversable_seg is no longer fixed: it follows the 10 s segmentation
        // capability below (19 section 3.3 / 5.1, plan 2f). t_capture_estimated is no longer
        // fixed either: capture times come from the camera's frame timestamp, and the reason is
        // raised only while framesets are stamped in the fallback grade (below).
        CaptureTimeReasonWatch capture_time_reasons;
        const std::uint64_t fixed =
            (s.extrinsic_calibrated ? 0 : bit("roi_fallback")) |
            (s.geometry_unconfigured ? bit("geometry_unconfigured") : 0) |
            (s.med2_unconfigured ? bit("med2_not_configured") : 0);
        // Frame-level Profile reasons (ground_fit_fallback, ground_free_withdrawn):
        // the state of the latest PROCESSED Profile frame. Only a new normal frame
        // clears them; a heartbeat, a failed beat or no new frame never does
        // (reply batch 3, B3-3). Window counts are internal diagnostics only.
        auto window_start = s.profile_reasons.snapshot();
        std::vector<std::string> timing_lines;
        while (!ctl.finish.load(std::memory_order_acquire)) {
            // Last T slot publication (CLOCK_MONOTONIC), read BEFORE the cut so it
            // can never lie after the observation instant. Only while a T owner
            // consumes the slot; a locked producer publishes nothing more.
            const std::int64_t seg_ns = s.t_slot && s.t_state && s.t_state->ready.load(std::memory_order_acquire)
                ? s.t_slot->last_publish_mono_ns() : -1;
            // One consistent cut of all journals at one observation instant.
            const auto cut = handoff.drain(clock);
            if (cut.code == HandoffCode::Ready) {
                const auto h = health.snapshot(cut.observed_ms);
                // 19 section 5.1 / 3.3: traversable_seg_available and no_traversable_seg
                // from the last slot publication, exactly as apply_health_fields checks
                // it (service/t_status.hpp; a valid update is any producer publication,
                // including the all-zero mask of a frame without class 92, accepted in
                // issues.md 3.2(1), 2026-10-08).
                const auto seg = service::seg_capability(seg_ns >= 0, seg_ns, h.now_ms, cut.sampled_now_ms,
                                                         HealthWindow::kWindowMs);
                if (s.t_state && seg.in_open_ms) s.t_state->seg_in_open_ms.fetch_add(1, std::memory_order_relaxed);
                if (s.t_state && seg.clock_anomaly) s.t_state->clock_anomaly.fetch_add(1, std::memory_order_relaxed);
                const std::optional<Millis> seg_ms = seg.last_update_ms;
                const bool seg_available = seg.available;
                const auto c = cadence.snapshot(cut.observed_ms);
                const auto d = depth.snapshot(cut.observed_ms);
                // Active reasons: fixed ones, latest Profile/Objects frame reasons,
                // and the Objects cadence gates of 11 section 3.1B.3 (v2.1):
                // infer_gap = any publish interval > 50 ms in the window;
                // infer_rate_low = a tier gate fails (max interval > 100 ms, or
                // more than 1 % of intervals above 50 ms).
                std::uint64_t active = fixed | s.profile_reasons.current() | s.objects_reasons.current() |
                    (seg_available ? 0 : bit("no_traversable_seg"));
                if (s.t_state && s.t_state->projection_unavailable.load(std::memory_order_acquire))
                    active |= bit("t_projection_unavailable");
                if (sdk_thread_placement_state().load(std::memory_order_relaxed) ==
                    static_cast<int>(SdkThreadPlacementState::Fault))
                    active |= bit("capture_threads_misplaced");
                // Capture time grade of the last 1.5 s (capture_time_state.hpp).
                const auto capture_time = capture_time_reasons.poll(capture_time_counters(), cut.observed_ms);
                if (capture_time.estimated) active |= bit("t_capture_estimated");
                if (capture_time.unavailable) active |= bit("capture_time_unavailable");
                if (c.infer_gap || c.max_gap_ms > 50) active |= bit("infer_gap");
                if (c.infer_rate_low || c.max_gap_ms > 100 || c.over50_fraction > .01) active |= bit("infer_rate_low");
                const auto update = edges.update(active, cut.observed_ms);
                if (update.code == ReasonEdgeCode::Overflow || update.code == ReasonEdgeCode::Invalid)
                    throw std::runtime_error("reason edge journal fault");
                // Each reason change becomes one GEN event; a full queue loses the
                // edge (counted), it never blocks the heartbeat.
                ReasonEdge edge;
                while (edges.pop(edge)) {
                    const auto offered = s.gen.offer(edge, clock.wall_millis(), clock.mono_millis());
                    if (offered.code == GenOfferCode::Accepted) s.edges_offered.fetch_add(1, std::memory_order_relaxed);
                    else s.edges_lost.fetch_add(1, std::memory_order_relaxed);
                }
                body.degraded_reasons.clear();
                for (std::size_t i = 0; i < reason_names().size(); ++i)
                    if (active & (std::uint64_t{1} << i)) body.degraded_reasons.emplace_back(reason_names()[i]);
                HealthFieldContext context;
                context.extrinsic_calibrated = s.extrinsic_calibrated;  // 19 section 7.3 verdict
                context.ground_seg_level = 0;          // semantics not closed upstream (reply Q14)
                context.last_valid_seg_update_ms = seg_ms;
                if (d.latest_source) context.expected_depth_source = *d.latest_source;
                const auto code = apply_health_fields(h, c, cut.observed_ms, d, context, body);
                const bool ready = code == HealthFieldsCode::Ready;
                // The heartbeat decides when a beat is due (1 Hz); an unready
                // field set is a failed beat, never a stale body.
                const auto beat = heartbeat.poll(clock.mono_millis(), ready ? &body : nullptr, cut.observed_ms);
                if (beat.due) {
                    if (ready && beat.code == HeartbeatCode::Published) {
                        s.beats.fetch_add(1, std::memory_order_relaxed);
                        consecutive_failures = 0;
                        const auto now_window = s.profile_reasons.snapshot();
                        const auto fit = bit("ground_fit_fallback"), withdrawn = bit("ground_free_withdrawn");
                        const auto index = [](std::uint64_t b) {
                            std::size_t i = 0;
                            while (!(b & (std::uint64_t{1} << i))) ++i;
                            return i;
                        };
                        const auto processed = now_window.processed_frames - window_start.processed_frames;
                        const auto fit_frames = now_window.frames_with[index(fit)] - window_start.frames_with[index(fit)];
                        const auto withdrawn_frames =
                            now_window.frames_with[index(withdrawn)] - window_start.frames_with[index(withdrawn)];
                        if (fit_frames || withdrawn_frames)
                            LOG_DEBUG_M("Service", "PROFILE_REASON_WINDOW processed=" << processed << " ground_fit_fallback="
                                            << fit_frames << " ground_free_withdrawn=" << withdrawn_frames);
                        window_start = now_window;
                    } else {
                        s.beat_failures.fetch_add(1, std::memory_order_relaxed);
                        LOG_WARN_M("Service", "status beat not published fields=" << static_cast<int>(code)
                                       << " heartbeat=" << static_cast<int>(beat.code));
                        if (++consecutive_failures >= 5) throw std::runtime_error("status heartbeat failed 5 consecutive beats");
                    }
                }
            } else if (cut.code == HandoffCode::Invalid) {
                throw std::runtime_error("status handoff cut invalid");
            }
            // Any other cut code (no consistent cut this round): retry on the next 10 ms tick;
            // the heartbeat's own timing decides when a missing beat counts as a failure.
            // The Profile thread's timing groups waiting since its last 10 s mark, after this
            // round's beat; never waits for the Profile thread (a round it is closing is skipped).
            if (s.profile_timing)
                (void)s.profile_timing->take_lines(timing_lines,
                                                   [](const std::string& line) { LOG_INFO_M("Service", line); });
            // Profile thread placement: the first step once the capture thread is pinned, then a
            // recheck of new threads once a second; a line only for a step that changed or failed.
            if (s.profile_placement) {
                const auto* placed = s.profile_placement->poll(capture_pinned_tid().load(std::memory_order_acquire),
                                                               profile_thread_tid().load(std::memory_order_acquire),
                                                               clock.mono_millis());
                if (placed && placed->success) LOG_INFO_M("Service", *placed);
                else if (placed) LOG_ERROR_M("Service", *placed);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        log_pool("status", transport);
    } catch (const std::exception& e) {
        ctl.fault(std::string("status owner: ") + e.what());
    } catch (...) {
        ctl.fault("status owner: nonstandard exception");
    }
}

// ---- Profile owner (fast line) ----
// The Profile thread's counters are the base (service/profile_steady_loop.hpp), so the steady loop
// and the summary lines below read the same atomics as before (profile_shared.puts, ...).
struct ProfileShared : service::ProfileCounters {
    nd::Mailbox& mailbox;
    SourceJournal& processed;
    ProcessEnvelope& envelope;
    ZenohRtSession& session;
    std::string key;
    PublicationJournal& journal;
    UnconfiguredProfileDomain domain;
    std::string estimate_audit;
    // Configured geometry (nullptr config -> geometry-unconfigured mode).
    const service::ServiceConfig* config;
    const service::CalibrationVerdict* verdict;
    // S channel (19 section 3.4A). nullptr = no S owner: the Profile stays
    // geometry-only, exactly as before this wiring.
    service::SemanticSlot* semantic_slot = nullptr;
    service::ProfileReasonState reasons{bit("ground_fit_fallback") | bit("ground_free_withdrawn")};
    // T channel (19 section 3.3, plan 2f). t_slot null = no T owner (geometry
    // unconfigured, or traversable_seg off). The Profile thread builds the colour projection from the
    // session calibration of `pipeline` after its first frame.
    service::TMaskSlot* t_slot = nullptr;
    const perception::Supervisor* pipeline = nullptr;
    TProducerState* t_state = nullptr;
    // Where the loop's timing figures go (the Status thread logs them); set in run(), never null.
    service::ProfileTimingHandoff* timing_handoff = nullptr;
};

// The Profile publisher's last native put, for the loops' timing ledger (service::PutAuditReader).
rns::NativePutAudit rt_last_audit(const void* transport) noexcept {
    return static_cast<const ZenohRtTransport*>(transport)->last_audit();
}

// Profile loop without a configured transform: the explicit
// geometry-unconfigured Profile (no geometric claims), still at frame rate.
void unconfigured_loop(OwnerControl& ctl, ProfileShared& s, EstimatedCaptureClock& estimated,
                      ZenohRtTransport& transport, PublisherChannel& channel) {
    // The loop itself is service::run_profile_unconfigured() (profile_steady_loop.cpp, moved there
    // unchanged 2026-10-14 so that its timing figures are tested like the steady loop's).
    const service::ProfileUnconfiguredInputs in{s.mailbox, kPixels, s.domain, estimated.native_clock(), channel,
        &s.processed, {&transport, rt_last_audit}, *s.timing_handoff};
    service::run_profile_unconfigured(ctl.finish, in, s);
}

// Profile loop with a configured transform: geometry from the first frame, then
// NativeProfileConsumer steps with the timing and S bookkeeping around them.
void configured_loop(OwnerControl& ctl, ProfileShared& s, EstimatedCaptureClock& estimated,
                    ZenohRtTransport& transport, PublisherChannel& channel) {
    // 1. First native frame: its actual metadata binds the ray table. The frame
    //    itself is consumed for setup and is not published (counted).
    // Wait in 20 ms slices so a stop request during a camera that never delivers is honoured.
    nd::Frame setup_frame(kPixels);
    while (!ctl.finish.load(std::memory_order_acquire)) {
        const auto before = s.mailbox.accepted_count();
        if (s.mailbox.try_take(setup_frame) == nd::Take::Taken) break;
        (void)s.mailbox.wait_for_accepted_after(before, std::chrono::milliseconds(20));
    }
    if (ctl.finish.load(std::memory_order_acquire)) return;
    s.setup_frames.fetch_add(1, std::memory_order_relaxed);
    const auto& native = setup_frame.metadata();
    // The Orbbec native ray model (undistortion of the 640x400 depth raster) behind the factory.
    service::GeometryFactory factory;
    factory.prepare_rays = [](const geometry::ProjectionBinding& binding) {
        return std::static_pointer_cast<void>(std::make_shared<geometry::OrbbecNativeRays>(binding));
    };
    factory.make_geometry = [](const std::shared_ptr<void>& rays, const geometry::GeometryOptions& options) {
        return std::static_pointer_cast<geometry::OrbbecNativeRays>(rays)->make_geometry(options);
    };
    // A refused geometry setup is a startup refusal (exit 78), raised from here
    // because it needs the device's real metadata.
    std::unique_ptr<service::ProfileGeometry> g;
    try {
        g = service::build_profile_geometry(*s.config, native, *s.verdict, factory);
    } catch (const service::StartupCheckError& e) {
        ctl.refuse(e);
        return;
    }
    LOG_INFO_M("Service", "PROFILE_GEOMETRY_READY roi_pixels=" << g->roi_pixels << " calibrated=" << g->calibrated
        << " health_roi=" << (g->calibrated ? "calibrated_static_mask" : "lower_half_uncalibrated")
        << " fx=" << native.intrinsics.fx << " fy=" << native.intrinsics.fy << " cx=" << native.intrinsics.cx
        << " cy=" << native.intrinsics.cy << " distortion_model=" << native.distortion.model << " unit_m=" << native.unit_m
        << " setup_stream=" << native.source.stream_id << " setup_sequence=" << native.source.sequence
        << " geometry_bytes=" << g->geometry->storage_bytes() << " grid_bytes=" << g->grid->storage_bytes());

    // 2. Owners of the steady state: geometry + optional T owner (plan 2f, below) + optional S
    //    owner. Separate matter: the RNS Q4 prior-plane fallback is closed (a
    //    failed fit withdraws FREE, see the withdrawn builder).
    // The S envelope reuses the platform keys of the same configuration block:
    // v_ego_max_mps / omega_max_rps / m_jitter_m describe the robot, not a
    // channel, and objects_stale_ms is the 19 section 3.4A staleness bound.
    const auto& profile_cfg = s.config->contract.profile;
    // Publish guard (runtime switch semantic_publish_guard_ms, 2026-10-10; 0 when unset): a
    // snapshot within this many milliseconds of objects_stale_ms at the check is treated as
    // Stale, so the frame is published geometry-only instead of being refused at publish
    // (rns_semantic.hpp). The switch table limits the values; the projector refuses a guard
    // above objects_stale_ms.
    const char* const publish_guard_text = std::getenv("PERCEPTION_SEMANTIC_PUBLISH_GUARD_MS");
    const std::int64_t publish_guard_ms = publish_guard_text ? std::strtoll(publish_guard_text, nullptr, 10) : 0;
    const rns::SemanticOptions semantic_options{profile_cfg.v_ego_max_mps, profile_cfg.omega_max_rps,
                                                profile_cfg.m_jitter_m,
                                                s.config->contract.objects_stale_ms * 1000000LL,
                                                service::kSemanticBaseRevision,
                                                publish_guard_ms * 1000000LL};
    std::unique_ptr<rns::SemanticProjector> semantic;
    // NormalOrFitFailed: 19 A19-FIT-3 keeps an INDEPENDENT S boundary when the
    // ground fit fails (the withdrawn packet still carries it). Footprints that
    // depend on the failed plane are refused by the projector itself.
    if (s.semantic_slot)
        semantic = std::make_unique<rns::SemanticProjector>(*g->geometry, semantic_options,
                                                            rns::SemanticDepthPolicy::NormalOrFitFailed);
    // T channel (19 section 3.3, plan 2f): the factory-calibrated colour projection of
    // this capture session, then the T owner. The capture stored its calibration
    // before streaming, so it is available once this thread has a frame. A
    // capability gap (user decision 2026-09-30: reuse the Objects registration
    // gate) leaves the Profile without T (t_seg null, bit0 clear) and Status
    // without segmentation updates (no_traversable_seg); the service runs on.
    std::optional<service::TProjectionReady> t_projection;
    std::unique_ptr<geometry::NormalTraversability> t_owner;
    const geometry::TOptions t_options{profile_cfg.tau_T, profile_cfg.v_ego_max_mps, profile_cfg.omega_max_rps,
                                       profile_cfg.m_jitter_m, profile_cfg.seg_max_age_ms * 1000000LL};
    if (s.t_slot && s.pipeline) {
        // A plain copy of the capture session's calibration: service/ code needs no SDK types,
        // and a missing calibration is reported by setup_t_projection as a capability gap.
        const auto calibration = s.pipeline->rgbd_calibration();
        service::TProjectionSession session;
        if (calibration) {
            session.present = true;
            session.fx = calibration->colour.fx;
            session.fy = calibration->colour.fy;
            session.cx = calibration->colour.cx;
            session.cy = calibration->colour.cy;
            session.width = calibration->colour.width;
            session.height = calibration->colour.height;
            session.distortion_model = calibration->colour_distortion_model;
            session.distortion = calibration->colour_distortion;
            session.depth_to_colour_R = calibration->depth_to_colour_R;
            session.depth_to_colour_t_m = calibration->depth_to_colour_t_m;
            session.registration = calibration->registration;
            session.registration_status = calibration->registration_status;
        }
        try {
            t_projection.emplace(service::setup_t_projection(session, s.config->contract_yaml,
                                                             service::kTProjectionRevision));
            t_owner = std::make_unique<geometry::NormalTraversability>(*g->geometry, *g->grid,
                                                                       t_projection->projection, t_options);
            const auto& c = *t_projection->color;
            LOG_INFO_M("Service", "PROFILE_T_READY tau_T=" << t_options.tau << " v_ego_max_mps=" << t_options.v_max_mps
                << " omega_max_rps=" << t_options.omega_max_rps << " m_jitter_m=" << t_options.jitter_m
                << " seg_max_age_ms=" << profile_cfg.seg_max_age_ms << " revision=" << service::kTProjectionRevision
                << " colour_fx=" << c.fx << " colour_fy=" << c.fy << " colour_cx=" << c.cx << " colour_cy=" << c.cy
                << " max_r2=" << c.max_r2 << " colour_from_base_t=[" << c.t_color_base.x << "," << c.t_color_base.y
                << "," << c.t_color_base.z << "] t_bytes=" << t_owner->storage_bytes()
                << "; B3-1 HOLD: fresh T with a positive margin backs no FREE (near-side support missing)");
        } catch (const service::TProjectionCapabilityError& e) {
            if (s.t_state) s.t_state->projection_unavailable.store(true, std::memory_order_release);
            LOG_WARN_M("Service", "PROFILE_T state=unavailable reason=\"" << e.what()
                << "\" Profile without T (t_seg null); Status no_traversable_seg, t_projection_unavailable;"
                   " input-dependency gap");
        }
    }
    // 3. Steady state (service/profile_steady_loop.cpp, moved there unchanged 2026-10-14 so that
    //    tests/test_rns_profile_time2.cpp runs the same code, A19-TIME-2): the S and T slot
    //    adapters, the consumer over the owners above, the T ready flag, then one step per frame
    //    until finish. Its exceptions reach profile_owner as before.
    const service::ProfileSteadyInputs steady{s.mailbox, *g->geometry, *g->grid, g->health_roi, g->calibrated,
        &s.processed, estimated.native_clock(), channel,
        {&transport, rt_last_audit}, semantic.get(), s.semantic_slot, semantic_options, t_owner.get(), s.t_slot,
        s.t_state ? &s.t_state->ready : nullptr, t_options.max_age_ns, s.reasons,
        bit("ground_fit_fallback"), bit("ground_free_withdrawn"), *s.timing_handoff};
    service::run_profile_steady(ctl.finish, steady, s);
}

// Profile owner thread (the fast line). Profile is Q1 RT data like Objects.
// Capture times are the stamps the capture thread gave each frameset from the camera's frame
// timestamp (rns_capture_estimate.hpp); Status reports the fallback grade as t_capture_estimated.
void profile_owner(OwnerControl& ctl, ProfileShared& s) noexcept {
    try {
        perception::set_current_thread_name("svc_profile");
        perception::publish_profile_thread_tid();
        if (!s.timing_handoff) throw std::logic_error("Profile timing handoff not wired");
        SystemPublishClock clock;
        const int context = 0;
        EstimatedCaptureClock estimated({kEpoch, s.estimate_audit}, &context, kEpoch,
                                        [](const void*) noexcept { return monotonic_ns(); });
        if (!estimated.valid()) throw std::runtime_error("capture estimate configuration invalid");
        ZenohRtTransport transport(s.session, s.key, {Priority::RealTime, false, true, true}, kPacketCapacity);
        PublisherChannel channel(Topic::Profile, s.envelope, clock, transport, kPacketCapacity - 4096,
                                 kPacketCapacity, nullptr, &s.journal);
        if (s.config) configured_loop(ctl, s, estimated, transport, channel);
        else unconfigured_loop(ctl, s, estimated, transport, channel);
        log_pool("profile", transport);
    } catch (const std::exception& e) {
        ctl.fault(std::string("profile owner: ") + e.what());
    } catch (...) {
        ctl.fault("profile owner: nonstandard exception");
    }
}

// ---- GEN owner ----
// The GEN plane carries events only: the config digest at start, reason edges
// queued by Status, and RT router loss/restore notices.
struct GenShared {
    GenEvents& events;
    service::GenNotices& notices;
    ZenohRtSession& rt;
    service::ConfigDigestFacts facts;
    std::atomic<std::uint64_t> sent{0}, not_accepted{0};
};

void gen_owner(OwnerControl& ctl, GenShared& s) noexcept {
    try {
        perception::set_current_thread_name("svc_gen");
        const auto log = [&](const service::NoticeResult& r) {
            if (!r.attempted) return;
            if (r.receipt.outcome == PutOutcome::Accepted) s.sent.fetch_add(1, std::memory_order_relaxed);
            else s.not_accepted.fetch_add(1, std::memory_order_relaxed);
            LOG_INFO_M("Service", "GEN_NOTICE kind=" << r.kind << " key=" << r.key << " seq=" << r.seq
                           << " outcome=" << static_cast<int>(r.receipt.outcome));
        };
        log(s.notices.config_digest(s.facts));
        auto next_router_check = Steady::now();
        // Every 20 ms send up to 64 queued events; once per second check the RT
        // session's routers. On finish the queue is drained once more, then stop.
        while (true) {
            const bool finishing = ctl.finish.load(std::memory_order_acquire);
            for (int i = 0; i < 64; ++i) {
                const auto sent = s.events.send_one();
                if (sent.code == GenSendCode::Empty) break;
                if (sent.code == GenSendCode::Accepted) s.sent.fetch_add(1, std::memory_order_relaxed);
                else s.not_accepted.fetch_add(1, std::memory_order_relaxed);
            }
            if (finishing) break;
            if (Steady::now() >= next_router_check) {
                log(s.notices.observe_rt_routers(s.rt.connected_router_count()));
                next_router_check += std::chrono::seconds(1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    } catch (const std::exception& e) {
        ctl.fault(std::string("gen owner: ") + e.what());
    } catch (...) {
        ctl.fault("gen owner: nonstandard exception");
    }
}

// One stderr line the unit log keeps, then exit 78 (configuration refused).
int refuse(const service::StartupCheckError& e) {
    std::cerr << "PERCEPTION_STARTUP_REFUSED check=" << e.check() << " key=" << e.key() << " reason=\""
              << e.what() << "\"" << std::endl;
    return kExitConfig;
}

int run(char** envp, bool check_only) {
    // ---- 1. configuration and identity, before any logger/module reads the environment ----
    service::ServiceConfig cfg;
    service::PublisherKeys pub;
    try {
        service::reject_inherited_runtime_environment(envp);
        service::LoadInputs inputs;
        inputs.host = service::read_host_identity();
        // The test-only prefix override must never be active in the service binary.
        if (std::string(inputs.resolved_prefix) != service::kResolvedPrefix) return kExitConfig;
        cfg = service::load_service_config(inputs);
        // PSC-5 / A19-BOOT-4: the six keys the publishers below are given, compared literally with
        // the keys registered for this robot (declared_keys.hpp), before any session exists; an
        // unusable robot id refuses the start here as well.
        pub = service::publisher_keys(cfg.rid);
        service::require_declared_keys(cfg.rid, pub.all());
        // Geometry keys are validated before any device access; the ray table,
        // PSC-3 projection support and PSC-7 run on the first native frame.
        service::prevalidate_geometry_setup(cfg);
        service::apply_runtime_environment(cfg);
    } catch (const service::StartupCheckError& e) {
        return refuse(e);
    }
    // 19 section 7.3: read once, the only source of extrinsic_calibrated=true.
    const auto verdict = service::verify_calibration(cfg.contract.extrinsic, cfg.pipeline["camera"]["serial"].as<std::string>());
    const bool geometry_configured = cfg.contract.extrinsic.mode == service::ExtrinsicMode::TransformConfigured;
    // T channel (issues.md 3.1, 2026-10-08): only with configured geometry AND traversable_seg "on".
    // "off" (formal, the transition) or "unset" runs no T at all: Profile without T, Status
    // no_traversable_seg, which the integration lead expects while T is off.
    const char* const traversable_seg = std::getenv("PERCEPTION_TRAVERSABLE_SEG");
    const bool t_enabled = geometry_configured && traversable_seg != nullptr && std::strcmp(traversable_seg, "on") == 0;
    // The PSC-4 seal tool (psc4_seal_main.cpp) repeats this process preparation
    // in the same order; a divergence shows up as a PSC-4 refusal, never a pass.
    const int process_cv_threads = service::process_open_cv_threads(cfg);
    if (process_cv_threads) cv::setNumThreads(process_cv_threads);

    // The existing pipeline configuration from the implementation section.
    yolo::Config config;
    if (!config.load_from_node(cfg.pipeline, cfg.resolved_path)) {
        std::cerr << "PERCEPTION_STARTUP_REFUSED check=PSC-2 key=perception.pipeline reason=\"implementation section rejected by pipeline configuration\"" << std::endl;
        return kExitConfig;
    }
    yolo::Logger::instance().set_level(config.log_level());
    // The preprocess mode is read once from the (now applied) runtime environment; an invalid
    // value is a configuration refusal, not a fault at the first frame.
    try {
        (void)yolo::initialize_preprocess_mode_from_environment();
    } catch (const std::exception& e) {
        std::cerr << "PERCEPTION_STARTUP_REFUSED check=RUNTIME key=perception.pipeline.runtime.preprocess_impl reason=\"" << e.what() << "\"" << std::endl;
        return kExitConfig;
    }
    // PSC-4 (19 section 11 / 13.2): load the engines and run ONE real golden
    // frame against the sealed assets before anything else starts. A missing or
    // malformed asset, an identity mismatch, a failed or timed-out run, or a
    // different output refuses the start. No expected value is ever derived
    // here and no engine is ever rebuilt (reply batch 7, B7-3).
    {
        const auto golden = service::psc4::startup_check(cfg, config);
        if (!golden.ok) {
            std::cerr << "PERCEPTION_STARTUP_REFUSED check=PSC-4 key=" << golden.key << " reason=\"" << golden.reason
                      << "\"" << std::endl;
            return kExitConfig;
        }
        LOG_INFO_M("Service", golden.summary);
    }
    // Identity, switches, declared keys and calibration verdict go to the log
    // before anything runs, so every later line can be tied to this setup.
    LOG_INFO_M("Service", "SERVICE_IDENTITY rid=" << cfg.rid << " boot=" << cfg.boot8
        << " resolved_sha256=" << cfg.resolved_sha256 << " manifest_sha256=" << cfg.manifest_sha256
        << " config_rev=" << cfg.manifest.config_rev << " common_digest=" << cfg.manifest.common_digest
        << " runtime_digest=" << cfg.runtime_digest << " geometry=" << (geometry_configured ? "transform" : "absent")
        << " extrinsic_calibrated=" << verdict.calibrated() << " calibration=" << service::calibration_code_name(verdict.code())
        << " med2_port=" << cfg.contract.med2.rtsp_port << " med2_pub=" << (cfg.contract.med2.pub_port ? std::to_string(*cfg.contract.med2.pub_port) : "null")
        << " latency_fallback_ms=" << cfg.contract.latency_fallback_ms << "(validated, not used for timestamps)"
        << " capture_time=camera_frame_timestamp ts_sync_source=none");
    if (geometry_configured && !t_enabled)
        LOG_INFO_M("Service", "PROFILE_T state=off reason=\"traversable_seg off (issues.md 3.1: T off during the "
            "transition)\" Profile without T (t_seg null); Status no_traversable_seg");
    for (const auto& r : cfg.runtime)
        LOG_INFO_M("Service", "SERVICE_RUNTIME_SWITCH key=" << r.key << " env=" << (r.env.empty() ? "<service>" : r.env)
            << " value=" << (r.set ? r.value : "<unset>"));
    for (const auto& k : pub.all()) LOG_INFO_M("Service", "SERVICE_DECLARED_KEY " << k);
    {
        std::ostringstream line;
        line << "SERVICE_CALIBRATION code=" << service::calibration_code_name(verdict.code()) << " calibrated=" << verdict.calibrated()
             << " record=" << (verdict.record_path().empty() ? "null" : verdict.record_path())
             << " record_sha256=" << (verdict.record_sha256().empty() ? "null" : verdict.record_sha256())
             << " record_id=" << (verdict.record_id().empty() ? "null" : verdict.record_id())
             << " inlier_rms_m=" << (verdict.inlier_rms() ? std::to_string(*verdict.inlier_rms()) : std::string("null"))
             << " detail=\"" << verdict.detail() << "\"";
        const bool rejected = verdict.code() != service::CalibrationCode::NoTransform &&
            verdict.code() != service::CalibrationCode::TransformOnly && verdict.code() != service::CalibrationCode::Verified;
        if (rejected) LOG_WARN_M("Service", line.str());
        else LOG_INFO_M("Service", line.str());
    }
    // --check-config stops here: every startup check except device access and the CPU set of
    // the Profile thread placement ran (that set is the service unit's, not the caller's).
    if (check_only) {
        (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
        std::cout << "PERCEPTION_CONFIG_CHECK_PASS rid=" << cfg.rid << " resolved_sha256=" << cfg.resolved_sha256
                  << " runtime_digest=" << cfg.runtime_digest << std::endl;
        return 0;
    }
    // The Profile thread placement is planned from the main thread's set, the one the service was
    // started with: read here, before the camera start narrows any thread. A set that does not
    // allow the configured placement refuses the start.
    ProfileThreadPlan profile_plan;
    try {
        const auto inherited = read_capture_affinity();
        profile_plan = service::profile_thread_plan(cfg, inherited.valid, inherited.mask);
    } catch (const service::StartupCheckError& e) {
        (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
        return refuse(e);
    }
    if (profile_plan.ok) LOG_INFO_M("Service", profile_plan);

    // ---- 2. sessions, envelope, journals ----
    std::unique_ptr<ZenohRtSession> rt;
    std::unique_ptr<ZenohGenSession> gen_session;
    try {
        rt = std::make_unique<ZenohRtSession>(cfg.contract.rt_endpoint);
        gen_session = std::make_unique<ZenohGenSession>(cfg.contract.gen_endpoint);
    } catch (const std::exception& e) {
        LOG_ERROR_M("Service", "SERVICE_UNAVAILABLE zenoh session: " << e.what());
        (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
        return kExitUnavailable;
    }
    // One envelope (11 section 3.0 seq counter) shared by every publisher.
    ProcessEnvelope envelope(cfg.rid, cfg.boot8, kClockStatusTimeoutMs);
    if (!envelope.valid()) throw std::runtime_error("process envelope invalid");
    SystemPublishClock main_clock;
    const Millis startup = main_clock.mono_millis();
    // Journals: publication outcomes (Profile, Objects) and source events
    // (depth arrival, inference completion, processed depth) for Status.
    PublicationJournal profile_journal(Topic::Profile, kJournalCapacity), objects_journal(Topic::Objects, kJournalCapacity);
    auto arrival = std::make_shared<SourceJournal>(SourceKind::DepthArrival, kSourceCapacity, SourceClock{nullptr, &source_now});
    auto inference = std::make_shared<SourceJournal>(SourceKind::InferenceComplete, kSourceCapacity, SourceClock{nullptr, &source_now});
    SourceJournal processed(SourceKind::DepthProcessed, kSourceCapacity, SourceClock{nullptr, &source_now});
    auto mailbox = std::make_shared<nd::Mailbox>(kPixels);

    // ---- 3. GEN publishers and owners' shared state ----
    // The three event keys (publisher_keys; checked against the registry at startup, PSC-5).
    const std::string nonce = random_nonce();
    ZenohGenTransport gen_warn(*gen_session, pub.warn, GenEvents::qos(), GenEvents::kMaxPacket);
    ZenohGenTransport gen_info(*gen_session, pub.info, GenEvents::qos(), GenEvents::kMaxPacket);
    ZenohGenTransport gen_fault(*gen_session, pub.fault, GenEvents::qos(), GenEvents::kMaxPacket);
    SystemPublishClock gen_clock;
    GenEvents gen_events(envelope, gen_clock, gen_warn, gen_info, nonce, reason_names(), 256, GenEvents::kMaxPacket);
    service::GenNotices notices(envelope, gen_clock, gen_info, gen_fault, nonce, cfg.contract.rt_endpoint);

    // Where the capture times come from, for the logs: the capture clock on the camera's frame
    // timestamp with the calibrated shortest path of the runtime switch capture_min_path_us.
    // perception.capture.latency_fallback_ms is still validated and logged, and read by nothing.
    const char* const min_path_text = std::getenv("PERCEPTION_CAPTURE_MIN_PATH_US");
    const std::string estimate_audit = std::string("capture clock on the camera frame timestamp, capture_min_path_us=") +
        (min_path_text ? min_path_text : "unset") + " (19 section 2.4; PD-11 measured 2026-10-13/14)";

    // R6 wiring handed to the Supervisor: journals, clock, capture stamps and
    // the Objects callback (see SupervisorRnsContext).
    auto objects = std::make_shared<ObjectsOwner>(envelope, *rt, pub.objects, startup, objects_journal);
    // Paced Objects release (runtime switch objects_release_delay_ms, 2026-10-10; 0 when unset):
    // with a delay the Objects callback hands each body to a release thread, which becomes the
    // one owner of the Objects channel and publishes the latest body once the delay after its ready
    // time (or after that of a body it replaced) has passed (rns_objects_release.hpp). 0 builds nothing:
    // the callback publishes as before. The switch table limits the values and requires the
    // two-slot JIT loop for a delay. objects_release_log "1": one OBJ_RELEASE line per body.
    const char* const release_delay_text = std::getenv("PERCEPTION_OBJECTS_RELEASE_DELAY_MS");
    const std::int64_t release_delay_ms = release_delay_text ? std::strtoll(release_delay_text, nullptr, 10) : 0;
    const char* const release_log_text = std::getenv("PERCEPTION_OBJECTS_RELEASE_LOG");
    const bool release_log = release_log_text != nullptr && std::string(release_log_text) == "1";
    std::shared_ptr<ObjectsRelease> release;
    if (release_delay_ms > 0) {
        ObjectsRelease::Observer observer;
        if (release_log) {
            observer = [](const ReleaseRecord& r) {
                LOG_INFO_M("Service", "OBJ_RELEASE frame_id=" << r.frame_id << " end=" << release_end_name(r.end)
                    << " reason=" << release_reason_name(r.reason) << " capture_ms=" << r.capture_ms
                    << " objects=" << r.objects << " ready_ns=" << r.ready_ns << " offer_begin_ns=" << r.offer_begin_ns
                    << " handoff_ns=" << r.handoff_ns
                    << " deadline_ns=" << r.deadline_ns << " due_ns=" << r.due_ns << " claim_ns=" << r.claim_ns
                    << " call_ns=" << r.call_ns
                    << " return_ns=" << r.return_ns << " put_before_ms=" << r.result.audit.put_before_ms
                    << " put_return_ms=" << r.result.audit.put_return_ms << " seq=" << r.result.audit.seq
                    << " error=" << static_cast<int>(r.result.error) << " replaced_by=" << r.replaced_by);
            };
        }
        release = std::make_shared<ObjectsRelease>(release_delay_ms * INT64_C(1000000), [] { return monotonic_ns(); },
            [objects](const Objects& body) { return objects->publish_result(body); }, std::move(observer), "svc_objects");
        if (!release->valid() || !release->start()) throw std::runtime_error("Objects release could not be started");
    }
    LOG_INFO_M("Service", "OBJECTS_RELEASE_CONFIG delay_ms=" << release_delay_ms << " log=" << release_log
        << " publisher=" << (release ? "release_thread" : "objects_callback"));
    SupervisorRnsContext wiring;
    wiring.native_mailbox = mailbox;
    wiring.depth_arrivals = arrival;
    wiring.inference_completions = inference;
    wiring.clock_epoch = kEpoch;
    wiring.extrinsic_calibrated = verdict.calibrated();
    wiring.callback_owner = objects;
    wiring.now_ns = [] { return monotonic_ns(); };
    // Objects capture stamps are the frame's own stamp, the one the Profile reads too, so S and T
    // ages (depth capture minus object capture) compare like with like.
    wiring.capture_stamp = [](const nd::SourceId& source, geometry::CaptureStamp& stamp) {
        nd::Metadata metadata;
        metadata.source = source;
        const auto estimate = estimate_capture(metadata, {kEpoch, "capture clock (capture_min_path_us)"},
                                              kEpoch, monotonic_ns());
        if (estimate.status != CaptureEstimateStatus::Ready) return false;
        stamp = estimate.capture;
        return true;
    };
    auto object_geometry = std::make_shared<ObjectGeometryShared>();
    // 19 section 3.4A latest-wins slot: the inference thread publishes measured
    // footprints, the Profile thread reads the newest one. Only with a
    // configured geometry is there a base_link footprint to publish at all.
    auto semantic_slot = std::make_shared<service::SemanticSlot>();
    // T channel (19 section 3.3, P19-1/P19-2, plan 2f): the Objects side publishes the
    // raw class-92 union of each inference frame; the Profile thread reads the newest.
    auto t_slot = std::make_shared<service::TMaskSlot>();
    auto t_state = std::make_shared<TProducerState>();
    // Frame-level Objects reason (B4-3): the state of the latest built Objects
    // frame; only a ready frame clears it, heartbeats never do.
    auto objects_reasons = std::make_shared<service::ProfileReasonState>(bit("object_geometry_unavailable"));
    // Called once per built Objects body: update the Objects frame reason, add
    // the body counters, feed the S slot, then publish (or hand the body to the paced release).
    wiring.objects_ready = [objects, release, object_geometry, objects_reasons, geometry_configured, t_enabled, semantic_slot,
                            t_slot, t_state](const yolo::DetectionResult& result, const geometry::CaptureStamp& stamp,
                                     const Objects& body, const FusedObjectsReport& report) {
        const bool miss = object_geometry->frame_capability_miss.exchange(false, std::memory_order_relaxed);
        const bool unavailable = geometry_configured && (miss || !object_geometry->ready.load(std::memory_order_relaxed));
        (void)objects_reasons->commit_frame(unavailable ? bit("object_geometry_unavailable") : 0);
        object_geometry->body_located.fetch_add(report.located, std::memory_order_relaxed);
        object_geometry->body_unlocatable.fetch_add(report.unlocatable, std::memory_order_relaxed);
        object_geometry->velocity_numeric.fetch_add(report.velocity_numeric, std::memory_order_relaxed);
        object_geometry->velocity_gating_mapped.fetch_add(report.velocity_gating_mapped, std::memory_order_relaxed);
        object_geometry->velocity_unconfirmed_numeric.fetch_add(report.velocity_unconfirmed_numeric, std::memory_order_relaxed);
        object_geometry->velocity_gated_track_unstable.fetch_add(report.velocity_gated_track_unstable, std::memory_order_relaxed);
        object_geometry->velocity_gated_low_depth_confidence.fetch_add(report.velocity_gated_low_depth_confidence, std::memory_order_relaxed);
        object_geometry->velocity_gated_depth_unreliable.fetch_add(report.velocity_gated_depth_unreliable, std::memory_order_relaxed);
        // The S channel carries the SAME measured footprints this body carries.
        // Publishing is unconditional on the Objects side: an all-unlocatable
        // body simply publishes an empty snapshot, which the projector reads as
        // "nothing detected here", never as "no observation".
        if (geometry_configured) semantic_slot->publish(body, stamp, service::kSemanticBaseRevision);
        // With the paced release "published" means handed over: the call does not wait, and a
        // body refused because the service is stopping is not a failure (objects_delivery.hpp).
        const bool published = service::deliver_objects(
            release.get(), [&objects](const Objects& b) { return objects->publish(b); }, body, result.frame_id,
            result.ready_observed_mono_ns);
        // T channel (plan 2f), after the Objects body is out or handed over: the raw class-92 union of
        // this inference frame (11 section 3.1B.1: raw, never safe_traversable), same
        // capture stamp as the S snapshot. Only once the Profile thread has a T owner.
        // No class 92 at all -> an all-zero mask ("looked, nothing traversable";
        // accepted, issues.md 3.2(1)). A producer fault disables T for this run and is
        // logged loudly; Objects and Profile go on (user decision 2026-09-30, B9-9 (c)).
        // Nothing at all with traversable_seg off (issues.md 3.1).
        if (t_enabled) {
            std::string fault;
            if (service::produce_traversable(*t_slot, *t_state, result.map_detections, stamp, &fault) ==
                service::TProduceOutcome::Fault)
                LOG_ERROR_M("Service", "T_PRODUCER_FAULT frame_id=" << result.frame_id << " source_sequence="
                    << stamp.source.sequence << " error=\"" << fault
                    << "\"; T disabled for this run, Objects and Profile continue, no_traversable_seg within 10 s");
        }
        return published;
    };
    if (geometry_configured) {
        wiring.object_geometry = [object_geometry](const yolo::DetectionResult& result, const cv::Rect& bbox,
                                                   const cv::Mat& mask, float lo, float hi,
                                                   const std::vector<std::uint32_t>* selected_pixels,
                                                   FusedDetection& detection) {
            measure_object_geometry(*object_geometry, result, bbox, mask, lo, hi, selected_pixels, detection);
        };
    }

    // Per-owner fault latches and the state each owner thread reads.
    OwnerControl profile_ctl, status_ctl, gen_ctl;
    // The Profile thread's timing figures, logged by the Status thread; outlives both threads.
    service::ProfileTimingHandoff profile_timing;
    // The leading {} is the counters base (service::ProfileCounters), all zero.
    ProfileShared profile_shared{{}, *mailbox, processed, envelope, *rt, pub.profile, profile_journal,
        {static_cast<float>(cfg.contract.profile.z_pass_m), static_cast<float>(cfg.contract.profile.angle_min_rad),
         static_cast<float>(cfg.contract.profile.angle_step_rad), static_cast<float>(cfg.contract.profile.range_max_m),
         cfg.contract.profile.n_bins, kEpoch},
        estimate_audit, geometry_configured ? &cfg : nullptr, &verdict,
        geometry_configured ? semantic_slot.get() : nullptr};
    profile_shared.t_slot = t_enabled ? t_slot.get() : nullptr;
    profile_shared.t_state = t_state.get();
    profile_shared.timing_handoff = &profile_timing;
    StatusShared status_shared{envelope, *rt, pub.status, startup, profile_journal, objects_journal, *arrival, *inference,
        processed, gen_events, !geometry_configured, !cfg.contract.med2.pub_port, verdict.calibrated(),
        profile_shared.reasons, *objects_reasons};
    status_shared.t_slot = t_enabled ? t_slot.get() : nullptr;
    status_shared.t_state = t_state.get();
    status_shared.profile_timing = &profile_timing;
    std::unique_ptr<ProfileThreadPlacement> profile_placement;
    if (profile_plan.ok) profile_placement = std::make_unique<ProfileThreadPlacement>(profile_plan);
    status_shared.profile_placement = profile_placement.get();
    GenShared gen_shared{gen_events, notices, *rt,
        {cfg.manifest.common_digest, cfg.manifest.config_rev, cfg.resolved_sha256, cfg.manifest_sha256, cfg.runtime_digest,
         geometry_configured ? "transform" : "absent", service::calibration_code_name(verdict.code()), verdict.calibrated(),
         verdict.record_sha256(), verdict.detail()}};

    // The existing capture/inference/tracking pipeline, R6-wired before init.
    perception::Supervisor pipeline;
    if (!pipeline.configure_rns(std::move(wiring))) {
        LOG_ERROR_M("Service", "SERVICE_FAULT R6 Supervisor wiring rejected");
        (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
        return kExitRuntime;
    }
    if (!pipeline.init(config)) {
        LOG_ERROR_M("Service", "SERVICE_UNAVAILABLE pipeline init failed (camera/model/encoder)");
        (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
        return kExitUnavailable;
    }
    // The Profile thread reads this session's calibration for the T projection.
    profile_shared.pipeline = &pipeline;
    // Optional ROS executor (TF listener + capture command only, P19-4).
    std::unique_ptr<service::RosBridge> ros;
    if (cfg.ros.enable) {
        try {
            ros = std::make_unique<service::RosBridge>(cfg.ros, pipeline);
            ros->attach_pose_provider();
        } catch (const std::exception& e) {
            LOG_ERROR_M("Service", "SERVICE_UNAVAILABLE ROS executor: " << e.what());
            pipeline.stop();
            (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
            return kExitUnavailable;
        }
    }

    // Owner threads start before the camera, ready when the first frame arrives.
    std::thread gen_thread([&] { gen_owner(gen_ctl, gen_shared); });
    std::thread status_thread([&] { status_owner(status_ctl, status_shared); });
    std::thread profile_thread([&] { profile_owner(profile_ctl, profile_shared); });

    int exit_code = 0;
    std::string stop_reason;
    if (!pipeline.start()) {
        stop_reason = "pipeline start failed (camera/model/encoder)";
        exit_code = kExitUnavailable;
    } else {
        if (ros) ros->start();
        // Object geometry capability for this capture session (B4-2 registration,
        // B4-3 outcomes): ready; capability not available -> keep publishing with
        // unlocatable objects and the object_geometry_unavailable reason; any
        // other exception is a configuration / program fault (runtime exit).
        // Done after pipeline.start(): the session calibration exists only once the camera runs.
        std::string object_geometry_state = geometry_configured ? "unavailable" : "absent_no_transform";
        bool setup_fault = false;
        if (geometry_configured) {
            try {
                const auto calibration = pipeline.rgbd_calibration();
                service::ObjectGeometrySession session;
                if (calibration) {
                    session.present = true;
                    session.depth_to_colour_R = calibration->depth_to_colour_R;
                    session.depth_to_colour_t_m = calibration->depth_to_colour_t_m;
                    session.registration = calibration->registration;
                    session.registration_status = calibration->registration_status;
                }
                const ObjectGeometryOptions geometry_options = object_geometry_options();
                auto ready = service::setup_object_geometry(session, cfg.contract_yaml, geometry_options);
                {
                    std::lock_guard<std::mutex> lock(object_geometry->mutex);
                    object_geometry->base_from_colour_R = ready.base_from_colour_R;
                    object_geometry->colour = calibration->colour;
                    object_geometry->builder = std::move(ready.builder);
                    object_geometry->registration = ready.registration;
                }
                object_geometry->ready.store(true, std::memory_order_relaxed);
                object_geometry_state = "ready";
                const auto& t = ready.base_from_colour.t;
                LOG_INFO_M("Service", "SERVICE_OBJECT_GEOMETRY state=ready extrinsic_calibrated=" << verdict.calibrated()
                    << " depth_raster=rgb_registered max_shift_px=" << calibration->registration->max_shift_px()
                    << " device=\"" << calibration->device_name << "\" firmware=" << calibration->firmware
                    << " sdk=" << calibration->sdk_version << " colour=" << calibration->colour.width << "x"
                    << calibration->colour.height << " base_from_colour_t=[" << t[0] << "," << t[1] << "," << t[2]
                    << "] min_points=" << kObjectGeometryOptions.min_points << " bucket_m=" << kObjectGeometryOptions.bucket_m
                    << " degenerate_half_width_m=" << kObjectGeometryOptions.degenerate_half_width_m
                    << " float_margin_m=" << geometry_options.float_margin_m);
            } catch (const service::ObjectGeometryCapabilityError& e) {
                object_geometry_state = std::string("unavailable: ") + e.what();
                LOG_WARN_M("Service", "SERVICE_OBJECT_GEOMETRY state=unavailable reason=\"" << e.what()
                    << "\" objects stay unlocatable; Status reason object_geometry_unavailable");
            } catch (const std::exception& e) {
                setup_fault = true;
                stop_reason = std::string("object geometry setup fault: ") + e.what();
                exit_code = kExitRuntime;
                LOG_ERROR_M("Service", "SERVICE_FAULT " << stop_reason);
            }
        }
        // Main loop (10 ms): stop on a signal, an owner fault (exit 78 for a
        // first-frame refusal, else 70) or a pipeline fault; log a summary
        // every 10 s.
        if (!setup_fault) LOG_INFO_M("Service", "SERVICE_RUNNING rid=" << cfg.rid << " keys=" << pub.all().size() << " ros=" << (ros ? 1 : 0));
        auto next_summary = Steady::now() + std::chrono::seconds(10);
        while (!setup_fault) {
            if (const int signo = g_signal.load(std::memory_order_relaxed)) {
                stop_reason = std::string("signal ") + std::to_string(signo);
                break;
            }
            // An owner fault: collect every owner's first reason (locks always taken in this
            // order, the only place more than one is held). A first-frame refusal exits 78.
            if (profile_ctl.faulted || status_ctl.faulted || gen_ctl.faulted) {
                std::lock_guard<std::mutex> a(profile_ctl.mutex);
                std::lock_guard<std::mutex> b(status_ctl.mutex);
                std::lock_guard<std::mutex> c(gen_ctl.mutex);
                stop_reason = profile_ctl.reason + status_ctl.reason + gen_ctl.reason;
                exit_code = profile_ctl.refused.load(std::memory_order_acquire) ? kExitConfig : kExitRuntime;
                break;
            }
            // A fault of the paced release (a publish contract fault, a frame without a usable
            // ready time): seen here whether or not another frame arrives. Checked before the
            // pipeline, whose callback fails on the same fault, so that the reason is the cause.
            if (service::release_fault(release.get(), stop_reason)) {
                exit_code = kExitRuntime;
                break;
            }
            if (pipeline.fatal_failure_detected() || pipeline.shutdown_requested()) {
                stop_reason = pipeline.fatal_failure_detected() ? "pipeline fatal failure" : "pipeline shutdown requested";
                exit_code = kExitRuntime;
                break;
            }
            if (Steady::now() >= next_summary) {
                next_summary += std::chrono::seconds(10);
                LOG_INFO_M("Service", "SERVICE_SUMMARY objects_attempts=" << objects->counters.channel_calls << " objects_accepted="
                    << objects->counters.accepted << " objects_rejected=" << objects->counters.rejected << " objects_unknown=" << objects->counters.unknown
                    << " profile_puts=" << profile_shared.puts << " profile_abnormal=" << profile_shared.abnormal
                    << " profile_route_normal=" << profile_shared.route_normal << " profile_route_no_ground="
                    << profile_shared.route_no_ground << " profile_route_withdrawn=" << profile_shared.route_withdrawn
                    << " status_beats=" << status_shared.beats << " status_failures=" << status_shared.beat_failures
                    << " gen_sent=" << gen_shared.sent << " gen_not_accepted=" << gen_shared.not_accepted
                    << " edges_offered=" << status_shared.edges_offered << " edges_lost=" << status_shared.edges_lost
                    << " rt_router_state=" << static_cast<int>(notices.rt_state())
                    << " objgeo_state=\"" << object_geometry_state << "\" objgeo_located=" << object_geometry->located << " objgeo_too_few=" << object_geometry->too_few_points
                    << " objgeo_nonfinite=" << object_geometry->non_finite << " objgeo_invalid=" << object_geometry->invalid_input
                    << " objgeo_not_ready=" << object_geometry->not_ready << " objgeo_raster_mismatch="
                    << object_geometry->raster_mismatch << " objgeo_mask_unusable=" << object_geometry->mask_unusable
                    << " objects_located=" << object_geometry->body_located << " objects_unlocatable="
                    << object_geometry->body_unlocatable << " velocity_numeric=" << object_geometry->velocity_numeric
                    << " velocity_gating_mapped=" << object_geometry->velocity_gating_mapped
                    << " velocity_unconfirmed_numeric=" << object_geometry->velocity_unconfirmed_numeric
                    << " velocity_gated_track_unstable=" << object_geometry->velocity_gated_track_unstable
                    << " velocity_gated_low_depth_confidence=" << object_geometry->velocity_gated_low_depth_confidence
                    << " velocity_gated_depth_unreliable=" << object_geometry->velocity_gated_depth_unreliable
                    << " objgeo_build_calls=" << object_geometry->build_calls << " objgeo_build_ns=" << object_geometry->build_ns
                    << " objgeo_build_max_ns=" << object_geometry->build_max_ns << " objgeo_build_points="
                    << object_geometry->build_points << " objgeo_selected_list_used=" << object_geometry->selected_list_used
                    << " objgeo_selected_list_rejected=" << object_geometry->selected_list_rejected
                    << " objgeo_float_points=" << object_geometry->float_points
                    << " objgeo_coast_skipped=" << object_geometry->coast_skipped
                    << " objgeo_coast_skipped_points=" << object_geometry->coast_skipped_points
                    << " sem_published=" << semantic_slot->published() << " sem_objects=" << semantic_slot->objects_published()
                    << " sem_taken=" << semantic_slot->taken()
                    << " sem_skipped_unlocatable=" << semantic_slot->skipped_unlocatable()
                    << " sem_skipped_overflow=" << semantic_slot->skipped_overflow()
                    << " sem_skipped_name=" << semantic_slot->skipped_name()
                    << " sem_skipped_vertices=" << semantic_slot->skipped_vertices()
                    << " sem_refused_future=" << semantic_slot->refused_future()
                    << " sem_refused_stream=" << semantic_slot->refused_stream()
                    << " sem_refused_epoch=" << semantic_slot->refused_epoch()
                    << " sem_fresh=" << profile_shared.semantic_fresh << " sem_missing=" << profile_shared.semantic_missing
                    << " sem_stale=" << profile_shared.semantic_stale << " sem_other=" << profile_shared.semantic_other
                    << " sem_bins=" << profile_shared.semantic_bins
                    << " sem_frames_with_bins=" << profile_shared.semantic_frames_with_bins
                    << " sem_stale_guard=" << profile_shared.semantic_stale_guard
                    << " sem_expired_at_publish=" << profile_shared.semantic_expired_publish
                    << " no_ground_refused=" << profile_shared.no_ground_refused);
                {
                    const auto tc = t_slot->counters();
                    LOG_INFO_M("Service", "SERVICE_T_SUMMARY enabled=" << t_enabled << " ready=" << t_state->ready.load()
                        << " projection_unavailable=" << t_state->projection_unavailable.load() << " locked="
                        << t_state->locked.load() << " producer_faults=" << t_state->faults
                        << " raster_mismatch=" << t_state->raster_mismatch << " seg_in_open_ms=" << t_state->seg_in_open_ms
                        << " clock_anomaly=" << t_state->clock_anomaly
                        << " published=" << tc.published << " published_empty=" << tc.published_empty
                        << " refused_input=" << tc.refused_input << " refused_concurrent=" << tc.refused_concurrent
                        << " taken=" << tc.taken << " reused=" << tc.reused << " missing=" << tc.missing
                        << " refused_stream=" << tc.refused_stream << " refused_epoch=" << tc.refused_epoch
                        << " refused_revision=" << tc.refused_revision << " refused_future=" << tc.refused_future
                        << " refused_order=" << tc.refused_order << " torn_retries=" << tc.torn_retries
                        << " torn_giveup=" << tc.torn_giveup << " t_fresh=" << profile_shared.t_fresh
                        << " t_missing=" << profile_shared.t_missing << " t_stale=" << profile_shared.t_stale
                        << " t_geometry_unavailable=" << profile_shared.t_geometry_unavailable
                        << " t_other=" << profile_shared.t_other
                        << " t_near_support_missing=" << profile_shared.t_near_support_missing
                        << " t_bins=" << profile_shared.t_bins << " t_frames_with_bins=" << profile_shared.t_frames_with_bins);
                }
                LOG_INFO_M("Service", "RESULT_HANDOFF " << result_handoff_fields(pipeline.drain_result_handoff()));
                if (release) LOG_INFO_M("Service", "OBJECTS_RELEASE_SUMMARY delay_ms=" << release_delay_ms << " "
                    << release_fields(release->counters()));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    LOG_INFO_M("Service", "SERVICE_STOPPING reason=\"" << stop_reason << "\" exit_code=" << exit_code);

    // ---- 4. ordered shutdown: fast line, pipeline (retires the mailbox), status, GEN drain ----
    // The Profile owner stops first: it reads the mailbox the pipeline retires on stop. Status
    // keeps running until the pipeline has stopped (pipeline threads write its source journals),
    // and GEN stops last: on finish it drains the edge queue once more (gen_owner).
    // The paced Objects release, when on, is closed before the pipeline stops (bodies the
    // pipeline still finishes are refused and counted, what has no publication commitment is
    // discarded, a committed call completes) and joined after it. Status keeps running until the
    // release thread has ended, so the journal it reads outlives the publisher; it is NOT
    // guaranteed to take another cut after the last completion (its loop may be asleep and is
    // then told to finish), the final journal is checked on its own. A committed call can
    // return a contract fault after the main loop above has left: the fault latch is read again
    // after the join and merged into the exit code.
    profile_ctl.finish.store(true, std::memory_order_release);
    if (profile_thread.joinable()) profile_thread.join();
    if (ros) ros->stop();  // no capture command or TF provider call during pipeline stop
    if (release) release->close();
    pipeline.stop();
    const service::ReleaseStopSummary release_stop = service::join_objects_release(release.get());
    if (release_stop.faulted) {
        const int before = exit_code;
        exit_code = service::merge_release_fault(exit_code, stop_reason, release_stop, kExitRuntime);
        LOG_ERROR_M("Service", "OBJECTS_RELEASE_FAULT exit_code_before_join=" << before << " exit_code=" << exit_code
            << " reason=\"" << stop_reason << "\"");
    }
    const auto audit = pipeline.runtime_audit();
    status_ctl.finish.store(true, std::memory_order_release);
    if (status_thread.joinable()) status_thread.join();
    gen_ctl.finish.store(true, std::memory_order_release);
    if (gen_thread.joinable()) gen_thread.join();
    // The Objects publisher (Supervisor or release thread) has ended above: its counters are final.
    log_pool("objects", objects->transport);
    LOG_INFO_M("Service", "SERVICE_STOPPED exit_code=" << exit_code << " audit_available=" << audit.available
        << " camera_running=" << audit.camera_running << " inference_running=" << audit.inference_running
        << " sdk_stop_failures=" << audit.sdk_stop_failures << " encoded_frames=" << audit.encoded_frames
        << " encode_dropped=" << audit.encode_dropped << " input_dropped=" << audit.input_dropped
        << " result_dropped=" << audit.result_dropped << " rns_callback_attempts=" << audit.rns_callback_attempts
        << " rns_callback_successes=" << audit.rns_callback_successes
        << " objects_accepted=" << objects->counters.accepted << " profile_puts=" << profile_shared.puts
        << " profile_setup_frames=" << profile_shared.setup_frames << " profile_route_normal=" << profile_shared.route_normal
        << " profile_route_no_ground=" << profile_shared.route_no_ground << " profile_route_withdrawn=" << profile_shared.route_withdrawn
        << " profile_clock_rejected=" << profile_shared.clock_rejected << " profile_geometry_unavailable="
        << profile_shared.geometry_unavailable << " profile_grid_unavailable=" << profile_shared.grid_unavailable
        << " profile_body_unavailable=" << profile_shared.body_unavailable
        << " status_beats=" << status_shared.beats << " gen_sent=" << gen_shared.sent);
    {
        // Every result inference pushed is popped, overwritten or cleared (by
        // frame_id); every popped result reaches exactly one Objects callback
        // and one publish unless the run stopped on a fault.
        const auto handoff = pipeline.drain_result_handoff();
        const auto& l = handoff.ledger;
        const std::uint64_t attempts = objects->counters.channel_calls, accepted = objects->counters.accepted,
                            rejected = objects->counters.rejected, unknown = objects->counters.unknown;
        LOG_INFO_M("Service", "RESULT_HANDOFF_FINAL " << result_handoff_fields(handoff)
            << " input_balanced=" << handoff.input.balanced() << " result_balanced=" << handoff.result.balanced()
            << " ledger_matches_queue=" << (l.popped == handoff.result.popped &&
                                            l.overwritten == handoff.result.dropped &&
                                            l.cleared == handoff.result.cleared)
            << " rns_prepared=" << audit.rns_prepared << " rns_callback_attempts=" << audit.rns_callback_attempts
            << " rns_callback_failures=" << (audit.rns_callback_attempts - audit.rns_callback_successes)
            << " popped_minus_callback_attempts="
            << static_cast<std::int64_t>(l.popped) - static_cast<std::int64_t>(audit.rns_callback_attempts)
            << " objects_attempts=" << attempts << " objects_accepted=" << accepted << " objects_rejected=" << rejected
            << " objects_unknown=" << unknown
            << " objects_attempts_minus_callbacks="
            << static_cast<std::int64_t>(attempts) - static_cast<std::int64_t>(audit.rns_callback_attempts)
            << " objects_unaccounted="
            << static_cast<std::int64_t>(attempts) - static_cast<std::int64_t>(accepted + rejected + unknown)
            << " objects_release=" << (release ? 1 : 0));
        // With the paced release a callback hands a body over, it does not publish it, so
        // objects_attempts_minus_callbacks above is no longer 0 by rule (overwritten and discarded
        // bodies have no channel call). The two conservations that replace it: every callback is
        // one offer and every admitted body has exactly one end (unaccounted=0); every committed
        // call of the release that has returned is one channel call counted by the owner
        // (objects_attempts is the owner's channel_calls, see ObjectsOwner).
        if (release) {
            const auto c = release->counters();
            LOG_INFO_M("Service", "OBJECTS_RELEASE_FINAL delay_ms=" << release_delay_ms << " " << release_fields(c)
                << " offered_minus_callback_attempts="
                << static_cast<std::int64_t>(c.offered) - static_cast<std::int64_t>(audit.rns_callback_attempts)
                << " objects_attempts_minus_channel_calls="
                << static_cast<std::int64_t>(attempts) - static_cast<std::int64_t>(c.published + c.channel_refused)
                << " faulted=" << release_stop.faulted);
        }
    }
    if (profile_ctl.refused.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(profile_ctl.mutex);
        std::cerr << "PERCEPTION_STARTUP_REFUSED check=" << profile_ctl.refused_check << " key=" << profile_ctl.refused_key
                  << " reason=\"" << profile_ctl.reason << "\"" << std::endl;
    }
    // A clean exit also requires a clean pipeline stop (camera and inference
    // stopped, no SDK stop failure); otherwise report a runtime fault.
    if (exit_code == 0 && (!audit.available || audit.camera_running || audit.inference_running || audit.sdk_stop_failures))
        exit_code = kExitRuntime;
    (void)yolo::Logger::instance().flush_for(std::chrono::seconds(2));
    return exit_code;
}
}  // namespace

// Entry: optional --check-config, signal handlers (SIGTERM/SIGINT stop,
// SIGPIPE ignored), run(), bounded log drain, exit without static destructors.
int main(int argc, char** argv, char** envp) {
    bool check_only = false;
    if (argc == 2 && std::string(argv[1]) == "--check-config") {
        check_only = true;
    } else if (argc != 1) {
        std::cerr << "usage: perception_node [--check-config]  (reads " << service::kResolvedConfigPath << ")"
                  << std::endl;
        return kExitConfig;
    }
    struct sigaction action {};
    action.sa_handler = on_signal;
    sigemptyset(&action.sa_mask);
    ::sigaction(SIGTERM, &action, nullptr);
    ::sigaction(SIGINT, &action, nullptr);
    ::signal(SIGPIPE, SIG_IGN);
    int code = kExitRuntime;
    try {
        code = run(envp, check_only);
    } catch (const service::StartupCheckError& e) {
        code = refuse(e);
    } catch (const std::exception& e) {
        std::cerr << "PERCEPTION_RUNTIME_FAULT " << e.what() << std::endl;
        code = kExitRuntime;
    }
    // Every owned resource was released inside run(). Bound the final log drain
    // and skip static destructors, so a blocked stdout cannot hang the exit.
    const auto drained = yolo::Logger::instance().flush_for(std::chrono::seconds(2));
    const auto stats = yolo::Logger::instance().async_stats();
    if (drained != yolo::Logger::FlushResult::Drained || stats.write_failures || stats.truncated_lines) {
        char text[256];
        const int len = std::snprintf(text, sizeof(text),
            "PERCEPTION_LOG_EXIT drain=%d write_failures=%llu failed_batch_lines=%llu truncated=%llu order_timeouts=%llu\n",
            static_cast<int>(drained), static_cast<unsigned long long>(stats.write_failures),
            static_cast<unsigned long long>(stats.failed_batch_lines), static_cast<unsigned long long>(stats.truncated_lines),
            static_cast<unsigned long long>(stats.error_order_timeouts));
        if (len > 0) (void)!::write(STDERR_FILENO, text, static_cast<std::size_t>(std::min(len, 255)));
    }
    std::_Exit(code);
}
