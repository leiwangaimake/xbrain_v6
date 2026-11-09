/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_health_fields.hpp
 * Brief: Cross-checks the 1 Hz Status health inputs and copies them into the message.
 *
 * Description:
 * What problem this solves. The Status message (19 section 5.1) carries the measured health of
 * the service: fps_depth, fps_infer, latency_ms_p50/p99, infer_gap_ms_p99/max,
 * invalid_pixel_ratio, ground_seg_level, extrinsic_calibrated, traversable_seg_available and the
 * degraded_reasons list. Each number comes from a different owner (publication health window,
 * Objects cadence ledger, 1 s depth window, calibration verdict). This function is the single
 * place where they meet: it checks that the snapshots describe the same instant and are
 * internally consistent, checks that every reason the numbers require is present, and only then
 * writes the fields.
 *
 * Why a failed check publishes nothing. A Status with a wrong or fabricated field is worse than
 * a missing beat: the missing beat is visible to the consumer's own timeout, a plausible wrong
 * number is not. So every inconsistency returns a code, the caller counts it as a failed beat
 * (5 in a row stop the service) and Status is left untouched.
 *
 * Boundaries. Nothing here measures or decides a reason; the reason/edge owner does that. This
 * function only refuses to publish a Status whose reasons contradict its own numbers.
 */
#pragma once
#include "output/rns_depth_health.hpp"
#include "output/rns_health_window.hpp"

namespace perception::rns {
// Result of apply_health_fields; anything but Ready means Status was not modified.
enum class HealthFieldsCode {
    Ready,
    InvalidWindow,            // health window latched invalid or its time is out of range
    MissingRates,             // fps not equal to frames/10 s, or frames without any observation
    MissingLatency,           // latency percentiles incoherent with the publication count
    MissingCadence,           // cadence snapshot from another instant, or gap stats incoherent
    InvalidDepth,             // depth window incoherent, other instant, or other depth source
    StaleDepth,               // reserved: not returned; an empty depth window is null, not stale
    MissingContext,           // calibration verdict / ground level missing, or ROI kind mismatch
    InvalidSegmentationTime,  // last segmentation update negative or in the future
    MissingReason             // a reason required by the numbers is absent from Status
};
// Inputs that are not measured by any window: they are verdicts owned elsewhere.
struct HealthFieldContext {
    // Not fabricated by the metrics producer. Integration supplies an actually
    // established calibration verdict and the existing ground level convention.
    // std::optional so that "not supplied" is an error rather than a silent false/0.
    std::optional<bool> extrinsic_calibrated;
    std::optional<int> ground_seg_level;
    // Last VALID segmentation-slot UPDATE, not last reuse/publication. Absence
    // explicitly means no valid update observed. Capability window is 10s;
    // this does NOT change the per-frame T freshness rule.
    std::optional<Millis> last_valid_seg_update_ms;
    // The depth source the caller believes produced the depth window. The service
    // currently fills it from the same depth report, so in production this match
    // cannot fail; it binds only when a caller supplies an independent value.
    native_depth::SourceId expected_depth_source;
};
// Binds computed fields into caller-owned Status without allocation.
//
// Reasons. Status reasons are supplied by the separate reason/hysteresis owner:
// required active conditions are checked but recovery history/GEN edge events
// are NOT invented here. Extra/hysteretic reasons are retained. No reason or
// timestamp is mutated. Required: roi_fallback when uncalibrated,
// no_traversable_seg when no segmentation update in 10 s, infer_gap when any
// gap > 50 ms, infer_rate_low when a gap > 100 ms or > 1% of gaps > 50 ms
// (19 section 6.3). reconnecting/reconfiguring must match the active
// declaration exactly in both directions: a stale one would falsely exclude
// wire time from the cadence gate. Startup warmup is not a reconnect
// declaration. Other hysteretic reasons are retained, not cleared by a
// declaration ending.
//
// Cadence. Must be a same-time snapshot from the actual objects publisher
// ledger; this function never observes that ledger again. Cadence exclusions
// affect acceptance, not the independently measured health gaps. R6 valid
// empty windows become null statistics and do not suppress a heartbeat.
//
// Errors. Failure leaves Status intact. Corrupt windows/ROI, missing
// calibration/level or wrong source remain explicit errors, not an empty
// observation or a substituted profile latency.
//
// Caller duties. Caller serializes handoff and checks actual stream ownership;
// expected_source binds metadata, not pointer provenance. Production/no-data
// status integration must supply this window's complete processed-depth
// snapshot, never a latest frame ratio with a freshly substituted observation
// time.
HealthFieldsCode apply_health_fields(const HealthWindowReport&, const CadenceSnapshot&,
    Millis cadence_observed_ms, const DepthHealthWindowReport&, const HealthFieldContext&,
    Status&) noexcept;
}  // namespace perception::rns
