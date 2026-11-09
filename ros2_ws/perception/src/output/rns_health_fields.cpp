/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_health_fields.cpp
 * Brief: Ordered consistency checks, then one unconditional copy into Status.
 *
 * Description:
 * The checks run in a fixed order (window, rates, latency, cadence, gaps, depth, context,
 * segmentation time, reasons) and the first failure returns its code, so a failed beat in the
 * service log names the first inconsistent input. Only after every check passes is Status
 * written, all fields at once; there is no partially updated Status.
 *
 * Why equality instead of tolerance on the derived numbers. fps and invalid_ratio are recomputed
 * here from the same integer counts the windows used, with the same expression, so a correct
 * producer reproduces them bit for bit. Any difference means the report was assembled from
 * mixed sources, which is exactly the fault this module exists to catch.
 */
#include "output/rns_health_fields.hpp"
#include <algorithm>
#include <cmath>

namespace perception::rns {
namespace {
// Field-by-field identity of a native depth source (stream, sequence, SDK indices and times).
// The latest depth frame must be the one the caller expects, not merely one from the same stream.
bool same_source(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
// Reasons are compared as exact strings: 19 section 5.1 fixes their spelling and the
// acceptance tests grep for the literal value.
bool has(const Status& s, std::string_view reason) noexcept {
    return std::any_of(s.degraded_reasons.begin(), s.degraded_reasons.end(),
        [reason](const auto& r) { return r == reason; });
}
}
HealthFieldsCode apply_health_fields(const HealthWindowReport& w, const CadenceSnapshot& c,
    Millis cadence_observed, const DepthHealthWindowReport& d, const HealthFieldContext& x,
    Status& out) noexcept {
    // 1. The publication health window must be usable at a representable time.
    if (!w.valid || w.now_ms < 0 || w.now_ms > ProcessEnvelope::kMaxMonoMs)
        return HealthFieldsCode::InvalidWindow;
    // 2. Rates are frames per 10 s window, divided by exactly 10 even during startup's
    // partial window (the rate ramps up instead of being extrapolated); a frame count
    // without any observation ever would be a counter from somewhere else.
    if (!std::isfinite(w.depth.fps) || !std::isfinite(w.inference.fps) ||
        w.depth.fps != static_cast<double>(w.depth.window_frames) / 10.0 ||
        w.inference.fps != static_cast<double>(w.inference.window_frames) / 10.0 ||
        (!w.depth.ever_observed && w.depth.window_frames) ||
        (!w.inference.ever_observed && w.inference.window_frames)) return HealthFieldsCode::MissingRates;
    // 3. Latency percentiles exist together and exactly when Objects publications were
    // attempted in the window (19 section 5.1: no samples -> null, never invented), and
    // p99 >= p50 >= 0.
    if (w.objects.latency_p50_ms.has_value() != w.objects.latency_p99_ms.has_value() ||
        (w.objects.latency_p50_ms && (*w.objects.latency_p50_ms < 0 ||
          *w.objects.latency_p99_ms < *w.objects.latency_p50_ms)) ||
        w.objects.latency_p50_ms.has_value() != (w.objects.window.attempted > 0))
        return HealthFieldsCode::MissingLatency;
    // 4. The cadence snapshot must be cut at the same instant as the health window, be
    // valid, carry a known declaration, and a declaration must actually exclude time.
    if (cadence_observed != w.now_ms || c.observed_ms != w.now_ms ||
        !c.valid ||
        // Only the two declarations 11 section 3.1B.3 defines may exclude time.
        (c.declared_exclusion != Exclusion::None &&
         c.declared_exclusion != Exclusion::Reconnecting &&
         c.declared_exclusion != Exclusion::Reconfiguring) ||
        (c.declared_exclusion != Exclusion::None && !c.excluded) ||
        // over50_fraction feeds the 1% rule below; NaN would compare false and hide it.
        !std::isfinite(c.over50_fraction) || c.over50_fraction < 0 || c.over50_fraction > 1)
        return HealthFieldsCode::MissingCadence;
    // 5. Gap statistics exist together once any Objects was ever published (the open gap
    // now - last publish keeps counting after that, so a dead feed cannot read as an empty
    // window), and max >= open gap, max >= p99, all non-negative.
    if (w.objects.gap_p99_ms.has_value() != w.objects.gap_max_ms.has_value() ||
        w.objects.gap_max_ms.has_value() != w.objects.open_gap_ms.has_value() ||
        w.objects.gap_max_ms.has_value() != w.objects.ever_observed ||
        (w.objects.gap_max_ms && (*w.objects.open_gap_ms < 0 || *w.objects.gap_p99_ms < 0 ||
         *w.objects.gap_max_ms < *w.objects.open_gap_ms || *w.objects.gap_max_ms < *w.objects.gap_p99_ms)))
        return HealthFieldsCode::MissingCadence;
    // 6. Depth window: same instant; a ratio, ROI kind and source exist exactly when
    // processed frames exist (19 section 5.1 v1.7: none in the 1 s window -> null, never
    // the old value, 0 or 1); ratio = SUM invalid / SUM ROI pixels, recomputed here; and
    // the latest frame is from the expected source.
    if (!d.valid || d.now_ms != w.now_ms || d.invalid_pixels > d.roi_pixels ||
        d.processed_frames > DepthHealthWindow::kMaxCapacity ||
        d.invalid_ratio.has_value() != (d.processed_frames > 0) ||
        d.roi_kind.has_value() != (d.processed_frames > 0) ||
        d.latest_source.has_value() != (d.processed_frames > 0) ||
        (!d.processed_frames && (d.roi_pixels || d.invalid_pixels)) ||
        (d.processed_frames && (!d.ever_observed || !d.roi_pixels ||
         // More ROI pixels than the frames can hold means the sums are corrupt.
         d.roi_pixels > static_cast<std::uint64_t>(d.processed_frames) * native_depth::kMaxPixels ||
         !std::isfinite(*d.invalid_ratio) ||
         *d.invalid_ratio != static_cast<double>(d.invalid_pixels) / d.roi_pixels ||
         !same_source(*d.latest_source, x.expected_depth_source)))) return HealthFieldsCode::InvalidDepth;
    // 7. Context verdicts are required, and the ROI actually used must match the
    // calibration verdict: calibrated -> static mask, uncalibrated -> lower half.
    if (!x.extrinsic_calibrated || !x.ground_seg_level || *x.ground_seg_level < 0 ||
        (d.roi_kind && (*x.extrinsic_calibrated ? *d.roi_kind != HealthRoiKind::CalibratedStaticMask :
                                 *d.roi_kind != HealthRoiKind::LowerHalfUncalibrated)))
        return HealthFieldsCode::MissingContext;
    // 8. A segmentation update from the future would make the slot look fresh forever.
    if (x.last_valid_seg_update_ms && (*x.last_valid_seg_update_ms < 0 ||
        *x.last_valid_seg_update_ms > w.now_ms)) return HealthFieldsCode::InvalidSegmentationTime;
    // 19 section 5.1: traversable_seg_available = a valid slot update within 10 s.
    const bool seg_available = x.last_valid_seg_update_ms &&
        w.now_ms - *x.last_valid_seg_update_ms < HealthWindow::kWindowMs;
    // 9. Every reason the numbers above require must be present (thresholds from
    // 19 section 6.3); the two declaration reasons must match in both directions.
    // Presence only: the reason owner may hold a reason longer (hysteresis).
    if ((!*x.extrinsic_calibrated && !has(out, "roi_fallback")) ||
        (!seg_available && !has(out, "no_traversable_seg")) ||
        ((c.declared_exclusion == Exclusion::Reconnecting) != has(out, "reconnecting")) ||
        ((c.declared_exclusion == Exclusion::Reconfiguring) != has(out, "reconfiguring")) ||
        // Either source demands the reason: the ledger's runtime flag (updated on every
        // put) or this window's own maximum gap. The caller derives it from the same two.
        ((c.infer_gap || c.max_gap_ms > 50) && !has(out, "infer_gap")) ||
        ((c.infer_rate_low || c.max_gap_ms > 100 || c.over50_fraction > .01) && !has(out, "infer_rate_low")))
        return HealthFieldsCode::MissingReason;
    // All checks passed: write every field in one go.
    // 10 s averages (frames / 10); statistics only, the cadence gate is gap based.
    out.fps_depth = w.depth.fps;
    out.fps_infer = w.inference.fps;
    // Status carries these as JSON numbers (double); null stays null.
    const auto numeric = [](const std::optional<Millis>& v) -> std::optional<double> {
        return v ? std::optional<double>{static_cast<double>(*v)} : std::nullopt;
    };
    out.latency_ms_p50 = numeric(w.objects.latency_p50_ms);
    out.latency_ms_p99 = numeric(w.objects.latency_p99_ms);
    out.infer_gap_ms_p99 = numeric(w.objects.gap_p99_ms);
    out.infer_gap_ms_max = numeric(w.objects.gap_max_ms);
    // null when no depth frame was processed in the last 1 s (checked in step 6).
    out.invalid_pixel_ratio = d.invalid_ratio;
    // Passed through as supplied; the service currently supplies 0 because the
    // level semantics are not closed upstream (reply Q14).
    out.ground_seg_level = *x.ground_seg_level;
    // The 19 section 7.3 calibration verdict, copied; this module never decides it.
    out.extrinsic_calibrated = *x.extrinsic_calibrated;
    out.traversable_seg_available = seg_available;
    return HealthFieldsCode::Ready;
}
}  // namespace perception::rns
