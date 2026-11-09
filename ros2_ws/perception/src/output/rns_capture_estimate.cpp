/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_capture_estimate.cpp
 * Brief: Checked hand-over of the frame's own capture stamp and its NativeProfileClock adapter.
 *
 * Description:
 * estimate_capture() is a pure function: all inputs are arguments, nothing is read from a device
 * or a global clock, nothing allocates. Each failure returns a distinct status and no stamp, so
 * the consumer rejects the frame instead of publishing it with a guessed capture time.
 *
 * Why the adapter copies the audit string. The service builds the config from a string it owns,
 * but the adapter lives for the whole Profile thread; copying at construction removes any
 * lifetime coupling to the caller's storage.
 *
 * Worked example: a frame whose SDK wait returned at host monotonic 5 000.046 600 000 s and
 * which the capture thread stamped 5 000.000 000 000 s (its camera timestamp mapped to the
 * host, half the exposure and the guard taken off) gets exactly that capture time, the service
 * epoch, and the full SourceId of the frame, so the stamp can be matched back to its frame.
 *
 * Two callers. The Profile thread goes through EstimatedCaptureClock (below). The Objects path
 * (perception_service_main.cpp, capture_stamp wiring) calls estimate_capture() directly for each
 * inference result, with a Metadata that carries only the SourceId: only metadata.source is
 * read here, so that is sufficient. Both read the same stamp of the same frame.
 *
 * What is checked elsewhere. This file checks one frame in isolation. Across frames, the
 * capture clock makes the published millisecond increase, and the Profile consumer
 * (rns_native_profile.cpp) requires each capture time to be later than the previous one and not
 * later than the frame's own host ready time, and the stamp's SourceId and epoch to match the
 * frame it took. How good the stamp is (the capture clock's rules, its watchdogs, the grade) is
 * the capture thread's matter; Ready here says only that the frame carries one that is
 * consistent with its own times.
 */
#include "output/rns_capture_estimate.hpp"

namespace perception::rns {

bool valid_capture_estimate_config(const CaptureEstimateConfig& config) noexcept {
    if (!config.clock_epoch || config.audit_source.empty() ||
        config.audit_source.size() > kCaptureEstimateAuditSourceMax) return false;
    // The audit identifier ends up in single-line logs: control characters (newline
    // included) would forge log lines, and an all-blank identifier identifies nothing.
    bool nonblank = false;
    for (const unsigned char c : config.audit_source) {
        if (c < 0x20 || c == 0x7f) return false;
        nonblank |= c != ' ';
    }
    return nonblank;
}

CaptureEstimateResult estimate_capture(const native_depth::Metadata& metadata,
        const CaptureEstimateConfig& config, std::uint64_t observed_epoch,
        std::int64_t now_mono_ns) noexcept {
    // Only metadata.source is read; intrinsics and the raster are not needed.
    // result starts as InvalidConfig with no stamp; each check below overwrites status only.
    CaptureEstimateResult result;
    if (!valid_capture_estimate_config(config)) return result;
    // The "now" sample must come from the clock domain the estimate was declared for.
    if (!observed_epoch || observed_epoch != config.clock_epoch) {
        result.status = CaptureEstimateStatus::InvalidEpoch;
        return result;
    }
    const auto& source = metadata.source;
    // A real source: stream and sequence set, both SDK device times present, and a
    // host ready time actually sampled (-1 = never sampled).
    if (!source.stream_id || !source.sequence || !source.color_device_us ||
        !source.depth_device_us || source.host_ready_mono_ns < 0) {
        result.status = CaptureEstimateStatus::InvalidSource;
        return result;
    }
    // A frame that became ready after "now" means two clock domains were mixed.
    if (now_mono_ns < 0 || source.host_ready_mono_ns > now_mono_ns) {
        result.status = CaptureEstimateStatus::ClockRejected;
        return result;
    }
    // The capture thread hands on stamped framesets only; one without a stamp gets none here.
    if (source.capture_mono_ns < 0) {
        result.status = CaptureEstimateStatus::NoCaptureTime;
        return result;
    }
    // The exposure lies before the pickup: a later stamp is a mixed clock domain or a damaged frame.
    if (source.capture_mono_ns > source.host_ready_mono_ns) {
        result.status = CaptureEstimateStatus::ClockRejected;
        return result;
    }
    // The stamp keeps the whole SourceId, never only a time.
    result.capture = {source, config.clock_epoch, source.capture_mono_ns};
    result.status = CaptureEstimateStatus::Ready;
    result.estimated = source.capture_estimated;
    if (result.estimated) result.reason = kCaptureEstimatedReason;
    result.audit_source = config.audit_source;
    return result;
}

EstimatedCaptureClock::EstimatedCaptureClock(CaptureEstimateConfig config,
        const void* now_context, std::uint64_t now_clock_epoch, Now now) noexcept
    : config_(config), now_context_(now_context), now_clock_epoch_(now_clock_epoch), now_(now) {
    // Validation bounds the size, so the copy always fits audit_storage_.
    if (valid_capture_estimate_config(config)) {
        for (std::size_t i=0; i<config.audit_source.size(); ++i)
            audit_storage_[i] = config.audit_source[i];
        config_.audit_source = {audit_storage_.data(), config.audit_source.size()};
    } else {
        // Invalid construction must not retain a possibly temporary audit view.
        config_.audit_source = {};
    }
}

// Rechecked on every call path, so an invalid adapter can never produce a stamp.
bool EstimatedCaptureClock::valid() const noexcept {
    return valid_capture_estimate_config(config_) && now_context_ && now_ &&
        now_clock_epoch_ == config_.clock_epoch;
}

// NativeProfileClock "now": -1 on any error, never a substitute value.
std::int64_t EstimatedCaptureClock::forward_now(const void* context) noexcept {
    if (!context) return -1;
    const auto& self = *static_cast<const EstimatedCaptureClock*>(context);
    if (!self.valid()) return -1;
    // Any negative reading is normalised to the single error value -1.
    const auto now = self.now_(self.now_context_);
    return now < 0 ? -1 : now;
}

// "now" is sampled per frame, so the ClockRejected check sees the frame's own time.
CaptureEstimateResult EstimatedCaptureClock::estimate(
        const native_depth::Metadata& metadata) const noexcept {
    return estimate_capture(metadata, config_, now_clock_epoch_, forward_now(this));
}

// NativeProfileClock "capture": fills the stamp only on Ready.
bool EstimatedCaptureClock::resolve(const void* context,
        const native_depth::Metadata& metadata, geometry::CaptureStamp& out) noexcept {
    // The out parameter is reused across frames by the caller.
    out = {};  // Do not leave an older usable stamp on a failed callback.
    if (!context) return false;
    const auto& self = *static_cast<const EstimatedCaptureClock*>(context);
    const auto result = self.estimate(metadata);
    if (result.status != CaptureEstimateStatus::Ready) return false;
    out = result.capture;
    return true;
}

// The returned clock borrows this object; see the lifetime rule in the header.
NativeProfileClock EstimatedCaptureClock::native_clock() const noexcept {
    // An empty clock is rejected by NativeProfileConsumer at construction.
    if (!valid()) return {};
    return {this, config_.clock_epoch, &forward_now, &resolve};
}

}  // namespace perception::rns
