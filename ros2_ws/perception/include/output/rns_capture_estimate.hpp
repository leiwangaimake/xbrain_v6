/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_capture_estimate.hpp
 * Brief: Capture time of a frame as the capture thread stamped it, handed to Profile and Objects.
 *
 * Description:
 * What problem this solves. Every Profile and Objects message carries t_capture_mono_ms, and
 * every age downstream is computed from it. Since 2026-10-14 (19 section 2.4, PD-11 measured)
 * the capture thread stamps each frameset once, from the camera's own frame timestamp
 * (common/capture_clock.hpp), and carries the stamp in the frame's SourceId. This file is the
 * one checked way from that SourceId to the CaptureStamp the builders use, so the Profile and
 * the Objects of one frameset can never disagree about its capture time.
 *
 * What is gone. Until that day the capture time was "host ready time minus
 * perception.capture.latency_fallback_ms" (23 ms). That constant takes part in no timestamp any
 * more: measured, a fresh frameset is taken 46..55 ms after the middle of its exposure and a
 * queued one up to 0.36 s after, both stamped as 23 ms old by a constant. The configuration key
 * stays in the contract file and is still validated; nothing here reads it.
 *
 * How the service uses it. The Profile thread installs EstimatedCaptureClock as its capture
 * clock; the Objects path calls estimate_capture() directly. Status carries the reason
 * t_capture_estimated only while framesets are stamped in the declared fallback grade (the
 * host-side envelope alone, SourceId::capture_estimated), see common/capture_time_state.hpp.
 *
 * Trap. A frameset without a stamp must never get here: the capture thread withholds it. So
 * NoCaptureTime is a fault of the wiring, not a normal case, and both callers treat it as one.
 */
#pragma once

#include "output/rns_native_profile.hpp"
#include <array>
#include <string_view>

namespace perception::rns {

// Status reason literal fixed by 19 section 5.1; tests grep for it.
inline constexpr std::string_view kCaptureEstimatedReason = "t_capture_estimated";
// Bounds validation work, not an upstream limit on external evidence files.
inline constexpr std::size_t kCaptureEstimateAuditSourceMax = 256;

// The clock domain of the stamps and the identity of what makes them.
struct CaptureEstimateConfig {
    // Identifies the monotonic clock domain; 0 is reserved for "unset".
    std::uint64_t clock_epoch = 0;
    // Nonblank, single-line evidence/config identifier. Borrowed immutable
    // storage must outlive the config, adapter, and returned audit views.
    std::string_view audit_source;
};

// True only for a nonzero epoch and a printable, nonblank audit identifier of at most
// kCaptureEstimateAuditSourceMax bytes.
bool valid_capture_estimate_config(const CaptureEstimateConfig&) noexcept;

// Ready is the only status with a capture stamp. NoCaptureTime: the frame's SourceId carries
// none (the capture thread should not have handed it on). ClockRejected: the stamp lies after
// the frame's own host ready time, or that lies after "now".
enum class CaptureEstimateStatus { InvalidConfig, InvalidEpoch, InvalidSource,
    ClockRejected, NoCaptureTime, Ready };

// On Ready, estimated says in which grade the capture thread stamped the frame: false from the
// SDK's mapping of the camera clock (or the envelope agreeing with it), true from the envelope
// alone; reason is then the Status literal, otherwise empty.
struct CaptureEstimateResult {
    CaptureEstimateStatus status = CaptureEstimateStatus::InvalidConfig;
    geometry::CaptureStamp capture;
    bool estimated = false;
    std::string_view reason;
    std::string_view audit_source;
};

// capture = source.capture_mono_ns, checked: a real source, a stamp, and
//   stamp <= source.host_ready_mono_ns <= now_mono_ns.
// now_mono_ns and observed_epoch must identify the SAME MONOTONIC domain as the stamp.
// This is a pure, allocation-free numeric/source check; it reads no device/global/wall clock
// and validates no geometry fields. Full SourceId is preserved; failed output has no stamp.
// Source index 0 and MONOTONIC 0 are valid; negative clock samples are errors.
// Stream freshness/owner/generation and cross-call monotonic checks remain at
// the mailbox/NativeProfileConsumer.
CaptureEstimateResult estimate_capture(const native_depth::Metadata&,
    const CaptureEstimateConfig&, std::uint64_t observed_epoch,
    std::int64_t now_mono_ns) noexcept;

// Explicit opt-in bridge; never installed by NativeProfileConsumer itself.
// The caller persists config/audit_source; Status takes the estimated grade from the capture
// thread's counters (capture_time_state.hpp), not from this object.
// A valid audit identifier is copied into bounded owned storage at construction
// (including when config borrows a temporary string for that constructor call).
// now_context must outlive this immovable object; it must outlive any returned
// NativeProfileClock and audit views. The supplied clock
// must be bounded, noexcept, nonblocking and allocation-free; negative = error.
class EstimatedCaptureClock {
public:
    // Plain function pointer: cannot allocate or throw on the per-frame path.
    using Now = std::int64_t (*)(const void*) noexcept;
    EstimatedCaptureClock(CaptureEstimateConfig, const void* now_context,
        std::uint64_t now_clock_epoch, Now) noexcept;
    EstimatedCaptureClock(const EstimatedCaptureClock&) = delete;
    EstimatedCaptureClock& operator=(const EstimatedCaptureClock&) = delete;

    // Valid config, a clock, and the clock's epoch equal to the configured one.
    bool valid() const noexcept;
    // audit_source points into this object's own storage (see above).
    const CaptureEstimateConfig& config() const noexcept { return config_; }
    // estimate_capture() with this object's config and clock.
    CaptureEstimateResult estimate(const native_depth::Metadata&) const noexcept;
    // Invalid configuration returns an empty clock, rejected by the consumer.
    NativeProfileClock native_clock() const noexcept;

private:
    // C-style trampolines for NativeProfileClock; context is this object.
    static std::int64_t forward_now(const void*) noexcept;
    static bool resolve(const void*, const native_depth::Metadata&,
                        geometry::CaptureStamp&) noexcept;
    // Owned copy of the audit identifier, so config_ never borrows the caller's string.
    std::array<char, kCaptureEstimateAuditSourceMax> audit_storage_{};
    CaptureEstimateConfig config_;
    const void* now_context_;
    std::uint64_t now_clock_epoch_;
    Now now_;
};

}  // namespace perception::rns
