/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_withdrawn_profile.cpp
 * Brief: Builds and publishes the withdrawn Profile body, one dispatch per frame.
 *
 * Description:
 * Constructor: proves once that the geometry's wire metadata (angles, z_pass, range_max) is
 * representable in float32 exactly as the contract encoder will write it, so no frame can fail
 * later on metadata.
 *
 * build(): admits a frame only if it is newer than the last one (stream, sequence, capture ms),
 * consumes it immediately, re-verifies every evidence bin against the withdrawn shape, copies the
 * bins and fills the metadata. publish(): re-checks that the evidence is still current, then puts
 * exactly once; bodies carrying S bins go through the snapshot-age guard of the publisher.
 *
 * Caller. NativeProfileConsumer (rns_native_profile.cpp) on the Profile thread: build() right
 * after the evidence for a FitFailed frame, then publish() immediately, with a fresh clock
 * sample when an S owner exists. publish() still revalidates, because an S snapshot inside the
 * body can age out between build and put; the channel checks its age once more at the commit
 * instant. The consumer records the build status and whether a put was dispatched.
 *
 * Tests. tests/test_rns_withdrawn_evidence.cpp covers this builder together with the evidence
 * class (withdrawn shape, one dispatch per frame, no allocation per frame).
 *
 * Why float values are rounded down. z_pass_m and range_max_m are distances the consumer uses
 * as limits; rounding them up by half an ulp would claim slightly more than configured. The
 * angles, by contrast, must be exact: the constructor refuses any angle float32 cannot hold.
 */
#include "output/rns_withdrawn_profile.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// Full source identity, as in the evidence class.
bool same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
// Same frame, clock domain and capture time.
bool same(const geometry::CaptureStamp& a, const geometry::CaptureStamp& b) noexcept {
    return same(a.source, b.source) && a.clock_epoch == b.clock_epoch &&
        a.capture_mono_ns == b.capture_mono_ns;
}
// double -> float rounded toward -infinity (never above x); see the file header.
float down(double x) noexcept {
    float f = static_cast<float>(x);
    if (static_cast<double>(f) > x) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return f;
}
} // namespace

WithdrawnProfileBuilder::WithdrawnProfileBuilder(const WithdrawnSemanticEvidence& e) : evidence_(e) {
    const auto& o = evidence_.geometry().options();
    // z_pass must be a positive float32 before rounding makes sense.
    if (!std::isfinite(o.z_pass_m) || o.z_pass_m <= 0 ||
        o.z_pass_m > std::numeric_limits<float>::max())
        throw std::invalid_argument("withdrawn Profile z_pass exceeds float32 range");
    const float step = static_cast<float>(o.angle_step_rad), z = down(o.z_pass_m);
    // Bin count within the contract, angles exactly representable, range_max within
    // the contract validator's 6 m bound (rns_contract.cpp) and still positive after
    // rounding. These are configuration errors, so they throw at startup.
    if (o.n_bins == 0 || o.n_bins > kMaxBins || !std::isfinite(step) || step <= 0 ||
        // z_pass: a float32 underflow to 0 would publish a zero pass height
        !std::isfinite(z) || z <= 0 || !std::isfinite(static_cast<float>(o.angle_min_rad)) ||
        // angles: the float32 value must equal the configured double bit for bit
        static_cast<double>(step) != o.angle_step_rad ||
        static_cast<double>(static_cast<float>(o.angle_min_rad)) != o.angle_min_rad ||
        // range_max: finite, within the contract bound, positive after rounding down
        !std::isfinite(o.range_max_m) || o.range_max_m > 6 || down(o.range_max_m) <= 0)
        throw std::invalid_argument("withdrawn Profile wire metadata is not representable");
    // The only allocation: one Bin per bearing, reused for every frame.
    body_.bins.resize(o.n_bins);
    clear(WithdrawnProfileStatus::NotBuilt);
}
void WithdrawnProfileBuilder::clear(WithdrawnProfileStatus status) noexcept {
    std::fill(body_.bins.begin(), body_.bins.end(), Bin{});
    body_.t_capture_mono_ms = body_.t_publish_mono_ms = -1;
    // Both stay null on every withdrawn body; build() never sets them.
    body_.t_seg_mono_ms.reset(); body_.blind_near_m.reset();
    body_.extrinsic_calibrated = false;
    body_.z_pass_m = body_.angle_min_rad = body_.angle_step_rad = body_.range_max_m = 0;
    // A cleared body has not been dispatched; only a new Ready build can be.
    report_ = {}; report_.status = status; dispatched_ = false;
}
bool WithdrawnProfileBuilder::current(const native_depth::Mailbox& m, std::int64_t now) const noexcept {
    // bins() additionally checks exact semantic/geometry ownership, current
    // source, revision, FitFailed evidence and the independent S lifetime.
    return mailbox_ == &m && now >= checked_at_ns_ && evidence_.geometry().bound_to(m) &&
        evidence_.report().status == WithdrawnStatus::ReadyEvidenceOnly &&
        same(evidence_.report().capture, report_.evidence.capture) && evidence_.bins(m, now);
}
const WithdrawnProfileReport& WithdrawnProfileBuilder::build(const native_depth::Mailbox& m,
                                                           std::int64_t now) noexcept {
    // Start from an empty body; every rejection leaves it empty.
    clear(WithdrawnProfileStatus::NotBuilt);
    const auto reject = [&](WithdrawnProfileStatus status) -> const WithdrawnProfileReport& {
        clear(status); return report_;
    };
    const auto& er = evidence_.report();
    const auto& c = er.capture;
    const auto& s = c.source;
    // Source admission: same mailbox, active stream, strictly newer stream/sequence,
    // valid epoch (constant within a stream) and a capture time not after 'now'.
    if (!evidence_.geometry().bound_to(m) || (mailbox_ && mailbox_ != &m) ||
        // the frame belongs to the stream the mailbox is currently delivering
        m.active_stream() == 0 || s.stream_id != m.active_stream() || s.sequence == 0 ||
        // never an older stream, never a repeated or older sequence
        s.stream_id < last_.stream_id || (s.stream_id == last_.stream_id && s.sequence <= last_.sequence) ||
        // a real clock domain; capture and 'now' in order; 'now' never backwards
        c.clock_epoch == 0 || c.capture_mono_ns < 0 || now < c.capture_mono_ns || now < checked_at_ns_ ||
        // one clock domain per stream: a changed epoch mid-stream is a wiring fault
        (s.stream_id == last_.stream_id && c.clock_epoch != epoch_))
        return reject(WithdrawnProfileStatus::InvalidSource);
    // The wire carries milliseconds: two frames in one millisecond would publish the
    // same t_capture_mono_ms, so the second one is refused, not published twice.
    const auto capture_ms = c.capture_mono_ns / 1000000;
    if (capture_ms <= last_capture_ms_) return reject(WithdrawnProfileStatus::InactiveOrStale);
    // Consume this source even if downstream evidence/body validation fails.
    // A frame that failed once is not retried: a retry would publish it late.
    mailbox_ = &m; last_ = s; epoch_ = c.clock_epoch;
    last_capture_ms_ = capture_ms; checked_at_ns_ = now;
    // Evidence must be Ready, withdrawn, current now, the right size, and carry
    // exactly the two reasons in the fixed order.
    const auto* bins = evidence_.bins(m, now);
    if (er.status != WithdrawnStatus::ReadyEvidenceOnly || !er.free_withdrawn || !bins ||
        bins->size() != body_.bins.size() || er.reasons[0] != "ground_fit_fallback" ||
        er.reasons[1] != "ground_free_withdrawn")
        return reject(WithdrawnProfileStatus::InvalidEvidence);
    const auto& o = evidence_.geometry().options();
    // Withdrawn shape, bin by bin: no FREE, height, slope, conf or terrain; only kG/kS
    // bits; a d_block exactly when kS is set, finite and within [0, range_max].
    for (const auto& b : *bins) {
        // fields that only a verified ground plane could justify
        if (b.d_free || b.h_block || b.slope_deg || b.conf || b.terrain ||
            // kT and kNegative stay off on a withdrawn frame (A19-FIT-3: bit0 = 0)
            (b.src & ~(kG | kS)) || (b.d_block && (!(b.src & kS) ||
             // an S block must be a real distance inside the published range
             !std::isfinite(*b.d_block) || *b.d_block < 0 || *b.d_block > down(o.range_max_m))) ||
            // and kS without a distance would be a claim without a value
            ((b.src & kS) && !b.d_block))
            return reject(WithdrawnProfileStatus::InvalidEvidence);
    }
    // Copy, not borrow: the body must stay intact even if the evidence is rebuilt.
    std::copy(bins->begin(), bins->end(), body_.bins.begin());
    body_.t_capture_mono_ms = capture_ms;
    // Calibration flag as set by the owner; this builder does not certify it.
    body_.extrinsic_calibrated = calibrated_;
    // Wire metadata; the constructor proved these conversions are valid.
    body_.z_pass_m = down(o.z_pass_m);
    body_.angle_min_rad = static_cast<float>(o.angle_min_rad);
    body_.angle_step_rad = static_cast<float>(o.angle_step_rad);
    body_.range_max_m = down(o.range_max_m);
    report_.evidence = er;
    // Final recheck at the same 'now', after all copying.
    if (!current(m, now)) return reject(WithdrawnProfileStatus::InactiveOrStale);
    report_.status = WithdrawnProfileStatus::Ready;
    return report_;
}
// nullptr unless the last build is Ready (a rejected build never exposes a body).
const Profile* WithdrawnProfileBuilder::body() const noexcept {
    return report_.status == WithdrawnProfileStatus::Ready ? &body_ : nullptr;
}
WithdrawnProfilePut WithdrawnProfileBuilder::publish(const native_depth::Mailbox& m,
        PublisherChannel& channel, std::int64_t now) noexcept {
    WithdrawnProfilePut result;
    result.status = report_.status;
    // Nothing to publish: report the build status, dispatch nothing.
    if (result.status != WithdrawnProfileStatus::Ready) return result;
    // Second publish of the same body, or evidence expired since build: drop it.
    if (dispatched_ || !current(m, now)) {
        clear(WithdrawnProfileStatus::InactiveOrStale); result.status = report_.status; return result;
    }
    // Mark before the put: a put that throws or times out must not be retried.
    dispatched_ = true; result.dispatched = true;
    // Same rule as the normal builder: a withdrawn body that carries S bins has
    // its snapshot age rechecked at the actual commit instant (floored, never
    // looser than the projector's own gate).
    const auto* semantic = evidence_.semantic();
    if (semantic && report_.evidence.semantic_bins > 0 && semantic->report().objects) {
        // Age of the Objects snapshot the S blocks came from, checked at the put.
        const Millis observed = semantic->report().objects->capture_mono_ns / 1000000;
        // Integer division floors the limit: never looser than the projector's gate.
        const Millis max_age = semantic->max_age_ns() / 1000000;
        result.publish = channel.publish_guarded(body_, observed, max_age, observed);
    } else {
        // No S bins in this body (G-only, or no S target this frame): no snapshot
        // age to guard, the plain publish path applies.
        result.publish = channel.publish(body_);
    }
    return result;
}
std::size_t WithdrawnProfileBuilder::storage_bytes() const noexcept {
    return body_.bins.capacity() * sizeof(Bin);
}
} // namespace perception::rns
