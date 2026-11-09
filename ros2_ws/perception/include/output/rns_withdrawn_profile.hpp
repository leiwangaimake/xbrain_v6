/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_withdrawn_profile.hpp
 * Brief: r6 complete withdrawn Profile: no guessed blind region or prior-plane FREE.
 *
 * Description:
 * What problem this solves. A fit-failed frame must still be published at the normal rate
 * (docs perception-rns-reply-20260911-r4: publish as usual, never stop), so the consumer sees
 * "this frame cannot be verified" (d_free null, 11 section 3.1B.1 v2.2) rather than a gap. WithdrawnSemanticEvidence (rns_withdrawn_evidence) supplies the
 * per-bin evidence; this builder turns it into a complete wire Profile and publishes it once.
 *
 * The withdrawn shape. Every d_free, h_block, slope, conf and terrain is null or 0; src holds
 * only kG (raw samples seen) and kS (independent semantic block with its d_block). blind_near_m
 * is null because the frame has no valid ground sample to measure it from (19 section 3.4 v1.7),
 * and the contract allows a null blind range only together with all d_free null and no kT bit.
 * t_seg_mono_ms is null: the traversable channel is not consulted on a withdrawn frame.
 *
 * Why the evidence is checked again here. The builder is the last step before the wire, so it
 * refuses any bin that does not have exactly the withdrawn shape, even if the evidence class
 * claims Ready. A wrong FREE value on a withdrawn frame is the one output this path must never
 * produce.
 */
#pragma once
#include "output/rns_withdrawn_evidence.hpp"
#include "output/rns_publisher.hpp"

namespace perception::rns {
// InactiveOrStale: the frame is not newer than the last one, or its evidence/S state
// expired between build and publish. Only Ready has a body.
enum class WithdrawnProfileStatus { NotBuilt, Ready, InvalidSource, InvalidEvidence, InactiveOrStale };
// Build result plus a copy of the evidence report it was built from (for logs).
struct WithdrawnProfileReport {
    WithdrawnProfileStatus status = WithdrawnProfileStatus::NotBuilt;
    WithdrawnReport evidence;
};
// Publish result: dispatched says a put was attempted (the frame is then used up,
// whatever the put outcome in publish).
struct WithdrawnProfilePut {
    WithdrawnProfileStatus status = WithdrawnProfileStatus::NotBuilt;
    bool dispatched = false;
    PublishResult publish;
};
class WithdrawnProfileBuilder {
public:
    // Exact evidence/geometry/projector/mailbox owners outlive this serial
    // consumer. All upstream work and lifecycle changes must be serialized
    // through publish(). This adapter does not certify extrinsic calibration.
    // Geometry's angle min/step must already be exactly representable FP32.
    explicit WithdrawnProfileBuilder(const WithdrawnSemanticEvidence&);
    WithdrawnProfileBuilder(const WithdrawnProfileBuilder&) = delete;
    WithdrawnProfileBuilder& operator=(const WithdrawnProfileBuilder&) = delete;
    // Same contract as NormalProfileBuilder::set_extrinsic_calibrated.
    void set_extrinsic_calibrated(bool calibrated) noexcept { calibrated_ = calibrated; }
    // Builds the body for the frame the evidence currently describes; each frame is
    // consumed once, even when the build is rejected.
    const WithdrawnProfileReport& build(const native_depth::Mailbox&, std::int64_t now_mono_ns) noexcept;
    const WithdrawnProfileReport& report() const noexcept { return report_; }
    // Borrowed until the next build/publish. publish() revalidates the evidence,
    // source, clock and semantic lifetime after any intervening upstream work.
    const Profile* body() const noexcept;
    // At most one dispatch per frame, including a rejected/unknown put. An
    // expired semantic boundary is never silently removed from an old packet.
    WithdrawnProfilePut publish(const native_depth::Mailbox&, PublisherChannel&,
                                std::int64_t now_mono_ns) noexcept;
    // Bin storage held by the body (allocated once in the constructor).
    std::size_t storage_bytes() const noexcept;
private:
    // Empty body (all bins empty, metadata zero, times -1) and a new status.
    void clear(WithdrawnProfileStatus) noexcept;
    // Evidence still Ready, same capture, and evidence_.bins() still non-null now.
    bool current(const native_depth::Mailbox&, std::int64_t now_mono_ns) const noexcept;
    const WithdrawnSemanticEvidence& evidence_;
    const native_depth::Mailbox* mailbox_ = nullptr;
    Profile body_;
    WithdrawnProfileReport report_;
    // Watermarks: a newer stream/sequence and a strictly later capture millisecond are
    // required for every build, so two frames can never share one t_capture_mono_ms.
    native_depth::SourceId last_;
    std::uint64_t epoch_ = 0;
    Millis last_capture_ms_ = -1;
    std::int64_t checked_at_ns_ = -1;
    bool dispatched_ = false;
    bool calibrated_ = false;
};
} // namespace perception::rns
