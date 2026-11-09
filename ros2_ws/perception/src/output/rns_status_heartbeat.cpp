/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_status_heartbeat.cpp
 * Brief: Slot arithmetic, snapshot preflight and one guarded Status put per due slot.
 *
 * Description:
 * poll() in three parts. Clock: reject invalid or backwards 'now'. Schedule: if a slot has
 * elapsed and the spacing allows, consume the newest elapsed slot and count skipped ones as
 * missed; from here on the beat is spent whatever happens next. Publish: preflight the snapshot
 * (present, observation time valid, not from the future, at most max_age old, reason list within
 * the storage bounds), then hand it to the publisher's guarded Status path, which re-checks the
 * snapshot age at the actual put instant.
 *
 * Why the beat is consumed before the snapshot is checked. Otherwise a bad snapshot would leave
 * the slot open and the next poll 10 ms later would try again, turning one bad second into a
 * burst of attempts and hiding the failure from missed_deadlines.
 *
 * Worked example. startup 1000 ms: first slot 2000. A poll at 4350 finds elapsed 2350 ms, so
 * missed = 2, scheduled = 4000, lateness = 350, next slot 5000. The next beat is then due at
 * 5000 but not eligible before 5350 (this poll + 1000) or put_before + 1000, whichever is
 * later; a poll at 5360 publishes it as slot 5000 with lateness 360 and no missed deadline.
 *
 * Tests. tests/test_rns_status_heartbeat.cpp.
 */
#include "output/rns_status_heartbeat.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace perception::rns {
StatusHeartbeat::StatusHeartbeat(PublisherChannel& channel, Millis startup, Millis max_age)
    : channel_(channel), max_age_(max_age), next_due_(0), eligible_(0),
      clock_high_water_(startup) {
    // Only a valid Status channel; the first slot (startup + 1000) must be representable.
    if (!channel.valid() || channel.topic() != Topic::Status || startup < 0 ||
        startup > ProcessEnvelope::kMaxMonoMs - kPeriodMs || max_age < 0 ||
        max_age > ProcessEnvelope::kMaxMonoMs)
        throw std::invalid_argument("invalid status heartbeat configuration");
    // The first beat is one period after startup, on the same grid as all later ones.
    next_due_ = startup + kPeriodMs;
    eligible_ = next_due_;
}
void StatusHeartbeat::add(std::uint64_t& count, std::uint64_t amount) noexcept {
    constexpr auto max = std::numeric_limits<std::uint64_t>::max();
    // Saturate and flag rather than wrap.
    if (amount > max - count) { count = max; counters_.saturated = true; }
    else count += amount;
}
void StatusHeartbeat::cooldown(Millis from) noexcept {
    // Near the end of the representable range: stop instead of overflowing.
    if (from > ProcessEnvelope::kMaxMonoMs - kPeriodMs) exhausted_ = true;
    // max(): a later cooldown can extend the spacing but never shorten it.
    else eligible_ = std::max(eligible_, from + kPeriodMs);
}
// A beat needs both: its slot has come, and 1000 ms since the last dispatch.
Millis StatusHeartbeat::next_eligible_ms() const noexcept {
    return exhausted_ ? -1 : std::max(next_due_, eligible_);
}
HeartbeatResult StatusHeartbeat::poll(Millis now, const Status* snapshot, Millis observed) noexcept {
    HeartbeatResult result;
    result.now_ms = now;
    // Clock faults consume nothing: no slot, no spacing change.
    if (now < 0 || now > ProcessEnvelope::kMaxMonoMs) {
        add(counters_.clock_rejections); result.code = HeartbeatCode::InvalidClock; return result;
    }
    if (now < clock_high_water_) {
        add(counters_.clock_rejections); result.code = HeartbeatCode::ClockRollback; return result;
    }
    clock_high_water_ = now;
    if (exhausted_) { result.code = HeartbeatCode::Exhausted; return result; }
    // The common case: nothing due yet (NotDue, due == false).
    if (now < next_due_ || now < eligible_) return result;

    const Millis elapsed = now - next_due_; // nonnegative: subtraction cannot overflow
    // due == true from here: the caller counts this beat whatever its outcome.
    result.due = true;
    // Whole periods beyond the first elapsed slot were missed; the beat belongs to
    // the newest slot at or before 'now'.
    result.missed_deadlines = static_cast<std::uint64_t>(elapsed / kPeriodMs);
    result.scheduled_ms = now - elapsed % kPeriodMs;
    result.lateness_ms = now - result.scheduled_ms;
    add(counters_.due_beats); add(counters_.missed_deadlines, result.missed_deadlines);
    // Advance the grid by exactly one period from the consumed slot.
    if (result.scheduled_ms > ProcessEnvelope::kMaxMonoMs - kPeriodMs) exhausted_ = true;
    else next_due_ = result.scheduled_ms + kPeriodMs;
    // Spacing from the poll time; a successful put below moves it to put_before.
    cooldown(now); // Even failed/preflight-rejected beats consume this opportunity.

    // Every rejection below happens after the slot was consumed (see file header).
    auto reject = [&](HeartbeatCode code) {
        add(counters_.snapshot_rejections); result.code = code; return result;
    };
    // Snapshot preflight: present, valid time, not newer than 'now', fresh enough.
    if (!snapshot) return reject(HeartbeatCode::MissingSnapshot);
    if (observed < 0 || observed > ProcessEnvelope::kMaxMonoMs)
        return reject(HeartbeatCode::InvalidSnapshot);
    // A snapshot observed after 'now' means two clocks were mixed.
    if (observed > now) return reject(HeartbeatCode::FutureSnapshot);
    result.snapshot_age_ms = now - observed;
    if (result.snapshot_age_ms > max_age_) return reject(HeartbeatCode::StaleSnapshot);
    // Storage bounds of the fixed packet buffer, checked before any serialisation.
    if (snapshot->degraded_reasons.size() > kMaxReasons)
        return reject(HeartbeatCode::InvalidSnapshot);
    // Total reason text, checked without overflow (remaining budget, not a sum).
    std::size_t reason_bytes = 0;
    for (const auto& reason : snapshot->degraded_reasons) {
        if (reason.size() > kMaxReasonBytes - reason_bytes)
            return reject(HeartbeatCode::InvalidSnapshot);
        reason_bytes += reason.size();
    }

    // Guarded put: the channel re-checks observed/max_age at its own clock sample,
    // taken after serialisation, and refuses a snapshot that aged out meanwhile.
    result.publisher_called = true; add(counters_.publisher_calls);
    result.publication = channel_.publish_status_snapshot(*snapshot, observed, max_age_, now);
    const auto& audit = result.publication.audit;
    // Receipt accounting only when the publisher reached the transport.
    if (audit.attempted) {
        add(counters_.put_attempts);
        switch (audit.receipt.outcome) {
            case PutOutcome::Accepted: add(counters_.accepted); break;
            case PutOutcome::Rejected: add(counters_.rejected); break;
            default: add(counters_.unknown); break;
        }
        // The guarded publisher proves before >= now, within the supported range.
        // dispatch_delay = serialisation and envelope time inside the publisher.
        result.dispatch_delay_ms = audit.put_before_ms - now;
        // The publisher's clock samples are later than 'now'; keep them as the high
        // water so the next poll cannot report an earlier time without a rollback.
        clock_high_water_ = std::max(clock_high_water_, audit.put_before_ms);
        if (audit.clock_valid) clock_high_water_ = std::max(clock_high_water_, audit.put_return_ms);
        // Spacing counts from the real put instant, not from the poll time.
        cooldown(audit.put_before_ms);
    }
    // Map publisher errors that are about the snapshot to snapshot codes.
    if (result.publication.error == PublishError::InvalidBody)
        return reject(HeartbeatCode::InvalidSnapshot);
    if (result.publication.error == PublishError::SnapshotInvalid)
        return reject(HeartbeatCode::SnapshotRejectedAtPut);
    // Published only when the publisher reports success (local put accepted).
    result.code = result.publication ? HeartbeatCode::Published : HeartbeatCode::PublisherFailure;
    return result;
}
}  // namespace perception::rns
