/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_status_heartbeat.hpp
 * Brief: Externally driven 1 Hz status scheduling, not a metrics producer.
 *
 * Description:
 * What problem this solves. Status is a 1 Hz heartbeat (19 section 5): the consumer treats a
 * missing beat as "perception may be down", so beats must neither stop silently nor bunch up
 * after a delay (two beats 10 ms apart look healthy while the process was stalled for 2 s).
 * This class decides WHEN a beat is due and publishes the caller's snapshot at most once per
 * due slot; it does not compute any Status field.
 *
 * How the service uses it. The Status thread polls it every loop (about every 10 ms) with the
 * snapshot apply_health_fields() just produced, or nullptr when that failed, and a maximum
 * snapshot age of 100 ms. Five consecutive due beats without a publication stop the service.
 *
 * Why slots are anchored at startup. Deadlines are startup + k * 1000 ms. A late poll takes the
 * newest elapsed slot and counts the skipped ones as missed, so the beat phase never drifts and
 * a stall shows up as missed_deadlines rather than as a burst of catch-up beats.
 */
#pragma once
#include "output/rns_publisher.hpp"

namespace perception::rns {
// Outcome of one poll. NotDue is the normal result between beats; every other code
// except Published means a due beat was consumed without a publication.
enum class HeartbeatCode {
    NotDue, Published, MissingSnapshot, InvalidSnapshot, FutureSnapshot,
    StaleSnapshot, SnapshotRejectedAtPut, PublisherFailure,
    InvalidClock, ClockRollback, Exhausted
};
// Per-poll detail for the service log (lateness, snapshot age, dispatch delay).
struct HeartbeatResult {
    HeartbeatCode code = HeartbeatCode::NotDue;
    bool due = false, publisher_called = false;
    Millis now_ms = -1, scheduled_ms = -1, lateness_ms = -1;
    Millis snapshot_age_ms = -1, dispatch_delay_ms = -1;
    std::uint64_t missed_deadlines = 0;
    PublishResult publication;
};
// Lifetime counters; saturated reports that one of them hit UINT64_MAX.
struct HeartbeatCounters {
    std::uint64_t due_beats = 0, missed_deadlines = 0;
    std::uint64_t snapshot_rejections = 0, publisher_calls = 0;
    std::uint64_t put_attempts = 0, accepted = 0, rejected = 0, unknown = 0;
    std::uint64_t clock_rejections = 0;
    bool saturated = false;
};

// docs/19 section 5: independent 1 Hz status. A SINGLE external owning thread
// must call poll, provide synchronized immutable snapshots, and exclusively own
// this Status channel. Neither a thread nor any status metric producer is
// installed here. Geometry/objects may be silent; their identity/cadence is not
// read or refreshed. Accepted means local put acceptance, NOT remote receipt.
//
// startup, poll now, snapshot observation and PublishClock must share one local
// monotonic epoch. A constant cross-domain clock offset cannot be detected here.
// Snapshots are borrowed only for this call, never retained/replayed/defaulted.
// observed_ms means the complete metrics snapshot's observation time, not a
// freshly substituted send time. Metric provenance/window completeness remain
// the caller's obligation; a valid all-zero explicitly supplied DTO is allowed.
//
// First deadline is startup + 1000 ms. A late call consumes the newest elapsed
// startup-anchored slot and counts earlier slots as missed; it sends at most once.
// Every due beat is consumed, including missing/invalid snapshots and uncertain
// puts. At least 1000 ms also separates dispatches, using actual put-before when
// available: late calls cannot cause a catch-up burst near the next deadline.
// Poll/counter access must be serialized. No allocation after construction in
// this adapter; injected clocks/transport have their own allocation contract.
class StatusHeartbeat {
public:
    // 1 Hz, 19 section 5.
    static constexpr Millis kPeriodMs = 1000;
    // Numeric/storage support bounds, not product detection or health gates.
    static constexpr std::size_t kMaxReasons = 32, kMaxReasonBytes = 4096;
    // Throws invalid_argument for non-Status/invalid channels, invalid age, or
    // startup for which the first deadline cannot be represented.
    StatusHeartbeat(PublisherChannel&, Millis startup_ms, Millis max_snapshot_age_ms);
    StatusHeartbeat(const StatusHeartbeat&) = delete;
    StatusHeartbeat& operator=(const StatusHeartbeat&) = delete;
    // snapshot may be nullptr (no valid Status this time); a due beat is then
    // consumed as MissingSnapshot rather than filled with anything.
    HeartbeatResult poll(Millis now_ms, const Status* snapshot, Millis observed_ms) noexcept;
    HeartbeatCounters counters() const noexcept { return counters_; }
    // Next startup-anchored slot; -1 once the time range is exhausted.
    Millis next_deadline_ms() const noexcept { return exhausted_ ? -1 : next_due_; }
    // Earliest time a beat may actually be sent (slot and 1000 ms spacing).
    Millis next_eligible_ms() const noexcept;
private:
    // Saturating counter add.
    void add(std::uint64_t& counter, std::uint64_t amount = 1) noexcept;
    // Push the earliest next dispatch to from + 1000 ms (never earlier than before).
    void cooldown(Millis from) noexcept;
    PublisherChannel& channel_;
    const Millis max_age_;
    // next_due_: next slot; eligible_: spacing limit; clock_high_water_: latest
    // time seen from any source, so a rollback is detected against all of them.
    Millis next_due_, eligible_, clock_high_water_;
    // Set when the next slot would exceed kMaxMonoMs; no further beats.
    bool exhausted_ = false;
    HeartbeatCounters counters_;
};
}  // namespace perception::rns
