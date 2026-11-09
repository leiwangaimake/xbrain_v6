/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_publisher.cpp
 * Brief: Assemble the whole packet with fixed-width placeholders, patch them at the put.
 *
 * Description:
 * publish_body() in order:
 *   1. Preflight: owner, topic, capture identity (strictly newer, except Status).
 *   2. Encode the body with t_publish_mono_ms = INT64_MAX, a 19-digit placeholder.
 *   3. Assemble envelope + body, with space-filled fixed-width slots for ts, mono, seq and
 *      ts_sync. JSON allows the padding whitespace, so no byte moves later.
 *   4. Reserve two journal slots (no accounting, no put).
 *   5. transport.put_finalized(): the transport copies the packet into owned bytes and calls the
 *      finalizer, which samples wall and monotonic time, checks them (and the snapshot guard),
 *      stamps the envelope, and patches the slots and t_publish in place. Only then the put.
 *   6. Sample the return clock, count the outcome, feed the cadence ledger, record the
 *      completion.
 *
 * Why patching instead of re-encoding. Everything slow (encoding, allocation, copy) happens
 * before the clock sample; after it only a few fixed-size writes remain, so t_publish is within
 * a few microseconds of the native put.
 *
 * Tests. tests/test_rns_publisher.cpp, tests/test_rns_declaration.cpp and the Zenoh loopback
 * tests.
 */
#include "output/rns_publisher.hpp"
#include "output/rns_publication_journal.hpp"
#include <charconv>
#include <cstring>
#include <limits>

namespace perception::rns {
namespace {
// Slot widths: 19 digits hold any non-negative int64 ms, 20 hold uint64 seq and the
// seconds form "<int>.<3 digits>".
constexpr std::size_t kMillisSlot = 19, kSecondsSlot = 20, kSeqSlot = 20;
// INT64_MAX encodes as exactly 19 digits, the placeholder the slot patch looks for.
constexpr auto kPlaceholder = std::numeric_limits<Millis>::max();
constexpr std::string_view kPublishField = "\"t_publish_mono_ms\":";
// No variable-size editing after the clock sample. Values are followed by JSON
// whitespace, never leading zeroes. Milliseconds -> seconds uses integer parts
// to avoid intermediate FP rounding, e.g. 12345 -> 12.345.
// Overwrite a space-filled slot with a left-aligned number; trailing spaces remain.
void integer_slot(char* out, std::size_t width, std::uint64_t value) noexcept {
    std::memset(out, ' ', width);
    (void)std::to_chars(out, out + width, value); // widths cover the entire type
}
// Seconds with three decimals from integer parts: 12345 ms -> "12.345".
void seconds_slot(char* out, Millis value) noexcept {
    std::memset(out, ' ', kSecondsSlot);
    auto* end = std::to_chars(out, out + kSecondsSlot, value / 1000).ptr;
    *end++ = '.';
    const auto fraction = value % 1000;
    *end++ = static_cast<char>('0' + fraction / 100);
    *end++ = static_cast<char>('0' + fraction / 10 % 10);
    *end = static_cast<char>('0' + fraction % 10);
}
// Bounded writer into the preallocated packet buffer; overflow sets ok = false.
struct PacketBuilder {
    char* out;
    std::size_t capacity, size = 0;
    bool ok = true;
    void add(std::string_view s) noexcept {
        if (!ok) return;
        if (size >= capacity || s.size() >= capacity - size) { ok = false; return; }
        std::memcpy(out + size, s.data(), s.size()); size += s.size();
    }
    // Reserve a space-filled slot and return its offset for the later patch.
    std::size_t slot(std::size_t width) noexcept {
        const auto at = size;
        add(std::string_view("                    ", width));
        return at;
    }
};
// Adapts the finalize lambda to the transport's PacketFinalizer interface.
template <class Function> struct FinalizeCall final : PacketFinalizer {
    explicit FinalizeCall(Function& f) : function(f) {}
    Function& function;
    bool finalize(char* packet, std::size_t size) noexcept override { return function(packet, size); }
};
// Marks the channel busy for the duration of one call; a nested call is refused.
struct OwnerCall {
    bool& active;
    explicit OwnerCall(bool& value) noexcept : active(value) { active = true; }
    ~OwnerCall() { active = false; }
};
}  // namespace

PublisherChannel::PublisherChannel(Topic topic, ProcessEnvelope& envelope, PublishClock& clock,
                                   RnsTransport& transport, std::size_t body_capacity,
                                   std::size_t packet_capacity, ObjectsCadence* cadence,
                                   PublicationJournal* journal)
    : topic_(topic), envelope_(envelope), clock_(clock), transport_(transport), cadence_(cadence),
      journal_(journal),
      // 11 section 2.4: Status Q2 (data-high, reliable), Profile/Objects Q1 (real-time,
      // best effort, express); all drop when congested.
      qos_{topic == Topic::Status ? Priority::DataHigh : Priority::RealTime,
           topic == Topic::Status, topic != Topic::Status, true},
      body_(body_capacity), packet_(packet_capacity) {
    const bool known = topic == Topic::Profile || topic == Topic::Objects || topic == Topic::Status;
    // Objects needs a cadence ledger and nothing else may have one; a journal must be
    // of this topic (Status has none: it is the report, not a reported put).
    valid_ = known && envelope.valid() && body_capacity > 0 && packet_capacity > 0 &&
             ((topic == Topic::Objects) == (cadence != nullptr)) &&
             (!journal || (topic != Topic::Status && journal->valid() && journal->topic() == topic));
    // xbrain/{rid}/rt/perception/{profile|objects|status} (11 section 2.2.1).
    if (valid_) {
        key_ = "xbrain/"; key_.append(envelope.rid()); key_ += "/rt/perception/";
        key_ += topic == Topic::Profile ? "profile" : topic == Topic::Objects ? "objects" : "status";
    }
}
// Preflight failures: nothing was stamped or put; each kind has its counter.
PublishResult PublisherChannel::fail(PublishError error) noexcept {
    if (error == PublishError::Duplicate) ++counters_.duplicates;
    else if (error == PublishError::OutOfOrder) ++counters_.out_of_order;
    else ++counters_.preflight_failed;
    if (error == PublishError::ClockBefore) ++counters_.clock_faults;
    return {error, {}};
}
template <class T>
PublishResult PublisherChannel::publish_body(const T& body, Topic topic, Millis capture,
                                            const SnapshotGuard* guard) noexcept {
    // Step 1: preflight.
    if (owner_active_) return {PublishError::Reentrant, {}};
    const OwnerCall owner(owner_active_);
    if (!valid_) return fail(PublishError::Configuration);
    if (topic != topic_) return fail(PublishError::WrongTopic);
    // A capture is published at most once and never after a newer one: a repeat
    // would look like a fresh result to the consumer (Status has no capture).
    if (topic != Topic::Status) {
        if (capture < 0) return fail(PublishError::InvalidBody);
        if (capture == last_capture_) return fail(PublishError::Duplicate);
        if (capture < last_capture_) return fail(PublishError::OutOfOrder);
    }
    // Step 2: encode with the placeholder publish time (the contract encoder validates
    // the body; an invalid body is never put).
    Encoded encoded{Error::InvalidInput, 0};
    try { encoded = encode_at_publish(body, kPlaceholder, body_.data(), body_.size()); }
    catch (...) { return fail(PublishError::Internal); }
    if (!encoded) return fail(encoded.error == Error::Capacity ? PublishError::Capacity : PublishError::InvalidBody);
    // Locate the placeholder inside the body; it must be exactly the 19 digits.
    const std::string_view body_view(body_.data(), encoded.size);
    const auto field = body_view.find(kPublishField);
    if (field == std::string_view::npos) return fail(PublishError::Internal);
    const auto publish_at = field + kPublishField.size();
    if (body_view.substr(publish_at, kMillisSlot) != "9223372036854775807") return fail(PublishError::Internal);

    // Step 3: envelope around the body, slots for the late fields (11 section 3.0).
    PacketBuilder w{packet_.data(), packet_.size()};
    w.add("{\"v\":1,\"rid\":\""); w.add(envelope_.rid()); w.add("\",\"ts\":");
    const auto wall_at = w.slot(kSecondsSlot);
    w.add(",\"mono\":"); const auto mono_at = w.slot(kSecondsSlot);
    w.add(",\"boot\":\""); w.add(envelope_.boot()); w.add("\",\"seq\":");
    const auto seq_at = w.slot(kSeqSlot);
    w.add(",\"src\":\"perception\",\"ts_sync\":"); const auto sync_at = w.slot(5);
    w.add(",\"data\":"); const auto data_at = w.size;
    w.add(body_view); w.add("}");
    if (!w.ok) return fail(PublishError::Capacity);
    // A terminating NUL (the builder always leaves room); the put uses the explicit size.
    packet_[w.size] = '\0';
    // Admission reserves BOTH events before any put can happen. Full telemetry
    // never silently drops an actual publication or overwrites an older event.
    // Step 4.
    if (journal_ && !journal_->reserve()) return fail(PublishError::TelemetryUnavailable);

    PublishResult result;
    auto& audit = result.audit;
    bool finalizer_entered = false;
    // Step 5, the finalizer: runs inside the transport, on its private copy of the
    // packet, after all allocation and copying, immediately before the native put.
    auto finalize = [&](char* target, std::size_t size) noexcept {
        // Reject malformed/repeated calls before sampling or reserving a seq.
        // (A transport must call the finalizer exactly once, on the same-size copy.)
        if (finalizer_entered || !target || size != w.size) {
            result.error = PublishError::Internal;
            return false;
        }
        finalizer_entered = true;
        // Mark the timestamp-to-commit interval BEFORE either clock is read.
        // A Status owner cannot certify a complete snapshot through this gap.
        if (journal_) journal_->begin_attempt();
        // Native transport has now completed allocation/copy/wrap/options.
        const auto wall = clock_.wall_millis(), before = clock_.mono_millis();
        // Both clocks readable; put-before not before the capture, not before the last
        // clock sample of this channel, representable.
        if (wall < 0 || before < 0 || before < capture || before < last_clock_ ||
            before > ProcessEnvelope::kMaxMonoMs) {
            result.error = PublishError::ClockBefore; return false;
        }
        // Snapshot freshness at THIS instant: an evidence snapshot older than its
        // max age at the put is refused, whatever the caller believed earlier.
        if (guard && (guard->observed < 0 || guard->max_age < 0 || guard->not_before < 0 ||
            before < guard->not_before || before < guard->observed ||
            before - guard->observed > guard->max_age)) {
            result.error = PublishError::SnapshotInvalid; return false;
        }
        // Two Status in one millisecond would carry the same t_publish: refused.
        if (topic == Topic::Status && before == last_put_before_) {
            result.error = PublishError::Duplicate; return false;
        }
        // Consume the seq only now, when the put is certain to be attempted.
        EnvelopeStamp stamp;
        if (!envelope_.stamp(wall, before, stamp)) {
            result.error = PublishError::StampUnavailable; return false;
        }
        // Patch the slots in the transport's copy: ts, mono, seq, ts_sync, t_publish.
        seconds_slot(target + wall_at, stamp.wall_ms);
        seconds_slot(target + mono_at, stamp.mono_ms);
        integer_slot(target + seq_at, kSeqSlot, stamp.seq);
        // "true " keeps the 5-byte width of "false".
        std::memcpy(target + sync_at, stamp.ts_sync ? "true " : "false", 5);
        integer_slot(target + data_at + publish_at, kMillisSlot, static_cast<std::uint64_t>(before));
        // From here the put counts as attempted; the Attempt goes to the journal.
        audit.attempted = true; audit.seq = stamp.seq; audit.capture_ms = capture;
        audit.put_before_ms = before; audit.packet_bytes = size;
        if (journal_) journal_->record_attempt(audit);
        return true;
    };
    FinalizeCall<decltype(finalize)> finalizer(finalize);
    // A throwing transport is Unknown (it may have sent), never retried.
    try { audit.receipt = transport_.put_finalized(key_, packet_.data(), w.size, qos_, finalizer); }
    catch (...) { audit.threw = true; audit.receipt = {PutOutcome::Unknown, 0}; }
    if (!audit.attempted) {
        if (journal_) journal_->cancel();
        // The transport contract prohibits putting before successful finalize.
        // Thus a prepare exception/rejection consumes neither frame nor seq.
        if (result.error == PublishError::None)
            result.error = audit.receipt.outcome == PutOutcome::Rejected
                ? PublishError::PrepareRejected : PublishError::PrepareUnknown;
        (void)fail(result.error);
        return result;
    }
    // Step 6: after the put.
    const auto before = audit.put_before_ms;
    // Same-frame API return timestamp; never substituted from the previous body.
    if (journal_) journal_->begin_completion();
    const auto after = clock_.mono_millis();
    audit.put_return_ms = after;
    audit.clock_valid = after >= before && after <= ProcessEnvelope::kMaxMonoMs;
    // Watermarks advance even for a failed put: the capture is consumed (no retry
    // could make a stale frame look new).
    last_clock_ = audit.clock_valid ? after : before;
    last_put_before_ = before;
    if (topic != Topic::Status) last_capture_ = capture;
    // Every attempted put is counted with its local API outcome.
    ++counters_.attempted;
    switch (audit.receipt.outcome) {
        case PutOutcome::Accepted: ++counters_.accepted; break;
        case PutOutcome::Rejected: ++counters_.rejected; result.error = PublishError::PutRejected; break;
        default: ++counters_.unknown; result.error = PublishError::PutUnknown; break;
    }
    // A broken return clock outranks the outcome in the error; counts stay truthful.
    if (!audit.clock_valid) { ++counters_.clock_faults; result.error = PublishError::ClockAfter; }
    // Objects only: every attempted unique put enters the cadence ledger.
    if (cadence_) audit.cadence_event = cadence_->observe(capture, before);
    if (journal_) journal_->record_completion(audit);
    return result;
}
// Objects owner declares reconnecting / reconfiguring (11 section 3.1B.3) on the same
// thread and clock as its puts, so a declaration can never be backdated.
DeclarationResult PublisherChannel::declare_exclusion(Exclusion next) noexcept {
    if (owner_active_) return {PublishError::Reentrant};
    const OwnerCall owner(owner_active_);
    if (!valid_) return {PublishError::Configuration};
    if (topic_ != Topic::Objects) return {PublishError::WrongTopic};
    if (next != Exclusion::None && next != Exclusion::Reconnecting &&
        next != Exclusion::Reconfiguring) return {PublishError::InvalidExclusion};
    // Journal first (odd generation before the clock read), as for a put.
    if (journal_ && !journal_->begin_declaration()) return {PublishError::TelemetryUnavailable};
    const auto now = clock_.mono_millis();
    if (now < 0 || now < last_clock_ || now > ProcessEnvelope::kMaxMonoMs) {
        // Close the journal's odd window without recording anything.
        if (journal_) journal_->cancel();
        return {PublishError::ClockBefore};
    }
    // The publisher's own ledger first; the Status mirror gets it via the journal.
    if (!cadence_->declare_exclusion(next, now)) {
        if (journal_) journal_->cancel();
        return {PublishError::CadenceRejected};
    }
    // This clock was actually used by cadence; never let a later failed clock
    // or commit move the owner's watermark backwards.
    last_clock_ = now;
    if (journal_ && !journal_->record_declaration(next, now))
        return {PublishError::TelemetryUnavailable};
    return {PublishError::None, next, now};
}
// The typed entry points. Profile and Objects are identified by their capture time;
// Status has none (-1) and relies on the same-millisecond check instead.
PublishResult PublisherChannel::publish(const Profile& b) noexcept { return publish_body(b, Topic::Profile, b.t_capture_mono_ms); }
PublishResult PublisherChannel::publish(const Objects& b) noexcept { return publish_body(b, Topic::Objects, b.t_capture_mono_ms); }
PublishResult PublisherChannel::publish(const Status& b) noexcept { return publish_body(b, Topic::Status, -1); }
// Profile with S evidence: the S snapshot's age is rechecked at the put instant.
PublishResult PublisherChannel::publish_guarded(const Profile& b, Millis observed,
        Millis max_age, Millis not_before) noexcept {
    const SnapshotGuard guard{observed, max_age, not_before};
    return publish_body(b, Topic::Profile, b.t_capture_mono_ms, &guard);
}
// Status from the heartbeat: the snapshot's age is rechecked at the put instant.
PublishResult PublisherChannel::publish_status_snapshot(const Status& b, Millis observed,
        Millis max_age, Millis not_before) noexcept {
    const SnapshotGuard guard{observed, max_age, not_before};
    return publish_body(b, Topic::Status, -1, &guard);
}
}  // namespace perception::rns
