/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_publisher.hpp
 * Brief: Typed R4 envelope/put adapter; no Zenoh runtime or legacy takeover.
 *
 * Description:
 * What problem this solves. Every r6 message is one JSON document: the 11 section 3.0 envelope
 * around a typed body (Profile, Objects or Status, 11 section 3.1B). Two fields are hard to get
 * right: t_publish_mono_ms / the envelope times must be the moment of the put, and seq must come
 * from the one process envelope. PublisherChannel assembles the whole packet first with
 * placeholders for those fields, and writes them only inside the transport's finalize step,
 * immediately before the native put (see rns_zenoh_transport). It also refuses to republish a
 * stale or repeated capture, and reports every put to its publication journal and, for Objects,
 * to the cadence ledger.
 *
 * In the service there is one channel per key and thread: Objects (inference callback),
 * Profile (svc_profile) and Status (svc_status), all sharing one ProcessEnvelope.
 */
#pragma once
#include "output/rns_contract.hpp"
#include "output/rns_cadence.hpp"
#include "output/rns_envelope.hpp"
#include <string_view>

namespace perception::rns {
class PublicationJournal;
// The three r6 RT messages (11 section 3.1B), each on its own key.
enum class Topic { Profile, Objects, Status };
// All three are RT session profiles. This is an intent, not proof that an
// eventual native transport applied its QoS. GEN events are a separate path.
enum class Priority { RealTime, DataHigh, Data };
struct PublishQos {
    Priority priority;
    bool reliable, express;
    bool drop = true;  // RT must never use blocking congestion control.
};
// Local API outcome of one put: Rejected = certainly not sent, Unknown = maybe sent.
enum class PutOutcome { Accepted, Rejected, Unknown };
struct PutReceipt { PutOutcome outcome = PutOutcome::Unknown; int native_code = 0; };
// Synchronous, non-retained callback. A transport MUST finish all packet copies
// and owned-byte preparation before calling this exactly once. False cancels
// without a native put. After true, bytes are immutable and a put may have been
// attempted even if the transport subsequently throws/returns Unknown.
class PacketFinalizer {
public:
    virtual ~PacketFinalizer() = default;
    virtual bool finalize(char* packet, std::size_t size) noexcept = 0;
};
// Both clocks the envelope needs; the production one is SystemPublishClock.
class PublishClock {
public:
    virtual ~PublishClock() = default;
    virtual Millis wall_millis() noexcept = 0;
    virtual Millis mono_millis() noexcept = 0;
};
// The put side, one key per instance (ZenohRtTransport, ZenohGenTransport, test fakes).
class RnsTransport {
public:
    virtual ~RnsTransport() = default;
    // Must consume/copy bytes before returning; may NOT retain these views.
    // Accepted means only the local API accepted a put, NEVER remote receipt.
    // Native binding must explicitly apply every QoS field. Exceptions are
    // caught as Unknown (a put might already have happened), not retried.
    virtual PutReceipt put(std::string_view key, std::string_view packet, PublishQos) = 0;
    // Backwards-compatible injected/GEN transport interface. The default has
    // no allocation, but cannot establish an arbitrary old put's native timing
    // boundary: a native implementation must override and finalize owned bytes.
    virtual PutReceipt put_finalized(std::string_view key, char* packet,
        std::size_t size, PublishQos qos, PacketFinalizer& finalizer) {
        if (!finalizer.finalize(packet, size)) return {PutOutcome::Rejected, 0};
        return put(key, {packet, size}, qos);
    }
};
// Why a publish did not happen or did not succeed; None = accepted locally.
enum class PublishError {
    None, Configuration, WrongTopic, InvalidBody, Capacity, Duplicate, OutOfOrder,
    ClockBefore, StampUnavailable, Internal, PutRejected, PutUnknown, ClockAfter,
    SnapshotInvalid, PrepareRejected, PrepareUnknown, TelemetryUnavailable,
    Reentrant, InvalidExclusion, CadenceRejected
};
// What happened in one publish: identity, times and outcome, for the journals and logs.
struct PublishAudit {
    bool attempted = false, threw = false, clock_valid = false;
    std::uint64_t seq = 0;
    Millis capture_ms = -1, put_before_ms = -1, put_return_ms = -1;
    std::size_t packet_bytes = 0;
    PutReceipt receipt;
    CadenceEvent cadence_event = CadenceEvent::Invalid;
};
struct PublishResult {
    PublishError error = PublishError::None;
    PublishAudit audit;
    explicit operator bool() const noexcept { return error == PublishError::None; }
};
struct DeclarationResult {
    PublishError error = PublishError::None;
    // Populated only on success: defaults on failure mean no accepted
    // declaration, NOT a claim about the channel's current exclusion state.
    Exclusion declaration = Exclusion::None;
    Millis declared_ms = -1;
    explicit operator bool() const noexcept { return error == PublishError::None; }
};
// Lifetime counters of one channel (owning thread only).
struct PublishCounters {
    std::uint64_t attempted = 0, accepted = 0, rejected = 0, unknown = 0;
    std::uint64_t preflight_failed = 0, duplicates = 0, out_of_order = 0, clock_faults = 0;
};

// Exactly ONE owning thread per key, with its own channel/buffers/transport.
// ProcessEnvelope may be shared between keys. An optional per-channel journal
// hands immutable attempt/completion events to ONE other owner; the channel,
// counters and cadence object themselves must never be shared between threads.
// No hot-path allocation in this adapter; constructor allocates bounded buffers.
// Transport itself must separately satisfy real-time/buffer-lifetime contracts.
//
// Full body/envelope assembly is BEFORE the put clock sample. After it, only
// bounded numeric/boolean slot patches and (when enabled) one bounded journal
// commit precede put; native owned-byte copies and allocation happen before the
// finalizer. The journal commit adds atomic/copy work to this measured boundary;
// it is not evidence of zero stamp-to-native delay. No cadence/logging is inserted.
// The fallback legacy transport interface does not prove this native boundary.
// Padded JSON whitespace is legal. OS scheduling can still delay native entry.
// Source bodies stay unchanged; their publish field is overridden on the wire.
//
// Repeated/stale captures are not republished. Even a failed/unknown put consumes
// the identity: no retry can turn stale output into a new inference result.
// Cadence measures attempted unique puts, with API outcomes in a SEPARATE ledger.
// Passing cadence alone cannot justify publish/receive acceptance.
class PublisherChannel {
public:
    PublisherChannel(Topic, ProcessEnvelope&, PublishClock&, RnsTransport&,
                     std::size_t body_capacity, std::size_t packet_capacity,
                     ObjectsCadence* objects_cadence = nullptr,
                     PublicationJournal* publication_journal = nullptr);
    PublisherChannel(const PublisherChannel&) = delete;
    PublisherChannel& operator=(const PublisherChannel&) = delete;
    bool valid() const noexcept { return valid_; }
    Topic topic() const noexcept { return topic_; }
    std::string_view key() const noexcept { return key_; }
    PublishCounters counters() const noexcept { return counters_; } // owning thread only
    // One typed publish each; the body is encoded, enveloped, stamped late and put.
    PublishResult publish(const Profile&) noexcept;
    PublishResult publish(const Objects&) noexcept;
    PublishResult publish(const Status&) noexcept;
    // Same single owner as publish(), including its cadence instance. Samples
    // this channel's MONOTONIC clock; callers cannot backdate a declaration.
    // No wire packet, sequence allocation or put counter is generated. Never
    // call during a pending publish; same-thread reentrancy is rejected, not a
    // second-producer synchronisation mechanism. Queue-full may be retried.
    DeclarationResult declare_exclusion(Exclusion) noexcept;
    // Borrow a caller-assembled, immutable status snapshot. Its observation and
    // not-before times use this channel's MONOTONIC epoch. Recheck freshness at
    // the actual put-before sample, AFTER packet assembly and BEFORE stamping.
    // Does not manufacture metrics, infer their provenance, or refresh data age.
    PublishResult publish_status_snapshot(const Status&, Millis observed_ms,
        Millis max_age_ms, Millis not_before_ms) noexcept;
    // A Profile whose bins carry evidence with a lifetime (the S channel,
    // 19 section 3.4A): the evidence age is rechecked at the actual put-before
    // sample, AFTER serialization and envelope assembly, on this channel's
    // MONOTONIC clock -- the same instant t_publish_mono_ms is stamped with. An
    // earlier "now" held by the caller cannot keep expired evidence valid.
    PublishResult publish_guarded(const Profile&, Millis observed_ms,
        Millis max_age_ms, Millis not_before_ms) noexcept;
private:
    // Freshness condition rechecked at the put-before sample (Status, guarded Profile).
    struct SnapshotGuard { Millis observed, max_age, not_before; };
    // Shared body of every publish variant.
    template <class T> PublishResult publish_body(const T&, Topic, Millis capture,
                                                const SnapshotGuard* = nullptr) noexcept;
    // Count a preflight failure and build its result.
    PublishResult fail(PublishError) noexcept;
    Topic topic_;
    ProcessEnvelope& envelope_;
    PublishClock& clock_;
    RnsTransport& transport_;
    ObjectsCadence* cadence_;
    PublicationJournal* journal_;
    PublishQos qos_;
    std::string key_;
    // Preallocated body and packet buffers (no hot-path allocation).
    std::vector<char> body_, packet_;
    // Identity and clock watermarks: capture strictly increases (not Status), clocks
    // never go back, and Status is not sent twice in one millisecond.
    Millis last_capture_ = -1, last_clock_ = -1, last_put_before_ = -1;
    // owner_active_ rejects same-thread reentrancy into publish/declare.
    bool valid_ = false, owner_active_ = false;
    PublishCounters counters_;
};
}  // namespace perception::rns
