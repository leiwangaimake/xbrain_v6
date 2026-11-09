/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_gen_events.hpp
 * Brief: One-producer/one-GEN-consumer bounded event handoff; no RT takeover.
 *
 * Description:
 * What problem this solves. A degraded-reason edge is detected on the Status thread, but it is
 * sent as an 11 section 6.1 event on the GEN session, whose reliable BLOCK put may wait. GenEvents
 * decouples the two: offer() (Status thread) stamps the envelope and queues the edge without any
 * network call; send_one() (GEN thread, svc_gen) encodes one queued edge and puts it on
 * event/warn/perception or event/info/perception.
 *
 * In the service: queue of 256 edges, packet buffer kMaxPacket, the 12 reason names of
 * perception_service_main.cpp, and a process-instance nonce read from the kernel's random UUID
 * (/proc/sys/kernel/random/uuid) at startup.
 *
 * Event identity. eid = "p-" + nonce + "-" + envelope seq. The seq restarts with every process,
 * so the nonce is what keeps eids of two runs apart. A duplicated delivery of one event carries
 * the same eid, which lets a consumer deduplicate; this class itself never resends.
 */
#pragma once
#include "output/rns_publisher.hpp"
#include "output/rns_reason_edges.hpp"
#include <array>
#include <atomic>
#include <string>
#include <vector>

namespace perception::rns {
// offer(): Accepted with the envelope seq it was stamped with, or why it was not queued.
enum class GenOfferCode { Accepted, Full, Invalid, StampUnavailable, Exhausted };
struct GenOfferResult { GenOfferCode code = GenOfferCode::Invalid; std::uint64_t seq = 0; };
// send_one(): Empty when nothing was queued; otherwise the outcome of that one entry.
enum class GenSendCode { Empty, Accepted, Rejected, Unknown, ClockInvalid, EncodeFailed };
// Everything about one send, for the GEN thread's counters and log.
struct GenSendResult {
    GenSendCode code = GenSendCode::Empty;
    EnvelopeStamp stamp;
    ReasonEdge edge;
    Millis put_before_ms = -1, put_return_ms = -1;
    PutReceipt receipt;
    bool attempted = false, threw = false, clock_valid = false;
    std::size_t packet_bytes = 0;
};
// Producer-side lifetime counters (read by the producer or after both threads stopped).
struct GenOfferCounters {
    std::uint64_t accepted = 0, full = 0, invalid = 0, stamp_failed = 0;
    bool exhausted = false;
};
// Consumer-side lifetime counters (read by the GEN thread or after both stopped).
struct GenSendCounters {
    std::uint64_t popped = 0, accepted = 0, rejected = 0, unknown = 0, unsent = 0;
};

// Ownership: exactly one producer calls offer/offer_counters; exactly one
// separate GEN owner calls send_one/send_counters. No network on offer. Neither
// owner accesses the other's counters while active; read both after joining.
// queue_depth is a diagnostic bounded snapshot, not a transaction with counters.
// Construction/destruction require both owners stopped; no internal thread is
// spawned and transport lifetimes must outlive this object. Constructor throws.
//
// A full FIFO never overwrites old entries: offer returns Full, counts loss,
// and latches healthy=false; producer must retain the receipt in its audit.
// This is explicit local failure, NOT lossless GEN acceptance. send_one removes
// ONE entry and releases the SPSC slot before potentially blocking network I/O.
// Accepted/Rejected/Unknown are local API outcomes, never business receipts;
// unknown is NOT automatically retried. Unsent errors consume their entry too.
// A blocked native call can delay the GEN owner/shutdown; this class cannot
// cancel it. It never owns an RT lock, thread or transport.
//
// Instance nonce: EXACTLY 32 lowercase hex digits, nonzero. Startup must supply
// a genuinely new process-instance ID (e.g. OS UUID), not just 8-byte boot/PID.
// This component validates syntax, NOT randomness/global uniqueness. eid uses
// that nonce plus the shared ProcessEnvelope seq; restart needs a new nonce.
// Caller passes message-creation wall/mono time to offer; edge.observed may be
// earlier. Envelope stamps are frozen there, not refreshed after queue delay.
// Detail reason fields are our explicit diagnostic schema, not new RT fields.
// No remote ack, periodic retry, persistence, startup UUID/ClockStatus factory,
// production scheduler or policy is supplied by this component.
class GenEvents {
public:
    // The fixed event QoS: Data priority, reliable, not express, BLOCK (rns_gen_transport).
    static constexpr PublishQos qos() noexcept { return {Priority::Data, true, false, false}; }
    // Support bounds for the configuration, not tuned values.
    static constexpr std::size_t kMaxQueue = 65536, kMaxPacket = 262144;
    GenEvents(ProcessEnvelope&, PublishClock& consumer_clock,
              RnsTransport& warn, RnsTransport& info, std::string instance_nonce,
              std::vector<std::string> reason_names, std::size_t queue_capacity,
              std::size_t packet_capacity);
    GenEvents(const GenEvents&) = delete;
    GenEvents& operator=(const GenEvents&) = delete;
    // Producer: validate, stamp and queue one edge; never touches the network.
    GenOfferResult offer(const ReasonEdge&, Millis wall_ms, Millis mono_ms) noexcept;
    // Consumer: take the oldest entry, encode and put it; may block in the put.
    GenSendResult send_one() noexcept;
    // The event key for a severity; empty for anything but warn/info.
    std::string_view key(ReasonEdgeSeverity) const noexcept;
    bool healthy() const noexcept { return healthy_.load(std::memory_order_acquire); }
    std::size_t queue_depth() const noexcept;
    std::size_t capacity() const noexcept { return queue_.size() - 1; }
    GenOfferCounters offer_counters() const noexcept { return offered_; } // producer/stopped
    GenSendCounters send_counters() const noexcept { return sent_; } // consumer/stopped
private:
    friend struct GenEventsTestAccess;
    // One queued event: the stamp is frozen at offer time, not refreshed at send.
    struct Entry { EnvelopeStamp stamp; ReasonEdge edge; };
    // Internal consistency of an edge's bit sets and severity.
    bool valid_edge(const ReasonEdge&) const noexcept;
    // JSON into packet_; returns the size, 0 if it did not fit.
    std::size_t encode(const Entry&) noexcept;
    // Saturating increment; saturation marks the handoff unhealthy.
    bool increment(std::uint64_t&) noexcept;
    ProcessEnvelope& envelope_;
    PublishClock& clock_;
    RnsTransport& warn_;
    RnsTransport& info_;
    std::string nonce_, warn_key_, info_key_;
    std::vector<std::string> names_;
    // Ring of capacity + 1 slots (one stays empty to tell full from empty).
    std::vector<Entry> queue_;
    // Encode buffer, owned by the consumer.
    std::vector<char> packet_;
    // Bits that name a reason; anything outside is invalid.
    std::uint64_t allowed_ = 0;
    alignas(64) std::atomic<std::size_t> head_{0}; // producer publishes head
    alignas(64) std::atomic<std::size_t> tail_{0}; // consumer releases tail
    std::atomic<bool> healthy_{true};
    // Producer watermarks (offer time, edge time) and the consumer's last send time.
    Millis last_offer_ = -1, last_observed_ = -1, last_send_ = -1;
    GenOfferCounters offered_;
    GenSendCounters sent_;
};
} // namespace perception::rns
