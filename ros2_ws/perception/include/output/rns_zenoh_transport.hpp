/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_zenoh_transport.hpp
 * Brief: Isolated Zenoh 1.9.0 RT binding. Not a production runtime entrypoint.
 *
 * Description:
 * What problem this solves. Profile, Objects and Status go out on the RT session
 * (tcp/127.0.0.1:7449 formally, 19 section 1.1) with the QoS classes of 11 section 2.4:
 * Profile and Objects are Q1 (real-time priority, best effort, express, drop when congested:
 * a late geometry frame is worse than none), Status is Q2 (data-high priority, reliable, drop).
 * ZenohRtSession opens that session; ZenohRtTransport is one publisher on one of the three keys.
 *
 * Why put_finalized(). The envelope timestamp (t_publish) must be as close as possible to the
 * native put. So the packet is first copied into bytes Zenoh will own (a preallocated slot of
 * PacketSlotPool, rns_packet_slot_pool.hpp), and only then does the publisher's finalizer write
 * the clock sample, seq and stamp into that private copy; the native put follows immediately.
 * Copy and ownership time therefore never count as message age.
 *
 * Why the SDK version is pinned at compile time. The zero-copy hand-over of the finalized bytes
 * relies on how zenoh-c 1.9.0 wraps an owned buffer (audited in the .cpp); another version could
 * copy or share differently, so the build refuses any other version.
 */
#pragma once
#include "output/rns_native_put_audit.hpp"
#include "output/rns_packet_slot_pool.hpp"
#include "output/rns_publisher.hpp"
#include <zenoh.h>
#include <cstdint>
#include <memory>
#include <string>

#if ZENOH_C_MAJOR != 1 || ZENOH_C_MINOR != 9 || ZENOH_C_PATCH != 0
#error "R4 native adapter requires zenoh-c 1.9.0; do not silently use the legacy SDK"
#endif
#ifndef Z_FEATURE_UNSTABLE_API
#error "R4 explicit reliability requires the pinned SDK unstable API"
#endif

namespace perception::rns {

// Endpoint is explicitly injected at startup. This first binding accepts only
// tcp/127.0.0.1:<1..65535>, the local-router shape; no fallback endpoint/scouting.
// Configuration: peer, one connect, no listen, multicast off, gossip on (r4).
// SHM is explicitly disabled for this TCP integration slice, not a production
// transport-performance or shared-memory decision.
// All publishers must be destroyed before their session. Startup may throw;
// neither session creation nor destruction belongs in the realtime loop.
class ZenohRtSession {
public:
    // Opens the peer session to the local router; throws on a bad endpoint or open failure.
    explicit ZenohRtSession(std::string endpoint);
    ~ZenohRtSession();
    ZenohRtSession(const ZenohRtSession&) = delete;
    ZenohRtSession& operator=(const ZenohRtSession&) = delete;
    // Borrowed native handle for declaring publishers (valid while this object lives).
    const z_loaned_session_t* loan() const noexcept;
    // The exact configuration the session was opened with, for the startup log.
    const std::string& config_json() const noexcept { return config_json_; }
    // Number of routers this peer session is currently connected to, or -1 if
    // the query failed. Bounded synchronous query; not for the realtime loop.
    int connected_router_count() const noexcept;
private:
    z_owned_session_t session_;
    std::string config_json_;
};

// NativePutAudit (timing and codes of the last put) is in rns_native_put_audit.hpp.

// The transport's packet pool, for diagnostics: puts that took the malloc fallback
// (owning thread's count), slots zenoh-c has not returned yet, and deleter calls
// with a pointer that was not a held slot (must stay 0).
struct PacketPoolCounters {
    std::uint64_t fallbacks = 0, bad_releases = 0;
    std::size_t held = 0;
};

// Single owner, fixed key/QoS. Native bytes OWN a copy: no unsafe alias of the
// PublisherChannel's reusable buffer, even if delivery continues after return.
// The copy goes into one of kPoolSlots preallocated slots (rns_packet_slot_pool.hpp),
// so our side allocates nothing per packet in steady state; only when zenoh-c still
// holds every slot does a put fall back to malloc (counted, never blocking or
// dropping). zenoh-c 1.9.0 itself still allocates per packet (z_bytes_from_buf
// wraps the bytes in a new Arc; z_publisher_put allocates inside), so this binding
// is still NOT realtime-certified.
// In put_finalized(), private owned bytes and options are prepared BEFORE the
// final clock/slot patch. Native entry is separately measured: thread preemption
// and millisecond truncation remain, so the stamp is not the exact entry ns.
// The legacy put() still accepts already-finalized packets (e.g. GEN semantics).
// Accepted = local API result only, not receipt, persistence, or retransmission.
class ZenohRtTransport final : public RnsTransport {
public:
    ZenohRtTransport(ZenohRtSession&, std::string key, PublishQos, std::size_t max_packet);
    ~ZenohRtTransport();
    ZenohRtTransport(const ZenohRtTransport&) = delete;
    ZenohRtTransport& operator=(const ZenohRtTransport&) = delete;
    // Put an already complete packet (no finalizer step).
    PutReceipt put(std::string_view key, std::string_view packet, PublishQos) noexcept override;
    // Copy, let the finalizer stamp the private copy, then put (see the file header).
    PutReceipt put_finalized(std::string_view key, char* packet, std::size_t size,
        PublishQos, PacketFinalizer&) noexcept override;
    // Diagnostic only: whether any subscriber currently matches the key.
    bool has_matching_subscriber() const noexcept;
    // Undeclare the publisher; later puts are Rejected with kClosed. Idempotent.
    void close() noexcept;
    NativePutAudit last_audit() const noexcept { return audit_; } // owning thread only
    PacketPoolCounters pool_counters() const noexcept;              // owning thread only
    // Local rejection codes (nothing reached Zenoh), outside the native code range.
    static constexpr int kInvalidRequest = -10001, kClosed = -10002,
        kBusy = -10003, kAllocation = -10004, kCancelled = -10005;
    // Packet slots of max_packet bytes each. When zenoh-c runs the deleter inside
    // the put, only slot 0 is ever used; the others cover late releases.
    static constexpr std::size_t kPoolSlots = 4;
private:
    // Shared body of put() and put_finalized().
    PutReceipt prepared_put(std::string_view, std::string_view, PublishQos, PacketFinalizer&) noexcept;
    // The owner's reference is dropped, never deleted directly: zenoh-c may still
    // hold slots after this transport is gone (rns_packet_slot_pool.hpp).
    struct PoolDetach {
        void operator()(PacketSlotPool* pool) const noexcept { pool->detach(); }
    };
    std::string key_;
    PublishQos qos_;
    std::size_t max_packet_;
    // Made in the constructor before the publisher is declared; a constructor
    // that throws afterwards still detaches it.
    std::unique_ptr<PacketSlotPool, PoolDetach> pool_;
    std::uint64_t pool_fallbacks_ = 0;
    z_owned_publisher_t publisher_;
    // busy_ refuses a reentrant put while one is in progress (single owner).
    bool open_ = false, busy_ = false;
    NativePutAudit audit_;
};
}  // namespace perception::rns
