/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_gen_transport.hpp
 * Brief: Independent local-TCP Zenoh 1.9.0 GEN session and fixed event QoS.
 *
 * Description:
 * What problem this solves. 19 section 1.1 gives perception two Zenoh sessions: RT (Profile,
 * Objects, Status on tcp/127.0.0.1:7449) and GEN (events on tcp/127.0.0.1:7447). Events must
 * be reliable (an edge that is lost is never resent, see rns_reason_edges), while RT traffic
 * must never wait. Keeping events on their own session and their own thread means a blocking
 * event put cannot delay a Profile.
 *
 * How the service uses it. One ZenohGenSession on perception gen_endpoint (from the resolved
 * config, never hard-coded, 19 P19-5) and three transports on it: event/warn, event/info and
 * event/fault. The Status thread hands reason edges to GenEvents; the GEN thread (svc_gen)
 * sends them, plus the notices (config digest, RT router loss), through these transports.
 *
 * Why BLOCK and no retry. BLOCK makes Zenoh wait for buffer space instead of dropping an event;
 * that wait happens on the GEN thread only. A failed native put is reported as Unknown and not
 * retried, because the error does not prove the event was not delivered, and a retry could
 * deliver it twice.
 */
#pragma once
#include "output/rns_zenoh_transport.hpp"

namespace perception::rns {
class ZenohGenTransport;
// Owns a NEW native session, never an externally supplied/shared RT session.
// Private composition reuses the verified TCP startup configuration only:
// explicit tcp/127.0.0.1 port, peer, no listen/multicast/SHM, gossip on.
// There is deliberately no public native loan or RT-session conversion. Only
// ZenohGenTransport can declare publishers on this session through this API.
// Destroy transports first. Startup/shutdown are outside the event-send loop.
class ZenohGenSession {
public:
    explicit ZenohGenSession(std::string endpoint);
    ~ZenohGenSession() = default;
    ZenohGenSession(const ZenohGenSession&) = delete;
    ZenohGenSession& operator=(const ZenohGenSession&) = delete;
    // The exact JSON the session was opened with (for the startup log).
    const std::string& config_json() const noexcept { return session_.config_json(); }
    // Router connections right now (diagnostic; the service does not gate on it).
    int connected_router_count() const noexcept { return session_.connected_router_count(); }
private:
    friend class ZenohGenTransport;
    ZenohRtSession session_;
};

// Single consumer/owner; close/getters/put must not race. Fixed key is exactly
// event/{warn|info|fault}/perception (bare, 11 section 1.1.6 PC-2; no robot-id prefix).
// Fixed QoS is Data / RELIABLE / BLOCK / express=false. BLOCK may block: this
// belongs on an independent GEN consumer, never RT or an ISR. No retry is done.
// Own-copy of payload before native put permits caller buffer reuse on return.
// Native allocation/copy and potentially blocking time are separately audited;
// this adapter has no zero-allocation, latency or delivery guarantee.
// Accepted means local API acceptance only; native put errors are Unknown.
class ZenohGenTransport final : public RnsTransport {
public:
    static constexpr std::size_t kMaxPacketBytes = 1024 * 1024; // Explicit implementation support bound.
    // Local rejection codes, outside the range of native Zenoh error codes.
    static constexpr int kInvalidRequest = -11001, kClosed = -11002;
    ZenohGenTransport(ZenohGenSession&, std::string key, PublishQos, std::size_t max_packet);
    ~ZenohGenTransport();
    ZenohGenTransport(const ZenohGenTransport&) = delete;
    ZenohGenTransport& operator=(const ZenohGenTransport&) = delete;
    // key and qos must equal the ones fixed at construction (no per-call routing).
    PutReceipt put(std::string_view key, std::string_view packet, PublishQos) noexcept override;
    // Diagnostic only: a subscriber may exist without having received anything.
    bool has_matching_subscriber() const noexcept;
    // Undeclare the publisher; later puts are Rejected with kClosed. Idempotent.
    void close() noexcept;
    // Copy and native-put timing of the last put() call.
    NativePutAudit last_audit() const noexcept { return audit_; }
private:
    std::string key_;
    PublishQos qos_;
    std::size_t max_packet_;
    // Declared once in the constructor, dropped in close().
    z_owned_publisher_t publisher_;
    bool open_ = false;
    NativePutAudit audit_;
};
}  // namespace perception::rns
