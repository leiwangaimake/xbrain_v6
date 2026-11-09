/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_gen_transport.cpp
 * Brief: GEN publisher declaration with fixed QoS and an audited copy-then-put.
 *
 * Description:
 * The constructor refuses any key outside the three event keys and any QoS other than
 * Data / reliable / not express / BLOCK, so a wiring mistake cannot turn events into a lossy
 * or high-priority stream. put() copies the caller's packet into Zenoh-owned bytes (the caller
 * may reuse its buffer on return) and records copy and put times on CLOCK_MONOTONIC for the
 * audit; those samples are diagnostics and never enter an envelope.
 *
 * Outcome of put(), and what the caller may conclude:
 *   Rejected, kClosed          the transport was closed; nothing handed to Zenoh.
 *   Rejected, kInvalidRequest  wrong key or QoS, empty or oversized packet; nothing handed over.
 *   Rejected, copy code        Zenoh could not take a copy of the payload; nothing handed over.
 *   Accepted, 0                z_publisher_put returned 0: accepted locally, delivery unknown.
 *   Unknown, put code          the native put failed after the hand-off: it may or may not
 *                              have left the process, so the caller must not resend it.
 *
 * Lifetime. Publishers must be dropped before their session. In the service the three
 * transports are declared after the session object, so C++ destroys them first; close() is
 * idempotent and also runs from the destructor.
 */
#include "output/rns_gen_transport.hpp"
#include <limits>
#include <stdexcept>
#include <time.h>
#include <utility>

namespace perception::rns {
namespace {
// All four QoS fields; a partial match is a different QoS.
bool same_qos(PublishQos a, PublishQos b) noexcept {
    return a.priority == b.priority && a.reliable == b.reliable && a.express == b.express && a.drop == b.drop;
}
// The bare keys event/{warn|info|fault}/perception of 11 section 1.1.6 PC-2. In-machine
// processes publish bare GEN keys; the xbrain/{rid}/... form is the cloud layer, which only
// p5_gateway writes (issues 14.3(b), 2026-10-15). fault: RT session loss notice (19 section 9;
// reply 2026-09-17 Q16).
bool valid_key(std::string_view key) noexcept {
    return key == "event/warn/perception" || key == "event/info/perception" || key == "event/fault/perception";
}
// Constructor-only error path: a native failure at declaration is a startup error.
void checked(int code, const char* operation) {
    if (code != 0) throw std::runtime_error(std::string(operation) + ": " + std::to_string(code));
}
// Checked CLOCK_MONOTONIC nanoseconds for the audit; -1 on any error.
std::int64_t mono_ns() noexcept {
    timespec sample{};
    // Read error or a malformed timespec: report "no time", never a guess.
    if (::clock_gettime(CLOCK_MONOTONIC, &sample) != 0 || sample.tv_sec < 0 ||
        sample.tv_nsec < 0 || sample.tv_nsec >= 1000000000L) return -1;
    constexpr auto limit = static_cast<std::uintmax_t>(std::numeric_limits<std::int64_t>::max());
    // Unsigned arithmetic after the sign check; bound checked before multiplying.
    const auto sec = static_cast<std::uintmax_t>(sample.tv_sec), ns = static_cast<std::uintmax_t>(sample.tv_nsec);
    if (sec > (limit - ns) / 1000000000U) return -1;
    return static_cast<std::int64_t>(sec * 1000000000U + ns);
}
}  // namespace
// Same startup code as the RT session; only the endpoint differs (formal ports 7447 for
// GEN and 7449 for RT, 19 section 1.1; both come from the resolved configuration).
ZenohGenSession::ZenohGenSession(std::string endpoint) : session_(std::move(endpoint)) {}
ZenohGenTransport::ZenohGenTransport(ZenohGenSession& session, std::string key,
        PublishQos qos, std::size_t max_packet)
    : key_(std::move(key)), qos_(qos), max_packet_(max_packet) {
    // Null first, so an early throw leaves nothing to drop.
    z_internal_publisher_null(&publisher_);
    if (!valid_key(key_) || !same_qos(qos_, {Priority::Data, true, false, false}) ||
        max_packet_ == 0 || max_packet_ > kMaxPacketBytes)
        throw std::invalid_argument("invalid GEN key/QoS/capacity");
    // A view, not an owned key expression: key_ outlives the declaration call.
    z_view_keyexpr_t expression;
    checked(z_view_keyexpr_from_str(&expression, key_.c_str()), "GEN publisher key");
    z_publisher_options_t options;
    z_publisher_options_default(&options);
    // The fixed event QoS (see the header): wait, never drop.
    options.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
    options.priority = Z_PRIORITY_DATA;
    options.reliability = Z_RELIABILITY_RELIABLE;
    options.is_express = false;
    // Events are JSON documents (11 section 3.0 envelope).
    z_owned_encoding_t encoding;
    z_encoding_clone(&encoding, z_encoding_application_json());
    options.encoding = z_move(encoding);
    const auto code = z_declare_publisher(session.session_.loan(), &publisher_, z_loan(expression), &options);
    // Drop whatever the failed call may have left before throwing.
    if (code != 0) { z_drop(z_move(publisher_)); checked(code, "declare GEN publisher"); }
    // Only a fully declared publisher is marked open.
    open_ = true;
}
// The destructor only closes; the session is not touched here.
ZenohGenTransport::~ZenohGenTransport() { close(); }
void ZenohGenTransport::close() noexcept {
    // open_ guards against a double drop (close() then destructor).
    if (open_) { z_drop(z_move(publisher_)); open_ = false; }
}
// Zenoh's matching status: whether any subscriber currently matches the key.
bool ZenohGenTransport::has_matching_subscriber() const noexcept {
    z_matching_status_t status{};
    return open_ && z_publisher_get_matching_status(z_loan(publisher_), &status) == 0 && status.matching;
}
PutReceipt ZenohGenTransport::put(std::string_view key, std::string_view packet, PublishQos qos) noexcept {
    // Fresh audit per call, so last_audit() never mixes two puts.
    audit_ = {}; audit_.bytes = packet.size();
    // Local rejections: nothing was handed to Zenoh, so Rejected is certain.
    if (!open_) return {PutOutcome::Rejected, kClosed};
    // The call must repeat the constructor's key and QoS exactly.
    if (key != key_ || !same_qos(qos, qos_) || packet.empty() || packet.size() > max_packet_)
        return {PutOutcome::Rejected, kInvalidRequest};
    // Own copy: the caller may reuse its buffer as soon as put() returns.
    z_owned_bytes_t bytes;
    z_internal_bytes_null(&bytes);
    audit_.copy_begin_ns = mono_ns();
    audit_.copy_code = z_bytes_copy_from_buf(&bytes, reinterpret_cast<const std::uint8_t*>(packet.data()), packet.size());
    audit_.copy_end_ns = mono_ns();
    // Copy failed: release the partial bytes; nothing was handed to the publisher.
    if (audit_.copy_code != 0) { z_drop(z_move(bytes)); return {PutOutcome::Rejected, audit_.copy_code}; }
    // Per-put options stay default: QoS was fixed on the publisher.
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);
    // From here the outcome can only be Accepted or Unknown (see below).
    audit_.native_called = true;
    audit_.put_begin_ns = mono_ns();
    audit_.put_code = z_publisher_put(z_loan(publisher_), z_move(bytes), &options);
    audit_.put_end_ns = mono_ns();
    // The four audit samples must be ordered; otherwise the timing is not usable.
    audit_.clock_valid = audit_.copy_begin_ns >= 0 && audit_.copy_end_ns >= audit_.copy_begin_ns &&
        audit_.put_begin_ns >= audit_.copy_end_ns && audit_.put_end_ns >= audit_.put_begin_ns;
    // A native error does not prove non-delivery, so retry could duplicate an event.
    // Hence Unknown, never Rejected, once the native put was called.
    return {audit_.put_code == 0 ? PutOutcome::Accepted : PutOutcome::Unknown, audit_.put_code};
}
}  // namespace perception::rns
