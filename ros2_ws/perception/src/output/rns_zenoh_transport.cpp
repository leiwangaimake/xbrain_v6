/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_zenoh_transport.cpp
 * Brief: Checked native ownership/QoS; copy and native put have separate clocks.
 *
 * Description:
 * Session: peer mode, exactly one connect endpoint (the local router), no listen, no multicast
 * scouting, gossip on but not multihop, no shared memory. Publisher: key and QoS must be one of
 * the three allowed pairs (11 section 2.4: Profile/Objects Q1, Status Q2); anything else is a
 * startup error.
 *
 * prepared_put() order: refuse reentrancy and bad requests; take a free slot of the transport's
 * PacketSlotPool (all slots still held by zenoh-c: malloc, counted) and copy the packet into it;
 * hand that buffer to zenoh-c as owned bytes (no copy; our deleter returns the slot, or frees the
 * fallback buffer); call the finalizer on that private buffer (the publisher stamps it there);
 * call z_publisher_put. Each stage is timed on CLOCK_MONOTONIC into NativePutAudit, so copy,
 * finalizer and native put can be told apart in a latency investigation.
 *
 * Outcome rule. Everything before z_publisher_put is Rejected with a local code (nothing left the
 * process). A native error after the call is Unknown, never Rejected, and never retried: the SDK
 * may already have queued the sample.
 *
 * Tests. tests/test_rns_zenoh_loopback.cpp, tests/test_rns_zenoh_owners.cpp and
 * tests/test_rns_zenoh_generated.cpp (loopback router, no network). Heap calls per put (ours
 * and zenoh-c's apart, A19-ALLOC-1): tests/test_rns_zenoh_alloc.cpp (Linux) and the pool alone
 * in tests/test_rns_packet_slot_pool.cpp.
 */
#include "output/rns_zenoh_transport.hpp"
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <time.h>
#include <utility>

namespace perception::rns {
namespace {
// RAII holders for owned zenoh-c objects: every path (including throws) drops them.
struct Config {
    z_owned_config_t value;
    Config() { z_internal_config_null(&value); }
    ~Config() { z_drop(z_move(value)); }
};
struct NativeString {
    z_owned_string_t value;
    NativeString() { z_internal_string_null(&value); }
    ~NativeString() { z_drop(z_move(value)); }
};
struct NativeBytes {
    z_owned_bytes_t value;
    NativeBytes() { z_internal_bytes_null(&value); }
    ~NativeBytes() { z_drop(z_move(value)); }
};
// Deleter of the malloc fallback buffer (thread-safe free). Pool slots use
// PacketSlotPool::release instead.
void free_packet(void* data, void*) noexcept { std::free(data); }
// put() uses the same path as put_finalized() with a no-op finalizer.
struct AlreadyFinalized final : PacketFinalizer {
    bool finalize(char*, std::size_t) noexcept override { return true; }
};
// Startup-only error path: a failed native setup call is a startup error.
void checked(int code, const char* operation) {
    if (code != 0) throw std::runtime_error(std::string(operation) + ": " + std::to_string(code));
}
// Only tcp/127.0.0.1:<1..65535> is accepted: the service talks to its local router only
// (19 P19-5: never 0.0.0.0, no scouting fallback).
bool local_endpoint(std::string_view value) noexcept {
    // LINT-ALLOW(endpoint): loopback VALIDATOR, not an endpoint. The endpoint itself comes from the resolved config.
    constexpr std::string_view prefix = "tcp/127.0.0.1:";
    if (value.substr(0, prefix.size()) != prefix) return false;
    value.remove_prefix(prefix.size());
    if (value.empty() || value.size() > 5) return false;
    unsigned port = 0;
    for (const char c : value) {
        if (c < '0' || c > '9') return false;
        port = port * 10U + static_cast<unsigned>(c - '0');
    }
    return port > 0 && port <= 65535;
}
// All four QoS fields must match.
bool same_qos(PublishQos a, PublishQos b) noexcept {
    return a.priority == b.priority && a.reliable == b.reliable &&
           a.express == b.express && a.drop == b.drop;
}
// The three RT keys this service may publish, each with its fixed QoS class.
bool allowed_key_qos(std::string_view key, PublishQos qos) noexcept {
    constexpr std::string_view prefix = "xbrain/";
    if (key.substr(0, prefix.size()) != prefix) return false;
    key.remove_prefix(prefix.size());
    const auto slash = key.find('/');
    if (slash == std::string_view::npos || slash == 0 || slash > 32) return false;
    for (const char c : key.substr(0, slash))
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
    key.remove_prefix(slash);
    // Q1: real-time priority, best effort, express, drop.
    if (key == "/rt/perception/profile" || key == "/rt/perception/objects")
        return same_qos(qos, {Priority::RealTime, false, true, true});
    // Q2: data-high priority, reliable, not express, drop.
    return key == "/rt/perception/status" && same_qos(qos, {Priority::DataHigh, true, false, true});
}
// Checked CLOCK_MONOTONIC nanoseconds for the audit; -1 on any error.
std::int64_t mono_ns() noexcept {
    timespec sample{};
    if (::clock_gettime(CLOCK_MONOTONIC, &sample) != 0 || sample.tv_sec < 0 ||
        sample.tv_nsec < 0 || sample.tv_nsec >= 1000000000L) return -1;
    constexpr auto limit = static_cast<std::uintmax_t>(std::numeric_limits<std::int64_t>::max());
    const auto sec = static_cast<std::uintmax_t>(sample.tv_sec);
    const auto ns = static_cast<std::uintmax_t>(sample.tv_nsec);
    if (sec > (limit - ns) / 1000000000U) return -1;
    return static_cast<std::int64_t>(sec * 1000000000U + ns);
}
}  // namespace

ZenohRtSession::ZenohRtSession(std::string endpoint) {
    // Null first so the destructor is safe even if the constructor throws below.
    z_internal_session_null(&session_);
    if (!local_endpoint(endpoint)) throw std::invalid_argument("RT endpoint must be an explicit local TCP router endpoint");
    Config cfg;
    checked(z_config_default(&cfg.value), "config default");
    const auto insert = [&](const char* key, const char* value) {
        checked(zc_config_insert_json5(z_loan_mut(cfg.value), key, value), key);
    };
    // Peer that only connects out: no listening socket on any interface.
    insert("mode", "\"peer\"");
    insert("listen/endpoints", "[]");
    const std::string connect = "[\"" + endpoint + "\"]"; // syntax validated, not arbitrary JSON
    insert("connect/endpoints", connect.c_str());
    // No multicast discovery: the router endpoint is the only way in.
    insert("scouting/multicast/enabled", "false");
    insert("scouting/gossip/enabled", "true");
    insert("scouting/gossip/multihop", "false"); // r4 docs11 latest RT/GEN isolation correction
    // This isolated binding deliberately exercises TCP, not SHM. Apart from
    // making ownership observable across the socket, this avoids creating
    // global POSIX SHM objects merely to run a local host contract test.
    insert("transport/shared_memory/enabled", "false");
    // Read the effective configuration back for the log before opening.
    NativeString text;
    checked(zc_config_to_string(z_loan(cfg.value), &text.value), "config readback");
    config_json_.assign(z_string_data(z_loan(text.value)), z_string_len(z_loan(text.value)));
    const auto code = z_open(&session_, z_move(cfg.value), nullptr); // consumes config, including failure
    if (code != 0) {
        z_drop(z_move(session_));
        checked(code, "RT session open");
    }
}
ZenohRtSession::~ZenohRtSession() { z_drop(z_move(session_)); }
const z_loaned_session_t* ZenohRtSession::loan() const noexcept { return z_loan(session_); }
namespace {
// Closure body: one call per connected router id.
void count_zid(const z_id_t*, void* context) { ++*static_cast<int*>(context); }
}  // namespace
int ZenohRtSession::connected_router_count() const noexcept {
    int count = 0;
    z_owned_closure_zid_t callback;
    // The SDK calls and drops the closure before returning (pinned 1.9.0 docs),
    // so a stack counter is valid for the whole call.
    z_closure_zid(&callback, &count_zid, nullptr, &count);
    if (z_info_routers_zid(z_loan(session_), z_move(callback)) != 0) return -1;
    return count;
}

ZenohRtTransport::ZenohRtTransport(ZenohRtSession& session, std::string key,
                                 PublishQos qos, std::size_t max_packet)
    : key_(std::move(key)), qos_(qos), max_packet_(max_packet) {
    z_internal_publisher_null(&publisher_);
    // Only the three RT key/QoS pairs; a wrong pair is a wiring error at startup.
    if (!allowed_key_qos(key_, qos_) || max_packet_ == 0)
        throw std::invalid_argument("invalid RT key/QoS/capacity");
    // Packet slots of the full packet bound, so the size rule of prepared_put() is unchanged.
    // Startup-only allocation; failing here is a startup error like the checks above.
    pool_.reset(PacketSlotPool::create(max_packet_, kPoolSlots));
    if (!pool_) throw std::bad_alloc();
    z_view_keyexpr_t expression;
    checked(z_view_keyexpr_from_str(&expression, key_.c_str()), "publisher key");
    z_publisher_options_t options;
    z_publisher_options_default(&options);
    // Map our PublishQos onto the native options (all RT classes drop when congested).
    options.congestion_control = Z_CONGESTION_CONTROL_DROP;
    options.priority = qos_.priority == Priority::RealTime ? Z_PRIORITY_REAL_TIME : Z_PRIORITY_DATA_HIGH;
    options.is_express = qos_.express;
    options.reliability = qos_.reliable ? Z_RELIABILITY_RELIABLE : Z_RELIABILITY_BEST_EFFORT;
    z_owned_encoding_t encoding;
    z_encoding_clone(&encoding, z_encoding_application_json());
    options.encoding = z_move(encoding); // consumed by declaration
    const auto code = z_declare_publisher(session.loan(), &publisher_, z_loan(expression), &options);
    if (code != 0) {
        // Drop whatever the failed call left, then report the startup error.
        z_drop(z_move(publisher_));
        checked(code, "declare RT publisher");
    }
    open_ = true;
}
// Publishers must go before their session; the service declares them after it.
ZenohRtTransport::~ZenohRtTransport() { close(); }
void ZenohRtTransport::close() noexcept {
    if (open_) { z_drop(z_move(publisher_)); open_ = false; }
}
// Test and smoke-tool helper (they wait for a subscriber before measuring); the service's
// puts never wait for one.
bool ZenohRtTransport::has_matching_subscriber() const noexcept {
    z_matching_status_t status{};
    return open_ && z_publisher_get_matching_status(z_loan(publisher_), &status) == 0 && status.matching;
}
PacketPoolCounters ZenohRtTransport::pool_counters() const noexcept {
    return {pool_fallbacks_, pool_->bad_releases(), pool_->held()};
}
PutReceipt ZenohRtTransport::put(std::string_view key, std::string_view packet, PublishQos qos) noexcept {
    AlreadyFinalized finalizer;
    return prepared_put(key, packet, qos, finalizer);
}
PutReceipt ZenohRtTransport::put_finalized(std::string_view key, char* packet,
        std::size_t size, PublishQos qos, PacketFinalizer& finalizer) noexcept {
    // A null packet is a caller bug: reject without touching an in-progress audit.
    if (!packet) {
        if (busy_) return {PutOutcome::Rejected, kBusy};
        audit_ = {};
        audit_.bytes = size;
        return {PutOutcome::Rejected, kInvalidRequest};
    }
    return prepared_put(key, {packet, size}, qos, finalizer);
}
PutReceipt ZenohRtTransport::prepared_put(std::string_view key, std::string_view packet,
        PublishQos qos, PacketFinalizer& finalizer) noexcept {
    // Synchronous single owner. Reentrant calls cannot replace the active
    // operation's audit or retain a second pending native payload.
    if (busy_) return {PutOutcome::Rejected, kBusy};
    // Fresh audit for this call.
    audit_ = {};
    audit_.bytes = packet.size();
    if (!open_) return {PutOutcome::Rejected, kClosed};
    // The call must repeat the constructor's key and QoS; size within the bound.
    if (key != key_ || !same_qos(qos, qos_) || packet.empty() || packet.size() > max_packet_)
        return {PutOutcome::Rejected, kInvalidRequest};
    // busy_ is cleared on every return path by this guard.
    busy_ = true;
    struct BusyReset { bool& value; ~BusyReset() { value = false; } } reset{busy_};
    NativeBytes bytes;
    audit_.copy_begin_ns = mono_ns();
    // Steady state: a free slot of the pool; zenoh-c's deleter returns it. Only when
    // zenoh-c still holds every slot: the pre-pool malloc path, counted, so a late
    // release never blocks or drops a packet and the outcome rules stay as they were.
    void (*deleter)(void*, void*) = &PacketSlotPool::release;
    void* context = pool_.get();
    auto* writable = pool_->acquire();
    if (!writable) {
        audit_.pool_fallback = true;
        ++pool_fallbacks_;
        writable = static_cast<std::uint8_t*>(std::malloc(packet.size()));
        if (!writable) { audit_.copy_code = kAllocation; return {PutOutcome::Rejected, kAllocation}; }
        deleter = &free_packet;
        context = nullptr;
    }
    // Our own copy: the publisher reuses its packet buffer as soon as we return.
    std::memcpy(writable, packet.data(), packet.size());
    // Pinned zenoh-c 1.9.0: src/zbytes.rs:303-325 wraps CSliceOwned;
    // collections.rs:121-160 stores pointer/len/deleter (non-null is valid),
    // zbytes.rs:239-262 -> zenoh 1.9.0 zslice.rs:245-268 creates a private Arc
    // with only a temporary length borrow. It neither clones payload bytes nor
    // exports a reference to an async consumer. This MUST be re-audited on SDK
    // upgrade. We keep no SDK immutable slice/view, use no const_cast, never
    // loan/clone/publish these private bytes before the synchronous patch, and
    // never write writable again after finalize. Ownership alone is not a loan:
    // the buffer (pool slot or fallback allocation) remains alive under NativeBytes
    // through that patch. A pool slot's held flag is cleared only by the deleter, so
    // a slot zenoh-c may still read is never handed to the next packet.
    audit_.copy_code = z_bytes_from_buf(&bytes.value, writable, packet.size(), deleter, context);
    if (audit_.copy_code != 0) {
        // Pinned implementation's failure writes empty bytes without adopting
        // data, so the buffer is still ours: return the slot (or free the
        // fallback) here, once; NativeBytes then drops nothing of it.
        deleter(writable, context);
        return {PutOutcome::Rejected, audit_.copy_code};
    }
    // From here only the SDK's thread-safe deleter returns the slot or frees the fallback.
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);
    audit_.copy_end_ns = mono_ns();
    // The finalizer stamps time, seq and ts_sync into the private copy right now,
    // as late as possible before the native put.
    audit_.finalizer_begin_ns = mono_ns();
    audit_.finalized = finalizer.finalize(reinterpret_cast<char*>(writable), packet.size());
    audit_.finalizer_end_ns = mono_ns();
    // Finalizer refused (e.g. clock or stamp failure): nothing is sent; the owned
    // bytes are dropped with their deleter.
    if (!audit_.finalized || !open_) {
        audit_.cancelled = true;
        return {PutOutcome::Rejected, open_ ? kCancelled : kClosed};
    }
    audit_.native_called = true;
    audit_.put_begin_ns = mono_ns();
    audit_.put_code = z_publisher_put(z_loan(publisher_), z_move(bytes.value), &options);
    audit_.put_end_ns = mono_ns();
    // The audit timings are usable only if all six samples are in order.
    audit_.clock_valid = audit_.copy_begin_ns >= 0 && audit_.copy_end_ns >= audit_.copy_begin_ns &&
                         audit_.finalizer_begin_ns >= audit_.copy_end_ns &&
                         audit_.finalizer_end_ns >= audit_.finalizer_begin_ns &&
                         audit_.put_begin_ns >= audit_.finalizer_end_ns && audit_.put_end_ns >= audit_.put_begin_ns;
    // A native error is not proof that no remote sample can exist. No retry.
    return {audit_.put_code == 0 ? PutOutcome::Accepted : PutOutcome::Unknown, audit_.put_code};
}
}  // namespace perception::rns
