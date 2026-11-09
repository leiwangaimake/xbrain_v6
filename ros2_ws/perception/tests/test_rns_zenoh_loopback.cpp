/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_zenoh_loopback.cpp
 * Brief: Bounded diagnostic two-session Zenoh 1.9 TCP loopback of the RT transport.
 *
 * Description:
 * What it checks. Two local Zenoh sessions exchange puts over TCP through the RT transport with
 * its QoS, and the received bytes are compared with what was sent. It also probes rejection:
 * construction errors and session configurations that the implementation refuses before
 * config/open.
 * Scope. Byte-copy instrumentation adds overhead, so this is NOT a timing acceptance run; no
 * production router is involved.
 */

#include "output/rns_system_clock.hpp"
#include "output/rns_zenoh_transport.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

namespace {
using namespace perception::rns;
using Steady = std::chrono::steady_clock;
constexpr std::size_t kPackets = 9;
constexpr std::size_t kPacketCapacity = 65536;
constexpr std::size_t kKeyCapacity = 128;
constexpr auto kWait = std::chrono::seconds(5);

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

std::int64_t monotonic_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0 || ts.tv_sec < 0 ||
        ts.tv_nsec < 0 || ts.tv_nsec >= 1000000000) return -1;
    // Actual host uptime is bounded far below the int64 nanosecond limit.
    if (static_cast<std::uint64_t>(ts.tv_sec) >
        (UINT64_C(9223372036854775807) - static_cast<std::uint64_t>(ts.tv_nsec)) /
            UINT64_C(1000000000)) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * INT64_C(1000000000) + ts.tv_nsec;
}

std::string ephemeral_loopback_endpoint() {
    const int socket = ::socket(AF_INET, SOCK_STREAM, 0);
    require(socket >= 0, "create loopback port reservation socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;  // The OS selects the port; no production port probe.
    if (::bind(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(socket);
        throw std::runtime_error("bind ephemeral loopback port");
    }
    socklen_t size = sizeof(address);
    if (::getsockname(socket, reinterpret_cast<sockaddr*>(&address), &size) != 0) {
        ::close(socket);
        throw std::runtime_error("read selected loopback port");
    }
    const auto port = ntohs(address.sin_port);
    ::close(socket);
    require(port != 0, "OS selected a nonzero port");
    // Zenoh cannot adopt the reservation socket. If another process wins the
    // close/open race, receiver open fails; never silently pick a live service.
    return "tcp/127.0.0.1:" + std::to_string(port);
}

struct NativeQos {
    int congestion_control = -1;
    int priority = -1;
    bool express = false;
    int reliability = -1;
};

NativeQos intended_native(PublishQos qos) {
    return {qos.drop ? Z_CONGESTION_CONTROL_DROP : Z_CONGESTION_CONTROL_BLOCK,
            qos.priority == Priority::RealTime ? Z_PRIORITY_REAL_TIME : Z_PRIORITY_DATA_HIGH,
            qos.express,
            qos.reliable ? Z_RELIABILITY_RELIABLE : Z_RELIABILITY_BEST_EFFORT};
}

bool same_qos(const NativeQos& a, const NativeQos& b) {
    return a.congestion_control == b.congestion_control && a.priority == b.priority &&
           a.express == b.express && a.reliability == b.reliability;
}

struct PacketBytes {
    std::array<char, kKeyCapacity> key{};
    std::array<char, kPacketCapacity> payload{};
    std::size_t key_size = 0, payload_size = 0;
    std::string_view key_view() const { return {key.data(), key_size}; }
    std::string_view payload_view() const { return {payload.data(), payload_size}; }
};

struct Received : PacketBytes {
    std::int64_t callback_entry_ns = -1;
    NativeQos qos;
};

struct ReceiverState {
    std::mutex mutex;
    std::condition_variable changed;
    // Extra fixed slots expose duplicate/unexpected receipts without growth.
    std::array<Received, kPackets + 3> packets{};
    std::size_t count = 0, callback_count = 0, errors = 0;
    bool closure_dropped = false;
};

struct ReceiverContext {
    // The callback owns this context until its SDK drop hook runs. Even an
    // exceptional test exit cannot destroy ReceiverState under a pending call.
    std::shared_ptr<ReceiverState> state;
};

void receive_sample(z_loaned_sample_t* sample, void* context) noexcept {
    const auto entry_ns = monotonic_ns();  // First callback action, before lock/copy.
    auto& state = *static_cast<ReceiverContext*>(context)->state;
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.callback_count;
    if (state.count == state.packets.size() || entry_ns < 0) {
        ++state.errors;
        state.changed.notify_all();
        return;
    }
    z_view_string_t key{};
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key);
    const auto key_size = z_string_len(z_loan(key));
    const auto* payload = z_sample_payload(sample);
    const auto payload_size = z_bytes_len(payload);
    if (key_size >= kKeyCapacity || payload_size >= kPacketCapacity) {
        ++state.errors;
        state.changed.notify_all();
        return;
    }
    auto& received = state.packets[state.count];
    received.callback_entry_ns = entry_ns;
    received.key_size = key_size;
    received.payload_size = payload_size;
    std::memcpy(received.key.data(), z_string_data(z_loan(key)), key_size);
    received.key[key_size] = '\0';
    auto reader = z_bytes_get_reader(payload);
    const auto copied = z_bytes_reader_read(
        &reader, reinterpret_cast<std::uint8_t*>(received.payload.data()), payload_size);
    if (copied != payload_size || z_bytes_reader_remaining(&reader) != 0) {
        ++state.errors;
        state.changed.notify_all();
        return;
    }
    received.payload[payload_size] = '\0';
    received.qos = {static_cast<int>(z_sample_congestion_control(sample)),
                    static_cast<int>(z_sample_priority(sample)), z_sample_express(sample),
                    static_cast<int>(z_sample_reliability(sample))};
    ++state.count;
    state.changed.notify_all();
    // No key, payload, reader, sample, or callback loan escapes this invocation.
}

void drop_receiver_context(void* context) noexcept {
    auto* owned = static_cast<ReceiverContext*>(context);
    {
        std::lock_guard<std::mutex> lock(owned->state->mutex);
        // SDK closure contract: drop starts only after every call has ended,
        // and no call may start once drop has started.
        owned->state->closure_dropped = true;
        owned->state->changed.notify_all();
    }
    delete owned;
}

class Receiver {
public:
    Receiver(const std::string& endpoint, const std::string& pattern,
             const std::shared_ptr<ReceiverState>& state) {
        auto callback_context = std::make_unique<ReceiverContext>(ReceiverContext{state});
        const std::string config_json =
            "{mode:'peer',listen:{endpoints:['" + endpoint +
            "']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:true}},"
            "transport:{shared_memory:{enabled:false}}}";
        z_owned_config_t config{};
        require(zc_config_from_str(&config, config_json.c_str()) == 0, "parse receiver config");
        const auto open_code = z_open(&session_, z_move(config), nullptr);
        if (open_code != 0) {
            z_view_string_t error{};
            zc_get_last_error(&error);
            const std::string detail(z_string_data(z_loan(error)), z_string_len(z_loan(error)));
            z_drop(z_move(session_));
            throw std::runtime_error("open receiver loopback listener: " +
                                     std::to_string(open_code) + " " + detail);
        }
        session_open_ = true;
        z_view_keyexpr_t key{};
        if (z_view_keyexpr_from_str(&key, pattern.c_str()) != 0) {
            close();
            throw std::runtime_error("construct isolated receiver key");
        }
        z_owned_closure_sample_t callback{};
        z_closure_sample(&callback, receive_sample, drop_receiver_context, callback_context.release());
        z_subscriber_options_t options{};
        z_subscriber_options_default(&options);
        options.allowed_origin = Z_LOCALITY_REMOTE;
        if (z_declare_subscriber(z_loan(session_), &subscriber_, z_loan(key),
                                 z_move(callback), &options) != 0) {
            close();
            throw std::runtime_error("declare independent receiver subscriber");
        }
        subscriber_open_ = true;
    }
    ~Receiver() { close(); }
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;
    void close() noexcept {
        if (subscriber_open_) { z_drop(z_move(subscriber_)); subscriber_open_ = false; }
        if (session_open_) { z_drop(z_move(session_)); session_open_ = false; }
    }
private:
    z_owned_session_t session_{};
    z_owned_subscriber_t subscriber_{};
    bool session_open_ = false, subscriber_open_ = false;
};

// Diagnostic-only wrapper copies exact finalized TX bytes during the callback,
// before native put consumes ownership. This test-only capture is explicitly
// measured inside the finalizer bracket; the production native implementation
// has no such copy. Never read a consumed bytes pointer after native put.
struct CaptureTransport final : RnsTransport {
    explicit CaptureTransport(ZenohRtTransport& transport) : native(transport) {}
    ZenohRtTransport& native;
    PacketBytes last;
    PublishQos last_qos{Priority::DataHigh, true, false};
    std::size_t calls = 0;
    PutReceipt put(std::string_view key, std::string_view packet, PublishQos qos) override {
        require(key.size() < last.key.size() && packet.size() < last.payload.size(),
                "diagnostic TX capture capacity");
        ++calls;
        last.key_size = key.size(); last.payload_size = packet.size(); last_qos = qos;
        std::memcpy(last.key.data(), key.data(), key.size()); last.key[key.size()] = '\0';
        std::memcpy(last.payload.data(), packet.data(), packet.size());
        last.payload[packet.size()] = '\0';
        return native.put(key, packet, qos);
    }
    PutReceipt put_finalized(std::string_view key, char* packet, std::size_t size,
                           PublishQos qos, PacketFinalizer& finalizer) override {
        require(key.size() < last.key.size() && size < last.payload.size(), "capture capacity");
        ++calls;
        last.key_size = key.size(); last.payload_size = size; last_qos = qos;
        std::memcpy(last.key.data(), key.data(), key.size()); last.key[key.size()] = '\0';
        struct Capture final : PacketFinalizer {
            PacketFinalizer& inner;
            PacketBytes& last;
            Capture(PacketFinalizer& f, PacketBytes& p) : inner(f), last(p) {}
            bool finalize(char* data, std::size_t size) noexcept override {
                if (!inner.finalize(data, size)) return false;
                std::memcpy(last.payload.data(), data, size); last.payload[size] = '\0';
                return true;
            }
        } capture(finalizer, last);
        return native.put_finalized(key, packet, size, qos, capture);
    }
};

struct Transmitted : PacketBytes {
    PublishResult result;
    NativePutAudit native;
    PublishQos qos{Priority::DataHigh, true, false};
    PublishCounters counts;
};

Profile profile(Millis capture) {
    Profile body;
    body.t_capture_mono_ms = capture;
    body.z_pass_m = .75F; body.range_max_m = 6; body.blind_near_m = .5F;
    body.angle_min_rad = -.01F; body.angle_step_rad = .01F; body.bins.resize(3);
    body.bins[0].d_block = 2.5F; body.bins[0].src = kS; body.bins[2].src = kG;
    return body;
}

Objects objects(Millis capture) {
    Objects body; body.t_capture_mono_ms = capture;
    Object object;
    object.track_id = 9; object.class_id = 0; object.class_name = "箱体\"\n\x01";
    object.confidence = .9F; object.semantic_status = "confirmed";
    object.geometry = ObjectGeometry{{{2, -1}, {3, -1}, {3, 1}, {2, 1}}, .1F, 1};
    object.velocity_xy = std::array<float, 2>{0, 0}; // Preserve this legacy measured fixture.
    object.velocity_frame = "raw"; object.velocity_status = "warming_up";
    object.depth_quality = "fair";
    object.first_seen_mono_ms = capture; object.last_seen_mono_ms = capture;
    body.objects.push_back(object);
    return body;
}

Status status() {
    Status body; body.fps_depth = 30; body.fps_infer = 25;
    body.latency_ms_p50 = 100; body.latency_ms_p99 = 180;
    body.infer_gap_ms_p99 = 47; body.infer_gap_ms_max = 62;
    body.invalid_pixel_ratio = .1;
    body.degraded_reasons = {"ground_fit_fallback", "ground_free_withdrawn"};
    return body;
}

void save(Transmitted& tx, const PublishResult& result, const CaptureTransport& capture,
          const PublisherChannel& channel) {
    require(static_cast<bool>(result), "PublisherChannel local put accepted");
    require(result.audit.receipt.outcome == PutOutcome::Accepted && result.audit.clock_valid,
            "local acceptance and valid adapter clock are separate ledger facts");
    static_cast<PacketBytes&>(tx) = capture.last;
    tx.result = result; tx.native = capture.native.last_audit();
    tx.qos = capture.last_qos; tx.counts = channel.counters();
    const auto& n = tx.native;
    require(n.native_called && n.clock_valid && n.copy_code == 0 && n.put_code == 0,
            "owned byte copy and native put succeeded with valid clocks");
    require(n.bytes == tx.payload_size, "native audit matches exact transmitted byte count");
    require(n.finalized && !n.cancelled && n.copy_begin_ns <= n.copy_end_ns &&
            n.copy_end_ns <= n.finalizer_begin_ns &&
            n.finalizer_begin_ns / 1000000 <= result.audit.put_before_ms &&
            result.audit.put_before_ms <= n.finalizer_end_ns / 1000000 &&
            n.finalizer_end_ns <= n.put_begin_ns &&
            n.put_begin_ns <= n.put_end_ns &&
            n.put_end_ns / 1000000 <= result.audit.put_return_ms,
            "owned-byte preparation precedes finalizer clock; native entry follows it");
}

void print_qos(const NativeQos& qos) {
    std::cout << "{\"congestion_control\":" << qos.congestion_control
              << ",\"priority\":" << qos.priority << ",\"express\":" << qos.express
              << ",\"reliability\":" << qos.reliability << '}';
}

void print_row(const Transmitted& tx, const Received& rx, std::size_t receiver_count,
              const std::string& endpoint, const std::string& producer_config) {
    const auto& a = tx.result.audit;
    const auto& n = tx.native;
    const auto intended = intended_native(tx.qos);
    const bool exact = tx.key_view() == rx.key_view() && tx.payload_view() == rx.payload_view();
    std::cout << "{\"test\":\"rns_zenoh190_tcp_loopback\",\"scope\":\"diagnostic_only\","
                 "\"key\":\"" << rx.key_view() << "\",\"packet\":" << rx.payload_view()
              << ",\"endpoint\":\"" << endpoint << "\",\"producer_config\":" << producer_config
              << ",\"receiver_allowed_origin\":\"remote\""
              << ",\"before\":" << a.put_before_ms << ",\"after\":" << a.put_return_ms
              << ",\"capture\":" << a.capture_ms << ",\"outcome\":\"accepted\""
              << ",\"before_is_native_put_begin\":false,\"wrapper_copy_overhead\":true"
              << ",\"native\":{\"bytes\":" << n.bytes << ",\"called\":" << n.native_called
              << ",\"clock_valid\":" << n.clock_valid << ",\"copy_code\":" << n.copy_code
              << ",\"put_code\":" << n.put_code << ",\"copy_begin_ns\":" << n.copy_begin_ns
              << ",\"copy_end_ns\":" << n.copy_end_ns << ",\"put_begin_ns\":" << n.put_begin_ns
              << ",\"put_end_ns\":" << n.put_end_ns
              << ",\"finalizer_begin_ns\":" << n.finalizer_begin_ns
              << ",\"finalizer_end_ns\":" << n.finalizer_end_ns
              << ",\"finalized\":" << n.finalized << ",\"cancelled\":" << n.cancelled << '}'
              << ",\"receive_callback_entry_ns\":" << rx.callback_entry_ns
              << ",\"tx_rx_exact\":" << exact
              << ",\"intended_qos\":{\"drop\":" << tx.qos.drop
              << ",\"reliable\":" << tx.qos.reliable << ",\"express\":" << tx.qos.express
              << ",\"priority\":\""
              << (tx.qos.priority == Priority::RealTime ? "real_time" : "data_high")
              << "\"},\"intended_native_qos\":";
    print_qos(intended);
    std::cout << ",\"observed_native_qos\":";
    print_qos(rx.qos);
    std::cout << ",\"native_qos_getters_equal_intent\":" << same_qos(intended, rx.qos)
              << ",\"reliability_is_retransmission_proof\":false"
              << ",\"counts\":{\"channel_attempted\":" << tx.counts.attempted
              << ",\"channel_accepted\":" << tx.counts.accepted
              << ",\"total_tx\":" << kPackets << ",\"total_rx\":" << receiver_count
              << ",\"duplicate_rx\":0}}\n";
}

void rejection_probes(ZenohRtTransport& transport, std::string_view key, PublishQos qos) {
    const auto rejects = [&](std::string_view probe_key, std::string_view packet, PublishQos probe_qos) {
        const auto result = transport.put(probe_key, packet, probe_qos);
        require(result.outcome == PutOutcome::Rejected, "invalid native put is rejected");
        require(!transport.last_audit().native_called, "rejection never invokes z_publisher_put");
    };
    rejects("xbrain/rns_probe_invalid/rt/perception/profile", "{}", qos);
    auto bad = qos; bad.priority = Priority::DataHigh; rejects(key, "{}", bad);
    bad = qos; bad.reliable = !qos.reliable; rejects(key, "{}", bad);
    bad = qos; bad.express = !qos.express; rejects(key, "{}", bad);
    bad = qos; bad.drop = !qos.drop; rejects(key, "{}", bad);
    const std::string oversized(kPacketCapacity + 1, 'x');
    rejects(key, oversized, qos);
    struct Cancel final : PacketFinalizer {
        ZenohRtTransport& native;
        bool close_here = false;
        unsigned calls = 0;
        explicit Cancel(ZenohRtTransport& n) : native(n) {}
        bool finalize(char* data, std::size_t size) noexcept override {
            ++calls;
            if (size != 2 || data[0] != '{' || data[1] != '}') std::abort();
            data[0] = '['; // privately owned mutable bytes, never published
            if (close_here) { native.close(); return true; }
            return false;
        }
    } cancel(transport);
    char tiny[] = "{}";
    require(transport.put_finalized(key, nullptr, 2, qos, cancel).native_code == ZenohRtTransport::kInvalidRequest &&
            !transport.last_audit().native_called && cancel.calls == 0, "null prepare rejection clears previous audit");
    for (unsigned i = 0; i < 32; ++i) {
        require(transport.put_finalized(key, tiny, 2, qos, cancel).native_code == ZenohRtTransport::kCancelled,
                "cancel private native bytes without a put");
        const auto a = transport.last_audit();
        require(a.cancelled && !a.finalized && !a.native_called && tiny[0] == '{',
                "cancel cleanup keeps caller buffer immutable and reusable");
    }
    cancel.close_here = true;
    require(transport.put_finalized(key, tiny, 2, qos, cancel).native_code == ZenohRtTransport::kClosed,
            "close during synchronous finalizer cancels pending owned bytes");
    require(!transport.last_audit().native_called && transport.last_audit().cancelled,
            "closed finalizer cannot invoke native put");
    transport.close();
    rejects(key, "{}", qos);
    transport.close();  // Idempotent shutdown is safe.
    require(!transport.has_matching_subscriber(), "closed publisher has no matching subscriber");
}

template <class Construct>
void rejects_construction(Construct&& construct) {
    bool rejected = false;
    try { construct(); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid configuration rejected before native construction");
}

void rejected_session_configurations() {
    // Implementation rejects these strings before config/open. In particular,
    // the non-TCP/nonliteral-loopback strings are never endpoint probes.
    for (const auto* endpoint : {"", "udp/127.0.0.1:1", "tcp/localhost:1",
                                 "tcp/127.0.0.1:0", "tcp/127.0.0.1:65536",
                                 "tcp/127.0.0.1:1\"],listen:{endpoints:[\"tcp/0.0.0.0:1"}) {
        rejects_construction([&] { ZenohRtSession invalid(endpoint); });
    }
}

void rejected_publisher_configurations(ZenohRtSession& session, const std::string& key,
                                     PublishQos qos) {
    rejects_construction([&] { ZenohRtTransport invalid(session, key, qos, 0); });
    rejects_construction([&] { ZenohRtTransport invalid(session, key + "/*", qos, kPacketCapacity); });
    rejects_construction([&] {
        ZenohRtTransport invalid(session, "xbrain/BAD/rt/perception/profile", qos, kPacketCapacity);
    });
    // A19-BOOT-4: keys that are not in the registry of 11 section 2.2.1 for this process are refused
    // even without a wildcard and with the right QoS: a registered key with a suffix (it must match
    // as a whole, not as a prefix), other names under the same namespace, and the old V5 key.
    const std::string robot = key.substr(0, key.find("/rt/perception/"));
    for (const std::string& unregistered : {key + "_v2", key + "/x", robot + "/rt/perception/detections",
                                           robot + "/rt/perception/targets", robot + "/rt/perception/pointcloud",
                                           robot + "/perception/detections"}) {
        rejects_construction([&] { ZenohRtTransport invalid(session, unregistered, qos, kPacketCapacity); });
    }
    for (unsigned field = 0; field < 4; ++field) {
        auto bad = qos;
        if (field == 0) bad.priority = Priority::DataHigh;
        if (field == 1) bad.reliable = !bad.reliable;
        if (field == 2) bad.express = !bad.express;
        if (field == 3) bad.drop = !bad.drop;
        rejects_construction([&] { ZenohRtTransport invalid(session, key, bad, kPacketCapacity); });
    }
}

void run() {
    require(std::string_view(ZENOH_C) == "1.9.0", "test requires verified Zenoh C 1.9.0 SDK");
    rejected_session_configurations();
    const auto endpoint = ephemeral_loopback_endpoint();
    const auto rid = "rns_probe_" + std::to_string(::getpid());
    const auto prefix = "xbrain/" + rid + "/rt/perception/";
    SystemPublishClock clock;
    const auto start_ms = clock.mono_millis();
    require(start_ms >= 0, "actual CLOCK_MONOTONIC available");
    char boot[9]{};
    std::snprintf(boot, sizeof(boot), "%08x", static_cast<unsigned>(monotonic_ns()) ^
                                             static_cast<unsigned>(::getpid()));
    ProcessEnvelope envelope(rid, boot, 5000);
    require(envelope.valid(), "isolated process identity is valid");
    // No actual ClockStatus is supplied; ts_sync stays honestly false.
    auto received_owner = std::make_shared<ReceiverState>();
    auto& received = *received_owner;
    Receiver receiver(endpoint, prefix + "*", received_owner);
    ZenohRtSession producer(endpoint);
    const PublishQos fast_qos{Priority::RealTime, false, true, true};
    const PublishQos status_qos{Priority::DataHigh, true, false, true};
    rejected_publisher_configurations(producer, prefix + "profile", fast_qos);
    ZenohRtTransport profile_native(producer, prefix + "profile", fast_qos, kPacketCapacity);
    ZenohRtTransport objects_native(producer, prefix + "objects", fast_qos, kPacketCapacity);
    ZenohRtTransport status_native(producer, prefix + "status", status_qos, kPacketCapacity);
    CaptureTransport profile_tx(profile_native), objects_tx(objects_native), status_tx(status_native);
    ObjectsCadence cadence(start_ms, 32);
    PublisherChannel profile_channel(Topic::Profile, envelope, clock, profile_tx, 32768, kPacketCapacity);
    PublisherChannel objects_channel(Topic::Objects, envelope, clock, objects_tx, 32768, kPacketCapacity, &cadence);
    PublisherChannel status_channel(Topic::Status, envelope, clock, status_tx, 32768, kPacketCapacity);
    require(profile_channel.valid() && objects_channel.valid() && status_channel.valid(), "three valid channels");

    const auto matching_deadline = Steady::now() + kWait;
    while (!(profile_native.has_matching_subscriber() && objects_native.has_matching_subscriber() &&
             status_native.has_matching_subscriber())) {
        require(Steady::now() < matching_deadline, "matching subscriber timeout after 5 seconds");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    struct CancelBeforeRealPackets final : PacketFinalizer {
        ZenohRtTransport& transport;
        std::string_view key;
        PublishQos qos;
        CancelBeforeRealPackets(ZenohRtTransport& t, std::string_view k, PublishQos q)
            : transport(t), key(k), qos(q) {}
        bool finalize(char*, std::size_t) noexcept override {
            const auto before = transport.last_audit();
            const auto nested = transport.put(key, "{}", qos);
            const auto nested_null = transport.put_finalized(key, nullptr, 2, qos, *this);
            if (nested.native_code != ZenohRtTransport::kBusy || nested_null.native_code != ZenohRtTransport::kBusy ||
                transport.last_audit().copy_begin_ns != before.copy_begin_ns) std::abort();
            return false;
        }
    } cancel_before(objects_native, objects_channel.key(), fast_qos);
    char cancelled_packet[] = "{}";
    require(objects_native.put_finalized(objects_channel.key(), cancelled_packet, 2, fast_qos,
                cancel_before).native_code == ZenohRtTransport::kCancelled,
            "cancel plus reentrant negative controls before successful subsequent packets");
    std::array<Transmitted, kPackets> transmitted{};
    Millis previous_status = -1;
    for (std::size_t round = 0; round < 3; ++round) {
        // Test pacing only: actual system clocks still stamp every packet. This
        // avoids same-ms status duplicates; it establishes no cadence gate.
        const auto advance_deadline = Steady::now() + std::chrono::milliseconds(100);
        auto capture_ms = clock.mono_millis();
        while (capture_ms <= previous_status) {
            require(Steady::now() < advance_deadline, "test clock did not advance within 100 ms");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            capture_ms = clock.mono_millis();
        }
        require(capture_ms >= 0, "live capture clock valid");
        save(transmitted[round * 3], profile_channel.publish(profile(capture_ms)), profile_tx, profile_channel);
        save(transmitted[round * 3 + 1], objects_channel.publish(objects(capture_ms)), objects_tx, objects_channel);
        const auto result = status_channel.publish(status());
        save(transmitted[round * 3 + 2], result, status_tx, status_channel);
        previous_status = result.audit.put_before_ms;
    }
    const auto receipt_deadline = Steady::now() + kWait;
    {
        std::unique_lock<std::mutex> lock(received.mutex);
        require(received.changed.wait_until(lock, receipt_deadline, [&] {
                    return received.count >= kPackets || received.errors != 0;
                }), "remote receipt timeout after 5 seconds");
        require(received.errors == 0 && received.count == kPackets, "nine bounded remote receipts");
    }
    rejection_probes(profile_native, profile_channel.key(), fast_qos);
    {
        std::unique_lock<std::mutex> lock(received.mutex);
        // A bounded observation tail catches any late duplicate/negative send.
        received.changed.wait_for(lock, std::chrono::milliseconds(50), [&] {
            return received.count != kPackets || received.errors != 0;
        });
    }
    receiver.close();
    {
        std::unique_lock<std::mutex> lock(received.mutex);
        require(received.changed.wait_for(lock, kWait, [&] { return received.closure_dropped; }),
                "callback closure drop did not confirm all calls ended within 5 seconds");
    }
    // The explicit closure-drop notification proves no callback can mutate the
    // retained state; subscriber undeclaration alone is not assumed to join.
    require(received.errors == 0 && received.count == kPackets && received.callback_count == kPackets,
            "no unexpected, duplicate, truncated, or rejection-probe receipt");
    require(profile_tx.calls == 3 && objects_tx.calls == 3 && status_tx.calls == 3,
            "exactly three publisher calls on each topic");
    const auto cadence_snapshot = cadence.snapshot(clock.mono_millis());
    require(cadence_snapshot.unique_results + cadence_snapshot.excluded_results == 3 &&
            cadence_snapshot.valid && !cadence_snapshot.full_window && !cadence_snapshot.pass,
            "three Objects identities, including honest startup warmup, cannot pass full-window cadence");
    std::array<bool, kPackets> matched{};
    std::size_t qos_differences = 0;
    std::cout << std::boolalpha;
    for (std::size_t index = 0; index < transmitted.size(); ++index) {
        const auto& tx = transmitted[index];
        require(tx.result.audit.seq == index + 1, "one process-wide unique sequence across topics");
        std::size_t matches = 0, match = 0;
        for (std::size_t candidate = 0; candidate < received.count; ++candidate) {
            const auto& rx = received.packets[candidate];
            if (tx.key_view() == rx.key_view() && tx.payload_view() == rx.payload_view()) {
                ++matches; match = candidate;
            }
        }
        require(matches == 1 && !matched[match], "match unique exact key plus packet bytes, independent of arrival order");
        matched[match] = true;
        const auto& rx = received.packets[match];
        require(rx.callback_entry_ns >= tx.native.put_begin_ns, "remote callback follows this native call entry");
        if (!same_qos(intended_native(tx.qos), rx.qos)) ++qos_differences;
        print_row(tx, rx, received.count, endpoint, producer.config_json());
    }
    std::cerr << "RNS Zenoh 1.9 local TCP " << endpoint
              << ": TX accepted=9 RX exact=9 duplicates=0; "
                 "7 put rejection probes made no native put; 13 invalid constructors rejected. "
                 "Native QoS getter differences="
              << qos_differences << "/9. Diagnostic only; no timing, retransmission, camera, "
                 "RNS product, or sustained-cadence acceptance claim.\n";
}
}  // namespace

int main() {
    try { run(); return 0; }
    catch (const std::exception& error) {
        std::cerr << "RNS Zenoh loopback FAIL: " << error.what() << '\n';
        return 1;
    }
}
