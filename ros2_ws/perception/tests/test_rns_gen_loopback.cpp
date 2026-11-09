/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_gen_loopback.cpp
 * Brief: Six real GEN encoder packets over independent local TCP sessions.
 *
 * Description:
 * What it checks. Each GEN event kind is encoded and put through the GEN transport to a local
 * subscriber, with constructor and put probes and the QoS printed for the record.
 * Scope. Diagnostic only: no production router, RT load, delivery or timing acceptance.
 */

#include "output/rns_gen_events.hpp"
#include "output/rns_gen_transport.hpp"
#include "output/rns_system_clock.hpp"
#include <arpa/inet.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>

namespace {
using namespace perception::rns;
using Steady = std::chrono::steady_clock;
constexpr std::size_t kPackets = 6, kPacketCapacity = 65536, kKeyCapacity = 128;
constexpr auto kWait = std::chrono::seconds(5);
std::size_t rejected_constructors = 0, rejected_puts = 0;
static_assert(!std::is_constructible_v<ZenohGenSession, ZenohRtSession&>, "GEN cannot adopt an external RT session");
static_assert(!std::is_constructible_v<ZenohGenTransport, ZenohRtSession&, std::string, PublishQos, std::size_t>,
              "GEN transport cannot share an externally supplied RT session");
static_assert(!std::is_constructible_v<ZenohRtTransport, ZenohGenSession&, std::string, PublishQos, std::size_t>,
              "GEN session is not exposed to RT transports");
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
std::int64_t mono_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0 || ts.tv_sec < 0 || ts.tv_nsec < 0 || ts.tv_nsec >= 1000000000) return -1;
    if (static_cast<std::uint64_t>(ts.tv_sec) >
        (UINT64_C(9223372036854775807) - static_cast<std::uint64_t>(ts.tv_nsec)) / UINT64_C(1000000000)) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * INT64_C(1000000000) + ts.tv_nsec;
}
std::string ephemeral_endpoint() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    require(fd >= 0, "create ephemeral loopback reservation socket");
    sockaddr_in address{};
    address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = 0;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(fd); throw std::runtime_error("bind ephemeral loopback port");
    }
    socklen_t length = sizeof(address);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) != 0) {
        ::close(fd); throw std::runtime_error("read ephemeral loopback port");
    }
    const auto port = ntohs(address.sin_port); ::close(fd);
    require(port != 0, "OS chose nonzero port");
    // SDK cannot adopt fd. Fail if the close/open race is lost; never probe a live service as fallback.
    return "tcp/127.0.0.1:" + std::to_string(port);
}
std::string nonce() {
    std::array<unsigned char, 16> bytes{};
    require(::getentropy(bytes.data(), bytes.size()) == 0, "OS instance nonce entropy");
    constexpr char hex[] = "0123456789abcdef";
    std::string out(32, '0');
    for (std::size_t i = 0; i < bytes.size(); ++i) { out[i * 2] = hex[bytes[i] >> 4]; out[i * 2 + 1] = hex[bytes[i] & 15]; }
    require(out != std::string(32, '0'), "nonzero test instance nonce"); return out;
}
struct NativeQos { int congestion_control = -1, priority = -1; bool express = false; int reliability = -1; };
constexpr NativeQos expected_qos() { return {Z_CONGESTION_CONTROL_BLOCK, Z_PRIORITY_DATA, false, Z_RELIABILITY_RELIABLE}; }
bool same_qos(const NativeQos& a, const NativeQos& b) {
    return a.congestion_control == b.congestion_control && a.priority == b.priority &&
           a.express == b.express && a.reliability == b.reliability;
}
struct Bytes {
    std::array<char, kKeyCapacity> key{};
    std::array<char, kPacketCapacity> payload{};
    std::size_t key_size = 0, payload_size = 0;
    std::string_view key_view() const { return {key.data(), key_size}; }
    std::string_view payload_view() const { return {payload.data(), payload_size}; }
};
struct Received : Bytes { std::int64_t callback_entry_ns = -1; NativeQos qos; };
struct ReceiverState {
    std::mutex mutex;
    std::condition_variable changed;
    std::array<Received, kPackets + 3> packets{};
    std::size_t count = 0, callbacks = 0, errors = 0;
    bool closure_dropped = false;
};
struct ReceiverContext { std::shared_ptr<ReceiverState> state; };
void receive(z_loaned_sample_t* sample, void* context) noexcept {
    const auto entry = mono_ns();
    auto& state = *static_cast<ReceiverContext*>(context)->state;
    std::lock_guard<std::mutex> lock(state.mutex); ++state.callbacks;
    if (state.count == state.packets.size() || entry < 0) { ++state.errors; state.changed.notify_all(); return; }
    z_view_string_t key{}; z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key);
    const auto key_size = z_string_len(z_loan(key));
    const auto* payload = z_sample_payload(sample);
    const auto payload_size = z_bytes_len(payload);
    if (key_size >= kKeyCapacity || payload_size >= kPacketCapacity) { ++state.errors; state.changed.notify_all(); return; }
    auto& row = state.packets[state.count];
    row.callback_entry_ns = entry; row.key_size = key_size; row.payload_size = payload_size;
    std::memcpy(row.key.data(), z_string_data(z_loan(key)), key_size); row.key[key_size] = '\0';
    auto reader = z_bytes_get_reader(payload);
    const auto copied = z_bytes_reader_read(&reader, reinterpret_cast<std::uint8_t*>(row.payload.data()), payload_size);
    if (copied != payload_size || z_bytes_reader_remaining(&reader) != 0) { ++state.errors; state.changed.notify_all(); return; }
    row.payload[payload_size] = '\0';
    row.qos = {static_cast<int>(z_sample_congestion_control(sample)), static_cast<int>(z_sample_priority(sample)),
               z_sample_express(sample), static_cast<int>(z_sample_reliability(sample))};
    ++state.count; state.changed.notify_all();
}
void drop_context(void* context) noexcept {
    auto* owned = static_cast<ReceiverContext*>(context);
    {
        std::lock_guard<std::mutex> lock(owned->state->mutex);
        // Native closure-drop is after all callbacks and before any future call.
        owned->state->closure_dropped = true; owned->state->changed.notify_all();
    }
    delete owned;
}
class Receiver {
public:
    Receiver(const std::string& endpoint, const std::string& pattern, const std::shared_ptr<ReceiverState>& state) {
        auto context = std::make_unique<ReceiverContext>(ReceiverContext{state});
        const std::string json = "{mode:'peer',listen:{endpoints:['" + endpoint +
            "']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:true,multihop:false}},"
            "transport:{shared_memory:{enabled:false}}}";
        z_owned_config_t config{};
        require(zc_config_from_str(&config, json.c_str()) == 0, "parse isolated receiver config");
        const auto code = z_open(&session_, z_move(config), nullptr);
        if (code != 0) { z_drop(z_move(session_)); throw std::runtime_error("open isolated receiver listener: " + std::to_string(code)); }
        session_open_ = true;
        z_view_keyexpr_t key{};
        if (z_view_keyexpr_from_str(&key, pattern.c_str()) != 0) { close(); throw std::runtime_error("receiver key syntax"); }
        z_owned_closure_sample_t callback{};
        z_closure_sample(&callback, receive, drop_context, context.release());
        z_subscriber_options_t options{}; z_subscriber_options_default(&options);
        options.allowed_origin = Z_LOCALITY_REMOTE;
        if (z_declare_subscriber(z_loan(session_), &subscriber_, z_loan(key), z_move(callback), &options) != 0) {
            close(); throw std::runtime_error("declare independent GEN receiver");
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
// Diagnostic copy plus deliberate immediate reuse tests caller-buffer ownership;
// no native allocation-free or precise wrapper-stamp timing claim is made.
struct Capture final : RnsTransport {
    explicit Capture(ZenohGenTransport& transport) : native(transport) {}
    ZenohGenTransport& native;
    Bytes last;
    std::array<char, kPacketCapacity> reused{};
    std::size_t calls = 0;
    PutReceipt put(std::string_view key, std::string_view payload, PublishQos qos) override {
        require(key.size() < kKeyCapacity && payload.size() < kPacketCapacity, "bounded TX capture");
        ++calls; last.key_size = key.size(); last.payload_size = payload.size();
        std::memcpy(last.key.data(), key.data(), key.size()); last.key[key.size()] = '\0';
        std::memcpy(last.payload.data(), payload.data(), payload.size()); last.payload[payload.size()] = '\0';
        std::memcpy(reused.data(), payload.data(), payload.size());
        const auto result = native.put(key, {reused.data(), payload.size()}, qos);
        std::memset(reused.data(), 0x5a, payload.size());
        return result;
    }
};
struct Transmitted : Bytes { GenSendResult result; NativePutAudit native; Millis offered_wall = -1, offered_mono = -1; };
template <class F> void invalid_constructor(F fn) {
    bool rejected = false; try { fn(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid GEN constructor rejected before native declaration/open"); ++rejected_constructors;
}
void constructor_probes(ZenohGenSession& session, const std::string& key) {
    invalid_constructor([&] { ZenohGenTransport t(session, key, GenEvents::qos(), 0); });
    invalid_constructor([&] { ZenohGenTransport t(session, key, GenEvents::qos(), ZenohGenTransport::kMaxPacketBytes + 1); });
    for (const auto bad_key : {"event/error/perception", "rt/perception/status", "event/*/perception",
            "event/info/other", "/event/warn/perception", "event/warn/perception/", "Event/warn/perception",
            // GEN events are bare (11 PC-2): the robot-id form is p5_gateway's cloud layer.
            "xbrain/probe/event/warn/perception", "xbrain/probe/event/info/perception", "xbrain/probe/event/fault/perception",
            // A19-BOOT-4: a registered key must match as a whole, not as a prefix of a longer key.
            "event/warn/perception/extra", "event/warn/perception2", "event/info/perception/*"})
        invalid_constructor([&] { ZenohGenTransport t(session, bad_key, GenEvents::qos(), kPacketCapacity); });
    for (unsigned field = 0; field < 5; ++field) {
        auto qos = GenEvents::qos();
        if (field == 0) qos.priority = Priority::RealTime;
        if (field == 1) qos.priority = Priority::DataHigh;
        if (field == 2) qos.reliable = false;
        if (field == 3) qos.express = true;
        if (field == 4) qos.drop = true;
        invalid_constructor([&] { ZenohGenTransport t(session, key, qos, kPacketCapacity); });
    }
}
void put_probes(ZenohGenTransport& native, std::string_view key) {
    const auto rejected = [&](std::string_view k, std::string_view packet, PublishQos qos, int code) {
        const auto result = native.put(k, packet, qos); ++rejected_puts;
        require(result.outcome == PutOutcome::Rejected && result.native_code == code, "invalid native request/closed rejected");
        const auto audit = native.last_audit();
        require(!audit.native_called && audit.copy_begin_ns == -1 && audit.put_begin_ns == -1, "preflight made no native call or copy");
    };
    rejected("xbrain/rns_probe_invalid/event/warn/perception", "{}", GenEvents::qos(), ZenohGenTransport::kInvalidRequest);
    rejected(key, "", GenEvents::qos(), ZenohGenTransport::kInvalidRequest);
    const std::string oversized(kPacketCapacity + 1, 'x');
    rejected(key, oversized, GenEvents::qos(), ZenohGenTransport::kInvalidRequest);
    for (unsigned field = 0; field < 5; ++field) {
        auto qos = GenEvents::qos();
        if (field == 0) qos.priority = Priority::RealTime;
        if (field == 1) qos.priority = Priority::DataHigh;
        if (field == 2) qos.reliable = false;
        if (field == 3) qos.express = true;
        if (field == 4) qos.drop = true;
        rejected(key, "{}", qos, ZenohGenTransport::kInvalidRequest);
    }
    native.close(); rejected(key, "{}", GenEvents::qos(), ZenohGenTransport::kClosed);
    native.close(); require(!native.has_matching_subscriber(), "idempotently closed GEN publisher has no matching subscriber");
}
void print_qos(const NativeQos& q) {
    std::cout << "{\"congestion_control\":" << q.congestion_control << ",\"priority\":" << q.priority
              << ",\"express\":" << q.express << ",\"reliability\":" << q.reliability << '}';
}
void print_row(const Transmitted& tx, const Received& rx, std::size_t index,
              const std::string& endpoint, const std::string& config) {
    const auto& n = tx.native;
    std::cout << "{\"test\":\"rns_gen190_tcp_loopback\",\"scope\":\"diagnostic_only\",\"key\":\"" << rx.key_view()
              << "\",\"packet\":" << rx.payload_view() << ",\"endpoint\":\"" << endpoint
              << "\",\"producer_config\":" << config << ",\"receiver_allowed_origin\":\"remote\""
              << ",\"offer_wall_ms\":" << tx.offered_wall << ",\"offer_mono_ms\":" << tx.offered_mono
              << ",\"before\":" << tx.result.put_before_ms << ",\"after\":" << tx.result.put_return_ms
              << ",\"outcome\":\"accepted\",\"fifo_index\":" << index << ",\"tx_rx_exact\":true"
              << ",\"owned_copy_buffer_reused\":true,\"before_is_native_put_begin\":false"
              << ",\"native\":{\"bytes\":" << n.bytes << ",\"called\":" << n.native_called
              << ",\"clock_valid\":" << n.clock_valid << ",\"copy_code\":" << n.copy_code << ",\"put_code\":" << n.put_code
              << ",\"copy_begin_ns\":" << n.copy_begin_ns << ",\"copy_end_ns\":" << n.copy_end_ns
              << ",\"put_begin_ns\":" << n.put_begin_ns << ",\"put_end_ns\":" << n.put_end_ns << '}'
              << ",\"receive_callback_entry_ns\":" << rx.callback_entry_ns
              << ",\"intended_qos\":{\"priority\":\"data\",\"reliable\":true,\"express\":false,\"drop\":false}"
              << ",\"intended_native_qos\":";
    print_qos(expected_qos()); std::cout << ",\"observed_native_qos\":"; print_qos(rx.qos);
    std::cout << ",\"native_qos_getters_equal_intent\":" << same_qos(expected_qos(), rx.qos)
              << ",\"reliability_is_retransmission_proof\":false,\"rt_load_isolation_tested\":false"
              << ",\"counts\":{\"offered\":6,\"api_accepted\":6,\"received\":6,\"duplicate_rx\":0}}\n";
}
void run() {
    require(std::string_view(ZENOH_C) == "1.9.0", "verified native SDK is exactly Zenoh C 1.9.0");
    for (const auto endpoint : {"", "udp/127.0.0.1:1", "tcp/localhost:1", "tcp/127.0.0.1:0",
                               "tcp/127.0.0.1:65536", "tcp/127.0.0.1:1\"],listen:{endpoints:[\"tcp/0.0.0.0:1"})
        invalid_constructor([&] { ZenohGenSession session(endpoint); });
    const auto endpoint = ephemeral_endpoint();
    const auto rid = "rns_gen_probe_" + std::to_string(::getpid());
    const std::string prefix = "event/";
    const auto warn_key = prefix + "warn/perception", info_key = prefix + "info/perception";
    auto owner = std::make_shared<ReceiverState>(); auto& received = *owner;
    Receiver receiver(endpoint, prefix + "*/perception", owner);
    ZenohGenSession session(endpoint);
    constructor_probes(session, warn_key);
    ZenohGenTransport warn_native(session, warn_key, GenEvents::qos(), kPacketCapacity);
    ZenohGenTransport info_native(session, info_key, GenEvents::qos(), kPacketCapacity);
    const auto ready_deadline = Steady::now() + kWait;
    while (!(warn_native.has_matching_subscriber() && info_native.has_matching_subscriber())) {
        require(Steady::now() < ready_deadline, "GEN subscriber matching timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Capture warn(warn_native), info(info_native);
    SystemPublishClock clock;
    char boot[9]{};
    std::snprintf(boot, sizeof(boot), "%08x", static_cast<unsigned>(mono_ns()) ^ static_cast<unsigned>(::getpid()));
    ProcessEnvelope envelope(rid, boot, 5000);
    GenEvents events(envelope, clock, warn, info, nonce(), {"infer_gap", "infer_rate_low", "reconnecting"}, 8, kPacketCapacity);
    require(events.key(ReasonEdgeSeverity::Warn) == warn_key && events.key(ReasonEdgeSeverity::Info) == info_key, "encoder and native fixed keys agree");
    ReasonEdges reasons(clock.mono_millis(), 3, 8);
    std::array<Transmitted, kPackets> transmitted{};
    for (std::size_t i = 0; i < kPackets; ++i) {
        const auto now = clock.mono_millis(), wall = clock.wall_millis();
        const auto active = i % 2 ? std::uint64_t{0} : std::uint64_t{1} << (i / 2);
        require(reasons.update(active, now).enqueued_edges == 1, "distinct cause directional transition makes one local edge");
        ReasonEdge edge; require(reasons.pop(edge), "pop exact local edge for one offer");
        const auto offered = events.offer(edge, wall, now);
        require(offered.code == GenOfferCode::Accepted && offered.seq == i + 1, "one bounded offer and sequence per edge");
        transmitted[i].offered_wall = wall; transmitted[i].offered_mono = now;
    }
    require(events.queue_depth() == kPackets, "six event envelopes frozen before independent consumer sends");
    const auto clock_deadline = Steady::now() + std::chrono::milliseconds(100);
    while (clock.mono_millis() <= transmitted.back().offered_mono) {
        require(Steady::now() < clock_deadline, "host clock advanced to make queue age observable");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::exception_ptr consumer_error;
    std::thread consumer([&] {
        try {
            for (std::size_t i = 0; i < kPackets; ++i) {
                auto& tx = transmitted[i]; const auto& captured = i % 2 ? info : warn;
                tx.result = events.send_one();
                require(tx.result.code == GenSendCode::Accepted && tx.result.attempted && tx.result.clock_valid,
                        "independent GEN consumer local native put accepted");
                static_cast<Bytes&>(tx) = captured.last; tx.native = captured.native.last_audit();
                require(tx.result.stamp.seq == i + 1 && tx.result.stamp.mono_ms == tx.offered_mono &&
                        tx.result.stamp.wall_ms == tx.offered_wall, "FIFO envelope identity/time retained across queue delay");
                const auto& n = tx.native;
                require(n.native_called && n.clock_valid && n.copy_code == 0 && n.put_code == 0 && n.bytes == tx.payload_size,
                        "native owned copy, entry/return clocks and put result audit");
                require(tx.result.packet_bytes == tx.payload_size && tx.result.put_before_ms > tx.offered_mono,
                        "packet created earlier, never restamped at send");
                require(n.copy_begin_ns >= tx.result.put_before_ms * 1000000 && n.copy_end_ns <= n.put_begin_ns &&
                        n.put_end_ns / 1000000 <= tx.result.put_return_ms, "diagnostic copy/native put nested in actual consumer bracket");
            }
            require(events.send_one().code == GenSendCode::Empty, "no automatic periodic resend after draining GEN queue");
        } catch (...) { consumer_error = std::current_exception(); }
    });
    consumer.join(); if (consumer_error) std::rethrow_exception(consumer_error);
    {
        std::unique_lock<std::mutex> lock(received.mutex);
        require(received.changed.wait_for(lock, kWait, [&] { return received.count >= kPackets || received.errors; }), "GEN six receipt timeout");
        require(received.count == kPackets && received.errors == 0, "exactly six bounded callbacks");
    }
    put_probes(warn_native, warn_key);
    {
        std::unique_lock<std::mutex> lock(received.mutex);
        received.changed.wait_for(lock, std::chrono::milliseconds(50), [&] { return received.count != kPackets || received.errors; });
    }
    receiver.close();
    {
        std::unique_lock<std::mutex> lock(received.mutex);
        require(received.changed.wait_for(lock, kWait, [&] { return received.closure_dropped; }), "receiver callbacks ended before state access");
    }
    require(received.count == kPackets && received.callbacks == kPackets && received.errors == 0, "no extra/truncated/probe receipt");
    require(warn.calls == 3 && info.calls == 3 && events.queue_depth() == 0 && events.healthy(), "three warn/info events, queue empty and local journal healthy");
    require(events.offer_counters().accepted == 6 && events.send_counters().accepted == 6, "distinct offer and API accepted ledgers");
    std::cout << std::boolalpha;
    for (std::size_t i = 0; i < kPackets; ++i) {
        const auto& tx = transmitted[i]; const auto& rx = received.packets[i];
        require(tx.key_view() == rx.key_view() && tx.payload_view() == rx.payload_view(), "bounded TCP receipt FIFO and exact bytes after source reuse");
        require(rx.callback_entry_ns >= tx.native.put_begin_ns, "this callback follows this native call entry");
        require(same_qos(expected_qos(), rx.qos), "received QoS getters match DATA/RELIABLE/BLOCK/non-express");
        print_row(tx, rx, i, endpoint, session.config_json());
    }
    std::cerr << "RNS GEN Zenoh 1.9 local TCP " << endpoint << ": offered=6 API accepted=6 RX FIFO/exact=6; warn/info=3/3; "
              << rejected_constructors << " constructor and " << rejected_puts << " no-native-call put rejections. "
                 "Diagnostic only; no RT load isolation, retransmission, router, or production acceptance claim.\n";
}
}  // namespace
int main() {
    try { run(); return 0; }
    catch (const std::exception& error) { std::cerr << "RNS GEN loopback FAIL: " << error.what() << '\n'; return 1; }
}
