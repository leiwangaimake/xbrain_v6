/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_zenoh_owners.cpp
 * Brief: Three independent RT owners over real local TCP, with an explicit Objects pause before publish.
 *
 * Description:
 * What it checks. Profile, Objects and Status run as independent owners on their own sessions
 * through a local router; an explicit pause of the Objects owner before its put shows that one
 * owner never blocks the others, and the received bytes match what each owner sent with its QoS.
 * Scope. Byte-copy instrumentation adds overhead: this is NOT a timing acceptance run.
 */

#include "output/rns_system_clock.hpp"
#include "output/rns_zenoh_transport.hpp"
#include "output/rns_status_heartbeat.hpp"
#include <algorithm>
#include <atomic>
#include <exception>
#include <sstream>
#include <vector>

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
constexpr std::size_t kPackets = 7;
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



// Receiver and exact-byte wrapper above are test-local copies of the existing
// pinned 1.9.0 loopback instrumentation, not includes of another test main.
constexpr std::string_view kScope = "synthetic_three_owners_real_tcp_pre_publish_pause";
enum class Owner : std::size_t { Profile, Objects, Status };
constexpr std::array<std::string_view,3> kNames{"profile","objects","status"};
struct PhaseEvent { std::string_view phase; std::int64_t mono_ns=-1; };
struct Transmitted : PacketBytes {
    PublishResult result;
    NativePutAudit native;
    PublishQos qos{Priority::DataHigh,true,false,true};
    std::string_view phase;
    HeartbeatResult heartbeat;
};
struct OwnerState {
    std::thread::id thread_id;
    std::array<Transmitted,3> packets{};
    std::size_t count=0;
    std::array<PhaseEvent,8> events{};
    std::size_t event_count=0;
    void event(std::string_view name) {
        require(event_count<events.size(),"bounded phase log");
        const auto now=monotonic_ns(); require(now>=0,"phase host clock");
        events[event_count++]={name,now};
    }
};
struct Gate {
    std::mutex mutex;
    std::condition_variable changed;
    std::atomic<bool> abort{false};
    std::exception_ptr error;
    std::size_t ready=0;
    bool start=false,paused=false,release=false,profile_done=false,status_done=false;
    std::int64_t paused_begin_ns=-1,paused_end_ns=-1,release_requested_ns=-1;
    void fail(std::exception_ptr e) noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if(!error) error=e;
            abort.store(true);
        }
        changed.notify_all();
    }
};
template<class Pred> void gate_wait(Gate& g,Pred predicate,const char* description) {
    std::unique_lock<std::mutex> lock(g.mutex);
    const bool finished=g.changed.wait_until(lock,Steady::now()+kWait,[&] {
        return g.abort.load()||predicate();
    });
    require(finished&&!g.abort.load(),description);
}
void interruptible_delay(Gate& g,std::chrono::milliseconds delay) {
    std::unique_lock<std::mutex> lock(g.mutex);
    g.changed.wait_until(lock,Steady::now()+delay,[&]{return g.abort.load();});
    require(!g.abort.load(),"delay cancelled");
}
void wait_millis(Gate& g,SystemPublishClock& clock,Millis wanted) {
    const auto end=Steady::now()+kWait;
    while(clock.mono_millis()<wanted) {
        require(Steady::now()<end,"heartbeat wait deadline");
        interruptible_delay(g,std::chrono::milliseconds(2));
    }
}
void save(OwnerState& owner,std::string_view phase,const PublishResult& r,
          const CaptureTransport& transport,const HeartbeatResult& heartbeat={}) {
    require(r&&r.audit.attempted&&r.audit.clock_valid&&
            r.audit.receipt.outcome==PutOutcome::Accepted,"actual native put accepted");
    require(owner.count<owner.packets.size(),"bounded owner packet ledger");
    auto& tx=owner.packets[owner.count++];
    static_cast<PacketBytes&>(tx)=transport.last;
    tx.result=r;tx.native=transport.native.last_audit();tx.qos=transport.last_qos;
    tx.phase=phase;tx.heartbeat=heartbeat;
    const auto& n=tx.native;
    require(n.native_called&&n.clock_valid&&n.finalized&&!n.cancelled&&
            n.copy_code==0&&n.put_code==0&&n.bytes==tx.payload_size,"native audit bound");
    require(n.copy_begin_ns>=0&&n.copy_begin_ns<=n.copy_end_ns&&
        n.copy_end_ns<=n.finalizer_begin_ns&&n.finalizer_begin_ns<=n.finalizer_end_ns&&
        n.finalizer_end_ns<=n.put_begin_ns&&n.put_begin_ns<=n.put_end_ns,"native clock order");
    require(n.finalizer_begin_ns/1000000<=r.audit.put_before_ms&&
        r.audit.put_before_ms<=n.finalizer_end_ns/1000000&&
        n.put_end_ns/1000000<=r.audit.put_return_ms,"put ms/ns brackets");
}
Profile synthetic_profile(Millis capture) {
    Profile p;p.t_capture_mono_ms=capture;p.extrinsic_calibrated=false;
    p.z_pass_m=.75F;p.range_max_m=6;p.angle_min_rad=-.01F;p.angle_step_rad=.01F;
    p.bins.resize(3); // Explicit unknown ground; no fabricated FREE.
    p.bins[0].d_block=2.5F;p.bins[0].src=kS;
    return p;
}
Objects synthetic_objects(Millis capture) {
    Objects out;out.t_capture_mono_ms=capture;out.extrinsic_calibrated=false;
    Object o;o.track_id=9;o.class_id=0;o.class_name="synthetic_box";o.confidence=.9F;
    o.semantic_status="confirmed";o.velocity_frame="raw";o.velocity_status="warming_up";
    o.depth_quality="poor";o.first_seen_mono_ms=capture;o.last_seen_mono_ms=capture;
    // Explicit synthetic unlocalized detection: no zero geometry or velocity.
    out.objects.push_back(std::move(o));return out;
}
Status synthetic_status() {
    Status s;s.fps_depth=0;s.fps_infer=0;s.extrinsic_calibrated=false;
    s.degraded_reasons={"roi_fallback","no_traversable_seg"};
    // This is a supplied DTO fixture, not computed HealthWindow/cadence wiring.
    // Null statistics are fixture values, not independently verified windows.
    return s;
}
void owner_run(Owner kind,OwnerState& out,Gate& gate,ProcessEnvelope& envelope,
              ZenohRtSession& session,const std::string& prefix,
              const std::shared_ptr<ReceiverState>& receiver) noexcept {
    try {
        out.thread_id=std::this_thread::get_id();out.event("owner_enter");
        SystemPublishClock clock; // Never shared with another owner.
        const auto index=static_cast<std::size_t>(kind);
        const auto topic=kind==Owner::Profile?Topic::Profile:
                         kind==Owner::Objects?Topic::Objects:Topic::Status;
        const PublishQos qos=kind==Owner::Status?
            PublishQos{Priority::DataHigh,true,false,true}:
            PublishQos{Priority::RealTime,false,true,true};
        ZenohRtTransport native(session,prefix+std::string(kNames[index]),qos,kPacketCapacity);
        CaptureTransport transport(native);
        // No shared mutable channel, cadence ledger or transport. Session loan
        // is const; SDK publisher handles and all audit/buffer state are local.
        std::optional<ObjectsCadence> cadence;
        if(kind==Owner::Objects) cadence.emplace(clock.mono_millis(),64);
        PublisherChannel channel(topic,envelope,clock,transport,60000,kPacketCapacity,
                                 cadence?&*cadence:nullptr);
        require(channel.valid(),"owner channel valid");
        const auto match_deadline=Steady::now()+kWait;
        while(!native.has_matching_subscriber()) {
            require(Steady::now()<match_deadline,"owner matching subscriber timeout");
            interruptible_delay(gate,std::chrono::milliseconds(2));
        }
        {
            std::lock_guard<std::mutex> lock(gate.mutex);++gate.ready;
        }
        gate.changed.notify_all();out.event("owner_ready");
        gate_wait(gate,[&]{return gate.start;},"owner startup cancelled");
        if(kind==Owner::Objects) {
            out.event("objects_first_publish");
            save(out,"objects_before_pause",channel.publish(synthetic_objects(clock.mono_millis())),transport);
            {
                const auto& tx=out.packets[0];
                std::unique_lock<std::mutex> lock(receiver->mutex);
                const bool arrived=receiver->changed.wait_until(lock,Steady::now()+kWait,[&] {
                    if(gate.abort.load()||receiver->errors) return true;
                    return std::any_of(receiver->packets.begin(),receiver->packets.begin()+receiver->count,
                        [&](const auto& rx){return rx.key_view()==tx.key_view()&&rx.payload_view()==tx.payload_view();});
                });
                require(arrived&&!gate.abort.load()&&!receiver->errors,"first Objects exact RX before pause");
            }
            out.event("objects_first_received");
            {
                std::unique_lock<std::mutex> lock(gate.mutex);
                gate.paused_begin_ns=monotonic_ns();
                require(gate.paused_begin_ns>=0,"pause begin clock");
                gate.paused=true;out.event("objects_pause_begin");gate.changed.notify_all();
                // The explicit pause is BEFORE calling publish/finalizer for
                // the second Objects result. No SDK lock/put is held here.
                const bool released=gate.changed.wait_until(lock,Steady::now()+kWait,[&] {
                    return gate.release||gate.abort.load();
                });
                require(released&&!gate.abort.load(),"Objects owner pause timeout/cancelled");
                gate.paused_end_ns=monotonic_ns();
                require(gate.paused_end_ns>=gate.paused_begin_ns,"pause end clock");
                out.event("objects_pause_end");
            }
            out.event("objects_second_publish");
            save(out,"objects_after_resume",channel.publish(synthetic_objects(clock.mono_millis())),transport);
        } else {
            gate_wait(gate,[&]{return gate.paused;},"waiting for Objects pause");
            out.event("objects_pause_observed");
            if(kind==Owner::Profile) {
                for(int i=0;i<3;++i) {
                    interruptible_delay(gate,std::chrono::milliseconds(100));
                    save(out,"during_objects_pause",channel.publish(synthetic_profile(clock.mono_millis())),transport);
                }
            } else {
                StatusHeartbeat heartbeat(channel,clock.mono_millis(),500);
                for(int i=0;i<2;++i) {
                    wait_millis(gate,clock,heartbeat.next_eligible_ms());
                    const auto observed=clock.mono_millis();const auto dto=synthetic_status();
                    const auto beat=heartbeat.poll(observed,&dto,observed);
                    require(beat.code==HeartbeatCode::Published,"independent StatusHeartbeat put");
                    save(out,"during_objects_pause",beat.publication,transport,beat);
                }
            }
            {
                std::lock_guard<std::mutex> lock(gate.mutex);
                if(kind==Owner::Profile) gate.profile_done=true;
                else gate.status_done=true;
            }
            gate.changed.notify_all();
        }
        out.event("owner_complete");
        // Local publisher/transport destruction happens on its actual owner.
    } catch(...) {
        gate.fail(std::current_exception());receiver->changed.notify_all();
    }
}
struct JoinOwners {
    Gate& gate;
    std::array<std::thread,3>& threads;
    bool done=false;
    void join() noexcept {
        for(auto& t:threads) if(t.joinable()) t.join();
        done=true;
    }
    ~JoinOwners() {
        if(!done) {
            gate.abort.store(true);gate.changed.notify_all();
            join(); // Every fixture wait polls abort or has a <=5s deadline.
        }
    }
};
void quoted(std::string_view s) {
    std::cout<<'"';
    for(const unsigned char c:s) {
        require(c>=32&&c<127,"ASCII diagnostic label");
        if(c=='"'||c=='\\') std::cout<<'\\';
        std::cout<<static_cast<char>(c);
    }
    std::cout<<'"';
}
std::string thread_text(std::thread::id id) {
    std::ostringstream out;out<<id;return out.str();
}
void hex(std::string_view s) {
    constexpr char digits[]="0123456789abcdef";std::cout<<'"';
    for(const unsigned char c:s) std::cout<<digits[c>>4]<<digits[c&15];
    std::cout<<'"';
}
void print_qos(const NativeQos& q) {
    std::cout<<"{\"congestion_control\":"<<q.congestion_control<<",\"priority\":"<<q.priority
        <<",\"express\":"<<q.express<<",\"reliability\":"<<q.reliability<<'}';
}
void scope() {
    std::cout<<",\"scope\":\""<<kScope<<"\",\"synthetic\":true,\"capture_is_synthetic\":true"
        <<",\"production_integrated\":false,\"camera\":false,\"model_inference\":false"
        <<",\"health_integration\":false,\"sdk_put_blocked\":false,\"twenty_hz_accepted\":false";
}
void print_packet(const OwnerState& owner,std::size_t which,std::string_view name,const Received& rx,
                 const std::string& endpoint,const std::string& config) {
    const auto& tx=owner.packets[which];const auto& a=tx.result.audit;const auto& n=tx.native;
    std::cout<<"{\"kind\":\"packet\",\"owner\":\""<<name<<"\",\"owner_index\":"<<which
        <<",\"owner_thread_id\":";quoted(thread_text(owner.thread_id));
    std::cout<<",\"phase\":\""<<tx.phase<<"\",\"key\":";quoted(rx.key_view());
    std::cout<<",\"packet\":"<<rx.payload_view()<<",\"endpoint\":\""<<endpoint
        <<"\",\"producer_config\":"<<config;scope();
    std::cout<<",\"before\":"<<a.put_before_ms<<",\"after\":"<<a.put_return_ms
        <<",\"capture\":"<<a.capture_ms<<",\"outcome\":\"accepted\",\"seq\":"<<a.seq
        <<",\"tx_rx_exact\":true,\"before_is_native_put_begin\":false,\"wrapper_copy_overhead\":true"
        <<",\"receive_callback_entry_ns\":"<<rx.callback_entry_ns<<",\"tx_hex\":";
    hex(tx.payload_view());std::cout<<",\"rx_hex\":";hex(rx.payload_view());
    std::cout<<",\"native\":{\"bytes\":"<<n.bytes<<",\"called\":"<<n.native_called
        <<",\"clock_valid\":"<<n.clock_valid<<",\"copy_code\":"<<n.copy_code<<",\"put_code\":"<<n.put_code
        <<",\"copy_begin_ns\":"<<n.copy_begin_ns<<",\"copy_end_ns\":"<<n.copy_end_ns
        <<",\"finalizer_begin_ns\":"<<n.finalizer_begin_ns<<",\"finalizer_end_ns\":"<<n.finalizer_end_ns
        <<",\"finalized\":"<<n.finalized<<",\"cancelled\":"<<n.cancelled
        <<",\"put_begin_ns\":"<<n.put_begin_ns<<",\"put_end_ns\":"<<n.put_end_ns<<'}';
    std::cout<<",\"intended_native_qos\":";print_qos(intended_native(tx.qos));
    std::cout<<",\"observed_native_qos\":";print_qos(rx.qos);
    if(name=="status") std::cout<<",\"status_fixture\":true,\"heartbeat_scheduled_ms\":"
        <<tx.heartbeat.scheduled_ms<<",\"heartbeat_lateness_ms\":"<<tx.heartbeat.lateness_ms;
    std::cout<<"}\n";
}
void run() {
    require(std::string_view(ZENOH_C)=="1.9.0","pinned SDK version");
    const auto began=Steady::now();const auto endpoint=ephemeral_loopback_endpoint();
    const auto rid="rns_probe_"+std::to_string(::getpid()),prefix="xbrain/"+rid+"/rt/perception/";
    char boot[9]{};std::snprintf(boot,sizeof(boot),"%08x",
        static_cast<unsigned>(monotonic_ns())^static_cast<unsigned>(::getpid()));
    ProcessEnvelope envelope(rid,boot,5000);
    auto rx=std::make_shared<ReceiverState>();Receiver receiver(endpoint,prefix+"*",rx);
    ZenohRtSession session(endpoint);
    Gate gate;std::array<OwnerState,3> owners{};std::array<std::thread,3> threads;
    JoinOwners joiner{gate,threads};
    try {
        for(std::size_t i=0;i<threads.size();++i)
            threads[i]=std::thread(owner_run,static_cast<Owner>(i),std::ref(owners[i]),std::ref(gate),
                std::ref(envelope),std::ref(session),std::cref(prefix),rx);
        gate_wait(gate,[&]{return gate.ready==3;},"three owners ready");
        { std::lock_guard<std::mutex> lock(gate.mutex);gate.start=true; }
        gate.changed.notify_all();
        gate_wait(gate,[&]{return gate.paused;},"Objects owner entered explicit pause");
        std::int64_t pause_begin;
        {std::lock_guard<std::mutex> lock(gate.mutex);pause_begin=gate.paused_begin_ns;}
        {
            std::unique_lock<std::mutex> lock(rx->mutex);
            const auto enough=[&] {
                if(gate.abort.load()||rx->errors) return true;
                std::size_t p=0,s=0;
                for(std::size_t i=0;i<rx->count;++i) {
                    const auto& packet=rx->packets[i];
                    if(packet.callback_entry_ns>=pause_begin) {
                        p+=packet.key_view()==prefix+"profile";s+=packet.key_view()==prefix+"status";
                    }
                }
                return p>=3&&s>=2;
            };
            require(rx->changed.wait_until(lock,Steady::now()+kWait,enough)&&
                !gate.abort.load()&&!rx->errors,"Profile and Status received while Objects paused");
        }
        // Native reception may precede put RETURN. Wait for owner-local save
        // completion as well, without reading a still-mutating OwnerState.
        gate_wait(gate,[&]{return gate.profile_done&&gate.status_done;},
                 "paused publishers completed their native calls");
        {
            std::lock_guard<std::mutex> lock(gate.mutex);
            gate.release_requested_ns=monotonic_ns();gate.release=true;
        }
        gate.changed.notify_all();joiner.join();
        {std::lock_guard<std::mutex> lock(gate.mutex);if(gate.error) std::rethrow_exception(gate.error);}
        {
            std::unique_lock<std::mutex> lock(rx->mutex);
            require(rx->changed.wait_until(lock,Steady::now()+kWait,[&] {
                return rx->count>=kPackets||rx->errors;
            })&&!rx->errors&&rx->count==kPackets,"all seven received");
            rx->changed.wait_for(lock,std::chrono::milliseconds(50),[&] {
                return rx->count!=kPackets||rx->errors;
            });
        }
    } catch(...) {
        gate.fail(std::current_exception());rx->changed.notify_all();
        joiner.join();
        // The first owner exception is the cause; a main-thread cancelled
        // rendezvous must not overwrite it with a generic timeout message.
        std::exception_ptr cause;
        {std::lock_guard<std::mutex> lock(gate.mutex);cause=gate.error;}
        std::rethrow_exception(cause);
    }
    receiver.close();
    {
        std::unique_lock<std::mutex> lock(rx->mutex);
        require(rx->changed.wait_until(lock,Steady::now()+kWait,[&]{return rx->closure_dropped;}),
                "subscriber shutdown acknowledged");
    }
    require(rx->count==kPackets&&rx->callback_count==kPackets&&!rx->errors,"no extra/duplicate receive");
    require(owners[0].count==3&&owners[1].count==2&&owners[2].count==2,"fixed owner send counts");
    require(owners[0].thread_id!=owners[1].thread_id&&owners[0].thread_id!=owners[2].thread_id&&
            owners[1].thread_id!=owners[2].thread_id,"three actual distinct owner threads");
    require(gate.paused_begin_ns<=gate.release_requested_ns&&
            gate.release_requested_ns<=gate.paused_end_ns,"real pause/resume order");
    std::array<bool,kPackets> matched{};std::array<bool,kPackets+1> seqs{};
    std::array<std::array<std::size_t,3>,3> rx_index{};
    std::size_t paused_p=0,paused_s=0,paused_o=0;
    for(std::size_t k=0;k<owners.size();++k) {
        Millis previous=-1;std::uint64_t previous_seq=0;
        for(std::size_t i=0;i<owners[k].count;++i) {
            const auto& tx=owners[k].packets[i];
            require(tx.result.audit.seq>previous_seq&&tx.result.audit.seq<=kPackets&&
                !seqs[tx.result.audit.seq],"globally unique sequence; topic-local order");
            seqs[tx.result.audit.seq]=true;previous_seq=tx.result.audit.seq;
            if(k!=2) {
                require(tx.result.audit.capture_ms>previous,"fresh per-topic synthetic capture");
                previous=tx.result.audit.capture_ms;
            }
            std::size_t found=0,which=0;
            for(std::size_t j=0;j<rx->count;++j)
                if(tx.key_view()==rx->packets[j].key_view()&&tx.payload_view()==rx->packets[j].payload_view())
                    {++found;which=j;}
            require(found==1&&!matched[which],"unique exact-byte TX/RX join");
            matched[which]=true;rx_index[k][i]=which;
            const auto& receipt=rx->packets[which];
            require(receipt.callback_entry_ns>=tx.native.put_begin_ns,"RX after native call entry");
            require(same_qos(intended_native(tx.qos),receipt.qos),"actual native QoS matches");
            const bool inside=tx.native.finalizer_begin_ns>=gate.paused_begin_ns&&
                tx.native.put_end_ns<=gate.paused_end_ns&&
                receipt.callback_entry_ns>=gate.paused_begin_ns&&receipt.callback_entry_ns<=gate.paused_end_ns;
            if(k==0) {require(inside,"all Profile packets inside pause");++paused_p;}
            else if(k==2) {require(inside,"all Status heartbeat packets inside pause");++paused_s;}
            else {
                paused_o+=inside;
                require(i?tx.native.finalizer_begin_ns>=gate.paused_end_ns:
                    receipt.callback_entry_ns<=gate.paused_begin_ns,"Objects first RX / resume boundary");
            }
        }
    }
    require(paused_p==3&&paused_s==2&&paused_o==0,"independence observation closed");
    require(Steady::now()-began<std::chrono::seconds(10),"bounded whole diagnostic");
    std::cout<<std::boolalpha;
    for(std::size_t k=0;k<owners.size();++k)
        for(std::size_t i=0;i<owners[k].count;++i)
            print_packet(owners[k],i,kNames[k],rx->packets[rx_index[k][i]],endpoint,session.config_json());
    std::cout<<"{\"kind\":\"summary\"";scope();
    std::cout<<",\"shared_session\":true,\"shared_envelope\":true,\"paused_begin_ns\":"<<gate.paused_begin_ns
        <<",\"paused_end_ns\":"<<gate.paused_end_ns<<",\"release_requested_ns\":"<<gate.release_requested_ns
        <<",\"total_tx\":7,\"total_rx\":7,\"duplicate_rx\":0,\"subscriber_closure_dropped\":true"
        <<",\"paused_profile_rx\":"<<paused_p<<",\"paused_status_rx\":"<<paused_s
        <<",\"paused_objects_rx\":"<<paused_o<<",\"owners\":[";
    for(std::size_t k=0;k<owners.size();++k) {
        if(k)std::cout<<',';
        const auto& o=owners[k];std::cout<<"{\"owner\":\""<<kNames[k]<<"\",\"thread_id\":";
        quoted(thread_text(o.thread_id));std::cout<<",\"packets\":"<<o.count<<",\"phase_events\":[";
        for(std::size_t i=0;i<o.event_count;++i) {
            if(i)std::cout<<',';
            std::cout<<"{\"phase\":\""<<o.events[i].phase<<"\",\"mono_ns\":"<<o.events[i].mono_ns<<'}';
        }
        std::cout<<"]}";
    }
    std::cout<<"]}\n";
    std::cerr<<"Three-owner TCP PASS: explicit pre-publish Objects pause, Profile 3 / Status 2 received; synthetic DTOs, no health integration or SDK-block proof.\n";
}
} // namespace
int main() {
    try{run();return 0;}
    catch(const std::exception& e){std::cerr<<"Three-owner TCP FAIL: "<<e.what()<<'\n';return 1;}
}
