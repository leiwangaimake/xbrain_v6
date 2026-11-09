/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_zenoh_generated.cpp
 * Brief: Generated r6 bodies through real isolated TCP; synthetic inputs only.
 *
 * Description:
 * What it checks. Profile, Objects and Status bodies produced by the real encoders are put through
 * the RT transport to a local subscriber and compared byte for byte with their source, with the
 * QoS and source identity printed for the record.
 * Scope. Byte-copy instrumentation adds overhead: this is NOT a timing acceptance run.
 */

#include "output/rns_system_clock.hpp"
#include "output/rns_zenoh_transport.hpp"
#include "output/rns_native_profile.hpp"
#include "output/rns_health_fields.hpp"
#include "output/rns_status_heartbeat.hpp"
#include "rns_health_feed.hpp"
#include <algorithm>
#include <cmath>
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


/* Receiver/transport instrumentation above is copied from the already bounded
 * test_rns_zenoh_loopback.cpp. This standalone target deliberately does not
 * include another test's main or alter its nine legacy packets. */
namespace geo = perception::geometry;
namespace nd = perception::native_depth;

struct Transmitted : PacketBytes {
    std::string_view name;
    PublishResult result;
    NativePutAudit native;
    PublishQos qos{Priority::DataHigh, true, false};
    bool objects_paused = false;
    bool has_profile = false, has_status = false;
    NativeProfileTick profile;
    DepthHealthObservation depth_observation;
    Millis processed_ms = -1;
    Millis inference_completed_ms = -1;
    std::size_t synthetic_roi_pixels = 0, synthetic_invalid_pixels = 0;
    HealthWindowReport window;
    DepthHealthWindowReport depth;
    HeartbeatResult heartbeat;
};

void save(Transmitted& tx, std::string_view name, const PublishResult& result,
          const CaptureTransport& transport) {
    require(result && result.audit.attempted && result.audit.clock_valid &&
            result.audit.receipt.outcome == PutOutcome::Accepted, "actual native put accepted");
    static_cast<PacketBytes&>(tx) = transport.last;
    tx.name=name; tx.result=result; tx.native=transport.native.last_audit(); tx.qos=transport.last_qos;
    const auto& n=tx.native;
    require(n.native_called && n.clock_valid && n.finalized && !n.cancelled &&
            n.copy_code==0 && n.put_code==0 && n.bytes==tx.payload_size, "native audit bound");
    require(n.copy_begin_ns<=n.copy_end_ns && n.copy_end_ns<=n.finalizer_begin_ns &&
            n.finalizer_begin_ns<=n.finalizer_end_ns && n.finalizer_end_ns<=n.put_begin_ns &&
            n.put_begin_ns<=n.put_end_ns, "owned prepare/stamp/native order");
    require(n.finalizer_begin_ns/1000000<=result.audit.put_before_ms &&
            result.audit.put_before_ms<=n.finalizer_end_ns/1000000 &&
            n.put_end_ns/1000000<=result.audit.put_return_ms, "ms/ns clock brackets");
}

geo::ProjectionBinding binding() {
    geo::ProjectionBinding b;
    b.intrinsics={128,128,320,-128,640,400}; b.distortion.reported=true;
    b.distortion.model=0; b.unit_m=1.0F/1024;
    b.R_base_camera={0,0,1,-1,0,0,0,-1,0}; b.t_base_camera={0,0,1};
    return b; // Explicit synthetic projection, NEVER a hardware calibration.
}
geo::GeometryOptions make_geometry_options() {
    geo::GeometryOptions o;
    o.prior={{0,0,1},0}; o.fit.inlier_distance_m=.015; o.fit.min_inlier_fraction=.8;
    o.fit.max_inlier_rms_m=.005; o.fit.min_xy_stddev_m=.01;
    o.fit.min_up_dot=.5; o.fit.max_prior_angle_rad=.3; o.fit.min_inliers=100;
    o.fit.hypotheses=32; o.fit.seed=0x243f6a8885a308d3ULL;
    o.fit_near_min_m=.7; o.fit_near_max_m=2.9; o.fit_prior_band_m=.25;
    o.range_max_m=6; o.blind_near_lo_m=.01; o.h_tol_m=.125;
    o.z_pass_m=.5; o.eps_neg=.1; o.w_h_m=.1;
    o.angle_min_rad=-155.0/128; o.angle_step_rad=1.0/128; o.n_bins=311;
    return o;
}
std::vector<geo::Vec3> rays(const geo::ProjectionBinding& b) {
    std::vector<geo::Vec3> out(geo::kGeometryPixels);
    for(std::size_t y=0;y<geo::kNativeHeight;++y)
        for(std::size_t x=0;x<geo::kNativeWidth;++x)
            out[y*geo::kNativeWidth+x]={(x-b.intrinsics.cx)/b.intrinsics.fx,
                                       (y-b.intrinsics.cy)/b.intrinsics.fy,1};
    return out;
}
struct SyntheticCapture {
    geo::CaptureStamp expected;
    static std::int64_t now(const void*) noexcept { return monotonic_ns(); }
    static bool resolve(const void* context,const nd::Metadata& metadata,geo::CaptureStamp& out) noexcept {
        const auto& expected=static_cast<const SyntheticCapture*>(context)->expected;
        // The fixture, not the SDK, manufactured this exposure midpoint.
        // NativeProfileConsumer independently checks every SourceId field.
        if(metadata.source.sequence!=expected.source.sequence) return false;
        out=expected; return true;
    }
    NativeProfileClock clock() { return {this,42,now,resolve}; }
};
struct GeneratedDepth {
    geo::ProjectionBinding projection=binding();
    std::vector<geo::Vec3> optical=rays(projection);
    std::vector<std::uint16_t> raster=std::vector<std::uint16_t>(geo::kGeometryPixels);
    nd::Mailbox mailbox{geo::kGeometryPixels}, health_mirror{geo::kGeometryPixels};
    nd::Frame health_frame{geo::kGeometryPixels};
    geo::NativeGeometry geometry{projection,optical.data(),optical.size(),make_geometry_options()};
    geo::NormalGrid grid{geometry,{.25,.9,.4,.02}};
    SyntheticCapture capture;
    NativeProfileConsumer consumer;
    std::uint64_t stream=0,sequence=0;
    explicit GeneratedDepth(PublisherChannel& channel)
        : consumer(mailbox,geometry,grid,capture.clock(),channel) {
        stream=mailbox.begin_stream();
        require(health_mirror.begin_stream()==stream,"controlled mirror generation");
    }
    void offer(bool invalid) {
        if(invalid) std::fill(raster.begin(),raster.end(),0);
        else for(std::size_t i=0;i<raster.size();++i)
            raster[i]=static_cast<std::uint16_t>(std::llround(1.0/optical[i].y/projection.unit_m));
        const auto now_ns=monotonic_ns();
        require(now_ns>=2000000,"synthetic timestamps representable");
        nd::View view;
        view.metadata.intrinsics=projection.intrinsics;
        view.metadata.distortion=projection.distortion; view.metadata.unit_m=projection.unit_m;
        auto& s=view.metadata.source;
        s.stream_id=stream; s.sequence=++sequence; s.color_index=sequence*3; s.depth_index=sequence*5+1;
        s.color_device_us=sequence*1000; s.depth_device_us=sequence*1000+1;
        s.host_ready_mono_ns=now_ns-1000000;
        capture.expected={s,42,now_ns-2000000};
        view.pre_d2c=true; view.pixels=raster.data(); view.bytes=raster.size()*sizeof(raster[0]);
        require(mailbox.try_offer(view)==nd::Offer::Accepted,"new synthetic Profile input");
        // Separate owned copy solely for ROI health observation, same fixture
        // bytes/metadata. Not a second consumer of the Profile mailbox.
        require(health_mirror.try_offer(view)==nd::Offer::Accepted &&
                health_mirror.try_take(health_frame)==nd::Take::Taken,"owned health mirror");
    }
    void close() { mailbox.end_stream(); health_mirror.end_stream(); }
};

Objects synthetic_objects(Millis capture,bool known_geometry) {
    Objects out; out.t_capture_mono_ms=capture; out.extrinsic_calibrated=false;
    Object o; o.track_id=9; o.class_id=0; o.class_name="synthetic_box";
    o.confidence=.9F; o.semantic_status="confirmed"; o.velocity_status="warming_up";
    o.velocity_frame="raw"; o.depth_quality=known_geometry?"fair":"poor";
    o.first_seen_mono_ms=capture; o.last_seen_mono_ms=capture;
    if(known_geometry) o.geometry=ObjectGeometry{{{2,-1},{3,-1},{3,1},{2,1}},.1F,1};
    // Null velocity is unknown, never a fabricated static zero. Unknown
    // geometry retains a detection and all four geometry wire fields are null.
    out.objects.push_back(std::move(o)); return out;
}

void wait_until(SystemPublishClock& clock,Millis wanted,const Steady::time_point& deadline) {
    while(clock.mono_millis()<wanted) {
        require(Steady::now()<deadline,"generated test overall deadline");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
void print_qos(const NativeQos& q) {
    std::cout<<"{\"congestion_control\":"<<q.congestion_control<<",\"priority\":"<<q.priority
             <<",\"express\":"<<q.express<<",\"reliability\":"<<q.reliability<<'}';
}
void print_source(const nd::SourceId& s) {
    std::cout<<"{\"stream_id\":"<<s.stream_id<<",\"sequence\":"<<s.sequence
        <<",\"color_index\":"<<s.color_index<<",\"depth_index\":"<<s.depth_index
        <<",\"color_device_us\":"<<s.color_device_us<<",\"depth_device_us\":"<<s.depth_device_us
        <<",\"host_ready_mono_ns\":"<<s.host_ready_mono_ns<<'}';
}
void print_row(const Transmitted& tx,const Received& rx,const std::string& endpoint,
              const std::string& config) {
    const auto& a=tx.result.audit; const auto& n=tx.native;
    std::cout<<"{\"case\":\""<<tx.name<<"\",\"test\":\"rns_generated_tcp\",\"key\":\""<<rx.key_view()
        <<"\",\"packet\":"<<rx.payload_view()<<",\"endpoint\":\""<<endpoint<<"\",\"producer_config\":"<<config
        <<",\"scope\":\"synthetic_generators_real_tcp_controlled_single_owner\""
        <<",\"camera\":false,\"model_inference\":false,\"capture_is_synthetic\":true"
        <<",\"production_integrated\":false,\"three_owner_concurrency_proved\":false"
        <<",\"objects_paused\":"<<tx.objects_paused<<",\"before\":"<<a.put_before_ms
        <<",\"after\":"<<a.put_return_ms<<",\"capture\":"<<a.capture_ms
        <<",\"inference_completed_ms\":"<<tx.inference_completed_ms
        <<",\"tx_rx_exact\":"<<(tx.key_view()==rx.key_view()&&tx.payload_view()==rx.payload_view())
        <<",\"receive_callback_entry_ns\":"<<rx.callback_entry_ns
        <<",\"outcome\":\"accepted\",\"native\":{\"bytes\":"<<n.bytes
        <<",\"called\":"<<n.native_called<<",\"clock_valid\":"<<n.clock_valid
        <<",\"copy_code\":"<<n.copy_code<<",\"put_code\":"<<n.put_code
        <<",\"copy_begin_ns\":"<<n.copy_begin_ns<<",\"copy_end_ns\":"<<n.copy_end_ns
        <<",\"finalizer_begin_ns\":"<<n.finalizer_begin_ns<<",\"finalizer_end_ns\":"<<n.finalizer_end_ns
        <<",\"finalized\":"<<n.finalized<<",\"cancelled\":"<<n.cancelled
        <<",\"put_begin_ns\":"<<n.put_begin_ns<<",\"put_end_ns\":"<<n.put_end_ns<<'}'
        <<",\"intended_native_qos\":";
    print_qos(intended_native(tx.qos)); std::cout<<",\"observed_native_qos\":"; print_qos(rx.qos);
    if(tx.has_profile) {
        const auto& r=tx.profile;
        std::cout<<",\"profile\":{\"route\":\""
            <<(r.route==NativeProfileRoute::Normal?"normal":"fit_failed_withdrawal")
            <<"\",\"source\":"; print_source(r.source);
        std::cout<<",\"clock_epoch\":"<<r.capture.clock_epoch
            <<",\"capture_ns\":"<<r.capture.capture_mono_ns<<",\"free_withdrawn\":"<<r.free_withdrawn
            <<",\"grid_ran\":"<<r.grid_ran<<",\"begin_ns\":"<<r.begin_ns
            <<",\"geometry_end_ns\":"<<r.geometry_end_ns<<",\"build_end_ns\":"<<r.build_end_ns
            <<",\"end_ns\":"<<r.end_ns<<",\"publish_clock_bracket_valid\":"<<r.publish_clock_bracket_valid<<'}';
        std::cout<<",\"depth_health_observation\":{\"source\":";
        print_source(tx.depth_observation.source);
        std::cout<<",\"processed_ms\":"<<tx.processed_ms
            <<",\"roi_pixels\":"<<tx.depth_observation.roi_pixels
            <<",\"invalid_pixels\":"<<tx.depth_observation.invalid_pixels
            <<",\"roi_kind\":\"lower_half_uncalibrated\"}"
            <<",\"synthetic_input\":{\"width\":640,\"height\":400,\"unit_m\":0.0009765625"
            <<",\"roi_pixels\":"<<tx.synthetic_roi_pixels
            <<",\"invalid_pixels\":"<<tx.synthetic_invalid_pixels
            <<",\"raster_kind\":\""<<(r.free_withdrawn?"all_zero":"plane_formula")<<"\"}";
    }
    if(tx.has_status) {
        std::cout<<",\"health\":{\"observed_ms\":"<<tx.window.now_ms
            <<",\"processed_frames\":"<<tx.depth.processed_frames<<",\"roi\":"<<tx.depth.roi_pixels
            <<",\"invalid\":"<<tx.depth.invalid_pixels
            <<",\"depth_window_frames\":"<<tx.window.depth.window_frames
            <<",\"inference_window_frames\":"<<tx.window.inference.window_frames
            <<",\"profile_attempted\":"<<tx.window.profile.window.attempted
            <<",\"objects_attempted\":"<<tx.window.objects.window.attempted
            <<",\"closed_gap_samples\":"<<tx.window.objects.closed_gap_samples
            <<",\"heartbeat_scheduled_ms\":"<<tx.heartbeat.scheduled_ms
            <<",\"heartbeat_lateness_ms\":"<<tx.heartbeat.lateness_ms
            <<",\"heartbeat_missed_deadlines\":"<<tx.heartbeat.missed_deadlines<<'}';
    }
    std::cout<<",\"total_tx\":9,\"total_rx\":9,\"duplicate_rx\":0}\n";
}

void run() {
    require(std::string_view(ZENOH_C)=="1.9.0","verified SDK version");
    const auto endpoint=ephemeral_loopback_endpoint();
    const auto rid="rns_probe_"+std::to_string(::getpid());
    const auto prefix="xbrain/"+rid+"/rt/perception/";
    SystemPublishClock clock;
    const auto start=clock.mono_millis();
    require(start>=0,"real host MONOTONIC available");
    const auto deadline=Steady::now()+std::chrono::seconds(25);
    char boot[9]{};
    std::snprintf(boot,sizeof(boot),"%08x",static_cast<unsigned>(monotonic_ns())^static_cast<unsigned>(::getpid()));
    ProcessEnvelope envelope(rid,boot,5000); // No ClockStatus: ts_sync=false.
    auto receiver_state=std::make_shared<ReceiverState>();
    Receiver receiver(endpoint,prefix+"*",receiver_state);
    ZenohRtSession session(endpoint);
    const PublishQos fast{Priority::RealTime,false,true,true}, health_qos{Priority::DataHigh,true,false,true};
    ZenohRtTransport p_native(session,prefix+"profile",fast,kPacketCapacity);
    ZenohRtTransport o_native(session,prefix+"objects",fast,kPacketCapacity);
    ZenohRtTransport s_native(session,prefix+"status",health_qos,kPacketCapacity);
    CaptureTransport p_tx(p_native),o_tx(o_native),s_tx(s_native);
    ObjectsCadence cadence(start,64);
    PublisherChannel p_channel(Topic::Profile,envelope,clock,p_tx,60000,kPacketCapacity);
    PublisherChannel o_channel(Topic::Objects,envelope,clock,o_tx,60000,kPacketCapacity,&cadence);
    PublisherChannel s_channel(Topic::Status,envelope,clock,s_tx,60000,kPacketCapacity);
    require(p_channel.valid()&&o_channel.valid()&&s_channel.valid(),"three channels");
    GeneratedDepth depth(p_channel);
    HealthWindow health(start,64);
    DepthHealthWindow processed(start,64);
    StatusHeartbeat heartbeat(s_channel,start,500); // Explicit fixture age, not product config.
    std::array<Transmitted,kPackets> sent{};
    std::size_t count=0;
    Millis last_processed=start;
    nd::SourceId latest_source;
    NativeProfileTick latest_profile;
    const auto matching_deadline=Steady::now()+kWait;
    while(!(p_native.has_matching_subscriber()&&o_native.has_matching_subscriber()&&s_native.has_matching_subscriber())) {
        require(Steady::now()<matching_deadline,"matching subscription");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // ALL generators/windows/channels below are owned by this one thread.
    // Only the SDK receive callback is concurrent. No three-lane isolation
    // assertion is made; objects_pause means controlled absence, not blocking.
    auto status_beat=[&](std::string_view name,bool objects_paused) {
        const auto now=clock.mono_millis();
        const auto w=health.snapshot(now); const auto d=processed.snapshot(now);
        const auto c=cadence.snapshot(now);
        Status body; body.degraded_reasons={"roi_fallback","no_traversable_seg"};
        // Controlled raw reason adapter using the same current cadence
        // evidence that apply_health_fields checks. Startup exclusions are kept;
        // this is not a production reason hysteresis/GEN wiring claim.
        if(c.infer_gap||c.max_gap_ms>50) body.degraded_reasons.emplace_back("infer_gap");
        if(c.infer_rate_low||c.max_gap_ms>100||c.over50_fraction>.01)
            body.degraded_reasons.emplace_back("infer_rate_low");
        if(latest_profile.free_withdrawn)
            for(auto reason:latest_profile.reasons) if(!reason.empty()) body.degraded_reasons.emplace_back(reason);
        HealthFieldContext context;
        context.extrinsic_calibrated=false; context.ground_seg_level=0;
        context.expected_depth_source=latest_source;
        require(apply_health_fields(w,c,now,d,context,body)==HealthFieldsCode::Ready,"computed Status ready");
        const auto result=heartbeat.poll(now,&body,now);
        require(result.code==HeartbeatCode::Published,"real heartbeat native put");
        auto& tx=sent.at(count++); save(tx,name,result.publication,s_tx);
        tx.objects_paused=objects_paused; tx.has_status=true;
        tx.window=w; tx.depth=d; tx.heartbeat=result;
        if(name=="status_startup_unknown") {
            require(!body.invalid_pixel_ratio&&!body.latency_ms_p50&&!body.infer_gap_ms_max,"honest startup nulls");
        } else if(name=="status_all_invalid") require(body.invalid_pixel_ratio==1.0,"all-invalid ratio one");
        else if(name=="status_recovered") require(body.invalid_pixel_ratio==0.0,"recovered ratio zero");
        else require(!body.invalid_pixel_ratio,"empty processing window is null");
    };
    auto profile_step=[&](std::string_view name,bool invalid,bool objects_paused) {
        depth.offer(invalid);
        latest_profile=depth.consumer.step();
        const auto& r=latest_profile;
        require(r.status==NativeProfileStatus::PutFinished&&r.put.dispatched&&r.publish_clock_bracket_valid,"generated Profile put");
        require(r.route==(invalid?NativeProfileRoute::FitFailedWithdrawal:NativeProfileRoute::Normal),"expected generated route");
        require(r.free_withdrawn==invalid,"withdrawal state");
        auto& tx=sent.at(count++); save(tx,name,r.put.publish,p_tx);
        tx.has_profile=true; tx.profile=r; tx.objects_paused=objects_paused;
        latest_source=r.source;
        const auto capture_ms=r.capture.capture_mono_ns/1000000;
        require(test_feed::depth(health,capture_ms,r.source.host_ready_mono_ns/1000000,clock.mono_millis())==HealthEventCode::Accepted,"real fixture depth arrival");
        require(test_feed::publish(health,Topic::Profile,r.put.publish.audit,clock.mono_millis())==HealthEventCode::Accepted,"Profile put health");
        const auto observation=observe_depth_health(depth.health_frame,{});
        require(observation.source.stream_id==r.source.stream_id&&observation.source.sequence==r.source.sequence&&
                observation.source.host_ready_mono_ns==r.source.host_ready_mono_ns,"same generated frame health");
        last_processed=clock.mono_millis();
        tx.depth_observation=observation; tx.processed_ms=last_processed;
        // Independent fixture-side denominator/zero count, not copied from
        // the health observer. Python can also use raster_kind to cross-check
        // the all-zero/nonzero synthetic ROI without trusting window totals.
        const auto roi_begin=depth.raster.begin()+200*640;
        tx.synthetic_roi_pixels=static_cast<std::size_t>(depth.raster.end()-roi_begin);
        tx.synthetic_invalid_pixels=static_cast<std::size_t>(std::count(roi_begin,depth.raster.end(),0));
        require(observation.roi_pixels==tx.synthetic_roi_pixels&&
                observation.invalid_pixels==tx.synthetic_invalid_pixels,"ROI observation agrees with independent synthetic raster count");
        require(processed.observe(observation,last_processed,last_processed)==DepthWindowCode::Accepted,"actual ROI processing completion");
    };
    auto object_step=[&](std::string_view name,bool known) {
        const auto capture=latest_profile.capture.capture_mono_ns/1000000;
        auto body=synthetic_objects(capture,known);
        const auto completed=clock.mono_millis();
        require(test_feed::inference(health,capture,completed,completed)==HealthEventCode::Accepted,"explicit synthetic model-completion event");
        const auto result=o_channel.publish(body);
        auto& tx=sent.at(count++);
        save(tx,name,result,o_tx);
        tx.inference_completed_ms=completed;
        require(test_feed::publish(health,Topic::Objects,result.audit,clock.mono_millis())==HealthEventCode::Accepted,"Objects put health");
    };
    wait_until(clock,heartbeat.next_eligible_ms(),deadline);
    status_beat("status_startup_unknown",true);
    profile_step("profile_normal",false,false);
    object_step("objects_known_geometry",true);
    const auto object_calls=o_tx.calls;
    wait_until(clock,std::max(heartbeat.next_eligible_ms(),last_processed+1001),deadline);
    profile_step("profile_fit_failed",true,true);
    status_beat("status_all_invalid",true);
    require(o_tx.calls==object_calls&&p_tx.calls==2&&s_tx.calls==2,"controlled objects silence does not prevent generator calls");
    wait_until(clock,std::max(heartbeat.next_eligible_ms(),last_processed+1001),deadline);
    profile_step("profile_recovered",false,false);
    object_step("objects_unknown_geometry",false);
    status_beat("status_recovered",false);
    wait_until(clock,std::max(heartbeat.next_eligible_ms(),last_processed+1001),deadline);
    status_beat("status_no_processed_depth",true);
    require(count==kPackets&&p_tx.calls==3&&o_tx.calls==2&&s_tx.calls==4,"exact generated send counts");
    require(depth.consumer.step().status==NativeProfileStatus::Idle&&p_tx.calls==3,"no Profile replay during silence");
    depth.close();
    {
        std::unique_lock<std::mutex> lock(receiver_state->mutex);
        require(receiver_state->changed.wait_for(lock,kWait,[&]{return receiver_state->count>=kPackets||receiver_state->errors;}),"generated receipts timeout");
        require(receiver_state->errors==0&&receiver_state->count==kPackets,"exact initial receive count");
        receiver_state->changed.wait_for(lock,std::chrono::milliseconds(50),[&]{return receiver_state->count!=kPackets||receiver_state->errors;});
    }
    receiver.close();
    {
        std::unique_lock<std::mutex> lock(receiver_state->mutex);
        require(receiver_state->changed.wait_for(lock,kWait,[&]{return receiver_state->closure_dropped;}),"receiver shutdown acknowledgement");
    }
    require(receiver_state->count==kPackets&&receiver_state->callback_count==kPackets&&!receiver_state->errors,"no duplicate/truncated/unexpected receipt");
    std::array<bool,kPackets> matched{};
    std::cout<<std::boolalpha;
    for(std::size_t i=0;i<count;++i) {
        require(sent[i].result.audit.seq==i+1,"shared process sequence allocation");
        std::size_t matches=0,which=0;
        for(std::size_t j=0;j<receiver_state->count;++j)
            if(sent[i].key_view()==receiver_state->packets[j].key_view()&&
               sent[i].payload_view()==receiver_state->packets[j].payload_view()) {++matches;which=j;}
        require(matches==1&&!matched[which],"unique byte-exact TX/RX mapping"); matched[which]=true;
        const auto& rx=receiver_state->packets[which];
        require(rx.callback_entry_ns>=sent[i].native.put_begin_ns,"real callback after put entry");
        require(same_qos(intended_native(sent[i].qos),rx.qos),"native QoS readback");
        print_row(sent[i],rx,endpoint,session.config_json());
    }
    require(Steady::now()<deadline,"overall bounded diagnostic");
    std::cerr<<"Generated r6 TCP PASS: Profile 3 / Objects 2 / Status 4; synthetic input and controlled single owner, not three-owner/camera/model/cadence acceptance.\n";
}
} // namespace
int main() {
    try { run(); return 0; }
    catch(const std::exception& error) {
        std::cerr<<"Generated r6 TCP FAIL: "<<error.what()<<'\n'; return 1;
    }
}
