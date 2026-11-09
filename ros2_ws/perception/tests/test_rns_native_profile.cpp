/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_native_profile.cpp
 * Brief: Unit tests for the native Profile owner: latest-frame pipeline, clocks, failures and S/T owners.
 *
 * Description:
 * What it checks. The Profile owner takes the newest native depth frame, runs the geometry and
 * publishes. Cases cover latest-frame and idle behaviour, clock and source identity, failure
 * recovery and restart, the no-ground-reference route, withdrawal guards, put failures and late
 * invalidation, setup, processed health, the calibration verdict carried into the message, and the
 * semantic (S) owner: a projector built on another geometry, or a snapshot without a source, is
 * refused (19 section 3.4A, service wiring 2e).
 */

/* Synthetic input and clock ONLY. Reusable mailbox consumer, no T/S objects,
 * camera, network, real calibration, exposure mapping or cadence acceptance. */
#include "output/rns_native_profile.hpp"
#include "service/t_mask_slot.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <string_view>

namespace {
std::atomic<std::size_t> allocations{0};
void* alloc(std::size_t n) {
    void* p = std::malloc(n ? n : 1);
    if (!p) throw std::bad_alloc();
    ++allocations; return p;
}
void* aligned(std::size_t n, std::size_t a) {
    void* p = nullptr;
    if (posix_memalign(&p, a, n ? n : 1)) throw std::bad_alloc();
    ++allocations; return p;
}
[[noreturn]] void fail(const char* expr, int line) {
    std::cerr << "native Profile CHECK " << line << ": " << expr << '\n'; std::abort();
}
#define CHECK(x) do { if (!(x)) fail(#x, __LINE__); } while (false)
}
void* operator new(std::size_t n) { return alloc(n); }
void* operator new[](std::size_t n) { return alloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) { return aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::geometry;
using namespace perception::rns;
namespace nd = perception::native_depth;
std::size_t steps = 0;
ProjectionBinding binding() {
    ProjectionBinding b;
    b.intrinsics = {128, 128, 320, -128, 640, 400};
    b.distortion.reported = true; b.distortion.model = 0;
    b.unit_m = 1.0F/1024;
    b.R_base_camera = {0,0,1, -1,0,0, 0,-1,0}; b.t_base_camera = {0,0,1};
    return b;
}
GeometryOptions options() {
    GeometryOptions o;
    o.prior = {{0,0,1},0};
    o.fit.inlier_distance_m=.015; o.fit.min_inlier_fraction=.8;
    o.fit.max_inlier_rms_m=.005; o.fit.min_xy_stddev_m=.01;
    o.fit.min_up_dot=.5; o.fit.max_prior_angle_rad=.3;
    o.fit.min_inliers=100; o.fit.hypotheses=32; o.fit.seed=0x243f6a8885a308d3ULL;
    o.fit_near_min_m=.7; o.fit_near_max_m=2.9; o.fit_prior_band_m=.25;
    o.range_max_m=6; o.blind_near_lo_m=.01; o.h_tol_m=.125;
    o.z_pass_m=.5; o.eps_neg=.1; o.w_h_m=.1;
    o.angle_min_rad=-155.0/128; o.angle_step_rad=1.0/128; o.n_bins=311;
    return o;
}
std::vector<Vec3> rays(const ProjectionBinding& b) {
    std::vector<Vec3> r(kGeometryPixels);
    for (std::size_t y=0; y<kNativeHeight; ++y) for (std::size_t x=0; x<kNativeWidth; ++x)
        r[y*kNativeWidth+x] = {(x-b.intrinsics.cx)/b.intrinsics.fx,
                              (y-b.intrinsics.cy)/b.intrinsics.fy,1};
    return r;
}
// Publish guard of the next Fixture's S owner (rns_semantic.hpp); a test sets it just before it
// constructs the fixture and restores 0.
std::int64_t fixture_publish_guard_ns=0;
struct Clock final : PublishClock {
    mutable std::int64_t ns=10000000000LL;
    mutable std::size_t calls=0;
    std::size_t bad_at=0, stop_at=0;
    nd::Mailbox* mailbox=nullptr;
    CaptureStamp expected;
    int mutation=0;
    bool fail_next_publish_sample=false;
    Millis publish_offset_ms=0;
    // The read with this (1-based, cumulative) index first jumps the clock forward by jump_ns:
    // time passing between two stages of one step.
    std::size_t jump_at=0;
    std::int64_t jump_ns=0;
    Millis wall_millis() noexcept override { return 1700000000123; }
    Millis mono_millis() noexcept override {
        if (fail_next_publish_sample) { fail_next_publish_sample=false; return -1; }
        return ns/1000000+publish_offset_ms;
    }
    static std::int64_t now(const void* p) noexcept {
        const auto& c=*static_cast<const Clock*>(p);
        ++c.calls;
        if (c.stop_at==c.calls) c.mailbox->end_stream();
        if (c.bad_at==c.calls) return -1;
        if (c.jump_at==c.calls) c.ns+=c.jump_ns;
        c.ns+=100000; return c.ns;
    }
    static bool resolve(const void* p, const nd::Metadata& m, CaptureStamp& out) noexcept {
        const auto& c=*static_cast<const Clock*>(p);
        CHECK(m.source.sequence==c.expected.source.sequence);
        out=c.expected; // a frozen synthetic fixture, NOT a camera clock formula
        switch (c.mutation) {
        case 1: return false;
        case 2: ++out.source.depth_index; break;
        case 3: ++out.clock_epoch; break;
        case 4: out.capture_mono_ns=m.source.host_ready_mono_ns+1; break;
        case 5: out.capture_mono_ns=-1; break;
        case 6: out.capture_mono_ns=0; break;
        case 7: ++out.source.host_ready_mono_ns; break;
        }
        return true;
    }
    NativeProfileClock config() { return {this,42,now,resolve}; }
};
struct Sink final : RnsTransport {
    std::array<char,262144> packet{};
    std::size_t calls=0, size=0;
    PutOutcome outcome=PutOutcome::Accepted;
    Clock* fail_return_clock=nullptr;
    SourceJournal* required_health=nullptr;
    Clock* delay_clock=nullptr;
    PutReceipt put(std::string_view key, std::string_view bytes, PublishQos q) override {
        CHECK(key=="xbrain/robot_1/rt/perception/profile");
        CHECK(q.priority==Priority::RealTime && q.drop && q.express && !q.reliable);
        CHECK(bytes.size()<packet.size());
        if (required_health) {
            SourceEvent e;
            CHECK(required_health->peek(e));
            CHECK(e.kind==SourceKind::DepthProcessed && e.depth.code==DepthHealthCode::Ready);
            CHECK(e.observed_ns<=delay_clock->ns);
            delay_clock->ns+=2000000000LL; // injected slow put, NOT a real SDK delay
        }
        ++calls; size=bytes.size(); std::memcpy(packet.data(),bytes.data(),size); packet[size]=0;
        if (fail_return_clock) fail_return_clock->fail_next_publish_sample=true;
        return {outcome,0};
    }
};
struct Fixture {
    ProjectionBinding b=binding();
    std::vector<Vec3> optical=rays(b);
    std::vector<std::uint16_t> raster=std::vector<std::uint16_t>(kGeometryPixels);
    nd::Mailbox mailbox{kGeometryPixels};
    NativeGeometry geometry;
    NormalGrid grid{geometry,{.25,.9,.4,.02}};
    Clock clock;
    Sink sink;
    ProcessEnvelope envelope{"robot_1","abcdef01",5000};
    PublisherChannel channel{Topic::Profile,envelope,clock,sink,131072,262144};
    SourceJournal depth_health;
    // Optional S owner (19 section 3.4A). semantic_slot is the latest-wins slot
    // the service fills from the inference thread; here the test fills it.
    SemanticOptions semantic_options{0,0,0,50000000,77,fixture_publish_guard_ns};
    SemanticProjector semantic{geometry,semantic_options,SemanticDepthPolicy::NormalOrFitFailed};
    const SemanticFrame* semantic_slot=nullptr;
    std::size_t semantic_fetches=0;
    static const SemanticFrame* latest_semantic(void* ctx, const CaptureStamp&) noexcept {
        auto* self=static_cast<Fixture*>(ctx); ++self->semantic_fetches; return self->semantic_slot;
    }
    // Optional T owner (19 section 3.3, plan 2f) with the rectified reference
    // projection of test_rns_normal_profile; t_slot plays the service's T slot.
    RectifiedColor color=[]{ RectifiedColor c; c.R_color_base={0,-1,0,0,0,-1,1,0,0};
                             c.t_color_base={.125,1.25,.25}; c.fx=200; c.fy=160; c.cx=640; c.cy=100; return c; }();
    ColorProjection projection{1280,800,77,&color,project_rectified_color};
    std::vector<std::uint8_t> mask_pixels=std::vector<std::uint8_t>(1280U*800U,1);
    NormalTraversability t;
    RawTMask t_mask;
    const RawTMask* t_slot=nullptr;
    std::size_t t_fetches=0;
    // Optional real service slot (plan 2f P19-2 handoff) instead of the fixed pointer.
    perception::service::TMaskSlot* real_slot=nullptr;
    std::unique_ptr<perception::service::TMaskSlot::Snapshot> slot_snapshot;
    static const RawTMask* latest_t(void* ctx, const CaptureStamp& depth) noexcept {
        auto* self=static_cast<Fixture*>(ctx); ++self->t_fetches;
        if (self->real_slot) return self->real_slot->latest(*self->slot_snapshot,depth,self->projection.revision);
        return self->t_slot;
    }
    // The mask of the frame set just offered (same identity and capture: age 0).
    void publish_mask_of_current_frame() {
        t_mask={clock.expected,MaskMaterial::RawTraversable,projection.revision,1280,800,1280,
                mask_pixels.size(),mask_pixels.data()};
        t_slot=&t_mask;
    }
    NativeProfileConsumer consumer;
    std::uint64_t stream=0, seq=0;
    static std::int64_t health_now(void* p) noexcept { return Clock::now(p); }
    HealthRoiView roi_view() const {
        HealthRoiView v; v.kind=HealthRoiKind::CalibratedStaticMask;
        v.mask=geometry.prior_roi().data(); v.mask_pixels=geometry.prior_roi().size();
        v.width=640; v.height=400; return v;
    }
    // The existing G / G+S constructor unless with_t: then the T constructor (plan 2f).
    static NativeProfileConsumer make_consumer(Fixture& f, bool health, bool calibrated, bool with_semantic,
                                               bool with_t) {
        if (with_t)
            return NativeProfileConsumer(f.mailbox,f.geometry,f.grid,f.clock.config(),f.channel,&f.t,
                                         TFrameSource{latest_t,&f},with_semantic ? &f.semantic : nullptr,
                                         SemanticFrameSource{latest_semantic,&f},health ? &f.depth_health : nullptr,
                                         calibrated ? f.roi_view() : HealthRoiView{},calibrated);
        return NativeProfileConsumer(f.mailbox,f.geometry,f.grid,f.clock.config(),f.channel,
                                     with_semantic ? &f.semantic : nullptr,SemanticFrameSource{latest_semantic,&f},
                                     health ? &f.depth_health : nullptr,calibrated ? f.roi_view() : HealthRoiView{},
                                     calibrated);
    }
    explicit Fixture(GeometryOptions go=options(), bool health=false, std::size_t capacity=4, bool calibrated=false,
                     bool with_semantic=false, bool with_t=false, TOptions t_options={.8,0,0,0,50000000})
        : geometry(b,optical.data(),optical.size(),go),
          depth_health(SourceKind::DepthProcessed,capacity,{&clock,health_now}),
          t(geometry,grid,projection,t_options),
          consumer(make_consumer(*this,health,calibrated,with_semantic,with_t)) {
        stream=mailbox.begin_stream(); clock.mailbox=&mailbox; plane();
    }
    void plane() {
        for (std::size_t i=0;i<raster.size();++i)
            raster[i]=static_cast<std::uint16_t>(std::llround(1.0/optical[i].y/b.unit_m));
    }
    void exact_ground_support() {
        std::fill(raster.begin(),raster.end(),0);
        for (std::size_t x=0;x<640;++x) {
            raster[x]=1024; raster[128*640+x]=512; raster[384*640+x]=256;
        }
    }
    nd::Offer offer(bool mismatch=false) {
        clock.ns+=10000000;
        nd::View v;
        v.metadata.intrinsics=b.intrinsics; v.metadata.distortion=b.distortion;
        v.metadata.unit_m=mismatch ? b.unit_m*2 : b.unit_m;
        auto& s=v.metadata.source;
        s.stream_id=stream; s.sequence=++seq;
        s.color_index=seq*3; s.depth_index=seq*5+1;
        s.color_device_us=seq*1000; s.depth_device_us=seq*1000+1;
        s.host_ready_mono_ns=clock.ns-1000000;
        clock.expected={s,42,clock.ns-2000000};
        v.pre_d2c=true; v.pixels=raster.data(); v.bytes=raster.size()*sizeof(raster[0]);
        return mailbox.try_offer(v);
    }
    NativeProfileTick step() {
        const auto before=allocations.load();
        auto r=consumer.step(); ++steps;
        CHECK(allocations.load()==before);
        return r;
    }
    void restart() { mailbox.end_stream(); stream=mailbox.begin_stream(); seq=0; }
};
void success(const NativeProfileTick& r) {
    CHECK(r.status==NativeProfileStatus::PutFinished && r.take==nd::Take::Taken);
    CHECK(r.geometry_ran && r.grid_ran && r.build_ran && r.put.dispatched && r.put.publish);
    CHECK(r.capture.capture_mono_ns<=r.source.host_ready_mono_ns);
    CHECK(r.source.host_ready_mono_ns<=r.begin_ns && r.begin_ns<=r.taken_ns);
    CHECK(r.taken_ns<=r.capture_checked_ns && r.capture_checked_ns<=r.geometry_end_ns);
    CHECK(r.geometry_end_ns<=r.grid_end_ns && r.grid_end_ns<=r.build_end_ns && r.build_end_ns<=r.end_ns);
    CHECK(r.put.publish.audit.capture_ms==r.capture.capture_mono_ns/1000000);
    CHECK(r.put.publish.audit.put_before_ms>=r.build_end_ns/1000000);
    CHECK(r.publish_clock_bracket_valid && r.put.publish.audit.put_return_ms<=r.end_ns/1000000);
}
void withdrawn_success(const NativeProfileTick& r) {
    CHECK(r.status==NativeProfileStatus::PutFinished && r.put.dispatched && r.put.publish);
    CHECK(r.route==NativeProfileRoute::FitFailedWithdrawal && r.free_withdrawn);
    CHECK(r.geometry_status==GeometryStatus::FitFailed && !r.grid_ran && r.grid_end_ns==-1);
    CHECK(r.withdrawn_status==WithdrawnProfileStatus::Ready && r.normal_status==NormalProfileStatus::NotBuilt);
    CHECK(r.reasons[0]=="ground_fit_fallback" && r.reasons[1]=="ground_free_withdrawn");
    CHECK(r.geometry_end_ns<=r.build_end_ns && r.build_end_ns<=r.end_ns);
    CHECK(r.publish_clock_bracket_valid);
}
void emit_native(std::string_view name, const Fixture& f, const NativeProfileTick& r) {
    std::cout << "{\"case\":\"" << name << "\",\"packet\":"
        << std::string_view(f.sink.packet.data(),f.sink.size)
        << ",\"source_sequence\":" << r.source.sequence
        << ",\"capture_ns\":" << r.capture.capture_mono_ns
        << ",\"host_ready_ns\":" << r.source.host_ready_mono_ns
        << ",\"before_ms\":" << r.put.publish.audit.put_before_ms
        << ",\"free_withdrawn\":" << (r.free_withdrawn ? "true" : "false")
        << ",\"grid_ran\":" << (r.grid_ran ? "true" : "false")
        << ",\"reasons\":[";
    if (!r.reasons[0].empty()) std::cout << '"' << r.reasons[0] << "\",\"" << r.reasons[1] << '"';
    std::cout << "],\"camera\":false,\"network\":false}\n";
}
// With no frame the owner is idle; of several offered frames it publishes only the newest, once,
// stamped with the capture time (not host-ready), then idles until the next frame.
void test_latest_and_idle(bool emit) {
    Fixture f;
    CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==0);
    CHECK(f.offer()==nd::Offer::Accepted);
    CHECK(f.offer()==nd::Offer::Replaced);
    auto r=f.step(); success(r);
    CHECK(r.source.sequence==2 && f.mailbox.counters().replaced==1);
    CHECK(r.put.publish.audit.capture_ms!=r.source.host_ready_mono_ns/1000000);
    const std::string_view packet(f.sink.packet.data(),f.sink.size);
    CHECK(packet.find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
    CHECK(packet.find("\"extrinsic_calibrated\":false")!=std::string_view::npos);
    if (emit) std::cout << "{\"case\":\"native_g_only\",\"packet\":" << packet
        << ",\"source_sequence\":" << r.source.sequence
        << ",\"capture_ns\":" << r.capture.capture_mono_ns
        << ",\"host_ready_ns\":" << r.source.host_ready_mono_ns
        << ",\"before_ms\":" << r.put.publish.audit.put_before_ms
        << ",\"replaced\":1,\"camera\":false,\"network\":false}\n";
    r=f.step(); CHECK(r.status==NativeProfileStatus::Idle && !r.put.dispatched && r.source.stream_id==0);
    CHECK(f.sink.calls==1 && f.consumer.counts().taken==1);
    CHECK(f.offer()==nd::Offer::Accepted); success(f.step());
    CHECK(f.consumer.counts().accepted==2);
}
// Any bad clock read rejects the frame without sending old bytes or losing a queued frame; a
// publish clock from another domain shows the put but fails the host bracket.
void test_clock_and_identity() {
    Fixture f;
    CHECK(f.offer()==nd::Offer::Accepted); success(f.step());
    for (int m=1;m<=7;++m) {
        f.clock.mutation=m; CHECK(f.offer()==nd::Offer::Accepted);
        auto r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected);
        CHECK(!r.geometry_ran && !r.put.dispatched && f.sink.calls==1);
    }
    f.clock.mutation=0;
    CHECK(f.offer()==nd::Offer::Accepted); success(f.step());
    CHECK(f.consumer.counts().clock_rejected==7);
    // A bad read before take does not destroy the queued frame.
    CHECK(f.offer()==nd::Offer::Accepted);
    f.clock.bad_at=f.clock.calls+1;
    auto r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected && !r.take_called);
    f.clock.bad_at=0; success(f.step());
    // A bad read after geometry never emits a previous body's bytes.
    CHECK(f.offer()==nd::Offer::Accepted); const auto puts=f.sink.calls;
    f.clock.bad_at=f.clock.calls+4;
    r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected && r.geometry_ran && !r.grid_ran);
    CHECK(f.sink.calls==puts); f.clock.bad_at=0;
    CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==puts);
    CHECK(f.offer()==nd::Offer::Accepted); success(f.step());
    // Numerically valid but different publish-clock domain: accepted receipt
    // stays visible; the cross-clock host bracket must fail, never "success".
    CHECK(f.offer()==nd::Offer::Accepted); f.clock.publish_offset_ms=1000;
    r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected && r.put.dispatched);
    CHECK(r.put.publish && !r.publish_clock_bracket_valid);
    CHECK(f.consumer.counts().accepted==f.channel.counters().accepted);
}
// A failed fit (empty or sparse frame) still publishes a withdrawn Profile; a profile mismatch
// publishes nothing; normal frames and a restarted stream recover; an ended stream is
// SourceUnavailable.
void test_failure_recovery_and_restart(bool emit=false) {
    Fixture f;
    CHECK(f.offer()==nd::Offer::Accepted); auto r=f.step(); success(r);
    CHECK(r.route==NativeProfileRoute::Normal && !r.free_withdrawn);
    if (emit) emit_native("normal_start",f,r);
    std::fill(f.raster.begin(),f.raster.end(),0);
    CHECK(f.offer()==nd::Offer::Accepted);
    r=f.step(); withdrawn_success(r);
    CHECK(f.geometry.result().has_raw_sample_evidence() && f.sink.calls==2);
    CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find("\"blind_near_m\":null")!=std::string_view::npos);
    if (emit) emit_native("fit_failed_empty",f,r);
    f.raster[64*640+320]=1024; f.raster[64*640+321]=1024;
    CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); withdrawn_success(r);
    CHECK(f.geometry.result().valid_depth_pixels==2);
    if (emit) emit_native("fit_failed_raw",f,r);
    f.plane(); CHECK(f.offer(true)==nd::Offer::Accepted);
    r=f.step(); CHECK(r.geometry_status==GeometryStatus::ProfileMismatch && !r.put.dispatched);
    CHECK(!f.geometry.result().has_raw_sample_evidence());
    CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); success(r);
    CHECK(r.route==NativeProfileRoute::Normal && !r.free_withdrawn && r.reasons[0].empty());
    if (emit) emit_native("normal_recovered",f,r);
    f.restart(); CHECK(f.offer()==nd::Offer::Accepted);
    r=f.step(); success(r); CHECK(r.source.stream_id==2 && r.source.sequence==1);
    f.mailbox.end_stream(); CHECK(f.step().status==NativeProfileStatus::SourceUnavailable);
    CHECK(f.sink.calls==5);
}
// A frame with no ground in range takes the no-ground route (FREE withdrawn, measured block kept),
// also when the ground lies exactly at range_max; normal ground returns to the normal route.
void test_no_ground_reference(bool emit=false) {
    auto go=options(); go.angle_min_rad=0; go.n_bins=1;
    Fixture f(go);
    f.exact_ground_support();
    for (std::size_t y=0;y<400;++y) f.raster[y*640+320]=0;
    f.raster[128*640+320]=384;
    CHECK(f.offer()==nd::Offer::Accepted); auto r=f.step(); success(r);
    CHECK(r.route==NativeProfileRoute::NoGroundReference && r.free_withdrawn);
    CHECK(r.geometry_status==GeometryStatus::NormalEvidence && !f.grid.result().blind_m);
    CHECK(r.reasons[0].empty() && r.withdrawn_status==WithdrawnProfileStatus::NotBuilt);
    CHECK(f.grid.bins()[0].traversal.block_m==.375);
    if (emit) emit_native("no_ground_reference",f,r);
    std::fill(f.raster.begin(),f.raster.end(),0);
    CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); withdrawn_success(r);
    f.plane(); CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); success(r);
    CHECK(r.route==NativeProfileRoute::Normal && !r.free_withdrawn && r.reasons[0].empty());
    CHECK(f.sink.calls==3); // one current frame in each of the three routes
    // A real blind bound at range is not the absence of a measurement.
    go.range_max_m=1; go.fit_near_min_m=.3; go.fit_near_max_m=1;
    Fixture edge(go); edge.exact_ground_support();
    for (std::size_t y=0;y<400;++y) edge.raster[y*640+320]=0;
    edge.raster[320]=1024;
    CHECK(edge.offer()==nd::Offer::Accepted); r=edge.step(); success(r);
    CHECK(r.route==NativeProfileRoute::NoGroundReference && edge.grid.result().blind_m==1);
    if (emit) emit_native("no_ground_range_edge",edge,r);
}
// A withdrawn Profile with the same capture ms as the last put is a Duplicate and is not sent, and
// it needs a live source; rejected or unknown puts are not retried; the geometry-only evidence
// builder refuses a foreign mailbox and invents no S.
void test_withdrawal_guards() {
    Fixture f; CHECK(f.offer()==nd::Offer::Accepted); const auto first=f.step(); success(first);
    std::fill(f.raster.begin(),f.raster.end(),0);
    CHECK(f.offer()==nd::Offer::Accepted);
    f.clock.expected.capture_mono_ns=first.capture.capture_mono_ns+1;
    auto r=f.step();
    CHECK(r.route==NativeProfileRoute::FitFailedWithdrawal && r.put.dispatched);
    CHECK(r.put.publish.error==PublishError::Duplicate && !r.put.publish.audit.attempted && f.sink.calls==1);
    CHECK(f.step().status==NativeProfileStatus::Idle);
    // FitFailed still requires a live source immediately before dispatch.
    CHECK(f.offer()==nd::Offer::Accepted); f.clock.stop_at=f.clock.calls+5;
    r=f.step(); CHECK(r.build_ran && !r.put.dispatched && f.sink.calls==1);
    f.clock.stop_at=0; f.restart();
    // Unknown/rejected actual puts are consumed, not retried from old bins.
    for (auto outcome : {PutOutcome::Rejected,PutOutcome::Unknown}) {
        f.sink.outcome=outcome; CHECK(f.offer()==nd::Offer::Accepted); r=f.step();
        CHECK(r.put.dispatched && r.put.publish.audit.attempted && !r.put.publish);
        const auto calls=f.sink.calls;
        CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==calls);
    }
    f.sink.outcome=PutOutcome::Accepted; CHECK(f.offer()==nd::Offer::Accepted);
    f.clock.bad_at=f.clock.calls+6;
    r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected && r.put.publish.audit.attempted);
    CHECK(r.put.publish.audit.receipt.outcome==PutOutcome::Accepted);
    const auto calls=f.sink.calls; f.clock.bad_at=0;
    CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==calls);
    // Explicit G-only overload neither accepts the wrong owner nor invents S.
    CHECK(f.offer()==nd::Offer::Accepted); withdrawn_success(f.step());
    WithdrawnSemanticEvidence evidence(f.geometry);
    nd::Mailbox wrong(kGeometryPixels); wrong.begin_stream();
    CHECK(evidence.build(wrong,f.clock.expected,f.clock.ns).status==WithdrawnStatus::InvalidSource);
    CHECK(evidence.build(f.mailbox,f.clock.expected,f.clock.ns).status==WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(evidence.report().semantic_status==SemanticStatus::Missing);
    CHECK(evidence.build(f.mailbox,f.clock.ns).status==WithdrawnStatus::InvalidSource);
}
// A rejected put is consumed, not retried; a stop after the build sends nothing; a clock failure
// after the put keeps the receipt visible and never resends.
void test_put_and_late_invalidation() {
    Fixture f;
    f.sink.outcome=PutOutcome::Rejected;
    CHECK(f.offer()==nd::Offer::Accepted);
    auto r=f.step(); CHECK(r.put.dispatched && r.put.publish.error==PublishError::PutRejected);
    CHECK(f.consumer.counts().dispatches==1 && f.consumer.counts().accepted==0);
    CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==1);
    f.sink.outcome=PutOutcome::Accepted;
    CHECK(f.offer()==nd::Offer::Accepted); success(f.step());
    // Deliberately invalid lifecycle: owner guard catches stop after build.
    CHECK(f.offer()==nd::Offer::Accepted); f.clock.stop_at=f.clock.calls+6;
    r=f.step(); CHECK(r.build_ran && !r.put.dispatched && f.sink.calls==2);
    f.clock.stop_at=0; f.restart();
    CHECK(f.offer()==nd::Offer::Accepted);
    f.clock.bad_at=f.clock.calls+7; // after put; do not hide or retry receipt
    r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected && r.put.dispatched && r.put.publish);
    CHECK(f.sink.calls==3); f.clock.bad_at=0;
    CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==3);
    CHECK(f.offer()==nd::Offer::Accepted);
    f.sink.fail_return_clock=&f.clock;
    r=f.step(); CHECK(r.status==NativeProfileStatus::ClockRejected && r.put.dispatched);
    CHECK(r.put.publish.error==PublishError::ClockAfter && !r.put.publish);
    CHECK(r.put.publish.audit.receipt.outcome==PutOutcome::Accepted);
    CHECK(f.consumer.counts().accepted==f.channel.counters().accepted);
    CHECK(f.consumer.counts().accepted==3 && f.sink.calls==4);
    f.sink.fail_return_clock=nullptr;
    CHECK(f.step().status==NativeProfileStatus::Idle && f.sink.calls==4);
}
// The constructor refuses a clock config missing its context, epoch, now or capture function.
void test_setup() {
    Fixture f;
    for (int m=0;m<4;++m) {
        auto c=f.clock.config();
        if (m==0) c.context=nullptr;
        if (m==1) c.epoch=0;
        if (m==2) c.now_ns=nullptr;
        if (m==3) c.capture=nullptr;
        bool rejected=false;
        try { NativeProfileConsumer bad(f.mailbox,f.geometry,f.grid,c,f.channel); }
        catch (const std::invalid_argument&) { rejected=true; }
        CHECK(rejected);
    }
}
// Depth health is recorded once per processed frame (the newest, not every arrival) before the
// put, failed fits included; a full health queue is visible but never blocks the Profile; with
// health off nothing is recorded.
void test_processed_health() {
    Fixture f(options(),true);
    CHECK(f.step().status==NativeProfileStatus::Idle && f.depth_health.published()==0);
    CHECK(f.offer()==nd::Offer::Accepted);
    CHECK(f.offer()==nd::Offer::Replaced);
    f.sink.required_health=&f.depth_health; f.sink.delay_clock=&f.clock;
    const auto r=f.step(); success(r);
    CHECK(r.depth_health_ran && r.depth_health_recorded);
    CHECK(f.depth_health.published()==1); // processed latest only, NOT two arrivals
    SourceEvent e; CHECK(f.depth_health.pop(e));
    CHECK(e.source.sequence==2 && e.source.depth_index==r.source.depth_index);
    CHECK(e.observed_ns>=r.taken_ns && e.observed_ns<=r.capture_checked_ns);
    CHECK(r.end_ns-e.observed_ns>=2000000000LL); // event preceded put
    CHECK(e.depth.roi_pixels==640*200 && e.depth.invalid_pixels==0);
    DepthHealthWindow window(0,4);
    const auto ms=e.observed_ns/1000000;
    CHECK(window.observe(e.depth,ms,ms)==DepthWindowCode::Accepted);
    CHECK(window.snapshot(ms).invalid_ratio==0);
    CHECK(!window.snapshot(ms+1000).invalid_ratio); // expire, don't reuse last quality
    f.sink.required_health=nullptr;
    CHECK(!f.step().depth_health_ran && f.depth_health.published()==1);
    std::fill(f.raster.begin(),f.raster.end(),0);
    CHECK(f.offer()==nd::Offer::Accepted);
    const auto failed_fit=f.step(); withdrawn_success(failed_fit);
    CHECK(failed_fit.depth_health_recorded && f.depth_health.pop(e));
    CHECK(e.depth.invalid_pixels==e.depth.roi_pixels && e.depth.invalid_ratio==1);
    CHECK(window.observe(e.depth,e.observed_ns/1000000,e.observed_ns/1000000)==DepthWindowCode::Accepted);
    CHECK(window.snapshot(e.observed_ns/1000000).invalid_ratio==1);
    // Telemetry failure is visible, but never silently blocks the separate
    // Profile path or substitutes an empty/healthy depth observation.
    Fixture full(options(),true,1);
    CHECK(full.offer()==nd::Offer::Accepted); success(full.step());
    CHECK(full.offer()==nd::Offer::Accepted); const auto over=full.step(); success(over);
    CHECK(over.depth_health_ran && !over.depth_health_recorded);
    CHECK(!full.depth_health.valid() && full.depth_health.overflow_count()==1);
    CHECK(full.sink.calls==2 && full.depth_health.published()==1);
    Fixture disabled;
    CHECK(disabled.offer()==nd::Offer::Accepted); const auto off=disabled.step(); success(off);
    CHECK(!off.depth_health_ran && !off.depth_health_recorded && disabled.depth_health.published()==0);
}
}
// Wire flag comes only from the caller's calibration verdict and reaches both
// the normal and the withdrawn body; the health ROI kind must agree with it.
void test_calibration_verdict() {
    Fixture f(options(),true,8,true);
    std::size_t roi=0; for (auto m : f.geometry.prior_roi()) roi+=m;
    CHECK(f.geometry.roi_pixels()>0 && roi==f.geometry.roi_pixels() && f.geometry.prior_roi().size()==kGeometryPixels);
    CHECK(f.offer()==nd::Offer::Accepted); auto r=f.step(); success(r);
    CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find("\"extrinsic_calibrated\":true")!=std::string_view::npos);
    std::fill(f.raster.begin(),f.raster.end(),0);
    CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); withdrawn_success(r);
    CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find("\"extrinsic_calibrated\":true")!=std::string_view::npos);
    CHECK(f.depth_health.published()==2);
    SourceEvent e; CHECK(f.depth_health.peek(e) && e.depth.roi_kind==HealthRoiKind::CalibratedStaticMask &&
                         e.depth.roi_pixels==f.geometry.roi_pixels());
    bool lower_half_but_calibrated=false, mask_but_uncalibrated=false;
    try { NativeProfileConsumer c(f.mailbox,f.geometry,f.grid,f.clock.config(),f.channel,&f.depth_health,HealthRoiView{},true); }
    catch (const std::invalid_argument&) { lower_half_but_calibrated=true; }
    try { NativeProfileConsumer c(f.mailbox,f.geometry,f.grid,f.clock.config(),f.channel,&f.depth_health,f.roi_view(),false); }
    catch (const std::invalid_argument&) { mask_but_uncalibrated=true; }
    CHECK(lower_half_but_calibrated && mask_but_uncalibrated);
    Fixture plain; CHECK(plain.offer()==nd::Offer::Accepted); success(plain.step());
    CHECK(std::string_view(plain.sink.packet.data(),plain.sink.size).find("\"extrinsic_calibrated\":false")!=std::string_view::npos);
}
// ---- S owner inside the consumer (19 section 3.4A; service wiring 2e) ------
void test_semantic_owner() {
    // wrong geometry owner, and a missing snapshot source, are both refused
    {
        Fixture other; Fixture here;
        bool foreign=false, no_source=false;
        try { NativeProfileConsumer c(here.mailbox,here.geometry,here.grid,here.clock.config(),here.channel,
                                      &other.semantic,SemanticFrameSource{Fixture::latest_semantic,&here},
                                      nullptr,HealthRoiView{},false); }
        catch (const std::invalid_argument&) { foreign=true; }
        try { NativeProfileConsumer c(here.mailbox,here.geometry,here.grid,here.clock.config(),here.channel,
                                      &here.semantic,SemanticFrameSource{},nullptr,HealthRoiView{},false); }
        catch (const std::invalid_argument&) { no_source=true; }
        CHECK(foreign && no_source);
    }
    Fixture f(options(),false,4,false,true);
    // 1. no snapshot published yet: the owner still runs, the frame stays G-only
    CHECK(f.offer()==nd::Offer::Accepted);
    auto r=f.step(); success(r);
    CHECK(f.semantic_fetches==1 && r.semantic_status==SemanticStatus::Missing && r.semantic_bins==0);
    CHECK(r.semantic_end_ns>=r.grid_end_ns && r.semantic_end_ns<=r.build_end_ns);
    // 2. a measured footprint in front of the camera blocks its bins
    SemanticObject object;
    object.track_id=5; object.class_id=3; object.class_name="chair";
    object.vertices=4;
    object.footprint[0]={.60F,-.10F}; object.footprint[1]={.70F,-.10F};
    object.footprint[2]={.70F,.10F}; object.footprint[3]={.60F,.10F};
    CHECK(f.offer()==nd::Offer::Accepted);
    SemanticFrame frame{f.clock.expected,77,SemanticMaterial::MeasuredBaseFootprint,&object,1};
    f.semantic_slot=&frame;
    r=f.step(); success(r);
    CHECK(r.semantic_status==SemanticStatus::Fresh && r.semantic_bins>0);
    const std::string_view packet(f.sink.packet.data(),f.sink.size);
    CHECK(packet.find("\"t_seg_mono_ms\":null")!=std::string_view::npos);  // S never invents T
    // 3. the same snapshot serves a later depth frame until it expires
    CHECK(f.offer()==nd::Offer::Accepted);
    r=f.step(); success(r);
    CHECK(r.semantic_status==SemanticStatus::Fresh && r.semantic_bins>0);
    for (int i=0;i<6;++i) { CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); }
    success(r);
    CHECK(r.semantic_status==SemanticStatus::Stale && r.semantic_bins==0);  // expired: geometry only
    // 4. A NoGroundReference frame is PUBLISHED with an S owner configured
    // (19 A19-NULL-1, reply batch 6 B6-2): FREE/T/slope withdrawn, blind null,
    // the measured obstacle kept, and an independent S boundary still allowed.
    auto go=options(); go.angle_min_rad=0; go.n_bins=1;
    Fixture ng(go,false,4,false,true);
    ng.exact_ground_support();
    for (std::size_t y=0;y<400;++y) ng.raster[y*640+320]=0;
    ng.raster[128*640+320]=384;
    CHECK(ng.offer()==nd::Offer::Accepted);
    auto n=ng.step(); success(n);
    CHECK(n.route==NativeProfileRoute::NoGroundReference && n.free_withdrawn && ng.sink.calls==1);
    CHECK(n.semantic_status==SemanticStatus::Missing && n.semantic_bins==0);
    {
        const std::string_view packet(ng.sink.packet.data(),ng.sink.size);
        CHECK(packet.find("\"blind_near_m\":null")!=std::string_view::npos);
        CHECK(packet.find("\"d_free\":[null]")!=std::string_view::npos);
        CHECK(packet.find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
        CHECK(packet.find("\"src\":[2]")!=std::string_view::npos);      // geometry only
    }
    // the same scene with a fresh S nearer than the measured obstacle
    SemanticObject door;
    door.track_id=9; door.class_id=1; door.class_name="door"; door.vertices=4;
    door.footprint[0]={.20F,-.05F}; door.footprint[1]={.30F,-.05F};
    door.footprint[2]={.30F,.05F}; door.footprint[3]={.20F,.05F};
    CHECK(ng.offer()==nd::Offer::Accepted);
    SemanticFrame door_frame{ng.clock.expected,77,SemanticMaterial::MeasuredBaseFootprint,&door,1};
    ng.semantic_slot=&door_frame;
    n=ng.step(); success(n);
    CHECK(n.route==NativeProfileRoute::NoGroundReference && n.semantic_status==SemanticStatus::Fresh);
    CHECK(n.semantic_bins==1 && ng.sink.calls==2);
    {
        const std::string_view packet(ng.sink.packet.data(),ng.sink.size);
        CHECK(packet.find("\"src\":[6]")!=std::string_view::npos);      // geometry + semantic
        CHECK(packet.find("\"h_block\":[null]")!=std::string_view::npos);
        CHECK(packet.find("\"d_free\":[null]")!=std::string_view::npos);
    }
    // 5. FitFailed with an S owner (19 A19-FIT-3): the withdrawn packet still
    // publishes, and an INDEPENDENT S boundary survives the failed fit.
    {
        Fixture ff(options(),false,4,false,true);
        std::fill(ff.raster.begin(),ff.raster.end(),0);
        CHECK(ff.offer()==nd::Offer::Accepted);
        auto w=ff.step(); withdrawn_success(w);          // S missing: as before
        CHECK(w.semantic_status==SemanticStatus::Missing && w.semantic_bins==0 && ff.sink.calls==1);
        SemanticObject glass;
        glass.track_id=4; glass.class_id=2; glass.class_name="door"; glass.vertices=4;
        // independent of any ground-plane estimate: this is what survives a
        // failed fit. The paired negative (plane-dependent) follows below.
        glass.plane_basis=SemanticPlaneBasis::IndependentMeasurement;
        glass.footprint[0]={.60F,-.10F}; glass.footprint[1]={.70F,-.10F};
        glass.footprint[2]={.70F,.10F}; glass.footprint[3]={.60F,.10F};
        CHECK(ff.offer()==nd::Offer::Accepted);
        SemanticFrame glass_frame{ff.clock.expected,77,SemanticMaterial::MeasuredBaseFootprint,&glass,1};
        ff.semantic_slot=&glass_frame;
        w=ff.step(); withdrawn_success(w);
        CHECK(w.semantic_status==SemanticStatus::Fresh && w.semantic_bins>0 && ff.sink.calls==2);
        const std::string_view packet(ff.sink.packet.data(),ff.sink.size);
        CHECK(packet.find("\"blind_near_m\":null")!=std::string_view::npos);
        CHECK(packet.find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
        CHECK(packet.find("\"d_free\":[null")!=std::string_view::npos);
        // paired negative: a footprint that DEPENDS on the failed ground plane
        // is refused, and the frame publishes without it
        SemanticObject derived=glass;
        derived.track_id=5;
        derived.plane_basis=SemanticPlaneBasis::GroundPlaneDependent;
        CHECK(ff.offer()==nd::Offer::Accepted);
        SemanticFrame derived_frame{ff.clock.expected,77,SemanticMaterial::MeasuredBaseFootprint,&derived,1};
        ff.semantic_slot=&derived_frame;
        w=ff.step(); withdrawn_success(w);
        CHECK(w.semantic_status==SemanticStatus::Fresh && w.semantic_bins==0 && ff.sink.calls==3);
        CHECK(ff.semantic.report().excluded_plane_basis_objects==1);
    }
    // 6. recovery: normal -> no ground -> normal leaves no residue
    ng.plane();
    ng.semantic_slot=nullptr;
    CHECK(ng.offer()==nd::Offer::Accepted);
    n=ng.step(); success(n);
    CHECK(n.route==NativeProfileRoute::Normal && !n.free_withdrawn && n.semantic_bins==0);
    {
        const std::string_view packet(ng.sink.packet.data(),ng.sink.size);
        CHECK(packet.find("\"blind_near_m\":null")==std::string_view::npos);   // a measured blind is back
        CHECK(packet.find("\"d_free\":[null]")==std::string_view::npos);       // FREE is back
        CHECK(packet.find("\"src\":[6]")==std::string_view::npos);             // no stale S bit
    }
    // the same frame WITHOUT an S owner publishes exactly as before
    Fixture ok(go);
    ok.exact_ground_support();
    for (std::size_t y=0;y<400;++y) ok.raster[y*640+320]=0;
    ok.raster[128*640+320]=384;
    CHECK(ok.offer()==nd::Offer::Accepted);
    auto o2=ok.step(); success(o2);
    CHECK(o2.route==NativeProfileRoute::NoGroundReference && ok.sink.calls==1);
}
// Plan 2f: the T owner runs for every frame on the normal and no-ground routes,
// with the slot's mask or none; fresh masks back FREE only when the margin allows
// it (zero margin = test branch), a positive margin collapses the backing (the
// B3-1 HOLD, reported as Fresh with near-side support missing, never as Missing),
// old masks are Stale, an unbinarised 0/255 mask stops the frame (fail-closed),
// and the fit-failed withdrawal does not consult T.
void test_t_owner() {
    {
        Fixture other; Fixture here;
        bool foreign=false, no_source=false;
        try { NativeProfileConsumer c(here.mailbox,here.geometry,here.grid,here.clock.config(),here.channel,
                                      &other.t,TFrameSource{Fixture::latest_t,&here},nullptr,SemanticFrameSource{},
                                      nullptr,HealthRoiView{},false); }
        catch (const std::invalid_argument&) { foreign=true; }
        try { NativeProfileConsumer c(here.mailbox,here.geometry,here.grid,here.clock.config(),here.channel,
                                      &here.t,TFrameSource{},nullptr,SemanticFrameSource{},nullptr,HealthRoiView{},false); }
        catch (const std::invalid_argument&) { no_source=true; }
        CHECK(foreign && no_source);
    }
    const TOptions zero_margin{.8,0,0,0,50000000};  // test branch only (B3R:42)
    Fixture f(options(),false,4,false,false,true,zero_margin);
    // 1. nothing in the slot: T still runs (Missing), the frame stays G-only
    CHECK(f.offer()==nd::Offer::Accepted);
    auto r=f.step(); success(r);
    CHECK(r.t_ran && r.t_status==TStatus::Missing && r.t_bins==0 && f.t_fetches==1);
    CHECK(r.grid_end_ns<=r.t_end_ns && r.t_end_ns<=r.build_end_ns);
    CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
    // 2. a fresh all-traversable mask of this frame set backs the FREE prefixes
    CHECK(f.offer()==nd::Offer::Accepted);
    f.publish_mask_of_current_frame();
    r=f.step(); success(r);
    CHECK(r.t_status==TStatus::Fresh && r.t_age_ns==0 && r.t_bins>0 && !r.t_near_support_missing);
    CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find(
        "\"t_seg_mono_ms\":"+std::to_string(r.capture.capture_mono_ns/1000000))!=std::string_view::npos);
    // 3. the same mask for later depth frames until it is older than 50 ms: Stale, G-only
    for (int i=0;i<6;++i) { CHECK(f.offer()==nd::Offer::Accepted); r=f.step(); }
    success(r);
    CHECK(r.t_status==TStatus::Stale && r.t_bins==0);
    CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
    // 4. an unbinarised 0/255 mask is InvalidMask: the frame is not published (fail-closed),
    //    which is why the service's slot converts 0/255 to 0/1
    std::fill(f.mask_pixels.begin(),f.mask_pixels.end(),255);
    CHECK(f.offer()==nd::Offer::Accepted);
    f.publish_mask_of_current_frame();
    const auto calls=f.sink.calls;
    r=f.step();
    CHECK(r.t_status==TStatus::InvalidMask && r.status==NativeProfileStatus::ProfileUnavailable && f.sink.calls==calls);
    // 5. fit failed: the withdrawal route does not consult T
    std::fill(f.raster.begin(),f.raster.end(),0);
    CHECK(f.offer()==nd::Offer::Accepted);
    r=f.step(); withdrawn_success(r);
    CHECK(!r.t_ran && r.t_end_ns==-1);
    // 6. positive margin (the B3-1 HOLD): Fresh, near-side support missing, no T backing
    Fixture margin(options(),false,4,false,false,true,TOptions{.8,0,0,.02,50000000});
    CHECK(margin.offer()==nd::Offer::Accepted);
    margin.publish_mask_of_current_frame();
    r=margin.step(); success(r);
    CHECK(r.t_status==TStatus::Fresh && r.t_near_support_missing && r.t_bins==0);
    CHECK(std::string_view(margin.sink.packet.data(),margin.sink.size).find("\"t_seg_mono_ms\":null")==std::string_view::npos);
    // 7. no ground reference: T reports GeometryUnavailable, the frame is still published
    auto go=options(); go.angle_min_rad=0; go.n_bins=1;
    Fixture ng(go,false,4,false,false,true,zero_margin);
    ng.exact_ground_support();
    for (std::size_t y=0;y<400;++y) ng.raster[y*640+320]=0;
    ng.raster[128*640+320]=384;
    CHECK(ng.offer()==nd::Offer::Accepted);
    ng.publish_mask_of_current_frame();
    r=ng.step(); success(r);
    CHECK(r.route==NativeProfileRoute::NoGroundReference && r.t_ran && r.t_status==TStatus::GeometryUnavailable);
    CHECK(std::string_view(ng.sink.packet.data(),ng.sink.size).find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
    // 8. T and S together: order geometry -> grid -> T -> S -> build
    Fixture both(options(),false,4,false,true,true,zero_margin);
    CHECK(both.offer()==nd::Offer::Accepted);
    both.publish_mask_of_current_frame();
    r=both.step(); success(r);
    CHECK(r.t_status==TStatus::Fresh && r.semantic_status==SemanticStatus::Missing && r.t_bins>0);
    CHECK(r.grid_end_ns<=r.t_end_ns && r.t_end_ns<=r.semantic_end_ns && r.semantic_end_ns<=r.build_end_ns);
}

// 2026-10-10: an S snapshot that expires between its check and the publish entry. Without a
// publish guard the body is refused and the tick names the cause (the 1.8 A/B i1ab1010 lost 2 to
// 4 % of the Profile frames this way once Objects became fast enough to sit at the lifetime
// bound); with the guard the same snapshot is Stale at the check and the frame is published
// geometry-only. Lifetime 50 ms, frames 10 ms apart, the check about 2.6 ms after capture.
void test_semantic_publish_guard() {
    SemanticObject object;
    object.track_id=5; object.class_id=3; object.class_name="chair";
    object.vertices=4;
    object.footprint[0]={.60F,-.10F}; object.footprint[1]={.70F,-.10F};
    object.footprint[2]={.70F,.10F}; object.footprint[3]={.60F,.10F};
    // 1. no guard: Fresh at the check, 60 ms pass before the publish entry, the body is not put
    {
        Fixture f(options(),false,4,false,true);
        CHECK(f.offer()==nd::Offer::Accepted);
        SemanticFrame frame{f.clock.expected,77,SemanticMaterial::MeasuredBaseFootprint,&object,1};
        f.semantic_slot=&frame;
        auto r=f.step(); success(r);
        CHECK(r.semantic_status==SemanticStatus::Fresh && r.semantic_bins>0);
        CHECK(!r.semantic_stale_by_guard && !r.semantic_expired_at_publish);
        CHECK(r.semantic_check_ns>=r.grid_end_ns && r.semantic_check_ns<=r.semantic_end_ns);
        CHECK(r.publish_entry_ns>=r.build_end_ns && r.publish_entry_ns<=r.end_ns);
        // The clock advances 0.1 ms per read: the index of the S check among this step's reads.
        const auto check_read=static_cast<std::size_t>((r.semantic_check_ns-r.begin_ns)/100000)+1;
        CHECK(f.offer()==nd::Offer::Accepted);
        f.clock.jump_at=f.clock.calls+check_read+1;       // the read right after the S check
        f.clock.jump_ns=60000000;
        const auto calls=f.sink.calls;
        r=f.step();
        CHECK(r.semantic_status==SemanticStatus::Fresh && r.semantic_expired_at_publish);
        CHECK(r.status==NativeProfileStatus::ProfileUnavailable && !r.put.dispatched && f.sink.calls==calls);
        CHECK(r.publish_entry_ns-r.semantic_check_ns>=60000000);
        // the next frame is unaffected: the snapshot is now simply stale
        CHECK(f.offer()==nd::Offer::Accepted);
        r=f.step(); success(r);
        CHECK(r.semantic_status==SemanticStatus::Stale && !r.semantic_stale_by_guard && !r.semantic_expired_at_publish);
    }
    // 2. guard 10 ms: the snapshot is used while younger than 40 ms at the check, is Stale by
    //    guard between 40 and 50 ms, and plain Stale after that; every frame is published, also
    //    when 60 ms pass between the check and the publish entry of a guarded frame
    {
        fixture_publish_guard_ns=10000000;
        Fixture f(options(),false,4,false,true);
        fixture_publish_guard_ns=0;
        CHECK(f.offer()==nd::Offer::Accepted);
        SemanticFrame frame{f.clock.expected,77,SemanticMaterial::MeasuredBaseFootprint,&object,1};
        f.semantic_slot=&frame;
        auto r=f.step(); success(r);
        const auto check_read=static_cast<std::size_t>((r.semantic_check_ns-r.begin_ns)/100000)+1;
        CHECK(r.semantic_status==SemanticStatus::Fresh && r.semantic_bins>0);
        for (int i=0;i<3;++i) {                           // ages about 13, 23 and 33 ms
            CHECK(f.offer()==nd::Offer::Accepted);
            r=f.step(); success(r);
            CHECK(r.semantic_status==SemanticStatus::Fresh && r.semantic_bins>0 && !r.semantic_stale_by_guard);
        }
        CHECK(f.offer()==nd::Offer::Accepted);            // about 43 ms: inside the guard
        f.clock.jump_at=f.clock.calls+check_read+1;
        f.clock.jump_ns=60000000;
        r=f.step(); success(r);
        CHECK(r.semantic_status==SemanticStatus::Stale && r.semantic_stale_by_guard && r.semantic_bins==0);
        CHECK(!r.semantic_expired_at_publish && r.publish_entry_ns-r.semantic_check_ns>=60000000);
        CHECK(f.offer()==nd::Offer::Accepted);            // far past the lifetime now
        r=f.step(); success(r);
        CHECK(r.semantic_status==SemanticStatus::Stale && !r.semantic_stale_by_guard && r.semantic_bins==0);
    }
}

// A19-TIME-2 / A19-SEG-2 at host level (user decision 2026-09-30, B9-13 (a)), with the real
// service slot between a fake producer and the real consumer: the producer publishes the mask of
// every frame, stalls for 5 s (10 depth frames 10 ms apart, then 49 frames 100 ms apart: few frames,
// because each runs the full T owner and the 1.8 sanitizer build takes about 0.2 s per frame),
// then resumes. Every depth frame is
// still published (P19-1: the fast line never waits for the slow one); the stale mask stops
// backing T once older than seg_max_age (50 ms here: t_seg null, no T-backed bin); the first
// publication after the stall backs T again on the next frame (no perception-side hysteresis).
// Zero margin (test branch only, B3R:42) so the T backing is visible.
void test_t_slot_stall() {
    Fixture f(options(),false,4,false,false,true,TOptions{.8,0,0,0,50000000});
    perception::service::TMaskSlot slot;
    f.slot_snapshot=std::make_unique<perception::service::TMaskSlot::Snapshot>();
    f.real_slot=&slot;
    std::size_t published=0, offered=0, fresh=0, stale=0, backed_during_stall=0;
    std::int64_t first_stale_frame=-1;
    const auto frame=[&](bool publish_mask, int index, std::int64_t extra_gap_ns=0) {
        f.clock.ns+=extra_gap_ns;
        CHECK(f.offer()==nd::Offer::Accepted); ++offered;
        if (publish_mask)
            CHECK(slot.publish(f.mask_pixels.data(),1280,800,1280,f.clock.expected,f.projection.revision));
        const auto r=f.step();
        published+=r.status==NativeProfileStatus::PutFinished && r.put.dispatched;
        fresh+=r.t_status==TStatus::Fresh;
        if (r.t_status==TStatus::Stale) { ++stale; if (first_stale_frame<0) first_stale_frame=index; }
        return r;
    };
    for (int i=0;i<5;++i) { const auto r=frame(true,i); CHECK(r.t_status==TStatus::Fresh && r.t_bins>0); }
    const auto stall_begin_ns=f.clock.ns;
    for (int i=0;i<59;++i) {              // producer stalled for 5 s
        const auto r=frame(false,i,i<10 ? 0 : 90000000);
        backed_during_stall+=r.t_bins>0;
        if (r.t_status==TStatus::Stale)
            CHECK(std::string_view(f.sink.packet.data(),f.sink.size).find("\"t_seg_mono_ms\":null")!=std::string_view::npos);
    }
    const auto after=frame(true,0);         // producer resumes
    CHECK(after.t_status==TStatus::Fresh && after.t_bins>0);
    CHECK(f.clock.ns-stall_begin_ns>=5000000000LL);                // at least 5 s without a mask
    CHECK(published==offered && offered==65);                      // publication never stalled
    CHECK(first_stale_frame>=4 && first_stale_frame<=6);          // mask older than 50 ms after 5 frames
    CHECK(stale==59-static_cast<std::size_t>(first_stale_frame) && backed_during_stall==static_cast<std::size_t>(first_stale_frame));
    CHECK(slot.counters().torn_giveup==0 && slot.counters().refused_future==0 && slot.counters().refused_order==0);
    std::cout << "t slot stall: offered=" << offered << " published=" << published << " fresh=" << fresh
              << " stale=" << stale << " first_stale_frame=" << first_stale_frame << "\n";
}

int main(int argc,char** argv) {
    if (argc==2 && std::string_view(argv[1])=="--emit-r6-native-fixtures") {
        test_failure_recovery_and_restart(true); test_no_ground_reference(true); return 0;
    }
    const bool emit=argc==2 && std::string_view(argv[1])=="--fixtures";
    test_latest_and_idle(emit);
    if (emit) return 0;
    test_clock_and_identity(); test_failure_recovery_and_restart(); test_put_and_late_invalidation(); test_setup();
    test_no_ground_reference(); test_withdrawal_guards(); test_processed_health(); test_calibration_verdict();
    test_semantic_owner(); test_t_owner(); test_t_slot_stall(); test_semantic_publish_guard();
    std::cout << "native consumer 13/13 groups PASS; " << steps
        << " steps C++ allocation delta=0; G-only, G+S, G+T and G+T+S owners; synthetic clocks/rays/input; NOT camera/network/20Hz\n";
}
