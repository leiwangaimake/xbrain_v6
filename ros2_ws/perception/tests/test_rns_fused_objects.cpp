/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_fused_objects.cpp
 * Brief: Unit tests for the Objects adapter from fused detections to the r6 Objects body.
 *
 * Description:
 * What it checks. prepare() validates the frame before the tracker runs and build() maps the
 * fused, tracked detections into Objects. Cases cover preflight checks, the final population,
 * empty frames and coasting tracks, malformed final input, consumed and mismatched frames, context
 * lifetime, and the geometry/velocity mapping for a level forward camera (x_b = z_c, y_b = -x_c,
 * z_b = -y_c); explicit refusals cover contradictory flags, unknown status and a non-camera frame.
 */

/* Offline real-tracker / final-fused R6 bridge regression. No SDK/device/model/network. */
#include "output/rns_fused_objects.hpp"
#include "modules/bytetrack.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace perception;
using namespace perception::rns;
using S=FusedObjectsStatus;
std::size_t checks=0, groups=0;
void check(bool ok,int line) { ++checks; if(!ok) throw std::runtime_error("fused objects line "+std::to_string(line)); }
#define CHECK(x) check(static_cast<bool>(x),__LINE__)
yolo::DetectionResult input(std::uint64_t seq, bool detected=true) {
    yolo::DetectionResult r;
    r.inference_completed=true; r.frame_id=seq*7;
    r.native_depth_source={31,seq,seq+10,seq+20,1000000+seq*1000,1000001+seq*1000,
        static_cast<std::int64_t>(1000000000+seq*50000000+5000000)};
    r.stamp_sec=42; // deliberately unrelated old wall clock
    if(detected) {
        yolo::Detection d; d.class_id=0; d.class_name="person"; d.confidence=.9F;
        d.bbox={10,20,30,40}; d.mask=cv::Mat::ones(40,30,CV_8UC1);
        r.detections.push_back(d);
    }
    return r;
}
geometry::CaptureStamp stamp(const yolo::DetectionResult& r) {
    return {r.native_depth_source,7,r.native_depth_source.host_ready_mono_ns-5000000};
}
ByteTracker::Params params() {
    ByteTracker::Params p; p.min_hits=1; p.low_confidence_min_hits=1; p.max_age=30; p.match_thresh=.5F; return p;
}
FusedDetection fused(const Track& t) {
    FusedDetection f; f.track_id=t.track_id; f.class_id=t.class_id; f.class_name=t.class_name;
    f.confidence=t.confidence; f.semantic_status=t.semantic_status; f.stable_frames=t.stable_frames;
    f.last_seen_frame=t.last_seen_frame;
    f.first_seen_ms=9999999999; f.last_seen_ms=9999999999; // must never become capture-mono
    f.velocity_valid=true; f.velocity_status="moving"; f.velocity_frame=FusedDetection::VelocityFrame::Camera;
    f.velocity_3d[0]=123; f.centroid_3d[0]=456;
    f.ground_footprint_xy={{{1,2}},{{3,4}},{{5,6}}};
    return f;
}
struct Fixture {
    FusedObjectsAdapter adapter{7}; ByteTracker tracker{params()};
    std::vector<Track> tracks; std::vector<FusedDetection> final;
    std::size_t tracker_calls=0;
    void prepare(const yolo::DetectionResult& r) {
        const auto c=stamp(r);
        CHECK(adapter.prepare(r,c,2000000000)==S::Prepared);
        ++tracker_calls; tracks=tracker.update(r.detections,r.frame_id,c.capture_mono_ns/1000000);
        final.clear(); for(const auto& t:tracks) final.push_back(fused(t));
    }
    FusedObjectsReport build(const yolo::DetectionResult& r) { return adapter.build(r,stamp(r),final,tracks); }
};
void preflight() {
    ++groups; FusedObjectsAdapter bad(0); CHECK(!bad.valid());
    Fixture f; auto r=input(1); auto c=stamp(r);
    auto failed=r; failed.inference_completed=false;
    CHECK(f.adapter.prepare(failed,c,2000000000)==S::FailedInference);
    for(int i=0;i<7;++i) {
        auto mismatch=c;
        switch(i) {
            case 0: ++mismatch.source.stream_id; break;
            case 1: ++mismatch.source.sequence; break;
            case 2: ++mismatch.source.color_index; break;
            case 3: ++mismatch.source.depth_index; break;
            case 4: ++mismatch.source.color_device_us; break;
            case 5: ++mismatch.source.depth_device_us; break;
            default: ++mismatch.source.host_ready_mono_ns;
        }
        CHECK(f.adapter.prepare(r,mismatch,2000000000)==S::SourceMismatch);
    }
    auto wrong_epoch=c; ++wrong_epoch.clock_epoch;
    CHECK(f.adapter.prepare(r,wrong_epoch,2000000000)==S::ClockRejected);
    auto future=c; future.capture_mono_ns=r.native_depth_source.host_ready_mono_ns+1;
    CHECK(f.adapter.prepare(r,future,2000000000)==S::ClockRejected);
    CHECK(f.adapter.prepare(r,c,-1)==S::ClockRejected);
    auto invalid=r; invalid.native_depth_source.stream_id=0;
    CHECK(f.adapter.prepare(invalid,stamp(invalid),2000000000)==S::InvalidSource);
    invalid=r; invalid.frame_id=static_cast<std::uint64_t>(std::numeric_limits<int>::max())+1;
    CHECK(f.adapter.prepare(invalid,c,2000000000)==S::InvalidSource);
    invalid=r; invalid.detections[0].confidence=std::numeric_limits<float>::quiet_NaN();
    CHECK(f.adapter.prepare(invalid,c,2000000000)==S::InvalidDetections);
    CHECK(f.tracker_calls==0); f.prepare(r); CHECK(f.tracker_calls==1);
    CHECK(f.adapter.prepare(r,c,2000000000)==S::Pending);
    CHECK(f.build(r).status==S::Ready);
    CHECK(f.adapter.prepare(r,c,2000000000)==S::Duplicate);
    auto changed=input(2); ++changed.native_depth_source.stream_id;
    CHECK(f.adapter.prepare(changed,stamp(changed),2000000000)==S::StreamChanged);
    auto old=input(2); old.frame_id=r.frame_id;
    CHECK(f.adapter.prepare(old,stamp(old),2000000000)==S::OutOfOrder);
    CHECK(f.tracker_calls==1);
}
void final_population() {
    ++groups; Fixture f; auto r=input(1);
    auto second=r.detections[0]; second.class_id=1; second.class_name="chair"; second.bbox.x=100;
    r.detections.push_back(second); f.prepare(r); CHECK(f.tracks.size()==2);
    const int kept=f.final[1].track_id;
    f.final.erase(f.final.begin()); // actual downstream gate rejected the first track
    const auto report=f.build(r);
    CHECK(report.status==S::Ready && report.input_fused==1 && report.tracks==2 && report.objects==1);
    const auto& b=f.adapter.last_body(); const auto& o=b.objects.at(0);
    CHECK(o.track_id==kept && !o.geometry && !o.velocity_xy && !o.velocity_valid);
    CHECK(o.velocity_frame=="raw" && o.velocity_status=="warming_up" && o.depth_quality=="poor");
    CHECK(o.first_seen_mono_ms==1050 && o.last_seen_mono_ms==1050);
    CHECK(!b.extrinsic_calibrated && b.t_capture_mono_ms==1050 && b.t_publish_mono_ms==-1);
    CHECK(f.tracker_calls==1);
    char wire[8192]; const auto encoded=encode_at_publish(b,1051,wire,sizeof(wire));
    CHECK(encoded); // no old camera geometry leaked into the wire bundle
}
void empty_and_coast() {
    ++groups; Fixture f; auto r=input(1); f.prepare(r); CHECK(f.build(r).status==S::Ready);
    const int id=f.adapter.last_body().objects.at(0).track_id;
    auto empty=input(2,false); f.prepare(empty); CHECK(f.tracks.size()==1);
    const auto report=f.build(empty); CHECK(report.status==S::Ready && report.coast_filtered==1 && report.objects==0);
    auto again=input(3); f.prepare(again); CHECK(f.build(again).status==S::Ready);
    CHECK(f.adapter.last_body().objects.at(0).track_id==id);
    CHECK(f.adapter.last_body().objects.at(0).first_seen_mono_ms==1050);
    CHECK(f.adapter.last_body().objects.at(0).last_seen_mono_ms==1150);
    Fixture truly_empty; auto e=input(1,false); truly_empty.prepare(e);
    CHECK(truly_empty.build(e).status==S::Ready && truly_empty.adapter.last_body().objects.empty());
}
// 2026-10-13: the service no longer builds geometry for a coasting track. That is only an
// equivalent change if build() never reads the base_* fields of such a track: here it is handed
// geometry that would be refused as InvalidFused for a published object, and must not notice.
void coast_geometry_not_read() {
    ++groups;
    CHECK(observed_in_frame(7,7) && !observed_in_frame(6,7) && !observed_in_frame(8,7) && !observed_in_frame(-1,7));
    CHECK(observed_in_frame(0,0) && observed_in_frame(std::numeric_limits<int>::max(),
        static_cast<std::uint64_t>(std::numeric_limits<int>::max())));
    // A frame id that an int cannot hold is never "observed" (prepare() refuses such a frame anyway).
    CHECK(!observed_in_frame(0, static_cast<std::uint64_t>(std::numeric_limits<int>::max())+1));
    CHECK(!observed_in_frame(-1, std::numeric_limits<std::uint64_t>::max()));
    const auto poison=[](FusedDetection& d) {
        d.base_geometry_valid=true; d.base_footprint_xy={{{0,0}},{{0,0}}}; // not a polygon
        d.base_z_min=std::numeric_limits<float>::quiet_NaN(); d.base_z_max=-1;
    };
    // The same poisoned geometry on an observed track IS read and refused: the check is live.
    { Fixture f; auto r=input(1); f.prepare(r); poison(f.final.at(0)); CHECK(f.build(r).status==S::InvalidFused); }
    // On a coasting track it changes nothing: same report, same (empty) body, and the next
    // frame, in which the track is seen again, is published as if nothing had happened.
    Fixture plain, poisoned;
    for (Fixture* f : {&plain, &poisoned}) { auto r=input(1); f->prepare(r); CHECK(f->build(r).status==S::Ready); }
    auto gap=input(2,false);
    plain.prepare(gap); poisoned.prepare(gap); CHECK(poisoned.final.size()==1); poison(poisoned.final.at(0));
    const auto a=plain.build(gap), b=poisoned.build(gap);
    CHECK(a.status==S::Ready && b.status==S::Ready && a.coast_filtered==1 && b.coast_filtered==1);
    CHECK(a.objects==0 && b.objects==0 && a.located==b.located && a.unlocatable==b.unlocatable);
    CHECK(plain.adapter.last_body().objects.empty() && poisoned.adapter.last_body().objects.empty());
    auto again=input(3);
    plain.prepare(again); poisoned.prepare(again);
    CHECK(plain.build(again).status==S::Ready && poisoned.build(again).status==S::Ready);
    char wa[8192], wb[8192];
    const auto ea=encode_at_publish(plain.adapter.last_body(),1151,wa,sizeof(wa));
    const auto eb=encode_at_publish(poisoned.adapter.last_body(),1151,wb,sizeof(wb));
    CHECK(ea && eb && ea.size==eb.size && std::string(wa,ea.size)==std::string(wb,eb.size));
}
void malformed_final() {
    ++groups;
    for(int i=0;i<8;++i) {
        Fixture f; auto r=input(1); f.prepare(r);
        switch(i) {
            case 0: f.tracks.clear(); break;
            case 1: f.tracks.push_back(f.tracks.front()); break;
            case 2: f.final.push_back(f.final.front()); break;
            case 3: ++f.final.front().class_id; break;
            case 4: f.tracks.front().rns_observation_times.reset(); break;
            case 5: ++f.tracks.front().rns_observation_times->last_seen_mono_ms; break;
            case 6: f.final.front().confidence=.123F; break;
            default: f.tracks.front().rns_observation_times->first_seen_mono_ms=-1;
        }
        CHECK(f.build(r).status!=S::Ready && !f.adapter.valid());
        CHECK(f.adapter.last_body().objects.empty());
        CHECK(f.adapter.prepare(input(2),stamp(input(2)),2000000000)==S::Faulted);
        CHECK(f.tracker_calls==1);
    }
}
void consumed_and_mismatch() {
    ++groups; auto r=input(1);
    FusedObjectsAdapter no_prepare(7); CHECK(no_prepare.build(r,stamp(r),{},{}).status==S::NoPreparation);
    Fixture f; f.prepare(r); auto wrong=stamp(r); ++wrong.source.color_index;
    CHECK(f.adapter.build(r,wrong,f.final,f.tracks).status==S::SourceMismatch);
    CHECK(f.adapter.prepare(r,stamp(r),2000000000)==S::Faulted);
    Fixture consumed; consumed.prepare(r); CHECK(consumed.build(r).status==S::Ready);
    CHECK(consumed.build(r).status==S::NoPreparation);
    Fixture aborted; aborted.prepare(r); aborted.adapter.abort();
    CHECK(aborted.build(r).status==S::Faulted && aborted.tracker_calls==1);
}
std::int64_t now(void*) noexcept { return 2000000000; }
void context_lifetime() {
    ++groups; SupervisorRnsContext c; CHECK(!c.valid());
    c.native_mailbox=std::make_shared<native_depth::Mailbox>(64);
    c.depth_arrivals=std::make_shared<SourceJournal>(SourceKind::DepthArrival,8,SourceClock{nullptr,&now});
    c.inference_completions=std::make_shared<SourceJournal>(SourceKind::InferenceComplete,8,SourceClock{nullptr,&now});
    auto owner=std::make_shared<int>(3); std::weak_ptr<int> weak=owner; c.callback_owner=owner;
    c.clock_epoch=7; c.now_ns=[] { return std::int64_t{2000000000}; };
    c.capture_stamp=[](const native_depth::SourceId& s,geometry::CaptureStamp& out) { out={s,7,s.host_ready_mono_ns-5000000}; return true; };
    c.objects_ready=[](const yolo::DetectionResult&,const geometry::CaptureStamp&,const Objects&,const FusedObjectsReport&) { return true; };
    CHECK(c.valid()); auto copied=c; owner.reset(); c={}; CHECK(!weak.expired() && copied.valid());
    auto wrong=copied; wrong.inference_completions=wrong.depth_arrivals; CHECK(!wrong.valid());
    wrong={}; copied={}; CHECK(weak.expired());
}
}
// Measured base_link geometry and raw velocity mapping (19 section 4.1, 11 v2.3
// row IV; gating statuses outside the five-value set -> null + warming_up).
void geometry_and_velocity() {
    ++groups;
    // base <- colour optical for a level forward camera: x_b = z_c, y_b = -x_c, z_b = -y_c
    const std::array<float, 9> R{0, 0, 1, -1, 0, 0, 0, -1, 0};
    const auto located = [&](FusedDetection& f, const std::string& status) {
        f.base_geometry_valid = true;
        f.base_footprint_xy = {{{2.0F, -0.2F}}, {{2.5F, -0.2F}}, {{2.5F, 0.2F}}, {{2.0F, 0.2F}}};
        f.base_z_min = 0.0F; f.base_z_max = 1.0F; f.base_from_colour_R = R;
        f.velocity_3d[0] = 0.25F; f.velocity_3d[1] = -0.1F; f.velocity_3d[2] = 1.5F;
        f.velocity_status = status;
    };
    {
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), "moving"); f.final.at(0).velocity_valid = true;
        const auto report = f.build(r);
        CHECK(report.status == S::Ready && report.located == 1 && report.unlocatable == 0 && report.velocity_numeric == 1);
        const auto& o = f.adapter.last_body().objects.at(0);
        CHECK(o.geometry && o.geometry->footprint_xy.size() == 4 && o.geometry->z_min == 0.0F && o.geometry->z_max == 1.0F);
        CHECK(o.velocity_xy && (*o.velocity_xy)[0] == 1.5F && (*o.velocity_xy)[1] == -0.25F);
        CHECK(o.velocity_valid && o.velocity_status == "moving" && o.velocity_frame == "raw");
        char wire[8192]; const auto e = encode_at_publish(f.adapter.last_body(), 1051, wire, sizeof(wire));
        CHECK(e && std::string(wire, e.size).find("\"r_near\":2") != std::string::npos);
    }
    {
        // B4-1: an unconfirmed window keeps its real candidate, numeric and not valid
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), "motion_unconfirmed"); f.final.at(0).velocity_valid = false;
        const auto report = f.build(r);
        CHECK(report.status == S::Ready && report.velocity_unconfirmed_numeric == 1 && report.velocity_numeric == 1);
        const auto& o = f.adapter.last_body().objects.at(0);
        CHECK(o.geometry && o.velocity_xy && (*o.velocity_xy)[0] == 1.5F && (*o.velocity_xy)[1] == -0.25F);
        CHECK(!o.velocity_valid && o.velocity_status == "motion_unconfirmed");
        char wire[8192]; CHECK(encode_at_publish(f.adapter.last_body(), 1051, wire, sizeof(wire)));
    }
    {
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), "static"); f.final.at(0).velocity_valid = true;
        for (auto& v : f.final.at(0).velocity_3d) v = 0.0F;
        CHECK(f.build(r).status == S::Ready);
        const auto& o = f.adapter.last_body().objects.at(0);
        CHECK(o.velocity_xy && (*o.velocity_xy)[0] == 0.0F && o.velocity_valid && o.velocity_status == "static");
    }
    // A19-NULL-2 (19 section 14; 11 section 3.1B.2), this loop and the next: a located object
    // whose estimator has no estimate is published with velocity_xy null and velocity_valid false,
    // although velocity_3d holds numbers here; warming_up and timestamp_gap pass through, the
    // three gate statuses are published as warming_up. The mandated mutant of the row, [0, 0] with
    // velocity_valid true, turns the body CHECK of its loop red. The odom-frame block further
    // down belongs to the same row; the PASS line is printed after it.
    for (const char* status : {"warming_up", "timestamp_gap"}) {
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), status); f.final.at(0).velocity_valid = false;
        CHECK(f.build(r).status == S::Ready);
        const auto& o = f.adapter.last_body().objects.at(0);
        CHECK(o.geometry && !o.velocity_xy && !o.velocity_valid && o.velocity_status == status);
    }
    for (const char* gating : {"track_unstable", "low_depth_confidence", "depth_unreliable"}) {
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), gating); f.final.at(0).velocity_valid = false;
        const auto report = f.build(r);
        CHECK(report.status == S::Ready && report.velocity_gating_mapped == 1);
        CHECK(report.velocity_gated_track_unstable + report.velocity_gated_low_depth_confidence +
              report.velocity_gated_depth_unreliable == 1);
        CHECK((std::string(gating) == "track_unstable") == (report.velocity_gated_track_unstable == 1));
        CHECK((std::string(gating) == "depth_unreliable") == (report.velocity_gated_depth_unreliable == 1));
        const auto& o = f.adapter.last_body().objects.at(0);
        CHECK(o.geometry && !o.velocity_xy && !o.velocity_valid && o.velocity_status == "warming_up");
        char wire[8192]; CHECK(encode_at_publish(f.adapter.last_body(), 1051, wire, sizeof(wire)));
    }
    {
        // unlocatable keeps the detection with all geometry and velocity null
        Fixture f; auto r = input(1); f.prepare(r);
        f.final.at(0).velocity_status = "moving";
        const auto report = f.build(r);
        CHECK(report.status == S::Ready && report.unlocatable == 1);
        const auto& o = f.adapter.last_body().objects.at(0);
        CHECK(!o.geometry && !o.velocity_xy && !o.velocity_valid && o.velocity_status == "warming_up");
    }
    // ---- explicit refusals: contradictory flags, unknown status, non-camera frame ----
    struct Bad { const char* status; bool valid; FusedDetection::VelocityFrame frame; bool located; const char* detail; };
    const Bad bad[] = {
        {"sprinting", false, FusedDetection::VelocityFrame::Camera, true, "unknown velocity_status 'sprinting'"},
        {"sprinting", false, FusedDetection::VelocityFrame::Camera, false, "unknown velocity_status"},  // also when unlocatable
        {"moving", false, FusedDetection::VelocityFrame::Camera, true, "velocity_valid=false"},
        {"static", false, FusedDetection::VelocityFrame::Camera, true, "velocity_valid=false"},
        {"motion_unconfirmed", true, FusedDetection::VelocityFrame::Camera, true, "velocity_valid=true"},
        {"warming_up", true, FusedDetection::VelocityFrame::Camera, true, "velocity_valid=true"},
        {"depth_unreliable", true, FusedDetection::VelocityFrame::Camera, true, "velocity_valid=true"},
        {"moving", true, FusedDetection::VelocityFrame::Odom, true, "odom frame"},
        {"motion_unconfirmed", false, FusedDetection::VelocityFrame::Odom, false, "odom frame"},
        {"static", true, FusedDetection::VelocityFrame::Unset, true, "unset frame"},
    };
    for (const auto& b : bad) {
        Fixture f; auto r = input(1); f.prepare(r);
        if (b.located) located(f.final.at(0), b.status); else f.final.at(0).velocity_status = b.status;
        f.final.at(0).velocity_valid = b.valid; f.final.at(0).velocity_frame = b.frame;
        const auto report = f.build(r);
        CHECK(report.status == S::InvalidFused && !f.adapter.valid() && report.detail.find(b.detail) != std::string::npos);
    }
    {
        // null statuses need no frame: a gate or warm-up in the odom frame is still publishable (null)
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), "timestamp_gap"); f.final.at(0).velocity_valid = false;
        f.final.at(0).velocity_frame = FusedDetection::VelocityFrame::Odom;
        CHECK(f.build(r).status == S::Ready && !f.adapter.last_body().objects.at(0).velocity_xy);
    }
    std::cout << "A19-NULL-2 adapter: no velocity estimate -> velocity_xy null, velocity_valid false, "
                 "warming_up / timestamp_gap PASS\n";
    {
        // clockwise footprint is a malformed fused population, never published
        Fixture f; auto r = input(1); f.prepare(r);
        located(f.final.at(0), "static");
        std::reverse(f.final.at(0).base_footprint_xy.begin(), f.final.at(0).base_footprint_xy.end());
        CHECK(f.build(r).status == S::InvalidFused && !f.adapter.valid());
    }
}
int main() {
    try {
        preflight(); final_population(); empty_and_coast(); coast_geometry_not_read(); malformed_final(); consumed_and_mismatch(); context_lifetime();
        geometry_and_velocity();
        std::cout << "{\"status\":\"PASS\",\"groups\":" << groups << ",\"checks\":" << checks
            << ",\"device\":false,\"model_execution\":false,\"network\":false,\"physical_geometry\":false}\n";
        return 0;
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
