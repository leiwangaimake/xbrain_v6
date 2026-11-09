/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_supervisor_rns_context.cpp
 * Brief: Unit tests for the Supervisor's per-frame timing ledger handed to both output paths.
 *
 * Description:
 * What it checks. Before a frame is published, finalize_prepublish_timing() completes one timing
 * ledger for the Objects frame. With two distinct frames the test shows that no timestamp or
 * source identity of a previous frame is reused. It also covers the Supervisor's R6-only start
 * (init refuses without configure_rns), the fail-closed Gemini runtime geometry check and the
 * parallel depth-sample environment gate; the last two moved here on 2026-09-28 from the retired
 * legacy-mode test test_supervisor_zenoh_degrade.cpp.
 */

/* Startup wiring and same-frame metadata only: no modules, camera or model run. */
#include "supervisor/supervisor.hpp"
#include "common/time_utils.hpp"
#include "modules/rgbd_capture_factory.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace perception {
struct SupervisorRnsJoinTestAccess {
    static void finalize_timing(yolo::DetectionResult& result, PerceptionFrame& frame) {
        Supervisor::finalize_prepublish_timing(result, frame);
    }
    static bool join(Supervisor& owner, std::thread& thread,
                     std::future<void> (*launch)(std::thread&)) {
        return owner.wait_for_thread_exit(thread,"offline_owned_join",std::chrono::milliseconds(1),launch);
    }
    static void update_velocity(Supervisor& owner, std::vector<FusedDetection>& fused,
                                const geometry::CaptureStamp& capture, bool has_odom) {
        owner.update_velocity(fused, capture, has_odom);
    }
    static bool parallel_depth_sample_enabled(const std::string& camera_driver) {
        return Supervisor::parallel_depth_sample_enabled(camera_driver);
    }
    // The geometry check reads only the camera driver and output size from the
    // configuration; a borrowed Config stands in for a full R6 init.
    static bool apply_geometry(Supervisor& owner, const yolo::Config& config,
                              const yolo::DetectionResult& result, PerceptionFrame& frame) {
        owner.config_ = &config;
        const bool ok = owner.apply_gemini_runtime_geometry(result, frame);
        owner.config_ = nullptr;
        return ok;
    }
};
}

namespace {
using namespace perception;
using namespace perception::rns;
std::size_t checks=0;
void check(bool ok,int line) { ++checks; if(!ok) throw std::runtime_error("Supervisor R6 context line "+std::to_string(line)); }
#define CHECK(x) check(static_cast<bool>(x),__LINE__)
void same_timing(const yolo::PipelineTiming& a,const yolo::PipelineTiming& b) {
    CHECK(a.valid==b.valid);
    CHECK(a.capture_epoch_ns==b.capture_epoch_ns);
    CHECK(a.capture_ready_ns==b.capture_ready_ns);
    CHECK(a.frame_push_begin_ns==b.frame_push_begin_ns);
    CHECK(a.inference_pop_end_ns==b.inference_pop_end_ns);
    CHECK(a.result_push_begin_ns==b.result_push_begin_ns);
    CHECK(a.supervisor_pop_end_ns==b.supervisor_pop_end_ns);
    CHECK(a.prepublish_ns==b.prepublish_ns);
    CHECK(a.preprocess_ms==b.preprocess_ms && a.clone_ms==b.clone_ms);
    CHECK(a.execute_pair_ms==b.execute_pair_ms && a.post_wall_ms==b.post_wall_ms);
    CHECK(a.merge_ms==b.merge_ms);
}
void same_source(const native_depth::SourceId& a,const native_depth::SourceId& b) {
    CHECK(a.stream_id==b.stream_id && a.sequence==b.sequence);
    CHECK(a.color_index==b.color_index && a.depth_index==b.depth_index);
    CHECK(a.color_device_us==b.color_device_us && a.depth_device_us==b.depth_device_us);
    CHECK(a.host_ready_mono_ns==b.host_ready_mono_ns);
}
// Each frame's ledger gets its own prepublish time and source, never a previous frame's; disabled
// or broken timing stays invalid in both views.
void test_prepublish_timing() {
    // Two distinct same-frame ledgers: no prior-frame timestamp/source reuse.
    std::int64_t prior_publish=0;
    for(std::uint64_t sequence=1;sequence<=2;++sequence) {
        yolo::DetectionResult result;
        result.frame_id=100+sequence; result.inference_completed=true;
        const auto ready=monotonic_now_ns()-10000;
        CHECK(ready>0);
        result.native_depth_source={9,sequence,sequence+10,sequence+20,
                                    sequence*1000+1,sequence*1000+2,ready};
        auto& timing=result.pipeline_timing;
        timing.valid=true; timing.capture_epoch_ns=1700000000000000000LL+sequence;
        timing.capture_ready_ns=ready; timing.frame_push_begin_ns=ready+1;
        timing.inference_pop_end_ns=ready+2; timing.result_push_begin_ns=ready+3;
        timing.supervisor_pop_end_ns=ready+4;
        timing.preprocess_ms=sequence+0.1; timing.clone_ms=sequence+0.2;
        timing.execute_pair_ms=sequence+0.3; timing.post_wall_ms=sequence+0.4;
        timing.merge_ms=sequence+0.5;
        const auto original_source=result.native_depth_source;
        const auto original_timing=timing;
        PerceptionFrame frame; frame.frame_id=result.frame_id;
        frame.pipeline_timing=timing; // Same value-copy seam as build_frame.
        const auto before=monotonic_now_ns();
        SupervisorRnsJoinTestAccess::finalize_timing(result,frame);
        const auto after=monotonic_now_ns();
        CHECK(timing.valid && frame.pipeline_timing.valid);
        CHECK(timing.prepublish_ns>0 && before<=timing.prepublish_ns && timing.prepublish_ns<=after);
        CHECK(prior_publish<=timing.prepublish_ns); prior_publish=timing.prepublish_ns;
        same_timing(timing,frame.pipeline_timing);
        auto expected=original_timing; expected.prepublish_ns=timing.prepublish_ns;
        same_timing(timing,expected);
        same_source(result.native_depth_source,original_source);
        CHECK(result.frame_id==100+sequence && frame.frame_id==result.frame_id);
        CHECK(result.inference_completed);
        // This is the exact result argument type consumed by objects_ready.
        const auto observe=[&](const yolo::DetectionResult& delivered) {
            CHECK(&delivered==&result);
            CHECK(delivered.pipeline_timing.prepublish_ns==frame.pipeline_timing.prepublish_ns);
            same_source(delivered.native_depth_source,original_source);
        };
        observe(result);
    }
    // Disabled profiling must not acquire a seemingly valid observation.
    yolo::DetectionResult invalid; PerceptionFrame frame;
    frame.pipeline_timing=invalid.pipeline_timing;
    SupervisorRnsJoinTestAccess::finalize_timing(invalid,frame);
    CHECK(!invalid.pipeline_timing.valid && invalid.pipeline_timing.prepublish_ns==0);
    same_timing(invalid.pipeline_timing,frame.pipeline_timing);
    // A missing or reversed endpoint remains invalid in BOTH callback views.
    for(const bool reversed:{false,true}) {
        yolo::PipelineTiming broken;
        broken.valid=true; broken.capture_ready_ns=100;
        broken.frame_push_begin_ns=101; broken.inference_pop_end_ns=102;
        broken.result_push_begin_ns=reversed?99:0; broken.supervisor_pop_end_ns=104;
        invalid.pipeline_timing=broken; frame.pipeline_timing=broken;
        SupervisorRnsJoinTestAccess::finalize_timing(invalid,frame);
        CHECK(!invalid.pipeline_timing.valid && !frame.pipeline_timing.valid);
        CHECK(invalid.pipeline_timing.prepublish_ns>0);
        CHECK(invalid.pipeline_timing.result_push_begin_ns==broken.result_push_begin_ns);
        same_timing(invalid.pipeline_timing,frame.pipeline_timing);
    }
}
#ifndef PERCEPTION_SUPERVISOR_TIMING_TEST_ONLY
// Exact opt-in only, Gemini only (moved from the retired legacy-mode test).
void test_parallel_depth_sample_env_gate() {
    constexpr const char* kEnvName = "PERCEPTION_PARALLEL_DEPTH_SAMPLE";
    unsetenv(kEnvName);
    CHECK(!SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("orbbec_gemini338le"));
    CHECK(setenv(kEnvName, "1", 1) == 0);
    CHECK(SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("orbbec_gemini338le"));
    CHECK(!SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("d435"));
    CHECK(setenv(kEnvName, "2", 1) == 0);  // shared split (2026-10-08)
    CHECK(SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("orbbec_gemini338le"));
    CHECK(setenv(kEnvName, "alt", 1) == 0);  // diagnostic alternation (2026-10-08)
    CHECK(SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("orbbec_gemini338le"));
    CHECK(!SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("d435"));
    for (const char* invalid : {"0", "true", "01", " 1", "1 ", "3", "2 ", "ALT", "alt "}) {
        CHECK(setenv(kEnvName, invalid, 1) == 0);
        CHECK(!SupervisorRnsJoinTestAccess::parallel_depth_sample_enabled("orbbec_gemini338le"));
    }
    unsetenv(kEnvName);
}
// A frame whose depth intrinsics do not describe the delivered raster is fatal:
// the Supervisor marks the failure and requests shutdown instead of publishing
// geometry computed with the wrong camera model (moved from the retired test).
// B1 (GPT answer 1.2, 2026-10-04): the velocity windows run on the capture-monotonic stamp.
// Ten frames 40 ms apart on the capture clock are fed back to back in wall time; a static object
// becomes "static" (valid) once the 0.25 s window is full. A wall-clock time base would see
// microseconds between these calls and never fill the window. A capture gap above dt_max (0.5 s)
// restarts the window. Default estimator settings (init() is not run here).
void test_velocity_uses_capture_time() {
    Supervisor s;
    FusedDetection det;
    det.track_id = 7; det.class_name = "person"; det.stable_frames = 20;
    det.depth_confidence = 1.0F; det.fusion_quality = "good";
    det.centroid_3d[0] = 0.5F; det.centroid_3d[1] = 0.0F; det.centroid_3d[2] = 3.0F;
    geometry::CaptureStamp capture;
    capture.clock_epoch = 7;
    std::vector<std::string> statuses;
    for (std::int64_t k = 0; k < 10; ++k) {
        capture.capture_mono_ns = 5000000000LL + k * 40000000LL;
        std::vector<FusedDetection> fused{det};
        SupervisorRnsJoinTestAccess::update_velocity(s, fused, capture, false);
        CHECK(fused[0].velocity_frame == FusedDetection::VelocityFrame::Camera);
        statuses.push_back(fused[0].velocity_status);
        if (fused[0].velocity_status == "static") CHECK(fused[0].velocity_valid);
    }
    // Window opened at 5.00 s and closes at the first sample >= 5.25 s (k = 7, 5.28 s).
    CHECK(statuses[6] == "warming_up");
    CHECK(statuses[7] == "static" && statuses[9] == "static");
    capture.capture_mono_ns += 600000000LL;
    std::vector<FusedDetection> fused{det};
    SupervisorRnsJoinTestAccess::update_velocity(s, fused, capture, false);
    CHECK(fused[0].velocity_status == "warming_up" && !fused[0].velocity_valid);
}
// Valid Gemini intrinsics reach the frame; a non-finite focal length or a size other than the
// camera output fails closed (fatal and shutdown).
void test_gemini_runtime_geometry_fails_closed() {
    const auto path = std::filesystem::temp_directory_path() / "perception_rns_geometry_camera.yaml";
    {
        std::ofstream out(path);
        out << "hevc:\n  enable: true\n  sink: file_rtsp\n  rtsp_listen: rtsp://0.0.0.0:8555/rgb\n"
               "  width: 1920\n  height: 1080\n  fps: 30\n  bitrate: 4000000\n  codec: nvv4l2h265enc\n  gop: 30\n"
               "camera:\n  driver: orbbec_gemini338le\n  serial: CHL5663000DS\n"
               "  color:\n    width: 1280\n    height: 720\n    fps: 30\n    device_format: mjpg\n"
               "  depth:\n    width: 848\n    height: 530\n    fps: 30\n    device_format: y16\n"
               "  alignment_mode: hw_d2c\n  align_to_color: true\n"
               "  output:\n    width: 1920\n    height: 1080\n    fps: 30\n";
        // dual_model.enabled is required and true (level-3 trim): the bundled sealed contracts.
        const auto root = std::filesystem::absolute(std::filesystem::path(__FILE__)).parent_path().parent_path();
        out << "dual_model:\n  enabled: true\n"
            << "  runtime_config_path: \"" << (root / "config/dual_model_runtime_config.json").string() << "\"\n"
            << "  class_schema_path: \"" << (root / "config/class_schema_94.json").string() << "\"\n";
    }
    yolo::Config config;
    CHECK(config.load(path.string()));
    CHECK(config.camera_driver() == "orbbec_gemini338le");
    CHECK(config.camera_output_width() == 1920 && config.camera_output_height() == 1080);

    yolo::DetectionResult result;
    result.frame_id = 7;
    result.frame = cv::Mat(1080, 1920, CV_8UC3);
    result.depth = cv::Mat(1080, 1920, CV_16UC1);
    result.depth_intrinsics = yolo::DepthIntrinsics{917.719F, 917.714F, 961.232F, 540.977F, 1920, 1080};
    {
        Supervisor s;
        PerceptionFrame frame;
        CHECK(SupervisorRnsJoinTestAccess::apply_geometry(s, config, result, frame));
        CHECK(frame.camera_intrinsics_valid && frame.camera_intrinsics.fx == result.depth_intrinsics.fx);
        CHECK(!s.fatal_failure_detected() && !s.shutdown_requested());
    }
    auto bad_focal = result;
    bad_focal.depth_intrinsics.fx = std::numeric_limits<float>::infinity();
    auto other_size = result;
    other_size.depth_intrinsics.width = 1280;
    other_size.depth_intrinsics.cx = 640.0F;
    for (const auto* invalid : {&bad_focal, &other_size}) {
        Supervisor s;
        PerceptionFrame frame;
        CHECK(!SupervisorRnsJoinTestAccess::apply_geometry(s, config, *invalid, frame));
        CHECK(!frame.camera_intrinsics_valid);
        CHECK(s.fatal_failure_detected() && s.shutdown_requested());
    }
    std::filesystem::remove(path);
}
std::int64_t now(void*) noexcept { return 1000000000; }
SupervisorRnsContext context(const std::shared_ptr<int>& owner) {
    SupervisorRnsContext c;
    c.native_mailbox=std::make_shared<native_depth::Mailbox>(64);
    c.depth_arrivals=std::make_shared<SourceJournal>(SourceKind::DepthArrival,8,SourceClock{nullptr,&now});
    c.inference_completions=std::make_shared<SourceJournal>(SourceKind::InferenceComplete,8,SourceClock{nullptr,&now});
    c.clock_epoch=7; c.callback_owner=owner;
    c.now_ns=[] { return std::int64_t{1000000000}; };
    c.capture_stamp=[](const native_depth::SourceId&,geometry::CaptureStamp&) { return false; };
    c.objects_ready=[](const yolo::DetectionResult&,const geometry::CaptureStamp&,const Objects&,const FusedObjectsReport&) { return false; };
    return c;
}
#endif
}
int main() {
    try {
        test_prepublish_timing();
#ifndef PERCEPTION_SUPERVISOR_TIMING_TEST_ONLY
        {
            Supervisor s; std::atomic<bool> completed{false};
            std::thread owned([&] { completed.store(true,std::memory_order_release); });
            const auto failed_launcher=+[](std::thread&) -> std::future<void> {
                throw std::system_error(std::make_error_code(std::errc::resource_unavailable_try_again));
            };
            CHECK(!SupervisorRnsJoinTestAccess::join(s,owned,failed_launcher));
            CHECK(!owned.joinable() && completed.load(std::memory_order_acquire));
            std::thread absent;
            CHECK(SupervisorRnsJoinTestAccess::join(s,absent,failed_launcher));
        }
        auto owner=std::make_shared<int>(1); std::weak_ptr<int> weak=owner;
        {
            Supervisor s;
            CHECK(!s.configure_rns({})); CHECK(!s.runtime_audit().available);
            CHECK(!s.jit_calibration_complete() && s.jit_frozen_delay_us()==-1);
            auto c=context(owner); CHECK(c.valid()); CHECK(s.configure_rns(c));
            CHECK(!s.configure_rns(c));
            owner.reset(); c={}; CHECK(!weak.expired());
            // Default Config is not the explicit native800 R6 setup. Rejection
            // happens before module construction, and cannot re-open config.
            yolo::Config config;
            CHECK(!s.init(config)); CHECK(!s.init(config));
            CHECK(!s.configure_rns(context(std::make_shared<int>(2))));
            CHECK(!s.start()); // initialized=false; no engine/camera start
            s.stop(); CHECK(!s.runtime_audit().available);
        }
        CHECK(weak.expired());
        {
            // R6-only: without configure_rns there is no output at all, so init refuses.
            Supervisor unwired; yolo::Config config;
            CHECK(!unwired.init(config)); CHECK(!unwired.start());
            CHECK(unwired.configure_rns(context(std::make_shared<int>(3))));
        }
        test_parallel_depth_sample_env_gate();
        test_gemini_runtime_geometry_fails_closed();
        test_velocity_uses_capture_time();
        {
            yolo::Config config; std::string error;
            auto c=context(std::make_shared<int>(4));
            CHECK(!create_rgbd_capture(config,nullptr,nullptr,error,c.native_mailbox,c.depth_arrivals.get()));
            CHECK(error=="native R6 hooks require an explicit Gemini native mailbox");
            CHECK(!create_rgbd_capture(config,nullptr,nullptr,error,nullptr,c.depth_arrivals.get()));
            CHECK(error=="native R6 hooks require an explicit Gemini native mailbox");
        }
#endif
        std::cout << "{\"status\":\"PASS\",\"checks\":" << checks
                  << ",\"prepublish_metadata\":true,\"module_starts\":0,\"device\":false,\"model_execution\":false,\"network\":false}\n";
        return 0;
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
