/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: bench_calib_main.cpp
 * Brief: perception_bench_calib, the bench extrinsic tool (19 section 7.1 step 1, reply Q11/Q13).
 *
 * Description:
 * Isolated bench use only; never installed as a service and never writes configuration in place.
 * Usage (inside the same private mount namespace as the service):
 *   perception_bench_calib <output_dir> <frames>
 * Reads /opt/xbrain_v6/data/run/resolved/perception.yaml + MANIFEST.json exactly like the
 * service (camera identity, native 640x400 profile), captures <frames> native
 * depth frames (no colour image is written), fits the floor per frame and
 * writes into <output_dir> (must not exist):
 *   calib_record.yaml   scope bench, base_frame bench (see bench_calibration.hpp)
 *   config_snippet.yaml perception.extrinsic_base_cam transform + record sha256
 *   report.json         per-run numbers incl. tool options and failures
 * Exit: 0 record written; 2 usage; 3 calibration rejected (report written);
 *       78 configuration refused; 69 camera unavailable.
 */
#include "service/bench_calibration.hpp"
#include "service/calibration_record.hpp"
#include "service/service_config.hpp"

#include "common/config.hpp"
#include "common/logger.hpp"
#include "common/threadsafe_queue.hpp"
#include "modules/native_depth.hpp"
#include "modules/orbbec_capture.hpp"
#include "modules/orbbec_native_rays.hpp"

#include <sys/stat.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {
using namespace perception;
namespace nd = perception::native_depth;
constexpr std::size_t kPixels = 640U * 400U;
constexpr double kPi = 3.14159265358979323846;

// 17 significant digits: the double round-trips exactly through the record and the snippet.
std::string num(double v) {
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}
// Wall-clock UTC is only a human-readable label in the record (id and created_utc), never a
// timestamp any computation uses.
std::string utc_now() {
    const auto t = std::time(nullptr);
    std::tm tm{};
    ::gmtime_r(&t, &tm);
    char b[32];
    std::strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return b;
}
bool write_file(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << text;
    return static_cast<bool>(out);
}

// Tool thresholds, stricter than the service's per-frame fit: 1 cm inliers and RMS, at least
// 2000 inliers covering 10 cm of x/y spread, 256 hypotheses (time is no concern at the bench),
// and across >= 10 passing frames the normal within 0.5 degree and the height within 5 mm.
// They are written into report.json so every record says how strictly it was accepted.
service::BenchCalibrationOptions tool_options() {
    service::BenchCalibrationOptions o;
    o.near_min_m = 0.4;
    o.near_max_m = 3.0;
    o.min_below_m = 0.1;
    o.fit.inlier_distance_m = 0.01;
    o.fit.min_inlier_fraction = 0.5;
    o.fit.max_inlier_rms_m = 0.01;
    o.fit.min_xy_stddev_m = 0.1;
    o.fit.min_up_dot = 0.5;
    o.fit.max_prior_angle_rad = 50.0 * kPi / 180.0;
    o.fit.min_inliers = 2000;
    o.fit.hypotheses = 256;
    o.fit.seed = 0x243f6a8885a308d3ULL;
    o.max_normal_spread_rad = 0.5 * kPi / 180.0;
    o.max_height_spread_m = 0.005;
    o.min_frames = 10;
    return o;
}
std::string options_json(const service::BenchCalibrationOptions& o) {
    std::ostringstream s;
    s << "{\"near_min_m\":" << o.near_min_m << ",\"near_max_m\":" << o.near_max_m << ",\"min_below_m\":" << o.min_below_m
      << ",\"inlier_distance_m\":" << o.fit.inlier_distance_m << ",\"min_inlier_fraction\":" << o.fit.min_inlier_fraction
      << ",\"max_inlier_rms_m\":" << o.fit.max_inlier_rms_m << ",\"min_xy_stddev_m\":" << o.fit.min_xy_stddev_m
      << ",\"min_up_dot\":" << o.fit.min_up_dot << ",\"max_prior_angle_deg\":" << o.fit.max_prior_angle_rad * 180.0 / kPi
      << ",\"min_inliers\":" << o.fit.min_inliers << ",\"hypotheses\":" << o.fit.hypotheses
      << ",\"max_normal_spread_deg\":" << o.max_normal_spread_rad * 180.0 / kPi
      << ",\"max_height_spread_m\":" << o.max_height_spread_m << ",\"min_frames\":" << o.min_frames << "}";
    return s.str();
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: perception_bench_calib <output_dir(new)> <frames 10..300>\n";
        return 2;
    }
    // A new absolute output directory only: the tool never overwrites an earlier record and
    // never writes configuration in place.
    const std::string out_dir = argv[1];
    const int frames = std::atoi(argv[2]);
    struct stat st {};
    if (frames < 10 || frames > 300 || out_dir.empty() || out_dir.front() != '/' || ::stat(out_dir.c_str(), &st) == 0) {
        std::cerr << "output_dir must be a new absolute path and frames in [10, 300]\n";
        return 2;
    }
    // The same resolved configuration the service reads: camera identity and native profile.
    service::ServiceConfig cfg;
    try {
        service::LoadInputs inputs;
        inputs.host = service::read_host_identity();
        cfg = service::load_service_config(inputs);
    } catch (const service::StartupCheckError& e) {
        std::cerr << "PERCEPTION_STARTUP_REFUSED check=" << e.check() << " key=" << e.key() << " reason=\"" << e.what() << "\"\n";
        return 78;
    }
    yolo::Config config;
    if (!config.load_from_node(cfg.pipeline, cfg.resolved_path)) {
        std::cerr << "pipeline configuration rejected\n";
        return 78;
    }
    yolo::Logger::instance().set_level(yolo::LogLevel::WARN);
    const auto serial = cfg.pipeline["camera"]["serial"].as<std::string>();

    auto queue = std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(4);
    auto mailbox = std::make_shared<nd::Mailbox>(kPixels);
    nd::Frame frame(kPixels);
    std::unique_ptr<service::BenchCalibrator> calibrator;
    std::string failures_json = "[";
    std::size_t taken = 0, fit_failures = 0;
    nd::Metadata first;
    {
        OrbbecCapture capture(config, queue, nullptr, mailbox);
        if (!capture.start()) {
            std::cerr << "camera start failed\n";
            return 69;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30 + frames);
        while (taken < static_cast<std::size_t>(frames) && std::chrono::steady_clock::now() < deadline) {
            yolo::FrameData dropped;
            while (queue->try_pop(dropped)) {}  // aligned colour frames are not used or stored
            const auto before = mailbox->accepted_count();
            if (mailbox->try_take(frame) != nd::Take::Taken) {
                (void)mailbox->wait_for_accepted_after(before, std::chrono::milliseconds(50));
                continue;
            }
            // The calibrator is built from the first frame's native intrinsics; rays only, in the
            // optical frame (identity rotation). A later frame with other metadata means the
            // stream changed mid-capture, so the run is abandoned rather than mixed.
            if (!calibrator) {
                first = frame.metadata();
                geometry::ProjectionBinding b;
                b.intrinsics = first.intrinsics;
                b.distortion = first.distortion;
                b.unit_m = first.unit_m;
                b.R_base_camera = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // rays only: optical frame
                geometry::OrbbecNativeRays rays(b);
                calibrator = std::make_unique<service::BenchCalibrator>(rays.rays(), tool_options());
            } else if (frame.metadata().intrinsics.fx != first.intrinsics.fx || frame.metadata().unit_m != first.unit_m) {
                std::cerr << "native metadata changed during capture\n";
                capture.stop();
                return 69;
            }
            ++taken;
            const auto fit = calibrator->add_frame(frame.pixels(), frame.pixel_count(), frame.metadata().unit_m);
            // Failed frames are counted; the first 20 are sampled into the report for diagnosis.
            if (fit.status != geometry::FitStatus::Pass) {
                ++fit_failures;
                if (fit_failures <= 20)
                    failures_json += std::string(fit_failures > 1 ? "," : "") + "{\"sequence\":" +
                        std::to_string(frame.metadata().source.sequence) + ",\"status\":" +
                        std::to_string(static_cast<int>(fit.status)) + ",\"candidates\":" + std::to_string(fit.candidates) +
                        ",\"inliers\":" + std::to_string(fit.inliers) + ",\"rms_m\":" + num(fit.inlier_rms_m) + "}";
            }
        }
        capture.stop();
    }
    failures_json += "]";
    if (::mkdir(out_dir.c_str(), 0755) != 0) {
        std::cerr << "cannot create output_dir\n";
        return 2;
    }
    // report.json is written for every completed capture, accepted or rejected, so a rejection
    // can be diagnosed from the numbers (scope bench, never a vehicle calibration).
    const auto result = calibrator ? calibrator->finish() : service::BenchCalibrationResult{};
    std::ostringstream report;
    report << "{\"tool\":\"perception_bench_calib\",\"version\":\"1\",\"camera_serial\":\"" << serial << "\""
           << ",\"frames_requested\":" << frames << ",\"frames_taken\":" << taken << ",\"fit_failures\":" << fit_failures
           << ",\"fit_failure_samples\":" << failures_json
           << ",\"native\":{\"fx\":" << num(first.intrinsics.fx) << ",\"fy\":" << num(first.intrinsics.fy)
           << ",\"cx\":" << num(first.intrinsics.cx) << ",\"cy\":" << num(first.intrinsics.cy)
           << ",\"width\":" << first.intrinsics.width << ",\"height\":" << first.intrinsics.height
           << ",\"distortion_model\":" << first.distortion.model << ",\"unit_m\":" << num(first.unit_m) << "}"
           << ",\"options\":" << options_json(tool_options())
           << ",\"ok\":" << (result.ok ? "true" : "false") << ",\"failure\":\"" << result.failure << "\""
           << ",\"frames_passed\":" << result.frames_passed << ",\"inliers_total\":" << result.inliers_total
           << ",\"worst_inlier_fraction\":" << num(result.worst_inlier_fraction)
           << ",\"worst_inlier_rms_m\":" << num(result.worst_inlier_rms_m)
           << ",\"height_m\":" << num(result.height_m) << ",\"pitch_down_deg\":" << num(result.pitch_down_deg)
           << ",\"roll_deg\":" << num(result.roll_deg) << ",\"normal_spread_deg\":" << num(result.normal_spread_rad * 180.0 / kPi)
           << ",\"height_spread_m\":" << num(result.height_spread_m)
           << ",\"scope\":\"bench\",\"vehicle_calibration\":false}\n";
    (void)write_file(out_dir + "/report.json", report.str());
    std::cout << report.str();
    if (!result.ok) return 3;

    // Accepted: the calibration record (scope bench; yaw and x/y are fixed by the bench axis
    // definition, not measured) with the worst per-frame residuals.
    service::CalibrationRecord record;
    record.schema_version = 1;
    record.record_id = "bench-" + serial + "-" + utc_now().substr(0, 10);
    record.scope = service::CalibrationScope::Bench;
    record.created_utc = utc_now();
    record.camera_driver = "orbbec_gemini338le";
    record.camera_serial = serial;
    record.depth_stream = service::kCalibrationDepthStream;
    record.base_frame = "bench";
    record.base_origin = "camera foot point on the fitted floor; z = floor normal up; x = optical axis projected on the floor";
    record.translation = result.translation;
    record.rotation_xyzw = result.rotation_xyzw;
    record.method_plane = "depth_plane_fit";
    record.method_yaw = "bench_axis_definition";
    record.method_xy = "bench_axis_definition";
    record.plane_frames = static_cast<long>(result.frames_passed);
    record.plane_inliers = result.inliers_total;
    record.plane_inlier_fraction = result.worst_inlier_fraction;
    record.plane_fit_inlier_rms_m = result.worst_inlier_rms_m;
    record.tool_name = "perception_bench_calib";
    record.tool_version = "1";
    const auto bytes = service::format_calibration_record(record);
    (void)service::parse_calibration_record(bytes);  // the tool never writes a record the verifier rejects
    const auto record_path = out_dir + "/calib_record.yaml";
    if (!write_file(record_path, bytes)) return 2;
    // A configuration snippet to paste by hand (with the record's path and SHA-256 for the
    // service's verifier); residual_max_m is left null on purpose, see the comment it carries.
    const auto& t = result.translation;
    const auto& q = result.rotation_xyzw;
    const std::string snippet = "  extrinsic_base_cam:\n    translation:      [" + num(t[0]) + ", " + num(t[1]) + ", " + num(t[2]) +
        "]\n    rotation_xyzw:    [" + num(q[0]) + ", " + num(q[1]) + ", " + num(q[2]) + ", " + num(q[3]) +
        "]\n    calib_record:     " + record_path + "\n    calib_sha256:     " + yolo::sha256_hex(bytes) +
        "\n    residual_max_m:   null  # choose explicitly; a bench record never certifies (BenchScope)\n";
    if (!write_file(out_dir + "/config_snippet.yaml", snippet)) return 2;
    std::cout << "BENCH_CALIBRATION_RECORD_WRITTEN path=" << record_path << " sha256=" << yolo::sha256_hex(bytes) << std::endl;
    return 0;
}
