/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_calibration_record.cpp
 * Brief: Host test of the calibration record schema v1 and the extrinsic_calibrated verdict.
 *
 * Description:
 * What it checks. 19 section 7.3 and reply 2026-09-17 Q11/Q12/Q13: records round-trip through
 * format and parse; every verdict code is reached in its order (no transform, transform only,
 * record unreadable, sha mismatch, schema invalid, camera mismatch, extrinsic mismatch by one ulp,
 * residual exceeded, bench scope, verified with rms equal to the threshold); the parser rejects
 * comments, quotes, tabs, a pixel reprojection error in place of the plane RMS, and a null serial.
 * No camera, SDK or network.
 */

#include "service/calibration_record.hpp"

#include "common/config.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace perception::service;

namespace {
int checks = 0, failures = 0, rejections = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

const std::string kSerial = "CHL5663000DS";

CalibrationRecord vehicle_record() {
    CalibrationRecord r;
    r.schema_version = 1;
    r.record_id = "veh-01.2026-09-18_a";
    r.scope = CalibrationScope::Vehicle;
    r.created_utc = "2026-09-18T10:20:30Z";
    r.camera_driver = "orbbec_gemini338le";
    r.camera_serial = kSerial;
    r.depth_stream = "640x400_y16_30";
    r.base_frame = "base_link";
    r.base_origin = "body geometric centre projected to ground (quadruped substitute, 19 section 7.1)";
    r.translation = {0.35, -0.012345678901234567, 0.645};
    // pitch-down rotation, unit to double precision
    const double half = 0.5 * 15.0 * 3.14159265358979323846 / 180.0;
    r.rotation_xyzw = {0.0, std::sin(half), 0.0, std::cos(half)};
    r.method_plane = "depth_plane_fit";
    r.method_yaw = "straight_edge_reference";
    r.method_xy = "tape_measure";
    r.plane_frames = 30;
    r.plane_inliers = 123456;
    r.plane_inlier_fraction = 0.8125;
    r.plane_fit_inlier_rms_m = 0.0041;
    r.tool_name = "perception_calib_tool";
    r.tool_version = "1";
    return r;
}

ExtrinsicConfig config_for(const CalibrationRecord& r, const std::string& bytes, const std::string& path, double residual) {
    ExtrinsicConfig e;
    e.mode = ExtrinsicMode::TransformConfigured;
    e.translation = r.translation;
    e.rotation_xyzw = r.rotation_xyzw;
    e.calib_record = path;
    e.calib_sha256 = yolo::sha256_hex(bytes);
    e.residual_max_m = residual;
    return e;
}

CalibrationFileReader memory_reader(const std::string& bytes) {
    return [bytes](const std::string&, std::size_t, std::string& out, std::string&) {
        out = bytes;
        return true;
    };
}

std::string replace_once(std::string s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    if (at == std::string::npos) { std::cerr << "pattern missing: " << from << "\n"; std::exit(2); }
    return s.replace(at, from.size(), to);
}

void expect_parse_reject(const std::string& name, const std::string& bytes, const std::string& path_contains) {
    try {
        (void)parse_calibration_record(bytes);
        ++checks; ++failures;
        std::cerr << "FAIL accepted: " << name << "\n";
    } catch (const std::exception& ex) {
        ++checks; ++rejections;
        if (std::string(ex.what()).find(path_contains) == std::string::npos) {
            ++failures;
            std::cerr << "FAIL wrong rejection for " << name << ": " << ex.what() << "\n";
        }
    }
}

void expect_code(const std::string& name, const CalibrationVerdict& v, CalibrationCode code) {
    ++checks;
    if (v.code() != code || v.calibrated() != (code == CalibrationCode::Verified)) {
        ++failures;
        std::cerr << "FAIL " << name << ": got " << calibration_code_name(v.code()) << " calibrated=" << v.calibrated()
                  << " detail=" << v.detail() << "\n";
    }
    if (code != CalibrationCode::Verified) ++rejections;
}

// A formatted record parses back to the same values and formats to the same bytes; block-style
// vectors parse to the same doubles.
void test_round_trip() {
    const auto r = vehicle_record();
    const auto bytes = format_calibration_record(r);
    const auto p = parse_calibration_record(bytes);
    CHECK(format_calibration_record(p) == bytes);
    CHECK(p.translation == r.translation && p.rotation_xyzw == r.rotation_xyzw);
    CHECK(p.plane_fit_inlier_rms_m == r.plane_fit_inlier_rms_m && p.plane_inlier_fraction == r.plane_inlier_fraction);
    CHECK(p.camera_serial == kSerial && p.scope == CalibrationScope::Vehicle && p.record_id == r.record_id);
    CHECK(p.created_utc == r.created_utc && p.base_origin == r.base_origin);
    // block-style vectors parse to the same doubles
    auto block = replace_once(bytes, "    translation: [0.34999999999999998, -0.012345678901234567, 0.64500000000000002]\n",
                             "    translation:\n    - 0.34999999999999998\n    - -0.012345678901234567\n    - 0.64500000000000002\n");
    CHECK(parse_calibration_record(block).translation == r.translation);
}

// Each verdict has a case: verified (RMS equal to the limit included), no transform, transform
// only, partial config, unreadable file (missing, relative, directory, oversize), SHA mismatch,
// schema invalid, camera or extrinsic mismatch (one ulp), residual exceeded and bench scope.
void test_verdicts(const fs::path& dir) {
    const auto r = vehicle_record();
    const auto bytes = format_calibration_record(r);
    const std::string path = (dir / "record.yaml").string();
    { std::ofstream(path, std::ios::binary) << bytes; }

    // verified from a real file (reader = production read_calibration_file)
    auto e = config_for(r, bytes, path, 0.005);
    auto v = verify_calibration(e, kSerial);
    expect_code("verified", v, CalibrationCode::Verified);
    CHECK(v.record_sha256() == *e.calib_sha256 && v.record_id() == r.record_id && v.inlier_rms() && *v.inlier_rms() == 0.0041);
    // boundary: rms == threshold is accepted (<=)
    expect_code("rms equals threshold", verify_calibration(config_for(r, bytes, path, 0.0041), kSerial), CalibrationCode::Verified);

    ExtrinsicConfig absent;
    expect_code("no transform", verify_calibration(absent, kSerial), CalibrationCode::NoTransform);
    auto only = e;
    only.calib_record.reset(); only.calib_sha256.reset(); only.residual_max_m.reset();
    expect_code("transform only", verify_calibration(only, kSerial), CalibrationCode::TransformOnly);
    auto partial = e;
    partial.residual_max_m.reset();
    expect_code("partial triple", verify_calibration(partial, kSerial), CalibrationCode::SchemaInvalid);

    auto missing = e;
    missing.calib_record = (dir / "absent.yaml").string();
    expect_code("missing file", verify_calibration(missing, kSerial), CalibrationCode::RecordUnreadable);
    // the relative name resolves to the real record from this working directory:
    // only the absolute-path rule may reject it
    auto relative = e;
    relative.calib_record = "record.yaml";
    const auto previous = fs::current_path();
    fs::current_path(dir);
    expect_code("relative path", verify_calibration(relative, kSerial), CalibrationCode::RecordUnreadable);
    fs::current_path(previous);
    auto directory = e;
    directory.calib_record = dir.string();
    expect_code("directory", verify_calibration(directory, kSerial), CalibrationCode::RecordUnreadable);
    const std::string big = (dir / "big.yaml").string();
    { std::ofstream(big, std::ios::binary) << bytes << std::string(kCalibrationRecordMaxBytes, '\n'); }
    auto oversize = e;
    oversize.calib_record = big;
    expect_code("oversize file", verify_calibration(oversize, kSerial), CalibrationCode::RecordUnreadable);

    // tampered bytes (one trailing newline) with the registered sha unchanged
    expect_code("sha mismatch", verify_calibration(e, kSerial, memory_reader(bytes + "\n")), CalibrationCode::ShaMismatch);
    // registered sha of an invalid document: schema check still runs on the SAME bytes
    const auto broken = replace_once(bytes, "  scope: vehicle\n", "  scope: fleet\n");
    expect_code("schema invalid", verify_calibration(config_for(r, broken, path, 0.005), kSerial, memory_reader(broken)),
               CalibrationCode::SchemaInvalid);
    expect_code("camera mismatch", verify_calibration(e, "CHL0000000XX", memory_reader(bytes)), CalibrationCode::CameraMismatch);

    auto moved = e;
    (*moved.translation)[1] = std::nextafter((*moved.translation)[1], 1.0);
    expect_code("translation one ulp off", verify_calibration(moved, kSerial, memory_reader(bytes)), CalibrationCode::ExtrinsicMismatch);
    auto turned = e;
    (*turned.rotation_xyzw)[3] = std::nextafter((*turned.rotation_xyzw)[3], 0.0);
    expect_code("rotation one ulp off", verify_calibration(turned, kSerial, memory_reader(bytes)), CalibrationCode::ExtrinsicMismatch);

    expect_code("residual exceeded", verify_calibration(config_for(r, bytes, path, std::nextafter(0.0041, 0.0)), kSerial,
                                                      memory_reader(bytes)), CalibrationCode::ResidualExceeded);

    auto bench = r;
    bench.scope = CalibrationScope::Bench;
    bench.base_frame = "bench";
    bench.method_yaw = "bench_axis_definition";
    bench.method_xy = "bench_axis_definition";
    const auto bench_bytes = format_calibration_record(bench);
    expect_code("bench scope", verify_calibration(config_for(bench, bench_bytes, path, 0.005), kSerial, memory_reader(bench_bytes)),
               CalibrationCode::BenchScope);

    auto no_reader = e;
    expect_code("null reader", verify_calibration(no_reader, kSerial, CalibrationFileReader{}), CalibrationCode::RecordUnreadable);
}

// The strict parser refuses unknown, missing or duplicate keys, out-of-range or wrong-kind values,
// comments, quotes, tabs, non-ASCII text and oversize files, naming the field.
void test_parser_rejections() {
    const auto bytes = format_calibration_record(vehicle_record());
    const auto rep = [&](const std::string& from, const std::string& to) { return replace_once(bytes, from, to); };
    expect_parse_reject("unknown top-level", bytes + "extra: 1\n", "extra");
    expect_parse_reject("unknown record key", rep("  tool:\n", "  note: x\n  tool:\n"), "calib_record.note");
    expect_parse_reject("missing key", rep("  created_utc: 2026-09-18T10:20:30Z\n", ""), "calib_record.created_utc");
    expect_parse_reject("duplicate key", rep("  scope: vehicle\n", "  scope: vehicle\n  scope: bench\n"), "duplicate");
    expect_parse_reject("schema 2", rep("  schema_version: 1\n", "  schema_version: 2\n"), "schema_version");
    expect_parse_reject("record id spaces", rep("  record_id: veh-01.2026-09-18_a\n", "  record_id: veh 01\n"), "record_id");
    expect_parse_reject("scope", rep("  scope: vehicle\n", "  scope: fleet\n"), "scope");
    expect_parse_reject("created_utc format", rep("2026-09-18T10:20:30Z", "2026-09-18 10:20:30"), "created_utc");
    expect_parse_reject("driver", rep("    driver: orbbec_gemini338le\n", "    driver: realsense_d435\n"), "camera.driver");
    expect_parse_reject("depth stream", rep("    depth_stream: 640x400_y16_30\n", "    depth_stream: 848x530_y16_30\n"), "depth_stream");
    expect_parse_reject("vehicle with bench frame", rep("    name: base_link\n", "    name: bench\n"), "base_frame.name");
    expect_parse_reject("translation length", rep("[0.34999999999999998, -0.012345678901234567, 0.64500000000000002]",
                                                "[0.35, 0.645]"), "translation");
    expect_parse_reject("translation nan", rep("0.64500000000000002]", "nan]"), "translation");
    expect_parse_reject("translation beyond 10 m", rep("0.64500000000000002]", "10.5]"), "translation");
    expect_parse_reject("non-unit quaternion", rep("    rotation_xyzw: [0, ", "    rotation_xyzw: [0.001, "), "rotation_xyzw");
    expect_parse_reject("bench yaw in vehicle", rep("    yaw: straight_edge_reference\n", "    yaw: bench_axis_definition\n"), "method.yaw");
    expect_parse_reject("bench xy in vehicle", rep("    xy: tape_measure\n", "    xy: bench_axis_definition\n"), "method.xy");
    expect_parse_reject("unknown plane method", rep("    plane: depth_plane_fit\n", "    plane: reprojection\n"), "method.plane");
    expect_parse_reject("inlier fraction zero", rep("    inlier_fraction: 0.8125\n", "    inlier_fraction: 0\n"), "inlier_fraction");
    expect_parse_reject("inlier fraction > 1", rep("    inlier_fraction: 0.8125\n", "    inlier_fraction: 1.5\n"), "inlier_fraction");
    expect_parse_reject("negative rms", rep("    plane_fit_inlier_rms_m: 0.0041000000000000003\n",
                                          "    plane_fit_inlier_rms_m: -0.001\n"), "plane_fit_inlier_rms_m");
    expect_parse_reject("pixel reprojection is not rms", rep("    plane_fit_inlier_rms_m: 0.0041000000000000003\n",
                                                           "    reprojection_error_px: 0.3\n"), "plane_fit");
    expect_parse_reject("zero frames", rep("    frames: 30\n", "    frames: 0\n"), "plane_fit.frames");
    expect_parse_reject("two inliers", rep("    inliers: 123456\n", "    inliers: 2\n"), "plane_fit.inliers");
    expect_parse_reject("comment", rep("    version: 1\n", "    version: 1 # note\n"), "comments");
    expect_parse_reject("quoted value", rep("    version: 1\n", "    version: \"1\"\n"), "quotes");
    expect_parse_reject("tab indent", rep("    version: 1\n", "\tversion: 1\n"), "ASCII");
    expect_parse_reject("non ascii", rep("    name: perception_calib_tool\n", "    name: perception_calib_tool\xc3\xa9\n"), "ASCII");
    expect_parse_reject("sequence elsewhere", rep("    frames: 30\n", "    frames:\n    - 30\n"), "sequence");
    expect_parse_reject("oversize", bytes + std::string(kCalibrationRecordMaxBytes, '\n'), "64 KiB");
    expect_parse_reject("null serial", rep("    serial: CHL5663000DS\n", "    serial: null\n"), "camera.serial");
}
}  // namespace

int main() {
    char tmpl[] = "/tmp/calib_record_test.XXXXXX";
    const fs::path dir = fs::canonical(::mkdtemp(tmpl));
    try {
        test_round_trip();
        test_verdicts(dir);
        test_parser_rejections();
    } catch (const std::exception& ex) {
        ++failures;
        std::cerr << "FAIL unexpected exception: " << ex.what() << "\n";
    }
    std::error_code ec;
    fs::remove_all(dir, ec);
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"rejections\":" << rejections << ",\"scope\":\"calibration_record_host\",\"camera\":false"
              << ",\"physical_calibration_verified\":false}" << std::endl;
    return failures ? 1 : 0;
}
