/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_geometry_config.cpp
 * Brief: Unit tests for admitting the resolved geometry YAML and wiring it into synthetic kernels.
 *
 * Description:
 * What it checks. Transforms and units in flow, block and indentless vectors; literal zeros and
 * the ignored null fallback; calibration record keys (status not configurable); missing, null and
 * old-schema entries; scalar types, finiteness and unsigned values; vector shapes, the quaternion
 * and the prior; the Objects extrinsic equal to the setup's; numeric domains and capacities;
 * native metadata admission; strict resolved YAML; a synthetic geometry -> grid -> Profile run.
 * No SDK, file-based calibration verification, camera, transport, or production acceptance.
 */

#include "common/rns_geometry_config.hpp"
#include "output/rns_normal_profile.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace cfg = perception::config;
namespace geom = perception::geometry;
namespace nd = perception::native_depth;
namespace rns = perception::rns;
constexpr double pi = 3.1415926535897932384626433832795;
std::size_t checks = 0, groups = 0, rejected = 0;
std::string current_group;

void check(bool result, const char* expression, int line) {
    ++checks;
    if (!result) throw std::runtime_error(current_group + ": line " +
        std::to_string(line) + ": " + expression);
}
#define CHECK(x) check(static_cast<bool>(x), #x, __LINE__)

const std::string fixture = R"yaml(perception:
  profile:
    angle_min_rad: -0.5
    angle_step_rad: 0.25
    n_bins: 5
    range_max_m: 6
    dr_m: 0.1
    blind_near_lo_m: 0.25
    h_tol_m: 0.08
    z_pass_m: 0.75
    cover_min: 0.9
    plane_fit_min_inlier_frac: 0.5
    plane_fit_resid_max_m: 0.02
    slope_max_deg: 10
    sigma_max_m: 0.02
    eps_neg: 0.05
    w_h_m: 0.2
  extrinsic_base_cam:
    translation: [0, 0, 1]
    rotation_xyzw: [-0.5, 0.5, -0.5, 0.5]
    calib_record: null
    calib_sha256: null
    residual_max_m: null
  native_geometry_setup:
    schema_version: 1
    prior_normal: [0, 0, 1]
    prior_d_m: 0
    fit_near_min_m: 0.25
    fit_near_max_m: 2
    fit_prior_band_m: 0.2
    fit_inlier_distance_m: 0.02
    fit_min_xy_stddev_m: 0.01
    fit_min_up_dot: 0.9
    fit_max_prior_angle_deg: 10
    fit_min_inliers: 3
    fit_hypotheses: 32
    fit_seed: 1
)yaml";

nd::Metadata metadata() {
    nd::Metadata m;
    m.intrinsics = {308, 308, 320, 200, 640, 400};
    m.distortion.reported = true;
    m.distortion.model = 3;
    m.unit_m = 0.001F;
    return m;
}

std::string replace(std::string text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    if (at == std::string::npos) throw std::runtime_error("test fixture replacement not found: " + from);
    text.replace(at, from.size(), to);
    return text;
}

std::string set(std::string text, const std::string& key, const std::string& value) {
    const std::string prefix = "    " + key + ":";
    const auto at = text.find(prefix);
    if (at == std::string::npos) throw std::runtime_error("test fixture key not found: " + key);
    const auto end = text.find('\n', at);
    text.replace(at, end - at, prefix + " " + value);
    return text;
}

std::string omit(std::string text, const std::string& key) {
    const auto at = text.find("    " + key + ":");
    if (at == std::string::npos) throw std::runtime_error("test fixture key not found: " + key);
    text.erase(at, text.find('\n', at) - at + 1);
    return text;
}

cfg::NativeGeometrySetup parse(const std::string& text = fixture) {
    return cfg::parse_native_geometry_setup(text, metadata());
}

void reject(const std::string& text, const std::string& path = "",
            const nd::Metadata& native = metadata()) {
    std::string message;
    try { (void)cfg::parse_native_geometry_setup(text, native); }
    catch (const std::exception& e) { message = e.what(); }
    CHECK(!message.empty());
    if (!path.empty()) CHECK(message.find(path) != std::string::npos);
    ++rejected;
}

bool close(double a, double b) { return std::abs(a-b) <= 1e-12; }

void flow_and_transform() {
    const auto c = parse();
    const std::array<double, 9> expected{0, 0, 1, -1, 0, 0, 0, -1, 0};
    CHECK(c.projection.R_base_camera == expected);
    CHECK(c.projection.t_base_camera.x == 0 && c.projection.t_base_camera.y == 0 &&
          c.projection.t_base_camera.z == 1);
    CHECK(c.projection.intrinsics.width == 640 && c.projection.intrinsics.height == 400);
    CHECK(c.projection.intrinsics.fx == 308 && c.projection.intrinsics.fy == 308);
    CHECK(c.projection.intrinsics.cx == 320 && c.projection.intrinsics.cy == 200);
    CHECK(c.projection.distortion.reported && c.projection.distortion.model == 3);
    CHECK(c.projection.distortion.coefficients == metadata().distortion.coefficients);
    CHECK(c.projection.unit_m == metadata().unit_m);
    CHECK(c.geometry.n_bins == 5 && c.geometry.angle_min_rad == -0.5 && c.geometry.angle_step_rad == 0.25);
    CHECK(c.geometry.range_max_m == 6 && c.geometry.blind_near_lo_m == .25);
    CHECK(c.geometry.h_tol_m == .08 && c.geometry.z_pass_m == .75);
    CHECK(c.geometry.eps_neg == .05 && c.geometry.w_h_m == .2);
    CHECK(c.geometry.prior.normal.x == 0 && c.geometry.prior.normal.y == 0 &&
          c.geometry.prior.normal.z == 1 && c.geometry.prior.d == 0);
    CHECK(c.geometry.fit_near_min_m == .25 && c.geometry.fit_near_max_m == 2);
    CHECK(c.geometry.fit_prior_band_m == .2 && c.geometry.fit.inlier_distance_m == .02);
    CHECK(c.geometry.fit.min_inlier_fraction == .5 && c.geometry.fit.max_inlier_rms_m == .02);
    CHECK(c.geometry.fit.min_xy_stddev_m == .01 && c.geometry.fit.min_up_dot == .9);
    CHECK(close(c.geometry.fit.max_prior_angle_rad, 10*pi/180));
    CHECK(c.geometry.fit.min_inliers == 3 && c.geometry.fit.hypotheses == 32 && c.geometry.fit.seed == 1);
    CHECK(c.grid.dr_m == .1 && c.grid.cover_min == .9 && c.grid.sigma_max_m == .02);
    CHECK(close(c.grid.slope_max_rad, 10*pi/180));
    // A caller-supplied coefficient is copied, not erased by generic defaults.
    auto m = metadata(); m.distortion.coefficients[0] = .125F;
    CHECK(cfg::parse_native_geometry_setup(fixture, m).projection.distortion.coefficients[0] == .125F);
    const auto rounded = parse(set(set(fixture, "angle_step_rad", "0.1"), "angle_min_rad", "-0.2"));
    CHECK(rounded.geometry.angle_step_rad == static_cast<double>(0.1F));
    CHECK(rounded.geometry.angle_min_rad == static_cast<double>(-0.2F));
}

void vector_styles() {
    for (const auto& indent : {std::string("      "), std::string("    ")}) {
        auto y = set(fixture, "translation", "\n" + indent + "- 0\n" + indent + "- 0\n" + indent + "- 1");
        y = set(y, "rotation_xyzw", "\n" + indent + "- -0.5\n" + indent + "- 0.5\n" + indent + "- -0.5\n" + indent + "- 0.5");
        y = set(y, "prior_normal", "\n" + indent + "- 0\n" + indent + "- 0\n" + indent + "- 1");
        const auto c = parse(y);
        CHECK(c.projection.R_base_camera == parse().projection.R_base_camera);
        CHECK(c.projection.t_base_camera.z == 1 && c.geometry.prior.normal.z == 1);
    }
    CHECK(parse(set(fixture, "translation", "[ 0, 0, 1 ] # synthetic")).projection.t_base_camera.z == 1);
}

void literal_zeros_and_ignored_fallback() {
    auto y = fixture;
    for (const auto* key : {"blind_near_lo_m", "plane_fit_resid_max_m", "slope_max_deg",
            "sigma_max_m", "eps_neg", "w_h_m", "prior_d_m", "fit_near_min_m",
            "fit_max_prior_angle_deg", "fit_min_up_dot", "fit_seed"}) y = set(y, key, "0");
    const auto c = parse(y);
    CHECK(c.geometry.blind_near_lo_m == 0 && c.geometry.fit.max_inlier_rms_m == 0);
    CHECK(c.grid.slope_max_rad == 0 && c.grid.sigma_max_m == 0);
    CHECK(c.geometry.eps_neg == 0 && c.geometry.w_h_m == 0 && c.geometry.prior.d == 0);
    CHECK(c.geometry.fit_near_min_m == 0 && c.geometry.fit.max_prior_angle_rad == 0 && c.geometry.fit.seed == 0);
    CHECK(c.geometry.fit.min_up_dot == 0);
    CHECK(parse(set(fixture, "fit_seed", "18446744073709551615")).geometry.fit.seed ==
          std::numeric_limits<std::uint64_t>::max());
    const auto single = parse(set(set(fixture, "n_bins", "1"), "angle_min_rad", "0"));
    CHECK(single.geometry.angle_min_rad == 0 && single.geometry.n_bins == 1);
    const auto fallback = replace(fixture, "  extrinsic_base_cam:",
        "    dfree_cap_fallback_m: null\n    fallback_tilt_tol_deg: null\n"
        "    fallback_tilt_err_max_deg: null\n    fallback_align_err_m: null\n"
        "    omega_max_rps: null\n    seg_max_age_ms: null\n    tau_T: null\n"
        "    v_ego_max_mps: null\n    m_jitter_m: null\n  extrinsic_base_cam:");
    const auto f = parse(fallback);
    CHECK(f.geometry.z_pass_m == .75 && f.geometry.blind_near_lo_m == .25);
}

std::string with_record() {
    auto y = set(fixture, "calib_record", "/nonexistent/synthetic-only-calibration.json");
    y = set(y, "calib_sha256", std::string(64, 'a'));
    return set(y, "residual_max_m", "0.01");
}

// The record keys are allowed but not read here (ServiceConfig validates them,
// service::CalibrationVerifier reads them): set, null or omitted, the parsed
// setup is the same. A calibration status is never a configuration value.
void record_keys_and_calibration_status() {
    const auto plain = parse();
    auto absent = fixture;
    for (const auto* key : {"calib_record", "calib_sha256", "residual_max_m"}) absent = omit(absent, key);
    for (const auto& text : {with_record(), absent}) {
        const auto c = parse(text);
        CHECK(c.projection.R_base_camera == plain.projection.R_base_camera);
        CHECK(c.projection.t_base_camera.z == plain.projection.t_base_camera.z);
        CHECK(c.projection.unit_m == plain.projection.unit_m && c.geometry.n_bins == plain.geometry.n_bins);
        CHECK(c.grid.dr_m == plain.grid.dr_m);
    }
    reject(replace(fixture, "  profile:", "  extrinsic_calibrated: true\n  profile:"), "extrinsic_calibrated");
    reject(replace(fixture, "    n_bins: 5", "    extrinsic_calibrated: true\n    n_bins: 5"), "extrinsic_calibrated");
    reject(replace(fixture, "    translation:", "    calibrated: true\n    translation:"), "calibrated");
}

void missing_and_null() {
    const std::vector<std::string> keys{
        "angle_min_rad", "angle_step_rad", "n_bins", "range_max_m", "dr_m", "blind_near_lo_m",
        "h_tol_m", "z_pass_m", "cover_min", "plane_fit_min_inlier_frac", "plane_fit_resid_max_m",
        "slope_max_deg", "sigma_max_m", "eps_neg", "w_h_m", "translation", "rotation_xyzw",
        "schema_version", "prior_normal", "prior_d_m", "fit_near_min_m", "fit_near_max_m",
        "fit_prior_band_m", "fit_inlier_distance_m", "fit_min_xy_stddev_m", "fit_min_up_dot",
        "fit_max_prior_angle_deg", "fit_min_inliers", "fit_hypotheses", "fit_seed"};
    for (const auto& key : keys) {
        reject(omit(fixture, key), key);
        reject(set(fixture, key, "null"), key);
    }
    reject("lidar:\n  extrinsic_base_lidar:\n    translation: [0,0,1]\n");
    reject(replace(fixture, "extrinsic_base_cam:", "extrinsic_base_lidar:"));
    reject(set(fixture, "schema_version", "2"), "schema_version");
    reject(replace(fixture, "  profile:", "  profile: 7"));
    reject(replace(fixture, "  native_geometry_setup:", "  native_geometry_setup: false"));
}

void scalar_types_and_finiteness() {
    for (const auto* key : {"angle_min_rad", "angle_step_rad", "range_max_m", "z_pass_m",
            "dr_m", "prior_d_m", "fit_inlier_distance_m"}) {
        for (const auto* bad : {"nan", ".nan", "inf", "-inf", ".inf", "1e309", "true", "false", "[1, 2]", "nonnumeric"})
            reject(set(fixture, key, bad), key);
    }
    for (const auto* key : {"n_bins", "schema_version", "fit_min_inliers", "fit_hypotheses", "fit_seed"}) {
        for (const auto* bad : {"1.5", "1.0", "1e0", "true", "false", "-1", "18446744073709551616", "[]"})
            reject(set(fixture, key, bad), key);
    }
    reject(set(fixture, "angle_step_rad", "1e-300")); // Finite double becomes zero in the wire domain.
    reject(set(fixture, "z_pass_m", "1e100"), "z_pass_m");
}

void vectors_and_rigid_admission() {
    for (const auto* key : {"translation", "prior_normal"})
        for (const auto* bad : {"0", "true", "[]", "[0,1]", "[0,0,1,2]", "[0,null,1]", "[0,nan,1]",
                "[0,inf,1]", "[0,1e309,1]", "[0,true,1]", "[0, [0], 1]", "[0,0,1,]"})
            reject(set(fixture, key, bad), key);
    for (const auto* bad : {"0", "[]", "[0,0,1]", "[0,0,0,0]", "[0,0,0,2]", "[0,0,0,1.00001]",
            "[0,0,0,nan]", "[0,0,0,inf]", "[0,0,0,true]", "[0,0,0,null]", "[0,0,0,1,0]"})
        reject(set(fixture, "rotation_xyzw", bad), "rotation_xyzw");
    for (const auto* bad : {"[0,0,0]", "[0,0,2]", "[0,0,-1]", "[1,0,0]"})
        reject(set(fixture, "prior_normal", bad));
    const auto identity = parse(set(fixture, "rotation_xyzw", "[0,0,0,1]"));
    const std::array<double, 9> expected{1,0,0,0,1,0,0,0,1};
    CHECK(identity.projection.R_base_camera == expected); // Ray/prior intersection admission remains downstream.
}

// Object geometry path: the same extrinsic, bit for bit, and the same refusals.
void base_from_camera_matches_setup() {
    const auto setup = parse(fixture);
    const auto base = cfg::parse_base_from_camera(fixture);
    CHECK(base.R == setup.projection.R_base_camera);
    CHECK(base.t.x == setup.projection.t_base_camera.x && base.t.y == setup.projection.t_base_camera.y &&
          base.t.z == setup.projection.t_base_camera.z);
    const auto refuses = [&](const std::string& text, const std::string& key) {
        try { (void)cfg::parse_base_from_camera(text); }
        catch (const std::exception& e) { ++rejected; return std::string(e.what()).find(key) != std::string::npos; }
        return false;
    };
    for (const auto* bad : {"[0,0,0,2]", "[0,0,0,1.00001]", "[0,0,0,nan]", "[0,0,1]"})
        CHECK(refuses(set(fixture, "rotation_xyzw", bad), "rotation_xyzw"));
    CHECK(refuses(set(fixture, "translation", "[0,1]"), "translation"));
    CHECK(refuses(replace(fixture, "    translation:", "    calibrated: true\n    translation:"), "calibrated"));
}

void domains_and_capacities() {
    const std::vector<std::pair<std::string, std::string>> cases{
        {"n_bins","0"}, {"n_bins","4"}, {"n_bins","1025"}, {"angle_min_rad","0.5"},
        {"angle_min_rad","-4"}, {"angle_step_rad","0"}, {"angle_step_rad","-0.25"},
        {"angle_step_rad","7"}, {"range_max_m","0"}, {"range_max_m","6.01"},
        {"range_max_m","-1"}, {"blind_near_lo_m","6"}, {"blind_near_lo_m","-1"},
        {"h_tol_m","0"}, {"h_tol_m","0.75"}, {"z_pass_m","0.08"}, {"eps_neg","-0.1"},
        {"w_h_m","-0.1"}, {"dr_m","0"}, {"dr_m","0.001"}, {"cover_min","0"}, {"cover_min","1.01"},
        {"slope_max_deg","-1"}, {"slope_max_deg","90"}, {"sigma_max_m","-0.1"},
        {"plane_fit_min_inlier_frac","0"}, {"plane_fit_min_inlier_frac","1.01"},
        {"plane_fit_resid_max_m","-0.1"}, {"fit_near_min_m","-1"}, {"fit_near_min_m","2"},
        {"fit_near_max_m","0.25"}, {"fit_near_max_m","7"}, {"fit_prior_band_m","0"},
        {"fit_inlier_distance_m","0"}, {"fit_min_xy_stddev_m","0"}, {"fit_min_up_dot","1.01"},
        {"fit_max_prior_angle_deg","-1"}, {"fit_max_prior_angle_deg","181"},
        {"fit_min_inliers","2"}, {"fit_min_inliers","256001"}, {"fit_hypotheses","0"}, {"fit_hypotheses","257"}};
    for (const auto& c : cases) reject(set(fixture, c.first, c.second));
    auto too_many = set(set(set(fixture, "n_bins", "1023"), "angle_step_rad", "0.001953125"),
                        "angle_min_rad", "-0.998046875");
    reject(set(too_many, "dr_m", "0.025"), "n_bins/dr_m");
    reject(set(set(fixture, "angle_step_rad", "1.5"), "angle_min_rad", "-3")); // Outer edge beyond pi.
}

void native_metadata_admission() {
    auto m = metadata(); m.intrinsics.width = 1280; m.intrinsics.height = 800;
    reject(fixture, "native_profile", m);
    m = metadata(); m.intrinsics.height = 401; reject(fixture, "native_profile", m);
    m = metadata(); m.intrinsics.fx = 0; reject(fixture, "native_profile", m);
    m = metadata(); m.intrinsics.fy = -1; reject(fixture, "native_profile", m);
    m = metadata(); m.intrinsics.cx = std::numeric_limits<float>::quiet_NaN(); reject(fixture, "native_profile", m);
    m = metadata(); m.intrinsics.cy = std::numeric_limits<float>::infinity(); reject(fixture, "native_profile", m);
    m = metadata(); m.unit_m = 0; reject(fixture, "native_profile", m);
    m = metadata(); m.unit_m = -1; reject(fixture, "native_profile", m);
    m = metadata(); m.unit_m = std::numeric_limits<float>::quiet_NaN(); reject(fixture, "native_profile", m);
    m = metadata(); m.distortion.reported = false; reject(fixture, "native_profile", m);
    m = metadata(); m.distortion.coefficients[7] = std::numeric_limits<float>::infinity(); reject(fixture, "native_profile", m);
    m = metadata(); m.distortion.model = 0;
    CHECK(cfg::parse_native_geometry_setup(fixture, m).projection.distortion.model == 0);
    // This layer copies provenance; it does not claim that an SDK supports it.
    m.distortion.model = 999;
    CHECK(cfg::parse_native_geometry_setup(fixture, m).projection.distortion.model == 999);
}

void strict_resolved_yaml() {
    reject(set(fixture, "range_max_m", "${range_max}"), "resolved_config");
    reject(replace(fixture, "    n_bins: 5", "    n_bins: 5\n    n_bins: 5"), "n_bins");
    reject(fixture + "perception:\n  profile:\n    n_bins: 5\n", "perception");
    reject(replace(fixture, "  profile:", "\tprofile:"), "resolved_config");
    reject(set(fixture, "translation", "\n\t- 0\n\t- 0\n\t- 1"));
    reject(set(fixture, "translation", "\n      - 0\n    - 0\n      - 1"));
    reject(replace(fixture, "    angle_step_rad:", "      angle_step_rad:"), "angle_step_rad");
    reject(replace(fixture, "  extrinsic_base_cam:", "   extrinsic_base_cam:"), "extrinsic_base_cam");
    for (const auto* key : {"translation", "prior_normal"})
        reject(set(fixture, key, "\n      - 0,0\n      - 1"), key);
    reject(set(fixture, "rotation_xyzw", "\n      - -0.5,0.5\n      - -0.5\n      - 0.5"), "rotation_xyzw");
    reject(set(fixture, "range_max_m", "\n      - 6"));
    reject("---\n" + fixture, "resolved_config");
    reject(fixture + "...\n", "resolved_config");
    reject(set(fixture, "translation", "&position [0,0,1]"));
    reject(set(fixture, "translation", "*position"));
    reject(set(fixture, "range_max_m", "!!float 6"));
    reject(replace(fixture, "    prior_d_m: 0", "    unknown_fit_option: 1\n    prior_d_m: 0"), "unknown_fit_option");
    reject(std::string(1024U*1024U + 1U, ' '), "resolved_config");
}

void synthetic_geometry_to_profile() {
    const auto c = parse(with_record());
    // Deliberately synthetic optical-z=1 rays, NOT a distortion model, measured
    // calibration, or a replacement for the separately admitted SDK ray factory.
    std::vector<geom::Vec3> rays(geom::kGeometryPixels);
    std::vector<std::uint16_t> raster(geom::kGeometryPixels);
    for (std::size_t y = 0; y < geom::kNativeHeight; ++y) {
        for (std::size_t x = 0; x < geom::kNativeWidth; ++x) {
            const auto at = y*geom::kNativeWidth+x;
            rays[at] = {-.5 + static_cast<double>(x)/639.0,
                         .6 + .9*static_cast<double>(y)/399.0, 1};
            raster[at] = static_cast<std::uint16_t>(std::llround(1.0/rays[at].y/c.projection.unit_m));
        }
    }
    geom::NativeGeometry geometry(c.projection, rays.data(), rays.size(), c.geometry);
    geom::NormalGrid grid(geometry, c.grid);
    rns::NormalProfileBuilder builder(grid);
    CHECK(builder.body() == nullptr);
    nd::Mailbox mailbox(geom::kGeometryPixels);
    nd::Frame frame(geom::kGeometryPixels);
    nd::View view;
    view.metadata = metadata();
    view.metadata.source = {mailbox.begin_stream(), 1, 0, 0, 1000, 1001, 1000000000LL};
    view.pre_d2c = true;
    view.pixels = raster.data();
    view.bytes = raster.size()*sizeof(raster[0]);
    CHECK(mailbox.try_offer(view) == nd::Offer::Accepted);
    CHECK(mailbox.try_take(frame) == nd::Take::Taken);
    CHECK(geometry.process(frame, mailbox).status == geom::GeometryStatus::NormalEvidence);
    CHECK(grid.process(mailbox).status == geom::GridStatus::NormalEvidence);
    const geom::CaptureStamp capture{frame.metadata().source, 42, 1000000000LL};
    CHECK(builder.build(mailbox, capture).status == rns::NormalProfileStatus::Ready);
    const auto* body = builder.body();
    CHECK(body != nullptr);
    CHECK(!body->extrinsic_calibrated);
    CHECK(!body->t_seg_mono_ms.has_value());
    CHECK(body->bins.size() == 5 && body->range_max_m == 6 && body->z_pass_m == .75F);
    CHECK(body->angle_min_rad == -.5F && body->angle_step_rad == .25F);
    CHECK(body->t_capture_mono_ms == 1000);
    std::size_t measured_bins = 0;
    for (const auto& bin : body->bins) {
        CHECK((bin.src & (rns::kT | rns::kS)) == 0);
        if (bin.src & rns::kG) ++measured_bins;
    }
    CHECK(measured_bins > 0);
    mailbox.end_stream();
    CHECK(builder.build(mailbox, capture).status == rns::NormalProfileStatus::InactiveOrStale);
    CHECK(builder.body() == nullptr);
}

void run(const char* name, void (*test)()) {
    current_group = name;
    test();
    ++groups;
    std::cout << "PASS " << name << '\n';
}
} // namespace

int main() {
    try {
        run("flow_transform_and_units", flow_and_transform);
        run("flow_block_and_indentless_vectors", vector_styles);
        run("literal_zero_and_ignored_null_fallback", literal_zeros_and_ignored_fallback);
        run("record_keys_unread_status_not_configurable", record_keys_and_calibration_status);
        run("missing_null_and_old_schema", missing_and_null);
        run("scalar_types_finiteness_and_uints", scalar_types_and_finiteness);
        run("vector_shape_quaternion_and_prior", vectors_and_rigid_admission);
        run("object_path_extrinsic_matches_setup", base_from_camera_matches_setup);
        run("numeric_domains_and_bounded_capacity", domains_and_capacities);
        run("native_metadata_admission_not_sdk_model_support", native_metadata_admission);
        run("strict_resolved_yaml", strict_resolved_yaml);
        run("synthetic_geometry_grid_profile", synthetic_geometry_to_profile);
        std::cout << "{\"result\":\"PASS\",\"groups\":" << groups << ",\"checks\":" << checks
                  << ",\"rejections\":" << rejected
                  << ",\"scope\":\"resolved_yaml_and_synthetic_kernels\",\"camera\":false,\"sdk\":false,"
                     "\"physical_calibration_verified\":false,\"production_acceptance\":false}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL " << e.what() << '\n';
        std::cout << "{\"result\":\"FAIL\",\"groups\":" << groups << ",\"checks\":" << checks
                  << ",\"rejections\":" << rejected << "}\n";
        return 1;
    }
}
