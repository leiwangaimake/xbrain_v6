/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_geometry_setup.cpp
 * Brief: Host test of the configured-geometry setup of the formal service.
 *
 * Description:
 * Covers PSC-2 prevalidation, the PSC-3 ray table, the PSC-7 precomputed-table self-check
 * (including A19-BOOT-6) and the health ROI chosen from the calibration verdict. Synthetic pinhole
 * rays; no SDK or camera.
 */
#include "service/geometry_setup.hpp"

#include "common/config.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace perception;
using namespace perception::service;

namespace {
int checks = 0, failures = 0, rejections = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

const std::string kSerial = "CHL5663000DS";

std::string num(double v) {
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}
// Base (x forward, y left, z up) <- camera optical (x right, y down, z forward),
// camera pitched DOWN by pitch_rad (negative = up). Returns quaternion xyzw.
std::array<double, 4> quaternion_for_pitch(double pitch_rad) {
    const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
    // columns: cam x, cam y, cam z expressed in base
    const double R[3][3] = {{0, -s, c}, {-1, 0, 0}, {0, -c, -s}};
    const double trace = R[0][0] + R[1][1] + R[2][2];
    std::array<double, 4> q{};
    if (trace > 0) {
        const double t = std::sqrt(1.0 + trace) * 2;
        q = {(R[2][1] - R[1][2]) / t, (R[0][2] - R[2][0]) / t, (R[1][0] - R[0][1]) / t, 0.25 * t};
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        const double t = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2;
        q = {0.25 * t, (R[1][0] + R[0][1]) / t, (R[0][2] + R[2][0]) / t, (R[2][1] - R[1][2]) / t};
    } else if (R[1][1] > R[2][2]) {
        const double t = std::sqrt(1.0 - R[0][0] + R[1][1] - R[2][2]) * 2;
        q = {(R[1][0] + R[0][1]) / t, 0.25 * t, (R[2][1] + R[1][2]) / t, (R[0][2] - R[2][0]) / t};
    } else {
        const double t = std::sqrt(1.0 - R[0][0] - R[1][1] + R[2][2]) * 2;
        q = {(R[0][2] + R[2][0]) / t, (R[2][1] + R[1][2]) / t, 0.25 * t, (R[1][0] - R[0][1]) / t};
    }
    double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (auto& v : q) v /= n;
    return q;
}

struct Transform {
    std::array<double, 3> t{};
    std::array<double, 4> q{};
};

struct Prior {
    std::array<double, 3> n{0, 0, 1};
    double d = 0;
};

std::string contract_yaml(const Transform& x, const std::string& record_lines, const Prior& prior = Prior{}) {
    const auto vec = [](const double* v, std::size_t n) {
        std::string s = "[";
        for (std::size_t i = 0; i < n; ++i) s += (i ? ", " : "") + num(v[i]);
        return s + "]";
    };
    return std::string("perception:\n") +
        "  profile:\n"
        "    angle_min_rad: -0.5\n    angle_step_rad: 0.25\n    n_bins: 5\n    range_max_m: 6\n    dr_m: 0.1\n"
        "    blind_near_lo_m: 0.25\n    h_tol_m: 0.08\n    z_pass_m: 0.75\n    cover_min: 0.9\n"
        "    plane_fit_min_inlier_frac: 0.5\n    plane_fit_resid_max_m: 0.02\n    slope_max_deg: 10\n"
        "    sigma_max_m: 0.02\n    eps_neg: 0.05\n    w_h_m: 0.2\n"
        "  extrinsic_base_cam:\n"
        "    translation: " + vec(x.t.data(), 3) + "\n"
        "    rotation_xyzw: " + vec(x.q.data(), 4) + "\n" + record_lines +
        "  native_geometry_setup:\n"
        "    schema_version: 1\n    prior_normal: " + vec(prior.n.data(), 3) + "\n    prior_d_m: " + num(prior.d) + "\n    fit_near_min_m: 0.25\n"
        "    fit_near_max_m: 2\n    fit_prior_band_m: 0.2\n    fit_inlier_distance_m: 0.02\n"
        "    fit_min_xy_stddev_m: 0.01\n    fit_min_up_dot: 0.9\n    fit_max_prior_angle_deg: 10\n"
        "    fit_min_inliers: 3\n    fit_hypotheses: 32\n    fit_seed: 1\n";
}
const std::string kNoRecord = "    calib_record: null\n    calib_sha256: null\n    residual_max_m: null\n";

ServiceConfig service_config(const Transform& x, const std::string& record_lines = kNoRecord, const std::string& path = "",
                     const std::string& sha = "", double residual = 0, const Prior& prior = Prior{}) {
    ServiceConfig c;
    c.contract.extrinsic.mode = ExtrinsicMode::TransformConfigured;
    c.contract.extrinsic.translation = x.t;
    c.contract.extrinsic.rotation_xyzw = x.q;
    if (!path.empty()) {
        c.contract.extrinsic.calib_record = path;
        c.contract.extrinsic.calib_sha256 = sha;
        c.contract.extrinsic.residual_max_m = residual;
    }
    c.contract_yaml = contract_yaml(x, record_lines, prior);
    return c;
}

native_depth::Metadata metadata() {
    native_depth::Metadata m;
    m.intrinsics = {320.0F, 320.0F, 320.0F, 200.0F, 640U, 400U};
    m.distortion.reported = true;
    m.distortion.model = 3;
    m.unit_m = 0.001F;
    return m;
}

// Distortion-free pinhole optical rays (z == 1) for the bound intrinsics.
std::shared_ptr<void> pinhole_rays(const geometry::ProjectionBinding& b) {
    auto rays = std::make_shared<std::vector<geometry::Vec3>>(geometry::kGeometryPixels);
    for (std::size_t v = 0; v < geometry::kNativeHeight; ++v)
        for (std::size_t u = 0; u < geometry::kNativeWidth; ++u)
            (*rays)[v * geometry::kNativeWidth + u] = {(u - b.intrinsics.cx) / b.intrinsics.fx,
                                                       (v - b.intrinsics.cy) / b.intrinsics.fy, 1.0};
    return std::static_pointer_cast<void>(rays);
}

void expect_check(const std::string& name, const std::function<void()>& run, const std::string& check,
                 const std::string& key_contains) {
    try {
        run();
        ++checks; ++failures;
        std::cerr << "FAIL accepted: " << name << "\n";
    } catch (const StartupCheckError& e) {
        ++checks; ++rejections;
        if (e.check() != check || e.key().find(key_contains) == std::string::npos) {
            ++failures;
            std::cerr << "FAIL " << name << ": check=" << e.check() << " key=" << e.key() << " why=" << e.what() << "\n";
        }
    } catch (const std::exception& e) {
        ++checks; ++failures;
        std::cerr << "FAIL " << name << ": unexpected " << e.what() << "\n";
    }
}
}  // namespace

int main() {
    char tmpl[] = "/tmp/geometry_setup_test.XXXXXX";
    const fs::path dir = fs::canonical(::mkdtemp(tmpl));
    const Transform bench{{0, 0, 0.6}, quaternion_for_pitch(30.0 * M_PI / 180.0)};
    const Transform sky{{0, 0, 0.6}, quaternion_for_pitch(-60.0 * M_PI / 180.0)};
    try {
        // production-shaped factory: rays are bound to the parsed binding, the
        // table is built from exactly those rays and that binding
        GeometryFactory factory;
        geometry::ProjectionBinding bound;
        factory.prepare_rays = [&bound](const geometry::ProjectionBinding& b) {
            bound = b;
            return pinhole_rays(b);
        };
        factory.make_geometry = [&bound](const std::shared_ptr<void>& p, const geometry::GeometryOptions& o) {
            const auto rays = std::static_pointer_cast<std::vector<geometry::Vec3>>(p);
            return std::make_unique<geometry::NativeGeometry>(bound, rays->data(), rays->size(), o);
        };

        // ---- uncertified transform (Q12): compute, lower-half ROI ----
        ExtrinsicConfig absent;
        const auto no_transform = verify_calibration(absent, kSerial);
        auto cfg = service_config(bench);
        const auto transform_only = verify_calibration(cfg.contract.extrinsic, kSerial);
        CHECK(transform_only.code() == CalibrationCode::TransformOnly);
        prevalidate_geometry_setup(cfg);
        auto g = build_profile_geometry(cfg, metadata(), transform_only, factory);
        CHECK(g && g->geometry && g->grid && g->roi_pixels > 0 && g->roi_pixels == g->geometry->roi_pixels());
        CHECK(!g->calibrated && g->health_mask.empty() && g->health_roi.kind == rns::HealthRoiKind::LowerHalfUncalibrated &&
              g->health_roi.mask == nullptr);
        CHECK(bound.t_base_camera.z == 0.6 && bound.intrinsics.fx == 320.0F);
        // 30 deg down with a +-32 deg vertical FOV: the top row looks above the horizon
        std::size_t top_row = 0;
        for (std::size_t u = 0; u < geometry::kNativeWidth; ++u) top_row += g->geometry->prior_roi()[u];
        CHECK(top_row == 0);

        // ---- certified record: calibrated verdict selects the precomputed ROI ----
        CalibrationRecord r;
        r.schema_version = 1; r.record_id = "geom-test"; r.scope = CalibrationScope::Vehicle;
        r.created_utc = "2026-09-18T00:00:00Z"; r.camera_driver = "orbbec_gemini338le"; r.camera_serial = kSerial;
        r.depth_stream = "640x400_y16_30"; r.base_frame = "base_link"; r.base_origin = "synthetic host test";
        r.translation = bench.t; r.rotation_xyzw = bench.q; r.method_plane = "depth_plane_fit";
        r.method_yaw = "straight_edge_reference"; r.method_xy = "tape_measure"; r.plane_frames = 1;
        r.plane_inliers = 3; r.plane_inlier_fraction = 1; r.plane_fit_inlier_rms_m = 0.001;
        r.tool_name = "host_test"; r.tool_version = "1";
        const auto bytes = format_calibration_record(r);
        const auto path = (dir / "record.yaml").string();
        { std::ofstream(path, std::ios::binary) << bytes; }
        const auto sha = yolo::sha256_hex(bytes);
        const std::string record_lines = "    calib_record: " + path + "\n    calib_sha256: " + sha + "\n    residual_max_m: 0.002\n";
        auto certified = service_config(bench, record_lines, path, sha, 0.002);
        const auto verified = verify_calibration(certified.contract.extrinsic, kSerial);
        CHECK(verified.calibrated());
        prevalidate_geometry_setup(certified);
        auto gc = build_profile_geometry(certified, metadata(), verified, factory);
        CHECK(gc->calibrated && gc->health_roi.kind == rns::HealthRoiKind::CalibratedStaticMask);
        CHECK(gc->health_mask.size() == geometry::kGeometryPixels && gc->health_roi.mask == gc->health_mask.data() &&
              gc->health_roi.mask_pixels == geometry::kGeometryPixels && gc->health_roi.width == 640 && gc->health_roi.height == 400);
        std::size_t ones = 0;
        for (auto m : gc->health_mask) ones += m;
        CHECK(ones == gc->roi_pixels && gc->roi_pixels == g->roi_pixels);
        // the SAME transform with an uncertified verdict never gets the calibrated ROI
        auto gu = build_profile_geometry(certified, metadata(), transform_only, factory);
        CHECK(!gu->calibrated && gu->health_roi.kind == rns::HealthRoiKind::LowerHalfUncalibrated);

        // ---- PSC-7 / A19-BOOT-6: upward-facing transform, empty table ----
        auto up = service_config(sky);
        prevalidate_geometry_setup(up);  // numerically valid configuration
        expect_check("A19-BOOT-6 sky-facing transform", [&] {
            (void)build_profile_geometry(up, metadata(), verify_calibration(up.contract.extrinsic, kSerial), factory);
        }, "PSC-7", "extrinsic_base_cam");
        // ---- GEOM-PRIOR-HEIGHT extension self-check (B3-4), separate from PSC-7 ----
        const auto build = [&](const ServiceConfig& c) {
            return build_profile_geometry(c, metadata(), verify_calibration(c.contract.extrinsic, kSerial), factory);
        };
        auto below_down = service_config({{0, 0, -0.2}, quaternion_for_pitch(60.0 * M_PI / 180.0)});
        expect_check("below the plane, every ray downward", [&] { (void)build(below_down); },
                    "GEOM-PRIOR-HEIGHT", "extrinsic_base_cam.translation");
        auto below = service_config({{0, 0, -0.2}, bench.q});
        expect_check("below the prior plane with upward rays", [&] { (void)build(below); },
                    "GEOM-PRIOR-HEIGHT", "extrinsic_base_cam.translation");
        auto on_plane = service_config({{0, 0, 0}, bench.q});
        expect_check("optical centre on the prior plane", [&] { (void)build(on_plane); },
                    "GEOM-PRIOR-HEIGHT", "extrinsic_base_cam.translation");
        // the message carries n, d, t and the signed height
        try { (void)build(below); } catch (const StartupCheckError& e) {
            const std::string why = e.what();
            CHECK(why.find("n=(") != std::string::npos && why.find(" d=") != std::string::npos &&
                  why.find("t=(") != std::string::npos && why.find("signed_height=-0.2") != std::string::npos);
        }
        // t.z < 0 but above a nonzero-d prior (plane z = -1): legal
        const Prior lowered{{0, 0, 1}, 1.0};
        auto legal_negative_z = service_config({{0, 0, -0.3}, bench.q}, kNoRecord, "", "", 0, lowered);
        prevalidate_geometry_setup(legal_negative_z);
        CHECK(build(legal_negative_z) != nullptr);
        // t.z > 0 but below a raised prior (plane z = +1): refused
        const Prior raised{{0, 0, 1}, -1.0};
        auto illegal_positive_z = service_config({{0, 0, 0.6}, bench.q}, kNoRecord, "", "", 0, raised);
        expect_check("t.z > 0 below a raised prior", [&] { (void)build(illegal_positive_z); },
                    "GEOM-PRIOR-HEIGHT", "extrinsic_base_cam.translation");
        // tilted unit normal (5 deg about y)
        const double a5 = 5.0 * M_PI / 180.0;
        const Prior tilted{{std::sin(a5), 0, std::cos(a5)}, 0.0};
        auto tilted_ok = service_config({{0, 0, 0.6}, bench.q}, kNoRecord, "", "", 0, tilted);  // height 0.6 cos 5
        CHECK(build(tilted_ok) != nullptr);
        auto tilted_bad = service_config({{-8.0, 0, 0.6}, bench.q}, kNoRecord, "", "", 0, tilted);  // 0.6cos5 - 8sin5 < 0
        expect_check("tilted prior, forward-offset camera below it", [&] { (void)build(tilted_bad); },
                    "GEOM-PRIOR-HEIGHT", "extrinsic_base_cam.translation");
        // origin shift invariance: p' = p - a, t' = t - a, d' = d + n.a
        const std::array<double, 3> shift{0.2, -0.1, 0.5};
        const auto shifted = [&](const Transform& x, const Prior& pr) {
            Transform y = x;
            for (int i = 0; i < 3; ++i) y.t[i] -= shift[i];
            Prior q = pr;
            q.d += pr.n[0] * shift[0] + pr.n[1] * shift[1] + pr.n[2] * shift[2];
            return service_config(y, kNoRecord, "", "", 0, q);
        };
        CHECK(build(shifted({{0, 0, 0.6}, bench.q}, tilted)) != nullptr);
        expect_check("shifted origin keeps a below-plane verdict", [&] { (void)build(shifted({{-8.0, 0, 0.6}, bench.q}, tilted)); },
                    "GEOM-PRIOR-HEIGHT", "extrinsic_base_cam.translation");
        // non-unit / reversed / non-finite prior normals: parameter check (PSC-2)
        for (const auto& bad : {Prior{{0, 0, 2}, 0}, Prior{{0, 0, -1}, 0}}) {
            auto c = service_config({{0, 0, 0.6}, bench.q}, kNoRecord, "", "", 0, bad);
            expect_check("invalid prior normal", [&] { prevalidate_geometry_setup(c); }, "PSC-2", "native_geometry_setup");
        }
        {
            auto c = service_config({{0, 0, 0.6}, bench.q});
            c.contract_yaml.replace(c.contract_yaml.find("prior_normal: [0, 0, 1]"), 23, "prior_normal: [0, nan, 1]");
            expect_check("non-finite prior normal", [&] { prevalidate_geometry_setup(c); }, "PSC-2", "native_geometry_setup");
        }

        // ---- PSC-3: unsupported projection metadata (ray factory throws) ----
        GeometryFactory rejecting = factory;
        rejecting.prepare_rays = [](const geometry::ProjectionBinding&) -> std::shared_ptr<void> {
            throw std::invalid_argument("unsupported distortion model");
        };
        expect_check("ray table rejected", [&] {
            (void)build_profile_geometry(cfg, metadata(), transform_only, rejecting);
        }, "PSC-3", "native_depth");

        // ---- coverage tickets (2026-10-27): the variable the service exports for coverage_tickets ----
        CHECK(std::getenv("PERCEPTION_COVERAGE_TICKETS") == nullptr && !g->grid->options().fitted_tickets);
        CHECK(::setenv("PERCEPTION_COVERAGE_TICKETS", "fitted", 1) == 0);
        CHECK(build_profile_geometry(cfg, metadata(), transform_only, factory)->grid->options().fitted_tickets);
        CHECK(::setenv("PERCEPTION_COVERAGE_TICKETS", "prior", 1) == 0);
        CHECK(!build_profile_geometry(cfg, metadata(), transform_only, factory)->grid->options().fitted_tickets);
        CHECK(::setenv("PERCEPTION_COVERAGE_TICKETS", "on", 1) == 0);
        expect_check("coverage tickets value", [&] {
            (void)build_profile_geometry(cfg, metadata(), transform_only, factory);
        }, "ENV", "PERCEPTION_COVERAGE_TICKETS");
        CHECK(::unsetenv("PERCEPTION_COVERAGE_TICKETS") == 0);

        // ---- PSC-2: configuration errors, prevalidation and real metadata ----
        auto missing = cfg;
        missing.contract_yaml = contract_yaml(bench, kNoRecord);
        missing.contract_yaml.replace(missing.contract_yaml.find("    fit_hypotheses: 32\n"), 22, "");
        expect_check("prevalidate missing fit_hypotheses", [&] { prevalidate_geometry_setup(missing); },
                    "PSC-2", "native_geometry_setup.fit_hypotheses");
        auto unknown = cfg;
        unknown.contract_yaml += "    fit_magic: 1\n";
        expect_check("prevalidate unknown setup key", [&] { prevalidate_geometry_setup(unknown); },
                    "PSC-2", "native_geometry_setup.fit_magic");
        auto wrong_metadata = metadata();
        wrong_metadata.intrinsics.width = 848;
        expect_check("real metadata not 640x400", [&] {
            (void)build_profile_geometry(cfg, wrong_metadata, transform_only, factory);
        }, "PSC-2", "native_profile");

        // ---- misuse ----
        bool logic = false;
        try { ServiceConfig none; (void)build_profile_geometry(none, metadata(), no_transform, factory); }
        catch (const std::logic_error&) { logic = true; }
        CHECK(logic);
        ServiceConfig none;
        prevalidate_geometry_setup(none);  // absent transform: nothing to validate
        ++checks;
    } catch (const std::exception& e) {
        ++failures;
        std::cerr << "FAIL unexpected exception: " << e.what() << "\n";
    }
    std::error_code ec;
    fs::remove_all(dir, ec);
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"rejections\":" << rejections << ",\"scope\":\"geometry_setup_host\",\"sdk\":false,\"camera\":false}" << std::endl;
    return failures ? 1 : 0;
}
