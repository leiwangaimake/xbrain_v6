/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_t_projection_setup.cpp
 * Brief: T's colour projection: the Objects registration's pixel, independent of base <- depth.
 *
 * Description:
 * Also covers the capability / fault split of its setup (item 5).
 * Uses the session calibration logged on 1.8 (rf2_smoke, 2026-09-30: Gemini 338Le CHL5663000DS,
 * firmware 1.8.17, SDK 2.9.3).
 * 1. Round trip: for RGB pixel centres across the raster (edges included) and three depths, the
 *    registration's undistorted ray, taken to base as the Objects path does and projected, lands on
 *    (u + 0.5, v + 0.5) within 2e-3 px, so floor() gives the same pixel the Objects path uses. The
 *    residual (measured 6.4e-4 px) is not the model: the SDK rotation arrives as float and is not
 *    exactly orthonormal, and each path inverts it by transposition (rns::inverse). It is far below
 *    the registration's own 1.1-1.4 px residual.
 * 2. Two different configured base <- depth transforms give the same colour pixel for the same
 *    depth-frame point (the configured extrinsic cancels).
 * 3. Zero distortion equals the rectified pinhole reference plus the 0.5 convention shift.
 * 4. Behind the camera, and far beyond the image corners (where the polynomial could fold back),
 *    is no sample.
 * 5. No session, a failed registration gate, another raster size, another distortion model, a
 *    nonzero k4 and an invalid SDK rotation are capability gaps; an unparsable extrinsic_base_cam is
 *    a fault, not a capability gap.
 */

#include "service/t_projection_setup.hpp"

#include "common/rns_geometry_config.hpp"
#include "output/rns_object_geometry.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using perception::RgbDepthRegistration;
using perception::StandardDistortion;
using perception::geometry::CalibratedColor;
using perception::geometry::RectifiedColor;
using perception::geometry::Vec3;
using perception::service::TProjectionCapabilityError;
using perception::service::TProjectionSession;
using perception::service::setup_t_projection;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

// base_link <- depth optical: camera 0.8 m high, level, looking along +x.
const std::string kYamlLevel =
    "perception:\n"
    "  extrinsic_base_cam:\n"
    "    translation: [0.0, 0.0, 0.8]\n"
    "    rotation_xyzw: [-0.5, 0.5, -0.5, 0.5]\n"
    "    calib_record: null\n"
    "    calib_sha256: null\n"
    "    residual_max_m: null\n";
// Another mount: 1.2 m high, shifted, pitched down about 20 degrees.
const std::string kYamlPitched =
    "perception:\n"
    "  extrinsic_base_cam:\n"
    "    translation: [0.15, -0.05, 1.2]\n"
    "    rotation_xyzw: [-0.579227965339569, 0.579227965339569, -0.405579787672639, 0.405579787672639]\n"
    "    calib_record: null\n"
    "    calib_sha256: null\n"
    "    residual_max_m: null\n";

const StandardDistortion kDistortion{-0.0325959, 0.0366757, -0.0124716, 4.03676e-06, -6.68168e-05};

TProjectionSession session_18(std::shared_ptr<const RgbDepthRegistration> registration) {
    TProjectionSession s;
    s.present = true;
    s.fx = 611.813;
    s.fy = 611.809;
    s.cx = 640.654;
    s.cy = 400.484;
    s.width = 1280;
    s.height = 800;
    s.distortion_model = 4;
    s.distortion = {-0.0325959F, 0.0366757F, -0.0124716F, 0, 0, 0, 4.03676e-06F, -6.68168e-05F};
    s.depth_to_colour_R = {0.999996F, 0.00120195F, -0.00249771F, -0.00120036F, 0.999999F,
                           0.000636043F, 0.00249847F, -0.000633042F, 0.999997F};
    s.depth_to_colour_t_m = {-0.0238482F, 0.0000745797F, -0.000111854F};
    s.registration = std::move(registration);
    s.registration_status = "active";
    return s;
}

perception::rns::RigidTransform base_from_depth(const std::string& yaml) {
    const auto base = perception::config::parse_base_from_camera(yaml);
    perception::rns::RigidTransform t;
    t.R = base.R;
    t.t = {base.t.x, base.t.y, base.t.z};
    return t;
}

perception::rns::RigidTransform colour_from_depth(const TProjectionSession& s) {
    perception::rns::RigidTransform t;
    for (std::size_t i = 0; i < 9; ++i) t.R[i] = s.depth_to_colour_R[i];
    for (std::size_t i = 0; i < 3; ++i) t.t[i] = s.depth_to_colour_t_m[i];
    return t;
}

Vec3 apply(const perception::rns::RigidTransform& t, Vec3 p) {
    const auto& r = t.R;
    return {r[0] * p.x + r[1] * p.y + r[2] * p.z + t.t[0], r[3] * p.x + r[4] * p.y + r[5] * p.z + t.t[1],
            r[6] * p.x + r[7] * p.y + r[8] * p.z + t.t[2]};
}

template <typename F>
bool throws_capability(F&& f) {
    try {
        f();
    } catch (const TProjectionCapabilityError&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

template <typename F>
bool throws_fault(F&& f) {
    try {
        f();
    } catch (const TProjectionCapabilityError&) {
        return false;
    } catch (...) {
        return true;
    }
    return false;
}

}  // namespace

int main() {
    const auto registration = std::make_shared<const RgbDepthRegistration>(1280, 800, 611.813, 611.809, 640.654,
                                                                           400.484, kDistortion);
    const TProjectionSession session = session_18(registration);
    const auto ready = setup_t_projection(session, kYamlLevel, 7);
    CHECK(ready.projection.width == 1280 && ready.projection.height == 800 && ready.projection.revision == 7 &&
          ready.projection.context == ready.color.get() && ready.projection.project != nullptr);
    // Split form (round 2, 2026-10-04): the rigid part is the context's own colour <- base, and
    // project_camera(R p + t) is project(p) bit for bit (same transform arithmetic, same tail).
    CHECK(ready.projection.rotation == &ready.color->R_color_base &&
          ready.projection.translation == &ready.color->t_color_base && ready.projection.project_camera != nullptr);
    {
        const auto& r = *ready.projection.rotation;
        const auto& t = *ready.projection.translation;
        std::size_t same = 0, total = 0;
        for (double x = 0.3; x < 6.0; x += 0.37)
            for (double y = -3.0; y < 3.0; y += 0.29)
                for (double z = -0.2; z < 1.5; z += 0.41) {
                    const Vec3 p{x, y, z};
                    const Vec3 q{r[0] * p.x + r[1] * p.y + r[2] * p.z + t.x, r[3] * p.x + r[4] * p.y + r[5] * p.z + t.y,
                                 r[6] * p.x + r[7] * p.y + r[8] * p.z + t.z};
                    double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                    const bool a = ready.projection.project(ready.projection.context, p, u1, v1);
                    const bool b = ready.projection.project_camera(ready.projection.context, q, u2, v2);
                    ++total;
                    same += (a == b && u1 == u2 && v1 == v2) ? 1 : 0;
                }
        CHECK(total > 1000 && same == total);
    }

    // 1. Round trip against the registration's undistortion (the Objects path).
    const auto base_from_colour =
        perception::rns::compose(base_from_depth(kYamlLevel), perception::rns::inverse(colour_from_depth(session)));
    double worst = 0;
    int tested = 0, floor_mismatch = 0;
    for (int v = 0; v < 800; v += 23) {
        for (int u = 0; u < 1280; u += 37) {
            for (const int uu : {u, 1279 - u % 3}) {
                double x = 0, y = 0;
                if (!registration->undistort_pixel(uu, v, x, y)) continue;
                for (const double z : {0.6, 1.5, 4.0}) {
                    const Vec3 base_point = apply(base_from_colour, {x * z, y * z, z});
                    double pu = 0, pv = 0;
                    const bool ok = ready.projection.project(ready.projection.context, base_point, pu, pv);
                    CHECK(ok);
                    worst = std::max(worst, std::max(std::abs(pu - (uu + 0.5)), std::abs(pv - (v + 0.5))));
                    if (static_cast<int>(std::floor(pu)) != uu || static_cast<int>(std::floor(pv)) != v) ++floor_mismatch;
                    ++tested;
                }
            }
        }
    }
    CHECK(tested > 3000);
    CHECK(worst < 2e-3);
    CHECK(floor_mismatch == 0);
    std::cout << "round trip: points=" << tested << " worst_px=" << worst << " floor_mismatch=" << floor_mismatch << "\n";

    // 2. The configured base <- depth cancels.
    const auto pitched = setup_t_projection(session, kYamlPitched, 7);
    const auto b1 = base_from_depth(kYamlLevel), b2 = base_from_depth(kYamlPitched);
    double spread = 0;
    for (const Vec3 depth_point : {Vec3{0.3, 0.2, 1.1}, Vec3{-1.2, 0.5, 3.0}, Vec3{0.05, -0.3, 0.7}, Vec3{2.0, 0.9, 5.5}}) {
        double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
        CHECK(ready.projection.project(ready.projection.context, apply(b1, depth_point), u1, v1));
        CHECK(pitched.projection.project(pitched.projection.context, apply(b2, depth_point), u2, v2));
        spread = std::max(spread, std::max(std::abs(u1 - u2), std::abs(v1 - v2)));
    }
    CHECK(spread < 1e-6);
    std::cout << "base<-depth cancels: max_difference_px=" << spread << "\n";

    // 3. Zero distortion equals the rectified pinhole reference (+0.5).
    CalibratedColor pinhole = *ready.color;
    pinhole.distortion = StandardDistortion{};
    RectifiedColor rectified;
    rectified.R_color_base = pinhole.R_color_base;
    rectified.t_color_base = pinhole.t_color_base;
    rectified.fx = pinhole.fx;
    rectified.fy = pinhole.fy;
    rectified.cx = pinhole.cx;
    rectified.cy = pinhole.cy;
    for (const Vec3 p : {Vec3{2.0, 0.3, 0.0}, Vec3{1.0, -0.4, 0.1}, Vec3{3.5, 1.0, -0.2}}) {
        double a = 0, b = 0, c = 0, d = 0;
        CHECK(perception::geometry::project_calibrated_color(&pinhole, p, a, b));
        CHECK(perception::geometry::project_rectified_color(&rectified, p, c, d));
        CHECK(std::abs(a - (c + 0.5)) < 1e-9 && std::abs(b - (d + 0.5)) < 1e-9);
    }

    // 4. Behind the camera, and far beyond the image corners, is no sample.
    double u = 0, v = 0;
    CHECK(!ready.projection.project(ready.projection.context, {-1.0, 0.0, 0.8}, u, v));   // behind (camera looks +x)
    CHECK(!ready.projection.project(ready.projection.context, {0.5, 3.0, 0.8}, u, v));    // 80 degrees off axis
    CHECK(ready.color->max_r2 > 0.5 && ready.color->max_r2 < 2.0);
    CHECK(!ready.projection.project(nullptr, {1.0, 0.0, 0.0}, u, v));

    // 5. Capability gaps and faults.
    CHECK(throws_capability([&] { auto s = session; s.present = false; (void)setup_t_projection(s, kYamlLevel, 1); }));
    CHECK(throws_capability([&] {
        auto s = session;
        s.registration.reset();
        s.registration_status = "unavailable: firmware 1.8.16";
        (void)setup_t_projection(s, kYamlLevel, 1);
    }));
    CHECK(throws_capability([&] { auto s = session; s.width = 1920; (void)setup_t_projection(s, kYamlLevel, 1); }));
    CHECK(throws_capability([&] { auto s = session; s.distortion_model = 3; (void)setup_t_projection(s, kYamlLevel, 1); }));
    CHECK(throws_capability([&] { auto s = session; s.distortion[3] = 0.01F; (void)setup_t_projection(s, kYamlLevel, 1); }));
    CHECK(throws_capability([&] { auto s = session; s.depth_to_colour_R[0] = 1.2F; (void)setup_t_projection(s, kYamlLevel, 1); }));
    CHECK(throws_fault([&] { (void)setup_t_projection(session, "perception:\n  other: 1\n", 1); }));

    std::cout << "test_t_projection_setup: " << (failures == 0 ? "PASS" : "FAIL") << " checks=" << checks
              << " failures=" << failures << "\n";
    return failures == 0 ? 0 : 1;
}
