/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_orbbec_native_rays.cpp
 * Brief: Unit tests for the native depth ray table built offline with the real Orbbec SDK math.
 *
 * Description:
 * What it checks. Zero distortion over every pixel of the recorded profile; nonzero Brown
 * distortion against an independent forward projection; invalid profiles are refused; an SDK
 * success that still leaves invalid pixels is refused; the factory's binding copy outlives its
 * setup; no extrinsic means no defaults.
 * Recorded native K/scale are fixture inputs, not a current-device observation or an extrinsic
 * calibration certificate. Input provenance:
 * rns_native_target_20260911.H4xoX4/attempt3_evidence/camera.stdout.log (SHA256
 * fc289af1d66306c4b4dc177b6c3c6e007b94d08bab0adef212589a215672e2b0).
 * No Context/device/Pipeline/model/inference queue/network is constructed. The geometry lifetime
 * test uses only a synthetic in-process depth Mailbox.
 */

#include "modules/orbbec_native_rays.hpp"
#include <libobsensor/hpp/Utils.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace nd = perception::native_depth;
using namespace perception::geometry;

std::uint64_t checks = 0;
std::size_t groups = 0, rejected_profiles = 0, invalid_sdk_cases = 0;
std::size_t sdk_false_cases = 0;
std::size_t invalid_sdk_pixels = 0, zero_pixels = 0, distorted_pixels = 0;
double zero_max_error = 0, brown_max_pixel_error = 0, brown_max_pinhole_difference = 0;

void check(bool condition, const char* expression, int line) {
    ++checks;
    if (!condition)
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
}
#define CHECK(expression) check(static_cast<bool>(expression), #expression, __LINE__)

bool same_float(float a, float b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

void check_binding(const ProjectionBinding& a, const ProjectionBinding& b) {
    CHECK(a.intrinsics.width == b.intrinsics.width && a.intrinsics.height == b.intrinsics.height);
    CHECK(same_float(a.intrinsics.fx, b.intrinsics.fx));
    CHECK(same_float(a.intrinsics.fy, b.intrinsics.fy));
    CHECK(same_float(a.intrinsics.cx, b.intrinsics.cx));
    CHECK(same_float(a.intrinsics.cy, b.intrinsics.cy));
    CHECK(same_float(a.unit_m, b.unit_m));
    CHECK(a.distortion.reported == b.distortion.reported && a.distortion.model == b.distortion.model);
    for (std::size_t i = 0; i < a.distortion.coefficients.size(); ++i)
        CHECK(same_float(a.distortion.coefficients[i], b.distortion.coefficients[i]));
    CHECK(a.R_base_camera == b.R_base_camera);
    CHECK(a.t_base_camera.x == b.t_base_camera.x && a.t_base_camera.y == b.t_base_camera.y &&
          a.t_base_camera.z == b.t_base_camera.z);
}

ProjectionBinding recorded_binding() {
    ProjectionBinding b;
    b.intrinsics = {308.1480712890625F, 308.1480712890625F, 317.25F, 202.75F, 640, 400};
    b.distortion.reported = true;
    b.distortion.model = 3; // Recorded Brown-Conrady, explicitly not inferred from zero coefficients.
    b.unit_m = 0.0010000000474974513F;
    // SYNTHETIC mounting for the geometry lifetime test only:
    // optical +Z -> base +X, optical +X -> base -Y, optical +Y -> base -Z.
    b.R_base_camera = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    b.t_base_camera = {0, 0, 1};
    return b;
}

GeometryOptions synthetic_options() {
    GeometryOptions o;
    o.prior = {{0, 0, 1}, 0};
    o.fit.inlier_distance_m = .015;
    o.fit.min_inlier_fraction = .8;
    o.fit.max_inlier_rms_m = .005;
    o.fit.min_xy_stddev_m = .01;
    o.fit.min_up_dot = .5;
    o.fit.max_prior_angle_rad = .3;
    o.fit.min_inliers = 100;
    o.fit.hypotheses = 32;
    o.fit.seed = 0x243f6a8885a308d3ULL;
    o.fit_near_min_m = .7;
    o.fit_near_max_m = 2.9;
    o.fit_prior_band_m = .25;
    o.range_max_m = 6;
    o.blind_near_lo_m = .01;
    o.h_tol_m = .125;
    o.z_pass_m = .5;
    o.eps_neg = .1;
    o.w_h_m = .1;
    o.angle_min_rad = -1.55;
    o.angle_step_rad = .01;
    o.n_bins = 311;
    return o;
}

template<class Fn> void group(const char* name, Fn fn) {
    fn();
    ++groups;
    std::cout << "ORBBEC_RAYS_GROUP {\"name\":\"" << name << "\",\"passed\":true}\n";
}

void zero_distortion() {
    const auto b = recorded_binding();
    const OrbbecNativeRays factory(b);
    check_binding(factory.binding(), b);
    CHECK(factory.rays().size() == kGeometryPixels);
    // SDK CoordinateUtil.cpp:43-44 starts with float subtraction/division,
    // followed by double iteration. Four float epsilons times coordinate
    // magnitude bound numerical roundoff here, NOT calibration error.
    constexpr double rounding = 4.0 * std::numeric_limits<float>::epsilon();
    for (std::size_t v = 0; v < kNativeHeight; ++v) {
        for (std::size_t u = 0; u < kNativeWidth; ++u) {
            const auto& ray = factory.rays()[v * kNativeWidth + u];
            const double x = (static_cast<double>(u) - b.intrinsics.cx) / b.intrinsics.fx;
            const double y = (static_cast<double>(v) - b.intrinsics.cy) / b.intrinsics.fy;
            CHECK(std::isfinite(ray.x) && std::isfinite(ray.y) && ray.z == 1.0);
            CHECK(std::abs(ray.x - x) <= rounding * std::max(1.0, std::abs(x)));
            CHECK(std::abs(ray.y - y) <= rounding * std::max(1.0, std::abs(y)));
            zero_max_error = std::max({zero_max_error, std::abs(ray.x - x), std::abs(ray.y - y)});
            ++zero_pixels;
        }
    }
    CHECK(zero_pixels == 256000);
    // A focal ray table is dimensionless: depth scale must not enter its rays.
    auto scaled = b;
    scaled.unit_m *= 2;
    const OrbbecNativeRays other_scale(scaled);
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        CHECK(factory.rays()[i].x == other_scale.rays()[i].x);
        CHECK(factory.rays()[i].y == other_scale.rays()[i].y);
        CHECK(other_scale.rays()[i].z == 1.0);
    }
}

// Independent FORWARD Brown projection. Do not use deprecated SDK
// calibration2dTo3d as a nonzero-distortion oracle: it ignores distortion.
std::array<double, 2> forward_brown(const ProjectionBinding& b, const Vec3& ray) {
    const auto& c = b.distortion.coefficients;
    const double x = ray.x, y = ray.y, r2 = x*x + y*y;
    const double radial = 1.0 + c[0]*r2 + c[1]*r2*r2 + c[2]*r2*r2*r2;
    const double xd = x*radial + 2.0*c[6]*x*y + c[7]*(r2 + 2.0*x*x);
    const double yd = y*radial + c[6]*(r2 + 2.0*y*y) + 2.0*c[7]*x*y;
    return {xd*b.intrinsics.fx + b.intrinsics.cx, yd*b.intrinsics.fy + b.intrinsics.cy};
}

void nonzero_brown(std::array<float, 8> coefficients) {
    auto b = recorded_binding();
    b.distortion.coefficients = coefficients;
    const OrbbecNativeRays factory(b);
    check_binding(factory.binding(), b);
    CHECK(factory.rays().size() == kGeometryPixels);
    double maximum = 0, difference = 0;
    for (std::size_t v = 0; v < kNativeHeight; ++v) {
        for (std::size_t u = 0; u < kNativeWidth; ++u) {
            const auto& ray = factory.rays()[v * kNativeWidth + u];
            CHECK(std::isfinite(ray.x) && std::isfinite(ray.y) && ray.z == 1.0);
            const auto pixel = forward_brown(b, ray);
            const double error = std::hypot(pixel[0] - static_cast<double>(u), pixel[1] - static_cast<double>(v));
            CHECK(std::isfinite(error) && error <= .5); // SDK inverse pixel-validity bound, not camera accuracy.
            maximum = std::max(maximum, error);
            const double px = (static_cast<double>(u) - b.intrinsics.cx) / b.intrinsics.fx;
            const double py = (static_cast<double>(v) - b.intrinsics.cy) / b.intrinsics.fy;
            difference = std::max(difference, std::hypot(ray.x-px, ray.y-py));
            ++distorted_pixels;
        }
    }
    CHECK(difference > .001); // Off-axis rays must not silently take the zero-distortion branch.
    brown_max_pixel_error = std::max(brown_max_pixel_error, maximum);
    brown_max_pinhole_difference = std::max(brown_max_pinhole_difference, difference);
}

template<class Change> void reject_profile(Change change) {
    auto b = recorded_binding();
    change(b);
    bool rejected = false;
    try { const OrbbecNativeRays invalid(b); }
    catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
    ++rejected_profiles;
}

void invalid_profiles() {
    reject_profile([](ProjectionBinding& b) { b.intrinsics.width = 0; });
    reject_profile([](ProjectionBinding& b) { b.intrinsics.width = 1280; });
    reject_profile([](ProjectionBinding& b) { b.intrinsics.height = 401; });
    reject_profile([](ProjectionBinding& b) { b.intrinsics.width = 800; b.intrinsics.height = 320; });
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (float value : {0.F, -1.F, nan, inf}) {
        reject_profile([value](ProjectionBinding& b) { b.intrinsics.fx = value; });
        reject_profile([value](ProjectionBinding& b) { b.intrinsics.fy = value; });
        reject_profile([value](ProjectionBinding& b) { b.unit_m = value; });
    }
    for (float value : {nan, inf}) {
        reject_profile([value](ProjectionBinding& b) { b.intrinsics.cx = value; });
        reject_profile([value](ProjectionBinding& b) { b.intrinsics.cy = value; });
        for (std::size_t i = 0; i < 8; ++i)
            reject_profile([value, i](ProjectionBinding& b) { b.distortion.coefficients[i] = value; });
    }
    reject_profile([](ProjectionBinding& b) { b.distortion.reported = false; });
    for (int model : {-1, 0, 1, 2, 4, 5, 6, std::numeric_limits<int>::max()})
        reject_profile([model](ProjectionBinding& b) { b.distortion.model = model; });
    for (std::size_t i = 3; i < 6; ++i)
        reject_profile([i](ProjectionBinding& b) { b.distortion.coefficients[i] = .001F; });
}

struct TableObservation { bool success = false; std::size_t invalid = 0; };
TableObservation observe_raw_sdk_table(const ProjectionBinding& b) {
    OBCalibrationParam calibration{};
    const auto& k = b.intrinsics;
    const auto& c = b.distortion.coefficients;
    calibration.intrinsics[OB_SENSOR_DEPTH] = {k.fx, k.fy, k.cx, k.cy, 640, 400};
    calibration.distortion[OB_SENSOR_DEPTH] = {
        c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], OB_DISTORTION_BROWN_CONRADY};
    std::vector<float> buffer(2*kGeometryPixels, std::numeric_limits<float>::quiet_NaN());
    std::uint32_t capacity = static_cast<std::uint32_t>(buffer.size());
    OBXYTables table{};
    TableObservation result;
    result.success = ob::CoordinateTransformHelper::transformationInitXYTables(
        calibration, OB_SENSOR_DEPTH, buffer.data(), &capacity, &table);
    if (!result.success) return result;
    CHECK(capacity == buffer.size() && table.width == 640 && table.height == 400);
    CHECK(table.xTable == buffer.data() && table.yTable == buffer.data()+kGeometryPixels);
    for (std::size_t i = 0; i < kGeometryPixels; ++i)
        result.invalid += !std::isfinite(table.xTable[i]) || !std::isfinite(table.yTable[i]);
    return result;
}

void partial_sdk_success() {
    // Exactly two bounded numerical fixtures; no coefficient search or camera.
    for (int choice = 0; choice < 2; ++choice) {
        auto b = recorded_binding();
        if (choice == 0) b.intrinsics.fx = b.intrinsics.fy = 1e-30F;
        else b.distortion.coefficients[0] = std::numeric_limits<float>::max();
        const auto observed = observe_raw_sdk_table(b);
        std::cout << "ORBBEC_RAYS_INVALID_FIXTURE {\"choice\":" << choice
                  << ",\"name\":\"" << (choice == 0 ? "tiny_positive_focal" : "finite_max_k1")
                  << "\",\"sdk_success\":" << (observed.success ? "true" : "false")
                  << ",\"invalid_pixels\":";
        if (observed.success) std::cout << observed.invalid;
        else std::cout << "null";
        std::cout << ",\"partial_failure_observed\":"
                  << (observed.success && observed.invalid > 0 ? "true" : "false") << "}\n";
        if (!observed.success) {
            bool rejected = false;
            try { const OrbbecNativeRays invalid(b); }
            catch (const std::runtime_error& error) {
                rejected = std::string(error.what()) == "native rays: SDK table failure or unexpected layout";
            }
            CHECK(rejected); // SDK-level failure must also fail closed, but is not partial-pixel coverage.
            ++sdk_false_cases;
            continue;
        }
        if (observed.invalid == 0) continue;
        bool rejected = false;
        try { const OrbbecNativeRays invalid(b); }
        catch (const std::runtime_error& error) {
            rejected = std::string(error.what()).find("native rays: invalid SDK pixel at ") == 0;
        }
        CHECK(rejected); // Must be per-pixel rejection, not a metadata/schema error.
        ++invalid_sdk_cases;
        invalid_sdk_pixels += observed.invalid;
    }
    // Neither SDK false nor an all-valid table covers the required regression.
    // If both bounded choices miss it, fail visibly instead of calling it PASS.
    CHECK(invalid_sdk_cases > 0 && invalid_sdk_pixels > 0);
}

void factory_lifetime_and_geometry() {
    auto caller = recorded_binding();
    const auto frozen = caller;
    std::vector<Vec3> rays;
    std::unique_ptr<NativeGeometry> geometry;
    {
        OrbbecNativeRays factory(caller);
        rays = factory.rays();
        caller.intrinsics.fx += 1;
        caller.intrinsics.width = 320;
        caller.distortion.coefficients[0] = .2F;
        caller.unit_m *= 2;
        caller.R_base_camera.fill(0);
        caller.t_base_camera.z = 100;
        check_binding(factory.binding(), frozen);
        geometry = factory.make_geometry(synthetic_options());
    }
    CHECK(geometry != nullptr);
    // Exercise the copy after setup destruction. In sanitizer builds this also
    // exposes any stale factory-vector reference in the NativeGeometry path.
    const std::vector<Vec3> overwrite(kGeometryPixels, Vec3{9, 9, 9});
    CHECK(overwrite.back().z == 9);
    std::vector<std::uint16_t> raster(kGeometryPixels, 0);
    for (std::size_t i = 0; i < rays.size(); ++i) {
        if (rays[i].y <= 0) continue;
        const double raw = (1.0/rays[i].y)/static_cast<double>(frozen.unit_m);
        if (!std::isfinite(raw) || raw < .5 || raw > 65535) continue;
        raster[i] = static_cast<std::uint16_t>(std::llround(raw));
    }
    nd::Mailbox mailbox(kGeometryPixels);
    nd::Frame frame(kGeometryPixels);
    std::uint64_t stream = mailbox.begin_stream(), sequence = 0;
    const auto offer = [&](bool mismatch) {
        nd::View view;
        view.pre_d2c = true;
        view.metadata.intrinsics = frozen.intrinsics;
        view.metadata.distortion = frozen.distortion;
        view.metadata.unit_m = mismatch ? frozen.unit_m*2 : frozen.unit_m;
        ++sequence;
        view.metadata.source = {stream, sequence, sequence, sequence, sequence*1000,
                                sequence*1000+1, static_cast<std::int64_t>(sequence*1000000)};
        view.pixels = raster.data();
        view.bytes = raster.size()*sizeof(std::uint16_t);
        CHECK(mailbox.try_offer(view) == nd::Offer::Accepted);
        CHECK(mailbox.try_take(frame) == nd::Take::Taken);
    };
    offer(false);
    CHECK(geometry->process(frame, mailbox).status == GeometryStatus::NormalEvidence);
    CHECK(geometry->result().visited_pixels == kGeometryPixels && geometry->result().has_raw_sample_evidence());
    CHECK(geometry->result().fit.plane.has_value() && geometry->result().fit.inlier_rms_m <= .005);
    const std::size_t pixel = 350*kNativeWidth+500;
    Vec3 point;
    CHECK(geometry->ground_point(pixel, point));
    const double optical_z = raster[pixel]*static_cast<double>(frozen.unit_m);
    CHECK(std::abs(point.x-optical_z) <= 1e-12);
    CHECK(std::abs(point.y+rays[pixel].x*optical_z) <= 1e-12);
    CHECK(std::abs(point.z-(1.0-rays[pixel].y*optical_z)) <= 1e-12);
    CHECK(std::abs(geometry->pixels()[pixel].range_m-std::hypot(point.x,point.y)) <= 1e-12);
    CHECK(std::abs(geometry->pixels()[pixel].range_m-optical_z) > .1);
    CHECK(geometry->process(frame, mailbox).status == GeometryStatus::InactiveOrStale);
    CHECK(!geometry->result().has_raw_sample_evidence());
    offer(true);
    CHECK(geometry->process(frame, mailbox).status == GeometryStatus::ProfileMismatch);
    CHECK(!geometry->result().has_raw_sample_evidence());
    offer(false);
    CHECK(geometry->process(frame, mailbox).status == GeometryStatus::NormalEvidence);
    mailbox.end_stream();
    CHECK(geometry->process(frame, mailbox).status == GeometryStatus::InactiveOrStale);
    stream = mailbox.begin_stream();
    sequence = 0;
    offer(false);
    CHECK(geometry->process(frame, mailbox).status == GeometryStatus::NormalEvidence);
    CHECK(geometry->result().source.stream_id == stream);
    mailbox.end_stream();
}

void no_extrinsic_defaults() {
    auto b = recorded_binding();
    b.R_base_camera.fill(0);
    const OrbbecNativeRays factory(b); // Ray construction alone is not an SE(3) certificate.
    bool rejected = false;
    try { const auto geometry = factory.make_geometry(synthetic_options()); }
    catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
}
} // namespace

int main() {
    try {
        group("recorded_native_zero_all_pixels", zero_distortion);
        group("synthetic_brown_mixed_forward_oracle", [] {
            nonzero_brown({.05F, -.01F, .002F, 0, 0, 0, .001F, -.0015F});
        });
        group("synthetic_brown_k3_only_forward_oracle", [] {
            nonzero_brown({0, 0, .003F, 0, 0, 0, 0, 0});
        });
        group("synthetic_brown_tangential_only_forward_oracle", [] {
            nonzero_brown({0, 0, 0, 0, 0, 0, .002F, -.0015F});
        });
        group("invalid_profile_rejection", invalid_profiles);
        group("sdk_success_with_invalid_pixels_rejected", partial_sdk_success);
        group("factory_owned_binding_copy_lifetime_and_source_gates", factory_lifetime_and_geometry);
        group("no_extrinsic_defaults", no_extrinsic_defaults);
        CHECK(zero_pixels == 256000 && distorted_pixels == 3*256000);
        CHECK(std::isfinite(zero_max_error) && std::isfinite(brown_max_pixel_error) &&
              std::isfinite(brown_max_pinhole_difference));
        std::cout << std::setprecision(17)
            << "ORBBEC_RAYS_RESULT {\"status\":\"OFFLINE_SDK_NUMERICAL_PASS\",\"groups\":" << groups
            << ",\"checks\":" << checks << ",\"zero_pixels\":" << zero_pixels
            << ",\"nonzero_pixels\":" << distorted_pixels << ",\"rejected_profiles\":" << rejected_profiles
            << ",\"sdk_invalid_fixture_choices\":2,\"sdk_success_invalid_cases\":" << invalid_sdk_cases
            << ",\"sdk_false_cases_rejected\":" << sdk_false_cases
            << ",\"sdk_invalid_pixels\":" << invalid_sdk_pixels << ",\"zero_max_ray_error\":" << zero_max_error
            << ",\"brown_max_pixel_error\":" << brown_max_pixel_error
            << ",\"brown_max_pinhole_difference\":" << brown_max_pinhole_difference
            << ",\"context_created\":false,\"device_opened\":false,\"model_loaded\":false"
            << ",\"inference_ran\":false,\"inference_enqueues\":0,\"network_used\":false"
            << ",\"synthetic_depth_mailbox_used\":true"
            << ",\"physical_calibration_claim\":false,\"twenty_hz_accepted\":false}\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "orbbec native rays regression: " << error.what() << '\n';
        std::cout << "ORBBEC_RAYS_RESULT {\"status\":\"FAIL\",\"groups\":" << groups
                  << ",\"checks\":" << checks << ",\"device_opened\":false}\n";
        return 1;
    }
}
