/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rgb_depth_registration.cpp
 * Brief: Unit tests for registering the aligned depth raster onto the distorted RGB image (B4-2).
 *
 * Description:
 * What it checks. SDK software alignment produces a pinhole depth raster while the RGB image
 * follows the standard distortion model. Cases: zero distortion gives the identity mapping; the
 * forward residual with measured coefficients is checked against an independent formula; the
 * radial-only model against a bisection inverse; sub-pixel undistortion against the per-pixel
 * table; the pinhole back-projection; apply() gathering exactly the indexed aligned value and 0
 * outside; and the refusals.
 */

/* Host test: RGB <- aligned depth registration (B4-2 option 1). No SDK/OpenCV.
 * Independent references inside this test: its own distortion formula, a
 * radial bisection inverse for the radial-only model, and brute index checks. */
#include "modules/rgb_depth_registration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using perception::RgbDepthRegistration;
using perception::StandardDistortion;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

// .23 Gemini 338Le colour calibration (evidence/segment2/b42_samples meta.json)
constexpr int W = 1280, H = 800;
constexpr double FX = 611.812805, FY = 611.809082, CX = 640.65448, CY = 400.484406;
const StandardDistortion kMeasured{-0.0325959213, 0.0366756804, -0.0124715716, 4.03675767e-06, -6.68168141e-05};

// Independent reference: the standard distortion model (k1 k2 k3 radial, p1 p2 tangential) applied
// to a normalised point, written out here.
void test_distort(const StandardDistortion& d, double x, double y, double& xd, double& yd) {
    const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    const double k = 1.0 + d.k1 * r2 + d.k2 * r4 + d.k3 * r6;
    xd = x * k + 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x);
    yd = y * k + d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y;
}

bool throws(const StandardDistortion& d, double fx = FX, int w = W) {
    try { RgbDepthRegistration r(w, H, fx, FY, CX, CY, d); } catch (const std::invalid_argument&) { return true; }
    return false;
}
}  // namespace

int main() {
    // ---- zero distortion: identity mapping, pinhole rays ----
    {
        RgbDepthRegistration r(64, 40, 50.0, 50.0, 31.5, 19.5, {});
        bool identity = true, rays = true;
        for (int v = 0; v < 40; ++v)
            for (int u = 0; u < 64; ++u) {
                const std::size_t i = static_cast<std::size_t>(v * 64 + u);
                identity &= r.aligned_index(i) == static_cast<std::int32_t>(i);
                rays &= std::fabs(r.ray_x()[i] - static_cast<float>((u - 31.5) / 50.0)) <= 1e-7F &&
                        std::fabs(r.ray_y()[i] - static_cast<float>((v - 19.5) / 50.0)) <= 1e-7F;
            }
        CHECK(identity && rays && r.outside_pixels() == 0 && r.max_shift_px() < 1e-9);
    }
    // ---- measured coefficients: forward residual with an independent formula ----
    RgbDepthRegistration reg(W, H, FX, FY, CX, CY, kMeasured);
    {
        double worst = 0;
        bool index_ok = true;
        for (int v = 0; v < H; v += 3)
            for (int u = 0; u < W; u += 3) {
                const std::size_t i = static_cast<std::size_t>(v) * W + static_cast<std::size_t>(u);
                double xd, yd;
                test_distort(kMeasured, reg.ray_x()[i], reg.ray_y()[i], xd, yd);
                // float32 ray storage: allow its rounding (~1e-7 normalized ~ 6e-5 px)
                worst = std::max(worst, std::max(std::fabs(xd * FX + CX - u), std::fabs(yd * FY + CY - v)));
                const long iu = std::lround(FX * reg.ray_x()[i] + CX), iv = std::lround(FY * reg.ray_y()[i] + CY);
                const std::int32_t expect = (iu < 0 || iv < 0 || iu >= W || iv >= H) ? -1 : static_cast<std::int32_t>(iv * W + iu);
                // double vs float ray may round a .5 boundary differently: accept a one-pixel neighbour there
                const std::int32_t got = reg.aligned_index(i);
                if (got != expect) {
                    const bool near = got >= 0 && expect >= 0 && std::abs(got % W - expect % W) <= 1 && std::abs(got / W - expect / W) <= 1;
                    index_ok &= near;
                }
            }
        CHECK(worst < 1e-3);
        CHECK(index_ok);
        CHECK(reg.max_forward_residual_px() <= 1e-6);
        // barrel-type net shift at the corners for these coefficients: several pixels, some outside
        CHECK(reg.max_shift_px() > 4.0 && reg.max_shift_px() < 15.0);
        CHECK(reg.outside_pixels() > 0 && reg.outside_pixels() < static_cast<std::size_t>(W * H) / 50);
        // principal point maps onto itself
        const std::size_t centre = static_cast<std::size_t>(std::lround(CY)) * W + static_cast<std::size_t>(std::lround(CX));
        CHECK(reg.aligned_index(centre) == static_cast<std::int32_t>(centre));
    }
    // ---- radial-only model against a bisection inverse ----
    {
        StandardDistortion radial = kMeasured;
        radial.p1 = radial.p2 = 0;
        RgbDepthRegistration r(W, H, FX, FY, CX, CY, radial);
        double worst = 0;
        for (int v = 0; v < H; v += 17)
            for (int u = 0; u < W; u += 13) {
                const double xd = (u - CX) / FX, yd = (v - CY) / FY, rd = std::hypot(xd, yd);
                double lo = 0, hi = 2.0 * rd + 1e-9;
                for (int k = 0; k < 200; ++k) {
                    const double mid = 0.5 * (lo + hi), m2 = mid * mid;
                    const double f = mid * (1 + radial.k1 * m2 + radial.k2 * m2 * m2 + radial.k3 * m2 * m2 * m2);
                    (f < rd ? lo : hi) = mid;
                }
                const double ru = 0.5 * (lo + hi);
                const double ex = rd > 0 ? xd * ru / rd : 0, ey = rd > 0 ? yd * ru / rd : 0;
                double x, y;
                CHECK(r.undistort_pixel(u, v, x, y));
                worst = std::max(worst, std::max(std::fabs(x - ex), std::fabs(y - ey)));
            }
        CHECK(worst < 1e-10);
    }
    // ---- sub-pixel undistortion agrees with the per-pixel table ----
    {
        double x, y;
        bool same = true;
        for (int v = 0; v < H; v += 97)
            for (int u = 0; u < W; u += 101) {
                same &= reg.undistort_pixel(u, v, x, y) &&
                        static_cast<float>(x) == reg.ray_x()[static_cast<std::size_t>(v) * W + static_cast<std::size_t>(u)] &&
                        static_cast<float>(y) == reg.ray_y()[static_cast<std::size_t>(v) * W + static_cast<std::size_t>(u)];
            }
        CHECK(same);
        CHECK(!reg.undistort_pixel(std::numeric_limits<double>::quiet_NaN(), 3.0, x, y));
    }
    // ---- pinhole back-projected point -> undistorted back-projection ----
    {
        bool ok = true;
        for (int v = 5; v < H; v += 131)
            for (int u = 3; u < W; u += 157) {
                const float z = 1.0F + static_cast<float>(u % 7);
                float x = static_cast<float>((u - CX) * z / FX), y = static_cast<float>((v - CY) * z / FY);
                ok &= perception::undistort_pinhole_point(reg, FX, FY, CX, CY, x, y, z);
                const std::size_t i = static_cast<std::size_t>(v) * W + static_cast<std::size_t>(u);
                ok &= std::fabs(x - reg.ray_x()[i] * z) <= 2e-5F * z && std::fabs(y - reg.ray_y()[i] * z) <= 2e-5F * z;
            }
        CHECK(ok);
        float x = 1.0F, y = 2.0F;
        CHECK(!perception::undistort_pinhole_point(reg, FX, FY, CX, CY, x, y, 0.0F) && x == 1.0F && y == 2.0F);
    }
    // ---- apply gathers exactly the indexed aligned value, 0 outside ----
    {
        std::vector<std::uint16_t> aligned(static_cast<std::size_t>(W) * H), out(aligned.size(), 7);
        for (std::size_t i = 0; i < aligned.size(); ++i) aligned[i] = static_cast<std::uint16_t>(1 + (i * 2654435761U) % 65000);
        reg.apply(aligned.data(), out.data());
        bool exact = true;
        std::size_t zeros = 0;
        for (std::size_t i = 0; i < out.size(); ++i) {
            const std::int32_t k = reg.aligned_index(i);
            exact &= k >= 0 ? out[i] == aligned[static_cast<std::size_t>(k)] : out[i] == 0;
            zeros += out[i] == 0;
        }
        CHECK(exact && zeros == reg.outside_pixels());
        // run-based apply is byte-identical to a per-pixel gather over
        // aligned_index() written here, on several random inputs and a
        // pre-filled output buffer
        bool identical = true;
        std::vector<std::uint16_t> ref(aligned.size());
        std::uint64_t seed = 0x9E3779B97F4A7C15ULL;
        for (int round = 0; round < 4; ++round) {
            for (auto& a : aligned) { seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17; a = static_cast<std::uint16_t>(seed); }
            std::fill(out.begin(), out.end(), static_cast<std::uint16_t>(0xBEEF));
            std::fill(ref.begin(), ref.end(), static_cast<std::uint16_t>(0x1234));
            reg.apply(aligned.data(), out.data());
            for (std::size_t i = 0; i < ref.size(); ++i) {
                const std::int32_t k = reg.aligned_index(i);
                ref[i] = k >= 0 ? aligned[static_cast<std::size_t>(k)] : 0;
            }
            identical &= out == ref;
        }
        CHECK(identical);
        CHECK(reg.run_count() > static_cast<std::size_t>(H) && reg.run_count() < static_cast<std::size_t>(W) * H / 20);
    }
    // ---- refusals ----
    {
        StandardDistortion strong{-1.0, 0, 0, 0, 0};    // folds over inside the raster
        CHECK(throws(strong));
        StandardDistortion nan = kMeasured;
        nan.k2 = std::numeric_limits<double>::quiet_NaN();
        CHECK(throws(nan));
        CHECK(throws(kMeasured, 0.0));
        CHECK(throws(kMeasured, FX, 0));
    }
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"rgb_depth_registration_host\",\"max_shift_px\":" << reg.max_shift_px()
              << ",\"outside_pixels\":" << reg.outside_pixels() << ",\"runs\":" << reg.run_count() << ",\"max_forward_residual_px\":" << reg.max_forward_residual_px()
              << "}" << std::endl;
    return failures ? 1 : 0;
}
