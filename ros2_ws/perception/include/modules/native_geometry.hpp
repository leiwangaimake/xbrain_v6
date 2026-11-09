/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: native_geometry.hpp
 * Brief: Normal-plane and independent raw-sample evidence ONLY.
 *
 * Description:
 * Not a ProfileBuilder or navigation decision. No FREE/T/negative-obstacle confirmation, Q4 fallback, clock estimation, production configuration loading, or calibration certification.
 */

#pragma once

#include "modules/ground_fit.hpp"
#include "modules/native_depth.hpp"
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace perception::geometry {

inline constexpr std::size_t kNativeWidth = 640, kNativeHeight = 400;
inline constexpr std::size_t kGeometryPixels = kNativeWidth * kNativeHeight;
inline constexpr std::size_t kGeometryMaxBins = 1024;

// Numerically supplied projection, not proof of calibrated extrinsics.
// Camera optical axes: +x right, +y down, +z forward. R_base_camera is row-major.
// The caller must supply distortion-corrected optical rays (z==1) for EXACTLY
// the bound native profile. No K-inverse or distortion-model guess is made here.
struct ProjectionBinding {
    native_depth::Intrinsics intrinsics;
    native_depth::Distortion distortion;
    float unit_m = 0;
    std::array<double, 9> R_base_camera{};
    Vec3 t_base_camera;
};
struct GeometryOptions {
    Plane prior;
    FitOptions fit;
    double fit_near_min_m = 0, fit_near_max_m = 0, fit_prior_band_m = 0;
    double range_max_m = 0, blind_near_lo_m = 0;
    double h_tol_m = 0, z_pass_m = 0, eps_neg = 0, w_h_m = 0;
    // angle_min is FIRST BIN CENTRE, domain is [first-step/2, last+step/2).
    double angle_min_rad = 0, angle_step_rad = 0;
    std::size_t n_bins = 0;
};

// Pure numerical setup checks, not calibration provenance or ray validation.
bool valid_projection_binding(const ProjectionBinding&) noexcept;
bool valid_geometry_options(const GeometryOptions&) noexcept;

enum class SampleClass : std::uint8_t {
    Unclassified, InvalidDepth, OutsideDomain, Ground, Obstacle, Overhead,
    BelowPlane, Count
};
struct PixelEvidence {
    SampleClass classification = SampleClass::Unclassified;
    int bin = -1;
    double range_m = 0, height_m = 0;
    // A raw crossing candidate, NOT a confirmed negative boundary. Uses the
    // CURRENT fitted plane and EXPECTED intersection domain, not return bin.
    int pierce_bin = -1;
    std::optional<double> pierce_range_m;
};
// A ray's expected ground cell input: the azimuth bin and horizontal range where it meets a plane
// (bin -1: none). NormalGrid's coverage tickets.
struct GroundTicket {
    int bin = -1;
    double range_m = 0;
};
struct BinEvidence {
    std::size_t valid_samples = 0, ground_samples = 0;
    std::optional<double> nearest_positive_m, positive_height_window_max_m;
    std::optional<double> nearest_below_m, below_min_height_m;
    std::optional<double> nearest_pierce_expected_m;
};
enum class GeometryStatus {
    NormalEvidence, InvalidFrame, InactiveOrStale, ProfileMismatch,
    FitFailed, NumericalFailure
};
struct GeometryResult {
    GeometryStatus status = GeometryStatus::InvalidFrame;
    native_depth::SourceId source;
    FitResult fit;
    std::size_t visited_pixels = 0, valid_depth_pixels = 0;
    // Numerically projected prior-plane ROI, not a certified calibration ROI.
    std::size_t roi_pixels = 0, invalid_roi_pixels = 0;
    std::array<std::size_t, static_cast<std::size_t>(SampleClass::Count)> classes{};
    std::optional<double> nearest_ground_m;
    // On error classified pixel/bin evidence and plane are cleared. Failure
    // metadata and diagnostic candidate/attempt counts remain visible.
    bool has_evidence() const noexcept { return status == GeometryStatus::NormalEvidence; }
    // The raw ledger is validated even if every bin contains zero samples.
    // It certifies current-frame sample presence only, never a fitted plane,
    // Ground/Obstacle/BelowPlane classification, FREE, or negative evidence.
    bool has_raw_sample_evidence() const noexcept {
        return raw_sample_evidence_valid &&
            (status == GeometryStatus::NormalEvidence || status == GeometryStatus::FitFailed);
    }
    bool raw_sample_evidence_valid = false;
    // Diagnostic stage timestamps (CLOCK_MONOTONIC ns, -1 = not reached):
    // [0] entry [1] evidence cleared [2] projection scan [3] fit [4] classify [5] height window.
    std::array<std::int64_t, 6> stage_ns{{-1, -1, -1, -1, -1, -1}};
};

class NativeGeometry {
public:
    // Bounded setup allocations; copies all rays. Constructor rejects missing
    // or invalid options/binding. No built-in deployment thresholds.
    NativeGeometry(const ProjectionBinding&, const Vec3* optical_rays,
                   std::size_t ray_count, const GeometryOptions&);
    NativeGeometry(const NativeGeometry&) = delete;
    NativeGeometry& operator=(const NativeGeometry&) = delete;

    // One consumer / one Mailbox owner for this object's entire lifetime.
    // Active stream is checked before AND after computation; restart creates
    // a new generation, replay/out-of-order source is rejected. No frame data
    // is retained by reference. Caller must recheck stream before later publish.
    // Zero steady-state allocation, fixed scan/hypothesis bounds, no locks here.
    const GeometryResult& process(const native_depth::Frame&,
                                  const native_depth::Mailbox&) noexcept;
    const GeometryResult& result() const noexcept { return result_; }
    bool bound_to(const native_depth::Mailbox& m) const noexcept { return mailbox_ == &m; }
    // Valid until next process call; do not retain as an independent snapshot.
    const std::vector<PixelEvidence>& pixels() const noexcept { return pixels_; }
    const std::vector<BinEvidence>& bins() const noexcept { return bins_; }
    // Same lifetime and bin/domain definition as bins(); independent of fit
    // candidates and classifications. Use only with has_raw_sample_evidence().
    // Any invalid/stale/profile/numerical result clears every count.
    const std::vector<std::uint32_t>& raw_sample_counts() const noexcept { return raw_sample_counts_; }
    std::size_t storage_bytes() const noexcept;
    const GeometryOptions& options() const noexcept { return options_; }
    // Azimuth bin used for all evidence (fast exact path) and the atan2
    // reference it must equal; exposed for equivalence tests only.
    int azimuth_bin(Vec3 p) const noexcept { return bin_of(p); }
    // The same search started from `hint` (the scan's previous bin); equals azimuth_bin for any hint.
    int azimuth_bin(Vec3 p, int hint) const noexcept { return bin_of(p, hint); }
    // The general-path scan's own lookup (G-P1 phase 2): the hint and up to two neighbours inline,
    // otherwise bin_of from the hint; equals azimuth_bin for any hint. `inline_decided`, when given,
    // tells whether the inline step decided. For tests.
    int scan_azimuth_bin(Vec3 p, int hint, bool* inline_decided = nullptr) const noexcept;
    // The scan's horizontal range of (x, y); equals std::hypot(x, y) bit for bit. For tests.
    static double scan_range(double x, double y) noexcept;
    // Whether scan_range runs the inline copy of glibc 2.35's hypot (else it is std::hypot itself).
    static bool scan_range_inline() noexcept;
    // Whether the depth-independent azimuth path is active for this binding.
    bool fast_azimuth() const noexcept { return fast_azimuth_; }
    // Whether the setup proved every scan point bounded (finiteness checks skipped); for tests.
    bool bounded_scan() const noexcept { return scan_bounded_; }
    int reference_azimuth_bin(Vec3 p) const noexcept { return reference_bin_of(p); }
    // Numerically projected prior-plane ray ticket, including invalid-depth
    // pixels. Immutable setup provenance, NOT current-frame ground evidence.
    bool prior_ground_projection(std::size_t pixel, int& bin, double& range_m) const noexcept;
    // The same ticket for an arbitrary plane (2026-10-27: the current frame's fitted plane, NormalGrid
    // with fitted_tickets): only prior-ROI pixels hold one, with the same in-front, range-window and
    // bin rules; the range is sqrt(x*x + y*y) (not std::hypot, which can differ in the last bit).
    // `hint` only starts the bin search (any hint gives the same bin); -1 searches from scratch.
    bool plane_ground_projection(const Plane& plane, std::size_t pixel, int hint, int& bin,
                                 double& range_m) const noexcept;
    // -(n . t + d) of `plane`: the numerator of every ray's intersection, once per plane.
    double ticket_offset(const Plane& plane) const noexcept {
        const Vec3 t = binding_.t_base_camera;
        return -(plane.normal.x * t.x + plane.normal.y * t.y + plane.normal.z * t.z + plane.d);
    }
    // Everything ticket_on reads besides the pixel, for one plane: a caller's local copy, so a pass that
    // stores tickets and counts per pixel does not reload members it cannot prove unchanged.
    struct TicketPlane {
        Vec3 normal, t;
        double offset = 0, lo2 = 0, hi2 = 0;
        const Vec3* rays = nullptr;
        const double* boundary_cos = nullptr;  // null: no bracket walk (bin_of decides)
        const double* boundary_sin = nullptr;
        int n_bins = 0;
    };
    TicketPlane ticket_plane(const Plane& plane) const noexcept {
        TicketPlane c;
        c.normal = plane.normal;
        c.t = binding_.t_base_camera;
        c.offset = ticket_offset(plane);
        c.lo2 = options_.blind_near_lo_m * options_.blind_near_lo_m;
        c.hi2 = options_.range_max_m * options_.range_max_m;
        c.rays = rays_.data();
        c.boundary_cos = boundary_cos_.empty() ? nullptr : boundary_cos_.data();
        c.boundary_sin = boundary_sin_.empty() ? nullptr : boundary_sin_.data();
        c.n_bins = static_cast<int>(options_.n_bins);
        return c;
    }
    // The ticket of one pixel on the plane with this normal and ticket_offset (no ROI check), in
    // homogeneous form: the ray meets the plane in front of the camera at (x, y) = (xh, yh) / dh with
    // dh > 0, its horizontal range r = sqrt(xh^2 + yh^2) / dh lies within [blind_near_lo_m,
    // range_max_m] (tested as xh^2 + yh^2 against r^2 dh^2), and `bin` is its bin. Inline and without a
    // division or square root because NormalGrid runs it once per ROI pixel and frame (fitted tickets,
    // about 1e5 pixels; with them the pass took 3.7 ms on .26). The hint bin is tried first with the
    // bracket test of one step of bin_of's walk (both boundary crosses beyond the band; scaling x and y
    // by dh > 0 keeps every sign and the band), which gives the bin bin_of would; otherwise bin_of.
    bool ticket_on(const TicketPlane& c, std::size_t pixel, int hint, int& bin, double& xh, double& yh,
                   double& dh) const noexcept {
        const Vec3 ray = c.rays[pixel];
        const double d = c.normal.x * ray.x + c.normal.y * ray.y + c.normal.z * ray.z;
        // z = offset / d > 0: offset and d of one sign, neither zero (a NaN fails the test too).
        if (!(c.offset * d > 0) || !std::isfinite(d)) return false;
        const double sign = d > 0 ? 1.0 : -1.0;
        dh = sign * d;
        xh = sign * (c.t.x * d + c.offset * ray.x);
        yh = sign * (c.t.y * d + c.offset * ray.y);
        const double s = xh * xh + yh * yh, d2 = dh * dh;
        if (!std::isfinite(s) || s < c.lo2 * d2 || s > c.hi2 * d2) return false;
        int i = -1;
        if (hint >= 0 && hint < c.n_bins && c.boundary_cos) {
            const auto k = static_cast<std::size_t>(hint);
            const double band = 1e-12 * (std::abs(xh) + std::abs(yh));
            const double below = c.boundary_cos[k] * yh - c.boundary_sin[k] * xh;
            const double above = c.boundary_cos[k + 1] * yh - c.boundary_sin[k + 1] * xh;
            if (std::isfinite(band) && std::abs(below) > band && std::abs(above) > band && below > 0 && above < 0)
                i = hint;
        }
        if (i < 0) i = bin_of({xh / dh, yh / dh, 0}, hint);
        if (i < 0) return false;
        bin = i;
        return true;
    }
    // Setup-time prior-plane ROI (19 section 3.1 roi[v][u]: positive prior-plane
    // intersection within range_max_m), row-major native raster, values 0/1.
    // roi_pixels() > 0 is guaranteed by construction (PSC-7 table self-check).
    const std::vector<std::uint8_t>& prior_roi() const noexcept { return roi_; }
    std::size_t roi_pixels() const noexcept { return roi_pixels_; }
    // Current valid Ground point only; false clears out. Does not expose stale
    // storage after failure or allow T projection of obstacles/invalid depth.
    bool ground_point(std::size_t pixel, Vec3& out) const noexcept;
    // The depth behind ground_point() (same validity rules, same code * unit expression):
    // ground_point = base_rays()[pixel] * z + base_translation(). For a consumer that
    // precomputes per-pixel products of the ray (T's colour projection).
    bool ground_depth(std::size_t pixel, double& z) const noexcept;
    const std::vector<Vec3>& base_rays() const noexcept { return rays_; }
    const Vec3& base_translation() const noexcept { return binding_.t_base_camera; }

private:
    void clear_classified_evidence() noexcept;
    void clear_evidence() noexcept;
    void fail(GeometryStatus) noexcept;
    // Step 2 of process(): the raw ledger and the fit candidates; false after a numerical failure.
    bool scan(const native_depth::Frame&, std::size_t& candidate_count) noexcept;
    // hint in [0, n_bins): the bracket walk starts there instead of at the atan2 estimate.
    int bin_of(Vec3, int hint = -1) const noexcept;
    int reference_bin_of(Vec3) const noexcept;
    // Unit boundary directions at lo + k*step (k = 0..n_bins) for the exact
    // cross-product bin search; empty when the angular span is not < 3 rad.
    std::vector<double> boundary_cos_, boundary_sin_;
    bool profile_matches(const native_depth::Metadata&) const noexcept;
    ProjectionBinding binding_;
    GeometryOptions options_;
    std::vector<Vec3> rays_, candidates_;
    // Depth-independent azimuth (B6-1, reply batch 6): with no lateral offset
    // in base <- camera, p = (ray.x*z, ray.y*z, ray.z*z + t.z) and z > 0, so the
    // azimuth of every pixel -- and therefore its bin -- does not depend on the
    // depth. The bin, the radial factor and the prior-plane ray dot product are
    // then per-pixel constants computed once here. With ANY lateral offset the
    // constructor leaves this off and the general path runs unchanged.
    bool fast_azimuth_ = false;
    std::vector<int> pixel_bin_;
    std::vector<double> pixel_rho_;       // hypot(ray.x, ray.y)
    std::vector<double> pixel_prior_;     // dot(prior.normal, ray)
    double prior_const_ = 0;              // prior.normal.z * t.z + prior.d
    // Bounded scan (G-P1 phase 1): every point the scan can form is within kBoundedPoint (see the
    // constructor), so the scan's finiteness checks cannot fire and are skipped.
    bool scan_bounded_ = false;
    // Raw depth codes of the processed frame: ground_point() recomputes its point
    // with the same expression as the scan instead of storing 24 B per pixel
    // (no frame data is retained by reference).
    std::vector<std::uint16_t> codes_;
    std::vector<std::uint8_t> roi_;
    std::vector<PixelEvidence> pixels_;
    std::vector<BinEvidence> bins_;
    std::vector<std::uint32_t> raw_sample_counts_;
    // This frame's Obstacle pixels in pixel order, noted by the classification pass so the
    // height-window pass reads only them, not every pixel record (2026-10-06, same results).
    std::vector<std::uint32_t> obstacle_pixels_;
    GeometryResult result_;
    native_depth::SourceId last_;
    const native_depth::Mailbox* mailbox_ = nullptr;
    std::size_t roi_pixels_ = 0;
};

}  // namespace perception::geometry
