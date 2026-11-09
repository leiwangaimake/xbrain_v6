/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: native_geometry_parent.hpp
 * Brief: Declaration of the frozen parent NativeGeometry, the oracle for the pixel-record tests.
 *
 * Description:
 * Used only by test_native_geometry_pixel_record (namespace perception::geometry::native_geometry_parent).
 * The class is NativeGeometry as it was at commit 33a3032, before the fast path stopped writing
 * the pixel record in the scan: its scan writes every record and caches bin and range in it, and
 * its classification pass completes the record in place. The evidence types, the projection and
 * options checks and fit_ground are the production ones (native_geometry.hpp, ground_fit.cpp);
 * only the class is copied.
 */

#pragma once

#include "modules/native_geometry.hpp"

namespace perception::geometry::native_geometry_parent {
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
    // Whether the depth-independent azimuth path is active for this binding.
    bool fast_azimuth() const noexcept { return fast_azimuth_; }
    int reference_azimuth_bin(Vec3 p) const noexcept { return reference_bin_of(p); }
    // Numerically projected prior-plane ray ticket, including invalid-depth
    // pixels. Immutable setup provenance, NOT current-frame ground evidence.
    bool prior_ground_projection(std::size_t pixel, int& bin, double& range_m) const noexcept;
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
    int bin_of(Vec3) const noexcept;
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
}  // namespace perception::geometry::native_geometry_parent
