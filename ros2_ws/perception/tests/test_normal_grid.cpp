/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_normal_grid.cpp
 * Brief: Native Frame -> NativeGeometry -> NormalGrid synthetic integration tests.
 *
 * Description:
 * All projection and threshold numbers are explicit fixture inputs; these
 * checks do not certify calibration, wire Profiles or navigation permission.
 */

#include "modules/normal_grid.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <vector>

namespace {
std::atomic<std::uint64_t> allocations{0};
std::size_t geometry_process_calls = 0, grid_process_calls = 0;
void* allocate(std::size_t n) {
    void* p = std::malloc(n == 0 ? 1 : n);
    if (!p) throw std::bad_alloc();
    allocations.fetch_add(1, std::memory_order_relaxed);
    return p;
}
void* allocate_aligned(std::size_t n, std::size_t alignment) {
    void* p = nullptr;
    if (posix_memalign(&p, alignment, n == 0 ? 1 : n) != 0) throw std::bad_alloc();
    allocations.fetch_add(1, std::memory_order_relaxed);
    return p;
}
[[noreturn]] void fail(const char* expression, int line) {
    std::cerr << "normal grid CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
}
#define CHECK(expression) do { if (!(expression)) fail(#expression, __LINE__); } while (false)
}  // namespace

void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) {
    return allocate_aligned(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a) {
    return allocate_aligned(n, static_cast<std::size_t>(a));
}
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::geometry;
namespace nd = perception::native_depth;
bool close(double a, double b, double epsilon = 1e-9) { return std::abs(a - b) <= epsilon; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
std::size_t at(std::size_t u, std::size_t v) { return v * kNativeWidth + u; }
Vec3 rotated(const ProjectionBinding& b, Vec3 p) {
    const auto& m = b.R_base_camera;
    return {m[0] * p.x + m[1] * p.y + m[2] * p.z,
            m[3] * p.x + m[4] * p.y + m[5] * p.z,
            m[6] * p.x + m[7] * p.y + m[8] * p.z};
}

ProjectionBinding synthetic_binding() {
    ProjectionBinding b;
    b.intrinsics = {128, 128, 320, -128, 640, 400};
    b.distortion.reported = true;
    b.distortion.model = 0;
    b.unit_m = 1.0F / 1024.0F;
    b.R_base_camera = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    b.t_base_camera = {0, 0, 1};
    return b;
}
GeometryOptions synthetic_geometry_options() {
    GeometryOptions o;
    o.prior = {{0, 0, 1}, 0};
    o.fit.inlier_distance_m = 0.015;
    o.fit.min_inlier_fraction = 0.8;
    o.fit.max_inlier_rms_m = 0.005;
    o.fit.min_xy_stddev_m = 0.01;
    o.fit.min_up_dot = 0.5;
    o.fit.max_prior_angle_rad = 0.3;
    o.fit.min_inliers = 100;
    o.fit.hypotheses = 32;
    o.fit.seed = 0x243f6a8885a308d3ULL;
    o.fit_near_min_m = 0.7;
    o.fit_near_max_m = 2.9;
    o.fit_prior_band_m = 0.25;
    o.range_max_m = 6;
    o.blind_near_lo_m = 0.01;
    o.h_tol_m = 0.125;
    o.z_pass_m = 0.5;
    o.eps_neg = 0.1;
    o.w_h_m = 0.1;
    o.angle_min_rad = -1.55;
    o.angle_step_rad = 0.01;
    o.n_bins = 311;
    return o;
}
GridOptions synthetic_grid_options() { return {0.25, 0.9, 0.4, 0.02}; }
std::vector<Vec3> pinhole_rays(const ProjectionBinding& b) {
    std::vector<Vec3> rays(kGeometryPixels);
    for (std::size_t v = 0; v < kNativeHeight; ++v)
        for (std::size_t u = 0; u < kNativeWidth; ++u)
            rays[at(u, v)] = {(static_cast<double>(u) - b.intrinsics.cx) / b.intrinsics.fx,
                             (static_cast<double>(v) - b.intrinsics.cy) / b.intrinsics.fy, 1};
    return rays;
}

struct Fixture {
    ProjectionBinding binding;
    GeometryOptions geometry_options;
    GridOptions grid_options;
    std::vector<Vec3> rays;
    std::vector<std::uint16_t> raster;
    nd::Mailbox mailbox{kGeometryPixels};
    nd::Frame frame{kGeometryPixels};
    std::unique_ptr<NativeGeometry> geometry;
    std::unique_ptr<NormalGrid> grid;
    std::uint64_t stream = 0, sequence = 0;

    explicit Fixture(ProjectionBinding b = synthetic_binding(),
                     GeometryOptions go = synthetic_geometry_options(),
                     GridOptions gr = synthetic_grid_options())
        : binding(b), geometry_options(go), grid_options(gr), rays(pinhole_rays(binding)),
          raster(kGeometryPixels),
          geometry(std::make_unique<NativeGeometry>(binding, rays.data(), rays.size(), geometry_options)),
          grid(std::make_unique<NormalGrid>(*geometry, grid_options)) {
        stream = mailbox.begin_stream();
        CHECK(geometry->storage_bytes() <= 40U * 1024U * 1024U);
        CHECK(grid->storage_bytes() <= 16U * 1024U * 1024U);
        CHECK(grid->cell_capacity_per_bin() <= kMaxRadialCells);
        CHECK(grid->cells().size() <= kMaxGridCells);
    }

    void full_plane(Plane plane = {{0, 0, 1}, 0}) {
        for (std::size_t i = 0; i < rays.size(); ++i) {
            const double z = -(dot(plane.normal, binding.t_base_camera) + plane.d) /
                             dot(plane.normal, rotated(binding, rays[i]));
            const auto code = std::llround(z / binding.unit_m);
            CHECK(code > 0 && code <= 65535);
            raster[i] = static_cast<std::uint16_t>(code);
        }
    }
    void exact_ground_support() {
        std::fill(raster.begin(), raster.end(), 0);
        for (std::size_t u = 0; u < kNativeWidth; ++u) {
            raster[at(u, 0)] = 1024;
            raster[at(u, 128)] = 512;
            raster[at(u, 384)] = 256;
        }
    }
    nd::View next_view() {
        nd::View v;
        v.metadata.intrinsics = binding.intrinsics;
        v.metadata.distortion = binding.distortion;
        v.metadata.unit_m = binding.unit_m;
        auto& s = v.metadata.source;
        s.stream_id = stream;
        s.sequence = ++sequence;
        // Distinct, increasing indices expose accidental cross-field copies.
        s.color_index = sequence * 3;
        s.depth_index = sequence * 5 + 1;
        s.color_device_us = sequence * 1000;
        s.depth_device_us = sequence * 1000 + 1;
        s.host_ready_mono_ns = static_cast<std::int64_t>(sequence * 1000000);
        v.pre_d2c = true;
        v.pixels = raster.data();
        v.bytes = raster.size() * sizeof(std::uint16_t);
        return v;
    }
    void take(const nd::View& v) {
        CHECK(mailbox.try_offer(v) == nd::Offer::Accepted);
        CHECK(mailbox.try_take(frame) == nd::Take::Taken);
    }
    const GeometryResult& process_geometry() {
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto& r = geometry->process(frame, mailbox);
        CHECK(allocations.load(std::memory_order_relaxed) == before);
        ++geometry_process_calls;
        return r;
    }
    const GridResult& process_grid(const nd::Mailbox& owner) {
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto& r = grid->process(owner);
        CHECK(allocations.load(std::memory_order_relaxed) == before);
        ++grid_process_calls;
        return r;
    }
    const GridResult& process_grid() { return process_grid(mailbox); }
    const GridResult& process() {
        take(next_view());
        process_geometry();
        return process_grid();
    }
    int bin_of(Vec3 p) const {
        const double theta = std::atan2(p.y, p.x);
        const double lo = geometry_options.angle_min_rad - geometry_options.angle_step_rad / 2;
        const double hi = lo + geometry_options.n_bins * geometry_options.angle_step_rad;
        if ((p.x == 0 && p.y == 0) || theta < lo || theta >= hi) return -1;
        return static_cast<int>(std::floor((theta - lo) / geometry_options.angle_step_rad));
    }
    // Independent ticket oracle: analytic fixture projection, never reads depth
    // validity or delegates its bin/range computation to NormalGrid.
    bool ticket(std::size_t i, int& bin, double& range) const { return plane_ticket(i, geometry_options.prior, bin, range); }
    // The same oracle for any plane (fitted tickets); the caller adds the prior-ROI condition.
    bool plane_ticket(std::size_t i, Plane prior, int& bin, double& range) const {
        const auto q = rotated(binding, rays[i]);
        const double z = -(dot(prior.normal, binding.t_base_camera) + prior.d) / dot(prior.normal, q);
        if (!std::isfinite(z) || z <= 0) return false;
        const Vec3 p{q.x * z + binding.t_base_camera.x,
                     q.y * z + binding.t_base_camera.y,
                     q.z * z + binding.t_base_camera.z};
        range = std::hypot(p.x, p.y);
        bin = bin_of(p);
        return bin >= 0 && range >= geometry_options.blind_near_lo_m &&
               range <= geometry_options.range_max_m;
    }
    int cell_of(double range) const {
        if (!grid->result().blind_m || range < *grid->result().blind_m) return -1;
        for (std::size_t cell = 0; cell < grid->result().cells_per_bin; ++cell) {
            const double near = *grid->result().blind_m + cell * grid_options.dr_m;
            const double far = *grid->result().blind_m + (cell + 1) * grid_options.dr_m;
            if (range >= near && range < far) return static_cast<int>(cell);
        }
        return -1;
    }
    const CellEvidence& cell(std::size_t bin, std::size_t radial) const {
        return grid->cells()[bin * grid->cell_capacity_per_bin() + radial];
    }
};

void expect_source_matches_geometry(const Fixture& f) {
    const auto& actual = f.grid->result().source;
    const auto& expected = f.geometry->result().source;
    CHECK(actual.stream_id == expected.stream_id);
    CHECK(actual.sequence == expected.sequence);
    CHECK(actual.color_index == expected.color_index);
    CHECK(actual.depth_index == expected.depth_index);
    CHECK(actual.color_device_us == expected.color_device_us);
    CHECK(actual.depth_device_us == expected.depth_device_us);
    CHECK(actual.host_ready_mono_ns == expected.host_ready_mono_ns);
}

void expect_normal(const Fixture& f) {
    expect_source_matches_geometry(f);
    CHECK(f.geometry->result().has_evidence());
    CHECK(f.grid->result().status == GridStatus::NormalEvidence);
    CHECK(f.grid->result().blind_m && f.geometry->result().nearest_ground_m);
    CHECK(close(*f.grid->result().blind_m,
                std::max(f.geometry_options.blind_near_lo_m, *f.geometry->result().nearest_ground_m)));
    CHECK(f.grid->result().cells_per_bin <= f.grid->cell_capacity_per_bin());
    std::size_t complete_cells = 0;
    while (*f.grid->result().blind_m + (complete_cells + 1) * f.grid_options.dr_m <=
           f.geometry_options.range_max_m) ++complete_cells;
    CHECK(f.grid->result().cells_per_bin == complete_cells);
    CHECK(f.grid->result().source.stream_id == f.stream);
    CHECK(f.grid->result().source.sequence == f.sequence);
    for (const auto& b : f.grid->bins()) CHECK(!b.traversal.t_backed_prefix);
    for (const auto& c : f.grid->cells()) CHECK(!c.t_ok);
    // ground_cells: every Ground pixel in pixel order, each with its full_cell_index cell.
    if (f.grid->result().blind_m) {
        const auto& list = f.grid->ground_cells();
        std::size_t n = 0;
        bool list_matches = true;
        for (std::size_t p = 0; p < kGeometryPixels; ++p) {
            const auto& e = f.geometry->pixels()[p];
            if (e.classification != SampleClass::Ground) continue;
            const int k = full_cell_index(e.range_m, *f.grid->result().blind_m, f.grid_options.dr_m,
                                          f.grid->result().cells_per_bin);
            list_matches = list_matches && n < list.size() && list[n].pixel == p && list[n].cell == k;
            ++n;
        }
        CHECK(list_matches && n == list.size());
        CHECK(n == f.geometry->result().classes[static_cast<std::size_t>(SampleClass::Ground)]);
    }
}

void expect_cleared(const Fixture& f) {
    expect_source_matches_geometry(f);
    CHECK(f.grid->ground_cells().empty());
    CHECK(!f.grid->result().blind_m && f.grid->result().cells_per_bin == 0);
    CHECK(f.grid->result().unmatched_ground == 0);
    for (const auto& b : f.grid->bins()) {
        CHECK(b.valid_returns == 0 && b.valid_pierce_rays == 0 && b.expected_rays == 0);
        CHECK(!b.traversal.valid && !b.traversal.free_end_m && !b.traversal.block_m);
        CHECK(!b.traversal.block_height_m && !b.traversal.slope_rad);
        CHECK(b.traversal.passed_cells == 0 && !b.traversal.t_backed_prefix);
        CHECK(b.traversal.boundary == BoundaryKind::None && b.traversal.stop == StopReason::InvalidInput);
    }
    for (const auto& c : f.grid->cells()) {
        CHECK(c.expected == 0 && c.ground == 0 && c.covered_ground == 0);
        CHECK(c.mean_height_m == 0 && c.m2_height_m2 == 0);
        CHECK(!c.pierce && !c.below && !c.below_height_m && !c.t_ok);
    }
}

// Compare every full cell with a separately accumulated per-pixel ledger.
// Expected tickets include invalid pixels; Welford statistics use ALL actual
// Ground, whereas covered_ground requires the SAME source ray's ticket match.
void verify_cell_ledger(const Fixture& f) {
    struct Oracle {
        std::size_t expected = 0, ground = 0, covered = 0;
        long double sum = 0, square_sum = 0;
        bool pierce = false, below = false;
        std::optional<double> below_height;
    };
    std::vector<Oracle> cells(f.grid->cells().size());
    std::vector<std::size_t> expected_rays(f.grid->bins().size()), pierce_rays(f.grid->bins().size());
    std::size_t unmatched = 0;
    const auto offset = [&](int bin, int radial) {
        return static_cast<std::size_t>(bin) * f.grid->cell_capacity_per_bin() + static_cast<std::size_t>(radial);
    };
    // Fitted tickets: the cells use the frame's fitted-plane tickets of the prior-ROI pixels, while
    // the per-bin expected_rays (conf) stay the static prior counts.
    const bool fitted = f.grid_options.fitted_tickets;
    CHECK(!fitted || f.geometry->result().fit.plane);
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        int expected_bin = -1;
        double expected_range = 0;
        const bool has_prior_ticket = f.ticket(i, expected_bin, expected_range);
        int getter_bin = -1;
        double getter_range = 0;
        CHECK(f.geometry->prior_ground_projection(i, getter_bin, getter_range) == has_prior_ticket);
        if (has_prior_ticket) {
            CHECK(getter_bin == expected_bin && close(getter_range, expected_range));
            ++expected_rays[static_cast<std::size_t>(expected_bin)];
        }
        bool has_ticket = has_prior_ticket;
        if (fitted) {
            const Plane plane = *f.geometry->result().fit.plane;
            has_ticket = f.geometry->prior_roi()[i] && f.plane_ticket(i, plane, expected_bin, expected_range);
            const int hint = getter_bin;  // the prior ticket's bin, as NormalGrid passes it
            CHECK(f.geometry->plane_ground_projection(plane, i, hint, getter_bin, getter_range) == has_ticket);
            if (has_ticket) CHECK(getter_bin == expected_bin && close(getter_range, expected_range));
        }
        const int expected_cell = has_ticket ? f.cell_of(expected_range) : -1;
        if (expected_cell >= 0) ++cells[offset(expected_bin, expected_cell)].expected;
        const auto& p = f.geometry->pixels()[i];
        if (p.classification == SampleClass::Ground) {
            const int actual_cell = f.cell_of(p.range_m);
            if (actual_cell >= 0) {
                auto& c = cells[offset(p.bin, actual_cell)];
                ++c.ground;
                c.sum += p.height_m;
                c.square_sum += static_cast<long double>(p.height_m) * p.height_m;
                if (expected_bin == p.bin && expected_cell == actual_cell) ++c.covered;
                else ++unmatched;
            }
        }
        if (p.pierce_bin >= 0 && p.pierce_range_m) {
            ++pierce_rays[static_cast<std::size_t>(p.pierce_bin)];
            const int radial = f.cell_of(*p.pierce_range_m);
            if (radial >= 0) cells[offset(p.pierce_bin, radial)].pierce = true;
        }
        if (p.classification == SampleClass::BelowPlane) {
            const int radial = f.cell_of(p.range_m);
            if (radial >= 0) {
                auto& c = cells[offset(p.bin, radial)];
                c.below = true;
                if (!c.below_height || p.height_m < *c.below_height) c.below_height = p.height_m;
            }
        }
    }
    CHECK(f.grid->result().unmatched_ground == unmatched);
    for (std::size_t b = 0; b < f.grid->bins().size(); ++b) {
        CHECK(f.grid->bins()[b].expected_rays == expected_rays[b]);
        CHECK(f.grid->bins()[b].valid_returns == f.geometry->bins()[b].valid_samples);
        CHECK(f.grid->bins()[b].valid_pierce_rays == pierce_rays[b]);
    }
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto& actual = f.grid->cells()[i];
        const auto& expected = cells[i];
        CHECK(actual.expected == expected.expected && actual.ground == expected.ground);
        CHECK(actual.covered_ground == expected.covered);
        CHECK(actual.covered_ground <= actual.expected && actual.covered_ground <= actual.ground);
        CHECK(actual.pierce == expected.pierce && actual.below == expected.below);
        CHECK(actual.below_height_m.has_value() == expected.below_height.has_value());
        if (expected.below_height) CHECK(close(*actual.below_height_m, *expected.below_height));
        if (expected.ground != 0) {
            const long double mean = expected.sum / expected.ground;
            const long double m2 = expected.square_sum - expected.sum * mean;
            CHECK(close(actual.mean_height_m, static_cast<double>(mean), 1e-10));
            CHECK(close(actual.m2_height_m2, static_cast<double>(m2), 1e-10));
        } else CHECK(actual.mean_height_m == 0 && actual.m2_height_m2 == 0);
    }
}

std::uint64_t evidence_hash(const Fixture& f) {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto word = [&](std::uint64_t value) { hash = (hash ^ value) * 1099511628211ULL; };
    const auto number = [&](double value) {
        std::uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        word(bits);
    };
    const auto optional = [&](const std::optional<double>& value) {
        word(value.has_value());
        if (value) number(*value);
    };
    optional(f.grid->result().blind_m);
    word(f.grid->result().cells_per_bin);
    word(f.grid->result().unmatched_ground);
    for (const auto& c : f.grid->cells()) {
        word(c.expected); word(c.ground); word(c.covered_ground);
        number(c.mean_height_m); number(c.m2_height_m2);
        word(c.pierce); word(c.below); word(c.t_ok); optional(c.below_height_m);
    }
    for (const auto& b : f.grid->bins()) {
        word(b.expected_rays); word(b.valid_returns); word(b.valid_pierce_rays);
        word(b.traversal.valid); word(static_cast<unsigned>(b.traversal.stop));
        word(static_cast<unsigned>(b.traversal.boundary)); word(b.traversal.passed_cells);
        word(b.traversal.t_backed_prefix); optional(b.traversal.free_end_m);
        optional(b.traversal.block_m); optional(b.traversal.block_height_m); optional(b.traversal.slope_rad);
    }
    return hash;
}

// A cell's coverage denominator is its expected ray count, fixed by geometry; invalid returns
// lower only the covered count, so FREE stops at the under-covered cell (Coverage).
void test_flat_plane_and_independent_denominator() {
    Fixture f;
    f.full_plane();
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(f.geometry->result().classes[static_cast<std::size_t>(SampleClass::Ground)] == kGeometryPixels);
    const auto bin = static_cast<std::size_t>(f.bin_of({1, 0, 0}));
    const auto baseline_blind = *f.grid->result().blind_m;
    const auto baseline_expected_rays = f.grid->bins()[bin].expected_rays;
    const std::size_t target_cell = 1;
    CHECK(f.grid->bins()[bin].traversal.passed_cells > target_cell + 1);
    CHECK(f.grid->bins()[bin].traversal.free_end_m);
    const auto expected = f.cell(bin, target_cell).expected;
    std::vector<std::size_t> target_rays;
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        int b = -1;
        double range = 0;
        if (f.ticket(i, b, range) && b == static_cast<int>(bin) &&
            f.cell_of(range) == static_cast<int>(target_cell)) target_rays.push_back(i);
    }
    CHECK(target_rays.size() == expected && expected > 20);
    const std::size_t invalid = target_rays.size() / 3;
    for (std::size_t i = 0; i < invalid; ++i) f.raster[target_rays[i]] = 0;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(close(*f.grid->result().blind_m, baseline_blind, 0));
    CHECK(f.grid->bins()[bin].expected_rays == baseline_expected_rays);
    CHECK(f.cell(bin, target_cell).expected == expected);
    CHECK(f.cell(bin, target_cell).covered_ground <= expected - invalid);
    CHECK(f.grid->bins()[bin].traversal.stop == StopReason::Coverage);
    CHECK(f.grid->bins()[bin].traversal.passed_cells == target_cell);
    CHECK(close(*f.grid->bins()[bin].traversal.free_end_m,
                baseline_blind + target_cell * f.grid_options.dr_m));
    for (const auto i : target_rays) f.raster[i] = 0;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(f.cell(bin, target_cell).expected == expected);
    CHECK(f.cell(bin, target_cell).covered_ground == 0);
    CHECK(f.cell(bin, target_cell + 1).covered_ground > 0);
    CHECK(f.grid->bins()[bin].traversal.passed_cells == target_cell);
    CHECK(f.grid->bins()[bin].traversal.stop == StopReason::Coverage);
}

// When the actual ground moves, returns cannot count against another cell's expected rays: FREE
// stops at the blind edge for lack of coverage and no false crossings appear.
void test_moved_ground_cannot_borrow_tickets() {
    Fixture f;
    f.full_plane();
    f.process();
    expect_normal(f);
    const auto bin = static_cast<std::size_t>(f.bin_of({1, 0, 0}));
    const double initial_blind = *f.grid->result().blind_m;
    const auto initial_expected = f.cell(bin, 0).expected;
    const auto expected_rays = f.grid->bins()[bin].expected_rays;
    f.full_plane({{0, 0, 1}, 0.15});
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(*f.grid->result().blind_m > initial_blind);
    CHECK(f.cell(bin, 0).expected != initial_expected);
    CHECK(f.grid->bins()[bin].expected_rays == expected_rays);
    CHECK(f.grid->result().unmatched_ground > 100);
    const auto& c = f.cell(bin, 0);
    CHECK(c.expected > 0);
    CHECK(static_cast<double>(c.ground) / c.expected >= f.grid_options.cover_min);
    CHECK(static_cast<double>(c.covered_ground) / c.expected < f.grid_options.cover_min);
    CHECK(f.grid->bins()[bin].traversal.stop == StopReason::Coverage);
    CHECK(f.grid->bins()[bin].traversal.passed_cells == 0);
    CHECK(f.grid->bins()[bin].traversal.free_end_m);
    CHECK(close(*f.grid->bins()[bin].traversal.free_end_m, *f.grid->result().blind_m));
    // Moving the actual plane must not create prior-based crossing evidence.
    for (const auto& b : f.grid->bins()) CHECK(b.valid_pierce_rays == 0);
}

// A single obstacle pixel becomes its bin's positive boundary at the pixel's range and height,
// with FREE ending before it.
void test_single_pixel_positive_boundary() {
    Fixture f;
    f.full_plane();
    const auto thin = kGeometryPixels - 1;
    f.raster[thin] = 187;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    const auto& p = f.geometry->pixels()[thin];
    CHECK(p.classification == SampleClass::Obstacle);
    const auto& walk = f.grid->bins()[static_cast<std::size_t>(p.bin)].traversal;
    CHECK(walk.boundary == BoundaryKind::Positive && walk.block_m && walk.block_height_m);
    CHECK(close(*walk.block_m, p.range_m) && close(*walk.block_height_m, p.height_m));
    CHECK(!walk.free_end_m || *walk.free_end_m < *walk.block_m);
}

// A below-plane return marks its own cell below (with height) and its expected crossing marks the
// crossing's cell pierce; both bins get a negative boundary at the cell's near edge.
void test_current_pierce_and_below_cell_association() {
    auto binding = synthetic_binding();
    binding.t_base_camera = {0.5, 0.75, 1};
    Fixture f(binding);
    f.full_plane();
    const auto sample = at(320, 128);
    f.raster[sample] = 768;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    const auto& p = f.geometry->pixels()[sample];
    CHECK(p.classification == SampleClass::BelowPlane && p.pierce_range_m);
    CHECK(p.bin != p.pierce_bin);
    const int below_cell = f.cell_of(p.range_m);
    const int pierce_cell = f.cell_of(*p.pierce_range_m);
    CHECK(below_cell >= 0 && pierce_cell >= 0);
    const auto& below = f.cell(static_cast<std::size_t>(p.bin), static_cast<std::size_t>(below_cell));
    const auto& pierce = f.cell(static_cast<std::size_t>(p.pierce_bin), static_cast<std::size_t>(pierce_cell));
    CHECK(below.below && below.below_height_m && !below.pierce);
    CHECK(close(*below.below_height_m, p.height_m));
    CHECK(pierce.pierce && !pierce.below && !pierce.below_height_m);
    const auto& below_walk = f.grid->bins()[static_cast<std::size_t>(p.bin)].traversal;
    const auto& pierce_walk = f.grid->bins()[static_cast<std::size_t>(p.pierce_bin)].traversal;
    CHECK(below_walk.boundary == BoundaryKind::Negative && below_walk.block_m);
    CHECK(pierce_walk.boundary == BoundaryKind::Negative && pierce_walk.block_m);
    CHECK(close(*below_walk.block_m, *f.grid->result().blind_m + below_cell * f.grid_options.dr_m));
    CHECK(close(*pierce_walk.block_m, *f.grid->result().blind_m + pierce_cell * f.grid_options.dr_m));
    CHECK(!pierce_walk.block_height_m);
    CHECK(below_walk.block_height_m && close(*below_walk.block_height_m, p.height_m));
}

// The height of every negative boundary against the pixel evidence alone: the lowest height among
// the bin's below-plane returns, and none when the bin has none (19 section 3.2: neg_h[i] is a
// per-bin minimum; section 3.4: measured in-pit points ? neg_h[i] : null). NativeGeometry keeps
// the same minimum per bin on its own; both must agree. Returns the number of such bins.
std::size_t expect_negative_heights(const Fixture& f) {
    std::vector<std::optional<double>> lowest(f.grid->bins().size());
    for (const auto& p : f.geometry->pixels()) {
        if (p.classification != SampleClass::BelowPlane) continue;
        auto& l = lowest[static_cast<std::size_t>(p.bin)];
        if (!l || p.height_m < *l) l = p.height_m;
    }
    std::size_t negative_bins = 0;
    for (std::size_t b = 0; b < f.grid->bins().size(); ++b) {
        CHECK(f.geometry->bins()[b].below_min_height_m == lowest[b]);
        const auto& walk = f.grid->bins()[b].traversal;
        if (walk.boundary != BoundaryKind::Negative) continue;
        ++negative_bins;
        CHECK(walk.block_height_m == lowest[b]);
        if (walk.block_height_m) CHECK(*walk.block_height_m < 0);
    }
    return negative_bins;
}

// The height of a negative boundary is the lowest below-plane height measured in the bin,
// wherever the boundary itself comes from: a pierce in an earlier cell, a pierce before the grid,
// a cell that has a shallower return of its own, or returns in the trailing partial cell. Pierce
// alone gives no height.
void test_negative_height_is_the_lowest_of_the_bin() {
    const std::size_t column = 320;   // ray.x = 0: range equals depth, one bin for every row
    {
        // Two pit returns in one bin. Row 100 crosses the ground at 0.561 m and returns from
        // 0.640 m, 0.14 m below the plane: same cell as its crossing. Row 128 crosses at 0.5 m and
        // returns from 0.760 m, 0.52 m below: one cell farther. The boundary is the near edge of
        // the cell with the crossings; its own return is the shallow one.
        Fixture f;
        f.full_plane();
        const auto shallow_pixel = at(column, 100), deep_pixel = at(column, 128);
        f.raster[shallow_pixel] = static_cast<std::uint16_t>(std::llround(0.640 / f.binding.unit_m));
        f.raster[deep_pixel] = static_cast<std::uint16_t>(std::llround(0.760 / f.binding.unit_m));
        f.process();
        expect_normal(f);
        verify_cell_ledger(f);
        const auto& shallow = f.geometry->pixels()[shallow_pixel];
        const auto& deep = f.geometry->pixels()[deep_pixel];
        CHECK(shallow.classification == SampleClass::BelowPlane && shallow.pierce_range_m);
        CHECK(deep.classification == SampleClass::BelowPlane && deep.pierce_range_m);
        CHECK(shallow.bin == deep.bin && shallow.pierce_bin == shallow.bin && deep.pierce_bin == deep.bin);
        CHECK(deep.height_m < shallow.height_m && shallow.height_m < 0);
        const int edge_cell = f.cell_of(*deep.pierce_range_m);
        CHECK(edge_cell >= 0 && f.cell_of(*shallow.pierce_range_m) == edge_cell);
        CHECK(f.cell_of(shallow.range_m) == edge_cell && f.cell_of(deep.range_m) == edge_cell + 1);
        const auto bin = static_cast<std::size_t>(deep.bin);
        const auto& edge = f.cell(bin, static_cast<std::size_t>(edge_cell));
        CHECK(edge.pierce && edge.below && edge.below_height_m && close(*edge.below_height_m, shallow.height_m));
        const auto& walk = f.grid->bins()[bin].traversal;
        CHECK(walk.boundary == BoundaryKind::Negative && walk.block_m);
        CHECK(close(*walk.block_m, *f.grid->result().blind_m + edge_cell * f.grid_options.dr_m));
        CHECK(walk.block_height_m && *walk.block_height_m == deep.height_m);
        CHECK(expect_negative_heights(f) == 1);

        // The same edge by pierce alone: the deep return gone beyond the range, the shallow one
        // back on the ground. The boundary stays where it was and has no height.
        f.raster[deep_pixel] = 65535;
        f.raster[shallow_pixel] = static_cast<std::uint16_t>(std::llround(128.0 / 228.0 / f.binding.unit_m));
        f.process();
        expect_normal(f);
        verify_cell_ledger(f);
        const auto& gone = f.geometry->pixels()[deep_pixel];
        CHECK(gone.classification == SampleClass::OutsideDomain && gone.pierce_range_m);
        CHECK(f.geometry->pixels()[shallow_pixel].classification == SampleClass::Ground);
        const auto& pierce_walk = f.grid->bins()[bin].traversal;
        CHECK(pierce_walk.boundary == BoundaryKind::Negative && pierce_walk.block_m);
        CHECK(close(*pierce_walk.block_m, *f.grid->result().blind_m + edge_cell * f.grid_options.dr_m));
        CHECK(!pierce_walk.block_height_m);
        CHECK(expect_negative_heights(f) == 1);
    }
    {
        // The near field is a pit. Rows 300..386 have no return, so the nearest ground return is
        // row 299 (0.2998 m) and the cells start there. Rows 387..398 return from 1.2 times their
        // ground depth (0.2 m below the plane), row 399 from 1.15 times (0.15 m below): in the
        // centre bin all of these returns, and their crossings, lie before the first cell. The
        // nearest evidence is the crossing of row 399 (no height), the nearest return is the
        // shallow one of row 399; the height is the 0.2 m of the rows behind it.
        Fixture f;
        f.full_plane();
        for (std::size_t v = 300; v < kNativeHeight; ++v)
            for (std::size_t u = 0; u < kNativeWidth; ++u) {
                const double ground = 128.0 / static_cast<double>(v + 128);
                const double factor = v < 387 ? 0.0 : v == kNativeHeight - 1 ? 1.15 : 1.2;
                f.raster[at(u, v)] = static_cast<std::uint16_t>(std::llround(ground * factor / f.binding.unit_m));
            }
        f.process();
        expect_normal(f);
        verify_cell_ledger(f);
        const double blind = *f.grid->result().blind_m;
        CHECK(close(blind, f.geometry->pixels()[at(column, 299)].range_m));
        const auto& nearest = f.geometry->pixels()[at(column, kNativeHeight - 1)];
        const auto& behind = f.geometry->pixels()[at(column, 390)];
        CHECK(nearest.classification == SampleClass::BelowPlane && nearest.pierce_range_m);
        CHECK(behind.classification == SampleClass::BelowPlane && behind.bin == nearest.bin);
        CHECK(*nearest.pierce_range_m < nearest.range_m && nearest.range_m < behind.range_m && behind.range_m < blind);
        CHECK(behind.height_m < nearest.height_m && nearest.height_m < 0);
        const auto bin = static_cast<std::size_t>(nearest.bin);
        for (std::size_t k = 0; k < f.grid->result().cells_per_bin; ++k) CHECK(!f.cell(bin, k).below && !f.cell(bin, k).pierce);
        const auto& walk = f.grid->bins()[bin].traversal;
        CHECK(walk.boundary == BoundaryKind::Negative && walk.block_m && close(*walk.block_m, *nearest.pierce_range_m));
        CHECK(!walk.free_end_m && walk.passed_cells == 0);
        CHECK(walk.block_height_m && *walk.block_height_m <= behind.height_m && *walk.block_height_m < nearest.height_m);
        CHECK(expect_negative_heights(f) > 1);
    }
    {
        // Two pit returns of different depth in the trailing partial cell of one bin (the range
        // ends at 1.1 m, inside a cell). Their crossings lie in the last full cell, which holds
        // the boundary and has no return of its own; the height is the deeper of the two, which
        // is not the first one in pixel order.
        auto go = synthetic_geometry_options();
        go.range_max_m = 1.1;
        go.fit_near_max_m = 1.0;
        Fixture f(synthetic_binding(), go);
        f.full_plane();
        const auto shallow_pixel = at(column, 10), deep_pixel = at(column, 20);
        f.raster[shallow_pixel] = static_cast<std::uint16_t>(std::llround(1.08 / f.binding.unit_m));
        f.raster[deep_pixel] = static_cast<std::uint16_t>(std::llround(1.05 / f.binding.unit_m));
        f.process();
        expect_normal(f);
        verify_cell_ledger(f);
        const auto& shallow = f.geometry->pixels()[shallow_pixel];
        const auto& deep = f.geometry->pixels()[deep_pixel];
        CHECK(shallow.classification == SampleClass::BelowPlane && deep.classification == SampleClass::BelowPlane);
        CHECK(shallow_pixel < deep_pixel && shallow.bin == deep.bin && deep.height_m < shallow.height_m);
        const double blind = *f.grid->result().blind_m;
        CHECK(shallow.range_m >= blind && deep.range_m >= blind);
        CHECK(f.cell_of(shallow.range_m) == -1 && f.cell_of(deep.range_m) == -1);   // past the last full cell
        const auto bin = static_cast<std::size_t>(deep.bin);
        for (std::size_t k = 0; k < f.grid->result().cells_per_bin; ++k) CHECK(!f.cell(bin, k).below);
        const auto& walk = f.grid->bins()[bin].traversal;
        CHECK(walk.boundary == BoundaryKind::Negative && walk.block_m && *walk.block_m < deep.range_m);
        CHECK(walk.block_height_m && *walk.block_height_m == deep.height_m);
        CHECK(expect_negative_heights(f) == 1);
    }
}

// A partial cell at the end of the range never adds FREE; a crossing inside it puts the negative
// boundary at its near edge.
void test_trailing_partial_cell_never_adds_free() {
    auto go = synthetic_geometry_options();
    go.range_max_m = 1.1;
    go.fit_near_max_m = 1.0;
    Fixture f(synthetic_binding(), go);
    f.full_plane();
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(f.grid->result().cells_per_bin == 3);
    const auto bin = static_cast<std::size_t>(f.bin_of({1, 0, 0}));
    const double tail_start = *f.grid->result().blind_m + 3 * f.grid_options.dr_m;
    CHECK(tail_start < go.range_max_m && f.cell_of(1.0) == -1);
    const auto& flat = f.grid->bins()[bin].traversal;
    CHECK(flat.passed_cells == 3 && flat.free_end_m && close(*flat.free_end_m, tail_start));
    CHECK(flat.stop == StopReason::Range);
    // Expected intersection at 1 m lies in the trailing partial cell. The
    // actual return is outside range, but its raw crossing still supplies a
    // boundary at the tail near edge and never supplies a fourth FREE cell.
    f.raster[at(320, 0)] = 1200;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    const auto& p = f.geometry->pixels()[at(320, 0)];
    CHECK(p.classification == SampleClass::OutsideDomain && p.pierce_range_m);
    CHECK(f.cell_of(*p.pierce_range_m) == -1 && *p.pierce_range_m > tail_start);
    const auto& negative = f.grid->bins()[bin].traversal;
    CHECK(negative.boundary == BoundaryKind::Negative && negative.block_m);
    CHECK(close(*negative.block_m, tail_start));
    CHECK(!negative.free_end_m || *negative.free_end_m < *negative.block_m);
}

// With no ground in the bin the grid keeps only the obstacle boundary: no blind edge, cells or
// FREE.
void test_no_ground_reference_retains_only_boundary() {
    auto go = synthetic_geometry_options();
    go.angle_min_rad = 0;
    go.n_bins = 1;
    Fixture f(synthetic_binding(), go);
    f.exact_ground_support();
    for (std::size_t v = 0; v < kNativeHeight; ++v) f.raster[at(320, v)] = 0;
    f.raster[at(320, 128)] = 384;
    f.process();
    CHECK(f.geometry->result().has_evidence() && f.geometry->result().fit.inliers >= 100);
    CHECK(!f.geometry->result().nearest_ground_m);
    CHECK(f.grid->result().status == GridStatus::NoGroundReference);
    expect_source_matches_geometry(f);
    CHECK(!f.grid->result().blind_m && f.grid->result().cells_per_bin == 0);
    const auto& b = f.grid->bins()[0];
    CHECK(b.valid_returns == 1 && b.traversal.boundary == BoundaryKind::Positive);
    CHECK(b.traversal.block_m && close(*b.traversal.block_m, 0.375, 0));
    CHECK(!b.traversal.free_end_m && b.traversal.passed_cells == 0 && !b.traversal.t_backed_prefix);
    for (const auto& c : f.grid->cells()) CHECK(c.covered_ground == 0 && !c.t_ok);
}

// Repeated frames give identical evidence; a replay, failed fit, profile mismatch, or ended or
// restarted stream clears everything; good frames restore the same evidence.
void test_failure_replay_restart_and_metadata_clear() {
    Fixture f;
    f.full_plane();
    f.process();
    expect_normal(f);
    const auto baseline_hash = evidence_hash(f);
    f.process();
    expect_normal(f);
    CHECK(evidence_hash(f) == baseline_hash);
    CHECK(f.process_grid().status == GridStatus::InactiveOrStale);
    expect_cleared(f);
    CHECK(evidence_hash(f) != baseline_hash);
    f.process();
    expect_normal(f);
    CHECK(evidence_hash(f) == baseline_hash);
    // Populate both boundary kinds and their heights before failure. The
    // below return and its expected crossing lie in the same full radial cell,
    // so the successful grid must retain pierce, below and negative height.
    const auto positive_pixel = kGeometryPixels - 1;
    const auto negative_pixel = at(320, 128);
    f.raster[positive_pixel] = 187;
    f.raster[negative_pixel] = 584;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    const auto& positive = f.geometry->pixels()[positive_pixel];
    const auto& negative = f.geometry->pixels()[negative_pixel];
    CHECK(positive.classification == SampleClass::Obstacle);
    CHECK(negative.classification == SampleClass::BelowPlane && negative.pierce_range_m);
    CHECK(positive.bin != negative.bin && negative.bin == negative.pierce_bin);
    const auto& positive_walk = f.grid->bins()[static_cast<std::size_t>(positive.bin)].traversal;
    const auto& negative_bin = f.grid->bins()[static_cast<std::size_t>(negative.bin)];
    CHECK(positive_walk.boundary == BoundaryKind::Positive && positive_walk.block_m &&
          positive_walk.block_height_m && *positive_walk.block_height_m > 0);
    CHECK(close(*positive_walk.block_m, positive.range_m));
    CHECK(close(*positive_walk.block_height_m, positive.height_m));
    CHECK(negative_bin.valid_pierce_rays > 0);
    CHECK(negative_bin.traversal.boundary == BoundaryKind::Negative &&
          negative_bin.traversal.block_m && negative_bin.traversal.block_height_m &&
          *negative_bin.traversal.block_height_m < 0);
    const int negative_cell = f.cell_of(negative.range_m);
    CHECK(negative_cell >= 0 && f.cell_of(*negative.pierce_range_m) == negative_cell);
    const auto& seeded_cell = f.cell(static_cast<std::size_t>(negative.bin),
                                     static_cast<std::size_t>(negative_cell));
    CHECK(seeded_cell.pierce && seeded_cell.below && seeded_cell.below_height_m);
    CHECK(close(*seeded_cell.below_height_m, negative.height_m));
    CHECK(close(*negative_bin.traversal.block_height_m, negative.height_m));
    const auto seeded_hash = evidence_hash(f);
    CHECK(seeded_hash != baseline_hash);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    CHECK(f.process().status == GridStatus::GeometryUnavailable);
    CHECK(f.geometry->result().status == GeometryStatus::FitFailed);
    expect_cleared(f);
    CHECK(evidence_hash(f) != seeded_hash);
    f.full_plane();
    f.process();
    expect_normal(f);
    CHECK(evidence_hash(f) == baseline_hash);
    auto bad = f.next_view();
    bad.metadata.distortion.coefficients[0] = -0.0F;
    f.take(bad);
    CHECK(f.process_geometry().status == GeometryStatus::ProfileMismatch);
    CHECK(f.process_grid().status == GridStatus::GeometryUnavailable);
    CHECK(f.grid->result().source.sequence == f.sequence);
    expect_cleared(f);
    CHECK(evidence_hash(f) != baseline_hash);
    f.process();
    expect_normal(f);
    CHECK(evidence_hash(f) == baseline_hash);
    f.mailbox.end_stream();
    CHECK(f.process_grid().status == GridStatus::InactiveOrStale);
    expect_cleared(f);
    const auto old_stream = f.stream;
    f.stream = f.mailbox.begin_stream();
    f.sequence = 0;
    CHECK(f.stream != old_stream);
    CHECK(f.process_grid().status == GridStatus::InactiveOrStale);
    expect_cleared(f);
    f.process();
    expect_normal(f);
    CHECK(f.grid->result().source.stream_id == f.stream && f.grid->result().source.sequence == 1);
    CHECK(evidence_hash(f) == baseline_hash);
}

// A lookalike mailbox owner is refused and clears all evidence (FREE and both boundary kinds)
// without consuming the real frame.
void test_mailbox_owner_collision_clears_and_recovers() {
    Fixture f;
    f.full_plane();
    f.raster[kGeometryPixels - 1] = 187;
    f.raster[at(320, 128)] = 584;
    nd::Mailbox other(kGeometryPixels);
    nd::Frame other_frame(kGeometryPixels);
    CHECK(other.begin_stream() == f.stream);
    const auto deliver = [&] {
        const auto view = f.next_view();
        f.take(view);
        CHECK(other.try_offer(view) == nd::Offer::Accepted);
        CHECK(other.try_take(other_frame) == nd::Take::Taken);
        CHECK(other_frame.metadata().source.stream_id == f.frame.metadata().source.stream_id);
        CHECK(other_frame.metadata().source.sequence == f.frame.metadata().source.sequence);
        CHECK(f.process_geometry().has_evidence());
        CHECK(f.geometry->bound_to(f.mailbox) && !f.geometry->bound_to(other));
        CHECK(other.active_stream() == f.mailbox.active_stream());
    };
    // First grid call gets the wrong owner even though every numeric source
    // identifier is available in both Mailboxes. Rejection must not consume A.
    deliver();
    CHECK(f.process_grid(other).status == GridStatus::InactiveOrStale);
    expect_cleared(f);
    CHECK(f.process_grid().status == GridStatus::NormalEvidence);
    expect_normal(f);
    verify_cell_ledger(f);
    const auto expected_hash = evidence_hash(f);
    bool positive = false, negative = false, free = false;
    for (const auto& bin : f.grid->bins()) {
        positive |= bin.traversal.boundary == BoundaryKind::Positive;
        negative |= bin.traversal.boundary == BoundaryKind::Negative;
        free |= bin.traversal.free_end_m.has_value();
    }
    CHECK(positive && negative && free);
    // Seeded cell, FREE and both boundary kinds must all disappear on a later
    // owner error. A's same pending sequence remains processable afterwards.
    deliver();
    CHECK(f.process_grid(other).status == GridStatus::InactiveOrStale);
    expect_cleared(f);
    CHECK(evidence_hash(f) != expected_hash);
    CHECK(f.process_grid().status == GridStatus::NormalEvidence);
    expect_normal(f);
    CHECK(evidence_hash(f) == expected_hash);
    deliver();
    CHECK(f.process_grid().status == GridStatus::NormalEvidence);
    expect_normal(f);
    CHECK(evidence_hash(f) == expected_hash);
}

// Invalid grid options, and cell counts beyond the per-bin or total storage limit, are refused.
void test_invalid_options_and_storage_bounds() {
    Fixture f;
    const auto reject = [&](GridOptions o) {
        bool rejected = false;
        try { NormalGrid invalid(*f.geometry, o); }
        catch (const std::invalid_argument&) { rejected = true; }
        CHECK(rejected);
    };
    auto o = synthetic_grid_options();
    CHECK(valid_grid_options(o));
    o.dr_m = 0; reject(o);
    o = synthetic_grid_options(); o.dr_m = -1; reject(o);
    o = synthetic_grid_options(); o.dr_m = std::numeric_limits<double>::infinity(); reject(o);
    o = synthetic_grid_options(); o.cover_min = 1.1; reject(o);
    o = synthetic_grid_options(); o.cover_min = -1; reject(o);
    o = synthetic_grid_options(); o.slope_max_rad = -1; reject(o);
    o = synthetic_grid_options(); o.sigma_max_m = -1; reject(o);
    o = synthetic_grid_options(); o.sigma_max_m = std::numeric_limits<double>::quiet_NaN(); reject(o);
    o = synthetic_grid_options(); o.dr_m = 0.001; reject(o);  // Per-bin capacity overflow.
    auto many_bins = synthetic_geometry_options();
    many_bins.n_bins = 1024;
    many_bins.angle_step_rad = 0.003;
    NativeGeometry many(f.binding, f.rays.data(), f.rays.size(), many_bins);
    bool grid_rejected = false;
    try {
        auto dense = synthetic_grid_options();
        dense.dr_m = 0.05;  // <= 256 per bin but total cells exceed 65536.
        NormalGrid invalid(many, dense);
    } catch (const std::invalid_argument&) { grid_rejected = true; }
    CHECK(grid_rejected);
}

GridOptions fitted_grid_options() {
    auto o = synthetic_grid_options();
    o.fitted_tickets = true;
    return o;
}

// Fitted tickets (2026-10-27) with the ground on the prior plane: the fitted plane is the prior up to
// the fit's rounding, so every bin walks as with the static tickets, and conf's expected_rays stay
// the static counts.
void test_fitted_tickets_on_the_prior_plane() {
    Fixture prior, f(synthetic_binding(), synthetic_geometry_options(), fitted_grid_options());
    prior.full_plane();
    f.full_plane();
    prior.process();
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(f.grid->storage_bytes() > prior.grid->storage_bytes());
    for (std::size_t b = 0; b < prior.grid->bins().size(); ++b) {
        const auto& a = prior.grid->bins()[b].traversal;
        const auto& c = f.grid->bins()[b].traversal;
        CHECK(a.stop == c.stop && a.passed_cells == c.passed_cells && a.free_end_m == c.free_end_m);
        CHECK(prior.grid->bins()[b].expected_rays == f.grid->bins()[b].expected_rays);
    }
}

// The moved ground of test_moved_ground_cannot_borrow_tickets with fitted tickets: each ray's ticket
// follows the fitted plane, its own return redeems it, and FREE is no longer held at the blind edge.
// The moved plane still creates no prior-based crossing evidence.
void test_fitted_tickets_follow_moved_ground() {
    Fixture prior, f(synthetic_binding(), synthetic_geometry_options(), fitted_grid_options());
    prior.full_plane({{0, 0, 1}, 0.15});
    f.full_plane({{0, 0, 1}, 0.15});
    prior.process();
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    const auto bin = static_cast<std::size_t>(f.bin_of({1, 0, 0}));
    CHECK(prior.grid->bins()[bin].traversal.stop == StopReason::Coverage);
    CHECK(prior.grid->bins()[bin].traversal.passed_cells == 0);
    CHECK(f.grid->result().unmatched_ground * 10 < prior.grid->result().unmatched_ground);
    const auto& c = f.cell(bin, 0);
    CHECK(c.expected > 0 && static_cast<double>(c.covered_ground) / c.expected >= f.grid_options.cover_min);
    CHECK(f.grid->bins()[bin].traversal.passed_cells > 1);
    for (const auto& b : f.grid->bins()) CHECK(b.valid_pierce_rays == 0);
}

// A ground tilted by 5 degrees of pitch through the prior's origin: fitted tickets carry FREE further
// than the static ones, and a cell whose own rays return invalid depth still stops it (Coverage):
// returns of neighbouring rays cannot fill that hole.
void test_fitted_tickets_tilted_ground_and_no_borrowing() {
    const double a = 5.0 * 3.14159265358979323846 / 180.0;
    const Plane tilted{{std::sin(a), 0, std::cos(a)}, 0};
    Fixture prior, f(synthetic_binding(), synthetic_geometry_options(), fitted_grid_options());
    prior.full_plane(tilted);
    f.full_plane(tilted);
    prior.process();
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    const auto bin = static_cast<std::size_t>(f.bin_of({1, 0, 0}));
    const std::size_t target_cell = 1;
    CHECK(f.grid->bins()[bin].traversal.passed_cells > prior.grid->bins()[bin].traversal.passed_cells);
    CHECK(f.grid->bins()[bin].traversal.passed_cells > target_cell + 1);
    const Plane plane = *f.geometry->result().fit.plane;
    std::vector<std::size_t> target_rays;
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        int b = -1;
        double range = 0;
        if (f.geometry->prior_roi()[i] && f.plane_ticket(i, plane, b, range) && b == static_cast<int>(bin) &&
            f.cell_of(range) == static_cast<int>(target_cell)) target_rays.push_back(i);
    }
    CHECK(target_rays.size() == f.cell(bin, target_cell).expected && target_rays.size() > 20);
    for (std::size_t i = 0; i < target_rays.size() / 3; ++i) f.raster[target_rays[i]] = 0;
    f.process();
    expect_normal(f);
    verify_cell_ledger(f);
    CHECK(f.grid->bins()[bin].traversal.stop == StopReason::Coverage);
    CHECK(f.grid->bins()[bin].traversal.passed_cells == target_cell);
}
}  // namespace

int main() {
    const struct Test { const char* name; void (*run)(); } tests[] = {
        {"flat_plane_independent_invalid_denominator", test_flat_plane_and_independent_denominator},
        {"moved_ground_cannot_borrow_expected_tickets", test_moved_ground_cannot_borrow_tickets},
        {"single_native_pixel_positive_boundary", test_single_pixel_positive_boundary},
        {"current_pierce_below_cell_association", test_current_pierce_and_below_cell_association},
        {"negative_height_is_the_lowest_of_the_bin", test_negative_height_is_the_lowest_of_the_bin},
        {"trailing_partial_cell_never_adds_free", test_trailing_partial_cell_never_adds_free},
        {"no_ground_reference_retains_boundary", test_no_ground_reference_retains_only_boundary},
        {"failure_replay_restart_metadata_clear_hash", test_failure_replay_restart_and_metadata_clear},
        {"mailbox_owner_collision_clear_recovery", test_mailbox_owner_collision_clears_and_recovers},
        {"invalid_options_storage_bounds", test_invalid_options_and_storage_bounds},
        {"fitted_tickets_on_the_prior_plane", test_fitted_tickets_on_the_prior_plane},
        {"fitted_tickets_follow_moved_ground", test_fitted_tickets_follow_moved_ground},
        {"fitted_tickets_tilted_ground_no_borrowing", test_fitted_tickets_tilted_ground_and_no_borrowing},
    };
    for (const auto& test : tests) {
        test.run();
        std::cout << "PASS " << test.name << '\n';
    }
    const Fixture storage;
    std::cout << "{\"suite\":\"synthetic_normal_grid\",\"tests_passed\":" << std::size(tests)
              << ",\"zero_cpp_allocation_geometry_calls\":" << geometry_process_calls
              << ",\"zero_cpp_allocation_grid_calls\":" << grid_process_calls
              << ",\"geometry_buffer_capacity_bytes\":" << storage.geometry->storage_bytes()
              << ",\"grid_buffer_capacity_bytes\":" << storage.grid->storage_bytes()
              << ",\"production_calibration_claim\":false,\"wire_profile_claim\":false,\"t_path\":false}\n";
}
