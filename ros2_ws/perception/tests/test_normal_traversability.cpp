/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_normal_traversability.cpp
 * Brief: Native Frame -> geometry -> grid -> raw T synthetic integration tests.
 *
 * Description:
 * All projection and threshold numbers are explicit fixture inputs; these
 * checks do not certify calibration, wire Profiles or navigation permission.
 */

#include "modules/traversability.hpp"

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
std::size_t geometry_process_calls = 0, grid_process_calls = 0, t_process_calls = 0;
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

struct DepthFixture {
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

    explicit DepthFixture(ProjectionBinding b = synthetic_binding(),
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
    const GridResult& process_grid() {
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto& r = grid->process(mailbox);
        CHECK(allocations.load(std::memory_order_relaxed) == before);
        ++grid_process_calls;
        return r;
    }
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
    bool ticket(std::size_t i, int& bin, double& range) const {
        const auto q = rotated(binding, rays[i]);
        const auto prior = geometry_options.prior;
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


RectifiedColor synthetic_rectified_color() {
    RectifiedColor c;
    c.R_color_base = {0, -1, 0, 0, 0, -1, 1, 0, 0};
    c.t_color_base = {0.125, 1.25, 0.25};
    c.fx = 200; c.fy = 160; c.cx = 640; c.cy = 100;
    return c;
}
TOptions synthetic_t_options() { return {0.8, 0, 0, 0, 50000000}; }

struct ProjectionProbe { double u = 10.75, v = 20.25; bool success = true; };
bool constant_projection(const void* context, Vec3, double& u, double& v) noexcept {
    const auto& p = *static_cast<const ProjectionProbe*>(context);
    u = p.u; v = p.v;
    return p.success;
}
void check_source(const nd::SourceId& a, const nd::SourceId& b) {
    CHECK(a.stream_id == b.stream_id && a.sequence == b.sequence);
    CHECK(a.color_index == b.color_index && a.depth_index == b.depth_index);
    CHECK(a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us);
    CHECK(a.host_ready_mono_ns == b.host_ready_mono_ns);
}
void check_optional(const std::optional<double>& a, const std::optional<double>& b) {
    CHECK(a.has_value() == b.has_value());
    if (a) CHECK(close(*a, *b, 1e-12));
}
void check_same_prefix(const BinWalkResult& a, const BinWalkResult& b) {
    CHECK(a.valid == b.valid && a.stop == b.stop && a.boundary == b.boundary);
    CHECK(a.passed_cells == b.passed_cells);
    check_optional(a.free_end_m, b.free_end_m);
    check_optional(a.block_m, b.block_m);
    check_optional(a.block_height_m, b.block_height_m);
    check_optional(a.slope_rad, b.slope_rad);
}

struct Fixture : DepthFixture {
    static constexpr std::size_t mask_width = 1280, mask_height = 800, mask_stride = 1296;
    RectifiedColor color = synthetic_rectified_color();
    ProjectionProbe probe;
    ColorProjection projection;
    TOptions t_options;
    std::vector<std::uint8_t> mask_bytes;
    std::unique_ptr<NormalTraversability> t;
    CaptureStamp depth;
    std::uint64_t exposure_tick = 0;

    explicit Fixture(TOptions options = synthetic_t_options(), const ProjectionProbe* constant = nullptr,
                     GeometryOptions go = synthetic_geometry_options())
        : DepthFixture(synthetic_binding(), go), probe(constant ? *constant : ProjectionProbe{}), t_options(options),
          mask_bytes(mask_stride * mask_height, 253) {
        projection = {1280, 800, 77, constant ? static_cast<const void*>(&probe) :
                                            static_cast<const void*>(&color),
                      constant ? constant_projection : project_rectified_color};
        t = std::make_unique<NormalTraversability>(*geometry, *grid, projection, t_options);
        CHECK(t->storage_bytes() <= 2U * 1024U * 1024U);
        full_plane();
        fill_mask(1);
    }
    void fill_mask(std::uint8_t value) {
        for (std::size_t y = 0; y < mask_height; ++y)
            std::fill_n(mask_bytes.begin() + static_cast<std::ptrdiff_t>(y * mask_stride), mask_width, value);
    }
    void sync_capture() {
        depth = {frame.metadata().source, 42,
                 1000000000LL + static_cast<std::int64_t>(++exposure_tick * 10000000ULL)};
        CHECK(depth.capture_mono_ns != depth.source.host_ready_mono_ns);
    }
    void advance() {
        DepthFixture::process();
        sync_capture();
    }
    RawTMask mask() const {
        return {depth, MaskMaterial::RawTraversable, projection.revision, 1280, 800,
                mask_stride, mask_bytes.size(), mask_bytes.data()};
    }
    Vec3 base_point(std::size_t i) const {
        const auto q = rotated(binding, rays[i]);
        const double z = raster[i] * static_cast<double>(binding.unit_m);
        return {q.x * z + binding.t_base_camera.x, q.y * z + binding.t_base_camera.y,
                q.z * z + binding.t_base_camera.z};
    }
    std::uint64_t grid_hash() const {
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
        word(static_cast<unsigned>(grid->result().status));
        word(grid->result().source.stream_id); word(grid->result().source.sequence);
        optional(grid->result().blind_m);
        for (const auto& c : grid->cells()) {
            word(c.expected); word(c.ground); word(c.covered_ground);
            number(c.mean_height_m); number(c.m2_height_m2);
            word(c.pierce); word(c.below); word(c.t_ok); optional(c.below_height_m);
        }
        for (const auto& b : grid->bins()) {
            word(b.valid_returns); word(b.valid_pierce_rays); word(b.expected_rays);
            const auto& p = b.traversal;
            word(p.valid); word(static_cast<unsigned>(p.stop)); word(static_cast<unsigned>(p.boundary));
            word(p.passed_cells); word(p.t_backed_prefix);
            optional(p.free_end_m); optional(p.block_m); optional(p.block_height_m); optional(p.slope_rad);
        }
        return hash;
    }
    const TReport& process_t(const RawTMask* view, const CaptureStamp& capture, const nd::Mailbox& owner) {
        const auto before_hash = grid_hash();
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto& report = t->process(owner, capture, view);
        CHECK(allocations.load(std::memory_order_relaxed) == before);
        ++t_process_calls;
        CHECK(grid_hash() == before_hash);
        return report;
    }
    const TReport& process_t(const RawTMask* view, const CaptureStamp& capture) {
        return process_t(view, capture, mailbox);
    }
    const TReport& process_t(const RawTMask* view) { return process_t(view, depth); }
    std::size_t central_bin() const { return static_cast<std::size_t>(bin_of({1, 0, 0})); }
};

void expect_fresh(const Fixture& f, const RawTMask& mask) {
    const auto& r = f.t->report();
    CHECK(r.status == TStatus::Fresh && r.segmentation);
    check_source(r.depth.source, f.depth.source);
    check_source(r.depth.source, f.geometry->result().source);
    check_source(r.depth.source, f.grid->result().source);
    CHECK(r.depth.clock_epoch == f.depth.clock_epoch && r.depth.capture_mono_ns == f.depth.capture_mono_ns);
    check_source(r.segmentation->source, mask.stamp.source);
    CHECK(r.segmentation->clock_epoch == mask.stamp.clock_epoch);
    CHECK(r.segmentation->capture_mono_ns == mask.stamp.capture_mono_ns);
    CHECK(r.age_ns == f.depth.capture_mono_ns - mask.stamp.capture_mono_ns);
    std::size_t ground = 0;
    for (std::size_t i = 0; i < f.t->counts().size(); ++i) {
        const auto& c = f.t->counts()[i];
        CHECK(c.ground == f.grid->cells()[i].ground && c.traversable <= c.ground);
        ground += c.ground;
    }
    CHECK(r.ground_projected + r.projection_misses == ground);
    CHECK(r.raw_cells == static_cast<std::size_t>(std::count(f.t->raw().begin(), f.t->raw().end(), 1)));
    CHECK(r.eroded_cells == static_cast<std::size_t>(std::count(f.t->eroded().begin(), f.t->eroded().end(), 1)));
}
void expect_no_t(const Fixture& f, bool retain_geometry) {
    const auto& r = f.t->report();
    CHECK(!r.segmentation && r.age_ns == -1);
    CHECK(r.ground_projected == 0 && r.projection_misses == 0);
    CHECK(r.raw_cells == 0 && r.eroded_cells == 0 && !r.near_support_missing);
    for (const auto& c : f.t->counts()) CHECK(c.ground == 0 && c.traversable == 0);
    for (const auto value : f.t->raw()) CHECK(value == 0);
    for (const auto value : f.t->eroded()) CHECK(value == 0);
    for (std::size_t b = 0; b < f.t->prefixes().size(); ++b) {
        const auto& p = f.t->prefixes()[b];
        CHECK(!p.t_backed_prefix);
        if (retain_geometry) check_same_prefix(p, f.grid->bins()[b].traversal);
        else {
            CHECK(!p.valid && p.stop == StopReason::InvalidInput && p.boundary == BoundaryKind::None);
            CHECK(!p.free_end_m && !p.block_m && !p.block_height_m && !p.slope_rad && p.passed_cells == 0);
        }
    }
}
void expect_unadvanced(const Fixture& f, std::size_t bin) {
    const auto& p = f.t->prefixes()[bin];
    CHECK(p.valid && p.passed_cells == 0 && !p.t_backed_prefix);
    CHECK(!p.free_end_m || close(*p.free_end_m, *f.grid->result().blind_m));
}

// Only ground pixels are projected, through the full rectified colour transform (translation
// included); per-cell ground and passable counts match an independent projection, not a naive 2x
// scaling.
void test_rectified_translation_and_ground_only_ledger() {
    Fixture f;
    const auto invalid = at(320, 200), obstacle = kGeometryPixels - 1;
    f.raster[invalid] = 0;
    f.raster[obstacle] = 187;
    for (std::size_t y = 0; y < Fixture::mask_height; ++y)
        for (std::size_t x = 0; x < Fixture::mask_width; ++x)
            f.mask_bytes[y * Fixture::mask_stride + x] = (x < 640 && y < 420) ? 1 : 0;
    f.advance();
    auto mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    CHECK(valid_rectified_color(f.color));
    Vec3 point;
    CHECK(!f.geometry->ground_point(invalid, point));
    CHECK(!f.geometry->ground_point(obstacle, point));
    CHECK(!f.geometry->ground_point(kGeometryPixels, point));
    std::vector<TCount> oracle(f.t->counts().size());
    std::size_t projected = 0, misses = 0, wrong_scaled_total = 0, true_total = 0;
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        const auto& e = f.geometry->pixels()[i];
        if (e.classification != SampleClass::Ground) continue;
        CHECK(f.geometry->ground_point(i, point));
        const auto expected = f.base_point(i);
        CHECK(close(point.x, expected.x) && close(point.y, expected.y) && close(point.z, expected.z));
        const int radial = f.cell_of(e.range_m);
        if (radial < 0) continue;
        auto& count = oracle[static_cast<std::size_t>(e.bin) * f.grid->cell_capacity_per_bin() +
                             static_cast<std::size_t>(radial)];
        ++count.ground;
        // Independent literal projection of this fixture's explicit rectified
        // color transform, including translations in all three optical axes.
        const double u = 200 * ((-expected.y + 0.125) / (expected.x + 0.25)) + 640;
        const double v = 160 * ((-expected.z + 1.25) / (expected.x + 0.25)) + 100;
        if (u < 0 || v < 0 || u >= 1280 || v >= 800) ++misses;
        else {
            ++projected;
            const auto x = static_cast<std::size_t>(std::floor(u));
            const auto y = static_cast<std::size_t>(std::floor(v));
            count.traversable += f.mask_bytes[y * Fixture::mask_stride + x];
            true_total += f.mask_bytes[y * Fixture::mask_stride + x];
        }
        const std::size_t naive_x = 2 * (i % kNativeWidth), naive_y = 2 * (i / kNativeWidth);
        wrong_scaled_total += f.mask_bytes[naive_y * Fixture::mask_stride + naive_x];
    }
    CHECK(projected > 0 && f.t->report().ground_projected == projected &&
          f.t->report().projection_misses == misses);
    CHECK(true_total > 0 && true_total != wrong_scaled_total);
    for (std::size_t i = 0; i < oracle.size(); ++i) {
        CHECK(f.t->counts()[i].ground == oracle[i].ground);
        CHECK(f.t->counts()[i].traversable == oracle[i].traversable);
    }
    const auto sample = at(400, 200);
    CHECK(f.geometry->ground_point(sample, point));
    double u = 0, v = 0;
    CHECK(project_rectified_color(&f.color, point, u, v));
    CHECK(std::abs(u - 800) > 10 && std::abs(v - 400) > 10);
    CHECK(f.mask_bytes[Fixture::mask_width] == 253);  // Padding is intentionally nonbinary.
}

// A projected point takes its mask pixel by floor; points outside the image, NaN/inf or a refused
// projection count as misses and are never clamped to the border.
void test_projection_floor_and_no_clamp() {
    struct Case { double u, v; bool callback; bool inside; std::size_t x, y; };
    const Case cases[] = {
        {10.75, 20.25, true, true, 10, 20},
        {1279.999, 799.999, true, true, 1279, 799},
        {-0.001, 20, true, false, 0, 20},
        {1280, 20, true, false, 1279, 20},
        {10, -0.001, true, false, 10, 0},
        {10, 800, true, false, 10, 799},
        {std::numeric_limits<double>::quiet_NaN(), 20, true, false, 0, 20},
        {10, std::numeric_limits<double>::infinity(), true, false, 10, 799},
        {10.75, 20.25, false, false, 10, 20},
    };
    for (const auto& test : cases) {
        const ProjectionProbe binding{test.u, test.v, test.callback};
        Fixture f(synthetic_t_options(), &binding);
        f.fill_mask(0);
        f.mask_bytes[test.y * Fixture::mask_stride + test.x] = 1;
        f.advance();
        auto mask = f.mask();
        CHECK(f.process_t(&mask).status == TStatus::Fresh);
        expect_fresh(f, mask);
        std::size_t ground = 0, traversable = 0;
        for (const auto& c : f.t->counts()) { ground += c.ground; traversable += c.traversable; }
        CHECK(ground > 0);
        CHECK(traversable == (test.inside ? ground : 0));
        CHECK(f.t->report().ground_projected == (test.inside ? ground : 0));
        CHECK(f.t->report().projection_misses == (test.inside ? 0 : ground));
        if (!test.inside) {
            CHECK(f.t->report().raw_cells == 0 && f.t->report().eroded_cells == 0);
            expect_unadvanced(f, f.central_bin());
        }
    }
}

// An all-passable mask with zero margin keeps the geometric FREE exactly and marks it T-backed.
void test_fresh_all_true_zero_margin() {
    Fixture f;
    f.advance();
    auto mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    CHECK(f.t->report().raw_cells > 0 && f.t->report().raw_cells == f.t->report().eroded_cells);
    CHECK(!f.t->report().near_support_missing);
    bool advanced = false;
    for (std::size_t b = 0; b < f.t->prefixes().size(); ++b) {
        check_same_prefix(f.t->prefixes()[b], f.grid->bins()[b].traversal);
        if (f.grid->bins()[b].traversal.passed_cells > 0) {
            CHECK(f.t->prefixes()[b].t_backed_prefix);
            advanced = true;
        }
    }
    CHECK(advanced);
}

// An all-false fresh mask stops FREE at once (Traversability) and is never treated as missing T;
// with T actually missing the geometric FREE is kept.
void test_fresh_false_never_becomes_missing() {
    Fixture f;
    f.fill_mask(0);
    f.advance();
    auto mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    CHECK(f.t->report().ground_projected > 0 && f.t->report().raw_cells == 0);
    CHECK(f.grid->bins()[f.central_bin()].traversal.passed_cells > 0);
    expect_unadvanced(f, f.central_bin());
    CHECK(f.t->prefixes()[f.central_bin()].stop == StopReason::Traversability);
    f.advance();
    CHECK(f.process_t(nullptr).status == TStatus::Missing);
    expect_no_t(f, true);
    CHECK(f.t->prefixes()[f.central_bin()].passed_cells > 0);
}

// With a positive margin a cell stays passable only if its near and neighbouring-sector cells are
// observed passable; an obstacle's block passes through unchanged.
void test_positive_margin_needs_near_and_sector_support() {
    auto options = synthetic_t_options();
    options.jitter_m = 0.01;
    Fixture f(options);
    f.raster[kGeometryPixels - 1] = 187;
    f.advance();
    auto mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    CHECK(f.t->report().near_support_missing && f.t->report().raw_cells > 0);
    const auto stride = f.grid->cell_capacity_per_bin(), bins = f.t->prefixes().size();
    for (std::size_t b = 0; b < bins; ++b) CHECK(f.t->eroded()[b * stride] == 0);
    CHECK(f.t->raw()[f.central_bin() * stride] == 1);
    expect_unadvanced(f, f.central_bin());
    bool sector_missing = false;
    for (std::size_t b = 0; b < bins; ++b) {
        for (std::size_t k = 1; k + 1 < f.grid->result().cells_per_bin; ++k) {
            const auto i = b * stride + k;
            if (f.t->raw()[i] == 0) continue;
            if ((b > 0 && f.t->raw()[(b - 1) * stride + k] == 0) ||
                (b + 1 < bins && f.t->raw()[(b + 1) * stride + k] == 0)) {
                CHECK(f.t->eroded()[i] == 0);
                sector_missing = true;
            }
        }
    }
    CHECK(sector_missing);
    const auto& obstacle = f.geometry->pixels().back();
    CHECK(obstacle.classification == SampleClass::Obstacle);
    const auto b = static_cast<std::size_t>(obstacle.bin);
    CHECK(f.t->prefixes()[b].boundary == BoundaryKind::Positive);
    check_optional(f.t->prefixes()[b].block_m, f.grid->bins()[b].traversal.block_m);
    check_optional(f.t->prefixes()[b].block_height_m, f.grid->bins()[b].traversal.block_height_m);
    CHECK(f.t->prefixes()[b].block_m && f.t->prefixes()[b].block_height_m);
}

// An older mask is used up to max_age inclusive; 1 ns older is Stale, a mask from the future is
// Future, and another clock epoch is InvalidIdentity.
void test_capture_clock_age_and_older_mask() {
    Fixture f;
    f.advance();
    auto previous = f.mask();
    CHECK(f.process_t(&previous).status == TStatus::Fresh);
    f.advance();
    CHECK(f.process_t(&previous).status == TStatus::Fresh);
    expect_fresh(f, previous);
    CHECK(f.t->report().age_ns == 10000000);
    CHECK(previous.stamp.source.sequence < f.depth.source.sequence);
    f.advance();
    auto boundary_age = f.mask();
    boundary_age.stamp.capture_mono_ns -= f.t_options.max_age_ns;
    CHECK(f.process_t(&boundary_age).status == TStatus::Fresh);
    expect_fresh(f, boundary_age);
    CHECK(f.t->report().age_ns == f.t_options.max_age_ns);
    f.advance();
    auto stale = f.mask();
    stale.stamp.capture_mono_ns -= f.t_options.max_age_ns + 1;
    CHECK(f.process_t(&stale).status == TStatus::Stale);
    expect_no_t(f, true);
    f.advance();
    auto future = f.mask();
    ++future.stamp.capture_mono_ns;
    CHECK(f.process_t(&future).status == TStatus::Future);
    expect_no_t(f, true);
    f.advance();
    auto wrong_epoch = f.mask();
    ++wrong_epoch.stamp.clock_epoch;
    CHECK(f.process_t(&wrong_epoch).status == TStatus::InvalidIdentity);
    expect_no_t(f, true);
}

// A mask must match the depth frame's full source identity or be strictly older in every field;
// any other mismatch, or an invalid capture stamp, is InvalidIdentity and drops T.
void test_full_source_binding_and_invalid_capture() {
    Fixture f;
    const auto reject_mask = [&](auto change) {
        f.advance();
        auto mask = f.mask();
        change(mask.stamp);
        CHECK(f.process_t(&mask).status == TStatus::InvalidIdentity);
        expect_no_t(f, true);
    };
    reject_mask([](CaptureStamp& s) { ++s.source.stream_id; });
    reject_mask([](CaptureStamp& s) { ++s.source.sequence; });
    reject_mask([](CaptureStamp& s) { ++s.source.color_index; });
    reject_mask([](CaptureStamp& s) { ++s.source.depth_index; });
    reject_mask([](CaptureStamp& s) { ++s.source.color_device_us; });
    reject_mask([](CaptureStamp& s) { ++s.source.depth_device_us; });
    reject_mask([](CaptureStamp& s) { ++s.source.host_ready_mono_ns; });
    reject_mask([](CaptureStamp& s) { s.clock_epoch = 0; });
    reject_mask([](CaptureStamp& s) { s.capture_mono_ns = -1; });
    reject_mask([](CaptureStamp& s) { --s.source.sequence; });
    reject_mask([](CaptureStamp& s) { --s.source.sequence; ++s.source.color_index; });
    f.advance();
    const auto real_previous = f.mask();
    CHECK(f.process_t(&real_previous).status == TStatus::Fresh);
    f.advance();
    auto zero_device = real_previous;
    zero_device.stamp.source.color_device_us = 0;
    CHECK(f.process_t(&zero_device).status == TStatus::InvalidIdentity);
    expect_no_t(f, true);
    f.advance();
    zero_device = real_previous;
    zero_device.stamp.source.depth_device_us = 0;
    CHECK(f.process_t(&zero_device).status == TStatus::InvalidIdentity);
    expect_no_t(f, true);
    // Each older-frame field must be strictly older independently; changing
    // just sequence leaves the other guards able to hide a missing comparison.
    std::uint64_t nd::SourceId::* const strict_fields[] = {
        &nd::SourceId::color_index, &nd::SourceId::depth_index,
        &nd::SourceId::color_device_us, &nd::SourceId::depth_device_us,
    };
    for (const auto field : strict_fields) {
        f.advance();
        auto old = f.mask();  // Save a real previously delivered frame identity.
        f.advance();
        old.stamp.source.*field = f.depth.source.*field;
        CHECK(f.process_t(&old).status == TStatus::InvalidIdentity);
        expect_no_t(f, true);
    }
    const auto reject_capture = [&](auto change) {
        f.advance();
        auto mask = f.mask();
        auto depth = f.depth;
        change(depth);
        CHECK(f.process_t(&mask, depth).status == TStatus::InvalidIdentity);
        expect_no_t(f, false);
        check_source(f.t->report().depth.source, depth.source);
    };
    reject_capture([](CaptureStamp& s) { ++s.source.stream_id; });
    reject_capture([](CaptureStamp& s) { ++s.source.sequence; });
    reject_capture([](CaptureStamp& s) { ++s.source.color_index; });
    reject_capture([](CaptureStamp& s) { ++s.source.depth_index; });
    reject_capture([](CaptureStamp& s) { ++s.source.color_device_us; });
    reject_capture([](CaptureStamp& s) { ++s.source.depth_device_us; });
    reject_capture([](CaptureStamp& s) { ++s.source.host_ready_mono_ns; });
    reject_capture([](CaptureStamp& s) { s.clock_epoch = 0; });
    reject_capture([](CaptureStamp& s) { s.capture_mono_ns = -1; });
}

// A mask with another projection revision, the wrong material, a bad size, stride or byte count,
// or a value other than 0/1 is refused; geometric blocks are kept and a good mask recovers.
void test_raw_material_layout_and_revision_rejections() {
    Fixture f;
    f.raster[kGeometryPixels - 1] = 187;
    f.advance();
    auto warm = f.mask();
    CHECK(f.process_t(&warm).status == TStatus::Fresh);
    CHECK(f.t->report().raw_cells > 0);
    const auto reject = [&](auto change, TStatus expected = TStatus::InvalidMask) {
        f.advance();
        auto mask = f.mask();
        change(mask);
        CHECK(f.process_t(&mask).status == expected);
        expect_no_t(f, true);
        const auto b = static_cast<std::size_t>(f.geometry->pixels().back().bin);
        CHECK(f.t->prefixes()[b].block_m && f.t->prefixes()[b].block_height_m);
    };
    reject([](RawTMask& m) { ++m.projection_revision; }, TStatus::InvalidIdentity);
    reject([](RawTMask& m) { m.material = MaskMaterial::Unknown; });
    reject([](RawTMask& m) { m.material = MaskMaterial::FusedOrImageEroded; });
    reject([](RawTMask& m) { --m.width; });
    reject([](RawTMask& m) { --m.height; });
    reject([](RawTMask& m) { m.stride_bytes = 1279; });
    reject([](RawTMask& m) { m.stride_bytes = 4097; });
    reject([](RawTMask& m) { --m.bytes; });
    reject([](RawTMask& m) { m.pixels = nullptr; });
    f.mask_bytes[(Fixture::mask_height - 1) * Fixture::mask_stride + Fixture::mask_width - 1] = 255;
    reject([](RawTMask&) {});
    f.fill_mask(1);
    f.advance();
    auto recovered = f.mask();
    CHECK(f.process_t(&recovered).status == TStatus::Fresh);
    expect_fresh(f, recovered);
    CHECK(f.mask_bytes[Fixture::mask_width] == 253);
}

// The former walk of process(): every native pixel, full_cell_index, ground_point, projection and
// the floor lookup, recomputed here as the reference for the grid-list walk (round 2, 2026-10-03).
void reference_counts(const Fixture& f, const RawTMask& mask, std::vector<TCount>& counts,
                      std::size_t& projected, std::size_t& misses) {
    const auto& gr = f.grid->result();
    const auto stride = f.grid->cell_capacity_per_bin();
    counts.assign(f.t->counts().size(), TCount{});
    projected = misses = 0;
    const auto& pixels = f.geometry->pixels();
    for (std::size_t p = 0; p < pixels.size(); ++p) {
        const auto& e = pixels[p];
        if (e.classification != SampleClass::Ground) continue;
        const int k = full_cell_index(e.range_m, *gr.blind_m, f.grid->options().dr_m, gr.cells_per_bin);
        if (k < 0) continue;
        auto& count = counts[static_cast<std::size_t>(e.bin) * stride + static_cast<std::size_t>(k)];
        ++count.ground;
        Vec3 point;
        CHECK(f.geometry->ground_point(p, point));
        double u = 0, v = 0;
        if (!f.projection.project(f.projection.context, point, u, v) || !std::isfinite(u) || !std::isfinite(v) ||
            u < 0 || v < 0 || u >= mask.width || v >= mask.height) {
            ++misses;
            continue;
        }
        ++projected;
        const auto x = static_cast<std::size_t>(std::floor(u)), y = static_cast<std::size_t>(std::floor(v));
        if (mask.pixels[y * mask.stride_bytes + x] == 1) ++count.traversable;
    }
}

// Round 2: process() walks NormalGrid::ground_cells and checks the mask eight bytes at a time.
// Counts and projected/missed points equal the former full-raster walk on a plane and on a plane
// with a raised block (fewer Ground pixels); a byte above 1 in any word position of the first,
// middle or last row is still InvalidMask; row padding is still never read.
void test_ground_list_walk_and_word_mask_check() {
    Fixture f;
    for (std::size_t y = 0; y < Fixture::mask_height; ++y)
        for (std::size_t x = 0; x < Fixture::mask_width; ++x)
            f.mask_bytes[y * Fixture::mask_stride + x] = static_cast<std::uint8_t>(((x >> 3) ^ (y >> 3)) & 1U);
    for (int scene = 0; scene < 2; ++scene) {
        if (scene == 1)
            for (std::size_t v = 250; v < 330; ++v)
                for (std::size_t u = 260; u < 380; ++u) f.raster[at(u, v)] = static_cast<std::uint16_t>(f.raster[at(u, v)] / 2);
        f.advance();
        auto mask = f.mask();
        CHECK(f.process_t(&mask).status == TStatus::Fresh);
        std::vector<TCount> expected;
        std::size_t projected = 0, misses = 0;
        reference_counts(f, mask, expected, projected, misses);
        CHECK(projected > 0);
        CHECK(f.t->report().ground_projected == projected && f.t->report().projection_misses == misses);
        bool same = expected.size() == f.t->counts().size();
        std::size_t traversable = 0;
        for (std::size_t i = 0; same && i < expected.size(); ++i) {
            same = expected[i].ground == f.t->counts()[i].ground &&
                   expected[i].traversable == f.t->counts()[i].traversable;
            traversable += expected[i].traversable;
        }
        CHECK(same);
        CHECK(traversable > 0 && traversable < projected);
        CHECK(f.grid->ground_cells().size() ==
              f.geometry->result().classes[static_cast<std::size_t>(SampleClass::Ground)]);
        if (scene == 1)
            CHECK(f.geometry->result().classes[static_cast<std::size_t>(SampleClass::Ground)] < kGeometryPixels);
    }
    // Ground beyond the last whole cell (range_max 1.1 m leaves a trailing partial cell, as in
    // test_normal_grid): listed with cell -1, skipped, and the counts still equal the full walk.
    {
        auto go = synthetic_geometry_options();
        go.range_max_m = 1.1;
        go.fit_near_max_m = 1.0;
        Fixture tail(synthetic_t_options(), nullptr, go);
        tail.advance();
        auto mask = tail.mask();
        CHECK(tail.process_t(&mask).status == TStatus::Fresh);
        std::size_t outside = 0;
        for (const auto& g : tail.grid->ground_cells()) outside += g.cell < 0 ? 1 : 0;
        CHECK(outside > 0 && outside < tail.grid->ground_cells().size());
        std::vector<TCount> expected;
        std::size_t projected = 0, misses = 0;
        reference_counts(tail, mask, expected, projected, misses);
        bool same = expected.size() == tail.t->counts().size() && projected > 0 &&
                    tail.t->report().ground_projected == projected && tail.t->report().projection_misses == misses;
        for (std::size_t i = 0; same && i < expected.size(); ++i)
            same = expected[i].ground == tail.t->counts()[i].ground &&
                   expected[i].traversable == tail.t->counts()[i].traversable;
        CHECK(same);
    }
    const std::size_t rows[] = {0, Fixture::mask_height / 2, Fixture::mask_height - 1};
    const std::size_t cols[] = {0, 7, 8, 9, Fixture::mask_width - 9, Fixture::mask_width - 8, Fixture::mask_width - 1};
    const std::uint8_t values[] = {2, 0x80, 0xFF};
    for (const auto row : rows)
        for (const auto col : cols)
            for (const auto value : values) {
                auto& byte = f.mask_bytes[row * Fixture::mask_stride + col];
                const auto saved = byte;
                byte = value;
                f.advance();
                auto mask = f.mask();
                CHECK(f.process_t(&mask).status == TStatus::InvalidMask);
                byte = saved;
            }
    CHECK(f.mask_bytes[Fixture::mask_width] == 253);  // padding stays non-binary and unread
    f.advance();
    auto clean = f.mask();
    CHECK(f.process_t(&clean).status == TStatus::Fresh);
}

// The rectified projector split at its rigid part (the camera-frame tail of
// project_rectified_color), for the split form of ColorProjection.
bool project_rectified_camera(const void* context, Vec3 q, double& u, double& v) noexcept {
    u = v = 0;
    const auto& c = *static_cast<const RectifiedColor*>(context);
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || q.z <= 0) return false;
    const double x = c.fx*(q.x/q.z)+c.cx, y = c.fy*(q.y/q.z)+c.cy;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    u = x; v = y;
    return true;
}

// Round 2: with a split projection the owner precomputes rotation * ray per pixel. On the same
// frames and masks it must give the generic path's counts, raw/eroded cells and prefixes (the
// rounding differs; these scenes have no ground point within it of a pixel edge), and a half
// split projection is refused at construction.
void test_split_projection_matches_generic() {
    Fixture f;
    ColorProjection split = f.projection;
    split.rotation = &f.color.R_color_base;
    split.translation = &f.color.t_color_base;
    split.project_camera = project_rectified_camera;
    NormalTraversability fast(*f.geometry, *f.grid, split, f.t_options);
    CHECK(fast.storage_bytes() > f.t->storage_bytes());
    for (std::size_t y = 0; y < Fixture::mask_height; ++y)
        for (std::size_t x = 0; x < Fixture::mask_width; ++x)
            f.mask_bytes[y * Fixture::mask_stride + x] = static_cast<std::uint8_t>(((x / 7) ^ (y / 5)) & 1U);
    for (int scene = 0; scene < 2; ++scene) {
        if (scene == 1)
            for (std::size_t v = 250; v < 330; ++v)
                for (std::size_t u = 260; u < 380; ++u) f.raster[at(u, v)] = static_cast<std::uint16_t>(f.raster[at(u, v)] / 2);
        f.advance();
        auto mask = f.mask();
        const auto& generic = f.process_t(&mask);
        CHECK(generic.status == TStatus::Fresh);
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto& split_report = fast.process(f.mailbox, f.depth, &mask);
        CHECK(allocations.load(std::memory_order_relaxed) == before);
        CHECK(split_report.status == TStatus::Fresh);
        CHECK(split_report.ground_projected == generic.ground_projected && generic.ground_projected > 0);
        CHECK(split_report.projection_misses == generic.projection_misses);
        CHECK(split_report.raw_cells == generic.raw_cells && split_report.eroded_cells == generic.eroded_cells);
        bool same = fast.counts().size() == f.t->counts().size();
        for (std::size_t i = 0; same && i < fast.counts().size(); ++i)
            same = fast.counts()[i].ground == f.t->counts()[i].ground &&
                   fast.counts()[i].traversable == f.t->counts()[i].traversable;
        CHECK(same);
        CHECK(fast.raw() == f.t->raw() && fast.eroded() == f.t->eroded());
        for (std::size_t b = 0; b < fast.prefixes().size(); ++b) check_same_prefix(fast.prefixes()[b], f.t->prefixes()[b]);
    }
    ColorProjection half = f.projection;
    half.rotation = &f.color.R_color_base;
    bool refused = false;
    try { NormalTraversability invalid(*f.geometry, *f.grid, half, f.t_options); }
    catch (const std::invalid_argument&) { refused = true; }
    CHECK(refused);
}

// A replayed frame, an ended or restarted stream, an old-stream mask or failed geometry clears T;
// good frames recover.
void test_replay_stop_restart_and_geometry_failure_clear() {
    Fixture f;
    f.raster[kGeometryPixels - 1] = 187;
    f.advance();
    auto mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    CHECK(f.t->report().raw_cells > 0 && f.t->report().eroded_cells > 0);
    CHECK(f.process_t(&mask).status == TStatus::InactiveOrStale);
    expect_no_t(f, false);
    f.advance();
    mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    const auto prior_stream_mask = mask;
    f.mailbox.end_stream();
    CHECK(f.process_t(&mask).status == TStatus::InactiveOrStale);
    expect_no_t(f, false);
    f.stream = f.mailbox.begin_stream();
    f.sequence = 0;
    CHECK(f.process_t(&mask).status == TStatus::InactiveOrStale);
    expect_no_t(f, false);
    f.advance();
    CHECK(f.process_t(&prior_stream_mask).status == TStatus::InvalidIdentity);
    expect_no_t(f, true);
    f.advance();
    mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    const auto blocked_bin = static_cast<std::size_t>(f.geometry->pixels().back().bin);
    CHECK(f.t->prefixes()[blocked_bin].block_m && f.t->prefixes()[blocked_bin].block_height_m);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    f.advance();
    mask = f.mask();
    CHECK(f.geometry->result().status == GeometryStatus::FitFailed);
    CHECK(f.process_t(&mask).status == TStatus::GeometryUnavailable);
    expect_no_t(f, false);
    f.full_plane();
    f.advance();
    mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    auto bad = f.next_view();
    bad.metadata.distortion.coefficients[0] = -0.0F;
    f.take(bad);
    CHECK(f.process_geometry().status == GeometryStatus::ProfileMismatch);
    f.process_grid();
    f.sync_capture();
    mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::GeometryUnavailable);
    expect_no_t(f, false);
}

// T checks the geometry's own mailbox: a lookalike owner with the same stream and sequence is
// refused, clears all T evidence and does not consume the real frame.
void test_mailbox_owner_collision_clears_and_recovers() {
    Fixture f;
    f.raster[kGeometryPixels - 1] = 187;
    nd::Mailbox other(kGeometryPixels);
    nd::Frame other_frame(kGeometryPixels);
    CHECK(other.begin_stream() == f.stream);
    const auto deliver = [&] {
        const auto view = f.next_view();
        f.take(view);
        CHECK(other.try_offer(view) == nd::Offer::Accepted);
        CHECK(other.try_take(other_frame) == nd::Take::Taken);
        CHECK(f.process_geometry().has_evidence());
        CHECK(f.process_grid().status == GridStatus::NormalEvidence);
        f.sync_capture();
        check_source(other_frame.metadata().source, f.depth.source);
        CHECK(other.active_stream() == f.mailbox.active_stream());
        CHECK(f.geometry->bound_to(f.mailbox) && !f.geometry->bound_to(other));
    };
    // T's first call must verify the geometry's actual Mailbox owner rather
    // than bind to whichever object happens to present the same generation.
    deliver();
    auto mask = f.mask();
    CHECK(f.process_t(&mask, f.depth, other).status == TStatus::InactiveOrStale);
    expect_no_t(f, false);
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    CHECK(f.t->report().raw_cells > 0 && f.t->report().eroded_cells > 0);
    const auto blocked_bin = static_cast<std::size_t>(f.geometry->pixels().back().bin);
    CHECK(f.t->prefixes()[blocked_bin].block_m && f.t->prefixes()[blocked_bin].block_height_m);
    // Reject after T-backed and geometric boundary evidence exist. The wrong
    // owner must clear all of it and leave A's pending sequence unconsumed.
    deliver();
    mask = f.mask();
    CHECK(f.process_t(&mask, f.depth, other).status == TStatus::InactiveOrStale);
    expect_no_t(f, false);
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
    // Missing-mask fallback must not retain geometry across an owner error.
    // The same correctly owned frame still keeps the existing Missing policy.
    deliver();
    CHECK(f.process_t(nullptr, f.depth, other).status == TStatus::InactiveOrStale);
    expect_no_t(f, false);
    CHECK(f.process_t(nullptr).status == TStatus::Missing);
    expect_no_t(f, true);
    deliver();
    mask = f.mask();
    CHECK(f.process_t(&mask).status == TStatus::Fresh);
    expect_fresh(f, mask);
}

// The constructor refuses a wrong mask size, a missing projection, bad options, a non-rotation or
// zero-focal colour transform and geometry from another chain; points behind the camera or non-
// finite are not projected.
void test_constructor_bindings_and_rectified_validation() {
    Fixture f;
    const auto reject = [&](ColorProjection p, TOptions o) {
        bool rejected = false;
        try { NormalTraversability invalid(*f.geometry, *f.grid, p, o); }
        catch (const std::invalid_argument&) { rejected = true; }
        CHECK(rejected);
    };
    auto p = f.projection;
    p.width = 1920; reject(p, f.t_options);
    p = f.projection; p.height = 1080; reject(p, f.t_options);
    p = f.projection; p.revision = 0; reject(p, f.t_options);
    p = f.projection; p.context = nullptr; reject(p, f.t_options);
    p = f.projection; p.project = nullptr; reject(p, f.t_options);
    auto options = f.t_options;
    options.tau = 1.1; reject(f.projection, options);
    options = f.t_options; options.v_max_mps = -1; reject(f.projection, options);
    options = f.t_options; options.omega_max_rps = -1; reject(f.projection, options);
    options = f.t_options; options.jitter_m = -1; reject(f.projection, options);
    options = f.t_options; options.max_age_ns = -1; reject(f.projection, options);
    auto color = f.color;
    color.R_color_base[0] = 0.1;
    CHECK(!valid_rectified_color(color));
    p = f.projection; p.context = &color; reject(p, f.t_options);
    color = f.color; color.fx = 0;
    CHECK(!valid_rectified_color(color));
    reject(p, f.t_options);
    color = f.color; color.R_color_base[1] = 1;  // Orthogonal reflection.
    CHECK(!valid_rectified_color(color));
    reject(p, f.t_options);
    double u = 0, v = 0;
    CHECK(!project_rectified_color(nullptr, {1, 0, 0}, u, v));
    CHECK(!project_rectified_color(&f.color, {-1, 0, 0}, u, v));
    CHECK(!project_rectified_color(&f.color, {std::numeric_limits<double>::infinity(), 0, 0}, u, v));
    DepthFixture other;
    bool owner_rejected = false;
    try { NormalTraversability invalid(*other.geometry, *f.grid, f.projection, f.t_options); }
    catch (const std::invalid_argument&) { owner_rejected = true; }
    CHECK(owner_rejected);
}
}  // namespace

int main() {
    const struct Test { const char* name; void (*run)(); } tests[] = {
        {"rectified_translation_ground_only_ledger", test_rectified_translation_and_ground_only_ledger},
        {"projection_floor_and_no_clamp", test_projection_floor_and_no_clamp},
        {"fresh_true_zero_margin_matches_geometry", test_fresh_all_true_zero_margin},
        {"fresh_false_never_becomes_missing", test_fresh_false_never_becomes_missing},
        {"positive_margin_near_sector_support_and_block", test_positive_margin_needs_near_and_sector_support},
        {"external_capture_clock_age_older_mask", test_capture_clock_age_and_older_mask},
        {"full_source_binding_invalid_capture", test_full_source_binding_and_invalid_capture},
        {"raw_material_layout_revision_rejections", test_raw_material_layout_and_revision_rejections},
        {"ground_list_walk_word_mask_check", test_ground_list_walk_and_word_mask_check},
        {"split_projection_matches_generic", test_split_projection_matches_generic},
        {"replay_restart_geometry_failure_clear", test_replay_stop_restart_and_geometry_failure_clear},
        {"mailbox_owner_collision_clear_recovery", test_mailbox_owner_collision_clears_and_recovers},
        {"constructor_owner_rectified_validation", test_constructor_bindings_and_rectified_validation},
    };
    for (const auto& test : tests) {
        test.run();
        std::cout << "PASS " << test.name << '\n';
    }
    const Fixture storage;
    std::cout << "{\"suite\":\"synthetic_normal_traversability\",\"tests_passed\":" << std::size(tests)
              << ",\"zero_cpp_allocation_geometry_calls\":" << geometry_process_calls
              << ",\"zero_cpp_allocation_grid_calls\":" << grid_process_calls
              << ",\"zero_cpp_allocation_t_calls\":" << t_process_calls
              << ",\"t_buffer_capacity_bytes\":" << storage.t->storage_bytes()
              << ",\"production_calibration_claim\":false,\"exposure_from_hostready\":false}\n";
}
