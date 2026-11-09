/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_normal_grid_ticket_cell.cpp
 * Brief: The ticket-cell decision of NormalGrid (in_cell) against the lookup it replaced.
 *
 * Description:
 * NormalGrid::process decides whether a Ground pixel redeems its own expectation ticket with
 * in_cell(ticket range, k) instead of a second cell_of(ticket range) == k. Three checks:
 *   1. in_cell(r, k) == (cell_of(r) == k) == (full_cell_index(r, ...) == k) for every cell edge
 *      and its neighbouring doubles, cell centres, real ticket ranges, non-finite and
 *      out-of-range values, and every k around the active range, with and without valid edges;
 *   2. every output of process() equals, bit for bit, the frozen parent class
 *      (tests/reference/normal_grid_parent.*) on the same geometry over synthetic scenes;
 *      one known difference of meaning since 2026-10-13: for negative evidence before the first
 *      cell the parent keeps the height of the nearest candidate, production the lowest measured
 *      one. The scenes here never carry such a height into a negative boundary, so the two
 *      still agree; a scene that does would differ in that one height and nowhere else;
 *   3. covered_ground and unmatched_ground equal a recount made here from the pixel evidence,
 *      the tickets and full_cell_index only.
 * All projection and threshold numbers are fixture inputs; nothing here certifies calibration,
 * wire Profiles or navigation permission.
 */

#include "modules/normal_grid.hpp"
#include "normal_grid_parent.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>

namespace perception::geometry {
// Friend of NormalGrid (normal_grid.hpp): the private lookups and tables read by these tests.
struct NormalGridTestAccess {
    static int cell_of(const NormalGrid& g, double r) { return g.cell_of(r); }
    static bool in_cell(const NormalGrid& g, double r, int k) { return g.in_cell(r, k); }
    static bool edges_valid(const NormalGrid& g) { return g.edges_valid_; }
    static void set_edges_valid(NormalGrid& g, bool valid) { g.edges_valid_ = valid; }
    static std::vector<double>& far_edges(NormalGrid& g) { return g.far_edges_; }
    static int ticket_bin(const NormalGrid& g, std::size_t p) { return g.expectation_[p].bin; }
    static double ticket_range(const NormalGrid& g, std::size_t p) { return g.expectation_[p].range_m; }
};
}  // namespace perception::geometry

namespace {
using namespace perception::geometry;
namespace nd = perception::native_depth;
namespace parent = perception::geometry::normal_grid_parent;
using Access = NormalGridTestAccess;

[[noreturn]] void fail(const char* expression, int line) {
    std::cerr << "normal grid ticket cell CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
}
#define CHECK(expression) do { if (!(expression)) fail(#expression, __LINE__); } while (false)

double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
std::size_t at(std::size_t u, std::size_t v) { return v * kNativeWidth + u; }
Vec3 rotated(const ProjectionBinding& b, Vec3 p) {
    const auto& m = b.R_base_camera;
    return {m[0] * p.x + m[1] * p.y + m[2] * p.z,
            m[3] * p.x + m[4] * p.y + m[5] * p.z,
            m[6] * p.x + m[7] * p.y + m[8] * p.z};
}
// Bit equality: also holds for NaN and distinguishes +0 from -0.
bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
bool same_optional(const std::optional<double>& a, const std::optional<double>& b) {
    return a.has_value() == b.has_value() && (!a || same_bits(*a, *b));
}

// Same synthetic camera as test_normal_grid: 640 x 400 pinhole one metre above the base origin.
ProjectionBinding synthetic_binding(Vec3 t = {0, 0, 1}) {
    ProjectionBinding b;
    b.intrinsics = {128, 128, 320, -128, 640, 400};
    b.distortion.reported = true;
    b.distortion.model = 0;
    b.unit_m = 1.0F / 1024.0F;
    b.R_base_camera = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    b.t_base_camera = t;
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
GridOptions grid_options(double dr) { return {dr, 0.9, 0.4, 0.02}; }

// Deterministic 64-bit generator (splitmix64); the scenes must not depend on the C library.
struct Random {
    std::uint64_t state;
    std::uint64_t next() {
        std::uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    std::size_t below(std::size_t n) { return static_cast<std::size_t>(next() % n); }
};

// How the Ground pixels of one frame divide at the changed line, recounted without NormalGrid.
struct Tally {
    std::size_t redeemed = 0, other_bin = 0, same_bin_other_cell = 0, no_ticket = 0;
    std::size_t unmatched() const { return other_bin + same_bin_other_cell + no_ticket; }
    void add(const Tally& t) {
        redeemed += t.redeemed; other_bin += t.other_bin;
        same_bin_other_cell += t.same_bin_other_cell; no_ticket += t.no_ticket;
    }
};

struct Fixture {
    ProjectionBinding binding;
    GeometryOptions geometry_options;
    GridOptions options;
    std::vector<Vec3> rays;
    std::vector<std::uint16_t> raster;
    nd::Mailbox mailbox{kGeometryPixels};
    nd::Frame frame{kGeometryPixels};
    std::unique_ptr<NativeGeometry> geometry;
    std::unique_ptr<NormalGrid> grid;                 // production
    std::unique_ptr<parent::NormalGrid> reference;    // frozen parent
    std::uint64_t stream = 0, sequence = 0;

    Fixture(ProjectionBinding b, GridOptions gr, GeometryOptions go = synthetic_geometry_options())
        : binding(b), geometry_options(go), options(gr), rays(kGeometryPixels), raster(kGeometryPixels) {
        for (std::size_t v = 0; v < kNativeHeight; ++v)
            for (std::size_t u = 0; u < kNativeWidth; ++u)
                rays[at(u, v)] = {(static_cast<double>(u) - b.intrinsics.cx) / b.intrinsics.fx,
                                 (static_cast<double>(v) - b.intrinsics.cy) / b.intrinsics.fy, 1};
        geometry = std::make_unique<NativeGeometry>(binding, rays.data(), rays.size(), geometry_options);
        grid = std::make_unique<NormalGrid>(*geometry, options);
        reference = std::make_unique<parent::NormalGrid>(*geometry, options);
        stream = mailbox.begin_stream();
    }
    // Depth codes of the plane normal . p + d = 0 (base frame); pixels that miss it or fall
    // outside the 16-bit code range become invalid depth.
    void plane(Plane p) {
        for (std::size_t i = 0; i < rays.size(); ++i) {
            const double z = -(dot(p.normal, binding.t_base_camera) + p.d) / dot(p.normal, rotated(binding, rays[i]));
            const double code = std::isfinite(z) ? std::round(z / binding.unit_m) : 0.0;
            raster[i] = code > 0 && code <= 65535 ? static_cast<std::uint16_t>(code) : 0;
        }
    }
    void add_noise(Random& random, int amplitude_codes) {
        for (auto& code : raster) {
            if (code == 0) continue;
            const int delta = static_cast<int>(random.below(static_cast<std::size_t>(2 * amplitude_codes + 1))) - amplitude_codes;
            const int value = static_cast<int>(code) + delta;
            code = static_cast<std::uint16_t>(std::clamp(value, 1, 65535));
        }
    }
    void add_holes(Random& random, std::size_t one_in) {
        for (auto& code : raster) if (random.below(one_in) == 0) code = 0;
    }
    // A rectangle whose depth is scaled: < 1 an obstacle in front of the ground, > 1 a return
    // from below the plane.
    void add_patch(Random& random, double scale) {
        const std::size_t w = 8 + random.below(72), h = 8 + random.below(56);
        const std::size_t u0 = random.below(kNativeWidth - w), v0 = random.below(kNativeHeight - h);
        for (std::size_t v = v0; v < v0 + h; ++v)
            for (std::size_t u = u0; u < u0 + w; ++u) {
                const double value = std::round(raster[at(u, v)] * scale);
                raster[at(u, v)] = value >= 1 && value <= 65535 ? static_cast<std::uint16_t>(value) : 0;
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
    // One frame through the geometry and then through both grids.
    void process() {
        CHECK(mailbox.try_offer(next_view()) == nd::Offer::Accepted);
        CHECK(mailbox.try_take(frame) == nd::Take::Taken);
        geometry->process(frame, mailbox);
        grid->process(mailbox);
        reference->process(mailbox);
    }
    // Both grids again on the frame they have already consumed (stale path).
    void replay() {
        grid->process(mailbox);
        reference->process(mailbox);
    }
};

// Check 2: everything process() leaves behind, except the diagnostic stage clocks.
void expect_same_outputs(const Fixture& f) {
    const auto& a = f.grid->result();
    const auto& b = f.reference->result();
    CHECK(a.status == b.status);
    CHECK(a.source.stream_id == b.source.stream_id && a.source.sequence == b.source.sequence);
    CHECK(a.source.color_index == b.source.color_index && a.source.depth_index == b.source.depth_index);
    CHECK(a.source.color_device_us == b.source.color_device_us && a.source.depth_device_us == b.source.depth_device_us);
    CHECK(a.source.host_ready_mono_ns == b.source.host_ready_mono_ns);
    CHECK(same_optional(a.blind_m, b.blind_m));
    CHECK(a.cells_per_bin == b.cells_per_bin);
    CHECK(a.unmatched_ground == b.unmatched_ground);
    CHECK(f.grid->cell_capacity_per_bin() == f.reference->cell_capacity_per_bin());
    const auto& ca = f.grid->cells();
    const auto& cb = f.reference->cells();
    CHECK(ca.size() == cb.size());
    for (std::size_t i = 0; i < ca.size(); ++i) {
        CHECK(ca[i].expected == cb[i].expected);
        CHECK(ca[i].ground == cb[i].ground);
        CHECK(ca[i].covered_ground == cb[i].covered_ground);
        CHECK(same_bits(ca[i].mean_height_m, cb[i].mean_height_m));
        CHECK(same_bits(ca[i].m2_height_m2, cb[i].m2_height_m2));
        CHECK(ca[i].pierce == cb[i].pierce && ca[i].below == cb[i].below && ca[i].t_ok == cb[i].t_ok);
        CHECK(same_optional(ca[i].below_height_m, cb[i].below_height_m));
    }
    const auto& ba = f.grid->bins();
    const auto& bb = f.reference->bins();
    CHECK(ba.size() == bb.size());
    for (std::size_t i = 0; i < ba.size(); ++i) {
        CHECK(ba[i].valid_returns == bb[i].valid_returns);
        CHECK(ba[i].valid_pierce_rays == bb[i].valid_pierce_rays);
        CHECK(ba[i].expected_rays == bb[i].expected_rays);
        const auto& ta = ba[i].traversal;
        const auto& tb = bb[i].traversal;
        CHECK(ta.valid == tb.valid && ta.stop == tb.stop && ta.boundary == tb.boundary);
        CHECK(ta.passed_cells == tb.passed_cells && ta.t_backed_prefix == tb.t_backed_prefix);
        CHECK(same_optional(ta.free_end_m, tb.free_end_m) && same_optional(ta.block_m, tb.block_m));
        CHECK(same_optional(ta.block_height_m, tb.block_height_m) && same_optional(ta.slope_rad, tb.slope_rad));
    }
    const auto& ga = f.grid->ground_cells();
    const auto& gb = f.reference->ground_cells();
    CHECK(ga.size() == gb.size());
    for (std::size_t i = 0; i < ga.size(); ++i) CHECK(ga[i].pixel == gb[i].pixel && ga[i].cell == gb[i].cell);
}

// Check 3: covered and unmatched Ground recounted from the pixel evidence, the tickets and
// full_cell_index. Neither cell_of nor in_cell is used here.
Tally expect_recount(const Fixture& f) {
    Tally tally;
    const auto& result = f.grid->result();
    if (result.status != GridStatus::NormalEvidence && result.status != GridStatus::NoGroundReference) {
        CHECK(result.unmatched_ground == 0);
        for (const auto& c : f.grid->cells()) CHECK(c.covered_ground == 0 && c.ground == 0);
        return tally;
    }
    const std::size_t count = result.cells_per_bin, capacity = f.grid->cell_capacity_per_bin();
    std::vector<std::size_t> covered(f.grid->cells().size(), 0);
    const auto& pixels = f.geometry->pixels();
    for (std::size_t p = 0; p < pixels.size(); ++p) {
        const auto& e = pixels[p];
        if (e.classification != SampleClass::Ground || !result.blind_m) continue;
        const int k = full_cell_index(e.range_m, *result.blind_m, f.options.dr_m, count);
        if (k < 0) continue;
        const int ticket_bin = Access::ticket_bin(*f.grid, p);
        if (ticket_bin < 0) { ++tally.no_ticket; continue; }
        if (ticket_bin != e.bin) { ++tally.other_bin; continue; }
        if (full_cell_index(Access::ticket_range(*f.grid, p), *result.blind_m, f.options.dr_m, count) != k) {
            ++tally.same_bin_other_cell;
            continue;
        }
        ++tally.redeemed;
        ++covered[static_cast<std::size_t>(e.bin) * capacity + static_cast<std::size_t>(k)];
    }
    CHECK(result.unmatched_ground == tally.unmatched());
    for (std::size_t i = 0; i < covered.size(); ++i) CHECK(f.grid->cells()[i].covered_ground == covered[i]);
    return tally;
}

struct EdgeCounts { std::size_t pairs = 0, inside = 0, at_edge_inside = 0, at_edge_outside = 0; };

// Check 1 on the grid's current frame state.
EdgeCounts expect_in_cell_is_cell_of(Fixture& f) {
    EdgeCounts counts;
    const auto& result = f.grid->result();
    const std::size_t count = result.cells_per_bin;
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> ranges = {
        std::numeric_limits<double>::quiet_NaN(), infinity, -infinity, 0.0, -0.0, -1.0, 1e-300,
        std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
        f.geometry_options.range_max_m, f.geometry_options.blind_near_lo_m, 1e9};
    std::vector<double> edges;
    const std::size_t edge_ranges_begin = ranges.size();
    if (result.blind_m) {
        edges.push_back(*result.blind_m);
        if (Access::edges_valid(*f.grid))
            for (std::size_t k = 0; k < count; ++k) edges.push_back(Access::far_edges(*f.grid)[k]);
        else
            for (std::size_t k = 1; k <= count; ++k)
                edges.push_back(*result.blind_m + static_cast<double>(k) * f.options.dr_m);
    }
    for (const double e : edges) {
        const double down = std::nextafter(e, -infinity), up = std::nextafter(e, infinity);
        for (const double r : {std::nextafter(down, -infinity), down, e, up, std::nextafter(up, infinity)})
            ranges.push_back(r);
    }
    const std::size_t edge_ranges_end = ranges.size();
    for (std::size_t i = 0; i + 1 < edges.size(); ++i) ranges.push_back(edges[i] + (edges[i + 1] - edges[i]) / 2);
    for (std::size_t p = 0; p < kGeometryPixels; p += 97)
        if (Access::ticket_bin(*f.grid, p) >= 0) ranges.push_back(Access::ticket_range(*f.grid, p));

    std::vector<int> cells = {INT_MIN, INT_MIN + 1, -3, -2, -1, INT_MAX - 1, INT_MAX,
                              static_cast<int>(kMaxRadialCells), static_cast<int>(kMaxRadialCells) + 1};
    for (int k = 0; k <= static_cast<int>(count) + 2; ++k) cells.push_back(k);

    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const double r = ranges[i];
        const int cell = Access::cell_of(*f.grid, r);
        const int oracle = result.blind_m ? full_cell_index(r, *result.blind_m, f.options.dr_m, count) : -1;
        CHECK(cell == oracle);
        const bool is_edge_range = i >= edge_ranges_begin && i < edge_ranges_end;
        for (const int k : cells) {
            const bool inside = Access::in_cell(*f.grid, r, k);
            CHECK(inside == (cell == k));
            ++counts.pairs;
            if (inside && k >= 0) ++counts.inside;
            if (is_edge_range && k >= 0 && k < static_cast<int>(count)) {
                if (inside) ++counts.at_edge_inside;
                else ++counts.at_edge_outside;
            }
        }
    }
    return counts;
}

std::size_t frames = 0, scenes_normal = 0, scenes_other = 0, edge_pairs = 0, edge_pairs_fallback = 0;
Tally total;

void check_frame(Fixture& f) {
    expect_same_outputs(f);
    total.add(expect_recount(f));
    ++frames;
    if (f.grid->result().status == GridStatus::NormalEvidence) ++scenes_normal;
    else ++scenes_other;
}

// in_cell against cell_of and full_cell_index on real frame states, then with the edge table
// switched off (the reference path), then before any frame.
void test_in_cell_is_cell_of() {
    const Vec3 mounts[] = {{0, 0, 1}, {0.5, 0.75, 1}};
    const Plane planes[] = {{{0, 0, 1}, 0}, {{0, 0, 1}, 0.15}, {{0, 0, 1}, -0.1}};
    for (const double dr : {0.25, 0.1, 0.07, 0.03}) {
        for (const auto& mount : mounts) {
            Fixture f(synthetic_binding(mount), grid_options(dr));
            {
                // No frame yet: no blind range, no active cell.
                const auto none = expect_in_cell_is_cell_of(f);
                CHECK(none.inside == 0 && none.pairs > 0);
            }
            for (const auto& plane : planes) {
                f.plane(plane);
                f.process();
                check_frame(f);
                CHECK(f.grid->result().status == GridStatus::NormalEvidence);
                CHECK(Access::edges_valid(*f.grid));
                const std::size_t count = f.grid->result().cells_per_bin;
                CHECK(count >= 2);
                const auto valid = expect_in_cell_is_cell_of(f);
                // Every active cell contains its own near edge and the two doubles above it, and
                // not the two below: the closed near side and the open far side were both hit.
                CHECK(valid.inside >= 3 * count);
                CHECK(valid.at_edge_inside >= 3 * count && valid.at_edge_outside > valid.at_edge_inside);
                edge_pairs += valid.pairs;
                // Reference path: the edge table is declared invalid and filled with values that
                // would give wrong answers if in_cell still read it.
                const std::vector<double> kept = Access::far_edges(*f.grid);
                std::fill(Access::far_edges(*f.grid).begin(), Access::far_edges(*f.grid).end(),
                          std::numeric_limits<double>::max());
                Access::set_edges_valid(*f.grid, false);
                const auto fallback = expect_in_cell_is_cell_of(f);
                Access::far_edges(*f.grid) = kept;
                Access::set_edges_valid(*f.grid, true);
                CHECK(fallback.inside >= 3 * count && fallback.pairs > 0);
                edge_pairs_fallback += fallback.pairs;
            }
        }
    }
}

// Whole frames: flat, moved and tilted ground, quantisation noise, holes, obstacles and
// below-plane patches, for two mounts and four cell sizes.
void test_process_equals_parent() {
    const Vec3 mounts[] = {{0, 0, 1}, {0.5, 0.75, 1}};
    std::uint64_t seed = 0x5ca1ab1e0ddba11ULL;
    for (const double dr : {0.25, 0.1, 0.07, 0.03}) {
        for (const auto& mount : mounts) {
            Fixture f(synthetic_binding(mount), grid_options(dr));
            Random random{seed += 0x1000193};
            // Before any frame, and a stale repeat, both grids refuse alike.
            f.replay();
            check_frame(f);
            CHECK(f.grid->result().status != GridStatus::NormalEvidence);
            for (const double d : {0.0, 0.15, -0.1, 0.05, -0.2}) {
                f.plane({{0, 0, 1}, d});
                f.process();
                check_frame(f);
            }
            for (const double angle : {0.02, -0.03, 0.06, -0.08}) {
                f.plane({{std::sin(angle), 0, std::cos(angle)}, 0.03});
                f.process();
                check_frame(f);
                f.plane({{0, std::sin(angle), std::cos(angle)}, -0.04});
                f.process();
                check_frame(f);
            }
            for (int round = 0; round < 6; ++round) {
                const double pitch = (static_cast<double>(random.below(1001)) - 500) / 10000;
                const double roll = (static_cast<double>(random.below(1001)) - 500) / 10000;
                const double d = (static_cast<double>(random.below(1001)) - 500) / 5000;
                const double nz = std::sqrt(1 - std::sin(pitch) * std::sin(pitch) - std::sin(roll) * std::sin(roll));
                f.plane({{std::sin(pitch), std::sin(roll), nz}, d});
                f.add_noise(random, 1 + round % 3);
                if (round % 2 == 0) f.add_holes(random, 7 + random.below(20));
                for (std::size_t n = random.below(4); n > 0; --n) f.add_patch(random, 0.5 + static_cast<double>(random.below(40)) / 100);
                for (std::size_t n = random.below(3); n > 0; --n) f.add_patch(random, 1.05 + static_cast<double>(random.below(40)) / 100);
                f.process();
                check_frame(f);
            }
            f.replay();
            check_frame(f);
            CHECK(f.grid->result().status == GridStatus::InactiveOrStale);
            // No ground at all: every pixel far above the prior plane, then no depth.
            f.plane({{0, 0, 1}, -0.7});
            f.process();
            check_frame(f);
            std::fill(f.raster.begin(), f.raster.end(), 0);
            f.process();
            check_frame(f);
            // And normal evidence again afterwards.
            f.plane({{0, 0, 1}, 0});
            f.process();
            check_frame(f);
            CHECK(f.grid->result().status == GridStatus::NormalEvidence);
        }
    }
}
}  // namespace

int main() {
    const struct Test { const char* name; void (*run)(); } tests[] = {
        {"in_cell_is_cell_of_at_every_edge", test_in_cell_is_cell_of},
        {"process_equals_parent_and_recount", test_process_equals_parent},
    };
    for (const auto& test : tests) {
        test.run();
        std::cout << "PASS " << test.name << '\n';
    }
    std::cout << "{\"suite\":\"normal_grid_ticket_cell\",\"tests_passed\":" << std::size(tests)
              << ",\"frames\":" << frames << ",\"normal_evidence_frames\":" << scenes_normal
              << ",\"other_status_frames\":" << scenes_other
              << ",\"ground_redeemed\":" << total.redeemed
              << ",\"ground_same_bin_other_cell\":" << total.same_bin_other_cell
              << ",\"ground_other_bin\":" << total.other_bin
              << ",\"ground_no_ticket\":" << total.no_ticket
              << ",\"in_cell_pairs\":" << edge_pairs
              << ",\"in_cell_pairs_reference_path\":" << edge_pairs_fallback << "}\n";
    // The scenes must have exercised both outcomes of the changed line, and both ways a ticket
    // can fail to be redeemed.
    CHECK(scenes_normal >= 100 && scenes_other >= 16);
    CHECK(total.redeemed > 1000000);
    CHECK(total.same_bin_other_cell > 100000);
    CHECK(total.other_bin > 100000);
}
