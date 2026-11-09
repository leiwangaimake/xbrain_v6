/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_semantic_kernel.cpp
 * Brief: Standalone synthetic semantic-kernel evidence.
 *
 * Description:
 * No device, calibration, freshness, class policy, wire-S, deployment or navigation acceptance claim.
 */

#include "output/rns_semantic.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>

namespace {
std::atomic<std::size_t> ordinary_allocations{0}, aligned_allocations{0};
std::size_t projection_calls = 0, restriction_calls = 0;

void* allocate(std::size_t size) {
    void* pointer = std::malloc(size == 0 ? 1 : size);
    if (!pointer) throw std::bad_alloc();
    ordinary_allocations.fetch_add(1, std::memory_order_relaxed);
    return pointer;
}
void* allocate_aligned(std::size_t size, std::size_t alignment) {
    void* pointer = nullptr;
    if (posix_memalign(&pointer, alignment, size == 0 ? 1 : size) != 0)
        throw std::bad_alloc();
    aligned_allocations.fetch_add(1, std::memory_order_relaxed);
    return pointer;
}
[[noreturn]] void fail(const char* expression, int line) {
    std::fprintf(stderr, "semantic kernel CHECK failed at line %d: %s\n", line, expression);
    std::abort();
}
#define CHECK(expression) do { if (!(expression)) fail(#expression, __LINE__); } while (false)
} // namespace

void* operator new(std::size_t size) { return allocate(size); }
void* operator new[](std::size_t size) { return allocate(size); }
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }
void* operator new(std::size_t size, std::align_val_t alignment) {
    return allocate_aligned(size, static_cast<std::size_t>(alignment));
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
    return allocate_aligned(size, static_cast<std::size_t>(alignment));
}
void operator delete(void* pointer, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::align_val_t) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }

namespace {
namespace rns = perception::rns;
namespace geometry = perception::geometry;
using geometry::BinWalkResult;
using geometry::BoundaryKind;
using geometry::CellEvidence;
using geometry::StopReason;
using rns::Point;
using rns::SemanticLayout;
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double poison = 47.125;

struct AllocationSnapshot {
    std::size_t ordinary = ordinary_allocations.load(std::memory_order_relaxed);
    std::size_t aligned = aligned_allocations.load(std::memory_order_relaxed);
    void unchanged() const {
        CHECK(ordinary_allocations.load(std::memory_order_relaxed) == ordinary);
        CHECK(aligned_allocations.load(std::memory_order_relaxed) == aligned);
    }
};

using Rectangle = std::array<Point, 4>;
Rectangle rectangle(float left, float bottom, float right, float top) {
    return {{{left, bottom}, {right, bottom}, {right, top}, {left, top}}};
}

// A larger supplied capacity catches stale optional evidence beyond active bins.
// Guard slots catch writes outside the exact caller-owned output interval.
struct Projection {
    std::array<std::optional<double>, rns::kMaxBins + 2> storage{};
    Projection() { storage.fill(poison); }
    std::optional<double>& operator[](std::size_t index) { return storage[index + 1]; }
    bool invoke(const Point* points, std::size_t count, SemanticLayout layout,
                double margin = 0, std::size_t capacity = rns::kMaxBins) {
        storage.fill(poison);
        const AllocationSnapshot before;
        const bool valid = rns::project_semantic_footprint(points, count, layout, margin,
                                                       storage.data() + 1, capacity);
        before.unchanged();
        ++projection_calls;
        CHECK(storage.front() == poison && storage.back() == poison);
        if (capacity <= rns::kMaxBins) {
            for (std::size_t index = capacity; index < rns::kMaxBins; ++index)
                CHECK((*this)[index] == poison);
            for (std::size_t index = 0; index < capacity; ++index) {
                if (!valid || index >= layout.bins) CHECK(!(*this)[index]);
                else if ((*this)[index]) {
                    CHECK(std::isfinite(*(*this)[index]));
                    CHECK(*(*this)[index] >= 0 && *(*this)[index] <= layout.range_max_m);
                }
            }
        }
        return valid;
    }
    template<std::size_t Count>
    bool invoke(const std::array<Point, Count>& points, SemanticLayout layout,
                double margin = 0, std::size_t capacity = rns::kMaxBins) {
        return invoke(points.data(), points.size(), layout, margin, capacity);
    }
};

void near_distance(const std::optional<double>& value, double expected) {
    CHECK(value);
    // Check both relevance (not arbitrary zero) and conservative numerical error.
    CHECK(*value <= std::nextafter(expected, std::numeric_limits<double>::infinity()));
    CHECK(std::abs(*value - expected) <= 1e-11 * std::max(1.0, expected));
}

// A footprint that misses the bin's centre ray still gets its true nearest distance in that bin.
void test_off_center_footprint() {
    Projection output;
    const auto points = rectangle(2.0F, 0.125F, 2.015625F, 0.140625F);
    const SemanticLayout layout{0, 0.5, 4, 1};
    // All y are positive, so the bin-centre ray y=0 misses the entire object.
    for (const auto point : points) {
        CHECK(point.y > 0);
        CHECK(std::atan2(point.y, point.x) > 0 && std::atan2(point.y, point.x) < 0.25);
    }
    CHECK(output.invoke(points, layout));
    near_distance(output[0], std::hypot(2.0, 0.125));
}

// A point on the edge shared by two sectors belongs to both closed sectors, including at the
// -pi/+pi seam.
void test_shared_closed_sector_edge() {
    Projection output;
    const auto points = rectangle(1.0F, 0, 1.5F, 0.125F);
    CHECK(output.invoke(points, {-0.75, 0.5, 4, 4}));
    CHECK(!output[0] && !output[3]);
    // Bin 1 intersects only the y=0 edge; both closed sectors must retain it.
    near_distance(output[1], 1);
    near_distance(output[2], 1);
    // The same geometric negative-x ray is both -pi and +pi. Neither domain
    // endpoint may disappear because atan2 or sin(pi) picks one sign.
    CHECK(output.invoke(rectangle(-2, -0.125F, -1, 0), {-pi + pi / 8, pi / 4, 4, 8}));
    near_distance(output[0], 1);
    near_distance(output[7], 1);
    for (std::size_t index = 1; index < 7; ++index) CHECK(!output[index]);
}

// A footprint containing the origin gives distance 0 in every bin; the 12-vertex maximum is
// accepted.
void test_origin_and_vertex_capacity() {
    Projection output;
    CHECK(output.invoke(rectangle(-0.25F, -0.5F, 0.25F, 0.5F), {-2.75, 0.5, 4, 12}));
    for (std::size_t index = 0; index < 12; ++index) CHECK(output[index] == 0);
    // The origin belongs to every closed sector even if it is only a vertex.
    CHECK(output.invoke(rectangle(0, 0, 0.25F, 0.5F), {-2.75, 0.5, 4, 12}));
    for (std::size_t index = 0; index < 12; ++index) CHECK(output[index] == 0);

    std::array<Point, rns::kSemanticVertices> polygon{};
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const double angle = static_cast<double>(index) * (2 * pi / polygon.size());
        polygon[index] = {static_cast<float>(0.25 * std::cos(angle)),
                          static_cast<float>(0.25 * std::sin(angle))};
    }
    CHECK(output.invoke(polygon, {-2.75, 0.5, 4, 12}));
    for (std::size_t index = 0; index < 12; ++index) CHECK(output[index] == 0);
    CHECK(output.invoke(polygon, {-2.5575, 0.005, 4, rns::kMaxBins}));
    for (std::size_t index = 0; index < rns::kMaxBins; ++index) CHECK(output[index] == 0);
}

// The inflation margin can bring a footprint lying beyond range_max into range.
void test_inflation_enters_range() {
    Projection output;
    const auto points = rectangle(2.25F, -0.125F, 2.5F, 0.125F);
    const SemanticLayout layout{0, 0.5, 2, 1};
    CHECK(output.invoke(points, layout));
    CHECK(!output[0]);
    CHECK(output.invoke(points, layout, 0.5));
    near_distance(output[0], 1.75);
}

// Inflation is a square envelope, and the nearest point may lie inside an edge rather than at a
// vertex.
void test_square_envelope_and_edge_nearest() {
    Projection output;
    CHECK(output.invoke(rectangle(1, 1, 1.5F, 1.5F), {pi / 4, 0.5, 4, 1}, 0.25));
    const double square_near = std::hypot(0.75, 0.75);
    near_distance(output[0], square_near);
    CHECK(*output[0] < std::hypot(1.0, 1.0) - 0.25);
    // Closest point lies inside the vertical edge, not at a polygon vertex.
    CHECK(output.invoke(rectangle(1, -0.5F, 2, 0.5F), {0, 0.5, 4, 1}));
    near_distance(output[0], 1);
    CHECK(*output[0] < std::hypot(1.0, 0.5));
}

// The nearest distance is taken over the part of the footprint inside the sector only.
void test_sector_clipped_nearest() {
    Projection output;
    // Global nearest vertex (1,.125) is outside [0.5,0.75]. The sector's
    // nearest point is the lower angular edge meeting x=1, r=1/cos(.5).
    CHECK(output.invoke(rectangle(1, 0.125F, 2, 1), {0.625, 0.25, 4, 1}));
    near_distance(output[0], 1 / std::cos(0.5));
    CHECK(*output[0] > std::hypot(1.0, 0.125));
}

// A diagonal edge is pushed out by the full square (Minkowski) margin, tighter than a bounding
// box.
void test_convex_diagonal_envelope() {
    Projection output;
    const std::array<Point, 4> diamond{{{1.5F, 0}, {2, -0.5F}, {2.5F, 0}, {2, 0.5F}}};
    // Along the sector's .1-radian lower edge, the original hull has x-y=1.5.
    // Square inflation shifts that diagonal by margin+margin, not margin.
    const SemanticLayout layout{0.15, 0.1, 4, 1};
    const double lower = layout.angle_min_rad - layout.angle_step_rad / 2;
    CHECK(output.invoke(diamond, layout));
    near_distance(output[0], 1.5 / (std::cos(lower) - std::sin(lower)));
    CHECK(output.invoke(diamond, layout, 0.125));
    near_distance(output[0], 1.25 / (std::cos(lower) - std::sin(lower)));
    // An axis-aligned bounding box is too broad to pass the nearest test.
    CHECK(*output[0] > 1.375 / std::cos(lower));
}

// 2*pi and 1.5*pi sector layouts include and exclude footprints correctly.
void test_wide_sector_coverage() {
    Projection output;
    CHECK(output.invoke(rectangle(1, -0.125F, 2, 0.125F), {0, 2 * pi, 4, 1}));
    near_distance(output[0], 1);
    CHECK(output.invoke(rectangle(-2, -0.125F, -1, 0.125F), {0, 2 * pi, 4, 1}));
    near_distance(output[0], 1);
    // A 270-degree sector includes the front but still excludes this rear box.
    CHECK(output.invoke(rectangle(-2, -0.125F, -1, 0.125F), {0, 1.5 * pi, 4, 1}));
    CHECK(!output[0]);
    CHECK(output.invoke(rectangle(1, -0.125F, 2, 0.125F), {0, 1.5 * pi, 4, 1}));
    near_distance(output[0], 1);
}

// A tiny positive margin never drops a real intersection that rounding could otherwise lose.
void test_tiny_positive_margin() {
    Projection output;
    const float positive_y = std::ldexp(1.0F, -100);
    const double margin = std::ldexp(1.0, -99);
    const auto points = rectangle(1, positive_y, 1.5F, 0.125F);
    const SemanticLayout layout{-0.25, 0.5, 4, 2};
    CHECK(static_cast<double>(positive_y) - margin < 0);
    CHECK(output.invoke(points, layout));
    CHECK(output[1]);
    // Outward numerical rounding may already conservatively touch bin 0 at
    // margin zero; the positive-margin case MUST retain its real intersection.
    CHECK(output.invoke(points, layout, margin));
    near_distance(output[0], 1);
    near_distance(output[1], 1);
}

// A footprint touching range_max counts and one float step beyond does not; far footprints enter
// only with a large enough margin.
void test_range_touch_and_coordinate_bound() {
    Projection output;
    const SemanticLayout layout{0, 0.5, 2, 1};
    CHECK(output.invoke(rectangle(2, -0.25F, 2.5F, 0.25F), layout));
    near_distance(output[0], 2);
    const float outside = std::nextafter(2.0F, std::numeric_limits<float>::infinity());
    CHECK(output.invoke(rectangle(outside, -0.25F, 2.5F, 0.25F), layout));
    CHECK(!output[0]);
    CHECK(output.invoke(rectangle(63, -0.125F, 64, 0.125F), {0, 0.5, 6, 1}));
    CHECK(!output[0]);
    CHECK(output.invoke(rectangle(-64, 63, -63, 64), {0, 0.5, 6, 1}));
    CHECK(!output[0]);
    CHECK(output.invoke(rectangle(63, -64, 64, -63), {0, 0.5, 6, 1}));
    CHECK(!output[0]);
    CHECK(output.invoke(rectangle(63, -0.125F, 64, 0.125F), {0, 0.5, 6, 1}, 64));
    CHECK(output[0] == 0);
}

// Malformed footprints are refused and the output is cleared.
void test_malformed_footprints_clear() {
    Projection output;
    const SemanticLayout layout{-0.25, 0.5, 4, 2};
    const auto good = rectangle(1, -0.125F, 2, 0.125F);
    CHECK(!output.invoke(nullptr, 4, layout));
    for (std::size_t count = 0; count < 3; ++count)
        CHECK(!output.invoke(good.data(), count, layout));
    std::array<Point, rns::kSemanticVertices + 1> too_many{};
    CHECK(!output.invoke(too_many, layout));
    auto bad = good;
    std::reverse(bad.begin(), bad.end());
    CHECK(!output.invoke(bad, layout));
    bad = good; bad[1] = bad[0];
    CHECK(!output.invoke(bad, layout));
    CHECK(!output.invoke(std::array<Point, 3>{{{1, 0}, {2, 0}, {3, 0}}}, layout));
    CHECK(!output.invoke(std::array<Point, 5>{{{1, -1}, {3, -1}, {2, 0}, {3, 1}, {1, 1}}}, layout));
    CHECK(!output.invoke(std::array<Point, 4>{{{1, -1}, {3, 1}, {3, -1}, {1, 1}}}, layout));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float beyond_bound = std::nextafter(64.0F, inf);
    for (const float value : {nan, inf, -inf, 65.0F, -65.0F, beyond_bound, -beyond_bound}) {
        bad = good; bad[2].x = value;
        CHECK(!output.invoke(bad, layout));
        bad = good; bad[2].y = value;
        CHECK(!output.invoke(bad, layout));
    }
    // Valid output capacity must be fully cleared, even on very early rejects.
    CHECK(!output.invoke(good, layout, 0, 1));
    CHECK(!output.invoke(good, layout, 0, 0));
    const AllocationSnapshot before;
    CHECK(!rns::project_semantic_footprint(good.data(), good.size(), layout, 0, nullptr, 2));
    before.unchanged();
    ++projection_calls;
}

// Invalid layouts, margins, bin counts and capacities are refused and the output is cleared.
void test_malformed_layouts_and_overflow_clear() {
    Projection output;
    const auto points = rectangle(1, -0.125F, 2, 0.125F);
    const SemanticLayout good{0, 0.5, 4, 1};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double value : {0.0, -0.5, nan, inf}) {
        auto bad = good; bad.angle_step_rad = value;
        CHECK(!output.invoke(points, bad));
        bad = good; bad.range_max_m = value;
        CHECK(!output.invoke(points, bad));
    }
    for (const double value : {nan, inf, -inf, 4.0, -4.0}) {
        auto bad = good; bad.angle_min_rad = value;
        CHECK(!output.invoke(points, bad));
    }
    for (const double margin : {-0.25, nan, inf, std::nextafter(64.0, inf),
                                std::numeric_limits<double>::max()})
        CHECK(!output.invoke(points, good, margin));
    auto bad = good; bad.bins = 0;
    CHECK(!output.invoke(points, bad));
    bad = good; bad.bins = rns::kMaxBins + 1;
    CHECK(!output.invoke(points, bad));
    bad = good; bad.angle_step_rad = std::numeric_limits<double>::max();
    CHECK(!output.invoke(points, bad));
    CHECK(!output.invoke(points, good, 0, rns::kMaxBins + 1));
}

using Cells = std::array<CellEvidence, 5>;
Cells cells() {
    Cells result{};
    for (auto& cell : result) cell.expected = cell.ground = cell.covered_ground = 4;
    result[1].mean_height_m = 0.03125;
    result[2].mean_height_m = 0.09375;
    result[3].mean_height_m = 0.34375;
    result[4].mean_height_m = 0.875;
    return result;
}
BinWalkResult prefix() {
    BinWalkResult result;
    result.valid = true;
    result.stop = StopReason::Boundary;
    result.boundary = BoundaryKind::Negative;
    result.free_end_m = 1.5;
    result.block_m = 1.75;
    result.block_height_m = -0.25;
    result.slope_rad = std::atan(1.0);
    result.passed_cells = 4;
    result.t_backed_prefix = true;
    return result;
}
void same_result(const BinWalkResult& a, const BinWalkResult& b) {
    CHECK(a.valid == b.valid && a.stop == b.stop && a.boundary == b.boundary);
    CHECK(a.free_end_m == b.free_end_m && a.block_m == b.block_m);
    CHECK(a.block_height_m == b.block_height_m && a.slope_rad == b.slope_rad);
    CHECK(a.passed_cells == b.passed_cells && a.t_backed_prefix == b.t_backed_prefix);
}
struct Restriction {
    BinWalkResult result = prefix();
    bool selected = true;
    bool invoke(const BinWalkResult& original, const CellEvidence* evidence,
                std::size_t count, double blind, double dr, double semantic) {
        result = prefix();
        selected = true;
        const AllocationSnapshot before;
        const bool valid = rns::restrict_semantic_prefix(original, evidence, count,
                                                       blind, dr, semantic, result, selected);
        before.unchanged();
        ++restriction_calls;
        if (!valid) {
            same_result(result, {});
            CHECK(!selected);
        } else {
            CHECK(result.valid);
            CHECK(result.passed_cells <= original.passed_cells);
            if (!original.free_end_m) CHECK(!result.free_end_m);
            if (result.free_end_m) {
                CHECK(original.free_end_m && *result.free_end_m <= *original.free_end_m);
                if (result.block_m) CHECK(*result.free_end_m < *result.block_m);
            }
        }
        return valid;
    }
    bool invoke(const BinWalkResult& original, const Cells& evidence, double semantic) {
        return invoke(original, evidence.data(), evidence.size(), 0.5, 0.25, semantic);
    }
};

// A semantic block S at or beyond the existing block (ties included) does not replace it.
void test_strict_selection_preserves_obstruction() {
    Restriction output;
    const auto evidence = cells();
    for (const auto boundary : {BoundaryKind::Negative, BoundaryKind::Positive}) {
        auto original = prefix();
        original.boundary = boundary;
        original.block_height_m = boundary == BoundaryKind::Negative ? -0.25 : 0.5;
        for (const double semantic : {1.75, std::nextafter(1.75, 2.0), 2.0}) {
            CHECK(output.invoke(original, evidence, semantic));
            CHECK(!output.selected);
            same_result(output.result, original);
        }
    }
    auto pending = prefix();
    pending.boundary = BoundaryKind::Positive;
    pending.block_height_m.reset();
    CHECK(output.invoke(pending, evidence, 1.75));
    CHECK(!output.selected);
    same_result(output.result, pending);
}

// A selected S becomes a positive boundary without the old block height; FREE keeps only the whole
// cells before S.
void test_selected_boundary_clears_old_height() {
    Restriction output;
    const auto evidence = cells();
    for (const auto boundary : {BoundaryKind::Negative, BoundaryKind::Positive}) {
        auto original = prefix();
        original.boundary = boundary;
        original.block_height_m = boundary == BoundaryKind::Negative ? -0.25 : 0.5;
        CHECK(output.invoke(original, evidence, 1.5));
        CHECK(output.selected && output.result.boundary == BoundaryKind::Positive);
        CHECK(output.result.block_m == 1.5 && !output.result.block_height_m);
        CHECK(output.result.passed_cells == 3 && output.result.free_end_m == 1.25);
        CHECK(output.result.stop == StopReason::Boundary && output.result.t_backed_prefix);
        CHECK(output.result.slope_rad == std::atan(0.25));
    }
}

// Cuts land on exact double cell edges, and the slope is recomputed over the retained cells only.
void test_exact_far_edges_and_retained_slope() {
    Restriction output;
    const auto evidence = cells();
    const auto original = prefix();
    CHECK(output.invoke(original, evidence, 1.25));
    CHECK(output.selected && output.result.passed_cells == 2 && output.result.free_end_m == 1);
    CHECK(output.result.slope_rad == std::atan(0.125));
    CHECK(output.invoke(original, evidence, std::nextafter(1.25, 2.0)));
    CHECK(output.result.passed_cells == 3 && output.result.free_end_m == 1.25);
    CHECK(output.result.slope_rad == std::atan(0.25));
    CHECK(output.invoke(original, evidence, std::nextafter(1.25, 0.0)));
    CHECK(output.result.passed_cells == 2 && output.result.slope_rad == std::atan(0.125));

    // Nonbinary grid values exercise the public SAME-double-edge requirement.
    // Testing below/exactly/above every edge catches floor((S-blind)/dr).
    constexpr double blind = 0.1, dr = 0.1;
    auto decimal = original;
    decimal.free_end_m = blind + static_cast<double>(decimal.passed_cells) * dr;
    decimal.slope_rad = std::atan((evidence[3].mean_height_m - evidence[2].mean_height_m) / dr);
    for (std::size_t edge_index = 1; edge_index <= decimal.passed_cells; ++edge_index) {
        const double far = blind + static_cast<double>(edge_index) * dr;
        for (const double semantic : {std::nextafter(far, 0.0), far, std::nextafter(far, 2.0)}) {
            CHECK(output.invoke(decimal, evidence.data(), evidence.size(), blind, dr, semantic));
            const std::size_t retained = semantic > far ? edge_index : edge_index - 1;
            CHECK(output.result.passed_cells == retained);
            CHECK(output.result.free_end_m == blind + static_cast<double>(retained) * dr);
            CHECK(output.result.t_backed_prefix == (retained > 0));
            double maximum = 0;
            for (std::size_t index = 1; index < retained; ++index)
                maximum = std::max(maximum, std::abs(evidence[index].mean_height_m -
                    evidence[index - 1].mean_height_m) / dr);
            if (retained >= 2) CHECK(output.result.slope_rad == std::atan(maximum));
            else CHECK(!output.result.slope_rad);
        }
    }
}

// Too few retained cells clear the slope (none also clear T); T is never upgraded; S at or inside
// the blind edge leaves no FREE.
void test_zero_retained_and_original_t_flag() {
    Restriction output;
    auto evidence = cells();
    auto original = prefix();
    CHECK(output.invoke(original, evidence, 0.75));
    CHECK(output.result.passed_cells == 0 && output.result.free_end_m == 0.5);
    CHECK(!output.result.slope_rad && !output.result.t_backed_prefix);
    CHECK(output.invoke(original, evidence, 1.0));
    CHECK(output.result.passed_cells == 1 && output.result.free_end_m == 0.75);
    CHECK(!output.result.slope_rad && output.result.t_backed_prefix);
    original.t_backed_prefix = false;
    CHECK(output.invoke(original, evidence, 1.25));
    CHECK(output.result.passed_cells == 2 && !output.result.t_backed_prefix);
    for (auto& cell : evidence) cell.t_ok = true;
    CHECK(output.invoke(original, evidence, 1.25));
    CHECK(!output.result.t_backed_prefix);
    for (const double semantic : {0.0, 0.25, 0.5}) {
        CHECK(output.invoke(original, evidence, semantic));
        CHECK(output.selected && output.result.block_m == semantic);
        CHECK(!output.result.free_end_m && output.result.passed_cells == 0);
        CHECK(!output.result.slope_rad && !output.result.t_backed_prefix);
    }
}

// S never extends FREE, and a null FREE stays null.
void test_no_free_extension_and_null_stays_null() {
    Restriction output;
    const auto evidence = cells();
    auto original = prefix();
    original.stop = StopReason::Coverage;
    original.free_end_m = 1;
    original.passed_cells = 2;
    original.slope_rad = std::atan(0.125);
    original.block_m = 2.5;
    CHECK(output.invoke(original, evidence, 2));
    CHECK(output.selected && output.result.block_m == 2);
    CHECK(output.result.free_end_m == 1 && output.result.passed_cells == 2);
    CHECK(output.result.stop == StopReason::Coverage && output.result.slope_rad == original.slope_rad);
    original.boundary = BoundaryKind::None;
    original.block_m.reset(); original.block_height_m.reset();
    CHECK(output.invoke(original, evidence, 2));
    CHECK(output.selected && output.result.free_end_m == 1 && output.result.passed_cells == 2);

    original.free_end_m.reset(); original.slope_rad.reset(); original.passed_cells = 0;
    original.t_backed_prefix = false;
    original.stop = StopReason::NoGroundReference;
    CHECK(output.invoke(original, evidence, 1.25));
    CHECK(output.selected && output.result.block_m == 1.25);
    CHECK(!output.result.free_end_m && !output.result.slope_rad && !output.result.t_backed_prefix);
    CHECK(output.invoke(original, nullptr, 0, 0.5, 0.25, 1.25));
    CHECK(!output.result.free_end_m);
}

// Malformed restriction inputs are refused, clearing both the result and the selection flag.
void test_malformed_restriction_clears() {
    Restriction output;
    const auto evidence = cells();
    const auto original = prefix();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double semantic : {-0.1, nan, inf})
        CHECK(!output.invoke(original, evidence, semantic));
    for (const double blind : {-0.1, nan, inf})
        CHECK(!output.invoke(original, evidence.data(), evidence.size(), blind, 0.25, 1.25));
    for (const double dr : {0.0, -0.1, nan, inf})
        CHECK(!output.invoke(original, evidence.data(), evidence.size(), 0.5, dr, 1.25));
    CHECK(!output.invoke(original, nullptr, evidence.size(), 0.5, 0.25, 1.25));
    CHECK(!output.invoke(original, evidence.data(), 3, 0.5, 0.25, 1.25));
    CHECK(!output.invoke(original, evidence.data(), geometry::kMaxRadialCells + 1, 0.5, 0.25, 1.25));
    auto bad = original; bad.valid = false;
    CHECK(!output.invoke(bad, evidence, 1.25));
    bad = original; bad.passed_cells = geometry::kMaxRadialCells + 1;
    CHECK(!output.invoke(bad, evidence, 1.25));
    bad = original; bad.free_end_m.reset();
    CHECK(!output.invoke(bad, evidence, 1.25));
    for (const double value : {-1.0, nan, inf, 1.6}) {
        bad = original; bad.free_end_m = value;
        CHECK(!output.invoke(bad, evidence, 1.25));
    }
    for (const double value : {-1.0, nan, inf}) {
        bad = original; bad.block_m = value;
        CHECK(!output.invoke(bad, evidence, 1.25));
    }
    for (const double value : {nan, inf}) {
        bad = original; bad.block_height_m = value;
        CHECK(!output.invoke(bad, evidence, 1.25));
        bad = original; bad.slope_rad = value;
        CHECK(!output.invoke(bad, evidence, 1.25));
    }
    // Whole-input validation includes cells outside the retained/original FREE.
    auto bad_cells = evidence; bad_cells.back().mean_height_m = nan;
    CHECK(!output.invoke(original, bad_cells, 1.25));
    bad_cells = evidence;
    bad_cells[0].mean_height_m = -std::numeric_limits<double>::max();
    bad_cells[1].mean_height_m = std::numeric_limits<double>::max();
    CHECK(!output.invoke(original, bad_cells, 1.25));
}

// Input and output may be the same object: selection, tie (unchanged) and NaN refusal (cleared)
// still hold.
void test_in_place_prefix_alias() {
    const auto evidence = cells();
    auto aliased = prefix();
    bool selected = false;
    const auto invoke = [&](double semantic) {
        const AllocationSnapshot before;
        const bool valid = rns::restrict_semantic_prefix(aliased, evidence.data(), evidence.size(),
                                                       0.5, 0.25, semantic, aliased, selected);
        before.unchanged();
        ++restriction_calls;
        return valid;
    };
    CHECK(invoke(1.25));
    CHECK(selected && aliased.valid && aliased.boundary == BoundaryKind::Positive);
    CHECK(aliased.block_m == 1.25 && !aliased.block_height_m);
    CHECK(aliased.free_end_m == 1 && aliased.passed_cells == 2);
    CHECK(aliased.slope_rad == std::atan(0.125) && aliased.t_backed_prefix);
    const auto before_tie = aliased;
    CHECK(invoke(1.25));
    CHECK(!selected);
    same_result(aliased, before_tie);
    selected = true;
    CHECK(!invoke(std::numeric_limits<double>::quiet_NaN()));
    CHECK(!selected);
    same_result(aliased, {});
}
} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const std::array<Test, 19> tests{{
        {"off-center footprint without center-ray hit", test_off_center_footprint},
        {"shared closed-sector edge", test_shared_closed_sector_edge},
        {"origin coverage and 12-vertex capacity", test_origin_and_vertex_capacity},
        {"inflation entering measured range", test_inflation_enters_range},
        {"square envelope and edge-interior nearest", test_square_envelope_and_edge_nearest},
        {"sector-clipped nearest distance", test_sector_clipped_nearest},
        {"convex diagonal Minkowski envelope", test_convex_diagonal_envelope},
        {"wide sector coverage and exclusions", test_wide_sector_coverage},
        {"tiny positive margin cannot omit boundary", test_tiny_positive_margin},
        {"closed range touch and coordinate bound", test_range_touch_and_coordinate_bound},
        {"malformed footprints clear valid capacity", test_malformed_footprints_clear},
        {"malformed layouts and overflow clear", test_malformed_layouts_and_overflow_clear},
        {"strict selection preserves obstruction and ties", test_strict_selection_preserves_obstruction},
        {"selected pending-S clears obsolete height", test_selected_boundary_clears_old_height},
        {"exact double far edges and retained slope", test_exact_far_edges_and_retained_slope},
        {"zero retained cells and original T flag", test_zero_retained_and_original_t_flag},
        {"no FREE extension and null remains null", test_no_free_extension_and_null_stays_null},
        {"malformed restriction clears result and selection", test_malformed_restriction_clears},
        {"same-object prefix input and output alias", test_in_place_prefix_alias}
    }};
    for (const auto& test : tests) {
        test.run();
        std::printf("PASS: %s\n", test.name);
    }
    std::printf("semantic kernel: %zu/%zu PASS; %zu projection calls, %zu restriction calls; "
                "zero ordinary/aligned C++ allocations per call\n",
                tests.size(), tests.size(), projection_calls, restriction_calls);
}
