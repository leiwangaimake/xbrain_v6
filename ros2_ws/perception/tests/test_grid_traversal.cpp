/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_grid_traversal.cpp
 * Brief: Unit tests for the normal-grid bin walk that produces d_free, d_block and heights.
 *
 * Description:
 * What it checks. Cases cover empty input and range, coverage and continuity, boundaries and
 * heights, the tail and partial cells, the slope, roughness and T stops, invalid inputs, the
 * actual edge helpers, and bounded random properties of the walk.
 */

#include "modules/normal_grid.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>

namespace {
bool count_allocations = false;
std::size_t allocations = 0, checked_walk_calls = 0, checked_helper_calls = 0;
int failures = 0;
}

void* operator new(std::size_t size) {
    if (count_allocations) ++allocations;
    if (void* pointer = std::malloc(size == 0 ? 1 : size)) return pointer;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void* operator new(std::size_t size, std::align_val_t alignment) {
    if (count_allocations) ++allocations;
    void* pointer = nullptr;
    if (posix_memalign(&pointer, static_cast<std::size_t>(alignment), size == 0 ? 1 : size) != 0) {
        throw std::bad_alloc();
    }
    return pointer;
}
void* operator new[](std::size_t size, std::align_val_t alignment) { return ::operator new(size, alignment); }
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::align_val_t) noexcept { std::free(pointer); }
#if defined(__cpp_sized_deallocation)
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }
#endif

namespace {
using namespace perception::geometry;

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        ++failures; \
    } \
} while (false)

struct Fixture {
    std::array<CellEvidence, kMaxRadialCells> cells{};
    GridOptions options{0.25, 0.8, std::atan(1.0), 0.125};
    BinWalkInput input;
    Fixture() {
        for (auto& cell : cells) {
            cell.expected = 5;
            cell.ground = 8;
            cell.covered_ground = 4;
            cell.t_ok = true;
        }
        input.cells = cells.data();
        input.cell_count = 8;
        input.blind_m = 0.5;
        input.range_max_m = 3.0;
        input.has_samples = true;
    }
};

BinWalkResult run(const BinWalkInput& input, const GridOptions& options) {
    const auto before = allocations;
    count_allocations = true;
    const auto result = walk_normal_bin(input, options);
    count_allocations = false;
    ++checked_walk_calls;
    CHECK(allocations == before);
    if (!result.valid) {
        CHECK(result.stop == StopReason::InvalidInput);
        CHECK(result.boundary == BoundaryKind::None);
        CHECK(!result.free_end_m && !result.block_m && !result.block_height_m && !result.slope_rad);
        CHECK(result.passed_cells == 0 && !result.t_backed_prefix);
    } else {
        CHECK(result.passed_cells <= input.cell_count);
        CHECK((result.boundary == BoundaryKind::None) == !result.block_m.has_value());
        if (result.free_end_m) {
            CHECK(*result.free_end_m >= input.blind_m && *result.free_end_m <= input.range_max_m);
            if (result.block_m) CHECK(*result.free_end_m < *result.block_m);
        }
        CHECK(result.slope_rad.has_value() == (result.passed_cells >= 2));
        CHECK(result.t_backed_prefix == (input.has_eroded_t && result.passed_cells > 0));
        if (result.block_height_m) {
            CHECK(result.block_m.has_value());
            CHECK(result.boundary == BoundaryKind::Negative ? *result.block_height_m <= 0 : *result.block_height_m > 0);
        }
    }
    return result;
}

BinWalkResult run(const Fixture& fixture) { return run(fixture.input, fixture.options); }

int index(double range, double blind, double dr, std::size_t count) {
    const auto before = allocations;
    count_allocations = true;
    const int result = full_cell_index(range, blind, dr, count);
    count_allocations = false;
    ++checked_helper_calls;
    CHECK(allocations == before);
    return result;
}

std::size_t count(double blind, double range, double dr, std::size_t capacity) {
    const auto before = allocations;
    count_allocations = true;
    const auto result = full_cell_count(blind, range, dr, capacity);
    count_allocations = false;
    ++checked_helper_calls;
    CHECK(allocations == before);
    return result;
}

void expect_stop(const BinWalkResult& result, StopReason reason, std::size_t passed,
                std::optional<double> free_end) {
    CHECK(result.valid);
    CHECK(result.stop == reason);
    CHECK(result.passed_cells == passed);
    CHECK(result.free_end_m == free_end);
}

// An unobstructed walk stops at range_max (Range), an empty bin stops with NoSamples (leftover
// evidence then makes it invalid), and zero-cell and maximum-cell grids end exactly where
// expected.
void test_empty_and_range() {
    Fixture fixture;
    auto result = run(fixture);
    expect_stop(result, StopReason::Range, 8, 2.5);
    CHECK(result.slope_rad == 0.0 && !result.block_m && !result.t_backed_prefix);

    fixture.input.has_samples = false;
    for (auto& cell : fixture.cells) {
        cell.ground = cell.covered_ground = 0;
    }
    result = run(fixture);
    expect_stop(result, StopReason::NoSamples, 0, std::nullopt);
    CHECK(!result.block_m && !result.block_height_m && !result.slope_rad);
    fixture.cells[7].ground = 1;
    CHECK(!run(fixture).valid);
    fixture.cells[7].ground = 0;
    fixture.cells[7].pierce = true;
    CHECK(!run(fixture).valid);

    fixture.input.cell_count = 0;
    fixture.input.cells = nullptr;
    fixture.input.has_samples = true;
    fixture.input.has_eroded_t = true;
    expect_stop(run(fixture), StopReason::Range, 0, 0.5);
    fixture.input.positive_m = 0.25;
    fixture.input.positive_height_m = 0.1;
    expect_stop(run(fixture), StopReason::Boundary, 0, std::nullopt);
    fixture.input.has_samples = false;
    CHECK(!run(fixture).valid);

    Fixture complete;
    complete.input.blind_m = 0;
    complete.input.range_max_m = 6;
    complete.options.dr_m = 0.5;
    complete.input.cell_count = 12;
    expect_stop(run(complete), StopReason::Range, 12, 6.0);
    complete.input.positive_m = 6;
    complete.input.positive_height_m = 0.25;
    expect_stop(run(complete), StopReason::Boundary, 11, 5.5);
    complete.input.positive_m.reset();
    complete.input.positive_height_m.reset();
    complete.options.dr_m = 0.015625;
    complete.input.cell_count = kMaxRadialCells;
    expect_stop(run(complete), StopReason::Range, kMaxRadialCells, 4.0);
}

// FREE stops at the first cell whose covered share is below cover_min (equal passes); a failed
// early cell does not hide a later negative boundary; impossible counts are invalid.
void test_coverage_and_continuity() {
    Fixture fixture;
    fixture.cells[1].expected = fixture.cells[1].covered_ground = 0;
    expect_stop(run(fixture), StopReason::Coverage, 1, 0.75);
    fixture.cells[1].expected = 5;
    fixture.cells[1].ground = 0;
    expect_stop(run(fixture), StopReason::Coverage, 1, 0.75);
    fixture.cells[1].ground = 8;
    expect_stop(run(fixture), StopReason::Coverage, 1, 0.75);
    fixture.cells[1].covered_ground = 3;
    expect_stop(run(fixture), StopReason::Coverage, 1, 0.75);
    fixture.cells[1].covered_ground = 4;
    expect_stop(run(fixture), StopReason::Range, 8, 2.5);
    fixture.options.cover_min = std::nextafter(0.8, 1.0);
    expect_stop(run(fixture), StopReason::Coverage, 0, 0.5);

    // A failed first cell must not hide negative evidence in a later cell.
    fixture.options.cover_min = 0.8;
    fixture.cells[0].covered_ground = 0;
    fixture.cells[5].below = true;
    fixture.cells[5].below_height_m = -0.125;
    const auto result = run(fixture);
    expect_stop(result, StopReason::Coverage, 0, 0.5);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 1.75);
    CHECK(result.block_height_m == -0.125);

    Fixture huge_count;
    huge_count.options.cover_min = 1;
    huge_count.cells[0].expected = std::numeric_limits<std::size_t>::max();
    huge_count.cells[0].ground = huge_count.cells[0].expected;
    huge_count.cells[0].covered_ground = huge_count.cells[0].expected - 1;
    CHECK(!run(huge_count).valid);
    huge_count.cells[0].expected = huge_count.cells[0].ground = kGeometryPixels;
    huge_count.cells[0].covered_ground = kGeometryPixels - 1;
    expect_stop(run(huge_count), StopReason::Coverage, 0, 0.5);
    huge_count.input.cell_count = 1;
    huge_count.cells[0].covered_ground = kGeometryPixels;
    expect_stop(run(huge_count), StopReason::Range, 1, 0.75);
}

// FREE ends at the last whole cell before the nearest boundary; the nearer of positive and
// negative wins. A positive boundary carries its own height. A negative boundary carries the
// lowest below-plane height measured anywhere in the bin (19 section 3.2: neg_h[i] is a per-bin
// minimum; section 3.4: measured in-pit points ? neg_h[i] : null), also when the boundary itself
// was located by pierce alone, and no height when nothing was measured below the plane.
void test_boundaries_and_heights() {
    Fixture fixture;
    fixture.input.positive_m = 1.0;
    fixture.input.positive_height_m = 0.25;
    auto result = run(fixture);
    expect_stop(result, StopReason::Boundary, 1, 0.75);
    CHECK(result.boundary == BoundaryKind::Positive && result.block_height_m == 0.25);
    fixture.input.positive_m = 0.25;
    expect_stop(run(fixture), StopReason::Boundary, 0, std::nullopt);
    fixture.input.positive_m = 0.5;
    expect_stop(run(fixture), StopReason::Boundary, 0, std::nullopt);

    fixture.input.blind_m = 0.6;
    fixture.input.positive_m = 0.8;
    expect_stop(run(fixture), StopReason::Boundary, 0, 0.6);

    Fixture negative;
    negative.input.positive_m = 1.1;
    negative.input.positive_height_m = 0.5;
    negative.cells[2].below = true;
    negative.cells[2].below_height_m = -0.25;
    result = run(negative);
    expect_stop(result, StopReason::Boundary, 1, 0.75);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 1.0);
    CHECK(result.block_height_m == -0.25);
    negative.input.positive_m = 1.0;
    CHECK(run(negative).boundary == BoundaryKind::Negative);
    negative.input.positive_m = 0.875;
    result = run(negative);
    CHECK(result.boundary == BoundaryKind::Positive && result.block_m == 0.875);
    CHECK(result.block_height_m == 0.5);

    // A nearer pierce boundary keeps its own distance and FREE, and reports the depth measured
    // farther out in the same bin: the edge comes from the pierce, the height from the in-pit
    // returns.
    negative.input.positive_m = 2.0;
    negative.cells[1].pierce = true;
    result = run(negative);
    expect_stop(result, StopReason::Boundary, 0, 0.5);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.75);
    CHECK(result.block_height_m == -0.25);
    // The lowest measured height of the bin, not the one of the cell that holds the boundary:
    // a shallower return in the boundary cell does not replace the deeper one behind it ...
    negative.cells[1].below = true;
    negative.cells[1].below_height_m = -0.0625;
    result = run(negative);
    CHECK(result.block_m == 0.75 && result.block_height_m == -0.25);
    // ... and a deeper one in the boundary cell is not replaced by a shallower one behind it.
    negative.cells[2].below_height_m = -0.03125;
    CHECK(run(negative).block_height_m == -0.0625);
    negative.cells[2].below_height_m = -0.25;
    // A measured height behind a positive boundary that wins never replaces the positive height.
    negative.input.positive_m = 0.625;
    result = run(negative);
    CHECK(result.boundary == BoundaryKind::Positive && result.block_height_m == 0.5);
    negative.input.positive_m = 2.0;

    // Not a window behind the boundary and not a count of cells: the only measured depth lies
    // four cells (1 m) behind a pierce-only edge, and it still counts when a positive obstacle
    // stands between the two. That is the letter of 19 section 3.2 (neg_h[i] accumulates over the
    // whole bin); whether a point that far behind the edge should count is an open question
    // (tests/rns_host/NORMAL_GRID.md item 4), and this case pins what is implemented meanwhile.
    Fixture far;
    far.cells[1].pierce = true;
    far.cells[5].below = true;
    far.cells[5].below_height_m = -0.25;
    result = run(far);
    expect_stop(result, StopReason::Boundary, 0, 0.5);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.75 && result.block_height_m == -0.25);
    far.input.positive_m = 1.25;
    far.input.positive_height_m = 0.5;
    result = run(far);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.75 && result.block_height_m == -0.25);
    // Four negative cells in a row, the deepest return in the last of them.
    Fixture row;
    for (std::size_t k = 1; k <= 4; ++k) {
        row.cells[k].below = true;
        row.cells[k].below_height_m = k == 4 ? -0.5 : -0.0625;
    }
    result = run(row);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.75 && result.block_height_m == -0.5);

    // Pierce alone, nothing measured below the plane anywhere in the bin: no height.
    Fixture pierce_only;
    pierce_only.cells[1].pierce = true;
    pierce_only.cells[4].pierce = true;
    result = run(pierce_only);
    expect_stop(result, StopReason::Boundary, 0, 0.5);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.75 && !result.block_height_m);

    negative.input.before_grid_negative_m = 0.25;
    negative.input.before_grid_negative_height_m = -0.5;
    result = run(negative);
    expect_stop(result, StopReason::Boundary, 0, std::nullopt);
    CHECK(result.block_m == 0.25 && result.block_height_m == -0.5);
    // A boundary before the grid without a measured height takes the bin's lowest measured one,
    // and a shallower height before the grid does not hide a deeper one in the cells.
    negative.input.before_grid_negative_m = 0.5;
    negative.input.before_grid_negative_height_m.reset();
    result = run(negative);
    expect_stop(result, StopReason::Boundary, 0, std::nullopt);
    CHECK(result.block_m == 0.5 && result.block_height_m == -0.25);
    negative.input.before_grid_negative_height_m = -0.125;
    result = run(negative);
    CHECK(result.block_m == 0.5 && result.block_height_m == -0.25);
}

// The trailing partial cell never adds FREE; a negative crossing in it becomes the boundary at its
// near edge, and tail values outside it are invalid.
void test_tail_and_partial_cell() {
    Fixture fixture;
    fixture.input.cell_count = 2;
    fixture.input.range_max_m = 1.125;
    expect_stop(run(fixture), StopReason::Range, 2, 1.0);
    fixture.input.cell_count = 3;
    CHECK(!run(fixture).valid);
    fixture.input.cell_count = 2;
    fixture.input.tail_negative_m = 1.0;
    auto result = run(fixture);
    expect_stop(result, StopReason::Boundary, 1, 0.75);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 1.0 && !result.block_height_m);
    fixture.input.tail_negative_height_m = -0.25;
    CHECK(run(fixture).block_height_m == -0.25);
    // A nearer negative cell is the boundary; the deeper height measured in the tail still counts,
    // and without it the cell's own height is reported.
    fixture.cells[1].below = true;
    fixture.cells[1].below_height_m = -0.0625;
    result = run(fixture);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.75 && result.block_height_m == -0.25);
    fixture.input.tail_negative_height_m.reset();
    CHECK(run(fixture).block_height_m == -0.0625);
    fixture.cells[1] = fixture.cells[0];
    fixture.input.tail_negative_height_m = -0.25;
    // A boundary before the grid without a height of its own takes the one measured in the tail.
    fixture.input.before_grid_negative_m = 0.25;
    result = run(fixture);
    expect_stop(result, StopReason::Boundary, 0, std::nullopt);
    CHECK(result.boundary == BoundaryKind::Negative && result.block_m == 0.25 && result.block_height_m == -0.25);
    fixture.input.before_grid_negative_m.reset();
    fixture.input.positive_m = 1.0;
    fixture.input.positive_height_m = 0.25;
    CHECK(run(fixture).boundary == BoundaryKind::Negative);
    fixture.input.tail_negative_m = 1.0625;
    CHECK(run(fixture).boundary == BoundaryKind::Positive);
    fixture.input.positive_m.reset();
    fixture.input.positive_height_m.reset();
    result = run(fixture);
    expect_stop(result, StopReason::Range, 2, 1.0);
    CHECK(result.block_m == 1.0625);
    fixture.input.tail_negative_m = 0.875;
    CHECK(!run(fixture).valid);
    fixture.input.tail_negative_m = 1.25;
    CHECK(!run(fixture).valid);
    fixture.input.cell_count = 0;
    fixture.input.tail_negative_m = 0.5;
    expect_stop(run(fixture), StopReason::Boundary, 0, std::nullopt);
    fixture.input.has_samples = false;
    CHECK(!run(fixture).valid);
}

// With eroded T present FREE stops at the first non-passable cell; slope and roughness stop at the
// first cell over their limits (equal passes).
void test_slope_roughness_and_t() {
    Fixture fixture;
    fixture.cells[0].t_ok = false;
    expect_stop(run(fixture), StopReason::Range, 8, 2.5);
    fixture.input.has_eroded_t = true;
    expect_stop(run(fixture), StopReason::Traversability, 0, 0.5);
    fixture.cells[0].t_ok = true;
    fixture.cells[1].t_ok = false;
    expect_stop(run(fixture), StopReason::Traversability, 1, 0.75);

    Fixture sloped;
    sloped.input.cell_count = 3;
    sloped.options.slope_max_rad = std::atan(0.5);
    const double limit = std::tan(sloped.options.slope_max_rad);
    sloped.cells[1].mean_height_m = sloped.options.dr_m * limit;
    sloped.cells[2].mean_height_m = sloped.cells[1].mean_height_m;
    auto result = run(sloped);
    expect_stop(result, StopReason::Range, 3, 1.25);
    CHECK(result.slope_rad == std::atan(limit));
    sloped.cells[1].mean_height_m = std::nextafter(sloped.cells[1].mean_height_m, 1.0);
    expect_stop(run(sloped), StopReason::Slope, 1, 0.75);

    sloped.options.slope_max_rad = std::atan(1.0);
    sloped.cells[1].mean_height_m = 0.0625;
    sloped.cells[2].mean_height_m = 1.0;
    result = run(sloped);
    expect_stop(result, StopReason::Slope, 2, 1.0);
    CHECK(result.slope_rad == std::atan(0.25));
    sloped.cells[2].mean_height_m = 0.1875;
    sloped.cells[2].m2_height_m2 = 1;
    result = run(sloped);
    expect_stop(result, StopReason::Roughness, 2, 1.0);
    CHECK(result.slope_rad == std::atan(0.25));

    Fixture rough;
    rough.input.cell_count = 1;
    rough.cells[0].m2_height_m2 = 0.125;
    expect_stop(run(rough), StopReason::Range, 1, 0.75);
    rough.cells[0].m2_height_m2 = std::nextafter(0.125, 1.0);
    expect_stop(run(rough), StopReason::Roughness, 0, 0.5);
    rough.options.sigma_max_m = 0;
    rough.cells[0].m2_height_m2 = 0;
    rough.options.slope_max_rad = 0;
    expect_stop(run(rough), StopReason::Range, 1, 0.75);
    rough.options.sigma_max_m = std::numeric_limits<double>::max();
    rough.cells[0].m2_height_m2 = std::numeric_limits<double>::max();
    expect_stop(run(rough), StopReason::Range, 1, 0.75);
}

// Non-finite or out-of-range options and inputs, and a malformed cell anywhere (even behind an
// early stop), make the whole result invalid.
void test_invalid_inputs() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const std::array<double, 3> bad{{nan, infinity, -infinity}};
    const std::array<double GridOptions::*, 4> fields{{&GridOptions::dr_m, &GridOptions::cover_min,
        &GridOptions::slope_max_rad, &GridOptions::sigma_max_m}};
    for (const auto field : fields) {
        for (const auto number : bad) {
            Fixture fixture;
            fixture.options.*field = number;
            CHECK(!valid_grid_options(fixture.options));
            CHECK(!run(fixture).valid);
        }
    }
    Fixture fixture;
    auto options = fixture.options;
    options.dr_m = 0; CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.dr_m = -0.25; CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.cover_min = 0; CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.cover_min = std::nextafter(1.0, 2.0); CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.slope_max_rad = std::acos(-1.0) / 2; CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.slope_max_rad = -0.125; CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.sigma_max_m = -0.125; CHECK(!run(fixture.input, options).valid);
    options = fixture.options; options.dr_m = std::numeric_limits<double>::denorm_min();
    CHECK(!run(fixture.input, options).valid);
    auto input = fixture.input;
    input.cells = nullptr; CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.cell_count = kMaxRadialCells + 1; CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.blind_m = -0.125; CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.blind_m = nan; CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.range_max_m = input.blind_m; CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.range_max_m = std::nextafter(6.0, 7.0); CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.range_max_m = infinity; CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.positive_m = 1; CHECK(!run(input, fixture.options).valid);
    input.positive_height_m = 0; CHECK(!run(input, fixture.options).valid);
    input.positive_height_m = nan; CHECK(!run(input, fixture.options).valid);
    input.positive_height_m = 0.25; input.positive_m = -0.125; CHECK(!run(input, fixture.options).valid);
    input.positive_m = 4; CHECK(!run(input, fixture.options).valid);
    input.positive_m = nan; CHECK(!run(input, fixture.options).valid);
    input.positive_m.reset(); CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.before_grid_negative_m = 0.75; CHECK(!run(input, fixture.options).valid);
    input.before_grid_negative_m = -0.125; CHECK(!run(input, fixture.options).valid);
    input.before_grid_negative_m = nan; CHECK(!run(input, fixture.options).valid);
    input.before_grid_negative_m = 0.25; input.before_grid_negative_height_m = 0.125;
    CHECK(!run(input, fixture.options).valid);
    input.before_grid_negative_height_m = nan; CHECK(!run(input, fixture.options).valid);
    input.before_grid_negative_m.reset(); CHECK(!run(input, fixture.options).valid);
    input = fixture.input; input.tail_negative_m = nan; CHECK(!run(input, fixture.options).valid);
    input.tail_negative_m = 2.5; input.tail_negative_height_m = 0.125; CHECK(!run(input, fixture.options).valid);
    input.tail_negative_height_m = nan; CHECK(!run(input, fixture.options).valid);
    input.tail_negative_m.reset(); CHECK(!run(input, fixture.options).valid);

    // Late malformed cells invalidate all output even behind an early stop.
    fixture.input.positive_m = 0.25;
    fixture.input.positive_height_m = 0.5;
    fixture.cells[7].mean_height_m = nan; CHECK(!run(fixture).valid);
    fixture.cells[7].mean_height_m = 0;
    fixture.cells[7].m2_height_m2 = -0.25; CHECK(!run(fixture).valid);
    fixture.cells[7].m2_height_m2 = infinity; CHECK(!run(fixture).valid);
    fixture.cells[7].m2_height_m2 = 0;
    fixture.cells[7].covered_ground = 6; CHECK(!run(fixture).valid);
    fixture.cells[7].covered_ground = 4; fixture.cells[7].ground = 3; CHECK(!run(fixture).valid);
    fixture.cells[7].ground = 8; fixture.cells[7].below_height_m = -0.125; CHECK(!run(fixture).valid);
    fixture.cells[7].below = true; fixture.cells[7].below_height_m = 0.125; CHECK(!run(fixture).valid);
    fixture.cells[7].below_height_m = nan; CHECK(!run(fixture).valid);
}

// Cell index and full-cell count use half-open edges at origin + k*dr and match a linear oracle
// at, just below and just above every edge; invalid arguments give -1 or 0.
void test_actual_edge_helpers() {
    constexpr double blind = 0.6, dr = 0.25;
    const double edge = blind + 5.0 * dr;
    CHECK(index(std::nextafter(edge, 0.0), blind, dr, 8) == 4);
    CHECK(index(edge, blind, dr, 8) == 5);
    CHECK(index(std::nextafter(edge, 3.0), blind, dr, 8) == 5);
    CHECK(count(0.1, 0.35, 0.25, 8) == 1);
    CHECK(count(0.1, std::nextafter(0.35, 0.0), 0.25, 8) == 0);
    CHECK(count(0.5, 1.125, 0.25, 8) == 2);
    CHECK(count(0.5, 1.0, 0.25, 8) == 2);
    CHECK(count(0.5, 6.0, 0.25, 8) == 8);
    CHECK(count(0.0, 6.0, 0.015625, kMaxRadialCells) == kMaxRadialCells);
    CHECK(index(0.5, 0.5, 0.25, 2) == 0);
    CHECK(index(std::nextafter(0.5, 0.0), 0.5, 0.25, 2) == -1);
    CHECK(index(1.0, 0.5, 0.25, 2) == -1);
    CHECK(index(1.0625, 0.5, 0.25, 2) == -1);
    CHECK(index(std::nextafter(1.0, 0.0), 0.5, 0.25, 2) == 1);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(index(nan, 0.5, 0.25, 2) == -1);
    CHECK(index(1.0, nan, 0.25, 2) == -1);
    CHECK(index(1.0, 0.5, nan, 2) == -1);
    CHECK(index(inf, 0.5, 0.25, 2) == -1);
    CHECK(index(1.0, 0.5, 0, 2) == -1);
    CHECK(index(1.0, -0.5, 0.25, 2) == -1);
    CHECK(index(1.0, 0.5, -0.25, 2) == -1);
    CHECK(index(1.0, 0.5, 0.25, 0) == -1);
    CHECK(index(1.0, 0.5, 0.25, kMaxRadialCells + 1) == -1);
    CHECK(count(nan, 3.0, 0.25, 8) == 0);
    CHECK(count(0.5, nan, 0.25, 8) == 0);
    CHECK(count(0.5, 3.0, nan, 8) == 0);
    CHECK(count(0.5, inf, 0.25, 8) == 0);
    CHECK(count(0.5, 3.0, 0, 8) == 0);
    CHECK(count(0.5, 0.5, 0.25, 8) == 0);
    CHECK(count(0.5, 6.25, 0.25, 8) == 0);
    CHECK(count(0.5, 3.0, 0.25, 0) == 0);
    CHECK(count(0.5, 3.0, 0.25, kMaxRadialCells + 1) == 0);
    CHECK(count(0.5, 3.0, std::numeric_limits<double>::denorm_min(), 8) == 0);

    // Independent linear oracle uses explicitly calculated half-open edges.
    // It never uses the quotient or the implementation's binary search.
    for (const double origin : std::array<double, 4>{{0.0, 0.1, 0.6, 0.7}}) {
        for (const double step : std::array<double, 4>{{0.015625, 0.1, 0.2, 0.25}}) {
            constexpr std::size_t cells = 16;
            for (std::size_t boundary = 0; boundary <= cells; ++boundary) {
                const double actual_edge = origin + static_cast<double>(boundary) * step;
                for (const double sample : std::array<double, 3>{{std::nextafter(actual_edge, -inf),
                                                                actual_edge, std::nextafter(actual_edge, inf)}}) {
                    int expected = -1;
                    for (std::size_t k = 0; k < cells; ++k) {
                        const double near = origin + static_cast<double>(k) * step;
                        const double far = origin + static_cast<double>(k + 1) * step;
                        if (sample >= near && sample < far) expected = static_cast<int>(k);
                    }
                    CHECK(index(sample, origin, step, cells) == expected);
                    std::size_t expected_count = 0;
                    if (sample > origin && sample <= 6.0) {
                        for (std::size_t k = 0; k < cells; ++k) {
                            if (origin + static_cast<double>(k + 1) * step <= sample) ++expected_count;
                        }
                    }
                    CHECK(count(origin, sample, step, cells) == expected_count);
                }
            }
        }
    }
}

std::uint64_t random_word(std::uint64_t& state) {
    state ^= state << 13U;
    state ^= state >> 7U;
    state ^= state << 17U;
    return state;
}

// 1000 random walks are all valid and repeat exactly, and the height of the boundary equals a
// reference: the positive height for a positive boundary; for a negative one the lowest of all
// measured below-plane heights of the bin (before the grid, every cell, the tail), none when
// there is none. The heights come from a second generator, so the walks are the ones the first
// generator has always produced.
void test_bounded_random_properties() {
    std::uint64_t state = UINT64_C(0x316c47b82de569af);
    std::uint64_t height_state = UINT64_C(0x9e3779b97f4a7c15);
    std::size_t negative_with_height = 0, negative_without_height = 0, height_not_in_boundary_cell = 0;
    for (std::size_t trial = 0; trial < 1000; ++trial) {
        Fixture fixture;
        fixture.input.cell_count = random_word(state) % 9;
        fixture.input.has_eroded_t = (random_word(state) & 1U) != 0;
        for (std::size_t i = 0; i < fixture.input.cell_count; ++i) {
            auto& cell = fixture.cells[i];
            cell.expected = random_word(state) % 10;
            cell.ground = 12;
            cell.covered_ground = random_word(state) % (cell.expected + 1);
            cell.mean_height_m = static_cast<double>(random_word(state) % 5) * 0.03125;
            cell.m2_height_m2 = static_cast<double>(random_word(state) % 5) * 0.03125;
            cell.t_ok = (random_word(state) % 4) != 0;
            cell.pierce = (random_word(state) % 11) == 0;
            cell.below = (random_word(state) % 13) == 0;
            if (cell.below) cell.below_height_m = -0.03125 * static_cast<double>(1 + random_word(height_state) % 8);
        }
        if ((random_word(state) & 1U) != 0) {
            fixture.input.positive_m = static_cast<double>(random_word(state) % 25) * 0.125;
            fixture.input.positive_height_m = 0.25;
        }
        if ((random_word(state) % 9) == 0) {
            fixture.input.before_grid_negative_m = static_cast<double>(random_word(state) % 5) * 0.125;
            if ((random_word(height_state) & 1U) != 0)
                fixture.input.before_grid_negative_height_m = -0.03125 * static_cast<double>(1 + random_word(height_state) % 8);
        }
        if ((random_word(state) % 7) == 0) {
            fixture.input.tail_negative_m = fixture.input.blind_m +
                static_cast<double>(fixture.input.cell_count) * fixture.options.dr_m;
            if ((random_word(height_state) & 1U) != 0)
                fixture.input.tail_negative_height_m = -0.03125 * static_cast<double>(1 + random_word(height_state) % 8);
        }
        const auto result = run(fixture);
        CHECK(result.valid);
        std::optional<double> lowest = fixture.input.before_grid_negative_height_m;
        const auto take = [&lowest](const std::optional<double>& height) {
            if (height && (!lowest || *height < *lowest)) lowest = height;
        };
        for (std::size_t i = 0; i < fixture.input.cell_count; ++i) take(fixture.cells[i].below_height_m);
        take(fixture.input.tail_negative_height_m);
        if (result.boundary == BoundaryKind::Negative) {
            CHECK(result.block_height_m == lowest);
            ++(lowest ? negative_with_height : negative_without_height);
            // How often the old rule (the boundary cell's own height) would have differed.
            const int cell = index(*result.block_m, fixture.input.blind_m, fixture.options.dr_m, fixture.input.cell_count);
            const std::optional<double> own = cell >= 0 && *result.block_m >= fixture.input.blind_m
                ? fixture.cells[static_cast<std::size_t>(cell)].below_height_m : std::optional<double>{};
            if (lowest && own != lowest) ++height_not_in_boundary_cell;
        } else if (result.boundary == BoundaryKind::Positive) {
            CHECK(result.block_height_m == fixture.input.positive_height_m);
        } else {
            CHECK(!result.block_height_m);
        }
        const auto repeat = run(fixture);
        CHECK(result.stop == repeat.stop && result.boundary == repeat.boundary);
        CHECK(result.free_end_m == repeat.free_end_m && result.block_m == repeat.block_m);
        CHECK(result.block_height_m == repeat.block_height_m && result.slope_rad == repeat.slope_rad);
        CHECK(result.passed_cells == repeat.passed_cells && result.t_backed_prefix == repeat.t_backed_prefix);
    }
    // The reference was exercised in all three ways, not vacuously.
    CHECK(negative_with_height >= 50 && negative_without_height >= 20 && height_not_in_boundary_cell >= 20);
}
}  // namespace

int main() {
    test_empty_and_range();
    test_coverage_and_continuity();
    test_boundaries_and_heights();
    test_tail_and_partial_cell();
    test_slope_roughness_and_t();
    test_invalid_inputs();
    test_actual_edge_helpers();
    test_bounded_random_properties();
    if (failures != 0) {
        std::fprintf(stderr, "grid traversal tests: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("grid traversal tests: PASS (%zu walk calls, %zu helper calls; "
                "zero ordinary/aligned C++ new allocations; "
                "synthetic continuous-prefix and boundary gates only)\n", checked_walk_calls, checked_helper_calls);
    return EXIT_SUCCESS;
}
