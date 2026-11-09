/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: normal_grid.hpp
 * Brief: Normal-fit-only cell evidence and continuous geometry prefix.
 *
 * Description:
 * Not a wire Profile, calibration certificate, Q4 fallback or navigation authorization.
 */

#pragma once

#include "modules/native_geometry.hpp"
#include <cstddef>
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace perception::geometry {
inline constexpr std::size_t kMaxRadialCells = 256;
inline constexpr std::size_t kMaxGridCells = 65536;

struct GridOptions {
    double dr_m = 0, cover_min = 0, slope_max_rad = 0, sigma_max_m = 0;
    // Coverage tickets (2026-10-27, runtime switch coverage_tickets): false = the static prior-plane
    // tickets of the setup (the original behaviour); true = each frame's tickets from its own fitted
    // plane, so a ground that is tilted or shifted against the calibration prior (0.67 degrees on .26
    // outdoors) no longer moves every far ray out of its expected cell. A ray is still redeemed only by
    // its own Ground return in its own cell (no borrowing). Bin expected_rays (conf) stay static.
    bool fitted_tickets = false;
};
bool valid_grid_options(const GridOptions&) noexcept;
// IEEE operations use the SAME explicit edge formula as traversal, not an
// epsilon or uncorrected floor division. Invalid inputs yield -1 / 0.
int full_cell_index(double range_m, double blind_m, double dr_m, std::size_t count) noexcept;
std::size_t full_cell_count(double blind_m, double range_max_m, double dr_m, std::size_t capacity) noexcept;

struct CellEvidence {
    std::size_t expected = 0, ground = 0, covered_ground = 0;
    // All actual Ground samples, independent of expectation matching.
    // Welford M2 = sum of squared deviations; population variance = M2/ground.
    double mean_height_m = 0, m2_height_m2 = 0;
    bool pierce = false, below = false;
    // Lowest height among the cell's below-plane returns.
    std::optional<double> below_height_m;
    // Only for an independently prepared, fresh AND ground-eroded T table.
    // NormalGrid does not supply that table; its actual path has no T.
    bool t_ok = false;
};
enum class BoundaryKind { None, Positive, Negative };
enum class StopReason { InvalidInput, NoSamples, NoGroundReference, Boundary,
                        Coverage, Traversability, Slope, Roughness, Range };
struct BinWalkInput {
    const CellEvidence* cells = nullptr;
    std::size_t cell_count = 0;
    double blind_m = 0, range_max_m = 0;
    bool has_samples = false, has_eroded_t = false;
    std::optional<double> positive_m, positive_height_m;
    // Negative evidence before the first full cell; never expanded into FREE.
    // Distance of the nearest candidate; height: the lowest measured among all of them.
    std::optional<double> before_grid_negative_m, before_grid_negative_height_m;
    // Trailing partial cell supplies an independent negative boundary at its
    // near edge, but NEVER supplies FREE. Can be present when cell_count==0.
    // Height: the lowest measured in it.
    std::optional<double> tail_negative_m, tail_negative_height_m;
};
struct BinWalkResult {
    bool valid = false;
    StopReason stop = StopReason::InvalidInput;
    BoundaryKind boundary = BoundaryKind::None;
    std::optional<double> free_end_m, block_m, block_height_m, slope_rad;
    std::size_t passed_cells = 0;
    bool t_backed_prefix = false;
};

// Cells are uniform [blind+k*dr, blind+(k+1)*dr); no trailing partial cell.
// Negative cell boundary is its near edge. Select closest positive/negative
// BEFORE walking (negative wins exact ties), then stop on far >= boundary.
// A positive boundary carries its own height. A negative boundary carries the
// lowest below-plane height measured anywhere in the bin (before the grid, in
// any cell, in the tail), also when it was located by pierce alone; none when
// nothing was measured below the plane (19 section 3.2 / 3.4).
// This strict end guard preserves an UNKNOWN gap; no d_free=d_block clamp.
// coverage uses covered_ground: one ticket per expected source ray whose
// actual Ground point is in that SAME cell/bin. Unmatched Ground cannot fill
// another ray's hole. This conservative matching is an implementation choice.
// The tickets come from the prior plane, or with GridOptions::fitted_tickets
// from the current frame's fitted plane (NormalGrid, not this walk).
// No allocation, clock, scene interpretation or T freshness/erosion here.
BinWalkResult walk_normal_bin(const BinWalkInput&, const GridOptions&) noexcept;

enum class GridStatus { NormalEvidence, NoGroundReference, GeometryUnavailable,
                        InactiveOrStale, InvalidEvidence };
struct GridResult {
    GridStatus status = GridStatus::GeometryUnavailable;
    native_depth::SourceId source;
    std::optional<double> blind_m;
    std::size_t cells_per_bin = 0, unmatched_ground = 0;
    // Diagnostic stage timestamps (CLOCK_MONOTONIC ns, -1 = not reached):
    // [0] entry [1] cleared [2] expectation tickets [3] evidence binning [4] bin walks.
    std::array<std::int64_t, 5> stage_ns{{-1, -1, -1, -1, -1}};
};
struct GridBinEvidence {
    std::size_t valid_returns = 0, valid_pierce_rays = 0, expected_rays = 0;
    BinWalkResult traversal;
};
// One Ground pixel of the current frame and the cell its ground statistics used
// (-1: outside the active cells).
struct GroundPixelCell {
    std::uint32_t pixel = 0;
    std::int32_t cell = -1;
};

class NormalGrid {
public:
    // geometry and its Mailbox must outlive this single-threaded consumer.
    // Projection/options identity is anchored to this exact immutable owner.
    NormalGrid(const NativeGeometry& geometry, const GridOptions& options);
    NormalGrid(const NormalGrid&) = delete;
    NormalGrid& operator=(const NormalGrid&) = delete;
    const GridResult& process(const native_depth::Mailbox&) noexcept;
    const GridResult& result() const noexcept { return result_; }
    const NativeGeometry& geometry() const noexcept { return geometry_; }
    const GridOptions& options() const noexcept { return options_; }
    const std::vector<GridBinEvidence>& bins() const noexcept { return bins_; }
    // Bin-major, padded to cell_capacity_per_bin(); only cells_per_bin are active.
    const std::vector<CellEvidence>& cells() const noexcept { return cells_; }
    std::size_t cell_capacity_per_bin() const noexcept { return capacity_per_bin_; }
    // Every Ground pixel of the current frame in pixel order, each with the cell
    // its ground statistics used (the same cell_of value). Empty after any failure.
    // T walks this list instead of re-scanning every pixel and re-deriving each
    // cell (optimization round 2, 2026-10-03): same pixels, same order, same cells.
    const std::vector<GroundPixelCell>& ground_cells() const noexcept { return ground_cells_; }
    std::size_t storage_bytes() const noexcept;
private:
    // Test-only access to cell_of, in_cell and the edge table (differential tests).
    friend struct NormalGridTestAccess;
    using ExpectedRay = GroundTicket;
    void clear() noexcept;
    void fail(GridStatus) noexcept;
    int cell_of(double range) const noexcept;
    // cell_of(range) == k for every double and every k. With valid edges and an active cell k it
    // only compares against that cell's own near/far edge; otherwise it asks cell_of.
    bool in_cell(double range, int k) const noexcept;
    // Per-frame far edges blind+(k+1)*dr with the exact full_cell_index arithmetic;
    // edges_valid_ only when strictly increasing (else the reference path is used).
    bool prepare_edges() noexcept;
    std::vector<double> far_edges_;
    bool edges_valid_ = false;
    // Setup-time expectation tickets sorted by range per bin (static prior ROI).
    std::vector<double> ticket_ranges_;
    std::vector<std::size_t> ticket_offsets_, expected_per_bin_;
    const NativeGeometry& geometry_;
    GridOptions options_;
    std::size_t capacity_per_bin_ = 0;
    std::vector<ExpectedRay> expectation_;
    // This frame's tickets on its fitted plane (fitted_tickets only; empty otherwise), pixel-indexed;
    // pixels outside the prior ROI keep {-1, 0}. With valid edges a ticket's range_m is the near edge
    // of its cell (or -1 outside every cell), which in_cell places in exactly that cell. The ROI
    // pixels in raster order and each one's prior ticket bin (the bin search's start) are fixed at
    // construction; the squared far edges are per frame.
    std::vector<ExpectedRay> frame_tickets_;
    std::vector<std::uint32_t> roi_pixels_;
    std::vector<int> roi_hint_bins_;
    std::vector<double> far_edges_sq_;
    std::vector<CellEvidence> cells_;
    std::vector<GridBinEvidence> bins_;
    std::vector<BinWalkInput> walk_;
    // Capacity reserved for every native pixel at construction (no per-frame allocation).
    std::vector<GroundPixelCell> ground_cells_;
    GridResult result_;
    native_depth::SourceId last_;
};
}  // namespace perception::geometry
