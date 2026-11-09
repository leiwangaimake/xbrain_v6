/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: normal_grid_parent.hpp
 * Brief: Declaration of the frozen parent NormalGrid, the oracle for the ticket-cell tests.
 *
 * Description:
 * Used only by test_normal_grid_ticket_cell (namespace perception::geometry::normal_grid_parent).
 * The class is NormalGrid as it was at commit 02d2665, before the expected ticket's cell was
 * decided with in_cell(): its process() looks the ticket's cell up with a second cell_of(). The
 * evidence types, full_cell_index, full_cell_count and walk_normal_bin are the production ones
 * (normal_grid.hpp, grid_traversal.cpp); only the class is copied.
 */

#pragma once

#include "modules/normal_grid.hpp"

namespace perception::geometry::normal_grid_parent {
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
    struct ExpectedRay { int bin = -1; double range_m = 0; };
    void clear() noexcept;
    void fail(GridStatus) noexcept;
    int cell_of(double range) const noexcept;
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
    std::vector<CellEvidence> cells_;
    std::vector<GridBinEvidence> bins_;
    std::vector<BinWalkInput> walk_;
    // Capacity reserved for every native pixel at construction (no per-frame allocation).
    std::vector<GroundPixelCell> ground_cells_;
    GridResult result_;
    native_depth::SourceId last_;
};
}  // namespace perception::geometry::normal_grid_parent
