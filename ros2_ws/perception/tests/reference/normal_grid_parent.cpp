/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: normal_grid_parent.cpp
 * Brief: Frozen copy of the parent NormalGrid, the oracle for the ticket-cell tests.
 *
 * Description:
 * Built only into test_normal_grid_ticket_cell (namespace perception::geometry::normal_grid_parent),
 * which compares every output of the production NormalGrid::process against it. Not production
 * code. The body below is src/modules/normal_grid.cpp of commit 02d2665 unchanged except for the
 * include and the namespace; do not edit it together with the production file.
 */
#include "normal_grid_parent.hpp"
#include <time.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace perception::geometry::normal_grid_parent {
namespace {
std::int64_t stage_clock_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
}  // namespace

namespace {
// Keep the smaller value.
void lower(std::optional<double>& value, double candidate) {
    if (!value || candidate < *value) value = candidate;
}
// Keep the nearest negative candidate (and at equal distance the lowest height).
void negative_candidate(std::optional<double>& distance, std::optional<double>& height,
                       double d, std::optional<double> h) {
    if (!distance || d < *distance) { distance = d; height = h; }
    else if (d == *distance && h) lower(height, *h);
}
// A valid return inside the domain (not invalid, outside or unclassified).
bool classified_return(SampleClass c) {
    return c == SampleClass::Ground || c == SampleClass::Obstacle ||
           c == SampleClass::Overhead || c == SampleClass::BelowPlane;
}
}  // namespace

NormalGrid::NormalGrid(const NativeGeometry& geometry, const GridOptions& options)
    : geometry_(geometry), options_(options) {
    if (!valid_grid_options(options)) throw std::invalid_argument("normal grid needs explicit valid thresholds");
    // Storage for the worst case (blind = 0): ceil(range / dr) cells per bin.
    const auto& g = geometry.options();
    const double capacity = std::ceil(g.range_max_m / options.dr_m);
    if (!std::isfinite(capacity) || capacity < 1 || capacity > kMaxRadialCells)
        throw std::invalid_argument("normal grid radial storage bound");
    capacity_per_bin_ = static_cast<std::size_t>(capacity);
    if (g.n_bins > kMaxGridCells / capacity_per_bin_)
        throw std::invalid_argument("normal grid total cell storage bound");
    expectation_.resize(kGeometryPixels);
    cells_.resize(g.n_bins * capacity_per_bin_);
    bins_.resize(g.n_bins);
    walk_.resize(g.n_bins);
    ground_cells_.reserve(kGeometryPixels);
    // Static expectation tickets from the prior plane, also kept per bin sorted
    // by range so each frame can count them per cell with binary searches.
    for (std::size_t p = 0; p < expectation_.size(); ++p) {
        int bin = -1;
        double r = 0;
        if (geometry.prior_ground_projection(p, bin, r)) expectation_[p] = {bin, r};
    }
    expected_per_bin_.assign(g.n_bins, 0);
    for (const auto& e : expectation_)
        if (e.bin >= 0) ++expected_per_bin_[static_cast<std::size_t>(e.bin)];
    ticket_offsets_.assign(g.n_bins + 1, 0);
    for (std::size_t b = 0; b < g.n_bins; ++b) ticket_offsets_[b + 1] = ticket_offsets_[b] + expected_per_bin_[b];
    ticket_ranges_.assign(ticket_offsets_.back(), 0.0);
    std::vector<std::size_t> fill(ticket_offsets_.begin(), ticket_offsets_.end() - 1);
    for (const auto& e : expectation_)
        if (e.bin >= 0) ticket_ranges_[fill[static_cast<std::size_t>(e.bin)]++] = e.range_m;
    for (std::size_t b = 0; b < g.n_bins; ++b)
        std::sort(ticket_ranges_.begin() + static_cast<long>(ticket_offsets_[b]),
                  ticket_ranges_.begin() + static_cast<long>(ticket_offsets_[b + 1]));
    far_edges_.assign(capacity_per_bin_, 0.0);
}
bool NormalGrid::prepare_edges() noexcept {
    edges_valid_ = false;
    if (!result_.blind_m) return false;
    const double blind = *result_.blind_m, dr = options_.dr_m;
    const auto count = result_.cells_per_bin;
    // Same validity guard as full_cell_index.
    if (!std::isfinite(blind) || blind < 0 || !std::isfinite(dr) || dr <= 0.0 || count == 0 ||
        count > kMaxRadialCells || count > far_edges_.size()) return false;
    const double end = blind + static_cast<double>(count) * dr;
    if (!std::isfinite(end) || blind + dr <= blind) return false;
    double previous = blind;
    for (std::size_t k = 0; k < count; ++k) {
        far_edges_[k] = blind + static_cast<double>(k + 1) * dr;
        if (!(far_edges_[k] > previous)) return false;
        previous = far_edges_[k];
    }
    edges_valid_ = true;
    return true;
}
void NormalGrid::clear() noexcept {
    edges_valid_ = false;
    std::fill(cells_.begin(), cells_.end(), CellEvidence{});
    std::fill(bins_.begin(), bins_.end(), GridBinEvidence{});
    std::fill(walk_.begin(), walk_.end(), BinWalkInput{});
    ground_cells_.clear();
}
// Any failure leaves no cells, no blind range and no walk result.
void NormalGrid::fail(GridStatus status) noexcept {
    clear();
    result_.blind_m.reset();
    result_.cells_per_bin = 0;
    result_.unmatched_ground = 0;
    result_.status = status;
}
int NormalGrid::cell_of(double r) const noexcept {
    if (!result_.blind_m) return -1;
    if (!edges_valid_) return full_cell_index(r, *result_.blind_m, options_.dr_m, result_.cells_per_bin);
    // Identical comparisons to full_cell_index: first far edge strictly greater than r,
    // then near <= r < far with near = blind + low*dr (== far_edges_[low-1]).
    const double blind = *result_.blind_m;
    const auto count = result_.cells_per_bin;
    if (!std::isfinite(r) || r < blind || r >= far_edges_[count - 1]) return -1;
    const auto it = std::upper_bound(far_edges_.begin(), far_edges_.begin() + static_cast<long>(count), r);
    const auto low = static_cast<std::size_t>(it - far_edges_.begin());
    if (low == count) return -1;
    const double near = low == 0 ? blind : far_edges_[low - 1];
    return near <= r && r < far_edges_[low] ? static_cast<int>(low) : -1;
}

const GridResult& NormalGrid::process(const native_depth::Mailbox& mailbox) noexcept {
    result_ = {};
    result_.stage_ns[0] = stage_clock_ns();
    result_.source = geometry_.result().source;
    clear();
    result_.stage_ns[1] = stage_clock_ns();
    // Same identity and order rules as the geometry it reads.
    const auto active = mailbox.active_stream();
    const auto& source = result_.source;
    if (!geometry_.bound_to(mailbox) || active == 0 || source.stream_id != active || source.stream_id < last_.stream_id ||
        source.sequence == 0 || (source.stream_id == last_.stream_id && source.sequence <= last_.sequence)) {
        fail(GridStatus::InactiveOrStale); return result_;
    }
    last_ = source;
    if (!geometry_.result().has_evidence() || !geometry_.result().fit.plane) {
        fail(GridStatus::GeometryUnavailable); return result_;
    }
    const auto& g = geometry_.options();
    const auto& evidence = geometry_.pixels();
    const auto& native_bins = geometry_.bins();
    if (evidence.size() != kGeometryPixels || native_bins.size() != bins_.size()) {
        fail(GridStatus::InvalidEvidence); return result_;
    }
    // The blind range starts at the nearest measured ground (never below the
    // configured lower bound); no ground at all leaves it unknown.
    const auto nearest_ground = geometry_.result().nearest_ground_m;
    if (nearest_ground) {
        if (!std::isfinite(*nearest_ground) || *nearest_ground < 0 || *nearest_ground > g.range_max_m) {
            fail(GridStatus::InvalidEvidence); return result_;
        }
        result_.blind_m = std::max(g.blind_near_lo_m, *nearest_ground);
        result_.cells_per_bin = full_cell_count(*result_.blind_m, g.range_max_m,
                                             options_.dr_m, capacity_per_bin_);
    }
    const bool usable_ground = result_.blind_m && *result_.blind_m < g.range_max_m;
    (void)prepare_edges();
    // Per-bin walk inputs: this bin's cells and its nearest positive obstacle.
    for (std::size_t b = 0; b < bins_.size(); ++b) {
        auto& w = walk_[b];
        w.cells = cells_.data() + b * capacity_per_bin_;
        w.cell_count = result_.cells_per_bin;
        w.blind_m = result_.blind_m.value_or(0);  // Not used to synthesize FREE when absent.
        w.range_max_m = g.range_max_m;
        w.positive_m = native_bins[b].nearest_positive_m;
        w.positive_height_m = native_bins[b].positive_height_window_max_m;
        // No segmentation projection/freshness/erosion is provided by this
        // component. The real path therefore NEVER claims T-backed FREE.
        // (This walk never does; T restricts the prefix afterwards in NormalTraversability.)
        w.has_eroded_t = false;
    }
    for (std::size_t b = 0; b < bins_.size(); ++b) bins_[b].expected_rays = expected_per_bin_[b];
    if (edges_valid_) {
        // Tickets are static; per cell count = |{r : near_k <= r < far_k}|, identical to cell_of(r) == k.
        const double blind = *result_.blind_m;
        for (std::size_t b = 0; b < bins_.size(); ++b) {
            const auto first = ticket_ranges_.begin() + static_cast<long>(ticket_offsets_[b]);
            const auto last = ticket_ranges_.begin() + static_cast<long>(ticket_offsets_[b + 1]);
            auto lower = std::lower_bound(first, last, blind);
            for (std::size_t k = 0; k < result_.cells_per_bin; ++k) {
                const auto upper = std::lower_bound(lower, last, far_edges_[k]);
                cells_[b * capacity_per_bin_ + k].expected = static_cast<std::size_t>(upper - lower);
                lower = upper;
            }
        }
    } else {
        for (std::size_t p = 0; p < expectation_.size(); ++p) {
            const auto& expected = expectation_[p];
            if (expected.bin < 0) continue;
            const int k = cell_of(expected.range_m);
            if (k >= 0) ++cells_[static_cast<std::size_t>(expected.bin) * capacity_per_bin_ + static_cast<std::size_t>(k)].expected;
        }
    }
    result_.stage_ns[2] = stage_clock_ns();
    // Each source pixel has at most one expectation ticket, INCLUDING invalid
    // depth. Only that same pixel's actual Ground in the same bin/cell can
    // redeem it. All Ground remains in the separate geometric statistics.
    for (std::size_t p = 0; p < evidence.size(); ++p) {
        const auto& e = evidence[p];
        const auto add_negative = [&](int bin, double r, bool pierce, std::optional<double> height) {
            if (!usable_ground) return;
            auto& w = walk_[static_cast<std::size_t>(bin)];
            if (r < *result_.blind_m) {
                negative_candidate(w.before_grid_negative_m, w.before_grid_negative_height_m, r, height);
                return;
            }
            const int k = cell_of(r);
            if (k < 0) {
                const double near = *result_.blind_m + static_cast<double>(result_.cells_per_bin) * options_.dr_m;
                negative_candidate(w.tail_negative_m, w.tail_negative_height_m, near, height);
                return;
            }
            auto& cell = cells_[static_cast<std::size_t>(bin) * capacity_per_bin_ + static_cast<std::size_t>(k)];
            if (pierce) cell.pierce = true;
            else cell.below = true;
            if (height) lower(cell.below_height_m, *height);
        };
        if (classified_return(e.classification)) {
            if (e.bin < 0 || static_cast<std::size_t>(e.bin) >= bins_.size() ||
                !std::isfinite(e.range_m) || e.range_m < g.blind_near_lo_m || e.range_m > g.range_max_m ||
                !std::isfinite(e.height_m)) { fail(GridStatus::InvalidEvidence); return result_; }
            const auto b = static_cast<std::size_t>(e.bin);
            ++bins_[b].valid_returns;
            if (e.classification == SampleClass::Ground) {
                // The cell is only needed for Ground (2026-10-06: it used to be looked up for every
                // return; cell_of has no side effects, so the results are the same).
                const int k = cell_of(e.range_m);
                // Within the capacity reserved at construction: never allocates.
                ground_cells_.push_back({static_cast<std::uint32_t>(p), static_cast<std::int32_t>(k)});
                if (k >= 0) {
                    // Welford mean/M2 of ground heights (roughness input), and the
                    // coverage ticket only if it is this pixel's own expected cell.
                    auto& cell = cells_[b * capacity_per_bin_ + static_cast<std::size_t>(k)];
                    ++cell.ground;
                    const double delta = e.height_m - cell.mean_height_m;
                    cell.mean_height_m += delta / static_cast<double>(cell.ground);
                    cell.m2_height_m2 += delta * (e.height_m - cell.mean_height_m);
                    const auto& ticket = expectation_[p];
                    if (ticket.bin == e.bin && cell_of(ticket.range_m) == k) ++cell.covered_ground;
                    else ++result_.unmatched_ground;
                }
            } else if (e.classification == SampleClass::BelowPlane) {
                add_negative(e.bin, e.range_m, false, e.height_m);
            }
        }
        if (e.pierce_range_m) {
            if (e.pierce_bin < 0 || static_cast<std::size_t>(e.pierce_bin) >= bins_.size() ||
                !std::isfinite(*e.pierce_range_m) || *e.pierce_range_m < g.blind_near_lo_m ||
                *e.pierce_range_m > g.range_max_m || e.classification == SampleClass::InvalidDepth ||
                e.classification == SampleClass::Unclassified) { fail(GridStatus::InvalidEvidence); return result_; }
            ++bins_[static_cast<std::size_t>(e.pierce_bin)].valid_pierce_rays;
            add_negative(e.pierce_bin, *e.pierce_range_m, true, std::nullopt);
        }
    }
    result_.stage_ns[3] = stage_clock_ns();
    for (std::size_t b = 0; b < bins_.size(); ++b) {
        auto& bin = bins_[b];
        auto& w = walk_[b];
        w.has_samples = bin.valid_returns > 0 || bin.valid_pierce_rays > 0;
        // No ground reference: no walk and no FREE; only the measured positive
        // block is kept.
        if (!usable_ground) {
            auto& r = bin.traversal;
            r.valid = true;
            r.stop = w.has_samples ? StopReason::NoGroundReference : StopReason::NoSamples;
            r.block_m = w.positive_m;
            r.block_height_m = w.positive_height_m;
            r.boundary = r.block_m ? BoundaryKind::Positive : BoundaryKind::None;
        } else {
            bin.traversal = walk_normal_bin(w, options_);
            if (!bin.traversal.valid) { fail(GridStatus::InvalidEvidence); return result_; }
        }
    }
    result_.stage_ns[4] = stage_clock_ns();
    if (mailbox.active_stream() != active) { fail(GridStatus::InactiveOrStale); return result_; }
    result_.status = usable_ground ? GridStatus::NormalEvidence : GridStatus::NoGroundReference;
    return result_;
}

std::size_t NormalGrid::storage_bytes() const noexcept {
    return expectation_.capacity() * sizeof(ExpectedRay) + cells_.capacity() * sizeof(CellEvidence) +
           bins_.capacity() * sizeof(GridBinEvidence) + walk_.capacity() * sizeof(BinWalkInput) +
           ground_cells_.capacity() * sizeof(GroundPixelCell);
}
}  // namespace perception::geometry::normal_grid_parent
