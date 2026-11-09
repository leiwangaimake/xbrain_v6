/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: normal_grid.cpp
 * Brief: NormalGrid: per-bin radial cells, coverage tickets, negative evidence and bin walks.
 *
 * Description:
 * What problem this solves. Implements normal_grid.hpp, the second stage of the Profile fast line
 * (after native_geometry). Per frame with a fitted ground plane:
 *   1. blind range = max(blind_near_lo_m, nearest measured ground); the cells of every bin are
 *      uniform [blind + k*dr, blind + (k+1)*dr) up to range_max_m;
 *   2. expectation: every pixel whose ray meets the prior plane holds one "ticket" for the
 *      cell where ground should be seen; only that same pixel's actual Ground return in the
 *      same cell redeems it (coverage cannot be borrowed from a neighbour ray). With
 *      fitted_tickets (2026-10-27) the ticket's cell is where the ray meets the CURRENT fitted
 *      plane: against the static prior, a 0.67 degree tilt (.26 outdoors) moved the far rays'
 *      returns out of their cells and capped FREE near 2 m although the ground was fully seen;
 *   3. ground height statistics per cell, and negative evidence (below-plane returns, pierce
 *      candidates) placed at their cell, before the grid, or in the unusable tail;
 *   4. walk_normal_bin() per bin: FREE advances cell by cell until coverage, slope, roughness or a
 *      boundary stops it.
 * Without usable ground the frame is NoGroundReference: only measured positive blocks remain,
 * no FREE (published with FREE withdrawn, reply B6-2).
 * 2026-10-25 (G-P1 phase 2, notes/PROFILE_GP1_PHASE1_20261024.md section 10): in the evidence pass a
 * Ground pixel first tries the previous Ground pixel's cell through in_cell (which equals
 * cell_of(r) == k for every r and k) before the full cell_of search, and the option values, sizes,
 * array pointers and the unmatched count are locals (stores to the cells could alias the members, so
 * GCC reloaded them per pixel). Same values, bit for bit (tests/test_normal_grid_ticket_cell.cpp
 * against the frozen parent); evidence pass 2.21 -> 1.89 ms on 1.8.
 */
#include "modules/normal_grid.hpp"
#include <time.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace perception::geometry {
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
// Negative evidence of one zone (before the grid, or the trailing partial cell): the nearest
// candidate gives the distance, and the height is the lowest one measured among all candidates
// of the zone, so a nearer candidate without a measured height (pierce) never erases one
// (19 section 3.2: neg_h[i] is the per-bin minimum; walk_normal_bin joins the zones and cells).
void negative_candidate(std::optional<double>& distance, std::optional<double>& height,
                       double d, std::optional<double> h) {
    if (!distance || d < *distance) distance = d;
    if (h) lower(height, *h);
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
    // Fitted tickets: the per-frame pass walks only the prior-ROI pixels, starting each bin search
    // at the pixel's prior ticket bin.
    if (options.fitted_tickets) {
        frame_tickets_.resize(kGeometryPixels);
        far_edges_sq_.assign(capacity_per_bin_, 0.0);
        const auto& roi = geometry.prior_roi();
        for (std::size_t p = 0; p < roi.size(); ++p) {
            if (!roi[p]) continue;
            roi_pixels_.push_back(static_cast<std::uint32_t>(p));
            roi_hint_bins_.push_back(expectation_[p].bin);
        }
    }
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
// cell_of(r) == k without a second search (2026-10-11). The far edges are strictly increasing
// above blind (prepare_edges), so the first far edge above r is far_edges_[k] exactly when
// near_k <= r < far_k: the same stored doubles and the same closed/open sides as cell_of. Every
// other case (no valid edges, k not an active cell) is left to cell_of itself.
bool NormalGrid::in_cell(double r, int k) const noexcept {
    if (!edges_valid_ || !result_.blind_m || k < 0 ||
        static_cast<std::size_t>(k) >= result_.cells_per_bin) return cell_of(r) == k;
    const auto low = static_cast<std::size_t>(k);
    const double near = low == 0 ? *result_.blind_m : far_edges_[low - 1];
    return near <= r && r < far_edges_[low];
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
    // The tickets the evidence pass redeems: the static prior ones, or this frame's (fitted_tickets).
    const ExpectedRay* tickets_in_use = expectation_.data();
    if (options_.fitted_tickets) {
        // Fitted tickets (2026-10-27): where each prior-ROI ray meets THIS frame's fitted plane
        // (NativeGeometry::ticket_on, homogeneous (xh, yh) / dh), written and counted in one pass over
        // the ROI pixels. The prior ticket's bin only starts each bin search (the fitted plane is near
        // the prior; any start gives the same bin). The cell compares squares, near_k^2 dh^2 <=
        // xh^2 + yh^2 < far_k^2 dh^2, trying the previous ticket's cell first, so no ray needs its range;
        // the ticket then carries its cell's near edge. Without valid edges (cell_of's reference path)
        // the range itself is formed and decided by cell_of. A pixel without a ticket gets {-1, 0}.
        const NativeGeometry::TicketPlane on = geometry_.ticket_plane(*geometry_.result().fit.plane);
        ExpectedRay* const frame = frame_tickets_.data();
        CellEvidence* const cell_base = cells_.data();
        const std::size_t cap = capacity_per_bin_, roi_count = roi_pixels_.size();
        const auto count = static_cast<int>(result_.cells_per_bin);
        const bool squares = edges_valid_ && count > 0;
        const double blind = result_.blind_m.value_or(0.0), blind_sq = blind * blind;
        double* const far_sq = far_edges_sq_.data();
        const double* const far = far_edges_.data();
        const std::uint32_t* const pixels = roi_pixels_.data();
        const int* const hints = roi_hint_bins_.data();
        if (squares)
            for (int k = 0; k < count; ++k) far_sq[k] = far[k] * far[k];
        int cell_hint = -1;
        for (std::size_t j = 0; j < roi_count; ++j) {
            const std::uint32_t p = pixels[j];
            ExpectedRay ticket;
            double xh = 0, yh = 0, dh = 0;
            if (!geometry_.ticket_on(on, p, hints[j], ticket.bin, xh, yh, dh)) {
                frame[p] = ExpectedRay{};
                continue;
            }
            int k = -1;
            if (squares) {
                const double s = xh * xh + yh * yh, d2 = dh * dh;
                const auto inside = [&](int c) {
                    return (c == 0 ? blind_sq : far_sq[c - 1]) * d2 <= s && s < far_sq[c] * d2;
                };
                if (cell_hint >= 0 && inside(cell_hint)) {
                    k = cell_hint;
                } else if (blind_sq * d2 <= s && s < far_sq[count - 1] * d2) {
                    int low = 0, high = count - 1;  // the first far edge above the range
                    while (low < high) {
                        const int middle = low + (high - low) / 2;
                        if (s < far_sq[middle] * d2) high = middle;
                        else low = middle + 1;
                    }
                    k = low;
                }
                ticket.range_m = k < 0 ? -1.0 : (k == 0 ? blind : far[k - 1]);
            } else {
                ticket.range_m = std::sqrt(xh * xh + yh * yh) / dh;
                k = cell_of(ticket.range_m);
            }
            cell_hint = k;
            if (k >= 0) ++cell_base[static_cast<std::size_t>(ticket.bin) * cap + static_cast<std::size_t>(k)].expected;
            frame[p] = ticket;
        }
        tickets_in_use = frame;
    } else if (edges_valid_) {
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
    // G-P1 phase 2: the previous Ground pixel's cell (raster neighbours mostly share it) and locals
    // for what the loop reads; the unmatched count is added to the result after the loop (every
    // failure inside it goes through fail(), which zeroes that count anyway).
    int ground_hint = -1;
    const double range_lo = g.blind_near_lo_m, range_hi = g.range_max_m;
    const std::size_t n_bins = bins_.size(), cap = capacity_per_bin_;
    GridBinEvidence* const bins = bins_.data();
    CellEvidence* const cells = cells_.data();
    const ExpectedRay* const tickets = tickets_in_use;
    std::size_t unmatched = 0;
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
            if (e.bin < 0 || static_cast<std::size_t>(e.bin) >= n_bins ||
                !std::isfinite(e.range_m) || e.range_m < range_lo || e.range_m > range_hi ||
                !std::isfinite(e.height_m)) { fail(GridStatus::InvalidEvidence); return result_; }
            const auto b = static_cast<std::size_t>(e.bin);
            ++bins[b].valid_returns;
            if (e.classification == SampleClass::Ground) {
                // The cell is only needed for Ground (2026-10-06: it used to be looked up for every
                // return; cell_of has no side effects, so the results are the same).
                // in_cell(r, k) == (cell_of(r) == k), so the hint never changes k (2026-10-25).
                const int k = ground_hint >= 0 && in_cell(e.range_m, ground_hint) ? ground_hint : cell_of(e.range_m);
                ground_hint = k;
                // Within the capacity reserved at construction: never allocates.
                ground_cells_.push_back({static_cast<std::uint32_t>(p), static_cast<std::int32_t>(k)});
                if (k >= 0) {
                    // Welford mean/M2 of ground heights (roughness input), and the
                    // coverage ticket only if it is this pixel's own expected cell.
                    auto& cell = cells[b * cap + static_cast<std::size_t>(k)];
                    ++cell.ground;
                    const double delta = e.height_m - cell.mean_height_m;
                    cell.mean_height_m += delta / static_cast<double>(cell.ground);
                    cell.m2_height_m2 += delta * (e.height_m - cell.mean_height_m);
                    const auto& ticket = tickets[p];
                    if (ticket.bin == e.bin && in_cell(ticket.range_m, k)) ++cell.covered_ground;
                    else ++unmatched;
                }
            } else if (e.classification == SampleClass::BelowPlane) {
                add_negative(e.bin, e.range_m, false, e.height_m);
            }
        }
        if (e.pierce_range_m) {
            if (e.pierce_bin < 0 || static_cast<std::size_t>(e.pierce_bin) >= n_bins ||
                !std::isfinite(*e.pierce_range_m) || *e.pierce_range_m < range_lo ||
                *e.pierce_range_m > range_hi || e.classification == SampleClass::InvalidDepth ||
                e.classification == SampleClass::Unclassified) { fail(GridStatus::InvalidEvidence); return result_; }
            ++bins[static_cast<std::size_t>(e.pierce_bin)].valid_pierce_rays;
            add_negative(e.pierce_bin, *e.pierce_range_m, true, std::nullopt);
        }
    }
    result_.unmatched_ground += unmatched;
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
    return (expectation_.capacity() + frame_tickets_.capacity()) * sizeof(ExpectedRay) +
           roi_pixels_.capacity() * sizeof(std::uint32_t) + roi_hint_bins_.capacity() * sizeof(int) +
           far_edges_sq_.capacity() * sizeof(double) +
           cells_.capacity() * sizeof(CellEvidence) +
           bins_.capacity() * sizeof(GridBinEvidence) + walk_.capacity() * sizeof(BinWalkInput) +
           ground_cells_.capacity() * sizeof(GroundPixelCell);
}
}  // namespace perception::geometry
