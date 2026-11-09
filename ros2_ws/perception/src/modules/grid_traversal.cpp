/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: grid_traversal.cpp
 * Brief: Cell index arithmetic and walk_normal_bin(): one bin's FREE prefix and boundary.
 *
 * Description:
 * What problem this solves. For one angular bin, decide how far the ground is verified free
 * (d_free) and where the first obstacle is (d_block). The rules (declared in normal_grid.hpp):
 *   - boundary: the nearest negative (pit/drop) or positive obstacle, the negative winning
 *     an exact tie; a boundary inside the blind range leaves no FREE at all;
 *   - FREE starts at the blind range and advances one whole cell at a time while the cell
 *     is well covered by its own expected rays, not too steep against the previous cell,
 *     and not too rough; it stops BEFORE the cell whose far edge reaches the boundary, so an
 *     unknown gap between FREE and the block is preserved (FREE is never clamped onto it);
 *   - the slope is reported only over at least two passed cells.
 * Every input is validated first, including cells the walk will never reach, so malformed
 * evidence can never produce a plausible-looking result.
 */
#include "modules/normal_grid.hpp"

#include <algorithm>
#include <cmath>

namespace perception::geometry {
namespace {
constexpr double kHalfPi = 1.570796326794896619231321691639751442;

bool finite_nonnegative(double value) noexcept {
    return std::isfinite(value) && value >= 0.0;
}

// A negative boundary's height is <= 0 (below the plane) or unknown.
bool valid_negative_height(const std::optional<double>& height) noexcept {
    return !height || (std::isfinite(*height) && *height <= 0.0);
}

// Validate the entire input before returning any evidence, including cells
// beyond a boundary or coverage stop that traversal will never visit.
bool valid_input(const BinWalkInput& input, const GridOptions& options) noexcept {
    if (!valid_grid_options(options) || input.cell_count > kMaxRadialCells ||
        (input.cell_count != 0 && input.cells == nullptr) ||
        !finite_nonnegative(input.blind_m) || !std::isfinite(input.range_max_m) ||
        input.range_max_m <= input.blind_m || input.range_max_m > 6.0) return false;

    if (input.positive_m) {
        if (!finite_nonnegative(*input.positive_m) || *input.positive_m > input.range_max_m ||
            !input.positive_height_m || !std::isfinite(*input.positive_height_m) ||
            *input.positive_height_m <= 0.0) return false;
    } else if (input.positive_height_m) {
        return false;
    }
    if (input.before_grid_negative_m) {
        if (!finite_nonnegative(*input.before_grid_negative_m) ||
            *input.before_grid_negative_m > input.blind_m ||
            !valid_negative_height(input.before_grid_negative_height_m)) return false;
    } else if (input.before_grid_negative_height_m) {
        return false;
    }
    if (input.tail_negative_m) {
        const double end = input.blind_m + static_cast<double>(input.cell_count) * options.dr_m;
        if (!std::isfinite(end) || !std::isfinite(*input.tail_negative_m) ||
            *input.tail_negative_m < end || *input.tail_negative_m > input.range_max_m ||
            !valid_negative_height(input.tail_negative_height_m)) return false;
    } else if (input.tail_negative_height_m) {
        return false;
    }
    if (!input.has_samples && (input.positive_m || input.before_grid_negative_m || input.tail_negative_m)) {
        return false;
    }

    for (std::size_t k = 0; k < input.cell_count; ++k) {
        const double near = input.blind_m + static_cast<double>(k) * options.dr_m;
        const double far = input.blind_m + static_cast<double>(k + 1) * options.dr_m;
        if (!std::isfinite(near) || !std::isfinite(far) || far <= near || far > input.range_max_m) {
            return false;
        }
        const CellEvidence& cell = input.cells[k];
        if (cell.expected > kGeometryPixels || cell.ground > kGeometryPixels ||
            cell.covered_ground > cell.ground || cell.covered_ground > cell.expected ||
            !std::isfinite(cell.mean_height_m) || !finite_nonnegative(cell.m2_height_m2) ||
            !valid_negative_height(cell.below_height_m) || (cell.below_height_m && !cell.below) ||
            (!input.has_samples && (cell.ground != 0 || cell.pierce || cell.below))) return false;
    }
    return true;
}

}  // namespace

// dr > 0, coverage fraction in (0, 1], slope limit below 90 degrees.
bool valid_grid_options(const GridOptions& options) noexcept {
    return std::isfinite(options.dr_m) && options.dr_m > 0.0 &&
           std::isfinite(options.cover_min) && options.cover_min > 0.0 && options.cover_min <= 1.0 &&
           std::isfinite(options.slope_max_rad) && options.slope_max_rad >= 0.0 &&
           options.slope_max_rad < kHalfPi && finite_nonnegative(options.sigma_max_m);
}

int full_cell_index(double range_m, double blind_m, double dr_m, std::size_t count) noexcept {
    if (!std::isfinite(range_m) || !finite_nonnegative(blind_m) ||
        !std::isfinite(dr_m) || dr_m <= 0.0 || count == 0 || count > kMaxRadialCells) return -1;
    const double end = blind_m + static_cast<double>(count) * dr_m;
    if (!std::isfinite(end) || range_m < blind_m || range_m >= end || blind_m + dr_m <= blind_m) return -1;
    // Find the first actual far edge strictly greater than the sample. Unlike
    // floor((r-blind)/dr), this cannot round a left neighbour into the next cell.
    std::size_t low = 0, high = count;
    while (low < high) {
        const std::size_t middle = low + (high - low) / 2;
        const double far = blind_m + static_cast<double>(middle + 1) * dr_m;
        if (range_m < far) high = middle;
        else low = middle + 1;
    }
    if (low == count) return -1;
    const double near = blind_m + static_cast<double>(low) * dr_m;
    const double far = blind_m + static_cast<double>(low + 1) * dr_m;
    return near <= range_m && range_m < far ? static_cast<int>(low) : -1;
}

// Number of WHOLE cells between blind and range_max (a trailing partial cell
// is not a cell: it can never contribute FREE).
std::size_t full_cell_count(double blind_m, double range_max_m, double dr_m,
                          std::size_t capacity) noexcept {
    if (!finite_nonnegative(blind_m) || !std::isfinite(range_max_m) || range_max_m <= blind_m ||
        range_max_m > 6.0 || !std::isfinite(dr_m) || dr_m <= 0.0 || capacity > kMaxRadialCells) return 0;
    std::size_t count = 0;
    for (; count < capacity; ++count) {
        const double near = blind_m + static_cast<double>(count) * dr_m;
        const double far = blind_m + static_cast<double>(count + 1) * dr_m;
        if (!std::isfinite(far) || far <= near) return 0;
        if (far > range_max_m) break;
    }
    return count;
}

BinWalkResult walk_normal_bin(const BinWalkInput& input, const GridOptions& options) noexcept {
    if (!valid_input(input, options)) return {};
    BinWalkResult result;
    result.valid = true;
    if (!input.has_samples) {
        result.stop = StopReason::NoSamples;
        return result;
    }

    // Select positive and negative candidates independently before applying
    // any continuity gate. A later cell may contain the closest negative.
    // The nearest negative evidence gives the boundary distance. Its height is the lowest
    // below-plane height measured anywhere in the bin (19 section 3.2: neg_h[i] is a per-bin
    // minimum; section 3.4: measured in-pit points ? neg_h[i] : null), not the height of the
    // cell that holds the boundary: an edge located by pierce alone still reports the depth
    // measured farther out in the same bin. Nothing measured below the plane in the bin: no
    // height. Until 2026-10-13 the height was taken from the boundary's own cell only, so such
    // an edge had none. By the letter of 19 the lowest point counts at any range, also behind a
    // positive obstacle; 11 says distance, source and height relate to the same evidence, which
    // reads differently when that point lies far behind the edge (an open question, see
    // tests/rns_host/NORMAL_GRID.md item 4).
    const auto lower_height = [](std::optional<double>& lowest, const std::optional<double>& height) {
        if (height && (!lowest || *height < *lowest)) lowest = height;
    };
    auto negative = input.before_grid_negative_m;
    auto negative_height = input.before_grid_negative_height_m;
    for (std::size_t k = 0; k < input.cell_count; ++k) {
        const CellEvidence& cell = input.cells[k];
        if (!cell.pierce && !cell.below) continue;
        lower_height(negative_height, cell.below_height_m);
        const double near = input.blind_m + static_cast<double>(k) * options.dr_m;
        if (!negative || near < *negative) negative = near;
    }
    if (input.tail_negative_m) {
        lower_height(negative_height, input.tail_negative_height_m);
        if (!negative || *input.tail_negative_m < *negative) negative = input.tail_negative_m;
    }
    if (negative && (!input.positive_m || *negative <= *input.positive_m)) {
        result.boundary = BoundaryKind::Negative;
        result.block_m = negative;
        result.block_height_m = negative_height;
    } else if (input.positive_m) {
        result.boundary = BoundaryKind::Positive;
        result.block_m = input.positive_m;
        result.block_height_m = input.positive_height_m;
    }
    // A boundary at or inside the blind range: nothing can be verified free.
    if (result.block_m && *result.block_m <= input.blind_m) {
        result.stop = StopReason::Boundary;
        return result;
    }
    // FREE starts empty at the blind range and grows cell by cell.
    result.free_end_m = input.blind_m;
    result.stop = StopReason::Range;
    double previous_mean = 0.0;
    double maximum_slope = 0.0;
    const double slope_limit = std::tan(options.slope_max_rad);
    const double variance_limit = options.sigma_max_m * options.sigma_max_m;
    for (std::size_t k = 0; k < input.cell_count; ++k) {
        const double far = input.blind_m + static_cast<double>(k + 1) * options.dr_m;
        if (result.block_m && far >= *result.block_m) {
            result.stop = StopReason::Boundary;
            break;
        }
        // Coverage: the share of this cell's expected rays redeemed by their own
        // ground returns must reach cover_min (all of them when cover_min = 1).
        const CellEvidence& cell = input.cells[k];
        if (cell.expected == 0 || cell.ground == 0 || cell.covered_ground == 0 ||
            (options.cover_min == 1.0 && cell.covered_ground != cell.expected) ||
            static_cast<double>(cell.covered_ground) / static_cast<double>(cell.expected) < options.cover_min) {
            result.stop = StopReason::Coverage;
            break;
        }
        if (input.has_eroded_t && !cell.t_ok) {
            result.stop = StopReason::Traversability;
            break;
        }
        // Slope: mean ground height step to the previous passed cell over dr.
        double adjacent_slope = 0.0;
        if (result.passed_cells != 0) {
            adjacent_slope = std::abs(cell.mean_height_m - previous_mean) / options.dr_m;
            if (!std::isfinite(adjacent_slope)) return {};
            if (adjacent_slope > slope_limit) {
                result.stop = StopReason::Slope;
                break;
            }
        }
        // Roughness: population variance of the cell's ground heights.
        const double variance = cell.m2_height_m2 / static_cast<double>(cell.ground);
        if (!finite_nonnegative(variance)) return {};
        if (variance > variance_limit) {
            result.stop = StopReason::Roughness;
            break;
        }
        // The cell passes: FREE extends to its far edge.
        result.free_end_m = far;
        ++result.passed_cells;
        maximum_slope = std::max(maximum_slope, adjacent_slope);
        previous_mean = cell.mean_height_m;
        if (result.passed_cells >= 2) result.slope_rad = std::atan(maximum_slope);
        result.t_backed_prefix = input.has_eroded_t;
    }
    return result;
}

}  // namespace perception::geometry
