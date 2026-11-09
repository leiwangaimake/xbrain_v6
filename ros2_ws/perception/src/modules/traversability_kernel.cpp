/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: traversability_kernel.cpp
 * Brief: Motion-margin bound, T-mask erosion and T-restricted FREE prefix (pure kernels).
 *
 * Description:
 * What problem this solves. Implements the kernels of traversability.hpp:
 *   t_margin_upper_bound()    how far the robot may have moved since an observation of a given
 *                          age: (v_max + omega_max * r) * age + jitter, rounded upward. Used
 *                          by the S channel (rns_semantic) to inflate object footprints.
 *   erode_traversability()  shrink a per-cell traversable mask by that margin (T channel);
 *   restrict_normal_prefix() cut a geometric FREE prefix at the first non-eroded cell (T).
 * All three run in the service: the T owner (NormalTraversability) is wired since 2026-09-30
 * (plan 2f) and calls the two T kernels per Profile frame with a fresh mask. Under the B3-1 hold
 * the nearest cell has no observed near-side neighbourhood, so erosion leaves it false and the
 * T-restricted prefix stops at the blind range.
 *
 * Numerics. Every bound is rounded toward the safe side (a larger margin), never to nearest.
 */
#include "modules/traversability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception::geometry {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
// Strictly above pi/2. Combined with asin(q) <= (pi/2)*q for q in [0,1],
// this avoids relying on a library asin's last-bit error as a safety bound.
constexpr double kHalfPiUpper = 1.5707963267948968;

bool finite_nonnegative(double value) noexcept {
    return std::isfinite(value) && value >= 0.0;
}

// a * b and a + b rounded one step upward (a margin must never shrink by rounding).
double multiply_up(double a, double b) noexcept {
    if (a == 0.0 || b == 0.0) return 0.0;
    return std::nextafter(a * b, kInfinity);
}

double add_up(double a, double b) noexcept {
    if (a == 0.0) return b;
    if (b == 0.0) return a;
    return std::nextafter(a + b, kInfinity);
}

// Smallest whole number of cells covering numerator / spacing (at least 1).
bool ceil_radius(double numerator, double spacing, std::size_t limit,
                std::size_t& radius) noexcept {
    const double candidate = std::max(1.0, std::ceil(numerator / spacing));
    if (!std::isfinite(candidate) || candidate >= static_cast<double>(limit)) return false;
    radius = static_cast<std::size_t>(candidate);
    // Division can round just above an integer down to that integer, e.g.
    // .9/.3. FMA computes the residual without rounding radius*spacing first.
    if (std::fma(-static_cast<double>(radius), spacing, numerator) > 0.0) ++radius;
    return radius < limit;
}

void clear_output(std::uint8_t* raw, std::uint8_t* eroded, std::size_t capacity) noexcept {
    if (raw) std::fill_n(raw, capacity, std::uint8_t{0});
    if (eroded) std::fill_n(eroded, capacity, std::uint8_t{0});
}

struct AddressRange { std::uintptr_t begin = 0, end = 0; };

bool address_range(const void* pointer, std::size_t count, std::size_t width,
                  AddressRange& result) noexcept {
    if (count != 0 && pointer == nullptr) return false;
    if (count > std::numeric_limits<std::size_t>::max() / width) return false;
    result.begin = reinterpret_cast<std::uintptr_t>(pointer);
    const std::size_t bytes = count * width;
    if (bytes > std::numeric_limits<std::uintptr_t>::max() - result.begin) return false;
    result.end = result.begin + bytes;
    return true;
}

bool overlaps(AddressRange a, AddressRange b) noexcept {
    return a.begin != a.end && b.begin != b.end && a.begin < b.end && b.begin < a.end;
}

// The input and the three output buffers must not overlap in memory.
bool independent_buffers(const TCount* counts, std::size_t capacity,
                         const std::uint8_t* raw, const std::uint8_t* eroded,
                         const std::uint32_t* prefix, std::size_t prefix_capacity) noexcept {
    AddressRange ranges[4];
    if (!address_range(counts, capacity, sizeof(TCount), ranges[0]) ||
        !address_range(raw, capacity, sizeof(std::uint8_t), ranges[1]) ||
        !address_range(eroded, capacity, sizeof(std::uint8_t), ranges[2]) ||
        !address_range(prefix, prefix_capacity, sizeof(std::uint32_t), ranges[3])) return false;
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = a + 1; b < 4; ++b) {
            if (overlaps(ranges[a], ranges[b])) return false;
        }
    }
    return true;
}

}  // namespace

bool valid_t_options(const TOptions& options) noexcept {
    return std::isfinite(options.tau) && options.tau > 0.0 && options.tau <= 1.0 &&
           finite_nonnegative(options.v_max_mps) && finite_nonnegative(options.omega_max_rps) &&
           finite_nonnegative(options.jitter_m) && options.max_age_ns >= 0;
}

// Upper bound of the robot's displacement over age_ns at radius far (19 section
// 3.4A); +inf for invalid input, which callers treat as "cannot be bounded".
double t_margin_upper_bound(const TOptions& options, std::int64_t age_ns, double far) noexcept {
    if (!valid_t_options(options) || age_ns < 0 || age_ns > options.max_age_ns ||
        !finite_nonnegative(far)) return kInfinity;
    if (age_ns == 0) return options.jitter_m;
    // Round both integer conversion and unit conversion outward. Positive
    // products that underflow to zero are lifted to the minimum subnormal.
    const double age_upper = std::nextafter(
        std::nextafter(static_cast<double>(age_ns), kInfinity) / 1e9, kInfinity);
    const double speed = add_up(options.v_max_mps, multiply_up(options.omega_max_rps, far));
    return add_up(multiply_up(speed, age_upper), options.jitter_m);
}

bool erode_traversability(const TLayout& layout, const TOptions& options, std::int64_t age_ns,
    const TCount* counts, std::size_t capacity, std::uint8_t* raw, std::uint8_t* eroded,
    std::uint32_t* prefix, std::size_t prefix_capacity) noexcept {
    // A truthful, bounded capacity backed by actual caller-owned allocations
    // is an API precondition. An unbounded declaration is rejected untouched.
    if (capacity > kMaxGridCells) return false;
    // Check addresses before writes: invalid aliases must not turn output
    // clearing into mutation of const input or scratch owned by another role.
    if (!independent_buffers(counts, capacity, raw, eroded, prefix, prefix_capacity)) return false;
    clear_output(raw, eroded, capacity);
    if (!valid_t_options(options) || age_ns < 0 || age_ns > options.max_age_ns ||
        layout.bins > kGeometryMaxBins || layout.stride > kMaxRadialCells ||
        layout.cells > layout.stride || !finite_nonnegative(layout.blind_m) ||
        !std::isfinite(layout.dr_m) || layout.dr_m <= 0.0 ||
        !std::isfinite(layout.angle_step_rad) || layout.angle_step_rad <= 0.0) return false;
    const std::size_t slots = layout.bins * layout.stride;
    const std::size_t pitch = layout.stride + 1;
    const std::size_t required_prefix = (layout.bins + 1) * pitch;
    if (slots > kMaxGridCells || capacity < slots || prefix_capacity < required_prefix) return false;
    for (std::size_t k = 0; k < layout.cells; ++k) {
        const double near = layout.blind_m + static_cast<double>(k) * layout.dr_m;
        const double far = layout.blind_m + static_cast<double>(k + 1) * layout.dr_m;
        if (!std::isfinite(far) || far <= near || far > 6.0) return false;
    }
    // Validate every active cell first, including cells that erosion will
    // reject for domain extent. Padding counts are intentionally not evidence.
    for (std::size_t bin = 0; bin < layout.bins; ++bin) {
        for (std::size_t k = 0; k < layout.cells; ++k) {
            const auto& cell = counts[bin * layout.stride + k];
            if (cell.ground > kGeometryPixels || cell.traversable > cell.ground) return false;
        }
    }
    std::fill_n(prefix, required_prefix, std::uint32_t{0});
    for (std::size_t bin = 0; bin < layout.bins; ++bin) {
        for (std::size_t k = 0; k < layout.stride; ++k) {
            if (k < layout.cells) {
                const auto& cell = counts[bin * layout.stride + k];
                raw[bin * layout.stride + k] = static_cast<std::uint8_t>(cell.ground != 0 &&
                    static_cast<double>(cell.traversable) / static_cast<double>(cell.ground) >= options.tau);
            }
            prefix[(bin + 1) * pitch + k + 1] = raw[bin * layout.stride + k] +
                prefix[bin * pitch + k + 1] + prefix[(bin + 1) * pitch + k] - prefix[bin * pitch + k];
        }
    }
    for (std::size_t k = 0; k < layout.cells; ++k) {
        const double near = layout.blind_m + static_cast<double>(k) * layout.dr_m;
        const double far = layout.blind_m + static_cast<double>(k + 1) * layout.dr_m;
        // Conservative metric envelope for the entire cell: angular velocity
        // uses far. Angular clearance uses an upward-rounded (pi/2)*M/near
        // envelope for asin(M/near). This is an explicit conservative choice,
        // not a unique mandated R4 kernel, and does not alter any threshold.
        const double margin = t_margin_upper_bound(options, age_ns, far);
        std::size_t radial = 0, angular = 0;
        if (margin > 0.0) {
            // Also safely handles an overflowed, arbitrarily large margin.
            if (margin >= near) continue;
            const double ratio_upper = std::nextafter(margin / near, kInfinity);
            const double angle_upper = multiply_up(kHalfPiUpper, ratio_upper);
            // Bounds precede integer conversion. Domain checks reject rather
            // than clip, including infinite quotients from tiny spacings.
            if (!ceil_radius(margin, layout.dr_m, layout.cells, radial) ||
                !ceil_radius(angle_upper, layout.angle_step_rad, layout.bins, angular)) continue;
        }
        if (radial > k || radial >= layout.cells - k) continue;
        for (std::size_t bin = 0; bin < layout.bins; ++bin) {
            if (angular > bin || angular >= layout.bins - bin) continue;
            const std::size_t top = bin - angular, bottom = bin + angular + 1;
            const std::size_t left = k - radial, right = k + radial + 1;
            const std::uint32_t ones = prefix[bottom * pitch + right] - prefix[top * pitch + right] -
                                       prefix[bottom * pitch + left] + prefix[top * pitch + left];
            const std::size_t area = (bottom - top) * (right - left);
            eroded[bin * layout.stride + k] = static_cast<std::uint8_t>(ones == area);
        }
    }
    return true;
}

BinWalkResult restrict_normal_prefix(const BinWalkResult& original, const CellEvidence* cells,
    const std::uint8_t* eroded, std::size_t count, double blind_m, double dr_m, bool fresh_t) noexcept {
    BinWalkResult result = original;
    result.t_backed_prefix = false;
    if (!fresh_t) return result;
    if (!original.valid || count > kMaxRadialCells || original.passed_cells > kMaxRadialCells ||
        original.passed_cells > count ||
        !finite_nonnegative(blind_m) || !std::isfinite(dr_m) || dr_m <= 0.0 ||
        (count != 0 && (!cells || !eroded))) return {};
    for (std::size_t k = 0; k < count; ++k) {
        if (eroded[k] > 1 || !std::isfinite(cells[k].mean_height_m)) return {};
    }
    if (!original.free_end_m) {
        if (original.passed_cells != 0) return {};
        return result;
    }
    const double original_end = blind_m + static_cast<double>(original.passed_cells) * dr_m;
    if (!std::isfinite(original_end) || *original.free_end_m != original_end) return {};
    std::size_t retained = 0;
    const std::size_t available = std::min(count, original.passed_cells);
    while (retained < available && eroded[retained] == 1) ++retained;
    if (retained < original.passed_cells) result.stop = StopReason::Traversability;
    result.passed_cells = retained;
    result.free_end_m = blind_m + static_cast<double>(retained) * dr_m;
    result.slope_rad.reset();
    double maximum = 0.0;
    for (std::size_t k = 1; k < retained; ++k) {
        const double slope = std::abs(cells[k].mean_height_m - cells[k - 1].mean_height_m) / dr_m;
        if (!std::isfinite(slope)) return {};
        maximum = std::max(maximum, slope);
    }
    if (retained >= 2) result.slope_rad = std::atan(maximum);
    result.t_backed_prefix = retained > 0;
    return result;
}

}  // namespace perception::geometry
