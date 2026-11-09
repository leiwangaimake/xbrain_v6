/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_semantic_kernel.cpp
 * Brief: Conservative footprint -> per-bin distance projection, and semantic FREE restriction.
 *
 * Description:
 * What problem this solves. Two pure numeric kernels of the S channel (19 section 3.4A):
 *   project_semantic_footprint  inflate one footprint and compute, per Profile bin, the nearest
 *                             distance of the inflated footprint inside that bin's sector;
 *   restrict_semantic_prefix    apply a strictly nearer semantic boundary to one bin's geometric
 *                             walk: move the block in and shorten FREE to whole cells.
 *
 * Why conservative numerics. A semantic block that is rounded away is a missed obstacle; one
 * that is slightly too near is only a shorter FREE. So every rounding step goes the safe way:
 * polygon validity uses an exact orientation sign, the inflation is rounded outward, the
 * sector clipping widens its tolerance, distances are rounded down, and a bin boundary shared
 * by two bins counts for both. Nothing here allocates; scratch lives on the stack.
 */
#include "output/rns_semantic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception::rns {
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double epsilon = std::numeric_limits<double>::epsilon();
constexpr double smallest = std::numeric_limits<double>::denorm_min();
// A 12-vertex polygon plus the 4 square corners: the envelope vertex bound.
constexpr std::size_t point_capacity = kSemanticVertices + 4;
struct XY { double x = 0, y = 0; };

// Error-free transformations: a + b == sum + error and a - b == difference +
// error exactly (Knuth two-sum), the building blocks of the exact sign below.
void two_sum(double a, double b, double& sum, double& error) noexcept {
    sum = a + b;
    const double b_virtual = sum - a;
    error = (a - (sum - b_virtual)) + (b - b_virtual);
}
void two_diff(double a, double b, double& difference, double& error) noexcept {
    difference = a - b;
    const double b_virtual = a - difference;
    error = (a - (difference + b_virtual)) + (b_virtual - b);
}
// a + b rounded toward +inf (upper) or -inf, decided from the exact error term.
double add_directed(double a, double b, bool upper) noexcept {
    double sum = 0, error = 0;
    two_sum(a, b, sum, error);
    if ((upper && error > 0) || (!upper && error < 0))
        sum = std::nextafter(sum, upper ? infinity : -infinity);
    return sum;
}

// Exact sign for original FLOAT vertices. Difference/product expansions have
// at most 16 components. Float input exponents and the 64-coordinate bound
// keep every product/error safely away from double overflow and underflow.
// Add value to a non-overlapping expansion (Shewchuk grow-expansion); zero
// components are dropped, the last component has the largest magnitude.
void grow(double* expansion, std::size_t& size, double value) noexcept {
    std::size_t out = 0;
    double carry = value;
    for (std::size_t i = 0; i < size; ++i) {
        double sum = 0, error = 0;
        two_sum(carry, expansion[i], sum, error);
        if (error != 0) expansion[out++] = error;
        carry = sum;
    }
    if (carry != 0 || out == 0) expansion[out++] = carry;
    size = out;
}
// Exact sign of cross(b - a, c - a): each difference is split exactly into two
// doubles, each of the 8 products exactly via fma, and all are summed exactly;
// the sign of the largest component is the sign of the true value.
int orientation(Point a, Point b, Point c) noexcept {
    double x[2]{}, y[2]{}, u[2]{}, v[2]{};
    two_diff(b.x, a.x, x[1], x[0]);
    two_diff(b.y, a.y, y[1], y[0]);
    two_diff(c.x, a.x, u[1], u[0]);
    two_diff(c.y, a.y, v[1], v[0]);
    double expansion[32]{};
    std::size_t size = 0;
    for (std::size_t i = 0; i < 2; ++i) for (std::size_t j = 0; j < 2; ++j) {
        const double p = x[i] * v[j], q = y[i] * u[j];
        grow(expansion, size, std::fma(x[i], v[j], -p));
        grow(expansion, size, p);
        grow(expansion, size, -std::fma(y[i], u[j], -q));
        grow(expansion, size, -q);
    }
    const double leading = expansion[size - 1];
    return leading > 0 ? 1 : leading < 0 ? -1 : 0;
}
// 3..12 finite vertices within the coordinate bound, no duplicates, every vertex
// left of or on every edge (CCW convex) and some positive area, all decided with
// the exact sign so rounding cannot flip the verdict.
bool valid_polygon(const Point* points, std::size_t count) noexcept {
    if (!points || count < 3 || count > kSemanticVertices) return false;
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
            std::abs(points[i].x) > kSemanticCoordinateBound ||
            std::abs(points[i].y) > kSemanticCoordinateBound) return false;
        for (std::size_t j = 0; j < i; ++j)
            if (points[i].x == points[j].x && points[i].y == points[j].y) return false;
    }
    bool area = false;
    for (std::size_t i = 0; i < count; ++i) for (std::size_t j = 0; j < count; ++j) {
        const int side = orientation(points[i], points[(i + 1) % count], points[j]);
        if (side < 0) return false;
        area = area || side > 0;
    }
    return area;
}

// Direction order starting at +x: half 0 is angles [0, pi), half 1 is [pi, 2pi).
// Used to merge the polygon's and the square's edges by direction.
int half(XY edge) noexcept { return edge.y > 0 || (edge.y == 0 && edge.x >= 0) ? 0 : 1; }
int compare_axis(XY edge, XY axis) noexcept {
    if (half(edge) != half(axis)) return half(edge) < half(axis) ? -1 : 1;
    // axis has exactly one nonzero component, equal to +/-1: no unstable
    // general cross product is needed to merge the edge-angle sequences.
    const double cross = edge.x * axis.y - edge.y * axis.x;
    return cross > 0 ? -1 : cross < 0 ? 1 : 0;
}
// Minkowski sum of the CCW convex polygon with the square [-m, m]^2: walk both
// edge sequences in direction order from the lowest vertex and the (-1,-1)
// corner, emitting vertex + corner at each step. The square contains the disk
// of radius m, so the result contains the round inflation of 19 section 3.4A.
bool envelope(const Point* p, std::size_t count, double margin,
              XY* output, std::size_t& size) noexcept {
    size = 0;
    // Reassert the public polygon gate locally: modulo below must never rely
    // on an implicit caller-only nonzero-count precondition.
    if (!p || !output || count < 3 || count > kSemanticVertices) return false;
    if (margin == 0) {
        for (std::size_t i = 0; i < count; ++i) output[size++] = {p[i].x, p[i].y};
        return true;
    }
    // Start at the lowest (then leftmost) vertex, where the edge direction
    // order begins, matching the square's (-1,-1) corner.
    std::size_t first = 0;
    for (std::size_t i = 1; i < count; ++i)
        if (p[i].y < p[first].y || (p[i].y == p[first].y && p[i].x < p[first].x)) first = i;
    constexpr XY corners[]{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    constexpr XY axes[]{{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    std::size_t polygon_edges = 0, square_edges = 0;
    while (polygon_edges < count || square_edges < 4) {
        if (size == point_capacity) return false;
        // The polygon vertex and square corner at the current merge position.
        const auto vertex = p[(first + polygon_edges) % count];
        const auto corner = corners[square_edges % 4];
        // Directed sums preserve even a positive margin below one coordinate
        // ULP. These outward support vertices enclose the exact square sum.
        const XY point{add_directed(vertex.x, corner.x * margin, corner.x > 0),
                       add_directed(vertex.y, corner.y * margin, corner.y > 0)};
        // 128 = coordinate bound 64 + margin bound 64.
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            std::abs(point.x) > 128 || std::abs(point.y) > 128) return false;
        output[size++] = point;
        if (polygon_edges == count) { ++square_edges; continue; }
        if (square_edges == 4) { ++polygon_edges; continue; }
        // Advance whichever edge comes first in direction (both when parallel).
        const auto next = p[(first + polygon_edges + 1) % count];
        const int order = compare_axis({static_cast<double>(next.x) - vertex.x,
                                       static_cast<double>(next.y) - vertex.y}, axes[square_edges]);
        if (order <= 0) ++polygon_edges;
        if (order >= 0) ++square_edges;
    }
    return size >= 3;
}

// Numerical uncertainty is widened, never used to discard a nearly collinear
// or shared-edge intersection. The absolute term covers subnormal products.
int origin_side(XY a, XY b) noexcept {
    const double p = a.x * b.y, q = a.y * b.x;
    const double value = p - q;
    const double error = std::nextafter(8 * epsilon * (std::abs(p) + std::abs(q)) + 8 * smallest, infinity);
    return value > error ? 1 : value < -error ? -1 : 0;
}
// Is the origin (base_link) inside the envelope? A point in a 2-D convex hull lies
// in some triangle of its points, so every triangle is tried (bounding-box pretest
// first). An uncertain side test counts as "on", i.e. inside: distance 0 is the
// most conservative answer.
bool origin_in_hull(const XY* points, std::size_t size) noexcept {
    for (std::size_t a = 0; a < size; ++a)
        for (std::size_t b = a + 1; b < size; ++b)
            for (std::size_t c = b + 1; c < size; ++c) {
                if (std::min({points[a].x, points[b].x, points[c].x}) > 0 ||
                    std::max({points[a].x, points[b].x, points[c].x}) < 0 ||
                    std::min({points[a].y, points[b].y, points[c].y}) > 0 ||
                    std::max({points[a].y, points[b].y, points[c].y}) < 0) continue;
                const int ab = origin_side(points[a], points[b]);
                const int bc = origin_side(points[b], points[c]);
                const int ca = origin_side(points[c], points[a]);
                // The triangle's orientation is unknown, so either common sign means inside (or on).
                if ((ab >= 0 && bc >= 0 && ca >= 0) || (ab <= 0 && bc <= 0 && ca <= 0)) return true;
            }
    return false;
}

// Unit direction of a sector boundary; angle 0 is the exact +x axis, avoiding
// cos/sin rounding on the most common boundary.
struct Ray { double x = 0, y = 0; bool exact_axis = false; };
Ray ray(double angle) noexcept {
    if (angle == 0) return {1, 0, true};
    return {std::cos(angle), std::sin(angle), false};
}
// Signed side of p relative to the ray's line (sense selects the inner side),
// rounded up and widened by allowance: a point near the line counts as inside.
double plane_upper(Ray r, XY p, double sense, double allowance) noexcept {
    // On the exact +x axis the side is p.y itself, exact, so no allowance is needed.
    if (r.exact_axis) return sense * p.y;
    const double value = sense * (r.x * p.y - r.y * p.x);
    return std::nextafter(value + allowance, infinity);
}
// Clip the parameter interval [lo, hi] of segment a-b to the inner half-plane of
// one sector ray; false when nothing of the segment remains.
bool clip(Ray r, double sense, XY a, XY b, double& lo, double& hi) noexcept {
    // Relative arithmetic/libm boundary envelope. It only adds possible edge
    // intersections; it never treats a small denominator as a missing object.
    const double scale = std::abs(a.x) + std::abs(a.y) + std::abs(b.x) + std::abs(b.y);
    const double allowance = std::nextafter(64 * epsilon * scale + 8 * smallest, infinity);
    const double fa = plane_upper(r, a, sense, allowance), fb = plane_upper(r, b, sense, allowance);
    // Both outside: nothing kept. Both inside: interval unchanged. Otherwise cut
    // at the crossing, rounded so the kept part only grows.
    if (fa < 0 && fb < 0) return false;
    if (fa >= 0 && fb >= 0) return true;
    if (fa < 0) {
        // a outside: the crossing raises lo (rounded down, keeping more).
        const double denominator = add_directed(fb, -fa, true);
        const double lower = std::nextafter((-fa) / denominator, -infinity);
        lo = std::max(lo, lower);
    } else {
        // b outside: the crossing lowers hi (rounded up); a non-positive
        // denominator keeps the whole remaining interval.
        const double denominator = add_directed(fa, -fb, false);
        const double upper = denominator > 0 ? std::nextafter(fa / denominator, infinity) : 1;
        hi = std::min(hi, upper);
    }
    return lo <= hi;
}
// Distance from the origin to the part [lo, hi] of segment a-b (projection
// clamped to the part), then rounded DOWN by an error bound and never below 0:
// a slightly nearer boundary is the safe error.
double nearest_on_segment(XY a, XY b, double lo, double hi) noexcept {
    const XY d{b.x - a.x, b.y - a.y};
    const double scale = std::max(std::abs(d.x), std::abs(d.y));
    double t = lo;
    // Normalise the direction before the dot products (no overflow/underflow);
    // a zero-length segment is just its end point.
    if (scale > 0) {
        const double x = d.x / scale, y = d.y / scale;
        const double optimum = -(a.x * x + a.y * y) / (scale * (x * x + y * y));
        t = std::max(lo, std::min(hi, optimum));
    }
    const XY p{std::fma(t, d.x, a.x), std::fma(t, d.y, a.y)};
    const double distance = std::hypot(p.x, p.y);
    const double error = std::nextafter(64 * epsilon *
        (std::abs(a.x) + std::abs(a.y) + std::abs(b.x) + std::abs(b.y)) + 8 * smallest, infinity);
    return std::max(0.0, std::nextafter(distance - error, -infinity));
}
// Same layout bounds as the Profile contract; the covered sector must lie within
// [-pi, pi] so sector angles never wrap around.
bool layout_valid(const SemanticLayout& layout) noexcept {
    if (layout.bins == 0 || layout.bins > kMaxBins || !std::isfinite(layout.angle_min_rad) ||
        !std::isfinite(layout.angle_step_rad) || layout.angle_step_rad <= 0 ||
        !std::isfinite(layout.range_max_m) || layout.range_max_m <= 0 || layout.range_max_m > 6) return false;
    const double lo = layout.angle_min_rad - layout.angle_step_rad / 2;
    const double hi = layout.angle_min_rad + (static_cast<double>(layout.bins) - 0.5) * layout.angle_step_rad;
    return std::isfinite(lo) && std::isfinite(hi) && lo >= -pi && hi <= pi && hi > lo;
}
bool nonnegative(double value) noexcept { return std::isfinite(value) && value >= 0; }

// Consistency of one bin's geometric walk before it is modified: boundary kind
// agrees with the block and the height sign, FREE ends exactly at
// blind + passed_cells * dr and before the block, a slope needs >= 2 cells, T
// backing needs >= 1 cell, and every cell lies within 6 m.
bool valid_prefix(const geometry::BinWalkResult& p, const geometry::CellEvidence* cells,
                 std::size_t count, double blind, double dr) noexcept {
    using geometry::BoundaryKind;
    if (!p.valid || count > geometry::kMaxRadialCells || p.passed_cells > count ||
        (count != 0 && cells == nullptr) || !nonnegative(blind) || blind > 6 ||
        !std::isfinite(dr) || dr <= 0) return false;
    if (p.boundary != BoundaryKind::None && p.boundary != BoundaryKind::Positive &&
        p.boundary != BoundaryKind::Negative) return false;
    if (p.block_m) {
        if (!nonnegative(*p.block_m) || *p.block_m > 6 || p.boundary == BoundaryKind::None) return false;
        if (p.block_height_m && (!std::isfinite(*p.block_height_m) ||
            (p.boundary == BoundaryKind::Positive && *p.block_height_m <= 0) ||
            (p.boundary == BoundaryKind::Negative && *p.block_height_m > 0))) return false;
    } else if (p.boundary != BoundaryKind::None || p.block_height_m) return false;
    if (p.slope_rad && (!nonnegative(*p.slope_rad) || *p.slope_rad >= pi / 2)) return false;
    if (p.free_end_m) {
        const double end = blind + static_cast<double>(p.passed_cells) * dr;
        if (!nonnegative(*p.free_end_m) || *p.free_end_m > 6 || !std::isfinite(end) ||
            *p.free_end_m != end || (p.block_m && *p.free_end_m >= *p.block_m)) return false;
    } else if (p.passed_cells != 0 || p.slope_rad || p.t_backed_prefix) return false;
    if (p.passed_cells < 2 && p.slope_rad) return false;
    if (p.passed_cells == 0 && p.t_backed_prefix) return false;
    for (std::size_t i = 0; i < count; ++i) {
        const double near = blind + static_cast<double>(i) * dr;
        const double far = blind + static_cast<double>(i + 1) * dr;
        if (!std::isfinite(far) || far <= near || far > 6 || !std::isfinite(cells[i].mean_height_m)) return false;
    }
    return true;
}
} // namespace

bool project_semantic_footprint(const Point* vertices, std::size_t count,
    const SemanticLayout& layout, double margin_m, std::optional<double>* distances,
    std::size_t capacity) noexcept {
    // A truthful allocated capacity is a caller precondition. Do not iterate
    // an unbounded declaration merely to clear it.
    if (capacity > kMaxBins || distances == nullptr) return false;
    // Cleared first, so a false return never leaves stale distances.
    std::fill_n(distances, capacity, std::nullopt);
    if (!layout_valid(layout) || capacity < layout.bins || !nonnegative(margin_m) ||
        margin_m > 64 || !valid_polygon(vertices, count)) return false;
    XY points[point_capacity]{};
    std::size_t size = 0;
    if (!envelope(vertices, count, margin_m, points, size)) return false;
    // The inflated footprint may contain the robot origin: every bin is blocked
    // at distance 0.
    if (origin_in_hull(points, size)) {
        std::fill_n(distances, layout.bins, std::optional<double>{0});
        return true;
    }
    // Closed sector [lo, hi] per bin (angle_min is bin 0's centre); adjacent bins
    // share an edge, so evidence exactly on it enters both.
    const double first = layout.angle_min_rad - layout.angle_step_rad / 2;
    for (std::size_t bin = 0; bin < layout.bins; ++bin) {
        const double lo = first + static_cast<double>(bin) * layout.angle_step_rad;
        const double hi = first + static_cast<double>(bin + 1) * layout.angle_step_rad;
        // A sector lost to rounding is a layout failure: clear and refuse.
        if (!std::isfinite(hi) || hi <= lo) {
            std::fill_n(distances, capacity, std::nullopt);
            return false;
        }
        // Split a wide sector into at most four convex wedges. Every pair of
        // support vertices is inside their convex hull, and the pairs include
        // every hull edge. Outside an origin-containing hull (handled above),
        // the sector's minimum radius occurs on one of those edges. Thus no
        // approximate hull predicate can delete a thin/off-centre footprint.
        const auto pieces = static_cast<std::size_t>(std::max(1.0, std::ceil((hi - lo) / (pi / 2))));
        if (pieces > 4) {
            std::fill_n(distances, capacity, std::nullopt);
            return false;
        }
        // Nearest distance over every envelope point pair clipped to each wedge.
        double nearest = infinity;
        for (std::size_t piece = 0; piece < pieces; ++piece) {
            const Ray lower = ray(lo + (hi - lo) * static_cast<double>(piece) / pieces);
            const Ray upper = ray(lo + (hi - lo) * static_cast<double>(piece + 1) / pieces);
            for (std::size_t a = 0; a < size; ++a) for (std::size_t b = a + 1; b < size; ++b) {
                double begin = 0, end = 1;
                if (clip(lower, 1, points[a], points[b], begin, end) &&
                    clip(upper, -1, points[a], points[b], begin, end))
                    nearest = std::min(nearest, nearest_on_segment(points[a], points[b], begin, end));
            }
        }
        // Beyond range_max_m the Profile has nothing to compare with: stays null.
        if (nearest <= layout.range_max_m) distances[bin] = nearest;
    }
    return true;
}

bool restrict_semantic_prefix(const geometry::BinWalkResult& original, const geometry::CellEvidence* cells,
    std::size_t count, double blind, double dr, double semantic,
    geometry::BinWalkResult& output, bool& selected) noexcept {
    const auto input = original; // The caller may use the same input/output object.
    output = {};
    selected = false;
    if (!nonnegative(semantic) || semantic > 6 || !valid_prefix(input, cells, count, blind, dr)) return false;
    // Not strictly nearer than the geometric block: unchanged (only tightens).
    if (input.block_m && semantic >= *input.block_m) { output = input; return true; }
    // The semantic boundary becomes a positive block at d_sem with unknown height.
    auto result = input;
    result.boundary = geometry::BoundaryKind::Positive;
    result.block_m = semantic;
    result.block_height_m.reset(); // Pending S identity; converter supplies wire S.
    // No FREE to shorten: it stays null (19 section 3.4A).
    if (!input.free_end_m) { output = result; selected = true; return true; }
    // Boundary inside the blind range: no FREE remains.
    if (semantic <= blind) {
        result.free_end_m.reset();
        result.passed_cells = 0;
        result.slope_rad.reset();
        result.t_backed_prefix = false;
        result.stop = geometry::StopReason::Boundary;
    } else {
        // Keep whole cells whose far edge is strictly before the boundary (the
        // same double formula as the walk); FREE ends at the last kept far edge.
        std::size_t retained = 0;
        while (retained < input.passed_cells && blind + static_cast<double>(retained + 1) * dr < semantic)
            ++retained;
        if (retained < input.passed_cells) result.stop = geometry::StopReason::Boundary;
        result.passed_cells = retained;
        result.free_end_m = blind + static_cast<double>(retained) * dr;
        // Slope over the kept cells only: max adjacent height step / dr.
        result.slope_rad.reset();
        double maximum = 0;
        for (std::size_t i = 1; i < retained; ++i) {
            const double slope = std::abs(cells[i].mean_height_m - cells[i - 1].mean_height_m) / dr;
            if (!std::isfinite(slope)) return false;
            maximum = std::max(maximum, slope);
        }
        if (retained >= 2) result.slope_rad = std::atan(maximum);
        // T backing survives only if some cells remain.
        result.t_backed_prefix = input.t_backed_prefix && retained != 0;
    }
    output = result;
    selected = true;
    return true;
}
} // namespace perception::rns
