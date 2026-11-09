/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_object_geometry_scratch.cpp
 * Brief: ObjectGeometryBuilder with its footprint scratch against the builder it replaced.
 *
 * Description:
 * The footprint's work lists (bucket extremes, widened points, hull, decimation flags, float
 * vertices) used to be locals of build(), allocated in every call; they are now members whose
 * capacity is reused. Two checks:
 *   1. every field of every result equals, bit for bit, the frozen original builder
 *      (tests/reference/rns_object_geometry_original.*) when both are given the same calls in
 *      the same order. The orders mix large and small objects, decimated, degenerate, fallback,
 *      too-few-point, non-finite and invalid calls, so that anything left in the scratch by one
 *      build would show in the next; results returned earlier must keep their values;
 *   2. allocation counts: once the scratch has grown to the largest object, a Located build
 *      allocates exactly once (the vertices its result owns) and any other build not at all; the
 *      original's count for the same calls is printed beside it.
 * Synthetic rays and scenes; no SDK, OpenCV, camera or calibration certification.
 */

#include "output/rns_object_geometry.hpp"
#include "rns_object_geometry_original.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <new>
#include <vector>

namespace {
std::atomic<std::uint64_t> allocations{0};
void* allocate(std::size_t n) {
    void* p = std::malloc(n == 0 ? 1 : n);
    if (!p) throw std::bad_alloc();
    allocations.fetch_add(1, std::memory_order_relaxed);
    return p;
}
void* allocate_aligned(std::size_t n, std::size_t alignment) {
    void* p = nullptr;
    if (posix_memalign(&p, alignment, n == 0 ? 1 : n) != 0) throw std::bad_alloc();
    allocations.fetch_add(1, std::memory_order_relaxed);
    return p;
}
}  // namespace

void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) { return allocate_aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return allocate_aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::rns;
namespace original = rns_object_geometry_original;

int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

constexpr float kScale = 0.001F;

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

OpticalRayTable pinhole(std::uint32_t width, std::uint32_t height, double focal, float factor = 1.0F) {
    OpticalRayTable t;
    t.width = width; t.height = height;
    t.x.resize(static_cast<std::size_t>(width) * height); t.y.resize(t.x.size());
    for (std::uint32_t v = 0; v < height; ++v)
        for (std::uint32_t u = 0; u < width; ++u) {
            t.x[static_cast<std::size_t>(v) * width + u] = static_cast<float>((u - (width - 1) / 2.0) / focal) * factor;
            t.y[static_cast<std::size_t>(v) * width + u] = static_cast<float>((v - (height - 1) / 2.0) / focal) * factor;
        }
    return t;
}
// base (x fwd, y left, z up) <- colour optical (x right, y down, z fwd), camera at height h
// pitched down by `pitch` and shifted laterally by `ty`.
RigidTransform camera(double h, double pitch, double ty) {
    const double s = std::sin(pitch), c = std::cos(pitch);
    RigidTransform T;
    T.R = {0, -s, c, -1, 0, 0, 0, -c, -s};
    T.t = {0.1, ty, h};
    return T;
}

// One build() call. The buffers are owned by the Set the call belongs to.
struct Call {
    const std::uint16_t* depth = nullptr;
    std::uint32_t width = 0, height = 0;
    float scale = kScale;
    int bx = 0, by = 0, bw = 0, bh = 0;
    const std::uint8_t* mask = nullptr;
    const std::uint32_t* selected = nullptr;
    std::size_t selected_count = 0;
    float lo = 0, hi = 0;
    DepthRegion region() const {
        DepthRegion r;
        r.depth = depth; r.width = width; r.height = height; r.depth_scale = scale;
        r.bbox_x = bx; r.bbox_y = by; r.bbox_width = bw; r.bbox_height = bh;
        r.mask = mask; r.selected = selected; r.selected_count = selected_count;
        return r;
    }
};

// The list DepthSampler hands over: the region's pixels whose depth lies in the band, row-major.
std::vector<std::uint32_t> listed(const Call& c) {
    std::vector<std::uint32_t> out;
    int x0 = std::max(0, c.bx), y0 = std::max(0, c.by);
    int x1 = std::min(static_cast<int>(c.width), c.bx + c.bw), y1 = std::min(static_cast<int>(c.height), c.by + c.bh);
    if (!c.mask) {
        const int w = x1 - x0, h = y1 - y0, mx = w / 4, my = h / 4;
        x0 += mx; y0 += my; x1 = x0 + (w - 2 * mx); y1 = y0 + (h - 2 * my);
    }
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) {
            if (c.mask && !c.mask[static_cast<std::size_t>(y - c.by) * c.bw + (x - c.bx)]) continue;
            const float d = static_cast<float>(c.depth[static_cast<std::size_t>(y) * c.width + x]) * c.scale;
            if (d >= c.lo && d <= c.hi) out.push_back(static_cast<std::uint32_t>(y) * c.width + static_cast<std::uint32_t>(x));
        }
    return out;
}

// One pair of builders (production and original) with the calls made on both.
struct Set {
    const char* name;
    OpticalRayTable rays;
    RigidTransform transform;
    ObjectGeometryOptions options;
    std::vector<std::unique_ptr<std::vector<std::uint16_t>>> depths;
    std::vector<std::unique_ptr<std::vector<std::uint8_t>>> masks;
    std::vector<std::unique_ptr<std::vector<std::uint32_t>>> lists;
    std::vector<Call> calls;

    std::vector<std::uint16_t>& new_depth() {
        depths.push_back(std::make_unique<std::vector<std::uint16_t>>(static_cast<std::size_t>(rays.width) * rays.height, 0));
        return *depths.back();
    }
    Call base(const std::vector<std::uint16_t>& depth, int bx, int by, int bw, int bh, float lo, float hi) const {
        Call c;
        c.depth = depth.data(); c.width = rays.width; c.height = rays.height;
        c.bx = bx; c.by = by; c.bw = bw; c.bh = bh; c.lo = lo; c.hi = hi;
        return c;
    }
    // The call as a region scan and, when the band selects something, again with the sampler's list.
    void add(Call c) {
        calls.push_back(c);
        auto list = listed(c);
        if (list.empty()) return;
        lists.push_back(std::make_unique<std::vector<std::uint32_t>>(std::move(list)));
        c.selected = lists.back()->data();
        c.selected_count = lists.back()->size();
        calls.push_back(c);
    }
    const std::uint8_t* new_mask(std::size_t size, Random& random, unsigned keep_percent) {
        masks.push_back(std::make_unique<std::vector<std::uint8_t>>(size));
        for (auto& m : *masks.back()) m = random.below(100) < keep_percent ? 1 : 0;
        return masks.back()->data();
    }
};

// Bit-identical results: every field, the footprint and z interval by their bits.
bool same(const ObjectGeometryResult& a, const ObjectGeometryResult& b) {
    bool ok = a.status == b.status && a.points == b.points && a.hull_vertices == b.hull_vertices &&
              a.degenerate == b.degenerate && a.decimated == b.decimated &&
              a.rectangle_fallback == b.rectangle_fallback && a.used_selected == b.used_selected &&
              a.selected_rejected == b.selected_rejected && a.geometry.has_value() == b.geometry.has_value();
    if (ok && a.geometry) {
        const auto& fa = a.geometry->footprint_xy;
        const auto& fb = b.geometry->footprint_xy;
        ok = fa.size() == fb.size() &&
             (fa.empty() || std::memcmp(fa.data(), fb.data(), fa.size() * sizeof(Point)) == 0) &&
             std::memcmp(&a.geometry->z_min, &b.geometry->z_min, sizeof(float)) == 0 &&
             std::memcmp(&a.geometry->z_max, &b.geometry->z_max, sizeof(float)) == 0;
    }
    return ok;
}

// Noisy rectangles of random size and place on a far background, masked and unmasked, several bands.
void add_random_objects(Set& set, Random& random, int scenes, std::uint16_t near_code, std::uint16_t spread) {
    const int W = static_cast<int>(set.rays.width), H = static_cast<int>(set.rays.height);
    for (int scene = 0; scene < scenes; ++scene) {
        auto& depth = set.new_depth();
        for (auto& c : depth) c = static_cast<std::uint16_t>(3 * near_code + random.below(spread));
        const int bw = 4 + static_cast<int>(random.below(static_cast<std::size_t>(W / 3)));
        const int bh = 4 + static_cast<int>(random.below(static_cast<std::size_t>(H / 3)));
        const int bx = static_cast<int>(random.below(static_cast<std::size_t>(W))) - bw / 4;
        const int by = static_cast<int>(random.below(static_cast<std::size_t>(H))) - bh / 4;
        const bool flat = scene % 5 == 4;
        for (int y = std::max(0, by); y < std::min(H, by + bh); ++y)
            for (int x = std::max(0, bx); x < std::min(W, bx + bw); ++x)
                depth[static_cast<std::size_t>(y) * W + x] =
                    static_cast<std::uint16_t>(flat ? near_code + spread / 3 : near_code + random.below(spread));
        const auto* mask = set.new_mask(static_cast<std::size_t>(bw) * bh, random, 20 + static_cast<unsigned>(random.below(80)));
        const float a = static_cast<float>(near_code) * kScale, s = static_cast<float>(spread) * kScale;
        const std::array<std::array<float, 2>, 5> bands = {{{a, a + s}, {a + 0.2F * s, a + 0.4F * s},
            {a + s / 3, a + s / 3}, {a + 0.97F * s, a + s}, {0.25F * a, 0.5F * a}}};
        for (const auto& band : bands) {
            Call c = set.base(depth, bx, by, bw, bh, band[0], band[1]);
            c.mask = mask;
            set.add(c);
            c.mask = nullptr;
            set.add(c);
        }
    }
}

// A filled disc of the given radius (pixels) at constant depth plus noise: a round object whose
// hull has far more than 12 vertices.
void add_disc(Set& set, Random& random, int cu, int cv, int radius, std::uint16_t code, std::uint16_t noise) {
    const int W = static_cast<int>(set.rays.width), H = static_cast<int>(set.rays.height);
    auto& depth = set.new_depth();
    const int bx = cu - radius, by = cv - radius, size = 2 * radius + 1;
    set.masks.push_back(std::make_unique<std::vector<std::uint8_t>>(static_cast<std::size_t>(size) * size, 0));
    auto& mask = *set.masks.back();
    for (int y = std::max(0, by); y < std::min(H, by + size); ++y)
        for (int x = std::max(0, bx); x < std::min(W, bx + size); ++x) {
            if ((x - cu) * (x - cu) + (y - cv) * (y - cv) > radius * radius) continue;
            depth[static_cast<std::size_t>(y) * W + x] = static_cast<std::uint16_t>(code + (noise ? random.below(noise) : 0));
            mask[static_cast<std::size_t>(y - by) * size + (x - bx)] = 1;
        }
    Call c = set.base(depth, bx, by, size, size, static_cast<float>(code) * kScale, static_cast<float>(code + noise) * kScale);
    c.mask = mask.data();
    set.add(c);
    c.hi = c.lo + static_cast<float>(noise) * kScale / 4;
    set.add(c);
}

// A single column, a single row, one pixel and two pixels of constant depth.
void add_lines(Set& set, std::uint16_t code) {
    const int W = static_cast<int>(set.rays.width), H = static_cast<int>(set.rays.height);
    auto& depth = set.new_depth();
    const float d = static_cast<float>(code) * kScale;
    for (int v = H / 2; v < H / 2 + 10; ++v) depth[static_cast<std::size_t>(v) * W + W / 2] = code;
    for (int u = W / 4; u < W / 4 + 12; ++u) depth[static_cast<std::size_t>(H / 4) * W + u] = code;
    depth[static_cast<std::size_t>(H / 8) * W + W / 8] = code;
    depth[static_cast<std::size_t>(H / 8) * W + W / 8 + 2] = code;
    static const std::vector<std::uint8_t> ones(4096, 1);
    for (const auto& box : std::vector<std::array<int, 4>>{
             {W / 2, H / 2, 1, 10}, {W / 4, H / 4, 12, 1}, {W / 8, H / 8, 1, 1}, {W / 8, H / 8, 3, 1},
             {W / 2, H / 2, 1, 3}, {W / 4, H / 4, 5, 1}}) {
        Call c = set.base(depth, box[0], box[1], box[2], box[3], d, d);
        c.mask = ones.data();
        set.add(c);
    }
}

// Calls build() refuses or cannot locate.
void add_refused(Set& set) {
    auto& depth = set.new_depth();
    std::fill(depth.begin(), depth.end(), static_cast<std::uint16_t>(2000));
    const int W = static_cast<int>(set.rays.width);
    Call c = set.base(depth, 10, 10, 20, 20, 2.5F, 1.0F);      // lo > hi
    set.calls.push_back(c);
    c = set.base(depth, 10, 10, 20, 20, 1.0F, 3.0F);
    c.width = static_cast<std::uint32_t>(W / 2);               // not the table's size
    set.calls.push_back(c);
    c = set.base(depth, 5 * W, 10, 20, 20, 1.0F, 3.0F);        // box outside the image
    set.calls.push_back(c);
    c = set.base(depth, 10, 10, 20, 20, 1.0F, 3.0F);
    c.scale = 0;                                               // no depth scale
    set.calls.push_back(c);
    set.calls.push_back(set.base(depth, 10, 10, 20, 20, 7.0F, 8.0F));  // nothing in the band
    // A list that is out of order: refused, the region is scanned instead.
    set.lists.push_back(std::make_unique<std::vector<std::uint32_t>>(
        std::vector<std::uint32_t>{static_cast<std::uint32_t>(11 * W + 12), static_cast<std::uint32_t>(11 * W + 11)}));
    c = set.base(depth, 10, 10, 20, 20, 1.0F, 3.0F);
    static const std::vector<std::uint8_t> ones(400, 1);
    c.mask = ones.data();
    c.selected = set.lists.back()->data();
    c.selected_count = 2;
    set.calls.push_back(c);
}

struct Totals {
    std::size_t builds = 0, located = 0, decimated = 0, degenerate = 0, fallback = 0, too_few = 0,
                non_finite = 0, invalid = 0, used_list = 0, rejected_list = 0, mismatches = 0;
    std::uint64_t steady_builds = 0, steady_located = 0, steady_allocations = 0, original_allocations = 0,
                  steady_wrong = 0;
};

void run(Set& set, Totals& totals, std::uint64_t seed) {
    ObjectGeometryBuilder production(set.rays, set.transform, set.options);
    original::ObjectGeometryBuilder reference(set.rays, set.transform, set.options);
    const std::size_t n = set.calls.size();
    Totals mine;
    // Results kept from earlier builds with a private copy of their vertices: a later build must
    // not change them.
    std::vector<ObjectGeometryResult> kept;
    std::vector<std::vector<Point>> kept_vertices;
    const auto step = [&](std::size_t index, bool keep) {
        const Call& call = set.calls[index];
        const auto a = production.build(call.region(), call.lo, call.hi);
        const auto b = reference.build(call.region(), call.lo, call.hi);
        ++mine.builds;
        mine.mismatches += !same(a, b);
        mine.located += a.status == ObjectGeometryStatus::Located;
        mine.too_few += a.status == ObjectGeometryStatus::TooFewPoints;
        mine.non_finite += a.status == ObjectGeometryStatus::NonFinite;
        mine.invalid += a.status == ObjectGeometryStatus::InvalidInput;
        mine.decimated += a.decimated; mine.degenerate += a.degenerate; mine.fallback += a.rectangle_fallback;
        mine.used_list += a.used_selected; mine.rejected_list += a.selected_rejected;
        if (keep && a.geometry) {
            kept_vertices.push_back(a.geometry->footprint_xy);
            kept.push_back(a);
        }
    };
    Random random{seed};
    // In order, in reverse, then at random: every kind of build follows every other kind.
    for (std::size_t i = 0; i < n; ++i) step(i, i % 5 == 0);
    for (std::size_t i = n; i-- > 0;) step(i, false);
    for (std::size_t i = 0; i < 6 * n; ++i) step(random.below(n), i % 11 == 0);
    std::size_t changed = 0;
    for (std::size_t i = 0; i < kept.size(); ++i) {
        const auto& now = kept[i].geometry->footprint_xy;
        changed += now.size() != kept_vertices[i].size() ||
                   std::memcmp(now.data(), kept_vertices[i].data(), now.size() * sizeof(Point)) != 0;
    }
    CHECK(mine.mismatches == 0);
    CHECK(changed == 0 && kept.size() > 0);

    // Allocation counts. Every call has now been made several times on both builders, so their
    // scratch has reached the largest object: from here on a Located build allocates once and
    // any other build not at all. The original's count for the same calls is only reported.
    for (std::size_t i = 0; i < 2 * n; ++i) {
        const Call& call = set.calls[i < n ? i : random.below(n)];
        const DepthRegion region = call.region();
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto a = production.build(region, call.lo, call.hi);
        const auto middle = allocations.load(std::memory_order_relaxed);
        const auto b = reference.build(region, call.lo, call.hi);
        const auto after = allocations.load(std::memory_order_relaxed);
        const bool located = a.status == ObjectGeometryStatus::Located;
        ++mine.steady_builds;
        mine.steady_located += located;
        mine.steady_allocations += middle - before;
        mine.original_allocations += after - middle;
        mine.steady_wrong += (middle - before) != (located ? 1U : 0U);
        mine.mismatches += !same(a, b);
    }
    CHECK(mine.mismatches == 0);
    CHECK(mine.steady_wrong == 0);
    CHECK(mine.steady_allocations == mine.steady_located);
    CHECK(mine.original_allocations > mine.steady_allocations);
    std::cout << "set " << set.name << ": calls=" << n << " builds=" << mine.builds << " located=" << mine.located
              << " decimated=" << mine.decimated << " degenerate=" << mine.degenerate << " fallback=" << mine.fallback
              << " too_few=" << mine.too_few << " non_finite=" << mine.non_finite << " invalid=" << mine.invalid
              << " used_list=" << mine.used_list << " rejected_list=" << mine.rejected_list
              << " mismatches=" << mine.mismatches << " | steady builds=" << mine.steady_builds
              << " located=" << mine.steady_located << " allocations=" << mine.steady_allocations
              << " original_allocations=" << mine.original_allocations << '\n';
    totals.builds += mine.builds; totals.located += mine.located; totals.decimated += mine.decimated;
    totals.degenerate += mine.degenerate; totals.fallback += mine.fallback; totals.too_few += mine.too_few;
    totals.non_finite += mine.non_finite; totals.invalid += mine.invalid; totals.used_list += mine.used_list;
    totals.rejected_list += mine.rejected_list; totals.mismatches += mine.mismatches;
    totals.steady_builds += mine.steady_builds; totals.steady_located += mine.steady_located;
    totals.steady_allocations += mine.steady_allocations; totals.original_allocations += mine.original_allocations;
    totals.steady_wrong += mine.steady_wrong;
}
}  // namespace

int main() {
    Totals totals;
    Random random{0x0b1ec7a11ce5ULL};
    {
        // The service's shape: 1280 x 800 colour rays, pitched camera, five points to locate.
        Set set{"pitched_1280x800", pinhole(1280, 800, 600), camera(0.8, 12.0 * M_PI / 180.0, 0.25), {5, 0.01, 0.02}, {}, {}, {}, {}};
        add_random_objects(set, random, 10, 1500, 1500);
        add_disc(set, random, 640, 400, 150, 2500, 400);
        add_disc(set, random, 300, 600, 60, 1200, 60);
        add_disc(set, random, 1000, 200, 220, 4000, 1500);
        add_lines(set, 2000);
        add_refused(set);
        run(set, totals, 11);
    }
    {
        // Camera looking straight along base x: a pixel column or row of one depth is a line
        // parallel to an axis (degenerate hulls); one point is enough to locate.
        RigidTransform down;
        down.R = {0, 0, 1, -1, 0, 0, 0, -1, 0};
        down.t = {0, 0, 1};
        Set set{"level_640x400_one_point", pinhole(640, 400, 300), down, {1, 0.01, 0.02}, {}, {}, {}, {}};
        add_lines(set, 2000);
        add_random_objects(set, random, 6, 900, 2400);
        add_disc(set, random, 320, 200, 120, 3000, 0);
        add_lines(set, 5000);
        add_disc(set, random, 100, 100, 40, 800, 30);
        add_refused(set);
        run(set, totals, 12);
    }
    {
        // Coarse buckets and a wide degenerate margin.
        Set set{"pitched_640x400_coarse", pinhole(640, 400, 300), camera(1.1, -0.2, -0.4), {3, 0.25, 0.1}, {}, {}, {}, {}};
        add_random_objects(set, random, 6, 2000, 3000);
        add_lines(set, 2500);
        add_disc(set, random, 320, 200, 150, 3000, 900);
        add_refused(set);
        run(set, totals, 13);
    }
    {
        // Every 16-bit code over a 256 x 256 table at one metre per code: tens of metres of
        // extent, so the bucket count reaches its bound and the scratch must grow past its
        // first reservation.
        Set set{"codes_256x256_far", pinhole(256, 256, 200), camera(0.8, 0.2, 0.25), {1, 0.01, 0.02}, {}, {}, {}, {}};
        auto& codes = set.new_depth();
        for (std::size_t i = 0; i < codes.size(); ++i) codes[i] = static_cast<std::uint16_t>(i);
        set.masks.push_back(std::make_unique<std::vector<std::uint8_t>>(codes.size(), 255));
        for (const auto& band : std::vector<std::array<float, 2>>{
                 {0.0F, 70.0F}, {1.0F, 2.0F}, {10.0F, 40.0F}, {0.5F, 0.6F}, {64.0F, 66.0F}, {20.0F, 20.5F}}) {
            Call c = set.base(codes, 0, 0, 256, 256, band[0], band[1]);
            c.mask = set.masks.back()->data();
            set.add(c);
            c.mask = nullptr;
            set.add(c);
        }
        add_random_objects(set, random, 3, 1500, 1500);
        add_refused(set);
        run(set, totals, 14);
    }
    {
        // Huge but finite coordinates: non-finite results and the rectangle fallbacks.
        Set set{"codes_256x256_huge", pinhole(256, 256, 200, 1e15F), camera(0.8, 0.2, 0.25), {1, 0.01, 0.02}, {}, {}, {}, {}};
        auto& codes = set.new_depth();
        for (std::size_t i = 0; i < codes.size(); ++i) codes[i] = static_cast<std::uint16_t>(i);
        for (const float scale : {1.0F, 1e-3F, 1e-9F, 1e-14F, 1e-16F, 1e20F}) {
            for (const auto& band : std::vector<std::array<float, 2>>{{0.0F, 1e30F}, {1000.0F, 20000.0F}}) {
                Call c = set.base(codes, 0, 0, 256, 256, band[0] * scale, band[1] * scale);
                c.scale = scale;
                set.calls.push_back(c);
                c = set.base(codes, 100, 100, 9, 7, band[0] * scale, band[1] * scale);
                c.scale = scale;
                set.calls.push_back(c);
            }
        }
        add_refused(set);
        run(set, totals, 15);
    }
    // Every kind of build took part.
    CHECK(totals.located > 1000 && totals.decimated > 100 && totals.degenerate > 50);
    CHECK(totals.too_few > 100 && totals.invalid > 50 && totals.used_list > 500 && totals.rejected_list > 10);
    CHECK(totals.fallback + totals.non_finite > 0);
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"object_geometry_scratch_host\",\"builds\":" << totals.builds
              << ",\"located\":" << totals.located << ",\"decimated\":" << totals.decimated
              << ",\"degenerate\":" << totals.degenerate << ",\"rectangle_fallback\":" << totals.fallback
              << ",\"too_few\":" << totals.too_few << ",\"non_finite\":" << totals.non_finite
              << ",\"invalid\":" << totals.invalid << ",\"used_list\":" << totals.used_list
              << ",\"rejected_list\":" << totals.rejected_list << ",\"mismatches\":" << totals.mismatches
              << ",\"steady_builds\":" << totals.steady_builds << ",\"steady_located\":" << totals.steady_located
              << ",\"steady_allocations\":" << totals.steady_allocations
              << ",\"original_allocations\":" << totals.original_allocations << "}" << std::endl;
    return failures ? 1 : 0;
}
