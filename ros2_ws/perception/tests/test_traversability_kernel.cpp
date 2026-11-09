/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_traversability_kernel.cpp
 * Brief: Unit tests for the raw traversability (T) kernel: thresholds, radii, aging and domains.
 *
 * Description:
 * What it checks. The T kernel turns raw segmentation evidence into per-bin traversability with
 * padding, angular radii and aging. Cases cover the raw threshold and padding, radii and the
 * unknown domain, rotation aging with negative controls, outward float bounds (an independently
 * specified regression where the rounded quotient is 3), invalid and empty inputs, a random brute-
 * force oracle and the prefix restriction.
 * Traps. The accepted radius is checked against an independent integer search rather than the same
 * ceil/division; positive motion must not underflow into a zero-margin identity; the real kernel
 * output at the near edge stays fresh=false, never no-T.
 */

#include "modules/traversability.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>
#include <vector>

namespace {
bool count_allocations = false;
std::size_t allocations = 0, erosion_calls = 0, restriction_calls = 0, margin_calls = 0;
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
    TLayout layout{9, 7, 8, 1.0, 0.25, 0.2};
    TOptions options{0.75, 0.1, 0.1, 0, INT64_C(2000000000)};
    std::vector<TCount> counts;
    std::vector<std::uint8_t> raw, eroded;
    std::vector<std::uint32_t> prefix;
    explicit Fixture(std::size_t capacity = 80, std::size_t scratch = 100)
        : counts(capacity, {4, 3}), raw(capacity, 0xa5), eroded(capacity, 0xa5), prefix(scratch, 0xffffffffU) {}
};

bool invoke(const TLayout& layout, const TOptions& options, std::int64_t age,
            const TCount* counts, std::size_t capacity, std::uint8_t* raw,
            std::uint8_t* eroded, std::uint32_t* prefix, std::size_t prefix_capacity,
            bool valid_buffers = true) {
    const auto before = allocations;
    count_allocations = true;
    const bool result = erode_traversability(layout, options, age, counts, capacity, raw, eroded, prefix, prefix_capacity);
    count_allocations = false;
    ++erosion_calls;
    CHECK(allocations == before);
    if (!result && valid_buffers && capacity <= kMaxGridCells) {
        for (std::size_t i = 0; i < capacity; ++i) {
            if (raw) CHECK(raw[i] == 0);
            if (eroded) CHECK(eroded[i] == 0);
        }
    }
    return result;
}

bool invoke(Fixture& fixture, std::int64_t age = 0) {
    return invoke(fixture.layout, fixture.options, age, fixture.counts.data(), fixture.counts.size(),
                  fixture.raw.data(), fixture.eroded.data(), fixture.prefix.data(), fixture.prefix.size());
}

double margin(const TOptions& options, std::int64_t age, double far) {
    const auto before = allocations;
    count_allocations = true;
    const double result = t_margin_upper_bound(options, age, far);
    count_allocations = false;
    ++margin_calls;
    CHECK(allocations == before);
    return result;
}

double oracle_margin(const TOptions& options, std::int64_t age, double far) {
    if (age == 0) return options.jitter_m;
    const double inf = std::numeric_limits<double>::infinity();
    const double seconds = std::nextafter(std::nextafter(static_cast<double>(age), inf) / 1e9, inf);
    double rotation = 0;
    if (options.omega_max_rps != 0 && far != 0) rotation = std::nextafter(options.omega_max_rps * far, inf);
    double speed = options.v_max_mps == 0 ? rotation : options.v_max_mps;
    if (options.v_max_mps != 0 && rotation != 0) speed = std::nextafter(options.v_max_mps + rotation, inf);
    double travel = speed == 0 ? 0 : std::nextafter(speed * seconds, inf);
    if (travel == 0) return options.jitter_m;
    if (options.jitter_m == 0) return travel;
    return std::nextafter(travel + options.jitter_m, inf);
}

std::size_t oracle_radius(double numerator, double spacing, std::size_t limit) {
    // Linear integer search is deliberately independent of ceil/division.
    // Every accepted radius must actually cover the exact double numerator.
    for (std::size_t radius = 0; radius < limit; ++radius) {
        if (std::fma(-static_cast<double>(radius), spacing, numerator) <= 0.0) return radius;
    }
    return limit;
}

// Independent pool oracle: inspect every neighbour directly, never consult
// prefix sums. Small explicit kernel cases below also discriminate formulas.
void check_brute(const Fixture& fixture, std::int64_t age) {
    const auto& layout = fixture.layout;
    for (std::size_t i = 0; i < fixture.counts.size(); ++i) {
        const std::size_t bin = layout.stride == 0 ? 0 : i / layout.stride;
        const std::size_t cell = layout.stride == 0 ? 0 : i % layout.stride;
        if (layout.stride == 0 || bin >= layout.bins || cell >= layout.cells) {
            CHECK(fixture.raw[i] == 0 && fixture.eroded[i] == 0);
            continue;
        }
        const auto raw_at = [&](std::size_t b, std::size_t k) {
            const auto value = fixture.counts[b * layout.stride + k];
            return value.ground != 0 && static_cast<double>(value.traversable) /
                   static_cast<double>(value.ground) >= fixture.options.tau;
        };
        CHECK(fixture.raw[i] == static_cast<std::uint8_t>(raw_at(bin, cell)));
        const double near = layout.blind_m + static_cast<double>(cell) * layout.dr_m;
        const double far = layout.blind_m + static_cast<double>(cell + 1) * layout.dr_m;
        const double margin = oracle_margin(fixture.options, age, far);
        bool expected = true;
        std::size_t radial = 0, angular = 0;
        if (margin > 0) {
            if (margin >= near) expected = false;
            else {
                const double inf = std::numeric_limits<double>::infinity();
                const double ratio = std::nextafter(margin / near, inf);
                const double angle = std::nextafter(1.5707963267948968 * ratio, inf);
                radial = oracle_radius(margin, layout.dr_m, layout.cells);
                angular = oracle_radius(angle, layout.angle_step_rad, layout.bins);
                if (radial >= layout.cells || angular >= layout.bins) expected = false;
            }
        }
        if (expected && (radial > cell || radial >= layout.cells - cell ||
                         angular > bin || angular >= layout.bins - bin)) expected = false;
        if (expected) {
            for (std::size_t b = bin - angular; b <= bin + angular; ++b) {
                for (std::size_t k = cell - radial; k <= cell + radial; ++k) {
                    if (!raw_at(b, k)) expected = false;
                }
            }
        }
        CHECK(fixture.eroded[i] == static_cast<std::uint8_t>(expected));
    }
}

// A cell is raw-passable when its passable share reaches tau (inclusive, checked at +-1 ulp);
// empty cells and row padding never count.
void test_raw_threshold_and_padding() {
    Fixture fixture;
    fixture.counts[0] = {0, 0};
    fixture.counts[1] = {4, 2};
    fixture.counts[2] = {4, 3};
    fixture.counts[3] = {static_cast<std::uint32_t>(kGeometryPixels), static_cast<std::uint32_t>(kGeometryPixels)};
    fixture.counts[7] = {0, 99}; // Padding is not active evidence.
    CHECK(invoke(fixture));
    check_brute(fixture, 0);
    CHECK(fixture.raw[0] == 0 && fixture.raw[1] == 0 && fixture.raw[2] == 1 && fixture.raw[3] == 1);
    fixture.options.tau = std::nextafter(0.75, 1.0);
    CHECK(invoke(fixture));
    CHECK(fixture.raw[2] == 0);
    fixture.options.tau = std::nextafter(0.75, 0.0);
    CHECK(invoke(fixture));
    CHECK(fixture.raw[2] == 1);
    fixture.options.tau = 1;
    CHECK(invoke(fixture));
    CHECK(fixture.raw[2] == 0 && fixture.raw[3] == 1);
    for (auto& value : fixture.counts) value = {0, 0};
    CHECK(invoke(fixture));
    CHECK(std::all_of(fixture.raw.begin(), fixture.raw.end(), [](auto value) { return value == 0; }));
    CHECK(std::all_of(fixture.eroded.begin(), fixture.eroded.end(), [](auto value) { return value == 0; }));
    CHECK(fixture.prefix[(fixture.layout.bins + 1) * (fixture.layout.stride + 1) - 1] == 0);
}

// Erosion radii follow the margin exactly at their limits (radial, angular and the outward pi/2
// envelope); neighbours outside the grid count as not passable; zero margin changes nothing.
void test_radii_and_unknown_domain() {
    Fixture fixture;
    fixture.options.jitter_m = 0.25;
    CHECK(invoke(fixture));
    check_brute(fixture, 0);
    CHECK(fixture.eroded[4 * 8 + 1] == 1);
    for (std::size_t bin = 0; bin < fixture.layout.bins; ++bin) {
        CHECK(fixture.eroded[bin * 8] == 0);
        CHECK(fixture.eroded[bin * 8 + 6] == 0);
    }
    fixture.options.jitter_m = std::nextafter(0.25, 1.0);
    CHECK(invoke(fixture));
    check_brute(fixture, 0);
    CHECK(fixture.eroded[4 * 8 + 1] == 0);
    fixture.options.jitter_m = std::nextafter(0.25, 0.0);
    CHECK(invoke(fixture));
    CHECK(fixture.eroded[4 * 8 + 1] == 1);

    // The outward pi/2 envelope requires four angular neighbours here; asin
    // alone would require three and the former centre/arc rule only two.
    Fixture envelope;
    envelope.layout = {9, 3, 3, 1.0, 1.0, 0.2};
    envelope.options.jitter_m = 1.0;
    CHECK(invoke(envelope));
    check_brute(envelope, 0);
    CHECK(envelope.eroded[4 * 3 + 1] == 1);
    CHECK(envelope.eroded[3 * 3 + 1] == 0);
    envelope.counts[1] = {4, 0};
    CHECK(invoke(envelope));
    CHECK(envelope.eroded[4 * 3 + 1] == 0);

    // Hold margin fixed and move angular step across the exact ceil boundary.
    Fixture angular;
    angular.options.jitter_m = 0.125;
    const double inf = std::numeric_limits<double>::infinity();
    const double quotient = std::nextafter(1.5707963267948968 *
        std::nextafter(0.125 / (1.0 + 2.0 * 0.25), inf), inf);
    angular.layout.angle_step_rad = quotient;
    CHECK(invoke(angular));
    CHECK(angular.eroded[1 * 8 + 2] == 1);
    angular.layout.angle_step_rad = std::nextafter(quotient, 0.0);
    CHECK(invoke(angular));
    CHECK(angular.eroded[1 * 8 + 2] == 0);
    angular.layout.angle_step_rad = std::nextafter(quotient, 1.0);
    CHECK(invoke(angular));
    CHECK(angular.eroded[1 * 8 + 2] == 1);

    Fixture origin;
    origin.layout = {5, 5, 5, 0, 1, 1};
    CHECK(invoke(origin));
    CHECK(origin.eroded[0] == 1); // Zero margin is identity even at near=0.
    origin.options.jitter_m = 2;
    CHECK(invoke(origin));
    CHECK(origin.eroded[2 * 5 + 2] == 0); // margin == near must reject.
    origin.options.jitter_m = std::numeric_limits<double>::denorm_min();
    CHECK(invoke(origin));
    CHECK(origin.eroded[0] == 0 && origin.eroded[2 * 5 + 2] == 1);
}

// An older mask or a higher speed or turn rate only ever removes passable cells; rotation needs a
// wider radius farther out; huge speeds at age 0 do not overflow.
void test_rotation_aging_and_negative_controls() {
    Fixture fixture;
    fixture.options.v_max_mps = 0;
    fixture.options.omega_max_rps = 0.2;
    CHECK(invoke(fixture, INT64_C(1000000000)));
    check_brute(fixture, INT64_C(1000000000));
    CHECK(fixture.eroded[4 * 8 + 1] == 0); // far-dependent radial radius > 1.
    CHECK(fixture.eroded[4 * 8 + 2] == 1);
    fixture.options.omega_max_rps = 0;
    CHECK(invoke(fixture, INT64_C(1000000000)));
    CHECK(fixture.eroded[4 * 8 + 1] == 1);

    fixture.options.v_max_mps = 0.1;
    fixture.options.omega_max_rps = 0.05;
    CHECK(invoke(fixture, 0));
    auto previous = fixture.eroded;
    for (std::int64_t age = 100000000; age <= 2000000000; age += 100000000) {
        CHECK(invoke(fixture, age));
        check_brute(fixture, age);
        for (std::size_t i = 0; i < previous.size(); ++i) CHECK(fixture.eroded[i] <= previous[i]);
        previous = fixture.eroded;
    }
    fixture.options.v_max_mps = 0;
    fixture.options.omega_max_rps = 0;
    CHECK(invoke(fixture, INT64_C(1000000000)));
    previous = fixture.eroded;
    for (int speed = 1; speed <= 10; ++speed) {
        fixture.options.v_max_mps = static_cast<double>(speed) * 0.05;
        fixture.options.omega_max_rps = static_cast<double>(speed) * 0.025;
        CHECK(invoke(fixture, INT64_C(1000000000)));
        for (std::size_t i = 0; i < previous.size(); ++i) CHECK(fixture.eroded[i] <= previous[i]);
        previous = fixture.eroded;
    }
    fixture.options.v_max_mps = fixture.options.omega_max_rps = std::numeric_limits<double>::max();
    CHECK(invoke(fixture, 0));
    CHECK(fixture.eroded[0] == 1); // No overflow-times-zero accident.
    CHECK(invoke(fixture, 1));
    CHECK(std::all_of(fixture.eroded.begin(), fixture.eroded.end(), [](auto value) { return value == 0; }));
}

// Radii round outward even where the float quotient rounds in (0.9/0.3 gives 3 but needs 4);
// positive motion never underflows to a zero margin; invalid ages or ranges give an infinite
// margin.
void test_outward_float_bounds() {
    // Independently specified regression: the rounded quotient is 3 although
    // the exact double .9 lies beyond three exact double .3 spacings.
    CHECK(std::ceil(0.9 / 0.3) == 3.0);
    CHECK(std::fma(-3.0, 0.3, 0.9) > 0.0);
    Fixture radial;
    radial.layout = {9, 7, 8, 1.0, 0.3, 0.5};
    radial.options.jitter_m = 0.9;
    CHECK(invoke(radial));
    CHECK(radial.raw[4 * 8 + 3] == 1);
    CHECK(radial.eroded[4 * 8 + 3] == 0); // Needs radius 4; near-side halo is absent.
    check_brute(radial, 0);
    radial.options.jitter_m = std::nextafter(0.9, 0.0);
    CHECK(std::fma(-3.0, 0.3, radial.options.jitter_m) <= 0.0);
    CHECK(invoke(radial));
    CHECK(radial.eroded[4 * 8 + 3] == 1); // Radius 3 has a complete 7-cell neighbourhood.

    // Positive motion may not underflow into a zero-margin identity operation.
    Fixture tiny_motion;
    tiny_motion.options.v_max_mps = std::numeric_limits<double>::denorm_min();
    tiny_motion.options.omega_max_rps = 0;
    tiny_motion.options.jitter_m = 0;
    CHECK(tiny_motion.options.v_max_mps * 1e-9 == 0.0);
    CHECK(margin(tiny_motion.options, 1, 2.0) > 0.0);
    CHECK(invoke(tiny_motion, 1));
    CHECK(tiny_motion.raw[0] == 1 && tiny_motion.eroded[0] == 0);
    CHECK(tiny_motion.eroded[4 * 8 + 2] == 1);
    check_brute(tiny_motion, 1);
    tiny_motion.options.v_max_mps = 0;
    CHECK(margin(tiny_motion.options, 1, 2.0) == 0.0);
    CHECK(invoke(tiny_motion, 1));
    CHECK(tiny_motion.eroded[0] == 1);

    TOptions options{0.75, 0.5, 0.25, 0.125, INT64_C(2000000000)};
    CHECK(margin(options, INT64_C(1000000000), 2.0) > 1.125);
    CHECK(margin(options, 0, 2.0) == 0.125);
    options.max_age_ns = std::numeric_limits<std::int64_t>::max();
    CHECK(margin(options, options.max_age_ns, 2.0) >= 9223372036.0);
    options.v_max_mps = options.omega_max_rps = std::numeric_limits<double>::max();
    CHECK(margin(options, 0, 2.0) == 0.125);
    CHECK(std::isinf(margin(options, 1, 2.0)));
    CHECK(std::isinf(margin(options, -1, 2.0)));
    options.max_age_ns = 10;
    CHECK(std::isinf(margin(options, 11, 2.0)));
    CHECK(std::isinf(margin(options, 0, -1.0)));
    CHECK(std::isinf(margin(options, 0, std::numeric_limits<double>::quiet_NaN())));
    options.v_max_mps = std::numeric_limits<double>::quiet_NaN();
    CHECK(std::isinf(margin(options, 0, 2.0)));
}

// Invalid options, layouts, counts, buffer sizes or aliasing are refused without touching any
// buffer; an empty grid and the maximum grid both work.
void test_invalid_and_empty() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<double TOptions::*, 4> fields{{&TOptions::tau, &TOptions::v_max_mps,
                                                  &TOptions::omega_max_rps, &TOptions::jitter_m}};
    for (const auto field : fields) {
        for (const double value : std::array<double, 4>{{nan, inf, -inf, -0.125}}) {
            Fixture fixture;
            fixture.options.*field = value;
            CHECK(!valid_t_options(fixture.options));
            CHECK(!invoke(fixture));
        }
    }
    Fixture fixture;
    fixture.options.tau = 0; CHECK(!invoke(fixture));
    fixture.options.tau = std::nextafter(1.0, 2.0); CHECK(!invoke(fixture));
    fixture.options.tau = 0.75;
    fixture.options.max_age_ns = -1; CHECK(!invoke(fixture));
    fixture.options.max_age_ns = 10;
    CHECK(invoke(fixture, 10));
    CHECK(!invoke(fixture, 11));
    CHECK(!invoke(fixture, -1));
    fixture.options.max_age_ns = 0;
    CHECK(invoke(fixture, 0));

    const auto original_layout = fixture.layout;
    fixture.layout.bins = kGeometryMaxBins + 1; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.stride = kMaxRadialCells + 1; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.cells = fixture.layout.stride + 1; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.blind_m = -0.1; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.blind_m = nan; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.dr_m = 0; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.dr_m = nan; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.angle_step_rad = 0; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.angle_step_rad = nan; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.dr_m = 1; CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.dr_m = std::numeric_limits<double>::denorm_min(); CHECK(!invoke(fixture));
    fixture.layout = original_layout; fixture.layout.bins = 1024; fixture.layout.stride = 256; CHECK(!invoke(fixture));
    fixture.layout = original_layout;
    fixture.counts[8 * 8 + 6] = {1, 2}; CHECK(!invoke(fixture));
    fixture.counts[8 * 8 + 6] = {static_cast<std::uint32_t>(kGeometryPixels + 1), 0}; CHECK(!invoke(fixture));
    fixture.counts[8 * 8 + 6] = {4, 3};
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), 71,
                  fixture.raw.data(), fixture.eroded.data(), fixture.prefix.data(), fixture.prefix.size()));
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), fixture.counts.size(),
                  fixture.raw.data(), fixture.eroded.data(), fixture.prefix.data(), 89));
    CHECK(!invoke(fixture.layout, fixture.options, 0, nullptr, fixture.counts.size(),
                  fixture.raw.data(), fixture.eroded.data(), fixture.prefix.data(), fixture.prefix.size(), false));
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), fixture.counts.size(),
                  nullptr, fixture.eroded.data(), fixture.prefix.data(), fixture.prefix.size(), false));
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), fixture.counts.size(),
                  fixture.raw.data(), fixture.eroded.data(), nullptr, fixture.prefix.size(), false));
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), fixture.counts.size(),
                  fixture.raw.data(), fixture.raw.data(), fixture.prefix.data(), fixture.prefix.size(), false));
    const auto counts_before_alias = fixture.counts;
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), fixture.counts.size(),
                  reinterpret_cast<std::uint8_t*>(fixture.counts.data()), fixture.eroded.data(),
                  fixture.prefix.data(), fixture.prefix.size(), false));
    for (std::size_t i = 0; i < fixture.counts.size(); ++i) {
        CHECK(fixture.counts[i].ground == counts_before_alias[i].ground);
        CHECK(fixture.counts[i].traversable == counts_before_alias[i].traversable);
    }
    // Over-limit capacity is an invalid allocation declaration, rejected untouched.
    fixture.raw[0] = fixture.eroded[0] = 0xa5;
    CHECK(!invoke(fixture.layout, fixture.options, 0, fixture.counts.data(), kMaxGridCells + 1,
                  fixture.raw.data(), fixture.eroded.data(), fixture.prefix.data(), fixture.prefix.size()));
    CHECK(fixture.raw[0] == 0xa5 && fixture.eroded[0] == 0xa5);

    Fixture empty;
    empty.layout.cells = 0;
    CHECK(invoke(empty));
    check_brute(empty, 0);
    empty.layout = {0, 0, 0, 0, 1, 1};
    CHECK(invoke(empty.layout, empty.options, 0, nullptr, 0, nullptr, nullptr, empty.prefix.data(), 1));
    CHECK(!invoke(empty.layout, empty.options, -1, nullptr, 0, nullptr, nullptr, empty.prefix.data(), 1));

    Fixture maximum(kMaxGridCells, (1024 + 1) * (64 + 1));
    maximum.layout = {1024, 64, 64, 0, 0.0625, 0.005};
    CHECK(invoke(maximum));
    check_brute(maximum, 0);
}

std::uint64_t random_word(std::uint64_t& state) {
    state ^= state << 13U;
    state ^= state >> 7U;
    state ^= state << 17U;
    return state;
}

// 200 random grids match a brute-force erosion oracle.
void test_random_oracle() {
    std::uint64_t state = UINT64_C(0x64b70af3582dc619);
    for (std::size_t trial = 0; trial < 200; ++trial) {
        Fixture fixture;
        fixture.layout.bins = 1 + random_word(state) % 9;
        fixture.layout.cells = random_word(state) % 8;
        fixture.layout.blind_m = static_cast<double>(random_word(state) % 5) * 0.25;
        fixture.layout.angle_step_rad = 0.125 + static_cast<double>(random_word(state) % 4) * 0.125;
        fixture.options.tau = 0.5 + static_cast<double>(random_word(state) % 5) * 0.125;
        fixture.options.v_max_mps = static_cast<double>(random_word(state) % 5) * 0.0625;
        fixture.options.omega_max_rps = static_cast<double>(random_word(state) % 5) * 0.03125;
        fixture.options.jitter_m = static_cast<double>(random_word(state) % 3) * 0.015625;
        for (auto& value : fixture.counts) {
            value.ground = static_cast<std::uint32_t>(random_word(state) % 10);
            value.traversable = static_cast<std::uint32_t>(random_word(state) % (value.ground + 1));
        }
        const auto age = static_cast<std::int64_t>(random_word(state) % 1000000001);
        CHECK(invoke(fixture, age));
        check_brute(fixture, age);
    }
}

BinWalkResult restrict(const BinWalkResult& geometry, const CellEvidence* cells, const std::uint8_t* t,
                       std::size_t count, double blind, double dr, bool fresh) {
    const auto before = allocations;
    count_allocations = true;
    const auto result = restrict_normal_prefix(geometry, cells, t, count, blind, dr, fresh);
    count_allocations = false;
    ++restriction_calls;
    CHECK(allocations == before);
    if (!result.valid) {
        CHECK(result.stop == StopReason::InvalidInput);
        CHECK(result.boundary == BoundaryKind::None);
        CHECK(!result.free_end_m && !result.block_m && !result.block_height_m && !result.slope_rad);
        CHECK(result.passed_cells == 0 && !result.t_backed_prefix);
    } else {
        CHECK(result.block_m == geometry.block_m && result.block_height_m == geometry.block_height_m);
        CHECK(result.boundary == geometry.boundary);
        CHECK(result.passed_cells <= geometry.passed_cells);
        if (!geometry.free_end_m) CHECK(!result.free_end_m);
        if (result.free_end_m) {
            CHECK(geometry.free_end_m && *result.free_end_m <= *geometry.free_end_m);
            if (result.block_m) CHECK(*result.free_end_m < *result.block_m);
        }
        if (!fresh) CHECK(!result.t_backed_prefix);
    }
    return result;
}

// T only cuts the geometric FREE back to the first non-passable cell (slope recomputed over what
// remains) and never moves the boundary; invalid input clears everything; at the near edge real
// kernel output stops FREE at the blind edge rather than meaning missing T.
void test_prefix_restriction() {
    std::array<CellEvidence, 5> cells{};
    for (auto& cell : cells) cell.ground = cell.expected = cell.covered_ground = 4;
    cells[1].mean_height_m = 0.0625;
    cells[2].mean_height_m = 0.1875;
    cells[3].mean_height_m = 0.4375;
    std::array<std::uint8_t, 5> t{{1, 1, 1, 1, 1}};
    BinWalkResult geometry;
    geometry.valid = true;
    geometry.stop = StopReason::Boundary;
    geometry.boundary = BoundaryKind::Negative;
    geometry.free_end_m = 1.5;
    geometry.block_m = 1.75;
    geometry.block_height_m = -0.25;
    geometry.passed_cells = 4;
    geometry.slope_rad = std::atan(1.0);
    geometry.t_backed_prefix = true;
    auto result = restrict(geometry, nullptr, nullptr, 0, 0, 0, false);
    CHECK(result.free_end_m == geometry.free_end_m && result.stop == geometry.stop && result.slope_rad == geometry.slope_rad);
    result = restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true);
    CHECK(result.stop == StopReason::Boundary && result.passed_cells == 4 && result.t_backed_prefix);
    t[2] = 0;
    result = restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true);
    CHECK(result.stop == StopReason::Traversability && result.passed_cells == 2 && result.free_end_m == 1.0);
    CHECK(result.slope_rad == std::atan(0.25) && result.t_backed_prefix);
    t[0] = 0;
    result = restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true);
    CHECK(result.passed_cells == 0 && result.free_end_m == 0.5 && !result.slope_rad && !result.t_backed_prefix);
    t[0] = 1; t[1] = 0;
    result = restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true);
    CHECK(result.passed_cells == 1 && result.free_end_m == 0.75 && !result.slope_rad);
    CHECK(!restrict(geometry, cells.data(), t.data(), 3, 0.5, 0.25, true).valid);
    CHECK(!restrict(geometry, nullptr, nullptr, 0, 0.5, 0.25, true).valid);
    CHECK(!restrict(geometry, nullptr, t.data(), t.size(), 0.5, 0.25, true).valid);
    t[4] = 2;
    CHECK(!restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true).valid);
    t[4] = 1;
    CHECK(!restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0, true).valid);
    CHECK(!restrict(geometry, cells.data(), t.data(), t.size(), std::numeric_limits<double>::quiet_NaN(), 0.25, true).valid);
    cells[4].mean_height_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(!restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true).valid);
    cells[4].mean_height_m = 0;
    geometry.boundary = BoundaryKind::Positive;
    geometry.block_height_m = 0.5;
    result = restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true);
    CHECK(result.block_height_m == 0.5 && result.boundary == BoundaryKind::Positive);
    geometry.free_end_m.reset(); geometry.passed_cells = 0; geometry.slope_rad.reset();
    geometry.block_m = 0.25;
    result = restrict(geometry, cells.data(), t.data(), t.size(), 0.5, 0.25, true);
    CHECK(!result.free_end_m && !result.t_backed_prefix && result.stop == StopReason::Boundary);
    geometry.free_end_m = 0.5; geometry.block_m.reset(); geometry.block_height_m.reset();
    geometry.boundary = BoundaryKind::None; geometry.stop = StopReason::Coverage;
    result = restrict(geometry, nullptr, nullptr, 0, 0.5, 0.25, true);
    CHECK(result.free_end_m == 0.5 && result.stop == StopReason::Coverage && !result.t_backed_prefix);

    // Real kernel output at the near edge remains fresh false, never no-T.
    Fixture fixture;
    fixture.options.jitter_m = 0.01;
    CHECK(invoke(fixture));
    geometry.free_end_m = 2.0; geometry.passed_cells = 4; geometry.stop = StopReason::Range;
    result = restrict(geometry, cells.data(), fixture.eroded.data() + 4 * fixture.layout.stride,
                      5, 1.0, 0.25, true);
    CHECK(result.free_end_m == 1.0 && result.passed_cells == 0 && result.stop == StopReason::Traversability);
}
}  // namespace

int main() {
    test_raw_threshold_and_padding();
    test_radii_and_unknown_domain();
    test_rotation_aging_and_negative_controls();
    test_outward_float_bounds();
    test_invalid_and_empty();
    test_random_oracle();
    test_prefix_restriction();
    if (failures != 0) {
        std::fprintf(stderr, "traversability kernel tests: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("traversability kernel tests: PASS (%zu erosion calls, %zu restriction calls, %zu margin calls; "
                "zero ordinary/aligned C++ new allocations; synthetic evidence only)\n",
                erosion_calls, restriction_calls, margin_calls);
    return EXIT_SUCCESS;
}
