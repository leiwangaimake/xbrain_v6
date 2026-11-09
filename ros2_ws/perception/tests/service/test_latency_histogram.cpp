/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_latency_histogram.cpp
 * Brief: Host test of the bounded latency ledger used by the service timing lines.
 *
 * Description:
 * What it checks. Nearest-rank upper-edge quantiles, exact threshold counts, overflow, invalid
 * samples and no allocation in add(). The ledger feeds PROFILE_TIMING and the S age lines.
 */

#include "service/latency_histogram.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>
#include <algorithm>
#include <random>
#include <vector>

namespace {
std::atomic<std::size_t> allocations{0};
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)
}  // namespace
void* operator new(std::size_t n) {
    ++allocations;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

using perception::service::LatencyHistogram;

int main() {
    constexpr std::int64_t ms = 1000000;
    // 0.1 ms bins up to 50 ms, thresholds 1 ms and 14 ms
    LatencyHistogram h(100000, 500, {1 * ms, 14 * ms});
    CHECK(h.count() == 0 && h.max_ns() == -1 && h.quantile_upper_ns(0.99) == -1);
    // 99 samples of 0.95 ms and one of 80 ms (the Q21 counterexample shape)
    const auto before = allocations.load();
    for (int i = 0; i < 99; ++i) h.add(950000);
    h.add(80 * ms);
    CHECK(allocations.load() == before);
    CHECK(h.count() == 100 && h.max_ns() == 80 * ms && h.overflow() == 1);
    // nearest rank: ceil(0.99*100)=99 -> the 99th sample is 0.95 ms -> bin [0.9, 1.0) upper edge 1.0 ms
    CHECK(h.quantile_upper_ns(0.99) == 1 * ms);
    CHECK(h.quantile_upper_ns(1.0) == 80 * ms);
    CHECK(h.quantile_upper_ns(0.5) == 1 * ms);
    CHECK(h.over(0) == 1 && h.over(1) == 1 && h.threshold(0) == ms && h.over(2) == 0 && h.threshold(3) == -1);
    // exactly at a threshold is not "over"
    h.add(1 * ms);
    CHECK(h.over(0) == 1);
    h.add(1 * ms + 1);
    CHECK(h.over(0) == 2);
    // invalid samples are not clamped into the distribution
    h.add(-5);
    CHECK(h.invalid() == 1 && h.count() == 102);
    CHECK(h.quantile_upper_ns(0) == -1 && h.quantile_upper_ns(1.5) == -1);

    // upper edge never below the true nearest-rank quantile (random property)
    std::mt19937_64 rng(7);
    std::uniform_int_distribution<std::int64_t> dist(0, 60 * ms);
    LatencyHistogram r(250000, 200);
    std::vector<std::int64_t> samples;
    for (int i = 0; i < 5000; ++i) {
        const auto v = dist(rng);
        samples.push_back(v);
        r.add(v);
    }
    std::sort(samples.begin(), samples.end());
    bool conservative = true, tight = true;
    for (double q : {0.5, 0.9, 0.99, 0.999, 1.0}) {
        const auto rank = static_cast<std::size_t>(std::ceil(q * samples.size()));
        const auto truth = samples[rank - 1];
        const auto reported = r.quantile_upper_ns(q);
        conservative &= reported >= truth;
        tight &= truth >= 50 * ms ? reported == samples.back() : reported - truth <= 250000;
    }
    CHECK(conservative && tight);

    bool bad_width = false, bad_bins = false, too_many = false, negative = false;
    try { LatencyHistogram x(0, 10); } catch (const std::invalid_argument&) { bad_width = true; }
    try { LatencyHistogram x(1, 0); } catch (const std::invalid_argument&) { bad_bins = true; }
    try { LatencyHistogram x(1, 10, {1, 2, 3, 4, 5}); } catch (const std::invalid_argument&) { too_many = true; }
    try { LatencyHistogram x(1, 10, {-1}); } catch (const std::invalid_argument&) { negative = true; }
    CHECK(bad_width && bad_bins && too_many && negative);

    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"latency_histogram_host\"}" << std::endl;
    return failures ? 1 : 0;
}
