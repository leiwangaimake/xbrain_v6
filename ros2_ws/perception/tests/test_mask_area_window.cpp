/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_mask_area_window.cpp
 * Brief: MaskAreaWindow: per-branch means, the log-histogram 99th percentile and reset.
 *
 * Description:
 * The p99 upper edge must bound the true 99th percentile from above and stay within one bucket
 * (x1.25 + 1 pixel); branches do not mix; an empty window reports zeros; reset clears everything.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/mask_area_window.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int checks = 0;

void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
    try {
        using W = perception::MaskAreaWindow;
        W w;
        check(W::mean(w.branch(W::kCoco80).roi) == 0 && W::quantile_upper(w.branch(W::kCoco80).roi, .99) == 0,
              "empty window not zero");
        // Edges increase and bound the whole 1280x800 raster.
        for (std::size_t i = 1; i < W::kBuckets; ++i) check(W::upper_edge(i) >= W::upper_edge(i - 1), "edges not monotone");
        check(W::upper_edge(W::kBuckets - 1) >= 1280U * 800U, "top edge below the raster size");
        // 1000 instances of 1000 px and 10 of 800000 px in the Active12 branch.
        for (int i = 0; i < 1000; ++i) w.add(W::kActive12, 1000, 400);
        for (int i = 0; i < 10; ++i) w.add(W::kActive12, 800000, 700000);
        w.frame();
        const auto& a = w.branch(W::kActive12);
        check(a.roi.n == 1010 && a.roi.sum == 1000ULL * 1000 + 10ULL * 800000, "counts wrong");
        check(W::quantile_upper(a.roi, .99) == 1024, "p99 edge of 1000 px should be 1024");
        check(W::quantile_upper(a.roi, .995) == 917504, "p99.5 edge of 800000 px should be 917504");
        check(W::quantile_upper(a.foreground, .995) == 786432, "foreground p99.5 edge of 700000 px should be 786432");
        check(w.branch(W::kCoco80).roi.n == 0 && w.branch(W::kClass92).roi.n == 0, "branches mixed");
        // Random values: the edge bounds the true p99 and stays within one bucket.
        std::mt19937 random(7U);
        std::uniform_int_distribution<std::uint32_t> pixels(0, 1280U * 800U);
        std::vector<std::uint64_t> values;
        W r;
        for (int i = 0; i < 5000; ++i) {
            const std::uint64_t v = pixels(random);
            values.push_back(v);
            r.add(W::kCoco80, v, v / 2);
        }
        std::sort(values.begin(), values.end());
        const std::uint64_t truth = values[static_cast<std::size_t>(std::ceil(.99 * 5000)) - 1];
        const std::uint64_t edge = W::quantile_upper(r.branch(W::kCoco80).roi, .99);
        check(edge >= truth && static_cast<double>(edge) <= 1.25 * static_cast<double>(truth) + 1, "p99 edge not within one bucket");
        w.reset();
        check(w.frames() == 0 && w.branch(W::kActive12).roi.n == 0, "reset incomplete");
        std::cout << "test_mask_area_window: PASS checks=" << checks << " random_p99_truth=" << truth << " edge=" << edge << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_mask_area_window: FAIL " << e.what() << "\n";
        return 1;
    }
}
