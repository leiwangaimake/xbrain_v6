/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_scene_luma_window.cpp
 * Brief: grid_luma_mean and SceneLumaWindow: the grid, the weights, padded rows, refusals, windows.
 *
 * Description:
 * The grid mean must equal a direct computation over the same points; only grid points count (a
 * value written off the grid changes nothing); row padding is skipped; unusable images give -1;
 * the window samples once a second, fills at kWindowSamples, keeps min / max / mean of the usable
 * samples only, records the first and last sample (time, frame) and the sampling cost, and its
 * sampling clock survives reset() while the window identity and cost are cleared.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/scene_luma_window.hpp"

#include <cmath>
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

// The same mean computed point by point, in double, from the BT.601 weights in 1/256.
double reference_mean(const std::vector<std::uint8_t>& image, int rows, int cols, std::size_t row_bytes, int step) {
    double sum = 0.0;
    int n = 0;
    for (int y = step / 2; y < rows; y += step)
        for (int x = step / 2; x < cols; x += step) {
            const std::uint8_t* p = image.data() + static_cast<std::size_t>(y) * row_bytes + static_cast<std::size_t>(x) * 3U;
            sum += std::floor((29.0 * p[0] + 150.0 * p[1] + 77.0 * p[2]) / 256.0);
            ++n;
        }
    return sum / n;
}

}  // namespace

int main() {
    try {
        using perception::grid_luma_mean;
        using W = perception::SceneLumaWindow;
        // Uniform grey: every weight set sums to 256, so Y is the grey level itself.
        {
            const int rows = 800, cols = 1280;
            std::vector<std::uint8_t> image(static_cast<std::size_t>(rows) * cols * 3U, 100);
            check(grid_luma_mean(image.data(), rows, cols, cols * 3U, 3, 16) == 100.0, "uniform grey");
            // Pure blue, green, red: the weights.
            for (std::size_t i = 0; i < image.size(); i += 3) { image[i] = 255; image[i + 1] = 0; image[i + 2] = 0; }
            check(grid_luma_mean(image.data(), rows, cols, cols * 3U, 3, 16) == std::floor(29.0 * 255 / 256), "blue weight");
            for (std::size_t i = 0; i < image.size(); i += 3) { image[i] = 0; image[i + 1] = 255; image[i + 2] = 0; }
            check(grid_luma_mean(image.data(), rows, cols, cols * 3U, 3, 16) == std::floor(150.0 * 255 / 256), "green weight");
            for (std::size_t i = 0; i < image.size(); i += 3) { image[i] = 0; image[i + 1] = 0; image[i + 2] = 255; }
            check(grid_luma_mean(image.data(), rows, cols, cols * 3U, 3, 16) == std::floor(77.0 * 255 / 256), "red weight");
        }
        // Random images, with and without row padding, against the point-by-point reference.
        std::mt19937 random(7);
        for (int trial = 0; trial < 40; ++trial) {
            const int rows = 1 + static_cast<int>(random() % 900), cols = 1 + static_cast<int>(random() % 1400);
            const std::size_t pad = (trial % 2) ? random() % 64U : 0U;
            const std::size_t row_bytes = static_cast<std::size_t>(cols) * 3U + pad;
            const int step = 1 + static_cast<int>(random() % 32);
            std::vector<std::uint8_t> image(static_cast<std::size_t>(rows) * row_bytes);
            for (auto& b : image) b = static_cast<std::uint8_t>(random());
            const double got = grid_luma_mean(image.data(), rows, cols, row_bytes, 3, step);
            check(std::fabs(got - reference_mean(image, rows, cols, row_bytes, step)) < 1e-9, "random image " + std::to_string(trial));
            // Off-grid pixels and padding do not count.
            if (step > 1) {
                std::vector<std::uint8_t> changed = image;
                for (int y = 0; y < rows; ++y)
                    for (std::size_t b = 0; b < row_bytes; ++b) {
                        const bool on_grid_row = y >= step / 2 && (y - step / 2) % step == 0;
                        const std::size_t x = b / 3U;
                        const bool on_grid_pixel = b < static_cast<std::size_t>(cols) * 3U &&
                            static_cast<int>(x) >= step / 2 && (static_cast<int>(x) - step / 2) % step == 0;
                        if (!(on_grid_row && on_grid_pixel)) changed[static_cast<std::size_t>(y) * row_bytes + b] ^= 0xA5;
                    }
                check(grid_luma_mean(changed.data(), rows, cols, row_bytes, 3, step) == got, "off-grid change " + std::to_string(trial));
            }
        }
        // Refusals.
        std::vector<std::uint8_t> small(30, 1);
        check(grid_luma_mean(nullptr, 2, 5, 15, 3, 1) < 0, "null data");
        check(grid_luma_mean(small.data(), 0, 5, 15, 3, 1) < 0 && grid_luma_mean(small.data(), 2, 0, 15, 3, 1) < 0, "empty");
        check(grid_luma_mean(small.data(), 2, 5, 15, 1, 1) < 0 && grid_luma_mean(small.data(), 2, 5, 15, 4, 1) < 0, "channels");
        check(grid_luma_mean(small.data(), 2, 5, 15, 3, 0) < 0, "step");
        check(grid_luma_mean(small.data(), 2, 5, 14, 3, 1) < 0, "short rows");
        check(grid_luma_mean(small.data(), 2, 5, 15, 3, 1) == 1.0, "small image");
        // The window.
        {
            W w;
            const std::int64_t s = 1000000000;
            check(w.due(5 * s) && w.mean() < 0 && w.min() < 0 && w.max() < 0, "empty window");
            check(w.first_ns() == 0 && w.last_frame() == 0 && w.cost_total_ns() == 0, "empty identity");
            check(!w.add(5 * s, 40.0, 1001, 300000), "first sample");
            check(!w.due(5 * s + s - 1) && w.due(6 * s), "one sample a second");
            check(!w.add(6 * s, -1.0, 1031, 900000) && w.unusable() == 1 && w.usable() == 1, "unusable sample");
            check(!w.add(7 * s, 20.0, 1061, 100000) && !w.add(8 * s, 60.0, 1091), "more samples");
            check(w.count() == 4 && w.mean() == 40.0 && w.min() == 20.0 && w.max() == 60.0, "min max mean of usable only");
            check(w.first_ns() == 5 * s && w.last_ns() == 8 * s && w.first_frame() == 1001 && w.last_frame() == 1091,
                  "first and last sample");
            check(w.cost_total_ns() == 1300000 && w.cost_max_ns() == 900000, "sampling cost (a zero cost adds nothing)");
            bool full = false;
            for (std::int64_t t = 9; !full; ++t) full = w.add(t * s, 30.0);
            check(w.count() == W::kWindowSamples, "fills at kWindowSamples");
            const std::int64_t last = 9 * s + (static_cast<std::int64_t>(W::kWindowSamples) - 5) * s;
            w.reset();
            check(w.count() == 0 && w.unusable() == 0 && w.mean() < 0, "reset clears the window");
            check(w.first_ns() == 0 && w.first_frame() == 0 && w.last_frame() == 0 && w.cost_total_ns() == 0 &&
                  w.cost_max_ns() == 0, "reset clears identity and cost");
            check(!w.due(last + s - 1) && w.due(last + s), "the sampling clock survives reset");
            // A window of unusable samples only reports -1.
            W bad;
            for (std::int64_t t = 1; !bad.add(t * s, -1.0);) ++t;
            check(bad.usable() == 0 && bad.mean() < 0 && bad.min() < 0 && bad.max() < 0, "only unusable samples");
        }
        std::cout << "test_scene_luma_window: PASS checks=" << checks << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_scene_luma_window: FAIL " << e.what() << "\n";
        return 1;
    }
}
