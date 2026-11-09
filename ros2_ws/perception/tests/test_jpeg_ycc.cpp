/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_jpeg_ycc.cpp
 * Brief: Unit tests for the libjpeg-equivalent 4:2:2 YCbCr to BGR conversion (jpeg_ycc).
 *
 * Description:
 * What it checks. The table-based conversion equals an independent per-pixel reference written
 * from the libjpeg formulas (jdsample.c h2v1_fancy_upsample, jdcolor.c with FIX() and ONE_HALF),
 * bit for bit, on random planes of several sizes with padded strides; fixed oracles computed by
 * hand (chroma widening through the blue channel; the JFIF primaries and grey); and the
 * refusals (odd or too small width, null plane, short stride).
 */

#include "modules/jpeg_ycc.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

int failures = 0;
#define CHECK(cond) do { if (!(cond)) { ++failures; std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #cond); } } while (0)

// Independent reference: the libjpeg formulas evaluated directly for each output pixel.
int fix(double v) { return static_cast<int>(v * 65536.0 + 0.5); }
int widened(const std::uint8_t* c, int n, int x) {  // chroma at output column x from n samples
    const int i = x / 2;
    if (x == 0) return c[0];
    if (x == 2 * n - 1) return c[n - 1];
    if (x == 1) return (c[0] * 3 + c[1] + 2) >> 2;
    if (x == 2 * n - 2) return (c[n - 1] * 3 + c[n - 2] + 1) >> 2;
    return x % 2 == 0 ? (c[i] * 3 + c[i - 1] + 1) >> 2 : (c[i] * 3 + c[i + 1] + 2) >> 2;
}
std::uint8_t clamp8(int v) { return static_cast<std::uint8_t>(v < 0 ? 0 : v > 255 ? 255 : v); }
void reference(const std::uint8_t* y, int ys, const std::uint8_t* cb, const std::uint8_t* cr, int cs, int w, int h,
               std::uint8_t* out, int os) {
    const int half = 1 << 15;
    for (int r = 0; r < h; ++r)
        for (int x = 0; x < w; ++x) {
            const int l = y[r * ys + x];
            const int b = widened(cb + r * cs, w / 2, x) - 128, c = widened(cr + r * cs, w / 2, x) - 128;
            out[r * os + 3 * x] = clamp8(l + ((fix(1.77200) * b + half) >> 16));
            out[r * os + 3 * x + 1] = clamp8(l + ((-fix(0.34414) * b + half - fix(0.71414) * c) >> 16));
            out[r * os + 3 * x + 2] = clamp8(l + ((fix(1.40200) * c + half) >> 16));
        }
}

template <typename F>
bool refused(F f) {
    try { f(); } catch (const std::invalid_argument&) { return true; }
    return false;
}

// Random planes with padded strides (padding bytes are random too, so a stride misuse shows).
void test_random_planes_match_reference() {
    std::mt19937 rng(20261005);
    for (const auto& size : std::vector<std::pair<int, int>>{{4, 1}, {6, 3}, {8, 2}, {18, 5}, {1280, 800}}) {
        const int w = size.first, h = size.second, ys = w + 3, cs = w / 2 + 5, os = 3 * w + 7;
        std::vector<std::uint8_t> y(ys * h), cb(cs * h), cr(cs * h), got(os * h, 0xA5), want(os * h, 0xA5);
        for (auto* v : {&y, &cb, &cr}) for (auto& b : *v) b = static_cast<std::uint8_t>(rng());
        perception::jfif_ycc422_to_bgr(y.data(), ys, cb.data(), cr.data(), cs, w, h, got.data(), os);
        reference(y.data(), ys, cb.data(), cr.data(), cs, w, h, want.data(), os);
        bool same = true;
        for (int r = 0; r < h; ++r)
            same = same && std::equal(got.begin() + r * os, got.begin() + r * os + 3 * w, want.begin() + r * os);
        CHECK(same);
        // Bytes past the row end are never written.
        bool padding_untouched = true;
        for (int r = 0; r < h; ++r)
            for (int k = 3 * w; k < os; ++k) padding_untouched = padding_untouched && got[r * os + k] == 0xA5;
        CHECK(padding_untouched);
    }
}

// Chroma [64, 128, 192] widens to [64, 80, 112, 144, 176, 192]; with Y = Cr = 128 the blue channel
// is 128 + cb_b(widened) = [15, 43, 100, 156, 213, 241] (computed by hand from FIX(1.772)).
void test_hand_computed_widening_and_primaries() {
    const std::uint8_t y[6] = {128, 128, 128, 128, 128, 128}, cb[3] = {64, 128, 192}, cr[3] = {128, 128, 128};
    std::uint8_t out[18];
    perception::jfif_ycc422_to_bgr(y, 6, cb, cr, 3, 6, 1, out, 18);
    const std::uint8_t blue[6] = {15, 43, 100, 156, 213, 241};
    for (int x = 0; x < 6; ++x) CHECK(out[3 * x] == blue[x] && out[3 * x + 2] == 128);
    struct Case { std::uint8_t y, cb, cr, b, g, r; };
    const Case cases[] = {{128, 128, 128, 128, 128, 128}, {76, 85, 255, 0, 0, 254}, {150, 44, 21, 1, 255, 0},
                          {29, 255, 107, 254, 0, 0}, {255, 128, 128, 255, 255, 255}, {0, 128, 128, 0, 0, 0}};
    for (const auto& c : cases) {
        const std::uint8_t ys[4] = {c.y, c.y, c.y, c.y}, cbs[2] = {c.cb, c.cb}, crs[2] = {c.cr, c.cr};
        std::uint8_t px[12];
        perception::jfif_ycc422_to_bgr(ys, 4, cbs, crs, 2, 4, 1, px, 12);
        for (int x = 0; x < 4; ++x) CHECK(px[3 * x] == c.b && px[3 * x + 1] == c.g && px[3 * x + 2] == c.r);
    }
}

// Odd or too small width, empty height, null planes and short strides are refused.
void test_refusals() {
    std::vector<std::uint8_t> p(64, 0), o(256, 0);
    const auto run = [&](const std::uint8_t* y, int ys, int cs, int w, int h, std::uint8_t* out, int os) {
        return refused([&] { perception::jfif_ycc422_to_bgr(y, ys, p.data(), p.data(), cs, w, h, out, os); });
    };
    CHECK(run(p.data(), 8, 4, 5, 1, o.data(), 24));
    CHECK(run(p.data(), 8, 4, 2, 1, o.data(), 24));
    CHECK(run(p.data(), 8, 4, 8, 0, o.data(), 24));
    CHECK(run(nullptr, 8, 4, 8, 1, o.data(), 24));
    CHECK(run(p.data(), 8, 4, 8, 1, nullptr, 24));
    CHECK(run(p.data(), 7, 4, 8, 1, o.data(), 24));
    CHECK(run(p.data(), 8, 3, 8, 1, o.data(), 24));
    CHECK(run(p.data(), 8, 4, 8, 1, o.data(), 23));
    CHECK(!run(p.data(), 8, 4, 8, 1, o.data(), 24));
}

}  // namespace

int main() {
    test_random_planes_match_reference();
    test_hand_computed_widening_and_primaries();
    test_refusals();
    if (failures) {
        std::fprintf(stderr, "test_jpeg_ycc: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("test_jpeg_ycc: PASS\n");
    return EXIT_SUCCESS;
}
