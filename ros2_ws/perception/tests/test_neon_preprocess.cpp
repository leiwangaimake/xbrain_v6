/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_neon_preprocess.cpp
 * Brief: Unit tests for the NEON letterbox preprocess against the fused path and the frozen legacy oracle.
 *
 * Description:
 * What it checks. preprocess_into_neon must give the same tensor bits, letterbox info and exceptions
 * as preprocess_into_fused and the frozen legacy API for every input: the production sizes
 * (1280 x 800 -> 896 linear and -> 640 exact 2x "area2", both fast on aarch64) over several image
 * kinds, a non-contiguous ROI, other fast sizes, sizes that fall back (upscales, widths that are not
 * a multiple of 16 or 8, the full plan cache), the rejections, and two threads at once with OpenCV
 * at 1 and 2 threads. Dirty outputs and guard sentinels catch unwritten or overwritten elements.
 */

#include "neon_preprocess.hpp"
#include "fused_preprocess.hpp"
#include "reference/preprocess_legacy_api.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
std::size_t checks = 0, failed = 0;
std::vector<std::string> errors;
void require(bool condition, const std::string& name) {
    ++checks;
    if (!condition) {
        ++failed;
        if (errors.size() < 50U) errors.push_back(name);
    }
}
bool same_info(const yolo::LetterboxInfo& a, const yolo::LetterboxInfo& b) {
    return std::memcmp(&a.scale, &b.scale, sizeof(float)) == 0 && a.pad_left == b.pad_left &&
        a.pad_top == b.pad_top && a.new_width == b.new_width && a.new_height == b.new_height &&
        a.original_width == b.original_width && a.original_height == b.original_height;
}
// Kinds: noise, ramps, checker, saturated, binary noise, smooth texture with noise.
cv::Mat image_of(int width, int height, int kind, std::uint32_t seed) {
    cv::Mat image(height, width, CV_8UC3);
    std::mt19937 random(seed);
    for (int y = 0; y < height; ++y) {
        std::uint8_t* row = image.ptr<std::uint8_t>(y);
        for (int x = 0; x < 3 * width; ++x) {
            switch (kind) {
                case 0: row[x] = static_cast<std::uint8_t>(random() & 255U); break;
                case 1: row[x] = static_cast<std::uint8_t>((x / 3 + y + x % 3 * 85) & 255); break;
                case 2: row[x] = ((x / 3 + y) & 1) ? 255 : 0; break;
                case 3: row[x] = 255; break;
                case 4: row[x] = (random() & 1U) ? 255 : 0; break;
                default: row[x] = static_cast<std::uint8_t>(128 + 60 * std::sin(0.05 * x + 0.03 * y) + random() % 9U); break;
            }
        }
    }
    return image;
}
using Function = void (*)(const cv::Mat&, int, yolo::LetterboxInfo&, float*, std::size_t);
std::string invoke(Function fn, const cv::Mat& image, int size, yolo::LetterboxInfo& info, float* out, std::size_t count) {
    try { fn(image, size, info, out, count); return "none"; }
    catch (const cv::Exception& e) { return std::string("cv::Exception:") + e.what(); }
    catch (const std::exception& e) { return std::string("std::exception:") + e.what(); }
}

// neon vs fused vs legacy on one input; returns the number of differing float elements.
std::size_t compare_case(const std::string& name, const cv::Mat& image, int size, int count_delta = 0,
                         bool null_output = false) {
    constexpr std::size_t guard = 16;
    constexpr float sentinel = 12345.25F;
    const std::size_t count = size > 0 ? 3U * static_cast<std::size_t>(size) * static_cast<std::size_t>(size) : 3U;
    const std::size_t provided = static_cast<std::size_t>(static_cast<long long>(count) + count_delta);
    std::vector<float> neon(count + 2 * guard, sentinel), fused(count + 2 * guard, sentinel), legacy(count + 2 * guard, sentinel);
    std::fill_n(neon.data() + guard, count, -7.0F);
    std::fill_n(fused.data() + guard, count, 8.0F);
    std::fill_n(legacy.data() + guard, count, 9.0F);
    const cv::Mat before = image.clone();
    yolo::LetterboxInfo neon_info{3.25F, 1, 2, 3, 4, 5, 6}, fused_info = neon_info, legacy_info = neon_info;
    const std::string e_neon = invoke(yolo::preprocess_into_neon, image, size, neon_info, null_output ? nullptr : neon.data() + guard, provided);
    const std::string e_fused = invoke(yolo_prototype::preprocess_into_fused, image, size, fused_info, null_output ? nullptr : fused.data() + guard, provided);
    const std::string e_legacy = invoke(yolo::preprocess_into_legacy, image, size, legacy_info, null_output ? nullptr : legacy.data() + guard, provided);
    require(e_neon == e_fused, name + ": exception parity with fused (" + e_neon + " | " + e_fused + ")");
    require(same_info(neon_info, fused_info) && same_info(neon_info, legacy_info), name + ": letterbox info");
    require(image.empty() || (before.size() == image.size() && cv::norm(before, image, cv::NORM_INF) == 0.0), name + ": input unchanged");
    std::size_t different = 0;
    if (e_neon == "none" && e_fused == "none" && e_legacy == "none") {
        for (std::size_t i = 0; i < count; ++i) {
            std::uint32_t a, b, c;
            std::memcpy(&a, &neon[guard + i], 4);
            std::memcpy(&b, &fused[guard + i], 4);
            std::memcpy(&c, &legacy[guard + i], 4);
            if (a != b || a != c) ++different;
        }
        require(different == 0U, name + ": tensor bits (" + std::to_string(different) + " differ)");
    }
    bool guards = true;
    for (std::size_t g = 0; g < guard; ++g) {
        guards = guards && neon[g] == sentinel && neon[guard + count + g] == sentinel;
    }
    require(guards, name + ": guards");
    return different;
}
}  // namespace

int main() {
    try {
        cv::setNumThreads(1);
        const auto production = yolo::neon_preprocess_detail::plan_status(1280, 800, 896);
        const auto coco = yolo::neon_preprocess_detail::plan_status(1280, 800, 640);
#if defined(__aarch64__)
        require(production.fast && production.reason == "ok" && production.kind == "linear",
                "1280x800 -> 896 takes the linear fast path: " + production.reason);
        require(coco.fast && coco.reason == "ok" && coco.kind == "area2",
                "1280x800 -> 640 (exact 2x, OpenCV INTER_AREA shortcut) takes the area2 path: " + coco.reason);
#else
        require(!production.fast && production.reason == "not_aarch64", "non-aarch64 always falls back");
        require(!coco.fast && coco.reason == "not_aarch64", "non-aarch64 always falls back (640)");
#endif
        const auto upscale = yolo::neon_preprocess_detail::plan_status(640, 400, 896);
        require(!upscale.fast, "upscale falls back: " + upscale.reason);
        const auto odd_width = yolo::neon_preprocess_detail::plan_status(1280, 800, 900);
        require(!odd_width.fast, "width 900 (not a multiple of 16) falls back: " + odd_width.reason);

        // Production size: six kinds, four seeds each, OpenCV at 1 and 2 threads.
        for (int threads : {1, 2}) {
            cv::setNumThreads(threads);
            for (int kind = 0; kind < 6; ++kind) {
                for (std::uint32_t seed = 0; seed < 4; ++seed) {
                    compare_case("prod_t" + std::to_string(threads) + "_k" + std::to_string(kind) + "_s" + std::to_string(seed),
                                 image_of(1280, 800, kind, 100U * kind + seed), 896);
                }
            }
        }
        cv::setNumThreads(1);
        for (int threads : {1, 2}) {
            cv::setNumThreads(threads);
            for (int kind = 0; kind < 6; ++kind) {
                compare_case("prod640_t" + std::to_string(threads) + "_k" + std::to_string(kind),
                             image_of(1280, 800, kind, 500U + kind), 640);
            }
        }
        cv::setNumThreads(1);
        const cv::Mat owner = image_of(1293, 807, 0, 3);
        const cv::Mat roi = owner(cv::Rect(5, 3, 1280, 800));
        require(!roi.isContinuous(), "ROI must be non-contiguous");
        compare_case("roi_to896", roi, 896);
        compare_case("roi_to640", roi, 640);
        // Other sizes, fast or not; the result must be the same either way. The plan cache holds 8
        // plans (4 made above), so the three fast candidates and one area2 width fallback come
        // first and are each checked; the later sizes may get "plan_cache_full", which must fall
        // back cleanly.
        struct Expect { int w, h, input; const char* kind; };
        for (const Expect& e : {Expect{1280, 720, 896, "linear"}, Expect{1000, 999, 512, "linear"},
                                Expect{1920, 1080, 960, "area2"}, Expect{1000, 998, 500, "none"}}) {
            const auto status = yolo::neon_preprocess_detail::plan_status(e.w, e.h, e.input);
            const std::string name = std::to_string(e.w) + "x" + std::to_string(e.h) + "->" + std::to_string(e.input);
#if defined(__aarch64__)
            const bool want_fast = std::string(e.kind) != "none";
            require(status.fast == want_fast && status.kind == e.kind,
                    name + " plan kind " + status.kind + " reason " + status.reason);
#endif
            compare_case("own_plan_" + name + "_noise", image_of(e.w, e.h, 0, 31), e.input);
            compare_case("own_plan_" + name + "_texture", image_of(e.w, e.h, 5, 32), e.input);
        }
        const auto full = yolo::neon_preprocess_detail::plan_status(1000, 1000, 896);
        require(!full.fast && full.reason == "plan_cache_full", "ninth plan is refused: " + full.reason);
        for (const auto& s : std::vector<std::array<int, 3>>{{1280, 720, 896}, {1920, 1080, 1280}, {1000, 999, 512},
                                                            {1290, 800, 896}, {1000, 1000, 896}, {1280, 800, 900},
                                                            {640, 400, 896}, {513, 205, 640}, {205, 513, 896},
                                                            {17, 13, 64}, {1, 9, 32}, {1, 1, 16}}) {
            const std::string name = "size_" + std::to_string(s[0]) + "x" + std::to_string(s[1]) + "_to" + std::to_string(s[2]);
            const auto status = yolo::neon_preprocess_detail::plan_status(s[0], s[1], s[2]);
            std::cout << "{\"plan\":\"" << name << "\",\"fast\":" << (status.fast ? 1 : 0) << ",\"reason\":\"" << status.reason << "\"}\n";
            compare_case(name + "_noise", image_of(s[0], s[1], 0, 11), s[2]);
            compare_case(name + "_texture", image_of(s[0], s[1], 5, 12), s[2]);
        }
        // Rejections and delegations: same outcome as fused.
        const cv::Mat small = image_of(16, 16, 0, 1);
        compare_case("empty", cv::Mat(), 32);
        compare_case("zero_size", small, 0);
        compare_case("negative_size", small, -1);
        compare_case("null_output", small, 32, 0, true);
        compare_case("short_count", small, 32, -1);
        compare_case("long_count", small, 32, 1);
        compare_case("one_channel", cv::Mat(16, 16, CV_8UC1, cv::Scalar(9)), 32);
        compare_case("four_channels", cv::Mat(16, 16, CV_8UC4, cv::Scalar(1, 2, 3, 4)), 32);
        compare_case("uint16", cv::Mat(16, 16, CV_16UC3, cv::Scalar(1, 2, 3)), 32);

        // Two threads at once on the production size, each against fused results made beforehand.
        cv::setNumThreads(2);
        const std::size_t count = 3U * 896U * 896U;
        std::vector<cv::Mat> images;
        std::vector<std::vector<float>> expected;
        for (int i = 0; i < 6; ++i) {
            images.push_back(image_of(1280, 800, i, 900U + i));
            expected.emplace_back(count);
            yolo::LetterboxInfo info;
            yolo_prototype::preprocess_into_fused(images.back(), 896, info, expected.back().data(), count);
        }
        std::array<std::size_t, 2> thread_differences{{0, 0}};
        auto worker = [&](int t) {
            std::vector<float> out(count);
            for (int iteration = 0; iteration < 12; ++iteration) {
                const int i = (iteration + 3 * t) % 6;
                yolo::LetterboxInfo info;
                yolo::preprocess_into_neon(images[i], 896, info, out.data(), count);
                if (std::memcmp(out.data(), expected[i].data(), count * sizeof(float)) != 0) ++thread_differences[t];
            }
        };
        std::thread first(worker, 0), second(worker, 1);
        first.join();
        second.join();
        require(thread_differences[0] == 0 && thread_differences[1] == 0, "two threads at once");
    } catch (const std::exception& e) {
        require(false, std::string("uncaught: ") + e.what());
    }
    std::cout << "{\"test\":\"neon_preprocess\",\"status\":\"" << (failed == 0 ? "PASS" : "FAIL") << "\",\"opencv_version\":\""
              << CV_VERSION << "\",\"checks\":" << checks << ",\"failed\":" << failed << ",\"errors\":[";
    for (std::size_t i = 0; i < errors.size(); ++i) std::cout << (i ? "," : "") << "\"" << errors[i] << "\"";
    std::cout << "]}\n";
    return failed == 0 ? 0 : 1;
}
