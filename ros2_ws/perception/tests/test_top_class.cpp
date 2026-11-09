/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_top_class.cpp
 * Brief: top_class_per_anchor equals the anchor-major scan bit for bit.
 *
 * Description:
 * The reference is the decoder's original per-anchor loop (select_raw_top_class): classes in
 * order 0..n-1, strict '>'. Cases: random scores (the COCO80 shape, 80 x 8400), ties (the first
 * class wins), NaN in the first class and in later classes, +0/-0, and anchor counts that are
 * not a multiple of four so the scalar tail after the NEON body is exercised.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/top_class.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
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

// The decoder's original scan of one anchor (postprocess.cpp select_raw_top_class).
void reference(const std::vector<float>& scores, std::int64_t anchors, int classes, std::vector<int>& top_class,
               std::vector<float>& top_score) {
    top_class.assign(static_cast<std::size_t>(anchors), 0);
    top_score.assign(static_cast<std::size_t>(anchors), 0.0F);
    for (std::int64_t i = 0; i < anchors; ++i) {
        int best = 0;
        float conf = scores[static_cast<std::size_t>(i)];
        for (int k = 1; k < classes; ++k) {
            const float v = scores[static_cast<std::size_t>(k * anchors + i)];
            if (v > conf) {
                best = k;
                conf = v;
            }
        }
        top_class[static_cast<std::size_t>(i)] = best;
        top_score[static_cast<std::size_t>(i)] = conf;
    }
}

void compare(const std::vector<float>& scores, std::int64_t anchors, int classes, const std::string& name) {
    std::vector<int> expected_class, got_class(static_cast<std::size_t>(anchors));
    std::vector<float> expected_score, got_score(static_cast<std::size_t>(anchors));
    reference(scores, anchors, classes, expected_class, expected_score);
    yolo::top_class_per_anchor(scores.data(), anchors, classes, got_class.data(), got_score.data());
    for (std::int64_t i = 0; i < anchors; ++i) {
        const auto j = static_cast<std::size_t>(i);
        check(got_class[j] == expected_class[j], name + ": class differs at anchor " + std::to_string(i));
        check(std::memcmp(&got_score[j], &expected_score[j], sizeof(float)) == 0,
              name + ": score bits differ at anchor " + std::to_string(i));
    }
}

}  // namespace

int main() {
    try {
        std::mt19937 random(20260929U);
        std::uniform_real_distribution<float> uniform(0.0F, 1.0F);
        // COCO80 shape with random scores, and odd anchor counts for the scalar tail.
        for (const std::int64_t anchors : {std::int64_t{8400}, std::int64_t{1}, std::int64_t{3}, std::int64_t{4},
                                           std::int64_t{5}, std::int64_t{1027}}) {
            std::vector<float> scores(static_cast<std::size_t>(80 * anchors));
            for (float& v : scores) v = uniform(random);
            compare(scores, anchors, 80, "random_" + std::to_string(anchors));
        }
        // Ties, NaN and signed zeros placed on every lane position and in the tail.
        const std::int64_t anchors = 12;
        const int classes = 6;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        std::vector<float> scores(static_cast<std::size_t>(classes * anchors), 0.25F);
        auto at = [&](int k, std::int64_t i) -> float& { return scores[static_cast<std::size_t>(k * anchors + i)]; };
        at(2, 0) = 0.9F; at(4, 0) = 0.9F;              // tie: class 2 wins
        at(0, 1) = nan; at(3, 1) = 0.99F;              // NaN first score is never replaced
        at(2, 2) = nan; at(4, 2) = 0.5F;               // a NaN later never wins; class 4 wins
        for (int k = 0; k < classes; ++k) at(k, 3) = 0.0F;
        at(1, 3) = -0.0F;                              // -0 after +0: still class 0, score +0
        for (int k = 0; k < classes; ++k) at(k, 4) = -0.0F;
        at(1, 4) = 0.0F;                               // +0 after -0: still class 0, score -0
        at(5, 5) = 0.3F;                               // the last class wins
        at(1, 6) = std::numeric_limits<float>::infinity();
        at(3, 7) = -std::numeric_limits<float>::infinity();
        at(5, 10) = 0.8F;                              // tail anchor
        at(0, 9) = nan; at(1, 9) = nan;                // all NaN first two
        for (int k = 0; k < classes; ++k) at(k, 11) = -0.0F;
        at(3, 11) = 0.0F;                              // tail anchor: +0 later does not replace -0
        compare(scores, anchors, classes, "edges");
        std::cout << "test_top_class: PASS checks=" << checks << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_top_class: FAIL " << e.what() << "\n";
        return 1;
    }
}
