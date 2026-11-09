/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_preprocess_dispatch.cpp
 * Brief: Unit tests for the preprocess mode dispatcher against the frozen legacy implementation.
 *
 * Description:
 * What it checks. PERCEPTION preprocess modes select the legacy, fused or neon letterbox path. Each case
 * compares the dispatcher's output pixels and letterbox info with the frozen original API
 * (tests/reference), never with the dispatcher itself; unit controls cover the mode parsing, and
 * child processes check the environment-driven selection in isolation.
 */

#include "common/preprocess_mode.hpp"
#include "reference/preprocess_legacy_api.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <vector>
extern char** environ;

namespace {
std::atomic<std::size_t> checks{0};
void require(bool condition, const char* message) {
    checks.fetch_add(1, std::memory_order_relaxed);
    if (!condition) { throw std::runtime_error(message); }
}
template<class F> void rejects(F function, const char* message) {
    bool caught = false;
    try { function(); } catch (const std::exception&) { caught = true; }
    require(caught, message);
}
bool same_info(const yolo::LetterboxInfo& a, const yolo::LetterboxInfo& b) {
    return std::memcmp(&a.scale, &b.scale, sizeof(float)) == 0 && a.pad_left == b.pad_left &&
        a.pad_top == b.pad_top && a.new_width == b.new_width && a.new_height == b.new_height &&
        a.original_width == b.original_width && a.original_height == b.original_height;
}
cv::Mat synthetic(int width, int height) {
    cv::Mat image(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        auto* row = image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; ++x) {
            row[x] = cv::Vec3b(static_cast<unsigned char>((17*x+5*y)&255),
                              static_cast<unsigned char>((3*x+31*y)&255),
                              static_cast<unsigned char>((19*x+7*y)&255));
        }
    }
    return image;
}
void check_case(const cv::Mat& image, int size) {
    const auto count = 3U * static_cast<std::size_t>(size) * static_cast<std::size_t>(size);
    constexpr std::size_t guard = 16;
    constexpr float sentinel = 12345.25F;
    std::vector<float> oracle(count, -123.0F), actual(count + 2U * guard, sentinel);
    const cv::Mat source_before = image.clone();
    yolo::LetterboxInfo reference_info, actual_info, wrapper_info;
    yolo::preprocess_into_legacy(image, size, reference_info, oracle.data(), count);
    yolo::preprocess_into(image, size, actual_info, actual.data() + guard, count);
    require(std::memcmp(oracle.data(), actual.data() + guard, count * sizeof(float)) == 0,
            "dispatcher tensor differs from independent frozen oracle");
    require(same_info(reference_info, actual_info), "dispatcher metadata differs");
    const auto wrapper = yolo::preprocess(image, size, wrapper_info);
    require(wrapper.size() == count && std::memcmp(oracle.data(), wrapper.data(), count*sizeof(float)) == 0,
            "vector entry bypassed frozen mode or differs");
    require(same_info(reference_info, wrapper_info), "vector entry metadata differs");
    for (std::size_t i = 0; i < guard; ++i) {
        require(actual[i] == sentinel && actual[guard + count + i] == sentinel, "caller buffer guard changed");
    }
    bool input_equal = true;
    for (int y = 0; y < image.rows; ++y) {
        input_equal = input_equal && std::memcmp(image.ptr(y), source_before.ptr(y),
            static_cast<std::size_t>(image.cols) * image.elemSize()) == 0;
    }
    require(input_equal, "source changed");
}
// The window entry under the frozen mode against the rows of the frozen legacy square tensor.
void check_window_case(const cv::Mat& image, yolo::ModelInputShape shape, int window_top) {
    const auto side = static_cast<std::size_t>(shape.width);
    const auto window_plane = side * static_cast<std::size_t>(shape.height);
    std::vector<float> oracle(3U * side * side, -123.0F), actual(3U * window_plane, 12345.25F);
    yolo::LetterboxInfo reference_info, actual_info;
    yolo::preprocess_into_legacy(image, shape.width, reference_info, oracle.data(), oracle.size());
    yolo::preprocess_window_into(image, shape, window_top, actual_info, actual.data(), actual.size());
    for (std::size_t channel = 0; channel < 3U; ++channel) {
        require(std::memcmp(oracle.data() + channel * side * side + static_cast<std::size_t>(window_top) * side,
                            actual.data() + channel * window_plane, window_plane * sizeof(float)) == 0,
                "window dispatcher tensor differs from the frozen oracle's rows");
    }
    reference_info.pad_top -= window_top;
    require(same_info(reference_info, actual_info), "window dispatcher metadata differs");
}
void unit_controls() {
    using yolo::PreprocessMode;
    require(yolo::parse_preprocess_mode(nullptr) == PreprocessMode::Legacy, "unset default legacy");
    require(yolo::parse_preprocess_mode("legacy") == PreprocessMode::Legacy, "legacy parser");
    require(yolo::parse_preprocess_mode("fused") == PreprocessMode::Fused, "fused parser");
    require(yolo::parse_preprocess_mode("shadow") == PreprocessMode::Shadow, "shadow parser");
    require(yolo::parse_preprocess_mode("neon") == PreprocessMode::Neon, "neon parser");
    for (const char* bad : {"", "FUSED", " fused", "fused ", "0", "1", "parallel", "legacy\nfused", "NEON", "neon "}) {
        rejects([&] { (void)yolo::parse_preprocess_mode(bad); }, "invalid mode accepted");
    }
    std::array<float,3> old_values{{0.0F,0.5F,1.0F}}, new_values = old_values;
    yolo::LetterboxInfo a, b;
    auto result = yolo::preprocess_detail::compare(old_values.data(), new_values.data(), 3U, a, b);
    require(result.elements == 3U && result.different_elements == 0U && result.metadata_equal,
            "positive comparison");
    yolo::preprocess_detail::require_equal(result);
    new_values[1] = std::nextafter(new_values[1], 1.0F);
    result = yolo::preprocess_detail::compare(old_values.data(), new_values.data(), 3U, a, b);
    require(result.different_elements == 1U && result.first_difference == 1U && result.old_bits != result.new_bits,
            "one ULP negative control not distinguished");
    rejects([&] { yolo::preprocess_detail::require_equal(result); }, "mismatch failed to throw");
    new_values = old_values; new_values[0] = -0.0F;
    result = yolo::preprocess_detail::compare(old_values.data(), new_values.data(), 3U, a, b);
    require(result.different_elements == 1U, "signed zero negative control");
    new_values = old_values;
    for (int field = 0; field < 7; ++field) {
        b = a;
        switch (field) {
            case 0: b.scale = std::nextafter(a.scale, 2.0F); break;
            case 1: ++b.pad_left; break;
            case 2: ++b.pad_top; break;
            case 3: ++b.new_width; break;
            case 4: ++b.new_height; break;
            case 5: ++b.original_width; break;
            case 6: ++b.original_height; break;
        }
        result = yolo::preprocess_detail::compare(old_values.data(), new_values.data(), 3U, a, b);
        require(!result.metadata_equal && result.different_elements == 0U, "metadata negative control");
        rejects([&] { yolo::preprocess_detail::require_equal(result); }, "metadata mismatch failed closed");
    }
    rejects([&] { (void)yolo::preprocess_detail::compare(nullptr, new_values.data(), 3U, a, b); },
            "null comparison accepted");
}
int child(const std::string& mode) {
    if (mode == "unset") { require(unsetenv("PERCEPTION_PREPROCESS_IMPL") == 0, "unsetenv failed"); }
    else { require(setenv("PERCEPTION_PREPROCESS_IMPL", mode.c_str(), 1) == 0, "setenv failed"); }
    if (mode == "invalid") {
        rejects([] { (void)yolo::initialize_preprocess_mode_from_environment(); }, "invalid init accepted");
        require(setenv("PERCEPTION_PREPROCESS_IMPL", "legacy", 1) == 0, "setenv failed");
        rejects([] { (void)yolo::initialize_preprocess_mode_from_environment(); }, "invalid selection was not frozen");
        cv::Mat input = synthetic(3, 2); std::vector<float> output(48U, 99.0F); yolo::LetterboxInfo info;
        rejects([&] { yolo::preprocess_into(input, 4, info, output.data(), output.size()); },
                "invalid frozen mode allowed tensor production");
        require(std::all_of(output.begin(), output.end(), [](float value) { return value == 99.0F; }),
                "invalid mode wrote output");
    } else {
        const auto expected = yolo::parse_preprocess_mode(mode == "unset" ? nullptr : mode.c_str());
        require(yolo::initialize_preprocess_mode_from_environment() == expected, "initial mode differs");
        require(setenv("PERCEPTION_PREPROCESS_IMPL", "invalid_after_freeze", 1) == 0, "setenv failed");
        require(yolo::initialize_preprocess_mode_from_environment() == expected, "mode changed after freeze");
        cv::setNumThreads(2);
        const auto image = synthetic(1280, 800);
        // Mirrors two independent caller destinations, not a production thread change.
        std::array<std::string,2> errors;
        std::thread first([&] { try { check_case(image, 640); } catch (const std::exception& e) { errors[0] = e.what(); } });
        std::thread second([&] { try { check_case(image, 896); } catch (const std::exception& e) { errors[1] = e.what(); } });
        first.join(); second.join();
        require(errors[0].empty() && errors[1].empty(), "parallel dispatch failed");
        check_case(synthetic(513, 205), 896);
        const auto owner = synthetic(1293,807);
        const auto roi = owner(cv::Rect(5,3,1280,800));
        require(!roi.isContinuous(), "ROI accidentally contiguous");
        check_case(roi, 640);
        check_case(cv::Mat(16,16,CV_16UC3,cv::Scalar(1,2,3)), 32);
        check_case(cv::Mat(16,16,CV_32FC3,cv::Scalar(1,2,3)), 32);
        // The window entry in the same mode: the Active12 window, a stepped source, the whole
        // square, and an image the window would cut.
        check_window_case(image, yolo::ModelInputShape{896, 608}, 160);
        check_window_case(roi, yolo::ModelInputShape{896, 608}, 160);
        check_window_case(synthetic(1920, 1080), yolo::ModelInputShape{896, 608}, 160);
        check_window_case(image, yolo::ModelInputShape{896, 896}, 0);
        {
            yolo::LetterboxInfo window_info; std::vector<float> window(3U * 896U * 608U, 99.0F);
            rejects([&] { yolo::preprocess_window_into(synthetic(640, 480), yolo::ModelInputShape{896, 608}, 160,
                                                       window_info, window.data(), window.size()); },
                    "image cut by the window accepted");
            require(window[0] == 99.0F && window.back() == 99.0F, "refused window call wrote output");
        }
        yolo::LetterboxInfo info; std::vector<float> output(48U,99.0F);
        rejects([&] { yolo::preprocess_into(cv::Mat(),4,info,output.data(),output.size()); }, "empty accepted");
        rejects([&] { yolo::preprocess_into(image,4,info,nullptr,48U); }, "null output accepted");
        rejects([&] { yolo::preprocess_into(image,4,info,output.data(),47U); }, "short count accepted");
    }
    std::cout << "{\"test\":\"preprocess_dispatch_child\",\"mode\":\"" << mode
              << "\",\"checks\":" << checks.load() << ",\"failed\":0}\n";
    return 0;
}
}  // namespace
int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--child") { return child(argv[2]); }
        require(argc == 1, "unknown test arguments");
        unit_controls();
        for (const char* mode : {"unset", "legacy", "fused", "shadow", "neon", "invalid"}) {
            pid_t pid = -1;
            char* arguments[] = {argv[0], const_cast<char*>("--child"), const_cast<char*>(mode), nullptr};
            require(posix_spawnp(&pid, argv[0], nullptr, nullptr, arguments, environ) == 0, "spawn failed");
            int status = 0;
            require(waitpid(pid, &status, 0) == pid, "waitpid failed");
            require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "child test did not exit successfully");
        }
        std::cout << "{\"test\":\"preprocess_dispatch\",\"checks\":" << checks.load()
                  << ",\"children\":6,\"failed\":0}\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "preprocess dispatch FAIL: " << error.what() << '\n'; return 1;
    }
}
