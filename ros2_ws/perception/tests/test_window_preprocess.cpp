/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_window_preprocess.cpp
 * Brief: Unit tests for preprocessing into a window of the square letterbox.
 *
 * Description:
 * What it checks. A window tensor must be exactly the rows of the square tensor the frozen legacy
 * implementation (tests/reference) gives for the same image: every float bit, for the reference,
 * the one-pass and the NEON implementation, for the Active12 window (896 x 608 from row 160) and two other
 * windows, for camera-like, odd-sized, stepped (ROI) and non-8-bit images; the letterbox seen
 * through the window; untouched memory around the caller's buffer and an unchanged source image;
 * and every refusal: content cut by the window, a window outside the square, wrong buffer.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "neon_preprocess.hpp"
#include "reference/preprocess_legacy_api.hpp"
#include "window_preprocess.hpp"

#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace {
using Implementation = std::function<void(const cv::Mat&, yolo::ModelInputShape, int, yolo::LetterboxInfo&, float*,
                                          std::size_t)>;
std::size_t checks = 0, floats_compared = 0;

void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

template <typename Call>
void expect_invalid_argument(Call&& call, const std::string& label) {
    try {
        call();
    } catch (const std::invalid_argument&) {
        ++checks;
        return;
    }
    throw std::runtime_error(label + ": expected std::invalid_argument");
}

// Every byte value and neighbour mix occurs.
cv::Mat random_image(int width, int height, int type, unsigned seed) {
    cv::Mat image(height, width, type);
    std::mt19937 random(seed);
    for (int y = 0; y < height; ++y) {
        unsigned char* row = image.ptr<unsigned char>(y);
        for (std::size_t x = 0; x < static_cast<std::size_t>(width) * image.elemSize(); ++x) {
            row[x] = static_cast<unsigned char>(random() & 0xFFU);
        }
    }
    return image;
}

// One implementation on one image and window against the rows of the frozen legacy square tensor.
void equals_square_rows(const Implementation& run, const std::string& name, const cv::Mat& image,
                        yolo::ModelInputShape shape, int window_top) {
    const std::size_t side = static_cast<std::size_t>(shape.width);
    const std::size_t window_plane = side * static_cast<std::size_t>(shape.height);
    std::vector<float> square(3U * side * side, -7.0F);
    yolo::LetterboxInfo square_info;
    yolo::preprocess_into_legacy(image, shape.width, square_info, square.data(), square.size());

    constexpr std::size_t guard = 32;
    constexpr float sentinel = 54321.5F;
    std::vector<float> window(3U * window_plane + 2U * guard, sentinel);
    const cv::Mat before = image.clone();
    yolo::LetterboxInfo info;
    info.pad_top = -999;
    run(image, shape, window_top, info, window.data() + guard, 3U * window_plane);

    for (std::size_t channel = 0; channel < 3U; ++channel) {
        const float* expected = square.data() + channel * side * side + static_cast<std::size_t>(window_top) * side;
        const float* actual = window.data() + guard + channel * window_plane;
        if (std::memcmp(expected, actual, window_plane * sizeof(float)) != 0) {
            std::size_t first = 0;
            while (std::memcmp(expected + first, actual + first, sizeof(float)) == 0) ++first;
            throw std::runtime_error(name + ": plane " + std::to_string(channel) + " differs at row " +
                                     std::to_string(first / side) + " column " + std::to_string(first % side));
        }
        floats_compared += window_plane;
    }
    ++checks;
    for (std::size_t i = 0; i < guard; ++i) {
        check(window[i] == sentinel && window[guard + 3U * window_plane + i] == sentinel, name + ": memory around the buffer changed");
    }
    check(std::memcmp(&info.scale, &square_info.scale, sizeof(float)) == 0 && info.pad_left == square_info.pad_left &&
          info.pad_top == square_info.pad_top - window_top && info.new_width == square_info.new_width &&
          info.new_height == square_info.new_height && info.original_width == square_info.original_width &&
          info.original_height == square_info.original_height,
          name + ": letterbox seen through the window");
    check(info.pad_top >= 0 && info.pad_top + info.new_height <= shape.height, name + ": content inside the window");
    bool unchanged = image.size() == before.size();
    for (int y = 0; unchanged && y < image.rows; ++y) {
        unchanged = std::memcmp(image.ptr(y), before.ptr(y), static_cast<std::size_t>(image.cols) * image.elemSize()) == 0;
    }
    check(unchanged, name + ": source image changed");
}

void window_equals_square_rows(const Implementation& run, const std::string& implementation) {
    // The Active12 window: rows 160..767 of the 896 x 896 letterbox. Camera frame (8 rows above
    // the image, 40 below), 16:9, an odd width, the tallest content the window holds (576 rows,
    // none above) and a size whose resized height is odd.
    const yolo::ModelInputShape active{896, 608};
    // (Seven source and input sizes in this file: the NEON plan cache holds eight.)
    const int sizes[][2] = {{1280, 800}, {1920, 1080}, {1281, 720}, {1400, 900}, {1279, 801}};
    unsigned seed = 1;
    for (const auto& size : sizes) {
        equals_square_rows(run, implementation + " 896x608 " + std::to_string(size[0]) + "x" + std::to_string(size[1]),
                           random_image(size[0], size[1], CV_8UC3, seed++), active, 160);
    }
    // A stepped source (an ROI of a larger image), as a capture buffer may be.
    const cv::Mat owner = random_image(1293, 807, CV_8UC3, 77U);
    const cv::Mat roi = owner(cv::Rect(5, 3, 1280, 800));
    check(!roi.isContinuous(), "ROI accidentally contiguous");
    equals_square_rows(run, implementation + " 896x608 ROI", roi, active, 160);
    // Image types the fused square path hands to legacy.
    equals_square_rows(run, implementation + " 896x608 16-bit", cv::Mat(800, 1280, CV_16UC3, cv::Scalar(1, 2, 3)), active, 160);
    equals_square_rows(run, implementation + " 896x608 float", cv::Mat(800, 1280, CV_32FC3, cv::Scalar(0.25, 0.5, 0.75)), active, 160);
    // Other windows: the centred 896 x 640 band, and a COCO80-sized one (640 x 416 from row 112).
    equals_square_rows(run, implementation + " 896x640", random_image(1280, 800, CV_8UC3, 91U), yolo::ModelInputShape{896, 640}, 128);
    equals_square_rows(run, implementation + " 640x416", random_image(1280, 800, CV_8UC3, 92U), yolo::ModelInputShape{640, 416}, 112);
    // A window that is the whole square.
    equals_square_rows(run, implementation + " 896x896", random_image(1280, 800, CV_8UC3, 93U), yolo::ModelInputShape{896, 896}, 0);
    equals_square_rows(run, implementation + " 64x64 of 37x23", random_image(37, 23, CV_8UC3, 94U), yolo::ModelInputShape{64, 64}, 0);
}

void refusals(const Implementation& run, const std::string& implementation) {
    const yolo::ModelInputShape active{896, 608};
    const std::size_t count = 3U * 896U * 608U;
    std::vector<float> output(count, 99.0F);
    yolo::LetterboxInfo info;
    const cv::Mat camera = random_image(1280, 800, CV_8UC3, 5U);
    const auto refuse = [&](const std::string& label, const cv::Mat& image, yolo::ModelInputShape shape, int top,
                            float* data, std::size_t n) {
        expect_invalid_argument([&] { run(image, shape, top, info, data, n); }, implementation + ": " + label);
    };
    // Content the window would cut: 4:3 (672 rows on the square input) and portrait.
    refuse("4:3 image", random_image(640, 480, CV_8UC3, 6U), active, 160, output.data(), count);
    refuse("portrait image", random_image(800, 1280, CV_8UC3, 7U), active, 160, output.data(), count);
    // The camera frame fits rows 160..767 but not a window that starts inside it or ends before it.
    refuse("window starting inside the image", camera, active, 176, output.data(), count);
    refuse("window ending inside the image", camera, yolo::ModelInputShape{896, 544}, 160, output.data(), 3U * 896U * 544U);
    // Windows that are not inside the square, and degenerate shapes.
    refuse("negative window top", camera, active, -16, output.data(), count);
    refuse("window past the square", camera, active, 320, output.data(), count);
    refuse("zero height", camera, yolo::ModelInputShape{896, 0}, 160, output.data(), 0U);
    refuse("zero width", camera, yolo::ModelInputShape{0, 608}, 0, output.data(), 0U);
    // The caller's buffer.
    refuse("null buffer", camera, active, 160, nullptr, count);
    refuse("short buffer", camera, active, 160, output.data(), count - 1U);
    refuse("square-sized buffer", camera, active, 160, output.data(), 3U * 896U * 896U);
    // Images the square code refuses.
    refuse("empty image", cv::Mat(), active, 160, output.data(), count);
    refuse("single-channel image", cv::Mat(800, 1280, CV_8UC1, cv::Scalar(7)), active, 160, output.data(), count);
    for (std::size_t i = 0; i < count; i += 4099U) {
        check(output[i] == 99.0F, implementation + ": a refused call wrote into the buffer");
    }
}
}  // namespace

int main() {
    try {
        cv::setNumThreads(2);
        const Implementation reference = yolo::preprocess_window_into_reference;
        const Implementation fused = yolo::preprocess_window_into_fused;
        const Implementation neon = yolo::preprocess_window_into_neon;
        window_equals_square_rows(reference, "reference");
        window_equals_square_rows(fused, "fused");
        window_equals_square_rows(neon, "neon");
        refusals(reference, "reference");
        refusals(fused, "fused");
        refusals(neon, "neon");
#if defined(__aarch64__)
        // On the target the camera sources took the NEON plans (not the fallback): the linear one
        // for the Active12 input and the 2x area one for a 640-wide input.
        const auto active_plan = yolo::neon_preprocess_detail::plan_status(1280, 800, 896);
        const auto coco_plan = yolo::neon_preprocess_detail::plan_status(1280, 800, 640);
        check(active_plan.fast && active_plan.kind == "linear", "1280x800 -> 896 is not a fast linear NEON plan: " + active_plan.reason);
        check(coco_plan.fast && coco_plan.kind == "area2", "1280x800 -> 640 is not a fast area2 NEON plan: " + coco_plan.reason);
        // The refused sources took no plan slot: an eighth plan can still be built.
        check(yolo::neon_preprocess_detail::plan_status(1024, 640, 896).reason != "plan_cache_full",
              "refused sources used up the NEON plan cache");
#endif
        std::cout << "test_window_preprocess: PASS checks=" << checks << " floats_compared=" << floats_compared << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_window_preprocess: FAIL " << error.what() << '\n';
        return 1;
    }
}
