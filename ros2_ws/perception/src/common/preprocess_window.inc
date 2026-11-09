/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: preprocess_window.inc
 * Brief: Letterbox preprocessing into a model input that is a window of the square letterbox.
 *
 * Description:
 * What problem this solves. In the 896 x 896 Active12 input the 1280 x 800 camera image has 168
 * grey rows above and below it. The 896 x 608 input keeps rows 160..767 of that letterbox (8 grey
 * rows above the image, 40 below; see active12_seg_model_spec in postprocess.hpp), and the DLA
 * time follows the row count. This file produces such a window tensor.
 *
 * How it stays identical. The window tensor is, by definition, those rows of the square tensor.
 * The reference therefore runs the frozen legacy square path and copies the rows. The one-pass
 * version repeats the fused path (same geometry expressions, same cv::resize call, same
 * value * (1 / 255) in float) for the window's rows only; the tests compare both with the rows of
 * the frozen legacy tensor bit for bit.
 *
 * Not a frozen copy: unlike preprocess_legacy.inc and preprocess_fused.inc this is maintained code.
 * It is an .inc so that every target that compiles preprocess.cpp gets it, with that file's
 * -fno-fast-math (like preprocess_neon.inc).
 */

#include "window_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace yolo {
namespace {

// Must remain a binary32 multiplication, as in preprocess_fused.inc.
constexpr float kWindowByteScale = static_cast<float>(1.0 / 255.0);
constexpr float kWindowPad = 114.0F * kWindowByteScale;

// The window itself and the caller's buffer; images are checked by the square code.
void require_window(ModelInputShape shape, int window_top, const float* output_data, std::size_t output_count) {
    if (shape.width <= 0 || shape.height <= 0) {
        throw std::invalid_argument("preprocess input shape must be positive");
    }
    if (window_top < 0 || window_top + shape.height > shape.width) {
        throw std::invalid_argument("preprocess input window does not lie inside the square letterbox");
    }
    if (output_data == nullptr) {
        throw std::invalid_argument("preprocess output buffer must not be null");
    }
    if (output_count != 3U * static_cast<std::size_t>(shape.width) * static_cast<std::size_t>(shape.height)) {
        throw std::invalid_argument("preprocess output buffer has an unexpected element count");
    }
}

// The square letterbox seen through the window. An image whose content is cut by the window is
// refused: the model would get a picture no caller asked for.
void to_window(LetterboxInfo& info, ModelInputShape shape, int window_top) {
    if (info.pad_top < window_top || info.pad_top + info.new_height > window_top + shape.height) {
        throw std::invalid_argument("preprocess image content does not lie inside the model input window");
    }
    info.pad_top -= window_top;
}

}  // namespace

void preprocess_window_into_reference(const cv::Mat& image, ModelInputShape shape, int window_top,
                                      LetterboxInfo& info, float* output_data, std::size_t output_count) {
    require_window(shape, window_top, output_data, output_count);
    const std::size_t side = static_cast<std::size_t>(shape.width);
    // The whole square tensor first (about 9.6 MB for 896); kept per thread between calls.
    thread_local std::vector<float> square;
    square.resize(3U * side * side);
    LetterboxInfo square_info;
    preprocess_into_legacy(image, shape.width, square_info, square.data(), square.size());
    to_window(square_info, shape, window_top);
    const std::size_t window_plane = side * static_cast<std::size_t>(shape.height);
    for (std::size_t channel = 0; channel < 3U; ++channel) {
        std::memcpy(output_data + channel * window_plane,
                    square.data() + channel * side * side + static_cast<std::size_t>(window_top) * side,
                    window_plane * sizeof(float));
    }
    info = square_info;
}

void preprocess_window_into_fused(const cv::Mat& image, ModelInputShape shape, int window_top,
                                  LetterboxInfo& info, float* output_data, std::size_t output_count) {
    if (image.type() != CV_8UC3) {
        preprocess_window_into_reference(image, shape, window_top, info, output_data, output_count);
        return;
    }
    if (image.empty()) {
        throw std::invalid_argument("preprocess received an empty image");
    }
    require_window(shape, window_top, output_data, output_count);

    // The square letterbox geometry: the same expressions as preprocess_fused.inc.
    const int side = shape.width;
    const int img_w = image.cols;
    const int img_h = image.rows;
    LetterboxInfo square;
    square.original_width = img_w;
    square.original_height = img_h;
    square.scale = std::min(
        static_cast<float>(side) / static_cast<float>(img_w),
        static_cast<float>(side) / static_cast<float>(img_h));
    square.new_width = std::max(1, static_cast<int>(std::round(static_cast<float>(img_w) * square.scale)));
    square.new_height = std::max(1, static_cast<int>(std::round(static_cast<float>(img_h) * square.scale)));
    square.pad_left = (side - square.new_width) / 2;
    square.pad_top = (side - square.new_height) / 2;
    LetterboxInfo window = square;
    to_window(window, shape, window_top);

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(square.new_width, square.new_height), 0.0, 0.0, cv::INTER_LINEAR);

    const std::size_t plane_size = static_cast<std::size_t>(side) * static_cast<std::size_t>(shape.height);
    for (int y = 0; y < shape.height; ++y) {
        const std::size_t row_offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(side);
        float* const red = output_data + row_offset;
        float* const green = red + plane_size;
        float* const blue = green + plane_size;
        const int source_y = y - window.pad_top;
        if (source_y < 0 || source_y >= window.new_height) {
            std::fill_n(red, side, kWindowPad);
            std::fill_n(green, side, kWindowPad);
            std::fill_n(blue, side, kWindowPad);
            continue;
        }
        std::fill_n(red, window.pad_left, kWindowPad);
        std::fill_n(green, window.pad_left, kWindowPad);
        std::fill_n(blue, window.pad_left, kWindowPad);
        const cv::Vec3b* const source = resized.ptr<cv::Vec3b>(source_y);
        for (int x = 0; x < window.new_width; ++x) {
            const int destination_x = window.pad_left + x;
            red[destination_x] = static_cast<float>(source[x][2]) * kWindowByteScale;
            green[destination_x] = static_cast<float>(source[x][1]) * kWindowByteScale;
            blue[destination_x] = static_cast<float>(source[x][0]) * kWindowByteScale;
        }
        const int right_start = window.pad_left + window.new_width;
        std::fill(red + right_start, red + side, kWindowPad);
        std::fill(green + right_start, green + side, kWindowPad);
        std::fill(blue + right_start, blue + side, kWindowPad);
    }
    info = window;
}

}  // namespace yolo
