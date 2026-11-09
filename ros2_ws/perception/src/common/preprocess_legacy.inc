/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 *
 * Description: YOLO input image preprocessing implementation.
 *   1. Performs aspect-ratio preserving letterbox resize.
 *   2. Produces normalized BGR channel-first float tensors.
 */

#include "common/preprocess.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace yolo {

static constexpr int LETTERBOX_PAD_VALUE = 114;

void preprocessInto(
    const cv::Mat& image,
    int input_size,
    LetterboxInfo& info,
    float* output_data,
    std::size_t output_count) {
    if (image.empty()) {
        throw std::invalid_argument("preprocess received an empty image");
    }
    if (input_size <= 0) {
        throw std::invalid_argument("preprocess input_size must be positive");
    }
    if (image.channels() != 3) {
        throw std::invalid_argument("preprocess expects a 3-channel BGR image");
    }
    const std::size_t plane_size =
        static_cast<std::size_t>(input_size) * static_cast<std::size_t>(input_size);
    const std::size_t expected_count = 3U * plane_size;
    if (output_data == nullptr) {
        throw std::invalid_argument("preprocess output buffer must not be null");
    }
    if (output_count != expected_count) {
        throw std::invalid_argument("preprocess output buffer has an unexpected element count");
    }

    const int img_w = image.cols;
    const int img_h = image.rows;
    info.original_width = img_w;
    info.original_height = img_h;
    info.scale = std::min(
        static_cast<float>(input_size) / static_cast<float>(img_w),
        static_cast<float>(input_size) / static_cast<float>(img_h));
    info.new_width = std::max(1, static_cast<int>(std::round(static_cast<float>(img_w) * info.scale)));
    info.new_height = std::max(1, static_cast<int>(std::round(static_cast<float>(img_h) * info.scale)));
    info.pad_left = (input_size - info.new_width) / 2;
    info.pad_top = (input_size - info.new_height) / 2;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(info.new_width, info.new_height), 0.0, 0.0, cv::INTER_LINEAR);

    // YOLO models expect RGB input; OpenCV captures BGR - convert before building the tensor.
    cv::Mat resized_rgb;
    cv::cvtColor(resized, resized_rgb, cv::COLOR_BGR2RGB);

    cv::Mat canvas(
        input_size,
        input_size,
        CV_8UC3,
        cv::Scalar(LETTERBOX_PAD_VALUE, LETTERBOX_PAD_VALUE, LETTERBOX_PAD_VALUE));
    const cv::Rect roi(info.pad_left, info.pad_top, info.new_width, info.new_height);
    resized_rgb.copyTo(canvas(roi));

    cv::Mat float_canvas;
    canvas.convertTo(float_canvas, CV_32FC3, 1.0 / 255.0);

    std::array<cv::Mat, 3> output_channels{
        cv::Mat(input_size, input_size, CV_32FC1, output_data),
        cv::Mat(input_size, input_size, CV_32FC1, output_data + plane_size),
        cv::Mat(input_size, input_size, CV_32FC1, output_data + 2U * plane_size)};
    cv::split(float_canvas, output_channels.data());
    for (std::size_t channel = 0; channel < output_channels.size(); ++channel) {
        if (output_channels[channel].ptr<float>() != output_data + channel * plane_size) {
            throw std::runtime_error("OpenCV replaced the caller-owned preprocess output plane");
        }
    }
}

std::vector<float> preprocess(const cv::Mat& image, int input_size, LetterboxInfo& info) {
    if (input_size <= 0) {
        throw std::invalid_argument("preprocess input_size must be positive");
    }
    const std::size_t plane_size =
        static_cast<std::size_t>(input_size) * static_cast<std::size_t>(input_size);
    std::vector<float> output(3U * plane_size);
    preprocessInto(image, input_size, info, output.data(), output.size());
    return output;
}

}  // namespace yolo
