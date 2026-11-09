/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_preprocess.cpp
 * Brief: Unit tests for the letterbox preprocess: scale, padding, colour order and legacy parity.
 *
 * Description:
 * What it checks. Letterbox scale and padding (value 114) for the 640 COCO and 896 active inputs;
 * BGR becomes RGB; odd padding leaves the extra pixel at the bottom/right (Ultralytics rule); the
 * caller-owned output path is byte-identical to the legacy split-then-memcpy implementation for
 * both model sizes and several aspect ratios; a too-small output buffer is refused.
 */

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/preprocess.hpp"

namespace {

bool near(float actual, float expected, float eps = 1.0e-5F) {
    return std::fabs(actual - expected) <= eps;
}

std::vector<float> legacy_preprocess_reference(
    const cv::Mat& image,
    int input_size,
    yolo::LetterboxInfo& info) {
    const int img_w = image.cols;
    const int img_h = image.rows;
    info.original_width = img_w;
    info.original_height = img_h;
    info.scale = std::min(
        static_cast<float>(input_size) / static_cast<float>(img_w),
        static_cast<float>(input_size) / static_cast<float>(img_h));
    info.new_width =
        std::max(1, static_cast<int>(std::round(static_cast<float>(img_w) * info.scale)));
    info.new_height =
        std::max(1, static_cast<int>(std::round(static_cast<float>(img_h) * info.scale)));
    info.pad_left = (input_size - info.new_width) / 2;
    info.pad_top = (input_size - info.new_height) / 2;

    cv::Mat resized;
    cv::resize(
        image,
        resized,
        cv::Size(info.new_width, info.new_height),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    cv::Mat resized_rgb;
    cv::cvtColor(resized, resized_rgb, cv::COLOR_BGR2RGB);
    cv::Mat canvas(
        input_size,
        input_size,
        CV_8UC3,
        cv::Scalar(114, 114, 114));
    resized_rgb.copyTo(
        canvas(cv::Rect(info.pad_left, info.pad_top, info.new_width, info.new_height)));
    cv::Mat float_canvas;
    canvas.convertTo(float_canvas, CV_32FC3, 1.0 / 255.0);
    std::vector<cv::Mat> channels;
    cv::split(float_canvas, channels);

    const int plane_size = input_size * input_size;
    std::vector<float> output(static_cast<std::size_t>(3 * plane_size));
    for (int channel = 0; channel < 3; ++channel) {
        if (!channels[channel].isContinuous()) {
            channels[channel] = channels[channel].clone();
        }
        std::memcpy(
            output.data() + channel * plane_size,
            channels[channel].ptr<float>(),
            static_cast<std::size_t>(plane_size) * sizeof(float));
    }
    return output;
}

void assert_same_info(
    const yolo::LetterboxInfo& actual,
    const yolo::LetterboxInfo& expected) {
    assert(actual.scale == expected.scale);
    assert(actual.pad_left == expected.pad_left);
    assert(actual.pad_top == expected.pad_top);
    assert(actual.new_width == expected.new_width);
    assert(actual.new_height == expected.new_height);
    assert(actual.original_width == expected.original_width);
    assert(actual.original_height == expected.original_height);
}

cv::Mat patterned_image(int rows, int cols) {
    cv::Mat image(rows, cols, CV_8UC3);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            image.at<cv::Vec3b>(row, col) = cv::Vec3b(
                static_cast<unsigned char>((17 * col + 3 * row) & 0xff),
                static_cast<unsigned char>((5 * col + 11 * row + 7) & 0xff),
                static_cast<unsigned char>((13 * col + 19 * row + 29) & 0xff));
        }
    }
    return image;
}

void assert_bit_exact(const cv::Mat& image, int input_size) {
    yolo::LetterboxInfo reference_info;
    const auto reference = legacy_preprocess_reference(image, input_size, reference_info);

    std::vector<float> direct(reference.size(), -1.0F);
    yolo::LetterboxInfo direct_info;
    yolo::preprocess_into(
        image,
        input_size,
        direct_info,
        direct.data(),
        direct.size());
    assert_same_info(direct_info, reference_info);
    assert(std::memcmp(
               direct.data(),
               reference.data(),
               reference.size() * sizeof(float)) == 0);

    yolo::LetterboxInfo wrapper_info;
    const auto wrapper = yolo::preprocess(image, input_size, wrapper_info);
    assert_same_info(wrapper_info, reference_info);
    assert(wrapper.size() == reference.size());
    assert(std::memcmp(
               wrapper.data(),
               reference.data(),
               reference.size() * sizeof(float)) == 0);
}

}  // namespace

int main() {
    constexpr int input_size = 640;
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));

    yolo::LetterboxInfo info;
    const std::vector<float> output = yolo::preprocess(image, input_size, info);

    assert(near(info.scale, 1.0F));
    assert(info.new_width == 640);
    assert(info.new_height == 480);
    assert(info.pad_left == 0);
    assert(info.pad_top == 80);
    assert(output.size() == static_cast<std::size_t>(1 * 3 * input_size * input_size));

    const int plane_size = input_size * input_size;
    const float padding_value = 114.0F / 255.0F;
    const int padding_index = 10 * input_size + 20;
    assert(near(output[padding_index], padding_value));
    assert(near(output[plane_size + padding_index], padding_value));
    assert(near(output[2 * plane_size + padding_index], padding_value));

    // After BGR->RGB conversion: channel 0=R=255, channel 1=G=0, channel 2=B=0
    const int image_index = info.pad_top * input_size + 20;
    assert(near(output[image_index], 1.0F));                    // R channel = 255/255
    assert(near(output[plane_size + image_index], 0.0F));       // G channel = 0
    assert(near(output[2 * plane_size + image_index], 0.0F));   // B channel = 0

    // Sealed active12 path: a 1920x1080 RGB frame becomes 896x504 with
    // centered 114 padding (196 rows on both sides).
    constexpr int active_input_size = 896;
    cv::Mat active_image(1080, 1920, CV_8UC3, cv::Scalar(10, 20, 30));
    yolo::LetterboxInfo active_info;
    const std::vector<float> active_output =
        yolo::preprocess(active_image, active_input_size, active_info);
    assert(near(active_info.scale, 896.0F / 1920.0F));
    assert(active_info.new_width == 896);
    assert(active_info.new_height == 504);
    assert(active_info.pad_left == 0);
    assert(active_info.pad_top == 196);
    assert(
        active_output.size() ==
        static_cast<std::size_t>(3 * active_input_size * active_input_size));

    const int active_plane = active_input_size * active_input_size;
    const int active_padding_index = 20 * active_input_size + 20;
    assert(near(active_output[active_padding_index], padding_value));
    assert(near(active_output[active_plane + active_padding_index], padding_value));
    assert(near(active_output[2 * active_plane + active_padding_index], padding_value));

    const int active_image_index =
        (active_info.pad_top + 100) * active_input_size + 100;
    assert(near(active_output[active_image_index], 30.0F / 255.0F));
    assert(near(active_output[active_plane + active_image_index], 20.0F / 255.0F));
    assert(near(active_output[2 * active_plane + active_image_index], 10.0F / 255.0F));

    // Ultralytics-compatible odd padding: top/left take floor(total/2), so
    // the unmatched pixel remains on bottom/right.
    cv::Mat odd_image(480, 641, CV_8UC3, cv::Scalar(0, 0, 0));
    yolo::LetterboxInfo odd_info;
    (void)yolo::preprocess(odd_image, active_input_size, odd_info);
    assert(odd_info.new_width == 896);
    assert(odd_info.new_height == 671);
    assert(odd_info.pad_left == 0);
    assert(odd_info.pad_top == 112);

    // The caller-owned output path must be byte-identical to the exact legacy
    // split-then-memcpy implementation for both production model sizes and
    // representative aspect ratios, including odd padding.
    const cv::Mat full_hd_pattern = patterned_image(1080, 1920);
    assert_bit_exact(full_hd_pattern, input_size);
    assert_bit_exact(full_hd_pattern, active_input_size);
    const cv::Mat gemini_pattern = patterned_image(720, 1280);
    assert_bit_exact(gemini_pattern, input_size);
    assert_bit_exact(gemini_pattern, active_input_size);
    assert_bit_exact(patterned_image(480, 641), active_input_size);
    assert_bit_exact(patterned_image(911, 317), input_size);

    bool rejected_bad_buffer = false;
    try {
        std::vector<float> too_small(3U * input_size * input_size - 1U);
        yolo::LetterboxInfo ignored;
        yolo::preprocess_into(
            image,
            input_size,
            ignored,
            too_small.data(),
            too_small.size());
    } catch (const std::invalid_argument&) {
        rejected_bad_buffer = true;
    }
    assert(rejected_bad_buffer);

    std::cout << "All preprocess tests passed!" << std::endl;
    return 0;
}
