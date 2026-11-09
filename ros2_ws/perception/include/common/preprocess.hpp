/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: preprocess.hpp
 * Brief: YOLO input image preprocessing utilities.
 *
 * Description:
 * What problem this solves. Both models take a square input (640 and 896) while the camera image
 * is 1280 x 800. The letterbox keeps the aspect ratio: scale = min(S / w, S / h), the resized image
 * is centred, the border filled with grey 114, channels reordered BGR -> RGB, values scaled to
 * [0, 1] and written as three planes (NCHW). LetterboxInfo records the transform so the
 * postprocessors can map boxes and masks back to the original image. A model input may also be a
 * window of that square: a band of its rows that still holds the whole image (preprocess_window_into).
 *
 * Boundaries. Which implementation runs (legacy, fused, shadow) is chosen in preprocess_mode.hpp;
 * all of them give the same bits.
 */

#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace yolo {

// A model input tensor [1, 3, height, width]. COCO80 is square (640). Active12 is 896 x 896 or
// 896 wide and 608 high; only shapes with a sealed contract are accepted, see
// active12_seg_model_spec(ModelInputShape) in postprocess.hpp.
struct ModelInputShape {
    int width = 0;
    int height = 0;
};

// original = (input - pad) / scale; new_width/new_height is the resized image inside the model
// input.
struct LetterboxInfo {
    float scale = 1.0F;
    int pad_left = 0;
    int pad_top = 0;
    int new_width = 0;
    int new_height = 0;
    int original_width = 0;
    int original_height = 0;
};

/**
 * @brief Letterbox resize, normalize to [0, 1], and convert BGR HWC to CHW.
 *
 * @param image Input BGR cv::Mat with any positive size.
 * @param input_size Square model input size, for example 640.
 * @param info Output letterbox metadata used to restore original coordinates.
 * @return Float tensor data with shape [1, 3, input_size, input_size].
 */
std::vector<float> preprocess(const cv::Mat& image, int input_size, LetterboxInfo& info);

/**
 * @brief Preprocess directly into an existing contiguous NCHW float buffer.
 *
 * The destination must contain exactly 3 * input_size * input_size floats and
 * remains owned by the caller. This is the allocation-free output form used by
 * the TensorRT pinned-input path.
 */
void preprocess_into(
    const cv::Mat& image,
    int input_size,
    LetterboxInfo& info,
    float* output_data,
    std::size_t output_count);

/**
 * @brief Preprocess into a model input that is a window of the square letterbox.
 *
 * The tensor [1, 3, shape.height, shape.width] is rows [window_top, window_top + shape.height) of
 * the shape.width x shape.width tensor preprocess_into() gives for the same image, bit for bit,
 * in every preprocessing mode. info is the square letterbox seen through the window
 * (pad_top = square pad_top - window_top). Throws std::invalid_argument if the window does not
 * lie inside the square or the image content does not lie inside the window. The whole square
 * (window_top 0, height == width) is preprocess_into() itself.
 */
void preprocess_window_into(
    const cv::Mat& image,
    ModelInputShape shape,
    int window_top,
    LetterboxInfo& info,
    float* output_data,
    std::size_t output_count);

}  // namespace yolo
