#include "fused_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace yolo_prototype {
namespace {
// Must remain a binary32 multiplication, not division or fast-math. Its
// equivalence to this host's OpenCV convertTo is tested for all 256 byte values;
// another backend/toolchain needs the same independent check.
constexpr float kByteScale = static_cast<float>(1.0 / 255.0);
constexpr float kPad = 114.0F * kByteScale;
}  // namespace

void preprocessIntoFused(const cv::Mat& image, int input_size,
                         yolo::LetterboxInfo& info, float* output_data,
                         std::size_t output_count) {
    if (image.type() != CV_8UC3) {
        yolo::preprocessInto(image, input_size, info, output_data, output_count);
        return;
    }
    if (image.empty()) {
        throw std::invalid_argument("preprocess received an empty image");
    }
    if (input_size <= 0) {
        throw std::invalid_argument("preprocess input_size must be positive");
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

    // Geometry and resize intentionally duplicate the original expressions.
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

    // No RGB Mat, padded Mat, float Mat or split: each destination element is
    // written once. Row pointers retain correctness for stepped input storage.
    for (int y = 0; y < input_size; ++y) {
        const std::size_t row_offset = static_cast<std::size_t>(y) *
                                       static_cast<std::size_t>(input_size);
        float* const red = output_data + row_offset;
        float* const green = red + plane_size;
        float* const blue = green + plane_size;
        const int source_y = y - info.pad_top;
        if (source_y < 0 || source_y >= info.new_height) {
            std::fill_n(red, input_size, kPad);
            std::fill_n(green, input_size, kPad);
            std::fill_n(blue, input_size, kPad);
            continue;
        }
        std::fill_n(red, info.pad_left, kPad);
        std::fill_n(green, info.pad_left, kPad);
        std::fill_n(blue, info.pad_left, kPad);
        const cv::Vec3b* const source = resized.ptr<cv::Vec3b>(source_y);
        for (int x = 0; x < info.new_width; ++x) {
            const int destination_x = info.pad_left + x;
            red[destination_x] = static_cast<float>(source[x][2]) * kByteScale;
            green[destination_x] = static_cast<float>(source[x][1]) * kByteScale;
            blue[destination_x] = static_cast<float>(source[x][0]) * kByteScale;
        }
        const int right_start = info.pad_left + info.new_width;
        std::fill(red + right_start, red + input_size, kPad);
        std::fill(green + right_start, green + input_size, kPad);
        std::fill(blue + right_start, blue + input_size, kPad);
    }
}

}  // namespace yolo_prototype
