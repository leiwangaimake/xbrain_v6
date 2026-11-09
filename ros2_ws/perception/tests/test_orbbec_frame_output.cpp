/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_orbbec_frame_output.cpp
 * Brief: The Orbbec frame output equals the former resize path and owns its BGR/Y16 memory.
 *
 * Description:
 * Compare queued output to the previous unconditional-resize path and prove
 * ownership across source mutation/destruction for borrowed BGR/Y16 memory.
 * OpenCV-only fixture: no SDK, device, model or publication is used.
 */

#include "modules/orbbec_frame_output.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace {

std::size_t assertions = 0;

void check(bool condition, const char* expression, int line) {
    ++assertions;
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check(static_cast<bool>(expression), #expression, __LINE__)

bool same_bytes(const cv::Mat& left, const cv::Mat& right) {
    if (left.size() != right.size() || left.type() != right.type()) return false;
    const auto row_bytes = static_cast<std::size_t>(left.cols) * left.elemSize();
    for (int row = 0; row < left.rows; ++row) {
        if (std::memcmp(left.ptr(row), right.ptr(row), row_bytes) != 0) return false;
    }
    return true;
}

template<typename T>
void run_case(const char* label, int type, int interpolation,
             cv::Size source_size, cv::Size output_size, std::size_t padding) {
    const auto row_elements = static_cast<std::size_t>(source_size.width) *
                              static_cast<std::size_t>(CV_MAT_CN(type));
    const auto stride_elements = row_elements + padding;
    cv::Mat output;
    cv::Mat previous_output;
    cv::Mat second_output;
    cv::Mat expected_second;
    {
        // This vector stands in for SDK-owned storage. The cv::Mat header
        // deliberately borrows it, without OpenCV refcounted ownership.
        std::vector<T> source_storage(
            stride_elements * static_cast<std::size_t>(source_size.height));
        for (std::size_t index = 0; index < source_storage.size(); ++index) {
            source_storage[index] = static_cast<T>((index * 7919U + index / 7U) &
                                                  std::numeric_limits<T>::max());
        }
        // Include invalid zero depth, maximum depth and hard edges; BGR uses
        // independent channel bytes in the same deterministic pattern.
        source_storage[0] = 0;
        source_storage[1] = std::numeric_limits<T>::max();
        cv::Mat view(source_size.height, source_size.width, type,
                     source_storage.data(), stride_elements * sizeof(T));
        CHECK(view.u == nullptr);
        CHECK(view.isContinuous() == (padding == 0));

        cv::resize(view, previous_output, output_size, 0.0, 0.0, interpolation);
        output = perception::orbbec_capture_detail::copy_or_resize_owned(
            view, output_size, interpolation);
        CHECK(output.size() == output_size);
        CHECK(output.type() == type);
        CHECK(output.u != nullptr);
        CHECK(output.data != view.data);
        CHECK(output.isContinuous());
        CHECK(same_bytes(output, previous_output));
        if (source_size == output_size) CHECK(same_bytes(output, view));

        // Simulate SDK buffer reuse for the next frame while the previous
        // output remains queued. This must not rewrite either queued output.
        std::fill(source_storage.begin(), source_storage.end(), static_cast<T>(37));
        CHECK(same_bytes(output, previous_output));
        cv::resize(view, expected_second, output_size, 0.0, 0.0, interpolation);
        second_output = perception::orbbec_capture_detail::copy_or_resize_owned(
            view, output_size, interpolation);
        CHECK(second_output.data != view.data);
        CHECK(second_output.data != output.data);
        CHECK(second_output.u != nullptr);
        CHECK(same_bytes(second_output, expected_second));
        CHECK(same_bytes(output, previous_output));

        std::fill(source_storage.begin(), source_storage.end(), static_cast<T>(0));
        CHECK(same_bytes(output, previous_output));
        CHECK(same_bytes(second_output, expected_second));
    }  // Both the borrowed Mat and its backing storage are destroyed here.
    CHECK(same_bytes(output, previous_output));
    CHECK(same_bytes(second_output, expected_second));
    std::cout << "ORBBEC_FRAME_OUTPUT_CASE name=" << label
              << " source=" << source_size.width << "x" << source_size.height
              << " output=" << output_size.width << "x" << output_size.height
              << " padded=" << (padding != 0)
              << " byte_equal=true owned_after_source_destroy=true result=PASS\n";
}

template<typename T>
void run_format(const char* label, int type, int interpolation) {
    static_assert(std::is_unsigned_v<T>);
    // Native output and a non-contiguous version cover the new copy branch.
    run_case<T>(label, type, interpolation, {1280, 800}, {1280, 800}, 0);
    run_case<T>(label, type, interpolation, {1280, 800}, {1280, 800}, 11);
    // The established 720p -> 1080p path must remain byte-identical.
    run_case<T>(label, type, interpolation, {1280, 720}, {1920, 1080}, 0);
    // Downscale a native raster and exercise padded, odd-size up/downscales.
    run_case<T>(label, type, interpolation, {1280, 800}, {640, 400}, 0);
    run_case<T>(label, type, interpolation, {19, 13}, {7, 5}, 11);
    run_case<T>(label, type, interpolation, {19, 13}, {31, 23}, 11);
}

}  // namespace

int main() {
    try {
        run_format<std::uint8_t>("BGR", CV_8UC3, cv::INTER_LINEAR);
        run_format<std::uint16_t>("Y16", CV_16UC1, cv::INTER_NEAREST_EXACT);
        std::cout << "ORBBEC_FRAME_OUTPUT_RESULT opencv=" << CV_VERSION
                  << " assertions=" << assertions << " result=PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ORBBEC_FRAME_OUTPUT_RESULT result=FAIL error="
                  << error.what() << '\n';
        return 1;
    }
}
