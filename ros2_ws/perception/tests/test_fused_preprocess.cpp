/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_fused_preprocess.cpp
 * Brief: Unit tests for the fused letterbox preprocess against the frozen legacy implementation.
 *
 * Description:
 * What it checks. The fused preprocess must produce exactly the same letterbox info and normalised
 * pixels as the frozen original API (tests/reference) for every case, so it can replace the legacy
 * path without changing model input.
 */

#include "fused_preprocess.hpp"
#include "reference/preprocess_legacy_api.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
std::string quote(const std::string& value) {
    std::string result = "\"";
    for (unsigned char c : value) {
        if (c == '\\' || c == '"') { result += '\\'; result += static_cast<char>(c); }
        else if (c == '\n') { result += "\\n"; }
        else if (c == '\r') { result += "\\r"; }
        else if (c == '\t') { result += "\\t"; }
        else if (c >= 32U) { result += static_cast<char>(c); }
    }
    return result + "\"";
}
std::uint32_t bits(float value) {
    std::uint32_t result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}
struct Audit {
    std::size_t checks = 0;
    std::size_t failed = 0;
    std::size_t floats_compared = 0;
    std::size_t different_float_elements = 0;
    std::vector<std::string> errors;
    std::vector<std::string> cases;
    void require(bool condition, const std::string& name) {
        ++checks;
        if (!condition) {
            ++failed;
            if (errors.size() < 100U) { errors.push_back(name); }
        }
    }
} audit;

bool same_info(const yolo::LetterboxInfo& a, const yolo::LetterboxInfo& b) {
    return bits(a.scale) == bits(b.scale) && a.pad_left == b.pad_left &&
           a.pad_top == b.pad_top && a.new_width == b.new_width &&
           a.new_height == b.new_height && a.original_width == b.original_width &&
           a.original_height == b.original_height;
}
std::size_t difference_count(const float* a, const float* b, std::size_t size) {
    std::size_t different = 0;
    for (std::size_t i = 0; i < size; ++i) {
        if (bits(a[i]) != bits(b[i])) { ++different; }
    }
    return different;
}
bool same_pixels(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size() || a.type() != b.type()) { return false; }
    for (int row = 0; row < a.rows; ++row) {
        if (std::memcmp(a.ptr(row), b.ptr(row), static_cast<std::size_t>(a.cols) * a.elemSize()) != 0) {
            return false;
        }
    }
    return true;
}
cv::Mat pattern(int width, int height, std::uint32_t salt = 0) {
    cv::Mat image(height, width, CV_8UC3);
    std::mt19937 random(9051337U + salt);
    for (int y = 0; y < height; ++y) {
        auto* row = image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; ++x) {
            for (int c = 0; c < 3; ++c) { row[x][c] = static_cast<unsigned char>(random() & 255U); }
        }
    }
    return image;
}
using Function = void (*)(const cv::Mat&, int, yolo::LetterboxInfo&, float*, std::size_t);
std::string invoke(Function fn, const cv::Mat& image, int size,
                   yolo::LetterboxInfo& info, float* output, std::size_t count) {
    try { fn(image, size, info, output, count); return "none"; }
    catch (const cv::Exception& error) { return "cv::Exception:" + std::string(error.what()); }
    catch (const std::invalid_argument& error) { return "invalid_argument:" + std::string(error.what()); }
    catch (const std::runtime_error& error) { return "runtime_error:" + std::string(error.what()); }
    catch (const std::exception& error) { return "other_std_exception:" + std::string(error.what()); }
}

void run_case(const std::string& name, const cv::Mat& image, int size,
             int repeats = 1, int count_delta = 0, bool null_output = false,
             bool expected_rejection = false, bool oracle_outcome_only = false) {
    constexpr std::size_t guard_size = 16;
    constexpr float sentinel = 12345.25F;
    const std::size_t count = size > 0 ? 3U * static_cast<std::size_t>(size) *
                                           static_cast<std::size_t>(size) : 3U;
    const auto provided = static_cast<std::size_t>(static_cast<std::int64_t>(count) + count_delta);
    std::vector<float> old_buffer(count + 2U * guard_size, sentinel);
    std::vector<float> new_buffer(count + 2U * guard_size, sentinel);
    const cv::Mat source_copy = image.clone();
    std::string old_exception;
    std::string new_exception;
    std::size_t differences = 0;
    yolo::LetterboxInfo last_info;
    for (int iteration = 0; iteration < repeats; ++iteration) {
        // Different dirty outputs detect unwritten elements, not just equality.
        std::fill_n(old_buffer.data() + guard_size, count, -123.0F);
        std::fill_n(new_buffer.data() + guard_size, count, 456.0F);
        yolo::LetterboxInfo old_info{3.25F, 51, 52, 53, 54, 55, 56};
        yolo::LetterboxInfo new_info = old_info;
        old_exception = invoke(yolo::preprocess_into_legacy, image, size, old_info,
                               null_output ? nullptr : old_buffer.data() + guard_size, provided);
        new_exception = invoke(yolo_prototype::preprocess_into_fused, image, size, new_info,
                               null_output ? nullptr : new_buffer.data() + guard_size, provided);
        last_info = old_info;
        audit.require(old_exception == new_exception, name + ": exception parity");
        audit.require(same_info(old_info, new_info), name + ": exact LetterboxInfo");
        audit.require(same_pixels(image, source_copy), name + ": input unchanged");
        if (!oracle_outcome_only) {
            audit.require(expected_rejection ? old_exception != "none" : old_exception == "none",
                          name + ": expected outcome");
        }
        if (old_exception == "none" && new_exception == "none") {
            differences = difference_count(old_buffer.data() + guard_size,
                                          new_buffer.data() + guard_size, count);
            audit.floats_compared += count;
            audit.different_float_elements += differences;
            audit.require(differences == 0U, name + ": complete tensor bitwise equality");
            if (differences != 0U) {
                for (std::size_t i = 0; i < count && audit.errors.size() < 100U; ++i) {
                    if (bits(old_buffer[i + guard_size]) != bits(new_buffer[i + guard_size])) {
                        audit.errors.push_back(name + ": index=" + std::to_string(i) +
                            " old_bits=" + std::to_string(bits(old_buffer[i + guard_size])) +
                            " new_bits=" + std::to_string(bits(new_buffer[i + guard_size])));
                    }
                }
            }
        }
        for (const auto* buffer : {&old_buffer, &new_buffer}) {
            bool guards_ok = true;
            for (std::size_t g = 0; g < guard_size; ++g) {
                guards_ok = guards_ok && bits((*buffer)[g]) == bits(sentinel) &&
                    bits((*buffer)[guard_size + count + g]) == bits(sentinel);
            }
            audit.require(guards_ok, name + ": caller buffer prefix/tail guards");
        }
    }
    std::ostringstream item;
    item << "{\"name\":" << quote(name) << ",\"width\":" << image.cols
         << ",\"height\":" << image.rows << ",\"type\":" << image.type()
         << ",\"input_contiguous\":" << (image.isContinuous() ? "true" : "false")
         << ",\"model_size\":" << size << ",\"repeats\":" << repeats
         << ",\"last_different_elements\":" << differences
         << ",\"geometry\":{\"scale_bits\":" << bits(last_info.scale)
         << ",\"new_width\":" << last_info.new_width << ",\"new_height\":" << last_info.new_height
         << ",\"pad_left\":" << last_info.pad_left << ",\"pad_top\":" << last_info.pad_top << "}"
         << ",\"route\":" << quote(image.type() == CV_8UC3 ? "fused_8u" : "legacy_fallback")
         << ",\"exception\":" << quote(old_exception) << "}";
    audit.cases.push_back(item.str());
}
}  // namespace

int main() {
    try {
#ifndef PROBE_OPENCV_THREADS
#define PROBE_OPENCV_THREADS 1
#endif
        cv::setNumThreads(PROBE_OPENCV_THREADS);
        cv::Mat bytes(1, 256, CV_8UC1);
        for (int i = 0; i < 256; ++i) { bytes.at<unsigned char>(0, i) = static_cast<unsigned char>(i); }
        cv::Mat normalized;
        bytes.convertTo(normalized, CV_32FC1, 1.0 / 255.0);
        for (int i = 0; i < 256; ++i) {
            const float candidate = static_cast<float>(i) * static_cast<float>(1.0 / 255.0);
            audit.require(bits(candidate) == bits(normalized.at<float>(0, i)),
                          "byte_normalization_" + std::to_string(i) + " old_bits=" +
                          std::to_string(bits(normalized.at<float>(0, i))) + " new_bits=" +
                          std::to_string(bits(candidate)));
        }
        cv::Mat all_values(16, 16, CV_8UC3);
        for (int i = 0; i < 256; ++i) {
            all_values.at<cv::Vec3b>(i / 16, i % 16) = cv::Vec3b(
                static_cast<unsigned char>(i), static_cast<unsigned char>((i + 37) % 256),
                static_cast<unsigned char>(255 - i));
        }
        run_case("all_256_values_rgb_channels", all_values, 16, 2);
        run_case("one_pixel_odd_bottom_pad", pattern(3,2), 4);
        const cv::Mat native = pattern(1280, 800);
        run_case("native1280x800_to640_repeated", native, 640, 3);
        run_case("native1280x800_to896_repeated", native, 896, 3);
        for (const auto& dimensions : {std::pair<int,int>{513, 205}, {205, 513}, {641, 479},
                                     {479, 641}, {17, 13}, {13, 17}, {1, 9}, {9, 1}, {1, 1}}) {
            const cv::Mat input = pattern(dimensions.first, dimensions.second, 1);
            for (int size : {640, 896}) {
                run_case("odd_" + std::to_string(dimensions.first) + "x" +
                        std::to_string(dimensions.second) + "_to" + std::to_string(size), input, size);
            }
        }
        cv::Mat roi_owner = pattern(1293, 807, 2);
        cv::Mat roi = roi_owner(cv::Rect(5, 3, 1280, 800));
        const cv::Mat original_owner = roi_owner.clone();
        audit.require(!roi.isContinuous(), "ROI must be non-contiguous");
        run_case("noncontinuous_native_roi_to640", roi, 640, 2);
        run_case("noncontinuous_native_roi_to896", roi, 896, 2);
        audit.require(same_pixels(roi_owner, original_owner), "entire ROI owner unchanged");
        run_case("all_pad_value", cv::Mat(205, 513, CV_8UC3, cv::Scalar(114,114,114)), 640);
        run_case("black", cv::Mat(33, 47, CV_8UC3, cv::Scalar(0,0,0)), 64);
        run_case("white", cv::Mat(47, 33, CV_8UC3, cv::Scalar(255,255,255)), 65);
        run_case("empty", cv::Mat(), 32, 1, 0, false, true);
        run_case("zero_model_size", all_values, 0, 1, 0, false, true);
        run_case("negative_model_size", all_values, -1, 1, 0, false, true);
        run_case("one_channel", bytes, 32, 1, 0, false, true);
        run_case("four_channels", cv::Mat(16,16,CV_8UC4), 32, 1, 0, false, true);
        run_case("null_output", all_values, 32, 1, 0, true, true);
        run_case("short_output_count", all_values, 32, 1, -1, false, true);
        run_case("long_output_count", all_values, 32, 1, 1, false, true);
        // The old API does not declare an 8-bit depth restriction. Preserve
        // its actual OpenCV acceptance or rejection by delegating, not guessing.
        run_case("uint16_legacy_fallback", cv::Mat(16,16,CV_16UC3,cv::Scalar(1,2,3)), 32,
                1, 0, false, false, true);
        run_case("float32_legacy_fallback", cv::Mat(16,16,CV_32FC3,cv::Scalar(1,2,3)), 32,
                1, 0, false, false, true);

        // Negative control: comparator must distinguish a single one-ULP edit.
        std::array<float,3> original{0.0F, 0.5F, 1.0F};
        auto altered = original;
        altered[1] = std::nextafter(altered[1], 1.0F);
        audit.require(difference_count(original.data(), altered.data(), original.size()) == 1U,
                      "one_ULP_negative_control");
        const float negative_zero = -0.0F;
        audit.require(difference_count(original.data(), &negative_zero, 1U) == 1U,
                      "signed_zero_negative_control");
    } catch (const std::exception& error) {
        audit.require(false, "uncaught: " + std::string(error.what()));
    }
    std::cout << "{\"status\":" << quote(audit.failed == 0U ? "LOCAL_SYNTHETIC_BITWISE_PASS" : "FAIL")
              << ",\"opencv_version\":" << quote(CV_VERSION)
              << ",\"opencv_threads\":" << cv::getNumThreads()
              << ",\"checks\":" << audit.checks << ",\"failed_checks\":" << audit.failed
              << ",\"float_elements_compared\":" << audit.floats_compared
              << ",\"different_float_elements\":" << audit.different_float_elements
              << ",\"normalization_byte_values\":256,\"cases\":[";
    for (std::size_t i = 0; i < audit.cases.size(); ++i) { if (i) { std::cout << ','; } std::cout << audit.cases[i]; }
    std::cout << "],\"errors\":[";
    for (std::size_t i = 0; i < audit.errors.size(); ++i) { if (i) { std::cout << ','; } std::cout << quote(audit.errors[i]); }
    std::cout << "],\"real_images\":0,\"device_access\":false,\"production_integrated\":false,"
                 "\"speed_measured\":false,\"target_validation\":\"NOT_RUN\"}\n";
    return audit.failed == 0U ? 0 : 1;
}
