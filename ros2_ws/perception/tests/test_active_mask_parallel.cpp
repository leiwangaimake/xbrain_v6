/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_active_mask_parallel.cpp
 * Brief: Unit tests for the active-model mask postprocess on the persistent task pair.
 *
 * Description:
 * What it checks. The parallel active mask path must match the parent postprocess: shapes, counts
 * and thresholds, NMS order and the first valid canvas (equal scores keep source tensor order),
 * reuse with independent owners, a missing worker and invalid input, the binary mode contract and
 * its primitive edges, and an exceptional prototype. A benchmark is included but is not a gate.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/postprocess.hpp"
#include "common/persistent_task_pair.hpp"
#include "active_mask_parent_postprocess.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {
using Mode = yolo::Active12MaskImplementation;
using BinaryMode = yolo::Active12MaskBinaryImplementation;
constexpr std::int64_t kBoxes = 16464;
constexpr int kChannels = 48;
constexpr int kProtoSide = 224;
constexpr int kPlane = kProtoSide * kProtoSide;
const std::vector<std::int64_t> kShape{1, kChannels, kBoxes};
const std::vector<std::int64_t> kProtoShape{1, 32, kProtoSide, kProtoSide};
std::atomic<int> fixture_checks{0};

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::uint32_t float_bits(float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

struct Fixture {
    std::string name;
    yolo::LetterboxInfo info;
    std::vector<float> output;
    std::vector<float> proto;
    float threshold = 0.5F;
};

void set_box(Fixture& f, int index, float left, float top, float width,
            float height, int local_class, float confidence) {
    const float scale = f.info.scale;
    f.output[index] = (left + width * 0.5F) * scale + f.info.pad_left;
    f.output[kBoxes + index] = (top + height * 0.5F) * scale + f.info.pad_top;
    f.output[2 * kBoxes + index] = width * scale;
    f.output[3 * kBoxes + index] = height * scale;
    f.output[(4 + local_class) * kBoxes + index] = confidence;
}

Fixture make_fixture(int width, int height, int count, unsigned seed = 12345) {
    Fixture f;
    f.name = std::to_string(width) + "x" + std::to_string(height) + "_n" + std::to_string(count);
    f.info.scale = std::min(896.0F / width, 896.0F / height);
    f.info.new_width = static_cast<int>(std::round(width * f.info.scale));
    f.info.new_height = static_cast<int>(std::round(height * f.info.scale));
    f.info.pad_left = (896 - f.info.new_width) / 2;
    f.info.pad_top = (896 - f.info.new_height) / 2;
    f.info.original_width = width;
    f.info.original_height = height;
    f.output.assign(static_cast<std::size_t>(kChannels * kBoxes), 0.0F);
    f.proto.resize(32U * kPlane);
    std::mt19937 random(seed);
    for (float& value : f.proto) {
        value = static_cast<float>(static_cast<int>(random() % 2001U) - 1000) / 1000.0F;
    }
    // Exact zeros and small signed logits coexist with all-channel random
    // prototypes; lane-local GEMM must retain its original reduction order.
    for (int channel = 0; channel < 32; ++channel) {
        f.proto[channel * kPlane] = 0.0F;
        f.proto[channel * kPlane + 1] = std::nextafter(0.0F, 1.0F);
        f.proto[channel * kPlane + 2] = std::nextafter(0.0F, -1.0F);
    }
    const int columns = std::max(1, static_cast<int>(std::ceil(std::sqrt(count))));
    const int rows = std::max(1, (count + columns - 1) / columns);
    const float cell_w = static_cast<float>(width) / columns;
    const float cell_h = static_cast<float>(height) / rows;
    for (int i = 0; i < count; ++i) {
        set_box(f, i, (i % columns + 0.1F) * cell_w,
               (i / columns + 0.1F) * cell_h, cell_w * 0.7F, cell_h * 0.7F,
               i == 0 ? 10 : i % 10, 0.99F - i * 0.001F);
        for (int channel = 0; channel < 32; ++channel) {
            f.output[(16 + channel) * kBoxes + i] =
                static_cast<float>(static_cast<int>(random() % 2001U) - 1000) / 2000.0F;
        }
    }
    return f;
}

std::vector<yolo::Detection> parent_decode(const Fixture& f) {
    return yolo_active_mask_parent::postprocess_seg(
        f.output.data(), kShape, f.proto.data(), kProtoShape, f.info,
        yolo_active_mask_parent::active12_seg_model_spec(), 0.25F, 0.70F,
        f.threshold, 896);
}

std::vector<yolo::Detection> decode(const Fixture& f, Mode mode,
                                   perception::PersistentTaskPair* lanes,
                                   yolo::Active12MaskProfile* profile = nullptr,
                                   BinaryMode binary = BinaryMode::Legacy) {
    return yolo::postprocess_seg(f.output.data(), kShape, f.proto.data(),
        kProtoShape, f.info, yolo::active12_seg_model_spec(), 0.25F, 0.70F,
        f.threshold, 896, mode, lanes, profile, binary);
}

void same_mat(const cv::Mat& lhs, const cv::Mat& rhs, const std::string& label) {
    check(lhs.empty() == rhs.empty(), label + ": empty");
    check(lhs.type() == rhs.type() && lhs.size() == rhs.size(), label + ": shape/type");
    if (lhs.empty()) return;
    check(lhs.type() == CV_8UC1, label + ": expected byte mask");
    for (int row = 0; row < lhs.rows; ++row) {
        check(std::memcmp(lhs.ptr(row), rhs.ptr(row), static_cast<std::size_t>(lhs.cols)) == 0,
              label + ": pixels at row " + std::to_string(row));
    }
}

void same(const std::vector<yolo::Detection>& lhs,
          const std::vector<yolo::Detection>& rhs, const std::string& label) {
    check(lhs.size() == rhs.size(), label + ": count");
    // Compare by output position, never sort away an NMS/order regression.
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        const auto& a = lhs[i]; const auto& b = rhs[i];
        const std::string item = label + "[" + std::to_string(i) + "]";
        check(a.bbox == b.bbox, item + ": bbox");
        check(float_bits(a.confidence) == float_bits(b.confidence), item + ": confidence bits");
        check(a.class_id == b.class_id && a.original_class_id == b.original_class_id &&
              a.class_name == b.class_name && a.source == b.source &&
              a.low_confidence_candidate == b.low_confidence_candidate, item + ": metadata");
        same_mat(a.mask, b.mask, item + ": local mask");
        same_mat(a.full_image_mask, b.full_image_mask, item + ": full mask");
    }
}

std::vector<yolo::Detection> owning_copy(std::vector<yolo::Detection> detections) {
    for (auto& detection : detections) {
        detection.mask = detection.mask.clone();
        detection.full_image_mask = detection.full_image_mask.clone();
    }
    return detections;
}

void profile_matches(const yolo::Active12MaskProfile& p,
                    const std::vector<yolo::Detection>& detections, Mode mode,
                    BinaryMode binary = BinaryMode::Legacy) {
    check(p.instances == detections.size() && p.tasks.size() == detections.size(), "profile count");
    check(p.parallel_used == (mode == Mode::Parallel2 && detections.size() >= 2), "parallel_used");
    check(p.shadow_checked == (mode == Mode::Shadow), "shadow_checked");
    check(p.binary_shadow_checked == (binary == BinaryMode::Shadow), "binary_shadow_checked");
    check(p.total_ns >= p.batch_wall_ns && p.batch_wall_ns >= 0, "profile outer bounds");
    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& t = p.tasks[i];
        check(t.index == i && t.bbox_pixels == detections[i].bbox.area(), "profile task identity");
        check(t.lane == (mode == Mode::Parallel2 ? static_cast<int>(i % 2) : 0), "profile lane");
        check(t.gemm_ns >= 0 && t.resize_ns >= 0 && t.threshold_convert_ns >= 0 &&
              t.canvas_ns >= 0 && t.total_ns >= 0, "profile negative duration");
        check(t.gemm_ns + t.resize_ns + t.threshold_convert_ns + t.canvas_ns == t.total_ns,
              "profile mutually exclusive closure");
        check(p.batch_wall_ns >= t.total_ns, "profile task outside batch");
    }
}

void compare_fixture(const Fixture& f, perception::PersistentTaskPair& lanes,
                    std::size_t expected_count) {
    const auto input_before = f.output;
    const auto proto_before = f.proto;
    const auto old_first = parent_decode(f);
    check(old_first.size() == expected_count, f.name + ": fixture expected count");
    const auto default_result = yolo::postprocess_seg(
        f.output.data(), kShape, f.proto.data(), kProtoShape, f.info,
        yolo::active12_seg_model_spec(), 0.25F, 0.70F, f.threshold, 896);
    same(old_first, default_result, f.name + ": default serial");
    for (const Mode mode : {Mode::Serial, Mode::Parallel2, Mode::Shadow}) {
      for (const BinaryMode binary : {BinaryMode::Legacy, BinaryMode::Compare, BinaryMode::Shadow}) {
        yolo::Active12MaskProfile profile;
        const auto result = decode(f, mode, &lanes, &profile, binary);
        const auto snapshot = owning_copy(result);
        same(old_first, result, f.name + ": parent differential");
        profile_matches(profile, result, mode, binary);
        const auto off = decode(f, mode, &lanes, nullptr, binary);
        same(result, off, f.name + ": profiling on/off");
        same(result, snapshot, f.name + ": owning output after reuse");
      }
    }
    same(old_first, parent_decode(f), f.name + ": old/new/old");
    check(std::memcmp(input_before.data(), f.output.data(), f.output.size() * sizeof(float)) == 0,
          f.name + ": detection input modified");
    check(std::memcmp(proto_before.data(), f.proto.data(), f.proto.size() * sizeof(float)) == 0,
          f.name + ": prototype input modified");
    ++fixture_checks;
}

// Every mode matches the frozen parent across image shapes (native 800p letterbox included),
// instance counts up to the 300 cap, thresholds around 0.5 and zero-logit edges.
void test_shapes_counts_and_thresholds(perception::PersistentTaskPair& lanes) {
    for (int count : {0, 1, 2, 3, 6, 12}) {
        compare_fixture(make_fixture(1920, 1080, count), lanes, count);
    }
    for (int count : {0, 1, 2, 3, 6, 12}) {
        const auto native = make_fixture(1280, 800, count);
        check(native.info.new_width == 896 && native.info.new_height == 560 &&
              native.info.pad_left == 0 && native.info.pad_top == 168,
              "native800 letterbox shape/padding");
        // Existing fixture compares parent/default/serial/parallel/shadow and
        // all binary modes, including output ownership after scratch reuse.
        compare_fixture(native, lanes, count);
        std::cout << "NATIVE800_ACTIVE_MASK instances=" << count
                  << " parent_serial_parallel_binary=PASS\n";
    }
    compare_fixture(make_fixture(641, 480, 3), lanes, 3);
    compare_fixture(make_fixture(896, 896, 2), lanes, 2);
    for (int count : {49, 50, 51, 300, 301}) {
        compare_fixture(make_fixture(100, 100, count), lanes, std::min(count, 300));
    }
    auto f = make_fixture(641, 480, 2);
    for (float threshold : {0.0F, std::nextafter(0.5F, 0.0F), 0.5F,
                            std::nextafter(0.5F, 1.0F), 0.70F, 0.75F, 1.0F}) {
        f.threshold = threshold;
        f.name = "probability_" + std::to_string(float_bits(threshold));
        compare_fixture(f, lanes, 2);
    }
    for (float logit : {0.0F, -0.0F, std::nextafter(0.0F, 1.0F),
                       std::nextafter(0.0F, -1.0F), 1.0F, -1.0F}) {
        std::fill(f.proto.begin(), f.proto.end(), 0.0F);
        std::fill(f.proto.begin(), f.proto.begin() + kPlane, logit);
        for (int i = 0; i < 2; ++i) for (int channel = 0; channel < 32; ++channel) {
            f.output[(16 + channel) * kBoxes + i] = channel == 0 ? 1.0F : 0.0F;
        }
        f.threshold = 0.5F;
        f.name = "zero_logit_boundary_" + std::to_string(float_bits(logit));
        compare_fixture(f, lanes, 2);
    }
}

// Equal scores keep source tensor order, NMS runs before clipping, and only the first valid
// class-92 instance gets the full-image canvas (the 49/50 crop rule included).
void test_nms_order_and_first_valid_canvas(perception::PersistentTaskPair& lanes) {
    auto f = make_fixture(896, 896, 6);
    std::fill(f.output.begin(), f.output.end(), 0.0F);
    // Equal score -> source tensor index order. Exact same-class duplicate is
    // suppressed, different-class overlap survives, and edge NMS precedes clip.
    set_box(f, 8, 400, 400, 100, 100, 1, 0.9F);
    set_box(f, 2, 200, 200, 100, 100, 1, 0.9F);
    set_box(f, 9, 200, 200, 100, 100, 1, 0.9F);
    set_box(f, 3, 200, 200, 100, 100, 7, 0.9F);
    set_box(f, 4, -100, 50, 200, 200, 1, 0.8F);
    set_box(f, 5, 0, 50, 100, 200, 1, 0.7F);
    for (int i : {2, 3, 4, 5, 8, 9}) f.output[16 * kBoxes + i] = 1.0F;
    f.name = "nms_ties_raw_edge_order";
    compare_fixture(f, lanes, 5);
    const auto order = parent_decode(f);
    check(order[0].class_id == 81 && order[0].bbox.x == 200 &&
          order[1].class_id == 89 && order[2].bbox.x == 400, "fixture stable tie order");

    for (int count : {49, 50}) {
        f = make_fixture(100, 100, count);
        std::fill(f.output.begin(), f.output.end(), 0.0F);
        // This mapped first 92 has positive float extents but no integer pixels.
        // It still counts toward mapped.size() for the 49/50 crop rule.
        set_box(f, 0, 0.1F, 0.1F, 0.1F, 0.1F, 10, 0.99F);
        for (int i = 1; i < count; ++i) {
            set_box(f, i, 1.1F + 12.0F * ((i - 1) % 8),
                   1.1F + 12.0F * ((i - 1) / 8), 4.0F, 4.0F,
                   i <= 2 ? 10 : 1, 0.99F - i * 0.001F);
            f.output[16 * kBoxes + i] = 1.0F;
        }
        f.name = "mapped_" + std::to_string(count) + "_empty_first_92";
        compare_fixture(f, lanes, count - 1);
        const auto result = decode(f, Mode::Parallel2, &lanes);
        check(result.front().bbox.x == (count == 49 ? 1 : 2), "mapped count crop boundary");
        check(result[0].class_id == 92 && !result[0].full_image_mask.empty(), "first valid 92 owns canvas");
        check(result[1].class_id == 92 && result[1].full_image_mask.empty(), "second valid 92 has no canvas");
    }
}

// Scratch reuse across sizes never overwrites returned output; instance masks and the canvas own
// their pixels; two owners on parallel threads do not interfere.
void test_reuse_and_independent_owners() {
    auto worker = [](unsigned seed) {
        perception::PersistentTaskPair lanes;
        const auto f = make_fixture(1920, 1080, 3, seed);
      for (const BinaryMode binary : {BinaryMode::Legacy, BinaryMode::Compare, BinaryMode::Shadow}) {
        auto retained = decode(f, Mode::Parallel2, &lanes, nullptr, binary);
        const auto saved = owning_copy(retained);
        for (const auto dimensions : {std::array<int, 3>{641, 480, 2}, {100, 100, 0}, {896, 896, 6}}) {
            compare_fixture(make_fixture(dimensions[0], dimensions[1], dimensions[2], seed + 1),
                           lanes, dimensions[2]);
            same(retained, saved, "cross-size scratch does not overwrite retained output");
        }
        if (retained.size() > 1) {
            retained[0].mask.at<std::uint8_t>(0, 0) ^= 255U;
            for (std::size_t i = 1; i < retained.size(); ++i) {
                same_mat(retained[i].mask, saved[i].mask, "instance masks do not alias");
            }
            same_mat(retained[0].full_image_mask, saved[0].full_image_mask, "canvas owns independent pixels");
        }
      }
    };
    auto a = std::async(std::launch::async, worker, 701U);
    auto b = std::async(std::launch::async, worker, 903U);
    a.get(); b.get();
}

// parallel2 and shadow without a worker, or an invalid threshold, throw before any output, and the
// worker stays usable.
void test_missing_worker_and_invalid_input(perception::PersistentTaskPair& lanes) {
    for (int count : {0, 1, 2}) {
        const auto f = make_fixture(100, 100, count);
        for (Mode mode : {Mode::Parallel2, Mode::Shadow}) {
            bool threw = false;
            try { (void)decode(f, mode, nullptr); }
            catch (const std::invalid_argument&) { threw = true; }
            check(threw, "parallel/shadow require worker, including empty result");
        }
    }
    auto f = make_fixture(100, 100, 2);
    for (float invalid : {-1.0F, 2.0F, std::numeric_limits<float>::infinity(),
                           std::numeric_limits<float>::quiet_NaN()}) {
        f.threshold = invalid;
        for (Mode mode : {Mode::Serial, Mode::Parallel2, Mode::Shadow}) {
          for (BinaryMode binary : {BinaryMode::Legacy, BinaryMode::Compare, BinaryMode::Shadow}) {
            bool threw = false;
            try { (void)decode(f, mode, &lanes, nullptr, binary); }
            catch (const std::invalid_argument&) { threw = true; }
            check(threw, "invalid threshold must fail before admitting output");
          }
        }
    }
    f.threshold = 0.5F;
    compare_fixture(f, lanes, 2); // Failure must not poison a persistent owner.
}

// Only legacy, compare and shadow parse (exact text) and an invalid mode throws; shadow records
// its check, and a reused profile clears the flag.
void test_binary_mode_contract(perception::PersistentTaskPair& lanes) {
    for (const auto& item : std::array<std::pair<const char*, BinaryMode>, 3>{
             std::pair{"legacy", BinaryMode::Legacy}, {"compare", BinaryMode::Compare},
             {"shadow", BinaryMode::Shadow}}) {
        check(yolo::parse_active12_mask_binary_implementation(item.first) == item.second,
              "binary configuration parse");
        check(std::string(yolo::active12_mask_binary_implementation_name(item.second)) == item.first,
              "binary configuration readback");
    }
    for (const char* invalid : {"", "Legacy", "compare ", "0", "parallel2"}) {
        bool threw = false;
        try { (void)yolo::parse_active12_mask_binary_implementation(invalid); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid binary configuration must fail closed");
    }
    const auto f = make_fixture(100, 100, 2);
    for (Mode mode : {Mode::Serial, Mode::Parallel2, Mode::Shadow}) {
        bool threw = false;
        try { (void)decode(f, mode, &lanes, nullptr, static_cast<BinaryMode>(99)); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid binary enum must fail closed");
        yolo::Active12MaskProfile profile;
        (void)decode(f, mode, &lanes, &profile, BinaryMode::Shadow);
        check(profile.binary_shadow_checked, "binary shadow must record successful check");
        const auto legacy = decode(f, mode, &lanes, &profile);
        profile_matches(profile, legacy, mode);
        check(!profile.binary_shadow_checked, "reused profile must clear binary shadow flag");
    }
}

// The direct compare equals threshold-then-convert on every float edge (signed zero, denormal,
// inf, NaN, threshold +-1 ulp) and ROI width, owns its output and leaves the input untouched.
void test_binary_primitive_edges() {
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    static_assert(std::numeric_limits<float>::is_iec559);
    for (const float probability : {0.0F, std::nextafter(0.0F, 1.0F), 0.25F,
             0.5F, 0.7F, std::nextafter(1.0F, 0.0F), 1.0F}) {
        const float threshold = probability == 0.5F ? 0.0F :
            probability > 0.0F && probability < 1.0F ?
            static_cast<float>(std::log(static_cast<double>(probability)) -
                               std::log1p(-static_cast<double>(probability))) : 0.0F;
        const std::array<float, 15> edges{0.0F, -0.0F,
            std::numeric_limits<float>::denorm_min(), -std::numeric_limits<float>::denorm_min(),
            std::numeric_limits<float>::min(), -std::numeric_limits<float>::min(),
            std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::quiet_NaN(), -std::numeric_limits<float>::quiet_NaN(),
            threshold, std::nextafter(threshold, -std::numeric_limits<float>::infinity()),
            std::nextafter(threshold, std::numeric_limits<float>::infinity())};
        for (const int width : {1, 15, 16, 17, 31, 32, 33}) {
          for (std::size_t rotation = 0; rotation < edges.size(); ++rotation) {
            cv::Mat storage(7, width + 19, CV_32FC1, cv::Scalar(42));
            cv::Mat roi = storage(cv::Rect(3, 2, width, 3));
            for (int row = 0; row < roi.rows; ++row) for (int col = 0; col < roi.cols; ++col) {
                roi.at<float>(row, col) = edges[(rotation + row * width + col) % edges.size()];
            }
            const cv::Mat before = storage.clone();
            cv::Mat binary, legacy, direct;
            if (probability <= 0.0F) {
                binary = cv::Mat(roi.size(), CV_8UC1, cv::Scalar(255));
                direct = cv::Mat(roi.size(), CV_8UC1, cv::Scalar(255));
            } else if (probability >= 1.0F) {
                binary = cv::Mat::zeros(roi.size(), CV_8UC1);
                direct = cv::Mat::zeros(roi.size(), CV_8UC1);
            } else {
                cv::threshold(roi, binary, threshold, 255.0, cv::THRESH_BINARY);
                cv::compare(roi, threshold, direct, cv::CMP_GT);
            }
            binary.convertTo(legacy, CV_8UC1);
            same_mat(legacy, direct, "binary primitive exceptional/ROI/tail parity");
            check(direct.u && direct.u != storage.u && !direct.isSubmatrix(), "direct mask ownership");
            for (int row = 0; row < storage.rows; ++row) {
                check(std::memcmp(storage.ptr(row), before.ptr(row), storage.cols * sizeof(float)) == 0,
                      "binary primitive changed input/guard bits");
            }
          }
        }
    }
    cv::Mat zero(1, 1, CV_32F, cv::Scalar(0));
    cv::Mat gt, ge;
    cv::compare(zero, 0.0F, gt, cv::CMP_GT);
    cv::compare(zero, 0.0F, ge, cv::CMP_GE);
    check(gt.at<std::uint8_t>(0, 0) == 0 && ge.at<std::uint8_t>(0, 0) == 255,
          "strict-threshold negative control has no discrimination");
}

// NaN or infinite prototype values give the same result in every mode.
void test_exceptional_prototype(perception::PersistentTaskPair& lanes) {
    auto fixture = make_fixture(641, 480, 2);
    for (const float value : {std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::infinity(),
                             -std::numeric_limits<float>::infinity()}) {
        std::fill(fixture.proto.begin(), fixture.proto.end(), value);
        for (const float probability : {0.0F, 0.5F, 1.0F}) {
            fixture.threshold = probability;
            fixture.name = "exceptional_prototype_" + std::to_string(float_bits(value)) +
                           "_threshold_" + std::to_string(float_bits(probability));
            compare_fixture(fixture, lanes, 2);
        }
    }
}

struct Stats { double median_ms; double p99_ms; };
Stats stats(std::vector<std::int64_t> values) {
    check(!values.empty(), "empty benchmark");
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    const double median = n % 2 ? static_cast<double>(values[n / 2]) :
        (static_cast<double>(values[n / 2 - 1]) + values[n / 2]) * 0.5;
    return {median / 1e6, values[(99 * n + 99) / 100 - 1] / 1e6};
}

void benchmark(int samples) {
    check(samples >= 100 && samples <= 1000, "benchmark samples must be in [100,1000]");
    perception::PersistentTaskPair lanes;
    using Clock = std::chrono::steady_clock;
    for (int count : {1, 2, 3, 6, 12}) {
        const auto f = make_fixture(1920, 1080, count, 1828U + count);
        const auto reference = parent_decode(f);
        check(reference.size() == static_cast<std::size_t>(count), "benchmark retained instance count");
        for (const auto& arm : std::array<std::pair<const char*, Mode>, 3>{
                 std::pair{"A1", Mode::Serial}, {"B", Mode::Parallel2}, {"A2", Mode::Serial}}) {
            for (int i = 0; i < 5; ++i) same(reference, decode(f, arm.second, &lanes), "benchmark warmup parity");
            std::array<std::vector<std::int64_t>, 7> metrics;
            for (int i = 0; i < samples; ++i) {
                yolo::Active12MaskProfile profile;
                const auto start = Clock::now();
                const auto result = decode(f, arm.second, &lanes, &profile);
                const auto end = Clock::now();
                metrics[0].push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
                metrics[1].push_back(profile.total_ns);
                metrics[2].push_back(profile.batch_wall_ns);
                std::array<std::int64_t, 4> work{};
                for (const auto& task : profile.tasks) {
                    work[0] += task.gemm_ns; work[1] += task.resize_ns;
                    work[2] += task.threshold_convert_ns; work[3] += task.canvas_ns;
                }
                for (std::size_t k = 0; k < work.size(); ++k) metrics[k + 3].push_back(work[k]);
                same(reference, result, "benchmark output parity outside measured interval");
                profile_matches(profile, result, arm.second);
            }
            std::cout << std::fixed << std::setprecision(6)
                      << "ACTIVE_MASK_BENCH arm=" << arm.first << " instances=" << count
                      << " width=1920 height=1080 samples=" << samples << " opencv_threads=" << cv::getNumThreads();
            constexpr std::array<const char*, 7> names{
                "call_wall", "profile_total", "batch_wall", "task_gemm_sum", "task_resize_sum",
                "task_threshold_convert_sum", "task_canvas_sum"};
            for (std::size_t k = 0; k < metrics.size(); ++k) {
                const auto value = stats(std::move(metrics[k]));
                std::cout << ' ' << names[k] << "_median_ms=" << value.median_ms
                          << ' ' << names[k] << "_p99_ms=" << value.p99_ms;
            }
            std::cout << " task_sums_overlap_not_additive_to_wall=1 component_only=1\n";
        }
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        // Gemini capture fixes this process-wide setting to 2 in production.
        // Set it once before any lane starts; it is not an optimization variable.
        cv::setNumThreads(2);
        check(cv::getNumThreads() == 2, "OpenCV thread setting mismatch");
        if (argc > 1) {
            check(std::string(argv[1]) == "--benchmark" && argc <= 3,
                  "usage: test_active_mask_parallel [--benchmark [samples]]");
            benchmark(argc == 3 ? std::stoi(argv[2]) : 100);
            return 0;
        }
        perception::PersistentTaskPair lanes;
        test_shapes_counts_and_thresholds(lanes);
        test_nms_order_and_first_valid_canvas(lanes);
        test_reuse_and_independent_owners();
        test_missing_worker_and_invalid_input(lanes);
        test_binary_mode_contract(lanes);
        test_binary_primitive_edges();
        test_exceptional_prototype(lanes);
        std::cout << "test_active_mask_parallel: PASS fixture_checks=" << fixture_checks.load()
                  << " frozen_parent=f8343042b8e9e79091765479471a5f2ef45c18e2\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_active_mask_parallel: FAIL " << error.what() << '\n';
        return 1;
    }
}
