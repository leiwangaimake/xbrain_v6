/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_active12_rect_decode.cpp
 * Brief: Unit tests for the Active12 decoder on the 896 wide, 608 high model input.
 *
 * Description:
 * What it checks. The sealed contract per input shape (896 x 896 unchanged, 896 x 608 added, any
 * other shape refused); the padding-anchor guard, anchor by anchor: on the square input against a
 * copy of the predicate as it was before the input shape existed, on the 608-high input against
 * an enumeration of the grid cells; the decode of the 608-high input against the square decoder
 * (the same candidates and prototypes placed 160 rows lower in square tensors must give the same
 * detections and masks, byte for byte) and against an independent mask reference; and mixed-up
 * shapes, a letterbox that is not the window's and content outside the window failing closed.
 *
 * Why the comparison with the square decoder. The 608-high input is rows 160..767 of the
 * 896 x 896 letterbox, and 160 is a multiple of every stride (20, 10 and 5 rows at strides 8, 16
 * and 32; 40 prototype rows). The square path is the one proven against the frozen parent decoder
 * (test_active_mask_parallel), which cannot decode a rectangle.
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/persistent_task_pair.hpp"
#include "common/postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {
using Mode = yolo::Active12MaskImplementation;
using BinaryMode = yolo::Active12MaskBinaryImplementation;
constexpr int kChannels = 48;
constexpr int kStrides[] = {8, 16, 32};
constexpr yolo::ModelInputShape kSquare{896, 896};
constexpr yolo::ModelInputShape kWide{896, 608};
// The 608-high input is the square letterbox from this row on.
constexpr int kRowShift = 160;
int checks = 0;

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

std::uint32_t float_bits(float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

struct Tensors {
    yolo::ModelInputShape shape;
    std::int64_t boxes = 0;
    int proto_height = 0;
    int proto_width = 0;
    std::vector<float> output;
    std::vector<float> proto;
    std::vector<std::int64_t> output_shape;
    std::vector<std::int64_t> proto_shape;
};

Tensors make_tensors(yolo::ModelInputShape shape) {
    const yolo::ModelSpec& spec = yolo::active12_seg_model_spec(shape);
    Tensors t;
    t.shape = shape;
    t.boxes = spec.expected_num_boxes;
    t.proto_height = spec.expected_proto_height;
    t.proto_width = spec.expected_proto_width;
    t.output.assign(static_cast<std::size_t>(kChannels * t.boxes), 0.0F);
    t.proto.assign(static_cast<std::size_t>(32 * t.proto_height * t.proto_width), 0.0F);
    t.output_shape = {1, kChannels, t.boxes};
    t.proto_shape = {1, 32, t.proto_height, t.proto_width};
    return t;
}

// Letterbox of a width x height image as preprocess() computes it for the square input, seen
// through the window of the given input (the whole square, or rows 160..767 of it). The caller
// picks images whose content lies inside the window.
yolo::LetterboxInfo letterbox(int width, int height, yolo::ModelInputShape shape) {
    const int side = shape.width;
    yolo::LetterboxInfo info;
    info.scale = std::min(static_cast<float>(side) / width, static_cast<float>(side) / height);
    info.new_width = static_cast<int>(std::round(width * info.scale));
    info.new_height = static_cast<int>(std::round(height * info.scale));
    info.pad_left = (side - info.new_width) / 2;
    info.pad_top = ((side - info.new_height) / 2) - yolo::active12_seg_model_spec(shape).input_window_top;
    info.original_width = width;
    info.original_height = height;
    return info;
}

// Anchor index of a grid cell: levels in stride order, each level row by row.
std::int64_t anchor_index(yolo::ModelInputShape shape, int level, int row, int column) {
    std::int64_t first = 0;
    for (int l = 0; l < level; ++l) {
        first += static_cast<std::int64_t>(shape.width / kStrides[l]) * (shape.height / kStrides[l]);
    }
    return first + static_cast<std::int64_t>(row) * (shape.width / kStrides[level]) + column;
}

std::vector<yolo::Detection> decode(const Tensors& t, const yolo::LetterboxInfo& info, Mode mode,
                                   perception::PersistentTaskPair* lanes, BinaryMode binary,
                                   yolo::Active12PaddingGuard* guard) {
    return yolo::postprocess_seg(t.output.data(), t.output_shape, t.proto.data(), t.proto_shape, info,
                                 yolo::active12_seg_model_spec(t.shape), 0.25F, 0.70F, 0.50F, t.shape, mode, lanes,
                                 nullptr, binary, false, guard);
}

void spec_contract() {
    const yolo::ModelSpec& square = yolo::active12_seg_model_spec();
    check(&square == &yolo::active12_seg_model_spec(kSquare), "896 x 896 is not the same sealed spec object");
    check(square.input_width == 896 && square.input_height == 896 && square.input_window_top == 0 &&
          square.expected_num_boxes == 16464 && square.expected_proto_height == 224 &&
          square.expected_proto_width == 224,
          "sealed square literals changed");
    const yolo::ModelSpec& wide = yolo::active12_seg_model_spec(kWide);
    check(wide.input_width == 896 && wide.input_height == 608 && wide.input_window_top == kRowShift &&
          wide.expected_num_boxes == 11172 && wide.expected_proto_height == 152 && wide.expected_proto_width == 224,
          "sealed 896 x 608 literals");
    check(wide.source == square.source && wide.num_classes == square.num_classes &&
          wide.mask_coefficient_count == square.mask_coefficient_count &&
          wide.mask_coefficient_offset == square.mask_coefficient_offset &&
          wide.expected_detection_channels == square.expected_detection_channels &&
          wide.max_detections == square.max_detections && wide.class_names == square.class_names &&
          wide.local_to_global_class_ids == square.local_to_global_class_ids,
          "the two shapes differ in more than the input, anchor and prototype sizes");
    for (const yolo::ModelInputShape shape :
         {yolo::ModelInputShape{896, 640}, yolo::ModelInputShape{896, 576}, yolo::ModelInputShape{608, 896},
          yolo::ModelInputShape{640, 640}, yolo::ModelInputShape{0, 0}, yolo::ModelInputShape{896, -608}}) {
        expect_invalid_argument([&] { (void)yolo::active12_seg_model_spec(shape); },
                                "unsealed shape " + std::to_string(shape.width) + "x" + std::to_string(shape.height));
    }
}

// How many candidates the guard drops when exactly the anchors in lit have a score above the
// threshold. Their boxes are invalid (all four values 0), so nothing is decoded and only the
// guard counts; the scores are cleared again.
std::size_t guard_drops(Tensors& t, const yolo::LetterboxInfo& info, const std::vector<std::int64_t>& lit) {
    for (const std::int64_t index : lit) t.output[static_cast<std::size_t>(4 * t.boxes + index)] = 0.9F;
    yolo::Active12PaddingGuard guard;
    guard.dropped = 123456U;
    const auto detections = decode(t, info, Mode::Serial, nullptr, BinaryMode::Legacy, &guard);
    for (const std::int64_t index : lit) t.output[static_cast<std::size_t>(4 * t.boxes + index)] = 0.0F;
    check(detections.empty(), "a candidate with an invalid box was decoded");
    return guard.dropped;
}

// Two decodes prove the guard anchor by anchor: with all anchors of the expected-padding set lit
// every one must be dropped, and with all others lit none may be.
void guard_equals(Tensors& t, const yolo::LetterboxInfo& info, const std::vector<char>& expected,
                  const std::string& label) {
    std::vector<std::int64_t> inside_padding, inside_image;
    for (std::int64_t index = 0; index < t.boxes; ++index) {
        (expected[static_cast<std::size_t>(index)] ? inside_padding : inside_image).push_back(index);
    }
    check(guard_drops(t, info, inside_padding) == inside_padding.size(), label + ": a padding anchor was kept");
    check(guard_drops(t, info, inside_image) == 0U, label + ": an image anchor was dropped");
}

// The guard's predicate as it was for the square input (commit 0144423), kept here unchanged.
bool previous_square_predicate(std::int64_t box_idx, int input_size, const yolo::LetterboxInfo& info) {
    for (const int stride : {8, 16, 32}) {
        const std::int64_t side = input_size / stride;
        if (box_idx < side * side) {
            const float cx = (static_cast<float>(box_idx % side) + 0.5F) * static_cast<float>(stride);
            const float cy = (static_cast<float>(box_idx / side) + 0.5F) * static_cast<float>(stride);
            return cx < static_cast<float>(info.pad_left) ||
                   cx >= static_cast<float>(info.pad_left + info.new_width) ||
                   cy < static_cast<float>(info.pad_top) ||
                   cy >= static_cast<float>(info.pad_top + info.new_height);
        }
        box_idx -= side * side;
    }
    return false;
}

void guard_square_unchanged() {
    Tensors t = make_tensors(kSquare);
    const int sizes[][2] = {{1280, 800}, {1920, 1080}, {641, 480}, {896, 896}, {800, 1280}, {500, 375}};
    for (const auto& size : sizes) {
        const yolo::LetterboxInfo info = letterbox(size[0], size[1], kSquare);
        std::vector<char> expected(static_cast<std::size_t>(t.boxes));
        for (std::int64_t index = 0; index < t.boxes; ++index) {
            expected[static_cast<std::size_t>(index)] = previous_square_predicate(index, 896, info);
        }
        guard_equals(t, info, expected, "square " + std::to_string(size[0]) + "x" + std::to_string(size[1]));
    }
}

// Padding flags of all anchors by walking the grid cells in head order, without index arithmetic.
std::vector<char> enumerate_padding(yolo::ModelInputShape shape, const yolo::LetterboxInfo& info) {
    std::vector<char> flags;
    for (const int stride : kStrides) {
        for (int row = 0; row < shape.height / stride; ++row) {
            for (int column = 0; column < shape.width / stride; ++column) {
                const float cx = (static_cast<float>(column) + 0.5F) * static_cast<float>(stride);
                const float cy = (static_cast<float>(row) + 0.5F) * static_cast<float>(stride);
                flags.push_back(cx < static_cast<float>(info.pad_left) ||
                                cx >= static_cast<float>(info.pad_left + info.new_width) ||
                                cy < static_cast<float>(info.pad_top) ||
                                cy >= static_cast<float>(info.pad_top + info.new_height));
            }
        }
    }
    return flags;
}

void guard_wide() {
    Tensors t = make_tensors(kWide);
    // The camera frame: 1280 x 800 resized to 896 x 560, 8 padding rows above and 40 below
    // (image rows 8..567). Padding anchors by centre row: stride 8 row 0 (centre y 4) and rows
    // 71..75 (572..604); stride 16 rows 35..37 (568..600; row 0 at y 8 is the first image row);
    // stride 32 row 18 (592; row 0 at y 16 and row 17 at y 560 are in the image):
    // 6 x 112 + 3 x 56 + 1 x 28 anchors.
    const yolo::LetterboxInfo camera = letterbox(1280, 800, kWide);
    check(camera.scale == 0.7F && camera.new_width == 896 && camera.new_height == 560 && camera.pad_left == 0 &&
          camera.pad_top == 8, "camera letterbox on the 896 x 608 input");
    const std::vector<char> flags = enumerate_padding(kWide, camera);
    check(static_cast<std::int64_t>(flags.size()) == t.boxes, "grid enumeration does not give 11172 anchors");
    check(std::count(flags.begin(), flags.end(), 1) == 868, "padding anchor count of the camera frame");
    const struct { int level, row; bool padding; } edges[] = {
        {0, 0, true}, {0, 1, false}, {0, 70, false}, {0, 71, true}, {1, 0, false}, {1, 34, false},
        {1, 35, true}, {2, 0, false}, {2, 17, false}, {2, 18, true}};
    for (const auto& edge : edges) {
        for (const int column : {0, (896 / kStrides[edge.level]) - 1}) {
            check(static_cast<bool>(flags[static_cast<std::size_t>(anchor_index(kWide, edge.level, edge.row, column))]) ==
                      edge.padding,
                  "edge row " + std::to_string(edge.row) + " of level " + std::to_string(edge.level));
        }
    }
    guard_equals(t, camera, flags, "896x608 camera");
    // Other sources whose content lies inside the window: other bar heights, an odd width, and
    // the tallest content the window holds (576 rows, no padding above).
    const int sizes[][2] = {{1920, 1080}, {2000, 1000}, {1281, 720}, {1600, 1000}, {1400, 900}};
    for (const auto& size : sizes) {
        const yolo::LetterboxInfo info = letterbox(size[0], size[1], kWide);
        check(info.pad_top >= 0 && info.pad_top + info.new_height <= kWide.height,
              "test source does not lie inside the window: " + std::to_string(size[0]) + "x" + std::to_string(size[1]));
        guard_equals(t, info, enumerate_padding(kWide, info),
                     "896x608 " + std::to_string(size[0]) + "x" + std::to_string(size[1]));
    }
}

struct Candidate {
    int level, row, column;       // grid cell on the 896 x 608 input
    float cx, cy, width, height;  // box in 896 x 608 input pixels
    int local_class;
    float confidence;
    float coefficients[32];
};

// Candidates on the 896 x 608 input for a 1280 x 800 frame. All box values are multiples of 1/4,
// so moving them down by 160 rows is exact in float32 and NMS sees the same overlaps.
std::vector<Candidate> make_candidates(int count, unsigned seed) {
    std::mt19937 random(seed);
    const auto pick = [&](int low, int high) { return low + static_cast<int>(random() % static_cast<unsigned>(high - low + 1)); };
    std::vector<Candidate> candidates;
    for (int i = 0; i < count; ++i) {
        Candidate c{};
        c.level = pick(0, 2);
        const int stride = kStrides[c.level];
        const int rows = kWide.height / stride, columns = kWide.width / stride;
        // Every seventh candidate sits on the last grid row, in the bar below the image at every
        // stride; the others in the image (rows 8..567).
        if (i % 7 == 3) {
            c.row = rows - 1;
        } else {
            c.row = pick((8 + stride - 1) / stride + 1, (568 / stride) - 2);
        }
        c.column = pick(0, columns - 1);
        c.cx = (static_cast<float>(c.column) + 0.5F) * static_cast<float>(stride) + static_cast<float>(pick(-8, 8)) * 0.25F;
        c.cy = (static_cast<float>(c.row) + 0.5F) * static_cast<float>(stride) + static_cast<float>(pick(-8, 8)) * 0.25F;
        c.width = static_cast<float>(pick(96, 1200)) * 0.25F;
        // A padding candidate is tall enough to reach well into the image, so that it is decoded
        // when the guard is off.
        c.height = static_cast<float>(i % 7 == 3 ? pick(800, 900) : pick(96, 900)) * 0.25F;
        // The first image candidate is a traversable_area (local class 10): the only class whose
        // first instance also carries an original-image mask.
        c.local_class = (i == 0) ? 10 : pick(0, 11);
        c.confidence = 0.97F - static_cast<float>(i) * 0.008F;
        for (float& value : c.coefficients) value = static_cast<float>(pick(-1000, 1000)) / 2000.0F;
        candidates.push_back(c);
    }
    return candidates;
}

void put_candidate(Tensors& t, std::int64_t index, const Candidate& c, float cy) {
    const auto at = [&](int channel) -> float& { return t.output[static_cast<std::size_t>(channel * t.boxes + index)]; };
    at(0) = c.cx;
    at(1) = cy;
    at(2) = c.width;
    at(3) = c.height;
    at(4 + c.local_class) = c.confidence;
    for (int k = 0; k < 32; ++k) at(16 + k) = c.coefficients[k];
}

struct Pair {
    Tensors wide, square;
    yolo::LetterboxInfo wide_info, square_info;
};

// The same candidates and prototypes as 896 x 608 tensors and placed kRowShift rows lower in
// 896 x 896 tensors; the square prototype rows outside the copied band hold other random values.
Pair make_pair(const std::vector<Candidate>& candidates, unsigned seed) {
    Pair p{make_tensors(kWide), make_tensors(kSquare), letterbox(1280, 800, kWide), letterbox(1280, 800, kSquare)};
    check(p.square_info.pad_top - p.wide_info.pad_top == kRowShift && p.square_info.scale == p.wide_info.scale &&
          p.square_info.new_width == p.wide_info.new_width && p.square_info.new_height == p.wide_info.new_height &&
          p.square_info.pad_left == p.wide_info.pad_left,
          "the two letterboxes differ by more than the row shift");
    std::mt19937 random(seed);
    const auto value = [&] { return static_cast<float>(static_cast<int>(random() % 2001U) - 1000) / 1000.0F; };
    for (float& v : p.wide.proto) v = value();
    for (float& v : p.square.proto) v = value();
    const int plane_wide = p.wide.proto_height * p.wide.proto_width;
    const int plane_square = p.square.proto_height * p.square.proto_width;
    for (int channel = 0; channel < 32; ++channel) {
        std::memcpy(&p.square.proto[static_cast<std::size_t>(channel * plane_square + (kRowShift / 4) * p.square.proto_width)],
                    &p.wide.proto[static_cast<std::size_t>(channel * plane_wide)],
                    static_cast<std::size_t>(plane_wide) * sizeof(float));
    }
    for (const Candidate& c : candidates) {
        put_candidate(p.wide, anchor_index(kWide, c.level, c.row, c.column), c, c.cy);
        put_candidate(p.square, anchor_index(kSquare, c.level, c.row + kRowShift / kStrides[c.level], c.column), c,
                      c.cy + static_cast<float>(kRowShift));
    }
    return p;
}

void same_mat(const cv::Mat& lhs, const cv::Mat& rhs, const std::string& label) {
    check(lhs.empty() == rhs.empty(), label + ": empty");
    check(lhs.type() == rhs.type() && lhs.size() == rhs.size(), label + ": shape/type");
    for (int row = 0; row < lhs.rows; ++row) {
        check(std::memcmp(lhs.ptr(row), rhs.ptr(row), static_cast<std::size_t>(lhs.cols)) == 0,
              label + ": pixels at row " + std::to_string(row));
    }
}

void same(const std::vector<yolo::Detection>& lhs, const std::vector<yolo::Detection>& rhs, const std::string& label) {
    check(lhs.size() == rhs.size(), label + ": count " + std::to_string(lhs.size()) + " vs " + std::to_string(rhs.size()));
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        const auto& a = lhs[i];
        const auto& b = rhs[i];
        const std::string item = label + "[" + std::to_string(i) + "]";
        check(a.bbox == b.bbox, item + ": bbox");
        check(float_bits(a.confidence) == float_bits(b.confidence), item + ": confidence bits");
        check(a.class_id == b.class_id && a.original_class_id == b.original_class_id && a.class_name == b.class_name &&
              a.source == b.source && a.low_confidence_candidate == b.low_confidence_candidate, item + ": metadata");
        same_mat(a.mask, b.mask, item + ": local mask");
        same_mat(a.full_image_mask, b.full_image_mask, item + ": full mask");
    }
}

// 12 candidates take Ultralytics' crop for fewer than 50 detections, 70 the other one.
void wide_equals_square_decoder(perception::PersistentTaskPair& lanes) {
    for (const int count : {12, 70}) {
        const std::vector<Candidate> candidates = make_candidates(count, 20261010U + static_cast<unsigned>(count));
        const Pair p = make_pair(candidates, 977U + static_cast<unsigned>(count));
        const std::string name = "n" + std::to_string(count);
        std::size_t reference_count = 0;
        for (const bool guarded : {false, true}) {
            for (const Mode mode : {Mode::Serial, Mode::Parallel2, Mode::Shadow}) {
                for (const BinaryMode binary : {BinaryMode::Legacy, BinaryMode::Compare}) {
                    yolo::Active12PaddingGuard wide_guard, square_guard;
                    perception::PersistentTaskPair* use = mode == Mode::Serial ? nullptr : &lanes;
                    const auto wide = decode(p.wide, p.wide_info, mode, use, binary, guarded ? &wide_guard : nullptr);
                    const auto square = decode(p.square, p.square_info, mode, use, binary, guarded ? &square_guard : nullptr);
                    same(wide, square, name + (guarded ? " guarded" : " unguarded"));
                    if (guarded) {
                        // The padding rows of the two inputs are the same grid cells of the frame.
                        check(wide_guard.dropped == square_guard.dropped && wide_guard.dropped > 0U,
                              name + ": guard counts " + std::to_string(wide_guard.dropped) + " vs " +
                                  std::to_string(square_guard.dropped));
                        check(wide.size() < reference_count, name + ": the guard dropped no decoded candidate");
                    } else {
                        reference_count = wide.size();
                    }
                }
            }
        }
        // The fixture is not trivial: several detections on either side of the 50-detection crop
        // switch, non-empty masks, and one original-image mask (the first traversable_area).
        const auto wide = decode(p.wide, p.wide_info, Mode::Serial, nullptr, BinaryMode::Legacy, nullptr);
        std::size_t filled = 0, canvases = 0;
        for (const auto& detection : wide) {
            filled += cv::countNonZero(detection.mask) > 0;
            canvases += !detection.full_image_mask.empty();
        }
        check(count == 12 ? (wide.size() >= 6U && wide.size() < 50U) : wide.size() >= 50U,
              name + ": detection count " + std::to_string(wide.size()));
        check(filled * 2U >= wide.size() && canvases == 1U, name + ": masks " + std::to_string(filled) + ", canvases " +
                                                                std::to_string(canvases));
        std::cout << "896x608 equals the square decoder: " << name << " detections=" << wide.size() << " masks_filled="
                  << filled << "\n";
    }
}

// An independent mask of one detection on the 896 x 608 input: the full 152 x 224 logit plane,
// Ultralytics' prototype crop on the square 224 x 224 plane moved up by the window's 40 rows, one
// cv::resize to the image, the zero-logit threshold (mask_threshold 0.5). Pixels whose logit is
// within 1e-4 of zero are left out: the sum over the 32 channels may be taken in another order.
void wide_masks_against_reference() {
    const std::vector<Candidate> candidates = make_candidates(12, 4242U);
    const Pair p = make_pair(candidates, 99U);
    const auto detections = decode(p.wide, p.wide_info, Mode::Serial, nullptr, BinaryMode::Legacy, nullptr);
    check(detections.size() >= 6U, "reference fixture has too few detections");
    // Ultralytics scale_masks on the square plane: gain = min(proto / original), symmetric padding
    // rounded -0.1 / +0.1; then the window (prototype rows 40..191 of the square plane).
    const float gain = std::min(224.0F / 800.0F, 224.0F / 1280.0F);
    const float pad_w = (224.0F - 1280.0F * gain) / 2.0F, pad_h = (224.0F - 800.0F * gain) / 2.0F;
    const int top = static_cast<int>(std::round(pad_h - 0.1F)) - kRowShift / 4, left = static_cast<int>(std::round(pad_w - 0.1F));
    const int bottom = 224 - static_cast<int>(std::round(pad_h + 0.1F)) - kRowShift / 4, right = 224 - static_cast<int>(std::round(pad_w + 0.1F));
    check(top == 2 && bottom == 142 && left == 0 && right == 224, "prototype crop of the camera frame on 152 x 224");
    std::size_t compared = 0, skipped = 0;
    for (const auto& detection : detections) {
        const Candidate* source = nullptr;
        for (const Candidate& c : candidates) {
            if (float_bits(c.confidence) == float_bits(detection.confidence)) source = &c;
        }
        check(source != nullptr, "a detection has no source candidate");
        cv::Mat logits(152, 224, CV_32F, cv::Scalar(0.0F));
        for (int k = 0; k < 32; ++k) {
            const cv::Mat plane(152, 224, CV_32F, const_cast<float*>(&p.wide.proto[static_cast<std::size_t>(k * 152 * 224)]));
            logits += plane * source->coefficients[k];
        }
        cv::Mat image_logits;
        cv::resize(logits(cv::Rect(left, top, right - left, bottom - top)), image_logits, cv::Size(1280, 800), 0.0, 0.0,
                   cv::INTER_LINEAR);
        check(detection.mask.size() == detection.bbox.size(), "bbox-local mask size");
        for (int y = 0; y < detection.bbox.height; ++y) {
            for (int x = 0; x < detection.bbox.width; ++x) {
                const float logit = image_logits.at<float>(detection.bbox.y + y, detection.bbox.x + x);
                if (std::fabs(logit) < 1.0e-4F) {
                    ++skipped;
                    continue;
                }
                ++compared;
                if ((detection.mask.at<std::uint8_t>(y, x) != 0) != (logit > 0.0F)) {
                    throw std::runtime_error("mask differs from the reference at " + std::to_string(x) + "," + std::to_string(y));
                }
            }
        }
    }
    check(compared > 100000U && skipped * 100U < compared, "reference compared " + std::to_string(compared) +
                                                                " pixels, skipped " + std::to_string(skipped));
    std::cout << "896x608 masks against the reference: detections=" << detections.size() << " pixels=" << compared
              << " near_zero_skipped=" << skipped << "\n";
}

void mixed_shapes_fail_closed() {
    const std::vector<Candidate> candidates = make_candidates(12, 7U);
    const Pair p = make_pair(candidates, 8U);
    const yolo::ModelSpec& square_spec = yolo::active12_seg_model_spec();
    const yolo::ModelSpec& wide_spec = yolo::active12_seg_model_spec(kWide);
    const auto run = [&](const Tensors& t, const yolo::LetterboxInfo& info, const yolo::ModelSpec& spec,
                         yolo::ModelInputShape shape) {
        (void)yolo::postprocess_seg(t.output.data(), t.output_shape, t.proto.data(), t.proto_shape, info, spec, 0.25F,
                                    0.70F, 0.50F, shape);
    };
    expect_invalid_argument([&] { run(p.wide, p.wide_info, square_spec, kSquare); }, "896 x 608 tensors, square spec");
    expect_invalid_argument([&] { run(p.square, p.square_info, wide_spec, kWide); }, "square tensors, 896 x 608 spec");
    expect_invalid_argument([&] { run(p.wide, p.wide_info, wide_spec, kSquare); }, "896 x 608 spec, square input shape");
    expect_invalid_argument([&] { run(p.square, p.square_info, square_spec, kWide); }, "square spec, 896 x 608 input shape");
    // The square call form names a square input.
    expect_invalid_argument(
        [&] {
            (void)yolo::postprocess_seg(p.wide.output.data(), p.wide.output_shape, p.wide.proto.data(),
                                        p.wide.proto_shape, p.wide_info, wide_spec, 0.25F, 0.70F, 0.50F, 896);
        },
        "896 x 608 spec through the square call form");
    // A spec that only claims a shape: wrong anchor count, another window, or a shape without a
    // sealed contract.
    yolo::ModelSpec wrong_count = wide_spec;
    wrong_count.expected_num_boxes = 16464;
    expect_invalid_argument([&] { run(p.square, p.square_info, wrong_count, kWide); }, "896 x 608 spec with the square anchor count");
    yolo::ModelSpec other_window = wide_spec;
    other_window.input_window_top = 144;
    expect_invalid_argument([&] { run(p.wide, p.wide_info, other_window, kWide); }, "896 x 608 spec with a centred window");
    yolo::ModelSpec unsealed = wide_spec;
    unsealed.input_height = 640;
    expect_invalid_argument([&] { run(p.wide, p.wide_info, unsealed, yolo::ModelInputShape{896, 640}); }, "unsealed 896 x 640 spec");
    // The letterbox must be the window's: a letterbox centred in 896 x 608 (24 rows above) is not.
    yolo::LetterboxInfo centred = p.wide_info;
    centred.pad_top = (kWide.height - centred.new_height) / 2;
    expect_invalid_argument([&] { run(p.wide, centred, wide_spec, kWide); }, "letterbox centred in the 896 x 608 input");
    yolo::LetterboxInfo shifted = p.wide_info;
    shifted.pad_left = 16;
    expect_invalid_argument([&] { run(p.wide, shifted, wide_spec, kWide); }, "letterbox with a horizontal offset");
    // A letterbox that claims the window while the original size is that of a 4:3 image: the
    // prototype crop of such an image does not lie inside the window's prototype rows.
    yolo::LetterboxInfo inconsistent = p.wide_info;
    inconsistent.original_width = 641;
    inconsistent.original_height = 480;
    expect_invalid_argument([&] { run(p.wide, inconsistent, wide_spec, kWide); }, "window letterbox with a 4:3 original size");
    // An image whose content does not lie inside the window (4:3: 672 rows on the square input).
    expect_invalid_argument([&] { run(p.wide, letterbox(641, 480, kWide), wide_spec, kWide); }, "4:3 image on the 896 x 608 input");
    // Sanity: the unmodified calls decode.
    run(p.wide, p.wide_info, wide_spec, kWide);
    run(p.square, p.square_info, square_spec, kSquare);
}
}  // namespace

int main() {
    try {
        // Gemini capture fixes this process-wide setting to 2 in production (as test_active_mask_parallel).
        cv::setNumThreads(2);
        check(cv::getNumThreads() == 2, "OpenCV thread setting mismatch");
        perception::PersistentTaskPair lanes;
        spec_contract();
        guard_square_unchanged();
        guard_wide();
        wide_equals_square_decoder(lanes);
        wide_masks_against_reference();
        mixed_shapes_fail_closed();
        std::cout << "test_active12_rect_decode: PASS checks=" << checks << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_active12_rect_decode: FAIL " << error.what() << '\n';
        return 1;
    }
}
