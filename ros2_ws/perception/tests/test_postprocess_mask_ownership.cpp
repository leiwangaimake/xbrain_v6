/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_postprocess_mask_ownership.cpp
 * Brief: Unit tests for detection mask ownership after postprocess (no borrowed decoder memory escapes).
 *
 * Description:
 * What it checks. Masks leaving postprocess must own their pixels, because decoder staging buffers
 * are reused for the next frame. Cases cover parsing and empty outputs, the release policy, a
 * negative control with deliberately broken decoder outputs backed by externally owned staging
 * data, and concurrent owners. A benchmark on fixed synthetic tensor outputs (generated outside
 * the timings) is included but is not a gate.
 */

// Ownership-only regression: real CPU postprocess outputs, no model/camera/GPU.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "common/logger.hpp"
#include "common/persistent_task_pair.hpp"
#include "common/postprocess.hpp"
#include "common/postprocess_mask_ownership.hpp"

namespace {
using Mode = perception::MaskOutputCopyMode;
using Detections = std::vector<yolo::Detection>;
std::atomic<std::uint64_t> checks{0};
volatile std::uint64_t observable_checksum = 0;

void require(bool condition, const std::string& message) {
    checks.fetch_add(1, std::memory_order_relaxed);
    if (!condition) throw std::runtime_error(message);
}

bool same_mat(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty()) return a.empty() == b.empty();
    if (a.type() != b.type() || a.size() != b.size()) return false;
    const auto bytes = static_cast<std::size_t>(a.cols) * a.elemSize();
    for (int row = 0; row < a.rows; ++row) {
        if (std::memcmp(a.ptr(row), b.ptr(row), bytes) != 0) return false;
    }
    return true;
}

bool same_detection(const yolo::Detection& a, const yolo::Detection& b) {
    return a.bbox == b.bbox &&
        std::memcmp(&a.confidence, &b.confidence, sizeof(float)) == 0 &&
        a.class_id == b.class_id && a.class_name == b.class_name &&
        a.low_confidence_candidate == b.low_confidence_candidate &&
        a.source == b.source && a.original_class_id == b.original_class_id &&
        same_mat(a.mask, b.mask) && same_mat(a.full_image_mask, b.full_image_mask);
}

bool same_detections(const Detections& a, const Detections& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!same_detection(a[i], b[i])) return false;
    }
    return true;
}

Detections independent_snapshot(const Detections& input, bool full = true) {
    Detections copy = input;
    for (auto& d : copy) {
        d.mask = d.mask.clone();
        d.full_image_mask = full ? d.full_image_mask.clone() : cv::Mat{};
    }
    return copy;
}

std::uint64_t mask_bytes(const Detections& input, bool full) {
    std::uint64_t bytes = 0;
    for (const auto& d : input) {
        bytes += d.mask.total() * d.mask.elemSize();
        if (full) bytes += d.full_image_mask.total() * d.full_image_mask.elemSize();
    }
    return bytes;
}

struct Fixture {
    bool active;
    int size;
    std::int64_t anchors;
    int channels;
    int prototype_size;
    yolo::LetterboxInfo info;
    std::vector<float> output;
    std::vector<float> proto;

    Fixture(bool is_active, int width, int height, int variation = 0)
        : active(is_active), size(active ? 896 : 640), anchors(active ? 16464 : 8400),
          channels(active ? 48 : 116), prototype_size(active ? 224 : 160),
          output(static_cast<std::size_t>(channels * anchors), 0.0F),
          proto(static_cast<std::size_t>(32 * prototype_size * prototype_size), 0.0F) {
        info.original_width = width;
        info.original_height = height;
        info.scale = std::min(static_cast<float>(size) / width,
                              static_cast<float>(size) / height);
        info.new_width = static_cast<int>(std::round(width * info.scale));
        info.new_height = static_cast<int>(std::round(height * info.scale));
        info.pad_left = (size - info.new_width) / 2;
        info.pad_top = (size - info.new_height) / 2;
        for (int y = 0; y < prototype_size; ++y) {
            for (int x = 0; x < prototype_size; ++x) {
                proto[static_cast<std::size_t>(y * prototype_size + x)] =
                    ((x / 11 + y / 13 + variation) % 3 == 0) ? -2.0F : 2.0F;
            }
        }
        // Large, medium, and small masks; different classes avoid NMS coupling.
        add(0, 0.03F, 0.03F, 0.58F, 0.91F, active ? 10 : 0, 0.95F);
        add(1, 0.66F, 0.06F, 0.31F, 0.55F, active ? 4 : 56, 0.90F);
        add(2, 0.69F, 0.70F, 0.15F, 0.22F, active ? 8 : 28, 0.85F);
        if (active) add(3, 0.87F, 0.72F, 0.10F, 0.20F, 10, 0.80F);
    }

    void add(std::int64_t index, float x, float y, float width, float height,
             int cls, float confidence) {
        const float w = info.original_width * width * info.scale;
        const float h = info.original_height * height * info.scale;
        output[static_cast<std::size_t>(index)] =
            info.pad_left + info.original_width * x * info.scale + w / 2.0F;
        output[static_cast<std::size_t>(anchors + index)] =
            info.pad_top + info.original_height * y * info.scale + h / 2.0F;
        output[static_cast<std::size_t>(2 * anchors + index)] = w;
        output[static_cast<std::size_t>(3 * anchors + index)] = h;
        output[static_cast<std::size_t>((4 + cls) * anchors + index)] = confidence;
        output[static_cast<std::size_t>((active ? 16 : 84) * anchors + index)] = 1.0F;
    }

    Detections decode(yolo::Active12MaskImplementation mode,
                      perception::PersistentTaskPair* lanes = nullptr) const {
        const std::vector<std::int64_t> shape{1, channels, anchors};
        const std::vector<std::int64_t> proto_shape{1, 32, prototype_size, prototype_size};
        if (!active) return yolo::postprocess_seg(output.data(), shape, proto.data(),
            proto_shape, info, 0.25F, {}, 0.0F, 8, 0.7F, 0.5F, size);
        return yolo::postprocess_seg(output.data(), shape, proto.data(), proto_shape,
            info, yolo::active12_seg_model_spec(), 0.25F, 0.7F, 0.5F, size,
            mode, lanes, nullptr, yolo::Active12MaskBinaryImplementation::Legacy);
    }
};

// Only clone and retain parse (unset means clone); empty lists and mask-less detections copy
// nothing; an invalid mode throws before changing anything.
void test_parsing_and_empty() {
    require(perception::parse_mask_output_copy_mode(nullptr) == Mode::Clone, "default clone");
    require(perception::parse_mask_output_copy_mode("clone") == Mode::Clone, "parse clone");
    require(perception::parse_mask_output_copy_mode("retain") == Mode::Retain, "parse retain");
    for (const auto* text : {"", "0", "1", "Clone", "retain ", "unknown"}) {
        bool caught = false;
        try { (void)perception::parse_mask_output_copy_mode(text); }
        catch (const std::invalid_argument&) { caught = true; }
        require(caught, "reject malformed mode");
    }
    Detections empty;
    for (auto mode : {Mode::Clone, Mode::Retain}) {
        require(perception::finalize_postprocess_mask_ownership(empty, mode, true) == 0,
                "empty list copies no bytes");
        Detections no_masks(1);
        no_masks[0].class_name = "metadata_survives";
        const auto golden = independent_snapshot(no_masks);
        require(perception::finalize_postprocess_mask_ownership(no_masks, mode, false) == 0,
                "empty masks copy no bytes");
        require(same_detections(no_masks, golden), "empty mask metadata preserved");
    }
    Fixture f(false, 1280, 800);
    auto list = f.decode(yolo::Active12MaskImplementation::Serial);
    const auto golden = independent_snapshot(list);
    bool caught = false;
    try { (void)perception::finalize_postprocess_mask_ownership(list, static_cast<Mode>(99), false); }
    catch (const std::invalid_argument&) { caught = true; }
    require(caught && same_detections(list, golden), "invalid enum rejects before mutation");
}

struct RetainedCase { Detections retained; Detections golden; };

// On real decoder outputs clone and retain give identical detections; retain copies nothing, and
// the retained masks survive freeing the tensors and later frames on the same owner.
RetainedCase test_real_outputs(bool active, int width, int height,
                             yolo::Active12MaskImplementation implementation) {
    Fixture f(active, width, height);
    perception::PersistentTaskPair lanes;
    auto raw = f.decode(implementation, &lanes);
    require(raw.size() == (active ? 4U : 3U), "fixture has intended detection count");
    const auto golden = independent_snapshot(raw, active);
    auto cloned = raw;
    auto retained = raw;
    const auto expected = mask_bytes(raw, active);
    require(expected > 0, "fixture masks are nonempty");
    require(perception::finalize_postprocess_mask_ownership(cloned, Mode::Clone, active) == expected,
            "clone byte accounting");
    require(perception::finalize_postprocess_mask_ownership(retained, Mode::Retain, active) == 0,
            "retain performs no clone");
    require(same_detections(cloned, golden) && same_detections(retained, golden),
            "clone and retain are metadata/pixel bit-identical");
    std::size_t traversable = 0, canvases = 0;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        require(raw[i].mask.type() == CV_8UC1 && raw[i].mask.u != nullptr,
                "real local output has OpenCV-owned storage");
        require(retained[i].mask.data == raw[i].mask.data, "retain leaves local buffer unchanged");
        require(cloned[i].mask.data != raw[i].mask.data, "clone produces independent local buffer");
        require(raw[i].mask.size() == raw[i].bbox.size(), "bbox-local dimensions");
        if (raw[i].class_id == 92) ++traversable;
        if (!raw[i].full_image_mask.empty()) {
            ++canvases;
            require(active && raw[i].class_id == 92, "only Active traversable has full canvas");
            require(raw[i].full_image_mask.size() == cv::Size(width, height), "full native dimensions");
            require(raw[i].full_image_mask.u != nullptr, "full canvas has owned storage");
            require(retained[i].full_image_mask.data == raw[i].full_image_mask.data,
                    "retain leaves full canvas unchanged");
            require(cloned[i].full_image_mask.data != raw[i].full_image_mask.data,
                    "clone makes independent full canvas");
        }
    }
    require(active ? traversable == 2 && canvases == 1 : canvases == 0,
            "first class92 owns exactly one full canvas; COCO stays bbox-only");
    // Raw TensorRT-style staging memory is overwritten and released; outputs must survive.
    std::fill(f.output.begin(), f.output.end(), -32768.0F);
    std::fill(f.proto.begin(), f.proto.end(), 32768.0F);
    f.output.clear(); f.output.shrink_to_fit();
    f.proto.clear(); f.proto.shrink_to_fit();
    raw.clear(); raw.shrink_to_fit();
    require(same_detections(retained, golden) && same_detections(cloned, golden),
            "input/prototype/temporary owner destruction cannot mutate masks");
    // New calls on the same lane owner use different dimensions and contents.
    for (int variation = 1; variation <= 3; ++variation) {
        Fixture next(active, variation % 2 ? 641 : 1920, variation % 2 ? 479 : 1080, variation);
        auto later = next.decode(implementation, &lanes);
        require(!later.empty(), "later frame decoded");
        require(same_detections(retained, golden), "later frame and size do not overwrite old outputs");
    }
    return {std::move(retained), golden}; // lane and all decode temporaries are destroyed here.
}

// With keep_full off the full canvas is dropped (never cloned) and everything else is kept.
void test_release_policy() {
    Fixture f(true, 1280, 800);
    const auto raw = f.decode(yolo::Active12MaskImplementation::Serial);
    for (auto mode : {Mode::Clone, Mode::Retain}) {
        auto list = raw;
        const auto expected = independent_snapshot(raw, false);
        require(perception::finalize_postprocess_mask_ownership(list, mode, false) ==
                    (mode == Mode::Clone ? mask_bytes(raw, false) : 0), "discarded full mask is not cloned");
        require(same_detections(list, expected), "keep_full=false always clears canvas only");
    }
}

// Negative control: a mask borrowing external memory is caught once that memory is overwritten;
// retain does not fix borrowed memory, clone does.
void test_borrowed_memory_negative_control() {
    Fixture f(true, 1280, 800);
    const auto raw = f.decode(yolo::Active12MaskImplementation::Serial);
    // Deliberately broken decoder outputs backed by externally owned staging data.
    // The comparator must detect overwrite; retain is not a cure for borrowed memory.
    for (bool full : {false, true}) {
        auto broken = raw;
        cv::Mat& selected = full ? broken[0].full_image_mask : broken[0].mask;
        require(!selected.empty(), "negative control has selected bytes");
        const auto bytes = selected.total() * selected.elemSize();
        std::vector<unsigned char> staging(bytes);
        for (int row = 0; row < selected.rows; ++row) {
            std::memcpy(staging.data() + static_cast<std::size_t>(row * selected.cols),
                        selected.ptr(row), static_cast<std::size_t>(selected.cols));
        }
        selected = cv::Mat(selected.rows, selected.cols, CV_8UC1, staging.data());
        const auto golden = independent_snapshot(broken);
        auto safe_clone = broken;
        (void)perception::finalize_postprocess_mask_ownership(safe_clone, Mode::Clone, true);
        (void)perception::finalize_postprocess_mask_ownership(broken, Mode::Retain, true);
        require(same_detections(broken, golden), "broken borrowed output initially matches");
        for (auto& byte : staging) byte ^= 0xffU;
        require(!same_detections(broken, golden), "negative control catches overwritten borrowed memory");
        require(same_detections(safe_clone, golden), "negative clone control remains independent");
    }
}

// Retained outputs stay intact while another thread reads them and new frames decode on separate
// owners.
void test_concurrent_owners(const std::vector<RetainedCase>& saved) {
    std::exception_ptr error;
    std::thread consumer([&]() {
        try {
            for (int repetition = 0; repetition < 12; ++repetition) {
                for (const auto& item : saved) {
                    const auto handoff = item.retained; // independent cv::Mat references
                    require(same_detections(handoff, item.golden), "concurrent downstream retains prior owners");
                }
            }
        } catch (...) { error = std::current_exception(); }
    });
    std::exception_ptr owner_error;
    try {
        for (int repetition = 0; repetition < 6; ++repetition) {
            Fixture f(true, repetition % 2 ? 1280 : 640, repetition % 2 ? 800 : 480, repetition + 5);
            perception::PersistentTaskPair separate_owner;
            auto output = f.decode(yolo::Active12MaskImplementation::Parallel2, &separate_owner);
            require(output.size() == 4, "separate parallel owner produces complete outputs");
        }
    } catch (...) { owner_error = std::current_exception(); }
    consumer.join();
    if (error) std::rethrow_exception(error);
    if (owner_error) std::rethrow_exception(owner_error);
    for (const auto& item : saved) require(same_detections(item.retained, item.golden), "final old outputs preserved");
}

std::uint64_t sparse_checksum(const Detections& detections) {
    std::uint64_t result = detections.size();
    for (const auto& d : detections) {
        result += static_cast<std::uint64_t>(d.bbox.area() + d.class_id + 1);
        for (const cv::Mat* mat : {&d.mask, &d.full_image_mask}) {
            if (!mat->empty()) {
                result += mat->ptr<unsigned char>(0)[0];
                result += mat->ptr<unsigned char>(mat->rows / 2)[mat->cols / 2];
                result += mat->ptr<unsigned char>(mat->rows - 1)[mat->cols - 1];
            }
        }
    }
    return result;
}

void benchmark() {
    // Fixed real synthetic-tensor postprocess outputs, generated OUTSIDE timings.
    Fixture coco(false, 1280, 800), active(true, 1280, 800);
    perception::PersistentTaskPair lanes;
    auto fixed = coco.decode(yolo::Active12MaskImplementation::Serial);
    auto active_outputs = active.decode(yolo::Active12MaskImplementation::Parallel2, &lanes);
    fixed.insert(fixed.end(), active_outputs.begin(), active_outputs.end());
    active_outputs.clear();
    const auto golden = independent_snapshot(fixed);
    const auto bytes_per_clone = mask_bytes(fixed, true);
    const auto expected_checksum = sparse_checksum(fixed);
    constexpr int warmup = 50;
    constexpr int count = 800;
    std::cout << "{\"kind\":\"benchmark_manifest\",\"scope\":\"ownership_copy_release_component_only\","
        "\"not_inference_or_pipeline_hz\":true,\"opencv_threads\":" << cv::getNumThreads()
        << ",\"native_width\":1280,\"native_height\":800,\"coco_input\":640,\"active_input\":896,"
        "\"instances\":" << fixed.size() << ",\"clone_destination_bytes_per_iteration\":" << bytes_per_clone
        << ",\"warmup_per_arm\":" << warmup << ",\"samples_per_arm\":" << count
        << ",\"includes_vector_shallow_copy_and_destruction\":true,\"mask_generation_timed\":false,"
        "\"source_masks_held_constant\":true,\"affinity_owned_by_runner\":true}" << std::endl;
    int arm = 0;
    for (auto mode : {Mode::Clone, Mode::Retain, Mode::Clone}) {
        std::vector<std::int64_t> samples;
        samples.reserve(count);
        std::uint64_t observed_bytes = 0;
        for (int i = -warmup; i < count; ++i) {
            std::uint64_t checksum = 0, copied = 0;
            const auto start = std::chrono::steady_clock::now();
            {
                auto work = fixed;
                copied = perception::finalize_postprocess_mask_ownership(work, mode, true);
                checksum = sparse_checksum(work);
                observable_checksum = checksum; // externally observable use before destruction
            } // includes destructor/free work for this arm's masks and vector
            const auto end = std::chrono::steady_clock::now();
            require(checksum == expected_checksum, "benchmark output remains observable and correct");
            require(copied == (mode == Mode::Clone ? bytes_per_clone : 0), "benchmark byte accounting");
            if (i >= 0) {
                observed_bytes += copied;
                samples.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
            }
        }
        auto sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        const auto percentile = [&](double p) {
            return sorted[static_cast<std::size_t>(std::ceil(p * sorted.size())) - 1U];
        };
        std::cout << "{\"kind\":\"ownership_benchmark\",\"arm\":\"" << (++arm == 1 ? "A1" : arm == 2 ? "B" : "A2")
            << "\",\"mode\":\"" << perception::mask_output_copy_mode_name(mode)
            << "\",\"samples\":" << samples.size() << ",\"clone_destination_bytes\":" << observed_bytes
            << ",\"checksum\":" << expected_checksum << ",\"p50_ns\":" << percentile(0.50)
            << ",\"p99_ns\":" << percentile(0.99) << ",\"max_ns\":" << sorted.back() << ",\"samples_ns\":[";
        for (std::size_t i = 0; i < samples.size(); ++i) std::cout << (i ? "," : "") << samples[i];
        std::cout << "]}" << std::endl;
        require(same_detections(fixed, golden), "benchmark has not mutated source masks");
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        const bool timing = argc == 2 && std::string(argv[1]) == "--benchmark";
        if (argc > 1 && !timing) throw std::invalid_argument("usage: test_postprocess_mask_ownership [--benchmark]");
        cv::setNumThreads(1);
        yolo::Logger::instance().set_level(yolo::LogLevel::ERROR);
        test_parsing_and_empty();
        std::vector<RetainedCase> saved;
        for (const auto& dimensions : {cv::Size(1280, 800), cv::Size(641, 479)}) {
            saved.push_back(test_real_outputs(false, dimensions.width, dimensions.height,
                yolo::Active12MaskImplementation::Serial));
            saved.push_back(test_real_outputs(true, dimensions.width, dimensions.height,
                yolo::Active12MaskImplementation::Serial));
            saved.push_back(test_real_outputs(true, dimensions.width, dimensions.height,
                yolo::Active12MaskImplementation::Parallel2));
            require(same_detections(saved[saved.size() - 1].retained, saved[saved.size() - 2].retained),
                    "real serial/parallel2 legacy outputs match");
        }
        test_release_policy();
        test_borrowed_memory_negative_control();
        test_concurrent_owners(saved);
        if (timing) benchmark();
        std::cout << "{\"kind\":\"ownership_test_result\",\"status\":\"PASS\",\"checks\":"
            << checks.load() << ",\"negative_borrow_controls\":2,\"real_output_cases\":6,"
            "\"opencv_threads\":" << cv::getNumThreads() << ",\"backend_or_camera_calls\":0}" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MASK_OWNERSHIP_TEST_FAIL: " << error.what() << std::endl;
        return 1;
    }
}
