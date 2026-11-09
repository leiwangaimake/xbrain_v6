/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_orbbec_inference_handoff.cpp
 * Brief: Unit tests for handing Orbbec frames to inference without copying or losing ownership.
 *
 * Description:
 * What it checks. The capture passes the decoded colour frame and its RGB-D source identity to the
 * inference queue. The tests cover the generic controls of the handoff (order, encoding on or off,
 * failure propagation), that the pixels inference receives are exactly the captured ones, and the
 * real ownership rules of the frame buffers.
 */

// Always test the generic ownership/order helper; target additionally exercises
// the real OpenCV owned-raster and FrameData/queue path. No SDK or model access.
#include "modules/orbbec_inference_handoff.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/rgbd_source_identity.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef PERCEPTION_HANDOFF_PORTABLE_TEST
#include "modules/orbbec_frame_output.hpp"
#include "common/types.hpp"
#endif

namespace {
std::atomic<std::size_t> checks{0};
void require(bool condition, const char* message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
using perception::orbbec_capture_detail::handoff_complete_rgbd;
using perception::orbbec_capture_detail::inference_handoff_first;

struct TestFrame {
    std::shared_ptr<std::vector<std::uint8_t>> frame, depth;
    std::int64_t timestamp = 0;
    double stamp_sec = 0;
    std::uint64_t frame_id = 0;
    perception::native_depth::SourceId native_depth_source;
    std::string camera_metadata;
};

TestFrame generic_frame() {
    TestFrame frame;
    frame.frame = std::make_shared<std::vector<std::uint8_t>>(256, 73);
    frame.depth = std::make_shared<std::vector<std::uint8_t>>(64, 8);
    frame.timestamp = 12345;
    frame.stamp_sec = 987.25;
    frame.frame_id = 77;
    frame.native_depth_source = {3, 4, 5, 6, 7, 8, 9};
    frame.camera_metadata = "unchanged_metadata";
    return frame;
}

void generic_controls() {
    require(!inference_handoff_first(nullptr), "default must retain old ordering");
    require(!inference_handoff_first("0"), "explicit off must retain old ordering");
    require(inference_handoff_first("1"), "explicit on not enabled");
    for (const char* invalid : {"", "true", "false", "2", "01", "1 ", " 0", "-1"}) {
        bool rejected = false;
        try { (void)inference_handoff_first(invalid); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "non-exact option accepted");
    }
    for (bool first : {false, true}) {
        for (bool encoding : {false, true}) {
            for (bool empty_clone : {false, true}) {
                auto frame = generic_frame();
                std::vector<std::string> events;
                int deep_copies = 0, infer_count = 0, encode_count = 0;
                handoff_complete_rgbd(first, encoding, frame,
                    [&](const TestFrame& source) {
                        events.push_back("encode");
                        require(source.frame && !source.depth, "snapshot RGB only and still alive");
                        require(source.timestamp == 12345 && source.frame_id == 77 &&
                                source.stamp_sec == 987.25 && source.native_depth_source.depth_index == 6 &&
                                source.camera_metadata == "unchanged_metadata", "encoding metadata changed");
                        require(*source.frame == std::vector<std::uint8_t>(256, 73), "source pixels changed");
                        ++deep_copies;
                        if (empty_clone) return;  // original branch warns but still permits inference
                        TestFrame encoded;
                        encoded.frame = std::make_shared<std::vector<std::uint8_t>>(*source.frame);
                        require(encoded.frame.get() != source.frame.get(), "encoding must remain independent");
                        ++encode_count;
                    },
                    [&](TestFrame& input) {
                        events.push_back("infer");
                        ++infer_count;
                        yolo::ThreadsafeQueue<TestFrame> queue(1);
                        queue.push(std::move(input));
                        TestFrame consumed;
                        require(queue.try_pop(consumed), "synchronous queue consumer missing frame");
                        require(consumed.frame && consumed.depth && consumed.frame_id == 77,
                                "inference must get complete RGB-D exactly once");
                        require(*consumed.frame == std::vector<std::uint8_t>(256, 73), "inference bytes changed");
                        consumed = {};  // simulate earliest possible destruction before encode clones
                    });
                const auto expected = !encoding ? std::vector<std::string>{"infer"} : first
                    ? std::vector<std::string>{"infer", "encode"} : std::vector<std::string>{"encode", "infer"};
                require(events == expected, "selected order or disabled encoder changed");
                require(infer_count == 1 && deep_copies == static_cast<int>(encoding) &&
                        encode_count == static_cast<int>(encoding && !empty_clone), "extra handoff/clone");
                require(!frame.frame && !frame.depth, "input must be moved, not shallow-copied to inference");
            }
        }
        // Any encode-stage throw propagates; B's completed inference cannot be
        // recalled. Simulate both clone failure and queue-push failure.
        for (int failure : {0, 1}) {
            auto frame = generic_frame();
            int infers = 0, encodes = 0;
            bool caught = false;
            try {
                handoff_complete_rgbd(first, true, frame,
                    [&](const TestFrame& source) {
                        if (failure == 0) throw std::runtime_error("clone failure");
                        auto independent = *source.frame;
                        require(!independent.empty(), "clone simulation invalid");
                        ++encodes;
                        throw std::runtime_error("encode push failure");
                    },
                    [&](TestFrame& input) { ++infers; TestFrame received = std::move(input); });
            } catch (const std::runtime_error&) { caught = true; }
            require(caught && infers == static_cast<int>(first) && encodes == failure,
                    "exception swallowed, retried or inference rollback fabricated");
        }
        auto frame = generic_frame();
        int encodes = 0, infers = 0;
        bool caught = false;
        try {
            handoff_complete_rgbd(first, true, frame,
                [&](const TestFrame&) { ++encodes; },
                [&](TestFrame&) { ++infers; throw std::runtime_error("inference push failure"); });
        } catch (const std::runtime_error&) { caught = true; }
        require(caught && infers == 1 && encodes == static_cast<int>(!first),
                "inference failure must stop selected order without retry");
        // With encoding off the same inference failure still propagates, and
        // the encode stage is never called in either order.
        {
            auto off_frame = generic_frame();
            int off_encodes = 0, off_infers = 0;
            bool off_caught = false;
            try {
                handoff_complete_rgbd(first, false, off_frame,
                    [&](const TestFrame&) { ++off_encodes; },
                    [&](TestFrame&) { ++off_infers; throw std::runtime_error("inference push failure"); });
            } catch (const std::runtime_error&) { off_caught = true; }
            require(off_caught && off_infers == 1 && off_encodes == 0,
                    "encoding off must not swallow an inference failure or call the encoder");
        }

        yolo::ThreadsafeQueue<TestFrame> infer_queue(1), encoder_queue(1);
        for (std::uint64_t id = 1; id <= 3; ++id) {
            auto newest = generic_frame();
            newest.frame_id = id;
            handoff_complete_rgbd(first, true, newest,
                [&](const TestFrame& source) {
                    TestFrame encoded;
                    encoded.frame = std::make_shared<std::vector<std::uint8_t>>(*source.frame);
                    encoded.frame_id = source.frame_id;
                    encoder_queue.push(std::move(encoded));
                },
                [&](TestFrame& input) { infer_queue.push(std::move(input)); });
        }
        TestFrame newest_infer, newest_encode;
        require(infer_queue.dropped_count() == 2 && encoder_queue.dropped_count() == 2,
                "bounded queues must preserve drop-old policy in both orders");
        require(infer_queue.try_pop(newest_infer) && encoder_queue.try_pop(newest_encode) &&
                newest_infer.frame_id == 3 && newest_encode.frame_id == 3,
                "latest surviving source IDs must remain consistent");
    }
}

#ifndef PERCEPTION_HANDOFF_PORTABLE_TEST
bool same_pixels(const cv::Mat& a, const cv::Mat& b) {
    if (a.size() != b.size() || a.type() != b.type()) return false;
    const auto row_bytes = static_cast<std::size_t>(a.cols) * a.elemSize();
    for (int row = 0; row < a.rows; ++row)
        if (std::memcmp(a.ptr(row), b.ptr(row), row_bytes) != 0) return false;
    return true;
}

void real_ownership(bool first, bool concurrent, cv::Size shape) {
    yolo::FrameData frame;
    cv::Mat expected_rgb, expected_depth;
    {
        const auto pixels = static_cast<std::size_t>(shape.width) * shape.height;
        std::vector<std::uint8_t> sdk_rgb(pixels * 3);
        std::vector<std::uint16_t> sdk_depth(pixels);
        for (std::size_t i = 0; i < sdk_rgb.size(); ++i) sdk_rgb[i] = static_cast<std::uint8_t>(i * 31 + i / 7);
        for (std::size_t i = 0; i < sdk_depth.size(); ++i) sdk_depth[i] = static_cast<std::uint16_t>(i * 11);
        cv::Mat rgb_view(shape.height, shape.width, CV_8UC3, sdk_rgb.data());
        cv::Mat depth_view(shape.height, shape.width, CV_16UC1, sdk_depth.data());
        frame.frame = perception::orbbec_capture_detail::copy_or_resize_owned(rgb_view, shape, cv::INTER_LINEAR);
        frame.depth = perception::orbbec_capture_detail::copy_or_resize_owned(depth_view, shape, cv::INTER_NEAREST_EXACT);
        expected_rgb = rgb_view.clone();
        expected_depth = depth_view.clone();
        require(frame.frame.u && frame.depth.u && frame.frame.data != sdk_rgb.data(), "SDK memory still borrowed");
        std::fill(sdk_rgb.begin(), sdk_rgb.end(), 0);
        std::fill(sdk_depth.begin(), sdk_depth.end(), 65535);
    }  // backing SDK-like buffers freed BEFORE either branch receives the frame
    frame.timestamp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(12345));
    frame.stamp_sec = 123.25;
    frame.frame_id = 919;
    frame.native_depth_source = {10, 20, 30, 40, 50, 60, 70};
    frame.camera_metadata.source = "test_metadata";
    frame.camera_metadata.exposure_valid = false;
    frame.camera_metadata.gain_valid = false;
    frame.depth_scale = 0.001F;
    frame.depth_intrinsics = {11, 22, 33, 44, shape.width, shape.height};
    const auto* inference_rgb_pointer = frame.frame.data;
    yolo::ThreadsafeQueue<yolo::FrameData> inference_queue(1), encode_queue(1);
    std::atomic<bool> reading{false}, finish{false};
    std::thread consumer;
    std::exception_ptr failure;
    int clone_calls = 0, infer_calls = 0, encode_calls = 0;
    auto inspect = [&](const yolo::FrameData& received) {
        require(same_pixels(received.frame, expected_rgb) && same_pixels(received.depth, expected_depth),
                "real synchronous/concurrent consumer pixel mismatch");
        require(received.frame_id == 919 && received.native_depth_source.sequence == 20 &&
                received.depth_scale == 0.001F && received.depth_intrinsics.fx == 11,
                "inference payload metadata changed");
    };
    try {
        handoff_complete_rgbd(first, true, frame,
            [&](const yolo::FrameData& source) {
                ++encode_calls;
                require(source.depth.empty() && source.frame.data == inference_rgb_pointer,
                        "encoding snapshot must be shallow RGB-only owner");
                require(source.frame_id == 919 && source.native_depth_source.color_index == 30 &&
                        source.native_depth_source.depth_device_us == 60 && source.stamp_sec == 123.25 &&
                        source.timestamp.time_since_epoch() == std::chrono::nanoseconds(12345) &&
                        source.camera_metadata.source == "test_metadata" && !source.camera_metadata.exposure_valid,
                        "encoding snapshot metadata incomplete");
                yolo::FrameData encoded;
                ++clone_calls;
                encoded.frame = source.frame.clone();
                encoded.frame_id = source.frame_id;
                encode_queue.push(std::move(encoded));
            },
            [&](yolo::FrameData& input) {
                ++infer_calls;
                input.capture_time_ms = 7.125;
                inference_queue.push(std::move(input));
                yolo::FrameData received;
                require(inference_queue.try_pop(received), "real inference queue lost frame");
                require(received.capture_time_ms == 7.125, "capture time not frozen before handoff");
                if (!concurrent) {
                    inspect(received);
                    received = {};  // destroy original owner's RGB before B encode
                    return;
                }
                consumer = std::thread([&, owned = std::move(received)]() mutable {
                    reading.store(true, std::memory_order_release);
                    try {
                        do { inspect(owned); } while (!finish.load(std::memory_order_acquire));
                    } catch (...) { failure = std::current_exception(); }
                });
                while (!reading.load(std::memory_order_acquire)) std::this_thread::yield();
            });
    } catch (...) {
        finish.store(true, std::memory_order_release);
        if (consumer.joinable()) consumer.join();
        throw;
    }
    finish.store(true, std::memory_order_release);
    if (consumer.joinable()) consumer.join();
    if (failure) std::rethrow_exception(failure);
    require(infer_calls == 1 && encode_calls == 1 && clone_calls == 1, "real duplicate copy/handoff");
    require(frame.frame.empty() && frame.depth.empty(), "real FrameData must be moved");
    yolo::FrameData encoded;
    require(encode_queue.try_pop(encoded) && !encode_queue.try_pop(frame), "exactly one encoded frame required");
    require(encoded.frame.data != inference_rgb_pointer && same_pixels(encoded.frame, expected_rgb),
            "encoded raster must be independent and bitwise equal");
    require(inference_queue.dropped_count() == 0 && encode_queue.dropped_count() == 0, "unexpected queue policy change");
}
#endif
}  // namespace

int main() {
    try {
        generic_controls();
#ifndef PERCEPTION_HANDOFF_PORTABLE_TEST
        cv::setNumThreads(1);
        for (bool first : {false, true})
            for (bool concurrent : {false, true})
                for (cv::Size shape : {cv::Size(1280, 800), cv::Size(37, 19)})
                    real_ownership(first, concurrent, shape);
        constexpr const char* scope = "REAL_OPENCV_AND_PORTABLE_OWNERSHIP";
#else
        constexpr const char* scope = "PORTABLE_OWNERSHIP_MODEL_ONLY";
#endif
        std::cout << "ORBBEC_INFERENCE_HANDOFF_TEST PASS checks=" << checks.load()
                  << " scope=" << scope << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ORBBEC_INFERENCE_HANDOFF_TEST FAIL " << error.what() << '\n';
        return 1;
    }
}
