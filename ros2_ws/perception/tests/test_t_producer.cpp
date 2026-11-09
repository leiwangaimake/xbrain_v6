/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_t_producer.cpp
 * Brief: produce_traversable: what the Objects side puts into the T slot, and how a fault locks T.
 *
 * Description:
 * 1. Before a T owner exists nothing is published.
 * 2. No class-92 detection: an all-zero mask is published (interim for issues.md 3.2(1)).
 * 3. A full-image class-92 mask (0/255) is published and read back binarised to 0/1, other
 *    classes ignored.
 * 4. A union of another size is counted and not published.
 * 5. A bbox-local first class-92 mask makes the union throw: T is locked for the run with the
 *    message returned, and later frames publish nothing (no silent retry).
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "service/t_producer.hpp"

#include <opencv2/core.hpp>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using perception::geometry::CaptureStamp;
using perception::service::TMaskSlot;
using perception::service::TProduceOutcome;
using perception::service::TProducerState;
using perception::service::produce_traversable;

int checks = 0;

void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

CaptureStamp stamp_of(std::uint64_t g) {
    CaptureStamp s;
    s.source.stream_id = 3;
    s.source.sequence = g;
    s.source.color_index = s.source.depth_index = g;
    s.source.color_device_us = 1000 * g;
    s.source.depth_device_us = 1000 * g + 1;
    s.source.host_ready_mono_ns = static_cast<std::int64_t>(1000000 * g) + 50000000;
    s.clock_epoch = 1;
    s.capture_mono_ns = s.source.host_ready_mono_ns - 23000000;
    return s;
}

yolo::Detection detection(int class_id, const cv::Rect& bbox, const cv::Mat& full_mask) {
    yolo::Detection d;
    d.class_id = class_id;
    d.bbox = bbox;
    d.full_image_mask = full_mask;
    d.mask = cv::Mat(bbox.size(), CV_8UC1, cv::Scalar(255));
    d.source = yolo::DetectionSource::active12;
    return d;
}

}  // namespace

int main() {
    try {
        TMaskSlot slot;
        TProducerState state;
        TMaskSlot::Snapshot snapshot;
        // 1. no T owner yet
        check(produce_traversable(slot, state, {}, stamp_of(1), nullptr) == TProduceOutcome::NotReady &&
              slot.counters().published_empty == 0, "published before a T owner exists");
        state.ready.store(true);
        // 2. no class 92: all-zero mask
        check(produce_traversable(slot, state, {}, stamp_of(2), nullptr) == TProduceOutcome::PublishedEmpty,
              "no class 92 did not publish the empty mask");
        const auto* mask = slot.latest(snapshot, stamp_of(2), perception::service::kTProjectionRevision);
        check(mask != nullptr && snapshot.empty && mask->pixels[640 * 1280 + 3] == 0, "empty mask not all zero");
        // 3. a full-image class-92 mask, 0/255, next to another class
        cv::Mat ground = cv::Mat::zeros(800, 1280, CV_8UC1);
        ground(cv::Rect(100, 400, 1000, 400)).setTo(255);
        const std::vector<yolo::Detection> map{detection(92, cv::Rect(100, 400, 1000, 400), ground)};
        check(produce_traversable(slot, state, map, stamp_of(3), nullptr) == TProduceOutcome::Published,
              "class-92 union not published");
        mask = slot.latest(snapshot, stamp_of(3), perception::service::kTProjectionRevision);
        check(mask != nullptr && !snapshot.empty, "class-92 mask not taken");
        check(mask->pixels[500 * 1280 + 200] == 1 && mask->pixels[100 * 1280 + 200] == 0 &&
              mask->pixels[500 * 1280 + 50] == 0, "class-92 mask not binarised to 0/1 in place");
        // 4. another raster size is counted, not published
        const std::vector<yolo::Detection> small{detection(92, cv::Rect(0, 0, 10, 10), cv::Mat::zeros(400, 640, CV_8UC1))};
        check(produce_traversable(slot, state, small, stamp_of(4), nullptr) == TProduceOutcome::RasterMismatch &&
              state.raster_mismatch.load() == 1 && slot.counters().published == 1, "wrong size not refused");
        // 5. a bbox-local first class-92 mask: the union throws, T is locked, later frames do nothing
        const std::vector<yolo::Detection> local{detection(92, cv::Rect(10, 10, 20, 20), cv::Mat())};
        std::string fault;
        check(produce_traversable(slot, state, local, stamp_of(5), &fault) == TProduceOutcome::Fault &&
              state.locked.load() && state.faults.load() == 1 && !fault.empty(), "fault did not lock T");
        check(produce_traversable(slot, state, map, stamp_of(6), nullptr) == TProduceOutcome::Locked &&
              slot.counters().published == 1, "locked T still published");
        std::cout << "test_t_producer: PASS checks=" << checks << " fault=\"" << fault << "\"\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_t_producer: FAIL " << e.what() << "\n";
        return 1;
    }
}
