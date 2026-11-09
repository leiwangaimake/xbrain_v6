/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_semantic_slot.cpp
 * Brief: Host test of the latest-wins semantic slot between inference and the Profile owner (19 section 3.4A).
 *
 * Description:
 * What it checks. Nothing published yet; a one-frame round trip where the copy owns its class
 * names; latest wins, an unread snapshot is overwritten and never queued; results skipped upstream
 * leave the last published stamp in place (B8-4); overflow, oversize names and bad vertex counts
 * are counted; future, foreign-stream and foreign-epoch snapshots are refused rather than offered;
 * a concurrent publisher and reader see whole snapshots, never halves. No SDK, OpenCV or device.
 */

#include "service/semantic_slot.hpp"

#include <iostream>
#include <string>
#include <thread>

using perception::rns::Object;
using perception::rns::ObjectGeometry;
using perception::rns::Point;
using perception::rns::SemanticMaterial;
using perception::rns::SemanticPlaneBasis;
using perception::service::SemanticSlot;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

Object located(std::int64_t id, const std::string& name, float x, float y) {
    Object o;
    o.track_id = id;
    o.class_id = 3;
    o.class_name = name;
    o.geometry.emplace(std::vector<Point>{{x, y}, {x + 0.1F, y}, {x + 0.1F, y + 0.1F}, {x, y + 0.1F}}, 0.0F, 0.5F);
    return o;
}
Object unlocatable(std::int64_t id) {
    Object o;
    o.track_id = id;
    o.class_name = "person";
    return o;                                  // no geometry: nothing to inject
}
perception::geometry::CaptureStamp stamp_of(std::uint64_t sequence, std::int64_t capture_ns) {
    perception::geometry::CaptureStamp s;
    s.source.stream_id = 7;
    s.source.sequence = sequence;
    s.clock_epoch = 42;
    s.capture_mono_ns = capture_ns;
    return s;
}
}  // namespace

int main() {
    // ---- nothing published yet ----
    {
        SemanticSlot slot;
        SemanticSlot::Snapshot snapshot;
        CHECK(slot.latest(snapshot, stamp_of(1, 1)) == nullptr && slot.published() == 0 && slot.taken() == 0);
    }
    // ---- one frame round trip; the copy owns its class names ----
    {
        SemanticSlot slot;
        perception::rns::Objects body;
        body.objects.push_back(located(11, "chair", 0.6F, -0.05F));
        body.objects.push_back(unlocatable(12));
        body.objects.push_back(located(13, "traversable_area", 1.0F, 0.2F));  // the projector excludes it, not us
        slot.publish(body, stamp_of(5, 1000), 99);
        CHECK(slot.published() == 1 && slot.objects_published() == 2 && slot.skipped_unlocatable() == 1);
        SemanticSlot::Snapshot snapshot;
        const auto* frame = slot.latest(snapshot, stamp_of(5, 1000));
        CHECK(frame && frame->count == 2 && frame->material == SemanticMaterial::MeasuredBaseFootprint);
        CHECK(frame->base_revision == 99 && frame->stamp.source.sequence == 5 && frame->stamp.capture_mono_ns == 1000);
        CHECK(frame->objects[0].track_id == 11 && frame->objects[0].class_name == "chair");
        CHECK(frame->objects[1].class_name == "traversable_area");
        CHECK(frame->objects[0].vertices == 4 && frame->objects[0].plane_basis == SemanticPlaneBasis::IndependentMeasurement);
        CHECK(frame->objects[0].footprint[0].x == 0.6F && frame->objects[0].footprint[2].y == 0.05F);
        // the views point into the reader's own storage, not the slot
        CHECK(frame->objects[0].class_name.data() == snapshot.data.names[0].data());
        // reading twice without a new publish repeats the same snapshot
        SemanticSlot::Snapshot second;
        const auto* again = slot.latest(second, stamp_of(5, 1000));
        CHECK(again && again->count == 2 && again->objects[0].class_name == "chair" && slot.taken() == 2);
    }
    // ---- latest wins: an unread snapshot is overwritten, never queued ----
    {
        SemanticSlot slot;
        perception::rns::Objects first, second;
        first.objects.push_back(located(1, "chair", 0.5F, 0.0F));
        second.objects.push_back(located(2, "box", 1.5F, 0.0F));
        slot.publish(first, stamp_of(1, 100), 3);
        slot.publish(second, stamp_of(2, 200), 3);
        SemanticSlot::Snapshot snapshot;
        const auto* frame = slot.latest(snapshot, stamp_of(3, 300));
        CHECK(frame && frame->count == 1 && frame->objects[0].track_id == 2);
        CHECK(frame->stamp.source.sequence == 2 && slot.published() == 2 && slot.taken() == 1);
    }
    // ---- B8-4: results skipped upstream leave the last published stamp in place ----
    // Nothing fresher is synthesised for results that were never published: the
    // age a depth frame sees keeps growing until the next publish replaces it.
    {
        SemanticSlot slot;
        perception::rns::Objects body;
        body.objects.push_back(located(1, "chair", 0.5F, 0.0F));
        slot.publish(body, stamp_of(1, 1000), 1);  // results 2 and 3 were overwritten upstream
        SemanticSlot::Snapshot snapshot;
        for (const std::int64_t depth_ns : {2000, 3000, 3500}) {
            const auto* frame = slot.latest(snapshot, stamp_of(static_cast<std::uint64_t>(depth_ns / 1000), depth_ns));
            CHECK(frame && frame->stamp.source.sequence == 1 && frame->stamp.capture_mono_ns == 1000);
        }
        slot.publish(body, stamp_of(4, 4000), 1);
        CHECK(slot.latest(snapshot, stamp_of(3, 3900)) == nullptr && slot.refused_future() == 1);
        const auto* frame = slot.latest(snapshot, stamp_of(4, 4000));
        CHECK(frame && frame->stamp.source.sequence == 4 && frame->stamp.capture_mono_ns == 4000);
        CHECK(slot.published() == 2 && slot.taken() == 4);
    }
    // ---- bounded: overflow, oversize names and bad vertex counts are counted ----
    {
        SemanticSlot slot;
        perception::rns::Objects body;
        for (std::size_t i = 0; i < SemanticSlot::kMaxObjects + 3; ++i)
            body.objects.push_back(located(static_cast<std::int64_t>(i), "chair", 0.5F, 0.0F));
        body.objects.push_back(located(900, std::string(SemanticSlot::kMaxNameBytes, 'x'), 0.5F, 0.0F));
        Object thin = located(901, "pole", 0.5F, 0.0F);
        thin.geometry->footprint_xy.resize(2);                 // below the contract minimum
        body.objects.push_back(thin);
        Object many = located(902, "wall", 0.5F, 0.0F);
        many.geometry->footprint_xy.resize(13, Point{0.5F, 0.0F});  // above kSemanticVertices
        body.objects.push_back(many);
        slot.publish(body, stamp_of(9, 900), 1);
        CHECK(slot.skipped_overflow() == 3 && slot.skipped_name() == 1 && slot.skipped_vertices() == 2);
        SemanticSlot::Snapshot snapshot;
        const auto* frame = slot.latest(snapshot, stamp_of(9, 900));
        CHECK(frame && frame->count == SemanticSlot::kMaxObjects);
        for (std::size_t i = 0; i < frame->count; ++i) CHECK(frame->objects[i].class_name == "chair");
    }
    // ---- refused, not offered: future / foreign stream / foreign epoch ----
    // Offering any of these would make the projector reject the whole depth
    // frame, so the Profile would stop publishing during an ordinary race.
    {
        SemanticSlot slot;
        perception::rns::Objects body;
        body.objects.push_back(located(1, "chair", 0.5F, 0.0F));
        slot.publish(body, stamp_of(10, 5000), 1);
        SemanticSlot::Snapshot snapshot;
        CHECK(slot.latest(snapshot, stamp_of(10, 4999)) == nullptr && slot.refused_future() == 1);
        CHECK(slot.latest(snapshot, stamp_of(10, 5000)) != nullptr);   // equal capture time is usable
        CHECK(slot.latest(snapshot, stamp_of(10, 6000)) != nullptr);   // the normal case: inference lags
        auto other_stream = stamp_of(10, 6000);
        other_stream.source.stream_id = 8;
        CHECK(slot.latest(snapshot, other_stream) == nullptr && slot.refused_stream() == 1);
        auto other_epoch = stamp_of(10, 6000);
        other_epoch.clock_epoch = 43;
        CHECK(slot.latest(snapshot, other_epoch) == nullptr && slot.refused_epoch() == 1);
        CHECK(slot.taken() == 2);
    }
    // ---- concurrent publisher and reader see whole snapshots, never halves ----
    {
        SemanticSlot slot;
        std::atomic<bool> stop{false};
        std::thread writer([&] {
            for (std::int64_t i = 1; !stop.load(std::memory_order_relaxed); ++i) {
                perception::rns::Objects body;
                const auto n = static_cast<std::size_t>(i % 5) + 1;
                for (std::size_t k = 0; k < n; ++k)
                    body.objects.push_back(located(i, i % 2 ? "chair" : "box", 0.5F + 0.01F * k, 0.0F));
                slot.publish(body, stamp_of(static_cast<std::uint64_t>(i), i * 1000), 1);
            }
        });
        SemanticSlot::Snapshot snapshot;
        for (int i = 0; i < 20000; ++i) {
            // a far-future depth stamp: whatever the writer has published is usable
            const auto* frame = slot.latest(snapshot, stamp_of(0, 1LL << 40));
            if (!frame) continue;
            CHECK(frame->count >= 1 && frame->count <= 5);
            const std::int64_t id = frame->objects[0].track_id;
            const std::string_view expected = id % 2 ? "chair" : "box";
            for (std::size_t k = 0; k < frame->count; ++k) {
                // one consistent frame: same track id, matching name, matching stamp
                CHECK(frame->objects[k].track_id == id && frame->objects[k].class_name == expected);
            }
            CHECK(frame->stamp.capture_mono_ns == id * 1000 &&
                  frame->stamp.source.sequence == static_cast<std::uint64_t>(id));
            if (failures) break;
        }
        stop.store(true, std::memory_order_relaxed);
        writer.join();
    }
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks
              << ",\"failures\":" << failures << ",\"scope\":\"semantic_slot_host\",\"sdk\":false,\"camera\":false}"
              << std::endl;
    return failures ? 1 : 0;
}
