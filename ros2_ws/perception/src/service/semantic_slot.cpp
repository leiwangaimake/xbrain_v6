/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: semantic_slot.cpp
 * Brief: SemanticSlot publish (Objects side) and latest (Profile side).
 *
 * Description:
 * What problem this solves. Implements semantic_slot.hpp. publish() stages a complete snapshot
 * outside the lock and swaps it in under the lock, so the Profile thread never sees a half
 * written snapshot and never waits for more than one fixed-size copy. latest() copies out under
 * the lock and does all checks on its private copy.
 *
 * Counters. Every object that does not enter the slot and every snapshot the reader refuses is
 * counted by reason; the service logs them, so S being absent is always explainable.
 */
#include "service/semantic_slot.hpp"

#include <algorithm>
#include <cstring>

namespace perception::service {

void SemanticSlot::publish(const rns::Objects& body, const geometry::CaptureStamp& stamp,
                           std::uint64_t base_revision) noexcept {
    // Stage the snapshot without the lock; skip, never repair, unusable objects.
    Objects staged;
    for (const auto& object : body.objects) {
        if (!object.geometry) {            // unlocatable: nothing measured to inject
            skipped_unlocatable_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        const auto& footprint = object.geometry->footprint_xy;
        // The projector's vertex limits; anything else cannot be projected.
        if (footprint.size() < 3 || footprint.size() > rns::kSemanticVertices) {
            skipped_vertices_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        // The snapshot is fixed-size (no allocation on either thread), so a name must fit its
        // buffer with the terminator; an empty name cannot be shown to RNS either.
        if (object.class_name.empty() || object.class_name.size() >= kMaxNameBytes) {
            skipped_name_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        // Capacity is above the design load (32 objects per frame, 2026-09-29); beyond it the
        // extra objects are counted, never truncated silently.
        if (staged.count >= kMaxObjects) {
            skipped_overflow_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        // Own copy of the class name, NUL-terminated, with its length.
        auto& name = staged.names[staged.count];
        std::memcpy(name.data(), object.class_name.data(), object.class_name.size());
        name[object.class_name.size()] = '\0';
        staged.name_len[staged.count] = static_cast<std::uint8_t>(object.class_name.size());
        auto& out = staged.objects[staged.count];
        out.track_id = object.track_id;
        out.class_id = object.class_id;
        out.class_name = {};               // bound by the reader to its own copy
        out.vertices = footprint.size();
        for (std::size_t i = 0; i < footprint.size(); ++i)
            out.footprint[i] = {footprint[i].x, footprint[i].y};
        // The footprint is built from depth returns inside the object mask and
        // the configured base <- camera transform; no ground-plane estimate is
        // involved, which is what IndependentMeasurement states.
        out.plane_basis = rns::SemanticPlaneBasis::IndependentMeasurement;
        ++staged.count;
    }
    // The snapshot identity: capture stamp of its inference frame and the base
    // transform revision the footprints were built with.
    staged.stamp = stamp;
    staged.base_revision = base_revision;
    staged.valid = true;
    // The only work under the lock is one fixed-size copy, so the Profile thread's wait is
    // bounded and short.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        slot_ = staged;                    // latest wins: the older snapshot is dropped
    }
    published_.fetch_add(1, std::memory_order_relaxed);
    objects_published_.fetch_add(staged.count, std::memory_order_relaxed);
}

const rns::SemanticFrame* SemanticSlot::latest(Snapshot& into, const geometry::CaptureStamp& depth) noexcept {
    // Copy out under the lock; everything below works on the private copy.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!slot_.valid) return nullptr;
        into.data = slot_;
    }
    const auto& stamp = into.data.stamp;
    if (stamp.source.stream_id != depth.source.stream_id) {
        refused_stream_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;                    // a snapshot from a retired stream
    }
    // Ages are only comparable within one calibrated clock epoch (the capture-time estimate is
    // re-anchored when it changes).
    if (stamp.clock_epoch != depth.clock_epoch) {
        refused_epoch_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    if (stamp.capture_mono_ns > depth.capture_mono_ns) {
        refused_future_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;                    // inference is ahead of this depth frame
    }
    // The class_name views of the copy must point into the copy, not the slot.
    for (std::size_t i = 0; i < into.data.count; ++i)
        into.data.objects[i].class_name =
            std::string_view(into.data.names[i].data(), into.data.name_len[i]);
    // Present the copy as the projector's input view. MeasuredBaseFootprint: measured base_link
    // footprints, the only material the projector accepts (an image box plus depth is a view
    // frustum, not ground occupancy).
    into.frame.stamp = into.data.stamp;
    into.frame.base_revision = into.data.base_revision;
    into.frame.material = rns::SemanticMaterial::MeasuredBaseFootprint;
    into.frame.objects = into.data.objects.data();
    into.frame.count = into.data.count;
    taken_.fetch_add(1, std::memory_order_relaxed);
    return &into.frame;
}

}  // namespace perception::service
