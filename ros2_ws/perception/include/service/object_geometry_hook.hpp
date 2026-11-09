/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: object_geometry_hook.hpp
 * Brief: The service's object geometry hook: one tracked object's measured base_link geometry.
 *
 * Description:
 * What it does. The Supervisor calls the hook once per track whose raw depth sample of this
 * inference frame is valid (SupervisorRnsContext::object_geometry). measure_object_geometry() checks
 * that the capability is there for exactly this frame (builder set up, frame on the registered
 * raster, a mask the builder can read), skips a track that was not observed in this frame, and
 * otherwise builds the footprint and height from the same pixels DepthSampler selected and writes
 * them to the detection's base_* fields. ObjectGeometryShared is the state shared with the service
 * entry: the builder of the capture session, the per-frame capability-miss flag the Objects owner
 * reads, and the counters of the audit line.
 *
 * Moved here unchanged from perception_service_main.cpp on 2026-10-13 so that a test can call the
 * very function the service runs. Header only; called only from the Supervisor processing thread.
 */

#pragma once

#include "common/types.hpp"
#include "modules/rgb_depth_registration.hpp"
#include "output/rns_fused_objects.hpp"
#include "output/rns_object_geometry.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace perception::service {

// Object geometry (19 section 4.1, reply Q12): measured base_link geometry from
// the same aligned-depth point set DepthSampler selected for this inference
// frame. Set up once after the capture reports its session calibration; until
// then, and whenever the input is not exactly that raster, the object stays in
// the list unlocatable. Called only from the Supervisor processing thread.
struct ObjectGeometryShared {
    std::mutex mutex;
    std::unique_ptr<rns::ObjectGeometryBuilder> builder;  // guarded by mutex
    yolo::DepthIntrinsics colour{};                  // guarded by mutex
    std::array<float, 9> base_from_colour_R{};       // guarded by mutex
    std::shared_ptr<const RgbDepthRegistration> registration;  // guarded by mutex: the raster the rays belong to
    std::atomic<bool> ready{false};
    // Set on the Supervisor thread when a detection of the frame being built hit
    // a capability miss (builder not ready / frame not on the registered raster).
    std::atomic<bool> frame_capability_miss{false};
    std::atomic<std::uint64_t> located{0}, too_few_points{0}, non_finite{0}, invalid_input{0}, not_ready{0},
        raster_mismatch{0}, mask_unusable{0};
    // Builds that took the sampler's selected-pixel list, and lists build() refused (then it
    // scanned the region itself; expected to stay 0).
    std::atomic<std::uint64_t> selected_list_used{0}, selected_list_rejected{0}, float_points{0};
    // Coasting tracks whose geometry was not built (they are never published), and the selected
    // pixels that build() would have projected for them.
    std::atomic<std::uint64_t> coast_skipped{0}, coast_skipped_points{0};
    // Supervisor-thread cost of build() (diagnostic): total / max ns, points. Steady-clock (wall)
    // time, so on a loaded machine it includes preemption; per-object CPU comes from the
    // object bench, not from these counters.
    std::atomic<std::uint64_t> build_calls{0}, build_ns{0}, build_max_ns{0}, build_points{0};
    // FusedObjectsReport totals of built Objects bodies.
    std::atomic<std::uint64_t> body_located{0}, body_unlocatable{0}, velocity_numeric{0}, velocity_gating_mapped{0},
        velocity_unconfirmed_numeric{0}, velocity_gated_track_unstable{0}, velocity_gated_low_depth_confidence{0},
        velocity_gated_depth_unreliable{0};
};

inline void measure_object_geometry(ObjectGeometryShared& shared, const yolo::DetectionResult& result, const cv::Rect& bbox,
                    const cv::Mat& mask, float selected_min_m, float selected_max_m,
                    const std::vector<std::uint32_t>* selected_pixels, FusedDetection& detection) {
    using Steady = std::chrono::steady_clock;
    detection.base_geometry_valid = false;
    const auto count = [](std::atomic<std::uint64_t>& c) { c.fetch_add(1, std::memory_order_relaxed); };
    // The builder can be replaced when the capture session changes; hold the lock
    // for the whole build so the rays and the raster check stay consistent.
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (!shared.builder) {
        shared.frame_capability_miss.store(true, std::memory_order_relaxed);
        return count(shared.not_ready);
    }
    const auto& k = shared.colour;
    // Only the raster the rays were built for: same size and the same
    // (uniform_scale_1) intrinsics the sampler used for this frame.
    if (!result.depth_registration || result.depth_registration != shared.registration ||
        result.depth.type() != CV_16UC1 || !result.depth.isContinuous() || result.depth.cols != k.width ||
        result.depth.rows != k.height || result.depth_intrinsics.fx != k.fx || result.depth_intrinsics.fy != k.fy ||
        result.depth_intrinsics.cx != k.cx || result.depth_intrinsics.cy != k.cy) {
        shared.frame_capability_miss.store(true, std::memory_order_relaxed);
        return count(shared.raster_mismatch);
    }
    // DepthSampler uses the mask whenever its size equals the bbox; the builder
    // indexes it flat, so a mask it cannot read as the sampler did is refused
    // rather than silently replaced by the bbox-centre region.
    const bool sampler_uses_mask = !mask.empty() && mask.size() == bbox.size();
    if (sampler_uses_mask && (mask.type() != CV_8UC1 || !mask.isContinuous())) return count(shared.mask_unusable);
    // A track that was not observed in this frame is only predicted (coasting): FusedObjectsAdapter
    // drops it by this same rule before it reads any base_* field, and nothing else reads them, so
    // its geometry is not built (2026-10-13). Only here, after the checks above: the capability
    // miss belongs to the frame and is raised whatever the object's state, and the sample, the
    // hysteresis, the fused list and the velocity history of the track are untouched.
    if (!rns::observed_in_frame(detection.last_seen_frame, result.frame_id)) {
        if (selected_pixels != nullptr)
            shared.coast_skipped_points.fetch_add(selected_pixels->size(), std::memory_order_relaxed);
        return count(shared.coast_skipped);
    }
    // The same region DepthSampler selected from, with its selected depth band.
    rns::DepthRegion region;
    region.depth = result.depth.ptr<std::uint16_t>();
    region.width = static_cast<std::uint32_t>(result.depth.cols);
    region.height = static_cast<std::uint32_t>(result.depth.rows);
    region.depth_scale = result.depth_scale;
    region.bbox_x = bbox.x;
    region.bbox_y = bbox.y;
    region.bbox_width = bbox.width;
    region.bbox_height = bbox.height;
    region.mask = sampler_uses_mask ? mask.ptr<std::uint8_t>() : nullptr;
    // The sampler's own list of the selected pixels (same region, band and order), when given.
    // An empty list is no list: the region is scanned (a valid sample selects at least one pixel).
    if (selected_pixels != nullptr && !selected_pixels->empty()) {
        region.selected = selected_pixels->data();
        region.selected_count = selected_pixels->size();
    }
    // Build cost is accumulated for the geometry audit line (diagnostic).
    const auto build_start = Steady::now();
    const auto built = shared.builder->build(region, selected_min_m, selected_max_m);
    const auto build_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now() - build_start).count());
    shared.build_calls.fetch_add(1, std::memory_order_relaxed);
    shared.build_ns.fetch_add(build_ns, std::memory_order_relaxed);
    shared.build_points.fetch_add(built.points, std::memory_order_relaxed);
    if (built.used_selected) shared.selected_list_used.fetch_add(1, std::memory_order_relaxed);
    if (built.selected_rejected) shared.selected_list_rejected.fetch_add(1, std::memory_order_relaxed);
    if (built.float_points) shared.float_points.fetch_add(1, std::memory_order_relaxed);
    if (build_ns > shared.build_max_ns.load(std::memory_order_relaxed)) shared.build_max_ns.store(build_ns, std::memory_order_relaxed);
    switch (built.status) {
    case rns::ObjectGeometryStatus::Located: break;
    case rns::ObjectGeometryStatus::TooFewPoints: return count(shared.too_few_points);
    case rns::ObjectGeometryStatus::NonFinite: return count(shared.non_finite);
    case rns::ObjectGeometryStatus::InvalidInput: return count(shared.invalid_input);
    }
    // Located: hand the measured geometry and the rotation used to the detection.
    detection.base_footprint_xy.clear();
    for (const auto& v : built.geometry->footprint_xy) detection.base_footprint_xy.push_back({v.x, v.y});
    detection.base_z_min = built.geometry->z_min;
    detection.base_z_max = built.geometry->z_max;
    detection.base_from_colour_R = shared.base_from_colour_R;
    detection.base_geometry_valid = true;
    count(shared.located);
}

}  // namespace perception::service
