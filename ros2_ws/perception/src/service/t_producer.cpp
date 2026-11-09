/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_producer.cpp
 * Brief: produce_traversable (see t_producer.hpp).
 *
 * Description:
 * The class-92 union (common/dual_model_merge) is the unchanged plan-2f input; this file only
 * decides what goes into the slot and turns a fault into a locked T instead of a stopped service.
 */
#include "service/t_producer.hpp"

#include "common/dual_model_merge.hpp"

#include <exception>

namespace perception::service {

TProduceOutcome produce_traversable(TMaskSlot& slot, TProducerState& state,
                                    const std::vector<yolo::Detection>& map_detections,
                                    const geometry::CaptureStamp& stamp, std::string* fault) noexcept {
    // Nothing is published before the Profile thread owns a T consumer (its setup needs the
    // first depth frame), and nothing after a fault locked T for this run (B9-9 (c)).
    if (!state.ready.load(std::memory_order_acquire)) return TProduceOutcome::NotReady;
    if (state.locked.load(std::memory_order_relaxed)) return TProduceOutcome::Locked;
    try {
        // The raw class-92 union of this inference frame (never safe_traversable).
        const cv::Mat traversable = yolo::union_traversable_mask(map_detections);
        // No class-92 detection: an all-zero mask, "looked, nothing traversable", which keeps
        // the segmentation fresh (accepted by the integration lead, issues.md 3.2 (1), 2026-10-08).
        if (traversable.empty())
            return slot.publish_empty(stamp, kTProjectionRevision) ? TProduceOutcome::PublishedEmpty
                                                                  : TProduceOutcome::Refused;
        // The slot takes only the full colour raster; anything else is counted, not resized,
        // because a resized mask would no longer match the calibrated projection.
        if (traversable.type() != CV_8UC1 || traversable.cols != static_cast<int>(TMaskSlot::kWidth) ||
            traversable.rows != static_cast<int>(TMaskSlot::kHeight)) {
            state.raster_mismatch.fetch_add(1, std::memory_order_relaxed);
            return TProduceOutcome::RasterMismatch;
        }
        return slot.publish(traversable.ptr<std::uint8_t>(0), TMaskSlot::kWidth, TMaskSlot::kHeight,
                            traversable.step[0], stamp, kTProjectionRevision)
                   ? TProduceOutcome::Published
                   : TProduceOutcome::Refused;
    } catch (const std::exception& e) {
        // A fault stops T for the rest of the run instead of the whole service: Objects and
        // Profile go on, Status reports no_traversable_seg once the 10 s window expires.
        state.locked.store(true, std::memory_order_relaxed);
        state.faults.fetch_add(1, std::memory_order_relaxed);
        // Copying the message may itself throw (allocation); the function stays noexcept.
        if (fault) {
            try { *fault = e.what(); } catch (...) {}
        }
        return TProduceOutcome::Fault;
    } catch (...) {
        state.locked.store(true, std::memory_order_relaxed);
        state.faults.fetch_add(1, std::memory_order_relaxed);
        if (fault) {
            try { *fault = "nonstandard exception"; } catch (...) {}
        }
        return TProduceOutcome::Fault;
    }
}

}  // namespace perception::service
