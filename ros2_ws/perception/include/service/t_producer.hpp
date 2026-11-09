/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_producer.hpp
 * Brief: The Objects-side T producer: the raw class-92 union of one inference frame into the T slot.
 *
 * Description:
 * Plan 2f; also its per-run state shared with the Profile and Status owners.
 * Called once per built Objects body, after that body was published. It publishes only while a T
 * owner consumes the slot (ready) and T was not disabled (locked):
 *   - no class-92 mask at all -> an all-zero mask ("looked, nothing traversable"; accepted in
 *     issues.md 3.2(1), 2026-10-08);
 *   - a 1280x800 single-byte union -> published (the slot binarises 0/255 to 0/1);
 *   - any other raster -> counted, not published;
 *   - an exception (for example a bbox-local first class-92 mask, which the union refuses) ->
 *     T is locked for this run and the message returned for a loud log; Objects and Profile go on
 *     (user decision 2026-09-30, B9-9 (c): a T fault must not stop the service, but it is not a
 *     normal degradation either, so it is never retried silently).
 * Raw class 92, never safe_traversable (11 section 3.1B.1).
 */

#pragma once

#include "common/types.hpp"
#include "service/t_mask_slot.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace perception::service {

struct TProducerState {
    std::atomic<bool> ready{false};    // a T owner consumes the slot; publish only then
    std::atomic<bool> locked{false};   // a producer fault disabled T for this run
    // The Profile thread could not set up the T colour projection (calibration or registration
    // gate): Status raises the free reason t_projection_unavailable (issues.md 3.3, 2026-10-08).
    std::atomic<bool> projection_unavailable{false};
    std::atomic<std::uint64_t> raster_mismatch{0}, faults{0}, seg_in_open_ms{0}, clock_anomaly{0};
};

enum class TProduceOutcome { NotReady, Locked, Published, PublishedEmpty, RasterMismatch, Refused, Fault };

// `fault` receives the exception text on Fault (may be null).
TProduceOutcome produce_traversable(TMaskSlot& slot, TProducerState& state,
                                    const std::vector<yolo::Detection>& map_detections,
                                    const geometry::CaptureStamp& stamp, std::string* fault) noexcept;

}  // namespace perception::service
