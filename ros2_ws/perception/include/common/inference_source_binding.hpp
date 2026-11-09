/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: inference_source_binding.hpp
 * Brief: Carries a frame's native source identity from preparation to the published result.
 *
 * Description:
 * The identity is bound when a frame is prepared and committed exactly once when its result is
 * complete, together with the inference-completion record of the Status health journal; abandoned
 * slots never commit.
 */

#pragma once

#include <cstdint>
#include <stdexcept>

#include "common/types.hpp"
#include "output/rns_source_hooks.hpp"

namespace perception {

// Owning source metadata for one prepared input. The inference owner binds this
// before admission; background submit/post lanes never commit source events.
// This class does not mint IDs, derive timestamps, or infer success from masks.
class InferenceSourceBinding {
public:
    void bind(const yolo::FrameData& frame) noexcept {
        source_ = frame.native_depth_source;
        frame_id_ = frame.frame_id;
        bound_ = true;
        completed_ = false;
    }

    const native_depth::SourceId& source() const noexcept { return source_; }

    // Production JIT calls once after its actual output/post/merge and slot
    // checks, before queue admission. Phase labels (including calibration) do
    // not change completion semantics. No call is made for abandoned slots.
    void complete(yolo::DetectionResult& result, bool succeeded,
                  rns::SourceJournal* journal) {
        if (!bound_ || completed_ || result.frame_id != frame_id_) {
            throw std::logic_error("inference completion has unbound, repeated or wrong frame source");
        }
        result.native_depth_source = source_;
        result.inference_completed = succeeded;
        completed_ = true;
        // A journal fault is telemetry failure, not a different model result.
        // Missing native identity is left missing and rejected by the journal.
        rns::record_inference_completion(journal, result.inference_completed, source_);
    }

private:
    native_depth::SourceId source_{};
    std::uint64_t frame_id_ = 0;
    bool bound_ = false;
    bool completed_ = false;
};

}  // namespace perception
