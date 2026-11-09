/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_inference_handoff.hpp
 * Brief: Order of the capture's two handoffs: MED-2 encoder queue and inference queue.
 *
 * Description:
 * One-variable handoff ordering. No image arithmetic or queue policy changes.
 * PERCEPTION_INFERENCE_HANDOFF_FIRST (runtime switch inference_handoff_first, production 0)
 * selects whether inference or encoding receives the frame first; capture_time_ms of the
 * inference frame covers only the work done before its own handoff.
 */
#pragma once

#include <stdexcept>
#include <string_view>
#include <utility>

namespace perception::orbbec_capture_detail {

inline bool inference_handoff_first(const char* value) {
    if (!value || std::string_view(value) == "0") return false;
    if (std::string_view(value) == "1") return true;
    throw std::invalid_argument("PERCEPTION_INFERENCE_HANDOFF_FIRST must be 0 or 1");
}

// Frame must contain an OWNED, immutable, reference-counted RGB image. In the
// Gemini caller copy_or_resize_owned has already detached it from SDK memory.
// The inference consumer is allowed to read/destroy its Frame but NOT mutate
// those image pixels. Encoding clones exactly once inside encode_stage, so
// its eventual queue consumer retains the same independent pixels as before.
// (With the media stream on, as in production, encode_stage shares the same immutable image
// instead of cloning it; see OrbbecCapture::capture_loop.)
// No depth alias, extra deep copy, queue mutation, retry, or extra task exists.
template <typename Frame, typename EncodeStage, typename InferenceStage>
void handoff_complete_rgbd(bool inference_first, bool encoding_enabled, Frame& frame,
                         EncodeStage&& encode_stage, InferenceStage&& inference_stage) {
    if (!encoding_enabled) {
        inference_stage(frame);
        return;
    }
    // Snapshot only fields the original encoding branch copied. This is a
    // shallow reference-counted RGB owner, not a pointer into moved FrameData.
    const Frame encoding_source = [&frame] {
        Frame snapshot;
        snapshot.frame = frame.frame;
        snapshot.timestamp = frame.timestamp;
        snapshot.stamp_sec = frame.stamp_sec;
        snapshot.frame_id = frame.frame_id;
        snapshot.native_depth_source = frame.native_depth_source;
        snapshot.camera_metadata = frame.camera_metadata;
        return snapshot;
    }();
    if (inference_first) {
        inference_stage(frame);
        // If encoding throws, the inference handoff cannot be withdrawn. Let
        // the existing capture exception handler stop; never retry either arm.
        encode_stage(encoding_source);
    } else {
        encode_stage(encoding_source);
        inference_stage(frame);
    }
}

}  // namespace perception::orbbec_capture_detail
