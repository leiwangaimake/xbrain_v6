/* Copyright (c) 2026 Hachist Robotics
 * Brief: Raw RGB-D identity of one frameset, and the capture time the capture thread gave it.
 */
#pragma once
#include <cstdint>

namespace perception::native_depth {
struct SourceId {
    // stream_id=0 means unavailable. Source sequence starts at 1 per stream;
    // gaps are permitted (rejected/raw-only/overwritten frames remain visible).
    std::uint64_t stream_id = 0, sequence = 0;
    // SDK-managed per-stream indices: color_index need not equal depth_index.
    std::uint64_t color_index = 0, depth_index = 0;
    // Device clock microseconds, NOT Unix or host CLOCK_MONOTONIC timestamps.
    std::uint64_t color_device_us = 0, depth_device_us = 0;
    // Host CLOCK_MONOTONIC sample after SDK wait returns. NOT exposure midpoint.
    std::int64_t host_ready_mono_ns = -1;
    // Middle of the depth exposure on CLOCK_MONOTONIC, stamped once per frameset by the capture
    // thread from the camera's frame timestamp (common/capture_clock.hpp, 19 section 2.4).
    // -1: none. The capture thread hands no frameset without a stamp to the Profile or to the
    // inference, so downstream a missing stamp is a fault, not a normal case.
    std::int64_t capture_mono_ns = -1;
    // The stamp came from the host-side lower envelope alone because the SDK's mapping of the
    // camera clock was not usable: Status reports it as t_capture_estimated.
    bool capture_estimated = false;
};
}  // namespace perception::native_depth
