/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: frame_sync_gst.hpp
 * Brief: Bounded PTS-to-frame metadata injector for a GStreamer HEVC or AVC (H.264) pipeline.
 *
 * Description:
 * The encoder worker remember()s each frame's metadata under its PTS before pushing the raw
 * frame; a pad probe after the hardware encoder finds the encoded access unit with that PTS and
 * inserts the SEI before its first picture NAL. At most max_pending records wait (oldest
 * expire first). Any unmatched, duplicate, expired or failed injection is counted, and the
 * encoder treats a non-zero count as a hard-contract failure.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include <gst/gst.h>

#include "common/frame_sync_sei.hpp"

namespace perception::frame_sync {

class GstSeiInjector {
public:
    struct Stats {
        std::uint64_t remembered = 0;
        std::uint64_t injected = 0;
        std::uint64_t unmatched_buffers = 0;
        std::uint64_t duplicate_pts = 0;
        std::uint64_t expired = 0;
        std::uint64_t injection_failures = 0;
        std::size_t pending = 0;
    };

    explicit GstSeiInjector(std::size_t max_pending = 120, Codec codec = Codec::Hevc);

    /**
     * @brief Records metadata before the matching raw buffer is pushed.
     */
    bool remember(GstClockTime pts, const Metadata& metadata);

    /**
     * @brief Removes a record when appsrc rejects the corresponding raw frame.
     */
    void forget(GstClockTime pts);

    Stats stats() const;

    static GstPadProbeReturn pad_probe(
        GstPad* pad,
        GstPadProbeInfo* info,
        gpointer user_data);

private:
    GstPadProbeReturn inject(GstPadProbeInfo* info);

    const std::size_t max_pending_;
    const Codec codec_;
    mutable std::mutex mutex_;
    std::map<GstClockTime, Metadata> pending_;
    std::atomic<std::uint64_t> remembered_{0};
    std::atomic<std::uint64_t> injected_{0};
    std::atomic<std::uint64_t> unmatched_buffers_{0};
    std::atomic<std::uint64_t> duplicate_pts_{0};
    std::atomic<std::uint64_t> expired_{0};
    std::atomic<std::uint64_t> injection_failures_{0};
};

}  // namespace perception::frame_sync
