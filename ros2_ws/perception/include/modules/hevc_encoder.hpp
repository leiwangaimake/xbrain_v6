/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: hevc_encoder.hpp
 * Brief: Isolated GStreamer NVENC encoder of the RGB preview (R6: the MED-2 stream).
 *
 * Description:
 * The class name and "Hevc" in it are historical. R6 runs it only in the med2_loopback sink
 * that the service configures (19 section 13.1): H.264 by default, centre crop and capture-time
 * frame selection to the preview size and rate, frame-sync SEI, RTP to a loopback port and an
 * RTSP relay on 127.0.0.1. The file_rtsp (MP4 + RTSP) and stream_rtsp sinks are V5 modes that
 * the R6 service cannot select. A failed encoder or SEI contract sets failed(), and the
 * Supervisor heartbeat then stops the service (Gemini hard contract).
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "common/config.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/module_base.hpp"

namespace perception {

class HevcEncoder final : public yolo::ModuleBase {
public:
    struct FrameSyncStats {
        std::uint64_t remembered = 0;
        std::uint64_t injected = 0;
        std::uint64_t unmatched_buffers = 0;
        std::uint64_t duplicate_pts = 0;
        std::uint64_t expired = 0;
        std::uint64_t injection_failures = 0;
        std::size_t pending = 0;
    };

    // Stream-only (sink stream_rtsp) explicit accounting. Every frame handed to the
    // worker is either pushed into appsrc or counted as a backpressure drop;
    // appsrc_out/encoder_out are pad probes; eos_drained is set by stop().
    struct MediaAccounting {
        bool med2 = false;
        std::uint64_t received = 0;
        std::uint64_t pushed = 0;
        std::uint64_t backpressure_drops = 0;
        std::uint64_t rate_skipped = 0;        // intentional preview frame selection (not a drop)
        std::uint64_t out_of_order = 0, out_of_order_skipped = 0;   // capture time not after the previous frame
        std::uint64_t appsrc_out = 0;
        std::uint64_t encoder_out = 0;
        bool eos_drained = false;
    };

    HevcEncoder(
        const yolo::Config& config,
        std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> input_queue);
    ~HevcEncoder() override;

    bool start() override;
    void stop() override;
    std::uint64_t encoded_frames() const;
    bool failed() const;
    int rtsp_client_count() const;
    FrameSyncStats frame_sync_stats() const;
    MediaAccounting media_accounting() const;

private:
    struct Impl;

    bool config_is_valid() const;
    bool launch_pipeline();
    bool launch_stream_pipeline();
    bool push_stream_frame(const yolo::FrameData& frame);
    bool start_rtsp_server();
    bool push_frame(const yolo::FrameData& frame);
    void worker_loop();
    void stop_rtsp_server();
    void stop_pipeline();

    const yolo::Config& config_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> input_queue_;
    std::unique_ptr<Impl> impl_;
    std::thread worker_;
    std::atomic<std::uint64_t> encoded_frames_{0};
    std::atomic<bool> failed_{false};
    mutable std::mutex frame_sync_stats_mutex_;
    FrameSyncStats frame_sync_stats_;
};

}  // namespace perception
