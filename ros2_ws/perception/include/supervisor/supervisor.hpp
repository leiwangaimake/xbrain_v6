/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: supervisor.hpp
 * Brief: R6 perception pipeline supervisor interface.
 *
 * Description:
 *   1. Owns the lifecycle of RGB-D capture, the MED-2 HEVC encoder, inference and tracking.
 *   2. Runs the detection-to-Objects processing loop and the contract heartbeat loop.
 *   3. Requires the R6 wiring (configure_rns) before init; see supervisor.cpp.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "common/config.hpp"
#include "common/odom_transform.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/track_velocity.hpp"
#include "common/types.hpp"
#include "modules/bytetrack.hpp"
#include "modules/depth_sampler.hpp"
#include "modules/hevc_encoder.hpp"
#include "modules/inference_module.hpp"
#include "modules/odom_buffer.hpp"
#include "modules/rgbd_capture.hpp"
#include "output/rns_fused_objects.hpp"
#include "supervisor/result_handoff_ledger.hpp"

namespace perception {

struct PipelineRuntimeAudit {
    bool available = false, camera_running = false, inference_running = false;
    bool sdk_stop_available = false;
    std::uint64_t sdk_stop_attempts = 0, sdk_stop_successes = 0, sdk_stop_failures = 0;
    std::uint64_t input_dropped = 0, result_dropped = 0, encode_dropped = 0;
    std::uint64_t rns_prepared = 0, rns_callback_attempts = 0, rns_callback_successes = 0;
    std::uint64_t depth_arrivals = 0, inference_completions = 0;
    std::uint64_t encoded_frames = 0;
};

// Counters of one inter-thread queue; pushed == popped + dropped + cleared +
// pending holds once no push or pop is in flight (after stop()).
struct QueueAccounting {
    std::uint64_t pushed = 0, popped = 0, dropped = 0, cleared = 0, pending = 0;
    bool balanced() const { return pushed == popped + dropped + cleared + pending; }
};

// Capture -> inference input queue and inference -> Supervisor result handoff
// (19 P19-2 latest-wins when result_queue_size is 1), with the frame_id ledger
// of the result handoff. dropped = overwritten by a newer item.
struct ResultHandoffSnapshot {
    bool available = false, result_latest_wins = false;
    QueueAccounting input, result;
    ResultHandoffWindow ledger;
};

/**
 * @brief Top-level owner and lifecycle manager for the perception service.
 */
class Supervisor {
public:
    Supervisor();
    ~Supervisor();

    bool init(const yolo::Config& config);
    // Lifecycle-owner calls only. Configure once before first init; the shared
    // context is retained through stop. R6 instances admit one start only.
    bool configure_rns(rns::SupervisorRnsContext context);
    // Plain worker counters are exposed only after stop() has joined workers.
    PipelineRuntimeAudit runtime_audit() const;
    // Queue counters plus the result ledger window since the previous call.
    // Lifecycle-owner thread only, after init() (also after stop()).
    ResultHandoffSnapshot drain_result_handoff();
    // Atomic inference observations after successful init; no concurrent init
    // or destruction. R6 forbids re-init and stop does not replace the module.
    bool jit_calibration_complete() const noexcept;
    std::int64_t jit_frozen_delay_us() const noexcept;
    bool start();
    void stop();

    /** @brief Blocks until request_shutdown() or stop() is called. */
    void run();

    /** @brief Wakes run() so main signal handling can exit cleanly. */
    void request_shutdown();

    /** @brief Reports a supervisor-initiated fatal shutdown request to ROS wrappers. */
    bool shutdown_requested() const;

    /** @brief Distinguishes a hard-contract runtime failure from a graceful shutdown. */
    bool fatal_failure_detected() const;

    /** @brief Returns the supervisor-owned odom buffer for ROS2 odometry ingestion. */
    OdomBuffer* odom_buffer() { return odom_buffer_.get(); }
    // SDK colour calibration of the running capture stream (nullopt otherwise).
    std::optional<yolo::RgbdCalibration> rgbd_calibration() const {
        return rgbd_capture_ ? rgbd_capture_->rgbd_calibration() : std::nullopt;
    }


    // T11 capture worker (the V5 evidence-capture command, kept in R6 on the ROS topics
    // /perception/capture_cmd_json -> /perception/capture_done_json; the V5 design document
    // PERCEPTION_DESIGN it was specified in is not part of this package): handles one
    // "capture" command synchronously on the caller's thread (the ROS executor thread
    // svc_ros, NOT the Objects process loop). Picks the buffered frame nearest the
    // command's image_time_hint, encodes it to JPEG under
    // <save_dir>/<event_uuid>_<ts>.jpg, LRU-prunes the evidence dir to the quota,
    // and RETURNS the capture_done JSON (incl. pruned_paths) for the node to
    // publish. Still CPU/cv::Mat (no GPU ring buffer / libjpeg-turbo) -- fine at
    // 10Hz; a standalone perception.evidence_pruned event topic is deferred.
    std::string handle_capture_command(const std::string& cmd_json);

private:
    friend struct SupervisorRnsJoinTestAccess;
#ifdef PERCEPTION_TESTING
    friend struct SupervisorTestAccess;
#endif

    void process_loop();
    void heartbeat_loop();
    // Stamps prepublish_ns on frame's copy of the timing ledger (frame was built from result),
    // re-checks the ledger's order and copies it back into result, which the R6 callback reads.
    static void finalize_prepublish_timing(
        yolo::DetectionResult& result, PerceptionFrame& frame);
    // T11 capture helpers: pick the buffered frame nearest image_time_hint (newest
    // if hint empty/unparsable); LRU-prune the evidence dir to evidence_quota_bytes_.
    cv::Mat select_capture_frame(const std::string& image_time_hint, double& out_stamp_sec);
    std::vector<std::string> enforce_evidence_quota(const std::string& save_dir) const;
    bool apply_gemini_runtime_geometry(
        const yolo::DetectionResult& result,
        PerceptionFrame& frame);
    bool validate_gemini_runtime_geometry(const yolo::DetectionResult& result) const;
    // Velocity of this frame's fused detections, on the frame's capture-monotonic stamp. Taking
    // the CaptureStamp instead of seconds keeps a wall time from being passed in by mistake.
    void update_velocity(std::vector<FusedDetection>& fused,
                         const geometry::CaptureStamp& capture, bool has_odom);
    bool gemini_runtime_healthy() const;
    using JoinLauncher = std::future<void> (*)(std::thread&);
    bool wait_for_thread_exit(std::thread& thread, const char* thread_name,
        std::chrono::milliseconds timeout, JoinLauncher launcher = nullptr);
    bool start_module(yolo::ModuleBase* module, const char* name);
    PerceptionFrame build_frame(
        const yolo::DetectionResult& result,
        const std::vector<FusedDetection>& fused,
        float sync_diff_ms,
        const std::string& output_frame,
        double publish_stamp_sec,
        double publish_lidar_stamp_sec,
        float tracker_ms,
        float fusion_core_ms,
        float depth_sample_ms,
        float odom_ms,
        float velocity_ms,
        float publish_ms) const;
    static double steady_now_sec();
    static double system_now_sec();
    static bool parallel_depth_sample_enabled(const std::string& camera_driver);
    // PERCEPTION_PARALLEL_DEPTH_SAMPLE "2": the shared split (2026-10-08).
    static bool parallel_depth_sample_split_shared();
    // "alt" (diagnostic, 2026-10-08): fixed and shared split on alternate frames.
    static bool parallel_depth_sample_split_alternate();

    // Unconditional fields: R6 ownership never changes ABI by build macro.
    // Declared before modules, so it is destroyed after every journal borrower.
    std::optional<rns::SupervisorRnsContext> rns_context_;
    std::unique_ptr<rns::FusedObjectsAdapter> rns_objects_;
    mutable std::mutex lifecycle_mutex_;
    bool init_attempted_ = false, rns_start_attempted_ = false, stopped_ = false;
    std::uint64_t rns_prepared_ = 0, rns_callback_attempts_ = 0, rns_callback_successes_ = 0;
    // Before the queues and modules: the result queue's discard observer and
    // the process thread write it until both are gone.
    ResultHandoffLedger result_ledger_;
    std::unique_ptr<RgbdCapture> rgbd_capture_;
    std::unique_ptr<HevcEncoder> hevc_encoder_;
    std::unique_ptr<yolo::InferenceModule> inference_;
    std::unique_ptr<ByteTracker> tracker_;
    // depth section of the config; per_class_depth_samplers_ hold depth.per_class_sample_mode
    // overrides (weak-tier tracks always use the default sampler).
    DepthSampler depth_sampler_;
    std::map<std::string, DepthSampler> per_class_depth_samplers_;
    TrackDepthHysteresis depth_hysteresis_;
    std::unique_ptr<OdomBuffer> odom_buffer_;
    // Velocity in odom axes (XY) when an odom pose exists for the frame, otherwise in camera
    // axes (XZ) by lidar_velocity_estimator_, the one production uses while TF is refused.
    TrackVelocityEstimator odom_velocity_estimator_;
    TrackVelocityEstimator lidar_velocity_estimator_;
    // Placeholder camera->base for the odom path, see init().
    RigidTransform extrinsic_camera_to_base_;

    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> frame_queue_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue_;
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::DetectionResult>> detection_queue_;

    std::thread process_thread_;
    std::thread heartbeat_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> fatal_failure_detected_{false};

    mutable std::mutex run_mutex_;
    std::condition_variable run_cv_;
    bool shutdown_requested_ = false;

    yolo::Config resolved_config_;
    const yolo::Config* config_ = nullptr;
    bool initialized_ = false;
#ifdef PERCEPTION_TESTING
    // Test-only dependency injection keeps Supervisor lifecycle tests independent
    // of whether the build host has a concrete RGB-D SDK available. Keep this
    // next to initialized_ so production-only translation units retain the same
    // offsets for every field they access.
    bool use_injected_rgbd_capture_ = false;
#endif
    bool has_last_velocity_frame_ = false;
    bool last_velocity_frame_has_odom_ = false;
    float last_publish_ms_ = 0.0F;

    // T11 capture: ring buffer of recently decoded RGB frames (shared, ref-counted
    // from the process loop). A capture command picks the frame nearest its
    // image_time_hint (wall-clock stamps on both sides); empty/absent/unparsable
    // hint -> newest frame. Capacity + evidence quota are set from config in init().
    struct FrameBufferEntry {
        cv::Mat frame;
        double stamp_sec = 0.0;
    };
    std::mutex frame_buffer_mutex_;
    std::deque<FrameBufferEntry> frame_buffer_;
    std::size_t frame_buffer_capacity_ = 20;
    std::uint64_t evidence_quota_bytes_ = 0;  // 0 = quota disabled
};

}  // namespace perception
