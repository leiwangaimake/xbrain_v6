/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: types.hpp
 * Brief: Shared data structures for detector pipeline messages.
 *
 * Description:
 *   1. Defines captured frame and detection result payloads.
 *   2. Keeps module interfaces stable across implementation phases.
 * Flow: FrameData (capture) -> DetectionResult (inference) -> Track (ByteTrack) ->
 * FusedDetection (Supervisor: depth, velocity, geometry) -> the R6 Objects body built by the
 * service adapter. Several fields and PerceptionFrame date from V5 (D435, LiDAR fusion, the
 * Zenoh JSON publisher); their notes below say what R6 still uses.
 */

#pragma once

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>
#include "common/rgbd_source_identity.hpp"

namespace perception {
class RgbDepthRegistration;
}

namespace yolo {

// Global class 92 is a map-like traversable-area output. It must never enter
// ordinary instance tracking or the ordinary detections JSON array.
inline constexpr int kTraversableAreaGlobalClassId = 92;

/**
 * @brief Detector branch that produced one observation.
 *
 * This is internal pipeline metadata. The published global class ID remains
 * the stable external contract.
 */
enum class DetectionSource {
    coco80,
    active12,
};

/**
 * @brief Pinhole intrinsics for a depth image registered to the color frame.
 */
struct DepthIntrinsics {
    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
    int width = 0;
    int height = 0;
};

/**
 * @brief SDK colour-plane calibration of one capture session (software D2C).
 *
 * The aligned depth lives on the colour raster: colour intrinsics/distortion
 * deproject it; depth_to_colour maps native depth-camera points to colour
 * points (p_c = R * p_d + t, t in metres). Values are the SDK's report for this
 * stream start, not a calibration certificate.
 */
struct RgbdCalibration {
    DepthIntrinsics colour;
    int colour_distortion_model = 0;                 // raw SDK OBCameraDistortionModel
    std::array<float, 8> colour_distortion{};        // k1 k2 k3 k4 k5 k6 p1 p2
    std::array<float, 9> depth_to_colour_R{};        // row-major
    std::array<float, 3> depth_to_colour_t_m{};
    int aligned_width = 0, aligned_height = 0;       // aligned depth raster
    int output_width = 0, output_height = 0;         // published FrameData depth raster
    // Session identity the registration gate was evaluated on.
    std::string device_name, device_serial, firmware, sdk_version;
    // RGB <- aligned registration when the gate passed (depth rasters in
    // FrameData are then on the RGB image); status explains a null pointer.
    std::shared_ptr<const perception::RgbDepthRegistration> registration;
    std::string registration_status;
};

/**
 * @brief Optional RGB sensor metadata copied from the same D435 color frame.
 *
 * Values stay diagnostic-only: unsupported metadata is represented by the
 * corresponding valid flag instead of a fabricated zero measurement.
 * R6 (Gemini 338Le) never fills it: the SDK exposure units are not verified for this model.
 */
struct CameraFrameMetadata {
    std::string source = "unavailable";
    bool exposure_valid = false;
    double exposure_us = 0.0;
    bool gain_valid = false;
    double gain = 0.0;
};

/**
 * @brief Diagnostic-only timestamps for one frame's host perception path.
 *
 * The six path endpoints use the host monotonic clock and form adjacent,
 * non-overlapping intervals from SDK frame readiness to the local ROS
 * subscriber. capture_epoch_ns is the one explicit system-clock sample used
 * only to cross-check the legacy wall-clock latency. The final subscriber
 * timestamps are added by the probe. This structure is propagated in-process
 * only and serialized solely when valid.
 * R6: filled with PERCEPTION_INFERENCE_PROFILE=1 (production) up to prepublish_ns; the
 * inference module writes and logs its own part, but nothing in the delivered service reads
 * the complete ledger (its reader was the V5 Zenoh publisher).
 */
struct PipelineTiming {
    bool valid = false;
    std::int64_t capture_epoch_ns = 0;
    std::int64_t capture_ready_ns = 0;
    std::int64_t frame_push_begin_ns = 0;
    std::int64_t inference_pop_end_ns = 0;
    std::int64_t result_push_begin_ns = 0;
    std::int64_t supervisor_pop_end_ns = 0;
    std::int64_t prepublish_ns = 0;
    double preprocess_ms = 0.0;
    double clone_ms = 0.0;
    double execute_pair_ms = 0.0;
    double post_wall_ms = 0.0;
    double merge_ms = 0.0;
};

/**
 * @brief One captured image frame plus metadata needed by downstream modules.
 */
struct FrameData {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
    double stamp_sec = 0.0;
    std::uint64_t frame_id = 0;
    double capture_time_ms = 0.0;
    cv::Mat depth;
    DepthIntrinsics depth_intrinsics;
    // Non-null: depth is registered onto the (distorted) RGB image; back-project
    // RGB pixels through its undistortion, not the pinhole intrinsics alone.
    std::shared_ptr<const perception::RgbDepthRegistration> depth_registration;
    float depth_scale = 0.0F;
    CameraFrameMetadata camera_metadata;
    PipelineTiming pipeline_timing;
    // Present only when the optional native-depth tap was enabled. Correlates
    // this aligned image with the pre-D2C sample without changing legacy IDs.
    // It is not yet a device-to-host exposure-time mapping.
    perception::native_depth::SourceId native_depth_source;
};

/**
 * @brief One YOLO detection after postprocessing and class mapping.
 */
struct Detection {
    cv::Rect bbox;
    float confidence = 0.0F;
    int class_id = -1;
    std::string class_name;
    // Bbox-local CV_8UC1 mask consumed by DepthSampler and ByteTrack.
    cv::Mat mask;
    // Internal-only marker for the generic weak-obstacle path. It is disabled
    // by default and is not serialized into the public perception payload.
    bool low_confidence_candidate = false;
    // Original-image-coordinate CV_8UC1 mask used only by the dual-model
    // fusion layer. Keeping both representations avoids changing the
    // established bbox-local DepthSampler contract.
    cv::Mat full_image_mask;
    DetectionSource source = DetectionSource::coco80;
    // Branch-local class ID before sealed active12 -> global ID mapping.
    int original_class_id = -1;
};

/**
 * @brief One audit-only overlap between a COCO vehicle and active12 heavy machinery.
 */
struct CrossModelConflictAudit {
    std::string kind = "vehicle_vs_heavy_machinery";
    std::size_t old_index = 0;
    std::size_t active_index = 0;
    int old_class_id = -1;
    int active_class_id = -1;
    float box_iou = 0.0F;
    std::string mode = "audit_only";
    bool old_suppressed = false;
};

/**
 * @brief Detection list for one frame, including basic timing metadata.
 */
// Immutable per-result observation from the JIT owner's actual publish call.
// This is not the current/global calibration state and does not control JIT.
enum class JitResultPhase : std::uint8_t {
    Unavailable, CalibrationWarmup, Calibrating, Measuring
};

inline JitResultPhase jit_result_phase_from_name(std::string_view phase) noexcept {
    if (phase == "CALIBRATION_WARMUP") return JitResultPhase::CalibrationWarmup;
    if (phase == "CALIBRATING") return JitResultPhase::Calibrating;
    if (phase == "MEASURING") return JitResultPhase::Measuring;
    return JitResultPhase::Unavailable;
}

struct DetectionResult {
    // Ordinary instances admitted to ByteTrack. Map-like class 92 is
    // deliberately separated into map_detections.
    std::vector<Detection> detections;
    std::vector<Detection> map_detections;
    std::vector<CrossModelConflictAudit> cross_model_conflicts;
    cv::Mat frame;
    cv::Mat depth;
    DepthIntrinsics depth_intrinsics;
    std::shared_ptr<const perception::RgbDepthRegistration> depth_registration;  // see FrameData
    float depth_scale = 0.0F;
    std::uint64_t frame_id = 0;
    double stamp_sec = 0.0;
    double capture_time_ms = 0.0;
    double inference_time_ms = 0.0;
    CameraFrameMetadata camera_metadata;
    PipelineTiming pipeline_timing;
    // Original pre-D2C identity, retained through both preparation paths even
    // when optional profiling is off. This is not a successful inference flag.
    perception::native_depth::SourceId native_depth_source;
    // True only after usable normal inference outputs, postprocessing and merge.
    // Source identity or an empty detections vector alone never imply success.
    bool inference_completed = false;
    // Host CLOCK_MONOTONIC time at which the inference loop saw both models' outputs ready for
    // this frame (the JIT loop's ready_seen, before postprocessing and merge). Set with tracing
    // on or off; 0 on a path without that observation. The paced Objects release starts from it.
    std::int64_t ready_observed_mono_ns = 0;
    JitResultPhase jit_phase = JitResultPhase::Unavailable;
    double jit_frozen_delay_ms = -1.0;
};

}  // namespace yolo

namespace perception {

/**
 * @brief Compact statistics for LiDAR depth samples that hit one object mask.
 * (R6: the depth camera's selected band of one object, from DepthSampler; no LiDAR.)
 */
struct DepthStats {
    float min = 0.0F;
    float median = 0.0F;
    float max = 0.0F;
    float stddev = 0.0F;
};

// Capture-clock observations supplied explicitly by the caller, never inferred
// from the legacy system-clock timestamps or from tracker coasting updates.
struct RnsObservationTimes {
    std::int64_t first_seen_mono_ms = -1;
    std::int64_t last_seen_mono_ms = -1;
};

/** @brief Stable 2D track state after ByteTrack association. */
struct Track {
    int track_id = -1;
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;
    // Semantic class publication is deliberately independent from tracker
    // admission. A visible track remains publishable while this current-frame
    // evidence state is still only a proxy.
    std::string semantic_status = "proxy";
    int semantic_evidence_class_id = -1;
    std::string semantic_evidence_class_name;
    int semantic_consecutive_hits = 0;
    int semantic_low_quality_frames = 0;
    cv::Mat mask;
    // Current detector geometry for association/mask/depth. bbox_2d is the EMA-smoothed output.
    cv::Rect raw_bbox_2d;
    cv::Rect bbox_2d;
    int first_seen_frame = 0;
    int last_seen_frame = 0;
    std::uint64_t first_seen_ms = 0;
    std::uint64_t last_seen_ms = 0;
    int stable_frames = 0;
    int consecutive_hits = 0;
    bool confirmed = false;
    // Tracks born from the generic weak-obstacle path require stronger temporal
    // confirmation and valid depth before downstream publication.
    bool low_confidence_candidate = false;
    // Tier of the detector observation applied in this update. This is reset
    // when a track is only coasting, so downstream safety gates do not confuse
    // track history with the evidence used by the current frame.
    bool current_observation_low_confidence_candidate = false;
    // Absent when either endpoint is unknown. Mixing an untimed observation
    // into a timed history invalidates this pair rather than inventing a date.
    std::optional<RnsObservationTimes> rns_observation_times;
};

/**
 * @brief One tracked object after RGB mask and LiDAR point fusion.
 * (R6: after RGB mask and registered-depth sampling; there is no LiDAR.)
 */
struct FusedDetection {
    int track_id = -1;
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;
    // Backward-compatible semantic evidence qualifier serialized as
    // "proxy" or "confirmed"; all existing geometry/depth fields remain valid.
    std::string semantic_status = "proxy";
    cv::Rect bbox_2d;
    // Display diagnostics only: current detector rectangle before publication
    // EMA/output-mode holding. Never use this field for depth, velocity,
    // footprint or navigation decisions.
    cv::Rect raw_bbox_2d;
    int first_seen_frame = 0;
    int last_seen_frame = 0;
    std::uint64_t first_seen_ms = 0;
    std::uint64_t last_seen_ms = 0;
    int stable_frames = 0;

    std::vector<cv::Point> mask_polygon_2d;
    int mask_pixel_count = 0;

    int fused_points_count = 0;
    int depth_sample_region_pixel_count = 0;
    int depth_valid_pixel_count = 0;
    int depth_required_cluster_pixel_count = 0;
    float depth_valid_fraction = 0.0F;
    float depth_selected_fraction = 0.0F;
    std::string depth_sample_source = "unavailable";
    std::string depth_selection_status = "invalid_input";
    bool depth_selection_fallback = false;
    // Raw per-frame depth evidence remains observable even when the track-level
    // hysteresis deliberately holds the previously accepted published depth.
    bool depth_raw_valid = false;
    float depth_raw_m = 0.0F;
    int depth_raw_sample_region_pixel_count = 0;
    int depth_raw_valid_pixel_count = 0;
    int depth_raw_selected_pixel_count = 0;
    int depth_raw_required_cluster_pixel_count = 0;
    float depth_raw_valid_fraction = 0.0F;
    float depth_raw_selected_fraction = 0.0F;
    float depth_raw_selected_stddev_m = 0.0F;
    std::string depth_raw_sample_source = "unavailable";
    std::string depth_raw_selection_status = "invalid_input";
    bool depth_raw_selection_fallback = false;
    bool depth_hysteresis_held = false;
    float centroid_3d[3] = {0.0F, 0.0F, 0.0F};
    float velocity_3d[3] = {0.0F, 0.0F, 0.0F};
    // Diagnostic-only validity. Numeric velocity_3d remains backward-compatible.
    bool velocity_valid = false;
    std::string velocity_status = "warming_up";
    // Frame of centroid_3d / velocity_3d when the velocity was estimated. The
    // Supervisor sets it after each estimator update; Unset means no estimator
    // has run. Only Camera velocities may be rotated into base_link as "raw".
    enum class VelocityFrame { Unset, Camera, Odom };
    VelocityFrame velocity_frame = VelocityFrame::Unset;
    // Object extents {w,h,d} in meters along camera axes, from the p5..p95 span
    // of the selected (foreground/dominant-cluster) points. Height is a LOWER
    // bound when the object's lower part sits below the Mid360 vertical FOV.
    // V5 LiDAR field: nothing in R6 writes or reads it (always zero).
    float size_3d[3] = {0.0F, 0.0F, 0.0F};
    // Degenerate footprints with vertex_count<3 pass through; downstream RNS uses the section 8.0.8 category fallback.
    // V5 field: nothing in R6 fills it (only the idle odom transform moves it), so it stays empty;
    // R6 publishes base_footprint_xy below. The "8.0.8" reference has no known source (GPT 1.4).
    std::vector<std::array<float, 2>> ground_footprint_xy;
    // R6 measured base_link geometry of THIS inference frame (19 section 4.1),
    // filled by the service's object geometry hook; invalid means unlocatable.
    bool base_geometry_valid = false;
    std::vector<std::array<float, 2>> base_footprint_xy;   // CCW convex, <= 12
    float base_z_min = 0.0F, base_z_max = 0.0F;
    // base <- colour rotation used for the geometry; rotates the raw
    // camera-frame velocity_3d into base_link axes (no ego-motion removal).
    std::array<float, 9> base_from_colour_R{};
    float depth_m = 0.0F;
    float depth_confidence = 0.0F;
    DepthStats depth_stat;
    std::string fusion_quality = "depth_unreliable";
    // Internal-only policy marker. Weak-obstacle proxy labels must not select
    // class-specific velocity smoothing/deadzones. This field is deliberately
    // omitted from the public Zenoh JSON payload.
    // (R6 publishes typed Objects; this marker is not in them either.)
    bool use_generic_velocity_policy = false;
};

/**
 * @brief Publishable perception payload for one RGB frame.
 * V5 payload (its publisher is not part of R6). The Supervisor still builds one per frame for
 * the timing ledger and a debug line; the R6 Objects body is built from DetectionResult and
 * FusedDetection by the service adapter instead. The lidar_* and frame fields are V5 leftovers.
 */
struct PerceptionFrame {
    std::uint64_t frame_id = 0;
    std::string frame = "lidar";
    double stamp_sec = 0.0;
    std::string video_source = "rgb";
    yolo::CameraFrameMetadata camera_metadata;
    double lidar_stamp_sec = 0.0;
    float lidar_sync_diff_ms = 0.0F;
    float inference_ms = 0.0F;
    float capture_ms = 0.0F;
    float depth_sample_ms = 0.0F;
    float fusion_ms = 0.0F;
    float tracker_ms = 0.0F;
    float fusion_core_ms = 0.0F;
    float odom_ms = 0.0F;
    float velocity_ms = 0.0F;
    float publish_ms = 0.0F;
    std::uint64_t frame_queue_depth = 0;
    std::uint64_t frame_queue_dropped = 0;
    std::uint64_t detection_queue_depth = 0;
    std::uint64_t detection_queue_dropped = 0;
    std::vector<FusedDetection> detections;
    // Runtime geometry for this exact RGB-D frame. Kept at the end so existing
    // positional aggregate initialization retains its field order. D435 leaves
    // valid=false and keeps its accepted config-driven publisher metadata.
    // (R6: set from the SDK intrinsics of every frame by the Supervisor's Gemini check.)
    yolo::DepthIntrinsics camera_intrinsics;
    bool camera_intrinsics_valid = false;
    yolo::PipelineTiming pipeline_timing;
};

}  // namespace perception
