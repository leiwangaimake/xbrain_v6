/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: track_velocity.hpp
 * Brief: Track velocity estimation helpers.
 *
 * Description:
 * What problem this solves. Objects carries a velocity per object with a closed status set
 * (11 section 3.1B.2: warming_up, motion_unconfirmed, moving, static, timestamp_gap; mapping in
 * 19 section 4.1). A centroid difference between two frames is mostly depth noise, so the
 * estimator works on windows: an EMA-smoothed centroid, a displacement over at least
 * velocity_window_s, a static threshold, and a direction that must persist for
 * motion_confirmation_windows windows before the motion is reported as valid.
 *
 * Boundaries. It works in whatever frame the centroids are in: camera frame on the R6 path; the
 * odom path is a pending capability (19 section 4.2). Rotation into base_link and the wire
 * mapping of the three internal gate statuses belong to rns_fused_objects.
 *
 * Pitfalls. stamp_sec is only used through differences, so any clock without jumps works, and a
 * gap outside (1 us, dt_max] restarts the window ("timestamp_gap"). All state is keyed by
 * track_id, so the tracker's identity continuity is what keeps a window alive.
 */

#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.hpp"

namespace perception {

// A sample fails the gate, and ends the track's window, when the track is younger than
// min_stable_frames (the tracker's stable_frames), its depth confidence is below
// min_depth_confidence, or its fusion quality is listed in skip_fusion_quality. The first failed
// gate names the status: track_unstable, low_depth_confidence or depth_unreliable.
struct VelocityGatingConfig {
    int min_stable_frames = 10;
    float min_depth_confidence = 0.5F;
    std::vector<std::string> skip_fusion_quality = {"depth_unreliable"};
};

// XZ decisions only: a camera-Z displacement within min(max_threshold_m, stddev_multiplier times
// the combined depth stddev of the two samples) is treated as depth noise and left out of the
// static / moving decision.
struct CameraZUncertaintyGateConfig {
    // Default-off preserves the legacy velocity decision when the runtime
    // configuration does not explicitly opt into the camera-only canary.
    bool enabled = false;
    float stddev_multiplier = 1.0F;
    float max_threshold_m = 0.06F;
};

// Axes of the static / moving decision and of the direction check: the horizontal plane of the
// frame in use, XZ in the camera frame (y points down) and XY in odom.
enum class MotionDecisionAxes {
    XYZ,
    XY,
    XZ,
};

/**
 * @brief Estimates per-track 3D velocity from centroid deltas between frames.
 */
class TrackVelocityEstimator {
public:
    explicit TrackVelocityEstimator(double dt_max_s = 0.5);

    // One frame: fills velocity_3d / velocity_valid / velocity_status of every detection and
    // replaces centroid_3d by the smoothed centroid. stamp_sec is the frame time (see Pitfalls).
    void update(std::vector<FusedDetection>& detections, double stamp_sec);
    // Forget every track, e.g. when the service switches the velocity frame (camera / odom).
    void reset();
    // Configuration. Values are clamped: EMA weights to [0.01, 1], distances and the gate
    // thresholds to >= 0, confirmation windows to >= 1, and the window to at most dt_max.
    void set_dt_max_s(double dt_max_s);
    double dt_max_s() const;
    void set_centroid_ema_alpha(float alpha);
    float centroid_ema_alpha() const;
    void set_per_class_centroid_ema_alphas(const std::map<std::string, float>& alphas);
    const std::map<std::string, float>& per_class_centroid_ema_alphas() const;
    void set_velocity_window_s(double window_s);
    double velocity_window_s() const;
    void set_motion_confirmation_windows(int windows);
    int motion_confirmation_windows() const;
    void set_static_displacement_m(float displacement_m);
    float static_displacement_m() const;
    void set_generic_static_displacement_m(float displacement_m);
    float generic_static_displacement_m() const;
    void set_per_class_static_displacements_m(const std::map<std::string, float>& displacements_m);
    const std::map<std::string, float>& per_class_static_displacements_m() const;
    void set_gating_config(const VelocityGatingConfig& gating_config);
    VelocityGatingConfig gating_config() const;
    void set_motion_decision_axes(MotionDecisionAxes axes);
    MotionDecisionAxes motion_decision_axes() const;
    void set_lateral_image_evidence_gate_enabled(bool enabled);
    bool lateral_image_evidence_gate_enabled() const;
    void set_camera_z_uncertainty_gate_config(
        const CameraZUncertaintyGateConfig& uncertainty_config);
    CameraZUncertaintyGateConfig camera_z_uncertainty_gate_config() const;

private:
    // One smoothed centroid sample of a track.
    struct PositionState {
        float centroid_3d[3] = {0.0F, 0.0F, 0.0F};
        double stamp_sec = 0.0;
        bool use_generic_velocity_policy = false;
    };

    // The open velocity window of one track: its anchor sample (smoothed and raw centroid, time,
    // depth stddev, image centre x), the last window's direction and velocity, and how many
    // windows in a row agreed in direction.
    struct TrackState {
        float anchor_centroid_3d[3] = {0.0F, 0.0F, 0.0F};
        float anchor_raw_centroid_3d[3] = {0.0F, 0.0F, 0.0F};
        float last_direction_displacement_3d[3] = {0.0F, 0.0F, 0.0F};
        float velocity_3d[3] = {0.0F, 0.0F, 0.0F};
        double stamp_sec = 0.0;
        double anchor_stamp_sec = 0.0;
        float anchor_depth_stddev_m = 0.0F;
        bool anchor_depth_stddev_valid = false;
        float anchor_image_center_x_px = 0.0F;
        bool anchor_image_center_x_valid = false;
        int consistent_motion_windows = 0;
        bool velocity_valid = false;
        std::string velocity_status = "warming_up";
    };

    std::unordered_map<int, PositionState> position_tracks_;          // published centroid smoothing
    std::unordered_map<int, PositionState> support_position_tracks_;  // velocity support: accepted samples only
    std::unordered_map<int, TrackState> previous_tracks_;
    double dt_max_s_ = 0.5;
    float centroid_ema_alpha_ = 0.35F;
    std::map<std::string, float> per_class_centroid_ema_alphas_;
    double velocity_window_s_ = 0.25;
    int motion_confirmation_windows_ = 1;
    float static_displacement_m_ = 0.04F;
    std::optional<float> generic_static_displacement_m_;
    std::map<std::string, float> per_class_static_displacements_m_;
    VelocityGatingConfig gating_config_;
    MotionDecisionAxes motion_decision_axes_ = MotionDecisionAxes::XYZ;
    bool lateral_image_evidence_gate_enabled_ = false;
    CameraZUncertaintyGateConfig camera_z_uncertainty_gate_config_;
};

}  // namespace perception
