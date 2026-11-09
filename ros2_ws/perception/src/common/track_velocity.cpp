/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: track_velocity.cpp
 * Brief: Track velocity estimator implementation.
 *
 * Description:
 *   1. Computes centroid delta divided by frame timestamp delta.
 *   2. Enforces RNS static threshold and speed cap.
 *
 * How a velocity is produced (camera frame, per track):
 *   - the centroid is EMA-smoothed; a velocity "window" starts at an anchor sample and is
 *     evaluated once velocity_window_s has passed;
 *   - displacement below the (per class) static threshold -> "static" (zero, valid);
 *   - otherwise the window velocity (displacement / time, capped at 3 m/s) is reported as
 *     "motion_unconfirmed" (valid=false) until enough consecutive windows agree in direction,
 *     then "moving" (valid);
 *   - a gated sample (unstable track, low depth confidence, unreliable depth) gives no
 *     estimate and ends the window (reply B4-1); a sample gap longer than dt_max restarts it
 *     ("timestamp_gap"); a new track starts "warming_up".
 * The five non-gate statuses are the closed set of 11 section 3.1B.2; the three gate names are
 * internal, and rns_fused_objects maps them to warming_up on the wire.
 */

#include "common/track_velocity.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace perception {

namespace {

// Speed cap of the window estimate, and the smallest usable time step.
constexpr float MAX_SPEED_MPS = 3.0F;
constexpr double MIN_DT_SEC = 1e-6;
constexpr double DEFAULT_DT_MAX_SEC = 0.5;
constexpr double DEFAULT_VELOCITY_WINDOW_SEC = 0.25;
constexpr float MIN_LATERAL_IMAGE_DISPLACEMENT_PX = 2.0F;
constexpr float DEFAULT_CAMERA_Z_STDDEV_MULTIPLIER = 1.0F;
constexpr float DEFAULT_CAMERA_Z_MAX_THRESHOLD_M = 0.06F;

void set_zero(float velocity[3]) {
    velocity[0] = 0.0F;
    velocity[1] = 0.0F;
    velocity[2] = 0.0F;
}

void copy3(const float source[3], float destination[3]) {
    for (int axis = 0; axis < 3; ++axis) {
        destination[axis] = source[axis];
    }
}

float norm3(const float value[3]) {
    return std::sqrt((value[0] * value[0]) + (value[1] * value[1]) + (value[2] * value[2]));
}

float dot3(const float lhs[3], const float rhs[3]) {
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2]);
}

float motion_norm(const float value[3], MotionDecisionAxes axes) {
    switch (axes) {
        case MotionDecisionAxes::XY:
            return std::sqrt((value[0] * value[0]) + (value[1] * value[1]));
        case MotionDecisionAxes::XZ:
            return std::sqrt((value[0] * value[0]) + (value[2] * value[2]));
        case MotionDecisionAxes::XYZ:
            return norm3(value);
    }
    return norm3(value);
}

float motion_dot(const float lhs[3], const float rhs[3], MotionDecisionAxes axes) {
    switch (axes) {
        case MotionDecisionAxes::XY:
            return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]);
        case MotionDecisionAxes::XZ:
            return (lhs[0] * rhs[0]) + (lhs[2] * rhs[2]);
        case MotionDecisionAxes::XYZ:
            return dot3(lhs, rhs);
    }
    return dot3(lhs, rhs);
}

// Horizontal image centre of the object: the mask polygon's x extent when there is one, else
// the box centre. Feeds the lateral image evidence gate.
bool image_center_x(const FusedDetection& det, float& center_x_px) {
    if (det.mask_polygon_2d.size() >= 3U) {
        int min_x = det.mask_polygon_2d.front().x;
        int max_x = min_x;
        for (const auto& point : det.mask_polygon_2d) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
        }
        center_x_px =
            0.5F * (static_cast<float>(min_x) + static_cast<float>(max_x));
        return true;
    }
    if (det.bbox_2d.width > 0 && det.bbox_2d.height > 0) {
        center_x_px =
            static_cast<float>(det.bbox_2d.x) +
            0.5F * static_cast<float>(det.bbox_2d.width);
        return true;
    }
    return false;
}

bool contains_quality(const std::vector<std::string>& values, const std::string& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

// The first failed gate names the status (order: stability, confidence, quality).
const char* gating_failure_status(
    const FusedDetection& det,
    const VelocityGatingConfig& config) {
    if (det.stable_frames < config.min_stable_frames) {
        return "track_unstable";
    }
    if (det.depth_confidence < config.min_depth_confidence) {
        return "low_depth_confidence";
    }
    if (contains_quality(config.skip_fusion_quality, det.fusion_quality)) {
        return "depth_unreliable";
    }
    return nullptr;
}

}  // namespace

TrackVelocityEstimator::TrackVelocityEstimator(double dt_max_s) {
    set_dt_max_s(dt_max_s);
}

void TrackVelocityEstimator::update(std::vector<FusedDetection>& detections, double stamp_sec) {
    // Forget every track state older than dt_max: its next sample starts anew.
    const double cutoff_stamp_sec = stamp_sec - dt_max_s_;
    for (auto iter = previous_tracks_.begin(); iter != previous_tracks_.end();) {
        if (iter->second.stamp_sec < cutoff_stamp_sec) {
            iter = previous_tracks_.erase(iter);
        } else {
            ++iter;
        }
    }
    for (auto iter = position_tracks_.begin(); iter != position_tracks_.end();) {
        if (iter->second.stamp_sec < cutoff_stamp_sec) {
            iter = position_tracks_.erase(iter);
        } else {
            ++iter;
        }
    }
    for (auto iter = support_position_tracks_.begin(); iter != support_position_tracks_.end();) {
        if (iter->second.stamp_sec < cutoff_stamp_sec) {
            iter = support_position_tracks_.erase(iter);
        } else {
            ++iter;
        }
    }

    // Every detection starts as "no estimate" and is upgraded below.
    for (auto& det : detections) {
        set_zero(det.velocity_3d);
        det.velocity_valid = false;
        det.velocity_status = "warming_up";
        if (det.depth_hysteresis_held) {
            // The published centroid still belongs to the previously accepted
            // depth mode while raw depth is proposing another mode. Do not let
            // either side become a velocity anchor: otherwise the eventual
            // handover can be interpreted as physical motion for the same ID.
            // "warming_up" is intentionally reused instead of adding a new
            // public velocity_status value.
            position_tracks_.erase(det.track_id);
            support_position_tracks_.erase(det.track_id);
            previous_tracks_.erase(det.track_id);
            continue;
        }
        float image_center_x_px = 0.0F;
        const bool image_center_x_valid = image_center_x(det, image_center_x_px);
        const float raw_centroid[3] = {
            det.centroid_3d[0], det.centroid_3d[1], det.centroid_3d[2]};
        const float depth_stddev_m = det.depth_stat.stddev;
        const bool depth_stddev_valid =
            std::isfinite(depth_stddev_m) && depth_stddev_m > 0.0F;
        float smoothed_centroid[3] = {
            raw_centroid[0], raw_centroid[1], raw_centroid[2]};
        auto position_previous = position_tracks_.find(det.track_id);
        if (position_previous != position_tracks_.end() &&
            position_previous->second.use_generic_velocity_policy !=
                det.use_generic_velocity_policy) {
            // A weak proxy promoted to a normal semantic track keeps its ID,
            // but must not inherit centroid smoothing, cached velocity, or
            // motion confirmation from the previous policy.
            position_tracks_.erase(position_previous);
            support_position_tracks_.erase(det.track_id);
            previous_tracks_.erase(det.track_id);
            position_previous = position_tracks_.end();
        }
        const auto class_alpha = det.use_generic_velocity_policy
                                     ? per_class_centroid_ema_alphas_.end()
                                     : per_class_centroid_ema_alphas_.find(det.class_name);
        const float centroid_ema_alpha =
            class_alpha == per_class_centroid_ema_alphas_.end()
                ? centroid_ema_alpha_
                : class_alpha->second;
        const auto smooth = [&](const std::unordered_map<int, PositionState>::const_iterator& previous_position,
                                const std::unordered_map<int, PositionState>& states,
                                float out[3]) {
            copy3(raw_centroid, out);
            if (previous_position == states.end()) return;
            const double position_dt = stamp_sec - previous_position->second.stamp_sec;
            if (position_dt > MIN_DT_SEC && position_dt <= dt_max_s_) {
                for (int axis = 0; axis < 3; ++axis) {
                    out[axis] =
                        centroid_ema_alpha * raw_centroid[axis] +
                        (1.0F - centroid_ema_alpha) *
                            previous_position->second.centroid_3d[axis];
                }
            }
        };
        // The published centroid is the smoothed one.
        smooth(position_previous, position_tracks_, smoothed_centroid);
        PositionState position_current;
        copy3(smoothed_centroid, position_current.centroid_3d);
        position_current.stamp_sec = stamp_sec;
        position_current.use_generic_velocity_policy = det.use_generic_velocity_policy;
        position_tracks_[det.track_id] = position_current;
        copy3(smoothed_centroid, det.centroid_3d);

        if (const char* failure = gating_failure_status(det, gating_config_)) {
            // Batch-4 reply B4-1: a gated sample gives no estimate AND ends the
            // velocity window: anchors, direction confirmation, cached velocity
            // and the velocity-support centroid smoothing are dropped, so the
            // next accepted sample starts a fresh window (warming_up) built only
            // from accepted samples. Track identity and the published centroid
            // smoothing (position_tracks_) are unchanged.
            det.velocity_status = failure;
            previous_tracks_.erase(det.track_id);
            support_position_tracks_.erase(det.track_id);
            continue;
        }
        // Velocity-support centroid: the same EMA over accepted samples only.
        // Without any gated sample in the track history it equals the
        // published smoothed centroid bit for bit.
        float support_centroid[3];
        smooth(support_position_tracks_.find(det.track_id), support_position_tracks_, support_centroid);
        {
            PositionState support_current;
            copy3(support_centroid, support_current.centroid_3d);
            support_current.stamp_sec = stamp_sec;
            support_current.use_generic_velocity_policy = det.use_generic_velocity_policy;
            support_position_tracks_[det.track_id] = support_current;
        }

        // Continue the track's window, restart it after a gap, or start it.
        const auto previous = previous_tracks_.find(det.track_id);
        TrackState current;
        if (previous != previous_tracks_.end()) {
            current = previous->second;
            det.velocity_valid = current.velocity_valid;
            det.velocity_status = current.velocity_status;
            const double sample_dt = stamp_sec - previous->second.stamp_sec;
            if (sample_dt > MIN_DT_SEC && sample_dt <= dt_max_s_) {
                copy3(previous->second.velocity_3d, det.velocity_3d);
                // Until the window is full, the last window's result is repeated.
                const double velocity_dt = stamp_sec - previous->second.anchor_stamp_sec;
                if (velocity_dt >= velocity_window_s_) {
                    float static_displacement_m = generic_static_displacement_m();
                    if (!det.use_generic_velocity_policy) {
                        const auto class_displacement =
                            per_class_static_displacements_m_.find(det.class_name);
                        static_displacement_m =
                            class_displacement == per_class_static_displacements_m_.end()
                                ? static_displacement_m_
                                : class_displacement->second;
                    }
                    const float displacement[3] = {
                        support_centroid[0] - previous->second.anchor_centroid_3d[0],
                        support_centroid[1] - previous->second.anchor_centroid_3d[1],
                        support_centroid[2] - previous->second.anchor_centroid_3d[2]};
                    const float raw_displacement[3] = {
                        raw_centroid[0] - previous->second.anchor_raw_centroid_3d[0],
                        raw_centroid[1] - previous->second.anchor_raw_centroid_3d[1],
                        raw_centroid[2] - previous->second.anchor_raw_centroid_3d[2]};
                    float motion_displacement[3] = {
                        displacement[0], displacement[1], displacement[2]};
                    // Lateral image evidence (XZ decisions): a camera-X displacement counts only
                    // when the image centre moved at least MIN_LATERAL_IMAGE_DISPLACEMENT_PX the
                    // same way. A real sideways move of a few centimetres shifts the image by
                    // several pixels at these ranges; an X change with a still image is centroid
                    // noise from a different point selection, not motion.
                    if (lateral_image_evidence_gate_enabled_ &&
                        motion_decision_axes_ == MotionDecisionAxes::XZ &&
                        image_center_x_valid &&
                        previous->second.anchor_image_center_x_valid) {
                        const float image_displacement_x_px =
                            image_center_x_px -
                            previous->second.anchor_image_center_x_px;
                        const bool lateral_image_support =
                            std::fabs(image_displacement_x_px) >=
                                MIN_LATERAL_IMAGE_DISPLACEMENT_PX &&
                            displacement[0] * image_displacement_x_px > 0.0F;
                        if (!lateral_image_support) {
                            motion_displacement[0] = 0.0F;
                        }
                    }
                    float direction_displacement[3] = {
                        motion_displacement[0],
                        motion_displacement[1],
                        motion_displacement[2]};
                    const bool camera_z_uncertainty_available =
                        camera_z_uncertainty_gate_config_.enabled &&
                        motion_decision_axes_ == MotionDecisionAxes::XZ &&
                        depth_stddev_valid &&
                        previous->second.anchor_depth_stddev_valid;
                    if (camera_z_uncertainty_available) {
                        const float combined_stddev_m = std::hypot(
                            depth_stddev_m,
                            previous->second.anchor_depth_stddev_m);
                        const float z_uncertainty_threshold_m = std::min(
                            camera_z_uncertainty_gate_config_.max_threshold_m,
                            camera_z_uncertainty_gate_config_.stddev_multiplier *
                                combined_stddev_m);
                        if (std::fabs(displacement[2]) <= z_uncertainty_threshold_m) {
                            // This only removes unsupported Z from the camera-XZ
                            // motion decision. If image-supported X independently
                            // confirms motion, the published full 3D velocity stays
                            // backward-compatible, including its numeric Z value.
                            motion_displacement[2] = 0.0F;
                            direction_displacement[2] = 0.0F;
                        } else {
                            // EMA samples within adjacent windows are correlated.
                            // Use the raw Z window direction once uncertainty is
                            // available, so a raw reversal cannot be confirmed only
                            // because the EMA is still drifting the old way.
                            direction_displacement[2] = raw_displacement[2];
                        }
                    }
                    // Decide static or moving on the configured axes.
                    if (motion_norm(motion_displacement, motion_decision_axes_) <
                        static_displacement_m) {
                        set_zero(det.velocity_3d);
                        set_zero(current.last_direction_displacement_3d);
                        current.consistent_motion_windows = 0;
                        det.velocity_valid = true;
                        det.velocity_status = "static";
                    } else {
                        const bool direction_consistent =
                            current.consistent_motion_windows > 0 &&
                            motion_dot(
                                direction_displacement,
                                current.last_direction_displacement_3d,
                                motion_decision_axes_) > 0.0F;
                        current.consistent_motion_windows =
                            direction_consistent ? current.consistent_motion_windows + 1 : 1;
                        copy3(
                            direction_displacement,
                            current.last_direction_displacement_3d);
                        // The window estimate is the same for an unconfirmed and a
                        // confirmed window (batch-4 reply B4-1): EMA displacement
                        // over the window, capped. Confirmation only decides the
                        // status and validity; an unconfirmed candidate is never
                        // replaced by an invented zero.
                        for (int axis = 0; axis < 3; ++axis) {
                            det.velocity_3d[axis] =
                                displacement[axis] / static_cast<float>(velocity_dt);
                        }
                        const float speed = norm3(det.velocity_3d);
                        if (speed > MAX_SPEED_MPS) {
                            const float scale = MAX_SPEED_MPS / speed;
                            for (int axis = 0; axis < 3; ++axis) {
                                det.velocity_3d[axis] *= scale;
                            }
                        }
                        if (current.consistent_motion_windows < motion_confirmation_windows_) {
                            det.velocity_valid = false;
                            det.velocity_status = "motion_unconfirmed";
                        } else {
                            det.velocity_valid = true;
                            det.velocity_status = "moving";
                        }
                    }
                    // This sample anchors the next window.
                    copy3(support_centroid, current.anchor_centroid_3d);
                    copy3(raw_centroid, current.anchor_raw_centroid_3d);
                    current.anchor_stamp_sec = stamp_sec;
                    current.anchor_depth_stddev_m = depth_stddev_m;
                    current.anchor_depth_stddev_valid = depth_stddev_valid;
                    current.anchor_image_center_x_px = image_center_x_px;
                    current.anchor_image_center_x_valid = image_center_x_valid;
                    copy3(det.velocity_3d, current.velocity_3d);
                    current.velocity_valid = det.velocity_valid;
                    current.velocity_status = det.velocity_status;
                }
            } else {
                // Sample gap outside (MIN_DT, dt_max]: restart the window here.
                set_zero(current.velocity_3d);
                set_zero(current.last_direction_displacement_3d);
                current.consistent_motion_windows = 0;
                copy3(support_centroid, current.anchor_centroid_3d);
                copy3(raw_centroid, current.anchor_raw_centroid_3d);
                current.anchor_stamp_sec = stamp_sec;
                current.anchor_depth_stddev_m = depth_stddev_m;
                current.anchor_depth_stddev_valid = depth_stddev_valid;
                current.anchor_image_center_x_px = image_center_x_px;
                current.anchor_image_center_x_valid = image_center_x_valid;
                current.velocity_valid = false;
                current.velocity_status = "timestamp_gap";
                det.velocity_valid = false;
                det.velocity_status = "timestamp_gap";
            }
        } else {
            // First accepted sample of this track (or after a gated one).
            copy3(support_centroid, current.anchor_centroid_3d);
            copy3(raw_centroid, current.anchor_raw_centroid_3d);
            current.anchor_stamp_sec = stamp_sec;
            current.anchor_depth_stddev_m = depth_stddev_m;
            current.anchor_depth_stddev_valid = depth_stddev_valid;
            current.anchor_image_center_x_px = image_center_x_px;
            current.anchor_image_center_x_valid = image_center_x_valid;
            current.velocity_valid = false;
            current.velocity_status = "warming_up";
        }

        current.stamp_sec = stamp_sec;
        previous_tracks_[det.track_id] = current;
    }
}

void TrackVelocityEstimator::reset() {
    position_tracks_.clear();
    support_position_tracks_.clear();
    previous_tracks_.clear();
}

void TrackVelocityEstimator::set_dt_max_s(double dt_max_s) {
    dt_max_s_ = dt_max_s > MIN_DT_SEC ? dt_max_s : DEFAULT_DT_MAX_SEC;
    velocity_window_s_ = std::min(velocity_window_s_, dt_max_s_);
}

double TrackVelocityEstimator::dt_max_s() const {
    return dt_max_s_;
}

void TrackVelocityEstimator::set_centroid_ema_alpha(float alpha) {
    centroid_ema_alpha_ = std::clamp(alpha, 0.01F, 1.0F);
}

float TrackVelocityEstimator::centroid_ema_alpha() const {
    return centroid_ema_alpha_;
}

void TrackVelocityEstimator::set_per_class_centroid_ema_alphas(
    const std::map<std::string, float>& alphas) {
    per_class_centroid_ema_alphas_.clear();
    for (const auto& [class_name, alpha] : alphas) {
        if (!class_name.empty()) {
            per_class_centroid_ema_alphas_[class_name] = std::clamp(alpha, 0.01F, 1.0F);
        }
    }
}

const std::map<std::string, float>& TrackVelocityEstimator::per_class_centroid_ema_alphas() const {
    return per_class_centroid_ema_alphas_;
}

void TrackVelocityEstimator::set_velocity_window_s(double window_s) {
    const double resolved = window_s > MIN_DT_SEC ? window_s : DEFAULT_VELOCITY_WINDOW_SEC;
    velocity_window_s_ = std::min(resolved, dt_max_s_);
}

double TrackVelocityEstimator::velocity_window_s() const {
    return velocity_window_s_;
}

void TrackVelocityEstimator::set_motion_confirmation_windows(int windows) {
    motion_confirmation_windows_ = std::max(1, windows);
}

int TrackVelocityEstimator::motion_confirmation_windows() const {
    return motion_confirmation_windows_;
}

void TrackVelocityEstimator::set_static_displacement_m(float displacement_m) {
    static_displacement_m_ = std::max(0.0F, displacement_m);
}

float TrackVelocityEstimator::static_displacement_m() const {
    return static_displacement_m_;
}

void TrackVelocityEstimator::set_generic_static_displacement_m(float displacement_m) {
    generic_static_displacement_m_ = std::max(0.0F, displacement_m);
}

float TrackVelocityEstimator::generic_static_displacement_m() const {
    return generic_static_displacement_m_.value_or(static_displacement_m_);
}

void TrackVelocityEstimator::set_per_class_static_displacements_m(
    const std::map<std::string, float>& displacements_m) {
    per_class_static_displacements_m_.clear();
    for (const auto& [class_name, displacement_m] : displacements_m) {
        if (!class_name.empty()) {
            per_class_static_displacements_m_[class_name] = std::max(0.0F, displacement_m);
        }
    }
}

const std::map<std::string, float>& TrackVelocityEstimator::per_class_static_displacements_m() const {
    return per_class_static_displacements_m_;
}

void TrackVelocityEstimator::set_gating_config(const VelocityGatingConfig& gating_config) {
    gating_config_ = gating_config;
    if (gating_config_.min_stable_frames < 0) {
        gating_config_.min_stable_frames = 0;
    }
    if (gating_config_.min_depth_confidence < 0.0F) {
        gating_config_.min_depth_confidence = 0.0F;
    }
}

VelocityGatingConfig TrackVelocityEstimator::gating_config() const {
    return gating_config_;
}

void TrackVelocityEstimator::set_motion_decision_axes(MotionDecisionAxes axes) {
    motion_decision_axes_ = axes;
}

MotionDecisionAxes TrackVelocityEstimator::motion_decision_axes() const {
    return motion_decision_axes_;
}

void TrackVelocityEstimator::set_lateral_image_evidence_gate_enabled(bool enabled) {
    lateral_image_evidence_gate_enabled_ = enabled;
}

bool TrackVelocityEstimator::lateral_image_evidence_gate_enabled() const {
    return lateral_image_evidence_gate_enabled_;
}

void TrackVelocityEstimator::set_camera_z_uncertainty_gate_config(
    const CameraZUncertaintyGateConfig& uncertainty_config) {
    camera_z_uncertainty_gate_config_ = uncertainty_config;
    if (!std::isfinite(camera_z_uncertainty_gate_config_.stddev_multiplier) ||
        camera_z_uncertainty_gate_config_.stddev_multiplier < 0.0F) {
        camera_z_uncertainty_gate_config_.stddev_multiplier =
            DEFAULT_CAMERA_Z_STDDEV_MULTIPLIER;
    }
    if (!std::isfinite(camera_z_uncertainty_gate_config_.max_threshold_m) ||
        camera_z_uncertainty_gate_config_.max_threshold_m < 0.0F) {
        camera_z_uncertainty_gate_config_.max_threshold_m =
            DEFAULT_CAMERA_Z_MAX_THRESHOLD_M;
    }
}

CameraZUncertaintyGateConfig TrackVelocityEstimator::camera_z_uncertainty_gate_config() const {
    return camera_z_uncertainty_gate_config_;
}

}  // namespace perception
