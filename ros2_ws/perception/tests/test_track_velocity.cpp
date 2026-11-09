/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_track_velocity.cpp
 * Brief: Unit tests for track velocity estimation.
 *
 * Description:
 *   1. Verifies cross-frame centroid differencing.
 *   2. Verifies static threshold and 3 m/s speed cap.
 */

#include "common/track_velocity.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

perception::FusedDetection make_detection(int track_id, float x, float y, float z) {
    perception::FusedDetection det;
    det.track_id = track_id;
    det.centroid_3d[0] = x;
    det.centroid_3d[1] = y;
    det.centroid_3d[2] = z;
    det.stable_frames = 10;
    det.depth_confidence = 0.8F;
    det.fusion_quality = "good";
    return det;
}

void set_bbox_center_x(perception::FusedDetection& det, int center_x_px) {
    det.bbox_2d.x = center_x_px - 10;
    det.bbox_2d.y = 20;
    det.bbox_2d.width = 20;
    det.bbox_2d.height = 20;
}

void set_mask_center_x(perception::FusedDetection& det, int center_x_px) {
    det.mask_polygon_2d = {
        cv::Point{center_x_px - 5, 20},
        cv::Point{center_x_px + 5, 20},
        cv::Point{center_x_px + 5, 40},
        cv::Point{center_x_px - 5, 40}};
}

float speed_of(const perception::FusedDetection& det) {
    return std::sqrt(
        (det.velocity_3d[0] * det.velocity_3d[0]) +
        (det.velocity_3d[1] * det.velocity_3d[1]) +
        (det.velocity_3d[2] * det.velocity_3d[2]));
}

void set_depth_stddev(perception::FusedDetection& det, float stddev_m) {
    det.depth_stat.stddev = stddev_m;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// The first frame is warming_up with zero, invalid velocity; the next gives the exact x, y, z
// velocity.
void test_known_velocity() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    std::vector<perception::FusedDetection> first = {make_detection(1, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 10.0);
    require(speed_of(first.front()) == 0.0F, "first frame velocity was not zero");
    require(!first.front().velocity_valid, "first frame velocity was marked valid");
    require(first.front().velocity_status == "warming_up", "first frame status mismatch");

    std::vector<perception::FusedDetection> second = {make_detection(1, 1.0F, 0.0F, 0.0F)};
    estimator.update(second, 11.0);
    require(std::fabs(second.front().velocity_3d[0] - 1.0F) < 1e-6F, "x velocity mismatch");
    require(std::fabs(second.front().velocity_3d[1]) < 1e-6F, "y velocity mismatch");
    require(std::fabs(second.front().velocity_3d[2]) < 1e-6F, "z velocity mismatch");
    require(second.front().velocity_valid, "known velocity was not marked valid");
    require(second.front().velocity_status == "moving", "known velocity status mismatch");
}

// A displacement under the static threshold gives a valid zero velocity (static); speeds are
// capped.
void test_static_threshold_and_cap() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_static_displacement_m(0.06F);
    std::vector<perception::FusedDetection> first = {make_detection(2, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 20.0);

    std::vector<perception::FusedDetection> static_frame = {make_detection(2, 0.05F, 0.0F, 0.0F)};
    estimator.update(static_frame, 21.0);
    require(speed_of(static_frame.front()) == 0.0F, "static threshold did not zero velocity");
    require(static_frame.front().velocity_valid, "static estimate was not marked valid");
    require(static_frame.front().velocity_status == "static", "static status mismatch");

    std::vector<perception::FusedDetection> capped_frame = {make_detection(2, 10.0F, 0.0F, 0.0F)};
    estimator.update(capped_frame, 22.0);
    require(std::fabs(speed_of(capped_frame.front()) - 3.0F) < 1e-5F, "velocity cap mismatch");
}

// A gap beyond dt_max restarts the estimate (zero, invalid) and later frames resume a correct
// velocity.
void test_dt_max_warm_start() {
    perception::TrackVelocityEstimator estimator(0.5);
    estimator.set_centroid_ema_alpha(1.0F);
    std::vector<perception::FusedDetection> first = {make_detection(3, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 30.0);

    std::vector<perception::FusedDetection> expired = {make_detection(3, 1.0F, 0.0F, 0.0F)};
    estimator.update(expired, 30.6);
    require(speed_of(expired.front()) == 0.0F, "expired dt did not zero velocity");
    require(!expired.front().velocity_valid, "expired dt velocity was marked valid");

    std::vector<perception::FusedDetection> resumed = {make_detection(3, 1.5F, 0.0F, 0.0F)};
    estimator.update(resumed, 31.0);
    require(std::fabs(resumed.front().velocity_3d[0] - 1.25F) < 1e-6F, "resumed velocity mismatch");
}

// A19-NULL-2 (19 section 14; rule in 19 section 4.1 and 11 section 3.1B.2), estimator side: the
// cases without an estimate are reported as such, so that the adapter publishes velocity_xy null.
// A new track: its first sample and the next sample inside a window that is not full report
// warming_up, not valid, zero velocity. A sample whose stamp does not advance: the same stamp
// twice and then, once the track is valid again, an earlier stamp. Both times the track was
// reporting a valid velocity, so a forged "zero and valid" or a repeated old value would be read
// downstream as a measurement. Checked for both: the sample reports timestamp_gap, not valid,
// zero velocity; the next sample inside the window is still not valid and zero; the first full
// window after the restart is measured from the restart sample alone. (Stamps use binary-exact
// 0.125 s steps so window boundaries are exact.)
void test_warm_up_and_timestamp_gap_give_no_estimate() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(1);
    const auto step = [&](float x, double stamp_sec) {
        std::vector<perception::FusedDetection> frame = {make_detection(99, x, 0.0F, 2.0F)};
        estimator.update(frame, stamp_sec);
        return frame.front();
    };
    // A new track: no estimate on its first sample, and none 0.125 s later while the 0.25 s
    // window is not full. A cached "static and valid" here would be published as [0, 0] with
    // velocity_valid true.
    const auto first = step(0.0F, 600.0);
    require(first.velocity_status == "warming_up" && !first.velocity_valid && speed_of(first) == 0.0F,
            "A19-NULL-2: first sample of a new track was not warming_up, not valid, zero");
    const auto partial = step(0.06F, 600.125);
    require(partial.velocity_status == "warming_up" && !partial.velocity_valid && speed_of(partial) == 0.0F,
            "A19-NULL-2: new track reported an estimate before a full window");
    const auto moving = step(0.12F, 600.25);
    require(moving.velocity_status == "moving" && moving.velocity_valid &&
                std::fabs(moving.velocity_3d[0] - 0.48F) < 1e-5F,
            "A19-NULL-2 setup: track was not moving before the repeated stamp");

    // The same stamp twice.
    const auto repeated = step(0.24F, 600.25);
    require(repeated.velocity_status == "timestamp_gap", "A19-NULL-2: repeated stamp status is not timestamp_gap");
    require(!repeated.velocity_valid, "A19-NULL-2: repeated stamp velocity was marked valid");
    require(speed_of(repeated) == 0.0F, "A19-NULL-2: repeated stamp velocity was not zero");
    // 0.125 s after the restart the 0.25 s window is not full: still no estimate, and the status
    // stays inside the no-estimate pair that is published as velocity_xy null.
    const auto held = step(0.27F, 600.375);
    require(!held.velocity_valid,
            "A19-NULL-2: sample after the repeated stamp was marked valid before a full window");
    require(speed_of(held) == 0.0F, "A19-NULL-2: sample after the repeated stamp reused the old velocity");
    require(held.velocity_status == "timestamp_gap" || held.velocity_status == "warming_up",
            "A19-NULL-2: sample after the repeated stamp left the no-estimate statuses");
    // The restart sample (x 0.24 at 600.25) is the only anchor: 0.06 m in 0.25 s.
    const auto rebuilt = step(0.30F, 600.5);
    require(rebuilt.velocity_status == "moving" && rebuilt.velocity_valid &&
                std::fabs(rebuilt.velocity_3d[0] - 0.24F) < 1e-5F,
            "A19-NULL-2: first window after the repeated stamp was not measured from the restart sample");

    // An earlier stamp, on the track that is valid again.
    const auto earlier = step(0.42F, 600.375);
    require(earlier.velocity_status == "timestamp_gap", "A19-NULL-2: earlier stamp status is not timestamp_gap");
    require(!earlier.velocity_valid, "A19-NULL-2: earlier stamp velocity was marked valid");
    require(speed_of(earlier) == 0.0F, "A19-NULL-2: earlier stamp velocity was not zero");
    const auto held_again = step(0.44F, 600.5);
    require(!held_again.velocity_valid,
            "A19-NULL-2: sample after the earlier stamp was marked valid before a full window");
    require(speed_of(held_again) == 0.0F, "A19-NULL-2: sample after the earlier stamp reused the old velocity");
    require(held_again.velocity_status == "timestamp_gap" || held_again.velocity_status == "warming_up",
            "A19-NULL-2: sample after the earlier stamp left the no-estimate statuses");
    // The restart sample (x 0.42 at 600.375) is the only anchor: 0.08 m in 0.25 s.
    const auto rebuilt_again = step(0.50F, 600.625);
    require(rebuilt_again.velocity_status == "moving" && rebuilt_again.velocity_valid &&
                std::fabs(rebuilt_again.velocity_3d[0] - 0.32F) < 1e-5F,
            "A19-NULL-2: first window after the earlier stamp was not measured from the restart sample");
    std::cout << "A19-NULL-2 estimator: new track, repeated or earlier stamp -> warming_up / timestamp_gap, "
                 "not valid, zero velocity PASS\n";
}

// A19-NULL-2, the confirmation half: a restart on a stamp that does not advance also restarts
// motion confirmation. With two confirmation windows, a track that was confirmed moving before
// the repeated stamp reports "motion_unconfirmed" (a numeric candidate, not valid) for its first
// full window after it and "moving" only from the second window on, even though it goes on in the
// same direction. An estimator that carries the confirmation over the restart reports "moving",
// valid, one window early: 0.25 s of velocity_valid on evidence from before the gap.
// test_warm_up_and_timestamp_gap_give_no_estimate() cannot see this (one confirmation window).
// The restart clears two things, the count of agreeing windows and the last direction, and a
// window agrees only with both, so the failure needs BOTH left uncleared; leaving out one of the
// two alone changes no output (checked on 2026-10-14 with the three variants).
void test_timestamp_gap_restarts_motion_confirmation() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(2);
    const auto step = [&](float x, double stamp_sec) {
        std::vector<perception::FusedDetection> frame = {make_detection(98, x, 0.0F, 2.0F)};
        estimator.update(frame, stamp_sec);
        return frame.front();
    };
    step(0.0F, 700.0);
    const auto unconfirmed = step(0.12F, 700.25);
    require(unconfirmed.velocity_status == "motion_unconfirmed" && !unconfirmed.velocity_valid &&
                std::fabs(unconfirmed.velocity_3d[0] - 0.48F) < 1e-5F,
            "A19-NULL-2 setup: first window was not an unconfirmed candidate of 0.48 m/s");
    const auto confirmed = step(0.24F, 700.5);
    require(confirmed.velocity_status == "moving" && confirmed.velocity_valid,
            "A19-NULL-2 setup: second agreeing window did not confirm the motion");

    // The same stamp again: the estimate is withdrawn and the window restarts at this sample.
    const auto repeated = step(0.30F, 700.5);
    require(repeated.velocity_status == "timestamp_gap" && !repeated.velocity_valid && speed_of(repeated) == 0.0F,
            "A19-NULL-2: repeated stamp did not withdraw the estimate");
    // First full window after the restart, same direction as before (0.06 m in 0.25 s): a
    // candidate again, not a confirmed motion.
    const auto first_after = step(0.36F, 700.75);
    require(first_after.velocity_status == "motion_unconfirmed",
            "A19-NULL-2: first window after the repeated stamp was confirmed by windows from before it");
    require(!first_after.velocity_valid, "A19-NULL-2: first window after the repeated stamp was marked valid");
    require(std::fabs(first_after.velocity_3d[0] - 0.24F) < 1e-5F,
            "A19-NULL-2: first window after the repeated stamp was not measured from the restart sample");
    // The second agreeing window after the restart confirms it.
    const auto second_after = step(0.42F, 701.0);
    require(second_after.velocity_status == "moving" && second_after.velocity_valid &&
                std::fabs(second_after.velocity_3d[0] - 0.24F) < 1e-5F,
            "A19-NULL-2: second agreeing window after the repeated stamp did not confirm the motion");
    std::cout << "A19-NULL-2 estimator: a repeated stamp restarts motion confirmation PASS\n";
}

// B8-4: skipped results reach the estimator as irregular capture spacing. A
// constant 1 m/s target must read 1 m/s at every valid output whether or not
// results were skipped, including a 0.3 s hole inside dt_max.
void test_skipped_results_follow_capture_time() {
    const auto run = [](const std::vector<int>& frames) {
        perception::TrackVelocityEstimator estimator(0.5);
        estimator.set_centroid_ema_alpha(1.0F);
        estimator.set_velocity_window_s(0.25);
        estimator.set_static_displacement_m(0.04F);
        std::vector<float> estimates;
        for (const int frame : frames) {
            const double seconds = frame / 30.0;
            std::vector<perception::FusedDetection> detections = {
                make_detection(21, static_cast<float>(seconds), 0.0F, 0.0F)};
            estimator.update(detections, 200.0 + seconds);
            if (!detections.front().velocity_valid) continue;
            require(detections.front().velocity_status == "moving", "skipped-result velocity status mismatch");
            estimates.push_back(detections.front().velocity_3d[0]);
        }
        return estimates;
    };
    std::vector<int> every, skipped;
    for (int frame = 0; frame <= 60; ++frame) {
        every.push_back(frame);
        if (frame % 3 != 2 && (frame < 31 || frame > 39)) skipped.push_back(frame);
    }
    for (const auto& estimates : {run(every), run(skipped)}) {
        require(estimates.size() >= 20U, "velocity never became valid");
        for (const float vx : estimates) {
            require(std::fabs(vx - 1.0F) < 1e-4F, "skipped results changed the capture-time velocity");
        }
    }
}

// History of a track not seen within the TTL is dropped, never reused.
void test_ttl_drops_stale_history() {
    perception::TrackVelocityEstimator estimator(0.5);
    estimator.set_centroid_ema_alpha(1.0F);
    std::vector<perception::FusedDetection> first = {make_detection(4, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 40.0);

    std::vector<perception::FusedDetection> other = {make_detection(5, 0.0F, 0.0F, 0.0F)};
    estimator.update(other, 40.6);

    std::vector<perception::FusedDetection> stale_track = {make_detection(4, 1.0F, 0.0F, 0.0F)};
    estimator.update(stale_track, 40.7);
    require(speed_of(stale_track.front()) == 0.0F, "stale track history was reused");
}

// Gated frames (depth_unreliable) report zero, invalid velocity and do not enter the window
// history.
void test_low_quality_does_not_pollute_history() {
    perception::TrackVelocityEstimator estimator(5.0);
    estimator.set_centroid_ema_alpha(1.0F);
    std::vector<perception::FusedDetection> first = {make_detection(6, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 50.0);
    require(speed_of(first.front()) == 0.0F, "first gated frame velocity was not zero");

    std::vector<perception::FusedDetection> low_quality = {make_detection(6, 100.0F, 0.0F, 0.0F)};
    low_quality.front().fusion_quality = "depth_unreliable";
    estimator.update(low_quality, 51.0);
    require(speed_of(low_quality.front()) == 0.0F, "depth_unreliable frame velocity was not zero");
    require(!low_quality.front().velocity_valid, "depth_unreliable velocity was marked valid");
    require(
        low_quality.front().velocity_status == "depth_unreliable",
        "depth_unreliable status mismatch");

    // B4-1: the gate ended the window; the next accepted sample warm-starts
    // instead of reusing the pre-gate anchor (0.0 at t=50).
    std::vector<perception::FusedDetection> normal = {make_detection(6, 1.0F, 0.0F, 0.0F)};
    estimator.update(normal, 52.0);
    require(speed_of(normal.front()) == 0.0F && !normal.front().velocity_valid &&
                normal.front().velocity_status == "warming_up",
            "depth_unreliable gate did not end the velocity window");
    std::vector<perception::FusedDetection> rebuilt = {make_detection(6, 2.0F, 0.0F, 0.0F)};
    estimator.update(rebuilt, 53.0);
    require(std::fabs(rebuilt.front().velocity_3d[0] - 1.0F) < 1e-6F && rebuilt.front().velocity_status == "moving",
            "rebuilt window did not use the first accepted post-gate anchor");
}

// Below min stable_frames the status is track_unstable with zero velocity, and the rebuild starts
// from a fresh anchor.
void test_stable_frames_gate_does_not_update_history() {
    perception::TrackVelocityEstimator estimator(5.0);
    estimator.set_centroid_ema_alpha(1.0F);
    std::vector<perception::FusedDetection> first = {make_detection(7, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 60.0);

    std::vector<perception::FusedDetection> low_stable = {make_detection(7, 20.0F, 0.0F, 0.0F)};
    low_stable.front().stable_frames = 9;
    estimator.update(low_stable, 61.0);
    require(speed_of(low_stable.front()) == 0.0F, "low stable_frames velocity was not zero");
    require(!low_stable.front().velocity_valid, "unstable-track velocity was marked valid");
    require(low_stable.front().velocity_status == "track_unstable", "unstable-track status mismatch");

    std::vector<perception::FusedDetection> normal = {make_detection(7, 1.0F, 0.0F, 0.0F)};
    estimator.update(normal, 62.0);
    require(speed_of(normal.front()) == 0.0F && normal.front().velocity_status == "warming_up",
            "stable_frames gate did not end the velocity window");
    std::vector<perception::FusedDetection> rebuilt = {make_detection(7, 1.5F, 0.0F, 0.0F)};
    estimator.update(rebuilt, 63.0);
    require(std::fabs(rebuilt.front().velocity_3d[0] - 0.5F) < 1e-6F, "stable_frames rebuild used a stale anchor");
}

// Below min depth_confidence the status is low_depth_confidence with zero velocity, and the
// rebuild starts from a fresh anchor.
void test_depth_confidence_gate_does_not_update_history() {
    perception::TrackVelocityEstimator estimator(5.0);
    estimator.set_centroid_ema_alpha(1.0F);
    std::vector<perception::FusedDetection> first = {make_detection(8, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 70.0);

    std::vector<perception::FusedDetection> low_confidence = {make_detection(8, 20.0F, 0.0F, 0.0F)};
    low_confidence.front().depth_confidence = 0.49F;
    estimator.update(low_confidence, 71.0);
    require(speed_of(low_confidence.front()) == 0.0F, "low depth_confidence velocity was not zero");
    require(!low_confidence.front().velocity_valid, "low-confidence velocity was marked valid");
    require(
        low_confidence.front().velocity_status == "low_depth_confidence",
        "low-confidence status mismatch");

    std::vector<perception::FusedDetection> normal = {make_detection(8, 1.0F, 0.0F, 0.0F)};
    estimator.update(normal, 72.0);
    require(speed_of(normal.front()) == 0.0F && normal.front().velocity_status == "warming_up",
            "depth_confidence gate did not end the velocity window");
    std::vector<perception::FusedDetection> rebuilt = {make_detection(8, 1.25F, 0.0F, 0.0F)};
    estimator.update(rebuilt, 73.0);
    require(std::fabs(rebuilt.front().velocity_3d[0] - 0.25F) < 1e-6F, "depth_confidence rebuild used a stale anchor");
}

// Velocity is computed from the EMA-smoothed centroid, and a smoothed displacement inside the
// deadzone gives zero.
void test_centroid_ema_and_displacement_deadzone() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(0.5F);
    estimator.set_static_displacement_m(0.04F);

    std::vector<perception::FusedDetection> first = {make_detection(9, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 80.0);

    std::vector<perception::FusedDetection> jitter = {make_detection(9, 0.06F, 0.0F, 0.0F)};
    estimator.update(jitter, 81.0);
    require(speed_of(jitter.front()) == 0.0F, "smoothed displacement below deadzone was not zero");
    require(std::fabs(jitter.front().centroid_3d[0] - 0.03F) < 1e-6F, "smoothed centroid was not published");

    std::vector<perception::FusedDetection> moving = {make_detection(9, 0.30F, 0.0F, 0.0F)};
    estimator.update(moving, 82.0);
    require(
        std::fabs(moving.front().velocity_3d[0] - 0.135F) < 1e-6F,
        "velocity was not computed from EMA-smoothed centroid");
    require(std::fabs(moving.front().centroid_3d[0] - 0.165F) < 1e-6F, "published centroid lost EMA state");
}

// The window rejects high-rate static jitter, never emits a partial estimate, catches slow motion
// and holds it valid between windows.
void test_window_rejects_frame_jitter_and_retains_slow_motion() {
    perception::TrackVelocityEstimator static_estimator(1.0);
    static_estimator.set_centroid_ema_alpha(1.0F);
    static_estimator.set_velocity_window_s(0.25);
    static_estimator.set_static_displacement_m(0.04F);
    std::vector<perception::FusedDetection> first = {make_detection(10, 0.0F, 0.0F, 0.0F)};
    static_estimator.update(first, 90.0);
    const float jitter_positions[] = {0.03F, -0.03F, 0.03F, -0.03F, 0.03F};
    const double jitter_stamps[] = {90.05, 90.10, 90.15, 90.20, 90.25};
    for (int i = 0; i < 5; ++i) {
        std::vector<perception::FusedDetection> jitter = {
            make_detection(10, jitter_positions[i], 0.0F, 0.0F)};
        static_estimator.update(jitter, jitter_stamps[i]);
        require(speed_of(jitter.front()) == 0.0F, "high-rate static jitter produced velocity");
    }

    perception::TrackVelocityEstimator moving_estimator(1.0);
    moving_estimator.set_centroid_ema_alpha(1.0F);
    moving_estimator.set_velocity_window_s(0.25);
    moving_estimator.set_static_displacement_m(0.04F);
    std::vector<perception::FusedDetection> moving_first = {make_detection(11, 0.0F, 0.0F, 0.0F)};
    moving_estimator.update(moving_first, 100.0);
    for (int i = 1; i <= 5; ++i) {
        std::vector<perception::FusedDetection> moving = {
            make_detection(11, 0.025F * static_cast<float>(i), 0.0F, 0.0F)};
        moving_estimator.update(moving, 100.0 + 0.05 * static_cast<double>(i));
        if (i < 5) {
            require(speed_of(moving.front()) == 0.0F, "window emitted a partial velocity estimate");
        } else {
            require(std::fabs(moving.front().velocity_3d[0] - 0.5F) < 1e-5F, "window missed slow motion");
        }
    }

    std::vector<perception::FusedDetection> held = {make_detection(11, 0.15F, 0.0F, 0.0F)};
    moving_estimator.update(held, 100.30);
    require(std::fabs(held.front().velocity_3d[0] - 0.5F) < 1e-5F, "velocity blinked between windows");
    require(held.front().velocity_valid, "held velocity lost its valid state");
    require(held.front().velocity_status == "moving", "held velocity status mismatch");
}

// Motion needs confirmation_windows windows in one direction: alternating jitter stays
// motion_unconfirmed (invalid, with its real candidate), sustained motion becomes moving.
void test_motion_confirmation_rejects_alternating_jitter_and_keeps_motion() {
    perception::TrackVelocityEstimator jitter_estimator(1.0);
    jitter_estimator.set_centroid_ema_alpha(1.0F);
    jitter_estimator.set_velocity_window_s(0.25);
    jitter_estimator.set_static_displacement_m(0.04F);
    jitter_estimator.set_motion_confirmation_windows(2);
    require(
        jitter_estimator.motion_confirmation_windows() == 2,
        "motion confirmation window count was not stored");

    std::vector<perception::FusedDetection> first = {make_detection(16, 0.0F, 0.0F, 0.0F)};
    jitter_estimator.update(first, 130.0);
    const float alternating_positions[] = {0.20F, 0.0F, 0.20F, 0.0F};
    for (int i = 0; i < 4; ++i) {
        std::vector<perception::FusedDetection> jitter = {
            make_detection(16, alternating_positions[i], 0.0F, 0.0F)};
        jitter_estimator.update(jitter, 130.3 + 0.3 * static_cast<double>(i));
        // B4-1: unconfirmed windows keep their real (alternating) candidate,
        // never valid and never moving.
        require(
            jitter.front().velocity_status == "motion_unconfirmed" && !jitter.front().velocity_valid,
            "alternating centroid jump passed motion confirmation");
        require(
            std::fabs(std::fabs(jitter.front().velocity_3d[0]) - (0.20F / 0.3F)) < 1e-4F,
            "unconfirmed alternating window lost its real candidate");
    }

    perception::TrackVelocityEstimator moving_estimator(1.0);
    moving_estimator.set_centroid_ema_alpha(1.0F);
    moving_estimator.set_velocity_window_s(0.25);
    moving_estimator.set_static_displacement_m(0.04F);
    moving_estimator.set_motion_confirmation_windows(2);
    std::vector<perception::FusedDetection> moving_first = {
        make_detection(17, 0.0F, 0.0F, 0.0F)};
    moving_estimator.update(moving_first, 140.0);
    std::vector<perception::FusedDetection> moving_candidate = {
        make_detection(17, 0.10F, 0.0F, 0.0F)};
    moving_estimator.update(moving_candidate, 140.3);
    require(
        std::fabs(moving_candidate.front().velocity_3d[0] - (0.10F / 0.3F)) < 1e-5F,
        "first motion window did not carry its real candidate");
    require(
        !moving_candidate.front().velocity_valid,
        "unconfirmed motion candidate was marked valid");
    require(
        moving_candidate.front().velocity_status == "motion_unconfirmed",
        "unconfirmed motion status mismatch");
    std::vector<perception::FusedDetection> moving_confirmed = {
        make_detection(17, 0.20F, 0.0F, 0.0F)};
    moving_estimator.update(moving_confirmed, 140.6);
    require(
        std::fabs(moving_confirmed.front().velocity_3d[0] - (1.0F / 3.0F)) < 1e-5F,
        "consistent motion did not pass confirmation");
    require(moving_confirmed.front().velocity_valid, "confirmed motion was not marked valid");
    require(moving_confirmed.front().velocity_status == "moving", "confirmed motion status mismatch");
}

// The chair static-displacement override rejects chair jitter without reducing person sensitivity.
void test_per_class_static_displacement_does_not_blunt_other_classes() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_static_displacement_m(0.06F);
    estimator.set_per_class_static_displacements_m({{"chair", 0.10F}});
    require(
        estimator.per_class_static_displacements_m().at("chair") == 0.10F,
        "chair displacement override was not stored");

    std::vector<perception::FusedDetection> first = {
        make_detection(12, 0.0F, 0.0F, 0.0F),
        make_detection(13, 0.0F, 0.0F, 0.0F)};
    first[0].class_name = "chair";
    first[1].class_name = "person";
    estimator.update(first, 110.0);

    std::vector<perception::FusedDetection> moved = {
        make_detection(12, 0.08F, 0.0F, 0.0F),
        make_detection(13, 0.08F, 0.0F, 0.0F)};
    moved[0].class_name = "chair";
    moved[1].class_name = "person";
    estimator.update(moved, 111.0);
    require(speed_of(moved[0]) == 0.0F, "chair override did not reject static centroid jitter");
    require(
        std::fabs(moved[1].velocity_3d[0] - 0.08F) < 1e-6F,
        "chair override reduced person motion sensitivity");
}

// The chair centroid EMA applies to chair only, not to person.
void test_per_class_centroid_ema_does_not_slow_other_classes() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(0.5F);
    estimator.set_per_class_centroid_ema_alphas({{"chair", 0.25F}});
    require(
        estimator.per_class_centroid_ema_alphas().at("chair") == 0.25F,
        "chair centroid EMA override was not stored");

    std::vector<perception::FusedDetection> first = {
        make_detection(14, 0.0F, 0.0F, 0.0F),
        make_detection(15, 0.0F, 0.0F, 0.0F)};
    first[0].class_name = "chair";
    first[1].class_name = "person";
    estimator.update(first, 120.0);

    std::vector<perception::FusedDetection> moved = {
        make_detection(14, 1.0F, 0.0F, 0.0F),
        make_detection(15, 1.0F, 0.0F, 0.0F)};
    moved[0].class_name = "chair";
    moved[1].class_name = "person";
    estimator.update(moved, 121.0);
    require(
        std::fabs(moved[0].centroid_3d[0] - 0.25F) < 1e-6F,
        "chair did not use per-class centroid EMA");
    require(
        std::fabs(moved[1].centroid_3d[0] - 0.5F) < 1e-6F,
        "chair centroid EMA leaked into person");
}

// Generic weak proxies use the generic EMA and static displacement, not their proxy class's
// overrides.
void test_generic_weak_policy_bypasses_proxy_class_overrides() {
    perception::TrackVelocityEstimator ema_estimator(2.0);
    ema_estimator.set_centroid_ema_alpha(0.5F);
    ema_estimator.set_per_class_centroid_ema_alphas({{"chair", 0.05F}});

    std::vector<perception::FusedDetection> ema_first = {
        make_detection(18, 0.0F, 0.0F, 0.0F)};
    ema_first.front().class_name = "chair";
    ema_first.front().use_generic_velocity_policy = true;
    ema_estimator.update(ema_first, 150.0);

    std::vector<perception::FusedDetection> ema_moved = {
        make_detection(18, 1.0F, 0.0F, 0.0F)};
    ema_moved.front().class_name = "chair";
    ema_moved.front().use_generic_velocity_policy = true;
    ema_estimator.update(ema_moved, 151.0);
    require(
        std::fabs(ema_moved.front().centroid_3d[0] - 0.5F) < 1e-6F,
        "weak proxy used class-specific centroid EMA");

    perception::TrackVelocityEstimator displacement_estimator(2.0);
    displacement_estimator.set_centroid_ema_alpha(1.0F);
    displacement_estimator.set_static_displacement_m(0.02F);
    displacement_estimator.set_generic_static_displacement_m(0.06F);
    displacement_estimator.set_per_class_static_displacements_m({{"chair", 0.16F}});
    require(
        displacement_estimator.generic_static_displacement_m() == 0.06F,
        "generic static displacement was not stored");

    std::vector<perception::FusedDetection> displacement_first = {
        make_detection(19, 0.0F, 0.0F, 0.0F)};
    displacement_first.front().class_name = "chair";
    displacement_first.front().use_generic_velocity_policy = true;
    displacement_estimator.update(displacement_first, 160.0);

    std::vector<perception::FusedDetection> displacement_moved = {
        make_detection(19, 0.10F, 0.0F, 0.0F)};
    displacement_moved.front().class_name = "chair";
    displacement_moved.front().use_generic_velocity_policy = true;
    displacement_estimator.update(displacement_moved, 161.0);
    require(
        std::fabs(displacement_moved.front().velocity_3d[0] - 0.10F) < 1e-6F,
        "weak proxy used class-specific static displacement");
    require(
        displacement_moved.front().velocity_status == "moving",
        "weak proxy motion was suppressed by proxy-class policy");
}

// A normal 0.1 m/s track becomes moving within the expected delay and static after stopping; an
// unset generic displacement falls back to the global one.
void test_normal_slow_motion_and_generic_weak_proxy_thresholds() {
    perception::TrackVelocityEstimator normal_estimator(0.5);
    normal_estimator.set_centroid_ema_alpha(0.35F);
    normal_estimator.set_velocity_window_s(0.30);
    normal_estimator.set_motion_confirmation_windows(2);
    normal_estimator.set_static_displacement_m(0.02F);
    require(
        normal_estimator.generic_static_displacement_m() == 0.02F,
        "unset generic displacement did not fall back to the global value");
    normal_estimator.set_generic_static_displacement_m(0.06F);

    double first_moving_stamp = -1.0;
    for (int sample = 0; sample <= 20; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(21, 0.0125F * static_cast<float>(sample), 0.0F, 0.0F)};
        frame.front().class_name = "person";
        const double stamp = 180.0 + 0.125 * static_cast<double>(sample);
        normal_estimator.update(frame, stamp);
        if (sample == 3) {
            require(
                frame.front().velocity_status == "motion_unconfirmed",
                "normal slow motion skipped the confirmation state");
            require(
                !frame.front().velocity_valid,
                "unconfirmed normal slow motion was marked valid");
        }
        if (frame.front().velocity_status == "moving" && first_moving_stamp < 0.0) {
            first_moving_stamp = stamp;
        }
    }
    require(first_moving_stamp >= 0.0, "normal 0.1 m/s track never became moving");
    require(
        first_moving_stamp - 180.0 <= 0.75 + 1e-9,
        "normal 0.1 m/s track took too long to become moving");

    double first_static_stamp = -1.0;
    for (int sample = 21; sample <= 24; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(21, 0.25F, 0.0F, 0.0F)};
        frame.front().class_name = "person";
        const double stamp = 180.0 + 0.125 * static_cast<double>(sample);
        normal_estimator.update(frame, stamp);
        if (frame.front().velocity_status == "static" && first_static_stamp < 0.0) {
            first_static_stamp = stamp;
        }
    }
    require(first_static_stamp >= 0.0, "stopped normal track never became static");
    require(
        first_static_stamp - 182.5 <= 0.6,
        "stopped normal track exceeded the 0.6 s static-response bound");
    require(
        std::fabs(first_static_stamp - 183.0) < 1e-9,
        "stopped normal track did not become static at the expected window");

    perception::TrackVelocityEstimator generic_estimator(0.5);
    generic_estimator.set_centroid_ema_alpha(0.35F);
    generic_estimator.set_velocity_window_s(0.30);
    generic_estimator.set_motion_confirmation_windows(2);
    generic_estimator.set_static_displacement_m(0.02F);
    generic_estimator.set_generic_static_displacement_m(0.06F);
    for (int sample = 0; sample <= 20; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(22, 0.0125F * static_cast<float>(sample), 0.0F, 0.0F)};
        frame.front().class_name = "chair";
        frame.front().use_generic_velocity_policy = true;
        generic_estimator.update(
            frame, 190.0 + 0.125 * static_cast<double>(sample));
        if (sample >= 3) {
            require(
                frame.front().velocity_status == "static",
                "generic weak proxy did not use the independent 0.06 m deadzone");
            require(
                frame.front().velocity_valid,
                "generic weak proxy static estimate was not valid");
            require(
                speed_of(frame.front()) == 0.0F,
                "generic weak proxy static estimate was nonzero");
        }
    }
}

// Static and alternating generic weak proxies never become moving (static valid zero, or
// unconfirmed invalid).
void test_static_and_alternating_generic_weak_proxy_stay_non_moving() {
    perception::TrackVelocityEstimator static_estimator(0.5);
    static_estimator.set_centroid_ema_alpha(1.0F);
    static_estimator.set_velocity_window_s(0.30);
    static_estimator.set_motion_confirmation_windows(2);
    static_estimator.set_static_displacement_m(0.02F);
    static_estimator.set_generic_static_displacement_m(0.06F);
    for (int sample = 0; sample <= 4; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(23, 0.0F, 0.0F, 0.0F)};
        frame.front().use_generic_velocity_policy = true;
        static_estimator.update(
            frame, 200.0 + 0.125 * static_cast<double>(sample));
        if (sample >= 3) {
            require(
                frame.front().velocity_status == "static",
                "static generic weak proxy did not settle to static");
            require(
                frame.front().velocity_valid,
                "static generic weak proxy estimate was not valid");
            require(
                speed_of(frame.front()) == 0.0F,
                "static generic weak proxy produced nonzero velocity");
        }
    }

    perception::TrackVelocityEstimator alternating_estimator(0.5);
    alternating_estimator.set_centroid_ema_alpha(1.0F);
    alternating_estimator.set_velocity_window_s(0.30);
    alternating_estimator.set_motion_confirmation_windows(2);
    alternating_estimator.set_static_displacement_m(0.02F);
    alternating_estimator.set_generic_static_displacement_m(0.06F);
    const float alternating_positions[] = {0.0F, 0.08F, 0.0F, 0.08F, 0.0F};
    for (int sample = 0; sample < 5; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(24, alternating_positions[sample], 0.0F, 0.0F)};
        frame.front().use_generic_velocity_policy = true;
        alternating_estimator.update(
            frame, 210.0 + 0.375 * static_cast<double>(sample));
        if (sample > 0) {
            require(
                frame.front().velocity_status == "motion_unconfirmed",
                "alternating generic proxy did not remain unconfirmed");
            require(
                !frame.front().velocity_valid,
                "alternating generic proxy was marked velocity-valid");
            require(
                std::fabs(std::fabs(frame.front().velocity_3d[0]) - (0.08F / 0.375F)) < 1e-4F,
                "alternating generic proxy lost its real unconfirmed candidate");
        }
    }
}

// When a weak track is promoted, its cached weak velocity is not reused: the same id warm-starts.
void test_velocity_policy_promotion_warm_starts_same_id() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_static_displacement_m(0.06F);
    estimator.set_per_class_static_displacements_m({{"chair", 0.16F}});

    std::vector<perception::FusedDetection> weak_first = {
        make_detection(20, 0.0F, 0.0F, 0.0F)};
    weak_first.front().class_name = "suitcase";
    weak_first.front().use_generic_velocity_policy = true;
    estimator.update(weak_first, 170.0);

    std::vector<perception::FusedDetection> weak_moving = {
        make_detection(20, 0.10F, 0.0F, 0.0F)};
    weak_moving.front().class_name = "suitcase";
    weak_moving.front().use_generic_velocity_policy = true;
    estimator.update(weak_moving, 171.0);
    require(
        weak_moving.front().velocity_status == "moving",
        "promotion setup did not create cached weak motion");

    std::vector<perception::FusedDetection> promoted = {
        make_detection(20, 0.10F, 0.0F, 0.0F)};
    promoted.front().class_name = "chair";
    promoted.front().use_generic_velocity_policy = false;
    estimator.update(promoted, 171.1);
    require(speed_of(promoted.front()) == 0.0F, "promotion leaked cached weak velocity");
    require(!promoted.front().velocity_valid, "promotion did not warm-start velocity");
    require(
        promoted.front().velocity_status == "warming_up",
        "promotion retained cached weak velocity status");
}

// While the depth is held no velocity is produced, and an accepted depth jump restarts the history
// instead of comparing with the old anchor.
void test_depth_hysteresis_transition_resets_history() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);

    std::vector<perception::FusedDetection> old_depth_first = {
        make_detection(41, 0.0F, 0.0F, 3.5F)};
    estimator.update(old_depth_first, 350.0);

    std::vector<perception::FusedDetection> old_depth_static = {
        make_detection(41, 0.0F, 0.0F, 3.5F)};
    estimator.update(old_depth_static, 350.3);
    require(
        old_depth_static.front().velocity_valid &&
            old_depth_static.front().velocity_status == "static",
        "depth-transition setup did not establish a valid static history");

    std::vector<perception::FusedDetection> held = {
        make_detection(41, 0.0F, 0.0F, 3.5F)};
    held.front().depth_hysteresis_held = true;
    estimator.update(held, 350.4);
    require(speed_of(held.front()) == 0.0F, "held depth mode produced velocity");
    require(!held.front().velocity_valid, "held depth mode retained velocity validity");
    require(
        held.front().velocity_status == "warming_up",
        "held depth mode changed the compatible warm-up status");

    std::vector<perception::FusedDetection> accepted_new_depth = {
        make_detection(41, 0.0F, 0.0F, 1.3F)};
    estimator.update(accepted_new_depth, 350.7);
    require(
        speed_of(accepted_new_depth.front()) == 0.0F,
        "accepted depth mode was compared with the old depth anchor");
    require(
        !accepted_new_depth.front().velocity_valid,
        "first frame of the accepted depth mode was marked velocity-valid");
    require(
        accepted_new_depth.front().velocity_status == "warming_up",
        "first frame of the accepted depth mode did not warm-start");

    std::vector<perception::FusedDetection> new_depth_static = {
        make_detection(41, 0.0F, 0.0F, 1.3F)};
    estimator.update(new_depth_static, 351.0);
    require(
        speed_of(new_depth_static.front()) == 0.0F,
        "new static depth produced velocity");
    require(
        new_depth_static.front().velocity_valid &&
            new_depth_static.front().velocity_status == "static",
        "new static depth did not settle after a fresh window");
}

// After a depth-mode change, real motion is recovered without the old mode leaking into the
// velocity.
void test_depth_hysteresis_transition_recovers_real_motion() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(2);

    std::vector<perception::FusedDetection> old_depth = {
        make_detection(42, 0.0F, 0.0F, 3.5F)};
    estimator.update(old_depth, 360.0);

    std::vector<perception::FusedDetection> held = {
        make_detection(42, 0.0F, 0.0F, 3.5F)};
    held.front().depth_hysteresis_held = true;
    estimator.update(held, 360.1);

    std::vector<perception::FusedDetection> accepted_new_depth = {
        make_detection(42, 0.0F, 0.0F, 1.3F)};
    estimator.update(accepted_new_depth, 360.2);
    require(
        !accepted_new_depth.front().velocity_valid &&
            accepted_new_depth.front().velocity_status == "warming_up",
        "accepted depth mode did not start a fresh motion history");

    std::vector<perception::FusedDetection> motion_candidate = {
        make_detection(42, 0.10F, 0.0F, 1.3F)};
    estimator.update(motion_candidate, 360.5);
    require(
        std::fabs(motion_candidate.front().velocity_3d[0] - (0.10F / 0.3F)) < 1e-5F &&
            std::fabs(motion_candidate.front().velocity_3d[2]) < 1e-6F &&
            !motion_candidate.front().velocity_valid &&
            motion_candidate.front().velocity_status == "motion_unconfirmed",
        "first real-motion window after depth transition bypassed confirmation");

    std::vector<perception::FusedDetection> motion_confirmed = {
        make_detection(42, 0.20F, 0.0F, 1.3F)};
    estimator.update(motion_confirmed, 360.8);
    require(
        motion_confirmed.front().velocity_valid &&
            motion_confirmed.front().velocity_status == "moving",
        "sustained real motion did not recover after depth transition");
    require(
        std::fabs(motion_confirmed.front().velocity_3d[0] - (1.0F / 3.0F)) < 1e-5F,
        "recovered real-motion velocity mismatch");
    require(
        std::fabs(motion_confirmed.front().velocity_3d[2]) < 1e-6F,
        "old depth mode leaked into recovered velocity");
}

// The default decision axes keep the legacy full-3D behaviour.
void test_default_motion_decision_keeps_legacy3d_behavior() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_static_displacement_m(0.04F);
    require(
        estimator.motion_decision_axes() == perception::MotionDecisionAxes::XYZ,
        "default motion-decision axes did not preserve XYZ behavior");

    std::vector<perception::FusedDetection> first = {
        make_detection(25, 0.0F, 0.0F, 0.0F)};
    estimator.update(first, 220.0);
    std::vector<perception::FusedDetection> moved = {
        make_detection(25, 0.0F, 0.10F, 0.0F)};
    estimator.update(moved, 221.0);
    require(
        moved.front().velocity_status == "moving",
        "default XYZ decision no longer detects Y motion");
    require(
        std::fabs(moved.front().velocity_3d[1] - 0.10F) < 1e-6F,
        "default XYZ decision changed the legacy 3D velocity");
}

// Camera XZ decision axes ignore pure Y drift (no nonzero velocity).
void test_camera_motion_decision_ignores_y_drift() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(2);
    require(
        estimator.motion_decision_axes() == perception::MotionDecisionAxes::XZ,
        "camera XZ decision axes were not stored");

    for (int sample = 0; sample <= 12; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(26, 0.0F, 0.10F * static_cast<float>(sample), 0.0F)};
        estimator.update(frame, 230.0 + 0.125 * static_cast<double>(sample));
        require(
            frame.front().velocity_status != "moving",
            "camera-only Y drift was classified as moving");
        require(
            speed_of(frame.front()) == 0.0F,
            "camera-only Y drift produced a nonzero velocity");
    }
}

// Camera XZ decision axes detect X/Z motion within the usual confirmation delay.
void test_camera_motion_decision_detects_x_and_z_motion() {
    for (int moving_axis : {0, 2}) {
        perception::TrackVelocityEstimator estimator(1.0);
        estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
        estimator.set_centroid_ema_alpha(1.0F);
        estimator.set_velocity_window_s(0.30);
        estimator.set_static_displacement_m(0.02F);
        estimator.set_motion_confirmation_windows(2);

        double first_moving_stamp = -1.0;
        for (int sample = 0; sample <= 8; ++sample) {
            float position[3] = {0.0F, 0.0F, 0.0F};
            position[moving_axis] = 0.0125F * static_cast<float>(sample);
            std::vector<perception::FusedDetection> frame = {
                make_detection(27 + moving_axis, position[0], position[1], position[2])};
            const double stamp = 240.0 + 0.125 * static_cast<double>(sample);
            estimator.update(frame, stamp);
            if (frame.front().velocity_status == "moving" && first_moving_stamp < 0.0) {
                first_moving_stamp = stamp;
            }
        }
        require(first_moving_stamp >= 0.0, "camera X/Z motion never became moving");
        require(
            first_moving_stamp - 240.0 <= 0.75 + 1e-9,
            "camera X/Z motion exceeded the existing confirmation delay");
    }
}

// Odom XY decision axes ignore Z drift and detect X/Y motion within the usual delay.
void test_odom_motion_decision_ignores_z_and_detects_xy_motion() {
    perception::TrackVelocityEstimator static_estimator(1.0);
    static_estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XY);
    static_estimator.set_centroid_ema_alpha(1.0F);
    static_estimator.set_velocity_window_s(0.25);
    static_estimator.set_static_displacement_m(0.04F);
    static_estimator.set_motion_confirmation_windows(2);
    require(
        static_estimator.motion_decision_axes() == perception::MotionDecisionAxes::XY,
        "odom XY decision axes were not stored");

    for (int sample = 0; sample <= 12; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(30, 0.0F, 0.0F, 0.10F * static_cast<float>(sample))};
        static_estimator.update(frame, 250.0 + 0.125 * static_cast<double>(sample));
        require(
            frame.front().velocity_status != "moving",
            "odom-only Z drift was classified as moving");
        require(
            speed_of(frame.front()) == 0.0F,
            "odom-only Z drift produced a nonzero velocity");
    }

    for (int moving_axis : {0, 1}) {
        perception::TrackVelocityEstimator moving_estimator(1.0);
        moving_estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XY);
        moving_estimator.set_centroid_ema_alpha(1.0F);
        moving_estimator.set_velocity_window_s(0.30);
        moving_estimator.set_static_displacement_m(0.02F);
        moving_estimator.set_motion_confirmation_windows(2);

        double first_moving_stamp = -1.0;
        for (int sample = 0; sample <= 8; ++sample) {
            float position[3] = {0.0F, 0.0F, 0.0F};
            position[moving_axis] = 0.0125F * static_cast<float>(sample);
            std::vector<perception::FusedDetection> frame = {
                make_detection(31 + moving_axis, position[0], position[1], position[2])};
            const double stamp = 260.0 + 0.125 * static_cast<double>(sample);
            moving_estimator.update(frame, stamp);
            if (frame.front().velocity_status == "moving" &&
                first_moving_stamp < 0.0) {
                first_moving_stamp = stamp;
            }
        }
        require(first_moving_stamp >= 0.0, "odom X/Y motion never became moving");
        require(
            first_moving_stamp - 260.0 <= 0.75 + 1e-9,
            "odom X/Y motion exceeded the existing confirmation delay");
    }
}

// Decision axes only gate the motion decision; a moving output keeps the full 3D velocity.
void test_decision_axes_control_gating_but_keep_full3d_velocity() {
    const auto verify = [](perception::MotionDecisionAxes axes,
                           int excluded_axis,
                           int track_id,
                           double base_stamp) {
        perception::TrackVelocityEstimator estimator(1.0);
        estimator.set_motion_decision_axes(axes);
        estimator.set_centroid_ema_alpha(1.0F);
        estimator.set_velocity_window_s(0.25);
        estimator.set_static_displacement_m(0.04F);
        estimator.set_motion_confirmation_windows(2);

        std::vector<perception::FusedDetection> first = {
            make_detection(track_id, 0.0F, 0.0F, 0.0F)};
        estimator.update(first, base_stamp);

        float candidate_position[3] = {0.10F, 0.0F, 0.0F};
        candidate_position[excluded_axis] = 0.20F;
        std::vector<perception::FusedDetection> candidate = {
            make_detection(
                track_id,
                candidate_position[0],
                candidate_position[1],
                candidate_position[2])};
        estimator.update(candidate, base_stamp + 0.30);
        require(
            candidate.front().velocity_status == "motion_unconfirmed",
            "first planar-motion window skipped confirmation");

        float confirmed_position[3] = {0.20F, 0.0F, 0.0F};
        confirmed_position[excluded_axis] = -0.20F;
        std::vector<perception::FusedDetection> confirmed = {
            make_detection(
                track_id,
                confirmed_position[0],
                confirmed_position[1],
                confirmed_position[2])};
        estimator.update(confirmed, base_stamp + 0.60);
        require(
            confirmed.front().velocity_status == "moving",
            "excluded-axis reversal broke planar direction confirmation");
        require(
            std::fabs(confirmed.front().velocity_3d[0] - (1.0F / 3.0F)) < 1e-5F,
            "planar decision changed the included-axis velocity");
        require(
            std::fabs(confirmed.front().velocity_3d[excluded_axis] + (4.0F / 3.0F)) <
                1e-5F,
            "moving output dropped the excluded component of the 3D velocity");
    };

    verify(perception::MotionDecisionAxes::XZ, 1, 33, 270.0);
    verify(perception::MotionDecisionAxes::XY, 2, 34, 280.0);
}

// The lateral image-evidence gate is off by default and then leaves camera X velocity unchanged.
void test_lateral_image_evidence_gate_defaults_off() {
    perception::TrackVelocityEstimator estimator(2.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_static_displacement_m(0.04F);
    require(
        !estimator.lateral_image_evidence_gate_enabled(),
        "lateral image-evidence gate was not default-off");

    std::vector<perception::FusedDetection> first = {
        make_detection(35, 0.0F, 0.0F, 0.0F)};
    set_bbox_center_x(first.front(), 100);
    estimator.update(first, 290.0);
    std::vector<perception::FusedDetection> moved = {
        make_detection(35, 0.10F, 0.0F, 0.0F)};
    set_bbox_center_x(moved.front(), 100);
    estimator.update(moved, 291.0);
    require(
        moved.front().velocity_status == "moving",
        "default-off image gate changed legacy camera X motion");
    require(
        std::fabs(moved.front().velocity_3d[0] - 0.10F) < 1e-6F,
        "default-off image gate changed legacy camera X velocity");
}

// With the gate on, X drift of the depth centroid without image evidence stays static.
void test_camera_lateral_image_gate_rejects_depth_centroid_x_drift() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_lateral_image_evidence_gate_enabled(true);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    require(
        estimator.lateral_image_evidence_gate_enabled(),
        "camera lateral image-evidence gate was not enabled");

    std::vector<perception::FusedDetection> first = {
        make_detection(36, 0.0F, 0.0F, 0.0F)};
    set_mask_center_x(first.front(), 100);
    set_bbox_center_x(first.front(), 100);
    estimator.update(first, 300.0);

    const float x_displacements[] = {0.035F, 0.024F, 0.047F};
    const float z_displacements[] = {0.003F, 0.008F, -0.012F};
    float x = 0.0F;
    float z = 0.0F;
    for (int window = 0; window < 3; ++window) {
        x += x_displacements[window];
        z += z_displacements[window];
        std::vector<perception::FusedDetection> frame = {
            make_detection(36, x, 0.0F, z)};
        // The raw-mask center is the preferred evidence and stays fixed. The
        // deliberately moving bbox proves that it cannot override the mask.
        set_mask_center_x(frame.front(), 100);
        set_bbox_center_x(frame.front(), 103 + 3 * window);
        estimator.update(frame, 300.3 + 0.3 * static_cast<double>(window));
        require(
            frame.front().velocity_status == "static",
            "depth-only X centroid drift passed the camera image gate");
        require(
            speed_of(frame.front()) == 0.0F,
            "depth-only X centroid drift produced nonzero velocity");
    }
}

// With the gate on, image-supported X motion is accepted and the full 3D velocity is kept.
void test_camera_lateral_image_gate_accepts_real_x_and_keeps3d_output() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_lateral_image_evidence_gate_enabled(true);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(2);

    std::vector<perception::FusedDetection> first = {
        make_detection(37, 0.0F, 0.0F, 0.0F)};
    set_bbox_center_x(first.front(), 100);
    estimator.update(first, 310.0);

    std::vector<perception::FusedDetection> candidate = {
        make_detection(37, 0.10F, 0.20F, 0.0F)};
    set_bbox_center_x(candidate.front(), 102);
    estimator.update(candidate, 310.3);
    require(
        candidate.front().velocity_status == "motion_unconfirmed",
        "first image-supported X window skipped confirmation");

    std::vector<perception::FusedDetection> confirmed = {
        make_detection(37, 0.20F, -0.20F, 0.0F)};
    set_bbox_center_x(confirmed.front(), 104);
    estimator.update(confirmed, 310.6);
    require(
        confirmed.front().velocity_status == "moving",
        "same-direction 2 px X evidence did not confirm motion");
    require(
        std::fabs(confirmed.front().velocity_3d[0] - (1.0F / 3.0F)) < 1e-5F,
        "image-supported X velocity mismatch");
    require(
        std::fabs(confirmed.front().velocity_3d[1] + (4.0F / 3.0F)) < 1e-5F,
        "image gate dropped the non-decision component of full 3D velocity");
}

// The camera image gate does not change independent Z velocity.
void test_camera_lateral_image_gate_keeps_z_motion_independent() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_lateral_image_evidence_gate_enabled(true);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(2);

    std::vector<perception::FusedDetection> first = {
        make_detection(38, 0.0F, 0.0F, 0.0F)};
    set_bbox_center_x(first.front(), 100);
    estimator.update(first, 320.0);
    std::vector<perception::FusedDetection> candidate = {
        make_detection(38, 0.0F, 0.0F, 0.10F)};
    set_bbox_center_x(candidate.front(), 100);
    estimator.update(candidate, 320.3);
    std::vector<perception::FusedDetection> confirmed = {
        make_detection(38, 0.0F, 0.0F, 0.20F)};
    set_bbox_center_x(confirmed.front(), 100);
    estimator.update(confirmed, 320.6);
    require(
        confirmed.front().velocity_status == "moving",
        "camera image gate incorrectly suppressed independent Z motion");
    require(
        std::fabs(confirmed.front().velocity_3d[2] - (1.0F / 3.0F)) < 1e-5F,
        "camera image gate changed independent Z velocity");
}

// The image gate affects neither odom velocities nor tracks without image evidence.
void test_lateral_image_gate_does_not_affect_odom_or_missing_evidence() {
    perception::TrackVelocityEstimator odom_estimator(2.0);
    odom_estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XY);
    odom_estimator.set_lateral_image_evidence_gate_enabled(true);
    odom_estimator.set_centroid_ema_alpha(1.0F);
    odom_estimator.set_static_displacement_m(0.04F);
    std::vector<perception::FusedDetection> odom_first = {
        make_detection(39, 0.0F, 0.0F, 0.0F)};
    set_bbox_center_x(odom_first.front(), 100);
    odom_estimator.update(odom_first, 330.0);
    std::vector<perception::FusedDetection> odom_moved = {
        make_detection(39, 0.10F, 0.0F, 0.0F)};
    set_bbox_center_x(odom_moved.front(), 100);
    odom_estimator.update(odom_moved, 331.0);
    require(
        odom_moved.front().velocity_status == "moving",
        "camera-only image gate leaked into odom XY decisions");

    perception::TrackVelocityEstimator no_image_estimator(2.0);
    no_image_estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    no_image_estimator.set_lateral_image_evidence_gate_enabled(true);
    no_image_estimator.set_centroid_ema_alpha(1.0F);
    no_image_estimator.set_static_displacement_m(0.04F);
    std::vector<perception::FusedDetection> no_image_first = {
        make_detection(40, 0.0F, 0.0F, 0.0F)};
    no_image_estimator.update(no_image_first, 340.0);
    std::vector<perception::FusedDetection> no_image_moved = {
        make_detection(40, 0.10F, 0.0F, 0.0F)};
    no_image_estimator.update(no_image_moved, 341.0);
    require(
        no_image_moved.front().velocity_status == "moving",
        "missing image evidence did not preserve legacy X decisions");
}

// The camera-Z uncertainty gate is off by default (multiplier 1, cap 0.06 m) and then changes
// nothing.
void test_camera_z_uncertainty_gate_defaults_off_and_preserves_legacy_behavior() {
    perception::TrackVelocityEstimator legacy(1.0);
    perception::TrackVelocityEstimator explicitly_disabled(1.0);
    for (auto* estimator : {&legacy, &explicitly_disabled}) {
        estimator->set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
        estimator->set_centroid_ema_alpha(0.35F);
        estimator->set_velocity_window_s(0.30);
        estimator->set_static_displacement_m(0.02F);
        estimator->set_motion_confirmation_windows(2);
    }
    explicitly_disabled.set_camera_z_uncertainty_gate_config({false, 1.0F, 0.06F});

    const auto default_gate = legacy.camera_z_uncertainty_gate_config();
    require(!default_gate.enabled, "camera-Z uncertainty gate was not default-off");
    require(
        std::fabs(default_gate.stddev_multiplier - 1.0F) < 1e-6F,
        "camera-Z uncertainty default multiplier mismatch");
    require(
        std::fabs(default_gate.max_threshold_m - 0.06F) < 1e-6F,
        "camera-Z uncertainty default cap mismatch");

    // The second raw window reverses (0.20 -> 0.15 m), while alpha=.35 makes
    // the two EMA window displacements remain positive. Gate=false must retain
    // the exact legacy confirmation result and numeric velocity.
    const float raw_z[] = {0.0F, 0.20F, 0.15F};
    for (int sample = 0; sample < 3; ++sample) {
        std::vector<perception::FusedDetection> legacy_frame = {
            make_detection(43, 0.0F, 0.0F, raw_z[sample])};
        std::vector<perception::FusedDetection> disabled_frame = legacy_frame;
        set_depth_stddev(legacy_frame.front(), 0.001F);
        set_depth_stddev(disabled_frame.front(), 0.001F);
        const double stamp = 370.0 + 0.30 * static_cast<double>(sample);
        legacy.update(legacy_frame, stamp);
        explicitly_disabled.update(disabled_frame, stamp);

        require(
            legacy_frame.front().velocity_status == disabled_frame.front().velocity_status,
            "gate=false changed legacy velocity status");
        require(
            legacy_frame.front().velocity_valid == disabled_frame.front().velocity_valid,
            "gate=false changed legacy velocity validity");
        for (int axis = 0; axis < 3; ++axis) {
            require(
                std::fabs(
                    legacy_frame.front().centroid_3d[axis] -
                    disabled_frame.front().centroid_3d[axis]) < 1e-6F,
                "gate=false changed the legacy EMA centroid");
            require(
                std::fabs(
                    legacy_frame.front().velocity_3d[axis] -
                    disabled_frame.front().velocity_3d[axis]) < 1e-6F,
                "gate=false changed the legacy numeric velocity");
        }
        if (sample == 2) {
            require(
                legacy_frame.front().velocity_status == "moving" &&
                    legacy_frame.front().velocity_3d[2] > 0.0F,
                "legacy EMA self-correlation setup did not reproduce moving");
        }
    }
}

// A raw Z reversal does not confirm motion that only remains in the EMA's inertia.
void test_camera_z_raw_reversal_does_not_confirm_ema_inertia() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_centroid_ema_alpha(0.35F);
    estimator.set_velocity_window_s(0.30);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

    std::vector<perception::FusedDetection> first = {
        make_detection(44, 0.0F, 0.0F, 0.0F)};
    set_depth_stddev(first.front(), 0.001F);
    estimator.update(first, 380.0);

    std::vector<perception::FusedDetection> candidate = {
        make_detection(44, 0.0F, 0.0F, 0.20F)};
    set_depth_stddev(candidate.front(), 0.001F);
    estimator.update(candidate, 380.3);
    require(
        candidate.front().velocity_status == "motion_unconfirmed",
        "first raw-Z motion window skipped confirmation");

    std::vector<perception::FusedDetection> raw_reversed = {
        make_detection(44, 0.0F, 0.0F, 0.15F)};
    set_depth_stddev(raw_reversed.front(), 0.001F);
    estimator.update(raw_reversed, 380.6);
    require(
        raw_reversed.front().centroid_3d[2] > candidate.front().centroid_3d[2],
        "alpha=.35 setup did not keep the EMA displacement in the old direction");
    require(
        raw_reversed.front().velocity_status == "motion_unconfirmed" &&
            !raw_reversed.front().velocity_valid,
        "raw-Z reversal was confirmed from EMA inertia");
    // B4-1: the unconfirmed window still reports its EMA-window candidate
    // (numeric, not valid); only the decision uses the raw reversal.
    const float expected_candidate_z =
        (raw_reversed.front().centroid_3d[2] - candidate.front().centroid_3d[2]) / 0.3F;  // anchor = previous window
    require(
        std::fabs(raw_reversed.front().velocity_3d[2] - expected_candidate_z) < 1e-4F,
        "raw-Z reversal candidate is not the EMA window estimate");
}

// Two recorded field depth sequences: with the gate off both false-moving events reproduce, with
// it on neither does.
void test_camera_z_uncertainty_reconstructs_two_field_depth_sequences() {
    // The public JSON preserves selected depth/stddev and the post-EMA
    // centroid, but not the estimator's exact pre-EMA centroid input.  These
    // are therefore evidence-informed reconstructions of the two observed
    // events, not byte-exact runtime replays.  The target Orin static canary is
    // the authoritative end-to-end regression.
    const auto replay = [](
                            bool gate_enabled,
                            int track_id,
                            double base_stamp,
                            float seed_z,
                            float seed_static_displacement_m,
                            const float* raw_z,
                            const float* stddev_m,
                            const double* offsets_s,
                            int sample_count) {
        perception::TrackVelocityEstimator estimator(1.0);
        estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
        estimator.set_centroid_ema_alpha(0.35F);
        estimator.set_velocity_window_s(0.30);
        estimator.set_static_displacement_m(seed_static_displacement_m);
        estimator.set_motion_confirmation_windows(2);
        estimator.set_camera_z_uncertainty_gate_config({gate_enabled, 1.0F, 0.06F});

        std::vector<perception::FusedDetection> seed = {
            make_detection(track_id, 0.0F, 0.0F, seed_z)};
        set_depth_stddev(seed.front(), stddev_m[0]);
        estimator.update(seed, base_stamp);

        bool saw_moving = false;
        for (int sample = 0; sample < sample_count; ++sample) {
            std::vector<perception::FusedDetection> frame = {
                make_detection(track_id, 0.0F, 0.0F, raw_z[sample])};
            frame.front().class_name = "field_far_small_target";
            set_depth_stddev(frame.front(), stddev_m[sample]);
            estimator.update(frame, base_stamp + offsets_s[sample]);
            saw_moving = saw_moving || frame.front().velocity_status == "moving";
            if (gate_enabled) {
                require(
                    frame.front().velocity_status != "moving",
                    "camera-Z gate did not suppress a field false-moving sequence");
            }
            if (sample == 0) {
                // The second field segment needs a 0.03 m setup deadzone only
                // to recreate its already-in-flight EMA state. The evidence
                // windows themselves use the canary's actual 0.02 m threshold.
                estimator.set_static_displacement_m(0.02F);
            }
        }
        return saw_moving;
    };

    // Event 1 uses the published selected-depth sequence around window anchors
    // 3.704 -> 3.757 -> 3.614 -> 3.639 m. Interstitial samples come from the
    // same zenoh_raw.jsonl interval.
    const float event1_raw_z[] = {
        3.704F,
        3.639F,
        3.665F,
        3.757F,
        3.652F,
        3.704F,
        3.614F,
        3.652F,
        3.577F,
        3.665F,
        3.639F};
    const float event1_stddev_m[] = {
        0.034F,
        0.034F,
        0.024F,
        0.026F,
        0.027F,
        0.034F,
        0.036F,
        0.031F,
        0.033F,
        0.036F,
        0.031F};
    const double event1_offsets_s[] = {
        0.31, 0.41, 0.52, 0.62, 0.72, 0.83, 0.93, 1.00, 1.08, 1.16, 1.24};
    constexpr int event1_count =
        static_cast<int>(sizeof(event1_raw_z) / sizeof(event1_raw_z[0]));

    // Event 2 uses the published selected-depth sequence around window anchors
    // 3.602 -> 3.730 -> 3.627 -> 3.589 m.
    const float event2_raw_z[] = {
        3.602F,
        3.704F,
        3.639F,
        3.730F,
        3.614F,
        3.639F,
        3.627F,
        3.602F,
        3.652F,
        3.639F,
        3.589F};
    const float event2_stddev_m[] = {
        0.037F,
        0.026F,
        0.027F,
        0.026F,
        0.031F,
        0.035F,
        0.029F,
        0.030F,
        0.028F,
        0.035F,
        0.030F};
    const double event2_offsets_s[] = {
        0.31, 0.41, 0.52, 0.62, 0.72, 0.83, 0.93, 1.00, 1.08, 1.16, 1.24};
    constexpr int event2_count =
        static_cast<int>(sizeof(event2_raw_z) / sizeof(event2_raw_z[0]));

    require(
        replay(
            false,
            53,
            470.0,
            3.648615F,
            0.02F,
            event1_raw_z,
            event1_stddev_m,
            event1_offsets_s,
            event1_count),
        "gate=false did not reproduce field false-moving event 1");
    require(
        replay(
            false,
            54,
            480.0,
            3.669692F,
            0.03F,
            event2_raw_z,
            event2_stddev_m,
            event2_offsets_s,
            event2_count),
        "gate=false did not reproduce field false-moving event 2");
    require(
        !replay(
            true,
            55,
            490.0,
            3.648615F,
            0.02F,
            event1_raw_z,
            event1_stddev_m,
            event1_offsets_s,
            event1_count),
        "gate=true retained field false-moving event 1");
    require(
        !replay(
            true,
            56,
            500.0,
            3.669692F,
            0.03F,
            event2_raw_z,
            event2_stddev_m,
            event2_offsets_s,
            event2_count),
        "gate=true retained field false-moving event 2");
}

// Depth noise of a far small target stays static with the gate on.
void test_camera_z_uncertainty_rejects_far_small_target_depth_noise() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.30);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

    const float z_positions[] = {3.650F, 3.689F, 3.668F, 3.705F, 3.679F};
    const float depth_stddevs[] = {0.026F, 0.037F, 0.031F, 0.034F, 0.029F};
    for (int sample = 0; sample < 5; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(45, 0.0F, 0.0F, z_positions[sample])};
        frame.front().class_name = "far_small_target";
        set_depth_stddev(frame.front(), depth_stddevs[sample]);
        estimator.update(frame, 390.0 + 0.30 * static_cast<double>(sample));
        if (sample > 0) {
            require(
                frame.front().velocity_status == "static" && frame.front().velocity_valid,
                "0.021-0.039 m far-target depth noise was classified as moving");
            require(
                speed_of(frame.front()) == 0.0F,
                "far-target uncertainty gate emitted a numeric velocity");
        }
    }
}

// Near, low-noise 0.1 m/s Z motion still becomes moving within 0.75 s with the gate on.
void test_camera_z_uncertainty_keeps_near_low_noise_slow_motion_responsive() {
    perception::TrackVelocityEstimator estimator(0.5);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_centroid_ema_alpha(0.35F);
    estimator.set_velocity_window_s(0.30);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

    double first_moving_stamp = -1.0;
    for (int sample = 0; sample <= 8; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(46, 0.0F, 0.0F, 0.0125F * static_cast<float>(sample))};
        frame.front().class_name = "near_low_noise_target";
        set_depth_stddev(frame.front(), 0.003F);
        const double stamp = 400.0 + 0.125 * static_cast<double>(sample);
        estimator.update(frame, stamp);
        if (frame.front().velocity_status == "moving" && first_moving_stamp < 0.0) {
            first_moving_stamp = stamp;
        }
    }
    require(first_moving_stamp >= 0.0, "near low-noise 0.1 m/s Z motion never became moving");
    require(
        first_moving_stamp - 400.0 <= 0.75 + 1e-9,
        "near low-noise 0.1 m/s Z motion exceeded the 0.75 s response bound");
}

// The Z gate changes neither image-supported X velocity nor the published numeric Z component.
void test_camera_z_uncertainty_does_not_affect_image_supported_x_or_numeric_z() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_lateral_image_evidence_gate_enabled(true);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.30);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

    std::vector<perception::FusedDetection> first = {
        make_detection(47, 0.0F, 0.0F, 0.0F)};
    set_bbox_center_x(first.front(), 100);
    set_depth_stddev(first.front(), 0.030F);
    estimator.update(first, 410.0);

    std::vector<perception::FusedDetection> candidate = {
        make_detection(47, 0.10F, 0.0F, 0.03F)};
    set_bbox_center_x(candidate.front(), 102);
    set_depth_stddev(candidate.front(), 0.030F);
    estimator.update(candidate, 410.3);
    require(
        candidate.front().velocity_status == "motion_unconfirmed",
        "first image-supported X window skipped confirmation");

    std::vector<perception::FusedDetection> confirmed = {
        make_detection(47, 0.20F, 0.0F, 0.01F)};
    set_bbox_center_x(confirmed.front(), 104);
    set_depth_stddev(confirmed.front(), 0.030F);
    estimator.update(confirmed, 410.6);
    require(
        confirmed.front().velocity_status == "moving" && confirmed.front().velocity_valid,
        "camera-Z uncertainty gate suppressed image-supported X motion");
    require(
        std::fabs(confirmed.front().velocity_3d[0] - (1.0F / 3.0F)) < 1e-5F,
        "camera-Z uncertainty gate changed supported X velocity");
    require(
        std::fabs(confirmed.front().velocity_3d[2] + (1.0F / 15.0F)) < 1e-5F,
        "camera-Z decision gate changed the published numeric Z component");
}

// A missing or non-finite depth stddev falls back to the legacy Z behaviour (fail open).
void test_camera_z_uncertainty_missing_or_non_finite_stddev_fails_open() {
    const auto verify = [](float stddev_m, int track_id, double base_stamp) {
        perception::TrackVelocityEstimator estimator(1.0);
        estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
        estimator.set_centroid_ema_alpha(1.0F);
        estimator.set_velocity_window_s(0.30);
        estimator.set_static_displacement_m(0.02F);
        estimator.set_motion_confirmation_windows(2);
        estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

        const float z_positions[] = {0.0F, 0.03F, 0.06F};
        for (int sample = 0; sample < 3; ++sample) {
            std::vector<perception::FusedDetection> frame = {
                make_detection(track_id, 0.0F, 0.0F, z_positions[sample])};
            set_depth_stddev(frame.front(), stddev_m);
            estimator.update(frame, base_stamp + 0.30 * static_cast<double>(sample));
            if (sample == 2) {
                require(
                    frame.front().velocity_status == "moving" &&
                        frame.front().velocity_valid,
                    "missing/non-finite depth stddev did not fail open");
                require(
                    std::fabs(frame.front().velocity_3d[2] - 0.10F) < 1e-5F,
                    "fail-open path changed the legacy Z velocity");
            }
        }
    };

    verify(0.0F, 48, 420.0);
    verify(std::numeric_limits<float>::quiet_NaN(), 49, 430.0);
    verify(std::numeric_limits<float>::infinity(), 50, 440.0);
}

// The camera-only Z gate does not change odom velocities.
void test_camera_z_uncertainty_does_not_affect_odom_xy() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XY);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.30);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

    const float x_positions[] = {0.0F, 0.10F, 0.20F};
    const float z_positions[] = {0.0F, 0.03F, 0.01F};
    for (int sample = 0; sample < 3; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(51, x_positions[sample], 0.0F, z_positions[sample])};
        set_depth_stddev(frame.front(), 0.030F);
        estimator.update(frame, 450.0 + 0.30 * static_cast<double>(sample));
        if (sample == 2) {
            require(
                frame.front().velocity_status == "moving" && frame.front().velocity_valid,
                "camera-only Z uncertainty gate leaked into odom XY motion");
            require(
                std::fabs(frame.front().velocity_3d[0] - (1.0F / 3.0F)) < 1e-5F,
                "camera-only Z uncertainty gate changed odom X velocity");
            require(
                std::fabs(frame.front().velocity_3d[2] + (1.0F / 15.0F)) < 1e-5F,
                "camera-only Z uncertainty gate changed odom full-3D output");
        }
    }
}

// The deadband cap keeps larger sustained Z motion eligible to become moving.
void test_camera_z_uncertainty_cap_keeps_larger_motion_eligible() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_motion_decision_axes(perception::MotionDecisionAxes::XZ);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.30);
    estimator.set_static_displacement_m(0.02F);
    estimator.set_motion_confirmation_windows(2);
    estimator.set_camera_z_uncertainty_gate_config({true, 1.0F, 0.06F});

    const float z_positions[] = {0.0F, 0.08F, 0.16F};
    for (int sample = 0; sample < 3; ++sample) {
        std::vector<perception::FusedDetection> frame = {
            make_detection(52, 0.0F, 0.0F, z_positions[sample])};
        set_depth_stddev(frame.front(), 0.20F);
        estimator.update(frame, 460.0 + 0.30 * static_cast<double>(sample));
        if (sample == 2) {
            require(
                frame.front().velocity_status == "moving" && frame.front().velocity_valid,
                "0.06 m uncertainty cap suppressed a larger sustained Z motion");
        }
    }
}

}  // namespace


// ---- batch-4 reply B4-1 acceptance ----

// 1. First window candidate, cached between windows, confirmed by the next same-direction window.
//    (Stamps use binary-exact 0.125 s steps so window boundaries are exact.)
void test_unconfirmed_candidate_is_cached_and_confirmed() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_centroid_ema_alpha(1.0F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(2);
    std::vector<perception::FusedDetection> f0 = {make_detection(90, 0.0F, 0.0F, 2.0F)};
    estimator.update(f0, 500.0);
    std::vector<perception::FusedDetection> f1 = {make_detection(90, 0.02F, 0.0F, 2.0F)};
    estimator.update(f1, 500.125);
    require(f1.front().velocity_status == "warming_up" && speed_of(f1.front()) == 0.0F, "window filled early");
    std::vector<perception::FusedDetection> f2 = {make_detection(90, 0.12F, 0.0F, 2.0F)};
    estimator.update(f2, 500.25);
    require(f2.front().velocity_status == "motion_unconfirmed" && !f2.front().velocity_valid, "first window status");
    require(std::fabs(f2.front().velocity_3d[0] - 0.48F) < 1e-5F && std::fabs(f2.front().velocity_3d[2]) < 1e-6F,
            "first window candidate is not displacement/window");
    std::vector<perception::FusedDetection> f3 = {make_detection(90, 0.16F, 0.0F, 2.0F)};
    estimator.update(f3, 500.375);
    require(f3.front().velocity_status == "motion_unconfirmed" && !f3.front().velocity_valid &&
                f3.front().velocity_3d[0] == f2.front().velocity_3d[0],
            "held frame between windows did not reuse the cached candidate");
    std::vector<perception::FusedDetection> f4 = {make_detection(90, 0.24F, 0.0F, 2.0F)};
    estimator.update(f4, 500.5);
    require(f4.front().velocity_status == "moving" && f4.front().velocity_valid &&
                std::fabs(f4.front().velocity_3d[0] - 0.48F) < 1e-5F,
            "same-direction second window was not confirmed with its own estimate");
}

// 2. EMA alpha < 1 (not the raw displacement), cap, reversal, static.
void test_unconfirmed_candidate_ema_cap_reversal_static() {
    perception::TrackVelocityEstimator estimator(1.0);
    estimator.set_centroid_ema_alpha(0.5F);
    estimator.set_velocity_window_s(0.25);
    estimator.set_static_displacement_m(0.04F);
    estimator.set_motion_confirmation_windows(3);
    std::vector<perception::FusedDetection> f0 = {make_detection(91, 0.0F, 0.0F, 2.0F)};
    estimator.update(f0, 510.0);
    std::vector<perception::FusedDetection> f1 = {make_detection(91, 0.40F, 0.0F, 2.0F)};
    estimator.update(f1, 510.25);  // EMA 0.20, raw 0.40
    require(f1.front().velocity_status == "motion_unconfirmed", "EMA window status");
    require(std::fabs(f1.front().velocity_3d[0] - 0.8F) < 1e-5F,
            "candidate used the raw instead of the EMA displacement");
    std::vector<perception::FusedDetection> f2 = {make_detection(91, 5.0F, 0.0F, 2.0F)};
    estimator.update(f2, 510.5);  // EMA 2.6 -> 2.4 / 0.25 = 9.6 m/s, capped to 3
    require(f2.front().velocity_status == "motion_unconfirmed" && !f2.front().velocity_valid, "capped window status");
    require(std::fabs(speed_of(f2.front()) - 3.0F) < 1e-5F, "unconfirmed candidate was not capped like moving");
    std::vector<perception::FusedDetection> f3 = {make_detection(91, -3.0F, 0.0F, 2.0F)};
    estimator.update(f3, 510.75);  // EMA -0.2: reversal restarts the confirmation count
    require(f3.front().velocity_status == "motion_unconfirmed" && !f3.front().velocity_valid &&
                f3.front().velocity_3d[0] < 0.0F,
            "reversal did not restart confirmation with a real negative candidate");
    std::vector<perception::FusedDetection> f4 = {make_detection(91, -0.2F, 0.0F, 2.0F)};
    estimator.update(f4, 511.0);  // EMA -0.2: no displacement
    require(f4.front().velocity_status == "static" && f4.front().velocity_valid && speed_of(f4.front()) == 0.0F,
            "static window changed");
}

// 3. A mature moving track hit by each gate, then recovery: exact timeline,
//    no stale cache, added confirmation delay recorded by the assertions.
void test_each_gate_ends_mature_window_and_rebuilds() {
    for (int gate = 0; gate < 3; ++gate) {
        perception::TrackVelocityEstimator estimator(1.0);
        estimator.set_centroid_ema_alpha(1.0F);
        estimator.set_velocity_window_s(0.25);
        estimator.set_static_displacement_m(0.04F);
        estimator.set_motion_confirmation_windows(2);
        const int id = 92 + gate;
        double t = 520.0 + 10.0 * gate;
        float x = 0.0F;
        const auto step = [&](float dx, double dt, int gated) {
            x += dx;
            t += dt;
            std::vector<perception::FusedDetection> f = {make_detection(id, x, 0.0F, 2.0F)};
            if (gated == 1) f.front().stable_frames = 3;
            if (gated == 2) f.front().depth_confidence = 0.1F;
            if (gated == 3) f.front().fusion_quality = "depth_unreliable";
            estimator.update(f, t);
            return f.front();
        };
        step(0.0F, 0.0, 0);
        step(0.12F, 0.25, 0);
        const auto moving = step(0.12F, 0.25, 0);
        require(moving.velocity_status == "moving" && std::fabs(moving.velocity_3d[0] - 0.48F) < 1e-5F, "gate setup not moving");
        const auto gated = step(0.04F, 0.125, gate + 1);
        const char* expected = gate == 0 ? "track_unstable" : gate == 1 ? "low_depth_confidence" : "depth_unreliable";
        require(gated.velocity_status == expected && !gated.velocity_valid && speed_of(gated) == 0.0F,
                "gated frame carried a velocity");
        // Recovery timeline (0.125 s frames, 0.25 s window, 2 confirmations):
        //   +0.125 warming_up (anchor; the 0.48 m/s cache is gone)
        //   +0.250 warming_up (window not yet full)
        //   +0.375 motion_unconfirmed (first accepted window)
        //   +0.625 moving (second window)  => moving again 0.625 s after the gate
        const auto r0 = step(0.04F, 0.125, 0);
        require(r0.velocity_status == "warming_up" && !r0.velocity_valid && speed_of(r0) == 0.0F,
                "recovery reused the pre-gate velocity cache");
        const auto r1 = step(0.04F, 0.125, 0);
        require(r1.velocity_status == "warming_up" && speed_of(r1) == 0.0F, "recovery window filled early");
        const auto r2 = step(0.08F, 0.125, 0);
        require(r2.velocity_status == "motion_unconfirmed" && !r2.velocity_valid &&
                    std::fabs(r2.velocity_3d[0] - 0.48F) < 1e-4F,
                "first post-gate window is not an unconfirmed candidate from accepted samples");
        const auto r3 = step(0.12F, 0.25, 0);
        require(r3.velocity_status == "moving" && r3.velocity_valid, "post-gate motion was not re-confirmed");
    }
}

// 4. Gated samples keep the published centroid smoothing unchanged but never
//    enter the velocity support used after recovery.
void test_gated_samples_stay_out_of_velocity_support() {
    const float xs[] = {0.0F, 0.1F, 0.9F, 0.9F, 0.3F, 0.4F, 0.5F};
    const bool gated[] = {false, false, true, true, false, false, false};
    perception::TrackVelocityEstimator gated_estimator(1.0), reference(1.0);
    for (auto* e : {&gated_estimator, &reference}) {
        e->set_centroid_ema_alpha(0.5F);
        e->set_velocity_window_s(0.25);
        e->set_static_displacement_m(0.01F);
        e->set_motion_confirmation_windows(1);
    }
    float support = 0.0F;
    bool support_started = false;
    float anchor = 0.0F;
    for (int i = 0; i < 7; ++i) {
        const double t = 540.0 + 0.125 * i;
        std::vector<perception::FusedDetection> g = {make_detection(97, xs[i], 0.0F, 2.0F)};
        std::vector<perception::FusedDetection> r = {make_detection(98, xs[i], 0.0F, 2.0F)};
        if (gated[i]) g.front().fusion_quality = "depth_unreliable";
        gated_estimator.update(g, t);
        reference.update(r, t);
        require(g.front().centroid_3d[0] == r.front().centroid_3d[0],
                "gating changed the published centroid smoothing");
        if (gated[i]) {
            support_started = false;
            continue;
        }
        if (!support_started) {
            support = xs[i];
            support_started = true;
            anchor = support;
            require(i == 0 || g.front().velocity_status == "warming_up", "post-gate sample did not warm-start");
            continue;
        }
        support = 0.5F * xs[i] + 0.5F * support;
        if (i == 6) {
            // window 0.25 s from the anchor at i=4: support EMA over i=4..6 only
            const float expected = (support - anchor) / 0.25F;
            require(std::fabs(g.front().velocity_3d[0] - expected) < 1e-4F,
                    "post-gate velocity used gated samples in its support");
            require(std::fabs(g.front().velocity_3d[0] - r.front().velocity_3d[0]) > 1e-3F,
                    "reference without gating should differ (setup check)");
        }
    }
}

int main() {
    test_known_velocity();
    test_static_threshold_and_cap();
    test_dt_max_warm_start();
    test_warm_up_and_timestamp_gap_give_no_estimate();
    test_timestamp_gap_restarts_motion_confirmation();
    test_skipped_results_follow_capture_time();
    test_ttl_drops_stale_history();
    test_low_quality_does_not_pollute_history();
    test_stable_frames_gate_does_not_update_history();
    test_depth_confidence_gate_does_not_update_history();
    test_centroid_ema_and_displacement_deadzone();
    test_window_rejects_frame_jitter_and_retains_slow_motion();
    test_motion_confirmation_rejects_alternating_jitter_and_keeps_motion();
    test_per_class_static_displacement_does_not_blunt_other_classes();
    test_per_class_centroid_ema_does_not_slow_other_classes();
    test_generic_weak_policy_bypasses_proxy_class_overrides();
    test_normal_slow_motion_and_generic_weak_proxy_thresholds();
    test_static_and_alternating_generic_weak_proxy_stay_non_moving();
    test_velocity_policy_promotion_warm_starts_same_id();
    test_depth_hysteresis_transition_resets_history();
    test_depth_hysteresis_transition_recovers_real_motion();
    test_default_motion_decision_keeps_legacy3d_behavior();
    test_camera_motion_decision_ignores_y_drift();
    test_camera_motion_decision_detects_x_and_z_motion();
    test_odom_motion_decision_ignores_z_and_detects_xy_motion();
    test_decision_axes_control_gating_but_keep_full3d_velocity();
    test_lateral_image_evidence_gate_defaults_off();
    test_camera_lateral_image_gate_rejects_depth_centroid_x_drift();
    test_camera_lateral_image_gate_accepts_real_x_and_keeps3d_output();
    test_camera_lateral_image_gate_keeps_z_motion_independent();
    test_lateral_image_gate_does_not_affect_odom_or_missing_evidence();
    test_camera_z_uncertainty_gate_defaults_off_and_preserves_legacy_behavior();
    test_camera_z_raw_reversal_does_not_confirm_ema_inertia();
    test_camera_z_uncertainty_reconstructs_two_field_depth_sequences();
    test_camera_z_uncertainty_rejects_far_small_target_depth_noise();
    test_camera_z_uncertainty_keeps_near_low_noise_slow_motion_responsive();
    test_camera_z_uncertainty_does_not_affect_image_supported_x_or_numeric_z();
    test_camera_z_uncertainty_missing_or_non_finite_stddev_fails_open();
    test_camera_z_uncertainty_does_not_affect_odom_xy();
    test_camera_z_uncertainty_cap_keeps_larger_motion_eligible();
    test_unconfirmed_candidate_is_cached_and_confirmed();
    test_unconfirmed_candidate_ema_cap_reversal_static();
    test_each_gate_ends_mature_window_and_rebuilds();
    test_gated_samples_stay_out_of_velocity_support();
    std::cout << "test_track_velocity passed\n";
    return 0;
}
