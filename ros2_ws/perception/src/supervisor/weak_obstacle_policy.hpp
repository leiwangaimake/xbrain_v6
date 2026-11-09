/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: weak_obstacle_policy.hpp
 * Brief: Publication gate for weak (low-confidence) observations.
 *
 * Description:
 * Internal policy helpers for the generic weak-obstacle path.
 * A track whose identity or current observation came from the weak tier (a COCO detection
 * below its class threshold but not below low_confidence_floor; never a person) is published
 * only when this frame's raw depth sample is good evidence and the hysteresis-held depth is good
 * too; other tracks are always published. Supervisor::process_loop calls it once per track.
 *
 * This header intentionally lives under src/: it is shared only by supervisor
 * implementation and unit tests, and does not expand the public perception API.
 */

#pragma once

namespace perception::detail {

// Where 5 and 6.0 come from (GPT answer 2.1, 2026-10-04): the V5 D435 integration task
// (2026-07-20) marked depth unreliable beyond about 6 m or with too few pixels. 6.0 m is that
// limit; "too few" was coded as 5 by that implementation, first on all valid pixels and since
// 2026-07-22 on the selected band, and why 5 was chosen is not recorded. An inherited policy, not
// validated on the Gemini 338Le (whose vendor-recommended range is about 0.25-6 m); the field
// matrix of size x material x distance is still open (19 section 17 PD-16). The YAML key
// fusion.min_fused_points was never read and does not drive this. This internal good/unreliable
// test is not the contract depth_quality (good/fair/poor, 19 section 4.1).
inline bool has_good_raw_depth_evidence(
    bool valid,
    bool selection_fallback,
    int selected_pixel_count,
    float depth_m) {
    return valid && !selection_fallback && selected_pixel_count >= 5 && depth_m <= 6.0F;
}

inline bool should_publish_observation(
    bool requires_weak_depth_gate,
    bool raw_depth_valid,
    bool raw_depth_selection_fallback,
    int raw_depth_selected_pixel_count,
    float raw_depth_m,
    bool stabilized_depth_good) {
    return !requires_weak_depth_gate ||
           (has_good_raw_depth_evidence(
                raw_depth_valid,
                raw_depth_selection_fallback,
                raw_depth_selected_pixel_count,
                raw_depth_m) &&
            stabilized_depth_good);
}

}  // namespace perception::detail
