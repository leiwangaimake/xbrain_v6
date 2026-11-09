/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_status.hpp
 * Brief: The Status side of T: traversable_seg_available and the no_traversable_seg reason.
 *
 * Description:
 * For one Status cut (19 section 3.3 / 5.1), from the T slot's last publication time.
 * traversable_seg_available = a valid segmentation-slot update within the 10 s window, computed
 * exactly as apply_health_fields checks it (window end = the cut, the last COMPLETED millisecond).
 * The publication time is read before the cut is taken, so it is never after the sampled clock;
 * a publication inside the cut's still-open millisecond counts at the cut (at most 1 ms later,
 * which cannot change a 10 s availability; dropping it reported "no update" for that cut and made
 * the reason flap, found on 1.8 in t_smoke1). A time after the sampled clock can only come from
 * another clock and is never used. Before a T owner exists (ready false) there is no update.
 * Rule confirmed in issues.md 3.3 (2026-10-08): the reason is raised iff the flag is false; the
 * per-frame t_seg warning belongs to RNS.
 * Pure function: no clock, no allocation.
 */

#pragma once

#include <cstdint>
#include <optional>

namespace perception::service {

struct SegCapability {
    std::optional<std::int64_t> last_update_ms;  // for HealthFieldContext::last_valid_seg_update_ms
    bool available = false;                      // traversable_seg_available
    bool raise_reason = true;                    // no_traversable_seg
    bool in_open_ms = false;                     // counted at the cut (see above)
    bool clock_anomaly = false;                  // after the sampled clock: not used
};

// last_publish_ns: CLOCK_MONOTONIC ns of the last slot publication (-1: none), read before the
// cut. cut_ms: the cut (last completed millisecond); sampled_now_ms: the clock the cut came from.
inline SegCapability seg_capability(bool t_ready, std::int64_t last_publish_ns, std::int64_t cut_ms,
                                    std::int64_t sampled_now_ms, std::int64_t window_ms) noexcept {
    SegCapability c;
    if (t_ready && last_publish_ns >= 0) {
        const std::int64_t ms = last_publish_ns / 1000000;
        if (ms <= cut_ms) {
            c.last_update_ms = ms;
        } else if (ms <= sampled_now_ms) {
            c.last_update_ms = cut_ms;
            c.in_open_ms = true;
        } else {
            c.clock_anomaly = true;
        }
    }
    c.available = c.last_update_ms && cut_ms - *c.last_update_ms < window_ms;
    c.raise_reason = !c.available;
    return c;
}

}  // namespace perception::service
