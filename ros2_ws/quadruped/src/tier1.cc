/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tier1.cc
 * Brief: Tier 1 implementation -- the branch ladder of 11 S9.12.2, in order
 *
 * Description:
 * Step() is the pseudocode of 11 S9.12.2 with nothing added and nothing
 * reordered. It is written as one straight-line ladder rather than split into
 * per-branch helpers, and that is deliberate: the ORDER is the specification,
 * and a reader comparing this file against the contract has to be able to see
 * the whole sequence at once. Helpers would let a later edit move a branch
 * without the move being visible in a diff of the ladder.
 *
 * Three implementation points that are easy to get wrong:
 *
 *   * the timeout is TWO branches, not one. The first fires while the command
 *     is stale and sets the lock; the second fires once the command is fresh
 *     again and holds zero until an explicit enable. Merging them would make
 *     the lock self-clearing the moment the upstream came back, and 11 S9.12.1
 *     is explicit that a timeout lock needs a human.
 *   * the non-finite check covers ALL SIX axes, including the three that are
 *     trimmed to zero on every ordinary gait. The branch exists because the
 *     PAYLOAD is contaminated (11 S9.12.1 lists it as 载荷污染), and a message
 *     with a NaN in an unused field is not a message whose other fields can be
 *     trusted.
 *   * the epoch comparison is !=, not <. A cmd echoing a HIGHER generation than
 *     ours is just as much a disagreement as a lower one, and it means our own
 *     state is behind -- proceeding on that command would be acting on a stop
 *     we have not processed.
 */

#include "quadruped/tier1.h"

#include <cmath>
#include <cstddef>

#include "xbrain/enums/closed_sets.h"
#include "xbrain/units/units.h"

namespace quadruped {
namespace {

namespace sets = hachist::xbrain::enums;
using xbrain::units::Clamp;
using xbrain::units::Mps;
using xbrain::units::Radps;

constexpr std::size_t kStopReasonCount =
    sizeof(sets::kStopReason) / sizeof(sets::kStopReason[0]);

// The enum IS the closed set, in the closed set's order (11 S4.1). If a value
// is ever added to sets.yaml, this fails to compile rather than leaving an
// index that silently reads past the end of the table.
static_assert(static_cast<std::size_t>(StopReason::kNoSource) + 1 == kStopReasonCount,
              "StopReason and kStopReason must have the same length: 11 S4.1 "
              "states the closed set order IS the S9.12.2 decision order");

bool AllFinite(const Tier1Input& in) noexcept {
  return std::isfinite(in.vx) && std::isfinite(in.vy) && std::isfinite(in.wz) &&
         std::isfinite(in.vz) && std::isfinite(in.v_roll) &&
         std::isfinite(in.v_pitch);
}

}  // namespace

std::string_view StopReasonName(StopReason r) {
  const std::size_t i = static_cast<std::size_t>(r);
  // Out of range is unreachable for a value of the enum, and returning a marker
  // rather than indexing anyway keeps a future enum value from reading past the
  // table on the control thread.
  if (i >= kStopReasonCount) return std::string_view("invalid");
  return sets::kStopReason[i];
}

Tier1::Tier1(const Tier1Config& cfg)
    : limits_(cfg.limits),
      // Seconds once, at construction. Converting at each comparison is how a
      // missed division by 1000 survives review: both forms look like a number
      // compared against a number.
      cmd_timeout_s_(static_cast<double>(cfg.cmd_timeout_ms) / 1000.0) {}

Tier1Output Tier1::Stop(StopReason r) noexcept {
  Tier1Output o;
  o.stop_reason = r;
  // Every axis already defaults to 0.0; naming that here rather than relying on
  // the default is what makes "stopped" one place in this file.
  o.vx = 0.0;
  o.vy = 0.0;
  o.wz = 0.0;
  o.vz = 0.0;
  o.v_roll = 0.0;
  o.v_pitch = 0.0;
  return o;
}

Tier1Output Tier1::Step(const Tier1Input& in) noexcept {
  // ---- (1) hardware emergency stop -- LOCKS, software cannot clear it -----
  if (in.hes_raw) {
    hes_lock_ = true;
  }
  if (hes_lock_) {
    Tier1Output o = Stop(StopReason::kHes);
    // The only release path: the physical signal is gone AND a human asked.
    // Either half alone must not release it -- releasing on the signal alone
    // would restart a robot the moment someone pulled the button back out.
    if (!in.hes_raw && in.enable_requested) {
      hes_lock_ = false;
      o.event_estop_unlock = true;
    }
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }

  // ---- (2) command timeout -- LOCKS, needs an explicit enable -------------
  // A command that never arrived has unbounded age, so has_cmd == false lands
  // here. That is the contract's answer, not an omission: see the note about
  // no_source in the header.
  const double cmd_age_s = in.now_mono_s - in.last_cmd_rx_mono_s;
  if (!in.has_cmd || cmd_age_s > cmd_timeout_s_) {
    Tier1Output o = Stop(StopReason::kTimeout);
    if (!timeout_lock_) {
      timeout_lock_ = true;
      o.event_timeout_lock = true;
    }
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }
  if (timeout_lock_) {
    // Reached only once the command is fresh again. The lock still holds, and
    // only a human clears it (11 S9.12.1: the upstream came back is not the
    // same event as the upstream is trusted).
    Tier1Output o = Stop(StopReason::kTimeout);
    if (in.enable_requested) {
      timeout_lock_ = false;
      o.event_estop_unlock = true;
    }
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }

  // ---- (3) soft stop -- does NOT lock, converges by generation ------------
  // Zero is HELD, not pulsed for one period: until the upstream echoes our
  // generation it has not cancelled its behaviour sources, so its velocity is
  // not trustworthy. != and not < : see the file comment.
  if (in.cmd_estop_epoch != in.local_estop_epoch) {
    Tier1Output o = Stop(StopReason::kSoftEstop);
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }

  // ---- (4) transient protections -- none of them lock --------------------
  if (in.mode_switching) {
    Tier1Output o = Stop(StopReason::kModeSwitching);
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }

  if (in.sleep_readback) {
    // 13 F-21: the motors are already unpowered and no command will be obeyed.
    // Not a lock -- a human wakes the machine and the readback clears itself.
    Tier1Output o = Stop(StopReason::kSleep);
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }

  if (in.usage_mode_raw != kUsageModeNavigation) {
    // NAV-111. The velocity UNITS differ between usage modes, so a command
    // interpreted in the wrong one is wrong by an unknown factor. Zero is the
    // only value that means the same thing in both.
    Tier1Output o = Stop(StopReason::kModeMismatch);
    if (!mode_mismatch_seen_) {
      mode_mismatch_seen_ = true;
      o.event_mode_mismatch = true;
      o.mode_mismatch_actual = in.usage_mode_raw;
    }
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }
  mode_mismatch_seen_ = false;

  if (!AllFinite(in)) {
    Tier1Output o = Stop(StopReason::kNan);
    o.event_nan = true;
    o.hes_lock = hes_lock_;
    o.timeout_lock = timeout_lock_;
    return o;
  }

  // ---- (5) normal: clamp, then trim the axes this gait does not have -----
  Tier1Output o;
  o.stop_reason = StopReason::kNone;
  // Through the dimensional types, so a yaw rate cannot be clamped against a
  // linear limit: that call does not compile (CFG-CM-18).
  o.vx = Clamp(Mps{in.vx}, Mps{limits_.max_vx_mps}).value;
  o.vy = Clamp(Mps{in.vy}, Mps{limits_.max_vy_mps}).value;
  o.wz = Clamp(Radps{in.wz}, Radps{limits_.max_wz_radps}).value;

  // 11 S9.3.1: an inactive axis is ZEROED, never passed through -- otherwise
  // the layer above believes a command it issued was executed.
  if (!limits_.holonomic) {
    o.vy = 0.0;
  }
  // vz / v_roll / v_pitch stay at zero, unconditionally. spec.* defines no
  // limit for any of them, and Tier 1 cannot hold a command inside a limit that
  // does not exist. Clamping them against max_vx or max_wz would be inventing a
  // safety number here, in the one file where that is least acceptable. See the
  // note in tier1.h and the load-time assertion on motion.axes.special_gaits.
  o.hes_lock = hes_lock_;
  o.timeout_lock = timeout_lock_;
  return o;
}

}  // namespace quadruped
