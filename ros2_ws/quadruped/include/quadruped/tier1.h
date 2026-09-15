/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tier1.h
 * Brief: Tier 1 safety fallback -- the single function of 11 S9.12.2 / 13 S3.2
 *
 * Description:
 * This is the last thing between a command and the legs. It runs every control
 * period at 100 Hz and decides one of two outcomes: a clamped velocity, or
 * zero with exactly one reason attached.
 *
 * The decision ORDER is not an implementation choice. 11 S4.1 states it as the
 * order of the stop_reason closed set, verbatim: "取值顺序即 S9.12.2 的判定顺序"
 * -- the enum below is that set, in that order, and a static assertion ties it
 * to the shared library so the two cannot drift. Reordering the branches would
 * change which reason a robot reports when two conditions hold at once, and the
 * first one in the list is the one that must win because it is the one that
 * cannot be cleared from software.
 *
 * Why it is a pure function over plain data, with the clock passed in:
 *
 *   * it runs on ctrl, SCHED_FIFO 80 (13 S9.1). QD-7 / RTC-5 forbid allocation
 *     and RTC-4 forbids blocking I/O there, so there is no std::string, no
 *     container, no log call and no lock anywhere in Step();
 *   * every deadline is monotonic (CLK-C1), and injecting the reading is what
 *     makes a 200 ms timeout testable without waiting 200 ms. A test that
 *     sleeps is flaky on a loaded machine, and this is the one function whose
 *     tests must never be skipped for flakiness;
 *   * the limits are injected at construction and have no defaults. On the real
 *     machine spec.max_* is still null pending V-01, so the process refuses to
 *     start -- which is the designed behaviour. Writing 0.0 to make it run
 *     would give v_max = 0 with no error at all (CLAUDE.md 3.1, fail-silent).
 *
 * Two things this file deliberately does NOT do:
 *
 *   * it does not produce stop_reason "no_source". That value is in the closed
 *     set and has no producer anywhere -- 11's own self-audit registers the
 *     contradiction ("stop_reason 八值闭集里的 no_source 无任何产生者"). The
 *     pseudocode's answer for "no command has ever arrived" is the TIMEOUT
 *     branch, because the age of a command that never came is unbounded. An
 *     implementation that invented a producer would be changing the contract
 *     from inside the code, which is what S9.12.2's "不改一字" forbids.
 *   * it does not send anything. It returns a decision; the single tx owner
 *     (13 CA-4) writes frames, and the non-realtime publisher turns the event
 *     flags into RT-plane events. Tier 1 that could also send would be a
 *     function that allocates and blocks on the FIFO thread.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_TIER1_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_TIER1_H_

#include <cstdint>
#include <string_view>

#include "quadruped/quadruped_config.h"

namespace quadruped {

// The stop_reason closed set (11 S4.1), IN DECISION ORDER. The numbering is
// load-bearing twice over: it indexes the shared library's string table, and it
// states the precedence. kHes is first because it is the only stop software
// cannot clear.
enum class StopReason : std::uint8_t {
  kNone = 0,
  kHes = 1,
  kTimeout = 2,
  kSoftEstop = 3,
  kModeSwitching = 4,
  kSleep = 5,
  kModeMismatch = 6,
  kNan = 7,
  // In the closed set, produced by nothing -- see the file comment. Present so
  // the enum and the library's table are the same length, which is what the
  // static assertion in the .cc file checks.
  kNoSource = 8,
};

// The contract spelling, from common/include/xbrain/enums/closed_sets.h. Never
// a literal here: CLAUDE.md 3.5 keeps closed-set strings in one place, and the
// one place is generated from xbrain/common/enums/sets.yaml.
std::string_view StopReasonName(StopReason r);

// One control period's worth of input. Plain data, so a test can state a whole
// situation in one initialiser and a reader can see every condition at once.
struct Tier1Input {
  // CLOCK_MONOTONIC seconds. Passed in, never read here (CLK-C1).
  double now_mono_s = 0.0;
  // When the last cmd_vel arrived, on the same clock. Meaningless unless
  // has_cmd is true.
  double last_cmd_rx_mono_s = 0.0;
  // False until the first cmd_vel of the process's life. The age of a command
  // that never arrived is unbounded, so this lands in the timeout branch --
  // and it locks, which is correct: nobody is driving.
  bool has_cmd = false;

  // The commanded axes. All six are carried even though three of them are
  // trimmed on every ordinary gait, because a payload is checked as a whole.
  double vx = 0.0;       // m/s
  double vy = 0.0;       // m/s
  double wz = 0.0;       // rad/s
  double vz = 0.0;       // m/s     always trimmed to zero, see the note below
  double v_roll = 0.0;   // rad/s   always trimmed to zero
  double v_pitch = 0.0;  // rad/s   always trimmed to zero

  // The generation P1 echoed on this command, and ours. A mismatch means the
  // upstream has not yet acknowledged the last soft stop, so its velocity is
  // not trustworthy (11 S9.12.2 (3)).
  std::uint64_t cmd_estop_epoch = 0;
  std::uint64_t local_estop_epoch = 0;

  // BasicStatus.HES, straight off the wire.
  bool hes_raw = false;
  // BasicStatus.Sleep. The chassis has lain down and cut motor power, and
  // there is no wake command in the protocol (13 F-21 / V-63).
  bool sleep_readback = false;
  // A mode switch is in flight (11 S9.2.2).
  bool mode_switching = false;
  // BasicStatus.ControlUsageMode, RAW. Compared against the navigation value
  // rather than a resolved label, so an unregistered readback (which resolves
  // to no label at all) still fails the comparison and stops the robot.
  std::int64_t usage_mode_raw = 0;

  // A ctrl.enable that has already passed the full 11 S3.0 envelope check.
  // Tier 1 receives the verdict, never the message: validating an envelope
  // means parsing, and parsing on a FIFO thread is what QD-7 forbids.
  bool enable_requested = false;

  // NOTE: there is deliberately no "this gait has six axes" input.
  //
  // 13 S5.4 / 11 S9.3.1 describe vz, v_roll and v_pitch as live "only in
  // special gaits", and 13 V-51 records that the manual never says which gaits
  // those are. The deeper gap is the one V-51 does not mention: spec.* defines
  // max_vx_mps, max_vy_mps and max_wz_radps and NOTHING for the other three
  // axes. Tier 1 exists to hold a command inside a limit, so an axis with no
  // limit is an axis it cannot let through -- and picking max_vx as a stand-in
  // for vz would be inventing a safety number in the one file that must not.
  //
  // So the three are zeroed unconditionally. The corresponding config key,
  // motion.axes.special_gaits, is asserted EMPTY at load (see
  // quadruped_config.cc): a non-empty whitelist that this code ignores would be
  // an operator setting something and nothing happening, which is the failure
  // this project keeps finding.
};

// What the period decided.
struct Tier1Output {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  double vz = 0.0;
  double v_roll = 0.0;
  double v_pitch = 0.0;

  StopReason stop_reason = StopReason::kNone;
  // Latched state, reported every period rather than only on change: 11 CR-12
  // makes the READBACK of these two the only proof that a lock is gone, so a
  // consumer must be able to see them at any moment, not reconstruct them from
  // a stream of edges it might have missed.
  bool hes_lock = false;
  bool timeout_lock = false;

  // Edge flags for the non-realtime publisher. Booleans and not events because
  // building an event means allocating (13 QD-7): the publisher turns these
  // into 11 S6 events on its own thread.
  bool event_estop_unlock = false;      // info  -- a lock was just released
  bool event_timeout_lock = false;      // fault -- the upstream just went away
  bool event_nan = false;               // fault -- a poisoned payload
  bool event_mode_mismatch = false;     // first entry only, 100 Hz would storm
  // Populated with usage_mode_raw when event_mode_mismatch fires, so the
  // consumer can report what the mode ACTUALLY is (11 S4.1 mode_mismatch:
  // without it, "mode mismatch" says that something happened and not what).
  std::int64_t mode_mismatch_actual = 0;
};

// Holds the three pieces of state that outlive a period: the two locks and the
// mode-mismatch idempotence flag. Everything else is derived per call, which
// keeps "why are we stopped" from having two answers that can disagree.
class Tier1 {
 public:
  // Takes the whole tier1 block, not just the limits: the timeout lives beside
  // them in 13 S8.2 and both are read once here. Copied at construction and
  // never changed afterwards -- CFG-31 makes the hard limits read-only at
  // runtime, because a limit that can be raised while moving is not a limit.
  explicit Tier1(const Tier1Config& cfg);

  // One control period. noexcept, allocation-free, lock-free, clock-free.
  Tier1Output Step(const Tier1Input& in) noexcept;

  bool hes_lock() const noexcept { return hes_lock_; }
  bool timeout_lock() const noexcept { return timeout_lock_; }

  // The configured command timeout, in seconds. Exposed so a caller can report
  // the value actually in force rather than the one it believes is in force.
  double cmd_timeout_s() const noexcept { return cmd_timeout_s_; }

 private:
  // Zero every axis and stamp a reason. The only way an output becomes a stop,
  // so there is exactly one place where "stopped" is spelled out.
  static Tier1Output Stop(StopReason r) noexcept;

  Tier1Limits limits_;
  double cmd_timeout_s_ = 0.0;
  bool hes_lock_ = false;
  bool timeout_lock_ = false;
  bool mode_mismatch_seen_ = false;
};

// The chassis value for navigation (11 S9.2.4: normal 0 / navigation 1 /
// assist 2). Named rather than inline so the Tier 1 comparison reads as a
// statement about navigation and not about the number 1.
inline constexpr std::int64_t kUsageModeNavigation = 1;

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_TIER1_H_
