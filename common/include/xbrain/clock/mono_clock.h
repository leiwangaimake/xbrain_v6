/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: mono_clock.h
 * Brief: The monotonic clock every XBRAIN C++ process reads its time from
 *
 * Description:
 * The C++ half of CLK-C1 (11 S0.2.1, grep the heading 时间基准), mirroring
 * xbrain/common/clock/mono_now_s() on the Python side. That section makes the
 * rule a safety item, not a style choice: every timeout, period and age
 * judgement reads a monotonic clock, and the message ts (a wall clock) is only
 * for cross-host alignment, recording and latency statistics. The failure it
 * rules out is concrete -- chronyd steps the wall clock when RTK first locks,
 * and on a cold start with no RTC battery the step can be seconds to years. An
 * age computed across a forward step exceeds the 200 ms Tier 1 deadline (T-01)
 * instantly and locks the chassis mid-drive; across a backward step it goes
 * negative, and the timeout protection is simply off for the length of the step
 * while the payload keeps executing its last velocity -- a failure that makes
 * the system FASTER, which 10 G-3 / CON-07 forbid by name.
 *
 * Why this header exists rather than each caller writing the reading inline.
 * scripts/lint/clock_scan.py is the negative half of enforcing CLK-C1: it fails
 * the build on the forbidden spellings (the realtime clock type and the POSIX
 * realtime clock id). A lint that only forbids leaves every caller to reach for
 * the clock directly and rediscover this reasoning from an incident; this header
 * is the one place the reading is defined, with the reasoning attached.
 *
 * Consumers include chassis_relay, which 11 S1.1.6 places on the emergency-stop
 * path under CRL-4 (no dynamic allocation) and CRL-5 (single hop under 200 us),
 * so this header stays in the C++17 standard library and pulls in NOTHING from
 * ROS -- 19 S1.2. tests/common/link_no_ros/ builds a translation unit that
 * includes it with no ROS on the command line and fails if that ever changes.
 *
 * What this header deliberately does NOT do:
 *   * it does not read the wall clock, and no wall_now_s belongs here. The three
 *     audited wall-clock uses (11 S0.2.1) stay at their call sites carrying the
 *     WALL-CLOCK-OK marker clock_scan.py requires; a friendly name here would
 *     turn three reviewable exceptions into a function anyone may call.
 *   * it does not compute age. Age has four branches that need the envelope
 *     fields to decide between (11 S3.0.1), and the negative-age branch must
 *     clamp to zero AND emit an event -- a subtraction helper here would satisfy
 *     the clamp and silently drop the event. Age is a decision, not arithmetic.
 *   * it does not hold the S1.6 timeout thresholds. Those are the generated
 *     table beside this file (timeout_defs.h); this is only the reading.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_CLOCK_MONO_CLOCK_H_
#define HACHIST_XBRAIN_V6_COMMON_CLOCK_MONO_CLOCK_H_

/* chrono for steady_clock, and nothing else. Every include here is one every
 * consumer pays for, and one of them is on a hop CRL-5 budgets at 200 us. */
#include <chrono>

namespace hachist {
namespace xbrain {
namespace clock {

// Seconds from the steady (monotonic) clock, as a double. This is the one call
// every timeout, period and age in C++ starts from.
//
// Why double seconds and not an integer count of ticks. The S1.6 thresholds are
// written in seconds and milliseconds (Tier 1 at 200 ms), so a caller compares
// this reading against a number in the same unit; a raw tick count would push
// the unit conversion out to every call site, which is where it gets done
// differently by different people.
//
// Why there is no fallback branch. The obvious "robust" version tries the steady
// clock and drops back to something else if it is unavailable -- and the
// something else is always the realtime clock, so the fallback path is the exact
// defect this header exists to prevent, reached only on the rare machine where
// nobody is watching. steady_clock is mandated by the standard to exist. There
// is nothing to fall back to and nothing to guard against.
inline double mono_now_s() {
  const auto since_epoch = std::chrono::steady_clock::now().time_since_epoch();
  // duration<double> divides the tick count by the period once, here, so the
  // result is seconds regardless of the platform's steady_clock resolution.
  return std::chrono::duration<double>(since_epoch).count();
}

}  // namespace clock
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_CLOCK_MONO_CLOCK_H_
