/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: mono_clock.h
 * Brief: The one place this process reads a clock
 *
 * Description:
 * One function, and it exists to be the ONLY one. Every deadline, age and
 * timeout in this process is measured on CLOCK_MONOTONIC in seconds (CLK-C1),
 * and a second reader is how two time bases appear in one program: they agree
 * for as long as anyone tests them and diverge under a condition nobody
 * reproduced -- a suspend, a leap second reaching the wrong clock, or simply
 * one of them being changed to steady_clock while the other stays.
 *
 * SECONDS, as double. The unit is part of the contract (11 S3.0 makes the
 * envelope's mono a float64 in seconds), and it is stated here because the
 * milliseconds in RobotState.cmd_age_ms and EstopAck.recv_mono_ms are different
 * fields of the same message -- the one mistake this file cannot prevent is
 * mixing those two up, so it at least makes this half unambiguous.
 *
 * THE WALL CLOCK IS NOT HERE. It appears exactly once in this package, in the
 * ASDU Time field, because the vendor protocol asks for a local timestamp.
 * Nothing else may read it: scripts/lint/clock_scan.py greps for system_clock,
 * CLOCK_REALTIME and a bare rclcpp::Clock() precisely because those three look
 * identical to the steady versions at the call site.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_MONO_CLOCK_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_MONO_CLOCK_H_

#include <ctime>

namespace quadruped {

// CLOCK_MONOTONIC, in seconds. Allocation-free and safe to call from the
// realtime threads: clock_gettime on a monotonic clock is a vDSO call on this
// platform, not a syscall.
inline double MonoNowSeconds() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_MONO_CLOCK_H_
