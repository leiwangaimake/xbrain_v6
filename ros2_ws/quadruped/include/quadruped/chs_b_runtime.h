/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_b_runtime.h
 * Brief: The chs_b thread -- domain-0 reader into the process's two entry points
 *
 * Description:
 * A thread and two calls. Everything that decides anything lives elsewhere:
 * ChassisDds owns the DDS entities, QuadrupedProcess owns the 13 S4.2 priority
 * between this source and the monitor protocol, and this class only moves
 * samples from one to the other.
 *
 * WHICH THREAD. 13 S9.1's chs_b row: SCHED_FIFO 70, event driven at 200 Hz. It
 * is realtime because the IMU is the yaw integration's primary source and a
 * late sample is a yaw error, not a latency figure -- but it is BELOW ctrl (80),
 * because missing a control period costs a command and missing an IMU sample
 * costs a fraction of a degree.
 *
 * WHY IT DOES NOT TOUCH THE ODOMETRY DIRECTLY. Odometry lives on ctrl. Handing
 * it a sample from here would be two threads inside one integrator; the process
 * takes these through lock-free slots instead (12 RTC-6), and the choice
 * between this source and the 10 Hz one is made on ctrl where both are visible.
 *
 * The priority failure is reported rather than hidden: the process publishes
 * which source fed each axis, because both produce a pose and nothing
 * downstream can tell a 200 Hz yaw from a 10 Hz one by looking at it.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHS_B_RUNTIME_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHS_B_RUNTIME_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "quadruped/chs_b.h"
#include "quadruped/process.h"
#include "quadruped/quadruped_config.h"

namespace quadruped {

class ChsBRuntime {
 public:
  ChsBRuntime(QuadrupedProcess* proc, const QuadrupedConfig& cfg);
  ~ChsBRuntime();

  ChsBRuntime(const ChsBRuntime&) = delete;
  ChsBRuntime& operator=(const ChsBRuntime&) = delete;

  // Opens the participant and starts the thread. Returns false with `err` set;
  // a failure here is REPORTED and does not stop the process -- 13 S4.2's
  // second row is the whole point of having a priority table, and a robot that
  // refuses to run because the better odometry source is missing is worse than
  // one that runs on the 10 Hz source and says so.
  bool Start(std::string* err);
  void Stop();
  bool running() const { return running_.load(std::memory_order_acquire); }

  // Non-zero when the SCHED_FIFO change was refused (13 S9.1). Reported for the
  // same reason ctrl's is: under the systemd defaults it fails, and nothing
  // about it prints by itself.
  int priority_error() const { return priority_error_; }
  std::uint64_t imu_samples() const;
  std::uint64_t motion_info_samples() const;
  int actual_domain_id() const;

 private:
  void Loop();

  QuadrupedProcess* proc_;
  QuadrupedConfig cfg_;
  std::unique_ptr<ChassisDds> dds_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  int priority_error_ = 0;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_B_RUNTIME_H_
