/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_b_runtime.cc
 * Brief: The chs_b thread (see chs_b_runtime.h)
 *
 * Description:
 * The one judgement in this file is the poll pacing, and it is a trade rather
 * than a tuning constant.
 *
 * DDS-6 forbids blocking here -- one blocking call across planes would take out
 * both -- so the loop polls. At 200 Hz the IMU produces a sample every 5 ms, and
 * polling at exactly that rate would alias: a poll that drifts a little behind
 * the publisher takes two samples on one pass and none on the next, which the
 * age check then reads as a source that keeps going stale. Polling faster than
 * the source removes the aliasing without a queue, and the cost is a sleeping
 * thread rather than a deeper buffer that would hold stale samples (QD-5).
 *
 * The sleep is a plain sleep_for and not an absolute deadline, unlike ctrl's.
 * ctrl's cadence IS its contract (100 Hz, T-ODOM-1); this loop's cadence is
 * only "faster than the source", so drift costs nothing and the simpler form
 * is the honest one.
 */

#include "quadruped/chs_b_runtime.h"

#include <chrono>

#include "quadruped/mono_clock.h"
#include "xbrain/rtcomm/rt_thread.h"

namespace quadruped {
namespace {

namespace rt = hachist::xbrain::rtcomm;

// 13 S9.1: chs_b is SCHED_FIFO 70 -- below ctrl's 80. Missing a control period
// costs a command; missing an IMU sample costs a fraction of a degree.
constexpr int kChsBFifoPriority = 70;

// Twice the 200 Hz source rate. See the file comment: polling AT the source
// rate aliases into "the IMU keeps going stale".
constexpr auto kPollPeriod = std::chrono::microseconds(2500);

}  // namespace

ChsBRuntime::ChsBRuntime(QuadrupedProcess* proc, const QuadrupedConfig& cfg)
    : proc_(proc), cfg_(cfg) {}

ChsBRuntime::~ChsBRuntime() { Stop(); }

bool ChsBRuntime::Start(std::string* err) {
  if (running_.load(std::memory_order_acquire)) {
    if (err) *err = "chs_b runtime already started";
    return false;
  }
  try {
    // The constructor creates the domain and the readers, and throws with the
    // reason. Caught here so a missing domain-0 is a REPORT rather than an
    // abort: 13 S4.2's second row exists precisely for this case.
    dds_.reset(new ChassisDds(cfg_.dds));
  } catch (const std::exception& e) {
    if (err) *err = std::string("chs_b: ") + e.what();
    dds_.reset();
    return false;
  }
  running_.store(true, std::memory_order_release);
  thread_ = std::thread([this] { Loop(); });
  return true;
}

void ChsBRuntime::Stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
  // After the thread is joined, never before: the reader is polled from it.
  dds_.reset();
}

void ChsBRuntime::Loop() {
  priority_error_ = rt::ApplyFifoPriority(pthread_self(), kChsBFifoPriority);

  ImuSample imu;
  MotionInfoSample mi;
  double last_imu_rx = -1.0;
  double last_mi_rx = -1.0;

  while (running_.load(std::memory_order_acquire)) {
    const double now = MonoNowSeconds();
    dds_->Poll(now);

    // Handed on only when the sample is NEW. Re-offering the same reading every
    // poll would make a dead publisher look like a live one: the age check on
    // ctrl measures OUR receive time, and pushing the same sample forward every
    // 2.5 ms would keep resetting it (13 DDS-5's value-copy is what makes the
    // receive time meaningful in the first place).
    if (dds_->latest_imu(&imu) && imu.rx_mono_s > last_imu_rx) {
      last_imu_rx = imu.rx_mono_s;
      proc_->OnImu(imu.rx_mono_s, imu.wz);
    }
    if (dds_->latest_motion_info(&mi) && mi.rx_mono_s > last_mi_rx) {
      last_mi_rx = mi.rx_mono_s;
      proc_->OnMotionInfo(mi.rx_mono_s, mi.vel_x, mi.vel_y);
    }

    std::this_thread::sleep_for(kPollPeriod);
  }
}

std::uint64_t ChsBRuntime::imu_samples() const {
  return dds_ ? dds_->imu_count() : 0;
}
std::uint64_t ChsBRuntime::motion_info_samples() const {
  return dds_ ? dds_->motion_info_count() : 0;
}
int ChsBRuntime::actual_domain_id() const {
  return dds_ ? dds_->actual_domain_id() : -1;
}

}  // namespace quadruped
