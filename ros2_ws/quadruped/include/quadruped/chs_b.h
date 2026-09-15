/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_b.h
 * Brief: Channel two -- the bare domain-0 reader (13 DDS-1..DDS-9)
 *
 * Description:
 * A CycloneDDS participant on domain 0, reading the chassis's own ROS 2 topics
 * without going through rclcpp or rmw. The header carries no DDS type: the
 * pimpl keeps the core library, its tests and chassis_relay building on a
 * machine with neither ROS nor CycloneDDS.
 *
 * Why the domain is passed to dds_create_participant explicitly and why the
 * configuration is injected rather than set in the environment (DDS-1, DDS-3):
 * this process ALSO holds an rclcpp context on domain 42, and both ROS_DOMAIN_ID
 * and CYCLONEDDS_URI are process-wide. One environment read is all it takes for
 * the two domains to collapse into one, and the symptom is a participant that
 * comes up with no traffic -- which 13 DDS-9 records as indistinguishable from
 * a dead network.
 *
 * DDS-4 / 11 RT-C5: readers only. No writer is created here and none can be:
 * the IDL this package generates from carries no command type at all, so
 * /NAV_CMD cannot be published by accident. 13 S2.3 is explicit that it is not
 * a backup velocity channel.
 *
 * DDS-5: samples are VALUE-COPIED into plain structs before anything else sees
 * them. Handing a DDS-allocated message to the rest of the process would weld
 * the two type systems together, and the loan would have to be returned from
 * whichever thread happened to finish with it.
 *
 * DDS-6: Poll() is called from the chs_b thread and does no Zenoh and no socket
 * work. One blocking call across planes would take out both.
 *
 * The two names this file depends on are in dds_names.h and the IDL, tested
 * separately: /IMU is `rt/IMU` on the wire and the type is
 * sensor_msgs::msg::dds_::Imu_. Either one wrong produces the same silence.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHS_B_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHS_B_H_

#include <cstdint>
#include <memory>
#include <string>

#include "quadruped/dds_names.h"
#include "quadruped/quadruped_config.h"

namespace quadruped {

// One IMU sample, value-copied out of the DDS loan.
struct ImuSample {
  double wz = 0.0;   // rad/s, the yaw-rate source the odometry integrates
  double wx = 0.0;
  double wy = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  double ax = 0.0, ay = 0.0, az = 0.0;
  double rx_mono_s = -1.0;   // when WE received it, not the publisher's stamp
};

// One /MOTION_INFO sample (drdds), the preferred linear-velocity source when
// the vendor topic is available (13 S4.2).
struct MotionInfoSample {
  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_yaw = 0.0;
  double height = 0.0;
  double payload = 0.0;
  double remain_mile = 0.0;
  std::int32_t motion_state = 0;
  std::uint32_t gait = 0;
  double rx_mono_s = -1.0;
};

class ChassisDds {
 public:
  // Brings up the domain and the participant. Throws on failure rather than
  // leaving an object that silently reads nothing -- which is the one failure
  // mode this whole file is arranged to avoid.
  ChassisDds(const ChassisDdsConfig& cfg);
  ~ChassisDds();

  ChassisDds(const ChassisDds&) = delete;
  ChassisDds& operator=(const ChassisDds&) = delete;

  // Take whatever has arrived. Returns the number of samples copied out.
  // Non-blocking: the caller owns its own loop rate (13 S9.1, the chs_b
  // thread at 200 Hz).
  int Poll(double now_mono_s);

  bool latest_imu(ImuSample* out) const;
  bool latest_motion_info(MotionInfoSample* out) const;

  // Negative when nothing has arrived yet, which ClassifyImuAge turns into the
  // kNeverSeen band -- a different message from "stale", because the two have
  // different causes.
  double imu_age_s(double now_mono_s) const;
  double motion_info_age_s(double now_mono_s) const;

  // The domain the participant actually joined, for the startup self-report
  // (DDS-9). Reading it back rather than echoing the config is the point: the
  // two differ exactly when something read an environment variable.
  int actual_domain_id() const;

  std::uint64_t imu_count() const;
  std::uint64_t motion_info_count() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_B_H_
