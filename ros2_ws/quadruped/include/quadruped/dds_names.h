/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: dds_names.h
 * Brief: ROS topic name to DDS topic name, and the IMU freshness bands
 *
 * Description:
 * The name mapping that CLAUDE.md 5.4 calls the easiest trap in this package,
 * isolated here so it can be tested on a machine with no DDS at all.
 *
 * A ROS 2 topic has TWO names. `/IMU` is what ros2 topic list prints; the name
 * on the DDS wire is `rt/IMU`, with the leading slash removed and an `rt/`
 * prefix added. A bare CycloneDDS reader created with `/IMU`, or with `IMU`,
 * is perfectly valid and matches nothing -- the participant comes up, the topic
 * exists, and no sample ever arrives. 13 DDS-9 records that this is
 * indistinguishable from a network that is down, and 13 does not mention the
 * mapping anywhere.
 *
 * The TYPE name has the same shape of trap and is handled in
 * ros2_ws/quadruped/idl/chassis_dds_types.idl: the wire name is
 * sensor_msgs::msg::dds_::Imu_, while idlc over the IDL that ships with
 * sensor_msgs emits sensor_msgs::msg::Imu. Measured 2026-09-15.
 *
 * Both halves have to be right. Either one alone produces the same silence.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_DDS_NAMES_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_DDS_NAMES_H_

#include <cstddef>
#include <string>

namespace quadruped {

// The prefix ROS 2 puts on every topic at the DDS layer. `rt` is for "ROS
// topic"; services and actions use other prefixes this process never needs.
inline constexpr const char* kRosTopicPrefix = "rt/";

// "/IMU" -> "rt/IMU". Throws std::invalid_argument on a name that is not a
// valid ROS topic, rather than producing something that would subscribe to
// silence: a mapping that quietly accepted "" would create a reader on "rt/"
// and the failure would present as a dead chassis.
std::string RosTopicToDdsTopic(const std::string& ros_topic);

// How fresh the IMU stream is. 13 S2.5: an age over ten sample periods (50 ms
// at 200 Hz) degrades the odometry onto the 10 Hz attitude from the monitor
// protocol. Three bands rather than a boolean, because "never arrived" and
// "stopped arriving" call for different messages -- the first is usually the
// name mapping above, the second is usually the cable.
enum class ImuFreshness {
  kNeverSeen,
  kFresh,
  kStale,
};

const char* ImuFreshnessName(ImuFreshness f);

// age_s < 0 means nothing has been received yet.
ImuFreshness ClassifyImuAge(double age_s, int warn_ms);

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_DDS_NAMES_H_
