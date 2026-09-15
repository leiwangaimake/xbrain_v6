/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: dds_names.cc
 * Brief: The ROS-to-DDS topic mapping and the IMU freshness bands
 *
 * Description:
 * Small on purpose. The value is that it is SEPARATE: the mapping can be tested
 * without a DDS stack, and a wrong mapping is otherwise only visible as a
 * chassis that never speaks.
 */

#include "quadruped/dds_names.h"

#include <stdexcept>

namespace quadruped {

std::string RosTopicToDdsTopic(const std::string& ros_topic) {
  // A ROS topic is absolute and non-empty. Refusing the degenerate forms here
  // is what keeps a reader from being created on "rt/" -- which is a valid
  // topic name that nothing publishes, so the failure would look like silence
  // from the chassis rather than like a bad configuration.
  if (ros_topic.size() < 2 || ros_topic[0] != '/') {
    throw std::invalid_argument(
        "ROS topic must be absolute and non-empty, got: \"" + ros_topic + "\"");
  }
  if (ros_topic[ros_topic.size() - 1] == '/') {
    throw std::invalid_argument(
        "ROS topic must not end in a slash, got: \"" + ros_topic + "\"");
  }
  // Strip exactly ONE leading slash and prefix rt/. Stripping all of them would
  // silently accept "//IMU" and map it to the same place as "/IMU", hiding a
  // configuration typo that a person would otherwise be told about.
  return std::string(kRosTopicPrefix) + ros_topic.substr(1);
}

const char* ImuFreshnessName(ImuFreshness f) {
  switch (f) {
    case ImuFreshness::kNeverSeen: return "never_seen";
    case ImuFreshness::kFresh: return "fresh";
    case ImuFreshness::kStale: return "stale";
  }
  return "invalid";
}

ImuFreshness ClassifyImuAge(double age_s, int warn_ms) {
  // Negative age is the sentinel for "nothing has arrived". It is a distinct
  // band because the two causes differ: never-seen is almost always the name
  // mapping, stale is almost always the link.
  if (age_s < 0.0) return ImuFreshness::kNeverSeen;
  if (age_s * 1000.0 > static_cast<double>(warn_ms)) return ImuFreshness::kStale;
  return ImuFreshness::kFresh;
}

}  // namespace quadruped
