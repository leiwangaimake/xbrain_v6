/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: publish_sink.h
 * Brief: The RT-plane transport seam -- rtk_driver produces JSON, a sink sends it
 *
 * Description:
 * The problem this solves. rtk_driver must publish rt/gnss/heading (and later
 * rt/gnss/fix, rt/clock/status) on the RT plane, but the Zenoh binding cannot be
 * linked yet: 11 S2.4.1 records the Zenoh version as unlocked and neither zenoh-c
 * nor zenoh-cpp is installed on the build host (the same fact common/zenoh/
 * session_config.h documents). So the driver is written against THIS interface --
 * it builds the keyexpr + JSON and hands them to a sink -- and the concrete Zenoh
 * sink is a one-file follow-up when the binding lands. Until then a capturing
 * sink makes the whole driver unit-testable, and a stub sink lets it run.
 *
 * What it does NOT do. It does not know Zenoh, QoS, or the two-plane split: the
 * driver picks the keyexpr, the eventual Zenoh sink opens the RT session with
 * common/zenoh/session_config.h. Keeping this a pure abstract class means this
 * header, like everything the driver links, pulls in nothing from zenoh or ROS.
 */

#ifndef SENSOR__PUBLISH_SINK_H_
#define SENSOR__PUBLISH_SINK_H_

#include <string>

namespace sensor {

// One RT-plane publish. keyexpr is the full key (xbrain/{rid}/rt/gnss/heading);
// payload is the JSON message (envelope + data). Implementations must not throw
// on the hot path -- a transport error is logged and dropped, never propagated
// into the 20 Hz loop.
class PublishSink {
 public:
  virtual ~PublishSink() = default;
  virtual void publish(const std::string& keyexpr, const std::string& payload) = 0;
};

}  // namespace sensor

#endif  // SENSOR__PUBLISH_SINK_H_
