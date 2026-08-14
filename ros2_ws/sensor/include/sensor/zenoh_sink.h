/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: zenoh_sink.h
 * Brief: The concrete RT-plane PublishSink -- zenoh-cpp, hidden behind pImpl
 *
 * Description:
 * The real transport for rtk_driver: opens ONE RT-plane Zenoh session (peer,
 * connect tcp/127.0.0.1:7449, multicast off, gossip on -- exactly what
 * common/zenoh/session_config.h emits for Plane::kRt) and publishes each message
 * the driver hands it. zenoh-cpp lives entirely in the .cc via pImpl, so this
 * header -- and therefore rtk_driver_core and every offline test -- pulls in
 * NOTHING from zenoh: only the executable link (which sets ZENOHCXX_ZENOHC and
 * links libzenohc) ever sees a zenoh type. That is what keeps the driver logic
 * unit-testable against a CaptureSink with no binding installed.
 *
 * The publish() override honours the PublishSink contract: it NEVER throws on the
 * hot path. A transport error is caught, logged in English, and dropped -- the
 * 20 Hz loop must not die because one datagram failed to leave.
 */
#ifndef SENSOR__ZENOH_SINK_H_
#define SENSOR__ZENOH_SINK_H_

#include <memory>
#include <string>

#include "sensor/publish_sink.h"

namespace sensor {

class ZenohSink : public PublishSink {
 public:
  // Opens the RT-plane session. THROWS (std::runtime_error) if the session
  // cannot open -- a driver that cannot reach the RT router must fail at startup,
  // not silently publish into the void.
  ZenohSink();
  ~ZenohSink() override;

  ZenohSink(const ZenohSink&) = delete;
  ZenohSink& operator=(const ZenohSink&) = delete;

  void publish(const std::string& keyexpr, const std::string& payload) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sensor

#endif  // SENSOR__ZENOH_SINK_H_
