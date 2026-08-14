/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: zenoh_sink.cc
 * Brief: ZenohSink implementation (see zenoh_sink.h)
 *
 * Description:
 * The only translation unit in rtk_driver that includes zenoh.hxx. It builds the
 * RT-plane session config from session_config.h (NOT hand-written here, so the
 * cxx-vs-python parity stays intact), opens the session, and publishes with a
 * per-key publisher declared once and reused. publish() catches every zenoh error
 * because PublishSink forbids throwing on the 20 Hz path.
 */

#define ZENOHCXX_ZENOHC  // select the zenoh-c backend for zenoh-cpp

#include "sensor/zenoh_sink.h"

#include <cstdio>
#include <map>

#include "xbrain/zenoh/session_config.h"
#include "zenoh.hxx"

namespace sensor {

namespace {
// Build the RT-plane session from the shared config emitter and open it. Kept a
// free function so the Impl ctor initialiser can hand the opened session straight
// into the member (zenoh::Session is not default-constructible).
zenoh::Session OpenRtSession() {
  namespace zc = hachist::xbrain::zenoh_config;
  const std::string json5 = zc::ToJson5(zc::PlaneConfig(zc::Plane::kRt));
  auto config = zenoh::Config::from_str(json5);
  return zenoh::Session::open(std::move(config));
}
}  // namespace

struct ZenohSink::Impl {
  zenoh::Session session;
  // One publisher per key (heading / fix / clock), declared on first use. A map
  // keeps the declaration out of the hot path after the first tick per key.
  std::map<std::string, zenoh::Publisher> pubs;

  Impl() : session(OpenRtSession()) {}
};

ZenohSink::ZenohSink() {
  try {
    impl_ = std::make_unique<Impl>();
  } catch (const std::exception& e) {
    // Startup failure is fatal and MUST surface (unlike a hot-path publish).
    throw std::runtime_error(std::string("ZenohSink: cannot open RT session: ") + e.what());
  }
}

ZenohSink::~ZenohSink() = default;

void ZenohSink::publish(const std::string& keyexpr, const std::string& payload) {
  // Never throw on the hot path (PublishSink contract): a transport error is
  // logged in English and dropped so the 20 Hz loop keeps running.
  try {
    auto it = impl_->pubs.find(keyexpr);
    if (it == impl_->pubs.end()) {
      it = impl_->pubs
               .emplace(keyexpr, impl_->session.declare_publisher(zenoh::KeyExpr(keyexpr)))
               .first;
    }
    it->second.put(zenoh::Bytes(payload));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "rtk_driver: zenoh publish dropped on %s: %s\n",
                 keyexpr.c_str(), e.what());
  }
}

}  // namespace sensor
