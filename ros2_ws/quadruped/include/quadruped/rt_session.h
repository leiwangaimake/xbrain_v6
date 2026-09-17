/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_session.h
 * Brief: The one Zenoh session quadruped is allowed to hold -- RT plane only
 *
 * Description:
 * Transport, and nothing else. What arrives is handed to rt_parse.cc, which is
 * where the safety rule lives; keeping the two apart is what lets that rule be
 * tested with no router and no network.
 *
 * ONE SESSION, RT PLANE ONLY. 11 RT-C4 forbids this process from holding a
 * general-plane session at all: it is the only process that can reach CHS-A,
 * and a general-plane session would make it a bridge between the plane anyone
 * can publish on and the one that moves the legs. There is no constructor here
 * that can open the general plane -- the endpoint is passed in, but everything
 * else about the configuration is fixed in this file.
 *
 * *** THE CONFIGURATION IS NOT COPIED FROM 11 S1.1.2's CODE BLOCK, AND MUST NOT
 * BE. That block still reads `gossip: { enabled: false }`, which is the
 * SUPERSEDED form: the RT-C2 row in the same section carries a 2026-08-23
 * correction, and configs/zenoh/router_rt.json5 plus
 * xbrain/common/zenoh/session_factory.py carry the corrected values with the
 * measurement that forced them --
 *
 *     peer + gossip off  -> 0 samples received
 *     peer + gossip on   -> samples received
 *
 * because in the deployed hub-and-spoke topology every peer connects only to
 * the local router, peers never see each other directly, and a subscription
 * table reaches a remote publisher ONLY by gossip through that router. A
 * session built from the code block joins the plane, connects, reports no
 * error and receives nothing -- which 13 DDS-9 records as indistinguishable
 * from a dead network.
 *
 * The isolation RT-C1/RT-C2 are written for survives because MULTICAST stays
 * off and gossip.multihop stays false: gossip then propagates only over links
 * already established to the loopback RT router, and cannot reach the general
 * plane's gossip domain.
 *
 * RtSessionConfigJson is public so a test can compare this file's values with
 * session_factory.py's WITHOUT opening a session. Two implementations of one
 * configuration diverge silently otherwise, and the symptom of that divergence
 * is silence.
 *
 * WHERE THE CALLBACKS RUN. On Zenoh's own threads, not on ctrl and not on
 * chs_a_rx. They must therefore do what chs_a_rx does -- parse, then hand a POD
 * across a lock-free slot -- and must never block: a callback that waits stalls
 * the runtime that feeds every other subscription, including the estop.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_SESSION_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_SESSION_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace quadruped {
namespace rt {

// The exact Zenoh configuration this process uses for the RT plane, as a JSON5
// string. Built here rather than read from a file so that a deployment cannot
// quietly acquire a general-plane endpoint or re-enable multicast, and exposed
// so that test_rt_session can hold it beside session_factory.py.
//
// `endpoint` is the single connect endpoint (11 S1.1.2: tcp/127.0.0.1:7449).
std::string RtSessionConfigJson(const std::string& endpoint);

class RtSession {
 public:
  RtSession();
  ~RtSession();

  RtSession(const RtSession&) = delete;
  RtSession& operator=(const RtSession&) = delete;

  // Opens the session. Returns false and fills `err` on failure; a failure here
  // is reported by the caller rather than retried silently, because "the RT
  // plane is not there" and "the RT plane is there and empty" need different
  // answers from an operator.
  bool Open(const std::string& endpoint, std::string* err);
  void Close();
  bool is_open() const;

  // Declares a publisher. Publishers are declared ONCE at startup, never per
  // message: 13 S5.7 requires it, and a publisher declared per message would
  // put a discovery round trip on the path of every state update.
  //
  // Returns a handle, or -1 on failure.
  int DeclarePublisher(const std::string& key, std::string* err);

  // Sends one payload on a previously declared handle. Returns false when the
  // handle is unknown or the session is closed -- never throws, because the
  // callers are periodic and a throw would take the publishing thread out.
  bool Put(int handle, const char* data, std::size_t len);

  // Called on a Zenoh thread with the raw payload. See the file comment: no
  // blocking, no long work, hand off and return.
  using SampleFn = std::function<void(const char* data, std::size_t len)>;

  // Declares a subscriber. The key is a FULL key expression -- this class
  // declares no wildcards and offers no way to ask for one, because 11 RT-C3.b
  // makes a wildcard subscription the technical precondition of the general
  // forwarding that RT-C3 exists to prevent.
  bool DeclareSubscriber(const std::string& key, SampleFn on_sample,
                         std::string* err);

  // Counters, so "it is connected" is a number rather than an impression.
  std::size_t publisher_count() const;
  std::size_t subscriber_count() const;
  std::uint64_t samples_received() const;
  std::uint64_t puts_sent() const;
  std::uint64_t put_failures() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_SESSION_H_
