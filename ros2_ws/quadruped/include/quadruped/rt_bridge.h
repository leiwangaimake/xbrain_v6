/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_bridge.h
 * Brief: RT plane in, process out -- the routing, with no transport in it
 *
 * Description:
 * Which arriving message causes which call, and what gets acked. That is all.
 *
 * IT HAS NO ZENOH IN IT, and that is the design. The bridge needs rt_parse, the
 * process seams and somewhere to send bytes -- none of which is a transport. So
 * it lives in quadruped_core, is built on every machine, and its tests run with
 * no router: a case like "an estop with truncated JSON still stops the robot"
 * is exercised by calling HandleEstop with those bytes. rt_session.cc does
 * nothing but hand zenoh's callbacks to these four functions.
 *
 * The publish side is INJECTED for the same reason. A test captures the acks
 * and asserts on them, instead of inferring from a counter that the right thing
 * was probably sent.
 *
 * THE ASYMMETRY SURVIVES THE TRIP (11 S3.0.1). HandleCmdVel and
 * HandleChassisCtrl can refuse; HandleEstop cannot, and its body has no branch
 * that skips the stop. The signatures of rt_parse already make that hard to get
 * wrong, and this layer keeps it that way: the stop is issued BEFORE anything
 * is parsed for the ack.
 *
 * *** WHY A REFUSED cmd_vel IS COUNTED AND REPORTED RATHER THAN DROPPED.
 * cmd_vel arrives at 20 Hz. A publisher that gets one field wrong produces a
 * robot that never moves and a process that looks perfectly healthy -- the link
 * is up, the chassis is reporting, Tier 1 says "timeout" exactly as it would if
 * nothing were publishing at all. That case is live today: p1_motion does not
 * yet send the mandatory estop_epoch (11:1722, 13 RX-3), so every one of its
 * commands is refused. first_refusal() exists so the operator is told WHY, once,
 * instead of being left to compare two indistinguishable symptoms.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_BRIDGE_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_BRIDGE_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "quadruped/process.h"
#include "quadruped/rt_parse.h"

namespace quadruped {
namespace rt {

// 13 S9.12.6 / 11 S9.12.6: repeated stops inside this window are swallowed.
// They do NOT advance the generation and do NOT raise an event -- and they are
// still ACKED, because a sender that gets no answer retries, which is the event
// storm the window exists to prevent.
inline constexpr double kEstopDedupS = 0.050;

class RtBridge {
 public:
  // Where an outbound message goes. `key_suffix` is the part after the rid --
  // "rt/chassis/ctrl/ack" and so on -- so this class never builds a full key
  // and cannot accidentally publish onto another robot's.
  using PublishFn =
      std::function<bool(const std::string& key_suffix, const char* data,
                         std::size_t len)>;

  RtBridge(QuadrupedProcess* proc, std::string rid, std::string boot,
           PublishFn publish);

  // ---- the four inbound handlers ---------------------------------------
  //
  // Called on Zenoh threads in production and directly from tests. They take
  // the clock as an argument for the same reason CtrlTick does: a 50 ms dedup
  // window is then exercised in microseconds.

  // LOOSENING. Refused messages change nothing and are counted.
  void HandleCmdVel(double now_mono_s, const char* data, std::size_t len);

  // LOOSENING. Every outcome is acked, including refusals -- an ack that only
  // appears on success leaves the sender unable to tell "refused" from "lost".
  void HandleChassisCtrl(double now_mono_s, const char* data, std::size_t len);

  // TIGHTENING. Always stops. Returns void because there is no outcome a caller
  // could act on differently (11 S3.0.1).
  void HandleEstop(double now_mono_s, const char* data, std::size_t len);

  // Answers on rt/safety/probe/pong. 13 Q-4: the reply is driven by the ping,
  // never by a timer of our own -- a timer would keep answering after the
  // subscription died, which is the one thing this probe exists to detect.
  void HandlePing(double now_mono_s, const char* data, std::size_t len);

  // ---- outbound ----------------------------------------------------------
  //
  // One state publish, from a snapshot ctrl produced. Returns false when there
  // was nothing new to send, which is the ordinary case for a publisher running
  // faster than the producer -- not an error.
  //
  // *** It carries estop_epoch, and that is the point of this function today.
  // 11:1722 makes the field mandatory on rt/motion/cmd_vel, and p1_motion
  // cannot echo a generation it has never been told; 13 RX-3 and NEXT.md
  // P7.3 (7) both record the dependency as "quadruped publishes it first".
  // This is that publish.
  bool PublishState(const QuadrupedProcess::StateSnapshot& snap);

  // ---- observables -------------------------------------------------------
  std::uint64_t cmd_vel_accepted() const { return cmd_ok_; }
  std::uint64_t cmd_vel_refused() const { return cmd_refused_; }
  std::uint64_t ctrl_accepted() const { return ctrl_ok_; }
  std::uint64_t ctrl_refused() const { return ctrl_refused_; }
  std::uint64_t estops_applied() const { return estop_applied_; }
  std::uint64_t estops_deduped() const { return estop_deduped_; }
  std::uint64_t pongs_sent() const { return pongs_; }
  std::uint64_t acks_sent() const { return acks_; }
  std::uint64_t states_published() const { return states_; }

  // Why the FIRST refused cmd_vel was refused, kOk while none has been. See the
  // file comment: at 20 Hz the difference between "refused" and "nobody is
  // publishing" is invisible without this.
  RtParse first_refusal() const { return first_refusal_; }

 private:
  bool Publish(const std::string& suffix, const char* data, std::size_t len);

  QuadrupedProcess* proc_;
  std::string rid_;
  std::string boot_;
  PublishFn publish_;

  double last_estop_mono_s_ = -1.0;

  std::uint64_t cmd_ok_ = 0;
  std::uint64_t cmd_refused_ = 0;
  std::uint64_t ctrl_ok_ = 0;
  std::uint64_t ctrl_refused_ = 0;
  std::uint64_t estop_applied_ = 0;
  std::uint64_t estop_deduped_ = 0;
  std::uint64_t pongs_ = 0;
  std::uint64_t acks_ = 0;
  std::uint64_t states_ = 0;
  RtParse first_refusal_ = RtParse::kOk;
};

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_BRIDGE_H_
