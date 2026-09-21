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
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
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
  // 13 S7.1 Q-5: the four chassis report streams, each onto its own key.
  // Called from the chs_a_rx thread via QuadrupedProcess::SetReportSink --
  // these structs hold std::string and cannot cross a lock-free slot, and
  // forwarding them is not realtime work.
  void PublishReports(double now_mono_s, const chs_a::BasicStatus* basic,
                      const chs_a::MotionStatus* motion,
                      const chs_a::DeviceStatus* device,
                      const chs_a::FaultReport* fault);

  // 11 S9.1.4 / 13 ASM-4 (2): answer the upstream's handshake. Without it
  // 10 S3.3 Stage 1 never completes at this process.
  void HandleHello(double now_mono_s, const char* data, std::size_t len);

  // The transport facts 13 CB-4 / DDS-9 / TF-1 require in hello_ack.runtime.
  // Set once from main after the config is loaded; strings are copied because
  // the handshake can be answered long after the caller's buffers are gone.
  void SetTransport(const std::string& endpoint, const std::string& codebook,
                    int chassis_dds_domain, int uplink_ros_domain,
                    const std::string& imu_frame_id, bool drdds_available);
  // 11 S9.6 spec block, from the resolved config.
  void SetSpec(bool holonomic, double max_vx, double max_vy, double max_wz);

  // 11 S9.4.1 rt/chassis/light. Subscribed, never published: p1_motion
  // forwards it from cmd/chassis/light (11 P1-8).
  void HandleLight(double now_mono_s, const char* data, std::size_t len);

  void HandleCmdVel(double now_mono_s, const char* data, std::size_t len);

  // LOOSENING. Every outcome is acked, including refusals -- an ack that only
  // appears on success leaves the sender unable to tell "refused" from "lost".
  void HandleChassisCtrl(double now_mono_s, const char* data, std::size_t len);
  // 11 S9.2.4 rt/chassis/mode. No ack key exists for this one (the contract
  // registers rt/chassis/mode alone); the result is observable as the triple
  // in rt/chassis/state, which is what 13 MS-1 compares against anyway.
  void HandleChassisMode(double now_mono_s, const char* data, std::size_t len);

  // TIGHTENING. Always stops. Returns void because there is no outcome a caller
  // could act on differently (11 S3.0.1).
  void HandleEstop(double now_mono_s, const char* data, std::size_t len);

  // Answers on rt/safety/probe/pong. 13 Q-4: the reply is driven by the ping,
  // never by a timer of our own -- a timer would keep answering after the
  // subscription died, which is the one thing this probe exists to detect.
  void HandlePing(double now_mono_s, const char* data, std::size_t len);
  // 13 Q-5 / A2: rt/clock/status, the subscription that was declared in
  // rt_keys.cc from the start and never handled -- every envelope's ts_sync
  // was the PB-Q3 default (false) because nothing ever fed it. Stores the
  // latest sync verdict and its arrival time; produces no reply.
  void HandleClockStatus(double now_mono_s, const char* data, std::size_t len);

  // The envelope's ts_sync as of `now_mono_s`: the last received
  // ClockStatus.sync, aged out to false after kClockSyncTimeoutS (11 CLK-A3,
  // monotonic), false before the first message ever arrives. Public and
  // time-injected so the aging half is assertable without sleeping --
  // Publish itself reads the real clock.
  bool TsSyncAt(double now_mono_s) const;

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
  // Mode-triple counters. Separate from the ctrl ones because a refused mode
  // switch and a refused stand have different causes and different fixes.
  std::uint64_t mode_accepted() const { return mode_ok_; }
  std::uint64_t mode_refused() const { return mode_refused_; }
  // Accepted: parsed, and past 13 V-47. Counted BEFORE the send and separately
  // from it -- 13 ASM-6 is what happens when one counter stands for both
  // "accepted" and "went out".
  std::uint64_t lights_accepted() const { return light_accepted_; }
  // Refused: malformed, or carrying `illumination` (13 V-47). Distinct from a
  // send failure, because an operator watching a lamp that stayed dark needs
  // to know whether the message was rejected or the link was.
  std::uint64_t lights_refused() const { return light_refused_; }
  std::uint64_t light_send_failures() const { return light_send_failed_; }
  std::uint64_t hello_answered() const { return hello_ok_; }
  std::uint64_t hello_refused() const { return hello_refused_; }
  std::uint64_t ctrl_refused() const { return ctrl_refused_; }
  std::uint64_t estops_applied() const { return estop_applied_; }
  std::uint64_t estops_deduped() const { return estop_deduped_; }
  std::uint64_t pongs_sent() const { return pongs_; }
  std::uint64_t acks_sent() const { return acks_; }
  std::uint64_t envelope_overflows() const { return envelope_overflows_; }
  std::uint64_t states_published() const { return states_; }

  // Why the FIRST refused cmd_vel was refused, kOk while none has been. See the
  // file comment: at 20 Hz the difference between "refused" and "nobody is
  // publishing" is invisible without this.
  RtParse first_refusal() const { return first_refusal_; }

 private:
  bool Publish(const std::string& suffix, const char* data, std::size_t len);
  std::uint64_t NextSeq(const std::string& suffix);

  QuadrupedProcess* proc_;
  std::string rid_;
  std::string boot_;
  PublishFn publish_;

  double last_estop_mono_s_ = -1.0;

  std::uint64_t cmd_ok_ = 0;
  std::uint64_t cmd_refused_ = 0;
  std::uint64_t ctrl_ok_ = 0;
  std::uint64_t mode_ok_ = 0;
  std::uint64_t light_accepted_ = 0;
  std::uint64_t light_refused_ = 0;
  std::uint64_t light_send_failed_ = 0;
  std::uint64_t hello_ok_ = 0;
  std::uint64_t hello_refused_ = 0;
  // model / version arrive on the chs_a_rx thread (PublishReports) and are
  // read on the zenoh subscription thread (HandleHello). std::string, so a
  // lock-free slot is not available (12 RTC-6) -- a mutex is correct here:
  // neither thread is realtime, and the critical section is two string
  // assignments.
  mutable std::mutex chassis_id_mu_;
  std::string chassis_model_;
  std::string chassis_version_;
  // 11 S4.2 PowerState is assembled from three different reports, so the two
  // that are not the trigger are cached here. Same mutex and the same reason:
  // BasicStatus holds std::string and cannot cross a lock-free slot (12 RTC-6),
  // and neither thread touching this is realtime.
  chs_a::BasicStatus last_basic_;
  bool have_basic_ = false;
  double last_remain_mile_km_ = 0.0;
  // Monotonic deadline for the next PowerState. Negative means none has been
  // published yet, so the first device report publishes immediately rather
  // than waiting out a period the process has not lived through.
  double power_next_s_ = -1.0;
  // The last triple this bridge published, remembered for hello_ack (answered
  // on the subscription thread, which cannot consume from the state slot).
  bool last_has_triple_ = false;
  std::int64_t last_usage_mode_ = 0;
  std::int64_t last_motion_state_ = 0;
  std::int64_t last_gait_ = 0;
  // Transport / spec: written once before Start, read in HandleHello.
  std::string tp_endpoint_;
  std::string tp_codebook_;
  std::string tp_imu_frame_;
  int tp_chs_b_domain_ = -1;
  int tp_uplink_domain_ = -1;
  bool tp_drdds_ = false;
  bool spec_holonomic_ = false;
  double spec_max_vx_ = 0.0;
  double spec_max_vy_ = 0.0;
  double spec_max_wz_ = 0.0;
  std::uint64_t mode_refused_ = 0;
  std::uint64_t ctrl_refused_ = 0;
  std::uint64_t estop_applied_ = 0;
  std::uint64_t estop_deduped_ = 0;
  std::uint64_t pongs_ = 0;
  std::uint64_t acks_ = 0;
  // ClockStatus intake (13 Q-5). Two atomics rather than one struct under a
  // mutex: written on the zenoh thread, read on every publish from rt_pub
  // and chs_a_rx, and the worst interleaving -- a fresh flag read beside the
  // previous arrival time, or the reverse -- lasts one publish and errs
  // toward false (the aged-out side) or extends a 1 Hz verdict by
  // microseconds. Neither is worth a lock on the publish path.
  std::atomic<double> clock_rx_mono_{-1.0};
  std::atomic<bool> clock_sync_{false};
  std::uint64_t clock_accepted_ = 0;
  std::uint64_t clock_refused_ = 0;
  // 11 S3.0's per-key sequence, and the count of payloads too big to wrap.
  // The latter is published rather than swallowed for the reason every other
  // counter here exists: a message that never went out and a message nobody
  // subscribed to look identical from outside.
  std::map<std::string, std::uint64_t> seq_;
  mutable std::mutex seq_mu_;
  std::uint64_t envelope_overflows_ = 0;
  std::uint64_t states_ = 0;
  RtParse first_refusal_ = RtParse::kOk;
};

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_BRIDGE_H_
