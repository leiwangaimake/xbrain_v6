/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: process.h
 * Brief: The assembly -- the layers wired into threads that actually run
 *
 * Description:
 * Every batch before this one ended its commit message with "no production call
 * site". This is the call site. The layers were built to be assembled, and this
 * is where the socket, the framer, the codec, the session, Tier 1, the mode
 * machine and the odometry become one running process.
 *
 * The assembly rules are 13 S9.4 ASM-1..ASM-5, which were written FROM this
 * file rather than before it: S9.1 gave the thread table, S3 gave Tier 1 and
 * S2.2 gave channel one, and nothing said how the layers join. The defect that
 * gap produced is recorded in ASM-1.
 *
 * Two threads, from the table in 13 S9.1:
 *
 *   ctrl        SCHED_FIFO 80, 100 Hz. Tier 1, the axis command, the heartbeat
 *               riding along (TX-5), and the odometry integration. Everything
 *               with a deadline.
 *   chs_a_rx    ordinary priority, event driven. Reads the socket, reassembles
 *               frames, parses JSON. The parse is HERE and not in ctrl, which
 *               is the whole reason the split exists: JSON parsing allocates,
 *               and QD-7 forbids allocation on the realtime path.
 *
 * They meet at one lock-free slot carrying a POD snapshot, which is the only
 * cross-thread mechanism 12 RTC-6 allows. A queue would accumulate stale
 * samples, and a stale sample in an odometry integration is a violation of
 * QD-5 rather than a performance question.
 *
 * The snapshot is a fixed-size struct and not the parsed report, because
 * LockfreeSlot refuses a payload with a heap member -- publishing one would
 * allocate inside the producer, which is the thing the split was arranged to
 * prevent. The compiler enforces it; this comment only explains it.
 *
 * WHAT IS NOT WIRED HERE (13 S9.4 ASM-4), stated so the shape of the process
 * is not overstated:
 * the domain-0 DDS reader (chs_b) and the RT-plane publisher (rt_pub) are built
 * as OPTIONAL targets because they need CycloneDDS and rclcpp, and this class
 * does not start them. The IMU therefore has no source yet and the odometry
 * runs on the monitor protocol's 10 Hz velocity alone. That is a real gap, not
 * a design decision, and it is named rather than hidden behind a stub that
 * would make the process look complete.
 *
 * Testability. CtrlTick and RxPump are public and take the clock as an
 * argument, so a test drives the whole assembly with no threads and no waiting:
 * a 200 ms Tier 1 timeout is exercised in microseconds. The threads are thin
 * loops around them, which is deliberate -- logic inside a thread body is logic
 * no test can reach.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_PROCESS_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_PROCESS_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "quadruped/chassis_socket.h"
#include "quadruped/chs_a_framer.h"
#include "quadruped/chs_a_reports.h"
#include "quadruped/chs_a_session.h"
#include "quadruped/mode_machine.h"
#include "quadruped/odometry.h"
#include "quadruped/quadruped_config.h"
#include "quadruped/tier1.h"
#include "quadruped/tx_owner.h"
#include "xbrain/rtcomm/lockfree_slot.h"

namespace quadruped {

// What chs_a_rx hands to ctrl. Trivially copyable by requirement, not by
// preference: LockfreeSlot refuses anything else, because publishing a payload
// with a heap member would allocate in the producer.
//
// It carries only what ctrl ACTS on. The full-fidelity report stays on the rx
// side for the publisher to forward; copying it here would mean copying strings
// onto the realtime path to satisfy a consumer that does not read them.
struct ChassisSnapshot {
  // Tier 1's inputs (13 S3.2).
  bool hes = false;
  bool sleep = false;
  std::int64_t usage_mode_raw = 0;
  std::int64_t motion_state_raw = 0;
  std::int64_t gait_raw = 0;
  // The odometry's linear source (13 S4.2, the 10 Hz monitor-protocol velocity
  // until the domain-0 reader is wired).
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
  // When WE received it, on the monotonic clock. Not the chassis's stamp: that
  // is another machine's wall clock and cannot be compared with ours (CLK-C4).
  double rx_mono_s = -1.0;
  // Which report this came from, so a consumer can tell "the basic status said
  // the robot is awake" from "the motion status happened not to mention it".
  bool from_basic = false;
  bool from_motion = false;
};

class QuadrupedProcess {
 public:
  QuadrupedProcess(const QuadrupedConfig& cfg);
  ~QuadrupedProcess();

  QuadrupedProcess(const QuadrupedProcess&) = delete;
  QuadrupedProcess& operator=(const QuadrupedProcess&) = delete;

  // ---- the two thread bodies, callable directly ------------------------
  //
  // One control period: session, Tier 1, the axis command, the heartbeat.
  // Everything it needs is passed in or already latched, so a test can run ten
  // thousand periods in a millisecond.
  void CtrlTick(double now_mono_s);

  // Drain the socket once: read, reassemble, parse, publish a snapshot.
  // Returns the number of frames consumed.
  int RxPump(double now_mono_s);

  // ---- the threads -----------------------------------------------------
  //
  // Start applies mlockall and the SCHED_FIFO priorities of 13 S9.1, and
  // reports which of them failed rather than carrying on quietly: under the
  // systemd defaults BOTH fail, and neither prints anything by itself.
  bool Start();
  void Stop();
  bool running() const { return running_.load(std::memory_order_acquire); }

  // Non-zero when mlockall or a priority change was refused. The value is the
  // errno, so a message can say WHICH -- rt_thread.h returns it for that reason.
  int mlock_error() const { return mlock_error_; }
  int ctrl_priority_error() const { return ctrl_priority_error_; }

  // ---- observables, for the self-report and for tests -------------------
  //
  // These read the layers DIRECTLY and are therefore only safe from the thread
  // that drives them -- which means from a test, where nothing else is running.
  // The supervisor thread must use link_status() below instead.
  chs_a::ConnState conn_state() const { return session_.state(); }
  const Tier1Output& last_tier1() const { return last_tier1_; }
  bool motion_allowed() const { return session_.motion_allowed(); }
  std::uint64_t ctrl_ticks() const { return ctrl_ticks_; }
  std::uint64_t heartbeats_sent() const { return heartbeats_sent_; }
  std::uint64_t axis_frames_sent() const { return axis_frames_sent_; }
  std::uint64_t frames_received() const { return frames_received_; }
  std::uint64_t tx_skipped() const { return tx_skipped_; }
  const OdomSample& last_odom() const { return last_odom_; }

  // A cmd_vel arrived from the RT plane. Called by the publisher thread once it
  // exists; a test calls it directly. Taking the values rather than a message
  // keeps this class free of any transport type.
  void OnCmdVel(double now_mono_s, double vx, double vy, double wz,
                std::uint64_t estop_epoch);

  // An operator enable arrived on rt/{rid}/ctrl/enable (13 S3.2 / 11 S9.12.1).
  // It is the ONE thing that clears a Tier 1 lock, and it is consumed by the
  // next control period rather than latched: a latched enable would re-clear
  // the lock every period forever, which makes the lock decorative. 11 S9.12.1
  // is explicit that "the upstream came back" and "the upstream is trusted" are
  // different events, and a level-held flag collapses them into one.
  //
  // This is the other half of the seam OnCmdVel opens, not a hook kept for
  // later: without it the axis-command branch of CtrlTick is unreachable, since
  // Tier 1 locks on the opening silence and nothing could ever unlock it.
  void OnEnable();

  // A soft stop. 13 S9.12.2 (3) and T-1: the generation advances HERE, in the
  // callback, and a zero frame goes out immediately rather than next period --
  // the next period is up to 10 ms away and the budget is 5 ms.
  void OnSoftEstop(double now_mono_s);
  std::uint64_t estop_epoch() const { return estop_epoch_; }

  // What the supervisor thread is allowed to read while ctrl is running.
  //
  // It is published through atomics rather than read off the session because
  // the session's fields are plain members written every control period: a
  // second thread reading them is a data race, and the fact that it would
  // almost always produce the right answer is what makes that kind of race
  // survive review. The cost is three relaxed stores per period.
  struct LinkStatus {
    chs_a::ConnState conn = chs_a::ConnState::kProbing;
    int active_endpoint = -1;       // index into the candidate list, -1 = none
    std::uint64_t probe_cycles = 0; // full walks of the list that found nothing
    std::uint64_t frames = 0;       // reports received since start
  };
  LinkStatus link_status() const;

 private:
  void CtrlLoop();
  void RxLoop();

  QuadrupedConfig cfg_;
  ChassisSocket socket_;
  TxOwner tx_;
  chs_a::Session session_;
  chs_a::Framer framer_;
  Tier1 tier1_;
  Odometry odom_;
  ModeMachine mode_;

  hachist::xbrain::rtcomm::LockfreeSlot<ChassisSnapshot> snapshot_slot_;
  ChassisSnapshot latest_;
  bool have_snapshot_ = false;

  // The last command from above, and the generation it echoed.
  double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;
  double cmd_rx_mono_s_ = -1.0;
  bool have_cmd_ = false;
  // One-shot, cleared by the period that reads it. See OnEnable.
  bool enable_pending_ = false;
  std::uint64_t cmd_estop_epoch_ = 0;
  std::uint64_t estop_epoch_ = 0;

  Tier1Output last_tier1_;
  OdomSample last_odom_;

  std::uint16_t msg_id_ = 0;
  double last_ctrl_s_ = -1.0;

  // Published for the supervisor thread; see link_status(). int rather than the
  // enum because atomic<enum class> is well-formed but reads badly at the use
  // site, and the conversion is checked in one place.
  std::atomic<int> pub_conn_{0};
  std::atomic<int> pub_active_ep_{-1};
  std::atomic<std::uint64_t> pub_probe_cycles_{0};
  std::atomic<std::uint64_t> pub_frames_{0};

  std::atomic<bool> running_{false};
  std::thread ctrl_thread_;
  std::thread rx_thread_;
  int mlock_error_ = 0;
  int ctrl_priority_error_ = 0;

  std::uint64_t ctrl_ticks_ = 0;
  std::uint64_t heartbeats_sent_ = 0;
  std::uint64_t axis_frames_sent_ = 0;
  std::uint64_t frames_received_ = 0;
  std::uint64_t tx_skipped_ = 0;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_PROCESS_H_
