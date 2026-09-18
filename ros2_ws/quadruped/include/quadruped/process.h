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
 *               riding along (TX-5), and the odometry INTEGRATION. Everything
 *               with a deadline.
 *
 *               It does NOT publish the odometry (13 S9.1 ctrl row, v1.11, and
 *               13 V-69). Publishing a nav_msgs/Odometry through rclcpp
 *               allocates -- measured, see test_uplink_alloc.cc -- and QD-7
 *               forbids allocation here. A malloc on this thread can queue
 *               behind an ordinary-priority thread holding the glibc arena
 *               lock, which is priority inversion on the thread carrying the
 *               200 ms Tier 1 deadline. The integration stays because it is a
 *               time accumulation tied to the control period; only the publish
 *               leaves, through the slot below.
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
#include <functional>
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

  // ---- what rt_pub takes (13 S9.1 rt_pub row, v1.11 / V-69) -------------
  //
  // The newest integrated sample, or false when ctrl has not produced a new
  // one since the last call. The publisher thread lives outside this class --
  // it needs rclcpp, and quadruped_core must stay ROS-free (CLAUDE.md 5.3,
  // because chassis_relay is on the emergency-stop path and consumes it).
  //
  // A slot and not a queue: a publisher that fell behind must send the CURRENT
  // pose, not work through a backlog of stale ones. That is RTC-6, and for an
  // odometry a stale sample is a QD-5 violation rather than a latency figure.
  bool TakeOdomForPublish(OdomSample* out);
  // Diagnostics for the mode sequence; see OnChassisMode.
  std::uint64_t mode_steps() const { return mode_steps_; }
  bool mode_sequence_pending() const {
    return mode_want_state_ || mode_want_gait_ || mode_want_usage_;
  }

  // What rt_pub needs to build a RobotState, as a POD.
  //
  // It carries only the fields CTRL owns. The chassis's own reports are NOT
  // here and cannot be: BasicStatus and MotionStatus hold std::string (model,
  // version, serial), so they are not trivially copyable and LockfreeSlot
  // refuses them -- which is the compiler enforcing 12 RTC-6 rather than a
  // preference. Forwarding those four report streams is the rx thread's job,
  // on its own ordinary-priority thread where allocating is allowed.
  //
  // Consequence, stated rather than discovered: rt/chassis/state published from
  // this snapshot carries NO basic/motion block yet (WriteRobotState accepts
  // null for both). What it does carry is estop_epoch, which is the field
  // p1_motion is waiting on (11:1722 / 13 RX-3).
  struct StateSnapshot {
    chs_a::ConnState conn = chs_a::ConnState::kProbing;
    Tier1Output tier1;
    std::uint64_t estop_epoch = 0;
    // Milliseconds, as 11 S4.1 names the field. Negative means no command has
    // arrived, which is reported as null rather than as a very large age.
    double cmd_age_ms = -1.0;
    // 13 S9.12.2 (3): the last command echoed a generation other than ours.
    // NOT a lock -- it clears by itself when the upstream catches up.
    bool soft_estop_active = false;
    bool mode_switching = false;
    bool motion_allowed = false;
  };

  // The newest snapshot, or false when ctrl has not produced one since the last
  // call. Same slot discipline as the odometry: newest wins, no backlog.
  bool TakeStateForPublish(StateSnapshot* out);

  // ---- the four report streams (13 S7.1) --------------------------------
  //
  // Called from RxPump, on the chs_a_rx thread, with the report that was just
  // parsed and nullptr for the others. It is a callback rather than a slot
  // because these structs hold std::string and cannot cross a LockfreeSlot --
  // and because there is nothing to hand to ctrl anyway: forwarding a report is
  // not realtime work, and it belongs on the thread that already did the
  // allocating parse (QD-7's whole reason for that thread existing).
  //
  // It runs INSIDE RxPump, so a sink that blocks stalls report reception, and a
  // stalled reception is what the session reads as a dead link (13 CA-7). The
  // sink in production is one zenoh put.
  using ReportSink = std::function<void(double now_mono_s,
                                        const chs_a::BasicStatus*,
                                        const chs_a::MotionStatus*,
                                        const chs_a::DeviceStatus*,
                                        const chs_a::FaultReport*)>;
  void SetReportSink(ReportSink sink);

  // ---- channel two, the domain-0 sources (13 S4.2) ----------------------
  //
  // Called from the chs_b thread (SCHED_FIFO 70), so both go through a
  // lock-free slot exactly as chs_a_rx's snapshot does -- same thread boundary,
  // same rule (12 RTC-6).
  //
  // 13 S4.2's priority tables, and how each half is decided:
  //
  //   ANGULAR  1) /IMU 200 Hz  2) the monitor protocol's OmegaZ, 10 Hz, "IMU
  //            失效时降级". The threshold is CONFIGURED -- chassis_dds
  //            .imu_age_warn_ms, whose own comment reads "older than this ->
  //            yaw falls back to the 10 Hz source" -- so it is read, not
  //            invented.
  //   LINEAR   1) /MOTION_INFO 20 Hz  2) the monitor protocol's LinearX/Y,
  //            10 Hz. There is NO configured threshold for this one, and
  //            CLAUDE.md 3.1 forbids inventing a safety parameter here, so the
  //            rule is "the NEWER sample wins": at 20 Hz against 10 Hz the
  //            drdds source wins nearly every period (which is the priority the
  //            table asks for), and if it dies the monitor reports start
  //            winning by themselves. No constant, no silent default.
  //
  // 13 S4.2 also bans a third source outright: the axis command read back as
  // velocity. Using a commanded value as feedback is open loop pretending to be
  // closed loop, and there is deliberately no entry point for it here.
  void OnImu(double now_mono_s, double wz);
  void OnMotionInfo(double now_mono_s, double vx, double vy);

  // Which source fed the odometry on the last control period. Published so the
  // degradation is VISIBLE: the covariance model does not know the difference
  // between a 200 Hz yaw and a 10 Hz one, so nothing downstream would show it.
  enum class OdomSource { kNone, kMonitor, kDrdds };
  OdomSource linear_source() const { return linear_src_; }
  OdomSource angular_source() const { return angular_src_; }

  // How many control periods each source fed. Reported instead of the
  // instantaneous winner because the two linear sources run at 20 Hz and 10 Hz
  // and the winner alternates -- measured on the chassis 2026-09-17 -- so a
  // line that printed the winner would flap, and a diagnostic that flaps is a
  // diagnostic that gets ignored. The counts do not flap and they answer the
  // question that matters: is the 20 Hz source contributing at all.
  std::uint64_t linear_from_drdds() const { return lin_drdds_; }
  std::uint64_t linear_from_monitor() const { return lin_monitor_; }
  std::uint64_t angular_from_drdds() const { return ang_drdds_; }
  std::uint64_t angular_from_monitor() const { return ang_monitor_; }
  std::uint64_t reports_forwarded() const { return reports_forwarded_; }
  // How many integrated samples ctrl has offered. Compared against what the
  // publisher actually sent, the difference is the overrun rate -- the number
  // T-ODOM-1 is about, and the one V-69 put on the table.
  std::uint64_t odom_offered() const { return odom_offered_; }

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

  // A discrete chassis action from rt/chassis/ctrl (11 S9.3.3): stand, prone,
  // a gait or a usage mode. Routed through the mode machine, which owns the
  // read-back window (MS-1/MS-2) and the stair precondition of 13 PR-1 --
  // none of which the caller should be re-deciding.
  //
  // Returns the machine's verdict so the ack can carry it. An ack that said
  // "accepted" for a request the machine refused would be worse than no ack:
  // 11 CR-12 is explicit that "ack = accepted" does not mean the lock cleared,
  // and an operator who cannot trust the refusal either has nothing left.
  // 11 S9.2.4: a whole mode triple, applied as a SEQUENCE. Returns false when
  // a switch is already in flight (13 MS-3), which the caller reports rather
  // than queueing -- two pending triples have no defined read-back expectation.
  //
  // Why a sequence and not one Request: 13 MS-5 records that the chassis
  // couples the three (a gait switch moves the motion mode, and the reverse),
  // and MS-3 refuses a second switch while one is in flight. So the triple has
  // to be walked one step at a time, each step waiting for its read-back.
  bool OnChassisMode(bool has_usage_mode, std::int64_t usage_mode,
                     bool has_motion_state, std::int64_t motion_state,
                     bool has_gait, std::int64_t gait);

  ModeRequestResult OnChassisAction(double now_mono_s, ModeAction action,
                                    std::int64_t param);

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
  // ctrl -> rt_pub. The same mechanism as chs_a_rx -> ctrl above, for the same
  // reason (RTC-6): it is the only cross-thread hand-off that cannot block the
  // realtime side and cannot accumulate stale samples.
  hachist::xbrain::rtcomm::LockfreeSlot<OdomSample> odom_slot_;
  hachist::xbrain::rtcomm::LockfreeSlot<StateSnapshot> state_slot_;

  // chs_b -> ctrl. Trivially copyable by requirement, like every other slot
  // payload in this class.
  struct ImuTick {
    double wz = 0.0;
    double rx_mono_s = -1.0;
  };
  struct MotionInfoTick {
    double vx = 0.0;
    double vy = 0.0;
    double rx_mono_s = -1.0;
  };
  hachist::xbrain::rtcomm::LockfreeSlot<ImuTick> imu_slot_;
  hachist::xbrain::rtcomm::LockfreeSlot<MotionInfoTick> mi_slot_;
  ImuTick last_imu_;
  MotionInfoTick last_mi_;
  // When the MONITOR velocity last arrived. Distinct from the snapshot's
  // rx_mono_s, which every report refreshes whether or not it carries one.
  double last_monitor_vel_s_ = -1.0;
  OdomSource linear_src_ = OdomSource::kNone;
  OdomSource angular_src_ = OdomSource::kNone;
  std::uint64_t lin_drdds_ = 0, lin_monitor_ = 0;
  std::uint64_t ang_drdds_ = 0, ang_monitor_ = 0;
  std::uint64_t odom_offered_ = 0;
  ReportSink report_sink_;
  std::uint64_t reports_forwarded_ = 0;
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
  // 11 S9.2.4 pending mode triple, walked one step per control period.
  // ORDER (see StepModeSequence): motion_state, then gait, then usage_mode.
  // 13 MS-5 fixes the first two; usage_mode goes LAST on purpose -- it is the
  // gate Tier 1 opens on (NAV-111), so opening it only after the posture and
  // gait have read back keeps the permissive step behind the others.
  bool mode_want_usage_ = false;
  std::int64_t mode_usage_ = 0;
  bool mode_want_state_ = false;
  std::int64_t mode_state_ = 0;
  bool mode_want_gait_ = false;
  std::int64_t mode_gait_ = 0;
  // Counts steps actually dispatched, so "the sequence ran" is a number.
  std::uint64_t mode_steps_ = 0;

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
