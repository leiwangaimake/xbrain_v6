/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: process.cc
 * Brief: The assembly (see process.h)
 *
 * Description:
 * The order inside CtrlTick is 13 S9.4 ASM-1, and it is not interchangeable:
 *
 *   1. the snapshot that arrived this period is taken first. An arriving report
 *      is the only evidence the link is alive (13 CA-7), so the session has to
 *      see it before it judges liveness -- ticking first made every report one
 *      period late and would drop a link whose report was sitting unread.
 *   2. the session decides whether there is a link and whether a heartbeat is
 *      due. The heartbeat is what keeps the chassis reporting to this address
 *      at all (13 S2.2).
 *   3. Tier 1 decides the velocity. It runs whether or not the link is up: a
 *      dead link is one of the things it reports (timeout), and skipping it
 *      would leave stop_reason stale exactly when it matters.
 *   4. the axis command goes out only if BOTH Tier 1 produced motion and the
 *      session allows it. Two gates and not one: Tier 1 answers "is this
 *      command safe", the session answers "is there a link to put it on", and
 *      collapsing them would send into a socket that is reconnecting.
 *   5. the odometry INTEGRATES last, on the snapshot taken in step 1, and the
 *      result goes into a slot for rt_pub to publish. 13 S9.1 (v1.11) moved the
 *      publish off this thread because it allocates and this thread is QD-7;
 *      13 V-69 carries the decision and what it cost.
 *
 * The heartbeat rides on this thread rather than a timer of its own (TX-5).
 * CA-2 requires it on the same socket as the axis command, and being on the
 * same THREAD as well is what makes TCP interleaving impossible rather than
 * merely unlikely.
 *
 * Both sends go through TxOwner, which is the single outlet CA-4 requires. The
 * realtime caller skips a period rather than waiting (TX-6), and a skip is
 * counted: at 100 Hz a few are ordinary and a rising rate means the critical
 * section is too long, which is the only way to see that from outside.
 */

#include "quadruped/process.h"

#include <chrono>
#include <cstring>
#include <ctime>

#include "quadruped/chs_a_codec.h"
#include "quadruped/dds_names.h"
#include "quadruped/mono_clock.h"
#include "xbrain/rtcomm/rt_thread.h"

namespace quadruped {
namespace {

namespace rt = hachist::xbrain::rtcomm;


std::int64_t WallNow() { return static_cast<std::int64_t>(::time(nullptr)); }

// Sub-second wall clock, for the ROS header stamp carried on OdomSample.
// WallNow above is SECOND resolution (CHS-A's "Time" field is a formatted
// second string, 13 S2.2), and reusing it here would give all hundred samples
// in a second the same stamp -- downstream would see a 100 Hz stream whose
// timestamps advance in 1 Hz steps, which reads as a stalled publisher.
//
double WallNowSeconds() {
  // WALL-CLOCK-OK(align): a ROS header.stamp, required on every locally
  // produced message by CLK-C3 for cross-host alignment. It labels the sample
  // and decides nothing; every age, period and timeout here is steady_clock.
  const auto d = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(d).count();
}

// 13 S9.1: ctrl is SCHED_FIFO 80, chs_b is 70, everything else ordinary.
// *** No longer a literal. 13 S9.1's table gives 80, and the config key
// realtime.sched_fifo_priority.ctrl carried that number while this constant
// was what the thread actually used -- "填了不生效 = 让设置的人以为改了
// 什么". Kept only as the value the fixture-free unit tests would see if they
// ever built a process without a config, which none do.


}  // namespace

QuadrupedProcess::QuadrupedProcess(const QuadrupedConfig& cfg)
    : cfg_(cfg),
      // The socket is the FrameWriter. TxOwner owns the sending discipline --
      // the single critical section, the bounded short-write completion -- and
      // the socket owns only the system call.
      tx_([this](const std::uint8_t* d, std::size_t n) { return socket_.Send(d, n); },
          cfg.link.partial_send_retry),
      session_(chs_a::SessionConfig::FromLinkConfig(cfg.link),
               [this](const EndpointCandidate& ep) {
                 return socket_.Dial(ep, cfg_.link.tcp_nodelay);
               },
               [this]() { socket_.Close(); },
               [](const EndpointCandidate&) {
                 // 13 TLS-4: credentials are judged before dialling so a
                 // missing certificate costs no probe window. This build has no
                 // TLS at all, so the answer is always no -- and the socket
                 // refuses a tls:true candidate independently, which means a
                 // change here alone cannot turn TLS on by accident.
                 return false;
               }),
      framer_(static_cast<std::size_t>(cfg.link.resync_max_bytes),
              cfg.link.frame_assembly_timeout_ms),
      tier1_(cfg.tier1),
      odom_(cfg.odom, cfg.tier1.limits.holonomic),
      mode_([&cfg] {
        ModeConfig m;
        // *** This lambda used to take cfg and discard it with (void)cfg,
        // hardcoding two numbers and leaving prone_forbidden_gaits EMPTY.
        // ProneAllowed answers !Contains(list, gait), so an empty list made it
        // true for every gait: PR-1 never fired and `prone` was accepted on a
        // staircase. 13 V-54 calls that a safety incident. The config carried
        // the right two gaits the whole time.
        m.switch_timeout_s = cfg.motion.mode_switch_timeout_s;
        m.external_transition_hold_s = cfg.motion.external_transition_hold_s;
        m.prone_forbidden_gaits = cfg.motion.prone_forbidden_gaits;
        // *** The FOURTH field, and it was missed when the other three were
        // wired -- ModeConfig has two gait lists and fixing one of them looks
        // finished. GS-1 stayed dead a batch longer because of it: without
        // this line GaitCommandable returns true for everything and 0x1003
        // goes out to a chassis that can never read it back.
        m.command_forbidden_gaits = cfg.motion.command_forbidden_gaits;
        return m;
      }()) {}

QuadrupedProcess::~QuadrupedProcess() { Stop(); }

void QuadrupedProcess::OnCmdVel(double now_mono_s, double vx, double vy,
                                double wz, std::uint64_t estop_epoch) {
  cmd_vx_ = vx;
  cmd_vy_ = vy;
  cmd_wz_ = wz;
  cmd_rx_mono_s_ = now_mono_s;
  cmd_estop_epoch_ = estop_epoch;
  have_cmd_ = true;
}

void QuadrupedProcess::OnEnable() { enable_pending_ = true; }

bool QuadrupedProcess::OnChassisMode(bool has_usage_mode,
                                     std::int64_t usage_mode,
                                     bool has_motion_state,
                                     std::int64_t motion_state, bool has_gait,
                                     std::int64_t gait) {
  // 13 MS-3: one switch in flight at a time. Refusing a second triple rather
  // than overwriting the pending one -- overwriting would abandon a read-back
  // expectation the machine is still waiting on, and MS-2 would then time out
  // on a switch nobody is waiting for any more.
  if (mode_sequence_pending() || mode_.mode_switching()) return false;
  mode_want_usage_ = has_usage_mode;
  mode_usage_ = usage_mode;
  mode_want_state_ = has_motion_state;
  mode_state_ = motion_state;
  mode_want_gait_ = has_gait;
  mode_gait_ = gait;
  // Nothing is dispatched here. The steps go out from the control period, on
  // the thread that owns the mode machine and the chassis socket -- dispatching
  // from the zenoh callback thread would be a data race on both.
  return true;
}

bool QuadrupedProcess::SendModeFrame(ModeAction action, std::int64_t param,
                                     TxCaller caller) {
  // *** THE FRAME. Before this existed (13 ASM-6) ModeMachine::Request
  // returned accepted, switching_ went true, the read-back was waited for --
  // and nothing was ever sent to the chassis. Measured on the bench:
  // `stand` acked "accepted" while the chassis reported MotionState 0
  // throughout, so the robot could not be made to move and every layer looked
  // healthy.
  //
  // ONE function for BOTH paths (the rt/chassis/mode sequencer and the
  // rt/chassis/ctrl stand/prone). The first fix wired only the sequencer, and
  // the ctrl path stayed silent -- the robot stood up and then would not lie
  // down, which is the same defect a second time in the same hour.
  //
  // Through tx_, the same send seam as the axis command, so TX-2 / TX-3 hold
  // for mode frames too: a mode switch must not interleave with the zero frame
  // the estop callback sends.
  std::uint8_t buf[512];
  std::size_t n = 0;
  switch (action) {
    case ModeAction::kStand:
      n = chs_a::EncodeMotionState(buf, sizeof(buf), msg_id_++, WallNow(),
                                   static_cast<int>(kCommandMotionStateStand));
      break;
    case ModeAction::kProne:
      n = chs_a::EncodeMotionState(buf, sizeof(buf), msg_id_++, WallNow(),
                                   static_cast<int>(kCommandMotionStateProne));
      break;
    case ModeAction::kRlControl:
      n = chs_a::EncodeMotionState(
          buf, sizeof(buf), msg_id_++, WallNow(),
          static_cast<int>(kCommandMotionStateRlControl));
      break;
    case ModeAction::kSetGait:
      n = chs_a::EncodeGait(buf, sizeof(buf), msg_id_++, WallNow(),
                            static_cast<std::uint32_t>(param));
      break;
    case ModeAction::kSetUsageMode:
      n = chs_a::EncodeUsageMode(buf, sizeof(buf), msg_id_++, WallNow(),
                                 static_cast<int>(param));
      break;
  }
  // A zero-length encode is a defect in the value, not a transient: the
  // encoders return 0 only for a buffer too small or a value that would make
  // invalid JSON. Sending nothing while still waiting for the read-back
  // reproduces ASM-6 exactly, so the caller is told and mode_switching times
  // out through MS-2 -- the path that reports a failed switch.
  if (n == 0) return false;
  if (tx_.Send(caller, buf, n) != TxResult::kSent) return false;
  ++mode_frames_sent_;
  return true;
}

bool QuadrupedProcess::SendLightFrame(bool custom_mode,
                                     const chs_a::LedSetting& head,
                                     const chs_a::LedSetting& tail) {
  std::uint8_t buf[512];
  const std::size_t n = chs_a::EncodeCustomLight(
      buf, sizeof(buf), msg_id_++, WallNow(), custom_mode, head, tail);
  // A zero-length encode means the buffer was too small for this message --
  // a defect, not a transient. Reporting false lets the caller count it
  // instead of incrementing a "sent" counter for a frame nobody sent, which
  // is the shape 13 ASM-6 recorded.
  if (n == 0) return false;
  if (tx_.Send(TxCaller::kNonRealtime, buf, n) != TxResult::kSent) return false;
  ++light_frames_sent_;
  return true;
}

ModeRequestResult QuadrupedProcess::OnChassisAction(double now_mono_s,
                                                    ModeAction action,
                                                    std::int64_t param) {
  // Straight through to the machine. There is deliberately no pre-filtering
  // here: the stair precondition (13 PR-1 / D-40), the switch window (MS-1) and
  // the read-back expectation all live in one place, and a second opinion at
  // this level is how the two drift apart.
  const ModeRequestResult r = mode_.Request(now_mono_s, action, param);
  if (r.accepted) {
    // kNonRealtime: this runs on the zenoh callback thread, not on ctrl. TX-2
    // makes that caller spin for the send guard rather than skip, which is
    // right here -- a stand or a prone the operator asked for must not be
    // dropped because ctrl happened to hold the section.
    SendModeFrame(action, param, TxCaller::kNonRealtime);
  }
  return r;
}

void QuadrupedProcess::OnSoftEstop(double now_mono_s) {
  // 13 S9.12.2 (3) and T-1: the generation advances HERE and a zero frame goes
  // out NOW. Setting a flag for the next control period would be up to 10 ms of
  // travel at whatever speed the robot had, against a 5 ms budget.
  ++estop_epoch_;
  std::uint8_t buf[512];
  chs_a::AxisCommand zero;   // every axis defaults to 0.0
  const std::size_t n =
      chs_a::EncodeRealAxis(buf, sizeof(buf), msg_id_++, WallNow(), zero);
  if (n > 0) {
    // The non-realtime caller spins for the guard rather than skipping: this
    // frame is the emergency stop and must not be dropped because ctrl happened
    // to hold the section (TX-2).
    const TxResult r = tx_.Send(TxCaller::kNonRealtime, buf, n);
    if (r == TxResult::kSent) ++axis_frames_sent_;
  }
  (void)now_mono_s;
}

void QuadrupedProcess::CtrlTick(double now_mono_s) {
  ++ctrl_ticks_;

  // ---- 1. the newest chassis snapshot, if one arrived -------------------
  //
  // BEFORE the session tick, not after. An arriving report is the ONLY evidence
  // the link is alive (13 CA-7: axis commands are never acknowledged), so the
  // session has to see it in the period it arrived. Ticking first put every
  // report one period late, and worse: a report landing just inside the lost
  // timeout would have been ignored and the link dropped with the evidence
  // sitting unread in the slot. Found by the assembly test, which could not
  // bring the link up at all.
  ChassisSnapshot fresh;
  if (snapshot_slot_.TakeFresh(&fresh)) {
    // MERGED BY SOURCE, never assigned whole. Each report fills only its own
    // fields and leaves the rest at the struct's defaults, so `latest_ = fresh`
    // let the 10 Hz MotionStatus overwrite everything the 2 Hz BasicStatus had
    // just delivered -- usage_mode, motion_state, gait and the HES all fell
    // back to 0 five times out of six.
    //
    // *** What that cost, measured on the bench 2026-09-18: Tier 1 reads
    // usage_mode_raw and holds every axis command at zero unless it equals
    // navigation (NAV-111). With the value being reset to 0 at 10 Hz it could
    // never equal navigation, so the robot could not be commanded to move AT
    // ALL -- and the symptom was stop_reason = mode_mismatch, which reads as
    // "the chassis is in the wrong mode" rather than "we are losing the field".
    //
    // The from_basic / from_motion flags existed for exactly this distinction
    // (see their comment in process.h: "the basic status said the robot is
    // awake" vs "the motion status happened not to mention it") and the
    // consumer simply did not use them.
    if (fresh.from_basic) {
      latest_.hes = fresh.hes;
      latest_.sleep = fresh.sleep;
      latest_.usage_mode_raw = fresh.usage_mode_raw;
      latest_.motion_state_raw = fresh.motion_state_raw;
      latest_.gait_raw = fresh.gait_raw;
      latest_.charge_raw = fresh.charge_raw;
      latest_.from_basic = true;
    }
    if (fresh.from_motion) {
      latest_.linear_x = fresh.linear_x;
      latest_.linear_y = fresh.linear_y;
      latest_.angular_z = fresh.angular_z;
      // MotionStatus carries motion_state and gait too, at 10 Hz against
      // BasicStatus's 2 Hz, so the faster source wins for those two.
      // *** It does NOT carry ControlUsageMode -- that field appears only in
      // BasicStatus (see ParseMotionStatus), which is the whole reason the
      // wholesale assignment erased it. usage_mode_raw is updated in the
      // from_basic branch and NOWHERE else; adding it here would be writing a
      // field this report never mentioned.
      latest_.motion_state_raw = fresh.motion_state_raw;
      latest_.gait_raw = fresh.gait_raw;
      latest_.from_motion = true;
    }
    // 13 S4.4 (4) / 11 S9.9: the gait decides whether wheel odometry may be
    // called valid AT ALL. On a stair gait 13 takes both measures -- covariance
    // inflated 3.33x AND valid = false -- and says in so many words that the
    // inflation alone is not enough there.
    //
    // *** Before this line, Odometry::OnGait had ZERO production call sites.
    // is_stair_gait_ therefore sat at its initialiser forever, so a robot on a
    // staircase published odometry with FLAT-ground trust and valid = true.
    // Everything else was in place: the trust divisor, the valid expression,
    // the config keys, and a unit test that calls OnGait(true) directly and
    // passes. The fifth instance of the same shape in this process, after
    // Uplink::Publish, the rt/chassis/mode subscription, the three mode-frame
    // encoders, and SetReportSink -- capability present, compiles, tests green,
    // last segment not connected.
    //
    // Placed after BOTH branches because both reports carry Gait. That caps the
    // update rate at the 10 Hz MotionStatus; the drdds /MOTION_INFO does carry
    // gait_state but this process does not read it (MotionInfoTick carries
    // velocity only). Not a safety gap: entering a stair gait is a commanded
    // transition that takes seconds, not one period.
    odom_.OnGait(chs_a::IsStairGait(latest_.gait_raw));
    // Receive time always: it is a property of the ARRIVAL, not of the report
    // kind, and the session's liveness judgement is about arrivals.
    latest_.rx_mono_s = fresh.rx_mono_s;
    have_snapshot_ = true;
    session_.OnReport(fresh.rx_mono_s);
    if (fresh.from_basic) {
      // 13 F-21: the sleep flag gates every motion command, and the session is
      // where that gate lives.
      session_.OnSleep(fresh.sleep);
    }
    if (fresh.from_motion) {
      odom_.OnVelocitySample(fresh.rx_mono_s, fresh.linear_x, fresh.linear_y);
      // When the MONITOR velocity last arrived -- not when any report did. The
      // linear source choice below compares against this, and the two are not
      // the same thing: basic, device and fault reports all refresh
      // latest_.rx_mono_s without carrying a velocity, so comparing against
      // that made a 20 Hz drdds source lose to a 2 Hz fault report. Found on
      // the real chassis (2026-09-17: angular=drdds while linear fell back to
      // monitor_10hz with drdds publishing at 19 Hz); the offline test missed
      // it because it only ever sent motion reports.
      last_monitor_vel_s_ = fresh.rx_mono_s;
      // Until the domain-0 IMU reader is wired, the yaw rate comes from the
      // 10 Hz monitor protocol. 13 S4.2 names this the DEGRADED source, and the
      // covariance model does not know the difference -- which is a gap worth
      // naming here rather than in a commit message nobody reads twice.
      odom_.OnYawRate(fresh.angular_z);
    }
  }

  // ---- 2. the link ------------------------------------------------------
  const chs_a::TickResult link = session_.Tick(now_mono_s);
  if (link.connected) {
    // FR-5 / SD-3 verified HERE, once per connection, and read BACK from the
    // kernel rather than trusted from the setsockopt return. The two are
    // different claims: the call can succeed on a socket where the option does
    // not take effect, and 13 S3.6's latency budget rests on the second one.
    //
    // Nothing was checking either. Reported, not fatal -- Nagle degrades the
    // timing, it does not break the link, and refusing the connection would
    // take Tier 1 down over a latency problem.
    // *** Publish the two FACTS, not the verdict. The verdict is one boolean
    // that happens to be true in every configuration a test can build -- so a
    // mutant replacing the whole expression with `true` was indistinguishable,
    // and that mutant is precisely the defect this check exists to prevent.
    //
    // Split, each half is observable: nodelay_active is read BACK from the
    // kernel and is false on a socket nobody set it on, and nodelay_expected
    // says whether we had any business asking. The supervisor combines them.
    // Same principle as 13 S4.4's event column: publish the quantity the
    // judgement is made from, and let the consumer make it.
    //
    // is_udp is part of `expected` rather than of `active`: TCP_NODELAY is
    // meaningless on a datagram socket and nodelay_enabled() answers false for
    // one by construction, so without it the process would report a timing
    // fault on every connection to the udp:30004 candidate 13 S8.2 enables.
    pub_nodelay_active_.store(socket_.nodelay_enabled(),
                              std::memory_order_relaxed);
    pub_nodelay_expected_.store(cfg_.link.tcp_nodelay && !socket_.is_udp(),
                                std::memory_order_relaxed);
    pub_nodelay_requested_.store(socket_.nodelay_requested() != 0,
                                 std::memory_order_relaxed);
    pub_nodelay_setopt_ok_.store(socket_.nodelay_setopt_ok() != 0,
                                 std::memory_order_relaxed);
  }
  if (link.disconnected) {
    // Bytes from the old connection must never be read as the start of the new
    // one, so the reassembly buffer is dropped with the socket.
    framer_.Reset();
  }
  if (link.send_heartbeat) {
    std::uint8_t buf[256];
    const std::size_t n =
        chs_a::EncodeHeartbeat(buf, sizeof(buf), msg_id_++, WallNow());
    if (n > 0) {
      const TxResult r = tx_.Send(TxCaller::kRealtime, buf, n);
      if (r == TxResult::kSent) {
        ++heartbeats_sent_;
        session_.OnSendSuccess();
      } else if (r == TxResult::kSkipped) {
        ++tx_skipped_;
      } else {
        session_.OnSendFailure(now_mono_s);
      }
    }
  }

  // ---- 3. Tier 1 --------------------------------------------------------
  Tier1Input in;
  in.now_mono_s = now_mono_s;
  in.last_cmd_rx_mono_s = cmd_rx_mono_s_;
  in.has_cmd = have_cmd_;
  in.vx = cmd_vx_;
  in.vy = cmd_vy_;
  in.wz = cmd_wz_;
  in.cmd_estop_epoch = cmd_estop_epoch_;
  in.local_estop_epoch = estop_epoch_;
  // *** motion_state_transitioning(), NOT mode_switching(). 13 TR-1 is
  // explicit about which one Tier 1 gets: a MotionState that changes WITHOUT
  // our having commanded it (the factory handset, a second client) must "置
  // mode_switching = true 并保持 external_transition_hold_s, 期间零速".
  //
  // mode_switching() is only OUR OWN switch. Feeding it here meant an external
  // transition produced no zero-speed hold at all: the handset put the robot
  // into a 2-3 s stand-up while we kept sending axis commands into it. TR-1
  // calls the alternative "believing a moving robot is stationary", and 13
  // V-61 is why we cannot see the transition end any other way.
  in.mode_switching = mode_.motion_state_transitioning();
  // Consumed here, on the period that reads it (13 S9.4 ASM-2). A held flag
  // would re-unlock the lock on every period after the first -- see OnEnable.
  in.enable_requested = enable_pending_;
  enable_pending_ = false;
  if (have_snapshot_) {
    in.hes_raw = latest_.hes;
    in.sleep_readback = latest_.sleep;
    in.usage_mode_raw = latest_.usage_mode_raw;
  } else {
    // Nothing has been heard from the chassis. usage_mode stays at its default
    // of 0, which is NOT navigation, so Tier 1 reports mode_mismatch and holds
    // zero -- the correct answer before the mode is known.
    in.usage_mode_raw = 0;
  }
  last_tier1_ = tier1_.Step(in);

  // ---- 4. the axis command, behind BOTH gates (13 S9.4 ASM-3) -----------
  if (last_tier1_.stop_reason == StopReason::kNone && session_.motion_allowed()) {
    std::uint8_t buf[512];
    chs_a::AxisCommand axis;
    axis.vx = last_tier1_.vx;
    axis.vy = last_tier1_.vy;
    axis.yaw = last_tier1_.wz;
    const std::size_t n =
        chs_a::EncodeRealAxis(buf, sizeof(buf), msg_id_++, WallNow(), axis);
    if (n > 0) {
      const TxResult r = tx_.Send(TxCaller::kRealtime, buf, n);
      if (r == TxResult::kSent) {
        ++axis_frames_sent_;
        session_.OnSendSuccess();
      } else if (r == TxResult::kSkipped) {
        // TX-3: skipping is safe. The frame this period would have carried is
        // superseded by the next one 10 ms later, and a stop is never carried
        // by this path -- it goes out from the callback.
        ++tx_skipped_;
      } else {
        session_.OnSendFailure(now_mono_s);
      }
    }
  }

  // ---- 4b. channel two, and the source choice (13 S4.2) -----------------
  {
    ImuTick imu;
    if (imu_slot_.TakeFresh(&imu)) last_imu_ = imu;
    MotionInfoTick mi;
    if (mi_slot_.TakeFresh(&mi)) last_mi_ = mi;

    // ANGULAR. The threshold is the configured one, whose own comment says what
    // it is for. ClassifyImuAge draws the "never seen" case apart from "stale"
    // -- they call for the same fallback but a different message, and merging
    // them is how "the IMU was never wired" gets reported as "the IMU is late".
    const double imu_age =
        (last_imu_.rx_mono_s < 0.0) ? -1.0 : (now_mono_s - last_imu_.rx_mono_s);
    const ImuFreshness imu_fresh =
        ClassifyImuAge(imu_age, cfg_.dds.imu_age_warn_ms);
    if (imu_fresh == ImuFreshness::kFresh) {
      odom_.OnYawRate(last_imu_.wz);
      angular_src_ = OdomSource::kDrdds;
      ++ang_drdds_;
    } else if (have_snapshot_ && latest_.from_motion) {
      // Degraded, per the table's second row. The monitor value was already
      // handed to the odometry in step 1; recording the source here is what
      // makes the degradation visible at all.
      angular_src_ = OdomSource::kMonitor;
      ++ang_monitor_;
    } else {
      angular_src_ = OdomSource::kNone;
    }

    // LINEAR. Newest wins -- see the header for why there is no threshold here.
    const bool mi_newer = last_mi_.rx_mono_s >= 0.0 &&
                          last_mi_.rx_mono_s > last_monitor_vel_s_;
    if (mi_newer) {
      odom_.OnVelocitySample(last_mi_.rx_mono_s, last_mi_.vx, last_mi_.vy);
      linear_src_ = OdomSource::kDrdds;
      ++lin_drdds_;
    } else if (last_monitor_vel_s_ >= 0.0) {
      linear_src_ = OdomSource::kMonitor;
      ++lin_monitor_;
    }
  }

  // ---- 4c. the mode sequence, one step per period (11 S9.2.4) -----------
  // AFTER the axis command above, never before: 11 S9.2.2 requires zero speed
  // across a switch, and Tier 1 already holds zero while mode_switching is
  // true. Dispatching first would put the switch and this period's axis frame
  // in the same period with the frame decided before the switch was known.
  if (mode_sequence_pending() && !mode_.mode_switching()) {
    // Fixed order: motion_state, then gait, then usage_mode. 13 MS-5 fixes the
    // first two (the chassis couples them, and the posture has to settle
    // before the gait means anything). usage_mode goes LAST because it is the
    // gate Tier 1 opens on -- the permissive step belongs behind the others,
    // so a sequence that fails halfway leaves the robot unable to move rather
    // than able to move in a posture nobody confirmed.
    ModeAction action = ModeAction::kSetUsageMode;
    std::int64_t param = 0;
    if (mode_want_state_) {
      // stand / prone have their own actions; rl_control is the third
      // commandable value and maps to kRlControl. Anything else never reaches
      // here -- rt_parse's table only admits the three (11 S9.2.4).
      action = (mode_state_ == kCommandMotionStateStand) ? ModeAction::kStand
             : (mode_state_ == kCommandMotionStateProne) ? ModeAction::kProne
                                                         : ModeAction::kRlControl;
      param = 0;
      mode_want_state_ = false;
    } else if (mode_want_gait_) {
      action = ModeAction::kSetGait;
      param = mode_gait_;
      mode_want_gait_ = false;
    } else {
      action = ModeAction::kSetUsageMode;
      param = mode_usage_;
      mode_want_usage_ = false;
    }
    const ModeRequestResult r = mode_.Request(now_mono_s, action, param);
    if (r.accepted) {
      SendModeFrame(action, param, TxCaller::kRealtime);
      ++mode_steps_;
    } else {
      // A refused step abandons the REST of the triple. Carrying on would
      // apply a partial triple, and 13 MS-5 compares all three -- a partial
      // application is a state no expectation describes.
      mode_want_state_ = false;
      mode_want_gait_ = false;
      mode_want_usage_ = false;
      // *** The REASON, kept rather than dropped. 13 MS-3a records that this
      // refusal is silent -- rt/chassis/mode has no ack key and RT-C4 keeps
      // this process off the plane where the cmd acks live -- so the log is
      // the only place it can surface, and until now it did not reach there
      // either: GS-1 refusing stair_standard and MS-3 refusing a second switch
      // produced the same nothing.
      //
      // 13 RX-9's argument applied one key over: a refusal counted without its
      // reason is indistinguishable from a command nobody sent, and both look
      // like a robot that will not change mode.
      last_mode_reject_ = r.reject;
      ++mode_steps_refused_;
    }
  }

  // ---- 5. the odometry --------------------------------------------------
  const double dt = (last_ctrl_s_ < 0.0) ? (1.0 / cfg_.tier1.control_loop_hz)
                                         : (now_mono_s - last_ctrl_s_);
  last_odom_ = odom_.Tick(now_mono_s, dt);
  // WALL-CLOCK-OK(align): stamps the sample for the ROS header downstream
  // (13 S9.1 "时间戳取自样本, 不取自发布时刻"). Read HERE, on the thread that
  // produced the pose, so the label says when the pose was true rather than
  // when someone got around to sending it. It decides nothing -- dt above and
  // every Tier 1 age are steady_clock.
  last_odom_.stamp_wall_s = WallNowSeconds();
  // Offered to rt_pub, every tick, including the ticks whose sample says not to
  // publish: the DECISION not to publish is itself something the publisher has
  // to see. Dropping those here would leave rt_pub sending the last good pose
  // forever, and 13 S4.4 (4) requires the TF to stop with the odometry -- a
  // frozen TF makes Nav2 believe the robot is stationary and keep commanding
  // rotation.
  odom_slot_.Publish(last_odom_);
  ++odom_offered_;

  // The state rt_pub publishes from. Built here, on the thread that owns every
  // field in it, rather than read piecemeal from another thread -- the members
  // below are plain and written every period, so a second reader would be a
  // data race whose usual symptom is a correct-looking answer.
  StateSnapshot snap;
  snap.conn = session_.state();
  snap.tier1 = last_tier1_;
  snap.estop_epoch = estop_epoch_;
  snap.cmd_age_ms =
      have_cmd_ ? (now_mono_s - cmd_rx_mono_s_) * 1000.0 : -1.0;
  snap.soft_estop_active = have_cmd_ && (cmd_estop_epoch_ != estop_epoch_);
  // Same predicate Tier 1 was given, for the same reason: 13 TR-1 says an
  // external change sets mode_switching, and a state key that disagreed with
  // the gate would have an operator watching `false` while the robot is held
  // at zero.
  // Two fields, two questions -- see StateSnapshot. v1.23 shipped the
  // transitioning meaning under the mode_switching name because there was
  // only one field; the 2026-09-21 F-5 unfreeze gave each its own.
  snap.mode_switching = mode_.mode_switching();
  snap.motion_state_transitioning = mode_.motion_state_transitioning();
  snap.odom_source = linear_src_;
  snap.motion_allowed = session_.motion_allowed();
  // From latest_, which is merged by source (see the merge note above) -- so
  // the usage_mode here is the one BasicStatus delivered, not a value some
  // faster report happened to leave behind.
  snap.has_readback = have_snapshot_ && latest_.from_basic;
  snap.usage_mode_raw = latest_.usage_mode_raw;
  snap.motion_state_raw = latest_.motion_state_raw;
  snap.gait_raw = latest_.gait_raw;
  // Set from last_odom_ above, on the same tick that produced it, so the pose
  // in the state message and the pose in /odom_quadruped are the same sample
  // rather than two samples that happen to be close.
  snap.has_charge = have_snapshot_ && latest_.from_basic;
  snap.charge_raw = latest_.charge_raw;
  snap.hes = latest_.hes;
  snap.sleep = latest_.sleep;
  snap.odom = last_odom_;
  pub_switch_fail_.store(mode_.switch_failures(), std::memory_order_relaxed);
  pub_mode_frames_.store(mode_frames_sent_, std::memory_order_relaxed);
  // The tx-guard triple, from the thread that owns the realtime send path.
  pub_tx_skips_.store(tx_.tx_skip_count(), std::memory_order_relaxed);
  pub_tx_acquires_.store(tx_.rt_acquire_count(), std::memory_order_relaxed);
  pub_tx_sent_.store(tx_.sent_count(), std::memory_order_relaxed);
  state_slot_.Publish(snap);

  last_ctrl_s_ = now_mono_s;
  mode_.Tick(now_mono_s);

  // Publish for the supervisor. Relaxed: these are a status display, not a
  // synchronisation point, and nothing downstream orders anything on them.
  // The frame count is NOT published here -- it is written by the rx thread,
  // so reading it from ctrl would be exactly the race this block avoids.
  pub_conn_.store(static_cast<int>(session_.state()), std::memory_order_relaxed);
  pub_active_ep_.store(session_.active_endpoint(), std::memory_order_relaxed);
  pub_probe_cycles_.store(session_.probe_cycles(), std::memory_order_relaxed);
}

bool QuadrupedProcess::TakeOdomForPublish(OdomSample* out) {
  if (out == nullptr) return false;
  return odom_slot_.TakeFresh(out);
}

void QuadrupedProcess::OnImu(double now_mono_s, double wz) {
  ImuTick t;
  t.wz = wz;
  t.rx_mono_s = now_mono_s;
  imu_slot_.Publish(t);
}

void QuadrupedProcess::OnMotionInfo(double now_mono_s, double vx, double vy) {
  MotionInfoTick t;
  t.vx = vx;
  t.vy = vy;
  t.rx_mono_s = now_mono_s;
  mi_slot_.Publish(t);
}

void QuadrupedProcess::SetReportSink(ReportSink sink) {
  // Installed before the threads start. There is no lock: the only writer is
  // the setup path and the only reader is chs_a_rx, and they do not overlap in
  // time. Calling this on a running process would be a defect, which is why it
  // is not guarded -- a guard would make it look supported.
  report_sink_ = std::move(sink);
}

bool QuadrupedProcess::TakeStateForPublish(StateSnapshot* out) {
  if (out == nullptr) return false;
  return state_slot_.TakeFresh(out);
}

QuadrupedProcess::LinkStatus QuadrupedProcess::link_status() const {
  LinkStatus s;
  s.conn = static_cast<chs_a::ConnState>(pub_conn_.load(std::memory_order_relaxed));
  s.active_endpoint = pub_active_ep_.load(std::memory_order_relaxed);
  s.probe_cycles = pub_probe_cycles_.load(std::memory_order_relaxed);
  s.frames = pub_frames_.load(std::memory_order_relaxed);
  s.dropped = pub_dropped_.load(std::memory_order_relaxed);
  s.last_error_code = pub_err_code_.load(std::memory_order_relaxed);
  s.error_codes_seen = pub_err_seen_.load(std::memory_order_relaxed);
  s.switch_failures = pub_switch_fail_.load(std::memory_order_relaxed);
  s.mode_frames_sent = pub_mode_frames_.load(std::memory_order_relaxed);
  s.nodelay_active = pub_nodelay_active_.load(std::memory_order_relaxed);
  s.nodelay_expected = pub_nodelay_expected_.load(std::memory_order_relaxed);
  s.nodelay_requested = pub_nodelay_requested_.load(std::memory_order_relaxed);
  s.nodelay_setopt_ok = pub_nodelay_setopt_ok_.load(std::memory_order_relaxed);
  s.resync_bytes = pub_resync_bytes_.load(std::memory_order_relaxed);
  s.tx_skips = pub_tx_skips_.load(std::memory_order_relaxed);
  s.tx_acquires = pub_tx_acquires_.load(std::memory_order_relaxed);
  s.tx_sent = pub_tx_sent_.load(std::memory_order_relaxed);
  return s;
}

// One decoded frame, whatever transport carried it. Extracted so the two
// framing paths below share it verbatim: a stream path and a datagram path
// that each grew their own copy would drift, and the drift would show up as
// "the UDP endpoint reports a different set of things".
void QuadrupedProcess::HandleFrame(double now_mono_s) {
  chs_a::AsduRouting route;
  if (!chs_a::ParseAsduRouting(framer_.asdu(), framer_.asdu_len(), &route)) {
    return;
  }
  if (route.has_error_code) {
    session_.OnErrorCode(now_mono_s, route.error_code);
    return;
  }

  // The JSON parse happens HERE, on the ordinary-priority thread. That is the
  // entire reason this thread exists (13 S9.1): parsing allocates, and QD-7
  // forbids allocation in ctrl.
  ChassisSnapshot snap;
  snap.rx_mono_s = now_mono_s;
  if (route.type == chs_a::kTypeBasic && route.command == chs_a::kReportCommand) {
    chs_a::BasicStatus b;
    if (chs_a::ParseBasicStatus(framer_.asdu(), framer_.asdu_len(), &b)) {
      snap.from_basic = true;
      snap.hes = b.hes;
      snap.sleep = b.sleep;
      snap.usage_mode_raw = b.usage_mode.raw;
      snap.motion_state_raw = b.motion_state.raw;
      snap.gait_raw = b.gait.raw;
      snap.charge_raw = b.charge;
      mode_.OnReadback(now_mono_s, b);
      snapshot_slot_.Publish(snap);
      // Forwarded from HERE, with the full report including its strings.
      // What went into the slot above is the trimmed POD ctrl acts on; this
      // is the report itself, and the two are not interchangeable.
      if (report_sink_) {
        report_sink_(now_mono_s, &b, nullptr, nullptr, nullptr);
        ++reports_forwarded_;
      }
    }
  } else if (route.type == chs_a::kTypeMotion &&
             route.command == chs_a::kReportCommand) {
    chs_a::MotionStatus m;
    if (chs_a::ParseMotionStatus(framer_.asdu(), framer_.asdu_len(), &m)) {
      snap.from_motion = true;
      snap.linear_x = m.linear_x;
      snap.linear_y = m.linear_y;
      snap.angular_z = m.angular_z;
      snap.motion_state_raw = m.motion_state.raw;
      snap.gait_raw = m.gait.raw;
      // 13 TR-1 (2026-09-21 ruling): this stream also feeds the external-
      // transition hold -- it reports motion_state five times as often as
      // BasicStatus, and before this line the state message could carry a
      // moved triple while mode_switching still read false for up to 0.5 s
      // (measured on the chassis with the factory handset).
      mode_.OnMotionSample(now_mono_s, m.motion_state.raw, m.gait.raw);
      snapshot_slot_.Publish(snap);
      if (report_sink_) {
        report_sink_(now_mono_s, nullptr, &m, nullptr, nullptr);
        ++reports_forwarded_;
      }
    }
  } else if (route.type == chs_a::kTypeDevice &&
             route.command == chs_a::kReportCommand) {
    chs_a::DeviceStatus d;
    if (chs_a::ParseDeviceStatus(framer_.asdu(), framer_.asdu_len(), &d)) {
      snapshot_slot_.Publish(snap);
      if (report_sink_) {
        report_sink_(now_mono_s, nullptr, nullptr, &d, nullptr);
        ++reports_forwarded_;
      }
    }
  } else if (route.type == chs_a::kTypeFault) {
    // *** NOT gated on kReportCommand. 13 S7.3 makes the fault stream "2 Hz
    // PLUS on change", and the change-driven frame does not carry the
    // periodic report command. Requiring it would drop exactly the frames
    // that matter -- a new fault is reported the moment it appears, and that
    // is the one an operator is waiting for.
    chs_a::FaultReport f;
    if (chs_a::ParseFaultReport(framer_.asdu(), framer_.asdu_len(), &f)) {
      snapshot_slot_.Publish(snap);
      if (report_sink_) {
        report_sink_(now_mono_s, nullptr, nullptr, nullptr, &f);
        ++reports_forwarded_;
      }
    }
  } else {
    // Location and anything else this build does not model. They still count
    // as liveness -- 13 CA-7 makes an arriving report the ONLY evidence the
    // link is alive, whatever kind of report it is -- and they are NOT
    // forwarded, because forwarding a frame nobody parsed would put bytes of
    // unknown shape onto a key with a schema.
    snapshot_slot_.Publish(snap);
  }
}

int QuadrupedProcess::RxPump(double now_mono_s) {
  if (!socket_.is_open()) return 0;
  std::uint8_t rx[4096];
  const long n = socket_.Recv(rx, sizeof(rx));
  if (n < 0) {
    // The peer closed or the link broke. The session decides what to do about
    // it on its own timetable; this thread only reports the fact.
    session_.OnSendFailure(now_mono_s);
    return 0;
  }

  // *** Which framing. 13 S2.2 gives channel one TWO endpoint candidates --
  // tcp:30003 and udp:30004 -- and BOTH are enabled in the resolved config,
  // with S8.2 picking "the first one that delivers a status report". Before
  // this branch every byte went through Push(), the STREAM entry point, and
  // Framer::PushDatagram had ZERO production call sites.
  //
  // What that would have cost on the UDP candidate: the stream framer resyncs
  // on the sync word and carries leftover bytes into the next push, so a
  // truncated datagram is silently CONCATENATED with the next unrelated one
  // and can pass the header check as a frame. FR-5 exists to make that
  // impossible -- one datagram is one frame, anything else is dropped -- and
  // its comment says so in as many words. UDP has no ordering guarantee that
  // would make a continuation meaningful in the first place.
  //
  // Not reachable today (tcp:30003 answers first, measured 2026-09-18), which
  // is exactly why it would have stayed broken until the day TCP failed.
  if (socket_.is_udp()) {
    if (n == 0) return 0;
    if (framer_.PushDatagram(rx, static_cast<std::size_t>(n)) !=
        chs_a::FrameStatus::kFrame) {
      // Counted, not logged: FR-5's `warn` needs an event path this process
      // does not have (11 RT-C4), and a log line per bad datagram is how a
      // merely noisy link takes down the thread meant to report it. The count
      // reaches the supervisor through link_status().
      pub_dropped_.store(framer_.dropped_frames(), std::memory_order_relaxed);
      return 0;
    }
    ++frames_received_;
    HandleFrame(now_mono_s);
    pub_frames_.store(frames_received_, std::memory_order_relaxed);
    pub_dropped_.store(framer_.dropped_frames(), std::memory_order_relaxed);
    // One frame per datagram, by FR-5. Next() would not find it anyway: it
    // reads the stream buffer, which PushDatagram deliberately leaves empty.
    return 1;
  }

  if (n > 0) {
    framer_.Push(rx, static_cast<std::size_t>(n));
  }

  int frames = 0;
  for (;;) {
    const chs_a::FrameStatus st = framer_.Next(now_mono_s);
    if (st != chs_a::FrameStatus::kFrame) {
      if (st == chs_a::FrameStatus::kPoisoned) socket_.Close();
      break;
    }
    ++frames;
    ++frames_received_;
    HandleFrame(now_mono_s);
  }
  // Published from THIS thread, which is the one that owns the counters.
  pub_frames_.store(frames_received_, std::memory_order_relaxed);
  pub_dropped_.store(framer_.dropped_frames(), std::memory_order_relaxed);
  pub_err_code_.store(session_.last_error_code(), std::memory_order_relaxed);
  pub_err_seen_.store(session_.error_codes_seen(), std::memory_order_relaxed);
  pub_resync_bytes_.store(framer_.resync_bytes_total(),
                          std::memory_order_relaxed);
  return frames;
}

bool QuadrupedProcess::Start() {
  if (running_.exchange(true)) return false;

  // 13 S9.2 RTC-7. Reported rather than ignored: under the systemd defaults
  // this returns ENOMEM, the process runs on with pageable memory, and the
  // first deadline it misses is whenever the kernel next reclaims a page.
  mlock_error_ = rt::LockAllMemory();

  rx_thread_ = std::thread([this] { RxLoop(); });
  ctrl_thread_ = std::thread([this] { CtrlLoop(); });
  return true;
}

void QuadrupedProcess::Stop() {
  if (!running_.exchange(false)) return;
  if (ctrl_thread_.joinable()) ctrl_thread_.join();
  if (rx_thread_.joinable()) rx_thread_.join();
  socket_.Close();
}

void QuadrupedProcess::CtrlLoop() {
  // 13 S9.1: SCHED_FIFO 80. The errno is kept rather than discarded -- EPERM
  // here means the unit did not raise LimitRTPRIO, and the loop then runs at
  // ordinary priority with nothing to show for it.
  ctrl_priority_error_ =
      rt::ApplyFifoPriority(pthread_self(), cfg_.realtime.ctrl_priority);

  const double period_s = 1.0 / cfg_.tier1.control_loop_hz;
  const auto period = std::chrono::duration<double>(period_s);
  auto next = std::chrono::steady_clock::now();
  while (running_.load(std::memory_order_acquire)) {
    CtrlTick(MonoNowSeconds());
    // Absolute deadlines, not sleep-for. Sleeping for a period after the work
    // makes the loop drift by however long the work took, every period, and at
    // 100 Hz that drift is what the jitter budget is made of.
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
}

void QuadrupedProcess::RxLoop() {
  // Ordinary priority by design: the JSON parse lives here precisely so it is
  // NOT on a realtime thread.
  while (running_.load(std::memory_order_acquire)) {
    const int n = RxPump(MonoNowSeconds());
    if (n == 0) {
      // Nothing waiting. A short sleep rather than a spin: this thread has no
      // deadline, and a spin would take a core away from one that does.
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

}  // namespace quadruped
