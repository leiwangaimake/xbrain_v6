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
#include "quadruped/mono_clock.h"
#include "xbrain/rtcomm/rt_thread.h"

namespace quadruped {
namespace {

namespace rt = hachist::xbrain::rtcomm;


std::int64_t WallNow() { return static_cast<std::int64_t>(::time(nullptr)); }

// 13 S9.1: ctrl is SCHED_FIFO 80, chs_b is 70, everything else ordinary.
constexpr int kCtrlFifoPriority = 80;

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
        m.switch_timeout_s = 5.0;
        m.external_transition_hold_s = 3.5;
        (void)cfg;
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

ModeRequestResult QuadrupedProcess::OnChassisAction(double now_mono_s,
                                                    ModeAction action,
                                                    std::int64_t param) {
  // Straight through to the machine. There is deliberately no pre-filtering
  // here: the stair precondition (13 PR-1 / D-40), the switch window (MS-1) and
  // the read-back expectation all live in one place, and a second opinion at
  // this level is how the two drift apart.
  return mode_.Request(now_mono_s, action, param);
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
    latest_ = fresh;
    have_snapshot_ = true;
    session_.OnReport(fresh.rx_mono_s);
    if (fresh.from_basic) {
      // 13 F-21: the sleep flag gates every motion command, and the session is
      // where that gate lives.
      session_.OnSleep(fresh.sleep);
    }
    if (fresh.from_motion) {
      odom_.OnVelocitySample(fresh.rx_mono_s, fresh.linear_x, fresh.linear_y);
      // Until the domain-0 IMU reader is wired, the yaw rate comes from the
      // 10 Hz monitor protocol. 13 S4.2 names this the DEGRADED source, and the
      // covariance model does not know the difference -- which is a gap worth
      // naming here rather than in a commit message nobody reads twice.
      odom_.OnYawRate(fresh.angular_z);
    }
  }

  // ---- 2. the link ------------------------------------------------------
  const chs_a::TickResult link = session_.Tick(now_mono_s);
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
  in.mode_switching = mode_.mode_switching();
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

  // ---- 5. the odometry --------------------------------------------------
  const double dt = (last_ctrl_s_ < 0.0) ? (1.0 / cfg_.tier1.control_loop_hz)
                                         : (now_mono_s - last_ctrl_s_);
  last_odom_ = odom_.Tick(now_mono_s, dt);
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
  snap.mode_switching = mode_.mode_switching();
  snap.motion_allowed = session_.motion_allowed();
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
  return s;
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

    chs_a::AsduRouting route;
    if (!chs_a::ParseAsduRouting(framer_.asdu(), framer_.asdu_len(), &route)) {
      continue;
    }
    if (route.has_error_code) {
      session_.OnErrorCode(now_mono_s, route.error_code);
      continue;
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
        mode_.OnReadback(now_mono_s, b);
        snapshot_slot_.Publish(snap);
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
        snapshot_slot_.Publish(snap);
      }
    } else {
      // The device, fault and location reports are parsed by the publisher once
      // it exists. They still count as liveness, which is what OnReport below
      // records -- 13 CA-7 makes an arriving report the ONLY evidence the link
      // is alive, whatever kind of report it is.
      snapshot_slot_.Publish(snap);
    }
  }
  // Published from THIS thread, which is the one that owns the counter.
  pub_frames_.store(frames_received_, std::memory_order_relaxed);
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
  ctrl_priority_error_ = rt::ApplyFifoPriority(pthread_self(), kCtrlFifoPriority);

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
