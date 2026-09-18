/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: main.cc
 * Brief: quadruped_m20 entry -- runs the process, or self-checks the config
 *
 * Description:
 * With no argument this now RUNS: it loads the resolved snapshot, builds the
 * assembly (process.h) and starts the control and receive threads. The B0
 * skeleton refused to start, on the grounds that a process which looks alive
 * while talking to nothing is worse than one that fails -- that branch is gone
 * because the thing it was standing in for exists.
 *
 * What is still missing (13 S9.4 ASM-4), said here rather than discovered
 * later: the domain-0 DDS reader and the RT-plane publisher are separate
 * optional targets and this entry point does not start them. So the process
 * drives the chassis and holds it safe, and it does NOT yet publish state onto
 * the RT plane or answer a hello. p1_motion will still wait for a hello_ack
 * that does not come.
 *
 * The exit-78 branch that remains is a GENUINE config failure (13 S9.4
 * ASM-5), and the unit keeps RestartPreventExitStatus=78 for it. Restarting
 * cannot fix a config that is missing a key or carries an uncalibrated null;
 * without the line systemd would retry every two seconds forever and the one
 * useful message would scroll out of the journal. (The B0 comment said to
 * remove the line together with the refusal branch. On re-examination that was
 * wrong in one half: the branch is gone, the line should stay, and the unit
 * now says why.)
 *
 * What --selfcheck does. Loads a resolved snapshot and prints the effective
 * transport block (DDS-9: the two domain ids, the probe order, the codebook).
 * It is the only cheap way to tell "the two DDS domains collapsed into one"
 * from "the network is down" -- both present as a participant that is up and
 * receives nothing. It is also the first thing to run on the ORIN after a
 * freeze, because it answers "did my config actually expand" in one line.
 *
 * Exit codes are the sysexits convention so an operator and a script read the
 * same thing: 0 ok, 64 usage, 78 config (the resolved snapshot is missing a
 * key, carries an uncalibrated null, or could not be read at all). No other
 * code is produced, and 78 no longer means "this binary is a placeholder".
 */

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <fstream>

#include "quadruped/process.h"
#include "quadruped/quadruped_config.h"
#if QUADRUPED_HAVE_CHS_B
#include "quadruped/chs_b_runtime.h"
#endif
#if QUADRUPED_HAVE_RT
#include "quadruped/rt_runtime.h"
#endif
#if QUADRUPED_HAVE_UPLINK
#include "quadruped/uplink.h"
#endif

namespace {

// sysexits.h values, written out rather than included: the header is not
// guaranteed on every toolchain and these three are the whole vocabulary.
constexpr int kExitOk = 0;
constexpr int kExitUsage = 64;
constexpr int kExitConfig = 78;

void PrintUsage(const char* argv0) {
  std::printf(
      "usage: %s --selfcheck [<resolved.yaml>]\n"
      "\n"
      "  --selfcheck   load the resolved config and print the effective\n"
      "                transport block (13 DDS-9). Default path: %s\n"
      "\n"
      "With no argument the process RUNS: it loads %s, connects to the\n"
      "chassis and starts the control and receive threads.\n"
      "\n"
      "All four links start from here: channel one to the chassis, channel\n"
      "two on domain 0 (/IMU + /MOTION_INFO), the RT plane, and the ROS 2\n"
      "uplink (odom + TF). A link that fails to come up is reported and the\n"
      "process continues -- 13 S4.2 row 2 exists so that losing the better\n"
      "odometry source degrades rather than refuses.\n",
      argv0, quadruped::DefaultResolvedPath(),
      quadruped::DefaultResolvedPath());
}

// Set by the signal handler, read by the run loop. sig_atomic_t and volatile
// because that is the only thing a handler may portably touch, and because the
// alternative -- calling Stop() from the handler -- would join threads inside
// a signal context.
volatile std::sig_atomic_t g_stop = 0;

void OnSignal(int) { g_stop = 1; }

// The first 8 hex of /proc/sys/kernel/random/boot_id, which is what 11 S3.0
// puts in the envelope's `boot` field. It rides with `mono` so a receiver can
// tell a monotonic reading from THIS boot from one that is not comparable with
// its own clock (CLK-C4) -- a foreign mono produces an age wrong by however
// long the two machines have been up, which looks like anything but a clock
// problem.
//
// An unreadable boot id yields an empty string, and an empty one never matches,
// so every `mono` we receive falls back to the receive-time age. That is the
// safe direction: the fallback is merely less precise, while a wrong match is
// an age that is wrong by hours.
std::string ReadBootId() {
  std::ifstream f("/proc/sys/kernel/random/boot_id");
  std::string id;
  if (!f || !(f >> id)) return std::string();
  // "a1b2c3d4-...." -> "a1b2c3d4"
  return id.substr(0, 8);
}

int Run(const std::string& path) {
  const quadruped::QuadrupedConfig cfg = quadruped::LoadQuadrupedConfig(path);
  // The effective values, once, at startup (13 DDS-9 / CB-4). This is the only
  // cheap way to tell a misconfigured domain from a dead network later.
  std::printf("config: %s\n", path.c_str());
  std::fputs(quadruped::DescribeConfig(cfg).c_str(), stdout);
  std::fflush(stdout);

  // Installed BEFORE the threads exist, not after. A SIGTERM in the window
  // between Start and here would otherwise take the default disposition and
  // kill the process outright -- no Stop, no join, and the chassis left holding
  // whatever the last axis command was. The window is microseconds wide, which
  // is an argument about how often it happens and not about what happens.
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  quadruped::QuadrupedProcess proc(cfg);
  if (!proc.Start()) {
    // Start refuses only when the object is already running, which a freshly
    // constructed one is not. It is checked rather than discarded because an
    // ignored bool return is how a process that started nothing goes on to
    // report itself healthy -- and 78 is the only code in this binary's
    // vocabulary that is not "ok", so it carries this too.
    std::fprintf(stderr,
                 "quadruped_m20: internal error -- the assembly refused to "
                 "start\n");
    return kExitConfig;
  }
  // Neither failure stops the process, and both are REPORTED. Under the systemd
  // defaults both happen (13 S9.2 RTC-7 and S9.1), and neither prints anything
  // by itself -- the symptom is jitter, which is the hardest thing to trace
  // back to a unit file. The unit now raises both limits; this line is what
  // says so when it does not.
  if (proc.mlock_error() != 0) {
    std::fprintf(stderr,
                 "quadruped_m20: mlockall failed (errno %d) -- pages stay "
                 "pageable and a reclaim will cost a deadline (13 RTC-7). "
                 "Check LimitMEMLOCK on the unit.\n",
                 proc.mlock_error());
  }

#if QUADRUPED_HAVE_CHS_B
  // Channel two. A failure here does NOT stop the process: 13 S4.2's second row
  // is the whole point of having a priority table, and a robot that refuses to
  // run because the better odometry source is missing is worse than one that
  // runs on the 10 Hz source and says which one it is using.
  quadruped::ChsBRuntime chs_b(&proc, cfg);
  std::string chs_b_err;
  const bool chs_b_up = chs_b.Start(&chs_b_err);
  if (!chs_b_up) {
    std::fprintf(stderr,
                 "quadruped_m20: channel two did NOT come up (%s) -- the "
                 "odometry runs on the monitor protocol's 10 Hz velocity and "
                 "yaw (13 S4.2 row 2), which is the DEGRADED source.\n",
                 chs_b_err.c_str());
  } else {
    std::printf("chs_b: domain %d, %s at %.0f Hz expected\n",
                chs_b.actual_domain_id(), cfg.dds.imu_topic.c_str(),
                cfg.dds.imu_expect_hz);
    std::fflush(stdout);
  }
#else
  std::fprintf(stderr,
               "quadruped_m20: built WITHOUT channel two -- the odometry runs "
               "on the 10 Hz DEGRADED source (13 S4.2 row 2).\n");
#endif

#if QUADRUPED_HAVE_RT
  // The RT plane. Started AFTER the process so a command arriving on the first
  // millisecond has somewhere to go.
  const std::string boot = ReadBootId();
  if (boot.empty()) {
    std::fprintf(stderr,
                 "quadruped_m20: cannot read /proc/sys/kernel/random/boot_id "
                 "-- every inbound `mono` will fall back to receive-time age "
                 "(11 S3.0). Less precise, never wrong.\n");
  }
  quadruped::rt::RtRuntime rt(&proc, cfg, boot);
#if QUADRUPED_HAVE_UPLINK
  // 13 V-69: odom + TF publish on rt_pub, integration stays in ctrl. This is
  // the binding that makes that true -- before it, Uplink::Publish had no call
  // site anywhere in the process and the whole of channel three was a library
  // nobody invoked. The sink is injected (rather than rt_runtime calling
  // Uplink directly) so quadruped_rt keeps its zero-rclcpp dependency; see
  // OdomSink in rt_runtime.h.
  //
  // Constructed BEFORE rt.Start(): the loop reads odom_sink_ without a lock
  // (see SetOdomSink), so binding it after the thread exists would be a race.
  std::unique_ptr<quadruped::Uplink> uplink;
  try {
    uplink.reset(new quadruped::Uplink(cfg.uplink, cfg.robot_id));
    // Read BACK from the context, never echoed from the config -- an
    // implementation that took the domain from ROS_DOMAIN_ID would print 42
    // here while publishing somewhere nobody listens (13 DDS-1).
    std::printf("uplink: ROS 2 domain %d, odom on %s\n",
                uplink->actual_domain_id(), cfg.uplink.odom_topic.c_str());
    quadruped::Uplink* up = uplink.get();
    rt.SetOdomSink([up](const quadruped::OdomSample& s, double wall_ts_s) {
      up->Publish(s, wall_ts_s);
    });
  } catch (const std::exception& e) {
    // Reported and the process continues. Channel three is the way the upper
    // stack SEES the robot; losing it is serious, but it does not make the
    // chassis less safe, and refusing to start here would take Tier 1 down
    // with it. Same reasoning as the RT-plane branch below.
    std::fprintf(stderr,
                 "quadruped_m20: uplink (channel three) did NOT come up (%s). "
                 "No odom and no TF will be published; the chassis link and "
                 "Tier 1 are unaffected.\n", e.what());
  }
#endif
  std::string rt_err;
  const bool rt_up = rt.Start(&rt_err);
  if (rt_up) {
    // 13 S7.1 Q-5 / ASM-4 (3): the four chassis report streams onto their own
    // keys. Bound AFTER Start so the bridge exists and its publishers are
    // declared -- binding earlier would hand the rx thread a null bridge.
    //
    // *** This binding is what ASM-4 (3) recorded as "v1.15 已做" while
    // SetReportSink had ZERO production call sites. All four keys were
    // declared, all four writers implemented and tested, and not one frame
    // ever went out: subscribing to xbrain/dev/rt/chassis/** for 12 s on
    // 2026-09-18 returned only rt/chassis/state.
    quadruped::rt::RtBridge* bridge = rt.bridge_mut();
    // 13 CB-4 / DDS-9 / TF-1 all require the EFFECTIVE transport values in
    // hello_ack.runtime, and they give the same reason: a wrong domain, a
    // wrong codebook, or an unannounced frame assignment each fail as
    // "connected, receiving nothing" -- indistinguishable from a dead network
    // unless the process says what it actually bound to.
    //
    // The endpoint here is the FIRST enabled candidate, i.e. what will be
    // probed first, not what is live -- the handshake can arrive before any
    // link is up. 13 S8.2's "首个能收到状态上报的即为生效端点" is about the
    // session's choice, which the state key reports separately.
    std::string ep;
    for (const auto& c : cfg.link.endpoints) {
      if (!c.enabled) continue;
      ep = c.proto + "://" + c.host + ":" + std::to_string(c.port);
      break;
    }
    // drdds_available is a CONSTANT false, and the authority is 21 V-20
    // verbatim: "drdds_available 恒 false, 三项底盘能力只走主通道; 不得实现
    // 主通道失败自动切 drdds 的备选路径". The flag is what keeps those
    // fallback paths off, so a live value here would switch them on.
    //
    // It would also over-claim. chs_b reads /IMU and /MOTION_INFO through the
    // self-built package (13 v1.16), but the three capabilities V-20 names --
    // /NAV_CMD, /fault_aggregator, /GAIT -- are NOT built: 21 V-12 forbids
    // building /GAIT at all while its type name is unconfirmed. "The package
    // is available" would therefore be read as more than we have.
    bridge->SetTransport(ep, cfg.link.codebook, cfg.dds.domain_id,
                         cfg.uplink.ros_domain_id, cfg.dds.imu_frame_id,
                         /*drdds_available=*/false);
    // 11 S9.6: the static limits come from the RESOLVED config, never from the
    // chassis -- the contract is explicit that the chassis does not provide
    // them, and v0.1's mistake was putting them in the handshake as if it did.
    bridge->SetSpec(cfg.tier1.limits.holonomic, cfg.tier1.limits.max_vx_mps,
                    cfg.tier1.limits.max_vy_mps, cfg.tier1.limits.max_wz_radps);
    proc.SetReportSink([bridge](double now_mono_s,
                                const quadruped::chs_a::BasicStatus* b,
                                const quadruped::chs_a::MotionStatus* m,
                                const quadruped::chs_a::DeviceStatus* d,
                                const quadruped::chs_a::FaultReport* f) {
      bridge->PublishReports(now_mono_s, b, m, d, f);
    });
  }
  if (!rt_up) {
    // Reported, and the process keeps running. Without the RT plane it cannot
    // receive a command -- but it still holds the chassis safe, and Tier 1
    // still stops it, which is the half that matters when the upstream is gone.
    std::fprintf(stderr,
                 "quadruped_m20: RT plane did NOT come up (%s). The chassis "
                 "link and Tier 1 are running; no velocity command can be "
                 "received until this is fixed.\n",
                 rt_err.c_str());
  } else {
    std::printf("rt plane: connected to %s, %zu publishers, %zu subscribers\n",
                cfg.uplink.zenoh_rt_endpoint.c_str(),
                rt.session().publisher_count(), rt.session().subscriber_count());
    std::fflush(stdout);
  }
#else
  std::fprintf(stderr,
               "quadruped_m20: built WITHOUT the RT plane -- this binary "
               "cannot receive a velocity command. Install zenoh-c and "
               "rebuild.\n");
#endif

  // The link state as of the last line printed. Transitions are logged, steady
  // state is not: a line a second saying "still probing" is how a journal stops
  // being read. The FIRST comparison is against kProbing, which is the state
  // the session starts in, so a link that comes up prints exactly one line and
  // a link that never comes up prints exactly one too.
  quadruped::chs_a::ConnState said_conn = quadruped::chs_a::ConnState::kProbing;
  bool said_anything = false;
  bool said_priority = false;

  while (g_stop == 0 && proc.running()) {
    // The work is on the threads; this one waits. Polling once a second rather
    // than blocking on a condition keeps the shutdown path to one mechanism.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (proc.ctrl_priority_error() != 0 && !said_priority) {
      said_priority = true;
      std::fprintf(stderr,
                   "quadruped_m20: ctrl thread is NOT SCHED_FIFO (errno %d) "
                   "-- it runs at ordinary priority and will miss deadlines "
                   "under load (13 S9.1). Check LimitRTPRIO on the unit.\n",
                   proc.ctrl_priority_error());
    }

#if QUADRUPED_HAVE_RT
    // The RT plane, reported on the same transition-only basis as the chassis
    // link. The refusal REASON is the important half (13 RX-9): a publisher
    // that gets one field wrong produces a robot that never moves and a process
    // that looks healthy, and without this line that is indistinguishable from
    // nobody publishing at all.
    if (rt_up) {
      static std::uint64_t said_refused = 0;
      const std::uint64_t refused = rt.bridge().cmd_vel_refused();
      if (refused > 0 && said_refused == 0) {
        said_refused = refused;
        std::fprintf(stderr,
                     "quadruped_m20: cmd_vel REFUSED (%llu so far, first "
                     "reason: %s). The robot will not move until the publisher "
                     "is fixed -- 11 S3.0.1 refuses a loosening command that is "
                     "missing a mandatory field, and 11:1722 makes estop_epoch "
                     "mandatory on this key.\n",
                     static_cast<unsigned long long>(refused),
                     quadruped::rt::RtParseName(rt.bridge().first_refusal()));
      }
      static bool said_accepted = false;
      if (!said_accepted && rt.bridge().cmd_vel_accepted() > 0) {
        said_accepted = true;
        std::printf("quadruped_m20: cmd_vel accepted -- the RT plane path is "
                    "live (accepted=%llu)\n",
                    static_cast<unsigned long long>(
                        rt.bridge().cmd_vel_accepted()));
        std::fflush(stdout);
      }
    }
#endif

#if QUADRUPED_HAVE_CHS_B
    // *** The odometry sources, as COUNTS rather than as this tick's winner.
    //
    // Both linear sources are real measurements of the same body, so the pose
    // is right either way and the winner alternates: drdds runs at 20 Hz and
    // the monitor at 10, and on the chassis they trade periods (measured
    // 2026-09-17). A line that printed the winner would flap, and a diagnostic
    // that flaps is one people learn to ignore. The counts answer the question
    // that actually matters -- is the 200 Hz / 20 Hz source contributing --
    // and 13 S4.2 calls the other row the DEGRADED source, which nothing
    // downstream can see: both produce a pose, and the covariance model does
    // not know the difference.
    {
      static std::uint64_t said_at = 0;
      if (proc.ctrl_ticks() - said_at >= 500) {     // ~5 s at 100 Hz
        said_at = proc.ctrl_ticks();
        std::fprintf(stderr,
                     "quadruped_m20: odom periods -- angular drdds=%llu "
                     "monitor=%llu | linear drdds=%llu monitor=%llu "
                     "(imu rx=%llu, motion_info rx=%llu)\n",
                     static_cast<unsigned long long>(proc.angular_from_drdds()),
                     static_cast<unsigned long long>(proc.angular_from_monitor()),
                     static_cast<unsigned long long>(proc.linear_from_drdds()),
                     static_cast<unsigned long long>(proc.linear_from_monitor()),
                     static_cast<unsigned long long>(chs_b_up ? chs_b.imu_samples() : 0),
                     static_cast<unsigned long long>(
                         chs_b_up ? chs_b.motion_info_samples() : 0));
      }
    }
#endif

    // The link. Without this the process is silent while it cannot reach the
    // chassis, which is indistinguishable from working -- CLAUDE.md 3.2 calls
    // that "assuming a guarantee you do not have", and it is the reason a dead
    // chassis is usually diagnosed as a dead network.
    const quadruped::QuadrupedProcess::LinkStatus st = proc.link_status();
    if (!said_anything || st.conn != said_conn) {
      said_anything = true;
      said_conn = st.conn;
      std::fprintf(stderr,
                   "quadruped_m20: chassis link %s (endpoint=%d, probe_cycles="
                   "%llu, reports=%llu)\n",
                   quadruped::chs_a::ConnStateName(st.conn), st.active_endpoint,
                   static_cast<unsigned long long>(st.probe_cycles),
                   static_cast<unsigned long long>(st.frames));
    }
    // Refused frames. FR-5 asks for a `warn` on a length mismatch and this
    // process cannot emit events (11 RT-C4), so the honest substitute is to
    // make the count visible -- the framer had kept it since day one and
    // nothing read it.
    //
    // Only when it MOVES, and only every few seconds: a rate is the signal,
    // and a line per bad frame is how a noisy link drowns the thread that was
    // meant to report it. Silent on a healthy link, by construction.
    {
      static std::uint64_t said_dropped = 0;
      static std::uint64_t dropped_said_at = 0;
      if (st.dropped != said_dropped &&
          proc.ctrl_ticks() - dropped_said_at >= 500) {   // ~5 s at 100 Hz
        dropped_said_at = proc.ctrl_ticks();
        std::fprintf(stderr,
                     "quadruped_m20: framer REFUSED %llu frame(s) so far "
                     "(+%llu since last report) -- a rising rate means the "
                     "peer and we disagree about the frame format, not that "
                     "the link is slow (13 FR-5)\n",
                     static_cast<unsigned long long>(st.dropped),
                     static_cast<unsigned long long>(st.dropped -
                                                     said_dropped));
        said_dropped = st.dropped;
      }
    }
  }
#if QUADRUPED_HAVE_RT
  // The RT plane goes down first: its subscriptions call into the process, and
  // stopping the process while a callback is inside one is a use-after-free
  // waiting for the right timing.
  if (rt_up) rt.Stop();
#endif
#if QUADRUPED_HAVE_CHS_B
  // Same reason, same order.
  if (chs_b_up) chs_b.Stop();
#endif
  proc.Stop();
  // The axis count is printed because "the robot did not move" needs to be a
  // NUMBER, not an inference from "we did not send an enable". Tier 1 holding
  // zero and Tier 1 having been unlocked without anyone noticing look the same
  // from outside the process; this is the line that tells them apart.
#if QUADRUPED_HAVE_RT
  // T-ODOM-1's evidence (13 S11.1), printed after the loop has stopped so the
  // histogram is not read while it is being written. The criterion is
  // P99 <= 12 ms and max <= 20 ms over ten minutes UNDER LOAD; this line is
  // what a bench run reads off. overflow is printed alongside because a
  // non-zero count means the P99 figure fell back to the exact max rather
  // than naming a bucket -- a reader has to be able to see that happened.
  {
    const quadruped::TickStats& ts = rt.tick_stats();
    std::printf("quadruped_m20: rt_pub period -- samples=%llu p99=%.2f ms "
                "max=%.2f ms overflow=%llu (odom sent=%llu)\n",
                static_cast<unsigned long long>(ts.count()),
                ts.PercentileMs(0.99), ts.max_ms(),
                static_cast<unsigned long long>(ts.overflow()),
                static_cast<unsigned long long>(rt.odom_sent()));
  }
#endif
  std::printf("quadruped_m20: stopped after %llu control periods "
              "(axis frames sent=%llu, heartbeats=%llu, tx skipped=%llu, "
              "stop_reason=%s)\n",
              static_cast<unsigned long long>(proc.ctrl_ticks()),
              static_cast<unsigned long long>(proc.axis_frames_sent()),
              static_cast<unsigned long long>(proc.heartbeats_sent()),
              static_cast<unsigned long long>(proc.tx_skipped()),
              std::string(StopReasonName(proc.last_tier1().stop_reason)).c_str());
  return kExitOk;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    // Service invocation: run.
    try {
      return Run(quadruped::DefaultResolvedPath());
    } catch (const quadruped::ConfigError& e) {
      std::fprintf(stderr, "%s\n", e.what());
      return kExitConfig;
    } catch (const std::exception& e) {
      std::fprintf(stderr, "quadruped_m20: %s\n", e.what());
      return kExitConfig;
    }
  }
  const std::string arg1 = argv[1];
  if (arg1 == "-h" || arg1 == "--help") {
    PrintUsage(argv[0]);
    return kExitOk;
  }
  if (arg1 != "--selfcheck") {
    PrintUsage(argv[0]);
    return kExitUsage;
  }
  const std::string path =
      (argc >= 3) ? std::string(argv[2])
                  : std::string(quadruped::DefaultResolvedPath());
  try {
    const quadruped::QuadrupedConfig cfg = quadruped::LoadQuadrupedConfig(path);
    std::printf("config: %s\n", path.c_str());
    std::fputs(quadruped::DescribeConfig(cfg).c_str(), stdout);
    return kExitOk;
  } catch (const quadruped::ConfigError& e) {
    // The message already carries the dotted key path (CLAUDE.md 3.1). An
    // uncalibrated value reaching here is the DESIGNED outcome today:
    // common.spec.max_* are null pending V-01, so a real snapshot stops at
    // tier1.limits.max_vx_mps and names it.
    std::fprintf(stderr, "%s\n", e.what());
    return kExitConfig;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "quadruped_m20: %s\n", e.what());
    return kExitConfig;
  }
}
