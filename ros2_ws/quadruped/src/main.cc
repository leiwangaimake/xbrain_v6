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
    // chs_b's twin of the block above. Its accessor's comment has promised
    // this report since the day it was written ("Reported for the same
    // reason ctrl's is") -- and nothing ever read it: the one place in the
    // package where a comment claimed behaviour the code did not have.
    static bool said_chs_b_priority = false;
    if (chs_b_up && chs_b.priority_error() != 0 && !said_chs_b_priority) {
      said_chs_b_priority = true;
      std::fprintf(stderr,
                   "quadruped_m20: chs_b thread is NOT SCHED_FIFO (errno %d) "
                   "-- the 200 Hz DDS reader runs at ordinary priority "
                   "(13 S9.1). Check LimitRTPRIO on the unit.\n",
                   chs_b.priority_error());
    }

#if QUADRUPED_HAVE_RT
    // The RT plane, reported on the same transition-only basis as the chassis
    // link. The refusal REASON is the important half (13 RX-9): a publisher
    // that gets one field wrong produces a robot that never moves and a process
    // that looks healthy, and without this line that is indistinguishable from
    // nobody publishing at all.
    if (rt_up) {
      // *** RT-plane PUT failures. The session has counted them since it was
      // written and nothing ever read the counter -- a counter with no reader
      // is a number, not a diagnostic.
      //
      // What it costs when it is invisible: every state key this process
      // publishes goes out through one put. If those are failing, the upper
      // stack sees a robot that reports nothing, and the process itself looks
      // perfectly healthy from the inside -- the same "connected, receiving
      // nothing" shape 13 DDS-9 names, one layer up. Reported only when the
      // count MOVES, and at most every few seconds, because a rate is the
      // signal and a line per failed put would bury it.
      {
        static std::uint64_t said_puts = 0;
        static std::uint64_t puts_said_at = 0;
        const std::uint64_t failed = rt.session().put_failures();
        if (failed != said_puts && proc.ctrl_ticks() - puts_said_at >= 500) {
          puts_said_at = proc.ctrl_ticks();
          std::fprintf(stderr,
                       "quadruped_m20: RT plane PUT FAILED %llu time(s) so far "
                       "(+%llu since last report, %llu succeeded). Every state "
                       "key leaves through one put -- while these fail the "
                       "upper stack sees a silent robot and this process looks "
                       "healthy from the inside.\n",
                       static_cast<unsigned long long>(failed),
                       static_cast<unsigned long long>(failed - said_puts),
                       static_cast<unsigned long long>(rt.session().puts_sent()));
          said_puts = failed;
        }
      }
      // *** Mode-step refusals, with their REASON. 13 MS-3a: rt/chassis/mode
      // has no ack key and RT-C4 keeps this process off the plane where the
      // cmd acks live, so this line is the only place a refusal surfaces --
      // and until it existed, GS-1 refusing stair_standard and MS-3 refusing
      // a second switch produced exactly the same nothing.
      //
      // Measured 2026-09-21 on the live chassis: a stair_standard request was
      // correctly refused and NOTHING said so anywhere.
      {
        static std::uint64_t said_mode = 0;
        const std::uint64_t refused = proc.mode_steps_refused();
        if (refused != said_mode) {
          said_mode = refused;
          std::fprintf(stderr,
                       "quadruped_m20: mode step REFUSED (%llu so far, last "
                       "reason: %s). This key has no ack (13 MS-3a), so the "
                       "requester sees nothing -- a refused switch and a switch "
                       "nobody sent look identical from there.\n",
                       static_cast<unsigned long long>(refused),
                       quadruped::ModeRejectItem(proc.last_mode_reject()));
        }
      }
      // *** The RT plane's PARSE-layer refusals, aggregated. Five of these
      // counters had been incrementing since the day they were written with
      // no reader anywhere -- a publisher with a wrong envelope field saw
      // its commands vanish and this process looked healthy from inside
      // (the same closing argument as put_failures above). One line, on
      // change only, all six: which stream is being refused is the half a
      // bench engineer actually needs.
      {
        static std::uint64_t said_refuse_sum = 0;
        const std::uint64_t refuse_sum =
            rt.bridge().ctrl_refused() + rt.bridge().mode_refused() +
            rt.bridge().hello_refused() + rt.bridge().lights_refused() +
            rt.bridge().clock_refused() + rt.bridge().estops_deduped();
        if (refuse_sum != said_refuse_sum) {
          said_refuse_sum = refuse_sum;
          std::fprintf(stderr,
                       "quadruped_m20: rt refusals -- ctrl=%llu mode=%llu "
                       "hello=%llu light=%llu clock=%llu estop_dedup=%llu "
                       "(dedup is not an error: 11 S9.12.6 swallows repeats "
                       "inside 50 ms and still acks)\n",
                       static_cast<unsigned long long>(rt.bridge().ctrl_refused()),
                       static_cast<unsigned long long>(rt.bridge().mode_refused()),
                       static_cast<unsigned long long>(rt.bridge().hello_refused()),
                       static_cast<unsigned long long>(rt.bridge().lights_refused()),
                       static_cast<unsigned long long>(rt.bridge().clock_refused()),
                       static_cast<unsigned long long>(rt.bridge().estops_deduped()));
        }
      }
      // Soft estops, the moment they land: 11 S3.0.1 makes this the one
      // command a hostile payload may legally deliver, so every application
      // is worth a line of its own.
      {
        static std::uint64_t said_estops = 0;
        if (rt.bridge().estops_applied() != said_estops) {
          said_estops = rt.bridge().estops_applied();
          std::fprintf(stderr,
                       "quadruped_m20: SOFT ESTOP applied (%llu so far, "
                       "epoch=%llu). Zero velocity until a fresh cmd_vel "
                       "carries the new epoch (11 S9.12.2).\n",
                       static_cast<unsigned long long>(said_estops),
                       static_cast<unsigned long long>(proc.estop_epoch()));
        }
      }
      // The envelope that did not fit. Unreachable by construction today
      // (see Publish) -- which is exactly why a nonzero here must be loud:
      // it means one of the two capacity constants was changed without the
      // other.
      {
        static bool said_env_overflow = false;
        if (rt.bridge().envelope_overflows() > 0 && !said_env_overflow) {
          said_env_overflow = true;
          std::fprintf(stderr,
                       "quadruped_m20: %llu payload(s) TOO BIG to wrap in the "
                       "11 S3.0 envelope and dropped. kOutCap grew past "
                       "kEnvCap's slack; nothing on this key reached the "
                       "wire.\n",
                       static_cast<unsigned long long>(
                           rt.bridge().envelope_overflows()));
        }
      }
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

#if QUADRUPED_HAVE_RT
    // Once a minute, the RT plane's throughput in one line -- the accepted
    // side of every stream, plus the three transport counters that only
    // matter as RATES: resync bytes (FR-2 -- a nonzero rate means we and the
    // peer disagree about the framing), and the tx-guard triple (TX-3 --
    // skips are safe by design, but a skip RATE near the send rate means the
    // realtime path almost never gets the line). "It is publishing" becomes
    // a number rather than an impression, which is the same sentence
    // uplink.h wrote about its own counters.
    if (rt_up) {
      static std::uint64_t stats_said_at = 0;
      if (proc.ctrl_ticks() - stats_said_at >= 6000) {   // ~60 s at 100 Hz
        stats_said_at = proc.ctrl_ticks();
        const quadruped::QuadrupedProcess::LinkStatus st = proc.link_status();
        std::fprintf(stderr,
                     "quadruped_m20: rt stats -- states=%llu acks=%llu "
                     "pongs=%llu ping_no_seq=%llu hello=%llu ctrl=%llu mode=%llu light=%llu "
                     "light_send_fail=%llu clock=%llu | resync_bytes=%llu "
                     "tx skip/acq/sent=%llu/%llu/%llu\n",
                     static_cast<unsigned long long>(rt.bridge().states_published()),
                     static_cast<unsigned long long>(rt.bridge().acks_sent()),
                     static_cast<unsigned long long>(rt.bridge().pongs_sent()),
                     // 11 S8.5: non-zero means a publisher is omitting
                     // data.seq, which reads on the p5 side exactly like a
                     // dead link. Printed so the two are separable at a glance.
                     static_cast<unsigned long long>(rt.bridge().pings_without_seq()),
                     static_cast<unsigned long long>(rt.bridge().hello_answered()),
                     static_cast<unsigned long long>(rt.bridge().ctrl_accepted()),
                     static_cast<unsigned long long>(rt.bridge().mode_accepted()),
                     static_cast<unsigned long long>(rt.bridge().lights_accepted()),
                     static_cast<unsigned long long>(rt.bridge().light_send_failures()),
                     static_cast<unsigned long long>(rt.bridge().clock_accepted()),
                     static_cast<unsigned long long>(st.resync_bytes),
                     static_cast<unsigned long long>(st.tx_skips),
                     static_cast<unsigned long long>(st.tx_acquires),
                     static_cast<unsigned long long>(st.tx_sent));
#if QUADRUPED_HAVE_UPLINK
        if (uplink) {
          std::fprintf(stderr,
                       "quadruped_m20: uplink stats -- odom=%llu tf=%llu "
                       "withheld=%llu\n",
                       static_cast<unsigned long long>(uplink->odom_published()),
                       static_cast<unsigned long long>(uplink->tf_published()),
                       static_cast<unsigned long long>(uplink->suppressed()));
        }
#endif
      }
    }
#endif

#if QUADRUPED_HAVE_UPLINK
    // 13 S4.4 (4): past one second of staleness the odometry AND the TF stop
    // going out. That is the intended fail-safe -- a frozen TF makes Nav2
    // believe the robot is stationary and keep commanding rotation -- but it
    // is also invisible from outside, because the symptom IS silence. The
    // counter existed and nothing read it.
    if (uplink) {
      static std::uint64_t said_suppressed = 0;
      static std::uint64_t supp_said_at = 0;
      const std::uint64_t supp = uplink->suppressed();
      if (supp != said_suppressed && proc.ctrl_ticks() - supp_said_at >= 500) {
        supp_said_at = proc.ctrl_ticks();
        std::fprintf(stderr,
                     "quadruped_m20: odom/TF WITHHELD %llu tick(s) so far "
                     "(+%llu since last report) -- 13 S4.4 (4) stops both past "
                     "one second of staleness. Nav2 will abort its behaviours "
                     "on the missing TF, which is the intended direction; the "
                     "cause is upstream of here (the velocity source), not in "
                     "the publisher.\n",
                     static_cast<unsigned long long>(supp),
                     static_cast<unsigned long long>(supp - said_suppressed));
        said_suppressed = supp;
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
    // *** Chassis response codes and failed mode switches. 13 S7.5 gives each
    // code a disposition and the session acted on it, then DISCARDED the code
    // -- so a chassis that refused a command left no trace at all.
    //
    // Measured 2026-09-21: a usage_mode switch to navigation was dispatched,
    // the chassis did not change mode, and nothing in the process could say
    // whether a frame had gone out, whether the chassis had answered, or what
    // it answered. 11 S9.10.1 has a specific candidate for that case (E_BUSY,
    // charge_manager still running) and it was unreachable from here.
    {
      static std::uint64_t said_err = 0;
      if (st.error_codes_seen != said_err) {
        said_err = st.error_codes_seen;
        std::fprintf(stderr,
                     "quadruped_m20: chassis answered 0x%04X (%llu non-success "
                     "code(s) so far). 13 S7.5 names the disposition; the code "
                     "names the problem -- E_BUSY on a mode switch is 11 "
                     "S9.10.1's charge_manager, not a framing fault.\n",
                     st.last_error_code,
                     static_cast<unsigned long long>(st.error_codes_seen));
      }
      static std::uint64_t said_sw = 0;
      if (st.switch_failures != said_sw) {
        said_sw = st.switch_failures;
        std::fprintf(stderr,
                     "quadruped_m20: mode switch TIMED OUT %llu time(s) -- the "
                     "read-back never matched inside the configured window "
                     "(13 MS-2). %llu mode frame(s) have been sent, so the "
                     "chassis heard us and did not follow.\n",
                     static_cast<unsigned long long>(st.switch_failures),
                     static_cast<unsigned long long>(st.mode_frames_sent));
      }
    }

    // FR-5 / SD-3: TCP_NODELAY, read back from the kernel on the live socket.
    // Said ONCE, because it is a property of the connection rather than a
    // rate. Before this nothing checked it at all -- the setsockopt return was
    // discarded and the read-back accessor had no caller, so "Nagle is off"
    // was a guarantee the process did not hold.
    //
    // *** Said UNCONDITIONALLY, both facts, once per connection -- not only
    // when they disagree. A line that appears only on failure cannot be used
    // to verify the check itself: silence means "compliant" and "this code
    // never ran" equally well, and on the bench 2026-09-21 there was no way
    // to tell the two apart from outside the process. That is the same defect
    // v1.25 fixed one layer down, where the single boolean verdict was split
    // into two facts precisely because no mutant could kill the verdict.
    // Splitting it in LinkStatus and then collapsing it again at the log
    // leaves the reporting end exactly where it started.
    {
      static bool said_nodelay = false;
      // Any state past probing means a socket is open and the read-back
      // is meaningful. kDegraded counts: the socket is up, the reports
      // are merely late, and the option is a property of the socket.
      if (st.active_endpoint >= 0 &&
          st.conn != quadruped::chs_a::ConnState::kProbing && !said_nodelay) {
        said_nodelay = true;
        // sndbuf rides the same once-per-connection line (13 SD-1). The value
        // is only REPORTED: the kernel default differs per platform, SD-1
        // forbids US enlarging it (and no setsockopt for it exists in this
        // package), so the number is for the bench ledger, not a verdict.
        std::fprintf(stderr,
                     "quadruped_m20: TCP_NODELAY read back from the kernel: "
                     "active=%s expected=%s sndbuf=%d (FR-5 / SD-3 / SD-1). "
                     "A mismatch means "
                     "Nagle batches the heartbeat with whatever follows it and "
                     "every latency figure in 13 S3.6 measures something else "
                     "-- the link works, its timing does not.\n",
                     st.nodelay_active ? "true" : "false",
                     st.nodelay_expected ? "true" : "false",
                     st.sndbuf_bytes);
      }
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
