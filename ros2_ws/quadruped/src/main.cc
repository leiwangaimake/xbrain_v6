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

#include "quadruped/process.h"
#include "quadruped/quadruped_config.h"

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
      "Not yet started from here: the domain-0 DDS reader and the RT-plane\n"
      "publisher. The process drives and protects the chassis; it does not\n"
      "publish state upward yet.\n",
      argv0, quadruped::DefaultResolvedPath(),
      quadruped::DefaultResolvedPath());
}

// Set by the signal handler, read by the run loop. sig_atomic_t and volatile
// because that is the only thing a handler may portably touch, and because the
// alternative -- calling Stop() from the handler -- would join threads inside
// a signal context.
volatile std::sig_atomic_t g_stop = 0;

void OnSignal(int) { g_stop = 1; }

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
  }
  proc.Stop();
  std::printf("quadruped_m20: stopped after %llu control periods\n",
              static_cast<unsigned long long>(proc.ctrl_ticks()));
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
