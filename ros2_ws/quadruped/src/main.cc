/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: main.cc
 * Brief: quadruped_m20 entry -- B0 self-check only, refuses to run as a service
 *
 * Description:
 * What this binary is today. B0 built the package skeleton: the config loader,
 * the single-tx-owner seam, and this entry point. The three channels of 13 S2
 * do not exist yet (codec B1, link B2, Tier 1 B3, RT plane B4, DDS B7/B8), so
 * this program CANNOT control a chassis and does not pretend to.
 *
 * Why it refuses to idle. The obvious skeleton -- start, log "running", sleep
 * forever -- is the failure CLAUDE.md 3.2 calls "假装有保证": systemd would show
 * active, Stage 1 of 10 S3.3 would look satisfied, and p1_motion would sit
 * waiting for a hello_ack that nothing will ever send, presenting as a chassis
 * fault. So with no argument (which is how the unit invokes it) this binary
 * prints what it is and exits EX_CONFIG(78), and the unit carries
 * RestartPreventExitStatus=78 so the failure is one loud line rather than a
 * restart loop. The moment the process is real, both this branch and that unit
 * line go away together.
 *
 * What --selfcheck does. Loads a resolved snapshot and prints the effective
 * transport block (DDS-9: the two domain ids, the probe order, the codebook).
 * It is the only cheap way to tell "the two DDS domains collapsed into one"
 * from "the network is down" -- both present as a participant that is up and
 * receives nothing. It is also the first thing to run on the ORIN after a
 * freeze, because it answers "did my config actually expand" in one line.
 *
 * Exit codes are the sysexits convention so an operator and a script read the
 * same thing: 0 ok, 64 usage, 78 config (missing key / null / refusing to run
 * as a service). No other code is produced.
 */

#include <cstdio>
#include <cstring>
#include <string>

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
      "This binary is the B0 skeleton: config + tx seam + self-check. It does\n"
      "NOT talk to the chassis (channel one lands in B1/B2), so it refuses to\n"
      "run as a service rather than look alive while doing nothing.\n",
      argv0, quadruped::DefaultResolvedPath());
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    // Service invocation. Say plainly what is missing and stop; see the file
    // comment for why idling here would be worse than failing.
    std::fprintf(stderr,
                 "quadruped_m20: B0 skeleton -- the chassis link is not "
                 "implemented yet (13 S2.2, batch B1/B2), refusing to run as a "
                 "service. Run with --selfcheck to validate the config.\n");
    return kExitConfig;
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
