/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: main.cc
 * Brief: chassis_relay process assembly -- two sessions, twelve forwards,
 *        one event thread, one housekeeping thread
 *
 * Description:
 * The cross-plane emergency-stop relay of 11 S1.1.4/S1.1.6 (3): subscribe
 * three general-plane command keys and forward them to the RT plane, and
 * nine RT-plane keys back the other way, rebuilding the envelope per
 * RT-C3.e. The forwarding itself lives in relay_core.cc; this file only
 * wires it: config -> audit gate -> sessions -> publishers -> core ->
 * subscribers -> threads.
 *
 * Thread map, and where each CRL lands (the long-form argument sits in
 * relay_core.h):
 *   * GEN session callback threads: CR-1/2/11 inbound -> rebuild -> Q0 put
 *     on RT publishers (drop congestion, never blocks). These threads carry
 *     the stop direction and NOTHING else -- CRL-6 holds by construction
 *     because the relay subscribes nothing but Q0 on this plane.
 *   * RT session callback threads: the nine upstream keys -> rebuild -> Q0
 *     and Q2 puts on GEN publishers (drop; never blocks), EXCEPT CR-9,
 *     whose sink writes a lock-free latest-value slot instead of putting.
 *   * event thread: drains that slot and does the ONE blocking-capable put
 *     (event/fault/chassis, Q3 per anti-pattern A-4). A stall here delays
 *     fault events only -- it cannot touch estop, states or the watchdog.
 *   * housekeeping thread: WATCHDOG=1 pings, per-event safety audit lines,
 *     the 60 s stats line. All logging in the process happens HERE (CRL-4:
 *     no blocking log on forward paths -- callbacks only bump atomics).
 *   * main thread: sleeps until SIGINT/SIGTERM, then tears down in reverse.
 *
 * Two orderings in Run() are load-bearing, not style:
 *   * every publisher on BOTH planes is declared before any subscriber, so
 *     the first estop arriving in the declaration window has somewhere to
 *     go;
 *   * both sessions are CLOSED (subscribers dropped) before Run returns.
 *     The core and the fault slot are declared after the sessions and so
 *     die FIRST on unwind; a callback firing in that window would use a
 *     destroyed core. Close() ends callbacks, making the unwind safe on
 *     every return path below the core's construction.
 *
 * Exit codes follow quadruped's vocabulary: 0 ok, 64 usage, 78 config
 * (EX_CONFIG -- missing key, unexpanded reference, audit mismatch, dead
 * router). 78 under systemd Restart=always means a visible crash loop,
 * which is the wanted failure mode for a misdeployed safety relay.
 */

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "chassis_relay/envelope_rebuild.h"
#include "chassis_relay/relay_config.h"
#include "chassis_relay/relay_core.h"
#include "chassis_relay/relay_keys.h"
#include "chassis_relay/relay_session.h"
#include "chassis_relay/sd_notify.h"
#include "xbrain/rtcomm/lockfree_slot.h"

namespace {

using chassis_relay::Direction;
using chassis_relay::kMaxForwardBytes;
using chassis_relay::kRebuildSlack;
using chassis_relay::kRelayCount;
using chassis_relay::kRelayTable;
using chassis_relay::RelayConfig;
using chassis_relay::RelayCore;
using chassis_relay::RelaySession;

constexpr int kExitOk = 0;
constexpr int kExitUsage = 64;   // EX_USAGE
constexpr int kExitConfig = 78;  // EX_CONFIG, same vocabulary as quadruped

// The transitional direct-read source; see relay_config.h for the note that
// moves this to data/run/resolved/chassis_relay.yaml when the freeze line
// covers this process.
constexpr const char* kDefaultConfigPath =
    "/opt/xbrain_v6/configs/chassis_relay.yaml";

// One buffered fault frame for the CR-9 handoff. Trivially copyable, as the
// slot requires; len bounds the meaningful prefix of bytes.
struct FaultFrame {
  std::uint32_t len;
  char bytes[kMaxForwardBytes + kRebuildSlack];
};
using FaultSlot = hachist::xbrain::rtcomm::LockfreeSlot<FaultFrame>;

// Set by SIGINT/SIGTERM; polled by main. sig_atomic_t is the only type the
// standard lets a handler touch.
volatile std::sig_atomic_t g_stop = 0;
void OnSignal(int) { g_stop = 1; }

// ts for rebuilt envelopes: 11 S3.0 defines the field as the WALL clock
// (Unix seconds UTC); this is one of the wall clock's three legitimate uses
// (cross-machine alignment). Every interval below uses steady_clock (CLK-C1).
double WallNowS() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

int Usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s [--selfcheck] [<config.yaml>]\n"
               "  --selfcheck   load config, run the whitelist audit gate,\n"
               "                print the effective table and exit\n"
               "  default config: %s\n",
               argv0, kDefaultConfigPath);
  return kExitUsage;
}

// Print the effective configuration and the whole table once at startup --
// the cheap way to tell a misconfigured relay from a dead router later
// (same practice as quadruped's DescribeConfig).
void PrintEffective(const RelayConfig& cfg) {
  std::printf("chassis_relay: rid=%s gen=%s rt=%s audit=%s\n",
              cfg.robot_id.c_str(), cfg.gen_endpoint.c_str(),
              cfg.rt_endpoint.c_str(), cfg.whitelist_audit_path.c_str());
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const auto& row = kRelayTable[i];
    std::printf("  %-6s %s  %s %s %s  [%s]%s\n", row.cr_id,
                row.direction == Direction::kGenToRt ? "GEN->RT " : "RT ->GEN",
                row.gen_key,
                row.direction == Direction::kGenToRt ? "->" : "<-",
                row.rt_suffix, row.qos_profile,
                row.estop_exempt ? " (estop-exempt)" : "");
  }
}

// The audit gate (CRL-3 double insurance): the hardcoded table is the law;
// the generated registry must agree or the process refuses to start.
bool RunAuditGate(const RelayConfig& cfg) {
  try {
    const chassis_relay::WhitelistAudit audit =
        chassis_relay::LoadWhitelistAudit(cfg.whitelist_audit_path);
    const std::string diff = chassis_relay::CompareWhitelistAudit(audit);
    if (!diff.empty()) {
      std::fprintf(stderr,
                   "chassis_relay: whitelist audit MISMATCH against %s\n%s"
                   "refusing to start (CRL-3; fix the generator or the "
                   "contract table, never this binary)\n",
                   cfg.whitelist_audit_path.c_str(), diff.c_str());
      return false;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "chassis_relay: whitelist audit failed: %s\n",
                 e.what());
    return false;
  }
  std::printf("chassis_relay: whitelist audit ok (12 rows, pub 9 / sub 3)\n");
  return true;
}

// Declare the twelve output publishers, RT plane targets first (they are
// where an estop lands). Fills pub[] with per-row handles and *fault_row
// with the CR-9 index. False + message on any failure.
bool DeclareAllPublishers(const RelayConfig& cfg, RelaySession* gen,
                          RelaySession* rt, int* pub, std::size_t* fault_row) {
  std::string err;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const auto& row = kRelayTable[i];
    if (row.direction == Direction::kGenToRt) {
      const std::string key =
          chassis_relay::BuildRtKey(cfg.robot_id, row.rt_suffix);
      pub[i] = rt->DeclarePublisher(key, row.qos_profile, &err);
    } else {
      // Deployed general-plane spelling is BARE (see relay_keys.h).
      pub[i] = gen->DeclarePublisher(row.gen_key, row.qos_profile, &err);
    }
    if (pub[i] < 0) {
      std::fprintf(stderr, "chassis_relay: %s\n", err.c_str());
      return false;
    }
    if (std::strcmp(row.cr_id, "CR-9") == 0) *fault_row = i;
  }
  return true;
}

// Subscribe all twelve inputs, RT side first so the GEN estop inputs come
// up last, when everything downstream of them already exists.
bool DeclareAllSubscribers(const RelayConfig& cfg, RelaySession* gen,
                           RelaySession* rt, RelayCore* core) {
  std::string err;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const auto& row = kRelayTable[i];
    // i by value: the lambda outlives this loop inside the session.
    const auto cb = [core, i](const char* bytes, std::size_t len) {
      core->OnSample(i, bytes, len, WallNowS());
    };
    const bool ok =
        (row.direction == Direction::kRtToGen)
            ? rt->DeclareSubscriber(
                  chassis_relay::BuildRtKey(cfg.robot_id, row.rt_suffix), cb,
                  &err)
            : gen->DeclareSubscriber(row.gen_key, cb, &err);
    if (!ok) {
      std::fprintf(stderr, "chassis_relay: %s\n", err.c_str());
      return false;
    }
  }
  return true;
}

// The event thread body: the one blocking-capable put in the process
// (event/fault/chassis is Q3 "block" per anti-pattern A-4). 20 ms polling
// bounds the added latency at a fraction of the 2 Hz fault period; the slot
// keeps only the newest frame, which for a full-list fault stream is the
// correct staleness behaviour (relay_core.h).
void EventLoop(const std::atomic<bool>* run, FaultSlot* slot,
               RelaySession* gen, int fault_handle) {
  FaultFrame f;
  while (run->load(std::memory_order_relaxed)) {
    if (slot->TakeFresh(&f)) {
      // A refused put is counted by the session; the frame is superseded by
      // the next 2 Hz publication rather than retried (a retry loop here
      // would stall the drain and grow the coalescing window).
      gen->Put(fault_handle, f.bytes, f.len);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
}

// One safety audit pass: a line per event-class Q0 row whose forward count
// moved since the last pass (estop, ctrl, both acks -- the 1 Hz probe pair
// would flood the journal and lives in the 60 s stats instead). Runs on the
// housekeeping thread; the forward paths only bump the atomics this reads.
void PrintSafetyAudit(const RelayCore& core, std::uint64_t* seen) {
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const auto& row = kRelayTable[i];
    if (std::strcmp(row.qos_profile, "Q0_safety") != 0) continue;
    if (std::strncmp(row.gen_key, "probe/", 6) == 0) continue;
    const auto& st = core.stats(i);
    const std::uint64_t n = st.forwarded.load(std::memory_order_relaxed) +
                            st.forwarded_raw.load(std::memory_order_relaxed);
    if (n == seen[i]) continue;
    std::printf("safety forward: %s %s total=%llu (+%llu, raw=%llu)\n",
                row.cr_id, row.gen_key, static_cast<unsigned long long>(n),
                static_cast<unsigned long long>(n - seen[i]),
                static_cast<unsigned long long>(
                    st.forwarded_raw.load(std::memory_order_relaxed)));
    std::fflush(stdout);
    seen[i] = n;
  }
}

// The 60 s stats line: rx/forwarded per row plus the session-level drop and
// refusal counters, one line so journal grep stays cheap.
void PrintStats(const RelayCore& core, const RelaySession& gen,
                const RelaySession& rt) {
  std::printf("stats: total_rx=%llu",
              static_cast<unsigned long long>(core.total_rx()));
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const auto& st = core.stats(i);
    std::printf(" %s=%llu/%llu", kRelayTable[i].cr_id,
                static_cast<unsigned long long>(
                    st.rx.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    st.forwarded.load(std::memory_order_relaxed) +
                    st.forwarded_raw.load(std::memory_order_relaxed)));
  }
  std::printf(" oversize=%llu put_fail=%llu\n",
              static_cast<unsigned long long>(gen.samples_oversize() +
                                              rt.samples_oversize()),
              static_cast<unsigned long long>(gen.put_failures() +
                                              rt.put_failures()));
  std::fflush(stdout);
}

// The housekeeping thread body: READY once, WATCHDOG=1 every second (the
// unit carries WatchdogSec; a hung process misses the ping and systemd
// restarts it -- CRL-4's supervision), audit lines at 10 Hz granularity,
// stats every 60 s. This thread never touches zenoh, so no put can stall
// the watchdog.
void HousekeepingLoop(const std::atomic<bool>* run, const RelayCore* core,
                      const RelaySession* gen, const RelaySession* rt) {
  using clock = std::chrono::steady_clock;  // CLK-C1: intervals, never wall
  auto last_wd = clock::now();
  auto last_stats = clock::now();
  std::uint64_t seen[kRelayCount] = {};
  chassis_relay::SdNotify("READY=1");
  while (run->load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto now = clock::now();
    if (now - last_wd >= std::chrono::seconds(1)) {
      chassis_relay::SdNotify("WATCHDOG=1");
      last_wd = now;
    }
    PrintSafetyAudit(*core, seen);
    if (now - last_stats >= std::chrono::seconds(60)) {
      PrintStats(*core, *gen, *rt);
      last_stats = now;
    }
  }
}

// The wired process. Both sessions are explicitly Close()d on every return
// path below the core's construction -- see the header on why the unwind
// order alone would be a use-after-free window.
int Run(const RelayConfig& cfg) {
  RelaySession gen;
  RelaySession rt;
  std::string err;
  // RT first: its publishers are the estop targets; a dead router exits 78
  // and systemd restarts the process into a VISIBLE loop.
  if (!rt.Open(cfg.rt_endpoint, &err) || !gen.Open(cfg.gen_endpoint, &err)) {
    std::fprintf(stderr, "chassis_relay: %s\n", err.c_str());
    return kExitConfig;
  }

  int pub[kRelayCount];
  std::size_t fault_row = kRelayCount;
  if (!DeclareAllPublishers(cfg, &gen, &rt, pub, &fault_row)) {
    return kExitConfig;
  }

  // The CR-9 handoff slot. Heap allocated ONCE at startup; the forward path
  // only memcpys into it (CRL-4 forbids allocation on forwards, not here).
  auto fault_slot = std::make_unique<FaultSlot>();
  FaultSlot* slot = fault_slot.get();

  // The sink: every row puts directly on its output plane except the fault
  // row, which hands off to the event thread. Wired once; calls are
  // allocation-free.
  RelayCore core([&gen, &rt, &pub, fault_row, slot](
                     std::size_t i, const char* bytes, std::size_t len) {
    if (i == fault_row) {
      FaultFrame f;
      f.len = static_cast<std::uint32_t>(len);
      std::memcpy(f.bytes, bytes, len);
      slot->Publish(f);
      return true;  // accepted for deferred publish
    }
    RelaySession& out =
        kRelayTable[i].direction == Direction::kGenToRt ? rt : gen;
    return out.Put(pub[i], bytes, len);
  });

  if (!DeclareAllSubscribers(cfg, &gen, &rt, &core)) {
    gen.Close();  // ends callbacks before core unwinds (header note)
    rt.Close();
    return kExitConfig;
  }
  std::printf("chassis_relay: forwarding up (%zu pubs, %zu subs)\n",
              gen.publisher_count() + rt.publisher_count(),
              gen.subscriber_count() + rt.subscriber_count());
  std::fflush(stdout);

  std::atomic<bool> run{true};
  std::thread event_thread(EventLoop, &run, slot, &gen, pub[fault_row]);
  std::thread housekeeping(HousekeepingLoop, &run, &core, &gen, &rt);

  // Sleep until a signal; the handler only flips the flag, so teardown
  // still runs (no _exit shortcuts on a process holding sessions).
  while (g_stop == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::printf("chassis_relay: signal received, shutting down\n");
  run.store(false, std::memory_order_relaxed);
  event_thread.join();
  housekeeping.join();
  gen.Close();  // drops subscribers -> no callback outlives the core
  rt.Close();
  return kExitOk;
}

}  // namespace

int main(int argc, char** argv) {
  bool selfcheck = false;
  std::string path = kDefaultConfigPath;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--selfcheck") == 0) {
      selfcheck = true;
    } else if (argv[i][0] == '-') {
      return Usage(argv[0]);
    } else {
      path = argv[i];
    }
  }

  RelayConfig cfg;
  try {
    cfg = chassis_relay::LoadRelayConfig(path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "chassis_relay: config: %s\n", e.what());
    return kExitConfig;
  }
  // XBRAIN_ROBOT_ID is the documented L5 override for the rid (see
  // configs/common.yaml's robot_id note); honouring the same mechanism the
  // Python stack does means one env var renames the whole robot coherently.
  const char* env_rid = std::getenv("XBRAIN_ROBOT_ID");
  if (env_rid != nullptr && env_rid[0] != '\0') {
    std::printf("chassis_relay: XBRAIN_ROBOT_ID overrides rid: %s\n", env_rid);
    cfg.robot_id = env_rid;
  }

  PrintEffective(cfg);
  if (!RunAuditGate(cfg)) return kExitConfig;
  if (selfcheck) {
    std::printf("chassis_relay: selfcheck ok\n");
    return kExitOk;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  return Run(cfg);
}
