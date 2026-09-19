/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rt_bridge.cc
 * Brief: Which message causes which call, and what gets acked
 *
 * Description:
 * No router, no chassis, no threads. The bridge's four handlers are called
 * directly with bytes and the acks are captured through the injected publisher,
 * so the cases that matter can be ASSERTED rather than inferred from a counter.
 *
 * The three that carry the most weight:
 *
 *   * "p1_motion's actual body is refused AND first_refusal names why". At
 *     20 Hz, a refused stream and an absent one look identical from outside:
 *     Tier 1 says "timeout" either way, the link is up either way, the chassis
 *     reports either way. first_refusal is the only thing that tells the two
 *     apart, so it is asserted, not just incremented.
 *   * "every malformed estop still stops, and is still acked". Truncated,
 *     wrong version, another robot's id -- each one, separately, because
 *     11 S3.0.1 waives validation for exactly these and 99 U75 accepts that a
 *     hostile payload can stop this robot.
 *   * "a duplicate inside 50 ms does NOT advance the generation and IS still
 *     acked". Both halves. Swallowing the ack too would make the sender retry,
 *     which is the storm the window exists to prevent (11 S9.12.6).
 */

#include "quadruped/rt_bridge.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace quadruped;       // NOLINT: test-local
using namespace quadruped::rt;   // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

const char* kRid = "gj-001";
const char* kBoot = "a1b2c3d4";

struct Sent {
  std::string key;
  std::string body;
};

QuadrupedConfig Cfg() {
  QuadrupedConfig c;
  c.robot_id = kRid;
  EndpointCandidate ep;
  ep.proto = "tcp";
  ep.host = "127.0.0.1";
  ep.port = 1;          // nothing there; the bridge does not need a chassis
  ep.tls = false;
  ep.enabled = true;
  c.link.endpoints = {ep};
  c.link.probe_timeout_ms = 2000;
  c.link.heartbeat_hz = 2.0;
  c.link.codebook = "hex32";
  c.link.resync_max_bytes = 4096;
  c.link.frame_assembly_timeout_ms = 500;
  c.link.partial_send_retry = 3;
  c.link.tcp_nodelay = true;
  c.link.axis_cmd_hz = 20.0;
  c.link.cmd_fail_threshold = 3;
  c.link.state_timeout_degraded_s = 1.0;
  c.link.state_timeout_lost_s = 3.0;
  c.link.reconnect_backoff_s = {0.5, 1.0, 5.0};
  c.tier1.limits.max_vx_mps = 2.0;
  c.tier1.limits.max_vy_mps = 1.0;
  c.tier1.limits.max_wz_radps = 0.8;
  c.tier1.limits.holonomic = true;
  c.tier1.cmd_timeout_ms = 200;
  c.tier1.control_loop_hz = 100.0;
  c.odom.publish_hz = 100.0;
  c.odom.sigma_v0_mps = 0.05;
  c.odom.a_max_mps2 = 2.5;
  c.odom.gyro_bias_radps = 0.008;
  c.odom.arw_rad_sqrt_s = 0.002;
  c.odom.trust_flat = 1.0;
  c.odom.trust_stair = 0.3;
  c.odom.stale_warn_ms = 150;
  c.odom.stale_invalid_ms = 300;
  c.odom.stale_stop_publish_ms = 1000;
  return c;
}

std::string Wrap(const std::string& body) {
  return std::string("{\"v\":1,\"rid\":\"") + kRid +
         "\",\"ts\":1789455340.125,\"mono\":812.5,\"boot\":\"" + kBoot +
         "\",\"seq\":7,\"src\":\"p1_motion\",\"ts_sync\":true,\"data\":" + body + "}";
}

bool Has(const std::string& hay, const std::string& needle) {
  return hay.find(needle) != std::string::npos;
}

}  // namespace

// The payload of the LAST message sent on one key, or empty when the key never
// appeared. Reading the last rather than scanning for any is deliberate: a case
// that publishes twice must be checked against what a subscriber would hold.
std::string FindLast(const std::vector<Sent>& sent, const std::string& key) {
  for (std::size_t i = sent.size(); i > 0; --i) {
    if (sent[i - 1].key == key) return sent[i - 1].body;
  }
  return std::string();
}

int main() {
  // ---- a conformant cmd_vel reaches the process --------------------------
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });

    const std::string good = Wrap(
        "{\"vx\":0.3,\"vy\":0.0,\"wz\":0.2,\"vz\":0.0,\"v_roll\":0.0,"
        "\"v_pitch\":0.0,\"estop_epoch\":0}");
    b.HandleCmdVel(1.0, good.c_str(), good.size());
    CHECK(b.cmd_vel_accepted() == 1);
    CHECK(b.cmd_vel_refused() == 0);
    CHECK(b.first_refusal() == RtParse::kOk);
    // cmd_vel has no ack key -- it is a 20 Hz stream, not a request.
    CHECK(sent.empty());
  }

  // ---- *** p1_motion's ACTUAL body: refused, and the reason is reportable -
  {
    QuadrupedProcess p(Cfg());
    RtBridge b(&p, kRid, kBoot,
               [](const std::string&, const char*, std::size_t) { return true; });

    const std::string p1 = Wrap(
        "{\"vx\":0.3,\"vy\":0.0,\"wz\":0.2,\"vz\":0.0,\"v_roll\":0.0,"
        "\"v_pitch\":0.0,\"gate\":{\"v_max\":2.0,\"limiter\":\"none\"}}");
    for (int i = 0; i < 20; ++i) b.HandleCmdVel(1.0 + 0.05 * i, p1.c_str(), p1.size());

    CHECK(b.cmd_vel_accepted() == 0);
    CHECK(b.cmd_vel_refused() == 20);
    // *** The operator-visible half. Without this, 20 refusals a second and a
    // silent publisher are the same observation.
    CHECK(b.first_refusal() == RtParse::kMissingField);
  }

  // ---- every malformed estop still stops, and is still acked -------------
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });

    const std::string good = Wrap("{\"cmd_id\":\"e-1\",\"action\":\"stop\"}");
    std::uint64_t epoch = p.estop_epoch();

    // Well formed.
    b.HandleEstop(1.0, good.c_str(), good.size());
    CHECK(p.estop_epoch() == epoch + 1);
    CHECK(sent.size() == 1);
    CHECK(sent.back().key == "rt/safety/estop/ack");
    CHECK(Has(sent.back().body, "\"result\": \"accepted\"") ||
          Has(sent.back().body, "\"result\":\"accepted\""));
    CHECK(Has(sent.back().body, "e-1"));

    // Truncated mid-object, 200 ms later so the dedup window is closed.
    epoch = p.estop_epoch();
    const std::string trunc = good.substr(0, good.size() / 2);
    b.HandleEstop(1.2, trunc.c_str(), trunc.size());
    CHECK(p.estop_epoch() == epoch + 1);    // *** it STILL stopped
    CHECK(sent.size() == 2);                // *** and still acked

    // An unknown envelope version. A loosening command would be refused.
    epoch = p.estop_epoch();
    std::string badv = good;
    badv.replace(badv.find("\"v\":1"), 5, "\"v\":9");
    b.HandleEstop(1.4, badv.c_str(), badv.size());
    CHECK(p.estop_epoch() == epoch + 1);
    CHECK(sent.size() == 3);
    CHECK(Has(sent.back().body, "e-1"));    // the ack can still name it

    // Another robot's id. 99 U75: a wrongful stop costs nothing.
    epoch = p.estop_epoch();
    std::string other = good;
    other.replace(other.find(kRid), std::string(kRid).size(), "gj-999");
    b.HandleEstop(1.6, other.c_str(), other.size());
    CHECK(p.estop_epoch() == epoch + 1);
    CHECK(sent.size() == 4);

    // Empty payload.
    epoch = p.estop_epoch();
    b.HandleEstop(1.8, "", 0);
    CHECK(p.estop_epoch() == epoch + 1);
    CHECK(sent.size() == 5);
    CHECK(b.estops_applied() == 5);
  }

  // ---- the 50 ms dedup: no new generation, no event, STILL acked ---------
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });
    const std::string stop = Wrap("{\"cmd_id\":\"e-2\",\"action\":\"stop\"}");

    b.HandleEstop(10.0, stop.c_str(), stop.size());
    const std::uint64_t after_first = p.estop_epoch();
    CHECK(b.estops_applied() == 1);

    // 20 ms later -- inside the window.
    b.HandleEstop(10.020, stop.c_str(), stop.size());
    CHECK(p.estop_epoch() == after_first);   // *** generation NOT advanced
    CHECK(b.estops_deduped() == 1);
    CHECK(sent.size() == 2);                 // *** and STILL acked
    CHECK(Has(sent.back().body, "duplicate"));

    // 60 ms after the first -- outside the window.
    b.HandleEstop(10.060, stop.c_str(), stop.size());
    CHECK(p.estop_epoch() == after_first + 1);
    CHECK(b.estops_applied() == 2);
    CHECK(sent.size() == 3);
  }

  // ---- rt/chassis/ctrl: enable clears the lock, and the ack reads back ---
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });

    // The opening silence locks Tier 1.
    p.CtrlTick(0.0);
    p.CtrlTick(0.5);
    CHECK(p.last_tier1().timeout_lock == true);

    const std::string en = Wrap("{\"cmd_id\":\"c-04\",\"action\":\"enable\"}");
    b.HandleChassisCtrl(0.6, en.c_str(), en.size());
    CHECK(b.ctrl_accepted() == 1);
    CHECK(sent.size() == 1);
    CHECK(sent.back().key == "rt/chassis/ctrl/ack");
    CHECK(Has(sent.back().body, "accepted"));
    CHECK(Has(sent.back().body, "enable"));
    // *** 11 CR-12: the ack carries the READ-BACK locks. At this instant the
    // lock is still on -- the enable has not been consumed by a control period
    // yet -- and saying otherwise would be the "ack = accepted means unlocked"
    // mistake the contract calls out by name.
    CHECK(Has(sent.back().body, "\"timeout_lock\": true") ||
          Has(sent.back().body, "\"timeout_lock\":true"));

    // The next period consumes it, with a fresh command, and the lock goes.
    const std::string good = Wrap(
        "{\"vx\":0.1,\"vy\":0.0,\"wz\":0.0,\"vz\":0.0,\"v_roll\":0.0,"
        "\"v_pitch\":0.0,\"estop_epoch\":0}");
    b.HandleCmdVel(0.61, good.c_str(), good.size());
    p.CtrlTick(0.62);
    CHECK(p.last_tier1().timeout_lock == false);
  }

  // ---- rt/chassis/ctrl: refusals are acked, with the RIGHT code ----------
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });

    // *** A DELETED action answers E_CAPABILITY, not E_SCHEMA. Telling an
    // operator "malformed" about a word that was valid last release sends them
    // hunting a typo instead of reading the release notes (11 S9.3.3 v0.3).
    for (const char* gone : {"soft_estop", "estop_release", "idle"}) {
      const std::string body =
          Wrap(std::string("{\"cmd_id\":\"c-9\",\"action\":\"") + gone + "\"}");
      b.HandleChassisCtrl(1.0, body.c_str(), body.size());
      CHECK(Has(sent.back().body, "E_CAPABILITY"));
      CHECK(Has(sent.back().body, "rejected"));
      CHECK(Has(sent.back().body, gone));   // the ack names what it refused
    }

    // A word that was never valid.
    const std::string fly = Wrap("{\"cmd_id\":\"c-9\",\"action\":\"fly\"}");
    b.HandleChassisCtrl(1.0, fly.c_str(), fly.size());
    CHECK(Has(sent.back().body, "E_SCHEMA"));

    // No cmd_id: a loosening command that cannot be acked or de-duplicated.
    const std::string noid = Wrap("{\"action\":\"enable\"}");
    b.HandleChassisCtrl(1.0, noid.c_str(), noid.size());
    CHECK(Has(sent.back().body, "E_SCHEMA"));
    CHECK(Has(sent.back().body, "anonymous"));
    CHECK(b.ctrl_accepted() == 0);
  }

  // ---- ping answers pong, and a malformed ping answers too ---------------
  //
  // 13 F-15: the far end reads silence as "the estop chain is dead" and
  // degrades the system to hold. Withholding the answer over a bad field would
  // turn a publisher's bug into a stopped robot.
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });

    const std::string ping = Wrap("{\"type\":\"ping\"}");
    b.HandlePing(5.0, ping.c_str(), ping.size());
    CHECK(b.pongs_sent() == 1);
    CHECK(sent.back().key == "rt/safety/probe/pong");
    CHECK(Has(sent.back().body, "pong"));

    b.HandlePing(6.0, "not json at all", 15);
    CHECK(b.pongs_sent() == 2);   // *** answered anyway
  }

  // ---- *** rt/chassis/state carries estop_epoch ---------------------------
  //
  // This is the field p1_motion is waiting on. 11:1722 makes it mandatory on
  // rt/motion/cmd_vel, and p1 cannot echo a generation nobody has told it;
  // 13 RX-3 and NEXT.md P7.3 (7) both record the order as "quadruped publishes
  // it first". So what is asserted is not that a state message went out -- it
  // is that the epoch in it TRACKS the process, including across a stop.
  {
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });

    p.CtrlTick(0.0);
    QuadrupedProcess::StateSnapshot snap;
    CHECK(p.TakeStateForPublish(&snap) == true);
    CHECK(snap.estop_epoch == p.estop_epoch());
    CHECK(b.PublishState(snap) == true);
    CHECK(sent.back().key == "rt/chassis/state");
    CHECK(b.states_published() == 1);
    // No command yet, so the age is reported as null rather than as a huge
    // number -- "never" and "very old" are different facts (11 S4.1).
    CHECK(Has(sent.back().body, "\"cmd_age_ms\": null") ||
          Has(sent.back().body, "\"cmd_age_ms\":null"));

    // A stop advances the generation, and the NEXT published state must show
    // it. A state that lagged here would keep p1 echoing the old generation,
    // and Tier 1 would hold zero forever waiting for a number that never comes.
    const std::string stop = Wrap("{\"cmd_id\":\"e-9\",\"action\":\"stop\"}");
    b.HandleEstop(1.0, stop.c_str(), stop.size());
    const std::uint64_t after = p.estop_epoch();
    CHECK(after >= 1);
    p.CtrlTick(1.01);
    CHECK(p.TakeStateForPublish(&snap) == true);
    CHECK(snap.estop_epoch == after);
    CHECK(b.PublishState(snap) == true);
    {
      char want[64];
      std::snprintf(want, sizeof(want), "\"estop_epoch\": %llu",
                    static_cast<unsigned long long>(after));
      char want2[64];
      std::snprintf(want2, sizeof(want2), "\"estop_epoch\":%llu",
                    static_cast<unsigned long long>(after));
      CHECK(Has(sent.back().body, want) || Has(sent.back().body, want2));
    }

    // The slot is a slot: taken once, and the newest wins.
    QuadrupedProcess::StateSnapshot again;
    CHECK(p.TakeStateForPublish(&again) == false);
    for (int i = 0; i < 5; ++i) p.CtrlTick(1.02 + 0.01 * i);
    CHECK(p.TakeStateForPublish(&again) == true);
    CHECK(p.TakeStateForPublish(&snap) == false);

    // With a command in hand the age becomes a number.
    const std::string good = Wrap(
        "{\"vx\":0.1,\"vy\":0.0,\"wz\":0.0,\"vz\":0.0,\"v_roll\":0.0,"
        "\"v_pitch\":0.0,\"estop_epoch\":0}");
    b.HandleCmdVel(2.0, good.c_str(), good.size());
    p.CtrlTick(2.05);
    CHECK(p.TakeStateForPublish(&snap) == true);
    CHECK(snap.cmd_age_ms > 40.0);
    CHECK(snap.cmd_age_ms < 60.0);
    // ...and the generation disagreement is visible while it lasts: the command
    // echoed 0 and the process now holds `after`.
    CHECK(snap.soft_estop_active == (after != 0));
  }

  // ---- the four report streams reach their own keys (13 S7.1 Q-5) --------
  {
    // 13 ASM-4 (3) recorded this forwarding as "v1.15 已做" while
    // SetReportSink had ZERO production call sites: all four keys declared,
    // all four writers implemented and tested, and not one frame ever sent.
    // Measured 2026-09-18 -- subscribing to xbrain/dev/rt/chassis/** for 12 s
    // returned ONLY rt/chassis/state.
    //
    // The case asserts the KEY each report lands on, because that is what was
    // missing. A test that only checked "the writer produces JSON" passed
    // throughout the whole period the feature did not exist.
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });
    chs_a::BasicStatus basic;
    basic.model = "CA9C";
    b.PublishReports(0.0, &basic, nullptr, nullptr, nullptr);
    CHECK(sent.size() == 1);
    CHECK(sent.back().key == "rt/chassis/basic");

    chs_a::MotionStatus motion;
    b.PublishReports(0.1, nullptr, &motion, nullptr, nullptr);
    CHECK(sent.back().key == "rt/chassis/motion");

    // The device report carries the batteries, so it ALSO triggers PowerState
    // (11 S4.2, 1 Hz). Two messages land here, not one.
    chs_a::DeviceStatus device;
    b.PublishReports(0.2, nullptr, nullptr, &device, nullptr);
    CHECK(sent.size() == 4);
    CHECK(sent[2].key == "rt/chassis/device");
    CHECK(sent[3].key == "rt/chassis/power");

    chs_a::FaultReport fault;
    b.PublishReports(0.3, nullptr, nullptr, nullptr, &fault);
    CHECK(sent.back().key == "rt/chassis/fault");
    CHECK(sent.size() == 5);

    // A null report publishes NOTHING. The rx thread hands one struct and
    // four nulls per frame; publishing empties for the other three would put
    // three fabricated messages on the bus for every real one.
    // mutant: drop the null guards -> sent.size() jumps -> red.
    const std::size_t before = sent.size();
    b.PublishReports(0.4, nullptr, nullptr, nullptr, nullptr);
    CHECK(sent.size() == before);
  }

  // ---- PowerState reaches rt/chassis/power at 1 Hz (11 S4.2 / 13 S7.1) ----
  {
    // The gap this closes: rt/chassis/power was declared in rt_keys.cc and
    // WritePowerState was fully implemented WITH tests, and nothing ever
    // called it. CHG-10's low-battery return reads soc_pct off state/power
    // (CR-5 relays this key), so the rule had no data source at all.
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });
    // The three pieces arrive on three DIFFERENT reports. Basic and motion
    // first, so the assembled message can be checked for their fields --
    // publishing only what the triggering report carries is the failure this
    // ordering is chosen to expose.
    chs_a::BasicStatus basic;
    basic.power_management = 1;          // single_battery (11 S9.8.1)
    basic.charge = 2;                    // charging
    b.PublishReports(0.0, &basic, nullptr, nullptr, nullptr);
    chs_a::MotionStatus motion;
    motion.remain_mile = 4.2;
    b.PublishReports(0.05, nullptr, &motion, nullptr, nullptr);

    chs_a::DeviceStatus device;
    chs_a::BatteryEntry b0;
    b0.level = 47;
    b0.voltage = 51.2;
    b0.present = true;
    device.batteries.push_back(b0);
    device.min_level = 47;
    device.present_count = 1;
    b.PublishReports(0.1, nullptr, nullptr, &device, nullptr);

    const std::string power = FindLast(sent, "rt/chassis/power");
    CHECK(!power.empty());
    // Fields from all three reports, which is the point of caching them.
    CHECK(power.find("\"soc_pct\":47") != std::string::npos);
    CHECK(power.find("\"remain_mile_km\":4.2") != std::string::npos);
    // From the SHARED closed set, not a literal in the writer (CLAUDE.md 3.5).
    CHECK(power.find("\"power_management\":\"single_battery\"") !=
          std::string::npos);
    // 11 S4.2 lists `charge`; it used to be absent from this object entirely.
    CHECK(power.find("\"charge\":\"charging\"") != std::string::npos);
    // 13 BAT-2: left/right stay null while the mapping is unknown (V-55).
    CHECK(power.find("\"battery_mapping\":\"unknown\"") != std::string::npos);
    CHECK(power.find("\"batteries\":null") != std::string::npos);

    // 1 Hz, on the CLOCK. A second device report 0.2 s later must NOT publish
    // again -- 13 S7.1 gives PowerState 1 Hz while the device stream runs at
    // 2 Hz, so a rate enforced by counting reports would double the moment the
    // chassis changed that cadence.
    // mutant: drop the deadline check -> a second power message appears.
    const std::size_t before = sent.size();
    b.PublishReports(0.2, nullptr, nullptr, &device, nullptr);
    CHECK(sent.size() == before + 1);          // the device report itself
    CHECK(sent.back().key == "rt/chassis/device");

    // And it DOES publish again once the period has passed. Without this the
    // assertion above is satisfied by an implementation that publishes once
    // and never again.
    b.PublishReports(1.3, nullptr, nullptr, &device, nullptr);
    CHECK(sent.back().key == "rt/chassis/power");

    // A charge value outside the closed set is NULL, never a nearby member.
    // 11 S13.6 bans degrading to something close, and `idle` in particular
    // tells the upper stack the robot is free to drive away from a dock.
    // mutant: answer kCharge[0] instead of null -> red.
    chs_a::BasicStatus odd;
    odd.charge = 9;                      // outside idle..on_dock_no_current
    odd.power_management = 7;            // outside normal / single_battery
    b.PublishReports(2.4, &odd, nullptr, nullptr, nullptr);
    b.PublishReports(2.5, nullptr, nullptr, &device, nullptr);
    const std::string odd_power = FindLast(sent, "rt/chassis/power");
    CHECK(odd_power.find("\"charge\":null") != std::string::npos);
    // power_management keeps the open-set form the writer already used for it
    // -- the raw value is preserved rather than dropped, which is the 13 S6.5
    // discipline, and the two fields differ deliberately: `charge` has no
    // unknown_ form in 11 S4.2's schema.
    CHECK(odd_power.find("\"power_management\":\"unknown_7\"") !=
          std::string::npos);
  }

  // ---- rt/chassis/light: accepted, and 13 V-47 refused -------------------
  {
    // The gap this closes: rt/chassis/light was declared in rt_keys.cc and
    // NOTHING subscribed to it -- quadruped has no light code at all, so the
    // chassis lamps and 13 C-07's custom patterns were unreachable from the
    // upper stack (11 P1-8 forwards this key from cmd/chassis/light).
    QuadrupedProcess p(Cfg());
    std::vector<Sent> sent;
    RtBridge b(&p, kRid, kBoot,
               [&sent](const std::string& k, const char* d, std::size_t n) {
                 sent.push_back({k, std::string(d, n)});
                 return true;
               });
    const std::string ok = Wrap(
        "{\"cmd_id\":\"l-01\",\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"blink\",\"color\":\"white\",\"cycle_s\":1},"
        "\"tail\":{\"pattern\":\"breath\",\"color\":\"green\",\"cycle_s\":2}}}");
    b.HandleLight(0.0, ok.data(), ok.size());
    // ACCEPTED, counted before the send and separately from it. There is no
    // chassis behind this fixture, so the send fails -- and that is precisely
    // the case 13 ASM-6 shows must stay distinguishable: a single counter
    // standing for both would report "accepted" for a frame nobody sent.
    CHECK(b.lights_accepted() == 1);
    CHECK(b.lights_refused() == 0);
    CHECK(b.light_send_failures() == 1);

    // 13 V-47: illumination is REFUSED. Not accepted, not silently dropped.
    const std::string illum = Wrap(
        "{\"cmd_id\":\"l-02\",\"illumination\":{\"front\":1,\"back\":0}}");
    b.HandleLight(0.1, illum.data(), illum.size());
    CHECK(b.lights_accepted() == 1);          // unchanged
    CHECK(b.lights_refused() == 1);

    // *** And a message carrying BOTH halves applies NEITHER. A sender that
    // asked for two things and got one has no way to learn which took effect,
    // which is worse than a refusal it can see.
    // mutant: apply custom anyway when illumination is present -> accepted
    // goes to 2.
    const std::string both = Wrap(
        "{\"cmd_id\":\"l-03\",\"illumination\":{\"front\":1},"
        "\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"solid\",\"color\":\"white\",\"cycle_s\":0},"
        "\"tail\":{\"pattern\":\"solid\",\"color\":\"white\",\"cycle_s\":0}}}");
    b.HandleLight(0.2, both.data(), both.size());
    CHECK(b.lights_accepted() == 1);          // still unchanged
    CHECK(b.lights_refused() == 2);

    // A malformed one is refused too, and nothing is published either way:
    // this key has no ack (11 S2.2.1 declares none), so the bridge must not
    // invent one.
    const std::string bad = Wrap("{\"cmd_id\":\"l-04\"}");
    b.HandleLight(0.3, bad.data(), bad.size());
    CHECK(b.lights_refused() == 3);
    CHECK(sent.empty());
  }

  if (g_failures == 0) {
    std::printf("ALL RT BRIDGE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT BRIDGE TEST(S) FAILED\n", g_failures);
  return 1;
}
