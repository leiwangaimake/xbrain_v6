/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_chs_a_session.cc
 * Brief: Session policy -- probe order, TLS-4, aging, backoff, 13 S7.5, F-21
 *
 * Description:
 * Every case drives the session with an injected dialler and an explicit clock,
 * so a three-second timeout is tested in microseconds and nothing here sleeps.
 * A test that sleeps is slow on a quiet machine and flaky on a loaded one, and
 * the timings under test (1 s degraded, 3 s lost, a 5 s backoff rung) would
 * make a real-time suite take minutes.
 *
 * The cases that exist because the rule is easy to implement backwards:
 *
 *   * TLS-4 -- a candidate with no credentials is skipped WITHOUT SPENDING THE
 *     PROBE WINDOW. Asserted by advancing the clock not at all: the next
 *     candidate must be dialled on the same tick. An implementation that
 *     dialled anyway and waited would pass a test that only checked the final
 *     endpoint, and would turn "no certificate installed" into six seconds
 *     indistinguishable from an unplugged cable.
 *   * exactly one socket is open at a time (13 CA-1). A second live socket is a
 *     second CLIENT to the chassis, and axis commands then come back 0xE006 for
 *     two seconds -- accepted, and the robot does not move. The fake counts
 *     dials and hangups so an overlap is a failure and not a code reading.
 *   * an absent ACK is not a failure. 13 CA-7 measured that axis commands are
 *     never acknowledged; a session that counted missing acks would declare a
 *     healthy link degraded within a second of the first command.
 *   * the backoff ladder resets after a good connection. Without that, an hour
 *     of healthy link followed by one drop waits on the LAST rung, and the
 *     robot is out of contact for five seconds instead of half of one.
 *
 * One thing this file does NOT test, deliberately: that a real socket connects.
 * That is T-CHS-3's, needs a stub server or the chassis, and belongs to B9.
 */

#include "quadruped/chs_a_session.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace quadruped::chs_a;  // NOLINT: test-local, keeps the cases readable
using quadruped::ChassisLinkConfig;
using quadruped::EndpointCandidate;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

EndpointCandidate Ep(const char* proto, int port, bool tls, bool enabled) {
  EndpointCandidate e;
  e.proto = proto;
  e.host = "10.21.33.103";
  e.port = port;
  e.tls = tls;
  e.enabled = enabled;
  return e;
}

// Three candidates in the shape configs/quadruped.yaml ships: plaintext TCP,
// plaintext UDP, then the TLS pair that TLS-1 leaves disabled.
SessionConfig BaseConfig() {
  SessionConfig c;
  c.endpoints = {Ep("tcp", 30003, false, true), Ep("udp", 30004, false, true),
                 Ep("tcp", 30003, true, true)};
  c.probe_timeout_s = 2.0;
  c.heartbeat_period_s = 0.5;
  c.state_timeout_degraded_s = 1.0;
  c.state_timeout_lost_s = 3.0;
  c.cmd_fail_threshold = 3;
  c.reconnect_backoff_s = {0.5, 1.0, 5.0};
  return c;
}

// Records what the session asked the transport to do. Counting rather than
// flagging: "dialled twice without a hangup between" is the CA-1 violation, and
// a boolean cannot see it.
struct FakeLink {
  std::vector<int> dialled_ports;
  std::vector<bool> dialled_tls;
  int dials = 0;
  int hangups = 0;
  int open = 0;          // must never exceed 1
  int max_open = 0;
  bool dial_succeeds = true;
  bool creds_present = false;  // TLS-1 ships with no certificates installed

  Session::Dial dial() {
    return [this](const EndpointCandidate& e) {
      ++dials;
      dialled_ports.push_back(e.port);
      dialled_tls.push_back(e.tls);
      if (!dial_succeeds) return false;
      ++open;
      if (open > max_open) max_open = open;
      return true;
    };
  }
  Session::Hangup hangup() {
    return [this]() {
      ++hangups;
      if (open > 0) --open;
    };
  }
  Session::CredentialsReady creds() {
    return [this](const EndpointCandidate&) { return creds_present; };
  }
};

template <typename F>
bool Throws(F f) {
  try {
    f();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}

}  // namespace

int main() {
  // ---- the first enabled candidate that answers wins ----------------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    CHECK(s.state() == ConnState::kProbing);
    TickResult r = s.Tick(0.0);
    // The heartbeat goes out on the dialling tick: the chassis reports only to
    // an address already sending them, so waiting a period first would spend
    // that period of the probe window guaranteed to hear nothing.
    CHECK(r.send_heartbeat == true);
    CHECK(link.dials == 1);
    CHECK(link.dialled_ports[0] == 30003);
    s.OnReport(0.1);
    r = s.Tick(0.2);
    CHECK(r.connected == true);
    CHECK(s.state() == ConnState::kOk);
    CHECK(s.active_endpoint() == 0);
    CHECK(s.link_epoch() == 1);
    CHECK(s.motion_allowed() == true);
    CHECK(link.max_open == 1);
  }

  // ---- a disabled candidate is never dialled ------------------------------
  {
    SessionConfig c = BaseConfig();
    c.endpoints[0].enabled = false;
    FakeLink link;
    Session s(c, link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    CHECK(link.dials == 1);
    CHECK(link.dialled_ports[0] == 30004);  // straight to the second one
    CHECK(s.last_skip_reason() == SkipReason::kDisabled);
  }

  // ---- 13 TLS-4: no credentials, no dial, NO TIME SPENT -------------------
  {
    SessionConfig c = BaseConfig();
    c.endpoints = {Ep("tcp", 30003, true, true), Ep("tcp", 30003, false, true)};
    FakeLink link;
    link.creds_present = false;
    Session s(c, link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    // The clock has not moved and the plaintext candidate is already dialled.
    // An implementation that dialled the TLS candidate and waited out
    // probe_timeout_s would fail here -- and in the field it would make a
    // missing certificate look exactly like an unplugged cable.
    CHECK(link.dials == 1);
    CHECK(link.dialled_tls[0] == false);
    CHECK(link.dialled_ports[0] == 30003);
    CHECK(s.last_skip_reason() == SkipReason::kNoCredentials);
    // ...and with the certificates installed, the TLS candidate IS tried, so
    // the rule is "skip what cannot work" and not "never try TLS".
    FakeLink link2;
    link2.creds_present = true;
    Session s2(c, link2.dial(), link2.hangup(), link2.creds());
    s2.Tick(0.0);
    CHECK(link2.dials == 1);
    CHECK(link2.dialled_tls[0] == true);
  }

  // ---- a silent candidate is abandoned after the probe window -------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    CHECK(link.dials == 1);
    s.Tick(1.9);                      // still inside the 2 s window
    CHECK(link.dials == 1);
    s.Tick(2.1);                      // past it
    CHECK(link.dials == 2);
    CHECK(link.dialled_ports[1] == 30004);
    CHECK(s.last_skip_reason() == SkipReason::kNoReportInTime);
    // 13 CA-1: the previous socket was closed before the next was dialled.
    CHECK(link.hangups == 1);
    CHECK(link.max_open == 1);
  }

  // ---- every candidate silent: lost, then the ladder ----------------------
  {
    SessionConfig c = BaseConfig();
    c.endpoints = {Ep("tcp", 30003, false, true), Ep("udp", 30004, false, true)};
    FakeLink link;
    Session s(c, link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.Tick(2.1);   // first gave up, second dialled
    s.Tick(4.2);   // second gave up, list exhausted
    CHECK(s.state() == ConnState::kLost);
    CHECK(s.probe_cycles() == 1);
    CHECK(link.open == 0);   // nothing left open once the link is declared lost
    // First rung is 0.5 s: nothing happens before it, and the walk restarts
    // from the TOP of the list after it.
    const int before = link.dials;
    s.Tick(4.6);
    CHECK(link.dials == before);
    s.Tick(4.75);
    CHECK(link.dials == before + 1);
    CHECK(link.dialled_ports.back() == 30003);
  }

  // ---- the ladder climbs, and its last rung repeats -----------------------
  {
    SessionConfig c = BaseConfig();
    c.endpoints = {Ep("tcp", 30003, false, true)};
    c.probe_timeout_s = 1.0;
    FakeLink link;
    link.dial_succeeds = false;   // every dial fails immediately
    Session s(c, link.dial(), link.hangup(), link.creds());
    // Each Tick exhausts the one-candidate list at once, so each is one cycle.
    s.Tick(0.0);
    CHECK(s.state() == ConnState::kLost);
    CHECK(s.last_skip_reason() == SkipReason::kConnectFailed);
    // rung 0 = 0.5 s
    s.Tick(0.4);
    CHECK(s.probe_cycles() == 1);
    s.Tick(0.6);
    CHECK(s.probe_cycles() == 2);
    // rung 1 = 1.0 s
    s.Tick(1.4);
    CHECK(s.probe_cycles() == 2);
    s.Tick(1.7);
    CHECK(s.probe_cycles() == 3);
    // rung 2 = 5.0 s, and every rung after it
    s.Tick(6.0);
    CHECK(s.probe_cycles() == 3);
    s.Tick(6.8);
    CHECK(s.probe_cycles() == 4);
    s.Tick(11.0);
    CHECK(s.probe_cycles() == 4);
    s.Tick(11.9);
    CHECK(s.probe_cycles() == 5);
  }

  // ---- uplink aging: ok -> degraded -> lost -------------------------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    s.Tick(0.2);
    CHECK(s.state() == ConnState::kOk);
    s.Tick(0.9);                       // 0.8 s old, still current
    CHECK(s.state() == ConnState::kOk);
    s.Tick(1.3);                       // 1.2 s old
    CHECK(s.state() == ConnState::kDegraded);
    CHECK(s.motion_allowed() == false);
    // A report brings it back without any explicit recovery step: 13 S2.5
    // treats the gap itself as the state.
    s.OnReport(1.4);
    s.Tick(1.5);
    CHECK(s.state() == ConnState::kOk);
    // ...and past the lost threshold the socket is closed and the ladder
    // starts, because a link this stale is not a link.
    TickResult r = s.Tick(4.6);
    CHECK(s.state() == ConnState::kLost);
    CHECK(r.disconnected == true);
    CHECK(link.open == 0);
    CHECK(s.motion_allowed() == false);
  }

  // ---- a reconnect resets the ladder, and bumps the epoch -----------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    s.Tick(0.2);
    CHECK(s.link_epoch() == 1);
    s.Tick(4.0);                        // lost
    CHECK(s.state() == ConnState::kLost);
    TickResult again = s.Tick(4.6);     // first rung expired, dial again
    // *** The dialling tick must NOT declare the link up. The last report is
    // from before the drop, and a session that accepted it would call a dead
    // endpoint live -- then allow motion on a link that has said nothing since
    // t=0.1. The evidence has to be newer than the dial.
    CHECK(again.connected == false);
    CHECK(s.state() == ConnState::kProbing);
    CHECK(s.motion_allowed() == false);
    CHECK(s.link_epoch() == 1);
    s.OnReport(4.7);
    s.Tick(4.8);
    CHECK(s.state() == ConnState::kOk);
    // *** CON-05 / BIT-33: the link is back and that is NOT permission to
    // resume. The epoch moving is what tells the layer above to handshake
    // again -- the session offers no "resume" of its own.
    CHECK(s.link_epoch() == 2);
    // The ladder is back at rung 0, so the NEXT drop waits half a second and
    // not five. Without the reset a robot that has been up for hours goes out
    // of contact ten times longer than the config says.
    s.Tick(8.0);
    CHECK(s.state() == ConnState::kLost);
    const std::uint64_t cycles = s.probe_cycles();
    s.Tick(8.4);
    CHECK(s.probe_cycles() == cycles);
    s.Tick(8.6);
    s.OnReport(8.7);
    s.Tick(8.8);
    CHECK(s.state() == ConnState::kOk);
    CHECK(s.link_epoch() == 3);
  }

  // ---- send failures degrade; a success clears the run --------------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    s.Tick(0.2);
    CHECK(s.state() == ConnState::kOk);
    s.OnSendFailure(0.3);
    s.OnSendFailure(0.4);
    s.OnReport(0.45);
    s.Tick(0.5);
    CHECK(s.state() == ConnState::kOk);   // two of three
    s.OnSendFailure(0.6);
    s.OnReport(0.65);
    s.Tick(0.7);
    // 13 S2.5: stop sending axis commands, and say so. NOT "send zero" --
    // 13 S3.4 is explicit that a zero command is still a command, and this is
    // a link we can no longer place one on.
    CHECK(s.state() == ConnState::kDegraded);
    CHECK(s.motion_allowed() == false);
    CHECK(s.consecutive_send_failures() == 3);
    // CONSECUTIVE, so one good write clears the run. A cumulative counter
    // would eventually degrade a link that has been healthy for hours.
    s.OnSendSuccess();
    s.OnReport(0.75);
    s.Tick(0.8);
    CHECK(s.state() == ConnState::kOk);
    CHECK(s.motion_allowed() == true);
  }

  // ---- an absent ACK is not a failure ------------------------------------
  {
    // 13 CA-7, measured: the chassis neither answers an axis command nor logs
    // it. A session that counted silence would declare this healthy link
    // degraded after three commands.
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    for (int i = 1; i < 200; ++i) {
      const double t = 0.1 + 0.01 * i;
      s.OnReport(t);           // uplink healthy
      s.Tick(t);               // ...and we keep sending, hearing nothing back
    }
    CHECK(s.state() == ConnState::kOk);
    CHECK(s.consecutive_send_failures() == 0);
    CHECK(s.motion_allowed() == true);
  }

  // ---- heartbeat cadence -------------------------------------------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    int beats = 0;
    s.OnReport(0.0);
    // 100 Hz ctrl ticks for one second, heartbeat period 0.5 s (13 TX-5:
    // ctrl divides its own tick down rather than running a second timer).
    for (int i = 0; i <= 100; ++i) {
      const double t = 0.01 * i;
      s.OnReport(t);
      if (s.Tick(t).send_heartbeat) ++beats;
    }
    // One at t=0 (the dialling tick), one at 0.5, one at 1.0.
    CHECK(beats == 3);
  }

  // ---- 13 F-21: asleep means no motion command leaves at all --------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    s.Tick(0.2);
    CHECK(s.motion_allowed() == true);
    s.OnSleep(true);
    // The link is perfectly healthy; the chassis simply will not act. Sending
    // anyway costs five seconds per command before 0xE008 comes back, and the
    // protocol has no wake command at all -- so "send and handle the error"
    // is five seconds of believing we are driving, repeated.
    CHECK(s.asleep() == true);
    CHECK(s.motion_allowed() == false);
    CHECK(s.state() == ConnState::kOk);   // sleep is not a link fault
    s.OnSleep(false);
    CHECK(s.motion_allowed() == true);
  }

  // ---- 13 S7.5, code by code ---------------------------------------------
  {
    CHECK(ClassifyErrorCode(0x0000).success == true);
    for (std::uint32_t c = 0xE001; c <= 0xE005; ++c) {
      const ErrorDisposition d = ClassifyErrorCode(c);
      CHECK(d.our_encoding_bug == true);
      // 13 S7.5: these count toward cmd_fail_threshold. A peer that can parse
      // nothing we send is an unusable endpoint however healthy the socket is.
      CHECK(d.counts_toward_cmd_fail == true);
      CHECK(d.retry_once == false);   // retrying re-sends the same bad bytes
    }
    CHECK(ClassifyErrorCode(0xE006).second_client == true);
    CHECK(ClassifyErrorCode(0xE006).retry_once == false);
    CHECK(ClassifyErrorCode(0xE007).mode_switch_failed == true);
    CHECK(ClassifyErrorCode(0xE008).mode_switch_failed == true);
    CHECK(ClassifyErrorCode(0xE009).retry_once == true);
    CHECK(ClassifyErrorCode(0xE00A).capability == true);
    CHECK(ClassifyErrorCode(0xE00B).chassis_internal == true);
    // An unregistered code is reported, not ignored -- but it is NOT evidence
    // that the endpoint is unusable, so it does not count toward the threshold.
    CHECK(ClassifyErrorCode(0xE0FF).our_encoding_bug == true);
    CHECK(ClassifyErrorCode(0xE0FF).counts_toward_cmd_fail == false);
    CHECK(ClassifyErrorCode(0xE00A).counts_toward_cmd_fail == false);
  }

  // ---- E001..E005 reach the same threshold as a write failure -------------
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    s.Tick(0.2);
    s.OnErrorCode(0.3, 0xE002);
    s.OnErrorCode(0.4, 0xE002);
    s.OnErrorCode(0.5, 0xE002);
    s.OnReport(0.55);
    s.Tick(0.6);
    CHECK(s.state() == ConnState::kDegraded);
    // A success clears it, same as a good write.
    s.OnErrorCode(0.7, 0x0000);
    s.OnReport(0.75);
    s.Tick(0.8);
    CHECK(s.state() == ConnState::kOk);
  }

  // ---- 0xE00B: THREE IN A ROW, and a run that is broken does not count ----
  {
    FakeLink link;
    Session s(BaseConfig(), link.dial(), link.hangup(), link.creds());
    s.Tick(0.0);
    s.OnReport(0.1);
    s.Tick(0.2);
    s.OnErrorCode(0.3, 0xE00B);
    s.OnErrorCode(0.4, 0xE00B);
    // Something else in between. Without this reset the counter accumulates
    // across a whole session and degrades a link on three unrelated internal
    // errors hours apart.
    s.OnErrorCode(0.5, 0xE009);
    s.OnErrorCode(0.6, 0xE00B);
    s.OnErrorCode(0.7, 0xE00B);
    s.OnReport(0.75);
    s.Tick(0.8);
    CHECK(s.state() == ConnState::kOk);
    s.OnErrorCode(0.9, 0xE00B);
    s.OnReport(0.95);
    s.Tick(1.0);
    CHECK(s.state() == ConnState::kDegraded);
  }

  // ---- FromLinkConfig refuses values that would make the session degenerate
  {
    ChassisLinkConfig link;
    link.endpoints = {Ep("tcp", 30003, false, true)};
    link.probe_timeout_ms = 2000;
    link.heartbeat_hz = 2.0;
    link.state_timeout_degraded_s = 1.0;
    link.state_timeout_lost_s = 3.0;
    link.cmd_fail_threshold = 3;
    link.reconnect_backoff_s = {0.5, 1.0};
    const SessionConfig ok = SessionConfig::FromLinkConfig(link);
    CHECK(ok.heartbeat_period_s == 0.5);
    CHECK(ok.probe_timeout_s == 2.0);
    // A zero heartbeat rate is not "no heartbeats" -- it is "no reports",
    // because the chassis reports only to an address that keeps sending them.
    // It would present as a chassis that never answers.
    ChassisLinkConfig no_hb = link;
    no_hb.heartbeat_hz = 0.0;
    CHECK(Throws([&] { SessionConfig::FromLinkConfig(no_hb); }));
    ChassisLinkConfig no_ladder = link;
    no_ladder.reconnect_backoff_s.clear();
    CHECK(Throws([&] { SessionConfig::FromLinkConfig(no_ladder); }));
    ChassisLinkConfig no_probe = link;
    no_probe.probe_timeout_ms = 0;
    CHECK(Throws([&] { SessionConfig::FromLinkConfig(no_probe); }));
  }

  // ---- the state names are distinct ---------------------------------------
  {
    // A log that calls two states by the same word is worse than one that
    // prints a number, because it reads as if the transition never happened.
    const std::string names[] = {
        ConnStateName(ConnState::kProbing), ConnStateName(ConnState::kOk),
        ConnStateName(ConnState::kDegraded), ConnStateName(ConnState::kLost)};
    for (int i = 0; i < 4; ++i) {
      CHECK(!names[i].empty());
      for (int j = i + 1; j < 4; ++j) CHECK(names[i] != names[j]);
    }
  }

  if (g_failures == 0) {
    std::printf("ALL CHS_A_SESSION TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHS_A_SESSION TEST(S) FAILED\n", g_failures);
  return 1;
}
