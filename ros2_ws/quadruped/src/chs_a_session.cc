/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_session.cc
 * Brief: Session state machine implementation (see chs_a_session.h)
 *
 * Description:
 * The state is six numbers and a state enum, and the reason it is not fewer is
 * that each one answers a question the others cannot:
 *
 *   last_report_s_     when the chassis last said anything. The ONLY liveness
 *                      evidence there is -- 13 CA-7 measured that axis commands
 *                      get no reply at all, so a silent downlink means nothing.
 *   probe_started_s_   when the current candidate started being tried, so a
 *                      timeout is attributed to the candidate that earned it.
 *   last_heartbeat_s_  the heartbeat cadence. Separate from the tick because
 *                      ctrl runs at 100 Hz and the heartbeat is 2 Hz (13 TX-5).
 *   retry_at_s_        when the backoff expires. A deadline rather than a
 *                      countdown: a countdown decremented per tick drifts with
 *                      the tick rate and stops being the number in the config.
 *   backoff_attempt_   which rung. Reset on success, so an hour of healthy link
 *                      followed by one drop waits 0.5 s, not 5 s.
 *   send_failures_     consecutive write failures, NOT missing acks.
 *   recovery_at_s_     when the next 13 CA-9 probe of the PREFERRED candidates
 *                      is due. Negative = not armed, for any of three reasons
 *                      (already preferred / budget spent / feature off).
 *   last_axis_cmd_s_   when an axis command last left. Separate from every
 *                      other send because it is the only one that arms the
 *                      0xE006 affinity window, and because it is what tells
 *                      "the robot is standing still" from "the robot is being
 *                      driven" -- a 20 Hz stream never leaves a 2 s hole.
 *
 * All comparisons are on the monotonic clock passed in (CLK-C1). Nothing here
 * reads a clock: a state machine that reads its own clock cannot be tested for
 * a five-second timeout in less than five seconds, and a test that sleeps is
 * flaky on a loaded machine.
 *
 * 13 S10 F-01..F-20: the degradation matrix rows that land in this file are
 * tagged inline (F-06 uplink aging, F-07 downlink failure run, F-08 second
 * client). Spelled "13 S10 F-xx" on purpose: bare F-13/F-21 collide with 11
 * S14's freeze-item family, and 13 v1.35 rules that an unqualified F-number
 * in code refers to THAT family.
 */

#include "quadruped/chs_a_session.h"

#include <stdexcept>
#include <utility>

namespace quadruped {
namespace chs_a {
namespace {

// ---- 13 CA-9 constants -------------------------------------------------
//
// These three are protocol and deployment facts written into 13 CA-9, not
// tunables: there is no operator question they answer, and putting them in
// quadruped.yaml would add keys the deployed resolved snapshot does not have,
// which by CLAUDE.md 3.6 is a refusal to start. They are named here, cited
// there, and asserted in the tests -- the same treatment 13 S7.5's "three
// 0xE00B in a row" already gets a few lines below.

// How long to run on a lower-priority candidate before re-testing the
// preferred ones. Derived in 13 CA-9: one failed attempt costs at most
// (enabled candidates) x probe_timeout of link outage -- 2 x 2 s on the
// shipped config -- plus TWO robot_server voice prompts (CA-6 measured that
// it announces every heartbeat connect and disconnect). 120 s keeps the
// outage duty cycle near 3% and, with kRecoveryAttempts, spreads the retries
// over six minutes, which is longer than any transient this is meant to
// recover from (a TCP reset, a switch port bounce, a DHCP renew).
constexpr double kRecoveryPeriodS = 120.0;

// How many times, before giving up until the link genuinely drops again. NOT
// unlimited, and CA-6 is the whole reason: an unlimited retry against a
// permanently dead tcp:30003 turns a patrol robot into something that
// announces itself twice every two minutes, forever.
constexpr int kRecoveryAttempts = 3;

// 13 S7.5, 0xE006 row: the chassis requires axis commands inside a two-second
// window to come from the same client. Switching sockets with less quiet than
// that makes the new socket's first axis command come back refused -- and
// "accepted but the robot does not move" is the symptom CA-1 exists to
// prevent. It doubles as the "is the robot being driven" test: a 20 Hz axis
// stream never leaves a two-second hole.
constexpr double kAxisQuietBeforeSwitchS = 2.0;

}  // namespace

const char* ConnStateName(ConnState s) {
  switch (s) {
    case ConnState::kProbing: return "probing";
    case ConnState::kOk: return "ok";
    case ConnState::kDegraded: return "degraded";
    case ConnState::kLost: return "lost";
  }
  // Unreachable for a value of the enum. Returning a marker rather than
  // asserting keeps a future enum value from crashing the receive path, and
  // the marker is obviously wrong in a log rather than plausibly right.
  return "invalid";
}

ErrorDisposition ClassifyErrorCode(std::uint32_t code) {
  ErrorDisposition d;
  switch (code) {
    case 0x0000:
      d.success = true;
      return d;
    // 0xE001..0xE005 -- format / parse / type / field / type-mismatch. All of
    // them mean the chassis could not read what we sent, so retrying sends the
    // same broken bytes again. 13 S7.5 additionally counts them toward
    // cmd_fail_threshold: a peer that can parse nothing from us is an unusable
    // endpoint, however healthy the socket looks.
    case 0xE001:
    case 0xE002:
    case 0xE003:
    case 0xE004:
    case 0xE005:
      d.our_encoding_bug = true;
      d.counts_toward_cmd_fail = true;
      return d;
    // A second client talked to the chassis within the 2 s affinity window
    // (13 CA-1 / CA-3, matrix row 13 S10 F-08). Not retried: the window has
    // to pass, and re-sending inside it re-arms it.
    case 0xE006:
      d.second_client = true;
      return d;
    // No permission / not allowed. 0xE008 is also what a sleeping chassis
    // answers five seconds after a motion command (13 CA-8), which is why
    // F-21 refuses to send rather than waiting for this.
    case 0xE007:
    case 0xE008:
      d.mode_switch_failed = true;
      return d;
    case 0xE009:
      d.retry_once = true;
      return d;
    // "Unsupported function". 13 S7.5 warns explicitly against reading this as
    // "navigation is unavailable": the manual's example is about the chassis's
    // own patrol stack, which we do not use.
    case 0xE00A:
      d.capability = true;
      return d;
    case 0xE00B:
      d.chassis_internal = true;
      return d;
    default:
      // An unregistered code. Reported as an encoding-side fault rather than
      // ignored, because the alternative is a command that silently did
      // nothing -- but NOT counted toward cmd_fail_threshold, since an unknown
      // code is not evidence that the endpoint is unusable.
      d.our_encoding_bug = true;
      return d;
  }
}

SessionConfig SessionConfig::FromLinkConfig(const ChassisLinkConfig& link) {
  SessionConfig c;
  c.endpoints = link.endpoints;
  c.probe_timeout_s = static_cast<double>(link.probe_timeout_ms) / 1000.0;
  if (!(link.heartbeat_hz > 0.0)) {
    // 13 S2.2: the chassis reports state ONLY to whoever keeps sending
    // heartbeats, so a zero rate is not "no heartbeats", it is "no reports" --
    // which presents as a chassis that never answers.
    throw std::runtime_error(
        "chassis_link.heartbeat_hz must be > 0: the chassis reports state only "
        "to the address that keeps sending heartbeats (13 S2.2)");
  }
  c.heartbeat_period_s = 1.0 / link.heartbeat_hz;
  c.state_timeout_degraded_s = link.state_timeout_degraded_s;
  c.state_timeout_lost_s = link.state_timeout_lost_s;
  c.cmd_fail_threshold = link.cmd_fail_threshold;
  c.reconnect_backoff_s = link.reconnect_backoff_s;
  if (c.reconnect_backoff_s.empty()) {
    throw std::runtime_error(
        "chassis_link.reconnect_backoff_s is empty: there would be no delay to "
        "apply after a drop, and the reconnect becomes a busy loop (13 S8.2)");
  }
  if (!(c.probe_timeout_s > 0.0)) {
    throw std::runtime_error(
        "chassis_link.probe_timeout_ms must be > 0: a zero probe window rejects "
        "every candidate on the tick it is dialled");
  }
  // 13 CA-9. Set from the named constants above rather than from the file:
  // see their comment on why they are not config keys.
  c.endpoint_recovery_period_s = kRecoveryPeriodS;
  c.endpoint_recovery_attempts = kRecoveryAttempts;
  c.axis_quiet_before_switch_s = kAxisQuietBeforeSwitchS;
  return c;
}

Session::Session(SessionConfig cfg, Dial dial, Hangup hangup,
                 CredentialsReady creds)
    : cfg_(std::move(cfg)),
      dial_(std::move(dial)),
      hangup_(std::move(hangup)),
      creds_(std::move(creds)) {
  recovery_budget_ = cfg_.endpoint_recovery_attempts;
}

double Session::BackoffFor(std::size_t attempt) const {
  // The last rung repeats forever. A ladder that ran off its end would either
  // need a default here -- a number not in the config, which CLAUDE.md 3.1
  // forbids for exactly this reason -- or stop reconnecting altogether.
  const std::size_t n = cfg_.reconnect_backoff_s.size();
  return cfg_.reconnect_backoff_s[attempt < n ? attempt : n - 1];
}

void Session::EnterLost(double now_mono_s, TickResult* out) {
  if (socket_open_) {
    hangup_();
    socket_open_ = false;
    out->disconnected = true;
  }
  state_ = ConnState::kLost;
  active_ = -1;
  probe_started_s_ = -1.0;
  last_heartbeat_s_ = -1.0;
  retry_at_s_ = now_mono_s + BackoffFor(backoff_attempt_);
  ++backoff_attempt_;
  candidate_ = 0;
  // 13 CA-9: a genuine drop refills the recovery budget. The budget exists to
  // stop us re-testing an endpoint we have already found dead THIS TIME; a
  // link that went away and came back is a new world, and whatever made
  // tcp:30003 unreachable may have gone with it. Not refilling would mean a
  // robot that exhausted its three attempts in the morning stays on UDP for
  // the rest of the day even after the network is fixed and the link bounces.
  recovery_budget_ = cfg_.endpoint_recovery_attempts;
  recovering_ = false;
  recovery_from_ = -1;
  recovery_at_s_ = -1.0;
}

bool Session::HasHigherPriorityCandidate() {
  // active_ <= 0 covers both "already on the preferred candidate" and "no live
  // link", and in neither case is there anything to recover to.
  if (active_ <= 0) return false;
  for (std::size_t i = 0; i < static_cast<std::size_t>(active_); ++i) {
    const EndpointCandidate& ep = cfg_.endpoints[i];
    if (!ep.enabled) continue;
    // 13 TLS-4 is applied HERE as well as in the walk, and that is not
    // duplication: a TLS candidate with no certificate is skipped at no cost
    // during a walk, but scheduling a walk FOR it is not free -- the walk
    // itself drops the live link. Without this the session would break the
    // link every period for a candidate it already knows it cannot dial.
    if (ep.tls && creds_ && !creds_(ep)) continue;
    return true;
  }
  return false;
}

void Session::ScheduleRecovery(double now_mono_s) {
  recovery_at_s_ = -1.0;
  if (!(cfg_.endpoint_recovery_period_s > 0.0)) return;
  if (recovery_budget_ <= 0) return;
  if (!HasHigherPriorityCandidate()) return;
  recovery_at_s_ = now_mono_s + cfg_.endpoint_recovery_period_s;
}

bool Session::RecoveryDue(double now_mono_s) const {
  if (recovery_at_s_ < 0.0) return false;
  if (now_mono_s < recovery_at_s_) return false;
  // The axis stream must have been quiet for the 0xE006 affinity window. This
  // is the guard that keeps the probe off a robot that is moving: at 20 Hz the
  // stream never leaves a two-second hole, so a driving robot simply never
  // satisfies it and the deadline slides until it stops.
  //
  // Negative means no axis command has ever gone out, which passes: there is
  // no affinity window to wait out and nothing is in flight.
  if (last_axis_cmd_s_ >= 0.0 &&
      now_mono_s - last_axis_cmd_s_ < cfg_.axis_quiet_before_switch_s) {
    return false;
  }
  return true;
}

void Session::StartRecovery(double now_mono_s, TickResult* out) {
  recovering_ = true;
  recovery_from_ = active_;
  recovery_at_s_ = -1.0;
  ++recovery_attempts_;
  // From the TOP of the list, through the ordinary walk. Three things come for
  // free that a bespoke "dial candidate 0" would each have to redo: the
  // enabled / credentials skips, the probe window, and -- the one that matters
  // -- the FALLBACK. If the preferred candidates stay silent the walk simply
  // continues and arrives back at the one we were using, so "the probe failed"
  // and "we are back where we started" are the same code path rather than an
  // error branch nothing exercises.
  candidate_ = 0;
  AdvanceCandidate(now_mono_s, out);
}

void Session::AdvanceCandidate(double now_mono_s, TickResult* out) {
  // Close whatever the previous candidate left open before trying the next:
  // 13 CA-1 turns a second live socket into a second CLIENT, and the chassis
  // then refuses axis commands for two seconds with 0xE006.
  if (socket_open_) {
    hangup_();
    socket_open_ = false;
    out->disconnected = true;
  }
  while (candidate_ < cfg_.endpoints.size()) {
    const EndpointCandidate& ep = cfg_.endpoints[candidate_];
    if (!ep.enabled) {
      last_skip_ = SkipReason::kDisabled;
      ++candidate_;
      continue;
    }
    // 13 TLS-4. Judged before dialling and costing no timeout, so "there is no
    // certificate" cannot be mistaken for "the chassis did not answer".
    if (ep.tls && creds_ && !creds_(ep)) {
      last_skip_ = SkipReason::kNoCredentials;
      ++candidate_;
      continue;
    }
    if (!dial_(ep)) {
      last_skip_ = SkipReason::kConnectFailed;
      ++candidate_;
      continue;
    }
    socket_open_ = true;
    state_ = ConnState::kProbing;
    probe_started_s_ = now_mono_s;
    // Force a heartbeat on this very tick. The chassis reports only to an
    // address that is sending them, so waiting a heartbeat period first would
    // spend that period of the probe window guaranteed to hear nothing.
    last_heartbeat_s_ = -1.0;
    return;
  }
  // Off the end of the list: every candidate was disabled, uncredentialed,
  // unreachable, or silent.
  ++probe_cycles_;
  EnterLost(now_mono_s, out);
}

TickResult Session::Tick(double now_mono_s) {
  TickResult out;

  if (state_ == ConnState::kLost) {
    if (retry_at_s_ >= 0.0 && now_mono_s < retry_at_s_) return out;
    retry_at_s_ = -1.0;
    candidate_ = 0;
    AdvanceCandidate(now_mono_s, &out);
  } else if (!socket_open_) {
    // First entry: nothing dialled yet.
    AdvanceCandidate(now_mono_s, &out);
  }

  if (!socket_open_) return out;

  if (state_ == ConnState::kProbing) {
    if (last_report_s_ >= probe_started_s_ && last_report_s_ >= 0.0) {
      // The candidate answered. This is the only transition into a live link.
      state_ = ConnState::kOk;
      active_ = static_cast<int>(candidate_);
      backoff_attempt_ = 0;
      send_failures_ = 0;
      internal_errors_ = 0;
      ++link_epoch_;
      ++reconnects_;
      out.connected = true;
      // 13 CA-9: settle the recovery attempt this connection belongs to.
      // "Success" is landing on a HIGHER-priority candidate -- anything else
      // (including landing back on the one we left, which is the normal
      // outcome when the preferred endpoint is still dead) spends one of the
      // three attempts. Charging the budget only on failure is what makes the
      // feature converge: a successful recovery costs nothing, so a link that
      // flaps between candidates never runs out of attempts.
      if (recovering_) {
        recovering_ = false;
        if (active_ >= recovery_from_) --recovery_budget_;
      }
      ScheduleRecovery(now_mono_s);
    } else if (now_mono_s - probe_started_s_ > cfg_.probe_timeout_s) {
      last_skip_ = SkipReason::kNoReportInTime;
      ++candidate_;
      AdvanceCandidate(now_mono_s, &out);
      if (!socket_open_) return out;
    }
  } else {
    // Live link: age the uplink. 13 S2.5, matrix row 13 S10 F-06 -- late
    // reports degrade, absent ones are lost. The order matters: lost is
    // checked first so a long gap does not spend a tick in degraded on its
    // way past the lost threshold.
    const double age = now_mono_s - last_report_s_;
    if (last_report_s_ < 0.0 || age > cfg_.state_timeout_lost_s) {
      EnterLost(now_mono_s, &out);
      return out;
    }
    if (age > cfg_.state_timeout_degraded_s) {
      state_ = ConnState::kDegraded;
    } else if (send_failures_ < cfg_.cmd_fail_threshold &&
               internal_errors_ < 3) {
      // Reports are current AND the downlink is healthy (13 S10 F-07: a
      // failure run on the downlink degrades). Both halves are required: a
      // link whose writes keep failing is degraded even while the chassis is
      // still talking, because the robot is not being driven.
      state_ = ConnState::kOk;
    } else {
      state_ = ConnState::kDegraded;
    }
    // 13 CA-9, and ONLY from kOk. A degraded link is already being worked on
    // by the ladder above (late reports, or a downlink failure run), and
    // dropping it for a probe on top of that makes the next failure
    // unattributable: the operator cannot tell whether the endpoint switched
    // because the preferred one came back or because this one was dying.
    // Stable-and-healthy is the one state where the only thing wrong is which
    // candidate we are on.
    if (state_ == ConnState::kOk && RecoveryDue(now_mono_s)) {
      StartRecovery(now_mono_s, &out);
      // The walk can run off the end of the list (every candidate silent) and
      // land in kLost with no socket. Returning here rather than falling into
      // the heartbeat block below, which would ask for a heartbeat on a socket
      // that is not there.
      if (!socket_open_) return out;
    }
  }

  // Heartbeat, on the same socket as the axis commands (13 CA-2) and on the
  // ctrl thread (13 TX-5). Sent while probing too: that is how the chassis
  // learns which address to report to in the first place.
  if (last_heartbeat_s_ < 0.0 ||
      now_mono_s - last_heartbeat_s_ >= cfg_.heartbeat_period_s) {
    last_heartbeat_s_ = now_mono_s;
    out.send_heartbeat = true;
  }
  return out;
}

void Session::OnReport(double now_mono_s) { last_report_s_ = now_mono_s; }

void Session::OnSendFailure(double now_mono_s) {
  (void)now_mono_s;  // the count is what matters; the state ages on the tick
  ++send_failures_;
}

void Session::OnSendSuccess() {
  // 13 S2.5 counts CONSECUTIVE failures, so one good write clears the run. A
  // cumulative counter would eventually degrade a link that has been healthy
  // for hours with a handful of transient failures spread across them.
  send_failures_ = 0;
}

void Session::OnAxisCommandSent(double now_mono_s) {
  // Monotone on purpose. The caller forwards the estop path's send time from a
  // later control period (that frame goes out on the zenoh callback thread),
  // so the two sources arrive interleaved; taking the max means a stale
  // forward can only ever be ignored, never shorten the quiet window.
  if (now_mono_s > last_axis_cmd_s_) last_axis_cmd_s_ = now_mono_s;
}

void Session::OnErrorCode(double now_mono_s, std::uint32_t code) {
  (void)now_mono_s;
  const ErrorDisposition d = ClassifyErrorCode(code);
  // *** KEPT, not just classified. The code was consumed here and discarded,
  // so a chassis that refused a command left no trace anywhere: measured on
  // the live machine 2026-09-21, a usage_mode switch to navigation was
  // dispatched, the chassis did not change mode, and nothing in the process
  // could say whether a frame had gone out, whether the chassis had answered,
  // or what it had answered.
  //
  // 13 S7.5 gives each code a disposition; the code ITSELF is what names the
  // problem -- E_BUSY for a refused navigation mode (11 S9.10.1, charge_manager
  // still running) reads nothing like a framing error.
  if (!d.success) {
    last_error_code_ = code;
    ++error_codes_seen_;
  }
  if (d.success) {
    send_failures_ = 0;
    internal_errors_ = 0;
    return;
  }
  if (d.counts_toward_cmd_fail) ++send_failures_;
  if (d.chassis_internal) {
    ++internal_errors_;
  } else {
    // 13 S7.5 says THREE IN A ROW of 0xE00B degrade the link, so any other
    // outcome ends the run. Without this the counter accumulates across an
    // entire session and degrades a link on three unrelated internal errors
    // hours apart.
    internal_errors_ = 0;
  }
}

void Session::OnSleep(bool sleeping) { asleep_ = sleeping; }

bool Session::motion_allowed() const {
  // 13 F-21: asleep means every motion command comes back 0xE008 five seconds
  // later and there is no wake command in the protocol. Sending anyway costs
  // five seconds per command during which the process believes it is driving.
  if (asleep_) return false;
  // 13 S2.5: no link, or a downlink that keeps failing, means STOP SENDING --
  // not "send zero". 13 S3.4 is explicit that a zero-velocity command is
  // itself a command, and a link we cannot trust is not a link to send it on.
  if (state_ != ConnState::kOk) return false;
  return true;
}

}  // namespace chs_a
}  // namespace quadruped
