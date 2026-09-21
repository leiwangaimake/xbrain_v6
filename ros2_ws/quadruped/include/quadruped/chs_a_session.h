/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_session.h
 * Brief: Channel-one session -- endpoint probe, conn state, backoff, 13 S7.5
 *
 * Description:
 * What this owns: the question "is the chassis link usable right now, and if
 * not, what should happen next". Everything in it comes from 13 S2.2 (probe),
 * 13 S2.5 (failure behaviour), 13 S7.5 (the response codes) and 13 F-21
 * (sleep). It owns no socket and no bytes.
 *
 * The socket is injected as three callables rather than inherited from an
 * interface, for the same reason tx_owner takes a FrameWriter: the policy is
 * what is worth testing, it is where every one of these rules can go wrong, and
 * a policy that can only be exercised against a real chassis is a policy that
 * gets tested once, by hand, on the day it is written.
 *
 * Four rules that are easy to state and easy to implement backwards:
 *
 *   * a candidate whose CREDENTIALS are missing is skipped IMMEDIATELY and does
 *     not consume probe_timeout_ms (13 TLS-4). This is the one that makes a
 *     failed startup diagnosable: with three candidates at two seconds each,
 *     the difference between "no certificate installed" and "the cable is out"
 *     is otherwise six seconds of identical silence, and 13 S2.2 spends a
 *     paragraph on exactly that confusion.
 *   * reconnecting is NOT resuming. 13 CON-05 / BIT-33: the link coming back
 *     must never by itself restore motion, because the robot may have been
 *     moved, switched to another mode with the factory handset, or put to sleep
 *     while we were away. The session exposes a reconnect counter so the layer
 *     above can require a fresh handshake, and deliberately has no "resume" of
 *     its own.
 *   * a send failure is counted, an ABSENT ACK is not. The chassis never
 *     acknowledges an axis command (13 CA-7, measured) -- it neither replies
 *     nor logs. Counting missing acks as failures would report a perfectly
 *     healthy link as broken within one second of the first command.
 *   * while the chassis reports Sleep, NO motion command goes out (13 F-21).
 *     Not "send and handle the error": the chassis takes five seconds to answer
 *     0xE008, there is no wake command anywhere in the protocol, and each
 *     attempt is five seconds during which the process believes it is driving.
 *
 * Boundary: no bytes, no socket, no events. Encoding is chs_a_codec, framing is
 * chs_a_framer, the single write path is tx_owner, and turning a state change
 * into an RT-plane event is B4's. This file decides; it does not act.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_SESSION_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_SESSION_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "quadruped/quadruped_config.h"

namespace quadruped {
namespace chs_a {

// 11 S9.1.3 conn values, plus the startup state. Four and not three: "never
// connected yet" and "was connected and lost it" call for different messages,
// and collapsing them produces a startup that reports a link failure before it
// has finished trying.
enum class ConnState {
  kProbing,   // walking the candidate list
  kOk,        // reports arriving inside state_timeout_degraded_s
  kDegraded,  // reports late, or the downlink is failing; link still open
  kLost,      // no reports for state_timeout_lost_s; waiting out the backoff
};

const char* ConnStateName(ConnState s);

// Why a probe candidate was passed over. Kept because "skipped" and "timed out"
// are the two outcomes 13 S2.2 says must never look alike in a log.
enum class SkipReason {
  kNone,
  kDisabled,          // enabled: false in the config
  kNoCredentials,     // 13 TLS-4: judged before dialling, costs no timeout
  kConnectFailed,     // the dial itself failed, also immediate
  kNoReportInTime,    // connected, sent heartbeats, nothing came back
};

// What 13 S7.5 says to do about one generic response code. A struct rather than
// a single enum because the dispositions are not mutually exclusive: 0xE008 is
// both "our fault report" and "the mode switch failed".
struct ErrorDisposition {
  bool success = false;
  // 0xE001..0xE005: a malformed request, so OUR bug. 13 S7.5 also makes these
  // count toward cmd_fail_threshold, because a peer that cannot parse anything
  // we send is an unusable endpoint no matter how healthy the socket is.
  bool our_encoding_bug = false;
  bool counts_toward_cmd_fail = false;
  bool retry_once = false;        // 0xE009 only
  bool mode_switch_failed = false;  // 0xE007 / 0xE008 (MS-2)
  bool capability = false;        // 0xE00A -> E_CAPABILITY
  bool second_client = false;     // 0xE006: someone else is talking to it
  bool chassis_internal = false;  // 0xE00B: three in a row -> degraded
};

// Pure function of the code, so the table can be read against 13 S7.5 directly.
ErrorDisposition ClassifyErrorCode(std::uint32_t code);

// Everything the session needs, lifted out of ChassisLinkConfig so the tests
// can build one without a whole config file. Seconds throughout: the config
// carries milliseconds and the conversion happens once, at construction, rather
// than at each comparison where one missed division is a factor of a thousand.
struct SessionConfig {
  std::vector<EndpointCandidate> endpoints;
  double probe_timeout_s = 0.0;
  double heartbeat_period_s = 0.0;
  double state_timeout_degraded_s = 0.0;
  double state_timeout_lost_s = 0.0;
  int cmd_fail_threshold = 0;
  std::vector<double> reconnect_backoff_s;

  // Build from the loaded config. Throws when a value would make the session
  // degenerate (a zero heartbeat period is a busy loop; an empty ladder has no
  // delay to apply), rather than clamping -- a clamped value runs, and runs
  // differently from what the file says.
  static SessionConfig FromLinkConfig(const ChassisLinkConfig& link);
};

// What one Tick decided. The caller performs the send; the session never holds
// the socket (13 TX-4: only chs_a_send does).
struct TickResult {
  bool send_heartbeat = false;
  // The link just came up on `active_endpoint()`. NOT a licence to move:
  // see CON-05 in the file comment.
  bool connected = false;
  // The link just went down. The caller drops any half-assembled frame
  // (chs_a_framer::Reset) so bytes from the old connection are never read as
  // the start of the new one.
  bool disconnected = false;
};

class Session {
 public:
  // Dial one candidate. Returns false on an immediate failure; a dial that
  // would block belongs behind a non-blocking connect in the caller, because a
  // blocking one inside Tick would stall the control loop.
  using Dial = std::function<bool(const EndpointCandidate&)>;
  using Hangup = std::function<void()>;
  // 13 TLS-4: judged from the filesystem BEFORE dialling, so a missing
  // certificate costs nothing and is reported as itself.
  using CredentialsReady = std::function<bool(const EndpointCandidate&)>;

  Session(SessionConfig cfg, Dial dial, Hangup hangup, CredentialsReady creds);

  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  // Drive the session. Called from ctrl at control_loop_hz; does no I/O beyond
  // the injected callables and never blocks.
  TickResult Tick(double now_mono_s);

  // A report of any kind arrived. This is the ONLY thing that proves the link
  // is alive: 13 CA-7 says axis commands are never acknowledged, so silence on
  // the downlink says nothing at all.
  void OnReport(double now_mono_s);

  // Results of a frame write (tx_owner's verdict).
  void OnSendFailure(double now_mono_s);
  void OnSendSuccess();

  // A generic response arrived with this code (13 S7.5).
  void OnErrorCode(double now_mono_s, std::uint32_t code);

  // The last NON-success response code, and how many have arrived. 13 S7.5
  // classifies each one; the CODE ITSELF is what names the problem, and it was
  // being discarded -- so a chassis that refused a command left no trace.
  std::uint32_t last_error_code() const { return last_error_code_; }
  std::uint64_t error_codes_seen() const { return error_codes_seen_; }

  // The chassis said it is asleep or awake (BasicStatus.Sleep, 13 F-21).
  void OnSleep(bool sleeping);

  ConnState state() const { return state_; }
  bool asleep() const { return asleep_; }

  // The gate 13 F-21 / S2.5 put in front of every motion command. False while
  // asleep, while the downlink is failing, or while there is no link at all.
  bool motion_allowed() const;

  // Index into the configured candidate list, or -1 while none is live.
  int active_endpoint() const { return active_; }

  // Increments on every transition INTO a live link. The layer above compares
  // it with the epoch it last handshook on; a change means "re-handshake",
  // never "carry on" (CON-05).
  std::uint64_t link_epoch() const { return link_epoch_; }

  // Diagnostics. Counters rather than log lines: one probe failure at startup
  // is ordinary, one a minute is a link that keeps dying, and only a rate tells
  // them apart.
  SkipReason last_skip_reason() const { return last_skip_; }
  std::uint64_t probe_cycles() const { return probe_cycles_; }
  std::uint64_t reconnects() const { return reconnects_; }
  int consecutive_send_failures() const { return send_failures_; }

 private:
  // Move to the next candidate, skipping the ones that can be judged without
  // dialling. Sets state_ to kOk-pending (probing with a live socket) or walks
  // off the end of the list and starts the backoff.
  void AdvanceCandidate(double now_mono_s, TickResult* out);
  void EnterLost(double now_mono_s, TickResult* out);
  double BackoffFor(std::size_t attempt) const;

  SessionConfig cfg_;
  Dial dial_;
  Hangup hangup_;
  CredentialsReady creds_;

  ConnState state_ = ConnState::kProbing;
  int active_ = -1;
  // Candidate under consideration while probing. Separate from active_: a
  // candidate being dialled is not yet the live one, and merging the two
  // reports a link as up while it is still being tried.
  std::size_t candidate_ = 0;
  bool socket_open_ = false;

  // Negative means "not started", which is distinct from 0.0 -- a legitimate
  // monotonic reading immediately after boot.
  double probe_started_s_ = -1.0;
  double last_report_s_ = -1.0;
  double last_heartbeat_s_ = -1.0;
  double retry_at_s_ = -1.0;

  std::size_t backoff_attempt_ = 0;
  int send_failures_ = 0;
  int internal_errors_ = 0;
  std::uint32_t last_error_code_ = 0;
  std::uint64_t error_codes_seen_ = 0;
  bool asleep_ = false;

  SkipReason last_skip_ = SkipReason::kNone;
  std::uint64_t probe_cycles_ = 0;
  std::uint64_t reconnects_ = 0;
  std::uint64_t link_epoch_ = 0;
};

}  // namespace chs_a
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_SESSION_H_
