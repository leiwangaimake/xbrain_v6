/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_session.h
 * Brief: One Zenoh plane session: open, declare with explicit QoS, put,
 *        callback-subscribe -- plus the config document builder
 *
 * Description:
 * The relay holds TWO of these (general at :7447, RT at :7449), which RT-C3
 * permits exactly because each is its own independent runtime with scouting
 * locked down (RelaySessionConfigJson below; the a-e sub-conditions are
 * argued field by field in relay_session_config.cc). The class is a close
 * copy of quadruped's rt_session.cc -- deliberately self-held rather than
 * shared, because the two are separate processes and a shared session class
 * would be a shared failure domain; the zenoh-c traps its header documents
 * (owned keyexpr, z_move discipline, loaned samples) apply here verbatim.
 *
 * What is DIFFERENT from the quadruped copy, and why:
 *   * DeclarePublisher takes a QoS profile name and maps it onto
 *     z_publisher_options_t (congestion_control / priority / is_express).
 *     quadruped declares with binding defaults; for the relay that would put
 *     "block" congestion on the estop forward, which anti-pattern A-5
 *     forbids outright. The mapping is the frozen table of
 *     common/include/xbrain/zenoh/qos_profiles.h -- an unknown profile name
 *     refuses to declare (publishing with QoS nobody chose is A-7).
 *   * The subscriber trampoline copies the payload with a bounded reader
 *     (z_bytes_get_reader into a stack buffer), never z_bytes_to_slice:
 *     the slice form allocates per sample, and these callbacks ARE the
 *     emergency-stop path (CRL-4). A sample larger than the buffer is
 *     dropped and counted, the boundary relay_core.h argues.
 *
 * Reliability note: the frozen QoS table carries a reliability column, but
 * this zenoh-c's publisher options do not (1.x moved reliability semantics
 * into the transport; the option set here is congestion + priority +
 * express). The three fields that exist are set explicitly; the absent one
 * is recorded here so nobody hunts for a missing line.
 */

#ifndef HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_SESSION_H_
#define HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_SESSION_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace chassis_relay {

// The session config document, as text for zc_config_from_str. Same field
// set for both planes (only the endpoint differs): peer mode, one connect
// endpoint, empty listen (RT-C3.d), multicast scouting off (RT-C1), gossip
// on with multihop off (RT-C2 as corrected 2026-08-23). Lives in its own
// translation unit with no zenoh dependency so it is testable -- and
// mutable by the relay mutant suite -- on a machine with neither zenoh nor
// a router; the wrong values fail as SILENCE (a session that connects and
// receives nothing), which is why this is the one function that must be
// checkable everywhere (same reasoning as quadruped's rt_session_config.cc).
std::string RelaySessionConfigJson(const std::string& endpoint);

// One plane's session. Publishers are addressed by the integer handle
// DeclarePublisher returns; subscribers hold their callback for the life of
// the session (strong-reference discipline: dropping a subscriber silently
// unsubscribes, the zenoh trap CLAUDE.md 4.3 documents for Python and which
// holds for zenoh-c owned types just the same).
class RelaySession {
 public:
  // Bytes of one received sample, already copied out of the loan into a
  // bounded buffer. Runs on a zenoh runtime thread: everything called from
  // here must be allocation-free and non-blocking (CRL-4/CRL-6).
  using SampleFn = std::function<void(const char* bytes, std::size_t len)>;

  RelaySession();
  ~RelaySession();
  RelaySession(const RelaySession&) = delete;
  RelaySession& operator=(const RelaySession&) = delete;

  // Open against one router endpoint ("tcp/127.0.0.1:7449"). No retry: "the
  // router is down" and "the router is up and silent" are different faults,
  // and a retry loop turns the first into the second (quadruped's argument,
  // kept). False + *err on failure.
  bool Open(const std::string& endpoint, std::string* err);

  // Drop subscribers first, then publishers, then the session. Idempotent.
  void Close();

  bool is_open() const;

  // Declare a publisher on `key` with the named frozen QoS profile
  // ("Q0_safety" / "Q2_state" / "Q3_cmd"). Returns a handle >= 0, or -1 with
  // *err (unknown profile included -- see the header on A-7).
  int DeclarePublisher(const std::string& key, const char* qos_profile,
                       std::string* err);

  // Publish len bytes on a declared handle. Allocation in OUR code: none;
  // zenoh's own runtime copies the payload internally, the same acceptance
  // quadruped's rt made. False on refusal (counted by the caller).
  bool Put(int handle, const char* bytes, std::size_t len);

  // Subscribe `key`, invoking on_sample per received sample. The callback
  // context is owned by the session (never by the vector's storage -- the
  // reallocation trap rt_session.cc names).
  bool DeclareSubscriber(const std::string& key, SampleFn on_sample,
                         std::string* err);

  // Counters, for the stats line: samples delivered to callbacks, samples
  // dropped because they exceeded the bounded reader's buffer, puts sent,
  // puts refused.
  std::uint64_t samples_received() const;
  std::uint64_t samples_oversize() const;
  std::uint64_t puts_sent() const;
  std::uint64_t put_failures() const;

  std::size_t publisher_count() const;
  std::size_t subscriber_count() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chassis_relay

#endif  // HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_SESSION_H_
