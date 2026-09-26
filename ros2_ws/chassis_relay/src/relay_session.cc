/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_session.cc
 * Brief: The zenoh-c session wrapper (see relay_session.h for the contract)
 *
 * Description:
 * Structure copied from quadruped's rt_session.cc, whose header names the
 * three zenoh-c details that cost time when rediscovered; they are kept
 * short here because that file is the reference:
 *   * z_keyexpr_from_str (owned), never the view form -- the view canonises
 *     IN PLACE and may segfault on read-only strings;
 *   * every z_owned_* handed to zenoh is MOVED, or the double drop lands at
 *     shutdown, far from the cause;
 *   * the sample callback receives a LOAN, valid only for the call.
 *
 * The two local differences, argued in the header: explicit publisher QoS
 * from the frozen table (A-5/A-7), and a bounded allocation-free payload
 * copy in the trampoline (CRL-4). The trampoline's stack buffer is
 * kMaxForwardBytes + 1: one byte more than the largest frame relay_core
 * accepts, so "fits the buffer but exceeds the core's cap" and "exceeds the
 * buffer" stay two distinguishable counters rather than one ambiguous drop.
 */

#include "chassis_relay/relay_session.h"

#include <atomic>
#include <cstring>
#include <mutex>
#include <vector>

#include "chassis_relay/relay_core.h"
#include "xbrain/zenoh/qos_profiles.h"
#include "zenoh.h"

namespace chassis_relay {

namespace qos = hachist::xbrain::qos;

struct RelaySession::Impl {
  z_owned_session_t session;
  bool open = false;
  std::vector<z_owned_publisher_t> publishers;
  std::vector<z_owned_subscriber_t> subscribers;
  // One per subscriber, owned here so the callback context outlives the
  // declaration; never inside the vector's own storage (reallocation would
  // leave the context dangling -- rt_session.cc's trap, kept verbatim).
  std::vector<std::unique_ptr<SampleFn>> callbacks;
  std::mutex mu;  // declarations only -- never held on a forward path

  std::atomic<std::uint64_t> samples{0};
  std::atomic<std::uint64_t> oversize{0};
  std::atomic<std::uint64_t> puts{0};
  std::atomic<std::uint64_t> put_fail{0};
};

namespace {

struct CallbackCtx {
  RelaySession::SampleFn* fn;
  std::atomic<std::uint64_t>* samples;
  std::atomic<std::uint64_t>* oversize;
};

// Copy the loaned payload into a stack buffer and hand it on. No allocation:
// z_bytes_get_reader reads into caller memory, unlike z_bytes_to_slice which
// allocates a fresh slice per sample -- this callback is the estop path.
void OnSampleTrampoline(z_loaned_sample_t* sample, void* context) {
  auto* ctx = static_cast<CallbackCtx*>(context);
  if (ctx == nullptr || ctx->fn == nullptr) return;

  const z_loaned_bytes_t* payload = z_sample_payload(sample);
  const std::size_t len = z_bytes_len(payload);
  // The +1 headroom keeps the session's "does not fit" distinct from the
  // core's "over the forward cap" (see the file header).
  char buf[kMaxForwardBytes + 1];
  if (len > sizeof(buf)) {
    ctx->oversize->fetch_add(1, std::memory_order_relaxed);
    return;
  }
  z_bytes_reader_t reader = z_bytes_get_reader(payload);
  const std::size_t got =
      z_bytes_reader_read(&reader, reinterpret_cast<uint8_t*>(buf), len);
  if (got != len) {
    // A short read of a loan should be impossible; counted rather than
    // trusted, because a truncated frame forwarded as complete is the
    // silent-corruption direction.
    ctx->oversize->fetch_add(1, std::memory_order_relaxed);
    return;
  }
  ctx->samples->fetch_add(1, std::memory_order_relaxed);
  (*ctx->fn)(buf, len);
}

void OnDropTrampoline(void* context) {
  delete static_cast<CallbackCtx*>(context);
}

// The frozen profile, mapped onto this binding's option set. Returns false
// on a name the table does not carry -- declaring with binding defaults is
// anti-pattern A-7, so an unknown profile refuses rather than guesses.
bool ApplyProfile(const char* profile_name, z_publisher_options_t* opts) {
  const qos::QosProfile* p = qos::FindProfile(profile_name);
  if (p == nullptr) return false;
  opts->congestion_control =
      (std::strcmp(p->congestion_control, "block") == 0)
          ? Z_CONGESTION_CONTROL_BLOCK
          : Z_CONGESTION_CONTROL_DROP;
  // The frozen table names four priorities today; anything else refuses for
  // the same A-7 reason as an unknown profile name.
  if (std::strcmp(p->priority, "real_time") == 0) {
    opts->priority = Z_PRIORITY_REAL_TIME;
  } else if (std::strcmp(p->priority, "data_high") == 0) {
    opts->priority = Z_PRIORITY_DATA_HIGH;
  } else if (std::strcmp(p->priority, "data") == 0) {
    opts->priority = Z_PRIORITY_DATA;
  } else if (std::strcmp(p->priority, "interactive_high") == 0) {
    opts->priority = Z_PRIORITY_INTERACTIVE_HIGH;
  } else {
    return false;
  }
  opts->is_express = p->express;
  return true;
}

}  // namespace

RelaySession::RelaySession() : impl_(new Impl()) {}

RelaySession::~RelaySession() { Close(); }

bool RelaySession::Open(const std::string& endpoint, std::string* err) {
  if (impl_->open) {
    if (err) *err = "relay session already open";
    return false;
  }
  const std::string cfg_json = RelaySessionConfigJson(endpoint);
  z_owned_config_t cfg;
  if (zc_config_from_str(&cfg, cfg_json.c_str()) != Z_OK) {
    if (err) *err = "relay session: config rejected by zenoh: " + cfg_json;
    return false;
  }
  if (z_open(&impl_->session, z_move(cfg), nullptr) != Z_OK) {
    // No retry -- see the header's contract.
    if (err) *err = "relay session: z_open failed against " + endpoint;
    return false;
  }
  impl_->open = true;
  return true;
}

void RelaySession::Close() {
  if (!impl_ || !impl_->open) return;
  // Subscribers first: a subscriber whose session is already gone still has
  // a callback the runtime may be inside.
  for (auto& s : impl_->subscribers) z_drop(z_move(s));
  impl_->subscribers.clear();
  for (auto& p : impl_->publishers) z_drop(z_move(p));
  impl_->publishers.clear();
  impl_->callbacks.clear();
  z_drop(z_move(impl_->session));
  impl_->open = false;
}

bool RelaySession::is_open() const { return impl_ && impl_->open; }

int RelaySession::DeclarePublisher(const std::string& key,
                                   const char* qos_profile, std::string* err) {
  if (!is_open()) {
    if (err) *err = "relay session not open";
    return -1;
  }
  z_publisher_options_t opts;
  z_publisher_options_default(&opts);
  if (!ApplyProfile(qos_profile, &opts)) {
    // A-7: no publisher goes up carrying QoS nobody chose for it.
    if (err) {
      *err = "relay session: unknown qos profile '" +
             std::string(qos_profile ? qos_profile : "(null)") + "' for " + key;
    }
    return -1;
  }
  std::lock_guard<std::mutex> lock(impl_->mu);
  z_owned_keyexpr_t ke;
  if (z_keyexpr_from_str(&ke, key.c_str()) != Z_OK) {
    if (err) *err = "relay session: not a valid key expression: " + key;
    return -1;
  }
  z_owned_publisher_t pub;
  const z_result_t rc =
      z_declare_publisher(z_loan(impl_->session), &pub, z_loan(ke), &opts);
  z_drop(z_move(ke));
  if (rc != Z_OK) {
    if (err) *err = "relay session: declare_publisher failed on " + key;
    return -1;
  }
  impl_->publishers.push_back(pub);
  return static_cast<int>(impl_->publishers.size()) - 1;
}

bool RelaySession::Put(int handle, const char* bytes, std::size_t len) {
  if (!is_open() || bytes == nullptr) return false;
  if (handle < 0 ||
      static_cast<std::size_t>(handle) >= impl_->publishers.size()) {
    return false;
  }
  z_owned_bytes_t payload;
  if (z_bytes_copy_from_buf(&payload, reinterpret_cast<const uint8_t*>(bytes),
                            len) != Z_OK) {
    impl_->put_fail.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const z_result_t rc = z_publisher_put(z_loan(impl_->publishers[handle]),
                                        z_move(payload), nullptr);
  if (rc != Z_OK) {
    impl_->put_fail.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  impl_->puts.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool RelaySession::DeclareSubscriber(const std::string& key,
                                     SampleFn on_sample, std::string* err) {
  if (!is_open()) {
    if (err) *err = "relay session not open";
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->mu);
  z_owned_keyexpr_t ke;
  if (z_keyexpr_from_str(&ke, key.c_str()) != Z_OK) {
    if (err) *err = "relay session: not a valid key expression: " + key;
    return false;
  }
  impl_->callbacks.push_back(
      std::unique_ptr<SampleFn>(new SampleFn(std::move(on_sample))));
  auto* ctx = new CallbackCtx{impl_->callbacks.back().get(), &impl_->samples,
                              &impl_->oversize};
  z_owned_closure_sample_t closure;
  z_closure_sample(&closure, OnSampleTrampoline, OnDropTrampoline, ctx);
  z_owned_subscriber_t sub;
  const z_result_t rc = z_declare_subscriber(
      z_loan(impl_->session), &sub, z_loan(ke), z_move(closure), nullptr);
  z_drop(z_move(ke));
  if (rc != Z_OK) {
    if (err) *err = "relay session: declare_subscriber failed on " + key;
    impl_->callbacks.pop_back();
    return false;
  }
  impl_->subscribers.push_back(sub);
  return true;
}

std::uint64_t RelaySession::samples_received() const {
  return impl_ ? impl_->samples.load(std::memory_order_relaxed) : 0;
}
std::uint64_t RelaySession::samples_oversize() const {
  return impl_ ? impl_->oversize.load(std::memory_order_relaxed) : 0;
}
std::uint64_t RelaySession::puts_sent() const {
  return impl_ ? impl_->puts.load(std::memory_order_relaxed) : 0;
}
std::uint64_t RelaySession::put_failures() const {
  return impl_ ? impl_->put_fail.load(std::memory_order_relaxed) : 0;
}
std::size_t RelaySession::publisher_count() const {
  return impl_ ? impl_->publishers.size() : 0;
}
std::size_t RelaySession::subscriber_count() const {
  return impl_ ? impl_->subscribers.size() : 0;
}

}  // namespace chassis_relay
