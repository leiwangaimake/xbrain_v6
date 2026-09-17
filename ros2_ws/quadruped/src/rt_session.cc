/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_session.cc
 * Brief: The RT-plane Zenoh session (see rt_session.h)
 *
 * Description:
 * Three zenoh-c details that cost time if rediscovered:
 *
 *   * z_keyexpr_from_str is used, NOT z_view_keyexpr_from_str. The view form
 *     canonises the string IN PLACE, and the header says in so many words that
 *     it "may SEGFAULT if expr lies in read-only memory (as values initialized
 *     with string literals do)". The owned form copies. It costs one allocation
 *     per declaration, at startup, on a non-realtime thread.
 *   * every z_owned_* handed to a zenoh call must be MOVED (z_move). Passing it
 *     by address without moving leaves the caller owning a value zenoh has
 *     taken, and the double drop lands at shutdown -- far from the cause.
 *   * the sample callback receives a LOANED sample. Its payload is valid only
 *     for the duration of the call, so the bytes are copied out before the
 *     std::function runs. Handing the loan onward is the same class of mistake
 *     as holding a DDS loan across a thread boundary (13 DDS-5).
 */

#include "quadruped/rt_session.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include "zenoh.h"

namespace quadruped {
namespace rt {


struct RtSession::Impl {
  z_owned_session_t session;
  bool open = false;
  std::vector<z_owned_publisher_t> publishers;
  std::vector<z_owned_subscriber_t> subscribers;
  // One per subscriber, owned here so the callback context outlives the
  // declaration. A lambda captured by pointer into a vector that later
  // reallocates is the classic way this goes wrong, so these are never moved.
  std::vector<std::unique_ptr<SampleFn>> callbacks;
  std::mutex mu;   // declarations only -- never held on a publish path

  std::atomic<std::uint64_t> samples{0};
  std::atomic<std::uint64_t> puts{0};
  std::atomic<std::uint64_t> put_fail{0};
};

namespace {

struct CallbackCtx {
  RtSession::SampleFn* fn;
  std::atomic<std::uint64_t>* counter;
};

void OnSampleTrampoline(z_loaned_sample_t* sample, void* context) {
  auto* ctx = static_cast<CallbackCtx*>(context);
  if (ctx == nullptr || ctx->fn == nullptr) return;
  ctx->counter->fetch_add(1, std::memory_order_relaxed);

  const z_loaned_bytes_t* payload = z_sample_payload(sample);
  z_owned_slice_t slice;
  if (z_bytes_to_slice(payload, &slice) != Z_OK) return;
  const char* data = reinterpret_cast<const char*>(z_slice_data(z_loan(slice)));
  const std::size_t len = z_slice_len(z_loan(slice));
  // Copied out inside the call: the loan is valid only while this returns.
  (*ctx->fn)(data, len);
  z_drop(z_move(slice));
}

void OnDropTrampoline(void* context) {
  delete static_cast<CallbackCtx*>(context);
}

}  // namespace

RtSession::RtSession() : impl_(new Impl()) {}

RtSession::~RtSession() { Close(); }

bool RtSession::Open(const std::string& endpoint, std::string* err) {
  if (impl_->open) {
    if (err) *err = "rt session already open";
    return false;
  }
  const std::string cfg_json = RtSessionConfigJson(endpoint);
  z_owned_config_t cfg;
  if (zc_config_from_str(&cfg, cfg_json.c_str()) != Z_OK) {
    if (err) *err = "rt session: config rejected by zenoh: " + cfg_json;
    return false;
  }
  if (z_open(&impl_->session, z_move(cfg), nullptr) != Z_OK) {
    // No retry here. "The RT router is not running" and "the RT router is
    // running and nobody is publishing" are different faults with different
    // remedies, and a silent retry loop turns the first into the second.
    if (err) *err = "rt session: z_open failed against " + endpoint;
    return false;
  }
  impl_->open = true;
  return true;
}

void RtSession::Close() {
  if (!impl_ || !impl_->open) return;
  // Subscribers first: a subscriber whose session is already gone still has a
  // callback the runtime may be inside.
  for (auto& s : impl_->subscribers) z_drop(z_move(s));
  impl_->subscribers.clear();
  for (auto& p : impl_->publishers) z_drop(z_move(p));
  impl_->publishers.clear();
  impl_->callbacks.clear();
  z_drop(z_move(impl_->session));
  impl_->open = false;
}

bool RtSession::is_open() const { return impl_ && impl_->open; }

int RtSession::DeclarePublisher(const std::string& key, std::string* err) {
  if (!is_open()) {
    if (err) *err = "rt session not open";
    return -1;
  }
  std::lock_guard<std::mutex> lock(impl_->mu);
  z_owned_keyexpr_t ke;
  if (z_keyexpr_from_str(&ke, key.c_str()) != Z_OK) {
    if (err) *err = "rt session: not a valid key expression: " + key;
    return -1;
  }
  z_owned_publisher_t pub;
  const z_result_t rc =
      z_declare_publisher(z_loan(impl_->session), &pub, z_loan(ke), nullptr);
  z_drop(z_move(ke));
  if (rc != Z_OK) {
    if (err) *err = "rt session: declare_publisher failed on " + key;
    return -1;
  }
  impl_->publishers.push_back(pub);
  return static_cast<int>(impl_->publishers.size()) - 1;
}

bool RtSession::Put(int handle, const char* data, std::size_t len) {
  if (!is_open() || data == nullptr) return false;
  if (handle < 0 || static_cast<std::size_t>(handle) >= impl_->publishers.size()) {
    return false;
  }
  z_owned_bytes_t payload;
  if (z_bytes_copy_from_buf(&payload, reinterpret_cast<const uint8_t*>(data),
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

bool RtSession::DeclareSubscriber(const std::string& key, SampleFn on_sample,
                                  std::string* err) {
  if (!is_open()) {
    if (err) *err = "rt session not open";
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->mu);
  z_owned_keyexpr_t ke;
  if (z_keyexpr_from_str(&ke, key.c_str()) != Z_OK) {
    if (err) *err = "rt session: not a valid key expression: " + key;
    return false;
  }
  // Owned here, never in the vector's storage: the vector reallocates, and a
  // callback context pointing into old storage is a use-after-free that only
  // shows up once enough subscriptions exist to force a growth.
  impl_->callbacks.push_back(std::unique_ptr<SampleFn>(new SampleFn(on_sample)));
  auto* ctx = new CallbackCtx{impl_->callbacks.back().get(), &impl_->samples};

  z_owned_closure_sample_t closure;
  z_closure_sample(&closure, OnSampleTrampoline, OnDropTrampoline, ctx);

  z_owned_subscriber_t sub;
  const z_result_t rc = z_declare_subscriber(z_loan(impl_->session), &sub,
                                             z_loan(ke), z_move(closure), nullptr);
  z_drop(z_move(ke));
  if (rc != Z_OK) {
    if (err) *err = "rt session: declare_subscriber failed on " + key;
    impl_->callbacks.pop_back();
    return false;
  }
  impl_->subscribers.push_back(sub);
  return true;
}

std::size_t RtSession::publisher_count() const {
  return impl_ ? impl_->publishers.size() : 0;
}
std::size_t RtSession::subscriber_count() const {
  return impl_ ? impl_->subscribers.size() : 0;
}
std::uint64_t RtSession::samples_received() const {
  return impl_ ? impl_->samples.load(std::memory_order_relaxed) : 0;
}
std::uint64_t RtSession::puts_sent() const {
  return impl_ ? impl_->puts.load(std::memory_order_relaxed) : 0;
}
std::uint64_t RtSession::put_failures() const {
  return impl_ ? impl_->put_fail.load(std::memory_order_relaxed) : 0;
}

}  // namespace rt
}  // namespace quadruped
