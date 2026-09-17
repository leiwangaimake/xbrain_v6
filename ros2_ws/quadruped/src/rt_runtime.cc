/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_runtime.cc
 * Brief: The rt_pub thread (see rt_runtime.h)
 *
 * Description:
 * Two things here are decisions rather than plumbing.
 *
 * DECLARATIONS COME FROM THE TABLE, not from a list in this file. rt_keys.cc is
 * the transcription of 11 S2.2.1's closed set, and walking it means a key this
 * process publishes or subscribes cannot exist without a row somebody reviewed.
 * A hand-written list here would drift from that table silently, and the
 * symptom of the drift is a key nobody is listening on.
 *
 * A PARTIAL START IS A FAILURE. If one declaration fails, Start returns false
 * and tears the session down rather than running with the rest: the keys that
 * did come up would make the process look connected, and the one that did not
 * might be the estop.
 */

#include "quadruped/rt_runtime.h"

#include <chrono>
#include <cstring>

#include "quadruped/mono_clock.h"
#include "quadruped/rt_keys.h"

namespace quadruped {
namespace rt {
namespace {

// 13 S9.1 rt_pub: 100 Hz, slower keys divided off the same tick.
constexpr double kLoopHz = 100.0;
// rt/chassis/state is a 10 Hz key (13 S7.1), so one publish per ten ticks.
constexpr std::uint64_t kStateDivider = 10;

}  // namespace

RtRuntime::RtRuntime(QuadrupedProcess* proc, const QuadrupedConfig& cfg,
                     std::string boot)
    : proc_(proc), cfg_(cfg), boot_(std::move(boot)) {}

RtRuntime::~RtRuntime() { Stop(); }

bool RtRuntime::Start(std::string* err) {
  if (running_.load(std::memory_order_acquire)) {
    if (err) *err = "rt runtime already started";
    return false;
  }
  if (!session_.Open(cfg_.uplink.zenoh_rt_endpoint, err)) return false;

  // Publishers first, so the bridge has somewhere to send an ack before any
  // subscription can deliver one to answer.
  for (std::size_t i = 0; i < kKeyCount; ++i) {
    if (kKeys[i].role != KeyRole::kPublish) continue;
    const std::string key = BuildKey(cfg_.robot_id, kKeys[i].suffix);
    const int h = session_.DeclarePublisher(key, err);
    if (h < 0) {
      session_.Close();
      return false;
    }
    pub_handles_.push_back({kKeys[i].suffix, h});
  }

  bridge_.reset(new RtBridge(
      proc_, cfg_.robot_id, boot_,
      [this](const std::string& suffix, const char* d, std::size_t n) {
        for (const auto& p : pub_handles_) {
          if (p.first == suffix) return session_.Put(p.second, d, n);
        }
        // An undeclared suffix is a defect, not a fallback: the bridge already
        // aborts at construction for a key outside the table, so reaching here
        // means the table says publish and the declaration did not happen.
        return false;
      }));

  // Subscriptions. The clock is read here, at the edge, and passed inward --
  // every layer below takes time as an argument so a test can drive it.
  struct Sub {
    const char* suffix;
    void (RtBridge::*fn)(double, const char*, std::size_t);
  };
  const Sub subs[] = {
      {"rt/motion/cmd_vel", &RtBridge::HandleCmdVel},
      {"rt/chassis/ctrl", &RtBridge::HandleChassisCtrl},
      {"rt/safety/estop", &RtBridge::HandleEstop},
      {"rt/safety/probe/ping", &RtBridge::HandlePing},
  };
  for (const Sub& s : subs) {
    if (FindKey(s.suffix) == nullptr) {
      if (err) *err = std::string("rt runtime: key not in the table: ") + s.suffix;
      session_.Close();
      return false;
    }
    const std::string key = BuildKey(cfg_.robot_id, s.suffix);
    RtBridge* b = bridge_.get();
    auto fn = s.fn;
    if (!session_.DeclareSubscriber(
            key,
            [b, fn](const char* d, std::size_t n) {
              (b->*fn)(MonoNowSeconds(), d, n);
            },
            err)) {
      session_.Close();
      return false;
    }
  }

  running_.store(true, std::memory_order_release);
  thread_ = std::thread([this] { PubLoop(); });
  return true;
}

void RtRuntime::Stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
  // The session goes down AFTER the loop stops: closing it first would drop
  // subscribers while a callback may still be inside one.
  session_.Close();
}

void RtRuntime::PubLoop() {
  const auto period = std::chrono::duration<double>(1.0 / kLoopHz);
  auto next = std::chrono::steady_clock::now();
  while (running_.load(std::memory_order_acquire)) {
    ++ticks_;
    // 10 Hz off a 100 Hz tick, the same divider pattern ctrl uses for the
    // heartbeat (TX-5). Taking the snapshot only on the tick that publishes
    // means a missed period costs one sample, not a backlog.
    if ((ticks_ % kStateDivider) == 0) {
      QuadrupedProcess::StateSnapshot snap;
      if (proc_->TakeStateForPublish(&snap)) bridge_->PublishState(snap);
    }
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
}

}  // namespace rt
}  // namespace quadruped
