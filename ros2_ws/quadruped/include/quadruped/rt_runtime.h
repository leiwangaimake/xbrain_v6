/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_runtime.h
 * Brief: The rt_pub thread -- session + bridge + the declarations, wired up
 *
 * Description:
 * The only file in the RT path that needs zenoh AND knows what the keys mean,
 * which is why it is small and why everything interesting lives elsewhere:
 * rt_parse decides what a message is, rt_bridge decides what it causes, and
 * this class does nothing but hold them together and turn a thread.
 *
 * WHICH THREAD THIS IS. 13 S9.1's rt_pub row: ordinary priority, 100 Hz, with
 * the slower keys divided down off the same tick -- the same pattern ctrl uses
 * to carry the heartbeat (TX-5), and the reason there is one thread here rather
 * than one per rate.
 *
 * WHAT IT DOES NOT DO YET, said here rather than found later. 13 S9.1 (v1.11,
 * after V-69) gives rt_pub two jobs: the RT-plane keys and the odometry/TF
 * publish. Only the first is wired. The second needs rclcpp, which lives in a
 * separate optional target, and it will join THIS loop when it is wired -- not
 * a second thread. There is deliberately no hook here waiting for it (CLAUDE.md
 * 9.3): the day it lands, this class gains a dependency, and until then the
 * absence is visible rather than papered over by an unused callback.
 *
 * SUBSCRIPTION CALLBACKS RUN ON ZENOH'S THREADS, not on this loop. They go
 * straight into the bridge, which is written for exactly that: parse, hand off,
 * return. Nothing in that path blocks, because a callback that waits stalls the
 * runtime that feeds every other subscription -- including the estop.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_RUNTIME_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_RUNTIME_H_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "quadruped/process.h"
#include "quadruped/quadruped_config.h"
#include "quadruped/rt_bridge.h"
#include "quadruped/rt_session.h"

namespace quadruped {
namespace rt {

class RtRuntime {
 public:
  RtRuntime(QuadrupedProcess* proc, const QuadrupedConfig& cfg,
            std::string boot);
  ~RtRuntime();

  RtRuntime(const RtRuntime&) = delete;
  RtRuntime& operator=(const RtRuntime&) = delete;

  // Opens the session, declares every key in the table for this process's role,
  // and starts the loop. Returns false and fills `err` on the first failure --
  // a partially declared RT plane is worse than none, because the keys that DID
  // come up make the process look connected.
  bool Start(std::string* err);
  void Stop();
  bool running() const { return running_.load(std::memory_order_acquire); }

  const RtBridge& bridge() const { return *bridge_; }
  const RtSession& session() const { return session_; }
  std::uint64_t ticks() const { return ticks_; }

 private:
  void PubLoop();

  QuadrupedProcess* proc_;
  QuadrupedConfig cfg_;
  std::string boot_;
  RtSession session_;
  std::unique_ptr<RtBridge> bridge_;
  // Publisher handles by key suffix, resolved once at Start.
  std::vector<std::pair<std::string, int>> pub_handles_;

  std::atomic<bool> running_{false};
  std::thread thread_;
  std::uint64_t ticks_ = 0;
};

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_RUNTIME_H_
