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
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "quadruped/process.h"
#include "quadruped/quadruped_config.h"
#include "quadruped/rt_bridge.h"
#include "quadruped/rt_session.h"
#include "quadruped/tick_stats.h"

namespace quadruped {
namespace rt {

// Where an OdomSample goes when this loop publishes one. 13 V-69 moved
// odom/TF publishing here from the SCHED_FIFO ctrl thread; the SINK is
// injected rather than called directly because the implementation lives in
// quadruped_uplink, which links rclcpp. This library must not: 13 S5.3 keeps
// the ROS dependency out of everything chassis_relay could ever share, and a
// direct call here would drag rclcpp into the RT-plane library for one
// function. main.cc owns the binding, and a build without ROS simply leaves
// the sink unset -- the loop then publishes rt/chassis/state and nothing else,
// which is what it did before V-69's wiring landed.
using OdomSink = std::function<void(const OdomSample&, double wall_ts_s)>;

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
  // Non-const, for the report sink: the chs_a_rx thread publishes through the
  // bridge, and a const handle cannot. Valid only between Start() and Stop();
  // null before Start, which the caller must not dereference.
  RtBridge* bridge_mut() { return bridge_.get(); }
  const RtSession& session() const { return session_; }
  std::uint64_t ticks() const { return ticks_; }
  // Must be called BEFORE Start(). Setting it while the loop runs would be a
  // data race on a std::function, and there is no case for changing where odom
  // goes mid-flight -- the destination is a property of the build.
  void SetOdomSink(OdomSink sink) { odom_sink_ = std::move(sink); }
  // T-ODOM-1's evidence (13 S11.1): the measured period of THIS loop.
  // Read after Stop(); reading it while the loop runs gives a torn value, and
  // the item asks about a completed 10-minute run, not a live number.
  const TickStats& tick_stats() const { return tick_stats_; }
  // How many samples the loop actually handed to the sink. Separate from
  // ticks(): TakeFresh returns nothing when ctrl has not produced a new
  // sample since the last read, and "the loop ran" is not "odom went out".
  std::uint64_t odom_sent() const { return odom_sent_; }

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
  OdomSink odom_sink_;
  TickStats tick_stats_;
  std::uint64_t odom_sent_ = 0;
};

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_RUNTIME_H_
