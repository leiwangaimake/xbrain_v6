/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_uplink_alloc.cc
 * Brief: Does one rclcpp odom+tf publish allocate? (13 S9.1 ctrl row vs QD-7)
 *
 * Description:
 * This test exists to keep one ledger entry honest, and it is the only test in
 * the package whose purpose is to CONTRADICT a line of the design.
 *
 * 13 S9.1's thread table puts "odom 积分与发布 + TF" on the ctrl thread and
 * marks that row QD-7 -- no dynamic allocation on the realtime path. QD-7 is
 * not decoration: ctrl runs SCHED_FIFO 80 at 100 Hz, and a malloc there can
 * take the glibc arena lock behind an ordinary-priority thread that already
 * holds it. That is priority inversion on the thread with the 200 ms Tier 1
 * deadline, and it shows up as jitter with no line in any log.
 *
 * The two statements cannot both hold, because publishing a nav_msgs/Odometry
 * through rclcpp allocates. This test measures how much, and 13 V-69 is where
 * the decision is parked.
 *
 * WHAT IT ASSERTS, and why it is not an assertion about the number. A count is
 * a property of this rclcpp, this rmw and these message types; pinning it would
 * make the test fail on an upgrade that changed nothing that matters. What is
 * asserted is the FACT the ledger entry rests on -- that the steady-state
 * publish path allocates at all. If that ever becomes false, this test goes red
 * and V-69 must be re-read rather than quietly inherited.
 *
 * The 50 warm-up publishes are the point of the measurement, not a convenience:
 * a first publish allocates caches that a steady loop never rebuilds, and
 * counting those would prove nothing about a loop that has been running for an
 * hour. What is counted is the cost of one period in the state ctrl is in.
 *
 * This test does NOT decide anything. It does not say the publish should move
 * to rt_pub, or that QD-7 should be relaxed for that row, or that loaned
 * messages are the answer. Those are the three options in V-69 and they belong
 * to whoever owns 13 S9.1.
 */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <new>

#include "quadruped/odometry.h"
#include "quadruped/uplink.h"

namespace {

std::atomic<long> g_new_count{0};
std::atomic<long> g_new_bytes{0};
// Off until the measured window. Counting during construction and warm-up
// would report the startup cost as if it were the per-period cost.
std::atomic<bool> g_counting{false};

}  // namespace

// Replacing the global operator new is the only way to see EVERY allocation,
// including the ones inside rclcpp and the rmw. A hook on a custom allocator
// would see only what was routed through it, which is the subset that is
// already known to be fine.
void* operator new(std::size_t n) {
  if (g_counting.load(std::memory_order_relaxed)) {
    g_new_count.fetch_add(1, std::memory_order_relaxed);
    g_new_bytes.fetch_add(static_cast<long>(n), std::memory_order_relaxed);
  }
  void* p = std::malloc(n ? n : 1);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

using namespace quadruped;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

UplinkConfig Cfg() {
  UplinkConfig c;
  c.ros_domain_id = 42;
  c.rmw = "rmw_cyclonedds_cpp";
  c.odom_topic = "/odom_quadruped";
  c.odom_frame = "odom";
  c.base_frame = "base_link";
  c.publish_odom_tf = true;   // the ctrl row of 13 S9.1 carries the TF too
  c.zenoh_rt_endpoint = "tcp/127.0.0.1:7449";
  return c;
}

OdomSample Fresh() {
  OdomSample s;
  s.x = 1.25;
  s.y = -0.5;
  s.yaw = 0.3;
  s.vx = 0.4;
  s.var_x = 0.01;
  s.var_y = 0.01;
  s.var_yaw = 1e-4;
  s.var_vx = 0.0025;
  s.var_vy = 0.0025;
  s.var_wz = 6.4e-5;
  s.band = OdomBand::kFresh;
  s.valid = true;
  s.publish = true;
  return s;
}

}  // namespace

int main() {
  Uplink up(Cfg(), "gj-001");
  const OdomSample sample = Fresh();

  // Warm up. See the file comment: the first publishes build caches a steady
  // loop does not rebuild, and the question is what one PERIOD costs.
  for (int i = 0; i < 50; ++i) up.Publish(sample, 1789455340.0 + i * 0.01);

  const int kTicks = 100;
  g_counting.store(true, std::memory_order_relaxed);
  for (int i = 0; i < kTicks; ++i) up.Publish(sample, 1789455345.0 + i * 0.01);
  g_counting.store(false, std::memory_order_relaxed);

  const long allocs = g_new_count.load();
  const long bytes = g_new_bytes.load();
  // Printed, never written into a document (CLAUDE.md 3.7). 13 V-69 cites this
  // test by path rather than carrying the numbers.
  std::printf("publishes: %d\n", kTicks);
  std::printf("allocations: %ld (%.1f per publish)\n",
              allocs, allocs / static_cast<double>(kTicks));
  std::printf("bytes: %ld (%.1f per publish)\n",
              bytes, bytes / static_cast<double>(kTicks));

  // Sanity: the publishes happened. Without this, a Publish that returned early
  // would report zero allocations and look like the good news below.
  CHECK(up.odom_published() == static_cast<std::uint64_t>(kTicks) + 50);
  CHECK(up.tf_published() == static_cast<std::uint64_t>(kTicks) + 50);

  // *** The assertion. Red means the steady-state publish stopped allocating,
  // which would be good news AND would make 13 V-69 stale -- so it must be
  // noticed rather than inherited. It is deliberately not an assertion about
  // the count: that is a property of this rclcpp and this rmw.
  if (allocs == 0) {
    std::printf(
        "FAIL the rclcpp publish path no longer allocates in steady state.\n"
        "  This is the premise 13 V-69 rests on (S9.1 puts odom+tf publish on\n"
        "  the ctrl thread and marks that row QD-7). Re-read V-69 and close it\n"
        "  or restate it -- do not just delete this test.\n");
    ++g_failures;
  }

  if (g_failures == 0) {
    std::printf("ALL UPLINK ALLOC TESTS PASSED\n");
    return 0;
  }
  std::printf("%d UPLINK ALLOC TEST(S) FAILED\n", g_failures);
  return 1;
}
