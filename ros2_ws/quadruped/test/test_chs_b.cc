/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_chs_b.cc
 * Brief: DDS-1 -- the domain is the configured one even when the environment disagrees
 *
 * Description:
 * The case that matters here sets ROS_DOMAIN_ID to 42 and then requires the
 * participant to have joined domain 0 anyway.
 *
 * That is the only way to test DDS-1. This process holds an rclcpp context on
 * domain 42 at the same time as this participant on domain 0, and both
 * ROS_DOMAIN_ID and CYCLONEDDS_URI are process-wide: an implementation that
 * read either one would put both entities on the same domain. The result is a
 * participant that comes up, reports no error, and never receives anything --
 * which 13 DDS-9 records as indistinguishable from a dead network, on a system
 * where nobody would think to suspect an environment variable.
 *
 * The domain is read BACK from the entity rather than echoed from the config.
 * Echoing would pass on the broken implementation too.
 *
 * Two halves, and the second one is new.
 *
 * With nothing publishing, the reader must report "never seen" rather than a
 * zeroed sample -- the distinction that tells an engineer whether to look at
 * the name mapping or at the cable.
 *
 * Then a WRITER is created in this same process, on the same domain, using the
 * generated descriptor and the mapped topic name, and a sample with known
 * values is published. Until this existed, nothing anywhere proved that a
 * reader built from chassis_dds_types.idl actually receives what a writer using
 * those names sends: test_dds_names compares the mapping as STRINGS, and a
 * string comparison passes just as happily when both sides agree on a name the
 * chassis does not use. This half is what turns "rt/IMU and
 * sensor_msgs::msg::dds_::Imu_" from a claim into a round trip.
 *
 * It also makes the field transcription testable. The IDL comment warns that a
 * reordering "would still connect and would then decode garbage"; a writer that
 * sends distinct values in every field is what catches that, and no amount of
 * reading the file would.
 *
 * *** What this still does NOT establish: that the CHASSIS's IMU is received.
 * The writer here is ours and agrees with the reader by construction about the
 * descriptor. What it proves is the QoS match, the topic-name mapping and the
 * field copy; whether the vendor's type name is the one we wrote down is
 * T-CHS-1a/b, on the bench.
 *
 * *** THE WRITER RUNS ON AN ISOLATED DOMAIN, AND THAT IS NOT A FORMALITY.
 *
 * 13 DDS-4 forbids the quadruped PROCESS from writing on domain 0, because
 * domain 0 is the vendor's. The rule is about the process and not about a
 * test -- but the hazard it names is about the WIRE, and a test is on the same
 * wire. kDomainConfig leaves AllowMulticast on and restricts no interface, so
 * a writer created on domain 0 by anything running on the ORIN reaches the
 * chassis: this test would then publish fabricated rt/IMU samples into the
 * vendor's own domain, on a live robot, and whatever subscribes there would
 * receive them as real.
 *
 * So the round trip runs on kTestDomain, which nothing else uses, and the
 * domain-0 half of this file creates READERS ONLY -- the same shape as the
 * production process. Anyone changing this file has to keep that split.
 */

#include "quadruped/chs_b.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "dds/dds.h"
// The SAME generated descriptors the reader uses. Sharing them is deliberate:
// the question this test answers is whether the reader and a ROS-convention
// writer meet, and a second hand-written descriptor would be answering a
// different question with the same-looking assertions.
#include "chassis_dds_types.h"

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

// Publish one sample and wait for the reader to take it. Bounded, and it FAILS
// rather than returning quietly on timeout: a discovery that never completes
// and a sample that never arrives look the same from here, and both are
// findings.
bool PollUntil(ChassisDds* dds, int* got, double* t) {
  for (int i = 0; i < 2000; ++i) {   // 2000 x 1 ms = 2 s, well past discovery
    *t += 0.001;
    const int n = dds->Poll(*t);
    if (n > 0) {
      *got = n;
      return true;
    }
    dds_sleepfor(DDS_MSECS(1));
  }
  return false;
}

// The domain the round trip runs on. NOT 0: see the file comment -- a writer
// on domain 0 puts fabricated IMU samples on the vendor's wire. Any unused id
// works, and a high one stays clear of the ROS defaults people set by hand.
constexpr int kTestDomain = 89;

ChassisDdsConfig Cfg() {
  ChassisDdsConfig c;
  c.backend = "cyclone_raw";
  c.domain_id = 0;               // 13 DDS-1: the chassis domain
  c.imu_topic = "/IMU";
  c.imu_expect_hz = 200.0;
  c.imu_age_warn_ms = 50;
  c.imu_frame_id = "imu_link";
  c.forward_imu_to_rt = false;
  return c;
}
}  // namespace

int main() {
  // *** Set the environment to the WRONG domain before constructing anything.
  // An implementation that read it would join 42 and never hear the chassis.
  setenv("ROS_DOMAIN_ID", "42", 1);

  ChassisDds dds(Cfg());

  // Read back from the entity. Echoing the config would pass on the broken
  // implementation as well, which is the whole reason this is a read-back.
  CHECK(dds.actual_domain_id() == 0);

  // Nothing is publishing, so nothing arrives. The distinction asserted here is
  // "never seen" versus a zeroed sample: a zeroed one reads as a stationary,
  // level robot, and an engineer shown that would look at the chassis rather
  // than at the topic name.
  ImuSample s;
  CHECK(dds.latest_imu(&s) == false);
  CHECK(dds.imu_age_s(1.0) < 0.0);
  CHECK(ClassifyImuAge(dds.imu_age_s(1.0), 50) == ImuFreshness::kNeverSeen);
  CHECK(dds.imu_count() == 0);

  MotionInfoSample m;
  CHECK(dds.latest_motion_info(&m) == false);
  CHECK(dds.motion_info_age_s(1.0) < 0.0);

  // Polling an empty reader is a no-op, not an error and not a block. The
  // chs_b thread calls this at 200 Hz and must never stall there (DDS-6).
  for (int i = 0; i < 20; ++i) CHECK(dds.Poll(0.001 * i) == 0);
  CHECK(dds.imu_count() == 0);

  // ---- the round trip, on the ISOLATED domain ---------------------------
  //
  // A second reader, configured on kTestDomain, and a writer joining the same
  // one. Everything this half checks -- the topic-name mapping, the QoS match,
  // the field copy -- is independent of WHICH domain it runs on, and running it
  // on the vendor's would mean publishing fabricated IMU samples onto a live
  // robot's wire. The domain-0 instance above stays reader-only.
  ChassisDdsConfig loop_cfg = Cfg();
  loop_cfg.domain_id = kTestDomain;
  ChassisDds loop(loop_cfg);
  CHECK(loop.actual_domain_id() == kTestDomain);

  const dds_entity_t pub_dp =
      dds_create_participant(kTestDomain, nullptr, nullptr);
  CHECK(pub_dp > 0);

  // BEST_EFFORT / KEEP_LAST 1, matching the reader. If these disagreed the two
  // would simply never match, silently on both sides -- which is the failure
  // 13 DDS-2 is about, and the reason the reader's QoS is not a detail.
  dds_qos_t* wqos = dds_create_qos();
  dds_qset_reliability(wqos, DDS_RELIABILITY_BEST_EFFORT, 0);
  dds_qset_history(wqos, DDS_HISTORY_KEEP_LAST, 1);

  // *** The MAPPED name, from the same function the reader uses. Writing
  // "/IMU" here would make the test pass against a reader that also used the
  // ROS spelling -- and that reader hears nothing from the chassis.
  const std::string imu_topic = RosTopicToDdsTopic("/IMU");
  CHECK(imu_topic == "rt/IMU");
  const dds_entity_t imu_t = dds_create_topic(
      pub_dp, &sensor_msgs_msg_dds__Imu__desc, imu_topic.c_str(), nullptr,
      nullptr);
  CHECK(imu_t > 0);
  const dds_entity_t imu_w = dds_create_writer(pub_dp, imu_t, wqos, nullptr);
  CHECK(imu_w > 0);

  // Distinct values in EVERY field that is copied out. A transcription that
  // reordered angular_velocity and linear_acceleration would still connect and
  // would decode garbage -- the IDL comment says so, and this is what makes
  // that statement checkable.
  sensor_msgs_msg_dds__Imu_ msg;
  std::memset(&msg, 0, sizeof(msg));
  char frame[] = "imu_link";
  msg.header.frame_id = frame;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 2;
  msg.orientation.x = 0.11; msg.orientation.y = 0.22;
  msg.orientation.z = 0.33; msg.orientation.w = 0.44;
  msg.angular_velocity.x = 1.5; msg.angular_velocity.y = 2.5;
  msg.angular_velocity.z = 3.5;
  msg.linear_acceleration.x = 4.5; msg.linear_acceleration.y = 5.5;
  msg.linear_acceleration.z = 6.5;

  double t = 1.0;
  int got = 0;
  // BEST_EFFORT drops anything written before discovery finishes, so the write
  // is retried inside the wait rather than done once up front. A single write
  // here is the classic flaky DDS test.
  bool received = false;
  for (int attempt = 0; attempt < 20 && !received; ++attempt) {
    CHECK(dds_write(imu_w, &msg) == DDS_RETCODE_OK);
    received = PollUntil(&loop, &got, &t);
  }
  CHECK(received);

  if (received) {
    CHECK(loop.imu_count() == 1);
    ImuSample r;
    CHECK(loop.latest_imu(&r) == true);
    // Every field, by value. "It arrived" is satisfied by a reader that zeroes
    // everything, and a zeroed IMU reads as a level robot at rest.
    CHECK(r.wx == 1.5);
    CHECK(r.wy == 2.5);
    CHECK(r.wz == 3.5);         // the yaw rate the odometry integrates
    CHECK(r.qx == 0.11);
    CHECK(r.qy == 0.22);
    CHECK(r.qz == 0.33);
    CHECK(r.qw == 0.44);
    CHECK(r.ax == 4.5);
    CHECK(r.ay == 5.5);
    CHECK(r.az == 6.5);
    // OUR clock, not the publisher's stamp (CLK-C4). The stamp said 1.000000002
    // and this must not be it.
    CHECK(r.rx_mono_s == t);
    // ...and freshness is now measured from that, so a sample just taken is
    // neither never-seen nor stale.
    CHECK(loop.imu_age_s(t) == 0.0);
    CHECK(ClassifyImuAge(loop.imu_age_s(t), 50) == ImuFreshness::kFresh);
  }

  // ---- a disposal is NOT a reading (DDS-5) ------------------------------
  //
  // When an instance's state changes, DDS delivers a sample with
  // valid_data == false: the fields are meaningless and only the instance
  // state means anything. Counting one as a reading publishes a zeroed IMU --
  // a level robot at rest -- at the exact moment the publisher went away.
  CHECK(dds_dispose(imu_w, &msg) == DDS_RETCODE_OK);
  const std::uint64_t before_dispose = loop.imu_count();
  for (int i = 0; i < 200; ++i) {
    t += 0.001;
    loop.Poll(t);
    dds_sleepfor(DDS_MSECS(1));
  }
  CHECK(loop.imu_count() == before_dispose);
  ImuSample after;
  if (loop.latest_imu(&after)) {
    // Whatever is held must still be the real sample, not the disposal's zeros.
    CHECK(after.wz == 3.5);
  }

  // ---- the same for /MOTION_INFO, which is RELIABLE ----------------------
  dds_qos_t* rwqos = dds_create_qos();
  dds_qset_reliability(rwqos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
  dds_qset_history(rwqos, DDS_HISTORY_KEEP_LAST, 1);
  const std::string mi_topic = RosTopicToDdsTopic("/MOTION_INFO");
  CHECK(mi_topic == "rt/MOTION_INFO");
  const dds_entity_t mi_t = dds_create_topic(
      pub_dp, &drdds_msg_dds__MotionInfo__desc, mi_topic.c_str(), nullptr,
      nullptr);
  CHECK(mi_t > 0);
  const dds_entity_t mi_w = dds_create_writer(pub_dp, mi_t, rwqos, nullptr);
  CHECK(mi_w > 0);

  drdds_msg_dds__MotionInfo_ mi;
  std::memset(&mi, 0, sizeof(mi));
  mi.data.vel_x = 0.25f;
  mi.data.vel_y = -0.5f;
  mi.data.vel_yaw = 0.75f;
  mi.data.height = 0.4f;
  mi.data.payload = 3.0f;
  mi.data.remain_mile = 12.5f;
  mi.data.motion_state.state = 17;     // the standing read-back (13 V-66)
  mi.data.gait_state.gait = 0x1001u;

  int got_mi = 0;
  bool mi_ok = false;
  for (int attempt = 0; attempt < 20 && !mi_ok; ++attempt) {
    CHECK(dds_write(mi_w, &mi) == DDS_RETCODE_OK);
    mi_ok = PollUntil(&loop, &got_mi, &t);
  }
  CHECK(mi_ok);

  if (mi_ok) {
    MotionInfoSample r;
    CHECK(loop.latest_motion_info(&r) == true);
    // The vendor's fields are float on the wire and double here, so these are
    // compared as the float values they came from -- writing 0.25 and -0.5 and
    // 0.75 keeps that exact rather than making the test about rounding.
    CHECK(r.vel_x == 0.25);
    CHECK(r.vel_y == -0.5);
    CHECK(r.vel_yaw == 0.75);
    // 0.4f widened to double is NOT 0.4, so the comparison goes through the
    // same float. Writing `r.height > 0.0` here instead would be an assertion
    // that passes for every non-zero value the field could possibly hold.
    CHECK(r.height == static_cast<double>(0.4f));
    CHECK(r.motion_state == 17);
    CHECK(r.gait == 0x1001u);
    CHECK(r.rx_mono_s == t);
  }

  dds_delete_qos(rwqos);
  dds_delete_qos(wqos);
  dds_delete(pub_dp);

  if (g_failures == 0) {
    std::printf("ALL CHS_B TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHS_B TEST(S) FAILED\n", g_failures);
  return 1;
}
