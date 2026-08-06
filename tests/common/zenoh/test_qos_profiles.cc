/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_qos_profiles.cc
 * Brief: gtest replica of the frozen QoS table, plus the --emit mode Python
 *        compares against
 *
 * Description:
 * What this covers. common/include/xbrain/zenoh/qos_profiles.h is the C++17 half
 * of the 11 S2.4.2 frozen table, consumed by chassis_relay, perception and
 * quadruped. Two things have to hold and neither is checked by the header
 * existing: the values must be the ones 11 states, and they must be the same
 * values the Python half carries. The cases below take the first; --emit hands
 * the table to tests/common/zenoh/test_qos_profiles_cxx.py, which takes the
 * second.
 *
 * Why the assertions are written against literals transcribed from 11 rather
 * than against the header's own constants. Comparing the header with itself
 * passes for any value at all, which is CLAUDE.md 3.2 form 1. These literals are
 * a second transcription; the Python test greps 11 to prove both are current.
 *
 * *** What this does NOT establish. Nothing here opens a Zenoh session or sets
 * a single QoS knob on a real publisher. 11 S2.4.1 records the Zenoh version as
 * unlocked (grep "Zenoh 版本待锁定") and warns that 1.x moved reliability from
 * the subscriber side to the publisher side, so the API names these values map
 * onto are not yet fixed. The values are the contract; the mapping is the
 * consumer's, and QoS-T1 to QoS-T8 in 11 S2.4.9 are still pending T7.
 */

#include "xbrain/zenoh/qos_profiles.h"

#include <gtest/gtest.h>

#include <cstring>
#include <iostream>
#include <string>

namespace {

namespace qos = hachist::xbrain::qos;

// The frozen table, transcribed from 11 S2.4.2 a second time. Same order as the
// section, so a reader can hold the two side by side.
//
// Two fields are compared with strcmp rather than ==: these are const char*, and
// == would compare addresses. The comparison would then be right or wrong
// depending on whether the build merged identical literals, which is the worst
// kind of test -- one that passes on the machine it was written on.
TEST(QosProfiles, FrozenTableMatchesTheContract) {
  ASSERT_EQ(qos::ProfileCount(), 5u);

  const qos::QosProfile* q0 = qos::FindProfile("Q0_safety");
  ASSERT_NE(q0, nullptr);
  EXPECT_STREQ(q0->congestion_control, "drop");
  EXPECT_STREQ(q0->priority, "real_time");
  EXPECT_STREQ(q0->reliability, "reliable");
  EXPECT_TRUE(q0->express);
  EXPECT_STREQ(q0->handler.kind, "ring");
  EXPECT_EQ(q0->handler.depth, 8);

  const qos::QosProfile* q1 = qos::FindProfile("Q1_rt");
  ASSERT_NE(q1, nullptr);
  EXPECT_STREQ(q1->reliability, "best_effort");
  EXPECT_TRUE(q1->express);
  // Ring(1), which 11 S2.4.6 argues is the difference between bounded lag and
  // lag that diverges, and between an age check that works and one that is
  // silently vacuous.
  EXPECT_STREQ(q1->handler.kind, "ring");
  EXPECT_EQ(q1->handler.depth, 1);

  const qos::QosProfile* q2 = qos::FindProfile("Q2_state");
  ASSERT_NE(q2, nullptr);
  EXPECT_STREQ(q2->priority, "data_high");
  EXPECT_FALSE(q2->express);
  EXPECT_EQ(q2->handler.depth, 4);

  const qos::QosProfile* q3 = qos::FindProfile("Q3_cmd");
  ASSERT_NE(q3, nullptr);
  // The only profile carrying block, and the only one where block is right --
  // 11 S2.4.5 第 6 条 shows that dropping an event advances the replay cursor
  // past it, so the event is marked sent and lost with nobody informed.
  EXPECT_STREQ(q3->congestion_control, "block");
  EXPECT_STREQ(q3->handler.kind, "fifo");
  EXPECT_EQ(q3->handler.depth, 256);
}

// Q4's depth is the sentinel, and the guard against using it as a size must say
// so. A consumer that read the field directly would build a ring buffer of zero
// chunks, drop every one of them, and log nothing.
TEST(QosProfiles, Q4DepthIsNotSupplied) {
  const qos::QosProfile* q4 = qos::FindProfile("Q4_stream");
  ASSERT_NE(q4, nullptr);
  EXPECT_STREQ(q4->congestion_control, "drop");
  EXPECT_STREQ(q4->priority, "interactive_high");
  EXPECT_EQ(q4->handler.depth, qos::kDepthNotSupplied);
  EXPECT_FALSE(qos::HasSuppliedDepth(q4->handler));
  // And every other profile does have one, so the guard is not vacuously true
  // for the whole table.
  EXPECT_TRUE(qos::HasSuppliedDepth(qos::FindProfile("Q0_safety")->handler));
}

// QOS-C1. The override applies on the rt plane to a profile whose congestion
// control would be block, and nowhere else.
TEST(QosProfiles, RtOverrideAppliesOnlyToBlockOnTheRtPlane) {
  const qos::RtOverrideSpec spec = qos::RtOverride();
  EXPECT_STREQ(spec.congestion_control, "drop");
  EXPECT_STREQ(spec.priority, "interactive_high");
  EXPECT_STREQ(spec.handler.kind, "fifo");
  EXPECT_EQ(spec.handler.depth, 32);

  const qos::QosProfile* q3 = qos::FindProfile("Q3_cmd");
  const qos::QosProfile* q1 = qos::FindProfile("Q1_rt");
  ASSERT_NE(q3, nullptr);
  ASSERT_NE(q1, nullptr);
  // Q3 on rt: overridden. This is rt/behavior/request in the Python vectors.
  EXPECT_TRUE(qos::RequiresRtOverride("rt", *q3));
  // Q3 on the general plane: not overridden. cmd/** and event/** keep block,
  // which is the back-pressure S2.4.5 第 6 条 requires there.
  EXPECT_FALSE(qos::RequiresRtOverride("cmd", *q3));
  // Q1 on rt: nothing to override, it never carried block.
  EXPECT_FALSE(qos::RequiresRtOverride("rt", *q1));
}

// Lookups must fail as a null pointer, never as a plausible profile. 13 CPP-2
// forbids throwing from here, and returning any row for an unknown name would
// let a publisher come up with QoS nobody chose -- 11 S2.4.8 A-7.
TEST(QosProfiles, UnknownNamesAndIndicesReturnNull) {
  EXPECT_EQ(qos::FindProfile("Q5_custom"), nullptr);
  EXPECT_EQ(qos::FindProfile(nullptr), nullptr);
  EXPECT_EQ(qos::ProfileAt(qos::ProfileCount()), nullptr);
  EXPECT_NE(qos::ProfileAt(0), nullptr);
}

}  // namespace

// main is owned here rather than linked from gtest_main so that --emit can run
// before gtest parses the argument list. Linking gtest_main as well would give
// two definitions of main and fail at link.
int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--emit") == 0) {
    std::cout << hachist::xbrain::qos::TableToJson();
    return 0;
  }
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
