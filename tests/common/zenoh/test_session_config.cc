/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_session_config.cc
 * Brief: INF-ZN-1 C++ replica -- RT-C1, RT-C2, RT-C3.a and RT-C3.d in gtest
 *
 * Description:
 * The same cases tests/common/zenoh/test_session_factory.py runs against the
 * Python factory, run against common/include/xbrain/zenoh/session_config.h. Both
 * halves exist because the consumers are split: chassis_relay and quadruped are
 * C++ and perception is C++, while P1 through P5 are Python, and every one of
 * them must put the same five fields in its session config or the two Zenoh
 * planes stop being isolated (11 S1.1.2, grep
 * "RT 面配置与强制约束 RT-C1 ~ RT-C3").
 *
 * *** The endpoints below are transcribed from 11, not read from the header
 * under test. Reading them from the header would compare it against itself and
 * pass for any value. The Python side carries the same transcription and greps
 * the contract to prove it is current; this file is held to the Python side by
 * tests/common/zenoh/test_session_config_cxx.py, which runs the --emit mode
 * below and compares field by field.
 *
 * Why this file owns main() instead of linking gtest_main. --emit has to run
 * before gtest sees the arguments, because it is not a test: it prints the
 * document so the Python driver can compare the two implementations. Wiring
 * that through a test body would mean parsing gtest's output to recover it.
 *
 * Nothing here links a zenoh binding. 11 S2.4.1 records the version as unlocked
 * and neither zenoh-c nor zenoh-cpp is installed; the header emits text and this
 * file checks the text and the fields.
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "gtest/gtest.h"
#include "xbrain/zenoh/session_config.h"

// Shorthand for the namespace under test. Deliberately a namespace alias and not
// a using-directive: a using-directive would pull PlaneConfig and ToJson5 into
// this file's lookup, and the whole point of the file is to be explicit about
// which implementation each name comes from.
//
// At file scope rather than inside the anonymous namespace below, because main()
// uses it too and main() cannot live in an anonymous namespace.
namespace zc = hachist::xbrain::zenoh_config;

namespace {

// Transcribed from 11. See the header note above for why they are not read from
// the implementation.
//   RT  -- 11 S1.1.2 json5 block
//   GEN -- 11 S1.1.4 部署形态 row and S1.1.7 plane table, both reading
//          "connect tcp/127.0.0.1:7447" for on-board participants
const char* const kRtConnect = "tcp/127.0.0.1:7449";
const char* const kGenConnect = "tcp/127.0.0.1:7447";

// The full document each plane must serialise to. Written out rather than
// assembled from the two constants above, so that a change to the emitter's
// FORMAT -- a lost quote, a renamed parent key -- is caught here and not only by
// the cross-language driver. A document Zenoh parses into the wrong tree is the
// failure mode that matters: scouting.gossip under a misspelled parent is valid
// json, and the binding then falls back to its default, which for multicast is
// on.
const char* const kRtJson5 =
    "{\"mode\":\"peer\",\"connect\":{\"endpoints\":[\"tcp/127.0.0.1:7449\"]},"
    "\"listen\":{\"endpoints\":[]},\"scouting\":{\"multicast\":{\"enabled\":"
    "false},\"gossip\":{\"enabled\":false}}}";
const char* const kGenJson5 =
    "{\"mode\":\"peer\",\"connect\":{\"endpoints\":[\"tcp/127.0.0.1:7447\"]},"
    "\"listen\":{\"endpoints\":[]},\"scouting\":{\"multicast\":{\"enabled\":"
    "false},\"gossip\":{\"enabled\":false}}}";

// Both planes get the identical field checks. Written once and called twice
// rather than copied, because a copied second case is how one plane keeps a
// setting the other had fixed -- and RT-C1 and RT-C2 both say 两个 Zenoh 平面
// 都必须, which is exactly that failure stated as a rule.
void ExpectContractFields(const zc::SessionConfig& cfg, const char* connect) {
  // RT-C3.d. Both directions asserted: equality alone would still pass if the
  // value were later changed to something that is neither peer nor router, and
  // the router case is the one with a stated consequence -- a router propagates
  // subscription declarations between the planes.
  EXPECT_STREQ("peer", cfg.mode);
  EXPECT_STRNE(zc::ForbiddenMode(), cfg.mode);
  // RT-C3.d, empty listen.
  EXPECT_EQ(0u, cfg.listen_endpoint_count);
  // RT-C1 and RT-C2.
  EXPECT_FALSE(cfg.scouting_multicast_enabled);
  EXPECT_FALSE(cfg.scouting_gossip_enabled);
  // Exactly one connect endpoint is structural here: the struct holds a single
  // pointer, so "unique" cannot be violated by appending. What can go wrong is
  // the value, and that is what is checked.
  EXPECT_STREQ(connect, cfg.connect_endpoint);
  EXPECT_TRUE(zc::IsContractCompliant(cfg));
}

TEST(SessionConfig, RtPlaneFields) {
  ExpectContractFields(zc::PlaneConfig(zc::Plane::kRt), kRtConnect);
}

TEST(SessionConfig, GenPlaneFields) {
  ExpectContractFields(zc::PlaneConfig(zc::Plane::kGen), kGenConnect);
}

// The two planes must reach different routers. Asserted on its own so a table
// where both rows carry 7449 fails here, naming the cause, rather than failing a
// per-plane case with a confusing expected-value message.
TEST(SessionConfig, PlanesUseDifferentEndpoints) {
  const zc::SessionConfig rt = zc::PlaneConfig(zc::Plane::kRt);
  const zc::SessionConfig gen = zc::PlaneConfig(zc::Plane::kGen);
  EXPECT_STRNE(rt.connect_endpoint, gen.connect_endpoint);
}

// RT-C3.a, identity half (grep "两个独立的 Zenoh runtime"). An implementation
// handing out a reference to one static instance fails here.
TEST(SessionConfig, ConfigsAreDistinctObjects) {
  // decltype(auto), not a value binding. This is the whole case: a value
  // binding COPIES whatever PlaneConfig returns, so an implementation handing
  // out a reference to one static would still produce distinct addresses here
  // and the case would pass with the defect it exists to catch fully present.
  // decltype(auto) keeps the reference when there is one.
  decltype(auto) rt = zc::PlaneConfig(zc::Plane::kRt);
  decltype(auto) gen = zc::PlaneConfig(zc::Plane::kGen);
  EXPECT_NE(&rt, &gen);
  // Two calls for the same plane must also be two objects. Without this, a
  // per-plane static passes the cross-plane assertion above while still handing
  // two threads one shared object.
  decltype(auto) rt_again = zc::PlaneConfig(zc::Plane::kRt);
  EXPECT_NE(&rt, &rt_again);
}

// RT-C3.a, cross-talk half. Distinct addresses do not imply independent state --
// that only holds while the values are copied, which is what returning by value
// buys and what a reference return would take away.
TEST(SessionConfig, ConfigsDoNotShareState) {
  zc::SessionConfig rt = zc::PlaneConfig(zc::Plane::kRt);
  zc::SessionConfig gen = zc::PlaneConfig(zc::Plane::kGen);
  rt.scouting_gossip_enabled = true;
  rt.listen_endpoint_count = 1;
  // The edit is confirmed on the object that was edited before the other one is
  // checked. Without this the case would pass vacuously if the write never took
  // effect -- "gen is unchanged" says nothing when nothing was changed anywhere.
  EXPECT_TRUE(rt.scouting_gossip_enabled);
  EXPECT_EQ(1u, rt.listen_endpoint_count);
  EXPECT_FALSE(gen.scouting_gossip_enabled);
  EXPECT_EQ(0u, gen.listen_endpoint_count);
  // And a config fetched afterwards must be clean, which is what fails if the
  // table is a mutable static the first call handed out.
  const zc::SessionConfig rt_again = zc::PlaneConfig(zc::Plane::kRt);
  EXPECT_FALSE(rt_again.scouting_gossip_enabled);
  EXPECT_EQ(0u, rt_again.listen_endpoint_count);
}

// The self-check has to be able to say no. A checker that returns true for
// everything passes every case above, so each violation is fed to it here --
// this is the mutation CLAUDE.md 3.3 asks for, written as a case rather than
// left to a manual edit.
TEST(SessionConfig, ContractCheckRejectsEachViolation) {
  const zc::SessionConfig good = zc::PlaneConfig(zc::Plane::kRt);
  ASSERT_TRUE(zc::IsContractCompliant(good));

  zc::SessionConfig as_router = good;
  as_router.mode = "router";
  EXPECT_FALSE(zc::IsContractCompliant(as_router));

  zc::SessionConfig listening = good;
  listening.listen_endpoint_count = 1;
  EXPECT_FALSE(zc::IsContractCompliant(listening));

  zc::SessionConfig multicast_on = good;
  multicast_on.scouting_multicast_enabled = true;
  EXPECT_FALSE(zc::IsContractCompliant(multicast_on));

  zc::SessionConfig gossip_on = good;
  gossip_on.scouting_gossip_enabled = true;
  EXPECT_FALSE(zc::IsContractCompliant(gossip_on));

  // An empty endpoint string is not the same as no endpoint: the binding would
  // accept the document and fail at session-open with a message naming the empty
  // string rather than the config that produced it.
  zc::SessionConfig no_endpoint = good;
  no_endpoint.connect_endpoint = "";
  EXPECT_FALSE(zc::IsContractCompliant(no_endpoint));
}

// The emitted text, pinned. See kRtJson5 above for why the whole document is
// compared rather than a set of substrings.
TEST(SessionConfig, Json5MatchesTheContractDocument) {
  EXPECT_EQ(std::string(kRtJson5), zc::ToJson5(zc::PlaneConfig(zc::Plane::kRt)));
  EXPECT_EQ(std::string(kGenJson5), zc::ToJson5(zc::PlaneConfig(zc::Plane::kGen)));
}

// A document that breaks RT-C3.d must not serialise as though it did not. This
// is the emitter's own always-green risk: writing "[]" unconditionally would
// make every config look compliant no matter what it carries.
TEST(SessionConfig, Json5DoesNotHideAListenEndpoint) {
  zc::SessionConfig listening = zc::PlaneConfig(zc::Plane::kRt);
  listening.listen_endpoint_count = 1;
  const std::string text = zc::ToJson5(listening);
  EXPECT_EQ(std::string::npos, text.find("\"listen\":{\"endpoints\":[]}"));
}

TEST(SessionConfig, PlaneNames) {
  // The names are what the --emit mode selects on and what a log line carries,
  // so a silent rename would break the cross-language driver in a way that reads
  // as a missing plane rather than a renamed one.
  EXPECT_STREQ("rt", zc::PlaneName(zc::Plane::kRt));
  EXPECT_STREQ("gen", zc::PlaneName(zc::Plane::kGen));
}

}  // namespace

// main -- gtest, plus the --emit mode the cross-language driver uses.
//
// argv[1] is compared before gtest initialises, since --emit is not a gtest flag
// and gtest would report it as unrecognised. Exactly two arguments are required:
// a lenient parser here would let a typo in the driver silently select a
// different plane, and the driver would then compare the RT document against the
// general-plane expectation and report a drift that does not exist.
int main(int argc, char** argv) {
  if (argc == 3 && std::strcmp(argv[1], "--emit") == 0) {
    if (std::strcmp(argv[2], "rt") == 0) {
      std::printf("%s", zc::ToJson5(zc::PlaneConfig(zc::Plane::kRt)).c_str());
      return 0;
    }
    if (std::strcmp(argv[2], "gen") == 0) {
      std::printf("%s", zc::ToJson5(zc::PlaneConfig(zc::Plane::kGen)).c_str());
      return 0;
    }
    // An unknown plane is an error, never a default. A default would hand the
    // driver a valid document for the wrong plane, and the comparison would fail
    // pointing at the implementation instead of at the argument.
    std::fprintf(stderr, "unknown plane %s, expected rt or gen\n", argv[2]);
    return 2;
  }
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
