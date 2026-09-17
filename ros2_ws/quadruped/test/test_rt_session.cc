/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rt_session.cc
 * Brief: The RT-plane session config, checked against the OTHER implementation
 *
 * Description:
 * This test exists because the same configuration is written twice in this
 * repository -- once in C++ here and once in Python in
 * xbrain/common/zenoh/session_factory.py -- and two implementations of one
 * configuration diverge silently. The symptom of THIS divergence is not a
 * crash: it is a session that connects, reports no error, and receives nothing.
 *
 * The specific trap, which this file exists to keep shut: 11 S1.1.2's json5
 * code block still reads `gossip: { enabled: false }`, and so does the RT-C3.a
 * row. Both are the SUPERSEDED form. The RT-C2 row in the same section carries
 * a 2026-08-23 correction, and configs/zenoh/router_rt.json5 records the probe
 * that forced it: peer + gossip off gave 0 samples, gossip on gave samples,
 * because in a hub-and-spoke topology a subscription table reaches a remote
 * publisher only by gossip through the router. Anyone implementing a new
 * participant from the code block gets silence.
 *
 * So the assertions come in two halves:
 *   1. the C++ config says what it must say;
 *   2. the Python validator DEMANDS the same thing, checked by reading its
 *      source. If someone "restores" gossip=false on either side, one of these
 *      goes red.
 *
 * Reading the other implementation's source is the same technique test_rt_keys
 * uses against docs/11: compare with the authority rather than with a copy of
 * it. A copy is what drifts.
 *
 * *** What this does NOT establish: that a session actually connects. That
 * needs zenohd-rt running and is a bench item. What is established is that the
 * bytes handed to zenoh are the agreed ones -- which is the half that fails
 * silently.
 */

#include "quadruped/rt_session.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace quadruped;       // NOLINT: test-local
using namespace quadruped::rt;   // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

bool Contains(const std::string& hay, const std::string& needle) {
  return hay.find(needle) != std::string::npos;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string cfg = RtSessionConfigJson("tcp/127.0.0.1:7449");

  // ---- half 1: what the C++ side hands to zenoh --------------------------
  //
  // mode peer. 11 RT-C3.d forbids mode router outright ("永不使用"), and a
  // client-mode session would need the router to proxy every declaration.
  CHECK(Contains(cfg, "mode:\"peer\""));
  CHECK(!Contains(cfg, "\"router\""));

  // Exactly the endpoint passed in, and only it.
  CHECK(Contains(cfg, "connect:{endpoints:[\"tcp/127.0.0.1:7449\"]}"));
  // *** and NOT the general plane. RT-C4 forbids this process a general-plane
  // session; 7447 appearing here at all would mean it had acquired one.
  CHECK(!Contains(cfg, "7447"));

  // RT-C3.d: empty. A listening participant is reachable from outside the
  // router's peer set, which is what the plane isolation rests on.
  CHECK(Contains(cfg, "listen:{endpoints:[]}"));

  // RT-C1: Zenoh's default is multicast discovery plus peer autoconnect over
  // "router" and "peer" -- left on, the two planes find each other.
  CHECK(Contains(cfg, "multicast:{enabled:false}"));

  // *** RT-C2 as corrected. Both halves, and they say opposite things on
  // purpose: gossip must be ON so declarations propagate through the router,
  // and multihop must be OFF because that is the condition the correction
  // rests on and what keeps RT gossip out of the general plane's domain.
  CHECK(Contains(cfg, "gossip:{enabled:true,multihop:false}"));
  CHECK(!Contains(cfg, "gossip:{enabled:false"));

  // The endpoint really is a parameter, not a decoration -- a config that
  // ignored it would pass every assertion above.
  const std::string other = RtSessionConfigJson("tcp/127.0.0.1:7449x");
  CHECK(Contains(other, "tcp/127.0.0.1:7449x"));

  // ---- half 2: the Python validator demands the same ---------------------
  //
  // Read as SOURCE, not imported: the point is to catch an edit to that file,
  // and an import would only catch an edit that also changed its behaviour at
  // the moment this test happened to run.
  if (argc < 2) {
    std::printf("FAIL session_factory.py path not passed as argv[1]\n");
    ++g_failures;
  } else {
    std::ifstream f(argv[1]);
    if (!f) {
      // A comparison that cannot run is not a comparison (same rule as
      // test_rt_keys). Missing file FAILS rather than skipping.
      std::printf("FAIL cannot open %s\n", argv[1]);
      ++g_failures;
    } else {
      std::stringstream ss;
      ss << f.rdbuf();
      const std::string py = ss.str();

      // mode peer, router forbidden.
      CHECK(Contains(py, "_MODE = \"peer\""));
      CHECK(Contains(py, "_MODE_FORBIDDEN = \"router\""));
      // listen endpoints must be empty.
      CHECK(Contains(py, "if doc[\"listen\"][\"endpoints\"]:"));
      // multicast must be False.
      CHECK(Contains(py, "doc[\"scouting\"][\"multicast\"][\"enabled\"] is not False"));
      // *** gossip must be True -- the corrected form. If this line ever reads
      // "is not False", the two implementations have diverged and one of them
      // will be receiving nothing.
      CHECK(Contains(py, "gossip_cfg.get(\"enabled\") is not True"));
      // multihop must not be True.
      CHECK(Contains(py, "gossip_cfg.get(\"multihop\") is True"));
      // ...and the RT endpoint agrees with the one asserted above.
      CHECK(Contains(py, "tcp/127.0.0.1:7449"));
    }
  }

  if (g_failures == 0) {
    std::printf("ALL RT SESSION TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT SESSION TEST(S) FAILED\n", g_failures);
  return 1;
}
