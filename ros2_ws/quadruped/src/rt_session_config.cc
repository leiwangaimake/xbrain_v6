/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_session_config.cc
 * Brief: The RT-plane Zenoh config string -- split out so it needs no zenoh
 *
 * Description:
 * This is the one function of the RT session that can be checked on a machine
 * with no zenoh-c, no router and no robot, so it is compiled on its own.
 *
 * That is not packaging tidiness. The failure this config causes when it is
 * wrong is SILENCE -- a session that connects, reports no error and receives
 * nothing (11 RT-C2, measured 2026-08-23) -- and a check that only runs where
 * zenoh happens to be installed is a check that will not be run by the person
 * who needs it. test_rt_session links this file alone.
 *
 * The values follow xbrain/common/zenoh/session_factory.py, NOT the json5 block
 * in 11 S1.1.2: that block carries the superseded gossip setting. See
 * rt_session.h for the measurement, and test_rt_session.cc for the assertion
 * that keeps the two implementations from drifting apart.
 */

#include "quadruped/rt_session.h"

namespace quadruped {
namespace rt {

std::string RtSessionConfigJson(const std::string& endpoint) {
  // Field order follows the json5 block in 11 S1.1.2 so the two can be held
  // side by side, exactly as session_factory.py does and for the same reason:
  // a document that reads differently from the contract invites a reader to
  // assume something else differs too.
  //
  // The VALUES follow session_factory.py, not that block -- see rt_session.h
  // for the measurement that separates them. test_rt_session compares the two
  // files so this comment cannot drift away from the code it describes.
  return std::string("{") +
         "mode:\"peer\"," +
         "connect:{endpoints:[\"" + endpoint + "\"]}," +
         // RT-C3.d: empty, always. A listening participant is reachable from
         // outside the router's peer set, which is the property the whole
         // plane isolation rests on.
         "listen:{endpoints:[]}," +
         "scouting:{" +
         // RT-C1. Zenoh's default is multicast discovery on 224.0.0.224:7446
         // with peer autoconnect including "router" and "peer" -- left on, the
         // two planes find each other and the isolation is gone.
         "multicast:{enabled:false}," +
         // RT-C2 as corrected 2026-08-23. enabled MUST be true: in the deployed
         // hub-and-spoke topology a peer's subscription table reaches a remote
         // publisher only by gossip through the router. multihop MUST be false:
         // that is the condition the whole correction rests on, and it is what
         // keeps RT gossip out of the general plane's gossip domain.
         "gossip:{enabled:true,multihop:false}" +
         "}}";
}

}  // namespace rt
}  // namespace quadruped
