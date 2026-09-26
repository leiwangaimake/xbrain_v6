/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_session_config.cc
 * Brief: The session config document -- split out so it needs no zenoh
 *
 * Description:
 * Compiled on its own for the same reason quadruped splits its copy out:
 * the failure a wrong value causes is SILENCE -- a session that connects,
 * reports no error and receives nothing (11 RT-C2, measured 2026-08-23 on
 * the ORIN live stack) -- and a check that only runs where zenoh happens to
 * be installed will not be run by the person who needs it.
 * test_relay_config.cc asserts these bytes with no router and no zenoh, and
 * the relay mutant suite flips each field to prove the assertions can go red.
 *
 * The values are the ones the whole stack runs on (the authority chain:
 * xbrain/common/zenoh/session_factory.py's validator, quadruped's
 * rt_session_config.cc, and common/include/xbrain/zenoh/session_config.h all
 * agree on them):
 *   * mode peer, never router -- RT-C3.d forbids router mode outright, and
 *     the deployment note in 11 S1.1.2 keeps peer over client (99 C01).
 *   * connect = the ONE endpoint the caller passes; both planes of this
 *     process differ ONLY here.
 *   * listen empty -- RT-C3.d: a listening participant is reachable from
 *     outside the router's peer set, which is the property the plane
 *     isolation rests on.
 *   * multicast scouting OFF -- RT-C1: the default is discovery on
 *     224.0.0.224:7446 with autoconnect, which merges the planes.
 *   * gossip ON, multihop OFF -- RT-C2 as corrected 2026-08-23: in the
 *     hub-and-spoke topology a subscription table only reaches a publisher
 *     on another peer BY gossip through the router (enabled:false receives
 *     0 frames, measured); multihop:false is the condition the whole
 *     correction rests on -- it is what keeps RT gossip out of the general
 *     plane's gossip domain. Do NOT "fix" this back to enabled:false; that
 *     is the superseded value 11 S1.1.2's json5 block used to carry.
 */

#include "chassis_relay/relay_session.h"

namespace chassis_relay {

std::string RelaySessionConfigJson(const std::string& endpoint) {
  // Field order follows the json5 block in 11 S1.1.2 so a reader can hold
  // the two side by side; the VALUES follow the corrected contract lines
  // named above. Built once per session at startup -- allocation here never
  // touches a forward path.
  return std::string("{") +
         "mode:\"peer\"," +
         "connect:{endpoints:[\"" + endpoint + "\"]}," +
         // RT-C3.d: empty, always.
         "listen:{endpoints:[]}," +
         "scouting:{" +
         // RT-C1: hard off.
         "multicast:{enabled:false}," +
         // RT-C2 (2026-08-23 correction): enabled true, multihop false.
         "gossip:{enabled:true,multihop:false}" +
         "}}";
}

}  // namespace chassis_relay
