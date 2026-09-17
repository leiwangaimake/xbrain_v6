/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_parse.h
 * Brief: RT-plane inbound parsing -- the 11 S3.0 envelope and the three commands
 *
 * Description:
 * Everything arriving on the RT plane is decoded here, and the decoding is
 * asymmetric ON PURPOSE. 11 S3.0.1 splits inbound commands by which direction
 * they push the robot:
 *
 *   TIGHTENING (makes the robot LESS likely to move) -- cmd/estop action stop.
 *     Validation is WAIVED. An unknown v, a missing field, truncated JSON, a
 *     bad encoding: every one of them still results in a stop.
 *   LOOSENING (makes the robot MORE likely to move) -- cmd_vel, ctrl enable,
 *     mode switches. FULL validation: known v, every mandatory field present,
 *     a well-formed cmd_id. Anything else is REFUSED, answered E_SCHEMA, and
 *     changes no state.
 *
 * The contract says it in one line: "stop" may fire by mistake, "go" must not.
 *
 * That asymmetry is expressed in the SIGNATURES, not in comments. ParseCmdVel
 * and ParseChassisCtrl return a verdict the caller has to inspect; ParseEstop
 * returns void, because there is no answer it could give that would let a
 * caller skip the stop. A reviewer can see the rule without reading the bodies.
 *
 * WHY THIS FILE HAS NO ZENOH IN IT. Parsing is where the safety rule lives and
 * transport is where the dependency lives; keeping them apart is what lets the
 * rule be tested with no router, no network and no robot -- which is the only
 * way a case like "estop with truncated JSON still stops" gets exercised at all.
 *
 * WHERE IT RUNS. The rt_safety and rt_sub threads (13 S9.1), never ctrl. It
 * allocates -- nlohmann and std::string both do -- and QD-7 forbids that on the
 * realtime path. Same split, and same reason, as the JSON parse in chs_a_rx.
 *
 * TWO ENVELOPE RULES THAT LOOK LIKE DETAILS AND ARE NOT:
 *
 *   * `boot` mismatch means `mono` MUST be ignored (11 S3.0). Another host's
 *     monotonic clock counts from ITS boot; comparing it with ours produces an
 *     age that is wrong by however long the two machines have been up --
 *     typically a number so large the command reads as ancient, or so negative
 *     it reads as arriving from the future. Neither looks like a clock bug.
 *   * `ts_sync` absent means FALSE, never true (11 S3.0, "缺省 / 缺失一律视为
 *     false"). Defaulting to true would let a publisher that never had a
 *     synchronised clock be believed by claiming nothing.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_PARSE_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_PARSE_H_

#include <cstddef>
#include <cstdint>
#include <string>

namespace quadruped {
namespace rt {

// 11 S3.0. The only version this build knows; an unknown one is refused rather
// than guessed at ("接收方遇到不认识的 v 必须拒绝并告警, 不得猜测解析").
inline constexpr int kEnvelopeVersion = 1;

// The envelope, after parsing.
struct Envelope {
  int v = 0;
  std::uint64_t seq = 0;
  // Wall clock, seconds. 11 S3.0 allows it for alignment, recording and latency
  // statistics ONLY -- CLK-C1 forbids it in any age or timeout decision, which
  // is why nothing in this package reads it except a log line.
  double ts = 0.0;
  // CLOCK_MONOTONIC seconds at the publisher. Negative when absent.
  double mono = -1.0;
  // Whether `mono` may be compared with OUR clock. False when the publisher's
  // boot id differs from ours, in which case 11 S3.0 requires falling back to
  // the receive time. A cross-host publisher omits `mono` entirely (CLK-C4).
  bool mono_usable = false;
  // 11 S3.0: absent or malformed means false. Never true by default.
  bool ts_sync = false;
  std::string rid;
  std::string src;
  std::string boot;
};

// Why a loosening message was refused. Separate values because the contract
// answers them differently: a closed-set miss is E_SCHEMA, while an action the
// contract DELETED is E_CAPABILITY plus an event (11 S9.3.3).
enum class RtParse {
  kOk,
  kBadJson,            // not JSON, truncated, or the root is not an object
  kBadVersion,         // `v` is not kEnvelopeVersion
  kWrongRobot,         // `rid` is not ours -- a message for another robot
  kMissingField,       // a mandatory field of 11 S3.0 or the body is absent
  kNotFinite,          // a number that must be finite is NaN or infinite
  kBadValue,           // present, well-typed, outside its closed set
  kUnsupportedAction,  // an action the contract removed -- E_CAPABILITY
};

const char* RtParseName(RtParse r);

// ---------------------------------------------------------------------------
// rt/motion/cmd_vel -- LOOSENING, full validation (11:1722, 11 S3.4)
// ---------------------------------------------------------------------------
struct CmdVelMsg {
  Envelope env;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  double vz = 0.0;
  double v_roll = 0.0;
  double v_pitch = 0.0;
  // 11:1722 marks this MANDATORY on this key, and the row below it marks it
  // ABSENT on rt/nav2/cmd_vel -- the distinction is deliberate, because only
  // this key reaches Tier 1. Tier 1 holds zero until the upstream echoes our
  // generation (13 S9.12.2 (3)); without the field there is nothing to compare,
  // so a message that omits it is not a command this process can act on.
  std::uint64_t estop_epoch = 0;
};

// `our_boot` is compared with the envelope's `boot` to decide mono_usable.
// Pass the first 8 hex of /proc/sys/kernel/random/boot_id, the same string the
// envelope writer stamps.
RtParse ParseCmdVel(const char* json, std::size_t len, const std::string& our_rid,
                    const std::string& our_boot, CmdVelMsg* out);

// ---------------------------------------------------------------------------
// rt/chassis/ctrl -- LOOSENING, full validation (11 S9.3.3 / S9.3.4)
// ---------------------------------------------------------------------------
enum class CtrlAction { kStand, kProne, kEnable, kSetSdkMode };

const char* CtrlActionName(CtrlAction a);

struct ChassisCtrlMsg {
  Envelope env;
  std::string cmd_id;
  CtrlAction action = CtrlAction::kEnable;
  // set_sdk_mode only. joint_rate_hz must be in [1,200] AND divide 1000.
  bool sdk_enable = false;
  int joint_rate_hz = 0;
  // Whatever the message said, kept for the ack's detail.action even when the
  // verdict is a refusal -- an ack that cannot name what it refused is a log
  // line nobody can act on.
  std::string raw_action;
};

RtParse ParseChassisCtrl(const char* json, std::size_t len,
                         const std::string& our_rid, const std::string& our_boot,
                         ChassisCtrlMsg* out);

// ---------------------------------------------------------------------------
// rt/safety/estop -- TIGHTENING, validation WAIVED (11 S3.0.1, S7.1)
// ---------------------------------------------------------------------------
struct EstopMsg {
  Envelope env;
  std::string cmd_id;
  // What could be read out. Both may be false and the stop still happens: they
  // exist so the ACK can say what was understood, never so a caller can decide
  // whether to stop.
  bool envelope_ok = false;
  bool cmd_id_present = false;
};

// Returns void, and that is the design (11 S3.0.1). There is no verdict a
// caller could branch on, because every malformed reading of this key must
// still collapse to "stop" -- 99 U75 calls the property 收紧型 (collapse-safe) and
// forbids extending the waiver to any key with a loosening interpretation.
//
// A caller therefore reads: ParseEstop(...); Stop(); -- with no `if`. Giving
// this function a bool return is exactly the change that would let one appear.
void ParseEstop(const char* json, std::size_t len, const std::string& our_rid,
                const std::string& our_boot, EstopMsg* out);

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_PARSE_H_
