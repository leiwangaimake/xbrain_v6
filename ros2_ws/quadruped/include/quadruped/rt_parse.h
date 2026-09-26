/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_parse.h
 * Brief: RT-plane inbound parsing -- the 11 S3.0 envelope and the three commands
 *
 * Description:
 * Everything arriving on the RT plane is decoded here, and the decoding is
 * asymmetric ON PURPOSE. 11 S3.0.1 / 13 RX-1 split inbound commands by which
 * direction they push the robot:
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
 * That asymmetry is expressed in the SIGNATURES, not in comments (13 RX-2).
 * ParseCmdVel and ParseChassisCtrl return a verdict the caller has to inspect;
 * ParseEstop returns void, because there is no answer it could give that would
 * let a caller skip the stop. A reviewer can see the rule without reading the
 * bodies.
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
 *   * `boot` mismatch means `mono` MUST be ignored (11 S3.0, 13 RX-4). Another host's
 *     monotonic clock counts from ITS boot; comparing it with ours produces an
 *     age that is wrong by however long the two machines have been up --
 *     typically a number so large the command reads as ancient, or so negative
 *     it reads as arriving from the future. Neither looks like a clock bug.
 *   * `ts_sync` absent means FALSE, never true (11 S3.0, 13 RX-5, "缺省 /
 *     缺失一律视为 false"). Defaulting to true would let a publisher that
 *     never had a synchronised clock be believed by claiming nothing.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_PARSE_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_PARSE_H_

#include <cstddef>
#include <cstdint>
#include <string>

// For the commandable motion_state values (11 S9.2.4). They live in the core
// header beside the steady read-back values on purpose -- see the note there
// on why the pair must not be split.
#include "quadruped/mode_machine.h"

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

// 11 S9.2.4 rt/chassis/mode: the mode TRIPLE. Every field is optional on the
// wire -- a caller that wants to change one thing sends one thing -- so each
// carries its own presence flag. Absent and "set it to zero" are different
// requests, and a struct that could not tell them apart would turn an omitted
// usage_mode into a command to leave navigation mode.
//
// Values are the CHASSIS numbers, mapped from the contract's semantic strings
// at parse time. 11 S9.2.4 is explicit that the strings are the contract and
// "数值映射封装在 quadruped 内部, 底盘枚举变更不外溢" -- so the mapping lives
// here and nowhere upstream.
struct ChassisModeMsg {
  Envelope env;
  std::string cmd_id;
  bool has_usage_mode = false;
  std::int64_t usage_mode = 0;
  bool has_motion_state = false;
  std::int64_t motion_state = 0;
  bool has_gait = false;
  std::int64_t gait = 0;
};

// Semantic string -> chassis number, per the 11 S9.2.4 tables.
//
// The three differ in where their table lives, and that is 13 QD-3's line:
//   * MotionStateValue keeps a LOCAL table that is deliberately a SUBSET of
//     the read-back set -- only stand/prone/rl_control may be commanded;
//     11 S9.2.4 marks soft_estop / idle / joint_damp / boot_damp / zero_cal /
//     cart_move / damped_prone read-only ("只读, 禁止下发"), and accepting one
//     would send a value the contract says only ever comes back.
//   * GaitValue owns NO name table: it delegates to chs_a_reports'
//     GaitValueByName (merged 2026-09-26 -- the two copies had already
//     drifted by one member, platform), then applies a local DIRECTIONAL
//     value exclusion: platform (0x1002) is absent from the guide's command
//     enumeration altogether (13 S5.3 G-03), so it still refuses here.
//     stair_standard stays commandable at this layer -- its refusal is the
//     configured not_implemented.gaits (GS-1), not the parser's.
//   * UsageModeValue keeps a local table whose CONTENT coincides with the
//     read-back set (all three usage modes are commandable) -- a candidate
//     for the same merge, left as-is pending a ruling because unlike gait no
//     reverse lookup exists in chs_a_reports for it yet.
bool UsageModeValue(const std::string& name, std::int64_t* out);
bool MotionStateValue(const std::string& name, std::int64_t* out);
bool GaitValue(const std::string& name, std::int64_t* out);

// 11 S9.1.4 hello: { "type", "proto_version", "client" }. This message has NO
// envelope -- it is the handshake, sent before the two sides have agreed on
// anything, so requiring rid/seq/ts (which the envelope parser enforces) would
// make the handshake depend on the agreement it exists to establish.
struct HelloMsg {
  std::string client;
  int proto_major = 0;
  int proto_minor = 0;
};

// Parses and splits proto_version on the dot. A version that is not
// major.minor is a refusal, not a default: 11 S9.1.4 makes major the
// compatibility decision, and a missing major would have to be guessed.
RtParse ParseHello(const char* json, std::size_t len,
                   const std::string& our_rid, const std::string& our_boot,
                   HelloMsg* out);

// 11 S9.2.4. Every present field must be a KNOWN, COMMANDABLE name; an
// unknown or read-only one refuses the whole message rather than applying the
// fields that happened to parse -- a half-applied mode triple is a state no
// read-back expectation describes (13 MS-5 compares all three).
// 11 S9.4.1 rt/chassis/light. Two halves with very different fates:
//
//   custom.*       C-07, implemented. Names map to the vendor's integers.
//   illumination.* 13 V-47: the front/back lamp switch 11 S9.4 lists comes
//                  from the OLD manual (1.2.6) and does not exist in the
//                  current guide. Until the vendor answers, receiving it is
//                  refused with E_CAPABILITY -- 13 says in as many words
//                  "不静默丢弃, 也不假装设置成功".
//
// So the parser reports illumination's PRESENCE rather than its value: the
// caller has to refuse, and it cannot refuse what the parser silently dropped.
struct LightMsg {
  Envelope env;
  std::string cmd_id;
  // 13 V-47. True when the key was present at all, whatever it held.
  bool has_illumination = false;
  // custom.enable, and the two lamps. Absent custom means "nothing to do" --
  // a message with neither half is refused, because an empty light command is
  // more likely a schema mistake than an intention.
  bool has_custom = false;
  bool custom_enable = false;
  int head_pattern = 0;
  int head_color = 0;
  int head_cycle_s = 0;
  int tail_pattern = 0;
  int tail_color = 0;
  int tail_cycle_s = 0;
};

RtParse ParseLight(const char* json, std::size_t len, const std::string& our_rid,
                   const std::string& our_boot, LightMsg* out);

RtParse ParseChassisMode(const char* json, std::size_t len,
                         const std::string& our_rid, const std::string& our_boot,
                         ChassisModeMsg* out);

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
// rt/safety/probe/ping -- the estop liveness probe (11 S8.5)
// ---------------------------------------------------------------------------
// NEITHER tightening nor loosening: a ping moves nothing, so the 11 S3.0.1
// asymmetry has no side to put it on, and no verdict this parser could return
// would justify staying silent. The caller answers whatever arrives (13 F-15);
// this parser only reports how much of it could be trusted.
//
// *** THE SEQ TO ECHO IS data.seq, NOT the envelope seq (11 S8.5; user ruling
// of 2026-09-27). The envelope seq belongs to the TRANSPORT and RT-C3.e
// REQUIRES every forwarder to overwrite it with its own counter. chassis_relay
// sits between p5_gateway and this process on both legs (CR-2 / CR-3), so an
// envelope echo hands p5_gateway the relay's number instead of the one it
// sent: no reply ever matches, and the RTT that drives estop_path (11 T-23 /
// T-24) is never measured -- while every process involved looks healthy and
// the pongs keep flowing. An end-to-end correlation key survives only inside
// `data`, which forwarders copy byte for byte.
struct ProbePingMsg {
  Envelope env;
  // data.seq. `has_seq` is carried separately because absence is NOT a
  // refusal here: the caller still answers, it simply has nothing
  // trustworthy to echo. Folding the two into "seq == 0 means absent" would
  // work only until a publisher legitimately counts from zero.
  bool has_seq = false;
  std::uint64_t seq = 0;
};

// Returns kOk only when the envelope AND data.seq were both usable; a readable
// envelope with no usable data.seq is kMissingField, same as every other body
// field in this file. `has_seq` is what the caller branches on.
//
// data.seq is read ONLY when the envelope is whole, and that is deliberate
// rather than strict-for-its-own-sake: kWrongRobot means the ping is addressed
// to ANOTHER robot, and lifting a seq out of a message we just refused would
// answer someone else's probe with this robot's estop state.
//
// 13 F-15 applies to the CALLER, not here: whatever this returns, the pong
// still goes out. See RtBridge::HandlePing.
RtParse ParseProbePing(const char* json, std::size_t len,
                       const std::string& our_rid, const std::string& our_boot,
                       ProbePingMsg* out);

// ---------------------------------------------------------------------------
// rt/safety/estop -- TIGHTENING, validation WAIVED (11 S3.0.1, S7.1)
// ---------------------------------------------------------------------------
// 11 S3.11 ClockStatus, reduced to the one field this process consumes.
// 13 Q-5: every message we publish copies its envelope ts_sync from the most
// recent ClockStatus.sync, and CLK-A2 forbids judging sync any other way --
// no chronyc, no clock comparison, nothing. The rest of the message (source,
// offset_ms, ...) is the general plane's business (P1-13 mirrors it there);
// parsing fields nobody here reads would be surface for no consumer.
struct ClockStatusMsg {
  Envelope env;
  bool sync = false;
};

// Envelope rules as everywhere else. `sync` is MANDATORY and boolean: 11
// S3.11 marks it the single system-wide truth, and a message without it is a
// publisher speaking a different schema -- refused, never defaulted, in
// either direction (a default true is CLK-A3's exact failure, a default
// false would silently discard a valid report).
RtParse ParseClockStatus(const char* json, std::size_t len,
                         const std::string& our_rid,
                         const std::string& our_boot, ClockStatusMsg* out);

struct EstopMsg {
  Envelope env;
  std::string cmd_id;
  // 11 S9.12's audit pair, best-effort like everything else on this key:
  // reason is free text, src_role is the five-name audit set (hmi/cloud/
  // voice/agent/test, no authentication -- U23). Both may be empty and the
  // stop happens anyway; they exist so RobotState.last_soft_estop can say
  // "3.2 s ago, by the HMI" (11 S4.1).
  std::string reason;
  std::string src_role;
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
