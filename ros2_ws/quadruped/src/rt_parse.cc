/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_parse.cc
 * Brief: RT-plane inbound parsing (see rt_parse.h)
 *
 * Description:
 * The envelope reader is shared by all three commands and the two command
 * readers then diverge, which is the whole point: ParseEnvelope reports what it
 * found, and each caller decides what a gap MEANS. For a loosening command a
 * gap is a refusal; for the estop it is nothing at all.
 *
 * One implementation note worth more than the rest. GetFinite below refuses
 * NaN and infinity rather than passing them through, and it does so for fields
 * the robot does not even use on an ordinary gait (vz, v_roll, v_pitch). That
 * is not tidiness: 11 S9.12.1 lists a non-finite axis as 载荷污染, and a message
 * carrying a NaN in a field nobody reads is not a message whose OTHER fields
 * can be trusted. Tier 1 has the same rule on the same six axes, one layer
 * down, and the two are independent on purpose.
 */

#include "quadruped/rt_parse.h"

#include <cmath>

#include "nlohmann/json.hpp"

namespace quadruped {
namespace rt {
namespace {

using Json = nlohmann::json;

// True when the key exists and holds a finite number, which is written out
// rather than folded into a default-returning helper: a default would turn
// "the field is missing" into "the field said zero", and zero is a legal
// velocity. The whole loosening rule rests on telling those two apart.
//
// THE isfinite HALF IS UNREACHABLE THROUGH THIS READER, and it stays
// (CLAUDE.md 7.2.1). Measured 2026-09-17: JSON has no NaN literal at all, and
// nlohmann DISCARDS a whole document containing an overflowing one (1e999), so
// a non-finite value is refused a layer earlier and arrives here as kBadJson.
// The guard is kept because "the reader rejects it for us" is a property of the
// reader, not of this function, and the next person to swap the reader would
// inherit the assumption without being told. Registered as a declared
// equivalent mutant rather than defended by a test that cannot fail.
bool GetFinite(const Json& j, const char* key, double* out) {
  const auto it = j.find(key);
  if (it == j.end() || !it->is_number()) return false;
  const double v = it->get<double>();
  if (!std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool GetString(const Json& j, const char* key, std::string* out) {
  const auto it = j.find(key);
  if (it == j.end() || !it->is_string()) return false;
  *out = it->get<std::string>();
  return true;
}

// Reads the 11 S3.0 envelope. Returns kOk only when every mandatory field is
// present and usable; the caller decides whether that matters.
//
// It fills `out` as far as it got even when it fails, because the estop path
// needs whatever could be read in order to answer an ack, and a parser that
// wiped its output on failure would leave that ack blank.
RtParse ReadEnvelope(const Json& j, const std::string& our_rid,
                     const std::string& our_boot, Envelope* out, Json* data) {
  if (!j.is_object()) return RtParse::kBadJson;

  // v first. 11 S3.0: an unknown version is REFUSED, never guessed at -- a
  // guess would decode a future schema's fields by position and act on them.
  const auto v_it = j.find("v");
  if (v_it == j.end() || !v_it->is_number_integer()) return RtParse::kMissingField;
  out->v = v_it->get<int>();
  if (out->v != kEnvelopeVersion) return RtParse::kBadVersion;

  if (!GetString(j, "rid", &out->rid)) return RtParse::kMissingField;
  if (out->rid != our_rid) return RtParse::kWrongRobot;
  if (!GetString(j, "src", &out->src)) return RtParse::kMissingField;

  const auto seq_it = j.find("seq");
  if (seq_it == j.end() || !seq_it->is_number_unsigned()) return RtParse::kMissingField;
  out->seq = seq_it->get<std::uint64_t>();

  if (!GetFinite(j, "ts", &out->ts)) return RtParse::kMissingField;

  // ts_sync: 11 S3.0 makes it mandatory AND makes absence mean false. Both
  // halves are kept -- a missing field is still a malformed envelope for a
  // loosening command, and the value it would have had is still false.
  const auto sync_it = j.find("ts_sync");
  out->ts_sync = (sync_it != j.end() && sync_it->is_boolean())
                     ? sync_it->get<bool>()
                     : false;

  // mono + boot travel together (11 S3.0: "mono 存在时 boot 必填"). A publisher
  // on another machine omits both (CLK-C4).
  double mono = -1.0;
  const bool has_mono = GetFinite(j, "mono", &mono);
  const bool has_boot = GetString(j, "boot", &out->boot);
  if (has_mono) {
    out->mono = mono;
    // *** The boot comparison. A monotonic reading from another boot counts
    // from THAT boot; subtracting ours gives an age wrong by however long the
    // two machines have been up. 11 S3.0 requires ignoring mono and falling
    // back to the receive time, which is what mono_usable == false means here.
    out->mono_usable = has_boot && !our_boot.empty() && out->boot == our_boot;
  } else {
    out->mono = -1.0;
    out->mono_usable = false;
  }

  const auto d_it = j.find("data");
  if (d_it == j.end() || !d_it->is_object()) return RtParse::kMissingField;
  *data = *d_it;

  if (sync_it == j.end() || !sync_it->is_boolean()) return RtParse::kMissingField;
  return RtParse::kOk;
}

}  // namespace

const char* RtParseName(RtParse r) {
  switch (r) {
    case RtParse::kOk: return "ok";
    case RtParse::kBadJson: return "bad_json";
    case RtParse::kBadVersion: return "bad_version";
    case RtParse::kWrongRobot: return "wrong_robot";
    case RtParse::kMissingField: return "missing_field";
    case RtParse::kNotFinite: return "not_finite";
    case RtParse::kBadValue: return "bad_value";
    case RtParse::kUnsupportedAction: return "unsupported_action";
  }
  return "invalid";
}

const char* CtrlActionName(CtrlAction a) {
  switch (a) {
    case CtrlAction::kStand: return "stand";
    case CtrlAction::kProne: return "prone";
    case CtrlAction::kEnable: return "enable";
    case CtrlAction::kSetSdkMode: return "set_sdk_mode";
  }
  return "invalid";
}

RtParse ParseCmdVel(const char* json, std::size_t len, const std::string& our_rid,
                    const std::string& our_boot, CmdVelMsg* out) {
  if (json == nullptr || out == nullptr) return RtParse::kBadJson;
  const Json j = Json::parse(json, json + len, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return RtParse::kBadJson;

  Json data;
  const RtParse env = ReadEnvelope(j, our_rid, our_boot, &out->env, &data);
  if (env != RtParse::kOk) return env;

  // All six axes are mandatory (11 S3.4). vz / v_roll / v_pitch are zero on
  // every ordinary gait and are still REQUIRED to be present: 13 V-67 records
  // that spec.* defines no limit for them, so Tier 1 trims them to zero, and a
  // message that omits them is a message from a publisher that does not know
  // the schema this one is being validated against.
  if (!GetFinite(data, "vx", &out->vx)) return RtParse::kMissingField;
  if (!GetFinite(data, "vy", &out->vy)) return RtParse::kMissingField;
  if (!GetFinite(data, "wz", &out->wz)) return RtParse::kMissingField;
  if (!GetFinite(data, "vz", &out->vz)) return RtParse::kMissingField;
  if (!GetFinite(data, "v_roll", &out->v_roll)) return RtParse::kMissingField;
  if (!GetFinite(data, "v_pitch", &out->v_pitch)) return RtParse::kMissingField;

  // *** estop_epoch. 11:1722 marks it MANDATORY on this key. Defaulting it --
  // to zero, or to "whatever we hold" -- is the fail-silent option twice over:
  // zero locks the robot at standstill after the first soft stop and never
  // releases, and echoing our own value makes the comparison in 13 S9.12.2 (3)
  // true by construction, so the stop hold silently stops working.
  const auto ep = data.find("estop_epoch");
  if (ep == data.end() || !ep->is_number_unsigned()) return RtParse::kMissingField;
  out->estop_epoch = ep->get<std::uint64_t>();

  return RtParse::kOk;
}

namespace {

// One row of a semantic-name table. 11 S9.2.4 keeps the wire semantic and
// hides the chassis numbers here, so this file is the single place a firmware
// enum change would land.
struct NameValue {
  const char* name;
  std::int64_t value;
};

bool LookupName(const NameValue* table, std::size_t n, const std::string& name,
                std::int64_t* out) {
  for (std::size_t i = 0; i < n; ++i) {
    if (name == table[i].name) {
      *out = table[i].value;
      return true;
    }
  }
  return false;
}

// 11 S9.2.4: normal / navigation / assist -> 0 / 1 / 2.
constexpr NameValue kUsageModes[] = {
    {"normal", 0}, {"navigation", 1}, {"assist", 2},
};

// 11 S9.2.4 COMMANDABLE motion states only. The read-only ones (soft_estop -2,
// idle 0, joint_damp 2, boot_damp 3, zero_cal 5, cart_move 16, damped_prone)
// are deliberately ABSENT rather than present-and-rejected: a table that
// listed them would invite the next reader to "just allow this one", and the
// contract's line is that they only ever come back, never go out.
constexpr NameValue kMotionStates[] = {
    {"stand", kCommandMotionStateStand},
    {"prone", kCommandMotionStateProne},
    {"rl_control", kCommandMotionStateRlControl},
};

// 11 S9.2.4 gaits. platform (0x1002) is read-only and absent for the same
// reason. stair_standard (0x1003) IS listed: it is write-only in the contract
// and 13 GS-1 refuses it for a DIFFERENT reason (the read-back enum has no
// 0x1003, so MS-2 would time out on every switch). That refusal belongs to
// mode_machine's command_forbidden_gaits, which is configured -- keeping it
// out of this table would hard-code a config decision into the parser and
// make the two disagree the day the firmware gains the read-back value.
constexpr NameValue kGaits[] = {
    {"basic", 0x1001}, {"stair_standard", 0x1003},
    {"flat", 0x3002}, {"stair_agile", 0x3003},
};

}  // namespace

bool UsageModeValue(const std::string& name, std::int64_t* out) {
  return LookupName(kUsageModes, sizeof(kUsageModes) / sizeof(kUsageModes[0]),
                    name, out);
}

bool MotionStateValue(const std::string& name, std::int64_t* out) {
  return LookupName(kMotionStates,
                    sizeof(kMotionStates) / sizeof(kMotionStates[0]), name, out);
}

bool GaitValue(const std::string& name, std::int64_t* out) {
  return LookupName(kGaits, sizeof(kGaits) / sizeof(kGaits[0]), name, out);
}

RtParse ParseHello(const char* json, std::size_t len,
                   const std::string& our_rid, const std::string& our_boot,
                   HelloMsg* out) {
  // rid / boot are accepted and unused: the handshake carries no envelope
  // (11 S9.1.4 shows the whole message, and it has none), so there is nothing
  // to check them against. They stay in the signature so every parser in this
  // file has the same shape and the subscription table needs no special case.
  (void)our_rid;
  (void)our_boot;
  if (json == nullptr || out == nullptr) return RtParse::kBadJson;
  const Json j = Json::parse(json, json + len, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return RtParse::kBadJson;
  std::string type;
  if (!GetString(j, "type", &type) || type != "hello") {
    return RtParse::kUnsupportedAction;
  }
  GetString(j, "client", &out->client);   // informational, not required
  std::string ver;
  if (!GetString(j, "proto_version", &ver)) return RtParse::kMissingField;
  // major.minor, split on the FIRST dot. A version that does not parse is a
  // refusal rather than a default: 11 S9.1.4 makes major the compatibility
  // decision, and defaulting it to 1 would make an unreadable version
  // compatible with us -- which is the one answer it must never produce.
  const std::size_t dot = ver.find('.');
  if (dot == std::string::npos || dot == 0 || dot + 1 >= ver.size()) {
    return RtParse::kMissingField;
  }
  try {
    out->proto_major = std::stoi(ver.substr(0, dot));
    out->proto_minor = std::stoi(ver.substr(dot + 1));
  } catch (const std::exception&) {
    return RtParse::kMissingField;
  }
  return RtParse::kOk;
}

RtParse ParseChassisMode(const char* json, std::size_t len,
                         const std::string& our_rid, const std::string& our_boot,
                         ChassisModeMsg* out) {
  if (json == nullptr || out == nullptr) return RtParse::kBadJson;
  const Json j = Json::parse(json, json + len, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return RtParse::kBadJson;
  Json data;
  const RtParse env = ReadEnvelope(j, our_rid, our_boot, &out->env, &data);
  if (env != RtParse::kOk) return env;
  // cmd_id is mandatory: this is a loosening command in the 11 S3.0.1 sense
  // (it can end at usage_mode = navigation, which is what Tier 1 waits for),
  // and without a cmd_id the switch cannot be correlated with its read-back.
  if (!GetString(data, "cmd_id", &out->cmd_id) || out->cmd_id.empty()) {
    return RtParse::kMissingField;
  }
  // Each field optional, each validated when present. A name outside the
  // commandable set refuses the WHOLE message: 13 MS-5 compares the triple as
  // a unit, so applying the fields that parsed would leave the machine waiting
  // on an expectation nobody can satisfy.
  std::string name;
  if (GetString(data, "usage_mode", &name)) {
    if (!UsageModeValue(name, &out->usage_mode)) {
      return RtParse::kUnsupportedAction;
    }
    out->has_usage_mode = true;
  }
  if (GetString(data, "motion_state", &name)) {
    if (!MotionStateValue(name, &out->motion_state)) {
      return RtParse::kUnsupportedAction;
    }
    out->has_motion_state = true;
  }
  if (GetString(data, "gait", &name)) {
    if (!GaitValue(name, &out->gait)) return RtParse::kUnsupportedAction;
    out->has_gait = true;
  }
  // A message that changes nothing is a defect in the sender, not a no-op to
  // absorb quietly: it means a field name was misspelled (the envelope parsed,
  // so the message LOOKED fine) and the switch the operator asked for never
  // happened. Refusing makes that visible at the source.
  if (!out->has_usage_mode && !out->has_motion_state && !out->has_gait) {
    return RtParse::kMissingField;
  }
  return RtParse::kOk;
}

RtParse ParseChassisCtrl(const char* json, std::size_t len,
                         const std::string& our_rid, const std::string& our_boot,
                         ChassisCtrlMsg* out) {
  if (json == nullptr || out == nullptr) return RtParse::kBadJson;
  const Json j = Json::parse(json, json + len, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return RtParse::kBadJson;

  Json data;
  const RtParse env = ReadEnvelope(j, our_rid, our_boot, &out->env, &data);
  if (env != RtParse::kOk) return env;

  // cmd_id is mandatory for a loosening command (11 S3.0.1 names it directly).
  // Without it an ack cannot be correlated and the idempotency rule of Q-3
  // ("same cmd_id re-sent => duplicate, epoch unchanged") has nothing to key on.
  if (!GetString(data, "cmd_id", &out->cmd_id) || out->cmd_id.empty()) {
    return RtParse::kMissingField;
  }
  if (!GetString(data, "action", &out->raw_action)) return RtParse::kMissingField;

  const std::string& a = out->raw_action;
  // The three the contract DELETED (11 S9.3.3 v0.3). They are named here rather
  // than falling into the unknown bucket because the answer differs: these get
  // E_CAPABILITY and an event, an unknown string gets E_SCHEMA. Collapsing them
  // would tell an operator "malformed" about a word that used to be valid.
  if (a == "soft_estop" || a == "estop_release" || a == "idle") {
    return RtParse::kUnsupportedAction;
  }

  if (a == "stand") {
    out->action = CtrlAction::kStand;
  } else if (a == "prone") {
    out->action = CtrlAction::kProne;
  } else if (a == "enable") {
    out->action = CtrlAction::kEnable;
  } else if (a == "set_sdk_mode") {
    out->action = CtrlAction::kSetSdkMode;
    const auto en = data.find("enable");
    if (en == data.end() || !en->is_boolean()) return RtParse::kMissingField;
    out->sdk_enable = en->get<bool>();
    const auto hz = data.find("joint_rate_hz");
    if (hz == data.end() || !hz->is_number_integer()) return RtParse::kMissingField;
    out->joint_rate_hz = hz->get<int>();
    // 11 S9.3.3: in [1,200] AND divides 1000. Both halves, because a rate that
    // does not divide 1000 produces a period the chassis cannot hold and the
    // symptom is jitter rather than a refusal.
    if (out->joint_rate_hz < 1 || out->joint_rate_hz > 200 ||
        (1000 % out->joint_rate_hz) != 0) {
      return RtParse::kBadValue;
    }
  } else {
    return RtParse::kBadValue;
  }

  return RtParse::kOk;
}

void ParseEstop(const char* json, std::size_t len, const std::string& our_rid,
                const std::string& our_boot, EstopMsg* out) {
  if (out == nullptr) return;
  *out = EstopMsg();
  if (json == nullptr) return;

  // Every early return below leaves the caller with envelope_ok == false and
  // NOTHING ELSE. There is no path out of this function that tells a caller to
  // skip the stop, because 11 S3.0.1 does not permit one to exist.
  const Json j = Json::parse(json, json + len, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return;

  Json env_data;
  const RtParse env = ReadEnvelope(j, our_rid, our_boot, &out->env, &env_data);
  // Deliberately not returned. Even a WRONG-ROBOT estop is acted on: 99 U75
  // accepts that a malformed or hostile payload can stop the robot, on the
  // stated ground that a wrongful stop costs nothing and a wrongful release
  // costs everything. The camp network is the customer's encrypted channel
  // (U23), which is the assumption that makes the trade acceptable.
  out->envelope_ok = (env == RtParse::kOk);

  // *** `data` is read from the DOCUMENT, not from ReadEnvelope's out-param.
  //
  // ReadEnvelope returns early on a bad version or a foreign rid and never
  // reaches the data object, which is correct for a loosening command and wrong
  // here: this estop is going to be acted on regardless, so the ack has to be
  // able to name the cmd_id it is acknowledging. Taking the envelope reader's
  // output made the cmd_id vanish on exactly the malformed messages this path
  // exists to survive -- found by the truncated / bad-version / foreign-rid
  // cases in test_rt_parse.cc, which is why they are there.
  const auto d_it = j.find("data");
  if (d_it == j.end() || !d_it->is_object()) return;
  out->cmd_id_present = GetString(*d_it, "cmd_id", &out->cmd_id);
}

}  // namespace rt
}  // namespace quadruped
