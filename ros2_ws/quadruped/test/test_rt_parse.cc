/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rt_parse.cc
 * Brief: The loosening/tightening asymmetry of 11 S3.0.1, case by case
 *
 * Description:
 * The cases here are mostly NEGATIVE, and that is the shape the rule has: a
 * loosening command has one way to be accepted and many ways to be refused,
 * and every one of those ways is a thing a publisher will eventually do.
 *
 * Two cases carry more weight than the rest:
 *
 *   * "p1_motion's ACTUAL body is refused". The body is transcribed from
 *     xbrain/p1_motion/runtime/nav_wiring.py as it stands, and it is missing
 *     the estop_epoch that 11:1722 marks mandatory on this key. The assertion
 *     is that we refuse it. That is not a wish -- it is what the contract
 *     requires, and it is the reason the RNS path will not move the robot until
 *     p1 is fixed. If someone "fixes" this test by making the parser lenient,
 *     they have removed the soft-stop hold rather than fixed anything.
 *   * "a truncated estop still yields a stop". There is no assertion that
 *     ParseEstop SUCCEEDED, because success is not what matters: the function
 *     returns void so that no caller can branch on it, and what is asserted is
 *     that it survives the malformed input and still reports what it could.
 */

#include "quadruped/rt_parse.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

// For GaitValueByName: the gait parity case below asserts the mode parse
// resolves through the read-back unit's table (13 QD-3), so the expectation
// is read from that table rather than restated as numbers here.
#include "quadruped/chs_a_reports.h"

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

const char* kRid = "gj-001";
const char* kBoot = "a1b2c3d4";

// A conformant envelope wrapped around a body. Written as text rather than
// built with a JSON library on purpose: the thing under test is a decoder, and
// generating its input with the same abstraction would hide exactly the
// spelling mistakes it exists to catch.
std::string Wrap(const std::string& body, const std::string& extra = "") {
  return std::string("{\"v\":1,\"rid\":\"") + kRid +
         "\",\"ts\":1789455340.125,\"mono\":812.5,\"boot\":\"" + kBoot +
         "\",\"seq\":7,\"src\":\"p1_motion\",\"ts_sync\":true" + extra +
         ",\"data\":" + body + "}";
}

// Every axis present and finite, plus the mandatory generation.
std::string GoodCmdBody(const char* epoch = "0") {
  return std::string("{\"vx\":0.3,\"vy\":0.0,\"wz\":0.2,\"vz\":0.0,"
                     "\"v_roll\":0.0,\"v_pitch\":0.0,\"estop_epoch\":") +
         epoch + "}";
}

RtParse Cmd(const std::string& text, CmdVelMsg* m) {
  return ParseCmdVel(text.c_str(), text.size(), kRid, kBoot, m);
}

}  // namespace

int main() {
  // ---- the happy path, so the negatives below mean something --------------
  {
    CmdVelMsg m;
    CHECK(Cmd(Wrap(GoodCmdBody("4")), &m) == RtParse::kOk);
    CHECK(m.vx == 0.3);
    CHECK(m.wz == 0.2);
    CHECK(m.estop_epoch == 4);
    CHECK(m.env.seq == 7);
    CHECK(m.env.ts_sync == true);
    CHECK(m.env.mono == 812.5);
    CHECK(m.env.mono_usable == true);   // same boot id
    CHECK(m.env.src == "p1_motion");
  }

  // ---- *** p1_motion's ACTUAL body today: REFUSED ------------------------
  //
  // Transcribed from nav_wiring.py _publish_cmd_vel. 11:1722 requires
  // estop_epoch on this key; this body does not carry it, so a contract-correct
  // parser refuses it and Tier 1 never sees a command. That is the live gap
  // between p1 and quadruped, and it is asserted here so it cannot be closed by
  // accident on this side.
  {
    const std::string p1_body =
        "{\"vx\":0.3,\"vy\":0.0,\"wz\":0.2,\"vz\":0.0,\"v_roll\":0.0,"
        "\"v_pitch\":0.0,\"gate\":{\"v_max\":2.0,\"profile\":\"patrol\","
        "\"limiter\":\"none\",\"limiter_all\":[],\"h_factor\":1.0,"
        "\"i_factor\":1.0,\"raw_vx\":0.3,\"source\":\"rns_avoid\"}}";
    CmdVelMsg m;
    CHECK(Cmd(Wrap(p1_body), &m) == RtParse::kMissingField);
  }

  // ---- each mandatory axis, one at a time --------------------------------
  {
    const char* omit[] = {"vx", "vy", "wz", "vz", "v_roll", "v_pitch"};
    for (const char* name : omit) {
      // Build the good body with one field renamed, so the JSON stays valid and
      // only the NAME the parser looks for is gone.
      std::string body = GoodCmdBody();
      const std::string needle = std::string("\"") + name + "\":";
      const std::size_t at = body.find(needle);
      CHECK(at != std::string::npos);
      body.replace(at, needle.size(), std::string("\"zz_") + name + "\":");
      CmdVelMsg m;
      CHECK(Cmd(Wrap(body), &m) == RtParse::kMissingField);
    }
  }

  // ---- a non-finite axis is refused, one layer EARLIER than expected -----
  //
  // 11 S9.12.1 calls this 载荷污染: a message with a NaN in a field nobody reads
  // is not a message whose other fields can be trusted. What matters for safety
  // is that it is REFUSED, and it is -- but not where this test first looked.
  //
  // Measured 2026-09-17: JSON has no NaN literal, and nlohmann DISCARDS a whole
  // document containing an overflowing one (1e999), so the refusal is kBadJson
  // rather than a per-axis kMissingField. The assertion follows the measurement
  // instead of the expectation; asserting the tidier value would have been an
  // assertion about a code path no input can reach.
  {
    CmdVelMsg m;
    const std::string body =
        "{\"vx\":0.3,\"vy\":0.0,\"wz\":0.2,\"vz\":0.0,"
        "\"v_roll\":1e999,\"v_pitch\":0.0,\"estop_epoch\":0}";
    CHECK(Cmd(Wrap(body), &m) == RtParse::kBadJson);
    // A finite extreme is NOT refused -- the rule is about finiteness, not size.
    const std::string big =
        "{\"vx\":1e308,\"vy\":0.0,\"wz\":0.2,\"vz\":0.0,"
        "\"v_roll\":0.0,\"v_pitch\":0.0,\"estop_epoch\":0}";
    CHECK(Cmd(Wrap(big), &m) == RtParse::kOk);   // Tier 1 is what clamps it
  }

  // ---- the envelope rules -------------------------------------------------
  {
    CmdVelMsg m;
    // An unknown version is refused, not guessed at (11 S3.0).
    std::string t = Wrap(GoodCmdBody());
    t.replace(t.find("\"v\":1"), 5, "\"v\":2");
    CHECK(Cmd(t, &m) == RtParse::kBadVersion);

    // Another robot's message.
    t = Wrap(GoodCmdBody());
    t.replace(t.find(kRid), std::string(kRid).size(), "gj-999");
    CHECK(Cmd(t, &m) == RtParse::kWrongRobot);

    // ts_sync is mandatory AND absence means false, so a message without it is
    // refused for a loosening command -- both halves of the rule, not one.
    t = Wrap(GoodCmdBody());
    t.replace(t.find(",\"ts_sync\":true"), 15, "");
    CHECK(Cmd(t, &m) == RtParse::kMissingField);

    // ...and when it IS present and false, it is carried as false.
    t = Wrap(GoodCmdBody());
    t.replace(t.find("\"ts_sync\":true"), 14, "\"ts_sync\":false");
    CHECK(Cmd(t, &m) == RtParse::kOk);
    CHECK(m.env.ts_sync == false);
  }

  // ---- *** boot mismatch: mono must be IGNORED ---------------------------
  //
  // Another host's monotonic clock counts from ITS boot. Comparing it with ours
  // yields an age wrong by however long the two machines have been up -- a
  // number so large the command reads as ancient, or so negative it reads as
  // arriving from the future. 11 S3.0 requires falling back to the receive time.
  {
    CmdVelMsg m;
    std::string t = Wrap(GoodCmdBody());
    t.replace(t.find(kBoot), std::string(kBoot).size(), "deadbeef");
    CHECK(Cmd(t, &m) == RtParse::kOk);      // the message is still valid
    CHECK(m.env.mono == 812.5);             // the value is still reported
    CHECK(m.env.mono_usable == false);      // ...and must not be compared
  }

  // ---- a cross-host publisher omits mono entirely (CLK-C4) ---------------
  {
    CmdVelMsg m;
    std::string t = Wrap(GoodCmdBody());
    t.replace(t.find(",\"mono\":812.5"), 13, "");
    CHECK(Cmd(t, &m) == RtParse::kOk);
    CHECK(m.env.mono < 0.0);
    CHECK(m.env.mono_usable == false);
  }

  // ---- not JSON at all ----------------------------------------------------
  {
    CmdVelMsg m;
    const std::string trunc = Wrap(GoodCmdBody()).substr(0, 40);
    CHECK(Cmd(trunc, &m) == RtParse::kBadJson);
    const std::string empty = "";
    CHECK(Cmd(empty, &m) == RtParse::kBadJson);
  }

  // ---- rt/clock/status: the one consumed field, refused both ways --------
  {
    // 13 Q-5 / CLK-A2: sync is the single system-wide verdict and the ONLY
    // field this process reads. Mandatory boolean -- a missing or mistyped
    // one is a refusal, never a default: defaulting true is CLK-A3's exact
    // failure, defaulting false silently discards a valid report.
    ClockStatusMsg m;
    CHECK(ParseClockStatus(Wrap("{\"sync\":true}").c_str(),
                           Wrap("{\"sync\":true}").size(), kRid, kBoot,
                           &m) == RtParse::kOk);
    CHECK(m.sync == true);
    ClockStatusMsg f;
    CHECK(ParseClockStatus(Wrap("{\"sync\":false}").c_str(),
                           Wrap("{\"sync\":false}").size(), kRid, kBoot,
                           &f) == RtParse::kOk);
    CHECK(f.sync == false);
    ClockStatusMsg bad;
    const std::string none = Wrap("{\"source\":\"rtk\"}");
    CHECK(ParseClockStatus(none.c_str(), none.size(), kRid, kBoot, &bad) ==
          RtParse::kMissingField);
    const std::string typed = Wrap("{\"sync\":1}");
    CHECK(ParseClockStatus(typed.c_str(), typed.size(), kRid, kBoot, &bad) ==
          RtParse::kMissingField);
    // The envelope rules hold here as everywhere: another robot's report is
    // not our clock verdict.
    std::string other = Wrap("{\"sync\":true}");
    const std::size_t at = other.find(kRid);
    other = other.substr(0, at) + "gj-002" + other.substr(at + std::string(kRid).size());
    CHECK(ParseClockStatus(other.c_str(), other.size(), kRid, kBoot, &bad) ==
          RtParse::kWrongRobot);
  }

  // ---- rt/chassis/ctrl: the closed set -----------------------------------
  {
    ChassisCtrlMsg c;
    auto ctrl = [&c](const std::string& body) {
      const std::string t = Wrap(body);
      return ParseChassisCtrl(t.c_str(), t.size(), kRid, kBoot, &c);
    };

    CHECK(ctrl("{\"cmd_id\":\"c-04\",\"action\":\"enable\"}") == RtParse::kOk);
    CHECK(c.action == CtrlAction::kEnable);
    CHECK(c.cmd_id == "c-04");
    CHECK(ctrl("{\"cmd_id\":\"c-01\",\"action\":\"stand\"}") == RtParse::kOk);
    CHECK(c.action == CtrlAction::kStand);
    CHECK(ctrl("{\"cmd_id\":\"c-02\",\"action\":\"prone\"}") == RtParse::kOk);
    CHECK(c.action == CtrlAction::kProne);

    // cmd_id is mandatory for a loosening command: without it an ack cannot be
    // correlated and Q-3's idempotency rule has nothing to key on.
    CHECK(ctrl("{\"action\":\"enable\"}") == RtParse::kMissingField);
    CHECK(ctrl("{\"cmd_id\":\"\",\"action\":\"enable\"}") == RtParse::kMissingField);

    // *** The three the contract DELETED answer E_CAPABILITY, not E_SCHEMA.
    // Telling an operator "malformed" about a word that used to be valid sends
    // them looking for a typo instead of at the release notes (11 S9.3.3 v0.3).
    for (const char* gone : {"soft_estop", "estop_release", "idle"}) {
      const std::string body =
          std::string("{\"cmd_id\":\"c-9\",\"action\":\"") + gone + "\"}";
      CHECK(ctrl(body) == RtParse::kUnsupportedAction);
      CHECK(c.raw_action == gone);   // the ack must be able to name it
    }

    // A word that was never valid.
    CHECK(ctrl("{\"cmd_id\":\"c-9\",\"action\":\"fly\"}") == RtParse::kBadValue);

    // set_sdk_mode: in [1,200] AND divides 1000. A rate that does not divide
    // 1000 gives a period the chassis cannot hold, and the symptom is jitter
    // rather than a refusal -- so it is refused here.
    CHECK(ctrl("{\"cmd_id\":\"c-5\",\"action\":\"set_sdk_mode\","
               "\"enable\":true,\"joint_rate_hz\":100}") == RtParse::kOk);
    CHECK(c.sdk_enable == true);
    CHECK(c.joint_rate_hz == 100);
    CHECK(ctrl("{\"cmd_id\":\"c-5\",\"action\":\"set_sdk_mode\","
               "\"enable\":true,\"joint_rate_hz\":300}") == RtParse::kBadValue);
    CHECK(ctrl("{\"cmd_id\":\"c-5\",\"action\":\"set_sdk_mode\","
               "\"enable\":true,\"joint_rate_hz\":7}") == RtParse::kBadValue);
    CHECK(ctrl("{\"cmd_id\":\"c-5\",\"action\":\"set_sdk_mode\","
               "\"enable\":true,\"joint_rate_hz\":0}") == RtParse::kBadValue);
    CHECK(ctrl("{\"cmd_id\":\"c-5\",\"action\":\"set_sdk_mode\","
               "\"joint_rate_hz\":100}") == RtParse::kMissingField);
  }

  // ---- rt/safety/estop: every malformed reading still stops --------------
  //
  // What is asserted is that the function SURVIVES each of these and reports
  // what it could. There is no verdict to assert, because the signature returns
  // none -- that is the enforcement, and this block is what proves the inputs
  // do not crash or hang on the way through.
  {
    EstopMsg e;
    const std::string good = Wrap("{\"cmd_id\":\"e-1\",\"action\":\"stop\"}");
    ParseEstop(good.c_str(), good.size(), kRid, kBoot, &e);
    CHECK(e.envelope_ok == true);
    CHECK(e.cmd_id_present == true);
    CHECK(e.cmd_id == "e-1");
    // The audit pair is absent here, and absent means EMPTY -- never a
    // refusal (nothing on this key may gate the stop) and never a filler
    // value (RobotState.last_soft_estop publishes empty as null, and a
    // fabricated role would put a name on an anonymous stop).
    CHECK(e.reason.empty());
    CHECK(e.src_role.empty());

    // With the pair present (11 S9.12), both are carried through -- this is
    // the only source RobotState.last_soft_estop has for them.
    EstopMsg tagged;
    const std::string full = Wrap(
        "{\"cmd_id\":\"e-2\",\"action\":\"stop\","
        "\"reason\":\"operator_hmi\",\"src_role\":\"hmi\"}");
    ParseEstop(full.c_str(), full.size(), kRid, kBoot, &tagged);
    CHECK(tagged.reason == "operator_hmi");
    CHECK(tagged.src_role == "hmi");
    // Mistyped values follow the same best-effort rule as everything else on
    // this key: ignored, empty, and the stop is unaffected.
    EstopMsg mistyped;
    const std::string odd = Wrap(
        "{\"cmd_id\":\"e-3\",\"action\":\"stop\",\"reason\":7,"
        "\"src_role\":[\"hmi\"]}");
    ParseEstop(odd.c_str(), odd.size(), kRid, kBoot, &mistyped);
    CHECK(mistyped.cmd_id_present == true);
    CHECK(mistyped.reason.empty());
    CHECK(mistyped.src_role.empty());

    // Truncated mid-object.
    const std::string trunc = good.substr(0, good.size() / 2);
    ParseEstop(trunc.c_str(), trunc.size(), kRid, kBoot, &e);
    CHECK(e.envelope_ok == false);
    CHECK(e.cmd_id_present == false);

    // An unknown version. A loosening command would be refused here; this one
    // is not, because there is nothing to refuse it WITH.
    std::string badv = good;
    badv.replace(badv.find("\"v\":1"), 5, "\"v\":9");
    ParseEstop(badv.c_str(), badv.size(), kRid, kBoot, &e);
    CHECK(e.envelope_ok == false);
    CHECK(e.cmd_id_present == true);      // ...and the ack can still name it
    CHECK(e.cmd_id == "e-1");

    // Another robot's id. Still parsed, still acted on: 99 U75 accepts that a
    // malformed or hostile payload can stop this robot, because a wrongful stop
    // costs nothing and a wrongful release costs everything.
    std::string other = good;
    other.replace(other.find(kRid), std::string(kRid).size(), "gj-999");
    ParseEstop(other.c_str(), other.size(), kRid, kBoot, &e);
    CHECK(e.envelope_ok == false);
    CHECK(e.cmd_id_present == true);

    // *** A missing ts_sync survives INTO the struct here, and must be false.
    //
    // On a loosening command this default is unobservable: the field is
    // mandatory, so the message is refused before anyone reads the value. The
    // estop path is the one place it matters, because this path refuses
    // nothing -- and 11 S3.0 is explicit that absence means false. Defaulting
    // to true would let a publisher that never had a synchronised clock be
    // believed by saying nothing, and the latency figures computed from it
    // would look valid.
    std::string nosync = good;
    nosync.replace(nosync.find(",\"ts_sync\":true"), 15, "");
    ParseEstop(nosync.c_str(), nosync.size(), kRid, kBoot, &e);
    CHECK(e.env.ts_sync == false);
    CHECK(e.cmd_id_present == true);     // ...and it is still acted on

    // Empty and null inputs must not crash.
    ParseEstop("", 0, kRid, kBoot, &e);
    CHECK(e.envelope_ok == false);
    ParseEstop(nullptr, 0, kRid, kBoot, &e);
    CHECK(e.envelope_ok == false);
  }

  // ---- rt/chassis/mode: gait names resolve through the ONE table ----------
  {
    // 13 QD-3 (2026-09-26 merge): the parse direction owns no gait NAME table
    // any more -- GaitValue delegates to chs_a_reports' GaitValueByName, the
    // same table ResolveGait and the config loader read. The assertion is
    // PARITY on the commandable members: whatever the read-back unit
    // resolves, the mode parse resolves to the same number, so the two
    // directions cannot drift by a member again (they had -- the parse copy
    // lacked platform).
    for (const char* name : {"basic", "stair_standard", "flat",
                             "stair_agile"}) {
      std::int64_t expect = 0;
      CHECK(chs_a::GaitValueByName(name, &expect) == true);
      ChassisModeMsg m;
      const std::string body = Wrap(
          std::string("{\"cmd_id\":\"g-1\",\"gait\":\"") + name + "\"}");
      CHECK(ParseChassisMode(body.c_str(), body.size(), kRid, kBoot, &m)
            == RtParse::kOk);
      CHECK(m.has_gait == true);
      CHECK(m.gait == expect);
    }

    // *** platform is the DIRECTIONAL exception: the read-back unit resolves
    // it (it arrives in reports), and the command direction still refuses it
    // -- 13 S5.3 G-03, the guide's command enumeration has no 0x1002 at all,
    // and T-COV-1's COV-4 holds this line against the matrix. This is the
    // case that separates "one name table" (QD-3, wanted) from "one
    // permission set" (wrong: read-only members would become commandable).
    {
      std::int64_t v = 0;
      CHECK(chs_a::GaitValueByName("platform", &v) == true);   // read-back: yes
      CHECK(v == 0x1002);
      ChassisModeMsg m;
      const std::string body =
          Wrap("{\"cmd_id\":\"g-p\",\"gait\":\"platform\"}");
      CHECK(ParseChassisMode(body.c_str(), body.size(), kRid, kBoot, &m)
            == RtParse::kUnsupportedAction);                   // command: no
    }

    // A name outside the table still refuses the whole message (13 MS-5
    // compares the triple as a unit).
    ChassisModeMsg bad;
    const std::string unknown =
        Wrap("{\"cmd_id\":\"g-2\",\"gait\":\"trot\"}");
    CHECK(ParseChassisMode(unknown.c_str(), unknown.size(), kRid, kBoot, &bad)
          == RtParse::kUnsupportedAction);

    // And the motion_state direction stays the commandable SUBSET -- the
    // read-only members are refusals here even though the read-back unit
    // resolves them (this is the asymmetry the header documents; a merge of
    // THAT table would be a contract violation, not a cleanup).
    for (const char* ro : {"idle", "joint_damp", "cart_move", "soft_estop"}) {
      ChassisModeMsg m;
      const std::string body = Wrap(
          std::string("{\"cmd_id\":\"g-3\",\"motion_state\":\"") + ro + "\"}");
      CHECK(ParseChassisMode(body.c_str(), body.size(), kRid, kBoot, &m)
            == RtParse::kUnsupportedAction);
    }
  }

  // ---- hello: no envelope, and major decides compatibility (11 S9.1.4) ----
  {
    // The handshake carries NO envelope. It is sent before the two sides have
    // agreed on anything, so requiring rid/seq/ts would make the handshake
    // depend on the agreement it exists to establish.
    HelloMsg m;
    const char* ok = "{\"type\":\"hello\",\"proto_version\":\"1.0\","
                     "\"client\":\"p1_motion\"}";
    CHECK(ParseHello(ok, std::strlen(ok), "dev", "boot", &m) == RtParse::kOk);
    CHECK(m.proto_major == 1);
    CHECK(m.proto_minor == 0);
    CHECK(m.client == "p1_motion");

    // major.minor split. 11 S9.1.4: major differing means incompatible.
    HelloMsg m2;
    const char* v2 = "{\"type\":\"hello\",\"proto_version\":\"2.3\"}";
    CHECK(ParseHello(v2, std::strlen(v2), "dev", "boot", &m2) == RtParse::kOk);
    CHECK(m2.proto_major == 2);
    CHECK(m2.proto_minor == 3);

    // A version that is not major.minor REFUSES rather than defaulting.
    // mutant: default major to 1 on a parse failure -> an unreadable version
    // becomes compatible with us, which is the one answer it must never give.
    for (const char* bad : {
             "{\"type\":\"hello\",\"proto_version\":\"1\"}",
             "{\"type\":\"hello\",\"proto_version\":\"\"}",
             "{\"type\":\"hello\",\"proto_version\":\".5\"}",
             "{\"type\":\"hello\",\"proto_version\":\"x.y\"}",
             "{\"type\":\"hello\"}"}) {
      HelloMsg bm;
      CHECK(ParseHello(bad, std::strlen(bad), "dev", "boot", &bm)
            != RtParse::kOk);
    }

    // Wrong type on the hello key is refused, not silently accepted.
    HelloMsg m3;
    const char* wrong = "{\"type\":\"goodbye\",\"proto_version\":\"1.0\"}";
    CHECK(ParseHello(wrong, std::strlen(wrong), "dev", "boot", &m3)
          == RtParse::kUnsupportedAction);
  }

  // ---- ParseLight (11 S9.4.1 / 13 V-47) ----------------------------------
  {
    // A well-formed custom light command. The names map to the vendor's
    // integers (guide 1.2.7) and BOTH lamps are mandatory -- the chassis takes
    // them as a positional two-element array, so a message naming only one has
    // no representation on the wire.
    const std::string ok = Wrap(
        "{\"cmd_id\":\"l-01\",\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"blink\",\"color\":\"white\",\"cycle_s\":1},"
        "\"tail\":{\"pattern\":\"breath\",\"color\":\"green\",\"cycle_s\":2}}}");
    LightMsg m;
    CHECK(ParseLight(ok.data(), ok.size(), kRid, kBoot, &m) == RtParse::kOk);
    CHECK(m.cmd_id == "l-01");
    CHECK(m.has_custom);
    CHECK(m.custom_enable);
    CHECK(!m.has_illumination);
    CHECK(m.head_pattern == 5);        // blink
    CHECK(m.head_color == 1);          // white
    CHECK(m.head_cycle_s == 1);
    CHECK(m.tail_pattern == 4);        // breath
    CHECK(m.tail_color == 2);          // green
    CHECK(m.tail_cycle_s == 2);
    // head and tail must NOT be the same values: a parser that filled both
    // from one object would pass every check above if they matched.
    CHECK(m.head_pattern != m.tail_pattern);
    CHECK(m.head_color != m.tail_color);
  }
  {
    // *** RED. 11 S9.4.1 states the chassis lamps have no red and puts the
    // deterrent flash on our own payload (PAY-02). A request for it is
    // REFUSED, never mapped onto white -- a warning that does not warn is
    // worse than a refused command.
    const std::string red = Wrap(
        "{\"cmd_id\":\"l-02\",\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"blink\",\"color\":\"red\",\"cycle_s\":1},"
        "\"tail\":{\"pattern\":\"solid\",\"color\":\"black\",\"cycle_s\":0}}}");
    LightMsg m;
    CHECK(ParseLight(red.data(), red.size(), kRid, kBoot, &m) ==
          RtParse::kUnsupportedAction);
  }
  {
    // A pattern outside the six. Same rule, same outcome.
    const std::string bad = Wrap(
        "{\"cmd_id\":\"l-03\",\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"strobe\",\"color\":\"white\",\"cycle_s\":1},"
        "\"tail\":{\"pattern\":\"solid\",\"color\":\"black\",\"cycle_s\":0}}}");
    LightMsg m;
    CHECK(ParseLight(bad.data(), bad.size(), kRid, kBoot, &m) ==
          RtParse::kUnsupportedAction);
  }
  {
    // Only one lamp named. Refused: the wire form is positional over two.
    const std::string half = Wrap(
        "{\"cmd_id\":\"l-04\",\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"solid\",\"color\":\"white\",\"cycle_s\":0}}}");
    LightMsg m;
    CHECK(ParseLight(half.data(), half.size(), kRid, kBoot, &m) ==
          RtParse::kUnsupportedAction);
  }
  {
    // 13 V-47: illumination is REPORTED, not dropped. The caller has to refuse
    // it, and it cannot refuse what the parser silently discarded.
    const std::string illum = Wrap(
        "{\"cmd_id\":\"l-05\",\"illumination\":{\"front\":1,\"back\":0}}");
    LightMsg m;
    CHECK(ParseLight(illum.data(), illum.size(), kRid, kBoot, &m) ==
          RtParse::kOk);
    CHECK(m.has_illumination);
    CHECK(!m.has_custom);
  }
  {
    // Neither half. Refused rather than treated as a no-op: an empty light
    // command is a schema mistake at the sender, and "accepted" would hide it.
    const std::string empty = Wrap("{\"cmd_id\":\"l-06\"}");
    LightMsg m;
    CHECK(ParseLight(empty.data(), empty.size(), kRid, kBoot, &m) ==
          RtParse::kMissingField);
  }
  {
    // LOOSENING (11 S3.0.1): no envelope, no command. The estop path parses
    // after it stops; this one must not copy that shape.
    const std::string bare =
        "{\"cmd_id\":\"l-07\",\"custom\":{\"enable\":false,"
        "\"head\":{\"pattern\":\"solid\",\"color\":\"black\",\"cycle_s\":0},"
        "\"tail\":{\"pattern\":\"solid\",\"color\":\"black\",\"cycle_s\":0}}}";
    LightMsg m;
    CHECK(ParseLight(bare.data(), bare.size(), kRid, kBoot, &m) !=
          RtParse::kOk);
  }
  {
    // *** The case that actually pins the envelope CHECK rather than the
    // envelope's side effects. ReadEnvelope fills `data` before it validates
    // ts_sync, so a body that parses fine reaches the rest of the function
    // even when the envelope was rejected -- an implementation that ignored
    // ReadEnvelope's return value passes every other envelope case here and
    // fails only this one. Found by a surviving mutant, not by reading.
    //
    // 11 S3.0 makes ts_sync mandatory; its absence is a malformed envelope for
    // a loosening command.
    const std::string no_sync =
        std::string("{\"v\":1,\"rid\":\"") + kRid +
        "\",\"ts\":1789455340.125,\"mono\":812.5,\"boot\":\"" + kBoot +
        "\",\"seq\":7,\"src\":\"p1_motion\",\"data\":"
        "{\"cmd_id\":\"l-08\",\"custom\":{\"enable\":true,"
        "\"head\":{\"pattern\":\"solid\",\"color\":\"white\",\"cycle_s\":0},"
        "\"tail\":{\"pattern\":\"solid\",\"color\":\"white\",\"cycle_s\":0}}}}";
    LightMsg m;
    CHECK(ParseLight(no_sync.data(), no_sync.size(), kRid, kBoot, &m) ==
          RtParse::kMissingField);
    CHECK(!m.has_custom);
  }

  // ---- 11 S8.5: the probe ping's seq lives in data, not the envelope ------
  //
  // The whole point of this parser is that the two numbers are DIFFERENT on
  // the deployed bus: chassis_relay rewrites the envelope seq (RT-C3.e) on
  // both legs of the probe, so only data.seq gets from p5_gateway to here
  // intact. Every case below therefore uses an envelope seq (7, from Wrap)
  // that is not the data seq -- a parser reading the wrong one cannot pass by
  // accident.
  {
    // The shape the relay actually delivers: envelope seq 7, data.seq 12345.
    const std::string ping = Wrap("{\"type\":\"ping\",\"seq\":12345}");
    ProbePingMsg m;
    CHECK(ParseProbePing(ping.data(), ping.size(), kRid, kBoot, &m) ==
          RtParse::kOk);
    CHECK(m.has_seq);
    CHECK(m.seq == 12345u);
    // Pinned so a "read the envelope seq" implementation is red on the VALUE
    // and not only on the envelope field being present.
    CHECK(m.env.seq == 7u);
    CHECK(m.seq != m.env.seq);
  }
  {
    // No data.seq at all. kMissingField like every other absent body field,
    // and has_seq false -- the caller must be able to tell "absent" from
    // "present and zero", because 13 F-15 makes it answer either way.
    const std::string ping = Wrap("{\"type\":\"ping\"}");
    ProbePingMsg m;
    CHECK(ParseProbePing(ping.data(), ping.size(), kRid, kBoot, &m) ==
          RtParse::kMissingField);
    CHECK(!m.has_seq);
    CHECK(m.seq == 0u);
  }
  {
    // A NEGATIVE seq. This is the case an is_number() check would let through:
    // get<uint64_t>() on -5 wraps to 18446744073709551611 without throwing, so
    // the pong would carry a number nobody sent and it would look like a
    // perfectly ordinary counter on the wire.
    const std::string ping = Wrap("{\"type\":\"ping\",\"seq\":-5}");
    ProbePingMsg m;
    CHECK(ParseProbePing(ping.data(), ping.size(), kRid, kBoot, &m) ==
          RtParse::kMissingField);
    CHECK(!m.has_seq);
  }
  {
    // A string seq. Same refusal; separate case because it fails a different
    // half of the guard (is_number_unsigned is false for a different reason).
    const std::string ping = Wrap("{\"type\":\"ping\",\"seq\":\"12345\"}");
    ProbePingMsg m;
    CHECK(ParseProbePing(ping.data(), ping.size(), kRid, kBoot, &m) ==
          RtParse::kMissingField);
    CHECK(!m.has_seq);
  }
  {
    // *** A ping addressed to ANOTHER robot. The envelope verdict must win and
    // data.seq must stay unread: echoing it would answer someone else's probe
    // with this robot's estop state, and the operator on the other end would
    // read our hes/stop_reason as theirs. This is the case that pins "envelope
    // first" rather than "grab the seq wherever it is".
    const std::string ping =
        std::string("{\"v\":1,\"rid\":\"other\",\"ts\":1789455340.125,"
                    "\"mono\":812.5,\"boot\":\"") + kBoot +
        "\",\"seq\":7,\"src\":\"chassis_relay\",\"ts_sync\":true,"
        "\"data\":{\"type\":\"ping\",\"seq\":12345}}";
    ProbePingMsg m;
    CHECK(ParseProbePing(ping.data(), ping.size(), kRid, kBoot, &m) ==
          RtParse::kWrongRobot);
    CHECK(!m.has_seq);
    CHECK(m.seq == 0u);
  }
  {
    // Not JSON at all. Still no crash, still no seq.
    ProbePingMsg m;
    CHECK(ParseProbePing("not json at all", 15, kRid, kBoot, &m) ==
          RtParse::kBadJson);
    CHECK(!m.has_seq);
  }

  if (g_failures == 0) {
    std::printf("ALL RT PARSE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT PARSE TEST(S) FAILED\n", g_failures);
  return 1;
}
