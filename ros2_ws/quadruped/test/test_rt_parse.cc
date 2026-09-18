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

#include <cstdio>
#include <cstring>
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

  if (g_failures == 0) {
    std::printf("ALL RT PARSE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT PARSE TEST(S) FAILED\n", g_failures);
  return 1;
}
