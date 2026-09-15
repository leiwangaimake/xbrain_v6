/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_quadruped_config.cc
 * Brief: Offline test for LoadQuadrupedConfig -- fail-stop on null (3.1)
 *
 * Description:
 * The fixture is written in the shape the freeze materialiser emits
 * (yaml.safe_dump: dash at the parent key column, entry map two columns
 * further), not in a hand-pretty shape, because the reader has to survive the
 * bytes it will actually be given.
 *
 * Positive case first, then one mutant per contract (CLAUDE.md 3.3):
 *   * a null limit THROWS -- this is today's real state (common.spec.max_* are
 *     null pending V-01) and the single most important behaviour in the file:
 *     a loader that substituted 0.0 would clamp every command to zero and the
 *     robot would stand still with no error, which is the fail-silent 3.1
 *     exists to prevent;
 *   * a missing key THROWS with its path;
 *   * out-of-order odom thresholds THROW -- they select four bands and an
 *     inverted pair silently makes one unreachable;
 *   * zero control_loop_hz THROWS -- it becomes a sleep duration on a FIFO
 *     thread, i.e. a busy loop on an isolated core, which produces no log;
 *   * all-disabled endpoints THROW -- the probe would contact nothing and sit
 *     at conn=lost with a config that reads as complete.
 *
 * The endpoint COUNT is asserted before any field, because the realistic parser
 * defect merges consecutive entries and every field assertion would still pass
 * on the merged one.
 */

#include "quadruped/quadruped_config.h"

#include <cstdio>
#include <fstream>
#include <string>

using quadruped::ConfigError;
using quadruped::LoadQuadrupedConfig;
using quadruped::QuadrupedConfig;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// Minimal but COMPLETE resolved snapshot: every key the loader requires, in
// materialiser shape. Kept as one string so a test can copy it and break
// exactly one thing.
const char* kGood =
    "quadruped:\n"
    "  robot_id: xb-001\n"
    "  chassis_link:\n"
    "    asdu_format: json\n"
    "    axis_cmd_hz: 20.0\n"
    "    axis_cmd_socket_fixed: true\n"
    "    cmd_fail_threshold: 3\n"
    "    codebook: hex32\n"
    "    codebook_table:\n"
    "      legacy_decimal: {}\n"
    "    endpoint_candidates:\n"
    "    - enabled: true\n"
    "      host: 10.21.33.103\n"
    "      port: 30003\n"
    "      proto: tcp\n"
    "      tls: false\n"
    "    - enabled: false\n"
    "      host: 10.21.33.103\n"
    "      port: 30004\n"
    "      proto: udp\n"
    "      tls: true\n"
    "    frame_assembly_timeout_ms: 500\n"
    "    heartbeat_hz: 2.0\n"
    "    partial_send_retry: 3\n"
    "    probe_timeout_ms: 2000\n"
    "    proto_version_byte: 1\n"
    "    reconnect_backoff_s:\n"
    "    - 0.5\n"
    "    - 1.0\n"
    "    - 5.0\n"
    "    resync_max_bytes: 4096\n"
    "    single_tx_owner: true\n"
    "    state_timeout_degraded_s: 1.0\n"
    "    state_timeout_lost_s: 3.0\n"
    "    tcp_nodelay: true\n"
    "  chassis_dds:\n"
    "    backend: cyclone_raw\n"
    "    domain_id: 0\n"
    "    forward_imu_to_rt: false\n"
    "    imu_age_warn_ms: 50\n"
    "    imu_expect_hz: 200.0\n"
    "    imu_frame_id: imu_link\n"
    "    imu_rt_key: ''\n"
    "    imu_topic: /IMU\n"
    "  odom:\n"
    "    a_max_mps2: 2.5\n"
    "    arw_rad_sqrt_s: 0.002\n"
    "    gyro_bias_radps: 0.008\n"
    "    gyro_deadzone_radps: 0.01\n"
    "    publish_hz: 100.0\n"
    "    sigma_v0_mps: 0.05\n"
    "    stale_invalid_ms: 300\n"
    "    stale_stop_publish_ms: 1000\n"
    "    stale_warn_ms: 150\n"
    "    trust_by_gait:\n"
    "      flat: 1.0\n"
    "      stair: 0.3\n"
    "    vel_deadzone_mps: 0.05\n"
    "  tier1:\n"
    "    cmd_timeout_ms: 200\n"
    "    control_loop_hz: 100.0\n"
    "    estop_ack_ms: 100\n"
    "    estop_dedup_ms: 50\n"
    "    limits:\n"
    "      holonomic: true\n"
    "      max_accel_mps2: 2.5\n"
    "      max_decel_mps2: 2.5\n"
    "      max_vx_mps: 2.0\n"
    "      max_vy_mps: 1.0\n"
    "      max_wz_radps: 1.0\n"
    "    safety_probe_stale_s: 3.0\n"
    "  uplink:\n"
    "    base_frame: base_link\n"
    "    odom_frame: odom\n"
    "    odom_topic: /odom_quadruped\n"
    "    publish_odom_tf: true\n"
    "    rmw: rmw_cyclonedds_cpp\n"
    "    ros_domain_id: 42\n"
    "    zenoh_rt_endpoint: tcp/127.0.0.1:7449\n";

std::string g_dir;

// Write text to a temp file and return the path. The loader takes a path
// because the real caller reads the freeze product from disk; loading from a
// string would test a different function than the one that ships.
std::string WriteTemp(const std::string& name, const std::string& text) {
  const std::string path = g_dir + "/" + name;
  std::ofstream f(path, std::ios::binary);
  f << text;
  f.close();
  return path;
}

// Replace the first occurrence of `from` with `to`; aborts the case loudly if
// the anchor is gone, so a fixture edit cannot silently turn a mutant into a
// copy of the healthy config (which would pass and prove nothing).
std::string Mutate(const std::string& from, const std::string& to) {
  std::string s = kGood;
  const std::size_t at = s.find(from);
  if (at == std::string::npos) {
    std::printf("FAIL mutation anchor missing: %s\n", from.c_str());
    ++g_failures;
    return s;
  }
  return s.replace(at, from.size(), to);
}

template <class F>
bool Throws(F f) {
  try {
    f();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  // A writable directory for the fixtures; ctest passes the build dir.
  g_dir = (argc >= 2) ? argv[1] : ".";

  // ---- positive: the full snapshot loads and every field lands ----------
  {
    const std::string p = WriteTemp("q_good.yaml", kGood);
    const QuadrupedConfig c = LoadQuadrupedConfig(p);
    CHECK(c.robot_id == "xb-001");
    // Count before fields: a merged-entry parser bug would pass every field
    // assertion below on the single surviving entry.
    CHECK(c.link.endpoints.size() == 2);
    CHECK(c.link.endpoints[0].proto == "tcp");
    CHECK(c.link.endpoints[0].port == 30003);
    CHECK(c.link.endpoints[0].enabled == true);
    CHECK(c.link.endpoints[0].tls == false);
    CHECK(c.link.endpoints[1].port == 30004);
    CHECK(c.link.endpoints[1].tls == true);
    CHECK(c.link.endpoints[1].enabled == false);
    CHECK(c.link.codebook == "hex32");
    CHECK(c.link.partial_send_retry == 3);
    // The ladder arrives with its VALUES, not just its length. Asserting only
    // size() cannot tell a correctly read list from one whose entries all came
    // back as zero, and a zero rung is the failure this list guards against.
    CHECK(c.link.reconnect_backoff_s.size() == 3);
    CHECK(c.link.reconnect_backoff_s[0] == 0.5);
    CHECK(c.link.reconnect_backoff_s[2] == 5.0);
    CHECK(c.link.axis_cmd_socket_fixed == true);
    CHECK(c.link.single_tx_owner == true);
    CHECK(c.link.proto_version_byte == 1);
    CHECK(c.link.asdu_format == "json");
    CHECK(c.link.legacy_decimal_entries == 0);
    CHECK(c.dds.domain_id == 0);
    CHECK(c.uplink.ros_domain_id == 42);
    // The two domains must not be the same number; QC-4 owns the assertion at
    // freeze time, this is the value actually reaching the process.
    CHECK(c.dds.domain_id != c.uplink.ros_domain_id);
    CHECK(c.dds.imu_rt_key.empty());  // legal while forward_imu_to_rt is false
    CHECK(c.odom.a_max_mps2 == 2.5);
    CHECK(c.odom.trust_flat == 1.0);
    CHECK(c.odom.trust_stair == 0.3);
    CHECK(c.tier1.cmd_timeout_ms == 200);
    CHECK(c.tier1.limits.max_vx_mps == 2.0);
    CHECK(c.tier1.limits.holonomic == true);
    // The self report must name both domains: that is the whole point of
    // DDS-9, and a report that omitted one would look complete.
    const std::string d = quadruped::DescribeConfig(c);
    CHECK(d.find("chassis_dds.domain_id=0") != std::string::npos);
    CHECK(d.find("uplink.ros_domain_id=42") != std::string::npos);
    CHECK(d.find("endpoint[1]=udp://10.21.33.103:30004") != std::string::npos);
  }

  // ---- 3.1: a null limit stops the process, it does not become 0.0 -------
  // This is today's real snapshot state (common.spec.max_* pending V-01).
  {
    const std::string p = WriteTemp(
        "q_null.yaml", Mutate("      max_vx_mps: 2.0\n", "      max_vx_mps: null\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }

  // ---- a missing key stops the process -----------------------------------
  {
    const std::string p =
        WriteTemp("q_missing.yaml", Mutate("    cmd_timeout_ms: 200\n", ""));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }

  // ---- process-side invariants -------------------------------------------
  {
    // warn >= invalid: band 2 becomes unreachable.
    const std::string p = WriteTemp(
        "q_bands.yaml", Mutate("    stale_warn_ms: 150\n", "    stale_warn_ms: 400\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    const std::string p = WriteTemp(
        "q_hz.yaml", Mutate("    control_loop_hz: 100.0\n", "    control_loop_hz: 0.0\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // A zero rung in the reconnect ladder: the reconnect becomes a busy loop
    // against a chassis that is already in trouble, and 13 CA-6 means every
    // attempt plays a voice prompt and switches the LEDs.
    const std::string p = WriteTemp(
        "q_backoff0.yaml", Mutate("    - 0.5\n", "    - 0.0\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // A negative rung, which a formula-derived ladder could produce and which
    // "> 0" catches but "!= 0" would not.
    const std::string p = WriteTemp(
        "q_backoffneg.yaml", Mutate("    - 1.0\n", "    - -1.0\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // An empty ladder. The list is still PRESENT, so a loader that only
    // required the key would accept it and leave the session with no delay.
    const std::string p = WriteTemp(
        "q_backoffnone.yaml",
        Mutate("    reconnect_backoff_s:\n    - 0.5\n    - 1.0\n    - 5.0\n",
               "    reconnect_backoff_s: []\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // 13 CA-1: a per-command socket reads to the chassis as a new client and
    // axis commands come back 0xE006 -- accepted, and the robot does not move.
    const std::string p = WriteTemp(
        "q_sockfix.yaml",
        Mutate("    axis_cmd_socket_fixed: true\n",
               "    axis_cmd_socket_fixed: false\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // 13 CA-4 / QC-16: two writers on one TCP socket interleave two frames.
    const std::string p = WriteTemp(
        "q_txowner.yaml",
        Mutate("    single_tx_owner: true\n", "    single_tx_owner: false\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // The config and the codec must agree on the header version byte. This is
    // the case that makes the check worth having: 2 is a perfectly valid
    // integer, so nothing but the comparison rejects it.
    const std::string p = WriteTemp(
        "q_ver.yaml",
        Mutate("    proto_version_byte: 1\n", "    proto_version_byte: 2\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // XML is a documented ASDU format (format byte 0x00) with no encoder here.
    const std::string p = WriteTemp(
        "q_fmt.yaml", Mutate("    asdu_format: json\n", "    asdu_format: xml\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // CB-1: the only codebook the code implements.
    const std::string p = WriteTemp(
        "q_cb.yaml", Mutate("    codebook: hex32\n", "    codebook: legacy_decimal\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // QC-13: all-or-nothing. Four of the five codes is the dangerous shape --
    // selectable, and failing on the first command it does not cover.
    const std::string p = WriteTemp(
        "q_legacy4.yaml",
        Mutate("      legacy_decimal: {}\n",
               "      legacy_decimal:\n"
               "        axis: 4\n"
               "        gait: 3\n"
               "        heartbeat: 1\n"
               "        motion_state: 2\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }
  {
    // ...and the complete table is accepted (still with codebook hex32, which
    // CB-3 describes as the one-switch-to-flip state). Without this case the
    // rule above could be "any non-empty table is refused", which is a
    // different and wrong rule.
    const std::string p = WriteTemp(
        "q_legacy5.yaml",
        Mutate("      legacy_decimal: {}\n",
               "      legacy_decimal:\n"
               "        axis: 5\n"
               "        gait: 4\n"
               "        heartbeat: 1\n"
               "        motion_state: 3\n"
               "        usage_mode: 2\n"));
    const QuadrupedConfig c = LoadQuadrupedConfig(p);
    CHECK(c.link.legacy_decimal_entries == 5);
  }
  {
    // Every candidate disabled: the probe would contact nothing.
    const std::string p = WriteTemp(
        "q_noep.yaml", Mutate("    - enabled: true\n", "    - enabled: false\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }

  // ---- an unexpanded reference is loud, not a silent zero ----------------
  // If the freeze line did not run, the file still carries "${common.spec...}";
  // require_double must reject it rather than parse a leading digit.
  {
    const std::string p = WriteTemp(
        "q_ref.yaml",
        Mutate("      max_vx_mps: 2.0\n",
               "      max_vx_mps: ${common.spec.max_vx_mps}\n"));
    CHECK(Throws([&] { LoadQuadrupedConfig(p); }));
  }

  // ---- a missing file is an error, never an empty config ------------------
  CHECK(Throws([&] { LoadQuadrupedConfig(g_dir + "/does_not_exist.yaml"); }));

  if (g_failures == 0) {
    std::printf("ALL QUADRUPED_CONFIG TESTS PASSED\n");
    return 0;
  }
  std::printf("%d QUADRUPED_CONFIG TEST(S) FAILED\n", g_failures);
  return 1;
}
