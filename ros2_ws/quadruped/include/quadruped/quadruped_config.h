/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: quadruped_config.h
 * Brief: Load resolved quadruped.yaml into the structs the process needs (13 S8.2)
 *
 * Description:
 * What problem this solves. The quadruped process must not carry a single
 * default for any safety number: 13 S3.2 clamps every command with
 * spec.max_*, and a missing value that silently became 0.0 would produce a
 * robot that stands still and reports nothing wrong -- the fail-silent shape
 * CLAUDE.md 3.1 forbids. So every field here is pulled with a Require*
 * accessor that THROWS with its dotted key path, and there is no default
 * anywhere in this file.
 *
 * Which file is read. The RESOLVED snapshot
 * /opt/xbrain_v6/data/run/resolved/quadruped.yaml, never configs/quadruped.yaml
 * (10 S5.4.1: references are expanded once by the freeze line, not per
 * process). The caller passes the path so tests can point at a fixture.
 *
 * Why the limits live in the file at all. The freeze materialiser composes
 * {common: overlay.common, **per-proc} , expands ${common.*}, then DROPS the
 * common subtree. A value that is only mentioned in prose ("references
 * common.spec.max_decel_mps2") therefore never reaches this process. 13 S8.2
 * v1.4 added the reference keys for exactly that reason; this loader reads
 * them from tier1.limits and odom.
 *
 * Boundary: this maps FILE fields only. It does not open a socket, does not
 * touch DDS, does not validate cross-field invariants that assertion K already
 * evaluates at freeze time (QC-1..QC-17, 10 S5.4.4) -- re-checking them here
 * would be a second copy of a rule that is owned elsewhere. What it DOES
 * re-check is the handful of invariants the process itself depends on to be
 * memory-safe and honest: monotonic odom thresholds and a positive control
 * period, because those are read every tick.
 *
 * The trap worth naming. yaml_lite returns strings; a key present but null in
 * the resolved snapshot (uncalibrated L2, e.g. common.spec.max_vx_mps today)
 * arrives as the literal "null"/"~"/empty. Treating that as 0.0 is precisely
 * the fail-silent case, so IsNull() is checked BEFORE any numeric parse and
 * the key path is reported.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_QUADRUPED_CONFIG_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_QUADRUPED_CONFIG_H_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace quadruped {

// Thrown by every Require* accessor and by LoadQuadrupedConfig. Carries the
// dotted key path in what(), because "config invalid" without the key is a
// message that makes an operator grep the whole tree.
// * Why a dedicated type rather than std::runtime_error: main() must be able
//   to tell "your config is wrong" (exit EX_CONFIG) from "the file system
//   refused" without parsing the message text.
class ConfigError : public std::runtime_error {
 public:
  explicit ConfigError(const std::string& message)
      : std::runtime_error(message) {}
};

// One candidate endpoint of channel one (13 S2.2, QC-6 checks the port range).
// * enabled/tls are separate because TLS-1 keeps the encrypted candidates in
//   the file but disabled until the vendor issues a client certificate; the
//   deployment flips one boolean, not the code.
struct EndpointCandidate {
  std::string proto;   // "tcp" | "udp"
  std::string host;
  int port = 0;        // Always overwritten by the loader; QC-6 owns the range.
  bool tls = false;
  bool enabled = false;
};

// Tier 1 hard limits (13 S3.2 step 5). Every member is required and every one
// of them is a REFERENCE into common.spec.* in the source file -- see the
// header comment for why the reference must be written out.
// * Today common.spec.max_* are null (V-01 not measured), so loading a real
//   resolved snapshot THROWS here. That is the designed behaviour, not a bug:
//   a chassis that accepts unclamped commands must not be driven.
struct Tier1Limits {
  double max_vx_mps = 0.0;      // m/s  forward clamp
  double max_vy_mps = 0.0;      // m/s  lateral; only meaningful when holonomic
  double max_wz_radps = 0.0;    // rad/s yaw clamp
  double max_accel_mps2 = 0.0;  // m/s2 not a clamp input today; carried for hello_ack.spec
  double max_decel_mps2 = 0.0;  // m/s2 also the a_max of the odom covariance model
  // Whether vy is a real axis on this chassis. False makes vy a forbidden axis
  // rather than a small one: 11 S9.3.1 requires quadruped to ZERO an axis the
  // gait does not support instead of passing it through, so the caller can
  // tell "not supported" from "supported and commanded to zero".
  bool holonomic = false;
};

// Tier 1 timings (13 S8.2 tier1 block; bounds are QC-2 / QC-3 at freeze time).
struct Tier1Config {
  Tier1Limits limits;
  int cmd_timeout_ms = 0;       // ms   floor 200 (QC-2); upstream silence -> timeout_lock
  int estop_ack_ms = 0;         // ms   deadline for cmd/estop/ack (11 S9.12.6)
  int estop_dedup_ms = 0;       // ms   window that swallows a repeated stop
  double control_loop_hz = 0.0; // Hz   >= 100 (QC-3); becomes the tick period
  double safety_probe_stale_s = 0.0;  // s  no ping for this long -> E_SAFETY_LINK_LOST
};

// Channel one link parameters (13 S2.2). Framing and send discipline live
// here because they are read by the codec in B1 and by the single tx owner.
struct ChassisLinkConfig {
  // Probed in order; the first one that answers wins. Order is meaningful, so
  // this is a vector and never a set.
  std::vector<EndpointCandidate> endpoints;
  int probe_timeout_ms = 0;       // ms   per candidate, not for the whole list
  double heartbeat_hz = 0.0;      // Hz   >= 1 (QC-11); the chassis reports state
                                  //      ONLY to whoever keeps sending heartbeats
  std::string codebook;           // CB-1: "hex32" is the only legal value today
  int resync_max_bytes = 0;       // FR-2 bytes slid looking for the sync word
  int frame_assembly_timeout_ms = 0;  // FR-3 half-frame discard timer
  int partial_send_retry = 0;     // FR-4 bounded completion of a short write
  bool tcp_nodelay = false;       // FR-5 / SD-3: Nagle would make "when did the
                                  //      last frame leave" unanswerable
  double axis_cmd_hz = 0.0;       // Hz   same tick as P1 (20 Hz)
  // Consecutive send failures before conn=degraded. NOT ack failures: the
  // chassis never acks an axis command (13 CA-7), so counting missing acks
  // would report a healthy link as broken.
  int cmd_fail_threshold = 0;
  double state_timeout_degraded_s = 0.0;  // s  uplink gap -> degraded (11 S9.1.3)
  double state_timeout_lost_s = 0.0;      // s  ...then lost, then backoff reconnect
};

// Odometry model inputs (13 S4.4). a_max and trust_by_gait are references into
// common.spec.*; the sigma/bias values are per-process measurements (M-28/M-29)
// that live in this file.
struct OdomConfig {
  // PUBLISH rate, not information rate: the chassis reports linear velocity at
  // 10 Hz and 13 ODO-1 forbids describing the result as 100 Hz odometry. The
  // honesty lives in the covariance, which grows with the age of the sample.
  double publish_hz = 0.0;             // Hz
  double vel_deadzone_mps = 0.0;       // m/s  suppresses standstill drift
  double gyro_deadzone_radps = 0.0;    // rad/s V5 measured 0.01 as best; LARGER
                                       //      is worse (7.75 -> 16.4 deg/turn)
  double sigma_v0_mps = 0.0;           // m/s  velocity noise at sample instant
  double gyro_bias_radps = 0.0;        // rad/s residual bias, not raw bias
  double arw_rad_sqrt_s = 0.0;         // rad/sqrt(s) angle random walk
  double a_max_mps2 = 0.0;             // m/s2 sigma_v(tau) term; = spec.max_decel
  double trust_flat = 0.0;             // [0,1] covariance divisor by gait
  double trust_stair = 0.0;            // [0,1] 0.3 -> 3.33x inflation on stairs
  // Four bands (13 S4.4): publish | warn | invalidate | stop publishing. They
  // must strictly increase or one band is unreachable -- checked at load.
  int stale_warn_ms = 0;
  int stale_invalid_ms = 0;
  int stale_stop_publish_ms = 0;
};

// Domain isolation (13 S2.4 DDS-1/QC-4) plus the two topic names the process
// publishes on. Kept together because the self-report prints them as one block
// (DDS-9: the only cheap way to prove the two domains did not collapse).
struct UplinkConfig {
  std::string zenoh_rt_endpoint;  // RT plane only; the general plane is forbidden
                                  // to this process (11 RT-C4)
  int ros_domain_id = 0;          // MUST differ from chassis_dds.domain_id (QC-4)
  std::string rmw;                // rmw_cyclonedds_cpp on the Humble baseline
  std::string odom_topic;         // name owned by 13 S4.8; 11 registers no ROS names
  std::string odom_frame;         // "odom"
  std::string base_frame;         // "base_link"
  bool publish_odom_tf = false;   // quadruped is the SOLE publisher of this TF
};

struct ChassisDdsConfig {
  std::string backend;     // "cyclone_raw" | "fastdds_vendored" (the fallback if
                           // the cross-vendor RTPS test T-CHS-1 fails)
  int domain_id = 0;       // DDS-1: passed explicitly, never from the environment
  std::string imu_topic;   // /IMU (measured 201 Hz, frame_id empty)
  double imu_expect_hz = 0.0;
  int imu_age_warn_ms = 0; // ms  older than this -> yaw falls back to the 10 Hz source
  // The chassis sends no frame_id, so we assign one. That assignment must be
  // logged (13 TF-1): a downstream consumer otherwise cannot tell whether the
  // frame name came from the sensor or from us.
  std::string imu_frame_id;
  bool forward_imu_to_rt = false;  // D-44 default false; QC-14 ties it to the key
  std::string imu_rt_key;          // empty while the above is false, by design
};

// Everything the process needs from the file, in one value.
struct QuadrupedConfig {
  std::string robot_id;
  ChassisLinkConfig link;
  ChassisDdsConfig dds;
  UplinkConfig uplink;
  OdomConfig odom;
  Tier1Config tier1;
};

// Load and validate. Throws ConfigError on: unreadable file, missing key, null
// value, unparsable number, or a violated process-side invariant (see the
// header comment for which invariants are checked here and which are not).
// * path is the RESOLVED snapshot, not the source config.
QuadrupedConfig LoadQuadrupedConfig(const std::string& path);

// Default resolved-snapshot location (10 S5.4.1). Exposed as a function rather
// than a macro so a test can print it without the preprocessor.
const char* DefaultResolvedPath();

// Human-readable one-block self report used at startup and by --selfcheck.
// * DDS-9 and CB-4 both require the EFFECTIVE values to be printed: the two
//   domain ids, the endpoint that will be probed first, and the codebook in
//   force. A configuration mistake in either is otherwise indistinguishable
//   from a dead network.
std::string DescribeConfig(const QuadrupedConfig& cfg);

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_QUADRUPED_CONFIG_H_
