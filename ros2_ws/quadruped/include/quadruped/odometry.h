/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: odometry.h
 * Brief: Dead-reckoning core -- ODO-1..5, the closed-form covariance, CV-1..5
 *
 * Description:
 * Integrates a 10 Hz linear velocity and a 200 Hz yaw rate into a pose at the
 * 100 Hz publish tick, and reports how little it knows while doing it.
 *
 * The single most important thing in this file is that it does NOT pretend the
 * result is 100 Hz information. 13 ODO-1: a hundred publishes a second carrying
 * a velocity that updates ten times a second is still ten times a second of
 * information, and the difference is carried in the covariance. ODO-2 names
 * repeating a 10 Hz sample ten times and calling it 100 Hz as the sixth
 * instance of the project's recurring failure mode. What is allowed is a
 * zero-order hold with the uncertainty grown honestly, and those two halves are
 * inseparable: the hold without the growth is the thing ODO-2 forbids.
 *
 * ODO-3 forbids extrapolating the velocity. v + a*tau looks more accurate and
 * is not: the acceleration is unmeasured (M-22 has no number), so extrapolating
 * would be inventing data and presenting it as a reading.
 *
 * The covariance recursion is worth reading against 13 S4.4 directly, because
 * an earlier version of that section carried a per-tick form that disagreed
 * with its own stated equivalent by a factor of 2.45 -- and in the optimistic
 * direction. The form here is the corrected one (CV-1..CV-5):
 *
 *     P_xx_pub = P_xx_committed + (sigma_v(tau) * tau)^2
 *
 * committed grows ONCE per velocity sample, by (sigma_v(tau_used)*tau_used)^2,
 * and tau then restarts. Two properties fall out of that shape and both are
 * required: the error within one sample interval is fully correlated, so it is
 * accounted once rather than ten times (CV-2); and a dropped frame makes it
 * conservative automatically, because tau_used grows and the term is quadratic
 * in it while sigma_v itself already contains a_max*tau (CV-3).
 *
 * Yaw is a separate model on a separate rate (ODO-4): 200 Hz in, accumulated
 * per tick, and its covariance never mixes with the linear one. A single
 * "odometry uncertainty" number would have to be the worse of the two, and the
 * worse one is the linear one by two orders of magnitude.
 *
 * Boundary: no clock, no ROS, no publishing. It computes; the caller decides
 * what to send and when, and the band this file reports is what that decision
 * is made on.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_ODOMETRY_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_ODOMETRY_H_

#include <cstdint>

#include "quadruped/quadruped_config.h"

namespace quadruped {

// 13 S4.4 (4): the four staleness bands, which are the ONLY authority on what
// may be published. Ordered by severity so a comparison is meaningful.
enum class OdomBand {
  kFresh = 0,      // tau <= 150 ms: publish normally
  kWarn = 1,       // 150 < tau <= 300 ms: publish, warn once per window
  kTwistZero = 2,  // 300 < tau <= 1000 ms: twist zeroed, pose frozen, TF kept
  kStop = 3,       // tau > 1000 ms: stop publishing odom AND TF
};

const char* OdomBandName(OdomBand b);

// 13 S4.4: K_corr = T_s / dt = 10, the correlation correction in the yaw
// recursion. It is NOT a unit conversion, and 13 checks the result numerically
// (1.04e-5 rad^2/s, giving 0.18 degrees at one second): a reader who mistakes
// it for one and "simplifies" it away lands a factor of ten optimistic on the
// one estimate that stays good when the linear source dies.
inline constexpr double kYawCorrelationFactor = 10.0;

// What one publish tick produced.
struct OdomSample {
  double x = 0.0;        // m, odom frame
  double y = 0.0;        // m
  double yaw = 0.0;      // rad
  double vx = 0.0;       // m/s, body frame -- zeroed in kTwistZero and beyond
  double vy = 0.0;       // m/s
  double wz = 0.0;       // rad/s

  // Variances, already scaled by the gait trust factor.
  double var_x = 0.0;    // m^2
  double var_y = 0.0;    // m^2   -1 when the chassis is not holonomic
  double var_yaw = 0.0;  // rad^2
  double var_vx = 0.0;   // (m/s)^2
  double var_vy = 0.0;   // (m/s)^2  -1 when not holonomic
  double var_wz = 0.0;   // (rad/s)^2

  OdomBand band = OdomBand::kFresh;
  // 13 S4.4 (4) / 11 S9.9: false in kStop, and ALSO false on a stair gait --
  // the wheel odometry is not usable there, and 11 requires either a large
  // inflation or an explicit invalidation. 13 takes both.
  bool valid = false;
  // Whether the caller should publish at all. False only in kStop, where 13
  // requires the TF to stop too: a frozen TF makes Nav2 believe the robot is
  // stationary and keep commanding rotation, which is the worst direction.
  bool publish = false;
  double tau_s = 0.0;    // the velocity sample's age, for detail.tau_ms
  // 13 S9.1 rt_pub row, verbatim: "时间戳取自样本, 不取自发布时刻 => 发布晚了
  // 是到得晚, 不是数据错". Stamped by ctrl at integration time and carried
  // across the lock-free slot, so a publish that loses its slot to the
  // scheduler ships a LATE message, not a message that misdescribes when the
  // pose was true. Reading a clock at publish time instead would silently
  // re-label every delayed sample as current -- and the delay is precisely
  // what V-69 traded away, so it must stay visible downstream.
  //
  // Wall clock, because it lands in a ROS header.stamp (CLK-C3 / the align
  // exemption). Every AGE and PERIOD in this process stays on steady_clock;
  // this field is a label, never an input to a decision.
  double stamp_wall_s = 0.0;
};

class Odometry {
 public:
  // Every parameter is injected. sigma_v0, a_max, the gyro bias and the ARW
  // coefficient are measurements (M-28 / M-29) and have no defaults here:
  // CLAUDE.md 3.1, and a default would be a number nobody measured appearing in
  // a covariance somebody trusts.
  Odometry(const OdomConfig& cfg, bool holonomic);

  // A new linear velocity sample from the chassis (10 Hz, or 20 Hz on drdds).
  // Commits the covariance for the interval that just closed and restarts tau.
  void OnVelocitySample(double now_mono_s, double vx, double vy);

  // A yaw rate from the IMU (200 Hz). Integrated at the publish tick rather
  // than here, so the integration step is the publish period and not the
  // jittery arrival interval.
  void OnYawRate(double wz);

  // The gait currently read back, so the trust factor can be applied and a
  // stair gait can invalidate the sample (11 S9.9).
  void OnGait(bool is_stair_gait);

  // One 100 Hz publish tick. dt is the publish period, passed in rather than
  // derived from successive now values: a tick that was late must not integrate
  // a longer step on a velocity it does not have (that would be extrapolation
  // by the back door, ODO-3).
  OdomSample Tick(double now_mono_s, double dt_s);

  // The committed part alone, for a test that wants to see the two halves.
  double committed_var_xy() const { return p_xx_committed_; }
  bool has_velocity() const { return has_velocity_; }

 private:
  double SigmaV(double tau_s) const;
  double TrustDivisor() const;

  OdomConfig cfg_;
  bool holonomic_;
  bool is_stair_gait_ = false;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double vx_ = 0.0;
  double vy_ = 0.0;
  double wz_ = 0.0;

  bool has_velocity_ = false;
  double last_vel_s_ = -1.0;

  double p_xx_committed_ = 0.0;
  double p_yaw_ = 0.0;
};

// A yaw-only orientation as a quaternion (x, y, z, w). Kept HERE, in the
// ROS-free core, rather than in the uplink: it is pure arithmetic, it is a
// classic place for a half-angle to go missing, and putting it here means it is
// tested on a machine with no ROS at all.
//
// The half angle is the whole content. Writing sin(yaw) instead of sin(yaw/2)
// produces a quaternion that is correct at 0 and at 180 degrees and wrong
// everywhere in between -- which is exactly the shape that survives a casual
// test and fails in the field.
struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

Quaternion YawToQuaternion(double yaw_rad);

// Where each variance lands in the 6x6 row-major covariance array that
// nav_msgs/Odometry carries. Named because the numbers are the bug: [35] is
// yaw, not [5], and an implementation that used the latter writes the yaw
// variance into the x-row's yaw CORRELATION and leaves the yaw variance at
// zero -- a planner then treats the heading as exact.
inline constexpr int kCovIndexX = 0;      // row 0, col 0
inline constexpr int kCovIndexY = 7;      // row 1, col 1
inline constexpr int kCovIndexZ = 14;     // row 2, col 2
inline constexpr int kCovIndexRoll = 21;  // row 3, col 3
inline constexpr int kCovIndexPitch = 28; // row 4, col 4
inline constexpr int kCovIndexYaw = 35;   // row 5, col 5

// Fill a 36-element row-major covariance from the three estimated diagonal
// terms. The three unestimated axes get -1, the REP-105 convention for "not
// provided" -- zero would claim perfect knowledge of something never measured.
void FillCovariance36(double var_x, double var_y, double var_yaw, double* out36);

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_ODOMETRY_H_
