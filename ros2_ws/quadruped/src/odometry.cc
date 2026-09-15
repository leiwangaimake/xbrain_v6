/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: odometry.cc
 * Brief: Dead-reckoning implementation (see odometry.h)
 *
 * Description:
 * Three numbers decide everything here, and each is separate on purpose:
 *
 *   tau              the age of the linear velocity sample. The core quantity:
 *                    it drives the published variance, the band, and whether
 *                    anything is published at all.
 *   p_xx_committed_  the covariance already accounted for, grown ONCE per
 *                    velocity sample rather than once per tick. CV-2: within
 *                    one sample interval the error is fully correlated, so
 *                    accounting it ten times would over-count it -- and the
 *                    per-tick form 13 S4.4 used to carry did exactly that in
 *                    reverse, landing 2.45x OPTIMISTIC.
 *   p_yaw_           the angular covariance, on its own 200 Hz model and never
 *                    mixed with the linear one (ODO-4).
 *
 * The K_corr factor in the yaw recursion is 10, which is T_s / dt. It is a
 * correlation correction, not a unit conversion, and 13 S4.4 checks the result
 * numerically: 1.04e-5 rad^2/s, giving 0.18 degrees at one second and 3.20 at
 * three hundred. The test asserts those four numbers directly, so a change to
 * this recursion has to explain itself against the table.
 */

#include "quadruped/odometry.h"

#include <cmath>

namespace quadruped {

const char* OdomBandName(OdomBand b) {
  switch (b) {
    case OdomBand::kFresh: return "fresh";
    case OdomBand::kWarn: return "warn";
    case OdomBand::kTwistZero: return "twist_zero";
    case OdomBand::kStop: return "stop";
  }
  return "invalid";
}

Odometry::Odometry(const OdomConfig& cfg, bool holonomic)
    : cfg_(cfg), holonomic_(holonomic) {}

double Odometry::SigmaV(double tau_s) const {
  // 13 S4.4 (1). The a_max term is what makes a stale sample expensive: at one
  // sample period it already dominates sigma_v0, which is the quantitative
  // reason ODO-1 refuses to call this 100 Hz information.
  const double at = cfg_.a_max_mps2 * tau_s;
  return std::sqrt(cfg_.sigma_v0_mps * cfg_.sigma_v0_mps + at * at);
}

double Odometry::TrustDivisor() const {
  // 13 S4.4: the published covariance is P * (1 / trust_by_gait[gait]).
  // trust_flat is 1.0 and trust_stair 0.3, so a stair gait inflates by 3.33.
  const double t = is_stair_gait_ ? cfg_.trust_stair : cfg_.trust_flat;
  // A zero or negative trust factor would divide by zero and publish infinity.
  // Returning 1.0 would be worse than failing: it would silently publish an
  // UNINFLATED covariance on the gait that needs it most. The largest finite
  // inflation is the safe direction, so the value is floored rather than
  // defaulted.
  if (!(t > 0.0)) return 1.0e-6;
  return t;
}

void Odometry::OnVelocitySample(double now_mono_s, double vx, double vy) {
  if (has_velocity_ && last_vel_s_ >= 0.0) {
    // The interval that just closed. Normally T_s; larger when a frame was
    // dropped, and the growth is quadratic in it (CV-3) so a gap is
    // automatically conservative without a special case.
    const double tau_used = now_mono_s - last_vel_s_;
    if (tau_used > 0.0) {
      const double inc = SigmaV(tau_used) * tau_used;
      p_xx_committed_ += inc * inc;
    }
  }
  // 13 S4.4: a dead zone below which the reported velocity is treated as zero.
  // Without it a standing robot integrates its own measurement noise into a
  // slow drift across the floor.
  vx_ = (std::fabs(vx) < cfg_.vel_deadzone_mps) ? 0.0 : vx;
  vy_ = (std::fabs(vy) < cfg_.vel_deadzone_mps) ? 0.0 : vy;
  last_vel_s_ = now_mono_s;
  has_velocity_ = true;
}

void Odometry::OnYawRate(double wz) {
  // 13 S4.6: the V5 measurement found 0.01 rad/s to be the best dead zone, and
  // that a LARGER one is worse (it took the closed-loop error from 7.75 to 16.4
  // degrees per turn). The number is configured, not chosen here.
  wz_ = (std::fabs(wz) < cfg_.gyro_deadzone_radps) ? 0.0 : wz;
}

void Odometry::OnGait(bool is_stair_gait) { is_stair_gait_ = is_stair_gait; }

OdomSample Odometry::Tick(double now_mono_s, double dt_s) {
  OdomSample s;
  const double tau = (has_velocity_ && last_vel_s_ >= 0.0)
                         ? (now_mono_s - last_vel_s_)
                         : 1.0e9;  // never sampled: unboundedly stale
  s.tau_s = tau;

  // 13 S4.4 (4), the four bands. Evaluated worst-first so a long gap does not
  // spend a tick in a milder band on its way past the next threshold.
  const double tau_ms = tau * 1000.0;
  if (tau_ms > cfg_.stale_stop_publish_ms) {
    s.band = OdomBand::kStop;
  } else if (tau_ms > cfg_.stale_invalid_ms) {
    s.band = OdomBand::kTwistZero;
  } else if (tau_ms > cfg_.stale_warn_ms) {
    s.band = OdomBand::kWarn;
  } else {
    s.band = OdomBand::kFresh;
  }

  // yaw integrates regardless of the linear band: its source is the 200 Hz IMU
  // and its staleness is a different question (ODO-4). Stopping it because the
  // LINEAR sample is old would throw away the one estimate that is still good.
  if (s.band != OdomBand::kStop) {
    yaw_ += wz_ * dt_s;
    p_yaw_ += (cfg_.arw_rad_sqrt_s * cfg_.arw_rad_sqrt_s * dt_s) +
              (cfg_.gyro_bias_radps * dt_s) * (cfg_.gyro_bias_radps * dt_s) *
                  kYawCorrelationFactor;
  }

  // The pose integrates only while the velocity is trustworthy. Past 300 ms the
  // twist is zeroed and the pose is FROZEN rather than integrated on a held
  // value -- holding a velocity for a third of a second and calling the result
  // a position is the extrapolation ODO-3 forbids, arrived at by patience.
  if (s.band == OdomBand::kFresh || s.band == OdomBand::kWarn) {
    const double c = std::cos(yaw_);
    const double sn = std::sin(yaw_);
    x_ += (c * vx_ - sn * vy_) * dt_s;
    y_ += (sn * vx_ + c * vy_) * dt_s;
    s.vx = vx_;
    s.vy = holonomic_ ? vy_ : 0.0;
  } else {
    s.vx = 0.0;
    s.vy = 0.0;
  }
  s.wz = wz_;
  s.x = x_;
  s.y = y_;
  s.yaw = yaw_;

  // The published variance: what has been committed, plus the closed-form
  // growth for the part of the current interval that has elapsed (CV-1/CV-4).
  // Readable mid-interval and monotonic in tau, which is what lets Nav2 and RNS
  // see the cost of a late sample rather than a step every tenth of a second.
  const double sv = SigmaV(tau);
  const double open = sv * tau;
  const double divisor = TrustDivisor();
  s.var_x = (p_xx_committed_ + open * open) / divisor;
  s.var_y = holonomic_ ? s.var_x : -1.0;
  s.var_yaw = p_yaw_ / divisor;
  s.var_vx = (sv * sv) / divisor;
  s.var_vy = holonomic_ ? s.var_vx : -1.0;
  // The yaw rate comes from the 200 Hz source and does NOT grow with tau: tau
  // is the linear sample's age and has nothing to do with the gyro.
  s.var_wz = (cfg_.gyro_bias_radps * cfg_.gyro_bias_radps) / divisor;

  s.publish = (s.band != OdomBand::kStop);
  // 11 S9.9 / 13 S4.4 (4): invalid when stale, and invalid on a stair gait even
  // while publishing -- the inflation alone is not enough there, so 13 takes
  // both the inflation and the flag.
  s.valid = s.publish && !is_stair_gait_ &&
            (s.band == OdomBand::kFresh || s.band == OdomBand::kWarn);
  return s;
}

}  // namespace quadruped
