/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_odometry.cc
 * Brief: T-ODOM-2's four baseline rows, the four staleness bands, ODO-1..5
 *
 * Description:
 * The headline cases run the model for one, ten, sixty and three hundred
 * seconds and compare the growth against the numbers 13 S4.4 tabulates. Those
 * four rows are T-ODOM-2's acceptance baseline, so they are asserted as
 * numbers rather than as "grows monotonically" -- an implementation that grew
 * at any rate at all would satisfy the latter, and the rate is the whole point.
 *
 * That table is also why the recursion is worth checking against rather than
 * trusting: 13 S4.4 previously carried a per-tick form which disagreed with its
 * own stated equivalent by a factor of 2.45, in the OPTIMISTIC direction. A
 * covariance that is too small is a robot that believes it knows where it is.
 *
 * The other cases are the bands, and each one asserts the behaviour that makes
 * it different from its neighbour:
 *   * past 300 ms the twist is ZEROED and the pose FROZEN. Integrating a held
 *     velocity for a third of a second is the extrapolation ODO-3 forbids,
 *     arrived at by patience rather than by writing v + a*tau.
 *   * past 1000 ms the TF stops too. A frozen TF makes Nav2 believe the robot
 *     is stationary and keep commanding rotation, which 13 calls the worst
 *     direction of failure; stopping it makes Nav2 abort within its 200 ms
 *     transform tolerance instead.
 *   * yaw keeps integrating while the LINEAR sample is stale, because its
 *     source is a different sensor at a different rate (ODO-4). Stopping it
 *     would throw away the one estimate that is still good.
 */

#include "quadruped/odometry.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace quadruped;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// The values 13 S4.4 tabulates against, as configs/quadruped.yaml ships them.
OdomConfig Cfg() {
  OdomConfig c;
  c.publish_hz = 100.0;
  c.vel_deadzone_mps = 0.05;
  c.gyro_deadzone_radps = 0.01;
  c.sigma_v0_mps = 0.05;
  c.gyro_bias_radps = 0.008;
  c.arw_rad_sqrt_s = 0.002;
  c.a_max_mps2 = 2.5;
  c.trust_flat = 1.0;
  c.trust_stair = 0.3;
  c.stale_warn_ms = 150;
  c.stale_invalid_ms = 300;
  c.stale_stop_publish_ms = 1000;
  return c;
}

bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Run the model for `seconds` of healthy operation: a velocity sample every
// 100 ms and a 100 Hz publish tick. Returns the last sample.
OdomSample RunHealthy(Odometry* o, double seconds, double vx, double wz) {
  OdomSample last;
  const double dt = 0.01;
  const int ticks = static_cast<int>(seconds / dt + 0.5);
  o->OnVelocitySample(0.0, vx, 0.0);
  o->OnYawRate(wz);
  for (int i = 1; i <= ticks; ++i) {
    const double t = i * dt;
    // A sample every tenth tick, i.e. the 10 Hz the chassis actually reports.
    if (i % 10 == 0) o->OnVelocitySample(t, vx, 0.0);
    last = o->Tick(t, dt);
  }
  return last;
}

}  // namespace

int main() {
  // ---- T-ODOM-2: the four baseline rows of 13 S4.4 -----------------------
  {
    // Asserted as numbers because the rate IS the claim. "It grows" is
    // satisfied by any rate at all, including the 2.45x-optimistic one the
    // document used to carry.
    struct Row {
      double seconds;
      double sigma_x_m;     // 13 S4.4's table
      double sigma_yaw_deg;
      double tol_m;
      double tol_deg;
    };
    const Row rows[] = {
        {1.0, 0.081, 0.18, 0.002, 0.01},
        {10.0, 0.255, 0.58, 0.004, 0.02},
        {60.0, 0.62, 1.43, 0.01, 0.03},
        {300.0, 1.40, 3.20, 0.02, 0.05},
    };
    for (const Row& r : rows) {
      Odometry o(Cfg(), /*holonomic=*/true);
      const OdomSample s = RunHealthy(&o, r.seconds, 0.0, 0.0);
      const double sigma_x = std::sqrt(s.var_x);
      const double sigma_yaw_deg = std::sqrt(s.var_yaw) * 180.0 / M_PI;
      if (!Near(sigma_x, r.sigma_x_m, r.tol_m)) {
        std::printf("FAIL sigma_x at %gs: got %.4f, 13 S4.4 says %.3f\n",
                    r.seconds, sigma_x, r.sigma_x_m);
        ++g_failures;
      }
      if (!Near(sigma_yaw_deg, r.sigma_yaw_deg, r.tol_deg)) {
        std::printf("FAIL sigma_yaw at %gs: got %.4f deg, 13 S4.4 says %.2f\n",
                    r.seconds, sigma_yaw_deg, r.sigma_yaw_deg);
        ++g_failures;
      }
    }
  }

  // ---- sigma_v(tau): the quantitative reason ODO-1 exists ---------------
  {
    // 13 S4.4 (1) tabulates these. At one sample period the uncertainty is
    // 0.255 m/s, while the obstacle_avoid speed band's entire ceiling is
    // 0.5 m/s -- which is why repeating a 10 Hz sample and calling it 100 Hz
    // information is forbidden rather than merely discouraged.
    Odometry o(Cfg(), true);
    o.OnVelocitySample(0.0, 1.0, 0.0);
    struct P { double tau, sigma; };
    const P pts[] = {{0.0, 0.050}, {0.025, 0.080}, {0.050, 0.134}, {0.100, 0.255}};
    for (const P& p : pts) {
      const OdomSample s = o.Tick(p.tau, 0.01);
      const double sv = std::sqrt(s.var_vx);
      if (!Near(sv, p.sigma, 0.002)) {
        std::printf("FAIL sigma_v(%.3f): got %.4f, 13 S4.4 says %.3f\n", p.tau,
                    sv, p.sigma);
        ++g_failures;
      }
    }
  }

  // ---- ODO-4: the yaw-rate variance does NOT grow with the linear age ---
  {
    // tau is the LINEAR sample's age and has nothing to do with the gyro, which
    // reports at 200 Hz on its own path. A var_wz that inherited tau would make
    // a late chassis report look like a failing IMU, and the two have entirely
    // different remedies.
    Odometry o(Cfg(), true);
    o.OnVelocitySample(0.0, 1.0, 0.0);
    o.OnYawRate(0.5);
    const double at_zero = o.Tick(0.0, 0.01).var_wz;
    const double at_full = o.Tick(0.10, 0.01).var_wz;
    const double at_stale = o.Tick(0.50, 0.01).var_wz;
    // 13 S4.4 (2): the element is b_g^2 = 6.4e-5 (rad/s)^2, flat.
    CHECK(Near(at_zero, 6.4e-5, 1e-7));
    CHECK(at_full == at_zero);
    CHECK(at_stale == at_zero);
    // ...while the LINEAR one does grow over the same interval, so this is a
    // statement about the two being independent rather than about nothing
    // growing at all.
    Odometry o2(Cfg(), true);
    o2.OnVelocitySample(0.0, 1.0, 0.0);
    CHECK(o2.Tick(0.10, 0.01).var_vx > o2.Tick(0.0, 0.01).var_vx);
  }

  // ---- the published variance grows BETWEEN samples ---------------------
  {
    // CV-4. A downstream reader mid-interval must see the cost of the sample's
    // age, not a value that sits still for ten ticks and then steps. The step
    // form is what a per-tick commit produces, and it hides the moment when the
    // estimate is worst -- the end of the interval.
    Odometry o(Cfg(), true);
    o.OnVelocitySample(0.0, 1.0, 0.0);
    const double v0 = o.Tick(0.0, 0.01).var_x;
    const double v5 = o.Tick(0.05, 0.01).var_x;
    const double v10 = o.Tick(0.10, 0.01).var_x;
    CHECK(v0 < v5);
    CHECK(v5 < v10);
    // ...and the committed part does NOT move between samples: it is accounted
    // once per interval (CV-2), not once per tick.
    CHECK(o.committed_var_xy() == 0.0);
    o.OnVelocitySample(0.10, 1.0, 0.0);
    CHECK(o.committed_var_xy() > 0.0);
    // The closed interval contributes exactly (sigma_v(T_s)*T_s)^2.
    CHECK(Near(o.committed_var_xy(), 6.50e-4, 1e-5));
  }

  // ---- a dropped frame is conservative automatically --------------------
  {
    // CV-3: tau_used grows and the term is quadratic in it, while sigma_v
    // itself already contains a_max*tau. No special case, and no chance of
    // forgetting one.
    Odometry a(Cfg(), true);
    a.OnVelocitySample(0.0, 1.0, 0.0);
    a.OnVelocitySample(0.10, 1.0, 0.0);
    const double one_interval = a.committed_var_xy();

    Odometry b(Cfg(), true);
    b.OnVelocitySample(0.0, 1.0, 0.0);
    b.OnVelocitySample(0.30, 1.0, 0.0);   // two frames lost
    const double gap = b.committed_var_xy();
    // Not merely larger: much larger. Three times the interval gives roughly
    // (0.752*0.3)^2 / (0.255*0.1)^2 = 78x, so a linear model would be caught.
    CHECK(gap > one_interval * 20.0);
    CHECK(Near(gap, 5.09e-2, 5e-3));
  }

  // ---- the four bands ----------------------------------------------------
  {
    Odometry o(Cfg(), true);
    o.OnYawRate(0.5);
    o.OnVelocitySample(0.0, 1.0, 0.0);

    OdomSample s = o.Tick(0.10, 0.01);
    CHECK(s.band == OdomBand::kFresh);
    CHECK(s.publish && s.valid);
    CHECK(s.vx == 1.0);

    s = o.Tick(0.20, 0.01);
    CHECK(s.band == OdomBand::kWarn);
    // Still publishing and still valid: the warning is about the age, and the
    // covariance already carries the cost.
    CHECK(s.publish && s.valid);
    CHECK(s.vx == 1.0);

    const double x_before = s.x;
    s = o.Tick(0.50, 0.01);
    CHECK(s.band == OdomBand::kTwistZero);
    CHECK(s.publish);          // the TF is kept so the tree stays connected
    CHECK(s.valid == false);
    // *** The twist is ZEROED and the pose is FROZEN. Integrating a held
    // velocity for a third of a second is ODO-3's extrapolation reached by
    // patience rather than by arithmetic.
    CHECK(s.vx == 0.0);
    CHECK(s.x == x_before);
    // ...and yaw keeps integrating, because its source is a different sensor
    // at a different rate (ODO-4).
    CHECK(s.wz == 0.5);
    const double yaw_before = s.yaw;
    s = o.Tick(0.51, 0.01);
    CHECK(s.yaw > yaw_before);

    s = o.Tick(1.50, 0.01);
    CHECK(s.band == OdomBand::kStop);
    // *** Everything stops, TF included. A frozen TF makes Nav2 believe the
    // robot is stationary and keep commanding rotation; stopping it makes Nav2
    // abort inside its 200 ms transform tolerance, which is the fail-safe.
    CHECK(s.publish == false);
    CHECK(s.valid == false);
  }

  // ---- the band boundaries ----------------------------------------------
  {
    // Each threshold checked from both sides. A one-sided check passes on an
    // implementation whose comparison is off by one band.
    Odometry o(Cfg(), true);
    o.OnVelocitySample(0.0, 1.0, 0.0);
    CHECK(o.Tick(0.150, 0.01).band == OdomBand::kFresh);      // <= 150 ms
    CHECK(o.Tick(0.151, 0.01).band == OdomBand::kWarn);
    CHECK(o.Tick(0.300, 0.01).band == OdomBand::kWarn);       // <= 300 ms
    CHECK(o.Tick(0.301, 0.01).band == OdomBand::kTwistZero);
    CHECK(o.Tick(1.000, 0.01).band == OdomBand::kTwistZero);  // <= 1000 ms
    CHECK(o.Tick(1.001, 0.01).band == OdomBand::kStop);
  }

  // ---- before any velocity sample, nothing is published -----------------
  {
    // The age of a sample that never arrived is unbounded, so the stop band is
    // the honest answer -- not "fresh with zero velocity", which is what a
    // zero-initialised timestamp would produce and which reads as a robot
    // standing still.
    Odometry o(Cfg(), true);
    const OdomSample s = o.Tick(0.0, 0.01);
    CHECK(s.band == OdomBand::kStop);
    CHECK(s.publish == false);
    CHECK(s.valid == false);
  }

  // ---- a stair gait inflates AND invalidates ---------------------------
  {
    // 11 S9.9 asks for a large inflation or an explicit invalidation; 13 takes
    // both, because an inflated covariance is still a number a planner will use
    // if nothing tells it not to.
    Odometry flat(Cfg(), true);
    RunHealthy(&flat, 10.0, 0.0, 0.0);
    const OdomSample sf = flat.Tick(10.0, 0.01);

    Odometry stair(Cfg(), true);
    stair.OnGait(true);
    RunHealthy(&stair, 10.0, 0.0, 0.0);
    const OdomSample ss = stair.Tick(10.0, 0.01);

    // trust_stair 0.3 -> 1/0.3 = 3.33x on the variance.
    CHECK(Near(ss.var_x / sf.var_x, 1.0 / 0.3, 0.05));
    CHECK(Near(ss.var_yaw / sf.var_yaw, 1.0 / 0.3, 0.05));
    CHECK(sf.valid == true);
    CHECK(ss.valid == false);       // invalid even while publishing
    CHECK(ss.publish == true);
  }

  // ---- a non-holonomic chassis reports -1, not 0 -----------------------
  {
    // REP-105: a negative variance means "not provided". Reporting 0 would
    // claim perfect knowledge of a quantity that is not estimated at all.
    Odometry o(Cfg(), /*holonomic=*/false);
    o.OnVelocitySample(0.0, 1.0, 0.5);
    const OdomSample s = o.Tick(0.01, 0.01);
    CHECK(s.var_y == -1.0);
    CHECK(s.var_vy == -1.0);
    CHECK(s.vy == 0.0);
    CHECK(s.var_x > 0.0);
  }

  // ---- the dead zones ---------------------------------------------------
  {
    // Without them a standing robot integrates its own measurement noise into
    // a slow drift across the floor.
    Odometry o(Cfg(), true);
    o.OnVelocitySample(0.0, 0.04, 0.0);   // under 0.05
    o.OnYawRate(0.009);                   // under 0.01
    const OdomSample s = o.Tick(0.01, 0.01);
    CHECK(s.vx == 0.0);
    CHECK(s.wz == 0.0);
    CHECK(s.x == 0.0);
    CHECK(s.yaw == 0.0);
    // Just above, and they pass through: the zone suppresses noise, it does not
    // suppress motion.
    Odometry o2(Cfg(), true);
    o2.OnVelocitySample(0.0, 0.06, 0.0);
    o2.OnYawRate(0.02);
    const OdomSample s2 = o2.Tick(0.01, 0.01);
    CHECK(s2.vx == 0.06);
    CHECK(s2.wz == 0.02);
  }

  // ---- the pose integrates in the ODOM frame ---------------------------
  {
    // A body-frame velocity rotated by the current yaw. Driving a metre east
    // then turning ninety degrees and driving a metre must land at (1, 1),
    // which a missing rotation would turn into (2, 0).
    Odometry o(Cfg(), true);
    o.OnYawRate(0.0);
    for (int i = 1; i <= 100; ++i) {
      o.OnVelocitySample(i * 0.01, 1.0, 0.0);
      o.Tick(i * 0.01, 0.01);
    }
    OdomSample s = o.Tick(1.0, 0.01);
    CHECK(Near(s.x, 1.0, 0.02));
    CHECK(Near(s.y, 0.0, 1e-6));

    // Turn ninety degrees in place, then drive again.
    o.OnYawRate(M_PI / 2.0);
    for (int i = 1; i <= 100; ++i) {
      o.OnVelocitySample(1.0 + i * 0.01, 0.0, 0.0);
      o.Tick(1.0 + i * 0.01, 0.01);
    }
    o.OnYawRate(0.0);
    for (int i = 1; i <= 100; ++i) {
      o.OnVelocitySample(2.0 + i * 0.01, 1.0, 0.0);
      o.Tick(2.0 + i * 0.01, 0.01);
    }
    s = o.Tick(3.0, 0.01);
    CHECK(Near(s.yaw, M_PI / 2.0, 0.02));
    CHECK(Near(s.x, 1.0, 0.03));
    CHECK(Near(s.y, 1.0, 0.03));
  }

  // ---- yaw to quaternion: the half angle ---------------------------------
  {
    // *** Checked at angles where a MISSING half would show. At 0 and at pi
    // the two forms agree, so a test written with only those two values passes
    // on sin(yaw) -- and sin(yaw) is wrong everywhere else.
    const Quaternion q0 = YawToQuaternion(0.0);
    CHECK(Near(q0.z, 0.0, 1e-12) && Near(q0.w, 1.0, 1e-12));

    const Quaternion q90 = YawToQuaternion(M_PI / 2.0);
    // Half of 90 degrees is 45, so both components are sqrt(2)/2. Under
    // sin(yaw) they would be 1.0 and 0.0.
    CHECK(Near(q90.z, std::sqrt(2.0) / 2.0, 1e-9));
    CHECK(Near(q90.w, std::sqrt(2.0) / 2.0, 1e-9));

    const Quaternion q180 = YawToQuaternion(M_PI);
    CHECK(Near(q180.z, 1.0, 1e-9));
    CHECK(Near(q180.w, 0.0, 1e-9));

    const Quaternion qm90 = YawToQuaternion(-M_PI / 2.0);
    CHECK(Near(qm90.z, -std::sqrt(2.0) / 2.0, 1e-9));
    CHECK(Near(qm90.w, std::sqrt(2.0) / 2.0, 1e-9));

    // Unit norm at a handful of angles: a quaternion that is not unit is
    // rejected by tf2 with a message that names neither this file nor yaw.
    for (double a2 = -3.0; a2 <= 3.0; a2 += 0.37) {
      const Quaternion q = YawToQuaternion(a2);
      CHECK(Near(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12));
      CHECK(q.x == 0.0 && q.y == 0.0);   // yaw only
    }
  }

  // ---- the 6x6 covariance indices ---------------------------------------
  {
    // The numbers ARE the bug. Yaw lives at [35], not [5]: writing it to [5]
    // fills the x-row's yaw CORRELATION and leaves the yaw variance zero, and
    // a planner then treats the heading as exact.
    double cov[36];
    FillCovariance36(0.25, 0.36, 0.01, cov);
    CHECK(cov[0] == 0.25);
    CHECK(cov[7] == 0.36);
    CHECK(cov[35] == 0.01);
    // REP-105: the three unestimated axes say "not provided", not "exact".
    CHECK(cov[14] == -1.0);
    CHECK(cov[21] == -1.0);
    CHECK(cov[28] == -1.0);
    // Everything else is zero -- including [5], which is where a wrong yaw
    // index would have landed.
    CHECK(cov[5] == 0.0);
    int nonzero = 0;
    for (int i = 0; i < 36; ++i) {
      if (cov[i] != 0.0) ++nonzero;
    }
    CHECK(nonzero == 6);
    // A null buffer is ignored rather than crashing the publish path.
    FillCovariance36(1.0, 1.0, 1.0, nullptr);
  }

  // ---- the band names are distinct --------------------------------------
  {
    const std::string names[] = {
        OdomBandName(OdomBand::kFresh), OdomBandName(OdomBand::kWarn),
        OdomBandName(OdomBand::kTwistZero), OdomBandName(OdomBand::kStop)};
    for (int i = 0; i < 4; ++i) {
      CHECK(!names[i].empty());
      for (int j = i + 1; j < 4; ++j) CHECK(names[i] != names[j]);
    }
  }

  if (g_failures == 0) {
    std::printf("ALL ODOMETRY TESTS PASSED\n");
    return 0;
  }
  std::printf("%d ODOMETRY TEST(S) FAILED\n", g_failures);
  return 1;
}
