/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: units.h
 * Brief: C++17 strong typedefs for the four motion scalars, mirroring the
 *        Python xbrain.common.types module
 *
 * Description:
 * CFG-CM-18, C++ side. Same purpose as the Python units module: make a
 * dimension mix-up a compile error rather than a silently wrong number. Each
 * unit is a distinct struct, so std::min(Mps, Factor) has no viable overload
 * and fails to compile, while std::min(Mps, Mps) is fine.
 *
 * Why distinct structs and not `using Mps = double`. A type alias is
 * transparent: std::min(Mps, Factor) would deduce std::min<double> and compile,
 * which is the exact hole this item closes. tests/common/test_units.py compiles
 * a must-not-compile snippet to hold that line.
 *
 * What this header deliberately omits. No arithmetic operators are defined --
 * only same-type ordering, which is what std::min / std::max and a clamp need.
 * An operator is added when a real caller needs it (CLAUDE.md 9.3), together
 * with its test. This header pulls in NO ROS type and NO rclcpp: it is consumed
 * on the chassis_relay e-stop path (CLAUDE.md 5.3), which forbids both.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_UNITS_UNITS_H_
#define HACHIST_XBRAIN_V6_COMMON_UNITS_UNITS_H_

namespace xbrain {
namespace units {

// Speed, metres per second. Output of the speed gate; the only unit the single
// velocity exit (p1_motion) may emit.
struct Mps {
  double value;
};

// Dimensionless multiplier in [0, 1] (a speed-gate band coefficient, a boost
// factor). Must never reach a std::min that is choosing a speed.
struct Factor {
  double value;
};

// Acceleration / deceleration, metres per second squared (a brake limit).
struct Mps2 {
  double value;
};

// A duration in seconds -- a steady_clock delta or a timeout budget, NOT a
// wall-clock timestamp (CLK-C1).
struct Seconds {
  double value;
};

// Same-type ordering only. std::min / std::max resolve through operator<, so
// providing it per type makes std::min(Mps, Mps) compile. There is
// deliberately NO cross-type operator<: std::min(Mps, Factor) then has no
// overload and the translation unit fails to compile, which is the guarantee.
inline bool operator<(Mps a, Mps b) { return a.value < b.value; }
inline bool operator<(Factor a, Factor b) { return a.value < b.value; }
inline bool operator<(Mps2 a, Mps2 b) { return a.value < b.value; }
inline bool operator<(Seconds a, Seconds b) { return a.value < b.value; }

}  // namespace units
}  // namespace xbrain

#endif  // HACHIST_XBRAIN_V6_COMMON_UNITS_UNITS_H_
