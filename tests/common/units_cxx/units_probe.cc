/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: units_probe.cc
 * Brief: C++ side of the dimensional types -- Clamp, and the compile-time wall
 *
 * Description:
 * tests/common/test_units.py covers the PYTHON types. This is the C++ half,
 * which had no test at all until Tier 1 became its first consumer, and the two
 * halves guard different things: Python's guard is a runtime isinstance check,
 * C++'s is the absence of an overload.
 *
 * What is asserted here, and the failure each one prevents:
 *
 *   * Clamp is symmetric around zero. Every limit in this system is a MAGNITUDE
 *     (spec.max_vx_mps and friends), so a one-sided clamp to [0, max] would
 *     silently forbid reversing -- a robot that will not back away from an
 *     obstacle, with nothing in any log to say why.
 *   * a non-positive limit yields zero, not the value. An uncalibrated limit
 *     reads as null in the config and stops the process (CLAUDE.md 3.1), but a
 *     limit that arrives as 0.0 through some other path must clamp to zero
 *     rather than pass the command through unlimited. Fail-safe, not fail-open.
 *   * NaN passes through UNCHANGED. This looks wrong and is deliberate: Tier 1
 *     has its own branch that refuses a non-finite command and reports
 *     stop_reason "nan" (13 S3.2). If Clamp quietly turned NaN into a limit,
 *     that branch would never fire and a poisoned payload would be executed as
 *     a legal top-speed command.
 *   * Mps and Radps do not mix. The whole reason Radps exists is that
 *     Clamp(wz_as_Mps, max_vx) compiles and is wrong by whatever the two
 *     numbers happen to be. That guarantee is the ABSENCE of an overload, so it
 *     cannot be asserted from inside a program that must compile -- the
 *     negative case is compiled separately by the Python driver and is expected
 *     to FAIL. What is asserted here is only the positive half.
 */

#include "xbrain/units/units.h"

#include <cmath>
#include <cstdio>

using xbrain::units::Clamp;
using xbrain::units::Mps;
using xbrain::units::Radps;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

int main() {
  const Mps limit{2.0};

  // Inside the band, untouched. Asserting equality and not "close to" because
  // a clamp that returns the limit whenever it is asked anything would pass a
  // tolerance-based check on a value near the limit.
  CHECK(Clamp(Mps{0.0}, limit).value == 0.0);
  CHECK(Clamp(Mps{1.5}, limit).value == 1.5);
  CHECK(Clamp(Mps{-1.5}, limit).value == -1.5);
  // On the boundary, both signs: a strict-versus-non-strict slip shows here and
  // nowhere else.
  CHECK(Clamp(Mps{2.0}, limit).value == 2.0);
  CHECK(Clamp(Mps{-2.0}, limit).value == -2.0);
  // Outside, both signs. The NEGATIVE case is the one a one-sided clamp gets
  // wrong, and a robot that cannot reverse is the visible symptom.
  CHECK(Clamp(Mps{5.0}, limit).value == 2.0);
  CHECK(Clamp(Mps{-5.0}, limit).value == -2.0);

  // A non-positive limit clamps to zero rather than passing the value.
  CHECK(Clamp(Mps{5.0}, Mps{0.0}).value == 0.0);
  CHECK(Clamp(Mps{-5.0}, Mps{0.0}).value == 0.0);
  CHECK(Clamp(Mps{5.0}, Mps{-1.0}).value == 0.0);

  // NaN survives, so Tier 1's own refusal branch is the thing that sees it.
  CHECK(std::isnan(Clamp(Mps{std::nan("")}, limit).value));
  // ...and an infinite command IS clamped, because infinity is comparable and
  // the limit is the correct answer for it. The two non-finite cases differ,
  // and Tier 1 rejects both -- this pins that Clamp does not accidentally make
  // them behave alike.
  CHECK(Clamp(Mps{INFINITY}, limit).value == 2.0);
  CHECK(Clamp(Mps{-INFINITY}, limit).value == -2.0);

  // The angular overload is the same shape, on its own type.
  const Radps wz_limit{1.0};
  CHECK(Clamp(Radps{0.4}, wz_limit).value == 0.4);
  CHECK(Clamp(Radps{3.0}, wz_limit).value == 1.0);
  CHECK(Clamp(Radps{-3.0}, wz_limit).value == -1.0);
  CHECK(Clamp(Radps{3.0}, Radps{0.0}).value == 0.0);

  // Same-type ordering exists for every unit (std::min resolves through it),
  // and Radps was added to that list rather than left out -- an omission that
  // would only surface at the first std::min of two yaw rates.
  CHECK(Radps{1.0} < Radps{2.0});
  CHECK(!(Radps{2.0} < Radps{1.0}));

  // The positive half of the type wall: each overload returns ITS OWN type, so
  // assigning one to the other is a compile error rather than a conversion.
  // The negative half cannot live in a program that must compile; the Python
  // driver compiles it separately and requires the compile to fail.
  const Mps a = Clamp(Mps{1.0}, limit);
  const Radps b = Clamp(Radps{1.0}, wz_limit);
  CHECK(a.value == 1.0 && b.value == 1.0);

  if (g_failures == 0) {
    std::printf("ALL UNITS_CXX TESTS PASSED\n");
    return 0;
  }
  std::printf("%d UNITS_CXX TEST(S) FAILED\n", g_failures);
  return 1;
}
