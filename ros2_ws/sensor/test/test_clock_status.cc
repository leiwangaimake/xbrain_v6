/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_clock_status.cc
 * Brief: Offline unit test for the ClockStatus judgement (11 S3.11)
 *
 * Description:
 * The load-bearing mutants (CLAUDE.md 3.3): NTP must NOT sync when chrony is not
 * synchronised (leap) or the offset is over the gate; a PPS rtk source with a
 * stale (>5 s) reference must NOT sync; RTC must NOT sync after a wall step. And
 * the documented NTP reading: an NTP source with a huge ref_age (its poll age)
 * still syncs when leap is Normal + offset is in gate -- ref_age is NOT the NTP
 * gate (see clock_status.h). offset serialises null when there is no reference.
 */

#include "sensor/clock_status.h"

#include <cstdio>
#include <string>

using sensor::ChronyReading;
using sensor::ClockConfig;
using sensor::ClockStatus;
using sensor::JudgeClock;
using sensor::ToJsonData;

static int g_failures = 0;
#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)
static bool Has(const std::string& s, const std::string& sub) {
  return s.find(sub) != std::string::npos;
}

static const ClockConfig kCfg{20.0, 5.0, false};  // offset<=20, ref_age<=5, rtc untrusted

int main() {
  // NTP synced: leap normal, offset 4.5 ms, HUGE ref_age (720 s poll age) -> STILL
  // sync (ref_age is not the NTP gate, clock_status.h).
  ChronyReading ntp;
  ntp.have = true; ntp.is_ntp = true; ntp.leap_normal = true;
  ntp.offset_ms = 4.5; ntp.rms_ms = 1.5; ntp.ref_age_s = 720.0; ntp.utc_ref = 1786856085.0;
  ClockStatus a = JudgeClock(ntp, kCfg, 0, 100.0, "abcd1234");
  CHECK(a.sync == true);
  CHECK(a.source == "ntp");
  CHECK(a.quality == "precise");
  CHECK(a.boot == "abcd1234");

  // 3.3 mutant: NTP with leap NOT normal -> no sync.
  ChronyReading n2 = ntp; n2.leap_normal = false;
  CHECK(JudgeClock(n2, kCfg, 0, 100.0, "b").sync == false);
  // 3.3 mutant: NTP offset over the gate -> no sync.
  ChronyReading n3 = ntp; n3.offset_ms = 25.0;
  CHECK(JudgeClock(n3, kCfg, 0, 100.0, "b").sync == false);

  // rtk (PPS): fresh (ref_age 0.4 <= 5) -> sync rtk.
  ChronyReading pps;
  pps.have = true; pps.is_pps_refclock = true; pps.leap_normal = true;
  pps.offset_ms = 2.0; pps.ref_age_s = 0.4;
  ClockStatus p = JudgeClock(pps, kCfg, 0, 100.0, "b");
  CHECK(p.sync == true && p.source == "rtk");
  // 3.3 mutant: PPS with stale ref (>5 s) -> no sync (a lost pulse; PPS DOES gate
  // on ref_age, unlike NTP).
  ChronyReading p2 = pps; p2.ref_age_s = 9.0;
  CHECK(JudgeClock(p2, kCfg, 0, 100.0, "b").sync == false);

  // no source (chrony unreachable) -> sync false, source none, offset null.
  ChronyReading none;   // have=false
  ClockStatus z = JudgeClock(none, kCfg, 0, 100.0, "b");
  CHECK(z.sync == false && z.source == "none");
  CHECK(Has(ToJsonData(z), "\"offset_ms\":null"));   // no ref -> null, not 0

  // rtc: trusted + no step -> coarse sync; a step -> no sync.
  ClockConfig rtc_cfg{20.0, 5.0, true};
  CHECK(JudgeClock(none, rtc_cfg, 0, 100.0, "b").source == "rtc");
  CHECK(JudgeClock(none, rtc_cfg, 1, 100.0, "b").sync == false);   // step_count>0 -> no rtc

  // JSON of a real sync carries the fields.
  const std::string j = ToJsonData(a);
  CHECK(Has(j, "\"sync\":true"));
  CHECK(Has(j, "\"source\":\"ntp\""));
  CHECK(Has(j, "\"offset_ms\":"));
  CHECK(!Has(j, "\"offset_ms\":null"));

  if (g_failures == 0) {
    std::printf("ALL CLOCK STATUS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CLOCK STATUS TEST(S) FAILED\n", g_failures);
  return 1;
}
