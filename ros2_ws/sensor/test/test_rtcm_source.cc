/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rtcm_source.cc
 * Brief: Offline unit test for SourceHealth + SourceSelector (11 S3.2A RTG-3/4/5)
 *
 * Description:
 * Health: observations alone are not enough, the station message is required,
 * and a lost connection is invalid at once. Selector: every transition of the
 * table in README S4.6.2, the up-slow hold restarting on a single bad update,
 * and the cold-standby rule -- no dial during the start-up grace, linger only
 * after coming back from the backup.
 */

#include "sensor/rtcm_source.h"

#include <cstdio>

using sensor::ActiveSource;
using sensor::HealthConfig;
using sensor::Rtcm3Frame;
using sensor::SelectorConfig;
using sensor::SourceHealth;
using sensor::SourceSelector;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

static Rtcm3Frame frame(int type, int station) {
  Rtcm3Frame f;
  f.msg_type = type;
  f.station_id = station;
  return f;
}

static void test_health_needs_station_message() {
  SourceHealth h(HealthConfig{2.0, 15.0});
  CHECK(!h.valid(0.0));
  h.onFrame(frame(1074, 1), 1.0);
  h.onFrame(frame(1124, 1), 1.0);
  CHECK(!h.valid(1.0));                       // observations without 1005/1006
  h.onFrame(frame(1005, 1), 1.5);
  CHECK(h.valid(1.5));
  CHECK(h.valid(3.0));                        // obs window edge: 3.0 - 1.0 == 2.0
  CHECK(!h.valid(3.01));                      // observations stopped
  h.onFrame(frame(1074, 1), 10.0);
  CHECK(h.valid(10.0));
  CHECK(!h.valid(16.5));                      // obs 6.5 s old: invalid
}

static void test_health_station_window() {
  SourceHealth h(HealthConfig{2.0, 15.0});
  h.onFrame(frame(1006, 2), 0.0);
  h.onFrame(frame(1077, 2), 15.0);
  CHECK(h.valid(15.0));                       // station exactly 15.0 old
  h.onFrame(frame(1077, 2), 15.5);
  CHECK(!h.valid(15.5));                      // station too old, obs fresh
}

static void test_health_reset_and_station_changes() {
  SourceHealth h(HealthConfig{2.0, 15.0});
  h.onFrame(frame(1005, 7), 0.0);
  h.onFrame(frame(1074, 7), 0.0);
  CHECK(h.valid(0.5));
  h.reset();
  CHECK(!h.valid(0.5));                       // connection lost: invalid now
  CHECK(h.stationId() == 7);                  // kept for the logs
  h.onFrame(frame(1005, 8), 1.0);
  CHECK(h.stationIdChanges() == 1);
  CHECK(h.stationId() == 8);
  h.onFrame(frame(1033, -1), 1.0);            // no station field: not a change
  CHECK(h.stationIdChanges() == 1);
}

static SelectorConfig scfg() { return SelectorConfig{5.0, 30.0, 10.0}; }

static void test_selector_none_to_primary_is_immediate() {
  SourceSelector s(scfg());
  auto r = s.update(true, false, 0.0);
  CHECK(r.active == ActiveSource::kPrimary);
  CHECK(r.switched);
  r = s.update(true, false, 0.1);
  CHECK(!r.switched);
  CHECK(!s.wantBackup(0.1));                  // never used the backup: no linger
}

static void test_selector_down_fast() {
  SourceSelector s(scfg());
  s.update(true, true, 0.0);
  auto r = s.update(false, true, 1.0);
  CHECK(r.active == ActiveSource::kBackup);   // the same update, no hold
  CHECK(r.switched);
  SourceSelector s2(scfg());
  s2.update(true, false, 0.0);
  r = s2.update(false, false, 1.0);
  CHECK(r.active == ActiveSource::kNone);
  CHECK(r.switched);
}

static void test_selector_up_slow_and_restart() {
  SourceSelector s(scfg());
  s.update(false, true, 0.0);                 // NONE -> BACKUP
  CHECK(s.active() == ActiveSource::kBackup);
  s.update(true, true, 10.0);
  s.update(true, true, 14.9);
  CHECK(s.active() == ActiveSource::kBackup); // 4.9 s < hold_up_s
  s.update(false, true, 14.95);               // one bad update
  s.update(true, true, 15.0);
  s.update(true, true, 19.9);
  CHECK(s.active() == ActiveSource::kBackup); // the run restarted at 15.0
  auto r = s.update(true, true, 20.0);
  CHECK(r.active == ActiveSource::kPrimary);
  CHECK(r.switched);
}

static void test_selector_backup_lost() {
  SourceSelector s(scfg());
  s.update(false, true, 0.0);
  auto r = s.update(false, false, 1.0);
  CHECK(r.active == ActiveSource::kNone);
  SourceSelector s2(scfg());
  s2.update(false, true, 0.0);
  r = s2.update(true, false, 1.0);            // backup gone, primary valid but not held
  CHECK(r.active == ActiveSource::kPrimary);
}

static void test_selector_cold_standby() {
  SourceSelector s(scfg());
  s.update(false, false, 0.0);
  CHECK(!s.wantBackup(0.0));                  // start-up grace
  CHECK(!s.wantBackup(9.9));
  CHECK(s.wantBackup(10.0));                  // primary invalid for backup_after_s
  s.update(false, true, 12.0);                // backup up and valid
  CHECK(s.active() == ActiveSource::kBackup);
  CHECK(s.wantBackup(12.0));
  s.update(true, true, 20.0);
  CHECK(s.wantBackup(20.0));                  // primary valid but not held: still on backup
  s.update(true, true, 25.0);                 // held -> back to primary at 25.0
  CHECK(s.active() == ActiveSource::kPrimary);
  CHECK(s.wantBackup(54.9));                  // linger_s after returning
  CHECK(!s.wantBackup(55.0));
}

static void test_selector_grace_restarts() {
  SourceSelector s(scfg());
  s.update(false, false, 0.0);
  s.update(true, false, 5.0);                 // primary came up within the grace
  s.update(false, false, 6.0);                // and dropped
  CHECK(!s.wantBackup(15.9));                 // grace counts from 6.0
  CHECK(s.wantBackup(16.0));
}

int main() {
  test_health_needs_station_message();
  test_health_station_window();
  test_health_reset_and_station_changes();
  test_selector_none_to_primary_is_immediate();
  test_selector_down_fast();
  test_selector_up_slow_and_restart();
  test_selector_backup_lost();
  test_selector_cold_standby();
  test_selector_grace_restarts();
  if (g_failures == 0) {
    std::printf("ALL RTCM SOURCE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RTCM SOURCE TEST(S) FAILED\n", g_failures);
  return 1;
}
