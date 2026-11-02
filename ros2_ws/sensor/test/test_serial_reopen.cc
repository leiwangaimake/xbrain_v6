/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_serial_reopen.cc
 * Brief: classify_serial_read -- USB-hotplug read-result decision (11 CLK-C1)
 *
 * Description:
 * Pins the fix for the reported bug: unplugging the RTK USB and plugging it back in
 * must recover. The decision is pure, so each read() outcome is checked with no
 * device. The load-bearing case is EAGAIN-past-stale -> kReopen: the ORIGINAL loop
 * treated every non-positive read as "no data" and held a dead fd forever, so the
 * link never came back. Each check names the mutation it reddens.
 */

#include "sensor/serial_reopen.h"

#include <cerrno>
#include <cstdio>

using sensor::classify_serial_read;
using sensor::SerialAction;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

int main() {
  const double kStale = 3.0;

  // n > 0 -> feed the bytes. MUTATION: kKeep here would drop live GPS data.
  CHECK(classify_serial_read(64, 0, 10.0, 9.9, kStale, true) == SerialAction::kFeed);

  // n == 0 on a raw VMIN=0/VTIME=0 tty is "no byte waiting", NOT a hang-up
  // (measured 2026-11-02: treating it as unplug reopened 43 times in 45 s).
  // MUTATION: kReopen here thrashes the port and drops the RTCM queue.
  CHECK(classify_serial_read(0, 0, 10.0, 9.9, kStale, true) == SerialAction::kKeep);
  // ... but a silence past the stale window still reopens.
  CHECK(classify_serial_read(0, 0, 14.0, 10.0, kStale, true) == SerialAction::kReopen);

  // The device node vanished -> unplugged, whatever read() said.
  CHECK(classify_serial_read(0, 0, 10.0, 9.9, kStale, false) == SerialAction::kReopen);
  CHECK(classify_serial_read(-1, EAGAIN, 10.0, 9.9, kStale, false) == SerialAction::kReopen);

  // n < 0 with a hard errno (removed CDC-ACM) -> reopen.
  CHECK(classify_serial_read(-1, ENODEV, 10.0, 9.9, kStale, true) ==
        SerialAction::kReopen);
  CHECK(classify_serial_read(-1, EIO, 10.0, 9.9, kStale, true) == SerialAction::kReopen);

  // n < 0 EAGAIN, still fresh -> just "no data this tick", keep the fd.
  // MUTATION: kReopen here would thrash open()/close() every idle 50 ms tick.
  CHECK(classify_serial_read(-1, EAGAIN, 10.0, 9.5, kStale, true) ==
        SerialAction::kKeep);

  // n < 0 EAGAIN, past the stale window -> the reported bug: some kernels return
  // EAGAIN forever on an unplugged device. MUTATION: kKeep here is EXACTLY the old
  // behaviour that never recovered.
  CHECK(classify_serial_read(-1, EAGAIN, 14.0, 10.0, kStale, true) ==
        SerialAction::kReopen);
  // exactly at the threshold counts as stale.
  CHECK(classify_serial_read(-1, EAGAIN, 13.0, 10.0, kStale, true) ==
        SerialAction::kReopen);

  // EWOULDBLOCK behaves like EAGAIN (same intent; may be a distinct value).
  CHECK(classify_serial_read(-1, EWOULDBLOCK, 10.0, 9.5, kStale, true) ==
        SerialAction::kKeep);

  if (g_failures == 0) std::printf("test_serial_reopen: all passed\n");
  return g_failures;
}
