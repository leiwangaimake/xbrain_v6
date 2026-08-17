/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_serial_reopen.cc
 * Brief: ClassifySerialRead -- USB-hotplug read-result decision (11 CLK-C1)
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

using sensor::ClassifySerialRead;
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
  CHECK(ClassifySerialRead(64, 0, 10.0, 9.9, kStale) == SerialAction::kFeed);

  // n == 0 (EOF) -> the device hung up on unplug. MUTATION: kKeep would never
  // recover after a hangup.
  CHECK(ClassifySerialRead(0, 0, 10.0, 9.9, kStale) == SerialAction::kReopen);

  // n < 0 with a hard errno (removed CDC-ACM) -> reopen.
  CHECK(ClassifySerialRead(-1, ENODEV, 10.0, 9.9, kStale) ==
        SerialAction::kReopen);
  CHECK(ClassifySerialRead(-1, EIO, 10.0, 9.9, kStale) == SerialAction::kReopen);

  // n < 0 EAGAIN, still fresh -> just "no data this tick", keep the fd.
  // MUTATION: kReopen here would thrash open()/close() every idle 50 ms tick.
  CHECK(ClassifySerialRead(-1, EAGAIN, 10.0, 9.5, kStale) ==
        SerialAction::kKeep);

  // n < 0 EAGAIN, past the stale window -> the reported bug: some kernels return
  // EAGAIN forever on an unplugged device. MUTATION: kKeep here is EXACTLY the old
  // behaviour that never recovered.
  CHECK(ClassifySerialRead(-1, EAGAIN, 14.0, 10.0, kStale) ==
        SerialAction::kReopen);
  // exactly at the threshold counts as stale.
  CHECK(ClassifySerialRead(-1, EAGAIN, 13.0, 10.0, kStale) ==
        SerialAction::kReopen);

  // EWOULDBLOCK behaves like EAGAIN (same intent; may be a distinct value).
  CHECK(ClassifySerialRead(-1, EWOULDBLOCK, 10.0, 9.5, kStale) ==
        SerialAction::kKeep);

  if (g_failures == 0) std::printf("test_serial_reopen: all passed\n");
  return g_failures;
}
