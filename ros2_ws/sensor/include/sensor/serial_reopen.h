/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: serial_reopen.h
 * Brief: decide feed/keep/reopen from a non-blocking serial read result (USB hotplug)
 *
 * Description:
 * The rtk_driver reads its GPS/RTK module over a non-blocking CDC-ACM serial port.
 * When the USB cable is unplugged the fd goes dead; the ORIGINAL loop only acted on
 * n>0 and silently ignored everything else, so a dead fd was held forever and the
 * link never recovered when the cable was plugged back in (the reported bug). This
 * pure decision -- split from the I/O so it is unit-tested with no device -- tells
 * the loop what a read() result means:
 *
 *   n > 0                          -> kFeed   (real bytes)
 *   device node gone               -> kReopen (unplugged: /dev/ttyACMx vanished)
 *   n < 0, errno hard (ENODEV/EIO) -> kReopen (removed CDC-ACM)
 *   n == 0 or EAGAIN, within stale -> kKeep   (normal: no data THIS tick)
 *   n == 0 or EAGAIN, past stale   -> kReopen (silent-death catch-all: a GPS streams
 *                                     NMEA >= 1 Hz, so going quiet past `stale_s`
 *                                     means the port died)
 *
 * n == 0 is NOT a hang-up on this port. The fd is raw with VMIN = 0 / VTIME = 0,
 * and on such a tty read() returns 0 whenever no byte is waiting, O_NONBLOCK or
 * not. MEASURED 2026-11-02 on the ORIN (UM982 silent): the earlier "0 = unplug"
 * rule reopened the port 43 times in 45 s, and every reopen drops the RTCM write
 * queue (11 S3.2A) and any half-read NMEA line. An unplug is recognised instead
 * by the device node disappearing (the caller checks the path), with the stale
 * watchdog as the catch-all.
 *
 * On kReopen the loop closes the fd and re-open()s the SAME configured port; the
 * kernel gives the re-plugged CDC-ACM device its node back, so the link resumes.
 * All times are CLOCK_MONOTONIC seconds (CLK-C1), passed in.
 */

#ifndef SENSOR__SERIAL_REOPEN_H_
#define SENSOR__SERIAL_REOPEN_H_

#include <cerrno>
#include <sys/types.h>  // ssize_t

namespace sensor {

enum class SerialAction { kFeed, kKeep, kReopen };

// Pure classification of a non-blocking serial read() result. n/err are read()'s
// return value and errno; now_mono_s is the current monotonic time, last_byte_mono_s
// is when the last real bytes arrived, stale_s is the no-data reopen threshold,
// device_present is whether the configured device path still exists.
inline SerialAction classify_serial_read(ssize_t n, int err, double now_mono_s,
                                       double last_byte_mono_s, double stale_s,
                                       bool device_present) {
  if (n > 0) return SerialAction::kFeed;
  if (!device_present) return SerialAction::kReopen;   // node gone -- unplugged
  // n < 0 with a real error; n == 0 or EAGAIN is "no data yet" (see header).
  if (n < 0 && err != EAGAIN && err != EWOULDBLOCK) {
    return SerialAction::kReopen;             // ENODEV / EIO / EBADF -- port gone
  }
  if (now_mono_s - last_byte_mono_s >= stale_s) {
    return SerialAction::kReopen;             // EAGAIN for too long -- silent death
  }
  return SerialAction::kKeep;                 // normal: no data this tick
}

}  // namespace sensor

#endif  // SENSOR__SERIAL_REOPEN_H_
