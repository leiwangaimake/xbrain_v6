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
 *   n == 0 (EOF)                   -> kReopen (the device hung up = unplug)
 *   n < 0, errno hard (ENODEV/EIO) -> kReopen (removed CDC-ACM)
 *   n < 0, EAGAIN, within stale    -> kKeep   (normal: no data THIS tick)
 *   n < 0, EAGAIN, past stale       -> kReopen (silent-death catch-all: some kernels
 *                                     return EAGAIN forever on an unplugged device
 *                                     instead of ENODEV, so a GPS -- which streams
 *                                     NMEA >= 1 Hz -- going quiet past `stale_s`
 *                                     means the port died)
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
// is when the last real bytes arrived, stale_s is the no-data reopen threshold.
inline SerialAction ClassifySerialRead(ssize_t n, int err, double now_mono_s,
                                       double last_byte_mono_s, double stale_s) {
  if (n > 0) return SerialAction::kFeed;
  if (n == 0) return SerialAction::kReopen;   // EOF -- device hung up on unplug
  // n < 0: a real error, or just "no data yet" on a non-blocking fd.
  if (err != EAGAIN && err != EWOULDBLOCK) {
    return SerialAction::kReopen;             // ENODEV / EIO / EBADF -- port gone
  }
  if (now_mono_s - last_byte_mono_s >= stale_s) {
    return SerialAction::kReopen;             // EAGAIN for too long -- silent death
  }
  return SerialAction::kKeep;                 // normal: no data this tick
}

}  // namespace sensor

#endif  // SENSOR__SERIAL_REOPEN_H_
