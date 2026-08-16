/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: main.cc
 * Brief: rtk_driver process entry -- config -> serial -> RtkDriver -> RT publish
 *
 * Description:
 * The real process now (the earlier parse-only smoke tool is gone). Startup order
 * is deliberate and fail-STOP: read the runtime identity, load the resolved config
 * (3.1 -- a null threshold throws here), open the serial port, open the RT-plane
 * ZenohSink, then run the 20 Hz loop. Any of the first four failing prints an
 * English reason and exits non-zero, so the process never runs half-configured.
 *
 * Identity sourcing (not in the per-proc config file): rid comes from the L5
 * whitelist env XBRAIN_ROBOT_ID (layers.py maps it to common.robot_id) and is
 * required -- an empty rid cannot form a valid xbrain/{rid}/... key, so it throws
 * rather than defaulting. boot is the OS boot_id first-8-hex, the SAME value the
 * Python read_local_boot_id uses, so envelopes from C++ and Python age-compare on
 * one host. src is the fixed "rtk_driver".
 *
 * Clocks (CLK-C1 / 3.4): the loop period, and every age the driver computes, use
 * steady_clock. system_clock is read ONLY for the envelope ts (wall, for cross-
 * host align + logging), never for a timeout.
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include "sensor/rtk_config.h"
#include "sensor/rtk_driver.h"
#include "sensor/zenoh_sink.h"

namespace {

// Monotonic seconds for periods and ages (CLK-C1). Never the wall clock.
double MonoNowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Wall milliseconds for the envelope ts only (align + log, 11 S3.0). Not a timeout.
int64_t WallMs() {
  // Marker on the system_clock line itself (clock_scan associates the exemption
  // with the occurrence line): envelope ts is cross-machine align / log only.
  auto now = std::chrono::system_clock::now();  // WALL-CLOCK-OK(align): envelope ts (11 S3.0), never an age/timeout
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             now.time_since_epoch())
      .count();
}

// OS boot id, first 8 hex chars -- matches Python read_local_boot_id so a C++
// envelope's boot equals its Python neighbours' on the same host.
std::string ReadBootId() {
  std::ifstream f("/proc/sys/kernel/random/boot_id");
  std::string s;
  std::getline(f, s);
  std::string hex;
  for (char c : s) {
    if (std::isxdigit(static_cast<unsigned char>(c))) hex.push_back(c);
    if (hex.size() >= 8) break;
  }
  return hex;
}

speed_t BaudConst(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B0;
  }
}

// Open the serial port raw 8N1, non-blocking. Returns the fd or -1 on failure.
int OpenSerial(const std::string& port, int baud) {
  const int fd = open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    std::fprintf(stderr, "rtk_driver: open %s failed: %s\n", port.c_str(), std::strerror(errno));
    return -1;
  }
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::fprintf(stderr, "rtk_driver: tcgetattr failed: %s\n", std::strerror(errno));
    close(fd);
    return -1;
  }
  const speed_t b = BaudConst(baud);
  if (b == B0) {
    std::fprintf(stderr, "rtk_driver: unsupported baud %d\n", baud);
    close(fd);
    return -1;
  }
  cfsetispeed(&tty, b);
  cfsetospeed(&tty, b);
  tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tty.c_cflag |= (CS8 | CLOCAL | CREAD);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;  // non-blocking; the 20 Hz pacing is done by the loop
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::fprintf(stderr, "rtk_driver: tcsetattr failed: %s\n", std::strerror(errno));
    close(fd);
    return -1;
  }
  return fd;
}

const char* EnvOr(const char* name, const char* fallback) {
  const char* v = std::getenv(name);
  return (v && *v) ? v : fallback;
}

// Read chrony's tracking state (CLK-A2: only rtk_driver may read chrony). One CSV
// line from `chronyc -c tracking`: refid, name, stratum, ref_time, sys_offset,
// last_offset, rms_offset, ..., leap. have=false if chrony is unreachable, which
// the judge maps to source=none / sync=false (fail-safe).
sensor::ChronyReading ReadChrony(double wall_now_s) {
  sensor::ChronyReading r;
  FILE* p = popen("chronyc -c tracking 2>/dev/null", "r");
  if (p == nullptr) return r;
  char line[512];
  char* got = std::fgets(line, sizeof(line), p);
  pclose(p);
  if (got == nullptr) return r;
  std::string f[14];
  int fi = 0;
  std::string cur;
  for (const char* c = line; *c != '\0' && fi < 14; ++c) {
    if (*c == ',') {
      f[fi++] = cur;
      cur.clear();
    } else if (*c != '\n' && *c != '\r') {
      cur.push_back(*c);
    }
  }
  if (fi < 14) f[fi++] = cur;   // the last (leap) field has no trailing comma
  if (fi < 14) return r;        // malformed -> have=false
  r.have = true;
  const std::string& name = f[1];
  // A refclock name -> rtk (PPS); an IP/hostname -> ntp; 00000000 -> no source.
  if (name == "PPS" || name == "GPS" || name == "NMEA" || name == "SHM" ||
      name == "PHC" || name == "SOCK") {
    r.is_pps_refclock = true;
  } else if (!name.empty() && f[0] != "00000000") {
    r.is_ntp = true;
  }
  r.leap_normal =
      !f[13].empty() && f[13].find("Not synchronised") == std::string::npos;
  r.offset_ms = std::atof(f[5].c_str()) * 1000.0;
  r.rms_ms = std::atof(f[6].c_str()) * 1000.0;
  r.utc_ref = std::atof(f[3].c_str());
  r.ref_age_s = wall_now_s - r.utc_ref;
  if (r.ref_age_s < 0.0) r.ref_age_s = 0.0;
  return r;
}

}  // namespace

int main() {
  // 1) Identity. rid is required (3.1): no valid key without it.
  const char* rid_env = std::getenv("XBRAIN_ROBOT_ID");
  if (rid_env == nullptr || *rid_env == '\0') {
    std::fprintf(stderr, "rtk_driver: XBRAIN_ROBOT_ID is not set (required, no default)\n");
    return 1;
  }
  const std::string rid = rid_env;
  const std::string boot = ReadBootId();
  const std::string src = "rtk_driver";

  // 2) Config: read the resolved product (10 S5.4.1), never the source.
  const std::string resolved_dir = EnvOr("XBRAIN_RESOLVED_DIR", "/run/xbrain/resolved");
  const std::string cfg_path = resolved_dir + "/rtk_driver.yaml";
  sensor::RtkConfig cfg;
  try {
    cfg = sensor::LoadRtkConfig(cfg_path, rid, src, boot);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "rtk_driver: config load failed: %s\n", e.what());
    return 1;
  }

  // 3) Serial.
  const int fd = OpenSerial(cfg.serial_port, cfg.serial_baud);
  if (fd < 0) return 1;

  // 4) RT-plane transport (throws if the RT router is unreachable).
  std::unique_ptr<sensor::ZenohSink> sink;
  try {
    sink = std::make_unique<sensor::ZenohSink>();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "rtk_driver: %s\n", e.what());
    close(fd);
    return 1;
  }

  sensor::RtkDriver driver(cfg.driver, sink.get());
  std::printf("rtk_driver: rid=%s boot=%s serial=%s@%d -> RT rt/gnss/heading @20Hz\n",
              rid.c_str(), boot.c_str(), cfg.serial_port.c_str(), cfg.serial_baud);

  // 5) 20 Hz loop. steady_clock paces the period; each tick feeds fresh serial
  //    bytes and publishes the resolved GnssHeading.
  constexpr auto kPeriod = std::chrono::milliseconds(50);
  auto next = std::chrono::steady_clock::now();
  char buf[1024];
  int64_t ticks = 0;
  int64_t bytes_total = 0;
  double last_hb = MonoNowS();
  double last_clock = 0.0;
  for (;;) {
    const ssize_t n = read(fd, buf, sizeof(buf));
    const double mono = MonoNowS();
    if (n > 0) {
      driver.feed(buf, static_cast<std::size_t>(n), mono);
      bytes_total += n;
    }
    driver.tick(mono, WallMs());
    // 1 Hz rt/clock/status (11 S3.11). The chrony read is I/O, done here (not in
    // the 20 Hz gnss path), once per second.
    if (mono - last_clock >= 1.0) {
      const int64_t w = WallMs();
      driver.tickClock(ReadChrony(static_cast<double>(w) / 1000.0), mono, w);
      last_clock = mono;
    }
    ++ticks;
    if (mono - last_hb >= 2.0) {  // heartbeat: proves the loop and serial are alive
      std::printf("rtk_driver: alive ticks=%lld serial_bytes=%lld\n",
                  static_cast<long long>(ticks), static_cast<long long>(bytes_total));
      std::fflush(stdout);
      last_hb = mono;
    }
    next += kPeriod;
    std::this_thread::sleep_until(next);
  }
  // unreachable
}
