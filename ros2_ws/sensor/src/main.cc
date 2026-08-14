/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: main.cc
 * Brief: rtk_driver serial smoke tool -- open the module, parse NMEA, print facts
 *
 * Description:
 * What this runs today. The full rtk_driver loop (parse -> resolver -> GnssHeading
 * -> RT publish) needs two foundations that are not in place yet: a C++ config
 * loader for configs/rtk_driver.yaml (none exists under common/, 10 S5.4.1) and
 * the Zenoh binding (11 S2.4.1 version unlocked, not installed). The resolver and
 * driver core are already built and OFFLINE-tested (test_heading_resolver /
 * test_rtk_driver). So this executable is the piece that needs REAL HARDWARE: it
 * opens the serial port, parses the module's NMEA with the same nmea_parser the
 * driver uses, and prints the GGA / TRA / RMC facts plus the ENU heading the
 * resolver would compute -- verifying serial + parse + convert on the ORIN with
 * the real module, on the real aarch64 build.
 *
 * Why it does not build a RtkDriver. That needs DriverConfig, whose resolver
 * thresholds are safety values (CLAUDE.md 3.1 forbids a code default), so they
 * must come from configs/ via a loader that does not exist yet. Hardcoding them
 * here to make a demo run is exactly the silent default 3.1 rules out. When the
 * config loader lands, this becomes the 20 Hz loop feeding a RtkDriver + a Zenoh
 * PublishSink; until then it is an honest hardware smoke test, not a stub of the
 * running process.
 *
 * Boundary: no rclcpp, no zenoh. port/baud come from argv (I/O config, not safety
 * params -- a default is fine); nothing here reads configs/.
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "sensor/nmea_parser.h"

namespace {
constexpr double kPi = 3.14159265358979323846;

// True-north clockwise degrees -> ENU radians, the same flip the resolver does
// (11 S3.3: heading_enu = wrap(pi/2 - heading_ned)). Shown so the operator can
// sanity-check the conversion against the physical antenna direction on the bench.
double TrueDegToEnuDeg(double deg) {
  double a = kPi / 2.0 - deg * kPi / 180.0;
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return (a - kPi) * 180.0 / kPi;
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

// Open the serial port raw 8N1 at `baud`. Returns the fd, or -1 on failure.
int OpenSerial(const char* port, int baud) {
  const int fd = open(port, O_RDONLY | O_NOCTTY);
  if (fd < 0) {
    std::fprintf(stderr, "open %s failed: %s\n", port, std::strerror(errno));
    return -1;
  }
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::fprintf(stderr, "tcgetattr failed: %s\n", std::strerror(errno));
    close(fd);
    return -1;
  }
  const speed_t b = BaudConst(baud);
  if (b == B0) {
    std::fprintf(stderr, "unsupported baud %d\n", baud);
    close(fd);
    return -1;
  }
  cfsetispeed(&tty, b);
  cfsetospeed(&tty, b);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;   // 1 s read timeout so the loop can bound itself
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::fprintf(stderr, "tcsetattr failed: %s\n", std::strerror(errno));
    close(fd);
    return -1;
  }
  return fd;
}

void PrintLine(const std::string& line) {
  sensor::GgaFix gga;
  if (sensor::ParseGga(line, &gga)) {
    std::printf("GGA  q=%d sats=%d %s%.7f,%.7f alt=%.1f\n", gga.quality,
                gga.num_satellites, gga.valid ? "" : "(no-fix) ",
                gga.latitude_deg, gga.longitude_deg, gga.altitude_m);
    return;
  }
  sensor::TraHeading tra;
  if (sensor::ParseTra(line, &tra)) {
    std::printf("TRA  hdg=%.2f (ENU %.2f) pitch=%.2f QF=%d%s sats=%d\n",
                tra.heading_true_deg, TrueDegToEnuDeg(tra.heading_true_deg),
                tra.pitch_deg, tra.quality,
                tra.quality == 4 ? "(NARROW_INT)" : tra.quality == 5 ? "(FLOAT)" : "",
                tra.num_satellites);
    return;
  }
  sensor::RmcData rmc;
  if (sensor::ParseRmc(line, &rmc)) {
    std::printf("RMC  %s spd=%.3f m/s cog=%s\n", rmc.status_active ? "A" : "V",
                rmc.speed_mps, rmc.cog_present ? std::to_string(rmc.cog_deg).c_str()
                                               : "(empty)");
  }
}
}  // namespace

int main(int argc, char** argv) {
  const char* port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
  const int baud = (argc > 2) ? std::atoi(argv[2]) : 115200;
  const int max_sentences = (argc > 3) ? std::atoi(argv[3]) : 0;  // 0 = run forever

  const int fd = OpenSerial(port, baud);
  if (fd < 0) return 1;
  std::printf("rtk_driver smoke: %s @ %d 8N1 (parse-only; full loop needs config"
              " loader + zenoh)\n", port, baud);

  std::string rx;
  int printed = 0;
  char buf[512];
  for (;;) {
    const ssize_t n = read(fd, buf, sizeof(buf));
    if (n > 0) rx.append(buf, static_cast<size_t>(n));
    std::size_t nl;
    while ((nl = rx.find('\n')) != std::string::npos) {
      const std::string line = rx.substr(0, nl);
      rx.erase(0, nl + 1);
      if (line.find('$') == std::string::npos) continue;
      PrintLine(line);
      if (max_sentences > 0 && ++printed >= max_sentences) {
        close(fd);
        return 0;
      }
    }
  }
}
