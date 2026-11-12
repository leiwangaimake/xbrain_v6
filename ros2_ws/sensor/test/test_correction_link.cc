/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_correction_link.cc
 * Brief: Loopback test of the RTCM network thread against fake sources
 *
 * Description:
 * Real sockets on 127.0.0.1, no hardware, no internet, no commercial account:
 *   - LAN: a fake LAN bridge box streams frames; they come out of takeFrames()
 *     tagged as the primary;
 *   - NTRIP cold standby (LAN off): no position -> no dial at all (RTG-7); with
 *     a fresh position the link dials, sends a well-formed v1 request, reports
 *     GGA, and the caster's frames come out tagged as the backup;
 *   - moving beyond vrs_rebase_km reconnects with the new position (RTG-12);
 *   - a 401 is not retried before backoff_auth_s.
 * Timing-based, a few seconds in total; every wait is bounded.
 */

#include "sensor/correction_link.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "sensor/ntrip_protocol.h"
#include "sensor/rtcm3_framer.h"

using namespace sensor;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

static double mono() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

static std::string make_frame(int type, int station, std::size_t extra) {
  std::string payload(3 + extra, '\0');
  payload[0] = static_cast<char>((type >> 4) & 0xFF);
  payload[1] = static_cast<char>(((type & 0x0F) << 4) | ((station >> 8) & 0x0F));
  payload[2] = static_cast<char>(station & 0xFF);
  std::string f;
  f.push_back(static_cast<char>(0xD3));
  f.push_back(static_cast<char>((payload.size() >> 8) & 0x03));
  f.push_back(static_cast<char>(payload.size() & 0xFF));
  f += payload;
  const uint32_t crc = rtcm_crc24q(reinterpret_cast<const uint8_t*>(f.data()), f.size());
  f.push_back(static_cast<char>((crc >> 16) & 0xFF));
  f.push_back(static_cast<char>((crc >> 8) & 0xFF));
  f.push_back(static_cast<char>(crc & 0xFF));
  return f;
}

// Listening socket on 127.0.0.1, ephemeral port.
static int listen_local(int* port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  int on = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  a.sin_port = 0;
  bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a));
  listen(fd, 4);
  socklen_t len = sizeof(a);
  getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len);
  *port = ntohs(a.sin_port);
  return fd;
}

static int accept_within(int lfd, double timeout_s) {
  pollfd p{lfd, POLLIN, 0};
  if (poll(&p, 1, static_cast<int>(timeout_s * 1000)) <= 0) return -1;
  return accept(lfd, nullptr, nullptr);
}

// Read until `needle` has arrived or the timeout passes.
static std::string read_until(int fd, const std::string& needle, double timeout_s) {
  std::string got;
  const double end = mono() + timeout_s;
  while (mono() < end && got.find(needle) == std::string::npos) {
    pollfd p{fd, POLLIN, 0};
    if (poll(&p, 1, 50) > 0) {
      char b[1024];
      const ssize_t n = recv(fd, b, sizeof(b), 0);
      if (n <= 0) break;
      got.append(b, static_cast<std::size_t>(n));
    }
  }
  return got;
}

static std::vector<TaggedFrame> take_within(CorrectionLink& link, std::size_t want, double timeout_s) {
  std::vector<TaggedFrame> all;
  const double end = mono() + timeout_s;
  while (mono() < end && all.size() < want) {
    link.takeFrames(&all);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return all;
}

static CorrectionConfig base_cfg() {
  CorrectionConfig c{};
  c.queue_max_bytes = 16384;
  c.health = HealthConfig{2.0, 15.0};
  c.selector = SelectorConfig{1.0, 1.0, 0.2};
  c.lan = LanSourceConfig{false, "", 0, 1.0, 2.0, 0.2, 1.0};
  c.ntrip = NtripSourceConfig{false, "", 0, "", 1, "", "", 1.0, 2.0, 0.2, 1.0, 30.0, 0.3, 5.0, 10.0};
  return c;
}

static PositionSample position(double lat, double lon) {
  PositionSample s;
  s.has_position = true;
  s.pos.lat_deg = lat;
  s.pos.lon_deg = lon;
  s.pos.alt_m = 10.0;
  s.pos.quality = 1;
  s.pos.sats = 12;
  s.pos.hdop = 0.9;
  s.t_mono = mono();
  s.has_utc = true;
  s.utc_s_of_day = 3600.0;
  return s;
}

static void test_lan_primary() {
  int port = 0;
  const int lfd = listen_local(&port);
  CorrectionConfig c = base_cfg();
  c.lan.enabled = true;
  c.lan.host = "127.0.0.1";
  c.lan.port = port;
  CorrectionLink link(c);
  link.start();
  const int cfd = accept_within(lfd, 3.0);
  CHECK(cfd >= 0);
  if (cfd >= 0) {
    const std::string s = std::string("junk") + make_frame(1005, 3, 16) + make_frame(1074, 3, 40);
    send(cfd, s.data(), s.size(), MSG_NOSIGNAL);
    const auto got = take_within(link, 2, 3.0);
    CHECK(got.size() == 2);
    if (got.size() == 2) {
      CHECK(got[0].source == static_cast<int>(ActiveSource::kPrimary));
      CHECK(got[1].bytes == make_frame(1074, 3, 40));
    }
    CHECK(link.statusLine().find("active=lan") != std::string::npos);
    close(cfd);
  }
  const double t0 = mono();
  link.stop();
  CHECK(mono() - t0 < 1.0);                   // the poll timeout bounds stop()
  close(lfd);
}

static void test_ntrip_cold_standby_and_rebase() {
  int port = 0;
  const int lfd = listen_local(&port);
  CorrectionConfig c = base_cfg();
  c.ntrip.enabled = true;
  c.ntrip.host = "127.0.0.1";
  c.ntrip.port = port;
  c.ntrip.mountpoint = "AUTO";
  c.ntrip.user = "qxuser";
  c.ntrip.password = "secret";
  c.selector.linger_s = 1.0;
  // Long enough that silence can never be what reconnects in this test: the
  // rebuild after moving must be the ONLY cause (the 2 s default once masked a
  // disabled rebuild -- the mutant survived).
  c.ntrip.read_timeout_s = 30.0;
  CorrectionLink link(c);
  link.start();
  // No position: RTG-7 says wait -- the caster must not even see a connection.
  CHECK(accept_within(lfd, 0.8) < 0);
  link.setPosition(position(31.2304, 121.4737));          // Shanghai
  int cfd = accept_within(lfd, 3.0);
  CHECK(cfd >= 0);
  if (cfd < 0) {
    link.stop();
    close(lfd);
    return;
  }
  const std::string req = read_until(cfd, "\r\n\r\n", 2.0);
  CHECK(req.find("GET /AUTO HTTP/1.0\r\n") == 0);
  CHECK(req.find("Authorization: Basic " + base64_encode("qxuser:secret")) != std::string::npos);
  const std::string ok = "ICY 200 OK\r\n";
  send(cfd, ok.data(), ok.size(), MSG_NOSIGNAL);
  const std::string gga = read_until(cfd, "\r\n", 2.0);
  CHECK(gga.find("$GPGGA,") == 0);
  CHECK(gga.find(",3113.82400,N,12128.42200,E,") != std::string::npos);
  const std::string frames = make_frame(1005, 9, 16) + make_frame(1124, 9, 30);
  send(cfd, frames.data(), frames.size(), MSG_NOSIGNAL);
  const auto got = take_within(link, 2, 3.0);
  CHECK(got.size() == 2);
  if (!got.empty()) CHECK(got[0].source == static_cast<int>(ActiveSource::kBackup));

  // RTG-12: 270 km away -> reconnect, and the new GGA carries the new position.
  for (int i = 0; i < 20; ++i) {                          // keep the position fresh
    link.setPosition(position(32.0603, 118.7969));        // Nanjing
    send(cfd, frames.data(), frames.size(), MSG_NOSIGNAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const int cfd2 = accept_within(lfd, 3.0);
  CHECK(cfd2 >= 0);
  if (cfd2 >= 0) {
    read_until(cfd2, "\r\n\r\n", 2.0);
    send(cfd2, ok.data(), ok.size(), MSG_NOSIGNAL);
    for (int i = 0; i < 10; ++i) {
      link.setPosition(position(32.0603, 118.7969));
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    const std::string gga2 = read_until(cfd2, "\r\n", 2.0);
    CHECK(gga2.find(",3203.61800,N,11847.81400,E,") != std::string::npos);
    close(cfd2);
  }
  close(cfd);
  link.stop();
  close(lfd);
}

static void test_ntrip_401_long_backoff() {
  int port = 0;
  const int lfd = listen_local(&port);
  CorrectionConfig c = base_cfg();
  c.ntrip.enabled = true;
  c.ntrip.host = "127.0.0.1";
  c.ntrip.port = port;
  c.ntrip.mountpoint = "AUTO";
  c.ntrip.user = "u";
  c.ntrip.password = "p";
  c.ntrip.backoff_auth_s = 30.0;
  CorrectionLink link(c);
  link.start();
  link.setPosition(position(31.2304, 121.4737));
  const int cfd = accept_within(lfd, 3.0);
  CHECK(cfd >= 0);
  if (cfd >= 0) {
    read_until(cfd, "\r\n\r\n", 2.0);
    const std::string no = "HTTP/1.0 401 Unauthorized\r\n\r\n";
    send(cfd, no.data(), no.size(), MSG_NOSIGNAL);
    close(cfd);
  }
  for (int i = 0; i < 30; ++i) {
    link.setPosition(position(31.2304, 121.4737));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  CHECK(accept_within(lfd, 0.5) < 0);         // not retried at the normal back-off (1 s)
  link.stop();
  close(lfd);
}

int main() {
  test_lan_primary();
  test_ntrip_cold_standby_and_rebase();
  test_ntrip_401_long_backoff();
  if (g_failures == 0) {
    std::printf("ALL CORRECTION LINK TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CORRECTION LINK TEST(S) FAILED\n", g_failures);
  return 1;
}
