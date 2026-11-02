/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: correction_link.cc
 * Brief: RTCM correction network thread (see correction_link.h)
 *
 * Description:
 * One thread, one poll() loop with a 100 ms timeout, two sources. Each pass:
 * advance both connection state machines (timeouts, back-off, cold standby,
 * GGA), poll, read whatever arrived into each source's framer, feed the frames
 * to the source health, run the selector, and queue only the active source's
 * frames for the main loop.
 *
 * DNS for the caster runs in a DETACHED helper thread writing into a shared
 * state the loop polls. Not std::async: the future it returns blocks in its
 * destructor until the lookup ends, so a hung resolver would hang this thread.
 * An abandoned lookup simply finishes on its own and frees its state.
 *
 * Clocks: steady_clock only (CLK-C1). The GGA time field comes from the
 * receiver's own GGA time advanced by the monotonic time since, so no wall
 * clock is read here at all.
 */

#include "sensor/correction_link.h"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <utility>

#include "sensor/ntrip_protocol.h"

namespace sensor {

namespace {

double mono_now_s() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

constexpr std::size_t kMaxHeader = 4096;   // NTRIP reply header cap
constexpr int kPollMs = 100;               // also bounds how fast stop() is seen
constexpr std::size_t kReadChunk = 4096;
constexpr int kMaxReadsPerWake = 16;       // bounded work per source per pass

enum class LinkState { kIdle, kWaitPosition, kResolving, kConnecting, kRequestSent, kStreaming };

const char* state_name(LinkState s) {
  switch (s) {
    case LinkState::kIdle: return "idle";
    case LinkState::kWaitPosition: return "wait_position";
    case LinkState::kResolving: return "resolving";
    case LinkState::kConnecting: return "connecting";
    case LinkState::kRequestSent: return "request_sent";
    case LinkState::kStreaming: return "streaming";
  }
  return "?";
}

// Keepalive aligned with RTB-7 (idle 5 s, interval 2 s, 3 probes): a peer that
// vanished without a FIN is noticed by the kernel, not only by the read timeout.
void set_keepalive(int fd) {
  int on = 1, idle = 5, intvl = 2, cnt = 3;
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}

// Non-blocking connect; returns the fd (connect may still be in progress) or -1.
int start_connect(const sockaddr* addr, socklen_t len) {
  const int fd = socket(addr->sa_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) return -1;
  set_keepalive(fd);
  if (connect(fd, addr, len) != 0 && errno != EINPROGRESS) {
    close(fd);
    return -1;
  }
  return fd;
}

}  // namespace

struct CorrectionLink::DnsJob {
  std::mutex mu;
  bool done = false;
  bool ok = false;
  sockaddr_storage addr{};
  socklen_t len = 0;
};

struct CorrectionLink::Source {
  Source(const char* n, HealthConfig h) : name(n), health(h) {}
  void enter(LinkState s, double now) {
    state = s;
    t_state = now;
  }
  const char* name;
  LinkState state = LinkState::kIdle;
  int fd = -1;
  double next_try = 0.0;
  double backoff = 0.0;        // 0 = next failure uses backoff_min_s
  double t_state = 0.0;
  double last_rx = 0.0;
  Rtcm3Framer framer;
  SourceHealth health;
  std::string header;          // NTRIP reply accumulation
  bool chunked = false;
  ChunkedDecoder chunk;
  uint64_t bytes_rx = 0;
};

CorrectionLink::CorrectionLink(CorrectionConfig cfg)
    : cfg_(std::move(cfg)),
      lan_(new Source("lan", cfg_.health)),
      ntrip_(new Source("ntrip", cfg_.health)),
      selector_(cfg_.selector),
      anchor_(cfg_.ntrip.gga_max_pos_age_s, cfg_.ntrip.vrs_rebase_km) {}

CorrectionLink::~CorrectionLink() { stop(); }

void CorrectionLink::start() {
  if (!enabled() || thread_.joinable()) return;
  stop_ = false;
  thread_ = std::thread([this]() {
    try {
      run();
    } catch (...) {
      // run() already catches per pass; this is the last line before
      // std::terminate would take the whole process down (RTG-1 (3)).
      std::fprintf(stderr, "rtk_driver: rtcm link thread ended by an exception; "
                           "corrections stop, publishing continues\n");
    }
  });
}

void CorrectionLink::stop() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
}

void CorrectionLink::setPosition(const PositionSample& s) {
  std::lock_guard<std::mutex> lk(pos_mu_);
  pos_ = s;
}

void CorrectionLink::takeFrames(std::vector<TaggedFrame>* out) {
  std::deque<TaggedFrame> got;
  {
    std::lock_guard<std::mutex> lk(out_mu_);
    got.swap(out_);
    out_bytes_ = 0;
  }
  for (auto& f : got) out->push_back(std::move(f));
}

std::string CorrectionLink::statusLine() const {
  std::lock_guard<std::mutex> lk(status_mu_);
  return status_;
}

void CorrectionLink::run() {
  std::printf("rtk_driver: rtcm link started (lan=%s ntrip=%s)\n",
              cfg_.lan.enabled ? "on" : "off", cfg_.ntrip.enabled ? "on" : "off");
  std::fflush(stdout);
  while (!stop_) {
    try {
      loopOnce();
    } catch (const std::exception& e) {
      std::fprintf(stderr, "rtk_driver: rtcm link error: %s; resetting connections\n", e.what());
      closeAll(mono_now_s());
      for (int i = 0; i < 10 && !stop_; ++i) poll(nullptr, 0, kPollMs);
    } catch (...) {
      std::fprintf(stderr, "rtk_driver: rtcm link error (unknown); resetting connections\n");
      closeAll(mono_now_s());
      for (int i = 0; i < 10 && !stop_; ++i) poll(nullptr, 0, kPollMs);
    }
  }
  closeAll(mono_now_s());
}

void CorrectionLink::fail(Source& s, double now, double backoff_override, const char* why) {
  if (s.fd >= 0) close(s.fd);
  s.fd = -1;
  s.health.reset();
  s.framer.reset();
  s.header.clear();
  s.chunked = false;
  s.chunk = ChunkedDecoder();
  s.enter(LinkState::kIdle, now);
  double wait;
  if (backoff_override >= 0.0) {
    wait = backoff_override;
  } else {
    const bool is_lan = &s == lan_.get();
    const double lo = is_lan ? cfg_.lan.backoff_min_s : cfg_.ntrip.backoff_min_s;
    const double hi = is_lan ? cfg_.lan.backoff_max_s : cfg_.ntrip.backoff_max_s;
    s.backoff = (s.backoff <= 0.0) ? lo : std::min(s.backoff * 2.0, hi);
    wait = s.backoff;
  }
  s.next_try = now + wait;
  if (&s == ntrip_.get()) {
    anchor_.clear();
    last_gga_ = -1.0;
  }
  std::printf("rtk_driver: rtcm %s down (%s), retry in %.1f s\n", s.name, why, wait);
  std::fflush(stdout);
}

void CorrectionLink::closeAll(double now) {
  for (Source* s : {lan_.get(), ntrip_.get()}) {
    if (s->state != LinkState::kIdle || s->fd >= 0) fail(*s, now, -1.0, "reset");
  }
  dns_.reset();
}

void CorrectionLink::stepLan(double now) {
  if (!cfg_.lan.enabled) return;
  Source& s = *lan_;
  switch (s.state) {
    case LinkState::kIdle: {
      if (now < s.next_try) break;
      // Numeric IP only (RTB-1: the bridge has a static address): no DNS here.
      addrinfo hints{};
      hints.ai_family = AF_UNSPEC;
      hints.ai_socktype = SOCK_STREAM;
      hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
      addrinfo* res = nullptr;
      const std::string port = std::to_string(cfg_.lan.port);
      if (getaddrinfo(cfg_.lan.host.c_str(), port.c_str(), &hints, &res) != 0 || res == nullptr) {
        fail(s, now, -1.0, "lan.host is not a numeric address");
        break;
      }
      s.fd = start_connect(res->ai_addr, res->ai_addrlen);
      freeaddrinfo(res);
      if (s.fd < 0) {
        fail(s, now, -1.0, std::strerror(errno));
      } else {
        s.enter(LinkState::kConnecting, now);
      }
      break;
    }
    case LinkState::kConnecting:
      if (now - s.t_state > cfg_.lan.connect_timeout_s) fail(s, now, -1.0, "connect timeout");
      break;
    case LinkState::kStreaming:
      if (now - s.last_rx > cfg_.lan.read_timeout_s) fail(s, now, -1.0, "no data");
      break;
    default:
      break;
  }
}

void CorrectionLink::stepNtrip(double now, const PositionSample& pos) {
  if (!cfg_.ntrip.enabled) return;
  Source& s = *ntrip_;
  if (!selector_.wantBackup(now)) {
    // Cold standby (U-RTCM-2): not needed -> not connected, no back-off owed.
    if (s.state != LinkState::kIdle) {
      fail(s, now, 0.0, "backup not needed");
      s.backoff = 0.0;
      dns_.reset();
    }
    return;
  }
  switch (s.state) {
    case LinkState::kIdle:
      if (now >= s.next_try) s.enter(LinkState::kWaitPosition, now);
      break;
    case LinkState::kWaitPosition: {
      // RTG-7: dial only with a fresh measured fix to report. There is no
      // substitute position -- waiting is the designed behaviour.
      if (!anchor_.reportable(pos, now)) break;
      auto job = std::make_shared<DnsJob>();
      const std::string host = cfg_.ntrip.host;
      const std::string port = std::to_string(cfg_.ntrip.port);
      std::thread([job, host, port]() {
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* res = nullptr;
        const int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
        std::lock_guard<std::mutex> lk(job->mu);
        if (rc == 0 && res != nullptr && res->ai_addrlen <= sizeof(job->addr)) {
          std::memcpy(&job->addr, res->ai_addr, res->ai_addrlen);
          job->len = res->ai_addrlen;
          job->ok = true;
        }
        if (res != nullptr) freeaddrinfo(res);
        job->done = true;
      }).detach();
      dns_ = job;
      s.enter(LinkState::kResolving, now);
      break;
    }
    case LinkState::kResolving: {
      bool done = false, ok = false;
      sockaddr_storage addr{};
      socklen_t len = 0;
      if (dns_) {
        std::lock_guard<std::mutex> lk(dns_->mu);
        done = dns_->done;
        ok = dns_->ok;
        addr = dns_->addr;
        len = dns_->len;
      }
      if (!done) {
        if (now - s.t_state > cfg_.ntrip.connect_timeout_s) {
          dns_.reset();                         // abandon; the helper ends on its own
          fail(s, now, -1.0, "dns timeout");
        }
        break;
      }
      dns_.reset();
      if (!ok) {
        fail(s, now, -1.0, "dns failed");
        break;
      }
      s.fd = start_connect(reinterpret_cast<const sockaddr*>(&addr), len);
      if (s.fd < 0) {
        fail(s, now, -1.0, std::strerror(errno));
      } else {
        s.enter(LinkState::kConnecting, now);
      }
      break;
    }
    case LinkState::kConnecting:
      if (now - s.t_state > cfg_.ntrip.connect_timeout_s) fail(s, now, -1.0, "connect timeout");
      break;
    case LinkState::kRequestSent:
      if (now - s.t_state > cfg_.ntrip.read_timeout_s) fail(s, now, -1.0, "no reply");
      break;
    case LinkState::kStreaming: {
      if (now - s.last_rx > cfg_.ntrip.read_timeout_s) {
        fail(s, now, -1.0, "no data");
        break;
      }
      if (last_gga_ >= 0.0 && now - last_gga_ < cfg_.ntrip.gga_period_s) break;
      // A stale or missing fix is not reported (RTG-7): skip, try next pass.
      if (!anchor_.reportable(pos, now)) break;
      if (anchor_.needsRebase(pos.pos)) {
        // RTG-12: rebuild the virtual station where the robot now is.
        fail(s, now, 0.0, "moved beyond vrs_rebase_km from the VRS anchor");
        break;
      }
      double utc = pos.utc_s_of_day + (now - pos.t_mono);
      utc = std::fmod(utc, 86400.0);
      const std::string gga = build_gga(pos.pos, utc);
      const ssize_t n = send(s.fd, gga.data(), gga.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        fail(s, now, -1.0, "gga send failed");
        break;
      }
      if (!anchor_.hasAnchor()) anchor_.setAnchor(pos.pos);
      last_gga_ = now;
      break;
    }
  }
}

void CorrectionLink::onReadable(Source& s, bool is_ntrip, double now,
                                std::vector<Rtcm3Frame>* frames) {
  char buf[kReadChunk];
  for (int k = 0; k < kMaxReadsPerWake && s.fd >= 0; ++k) {
    const ssize_t n = recv(s.fd, buf, sizeof(buf), 0);
    if (n == 0) {
      fail(s, now, -1.0, "closed by peer");
      return;
    }
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) return;
      fail(s, now, -1.0, std::strerror(errno));
      return;
    }
    s.bytes_rx += static_cast<uint64_t>(n);
    s.last_rx = now;
    if (!is_ntrip || s.state == LinkState::kStreaming) {
      if (s.chunked) {
        std::string dec;
        if (!s.chunk.feed(buf, static_cast<std::size_t>(n), &dec)) {
          s.framer.feed(dec.data(), dec.size(), frames);
          fail(s, now, -1.0, "chunked stream ended");
          return;
        }
        s.framer.feed(dec.data(), dec.size(), frames);
      } else {
        s.framer.feed(buf, static_cast<std::size_t>(n), frames);
      }
      continue;
    }
    // NTRIP, waiting for the reply header.
    s.header.append(buf, static_cast<std::size_t>(n));
    const NtripReply r = parse_ntrip_reply(s.header, kMaxHeader);
    switch (r.kind) {
      case NtripReplyKind::kIncomplete:
        break;
      case NtripReplyKind::kStream:
      case NtripReplyKind::kStreamChunked: {
        std::printf("rtk_driver: rtcm ntrip connected (%s)\n", r.status_line.c_str());
        std::fflush(stdout);
        const std::string rest = s.header.substr(r.header_len);
        s.header.clear();
        s.chunked = r.kind == NtripReplyKind::kStreamChunked;
        s.enter(LinkState::kStreaming, now);
        s.backoff = 0.0;
        last_gga_ = -1.0;                       // GGA right away, next pass
        if (!rest.empty()) {
          if (s.chunked) {
            std::string dec;
            if (!s.chunk.feed(rest.data(), rest.size(), &dec)) {
              fail(s, now, -1.0, "chunked stream ended");
              return;
            }
            s.framer.feed(dec.data(), dec.size(), frames);
          } else {
            s.framer.feed(rest.data(), rest.size(), frames);
          }
        }
        break;
      }
      case NtripReplyKind::kSourceTable:
        std::printf("rtk_driver: rtcm ntrip mountpoint '%s' not offered by the caster\n",
                    cfg_.ntrip.mountpoint.c_str());
        fail(s, now, cfg_.ntrip.backoff_auth_s, "sourcetable");
        return;
      case NtripReplyKind::kUnauthorized:
        // User name only -- never the password (RTG-9).
        std::printf("rtk_driver: rtcm ntrip caster rejected user '%s'\n", cfg_.ntrip.user.c_str());
        fail(s, now, cfg_.ntrip.backoff_auth_s, "unauthorized");
        return;
      case NtripReplyKind::kError:
        std::printf("rtk_driver: rtcm ntrip unexpected reply '%s'\n", r.status_line.c_str());
        fail(s, now, -1.0, "bad reply");
        return;
    }
  }
}

void CorrectionLink::pushOut(const std::vector<Rtcm3Frame>& frames, int source, double now) {
  if (frames.empty()) return;
  std::lock_guard<std::mutex> lk(out_mu_);
  for (const auto& f : frames) {
    TaggedFrame t;
    t.bytes = f.bytes;
    t.source = source;
    t.t_mono = now;
    out_bytes_ += t.bytes.size();
    out_.push_back(std::move(t));
  }
  while (out_bytes_ > cfg_.queue_max_bytes && !out_.empty()) {
    out_bytes_ -= out_.front().bytes.size();
    out_.pop_front();
    ++out_dropped_;
  }
}

void CorrectionLink::publishStatus(double now) {
  auto one = [&](const Source& s, bool enabled) {
    char b[200];
    if (!enabled) return std::string("off");
    std::snprintf(b, sizeof(b), "%s/%s rx=%llu frames=%llu crc=%llu st=%d", state_name(s.state),
                  s.health.valid(now) ? "valid" : "invalid",
                  static_cast<unsigned long long>(s.bytes_rx),
                  static_cast<unsigned long long>(s.framer.frames_ok()),
                  static_cast<unsigned long long>(s.framer.crc_errors()), s.health.stationId());
    return std::string(b);
  };
  std::string line = std::string("rtcm active=") + active_source_name(selector_.active()) +
                     " lan=" + one(*lan_, cfg_.lan.enabled) +
                     " ntrip=" + one(*ntrip_, cfg_.ntrip.enabled);
  {
    std::lock_guard<std::mutex> lk(out_mu_);
    line += " out_dropped=" + std::to_string(out_dropped_);
  }
  std::lock_guard<std::mutex> lk(status_mu_);
  status_ = std::move(line);
}

void CorrectionLink::loopOnce() {
  const double now0 = mono_now_s();
  PositionSample pos;
  {
    std::lock_guard<std::mutex> lk(pos_mu_);
    pos = pos_;
  }
  stepLan(now0);
  stepNtrip(now0, pos);

  pollfd fds[2];
  Source* srcs[2];
  nfds_t n = 0;
  for (Source* s : {lan_.get(), ntrip_.get()}) {
    if (s->fd < 0) continue;
    fds[n].fd = s->fd;
    fds[n].events = (s->state == LinkState::kConnecting) ? POLLOUT : POLLIN;
    fds[n].revents = 0;
    srcs[n] = s;
    ++n;
  }
  if (n == 0) {
    poll(nullptr, 0, kPollMs);
  } else if (poll(fds, n, kPollMs) < 0 && errno != EINTR) {
    throw std::runtime_error(std::string("poll: ") + std::strerror(errno));
  }

  const double now = mono_now_s();
  std::vector<Rtcm3Frame> lan_frames, ntrip_frames;
  for (nfds_t i = 0; i < n; ++i) {
    if (fds[i].revents == 0) continue;
    Source& s = *srcs[i];
    const bool is_ntrip = &s == ntrip_.get();
    if (s.state == LinkState::kConnecting) {
      int err = 0;
      socklen_t len = sizeof(err);
      if (getsockopt(s.fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
        fail(s, now, -1.0, err != 0 ? std::strerror(err) : "connect failed");
        continue;
      }
      if (!is_ntrip) {
        std::printf("rtk_driver: rtcm lan connected to %s:%d\n", cfg_.lan.host.c_str(), cfg_.lan.port);
        std::fflush(stdout);
        s.enter(LinkState::kStreaming, now);
        s.last_rx = now;
        s.backoff = 0.0;
        continue;
      }
      NtripRequest q{cfg_.ntrip.version, cfg_.ntrip.host, cfg_.ntrip.port,
                     cfg_.ntrip.mountpoint, cfg_.ntrip.user, cfg_.ntrip.password};
      const std::string req = build_ntrip_request(q);
      const ssize_t sent = send(s.fd, req.data(), req.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
      if (sent != static_cast<ssize_t>(req.size())) {
        fail(s, now, -1.0, "request send failed");
        continue;
      }
      s.header.clear();
      s.enter(LinkState::kRequestSent, now);
      continue;
    }
    onReadable(s, is_ntrip, now, is_ntrip ? &ntrip_frames : &lan_frames);
  }

  // Frames read just before a failure in the same pass do not revive a source
  // whose connection is already closed: its health was reset by fail().
  if (lan_->fd < 0) lan_frames.clear();
  if (ntrip_->fd < 0) ntrip_frames.clear();
  for (const auto& f : lan_frames) lan_->health.onFrame(f, now);
  for (const auto& f : ntrip_frames) ntrip_->health.onFrame(f, now);
  const bool pv = cfg_.lan.enabled && lan_->health.valid(now);
  const bool bv = cfg_.ntrip.enabled && ntrip_->health.valid(now);
  const SelectorResult r = selector_.update(pv, bv, now);
  if (r.switched) {
    {
      std::lock_guard<std::mutex> lk(out_mu_);
      out_.clear();
      out_bytes_ = 0;
    }
    std::printf("rtk_driver: rtcm source -> %s\n", active_source_name(r.active));
    std::fflush(stdout);
  }
  if (r.active == ActiveSource::kPrimary) {
    pushOut(lan_frames, static_cast<int>(ActiveSource::kPrimary), now);
  } else if (r.active == ActiveSource::kBackup) {
    pushOut(ntrip_frames, static_cast<int>(ActiveSource::kBackup), now);
  }
  publishStatus(now);
}

}  // namespace sensor
