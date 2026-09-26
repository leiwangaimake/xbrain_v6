/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chassis_socket.cc
 * Brief: The socket itself (see chassis_socket.h)
 *
 * Description:
 * Four decisions in here are worth more than the rest of the file:
 *
 *   * O_NONBLOCK is set BEFORE connect, not after. A blocking connect to an
 *     unreachable host takes the kernel's SYN timeout -- on the order of two
 *     minutes -- and it would take it on whichever thread dialled. The probe in
 *     chs_a_session budgets two SECONDS per candidate, so a blocking connect
 *     would blow through the whole candidate list's budget on the first entry.
 *   * EINPROGRESS from a non-blocking TCP connect is SUCCESS, not failure. The
 *     handshake completes in the background and the first send tells us whether
 *     it worked. Treating it as failure would reject every TCP candidate on a
 *     healthy network.
 *   * a UDP socket is connect()ed too. It costs nothing, and it buys two
 *     things: send() without repeating the address on every frame, and ICMP
 *     port-unreachable surfacing as an error on a later call instead of
 *     vanishing. Without it a UDP "connection" to a dead port looks identical
 *     to one to a live silent port.
 *   * SIGPIPE is suppressed per-call with MSG_NOSIGNAL rather than globally.
 *     A library that installs a process-wide signal disposition takes it away
 *     from the owner, and writing to a closed socket must return EPIPE here --
 *     not kill the process.
 */

#include "quadruped/chassis_socket.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace quadruped {

const char* DialErrorName(DialError e) {
  switch (e) {
    case DialError::kNone: return "none";
    case DialError::kUnsupportedProto: return "unsupported_proto";
    case DialError::kAddressInvalid: return "address_invalid";
    case DialError::kSocketFailed: return "socket_failed";
    case DialError::kConnectFailed: return "connect_failed";
    case DialError::kTlsNotBuilt: return "tls_not_built";
  }
  return "invalid";
}

ChassisSocket::ChassisSocket() = default;

ChassisSocket::~ChassisSocket() { Close(); }

void ChassisSocket::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool ChassisSocket::Dial(const EndpointCandidate& ep, bool tcp_nodelay) {
  // CA-1: never two live sockets. A second one is a second CLIENT to the
  // chassis, and axis commands come back 0xE006 for two seconds -- accepted,
  // and the robot does not move.
  Close();
  last_error_ = DialError::kNone;
  last_errno_ = 0;

  if (ep.tls) {
    // 13 TLS-1 / TLS-5. The encrypted candidates stay disabled until the vendor
    // issues client certificates, and this build carries no TLS at all. Failing
    // is the point: falling back to plaintext on a candidate marked tls:true
    // would be a downgrade that happens by itself, and a downgrade that happens
    // by itself can be forced by somebody else.
    last_error_ = DialError::kTlsNotBuilt;
    return false;
  }

  if (ep.proto == "udp") {
    is_udp_ = true;
  } else if (ep.proto == "tcp") {
    is_udp_ = false;
  } else {
    last_error_ = DialError::kUnsupportedProto;
    return false;
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(ep.port));
  if (::inet_pton(AF_INET, ep.host.c_str(), &addr.sin_addr) != 1) {
    last_error_ = DialError::kAddressInvalid;
    return false;
  }

  // SOCK_NONBLOCK in the socket() call, so the flag is set BEFORE connect and
  // there is no window in which a blocking connect could start. Setting it
  // afterwards with fcntl is the common form and leaves exactly that window.
  const int type = (is_udp_ ? SOCK_DGRAM : SOCK_STREAM) | SOCK_NONBLOCK;
  fd_ = ::socket(AF_INET, type, 0);
  if (fd_ < 0) {
    last_error_ = DialError::kSocketFailed;
    last_errno_ = errno;
    return false;
  }

  if (!is_udp_ && tcp_nodelay) {
    // FR-5 / SD-3. Nagle would batch a heartbeat with whatever followed, and
    // then "when did the last frame leave" has no answer -- which is the
    // question every latency figure in 13 S3.6 rests on.
    int one = 1;
    // *** The return value was DISCARDED, and nothing else checked the
    // result either -- so FR-5 / SD-3's "TCP_NODELAY = 1" was a guarantee
    // nobody held (CLAUDE.md S3.2: "assuming a guarantee you do not have").
    // A silent failure here does not break the link; it makes Nagle batch the
    // heartbeat with whatever follows, and every latency figure in 13 S3.6
    // then measures something else.
    //
    // Recorded rather than fatal: the link still works, badly, and refusing to
    // connect over a timing degradation would take Tier 1 down with it.
    nodelay_requested_ = true;
    nodelay_ok_ =
        ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0;
  }

  if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    // EINPROGRESS is the normal answer for a non-blocking TCP connect: the
    // handshake is running and the first send will say whether it completed.
    // Treating it as a failure would reject every TCP candidate on a healthy
    // network.
    if (errno != EINPROGRESS) {
      last_error_ = DialError::kConnectFailed;
      last_errno_ = errno;
      Close();
      return false;
    }
  }
  return true;
}

long ChassisSocket::Send(const std::uint8_t* data, std::size_t len) {
  if (fd_ < 0 || data == nullptr) return -1;
  // MSG_NOSIGNAL per call rather than a process-wide SIGPIPE disposition: a
  // library that installs one takes the choice away from the process that owns
  // it. Writing to a closed socket must return EPIPE here, not end the process.
  const ssize_t n = ::send(fd_, data, len, MSG_NOSIGNAL);
  if (n >= 0) return static_cast<long>(n);
  // EAGAIN means the send buffer is full right now, which is a legitimate short
  // write and not an error -- 0 tells TxOwner to run its bounded completion
  // loop (FR-4) rather than to tear the connection down.
  if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
  return -1;
}

long ChassisSocket::Recv(std::uint8_t* out, std::size_t cap) {
  if (fd_ < 0 || out == nullptr || cap == 0) return -1;
  const ssize_t n = ::recv(fd_, out, cap, 0);
  if (n > 0) return static_cast<long>(n);
  if (n == 0) {
    // A TCP peer closed the connection. On UDP a zero-length datagram is legal
    // and means nothing arrived to interpret, so the two are told apart: a TCP
    // zero is fatal and a UDP zero is not.
    return is_udp_ ? 0 : -1;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
  return -1;
}

bool ChassisSocket::nodelay_enabled() const {
  if (fd_ < 0 || is_udp_) return false;
  int v = 0;
  socklen_t len = sizeof(v);
  if (::getsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &v, &len) != 0) return false;
  return v != 0;
}

int ChassisSocket::sndbuf_bytes() const {
  // 13 SD-1. Unlike TCP_NODELAY this option exists on datagram sockets too --
  // SD-1's UDP clause is precisely "SO_SNDBUF stays at the default" -- so
  // there is no is_udp_ guard here. -1 says "no socket", which a caller must
  // not confuse with a zero-byte buffer (the kernel never reports 0).
  if (fd_ < 0) return -1;
  int v = 0;
  socklen_t len = sizeof(v);
  if (::getsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &v, &len) != 0) return -1;
  return v;
}

int ChassisSocket::local_port() const {
  if (fd_ < 0) return -1;
  sockaddr_in addr;
  socklen_t len = sizeof(addr);
  if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
    return -1;
  }
  return static_cast<int>(ntohs(addr.sin_port));
}

}  // namespace quadruped
