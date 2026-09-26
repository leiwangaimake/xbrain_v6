/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chassis_socket.h
 * Brief: The one socket channel one owns -- TCP or UDP, non-blocking (13 CA-1)
 *
 * Description:
 * B2 built the link POLICY and deliberately left the socket out. This is the
 * socket, and it is deliberately small: everything that can be decided without
 * one already lives in chs_a_session, so what remains here is system calls and
 * the four options that matter.
 *
 * CA-1 is the reason this is a long-lived object rather than a function that
 * opens a connection per send. The chassis identifies its client by source
 * address AND port; a new socket is a new client, and axis commands then come
 * back 0xE006 for two seconds. The robot accepts the commands and does not
 * move, which reads as a mechanical fault. CA-2 puts the heartbeat on this same
 * socket for the same reason, and CA-6 adds a second one: the chassis plays a
 * voice prompt and changes its LEDs on every connect, so a reconnect loop is
 * audible in the room.
 *
 * Everything is non-blocking. The ctrl thread runs at 100 Hz on SCHED_FIFO 80
 * and calls Send from inside the tx critical section (13 TX-7); one blocking
 * write there stalls the control loop for as long as the far end takes, and the
 * Tier 1 deadline is 200 ms.
 *
 * TCP_NODELAY is set because Nagle would batch a 109-byte heartbeat with
 * whatever came next (FR-5 / SD-3). The visible effect is not slowness: it is
 * that "when did the last frame leave" stops having an answer, which is the
 * question every latency measurement in 13 S3.6 is built on.
 *
 * Boundary: bytes in, bytes out. It does not frame (chs_a_framer), does not
 * decide when to reconnect (chs_a_session), and does not serialise (chs_a_codec).
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHASSIS_SOCKET_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHASSIS_SOCKET_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "quadruped/quadruped_config.h"

namespace quadruped {

// Why a connect failed. The caller reports it, and the three cases have
// genuinely different remedies -- which is why they are not one bool.
enum class DialError {
  kNone,
  kUnsupportedProto,   // the config names something other than tcp or udp
  kAddressInvalid,     // the host does not parse as an address
  kSocketFailed,       // socket() itself failed: an fd limit, usually
  kConnectFailed,      // refused, unreachable, or timed out
  kTlsNotBuilt,        // the candidate asks for TLS and this build has none
};

const char* DialErrorName(DialError e);

class ChassisSocket {
 public:
  ChassisSocket();
  ~ChassisSocket();

  ChassisSocket(const ChassisSocket&) = delete;
  ChassisSocket& operator=(const ChassisSocket&) = delete;

  // Open a connection to one candidate. Returns false and leaves last_error()
  // set on failure. Closes any previous connection first: two live sockets
  // would be two clients to the chassis (CA-1).
  //
  // A TLS candidate always fails with kTlsNotBuilt. 13 TLS-1 leaves the
  // encrypted candidates disabled until the vendor issues client certificates,
  // and a build that silently fell back to plaintext on a candidate marked
  // tls:true would be a downgrade nobody asked for -- TLS-5 forbids exactly
  // that, because a downgrade that happens by itself can be forced.
  bool Dial(const EndpointCandidate& ep, bool tcp_nodelay);

  void Close();
  bool is_open() const { return fd_ >= 0; }

  // Write bytes. Returns the count written, 0 on EAGAIN (the socket is full
  // right now), or -1 on a real error. Never blocks.
  //
  // This is the FrameWriter TxOwner takes: the bounded short-write completion
  // of FR-4 happens there, inside the critical section, not here.
  long Send(const std::uint8_t* data, std::size_t len);

  // Read whatever is available. Returns the count, 0 when there is nothing
  // right now, or -1 on a closed or broken connection. Never blocks.
  long Recv(std::uint8_t* out, std::size_t cap);

  DialError last_error() const { return last_error_; }
  // The errno from the failing call, so a message can say "connection refused"
  // rather than "connect failed".
  int last_errno() const { return last_errno_; }
  bool is_udp() const { return is_udp_; }

  // The local port the kernel assigned. The chassis reports state to whichever
  // address sends heartbeats (13 S2.2), so this number is what it will answer
  // to -- worth printing once at startup, and worth NOT changing afterwards.
  int local_port() const;

  // Whether TCP_NODELAY is actually in force, read back from the socket.
  // Returns false on a UDP or closed socket, where the option does not exist.
  //
  // It is read back rather than remembered because setsockopt can be refused,
  // and because a remembered flag is a record of what was ASKED for. The same
  // reasoning as 13 DDS-9's startup self-report: the interesting value is the
  // one the kernel has, not the one the code believes it set.
  bool nodelay_enabled() const;
  // Whether TCP_NODELAY was ASKED FOR on this connection, and whether the
  // setsockopt succeeded. Kept apart from nodelay_enabled() above, which reads
  // the option BACK from the kernel: "we asked and it said yes" and "the
  // kernel reports it on" are two claims, and FR-5 / SD-3 needs the second.
  bool nodelay_requested() const { return nodelay_requested_; }
  bool nodelay_setopt_ok() const { return nodelay_ok_; }

  // SO_SNDBUF as the kernel holds it, or -1 on a closed socket. 13 SD-1: the
  // axis-command socket must NOT be given a large send buffer -- a dead
  // process with five full-speed frames queued in the kernel is a robot that
  // keeps driving for another 100 ms. This code never CALLS setsockopt for
  // it (grep: there is no SO_SNDBUF store anywhere), so the value is
  // REPORTED, not judged: the kernel default differs per platform, SD-1
  // forbids us enlarging it, and the read-back is what puts the actual
  // number in the bench ledger. Same read-back-not-remembered reasoning as
  // nodelay_enabled() above.
  int sndbuf_bytes() const;

 private:
  int fd_ = -1;
  bool is_udp_ = false;
  bool nodelay_requested_ = false;
  bool nodelay_ok_ = false;
  DialError last_error_ = DialError::kNone;
  int last_errno_ = 0;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHASSIS_SOCKET_H_
