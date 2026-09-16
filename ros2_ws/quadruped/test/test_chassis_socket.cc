/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_chassis_socket.cc
 * Brief: Real sockets against an in-process loopback server
 *
 * Description:
 * These are real system calls against a real listener on 127.0.0.1, not a fake.
 * A mock of a socket is a mock of the thing whose behaviour is in question:
 * EAGAIN, EINPROGRESS, a zero-length read meaning two different things on TCP
 * and UDP -- every one of those is a property of the kernel, and a test double
 * would encode whatever the author believed instead of what happens.
 *
 * The cases that are easy to get wrong and are asserted here:
 *
 *   * a non-blocking TCP connect returns EINPROGRESS, and that is SUCCESS. An
 *     implementation that treated it as failure would reject every TCP
 *     candidate on a healthy network -- and would look like "the chassis is
 *     not answering".
 *   * dialling twice CLOSES the first socket (CA-1). The stub sees the first
 *     connection end, which is the only way to observe it from outside. Two
 *     live sockets are two CLIENTS to the chassis, and axis commands then come
 *     back 0xE006 for two seconds: accepted, and the robot does not move.
 *   * a TLS candidate FAILS rather than quietly dialling plaintext. TLS-5
 *     forbids an automatic downgrade, because a downgrade that happens by
 *     itself can be forced by somebody else.
 *   * Recv with nothing waiting returns 0 and not -1. The receive loop treats
 *     -1 as a dead link and reconnects; getting that backwards turns an idle
 *     moment into a reconnect storm, and 13 CA-6 makes every reconnect play a
 *     voice prompt on the robot.
 */

#include "quadruped/chassis_socket.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using namespace quadruped;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// A listener on an ephemeral loopback port. Port 0 lets the kernel choose, so
// the test cannot collide with anything else on the machine -- a fixed port
// would make this fail on a developer's box for reasons unrelated to the code.
class TcpStub {
 public:
  TcpStub() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a;
    std::memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    ::listen(fd_, 4);
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    port_ = ntohs(a.sin_port);
  }
  ~TcpStub() {
    if (conn_ >= 0) ::close(conn_);
    if (fd_ >= 0) ::close(fd_);
  }
  int port() const { return port_; }
  // Accept the next connection, replacing any previous one.
  bool Accept() {
    if (conn_ >= 0) ::close(conn_);
    conn_ = ::accept(fd_, nullptr, nullptr);
    return conn_ >= 0;
  }
  long Read(std::uint8_t* out, std::size_t cap) {
    return conn_ < 0 ? -1 : ::recv(conn_, out, cap, 0);
  }
  long Write(const std::uint8_t* d, std::size_t n) {
    return conn_ < 0 ? -1 : ::send(conn_, d, n, MSG_NOSIGNAL);
  }
  int conn_fd() const { return conn_; }

 private:
  int fd_ = -1;
  int conn_ = -1;
  int port_ = 0;
};

class UdpStub {
 public:
  UdpStub() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a;
    std::memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    port_ = ntohs(a.sin_port);
  }
  ~UdpStub() {
    if (fd_ >= 0) ::close(fd_);
  }
  int port() const { return port_; }
  long RecvFrom(std::uint8_t* out, std::size_t cap) {
    peer_len_ = sizeof(peer_);
    return ::recvfrom(fd_, out, cap, 0, reinterpret_cast<sockaddr*>(&peer_),
                      &peer_len_);
  }
  long SendBack(const std::uint8_t* d, std::size_t n) {
    return ::sendto(fd_, d, n, 0, reinterpret_cast<sockaddr*>(&peer_), peer_len_);
  }

 private:
  int fd_ = -1;
  int port_ = 0;
  sockaddr_in peer_{};
  socklen_t peer_len_ = 0;
};

EndpointCandidate Ep(const char* proto, int port, bool tls = false) {
  EndpointCandidate e;
  e.proto = proto;
  e.host = "127.0.0.1";
  e.port = port;
  e.tls = tls;
  e.enabled = true;
  return e;
}

// Poll a non-blocking read until it produces something or the budget runs out.
// A fixed sleep would be either flaky or slow; this is bounded and usually
// returns on the first pass.
long ReadWithin(ChassisSocket* s, std::uint8_t* out, std::size_t cap, int ms) {
  for (int i = 0; i < ms; ++i) {
    const long n = s->Recv(out, cap);
    if (n != 0) return n;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return 0;
}

}  // namespace

int main() {
  // ---- TCP: dial, send, receive ------------------------------------------
  {
    TcpStub stub;
    ChassisSocket s;
    // A non-blocking connect to a live loopback listener returns either 0 or
    // EINPROGRESS; both are success. Rejecting EINPROGRESS would fail every
    // TCP candidate on a healthy network.
    CHECK(s.Dial(Ep("tcp", stub.port()), /*tcp_nodelay=*/true));
    CHECK(s.is_open());
    CHECK(s.is_udp() == false);
    CHECK(stub.Accept());

    // *** FR-5 / SD-3: Nagle off, READ BACK from the socket.
    //
    // The earlier version of this case sent one small frame and asserted it
    // arrived promptly, with a comment claiming that was the observable
    // consequence. It was not: Nagle only withholds a small write while an
    // earlier segment is unacknowledged, so a lone write on an idle connection
    // arrives immediately either way. That assertion discriminated nothing and
    // the mutant which removed the setsockopt survived it.
    CHECK(s.nodelay_enabled() == true);
    const std::uint8_t frame[] = {0xEB, 0x91, 0xEB, 0x90, 0x01, 0x02};
    CHECK(s.Send(frame, sizeof(frame)) == static_cast<long>(sizeof(frame)));
    std::uint8_t got[64];
    const long n = stub.Read(got, sizeof(got));
    CHECK(n == static_cast<long>(sizeof(frame)));
    CHECK(std::memcmp(got, frame, sizeof(frame)) == 0);

    // ...and back the other way.
    const std::uint8_t reply[] = {0xEB, 0x91, 0xEB, 0x90, 0x09};
    CHECK(stub.Write(reply, sizeof(reply)) == static_cast<long>(sizeof(reply)));
    std::uint8_t rx[64];
    CHECK(ReadWithin(&s, rx, sizeof(rx), 500) == static_cast<long>(sizeof(reply)));
    CHECK(std::memcmp(rx, reply, sizeof(reply)) == 0);

    // A local port was assigned, and it is what the chassis will answer to
    // (13 S2.2: reports go to whoever sends heartbeats).
    CHECK(s.local_port() > 0);
  }

  // ---- nothing waiting is 0, not -1 --------------------------------------
  {
    // The receive loop treats -1 as a dead link and reconnects. Getting this
    // backwards turns every idle moment into a reconnect, and 13 CA-6 makes
    // each reconnect play a voice prompt and change the LEDs on the robot.
    TcpStub stub;
    ChassisSocket s;
    CHECK(s.Dial(Ep("tcp", stub.port()), true));
    CHECK(stub.Accept());
    std::uint8_t buf[32];
    CHECK(s.Recv(buf, sizeof(buf)) == 0);
    CHECK(s.Recv(buf, sizeof(buf)) == 0);   // and it stays 0
  }

  // ---- a peer that goes away is -1, and Send does not kill the process ----
  {
    TcpStub* stub = new TcpStub();
    ChassisSocket s;
    CHECK(s.Dial(Ep("tcp", stub->port()), true));
    CHECK(stub->Accept());
    const std::uint8_t hello[] = {1, 2, 3};
    CHECK(s.Send(hello, sizeof(hello)) == 3);
    // *** The far end READS before closing, so the close is orderly and recv
    // returns 0. Closing with unread data in the buffer sends an RST instead,
    // and recv then returns ECONNRESET -- which takes the n < 0 path and never
    // reaches the zero-length branch this case exists to check. The mutant that
    // collapsed the TCP and UDP meanings of a zero-length read survived the
    // version without these two lines.
    std::uint8_t drain[8];
    CHECK(stub->Read(drain, sizeof(drain)) == 3);
    delete stub;                      // the far end closes, orderly

    std::uint8_t buf[32];
    // A TCP zero-length read means the peer closed, which is fatal for the
    // connection -- distinct from "nothing right now".
    CHECK(ReadWithin(&s, buf, sizeof(buf), 500) == -1);
    // And writing into it returns an error rather than raising SIGPIPE. Without
    // MSG_NOSIGNAL this line ends the process, and the test would not report a
    // failure -- it would vanish.
    for (int i = 0; i < 4; ++i) (void)s.Send(hello, sizeof(hello));
    CHECK(true);                      // reaching here at all is the assertion
  }

  // ---- nodelay_enabled reports the SOCKET, not the request ---------------
  {
    // Asserting only the true case passes on an accessor that returns true
    // unconditionally, which would make the self-report a statement about the
    // code's intention rather than about the kernel. Both directions, plus the
    // two shapes where the option does not exist at all.
    TcpStub stub;
    ChassisSocket on;
    CHECK(on.Dial(Ep("tcp", stub.port()), /*tcp_nodelay=*/true));
    CHECK(stub.Accept());
    CHECK(on.nodelay_enabled() == true);

    ChassisSocket off;
    CHECK(off.Dial(Ep("tcp", stub.port()), /*tcp_nodelay=*/false));
    CHECK(stub.Accept());
    CHECK(off.nodelay_enabled() == false);

    // UDP has no such option, and a closed socket has no option at all.
    UdpStub ustub;
    ChassisSocket u;
    CHECK(u.Dial(Ep("udp", ustub.port()), true));
    CHECK(u.nodelay_enabled() == false);
    ChassisSocket closed;
    CHECK(closed.nodelay_enabled() == false);
  }

  // ---- a full send buffer is a SHORT WRITE, not a broken link -------------
  {
    // EAGAIN means "not right now". The bounded completion loop of FR-4 exists
    // to finish the frame inside the same critical section; reporting an error
    // instead would tear down a perfectly good connection the first time the
    // chassis read a little slowly.
    TcpStub stub;
    ChassisSocket s;
    CHECK(s.Dial(Ep("tcp", stub.port()), true));
    CHECK(stub.Accept());
    // The stub never reads, so both socket buffers fill. A megabyte is well
    // past any default, and the loop stops at the first short write rather
    // than assuming which attempt it lands on.
    std::uint8_t block[65536];
    std::memset(block, 0xA5, sizeof(block));
    // *** Keep going until Send returns exactly ZERO, not merely until it
    // returns less than it was asked for. On a stream socket a partially full
    // buffer produces a PARTIAL write (n > 0); EAGAIN (n == 0) only appears
    // once the buffer is completely full. Stopping at the first partial write
    // never reaches the EAGAIN branch at all, and the mutant that turned EAGAIN
    // into an error survived that version of this case.
    bool saw_eagain = false;
    for (int i = 0; i < 512 && !saw_eagain; ++i) {
      const long n = s.Send(block, sizeof(block));
      CHECK(n >= 0);                  // never an error: the link is healthy
      if (n == 0) saw_eagain = true;
    }
    CHECK(saw_eagain);
    // ...and the socket is still usable: a short write is not a close.
    CHECK(s.is_open());
  }

  // ---- CA-1: dialling again closes the first socket -----------------------
  {
    // Two live sockets are two CLIENTS to the chassis. The only way to observe
    // the close from outside is the far end seeing its connection end, so the
    // stub keeps the first connection and reads it after the second dial.
    TcpStub stub;
    ChassisSocket s;
    CHECK(s.Dial(Ep("tcp", stub.port()), true));
    CHECK(stub.Accept());
    const int first_conn = stub.conn_fd();
    const int first_port = s.local_port();

    CHECK(s.Dial(Ep("tcp", stub.port()), true));
    const int second_port = s.local_port();
    // A different source port means a different socket, which is necessary but
    // not sufficient -- the old one could still be open.
    CHECK(second_port != first_port);
    // The first connection is gone: a read on it returns 0 (orderly close).
    std::uint8_t buf[8];
    long r = -2;
    for (int i = 0; i < 500 && r != 0; ++i) {
      r = ::recv(first_conn, buf, sizeof(buf), MSG_DONTWAIT);
      if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        r = -2;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    CHECK(r == 0);
  }

  // ---- UDP round trip -----------------------------------------------------
  {
    UdpStub stub;
    ChassisSocket s;
    CHECK(s.Dial(Ep("udp", stub.port()), /*tcp_nodelay=*/true));
    CHECK(s.is_udp() == true);
    const std::uint8_t frame[] = {0xEB, 0x91, 0xEB, 0x90, 0x55};
    CHECK(s.Send(frame, sizeof(frame)) == static_cast<long>(sizeof(frame)));
    std::uint8_t got[64];
    CHECK(stub.RecvFrom(got, sizeof(got)) == static_cast<long>(sizeof(frame)));
    const std::uint8_t reply[] = {0xEB, 0x91, 0xEB, 0x90, 0x66, 0x77};
    CHECK(stub.SendBack(reply, sizeof(reply)) == static_cast<long>(sizeof(reply)));
    std::uint8_t rx[64];
    CHECK(ReadWithin(&s, rx, sizeof(rx), 500) == static_cast<long>(sizeof(reply)));
    CHECK(std::memcmp(rx, reply, sizeof(reply)) == 0);
  }

  // ---- the refusals ------------------------------------------------------
  {
    ChassisSocket s;
    // *** TLS-5: a candidate marked tls:true FAILS. It must not quietly dial
    // plaintext -- a downgrade that happens by itself can be forced.
    CHECK(s.Dial(Ep("tcp", 30003, /*tls=*/true), true) == false);
    CHECK(s.last_error() == DialError::kTlsNotBuilt);
    CHECK(s.is_open() == false);      // and no socket was left behind

    CHECK(s.Dial(Ep("sctp", 30003), true) == false);
    CHECK(s.last_error() == DialError::kUnsupportedProto);

    EndpointCandidate bad = Ep("tcp", 30003);
    bad.host = "not-an-address";
    CHECK(s.Dial(bad, true) == false);
    CHECK(s.last_error() == DialError::kAddressInvalid);
    CHECK(s.is_open() == false);
  }

  // ---- a closed port fails rather than appearing to connect --------------
  {
    // The stub is destroyed, so nothing is listening on its port any more. On
    // loopback the refusal is immediate even for a non-blocking connect.
    int dead_port = 0;
    {
      TcpStub tmp;
      dead_port = tmp.port();
    }
    ChassisSocket s;
    const bool ok = s.Dial(Ep("tcp", dead_port), true);
    if (ok) {
      // Some kernels report the refusal on the first send instead of on
      // connect. Either is acceptable; what must NOT happen is a socket that
      // reads and writes as though connected.
      const std::uint8_t b[] = {1};
      long sent = s.Send(b, 1);
      std::uint8_t rx[8];
      const long got = ReadWithin(&s, rx, sizeof(rx), 200);
      CHECK(sent < 0 || got == -1);
    } else {
      CHECK(s.last_error() == DialError::kConnectFailed);
    }
  }

  // ---- operations on a closed socket are errors, not crashes -------------
  {
    ChassisSocket s;
    std::uint8_t buf[8] = {0};
    CHECK(s.is_open() == false);
    CHECK(s.Send(buf, sizeof(buf)) == -1);
    CHECK(s.Recv(buf, sizeof(buf)) == -1);
    CHECK(s.local_port() == -1);
    CHECK(s.Send(nullptr, 4) == -1);
    CHECK(s.Recv(nullptr, 4) == -1);
    s.Close();                        // idempotent
    s.Close();
  }

  // ---- the error names are distinct --------------------------------------
  {
    const std::string n[] = {
        DialErrorName(DialError::kNone), DialErrorName(DialError::kUnsupportedProto),
        DialErrorName(DialError::kAddressInvalid), DialErrorName(DialError::kSocketFailed),
        DialErrorName(DialError::kConnectFailed), DialErrorName(DialError::kTlsNotBuilt)};
    for (int i = 0; i < 6; ++i) {
      CHECK(!n[i].empty());
      for (int j = i + 1; j < 6; ++j) CHECK(n[i] != n[j]);
    }
  }

  if (g_failures == 0) {
    std::printf("ALL CHASSIS_SOCKET TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHASSIS_SOCKET TEST(S) FAILED\n", g_failures);
  return 1;
}
