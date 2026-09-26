/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: sd_notify.h
 * Brief: Hand-rolled sd_notify(3) -- READY/WATCHDOG datagrams, no libsystemd
 *
 * Description:
 * CRL-4 puts this unit under "systemd Restart=always + WatchdogSec", and the
 * unit file's own note (deploy/systemd/xbrain-chassis-relay.service) records
 * why the watchdog was DEFERRED rather than dropped: WatchdogSec without a
 * process that sends WATCHDOG=1 kills the process at every timeout (measured
 * 2026-08-16 -- a restart loop on the emergency-stop relay). This header is
 * the missing half: with it the unit can carry Type=notify + WatchdogSec.
 *
 * Why not link libsystemd: the protocol is one datagram to the socket named
 * by $NOTIFY_SOCKET (sd_notify(3) documents it as stable), and a dependency
 * on the emergency-stop binary buys nothing for ten lines of sendto. The
 * abstract-namespace form (leading '@') is handled because systemd uses it
 * on some configurations.
 *
 * Failure posture: returns false and does nothing when $NOTIFY_SOCKET is
 * absent -- a manual foreground run is not an error. A send failure under
 * systemd is ALSO survivable by design: the watchdog then restarts the
 * process, which is the supervised recovery CRL-4 asks for, not a fault to
 * be handled here.
 *
 * Threading: called from the housekeeping thread only. One socket per call
 * keeps the helper stateless; at 1 Hz the cost is noise.
 */

#ifndef HACHIST_XBRAIN_V6_CHASSIS_RELAY_SD_NOTIFY_H_
#define HACHIST_XBRAIN_V6_CHASSIS_RELAY_SD_NOTIFY_H_

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>

namespace chassis_relay {

// Send one state string ("READY=1", "WATCHDOG=1") to $NOTIFY_SOCKET.
// True when the datagram went out; false when there is no socket (normal
// outside systemd) or the send failed (systemd's watchdog handles it).
inline bool SdNotify(const char* state) {
  const char* path = std::getenv("NOTIFY_SOCKET");
  if (path == nullptr || path[0] == '\0' || state == nullptr) return false;

  sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  const std::size_t plen = std::strlen(path);
  if (plen == 0 || plen >= sizeof(addr.sun_path)) return false;
  std::memcpy(addr.sun_path, path, plen);
  // Abstract namespace: systemd spells it with a leading '@', the kernel
  // with a leading NUL.
  if (addr.sun_path[0] == '@') addr.sun_path[0] = '\0';

  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) return false;
  const std::size_t addr_len =
      offsetof(sockaddr_un, sun_path) + plen;  // exact, no trailing NUL
  const ssize_t n =
      ::sendto(fd, state, std::strlen(state), MSG_NOSIGNAL,
               reinterpret_cast<const sockaddr*>(&addr),
               static_cast<socklen_t>(addr_len));
  ::close(fd);
  return n == static_cast<ssize_t>(std::strlen(state));
}

}  // namespace chassis_relay

#endif  // HACHIST_XBRAIN_V6_CHASSIS_RELAY_SD_NOTIFY_H_
