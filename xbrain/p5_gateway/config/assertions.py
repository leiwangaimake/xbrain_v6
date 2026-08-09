"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: assertions.py
Brief: GWY-P5-17 p5_gateway.yaml assertions + hmi.bind per-port + startup.pending_keys

Description:
17 S17 freeze-time assertions for p5_gateway. Beyond the standard
A/B/C/J family shared with p3, p5 has three extra rules:

  P5-BIND-1  every hmi.bind entry is a fully-qualified 'host:port',
             no '0.0.0.0' anywhere (matches NET-C9 explicit-bind
             discipline). Port must be in [1024, 65535].
  P5-BIND-2  no port is reused across hmi.bind and other bind lists
             within the same config.
  P5-PEND-1  startup.pending_keys is a list of configs.* paths that
             p5 is allowed to receive at runtime (they may be
             initially null); every other null key remains fatal.
"""

from __future__ import annotations

import re
from typing import Iterable


class FreezeAssertionFailure(Exception):
    pass


BIND_RE = re.compile(r"^([\w\-.]+):(\d+)$")


def check_bind_entries(binds: Iterable[str]) -> None:
    """P5-BIND-1: no 0.0.0.0; port in [1024, 65535]."""
    for entry in binds:
        m = BIND_RE.match(entry)
        if m is None:
            raise FreezeAssertionFailure(
                f"P5-BIND-1: bad bind entry {entry!r}")
        host, port_s = m.group(1), m.group(2)
        if host in ("0.0.0.0", "::"):
            raise FreezeAssertionFailure(
                f"P5-BIND-1: bind '{host}' forbidden (NET-C9 "
                f"requires explicit per-port host)")
        port = int(port_s)
        if not (1024 <= port <= 65535):
            raise FreezeAssertionFailure(
                f"P5-BIND-1: port {port} out of range [1024, 65535]")


def check_bind_no_port_reuse(all_binds) -> None:
    """P5-BIND-2: no port reuse across bind lists."""
    ports = []
    for group_name, binds in all_binds.items():
        for entry in binds:
            m = BIND_RE.match(entry)
            if m is not None:
                ports.append((int(m.group(2)), group_name, entry))
    seen: dict = {}
    for port, group, entry in ports:
        if port in seen:
            raise FreezeAssertionFailure(
                f"P5-BIND-2: port {port} reused: "
                f"{seen[port]!r} vs {(group, entry)!r}")
        seen[port] = (group, entry)


def is_pending_key(key_path: str, pending_keys: Iterable[str]) -> bool:
    """P5-PEND-1: pending_keys is a whitelist for 'null OK at boot'."""
    return key_path in set(pending_keys)
