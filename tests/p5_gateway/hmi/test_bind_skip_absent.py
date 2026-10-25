"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_bind_skip_absent.py
Brief: HMI binding skips an address this host does not own, so a down wifi never takes the LAN2 face with it

Description:
The ruling (99 U92, 2026-10-25): LAN2 (192.168.188.0/24, the party-A ad-hoc
network) is the delivery network; wifi is a debug port kept bound during the
transition, and "wifi failure must not affect the 188 network". hmi.bind lists
both. Before this change make_bound_sockets bound the entries one by one and
let the first OSError escape, so a robot that booted with wifi down -- its
wifi IP absent from every interface -- could not serve the HMI on LAN2 either.

What these cases pin:
  * an entry whose address is not on this host (EADDRNOTAVAIL) is skipped,
    and the remaining entries are served;
  * when EVERY entry is absent the HMI still refuses to start (HmiBindError),
    exactly like the all-null case -- an HMI reachable from nowhere is a
    config bug, and skipping must never end in "serve nothing silently";
  * any OTHER bind error (port already taken) still raises: skipping it would
    hide a second process holding the port;
  * nothing is ever widened: the sockets that come back are bound to exactly
    the configured host, never a wildcard (NET-C9).

How "absent" is produced without touching the network. 203.0.113.7 is in
TEST-NET-3 (RFC 5737), reserved for documentation: no real host owns it, so
binding it fails with EADDRNOTAVAIL on any machine, deterministically.

Mutants this file kills (run by hand, see the batch commit):
  * no skip (old behaviour, error escapes)       -> skip case
  * every OSError skipped (port conflicts too)    -> in-use case
  * empty result returned instead of refusing     -> all-absent case

Boundaries: socket binding only. Which addresses hmi.bind lists is
configs/p5_gateway.yaml (and its own assertions); serving HTTP on the sockets
is uvicorn's job and tested elsewhere.
"""
from __future__ import annotations

import socket

import pytest

from xbrain.p5_gateway.hmi.web_server import HmiBindError, make_bound_sockets

# Pure unit cases on the loopback interface; no robot network needed.
pytestmark = pytest.mark.no_device

#: An address no host owns (RFC 5737 TEST-NET-3): stands in for "the wifi IP
#: while wifi is down" -- the kernel answers EADDRNOTAVAIL for both.
_ABSENT = "203.0.113.7"


def _close(socks):
    """Release every socket a case created, so ports do not leak across cases.

    The sockets are listening; leaving them open would keep loopback ports
    busy for the rest of the pytest run.
    """
    for s in socks:
        s.close()


def test_an_absent_address_is_skipped_and_the_rest_are_served():
    """wifi absent + loopback present: one socket, on loopback only.

    Port 0 lets the kernel pick a free port, so the case never collides with
    a service already running on the machine.
    """
    socks = make_bound_sockets([_ABSENT + ":0", "127.0.0.1:0"])
    try:
        # Only the present address was bound ...
        assert len(socks) == 1
        # ... on exactly that host -- never widened to a wildcard (NET-C9).
        assert socks[0].getsockname()[0] == "127.0.0.1"
    finally:
        _close(socks)


def test_when_every_address_is_absent_the_hmi_still_refuses():
    """Skipping must not end in an HMI that serves nothing without a word.

    Same outcome as the all-null configuration: HmiBindError, which p5 logs
    and which keeps the problem in front of the operator.
    """
    # One entry, and it is absent: nothing can be served.
    # The message names the condition so the operator knows it is the
    # address list, not a crash, that kept the HMI down.
    with pytest.raises(HmiBindError, match="no hmi.bind address"):
        make_bound_sockets([_ABSENT + ":0"])


def test_a_port_conflict_still_raises():
    """Port already taken is a real conflict, not an absent interface.

    A listener is opened on a free loopback port first; binding the same
    host:port again must fail loudly (OSError), because skipping it would let
    the HMI start while another process owns its port.
    """
    # The "other process": a plain listener on a kernel-chosen loopback port.
    holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    holder.bind(("127.0.0.1", 0))
    holder.listen(1)
    # The port it got; the HMI is then asked to bind exactly that.
    port = holder.getsockname()[1]
    try:
        # Not EADDRNOTAVAIL (it is EADDRINUSE): must propagate.
        with pytest.raises(OSError):
            socks = make_bound_sockets(["127.0.0.1:%d" % port])
            # Unreachable when the code is right; closes the socket if not.
            _close(socks)
    finally:
        holder.close()
