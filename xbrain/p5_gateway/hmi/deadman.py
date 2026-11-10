"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: deadman.py
Brief: 17 S6.2.1 WS session close -> cmd/teleop{deadman:false} + hmi.session_closed

Description:
The problem this solves. 11 S12.1.3 R-W13 took teleop off the HMI's writable
surface, but kept one duty for P5 in three places (S12.1.3, the S12.3 U36
table, S12A.9.5): when a WebSocket session closes, P5 must at once send
cmd/teleop{deadman:false}, without waiting for P1's 400 ms cmd_age timeout.
U36 lets the current task run on after a link loss, but a stick already in the
operator's hand must stop first. 17 S6.2.1 turns that into WD-1..WD-6; this
module builds the two messages, and the web server / wiring fire them.

The rules this follows (17 S6.2.1, numbered as there):
  WD-1  fire on EVERY connection close: TCP drop, server-side death, server close.
  WD-2  fire UNCONDITIONALLY. P5 keeps no account of who is driving (teleop is
        not an uplink class any more), and this is a tightening action: an
        extra release costs P1 nothing, a missed one leaves the teleop source
        alive.
  WD-3  no ack -- cmd/teleop deliberately has no ack key.
  WD-4  log an event/info/system hmi.session_closed with the client IP only.
  WD-6  fire once more on P5's own shutdown (SIGTERM path).

*** Shape: the 11 S12A.9.5 TeleopInput HMI variant, at the TOP level, not inside
an S3.0 envelope. P1 reads `device` / `source` / `deadman` / `axes` straight off
the top of the frame (p1_motion main_wiring _on_teleop); a frame with neither
`device` nor `source=hmi` is ignored there. The first version of this file sent
{deadman, vx, vy, wz} -- no device, no source, axes flattened -- which P1 drops
without a trace, so the release it existed for could never have happened.
`device` is keyboard_hmi: the name P1 records for an HMI sender (S12A.9.7).

What this does NOT do: it opens no socket, publishes nothing and reads no
clock. Delivery is the wiring's job (runtime/main_wiring _start_hmi).
"""

from __future__ import annotations

from typing import Any, Dict

#: S12A.9.7: the device P1 files an HMI-originated teleop frame under.
HMI_TELEOP_DEVICE = "keyboard_hmi"

#: 17 S6.2.1 WD-4: the event `type` inside detail. Also used, upper-cased, as the
#: event `code` -- the same construction p5's comm events use (CLOUD_LINK_UP),
#: since the cloud contract requires a non-empty machine code.
SESSION_CLOSED_TYPE = "hmi.session_closed"
SESSION_CLOSED_CODE = "HMI_SESSION_CLOSED"


def build_deadman_payload(client_ip: str) -> Dict[str, Any]:
    """WD-1: the release frame for cmd/teleop.

    client_id follows the S12A.9.5 example form "hmi-<ip>". No client_seq: the
    browser never sent teleop, so there is no sequence to continue, and P1 does
    not order by it. Zero axes and buttons spelled out so a reader of the bus
    sees a complete TeleopInput, not a partial one.
    """
    return {
        "device": HMI_TELEOP_DEVICE,
        "source": "hmi",
        "client_id": "hmi-" + client_ip,
        "deadman": False,
        "axes": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
        "buttons": {"mark": False, "release": False},
    }


def build_session_closed_detail(session_id: str, client_ip: str) -> Dict[str, Any]:
    """WD-4: the event detail. IP only, never an identity (11 U23a)."""
    return {
        "type": SESSION_CLOSED_TYPE,
        "session_id": session_id,
        "client_ip": client_ip,
        "deadman_released": True,
    }
