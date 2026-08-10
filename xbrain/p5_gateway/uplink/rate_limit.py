"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rate_limit.py
Brief: CHK-1-16 §12.1.5 HMI uplink rate-limit + degrade SM + sanitisation

Description:
17 §12.1.5 HMI uplink discipline. FIVE deliberate rules; a single
missing rule is a specific class of defect:

  * per-conn non-teleop: <= 10 msg/s; excess dropped + violation
    counted
  * per-conn teleop:     <= 20 Hz per client_id; excess dropped +
    ONE comm-warn event
  * payload cap:         geo <= 1 MiB, other <= 8 KiB
  * ack(rejected)  <= 5/s/conn; excess COALESCE into one ack with
    detail.suppressed == N (**never drop** -- an operator's rejected
    req_id must still round-trip an eventual ack, even if bundled)
  * degrade SM:  20 consecutive rejects with zero accepts -> enter
    'restricted' state; state/link.uplink == 'restricted'; only
    estop still admitted; 60 s clean -> auto back to 'normal'

  ** the two negative rules of value: NEVER close() the WebSocket
     (dropping the connection also drops the estop button) and
     NEVER blacklist by source IP (denies operator control) **

Sanitisation S-1..S-4:
  S-1  req_type not ^[a-z][a-z0-9_]{0,31}$ -> replace with '<invalid>'
  S-2  req_id not ^[A-Za-z0-9_-]{1,40}$    -> replace with None
  S-3  message MUST come from gateway text-table, NEVER concatenated
       from client input
  S-4  the offending payload is NEVER echoed into event.detail
       (would be rendered by HMI + logged forever)
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Dict, List


class UplinkConfigError(Exception):
    pass


REQ_TYPE_RE = re.compile(r"^[a-z][a-z0-9_]{0,31}$")
REQ_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")


DEGRADE_ENTER_CONSECUTIVE_REJECTS = 20
DEGRADE_EXIT_CLEAN_SECONDS = 60


@dataclass(frozen=True)
class UplinkLimits:
    """All required at construction (CLAUDE.md 3.1)."""
    non_teleop_msgs_per_sec: int
    teleop_msgs_per_sec_per_client_id: int
    payload_cap_geo_bytes: int
    payload_cap_other_bytes: int
    reject_ack_per_sec: int

    def __post_init__(self) -> None:
        for name in ("non_teleop_msgs_per_sec",
                      "teleop_msgs_per_sec_per_client_id",
                      "payload_cap_geo_bytes",
                      "payload_cap_other_bytes",
                      "reject_ack_per_sec"):
            v = getattr(self, name)
            if v <= 0:
                raise UplinkConfigError(
                    f"{name} must be > 0, got {v} (zero would either "
                    f"admit everything or reject everything silently)")


UPLINK_STATE_NORMAL = "normal"
UPLINK_STATE_RESTRICTED = "restricted"


@dataclass
class UplinkDegradeState:
    """Per-connection SM: enter restricted after 20 consecutive
    rejects with zero accepts; exit after 60s clean."""
    state: str = UPLINK_STATE_NORMAL
    consecutive_rejects: int = 0
    last_reject_or_accept_mono_ms: int = 0

    def note_reject(self, now_mono_ms: int) -> None:
        self.consecutive_rejects += 1
        self.last_reject_or_accept_mono_ms = now_mono_ms
        if (self.state == UPLINK_STATE_NORMAL
                and self.consecutive_rejects
                    >= DEGRADE_ENTER_CONSECUTIVE_REJECTS):
            self.state = UPLINK_STATE_RESTRICTED

    def note_accept(self, now_mono_ms: int) -> None:
        self.consecutive_rejects = 0
        self.last_reject_or_accept_mono_ms = now_mono_ms

    def tick_clock(self, now_mono_ms: int) -> None:
        """Called each second; may promote restricted -> normal."""
        if self.state != UPLINK_STATE_RESTRICTED:
            return
        idle_ms = now_mono_ms - self.last_reject_or_accept_mono_ms
        if idle_ms >= DEGRADE_EXIT_CLEAN_SECONDS * 1000:
            self.state = UPLINK_STATE_NORMAL
            self.consecutive_rejects = 0


def can_admit_in_state(req_type: str, state: str) -> bool:
    """Restricted state admits only estop; normal admits everything
    (subject to rate limit)."""
    if state == UPLINK_STATE_NORMAL:
        return True
    return req_type == "estop"


# ---- sanitisation --------------------------------------------------

INVALID_REQ_TYPE_PLACEHOLDER = "<invalid>"


def sanitise_req_type(raw: str) -> str:
    """S-1: bad -> placeholder (never propagates raw)."""
    if not isinstance(raw, str) or not REQ_TYPE_RE.match(raw):
        return INVALID_REQ_TYPE_PLACEHOLDER
    return raw


def sanitise_req_id(raw):
    """S-2: bad -> None."""
    if not isinstance(raw, str) or not REQ_ID_RE.match(raw):
        return None
    return raw


class ForbiddenMessagePassthrough(Exception):
    """S-3: caller tried to concatenate client input into an ack
    message. Ack messages MUST come from the gateway text-table."""


def emit_ack_message(local_key: str,
                       table: dict) -> str:
    """Look up an ack message from the table; NEVER format-concat
    client input into it."""
    if local_key not in table:
        raise ForbiddenMessagePassthrough(
            f"ack message key {local_key!r} not in gateway text table")
    return table[local_key]


def sanitise_event_detail_never_echo(rejected_payload: dict,
                                       detail: dict) -> dict:
    """S-4: strip any key from detail that could echo rejected_payload
    content. Returns a copy of detail with the payload's field NAMES
    removed."""
    poisoned_keys = set(rejected_payload.keys())
    return {k: v for k, v in detail.items() if k not in poisoned_keys}


# ---- ack coalesce --------------------------------------------------

@dataclass
class RejectAckCoalescer:
    """Bundles rejected acks that would exceed reject_ack_per_sec
    into a single ack with detail.suppressed == count."""
    limit_per_sec: int
    window_start_mono_ms: int = 0
    emitted_this_window: int = 0
    suppressed_this_window: int = 0
    suppressed_req_ids: List[str] = field(default_factory=list)

    def observe(self, req_id: str, now_mono_ms: int) -> dict:
        """Return an ack payload. If under limit: pass-through
        {req_id, suppressed:0}. If at/over limit: {req_id,
        suppressed:N} where N accumulates and req_id is 'coalesced'."""
        if now_mono_ms - self.window_start_mono_ms >= 1000:
            # New second: reset window.
            self.window_start_mono_ms = now_mono_ms
            self.emitted_this_window = 0
            self.suppressed_this_window = 0
            self.suppressed_req_ids = []
        if self.emitted_this_window < self.limit_per_sec:
            self.emitted_this_window += 1
            return {"req_id": req_id, "suppressed": 0}
        # Over limit: coalesce.
        self.suppressed_this_window += 1
        self.suppressed_req_ids.append(req_id)
        return {"req_id": "coalesced",
                "suppressed": self.suppressed_this_window}


# ---- 'never disconnect' guard --------------------------------------

class ForbiddenConnectionMutation(Exception):
    """Trying to close() a WebSocket or ban a source IP: forbidden
    (would strip the operator's estop button)."""


def refuse_close_ws() -> None:
    """Callers must NOT close a WebSocket during degrade. This is
    an explicit refusal point so a future PR that tries to close()
    fails obviously rather than silently."""
    raise ForbiddenConnectionMutation(
        "refuse_close_ws: WebSocket close is banned during degrade; "
        "dropping the socket removes the estop button")


def refuse_ip_ban(source_ip: str) -> None:
    raise ForbiddenConnectionMutation(
        f"refuse_ip_ban({source_ip!r}): IP blacklist banned "
        "(operator control cannot be denied by source)")
