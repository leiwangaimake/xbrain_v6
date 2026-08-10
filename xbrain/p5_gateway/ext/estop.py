"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop.py
Brief: CHK-0-40 甲方独立急停 Q0 直翻 (cmd/estop -> chassis_relay, <=100ms)

Description:
`xbrain/{rid}/cmd/estop` MUST bypass the normal task pipeline entirely
(R3.3 in docs/MISSON/任务枚举_qt端v2.0.md). It goes:

  Zenoh Q0 receive
    -> validate envelope (in this module)
    -> chassis_relay estop channel (direct forward, no arbiter, no
       queue, no ack from p3)
    -> emit cmd/estop/ack with estop_epoch/applied/recv_mono_ms/
       latency_ms/hes/timeout_lock within 100 ms

Latency budget: robot receives cmd/estop -> ack forwarded within
100 ms; Qt click -> ack landed within 300 ms end-to-end. Both are
measured using RECEIVER MONOTONIC CLOCK (never ts_utc). Any code
that reads ts_utc here to compute latency is a defect.

state/link.estop_path is an INDEPENDENT ok/degraded/down field.
It is not derivable from normal-plane liveness; the two are
different Zenoh sessions on different planes:
  * cmd/estop:      通用面 tcp/<ip>:7447 Q0 dedicated queue
  * state/link:     普通面 same session but different key
The plane-independence discipline is why estop-path down does not
imply cmd/task down and vice versa.

Down debounce: state/link.estop_path flips to 'down' after 3
consecutive missed heartbeats (contract 3 s wall time = ~3
one-second beats). Ack MUST NOT be treated as heartbeat: a single
successful ack does not clear a down mark that came from the
liveness beat, it only advances the last-ack-received timestamp.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from xbrain.common.errors import E_SCHEMA


ESTOP_ACTIONS = frozenset({"stop"})


ESTOP_MAX_FORWARD_MS = 100
ESTOP_MAX_E2E_MS = 300


class EstopSchemaError(Exception):
    """Envelope defective; cannot be forwarded."""


@dataclass(frozen=True)
class EstopFrame:
    """Validated, ready to hand to chassis_relay directly."""
    rid: str
    action: str
    reason: str
    ts_utc_sec: float
    seq: int
    src: str
    msg_id: str


@dataclass(frozen=True)
class EstopAck:
    """cmd/estop/ack payload. All time fields are receiver
    monotonic ms (NEVER ts_utc)."""
    rid: str
    estop_epoch: int
    applied: bool
    recv_mono_ms: int
    latency_ms: int
    hes: str            # 'engaged' / 'cleared' / 'unknown'
    timeout_lock: bool


def validate_and_forward(msg: dict, key_second_segment: str) -> EstopFrame:
    """Fast validate a cmd/estop envelope. Any defect raises
    EstopSchemaError -- the caller must NOT queue or retry; the safe
    action on a defective estop is to LOG and immediately forward a
    'safety-side' HES engaged signal to the chassis to preserve
    fail-safe semantics."""
    if not isinstance(msg, dict):
        raise EstopSchemaError("estop envelope not object")
    rid = msg.get("rid")
    if not isinstance(rid, str) or rid != key_second_segment:
        raise EstopSchemaError(
            f"estop rid mismatch: {rid!r} vs {key_second_segment!r}")
    data = msg.get("data") or {}
    action = data.get("action") if isinstance(data, dict) else None
    if action not in ESTOP_ACTIONS:
        raise EstopSchemaError(
            f"estop action not in closed set: got {action!r}")
    return EstopFrame(
        rid=rid,
        action=action,
        reason=str(data.get("reason") or ""),
        ts_utc_sec=float(msg.get("ts") or 0.0),
        seq=int(msg.get("seq") or 0),
        src=str(msg.get("src") or ""),
        msg_id=str(msg.get("msg_id") or ""),
    )


def build_ack(frame: EstopFrame,
                recv_mono_ms: int,
                sent_mono_ms: int,
                estop_epoch: int,
                applied: bool,
                hes: str,
                timeout_lock: bool) -> EstopAck:
    """Assemble the ack. latency_ms uses monotonic diff (R3.3
    "该判定使用机器人单调钟字段，不用两端 ts 相减")."""
    if hes not in ("engaged", "cleared", "unknown"):
        raise EstopSchemaError(
            f"hes closed set violation: {hes!r}")
    latency_ms = sent_mono_ms - recv_mono_ms
    if latency_ms < 0:
        raise EstopSchemaError(
            f"latency_ms negative: sent={sent_mono_ms} recv={recv_mono_ms}")
    return EstopAck(
        rid=frame.rid,
        estop_epoch=estop_epoch,
        applied=applied,
        recv_mono_ms=recv_mono_ms,
        latency_ms=latency_ms,
        hes=hes,
        timeout_lock=timeout_lock,
    )


def check_forward_budget(latency_ms: int) -> bool:
    """R3.3: <= 100 ms robot-side forward budget. Returns True if
    within budget."""
    return latency_ms <= ESTOP_MAX_FORWARD_MS


def check_e2e_budget(qt_click_mono_ms: int,
                       ack_received_mono_ms: int) -> bool:
    """R3.3: 300 ms end-to-end budget (Qt click -> ack visible)."""
    return (ack_received_mono_ms - qt_click_mono_ms) <= ESTOP_MAX_E2E_MS


@dataclass
class EstopPathHealth:
    """state/link.estop_path 独立字段 (R3.3).
    Down debounce: >=3 consecutive missed beats -> 'down'.
    A successful ack does NOT clear a 'down' mark; only a resumed
    beat sequence does."""
    consecutive_misses: int = 0
    state: str = "ok"        # ok / degraded / down
    miss_threshold: int = 3

    def on_beat_received(self) -> None:
        """Reset the miss counter; may promote from down back to ok."""
        self.consecutive_misses = 0
        if self.state != "ok":
            self.state = "ok"

    def on_beat_missed(self) -> None:
        """Advance miss counter; may demote to degraded then down."""
        self.consecutive_misses += 1
        if self.consecutive_misses >= self.miss_threshold:
            self.state = "down"
        elif self.consecutive_misses >= 1:
            self.state = "degraded"

    def on_ack_received(self, latency_ms: int) -> None:
        """Ack alone does NOT clear a 'down' beat state (R3.3:
        'It cannot be replaced by a single ack.'). It only records
        the fact of ack delivery for latency stats."""
        # Explicitly do not mutate state.
        _ = latency_ms
