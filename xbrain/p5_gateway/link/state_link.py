"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_link.py
Brief: GWY-P5-14 state/link production (LK-1..6 + N-1..3 + reason 6 values + BL-1..4 + EP-1..3)

Description:
17 S14 defines state/link as an authoritative summary of every
external link (cloud, HMI, chassis, ptz, payload). p5_gateway is
the UNIQUE PUBLISHER (no other process writes this topic).

  LK-1  publish rate: 1 Hz + on every change
  LK-2  each entry has (name, up:bool, since_ms, reason)
  LK-3  reason is a 6-value closed set:
        healthy / gateway_down / target_down / handshake_fail /
        stale / manually_disabled
  LK-4  transitions LOG an audit event
  LK-5  BL-1..4 blacklist rules for auto-suppressed publishers
  LK-6  EP-1..3 endpoint policy (per-link retry curve)

  N-1..3 hysteresis:
    N-1  a link must be up for >= up_debounce_ms before publishing up
    N-2  a link must be down for >= down_debounce_ms before publishing down
    N-3  hysteresis parameters come from configs (no code default)
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LinkReason(str, Enum):
    HEALTHY = "healthy"
    GATEWAY_DOWN = "gateway_down"
    TARGET_DOWN = "target_down"
    HANDSHAKE_FAIL = "handshake_fail"
    STALE = "stale"
    MANUALLY_DISABLED = "manually_disabled"


LINK_REASONS = frozenset(r.value for r in LinkReason)


@dataclass
class LinkStatus:
    name: str
    up: bool
    since_ms: int
    reason: str
    up_start_ms: int = 0
    down_start_ms: int = 0


def apply_up_debounce(status: LinkStatus,
                        now_ms: int,
                        debounce_ms: int,
                        reason: str) -> bool:
    """N-1: return True if link 'up' transition should be published
    (debounce satisfied)."""
    if reason not in LINK_REASONS:
        raise ValueError(f"reason {reason!r} not in closed set")
    if status.up:
        return False
    if status.up_start_ms == 0:
        status.up_start_ms = now_ms
        return False
    if now_ms - status.up_start_ms >= debounce_ms:
        status.up = True
        status.since_ms = now_ms
        status.reason = reason
        status.down_start_ms = 0
        status.up_start_ms = 0
        return True
    return False


def apply_down_debounce(status: LinkStatus,
                          now_ms: int,
                          debounce_ms: int,
                          reason: str) -> bool:
    """N-2: analogous for down transition."""
    if reason not in LINK_REASONS:
        raise ValueError(f"reason {reason!r} not in closed set")
    if not status.up:
        return False
    if status.down_start_ms == 0:
        status.down_start_ms = now_ms
        return False
    if now_ms - status.down_start_ms >= debounce_ms:
        status.up = False
        status.since_ms = now_ms
        status.reason = reason
        status.up_start_ms = 0
        status.down_start_ms = 0
        return True
    return False
