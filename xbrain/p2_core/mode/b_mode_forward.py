"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: b_mode_forward.py
Brief: BIZ-P2-29 -- B-mode cloud audio forwarding + gen-drop

Description:
B-mode plays cloud-streamed audio through the speaker. Cloud pushes
PCM chunks on `audio/broadcast`; P2 forwards them to payload-service
WS /play. Rules:

  * Hold and produce sound are DECOUPLED: the domain 2 holder is
    'broadcast_b' the whole time B is active, but silence between
    audio chunks is normal (the cloud can pause without dropping the
    hold).
  * gen-drop: audio chunks arrive tagged with the domain 2 grant's
    gen; if a chunk arrives whose gen is BEHIND the current gen,
    DROP it (the cloud stream from a previous holder still had
    frames in flight after preempt). This is 11 S7A.5 G-3.
  * broadcast_after_mode_exit: if mode exited B while a chunk was
    in flight, drop it -- audio must not play post-exit.

★ Actual WS forwarding is BIZ-P2-2 payload_client's responsibility;
this module owns the gen-check + mode-exit guard.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class BroadcastForwarder:
    """State for one B-mode session. Reset on B enter / exit."""
    # Current gen of the broadcast_b holder. Set on grant, incremented
    # on any preempt.
    current_gen: int = 0
    # True while B mode is active. Cleared on mode exit.
    b_mode_active: bool = False


def should_forward(fw: BroadcastForwarder, chunk_gen: int) -> bool:
    """Decide whether to forward this audio chunk to WS /play.

    Returns False if:
      * B mode is not active (drop post-exit chunks)
      * chunk_gen is behind current_gen (G-3 gen drop)
    """
    if not fw.b_mode_active:
        return False
    if chunk_gen < fw.current_gen:
        return False
    return True


def on_grant(fw: BroadcastForwarder, new_gen: int) -> None:
    """Called on domain 2 grant to broadcast_b."""
    fw.b_mode_active = True
    fw.current_gen = new_gen


def on_mode_exit(fw: BroadcastForwarder) -> None:
    """Called on B mode exit (timeout / cmd / safety)."""
    fw.b_mode_active = False
