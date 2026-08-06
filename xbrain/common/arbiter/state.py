"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state.py
Brief: BIZ-CM-2 state half -- Arbiter snapshot -> ArbDomainState for
       state/arb/{domain} (11 S7A.5.1)

Description:
The other half of BIZ-CM-2 (core.py: "turning them into ... state/arb/{domain}
is BIZ-CM-2"). render_domain_state reads the arbiter's frozen snapshots
(holder / gen / last_change / waiters / sources) and assembles the 11 S7A.5.1
ArbDomainState wire object. Every one of the seven domains publishes this same
shape, including the in-process domains motion and gpu (ARB-0), so p5_gateway can
aggregate them into state/arbitration uniformly.

Why held_ms / waited_ms are computed HERE and not stored. The arbiter never reads
the clock (its whole AB-6 discipline); held_ms and waited_ms are now -
since_mono_ms, so they can only be correct at publish time. The caller passes the
same now_mono_ms it drove tick() with. Storing them in the snapshot would make
them stale the instant they were written (see Holder / WaiterSnapshot docstrings).

Where suspended comes from. The disarm reason (soft_estop | hes | cmd_timeout)
is read from the arbiter via suspended() -- BIZ-CM-3 added that state and
validated the reason against ARB_SUSPENDED at the arb_suspend() boundary, so what
this reads is already a closed-set value or None. ArbDomainState REQUIRES the
field; a normal (armed) domain emits null.

What this deliberately omits. holder.note (11 S7A.5.1, 必填 "-", the HMI one-liner)
is NOT set: the arbiter model carries no note, and inventing one would be display
prose fabricated in the framework. It is left absent (an optional field); the
owning process adds it if it has one. Documented so its absence is a decision,
not a forgotten field.
"""

from typing import List, Optional

from .core import Arbiter
from .model import Holder, LastChange, SourceSnapshot, WaiterSnapshot

__all__ = ["render_domain_state"]


def _render_holder(holder: Optional[Holder], now_mono_ms: int) -> Optional[dict]:
    """The holder object of ArbDomainState, or None for an idle domain.

    held_ms is now - since_mono_ms, computed here (see module docstring). note is
    intentionally absent.
    """
    if holder is None:
        return None                                     # domain idle -> null
    return {
        "source_id": holder.source_id,                  # who holds it now
        "req_id": holder.req_id,                         # the hold attempt (S7A.1)
        "priority": holder.priority,                    # cached from the source spec
        "since_mono_ms": holder.since_mono_ms,          # when the hold began (monotonic)
        # held_ms = now - since. Computed here, never stored, because the arbiter
        # holds no clock and any stored copy would be stale-when-written.
        "held_ms": now_mono_ms - holder.since_mono_ms,
    }


def _render_waiting(waiters: List[WaiterSnapshot], now_mono_ms: int) -> List[dict]:
    """The waiting[] array; waited_ms derived at publish time like held_ms."""
    return [
        {
            "source_id": w.source_id,                   # who is queued
            "priority": w.priority,                     # its selection priority
            # waited_ms = now - when it queued; derived like held_ms, not stored.
            "waited_ms": now_mono_ms - w.since_mono_ms,
            "req_id": w.req_id,                          # echoed on the eventual grant
        }
        for w in waiters                                # already ordered by waiters()
    ]


def _render_last_change(lc: Optional[LastChange]) -> Optional[dict]:
    """The last_change object, or None before the first holder change.

    Same source as the audit event (LastChange mirrors it), so state and audit
    agree by construction rather than by two code paths kept in step.
    """
    if lc is None:
        return None                                     # no holder change yet
    return {
        "action": lc.action,                            # an ArbAction value
        "from": lc.from_source,                         # prior holder, or null
        "to": lc.to_source,                             # new holder, or null
        "reason": lc.reason,                            # why it changed
        "forced": lc.forced,                            # True if a T-1 forced preempt
        "mono_ms": lc.mono_ms,                          # when, monotonic
    }


def _render_sources(sources: List[SourceSnapshot]) -> List[dict]:
    """The sources[] array -- the whole registry, alive=false included."""
    return [
        {
            "source_id": s.source_id,                   # the registered id
            "priority": s.priority,                     # so the HMI can rank them
            "policy": s.policy,                         # immediate | wait_atomic | reject
            # alive=false is KEPT, not filtered: the HMI shows a source that could
            # have preempted but whose process died (S7A.5.1).
            "alive": s.alive,
        }
        for s in sources
    ]


def render_domain_state(arb: Arbiter, now_mono_ms: int) -> dict:
    """Assemble the 11 S7A.5.1 ArbDomainState for state/arb/{domain}.

    now_mono_ms must be the same monotonic reading the caller drove tick() with,
    so held_ms / waited_ms line up with the state they describe. The disarm reason
    is read from arb.suspended() (already a closed-set value or None; BIZ-CM-3
    validated it at the arb_suspend boundary).
    """
    return {
        "domain": arb.domain(),                         # already a closed-set value
        "gen": arb.gen(),                               # 11 S7A.2 G-1
        "suspended": arb.suspended(),                   # null | soft_estop | hes | cmd_timeout
        "holder": _render_holder(arb.holder(), now_mono_ms),
        "waiting": _render_waiting(arb.waiters(), now_mono_ms),
        "last_change": _render_last_change(arb.last_change()),
        "sources": _render_sources(arb.sources()),
    }
