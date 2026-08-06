"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: model.py
Brief: The immutable data model of the unified arbitration framework (11 S7A)

Description:
What problem this solves. 11 S7A defines one arbitration semantics shared by all
seven resource domains (motion, speaker, asr, payload_light, ptz, gpu, dock), and
14 S3 makes the framework a shared LIBRARY that every owning process instantiates
rather than a central service. This file is the value layer of that library: the
SourceSpec a caller registers, the Request it submits, and the Grant / Holder /
LastChange / ArbEvent objects the arbiter hands back. core.py holds the state
machine; keeping the data here means the wire shapes of 11 S7A.1 / S7A.2 / S7A.5.1
can be read in one place and can be frozen so no consumer mutates a shared object.

Which section each type comes from (NUM-4 anchors, grep the doc):
  * SourceSpec        14 S3.2 (grep "class SourceSpec"), lease rule 11 S7A.4
  * Request           11 S7A.1 ArbRequest (grep "ArbRequest") -- the acquire path
  * Grant             11 S7A.2 ArbGrant (grep "ArbGrant") -- broadcast result
  * Holder            11 S7A.5.1 ArbDomainState.holder
  * LastChange        11 S7A.5.1 ArbDomainState.last_change
  * ArbEvent          14 S3.6 / 11 S7A.7 audit event
  * PreemptPolicy     11 S7A.3 (immediate | wait_atomic | reject)
  * GrantResult       11 S7A.2 result column

What this file deliberately does NOT do, so nothing gets bolted onto it:
  * It holds no logic. Priority comparison, gen bookkeeping, lease and preempt
    timing all live in core.py. A method here that "just" compared two priorities
    would be a second place the arbitration rule lives, and the two would drift.
  * It does not read a clock. Every timestamp is an int millisecond value the
    CALLER measured from CLOCK_MONOTONIC (11 S7A.1: mono_ms is not the envelope
    ts). The model stores what it is given; it never asks the time itself. That
    is why a wall-clock forward step cannot enter through this layer -- there is
    no clock here to read the wrong one from.
  * It does not own the closed sets. domain and release_reason are validated in
    core.py against xbrain/common/enums; PreemptPolicy and GrantResult are
    framework-internal enums (11 lists their values on the wire but BIZ-CM-0 did
    not export them, so they are not part of the closed-set metatest and must not
    be spelled as bare literals elsewhere -- import the enum).

Traps -- things that look right and are not:
  1. lease_timeout_s has NO default here, and that is on purpose. 14 S3.2 sketches
     the dataclass with lease_timeout_s = 1.0, and 14 S4.3 records the bug that
     default caused: two payload_light sources with no explicit value fell to the
     1.0 s domain default, so a human holding the lights by hand lost them one
     second later (手动控灯用一秒就掉). None here is the EXPLICIT value meaning
     "resident, no lease" (mode_driver, 11 S7A.4). Requiring the field forces the
     choice to be made, the same way ClosedSet requires every field injected.
  2. None on lease_timeout_s is a real value, not "unset". Do NOT write 0.0 to
     mean resident: core.py reads None as no-lease and any number as a real lease,
     so 0.0 would make the source expire on the very next tick (CLAUDE.md 3.1, the
     fail-silent 0.0).
  3. A frozen dataclass whose field is a dict (ArbEvent.detail) is only shallowly
     immutable: rebinding the field raises, mutating the dict does not. detail is
     treated as read-only by convention; do not hand the same dict to two events.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional

# The whole public value surface. core.py adds the Arbiter class; the two are
# re-exported together from the package __init__ so a caller writes
# `from xbrain.common.arbiter import SourceSpec, Grant`.
__all__ = [
    "PreemptPolicy", "GrantResult", "ArbAction",
    "SourceSpec", "Request", "Preempt", "Grant", "Holder", "LastChange",
    "WaiterSnapshot", "SourceSnapshot",
    "ArbEvent",
    "DEFAULT_LEASE_MS", "IMMEDIATE_GRACE_MS", "DEFAULT_WAIT_ATOMIC_TIMEOUT_MS",
]


# ---------------------------------------------------------------------------
# Contract constants. These are the fixed values of 11 S7A, not tunable
# common.safety.* parameters (so CLAUDE.md 3.1 does not require them to be
# injected as null-until-calibrated). They are named here once so a mutation
# that changes one -- e.g. the 100 ms immediate deadline of BIZ-CM-1 mutant (4)
# -- has a single, greppable point to change, and so no call site spells the
# number as a bare literal.
# ---------------------------------------------------------------------------

#: 11 S7A.4: default lease is 1000 ms, with renew required at >= 2 Hz so a single
#: dropped renew still leaves one period of slack (11 S7A.4 "丢包容忍").
DEFAULT_LEASE_MS = 1000

#: 11 S7A.3: an immediate preemption gives the current holder until now + 100 ms
#: to ack. This is the constant BIZ-CM-1 mutation (4) unbounds to prove the T-1
#: forced-preempt path is really tested.
IMMEDIATE_GRACE_MS = 100

#: 11 S7A.3: wait_atomic waits for the holder's atomic action, default 3 s. It is
#: a per-domain configuration ceiling; the Arbiter takes it at construction rather
#: than defaulting it in code, but the contract default is recorded here for the
#: one caller (a test, a bring-up) that has no domain config to hand.
DEFAULT_WAIT_ATOMIC_TIMEOUT_MS = 3000


class PreemptPolicy(str, Enum):
    """How a source behaves at a preemption boundary (11 S7A.3).

    str-valued so .value is the exact wire spelling (immediate | wait_atomic |
    reject) that appears in ArbGrant.preempt.policy and ArbDomainState.sources[].
    """

    # As a HOLDER: yields within IMMEDIATE_GRACE_MS. As the D-mode siren, ASR
    # switch or manual PTZ takeover, this is the common case (11 S7A.3 用于 col).
    IMMEDIATE = "immediate"
    # As a HOLDER doing an atomic action (one TTS sentence, one move to preset):
    # the preemptor waits up to the domain's wait_atomic timeout before forcing.
    WAIT_ATOMIC = "wait_atomic"
    # As a REQUESTER: do not queue and do not wait. core.py grants a reject source
    # only when the domain is free, else denies E_BUSY. This is the manual light
    # source that must not fight the D-mode driver (14 S4.3).
    REJECT = "reject"


class GrantResult(str, Enum):
    """The result field of ArbGrant (11 S7A.2 result column).

    str-valued for the same reason as PreemptPolicy: the member value is the wire
    token. denied carries a code; granted/queued/preempt/revoked/released do not.
    """

    GRANTED = "granted"     # you hold it now; output carrying this gen
    DENIED = "denied"       # see .code (E_BUSY / E_ARB_NO_SOURCE / ...)
    QUEUED = "queued"       # accepted, holder is finishing an atomic action
    PREEMPT = "preempt"     # notice TO the current holder to stop (P-1..P-3)
    REVOKED = "revoked"     # forcibly reclaimed (lease / death / timeout)
    RELEASED = "released"   # holder let go, domain idle


class ArbAction(str, Enum):
    """The audit-event action (14 S3.6 / 11 S7A.7).

    BIZ-CM-2 maps these to severity (grant/release/preempt -> info,
    lease_timeout/forced_preempt -> warn, source_death -> fault) and applies the
    dedup rules; BIZ-CM-1 only stamps the action so that mapping has an input.
    Kept as an enum, not free text, so a typo in an action is a construction
    error rather than an event that silently never matches its severity row.
    """

    GRANT = "grant"                    # a new holder took the domain
    RELEASE = "release"                # holder released voluntarily
    PREEMPT = "preempt"                # a preemption was started
    LEASE_TIMEOUT = "lease_timeout"    # holder failed to renew in time
    SOURCE_DEATH = "source_death"      # holder's process was reported dead
    FORCED_PREEMPT = "forced_preempt"  # T-1: holder missed the ack deadline


@dataclass(frozen=True)
class SourceSpec:
    """A competing source, registered once at startup (14 S3.2, 11 S7A.5.1).

    Every field is required -- there are no constructor defaults. See trap (1) in
    the module docstring: the one default 14 S3.2 sketched (lease_timeout_s = 1.0)
    is the one 14 S4.3 records as a shipped bug. Requiring the value makes each
    source state its lease and policy on purpose, and matches the ClosedSet
    convention in xbrain/common/enums that nothing is defaulted in the signature.
    """

    # The wire identity, matched against ArbRequest.source_id. Unregistered ids
    # are denied E_ARB_NO_SOURCE by the arbiter (11 S7A.1).
    source_id: str
    # Higher wins (14 S3.2 "越大越高"). urgency (on Request) only breaks ties at
    # EQUAL priority (11 S7A.1 / 12 A-2); it never lets a lower priority win, or
    # ARB-2 would be violated.
    priority: int
    # Whether a strictly-higher-priority request may take this holder. Almost
    # every real source is True. False means protected: core.py denies even a
    # higher requester E_BUSY rather than preempt. There is no such source in the
    # domain configs today; the field is honoured for completeness and its risk
    # (a protected low-priority holder blocking a safety source) is the
    # registrant's to accept.
    preemptible: bool
    # How this source behaves at a preemption boundary (see PreemptPolicy).
    preempt_policy: PreemptPolicy
    # Seconds, or None for a resident source with no lease (mode_driver, 11
    # S7A.4). None is a VALUE, not "unset" -- do not pass 0.0 (module trap 2).
    lease_timeout_s: Optional[float]
    # Called when this source is the holder and a preemption starts (P-1: stop
    # producing new output now). None if the source has no such hook; passed
    # explicitly, never defaulted, so registration is deliberate.
    on_preempt: Optional[Callable]
    # Called when this source loses the resource INVOLUNTARILY (lease timeout,
    # process death, forced preempt). Not called on a graceful release or ack --
    # those are the source's own action and it already knows.
    on_lost: Optional[Callable]


@dataclass(frozen=True)
class Request:
    """One acquire attempt (11 S7A.1 ArbRequest, op = acquire).

    req_id and mono_ms are required; the rest carry the contract's own optional
    defaults (the 必填 column marks them "-"). These defaults are the wire
    optional-field defaults of 11 S7A.1, NOT the safety-parameter defaults
    CLAUDE.md 3.1 forbids -- there is no common.safety.* value here.
    """

    # Identifies this whole hold attempt; renew/release/ack_preempt/cancel all
    # echo it back (11 S7A.1 req_id row).
    req_id: str
    # CLOCK_MONOTONIC milliseconds measured by the CALLER. The arbiter does all
    # lease and deadline arithmetic against this and against tick()'s now_mono_ms;
    # it never reads a clock itself (11 S7A.1 "mono_ms 是 CLOCK_MONOTONIC").
    mono_ms: int
    # Tie-break among EQUAL-priority requests only (11 S7A.1). Default 0.
    urgency: int = 0
    # Requested lease in ms, clamped down to the source's own ceiling by the
    # arbiter (11 S7A.1 lease_ms row). None -> use the source's lease_timeout_s.
    lease_ms: Optional[int] = None
    # The holder declares its current action unbreakable (one TTS, one preset
    # move). Advisory to a wait_atomic preemptor; the timeout still bounds it.
    atomic: bool = False
    # Free text into the audit event; not branched on.
    reason: str = ""
    # Self-reported estimated duration, ms. Advisory only, never a promise
    # (11 S7A.1 est_ms row); the wait ceiling is the domain's wait_atomic timeout.
    est_ms: Optional[int] = None


@dataclass(frozen=True)
class Preempt:
    """The preempt sub-object of ArbGrant (11 S7A.2 "preempt 子对象").

    Attached to the QUEUED grant returned to a preemptor and describes the notice
    the current holder received: who is taking it, by what policy, and the
    monotonic millisecond by which the holder must have acked.
    """

    by: str                       # source_id taking the domain
    by_priority: int
    policy: str                   # PreemptPolicy value that set the deadline
    # None only under a mutated arbiter (BIZ-CM-1 mutant (4) unbounds the
    # immediate deadline). In the reference this is always a real deadline.
    deadline_mono_ms: Optional[int]
    reason: str


@dataclass(frozen=True)
class Holder:
    """The current holder snapshot (11 S7A.5.1 ArbDomainState.holder).

    held_ms is intentionally absent: it is derived (now - since_mono_ms) and the
    arbiter does not read now, so the consumer that publishes state/arb computes
    it at publish time. Storing it here would be a value that is stale the moment
    it is written.
    """

    source_id: str
    req_id: str
    priority: int
    since_mono_ms: int
    # The lease actually in force, ms, or None for a resident holder. Computed
    # once at grant time from the source's lease_timeout_s and any request
    # override; tick() reads this, never the spec, so a resident holder (None)
    # is skipped by the lease check.
    lease_ms: Optional[int]


@dataclass(frozen=True)
class WaiterSnapshot:
    """One queued requester, as ArbDomainState.waiting[] needs it (11 S7A.5.1).

    waited_ms is intentionally absent for the same reason held_ms is absent from
    Holder: it is now - since_mono_ms, and the arbiter does not read now. The
    state serialiser (BIZ-CM-2) computes it at publish time from the tick's
    now_mono_ms, so the number is never stale-when-stored.
    """

    source_id: str
    req_id: str
    priority: int
    since_mono_ms: int


@dataclass(frozen=True)
class SourceSnapshot:
    """One registered source, as ArbDomainState.sources[] needs it (11 S7A.5.1).

    The full registry is published, alive=false included, so the HMI can show a
    source that could have preempted but whose process is gone (11 S7A.5.1
    "谁本可以抢但进程没了"). policy is the PreemptPolicy wire value.
    """

    source_id: str
    priority: int
    policy: str
    alive: bool


@dataclass(frozen=True)
class Grant:
    """The result handed back for one request (11 S7A.2 ArbGrant).

    In-process this is the return value of request(); cross-process (BIZ-CM-2) the
    same fields serialise onto cmd/arb/{domain}/grant. gen is the arbitration
    generation the requester must stamp on its output (G-2); the executor discards
    output whose gen is behind (G-3).
    """

    domain: str
    result: GrantResult
    req_id: str
    source_id: str
    code: str                     # OK unless result is denied (11 S7A.2 code col)
    gen: int
    holder: Optional[Holder]      # the domain's holder AFTER this request
    preempt: Optional[Preempt]    # set when result is queued/preempt
    forced: bool                  # True when this granted came from a forced T-1
    mono_ms: int                  # when the arbiter decided (echo of the input)


@dataclass(frozen=True)
class LastChange:
    """The most recent holder change (11 S7A.5.1 ArbDomainState.last_change).

    Same source of truth as the audit event, so state and audit agree by
    construction rather than by two code paths that must be kept in step.
    """

    action: str                   # an ArbAction value
    from_source: Optional[str]
    to_source: Optional[str]
    reason: str
    forced: bool
    mono_ms: int


@dataclass(frozen=True)
class ArbEvent:
    """One audit record (14 S3.6 / 11 S7A.7).

    BIZ-CM-1 emits these on every holder change and on the reclamation paths;
    BIZ-CM-2 assigns severity, applies the dedup window, and assembles the
    state/arb message from them. detail carries per-action extras (overdue_ms for
    forced_preempt, held_ms for a release); see module trap (3) on its mutability.
    """

    action: str                   # an ArbAction value
    domain: str
    from_source: Optional[str]
    to_source: Optional[str]
    reason: str
    forced: bool
    gen: int
    mono_ms: int
    # default_factory, not a shared {} default: a bare mutable default is one dict
    # aliased by every event, so a later write to one event's detail would appear
    # in all of them.
    detail: Dict[str, object] = field(default_factory=dict)
