"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audit.py
Brief: BIZ-CM-2 audit half -- ArbEvent -> Event payload, severity by action, and
       the 10 s dedup-window merge (11 S7A.7 / 14 S3.6)

Description:
core.py (BIZ-CM-1) emits an ArbEvent on every holder change and reclamation but
deliberately does NOT serialise it (core.py: "turning them into ... the audit
stream is BIZ-CM-2"). This module is that serialisation: it maps an action to a
severity, stamps the Event fields for event/{severity}/arbitration (11 S6.1,
S2.2.11), and merges a burst of same-kind events into one within a 10 s window.

Why the merge is a separate pure function and not a stateful producer. 11 S6.1
makes dedup_key and dedup_window_s PAYLOAD fields: the producer stamps them on
every event and an aggregator collapses by (key, window). Keeping the collapse a
pure function over a list of ArbEvent -- which already carry mono_ms -- means it
is exactly testable, has one definition, and does not need a clock of its own
(the arbiter's clock discipline: nothing here reads the time; the caller's
mono_ms is used). p5_gateway calls it on drain; a test calls it on a fixture.

The severity map is the contract's, verbatim (11 S7A.7 line "severity 段取值随
action"): grant/release/preempt/suspend/rearm -> info; forced_preempt/
lease_timeout -> warn; source_death/source_disabled -> fault. Only the actions
ArbAction defines today are mapped; suspend/rearm/source_disabled arrive with
BIZ-CM-3 and MUST be added to the map then -- test_arbiter_report.py asserts the
map covers every ArbAction member, so a new action with no severity fails loudly
rather than defaulting to some quiet value.

Traps -- things that look right and are not:
  1. An exempt action (forced_preempt, source_death, and, once BIZ-CM-3 lands,
     source_disabled/suspend/rearm) must NEVER be merged: it is the rare, urgent
     one, and collapsing three source deaths into "count: 3" hides two of them.
     Exempt events are stamped dedup_window_s = 0 and merge_audit_window leaves
     any window-0 event untouched. Mutation: drop an action from DEDUP_EXEMPT and
     its events start collapsing -- test_arbiter_report catches it.
  2. severity and category are CLOSED SETS (xbrain/common/enums). They are looked
     up through SEVERITY.parse / EVENT_CATEGORY.parse, never spelled as literals,
     so an off-contract value is E_SCHEMA at the boundary, not a key the cloud
     alarm subscription silently never matches (11 S2.2.11 V-2).
  3. eid and delivered are NOT set here. The event bus (p5_gateway, the sole
     event publisher) stamps eid at publish and tracks delivered; this module
     owns only the arbitration-specific fields, the same way the envelope layer
     owns the outer wrapper and not the Event body.
"""

from typing import Dict, List, Optional

from xbrain.common.enums import EVENT_CATEGORY, SEVERITY

from .model import ArbAction, ArbEvent

__all__ = [
    "SEVERITY_BY_ACTION", "DEDUP_EXEMPT", "DEDUP_WINDOW_S", "AUDIT_CATEGORY",
    "AUDIT_CHANNEL", "severity_of", "render_audit_event", "merge_audit_window",
]

# 11 S6.2 event category for the arbitration audit stream. Parsed once through
# the closed set so a typo here is a construction-time ClosedSetViolation.
AUDIT_CATEGORY = EVENT_CATEGORY.parse("arbitration")

# 11 S2.2.11 line for event/{sev}/arbitration: "channel 硬编码 normal". channel
# is not an exported closed set; this is the one fixed value the category uses,
# named once so no call site spells it.
AUDIT_CHANNEL = "normal"

# 14 S3.6 / 11 S7A.7 severity map, verbatim. Values go through SEVERITY.parse so
# a wrong severity spelling fails here, not at the subscriber. Keyed by the
# ArbAction member (not its string) so a renamed action is a KeyError at import
# via the completeness check, not a row that silently never matches.
SEVERITY_BY_ACTION: Dict[ArbAction, str] = {
    ArbAction.GRANT: SEVERITY.parse("info"),
    ArbAction.RELEASE: SEVERITY.parse("info"),
    ArbAction.PREEMPT: SEVERITY.parse("info"),
    ArbAction.LEASE_TIMEOUT: SEVERITY.parse("warn"),
    ArbAction.FORCED_PREEMPT: SEVERITY.parse("warn"),
    ArbAction.SOURCE_DEATH: SEVERITY.parse("fault"),
    # BIZ-CM-3 disarm actions (11 S7A.7 / 14 S3.6: suspend/rearm -> info).
    ArbAction.SUSPEND: SEVERITY.parse("info"),
    ArbAction.REARM: SEVERITY.parse("info"),
    # BIZ-CM-5 T-3 stuck-source ban: fault-level. A source that got three
    # forced_preempts inside 60 s is unreliable enough that operators need
    # to know NOW (14 S3.4 T-3: "此后一律 denied E_ARB_DISABLED").
    ArbAction.SOURCE_DISABLED: SEVERITY.parse("fault"),
}

# 11 S7A.7 / 14 S3.6: the actions that are NEVER merged. The urgent, rare ones.
# suspend / rearm join forced_preempt / source_death here (a soft-estop must not
# be collapsed into a count). source_disabled also joins per its trap-1 rule
# (a ban is a single event; merging two bans would mean two banned sources
# turned into one event that could be read as one ban).
DEDUP_EXEMPT = frozenset({
    ArbAction.FORCED_PREEMPT, ArbAction.SOURCE_DEATH,
    ArbAction.SUSPEND, ArbAction.REARM,
    ArbAction.SOURCE_DISABLED,
})

# 11 S7A.7: the merge window for the dedupable actions, seconds.
DEDUP_WINDOW_S = 10


def severity_of(action: str) -> str:
    """The severity for an ArbAction value string.

    Raises KeyError on an action with no mapping rather than defaulting, so a new
    action added without a severity is loud. The caller passes the .value string
    (that is what ArbEvent.action holds); it is turned back into the enum member
    to key the map.
    """
    return SEVERITY_BY_ACTION[ArbAction(action)]


def _is_exempt(action: str) -> bool:
    """Whether this action must never be merged (DEDUP_EXEMPT)."""
    return ArbAction(action) in DEDUP_EXEMPT


def render_audit_event(event: ArbEvent, count: int = 1) -> dict:
    """One ArbEvent -> the arbitration Event payload (11 S6.1 fields).

    dedup_key is arb:{domain}:{action} for every event; dedup_window_s is 0 for an
    exempt action (never merge) and DEDUP_WINDOW_S otherwise. count is the number
    of events this record stands for -- 1 for a single event, more when
    merge_audit_window collapsed a burst. eid / delivered are left to the event
    bus (trap 3).
    """
    action = event.action                               # the .value string
    # detail carries the arbitration context plus the merge count. The context
    # mirrors last_change so state and audit agree; the per-action extras
    # (overdue_ms for a forced preempt, held_ms for a release) are spread in on
    # top; count is written last so an extra named "count" can never shadow it.
    detail: Dict[str, object] = {
        "domain": event.domain,                         # which domain's arbiter
        "from": event.from_source,                      # prior holder, or null
        "to": event.to_source,                          # new holder, or null
        "reason": event.reason,                         # why (higher_priority, ...)
        "forced": event.forced,                         # T-1 forced preempt?
        "gen": event.gen,                               # arbitration generation
        "count": count,                                 # placeholder, re-set below
    }
    detail.update(event.detail)                         # per-action extras
    detail["count"] = count                             # authoritative: extras cannot win
    return {
        "sev": severity_of(action),                     # closed-set severity (S7A.7)
        "cat": AUDIT_CATEGORY,                           # "arbitration" (S6.2)
        "channel": AUDIT_CHANNEL,                        # "normal" (S2.2.11)
        # ASCII, factual title. The human one-liner for the HMI is the domain
        # state's holder.note, not this; keeping title token-based avoids putting
        # display prose in source (CLAUDE.md 2.1 / 2.2).
        "title": "arbitration %s on %s" % (action, event.domain),
        "detail": detail,                               # context + count
        "dedup_key": "arb:%s:%s" % (event.domain, action),   # S7A.7 key shape
        # exempt actions carry window 0 so merge_audit_window leaves them alone.
        "dedup_window_s": 0 if _is_exempt(action) else DEDUP_WINDOW_S,
    }


def merge_audit_window(events: List[ArbEvent]) -> List[dict]:
    """Collapse same-kind dedupable events within DEDUP_WINDOW_S into one each.

    The aggregator half of 11 S7A.7 "10 s 窗口合并, 合并次数进 detail.count". A
    pure function over ArbEvent (which carry mono_ms), so it needs no clock and is
    directly testable. Rules:
      * An exempt action (dedup_window_s 0) is never merged: each such event is
        rendered on its own with count 1 (trap 1).
      * For a dedupable action, events sharing arb:{domain}:{action} are grouped;
        within a group, a run whose first and last mono_ms are <= window apart
        collapses to ONE rendered event (the FIRST, so the time is when the burst
        began) carrying count = size of the run. A later event more than the
        window after the run's start opens a new run.
    Order of the returned list follows the first event of each emitted record, in
    input order, so a consumer sees bursts in the order they began.

    Pseudocode:
        for each event, in order:
            if exempt(event):                 emit(event, count=1)      # never merge
            else:
                key = arb:{domain}:{action}
                if an open run for key exists and event is within window of its
                   FIRST event:               run.count += 1            # extend
                else:                         flush(key); open a new run of 1
        flush every still-open run
        return emitted, restored to input order
    """
    window_ms = DEDUP_WINDOW_S * 1000                   # window in ms for the mono compare
    # (sort_key, rendered) pairs; sort_key is the first event's index so output
    # order matches input order of first-occurrences.
    out: List[tuple] = []
    # Per dedup_key: the open run's [first_event, first_index, first_mono, count].
    open_run: Dict[str, list] = {}

    def _flush(key: str) -> None:
        """Render and emit the open run for `key`, if any."""
        run = open_run.pop(key, None)
        if run is not None:
            first_event, first_index, _first_mono, count = run
            out.append((first_index, render_audit_event(first_event, count)))

    for idx, event in enumerate(events):
        if _is_exempt(event.action):
            # Never merged; emit immediately with count 1. Does not disturb any
            # open run of a different (dedupable) key.
            out.append((idx, render_audit_event(event, 1)))
            continue
        key = "arb:%s:%s" % (event.domain, event.action)
        run = open_run.get(key)
        if run is not None and event.mono_ms - run[2] <= window_ms:
            run[3] += 1                                 # extend the open run
        else:
            _flush(key)                                 # window closed: emit prior run
            open_run[key] = [event, idx, event.mono_ms, 1]   # open a new run
    for key in list(open_run):                          # emit whatever is still open
        _flush(key)
    out.sort(key=lambda pair: pair[0])                  # restore input order
    return [rendered for _idx, rendered in out]


def audit_severity_covers_every_action() -> Optional[str]:
    """Return None if SEVERITY_BY_ACTION covers every ArbAction, else the gap.

    The startup / meta guard for trap in the module docstring: a new ArbAction
    with no severity row would otherwise KeyError only the first time that action
    fires. Called by the report package's self-check and by the metatest.
    """
    missing = [a.value for a in ArbAction if a not in SEVERITY_BY_ACTION]
    if missing:
        return "ArbAction members with no severity: %s" % sorted(missing)
    return None
