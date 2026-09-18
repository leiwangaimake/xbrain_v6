"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: config_digest.py
Brief: P2 Stage A caches common_digest, Stage C/D holds motion on a mismatch

Description:
Implements the 10 S5.4.4 failure row verbatim: "P2 放行前 (S3.3.3 Stage D)
校验 MANIFEST.boot_id 与自身内存中的 common_digest; 不一致 -> 保持 "禁止运动",
发 event/fault/bit, detail.kind = "config_digest_mismatch"". The 10 S3.3.3
Stage C decision table lists the same comparison as its third criterion, and
10 S5.4.4's fault table gives it severity B (keep motion disallowed and
report; not R, which would refuse to start).

"自身内存中的" is the whole point and the reason this is not a tautology. The
value is captured ONCE, at Stage A, from the same load_resolved() call that
produced P2's configuration -- so the cached digest describes the snapshot P2
is actually running on. Comparing MANIFEST against itself would always agree
and would be CLAUDE.md S3.2 form 1. What the comparison catches is the
freeze line producing a SECOND pass that P2 did not read:
  * the freeze unit re-running while P2 was still starting, so P2's values and
    the MANIFEST on disk come from different passes;
  * an operator re-running the freeze by hand on a live machine, which is
    exactly what xbrain/boot/freeze/check.py tells them not to do;
  * a MANIFEST copied in from another machine or another boot.
In every one of those the running processes disagree about the configuration
while each one individually looks healthy -- the "各进程解析出不同结果"
outcome 10 S5.4.1 calls the reason the whole freeze design exists.

Severity is B and not R, and the difference is operational: refusing to start
would take the machine off the network along with the HMI that would have
said why. Holding motion leaves the operator a robot that talks and does not
move, which is the state 10 S3.3.3 calls BLOCKED.

An unreadable MANIFEST is treated the same way as a mismatch, under a
different detail.kind. That is a deliberate choice on the safe side: this
process's grant is the thing that lets a 2.0 m/s machine move, and continuing
to grant motion on a configuration that can no longer be attested is the
"放行" direction -- the one CLAUDE.md's asymmetry rule says must never fire by
accident. /run is tmpfs; a read failure there is not routine noise. The grant
is recomputed every tick, so a genuinely transient failure costs one second of
hold, not a mission.

What this does NOT do:
  * it does not re-verify the snapshot's own sha256. load_resolved already
    does that at Stage A and raises (R level) -- doing it again here would be
    a second implementation of the same check that could disagree with the
    first.
  * it does not read the config sources. Comparing against configs/ is
    freeze --check's job and it is a different question (snapshot vs source,
    not memory vs snapshot). Neither subsumes the other.
  * it does not decide what to publish. It returns a verdict; the wiring holds
    the grant and emits the event. A module that reached for the publisher
    could not be tested without a bus.

Caller obligations, because a verdict nobody acts on is worse than no verdict
(it looks like the check is in place):
  1. build the guard from the SAME ResolvedConfig the process is running on,
     at startup, before any grant is published;
  2. call check() once per grant tick and, when it blocks, force the published
     grant to allow_motion=false / speed_factor=0.0 -- forcing only one of the
     two leaves P1's HealthFactorSlot holding a speed for a motion it is not
     allowed, and the next message that flips allow_motion back would resume
     at that speed;
  3. emit event/fault/bit only on verdict.edge, not on verdict.blocked, or the
     fault repeats once per second and buries every other event.

Two spellings that look right and are not:
  * caching the digest by reading MANIFEST.json at startup instead of taking
    it from the ResolvedConfig. Two reads can see two different files, which
    is the very fault being guarded, and the guard would then compare the
    second read against itself.
  * treating "MANIFEST is newer than my cache" as acceptable because "the
    freeze line is authoritative". It is authoritative about what the NEXT
    process to start will read; it says nothing about what this one already
    loaded, and this one is the one holding the grant.
"""

import json
import os
from typing import Any, Dict, Optional

from xbrain.common.config.resolved import RESOLVED_ROOT

#: How often the MANIFEST is re-read, in seconds of monotonic time. Not every
#: tick: the 1 Hz grant loop is also the health publish path, and 10 S5.4.4's
#: own comment on P2's startup read is that a file read does not belong in it.
#: Ten seconds bounds the exposure of the one fault this catches (a second
#: freeze pass) to ten seconds of motion on a configuration P2 did not read,
#: which is the same order as the health ladder's own degrade timeout.
DEFAULT_CHECK_PERIOD_S = 10.0

#: detail.kind values, closed (CLAUDE.md 3.5). The first is 10 S5.4.4 verbatim;
#: the second is this module's own, for the case the design text does not name.
# 10 S5.4.4 verbatim, and it travels to p5 / record.db / the cloud unchanged,
# so the spelling is part of the contract rather than a local label.
KIND_MISMATCH = "config_digest_mismatch"
KIND_UNREADABLE = "config_manifest_unreadable"


class DigestVerdict:
    """What the guard concluded this tick.

    `edge` is what keeps the fault event from repeating once per second: the
    wiring emits on the transition, not on the state. Carried here rather than
    left to the caller because a caller that tracked it itself would be a
    second copy of the same state, and the two would drift the first time
    somebody added an early return to the loop.
    """

    __slots__ = ("blocked", "detail", "edge")

    def __init__(self, blocked: bool, detail: Optional[Dict[str, Any]],
                 edge: bool):
        self.blocked = blocked
        self.detail = detail
        self.edge = edge

    def __repr__(self) -> str:                       # pragma: no cover -- debug
        return "DigestVerdict(blocked=%r, edge=%r, detail=%r)" % (
            self.blocked, self.edge, self.detail)


#: detail.kind on the RECOVERY edge. 10 S5.4.4 names only the fault; the
#: cleared event needs a kind too, because p5 persists detail verbatim and a
#: missing key there is a hole in the record, not an absence of information.
KIND_OK = "config_digest_ok"


def digest_fault_event(verdict: "DigestVerdict", eid: str,
                       wall_ts: float) -> Dict[str, Any]:
    """The 10 S5.4.4 event/fault/bit body for one verdict EDGE.

    A function rather than an inline dict in the wiring, so the shape has a
    test that does not need a bus. The wiring's job is to decide WHEN to emit
    (on verdict.edge, never on verdict.blocked -- blocked stays true every tick
    until an operator restarts, and a fault per second buries record.db).

    wall_ts is passed in rather than read here. It is the 11 S6.2 display
    stamp and nothing else, and a module that reached for time.time() itself
    would trip the CLAUDE.md S3.4 scan that exists to keep wall clocks out of
    timeout logic -- the exemption belongs at the call site that can justify it.
    """
    return {
        "eid": eid,
        # Title carries the direction in words, because an operator scanning
        # an event list sees titles before details, and "config digest
        # mismatch" and "config digest consistent" are opposite situations
        # that would otherwise share one line of text.
        "title": ("config digest mismatch" if verdict.blocked
                  else "config digest consistent"),
        # On the recovery edge there is no detail (the comparison passed), so
        # the kind is supplied here. Emitting an empty dict would make a
        # consumer branch on presence rather than on value.
        "detail": verdict.detail if verdict.blocked else {"kind": KIND_OK},
        "src": "p2_core",
        "ts": wall_ts,
    }


class ConfigDigestGuard:
    """Stage A caches; every Stage C/D pass compares.

    One instance per process, built once. Not a module-level singleton: a
    module global would be shared by every test in a session and the second
    test would inherit the first one's cached digest, which is the kind of
    cross-test leakage that makes a guard look like it works.
    """

    # __slots__ so the cached digest cannot be joined later by a second field
    # that also claims to hold it. There is exactly one cached value and the
    # object's whole meaning is "this is what P2 is running on".
    __slots__ = ("_digest", "_boot_id", "_root", "_period_s", "_next_s",
                 "_last")

    def __init__(self, cached_digest: str, cached_boot_id: str, *,
                 resolved_root: str = RESOLVED_ROOT,
                 period_s: float = DEFAULT_CHECK_PERIOD_S):
        # No defaults for the two cached values: they are the whole content of
        # this object, and a guard constructed without them would compare
        # against a placeholder and agree with everything (CLAUDE.md 3.1 is
        # about safety params, and this is one -- it gates motion).
        self._digest = cached_digest
        # boot_id as well as the digest, because 10 S5.4.4 says "校验
        # MANIFEST.boot_id 与自身内存中的 common_digest" -- both. A MANIFEST
        # dropped in from another machine can carry a plausible digest; the
        # boot_id is what makes it obviously foreign.
        self._boot_id = cached_boot_id
        self._root = resolved_root
        self._period_s = period_s
        # -inf, so the first check is due for ANY float the caller passes,
        # negative readings included. An earlier draft used None plus an
        # `is not None` test in check(); a mutation run showed that seeding
        # 0.0 instead changed nothing for any non-negative clock, i.e. the
        # extra branch was distinguishable only by a monotonic reading below
        # zero, which does not occur. One seed that is total beats a branch
        # no test can reach (CLAUDE.md 7.2.1 on equivalent mutants).
        self._next_s = float("-inf")
        self._last = DigestVerdict(False, None, False)

    @classmethod
    def at_stage_a(cls, resolved: Any, *,
                   resolved_root: str = RESOLVED_ROOT,
                   period_s: float = DEFAULT_CHECK_PERIOD_S
                   ) -> "ConfigDigestGuard":
        """Build from the ResolvedConfig the process just loaded.

        Takes the loaded object rather than reading the MANIFEST again. Two
        reads could see two different files -- which is precisely the fault
        being guarded against -- and the guard would then be comparing the
        second read to itself while P2 ran on the first.
        """
        manifest = resolved.manifest
        return cls(manifest.common_digest, manifest.boot_id,
                   resolved_root=resolved_root, period_s=period_s)

    @property
    def cached_digest(self) -> str:
        """Exposed so a startup log line can name what P2 is running on. An
        incident report that says "digest mismatch" without saying which value
        was cached leaves the reader unable to tell which pass was the odd
        one."""
        return self._digest

    def check(self, now_mono_s: float) -> DigestVerdict:
        """The Stage C/D comparison. Cheap on most ticks (returns the cached
        verdict); re-reads the MANIFEST every period_s."""
        if now_mono_s < self._next_s:
            # Same conclusion as last time, and NOT an edge: re-reporting the
            # edge on every tick would make the wiring emit a fault per second.
            return DigestVerdict(self._last.blocked, self._last.detail, False)
        self._next_s = now_mono_s + self._period_s
        blocked, detail = self._compare()
        # Edge is on the BLOCKED transition in either direction. The recovery
        # edge matters as much as the fault one: an operator watching events
        # needs to see that the condition cleared, otherwise the only way to
        # learn it is to notice the robot moving again.
        edge = blocked != self._last.blocked
        self._last = DigestVerdict(blocked, detail, edge)
        return self._last

    # Returns a bare tuple rather than a DigestVerdict: check() owns the edge
    # computation, and a _compare that built a verdict would have to invent an
    # edge value that check() then discarded -- a field with a lie in it, even
    # briefly, is a field somebody eventually reads.
    def _compare(self) -> "tuple":
        """(blocked, detail). Reads MANIFEST.json directly rather than through
        load_manifest, because load_manifest raises (R level) on a stale
        boot_id and this comparison is a B-level report -- raising here would
        take down the process whose job is to report the condition."""
        # Joined rather than formatted, so a root given with or without a
        # trailing separator behaves the same -- the same reasoning
        # load_manifest gives for its own path construction.
        path = os.path.join(self._root, "MANIFEST.json")
        try:
            with open(path, encoding="utf-8") as fh:
                raw = json.load(fh)
            current_digest = str(raw["common_digest"])
            current_boot = str(raw["boot_id"])
        except (OSError, json.JSONDecodeError, KeyError, TypeError) as exc:
            # TypeError covers a JSON document that is a list or a scalar --
            # raw["common_digest"] on a list raises TypeError, not KeyError.
            return True, {"kind": KIND_UNREADABLE, "path": path,
                          "reason": "%s: %s" % (type(exc).__name__, exc)}
        if current_digest != self._digest or current_boot != self._boot_id:
            # Both values in the detail. "they differ" without the two values
            # cannot be acted on: the operator has to know whether the MANIFEST
            # moved forward (someone re-froze) or P2 is the stale one.
            return True, {"kind": KIND_MISMATCH,
                          "cached_digest": self._digest,
                          "manifest_digest": current_digest,
                          "cached_boot_id": self._boot_id,
                          "manifest_boot_id": current_boot}
        return False, None
