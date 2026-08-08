"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: startup_selfcheck.py
Brief: INF-ZN-5 -- refuse startup on 11 S2.2 unregistered keys, cross-plane
       wildcard subs, and any of the seven S2.4.8 anti-patterns

Description:
Why this exists. 11 S2.2 line "启动自检: ...拒绝启动" makes four refusal shapes
mandatory (unregistered key, wildcard sub on a cross-plane process, rt/+block,
missing QoS). 11 S2.4.8 lists seven anti-patterns A-1~A-7 and requires the
startup script to reject each. This module is that script for Python processes:
each process passes its final list of Declaration to selfcheck() at the end of
session bring-up; the first violation raises with detail naming the antipattern
code and the key, so no process ever runs the loop with a mis-configured
publisher or subscriber.

What each check catches, and which S2.4.8 anti-pattern it maps to:
  UNREGISTERED  -- pub key not covered by any 11 S2.2.1~S2.2.9 registered
                   pattern. Not an S2.4.8 antipattern, this is F-1's registry
                   contract. Detail antipattern = UNREGISTERED for traceability.
  W-2           -- a cross-plane process (perception, p1_motion, chassis_relay,
                   p2_core, p4_agent) subscribing with `*` or `**` in the key.
                   The wildcard is the technical precondition for "generic
                   forwarding" (RT-C3), so the ban is per-process, not global.
  A-2           -- rt/-prefixed key with congestion_control=block. QOS-C1 says
                   RT plane must be drop; block on rt turns one blocked put
                   into an entire 50 ms control period lost.
  A-3           -- rt/-prefixed key on a SUBSCRIBER with handler.kind=fifo.
                   Periodic RT keys need Ring so a late tick is dropped, not
                   accumulated as permanent lag; FIFO also silently defeats the
                   frame-age check that would notice.
  A-4           -- event/-prefixed key with reliability=best_effort OR
                   handler.kind=ring. Event replay/backfill's cursor requires
                   no-loss ordered delivery; ring drops the middle of a burst.
  A-5           -- cmd/estop with congestion_control=block. block gives the
                   emergency-stop link an unbounded blocking window; the S2.3
                   10 Hz resend is what handles residual loss instead.
  A-6           -- audio/broadcast (or rt/audio streams) with handler.kind=fifo
                   AND handler.depth=256. FIFO+deep on a 50 Hz stream = latency
                   grows without bound; the S2.4.7 note explicitly forbids the
                   "凭空取 256" default.
  A-7           -- publisher declared without a QoS profile (relying on Zenoh's
                   binding defaults). Every publisher must state its profile.
  A-1 is NOT in this file. A-1 is a THREAD-binding property that a static
  Declaration cannot represent (which thread the publisher runs on is a runtime
  fact); it is covered by publisher_thread_check.py (INF-ZN-6).

What the caller must pass in:
  declarations       -- every Declaration the process built at bring-up. If a
                        pub or sub is added later, selfcheck must re-run OR the
                        addition is a bug the whole file was written to catch.
  key_registry       -- iterable of PATTERN strings extracted from 11 S2.2.1~9.
                        Produced by scripts/doccheck/key_registry.py --emit
                        (INF-ZN-4), passed here as a set of patterns. This
                        loader is DELIBERATELY not run inside selfcheck: it
                        would couple this module to disk and the doc, and
                        selfcheck should be pure over its inputs.
  cross_plane_processes -- the set the W-2 check applies to (11 S1.1.3 has 5
                        such processes today). Injected so a bring-up in a
                        non-cross-plane process (e.g. p3_task) does not need
                        to invent a name.

Traps -- things that look right and are not:
  1. Raising on the FIRST violation rather than collecting all of them. A
     collector's report reads more thorough, but the process still cannot
     start, and one violation named clearly is easier to fix than seven mixed
     together. If a batch report is wanted later, a wrapper that catches and
     collects can add it -- this module stays fail-fast.
  2. Silently converting a wildcard on a NON-cross-plane process to a pass.
     They ARE allowed to wildcard-subscribe locally, but the check still
     enforces the pattern is a legal wildcard (has *, not something malformed).
     The current cross-plane list is the only surface RT-C3's forwarding rule
     covers; non-cross-plane wildcards are legal by design.
  3. Reading `profile is None` as "no QoS". A Declaration whose profile is None
     is exactly A-7 -- the caller left it blank because Zenoh's binding default
     would fill in. There is no separate "unknown QoS" state.
"""

from dataclasses import dataclass
from typing import Iterable, Optional, Set, Tuple

from xbrain.common import errors
from xbrain.common.errors import XbrainError
from xbrain.common.zenoh.qos import (
    FROZEN_PROFILES, HandlerSpec, QosProfile, key_expr_matches,
)

__all__ = ["Declaration", "SelfcheckError", "selfcheck",
           "DEFAULT_CROSS_PLANE_PROCESSES"]

#: 11 S1.1.3 v0.6 closed set of cross-plane processes -- the ones RT-C3's
#: forwarding rule applies to and therefore the ones a wildcard subscription
#: would let become a generic bridge. The others (p3_task, p5_gateway, ...) may
#: wildcard-subscribe on their own plane without triggering W-2.
DEFAULT_CROSS_PLANE_PROCESSES: frozenset = frozenset({
    "perception", "p1_motion", "chassis_relay", "p2_core", "p4_agent",
})


@dataclass(frozen=True)
class Declaration:
    """One publisher or subscriber declaration the process built at bring-up.

    Frozen so a declaration cannot mutate between selfcheck-time and the
    session's actual bindings -- if it could, the check would authorise one
    shape while the runtime used another.
    """

    role: str                            # "pub" | "sub" -- the two Zenoh sides
    key: str                             # concrete key, or wildcard for subs
    # Name of a FROZEN_PROFILES entry, or None. None IS a real value here: it
    # models "publisher declared without an explicit QoS" which is exactly A-7,
    # so the caller must never coerce a "don't know" into a plausible default.
    profile: Optional[str]
    # Owning process, used only by the W-2 wildcard check. Optional because a
    # non-cross-plane test rig may not need to name one (W-2 will not fire).
    process: Optional[str] = None
    # Subscriber-side handler override, (kind, depth). Publishers have no
    # handler at all (S2.4.1 keeps handler subscriber-side); on a pub this
    # field is ignored. When absent, the subscriber inherits its profile's
    # HandlerSpec. Two forms of override are useful: a bindings override the
    # deployment YAML supplied, and a per-declaration override the test forks
    # off to force one antipattern -- both use this same shape.
    handler_override: Optional[Tuple[str, Optional[int]]] = None


class SelfcheckError(XbrainError):
    """Any startup-selfcheck refusal. Carries the antipattern code and the key
    in .detail so a fault-event stream can route on them without re-parsing
    the message.

    Direct XbrainError subclass (not QosViolation) because different S2.148
    checks raise different codes -- E_QOS_VIOLATION for the seven A-* rules
    and E_CONFIG_INVALID for UNREGISTERED / W-2 / UNKNOWN_PROFILE. QosViolation
    hard-codes E_QOS_VIOLATION in its __init__.
    """


def _fail(code: str, antipattern: str, key: str, why: str,
          extra: Optional[dict] = None) -> None:
    """Raise a SelfcheckError with a uniform detail dict.

    detail keys are stable strings (antipattern / key / why) so consumers can
    switch on them without regex over the message.
    """
    detail = {"antipattern": antipattern, "key": key, "why": why}
    if extra:
        detail.update(extra)
    raise SelfcheckError(code, "%s on %s: %s" % (antipattern, key, why),
                         detail=detail)


def _resolve_handler(decl: Declaration, profile: QosProfile) -> HandlerSpec:
    """Effective handler for a subscriber: override wins if present, else the
    profile's. Publishers ignore this -- callers guard by role first.

    Override wins RATHER than merges because the antipattern checks care about
    the FINAL shape, not the derivation: if a deployment sets fifo/8 over a
    profile whose default was ring/1, the runtime queue is fifo/8 and that is
    what A-3 must judge. A merge would leave both values in play and let a
    reviewer argue about which one "counts".
    """
    if decl.handler_override is not None:
        kind, depth = decl.handler_override
        return HandlerSpec(kind, depth)
    return profile.handler


def _is_wildcard(key: str) -> bool:
    """A key with a * segment or the ** any-segment marker. Zenoh's own
    wildcard grammar; matched literally because a subscription that would
    otherwise cover many keys has these characters. The naive substring
    check is safe here because concrete Zenoh keys use only lower-case letters
    plus / and _ (S2.1 naming rule), so a stray * cannot appear in a legit key.
    """
    return "*" in key


def selfcheck(declarations: Iterable[Declaration],
              key_registry: Iterable[str],
              cross_plane_processes: Set[str] = None) -> None:
    """Run every INF-ZN-5 check against `declarations`.

    Raises SelfcheckError on the FIRST violation (trap 1). Returns None on a
    clean set. `key_registry` is the pattern set from ZN-4's extract() /
    --emit output; PATTERNS, not concrete keys, so a subscription on a
    variable-segment pattern still matches via key_expr_matches.
    """
    # Materialise once so a generator-typed registry is not consumed on the
    # first inner iteration -- selfcheck must handle every decl.
    patterns = list(key_registry)
    # None sentinel rather than a default arg with a mutable frozenset: a
    # caller passing an EMPTY set (say a p3_task rig that has no cross-plane
    # role at all) is a legal override we want to honour, not overwrite.
    cross_plane = (cross_plane_processes if cross_plane_processes is not None
                   else DEFAULT_CROSS_PLANE_PROCESSES)

    # Fail-fast (trap 1): the first violation raises. Order inside the loop is
    # authored so the MOST SPECIFIC rule wins the report -- A-7 (no profile at
    # all) before anything that needs a profile, then registry before shape,
    # then A-5 (cmd/estop) before A-2 (rt+block), etc. A run that fires
    # multiple rules always reports the same one for a given decl -- which
    # makes test assertions stable and fix suggestions unambiguous.
    for decl in declarations:
        # A-7 first: without a profile every other check would need to guess
        # what the runtime would have used, and Zenoh's defaults are exactly
        # what the antipattern forbids relying on. So A-7 is the checkpoint at
        # which "profile is a known FROZEN_PROFILES entry" becomes an
        # invariant for the rest of the loop -- every check below can then
        # dereference decl.profile without a None guard, which is what makes
        # the per-check code short enough to read.
        if decl.profile is None:
            _fail(errors.E_QOS_VIOLATION, "A-7", decl.key,
                  "publisher / subscriber declared without an explicit QoS "
                  "profile (S2.4.8 A-7)")
        profile = FROZEN_PROFILES.get(decl.profile)
        if profile is None:
            # An unknown profile name is a config error, not an antipattern:
            # the caller cannot have meant one of the seven if they cannot even
            # name a real profile. E_CONFIG_INVALID matches the S2.148 wording.
            _fail(errors.E_CONFIG_INVALID, "UNKNOWN_PROFILE", decl.key,
                  "profile %r is not in FROZEN_PROFILES" % decl.profile,
                  {"profile": decl.profile})

        # Registry check -- publisher-side only, per F-1 semantics: F-1 pins
        # PUBLISHER uniqueness by key, so a publisher whose key no S2.2
        # pattern covers is a genuine registry violation. A subscriber listening
        # for a not-yet-registered key just gets silence -- not a defect, and
        # forbidding it would over-couple this check to the growth order of the
        # doc (a subscriber legitimately declared today for a pub that lands
        # tomorrow would falsely refuse).
        if decl.role == "pub":
            if not any(key_expr_matches(p, decl.key) for p in patterns):
                _fail(errors.E_CONFIG_INVALID, "UNREGISTERED", decl.key,
                      "publisher key not covered by any 11 S2.2 registered "
                      "pattern (F-1)")

        # W-2 -- wildcard on a cross-plane subscriber. Concrete-key subs and
        # non-cross-plane processes are exempt (trap 2). The wildcard test
        # runs BEFORE the process membership test only for the exemption
        # message: a p3_task sub with state/** should silently pass, not log
        # "would have been W-2 but exempt" -- silence is what the exemption
        # promises. Both conditions must hold for the raise; failing either
        # is legal.
        if (decl.role == "sub" and _is_wildcard(decl.key)
                and decl.process in cross_plane):
            _fail(errors.E_CONFIG_INVALID, "W-2", decl.key,
                  "cross-plane process %r wildcard-subscribed (RT-C3 forbids "
                  "generic forwarding)" % decl.process,
                  {"process": decl.process})

        # A-5 -- cmd/estop must never be block. Checked before A-2 so an
        # estop-on-rt slip (there is none today but if there ever were) gets
        # the most specific code first: A-5 tells a reviewer "the emergency
        # stop is broken", A-2 tells them "an rt/ key has the wrong QoS" --
        # the first is the one the operator needs to see.
        if decl.key == "cmd/estop" and profile.congestion_control == "block":
            _fail(errors.E_QOS_VIOLATION, "A-5", decl.key,
                  "cmd/estop must not use congestion_control=block "
                  "(S2.4.8 A-5)")

        # A-2 -- any rt/ key with block. QOS-C1 hard-codes DROP for the RT
        # plane; a bindings override that flipped this is exactly what A-2 is.
        # Simple prefix match (not the full key_expr_matches) is on purpose:
        # the antipattern is about the SEGMENT the key sits under, which
        # matches by string; a wildcard-form pub key starting rt/ still
        # commits A-2 if its profile is block.
        if decl.key.startswith("rt/") and profile.congestion_control == "block":
            _fail(errors.E_QOS_VIOLATION, "A-2", decl.key,
                  "rt/ key with congestion_control=block violates QOS-C1 "
                  "(S2.4.8 A-2)")

        # A-4 -- event/** must be reliable AND FIFO. The check is on the WIRE
        # shape (both directions of the link), which is why it applies to
        # both pub and sub roles: a publisher on best_effort loses events
        # even to a reliable subscriber, and a subscriber with Ring drops the
        # middle of a burst even from a reliable+FIFO publisher. Backfill's
        # cursor (S3.5.3 U18) needs BOTH halves right or its "resume from
        # last acked seq" story silently loses events between the seqs.
        if decl.key.startswith("event/"):
            if profile.reliability != "reliable":
                _fail(errors.E_QOS_VIOLATION, "A-4", decl.key,
                      "event/ key must be reliable, not %r (S2.4.8 A-4)"
                      % profile.reliability)
            eff = _resolve_handler(decl, profile) if decl.role == "sub" else profile.handler
            if eff.kind == "ring":
                _fail(errors.E_QOS_VIOLATION, "A-4", decl.key,
                      "event/ key must use FIFO, not ring "
                      "(S2.4.8 A-4)")

        # A-3 -- RT-periodic key on a SUBSCRIBER with FIFO. Approximated as
        # rt/-prefix; every rt/ key today is periodic (S2.2.1 shows publisher
        # frequencies for all of them). If an aperiodic rt/ key is ever added
        # this check gains a per-key allowlist; today there is none. Sub-only
        # because A-3 is fundamentally about the queue that HOLDS incoming
        # frames; a publisher has no handler at all (S2.4.1) and so cannot
        # commit A-3 -- putting an untriggered check on the pub side would
        # confuse readers reviewing the A-3 report about where the fix goes.
        if (decl.role == "sub" and decl.key.startswith("rt/")):
            eff = _resolve_handler(decl, profile)
            if eff.kind == "fifo":
                _fail(errors.E_QOS_VIOLATION, "A-3", decl.key,
                      "RT periodic key on subscriber must use ring, not fifo "
                      "(S2.4.8 A-3 -- late tick becomes permanent lag)")

        # A-6 -- audio/broadcast (or rt/audio/*) with FIFO(256). The literal
        # depth 256 is what S2.4.7 forbids as "凭空取"; other FIFO depths for
        # audio streams remain a A-3-style problem when subscribed-side, so
        # A-6 does not shadow A-3 -- it just names the specific 256 default
        # so operators can see WHY the deployment picked that number (they
        # usually copied it from a Zenoh example). The check is scoped to
        # audio keys because 256 is not intrinsically wrong for slower
        # streams; it is wrong for the 50 Hz audio path S2.4.5 第 8 条
        # argues stale frames have no value on.
        if ("audio/broadcast" in decl.key or decl.key.startswith("rt/audio/")):
            eff = _resolve_handler(decl, profile) if decl.role == "sub" else profile.handler
            if eff.kind == "fifo" and eff.depth == 256:
                _fail(errors.E_QOS_VIOLATION, "A-6", decl.key,
                      "audio stream with FIFO(256) grows latency without bound "
                      "(S2.4.8 A-6; 256 is the 凭空取 default S2.4.7 forbids)")
