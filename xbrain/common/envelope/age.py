"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: age.py
Brief: message_age_s -- the one canonical age computation of 11 S3.0.1

Description:
What this solves. Every timeout in 11 S1.6 -- Tier 1 at 200 ms, scan at 300 ms,
link at 3 s -- is a threshold on a message's AGE, and S3.0.1 states there is one
way to compute that age for the whole system. If each consumer rolled its own,
they would disagree at exactly the boundaries that matter, and the disagreement
would read as a flaky sensor rather than as an arithmetic difference. This module
is that one implementation, transcribed from the S3.0.1 pseudocode branch for
branch, and its C++ twin in common/include/xbrain/envelope/message_age.h is held
to the same golden vectors so the two produce byte-identical ages.

The four branches, verbatim from S3.0.1:
  1. mono present AND boot == LOCAL_BOOT_ID -> age = now_mono - mono. The real
     produced age, including transport and queueing.
  2. boot present but != LOCAL_BOOT_ID -> fall back to now_mono - rx_mono. A
     monotonic reading from another host or another boot is not comparable
     (CLK-C4); using it anyway yields an absurd age.
  3. mono absent (a cloud publisher, which CLK-C4 requires to omit mono) -> the
     same receive-time fallback.
  4. age < 0 -> emit a warn / system event with detail.kind == "negative_age" and
     clamp to 0. CLK-C5: a negative age is the protection-failed direction, and
     treating it as "very fresh" would let a stale command through at the worst
     possible moment (the wall clock stepped, or a boot was misjudged as local).

Why the event is emitted through an INJECTED sink rather than a call to some
global emitter. Two reasons that reinforce each other. First, testability: the
whole value of injection in this codebase (CLAUDE.md 7.1) is that a test can
observe the effect without standing up a transport, and INF-CM-2's fourth
mutation -- negative age clamps but does NOT emit -- is only catchable if the
emission is observable. Second, layering: this package must not import zenoh or
the event bus (they are higher layers, and pulling them in would make age
uncomputable on a machine without them). So the caller wires the sink, and it is
a REQUIRED argument with no default: a None default would let a caller forget the
event and reintroduce the very silence CLK-C5 exists to break -- which is the
worst-case fail-silent CLAUDE.md 3.1 keeps closing.

What this module deliberately does NOT do:
  * It does not read any clock. now_mono and rx_mono are passed in, stamped by
    the caller from xbrain.common.clock. Reading a clock here would (a) make the
    computation impure and untestable without sleeping, and (b) invite a wall
    clock in through the back door -- the exact thing S3.0.1 forbids and
    clock_scan.py hunts for.
  * It does not decode the envelope. It takes an already-decoded Envelope, so ts
    and mono are typed and boot's presence invariant already holds. That is why
    the produced-age branch can read env.mono directly.
  * It does not know the tightening / loosening distinction. Age is computed the
    same way for a stop and for a mode switch; the fail-safe on a DECODE failure
    is directionality.py's concern, not this one's.
  * It does not persist rx_mono or mono. 15 SS-1: a monotonic reading is
    meaningless across a restart, so nothing here writes one to disk.

Traps that look right and are not:
  1. Using env.ts to compute age. ts is the wall clock, and S3.0 says in bold it
     is for cross-host alignment and latency statistics ONLY, never for age. The
     produced-age branch reads env.mono; INF-CM-2's first mutation swaps in
     env.ts, and the RTK first-lock wall-clock step is precisely when that
     mutation produces a wrong -- often hugely negative -- age.
  2. Keeping the mono branch when boot mismatches. boot is the validity domain of
     mono (CLK-C4). A reading from another boot subtracted against this host's
     now_mono is off by the difference of two unrelated epochs. The condition is
     "mono present AND boot matches", not "mono present"; dropping the boot half
     is INF-CM-2's third mutation.
  3. Clamping the negative age silently. The clamp is correct; dropping the event
     is not. detail.age_s carries the RAW negative value (before the clamp) so an
     operator can see how far negative it went -- the clamp is what the caller
     acts on, the event is what a human diagnoses from.
"""

from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

# EVENT_CATEGORY is the closed set from the shared library; "system" is validated
# against it AT IMPORT below, so a future rename of that category breaks this
# module loudly at startup rather than emitting an off-contract event at run time.
# CLAUDE.md 3.5: closed-set values come from the library, never as bare literals.
from ..enums import EVENT_CATEGORY
from .envelope import Envelope

# The two fixed labels for the branch actually taken. They are internal
# diagnostics -- carried on AgeResult and printed by the cross-language harness so
# a disagreement can be pinned to a branch -- NOT a contract closed set, so they
# are defined here rather than sourced from enums/. Named constants rather than
# bare strings so the Python and C++ sides spell them identically and a typo is
# one edit, not two.
BRANCH_PRODUCED = "produced"        # branch 1: mono present and boot matched
BRANCH_RX_FALLBACK = "rx_fallback"  # branches 2 and 3: receive-time fallback

# The negative-age event's fixed fields, from CLK-C5 and the S3.0.1 pseudocode.
#
# cat is validated through the closed set at import: EVENT_CATEGORY.parse raises
# if "system" ever leaves the set, so this constant cannot silently name a
# category the contract dropped.
NEGATIVE_AGE_CAT = EVENT_CATEGORY.parse("system")
# sev is "warn" per CLK-C5 ("上报 warn 事件"). There is deliberately NO parse()
# call here: the event severities info / warn / alarm / fault (11 S6.1) are NOT
# an exported closed set in xbrain.common.enums today, so there is nothing to
# validate against, and inventing a severity ClosedSet inside this module would
# create a second, competing source of truth for a set this item does not own.
# The literal stands, with this note, rather than a fabricated import.
NEGATIVE_AGE_SEV = "warn"
# The detail discriminator, S3.0.1 verbatim. A named constant so the emit site
# and any consumer that branches on it share one spelling.
NEGATIVE_AGE_KIND = "negative_age"


# frozen: an event handed to the sink must not be edited by the sink and then
# observed changed by a later handler. The three fields mirror emit_event's
# arguments in the S3.0.1 pseudocode exactly (sev, cat, detail).
@dataclass(frozen=True)
class NegativeAgeEvent:
    """The warn / system event CLK-C5 requires when an age comes out negative."""

    sev: str
    cat: str
    detail: Dict[str, Any]


# The callback type. It takes the event and returns nothing; the caller routes it
# to the event bus (on the loop thread) or, in a test, to a recorder. Kept as a
# plain Callable rather than a Protocol because the single method is call().
NegativeAgeSink = Callable[[NegativeAgeEvent], None]


# frozen result carrying everything a caller or a cross-language check needs
# WITHOUT re-running the computation: the clamped age (what a timeout compares
# against), the raw age (what the event reports), the branch taken, and whether
# the negative path fired. was_negative is a field rather than "raw_age_s < 0"
# recomputed by the caller so the one place that decides negativity is here.
@dataclass(frozen=True)
class AgeResult:
    """The outcome of the S3.0.1 computation, before any event is emitted."""

    age_s: float        # clamped to 0 when negative -- this is what a timeout uses
    raw_age_s: float    # the value before clamping -- what the event carries
    branch: str         # BRANCH_PRODUCED or BRANCH_RX_FALLBACK
    was_negative: bool  # True iff raw_age_s < 0 and the CLK-C5 path applies


def compute_age(env: Envelope, *, rx_mono: float, now_mono: float,
                local_boot_id: str) -> AgeResult:
    """The pure S3.0.1 computation: pick the branch, clamp, report -- no event.

    Separated from message_age_s so the branch and clamp logic can be golden-
    tested across languages without a sink, and so the event emission (the one
    side effect) sits in exactly one place above. rx_mono is a required argument,
    not defaulted from a clock read: it is the receiver's monotonic timestamp for
    THIS message, taken once when the message arrived (S3.0.1 step 1), and a
    later clock read would measure the wrong instant.
    """
    # Branch selection is the S3.0.1 if / else, transcribed. The produced-age
    # branch requires BOTH a monotonic reading AND that it belongs to this boot:
    # env.mono is not None rules out the cloud case, env.boot == local_boot_id
    # rules out the other-host / other-boot case (CLK-C4). Either failing drops to
    # the receive-time fallback.
    #
    # env.mono reads the monotonic field. Trap 1 in the header: swapping in env.ts
    # here is the wall-clock defect S3.0 forbids and INF-CM-2 mutation one.
    if env.mono is not None and env.boot == local_boot_id:
        raw_age = now_mono - env.mono
        branch = BRANCH_PRODUCED
    else:
        # Fallback covers both branch 2 (boot mismatch) and branch 3 (mono
        # absent). now_mono and rx_mono are both this host's monotonic clock in
        # this boot, so their difference is always comparable, which is the whole
        # reason the fallback exists.
        raw_age = now_mono - rx_mono
        branch = BRANCH_RX_FALLBACK

    # CLK-C5: a negative age is clamped to 0 and reported. The clamp and the flag
    # are both computed here; the EVENT is emitted by message_age_s, which is the
    # split that makes mutation four (clamp without event) observable.
    if raw_age < 0:
        return AgeResult(age_s=0.0, raw_age_s=raw_age, branch=branch,
                         was_negative=True)
    return AgeResult(age_s=raw_age, raw_age_s=raw_age, branch=branch,
                     was_negative=False)


def message_age_s(env: Envelope, *, rx_mono: float, now_mono: float,
                  local_boot_id: str, on_negative_age: NegativeAgeSink) -> float:
    """The 11 S3.0.1 age of `env` in seconds, with the CLK-C5 event wired in.

    Returns the clamped age (>= 0). When the raw age is negative it FIRST emits
    the warn / system negative_age event through on_negative_age, THEN returns 0 --
    both halves of CLK-C5, in that order.

    on_negative_age has no default on purpose. A None-defaulted sink would let a
    caller silently drop the event, which is exactly the fail-silent CLK-C5
    forbids; requiring the argument makes wiring the event a precondition of using
    this function at all.
    """
    result = compute_age(env, rx_mono=rx_mono, now_mono=now_mono,
                         local_boot_id=local_boot_id)
    if result.was_negative:
        # detail carries the RAW negative age, per the S3.0.1 pseudocode
        # (detail={kind, src, age_s: age} with age still negative). src comes from
        # the envelope so the operator sees which producer went backwards.
        event = NegativeAgeEvent(
            sev=NEGATIVE_AGE_SEV,
            cat=NEGATIVE_AGE_CAT,
            detail={"kind": NEGATIVE_AGE_KIND, "src": env.src,
                    "age_s": result.raw_age_s},
        )
        # Emitted BEFORE the return. Removing this call is INF-CM-2 mutation four:
        # the clamp below still happens (result.age_s is already 0), so the only
        # observable difference is the missing event -- which the recorder-sink
        # test asserts on.
        on_negative_age(event)
    return result.age_s


def read_local_boot_id(path: str = "/proc/sys/kernel/random/boot_id") -> str:
    """This host's boot id, first 8 hex chars, lower case -- the LOCAL_BOOT_ID.

    S3.0 defines boot as the first 8 hex of /proc/sys/kernel/random/boot_id. The
    file holds a UUID like 9f2c1a44-1b2c-...; the dashes are stripped before the
    slice so the result is 8 hex characters regardless of where a dash falls, and
    it is lower-cased so a comparison against a wire boot is case-stable.

    The path is a parameter, not hardcoded into the read, so a test can point it
    at a fixture without monkeypatching open(). It is NOT the CLAUDE.md 3.1
    default-value defect: this is a fixed OS path from S3.0, not a tunable safety
    number, and there is no configuration key for it.
    """
    # Read once, at startup. This is file I/O and must never sit in the 20 Hz
    # loop; callers cache the result. The read is not wrapped in a fallback: if
    # the boot id is unreadable the process should fail loudly here rather than
    # invent an id that would make every produced-age branch mismatch and silently
    # degrade the whole system to receive-time fallback.
    with open(path, encoding="ascii") as fh:
        raw = fh.read().strip()
    # Remove dashes, then take the first 8 hex characters, lower-cased. The
    # example in S3.0 ("9f2c1a44") is exactly this transform on a standard UUID.
    hex_only = raw.replace("-", "")
    boot = hex_only[:8].lower()
    # A short or empty id is a corrupt boot_id file, not something to pad or
    # accept: it would compare unequal to every real wire boot and quietly force
    # the fallback branch everywhere. Fail loudly instead.
    if len(boot) != 8:
        raise ValueError(
            "boot id at %r yielded %r, not 8 hex chars (11 S3.0)" % (path, boot)
        )
    return boot


__all__ = ["NegativeAgeEvent", "NegativeAgeSink", "AgeResult",
           "compute_age", "message_age_s", "read_local_boot_id",
           "BRANCH_PRODUCED", "BRANCH_RX_FALLBACK",
           "NEGATIVE_AGE_CAT", "NEGATIVE_AGE_SEV", "NEGATIVE_AGE_KIND"]
