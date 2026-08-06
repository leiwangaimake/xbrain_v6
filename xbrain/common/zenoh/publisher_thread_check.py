"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: publisher_thread_check.py
Brief: The in-process A-1 self-check -- one thread may not publish both Q1/Q0 and Q3

Description:
What problem this solves. 11 S2.4.8 A-1 (grep "同一线程既发 Q1/Q0 又发 Q3")
forbids a single thread from publishing both a Q1/Q0 key and a Q3 key. The reason
is written into the row: the priority queues are separate but the THREAD is
shared, so Q3's block congestion control stalls the thread and drags down the
cmd_vel publish that shares it. resolve()'s own docstring in qos.py states the
same mechanism from the other side -- event publication is pushed onto its own
thread "precisely so a Q3 block cannot stall a Q1 publisher". A stalled cmd_vel
misses a 20 Hz tick; quadruped then sees cmd_age over 200 ms and Tier 1 locks the
machine. Nothing about that is visible in the code, which is why it needs a check.

Why this is a SEPARATE check from assertion F, and why it lives per-process.
10 S3.3.6 (grep "启动失败的分类处理") splits the seven anti-patterns by where they
can be detected. Line 8 gives A-2..A-7 to assertion F, the config-static
self-check in xbrain-config-freeze.service: each of those can be decided by
reading qos.bindings alone (an rt/ key with block, an event key with best_effort,
a publisher with no QoS at all). Line 9 gives A-1 to "各进程自身启动内自检 --
线程<->publisher 绑定只有进程自己知道": which thread will publish which key is a
fact only the process holds, so no amount of config inspection can find it. That
is why this module exists at all, and why the meta-test asserts A-1 is NOT in
assertion F's coverage set -- an F that claimed to cover A-1 would be claiming to
decide something its inputs do not contain (CLAUDE.md 3.2 form 1).

How a publisher is classified, and why by resolved KNOBS and not by profile name.
A-1 names the classes "Q1/Q0" and "Q3", but this module tags a publisher from the
QoS its key RESOLVES to, read straight off qos.QosResolution:
  * blocking side (the Q3 offender): congestion_control == block. qos.py records
    that Q3_cmd is "the only frozen profile carrying block", and _apply writes the
    QOS-C1 trigger as "congestion == BLOCK" rather than "profile is Q3_cmd" for
    the express purpose of not leaving a future block-carrying profile a way onto
    the RT plane by not having that name. Tagging by the block knob inherits that
    property: it is the block itself -- the thing that stalls the thread -- that
    is detected, so a hypothetical second block profile is caught too.
  * real-time side (the Q1/Q0 victim): priority == real_time. In 11 S2.4.2 exactly
    Q0_safety and Q1_rt carry real_time, and QOS-C1's override sets priority to
    interactive_high, never real_time, so a resolution reading real_time is a
    Q0/Q1 publisher and no other. real_time -- not express -- is the criterion
    because the harm is a STALL of a tick that must run every period, and it is
    the real_time priority that marks that hard-real-time class; express is a
    batching-latency knob and a different concern.
The FROZEN_PROFILES meta-test in the test module pins both facts, so if a later
contract adds a real_time profile or a second block profile the classification
here is forced to be revisited rather than drifting.

*** Why a Q3 key on the rt plane is deliberately NOT flagged. xbrain/dog-01/rt/
behavior/request resolves to profile Q3_cmd, but QOS-C1 (11 S2.4.3, "rt/ 前缀一律
drop, 不得 block") strips its block to drop at resolve time -- rt_override_applied
is true and congestion_control is drop. A-1's stated harm is block stalling the
thread; with the block gone the harm is structurally absent, so this publisher
carries neither tag and does not trip the check. This is not a judgement this
module makes on its own (CLAUDE.md 9.1); it is the composition of two contract
rules -- A-1's harm is block, QOS-C1 removes block on rt -- and the test module
documents it with rt/behavior/request co-located with cmd_vel and asserts no
violation.

What this module deliberately does NOT do:
  * It does not resolve keys. The caller resolves through QosTable.resolve and
    hands the resolution here; this module reads two of its fields. Resolving
    internally would couple the check to a live table and duplicate the door
    checks parse_full_key already did to produce the resolution.
  * It does not discover threads. 10 S3.3.6 line 9 is explicit that the binding is
    the process's own knowledge, so the publishing thread is named by the caller,
    not inferred here. current_thread_name() is offered for the common case where
    a publisher is created on the thread that will publish it, but the caller
    opts into it by calling it -- there is no silent capture, because a capture on
    the wrong (setup) thread would group every publisher together and either miss
    a real defect or invent one.
  * It does not check A-2..A-7. Those are assertion F (INF-ZN-5 / FZ-6). The
    partition constants below are the single source F consumes for its own
    coverage; F must not re-declare the set (CLAUDE.md 3.7).
  * It does not open sessions, read configuration, or import zenoh or any ROS
    type -- the same boundaries the rest of this package keeps, because its
    consumers include the emergency-stop path (CLAUDE.md 5.3).

Traps that look correct and are not:
  * Flagging any thread with two publishers. CRL-6 (11 S2.4.8 context, grep
    "同属 Q0 安全链路") REQUIRES the whole Q0 safety chain to share one thread, so
    two real_time publishers on a thread is mandated, not forbidden. The check
    fires only when a thread carries a block publisher AND a real_time one.
  * Reporting the first violating thread and stopping. A second bad thread would
    then hide until the first is fixed, costing a restart per defect. assert_no_a1
    collects every violating thread so one refusal surfaces them all.
  * Tagging by profile_name. It reads correctly until QOS-C1 or a future profile
    makes name and knob disagree, and then it is wrong in the silent direction --
    see the rt/behavior/request paragraph above.
"""

# Standard library only, matching the rest of this package: this module is
# reachable from every runtime process through xbrain.common.zenoh, and a
# third-party import here would turn one missing wheel into a startup failure for
# all of them. threading is used only to READ the current thread's name for the
# convenience helper; nothing here starts, joins or synchronises a thread.
import threading
from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Sequence, Tuple

# The code is imported, never spelled (CLAUDE.md 3.5). 11 S13.15 files A-1..A-7
# all under E_QOS_VIOLATION, and its row says 不得降级放行 -- there is no warn
# level for this, so the only outcome is a raise that refuses the process start.
from ..errors import E_QOS_VIOLATION
from ..errors.exceptions import XbrainError
# BLOCK is the block congestion-control literal, defined once in qos.py so the
# QOS-C1 trigger, the set-clause prohibition and the messages there cannot drift
# apart. Imported so the offender test compares against the same value the
# resolver wrote. PRIORITIES is the frozen priority closed set, imported to guard
# the real_time literal below. QosResolution is the input this module reads.
from .qos import BLOCK, PRIORITIES, QosResolution

# ---------------------------------------------------------------------------
# The anti-pattern partition, sourced from 10 S3.3.6 lines 8 and 9.
#
# These are doc anchors (the reference numbers of 11 S2.4.8 rows), not error
# codes and not a closed-set enum, so they are named here rather than imported
# from the shared library. They exist as constants for one reason: the meta-test
# has to state, in code, that A-1 is the in-process check and is NOT part of
# assertion F -- and FZ-6, when it lands, must read ASSERTION_F_ANTI_PATTERNS
# from here for its own coverage instead of writing the set a second time
# (CLAUDE.md 3.7, the enumeration-kept-in-sync-by-hand failure).
# ---------------------------------------------------------------------------

#: 11 S2.4.8 has seven rows, A-1 through A-7. Written as the whole table so the
#: completeness assertion below can prove the two halves cover it with nothing
#: left unowned -- an anti-pattern in neither half is an anti-pattern no check
#: runs.
ALL_ANTI_PATTERNS: Tuple[str, ...] = ("A-1", "A-2", "A-3", "A-4",
                                      "A-5", "A-6", "A-7")

#: 10 S3.3.6 line 8: A-2..A-7 are "静态可判" and belong to assertion F, the
#: config-static self-check in xbrain-config-freeze.service. This module does
#: NOT implement them; it publishes the set so F has one place to read it.
ASSERTION_F_ANTI_PATTERNS: Tuple[str, ...] = ("A-2", "A-3", "A-4",
                                              "A-5", "A-6", "A-7")

#: 10 S3.3.6 line 9: A-1 alone is the "各进程自身启动内自检". A tuple of one, not a
#: bare string, so the union and intersection below are set operations over two
#: sequences of the same shape and a reader does not have to special-case it.
IN_PROCESS_ANTI_PATTERNS: Tuple[str, ...] = ("A-1",)

#: The single row this module enforces. Carried into the failure detail because
#: 11 S13.15's E_QOS_VIOLATION row asks for "命中的反模式编号" beside the key.
A1: str = "A-1"

# ---------------------------------------------------------------------------
# Publisher classification.
# ---------------------------------------------------------------------------

#: 11 S2.4.2 priority carried by Q0_safety and Q1_rt, and by no other frozen
#: profile. Named here rather than reached at PRIORITIES[0], because an index is
#: an assumption about tuple order that a reader cannot see and an edit can break.
REALTIME_PRIORITY: str = "real_time"

# Guard, run once at import. If a future 11 S2.4.2 renames real_time, this literal
# would silently stop matching, _tags would never mark the victim side, and A-1
# would quietly stop firing -- the exact fail-silent shape CLAUDE.md 3.2 warns
# about. Checking membership against the imported closed set turns that into a
# loud failure at import time instead. qos.py does the same with PLANE.parse("rt").
if REALTIME_PRIORITY not in PRIORITIES:
    raise RuntimeError(
        "REALTIME_PRIORITY %r is not in qos.PRIORITIES; 11 S2.4.2 priority "
        "names changed and the A-1 victim-side classification must be "
        "revisited" % (REALTIME_PRIORITY,))

#: The two class tags A-1 talks about. Strings, so a failure detail can carry
#: them unchanged, and module-private so nothing outside builds a publisher's
#: class by any route but _tags.
_TAG_BLOCKING = "blocking"   # the Q3 offender: its block stalls the shared thread
_TAG_REALTIME = "realtime"   # the Q0/Q1 victim: the tick that must not be stalled


# _tags -- the class membership of one resolved publisher, as a set.
#
# A set and not a single label, for two reasons. First, a publisher can in
# principle carry both tags (a block AND real_time resolution); no frozen profile
# does -- the meta-test proves congestion==block and priority==real_time never
# coincide -- but if one ever did, that single publisher would BE the A-1 defect
# on its own (it stalls the very tick it is), and returning a set lets the check
# below catch it without a special case. Second, a publisher that is neither (Q2,
# Q4, or a QOS-C1-stripped Q3-rt) returns the empty set, which the check reads as
# "does not participate in A-1" without a third sentinel value.
def _tags(resolution: QosResolution) -> FrozenSet[str]:
    """The A-1 class tags of a resolved publisher: blocking, realtime, or neither."""
    tags = set()
    # Offender first only for readability; the two conditions are independent, so
    # order changes nothing. congestion_control is post-override: a Q3 on the rt
    # plane has already had its block rewritten to drop by _apply, so it lands
    # here as drop and is correctly NOT tagged blocking -- see the header.
    if resolution.congestion_control == BLOCK:
        tags.add(_TAG_BLOCKING)
    # priority is also post-override, and QOS-C1 sets it to interactive_high, so
    # only an un-overridden Q0/Q1 reads real_time here. That is the whole victim
    # side and nothing else.
    if resolution.priority == REALTIME_PRIORITY:
        tags.add(_TAG_REALTIME)
    return frozenset(tags)


# ---------------------------------------------------------------------------
# The failure.
# ---------------------------------------------------------------------------

# A dedicated type rather than reusing qos.QosViolation, even though both carry
# E_QOS_VIOLATION. QosViolation means "a key cannot be given a QoS the frozen
# table guarantees" (a malformed or unbound key); this means "the process wired a
# thread to publish two incompatible classes". They need different reader actions
# -- one is a bad key, one is a bad thread assignment -- so a handler that wants
# to tell them apart at the catch site can, and this one carries the structured
# detail 11 S13.15 asks for, which QosViolation's message-only constructor does
# not.
@dataclass(frozen=True)
class ThreadMix:
    """One offending thread: the keys that put it on each side of A-1.

    Frozen and carried in the exception so a test (and a fault-event consumer)
    can read exactly which keys collided without parsing the message string.
    """

    #: The caller-supplied publishing-thread name, printed verbatim.
    thread: str
    #: The real_time (Q0/Q1) publisher keys on this thread, sorted for a stable
    #: message. Non-empty by construction -- a ThreadMix is only built for a
    #: thread that has at least one key on each side.
    realtime_keys: Tuple[str, ...]
    #: The block (Q3) publisher keys on this thread, sorted likewise.
    blocking_keys: Tuple[str, ...]


class MixedQosThreadError(XbrainError):
    """A thread publishes both Q1/Q0 (real_time) and Q3 (block) -- 11 S2.4.8 A-1.

    The process must refuse to start (10 S3.3.6 line 9 classifies A-1 as class R,
    "该进程拒绝启动 -> 连带整栈"), so this is raised, never logged-and-continued:
    11 S13.15 gives E_QOS_VIOLATION retryable = no and 不得降级放行.
    """

    def __init__(self, mixes: Sequence[ThreadMix]):
        # Every violating thread is named in one message, so a single refusal
        # surfaces all of them and the operator does not fix one, restart, and
        # meet the next. The detail is structured for a fault-event consumer; the
        # message repeats it in English for the log line a human actually reads.
        parts: List[str] = []
        for mix in mixes:
            parts.append(
                "thread %r publishes Q1/Q0 (real_time) keys %s and Q3 (block) "
                "keys %s"
                % (mix.thread, list(mix.realtime_keys), list(mix.blocking_keys)))
        message = ("11 S2.4.8 %s: a thread must not publish both Q1/Q0 and Q3; "
                   "%s. Split event and command publication onto separate "
                   "threads." % (A1, "; ".join(parts)))
        # detail carries the anti-pattern number and the per-thread key lists.
        # 11 S13.15's E_QOS_VIOLATION row: "detail 应给出命中的反模式编号与 key".
        detail = {
            "anti_pattern": A1,
            "threads": [
                {"thread": mix.thread,
                 "realtime_keys": list(mix.realtime_keys),
                 "blocking_keys": list(mix.blocking_keys)}
                for mix in mixes
            ],
        }
        super().__init__(E_QOS_VIOLATION, message, detail)


# ---------------------------------------------------------------------------
# The registry.
# ---------------------------------------------------------------------------

# A single frozen record per registered publisher. Frozen so a registration
# cannot be edited after the fact into a different thread or class, which would
# let the check pass on data no publisher ever declared.
@dataclass(frozen=True)
class _Entry:
    thread: str
    key: str
    tags: FrozenSet[str]


class PublisherThreadRegistry:
    """Records each publisher's (resolution, publishing-thread) and runs A-1.

    One per process, populated as the process declares its publishers, with
    assert_no_a1() called once at the end of startup. The registry holds only
    what the check needs -- the key, the class tags, and the caller's name for the
    thread that will publish it -- because 10 S3.3.6 line 9 is explicit that the
    thread<->publisher binding is knowledge the process supplies, not something
    this module can discover.
    """

    def __init__(self) -> None:
        # Insertion order is preserved and used: assert_no_a1 reports violating
        # threads in the order they were first seen, so a given wiring produces a
        # stable message a test can assert on.
        self._entries: List[_Entry] = []

    # register -- declare that `thread` will publish the key `resolution` is for.
    #
    # It takes the resolution, not a bare key, because the class tags come from
    # the resolved knobs and only QosTable.resolve can produce them (it applies
    # QOS-C1, without which a Q3-rt would be misclassified). resolution.key is
    # used as the publisher key, so the key and the QoS it was resolved to can
    # never be a mismatched pair passed in two arguments.
    def register(self, resolution: QosResolution, thread: str) -> None:
        """Record one publisher on a named thread. Call once per publisher."""
        # thread is validated non-empty because it is the grouping key: an empty
        # string would silently merge publishers the caller meant to keep on
        # distinct threads (or print a blank thread name), so a caller bug here
        # must fail loudly rather than corrupt the grouping. A bad argument is a
        # programming defect, so it is a plain ValueError that reaches the fault
        # path, not an XbrainError the caller might catch as a contract failure.
        if not isinstance(thread, str) or not thread:
            raise ValueError("thread must be a non-empty str, got %r" % (thread,))
        # _tags reads resolution.congestion_control and .priority; a non-resolution
        # object raises AttributeError here, which is the correct outcome -- it is
        # our own process misusing its own API, and it must not be swallowed.
        self._entries.append(
            _Entry(thread=thread, key=resolution.key,
                   tags=_tags(resolution)))

    # assert_no_a1 -- the self-check itself, run once at end of startup.
    #
    # Groups the registered publishers by thread and raises if any single thread
    # carries a key on both sides of A-1. Raising (not returning a bool) is
    # deliberate: 11 S13.15 forbids degrading this to a warning, so there is no
    # "check returned false, carry on" path to write by accident.
    def assert_no_a1(self) -> None:
        """Raise MixedQosThreadError if any thread mixes Q1/Q0 and Q3 publishers."""
        # Grouping preserves first-seen thread order (dict keeps insertion order),
        # so the eventual message is deterministic for a given registration order.
        by_thread: Dict[str, List[_Entry]] = {}
        for entry in self._entries:
            by_thread.setdefault(entry.thread, []).append(entry)

        mixes: List[ThreadMix] = []
        for thread, entries in by_thread.items():
            # sorted so the two key lists are stable regardless of the order the
            # publishers on this thread were registered -- a message a test can
            # pin, and a detail a consumer can diff.
            realtime_keys = sorted(e.key for e in entries
                                   if _TAG_REALTIME in e.tags)
            blocking_keys = sorted(e.key for e in entries
                                   if _TAG_BLOCKING in e.tags)
            # The A-1 predicate: a thread is in violation only when it has at
            # least one publisher on EACH side. A thread with only real_time keys
            # (the Q0 safety chain CRL-6 mandates) or only block keys (an event
            # pump) is fine; it is the mix that stalls a tick.
            if realtime_keys and blocking_keys:
                mixes.append(ThreadMix(
                    thread=thread,
                    realtime_keys=tuple(realtime_keys),
                    blocking_keys=tuple(blocking_keys)))

        # One raise for all violators. An empty mixes list is the pass: the
        # method returns None and the process goes on to start.
        if mixes:
            raise MixedQosThreadError(mixes)


# current_thread_name -- the convenience for the common wiring.
#
# When a process creates a publisher on the very thread that will publish it,
# threading.current_thread().name IS the publishing-thread name A-1 wants. This
# helper is offered so that case reads register(res, current_thread_name())
# rather than reaching into threading at every call site. It is a FUNCTION the
# caller invokes, not an automatic default inside register, on purpose: a default
# capture would run on whatever thread happened to call register -- often a setup
# thread, not the publishing one -- and group every publisher together, which
# either hides a real A-1 or fabricates one. Making the caller ask for it keeps
# that decision visible.
def current_thread_name() -> str:
    """The running thread's name, for callers registering from their pub thread."""
    return threading.current_thread().name


# Listed explicitly rather than left to a star-export default, matching the
# package convention: every name here is the public surface, and __all__ is the
# declaration that the private helpers (_tags, _Entry) are not.
__all__ = ["ALL_ANTI_PATTERNS", "ASSERTION_F_ANTI_PATTERNS",
           "IN_PROCESS_ANTI_PATTERNS", "A1", "REALTIME_PRIORITY",
           "ThreadMix", "MixedQosThreadError", "PublisherThreadRegistry",
           "current_thread_name"]
