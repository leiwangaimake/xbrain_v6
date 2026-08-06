"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_publisher_thread_check.py
Brief: INF-ZN-6 / CFG-BT-16 -- the A-1 in-process self-check, with its mutants

Description:
What these cases are worth. 11 S2.4.8 A-1 is a defect with no run-time symptom of
its own: a thread that publishes both a Q1 cmd_vel and a Q3 event keeps working,
the event's block just stalls the thread now and then, cmd_vel misses a 20 Hz
tick, and quadruped's Tier 1 locks the machine for a cmd_age it cannot explain.
10 S3.3.6 line 9 puts the detection inside each process, because which thread
publishes which key is a fact only the process holds -- so the check is the test,
the same way resolution is the test for INF-ZN-3.

Why real keys resolved through the real table, not fabricated resolutions. The
class of a publisher is read off its resolved QoS, and QOS-C1 is applied at
resolve time; a hand-built QosResolution could carry a Q3-on-rt with its block
still set, which the real resolver never produces. So every case here resolves an
actual key against the golden 11 S2.4.7 document -- cmd_vel, an event, estop, a
behavior request on the rt plane -- and the classification is exercised on what
the resolver actually returns.

*** The one design decision these cases pin, so a later reader does not undo it by
accident. A Q3 key on the rt plane (xbrain/dog-01/rt/behavior/request) resolves to
profile Q3_cmd, but QOS-C1 has stripped its block to drop. A-1's harm IS the block
stalling the thread; with the block gone the harm cannot occur, so this publisher
is deliberately NOT flagged even when it shares a thread with cmd_vel. That is not
a private ruling -- it is A-1's harm composed with QOS-C1's override -- and
test_q3_on_rt_plane_not_flagged asserts it. Classifying by profile name instead of
by the block knob would flag it, which is why the mutant list below includes that
edit and confirms it turns this case red.

The named mutant the item's criterion carries -- p1_motion publishing cmd_vel(Q1)
and a Q3 event on one thread must refuse to start -- is
test_same_thread_q1_and_q3_refuses_start. Beyond it, each of the following edits
was injected into publisher_thread_check.py and confirmed to turn this suite red
before being reverted; they are listed because a reader deciding whether to trust
these cases needs to know which defects they actually catch:
  * assert_no_a1 made a no-op (return None) -> the named case goes red;
  * the offender tagged by profile_name == "Q3_cmd" instead of the block knob ->
    test_q3_on_rt_plane_not_flagged goes red (the stripped Q3-rt is flagged);
  * the violation predicate changed to "two or more publishers on a thread"
    instead of "one on each side" -> test_two_realtime_same_thread_ok goes red
    (the Q0 safety chain CRL-6 mandates would be rejected);
  * assert_no_a1 raising on the first violating thread and stopping ->
    test_multiple_violating_threads_all_reported goes red;
  * "A-1" added to ASSERTION_F_ANTI_PATTERNS ->
    test_assertion_f_does_not_cover_a1 goes red.

What these cases do NOT establish, so a green run is not read as more than it is:
  * Nothing here opens a Zenoh session or measures a real stall. That a block
    publisher actually stalls a shared thread is QoS-T7 (11 S2.4.9), pending T7.
  * Nothing here decides A-2..A-7. Those are assertion F (INF-ZN-5 / FZ-6); this
    file only asserts, against the doc, that A-1 is not among them.
"""

import copy
import json
import os
import sys

import pytest

# Four dirnames from tests/common/zenoh/<file> up to the repository root, matching
# the sibling test modules exactly rather than inventing a second way to find it.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

# noqa: E402 on the project imports, same as the sibling modules: the sys.path
# insert above has to run before them, and moving them to the top would import
# from whatever xbrain happens to be installed rather than this checkout.
from xbrain.common.errors import E_QOS_VIOLATION  # noqa: E402
from xbrain.common.zenoh.qos import (BLOCK, FROZEN_PROFILES,  # noqa: E402
                                     load_qos_table)
from xbrain.common.zenoh.publisher_thread_check import (  # noqa: E402
    A1, ALL_ANTI_PATTERNS, ASSERTION_F_ANTI_PATTERNS, IN_PROCESS_ANTI_PATTERNS,
    REALTIME_PRIORITY, MixedQosThreadError, PublisherThreadRegistry, _tags,
    current_thread_name)

# The same golden document the resolver tests use: the 11 S2.4.7 profiles plus the
# ordered bindings. Reused rather than duplicated so a drift in the bindings shows
# up in one place, and so these cases classify publishers by the QoS the CONTRACT
# assigns them, not by a QoS invented for the test.
GOLDEN_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden",
                           "qos_bindings_vectors.json")
with open(GOLDEN_PATH, encoding="utf-8") as handle:
    GOLDEN = json.load(handle)
DOCUMENT = GOLDEN["qos"]

# The docs, read by the two source-honesty cases. Vol 10 holds the A-1/assertion-F
# split this module's partition constants transcribe; vol 11 is not needed here
# because the anti-pattern rows themselves are only counted, not quoted.
DESIGN10 = os.path.join(ROOT, "docs", "10-顶层设计.md")

# A representative key for each class, resolved once against the golden table. The
# comments record what each resolves to and why it is in the class it is, so a
# reader does not have to re-run the resolver to follow the cases.
TABLE = load_qos_table(DOCUMENT)
#: Q1_rt: drop + real_time. The 20 Hz command tick, and A-1's victim.
KEY_CMD_VEL = "xbrain/dog-01/rt/motion/cmd_vel"
#: Q3_cmd on the event plane: block. QOS-C1 does not touch the event plane, so the
#: block survives -- this is the offender the named mutant pairs with cmd_vel.
KEY_EVENT = "xbrain/dog-01/event/fault/bit"
#: Q0_safety: drop + real_time. On the victim side too, which is why A-1 covers a
#: Q0 publisher and not only a Q1 one.
KEY_ESTOP = "xbrain/dog-01/cmd/estop"
#: Q3_cmd on the rt plane: QOS-C1 rewrites its block to drop. Neither class -- the
#: case this suite exists to pin.
KEY_Q3_RT = "xbrain/dog-01/rt/behavior/request"
#: Q2_state: data_high, drop. Neither class; present so a case can show a thread
#: carrying a state publisher beside cmd_vel is fine.
KEY_STATE = "xbrain/dog-01/rt/chassis/state"


def res(key):
    """Resolve a key against the golden table. A thin name so cases read short."""
    return TABLE.resolve(key)


# ---------------------------------------------------------------------------
# Classification -- the two tags, read off real resolutions.
# ---------------------------------------------------------------------------

def test_cmd_vel_is_realtime_only():
    """Q1 cmd_vel tags realtime and not blocking: it is the victim, not the stall."""
    tags = _tags(res(KEY_CMD_VEL))
    assert "realtime" in tags
    assert "blocking" not in tags


def test_event_is_blocking_only():
    """A Q3 event on the event plane keeps its block and tags blocking only."""
    tags = _tags(res(KEY_EVENT))
    assert "blocking" in tags
    assert "realtime" not in tags


def test_estop_is_realtime():
    """Q0 estop is on the victim side, so A-1 must guard it as it guards Q1."""
    assert "realtime" in _tags(res(KEY_ESTOP))


def test_q3_on_rt_plane_is_neither():
    """QOS-C1 stripped this Q3's block, so it carries neither tag.

    Read directly off the resolution so the reason is visible: the profile is
    still Q3_cmd, but congestion_control is drop and rt_override_applied is true.
    The block -- the thing A-1's harm needs -- is gone.
    """
    resolution = res(KEY_Q3_RT)
    assert resolution.profile_name == "Q3_cmd"
    assert resolution.congestion_control != BLOCK
    assert resolution.rt_override_applied is True
    assert _tags(resolution) == frozenset()


def test_state_is_neither():
    """A Q2 state publisher is outside A-1 entirely."""
    assert _tags(res(KEY_STATE)) == frozenset()


# ---------------------------------------------------------------------------
# The check -- violations and the passes that keep it honest.
# ---------------------------------------------------------------------------

def test_same_thread_q1_and_q3_refuses_start():
    """*** The named mutant. cmd_vel(Q1) and a Q3 event on one thread -> refuse.

    This is the criterion's own scenario: p1_motion wiring an event publisher onto
    its 20 Hz control thread. The process must refuse to start with
    E_QOS_VIOLATION. A no-op assert_no_a1 makes this case red, which is what proves
    it tests the check and not merely the registry's bookkeeping.
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "control_loop")
    reg.register(res(KEY_EVENT), "control_loop")
    with pytest.raises(MixedQosThreadError) as excinfo:
        reg.assert_no_a1()
    # The code is the closed-set value, not the message: a consumer branches on it.
    assert excinfo.value.code == E_QOS_VIOLATION


def test_separate_threads_ok():
    """The same two publishers on different threads are the correct wiring.

    This is the pass the named case is the failure of. A check that raised on the
    mere presence of a Q1 and a Q3 anywhere -- ignoring the thread -- would fail
    here, so this case is what stops the fix from being "reject every event".
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "control_loop")
    reg.register(res(KEY_EVENT), "event_pump")
    reg.assert_no_a1()  # must not raise


def test_two_realtime_same_thread_ok():
    """Two real_time publishers on one thread is mandated, not forbidden.

    CRL-6 (11 S2.4.8 context) requires the whole Q0 safety chain to share one
    thread. cmd_vel(Q1) beside estop(Q0) on a single thread must pass; a check
    that flagged any thread with two publishers would reject the very wiring the
    contract demands.
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "control_loop")
    reg.register(res(KEY_ESTOP), "control_loop")
    reg.assert_no_a1()  # must not raise


def test_q0_and_q3_same_thread_refuses():
    """A-1 covers the Q0 side: estop(Q0) with a Q3 event on one thread -> refuse.

    Written because a check that looked for Q1 by name would miss Q0. The victim
    criterion is real_time priority, which Q0 carries, so this must raise.
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_ESTOP), "control_loop")
    reg.register(res(KEY_EVENT), "control_loop")
    with pytest.raises(MixedQosThreadError):
        reg.assert_no_a1()


def test_q3_on_rt_plane_not_flagged():
    """*** Pins the decision: a QOS-C1-stripped Q3-rt beside cmd_vel is NOT A-1.

    Both on one thread. Because rt/behavior/request has had its block removed by
    QOS-C1, it can no longer stall the thread, so A-1's harm is absent and the
    check must pass. Tagging by profile_name would flag it and turn this red --
    that mutant is in the header list.
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "control_loop")
    reg.register(res(KEY_Q3_RT), "control_loop")
    reg.assert_no_a1()  # must not raise


def test_state_beside_cmd_vel_ok():
    """A Q2 state publisher beside cmd_vel is not A-1: neither carries block."""
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "control_loop")
    reg.register(res(KEY_STATE), "control_loop")
    reg.assert_no_a1()  # must not raise


def test_empty_registry_ok():
    """No publishers is a pass, not a special case."""
    PublisherThreadRegistry().assert_no_a1()


def test_error_names_thread_and_both_keys():
    """The refusal prints the thread name and BOTH keys, and carries them in detail.

    The criterion is explicit: "打印线程名与两侧 key". The message is what a human
    reads; the detail is what a fault-event consumer diffs. Both are asserted so a
    later edit cannot drop one to tidy the other.
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "control_loop")
    reg.register(res(KEY_EVENT), "control_loop")
    with pytest.raises(MixedQosThreadError) as excinfo:
        reg.assert_no_a1()
    err = excinfo.value
    message = str(err)
    # The thread name, both keys, and the anti-pattern number are all in the line.
    assert "control_loop" in message
    assert KEY_CMD_VEL in message
    assert KEY_EVENT in message
    assert A1 in message
    # The structured detail carries the same, keyed by side so a consumer does not
    # have to parse prose. anti_pattern is present because 11 S13.15 asks for it.
    assert err.detail["anti_pattern"] == A1
    threads = err.detail["threads"]
    assert len(threads) == 1
    assert threads[0]["thread"] == "control_loop"
    assert threads[0]["realtime_keys"] == [KEY_CMD_VEL]
    assert threads[0]["blocking_keys"] == [KEY_EVENT]


def test_multiple_violating_threads_all_reported():
    """Every violating thread is in the one refusal, so one restart surfaces all.

    Two independent bad threads. A check that raised on the first and stopped
    would report one, and the operator would fix it, restart, and meet the second.
    Both must appear in detail.threads.
    """
    reg = PublisherThreadRegistry()
    reg.register(res(KEY_CMD_VEL), "loop_a")
    reg.register(res(KEY_EVENT), "loop_a")
    reg.register(res(KEY_ESTOP), "loop_b")
    reg.register(res(KEY_EVENT), "loop_b")
    with pytest.raises(MixedQosThreadError) as excinfo:
        reg.assert_no_a1()
    reported = {t["thread"] for t in excinfo.value.detail["threads"]}
    assert reported == {"loop_a", "loop_b"}


def test_keys_in_detail_are_sorted():
    """Two blocking keys on a thread come back sorted, so the detail is stable.

    Registration order is loop-then-other; the detail must not depend on it, or a
    reordering of declaration code would change a fault payload for no reason.
    """
    reg = PublisherThreadRegistry()
    # Register the later-sorting event first to prove the sort, not the input order.
    reg.register(res(KEY_EVENT), "control_loop")            # event/fault/bit
    reg.register(res("xbrain/dog-01/event/aaa/bbb"), "control_loop")
    reg.register(res(KEY_CMD_VEL), "control_loop")
    with pytest.raises(MixedQosThreadError) as excinfo:
        reg.assert_no_a1()
    blocking = excinfo.value.detail["threads"][0]["blocking_keys"]
    assert blocking == sorted(blocking)


def test_register_rejects_empty_thread():
    """An empty thread name is a caller bug and must fail loudly, not group blank.

    A ValueError, not an XbrainError: a bad argument is a programming defect that
    must reach the fault path, not a contract failure a caller might catch.
    """
    reg = PublisherThreadRegistry()
    with pytest.raises(ValueError):
        reg.register(res(KEY_CMD_VEL), "")


def test_current_thread_name_returns_running_thread():
    """The convenience helper names the running thread, for the on-thread wiring."""
    import threading
    assert current_thread_name() == threading.current_thread().name


# ---------------------------------------------------------------------------
# Meta-tests -- the assertion-F partition, and the frozen-table assumptions.
# ---------------------------------------------------------------------------

def test_assertion_f_does_not_cover_a1():
    """*** The item's meta-test: assertion F must NOT claim A-1.

    A-1 needs a runtime thread<->publisher fact that config-static F cannot see,
    so an F that listed A-1 would be claiming to decide something its inputs do
    not contain. The partition is asserted three ways: A-1 is the in-process one,
    A-1 is not in F, and the two halves cover all seven rows with no overlap.
    Adding "A-1" to ASSERTION_F_ANTI_PATTERNS turns this red.
    """
    assert A1 in IN_PROCESS_ANTI_PATTERNS
    assert A1 not in ASSERTION_F_ANTI_PATTERNS
    in_process = set(IN_PROCESS_ANTI_PATTERNS)
    assertion_f = set(ASSERTION_F_ANTI_PATTERNS)
    # Disjoint: nothing is claimed by both checks.
    assert in_process & assertion_f == set()
    # Complete: every 11 S2.4.8 row is owned by exactly one of the two halves.
    assert in_process | assertion_f == set(ALL_ANTI_PATTERNS)


def test_partition_matches_design_doc():
    """The partition constants are the doc's, not this module's invention.

    10 S3.3.6 line 8 gives A-2..A-7 to assertion F; line 9 gives A-1 to the
    in-process check with E_QOS_VIOLATION. This case reads those two rows out of
    vol 10 and requires them to say exactly that, so the constants cannot drift
    from the section they transcribe (CLAUDE.md 3.2 form 4).
    """
    with open(DESIGN10, encoding="utf-8") as handle:
        lines = handle.readlines()
    # Line 8: a row naming A-2 ~ A-7 and 断言 F together -- F covers the static six.
    f_row = [ln for ln in lines if "A-2 ~ A-7" in ln and "断言 F" in ln]
    assert len(f_row) == 1, "10 S3.3.6 line 8 (A-2~A-7 -> assertion F) not found verbatim"
    # Line 9: a row naming A-1, the in-process self-check, and E_QOS_VIOLATION.
    a1_row = [ln for ln in lines
              if "A-1" in ln and "各进程自身启动内自检" in ln and "E_QOS_VIOLATION" in ln]
    assert len(a1_row) == 1, "10 S3.3.6 line 9 (A-1 -> in-process check) not found verbatim"


def test_frozen_table_class_assumptions():
    """The classifier's assumptions about FROZEN_PROFILES, pinned so a change forces a revisit.

    The victim tag is real_time priority and the offender tag is block congestion.
    Those are the right criteria only because, in 11 S2.4.2, exactly Q0_safety and
    Q1_rt carry real_time and exactly Q3_cmd carries block. If a later contract
    adds a real_time profile or a second block profile, this case goes red and the
    classification in publisher_thread_check must be revisited rather than silently
    mis-tagging the new profile.
    """
    realtime_profiles = {p.name for p in FROZEN_PROFILES.values()
                         if p.priority == REALTIME_PRIORITY}
    blocking_profiles = {p.name for p in FROZEN_PROFILES.values()
                        if p.congestion_control == BLOCK}
    assert realtime_profiles == {"Q0_safety", "Q1_rt"}
    assert blocking_profiles == {"Q3_cmd"}
    # And the two are disjoint, which is why _tags can never have to resolve a
    # publisher that is both -- the single-publisher A-1 the set return guards
    # against does not occur in the frozen table.
    assert realtime_profiles & blocking_profiles == set()


if __name__ == "__main__":
    # Runnable directly as well as under pytest, matching the sibling modules.
    raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
