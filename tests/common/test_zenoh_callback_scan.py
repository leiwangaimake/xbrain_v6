"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_zenoh_callback_scan.py
Brief: Metatests for the CFG-CM-17 callback scan, including its named mutation

Description:
What these cases are worth. scripts/lint/zenoh_callback_scan.py is the static
half of CLAUDE.md 4.2: no create_task, no asyncio queue put_nowait, no await, no
direct event_bus.publish inside a Zenoh subscriber callback. A lint reporting
zero is indistinguishable from a lint that reports zero on everything until
something proves it reacts, and comment_ratio.py in this repository was silently a
no-op through an entire fix for exactly that reason. So this file drives the
scanner rather than trusting it: it runs the probes the scanner carries, it feeds
the mutation CFG-CM-17 names and asserts red, it feeds the compliant twin and
asserts green, and it pins the scan surface so the check cannot be quietly
narrowed to nothing.

The load-bearing pair is test_the_named_mutation_turns_the_scan_red and
test_the_compliant_registry_is_clean. The first is CFG-CM-17's mutation two
written out as a permanent case: create_task placed in on_message, reached
through the registry's functools.partial. The second is the same registry with
publish_threadsafe, and it exists so the first proves the scanner discriminates
rather than flagging everything -- a scanner that returned "violation" for both
would satisfy neither, and that is the CLAUDE.md 3.2 form 2 failure (a check red
on correct code) which ends with the check switched off.

Why so many cases for one lint. The scanner has two independent halves that can
each fail silently: the resolver that follows a handler argument to a function,
and the body scan that reads that function for a forbidden op. A green result is
only meaningful if BOTH ran, so several cases assert not just "no violation" but
"no note either", which is how they distinguish "scanned and clean" from "could
not resolve, gave up". test_the_real_registry_file_is_resolved_and_clean is the
sharpest of these: it runs against the shipped file, not a fixture.

What this file does NOT try to establish:
  * that every callback in the tree is safe. The scanner cannot follow a callback
    it cannot resolve, and it says so; test_an_unresolvable_handler_is_noted pins
    that a note is emitted rather than a silent pass.
  * the runtime anti-patterns A-1 and A-7. Those are 11 S2.4.8 checks over a live
    process and belong to INF-ZN-5 / INF-ZN-6; a source scan cannot see them, and
    asserting them here would be a case that passes because it cannot fail --
    CLAUDE.md 3.2 form 1.
"""

import os
import subprocess
import sys

import pytest

# Three levels up from tests/common/ is the repository root. Deriving it rather
# than hardcoding keeps the file movable and keeps the subprocess reading the
# same tree the scanner ships against.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# The scanner lives under scripts/lint/; put it on the path so the resolver and
# body scan can be driven in-process, which is faster and gives structured
# results a subprocess cannot (a list of violations, not parsed stdout).
sys.path.insert(0, os.path.join(ROOT, "scripts", "lint"))

import zenoh_callback_scan as L  # noqa: E402

# The script path, used for the subprocess cases that exercise exit codes and the
# printed report -- the parts CI actually consumes.
SCRIPT = os.path.join(ROOT, "scripts", "lint", "zenoh_callback_scan.py")

# The real module the scan must reach through the registry partial. If the
# resolver ever stops following that shape, the callback body would go unread and
# on_message could hide anything -- test_the_real_registry_file_is_resolved_and_
# clean is what pins that it is genuinely read, not skipped.
REGISTRY_FILE = os.path.join(ROOT, "xbrain", "common", "zenoh",
                             "subscriber_registry.py")


def run_script(*args):
    """(returncode, stdout+stderr) for the scanner as a subprocess.

    A subprocess rather than an in-process main(): it is what CI runs, it reads
    the real tree through the real ROOT, and it cannot be fooled by module state
    a test left behind. The two streams are joined because the report a human
    reads is the whole of stdout and stderr, and a case that asserted on only one
    could pass while the other carried the failure.
    """
    proc = subprocess.run([sys.executable, SCRIPT] + list(args),
                          capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


# ---------------------------------------------------------------------------
# The scanner's own probes, run as part of the suite so they cannot rot.
#
# A self-test nobody invokes rots exactly as fast as a lint nobody invokes, so
# the two cases below invoke both the probes and the real scan. They are the
# floor: everything after them is a specific property the floor does not pin.
# ---------------------------------------------------------------------------

def test_self_test_passes():
    """The probes the scanner carries must all hold.

    The scanner's --self-test runs every case in its SELF_TEST_CASES table, which
    includes one violating sample per forbidden op, the compliant twins, and the
    unresolved case. Running it here keeps those probes honest.

    Turned red by: any probe in SELF_TEST_CASES no longer matching the scanner's
    behaviour -- for instance a matcher that stopped catching put_nowait, whose
    positive probe would then report want 1 got 0.
    """
    rc, out = run_script("--self-test")
    assert rc == 0, out
    # The word, not just the exit code: a self_test() that returned 0 without
    # running anything would still pass an exit check, so the PASS line is
    # required to be present in the output.
    assert "self-test: PASS" in out, out


def test_on_disk_scan_is_clean_today():
    """*** The gate itself: the runtime trees carry no forbidden callback op.

    This is the assertion that makes the script a gate rather than a formality.
    It runs the scanner over xbrain/, ros2_ws/ and services/ exactly as CI does.

    Turned red by: any create_task, put_nowait, await or direct publish landing
    in a resolved subscriber callback under those trees -- which is precisely the
    mutation demonstrated against the shipped registry during development.
    """
    rc, out = run_script()
    assert rc == 0, (
        "zenoh_callback_scan reports a forbidden operation in a callback:\n%s"
        % out)


# ---------------------------------------------------------------------------
# CFG-CM-17 mutation two, and the compliant twin that gives it meaning.
#
# These two cases are the heart of the item. They are kept as permanent cases
# rather than a one-off development check, because a mutation that is only ever
# run once proves the scanner worked that day and nothing after.
# ---------------------------------------------------------------------------

def test_the_named_mutation_turns_the_scan_red():
    """create_task in on_message, via the registry partial, must be flagged.

    This is CFG-CM-17's mutation two verbatim: 在 callback 内调 asyncio.create_task
    => scripts/lint/zenoh_callback_scan.py 必须红. It is reached the hard way,
    through functools.partial(self.on_message, key) and a local variable, because
    that is the exact shape SubscriberRegistry.declare uses -- a scanner that only
    caught a callback named directly would miss the real code and pass this
    mutation.

    Turned red by (the scanner side): a resolver that gives up on the partial, or
    a body scan that misses create_task. Either would leave violations empty.
    """
    # _REGISTRY_MUTATED is the registry with on_message rewritten to create_task;
    # it lives in the scanner module so the probe and the metatest cannot drift.
    violations, _notes = L.scan_source(L._REGISTRY_MUTATED, "mutant.py")
    assert violations, (
        "create_task reached through the registry partial was not flagged; the "
        "named mutation would go green")
    # And it was flagged for the right reason, not incidentally: the op name must
    # be create_task, so a scanner that flagged the line for some other reason
    # would not satisfy this.
    assert any(name == "asyncio.create_task" for _s, _l, name, _w in violations)


def test_the_compliant_registry_is_clean():
    """The same registry with publish_threadsafe must NOT be flagged.

    Without this, test_the_named_mutation_turns_the_scan_red is satisfied by a
    scanner that flags every registry -- CLAUDE.md 3.2 form 2, a check red on
    correct code, which is loosened until it catches nothing. publish_threadsafe
    contains the substring publish, so this also pins that the match is on the
    exact attribute name and not a substring.

    Turned red by: a scanner matching publish as a substring (it would catch
    publish_threadsafe), or a scanner flagging any callback body at all.
    """
    violations, notes = L.scan_source(L._REGISTRY_CLEAN, "clean.py")
    assert violations == [], (
        "the compliant registry was flagged: %r" % (violations,))
    # And it was genuinely resolved -- no note means on_message was reached and
    # read, so the clean result is "scanned and clean", not "gave up". A scanner
    # that returned zero violations AND a note would be hiding a blind spot behind
    # a green number.
    assert notes == [], (
        "the compliant registry callback was not resolved: %r" % (notes,))


def test_the_real_registry_file_is_resolved_and_clean():
    """The shipped subscriber_registry.py is read, not skipped, and is clean.

    A green result only means something if the callback was actually reached.
    This asserts both: zero violations AND zero notes for the real file, so the
    resolver demonstrably followed functools.partial(self.on_message, key) to
    on_message instead of emitting a note and moving on. It is the one case that
    binds the scanner to the actual code it exists to guard rather than to a
    fixture written to be convenient.

    Turned red by: on_message gaining a forbidden op (violations non-empty), or
    the resolver losing the ability to follow the partial (notes non-empty).
    """
    violations, notes = L.scan_file(REGISTRY_FILE, "subscriber_registry.py")
    assert violations == [], (
        "the shipped registry callback contains a forbidden op: %r"
        % (violations,))
    assert notes == [], (
        "the shipped registry callback was NOT resolved, so its clean result "
        "proves nothing: %r" % (notes,))


# ---------------------------------------------------------------------------
# Each forbidden operation, and the compliant counter-examples.
#
# One probe per operation guards against a mistyped matcher sitting in the
# scanner looking authoritative while catching nothing -- the do-nothing shape
# CLAUDE.md 3.2 form 1 warns of. The compliant probes guard the opposite failure.
# ---------------------------------------------------------------------------

# The four CLAUDE.md 4.2 operations, each inside a callback resolved by name. The
# op string is what the scanner must name in the violation, so the assertion can
# check it caught the right thing rather than merely something.
_FORBIDDEN_SAMPLES = [
    ("def cb(s):\n    asyncio.create_task(go(s))\n"
     "session.declare_subscriber(k, cb)\n", "asyncio.create_task"),
    ("def cb(s):\n    q.put_nowait(s)\n"
     "session.declare_subscriber(k, cb)\n", "asyncio.Queue.put_nowait"),
    ("def cb(s):\n    bus.publish(k, s)\n"
     "session.declare_subscriber(k, cb)\n", "direct event_bus.publish"),
    ("async def cb(s):\n    await sink.write(s)\n"
     "session.declare_subscriber(k, cb)\n", "await"),
]


@pytest.mark.parametrize("source,op", _FORBIDDEN_SAMPLES)
def test_each_forbidden_operation_is_caught(source, op):
    """Every one of the four CLAUDE.md 4.2 ops, in a resolved callback, is red.

    Turned red by: the matcher for that specific op being removed or mistyped;
    the op would drop out of the reported names and this parametrised case would
    fail for exactly that op while the others stayed green, which localises the
    regression.
    """
    violations, _notes = L.scan_source(source, "probe.py")
    names = {name for _s, _l, name, _w in violations}
    assert op in names, "%s was not caught; got %r" % (op, names)


# The counter-examples: correct code that must stay green. Each pairs with a real
# way the scanner could over-reach, and the third is the publish/publish_threadsafe
# distinction stated as its own case so it cannot be lost in a refactor.
_COMPLIANT_SAMPLES = [
    ("async def worker():\n    asyncio.create_task(go())\n",
     "create_task outside any subscriber callback is legal"),
    ("def on_targets(s):\n    loop.create_task(process(s))\n"
     "registry.declare(session, k, on_targets)\n",
     "a handler via SubscriberRegistry.declare runs on the loop; declare is not "
     "declare_subscriber"),
    ("def cb(s):\n    bus.publish_threadsafe(k, s)\n"
     "session.declare_subscriber(k, cb)\n",
     "publish_threadsafe is the required call and must never be flagged"),
]


@pytest.mark.parametrize("source,why", _COMPLIANT_SAMPLES)
def test_compliant_shapes_are_not_flagged(source, why):
    """Correct code stays green.

    A checker that flagged these would be red on the code CLAUDE.md 4.2 and the
    event_bus header require, and a check red on correct code gets switched off.

    Turned red by: widening create_task detection to the whole tree (first
    sample), following declare as if it were declare_subscriber (second), or
    matching publish as a substring (third).
    """
    violations, _notes = L.scan_source(source, "probe.py")
    assert violations == [], "%s: %r" % (why, violations)


def test_an_unresolvable_handler_is_noted_not_passed():
    """A callback the scan cannot follow is a NOTE, never a silent clean pass.

    "found nothing to scan" and "scanned and found nothing" must not print as the
    same zero (CLAUDE.md 3.2 form 1). An imported handler cannot be followed in a
    source scan, so it must surface as a note that a reader can see and act on.

    Turned red by: the resolver treating an unresolved handler as clean (no note),
    which would let a callback imported from another module pass unexamined.
    """
    source = "session.declare_subscriber(k, other_module.cb)\n"
    violations, notes = L.scan_source(source, "probe.py")
    # No false positive: an unresolved handler is not itself a violation.
    assert violations == [], violations
    # But it must be visible: the note is the honest record that a body went
    # unread. Its absence is the failure this case exists to catch.
    assert notes, "an unresolvable handler produced no note; it would read as clean"


# ---------------------------------------------------------------------------
# The scan surface cannot be silently narrowed.
#
# CLAUDE.md 3.2 form 6 (undeclared scan surface) and form 3 (self-harm) both bite
# here: a scanner that drops a tree keeps printing "clean" for code it no longer
# reads, and a scanner that includes its own tree can never reach zero.
# ---------------------------------------------------------------------------

def test_scan_surface_is_exactly_the_three_runtime_python_trees():
    """Mutation: drop a tree from SCAN_DIRS => red.

    The scanner would go on printing "clean" for a tree it no longer reads, which
    is the CLAUDE.md 3.2 form 6 failure made permanent. Pinning the tuple here
    means narrowing the surface has to defeat this case first.

    Turned red by: editing SCAN_DIRS to drop xbrain, ros2_ws or services.
    """
    assert L.SCAN_DIRS == ("xbrain", "ros2_ws", "services")


def test_the_lint_is_outside_its_own_scan_surface():
    """The scanner must not scan itself or the tests.

    Both would be self-harm in the CLAUDE.md 3.2 form 3 sense: this file names the
    forbidden calls to look for them, and the metatests write violating callbacks
    on purpose, so including either tree would make the surface unable to reach
    zero and the repair anyone reaches for is to loosen the criterion.

    Turned red by: adding "scripts" or "tests" to SCAN_DIRS.
    """
    assert "scripts" not in L.SCAN_DIRS
    assert "tests" not in L.SCAN_DIRS


def test_ros2_ws_emptiness_is_reported_not_hidden():
    """An absent or Python-empty tree is announced, not passed over in silence.

    Zero hits from a tree that was never read is indistinguishable from zero hits
    from a clean one. The surface print must name every tree either as a file
    count or as a NOTE, so a reader can tell the difference.

    Turned red by: a surface printer that omits a declared tree from its report.
    """
    _rc, out = run_script()
    assert "scan surface" in out
    # Every declared tree must appear by name in the report, so none can be read
    # (or skipped) without the output saying so.
    for tree in L.SCAN_DIRS:
        assert tree in out, "%s not named in the surface report" % tree


def test_the_report_states_a_criterion():
    """A lint that prints numbers without a criterion lets the reader invent one.

    CLAUDE.md 3.2 form 2: the invented criterion is always the one the current
    output satisfies. So the report must state what passing means, in the output,
    next to the number.

    Turned red by: removing the criterion block from the report.
    """
    _rc, out = run_script()
    assert "criterion" in out
