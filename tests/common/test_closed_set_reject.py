"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_closed_set_reject.py
Brief: CFG-CM-5 rejection semantics -- both languages, with the mutations performed

Description:
Why this file exists as its own file. The CFG-CM-5 row of docs/XBRAIN_V6_TODO.md
names this exact path -- tests/common/test_closed_set_reject.py -- as the test its
mutation must turn red, so the criterion cannot be satisfied by the general
rejection cases already in test_closed_sets.py. It pins one contract, stated by
11 S13.6: a value outside a closed set is REJECTED, with no silent pass-through
and no degrade-to-something-close. The section even names the tempting repair it
forbids -- reading an off-set value as a nearby legal one -- because that turns a
contract violation into ordinary-looking operation the operator never questions.

The criterion has two halves and this file holds both, on each language side:

  Python. parse_enum("event_category", "foo") must raise ClosedSetViolation, and
  the English message must carry the closed-set name and the illegal value so a
  reader of the log can tell WHICH set rejected WHAT. The named mutation --
  "downgrade an unknown category to system" -- is performed against the real
  ClosedSet.parse in an isolated subprocess and observed to accept "foo" as
  "system", which is exactly what would redden the negative assertion. A control
  subprocess with the real code untouched rejects it, so the difference is the
  mutation and not the harness.

  C++. The deployed header cannot throw -- its nearest consumer chassis_relay
  runs under CRL-4, where a throw would allocate -- so it answers with a bool
  from Contains() and the sentinel kNotAMember from IndexOf(), and a caller MUST
  branch on that. Two mutations are compiled against a COPY of the header and
  run: Contains()'s miss branch rewritten to return true (silent pass-through),
  and IndexOf()'s miss branch rewritten to return 0 (degrade to the strongest
  member -- gate_limiter holds estop at 0, so a miss read as 0 becomes the
  highest-priority limiter). The probe reports each break; the tracked header is
  never touched, so an interrupted run cannot leave the tree edited.

The positive assertions, and why the negative ones are not enough. An
implementation that raised on EVERYTHING -- or a Contains() that always returned
false -- would satisfy every rejection case above while rejecting legal values
too. That is the empty-shell pass CLAUDE.md 3.2 form 1 warns about and this
project has caught live. So each side asserts a legal value survives unchanged:
parse_enum("event_category", "system") == "system", and Contains/IndexOf still
find their real members. On the C++ side those positives double as the control
for the mutation runs -- the miss-branch edits leave them at 1, so a red
rejection fact is the mutation rather than a broken build.

What this file does NOT establish, stated so a green run is not read as more than
it is:
  * the VALUES in the sets are right. That is the symmetric-difference metatest
    in test_closed_sets.py, against the design volumes; repeating members here
    would create a second place to update.
  * the Python and C++ sets agree. That is test_closed_sets.py's drift and
    same-sets cases. This file exercises the reject/accept BEHAVIOUR, which the
    header text alone cannot show.
  * anything about whole-key legality. Pairing a plane name with a domain name
    is the transport layer's to judge; parse_enum has no opinion on the pair.

Scan surface note. tests/ is outside no_literal_ecode.py's surface, so the E_*
that appears inside a ClosedSetViolation message is not asserted on here by its
literal spelling; the checks read the set name and the bad value, which is what
the criterion asks for, and never hardcode the code string.
"""

import os
import subprocess
import sys

import pytest

# ROOT is three levels up: tests/common/this_file -> repo root. Inserted on
# sys.path so "from xbrain.common import enums" resolves the same way it does in
# test_closed_sets.py, since there is no conftest.py adding the root for us.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import enums  # noqa: E402
from xbrain.common.errors.exceptions import ClosedSetViolation  # noqa: E402


# ---------------------------------------------------------------------------
# Python side: parse_enum rejection semantics (CFG-CM-5, the exact named case)
# ---------------------------------------------------------------------------

def test_parse_enum_rejects_out_of_set_event_category():
    """The criterion verbatim: parse_enum('event_category','foo') must raise.

    The message assertions are the load-bearing part. 11 S13.6 wants an operator
    reading a log to learn which set rejected which value, so the message must
    carry both -- a bare "closed set violation" would name neither and send the
    reader back to the source to find out what happened.
    """
    with pytest.raises(ClosedSetViolation) as caught:
        # foo is in no set. The by-name entry point is exercised on purpose:
        # generic decoders reach parse_enum with the set name read from a schema,
        # so it is the path a real off-contract message travels.
        enums.parse_enum("event_category", "foo")
    exc = caught.value  # the raised instance, so its fields can be inspected
    text = str(exc)  # the log form: code prefix plus the English message
    # Both the set name and the illegal value must appear in the English message,
    # verbatim. repr in the message wraps them in quotes, so a substring check is
    # exact enough and does not depend on the surrounding wording.
    assert "event_category" in text, text
    assert "foo" in text, text
    # The structured fields exist beside the message so a handler branches on the
    # set that failed rather than parsing the string it printed.
    assert exc.set_name == "event_category"
    assert exc.bad == "foo"


def test_legal_event_category_passes_through_unchanged():
    """The positive half. Without it, an implementation that raised on every
    value would satisfy the negative case above and still be wrong.

    parse returns its input unchanged so it can weld the check to the decode --
    plane = PLANE.parse(msg["plane"]) -- so "unchanged" is the property, not just
    "did not raise".
    """
    assert enums.parse_enum("event_category", "system") == "system"  # returned as-is
    assert enums.parse_enum("event_category", "intrusion") == "intrusion"  # not merely non-raising


# The mutation named in the CFG-CM-5 row, expressed once as a source string so it
# can be applied to the REAL ClosedSet.parse in a child interpreter. A subprocess
# rather than a monkeypatch in this process: ClosedSet is a process-wide singleton
# and parse sits on every boundary decode, so patching it here would leak into any
# test that ran afterwards. The child imports the real module, so it is the real
# parse being mutated, not a transcription of it -- the same reason test_closed_sets.py
# drives the generator in a subprocess against a copy rather than re-implementing it.
_PY_MUT_DRIVER = (
    "import sys\n"
    "sys.path.insert(0, {root!r})\n"
    "from xbrain.common import enums\n"
    "from xbrain.common.enums import ClosedSetViolation, ClosedSet\n"
    "_orig = ClosedSet.parse\n"
    "def _downgrade(self, v):\n"
    # The forbidden repair 11 S13.6 names by example: an off-set event_category
    # read as the nearby legal value system instead of raising.
    "    if self.name == 'event_category' and v not in self.values:\n"
    "        return 'system'\n"
    "    return _orig(self, v)\n"
    # apply is either the mutation or a no-op, so control and mutant run the exact
    # same driver and differ only in whether parse was replaced.
    "{apply}\n"
    "try:\n"
    "    print('ACCEPTED:' + repr(enums.parse_enum('event_category', 'foo')))\n"
    "except ClosedSetViolation:\n"
    "    print('RAISED')\n"
)


def _run_py_driver(apply_mutation):
    """Run the driver once, mutated or not, and return its single output line."""
    # A no-op comment line as the apply statement for the control keeps the driver
    # a valid program without a branch, so the two runs are byte-identical except
    # for the one assignment that is the mutation.
    apply_line = ("ClosedSet.parse = _downgrade" if apply_mutation
                  else "pass  # control: real parse untouched")
    driver = _PY_MUT_DRIVER.format(root=ROOT, apply=apply_line)  # bake in the one differing line
    proc = subprocess.run([sys.executable, "-c", driver],
                          capture_output=True, text=True, timeout=60)
    # A crashed child would otherwise surface as an opaque empty string; assert the
    # exit status first, with both streams attached, so the failure points at the
    # driver rather than at the assertion that read its output.
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return proc.stdout.strip()  # the single RAISED or ACCEPTED:... line the driver prints


def test_the_downgrade_to_system_mutation_is_caught():
    """*** The CFG-CM-5 mutation, performed rather than described (CLAUDE.md 3.3).

    An assertion never shown to go red has not been written. The control proves
    the real code rejects; the mutant proves the exact edit the row names --
    downgrade an unknown category to system -- makes parse_enum ACCEPT "foo",
    which is precisely what would turn test_parse_enum_rejects_out_of_set_event_category
    red. The two together show the negative assertion is discriminating and not an
    empty shell that any implementation passes.
    """
    # Control: the real ClosedSet.parse rejects the off-set value.
    assert _run_py_driver(apply_mutation=False) == "RAISED"
    # Mutant: the downgrade makes it return the nearby legal value instead. If this
    # ever read RAISED, the mutation would have stopped reproducing and the note
    # above would have decayed into a claim nobody can re-run.
    assert _run_py_driver(apply_mutation=True) == "ACCEPTED:'system'"


# ---------------------------------------------------------------------------
# C++ side: the deployed header's Contains/IndexOf rejection semantics
#
# The header does not throw (CRL-4 forbids the allocation a throw needs), so
# rejection is a bool from Contains() and the sentinel kNotAMember from IndexOf().
# That is runtime behaviour, so the header text is not evidence it holds: the
# probe in tests/common/enums_cxx exercises it, and the two mutations below are
# compiled against a COPY of the header and run, so the break is observed rather
# than asserted about a file nobody executed.
# ---------------------------------------------------------------------------

#: The deployed header and the probe that exercises it. The probe includes the
#: header as "xbrain/enums/closed_sets.h", which is the path the artifact really
#: has, so pointing -I at a mutated tree swaps the header the probe sees.
REAL_HEADER = os.path.join(ROOT, "common", "include", "xbrain", "enums",
                           "closed_sets.h")
#: The real include root: the probe includes "xbrain/enums/closed_sets.h", so -I
#: points one level above xbrain/. The mutation cases swap this for a tmp root.
INCLUDE_ROOT = os.path.join(ROOT, "common", "include")
PROBE = os.path.join(ROOT, "tests", "common", "enums_cxx", "closed_set_reject.cc")

#: Exactly C++17 per CLAUDE.md 5.6 and CPP-1, with warnings fatal: these headers
#: are consumed by chassis_relay on the emergency-stop path, where a warning is a
#: defect and not a note. Building as C++20 would let a C++20-only construct into
#: a header whose consumers are pinned to 17, unnoticed until the ORIN build.
CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]

#: g++ or clang++, whichever the machine has. None means neither, and the tests
#: skip with a reason rather than passing silently.
import shutil  # noqa: E402  -- kept beside its only use, the compiler probe below

CXX = shutil.which("g++") or shutil.which("clang++")

#: Applied to every case that needs a compiler. The reason spells out what went
#: unverified, because a bare "skipped" on a suite whose whole subject is C++
#: runtime behaviour reads as a pass to anyone skimming the summary line.
requires_cxx = pytest.mark.skipif(
    CXX is None,
    reason="no C++ compiler on PATH; the Contains/IndexOf rejection semantics of "
           "common/include/xbrain/enums/closed_sets.h were NOT verified in this run",
)


def _build(include_root, out_dir):
    """Compile the probe with include_root on the search path; return the exe.

    include_root is a parameter, not a constant, because the whole method is to
    compile the SAME source against the real header and against a mutated copy of
    it. -Werror is in the flag list, so a warning fails the build and lands in the
    message below rather than producing a binary that runs on a lie.
    """
    exe = os.path.join(str(out_dir), "closed_set_reject")  # one disposable exe per tmp_path
    # Only the probe's own header is needed; closed_sets.h pulls in just <cstddef>
    # and <string_view>, so a single -I at the mutated root guarantees the mutated
    # header is the one compiled, with no fall-through to the real tree.
    cmd = [CXX] + CXX_FLAGS + ["-I", include_root, PROBE, "-o", exe]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        pytest.fail("compiling the enums reject probe failed:\n" + proc.stderr)
    return exe  # caller runs it; a failed build already raised above


def _run(exe):
    """Run the probe and parse its key=value lines into a dict of ints.

    Every value the probe prints is an integer, so the parse is total: a probe
    that printed something unparsable fails here rather than feeding a subtly
    wrong number into an assertion. The exit status is checked first, with both
    streams attached, so a probe that aborted does not surface as a KeyError.
    """
    proc = subprocess.run([exe], capture_output=True, text=True, timeout=60)  # a hung probe fails, not blocks
    assert proc.returncode == 0, (
        "%s exited %d\n%s\n%s"
        % (os.path.basename(exe), proc.returncode, proc.stdout, proc.stderr))
    facts = {}  # key -> int, one entry per printed fact
    for line in proc.stdout.strip().split("\n"):
        if line:
            key, _, value = line.partition("=")
            facts[key] = int(value)  # total parse: an unparsable line fails here, loudly
    return facts  # every fact the probe emitted, for the caller to assert on


def _mutated_include_root(tmp_path, replace_from, replace_to):
    """Write a copy of the header with one miss-branch edit; return its -I root.

    The edit is applied to a COPY under tmp_path, never to the tracked header: a
    mutation that edits a tracked file and restores it leaves the tree broken
    whenever the run is interrupted. The changed-text assertion turns a stale
    anchor -- the header reformatted so the target line no longer matches -- into
    a clear failure here rather than a mutation that silently did nothing and a
    test that then passes for the wrong reason.
    """
    real = open(REAL_HEADER, encoding="utf-8").read()  # the tracked header, read-only
    mutated = real.replace(replace_from, replace_to, 1)  # one occurrence only, the miss branch
    assert mutated != real, (
        "mutation anchor %r not found in closed_sets.h; the header changed shape "
        "and this mutation must be re-derived" % replace_from)
    root = tmp_path / "inc"  # the -I root; the probe includes xbrain/enums/... below it
    dest = root / "xbrain" / "enums" / "closed_sets.h"  # same relative path as the artifact
    dest.parent.mkdir(parents=True, exist_ok=True)  # create the xbrain/enums/ nesting
    dest.write_text(mutated, encoding="utf-8")  # the mutated copy the probe will compile against
    return str(root)  # hand back the -I root, not the file, so _build stays uniform


@requires_cxx
def test_cxx_contains_and_indexof_reject_out_of_set_values(tmp_path):
    """Against the real header: every rejection and positive fact holds.

    This is the behaviour the two mutation cases below break one at a time.
    Reading it as the baseline: an off-contract value is rejected by both
    Contains and IndexOf, a miss never collides with the strongest position, the
    sentinel is not itself a position, and the real members are still found.
    """
    facts = _run(_build(INCLUDE_ROOT, tmp_path))  # real header: the baseline the mutations break
    # Rejection: Contains says false, IndexOf says kNotAMember, for a value in no
    # set. These are the two forbidden repairs from 11 S13.6, refused.
    assert facts["contains_rejects_foo"] == 1
    assert facts["indexof_rejects_foo"] == 1
    # The sharp form: a miss must not read as position 0, which for the ordered
    # gate_limiter is estop, the strongest limiter of all.
    assert facts["miss_not_read_as_strongest"] == 1
    assert facts["not_a_member_sentinel_nonzero"] == 1
    # Positives, so an all-false Contains or an all-sentinel IndexOf cannot pass
    # the rejection facts by refusing everything.
    assert facts["contains_accepts_system"] == 1
    assert facts["indexof_finds_estop"] == 1


@requires_cxx
def test_cxx_silent_passthrough_mutation_is_caught(tmp_path):
    """*** Mutation: Contains()'s miss branch returns true (silent pass-through).

    This is the header analogue of the Python downgrade -- a Contains that answers
    true for a value it never held, so a caller writing "if Contains(v): use(v)"
    would use an off-contract value as though the contract allowed it. The probe
    must report the rejection broken. contains_accepts_system stays 1 because the
    hit branch is untouched, which is the control proving the red is the mutation
    and not a build that failed to reject anything at all.
    """
    root = _mutated_include_root(tmp_path,
                                 "\n  return false;\n", "\n  return true;\n")
    facts = _run(_build(root, tmp_path))  # same probe, header with the Contains miss flipped
    # Caught: Contains no longer rejects the off-set value.
    assert facts["contains_rejects_foo"] == 0
    # Control: the legal member is still found, so the case above is a genuine
    # rejection failure and not a header that rejects and accepts nothing.
    assert facts["contains_accepts_system"] == 1
    # Untouched: the IndexOf side is a different function, so it stays correct --
    # evidence the edit was surgical and this red is about Contains alone.
    assert facts["indexof_rejects_foo"] == 1


@requires_cxx
def test_cxx_degrade_to_strongest_mutation_is_caught(tmp_path):
    """*** Mutation: IndexOf()'s miss branch returns 0 (degrade to the strongest).

    The most dangerous of the three. gate_limiter is ordered with estop at
    position 0, so an IndexOf that answered 0 for a miss would report an
    off-contract value as the highest-priority limiter -- a fail-silent worse than
    a fail-safe, which is exactly why kNotAMember is deliberately not zero. The
    probe must report both the plain rejection and the strongest-position guard
    broken; indexof_finds_estop stays 1 as the control.
    """
    root = _mutated_include_root(tmp_path,
                                 "\n  return kNotAMember;\n", "\n  return 0;\n")
    facts = _run(_build(root, tmp_path))  # same probe, header with the IndexOf miss flipped
    # Caught: a miss now returns a real position instead of the sentinel.
    assert facts["indexof_rejects_foo"] == 0
    # Caught, and this is the one that matters: the miss now reads as position 0,
    # which is estop -- the degrade-to-something-close 11 S13.6 forbids, in its
    # worst form.
    assert facts["miss_not_read_as_strongest"] == 0
    # Control: estop is still found at 0, so the reds above are the mutation and
    # not an IndexOf that lost track of every value.
    assert facts["indexof_finds_estop"] == 1
