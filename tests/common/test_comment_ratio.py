"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_comment_ratio.py
Brief: The CLAUDE.md 2.4 ratchet -- the snapshot is the only way past the threshold

Description:
What this guards, and why it needs guarding more than an ordinary lint does.

scripts/lint/comment_ratio.py gained a debt snapshot on 2026-10-01 (user ruling:
ratchet rather than a bulk rewrite of the tree). A snapshot is a WIDENING, and a
widening is the single most dangerous edit to make to a gate -- CLAUDE.md 3.2
form 2 is precisely "a criterion that is always red gets loosened until it
passes", and the loosening usually arrives wearing the word "pragmatic". So the
cases here are not about the ratio at all. They are about whether the ratchet
can still bite:

  * a file below the threshold that is NOT in the snapshot must gate. This is
    the whole point, and it is asserted against a real fixture tree and a real
    subprocess run of main(), not by reading the source.
  * adding the row must be the ONLY way to make that run green. If anything else
    silences it, the snapshot is decoration.
  * deleting a row while its file is still below the threshold must gate again.
    Without this, "the snapshot may only shrink" would be a sentence in a
    docstring rather than a behaviour.
  * a row whose file has RISEN above the threshold must gate, so the list
    shrinks by itself instead of merely not growing.
  * there must be no flag that rewrites the snapshot. A gate that can silence
    itself is not a gate: the first response to a red run becomes running the
    flag, and nothing in the resulting diff says what was given up.

*** Why the one-time widening is defensible, stated here so a reader can argue
with it rather than having to reconstruct it.

Before the ratchet, the check was red on every run and had never been green. The
only thing asserted anywhere was "no file under xbrain/ is below the threshold"
(tests/common/test_lints.py), which was ALSO red; scripts/ and tests/ were
measured and deliberately not failed on, with that case's own docstring saying
it would be replaced "if the decision is taken to enforce it everywhere". So the
state being replaced is: one permanently red CI check, plus one permanently red
case, plus two trees nobody gated at all.

After it, every file outside the snapshot is gated in ALL THREE trees. A new
file under scripts/ or tests/ below the threshold now fails, and that was
previously invisible. The widening buys a strictly larger enforced surface, and
the thing it gives up -- an immediate demand to rewrite the comment blocks of
hundreds of existing files -- was never actually being demanded, because nobody
can act on a check that is red for everything at once.

That is the argument, and it has exactly one hole worth naming: it is only true
while the snapshot shrinks. A snapshot that grows turns this from a ratchet into
a permanent waiver, which is why three of the cases below are about shrinking
and why the ceiling exists at all. The ceiling is the 019f1ec precedent: it may
only ever be lowered, and lowering it is how the ratchet is seen to be working.

*** A measured note on the ratchet biting its own author, kept because it is the
most convincing evidence here. The first draft of THIS file came in below the
threshold and the live-run case went red on it immediately. The two available
responses were to add a row to the brand-new snapshot, or to write the comments.
Adding a row on the same commit that introduces the ratchet would have been the
clearest possible demonstration that the mechanism is decoration, so the
comments were written instead -- which is what the rule asks for and what the
reader is holding.

*** How to read a red run of this file, because the five failures ask for five
different things and confusing them wastes the time the gate was meant to save:

  the live-run case red
      A file in xbrain/, scripts/ or tests/ is below the threshold and has no
      row. The repair is comments, not a row. Run the lint directly to see which
      file and what its ratio is; it prints both.
  the ceiling case red
      Somebody added rows. The diff shows which, because the snapshot is one
      path per line and sorted. The question for review is why the comments were
      not written instead.
  a snapshot-shape case red
      The file was hand-edited into a shape that hides additions -- unsorted,
      duplicated, or two paths on a line. Re-sort it.
  a row-integrity case red
      A rename or a deletion left a row behind, or a row points outside the scan
      surface where it can never be cleared. Delete the row.
  a fixture case red
      The MECHANISM changed, not the tree. One of the four behaviours above has
      stopped happening, and the snapshot is no longer a ratchet. This is the
      serious one: the live run can be green while the mechanism is gone, which
      is exactly the state tests/meta/test_marker_coverage.py was found in.

*** What none of this establishes. Not that the carried files are acceptable.
CLAUDE.md 2.4 says the ratio is a means and the real requirement is that every
block explains WHY, which no scan can decide. The snapshot is the list of files
where that work has not been done, and a green run here says nothing about
whether it ever will be. Nor does it establish anything about the OTHER trees in
the repository: common/, ros2_ws/, services/ and configs/ hold no Python the
lint walks, so 2.4's requirement on them -- it is a rule about comments, not
about Python -- is not evaluated anywhere, by this file or by any other.
"""

import importlib.util
import os
import subprocess
import sys

import pytest

# INF-TS-1: a module-level hardware marker on every test file, so nothing lands
# silently in the default archive. This one reads files and runs subprocesses
# and touches no device, so no_device is the honest value rather than the
# convenient one.
pytestmark = pytest.mark.no_device

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LINT_DIR = os.path.join(ROOT, "scripts", "lint")
SCRIPT = os.path.join(LINT_DIR, "comment_ratio.py")


def _load():
    """The lint as a module, for its constants.

    Loaded by path rather than imported because scripts/ carries no __init__.py
    and is not a package. The alternative -- duplicating THRESHOLD, SOURCE_DIRS
    and DEBT_PATH into this file -- would make every case here agree with a copy
    instead of with the thing CI runs, which is the failure CLAUDE.md 3.7 names:
    two hand-kept values drift and the test keeps passing against the stale one.
    """
    spec = importlib.util.spec_from_file_location("comment_ratio_under_test",
                                                  SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


L = _load()

#: The debt ceiling. It may ONLY ever be lowered, and lowering it in the same
#: commit that shortens the snapshot is how the ratchet is seen to be working.
#: Raising it is the move that kills a ratchet (CLAUDE.md 3.2 form 2); if a
#: commit needs it raised, the question to ask is not "by how much" but why a
#: new file is joining the snapshot at all, since the gate already refuses an
#: unlisted file and the only way to reach this number is a hand edit.
DEBT_CEILING = 817


def _run(tree_root, debt_path):
    """(rc, output) for one run of main() with ROOT and DEBT_PATH repointed.

    A subprocess running main() rather than an in-process call of measure():
    main() is what CI invokes, so main() is what the mutations have to be shown
    to fail. Calling measure() directly would exercise a path CI never takes and
    would miss everything that lives in the reporting -- which is where the
    classification into gating and carried actually happens.

    Both knobs are patched because the ratchet's behaviour is a PAIR: what is on
    disk, and what the snapshot names. A case able to vary only one of them
    could not separate "this file is below the threshold" from "this row is
    missing", and those two need different repairs.
    """
    code = (
        "import sys, importlib\n"
        "sys.path.insert(0, %r)\n"
        "m = importlib.import_module('comment_ratio')\n"
        "m.ROOT = %r\n"
        "m.DEBT_PATH = %r\n"
        "sys.exit(m.main())\n" % (LINT_DIR, str(tree_root), str(debt_path))
    )
    proc = subprocess.run([sys.executable, "-c", code],
                          capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


#: A file with no comments at all: three code lines, zero comment lines, 0%.
#: Deliberately far below the threshold rather than just under it, so a case
#: that goes green can never be explained by an off-by-one in the comparison.
LOW_SRC = "x = 1\ny = 2\nz = 3\n"

#: Three comment lines over one code line: 75%, above the threshold. Also
#: deliberately clear of the boundary, for the same reason in the other
#: direction.
HIGH_SRC = "# why one\n# why two\n# why three\nx = 1\n"


def _tree(tmp_path, files):
    """A fixture tree under tmp_path. files maps relative path -> source.

    The paths given by the callers mirror real trees (xbrain/, scripts/) because
    the lint walks SOURCE_DIRS by name; a fixture file outside those names would
    never be reached and the case would pass for the wrong reason.
    """
    for rel, src in files.items():
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(src, encoding="utf-8")
    return tmp_path


def _snapshot(tmp_path, rows, name="debt.txt"):
    """Write a fixture snapshot and return its path.

    A distinct `name` per call matters: several cases below run the lint twice
    over the SAME tree with different snapshots, and reusing one filename would
    leave the two runs sharing a file whose content changed between them -- a
    race that would show up as a flaky case rather than as a wrong answer.
    """
    p = tmp_path / name
    p.write_text("".join(r + "\n" for r in rows), encoding="utf-8")
    return p


# -- the snapshot as committed ------------------------------------------------

def test_the_snapshot_is_a_file_on_disk_with_one_path_per_line():
    """The shape that keeps an added row to a one-line diff.

    An inline tuple would be re-wrapped by every formatter that touched the
    lint, and the diff noise would hide an addition -- the reason 019f1ec gives
    for moving the marker allowlist out of its test file and onto disk. Sorted,
    deduplicated, one path per line is what makes "somebody added a row" visible
    at a glance in review, and review is the only thing standing between this
    mechanism and a snapshot that quietly absorbs every new file.

    Mutation: wrap two paths onto one line, or unsort the file => red here.
    """
    assert os.path.isfile(L.DEBT_PATH), L.DEBT_PATH
    raw = open(L.DEBT_PATH, encoding="utf-8").read().splitlines()
    rows = [ln for ln in raw if ln.strip()]
    # An empty snapshot would satisfy every shape assertion below while proving
    # nothing about the shape of a real one.
    assert rows, "an empty snapshot would make this case prove nothing"
    for ln in rows:
        # Surrounding whitespace survives load_debt()'s strip() but would make a
        # hand edit compare unequal to what the walker produces.
        assert ln == ln.strip(), "row carries surrounding whitespace: %r" % ln
        assert " " not in ln, "more than one path on a line: %r" % ln
        assert ln.endswith(".py"), "row is not a python file: %r" % ln
        # Repo-relative, because that is what main() compares against; an
        # absolute path would match nothing and read as a stale row.
        assert not ln.startswith("/"), "row must be repo-relative: %r" % ln
    assert len(set(rows)) == len(rows), "the snapshot has duplicate rows"
    assert rows == sorted(rows), (
        "the snapshot must stay sorted; an unsorted list makes an insertion "
        "land anywhere and stops the diff from being one line"
    )


def test_every_snapshot_row_names_a_file_inside_the_scan_surface():
    """Rows must be live, and must be clearable.

    A row naming nothing carries debt for a file that no longer exists; a row
    outside SOURCE_DIRS can never be cleared, because the walk will never reach
    the file to notice it has risen. The lint reports both at run time -- this
    case says it about the COMMITTED snapshot specifically, so a bad row is
    caught by the suite rather than only by whoever next reads the lint output.
    """
    for rel in sorted(L.load_debt()):
        assert os.path.isfile(os.path.join(ROOT, rel)), (
            "snapshot row names a missing file: %s" % rel)
        assert rel.split("/", 1)[0] in L.SOURCE_DIRS, (
            "snapshot row is outside the scan surface: %s" % rel)


def test_the_debt_ceiling_may_only_be_lowered():
    """The backstop for a hand-added row.

    The gate below is what stops a new low file from landing unlisted. This is
    what stops the gate itself from being satisfied by growing the snapshot
    instead of the comments -- the two are different failures and need different
    guards, because the first is enforced by a program and the second can only
    be enforced by making the edit visible and giving it a number to exceed.

    Raising this is how a ratchet dies (CLAUDE.md 3.2 form 2). If a commit wants
    it raised, the right question is why a file is joining the snapshot at all.
    """
    rows = L.load_debt()
    assert len(rows) <= DEBT_CEILING, (
        "the debt snapshot has %d rows, above the %d ceiling -- the snapshot "
        "may only shrink; lower the ceiling together with it"
        % (len(rows), DEBT_CEILING)
    )


def test_the_repository_currently_passes():
    """The live run, which is what the CI check gates on.

    This passes today because every file below the threshold is named by the
    snapshot. That is a fact about today and not a property of the code: the
    first file that lands below the threshold without a row fails here, which is
    the entire purpose of the mechanism.
    """
    proc = subprocess.run([sys.executable, SCRIPT], capture_output=True,
                          text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


# -- the ratchet, run against fixtures ----------------------------------------
#
# *** Why the cases below use fixture trees at all, and what that costs.
#
# The behaviours under test are transitions: a file crossing the threshold, a
# row appearing, a row disappearing. None of them can be exercised against the
# real tree by a test that also has to leave the tree as it found it -- and a
# test that edited the repository to prove a point would be worse than no test,
# because a crash mid-run would leave the snapshot altered and the next person
# would be debugging a gate that had been quietly rewritten.
#
# The cost is that a fixture run proves things about main() and nothing about
# the repository. That gap is closed from the other side by
# test_the_repository_currently_passes, which runs the real script over the real
# tree with the real snapshot, and by the two row-integrity cases above, which
# read the committed file. Neither half is sufficient:
#
#   * fixtures alone would pass while the committed snapshot named files that do
#     not exist, or while the live run was red;
#   * the live run alone would pass while the ratchet had been gutted, because a
#     tree where nothing is wrong is green under a gate that cannot fire. That
#     is not a hypothetical shape in this repository -- it is what
#     tests/meta/test_marker_coverage.py was found doing on 2026-09-30, where
#     the allowlist was recomputed from the same expression the assertion
#     evaluated and the difference was empty by construction.
#
# So the pair is the claim, and neither case should be deleted on the grounds
# that the other one covers it.

def test_a_file_below_the_threshold_gates_unless_the_snapshot_names_it(tmp_path):
    """*** The positive control, both directions in one case.

    The SAME fixture file is scanned twice and only the snapshot differs, which
    is what makes the two halves mean something together:

      * without the second half, "an unlisted low file gates" could be true
        because the walk reached nothing at all, or because every run of this
        lint is red -- both were true of this check at various points in its
        life;
      * without the first half, a snapshot implementation that swallowed
        everything handed to it would pass just as happily.

    Mutation: make main() return 0 whenever the snapshot loaded => the first
    half goes red. Make it ignore the snapshot => the second half does.
    """
    tree = _tree(tmp_path / "t", {"xbrain/p9_demo/new_thing.py": LOW_SRC})

    rc, out = _run(tree, _snapshot(tmp_path, []))
    assert rc != 0, "an unlisted file below the threshold must gate:\n" + out
    # Both the name and the heading: the exit code alone would pass if the file
    # were reported under some other finding, and the heading alone would pass
    # if a different file were named.
    assert "xbrain/p9_demo/new_thing.py" in out
    assert "BELOW THRESHOLD and not in the snapshot" in out

    rc, out = _run(tree, _snapshot(tmp_path,
                                   ["xbrain/p9_demo/new_thing.py"],
                                   name="debt2.txt"))
    assert rc == 0, "naming the file in the snapshot must be the way out:\n" + out
    # The counter, so a run that went green by not scanning the file at all is
    # distinguishable from one that went green by carrying it.
    assert "carried debt:      1" in out


def test_deleting_a_row_while_its_file_is_still_low_gates(tmp_path):
    """"The snapshot may only shrink" has to mean shrink BECAUSE THE FILE
    IMPROVED, not shrink by deletion.

    Two low files and two rows: green. Remove one row and change nothing else:
    red, naming exactly that file and not the other. The "and not the other"
    half is what separates a working classification from a lint that reports
    every low file the moment anything is wrong -- which would be red on correct
    trees and would end up switched off.
    """
    tree = _tree(tmp_path / "t", {"xbrain/a/one.py": LOW_SRC,
                                  "scripts/b/two.py": LOW_SRC})
    both = ["scripts/b/two.py", "xbrain/a/one.py"]

    rc, out = _run(tree, _snapshot(tmp_path, both))
    assert rc == 0, out

    rc, out = _run(tree, _snapshot(tmp_path, ["scripts/b/two.py"],
                                   name="debt2.txt"))
    assert rc != 0, "deleting a row whose file is still low must gate:\n" + out
    assert "xbrain/a/one.py" in out
    # The still-listed file must NOT appear under the gating heading. Slicing
    # the output after that heading is how this stays specific: the file does
    # appear earlier in the per-file listing, where it belongs.
    assert "scripts/b/two.py" not in out.split(
        "BELOW THRESHOLD and not in the snapshot")[1]


def test_a_row_whose_file_has_risen_must_leave_the_snapshot(tmp_path):
    """This is what makes the list shrink on its own.

    Without it the snapshot would only stop growing, and a file could sit in it
    forever after being fixed -- which is how a debt list stops measuring
    anything: the number stays large, nobody believes it, and the next person to
    look proposes deleting the mechanism rather than the rows.

    Mutation: drop the RISEN branch from main() => the first half goes red,
    because the run it expects to fail now passes.
    """
    tree = _tree(tmp_path / "t", {"xbrain/a/improved.py": HIGH_SRC})

    rc, out = _run(tree, _snapshot(tmp_path, ["xbrain/a/improved.py"]))
    assert rc != 0, "a cleared file must not stay in the snapshot:\n" + out
    assert "RISEN to the threshold" in out
    assert "xbrain/a/improved.py" in out

    # And the repair the message asks for must actually work. A gate whose
    # instruction does not clear it teaches people to ignore the instruction.
    rc, out = _run(tree, _snapshot(tmp_path, [], name="debt2.txt"))
    assert rc == 0, "with the row gone the same tree must be clean:\n" + out


def test_a_row_naming_nothing_gates(tmp_path):
    """A rename must not carry its debt along unseen.

    The failure this prevents is a pair, not a single event: the file is renamed
    and the row is not, so the stale row goes on excusing a path that no longer
    exists while the renamed file lands as a fresh unlisted file. The second
    half of that pair already gates, so without this case the two would be
    reported as one confusing finding instead of two with obvious repairs.
    """
    tree = _tree(tmp_path / "t", {"xbrain/a/kept.py": HIGH_SRC})
    rc, out = _run(tree, _snapshot(tmp_path, ["xbrain/a/gone.py"]))
    assert rc != 0, out
    assert "naming no scanned file" in out
    assert "xbrain/a/gone.py" in out


def test_a_missing_snapshot_holds_every_file_to_the_threshold(tmp_path):
    """The strict direction on failure.

    If the snapshot cannot be read, the two available behaviours are "gate
    everything" and "excuse everything". The second makes a path typo silence
    the check while the report still looks entirely normal, which is the
    fail-silent shape CLAUDE.md 3.1 describes in another context and the same
    thing applies here.
    """
    tree = _tree(tmp_path / "t", {"xbrain/a/one.py": LOW_SRC})
    rc, out = _run(tree, tmp_path / "does_not_exist.txt")
    assert rc != 0, out
    assert "0 row(s)" in out, (
        "the report must say the snapshot was empty; a snapshot that failed to "
        "load must not look like a snapshot with nothing in it to carry"
    )


# -- the mechanism itself -----------------------------------------------------

def test_there_is_no_flag_that_rewrites_the_snapshot():
    """*** A gate that can silence itself is not a gate.

    With a regenerate option, the first response to a red run is to run it, the
    snapshot grows, and nothing in the diff says what was given up. Rows go in
    by hand or not at all.

    *** The scan starts after the module docstring, and that is not cosmetic.
    The lint's docstring has to NAME the flag it refuses to have in order to say
    why it refuses -- so scanning the whole file makes this case report the
    explanation as the defect, it can never reach zero, and the repair anyone
    reaches for is to delete the explanation. CLAUDE.md 3.2 form 3, 判据自伤,
    measured here on the first run of this very case. The needles are also
    assembled from pieces for the same reason, so this file would survive if
    tests/ were ever added to some future scan surface.
    """
    body = open(SCRIPT, encoding="utf-8").read()
    # Everything from the first import onward: the module docstring ends there,
    # and nothing above it can execute.
    code = body[body.index("\nimport io"):]
    for suspicious in ("--" + "write-debt", "--" + "update-debt",
                       "--" + "regenerate", "--" + "accept"):
        assert suspicious not in code, (
            "found what looks like a snapshot-rewriting flag: %r" % suspicious)
    # The file must never be opened for writing by the lint itself. Checked on
    # both quote spellings, because a formatter may rewrite one into the other.
    assert 'open(DEBT_PATH, "w"' not in code
    assert "open(DEBT_PATH, 'w'" not in code


def test_the_report_says_what_carrying_debt_does_not_mean():
    """CLAUDE.md 3.2 form 7, printed next to the number.

    "gating findings: 0" reads as "the tree meets 2.4" unless the output says
    otherwise, and the tree does not meet it -- most of the files in the surface
    are carried. The boundary is printed on every run rather than living in a
    document nobody opens alongside the output, which is the same reason
    clock_scan.py prints its four blind spots.

    Mutation: delete the closing paragraph of main() => red.
    """
    proc = subprocess.run([sys.executable, SCRIPT], capture_output=True,
                          text=True)
    out = proc.stdout + proc.stderr
    assert "declared debt, not exemption" in out
    assert "the snapshot may only shrink" in out
    assert "every block explains WHY" in out


def test_the_snapshot_path_is_reported_so_a_silent_swap_is_visible():
    """Which snapshot was read, and how many rows it had, on every run.

    Two trees with different snapshots produce identical-looking output without
    this line, and "the gate passed" would not say which list did the passing.
    The row count is printed for the same reason and is derived at run time, not
    written into any document (CLAUDE.md 3.7).
    """
    proc = subprocess.run([sys.executable, SCRIPT], capture_output=True,
                          text=True)
    out = proc.stdout + proc.stderr
    assert "debt snapshot:" in out
    assert os.path.relpath(L.DEBT_PATH, ROOT) in out


# -- what is deliberately NOT covered here ------------------------------------
#
# Stated rather than left to be discovered, because an unstated gap in a test
# file reads as a covered one:
#
#   * measure() itself. Its probes live in the lint's own --self-test and are
#     invoked by tests/common/test_lints.py; duplicating them here would give
#     two places to update and one of them would rot.
#   * comment QUALITY, which is the actual requirement in CLAUDE.md 2.4 and is
#     not decidable by any program. A file can sit at 100 percent with every
#     comment restating the line beneath it.
#   * whether the carried files SHOULD be carried. Nothing here reads a single
#     one of them; the snapshot was taken from the tree as it stood, so it
#     records what the measurement said and not what anybody judged.
#   * the CI wiring. That scripts/ci/run_all.sh invokes this lint at all is held
#     by tests/common/test_ci_registry.py, which compares the runner with
#     scripts/ci/checks.yaml in both directions.
