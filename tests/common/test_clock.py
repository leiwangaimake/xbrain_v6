"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_clock.py
Brief: CFG-CM-12 -- the monotonic clock export and the full-text wall-clock scan

Description:
What is asserted here, and why each half needs its own kind of test.

The module half (xbrain/common/clock/) is small enough that the only interesting
question is whether it really reads the monotonic clock. A wrapper returning the
wall clock would satisfy every shape-level assertion -- it returns a float, it
increases, two calls differ -- so the load-bearing case compares the reading
against time.monotonic() with a tight tolerance, which a wall-clock
implementation misses by the entire Unix epoch. Reading time.monotonic()
directly in this file is deliberate and not a violation of anything: tests/ is
outside the scan surface, and time.monotonic() is the call CLK-C1 requires
anyway.

The lint half needs metatests rather than more probes. clock_scan.py already
carries one probe per rule in its --self-test, and that is run from here so the
probes cannot rot. What a metatest has to cover is everything ABOUT the lint
that could quietly stop being true:

  * the scan surface. A lint reading one tree instead of four still prints
    "violations: 0", and zero from a shrunken surface is indistinguishable from
    zero from a clean tree.
  * the six spellings the CFG-CM-12 criterion names verbatim. Dropping one is
    invisible in the output.
  * the compliant spellings. A checker that flagged everything would pass every
    violation case below and be worthless, and the usual response to a check
    that fails everything is to delete it.
  * the exemption mechanism, which only works while the tag set stays closed.

The three mutations the criterion names are run here as cases:
  (1) a wall-clock age computation inside p1_motion must turn the scan red;
  (2) a default-constructed rclcpp::Clock() inside quadruped must turn it red;
  (3) rewriting the script to consult a changed-files diff must be reported as
      a shrunken scan surface -- and mutation (3) is exercised against a real
      mutated copy of the script, not merely described.

*** On (1) and (2) being run against fixture trees. Neither xbrain/p1_motion/
nor ros2_ws/quadruped/ exists today -- ros2_ws/ holds no file at all -- so the
mutation cannot be injected into the real tree by a test that also has to leave
the tree as it found it. The fixture stands in for the real thing only because
two further cases pin the connection: the scan surface really does include those
two trees, and the run against the real repository really does report ros2_ws/
as empty rather than counting it as checked. Without that pair, "the directory
has no files" would be doing the work of "the directory was read and was clean",
which is CLAUDE.md 3.2 form 6.
"""

import os
import re
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts", "lint"))

import clock_scan as L  # noqa: E402
from xbrain.common import clock  # noqa: E402

SCRIPT = os.path.join(ROOT, "scripts", "lint", "clock_scan.py")

#: How far a reading from the module may sit from a reading taken here. Two
#: consecutive clock reads on an idle machine are microseconds apart; 50 ms is
#: slack for a loaded CI box and is still eight orders of magnitude below the
#: gap a wall-clock implementation would show.
TOLERANCE_S = 0.05


def _run_script(script_path, tree_root, *args):
    """(returncode, output) for one run of a scan script with ROOT repointed.

    A subprocess with ROOT patched rather than an in-process call: main() is
    what CI invokes, so main() is what the mutations must be shown to fail.
    Patching the imported module and calling scan_file directly would exercise a
    path CI never takes, and the surface printing lives in main().
    """
    module = os.path.basename(script_path)[:-len(".py")]
    code = (
        "import sys, importlib\n"
        "sys.path.insert(0, %r)\n"
        "m = importlib.import_module(%r)\n"
        "m.ROOT = %r\n"
        "sys.exit(m.main())\n"
        % (os.path.dirname(script_path), module, tree_root)
    )
    # Extra arguments go after -c, so they land in sys.argv exactly as they
    # would on a real command line and main() parses them the same way.
    proc = subprocess.run([sys.executable, "-c", code] + list(args),
                          capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


def _run_real(*args):
    """(returncode, output) for the live run over the repository."""
    proc = subprocess.run([sys.executable, SCRIPT] + list(args),
                          capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


# -- the module ----------------------------------------------------------------

def test_mono_now_s_reads_the_monotonic_clock_and_not_the_wall_clock():
    """*** The one assertion an empty shell cannot pass.

    Everything else about this function is shape: it returns a float, it grows,
    two calls differ. A wall-clock implementation satisfies all of that. What it
    cannot do is land within milliseconds of time.monotonic(), because the two
    clocks differ by the Unix epoch -- roughly 1.7e9 seconds.

    Mutation, and it was run: return the seconds-resolution wall clock instead
    => this case fails with a difference of about 1.7e9 seconds.
    """
    before = time.monotonic()
    reading = clock.mono_now_s()
    after = time.monotonic()
    assert isinstance(reading, float)
    # Bracketed rather than compared to a single sample, so the case cannot be
    # satisfied by a constant that happens to sit near one reading.
    assert before - TOLERANCE_S <= reading <= after + TOLERANCE_S, (
        "mono_now_s() is not reading CLOCK_MONOTONIC: it returned %r while "
        "time.monotonic() was between %r and %r" % (reading, before, after)
    )


def test_mono_now_s_does_not_go_backwards():
    """Monotonicity is the property the whole rule exists for.

    Weak on its own -- a constant would pass -- which is why the case above
    carries the real weight and this one only rules out a reading assembled from
    something non-monotonic, such as a difference of two wall-clock samples.
    """
    readings = [clock.mono_now_s() for _ in range(200)]
    assert readings == sorted(readings)
    # And it must actually advance, or "never goes backwards" is satisfied by a
    # frozen clock, which would make every timeout in the system infinite.
    start = clock.mono_now_s()
    time.sleep(0.005)
    assert clock.mono_now_s() > start


def test_mono_clock_reads_the_same_clock_as_the_module_function():
    """One implementation point, asserted rather than assumed.

    If the class ever grows its own call to the clock, a later change to the
    module-level function leaves half the processes on the old one -- and both
    halves keep returning plausible floats that subtract cleanly.
    """
    a = clock.mono_now_s()
    b = clock.MonoClock().now_s()
    c = clock.mono_now_s()
    assert a - TOLERANCE_S <= b <= c + TOLERANCE_S


def test_mono_clock_refuses_an_injected_clock_source():
    """*** The seam must not be usable to smuggle in a wall clock.

    A source parameter is the obvious convenience, and it is exactly how a
    component ends up wall-clock timed with nothing in the component to show it:
    MonoClock(source=time.time) is one line, it type-checks and it runs. Test
    doubles subclass instead, which is visible at a class definition.

    Mutation: give __init__ a source parameter => this case fails.
    """
    with pytest.raises(TypeError):
        clock.MonoClock(time.monotonic)          # type: ignore[call-arg]


def test_a_test_double_can_drive_a_component_without_sleeping():
    """The reason the class exists at all.

    A hysteresis window of 3.0 s tested through the real clock costs 3.0 s per
    case; suites shaped that way get slow, then get skipped, and then the
    hysteresis is not tested at all. This is the pattern components should use.
    """

    class FakeClock(clock.MonoClock):
        """A clock the test moves by hand."""

        def __init__(self, start_s):
            self.t = start_s

        def now_s(self):
            return self.t

    fake = FakeClock(1000.0)
    # A component under test would hold this object and see 3.0 s pass in no
    # time at all. Asserted here so the documented seam is known to work rather
    # than merely described in the module docstring.
    assert fake.now_s() == 1000.0
    fake.t += 3.0
    assert fake.now_s() == 1003.0


def test_the_two_names_the_item_requires_are_exported():
    """CFG-CM-12 names mono_now_s() and MonoClock verbatim.

    Pinned because the same module is claimed by a second TODO row under a
    different name -- see this suite's report notes. The name in force is the
    one here.
    """
    assert "mono_now_s" in clock.__all__
    assert "MonoClock" in clock.__all__
    assert callable(clock.mono_now_s)
    assert isinstance(clock.MonoClock, type)


def test_no_wall_clock_reader_is_exported_from_the_clock_package():
    """*** A boundary, deliberately pinned, not an accident of what got written.

    The permitted wall-clock uses (11 S0.2.1: the envelope ts, event time, media
    file names, cloud alignment, cross-host latency) stay at their call sites
    carrying the WALL-CLOCK-OK marker, because the marker is what makes each one
    reviewable one at a time. A friendly wall_now_s() here would turn three
    audited exceptions into a function anyone may call and leave no audit trail.

    If that decision is ever reversed, reverse it in the documents first: a
    passing test is not where a contract change belongs.
    """
    suspicious = [name for name in dir(clock)
                  if not name.startswith("_")
                  and any(word in name.lower()
                          for word in ("wall", "utc", "realtime", "epoch"))]
    assert not suspicious, (
        "the clock package must not offer a wall-clock reader: %s" % suspicious
    )


# -- the lint: scan surface ----------------------------------------------------

def test_scan_surface_is_exactly_the_four_trees_the_item_names():
    """*** Pinned, because shrinking it is invisible in the output.

    Mutation: drop ros2_ws or common from SCAN_DIRS => red. The lint would go on
    printing "violations: 0" and nothing in the report would say a tree had
    stopped being read.
    """
    assert L.SCAN_DIRS == ("common", "xbrain", "ros2_ws", "services")


def test_the_lint_is_outside_its_own_scan_surface():
    """*** CLAUDE.md 3.2 form 3, judgement self-harm.

    The script has to spell out the patterns it hunts for -- two of them,
    system_clock and the POSIX clock id, are single words that cannot be
    disguised at all. If scripts/ were in the surface the hit count could never
    reach zero, and the repair anyone reaches for is to loosen the criterion
    until it passes.

    The premise is checked rather than asserted in prose: the script is scanned
    here, and it must produce hits. Were that to stop being true, the exclusion
    would be guarding against nothing and should be re-examined, not deleted.
    """
    assert "scripts" not in L.SCAN_DIRS
    assert "tests" not in L.SCAN_DIRS, (
        "tests/ must stay out: fixtures below write violating files on purpose"
    )
    violations, _exempt, prose = L.scan_file(SCRIPT, "scripts/lint/clock_scan.py")
    assert violations or prose, (
        "the script no longer contains any spelling it searches for, so the "
        "exclusion above is now free -- check whether it is still needed "
        "instead of deleting this case"
    )


def test_ros2_ws_is_in_the_surface_and_its_emptiness_is_reported_not_hidden():
    """*** The case that keeps "no files" from posing as "read and clean".

    ros2_ws/ holds no source today. Without the per-directory counts, the report
    would read as four trees checked and all clean, and the C++ half of CLK-C1
    would be unevaluated with nothing saying so -- CLAUDE.md 3.2 form 6.

    Mutation: delete the NOTE lines and the counts from _print_surface => red.
    """
    assert "ros2_ws" in L.SCAN_DIRS
    _rc, out = _run_real()
    for tree in L.SCAN_DIRS:
        assert tree + "/" in out or "NOTE " + tree in out, (
            "every tree in the surface must appear in the report with either a "
            "file count or a NOTE saying it contributed nothing: " + tree
        )
    # The specific claim, stated so a future tree of C++ sources landing in
    # ros2_ws/ makes this case fail and someone re-reads it rather than
    # inheriting a stale sentence.
    n_ros2 = sum(1 for _ in L._files_under(os.path.join(ROOT, "ros2_ws"))) \
        if os.path.isdir(os.path.join(ROOT, "ros2_ws")) else 0
    if n_ros2 == 0:
        assert "NOTE ros2_ws/" in out
    else:
        assert "ros2_ws/  " in out or "ros2_ws/ " in out


# -- the lint: the rules it must carry ----------------------------------------

#: The six spellings the CFG-CM-12 criterion names, each with a sample line that
#: must be caught. Written out here rather than read from the script, on purpose:
#: a test that took its expectations from the code under test would agree with
#: any change to it, which is the shape CLAUDE.md 3.2 form 7 warns about.
NAMED_BY_ITEM = [
    # Each sample is written the way the mistake actually arrives: an age
    # subtraction, a timestamp assignment, a chrono type alias, a syscall, a
    # member initialisation. A sample reduced to the bare token would still
    # pass if the rule were narrowed to something unusable in practice.
    ("time.time()", "age = time.time() - self._last_rx"),
    ("datetime.now(...)", "stamp = datetime.now()"),
    ("datetime.utcnow()", "stamp = datetime.utcnow()"),
    ("system_clock", "auto t0 = std::chrono::system_clock::now();"),
    ("CLOCK_REALTIME", "clock_gettime(CLOCK_REALTIME, &ts);"),
    ("bare rclcpp::Clock()", "auto clk = rclcpp::Clock();"),
]


@pytest.mark.parametrize("name,sample", NAMED_BY_ITEM,
                         ids=[n for n, _ in NAMED_BY_ITEM])
def test_each_spelling_the_item_names_is_caught(tmp_path, name, sample):
    """*** One case per spelling in the criterion.

    Mutation: delete any one rule from RULES => that case fails while the other
    five stay green, which is the point -- a single count would drop from six to
    five and nothing would say which.
    """
    probe = tmp_path / "probe.cc"
    probe.write_text(sample + "\n", encoding="utf-8")
    violations, _exempt, _prose = L.scan_file(str(probe), "xbrain/probe.cc")
    assert violations, "%s was not caught in: %s" % (name, sample)
    # And it must be reported under the spelling a human would grep for, not as
    # an anonymous hit: the report is what someone acts on.
    assert name in violations[0][2]


@pytest.mark.parametrize("sample", [
    # The three forms CLK-C1 names as the required ones, one per language.
    "age = time.monotonic() - self._last_rx",
    "auto t0 = std::chrono::steady_clock::now();",
    "auto clk = rclcpp::Clock(RCL_STEADY_TIME);",
    # CLOCK_MONOTONIC differs from its forbidden neighbour by nine characters,
    # so a rule matching the common prefix would flag the correct code.
    "clock_gettime(CLOCK_MONOTONIC, &ts);",
    # CLK-C2 covers timers as well; this is the shape it asks for.
    "timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);",
    # asyncio's loop clock, which CLK-C2 names for Python. It contains the
    # substring the first rule looks for, so a pattern written without the
    # module boundary would flag every asyncio timeout in the system.
    "loop_now = loop.time()",
])
def test_the_compliant_spellings_are_not_flagged(tmp_path, sample):
    """*** The other half, without which the cases above prove nothing.

    A checker that flagged every line would pass all six violation cases. These
    are the forms CLK-C1 and CLK-C2 require, and flagging one of them would make
    the check permanently red -- CLAUDE.md 3.2 form 2 -- after which it gets
    loosened into a check that catches nothing.
    """
    probe = tmp_path / "probe.cc"
    probe.write_text(sample + "\n", encoding="utf-8")
    violations, _exempt, _prose = L.scan_file(str(probe), "xbrain/probe.cc")
    assert not violations, "compliant line was flagged: %s -> %s" % (sample,
                                                                     violations)


# -- the lint: mutation (1) and mutation (2) ----------------------------------

def test_mutation_one_wall_clock_age_in_p1_motion_turns_the_scan_red(tmp_path):
    """*** CFG-CM-12 mutation (1), verbatim: a wall-clock age line in p1_motion.

    Built in a fixture tree because xbrain/p1_motion/ does not exist yet, and
    because a test that writes into the repository leaves a poisoned file behind
    if it dies halfway. The connection to the real surface is asserted below.
    """
    proc_dir = tmp_path / "xbrain" / "p1_motion"
    proc_dir.mkdir(parents=True)
    (proc_dir / "watchdog.py").write_text(
        "import time\n"
        "def cmd_is_stale(last_rx, limit_s):\n"
        "    return time.time() - last_rx > limit_s\n",
        encoding="utf-8")

    rc, out = _run_script(SCRIPT, str(tmp_path))
    assert rc == 1, "the scan must exit non-zero:\n" + out
    assert "p1_motion/watchdog.py" in out
    assert "time.time()" in out


def test_mutation_two_bare_ros_clock_in_quadruped_turns_the_scan_red(tmp_path):
    """*** CFG-CM-12 mutation (2), verbatim: a default-constructed ROS clock.

    ros2_ws/ is empty today, so this too runs against a fixture. That is stated
    plainly rather than glossed: what the case proves is that the rule fires on
    a file at that path, and the pairing with
    test_ros2_ws_is_in_the_surface_and_its_emptiness_is_reported_not_hidden is
    what stops an empty directory from being read as a clean one.
    """
    node_dir = tmp_path / "ros2_ws" / "src" / "quadruped" / "src"
    node_dir.mkdir(parents=True)
    (node_dir / "tier1.cc").write_text(
        "#include <rclcpp/rclcpp.hpp>\n"
        "void Tier1::setup() {\n"
        "  clock_ = rclcpp::Clock();\n"
        "}\n",
        encoding="utf-8")

    rc, out = _run_script(SCRIPT, str(tmp_path))
    assert rc == 1, "the scan must exit non-zero:\n" + out
    assert "quadruped/src/tier1.cc" in out
    assert "rclcpp::Clock()" in out


def test_the_steady_form_of_the_same_quadruped_line_is_clean(tmp_path):
    """The positive half of mutation (2).

    CLK-C1 does not forbid rclcpp::Clock -- it forbids the default constructor.
    A rule matching the type name would catch the mutation above AND every
    correct use, which is how a check becomes permanently red and then gets
    weakened to nothing.
    """
    node_dir = tmp_path / "ros2_ws" / "src" / "quadruped" / "src"
    node_dir.mkdir(parents=True)
    (node_dir / "tier1.cc").write_text(
        "  clock_ = rclcpp::Clock(RCL_STEADY_TIME);\n", encoding="utf-8")

    rc, out = _run_script(SCRIPT, str(tmp_path))
    assert rc == 0, out


# -- the lint: mutation (3), the scan surface shrinking ------------------------

#: The mutation itself: a redefinition appended to a copy of the script, which
#: replaces the whole-tree walk with a changed-files list. Appended rather than
#: patched into place because the later definition wins at import, so the mutant
#: differs from the original by exactly this block and by nothing else.
DIFF_ONLY_MUTATION = '''

def iter_sources():
    """Mutation for tests: scan only what the version-control diff reports."""
    import subprocess
    out = subprocess.run(["git", "diff", "--name-only"], cwd=ROOT,
                         capture_output=True, text=True)
    for name in out.stdout.split():
        yield os.path.join(ROOT, name)
'''


def _surface_shrink_findings(script_path, fixture_root):
    """Report the ways a scan script may have stopped reading the whole tree.

    Three independent probes, because each one alone is evadable:

      1. behaviour -- a violation planted in a tree with no version-control
         history at all must still be found. A diff-scoped scan finds nothing
         there, and this is the probe that cannot be satisfied by wording.
      2. text -- the scan path must not consult version control. This catches
         the variant that keeps working in a real checkout, where probe 1 could
         accidentally pass because the fixture file shows up as untracked.
      3. the declaration -- the report must still claim a full-text scan. A
         script whose behaviour changed while its report did not is worse than
         one that fails, because the output actively misleads.
    """
    findings = []

    text = open(script_path, encoding="utf-8").read()
    # From the first function onward, so the module docstring -- which has to
    # explain why a diff is not the surface -- is not itself a finding.
    code = text[text.index("def _files_under"):]
    if re.search(r"\bgit\b", code):
        findings.append("scan surface shrunk: the scan path consults version "
                        "control instead of walking the declared trees")

    rc, out = _run_script(script_path, fixture_root)
    if rc == 0:
        findings.append("scan surface shrunk: a planted violation in a tree "
                        "with no version-control history went unreported")
    if "FULL TEXT" not in out:
        findings.append("scan surface shrunk: the report no longer declares "
                        "that it reads the full text of every file")
    return findings


@pytest.fixture()
def planted_violation(tmp_path):
    """A fixture tree holding one obvious violation, and no repository."""
    proc_dir = tmp_path / "xbrain" / "p2_core"
    proc_dir.mkdir(parents=True)
    (proc_dir / "health.py").write_text(
        "import time\n"
        "def age(t0):\n"
        "    return time.time() - t0\n",
        encoding="utf-8")
    return str(tmp_path)


def test_the_real_script_reads_the_full_text_of_the_declared_trees(
        planted_violation):
    """The three probes above, applied to the script as it ships."""
    assert _surface_shrink_findings(SCRIPT, planted_violation) == []


def test_mutation_three_a_diff_only_script_is_reported_as_surface_shrink(
        tmp_path, planted_violation):
    """*** CFG-CM-12 mutation (3), run rather than described.

    A copy of the script is rewritten to scan a changed-files diff, and the
    metatest above must report it. Doing it this way -- against a real mutant --
    is the only way to know the detector is not itself an assertion that has
    never been red: a text check on the shipped script alone would pass whether
    or not it can detect anything.
    """
    mutant = tmp_path / "clock_scan_diff_only_mutant.py"
    mutant.write_text(open(SCRIPT, encoding="utf-8").read() + DIFF_ONLY_MUTATION,
                      encoding="utf-8")

    findings = _surface_shrink_findings(str(mutant), planted_violation)
    assert findings, "the diff-only mutant was not reported as a shrunken surface"
    joined = " ".join(findings)
    assert "scan surface shrunk" in joined
    # Both the behavioural probe and the textual one must fire. If only the text
    # probe did, the detector would be a spelling check that a rename defeats.
    assert "went unreported" in joined
    assert "consults version control" in joined


# -- the lint: the exemption mechanism ----------------------------------------

def test_the_exempt_tag_set_is_closed_and_each_tag_says_what_it_licenses():
    """The three permitted wall-clock purposes, and nothing else.

    Mutation: add a tag => red. An open tag set degrades the mechanism into
    "write any word and the check goes quiet" -- an exemption visible in the
    file and invisible in review.
    """
    assert set(L.EXEMPT_TAGS) == {"align", "record", "latency"}
    for tag, why in L.EXEMPT_TAGS.items():
        assert len(why) > 60, (
            "tag %r must say which contract text permits it, not just name "
            "itself" % tag
        )


@pytest.mark.parametrize("marker,expect_violation", [
    # Accepted: a permitted tag with a reason that names the contract rule.
    ("WALL-CLOCK-OK(align): envelope ts, required on every message by CLK-C3",
     False),
    ("WALL-CLOCK-OK(record): media file name for the clip being written", False),
    # Refused: a tag nobody agreed to. This is the case that decides whether
    # the mechanism is a closed set or a magic word.
    ("WALL-CLOCK-OK(bogus): I would rather this did not fail", True),
    # Refused: the right tag with no reason. An exemption with nothing to argue
    # with cannot be reviewed, which is the only thing it is for.
    ("WALL-CLOCK-OK(align): ok", True),
    # Refused: prose that means to reassure. A comment near the line must not
    # be able to silence the check by sounding confident.
    ("just a comment saying this one is fine really", True),
])
def test_marker_forms(tmp_path, marker, expect_violation):
    """One case per way a marker can be right or wrong.

    Parametrised because what matters is that all five are checked: a suite
    covering only the accepting case would bless the other four.
    """
    probe = tmp_path / "probe.py"
    probe.write_text("# %s\nenv['ts'] = time.time()\n" % marker,
                     encoding="utf-8")
    violations, _exempt, _prose = L.scan_file(str(probe), "xbrain/probe.py")
    assert bool(violations) is expect_violation, violations


def test_declared_exemptions_are_printed_not_merely_honoured(tmp_path):
    """An exemption nobody sees is a silent one.

    Mutation: stop printing the exemption block => red. The violation count
    would still be right and the report would no longer show which lines are
    exempt, on whose authority, or how many there are.
    """
    proc_dir = tmp_path / "xbrain" / "p5_gateway"
    proc_dir.mkdir(parents=True)
    (proc_dir / "envelope.py").write_text(
        "def stamp(env):\n"
        "    # WALL-CLOCK-OK(align): the ts field CLK-C3 requires on every\n"
        "    # locally produced message; mono carries the age instead\n"
        "    env['ts'] = time.time()\n"
        "    return env\n",
        encoding="utf-8")

    rc, out = _run_script(SCRIPT, str(tmp_path))
    assert rc == 0, out
    assert "declared wall-clock exemptions" in out
    assert "p5_gateway/envelope.py" in out
    assert "align:" in out


def test_a_mention_in_prose_is_reported_but_not_failed(tmp_path):
    """The rule that lets the clock module explain itself.

    xbrain/common/clock/ has to name the forbidden calls in order to say why
    they are forbidden, and it sits inside the scan surface. Failing prose would
    make the only correct response "delete the explanation".

    Mutation: treat comment lines as code => this case fails, and so does the
    live repository run, because the clock module names them.
    """
    proc_dir = tmp_path / "xbrain" / "p4_agent"
    proc_dir.mkdir(parents=True)
    (proc_dir / "notes.py").write_text(
        '"""Ages are monotonic here; time.time() would step with chronyd."""\n'
        "VALUE = 1\n",
        encoding="utf-8")

    rc, out = _run_script(SCRIPT, str(tmp_path), "-v")
    assert rc == 0, out
    # Both halves: the line is listed under -v, and the counter says one. The
    # counter alone would pass if the listing were dropped, and the listing
    # alone would pass if a second mention were silently miscounted.
    assert "prose mentions:      1" in out
    assert "p4_agent/notes.py" in out


def test_the_script_has_no_undeclared_per_file_skip():
    """Guards against the exemption mechanism growing a second, silent form.

    The declared mechanism is the marker. A filename comparison inside the scan
    path would be an exemption nobody can find by reading the declarations,
    which is what the item's visible-marker requirement rules out. A text check
    is coarse, and it does catch the realistic case: someone adding
    `if "p1_motion" in path: return` to get a build green.
    """
    body = open(SCRIPT, encoding="utf-8").read()
    code = body[body.index("def _files_under"):]
    for suspicious in ('if "p1_motion', 'if "quadruped', "SKIP_FILES",
                       "IGNORE_FILES", "EXCLUDE_FILES", "NOQA_FILES"):
        assert suspicious not in code, (
            "found what looks like an undeclared per-file skip: %r" % suspicious
        )


# -- the lint: its own probes and the live run --------------------------------

def test_the_scripts_self_test_passes():
    """The per-rule probes, run as part of the suite so they cannot rot."""
    rc, out = _run_real("--self-test")
    assert rc == 0, out
    assert "self-test: PASS" in out


def test_the_repository_currently_passes():
    """The live run, which is what CI gates on.

    Today this passes because the four trees hold no wall-clock read. That is a
    fact about today, not a property of the code: the value of the check is that
    the first one to arrive fails this case.
    """
    rc, out = _run_real()
    assert rc == 0, out


def test_the_report_states_what_it_cannot_establish():
    """CLAUDE.md 3.2 form 7.

    A text scan cannot decide that no process reads a wall clock -- an alias or
    a function held in a variable matches nothing. A green run reads as that
    guarantee unless the output says otherwise, so the boundary is printed next
    to the number every time.

    Mutation: delete the caveat block from _print_criterion => red.
    """
    _rc, out = _run_real()
    assert "does NOT establish" in out
    assert "alias" in out
    assert "the tag is a claim" in out
