"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cross_lang_codes.py
Brief: Compile the C++17 error header and hold it byte-for-byte to the Python export

Description:
This is the golden test of CFG-CM-3. The E_* closed set is deployed to C++ as
common/include/xbrain/errors/errors.h, generated from the same
xbrain/common/errors/codes.yaml the Python package loads. Asserting "they cannot
drift" in prose is worth nothing, so this compiles the header with the flags
CLAUDE.md 5.6 mandates, runs a tiny program that prints one line per code, and
compares that output BYTE FOR BYTE with what the Python package produces for the
same codes in the same order. Each line is "code TAB retryable TAB detail" -- the
three fields a C++ consumer on the estop path needs, which is exactly what the
criterion names.

Why the C++ program is fixed rather than generated. The digest cross-language test
transcribes its VECTORS into C++ source, because the values under test live in the
test. Here the values live in the header already; the program only has to iterate
kAllCodes and print. So the program is ten lines with no data in it, and the only
thing under test is the header.

Why sorted order on both sides. The header emits the table sorted by code, and the
Python side iterates sorted(ALL_CODES). Neither coordinates with the other -- they
each sort the same strings by the same rule -- so a match is evidence the two
carry the same codes, and a mismatch names the first code that differs rather than
printing a diff of a whole file. This does NOT establish the codes are CORRECT:
that is the symmetric-difference metatest in tests/common/test_error_codes.py,
against 11 S13.4~S13.15. Comparing a generated artifact with its own generator is
true by construction (CLAUDE.md 3.2 form 7); the two claims are kept separate on
purpose.

The three mutations the criterion names, each run here and each turning a test red
(CLAUDE.md 3.3, an assertion never red has not been written):

  * (1) a code added to errors.h by hand -- the generated-artifact comparison, run
    as `gen_errors.py --check`, must go red. Performed against a copy so the
    committed header is never touched.
  * (2) `#include <rclcpp/rclcpp.hpp>` in errors.h -- the CFG-CM-14 no-ROS build
    (tests/common/link_no_ros/, driven by test_link_no_ros.py) must fail. errors.h
    is now one of the headers that build globs, so this exercises the same gate on
    the new file. Confirmed here by staging the gate over a sandbox with the
    mutation in the copied errors.h.
  * (3) `#include <sensor_msgs/msg/point_cloud2.hpp>` in a common/ header -- 19
    PMT-1 verbatim, the chassis_relay compile target must fail on the spot. Same
    sandbox, sensor_msgs in the copied errors.h.

If no C++ compiler is present the whole module skips, and it skips LOUDLY -- the
reason names exactly what went unverified, because a silent skip on a cross-language
test turns the single most important claim of this item into an empty green tick.
The two mutations that build through CMake carry a second skip for a missing cmake,
worded the same way.
"""

import os
import shutil
import subprocess
import sys

import pytest

# Three levels up from tests/common/: the repository root. Derived rather than
# written out, so a checkout somewhere other than /opt/xbrain_v6 still resolves.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import errors  # noqa: E402

#: The deployed header and the include root a consumer compiles against. The
#: include root, not the header path, is what goes on the -I line, because the
#: header refers to itself as "xbrain/errors/errors.h" and a consumer includes it
#: by that spelling -- compiling against the file's own directory would let a
#: wrong self-reference pass unnoticed.
HEADER = os.path.join(ROOT, "common", "include", "xbrain", "errors", "errors.h")
INCLUDE_ROOT = os.path.join(ROOT, "common", "include")
#: The generator, driven as a subprocess for the drift check so the exercised path
#: is the one CI runs, not an in-process shortcut.
GEN_SCRIPT = os.path.join(ROOT, "scripts", "gen", "gen_errors.py")

#: The pieces the CFG-CM-14 build needs, in the RELATIVE layout its CMakeLists
#: resolves through. link_no_ros/CMakeLists.txt derives the root as ../../.. and
#: does add_subdirectory(<root>/common), so a sandbox that flattened the tree would
#: resolve back to the real headers and every mutation would come out green -- a
#: mutation suite that is green because it mutates nothing. These two paths are the
#: originals; the sandbox helper below copies from them into the staged tree.
LINK_NO_ROS_DIR = os.path.join(ROOT, "tests", "common", "link_no_ros")
BASELINE_CMAKE = os.path.join(ROOT, "common", "CMakeLists.txt")

#: CLAUDE.md 5.6 and 5.2. -std=c++17 exactly, not "at least": the header must keep
#: compiling on the C++17 baseline CPP-1 fixes, and building it as C++20 here would
#: let a C++20-only construct slip in unnoticed. -Werror so a warning in the header
#: -- an unused constant, a narrowing conversion in an initializer -- is a failure
#: rather than noise a reader scrolls past. -O1 so the optimiser at least looks at
#: the code, without paying for a full release build of a ten-line program.
CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]

#: Either compiler is acceptable; the header uses only standard C++17. cmake is
#: separate because only the two sandbox mutations need it, and a machine with a
#: compiler but no cmake should still run the golden comparison rather than skip
#: the whole module.
CXX = shutil.which("g++") or shutil.which("clang++")
CMAKE = shutil.which("cmake")

pytestmark = pytest.mark.skipif(
    CXX is None,
    reason="no C++ compiler on PATH, so the cross-language agreement required by "
           "CFG-CM-3 was NOT verified in this run -- the Python-side error-code "
           "tests passing does not establish that the deployed C++ header carries "
           "the same set",
)

#: The whole program under test. It includes the header, walks the table, and
#: prints exactly the golden line shape. Deliberately tiny: the only C++ under test
#: is the header, so the program must add nothing that could itself pass or fail.
#: No code literal is spelled here -- it reads the codes out of kAllCodes rather
#: than naming any -- so this test cannot accidentally become a second copy of the
#: spelling. operator<< for string_view is the C++17 feature that lets a view print
#: without a separate length argument; it ties this program to the same standard
#: the header needs. The tabs and the trailing newline per line are what _python
#: _expected reproduces byte for byte.
CXX_PROGRAM = (
    '#include "xbrain/errors/errors.h"\n'
    "#include <iostream>\n"
    "int main() {\n"
    "  for (const auto& c : hachist::xbrain::errors::kAllCodes) {\n"
    "    std::cout << c.code << '\\t' << c.retryable << '\\t' << c.detail"
    " << '\\n';\n"
    "  }\n"
    "  return 0;\n"
    "}\n"
)


def _python_expected():
    """The golden output, computed from the Python package.

    This is the reference the C++ program is held to. It is produced the same way
    every runtime process would produce it, so the test measures agreement with the
    shipping code path rather than with a transcription of it.

    sorted(ALL_CODES) is the order. It matters that this is a bare sort and not a
    hand-kept list: the header's table is emitted in the same bare sort, so the two
    orders coincide by rule rather than by anyone keeping them in step. info() is
    the single lookup point in the package, so retryable and detail come from the
    exact place a consumer would read them, already validated against their closed
    vocabularies at import.
    """
    lines = []
    # One line per code, tab separated: code, then the safety-classification
    # column, then the detail requirement. The same three fields, in the same
    # order, the C++ program prints -- see CXX_PROGRAM above.
    for code in sorted(errors.ALL_CODES):
        info = errors.info(code)
        lines.append("%s\t%s\t%s" % (info.code, info.retryable, info.detail))
    # Joined with newlines AND given a trailing one. The trailing newline is not
    # cosmetic: the C++ program prints a newline after every line including the
    # last, so an expectation without it would be one byte short and the byte
    # comparison would fail for a reason that is not a defect.
    return "\n".join(lines) + "\n"


@pytest.fixture(scope="module")
def cxx_output(tmp_path_factory):
    """Compile the program against the real header, run it, return its stdout.

    Module scoped: compiling is the slow part and several tests read the same
    result. The assertion lives in the fixture so a build break reports once, with
    the compiler output attached, rather than as a cascade of unreadable failures.
    """
    # A pytest temporary directory, never the repository: the compile leaves an
    # object file and a binary, and writing those into the tree would both dirty it
    # and risk a later run reading a stale artefact.
    work = tmp_path_factory.mktemp("cross_lang_codes")
    src = work / "print_codes.cc"
    src.write_text(CXX_PROGRAM, encoding="utf-8")
    exe = work / "print_codes"

    # The program is written out and compiled, not interpreted: there is no way to
    # ask a C++ header a question except to compile something that uses it, which
    # is why every cross-language check in this tree ends in a real build.
    # -I points at the real deployed include root, so the header under test is the
    # committed one and not a copy. The flags are the mandated set: building at
    # exactly C++17 is part of the claim, because the header uses inline constexpr
    # string_view, which is a C++17 feature, and a C++20 build would hide a
    # regression that only C++17 would reject.
    compile_cmd = [CXX] + CXX_FLAGS + ["-I", INCLUDE_ROOT, str(src), "-o", str(exe)]
    proc = subprocess.run(compile_cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        # The generated source is kept and its path reported: a compiler error in
        # a header is unreadable without the translation unit it refers to. pytest
        # .fail rather than assert so the message is attached to a failure and not
        # buried under an AssertionError repr of the whole command.
        pytest.fail("errors.h failed to compile (source kept at %s)\n%s"
                    % (src, proc.stderr))

    # Run it. The output is what the whole test turns on, so a non-zero exit is a
    # hard failure here rather than something a caller has to notice: a program
    # that built but crashed would otherwise return an empty string that a lenient
    # comparison could accept.
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, "the C++ program crashed:\n" + run.stderr
    return run.stdout


def test_the_two_languages_print_byte_identical_output(cxx_output):
    """*** The central claim of CFG-CM-3.

    Byte for byte, not set equality. The retryable and detail columns travel with
    each code on the same line, so a header that carried the right CODES but a
    wrong retryable would pass a membership test and fail here -- which is the
    point, because retryable is a safety column (11 S13.2) and a cloud client that
    read it wrong would resend motion commands into a locked machine. A single
    equality is enough because the ordering is fixed on both sides; when it fails,
    pytest's own diff names the first differing line.
    """
    assert cxx_output == _python_expected()


def test_every_code_was_actually_printed_by_the_cxx_side(cxx_output):
    """Guards the guard.

    The byte comparison above would also pass if BOTH sides produced nothing, and a
    header whose table rendered empty is exactly the failure that would produce --
    an empty-shell pass, the shape CLAUDE.md 3.2 form 1 warns about. So the line
    count is asserted against the closed set independently: an empty program cannot
    pass by matching an empty expectation here, because ALL_CODES is not empty and
    this count is taken from it, not from the program's own output.
    """
    # Blank lines dropped so the trailing newline does not count as a code. What
    # remains is one line per code the C++ side actually printed.
    printed = [ln for ln in cxx_output.split("\n") if ln]
    assert len(printed) == len(errors.ALL_CODES), (
        "the C++ program printed %d codes for a closed set of %d"
        % (len(printed), len(errors.ALL_CODES))
    )


def test_the_header_pulls_in_no_ros_and_no_third_party():
    """CLAUDE.md 5.3, the cheap check that names the file.

    tests/common/link_no_ros/ is the real gate: it compiles every header under
    common/include with no ROS include directory on the command line, so a ROS
    include added here would surface there as a build failure. This case is the
    cheap companion that reads the include lines directly, so the same mistake is
    reported against THIS header by name rather than as a compile error inside an
    aggregate header nobody wrote by hand. The two are not redundant: the build
    gate is authoritative, this one is legible.
    """
    with open(HEADER, encoding="utf-8") as fh:
        includes = [ln.strip() for ln in fh if ln.strip().startswith("#include")]
    # string_view and nothing else. The exact set is asserted rather than "no ROS
    # in it" for two reasons: an extra standard header is cheap to allow here
    # explicitly the day it is genuinely needed, and a membership test against a
    # denylist would have to guess every ROS name a stray include could carry. A
    # positive allowlist cannot be defeated by an unfamiliar name.
    assert includes == ["#include <string_view>"], (
        "errors.h includes something outside the string_view-only budget its "
        "estop-path consumers allow: %r" % includes
    )


def test_the_header_declares_no_distribution_macros():
    """13 PB-5: the humble/jazzy baseline D-45 is not decided.

    A version macro written now would have to be torn out when it is, and would
    meanwhile make the header behave differently on the two platforms while both
    claim to pass this suite -- the quiet divergence PB-5 exists to prevent. This
    header has no reason to branch on a distribution at all, so the check is a
    tripwire against one being added later rather than a live risk today. The
    __has_include probe is listed because it is the subtle spelling: it reads as
    portability rather than as distribution detection, and it is exactly how a ROS
    dependency creeps back in behind a conditional.
    """
    with open(HEADER, encoding="utf-8") as fh:
        text = fh.read()
    for banned in ("ROS_DISTRO", "HUMBLE", "JAZZY", "__has_include(<rclcpp"):
        assert banned not in text, "PB-5 forbids distribution detection: " + banned


# ---------------------------------------------------------------------------
# Mutation (1): a code added to the generated header by hand must be caught by
# the drift check. Performed against a copy, never the committed file.
# ---------------------------------------------------------------------------

def test_the_committed_header_matches_a_fresh_render():
    """The drift gate, run as CI would run it -- the control for mutation (1).

    If this is red on a clean checkout, the committed header was hand-edited or the
    generator changed without a regenerate; either way the two languages may have
    drifted. It is the positive half of the mutation below: the same --check that
    passes here must go red when a code is added by hand.
    """
    proc = subprocess.run([sys.executable, GEN_SCRIPT, "--check"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


def _run_check_against(out_path):
    """Run gen_errors.main() with OUT_PATH redirected; return (rc, stdout).

    A subprocess with OUT_PATH overridden, rather than a monkeypatch in this
    process, for two reasons. main() is what CI invokes, so main() is what gets
    exercised -- a test that reached inside render() would be checking a function
    the gate never calls. And the override lives only in the child process, so the
    committed header is never in reach of the edit even transiently.
    """
    # The child imports the generator, repoints OUT_PATH at the copy under test,
    # and returns whatever main() returns. sys.path.insert is how the child finds
    # the module without the package being installed, matching how the other
    # generator tests in this tree drive their scripts.
    driver = (
        "import sys\n"
        "sys.path.insert(0, %r)\n"
        "import gen_errors as g\n"
        "g.OUT_PATH = %r\n"
        "sys.exit(g.main())\n" % (os.path.join(ROOT, "scripts", "gen"), str(out_path))
    )
    # --check is passed on the child's argv, so it takes the compare-not-write
    # branch. Output is captured so the caller can assert on the DRIFT line rather
    # than only on the exit code.
    proc = subprocess.run([sys.executable, "-c", driver, "--check"],
                          capture_output=True, text=True)
    return proc.returncode, proc.stdout


def test_a_hand_edit_to_the_generated_header_is_detected(tmp_path):
    """*** Criterion mutation (1), performed rather than described.

    A copy of the header with one extra code added to the table by hand. The
    generator is pointed at the copy through its module global, so the committed
    file is never touched -- a mutation that edits a tracked file and restores it
    leaves the tree broken whenever it is interrupted.
    """
    victim = tmp_path / "errors.h"
    text = open(HEADER, encoding="utf-8").read()
    # Added into the real table, as a second code alongside an existing one, which
    # is the edit somebody would actually make: a code quietly added on the C++
    # side only. kEGhost is not defined, but --check compares bytes and does not
    # compile, so the added line is enough to make the copy differ from a render --
    # which is the realistic defect this drift check exists to catch.
    mutated = text.replace('    {kEBusy, "conditional", "unspecified"},\n',
                           '    {kEBusy, "conditional", "unspecified"},\n'
                           '    {kEGhost, "no", "unspecified"},\n', 1)
    # If the anchor row ever moves or is reformatted, the replace above becomes a
    # no-op and the "mutation" would test nothing while reporting a pass. Asserting
    # the text actually changed is what stops that silent rot (CLAUDE.md 3.3).
    assert mutated != text, "the anchor row moved; re-derive this mutation"
    victim.write_text(mutated, encoding="utf-8")

    # The mutated copy must be reported as drift: exit 1 and the DRIFT line. Both
    # are asserted, because a generator that exited 1 for an unrelated reason
    # (a missing file, say) would satisfy the code alone.
    rc, out = _run_check_against(victim)
    assert rc == 1, "a hand-added C++ code went undetected:\n" + out
    assert "DRIFT" in out
    # The control: the same driver against the UNMUTATED copy must pass. Without
    # it, the red above could be coming from the redirection machinery rather than
    # from the edit, and the mutation would prove nothing.
    control = tmp_path / "control.h"
    control.write_text(text, encoding="utf-8")
    rc_ok, _out_ok = _run_check_against(control)
    assert rc_ok == 0


# ---------------------------------------------------------------------------
# Mutations (2) and (3): a ROS include in errors.h must break the CFG-CM-14
# no-ROS build. Staged over a sandbox copy so the committed header is never
# touched, not even transiently -- a mutation that edits a tracked file and
# restores it leaves the tree broken whenever it is interrupted.
#
# These re-stage the CFG-CM-14 build rather than importing test_link_no_ros.py's
# helpers on purpose. That module tests the gate in general, over canonical_digest
# .h; this one tests that errors.h SPECIFICALLY is inside it. The claims are
# different, so the two suites are kept independent, and a break in one names its
# own file rather than a shared fixture.
# ---------------------------------------------------------------------------

# A separate skip from the module-level one: the golden comparison needs only a
# compiler, but staging the CFG-CM-14 build needs cmake too. Split so a machine
# with a compiler and no cmake still runs the central byte-comparison claim and
# skips only the two build mutations, LOUDLY -- the reason names exactly what went
# unverified, because a silent skip on a mutation turns it into an empty green tick.
needs_cmake = pytest.mark.skipif(
    CMAKE is None,
    reason="cmake is missing, so the CFG-CM-14 build could not be staged and the "
           "claim that a ROS include in errors.h breaks it (criterion mutations 2 "
           "and 3) was NOT verified in this run",
)


def _stage_sandbox(tmp_path):
    """A copy of the three pieces the build needs, in their relative layout.

    Three pieces, because the CFG-CM-14 build is three files that find each other
    by relative path: the deployed headers, the common/ build baseline, and the
    link_no_ros translation unit and its project file. This mirrors the sandbox in
    test_link_no_ros.py; it is reproduced here rather than imported because this
    file tests a different claim -- that errors.h SPECIFICALLY is inside the gate
    -- and coupling to another test module's private helper would make this test
    fail for reasons that have nothing to do with errors.h.

    The layout is reproduced exactly, not flattened. common/CMakeLists.txt finds
    its headers through a path relative to itself, and link_no_ros/CMakeLists.txt
    derives the repository root as ../../.. from its own location. A sandbox that
    put them at any other depth would resolve back to the REAL headers -- and every
    mutation that edits a header would then be applied to a file nothing reads,
    come out green, and be reported as run. That is the worst outcome available
    here: a mutation suite that is green because it mutates nothing.
    """
    sandbox = tmp_path / "tree"                       # stands in for the repo root
    # The two parent directories are made explicitly; copytree below will create
    # the leaf, but not the intermediates, on older Python.
    os.makedirs(sandbox / "common")                   # baseline plus headers side
    os.makedirs(sandbox / "tests" / "common")         # the gate side
    # The headers are copied WHOLE, so a mutation edits the copy and the real
    # deployment artefacts are never touched, not even for the length of one test.
    shutil.copytree(INCLUDE_ROOT, str(sandbox / "common" / "include"))
    # The build baseline: the function that pins C++17 and the warning flags, and
    # the glob that pulls every deployed header into the aggregate the gate builds.
    shutil.copy(BASELINE_CMAKE, str(sandbox / "common" / "CMakeLists.txt"))
    # The translation unit and its project file, at the depth its CMakeLists
    # expects relative to the common/ copy above.
    shutil.copytree(LINK_NO_ROS_DIR,
                    str(sandbox / "tests" / "common" / "link_no_ros"))
    return sandbox


def _build_sandbox(sandbox, tmp_path):
    """Configure and build the sandbox copy; return (ok, combined output).

    Two subprocess steps, configure then build, because CMake separates them and a
    mutation can fail at either: a ROS include fails while compiling, but a broken
    glob would fail while configuring. The output of both is concatenated and
    returned, because a caller that kept only one half would report a red with no
    reason attached -- and the attribution assertions below need the compiler's own
    words to tell a real catch from an unrelated breakage.
    """
    source = str(sandbox / "tests" / "common" / "link_no_ros")   # the project dir
    build = str(tmp_path / "build")                              # out-of-source
    # Build type is always passed explicitly. Left empty it matches neither the
    # Debug nor the Release generator expressions in the baseline, which would make
    # the definitions it guards evaluate to nothing.
    cfg = subprocess.run([CMAKE, "-S", source, "-B", build,
                          "-DCMAKE_BUILD_TYPE=Release"],
                         capture_output=True, text=True)
    # Returned early on a failed configure, because "cmake --build" on a tree that
    # never configured prints its own, uninformative, error on top of the real one.
    if cfg.returncode != 0:
        return False, cfg.stdout + cfg.stderr
    bld = subprocess.run([CMAKE, "--build", build], capture_output=True, text=True)
    return bld.returncode == 0, cfg.stdout + cfg.stderr + bld.stdout + bld.stderr


def _prepend_to_sandbox_errors_h(sandbox, line):
    """Inject a line at the very top of the sandbox copy of errors.h.

    Prepended rather than appended, and the position is load bearing: an include
    at the very top lands before the include guard and outside every namespace,
    which is where a real ROS include would be written and the only position from
    which it is processed unconditionally. Appended inside the guard, a second
    inclusion would skip it and the mutation could come out green.
    """
    # The exact path of errors.h inside the copied include tree. Only the copy is
    # ever written; the committed header is untouched.
    path = sandbox / "common" / "include" / "xbrain" / "errors" / "errors.h"
    text = path.read_text(encoding="utf-8")
    path.write_text(line + "\n" + text, encoding="utf-8")


@needs_cmake
def test_the_sandbox_itself_builds(tmp_path):
    """The control for the two mutations below.

    Without it a sandbox staged wrong -- a missing header, the wrong relative
    depth -- would make both mutations come out red and this pair would be green
    for a reason that has nothing to do with a ROS include. That is the same shape
    as a criterion that can never reach zero: the mutations would "pass" while
    testing nothing.

    It carries a second meaning too. errors.h now sits under common/include, so
    this unmutated build is the first proof that adding the header did not break
    the CFG-CM-14 gate -- the aggregate still compiles with the new header pulled
    in through the glob.
    """
    sandbox = _stage_sandbox(tmp_path)
    ok, output = _build_sandbox(sandbox, tmp_path)
    assert ok, "the unmutated sandbox must build:\n" + output


@needs_cmake
def test_mutation_rclcpp_in_errors_h_breaks_the_no_ros_build(tmp_path):
    """*** Criterion mutation (2): errors.h is under the CFG-CM-14 no-ROS gate.

    The criterion words this as "rclcpp in errors.h => the CM-14 no-ROS link test
    must fail". The CM-14 gate reaches a header only if its translation unit
    includes it, and errors.h reaches the compiler through the aggregate the build
    globs from common/include. So this mutation proves two things at once: that the
    gate breaks on a ROS include, and that the new header is really inside the gate
    and not merely sitting beside it.

    On this machine the build dies while preprocessing, because nothing puts a ROS
    include directory on the command line -- which is exactly the failure the gate
    is meant to produce. On a machine that already had a ROS prefix on the search
    path an include with no symbol reference could preprocess; that half of the
    claim is carried by test_link_no_ros.py's positive assertions (no ROS directory
    in the compile command, no ROS shared object in ldd), and is not re-derived
    here.
    """
    sandbox = _stage_sandbox(tmp_path)
    _prepend_to_sandbox_errors_h(sandbox, "#include <rclcpp/rclcpp.hpp>")
    ok, output = _build_sandbox(sandbox, tmp_path)
    assert not ok, ("errors.h included rclcpp and the no-ROS target still built; "
                    "19 S1.2 requires that to be impossible")
    # Attribution. Without this the test would also pass if the sandbox build broke
    # for an unrelated reason, and a red whose cause cannot be pinned proves
    # nothing (CLAUDE.md 3.2).
    assert "rclcpp" in output


@needs_cmake
def test_mutation_sensor_msgs_in_errors_h_breaks_the_build(tmp_path):
    """*** Criterion mutation (3), 19 PMT-1 verbatim.

    PMT-1 states the mutation in terms of sensor_msgs and names the chassis_relay
    compile target as what must fail on the spot. tests/common/link_no_ros/ is that
    target's stand-in -- the C++ translation unit that links common/ with no ROS,
    written precisely because chassis_relay's own sources do not exist yet. errors.h
    is a common/ header, so a sensor_msgs include in it is one instance of the "any
    common/ header" the rule forbids.

    Kept separate from the rclcpp mutation rather than parametrised over the two
    strings. The two are different claims -- one is the criterion's own spelling,
    the other is 19 PMT-1's -- and reading them as two named tests is what makes a
    failure say which claim broke.
    """
    sandbox = _stage_sandbox(tmp_path)
    _prepend_to_sandbox_errors_h(
        sandbox, "#include <sensor_msgs/msg/point_cloud2.hpp>")
    ok, output = _build_sandbox(sandbox, tmp_path)
    assert not ok, ("errors.h included sensor_msgs and the target still built; 19 "
                    "PMT-1 requires the chassis_relay compile target to fail")
    # Attribution again: the diagnostic has to be about sensor_msgs, not about some
    # other breakage the rewrite happened to introduce.
    assert "sensor_msgs" in output
