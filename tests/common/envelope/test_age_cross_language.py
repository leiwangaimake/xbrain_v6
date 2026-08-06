"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_age_cross_language.py
Brief: INF-CM-2 criterion four -- Python and C++ produce the same age, measured

Description:
Criterion four requires that Python and C++ compute a byte-identical age and
decision for the same envelope. Asserting that in prose is worth nothing, so this
compiles common/include/xbrain/envelope/message_age.h with the flags CLAUDE.md
5.6 mandates, runs ComputeAge over the same golden vectors test_age.py uses, and
compares the C++ output to BOTH the golden oracle and the Python result.

Why the vectors are transcribed into C++ source rather than parsed at run time --
identical reasoning to test_digest_cross_language.py. A C++ JSON reader would sit
between the header and the comparison, and a bug in it could mask a disagreement
or invent one. The generator emits each vector's scalar fields as a direct
ComputeAge call, so the only C++ code under test is the header's arithmetic and
its boot comparison.

*** What this cannot check, so the pass is not read as more than it is. The
transcription is written here, in Python: it decides which vector fields become
which C++ arguments (has_mono from the presence of a mono key, and so on). What it
establishes is that GIVEN the same scalar inputs, the two age computations agree
to the byte. The Python side additionally decodes the real JSON envelope in
test_age.py, so "the envelope decodes to those scalars" is covered there.

If no C++ compiler is present the module skips LOUDLY -- the reason names exactly
what went unverified -- because a silent skip on a cross-language test turns the
single most important claim of this item into an empty green tick.
"""

import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.envelope import compute_age, decode  # noqa: E402

GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden",
                      "message_age_vectors.json")
HEADER_DIR = os.path.join(ROOT, "common", "include")

#: -std=c++17 exactly, not "at least" (CLAUDE.md 5.6 / 5.2): the header must keep
#: compiling on the C++17 baseline CPP-1 fixes, and building it as C++20 here
#: would let a C++20-only construct slip in unnoticed.
CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]

CXX = shutil.which("g++") or shutil.which("clang++")

pytestmark = pytest.mark.skipif(
    CXX is None,
    reason="no C++ compiler on PATH, so the cross-language agreement required "
           "by INF-CM-2 criterion four was NOT verified in this run -- the "
           "Python-side golden tests passing does not establish it",
)


def _vectors():
    """The shared golden vectors."""
    with open(GOLDEN, encoding="utf-8") as fh:
        return json.load(fh)["vectors"]


def _cxx_string(text):
    """A C++ string literal carrying the exact bytes of a Python str.

    Every byte is emitted as a \\xNN escape, then the run is broken with empty
    literals so C++ does not read \\xE5a as one over-long hex escape. Identical to
    the digest harness's helper, and used for boot / local_boot_id here even
    though they are plain hex -- escaping everything removes the question of
    whether a value needs it.
    """
    out = []
    for byte in text.encode("utf-8"):
        out.append("\\x%02x" % byte)
    return '"' + '""'.join(out) + '"'


def _generate_program():
    """One translation unit that prints a TSV line per vector.

    One program, not one per vector: compiling is the slow part, and a compile
    per vector would push this test into the territory where someone disables it.
    Each line is name, branch, was_negative (0/1), raw age, clamped age -- the
    same fields the Python side compares, so a disagreement can be pinned to a
    field rather than just "the line differs".
    """
    lines = [
        '#include "xbrain/envelope/message_age.h"',
        "#include <cstdio>",
        "#include <string>",
        "using hachist::xbrain::envelope::ComputeAge;",
        "using hachist::xbrain::envelope::AgeResult;",
        "using hachist::xbrain::envelope::FormatAge;",
        "",
        "int main() {",
    ]
    for vec in _vectors():
        env = vec["envelope"]
        # has_mono is the presence of a mono key -- exactly what decode() keys the
        # cloud fallback on. When absent, mono's value is unused, so 0.0 is a safe
        # filler that the header never reads.
        has_mono = "mono" in env
        mono = env.get("mono", 0.0)
        boot = env.get("boot", "")
        lines.append("  {")
        # repr() round-trips a Python float to the exact same double the C++
        # compiler parses back, so the two languages subtract identical bits.
        lines.append("    AgeResult r = ComputeAge(%s, %r, %s, %r, %r, %s);"
                     % ("true" if has_mono else "false", float(mono),
                        _cxx_string(boot), float(vec["rx_mono"]),
                        float(vec["now_mono"]), _cxx_string(vec["local_boot_id"])))
        # Every printf specifier is passed as a %s ARGUMENT at the Python level so
        # Python's % operator never tries to interpret the C++ "%d" itself (it
        # would demand an int). The C++ format ends up name / branch / flag / raw
        # / age = %s %s %d %s %s; the numbers go through the header's own renderer
        # so the test exercises FormatAge too, not just ComputeAge.
        lines.append('    std::printf("%s\\t%s\\t%s\\t%s\\t%s\\n", %s, r.branch, '
                     'r.was_negative ? 1 : 0, FormatAge(r.raw_age_s).c_str(), '
                     'FormatAge(r.age_s).c_str());'
                     % ("%s", "%s", "%d", "%s", "%s", _cxx_string(vec["name"])))
        lines.append("  }")
    lines.append("  return 0;")
    lines.append("}")
    return "\n".join(lines) + "\n"


@pytest.fixture(scope="module")
def cxx_results(tmp_path_factory):
    """Compile once, run once, return {name: (branch, negative, raw, age)}."""
    work = tmp_path_factory.mktemp("age_cxx")
    src = work / "vectors.cc"
    src.write_text(_generate_program(), encoding="utf-8")
    exe = work / "vectors"

    compile_cmd = [CXX] + CXX_FLAGS + ["-I", HEADER_DIR, str(src), "-o", str(exe)]
    proc = subprocess.run(compile_cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        # Keep the generated source and name it: a compiler error in generated
        # code is unreadable without the file it points at.
        pytest.fail("C++ compilation failed (source kept at %s)\n%s"
                    % (src, proc.stderr))

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, "the C++ program crashed:\n" + run.stderr

    results = {}
    for line in run.stdout.strip().split("\n"):
        name, branch, negative, raw, age = line.split("\t", 4)
        results[name] = (branch, negative == "1", raw, age)
    return results


@pytest.mark.parametrize("vec", _vectors(), ids=lambda v: v["name"])
def test_cxx_age_matches_the_golden_vector(vec, cxx_results):
    """The C++ output equals the hand-verified golden oracle, field by field."""
    branch, negative, raw, age = cxx_results[vec["name"]]
    # Branch and negative flag are the "判定" (decision) half of criterion four;
    # raw and clamped age are the "age" half.
    assert branch == vec["expected_branch"]
    assert negative is vec["expected_negative"]
    assert raw == vec["expected_raw"]
    assert age == vec["expected_age"]


@pytest.mark.parametrize("vec", _vectors(), ids=lambda v: v["name"])
def test_cxx_and_python_agree_byte_for_byte(vec, cxx_results):
    """*** Criterion four stated directly: same envelope, same age, both languages.

    Rather than lean on both sides matching the golden transitively, this computes
    the Python age here and asserts the C++ rendering equals it character for
    character -- the byte-identical claim, made against the other language and not
    only against the file.
    """
    env = decode(vec["envelope"])
    py = compute_age(env, rx_mono=vec["rx_mono"], now_mono=vec["now_mono"],
                     local_boot_id=vec["local_boot_id"])
    branch, negative, raw, age = cxx_results[vec["name"]]
    assert branch == py.branch
    assert negative is py.was_negative
    # %.17g on both sides: Python via the % operator, C++ via FormatAge. Equal
    # strings mean equal bits under one rule, which is the whole cross-language
    # guarantee.
    assert raw == "%.17g" % py.raw_age_s
    assert age == "%.17g" % py.age_s


def test_every_vector_was_actually_run_by_the_cxx_side(cxx_results):
    """Guards the guard: a generator that emitted nothing would leave a green run.

    The parametrized tests look their vector up by name and would KeyError if one
    were missing -- loud. But a generator emitting nothing for ALL vectors would
    leave the map empty and the parametrization non-empty only because _vectors()
    is read directly; this asserts the counts match so a silent empty C++ run is
    caught.
    """
    assert len(cxx_results) == len(_vectors())


def test_the_header_pulls_in_no_ros_and_no_third_party():
    """CLAUDE.md 5.3: common/ must be usable from chassis_relay and quadruped.

    Checked by reading the includes rather than by the compile succeeding: the
    compile would also pass if ROS happened to be installed on the build machine,
    then start failing on a machine where it is not.
    """
    path = os.path.join(HEADER_DIR, "xbrain", "envelope", "message_age.h")
    with open(path, encoding="utf-8") as fh:
        includes = [ln.strip() for ln in fh if ln.strip().startswith("#include")]
    # The standard-library subset this header is allowed to use. cstdio for
    # snprintf, string for std::string; nothing else, and certainly no rclcpp.
    allowed = {"<cstdio>", "<string>"}
    for line in includes:
        target = line.split(None, 1)[1].strip()
        assert target in allowed, (
            "%s is outside the standard-library subset this header may use; its "
            "consumers include chassis_relay, which may not link ROS" % target
        )


def test_the_header_declares_no_distribution_macros():
    """13 PB-5: no humble/jazzy detection, even though U74 settled on Humble.

    A version macro written now would behave differently on the two platforms
    while both claim to pass this suite, and would have to be torn out. The header
    must stay platform-neutral.
    """
    path = os.path.join(HEADER_DIR, "xbrain", "envelope", "message_age.h")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    for banned in ("ROS_DISTRO", "HUMBLE", "JAZZY", "__has_include(<rclcpp"):
        assert banned not in text, "PB-5 forbids distribution detection: " + banned
