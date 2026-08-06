"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_digest_cross_language.py
Brief: Compile the C++17 header and hold it to the same golden vectors as Python

Description:
CFG-CM-10 requires that Python and C++ produce the same value for every golden
vector. Asserting that in prose is worth nothing, so this compiles
common/include/xbrain/digest/canonical_digest.h with the flags CLAUDE.md 5.6
mandates, runs it over the same two JSON files test_digest.py uses, and compares.

Why the vectors are transcribed into C++ source rather than parsed at run time.
Parsing them would require a JSON reader on the C++ side, and that reader would
then sit between the header and the comparison -- a bug in it could mask a
disagreement, or invent one. The generator below emits each vector as literal
constructor calls, so the only C++ code under test is the header itself.

*** What this cannot check, stated so the pass is not read as more than it is.
The transcription is written by this file, in Python. It faithfully reproduces
the vector's VALUES, but it chooses the C++ types: an integer vector becomes
Value::Int and a float one becomes Value::Double. A C++ consumer that receives
the same configuration through a different YAML reader might choose differently,
and this test would not see it. What it does establish is that given the same
values in the same types, the two serialisers and the two hashes agree.

If no C++ compiler is present the whole module skips. It skips LOUDLY -- the
reason names exactly what went unverified -- because a silent skip on a
cross-language test turns the single most important claim of this item into an
empty green tick.
"""

import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

GOLDEN = os.path.join(ROOT, "tests", "common", "golden")
HEADER_DIR = os.path.join(ROOT, "common", "include")

#: CLAUDE.md 5.6 and 5.2. -std=c++17 exactly, not "at least": the header must
#: keep compiling on the C++17 baseline CPP-1 fixes, and building it as C++20
#: here would let a C++20-only construct slip in unnoticed.
CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]

CXX = shutil.which("g++") or shutil.which("clang++")

pytestmark = pytest.mark.skipif(
    CXX is None,
    reason="no C++ compiler on PATH, so the cross-language agreement required "
           "by CFG-CM-10 was NOT verified in this run -- the Python-side golden "
           "tests passing does not establish it",
)


def _load(name):
    with open(os.path.join(GOLDEN, name), encoding="utf-8") as fh:
        return json.load(fh)["vectors"]


def _cxx_string(text):
    """A C++ string literal carrying the exact bytes of a Python str.

    Every byte is emitted as a \\xNN escape rather than as source text. Two
    reasons, both of which have bitten real projects: a UTF-8 sequence pasted
    into source depends on the compiler's input encoding, and a literal
    containing a quote or a backslash needs escaping anyway. Escaping everything
    removes both questions.

    The concatenation with an empty literal after each escape is not cosmetic.
    C++ parses \\x greedily -- "\\xE5" followed by the letter 'a' is read as the
    single hex escape \\xE5a, which overflows a char. Breaking the literal after
    every byte makes that impossible.
    """
    out = []
    for byte in text.encode("utf-8"):
        out.append("\\x%02x" % byte)
    return '"' + '""'.join(out) + '"'


def _emit_value(node, var, lines, counter):
    """Emit constructor calls building `node` into a C++ variable named `var`."""
    if node is None:
        lines.append("  auto %s = Value::Null();" % var)
    elif isinstance(node, bool):
        # Before the int branch, exactly as in the implementations. A
        # transcription that got this wrong would hand C++ an Int and the
        # comparison would fail for a reason that has nothing to do with the
        # header.
        lines.append("  auto %s = Value::Bool(%s);" % (var, "true" if node else "false"))
    elif isinstance(node, int):
        lines.append("  auto %s = Value::Int(%dLL);" % (var, node))
    elif isinstance(node, float):
        # repr() round-trips the double exactly in Python, and the C++ compiler
        # parses it back to the same bits. Passing "%.17g" text would work too;
        # repr is shorter and provably lossless.
        lines.append("  auto %s = Value::Double(%r);" % (var, node))
    elif isinstance(node, str):
        lines.append("  auto %s = Value::Str(%s);" % (var, _cxx_string(node)))
    elif isinstance(node, list):
        lines.append("  auto %s = Value::List();" % var)
        for item in node:
            counter[0] += 1
            child = "v%d" % counter[0]
            _emit_value(item, child, lines, counter)
            lines.append("  %s->Append(%s);" % (var, child))
    elif isinstance(node, dict):
        lines.append("  auto %s = Value::Map();" % var)
        for key, value in node.items():
            counter[0] += 1
            child = "v%d" % counter[0]
            _emit_value(value, child, lines, counter)
            lines.append("  %s->Set(%s, %s);" % (var, _cxx_string(key), child))
    else:
        raise AssertionError("cannot transcribe %r" % (node,))


def _generate_program():
    """A single translation unit that prints one line per vector.

    One program rather than one per vector: compiling is the slow part, and
    thirty compiles would push this test into the territory where someone
    disables it.
    """
    lines = [
        '#include "xbrain/digest/canonical_digest.h"',
        "#include <cstdio>",
        "#include <string>",
        "using hachist::xbrain::digest::Value;",
        "using hachist::xbrain::digest::CommonDigest;",
        "using hachist::xbrain::digest::CanonicalJson;",
        "using hachist::xbrain::digest::FenceSet;",
        "using hachist::xbrain::digest::Polygon;",
        "using hachist::xbrain::digest::Vertex;",
        "using hachist::xbrain::digest::FenceCrc32;",
        "using hachist::xbrain::digest::CanonicalFenceString;",
        "",
        "int main() {",
    ]
    counter = [0]

    for vec in _load("common_digest_vectors.json"):
        counter[0] += 1
        var = "v%d" % counter[0]
        lines.append("  {")
        sub = []
        _emit_value(vec["tree"].get("common", {}), var, sub, counter)
        lines.extend("  " + s for s in sub)
        # Both the canonical string and the digest are printed. When the two
        # languages disagree, knowing whether they disagreed on the bytes or on
        # the sha256 is the whole of the diagnosis.
        lines.append('    std::printf("D\\t%s\\t%s\\t%s\\n", %s, '
                     'CommonDigest(%s).c_str(), CanonicalJson(%s).c_str());'
                     % ("%s", "%s", "%s", _cxx_string(vec["name"]), var, var))
        lines.append("  }")

    for vec in _load("fence_crc32_vectors.json"):
        fence = vec["fence"]
        lines.append("  {")
        lines.append("    FenceSet f;")
        lines.append("    f.fence_set_id = %s;" % _cxx_string(fence["fence_set_id"]))
        lines.append("    f.rev = %dU;" % fence["rev"])
        for poly in fence["polygons"]:
            lines.append("    {")
            lines.append("      Polygon p;")
            lines.append("      p.poly_id = %s;" % _cxx_string(poly["poly_id"]))
            lines.append("      p.role = %s;" % _cxx_string(poly["role"]))
            lines.append("      p.winding = %s;" % _cxx_string(poly["winding"]))
            lines.append("      p.hard_enforce = %s;"
                         % ("true" if poly["hard_enforce"] else "false"))
            lines.append("      p.priority = %d;" % poly["priority"])
            if "speed_limit_mps" in poly:
                lines.append("      p.speed_limit_mps = %r;" % float(poly["speed_limit_mps"]))
            for vertex in poly["vertices"]:
                lines.append("      p.vertices.push_back(Vertex{%r, %r});"
                             % (float(vertex["lat"]), float(vertex["lon"])))
            lines.append("      f.polygons.push_back(p);")
            lines.append("    }")
        lines.append('    std::printf("F\\t%s\\t%s\\t%s\\n", %s, '
                     'FenceCrc32(f).c_str(), CanonicalFenceString(f).c_str());'
                     % ("%s", "%s", "%s", _cxx_string(vec["name"])))
        lines.append("  }")

    lines.append("  return 0;")
    lines.append("}")
    return "\n".join(lines) + "\n"


@pytest.fixture(scope="module")
def cxx_results(tmp_path_factory):
    """Compile once, run once, return {(kind, name): (value, canonical)}."""
    work = tmp_path_factory.mktemp("digest_cxx")
    src = work / "vectors.cc"
    src.write_text(_generate_program(), encoding="utf-8")
    exe = work / "vectors"

    compile_cmd = [CXX] + CXX_FLAGS + ["-I", HEADER_DIR, str(src), "-o", str(exe)]
    proc = subprocess.run(compile_cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        # The generated source is kept and its path reported. A compiler error
        # in generated code is unreadable without the file it refers to.
        pytest.fail("C++ compilation failed (source kept at %s)\n%s"
                    % (src, proc.stderr))

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, "the C++ program crashed:\n" + run.stderr

    results = {}
    for line in run.stdout.strip().split("\n"):
        kind, name, value, canonical = line.split("\t", 3)
        results[(kind, name)] = (value, canonical)
    return results


@pytest.mark.parametrize("vec", _load("common_digest_vectors.json"),
                         ids=lambda v: v["name"])
def test_cxx_common_digest_matches_the_golden_vector(vec, cxx_results):
    """*** The cross-language claim of CFG-CM-10, config side."""
    value, canonical = cxx_results[("D", vec["name"])]
    assert canonical == vec["canonical"], (
        "C++ produced a different canonical string; the disagreement is in the "
        "serialiser, not the hash"
    )
    assert value == vec["expected"]


@pytest.mark.parametrize("vec", _load("fence_crc32_vectors.json"),
                         ids=lambda v: v["name"])
def test_cxx_fence_crc32_matches_the_golden_vector(vec, cxx_results):
    """*** The cross-language claim of CFG-CM-10, fence side."""
    value, canonical = cxx_results[("F", vec["name"])]
    assert canonical == vec["canonical"]
    assert value == vec["expected"]


def test_every_vector_was_actually_run_by_the_cxx_side(cxx_results):
    """Guards the guard.

    The two parametrised tests above look up their vector by name; a generator
    that silently emitted nothing for some vector would make them raise KeyError,
    which is loud. But a generator that emitted nothing at ALL, for both files,
    would leave zero parametrised cases and a green run -- pytest reports no
    tests as no failures. This asserts the count instead.
    """
    expected = (len(_load("common_digest_vectors.json"))
                + len(_load("fence_crc32_vectors.json")))
    assert len(cxx_results) == expected, (
        "the C++ program printed %d results for %d vectors"
        % (len(cxx_results), expected)
    )


def test_the_header_pulls_in_no_ros_and_no_third_party():
    """CLAUDE.md 5.3: common/ must be usable from chassis_relay and rtk_driver.

    Checked by reading the includes rather than by the compile succeeding: the
    compile above would also succeed if ROS happened to be installed on the
    build machine, and would then start failing on a machine where it is not.
    """
    path = os.path.join(HEADER_DIR, "xbrain", "digest", "canonical_digest.h")
    with open(path, encoding="utf-8") as fh:
        includes = [ln.strip() for ln in fh if ln.strip().startswith("#include")]
    allowed = {"<cstdint>", "<cstdio>", "<cstring>", "<algorithm>", "<map>",
               "<memory>", "<string>", "<vector>"}
    for line in includes:
        target = line.split(None, 1)[1].strip()
        assert target in allowed, (
            "%s is outside the standard library subset this header may use; "
            "its consumers include chassis_relay, which may not link ROS" % target
        )


def test_the_header_declares_no_distribution_macros():
    """13 PB-5: the humble/jazzy baseline D-45 is not decided.

    A version macro written now would have to be torn out when it is, and in the
    meantime it would make the header behave differently on the two platforms
    while both claim to pass this suite.
    """
    path = os.path.join(HEADER_DIR, "xbrain", "digest", "canonical_digest.h")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    for banned in ("ROS_DISTRO", "HUMBLE", "JAZZY", "__has_include(<rclcpp"):
        assert banned not in text, "PB-5 forbids distribution detection: " + banned
