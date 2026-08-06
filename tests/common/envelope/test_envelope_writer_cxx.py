"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_envelope_writer_cxx.py
Brief: Compile and run the CPP-CXX-2 envelope-writer gtest, and scan for a
       second envelope seq source (the third mutation)

Description:
CPP-CXX-2 has three mutations. Two are behavioural (ts_sync false before any
ClockStatus; staleness on the monotonic clock, not the wall clock) and live in
test_envelope_writer.cc, compiled and run here with the CLAUDE.md 5.6 flags. The
third is structural -- "there must be exactly one place that stamps an envelope
and increments seq" -- and is a source scan: a second hand-rolled seq++ in the
C++ tree is what this file's scan test reports.

A compile failure is a hard assert, not a skip: a header that no longer compiles
is a red result, and skipping on it would turn the C++ half into a green tick
that checked nothing. The gtest-absent case IS a skip, and a loud one, because a
skipped cross-language check and a passing one look identical in a summary line.
"""

import os
import re
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

#: The C++ source under test and the include root it reads the header from --
#: only common/include, so the test compiles the header the way a deployed
#: consumer does (13 CPP-1 / CLAUDE.md 5.2, exactly C++17).
SOURCE = os.path.join(ROOT, "tests", "common", "envelope", "test_envelope_writer.cc")
HEADER_DIR = os.path.join(ROOT, "common", "include")

CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]
CXX = shutil.which("g++") or shutil.which("clang++")
GTEST_INCLUDE = "/usr/include/gtest"
GTEST_LIB = "/usr/lib/x86_64-linux-gnu/libgtest.a"

_cxx_missing = (CXX is None or not os.path.isdir(GTEST_INCLUDE)
                or not os.path.isfile(GTEST_LIB))


@pytest.fixture(scope="module")
def gtest_binary(tmp_path_factory):
    """Compile the writer gtest once and return the binary path."""
    if _cxx_missing:
        pytest.skip("no C++ compiler or no gtest on this host, so the C++ half of "
                    "CPP-CXX-2 (ts_sync false-before-status and the monotonic 5 s "
                    "window) was NOT compiled or run")
    out = str(tmp_path_factory.mktemp("env_cxx") / "test_envelope_writer")
    cmd = [CXX] + CXX_FLAGS + ["-I", HEADER_DIR, SOURCE, "-o", out,
                               GTEST_LIB, "-lpthread"]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    assert proc.returncode == 0, (
        "compiling the envelope writer test failed:\n%s\n%s"
        % (proc.stdout, proc.stderr))
    return out


def test_gtest_cases_pass(gtest_binary):
    """Every gtest case passes (mutations 1 and 2, and the field placement)."""
    proc = subprocess.run([gtest_binary], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


# --------------------------------------------------------------------------
# Mutation 3: exactly one envelope seq source in the C++ tree.
# --------------------------------------------------------------------------

#: Where a C++ envelope could be hand-rolled: the shared C++ library and the
#: robot-side C++ processes (99 U79 / U80). Scanned as text because the point is
#: to catch a SECOND seq++ before it is ever wired up -- a compile-time or
#: runtime check would only see it once it is already in use.
_CXX_ROOTS = [os.path.join(ROOT, "common"), os.path.join(ROOT, "ros2_ws")]

#: A seq increment: ++seq / seq++ (optionally with a trailing underscore field
#: name). Reads of .seq are not increments and must not match, so the pattern
#: requires the ++ operator adjacent to a seq identifier.
_SEQ_INC = re.compile(r"(\+\+\s*seq\w*)|(\bseq\w*\s*\+\+)")


def _cxx_files():
    """Every .h / .cc under the C++ roots that exist (ros2_ws/ may be empty)."""
    out = []
    for root in _CXX_ROOTS:
        if not os.path.isdir(root):
            continue
        for dirpath, _dirs, files in os.walk(root):
            if "build" in dirpath:
                continue                                # skip build products
            for f in files:
                if f.endswith((".h", ".cc", ".cpp", ".hpp")):
                    out.append(os.path.join(dirpath, f))
    return out


def test_exactly_one_envelope_seq_source_in_cxx():
    """*** Mutation 3: a second hand-rolled envelope seq++ must be reported.

    Today the only C++ seq increment is in envelope_writer.h. A second RT process
    that stamps its own envelope with its own seq++ -- instead of using
    EnvelopeWriter -- creates a second sequence space the consumer's gap detection
    cannot reconcile, and this scan names it. Reported as the set of files, so
    adding the second one turns the assertion red with the offender named.
    """
    offenders = []
    for path in _cxx_files():
        with open(path, encoding="utf-8", errors="replace") as fh:
            for lineno, line in enumerate(fh, 1):
                if _SEQ_INC.search(line):
                    offenders.append("%s:%d" % (os.path.relpath(path, ROOT), lineno))
    # The one legitimate source. Any others are the mutation.
    legit = "common/include/xbrain/envelope/envelope_writer.h"
    extra = [o for o in offenders if not o.startswith(legit)]
    assert not extra, (
        "a second C++ envelope seq++ exists outside the single EnvelopeWriter "
        "(CPP-CXX-2 mutation 3): %s" % extra
    )
    # And the legitimate one must still be there -- a scan that matched nothing
    # would pass vacuously, so assert the known source is seen.
    assert any(o.startswith(legit) for o in offenders), (
        "the EnvelopeWriter seq++ was not found; the scan pattern or the file "
        "moved, and this test would now pass without checking anything"
    )
