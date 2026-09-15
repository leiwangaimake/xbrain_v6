"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_units_cxx.py
Brief: Drives the C++ units probe, and the negative case that must NOT compile

Description:
tests/common/test_units.py covers the Python dimensional types. This covers the
C++ ones, which had no test at all until Tier 1 (13 S3.2) became their first
consumer. The two halves protect the same mistake through different mechanisms,
and only one of them can be asserted from inside a running program:

  * Python's guard is a runtime isinstance check, so a cross-unit comparison
    raises and a test can catch it;
  * C++'s guard is the ABSENCE of an overload. There is nothing to catch -- the
    translation unit simply does not build. Asserting it therefore means
    compiling a program that is EXPECTED TO FAIL and requiring that it does.

That negative case is the one worth writing carefully. A positive-only suite
passes just as happily against a header that defines a templated Clamp
accepting any two arguments, which is precisely the header this one is not.

Why a compiler probe and not a unit test in the quadruped package: units.h lives
in common/ and its consumers include chassis_relay, which is on the emergency
stop path and links no test framework at all. The probe is a plain program with
its own main, so it runs identically here, from cxx_mutants.py, and by hand.
"""

import os
import shutil
import subprocess

import pytest

pytestmark = pytest.mark.no_device

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
INCLUDE = os.path.join(ROOT, "common", "include")
PROBE = os.path.join(ROOT, "tests", "common", "units_cxx", "units_probe.cc")

#: g++ or clang++, whichever the machine has. None means neither, and the cases
#: below skip rather than pass -- a skipped test reads as "not checked here",
#: while a pass on a machine with no compiler would be a lie.
CXX = shutil.which("g++") or shutil.which("clang++")

#: The cross-unit call that must not compile: a yaw rate held to a LINEAR limit.
#: Written out rather than generated, so a reader can see exactly what shape of
#: mistake is being refused. It is wrong by whatever the two numbers happen to
#: be, and it reads as a perfectly ordinary line.
NEGATIVE_SRC = """
#include "xbrain/units/units.h"
int main() {
  return static_cast<int>(xbrain::units::Clamp(xbrain::units::Radps{1.0},
                                               xbrain::units::Mps{2.0}).value);
}
"""


def _compile(tmp_path, source_path, out_name, extra=()):
    """Compile one file with the project's C++ settings. Returns CompletedProcess."""
    out = os.path.join(str(tmp_path), out_name)
    cmd = [CXX, "-std=c++17", "-I", INCLUDE, "-o", out, source_path] + list(extra)
    return subprocess.run(cmd, capture_output=True, text=True, timeout=120), out


@pytest.mark.skipif(CXX is None, reason="no C++ compiler on this machine")
def test_the_probe_compiles_clean_and_passes(tmp_path):
    """Positive half: the probe builds with -Werror and every case passes.

    -Werror is part of the assertion, not decoration: 13 S5.6 requires it for
    the whole package, and a header that produces a warning here would produce
    a build failure in quadruped.
    """
    proc, exe = _compile(tmp_path, PROBE, "units_probe",
                         extra=("-Wall", "-Wextra", "-Wpedantic", "-Werror"))
    assert proc.returncode == 0, (
        "units_probe.cc failed to build:\n" + proc.stderr)
    run = subprocess.run([exe], capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "ALL UNITS_CXX TESTS PASSED" in run.stdout


@pytest.mark.skipif(CXX is None, reason="no C++ compiler on this machine")
def test_a_cross_unit_clamp_does_not_compile(tmp_path):
    """Negative half: the guarantee IS the compile failure.

    This is the case that separates the real header from one with a templated
    Clamp. Both pass every positive case; only the real one rejects this.

    The assertion is on the returncode AND on the message naming Clamp: a file
    that failed to compile for an unrelated reason (a typo, a missing include)
    would otherwise be read as the guarantee holding.
    """
    src = os.path.join(str(tmp_path), "cross_unit.cc")
    with open(src, "w", encoding="utf-8") as f:
        f.write(NEGATIVE_SRC)
    proc, _ = _compile(tmp_path, src, "cross_unit")
    assert proc.returncode != 0, (
        "a Radps clamped against an Mps limit COMPILED -- the dimensional "
        "guarantee of units.h is gone, and a yaw rate held to a linear limit "
        "is now an ordinary-looking line of code (CFG-CM-18)")
    assert "Clamp" in proc.stderr, (
        "the compile failed for some other reason, so this case proves "
        "nothing about Clamp:\n" + proc.stderr)
