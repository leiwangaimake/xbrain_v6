"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_session_config_cxx.py
Brief: Compile and run the gtest replica, then hold it equal to the Python side

Description:
What problem this solves. INF-ZN-1 delivers the session-config table twice, once
in Python for P1 through P5 and once in C++17 for chassis_relay, quadruped and
perception. A .cc file that nothing compiles is not a test, and two tables that
nothing compares are two tables that drift -- so this file does both: it builds
tests/common/zenoh/test_session_config.cc with the flags CLAUDE.md 5.6 mandates,
runs the gtest cases, and then runs the binary's --emit mode and compares the
emitted document field by field against session_config_document().

Why comparing the two matters more than either half passing. Both halves assert
against literals transcribed from 11 S1.1.2 (grep
"RT 面配置与强制约束 RT-C1 ~ RT-C3"). Two independent transcriptions can both be
wrong in the same way only if the same person mistyped twice; they can easily be
wrong in DIFFERENT ways, and that is the case this catches -- a C++ table that
keeps an old endpoint after the Python one moved. On the robot that presents as
chassis_relay connecting to a router nobody else is on, which looks exactly like
a dead publisher.

*** What this does NOT establish. It compares the two DOCUMENTS, not the two
runtimes. Neither half opens a session -- that needs both routers, which is a
separate item -- so nothing here shows that Zenoh accepts either document at
open() time. The Python half gets closer by parsing its document through
zenoh.Config, which at least proves the field paths exist; the C++ half links no
zenoh binding at all, because 11 S2.4.1 still records the version as unlocked
(grep "Zenoh 版本待锁定").

The skips are loud on purpose. A missing compiler or a missing gtest makes this
module skip with a reason that names what went unverified, because a skipped
cross-language check and a passing one are indistinguishable in a summary line --
and this project has already been caught reading one as the other.
"""

import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.zenoh import (PLANE_GEN, PLANE_RT,  # noqa: E402
                                 session_config_document)

#: The C++ source under test and the include root it reads the header from.
SOURCE = os.path.join(ROOT, "tests", "common", "zenoh", "test_session_config.cc")
HEADER_DIR = os.path.join(ROOT, "common", "include")

#: CLAUDE.md 5.6 and 5.2. -std=c++17 exactly and not "at least": 13 CPP-1 fixes
#: the baseline at C++17, and building this as C++20 here would let a C++20-only
#: construct into a header whose consumers cannot compile it.
CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]

#: g++ first, clang++ as the fallback. Either is acceptable -- the header is
#: plain C++17 with no compiler-specific construct, and 13 PB-5 forbids
#: distribution-detection macros precisely so this stays true while the
#: humble/jazzy baseline D-45 is unresolved.
CXX = shutil.which("g++") or shutil.which("clang++")

#: gtest ships as a static library on this platform. Both archives are checked
#: for rather than only the header, because a missing archive fails at link time
#: with a message about undefined references to testing::, which reads as a bug
#: in the test rather than as a missing package.
GTEST_INCLUDE = "/usr/include/gtest"
GTEST_LIB = "/usr/lib/x86_64-linux-gnu/libgtest.a"

pytestmark = pytest.mark.skipif(
    CXX is None or not os.path.isdir(GTEST_INCLUDE) or not os.path.isfile(GTEST_LIB),
    reason="no C++ compiler or no gtest on this host, so the C++ replica of "
           "RT-C1/RT-C2/RT-C3.a/RT-C3.d was NOT compiled or run, and neither "
           "was its agreement with the Python factory; the Python cases passing "
           "does not establish either",
)


# Built once per session, not once per case: the compile dominates the runtime
# and the binary is read-only afterwards. tmp_path_factory rather than a fixed
# path under the tree, so a failed run leaves nothing behind for the next one to
# pick up -- a stale binary that still passes is the worst possible outcome here.
@pytest.fixture(scope="module")
def gtest_binary(tmp_path_factory):
    """Compile the gtest replica and return the path to the binary.

    Failure here is a hard assert rather than a skip. A compile error means the
    header or the replica is broken, which is a red result; skipping on it would
    convert the item's C++ half into a green tick that checked nothing.
    """
    out = str(tmp_path_factory.mktemp("zenoh_cxx") / "test_session_config")
    # Only ONE include path is given, and it is common/include. The replica must
    # reach the header the way a deployed consumer does; adding the source tree
    # here would let it compile against something no consumer can see.
    #
    # libgtest.a and not libgtest_main.a: the replica owns main() so that its
    # --emit mode can run before gtest parses the arguments. Linking gtest_main
    # as well would give two definitions of main and fail at link.
    cmd = ([CXX] + CXX_FLAGS + ["-I", HEADER_DIR, SOURCE, "-o", out,
                                GTEST_LIB, "-lpthread"])
    proc = subprocess.run(cmd, capture_output=True, text=True)
    # The compiler's own output is put in the failure message rather than
    # summarised. A "compilation failed" with no diagnostics sends the reader to
    # reproduce the build by hand, and -Werror means most failures here are one
    # warning that names its own line.
    assert proc.returncode == 0, (
        "compiling the C++ replica failed:\n%s\n%s" % (proc.stdout, proc.stderr))
    return out


def test_gtest_cases_pass(gtest_binary):
    """Every gtest case in the C++ replica passes.

    The binary's own report is put in the failure message rather than a summary,
    because gtest already names the case, the line and the two values. Replacing
    that with "the C++ tests failed" would send the reader to reproduce a run
    that has already been done.
    """
    proc = subprocess.run([gtest_binary], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


# Parametrised over both planes. The RT plane is the one with the isolation
# argument behind it, but a drift on the general plane is just as invisible and
# the case costs nothing to run twice.
@pytest.mark.parametrize("plane", [PLANE_RT, PLANE_GEN])
def test_cxx_emits_the_same_document_as_python(gtest_binary, plane):
    """The C++ table and the Python table describe the same session config.

    This is the only case in either language that can catch the two tables
    drifting. Both halves assert against literals transcribed from 11, so both
    stay green while one of them holds a stale endpoint -- the disagreement is
    only visible by comparing them with each other.
    """
    proc = subprocess.run([gtest_binary, "--emit", plane],
                          capture_output=True, text=True)
    # A non-zero exit means the emit mode itself failed; stderr carries the
    # reason and stdout would be empty, so comparing documents first would
    # report a json parse error instead of the real cause.
    assert proc.returncode == 0, proc.stderr
    # Parsed and compared as data, not as text. A byte comparison would fail on
    # a difference in spacing that changes nothing, and this test would then be
    # relaxed by whoever hit that -- which is how a real comparison turns into a
    # substring check. The document is what both sides must agree on.
    assert json.loads(proc.stdout) == session_config_document(plane)


# The emit mode must refuse an unknown plane rather than default to one. If it
# defaulted, the case above would compare the RT document against the general
# plane's expectation and report a drift that does not exist, sending the reader
# to look for a discrepancy between two tables that agree.
def test_emit_rejects_an_unknown_plane(gtest_binary):
    """--emit with a plane that is not rt or gen exits non-zero.

    "general" and not a nonsense string: it is the word the contract uses for
    that plane in prose, so it is what someone editing this driver would
    plausibly type in place of the key name gen.
    """
    proc = subprocess.run([gtest_binary, "--emit", "general"],
                          capture_output=True, text=True)
    assert proc.returncode != 0
    # And nothing on stdout. An exit code alone would still allow a build that
    # printed a document AND failed, which the case above would then compare
    # against the wrong plane.
    assert proc.stdout == ""
