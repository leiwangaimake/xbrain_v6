"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_qos_profiles_cxx.py
Brief: Compile the C++ QoS table, run its cases, and hold it equal to Python

Description:
What problem this solves. INF-ZN-3 delivers the 11 S2.4.2 frozen profile table
twice: in Python for P1 through P5, and in C++17 for chassis_relay, perception
and quadruped. A .cc file nothing compiles is not a test, and two tables nothing
compares are two tables that drift -- so this module does both. It builds
tests/common/zenoh/test_qos_profiles.cc with the flags CLAUDE.md 5.6 mandates,
runs the gtest cases, then runs --emit and compares the emitted table field by
field against FROZEN_PROFILES and RT_OVERRIDE.

Why the comparison matters more than either half passing. Both halves assert
against literals transcribed from 11 S2.4.2 and S2.4.7. Two transcriptions can
easily be wrong in DIFFERENT ways -- a C++ table left on an old depth after the
Python one moved -- and that is the case this catches. On the robot it presents
as one process publishing with QoS the rest of the system does not expect, which
looks like nothing at all until a queue fills.

The TODO's own conflict register asks for exactly this: it records that
CPP-CXX-4 and INF-ZN-3 both claim "the QoS profile table exported from one
constant table", and resolves it by requiring a metatest that the two languages'
exported profile sets have an empty bidirectional difference. That assertion is
test_profile_names_match_bidirectionally below.

*** What this does NOT establish. It compares two TABLES, not two runtimes.
Neither half sets a QoS knob on a real publisher; 11 S2.4.1 still records the
Zenoh version as unlocked, and QoS-T1 to QoS-T8 in S2.4.9 are pending T7. The
skips are loud for the same reason the session-config module's are: a skipped
cross-language check and a passing one are indistinguishable in a summary line,
and this project has been caught reading one as the other.
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

from xbrain.common.config import MISSING  # noqa: E402
from xbrain.common.zenoh.qos import FROZEN_PROFILES, RT_OVERRIDE  # noqa: E402

#: The C++ source under test and the include root it reads the header from.
SOURCE = os.path.join(ROOT, "tests", "common", "zenoh", "test_qos_profiles.cc")
HEADER_DIR = os.path.join(ROOT, "common", "include")

#: CLAUDE.md 5.6 and 5.2. -std=c++17 exactly and not "at least": 13 CPP-1 fixes
#: the baseline, and building this as C++20 here would let a C++20-only
#: construct into a header whose consumers cannot compile it.
CXX_FLAGS = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-O1"]

#: g++ first, clang++ as the fallback. Either is acceptable: the header is plain
#: C++17 with no compiler-specific construct, and 13 PB-5 forbids
#: distribution-detection macros precisely so that stays true while the
#: humble/jazzy baseline D-45 is unresolved.
CXX = shutil.which("g++") or shutil.which("clang++")

#: gtest ships as a static library on this platform. Both the headers and the
#: archive are checked for rather than only the headers, because a missing
#: archive fails at link time with a message about undefined references to
#: testing::, which reads as a bug in the test rather than as a missing package.
GTEST_INCLUDE = "/usr/include/gtest"
GTEST_LIB = "/usr/lib/x86_64-linux-gnu/libgtest.a"

pytestmark = pytest.mark.skipif(
    CXX is None or not os.path.isdir(GTEST_INCLUDE)
    or not os.path.isfile(GTEST_LIB),
    reason="no C++ compiler or no gtest on this host, so the C++ half of the "
           "11 S2.4.2 frozen table was NOT compiled or run, and neither was its "
           "agreement with the Python table; the Python cases passing does not "
           "establish either",
)


# Built once per module, not once per case: the compile dominates the runtime and
# the binary is read-only afterwards. tmp_path_factory rather than a fixed path
# under the tree, so a failed run leaves nothing behind for the next one to pick
# up -- a stale binary that still passes is the worst outcome available here,
# because it reports agreement between the Python table and a C++ table that no
# longer exists.
@pytest.fixture(scope="module")
def gtest_binary(tmp_path_factory):
    """Compile the C++ table replica and return the path to the binary.

    A compile failure is a hard assert and not a skip: it means the header or
    the replica is broken, which is a red result. Skipping on it would turn the
    item's C++ half into a green tick that checked nothing.
    """
    out = str(tmp_path_factory.mktemp("qos_cxx") / "test_qos_profiles")
    # libgtest.a and not libgtest_main.a: the replica owns main() so that its
    # --emit mode can run before gtest parses the argument list. Linking
    # gtest_main as well would give two definitions of main and fail at link,
    # with a message that says nothing about why the replica has its own.
    # Only common/include is on the include path, so the replica reaches the
    # header the way a deployed consumer does. Adding the source tree would let
    # it compile against something no consumer can see.
    cmd = ([CXX] + CXX_FLAGS + ["-I", HEADER_DIR, SOURCE, "-o", out,
                                GTEST_LIB, "-lpthread"])
    proc = subprocess.run(cmd, capture_output=True, text=True)
    # The compiler's diagnostics go into the failure message rather than a
    # summary: -Werror means most failures here are one warning that names its
    # own line, and hiding it sends the reader to rebuild by hand.
    assert proc.returncode == 0, (
        "compiling the C++ QoS table failed:\n%s\n%s" % (proc.stdout,
                                                         proc.stderr))
    return out


def test_gtest_cases_pass(gtest_binary):
    """Every gtest case in the C++ replica passes.

    The binary's own report goes into the failure message rather than a summary,
    because gtest already names the case, the line and the two values. Replacing
    that with "the C++ tests failed" would send the reader to reproduce a run
    that has already been done.

    This case and the comparison cases below check different things and neither
    subsumes the other: gtest holds the C++ table to literals transcribed from
    11, and the comparisons hold it to the Python table. A drift that hit both
    transcriptions identically would pass the first and fail nothing -- which is
    why the transcription cases in test_qos_resolve.py grep the contract.
    """
    proc = subprocess.run([gtest_binary], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


@pytest.fixture(scope="module")
def emitted(gtest_binary):
    """The C++ table, parsed.

    Parsed as data rather than compared as text. A byte comparison would fail on
    a difference in spacing that changes nothing, and this module would then be
    relaxed by whoever hit that -- which is how a real comparison turns into a
    substring check.

    A non-zero exit is asserted before the parse, so a failure of --emit itself
    is reported as what it is instead of as a json syntax error on empty input.
    """
    proc = subprocess.run([gtest_binary, "--emit"], capture_output=True,
                          text=True)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


# The metatest the TODO's conflict register asks for by name. Its wording is
# "两语言导出的档位集合双向差集为空", and this is that assertion -- with the order
# added, since both tables are written in the order 11 S2.4.2 lists them.
def test_profile_names_match_bidirectionally(emitted):
    """The two languages export the same set of profile names, and in order.

    This is the first case to run against the emitted table, and deliberately so:
    if the two tables disagree about which profiles exist, every per-profile
    comparison below would fail with a KeyError naming one profile, and the
    reader would chase that instead of the missing or extra row.


    Bidirectional, because the two failures are different: a profile only C++
    knows about is one a Python consumer cannot select, and a profile only Python
    knows about is one a C++ consumer will look up and get nullptr for. Order is
    asserted as well, since both tables are written in the order 11 S2.4.2 lists
    them and a reordering usually means someone rewrote one of them from memory.
    """
    # list() and not set(): equality of lists carries both the membership and
    # the order. Comparing sets would leave the order unasserted, and a table
    # rewritten from memory usually keeps the members and loses the order.
    assert list(emitted["profiles"]) == list(FROZEN_PROFILES)


# Parametrised over the PYTHON table's names, after the bidirectional check above
# has already established the two sets are equal. Driving it from the C++ side
# instead would make a C++ table that had lost a profile produce fewer cases
# rather than a failure, and fewer green cases look exactly like more green cases
# in a summary line.
@pytest.mark.parametrize("name", sorted(FROZEN_PROFILES))
def test_each_profile_matches_python(emitted, name):
    """Every knob of every profile is the same on both sides.

    Parametrised by name so a failure says which profile drifted. One case
    looping over the table would stop at the first and leave the reader unsure
    whether the rest also moved.
    """
    got = emitted["profiles"][name]
    want = FROZEN_PROFILES[name]
    assert got["congestion_control"] == want.congestion_control
    assert got["priority"] == want.priority
    assert got["reliability"] == want.reliability
    # Identity against the bool: json.loads gives real booleans, and a C++ side
    # that emitted 1 instead of true would compare equal under == while being a
    # different json document.
    assert got["express"] is want.express
    assert got["handler"]["kind"] == want.handler.kind
    if want.handler.depth is MISSING:
        # The sentinel crosses the language boundary as null. The C++ side holds
        # it as 0 -- the spelling 11 S2.4.7 uses -- and must not emit that 0,
        # because a reader of the dump would take it for a queue size.
        assert got["handler"]["depth"] is None
    else:
        assert got["handler"]["depth"] == want.handler.depth


# rt_override is compared separately from the profiles because it is not one.
# It has three fields where a profile has five, it is hard-coded on both sides
# where profiles can be supplied by deployment, and no configuration can correct
# it -- so a drift here has nothing downstream that would notice.
def test_rt_override_matches_python(emitted):
    """QOS-C1's hard-coded override is identical in both languages.

    This is the constant no configuration can correct -- 11 S2.4.7 says it is
    硬编码在实现中 -- so a drift between the two implementations has nothing
    downstream that would notice it.
    """
    # Field by field rather than as one dict comparison. The C++ side emits its
    # handler depth as a number and the Python side holds an int, so a whole-dict
    # comparison would also be asserting that the emitter's json shape matches
    # some expectation written here -- two claims in one assertion, and the
    # uninteresting one fails first.
    got = emitted["rt_override"]
    assert got["congestion_control"] == RT_OVERRIDE.congestion_control
    assert got["priority"] == RT_OVERRIDE.priority
    assert got["handler"]["kind"] == RT_OVERRIDE.handler.kind
    assert got["handler"]["depth"] == RT_OVERRIDE.handler.depth
