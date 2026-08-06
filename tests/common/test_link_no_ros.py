"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_link_no_ros.py
Brief: Drives the CFG-CM-14 build gate -- common/ builds and links with no ROS,
       at exactly C++17, with the four warning flags and live debug assertions

Description:
What problem this solves. 19 S1.2 forbids anything under common/ from depending
on rclcpp or any ROS type, and states the reason rather than a preference: the
consumers include chassis_relay, which 11 S1.1.6 places on the emergency-stop
path under CRL-4 and CRL-5, and a foundation library that drags in rclcpp is
simply unusable there. 13 S9.3 CPP-1 fixes the language at exactly C++17 with
CXX_STANDARD_REQUIRED ON and CXX_EXTENSIONS OFF, and 19 S1.1 adds
-Wall -Wextra -Werror -Wpedantic plus a debug build that supports
-D_GLIBCXX_ASSERTIONS. Until this item there was no CMakeLists.txt anywhere in
the tree, so every one of those was a sentence with no build behind it.

How the claims are evaluated, and why not by reading the CMake text. Grepping
xbrain/common/CMakeLists.txt for "CXX_EXTENSIONS OFF" would test the text and
not the build -- the "定义式冒充实测结论" shape in CLAUDE.md S3.2, where the
conclusion is defined into the premise. Instead the build is run and three
independent artefacts of it are read back:

  * build_facts.txt, written by CMake at generate time from the RESOLVED target
    properties. This is where CXX_STANDARD_REQUIRED is read, since its documented
    effect -- fail rather than decay to an older standard -- cannot be triggered
    on a compiler that supports C++17 anyway.
  * compile_commands.json, the command the compiler is actually invoked with.
    This is where -std=c++17 (not gnu++17, not c++20) and the four warning flags
    are checked.
  * the produced binary, through ldd and through running it.

Each of those can be wrong in a way the others cannot see, which is why all
three are read rather than whichever one is cheapest.

The mutations, and what each one is worth. CLAUDE.md S3.3 is explicit that an
assertion which has never been red has not been written, so every claim above
has a mutation below and each was run:

  * a rclcpp include in a common/ header -- the criterion's mutation 1.
  * a concepts declaration -- the criterion's mutation 2. This one carries the
    standard-level claim, and it is backed by a discrimination test proving the
    snippet compiles at C++20, so its failure is attributable to the standard
    and not to a malformed snippet.
  * a std::format include -- the criterion's other spelling of mutation 2. It is
    run, and it does go red, but on a libstdc++ that predates <format> it goes
    red because the header is absent from the library, which is true at C++20 as
    well. On such a toolchain it is NOT evidence about the standard setting, and
    treating it as evidence would be the "永远红的断言" shape in CLAUDE.md S3.2.
    The concepts mutation is the one this suite relies on.
  * a typeof declaration -- not named by the criterion. It is the only
    behavioural discriminator between -std=c++17 and -std=gnu++17, so without it
    CXX_EXTENSIONS OFF would rest on the flag text alone.
  * deleting the baseline call from the test project -- proves the standard and
    the flag assertions are not vacuous, and demonstrates the trap the baseline
    function exists for: the CMAKE_CXX_* variables set inside xbrain/common/ do
    not cross the add_subdirectory boundary, and the build still SUCCEEDS with
    the compiler default of gnu++17.
  * narrowing the header glob to one subtree, and emptying it entirely -- the
    scan surface of this gate is the set of headers the aggregate pulls in, and
    a glob that matches too little loses headers with nothing failing.

*** What this suite cannot establish, stated so a pass is not read as more than
it is. The rclcpp mutation goes red on this machine at the PREPROCESSING stage,
with "no such file or directory", because the build deliberately puts no ROS
include directory on the command line. On a machine where a ROS prefix is
already on the compiler's search path, an include with no reference to any
rclcpp symbol could preprocess and still link, and the mutation would not be
red. The claim that no ROS reaches this target therefore rests on the two
positive assertions -- no ROS directory in the resolved include list or the
compile command, and no ROS shared object in ldd -- with the mutation as
corroboration rather than as the whole proof.

If cmake or a C++ compiler is missing the module skips, and it skips LOUDLY:
the reason names exactly which claim went unverified, because a silent skip
turns the central assertion of this item into an empty green tick.
"""

import json           # compile_commands.json, the compiler's own record
import os             # path work and the independent header walk
import re             # parsing include lines out of C++ sources
import shutil         # which(), and staging the mutation sandbox
import subprocess     # cmake, the compiler, ldd and the built binary

import pytest

# Three levels up from tests/common/: the repository root. Derived rather than
# written out, so a checkout somewhere other than /opt/xbrain_v6 still resolves.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: The test project, the build baseline it consumes, and the deployed headers.
#: Held as three separate paths because the mutation sandbox has to reproduce
#: their RELATIVE layout: xbrain/common/CMakeLists.txt finds the headers through
#: ../../common/include, so a sandbox that flattened the tree would silently
#: resolve back to the real headers and every mutation would come out green.
LINK_NO_ROS_DIR = os.path.join(ROOT, "tests", "common", "link_no_ros")   # the TU
BASELINE_CMAKE = os.path.join(ROOT, "xbrain", "common", "CMakeLists.txt")  # source
DEPLOY_INCLUDE = os.path.join(ROOT, "common", "include")   # deployment artefacts

CMAKE = shutil.which("cmake")                    # absent on a bare CI container
CXX = shutil.which("g++") or shutil.which("clang++")   # either is acceptable

pytestmark = pytest.mark.skipif(
    CMAKE is None or CXX is None,
    reason="cmake or a C++ compiler is missing, so the CFG-CM-14 gate was NOT "
           "verified in this run: neither 'common/ builds and links without any "
           "ROS library' (19 S1.2, 11 S1.1.6 CRL-4/CRL-5) nor 'the language is "
           "exactly C++17' (13 S9.3 CPP-1) was evaluated",
)

#: Substrings that betray a ROS dependency, in a path or in a library name.
#: Deliberately broader than rclcpp: the middleware, the IDL runtime and the
#: ament index would each be just as fatal for chassis_relay, and 19 S11 PMT-1
#: names sensor_msgs rather than rclcpp as its mutation.
ROS_MARKERS = ("rclcpp", "rclpy", "/opt/ros", "ament", "rosidl", "rmw",
               # the middleware and IDL runtimes: each one on its own would
               # already make the library unusable from chassis_relay
               "sensor_msgs", "std_msgs", "fastrtps", "cyclonedds", "rcutils")

#: Proves ldd itself produced a real answer. Without this a statically linked
#: binary, or an ldd that failed and printed nothing, would satisfy "no ROS
#: library" trivially -- an assertion that a do-nothing implementation passes.
LDD_SANITY = "libstdc++"


# ---------------------------------------------------------------------------
# Running the build.
# ---------------------------------------------------------------------------

def _cmake_build(source_dir, build_dir, build_type):
    """Configure and build; return (ok, combined output).

    Configure and build output are concatenated because a mutation can fail at
    either stage -- the rclcpp one fails while compiling, the empty-glob one
    fails while configuring -- and a caller that only kept one of them would
    report a red with no reason attached. Several assertions below check the
    output for the word that explains WHY the build failed, and they can only do
    that if both halves survive.

    The build type is always passed explicitly. Leaving it unset gives an empty
    CMAKE_BUILD_TYPE, which matches neither $<CONFIG:Debug> nor $<CONFIG:Release>
    and would make the _GLIBCXX_ASSERTIONS assertions test nothing at all.
    """
    # Out-of-source, always: -B pointing into a pytest temporary directory keeps
    # the repository free of build products, which matters here because the
    # sandbox copies below would otherwise pick up a stale build tree.
    configure = subprocess.run(
        [CMAKE, "-S", source_dir, "-B", build_dir,      # source tree, build tree
         "-DCMAKE_BUILD_TYPE=" + build_type],           # never left empty
        capture_output=True, text=True)                 # captured, not inherited
    # Returned early rather than falling through, because a failed configure
    # leaves no build system and "cmake --build" would then report its own,
    # entirely uninformative, error on top of the real one.
    if configure.returncode != 0:
        return False, configure.stdout + configure.stderr

    build = subprocess.run([CMAKE, "--build", build_dir],
                           capture_output=True, text=True)
    return (build.returncode == 0,
            configure.stdout + configure.stderr + build.stdout + build.stderr)


def _read_facts(build_dir):
    """The resolved target properties CMake wrote at generate time.

    Written by file(GENERATE), so these are the values CMake settled on for the
    target and not an echo of what the CMakeLists asked for. That distinction is
    the whole reason this file exists rather than a grep of the build script.

    Note it is produced at GENERATE time, which means it exists even when the
    compile afterwards fails -- the mutation that removes the baseline call reads
    it after a build that succeeded, and an empty-glob mutation never gets here
    at all because configure itself stops.
    """
    facts = {}                                       # key -> resolved value
    with open(os.path.join(build_dir, "build_facts.txt"), encoding="utf-8") as fh:
        for line in fh:                              # one property per line
            # split with maxsplit=1: a property value can itself contain "=",
            # for instance a -D flag, and splitting on every one would silently
            # truncate exactly the values this file is here to inspect.
            if "=" in line:
                key, value = line.rstrip("\n").split("=", 1)
                facts[key] = value
    return facts


def _compile_command(build_dir):
    """The command line for main.cc, from compile_commands.json.

    Exactly one entry is expected and asserted. Taking entries[0] instead would
    make every flag assertion below vacuous the day the database came out empty:
    an IndexError is loud, but a database with a single unrelated entry would be
    silent, and this project has been bitten by criteria that pass because their
    scan surface collapsed to nothing.
    """
    with open(os.path.join(build_dir, "compile_commands.json"), encoding="utf-8") as fh:
        entries = json.load(fh)
    assert len(entries) == 1, (
        "expected exactly one compiled translation unit, found %d" % len(entries))
    return entries[0]["command"]


@pytest.fixture(scope="module")
def release_build(tmp_path_factory):
    """The default build: no debug assertions, optimisation on.

    Module scoped because configuring and compiling is the slow part and eight
    tests read the same result. A function-scoped fixture would multiply the
    runtime by eight for no extra coverage, and a slow suite is a suite someone
    eventually stops running.

    The assertion here is deliberately inside the fixture. If the build breaks,
    every test that depends on it errors with this message attached, which is far
    easier to read than eight separate failures about a missing executable.
    """
    build_dir = str(tmp_path_factory.mktemp("link_no_ros_release"))
    ok, output = _cmake_build(LINK_NO_ROS_DIR, build_dir, "Release")
    assert ok, "the no-ROS target failed to build:\n" + output
    return build_dir


@pytest.fixture(scope="module")
def debug_build(tmp_path_factory):
    """The debug build, which is where _GLIBCXX_ASSERTIONS is expected.

    Kept as a separate build tree rather than reconfiguring the release one.
    Reconfiguring in place would leave object files compiled under the previous
    definitions, and the probe would then be testing whichever build happened to
    run last -- a result that changes with test ordering is not a result.
    """
    build_dir = str(tmp_path_factory.mktemp("link_no_ros_debug"))
    ok, output = _cmake_build(LINK_NO_ROS_DIR, build_dir, "Debug")
    assert ok, "the debug build of the no-ROS target failed:\n" + output
    return build_dir


# ---------------------------------------------------------------------------
# The positive half: the target builds, runs, and carries nothing from ROS.
# ---------------------------------------------------------------------------

def test_the_no_ros_target_builds_and_actually_runs(release_build):
    """*** The central claim of CFG-CM-14, positive half.

    Running it matters as much as building it. A translation unit that includes
    the headers and does nothing links cleanly whatever the include directory
    holds, so the token below is what separates "it built" from "the code in
    common/ was really instantiated and executed".

    Confirmed by mutation: deleting only the token print, leaving every call in
    place, makes this the single failing test -- so the assertion is attached to
    the program's behaviour and not to the build having happened.
    """
    exe = os.path.join(release_build, "link_no_ros")   # named by the CMake target
    run = subprocess.run([exe], capture_output=True, text=True)   # no arguments
    # stderr is what carries a libstdc++ abort message, so it is the half worth
    # reporting when this fails.
    assert run.returncode == 0, "the linked program failed:\n" + run.stderr
    assert "link_no_ros ok" in run.stdout
    # Both fingerprints must be present AND well formed. Checking only that the
    # token appeared would pass an implementation whose digest returned the empty
    # string, which is exactly the failure a header included but not really
    # compiled would produce. The widths are the ones the two functions promise:
    # sixteen hexadecimal characters for the config digest, eight for the crc32.
    # Their VALUES are not checked here -- that is CFG-CM-10's golden vector
    # suite, and a second copy of those vectors would be a second thing to keep
    # in step.
    assert re.search(r"^common_digest [0-9a-f]{16}$", run.stdout, re.M)
    assert re.search(r"^fence_crc32 [0-9a-f]{8}$", run.stdout, re.M)


def test_the_linked_binary_pulls_in_no_ros_shared_object(release_build):
    """19 S1.2: chassis_relay must be able to link this, and it links no ROS.

    This is the assertion that survives a friendlier machine. The rclcpp mutation
    fails here because no ROS include directory is on the command line, which is
    a property of how the build is configured; this one looks at the artefact
    that would actually be deployed and asks what it needs at run time.
    """
    exe = os.path.join(release_build, "link_no_ros")   # the release artefact
    ldd = subprocess.run(["ldd", exe], capture_output=True, text=True)   # DT_NEEDED
    # The sanity check comes first on purpose. "No ROS library in this output" is
    # trivially true of empty output, so without a positive marker the assertion
    # below would pass for a binary ldd could not read at all.
    assert LDD_SANITY in ldd.stdout, (
        "ldd produced no usable output, so 'no ROS library' would be true for "
        "the wrong reason:\n" + ldd.stdout + ldd.stderr)
    # Every marker, not just rclcpp: 19 S11 PMT-1 states the mutation in terms of
    # sensor_msgs, and a binary that pulled in only the middleware or only the
    # IDL runtime would be just as unusable from chassis_relay.
    for marker in ROS_MARKERS:
        assert marker not in ldd.stdout, (
            "the binary links %s; 11 S1.1.6 CRL-4 puts chassis_relay on the "
            "emergency-stop path and it cannot carry ROS" % marker)


def test_no_ros_directory_reaches_the_compiler(release_build):
    """The other half of "no ROS": nothing on the include path either.

    Checked on the resolved INCLUDE_DIRECTORIES property AND on the compiler
    command line. The property alone would miss a flag injected through
    CMAKE_CXX_FLAGS; the command line alone would miss a directory that CMake
    resolved but the compiler had not reached yet.

    Why this pair matters more than the rclcpp mutation. A ROS prefix arriving on
    the include path is how "common/ has no ROS" stops being true without anybody
    editing a header -- one find_package in a consumer, one environment variable
    from a sourced setup script. The mutation catches the header edit; these two
    catch the build drifting underneath it.
    """
    facts = _read_facts(release_build)      # the resolved property view
    # Split on the separator the CMake JOIN used; empties dropped so a trailing
    # separator does not become a phantom directory.
    include_dirs = [d for d in facts["include_directories"].split("|") if d]
    # Exactly two: the deployed headers, and the build directory holding the
    # generated aggregate. Asserting the exact set rather than "no ROS in it"
    # because an extra directory is how a ROS prefix would arrive, and a
    # membership test would have to guess every name it could carry.
    assert os.path.realpath(DEPLOY_INCLUDE) in include_dirs
    assert len(include_dirs) == 2, (
        "unexpected include directories on the target: %r" % (include_dirs,))
    generated = [d for d in include_dirs if d != os.path.realpath(DEPLOY_INCLUDE)]
    # Inside the build tree, so it cannot be a system or vendor prefix that
    # happened to be added while still leaving the count at two.
    assert generated[0].startswith(release_build), (
        "the second include directory is not inside the build tree: %r" % generated)

    command = _compile_command(release_build)     # now the command-line view
    for marker in ROS_MARKERS:
        assert marker not in command, (
            "the compile command reaches into %s" % marker)


def test_the_target_resolves_to_exactly_cxx17(release_build):
    """13 S9.3 CPP-1: exactly C++17, required, extensions off.

    CXX_STANDARD_REQUIRED is read from the resolved property rather than from the
    command line, because its documented effect -- fail instead of decaying to an
    older standard -- cannot be provoked on a compiler that supports C++17.

    It turned out to do something else here that is worth stating, since it was
    measured rather than assumed. Setting it OFF on cmake 3.22 with GCC 11 makes
    CMake emit NO -std flag at all, on the grounds that the compiler default
    already covers C++17. The default is gnu++17, so extensions come back on and
    the CXX_EXTENSIONS OFF setting is quietly defeated. That is why the mutation
    which flips this property red also turns the GNU-extension mutation red.
    """
    facts = _read_facts(release_build)
    # Equality, not "at least 17". CPP-1 says exactly, and a >= test would accept
    # the CXX_STANDARD 20 mutation that this suite has to reject.
    assert facts["cxx_standard"] == "17"
    # CMake prints booleans in several spellings depending on how they were set,
    # so all three truthy forms are accepted. Accepting them is not a loosening:
    # the mutation writes OFF, which is in none of these sets.
    assert facts["cxx_standard_required"] in ("ON", "TRUE", "1")
    assert facts["cxx_extensions"] in ("OFF", "FALSE", "0")

    command = _compile_command(release_build)
    # The property said 17; this says the compiler was actually told so. The two
    # can disagree -- see the CXX_STANDARD_REQUIRED finding above, where the
    # property is set and the flag silently disappears.
    assert "-std=c++17" in command
    # gnu++17 is what CXX_EXTENSIONS ON would produce, and 13 S2.0 PB-5 warns
    # that GNU extensions are the quiet route by which C++20 spellings arrive.
    assert "-std=gnu++" not in command
    # Catches c++20, c++2a and c++23 in one test. Written as a prefix rather than
    # a list of standard names so a future spelling does not slip past a list
    # nobody remembered to extend.
    assert "-std=c++2" not in command


def test_the_four_warning_flags_reach_the_compiler(release_build):
    """19 S1.1 compile switches. -Werror is what makes the other three a gate.

    Read off the compile command rather than off the target property, because
    that is the level at which a flag can be lost: an -isystem include, a
    CMAKE_CXX_FLAGS override or a later -Wno- would all leave the property intact.

    What this does NOT check is that the flags have any effect on the code in
    common/ -- a header marked SYSTEM would still be exempt from them. That is
    handled in the baseline, which deliberately adds the include directory
    without the SYSTEM keyword.
    """
    command = _compile_command(release_build)
    for flag in ("-Wall", "-Wextra", "-Werror", "-Wpedantic"):
        assert flag in command, "missing %s" % flag


def test_the_debug_build_defines_glibcxx_assertions_and_release_does_not(
        debug_build, release_build):
    """19 S1.1 asks the debug build to support -D_GLIBCXX_ASSERTIONS.

    Both directions are asserted. Only checking the debug side would pass an
    implementation that defines the macro unconditionally, which would put the
    library's bounds checks into the release binary that runs on the robot -- and
    on a 20 Hz control loop that cost is not free. Only checking the release side
    would pass an implementation that never defines it at all.

    Both the resolved property and the command line are read for each direction,
    for the same reason as the standard: a generator expression that evaluated to
    the wrong configuration would leave the property looking right.
    """
    # Debug: present in both views.
    assert "_GLIBCXX_ASSERTIONS" in _read_facts(debug_build)["compile_definitions"]
    assert "_GLIBCXX_ASSERTIONS" in _compile_command(debug_build)
    # Release: absent from both.
    assert "_GLIBCXX_ASSERTIONS" not in _read_facts(release_build)["compile_definitions"]
    assert "_GLIBCXX_ASSERTIONS" not in _compile_command(release_build)


def test_glibcxx_assertions_are_live_and_not_merely_defined(debug_build):
    """The macro being on a command line is not the same as it doing anything.

    19 S11 PMT-40 leans on this build actually trapping: the mutation that lets a
    ROS 2 publish hold a loaned sample is caught because the debug build crashes
    on it. So the probe performs a genuine out-of-range vector access and this
    asserts the process is killed by a signal.

    The no-argument run is asserted first. Without it, a binary that aborted on
    every start would satisfy the probe assertion, and the test would be green
    for a program that never works at all.

    The probe is not run against the release build. Without the macro the same
    access is undefined behaviour with no defined outcome -- it may read rubbish
    and continue -- so an assertion about it would be an assertion about luck.
    The release side is covered by the definition test above instead.
    """
    exe = os.path.join(debug_build, "link_no_ros")     # the Debug artefact

    normal = subprocess.run([exe], capture_output=True, text=True)   # no probe
    assert normal.returncode == 0, "the debug binary cannot even start normally"

    probe = subprocess.run([exe, "glibcxx-assert-probe"],
                           capture_output=True, text=True)
    # Negative means killed by a signal in Python's subprocess convention; the
    # libstdc++ check raises SIGABRT. Asserting "not zero" instead would also
    # accept a clean non-zero exit, which is what a program that merely printed
    # an error and returned would do -- and that is not a trap, it is a report.
    assert probe.returncode < 0, (
        "the out-of-range access did not trap, so -D_GLIBCXX_ASSERTIONS is on "
        "the command line but not in effect (exit %d)" % probe.returncode)


def _headers_on_disk(include_root):
    """Every deployed header, enumerated in Python.

    Independent of CMake on purpose. The aggregate is generated from a glob, and
    the failure that matters is a glob pattern that quietly matches less than it
    should; comparing it against another glob would not see that.

    Takes the root as an argument rather than reading the module constant,
    because the narrowed-glob mutation has to run it over the sandbox copy. A
    helper hard-wired to the real tree would compare the sandbox's aggregate
    against the real directory and be red for the wrong reason.
    """
    found = set()
    for dirpath, _dirnames, filenames in os.walk(include_root):
        for name in filenames:
            # Both suffixes, matching the glob patterns in the baseline. If the
            # two ever disagree this comparison is what says so, which is the
            # point of enumerating twice.
            if name.endswith((".h", ".hpp")):
                full = os.path.join(dirpath, name)
                # Relative to the include root, because that is the spelling the
                # aggregate uses in its include lines and the only form the two
                # sides can be compared in.
                found.add(os.path.relpath(full, include_root))
    return found


def _aggregate_includes(build_dir):
    """The include list inside the generated aggregate header.

    Located by walking the build tree rather than by joining a known path. The
    aggregate lands under the binary directory CMake assigns to the added
    subdirectory, and hard-coding that name here would couple this file to a
    string in the test project's CMakeLists that has no reason to be stable.
    """
    matches = []
    for dirpath, _dirnames, filenames in os.walk(build_dir):
        if "all_deployed_headers.h" in filenames:
            matches.append(os.path.join(dirpath, "all_deployed_headers.h"))
    # Exactly one. Two would mean a stale copy from an earlier configure is also
    # being found, and the caller would then be asserting against whichever the
    # walk happened to reach first.
    assert len(matches) == 1, (
        "expected exactly one generated aggregate, found %r" % (matches,))
    with open(matches[0], encoding="utf-8") as fh:
        return set(re.findall(r'^#include\s+"([^"]+)"', fh.read(), re.M))


def test_the_aggregate_covers_every_deployed_header(release_build):
    """Declares the scan surface, per CLAUDE.md S3.2 form 6.

    This gate only reaches headers the translation unit includes. Generating the
    include list removes the "somebody forgot" failure, but not a glob that
    matches too little -- a pattern of *.hpp alone would drop every .h and the
    aggregate would still compile, still link, and still look complete. So the
    directory is walked again here, in Python, and the two enumerations compared.

    This test earned its place during the item itself: a second header appeared
    under common/include from a parallel piece of work, and the hand-kept include
    list that preceded the generated one went red within minutes. That is the
    behaviour wanted -- every deployed header goes through the no-ROS gate, and
    nobody has to remember.
    """
    on_disk = _headers_on_disk(DEPLOY_INCLUDE)
    # Guards against the whole comparison collapsing to "empty equals empty",
    # which is what a moved or renamed include root would produce.
    assert on_disk, "no headers found under the deployed include directory"
    # Set equality, so both directions are covered: a header on disk and not in
    # the aggregate is a hole in the gate, and one in the aggregate but not on
    # disk means the build is including something that no longer exists.
    assert _aggregate_includes(release_build) == on_disk


# ---------------------------------------------------------------------------
# The mutations. CLAUDE.md S3.3: an assertion that has never been red has not
# been written.
#
# Every mutation runs inside a sandbox copy so the repository is never modified,
# not even transiently -- a mutation test that edits a tracked file and restores
# it leaves the tree broken whenever it is interrupted.
# ---------------------------------------------------------------------------

def _stage_sandbox(tmp_path):
    """A copy of the three pieces the build needs, in their relative layout.

    The layout is reproduced exactly, not flattened. xbrain/common/CMakeLists.txt
    finds its headers through ../../common/include, so a sandbox that put them
    anywhere else would resolve back to the REAL headers -- and every mutation
    that edits a header would then be applied to a file nothing reads, come out
    green, and be reported as run. That is the worst outcome available here: a
    mutation suite that is green because it mutates nothing.
    """
    sandbox = str(tmp_path / "tree")                 # stands in for the repo root
    # Parents created explicitly; copytree below refuses to create intermediate
    # directories for its destination on older Python.
    os.makedirs(os.path.join(sandbox, "xbrain", "common"))    # build script side
    os.makedirs(os.path.join(sandbox, "tests", "common"))     # gate side
    # The headers: copied whole, so a mutation edits the copy and the real
    # deployment artefacts are never touched even transiently.
    shutil.copytree(DEPLOY_INCLUDE, os.path.join(sandbox, "common", "include"))
    shutil.copy(BASELINE_CMAKE,     # the baseline under test
                os.path.join(sandbox, "xbrain", "common", "CMakeLists.txt"))
    shutil.copytree(LINK_NO_ROS_DIR,    # main.cc plus its project file
                    os.path.join(sandbox, "tests", "common", "link_no_ros"))
    return sandbox


def _build_sandbox(sandbox, tmp_path, build_type="Release"):
    """Build the sandbox copy, with its build tree beside the source copy.

    Release by default: none of the mutations are about the debug definitions,
    and an optimised build of one small translation unit is not the slow part.
    """
    return _cmake_build(os.path.join(sandbox, "tests", "common", "link_no_ros"),
                        str(tmp_path / "build"), build_type)


def _prepend_to_header(sandbox, line):
    """Inject a line at the top of the one deployed header.

    Prepended rather than appended so the include lands before the include guard
    and outside every namespace, which is where a real one would be written and
    the only position from which it is unconditionally processed.
    """
    path = os.path.join(sandbox, "common", "include", "xbrain", "digest",
                        "canonical_digest.h")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(line + "\n" + text)


def _append_to_main(sandbox, text):
    """Append a fragment to the sandbox copy of main.cc.

    Appended at file scope, after everything else, so the fragment cannot fail
    for a reason as boring as being inside a function body -- which would still
    make the build red and would still look like a successful mutation.
    """
    path = os.path.join(sandbox, "tests", "common", "link_no_ros", "main.cc")
    with open(path, "a", encoding="utf-8") as fh:
        fh.write("\n" + text + "\n")


def test_the_sandbox_itself_builds(tmp_path):
    """The control for every mutation below.

    Without it a sandbox that was merely staged wrong -- a missing header, the
    wrong relative depth -- would make every mutation come out red and the whole
    mutation suite would be green for a reason that has nothing to do with the
    mutations. This is the same shape as a criterion that can never reach zero.
    """
    sandbox = _stage_sandbox(tmp_path)
    ok, output = _build_sandbox(sandbox, tmp_path)
    assert ok, "the unmutated sandbox must build:\n" + output


def test_mutation_rclcpp_include_in_a_common_header_breaks_the_build(tmp_path):
    """*** Criterion mutation 1, and 19 S11 PMT-1 in its rclcpp spelling.

    On this machine the build dies while preprocessing, because nothing puts a
    ROS include directory on the command line. That is the failure the gate is
    supposed to produce, but see the module docstring: on a machine where a ROS
    prefix is already on the search path, an include that references no symbol
    could get through. The positive assertions carry that half of the claim.
    """
    sandbox = _stage_sandbox(tmp_path)          # the real tree is never touched
    _prepend_to_header(sandbox, "#include <rclcpp/rclcpp.hpp>")   # the mutation
    ok, output = _build_sandbox(sandbox, tmp_path)   # must not survive this
    assert not ok, (
        "a common/ header included rclcpp and the no-ROS target still built; "
        "19 S1.2 requires that to be impossible")
    # Attribution. Without this the test would also pass if the sandbox build
    # broke for an unrelated reason.
    assert "rclcpp" in output


def test_mutation_concepts_breaks_the_build(tmp_path):
    """*** Criterion mutation 2: a C++20 construct must not compile.

    Confirmed in both directions during the item. Raising CXX_STANDARD to 20 in
    the baseline turns this test red -- it is what tells the difference between
    a build pinned at 17 and one that merely happens to compile C++17 code.
    """
    sandbox = _stage_sandbox(tmp_path)
    _append_to_main(sandbox,
                    "template <typename T>\n"
                    "concept LinkNoRosProbe = requires(T t) { t; };")
    ok, output = _build_sandbox(sandbox, tmp_path)
    assert not ok, ("a concepts declaration compiled, so the target is not being "
                    "built at C++17 (13 S9.3 CPP-1)")
    # Attribution: the diagnostic must actually be about the concept. Without
    # this the test would also pass if the sandbox build broke for an unrelated
    # reason, and a mutation whose red cannot be attributed proves nothing.
    assert "concept" in output


def test_the_concepts_mutation_discriminates_the_standard_level():
    """Guards the guard: the snippet is valid C++20 and invalid C++17.

    Compiled directly, outside CMake, so the two runs differ in nothing but the
    -std flag. Without this the concepts mutation would be red for any reason at
    all -- a typo in the snippet would do -- and CLAUDE.md S3.2 calls an
    assertion that is red for the wrong reason a step on the way to one that is
    quietly loosened until it is always green.

    This is also why the criterion's other spelling, std::format, is not what the
    standard-level claim rests on. On a libstdc++ that predates <format> the
    header is absent at C++20 as well, so its failure says nothing about the
    standard in effect.
    """
    source = ("template <typename T>\n"
              "concept LinkNoRosProbe = requires(T t) { t; };\n"
              "int main() { return 0; }\n")
    # The positive half first: at C++20 this is valid code. If it were not, the
    # mutation above would be red for a reason having nothing to do with the
    # standard, and this suite would be quoting it as evidence anyway.
    assert _direct_compile(source, "-std=c++20"), (
        "the concepts snippet does not compile at C++20 either, so its failure "
        "at C++17 is not attributable to the standard level")
    # The negative half: at C++17 it is not. The two together are what make the
    # -std flag the only variable between them.
    assert not _direct_compile(source, "-std=c++17")


def test_mutation_std_format_breaks_the_build(tmp_path):
    """Criterion mutation 2, second spelling.

    Run because the criterion names it. What its red is worth depends on the
    library: where <format> is absent from libstdc++ it fails at C++20 too, and
    the test above is the one that carries the standard-level claim.

    Measured on the toolchain this was written against: raising CXX_STANDARD to
    20 does NOT turn this test green, while it does turn the concepts one green.
    So on that toolchain this mutation cannot distinguish a C++17 build from a
    C++20 one, and reporting it as evidence for CPP-1 would be the "永远红的断言"
    shape. It is kept because the criterion names it and because the day the
    library gains <format> it becomes a second real discriminator.

    No attribution assertion on the output here, deliberately. Demanding the word
    "format" in the diagnostic would encode which of the two failure modes the
    library happens to produce, and that is exactly the thing being left open.
    """
    sandbox = _stage_sandbox(tmp_path)
    _append_to_main(sandbox, "#include <format>")     # C++20 library header
    ok, _output = _build_sandbox(sandbox, tmp_path)   # output deliberately unused
    assert not ok, "a <format> include compiled into the C++17 target"


def test_mutation_gnu_extension_breaks_the_build(tmp_path):
    """CXX_EXTENSIONS OFF, behaviourally rather than by flag text.

    Not named by the criterion, and added because without it the third of the
    three CPP-1 settings would rest entirely on the string "-std=c++17" appearing
    in a command line. typeof is the discriminator: rejected under -std=c++17 and
    accepted under -std=gnu++17, which is exactly what the property controls.

    It catches more than the obvious edit. Setting CXX_STANDARD_REQUIRED to OFF
    turns this red too, because CMake then drops the -std flag entirely and the
    compiler default (gnu++17) applies -- extensions are back on with nobody
    having touched CXX_EXTENSIONS. That path is invisible to a check that only
    reads the property.
    """
    sandbox = _stage_sandbox(tmp_path)
    _append_to_main(sandbox,
                    "int gnu_extension_probe(int a) { typeof(a) b = a; return b; }")
    ok, _output = _build_sandbox(sandbox, tmp_path)
    assert not ok, ("a GNU extension compiled, so the target is being built as "
                    "gnu++17; 13 S9.3 CPP-1 requires CXX_EXTENSIONS OFF")

    # The discrimination, for the same reason as the concepts case: prove the
    # snippet is rejected because of the standard, not because it is malformed.
    # Note the direction is the mirror image of the concepts probe -- here the
    # GNU dialect is the one that accepts it.
    probe = "int f(int a) { typeof(a) b = a; return b; }\nint main() { return f(1); }\n"
    assert _direct_compile(probe, "-std=gnu++17")
    assert not _direct_compile(probe, "-std=c++17")


#: The two glob patterns in xbrain/common/CMakeLists.txt, verbatim. Held as a
#: literal so a rewrite of that block makes the mutations below fail loudly
#: rather than silently stop mutating anything.
GLOB_PATTERNS = ('"${XBRAIN_COMMON_DEPLOY_INCLUDE}/*.h"\n'
                 '     "${XBRAIN_COMMON_DEPLOY_INCLUDE}/*.hpp")')


def _rewrite_glob_patterns(sandbox, replacement):
    """Swap the header glob in the sandbox copy of the baseline.

    The "did anything change" assertion is the important line. A text-replacing
    mutation whose pattern no longer matches applies nothing at all, the build
    then behaves exactly as it should, and the test reports a pass -- a mutation
    that silently stopped mutating is indistinguishable from one that was never
    written, which is the state CLAUDE.md S3.3 exists to prevent.
    """
    path = os.path.join(sandbox, "xbrain", "common", "CMakeLists.txt")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    mutated = text.replace(GLOB_PATTERNS, replacement)
    assert mutated != text, "the header glob was not found to mutate"
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(mutated)


def _write_probe_header(sandbox):
    """A valid extra header outside the digest subtree.

    Added by the mutation rather than relying on whatever happens to sit under
    common/include today: a mutation whose effect depends on the current contents
    of a directory becomes a no-op the day that directory changes, and a no-op
    mutation is indistinguishable from one that was never run.
    """
    # Outside xbrain/digest/ on purpose: the narrowed glob below keeps that one
    # subtree, so the probe is what the narrowing drops.
    directory = os.path.join(sandbox, "common", "include", "xbrain", "probe")
    os.makedirs(directory)
    with open(os.path.join(directory, "mutation_probe.h"), "w",
              encoding="utf-8") as fh:
        # A real include guard and nothing else. Empty content would still be a
        # valid header, but a guard makes it survive being included twice, which
        # is what the aggregate could do if the glob ever returned duplicates.
        fh.write("#ifndef HACHIST_XBRAIN_V6_PROBE_H_\n"
                 "#define HACHIST_XBRAIN_V6_PROBE_H_\n"
                 "#endif\n")
    return "xbrain/probe/mutation_probe.h"     # the key both sets are compared on


def test_mutation_a_narrowed_header_glob_is_caught(tmp_path):
    """The mutation for test_the_aggregate_covers_every_deployed_header.

    Generating the include list removes the human step but not a glob that
    matches too little, and that failure is silent: the aggregate compiles, the
    binary links, the suite is green, and a header nobody looked at is outside
    the gate. Narrowing the glob to one subtree reproduces exactly that, and the
    comparison against the Python walk is what turns it red.
    """
    sandbox = _stage_sandbox(tmp_path)
    probe = _write_probe_header(sandbox)       # a header the narrowing will drop
    # Narrowed to one subtree rather than emptied: an empty glob hits the guard
    # and stops the configure, which is a different mutation (the next test).
    _rewrite_glob_patterns(
        sandbox, '"${XBRAIN_COMMON_DEPLOY_INCLUDE}/xbrain/digest/*.h")')

    ok, output = _build_sandbox(sandbox, tmp_path)
    # Asserting the build SUCCEEDS is the unusual part, and it is the whole
    # point: if narrowing the glob broke the build, nothing would need a
    # comparison test. It does not break it, which is why one is needed.
    assert ok, ("the narrowed glob must still build -- the point is that the "
                "loss is silent:\n" + output)

    on_disk = _headers_on_disk(os.path.join(sandbox, "common", "include"))
    aggregate = _aggregate_includes(str(tmp_path / "build"))
    # The probe header is the specific thing that fell out of the gate. Naming it
    # rather than only comparing the sets makes the failure message point at a
    # file instead of at a set difference.
    assert probe in on_disk
    assert probe not in aggregate
    # And the condition the real test asserts, now inverted: this is exactly the
    # state in which test_the_aggregate_covers_every_deployed_header goes red.
    assert aggregate != on_disk, (
        "the narrowed glob produced the same set as the directory walk, so the "
        "comparison test would not have gone red")


def test_mutation_an_empty_header_glob_refuses_to_configure(tmp_path):
    """A glob matching nothing must stop the build, not produce an empty gate.

    Without the guard this is the worst of the shapes CLAUDE.md S3.2 lists: an
    aggregate with no includes compiles instantly, links instantly, and reports
    that common/ is free of ROS while having looked at nothing.
    """
    sandbox = _stage_sandbox(tmp_path)
    _rewrite_glob_patterns(
        sandbox, '"${XBRAIN_COMMON_DEPLOY_INCLUDE}/*.no_such_suffix")')
    ok, output = _build_sandbox(sandbox, tmp_path)
    assert not ok
    # Attribution again: the failure has to be the guard firing, not a syntax
    # error introduced by the rewrite. Any CMake mistake would also give not-ok.
    assert "no deployed headers found" in output


def test_mutation_removing_the_baseline_call_loses_the_standard_and_the_flags(
        tmp_path):
    """Proves the standard and warning assertions are not vacuous.

    It also demonstrates the trap xbrain_common_apply_cxx_baseline() exists for.
    The CMAKE_CXX_* variables set inside xbrain/common/CMakeLists.txt are
    directory scoped and do NOT cross the add_subdirectory boundary, so a
    consumer that omits the call gets the compiler default -- gnu++17 on GCC 11,
    which is extensions back ON. The point is that the build still SUCCEEDS:
    nothing fails, nothing is logged, and the language level is wrong. That is
    why this asserts on the resolved facts rather than on the build result.
    """
    sandbox = _stage_sandbox(tmp_path)
    path = os.path.join(sandbox, "tests", "common", "link_no_ros", "CMakeLists.txt")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    mutated = text.replace("xbrain_common_apply_cxx_baseline(link_no_ros)", "")
    # Same reason as the glob rewrite: a replacement that matched nothing would
    # leave the file intact and the test would pass while mutating nothing.
    assert mutated != text, "the baseline call was not found to remove"
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(mutated)

    ok, output = _build_sandbox(sandbox, tmp_path)
    assert ok, ("the point of this mutation is that it builds anyway; if it now "
                "fails the demonstration is no longer about a silent failure:\n"
                + output)

    facts = _read_facts(str(tmp_path / "build"))
    # Empty, not merely different. With no target property set, CMake resolves
    # both to nothing at all -- which is precisely the state that would make
    # test_the_target_resolves_to_exactly_cxx17 and
    # test_the_four_warning_flags_reach_the_compiler red, so those two are not
    # vacuous. Verified by running this same edit against the real tree: six
    # tests turned red and the build stayed green.
    assert facts["cxx_standard"] == ""
    assert facts["compile_options"] == ""


def _direct_compile(source, std_flag):
    """Compile a fragment with the bare compiler; True when it succeeds.

    Deliberately does not go through CMake. These probes exist to establish what
    the -std flag alone decides, and routing them through the build system would
    put the thing under test back into the measurement.

    No warning flags either, for the same reason. -Werror could turn an unused
    variable into a failure and the caller would read that as "the standard
    rejected it", which is a different claim entirely.

    Compiles only, with -c: these snippets are about what the front end accepts,
    and dragging in a link step would add failure modes that have nothing to do
    with the language level.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as work:
        src = os.path.join(work, "probe.cc")
        with open(src, "w", encoding="utf-8") as fh:
            fh.write(source)
        proc = subprocess.run(
            [CXX, std_flag, "-c", src, "-o", os.path.join(work, "probe.o")],
            capture_output=True, text=True)
        return proc.returncode == 0
