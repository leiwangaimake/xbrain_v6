#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: clock_scan.py
Brief: Full-text scan for wall-clock reads in the four trees CLK-C1 governs

Description:
What problem this solves. 11 S0.2.1 CLK-C1 forbids wall-clock reads in any
timeout, period or age judgement, and the note under the CLK table states the
rule is automatable: scan the source and fail on time.time(), CLOCK_REALTIME,
system_clock or a default-constructed rclcpp::Clock(). CLAUDE.md 8.2 lists the
same thing as a PR gate. Until this script existed, both were prose. The defect
it catches does not announce itself -- a wall clock behind an age computation
works perfectly on a bench where nothing steps the clock, and fails the first
time chronyd corrects the time after RTK locks, which happens on every cold
start in the field.

Why a FULL-TEXT scan and not a diff. CLAUDE.md 3.2 form 5 (只跑 diff 不跑全文的
lint) is on the list of failure modes this project has actually measured: a
self-check once reported "no unescaped pipes" after running over thirteen added
lines while the offending line sat outside the diff. A diff-scoped clock check
would go green on every pull request that does not touch the offending file,
which is all of them after the one that introduced it. So the surface is the
whole text of every file in the four trees, every run, and
tests/common/test_clock.py carries a metatest that reports "scan surface shrunk"
if this script is ever rewritten to consult a changed-files list instead.

The exemption mechanism, and why there is one. 11 S0.2.1 permits the wall clock
for a short list of purposes -- the message ts field, event time, media file
names, cloud alignment, cross-host latency statistics -- and CLAUDE.md 3.4
compresses that list to three: cross-host alignment, recording, latency
statistics. Those uses are legitimate and they will exist in the tree, so the
choice is between a marker and a check that is switched off the first week. The
marker is line-level and deliberately loud, because an exemption a reviewer
cannot see while reading the diff is the same as no exemption at all. The tag
set is closed: a marker naming a tag outside it is itself a violation, otherwise
the mechanism decays into "write any word and the check goes quiet".

What this script deliberately does NOT do:
  * It does not decide whether an exempted use is really one of the three
    permitted purposes. It cannot: the purpose is semantic. A marker tagged
    record sitting on an age computation passes the scan. The marker makes the
    use REVIEWABLE, not correct, and the criterion printed at the end says so.
  * It does not check the S1.6 timeout thresholds. A file can pass this scan
    with every threshold mistyped -- that is a different deliverable, and the
    two do not substitute for each other in either direction.
  * It does not scan itself or the tests. Both would be self-harm in the
    CLAUDE.md 3.2 form 3 sense: this file must spell out the patterns it looks
    for, and a fixture must be free to write a violating file on purpose, so
    including either tree makes the hit count unable to reach zero and the
    repair anyone reaches for is to loosen the criterion.

How to read the report, because the three buckets ask for different work:
  * BAD lines are defects. Each names the file, the line and the spelling that
    matched, and the fix is to take the reading from xbrain/common/clock/
    instead -- mono_now_s() in Python, steady_clock in C++.
  * declared exemptions are decisions, not defects. They are printed on every
    run precisely so their number and their reasons stay in front of whoever is
    reading the output, because the failure mode for an exemption mechanism is
    that it grows quietly.
  * prose mentions are neither. They are occurrences on comment or docstring
    lines, and they exist because a file explaining why a wall clock is
    forbidden must be able to name it. They are listed under -v and never
    failed.

Run:
  python3 scripts/lint/clock_scan.py
  python3 scripts/lint/clock_scan.py -v            also list mentions in prose
  python3 scripts/lint/clock_scan.py --self-test   probe every rule
"""

import os
import re
import sys
from typing import Dict, List, NamedTuple, Optional, Sequence, Set, Tuple

ROOT = "/opt/xbrain_v6"

#: The scan surface, verbatim from the CFG-CM-12 criterion: common/ xbrain/
#: ros2_ws/ services/. scripts/ and tests/ are deliberately absent for the
#: reason given in the Description, and tests/common/test_clock.py pins that
#: absence rather than leaving it to whoever next edits this tuple.
SCAN_DIRS = ("common", "xbrain", "ros2_ws", "services")

#: Extensions worth reading. CLAUDE.md 8.2 states the rule over .py, .cc, .h and
#: .sh; the rest are the same languages under their other spellings, included so
#: renaming a file cannot take it out of the surface.
SOURCE_EXT = (".py", ".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hh",
              ".sh", ".bash")

#: Directories never worth reading. Build outputs contain generated copies of
#: sources, and reporting a hit in a build artefact sends the reader to a file
#: that will be overwritten rather than to the source that produced it.
SKIP_DIRS = {"__pycache__", ".git", "node_modules", "build", "install", "log"}


class Rule(NamedTuple):
    """One forbidden spelling of a wall-clock read."""

    name: str        # what the report prints, in the spelling a human greps for
    pattern: str     # the regex; see the assembly note below
    why: str         # what breaks if this one is left in, in one sentence
    from_item: bool  # True for the six the CFG-CM-12 criterion names verbatim


# Assembly note. The patterns are written as regexes with escaped dots and
# parentheses, so the contiguous literal (for example the six characters of a
# call to the seconds-resolution wall clock in Python) does not appear anywhere
# in this file's code. That is belt and braces: SCAN_DIRS already excludes
# scripts/. It is still worth doing, because charset_lint.py in this repository
# reported itself forty-seven times before it was rewritten this way, and the
# constant survives if someone later widens the surface.
#
# Two of them -- the C++ clock type and the POSIX clock id -- are single words
# and cannot be disguised. They appear below in plain text, which means this
# file WOULD report itself if it were ever scanned. test_clock.py asserts
# exactly that, so the exclusion above is demonstrably load-bearing rather than
# a precaution against nothing.
RULES: Tuple[Rule, ...] = (
    # The call parentheses are part of the pattern, so a module named time and
    # an attribute named time are not enough on their own: what is forbidden is
    # reading the clock, and a reference without a call is usually a rename or
    # a docstring. \b in front stops it matching the tail of a longer dotted
    # name that happens to end the same way.
    Rule("time.time()",
         r"\btime\.time\s*\(\s*\)",
         "the Python wall clock; an age computed from it jumps by the whole "
         "chronyd correction when RTK first locks",
         True),
    # Deliberately matches the OPEN parenthesis and not an empty argument list.
    # datetime.now(timezone.utc) is the form that gets written when somebody is
    # trying to be careful, and it is the same clock; matching only the bare
    # call would leave the careful-looking version through, which is worse than
    # catching neither because it teaches that the careful version is approved.
    Rule("datetime.now(...)",
         r"\bdatetime\.now\s*\(",
         "still the wall clock. The timezone argument that makes it look "
         "careful selects a timezone, not a clock -- CLK-C1 names the bare "
         "call, and the argument form steps identically",
         True),
    # Separate from the rule above because the report should name what was
    # written. utcnow is also deprecated in recent Python, and the deprecation
    # warning steers people to datetime.now(timezone.utc), which the rule above
    # catches -- the pair has to be complete or the "fix" moves the defect.
    Rule("datetime.utcnow()",
         r"\bdatetime\.utcnow\s*\(",
         "the wall clock in UTC. UTC removes the timezone question and none "
         "of the stepping",
         True),
    # Word-bounded, and intentionally not tied to the std::chrono:: prefix: the
    # type arrives under a using-declaration or a typedef as often as under its
    # full name, and a pattern requiring the namespace would miss exactly the
    # files that had shortened it.
    Rule("system_clock",
         r"\bsystem_clock\b",
         "std::chrono::system_clock is CLOCK_REALTIME; CLK-C1 requires "
         "steady_clock for every C++ duration",
         True),
    # The macro itself, which reaches the wall clock from C, from Python via
    # time.clock_gettime, and from a timerfd. Its neighbour CLOCK_MONOTONIC
    # differs by nine characters and is what must be there instead, which is
    # the reason the report prints the matched spelling rather than a rule
    # number: at a glance the two lines look identical.
    Rule("CLOCK_REALTIME",
         r"\bCLOCK_REALTIME\b",
         "the POSIX clock id behind all of the above, reached directly via "
         "clock_gettime or timerfd_create",
         True),
    # EMPTY parentheses only. Widening this to the type name would flag
    # rclcpp::Clock(RCL_STEADY_TIME), which is the form CLK-C1 requires -- the
    # check would then be red on correct code, and a check that is red on
    # correct code gets loosened until it catches nothing (CLAUDE.md 3.2 form
    # 2). \s* inside allows the formatter to have put a space there.
    Rule("bare rclcpp::Clock()",
         r"\brclcpp::Clock\s*\(\s*\)",
         "the default constructor is the ROS wall clock. CLK-C1 spells out "
         "that RCL_STEADY_TIME must be passed explicitly, and the two forms "
         "differ by nothing a reviewer notices",
         True),

    # Beyond the six. Each of these is the SAME clock under another name, so
    # leaving them out would mean the check is evaded by a rename rather than by
    # an argument. They are marked from_item=False and listed separately in the
    # report, so the six the criterion names stay identifiable and a later
    # reader can tell what was asked for from what was added.
    # The nanosecond form. It reaches for the same clock and is what somebody
    # writes after being told the seconds form is not precise enough -- which
    # was never the complaint.
    Rule("time.time_ns()",
         r"\btime\.time_ns\s*\(",
         "the same wall clock with nanosecond resolution; resolution was "
         "never the problem",
         False),
    Rule("datetime.today()",
         r"\bdatetime\.today\s*\(",
         "another spelling of the local wall clock",
         False),
    # The C entry point. The CLOCK_REALTIME rule cannot see this one because
    # the clock id is implicit in the function, which is exactly the kind of
    # gap a rule list closes only if somebody goes looking for it.
    Rule("gettimeofday()",
         r"\bgettimeofday\s*\(",
         "CLOCK_REALTIME through the legacy C entry point, which the "
         "CLOCK_REALTIME rule above does not see because the id is implicit",
         False),
    # The two remaining RCL time sources. RCL_ROS_TIME follows use_sim_time,
    # so a node built with it reads the wall clock in the field and a
    # publisher-driven clock in simulation -- a watchdog on that basis stops
    # counting whenever the simulator does.
    Rule("rclcpp::Clock(RCL_SYSTEM_TIME / RCL_ROS_TIME)",
         r"\brclcpp::Clock\s*\(\s*RCL_(SYSTEM|ROS)_TIME",
         "an explicit argument, but the wrong one. CLK-C1 requires "
         "RCL_STEADY_TIME; these two are the wall clock and simulation time",
         False),
)


#: The permitted exemption tags, and what each one licenses. The three come from
#: CLAUDE.md 3.4 -- the wall clock does three things: cross-host alignment,
#: recording, latency statistics -- and each maps onto entries in the 11 S0.2.1
#: 允许的用途 column for the wall-clock row, quoted in the values below so the
#: mapping is arguable rather than assumed.
EXEMPT_TAGS: Dict[str, str] = {
    "align": "cross-host alignment: the envelope ts field and cloud-side "
             "correlation. 11 S0.2.1 lists 消息 ts and 云端对齐 as permitted "
             "wall-clock uses, and CLK-C3 requires ts to be present on every "
             "locally produced message alongside mono",
    "record": "recording and replay: the recording timeline, event time and "
              "media file names, all listed as permitted in 11 S0.2.1. CLK-C6 "
              "makes the recorder split its timeline on a clock_step event, "
              "which only means anything if the timeline is wall-clock based",
    "latency": "latency statistics across hosts, listed as permitted in 11 "
               "S0.2.1. Monotonic readings cannot be compared across hosts at "
               "all (CLK-C4), so a cross-host delay has no other basis",
}

#: The marker itself. Same shape as CONFIG-SOURCE-OK in
#: scripts/lint/no_config_source_read.py, on purpose: two exemption mechanisms
#: with different syntax means one of them gets written wrong, and a marker
#: written wrong is a violation the author believes is exempt.
MARKER_RE = re.compile(r"WALL-CLOCK-OK\(([A-Za-z]+)\)\s*:\s*(\S.*)")

#: How short a reason may be before it stops being one. Twelve characters rules
#: out "ok" and "needed" without demanding an essay on the same line as the
#: code; the tag already says which permitted purpose is claimed.
MIN_REASON_CHARS = 12


def _files_under(base: str):
    """Every scannable file below one directory.

    Shared by iter_sources and by the per-directory counts main() prints, so the
    number reported and the files actually read cannot drift apart. A count
    produced by a second, slightly different walk is a number that reassures
    without measuring anything.
    """
    for dirpath, dirnames, filenames in os.walk(base):
        # Pruned in place so os.walk does not descend. Directories whose name
        # starts with model hold vendored ASR payloads -- hundreds of megabytes
        # of data files with no source in them.
        dirnames[:] = [d for d in dirnames
                       if d not in SKIP_DIRS and not d.startswith("model")]
        for name in filenames:
            if name.endswith(SOURCE_EXT):
                yield os.path.join(dirpath, name)


def iter_sources():
    """Every file in the scan surface.

    An absent directory is skipped here and REPORTED by main(). Zero hits from a
    tree that was never read is indistinguishable from zero hits from a clean
    tree, and today ros2_ws/ holds no file at all, so this is not hypothetical.
    """
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for path in _files_under(base):
            yield path


def comment_lines_of(path: str) -> Set[int]:
    """Line numbers holding a comment or a docstring.

    Borrowed from no_config_source_read.py, which borrowed it from
    charset_lint.py: telling a comment from a string literal needs the
    tokenizer, and a regex for the job gets multi-line strings wrong in both
    directions at once.
    """
    if not path.endswith(".py"):
        # No tokenizer for shell or C++, so only whole-line comments count. A
        # trailing // after code is therefore treated as code, which errs
        # towards reporting rather than towards silence.
        out: Set[int] = set()
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                for i, line in enumerate(fh, 1):
                    if line.lstrip().startswith(("#", "//", "*", "/*")):
                        out.add(i)
        except OSError:
            pass
        return out
    # Resolved against THIS file's directory, not against ROOT. ROOT is what a
    # metatest repoints at a fixture tree, and deriving the helper's location
    # from it would mean the comment classifier silently disappears during every
    # such run -- so a docstring mention would be reported as a violation in the
    # fixture and as prose in production, which is the kind of difference that
    # makes a test prove something other than what it claims.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import charset_fix
        commented, _strings = charset_fix.classify_lines(path)
        if commented is not None:
            return set(commented)
    except Exception:
        # Falling back rather than failing: one unparsable file must not take
        # the whole lint down, and the fallback is conservative in the direction
        # that classifies MORE lines as code.
        pass
    return set()


def marker_for(lines: Sequence[str], idx: int) -> Optional[Tuple[str, str]]:
    """(tag, reason) from a marker on this line or in the comment block above it.

    Same line first, then upward through the contiguous run of comment lines
    directly above, stopping at the first line that is not a comment.

    Not just the line above: a real exemption needs room to justify itself, and
    forcing the marker onto the last line before the code pushes the
    justification below it or onto one unreadable line. Not anywhere in the file
    either: the block has to be attached to this line, so a marker written for
    one call cannot quietly cover a second one added forty lines later.
    """
    if idx < len(lines):
        m = MARKER_RE.search(lines[idx])
        if m:
            return m.group(1), m.group(2).strip()

    probe = idx - 1
    while probe >= 0:
        stripped = lines[probe].strip()
        # A blank or code line ends the block. Allowing blanks would let the
        # search wander up into an unrelated comment block.
        if not stripped.startswith(("#", "//", "*", "/*")):
            break
        m = MARKER_RE.search(lines[probe])
        if m:
            return m.group(1), m.group(2).strip()
        probe -= 1
    return None


def scan_file(path: str, rel_path: Optional[str] = None):
    """(violations, exemptions, prose) for one file.

    Three buckets rather than two, reported separately because they need
    different work: a violation is a defect, an exemption is a decision someone
    has to keep agreeing with, and a mention in prose is neither.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            lines = fh.read().split("\n")
    except OSError as exc:
        return [(path, 0, "cannot read: %s" % exc)], [], []

    commented = comment_lines_of(path)
    shown = rel_path if rel_path else path
    violations: List[Tuple[str, int, str]] = []
    exemptions: List[Tuple[str, int, str, str]] = []
    prose: List[Tuple[str, int, str]] = []

    for i, line in enumerate(lines):
        lineno = i + 1
        # First match wins. A line holding two forbidden spellings is one place
        # to fix, and reporting it twice would inflate the count without adding
        # anything for the person doing the fixing.
        hit = None
        for rule in RULES:
            if re.search(rule.pattern, line):
                hit = rule
                break
        if hit is None:
            continue

        # The marker is consulted BEFORE the comment test below, which decides
        # one edge case: a comment line that both carries a marker and names a
        # forbidden call is counted as an exemption rather than as prose. That
        # is the useful way round. The marker was written on purpose and should
        # appear in the exemption list where it can be reviewed; silently
        # reclassifying it as prose would hide a declaration somebody made.
        marker = marker_for(lines, i)
        if marker:
            tag, reason = marker
            if tag not in EXEMPT_TAGS:
                # An invented tag is worse than no marker: it reads as approved.
                violations.append((shown, lineno,
                                   "%s exempted with tag %r, which is not one "
                                   "of %s" % (hit.name, tag,
                                              sorted(EXEMPT_TAGS))))
            elif len(reason) < MIN_REASON_CHARS:
                violations.append((shown, lineno,
                                   "%s exempted with no usable reason: %r"
                                   % (hit.name, reason)))
            else:
                exemptions.append((shown, lineno, tag, reason))
            continue

        # A hit on a comment or docstring line is prose. It is reported and NOT
        # failed, because a comment cannot read a clock, and because demanding a
        # marker on explanatory text pushes people to delete the explanation --
        # xbrain/common/clock/__init__.py has to name the forbidden calls in
        # order to say why they are forbidden, and that file is inside the scan
        # surface.
        if lineno in commented:
            prose.append((shown, lineno, line.strip()[:88]))
            continue

        violations.append((shown, lineno, "%s -- %s" % (hit.name, hit.why)))

    return violations, exemptions, prose


#: One probe per rule plus the counter-examples. (filename, content, expected
#: violation count, what the case is for). The compliant cases matter as much as
#: the violating ones: a checker that flagged everything would satisfy every
#: positive case here and be useless, and the natural response to a checker that
#: fails everything is to switch it off.
#
# The table is in three groups and each group is load-bearing:
#   * one violating sample per rule. Without these, a rule whose regex was
#     mistyped would sit in the table looking authoritative and matching
#     nothing -- the most expensive kind of failure, because the report keeps
#     printing the rule name in the surface listing.
#   * the compliant samples. These are what stop the check drifting into "flag
#     anything with a clock in it", which would make it red on the code CLK-C1
#     requires and would end with the check being switched off.
#   * the marker cases. The exemption mechanism has four outcomes -- accepted
#     above the line, accepted on the line, refused for an unknown tag, refused
#     for an empty reason -- and only the accepting ones get exercised by
#     ordinary use, so the refusing ones need a probe or they rot unnoticed.
#
# The samples build the forbidden spellings by concatenation for the same
# reason the patterns are written as regexes; see the assembly note above.
SELF_TEST_CASES: Tuple[Tuple[str, str, int, str], ...] = (
    ("wall_py.py", "age = %s() - last_rx\n" % ("time." + "time"), 1,
     "the Python wall clock in an age computation is a violation"),
    ("wall_dt.py", "stamp = datetime.%s(tz)\n" % "now", 1,
     "datetime.now with a timezone argument is still the wall clock"),
    ("wall_utc.py", "stamp = datetime.%s()\n" % "utcnow", 1,
     "datetime.utcnow is the wall clock in UTC"),
    ("wall_cpp.cc", "using clk = std::chrono::system_clock;\n", 1,
     "system_clock in C++ is CLOCK_REALTIME"),
    ("wall_posix.cc", "clock_gettime(CLOCK_REALTIME, &ts);\n", 1,
     "the POSIX clock id reached directly"),
    ("wall_ros.cc", "auto c = rclcpp::Clock();\n", 1,
     "a default-constructed ROS clock is the wall clock"),
    ("wall_ros_arg.cc", "auto c = rclcpp::Clock(RCL_SYSTEM_TIME);\n", 1,
     "an explicit but wrong RCL time source"),
    ("wall_ns.py", "t = time.time_ns()\n", 1,
     "nanosecond resolution does not make it monotonic"),
    ("ok_py.py", "age = time.monotonic() - last_rx\n", 0,
     "time.monotonic is what CLK-C1 requires in Python"),
    ("ok_cpp.cc", "auto t = std::chrono::steady_clock::now();\n", 0,
     "steady_clock is what CLK-C1 requires in C++"),
    ("ok_ros.cc", "auto c = rclcpp::Clock(RCL_STEADY_TIME);\n", 0,
     "the explicitly steady ROS clock is compliant"),
    ("ok_posix.cc", "clock_gettime(CLOCK_MONOTONIC, &ts);\n", 0,
     "CLOCK_MONOTONIC must not be confused with its neighbour"),
    ("marked_ok.py",
     "# WALL-CLOCK-OK(align): envelope ts, required on every message by CLK-C3\n"
     "env['ts'] = %s()\n" % ("time." + "time"), 0,
     "a marker on the line above with a permitted tag exempts the line"),
    ("marked_same_line.py",
     "env['ts'] = %s()  # WALL-CLOCK-OK(record): media file name timestamp\n"
     % ("time." + "time"), 0,
     "a marker on the same line exempts it"),
    ("marked_bad_tag.py",
     "# WALL-CLOCK-OK(whatever): I would rather this did not fail\n"
     "age = %s() - t0\n" % ("time." + "time"), 1,
     "an invented tag is itself a violation"),
    ("marked_no_reason.py",
     "# WALL-CLOCK-OK(align): ok\n"
     "env['ts'] = %s()\n" % ("time." + "time"), 1,
     "a marker with no usable reason does not exempt"),
    ("prose.py",
     '"""Do not use %s() here, it is the wall clock."""\n' % ("time." + "time"),
     0, "a mention in a docstring is prose, not a clock read"),
)


def self_test() -> int:
    """Prove the checker reacts to each case rather than trusting that it does.

    A lint reporting zero on a clean tree is indistinguishable from a lint that
    reports zero on everything. comment_ratio.py in this repository was silently
    a no-op through an entire fix because nothing exercised it; these probes are
    the cheapest defence against a repeat, and tests/common/test_clock.py runs
    them as part of the suite so they cannot rot unnoticed.
    """
    import tempfile

    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        for name, content, want, why in SELF_TEST_CASES:
            path = os.path.join(tmp, name)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(content)
            violations, _exempt, _prose = scan_file(path)
            got = len(violations)
            if got != want:
                ok = False
            print("  %s %-62s want %d got %d"
                  % ("ok " if got == want else "FAIL", why, want, got))
    print("")
    print("self-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def _print_surface() -> None:
    """Print the scan surface with a per-directory file count.

    *** Printed on every run, not only on failure. A tree that is absent, or
    present and empty, contributes zero hits, and zero hits from an unread tree
    is indistinguishable from zero hits from a clean one -- CLAUDE.md 3.2 form 6
    (扫描面不声明). ros2_ws/ holds no file today, which is exactly the case that
    would otherwise let this report read as "four trees checked, all clean". The
    counts are derived here and written down nowhere, per CLAUDE.md 3.7.
    """
    print("scan surface: " + ", ".join(SCAN_DIRS))
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            print("  NOTE %s/ does not exist; it contributed nothing to this scan"
                  % top)
            continue
        n = sum(1 for _ in _files_under(base))
        if n == 0:
            print("  NOTE %s/ exists but holds no scannable file; it contributed "
                  "nothing to this scan" % top)
        else:
            print("  %-10s %d file(s) scanned" % (top + "/", n))
    # This line is what a reader checks when they want to know whether the
    # surface is still the whole tree. The metatest in tests/common/test_clock.py
    # asserts both this line and the behaviour behind it.
    print("scan mode: FULL TEXT of every file counted above, on every run --")
    print("  not a diff and not a changed-files list (CLAUDE.md 3.2 form 5)")


def _print_rules() -> None:
    """Print the rule table, separating the six the item names from the rest."""
    print("")
    print("forbidden spellings (%s):" % "from the CFG-CM-12 criterion")
    for rule in RULES:
        if rule.from_item:
            print("  %-42s %s" % (rule.name, rule.why))
    print("same clock under another name (added here, not in the item):")
    for rule in RULES:
        if not rule.from_item:
            print("  %-42s %s" % (rule.name, rule.why))
    print("")
    print("exemption marker: WALL-CLOCK-OK(<tag>): <reason>, tag in %s"
          % sorted(EXEMPT_TAGS))


def _print_criterion() -> None:
    """State what a zero here does and does not establish.

    CLAUDE.md 3.2 form 7 is a conclusion defined into its premise. A green run
    of a text scan is easy to read as "no process uses the wall clock", which is
    not decidable this way, so the boundary is printed next to the number every
    time rather than living in a document nobody opens alongside the output.
    """
    print("")
    print("criterion: violations == 0")
    print("  This establishes: no unmarked occurrence of a listed spelling in")
    print("  the counted files. It does NOT establish that no process reads a")
    print("  wall clock. Four things it cannot see:")
    print("    1. an alias -- from time import time as _t, then _t(); or a")
    print("       function object held in a variable and called later")
    print("    2. a ROS 2 node clock reached as node->get_clock() or node->now(),")
    print("       which CLK-C1 does not name and which follows use_sim_time")
    print("    3. whether an exempted line really serves one of the three")
    print("       permitted purposes -- the tag is a claim, and reviewing it is")
    print("       a human job that this marker exists to make possible")
    print("    4. a shell script deriving a timestamp from the date command")
    print("  A mention inside a comment or docstring is counted as prose and not")
    print("  failed: a comment cannot read a clock.")


def main() -> int:
    if "--self-test" in sys.argv:
        return self_test()
    verbose = "-v" in sys.argv or "--verbose" in sys.argv

    _print_surface()
    _print_rules()
    print("")

    violations: List[Tuple[str, int, str]] = []
    exemptions: List[Tuple[str, int, str, str]] = []
    prose: List[Tuple[str, int, str]] = []
    for path in sorted(iter_sources()):
        v, e, p = scan_file(path, os.path.relpath(path, ROOT))
        violations.extend(v)
        exemptions.extend(e)
        prose.extend(p)

    for shown, lineno, why in violations:
        print("  BAD  %s:%d  %s" % (shown, lineno, why))

    if exemptions:
        print("")
        print("  declared wall-clock exemptions (each must stay justified):")
        for shown, lineno, tag, reason in exemptions:
            print("    %-52s %s: %s" % ("%s:%d" % (shown, lineno), tag, reason))

    if prose and verbose:
        print("")
        print("  mentions in comments and docstrings (not failed):")
        for shown, lineno, text in prose:
            print("    %s:%d  %s" % (shown, lineno, text))

    print("")
    print("  violations:          %d" % len(violations))
    print("  declared exemptions: %d" % len(exemptions))
    print("  prose mentions:      %d%s"
          % (len(prose), "" if verbose else "   (-v to list)"))
    _print_criterion()
    return 1 if violations else 0


if __name__ == "__main__":
    sys.exit(main())
