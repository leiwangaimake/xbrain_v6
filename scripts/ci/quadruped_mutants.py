#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: quadruped_mutants.py
Brief: Mutation runner for the quadruped CHS-A codec and framer (batch B1)

Description:
CLAUDE.md 3.3 says an assertion is not finished until something has made it
turn red. This script is how that is checked for the CHS-A layer, and it is a
committed artifact rather than a scratch file for one reason: the mutants ARE
the specification of what the tests are supposed to catch. Kept out of the
repo, the next person to touch chs_a_codec.cc has no way to tell which of its
lines are defended and which merely look defended.

Each entry replaces one verbatim fragment of a source file with a plausible
wrong version -- the version a careful implementer might actually have typed --
rebuilds, and runs the offline tests. A mutant that still passes is a hole in
the assertions, NOT a harmless variation. The two that survived on the first
run of this set were real: the encoders were never checked for the PatrolDevice
wrapper (only frames read back from the capture were), and the "reserved bytes
are zero" case ran on a stack buffer that happened to be zero already.

Three properties this runner has that an ad-hoc loop usually lacks:

  * the BASELINE is built and run first. Without that check every mutant is
    reported killed the moment the tests are broken for an unrelated reason,
    and the run reads as a perfect score -- CLAUDE.md 3.2 form 1.
  * a mutant that fails to COMPILE is reported separately and fails the run.
    A compiler error is not an assertion; counting it as a kill would credit
    the test suite with catching something it never saw.
  * the source is restored through a try/finally AND an atexit hook, so an
    interrupted run cannot leave a mutated file in the working tree. That file
    would otherwise be a silent candidate for the next commit.

One declared gap. This file sits below the CLAUDE.md 2.4 comment ratio and
cannot reasonably reach it: most of its length is the mutant TABLE, and a table
row is data, not a code block with a hidden why. Every row still carries the
field failure it stands for, which is what 2.4 is actually asking for -- the
text there says the ratio is a means and the real gate is that every block
explains why. Recorded here as declared debt with a reason rather than left to
be discovered and waved through.

No counts are written into any document (CLAUDE.md 3.7). Run it:
  python3 scripts/ci/quadruped_mutants.py
Exit status is 0 only when every mutant compiled and was killed.
"""

import atexit
import os
import shutil
import subprocess
import sys
import tempfile

# Repo root derived from this file, never hardcoded: the script is run from CI,
# from the repo root, and from an editor, and all three must agree.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
QUAD = os.path.join(ROOT, "ros2_ws", "quadruped")
CODEC = os.path.join(QUAD, "src", "chs_a_codec.cc")
FRAMER = os.path.join(QUAD, "src", "chs_a_framer.cc")
GOLDEN = os.path.join(QUAD, "test", "golden", "chs_a_frames.txt")

# Both offline tests are run for every mutant. A change in the codec can be
# caught by the framer's cases and the other way round, and running only the
# "obviously related" test is how a cross-file assertion gets credited to
# nobody and then deleted.
TESTS = ["test_chs_a_codec", "test_chs_a_framer"]

SOURCES = [
    os.path.join(QUAD, "src", "chs_a_codec.cc"),
    os.path.join(QUAD, "src", "chs_a_framer.cc"),
]

# (description, file, verbatim fragment to replace, replacement)
#
# The fragment must occur EXACTLY once. That is asserted rather than assumed:
# an anchor that no longer matches after a refactor would otherwise silently
# turn into a mutant that was never applied and is reported as killed.
MUTANTS = [
    # ---- codec: the wire format -------------------------------------------
    # The peer waits for 16 bytes that already arrived and never answers again.
    # Reads on the link as "the chassis stopped responding", not as a framing
    # bug, which sends the next person to the network instead of to this line.
    ("codec: length field counts the header too",
     CODEC,
     "  h.asdu_len = static_cast<std::uint16_t>(asdu_len);",
     "  h.asdu_len = static_cast<std::uint16_t>(kHeaderBytes + asdu_len);"),
    # Works for every length below 256 -- which is most heartbeats -- and fails
    # only once a report gets large. A bug that appears with traffic volume.
    ("codec: length field written big-endian",
     CODEC,
     "  buf[4] = static_cast<std::uint8_t>(h.asdu_len & 0xFFu);\n"
     "  buf[5] = static_cast<std::uint8_t>((h.asdu_len >> 8) & 0xFFu);",
     "  buf[5] = static_cast<std::uint8_t>(h.asdu_len & 0xFFu);\n"
     "  buf[4] = static_cast<std::uint8_t>((h.asdu_len >> 8) & 0xFFu);"),
    # Nothing is ever framed by the peer. Indistinguishable from a dead link.
    ("codec: fourth sync byte wrong",
     CODEC, "  buf[3] = kSync3;", "  buf[3] = kSync2;"),
    # Responses cannot be matched to the command that caused them, so a refusal
    # is attributed to whatever was sent most recently.
    ("codec: msg id never varies",
     CODEC,
     "  buf[6] = static_cast<std::uint8_t>(h.msg_id & 0xFFu);",
     "  buf[6] = 0;"),
    # The vendor sample leaves them uninitialised (13 V-42), so a build that
    # happens to have zeros on the stack works and the next one does not.
    ("codec: reserved bytes left as they were",
     CODEC, "  std::memset(buf + 11, 0, 5);", "  (void)0;"),
    # ---- codec: the three documented causes of 0xE002 ----------------------
    # 0xE002 on the first frame. One of three causes not in the guide tables.
    ("codec: PatrolDevice wrapper dropped",
     CODEC,
     '"{\\"PatrolDevice\\": {\\"Type\\": %u, \\"Command\\": %u, \\"Time\\": \\"%s\\", "\n'
     '      "\\"Items\\": {\\"MotionParam\\": %d}}}",',
     '"{\\"Type\\": %u, \\"Command\\": %u, \\"Time\\": \\"%s\\", "\n'
     '      "\\"Items\\": {\\"MotionParam\\": %d}}",'),
    # Valid JSON that the chassis rejects. The shape a reviewer would introduce
    # while "fixing" what looks like a decimal code that should be hex.
    ("codec: codes serialised as hex strings",
     CODEC,
     '"{\\"PatrolDevice\\": {\\"Type\\": %u, \\"Command\\": %u, \\"Time\\": \\"%s\\", "\n'
     '      "\\"Items\\": {\\"MotionParam\\": %d}}}",',
     '"{\\"PatrolDevice\\": {\\"Type\\": \\"0x%08x\\", \\"Command\\": \\"0x%08x\\", '
     '\\"Time\\": \\"%s\\", "\n'
     '      "\\"Items\\": {\\"MotionParam\\": %d}}}",'),
    # Same 0xE002 as an absent Time. Survived the suite until the test asserted
    # the field SHAPE rather than its presence -- the fallback string has the
    # right shape, so "there is a Time" was true of a broken renderer.
    ("codec: Time rendered empty",
     CODEC,
     'std::strftime(out, cap, "%Y-%m-%d %H:%M:%S", &tm_buf) == 0) {',
     'std::strftime(out, cap, "", &tm_buf) >= 0) {'),
    # The format a developer would reach for by habit; the chassis wants a space.
    ("codec: Time in ISO 8601 with a T separator",
     CODEC,
     'std::strftime(out, cap, "%Y-%m-%d %H:%M:%S", &tm_buf) == 0) {',
     'std::strftime(out, cap, "%Y-%m-%dT%H:%M:%S", &tm_buf) == 0) {'),
    # Uninitialised stack on the wire: sometimes parses, sometimes not. The
    # fallback exists to convert that into a consistent, loud rejection.
    ("codec: failed conversion leaves the Time buffer untouched",
     CODEC,
     '    std::snprintf(out, cap, "1970-01-01 00:00:00");',
     "    (void)0;"),
    # ---- codec: values and parsing ----------------------------------------
    # Renders "nan", which is not JSON. Our own bug arrives looking like the
    # chassis refusing a legitimate command.
    ("codec: non-finite axis values accepted",
     CODEC,
     "  return std::isfinite(c.vx) && std::isfinite(c.vy) && std::isfinite(c.vz) &&\n"
     "         std::isfinite(c.roll) && std::isfinite(c.pitch) && std::isfinite(c.yaw);",
     "  return true;"),
    # 0 means SUCCESS on this protocol, so every unparseable failure report
    # becomes an acknowledgement and the caller proceeds as if it worked.
    ("codec: unreadable ErrorCode reported as code 0",
     CODEC,
     "    if (ReadU32(s, len, ep, &code)) {\n"
     "      r.has_error_code = true;\n"
     "      r.error_code = code;\n"
     "    }",
     "    ReadU32(s, len, ep, &code);\n"
     "    r.has_error_code = true;\n"
     "    r.error_code = code;"),
    # ---- framer: FR-1 ------------------------------------------------------
    # Frames are sliced 16 bytes short; every ASDU is truncated JSON.
    ("framer: length read as if it included the header",
     FRAMER,
     "    const std::size_t need = kHeaderBytes + h.asdu_len;",
     "    const std::size_t need = h.asdu_len;"),
    # The same frame is delivered forever and the buffer never drains. At 10 Hz
    # the robot acts on one stale report and never sees another.
    ("framer: frame not marked consumed after being handed out",
     FRAMER,
     "    pending_consume_ = need;",
     "    pending_consume_ = 0;"),
    # ---- framer: FR-2 ------------------------------------------------------
    # A sync word split across two reads is the COMMON case at a frame
    # boundary; discarding the prefix loses one frame per occurrence, quietly.
    ("framer: failed resync discards the whole buffer",
     FRAMER,
     "    skip = (used_ >= 3) ? used_ - 3 : 0;",
     "    skip = used_;"),
    # A healthy link is declared poisoned after enough hours of small noise.
    # The failure gets more likely the longer the robot runs.
    ("framer: noise run not cleared by a good frame",
     FRAMER,
     "    resync_run_ = 0;  // a good frame ends the current noise run",
     "    (void)0;"),
    # Only visible at exactly the configured budget, which is why the test
    # pins both sides of it rather than one.
    ("framer: resync budget is off by one",
     FRAMER,
     "  return resync_run_ <= resync_max_bytes_;",
     "  return resync_run_ < resync_max_bytes_;"),
    # A frame parsed from a stream just declared untrustworthy is still handed
    # out, so the robot acts on it AND reconnects.
    ("framer: poisoned link keeps its buffered bytes",
     FRAMER,
     "        used_ = 0;\n"
     "        partial_since_s_ = -1.0;\n"
     "        resync_run_ = 0;\n"
     "        return FrameStatus::kPoisoned;",
     "        partial_since_s_ = -1.0;\n"
     "        resync_run_ = 0;\n"
     "        return FrameStatus::kPoisoned;"),
    # ---- framer: FR-3 ------------------------------------------------------
    # Makes the timeout unreachable on a link delivering one byte per poll --
    # precisely the stall FR-3 exists to break.
    ("framer: assembly timer refreshed on every call",
     FRAMER,
     "      if (partial_since_s_ < 0.0) {\n"
     "        partial_since_s_ = now_mono_s;\n"
     "        return FrameStatus::kNeedMore;\n"
     "      }",
     "      {\n"
     "        partial_since_s_ = now_mono_s;\n"
     "        return FrameStatus::kNeedMore;\n"
     "      }"),
    # The boundary is a specification statement ("older than"), not a taste.
    ("framer: assembly timeout is off by one",
     FRAMER,
     "      if (now_mono_s - partial_since_s_ <= frame_assembly_timeout_s_) {",
     "      if (now_mono_s - partial_since_s_ < frame_assembly_timeout_s_) {"),
    # ---- framer: FR-5 ------------------------------------------------------
    # A doubled datagram silently loses its second report; UDP gives no
    # ordering that would let a leftover be continued.
    ("framer: datagram not required to hold exactly one frame",
     FRAMER,
     "  if (kHeaderBytes + static_cast<std::size_t>(h.asdu_len) != len) {",
     "  if (kHeaderBytes + static_cast<std::size_t>(h.asdu_len) > len) {"),
    # Bytes from the previous connection are parsed as the start of the new
    # one, so the first frame after every reconnect can be garbage.
    ("framer: Reset keeps the bytes of the old connection",
     FRAMER,
     "void Framer::Reset() {\n  used_ = 0;",
     "void Framer::Reset() {\n  used_ = used_;"),
]


def build_and_run(workdir, quiet=True):
    """Compile both offline tests and run them.

    Returns (compiled, passed). The two are separate because a mutant that does
    not compile has NOT been caught by any assertion, and reporting it as a
    kill would credit the tests with work they did not do.
    """
    for test in TESTS:
        exe = os.path.join(workdir, test)
        cmd = [
            "g++", "-std=c++17", "-w",
            "-I", os.path.join(ROOT, "common", "include"),
            "-I", os.path.join(QUAD, "include"),
            "-o", exe,
            os.path.join(QUAD, "test", test + ".cc"),
        ] + SOURCES
        cc = subprocess.run(cmd, capture_output=True, text=True)
        if cc.returncode != 0:
            if not quiet:
                sys.stderr.write(cc.stderr)
            return (False, False)
        # A hung framer (a resync that fails to advance, say) must not hang the
        # run: without the timeout it is indistinguishable from a slow pass.
        try:
            run = subprocess.run([exe, GOLDEN], capture_output=True, text=True,
                                 timeout=60)
        except subprocess.TimeoutExpired:
            return (True, False)
        if run.returncode != 0:
            if not quiet:
                sys.stdout.write(run.stdout)
            return (True, False)
    return (True, True)


def main():
    backups = {}
    for path in SOURCES:
        with open(path, "r", encoding="utf-8") as f:
            backups[path] = f.read()

    def restore():
        for p, text in backups.items():
            with open(p, "r", encoding="utf-8") as f:
                if f.read() == text:
                    continue
            with open(p, "w", encoding="utf-8") as f:
                f.write(text)

    atexit.register(restore)
    workdir = tempfile.mkdtemp(prefix="quadruped_mut_")

    survived = []
    broken = []
    killed = 0
    try:
        ok, passed = build_and_run(workdir, quiet=False)
        if not ok or not passed:
            print("BASELINE IS NOT GREEN -- every mutant below would be "
                  "reported killed for the wrong reason. Fix the build or the "
                  "tests first.")
            return 2
        print("baseline: builds and passes")

        for desc, path, old, new in MUTANTS:
            text = backups[path]
            hits = text.count(old)
            if hits != 1:
                # An anchor that stopped matching is a broken mutant, and a
                # broken mutant that is silently skipped reads exactly like a
                # mutant that was killed.
                broken.append((desc, "anchor matched %d times, need 1" % hits))
                print("  ANCHOR   %s" % desc)
                continue
            with open(path, "w", encoding="utf-8") as f:
                f.write(text.replace(old, new))
            try:
                compiled, still_passes = build_and_run(workdir)
            finally:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(text)
            if not compiled:
                broken.append((desc, "does not compile"))
                print("  NOBUILD  %s" % desc)
            elif still_passes:
                survived.append(desc)
                print("  SURVIVED %s" % desc)
            else:
                killed += 1
                print("  killed   %s" % desc)
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
        restore()

    print("")
    print("mutants: %d killed, %d survived, %d unusable"
          % (killed, len(survived), len(broken)))
    for desc in survived:
        print("  SURVIVED: %s" % desc)
        print("    -> either an assertion is missing, or this is an equivalent")
        print("       mutant, in which case say so in the code (CLAUDE.md 7.2.1)")
    for desc, why in broken:
        print("  UNUSABLE: %s (%s)" % (desc, why))
    return 0 if (not survived and not broken) else 1


if __name__ == "__main__":
    sys.exit(main())
