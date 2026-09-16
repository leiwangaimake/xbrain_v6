#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cxx_mutants.py
Brief: Mutation runner for the C++ offline tests, one suite per module

Description:
CLAUDE.md 3.3 says an assertion is not finished until something has made it
turn red. This script is how that is checked for the C++ side, and it is a
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
  * the source is restored through a try/finally, an atexit hook AND a SIGTERM
    handler, so a killed run cannot leave a mutated file in the working tree.
    The signal handler is not belt-and-braces: atexit does NOT run on SIGTERM,
    which is what pkill and every job supervisor send, and a run killed that
    way on 2026-09-16 left one mutant behind in a file that was still untracked
    -- so `git status` showed nothing, the next build compiled the mutation,
    and the test that went red pointed at the test rather than at the cause.
  * --verify answers "is there a mutant in the tree right now" WITHOUT git,
    by requiring every mutant's original text to be present exactly once. That
    is the check git cannot do: an untracked file shows no diff whatever was
    written into it, and every new module is untracked for one batch.

One declared gap. This file sits below the CLAUDE.md 2.4 comment ratio and
cannot reasonably reach it: most of its length is the mutant TABLE, and a table
row is data, not a code block with a hidden why. Every row still carries the
field failure it stands for, which is what 2.4 is actually asking for -- the
text there says the ratio is a means and the real gate is that every block
explains why. Recorded here as declared debt with a reason rather than left to
be discovered and waved through.

Suites, each a (sources, tests, mutants) triple:
  quadruped  the CHS-A codec and framer (batch B1)
  yaml_lite  the shared config reader in common/, whose tests live with the
             sensor package -- it is header-only, so the header IS the source

No counts are written into any document (CLAUDE.md 3.7). Run it:
  python3 scripts/ci/cxx_mutants.py [suite ...]     (no args = every suite)
  python3 scripts/ci/cxx_mutants.py --verify        (tree clean? no builds)
Exit status is 0 only when every mutant compiled and was killed.
"""

import atexit
import os
import shutil
import signal
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

# Both quadruped tests run for every quadruped mutant. A change in the codec
# can be caught by the framer's cases and the other way round, and running only
# the "obviously related" test is how a cross-file assertion gets credited to
# nobody and then deleted.
QUAD_TESTS = [os.path.join(QUAD, "test", "test_chs_a_codec.cc"),
              os.path.join(QUAD, "test", "test_chs_a_framer.cc")]
QUAD_SOURCES = [
    os.path.join(QUAD, "src", "chs_a_codec.cc"),
    os.path.join(QUAD, "src", "chs_a_framer.cc"),
]

# yaml_lite is header-only, so the header is both the source and the thing
# mutated; its test lives with the sensor package for historical reasons.
YAML_LITE_H = os.path.join(ROOT, "common", "include", "xbrain", "config", "yaml_lite.h")
YAML_TESTS = [os.path.join(ROOT, "ros2_ws", "sensor", "test", "test_yaml_lite.cc")]

# (description, file, verbatim fragment to replace, replacement[, expect])
#
# The fragment must occur EXACTLY once. That is asserted rather than assumed:
# an anchor that no longer matches after a refactor would otherwise silently
# turn into a mutant that was never applied and is reported as killed.
#
# `expect` defaults to "killed". The other value is "equivalent": a mutant that
# provably CANNOT be caught, because the public behaviour is identical. Those
# are not holes and CLAUDE.md 7.2.1 says to note them in the code rather than
# invent an assertion for them -- but the claim that a mutant is equivalent is
# itself a claim, and it rots. Declaring it here turns it into something the
# runner checks: an "equivalent" mutant that starts being KILLED means the code
# changed so that the line now carries weight, and the run fails until the note
# in the source is corrected.
QUAD_MUTANTS = [
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


# yaml_lite mutants. The accessors these defend were added because a list of
# bare scalars (reconnect_backoff_s) could not be read from C++ at all: the key
# sat in the config, the loader never read it, and nothing failed.
YAML_MUTANTS = [
    ("yaml_lite: sequence entry read as a map is coerced, not refused",
     YAML_LITE_H,
     '    if (is_map_) {\n'
     '      throw std::runtime_error("config node is a map, not a scalar: " + label);\n'
     '    }',
     "    if (false) { (void)label; }"),
    # A null rung in the backoff ladder would become 0.0 s, turning a reconnect
    # into a busy loop against the chassis -- CLAUDE.md 3.1 at element level.
    ("yaml_lite: null sequence entry treated as a value",
     YAML_LITE_H,
     '    if (is_null()) {\n'
     '      throw std::runtime_error(\n'
     '          "config node is null (uncalibrated per CLAUDE.md 3.1): " + label);\n'
     '    }',
     "    if (false) { (void)label; }"),
    # stod stops at the first unusable character, so without the length check
    # "1.0abc" parses as 1.0 and a typo survives review.
    ("yaml_lite: trailing garbage after a number accepted",
     YAML_LITE_H,
     '    if (pos != s.size()) {\n'
     '      throw std::runtime_error("config key not a number: " + label + " = \'" + s + "\'");\n'
     '    }\n'
     "    return v;\n"
     "  }\n"
     "  long as_int(",
     "    (void)pos;\n"
     "    return v;\n"
     "  }\n"
     "  long as_int("),
    ("yaml_lite: trailing garbage after an int accepted",
     YAML_LITE_H,
     '    if (pos != s.size()) {\n'
     '      throw std::runtime_error("config key not an int: " + label + " = \'" + s + "\'");\n'
     "    }\n"
     "    return v;\n"
     "  }\n"
     "  bool as_bool(",
     "    (void)pos;\n"
     "    return v;\n"
     "  }\n"
     "  bool as_bool("),
    # A sequence read as a scalar would hand back the empty string, which then
    # parses as whatever the caller's type defaults to.
    ("yaml_lite: sequence node answers the scalar accessors",
     YAML_LITE_H,
     '    if (is_seq_) {\n'
     '      throw std::runtime_error("config node is a sequence, not a scalar: " + label);\n'
     '    }',
     "    if (false) { (void)label; }"),
]

# Config-layer mutants. These defend the data path itself: a key that is
# present in the file but never read by the loader fails nothing, and the value
# simply never reaches the process (13 S8.2 v1.4 is the same defect on the
# tier1 limits).
CONFIG_SOURCES = [
    os.path.join(QUAD, "src", "quadruped_config.cc"),
    os.path.join(QUAD, "src", "chs_a_codec.cc"),
    os.path.join(QUAD, "src", "chs_a_framer.cc"),
]
CONFIG_TESTS = [os.path.join(QUAD, "test", "test_quadruped_config.cc")]
CONFIG_CC = os.path.join(QUAD, "src", "quadruped_config.cc")

CONFIG_MUTANTS = [
    # Without the loop, a zero or negative rung passes and the reconnect turns
    # into a busy loop -- audible, because the chassis greets every connect.
    ("config: reconnect ladder rungs not checked for positivity",
     CONFIG_CC,
     "    if (!(cfg.link.reconnect_backoff_s[i] > 0.0)) {",
     "    if (false) {"),
    ("config: empty reconnect ladder accepted",
     CONFIG_CC,
     "  if (cfg.link.reconnect_backoff_s.empty()) {",
     "  if (false) {"),
    # The ladder read as a list of zeros would satisfy a size-only assertion.
    ("config: ladder read but values discarded",
     CONFIG_CC,
     "      cfg.link.reconnect_backoff_s.push_back(backoff.at_index(i).as_double(label));",
     "      (void)label;\n      cfg.link.reconnect_backoff_s.push_back(0.5);"),
    ("config: axis_cmd_socket_fixed=false accepted (13 CA-1)",
     CONFIG_CC, "  if (!cfg.link.axis_cmd_socket_fixed) {", "  if (false) {"),
    ("config: single_tx_owner=false accepted (13 CA-4 / QC-16)",
     CONFIG_CC, "  if (!cfg.link.single_tx_owner) {", "  if (false) {"),
    ("config: version byte may disagree with the codec",
     CONFIG_CC,
     "  if (cfg.link.proto_version_byte != static_cast<int>(chs_a::kProtoVersion)) {",
     "  if (false) {"),
    ("config: asdu_format not checked",
     CONFIG_CC, '  if (cfg.link.asdu_format != "json") {', "  if (false) {"),
    ("config: codebook not checked (CB-1)",
     CONFIG_CC, '  if (cfg.link.codebook != "hex32") {', "  if (false) {"),
    # QC-13 is all-or-nothing. "Any non-empty table is refused" is a DIFFERENT
    # and wrong rule, which is why the suite also loads a complete table.
    ("config: half-filled legacy codebook accepted (QC-13)",
     CONFIG_CC,
     "  if (cfg.link.legacy_decimal_entries != 0 &&\n"
     "      cfg.link.legacy_decimal_entries != kLegacyCodebookEntries) {",
     "  if (false) {"),
    ("config: complete legacy codebook wrongly refused",
     CONFIG_CC,
     "  if (cfg.link.legacy_decimal_entries != 0 &&\n"
     "      cfg.link.legacy_decimal_entries != kLegacyCodebookEntries) {",
     "  if (cfg.link.legacy_decimal_entries != 0) {"),
    # robot_id is the {rid} of every key. A malformed one yields keys that are
    # well-formed and match nothing, which looks exactly like a dead network.
    ("config: robot_id charset not checked",
     CONFIG_CC, "    if (!IsValidRobotId(cfg.robot_id)) {", "    if (false) {"),
    ("config: robot_id upper case accepted",
     CONFIG_CC, "    const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||",
     "    const bool ok = (c >= 'A' && c <= 'z') || (c >= '0' && c <= '9') ||"),
    ("config: robot_id length bound only refuses the empty string",
     CONFIG_CC, "  if (id.empty() || id.size() > 32) return false;",
     "  if (id.empty()) return false;"),
    ("config: robot_id length bound is off by one",
     CONFIG_CC, "  if (id.empty() || id.size() > 32) return false;",
     "  if (id.empty() || id.size() >= 32) return false;"),
    # A key an operator can set that changes nothing is worse than no key.
    ("config: a special-gait whitelist is accepted and then ignored",
     CONFIG_CC, "      if (special.size() != 0) {", "      if (false) {"),
    ("config: the axis list is not checked against 11 S9.3.1",
     CONFIG_CC, "      if (!matches) {", "      if (false) {"),
    # Order matters: [wz, vy, vx] is not [vx, vy, wz], and a membership test
    # would call them the same.
    ("config: the axis list is checked as a SET, losing the order",
     CONFIG_CC,
     "        matches = active.at_index(i).as_scalar(label) == kExpected[i];",
     "        matches = active.at_index(i).as_scalar(label).size() == 2;"),
]

# Report-parsing mutants. The two bans of 13 S6.5 are the ones with teeth: a
# parser that maps an unregistered value onto a known one invents chassis state,
# and one that rejects the report over a strange field throws away HES, which
# travels in the same message.
REPORTS_CC = os.path.join(QUAD, "src", "chs_a_reports.cc")
REPORTS_SOURCES = [REPORTS_CC, os.path.join(QUAD, "src", "chs_a_codec.cc")]
REPORTS_TESTS = [os.path.join(QUAD, "test", "test_chs_a_reports.cc")]

REPORTS_MUTANTS = [
    # Ban 1. Nearest-match is the plausible wrong answer: it produces a
    # readable label for every value and invents state that was never reported.
    ("reports: unregistered value falls back to the first table entry",
     REPORTS_CC,
     "  v.known = false;\n  v.label = UnknownLabel(raw);\n  return v;",
     "  v.known = true;\n  v.label = table[0].name;\n  return v;"),
    # Ban 3. The label alone is not actionable in the field.
    ("reports: raw value dropped once a label is found",
     REPORTS_CC, "  v.raw = raw;", "  v.raw = 0;"),
    # The 11-vs-13 trap: reading an emergency stop as a damping state.
    ("reports: soft_estop taken from the OLD manual (2, not -2)",
     REPORTS_CC, '    {-2, "soft_estop"},', '    {2, "soft_estop"},'),
    # 13 S7.3's asymmetric default. "warn" reads as merely noisy.
    ("reports: unknown severity mapped to warn instead of degraded",
     REPORTS_CC,
     "  return std::string(sets::kFaultLevel[1]);\n}",
     "  return std::string(sets::kFaultLevel[0]);\n}"),
    ("reports: a missing Severities field treated as present",
     REPORTS_CC, "  if (present) {", "  if (true) {"),
    # 11 S9.8.3 puts the SOC judgement on the minimum: the emptier pack decides
    # when the robot must come home.
    ("reports: battery SOC taken from the fuller pack",
     REPORTS_CC, "    if (first || b.level < s.min_level) {",
     "    if (first || b.level > s.min_level) {"),
    ("reports: SOC seeded from zero instead of the first pack",
     REPORTS_CC, "    if (first || b.level < s.min_level) {",
     "    if (b.level < s.min_level) {"),
    # CF-3 / 11 S9.8.4: raised and cleared are two lists, and merging them
    # reports a fault that has already gone away as still active.
    ("reports: cleared faults filed as active",
     REPORTS_CC, "    if (f.type == 2) {", "    if (false) {"),
    # CF-1 / CF-2: a bare code cannot be interpreted -- the two spaces overlap.
    ("reports: fault code emitted without its namespace prefix",
     REPORTS_CC, '  std::snprintf(buf, sizeof(buf), "%s:0x%04X", prefix,',
     '  (void)prefix;\n  std::snprintf(buf, sizeof(buf), "0x%04X",'),
    # CF-4's dedup key IS the formatted string, so two spellings are two faults.
    ("reports: hex body rendered lower case",
     REPORTS_CC, '"%s:0x%04X", prefix,', '"%s:0x%04x", prefix,'),
    # The healthy case, twice a second.
    ("reports: an empty ErrorList treated as a parse failure",
     REPORTS_CC,
     "  if (el == items->end() || !el->is_array()) return false;",
     "  if (el == items->end() || !el->is_array() || el->empty()) return false;"),
    # Ban 2 through a type error: a firmware that changed a field's type would
    # otherwise throw out of the parse and cost the whole report.
    ("reports: wrong-typed number field no longer falls back",
     REPORTS_CC,
     "  if (it == j.end() || !it->is_number()) return dflt;\n"
     "  return it->get<std::int64_t>();",
     "  if (it == j.end()) return dflt;\n"
     "  return it->get<std::int64_t>();"),
    # An empty slot and a flat pack both report level 0. Confusing them makes a
    # single-battery robot read 0% forever, or hides a genuinely empty one.
    ("reports: an empty slot counted as a present pack",
     REPORTS_CC, "    b.present = b.voltage > 0.0;", "    b.present = true;"),
    ("reports: a pack discharged to 0% counted as absent",
     REPORTS_CC, "    b.present = b.voltage > 0.0;",
     "    b.present = b.voltage > 0.0 && b.level > 0;"),
    ("reports: absent packs silently excluded from the SOC minimum",
     REPORTS_CC, "    if (first || b.level < s.min_level) {",
     "    if (!b.present) { s.batteries.push_back(b); continue; }\n"
     "    if (first || b.level < s.min_level) {"),
    # HES and Sleep arrive as 0/1 integers, `charge` as a real bool.
    ("reports: integer booleans no longer accepted",
     REPORTS_CC, "  if (it->is_number()) return it->get<std::int64_t>() != 0;",
     "  (void)0;"),
    # The envelope IS a real failure: a zeroed struct reads as "idle and fine".
    ("reports: a payload without the PatrolDevice wrapper accepted",
     REPORTS_CC,
     "  if (pd == root.end() || !pd->is_object()) return nullptr;",
     "  if (false) return nullptr;"),
    # Declared EQUIVALENT, with the reasoning in chs_a_reports.cc: every parse
    # function calls ItemsOf immediately after this, and find() on a discarded
    # or non-object value returns end() without throwing, so the report is
    # refused one step later with the same answer. Kept as a mutant so that the
    # day the line becomes load-bearing, the runner says so.
    ("reports: malformed JSON accepted",
     REPORTS_CC,
     "  return !out->is_discarded() && out->is_object();",
     "  return out->is_object() || true;",
     "equivalent"),
]

# Session-policy mutants. Each one is a rule that reads the same forwards and
# backwards until you ask what it costs in the field.
SESSION_CC = os.path.join(QUAD, "src", "chs_a_session.cc")
SESSION_SOURCES = [SESSION_CC, os.path.join(QUAD, "src", "quadruped_config.cc"),
                   os.path.join(QUAD, "src", "chs_a_codec.cc")]
SESSION_TESTS = [os.path.join(QUAD, "test", "test_chs_a_session.cc")]

SESSION_MUTANTS = [
    # 13 TLS-4. Dialling anyway turns "no certificate installed" into a probe
    # window of silence, which is what an unplugged cable looks like.
    ("session: TLS candidate dialled without credentials",
     SESSION_CC, "    if (ep.tls && creds_ && !creds_(ep)) {", "    if (false) {"),
    ("session: disabled candidate dialled anyway",
     SESSION_CC, "    if (!ep.enabled) {", "    if (false) {"),
    # 13 CA-1: two live sockets read to the chassis as two CLIENTS, and axis
    # commands come back 0xE006 for two seconds -- accepted, and no motion.
    ("session: previous socket left open while the next is dialled",
     SESSION_CC,
     "  if (socket_open_) {\n    hangup_();\n    socket_open_ = false;\n"
     "    out->disconnected = true;\n  }\n  while (candidate_ < cfg_.endpoints.size()) {",
     "  while (candidate_ < cfg_.endpoints.size()) {"),
    ("session: probe window never expires",
     SESSION_CC,
     "    } else if (now_mono_s - probe_started_s_ > cfg_.probe_timeout_s) {",
     "    } else if (false) {"),
    # A robot that has been up for hours and drops once must wait the FIRST
    # rung, not the last: otherwise it is out of contact ten times too long.
    ("session: backoff ladder not reset by a good connection",
     SESSION_CC, "      backoff_attempt_ = 0;\n      send_failures_ = 0;",
     "      send_failures_ = 0;"),
    ("session: backoff always uses the first rung",
     SESSION_CC,
     "  return cfg_.reconnect_backoff_s[attempt < n ? attempt : n - 1];",
     "  (void)attempt;\n  return cfg_.reconnect_backoff_s[0];"),
    # 13 S2.5 thresholds. Swapping them makes "degraded" unreachable, so the
    # link goes from fine to gone with no warning in between.
    ("session: degraded judged against the lost threshold",
     SESSION_CC, "    if (age > cfg_.state_timeout_degraded_s) {",
     "    if (age > cfg_.state_timeout_lost_s) {"),
    ("session: lost never declared, the link just stays degraded",
     SESSION_CC,
     "    if (last_report_s_ < 0.0 || age > cfg_.state_timeout_lost_s) {",
     "    if (false) {"),
    # CONSECUTIVE, not cumulative.
    ("session: send-failure run not cleared by a success",
     SESSION_CC,
     "  // cumulative counter would eventually degrade a link that has been healthy\n"
     "  // for hours with a handful of transient failures spread across them.\n"
     "  send_failures_ = 0;",
     "  // cumulative counter would eventually degrade a link that has been healthy\n"
     "  // for hours with a handful of transient failures spread across them.\n"
     "  (void)0;"),
    ("session: E001..E005 no longer count toward cmd_fail",
     SESSION_CC, "  if (d.counts_toward_cmd_fail) ++send_failures_;", "  (void)0;"),
    ("session: 0xE00B run not broken by another code",
     SESSION_CC, "    internal_errors_ = 0;\n  }\n}\n\nvoid Session::OnSleep",
     "    (void)0;\n  }\n}\n\nvoid Session::OnSleep"),
    # 13 F-21. There is no wake command in the protocol, and each attempt is
    # five seconds of believing we are driving.
    ("session: motion sent while the chassis reports Sleep",
     SESSION_CC, "  if (asleep_) return false;", "  if (false) return false;"),
    ("session: motion allowed on a degraded link",
     SESSION_CC, "  if (state_ != ConnState::kOk) return false;",
     "  if (state_ == ConnState::kLost) return false;"),
    # The chassis reports only to an address already sending heartbeats, so a
    # probe that waits a period first spends it guaranteed to hear nothing.
    ("session: no heartbeat forced on the dialling tick",
     SESSION_CC, "    last_heartbeat_s_ = -1.0;\n    return;",
     "    last_heartbeat_s_ = now_mono_s;\n    return;"),
    # CON-05: the epoch is how the layer above learns it must handshake again.
    ("session: link epoch never advances",
     SESSION_CC, "      ++link_epoch_;", "      (void)0;"),
    # A report that arrived BEFORE this candidate was dialled is evidence about
    # the previous one, and accepting it declares a silent endpoint live.
    ("session: a stale report counts as this candidate answering",
     SESSION_CC,
     "    if (last_report_s_ >= probe_started_s_ && last_report_s_ >= 0.0) {",
     "    if (last_report_s_ >= 0.0) {"),
]

# Tier 1 mutants. This is the last thing between a command and the legs, so the
# list is longer than the others and every entry names what reaches the robot.
TIER1_CC = os.path.join(QUAD, "src", "tier1.cc")
TIER1_SOURCES = [TIER1_CC, os.path.join(QUAD, "src", "quadruped_config.cc"),
                 os.path.join(QUAD, "src", "chs_a_codec.cc")]
TIER1_TESTS = [os.path.join(QUAD, "test", "test_tier1.cc")]

TIER1_MUTANTS = [
    # ---- the precedence ladder (11 S4.1: the closed set order IS the order) --
    # Each swap changes what a robot reports when two things are wrong at once,
    # which is the situation an engineer is actually looking at.
    ("tier1: hardware stop checked AFTER the timeout",
     TIER1_CC, "  if (in.hes_raw) {\n    hes_lock_ = true;\n  }",
     "  if (false) {\n    hes_lock_ = true;\n  }"),
    ("tier1: soft stop checked before the timeout lock",
     TIER1_CC, "  if (timeout_lock_) {\n    // Reached only once the command is fresh",
     "  if (false) {\n    // Reached only once the command is fresh"),
    ("tier1: sleep checked before the mode switch",
     TIER1_CC, "  if (in.mode_switching) {", "  if (false) {"),
    ("tier1: the mode mismatch branch never fires",
     TIER1_CC, "  if (in.usage_mode_raw != kUsageModeNavigation) {",
     "  if (false) {"),
    ("tier1: a poisoned payload is executed",
     TIER1_CC, "  if (!AllFinite(in)) {", "  if (false) {"),
    # ---- the locks, from the direction that matters ------------------------
    ("tier1: hardware stop released by the signal alone",
     TIER1_CC, "    if (!in.hes_raw && in.enable_requested) {",
     "    if (!in.hes_raw) {"),
    ("tier1: hardware stop released by the request alone",
     TIER1_CC, "    if (!in.hes_raw && in.enable_requested) {",
     "    if (in.enable_requested) {"),
    ("tier1: timeout lock clears itself when the upstream returns",
     TIER1_CC, "    Tier1Output o = Stop(StopReason::kTimeout);\n"
               "    if (in.enable_requested) {\n      timeout_lock_ = false;",
     "    Tier1Output o = Stop(StopReason::kTimeout);\n"
     "    if (true) {\n      timeout_lock_ = false;"),
    ("tier1: no command ever received looks FRESH",
     TIER1_CC, "  if (!in.has_cmd || cmd_age_s > cmd_timeout_s_) {",
     "  if (cmd_age_s > cmd_timeout_s_) {"),
    ("tier1: the timeout boundary is off by one",
     TIER1_CC, "  if (!in.has_cmd || cmd_age_s > cmd_timeout_s_) {",
     "  if (!in.has_cmd || cmd_age_s >= cmd_timeout_s_) {"),
    ("tier1: timeout read as milliseconds against a seconds clock",
     TIER1_CC, "      cmd_timeout_s_(static_cast<double>(cfg.cmd_timeout_ms) / 1000.0) {}",
     "      cmd_timeout_s_(static_cast<double>(cfg.cmd_timeout_ms)) {}"),
    # ---- the soft stop ------------------------------------------------------
    ("tier1: epoch compared with < instead of !=",
     TIER1_CC, "  if (in.cmd_estop_epoch != in.local_estop_epoch) {",
     "  if (in.cmd_estop_epoch < in.local_estop_epoch) {"),
    # ---- events -------------------------------------------------------------
    ("tier1: the timeout fault repeats every period (100 Hz storm)",
     TIER1_CC, "    if (!timeout_lock_) {\n      timeout_lock_ = true;\n"
               "      o.event_timeout_lock = true;\n    }",
     "    timeout_lock_ = true;\n    o.event_timeout_lock = true;"),
    ("tier1: the mode-mismatch event repeats every period",
     TIER1_CC, "    if (!mode_mismatch_seen_) {", "    if (true) {"),
    ("tier1: the mode-mismatch latch never resets, so only the first is seen",
     TIER1_CC, "  mode_mismatch_seen_ = false;\n\n  if (!AllFinite(in)) {",
     "  if (!AllFinite(in)) {"),
    ("tier1: the actual mode is not reported with the event",
     TIER1_CC, "      o.mode_mismatch_actual = in.usage_mode_raw;",
     "      o.mode_mismatch_actual = 0;"),
    # ---- clamp and trim -----------------------------------------------------
    ("tier1: yaw clamped against the LINEAR limit",
     TIER1_CC, "  o.wz = Clamp(Radps{in.wz}, Radps{limits_.max_wz_radps}).value;",
     "  o.wz = Clamp(Radps{in.wz}, Radps{limits_.max_vx_mps}).value;"),
    ("tier1: lateral axis not clamped",
     TIER1_CC, "  o.vy = Clamp(Mps{in.vy}, Mps{limits_.max_vy_mps}).value;",
     "  o.vy = in.vy;"),
    ("tier1: a non-holonomic chassis still gets a lateral command",
     TIER1_CC, "  if (!limits_.holonomic) {\n    o.vy = 0.0;\n  }",
     "  if (false) {\n    o.vy = 0.0;\n  }"),
    ("tier1: the unlimited axes are passed through",
     TIER1_CC, "  o.hes_lock = hes_lock_;\n  o.timeout_lock = timeout_lock_;\n"
               "  return o;\n}\n\n}  // namespace quadruped",
     "  o.vz = in.vz;\n  o.v_roll = in.v_roll;\n  o.v_pitch = in.v_pitch;\n"
     "  o.hes_lock = hes_lock_;\n  o.timeout_lock = timeout_lock_;\n"
     "  return o;\n}\n\n}  // namespace quadruped"),
    # ---- the closed-set table ----------------------------------------------
    ("tier1: stop reasons read from the table off by one",
     TIER1_CC, "  return sets::kStopReason[i];", "  return sets::kStopReason[i + 1];"),
]

# The units probe is where Clamp itself is exercised. The three mutants below
# lived briefly in the tier1 suite and all three SURVIVED there -- not because
# the assertions were missing, but because that suite compiles only
# test_tier1.cc, and Tier 1 refuses a non-finite command before Clamp ever sees
# one. A mutant is only as good as the test it is run against.
UNITS_H = os.path.join(ROOT, "common", "include", "xbrain", "units", "units.h")
UNITS_TESTS = [os.path.join(ROOT, "tests", "common", "units_cxx", "units_probe.cc")]

UNITS_MUTANTS = [
    ("units: clamp is one-sided, so the robot cannot reverse",
     UNITS_H, "  if (v.value < -limit.value) return Mps{-limit.value};\n  return v;\n}",
     "  return v;\n}"),
    ("units: a non-positive limit passes the value through",
     UNITS_H, "inline Mps Clamp(Mps v, Mps limit) {\n  if (!(limit.value > 0.0)) return Mps{0.0};",
     "inline Mps Clamp(Mps v, Mps limit) {\n  if (false) return Mps{0.0};"),
    ("units: NaN silently becomes the limit, hiding Tier 1's own branch",
     UNITS_H, "inline Mps Clamp(Mps v, Mps limit) {\n  if (!(limit.value > 0.0)) return Mps{0.0};",
     "inline Mps Clamp(Mps v, Mps limit) {\n  if (v.value != v.value) return limit;\n"
     "  if (!(limit.value > 0.0)) return Mps{0.0};"),
]

# RT key-table mutants. The failure a mistyped key produces is a process that
# starts, publishes without error, and is heard by nobody -- 13 DDS-9 calls it
# indistinguishable from a dead network, and neither side logs anything.
RT_KEYS_CC = os.path.join(QUAD, "src", "rt_keys.cc")
RT_KEYS_SOURCES = [RT_KEYS_CC]
RT_KEYS_TESTS = [os.path.join(QUAD, "test", "test_rt_keys.cc")]
CONTRACT_MD = os.path.join(ROOT, "docs", "11-接口契约.md")

RT_KEYS_MUTANTS = [
    # A key that is not in the contract. One character, and nothing hears it.
    ("rt_keys: one key suffix mistyped",
     RT_KEYS_CC, '{"rt/chassis/state", KeyRole::kPublish',
     '{"rt/chassis/stat", KeyRole::kPublish'),
    # Truncation is the same failure reached a different way: the result is a
    # well-formed key that matches nothing.
    ("rt_keys: a truncated key is returned instead of refused",
     RT_KEYS_CC, "  if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;",
     "  if (n < 0) return 0;"),
    ("rt_keys: the root segment is dropped",
     RT_KEYS_CC, '  const int n = std::snprintf(out, cap, "%s/%s/%s", kKeyRoot, rid, suffix);',
     '  const int n = std::snprintf(out, cap, "%s/%s", rid, suffix);\n  (void)kKeyRoot;'),
    # 11 F-1: one publisher per key. A role flip makes this process subscribe to
    # something it is supposed to produce, and the aggregate simply stops.
    ("rt_keys: a published key flipped to subscribe",
     RT_KEYS_CC, '{"rt/chassis/power", KeyRole::kPublish',
     '{"rt/chassis/power", KeyRole::kSubscribe'),
    # A safety key on the command profile loses the express budget 11 CRL-6
    # reserves for the stop path.
    ("rt_keys: an emergency-stop key demoted to the command profile",
     RT_KEYS_CC, '{"rt/safety/estop/ack", KeyRole::kPublish, "Q0_safety"',
     '{"rt/safety/estop/ack", KeyRole::kPublish, "Q3_cmd"'),
    # ...and the other direction: ordinary traffic promoted onto the express
    # budget, which is what PB-Q1 forbids on the rt_safety thread.
    ("rt_keys: cmd_vel promoted onto the safety profile",
     RT_KEYS_CC, '{"rt/motion/cmd_vel", KeyRole::kSubscribe, "Q1_rt"',
     '{"rt/motion/cmd_vel", KeyRole::kSubscribe, "Q0_safety"'),
    # A lookup that matches a prefix would let one key resolve to another's row.
    ("rt_keys: lookup matches on a prefix",
     RT_KEYS_CC, "    if (std::strcmp(kKeys[i].suffix, suffix) == 0) return &kKeys[i];",
     "    if (std::strncmp(kKeys[i].suffix, suffix, 12) == 0) return &kKeys[i];"),
    # Two rows for one key: the second silently shadows the first at lookup.
    ("rt_keys: a key declared twice",
     RT_KEYS_CC, '{"rt/chassis/basic", KeyRole::kPublish, "Q2_state",',
     '{"rt/chassis/motion", KeyRole::kPublish, "Q2_state",'),
]

# Payload mutants. These messages are what an operator watches, so the failures
# below are the ones that make a screen lie rather than go blank.
PAYLOADS_CC = os.path.join(QUAD, "src", "rt_payloads.cc")
PAYLOADS_SOURCES = [PAYLOADS_CC,
                    os.path.join(QUAD, "src", "chs_a_reports.cc"),
                    os.path.join(QUAD, "src", "chs_a_session.cc"),
                    os.path.join(QUAD, "src", "tier1.cc"),
                    os.path.join(QUAD, "src", "quadruped_config.cc"),
                    os.path.join(QUAD, "src", "chs_a_codec.cc")]
PAYLOADS_TESTS = [os.path.join(QUAD, "test", "test_rt_payloads.cc")]

PAYLOADS_MUTANTS = [
    # 11 D-08: four fields, four meanings. Deriving locked from one latch hides
    # the other, and an operator reads "not locked" on a robot that cannot move.
    ("payloads: locked derived from the HES latch alone",
     PAYLOADS_CC, "  a.Bool(in.tier1.hes_lock || in.tier1.timeout_lock);",
     "  a.Bool(in.tier1.hes_lock);"),
    ("payloads: locked derived from the timeout latch alone",
     PAYLOADS_CC, "  a.Bool(in.tier1.hes_lock || in.tier1.timeout_lock);",
     "  a.Bool(in.tier1.timeout_lock);"),
    # An absent report rendered as a zeroed struct reads as a healthy robot
    # standing still, which is the worst default for the message a human watches.
    ("payloads: an absent BasicStatus rendered as zeros instead of null",
     PAYLOADS_CC,
     '    a.Raw(",\\"usage_mode\\":null,\\"motion_state\\":null,\\"gait\\":null");',
     '    a.Raw(",\\"usage_mode\\":\\"normal\\",\\"motion_state\\":\\"idle\\",\\"gait\\":\\"basic\\"");'),
    # "Never received a command" is a different fact from "the command is old".
    ("payloads: a never-received command reported as an age",
     PAYLOADS_CC, "  if (in.cmd_age_ms < 0.0) {", "  if (false) {"),
    # 13 S6.5 ban 3: the raw value is what a field engineer matches to the
    # manual, and an unregistered value has no other handle at all.
    ("payloads: open-set raw value dropped, label only",
     PAYLOADS_CC, '  a->Int(static_cast<long long>(v.raw));', '  a->Int(0);'),
    # 11 S4.2 / 13 BAT-1: the MINIMUM. A maximum reports a robot as fuller than
    # its emptiest pack, which is the direction that strands it.
    ("payloads: SOC published from present_count instead of the minimum",
     PAYLOADS_CC, "  a.Int(in.device->min_level);",
     "  a.Int(static_cast<long long>(in.device->present_count));"),
    # 13 BAT-2: array order is not a measurement. Filling left/right from it is
    # the failure CLAUDE.md 3.2 calls a guess presented as a measurement, and
    # BAT-4 forbids even saying "left" in that state.
    ("payloads: left/right filled from array order while the mapping is unknown",
     PAYLOADS_CC, "  if (!in.index_map_known) {\n    a.Raw(\"null\");",
     "  if (false) {\n    a.Raw(\"null\");"),
    ("payloads: a mapping pointing outside the array is used anyway",
     PAYLOADS_CC,
     "    if (!ok) {\n      // A configured mapping that points outside the array is a configuration",
     "    if (false) {\n      // A configured mapping that points outside the array is a configuration"),
    # An unregistered power_management mapped onto one of the two known values
    # is the same fail-open the mode fields refuse.
    ("payloads: an unregistered power_management mapped to normal",
     PAYLOADS_CC, '  } else if (in.basic->power_management == 1) {',
     '  } else if (in.basic->power_management >= 1) {'),
    # CF-5: the prefix travels with the code. Without it the two overlapping
    # code spaces cannot be told apart at all.
    ("payloads: fault code published without its namespace prefix",
     PAYLOADS_CC, "      a.Str(f.code.c_str());",
     "      a.Str(f.code.size() > 4 ? f.code.c_str() + 4 : f.code.c_str());"),
    # Truncation: half an object decodes to the wrong thing, or to nothing.
    ("payloads: a truncated object is returned instead of refused",
     PAYLOADS_CC, "    if (overflow_) return 0;", "    if (false) return 0;"),
    ("payloads: overflow not sticky, so later fields hide an earlier loss",
     PAYLOADS_CC, "    if (len_ + n + 1 > cap_) {\n      overflow_ = true;\n      return;\n    }",
     "    if (len_ + n + 1 > cap_) {\n      return;\n    }"),
    # An unescaped quote produces text no decoder accepts, and the symptom is
    # state/robot going silent -- which reads as the robot having died.
    ("payloads: JSON string escaping removed",
     PAYLOADS_CC, '        case \'"\': Raw("\\\\\\""); break;',
     '        case \'"\': Raw("\\""); break;'),
    # The pong seq is ECHOED; a self-counted one answers a question nobody asked.
    ("payloads: pong seq replaced by a constant",
     PAYLOADS_CC, '  a.Raw("{\\"type\\":\\"pong\\",\\"seq\\":");\n  a.UInt(in.seq);',
     '  a.Raw("{\\"type\\":\\"pong\\",\\"seq\\":");\n  a.UInt(0);'),
    # 13 Q-2 makes detail.action required; without it an ack cannot be matched
    # to the command it answers.
    ("payloads: ctrl ack drops detail.action",
     PAYLOADS_CC, '  a.Str(in.action);', '  a.Str("");'),
]

# Mode-machine mutants. Three of the rules below are the opposite of the
# obvious implementation, and each mutant is that obvious implementation.
MODE_CC = os.path.join(QUAD, "src", "mode_machine.cc")
MODE_SOURCES = [MODE_CC, os.path.join(QUAD, "src", "chs_a_reports.cc"),
                os.path.join(QUAD, "src", "chs_a_codec.cc")]
MODE_TESTS = [os.path.join(QUAD, "test", "test_mode_machine.cc")]

MODE_MUTANTS = [
    # The two measured surprises. Expecting 1 after a stand times out on every
    # SUCCESSFUL stand, and the fault it raises points at the chassis.
    ("mode: a stand expects motion_state 1 instead of 17",
     MODE_CC, "      e.motion_state = kMotionStateStandSteady;",
     "      e.motion_state = 1;"),
    ("mode: a prone expects 4, which is only a waypoint",
     MODE_CC, "      e.motion_state = kMotionStateProneSteady;",
     "      e.motion_state = 4;"),
    # MS-5: a gait switch moves the motion mode too.
    ("mode: only the commanded field is compared",
     MODE_CC, "    if (t == expect_) {", "    if (t.gait == expect_.gait) {"),
    # MS-6 / TR-3: an unchanged read-back is expected, not a failure.
    ("mode: an unchanged read-back judged as a failed switch",
     MODE_CC, "    // MS-6 / TR-3: a read-back still holding the old value is expected during",
     "    if (t != expect_) { switching_ = false; ++switch_failures_; }\n"
     "    // MS-6 / TR-3: a read-back still holding the old value is expected during"),
    # MS-2 and its boundary.
    ("mode: the switch timeout never fires",
     MODE_CC, "  if (now_mono_s - switch_started_s_ <= cfg_.switch_timeout_s) return false;",
     "  return false;\n  if (now_mono_s - switch_started_s_ <= cfg_.switch_timeout_s) return false;"),
    ("mode: the failure is reported on every tick, not once",
     MODE_CC, "  switching_ = false;\n  switch_started_s_ = -1.0;\n  ++switch_failures_;\n  return true;",
     "  ++switch_failures_;\n  return true;"),
    # MS-1: the window opens at SEND time.
    ("mode: the switch window opens on the first read-back instead",
     MODE_CC, "  switch_started_s_ = now_mono_s;\n  // Our own switch supersedes",
     "  switch_started_s_ = now_mono_s + cfg_.switch_timeout_s;\n  // Our own switch supersedes"),
    # MS-3.
    ("mode: a second switch accepted while one is in flight",
     MODE_CC, "  if (switching_) {\n    // MS-3: a second switch", "  if (false) {\n    // MS-3: a second switch"),
    # PR-1, both directions.
    ("mode: prone allowed on a stair gait",
     MODE_CC, "  return !Contains(cfg_.prone_forbidden_gaits, steady_.gait);",
     "  return true;"),
    ("mode: prone allowed before any read-back",
     MODE_CC, "  if (!has_readback_) {\n    // Nothing has been read back yet",
     "  if (false) {\n    // Nothing has been read back yet"),
    # TR-2: the pre-check judges the STEADY value.
    ("mode: the prone pre-check judges the instantaneous read-back",
     MODE_CC, "  return !Contains(cfg_.prone_forbidden_gaits, steady_.gait);",
     "  return !Contains(cfg_.prone_forbidden_gaits, last_.gait);"),
    # GS-1.
    ("mode: the uncommandable gait is sent anyway",
     MODE_CC, "  return !Contains(cfg_.command_forbidden_gaits, gait);",
     "  return true;"),
    # TR-1 and its hold.
    ("mode: an external transition is not noticed",
     MODE_CC, "  if (changed) {\n    external_change_s_ = now_mono_s;\n  }",
     "  (void)changed;"),
    ("mode: the external hold never expires",
     MODE_CC,
     "  if (external_change_s_ >= 0.0 &&\n"
     "      now_mono_s - external_change_s_ > cfg_.external_transition_hold_s) {",
     "  if (false) {"),
    ("mode: the FIRST read-back treated as an external transition",
     MODE_CC, "  const bool changed = !first && t != last_;",
     "  const bool changed = t != last_;"),
    # Our own switch supersedes the hold; otherwise the hold outlives a switch
    # we CAN see finish.
    ("mode: an external hold survives our own switch",
     MODE_CC, "  external_change_s_ = -1.0;\n  return r;", "  return r;"),
]

# Odometry mutants. A covariance that is too small is a robot that believes it
# knows where it is, so most of these push in the optimistic direction -- the
# one 13 S4.4's own superseded formula went in.
ODOM_CC = os.path.join(QUAD, "src", "odometry.cc")
ODOM_SOURCES = [ODOM_CC, os.path.join(QUAD, "src", "quadruped_config.cc"),
                os.path.join(QUAD, "src", "chs_a_codec.cc")]
ODOM_TESTS = [os.path.join(QUAD, "test", "test_odometry.cc")]

ODOM_MUTANTS = [
    # The a_max term is the whole quantitative argument behind ODO-1: without
    # it, a sample a tenth of a second old looks as good as a fresh one.
    ("odom: sigma_v loses its a_max term",
     ODOM_CC, "  return std::sqrt(cfg_.sigma_v0_mps * cfg_.sigma_v0_mps + at * at);",
     "  (void)at;\n  return cfg_.sigma_v0_mps;"),
    # The per-tick form the document used to carry: 2.45x optimistic.
    ("odom: the open interval term dropped from the published variance",
     ODOM_CC, "  s.var_x = (p_xx_committed_ + open * open) / divisor;",
     "  s.var_x = p_xx_committed_ / divisor;"),
    ("odom: the closed interval accounted with dt instead of the whole tau",
     ODOM_CC, "      const double inc = SigmaV(tau_used) * tau_used;",
     "      const double inc = SigmaV(tau_used) * 0.01;"),
    # The yaw correlation factor, mistaken for a unit conversion.
    ("odom: the yaw correlation factor removed",
     ODOM_CC, "                  kYawCorrelationFactor;", "                  1.0;"),
    ("odom: the angle random walk term dropped",
     ODOM_CC, "    p_yaw_ += (cfg_.arw_rad_sqrt_s * cfg_.arw_rad_sqrt_s * dt_s) +",
     "    p_yaw_ += (0.0 * dt_s) +"),
    # ODO-4: the yaw model must not inherit the linear sample's age.
    ("odom: the yaw-rate variance grows with the LINEAR sample age",
     ODOM_CC, "  s.var_wz = (cfg_.gyro_bias_radps * cfg_.gyro_bias_radps) / divisor;",
     "  s.var_wz = (sv * sv) / divisor;"),
    # The bands.
    ("odom: the stop band never reached, so a dead TF keeps being published",
     ODOM_CC, "  if (tau_ms > cfg_.stale_stop_publish_ms) {", "  if (false) {"),
    ("odom: the twist-zero band still publishes the held velocity",
     ODOM_CC, "  if (s.band == OdomBand::kFresh || s.band == OdomBand::kWarn) {\n"
              "    const double c = std::cos(yaw_);",
     "  if (s.band != OdomBand::kStop) {\n    const double c = std::cos(yaw_);"),
    ("odom: a band boundary is off by one",
     ODOM_CC, "  } else if (tau_ms > cfg_.stale_invalid_ms) {",
     "  } else if (tau_ms >= cfg_.stale_invalid_ms) {"),
    ("odom: yaw stops integrating when the LINEAR sample goes stale",
     ODOM_CC, "  if (s.band != OdomBand::kStop) {\n    yaw_ += wz_ * dt_s;",
     "  if (s.band == OdomBand::kFresh) {\n    yaw_ += wz_ * dt_s;"),
    # Never sampled must be the stop band, not a fresh standstill.
    ("odom: a never-sampled velocity reads as fresh",
     ODOM_CC, "                         : 1.0e9;  // never sampled: unboundedly stale",
     "                         : 0.0;  // never sampled: unboundedly stale"),
    # The stair gait: 11 S9.9 wants both the inflation and the invalidation.
    ("odom: a stair gait inflates but stays valid",
     ODOM_CC, "  s.valid = s.publish && !is_stair_gait_ &&",
     "  s.valid = s.publish &&"),
    ("odom: the gait trust factor is not applied",
     ODOM_CC, "  const double t = is_stair_gait_ ? cfg_.trust_stair : cfg_.trust_flat;",
     "  const double t = cfg_.trust_flat;"),
    # REP-105: a negative variance means "not provided"; zero claims certainty.
    ("odom: an unestimated axis reports variance 0 instead of -1",
     ODOM_CC, "  s.var_y = holonomic_ ? s.var_x : -1.0;",
     "  s.var_y = holonomic_ ? s.var_x : 0.0;"),
    # The dead zones.
    ("odom: the velocity dead zone removed, so a standing robot drifts",
     ODOM_CC, "  vx_ = (std::fabs(vx) < cfg_.vel_deadzone_mps) ? 0.0 : vx;",
     "  vx_ = vx;"),
    ("odom: the gyro dead zone removed",
     ODOM_CC, "  wz_ = (std::fabs(wz) < cfg_.gyro_deadzone_radps) ? 0.0 : wz;",
     "  wz_ = wz;"),
    # The pose is integrated in the odom frame.
    ("odom: the body-to-odom rotation dropped",
     ODOM_CC, "    x_ += (c * vx_ - sn * vy_) * dt_s;\n    y_ += (sn * vx_ + c * vy_) * dt_s;",
     "    x_ += vx_ * dt_s;\n    y_ += vy_ * dt_s;"),
]

# Name-mapping mutants. A wrong mapping produces a participant that comes up, a
# topic that exists and not one sample -- 13 DDS-9 records that as
# indistinguishable from a dead network, so there is nothing to observe at run
# time and the assertion has to live here.
NAMES_CC = os.path.join(QUAD, "src", "dds_names.cc")
NAMES_SOURCES = [NAMES_CC]
NAMES_TESTS = [os.path.join(QUAD, "test", "test_dds_names.cc")]

NAMES_MUTANTS = [
    ("dds_names: the rt/ prefix dropped",
     NAMES_CC, '  return std::string(kRosTopicPrefix) + ros_topic.substr(1);',
     '  return ros_topic.substr(1);'),
    ("dds_names: the leading slash kept as well as the prefix",
     NAMES_CC, '  return std::string(kRosTopicPrefix) + ros_topic.substr(1);',
     '  return std::string(kRosTopicPrefix) + ros_topic;'),
    # An empty or relative name maps to a valid topic nothing publishes, so the
    # operator is told nothing at all.
    ("dds_names: a relative topic accepted",
     NAMES_CC, "  if (ros_topic.size() < 2 || ros_topic[0] != '/') {",
     "  if (false) {"),
    ("dds_names: a trailing slash accepted",
     NAMES_CC, "  if (ros_topic[ros_topic.size() - 1] == '/') {", "  if (false) {"),
    # never-seen and stale have different causes and different remedies.
    ("dds_names: never-seen folded into stale",
     NAMES_CC, "  if (age_s < 0.0) return ImuFreshness::kNeverSeen;",
     "  if (age_s < 0.0) return ImuFreshness::kStale;"),
    ("dds_names: the freshness boundary is off by one",
     NAMES_CC, "  if (age_s * 1000.0 > static_cast<double>(warn_ms)) return ImuFreshness::kStale;",
     "  if (age_s * 1000.0 >= static_cast<double>(warn_ms)) return ImuFreshness::kStale;"),
]

# Socket mutants. Every one of these compiles, connects, and then behaves
# wrongly in a way that reads as a chassis problem rather than a code problem.
SOCKET_CC = os.path.join(QUAD, "src", "chassis_socket.cc")
SOCKET_SOURCES = [SOCKET_CC, os.path.join(QUAD, "src", "quadruped_config.cc"),
                  os.path.join(QUAD, "src", "chs_a_codec.cc")]
SOCKET_TESTS = [os.path.join(QUAD, "test", "test_chassis_socket.cc")]

SOCKET_MUTANTS = [
    # EINPROGRESS is the NORMAL answer for a non-blocking TCP connect. Treating
    # it as failure rejects every TCP candidate on a healthy network, and the
    # operator is told the chassis is not answering.
    ("socket: EINPROGRESS treated as a connect failure",
     SOCKET_CC, "    if (errno != EINPROGRESS) {", "    if (true) {"),
    # 13 TLS-5: a downgrade that happens by itself can be forced.
    ("socket: a TLS candidate silently dialled in plaintext",
     SOCKET_CC, "  if (ep.tls) {", "  if (false) {"),
    # CA-1: two live sockets are two clients, and axis commands come back
    # 0xE006 for two seconds -- accepted, and the robot does not move.
    ("socket: the previous socket left open when redialling",
     SOCKET_CC, "  Close();\n  last_error_ = DialError::kNone;",
     "  last_error_ = DialError::kNone;"),
    # EAGAIN is a short write, not a broken link.
    ("socket: EAGAIN on send reported as an error",
     SOCKET_CC, "  if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;\n  return -1;\n}\n\nlong ChassisSocket::Recv",
     "  return -1;\n}\n\nlong ChassisSocket::Recv"),
    # ...and on the read side, EAGAIN as -1 turns every idle moment into a
    # reconnect. 13 CA-6 makes each reconnect play a voice prompt on the robot.
    ("socket: nothing-to-read reported as a dead link",
     SOCKET_CC, "  if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;\n  return -1;\n}\n\nbool ChassisSocket::nodelay_enabled",
     "  return -1;\n}\n\nbool ChassisSocket::nodelay_enabled"),
    # The option is READ BACK, not remembered: setsockopt can be refused, and a
    # remembered flag records what was asked for rather than what is in force.
    ("socket: nodelay_enabled reports the request instead of the socket",
     SOCKET_CC, "  if (::getsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &v, &len) != 0) return false;\n  return v != 0;",
     "  (void)v; (void)len;\n  return true;"),
    # A TCP zero-length read means the peer closed; a UDP one does not.
    ("socket: a TCP peer closing reported as nothing-to-read",
     SOCKET_CC, "    return is_udp_ ? 0 : -1;", "    return 0;"),
    # FR-5 / SD-3: with Nagle on, "when did the last frame leave" has no answer.
    ("socket: TCP_NODELAY not set",
     SOCKET_CC, "    ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));",
     "    (void)one;"),
    # Without MSG_NOSIGNAL a write to a closed socket ENDS THE PROCESS, and a
    # test would not report a failure -- it would vanish.
    ("socket: MSG_NOSIGNAL dropped, so a closed peer kills the process",
     SOCKET_CC, "  const ssize_t n = ::send(fd_, data, len, MSG_NOSIGNAL);",
     "  const ssize_t n = ::send(fd_, data, len, 0);"),
    # An unparseable host must be refused, not turned into a connect to 0.0.0.0.
    ("socket: an invalid address accepted",
     SOCKET_CC, '  if (::inet_pton(AF_INET, ep.host.c_str(), &addr.sin_addr) != 1) {',
     "  if (false) {"),
    ("socket: an unknown protocol treated as TCP",
     SOCKET_CC, "    last_error_ = DialError::kUnsupportedProto;\n    return false;",
     "    is_udp_ = false;"),
]

# Envelope mutants. The unit was wrong here for two days and no test turned
# red, because the existing case asserted only ts_sync semantics: the unit was
# an assumption, not an assertion. These are what make it an assertion.
ENVELOPE_H = os.path.join(ROOT, "common", "include", "xbrain", "envelope",
                          "envelope_writer.h")
ENVELOPE_TESTS = [os.path.join(ROOT, "tests", "common", "envelope",
                               "envelope_units_probe.cc")]

ENVELOPE_MUTANTS = [
    # The defect itself, both halves. 11 S3.0 calls mono the ONLY basis for
    # every timeout and age judgement, so a thousandfold error there is not a
    # cosmetic one.
    ("envelope: ts stamped in milliseconds again",
     ENVELOPE_H, "    env.ts = wall_ts_s;", "    env.ts = wall_ts_s * 1000.0;"),
    ("envelope: mono stamped in milliseconds again",
     ENVELOPE_H, "    env.mono = now_mono_s;", "    env.mono = now_mono_s * 1000.0;"),
    # Rendering. An integer format loses the sub-second part the Qt spec freezes.
    ("envelope: ts rendered as an integer",
     ENVELOPE_H, '\\"ts\\":%.6f', '\\"ts\\":%.0f'),
    ("envelope: ts rendered as a quoted string",
     ENVELOPE_H, '\\"ts\\":%.6f', '\\"ts\\":\\"%.6f\\"'),
    # Truncation. Half an envelope is valid-looking JSON that decodes wrong.
    ("envelope: a truncated object is returned instead of refused",
     ENVELOPE_H, "  if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;",
     "  if (n < 0) return 0;"),
    # CLK-A3 and its boundary.
    ("envelope: the sync window is off by one at the boundary",
     ENVELOPE_H, "    if (age_s >= sync_timeout_s_) {", "    if (age_s > sync_timeout_s_) {"),
    ("envelope: ts_sync defaults true before any ClockStatus",
     ENVELOPE_H, "    if (!clock_received_) {\n      return false;",
     "    if (!clock_received_) {\n      return true;"),
    # PB-Q3: one seq source per producer.
    ("envelope: seq does not advance",
     ENVELOPE_H, "    env.seq = ++seq_;", "    env.seq = seq_;"),
]

# Assembly mutants. These are the ones that matter most and the ones a unit
# test of any single layer cannot reach: every layer below is already covered,
# and each of these breaks the WIRING between two correct layers. The defect
# that was actually found while writing this suite's test -- the session ticked
# before the arriving report was taken, so the link could never come up -- was
# exactly that shape, and no test of the session or the framer could have seen
# it.
PROCESS_CC = os.path.join(QUAD, "src", "process.cc")
PROCESS_SOURCES = [
    PROCESS_CC,
    os.path.join(QUAD, "src", "chassis_socket.cc"),
    os.path.join(QUAD, "src", "chs_a_codec.cc"),
    os.path.join(QUAD, "src", "chs_a_framer.cc"),
    os.path.join(QUAD, "src", "chs_a_reports.cc"),
    os.path.join(QUAD, "src", "chs_a_session.cc"),
    os.path.join(QUAD, "src", "mode_machine.cc"),
    os.path.join(QUAD, "src", "odometry.cc"),
    os.path.join(QUAD, "src", "quadruped_config.cc"),
    os.path.join(QUAD, "src", "tier1.cc"),
    os.path.join(QUAD, "src", "tx_owner.cc"),
]
PROCESS_TESTS = [os.path.join(QUAD, "test", "test_process.cc")]

PROCESS_MUTANTS = [
    # 13 CA-7: an arriving report is the ONLY evidence the link is alive,
    # because axis commands are never acknowledged. Drop the call and the
    # session never learns it has a peer -- the socket is open, frames are
    # being parsed, and the link is declared lost anyway.
    ("process: the arriving report never reaches the session (CA-7)",
     PROCESS_CC, "    session_.OnReport(fresh.rx_mono_s);", "    (void)0;"),
    # The defect this suite's test found. Ticking the session first makes every
    # report one period late, and a report landing just inside the lost timeout
    # is ignored with the evidence sitting unread in the slot.
    ("process: the session is ticked before the snapshot is taken",
     PROCESS_CC,
     "  ChassisSnapshot fresh;\n  if (snapshot_slot_.TakeFresh(&fresh)) {",
     "  const chs_a::TickResult early = session_.Tick(now_mono_s);\n"
     "  (void)early;\n"
     "  ChassisSnapshot fresh;\n  if (snapshot_slot_.TakeFresh(&fresh)) {"),
    # 13 F-21. The sleep flag is the chassis saying it will ignore motion; a
    # process that reports it as awake commands a robot that is not listening,
    # and the symptom is a command stream with no movement.
    ("process: the sleep readback is reported as awake (F-21)",
     PROCESS_CC, "      session_.OnSleep(fresh.sleep);",
     "      session_.OnSleep(false);"),
    # The HES bit travels in the basic report. Losing it between the parser and
    # Tier 1 removes the one stop that software cannot clear.
    ("process: HES dropped between the report and Tier 1",
     PROCESS_CC, "    in.hes_raw = latest_.hes;", "    in.hes_raw = false;"),
    # 13 S9.12.2 (3): the hold is a GENERATION comparison. Feeding Tier 1 the
    # upstream's own generation as our own makes them equal by construction,
    # and the soft stop releases itself on the next period.
    ("process: the estop generation compared against itself",
     PROCESS_CC, "  in.local_estop_epoch = estop_epoch_;",
     "  in.local_estop_epoch = cmd_estop_epoch_;"),
    # T-1 budgets 5 ms. Advancing the generation without sending leaves the
    # robot travelling until the next period -- up to 10 ms at whatever speed
    # it had, and nothing in the process reports the difference.
    ("process: the soft stop waits for the next control period (T-1)",
     PROCESS_CC,
     "    const TxResult r = tx_.Send(TxCaller::kNonRealtime, buf, n);\n"
     "    if (r == TxResult::kSent) ++axis_frames_sent_;",
     "    (void)buf; (void)n;"),
    ("process: the soft stop does not advance the generation",
     PROCESS_CC, "  ++estop_epoch_;", "  /* not advanced */"),
    # 13 S2.2 / CA-2: the chassis reports only to an address that keeps sending
    # heartbeats. Without them the link comes up once and goes quiet, which
    # reads as a chassis that stopped reporting.
    ("process: the heartbeat is never sent (CA-2)",
     PROCESS_CC, "  if (link.send_heartbeat) {", "  if (false) {"),
    # The two gates of step 4, taken apart one at a time. Tier 1 answers "is
    # this command safe"; the session answers "is there a link to put it on".
    ("process: the axis command skips the Tier 1 gate",
     PROCESS_CC,
     "  if (last_tier1_.stop_reason == StopReason::kNone && session_.motion_allowed()) {",
     "  if (session_.motion_allowed()) {"),
    ("process: the axis command skips the session gate",
     PROCESS_CC,
     "  if (last_tier1_.stop_reason == StopReason::kNone && session_.motion_allowed()) {",
     "  if (last_tier1_.stop_reason == StopReason::kNone) {"),
    # A Tier 1 verdict that is computed and then not used. The frame carries
    # the RAW command, so every limit and every clamp is bypassed while the
    # stop reasons still read correctly from outside.
    ("process: the raw command is sent instead of the Tier 1 output",
     PROCESS_CC,
     "    axis.vx = last_tier1_.vx;\n    axis.vy = last_tier1_.vy;\n"
     "    axis.yaw = last_tier1_.wz;",
     "    axis.vx = cmd_vx_;\n    axis.vy = cmd_vy_;\n    axis.yaw = cmd_wz_;"),
    # 11 S9.12.1: the enable is an EVENT. Latching it re-clears the lock on
    # every period that follows, so the lock holds exactly once and never
    # again -- and nothing about the first unlock looks different.
    ("process: the operator enable is latched instead of consumed",
     PROCESS_CC, "  enable_pending_ = false;", "  /* left set */"),
    ("process: the operator enable never reaches Tier 1",
     PROCESS_CC, "  in.enable_requested = enable_pending_;",
     "  in.enable_requested = false;"),
    # The motion report is the odometry's only linear source until the domain-0
    # reader is wired. Losing it leaves dead reckoning integrating zeros, and
    # the pose stays put while the robot walks away.
    ("process: the velocity sample never reaches the odometry",
     PROCESS_CC,
     "      odom_.OnVelocitySample(fresh.rx_mono_s, fresh.linear_x, fresh.linear_y);",
     "      (void)0;"),
    # The counter the whole receive path is judged by.
    ("process: received frames are not counted",
     PROCESS_CC, "    ++frames_received_;", "    /* not counted */"),
]

# name -> (sources, test files, mutants, argv[1] passed to each test).
# `sources` are compiled into every test of the suite; a header-only module
# lists none. The argument differs per suite because the tests need different
# things: the CHS-A tests read the golden capture, the config test needs a
# WRITABLE DIRECTORY for its fixtures. Passing one to the other is not a
# no-op -- the config test would try to write fixtures inside a file path.
SUITES = {
    "quadruped": (QUAD_SOURCES, QUAD_TESTS, QUAD_MUTANTS, GOLDEN),
    "quadruped_config": (CONFIG_SOURCES, CONFIG_TESTS, CONFIG_MUTANTS, None),
    "reports": (REPORTS_SOURCES, REPORTS_TESTS, REPORTS_MUTANTS, GOLDEN),
    "session": (SESSION_SOURCES, SESSION_TESTS, SESSION_MUTANTS, None),
    "tier1": (TIER1_SOURCES, TIER1_TESTS, TIER1_MUTANTS, None),
    "units": ([], UNITS_TESTS, UNITS_MUTANTS, None),
    "envelope": ([], ENVELOPE_TESTS, ENVELOPE_MUTANTS, None),
    "rt_keys": (RT_KEYS_SOURCES, RT_KEYS_TESTS, RT_KEYS_MUTANTS, CONTRACT_MD),
    "payloads": (PAYLOADS_SOURCES, PAYLOADS_TESTS, PAYLOADS_MUTANTS, None),
    "mode": (MODE_SOURCES, MODE_TESTS, MODE_MUTANTS, None),
    "odom": (ODOM_SOURCES, ODOM_TESTS, ODOM_MUTANTS, None),
    "dds_names": (NAMES_SOURCES, NAMES_TESTS, NAMES_MUTANTS, None),
    "socket": (SOCKET_SOURCES, SOCKET_TESTS, SOCKET_MUTANTS, None),
    "process": (PROCESS_SOURCES, PROCESS_TESTS, PROCESS_MUTANTS, GOLDEN),
    "yaml_lite": ([], YAML_TESTS, YAML_MUTANTS, None),
}


def build_and_run(sources, tests, workdir, arg, quiet=True):
    """Compile and run every test of one suite.

    Returns (compiled, passed). The two are separate because a mutant that does
    not compile has NOT been caught by any assertion, and reporting it as a
    kill would credit the tests with work they did not do.
    """
    for test_src in tests:
        exe = os.path.join(workdir, os.path.basename(test_src)[:-3])
        cmd = [
            "g++", "-std=c++17", "-w",
            "-I", os.path.join(ROOT, "common", "include"),
            "-I", os.path.join(ROOT, "common", "third_party"),
            "-I", os.path.join(QUAD, "include"),
            "-o", exe, test_src,
        ] + sources
        cc = subprocess.run(cmd, capture_output=True, text=True)
        if cc.returncode != 0:
            if not quiet:
                sys.stderr.write(cc.stderr)
            return (False, False)
        # argv[1] is the suite's own: the golden capture, or the work
        # directory for a suite whose tests write fixtures. A hung framer (a
        # resync that fails to advance, say) must not hang the run: without the
        # timeout it is indistinguishable from a slow pass.
        try:
            run = subprocess.run([exe, arg if arg else workdir],
                                 capture_output=True, text=True, timeout=60)
        except subprocess.TimeoutExpired:
            return (True, False)
        if run.returncode != 0:
            if not quiet:
                sys.stdout.write(run.stdout)
            return (True, False)
    return (True, True)


def run_suite(name, workdir):
    """Run one suite. Returns (killed, survived[], broken[]) or None if the
    baseline is red -- in which case nothing below it means anything."""
    sources, tests, mutants, arg = SUITES[name]
    # Every file a mutant may touch is backed up, sources and headers alike.
    paths = sorted({m[1] for m in mutants})
    backups = {}
    for path in paths:
        with open(path, "r", encoding="utf-8") as f:
            backups[path] = f.read()

    def restore():
        for pth, text in backups.items():
            with open(pth, "r", encoding="utf-8") as f:
                if f.read() == text:
                    continue
            with open(pth, "w", encoding="utf-8") as f:
                f.write(text)

    # Registered as well as called in the finally below: an interrupt between
    # writing a mutant and restoring it would otherwise leave a mutated file in
    # the working tree, a silent candidate for the next commit.
    #
    # atexit alone is NOT enough, measured 2026-09-16: it runs on a normal exit
    # and on KeyboardInterrupt, and NOT on SIGTERM -- which is what pkill and
    # every job supervisor send. A run killed that way left the HES mutant in
    # process.cc, and because that file was still untracked `git status` had
    # nothing to say about it. The next build compiled the mutant, one test went
    # red, and the cause looked like anything but a mutation. install_guards()
    # turns the signal into a normal exit so this restore runs.
    atexit.register(restore)

    survived, broken, killed = [], [], 0
    try:
        ok, passed = build_and_run(sources, tests, workdir, arg, quiet=False)
        if not ok or not passed:
            print("  BASELINE IS NOT GREEN -- every mutant below would be "
                  "reported killed for the wrong reason.")
            return None
        print("  baseline: builds and passes")

        for entry in mutants:
            desc, path, old, new = entry[0], entry[1], entry[2], entry[3]
            expect = entry[4] if len(entry) > 4 else "killed"
            text = backups[path]
            hits = text.count(old)
            if hits != 1:
                # An anchor that stopped matching is a broken mutant, and a
                # broken mutant that is silently skipped reads exactly like a
                # mutant that was killed.
                broken.append((desc, "anchor matched %d times, need 1" % hits))
                print("    ANCHOR   %s" % desc)
                continue
            with open(path, "w", encoding="utf-8") as f:
                f.write(text.replace(old, new))
            try:
                compiled, still_passes = build_and_run(sources, tests, workdir, arg)
            finally:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(text)
            if not compiled:
                broken.append((desc, "does not compile"))
                print("    NOBUILD  %s" % desc)
            elif still_passes:
                if expect == "equivalent":
                    killed += 1  # behaved as declared
                    print("    equiv.   %s" % desc)
                else:
                    survived.append(desc)
                    print("    SURVIVED %s" % desc)
            elif expect == "equivalent":
                # The note in the source says nothing can catch this line, and
                # something just did. Either the note is wrong or the code
                # changed underneath it; both need a human.
                broken.append((desc, "declared equivalent but was KILLED -- "
                                     "the note in the source is now wrong"))
                print("    UNEXPECTED-KILL %s" % desc)
            else:
                killed += 1
                print("    killed   %s" % desc)
    finally:
        restore()
    return (killed, survived, broken)


def install_guards():
    """Make SIGTERM and SIGHUP exit the way SIGINT does, so atexit runs.

    Without this a killed run leaves whatever mutant was on disk at that moment
    in the working tree. That is worse than a crash: the file still compiles,
    and the failure it produces points at the test rather than at the mutation.
    """
    def die(signum, _frame):
        # SystemExit rather than os._exit: it unwinds, which is what runs the
        # finally blocks and the atexit handlers that put the sources back.
        raise SystemExit(128 + signum)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, die)


def verify_clean():
    """Check that no mutant is left in the tree. Returns the number found.

    This exists because `git status` cannot answer the question: a file that is
    not tracked yet -- every new module is, for one batch -- shows no diff no
    matter what was written into it. The anchors are the check that works
    regardless: every mutant's ORIGINAL text must be present exactly once, and
    a missing one means either a leftover mutation or an anchor that has rotted
    (the two need different fixes, so they are reported apart).
    """
    leftovers, rotted = [], []
    for name in sorted(SUITES):
        for entry in SUITES[name][2]:
            desc, path, old, new = entry[0], entry[1], entry[2], entry[3]
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
            if text.count(old) == 1:
                continue
            if new in text:
                leftovers.append((name, desc, path))
            else:
                rotted.append((name, desc, path))
    for name, desc, path in leftovers:
        print("  MUTANT LEFT IN TREE  [%s] %s\n    -> %s" % (name, desc, path))
    for name, desc, path in rotted:
        print("  ANCHOR NO LONGER MATCHES  [%s] %s\n    -> %s" % (name, desc, path))
    print("criterion: every mutant's original text present exactly once")
    print("anchors: %d checked, %d left mutated, %d rotted"
          % (sum(len(SUITES[n][2]) for n in SUITES), len(leftovers), len(rotted)))
    return len(leftovers) + len(rotted)


def main(argv):
    install_guards()
    if len(argv) > 1 and argv[1] == "--verify":
        return 1 if verify_clean() else 0
    names = argv[1:] if len(argv) > 1 else sorted(SUITES)
    for name in names:
        if name not in SUITES:
            print("unknown suite: %s (have: %s)" % (name, ", ".join(sorted(SUITES))))
            return 2

    workdir = tempfile.mkdtemp(prefix="cxx_mut_")
    killed_all, survived_all, broken_all = 0, [], []
    baseline_red = []
    try:
        for name in names:
            print("suite %s" % name)
            result = run_suite(name, workdir)
            if result is None:
                baseline_red.append(name)
                continue
            killed, survived, broken = result
            killed_all += killed
            survived_all += [(name, d) for d in survived]
            broken_all += [(name, d, w) for d, w in broken]
    finally:
        shutil.rmtree(workdir, ignore_errors=True)

    print("")
    print("mutants: %d killed, %d survived, %d unusable"
          % (killed_all, len(survived_all), len(broken_all)))
    for name, desc in survived_all:
        print("  SURVIVED: [%s] %s" % (name, desc))
        print("    -> either an assertion is missing, or this is an equivalent")
        print("       mutant, in which case say so in the code (CLAUDE.md 7.2.1)")
    for name, desc, why in broken_all:
        print("  UNUSABLE: [%s] %s (%s)" % (name, desc, why))
    for name in baseline_red:
        print("  BASELINE RED: %s -- fix the build or the tests first" % name)
    return 0 if (not survived_all and not broken_all and not baseline_red) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
