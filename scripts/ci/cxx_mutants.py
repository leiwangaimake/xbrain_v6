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

Suites, each a (sources, tests, mutants) triple:
  quadruped  the CHS-A codec and framer (batch B1)
  yaml_lite  the shared config reader in common/, whose tests live with the
             sensor package -- it is header-only, so the header IS the source

No counts are written into any document (CLAUDE.md 3.7). Run it:
  python3 scripts/ci/cxx_mutants.py [suite ...]     (no args = every suite)
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


def main(argv):
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
