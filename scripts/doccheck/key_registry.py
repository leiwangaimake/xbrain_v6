#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: key_registry.py
Brief: INF-ZN-4 -- extract the Zenoh key registry from 11 S2.2.1~S2.2.9 tables
       and cross-check against the S2.2.14 statistics table (KT-1)

Description:
Why this exists. 11 KT-1 is verbatim "the numbers in this table no longer grow
by hand -- they are computed by a script from the body of S2.2.1~S2.2.9 tables,
and CI fails when they disagree" (S2.2.14). This is that script. Without it,
KT-1 is an unbacked claim: an intent added to S2.2.3 but not tallied in S2.2.14
would ship, and WL-G1 has already ruled that P2/P4 whitelists (25-30 keys each)
are generated from THIS extractor's output rather than hand-copied.

What extracts, and the three official counting rules from S2.2.14 verbatim:
  1. only 独立登记项 counted -- the S2.2.5 "对照表" subtable is comparison
     material and does NOT count.
  2. 墓碑 rows (~~key~~, entries with the strike marker) do NOT count.
  3. keys with a variable segment ({domain} / {severity} / {channel} / ...)
     count as 1 PATTERN; the instance count expands via the S2.2.11 closed
     sets.

The extractor emits per-pattern records to JSON; the checker recomputes each
group's (pattern count, instance count) and diffs against the numbers KT-1
declares. When they disagree, exit(1) with the delta named -- the caller then
fixes the table body or updates KT-1 (deliberate: a growth without a KT-1
update is exactly what the metrology exists to catch).

Two known doc-drift finds this catches on the current tree (registered, not
suppressed): the S2.2.14 table's `event/` row is v0.6 (writes 2 patterns /
2 instances), but S2.2.5 was augmented in v0.7.10 with three CR-EVT-1 keys
(event/replay/{channel}, event/recon/req, event/recon/rsp). --check will
therefore report the delta, which is what forces KT-1 to be brought current.
Similarly for other v0.7+ additions that landed in table bodies without a KT-1
sweep. Suppressing the delta would defeat the whole purpose of the tool.
"""

import argparse
import collections
import json
import os
import re
import sys

DEFAULT_DOC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "docs", "11-接口契约.md",
)

# S2.2.n -> group name used by KT-1. cmd_ack is its own group in the KT-1 table.
SUBSECTION_TO_GROUP = {
    1: "rt", 2: "state", 3: "cmd", 4: "cmd_ack", 5: "event",
    6: "health", 7: "audio", 8: "data", 9: "probe",
}

# Variable-segment expansion sizes (11 S2.2.11). The event/{severity}/{category}
# combinatorial 4x23=92 is called out separately in KT-1 as 组合上限 and does
# NOT enter its instance count; the pattern counts 1 -- see KT-1 rule (3) and
# its worked case for event/{severity}/{category} where instance == pattern.
VAR_EXPANSION = {
    "domain": 7,       # motion/speaker/asr/payload_light/ptz/gpu/dock (11 S7A.0)
    "severity": 4,     # info/warn/alarm/fault (11 S6.1)
    "channel": 1,      # normal (event/replay/{channel} uses a single value today)
    "category": 1,     # accompanies {severity} in the one combinatorial key
}

# Keys where KT-1 explicitly overrides the pattern-instance formula. The one
# case is event/{severity}/{category} whose instance count IS 1 by KT-1 rule
# (both cells in the row read 2 for the two independent items, and the 92
# combinatorial is called out separately). This override table keeps the
# extractor's default behaviour aligned with KT-1 without teaching it to parse
# KT-1's prose.
COMBINATORIAL_ONE = {"event/{severity}/{category}"}


def _slice_subsections(text):
    """Return {n: (start, end)} character offsets for each S2.2.n heading in
    the doc, for n in 1..9. Bounded above by S2.2.10 (next section) so a run-on
    that swallowed later sections cannot count them."""
    heads = {}
    for m in re.finditer(r"^#### 2\.2\.(\d+)\b", text, re.M):
        heads[int(m.group(1))] = m.start()
    out = {}
    for n in range(1, 10):
        if n not in heads:
            continue
        end_key = min((k for k in heads if k > n), default=None)
        out[n] = (heads[n], heads[end_key] if end_key is not None else len(text))
    return out


def _split_row(line):
    """Split a markdown table row on unescaped pipes, trim cells. Returns [] on
    a separator row (|---|---|) so the caller can skip it uniformly."""
    if not line.startswith("|"):
        return []
    cells = []
    cur = ""
    i = 0
    while i < len(line):
        c = line[i]
        if c == "\\" and i + 1 < len(line) and line[i + 1] == "|":
            cur += "|"
            i += 2
            continue
        if c == "|":
            cells.append(cur.strip())
            cur = ""
            i += 1
            continue
        cur += c
        i += 1
    cells.append(cur.strip())
    trimmed = cells[1:-1] if len(cells) >= 2 else []
    # A separator row: all cells are dashes/colons/spaces.
    if trimmed and all(c and set(c) <= set("-: ") for c in trimmed):
        return []
    return trimmed


def _extract_key_pattern(cell):
    """From a Key-column cell, return the pattern (rt/... form, WITHOUT the
    xbrain/{rid}/ prefix) or None. Skips tombstones (strike-through) and any
    non-key content."""
    # Tombstone forms: ~~text~~ or the strike glyph. A cell whose first
    # non-whitespace token starts a strike is dropped whole -- half-tombstoned
    # rows are treated as tombstoned to keep the extractor conservative.
    stripped = cell.lstrip("*_ ")
    if stripped.startswith("~~") or "⌦" in cell:
        return None
    # xbrain/{rid|robot_id}/<key>. Backticked or bold-wrapped forms both work
    # because the regex reaches into the text. Match the first occurrence.
    m = re.search(r"xbrain/\{(?:rid|robot_id)\}/([^\s`*|]+)", cell)
    if not m:
        return None
    key = m.group(1).rstrip("`*")
    return key


def _plane_of(pattern):
    """First segment: rt/state/cmd/event/health/audio/data/probe."""
    head = pattern.split("/", 1)[0]
    return head if head in ("rt", "state", "cmd", "event", "health", "audio",
                            "data", "probe") else None


def _group_of(pattern, subsection):
    """The KT-1 group for a pattern. cmd_ack is a suffix rule that overrides
    the S2.2.n-based mapping, so a cmd/*/ack row that leaks into S2.2.3 (or a
    cmd row that leaks into S2.2.4) still lands in the right column."""
    if pattern.endswith("/ack") or "/ack/" in pattern:
        return "cmd_ack"
    return SUBSECTION_TO_GROUP.get(subsection)


def _tokens(cell):
    """The backticked identifiers in a cell -- publishers/subscribers/QoS live
    inside backticks in every S2.2.n table. Free-text notes outside backticks
    (`p2_core` at 1 s and the like) are ignored because their meaning is
    documentation, not data."""
    return re.findall(r"`([^`]+)`", cell)


def _instance_count(pattern):
    """Instance count for one pattern by the KT-1 rules: 1 unless the pattern
    has a variable segment, in which case multiply by each segment's expansion
    size, subject to the combinatorial override."""
    if pattern in COMBINATORIAL_ONE:
        return 1
    vars_in = re.findall(r"\{([a-z_]+)\}", pattern)
    if not vars_in:
        return 1
    n = 1
    for v in vars_in:
        n *= VAR_EXPANSION.get(v, 1)
    return n


def _is_comparison_subtable_line(line):
    """S2.2.5 introduces a second table titled: 正文中已实际引用的具体事件
    key (供实现方对照, 非独立登记项). The words 非独立登记项 mark it as
    non-counting per KT-1 rule (1); the extractor skips every row after this
    marker within the same S2.2.n slice."""
    return "非独立登记项" in line or "供实现方对照" in line


def extract(doc_text):
    """Extract the full registry from S2.2.1~S2.2.9. Returns a list of records
    in document order; one record per pattern per subsection (a pattern can
    appear in more than one subsection only if the doc genuinely duplicates it,
    and that duplication is a finding rather than a shape to normalize away)."""
    slices = _slice_subsections(doc_text)
    records = []
    for n, (start, end) in sorted(slices.items()):
        seg = doc_text[start:end]
        seen_comparison_marker = False
        for line in seg.split("\n"):
            if _is_comparison_subtable_line(line):
                seen_comparison_marker = True
                continue
            if seen_comparison_marker:
                continue
            cells = _split_row(line)
            if not cells:
                continue
            key_cell = cells[0]
            pattern = _extract_key_pattern(key_cell)
            if pattern is None:
                continue
            # The 6-col table shape is Key/pub/sub/freq/QoS/Payload; some rows
            # squeeze extra bold or admin cells in the middle. Trust column
            # positions only for the first three (Key/pub/sub) and the LAST
            # two (QoS/Payload), which is where every S2.2.n table agrees.
            pubs = _tokens(cells[1]) if len(cells) > 1 else []
            subs = _tokens(cells[2]) if len(cells) > 2 else []
            qos = _tokens(cells[-2])[:1] if len(cells) >= 2 else []
            records.append({
                "pattern": pattern,
                "plane": _plane_of(pattern),
                "group": _group_of(pattern, n),
                "subsection": "2.2.%d" % n,
                "publishers": pubs,
                "subscribers": subs,
                "qos": qos[0] if qos else None,
                "instance_count": _instance_count(pattern),
            })
    return records


# --------------------------------------------------------------------------
# KT-1 expected values (per subsection group). These are the numbers written
# INTO S2.2.14; the point of --check is to spot when the table body has drifted
# from these. Sourced verbatim from the v0.6 KT-1 row -- when KT-1 is updated,
# this dict updates in the same commit, or the extractor's report says why.
# --------------------------------------------------------------------------

KT1_EXPECTED = {
    # (patterns, instances)
    "rt": (32, 36),
    "state": (21, 27),
    "cmd": (26, 34),
    "cmd_ack": (13, 13),
    "event": (2, 2),
    "health": (3, 3),
    "audio": (2, 2),
    "data": (2, 2),
    "probe": (2, 2),
}


def tally(records):
    """{group: (patterns, instances)} tallied from records."""
    patt = collections.Counter()
    inst = collections.Counter()
    for r in records:
        g = r["group"]
        if g is None:
            continue
        patt[g] += 1
        inst[g] += r["instance_count"]
    return {g: (patt[g], inst[g]) for g in KT1_EXPECTED}


def check(records, expected=KT1_EXPECTED):
    """Return list of (group, got, want) triples where extraction disagrees
    with KT-1. Empty list = green."""
    got = tally(records)
    return [(g, got[g], expected[g]) for g in expected if got[g] != expected[g]]


# --------------------------------------------------------------------------
# --self-test mutations (the three the criterion names verbatim)
# --------------------------------------------------------------------------

def self_test(doc_text):
    """Run three mutations on an IN-MEMORY copy of the doc and assert each
    changes the tally in the direction KT-1 said it must. Prints PASS/FAIL and
    returns exit code (0 = pass).
    """
    records = extract(doc_text)
    baseline = tally(records)

    # (1) drop one row from a S2.2.3 (cmd) table body -- the pattern count for
    # cmd must fall by exactly 1. Pick an actual extracted cmd row (not a
    # regex guess), so bold-wrapped vs plain-quoted forms both work.
    cmd_records = [r for r in records if r["group"] == "cmd"]
    if not cmd_records:
        print("SELF-TEST FAIL: no cmd rows in the extracted registry")
        return 1
    victim = cmd_records[0]["pattern"]     # first cmd row (cmd/estop, typically)
    row_re = re.compile(
        r"^\| [^\n]*`?xbrain/\{rid\}/" + re.escape(victim) + r"`?\**[^\n]*\n",
        re.M)
    m = row_re.search(doc_text)
    if m is None:
        print("SELF-TEST FAIL: could not locate the doc row for %s" % victim)
        return 1
    mut1 = doc_text[:m.start()] + doc_text[m.end():]
    m1_got = tally(extract(mut1))
    if m1_got["cmd"][0] != baseline["cmd"][0] - 1:
        print("SELF-TEST FAIL: dropping one cmd/ row moved cmd patterns "
              "from %d to %d (expected %d)"
              % (baseline["cmd"][0], m1_got["cmd"][0], baseline["cmd"][0] - 1))
        return 1

    # (2) add a NEW row to a table body without touching KT-1 -- checker must
    # then report the delta. We add to the tail of the S2.2.9 (probe) block.
    probe_head = re.search(r"^#### 2\.2\.9 ", doc_text, re.M)
    injection = "| `xbrain/{rid}/probe/injected` | X | Y | 1 Hz | Q3 | z |\n"
    mut2 = doc_text[:probe_head.end()] + "\n" + injection + doc_text[probe_head.end():]
    m2_got = tally(extract(mut2))
    if m2_got["probe"][0] != baseline["probe"][0] + 1:
        print("SELF-TEST FAIL: adding one probe row did not raise the "
              "count by 1 (%d -> %d)"
              % (baseline["probe"][0], m2_got["probe"][0]))
        return 1

    # (3) The S2.2.5 comparison-subtable marker: a naked verification is
    # tempting (remove the marker, see if the count changes) but the rows
    # already in the comparison subtable use short form (event/alarm/rtk)
    # WITHOUT the xbrain/{rid}/ prefix, so the extractor's prefix filter
    # accidentally protects them today. That means removing the marker alone
    # would not raise the count -- and the naive test would then wrongly claim
    # the marker is non-load-bearing. To prove the marker actually does its
    # job, INJECT a fully-formed xbrain/{rid}/event/... row into the
    # comparison-subtable region and check both directions:
    #   with the marker    -> injected row is excluded, count is unchanged;
    #   without the marker -> injected row is counted, count rises by 1.
    if "非独立登记项" not in doc_text or "供实现方对照" not in doc_text:
        print("SELF-TEST FAIL: expected S2.2.5 comparison-subtable markers "
              "missing from doc")
        return 1
    marker_pos = doc_text.find("非独立登记项")
    inject_at = doc_text.find("\n", marker_pos) + 1  # after the marker line
    injection = ("| `xbrain/{rid}/event/comparison_leak` | X | Y | e | Q3 | z |"
                 "\n")
    mut3a = doc_text[:inject_at] + injection + doc_text[inject_at:]
    m3a_got = tally(extract(mut3a))
    if m3a_got["event"][0] != baseline["event"][0]:
        print("SELF-TEST FAIL: a full-prefix row injected AFTER the marker "
              "was still counted (%d -> %d); the marker is not guarding"
              % (baseline["event"][0], m3a_got["event"][0]))
        return 1
    # Both marker strings neutralised: the same injected row must now count.
    mut3b = mut3a.replace("非独立登记项", "占位", 1).replace(
        "供实现方对照", "参考", 1)
    m3b_got = tally(extract(mut3b))
    if m3b_got["event"][0] != baseline["event"][0] + 1:
        print("SELF-TEST FAIL: with markers removed, the injected row was "
              "not counted (%d -> %d); the marker does not gate the region"
              % (baseline["event"][0], m3b_got["event"][0]))
        return 1

    print("SELF-TEST PASS: all three KT-1 mutations behave as required")
    return 0


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--doc", default=DEFAULT_DOC,
                    help="path to 11-接口契约.md (default: project doc)")
    ap.add_argument("--emit", action="store_true",
                    help="print the extracted registry as JSON to stdout")
    ap.add_argument("--check", action="store_true",
                    help="recompute KT-1 tallies and exit 1 on any delta")
    ap.add_argument("--self-test", action="store_true",
                    help="run the three KT-1 mutations against an in-memory "
                    "copy of the doc (does not touch disk)")
    args = ap.parse_args()

    text = open(args.doc, encoding="utf-8").read()

    if args.self_test:
        return self_test(text)

    records = extract(text)

    if args.emit:
        json.dump(records, sys.stdout, ensure_ascii=False, indent=2)
        sys.stdout.write("\n")
        return 0

    if args.check:
        # Print the surface first so the reader knows what the tally covers,
        # then either the OK line or every delta named.
        print("scan surface: 11-接口契约.md §2.2.1 ~ §2.2.9 (KT-1 rules)")
        deltas = check(records)
        got = tally(records)
        for g in sorted(KT1_EXPECTED):
            marker = "ok " if got[g] == KT1_EXPECTED[g] else "DIFF"
            print("  %s  %-8s  got=%s  KT-1=%s"
                  % (marker, g, got[g], KT1_EXPECTED[g]))
        print("criterion: extracted tally == 11 S2.2.14 KT-1")
        if deltas:
            print("FAIL: %d group(s) disagree with KT-1 -- fix the body of "
                  "S2.2.1~S2.2.9 or update KT-1 in the same commit."
                  % len(deltas))
            return 1
        return 0

    # No flag: print a brief summary.
    got = tally(records)
    print("extracted %d patterns across %d groups"
          % (len(records), sum(1 for v in got.values() if v[0])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
