#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: whitelist_gen.py
Brief: INF-ZN-7 -- extract 11 S1.1.6 hand-written whitelists, generate
       p2_core/p4_agent whitelists from S2.2 per WL-G1

Description:
Why this exists. 11 S1.1.6 pins the cross-plane process whitelists three ways at
once: perception / p1_motion / chassis_relay are HAND-WRITTEN tables in S1.1.6
because their cross-plane surface is small and closed (rule RT-C3.b's literal
"逐条列出"); p2_core and p4_agent are generated from S2.2 by WL-G1 because their
25-30 keys change every round and a second hand-copied table would silently
lag ("抄本必然滞后" -- S1.1.6 note). This script is BOTH extractors in one
place, so the two families cannot drift into two grammars.

What each of the three modes does:
  --emit       print every extracted whitelist as one JSON document; consumers
               (xbrain/common/zenoh/whitelists.py, startup selfcheck fixtures)
               read this rather than re-parsing the doc.
  --check      compare re-extracted whitelists against the committed
               xbrain/common/zenoh/whitelists.py (the CI drift gate the S1.1.6
               裁决 asks for; exit 1 on any delta with the delta named).
  --self-test  four criterion mutations applied to in-memory doc copies:
               ① drop P1-21 -> p1_motion count falls by 1
               ② flip P1-2 direction from "--" to "sub" -> count rises by 1
               ③ add event/** to p4_agent's S2.2 sub set -> WL-G3 refusal
               ④ flip P1-22 direction from pub to sub -> refusal (S7A.8 policy)

Direction rules that make WL-G1's generation trustworthy:
  * only rows whose direction column is "pub" or "sub" enter the whitelist;
    "--" rows (P1-2: not published by P1, points at chassis_relay CR-4) are
    documentation cross-references and MUST NOT count.
  * WL-G3 (禁通配): a generated whitelist that would contain a wildcard key
    is a defect, not a compression -- refuse; a whitelist that promises "any
    event key" would let a generic bridge past the RT-C3.b prohibition.
  * S7A.8 direction policy: P1-22 (state/arb/motion) MUST be pub-only; a doc
    edit that flipped it to sub would silently register a subscription that
    never fires (see doc note "登记一条永不出现的订阅同样是缺陷"). This is
    the fourth mutation the criterion adds. Extend DIRECTION_POLICY when
    S7A.8 grows -- one entry per (process, key) rule.

Traps -- things that look right and are not:
  1. Counting the "--" direction. The p1_motion table intentionally lists
     P1-2 with a "--" cell to route a reader to chassis_relay CR-4 without
     duplicating the CR-4 row. A count that included "--" would drift by
     exactly one from any legitimate change to that row.
  2. Treating perception's table shape (# | key | dir | ...) as chassis_relay's
     (# | dir | key | ...). Column position differs across the three
     hand-tables; the extractor keys on the HEADER row so an accidental
     column swap gets caught by the header pattern rather than by silent
     mis-parsing of every row.
  3. Re-parsing S2.2 here. The ZN-4 extractor already knows how to read it,
     including tombstones and the S2.2.5 comparison subtable; importing
     extract() from key_registry.py keeps one parser, one truth, one bug fix.
"""

import argparse
import json
import os
import re
import sys

# key_registry lives in the same directory; import through sys.path so this
# script can be invoked from anywhere.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from key_registry import extract as extract_keys                # noqa: E402

DEFAULT_DOC = os.path.join(os.path.dirname(os.path.dirname(_HERE)),
                           "docs", "11-接口契约.md")

#: S1.1.6 direction cells that count toward the whitelist. Every other cell
#: ("--", "cross-mark", empty) is a cross-reference or a placeholder -- not a
#: declaration (trap 1).
_COUNTED_DIRECTIONS = ("pub", "sub")

#: S7A.8 direction policy: (process, key_or_pattern) -> required direction.
#: When a doc row's direction disagrees with policy, the extractor refuses.
#: Kept small on purpose -- only rules the contract writes down verbatim.
DIRECTION_POLICY = {
    # 11 S7A.8: 域① motion 只发 state 不收 grant.
    ("p1_motion", "state/arb/{domain}"): "pub",
    ("p1_motion", "state/arb/motion"): "pub",
}

#: The five cross-plane processes S1.1.3 v0.6 closed as such. perception /
#: p1_motion / chassis_relay use hand-tables in S1.1.6; p2_core / p4_agent
#: use WL-G1 (generated from S2.2).
_HAND_TABLE_PROCESSES = ("perception", "p1_motion", "chassis_relay")
_GENERATED_PROCESSES = ("p2_core", "p4_agent")


# --------------------------------------------------------------------------
# S1.1.6 hand-table extractor
# --------------------------------------------------------------------------

def _slice_hand_tables(text):
    """Return {process: substring} for each hand-table subsection ①②③.

    Anchored on the subsection headings the doc uses verbatim; slicing on the
    NEXT heading (④ p2_core marks the end of ③ chassis_relay), so the parser
    never confuses one table's rows for another's.
    """
    slices = {}
    anchors = [
        ("perception",     r"^\*\*① `perception`",     r"^\*\*② `p1_motion`"),
        ("p1_motion",      r"^\*\*② `p1_motion`",      r"^\*\*③ `chassis_relay`"),
        ("chassis_relay",  r"^\*\*③ `chassis_relay`",  r"^\*\*④"),
    ]
    for name, start_re, end_re in anchors:
        m1 = re.search(start_re, text, re.M)
        m2 = re.search(end_re, text, re.M) if m1 else None
        if m1 and m2:
            slices[name] = text[m1.start():m2.start()]
    return slices


def _split_row(line):
    """Trim cells from a markdown table row. Returns [] on separator rows."""
    if not line.startswith("|"):
        return []
    parts = [c.strip() for c in line.split("|")[1:-1]]
    if parts and all(c and set(c) <= set("-: ") for c in parts):
        return []
    return parts


def _column_indices(header_cells):
    """Given a header row's trimmed cells, return {logical_name: index} for
    the columns this file cares about (key, direction). Handles the shape
    difference between ①②'s "# | key | dir | ..." and ③'s "# | dir | key |
    ..." (trap 2)."""
    idx = {}
    for i, cell in enumerate(header_cells):
        low = cell.strip()
        if low in ("方向",):
            idx["dir"] = i
        elif "key" in low and "RT" not in low:
            # 通用面 key column -- distinguished from ③'s "RT 面 key" column
            # by the absence of "RT" in the header text.
            idx.setdefault("key", i)
    return idx


def _extract_hand_table(section_text, process):
    """Return (pub_set, sub_set, policy_errors) for one hand-table.

    policy_errors lists (process, key, got_dir, want_dir) triples for rows
    violating DIRECTION_POLICY. The caller decides how to surface them
    (--check reports; --emit includes them alongside the whitelist).
    """
    pub, sub = set(), set()
    errors = []
    idx = None
    for line in section_text.split("\n"):
        cells = _split_row(line)
        if not cells:
            continue
        if idx is None:
            # First table row encountered is the header; column layout comes
            # from here so later rows are indexed uniformly.
            idx = _column_indices(cells)
            if "key" not in idx or "dir" not in idx:
                # Not the header we expect -- skip and keep looking. The
                # rows before the header (bold "**" wrappers, blockquotes)
                # never satisfy _split_row anyway.
                idx = None
            continue
        # Body row: pull the two cells we need, ignore the rest.
        if len(cells) <= max(idx["key"], idx["dir"]):
            continue
        key_cell, dir_cell = cells[idx["key"]], cells[idx["dir"]]
        direction = _clean_direction(dir_cell)
        if direction not in _COUNTED_DIRECTIONS:
            continue                            # "--" / "cross-mark" / empty (trap 1)
        key = _clean_key(key_cell)
        if key is None:
            continue                            # non-key cell (rare, defensive)
        # Direction-policy gate: raise-worthy findings surface via errors.
        want = DIRECTION_POLICY.get((process, key))
        if want and direction != want:
            errors.append((process, key, direction, want))
            continue                            # do not count a defective row
        (pub if direction == "pub" else sub).add(key)
    return pub, sub, errors


def _clean_direction(cell):
    """Strip formatting from a direction cell; map chassis_relay's forwarding
    direction (GEN -> RT / RT -> GEN) to the GENERAL-PLANE side (sub / pub).

    perception and p1_motion write plain "pub"/"sub"; chassis_relay is a
    forwarder and its "方向" column is which way it moves data, not which
    Zenoh operation it uses on the general plane:
      GEN -> RT: chassis_relay SUBSCRIBES the general-plane key, publishes
                 on the RT side (S1.1.6 CR-1 = cmd/estop sub).
      RT -> GEN: chassis_relay PUBLISHES the general-plane key, subscribes
                 on the RT side (S1.1.6 CR-4 = state/robot pub).
    The whitelist tracks the GENERAL-plane operation, so the mapping above
    is what enters _COUNTED_DIRECTIONS.
    """
    t = re.sub(r"[*★☆✅❌]", "", cell).strip()
    # Chassis_relay forwarding direction; the arrow spellings the doc uses
    # are U+2192 (right arrow) with either narrow or full-width spacing.
    if "GEN" in t and "RT" in t:
        # "GEN -> RT" (general side subscribes the key it forwards inward)
        # "RT -> GEN" (general side publishes the key it forwards outward)
        if t.index("GEN") < t.index("RT"):
            return "sub"                        # GEN -> RT
        return "pub"                            # RT -> GEN
    return t


def _clean_key(cell):
    """Return the backticked key from a key cell, or None."""
    m = re.search(r"`([^`]+)`", cell)
    return m.group(1) if m else None


# --------------------------------------------------------------------------
# WL-G1 generator (from S2.2 via key_registry)
# --------------------------------------------------------------------------

# key_registry's key extractor drops wildcards on purpose (S2.2 rows must not
# have any). WL-G3 needs to catch them ANYWAY, because a doc edit that snuck
# a wildcard past S2.2 review would then silently pass WL-G1 too. So we
# scan the raw doc for wildcard-bearing key rows independently and report
# WL-G3 for each one whose pub/sub cell names a WL-G1 process.
_WILDCARD_ROW_RE = re.compile(
    r"^\| \*?\*?`xbrain/\{rid\}/([^\s`|]*\*[^\s`|]*)`\*?\*?\s*"
    r"\|([^|\n]*)\|([^|\n]*)\|",
    re.M,
)


def _scan_wildcards(doc_text, processes):
    """Return [(process, wildcard_key, kind), ...] for every S2.2 row whose
    key contains * AND whose pub or sub cell names one of `processes`."""
    hits = []
    for m in _WILDCARD_ROW_RE.finditer(doc_text):
        key, pub_cell, sub_cell = m.group(1), m.group(2), m.group(3)
        for proc in processes:
            if "`%s`" % proc in pub_cell or proc in pub_cell:
                hits.append((proc, key, "pub"))
            if "`%s`" % proc in sub_cell or proc in sub_cell:
                hits.append((proc, key, "sub"))
    return hits


def _generate_wl_g1(records, process, wildcard_hits):
    """Filter S2.2 records for `process`, returning (pub, sub, wl_g3_errors).

    key_registry's extractor already dropped wildcards; wildcard_hits is
    supplied by _scan_wildcards so this function can report them too. A
    non-wildcard row that reaches records is legal and enters the whitelist.
    """
    pub, sub = set(), set()
    for r in records:
        if process in r["publishers"]:
            pub.add(r["pattern"])
        if process in r["subscribers"]:
            sub.add(r["pattern"])
    errors = [(p, k, kind) for (p, k, kind) in wildcard_hits if p == process]
    return pub, sub, errors


# --------------------------------------------------------------------------
# public extract
# --------------------------------------------------------------------------

def extract_all(doc_text):
    """Return dict {process: {pub, sub, policy_errors, wl_g3_errors}} for
    all five cross-plane processes. Sets are sorted lists in the return so
    JSON output is stable across runs."""
    out = {}
    hand = _slice_hand_tables(doc_text)
    for name in _HAND_TABLE_PROCESSES:
        section = hand.get(name, "")
        pub, sub, perr = _extract_hand_table(section, name)
        out[name] = {"pub": sorted(pub), "sub": sorted(sub),
                     "policy_errors": perr, "wl_g3_errors": []}
    records = extract_keys(doc_text)
    # Single wildcard scan feeds every generated process; running the regex
    # once and filtering by name below keeps the doc parse count at one.
    wildcard_hits = _scan_wildcards(doc_text, _GENERATED_PROCESSES)
    for name in _GENERATED_PROCESSES:
        pub, sub, gerr = _generate_wl_g1(records, name, wildcard_hits)
        out[name] = {"pub": sorted(pub), "sub": sorted(sub),
                     "policy_errors": [], "wl_g3_errors": gerr}
    return out


# --------------------------------------------------------------------------
# --check: compare against committed common/zenoh/whitelists.py
# --------------------------------------------------------------------------

def _load_committed():
    """Import the committed whitelists module; return {proc: {pub, sub}} or
    None when it does not exist yet (first --write will create it)."""
    try:
        # Reach xbrain package from doccheck/. Adding the project root once.
        root = os.path.dirname(os.path.dirname(_HERE))
        if root not in sys.path:
            sys.path.insert(0, root)
        from xbrain.common.zenoh import whitelists as w
    except Exception:                            # importable? if not, --write
        return None
    return {p: {"pub": sorted(getattr(w, "%s_PUB" % p.upper(), set())),
                "sub": sorted(getattr(w, "%s_SUB" % p.upper(), set()))}
            for p in _HAND_TABLE_PROCESSES + _GENERATED_PROCESSES}


def check(extracted, committed):
    """Return list of (process, kind, only_extracted, only_committed) deltas."""
    if committed is None:
        return [("whitelists.py", "missing", [], [])]
    deltas = []
    for proc in extracted:
        for kind in ("pub", "sub"):
            e = set(extracted[proc][kind])
            c = set(committed.get(proc, {}).get(kind, []))
            if e != c:
                deltas.append((proc, kind, sorted(e - c), sorted(c - e)))
    return deltas


# --------------------------------------------------------------------------
# self-test (four criterion mutations)
# --------------------------------------------------------------------------

def self_test(doc_text):
    """Run the four criterion mutations on in-memory copies of the doc."""
    baseline = extract_all(doc_text)
    p1_base = len(baseline["p1_motion"]["pub"]) + len(baseline["p1_motion"]["sub"])

    # ① Delete P1-21 row -> p1_motion count falls by 1. Row prefix is
    # "| [three U+2605] **P1-21**<br>..." in the doc (CJK star + bold), so any
    # non-pipe prefix is permitted rather than guessing star/bold shape.
    m = re.search(r"^\| [^|\n]*P1-21[^\n]*\n", doc_text, re.M)
    if m is None:
        print("SELF-TEST FAIL: could not locate P1-21 row")
        return 1
    mut1 = doc_text[:m.start()] + doc_text[m.end():]
    r1 = extract_all(mut1)
    if len(r1["p1_motion"]["pub"]) + len(r1["p1_motion"]["sub"]) != p1_base - 1:
        print("SELF-TEST FAIL: dropping P1-21 did not reduce p1_motion by 1 "
              "(%d -> %d)" % (p1_base,
                              len(r1["p1_motion"]["pub"]) +
                              len(r1["p1_motion"]["sub"])))
        return 1

    # 2. Flip P1-2 direction from em-dash (U+2014) to "sub" -> count rises by 1.
    # The doc uses U+2014; escaping preserves the match after the ASCII-only
    # cleanup pass that turned em-dashes in prose into "--".
    p1_2_re = re.compile(r"(\| P1-2 \| `state/robot` \| )—", re.M)
    if not p1_2_re.search(doc_text):
        print("SELF-TEST FAIL: could not locate P1-2 row with dash direction")
        return 1
    mut2 = p1_2_re.sub(r"\1sub", doc_text)
    r2 = extract_all(mut2)
    if len(r2["p1_motion"]["pub"]) + len(r2["p1_motion"]["sub"]) != p1_base + 1:
        print("SELF-TEST FAIL: flipping P1-2 to sub did not raise p1_motion "
              "by 1 (%d -> %d)" % (p1_base,
                                   len(r2["p1_motion"]["pub"]) +
                                   len(r2["p1_motion"]["sub"])))
        return 1

    # ③ Inject an event/** row into a S2.2 subscriber cell for p4_agent
    # -> the WL-G1 generator must refuse (WL-G3).
    # The committed row is bold-wrapped ("| **`xbrain/{rid}/...`** |"), so the
    # regex must accept optional bold markers around the backticked key.
    p4_event = re.search(r"^\| \*?\*?`xbrain/\{rid\}/event/\{severity\}/"
                         r"\{category\}`\*?\*?[^\n]*\n", doc_text, re.M)
    if p4_event is None:
        print("SELF-TEST FAIL: could not locate event/{sev}/{cat} row")
        return 1
    # Rewrite the subscribers cell to include p4_agent AND change pattern to
    # a wildcard shape. Easiest: swap the event row's key for event/** and
    # add p4_agent to its subscriber list.
    injection = ("| `xbrain/{rid}/event/**` | X | `p4_agent` | e | Q3 | z |\n")
    mut3 = doc_text[:p4_event.end()] + injection + doc_text[p4_event.end():]
    r3 = extract_all(mut3)
    if not any(kind == "sub" for _proc, key, kind in r3["p4_agent"]["wl_g3_errors"]
               if key == "event/**"):
        print("SELF-TEST FAIL: wildcard sub on p4_agent was not reported "
              "as WL-G3")
        return 1

    # ④ Flip P1-22 direction from pub to sub -> DIRECTION_POLICY refusal.
    # Row form in the committed doc: "| [3xU+2605] **P1-22**<br>...  | `key` |
    # [3xU+2605] **pub** | ...". The direction cell (3rd column) has "pub"
    # possibly with bold+star wrappers; permit any prefix inside the cell.
    p1_22_re = re.compile(
        r"(\| [^|\n]*P1-22[^|\n]*\| [^|\n]*\| [^|\n]*?)pub(?!\w)",
        re.M,
    )
    if not p1_22_re.search(doc_text):
        print("SELF-TEST FAIL: could not locate P1-22 direction cell")
        return 1
    mut4 = p1_22_re.sub(r"\1sub", doc_text, count=1)
    r4 = extract_all(mut4)
    if not any(key == "state/arb/{domain}" or key == "state/arb/motion"
               for _proc, key, _got, _want in r4["p1_motion"]["policy_errors"]):
        # P1-22 might have any of a few variant key spellings; accept either.
        # If nothing reported, the policy did not fire.
        print("SELF-TEST FAIL: flipping P1-22 to sub did not trigger "
              "DIRECTION_POLICY (policy_errors=%r)"
              % r4["p1_motion"]["policy_errors"])
        return 1

    print("SELF-TEST PASS: all four INF-ZN-7 mutations behave as required")
    return 0


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--doc", default=DEFAULT_DOC)
    ap.add_argument("--emit", action="store_true",
                    help="print all five whitelists as JSON")
    ap.add_argument("--check", action="store_true",
                    help="compare against xbrain/common/zenoh/whitelists.py; "
                    "exit 1 on any delta")
    ap.add_argument("--self-test", action="store_true",
                    help="run the four criterion mutations against in-memory "
                    "copies of the doc")
    args = ap.parse_args()

    text = open(args.doc, encoding="utf-8").read()

    if args.self_test:
        return self_test(text)

    extracted = extract_all(text)

    if args.emit:
        json.dump(extracted, sys.stdout, ensure_ascii=False, indent=2)
        sys.stdout.write("\n")
        return 0

    if args.check:
        print("scan surface: 11 S1.1.6 hand-tables + S2.2 via WL-G1")
        # Surface policy_errors and wl_g3_errors first: those are defects in
        # the DOC that no amount of whitelists.py edits can suppress.
        for proc, data in extracted.items():
            for pe in data["policy_errors"]:
                print("  DIRECTION-POLICY: %s %s direction=%s want=%s"
                      % (pe[0], pe[1], pe[2], pe[3]))
            for we in data["wl_g3_errors"]:
                print("  WL-G3 wildcard:   %s %s (%s)" % we)
        committed = _load_committed()
        deltas = check(extracted, committed)
        for proc, kind, only_e, only_c in deltas:
            print("  DIFF  %-14s %s  extract-only=%s  committed-only=%s"
                  % (proc, kind, only_e, only_c))
        print("criterion: extracted == committed AND no policy/WL-G3 errors")
        bad = deltas or any(d["policy_errors"] or d["wl_g3_errors"]
                            for d in extracted.values())
        return 1 if bad else 0

    # No flag: summary.
    for proc, data in extracted.items():
        print("%-14s pub=%d sub=%d" % (proc, len(data["pub"]), len(data["sub"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
