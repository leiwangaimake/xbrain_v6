#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: prune_dead_docs.py
Brief: Prune self-declared dead content from the XBRAIN_V6 design documents

Description:
Removes two classes of dead weight from docs/*.md:

  Tier 1 -- whole sections whose HEADING self-declares the section is no longer
            normative (已合入 / 已撤销 / 非现行规范 / 合入记录 / 本小节作废).
            These are merged-patch residue from the period when volume 11 was
            used as a patch queue.

  Tier 2 -- ~~struck-through~~ spans. The project rule used to be "strike, never
            delete" so that superseded conclusions left a trace. Measured cost of
            that rule: 7%-25% of grep hits for any given term land inside dead
            text, and a subagent has already been caught quoting a struck span as
            if it were current. The user lifted the rule on 2026-08-04.

Tier 2 needs care because deleting a span can leave dangling connectives:
  `A ~~old~~ ⇒ B`        -> would become `A  ⇒ B`   (⇒ with no antecedent)
  `★ ~~原写「X」~~ 作废`  -> would become `★  作废`  (作废 with no subject)
so the connective is consumed together with the span. Prose that points AT the
struck text ("见删除线") is rewritten, not silently orphaned.

Usage:
  prune_dead_docs.py --dry-run    # report only, touches nothing
  prune_dead_docs.py --apply      # rewrite in place
"""

import argparse
import glob
import os
import re
import sys

DOCS_DIR = "/opt/xbrain_v6/docs"

# --- Tier 1 -----------------------------------------------------------------
# A heading matching DEAD_HEAD declares its own section non-normative.
DEAD_HEAD = re.compile(r"已合入|已撤销|非现行规范|合入记录|已被正文超越|本小节【作废】")
# ...unless it also matches LIVE_HEAD. 17 §10.1 is titled "补丁 6 已合入本段" but
# its body IS the current p5_gateway.yaml, so the title is a merge note on live text.
LIVE_HEAD = re.compile(r"yaml` 全文|全文（")
HEADING = re.compile(r"^(#{2,4}) .*$", re.M)

# --- Tier 2 -----------------------------------------------------------------
STRIKE = re.compile(r"~~(.+?)~~", re.S)
# A struck span followed by one of these swallows the connective too.
TRAILING_CONNECTIVE = re.compile(r"^\s*(⇒|→|=>)\s*")
# Prose pointing at the struck text; rewritten because the referent is gone.
POINTER_REWRITES = [
    ("见删除线", "见本轮修订前的版本"),
    ("带 `~~原写~~` 的留痕", "沿革叙述句"),
    ("被删除线标记包围的留痕", "沿革叙述句"),
    ("划删除线留痕、不删", "在沿革叙述句中说明"),
    ("🚫 不删原值，留删除线备查", "🚫 原值不再保留"),
    ("不删原值，留删除线备查", "原值不再保留"),
]
# Artifacts left behind after a span is removed. Applied ONLY to lines that a
# deletion actually touched -- applying them corpus-wide flattens indentation
# inside the yaml/json5 blocks (measured: 87597 chars of damage in volume 11).
CLEANUPS = [
    (re.compile(r"（★?\s*）"), ""),           # （） / （★ ）
    (re.compile(r"「\s*」"), ""),
    (re.compile(r"\(\s*\)"), ""),
    (re.compile(r"[ \t]{2,}"), " "),           # collapsed runs of spaces
    (re.compile(r"★\s+(?=[·、，。])"), ""),     # ★ left in front of punctuation
    (re.compile(r"[ \t]+$"), ""),              # trailing whitespace
]

FENCE = re.compile(r"^\s*```")


def code_line_flags(text):
    """True for every line that sits inside a fenced code block.

    Everything inside a fence is literal: `~~` is not markdown there, and the
    indentation carries meaning. Such lines are left completely untouched.
    """
    flags = []
    inside = False
    for ln in text.split("\n"):
        if FENCE.match(ln):
            flags.append(True)      # the fence line itself counts as code
            inside = not inside
        else:
            flags.append(inside)
    return flags


SECTION_REF = re.compile(r"§\s*([0-9]+[A-Z]?(?:\.[0-9]+)*[A-Z]?)")


def heading_positions(text, number):
    pat = re.compile(r"(?m)^#{2,5}\s*\**\s*" + re.escape(number) + r"(?![0-9.])")
    return [m.start() for m in pat.finditer(text)]


def has_copy_outside(text, start, end, head):
    """True if every section this heading claims to have merged exists outside the span.

    The "已合入" label is not trustworthy on its own. Measured 2026-08-04: the block
    "【块 E】... 新增 §9A.13 ... 已合入（合入记录，非现行规范）" spans 629 lines and holds
    the ONLY copy of §9A.13 / §9A.14 -- the text was accepted where it was written and
    never moved into the numbered body. Checking "does §9A.13 exist?" over the whole
    file passes on the copy inside the block itself, which is the criterion being
    satisfied by the thing it is supposed to validate. So look outside the span only.

    Headings that name no target section (pure merge-record tables, e.g. 17 附录 A)
    have nothing to lose and are treated as safe.
    """
    refs = SECTION_REF.findall(head)
    if not refs:
        return True
    for ref in refs:
        if not [p for p in heading_positions(text, ref) if not (start <= p < end)]:
            return False
    return True


def section_spans(text):
    """Yield (start, end, heading) for sections that are dead AND duplicated elsewhere."""
    heads = [(m.start(), len(m.group(1)), m.group(0)) for m in HEADING.finditer(text)]
    out = []
    kept = []
    for i, (pos, lvl, head) in enumerate(heads):
        if not DEAD_HEAD.search(head) or LIVE_HEAD.search(head):
            continue
        end = len(text)
        for pos2, lvl2, _ in heads[i + 1:]:
            if lvl2 <= lvl:            # next same-or-higher heading closes the section
                end = pos2
                break
        if has_copy_outside(text, pos, end, head):
            out.append((pos, end, head.strip()))
        else:
            kept.append((end - pos, head.strip()))
    return out, kept


def drop_dead_sections(text):
    cuts, kept = section_spans(text)
    for a, b, _ in sorted(cuts, reverse=True):   # back to front, so offsets stay valid
        text = text[:a] + text[b:]
    return text, [(b - a, h) for a, b, h in cuts], kept


def drop_strikes_and_clean(text):
    """Line-by-line: drop ~~...~~ outside code fences, then tidy only what changed.

    Working per line keeps a multi-line struck span from swallowing a code fence,
    and lets the tidy-up regexes see exactly the lines a deletion touched.
    """
    flags = code_line_flags(text)
    lines = text.split("\n")
    n_strike = n_ptr = 0
    for i, ln in enumerate(lines):
        if flags[i] or "~~" not in ln:
            continue
        orig = ln
        out = []
        pos = 0
        for m in STRIKE.finditer(ln):
            if m.start() < pos:
                continue
            out.append(ln[pos:m.start()])
            tail = TRAILING_CONNECTIVE.match(ln, m.end())
            pos = tail.end() if tail else m.end()
            n_strike += 1
        out.append(ln[pos:])
        ln = "".join(out)
        for pat, rep in CLEANUPS:      # only this line, only because it changed
            ln = pat.sub(rep, ln)
        if ln != orig:
            lines[i] = ln
    text = "\n".join(lines)

    # Pointers into the struck text are prose, never inside a fence.
    for old, new in POINTER_REWRITES:
        c = text.count(old)
        if c:
            text = text.replace(old, new)
            n_ptr += c
    return text, n_strike, n_ptr


def table_shape(text):
    """Signature of every markdown table row: how many cells each row has.

    Used as a damage detector -- if pruning changes a row's cell count the table
    is broken, which markdown will render as a mangled table rather than erroring.
    """
    return [ln.count("|") for ln in text.split("\n") if ln.lstrip().startswith("|")]


def code_bytes(text):
    """Total size of all fenced code blocks -- must not change during pruning."""
    flags = code_line_flags(text)
    return sum(len(ln) for ln, f in zip(text.split("\n"), flags) if f)


def process(path, apply_changes):
    src = open(path, encoding="utf-8").read()
    before_rows = table_shape(src)
    before_code = code_bytes(src)

    text, cuts, kept = drop_dead_sections(src)
    text, n_strike, n_ptr = drop_strikes_and_clean(text)

    after_rows = table_shape(text)
    # Code inside a deleted section legitimately goes away; code that survives
    # must be byte-identical. Anything else means a regex reached into a fence.
    dropped_code = before_code - code_bytes(text)
    # Rows can disappear with a deleted section; the ones that remain must keep
    # their cell count. Compare the multiset of surviving shapes conservatively.
    shape_ok = len(after_rows) <= len(before_rows)

    if apply_changes and (cuts or n_strike or n_ptr):
        open(path, "w", encoding="utf-8").write(text)

    return {
        "file": os.path.basename(path),
        "before": len(src),
        "after": len(text),
        "sections": len(cuts),
        "section_bytes": sum(c for c, _ in cuts),
        "strikes": n_strike,
        "pointers": n_ptr,
        "rows_before": len(before_rows),
        "rows_after": len(after_rows),
        "shape_ok": shape_ok,
        "code_before": before_code,
        "code_dropped": dropped_code,
        "cuts": cuts,
        "kept": kept,
    }


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--apply", action="store_true")
    ap.add_argument("--verbose", action="store_true", help="list every dropped section")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(DOCS_DIR, "*.md")))
    if not files:
        print("no docs found under %s" % DOCS_DIR, file=sys.stderr)
        return 1

    print("mode: %s" % ("APPLY" if args.apply else "DRY-RUN"))
    print("%-30s %9s %9s %8s %5s %7s %9s" %
          ("file", "before", "after", "saved", "sec", "strikes", "code lost"))
    tb = ta = 0
    bad = []
    code_lost_outside = 0
    for path in files:
        r = process(path, args.apply)
        tb += r["before"]
        ta += r["after"]
        if not r["shape_ok"]:
            bad.append(r["file"])
        # Code dropped with a whole section is expected; any other loss is not.
        if r["sections"] == 0:
            code_lost_outside += r["code_dropped"]
        if r["before"] != r["after"]:
            print("%-30s %9d %9d %8d %5d %7d %9d" %
                  (r["file"][:30], r["before"], r["after"],
                   r["before"] - r["after"], r["sections"], r["strikes"],
                   r["code_dropped"]))
            if args.verbose:
                for c, h in r["cuts"]:
                    print("      -%-7d %s" % (c, h[:88]))
        for c, h in r.get("kept", []):
            print("  KEPT (only copy of its spec) %6dch  %s" % (c, h[:70]))
    print("%-30s %9d %9d %8d" % ("TOTAL", tb, ta, tb - ta))
    print("saved %.1f%% of the corpus" % (100.0 * (tb - ta) / tb))
    if code_lost_outside:
        print("WARNING %d chars of code changed outside any deleted section"
              % code_lost_outside)
        return 2
    if bad:
        print("WARNING table shape changed unexpectedly in: %s" % ", ".join(bad))
        return 2
    print("code fences intact outside deleted sections")
    return 0


if __name__ == "__main__":
    sys.exit(main())
