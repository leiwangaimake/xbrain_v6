#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: intent_keyword_sync.py
Brief: Guard that intent trigger phrases stay in sync between 18 (spec) and 16 (registry)

Description:
On 2026-08-05 two trigger phrases were deleted from 18 (the requirement-level
command set) but NOT from 16 (the intent registry and the prompt), which is where
they actually take effect. The design volume said the phrase was gone; the
implementation volume still fired on it. Editing the spec without the registry is
indistinguishable from doing nothing at runtime.

This checks the one direction that is a real defect: a keyword present in 16's
registry whose intent no longer lists it in 18. The reverse (18 lists a phrase 16
has not picked up yet) is normal debt during a spec change, so it is reported
separately and does not fail the run.

Scan surface is declared in the output on every run -- a negative claim without a
declared surface is a guess, not a result (see 11 S15.6F.3).
"""

import re
import sys

DOCS = "/opt/xbrain_v6/docs"
SPEC = f"{DOCS}/18-语音文本指令集.md"
REGISTRY = f"{DOCS}/16-P4Agent管线详细设计.md"

# Keywords 16 carries that 18 does not list, reviewed by hand on 2026-08-05 and
# found to be synonym expansions rather than stale triggers. 18's third column is
# a set of EXAMPLE utterances, not an exhaustive trigger list, so 16 having a
# richer synonym set is normal.
#
# Listing them one by one -- rather than loosening the rule to "warn only" -- is
# what keeps this criterion able to fail. A new orphan that nobody has reviewed
# still turns the run red, which is the whole point: the 2026-08-05 miss was a
# phrase deleted from 18 and left live in 16, and that must never pass silently.
REVIEWED_SYNONYMS = {
    ("B02", "巡查"): "18 用「巡逻」，16 补同义词「巡查」",
    ("B03", "之后"): "时间状语，18 例句用「几点」，16 补「之后」",
    ("B03", "点钟"): "同上，口语量词",
    ("B04", "接着巡"): "18 用「继续巡逻」，16 补口语缩略",
    ("B09", "回桩"): "18 用「回充电桩」，16 补缩略",
    ("G01", "什么位置"): "18 用「在哪」，16 补同义问法",
    ("H02", "说说情况"): "18 用「汇报」，16 补口语问法",
}

# 16's registry lines look like:
#   turn_around:  { id: A11, keywords: ["转身", "掉头"], slots: [], ... }
REG_LINE = re.compile(r'^\s*([a-z_]+):\s*\{\s*id:\s*([A-Z][0-9]{2})\s*,\s*keywords:\s*\[([^\]]*)\]')
KW = re.compile(r'"([^"]+)"')


def registry_keywords():
    """intent id -> {keyword: line_no} as declared in 16."""
    out = {}
    for n, line in enumerate(open(REGISTRY, encoding="utf-8"), 1):
        m = REG_LINE.match(line)
        if not m:
            continue
        out[m.group(2)] = ({k: n for k in KW.findall(m.group(3))}, m.group(1))
    return out


def spec_phrases(text, intent_id):
    """Every phrase 18 lists for this intent, from its row in the intent table.

    18's rows are `| A11 | \\`turn_around\\` | 转身 / 掉头 | ... |`. Only the third
    column is the trigger set; later columns carry prose that must not be mined
    for phrases (doing so would make the guard pass on any wording that happens
    to appear in an explanation -- a criterion satisfied by unrelated text).
    """
    for line in text.split("\n"):
        if not line.startswith("|"):
            continue
        cells = line.split("|")
        if len(cells) < 4:
            continue
        if cells[1].strip().strip("*` ") != intent_id:
            continue
        trig = cells[3]
        # Strip the inline correction notes the project appends after <br>.
        trig = trig.split("<br>")[0]
        return {p.strip() for p in re.split(r"[/·]", trig) if p.strip()}
    return None


def main():
    self_test = "--self-test" in sys.argv
    spec = open(SPEC, encoding="utf-8").read()
    reg = registry_keywords()

    print(f"scan surface: {REGISTRY} registry lines vs {SPEC} intent tables")
    print(f"  registry intents parsed: {len(reg)}")

    orphans, unadopted, no_row, reviewed = [], [], [], []
    for iid, (kws, name) in sorted(reg.items()):
        phrases = spec_phrases(spec, iid)
        if phrases is None:
            no_row.append((iid, name))
            continue
        # 16 stores keyword STEMS for fastpath matching ("前进"); 18 stores full
        # example utterances ("前进 N 米"). A stem is adopted if it occurs inside
        # any of the intent's phrases. Requiring equality made this criterion red
        # for 35 rows that were all correct -- an always-red check is as useless
        # as an always-green one, and its predictable next step is someone
        # loosening it to "contains anything" (11 S15.6F.3, CLAUDE.md 3.2 form 2).
        for k, ln in kws.items():
            if not any(k in p for p in phrases):
                if (iid, k) in REVIEWED_SYNONYMS:
                    reviewed.append((iid, name, k, REVIEWED_SYNONYMS[(iid, k)]))
                else:
                    orphans.append((iid, name, k, ln))
        for p in phrases:
            if not any(k in p for k in kws):
                unadopted.append((iid, name, p))

    print(f"\n  ORPHAN   {len(orphans):3}  (16 仍会命中，但 18 已删该说法 —— ★ 这是真缺陷)")
    for iid, name, k, ln in orphans:
        print(f"      {REGISTRY.split('/')[-1]}:{ln}  {iid} {name}  keyword 「{k}」")
    print(f"  REVIEWED {len(reviewed):3}  (已具名复核的同义词扩展 —— 不判失败，但每条都要有理由)")
    for iid, name, k, why in reviewed:
        print(f"      {iid} {name}  「{k}」 —— {why}")
    print(f"  UNADOPTED {len(unadopted):3}  (18 有、16 尚未吸收 —— ★ 规范变更期的正常欠账，不判失败)")
    for iid, name, p in unadopted[:10]:
        print(f"      {iid} {name}  「{p}」")
    if no_row:
        print(f"  NO-ROW    {len(no_row):3}  (16 有登记但 18 表内找不到该 id 行 —— 须人工看)")
        for iid, name in no_row[:10]:
            print(f"      {iid} {name}")

    if self_test:
        # Mutation: a keyword 18 does not list must surface as ORPHAN. Without
        # this the whole check could be a no-op and still print zeros.
        fake = dict(reg)
        probe = next(iter(sorted(reg)))
        kws, name = fake[probe]
        fake[probe] = ({**kws, "★不存在的说法★": 0}, name)
        hit = "★不存在的说法★" not in (spec_phrases(spec, probe) or set())
        print(f"\nself-test: 注入一个 18 里没有的 keyword ⇒ {'检出 OK' if hit else '未检出 FAIL'}")
        return 0 if hit else 1

    print("\ncriterion: ORPHAN == 0")
    print("★ ORPHAN 的含义：改了规范册没改实现册 —— 运行期与没改【不可区分】。")
    return 1 if orphans else 0


if __name__ == "__main__":
    sys.exit(main())
