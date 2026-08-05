#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: closed_set_snapshot.py
Brief: Snapshot the closed sets and guard criteria of the design documents

Description:
Extracts everything the design package treats as a closed set -- Zenoh keys,
E_* error codes, section headings, constraint IDs -- plus the positive guard
criteria the docs define for themselves. Run before and after any bulk edit and
diff the two snapshots: anything that disappears from a closed set is a real
loss, whereas a change in a raw hit count is not (the docs' own GC-1d rule:
"数字不等不构成失败，归类出现活体命中才构成失败").

Deliberately extracts from LIVE prose only -- struck spans are excluded, so a
snapshot taken before pruning is directly comparable to one taken after.

Usage:
  closed_set_snapshot.py > before.json
  closed_set_snapshot.py > after.json
  closed_set_snapshot.py --diff before.json after.json
"""

import glob
import json
import os
import re
import sys

DOCS_DIR = "/opt/xbrain_v6/docs"

STRIKE = re.compile(r"~~(.+?)~~", re.S)

PATTERNS = {
    # xbrain/{rid}/plane/... -- the key table drives the startup self-check
    "zenoh_keys": re.compile(r"xbrain/\{?[a-z_]*\}?/[a-z_]+(?:/[a-z_0-9*{}]+)*"),
    # the 40-code closed set exported by common/errors/
    "error_codes": re.compile(r"\bE_[A-Z][A-Z0-9_]+"),
    # constraint / decision IDs that other volumes cite by number
    "constraint_ids": re.compile(
        r"\b(?:CLK|NET|RT|CR|CRL|SP|EC|UL|FV|FS|GC|QC|AS|HW|MI|GL|RCG|PP|ND|TSK|VOI|SEC|GATE|NAV|PER|PAY|CFG|COM|HMI|DDS|CPP|PB|BOOT|SYS|LNK|CHG|CA|AP|KT)-[0-9]+[a-z]?\b"),
    # decision-record numbering axes
    "decision_ids": re.compile(r"\b[MUVD]-[0-9]{1,3}\b"),
    # failure-timeout registry -- 11 §1.6 says every timeout lives in one table
    "timeouts": re.compile(r"\bT-[0-9]{2}\b"),
}

# Positive guard criteria the documents state about themselves. Each is
# (label, file glob, literal string, comparison) where comparison is "ge1"
# (must stay >= 1) or "keep" (record only, count may legitimately move).
# Headings whose disappearance was reviewed by hand on 2026-08-04 and found
# correct. Listed one by one with the evidence, rather than widening the regex
# above -- a broadened pattern would quietly absorb future real losses, which is
# how a check turns permanently green.
REVIEWED_HEADING_LOSSES = {
    "13.15 L · ★ 启动自检（v0.3 新增，⚠️ 待评审）":
        "intentional rename 2026-08-05: the group was promoted out of 待评审 when "
        "E_STORAGE_CORRUPT was adjudicated into it, so the heading now reads "
        "'评审通过转正'. The section did not disappear -- its title changed.",
    "7.4.1 ★★ 开机 pan 零位对齐（CAL-02）":
        "payload of 编辑 C3-4, which is marked 已撤销(C28.2). 99 records CAL-02 as "
        "改为回 Preset, so no live §7.4.1 should exist; its 19 tombstone citations remain.",
    "4.4.3 ★★ 域⑤ 持有者同时持有「码率提升权」（v0.3 新增 · C28.3 · U50）":
        "v0.3 payload copy inside 17 附录 B. Live copy is 14 §4.4.3 (v0.7), whose own "
        "heading reads 合入 `17` 附录 B 补丁 BW-8. Titles differ by version so the "
        "cross-file match does not fire.",
}

GUARDS = [
    ("lidar_必需件_12", "12-*.md", "整机层面 LiDAR 仍是必需件", "ge1"),
    ("lidar_必需件_14", "14-*.md", "整机层面 LiDAR 仍是必需件", "ge1"),
    ("lidar_必需件_18", "18-语音*.md", "整机层面 LiDAR 仍是必需件", "ge1"),
    ("as7_freeze_list", "11-*.md", "＋ `AS-7`", "ge1"),
    ("plane_closed_set", "11-*.md", "八值封闭集", "ge1"),
    ("estop_exempt_one_key", "11-*.md", "豁免范围", "ge1"),
    ("cloud_in_estop_pub", "11-*.md", "HMI · 云端 · 微信", "ge1"),
    ("gc1d_count_not_criterion", "11-*.md", "数字不等不构成失败", "ge1"),
    ("qos_fallback_binding", "11-*.md", 'match: "xbrain/*/**"', "ge1"),
    ("startup_reject_unregistered", "11-*.md", "拒绝启动", "ge1"),
]


def live_text(raw):
    """Prose with struck spans removed, so before/after snapshots are comparable."""
    return STRIKE.sub("", raw)


def snapshot():
    out = {"files": {}, "sets": {k: {} for k in PATTERNS}, "guards": {}, "headings": {}}
    for path in sorted(glob.glob(os.path.join(DOCS_DIR, "*.md"))):
        name = os.path.basename(path)
        raw = open(path, encoding="utf-8").read()
        text = live_text(raw)
        out["files"][name] = {"chars": len(raw), "live_chars": len(text)}
        for key, pat in PATTERNS.items():
            for hit in pat.findall(text):
                out["sets"][key].setdefault(hit, 0)
                out["sets"][key][hit] += 1
        out["headings"][name] = re.findall(r"(?m)^#{1,4} .*$", text)

    for label, pattern, needle, mode in GUARDS:
        total = 0
        for path in glob.glob(os.path.join(DOCS_DIR, pattern)):
            total += live_text(open(path, encoding="utf-8").read()).count(needle)
        out["guards"][label] = {"count": total, "mode": mode}
    return out


def diff(a_path, b_path):
    a = json.load(open(a_path))
    b = json.load(open(b_path))
    rc = 0

    print("== closed sets ==")
    for key in a["sets"]:
        lost = sorted(set(a["sets"][key]) - set(b["sets"][key]))
        gained = sorted(set(b["sets"][key]) - set(a["sets"][key]))
        status = "OK" if not lost else "LOST"
        if lost:
            rc = 1
        print("  %-16s before=%-5d after=%-5d %s" %
              (key, len(a["sets"][key]), len(b["sets"][key]), status))
        for item in lost[:20]:
            print("      LOST  %s   (was cited %dx)" % (item, a["sets"][key][item]))
        for item in gained[:10]:
            print("      NEW   %s" % item)

    print("== guards ==")
    for label, rec in a["guards"].items():
        after = b["guards"].get(label, {}).get("count", 0)
        ok = after >= 1 if rec["mode"] == "ge1" else True
        if not ok:
            rc = 1
        print("  %-28s %d -> %d  %s" %
              (label, rec["count"], after, "OK" if ok else "FAIL"))

    print("== headings ==")
    for name in a["headings"]:
        ha, hb = a["headings"][name], b["headings"].get(name, [])
        # Compare on a normalised key. Removing a struck span from inside a heading
        # changes its literal text ("### ~~编辑 C5-1~~ · X" -> "### · X"), which is a
        # rewrite, not a loss -- collapse whitespace and punctuation so the two match.
        def key(h):
            # Strip the level markers too, so a heading that moved between levels
            # (or is quoted here without them) still matches on its text.
            return re.sub(r"[#\s·—\-*★⚠️🚫✅❌]+", "", h)
        kb = {key(h) for h in hb}
        lost = [h for h in ha if key(h) not in kb]
        # Three reasons a heading may legitimately vanish:
        #  1. it declares its own section non-normative;
        #  2. it is the shell of a patch block (编辑 / 补丁 / 块 / E-n);
        #  3. it was the payload copy of a cross-volume patch -- the same heading
        #     still exists in the volume the patch targeted, which is where the
        #     spec was supposed to end up. Checked against every other file.
        elsewhere = set()
        for other, hs in b["headings"].items():
            if other != name:
                elsewhere |= {key(h) for h in hs}
        reviewed = {key(k) for k in REVIEWED_HEADING_LOSSES}
        real = [h for h in lost
                if key(h) not in elsewhere
                and key(h) not in reviewed
                and not re.search(r"已合入|已撤销|非现行规范|合入记录|本小节【作废】"
                                  r"|已被正文超越|不得按原样合入|已作废|不得合入"
                                  r"|^#+\s*(编辑|补丁|【?块|【[0-9A-Za-z-]+】|六 ·|五 ·)"
                                  r"|^#+\s*E-[0-9]"
                                  r"|类 —— 追加", h)]
        if real:
            rc = 1
            print("  %s: %d headings gone, %d unexpected" % (name, len(lost), len(real)))
            for h in real[:10]:
                print("      UNEXPECTED %s" % h[:90])
        elif lost:
            n_rev = sum(1 for h in lost if key(h) in reviewed)
            note = " (%d hand-reviewed)" % n_rev if n_rev else ""
            print("  %s: %d dead headings removed as intended%s" % (name, len(lost), note))

    print("== size ==")
    ta = sum(v["chars"] for v in a["files"].values())
    tb = sum(v["chars"] for v in b["files"].values())
    print("  %d -> %d chars (%.1f%% smaller)" % (ta, tb, 100.0 * (ta - tb) / ta))
    print("RESULT: %s" % ("PASS" if rc == 0 else "FAIL"))
    return rc


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--diff":
        sys.exit(diff(sys.argv[2], sys.argv[3]))
    json.dump(snapshot(), sys.stdout, ensure_ascii=False)
