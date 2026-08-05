#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sec12_scan.py
Brief: Executable body for SEC-12 -- the three document-surface checks

Description:
SEC-12 (10 §12.4) bundles three document checks -- SP-8 profile/speed pairing,
UL-6 uplink attribution, EC-2 error-code closure -- and states they share one
executable body, one scan, one line-level exemption rule. Until 2026-08-04 no
such body existed anywhere under scripts/, so all three were criteria without an
implementation: they could only ever be evaluated by hand, and were.

The scan surface used to be a list of volume numbers written verbatim in two
places (10 §12.4 and 11 §9.6 SP-8) kept in sync by hand. Across the package
seven such evaluation lists existed, and two separate "count corrections" each
missed one. The surface now lives only in scan_manifest.json; both volumes point
at it instead of repeating it.

Reporting follows the package's own GC-1d rule: a changed hit count is NOT a
failure, only a live hit is -- "live" meaning the line carries no exemption marker
from scan_manifest.exempt_markers. There is no other exemption: a regex that
excused lines for "looking explanatory" was removed on 2026-08-04 after it was
measured hiding 12.6% of the corpus (see explanatory() below). Raw counts are
printed as context, never as the pass/fail signal.

Usage:
  sec12_scan.py            # run all three checks
  sec12_scan.py --check SP-8
"""

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "scan_manifest.json")

# SP-8: a profile name and a speed sitting within 16 chars of each other.
SP8_FWD = re.compile(r"(obstacle_avoid|patrol)([^0-9\n]{0,16}?)([0-9]+(?:\.[0-9]+)?)\s*m/s")
SP8_REV = re.compile(r"([0-9]+(?:\.[0-9]+)?)\s*m/s([^0-9|\n·/、，,;；]{0,16}?)(obstacle_avoid|patrol)")
PROFILE_BLOCK = re.compile(
    r"(obstacle_avoid|patrol)\s*:\s*\{(.{0,400}?)\}", re.S)
NUM_KEY = re.compile(r"(max_mps|up_enter_mps)\s*:\s*([0-9]+(?:\.[0-9]+)?|null)")
# 11 §9.6.2 f() band values -- the lower bound of obstacle_avoid comes from the
# gate, not the profile block (up_enter_mps is null for the lowest profile).
F_BAND = re.compile(r"\|\s*③\s*\|.*?\*\*([0-9]+\.[0-9]+)\s*m/s\*\*")

_SP8_CACHE = {}


def sp8_values(docs_dir):
    """Legal {profile: speeds} read from the contract, never hardcoded.

    SP-8 defines its own value set verbatim: "合法值集（从 §9.6.1 档位表与 json5
    读入，不得硬编码）… 即区间下界（= up_enter_mps）与 max_mps 两值". Until
    2026-08-05 this function was a hardcoded dict whose comment claimed it was
    "cross-checked against the document at run time" -- the cross-check did not
    exist. Both the comment and scan_manifest's "禁硬编码" were describing a
    guarantee the code did not provide, which is the failure mode SP-8 exists to
    catch. It now parses §9.6.1.
    """
    # The legal value set is a property of THE CONTRACT, not of whatever surface
    # is being scanned. Reading it from docs_dir broke --self-test on 2026-08-05:
    # the fixture directory has no 11-接口契约.md, so the run died instead of
    # turning red. A criterion that crashes is not a criterion that passes, but it
    # is also not one that fails -- and "it errored" is exactly the state that gets
    # quietly excluded from a suite later. Always resolve against the manifest.
    contract_dir = load_manifest()["docs_dir"]
    if contract_dir in _SP8_CACHE:
        return _SP8_CACHE[contract_dir]
    text = open(os.path.join(contract_dir, "11-接口契约.md"), encoding="utf-8").read()
    out = {}
    for m in PROFILE_BLOCK.finditer(text):
        name, body = m.group(1), m.group(2)
        vals = set()
        for k, v in NUM_KEY.findall(body):
            if v != "null":
                vals.add(float(v))
        if vals:
            out.setdefault(name, set()).update(vals)
    # The lowest profile has up_enter_mps: null; its lower bound is f()'s band ③.
    band = F_BAND.search(text)
    if band and "obstacle_avoid" in out:
        out["obstacle_avoid"].add(float(band.group(1)))
    if not out:
        raise SystemExit("SP-8: could not read the profile table from 11 §9.6.1 -- "
                         "refusing to fall back to hardcoded values")
    _SP8_CACHE[contract_dir] = out
    return out

UL6 = re.compile(r"(?<![a-z_.])uplink(?![a-z_])")
# Attributed = the word is qualified: a JSON field ("uplink":), a member access
# (state/link.uplink, uplink.est_total_mbps), or quoted as an identifier.
UL6_ATTRIBUTED = re.compile(r'"uplink"|`uplink|uplink\s*[.:{]|\.uplink\b|uplink_[a-z]'
                            r'|uplink 组|uplink\{')

EC2 = re.compile(r"\bE_[A-Z][A-Z0-9_]+")
# "E_STATE_*" names a group of codes, not a code. The trailing * (or 组/族/前缀
# nearby) marks it as a family label; counting it as an unknown code produced 24
# of the first run's 29 false violations.
EC2_FAMILY = re.compile(r"E_[A-Z][A-Z0-9_]*_\*|E_[A-Z][A-Z0-9_]*\*")


def load_manifest():
    with open(MANIFEST, encoding="utf-8") as fh:
        return json.load(fh)


_CODE_CACHE = {}


def closed_error_codes(docs_dir):
    """The E_* closed set, read from the contract's own tables, never hardcoded.

    The code sits in the first cell of each row of the §13.4-§13.15 tables, but the
    cell is not bare: it carries backticks and sometimes a trailing note, e.g.
    "| `E_PROTO_VERSION`<br>（历史命名） | ...". Matching a whole-cell pattern finds
    nothing, which silently turns EC-2 into "every code is out of set".
    """
    if docs_dir in _CODE_CACHE:
        return _CODE_CACHE[docs_dir]
    path = os.path.join(docs_dir, "11-接口契约.md")
    text = open(path, encoding="utf-8").read()
    start = text.find("### 13.4")
    end = text.find("### 13.16")
    if end < 0:
        end = text.find("## 14.")
    body = text[start:end] if start >= 0 else text
    codes = set()
    for line in body.split("\n"):
        if not line.lstrip().startswith("|"):
            continue
        first = line.strip().strip("|").split("|")[0]
        codes.update(EC2.findall(first))
    _CODE_CACHE[docs_dir] = codes
    return codes


def is_exempt(line, markers):
    return any(mk in line for mk in markers)


def explanatory(line):
    """Always False. Kept as a named no-op so the removal stays visible.

    2026-08-04: this function used to carry a regex (判据|豁免|订正|墓碑|原文|旧值|...)
    that silently exempted any line "talking about" a rule. It was invented here and
    appears NOWHERE in the documents. Measured: it hid 7020 of 55744 lines (12.6%)
    from all three checks, and was the only reason SP-8 and EC-2 reported zero.

    That is the exact failure this check exists to catch -- the criterion was made
    green by widening the exemption instead of by fixing the content, and the
    widening was invisible because it lived in code rather than on the line.

    The documented mechanism is the line-level marker (SP-8 豁免 / EC-2 豁免 /
    UL-6 豁免) listed in scan_manifest.exempt_markers: it sits on the line itself,
    so any reader can see that the line was excused and why. A line that genuinely
    is 订正说明 / 墓碑 / 对照 earns a marker -- one visible decision at a time.
    """
    return False


def run_sp8(docs_dir, files, markers):
    live = []
    total = 0
    for name in files:
        for i, line in enumerate(open(os.path.join(docs_dir, name), encoding="utf-8"), 1):
            for m in list(SP8_FWD.finditer(line)) + list(SP8_REV.finditer(line)):
                groups = m.groups()
                profile = groups[0] if groups[0] in sp8_values(docs_dir) else groups[2]
                value = groups[2] if groups[0] in sp8_values(docs_dir) else groups[0]
                total += 1
                # Compare numerically: the docs write the same limit as 0.5 / 0.50
                # and 2.0 / 2.00, so string equality reports false violations.
                try:
                    num = float(value)
                except ValueError:
                    continue
                if any(abs(num - ok) < 1e-9 for ok in sp8_values(docs_dir).get(profile, ())):
                    continue
                if is_exempt(line, markers) or explanatory(line):
                    continue
                live.append((name, i, profile, value, line.strip()[:100]))
    return total, live


def run_ul6(docs_dir, files, markers):
    live = []
    total = 0
    for name in files:
        for i, line in enumerate(open(os.path.join(docs_dir, name), encoding="utf-8"), 1):
            for _ in UL6.finditer(line):
                total += 1
                if UL6_ATTRIBUTED.search(line) or is_exempt(line, markers) or explanatory(line):
                    continue
                live.append((name, i, "-", "-", line.strip()[:100]))
    return total, live


def run_ec2(docs_dir, files, markers):
    closed = closed_error_codes(docs_dir)
    live = []
    total = 0
    for name in files:
        for i, line in enumerate(open(os.path.join(docs_dir, name), encoding="utf-8"), 1):
            fams = {f.rstrip("*") for f in EC2_FAMILY.findall(line)}
            for m in EC2.finditer(line):
                total += 1
                if m.group(0) in closed:
                    continue
                if m.group(0) in fams or m.group(0).endswith("_"):
                    continue          # group prefix such as E_STATE_*
                if is_exempt(line, markers) or explanatory(line):
                    continue
                live.append((name, i, m.group(0), "-", line.strip()[:100]))
    return total, live


CHECKS = {"SP-8": run_sp8, "UL-6": run_ul6, "EC-2": run_ec2}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", choices=sorted(CHECKS), help="run only one check")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--self-test", action="store_true",
                    help="scan the mutant fixture; every planted violation must be caught")
    args = ap.parse_args()

    man = load_manifest()
    docs_dir = man["docs_dir"]
    files = [m["file"] for m in man["members"]]

    if args.self_test:
        # A criterion that has only ever been green is not a criterion (CLAUDE.md 3.3).
        # The fixture plants two violations per check; all six must come back.
        fx_dir = os.path.join(HERE, "fixtures")
        fx = ["sec12_mutants.md"]
        # EC-2 reads the closed set from the contract, which does not live in the
        # fixture dir -- seed the cache with the real one so the mutant codes are
        # judged against the same set the live scan uses.
        _CODE_CACHE[fx_dir] = closed_error_codes(docs_dir)
        expect = {"SP-8": 2, "EC-2": 2, "UL-6": 2}
        rc = 0
        print("self-test against %s" % os.path.join(fx_dir, fx[0]))
        for cname in sorted(CHECKS):
            _, live = CHECKS[cname](fx_dir, fx, man.get("exempt_markers", []))
            want = expect[cname]
            ok = len(live) >= want
            if not ok:
                rc = 1
            print("  %-6s planted %d, caught %d  %s"
                  % (cname, want, len(live), "OK" if ok else "MISSED"))
            for f, ln, a, b, txt in live:
                print("        %s:%d  %s %s  %s" % (f, ln, a, b, txt))
        print("self-test %s" % ("PASS" if rc == 0 else "FAIL -- a check is blind"))
        return rc
    markers = man.get("exempt_markers", [])

    missing = [f for f in files if not os.path.exists(os.path.join(docs_dir, f))]
    if missing:
        print("manifest lists files that do not exist: %s" % ", ".join(missing))
        return 2

    print("scan surface: %d files (manifest v%s, updated %s)"
          % (len(files), man["version"], man["updated"]))
    print("closed error-code set read from contract: %d codes"
          % len(closed_error_codes(docs_dir)))
    print()

    rc = 0
    for name in sorted(CHECKS):
        if args.check and name != args.check:
            continue
        total, live = CHECKS[name](docs_dir, files, markers)
        verdict = "PASS" if not live else "FAIL"
        if live:
            rc = 1
        print("%-6s raw hits %-5d live violations %-4d %s"
              % (name, total, len(live), verdict))
        for f, ln, a, b, txt in live[:15 if args.verbose else 5]:
            print("        %s:%d  %s %s  %s" % (f, ln, a, b, txt))
        if len(live) > 5 and not args.verbose:
            print("        ... %d more (use --verbose)" % (len(live) - 5))
    print()
    print("criterion is live violations == 0; raw hit counts drift with normal"
          " edits and are context only (GC-1d)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
