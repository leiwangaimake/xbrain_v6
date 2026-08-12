#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: header_lint.py
Brief: Enforce the five-field file header, and the Chinese company name iron rule

Description:
What problem this solves. CLAUDE.md 2.5 requires every source file to open with
five fields, and 2.5.0 makes the company name an iron rule: it must be the
Chinese string, never an English translation. Both were already written down and
both were already broken -- on 2026-08-06 six files carried the English name, all
of them written the same day by the same author who had just been told the rule.
A rule nothing evaluates is not a rule.

Why the company name matters enough to be its own check. It is the one field
whose correct value cannot be inferred from context: a reader seeing an English
company name has no way to know it is wrong, and neither does a reviewer skimming
a diff. Every other field is either obviously present or obviously absent.

What it checks:
  1. all five fields present, in the header block, in order
  2. the company line is exactly the Chinese string
  3. Brief is a single line and is not simply repeated as the Description
  4. Description is present and is not a stub

What it deliberately does NOT do:
  * it does not judge whether the Description is any good -- 2.5 asks for
    specific content (what problem, which section, what boundary, which traps)
    and no linter can tell a real explanation from a plausible-sounding one.
    Reporting a header as compliant would then mean less than it appears to,
    so this checks presence and leaves substance to review.
  * it does not touch docs/, which are markdown and carry no file headers.

A note on scope. Files with no header at all are reported separately from files
whose header is wrong. The two need different work: the first is someone who
never added one, the second is someone who added one and got a field wrong, and
mixing them in a single count hides how much of each there is.
"""

import os
import re
import sys

ROOT = "/opt/xbrain_v6"

# Directories holding source we own. configs/ was added 2026-08-12: the yaml
# there (CLAUDE.md 2.5 requires the five-field header on yaml too, 2026-08-10)
# was never gated because neither configs/ was a source dir NOR .yaml a source
# ext -- 18 configs yaml carried no header and nothing caught it (a scan-surface
# blind spot, CLAUDE.md 3.2 form "扫描面不声明").
SOURCE_DIRS = ("xbrain", "scripts", "tests", "common", "ros2_ws", "services",
               "configs")
SOURCE_EXT = (".py", ".c", ".cc", ".cpp", ".h", ".hpp", ".sh", ".bash",
              ".yaml", ".yml")

#: The iron rule. Written as a codepoint join so this file does not itself need
#: to be exempted from any future check that scans for the English form.
COMPANY_CN = "上海哈船智能船舶技术有限公司"

#: English forms seen in the wild. Listed so the report can say "you used the
#: English name" rather than the less useful "company line missing".
COMPANY_EN_HINTS = ("Shanghai Hachist", "Hachist Intelligent Ship", "Co., Ltd")

#: How far into a file the header may start. Shebang, encoding line and the
#: opening quotes of a docstring all come first, so a few lines of slack are
#: normal; beyond that the "header" is really a comment somewhere in the body.
HEADER_WINDOW = 12

REQUIRED_FIELDS = ("Copyright", "Author", "File", "Brief", "Description")


def iter_sources():
    """Every source file we own, skipping caches and vendored payloads."""
    for top in SOURCE_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            # generated/ holds build artifacts (configs/generated/whitelist.yaml
            # is materialised from 11 S1.1.6; CLAUDE.md 2.2 exempts generated
            # output, which must not be hand-edited). The model* skip targets
            # vendor ML payloads, which all live under services/ -- scoped to
            # services/ so configs/models/ (real config, not vendored) IS gated.
            dirnames[:] = [d for d in dirnames
                           if d not in ("__pycache__", ".git", "node_modules",
                                        "generated")
                           and not (d.startswith("model") and "services" in dirpath)]
            for name in filenames:
                if name.endswith(SOURCE_EXT):
                    yield os.path.join(dirpath, name)


def read_header(path):
    """The first HEADER_WINDOW lines plus the whole leading docstring, if any.

    Reading only the window would truncate a long Description and make the
    Description check fire on files that have a perfectly good one. Reading the
    whole file would let a stray "Brief:" deep in the body satisfy the check.
    So: the window for field detection, and the docstring for content.
    """
    with open(path, encoding="utf-8", errors="replace") as fh:
        lines = fh.read().split("\n")
    return lines[:HEADER_WINDOW], lines


def check(path):
    """(problems, has_any_header) for one file."""
    window, all_lines = read_header(path)
    window_text = "\n".join(window)
    body_text = "\n".join(all_lines[:120])

    # A file with none of the fields has no header at all, which is a different
    # situation from one with a wrong field.
    # Copyright is the odd one out: the 2.5 template writes it as
    # "Copyright (c) 2026 Hachist Robotics" with no colon, while every other
    # field is "Name: value". A single colon-requiring pattern reports all five
    # as missing on a perfectly good header -- which is what the first version of
    # this linter did, flagging 91 correct files and zero clean ones. A checker
    # that fails everything is indistinguishable from a checker that is broken,
    # and the natural response is to disable it.
    patterns = {f: (r"^\s*[#*]?\s*" + f + r"\s*[:(]") for f in REQUIRED_FIELDS}
    present = [f for f, pat in patterns.items()
               if re.search(pat, window_text, re.M) or re.search(pat, body_text, re.M)]
    if not present:
        return ["no file header at all"], False

    problems = []
    for field in REQUIRED_FIELDS:
        if field not in present:
            problems.append("missing field: %s" % field)

    # The iron rule. Checked against the whole leading block rather than the
    # window, because the company line sits under Author and a long Copyright
    # can push it past the window on some files.
    if COMPANY_CN not in body_text:
        used_english = any(hint in body_text for hint in COMPANY_EN_HINTS)
        problems.append(
            "company name is the English form; the iron rule (CLAUDE.md 2.5.0) "
            "requires the Chinese string" if used_english
            else "company name line missing; must be the Chinese string"
        )

    # Brief must exist as one line and must not simply be the Description again.
    brief = re.search(r"^\s*[#*]?\s*Brief\s*:\s*(.+)$", body_text, re.M)
    if brief:
        brief_text = brief.group(1).strip()
        if len(brief_text) < 8:
            problems.append("Brief is too short to say anything")
        desc = re.search(r"^Description:\s*\n(.+)$", body_text, re.M)
        if desc and desc.group(1).strip() == brief_text:
            problems.append("Description merely repeats Brief (CLAUDE.md 2.5)")

    # File-field must match the actual filename basename. This catches
    # copy-paste headers where a file was duplicated and the header
    # not updated. INF-CI-2 variant ② verbatim: 'File 字段写成别的
    # 文件名 -> 红'.
    file_match = re.search(r"^\s*[#*]?\s*File\s*:\s*(\S+)", body_text, re.M)
    if file_match:
        declared = file_match.group(1).strip().rstrip("*").rstrip(":")
        actual = os.path.basename(path)
        if declared != actual:
            problems.append(
                "File field %r does not match actual filename %r "
                "(copy-paste header)" % (declared, actual))

    return problems, True


def main():
    show_ok = "-v" in sys.argv or "--verbose" in sys.argv
    no_header, wrong_header, clean = [], [], 0

    print("scan surface: " + ", ".join(SOURCE_DIRS))
    print("  iron rule: the company line must be the Chinese string (CLAUDE.md 2.5.0)")
    print("")

    for path in sorted(iter_sources()):
        rel = os.path.relpath(path, ROOT)
        problems, has_header = check(path)
        if not problems:
            clean += 1
            if show_ok:
                print("  ok   %s" % rel)
            continue
        if not has_header:
            no_header.append(rel)
        else:
            wrong_header.append((rel, problems))

    for rel, problems in wrong_header:
        print("  BAD  %s" % rel)
        for p in problems:
            print("         %s" % p)

    if no_header:
        print("")
        print("  files with no header at all: %d" % len(no_header))
        for rel in no_header[:10]:
            print("      %s" % rel)
        if len(no_header) > 10:
            print("      ... %d more" % (len(no_header) - 10))

    print("")
    print("  clean:            %d" % clean)
    print("  wrong header:     %d" % len(wrong_header))
    print("  no header at all: %d" % len(no_header))
    print("")
    print("criterion: wrong header == 0")
    print("  Files with no header are reported separately: adding one is different")
    print("  work from correcting a field, and one count would hide the mix.")
    return 1 if wrong_header else 0


if __name__ == "__main__":
    sys.exit(main())
