#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: charset_lint.py
Brief: Enforce CLAUDE.md 2.2 -- ASCII punctuation and no special symbols in source

Description:
What problem this solves. CLAUDE.md 2.2 has forbidden emoji, star markers and
CJK punctuation in source files since the project started, and on 2026-08-06 a
self-check found the rule had been broken in 18 of our own .py files. The
first estimate quoted here was wrong -- a shell-quoting artifact inflated the
CJK-punctuation count. The real figures come from this script, never from a
hand-run grep. A rule nothing evaluates is not a rule, it is a wish.

What it checks, per CLAUDE.md 2.2 and the 2026-08-06 restatement:
  1. no emoji or decorative symbols anywhere in a source file, comments included
  2. ASCII punctuation only, in Chinese comments as well as English ones
  3. Chinese characters themselves stay allowed -- 2.1 lets a file use Chinese
     comments as long as it is consistent; what is banned is CJK punctuation and
     decorative marks, not Chinese words

What it deliberately does NOT do:
  * it does not touch docs/, because 2.2 exempts markdown by design -- the eleven
    design volumes use star and warning markers as a severity scale on purpose
  * it does not check comment ratio, that is comment_ratio.py
  * it does not rewrite anything; a linter that edits code hides what it changed

A trap worth naming. An earlier draft of this scan reported per-file counts only.
Counts tell you a file is dirty but not which line to open, and a developer who
cannot find the offending character gives up and adds an exemption. So every hit
is reported with file, line, column and the character itself.
"""

import os
import re
import sys
import unicodedata

ROOT = "/opt/xbrain_v6"

# Directories that hold source we own. services/ is included: it is our code too.
# configs/ added 2026-08-12: CLAUDE.md 2.2 (2026-08-10) put .yaml under this rule,
# but configs/ was never a source dir nor .yaml a source ext, so configs yaml were
# never scanned by default (only when an arg was passed, and this main() ignores
# file args) -- a scan-surface blind spot (CLAUDE.md 3.2).
#
# deploy/ added 2026-09-30, the same blind spot one level up. The tree holds
# eighteen systemd units, two zenoh router configs, the nftables and networkd
# drops and the logrotate rule -- all written and maintained by us, so all of
# them are inside CLAUDE.md 2.2's declared surface ("我方书写并维护的源文件与
# 配置"). Not one of them had ever been read by this script, and three of the
# units carry star markers and CJK punctuation in their header comments.
SOURCE_DIRS = ("xbrain", "scripts", "tests", "common", "ros2_ws", "services",
               "configs", "deploy")

# CLAUDE.md 2.2's 2026-08-11 iron rule covers EVERY file that is not .md /
# .pdf / .doc(x), and names .json / .json5 explicitly. The list stayed at ten
# extensions for seven weeks after that ruling, so the document asserted a
# check that did not exist -- CLAUDE.md 3.2's fourth shape ("拿未兑现的逐字当
# 证据"), this time in CLAUDE.md itself. The rule is the user's, so the lint
# moves to meet it rather than the other way round.
#
# .service and .mount are here on the same rule and are NOT named by it: the
# ruling's list ends in 等 and its stated scope is every non-document file we
# write. A systemd unit is a configuration file we author, review and ship, it
# carries a # comment block exactly like the .sh and .yaml the 2026-08-10
# ruling added, and deploy/systemd is where three of this repository's six
# dirtiest files turned out to be. Leaving unit files out would have meant the
# scan surface stopped precisely where the violations were.
SOURCE_EXT = (".py", ".c", ".cc", ".cpp", ".h", ".hpp", ".sh", ".bash",
              ".yaml", ".yml", ".json", ".json5", ".service", ".mount")

# Decorative symbols the project uses in markdown and forbids in source.
# Listed explicitly rather than by Unicode block: a block test would also catch
# characters that are legitimate in a comment, and an over-broad linter gets
# switched off.
# * Defined by codepoint, not by literal character. A linter written with the
# characters it forbids reports itself on every run; this project has caught
# that self-harming-criterion shape three times. Escapes keep this file clean.
BANNED_SYMBOLS = "".join(chr(c) for c in (
    0x2605, 0x2606, 0x26a0, 0x1f6ab, 0x2705, 0x274c, 0x1f534, 0x2611, 0x2610,
    0x2192, 0x21d2, 0x2326, 0x25cf, 0x25c6, 0x25a0, 0x25b2, 0x25bc, 0x2714, 0x2718,
))

# CJK and full-width punctuation. ASCII replacements exist for every one of them.
BANNED_PUNCT = "".join(chr(c) for c in (
    0xff0c, 0x3002, 0xff1b, 0xff1a, 0xff1f, 0xff01,
    0x201c, 0x201d, 0x2018, 0x2019,
    0xff08, 0xff09, 0x3010, 0x3011, 0x300a, 0x300b,
    0x3001, 0x2014, 0x2026, 0x00b7,
    0x300c, 0x300d, 0xff5e, 0xff0e, 0xff1d, 0xff06, 0xff5c,
))

_BANNED = BANNED_SYMBOLS + BANNED_PUNCT
_BANNED_RE = re.compile("[" + re.escape(_BANNED) + "]")

# Emoji and pictographs beyond the named list, so a symbol nobody thought of
# still gets caught. Restricted to the pictograph planes to avoid false hits on
# ordinary CJK text.
_EMOJI_RE = re.compile(
    "[\U0001f000-\U0001faff\u2600-\u27bf\ufe0f\U0001f1e6-\U0001f1ff]"
)


# Trees whose STRING LITERALS are frozen. User decision 2026-08-06: services/
# and the doccheck scripts are finalised and their Chinese output strings are not
# to be rewritten. Those strings are still a CLAUDE.md 2.1 defect (logs and
# messages must be English) and they stay reported as declared debt.
#
# * Why declared debt and not an ignore. If frozen trees were simply skipped the
# criterion "violations == 0" would be unreachable, and an unreachable criterion
# gets loosened until it passes -- this project has caught that shape three times.
# So the criterion is narrowed to what is actually enforceable today (comments)
# and the rest is printed every run with its reason attached.
FROZEN_STRING_TREES = {
    "services": "AI services are finalised and validated (ASR measured 92 percent "
                "on real speech); rewriting their output strings would change "
                "behaviour tests depend on",
    "scripts/doccheck": "document-check scripts print Chinese reports for humans; "
                        "frozen with the design package on 2026-08-06",
    "scripts/progress.py": "same as doccheck -- prints a Chinese progress report",
    "tests": "test assertion messages belong to the frozen suites above",
}


def is_frozen(rel_path):
    """The reason this file's string literals are exempt, or None.

    Python only, deliberately. Every reason in FROZEN_STRING_TREES is about a
    Chinese string a PROGRAM prints -- a doccheck report, a progress line, an
    assertion message -- and the exemption exists because rewriting those would
    change behaviour a test depends on. A data file has no such argument: it
    prints nothing and no test reads its punctuation.
      Applying the tree prefix to data files was a live hole from the moment
    .json joined the surface. scripts/doccheck/scan_manifest.json carried 141
    violations and would have been filed as "declared debt" purely because it
    sits next to the frozen scripts -- an extension that reports nothing and
    fails on nothing, which is CLAUDE.md 3.2's first shape (an assertion a
    do-nothing implementation also passes). The manifest is hand-maintained
    ("新增文档 = 在 members 里加一行"), so it is ours to clean, and it was.
    """
    if not rel_path.endswith(".py"):
        return None
    for prefix, why in FROZEN_STRING_TREES.items():
        if rel_path == prefix or rel_path.startswith(prefix + os.sep):
            return why
    return None


def comment_lines_of(path):
    """Line numbers that hold comments or docstrings, so string literals can be
    told apart from comments. Non-Python falls back to whole-line comments.

    classify_lines answers None for anything that is not Python, and this
    function's whole-line fallback is what every other extension gets. That
    split is load-bearing rather than an optimisation -- see the guard's own
    note in charset_fix.classify_lines for what the Python tokenizer does to a
    .json5 file, and why a wrong answer there is silent rather than loud.
    """
    try:
        import charset_fix
        c, _s = charset_fix.classify_lines(path)
        if c is not None:
            return c
    except Exception:
        pass
    out = set()
    with open(path, encoding="utf-8", errors="replace") as fh:
        for i, ln in enumerate(fh, 1):
            if ln.lstrip().startswith(("#", "//", "*", "/*")):
                out.add(i)
    return out


# Third-party frozen snapshots we did NOT write. CLAUDE.md 2.2 scopes the scan
# to sources we write and maintain ("扫描面 = 我方书写并维护的源文件与配置");
# these trees are another engineer's delivery, frozen for review (perception
# README: copy differences are recorded in reports/SNAPSHOT_IDENTITY.json --
# editing them breaks snapshot identity).
# * Why exclusion and not FROZEN_STRING_TREES: that list marks OUR strings we
# chose not to rewrite (declared debt); these files are not ours at all, so
# reporting them as our debt would misstate ownership. The cleanup request is
# on the delivering engineer (docs/perception-rns-interface-20260907.md item 9)
# and this entry must be REMOVED when the tree is adopted into our maintenance.
THIRD_PARTY_SNAPSHOTS = (
    "ros2_ws/perception",
    "ros2_ws/Perception_Gemini338Le_20260907",
    # Vendored third-party dependencies we did NOT write (99 U85 item 6,
    # 2026-09-15): common/third_party/<name>/ holds an unmodified upstream
    # copy plus its LICENSE (today: nlohmann/json v3.11.3, MIT). It is not
    # ours to re-punctuate or to give a Hachist header, and a hand edit
    # would break the upstream identity; the sha256 lives in the README
    # beside the vendored file (common/third_party/<name>/README.md). Consumers must keep it OUT of realtime
    # threads (13 QD-7): JSON parsing belongs to chs_a_rx, never ctrl.
    "common/third_party",
)

# Directory names CLAUDE.md 2.2 puts outside the surface wherever they appear.
#
# golden: CLAUDE.md 2.2 exempts the Chinese-under-test inside golden vectors,
# tests/**/golden/**, because that data is what a recogniser or a codec is
# compared against and cleaning its punctuation would break the reference.
#   (Quoted in translation, not verbatim: the ruling's own sentence uses the
# lenticular brackets this script forbids, so transcribing it here would make
# the file fail its own check -- CLAUDE.md 3.2's third shape, a criterion that
# harms itself. Anchor for grep in CLAUDE.md 2.2: the phrase golden 测试向量.)
# The ruling draws a line this script
# has to respect in both directions: a spoken line WE author and the device
# plays back is our output and gets ASCII punctuation (configs/speech_presets
# .yaml), while a line a recogniser is measured against is the INPUT and keeps
# whatever punctuation the corpus really has. Re-punctuating the second kind
# does not clean it, it silently changes what the test proves.
#   Matched by directory NAME rather than by the literal tests/**/golden/**
# path, because ros2_ws/quadruped/test/golden holds the same kind of data under
# a different parent and the ruling is about what the bytes ARE, not where they
# sit. Naming a directory `golden` for anything other than frozen vectors is
# the one way to misuse this, and it is visible in review.
#   This costs nothing today -- the four golden trees are clean -- and that is
# the point of writing it down now: the exclusion is a statement of contract,
# not cover for an existing violation.
EXCLUDED_DIR_NAMES = ("golden",)

# Generated files, by basename, wherever they appear. CLAUDE.md 2.2 excludes
# "运行期生成物 ... MANIFEST.json (freeze 产出, 不手改)" and iron rule 2 makes
# hand-editing a generated artefact a rule violation in itself -- so reporting
# one here would be an instruction to do the forbidden thing. compile_commands
# .json is the same shape from CMake: it appears under ros2_ws/*/build, it is
# gitignored, and it is rewritten by every configure.
#   Excluded by BASENAME, not by skipping build/ wholesale. build/ currently
# holds 58 .yaml fixtures that this script already scans and keeps clean;
# pruning the directory would shrink an existing surface to solve a problem
# that only three generated files have (CLAUDE.md 3.2's sixth shape -- a
# surface that quietly got smaller is worse than one that was never there).
EXCLUDED_BASENAMES = ("MANIFEST.json", "compile_commands.json")

# The ASR golden corpus, named separately from EXCLUDED_DIR_NAMES because it
# does not live in a directory called golden. CLAUDE.md 2.2 lists "ASR 金标语料"
# beside tests/**/golden/** for the same reason: it is the speech the recogniser
# is scored against, so its punctuation is measurement data, not our prose.
EXCLUDED_PATHS = ("services/asr/selftest/gold.json",)


def _is_excluded_file(rel_path, name):
    """True when CLAUDE.md 2.2 puts this file outside the scan surface.

    Kept as one predicate so the three reasons stay listed in one place. A
    caller that grew its own inline check is how a scan surface stops being
    declarable, which 11 S15.6F.3 wants stated on every run.
    """
    if name in EXCLUDED_BASENAMES:
        return True
    return rel_path in EXCLUDED_PATHS


def iter_sources():
    """Every source file we own, with docs/ and vendor trees excluded."""
    for top in SOURCE_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for dirpath, dirnames, filenames in os.walk(base):
            rel_dir = os.path.relpath(dirpath, ROOT)
            if any(rel_dir == t or rel_dir.startswith(t + os.sep)
                   for t in THIRD_PARTY_SNAPSHOTS):
                dirnames[:] = []
                continue
            # Skip caches, generated build artifacts, and vendored payloads.
            # generated/ = materialised output (configs/generated/whitelist.yaml,
            # CLAUDE.md 2.2 exempts it, hand-editing forbidden). The model* skip
            # is vendor ML payloads under services/ -- scoped to services/ so
            # configs/models/ (real config, not vendored) IS scanned.
            dirnames[:] = [d for d in dirnames
                           if d not in ("__pycache__", ".git", "node_modules",
                                        "generated")
                           and d not in EXCLUDED_DIR_NAMES
                           and not (d.startswith("model") and "services" in dirpath)]
            for name in filenames:
                if not name.endswith(SOURCE_EXT):
                    continue
                rel = os.path.relpath(os.path.join(dirpath, name), ROOT)
                if _is_excluded_file(rel, name):
                    continue
                yield os.path.join(dirpath, name)


def scan(path):
    """(lineno, col, char, name) for every banned character in one file."""
    hits = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            for col, ch in enumerate(line, 1):
                if _BANNED_RE.match(ch) or _EMOJI_RE.match(ch):
                    try:
                        name = unicodedata.name(ch)
                    except ValueError:
                        name = "UNNAMED"
                    hits.append((lineno, col, ch, name))
    return hits


def main():
    show_all = "-v" in sys.argv or "--verbose" in sys.argv
    in_comments, in_frozen_strings, other_strings, dirty = 0, 0, 0, 0
    print("scan surface: " + ", ".join(SOURCE_DIRS) + " (docs/ exempt per CLAUDE.md 2.2)")
    for path in sorted(iter_sources()):
        hits = scan(path)
        if not hits:
            continue
        rel = os.path.relpath(path, ROOT)
        comment_lines = comment_lines_of(path)
        frozen_why = is_frozen(rel)

        c_hits = [h for h in hits if h[0] in comment_lines]
        s_hits = [h for h in hits if h[0] not in comment_lines]
        in_comments += len(c_hits)
        if frozen_why:
            in_frozen_strings += len(s_hits)
        else:
            other_strings += len(s_hits)

        if not c_hits and not (s_hits and not frozen_why):
            continue          # only frozen-string debt here, summarised below
        dirty += 1
        print("  %-52s comments %3d  strings %3d%s"
              % (rel, len(c_hits), len(s_hits), "  [frozen]" if frozen_why else ""))
        for lineno, col, ch, name in ((c_hits + s_hits) if show_all else (c_hits + s_hits)[:3]):
            print("      %s:%d:%d  %r  %s" % (rel, lineno, col, ch, name))

    print("")
    print("  in comments (enforced):        %d" % in_comments)
    print("  in strings, frozen trees:      %d   [declared debt, see FROZEN_STRING_TREES]"
          % in_frozen_strings)
    print("  in strings, everywhere else:   %d" % other_strings)
    print("")
    print("criterion: in-comment violations == 0 AND non-frozen string violations == 0")
    print("  Frozen string literals stay a CLAUDE.md 2.1 defect (messages must be")
    print("  English). They are declared debt with a reason, not an ignore -- an")
    print("  unreachable criterion gets loosened until it passes.")
    return 1 if (in_comments or other_strings) else 0


if __name__ == "__main__":
    sys.exit(main())
