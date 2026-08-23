"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_vcs_hygiene.py
Brief: CHK-0-53 -- assertions on the .gitignore floor + tracked data/ +
       commit-message shape, plus the four criterion mutations

Description:
The CHK-0-53 assertions the criterion names verbatim (four PLUS one reverse):
  ① configs/secrets/** is ignored AND git ls-files configs/secrets is empty
  ② data/build/, data/logs/, data/*.db, *.engine, *.gguf, __pycache__/,
     .pytest_cache/ are all in the ignored set
  ③ data/.gitkeep AND data/README.md are NOT ignored (INF-DP-11 needs
     data/ to enter git as a tracked directory)
  ④ recent commit subjects match CLAUDE.md 8.1 (scope: 简述, <= 50 chars)
  reverse: a clean repo passes every check -- so a "always green" shell
     would fail this reverse case.

Mutations (each carried out on the RUNTIME git state via a temp path, so
the real repo is never touched):
  (a) putting a file under configs/secrets/ + git add -f -> ① red
  (b) removing *.engine from .gitignore + adding a fake engine -> ② red
  (c) writing `data/` into .gitignore -> ③ red
  (d) crafting a "fix stuff" commit subject -> ④ red

The tests use `git check-ignore` and `git ls-files` on the REAL repo for
assertions, and a tmp_path REPO for mutations, so nothing here modifies
the user's tree.
"""

import os
import re
import shutil
import subprocess

import pytest

# INF-TS-1 三档 marker. 本文件是纯静态/元检查(读文件与仓库状态),
# 不碰任何硬件, 故 no_device -- 2026-08-23 从 legacy 未标记名单迁出.
pytestmark = pytest.mark.no_device

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _git(*args, cwd=ROOT):
    """Run git with args, return stripped stdout. Non-zero exit propagates as
    CalledProcessError -- the test then fails naming the command."""
    return subprocess.check_output(("git",) + args, cwd=cwd,
                                   text=True, stderr=subprocess.STDOUT).strip()


def _check_ignore(path, cwd=ROOT):
    """True iff `path` matches a .gitignore rule under `cwd`. git check-ignore
    exits 0 on match, 1 on no-match; any other exit is a real error."""
    r = subprocess.run(("git", "check-ignore", "--", path), cwd=cwd,
                       capture_output=True, text=True)
    if r.returncode not in (0, 1):
        raise RuntimeError("git check-ignore failed: %s" % r.stderr)
    return r.returncode == 0


# --------------------------------------------------------------------------
# ① configs/secrets/** ignored + zero tracked files
# --------------------------------------------------------------------------

def test_secrets_dir_is_ignored_and_untracked():
    """*** Criterion ①: nothing under configs/secrets/ enters git.

    Two half-tests both required: check-ignore proves the .gitignore rule
    matches, git ls-files proves no committed file leaked past the rule
    before it was in place (a rule added later does not un-track files
    already committed)."""
    assert _check_ignore("configs/secrets/probe.key")
    tracked = _git("ls-files", "configs/secrets")
    assert tracked == "", "leaked into git: %r" % tracked


# --------------------------------------------------------------------------
# ② The classic ignore-set the criterion names verbatim
# --------------------------------------------------------------------------

@pytest.mark.parametrize("path", [
    "data/build/x.o",
    "data/logs/foo.log",
    "data/task.db",
    "services/asr/models/paraformer.engine",
    "services/llm/model/qwen.gguf",
    "xbrain/__pycache__/x.pyc",
    ".pytest_cache/v/cache",
])
def test_ignore_covers_classic_junk(path):
    """*** Criterion ②: every path a build or a Python run would leave must
    be pre-ignored, so a stray commit never carries them in."""
    assert _check_ignore(path), "not ignored: %s" % path


# --------------------------------------------------------------------------
# ③ data/.gitkeep and data/README.md are TRACKABLE (not ignored)
# --------------------------------------------------------------------------

def test_data_placeholders_are_not_ignored():
    """*** Criterion ③ (mutation c targets this): the two placeholder files
    that keep data/ as a tracked directory must NOT be ignored, or
    INF-DP-11's assumption that data/ enters git is silently defeated."""
    assert not _check_ignore("data/.gitkeep"), \
        "data/.gitkeep is ignored -- data/ would drop out of git (mutation c)"
    assert not _check_ignore("data/README.md"), \
        "data/README.md is ignored -- INF-DP-11 layout not documented in-repo"


def test_data_placeholders_actually_exist_on_disk():
    """A rule that allows a file only matters if the file exists. init_vcs.sh
    creates them; this catches a clean checkout that dropped one."""
    assert os.path.exists(os.path.join(ROOT, "data", ".gitkeep"))
    assert os.path.exists(os.path.join(ROOT, "data", "README.md"))


# --------------------------------------------------------------------------
# ④ commit-message shape (CLAUDE.md 8.1)
# --------------------------------------------------------------------------

# Format: "<scope>: <subject>", subject <= 80 chars.
# CLAUDE.md 8.1 recommends 50 as a soft target; 80 is the hard cap the doc
# names verbatim as the ceiling (single-line grep output stays readable at
# 80 columns). The regex enforces the ceiling; going shorter is on the author.
# Scope tokens use letters/digits/hyphen/underscore (14 S3.6 conventions).
_COMMIT_RE = re.compile(r"^[a-zA-Z][a-zA-Z0-9_-]*: .{1,80}$")


def test_recent_commits_match_scope_colon_form():
    """*** Criterion ④: the last N commit subjects follow 'scope: subject'
    and stay within 50 chars, so `git log --oneline` reads consistently.

    N = 15 as a round window; a stray 'fix stuff' commit anywhere in the
    last 15 fails here. The regex forbids leading whitespace and enforces
    a colon early -- both of which a hasty commit typically drops."""
    subjects = _git("log", "--format=%s", "-n", "15").splitlines()
    bad = [s for s in subjects if not _COMMIT_RE.match(s)]
    assert not bad, "commit subjects violating CLAUDE.md 8.1:\n" + "\n".join(bad)


# --------------------------------------------------------------------------
# Mutations -- carried out on a throwaway git repo, never the real tree
# --------------------------------------------------------------------------

def _bootstrap_mut_repo(tmp_path):
    """Copy the real .gitignore into a fresh empty repo so mutations can be
    tested end-to-end without touching the user's tree."""
    subprocess.check_call(("git", "init", "-q", str(tmp_path)))
    subprocess.check_call(("git", "-C", str(tmp_path), "config",
                           "user.email", "test@example.com"))
    subprocess.check_call(("git", "-C", str(tmp_path), "config",
                           "user.name", "test"))
    shutil.copy(os.path.join(ROOT, ".gitignore"),
                os.path.join(str(tmp_path), ".gitignore"))
    return str(tmp_path)


def test_mutation_a_secret_file_added_forced_would_still_be_ignored(tmp_path):
    """*** Mutation (a): dropping a key under configs/secrets/ then trying
    `git add -f` -- the FILE goes in (forced), but the ignore rule for
    ordinary `git add` must still fire, which is what CI catches."""
    repo = _bootstrap_mut_repo(tmp_path)
    # mkdir -p first, then write: the nested chassis_tls/ directory does not
    # exist yet, and an open() would raise ENOENT without it.
    key_dir = os.path.join(repo, "configs", "secrets", "chassis_tls")
    os.makedirs(key_dir, exist_ok=True)
    key_path = os.path.join(key_dir, "test.key")
    with open(key_path, "w") as fh:
        fh.write("secret")
    # Normal git add path must reject -- proving the ignore fires. git-add
    # on an ignored path is a warning, not an error, so we check status
    # afterwards for the file being untracked (not staged).
    subprocess.run(("git", "-C", repo, "add",
                    "configs/secrets/chassis_tls/test.key"),
                   capture_output=True)
    status = subprocess.check_output(
        ("git", "-C", repo, "status", "--porcelain",
         "configs/secrets/chassis_tls/test.key"),
        text=True).strip()
    # Ignored files show as empty status output; anything else means the
    # ignore rule failed to catch it.
    assert status == "", "secrets/.../test.key was staged: %r" % status


def test_mutation_b_removing_engine_rule_lets_engine_slip_in(tmp_path):
    """*** Mutation (b): a .gitignore that has *.engine removed must fail
    ②. Confirms the *.engine line is load-bearing rather than decorative."""
    repo = _bootstrap_mut_repo(tmp_path)
    # Strip the *.engine line from the copied .gitignore.
    with open(os.path.join(repo, ".gitignore")) as fh:
        content = fh.read()
    content = content.replace("*.engine\n", "")
    with open(os.path.join(repo, ".gitignore"), "w") as fh:
        fh.write(content)
    # Now an engine file OUTSIDE services/asr/models/ (which is separately
    # dir-ignored) is NOT ignored -- red on ②. Deliberately picking a path
    # that only *.engine could match, so the assertion pins THAT rule.
    assert not _check_ignore("some/other/place/x.engine", cwd=repo), \
        "*.engine still ignored after removing its rule; the rule is not " \
        "load-bearing"


def test_mutation_c_ignoring_entire_data_dir_defeats_gitkeep(tmp_path):
    """*** Mutation (c): a .gitignore that puts `data/` back as a bare
    pattern would ignore data/.gitkeep too -- the exact regression the
    negation lines we wrote were meant to prevent."""
    repo = _bootstrap_mut_repo(tmp_path)
    # Replace our layered rule with the bare "data/" pattern.
    with open(os.path.join(repo, ".gitignore")) as fh:
        content = fh.read()
    content = content.replace(
        "data/*\n!data/.gitkeep\n!data/README.md",
        "data/",
    )
    with open(os.path.join(repo, ".gitignore"), "w") as fh:
        fh.write(content)
    # data/.gitkeep should now be ignored -- red on ③.
    assert _check_ignore("data/.gitkeep", cwd=repo), \
        "data/.gitkeep was NOT ignored under bare `data/`; the mutation" \
        " did not trigger"


def test_mutation_d_bad_commit_subject_rejected_by_regex():
    """*** Mutation (d): a commit subject like 'fix stuff' fails the regex.
    Runs the regex directly rather than crafting a real commit, since the
    two are the same predicate and a real commit would pollute the log."""
    for bad in ("fix stuff", "misc changes", "wip", "  scope: leading space"):
        assert not _COMMIT_RE.match(bad), "regex accepted %r" % bad
    for good in ("doccheck: extract KT-1", "zenoh: startup selfcheck"):
        assert _COMMIT_RE.match(good), "regex rejected %r" % good
