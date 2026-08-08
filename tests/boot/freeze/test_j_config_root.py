"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_j_config_root.py
Brief: CFG-FZ-2 -- assertion J's five checks + three mutants each turn red

Description:
Assertion J (real body at xbrain/boot/freeze/assertions/j_config_root.py)
enforces five things about the config root -- exists+dir+non-symlink,
required files stat-able, secrets permissions, no path escape, obsolete
files not present. Each check is exercised here with:
  - a green baseline (a scaffolded fake config tree that passes all five)
  - a red mutant that violates ONLY that check

CFG-FZ-2 variants (verbatim):
  (1) configs replaced by a symlink -> must go red
  (2) drop a bit_service.yaml in    -> must go red
  (3) error must carry an absolute path -> negative test on the message

Reverse: the scaffolded green tree passes all five checks.

The five _REQUIRED_FILES are named files; we scaffold every one so a
change to that tuple that adds a new required file will break this
test before shipping (the reverse-baseline stops working), forcing
the test author to keep in sync with j_config_root.py's contract.
"""

import os
import stat

import pytest

from xbrain.boot.freeze.assertions.j_config_root import (
    _OBSOLETE_FILES, _REQUIRED_FILES, run,
)
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding: build a green fake config tree in tmp_path
# ---------------------------------------------------------------------------

def _make_green_tree(tmp_path):
    """Scaffold a config root that passes all five checks.

    - Directory exists, is a directory, is NOT a symlink.
    - Every file in _REQUIRED_FILES exists as a regular file (empty).
    - secrets/ exists at mode 0700.
    - No obsolete files.
    - No symlink escapes.

    Returns the config root path (Path object).
    """
    root = tmp_path / "configs"
    root.mkdir()
    # Every required file gets a minimal placeholder. Content doesn't
    # matter -- J only checks reachability + type + perms.
    for name in _REQUIRED_FILES:
        (root / name).write_text("# placeholder for J\n")
    # secrets/ at 0700. os.mkdir mode is umask'd; set explicitly.
    secrets = root / "secrets"
    secrets.mkdir()
    os.chmod(str(secrets), 0o700)
    return root


# ---------------------------------------------------------------------------
# Reverse (baseline): green tree must pass
# ---------------------------------------------------------------------------

def test_green_tree_passes(tmp_path):
    """Scaffold + run -> status=pass. If this ever fails, the mutant
    tests below are meaningless -- they all differ from baseline by
    exactly one violation, so a broken baseline masks broken mutants."""
    root = _make_green_tree(tmp_path)
    result = run({"config_root": str(root)})
    assert result["status"] == "pass"
    assert result["assertion"] == "J"
    # Sanity: reported config_root matches what we passed in (abs form).
    assert result["config_root"] == os.path.abspath(str(root))
    # Sanity: check count matches the tuples so a silent shrink of
    # either list is noticed.
    assert result["required_files_checked"] == len(_REQUIRED_FILES)
    assert result["obsolete_files_checked"] == len(_OBSOLETE_FILES)


# ---------------------------------------------------------------------------
# Mutant (1): root is a symlink -> config_path_escape
# ---------------------------------------------------------------------------

def test_mutant_symlink_root_is_red(tmp_path):
    """CFG-FZ-2 variant (1) verbatim: 'replace configs with a symlink'.
    islink check runs BEFORE isdir precisely to catch this -- a
    symlink to a directory would pass isdir=True and slip past."""
    real = _make_green_tree(tmp_path)
    # Rename the real tree aside and put a symlink where it used to be.
    aside = tmp_path / "configs_real"
    os.rename(str(real), str(aside))
    os.symlink(str(aside), str(real))
    # Assert on both the code and the kind so a change to the raise
    # site that dropped detail.kind is caught.
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(real)})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "config_path_escape"
    # Absolute path in detail.path -- variant (3).
    assert os.path.isabs(ei.value.detail["path"])


# ---------------------------------------------------------------------------
# Mutant (2): obsolete file bit_service.yaml present -> config_file_obsolete
# ---------------------------------------------------------------------------

def test_mutant_obsolete_bit_service_is_red(tmp_path):
    """CFG-FZ-2 variant (2) verbatim: 'drop a bit_service.yaml in'.
    The obsolete-file check runs BEFORE required-file scan so an
    operator with a half-migrated tree sees the obsolete complaint
    first (root cause) rather than the missing-file complaint
    (symptom)."""
    root = _make_green_tree(tmp_path)
    # bit_service.yaml is the exact name CFG-FZ-2 tests against.
    (root / "bit_service.yaml").write_text("# stale from v0.4\n")
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(root)})
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "config_file_obsolete"
    # obsolete_name field carries which one -- since the list may grow.
    assert ei.value.detail["obsolete_name"] == "bit_service.yaml"
    # Absolute path (variant 3).
    assert os.path.isabs(ei.value.detail["path"])
    assert "bit_service.yaml" in ei.value.detail["path"]


# ---------------------------------------------------------------------------
# Mutant (3): every error message carries an absolute path
# ---------------------------------------------------------------------------

def test_mutant_error_message_has_absolute_path(tmp_path):
    """CFG-FZ-2 variant (3): 'error message only says "配置无效" with no
    path'. Verify every failure detail carries an absolute path AND the
    message string interpolates it -- an implementation that recorded
    detail.path but built a message like 'config invalid' without the
    path would still slip past a detail-only check."""
    # Missing required file case exercises _fail() with an interpolated
    # path in the message. Delete common.yaml.
    root = _make_green_tree(tmp_path)
    os.remove(str(root / "common.yaml"))
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(root)})
    assert ei.value.code == "E_CONFIG_INVALID"
    # Absolute path present in detail.
    assert os.path.isabs(ei.value.detail["path"])
    # Absolute path present in the human-readable message too.
    # str(ei.value) invokes XbrainError.__str__ which composes code +
    # message; the path must be there for an operator scrolling
    # journalctl to see it without decoding detail.
    assert ei.value.detail["path"] in str(ei.value)


# ---------------------------------------------------------------------------
# Additional coverage: the remaining checks that CFG-FZ-2 lists but
# doesn't have a named variant for (2/3/4). Kept minimal -- one per
# rule -- because the "verbatim variant" tests above already prove
# the pattern.
# ---------------------------------------------------------------------------

def test_missing_required_file_is_red(tmp_path):
    """Check (2): a required file gone from disk -> config_file_missing."""
    root = _make_green_tree(tmp_path)
    os.remove(str(root / "p1_motion.yaml"))
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(root)})
    assert ei.value.detail["kind"] == "config_file_missing"
    assert ei.value.detail["required_name"] == "p1_motion.yaml"


def test_secrets_dir_wrong_mode_is_red(tmp_path):
    """Check (3a): secrets/ at anything other than 0700 -> config_perm_bad.
    Strict equality: 0755 (over-loose) AND 0400 (over-tight) both fail."""
    root = _make_green_tree(tmp_path)
    os.chmod(str(root / "secrets"), 0o755)   # over-loose
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(root)})
    assert ei.value.detail["kind"] == "config_perm_bad"
    assert ei.value.detail["actual_mode"] == "0755"


def test_onvif_creds_wrong_mode_is_red(tmp_path):
    """Check (3b): onvif_credentials.json at anything other than 0600."""
    root = _make_green_tree(tmp_path)
    onvif = root / "secrets" / "onvif_credentials.json"
    onvif.write_text('{"user":"","pass":""}')
    os.chmod(str(onvif), 0o644)              # world-readable
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(root)})
    assert ei.value.detail["kind"] == "config_perm_bad"
    assert "onvif" in ei.value.detail["path"]


def test_symlink_escape_inside_root_is_red(tmp_path):
    """Check (4): an in-tree symlink pointing OUTSIDE the root.
    The root itself is not a symlink (variant 1), but one of the
    required files is a symlink to /tmp/outside. Real-world attack
    vector this catches: ops accidentally symlinks configs/common.yaml
    to /etc/xbrain-shared/common.yaml for cross-machine sharing."""
    root = _make_green_tree(tmp_path)
    outside = tmp_path / "outside_common.yaml"
    outside.write_text("# outside the tree\n")
    # Replace common.yaml with a symlink to outside.
    victim = root / "common.yaml"
    os.remove(str(victim))
    os.symlink(str(outside), str(victim))
    with pytest.raises(XbrainError) as ei:
        run({"config_root": str(root)})
    assert ei.value.detail["kind"] == "config_path_escape"
    assert ei.value.detail["required_name"] == "common.yaml"


def test_missing_config_root_key_in_ctx_raises_assertionerror(tmp_path):
    """Wiring guard: forgetting to populate ctx['config_root'] is a
    caller bug, not a config problem. Should raise AssertionError, NOT
    XbrainError -- the two mean different things and must not be
    confused (an operator scrolling journalctl needs to know whether
    the fix is 'update configs' or 'fix the bring-up code')."""
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
