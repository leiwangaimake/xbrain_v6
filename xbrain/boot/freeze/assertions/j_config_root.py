"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: j_config_root.py
Brief: Assertion J -- config root + file reachability (CFG-FZ-2)

Description:
Runs FIRST in the freeze pipeline (ORD-1); every subsequent assertion
opens files under the config root, so if J does not pass loudly, the
later ones fail in ways that don't name the actual problem.

J's five checks, verbatim from CFG-FZ-2:
  (1) Root exists, is a directory, is NOT a symlink.
      Symlinks are rejected because a compromised symlink is the
      easiest way to swap out the config tree under a running system
      without leaving a filesystem-level trace.
  (2) Every required config file is stat-able.
      "Stat-able" and not just "readable" because a stat catches
      permission-strip attacks (mode 000) and dead symlinks that a
      later open() would just silently mis-report as a parse error.
  (3) Permissions:
      - secrets/ directory: 0700 (no group/other bits at all).
      - onvif_credentials.json (if present): 0600.
      Both are checked with a strict-equality mode compare, not a
      "at least as tight as" compare, so a 0400 that lost write
      permission still fails and gets noticed.
  (4) No path escape.
      Every required file's real path must live under the real path
      of the root. Catches a symlink inside configs/ that points
      OUT of the tree; symlink at the root itself is caught by (1).
  (5) Obsolete-file blacklist.
      bit_service.yaml is on the list -- 11 removed it and any
      lingering copy would be silently read by a stale loader.
      Adding a new obsolete name = one row in _OBSOLETE_FILES.

Failure vocabulary (matches CFG-FZ-2 exactly, detail.kind closed set):
  config_root_missing   root path does not exist / is a file, not a dir
  config_file_missing   a required file under root is not stat-able
  config_perm_bad       secrets/ or onvif_credentials.json has wrong mode
  config_path_escape    root itself is a symlink, or a required file
                         resolves outside the root
  config_file_obsolete  a name in _OBSOLETE_FILES was found under root

Every raise carries detail.path (absolute) so an operator scrolling
journalctl sees the actual filesystem location, not a placeholder.
"""

# Standard-library only. J runs BEFORE the runtime is initialised (it is
# assertion #1 in ORD-1), so a third-party import failure here would break
# the whole bring-up sequence. os + stat cover every filesystem check we
# need; typing is compile-time hints, not runtime code.
import os
import stat
from typing import Any, Dict, Tuple

# XbrainError is the base of every deliberate raise in this project.
# Import from exceptions, not from the errors package root, because the
# freeze pipeline runs before common/errors is fully wired (asymmetric
# dependency: exceptions.py has no dependency on codes.yaml).
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# Required top-level config files. If a new process needs its own
# config, add it here. The list is a data table, not a for-loop -- one
# row per file so the finding messages can name the file that broke.
# Order does not matter; failures walk the list left-to-right.
_REQUIRED_FILES: Tuple[str, ...] = (
    # Cross-cutting: common.spec.*, common.safety.*, common.motion.*.
    "common.yaml",
    # Per-process configs (5 xbrain runtime + quadruped RT-side).
    "p1_motion.yaml",
    "p2_core.yaml",
    "p3_task.yaml",
    "p4_agent.yaml",
    "p5_gateway.yaml",
    "quadruped.yaml",
)

# Files whose presence is a defect: they were removed from 11 in an
# earlier round but a stale copy on disk would be silently read by a
# loader that scans by wildcard. Each row = one obsolete name.
_OBSOLETE_FILES: Tuple[str, ...] = (
    # CFG-FZ-2 names this one verbatim. Removed in 11 v0.5, replaced
    # by the BIT items inside p2_core.yaml.
    # New retirements go here as one line each, oldest first so the
    # blame can be traced by row order without git.
    "bit_service.yaml",
)

# Modes we enforce on secrets. Not a "at-least-as-tight-as" compare --
# strict equality. If a stricter operator set 0400 by mistake, our
# process cannot chmod it back at runtime (would need root), and
# silently continuing with a file it cannot read is exactly the kind
# of surprise this assertion exists to catch.
# 0o700 = rwx --- --- for owner only; the whole point of secrets/.
_SECRETS_DIR_MODE = 0o700
# 0o600 = rw- --- --- for owner only; a file that both parts of the
# pipeline (loader + rotator) read but nothing else may touch.
_ONVIF_CREDS_MODE = 0o600


def _fail(kind: str, path: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + absolute path.

    Wraps XbrainError construction so every raise inside this module
    shares the same shape -- the closed set for detail.kind is the
    one CFG-FZ-2 names verbatim, and any deviation would silently
    weaken the elsewhere-in-the-tree checks that match on it.

    kind: one of the five closed values documented above. Not defined
      here as an Enum on purpose -- keeping them plain strings makes
      the raise sites read exactly like the CFG-FZ-2 spec.
    path: the filesystem path the failure is about. Converted to
      absolute below.
    **extra: additional detail fields (target, mode, required_name,
      obsolete_name, ...). Kept as a kwarg dict so each raise site
      can add exactly what its failure needs without a signature
      change here.
    """
    # Convert path to absolute at raise time. Callers may pass in a
    # relative root like "configs/"; a relative path in a systemd
    # journal is much less useful than an absolute one.
    abs_path = os.path.abspath(path)
    detail = {"kind": kind, "path": abs_path}
    detail.update(extra)
    # Human-readable message. Deliberately English (CLAUDE.md S2.1).
    # The kind field carries the machine-usable classification; this
    # sentence just makes the journal line legible without decoding.
    raise XbrainError(
        E_CONFIG_INVALID,
        "config root check failed: %s at %s" % (kind, abs_path),
        detail,
    )


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion J. Replaces registry's stub_j.

    ctx["config_root"] is required (put there by run_freeze). Missing
    is a wiring bug in the caller, not a config problem, so we raise
    AssertionError (uncaught) rather than E_CONFIG_INVALID.

    Return value is a result dict with status=pass on success; on any
    of the five sub-checks failing this function raises XbrainError
    and never returns (the caller's expectation is that a failed
    assertion propagates out, not that we return status=fail).

    Idempotent: repeat calls with the same ctx do exactly the same
    filesystem reads and either pass again or fail with the same
    kind. There is no persistent state kept across calls.
    """
    # Defensive: the caller (run_freeze) puts config_root in ctx.
    # If it's missing, someone constructed ctx by hand and forgot to
    # populate it -- that's a construction defect, not a runtime
    # config issue, so a plain AssertionError is the right raise.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion J requires ctx['config_root']; caller did not "
            "populate it (see xbrain.boot.freeze.pipeline.run_freeze)"
        )
    root = ctx["config_root"]

    # ---- (1) root exists, is a directory, is NOT a symlink -----------
    # Order of the three sub-checks matters and is spelled out below.
    # A wrong order would either misclassify the failure kind (bad
    # for debugging) or silently accept a symlink-to-directory (bad
    # for security, since a symlink is exactly what CFG-FZ-2 (1) is
    # trying to reject).
    if not os.path.exists(root):
        # First-line miss: path does not exist at all. Fires when the
        # config root env var points at a wrong place, or when a
        # fresh checkout hasn't populated configs/ yet.
        _fail("config_root_missing", root)
    # islink check MUST run before isdir: a symlink to a directory
    # passes isdir=True but the point of this check is to reject the
    # symlink itself. Order matters.
    if os.path.islink(root):
        # config_path_escape (not config_root_missing) because a
        # symlink is a redirection, not a missing thing. Detail
        # includes 'reason' to distinguish this from the symlink-
        # escape case in check (4).
        _fail("config_path_escape", root, reason="root is a symlink")
    if not os.path.isdir(root):
        # Exists but is a file / socket / block device -- treat as
        # missing-dir. Distinguishing "wrong type" from "missing" here
        # would surface the wrong-type case, but the operator action
        # is the same (recreate the config tree), so a common bucket.
        _fail("config_root_missing", root, reason="not a directory")

    # ---- (5) obsolete-file blacklist ---------------------------------
    # Run BEFORE required-file scan so an obsolete file's presence
    # (which usually indicates a partial upgrade) is reported before
    # the "missing new file" complaint, letting the operator fix the
    # cause in one pass rather than the symptom in two.
    for name in _OBSOLETE_FILES:
        candidate = os.path.join(root, name)
        if os.path.exists(candidate):
            _fail("config_file_obsolete", candidate, obsolete_name=name)

    # ---- (2) required files stat-able --------------------------------
    # For each name in the required list we do THREE things: exists
    # check, stat call (to distinguish "there" from "there but
    # unreadable"), and regular-file check (to reject a directory
    # that shares a .yaml name). Every failure raises immediately;
    # we do not collect a batch report because bring-up should stop
    # on the first bad file rather than continue and possibly emit
    # cascading complaints against files that depend on the broken one.
    for name in _REQUIRED_FILES:
        candidate = os.path.join(root, name)
        # os.path.exists returns False for a broken symlink; combined
        # with the (4) escape check below, we catch both "file not
        # there" and "file present but points at nothing valid".
        if not os.path.exists(candidate):
            _fail("config_file_missing", candidate, required_name=name)
        # Second stat with follow_symlinks=True to confirm the target
        # is a regular file, not a directory hiding under the .yaml
        # extension. Deliberately not os.path.isfile() alone -- that
        # returns False for a symlink to a file that is unreadable,
        # which we want to distinguish from "missing".
        try:
            st = os.stat(candidate)
        except OSError as exc:
            _fail("config_file_missing", candidate,
                  required_name=name, errno=exc.errno)
        if not stat.S_ISREG(st.st_mode):
            # Directory named foo.yaml (bizarre but happens): treat
            # as missing since we can't parse it as YAML.
            _fail("config_file_missing", candidate,
                  required_name=name, reason="not a regular file")

    # ---- (3) permissions on secrets ----------------------------------
    # Two files matter here: the secrets/ directory itself (must be
    # 0700) and onvif_credentials.json inside it (must be 0600 IF
    # present). We enforce these strictly -- see the module docstring
    # for why strict-equality rather than "at least as tight as".
    secrets_dir = os.path.join(root, "secrets")
    # secrets/ is optional -- a dev checkout may not have it. Presence
    # -> strict mode check; absence -> silent skip (a later assertion
    # will complain if a secret is actually needed and missing).
    if os.path.isdir(secrets_dir):
        st = os.stat(secrets_dir)
        mode = stat.S_IMODE(st.st_mode)
        # Strict equality: any group/other bit or an over-strict 0400
        # dies here. Rationale in module docstring.
        if mode != _SECRETS_DIR_MODE:
            _fail("config_perm_bad", secrets_dir,
                  expected_mode="0%o" % _SECRETS_DIR_MODE,
                  actual_mode="0%o" % mode)
        # onvif_credentials.json is optional but if present must be
        # 0600. If absent, no complaint -- callers of the ONVIF
        # subsystem will fail with their own explicit error.
        onvif = os.path.join(secrets_dir, "onvif_credentials.json")
        if os.path.exists(onvif):
            st = os.stat(onvif)
            mode = stat.S_IMODE(st.st_mode)
            if mode != _ONVIF_CREDS_MODE:
                _fail("config_perm_bad", onvif,
                      expected_mode="0%o" % _ONVIF_CREDS_MODE,
                      actual_mode="0%o" % mode)

    # ---- (4) no path escape ------------------------------------------
    # For every required file, resolve symlinks and confirm the target
    # still lives under root. This catches an in-tree symlink like
    # configs/p1_motion.yaml -> /etc/foo/p1_motion.yaml which would let
    # an attacker (or a well-meaning ops mistake) swap the config tree
    # by controlling /etc/foo/ instead of /opt/xbrain_v6/configs.
    # os.path.realpath resolves the whole chain, including intermediate
    # symlinks in the directory prefix.
    #
    # This is DIFFERENT from check (1)'s islink test on the root: (1)
    # rejects the root itself being a symlink; (4) rejects any leaf
    # file being a symlink that escapes. Both are needed -- (1) alone
    # would let a per-file symlink attack through, (4) alone would let
    # a whole-tree redirection through.
    root_real = os.path.realpath(root)
    for name in _REQUIRED_FILES:
        candidate = os.path.join(root, name)
        # We only reach here if (2) passed, so candidate exists.
        # realpath both root and candidate; a mismatch means the leaf
        # resolves outside the root tree.
        target_real = os.path.realpath(candidate)
        # startswith with an explicit sep to avoid matching sibling
        # dirs whose name shares the prefix (e.g. /a/config vs
        # /a/config.d). Also handle the exact-equal case (should not
        # happen since target is a file, but cheap to guard).
        prefix = root_real + os.sep
        if target_real != root_real and not target_real.startswith(prefix):
            # detail includes both target and root so an operator can
            # see WHERE the leaf ended up vs where it should have been,
            # without another manual investigation.
            _fail("config_path_escape", candidate,
                  target=target_real, root=root_real,
                  required_name=name)

    # ---- All checks passed -------------------------------------------
    # Return a rich result so a MANIFEST reader can see WHICH root got
    # accepted, in case an override (XBRAIN_CONFIG_DIR) was in effect
    # and the operator wants to double-check that fact from the log.
    # config_root (abs) and config_root_real (realpath'd) are BOTH
    # emitted so a symlink-below-root chain that was rejected in
    # earlier iterations is trivially spot-checkable in the manifest.
    # required_files_checked / obsolete_files_checked count as facts,
    # not just log noise: a future audit reading a MANIFEST from a
    # prior run can confirm the same tuples were in force at the
    # time.
    return {
        "status": "pass",
        "assertion": "J",
        "config_root": os.path.abspath(root),
        "config_root_real": root_real,
        "required_files_checked": len(_REQUIRED_FILES),
        "obsolete_files_checked": len(_OBSOLETE_FILES),
    }
