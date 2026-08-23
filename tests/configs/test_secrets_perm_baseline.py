"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_secrets_perm_baseline.py
Brief: CFG-CF-1 -- secrets/** permission baseline, positive plus named mutations

Description:
WHAT THIS GUARDS. 10 S5.4.4 assertion J item 3 (grep anchor:
凭据目录判据写成目录通配) fixes the credential-tree permission baseline as a
DIRECTORY GLOB: every file exactly 0600, every directory exactly 0700, every
entry owned by the run user. scripts/lint/secrets_perm_baseline.sh is the
executable form; this suite proves that executable actually reacts. The failure
it guards against is the one the design names: before the glob existed a chassis
TLS private key could ship at 0644 and no gate went red -- 13 TLS-3 required
0600 but assertion J did not name the file, SEC-11 did not name it, and 13's own
QC-15 sat among assertions with no executor.

WHY EVERY CHECK GETS A MUTATION. A checker that reports zero on a clean tree is
indistinguishable from one that reports zero on everything (CLAUDE.md 3.3). So
each of the script's three checks is reddened here by a tree built to violate
exactly that check, and two further cases prove the script is not stuck red
(CLAUDE.md 3.2 form 2): the clean tree must PASS, and so must an empty tree at
0700, because an empty credential directory has zero files to be insecure.

THE CHECK-TO-TEST MAP, so no case is redundant and none is missing:
  file check   0600  -> test_client_key_0644_fails... (the row's named mutation)
  dir check    0700  -> test_directory_0755_fails...
  owner check        -> test_wrong_owner_fails
  not-stuck-red      -> test_clean_tree_passes, test_empty_tree_at_0700_passes
  absent tree        -> test_absent_root_passes_with_note

WHY A REAL client.key IS NOT COMMITTED. .gitignore ignores secrets/ and
**/credentials*, and 13 TLS-3 forbids credential material in git. So this suite
never touches configs/secrets/. It builds a throwaway tree under pytest's
tmp_path, owned by the test user, and points the script at it. The tree mirrors
the real layout (a chassis_tls/ subdirectory holding ca.crt / client.crt /
client.key) only so the mutation the CFG-CF-1 row names -- chmod client.key to
0644 -- is reproduced exactly, absolute path and all.

WHAT THIS DOES NOT ESTABLISH. It does not run the freeze service and does not
evaluate the other four items of assertion J (root-not-a-symlink, the required
file list, path escape, the obsolete-file blacklist); those are CFG-FZ-2. Nor
does it prove the deployed configs/secrets/ is correct -- that tree is
provisioned at deploy time and is out of a unit test's reach by design. What is
proven is narrower and exact: given a tree, the item-3 glob names every
violation with an ABSOLUTE path and fails closed.
"""

import os               # filesystem layout and mode bits
import pwd              # resolve a real second username for the owner mutation
import subprocess       # run the baseline script as a black box, the way deploy will

import pytest           # skip honestly when the host has no second account

# INF-TS-1 三档 marker. 本文件是纯静态/元检查(读文件与仓库状态),
# 不碰任何硬件, 故 no_device -- 2026-08-23 从 legacy 未标记名单迁出.
pytestmark = pytest.mark.no_device

# The repository root is three levels up from tests/configs/<this file>. Derived,
# not a hardcoded literal, so the suite is not pinned to /opt and so this file
# carries no source-root literal for a future scan to flag.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# The artifact under test. tests/ is deliberately outside the scan surface of
# no_config_source_read.py, so naming a repo path here is allowed; this is a
# script path in any case, not the configuration source.
SCRIPT = os.path.join(ROOT, "scripts", "lint", "secrets_perm_baseline.sh")


def _run(secrets_root, expected_owner=None):
    """Run the baseline script on one tree; return (returncode, combined_output).

    Invoked through `bash` explicitly rather than via the execute bit: a checkout
    that lost the mode bit must fail loudly here, not silently pass. stderr is
    folded into stdout because a find error (should one ever arise) belongs in the
    same transcript the assertions read, not in a stream nobody inspects.
    """
    # Positional argv mirrors how deploy will call it: root first, optional owner
    # second. Omitting the owner exercises the script's default (id -un); passing
    # one exercises the owner-mismatch branch without needing a privileged chown.
    argv = ["bash", SCRIPT, secrets_root]           # root first, like deploy invokes it
    if expected_owner is not None:                  # owner arg is optional
        argv.append(expected_owner)                 # present -> exercise the mismatch branch
    # capture_output+text so the assertions can match on the human-readable report.
    proc = subprocess.run(argv, capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr   # stderr folded in on purpose


def _make_tree(tmp_path):
    """Build a correct secrets tree; return (root, key_path, tls_dir).

    Layout mirrors 13 TLS-3: secrets/ (0700) -> chassis_tls/ (0700) holding
    ca.crt / client.crt / client.key, each 0600. Returning the key path and the
    subdirectory lets each mutation test reach straight for the thing it corrupts.
    """
    # The root must be EXACTLY 0700: the directory check visits the root itself, so
    # a root left looser than 0700 would be a violation of its own and would mask
    # whatever mutation the test is actually trying to isolate.
    root = tmp_path / "secrets"
    root.mkdir(mode=0o700)

    # A subdirectory, so the directory check is exercised on more than the root and
    # so the named file (chassis_tls/client.key) sits where TLS-3 places it.
    tls_dir = root / "chassis_tls"
    tls_dir.mkdir(mode=0o700)

    # Three files, not one: a check that iterates incorrectly could still pass on a
    # single file, and TLS-3 lists exactly this set of credential artifacts.
    for name in ("ca.crt", "client.crt", "client.key"):
        f = tls_dir / name
        f.write_text("placeholder -- not a real credential\n")
        # write_text honours the umask, so pin the mode to the 0600 baseline
        # explicitly rather than trusting whatever the process umask happens to be.
        os.chmod(f, 0o600)

    # os.mkdir also applies the umask, so re-assert both directories at exactly
    # 0700; otherwise the "clean tree passes" case might not actually be at the
    # baseline it claims to test.
    os.chmod(tls_dir, 0o700)
    os.chmod(root, 0o700)

    return str(root), str(tls_dir / "client.key"), str(tls_dir)


def _a_different_real_user(me):
    """A real, existing username that is not `me`, or None if the host has none.

    The owner check is `find ! -user NAME`, and find ERRORS on a name it cannot
    resolve -- which would print nothing and make the branch look clean. So the
    owner mutation must name a user that genuinely exists and differs from the
    file owner. Well-known accounts are tried first so the test is stable across
    hosts; the password database is the fallback so it still works on a stripped
    image, whether it runs as an ordinary user or as root.
    """
    for name in ("nobody", "root", "daemon", "bin"):
        try:
            if pwd.getpwnam(name).pw_name != me:
                return name
        except KeyError:
            # This account is simply absent on this host; try the next candidate.
            continue
    # Fallback: any account at all that differs from the current user.
    for entry in pwd.getpwall():
        if entry.pw_name != me:
            return entry.pw_name
    return None


def test_clean_tree_passes(tmp_path):
    """A tree that is all 0600/0700 and correctly owned must PASS.

    This is the anti-stuck-red case. Without it, a script that failed everything
    would still satisfy every mutation below, and CLAUDE.md 3.2 form 2 warns that
    a permanently-red criterion gets loosened until it passes.
    """
    root, _key, _tls = _make_tree(tmp_path)
    rc, out = _run(root)
    # A correctly-provisioned credential tree must not be rejected, or nobody could
    # ever ship one and the check would be turned off.
    assert rc == 0, out
    # The PASS line is the positive signal the freeze service keys on.
    assert "PASS" in out
    # And there must be no violation line at all: a PASS that still printed BAD
    # would be a contradictory report a reader could not act on.
    assert "BAD" not in out


def test_empty_tree_at_0700_passes(tmp_path):
    """An empty secrets/ at 0700 is a correct baseline, not a failure.

    This mirrors the current on-disk state: configs/secrets is 0700 and empty. The
    file glob is vacuously zero-hit and the only directory (the root) is 0700, so
    the script must PASS. Reporting a violation here would invent a file-existence
    requirement the design does not state (see the script header): whether a
    credential is present is a deploy concern, not this baseline's.
    """
    # Build just the root at 0700, with no files inside it.
    root = tmp_path / "secrets"
    root.mkdir(mode=0o700)
    os.chmod(root, 0o700)
    rc, out = _run(str(root))
    # Empty-but-correct must not be mistaken for a failure.
    assert rc == 0, out
    assert "PASS" in out


def test_absent_root_passes_with_note(tmp_path):
    """A root that does not exist yet is nothing-to-verify, not a failure.

    The baseline must be safe to wire into a pipeline before any credential is
    provisioned, so an absent tree reports a NOTE and exits 0 rather than erroring.
    A hard failure here would push someone to run the check only after deploy,
    which is the one moment a bad mode most needs catching.
    """
    # A path that was never created.
    missing = tmp_path / "does_not_exist"
    rc, out = _run(str(missing))
    # Absent is pass-with-note, not error.
    assert rc == 0, out
    # The NOTE line records WHY it passed, so a green run is not read as "verified".
    assert "NOTE" in out


def test_client_key_0644_fails_and_prints_absolute_path(tmp_path):
    """THE NAMED MUTATION (CFG-CF-1 row): client.key 0600 -> 0644.

    A false PASS here is precisely the shipped-world-readable-key failure the
    whole item exists to prevent, so this is the load-bearing test. The script
    must go red AND print the offending file's ABSOLUTE path: a bare "config
    invalid" with no path is useless to the operator who has to fix it, which is
    why assertion J writes 必须打印绝对路径.
    """
    root, key_path, _tls = _make_tree(tmp_path)
    # Corrupt exactly the file the criterion names, to exactly the mode it names.
    os.chmod(key_path, 0o644)
    rc, out = _run(root)
    # The check must fail closed on a world-readable private key.
    assert rc != 0, out
    # key_path is already absolute (built under tmp_path), so it is the same string
    # the script emits after resolving the root with `cd && pwd -P`. If this ever
    # printed a relative path an operator could not locate the file.
    assert key_path in out, out
    # Reported under the file-permission branch specifically, not some other check,
    # so the operator is sent to the right fix.
    assert "file not 0600" in out


def test_directory_0755_fails_and_prints_absolute_path(tmp_path):
    """MUTATION for the directory check: chassis_tls/ 0700 -> 0755.

    Proves the `-type d ! -perm 700` glob is actually evaluated and not assumed
    clean, and that a directory violation, like a file one, is named by absolute
    path. A world-executable/searchable credential directory is how a key becomes
    reachable even when its own bits look fine.
    """
    root, _key, tls_dir = _make_tree(tmp_path)
    # Loosen the directory the credentials live in.
    os.chmod(tls_dir, 0o755)
    rc, out = _run(root)
    # A traversable credential directory must fail the baseline.
    assert rc != 0, out
    # Named by absolute path, so it is actionable.
    assert tls_dir in out, out
    # Under the directory branch, not misattributed to a file.
    assert "dir not 0700" in out


def test_wrong_owner_fails(tmp_path):
    """MUTATION for the owner check: an expected owner that is not the file owner.

    Re-owning a file needs privilege, so rather than chown the tree the script is
    TOLD to expect a different but real user; every entry, owned by the test user,
    then fails the owner conjunct. See _a_different_real_user for why the name must
    be one that actually exists. If the host truly has no second account the branch
    cannot be exercised, and the honest thing is to skip rather than assert nothing.
    """
    root, _key, _tls = _make_tree(tmp_path)
    # The user the files actually belong to.
    me = pwd.getpwuid(os.getuid()).pw_name
    other = _a_different_real_user(me)
    if other is None:
        pytest.skip("no second system user to exercise the owner branch")
    rc, out = _run(root, expected_owner=other)
    # A tree owned by the wrong account must fail, so a mis-provisioned owner (a
    # deploy that forgot to chown to the run user) is caught rather than trusted.
    assert rc != 0, out
    # The owner branch, specifically, must be the one that produced the failure.
    assert "owner not %s" % other in out, out


def test_script_has_no_hardcoded_config_root():
    """CLAUDE.md 6: the shell script must derive its paths, not hardcode /opt.

    The default secrets root is built from SCRIPT_DIR up to the repo root, so the
    literal configuration root must not appear in the source. Cheap insurance
    against a later edit reintroducing the hardcoded absolute path the style rule
    forbids -- the kind of regression a human diff-reader misses.
    """
    with open(SCRIPT, encoding="utf-8") as fh:
        source = fh.read()
    # Assembled from parts so this assertion does not itself embed the literal it
    # forbids, following the charset_lint / no_config_source_read precedent.
    forbidden = "/opt/" + "xbrain_v6/" + "configs"
    assert forbidden not in source
