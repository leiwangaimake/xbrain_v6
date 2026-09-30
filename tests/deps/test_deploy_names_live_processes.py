"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_deploy_names_live_processes.py
Brief: deploy/ must not describe a deleted process as a live participant

Description:
The defect, measured 2026-09-30. 12 S4.6 was voided whole on 2026-09-29 (user
ruling, verbatim "the robot's motion is all RNS, no ROS 2 action"), the
behavior_proxy and Nav2 behavior_server processes went away with it, and both
10 S3.1 and 10 S3.2 were struck through the same week. Four files under deploy/
were not, and went on asserting the processes as present:

  * xbrain-zenohd-gen.service and zenoh/zenohd-gen.json5 listed behavior_proxy
    as a CLIENT OF THE GENERAL-PLANE ROUTER -- wrong twice over, because even
    while it was designed it was an RT-plane-only participant holding no
    general-plane session (the struck 10 S3.1 row says so in as many words);
  * xbrain-p2-core.service and xbrain-ai-asr.service named it and Nav2 among
    the core-7 co-tenants.

Why this needs a door rather than just the four edits. scripts/ci/check_affinity.py
already guards the part of this that is MACHINE-READABLE -- it diffs 10 S3.2's
table against the units' CPUAffinity in both directions and reports NO-UNIT /
UNLISTED. It cannot see prose: all four defects sat in comments, so it printed
"failures: 0" throughout. A comment that names a process nobody can run is not
cosmetic here, because these particular comments are what someone sizing the
router's client set or re-planning core 7 reads first (10 S3.2 records exactly
that failure once already: a v0.6 correction removed an HMI process that does
not exist, because leaving it "would send an implementer off to pin a
non-existent process to a core").

WHAT IT CHECKS
  A file under deploy/ that names a deleted process must also carry the removal
  date, which is what a tombstone note has and a fresh live reference does not.

SCAN SURFACE (CLAUDE.md S3.2 form 6 -- an undeclared count is unreadable)
  deploy/**, every file, comments included (prose is the whole point).

BOUNDARIES AND A DELIBERATE WEAKNESS
  The check is per FILE, not per line: no window, no "within N lines of", because
  this repository has already been bitten by a criterion whose own length
  assumption expired. The cost is that it UNDER-REPORTS -- a file that carries
  the date in one note could name the process live somewhere else and still pass.
  That asymmetry is chosen: a door that fires on a correct tombstone is a door
  someone relaxes within the week (CLAUDE.md S3.2 form 2), and what this catches
  is the hard core, a deleted name reintroduced with no tombstone at all.

  It does NOT decide which processes exist -- 10 S3.1 does. The names below are
  the two struck there, listed rather than parsed because a struck-through
  markdown row is not a machine-readable list and guessing at one would be a
  second authority.
"""
from __future__ import annotations

import pathlib

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
DEPLOY = ROOT / "deploy"

#: Processes struck through in 10 S3.1 / S3.2 whose units are deleted. Spelled
#: here, not derived, for the reason in the header. Lowercased for matching.
DELETED_PROCESSES = ("behavior_proxy", "behavior-proxy", "nav2")

#: The token a tombstone note carries. The date the two names were struck in
#: 10 S3.2 (v0.9) and the units deleted -- so a note explaining the removal has
#: it, and a freshly written live reference does not.
REMOVAL_DATE = "2026-09-30"


def _deploy_files():
    """Every file under deploy/. Declared scan surface; no extension filter,
    because the defect appeared in both .service and .json5 comments."""
    return sorted(p for p in DEPLOY.rglob("*") if p.is_file())


def test_no_deploy_file_names_a_deleted_process_without_a_tombstone():
    """A deleted process may be MENTIONED, but only as history.

    MUTATION: add a line naming behavior_proxy to any deploy/ file that has no
    2026-09-30 note -> red, naming the file. Delete the removal-date note from
    xbrain-zenohd-gen.service while leaving the name -> red.
    """
    offences = []
    for path in _deploy_files():
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:              # pragma: no cover -- no binaries today
            continue
        lowered = text.lower()
        hits = sorted({name for name in DELETED_PROCESSES if name in lowered})
        if hits and REMOVAL_DATE not in text:
            offences.append("%s names %s with no %s tombstone"
                            % (path.relative_to(ROOT), hits, REMOVAL_DATE))
    assert offences == [], (
        "deploy/ describes a process that 10 S3.1 struck through and whose unit "
        "is deleted:\n  " + "\n  ".join(offences))


def test_the_four_known_sites_carry_their_tombstone():
    """Positive half: the door is vacuous if no file names these at all.

    Without this, deleting the four notes AND the four names together would
    leave the test above green while the history of why they went disappears --
    and the next person re-adds behavior_proxy to the router's client list
    because nothing records that it never held a general-plane session. That is
    the "forever-green assertion" of CLAUDE.md S3.2 form 1 applied to a
    deletion: the check must confirm the tombstones exist, not merely that no
    un-tombstoned name does.

    MUTATION: remove the behavior_proxy note from zenohd-gen.json5 -> red here.
    """
    expected = (
        "systemd/xbrain-zenohd-gen.service",
        "zenoh/zenohd-gen.json5",
        "systemd/xbrain-p2-core.service",
        "systemd/xbrain-ai-asr.service",
    )
    for rel in expected:
        path = DEPLOY / rel
        assert path.is_file(), "%s moved; retarget this door" % rel
        text = path.read_text(encoding="utf-8")
        lowered = text.lower()
        assert any(n in lowered for n in DELETED_PROCESSES), (
            "%s no longer records that behavior_proxy / Nav2 were removed from "
            "it; the reason they are not clients of this router is lost" % rel)
        assert REMOVAL_DATE in text, (
            "%s names a deleted process but lost its %s tombstone"
            % (rel, REMOVAL_DATE))


def test_no_systemd_unit_for_a_deleted_process_exists():
    """The units themselves, so the prose check cannot be the only guard.

    check_affinity.py catches a unit whose CPUAffinity is unlisted, but a unit
    with no CPUAffinity line at all would slip past it entirely.

    MUTATION: re-create deploy/systemd/xbrain-behavior-proxy.service -> red.
    """
    units = {p.name.lower() for p in (DEPLOY / "systemd").glob("*.service")}
    for banned in ("xbrain-behavior-proxy.service",
                   "xbrain-nav2-behavior.service"):
        assert banned not in units, (
            "%s is back; 12 S4.6 is void and the process does not exist" % banned)
