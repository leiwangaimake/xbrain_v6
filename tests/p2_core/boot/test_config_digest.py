"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_digest.py
Brief: 10 S5.4.4 -- P2 holds motion when its cached digest and MANIFEST differ

Description:
The design row being guarded, verbatim: "P2 放行前 (S3.3.3 Stage D) 校验
MANIFEST.boot_id 与自身内存中的 common_digest; 不一致 -> 保持 '禁止运动',
发 event/fault/bit, detail.kind = 'config_digest_mismatch'".

"自身内存中的" is what these cases are really about. A guard that re-read the
MANIFEST and compared it to itself would pass any test that only checked "no
fault on a healthy system" -- it agrees with itself forever, which is
CLAUDE.md S3.2 form 1. So the cases here always CHANGE the MANIFEST under a
guard that has already cached, which is the real fault mode: the freeze line
ran a second pass that this process never read.

The veto half is tested through hold_grant rather than by running P2's 1 Hz
loop. That is not a convenience: the loop needs a bus, and a test that could
not run without one would not be run. The split is the reason hold_grant
exists as a function at all -- before it the three field assignments were
inline in the loop and had no coverage of any kind (CLAUDE.md S9.1A).
"""

import json
import os

import pytest

from xbrain.p2_core.boot.config_digest import (
    KIND_MISMATCH, KIND_OK, KIND_UNREADABLE, ConfigDigestGuard, DigestVerdict,
    digest_fault_event)
from xbrain.p2_core.health.factor import WIRE_PROFILE_WHEN_BLOCKED, hold_grant

DIGEST = "9f2c4a1b7e5d0836"
BOOT = "3f2a8c1e-0000-0000-0000-000000000000"


def _manifest(tmp_path, digest=DIGEST, boot_id=BOOT):
    """Write a minimal MANIFEST.json and return its directory."""
    root = tmp_path / "resolved"
    root.mkdir(exist_ok=True)
    (root / "MANIFEST.json").write_text(
        json.dumps({"boot_id": boot_id, "common_digest": digest}),
        encoding="utf-8")
    return str(root)


def _guard(root, digest=DIGEST, boot_id=BOOT, period_s=10.0):
    return ConfigDigestGuard(digest, boot_id, resolved_root=root,
                             period_s=period_s)


# --------------------------------------------------------------------------
# The comparison
# --------------------------------------------------------------------------

def test_a_matching_manifest_does_not_block(tmp_path):
    """The case that has to stay silent, or every other case passes for the
    wrong reason and the guard gets switched off in the field.

    Mutant: return blocked=True unconditionally -> red.
    """
    guard = _guard(_manifest(tmp_path))
    verdict = guard.check(100.0)
    assert verdict.blocked is False
    assert verdict.detail is None
    assert verdict.edge is False, "starting consistent is not a transition"


def test_a_second_freeze_pass_blocks_motion(tmp_path):
    """The fault this exists for: the MANIFEST moves while P2 keeps running on
    what it loaded at Stage A.

    Mutant: re-read the digest into the cache on every check (the "the freeze
    line is authoritative" refactor) -> the guard agrees with itself forever
    -> red.
    """
    root = _manifest(tmp_path)
    guard = _guard(root)
    assert guard.check(100.0).blocked is False
    _manifest(tmp_path, digest="0000000000000000")
    verdict = guard.check(200.0)
    assert verdict.blocked is True
    assert verdict.edge is True
    assert verdict.detail["kind"] == KIND_MISMATCH
    # Both values, because "they differ" cannot be acted on: the operator has
    # to know whether the MANIFEST moved forward or P2 is the stale one.
    assert verdict.detail["cached_digest"] == DIGEST
    assert verdict.detail["manifest_digest"] == "0000000000000000"


def test_a_foreign_boot_id_blocks_even_when_the_digest_matches(tmp_path):
    """10 S5.4.4 says boot_id AND common_digest. A MANIFEST copied from
    another machine can carry a plausible digest; the boot_id is what makes it
    obviously foreign.

    Mutant: drop the boot_id half of the comparison -> red.
    """
    root = _manifest(tmp_path)
    guard = _guard(root)
    guard.check(100.0)
    _manifest(tmp_path, digest=DIGEST, boot_id="some-other-boot")
    verdict = guard.check(200.0)
    assert verdict.blocked is True
    assert verdict.detail["manifest_boot_id"] == "some-other-boot"
    assert verdict.detail["cached_boot_id"] == BOOT


@pytest.mark.parametrize("damage", ["delete", "truncate", "not-an-object"])
def test_an_unreadable_manifest_blocks_on_the_safe_side(tmp_path, damage):
    """Continuing to grant motion on a configuration that can no longer be
    attested is the "放行" direction, and that is the one that must never fire
    by accident. Reported under its own kind so an operator can tell it from a
    real mismatch.

    Mutant: return blocked=False on a read error ("be tolerant") -> red, and a
    machine whose /run was wiped keeps driving.
    """
    root = _manifest(tmp_path)
    guard = _guard(root)
    guard.check(100.0)
    path = os.path.join(root, "MANIFEST.json")
    if damage == "delete":
        os.unlink(path)
    elif damage == "truncate":
        open(path, "w", encoding="utf-8").write("{not json")
    else:
        # A JSON document may legally be a list; raw["common_digest"] on one
        # raises TypeError, not KeyError, and an except clause that named only
        # KeyError would let it escape into P2's publish loop.
        open(path, "w", encoding="utf-8").write("[1, 2, 3]")
    verdict = guard.check(200.0)
    assert verdict.blocked is True
    assert verdict.detail["kind"] == KIND_UNREADABLE


def test_a_manifest_without_the_digest_field_blocks(tmp_path):
    """An older freeze that predates common_digest is not evidence of a match.
    Mutant: .get with a None default -> None != cached is still a mismatch, so
    this one would pass anyway; the case pins the KIND, which would change to
    config_digest_mismatch and send the operator looking for a re-freeze that
    never happened."""
    root = _manifest(tmp_path)
    guard = _guard(root)
    (tmp_path / "resolved" / "MANIFEST.json").write_text(
        json.dumps({"boot_id": BOOT}), encoding="utf-8")
    verdict = guard.check(100.0)
    assert verdict.blocked is True
    assert verdict.detail["kind"] == KIND_UNREADABLE


# --------------------------------------------------------------------------
# Cadence and edges
# --------------------------------------------------------------------------

def test_the_manifest_is_not_re_read_every_tick(tmp_path):
    """10 S5.4.4's own note on P2's startup read is that a file read does not
    belong in the 1 Hz publish path.

    Mutant: drop the period check -> the changed MANIFEST is seen one second
    later instead of at the next period -> red here (and a file read plus a
    parse enters the grant path).
    """
    root = _manifest(tmp_path)
    guard = _guard(root, period_s=10.0)
    assert guard.check(100.0).blocked is False
    _manifest(tmp_path, digest="0000000000000000")
    # Inside the period: the cached verdict stands.
    assert guard.check(105.0).blocked is False
    assert guard.check(109.9).blocked is False
    # Past it: the change is seen.
    assert guard.check(110.1).blocked is True


def test_the_first_check_always_runs(tmp_path):
    """Whatever the caller's monotonic clock reads at startup.

    Mutant: start the period timer lazily on the first call ("set the deadline
    when we are first used, then check from the second tick") -> the first
    period of every boot runs unchecked, which is precisely the window this
    guard exists for (a freeze re-run DURING startup).

    Note what does NOT work as a mutant here: seeding the deadline at 0.0
    instead of -inf. Every realistic monotonic reading is above both, so the
    two seeds are indistinguishable -- an equivalent mutation, recorded in the
    code rather than defended with an assertion no input can exercise
    (CLAUDE.md 7.2.1).
    """
    guard = _guard(_manifest(tmp_path, digest="0000000000000000"))
    assert guard.check(1.0e9).blocked is True


def test_the_fault_edge_fires_once_not_once_per_tick(tmp_path):
    """blocked stays true until an operator restarts the stack; a fault per
    second would bury every other event in record.db.

    Mutant: set edge = blocked -> red.
    """
    root = _manifest(tmp_path)
    guard = _guard(root, period_s=1.0)
    guard.check(100.0)
    _manifest(tmp_path, digest="0000000000000000")
    assert guard.check(102.0).edge is True
    # Within the period, the cached verdict is returned -- and it must report
    # edge=False. Replaying the stored edge would emit one fault per tick for
    # the whole period, which is the same flood the edge exists to prevent.
    assert guard.check(102.2).edge is False
    assert guard.check(102.5).edge is False
    assert guard.check(102.5).blocked is True
    # Past the period, the comparison runs again and still finds a mismatch,
    # so still no edge -- the state did not change.
    assert guard.check(104.0).edge is False
    assert guard.check(106.0).blocked is True


def test_the_recovery_edge_fires_too(tmp_path):
    """An operator watching events needs to see the condition clear, or the
    only way to learn it is to notice the robot moving again.

    Mutant: edge = blocked and not last.blocked (fault edge only) -> red.
    """
    root = _manifest(tmp_path)
    guard = _guard(root, period_s=1.0)
    guard.check(100.0)
    _manifest(tmp_path, digest="0000000000000000")
    assert guard.check(102.0).edge is True
    _manifest(tmp_path, digest=DIGEST)
    recovered = guard.check(104.0)
    assert recovered.blocked is False
    assert recovered.edge is True


# --------------------------------------------------------------------------
# Stage A capture
# --------------------------------------------------------------------------

def test_stage_a_caches_from_the_loaded_config_not_a_fresh_read(tmp_path):
    """The guard's cache must come from the ResolvedConfig the process is
    running on. Two reads can see two different files -- which is the fault
    being guarded -- and the guard would then attest a snapshot P2 never read.

    The stand-in carries values that are NOT on disk, so a guard that re-read
    the MANIFEST would cache the disk values and report no mismatch.
    Mutant: have at_stage_a open MANIFEST.json itself -> red.
    """
    class _FakeManifest:
        common_digest = "from-the-loaded-object"
        boot_id = "loaded-boot"

    class _FakeResolved:
        manifest = _FakeManifest()

    root = _manifest(tmp_path)
    guard = ConfigDigestGuard.at_stage_a(_FakeResolved(), resolved_root=root)
    assert guard.cached_digest == "from-the-loaded-object"
    verdict = guard.check(100.0)
    assert verdict.blocked is True
    assert verdict.detail["cached_digest"] == "from-the-loaded-object"
    assert verdict.detail["manifest_digest"] == DIGEST


# --------------------------------------------------------------------------
# The veto applied to the grant body
# --------------------------------------------------------------------------

def test_hold_grant_forces_all_three_wire_fields():
    """P1 parses exactly speed_factor / allow_motion / max_profile.

    Mutant: force allow_motion only -> P1's HealthFactorSlot keeps the old
    speed alongside the false flag, and the next message that flips the flag
    back resumes at that speed instead of ramping from a stop -> red.
    Mutant: leave max_profile at "none" -> P1 rejects the whole body and keeps
    driving on the PREVIOUS grant, turning the veto into a no-op -> red.
    """
    body = {"speed_factor": 1.0, "allow_motion": True, "max_profile": "patrol",
            "reason": "none", "detail_ref": "x"}
    out = hold_grant(body, KIND_MISMATCH)
    assert out is body, "the body is mutated in place, not copied"
    assert out["allow_motion"] is False
    assert out["speed_factor"] == 0.0
    assert out["max_profile"] == WIRE_PROFILE_WHEN_BLOCKED
    assert out["reason"] == KIND_MISMATCH
    # Fields the veto has no opinion about are left alone -- rewriting
    # detail_ref would point the HMI at a summary that does not explain this.
    assert out["detail_ref"] == "x"


def test_the_fault_event_carries_the_contract_kind():
    """10 S5.4.4 verbatim: detail.kind = "config_digest_mismatch". The string
    travels to p5, record.db and the cloud unchanged.

    Mutant: rename the kind -> red. Mutant: emit an empty detail on recovery
    -> red on the second half (a consumer would have to branch on presence
    rather than on value).
    """
    blocked = DigestVerdict(True, {"kind": KIND_MISMATCH, "cached_digest": "a"},
                            True)
    ev = digest_fault_event(blocked, "cfgdigest-abc-1", 1753660000.0)
    assert ev["detail"]["kind"] == "config_digest_mismatch"
    assert ev["eid"] == "cfgdigest-abc-1"
    assert ev["src"] == "p2_core"
    assert ev["ts"] == 1753660000.0
    assert "mismatch" in ev["title"]
    ok = digest_fault_event(DigestVerdict(False, None, True), "e", 1.0)
    assert ok["detail"] == {"kind": KIND_OK}
    assert "consistent" in ok["title"]
