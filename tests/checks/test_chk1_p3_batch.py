"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chk1_p3_batch.py
Brief: CHK-1-15/58/61 P3 severe items batch

Description:
Three severe items covering the P3 slice: GeoManifest publish
+ sync; cmd/config scope dispatcher; geo.db snapshot backup +
restore.

CHK-1-15 GeoManifest (manifest.py)
  * catalog_hash is order-independent (sort then hash)
  * catalog_rev monotone; rewind refused outside restore path
  * items[] carries only summary; outbound_items strips dirty
  * §7.11 sync branches: get / upsert / accept_delete / tombstone
  * prune skips dirty items (offline edits not overwritten)
  * resync origin closed set; HMI / voice -> E_CHANNEL_DENIED

CHK-1-58 hot-reload dispatcher (dispatch.py)
  * scope closed set; safety params -> E_CONFIG_LOCKED
  * origin != cloud -> E_CHANNEL_DENIED
  * missing token / string token / wrong-issuer token / expired
    token -> E_CONFIRM_REQUIRED with distinct reasons
  * ack fanout: ONE per responding process (5 total)
  * token.expires_mono_ms is compared against monotonic (not wall)

CHK-1-61 snapshot backup (snapshot.py)
  * crossed_multiples(98, 203, 100) -> [100, 200] (exactly two)
  * snapshot_dir must differ from geo_db_dir
  * write + list_snapshots round-trip
  * RestoreCoordinator enforces restore -> resync ordering
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from xbrain.common.errors import (
    E_CHANNEL_DENIED, E_CONFIG_LOCKED, E_CONFIRM_REQUIRED,
)
from xbrain.common.config.hotreload.dispatch import (
    ALLOWED_ORIGINS, RESPONDING_PROCESSES, TOKEN_ISSUER,
    WHITELIST_SCOPES,
    build_ack, check_request, fanout_ack,
)
from xbrain.p3_task.geo.manifest import (
    ALLOWED_RESYNC_ORIGINS, CATALOG_HASH_HEX_LEN, GeoItem,
    MANIFEST_HEARTBEAT_PERIOD_MS, ManifestSchemaError,
    RevRewindForbidden, SYNC_BRANCH_GET, SYNC_BRANCH_S4_ACCEPT_DELETE,
    SYNC_BRANCH_TOMBSTONE, SYNC_BRANCH_UPSERT,
    build_snapshot, catalog_hash, check_resync_origin,
    check_rev_monotone, classify_sync, prune_skip_dirty,
)
from xbrain.p3_task.geo.snapshot import (
    RestoreCoordinator, RestoreWithoutResyncError,
    SNAPSHOT_STEP, SnapshotConfigError, SnapshotSink,
    crossed_multiples, latest_snapshot_rev, list_snapshots,
    restore_from_snapshot, snapshot_path, write_snapshot,
)


pytestmark = pytest.mark.no_device


# ---------------- CHK-1-15 manifest ----------------

def _mk_items(*specs):
    """Compact factory: pass ('id', 'kind', rev, 'state'[, dirty])."""
    out = []
    for s in specs:
        dirty = s[4] if len(s) == 5 else False
        out.append(GeoItem(geo_id=s[0], kind=s[1], rev=s[2],
                             state=s[3], dirty=dirty))
    return out


def test_catalog_hash_order_independent():
    """Same items in different insertion order -> same hash."""
    a = _mk_items(("w-a", "waypoint", 1, "active"),
                    ("w-b", "waypoint", 2, "active"),
                    ("w-c", "waypoint", 3, "active"))
    b = _mk_items(("w-c", "waypoint", 3, "active"),
                    ("w-a", "waypoint", 1, "active"),
                    ("w-b", "waypoint", 2, "active"))
    assert catalog_hash(a) == catalog_hash(b)


def test_catalog_hash_changes_on_rev():
    a = _mk_items(("w-a", "waypoint", 1, "active"))
    b = _mk_items(("w-a", "waypoint", 2, "active"))
    assert catalog_hash(a) != catalog_hash(b)


def test_catalog_hash_length_stable():
    h = catalog_hash(_mk_items(("w-x", "waypoint", 1, "active")))
    assert len(h) == CATALOG_HASH_HEX_LEN


def test_manifest_heartbeat_period_matches_spec():
    """§7.10: at least 0.1 Hz floor."""
    assert MANIFEST_HEARTBEAT_PERIOD_MS <= 10_000


def test_items_kind_closed_set_validated():
    with pytest.raises(ManifestSchemaError, match="kind"):
        build_snapshot(_mk_items(("x-a", "sofa", 1, "active")),
                          catalog_rev=1)


def test_items_state_closed_set_validated():
    with pytest.raises(ManifestSchemaError, match="state"):
        build_snapshot(_mk_items(("w-a", "waypoint", 1, "pending")),
                          catalog_rev=1)


def test_outbound_items_strips_dirty():
    """dirty is LOCAL only; must be stripped before uploading."""
    snap = build_snapshot(
        _mk_items(("w-a", "waypoint", 1, "active", True)),
        catalog_rev=1)
    for it in snap.outbound_items():
        assert it.dirty is False


def test_rev_monotone_rewind_refused_without_restore():
    with pytest.raises(RevRewindForbidden):
        check_rev_monotone(prev_rev=50, new_rev=30, via_restore=False)


def test_rev_monotone_rewind_ok_via_restore():
    check_rev_monotone(prev_rev=50, new_rev=30, via_restore=True)


def test_sync_get_when_local_missing():
    r = classify_sync(local=None,
                        remote=_mk_items(("w-a", "waypoint", 1, "active"))[0])
    assert r == SYNC_BRANCH_GET


def test_sync_upsert_when_remote_missing():
    r = classify_sync(local=_mk_items(("w-a", "waypoint", 1, "active"))[0],
                        remote=None)
    assert r == SYNC_BRANCH_UPSERT


def test_sync_accept_delete_when_cloud_deleted():
    r = classify_sync(
        local=_mk_items(("w-a", "waypoint", 1, "active"))[0],
        remote=_mk_items(("w-a", "waypoint", 1, "deleted"))[0])
    assert r == SYNC_BRANCH_S4_ACCEPT_DELETE


def test_sync_tombstone_when_local_deleted():
    r = classify_sync(
        local=_mk_items(("w-a", "waypoint", 1, "deleted"))[0],
        remote=_mk_items(("w-a", "waypoint", 1, "active"))[0])
    assert r == SYNC_BRANCH_TOMBSTONE


def test_prune_skips_dirty():
    kept = prune_skip_dirty(_mk_items(
        ("clean-1", "waypoint", 1, "active", False),
        ("dirty-1", "waypoint", 2, "active", True),
        ("clean-2", "waypoint", 3, "active", False)))
    assert {it.geo_id for it in kept} == {"clean-1", "clean-2"}


def test_resync_origin_allowed_set_matches_expectation():
    assert ALLOWED_RESYNC_ORIGINS == frozenset({"cloud", "cli"})


def test_resync_origin_hmi_denied():
    with pytest.raises(PermissionError, match=E_CHANNEL_DENIED):
        check_resync_origin("hmi")


def test_resync_origin_voice_denied():
    with pytest.raises(PermissionError):
        check_resync_origin("voice")


def test_resync_origin_cloud_ok():
    check_resync_origin("cloud")


# ---------------- CHK-1-58 hot-reload ----------------

def _valid_token(now_ms: int, ttl_ms: int = 60_000):
    return {"issuer": TOKEN_ISSUER, "value": "tk",
            "expires_mono_ms": now_ms + ttl_ms}


def _valid_cmd(scope: str, now_ms: int = 1000):
    return {"scope": scope, "origin": "cloud",
            "confirm_token": _valid_token(now_ms)}


def test_hotreload_whitelist_scopes_exactly_three():
    """Spec: log_level, debug_flags, asr_dictionary. Any change to
    this set is a spec change that must go through §7.6 review."""
    assert WHITELIST_SCOPES == ("log_level", "debug_flags", "asr_dictionary")


def test_hotreload_valid_log_level_accepted():
    v = check_request(_valid_cmd("log_level"), now_mono_ms=1000)
    assert v.accepted and v.code == "OK"


def test_hotreload_safety_scope_rejected_config_locked():
    """CHK-1-58 (c) guard: safety params must NEVER be whitelisted."""
    v = check_request(_valid_cmd("common.safety.brake"), now_mono_ms=1000)
    assert not v.accepted and v.code == E_CONFIG_LOCKED


def test_hotreload_unknown_scope_rejected_config_locked():
    v = check_request(_valid_cmd("cpu_affinity"), now_mono_ms=1000)
    assert not v.accepted and v.code == E_CONFIG_LOCKED


def test_hotreload_hmi_origin_denied():
    """CHK-1-58 (d) guard."""
    cmd = _valid_cmd("log_level")
    cmd["origin"] = "hmi"
    v = check_request(cmd, now_mono_ms=1000)
    assert not v.accepted and v.code == E_CHANNEL_DENIED


def test_hotreload_missing_token_refused():
    cmd = _valid_cmd("log_level")
    cmd.pop("confirm_token")
    v = check_request(cmd, now_mono_ms=1000)
    assert v.code == E_CONFIRM_REQUIRED


def test_hotreload_string_token_refused():
    cmd = _valid_cmd("log_level")
    cmd["confirm_token"] = "plain-string"
    v = check_request(cmd, now_mono_ms=1000)
    assert v.code == E_CONFIRM_REQUIRED


def test_hotreload_wrong_issuer_token_refused():
    cmd = _valid_cmd("log_level")
    cmd["confirm_token"]["issuer"] = "hmi"
    v = check_request(cmd, now_mono_ms=1000)
    assert v.code == E_CONFIRM_REQUIRED


def test_hotreload_expired_token_refused():
    """CHK-1-58 (e) guard: monotonic-clock expiry."""
    cmd = _valid_cmd("log_level", now_ms=0)   # expires at 60_000
    v = check_request(cmd, now_mono_ms=90_000)     # 90k > 60k
    assert v.code == E_CONFIRM_REQUIRED
    assert "expired" in v.reason


def test_hotreload_fanout_produces_one_ack_per_process():
    v = check_request(_valid_cmd("log_level"), now_mono_ms=1000)
    acks = fanout_ack("log_level", v)
    assert len(acks) == 5
    assert {a["process"] for a in acks} == set(RESPONDING_PROCESSES)
    for a in acks:
        assert a["rejected"] is False
        assert a["detail"]["scope"] == "log_level"


def test_hotreload_build_ack_unknown_process_raises():
    v = check_request(_valid_cmd("log_level"), now_mono_ms=1000)
    with pytest.raises(ValueError, match="unknown process"):
        build_ack("halfway", "log_level", v)


# ---------------- CHK-1-61 snapshot ----------------

def test_crossed_multiples_98_to_203():
    """The spec's marquee example: rev 98 -> 203 crosses 100 and
    200. Exactly two snapshots."""
    assert crossed_multiples(98, 203, step=100) == [100, 200]


def test_crossed_multiples_same_bucket():
    """Rev 150 -> 199 stays in the [100, 200) bucket -> no export."""
    assert crossed_multiples(150, 199, step=100) == []


def test_crossed_multiples_exact_boundary():
    """Rev 100 -> 100 does NOT re-export at 100 (already crossed)."""
    assert crossed_multiples(100, 100, step=100) == []


def test_crossed_multiples_rev_rewind_no_export():
    assert crossed_multiples(200, 150, step=100) == []


def test_snapshot_sink_same_dir_refused():
    with pytest.raises(SnapshotConfigError, match="differ"):
        SnapshotSink(snapshot_dir="/data/geo", geo_db_dir="/data/geo/")


def test_snapshot_sink_different_dir_ok():
    SnapshotSink(snapshot_dir="/backups/geo", geo_db_dir="/data/geo")


def test_write_and_list_snapshots(tmp_path):
    sink = SnapshotSink(snapshot_dir=str(tmp_path / "backups"),
                         geo_db_dir=str(tmp_path / "data"))
    write_snapshot(sink, rev=100, items_serialisable=[{"geo_id": "w-a"}])
    write_snapshot(sink, rev=200, items_serialisable=[{"geo_id": "w-a"}])
    revs = list_snapshots(sink)
    assert revs == [100, 200]
    assert latest_snapshot_rev(sink) == 200


def test_restore_from_snapshot_returns_parsed(tmp_path):
    sink = SnapshotSink(snapshot_dir=str(tmp_path / "backups"),
                         geo_db_dir=str(tmp_path / "data"))
    write_snapshot(sink, rev=100,
                     items_serialisable=[{"geo_id": "w-a", "rev": 5}])
    parsed = restore_from_snapshot(sink, rev=100)
    assert parsed["catalog_rev"] == 100
    assert parsed["items"][0]["geo_id"] == "w-a"


def test_restore_coordinator_requires_resync():
    """§7.11.2: restore MUST be followed by resync."""
    coord = RestoreCoordinator()
    coord.on_restore_done()
    with pytest.raises(RestoreWithoutResyncError, match="resync never"):
        coord.assert_complete()


def test_restore_coordinator_resync_without_restore_refused():
    coord = RestoreCoordinator()
    with pytest.raises(RestoreWithoutResyncError):
        coord.on_resync_emitted()


def test_restore_coordinator_happy_path():
    coord = RestoreCoordinator()
    coord.on_restore_done()
    coord.on_resync_emitted()
    coord.assert_complete()


def test_snapshot_step_constant_matches_spec():
    """CHK-1-61 spec: every 100 catalog_rev crossings."""
    assert SNAPSHOT_STEP == 100
