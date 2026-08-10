"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: manifest.py
Brief: CHK-1-15 state/geo/manifest emission + §7.11 sync protocol

Description:
15 §7.10 defines state/geo/manifest as an at-most-summary handle
for the WHOLE geographical catalog (waypoints + routes + docks +
fences). Publish discipline:

  * emit on ANY change + 0.1 Hz heartbeat floor
  * catalog_rev is strictly monotone (only reversed by a library
    restore path, see CHK-1-61)
  * catalog_hash = sha256(sorted [(geo_id, rev, state)] triples)[:12]
    -- sort MUST come first; two catalogs that differ only in
    insertion order MUST produce the same hash
  * items[] carries SUMMARY tuples only (geo_id, rev, state,
    kind). Geometry (points, polygons) stays in per-object
    fetches -- otherwise a 5000-point route makes the manifest
    itself as big as the catalog it summarises

15 §7.11 four-branch incremental sync:
  * cloud-missing / cloud-stale       -> get
  * cloud-newer / cloud-updated       -> upsert with base_rev
  * cloud-has-cloud-deleted           -> S4 branch (accept delete)
  * items[state=='deleted']            -> tombstone; DO NOT push
                                          back as missing

15 §7.11 prune discipline:
  * resync{prune: true}: skip any items with dirty=True
    (offline-recorded local edits must NOT be tombstoned by a
    full remote sync)
  * resync from HMI or voice channel -> E_CHANNEL_DENIED
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import Iterable, Optional

from xbrain.common.errors import E_CHANNEL_DENIED


MANIFEST_HEARTBEAT_PERIOD_MS = 10_000   # 0.1 Hz
CATALOG_HASH_HEX_LEN = 12


VALID_ITEM_STATES = frozenset({"active", "deleted"})
VALID_ITEM_KINDS = frozenset({"waypoint", "route", "dock", "fence"})
ALLOWED_RESYNC_ORIGINS = frozenset({"cloud", "cli"})


@dataclass(frozen=True)
class GeoItem:
    """Summary entry for the manifest -- no geometry inline."""
    geo_id: str
    kind: str
    rev: int
    state: str          # active / deleted
    dirty: bool = False


class ManifestSchemaError(Exception):
    pass


def _validate_items(items: Iterable[GeoItem]) -> None:
    for i, item in enumerate(items):
        if item.kind not in VALID_ITEM_KINDS:
            raise ManifestSchemaError(
                f"item[{i}].kind {item.kind!r} not in {sorted(VALID_ITEM_KINDS)}")
        if item.state not in VALID_ITEM_STATES:
            raise ManifestSchemaError(
                f"item[{i}].state {item.state!r} not in {sorted(VALID_ITEM_STATES)}")


def catalog_hash(items: Iterable[GeoItem]) -> str:
    """Deterministic hash: sort by geo_id first, then hash triples.
    Sort is CRITICAL -- two catalogs whose only difference is
    insertion order MUST produce the same hash (spec's variant
    guard: hashing insertion order -> different hashes -> red)."""
    ordered = sorted(items, key=lambda it: it.geo_id)
    payload = "".join(
        f"{it.geo_id}|{it.rev}|{it.state}\n" for it in ordered)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:CATALOG_HASH_HEX_LEN]


@dataclass(frozen=True)
class ManifestSnapshot:
    """One publishable frame."""
    catalog_rev: int
    catalog_hash: str
    items: tuple            # tuple[GeoItem, ...]

    def outbound_items(self) -> tuple:
        """Strip 'dirty' from items before uploading to cloud
        (§7.11: dirty is a LOCAL-only flag)."""
        return tuple(
            GeoItem(
                geo_id=it.geo_id, kind=it.kind, rev=it.rev,
                state=it.state, dirty=False)
            for it in self.items)


def build_snapshot(items: Iterable[GeoItem],
                    catalog_rev: int) -> ManifestSnapshot:
    items_tuple = tuple(items)
    _validate_items(items_tuple)
    return ManifestSnapshot(
        catalog_rev=catalog_rev,
        catalog_hash=catalog_hash(items_tuple),
        items=items_tuple)


class RevRewindForbidden(Exception):
    """catalog_rev decreased without a documented restore."""


def check_rev_monotone(prev_rev: int, new_rev: int,
                        via_restore: bool = False) -> None:
    if new_rev < prev_rev and not via_restore:
        raise RevRewindForbidden(
            f"catalog_rev cannot rewind ({prev_rev} -> {new_rev}) "
            f"outside a restore path")


# --- §7.11 sync branches --------------------------------------------

SYNC_BRANCH_GET = "get"
SYNC_BRANCH_UPSERT = "upsert"
SYNC_BRANCH_S4_ACCEPT_DELETE = "accept_delete"
SYNC_BRANCH_TOMBSTONE = "tombstone"


def classify_sync(local: Optional[GeoItem],
                    remote: Optional[GeoItem]) -> str:
    """Given local + remote entries for the same geo_id, decide the
    branch. `None` means the side doesn't know the object."""
    if remote is None and local is None:
        raise ManifestSchemaError("neither side knows the object")
    if remote is None:
        # Cloud missing -> push
        return SYNC_BRANCH_UPSERT
    if local is None:
        # Local missing -> pull
        return SYNC_BRANCH_GET
    if remote.state == "deleted" and local.state == "active":
        return SYNC_BRANCH_S4_ACCEPT_DELETE
    if local.state == "deleted":
        return SYNC_BRANCH_TOMBSTONE
    if remote.rev < local.rev:
        return SYNC_BRANCH_UPSERT
    if remote.rev > local.rev:
        return SYNC_BRANCH_GET
    return SYNC_BRANCH_UPSERT   # equal rev: idempotent no-op via upsert


# --- prune ----------------------------------------------------------

def prune_skip_dirty(items: Iterable[GeoItem]) -> tuple:
    """§7.11: prune must NEVER tombstone a dirty item."""
    return tuple(it for it in items if not it.dirty)


def check_resync_origin(origin: str) -> None:
    """§7.11: resync{prune:true} only allowed from cloud (or CLI
    for ops). HMI / voice channels -> E_CHANNEL_DENIED."""
    if origin not in ALLOWED_RESYNC_ORIGINS:
        raise PermissionError(
            "%s (origin=%r; expected one of %s)"
            % (E_CHANNEL_DENIED, origin, sorted(ALLOWED_RESYNC_ORIGINS)))
