"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: snapshot.py
Brief: CHK-1-61 geo_snapshot_{rev}.json export + restore_from_snapshot

Description:
Backup discipline:
  * export triggers on catalog_rev crossing an integer multiple of
    STEP (default 100). So rev 98 -> 100 -> 200 -> 203 exports
    exactly TWO snapshots (rev=100, rev=200), never one at each
    write. Rev crossings, not write counts.
  * export path is a separate directory from geo.db (a single-
    directory disk failure that corrupts the DB must not also
    lose the backup).
  * export runs OUTSIDE the upsert transaction (a slow-disk
    export must not block the upsert), invoked from a post-
    commit hook.

Restore discipline:
  * S-8 corruption check must run FIRST (integrity_check fails
    on the malformed DB); only then may restore_from_snapshot
    replay the latest snapshot.
  * After restore, §7.11.2 resync MUST be triggered so the
    cloud catches any rev-delta versus the snapshot boundary.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional


SNAPSHOT_STEP = 100
SNAPSHOT_FILE_PREFIX = "geo_snapshot_"
SNAPSHOT_FILE_SUFFIX = ".json"


class SnapshotConfigError(Exception):
    pass


@dataclass(frozen=True)
class SnapshotSink:
    """Where snapshots go. MUST be a different directory from
    geo.db (a single-directory outage that corrupts the DB
    must not also lose backups)."""
    snapshot_dir: str
    geo_db_dir: str

    def __post_init__(self) -> None:
        # normalise so trailing slashes don't fool the sameness check
        a = os.path.normpath(self.snapshot_dir)
        b = os.path.normpath(self.geo_db_dir)
        if a == b:
            raise SnapshotConfigError(
                "snapshot_dir (%r) must differ from geo_db_dir (%r); "
                "a single-directory failure that corrupts the DB "
                "would then also lose the backup" % (a, b))


def crossed_multiples(prev_rev: int, new_rev: int,
                        step: int = SNAPSHOT_STEP) -> List[int]:
    """Return integer multiples of `step` strictly greater than
    prev_rev and <= new_rev. So (98, 203, 100) -> [100, 200]."""
    if new_rev < prev_rev:
        return []       # rev went backwards (restore path)
    lo = (prev_rev // step + 1) * step
    hi = (new_rev // step) * step
    if lo > hi:
        return []
    return list(range(lo, hi + 1, step))


def snapshot_path(sink: SnapshotSink, rev: int) -> str:
    return os.path.join(sink.snapshot_dir,
                          f"{SNAPSHOT_FILE_PREFIX}{rev}{SNAPSHOT_FILE_SUFFIX}")


def write_snapshot(sink: SnapshotSink, rev: int,
                    items_serialisable: Iterable[dict]) -> str:
    """Write one snapshot atomically (write .new + rename)."""
    os.makedirs(sink.snapshot_dir, exist_ok=True)
    path = snapshot_path(sink, rev)
    tmp = path + ".new"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump({"catalog_rev": rev,
                     "items": list(items_serialisable)},
                    fh, ensure_ascii=False, sort_keys=True, indent=2)
    os.replace(tmp, path)
    return path


def list_snapshots(sink: SnapshotSink) -> List[int]:
    """Return sorted rev numbers of on-disk snapshots."""
    if not os.path.isdir(sink.snapshot_dir):
        return []
    out = []
    for name in os.listdir(sink.snapshot_dir):
        if (name.startswith(SNAPSHOT_FILE_PREFIX)
                and name.endswith(SNAPSHOT_FILE_SUFFIX)):
            core = name[len(SNAPSHOT_FILE_PREFIX):
                          -len(SNAPSHOT_FILE_SUFFIX)]
            try:
                out.append(int(core))
            except ValueError:
                continue
    return sorted(out)


def latest_snapshot_rev(sink: SnapshotSink) -> Optional[int]:
    revs = list_snapshots(sink)
    return revs[-1] if revs else None


def restore_from_snapshot(sink: SnapshotSink, rev: int) -> dict:
    """Return the parsed snapshot dict; caller replays into fresh
    geo.db. This function is intentionally I/O-only (no DB writes)
    so it can be unit tested without a live SQLite."""
    path = snapshot_path(sink, rev)
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


class RestoreWithoutResyncError(Exception):
    """Restore path completed but the required resync was skipped."""


@dataclass
class RestoreCoordinator:
    """Small state machine that enforces the restore -> resync
    ordering guaranteed by §7.11.2."""
    restore_completed: bool = False
    resync_emitted: bool = False

    def on_restore_done(self) -> None:
        self.restore_completed = True

    def on_resync_emitted(self) -> None:
        if not self.restore_completed:
            raise RestoreWithoutResyncError(
                "resync emitted without a preceding restore")
        self.resync_emitted = True

    def assert_complete(self) -> None:
        if self.restore_completed and not self.resync_emitted:
            raise RestoreWithoutResyncError(
                "restore completed but resync never emitted; "
                "cloud may have stale catalog")
