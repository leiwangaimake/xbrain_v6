"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_rev.py
Brief: BIZ-P3-27 cmd/geo + geo_object rev + content_hash + tombstone + SN-5 pending push

Description:
15 §5.9 rev is monotonically increasing PER OBJECT. The invariants:

  SN-5 in-order push: object push order is by rev ASC per object;
       across objects the order is push_id ASC (submission order).

  content_hash uniqueness: SAME hash ==> SAME rev (bump is a no-op).
       The DAO layer enforces this in GeoObjectDAO.bump_rev; here we
       define hash computation and tombstone semantics.

  Tombstone: an object with tombstone=1 is 'deleted'. It stays in
       the table so pending_push can propagate the deletion. A
       delete is a rev bump PLUS setting tombstone=1 in the SAME
       transaction.

  Same-transaction pending_push enqueue: the mutation to a geo
       object must enqueue its push_id in the SAME BEGIN IMMEDIATE
       transaction. Otherwise the outbound queue can lose an
       edit on a crash between the two writes.
"""

from __future__ import annotations

import hashlib
import json


def content_hash(payload: dict) -> str:
    """Deterministic hash over a JSON-serialisable payload.
    Sort keys so a dict rebuilt in a different order produces the
    same hash. hex digest (64 chars)."""
    encoded = json.dumps(payload, sort_keys=True, ensure_ascii=False,
                           separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def is_same_content(hash_a: str, hash_b: str) -> bool:
    """Idempotency helper: equal hash means no bump."""
    return hash_a == hash_b


def tombstone_delete_row(rev_before: int) -> dict:
    """Field-set for a delete op. New rev = rev_before + 1;
    tombstone=1; content_hash frozen at the pre-delete value."""
    return {"rev": rev_before + 1, "tombstone": 1}


def push_order_key(push_id: int, rev: int) -> tuple:
    """SN-5 sort key: within a single object -> rev ASC; across
    objects -> push_id ASC. The composite (push_id, rev) with
    rev-as-tiebreaker preserves both orders when a batch is
    stable-sorted."""
    return (push_id, rev)


class PushOrderViolation(Exception):
    """SN-5 detected: a lower push_id has a higher rev for the same
    object. Should be impossible if enqueue happens in the same tx
    as the mutation."""


def validate_push_batch(entries) -> None:
    """entries: list of (push_id, kind, object_id, rev). Detect a
    per-object rev inversion (SN-5). O(n)."""
    seen_rev: dict = {}
    ordered = sorted(entries, key=lambda e: e[0])
    for push_id, kind, obj_id, rev in ordered:
        key = (kind, obj_id)
        prev = seen_rev.get(key)
        if prev is not None and rev < prev:
            raise PushOrderViolation(
                f"object {obj_id!r} rev {rev} arrived after "
                f"rev {prev} at push_id={push_id}")
        seen_rev[key] = rev
