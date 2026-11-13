"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: settings.py
Brief: 99 U185 -- the speed-limit default of 限速区域 fences: cloud value, else the configured 0.6

Description:
User ruling 2026-11-13: the in-zone limit of a speed-limit fence comes from
the cloud; not sent = "缺损", 0.6 m/s; the robot persists what the cloud sent
and uses it after a restart; a new value overrides the old one; the cloud may
delete it, which goes back to 0.6. Agreed refinement: a zone the cloud sends
WITH its own limit keeps it (speed_limit_own = 1); every other zone -- all
recorded ones -- follows the default (own = 0).

Three levels, highest first:
  1. the zone's own value (fences.speed_limit_mps where speed_limit_own = 1);
  2. the cloud default (fence_settings key speed_limit_default_mps);
  3. the configured default (p3_task.yaml fence.speed_limit_default_mps, 0.6
     by the user's ruling -- a ruled value written in configuration, not a
     code default: CLAUDE.md 3.1 stays satisfied, the number lives in one file).

How "follows the default" is stored: speed_limit_mps of an own = 0 row holds
the default as of the last change, and apply_speed_default / reconcile REWRITE
those rows whenever the effective default moves. The FenceSet builder, the
geo broadcast and the cloud manifest then read one column and never need to
know the default exists -- and the row's rev / content_hash change with it, so
every change detector (cmd/fence crc32, catalog_rev) sees the new limit. The
alternative, resolving at read time, would leave three readers each having to
apply the same rule, the shape this repository has seen drift before.

Not here: the range check of a value (the cmd/geo applier does it against the
configured bounds) and the speed gate that applies the limit (P1 host_gate,
11 S9A.2 block B-2).
"""

from __future__ import annotations  # postponed annotations

import json  # value_json
from typing import Optional, Tuple  # annotations

from xbrain.p3_task.state.geo_rev import content_hash  # the row hash moves with the limit

SPEED_DEFAULT_KEY = "speed_limit_default_mps"  # the one fence_settings key today


async def stored_speed_default(conn) -> Optional[float]:
    """The cloud default from fence_settings, or None when not set."""
    cur = await conn.execute("SELECT value_json FROM fence_settings WHERE key=?",
                             (SPEED_DEFAULT_KEY,))  # one row or none
    row = await cur.fetchone()  # None: the cloud never set it (or deleted it)
    return None if row is None else float(json.loads(row[0]))  # stored as JSON number


async def effective_speed_default(conn, configured: Optional[float]) -> Optional[float]:
    """Level 2 if set, else level 3. None only when neither exists (a caller
    built without the configured value, i.e. a test): the caller refuses."""
    stored = await stored_speed_default(conn)  # cloud value first
    return stored if stored is not None else configured  # then the configured 0.6


async def _rewrite_followers(conn, value: float, now_ms: int) -> int:
    """Rewrite every live own = 0 speed zone to `value`; returns how many rows
    changed (rows already at the value are left alone, so a no-op call bumps
    no rev). Caller owns the transaction."""
    cur = await conn.execute(  # the zones that follow the default and differ
        "SELECT fence_id, rev, content_hash FROM fences WHERE role='speed_limit' "
        "AND speed_limit_own=0 AND tombstone=0 AND state<>'deleted' "
        "AND (speed_limit_mps IS NULL OR speed_limit_mps<>?)", (value,))
    rows = await cur.fetchall()  # each needs the new value
    for fid, rev, old_hash in rows:  # one UPDATE per row: rev and hash move too
        await conn.execute(
            "UPDATE fences SET speed_limit_mps=?, rev=?, content_hash=?, updated_ms=? "
            "WHERE fence_id=?",
            (value, int(rev) + 1,
             # the stored hash covered the old limit; chain it with the new one
             # so "did this row change" answers yes (crc32 changes on its own)
             content_hash({"prev": old_hash, "speed_limit_mps": value}),
             now_ms, fid))
    return len(rows)  # 0 when nothing moved


async def write_speed_default(conn, value: Optional[float], configured: Optional[float],
                              now_ms: int) -> Tuple[Optional[float], int, list]:
    """Set (value) or delete (None) the cloud default and rewrite the zones
    that follow it, inside the CALLER's transaction (the cmd/geo applier logs
    the command in the same one). Returns (effective default, rows rewritten,
    ids of the ACTIVE rows rewritten -- what changes in force)."""
    if value is None:  # deleted: back to the configured default
        # an absent row is how "not set" is stored, so a delete is just that
        await conn.execute("DELETE FROM fence_settings WHERE key=?", (SPEED_DEFAULT_KEY,))
    else:  # a new cloud value overrides the old one
        await conn.execute(
            "INSERT INTO fence_settings (key, value_json, updated_ms) VALUES (?,?,?) "
            "ON CONFLICT(key) DO UPDATE SET value_json=excluded.value_json, "
            "updated_ms=excluded.updated_ms",
            (SPEED_DEFAULT_KEY, json.dumps(float(value)), now_ms))
    effective = value if value is not None else configured  # what now applies
    active = []  # ids whose change reaches P1
    changed = 0  # rows rewritten
    if effective is not None:  # nothing to rewrite to without a value
        cur = await conn.execute(  # the active followers that will move
            "SELECT fence_id FROM fences WHERE role='speed_limit' AND speed_limit_own=0 "
            "AND tombstone=0 AND state='active' "
            "AND (speed_limit_mps IS NULL OR speed_limit_mps<>?)", (effective,))
        active = [r[0] for r in await cur.fetchall()]  # read before the rewrite
        changed = await _rewrite_followers(conn, effective, now_ms)  # then rewrite
    return effective, changed, active


async def reconcile_speed_default(conn, configured: Optional[float], now_ms: int) -> int:
    """At P3 start: make every follower carry the effective default (the
    configured value may have changed in p3_task.yaml since the last run).
    Returns the number of rows rewritten (0 on a normal start)."""
    effective = await effective_speed_default(conn, configured)  # level 2 or 3
    if effective is None:  # neither: nothing to reconcile to
        # (a production P3 always has the configured value: __main__ exits 78 without it)
        return 0
    await conn.execute("BEGIN IMMEDIATE")  # rows together
    try:
        changed = await _rewrite_followers(conn, effective, now_ms)  # bring them in line
        await conn.commit()
    except Exception:
        await conn.rollback()  # untouched
        raise
    return changed


# -- 99 U187: the autonomous-alarm settings of the warning zones ---------------
#
# User ruling 2026-11-13: the time window ("每天从什么时间到什么时间有效", sent
# by the cloud; not sent = "缺损", all day; persisted; a new one overrides;
# deletable) and the suspicious targets (sent by the cloud, several at once;
# persisted; add / delete by sending the new full list). Stored as fence.db
# fence_settings rows -- the robot-side truth P3 owns -- and broadcast to P2 on
# cmd/fence (alarm_view), where P2 already gets the warning polygons.
#
# The target classes: "人" = perception person / bicycle / motorcycle (a rider
# is a person), "车" = vehicle, "动物" = animal (birds excluded, user
# 2026-11-13). The cloud names them by rule type (v2.0 S2.4 rules[].type).
# Not sent = the user's default: people, in every warning zone.

#: the alarm keys set_fence_defaults accepts besides the speed default
ALARM_KEYS = ("alarm_window", "alarm_rules", "alarm_level", "siren_level",
              "duration_sec", "cooldown_sec")  # the six cloud fields (SET_ALARM_CONFIG names, rules -> alarm_rules)

#: rule type -> the target class it watches for (99 U187)
RULE_TARGET = {"person_in_region": "person", "vehicle_in_region": "vehicle",
               "animal_in_region": "animal"}  # 人 / 车 / 动物; the bird exclusion is applied where the class is matched (P2)

#: the user's default when the cloud never sent rules (or deleted them)
DEFAULT_RULES = ({"targets": ["person"], "region_ids": None},)  # None = every warning zone


async def write_setting(conn, key: str, value, now_ms: int) -> None:
    """Set (value) or delete (None) one fence_settings row, inside the
    caller's transaction. JSON so a window object / a rule list round-trips."""
    if value is None:  # deleted: the robot falls back to the default
        await conn.execute("DELETE FROM fence_settings WHERE key=?", (key,))  # gone: the default applies again
        return  # nothing more to write
    await conn.execute(  # upsert: a new value overrides the old one
        "INSERT INTO fence_settings (key, value_json, updated_ms) VALUES (?,?,?) "
        "ON CONFLICT(key) DO UPDATE SET value_json=excluded.value_json, "
        "updated_ms=excluded.updated_ms",
        (key, json.dumps(value, ensure_ascii=False, separators=(",", ":")), now_ms))  # compact JSON, Chinese names kept readable


async def read_settings(conn) -> dict:
    """Every stored fence_settings row as {key: value}."""
    cur = await conn.execute("SELECT key, value_json FROM fence_settings")  # all rows
    return {k: json.loads(v) for k, v in await cur.fetchall()}  # decoded


async def alarm_view(conn) -> dict:
    """The effective autonomous-alarm settings for the cmd/fence broadcast:
    window (None = all day), the watched targets per rule with the default
    applied, and the siren scalars as stored (None = P2's own default). Each
    entry says where it came from so P2 / the HMI can tell a default apart."""
    s = await read_settings(conn)  # what the cloud stored
    rules = s.get("alarm_rules")  # None: never sent / deleted
    if rules is None:  # the cloud never sent rules, or deleted them
        watched, source = [dict(r) for r in DEFAULT_RULES], "default"  # people, everywhere
    else:  # the cloud list, as sent
        watched = [{"targets": [RULE_TARGET[r["type"]]],  # one class per rule type
                    "region_ids": list(r.get("region_ids") or []) or None}  # [] = every zone
                   for r in rules if r.get("enabled", True) and r.get("type") in RULE_TARGET]
        source = "cloud"  # as sent (an all-disabled list watches nothing, on purpose)
    return {"window": s.get("alarm_window"),  # None = all day
            "rules": watched, "rules_source": source,
            "alarm_level": s.get("alarm_level"), "siren_level": s.get("siren_level"),
            "duration_sec": s.get("duration_sec"), "cooldown_sec": s.get("cooldown_sec")}  # None = P2 uses its own configured default


__all__ = ["ALARM_KEYS", "DEFAULT_RULES", "RULE_TARGET", "SPEED_DEFAULT_KEY", "alarm_view",
           "effective_speed_default", "read_settings", "reconcile_speed_default",
           "stored_speed_default", "write_setting", "write_speed_default"]
