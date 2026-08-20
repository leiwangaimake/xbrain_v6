"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_request.py
Brief: F11-F15 voice intents -> cmd/geo GeoCommand (11 S12A.1)

Description:
The CRUD half of the F class: delete (F11/F12/F13), rename (F14) and activate
(F15). These act on an object that already exists, so unlike the recording
intents they need to resolve a spoken NAME into a geo_id first.

*** Resolution, and why it refuses rather than guesses.

The operator says "delete the east gate route". The name space is the live
GeoManifest (11 S7.10), which carries name, num and state per object. This
module matches, in order: exact name, then an alias, then the spoken number
("route three"). If none of those produces EXACTLY ONE object, it raises and the
caller asks again.

A near-match is not accepted, and that is the whole point: the operator is about
to delete something. "Did you mean the east gate route?" costs a second; acting
on the wrong route is not recoverable from voice.

*** On F13 (delete fence), which is cloud-only.

11 S7.9.5 denies `delete fence` to every channel except cloud, so a spoken F13
will come back E_CHANNEL_DENIED. This module builds and sends it anyway rather
than refusing locally. The reason is single-source: CH-1 makes origin the sole
permission discriminant and P3 owns that matrix. A second copy of the policy
here would be a second thing to keep in step, and the failure mode of a drifted
copy is a channel silently gaining or losing a permission. The cost is one round
trip before the operator is told no.
"""
from __future__ import annotations

import re
from typing import Any, Dict, List, Mapping, Optional, Sequence

#: 18 intent name -> (geo action, geo type, slot carrying the target).
_GEO_INTENTS: Dict[str, tuple] = {
    "delete_route": ("delete", "route", "route"),          # F11
    "delete_waypoint": ("delete", "waypoint", "waypoint"),  # F12
    "delete_fence": ("delete", "fence", "fence"),          # F13 (cloud-only)
    "rename_object": ("rename", None, "old"),              # F14
    "set_active_fence": ("set_state", "fence", "fence"),   # F15
}

#: Spoken numbers the operator uses for `num` ("three hao lu jing"). Only the
#: digits are handled here; Chinese numerals are the ASR/LLM's to normalise, and
#: duplicating that table would give the system two answers for "three".
_NUM_RE = re.compile(r"^\s*(\d{1,2})\s*$")


class GeoRequestError(RuntimeError):
    """The target could not be resolved to exactly one object."""


def is_geo_intent(intent_name: str) -> bool:
    return intent_name in _GEO_INTENTS


def resolve_geo_id(items: Sequence[Mapping[str, Any]], gtype: Optional[str],
                   spoken: str) -> str:
    """Resolve a spoken target to one geo_id, or raise.

    items is GeoManifest.items (11 S7.10). Tombstones are excluded: deleting an
    already-deleted route is not what the operator meant, and it would make an
    ambiguous name resolve to the deleted one.
    """
    text = (spoken or "").strip()
    if not text:
        raise GeoRequestError("no target was named")
    live = [i for i in items
            if i.get("state") != "deleted"
            and (gtype is None or i.get("type") == gtype)]
    exact = [i for i in live if i.get("name") == text]
    if len(exact) == 1:
        return str(exact[0]["geo_id"])
    if len(exact) > 1:
        # name is UNIQUE per table in the schema, so this can only happen across
        # types when gtype is None (F14 rename without a type slot).
        raise GeoRequestError(f"{text!r} names more than one object")
    alias = [i for i in live if text in (i.get("alias") or [])]
    if len(alias) == 1:
        return str(alias[0]["geo_id"])
    m = _NUM_RE.match(text)
    if m:
        num = int(m.group(1))
        by_num = [i for i in live if i.get("num") == num]
        if len(by_num) == 1:
            return str(by_num[0]["geo_id"])
        if len(by_num) > 1:
            raise GeoRequestError(f"more than one object is numbered {num}")
    raise GeoRequestError(f"no {gtype or 'object'} named {text!r}")


def to_geo_command(intent_name: str, *, slots: Mapping[str, Any], cmd_id: str,
                   manifest: Optional[Mapping[str, Any]],
                   origin: str = "voice") -> Optional[Dict[str, Any]]:
    """Build the cmd/geo payload for an F11-F15 turn, or None if not one.

    Raises GeoRequestError when the target cannot be resolved -- the caller
    speaks that reason. base_rev comes from the manifest, which is what makes
    the optimistic-concurrency check meaningful: sending 0 would conflict with
    every object that has ever been edited.
    """
    entry = _GEO_INTENTS.get(intent_name)
    if entry is None:
        return None
    action, gtype, slot_name = entry
    if not isinstance(manifest, Mapping) or not manifest.get("items"):
        # Without the manifest there is no name space to resolve against. Said
        # plainly rather than sending geo_id=<the spoken text>, which would be
        # refused for the wrong reason (bad id prefix) and teach the operator
        # that the name was wrong when the catalogue was simply not loaded.
        raise GeoRequestError("the object catalogue is not available yet")
    items: List[Mapping[str, Any]] = list(manifest["items"])
    if action == "rename":
        # F14 carries type / old / new. The type slot narrows the search when
        # the operator gave one ("rename the THIRD ROUTE"); without it the
        # resolver searches every type and refuses an ambiguous name.
        gtype = slots.get("type") if isinstance(slots.get("type"), str) else None
    spoken = slots.get(slot_name)
    if not isinstance(spoken, str):
        raise GeoRequestError(f"the {slot_name} was not understood")
    geo_id = resolve_geo_id(items, gtype, spoken)
    item = next(i for i in items if i["geo_id"] == geo_id)
    payload: Dict[str, Any] = {
        "cmd_id": cmd_id,
        "action": action,
        "type": item.get("type") or gtype,
        "geo_id": geo_id,
        "origin": origin,
        "base_rev": item.get("rev", 0),
    }
    if action == "rename":
        new_name = slots.get("new")
        if not isinstance(new_name, str) or not new_name.strip():
            raise GeoRequestError("the new name was not understood")
        payload["obj"] = {"name": new_name.strip()}
    elif action == "set_state":
        # F15 "use the camp fence": activation, the L2 action that S12A.7
        # deliberately keeps separate from saving.
        payload["obj"] = {"state": "active"}
    return payload


def manifest_from_state(geo_manifest: Optional[Mapping[str, Any]]
                        ) -> Optional[Mapping[str, Any]]:
    """The manifest body out of a state/geo/manifest broadcast, or None."""
    if not isinstance(geo_manifest, Mapping):
        return None
    return geo_manifest if "items" in geo_manifest else None
