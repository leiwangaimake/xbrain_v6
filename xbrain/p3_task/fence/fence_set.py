"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fence_set.py
Brief: Build the 11 S9A.2 FenceSet (with crc32) from fence.db active rows

Description:
The GEOMETRY-BROADCAST half of the fence runtime (11 S9A.3). P3 is the sole
cmd/fence publisher; this module turns the active fence.db rows into a FenceSet
message (11 S9A.2) so P3 can broadcast the real stored geometry instead of the
demo injector. It:
  * runs validate_active_fence_set (11 S9A.1A FS-5A: exactly 1 allow, <= 5 total)
    -- FV-3 of the S9A.3 compile checklist -- BEFORE building, so an invalid set
    never goes on the wire;
  * shapes each row into a polygon {poly_id, role, name, winding, hard_enforce,
    vertices:[{lat,lon}]} (winding declared per role: allow->ccw, keep-out->cw;
    the receiver re-normalises per 11 S9A.2 and does not trust the sender);
  * computes crc32 by the EXACT 11 S9A.2 normalisation, which this module does
    NOT implement itself -- it delegates to xbrain.common.fence.geom, whose one
    implementation is xbrain.common.digest.fence.fence_crc32. CRC-32 IEEE 802.3
    == zlib.crc32, 8 lowercase hex, so the C++/Python receiver self-computes the
    SAME value (FV-8).

    *** CORRECTED 2026-10-09. This paragraph used to spell the recipe out as
    five segments: "fence_set_id|rev| then per polygon
    poly_id|role|winding|hard_enforce(1/0)| then each vertex %.8f,%.8f;".
    That was the v0.2 recipe. The current one has SEVEN segments per polygon --
    priority and, on a speed_limit polygon, speed_limit_mps sit between
    hard_enforce and the vertices (11 S9A.2, originally patch block B-3, merged
    into the contract body on 2026-10-09).

    The code was changed to the seven-segment form on 2026-10-08 and this
    header was not. Why that is worse than it sounds: the five-segment recipe
    makes "lower the slow zone" produce a byte-identical crc32, so S-5 answers
    duplicate and the change never takes effect with no error anywhere -- and
    anybody reading this header to implement a second producer would have
    reproduced exactly that bug. The recipe is now stated in ONE place (the
    contract) and implemented in ONE place (fence_crc32); this header points
    at them rather than restating either.

Scope: this is the geometry broadcast only. The two-stage FenceCommand
handshake of 11 S9A.3 (op=stage -> cmd/fence/ack -> op=commit +
require_ack_from + P1 state/fence) is NOT implemented here; P3 broadcasts the
FenceSet directly.

*** CORRECTED 2026-10-09. The original wording was "DEFERRED until P1 fence
clipping lands -- there is no consumer to ack yet". BOTH halves of that
justification have expired:
  * P1 fence clipping landed 2026-09-12 (xbrain/p1_motion/fence/clip.py) and
    was extended on 2026-10-08 (speed zones, candidate gate).
  * P1 publishes state/fence (11 S1.1.6 P1-14) and cmd/fence/ack (P1-16), so
    there IS a consumer to ack.
So the deferral's precondition is satisfied and the handshake is simply
missing. It is now registered as a gap in docs/NEXT.md rather than described
here as a deliberate wait -- a stale "deferred until X" reads as "somebody is
watching for X", and nobody was.

What the gap costs, concretely: 11 S2.2.3 (block C-2, merged 2026-10-09)
requires that when the active rule set references ANY warning polygon,
require_ack_from must include p2_core (S9A.13 SR-2). Without the handshake,
P2 can be running PER-36 suspicion checks against a stale rev of the warning
geometry while P3 has already committed a new set -- the two sides disagree
about which areas are alarm zones, with no error anywhere.

Do NOT read this module as the commit protocol.
"""
from __future__ import annotations

import json
import logging
from typing import Any, Callable, Dict, List, Optional, Sequence

from xbrain.common.enums import FENCE_ROLE  # 11 S9A.2 closed set (CLAUDE.md 3.5)

# fence_set_crc32 现由跨进程共享库导出 -- p1_motion 收到 cmd/fence 后自算比对必须
#与本进程用同一套归一化(11 S9A.2). re-import 保持 from ...fence_set import
# fence_set_crc32 的既有调用(test_fence_set / 跨语言金标)可用.
from xbrain.common.fence.geom import fence_set_crc32

# S9A.2 field-presence rule for speed_limit_mps, shared with the writers and
# with p1's receive-side FV-13 so one answer serves all of them.
from xbrain.common.fence.roles import carries_speed_limit, hard_enforce_for_role
from xbrain.p3_task.fence.geom import validate_active_fence_set

_logger = logging.getLogger(__name__)

# Declared winding per role (11 S9A.2: receiver normalises, this is a hint). Keys
# ARE the FENCE_ROLE closed set -- asserted at import so adding a role to 11 S9A.2
# without a winding here fails loudly (same shape as limiter_cn / RS-LIM).
_WINDING = {"allow": "ccw", "forbid": "cw", "speed_limit": "cw", "warning": "cw"}
assert set(_WINDING) == set(FENCE_ROLE), (
    "fence_set._WINDING keys must cover FENCE_ROLE (11 S9A.2)")


def _vertices(geom_json: str) -> List[Dict[str, float]]:
    """fence.db geom_json {"points": [[lat,lon],...]} -> [{lat,lon},...]."""
    pts = json.loads(geom_json).get("points", [])
    return [{"lat": float(la), "lon": float(lo)} for la, lo in pts]


def build_fence_set(active_rows: Sequence, *, fence_set_id: str, rev: int,
                    enu_origin: Optional[Dict[str, float]] = None,
                    soft_margin_min_m: Optional[float] = None,
                    on_unusable: Optional[Callable[[str, str], None]] = None
                    ) -> Dict[str, Any]:
    """Build a 11 S9A.2 FenceSet from FencesDAO.list_active() rows. Each row is
    (fence_id, name, role, kind, geom_json, hard_enforce, rev, priority,
    speed_limit_mps). Raises InvalidFenceSet (FV-3) if the active set is not
    exactly-1-allow / <= 5.

    *** NOTHING here depends on what this robot's fence.db happens to hold.
    A fence set is whatever the customer recorded on THEIR site: any count
    within FV-3, any role mix, any ring shape, anywhere on earth. The one
    site-specific number in the whole chain is enu_origin, and that comes from
    common.geo.enu_origin (config), not from a fence row.

    on_unusable(poly_id, reason) is called for a stored row that cannot be put
    on the wire; that polygon is DROPPED and the rest of the set still goes
    out. Default is a module-level warning, so a silent drop is not reachable
    by forgetting the argument -- callers with a once-per-id latch pass their
    own (the 1 Hz broadcast loop would otherwise log every second)."""
    roles = [r[2] for r in active_rows]
    validate_active_fence_set(roles)               # FV-3 / S9A.1A -- raises on bad set
    polygons: List[Dict[str, Any]] = []
    for (fence_id, name, role, _kind, geom_json, _hard_enforce, _rev,
         priority, speed_limit_mps) in active_rows:
        poly: Dict[str, Any] = {
            "poly_id": fence_id,
            "role": role,
            "name": name,
            "winding": _WINDING.get(role, "ccw"),
            # S9A.2 DERIVES this from role, and it is derived again here
            # rather than passed through. The stored column is right for every
            # row the current writers made, and can be wrong for any row that
            # predates them -- the superseded rule knew only about warning, so
            # a speed_limit row it wrote is stored hard-enforcing. Passing that
            # through would put a keep-out on the wire for a zone the operator
            # marked advisory. p1 normalises it again on receipt (S9A.2
            # "接收方按 role 归一化, 不得信任发送方"), but there is no reason
            # for the sender to state something it knows is wrong, and the
            # crc32 is computed over what is actually sent so the two sides
            # still agree byte for byte. The column stays unpacked as
            # _hard_enforce (the file's convention for a field the row carries
            # and this builder does not read) so the row shape stays
            # documented.
            "hard_enforce": hard_enforce_for_role(role),
            # S9A.2 block B-2: required on every polygon, painter order, low
            # paints first. int() because SQLite hands back whatever was stored
            # and the recipe renders an int32 -- a float 0.0 would raise in the
            # crc32 formatter rather than silently become "0.0" and disagree
            # with the C++ receiver's std::to_string on an int32.
            "priority": int(priority),
            "vertices": _vertices(geom_json),
        }
        # S9A.2: "role != speed_limit 时必须省略, 出现即 E_SCHEMA". The key is
        # OMITTED rather than sent as null -- a null would be a present field
        # with no value, which FV-13's reverse direction rejects at the
        # receiver, so every non-speed_limit polygon would be refused.
        if carries_speed_limit(role):
            if speed_limit_mps is None:
                # A stored speed_limit row with no limit. Only reachable from a
                # row written before the column existed -- the DAO, geo_object
                # and the CREATE TABLE form all refuse it now (FV-13).
                #
                # *** This DROPS the one polygon and keeps broadcasting the
                # rest; it does NOT refuse the set. The first cut raised here,
                # and that trade is backwards: refusing the set stops the
                # broadcast, and a FRESHLY STARTED p1 then holds no fence at
                # all -- allow / forbid stop being enforced -- to protect an
                # in-zone limit that does not exist anyway. FS-7 only saves a
                # p1 that already received a good set. So the safety-critical
                # boundaries stay in force and the unusable zone stays
                # unusable, which is what it already was.
                # NO no default is invented for it (3.1): whatever number were
                # chosen would silently become a real zone's speed ceiling.
                if on_unusable is not None:
                    on_unusable(fence_id, "role=speed_limit with no "
                                          "speed_limit_mps (11 FV-13)")
                else:
                    _logger.warning(
                        "fence %s dropped from the broadcast: role=speed_limit "
                        "with no speed_limit_mps (11 FV-13); set a limit or "
                        "deactivate the fence", fence_id)
                continue
            poly["speed_limit_mps"] = float(speed_limit_mps)
        polygons.append(poly)
    return {
        "fence_set_id": fence_set_id,
        "rev": rev,
        "crc32": fence_set_crc32(fence_set_id, rev, polygons),
        "enu_origin": enu_origin,
        "soft_margin_min_m": soft_margin_min_m,
        "polygons": polygons,
    }
