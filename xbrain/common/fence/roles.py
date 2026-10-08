"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: roles.py
Brief: the facts a FenceSet polygon's role DERIVES, spelled once (11 S9A.2)

Description:
Why this exists: 11 S9A.2's field table says two things about a polygon that a
writer must not take from the sender but derive from `role` -- whether the
polygon hard-enforces, and whether it carries a speed limit. Before this module
the first of those was hand-written in three p3 writers as

    hard_enforce = 0 if role == "warning" else 1

which is the contract's rule for ONE of the two advisory roles. S9A.2 reads
"role in {warning, speed_limit} 恒为 false, 填 true 即 E_SCHEMA" -- so a
speed_limit row was stored hard-enforcing, and once speed_limit compiles into
the P1 clip geometry (12 S7) that row reads as a keep-out: the robot cannot
enter the slow zone at all, which is the opposite of what a slow zone means.
Three copies of a rule is how one of them ends up being the old rule, so the
derivation lives here and the writers call it. Same reason the role closed set
itself is shared (common/enums, CLAUDE.md 3.5) rather than re-typed per table.

Who uses it: p3 writers (ingest/geo_object._fence, ingest/geo_commit, the
FencesDAO) on the way INTO fence.db, and p1 fence_set._parse_polygon on the way
out of the wire, where S9A.2's "接收方必须按 role 归一化, 不得信任发送方" makes
the same derivation a receive-side normalisation (FV-13).

What it does NOT do: geometry (geom.py), the crc32 recipe (common/digest/
fence.py), or any judgement about WHAT the limit should be -- the value is the
operator's, carried in speed_limit_mps, and 3.1 forbids a default for it.

Things that look right and are wrong:
  * deriving hard_enforce from "did the sender set soft_margin_min_m". The two
    are independent: an allow fence may declare a soft band and still hard
    enforce at the inset, and an advisory zone may declare one and never clip.
  * treating a missing speed_limit_mps on a speed_limit polygon as 0.0, or as
    "no limit". Both are 3.1 violations in opposite directions -- 0.0 pins the
    robot at a standstill inside the zone with no error, and "no limit" quietly
    switches the zone off. S9A.2 makes it required, so absence is E_SCHEMA and
    this module only answers whether the field BELONGS, never what it is.
"""
from __future__ import annotations

from typing import Tuple

#: 11 S9A.2: the one role whose polygons carry speed_limit_mps. Spelled here as
#: well as in common/digest/fence.py on purpose -- that module is a leaf (the
#: crc32 recipe imports nothing of ours so the C++ twin can be read against it
#: line by line), and the two spellings are pinned equal by a metatest rather
#: than by an import that would make the leaf depend on this file.
SPEED_LIMIT_ROLE = "speed_limit"
#: 11 S9A.2 field table: the roles that NEVER hard-enforce. warning is advisory
#: by definition (it only raises events, 12 FS-12); speed_limit constrains the
#: speed inside itself and never the ability to be there, so a hard enforcement
#: of it would be a keep-out zone wearing a speed sign.
ADVISORY_ROLES: Tuple[str, ...] = ("warning", SPEED_LIMIT_ROLE)


def hard_enforce_for_role(role: str) -> bool:
    """11 S9A.2: hard_enforce DERIVED from role, never taken from the sender.

    False for the advisory roles, True otherwise. Note what is deliberately
    absent: there is no "respect the sender's value when the role allows it"
    branch. allow / forbid always hard-enforce in this system -- S9A.2 lists
    hard_enforce as a field so a future advisory keep-out is expressible, and
    9.3 forbids writing the code that would consume that possibility before it
    exists. A writer wanting a soft keep-out today would be describing a
    different design, not using a flag.
    """
    return role not in ADVISORY_ROLES


def carries_speed_limit(role: str) -> bool:
    """11 FV-13, the field-presence half: True only for role=speed_limit.

    Both directions are the caller's to enforce (required when True, forbidden
    when False) because the two failures want different messages -- a slow zone
    with no limit is an unusable zone, a limit on a forbid polygon is a value
    nobody will ever read. This function only says which side of the rule the
    role is on, so the two places that check it cannot disagree about that much.
    """
    return role == SPEED_LIMIT_ROLE


__all__ = ["SPEED_LIMIT_ROLE", "ADVISORY_ROLES", "hard_enforce_for_role",
           "carries_speed_limit"]
