"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_roles.py
Brief: 11 S9A.2 role-derived facts -- hard_enforce and speed_limit_mps presence

Description:
Pins common/fence/roles, the one place the two derivations live. The checks here
are deliberately boring; what is NOT boring is why the module exists, and that
is what the first test guards.

The rule the live code had for a year was

    hard_enforce = 0 if role == "warning" else 1

written out in three p3 writers. 11 S9A.2's field table says hard_enforce is
false for role in {warning, speed_limit} -- so a speed_limit row was stored
hard-enforcing, in all three. It did nothing visible, because clip.py drops
speed_limit polygons before they reach the geometry; it would have become
visible the moment speed_limit compiled, as a slow zone the robot could not
enter. A test that only asserted the warning case would have passed against the
broken rule, which is why test_both_advisory_roles_never_hard_enforce names
speed_limit explicitly rather than iterating ADVISORY_ROLES (iterating the
module's own tuple would pass no matter what that tuple said -- 3.2 form (7),
a definition posing as a measured result).
"""
from __future__ import annotations

import pytest

from xbrain.common.digest.fence import SPEED_LIMIT_ROLE as DIGEST_SPEED_LIMIT
from xbrain.common.enums import FENCE_ROLE
from xbrain.common.fence.roles import (
    ADVISORY_ROLES,
    SPEED_LIMIT_ROLE,
    carries_speed_limit,
    hard_enforce_for_role,
)

pytestmark = pytest.mark.no_device


def test_both_advisory_roles_never_hard_enforce():
    # The four roles are spelled out one per line rather than looped over
    # ADVISORY_ROLES. Looping would assert that the function agrees with the
    # tuple beside it, which it does by construction -- a test that cannot
    # fail for the reason it exists (3.2 form (7)).
    assert hard_enforce_for_role("warning") is False
    # THE line this file is for. MUTATION: `return role != "warning"` -- the
    # rule the three p3 writers actually carried -- reddens here and nowhere
    # else, which is why nothing noticed it for as long as it was there.
    assert hard_enforce_for_role("speed_limit") is False
    # The other direction has to be pinned too. A function answering False for
    # everything would satisfy the two lines above and switch off fence
    # clipping entirely: allow and forbid would stop hard-enforcing and the
    # robot would be free to leave the camp, with every test still passing.
    assert hard_enforce_for_role("allow") is True
    assert hard_enforce_for_role("forbid") is True
    # `is True` / `is False`, not truthiness. The value goes straight into a
    # JSON field (S9A.2 types it bool) and a SQLite 0/1 column; a 1 or a "yes"
    # passes an == comparison and then renders as "1" on one side and "true" on
    # the other -- the cross-language divergence canonical_fence_string warns of.


def test_speed_limit_is_the_only_role_carrying_a_limit():
    # 11 FV-13's presence half: WHICH role may carry speed_limit_mps. Whether a
    # present-or-absent value is then legal is the caller's half, deliberately
    # not here -- the two failures want different messages (a slow zone with no
    # limit is unusable; a limit on a keep-out is a number the operator believes
    # is in force and no consumer will ever read).
    assert carries_speed_limit("speed_limit") is True
    # MUTATION: returning True for forbid lets a limit be stored on a keep-out.
    assert carries_speed_limit("allow") is False
    assert carries_speed_limit("forbid") is False
    # The three negatives matter as much as the positive: a function returning
    # True everywhere would make every writer demand a limit on the camp
    # boundary, and the allow fence could not be stored at all.
    assert carries_speed_limit("warning") is False


def test_the_roles_named_here_are_the_contract_closed_set():
    # Guards the gap the two tests above cannot see: a FIFTH role added to
    # 11 S9A.2 would get hard_enforce=True from the fall-through and nobody
    # would be asked whether that is right. MUTATION: adding a role to
    # sets.yaml without deciding its enforcement reddens here.
    assert set(FENCE_ROLE) == {"allow", "forbid", "speed_limit", "warning"}
    # ADVISORY_ROLES must be a SUBSET of the closed set, not an arbitrary tuple:
    # a typo ("speedlimit") would make hard_enforce_for_role return True for the
    # real role and this is the only check that notices.
    assert set(ADVISORY_ROLES) <= set(FENCE_ROLE)


def test_the_two_spellings_of_the_speed_limit_role_agree():
    # common/digest/fence.py keeps its own literal on purpose (it is a leaf, so
    # the C++ twin can be read against it line by line). Two spellings of one
    # string drift; this is the pin that makes them not. MUTATION: changing
    # either literal reddens here instead of silently producing a crc32 that
    # omits every zone's limit again.
    assert SPEED_LIMIT_ROLE == DIGEST_SPEED_LIMIT
