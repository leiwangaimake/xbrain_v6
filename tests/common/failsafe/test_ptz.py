"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz.py
Brief: Tests for the PTZ fail-safes (T-PTZ-1 homing, T-PTZ-3 speed/move/zoom), incl. two named mutants

Description:
Acceptance surface for INF-DB-3 branches (2) and (3). It pins: preset_effective
null/absent refusing boot; E02/E03/E04 rejecting with E_CAPABILITY and never
accepted; E10 always rejected with a guide to E01 and no motion estimate; E09
emitting a closed-set gear name with no degree/second value; G33 answering a gear
with no magnification. It also binds the E09 gear set to 18-B S2 by symmetric
difference, so the closed set cannot drift from the contract.

*** The two named mutants for these branches (INF-DB-3 done-criterion), each run
and confirmed red before this file was done (CLAUDE.md 3.3):
  * "用 accepted 冒充'已到位'": make ptz_preset_failsafe return
    status=STATUS_ACCEPTED. test_preset_never_returns_accepted and
    test_preset_intents_rejected go red. accepted on this hardware is
    indistinguishable from "转对了" (21 S1), so returning it is the fake
    guarantee this catches.
  * "给三档写度每秒当量": add a deg_per_s key to PtzSpeedGear.to_wire (or a gear->
    omega table feeding it). test_e09_gear_wire_is_level_only goes red. Writing an
    omega fabricates the T-PTZ-3 curve nobody has measured.

A third and fourth mutant are covered here too, for the assertions this file adds
beyond the two named: coercing preset_effective null to False (boot guard) makes
test_preset_effective_null_refuses_boot red; attaching a magnification to the G33
answer makes test_g33_answer_is_gear_only red.
"""

import os
import re
import sys

import pytest

# tests/common/failsafe -> repo root is four levels up.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common import errors  # noqa: E402
from xbrain.common.config import MISSING  # noqa: E402
from xbrain.common.errors.exceptions import ClosedSetViolation, XbrainError  # noqa: E402
# Imported from the submodules directly: the package __init__ is docstring-only
# (it does not re-export), the same convention as xbrain/common/__init__.py.
from xbrain.common.failsafe.outcome import STATUS_ACCEPTED, STATUS_REJECTED  # noqa: E402
from xbrain.common.failsafe.ptz import (  # noqa: E402
    GUIDE_USE_E01,
    PRESET_EFFECTIVE_KEY,
    PTZ_PRESET_INTENTS,
    PTZ_SPEED_LEVELS,
    parse_ptz_speed_level,
    ptz_move_deg_failsafe,
    ptz_preset_failsafe,
    query_ptz_zoom_gear,
    require_preset_effective,
    resolve_ptz_speed_gear,
)

# 18-B is where E02/E03/E04 (S1) and the E09 gear set (S2) are defined.
VOL18B = os.path.join(ROOT, "docs", "18-B-云台指令扩展.md")


def _first_cell_token(line):
    """The backticked identifier in a markdown row's first cell, or None.

    The E09 table has no escaped pipes, so a plain split is enough; only the first
    cell is read, so the `fast` that appears in a later semantics cell is ignored.
    """
    parts = [c.strip() for c in line.strip().strip("|").split("|")]
    if not parts:
        return None
    m = re.search(r"`([a-z_]+)`", parts[0])
    return m.group(1) if m else None


def _parse_e09_levels():
    """The E09 speed levels as written in 18-B S2, for the symmetric-diff test.

    Section 2 only (heading `## 2.`), stopping at the next `## ` heading, so the
    E10 slot table further down cannot leak values in. The header row's `level`
    token is dropped -- it names the column, it is not a member.
    """
    lines = open(VOL18B, encoding="utf-8").read().split("\n")
    start = next(i for i, ln in enumerate(lines) if re.match(r"^##\s+2\.", ln))
    end = next((i for i in range(start + 1, len(lines))
                if re.match(r"^##\s+\d", lines[i])), len(lines))
    out = set()
    for ln in lines[start:end]:
        if not ln.startswith("|"):
            continue
        tok = _first_cell_token(ln)
        if tok and tok != "level":
            out.add(tok)
    return out


def _intent_is_a_row(intent, path):
    """True if `intent` is the first cell of a table row in `path` (see test_rotation)."""
    pat = re.compile(r"^\|\s*`?" + re.escape(intent) + r"`?\s*\|")
    with open(path, encoding="utf-8") as fh:
        return any(pat.match(line) for line in fh)


# --------------------------------------------------------------------------
# Branch (2): T-PTZ-1 homing
# --------------------------------------------------------------------------
def test_preset_effective_null_refuses_boot():
    """11 S7.4.8 / 10 S5.4.2 R-3: null preset_effective 拒绝启动, naming the key.

    The mutant that coerces null to False to "let it boot" makes this red -- and
    that mutation is the fail-silent 10 S5.4.2 R-3 exists to forbid.
    """
    with pytest.raises(XbrainError) as ei:
        require_preset_effective(None)
    assert ei.value.code == errors.E_CONFIG_INVALID
    # The key path must be reported so the operator knows what to go decide.
    assert ei.value.detail.get("key") == PRESET_EFFECTIVE_KEY


def test_preset_effective_missing_refuses_boot():
    """An absent key is refused too -- MISSING, not None, but the same verdict.

    Kept distinct from the null case because the config layer keeps the two apart
    (MISSING vs None); both must refuse, and this proves the guard covers absent.
    """
    with pytest.raises(XbrainError) as ei:
        require_preset_effective(MISSING)
    assert ei.value.code == errors.E_CONFIG_INVALID


def test_preset_effective_bool_passes_through():
    """A decided bool is returned unchanged -- true and false both boot.

    Positive control: without it, a guard that raised on everything would pass the
    two refusal tests above. false is the fail-safe posture (homing unavailable),
    true is the hoped-for one; neither is invented, both come from config.
    """
    assert require_preset_effective(True) is True
    assert require_preset_effective(False) is False


def test_preset_effective_non_bool_refuses():
    """A string or number in the slot is a corrupt config, not a decision.

    isinstance(1, bool) is False, so an int 1 -- which would == True -- is refused
    rather than laundered into True.
    """
    for bad in ("true", 1, 1.0, 0):
        with pytest.raises(XbrainError):
            require_preset_effective(bad)


def test_preset_intents_rejected_with_capability():
    """INF-DB-3 (2): E02/E03/E04 一律 rejected + E_CAPABILITY, reason preset_ineffective."""
    for intent in sorted(PTZ_PRESET_INTENTS):
        result = ptz_preset_failsafe(intent)
        assert result.status == STATUS_REJECTED
        assert result.code == errors.E_CAPABILITY
        assert result.detail.get("reason") == "preset_ineffective"   # 11 S7.4 表


def test_preset_never_returns_accepted():
    """*** The "accepted 冒充已到位" mutant anchor (21 S1).

    Every homing intent must be rejected and must NOT be accepted. The mutant that
    swaps the status makes both halves red.
    """
    for intent in sorted(PTZ_PRESET_INTENTS):
        result = ptz_preset_failsafe(intent)
        assert result.status == STATUS_REJECTED
        assert result.status != STATUS_ACCEPTED


def test_preset_rejects_non_preset_intent():
    """A mis-routed intent raises rather than silently rejecting some other command."""
    with pytest.raises(ValueError):
        ptz_preset_failsafe("E09")   # a speed intent, not homing


def test_preset_intents_match_contract():
    """Declared coverage equals the criterion's E02/E03/E04, and each is real in 18-B."""
    assert PTZ_PRESET_INTENTS == {"E02", "E03", "E04"}
    for intent in PTZ_PRESET_INTENTS:
        assert _intent_is_a_row(intent, VOL18B), (
            f"{intent} is not a row in 18-B -- citation stale or intent renamed"
        )


# --------------------------------------------------------------------------
# Branch (3): T-PTZ-3 speed (E09), relative turn (E10), zoom query (G33)
# --------------------------------------------------------------------------
def test_e09_gear_wire_is_level_only():
    """*** The "度每秒当量" mutant anchor (18-B S2 / 21 S1).

    Every gear serialises to exactly {"level": <gear>}. A second key -- deg_per_s,
    omega, anything numeric -- is the fabricated T-PTZ-3 curve, and it makes the
    key-set assertion red.
    """
    for level in sorted(PTZ_SPEED_LEVELS):
        wire = resolve_ptz_speed_gear(level).to_wire()
        assert set(wire) == {"level"}, (
            f"E09 gear {level} must carry only its name, no degree/second value"
        )
        assert wire["level"] == level


def test_e09_out_of_set_gear_raises():
    """闭集外必抛 (11 S13.6): a percentage or an unknown gear is rejected, not degraded.

    "60" is the 0-100 form 18-B S2 explicitly rules out; "medium" is a plausible-
    looking near-miss for the real gear names. Both must raise, never snap to a
    nearest gear.
    """
    for bad in ("60", "medium", "", "SLOW"):
        with pytest.raises(ClosedSetViolation):
            parse_ptz_speed_level(bad)
        with pytest.raises(ClosedSetViolation):
            resolve_ptz_speed_gear(bad)


def test_ptz_speed_levels_match_18b_section2():
    """The E09 closed set equals 18-B S2, both directions.

    Symmetric difference, not containment -- the same shape test_closed_sets uses:
    one-directional containment would stay green while the library dropped a gear
    the contract still lists. No count is asserted (CLAUDE.md 3.7); the sets are
    compared as sets.
    """
    doc = _parse_e09_levels()
    assert doc, "parsed no E09 gears from 18-B S2 -- parser or document changed"
    only_doc = doc - set(PTZ_SPEED_LEVELS)
    only_lib = set(PTZ_SPEED_LEVELS) - doc
    assert not only_doc, f"18-B S2 lists gears the library is missing: {sorted(only_doc)}"
    assert not only_lib, f"library exports gears 18-B S2 does not: {sorted(only_lib)}"


def test_e10_always_rejected_and_guides_to_e01():
    """INF-DB-3 (3): E10 ptz_move_deg 一律 rejected + 引导改用 E01 三档."""
    result = ptz_move_deg_failsafe()
    assert result.status == STATUS_REJECTED
    assert result.code == errors.E_CAPABILITY
    assert result.guidance == GUIDE_USE_E01
    assert result.detail.get("reason") == "ptz_speed_uncalibrated"


def test_e10_carries_no_motion_estimate():
    """18-B hard-constraints 2 and 3: no guessed omega, no estimated angle.

    The reject must not smuggle a pulse_ms or an angle into its detail; doing so
    would be "拿猜的 omega 先跑" / writing an estimate as a position. A mutant that
    returns accepted with a computed pulse trips both this and the test above.
    """
    detail = ptz_move_deg_failsafe().detail
    for forbidden in ("pulse_ms", "angle_deg", "omega", "deg_per_s"):
        assert forbidden not in detail, (
            f"E10 reject must not carry {forbidden!r} -- omega is null (T-PTZ-3)"
        )


def test_g33_answer_is_gear_only():
    """INF-DB-3 (3): G33 只答档位, no magnification or angle.

    The answer serialises to exactly {"gear": <label>}. Folding MaxZoom (3300)
    into "33 倍" would add a magnification key and make this red (18 S9.7, T-PTZ-4).
    """
    wire = query_ptz_zoom_gear("gear_2").to_wire()
    assert set(wire) == {"gear"}, "G33 must answer a gear only, never a magnification"
    assert wire["gear"] == "gear_2"


def test_g33_rejects_a_numeric_gear():
    """A raw ratio must not be passed as the gear.

    query_ptz_zoom_gear(33.0) is the shape of "someone already folded 3300 into
    33x"; refusing a non-string gear keeps that number from riding through as a
    label.
    """
    with pytest.raises(ValueError):
        query_ptz_zoom_gear(33.0)
