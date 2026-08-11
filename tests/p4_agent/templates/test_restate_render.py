"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_restate_render.py
Brief: GWY-P4-35 (32.C) -- render_restate + RS-1/RS-2/RS-4 over the yaml

Description:
Tests the restate executor against the real configs/restate_templates.yaml
(16 S8.3 / S8.3B). Each criterion carries a mutation that must turn red
per CLAUDE.md 3.3: RS-1 (l1b numeric carries the request word), RS-4
(applied != requested appends a correction), RS-2 (action first).
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.templates.restate_engine import (
    RestateSchemaError,
    check_rs1_numeric_uses_request_word,
    check_rs2_starts_with_action,
    render_restate,
    render_rs4_correction,
    validate_restate_templates,
    _ACTION_LEADS,
    _has_numeric_slot,
)

pytestmark = pytest.mark.no_device

_TEMPLATES_PATH = "/opt/xbrain_v6/configs/restate_templates.yaml"


def _templates():
    with open(_TEMPLATES_PATH, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# -- the real file loads and passes RS-1 + RS-2 as a whole ----------------

def test_real_templates_validate():
    validate_restate_templates(_templates())   # no raise


# -- render fills and refuses leftovers -----------------------------------

def test_render_l1a_exact_fills():
    t = _templates()
    out = render_restate(t, "move_relative", "exact",
                         {"action_cn": "前进", "dist": 1, "unit_cn": "米",
                          "suffix": ""})
    assert out == "前进1米"
    assert "{" not in out


def test_render_leftover_suffix_placeholder_raises():
    """A resolved {suffix} that still carries {v_max_eff} must NOT be
    spoken. MUTATION guard: without the output leftover-scan the reply
    would be '前进1米,当前限速{v_max_eff}' read aloud."""
    t = _templates()
    with pytest.raises(RestateSchemaError):
        render_restate(t, "move_relative", "exact",
                       {"action_cn": "前进", "dist": 1, "unit_cn": "米",
                        "suffix": ",当前限速{v_max_eff}"})   # suffix half-resolved


# -- criterion 1: RS-1 l1b numeric carries the request word ---------------

def test_rs1_l1b_move_relative_carries_request_word():
    """16 S8.3 RS-1 source 3: the l1b_pre numeric line pre-announces a
    REQUEST value and must be marked as a request so the operator can tell
    it from an applied value."""
    t = _templates()
    text = t["l1b_pre"]["move_relative"]
    assert _has_numeric_slot(text)                      # it has {req_dist}
    check_rs1_numeric_uses_request_word(text, True)     # and carries 请求


def test_rs1_mutation_drop_request_word_turns_red():
    """MUTATION A: drop the request word from the l1b numeric template ->
    RS-1 must raise (else the pre-announced number is indistinguishable
    from an applied value)."""
    mutated = "即将{action_cn}{req_dist}{unit_cn}{suffix}"   # 请求 removed
    assert _has_numeric_slot(mutated)
    with pytest.raises(RestateSchemaError):
        check_rs1_numeric_uses_request_word(mutated, True)


# -- criterion 2: RS-4 applied != requested appends a correction ----------

def test_rs4_correction_on_mismatch():
    """16 S8.3 RS-4: applied != requested (clip/downgrade/lock) MUST play
    the l1b_correct line."""
    t = _templates()
    out = render_rs4_correction(
        t, "move_relative",
        {"action_cn": "前进", "applied_dist": 0.6, "unit_cn": "米",
         "limiter_cn": "前方障碍", "suffix": ""},
        requested=1.0, applied=0.6)
    assert out is not None
    assert "实际" in out and "0.6" in out


def test_rs4_mutation_silent_on_mismatch_turns_red():
    """MUTATION B: a caller that skips the correction on a mismatch (the
    RS-4 silence bug) would return None where a correction is mandatory.
    The 'is not None' assertion guards it."""
    t = _templates()
    out = render_rs4_correction(
        t, "move_relative",
        {"action_cn": "前进", "applied_dist": 0.6, "unit_cn": "米",
         "limiter_cn": "前方障碍", "suffix": ""},
        requested=1.0, applied=0.6)
    # The correct executor returns a non-None correction; the silence bug
    # (return None on mismatch) would fail here.
    assert out is not None


def test_rs4_no_correction_when_applied_equals_requested():
    t = _templates()
    out = render_rs4_correction(
        t, "move_relative",
        {"action_cn": "前进", "applied_dist": 1.0, "limiter_cn": "无限制",
         "suffix": ""},
        requested=1.0, applied=1.0)
    assert out is None   # nothing to correct


# -- criterion 3: RS-2 action first ---------------------------------------

def test_rs2_main_template_starts_with_action():
    t = _templates()
    check_rs2_starts_with_action(t["move_relative"]["exact"], _ACTION_LEADS)


def test_rs2_mutation_constraint_first_turns_red():
    """MUTATION C: put the constraint suffix before the action -> RS-2
    must raise (00 CMD-38: action first, the operator hears the keyword
    before the qualifier)."""
    mutated = "{suffix}{action_cn}{dist}{unit_cn}"   # suffix hoisted first
    with pytest.raises(RestateSchemaError):
        check_rs2_starts_with_action(mutated, _ACTION_LEADS)
