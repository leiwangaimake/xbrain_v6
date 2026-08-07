"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_missions.py
Brief: GWY-P4-11 -- 11 mission prompts load, each emission set binds to the 16
       S6.7 table, the M8/G11 gap stays registered; mutants for each rule

Description:
The prompts are verbatim first-fence extracts of 16 S6.7.1~S6.7.10; the loader
is the load-time assertion GWY-P4-11 (2) names (emitted closed set + 1 <= 5,
M4_follow = 6). Mutations run on temp copies of the prompt tree so the real
configs are never touched.
"""

import os
import shutil

import pytest
import yaml

from xbrain.p4_agent.registry.missions import (
    EXPECTED_EMISSIONS, KNOWN_GAPS, MISSIONS, MissionError, emitted_intents,
    load_missions,
)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
MISSIONS_DIR = os.path.join(ROOT, "configs", "prompts", "missions")


def registry_names():
    """The intent closed set, from the committed intents.yaml."""
    with open(os.path.join(ROOT, "configs", "intents.yaml"), encoding="utf-8") as fh:
        return list(yaml.safe_load(fh)["intents"])


@pytest.fixture()
def tree(tmp_path):
    """A private copy of the mission dir, for mutations."""
    dst = tmp_path / "missions"
    shutil.copytree(MISSIONS_DIR, dst)
    return str(dst)


def _mutate(tree_dir, mission, fn):
    """Rewrite one mission file through fn(text)."""
    path = os.path.join(tree_dir, mission + ".txt")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(fn(text))


# --------------------------------------------------------------------------
# the committed tree
# --------------------------------------------------------------------------

def test_real_tree_loads_all_eleven():
    """The committed prompts load clean and cover all 11 groups."""
    out = load_missions(MISSIONS_DIR, registry_names())
    assert set(out) == set(MISSIONS)
    assert all(text.strip() for text in out.values())


def test_expected_sets_respect_the_criterion_cap():
    """*** The criterion on the TABLE itself: every business mission's expected
    set + 1 (unknown) <= 5; the single exception M4_follow = 6. A growth of the
    doc table past the budget fails here even before any file is read."""
    for mission, expected in EXPECTED_EMISSIONS.items():
        if expected is None:
            continue
        cap = 5 if mission == "M4_follow" else 4
        assert len(expected) <= cap, mission


def test_m8_g11_gap_is_still_registered():
    """*** The registered doc-vs-prompt gap (16 S6.7 table lists G11 under M8;
    the prompt folds periods into query_events_recent and never emits it).

    Two directions guarded: the gap entry exists, AND the prompt still does not
    teach query_events_period. If 16 fixes either side, this fails and tells
    the fixer to clear KNOWN_GAPS -- the gap cannot silently rot in place.
    """
    assert "M8_events" in KNOWN_GAPS
    assert KNOWN_GAPS["M8_events"][0] == "query_events_period"
    with open(os.path.join(MISSIONS_DIR, "M8_events.txt"), encoding="utf-8") as fh:
        text = fh.read()
    assert "query_events_period" not in text
    assert "query_events_period" not in EXPECTED_EMISSIONS["M8_events"]


# --------------------------------------------------------------------------
# the measurement itself
# --------------------------------------------------------------------------

def test_slot_values_do_not_count_as_emissions():
    """*** The measured false-positives that killed the whole-word scan: M4
    uses `hold` as a behavior SLOT VALUE and M8 lists `estop` as a category
    VALUE -- both are registry intent names, neither may count as an emission."""
    names = registry_names()
    with open(os.path.join(MISSIONS_DIR, "M4_follow.txt"), encoding="utf-8") as fh:
        assert "hold" not in emitted_intents(fh.read(), names)
    with open(os.path.join(MISSIONS_DIR, "M8_events.txt"), encoding="utf-8") as fh:
        assert "estop" not in emitted_intents(fh.read(), names)


def test_arrow_taught_intents_do_count():
    """The rule-teaching shape: M3's patrol_repeat and M4's ptz_stop_track are
    taught ONLY by arrow rules -- a JSON-only measurement would drop them."""
    names = registry_names()
    with open(os.path.join(MISSIONS_DIR, "M3_nav.txt"), encoding="utf-8") as fh:
        assert "patrol_repeat" in emitted_intents(fh.read(), names)
    with open(os.path.join(MISSIONS_DIR, "M4_follow.txt"), encoding="utf-8") as fh:
        assert "ptz_stop_track" in emitted_intents(fh.read(), names)


# --------------------------------------------------------------------------
# mutations on a private tree
# --------------------------------------------------------------------------

def test_dropped_teaching_is_refused(tree):
    """*** Remove one taught intent from M2 => drift refusal naming it."""
    _mutate(tree, "M2_turn", lambda t: t.replace("turn_around", "turn_round"))
    with pytest.raises(MissionError, match="turn_around"):
        load_missions(tree, registry_names())


def test_stray_teaching_is_refused(tree):
    """*** Add an extra emission example to M5 => drift refusal naming it as
    stray -- the cap alone would still pass (3 <= 4), so equality is what
    catches a prompt quietly teaching a neighbouring group's intent."""
    _mutate(tree, "M5_speak",
            lambda t: t + '\n输出：{"intent":"set_volume","slots":{}}\n')
    with pytest.raises(MissionError, match="set_volume"):
        load_missions(tree, registry_names())


def test_missing_file_is_refused(tree):
    """Criterion 4: an absent mission file refuses with the path named."""
    os.remove(os.path.join(tree, "M7_objref.txt"))
    with pytest.raises(MissionError, match="M7_objref"):
        load_missions(tree, registry_names())


def test_empty_file_is_refused(tree):
    """An empty prompt 'loads' and routes nothing; refuse by name."""
    _mutate(tree, "M6b_mark", lambda t: "\n")
    with pytest.raises(MissionError, match="M6b_mark"):
        load_missions(tree, registry_names())


def test_m10_losing_its_candidate_contract_is_refused(tree):
    """M10's whole mechanism is the 可选 candidate line (top-K, U47f/R4);
    a rewrite that drops it must refuse."""
    _mutate(tree, "M10_fallback", lambda t: t.replace("可选", "备选"))
    with pytest.raises(MissionError, match="可选"):
        load_missions(tree, registry_names())
