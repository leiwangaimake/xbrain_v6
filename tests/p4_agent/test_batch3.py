"""GWY-P4-07/08/09/10 batch 3 tests."""

import pytest

from xbrain.p4_agent.prompt.assembler import (
    PromptLayers, PromptSchemaError,
    assemble, check_history_enable_on, trim_to_budget,
)
from xbrain.p4_agent.registry.cmdset_extractor import (
    build_cmdset_json, extract_rows,
)
from xbrain.p4_agent.registry.intents_check import (
    IntentsSchemaError, MI1_MOTION_INTENTS,
    check_all, check_id1_required_fields, check_id2_geo_ids,
    check_id3_no_direction_on_mi1,
)
from xbrain.p4_agent.registry.startup_assertions import (
    CsAssertionError, check_cs_a1, check_cs_a2, check_cs_a3, check_cs_a4,
)


pytestmark = pytest.mark.no_device


# --- P4-07 ID-1/2/3 ---

def test_id1_missing_route_raises():
    with pytest.raises(IntentsSchemaError) as ei:
        check_id1_required_fields("move_forward",
                                    {"id": "A05", "auth": "L1a", "slots": []})
    assert "route" in str(ei.value)


def test_id1_bad_route_raises():
    with pytest.raises(IntentsSchemaError):
        check_id1_required_fields("x", {
            "id": "A05", "route": "magic", "auth": "L1a", "slots": [],
        })


def test_id1_bad_auth_raises():
    with pytest.raises(IntentsSchemaError):
        check_id1_required_fields("x", {
            "id": "A05", "route": "fastpath", "auth": "L4", "slots": [],
        })


def test_id2_geo_id_pattern():
    check_id2_geo_ids("goto", ["r-east", "w-p03", "f-fence1"])
    with pytest.raises(IntentsSchemaError):
        check_id2_geo_ids("goto", ["route_1"])
    with pytest.raises(IntentsSchemaError):
        check_id2_geo_ids("goto", ["route_east_gate"])


def test_id3_direction_on_mi1_rejected():
    """VARIANT (spec): reintroducing the deleted relative_move
    with direction slot -> refuse."""
    for intent in MI1_MOTION_INTENTS:
        with pytest.raises(IntentsSchemaError):
            check_id3_no_direction_on_mi1(intent, ["direction", "amount", "unit"])


def test_id3_direction_ok_on_ptz_intents():
    """E01 ptz_move / E06 ptz_zoom legitimately have direction."""
    check_id3_no_direction_on_mi1("ptz_move", ["direction", "amount"])
    check_id3_no_direction_on_mi1("ptz_zoom", ["direction"])


def test_check_all_flags_first_bad_intent():
    reg = {
        "move_forward": {"id": "A05", "route": "fastpath",
                          "auth": "L1a",
                          "slots": ["direction"]},   # bad -- MI1 + direction
    }
    with pytest.raises(IntentsSchemaError):
        check_all(reg)


# --- P4-08 CS-A* ---

def test_cs_a1_extras_raise():
    cs = frozenset({"move_forward", "stop"})
    with pytest.raises(CsAssertionError) as ei:
        check_cs_a1(["move_forward", "stop", "invented"], cs)
    assert "invented" in str(ei.value)


def test_cs_a2_count_mismatch_raises():
    with pytest.raises(CsAssertionError):
        check_cs_a2(intents_yaml_count=100, cmdset_json_count=128)


def test_cs_a3_returns_dropped_in_transitional_mode():
    """CS-A3: instead of raising, returns dropped list (warn mode)."""
    dropped = check_cs_a3(
        mission_alternation=["move_forward", "ghost_intent"],
        cmdset_closed_set=frozenset({"move_forward"}),
    )
    assert dropped == ["ghost_intent"]


def test_cs_a4_alternation_over_limit_raises():
    """Non-M4 mission with 5 intents + unknown = 6 > 5 -> raise."""
    with pytest.raises(CsAssertionError):
        check_cs_a4("M3_nav", alternation_size=5)


def test_cs_a4_m4_follow_allowed_up_to_6():
    """M4_follow break: 5 intents + unknown = 6 is OK (limit = 6)."""
    check_cs_a4("M4_follow", alternation_size=5)


# --- P4-09 cmdset extractor ---

def test_extract_rows_matches_shape():
    md = (
        "| A05 | move_forward | fastpath | L1a |\n"
        "| E01 | ptz_move    | fastpath | L1a |\n"
        "| H08 | shutdown    | llm      | L3  |\n"
    )
    rows = extract_rows(md)
    assert len(rows) == 3
    assert rows[0] == {"id": "A05", "intent": "move_forward",
                        "route": "fastpath", "auth": "L1a"}


def test_extract_rows_ignores_non_matching_lines():
    md = "# Header\n\n| bad | shape |\n| A05 | move_forward | fastpath | L1a |\n"
    rows = extract_rows(md)
    assert len(rows) == 1


def test_build_cmdset_json_wraps_with_version():
    md = "| A05 | move_forward | fastpath | L1a |\n"
    doc = build_cmdset_json(md)
    assert doc["version"] == 1
    assert len(doc["intents"]) == 1


# --- P4-10 prompt assembler ---

def test_history_enable_on_valid_values():
    check_history_enable_on([])
    check_history_enable_on(["clarify"])
    check_history_enable_on(["clarify", "recent"])


def test_history_enable_on_unknown_raises():
    with pytest.raises(PromptSchemaError):
        check_history_enable_on(["magic"])
    with pytest.raises(PromptSchemaError):
        check_history_enable_on("not_a_list")


def test_trim_pops_history_first():
    l = PromptLayers(system="S", mission="M", few_shots=["F1"],
                      history=["H1", "H2"])
    # Budget forces history to shrink.
    trimmed = trim_to_budget(l, char_budget=len("S") + len("M") + len("F1"))
    assert trimmed.history == []
    # few_shots retained.
    assert trimmed.few_shots == ["F1"]


def test_trim_never_touches_system():
    l = PromptLayers(system="S" * 100, mission="", few_shots=[],
                      history=[])
    trimmed = trim_to_budget(l, char_budget=10)
    # system still full (only trim step 4 would touch mission; not system).
    assert trimmed.system == "S" * 100


def test_assemble_concatenates_layers():
    l = PromptLayers(system="SYS", mission="MISSION",
                      few_shots=["S1"], history=["H1"])
    out = assemble(l)
    assert "SYS" in out
    assert "MISSION" in out
    assert "S1" in out
    assert "H1" in out
