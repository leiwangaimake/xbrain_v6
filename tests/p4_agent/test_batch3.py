"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch3.py
Brief: GWY-P4-08/09/10 prompt assembler + cmdset extractor + CS-A1..A4 checks

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的 "p4_agent tests -- batch3" --
既没说清本文件测什么, 也无法据以索引任务号(CLAUDE.md 2.5). 同一处理已对
P2 做过; 补 Brief 而不是绕过去, 是因为绕过去只会让下一个人再读一遍.
GWY-P4-07/08/09/10 batch 3 tests.
"""


import pytest

from xbrain.p4_agent.prompt.assembler import (
    PromptLayers,
    PromptSchemaError,
    assemble,
    check_history_enable_on,
    trim_to_budget,
)
from xbrain.p4_agent.registry.cmdset_extractor import (
    build_cmdset_json,
    extract_rows,
)

pytestmark = pytest.mark.no_device


# GWY-P4-07 (ID-1/ID-2/ID-3) and GWY-P4-08 (CS-A1..CS-A4) used to be
# exercised here against xbrain/p4_agent/registry/intents_check.py and
# startup_assertions.py. Both implementations were deleted on 2026-09-28
# as weaker second opinions about rules that the WIRED loader already
# enforces, so the tests went with them:
#   ID-1 / closed sets / ID-3 -> registry/intents.py, covered (with
#       mutation tests) by tests/p4_agent/registry/test_intents.py;
#   ID-2                      -> registry/geo_id.py, covered by
#       tests/p4_agent/registry/test_geo_id.py;
#   CS-A1 + CS-A2             -> intents.check_intents_in_closed_set,
#       which is BIDIRECTIONAL (the deleted check_cs_a1 was one-way and
#       would pass a name 18 has that the registry dropped);
#   CS-A3 / CS-A4             -> registry/missions.py, which raises at
#       load_missions() and is called from p4_agent/__main__.py.
# The one CFG-BT-19 rule that had NO other implementation, the
# trigger-word conflict check, is now wired into load_intent_registry and
# is tested at tests/p4_agent/registry/test_intents.py.


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
    layers = PromptLayers(system="S", mission="M", few_shots=["F1"],
                      history=["H1", "H2"])
    # Budget forces history to shrink.
    trimmed = trim_to_budget(layers, char_budget=len("S") + len("M") + len("F1"))
    assert trimmed.history == []
    # few_shots retained.
    assert trimmed.few_shots == ["F1"]


def test_trim_never_touches_system():
    layers = PromptLayers(system="S" * 100, mission="", few_shots=[],
                      history=[])
    trimmed = trim_to_budget(layers, char_budget=10)
    # system still full (only trim step 4 would touch mission; not system).
    assert trimmed.system == "S" * 100


def test_assemble_concatenates_layers():
    layers = PromptLayers(system="SYS", mission="MISSION",
                      few_shots=["S1"], history=["H1"])
    out = assemble(layers)
    assert "SYS" in out
    assert "MISSION" in out
    assert "S1" in out
    assert "H1" in out
