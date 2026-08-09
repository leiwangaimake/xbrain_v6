"""MOT-CM-1 (PRC-69/PRC-70) permissive cls parser tests."""

import pytest

from xbrain.common.enums import CLS, parse_enum, ClosedSetViolation
from xbrain.common.enums.cls_permissive import (
    _reset_for_tests, off_set_count, parse_cls_permissive,
    seen_off_set_names, set_event_emitter,
)


@pytest.fixture(autouse=True)
def _clean():
    _reset_for_tests()
    set_event_emitter(None)
    yield
    _reset_for_tests()
    set_event_emitter(None)


def test_on_set_value_passes_through():
    for v in ("person", "vehicle", "bicycle", "motorcycle", "animal", "unknown"):
        assert parse_cls_permissive(v) == v


def test_off_set_maps_to_unknown():
    assert parse_cls_permissive("cat") == "unknown"


def test_off_set_first_sight_emits_event():
    events = []
    set_event_emitter(lambda sev, kind, detail: events.append((sev, kind, detail)))
    parse_cls_permissive("dog")
    assert len(events) == 1
    assert events[0][0] == "info"
    assert events[0][1] == "perception.cls_off_set"
    assert events[0][2]["name"] == "dog"


def test_repeated_off_set_dedups_events():
    events = []
    set_event_emitter(lambda sev, kind, detail: events.append(detail["name"]))
    for _ in range(5):
        parse_cls_permissive("dog")
    assert events == ["dog"]  # only first emit


def test_multiple_distinct_off_set_each_emits():
    events = []
    set_event_emitter(lambda sev, kind, detail: events.append(detail["name"]))
    parse_cls_permissive("dog")
    parse_cls_permissive("cat")
    parse_cls_permissive("dog")  # dedup
    parse_cls_permissive("cat")  # dedup
    assert sorted(events) == ["cat", "dog"]


def test_count_reflects_all_hits_not_distinct():
    for _ in range(10):
        parse_cls_permissive("dog")
    assert off_set_count() == 10
    assert seen_off_set_names() == frozenset({"dog"})


def test_case_normalization_for_dedup():
    events = []
    set_event_emitter(lambda sev, kind, detail: events.append(detail["name"]))
    parse_cls_permissive("Cat")
    parse_cls_permissive("cat")  # same name after lower
    parse_cls_permissive("CAT")  # same name after lower
    # Only first-sight fires. Original casing preserved in the event.
    assert len(events) == 1
    assert events[0] == "Cat"


def test_non_str_raises_type_error():
    with pytest.raises(TypeError):
        parse_cls_permissive(42)
    with pytest.raises(TypeError):
        parse_cls_permissive(None)


# The critical MOT-CM-1 mutation: permissive semantics must NOT be
# applied to other closed sets. Copying this behaviour to gate_limiter
# would map an off-contract limiter name to "unknown" silently -- a
# contract violation.
def test_gate_limiter_still_raises_on_off_set():
    """MOT-CM-1 verbatim: apply cls's leniency to limiter -> red.
    Here we verify the OTHER direction: gate_limiter parse still
    raises. A test that expected leniency there would have
    documented the bug that MOT-CM-1 warns against."""
    with pytest.raises(ClosedSetViolation):
        parse_enum("gate_limiter", "made_up")


def test_strict_cls_parse_still_raises_off_set():
    """parse_enum('cls', v) is the strict entry point: PRC-2 says
    off-set cls in received data must NOT be silently normalized."""
    with pytest.raises(ClosedSetViolation):
        parse_enum("cls", "dog")


def test_permissive_and_strict_share_the_on_set_answer():
    """For any in-set value, both parsers return the same thing."""
    for v in sorted(CLS.values):
        assert parse_cls_permissive(v) == parse_enum("cls", v)


def test_emitter_none_falls_back_to_print(capsys):
    parse_cls_permissive("dog")
    out = capsys.readouterr().out
    assert "'dog'" in out
    assert "unknown" in out


def test_reset_helper_clears_state():
    parse_cls_permissive("dog")
    assert off_set_count() == 1
    _reset_for_tests()
    assert off_set_count() == 0
    assert seen_off_set_names() == frozenset()
