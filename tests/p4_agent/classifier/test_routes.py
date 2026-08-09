"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_routes.py
Brief: classifier tests -- routes

Description:
16 §5.3 route closed set tests.
"""


import pytest

from xbrain.p4_agent.classifier import routes


pytestmark = pytest.mark.no_device


# --- Closed set has exactly the 4 documented values ----------------

def test_route_closed_set_has_exactly_four_values():
    """16 §5.3 four-value set. Adding a fifth requires a doc change +
    a change here; a regression that shrinks or grows the set gets
    caught."""
    assert routes.ROUTES == frozenset({
        "bypass", "fastpath", "fastpath_then_llm", "llm",
    })


def test_route_constants_are_the_documented_string_values():
    """The string constants must equal what 16 §5.3 intents.yaml
    schema uses. Any drift here would produce an intents.yaml file
    that fails to load (route ∈ ROUTES check)."""
    assert routes.ROUTE_BYPASS == "bypass"
    assert routes.ROUTE_FASTPATH == "fastpath"
    assert routes.ROUTE_FASTPATH_THEN_LLM == "fastpath_then_llm"
    assert routes.ROUTE_LLM == "llm"


# --- validate_route --------------------------------------------------

@pytest.mark.parametrize("value", list(routes.ROUTES))
def test_validate_route_accepts_every_documented_value(value):
    routes.validate_route(value)


@pytest.mark.parametrize("bogus", [
    "", "MAGIC", "Fastpath", "fast_path", "LLM", "llm-1",
    None, 0, [],
])
def test_validate_route_rejects_out_of_set(bogus):
    with pytest.raises((ValueError, TypeError)):
        routes.validate_route(bogus)


# --- RouteDecision constructor -----------------------------------

def test_route_decision_rejects_out_of_set_route_at_construction():
    """Frozen dataclass with __post_init__ validator: an invalid
    instance is unconstructable. This is stronger than a runtime
    boolean gate because there's no way to bypass __post_init__."""
    with pytest.raises(ValueError):
        routes.RouteDecision(route="magic")


def test_route_decision_defaults_reason_to_route_name():
    """Empty reason gets the route= form for logs."""
    d = routes.RouteDecision(route=routes.ROUTE_LLM)
    assert d.reason == "route=llm"


def test_route_decision_preserves_explicit_reason():
    d = routes.RouteDecision(
        route=routes.ROUTE_FASTPATH,
        matched_intent="A05",
        reason="matched A05 move_forward",
    )
    assert d.reason == "matched A05 move_forward"


def test_route_decision_is_frozen():
    """dataclass(frozen=True) invariant -- caller cannot mutate
    a decision after construction."""
    d = routes.RouteDecision(route=routes.ROUTE_BYPASS)
    with pytest.raises(Exception):   # FrozenInstanceError inherits from AttributeError
        d.route = "llm"   # type: ignore[misc]
