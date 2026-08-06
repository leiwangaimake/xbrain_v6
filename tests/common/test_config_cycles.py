"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_cycles.py
Brief: Three-colour DFS cycle detection, and the exact report shape 10 S5.4.3 wants

Description:
CFG-CM-9. The contract does not merely require that cycles be detected; it
requires that the error print the whole loop, and it gives the literal example
this suite asserts against character for character.

That distinction is the point of the item. A loader that says "reference cycle
detected" and stops has technically detected it, and has left the operator with a
tree of hundreds of keys and nowhere to start. The same reasoning is behind
startup assertion J, which insists on absolute paths rather than "a file was
missing".

Each case names the mutation that turns it red -- CLAUDE.md 3.3: an assertion
that has never been red has not been written.
"""

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config import cycles, refs, unflatten  # noqa: E402
from xbrain.common.config.cycles import CycleError  # noqa: E402

# The literal example from 10 S5.4.3. Kept as one string so a change to the
# rendering shows up as a diff of the whole block rather than a subtle
# indentation drift nobody notices.
CONTRACT_EXAMPLE = (
    "E_CONFIG_INVALID: reference cycle\n"
    "  common.recording.fence_close_tol_m\n"
    "    -> common.recording.min_dist_m\n"
    "    -> common.recording.fence_close_tol_m"
)


def two_key_cycle():
    """The exact tree from the contract's worked example."""
    return unflatten({
        "common.recording.fence_close_tol_m": "${common.recording.min_dist_m}",
        "common.recording.min_dist_m": "${common.recording.fence_close_tol_m}",
    })


# -- the report shape, which is the whole reason this item exists --------------

def test_report_matches_the_contract_example_character_for_character():
    """*** The core assertion of CFG-CM-9.

    Mutation: shorten the message to "reference cycle detected" => red. That
    variant detects the same defect and tells the operator nothing about where
    it is.
    """
    with pytest.raises(CycleError) as exc:
        cycles.detect(two_key_cycle())
    assert str(exc.value).endswith(CONTRACT_EXAMPLE), (
        "the printed cycle must match 10 S5.4.3 exactly:\n"
        + CONTRACT_EXAMPLE + "\ngot:\n" + str(exc.value)
    )


def test_the_path_closes_on_itself():
    """First and last entry are the same key.

    Without the repeat the reader has to work out for themselves that the last
    hop returns to the start, which is precisely the work the format exists to
    save. Mutation: drop the trailing repeat => red.
    """
    found = cycles.find_cycles(two_key_cycle())
    assert found, "a two-key loop must be found"
    assert found[0][0] == found[0][-1]


def test_cycle_is_also_available_as_data():
    """The path rides on the exception, not only inside the formatted text.

    A caller wanting to report it another way should not have to parse the
    message back apart -- and a parser of your own error text is a thing that
    silently breaks the next time the wording changes.
    """
    with pytest.raises(CycleError) as exc:
        cycles.detect(two_key_cycle())
    assert exc.value.path[0] == exc.value.path[-1]
    assert "common.recording.min_dist_m" in exc.value.path


# -- detection itself ----------------------------------------------------------

def test_self_reference_is_a_cycle():
    """A one-key loop, which is the degenerate case the walk must not miss.

    It is worth its own case because the slicing that extracts a cycle from the
    stack uses stack.index(node); for a self-reference that index is the last
    element rather than an earlier one, so an implementation that assumed the
    repeated node always sits strictly before the top would slice an empty path
    and report nothing.
    """
    with pytest.raises(CycleError):
        cycles.detect(unflatten({"common.a": "${common.a}"}))


def test_three_key_cycle_is_found_and_printed_whole():
    """Longer loops print every member, not just the two ends.

    A report naming only where the loop was entered and where it closed would
    still satisfy the two-key case above, because there the two ends are the
    whole loop. Three keys is the shortest tree that tells them apart.
    """
    tree = unflatten({"common.a": "${common.b}", "common.b": "${common.c}",
                      "common.c": "${common.a}"})
    with pytest.raises(CycleError) as exc:
        cycles.detect(tree)
    text = str(exc.value)
    for key in ("common.a", "common.b", "common.c"):
        assert key in text, "every member of the loop must appear"


def test_a_cycle_reachable_from_several_aliases_is_reported_once():
    """One report per loop, however many aliases lead into it.

    What makes this hold is the BLACK marking, not a de-duplication pass: a
    second alias reaching an explored loop meets a BLACK node rather than a GREY
    one. An earlier draft also carried an explicit dedup step; a mutation test
    showed disabling it changed nothing, so it was dead code and was removed.

    Mutation that DOES turn this red: stop marking the stack BLACK on the way
    out => every entry point rediscovers the same loop, and one defect looks
    like three.
    """
    tree = unflatten({
        "common.loop_a": "${common.loop_b}",
        "common.loop_b": "${common.loop_a}",
        "common.into_1": "${common.loop_a}",
        "common.into_2": "${common.loop_b}",
    })
    assert len(cycles.find_cycles(tree)) == 1


def test_acyclic_alias_chain_is_not_reported():
    """Pairs with every case above.

    A detector that raised on any alias at all would satisfy all the negative
    tests here and make every legal configuration unloadable. Both directions
    are needed -- CLAUDE.md 3.2 form 1.
    """
    tree = unflatten({"common.a": 5, "common.b": "${common.a}",
                      "common.c": "${common.b}"})
    assert cycles.find_cycles(tree) == []
    cycles.detect(tree)  # must not raise


def test_plain_values_do_not_enter_the_graph():
    """Non-references produce no edges -- including a list.

    The list matters: merge.flatten treats a list as a leaf so that R-5 keeps
    whole-table replacement, and this asserts the graph builder inherits that
    and does not descend into elements. A builder that walked into lists would
    turn a string element into an edge, and the resulting "cycle" would be
    reported against a key the operator cannot find in the file.
    """
    tree = unflatten({"common.x": 1, "common.y": "not a reference",
                      "common.z": ["a", "b"]})
    assert cycles.build_graph(tree) == {}


# -- R-4 chain length ----------------------------------------------------------

def test_chain_of_exactly_three_hops_is_allowed():
    """R-4 permits up to three hops; four keys is three hops.

    This is the boundary that a naive off-by-one gets wrong in the safe-looking
    direction: rejecting a legal chain merely stops the stack, so it would be
    found immediately. Accepting a four-hop chain is the silent one.
    """
    tree = unflatten({"common.a": 1, "common.b": "${common.a}",
                      "common.c": "${common.b}", "common.d": "${common.c}"})
    assert cycles.find_overlong_chains(tree) == []
    cycles.detect(tree)


def test_chain_of_four_hops_is_rejected_and_printed_whole():
    tree = unflatten({"common.a": 1, "common.b": "${common.a}",
                      "common.c": "${common.b}", "common.d": "${common.c}",
                      "common.e": "${common.d}"})
    with pytest.raises(CycleError) as exc:
        cycles.detect(tree)
    text = str(exc.value)
    assert "R-4" in text, "the message must name the rule the reader should look up"
    assert "common.e" in text and "common.a" in text, "the whole chain must print"


def test_only_maximal_chains_are_reported():
    """Every suffix of a too-long chain is also too long.

    Listing them all would bury the one entry the operator actually has to
    shorten. Mutation: start the walk from every node => red.
    """
    # SIX keys, not five. With five, the longest suffix is itself within the
    # limit, so removing the head filter changes nothing and the case passes
    # against a broken implementation -- which is what the first version of this
    # test did. Six is the shortest chain whose suffix is also over the limit.
    tree = unflatten({"common.k0": 1,
                      "common.k1": "${common.k0}", "common.k2": "${common.k1}",
                      "common.k3": "${common.k2}", "common.k4": "${common.k3}",
                      "common.k5": "${common.k4}"})
    assert len(cycles.find_overlong_chains(tree)) == 1


def test_cycles_are_reported_before_overlong_chains():
    """A cycle is unconditionally wrong; a long chain is merely unreadable.

    When a tree has both, the cycle is what to say first -- and fixing it often
    shortens the chain anyway.
    """
    tree = unflatten({
        "common.p": "${common.q}", "common.q": "${common.p}",
        "common.a": 1, "common.b": "${common.a}", "common.c": "${common.b}",
        "common.d": "${common.c}", "common.e": "${common.d}",
    })
    with pytest.raises(CycleError) as exc:
        cycles.detect(tree)
    assert "reference cycle" in str(exc.value)


# -- boundaries: what this module must NOT do ---------------------------------

def test_malformed_references_are_left_to_the_shape_validator():
    """A default-value reference is an R-3 defect, not a cycle.

    If this module raised on it, the reader would be sent to the cycle section
    of the contract for a problem described three sections earlier. Mutation:
    treat any ${...} as an edge => red.
    """
    tree = unflatten({"common.x": "${common.missing:-2.5}"})
    assert cycles.build_graph(tree) == {}
    cycles.detect(tree)  # silent here
    with pytest.raises(Exception) as exc:
        refs.resolve(tree)  # and refs names R-3
    assert "R-3" in str(exc.value)


def test_resolve_reports_a_cycle_between_three_keys_too():
    """Integration, beyond the worked example.

    The two-key case is the one the contract illustrates, and a formatter
    special-cased to two keys would pass it. This asserts the same path through
    resolve for a loop the example does not cover.
    """
    tree = unflatten({"common.a": "${common.b}", "common.b": "${common.c}",
                      "common.c": "${common.a}"})
    with pytest.raises(CycleError) as exc:
        refs.resolve(tree)
    assert str(exc.value).count("->") == 3, "three hops must print as three arrows"


def test_l6_cannot_form_a_cycle_structurally():
    """10 S5.4.3 states the reason detection only scans common.*.

    R-2 makes L6 references one-way, so a process key can point into common.*
    but nothing in common.* can point back. This asserts the graph builder does
    not pick up a non-common key even when one references another.
    """
    tree = unflatten({"p1_motion.a": "${p2_core.b}", "p2_core.b": "${p1_motion.a}"})
    assert cycles.build_graph(tree) == {}


# -- integration with the reference axis ---------------------------------------

def test_resolve_reports_the_cycle_not_a_missing_key():
    """resolve must run detection before expanding.

    Mutation: drop the cycles.detect call from refs.resolve => the loop is then
    caught by the recursion guard deep inside lookup, whose message can only
    name the walk it happened to take. This case asserts the message is the
    contract's cycle report.
    """
    with pytest.raises(CycleError) as exc:
        refs.resolve(two_key_cycle())
    assert str(exc.value).endswith(CONTRACT_EXAMPLE)


def test_resolve_still_works_on_a_legal_alias_chain():
    tree = unflatten({"common.a": 7, "common.b": "${common.a}"})
    assert refs.resolve(tree)["common"]["b"] == 7
