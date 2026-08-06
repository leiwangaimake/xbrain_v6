"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_id.py
Brief: GWY-P4-07 ID-2 -- the geographic-object id shape, with its red mutation

Description:
ID-2 (16 S5.3) says every geographic-object id is <prefix>-<slug>. This file
proves the validator both accepts the legal shape and REJECTS the v0.1 route_N
form the criterion names (route_1 / route_3 / route_east_gate), and that the
rejection is not an accident of one example: it holds for every prefix and
fails for the specific near-miss that shares a first letter with a real prefix.

CLAUDE.md 3.3: the criterion is only met if a mutation makes it red. Here the
mutation is baked into the corpus -- route_1 IS the mutant of r-1, differing by
exactly the hyphen ID-2 turns on -- so test_route_n_form_is_rejected is the red
case, and it would pass (wrongly) only if the validator dropped the hyphen
requirement.
"""

import os
import sys

# ROOT is four levels up: tests/p4_agent/registry/<this>. Inserted so
# `import xbrain...` resolves without an installed package, the same bootstrap
# tests/p4_agent/test_config_loader.py uses.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

import pytest  # noqa: E402

from xbrain.p4_agent.registry import (GEO_ID_PREFIXES, GeoIdError,  # noqa: E402
                                      is_valid_geo_object_id,
                                      validate_geo_object_id)

# The exact strings the GWY-P4-07 criterion 2 names as must-reject. They are the
# v0.1 route_N format 16 S7.0.2 uses as its worked example of an I3 defect.
CRITERION_REJECTS = ("route_1", "route_3", "route_east_gate", "route_9")


@pytest.mark.parametrize("bad", CRITERION_REJECTS)
def test_route_n_form_is_rejected(bad):
    """criterion 2 / the red mutation: route_N is not a legal geo id.

    Both surfaces must agree: the predicate says False and the boundary form
    raises. If someone relaxed the shape to 'first letter then anything', these
    would pass the predicate -- route_1 begins with r -- which is the precise
    mistake ID-2 exists to stop, so this is the case that catches it.
    """
    assert is_valid_geo_object_id(bad) is False
    with pytest.raises(GeoIdError):
        validate_geo_object_id(bad)


@pytest.mark.parametrize("prefix", GEO_ID_PREFIXES)
def test_every_prefix_accepts_a_well_formed_slug(prefix):
    """Each of the six prefixes accepts a <prefix>-<slug>.

    Parametrised over the prefix set itself rather than a hand-listed sample, so
    adding or removing a prefix in one place (geo_id.GEO_ID_PREFIXES) cannot
    leave this test asserting about a prefix that no longer exists, or silently
    not covering a new one.
    """
    good = "%s-main_loop_01" % prefix
    assert is_valid_geo_object_id(good) is True
    # The boundary form returns the input unchanged -- it validates, it does not
    # normalise (no case-fold, no trim), matching enums.ClosedSet.parse.
    assert validate_geo_object_id(good) == good


def test_slug_charset_is_lowercase_digits_underscore():
    """The slug is [a-z0-9_]+: uppercase and stray punctuation are out.

    Uppercase is excluded so r-Main and r-main cannot name what a human reads as
    one place while the store treats them as two -- the ids are machine slugs.
    """
    assert is_valid_geo_object_id("w-a1_b2") is True
    assert is_valid_geo_object_id("w-Main") is False      # uppercase
    assert is_valid_geo_object_id("w-main-2") is False     # second hyphen
    assert is_valid_geo_object_id("w-") is False           # empty slug
    assert is_valid_geo_object_id("-main") is False        # no prefix
    assert is_valid_geo_object_id("x-main") is False       # prefix not in set


def test_non_string_is_false_not_an_exception():
    """The predicate answers for odd inputs rather than raising on them.

    A few-shot linter may hand it a number or None from a half-parsed example;
    the question 'is this a geo id' has a defensible answer (no), and re.match on
    a non-str would otherwise raise TypeError inside a function named is_valid_*.
    The boundary form still raises, because there the caller wanted a valid id.
    """
    assert is_valid_geo_object_id(None) is False
    assert is_valid_geo_object_id(7) is False
    assert is_valid_geo_object_id(["r-x"]) is False
    with pytest.raises(GeoIdError):
        validate_geo_object_id(None)  # type: ignore[arg-type]


def test_geo_id_error_carries_closed_set_code():
    """GeoIdError reports E_SCHEMA from the shared library, never a literal.

    A malformed id on a boundary is a schema violation of the slot that should
    have held a <prefix>-<slug> (11 S13.6 forbids passing it through). The code
    is imported, so this also guards against a future literal creeping in.
    """
    from xbrain.common.errors import E_SCHEMA
    err = GeoIdError("route_1")
    assert err.code == E_SCHEMA
