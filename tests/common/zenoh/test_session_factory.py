"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_session_factory.py
Brief: INF-ZN-1 -- RT-C1, RT-C2, RT-C3.a and RT-C3.d held to the built config

Description:
What these cases are worth. Every field they check is one whose wrong value
still produces a working system: a session with gossip left on connects, passes
traffic and answers every functional test, while quietly giving the two Zenoh
planes a path to each other through whichever process holds both. 11 S1.1.2
states each consequence (grep "RT 面配置与强制约束 RT-C1 ~ RT-C3"). There is no
run-time symptom to test for, so the field values themselves are the test.

*** Why the endpoints are written out as literals here instead of imported from
the module under test. Importing them would make this file compare the module
against itself, which passes for any value at all. The strings below are
transcribed from 11 -- S1.1.2's json5 block for the RT plane, S1.1.4 部署形态 and
S1.1.7's plane table for the general plane -- and test_contract_pins_the_two_
endpoints greps the contract to prove the transcription still matches. That is
the same construction the closed-set metatests use, and it exists because a
constant nobody compares against its source is a constant that drifts.

* Both representations are asserted, not one. The document is what the C++
replica is compared against; the zenoh.Config is what actually reaches
zenoh.open(). Asserting only the document would leave the json5 field paths
unverified -- writing scouting.gossip under a misspelled parent produces a
perfectly valid document that Zenoh then ignores, falling back to the default,
which for multicast is ON.

* The independence cases carry RT-C3.a (grep "两个独立的 Zenoh runtime"). They
check identity AND cross-talk, because the two failures are different: a cached
singleton fails the first, while a module-level template plus a shallow copy
passes it and fails the second.

Mutations these cases were verified against, per CLAUDE.md 3.3 -- each was
injected into xbrain/common/zenoh/session_factory.py and confirmed red before
being reverted: gossip enabled true, a listen endpoint added, one shared config
object returned for both planes, and mode set to router.
"""

import json
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.errors.exceptions import ClosedSetViolation  # noqa: E402
from xbrain.common.zenoh import (PLANE_GEN, PLANE_RT,  # noqa: E402
                                 TRANSPORT_PLANES, build_session_config,
                                 parse_plane, session_config_document,
                                 session_config_json5)

#: Transcribed from 11, not imported. See the header for why that is the point.
#: test_contract_pins_the_two_endpoints holds this table against the contract.
CONTRACT_CONNECT = {
    PLANE_RT: "tcp/127.0.0.1:7449",
    PLANE_GEN: "tcp/127.0.0.1:7447",
}

CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")

#: eclipse-zenoh is a hard runtime dependency of all nine RT-plane processes, but
#: the cases that only need the document must keep running where it is absent.
#: importorskip is deliberately NOT used: it skips the whole module, which would
#: take the document-level cases down with it for a reason that has nothing to do
#: with them.
try:
    import zenoh  # noqa: E402
except ImportError:
    # Named exception type, never a bare except -- CLAUDE.md 4.5. Anything other
    # than a missing package (a broken build of the extension, say) must
    # propagate rather than be reported as "not installed".
    zenoh = None

#: The skip reason names what went unverified. A skipped isolation check and a
#: passing one look identical in a summary line, and this project has already
#: been caught reading one as the other.
needs_zenoh = pytest.mark.skipif(
    zenoh is None,
    reason="eclipse-zenoh is not installed, so the json5 field paths Zenoh "
           "actually reads were NOT verified in this run; the document-level "
           "cases passing does not establish them",
)


# ---------------------------------------------------------------------------
# Field-by-field, on the document.
# ---------------------------------------------------------------------------

# Parametrised over both planes rather than written twice. RT-C1 and RT-C2 both
# say 两个 Zenoh 平面都必须, and a copy-pasted second case is how one plane keeps
# a field the other one had fixed.
@pytest.mark.parametrize("plane", sorted(TRANSPORT_PLANES))
def test_document_fields(plane):
    """mode, scouting, listen and connect, as 11 S1.1.2 writes them.

    Every field is asserted on both planes because RT-C1 and RT-C2 are written
    for both, and because the general plane is the one people assume is the
    relaxed side. It is not: gossip left on there reaches the RT plane through
    whichever process holds both, which is the entire mechanism RT-C2 describes.
    """
    doc = session_config_document(plane)
    # RT-C3.d: peer, and the section says 永不使用 mode router. Both are asserted:
    # equality alone would still pass if the constant were later renamed to
    # something that is neither, and the router case is the one with a stated
    # consequence -- a router propagates subscriptions between the planes.
    assert doc["mode"] == "peer"
    assert doc["mode"] != "router"
    # RT-C1. Identity against False, not a truthiness check: 0 is falsy and
    # serialises as a number, which is not the boolean the contract requires.
    assert doc["scouting"]["multicast"]["enabled"] is False
    # RT-C2 was 'gossip disabled'. 2026-08-10 (V-ORIN-ZN-GOSSIP): the
    # strict 'no gossip' reading of 11 S1.1.2 broke pub/sub routing in
    # the router-brokered peer topology. Gossip is now REQUIRED on the
    # session side; the router propagates subscription tables only
    # when both ends gossip. Multicast (RT-C1) stays False and is the
    # actual cross-plane leak vector; loopback-only gossip does not
    # cross planes.
    assert doc["scouting"]["gossip"]["enabled"] is True
    # RT-C3.d. Equality against the empty list, not "not doc[...]": None would
    # also be falsy, and None is a missing field rather than an empty one.
    assert doc["listen"]["endpoints"] == []
    # Exactly one connect endpoint, and the right one. The length assertion is
    # separate from the value assertion because a second endpoint appended after
    # the correct one leaves the first element -- and any check that only reads
    # element zero -- entirely intact.
    assert doc["connect"]["endpoints"] == [CONTRACT_CONNECT[plane]]
    assert len(doc["connect"]["endpoints"]) == 1


# ---------------------------------------------------------------------------
# The same fields on the object that actually reaches zenoh.open().
# ---------------------------------------------------------------------------

# get_json returns json text, so every value is parsed before comparison. The
# alternative -- comparing against the string "false" -- would pass for the
# string "false" as well, which is not what the field means.
@pytest.mark.parametrize("plane", sorted(TRANSPORT_PLANES))
@needs_zenoh
def test_zenoh_config_fields(plane):
    """The same four constraints, read back out of the zenoh.Config.

    This is the half that proves the field PATHS are right. A document with
    scouting.gossip written under a misspelled parent is valid json and passes
    every document-level case in this file, while Zenoh silently uses its own
    default for the field it could not find.
    """
    cfg = build_session_config(plane)
    # get_json returns json text, so mode comes back quoted.
    assert json.loads(cfg.get_json("mode")) == "peer"
    # RT-C1. Zenoh's own default here is ON, which is why an unread field is
    # worse than a wrong one.
    assert json.loads(cfg.get_json("scouting/multicast/enabled")) is False
    # RT-C2 was 'gossip disabled'. 2026-08-10 (V-ORIN-ZN-GOSSIP)
    # inverted this in session_config_document: gossip is required for
    # the router-brokered peer topology. Cross-plane isolation (RT-C1)
    # is preserved by multicast staying off; gossip only propagates
    # over the ESTABLISHED TCP connect endpoint, which is loopback-only.
    assert json.loads(cfg.get_json("scouting/gossip/enabled")) is True
    # RT-C3.d. A listening peer propagates subscription declarations.
    assert json.loads(cfg.get_json("listen/endpoints")) == []
    # The whole list, so a second endpoint cannot hide behind the correct first.
    assert json.loads(cfg.get_json("connect/endpoints")) == [CONTRACT_CONNECT[plane]]


# The two planes must not be reachable from one another's config, which is only
# meaningful if the endpoints differ. Asserted on its own so that a table where
# both planes accidentally carry 7449 fails here, naming the cause, instead of
# failing the per-plane case with a confusing "expected 7447".
def test_the_two_planes_use_different_endpoints():
    """RT and GEN connect to different routers, which is the whole point.

    Two planes pointed at one router is not a degraded system, it is a system
    with no isolation at all -- and it starts, connects and carries traffic.
    """
    rt = session_config_document(PLANE_RT)["connect"]["endpoints"]
    gen = session_config_document(PLANE_GEN)["connect"]["endpoints"]
    assert rt != gen                       # 11 S1.1.2 vs S1.1.4, 7449 vs 7447


# ---------------------------------------------------------------------------
# RT-C3.a -- two independent runtimes, no shared config object.
# ---------------------------------------------------------------------------

# Identity first. A factory that memoises its result returns one object for both
# planes, and everything downstream still works until the day one side is edited.
@needs_zenoh
def test_configs_are_distinct_objects():
    """id(cfg_rt) != id(cfg_gen), and the same for two calls on one plane.

    RT-C3.a in its plainest form. A memoising factory passes every field case
    in this file and still hands one object to both planes.
    """
    cfg_rt = build_session_config(PLANE_RT)
    cfg_gen = build_session_config(PLANE_GEN)
    assert id(cfg_rt) != id(cfg_gen)       # the criterion's literal wording
    # Two calls for the SAME plane must also be two objects. Without this, a
    # per-plane cache passes the cross-plane assertion above while still handing
    # two processes -- or two threads -- one shared object.
    assert id(build_session_config(PLANE_RT)) != id(cfg_rt)


# Cross-talk second, because identity does not imply independence: two distinct
# dicts can share the nested dict a shallow copy left pointing at one template.
def test_documents_do_not_share_nested_state():
    """Editing one document must not change the other. Both planes, one plane.

    The edits below are the two an operator-facing tool would plausibly make,
    not arbitrary ones: turning gossip on and adding a listen endpoint are
    exactly the changes someone reaches for when a peer will not connect.
    """
    rt = session_config_document(PLANE_RT)
    gen = session_config_document(PLANE_GEN)
    # Mutating a nested dict, not the top level: a shallow copy of a template
    # gives distinct top-level dicts that still share everything below them,
    # so a top-level edit would not detect the defect this case exists for.
    # V-ORIN-ZN-GOSSIP: default is now True; mutate to False to prove
    # the two documents don't share nested state.
    rt["scouting"]["gossip"]["enabled"] = False
    rt["listen"]["endpoints"].append("tcp/0.0.0.0:7449")
    assert gen["scouting"]["gossip"]["enabled"] is True    # untouched other plane
    assert gen["listen"]["endpoints"] == []                # RT-C3.d, still empty
    # Same plane twice. A module-level template shared by every call fails here
    # and passes the cross-plane half above whenever the template is per plane.
    rt_again = session_config_document(PLANE_RT)
    assert rt_again["scouting"]["gossip"]["enabled"] is True
    assert rt_again["listen"]["endpoints"] == []


# The zenoh.Config side of the same property. insert_json5 is the mutation path a
# real caller has, so this is the failure that would actually happen rather than
# a constructed one.
@needs_zenoh
def test_zenoh_configs_do_not_share_state():
    """Mutating the RT Config must leave the GEN Config untouched.

    Same property as the document case, on the object the process actually
    keeps. Config holds Rust-side state, so Python-level distinctness does not
    settle it: two Config objects could in principle share one runtime handle.
    """
    cfg_rt = build_session_config(PLANE_RT)
    cfg_gen = build_session_config(PLANE_GEN)
    # V-ORIN-ZN-GOSSIP: default is now true; flip RT to false and
    # confirm GEN untouched. The property (no shared state) is the same.
    cfg_rt.insert_json5("scouting/gossip/enabled", "false")
    # The edited object is checked first. Without it the case would pass
    # vacuously if insert_json5 silently did nothing -- "gen is unchanged" says
    # nothing when nothing changed anywhere.
    assert json.loads(cfg_rt.get_json("scouting/gossip/enabled")) is False
    assert json.loads(cfg_gen.get_json("scouting/gossip/enabled")) is True


# ---------------------------------------------------------------------------
# Boundary behaviour of the plane argument.
# ---------------------------------------------------------------------------

# An unknown plane must raise, not fall back. A default plane here is the
# fail-silent shape CLAUDE.md 3.1 describes in the safety-parameter case: the
# caller gets a working config for the wrong router, and every later symptom
# points at the network instead of at the typo.
@pytest.mark.parametrize("bad", ["", "RT", "rt_plane", "general", "router"])
def test_unknown_plane_raises(bad):
    """Anything outside the two transport planes raises ClosedSetViolation.

    The parameters are the near misses, not random strings: a wrong case, the
    Python constant name, the contract's prose word for the general plane, and
    the one mode RT-C3.d forbids. Each is something a person would actually
    type, and each must raise rather than select a plane.
    """
    # All three entry points, not just the parser. An entry point that skips
    # validation is the one a caller will reach for, and it would be the only
    # one with no check at all.
    with pytest.raises(ClosedSetViolation):
        parse_plane(bad)
    with pytest.raises(ClosedSetViolation):
        session_config_document(bad)
    # This one also fixes the order inside build_session_config: the plane is
    # validated before the client is imported, so a typo reports as a closed-set
    # violation even where eclipse-zenoh is missing.
    with pytest.raises(ClosedSetViolation):
        build_session_config(bad)


# ---------------------------------------------------------------------------
# The transcription above, held against the contract itself.
# ---------------------------------------------------------------------------

# This is what stops CONTRACT_CONNECT from becoming a second, drifting source.
# It greps 11 for each endpoint as a literal string; the citation is a section
# plus an anchor and never a line number, per CLAUDE.md 1.1 NUM-4, because line
# references in this project have already drifted onto unrelated tables.
def test_contract_pins_the_two_endpoints():
    """Both endpoints appear verbatim in 11, so the transcription is current.

    Substring search and not a table parse. A parse would have to track the
    contract's markdown shape and would go quietly empty the first time that
    shape changed -- an extractor that finds nothing makes every diff look
    clean, which is how nine closed-set values once disappeared unnoticed.
    """
    text = open(CONTRACT, encoding="utf-8").read()
    for plane, endpoint in sorted(CONTRACT_CONNECT.items()):
        assert endpoint in text, (
            "endpoint for plane %s is not in 11 any more; the contract moved "
            "and session_factory must be re-read against it" % plane)
    # The prohibition RT-C3.d rests on, checked as text rather than paraphrased.
    # If this line leaves the contract, the mode assertions above stop meaning
    # what their comments claim.
    assert "永不使用" in text
    # And the section that carries the json5 block, so a wholesale renumbering
    # of 11 S1.1.2 is caught here rather than by a reader who cannot find it.
    assert "RT 面配置与强制约束 RT-C1 ~ RT-C3" in text


# ---------------------------------------------------------------------------
# Serialisation.
# ---------------------------------------------------------------------------

# The json5 text is what the C++ replica is compared against, so it has to be
# both valid json and stable. Compact separators are asserted through the absence
# of a space after each separator rather than by comparing the whole string: a
# whole-string comparison would have to be rewritten for any field the contract
# adds, and a test nobody can edit safely is a test that gets deleted.
@pytest.mark.parametrize("plane", sorted(TRANSPORT_PLANES))
def test_json5_is_parseable_and_compact(plane):
    """The serialised form round-trips to the document it came from.

    Round-trip rather than a string comparison: what has to hold is that the
    text carries the same tree, and pinning the exact bytes here would make
    every future contract field a two-place edit.
    """
    text = session_config_json5(plane)
    assert json.loads(text) == session_config_document(plane)
    # Compact separators, checked as the absence of a space after one. The C++
    # replica emits the same form, and a drift in spacing there would otherwise
    # only surface as a confusing diff in the cross-language driver.
    assert not re.search(r"[,:] ", text)
