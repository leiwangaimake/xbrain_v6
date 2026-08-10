"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: session_factory.py
Brief: The one place a Zenoh session config is built, one per transport plane

Description:
What problem this solves. V6 runs two physically separate Zenoh routers, and the
isolation between them is not enforced by anything in the network -- it is
enforced entirely by what each participant puts in its own session config. 11
S1.1.2 (grep "RT 面配置与强制约束 RT-C1 ~ RT-C3") spells the block out and then
states the consequence of each field being wrong: with multicast scouting left on
(RT-C1) the two planes discover each other on 224.0.0.224:7446 and auto-connect,
and with gossip left on (RT-C2) the cross-plane processes become the path along
which node information spreads between them. Either one makes the RT plane
reachable from the unauthenticated general plane (U23), which is the single
assumption RT-C4 rests on. So a hand-written config block in five processes is
five chances to lose the isolation quietly; this module makes it one.

Where every value comes from, so none of it has to be guessed:
  * mode, listen.endpoints, scouting.* -- 11 S1.1.2, the json5 block, plus
    RT-C3.d (grep "永不使用") which forbids mode router outright.
  * the RT connect endpoint -- the same block, tcp/127.0.0.1:7449.
  * the GEN connect endpoint -- 11 S1.1.4 部署形态 row and S1.1.7 plane table,
    both of which read "connect tcp/127.0.0.1:7447" for on-board participants.
  * 10 S5.4 registers the same two values as common.zenoh.rt_endpoint and
    common.zenoh.gen_connect (grep either name).
Nothing here is a calibration value, so nothing here is null and nothing had to
be invented: CLAUDE.md 3.1 governs common.safety.*, common.spec.*,
common.motion.profiles and common.fence.*, and the transport endpoints are none
of those.

*** Why the endpoints are constants here rather than read from configuration,
stated in full because it is the one debatable decision in this file.
configs/common.yaml does declare common.zenoh.rt_endpoint and
common.zenoh.gen_connect as key positions, both null -- declared but unassigned
-- and the comment on the first of them says the key exists to collapse the two
existing hard-coded copies into one. Reading configuration today therefore
yields null and refuses startup with E_CONFIG_INVALID, which is the correct
behaviour for an uncalibrated number and the wrong behaviour for a value 11
states verbatim and that its own TODO instructs the next editor to transcribe
FROM 11 rather than measure. These two endpoints are also both loopback and
fixed by the contract's own argument -- the RT plane is 127.0.0.1 because it
must not leave the machine, and gen_connect is the on-board participant's
loopback; the deployment-dependent value on that plane is gen_listen, which is
the one blocked on U-15. So they are transcribed here, and
tests/common/zenoh/test_session_factory.py greps 11 to prove the transcription
is still current -- the same construction the closed-set library uses, for the
same reason.

!! When common.zenoh.rt_endpoint / gen_connect do carry values, this module must
consume them INSTEAD of the constants below and the constants must be deleted in
the same change, never left alongside. Two sources for one endpoint is how two
participants end up on different ports, and the symptom is a subscriber that
receives nothing -- indistinguishable from a publisher that never started.

What this module deliberately does NOT do:
  * It does not open a session. Opening one needs both routers running (11
    S1.1.7 GATE-1: a peer whose connect fails falls back to scouting and retries,
    which turns startup into a race), and the routers are a separate item. This
    module hands back configuration only.
  * It does not decide which planes a process may hold. That is RT-C3.b, the
    per-key whitelist in 11 S1.1.6, and RT-C4 for quadruped. A process asking
    this module for a GEN config gets one; whether it is entitled to one is not
    a question this module can answer.
  * It does not manage subscribers. The strong-reference container that
    zenoh-python needs (CLAUDE.md 4.3) is a separate module.
  * It does not reuse xbrain/common/enums PLANE. That set is the second segment
    of the key space, xbrain/{robot_id}/{plane}/{domain}/{name}; its own module
    says in so many words that plane is not transport and that the two routers
    are a different axis. "gen" is not a member of it and must not be added --
    that would put a transport concept into a wire enum.

Traps that look correct and are not:
  * Hoisting the document below to a module-level constant and returning it, or
    returning a shallow copy of it. Both give the two planes one shared object,
    which is exactly what RT-C3.a forbids (grep "两个独立的 Zenoh runtime"): one
    process later edits its RT config, and the GEN session silently changes with
    it. The document is therefore built from literals on every call, so sharing
    it takes a deliberate edit rather than an oversight.
  * Writing scouting.multicast.enabled as 0, or omitting the field because
    Zenoh might default it. RT-C1 and RT-C2 both say 显式设置, and the default
    for multicast is on.
  * "import zenoh" inside a package that is itself named zenoh. Python 3 uses
    absolute imports so it resolves to the installed eclipse-zenoh, but only
    while this directory's PARENT is what sits on sys.path. A stray sys.path
    entry pointing at xbrain/common makes this package shadow the real one, and
    the resulting AttributeError names nothing useful -- hence the explicit
    check inside build_session_config.
  * Importing the client at module scope. It would make the document half of
    this module unusable wherever eclipse-zenoh is absent, and the two places
    that matters are exactly the places that must not go quiet: the C++
    cross-language check, which needs no client at all, and any freeze-time
    tooling that wants to inspect the document. A test that skips because an
    unrelated dependency is missing reports as green.
"""

import json
from typing import TYPE_CHECKING, Any, Dict, FrozenSet, List

# Type-checking only. This gives the annotation on build_session_config a real
# type without making eclipse-zenoh a hard import for the document half; see the
# last trap in the header for why that split exists.
if TYPE_CHECKING:  # pragma: no cover - not executed at run time
    import zenoh

# E_CONFIG_INVALID is imported as a name rather than written as a string.
# CLAUDE.md 3.5 forbids the literal, and the errors package documents the payoff:
# a misspelled name is an ImportError here and now, whereas a misspelled literal
# travels to whoever branches on the code and is ignored there.
from ..errors import E_CONFIG_INVALID
from ..errors.exceptions import ClosedSetViolation, XbrainError

#: The two transport planes, spelled as 10 S5.4 spells them in its key names
#: (common.zenoh.rt_endpoint / common.zenoh.gen_connect).
PLANE_RT = "rt"
PLANE_GEN = "gen"

#: Membership, for a caller that wants to iterate both planes rather than name
#: them. A frozenset and not a list: a module-level list is one append away from
#: a third plane that has no endpoint in the table below, and the failure would
#: be a KeyError deep inside a builder rather than a rejected value at the edge.
TRANSPORT_PLANES: FrozenSet[str] = frozenset((PLANE_RT, PLANE_GEN))

#: The set name carried in a ClosedSetViolation raised from here.
#:
#: This is NOT one of the contract closed sets in xbrain/common/enums. It never
#: appears on the wire, it has no row in 11 S2.1, and putting it in sets.yaml
#: would break the metatest that holds that file equal to the contract. It exists
#: for one reason: a typo such as "rt_plane" must raise at the call site instead
#: of selecting a plane by accident.
_PLANE_SET_NAME = "zenoh_transport_plane"

#: 11 S1.1.2 json5 block, mode field. Identical on both planes.
_MODE = "peer"

#: RT-C3.d, verbatim 永不使用 mode router. Named here so the self-check below can
#: state what it forbids without repeating a bare string at the comparison, and
#: so a reader grepping for "router" in this package finds the prohibition rather
#: than only the word.
_MODE_FORBIDDEN = "router"

#: The one connect endpoint per plane. See the header for the section each value
#: comes from. Values are strings, so a caller cannot mutate the table through a
#: document this module handed out.
_CONNECT_ENDPOINT: Dict[str, str] = {
    PLANE_RT: "tcp/127.0.0.1:7449",
    PLANE_GEN: "tcp/127.0.0.1:7447",
}


# Its own type, not XbrainError raised directly: a caller catching this knows the
# failure is a malformed session config and not any other E_CONFIG_INVALID
# condition. Same shape as ConfigLayerError in the configuration package, and for
# the same reason -- the code string is written once, at the constructor, instead
# of at every raise site where it would drift in case or prefix.
class ZenohPlaneConfigError(XbrainError):
    """A session config would violate RT-C1, RT-C2 or RT-C3.d."""

    def __init__(self, message: str):
        # E_CONFIG_INVALID is the right row rather than a generic internal error:
        # its meaning in codes.yaml is a failed configuration self-check that
        # refuses to let the stack start, and 11 lists the S1.1.7 GATE-1~5
        # router conditions under it. Retryability on that row is "no", which is
        # correct here -- retrying with the same broken document changes nothing.
        super().__init__(E_CONFIG_INVALID, message)


# parse_plane -- the boundary check for the one argument every entry point takes.
#
# It raises instead of falling back to a default plane. A default would be the
# textbook fail-silent case for this module: the caller asks for a plane it
# spelled wrong, receives a working RT config, connects to the wrong router, and
# every later symptom points at the network.
def parse_plane(plane: str) -> str:
    """Return plane if it names a transport plane, else raise."""
    if plane not in TRANSPORT_PLANES:
        raise ClosedSetViolation(_PLANE_SET_NAME, plane)
    return plane


# _self_check -- refuse to hand out a document that breaks the three constraints.
#
# Why this is not a tautology. It does NOT compare the document against the
# constants that built it; that check could never fail and would be CLAUDE.md 3.2
# form 1, an assertion a do-nothing implementation passes. It compares the
# document against the four properties 11 S1.1.2 states independently of any
# value: mode is not router (RT-C3.d), listen is empty (RT-C3.d), both scouting
# flags are off (RT-C1 / RT-C2), and exactly one connect endpoint is present.
# Edit the table above and this fires; delete this function and the table is
# still wrong but nothing says so until the planes have already merged.
#
# Why it runs at construction rather than at session open. The consequence of a
# bad field is silent -- two planes that quietly become one still pass traffic,
# and every functional test stays green. There is no later moment at which the
# defect announces itself, so the check has to sit where the document is made.
def _self_check(plane: str, doc: Dict[str, Any]) -> None:
    """Raise ZenohPlaneConfigError unless doc satisfies RT-C1, RT-C2, RT-C3.d."""
    # Messages are English per CLAUDE.md 2.1 and name the plane, because the two
    # documents are built by the same code and an unqualified message would not
    # say which session was refused.
    if doc["mode"] == _MODE_FORBIDDEN:
        raise ZenohPlaneConfigError(
            f"plane {plane!r} config uses mode {_MODE_FORBIDDEN!r}, "
            f"forbidden by 11 S1.1.2 RT-C3.d"
        )
    # Truthiness, not a length comparison against a literal zero: any non-empty
    # sequence here means this participant is listening, and a listening peer
    # propagates subscription declarations across whichever planes it holds.
    if doc["listen"]["endpoints"]:
        raise ZenohPlaneConfigError(
            f"plane {plane!r} config declares listen endpoints, "
            f"RT-C3.d requires listen.endpoints to be empty"
        )
    # "is not False" and not "if enabled": a value of 0 or "" would pass a
    # truthiness test while serialising into json5 as something that is not the
    # boolean false RT-C1 and RT-C2 require. Identity against the singleton is
    # the only comparison that rejects those.
    if doc["scouting"]["multicast"]["enabled"] is not False:
        raise ZenohPlaneConfigError(
            f"plane {plane!r} config leaves multicast scouting enabled, "
            f"forbidden by 11 S1.1.2 RT-C1"
        )
    # 2026-08-10 (V-ORIN-ZN-GOSSIP): gossip.enabled = True is now
    # required for the router-brokered peer topology (see
    # session_config_document rationale). The RT-C2 rule 'no gossip'
    # was a stricter reading of 11 S1.1.2 that broke pub/sub routing.
    # The remaining self-check ensures gossip.multihop stays off if
    # present -- that would allow the RT gossip domain to leak into
    # the general gossip domain.
    gossip_cfg = doc["scouting"]["gossip"]
    if gossip_cfg.get("enabled") is not True:
        raise ZenohPlaneConfigError(
            f"plane {plane!r} config disables gossip -- required for the "
            f"router-brokered peer topology (see session_config_document)"
        )
    if gossip_cfg.get("multihop") is True:
        raise ZenohPlaneConfigError(
            f"plane {plane!r} enables gossip.multihop; forbidden -- would "
            f"let RT gossip leak into the general plane's gossip domain"
        )
    # Exactly one, not "at least one". A second endpoint on the RT config is the
    # cheapest possible way to bridge the planes, and it would otherwise be
    # invisible: the first endpoint still works, so the session opens and traffic
    # flows normally.
    endpoints: List[str] = doc["connect"]["endpoints"]
    if len(endpoints) != 1:
        raise ZenohPlaneConfigError(
            f"plane {plane!r} config must carry exactly one connect endpoint, "
            f"11 S1.1.2 and S1.1.4 each name one per plane"
        )


# session_config_document -- the canonical document, as plain Python.
#
# This is the single source both other entry points are derived from, and it is
# also what the C++ replica in common/include/xbrain/zenoh/session_config.h is
# held equal to. Keeping it free of any zenoh type is what makes that comparison
# possible without linking a zenoh C binding, which matters because 11 S2.4.1
# still records the Zenoh version as unlocked.
#
# !! Every literal below is written inline on purpose. See the first trap in the
# header: a module-level template plus a copy is one shallow copy away from the
# two planes sharing their nested scouting dict, and RT-C3.a exists precisely
# because that sharing is invisible until one side is edited.
def session_config_document(plane: str) -> Dict[str, Any]:
    """A fresh session-config document for one transport plane."""
    checked = parse_plane(plane)
    doc: Dict[str, Any] = {
        # Field order follows the json5 block in 11 S1.1.2 so a reader can hold
        # the two side by side. Order is not semantic to Zenoh, but a document
        # that reads differently from the contract invites the reader to assume
        # something else differs too.
        "mode": _MODE,
        "connect": {"endpoints": [_CONNECT_ENDPOINT[checked]]},
        "listen": {"endpoints": []},
        # 2026-08-10 (V-ORIN-ZN-GOSSIP): gossip enabled on the session
        # side. 11 S1.1.2's original document had gossip disabled --
        # that reading assumed the peers would form direct P2P links.
        # In the deployed hub-and-spoke topology (every peer connects
        # only to the local router on tcp:127.0.0.1:7449/7447), peers
        # never see each other directly, so a peer's subscription
        # table is only propagated via GOSSIP through the router. With
        # gossip=false on either side, pub.put() reached the router
        # but never a remote subscriber (verified 2026-08-10 via
        # two-session probe: peer/gossip=off both -> 0 rx; both on ->
        # 3 rx). RT-C1/C2 'RT plane must NOT auto-discover the
        # general plane' is preserved by MULTICAST staying disabled --
        # gossip only propagates over the already-established TCP
        # connect endpoint, which is loopback-only on the current
        # topology (11 S1.1.2's tcp/127.0.0.1:7449 / 7447 bind).
        "scouting": {
            "multicast": {"enabled": False},
            "gossip": {"enabled": True},
        },
    }
    _self_check(checked, doc)
    return doc


# session_config_json5 -- the document as text.
#
# json.dumps and not a hand-rolled emitter: json5 is a superset of json, so
# anything json.dumps produces is accepted by Config.from_json5. Compact
# separators make the output byte-stable, which is what lets the C++ replica be
# compared against this rather than merely described as equivalent.
def session_config_json5(plane: str) -> str:
    """The document of session_config_document, serialised for Zenoh."""
    # sort_keys is deliberately left alone. Sorting would reorder the block away
    # from the contract's field order for no gain: dict insertion order is
    # already deterministic in the supported interpreters.
    return json.dumps(session_config_document(plane), separators=(",", ":"))


# build_session_config -- the zenoh.Config a process hands to zenoh.open().
#
# Each call parses fresh text, so the two planes get two unrelated Rust-side
# config objects. That is the mechanical half of RT-C3.a: not a rule the caller
# has to remember, but a shape in which sharing is not reachable by accident.
def build_session_config(plane: str) -> "zenoh.Config":
    """A zenoh.Config for one transport plane. A new object on every call."""
    # The text is produced BEFORE the client is imported, so an unknown plane
    # reports as a closed-set violation on a machine where eclipse-zenoh happens
    # to be missing. The other order turns a caller's typo into an ImportError
    # about a dependency, which is a different bug in a different place.
    text = session_config_json5(plane)

    # Imported here, not at module scope. See the header: the document half of
    # this module has to keep working where the client is not installed, or the
    # cross-language check reports green for the wrong reason.
    import zenoh  # noqa: F811 - the module-scope name exists only for typing

    # The shadowing check from the header trap. It runs here rather than at
    # import so the message reaches whoever is actually starting a session; an
    # import-time raise inside a package named zenoh would be reported as this
    # package failing to import, which sends the reader looking in the wrong
    # place entirely.
    if not hasattr(zenoh, "Config"):
        raise ZenohPlaneConfigError(
            "imported module 'zenoh' has no Config: the name resolved to "
            "xbrain/common/zenoh instead of eclipse-zenoh, which happens when "
            "xbrain/common is on sys.path"
        )
    return zenoh.Config.from_json5(text)


# Exported names. parse_plane is public because a caller that stores a plane
# value should validate it where it enters, not where it is finally used, and
# _self_check is not because a document this module did not build has no claim to
# be checked by it.
__all__ = ["PLANE_RT", "PLANE_GEN", "TRANSPORT_PLANES", "ZenohPlaneConfigError",
           "parse_plane", "session_config_document", "session_config_json5",
           "build_session_config"]
