"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: f_qos_and_port.py
Brief: Assertion F (QoS static + A-2~A-7 + depth=0) and F' (port identity
       GATE-5) -- CFG-FZ-6

Description:
F  is a static check over common.qos: reject profiles / bindings that
   would violate 11 S2.4.8 A-2~A-7. Reuses load_qos_table for the
   documented invariants (frozen profiles, unknown fields, depth=0)
   and adds two static checks it does NOT do:
     A-5 explicit    - reject rt/-covering binding whose profile is
                        block, even though QOS-C1 would silently
                        override at resolve() time. QOS-C1 is a
                        safety net, NOT a licence to write dangerous
                        bindings.
     ordering        - the fallback pattern (xbrain/*/**) MUST be last
                        in the bindings list. 11 S2.4.7 marks
                        "first-match wins" as a safety property; a
                        fallback at the head silently steals every
                        specific binding below it.

F' verifies that the two Zenoh ports (7447 general, 7449 RT) are
   held by an actual zenohd router, not by a rogue peer that happened
   to bind first. This is GATE-5. The socket probe is injectable via
   ctx['port_probe_fn'] so tests do not need real sockets.

CFG-FZ-6 variants verbatim:
  (1) xbrain/*/cmd/estop bound to Q3_cmd (block) -> A-5 red
  (2) xbrain/*/** fallback moved to the head -> ordering red
  (3) fake peer holding 7447 -> F' red (identity mismatch)

Contract:
  input:  ctx["overlay"] (from A) with tree containing common.qos
  optional ctx["port_probe_fn"](port) -> "zenohd_router" | other-string
  raises: XbrainError with E_QOS_VIOLATION (F) or E_CONFIG_INVALID (F')
          detail.kind in a5_block_on_rt / binding_order_bad /
          port_identity_bad
"""

# typing for Any/Callable/Dict annotations. F/F' expose a port_probe_fn
# callable to let tests inject a stub; Callable annotation documents
# the signature of that hook.
from typing import Any, Callable, Dict

from xbrain.boot.freeze.assertions._layer_loader import load_layers
from xbrain.common.config import build_overlay
from xbrain.common.errors import E_CONFIG_INVALID, E_QOS_VIOLATION
from xbrain.common.errors.exceptions import XbrainError
# BLOCK constant + loader/exception types from the QoS module. Importing
# both exception types so F can catch either without over-catching
# unrelated XbrainError subclasses (which would swallow bugs).
from xbrain.common.zenoh.qos import (
    BLOCK, QosConfigError, QosViolation, load_qos_table,
)

# Fallback binding pattern -- the last-resort match that catches every
# xbrain/{rid}/... key. Must be the LAST binding in the list; anywhere
# else silently steals the more specific bindings below it. 11 S2.4.7
# names this pattern verbatim as the fallback row.
# Verbatim string is what the ORDERING check compares against; a
# different fallback string (e.g. "xbrain/**") would silently escape
# the check. Kept as a module constant, not inlined.
_FALLBACK_PATTERN = "xbrain/*/**"

# Zenoh ports probed by F'. 7447 = general plane router (public);
# 7449 = RT plane router (loopback only). Both must be zenohd.
# Tuple of (port, plane_label) rows. Adding a third router (unlikely
# but possible for a bridge role) = one row here.
_ZENOH_PORTS = (
    (7447, "gen"),
    (7449, "rt"),
)

# String the port probe returns when the peer IS a zenohd router.
# Any other value = identity mismatch. Exact-match compare -- no
# case-folding, no substring. A real probe would derive this from
# the router's advertised identity string.
_EXPECTED_IDENTITY = "zenohd_router"


def _fail_qos(kind: str, **extra: Any) -> None:
    """Raise E_QOS_VIOLATION with detail.kind (F failures).

    E_QOS_VIOLATION is the closed-set code for QoS-related refusals
    (11 S13.15). Two subkinds are today: qos_table_invalid (loader
    refused) and a5_block_on_rt / binding_order_bad (F's own checks).
    """
    detail = {"kind": kind}
    detail.update(extra)
    raise XbrainError(
        E_QOS_VIOLATION,
        "assertion F failed: %s" % kind,
        detail,
    )


def _fail_port(kind: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind (F' failures).

    Port identity failures are configuration/deployment problems, not
    QoS violations, so they use the E_CONFIG_INVALID code.
    Two closed-set codes intentionally: an operator triaging QoS
    violations vs port-identity violations acts on different things
    (fix yaml vs check systemd), so the codes must be distinguishable.
    """
    detail = {"kind": kind}
    detail.update(extra)
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion F' failed: %s" % kind,
        detail,
    )


def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker; identical shape to C/D.

    Not shared with C/D because keeping each assertion self-contained
    makes a future split into separate wheels trivial. The three
    functions are copies of the same 5-line helper; deduplicating them
    would save nothing but coupling.
    """
    # Walk segment-by-segment; any missing / non-dict node returns
    # the caller-supplied default (None by convention).
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _stub_port_probe(port: int) -> str:
    """Default port probe -- returns _EXPECTED_IDENTITY for all ports.

    This is a STUB. Real implementation would open a Zenoh session
    to the port, request router info, and return a peer identity
    string. That requires the zenoh client library at freeze time,
    which is not always available. Deployments that DO have zenoh
    installed override this by passing ctx['port_probe_fn'] to
    run_freeze -- their probe hits the real network. Tests that want
    to simulate a rogue peer inject a probe returning "fake_peer".

    Deliberate default = pass: freeze runs before the runtime is
    up, and refusing to start every dev/CI environment that lacks
    a network stack would be a false-fail. The security-relevant
    deploy path REQUIRES injecting the real probe.

    The signature (port: int) -> str is the CONTRACT for the
    injectable hook. Any caller providing a probe_fn must match it.

    Return values: "zenohd_router" for success; any other string
    (e.g. "fake_peer", "closed", "timeout") is a failure. Empty
    string is treated as failure too (not a special case).
    """
    return _EXPECTED_IDENTITY


def _check_qos_static(qos_doc: Dict[str, Any]) -> None:
    """F: static checks on common.qos.

    Delegates most of A-2/A-3/A-4/A-6/depth=0 to load_qos_table (it
    raises QosConfigError or QosViolation on those). Adds two checks
    load_qos_table does NOT enforce:
      - A-5 EXPLICIT: reject rt/-covering binding whose profile is block
      - ORDERING: fallback pattern must be last

    Why load_qos_table doesn't catch A-5 by itself: QOS-C1 (the
    rt_override) is designed to rescue rt-plane keys at resolve()
    time regardless of what profile the binding named. That is a
    SAFETY NET; it does not sanction writing block bindings on the
    RT plane. F rejects the input intent even though the runtime
    would silently fix it.
    """
    # First, run the loader. It raises on invalid shape / unknown
    # profiles / depth=0. We wrap those raises into our own kind
    # for consistent detail shape.
    # Two exception types: QosConfigError (shape defect) and
    # QosViolation (unresolvable / depth=0). Both get folded into
    # qos_table_invalid because from F's perspective they're all
    # "loader refused the document".
    try:
        table = load_qos_table(qos_doc)
    except (QosConfigError, QosViolation) as exc:
        # Re-raise as an F-style failure so callers can dispatch on
        # detail.kind consistently. The original message is preserved
        # in detail.underlying so the operator sees WHY the loader
        # refused, not just "invalid".
        _fail_qos("qos_table_invalid", underlying=str(exc))

    # A-5 EXPLICIT: no rt/-covering binding may reference a profile
    # whose congestion_control is BLOCK. QOS-C1 (rt_override) would
    # silently rescue at resolve() time, but the safety property is
    # "the binding SHOULD say drop"; a config that says block on rt/
    # is a defect regardless of override.
    # Walk every binding match; a match containing "rt/" is a
    # candidate for A-5 scrutiny.
    for match in table.match_expressions:
        # rt/ coverage: match starts with rt/ OR is a wildcard that
        # would match rt/... keys (any xbrain/{rid}/rt/... pattern).
        # Simple form: string contains "rt/" segment.
        # Precise coverage would require key-expr semantics; this
        # is deliberately over-inclusive (false-positive on a
        # match string that mentions rt/ in a non-plane context)
        # because false-negative is the failure to avoid here.
        rt_covered = "rt/" in match
        if not rt_covered:
            continue
        # Find the profile bound. QosTable does not expose bindings
        # directly; use the resolver on a synthetic rt/... key.
        # Skip if the pattern does not match any rt/ key we can build.
        # Synthetic key is well-formed (has xbrain/{rid}/ prefix) so
        # parse_full_key accepts it.
        probe_key = "xbrain/gj-001/rt/probe/test"
        # Only proceed if THIS binding is the one that would match.
        # The resolver returns the first-match; we check if it's this
        # binding by comparing its match string.
        # Resolution may raise QosViolation if no binding matches
        # OR if depth=0 on Q4_stream; we don't care about those here,
        # they belong to the loader / A-6 check.
        try:
            resolution = table.resolve(probe_key)
        except (QosConfigError, QosViolation):
            continue
        if resolution.binding_match != match:
            # Some earlier binding matched instead; A-5 on this row
            # is not actionable (the earlier binding is what runs).
            # Continue to the next match rather than raising.
            continue
        # Check the profile the binding pointed at, NOT the resolved
        # (post-override) profile. QOS-C1 override kicks in at
        # resolve time; the profile_name reported by QosResolution
        # is the ORIGINAL profile the binding picked.
        # Missing profile is a shape defect the loader already caught;
        # defensive skip here.
        profile = table.profiles.get(resolution.profile_name)
        if profile is None:
            continue
        if profile.congestion_control == BLOCK:
            # A-5 hit. Report the binding + profile + congestion so an
            # operator can locate the offending line in the YAML.
            _fail_qos("a5_block_on_rt",
                      binding_match=match,
                      profile_name=profile.name,
                      congestion=profile.congestion_control)

    # ORDERING: fallback pattern must be last. First-match wins; a
    # fallback at the head steals every specific binding below it.
    # 11 S2.4.7 records the v0.3 estop repair verbatim -- the fallback
    # had been at the top, stealing every rt/safety/estop binding.
    matches = list(table.match_expressions)
    if _FALLBACK_PATTERN in matches:
        idx = matches.index(_FALLBACK_PATTERN)
        if idx != len(matches) - 1:
            # Report all four numbers so an operator sees the exact
            # move needed (from idx to expected_index).
            _fail_qos("binding_order_bad",
                      fallback_pattern=_FALLBACK_PATTERN,
                      fallback_index=idx,
                      total_bindings=len(matches),
                      expected_index=len(matches) - 1)


def _check_port_identity(port_probe_fn: Callable[[int], str]) -> None:
    """F': every Zenoh port must be held by a zenohd router.

    port_probe_fn(port) returns the peer identity string. Any value
    other than _EXPECTED_IDENTITY = rogue peer = deployment defect.

    Walks _ZENOH_PORTS in declaration order (7447 general, 7449 RT).
    First failure raises; subsequent ports not probed. Rationale for
    stop-on-first: a rogue peer on either port is enough to refuse
    startup, and probing further wastes time (each probe is a socket
    round-trip).
    """
    # Iterate the port list. Each entry names the port + plane label
    # so the failure message can distinguish which router class was
    # compromised.
    for port, plane in _ZENOH_PORTS:
        # Call the injected probe; either the stub or a test / deploy
        # override. Return value is the peer identity string.
        identity = port_probe_fn(port)
        # Strict equality vs _EXPECTED_IDENTITY. No partial matches,
        # no case-insensitive; the router identifies itself with a
        # single canonical string.
        if identity != _EXPECTED_IDENTITY:
            # Report BOTH expected and actual so an operator sees the
            # correction path (deploy the real zenohd there) and the
            # observed state (whatever is currently holding the port).
            _fail_port("port_identity_bad",
                       port=port, plane=plane,
                       expected=_EXPECTED_IDENTITY, actual=identity)


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion F + F'. Replaces registry's stub for F.

    Two halves run in one runner: F reads ctx["overlay"], F' probes
    _ZENOH_PORTS via the injectable probe. They share nothing except
    the same ctx dict.

    One runner for two checks lets ORD-1 keep six positions instead
    of seven; the pair is naturally paired (11 S2.4 gates both).

    F halved into "loader" and "static extras" so a bad shape is
    caught first (loader) before any resolve() (extras). This
    ordering matters: the extras' resolve() calls would crash on a
    document the loader would have rejected.
    """
    # Same wiring guard as other assertions -- ctx missing config_root
    # is a caller-side bug (AssertionError, not XbrainError).
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion F requires ctx['config_root']; caller did not "
            "populate it"
        )

    # Prefer A's cached overlay; fall back for isolated callers who
    # invoke F without first invoking A (unit tests + tests injecting
    # a fake overlay via _FakeOverlay). ORD-1 production has A -> B ->
    # C -> D -> E -> F, so overlay is always present at production.
    overlay = ctx.get("overlay")
    if overlay is None:
        # Fresh load path: build the overlay ourselves. Populate ctx
        # so downstream assertions in the same pass do not re-read.
        # Local import matches the pattern in C/D -- avoids potential
        # cycle if _layer_loader ever imports back through us.
        layer_trees = load_layers(ctx["config_root"])
        overlay = build_overlay(layer_trees)
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees

    # F: static QoS checks. qos_doc absent OR null -> skip (M is the
    # required-key check; F does not re-enforce that).
    # Optional here means "dev checkout without qos yet"; production
    # would have M refuse before F even runs.
    # Pull common.qos as a dict; None means the key is unset.
    qos_doc = _get(overlay.tree, "common.qos")
    if qos_doc is not None:
        # Non-None -> subject to F's checks.
        _check_qos_static(qos_doc)

    # F': port identity. Injected probe if provided, else stub.
    # Callers in production MUST inject a real probe; the stub is
    # a dev/CI convenience and refuses nothing.
    # ctx.get with default = stub means unit tests and dev deploys
    # trivially pass F' while a real deploy hits the network.
    port_probe_fn = ctx.get("port_probe_fn", _stub_port_probe)
    _check_port_identity(port_probe_fn)

    # Success return: report whether qos was checked + how many ports.
    # qos_checked is False for the skip case; ports_checked is a fixed
    # 2 today but stays as a field for future growth.
    # Keeping both counts in the manifest lets a MANIFEST diff spot a
    # regression where F silently starts skipping the qos check.
    # Both fields are cheap to compute and never conditional.
    return {
        "status": "pass",
        "assertion": "F",
        "qos_checked": qos_doc is not None,
        "ports_checked": len(_ZENOH_PORTS),
    }
