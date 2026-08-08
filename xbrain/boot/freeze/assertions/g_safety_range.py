"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: g_safety_range.py
Brief: Assertion G -- safety parameter range checks (CFG-FZ-7)

Description:
Runs NINTH in the freeze pipeline (ORD-1, after F). Walks a REGISTRY
of range checks, each named by its source rule (SP-N / AS-N / S-N),
and fails on the first violation with detail.key + detail.rule.

Check registry (11 S9.6 verbatim, minus SP-8 which is doc-CI's job):
  SP-1  spec.max_v{x,y}/w{z}/accel/decel  all > 0
  SP-2  spec.max_vx_mps >= max(motion.profiles[*].max_mps)
  SP-5  safety.brake.k >= 1.0 AND
        0 < safety.brake.a_mps2 <= spec.max_decel_mps2 AND
        safety.t_lat_s >= 0.4
  SP-11 gateway.gpu_token.throttle_speed_mps < spec.max_vx_mps
  AS-7  gateway.{asr,llm,tts}.timeout_s <= 5.0 (11 S8.13.1 upper bound)

SP-8 is EXPLICITLY out of scope for this runner -- 11 S9.6 verbatim
says its executable body is doc-CI, not freeze. SP-3/4/6/9/10 and
S-1~S-6 (12 S12.1) are documented but currently deferred (they need
either provenance tracking (SP-3), specific config keys not yet in
the tree (SP-9/10), or the f() speed-gate function (SP-6)). Deferred
rows land in a follow-on CFG-FZ-N item.
The deferred set is documented here (not silently absent) so the
next implementer sees exactly what still needs a runner.

CFG-FZ-7 named variants:
  (1) gateway.gpu_token.throttle_speed_mps >= spec.max_vx_mps
      -> SP-11 red
  (2) gateway.asr.timeout_s = 30.0
      -> AS-7 red (5 s upper bound)
  (3) safety.t_lat_s = 0.2
      -> SP-5 red (also 12 S12.1 S-5-2 in a later runner)

Contract:
  input:  ctx["overlay"] (from A) with merged tree
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind='sp_out_of_range'
          + detail.rule (SP-N/AS-N) + detail.key + detail.value +
          detail.limit
"""

# typing for Any/Callable/Dict/List/NamedTuple. NamedTuple is used to
# make each _CheckRow read as a data row rather than a bag of fields.
from typing import Any, Callable, Dict, List, NamedTuple

# Layer loader for the isolated-callers fallback path; overlay is A's
# job in the production ORD-1 sequence but tests may call G directly.
from xbrain.boot.freeze.assertions._layer_loader import load_layers
from xbrain.common.config import build_overlay
# XbrainError = base for every deliberate raise; G uses E_CONFIG_INVALID
# uniformly (all failures are "the tree has a bad value").
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# AS-7 upper bound for ASR/LLM/TTS timeouts. 11 S8.13.1 verbatim: 5 s.
# Anything above surfaces as user-visible latency in the voice loop.
# Hard-coded here rather than pulled from config -- AS-7 is the DOC
# upper bound, not a deploy-tunable knob.
_AS7_TIMEOUT_UPPER_S = 5.0

# SP-5 minimum control-latency. 11 S9.6.1 verbatim: 0.4 s. U54 tightened
# this from ">0" to ">=0.4" to reject the pre-U54 residual value 0.2 --
# which would low-estimate d(2.0) from 1.60 m to 1.20 m, over-authorising
# speed and under-reporting fence stopping distance simultaneously.
# Two-decimal-place literal matches the doc verbatim, not a float trick.
_SP5_T_LAT_MIN_S = 0.4

# SP-5 brake factor lower bound. Below 1.0 the brake model becomes
# non-conservative and speed_gate.f() over-estimates authorised speed.
# 1.0 is the safety-model minimum; deploys can set a higher k for
# stricter behaviour but not lower.
_SP5_BRAKE_K_MIN = 1.0

# SP-4 upper bound for max_vx_mps. Set to 2.0 per U54 (云深处 responded
# "actual max 2 m/s, does not support 9 m/s"). Values above are treated
# as a warning; SP-4's real body is deferred (needs event bus).
# Referenced here in comments to explain why 2.0 shows up in a comment
# above and NOT as an SP-4 check row.
# The `2.0` constant is documented here as the future-work anchor for
# SP-4's runner (deferred to a later CFG-FZ-N item).


class _CheckRow(NamedTuple):
    """One row of the check registry.

    rule:  identifier (SP-1 / SP-5 / SP-11 / AS-7 / ...)
    desc:  one-line human description (printed nowhere today; kept for
           future MANIFEST reporting).
    check: callable(tree) -> None; raises XbrainError on failure.

    NamedTuple is used so each row reads as data ("rule", "desc",
    "check") rather than three positional args. Adding a fourth field
    (e.g. per-rule severity) means one edit here + one per row.
    """
    rule: str
    desc: str
    check: Callable[[Dict[str, Any]], None]


def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker -- same shape as C/D/F.

    Kept local (not shared) so G stays self-contained. The three
    functions are copies of the same 5-line helper; deduplicating
    them would save nothing but coupling.
    """
    # Walk segment-by-segment; any missing / non-dict node returns
    # the caller-supplied default (None by convention).
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _fail(rule: str, key: str, value: Any, limit: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.rule + key + value + limit.

    rule:  which SP-N / AS-N / S-N was violated (used by operator triage).
    key:   dotted path of the offending config key.
    value: what the key held (for the operator to see what they wrote).
    limit: text describing the bound (e.g. "> 0", "<= 5.0", ">= 0.4").

    All four fields are required (positional) because a failure message
    without any of them leaves the operator unable to act. Extra kwargs
    let each raise site attach explanatory context ('reason' typically).
    detail.kind stays constant as 'sp_out_of_range' -- G's whole surface
    is a single kind; the rule field distinguishes which rule fired.
    """
    detail = {"kind": "sp_out_of_range", "rule": rule, "key": key,
              "value": value, "limit": limit}
    detail.update(extra)
    # Message string interpolates all four so a journalctl reader
    # without decoder scripts still sees the full picture.
    # Format: 'assertion G failed: <RULE> on <key> = <value> (<limit>)'
    # for at-a-glance triage.
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion G failed: %s on %s = %r (%s)"
        % (rule, key, value, limit),
        detail,
    )


# ---------------------------------------------------------------------------
# Individual check bodies
# ---------------------------------------------------------------------------

# SP-1 leaves: five spec keys that must all be > 0.
# Order matters for reproducible failure logs -- the walk raises on
# the first offender, so alphabetical (vx, vy, wz, accel, decel)
# gives predictable behaviour when multiple keys are bad.
_SP1_KEYS = (
    "common.spec.max_vx_mps",
    "common.spec.max_vy_mps",
    "common.spec.max_wz_radps",
    "common.spec.max_accel_mps2",
    "common.spec.max_decel_mps2",
)


def _check_sp1_spec_positive(tree: Dict[str, Any]) -> None:
    """SP-1: every spec.max_* > 0.

    Skips a key whose value is None (assertion A caught that). Skips a
    key that is not a number (shape defect, another check reports).
    Reports the first key that IS a number and IS not > 0.

    Why strict > 0 (not >= 0): a spec value of 0 makes v_max = min(...,
    0) = 0 in speed_gate.f(), which reads as "cannot move" -- machine
    silently limited to zero speed with no runtime warning. That is
    fail-silent, the failure mode CLAUDE.md S3.1 spends the most words
    on. SP-1 exists specifically to catch it before it gets that far.
    """
    # Walk the SP-1 key tuple in order; first bad value raises.
    # Order is fixed (see _SP1_KEYS) so a failure log names the same
    # key for the same input across runs. Reproducibility matters
    # for CI diffing.
    for key in _SP1_KEYS:
        val = _get(tree, key)
        if val is None:
            # Assertion A already refused the tree; we would not reach
            # here in production. Defensive skip for unit-test paths
            # that inject a partial tree without common.spec.max_vy.
            continue
        if not isinstance(val, (int, float)):
            # Shape defect -- other checks report. Skip so we don't
            # raise TypeError comparing None > 0. bool is a subclass
            # of int, so True/False would pass this instanceof; that
            # is a schema-shape defect for another check to catch.
            continue
        if val <= 0:
                # Zero and negatives both fail: negative is nonsense,
            # zero is silent lockdown. Same rule catches both without
            # branching on sign.
            _fail("SP-1", key, val, "> 0",
                  reason="a spec value of 0 or below limits v_max to 0 "
                         "in speed_gate.f() with no runtime warning")


def _check_sp2_max_vx_covers_profiles(tree: Dict[str, Any]) -> None:
    """SP-2: max_vx_mps >= max(profiles[*].max_mps).

    Otherwise the fastest profile is physically unreachable: the
    profile table says "patrol tops out at 2.0 m/s" but the vehicle
    spec caps at 1.5 m/s. In production the runtime clamps to
    min(spec, profile), so patrol would silently only reach 1.5 m/s
    without any complaint.

    Concrete example this catches: an operator narrows spec.max_vx
    for a specific site (e.g. narrow corridor) but forgets to shrink
    the profile table. Both values look reasonable in isolation but
    the combined behaviour surprises operators in the field.
    """
    # Pull the two operands; both must exist (M refuses if missing).
    # Pull both operands; both required (M refuses if either missing).
    # Pair-wise skip if either operand shape-defective.
    max_vx = _get(tree, "common.spec.max_vx_mps")
    profiles = _get(tree, "common.motion.profiles")
    # Type gate: shape defects skip to keep _fail's contract clean.
    # Non-numeric max_vx or non-dict profiles = defect for another
    # check to report; here we just avoid TypeError.
    if not isinstance(max_vx, (int, float)) or not isinstance(profiles, dict):
        return
    # Find the profile with the largest max_mps. Iterate + track the
    # winner so we can report BOTH the value and the profile name in
    # the failure detail. Using explicit winner-tracking (not max())
    # because we need the NAME too, not just the value.
    max_profile = None
    max_profile_name = None
    # Walk every profile row; pick the winner.
    # Iterate profile rows -- dict of {name: {max_mps: value, ...}}.
    for name, prof in profiles.items():
        if not isinstance(prof, dict):
            # Non-dict profile row = shape defect, skip. Real
            # schema check should report; SP-2 defers.
            continue
        p_max = prof.get("max_mps")
        if isinstance(p_max, (int, float)):
            # Track winner + name for the failure message.
            # First numeric value wins the initial round; ties in
            # value keep the first-seen name (deterministic when
            # dict iteration is insertion-ordered).
            if max_profile is None or p_max > max_profile:
                max_profile = p_max
                max_profile_name = name
    if max_profile is None:
        # No numeric profile found -- nothing to compare against.
        # Skip; a caller who cares about "must have at least one
        # profile" belongs in M's required-key list, not here.
        return
    # Strict < (not <=): equality is OK (max_vx equals largest
    # profile; profile is just reachable, not exceeded).
    if max_vx < max_profile:
        # Report the exceeded max plus the profile that owns it, so
        # the operator sees both operands of the inequality.
        _fail("SP-2", "common.spec.max_vx_mps", max_vx,
              ">= max(profiles[*].max_mps) = %r (%s)"
              % (max_profile, max_profile_name))


def _check_sp5_brake_and_latency(tree: Dict[str, Any]) -> None:
    """SP-5: brake.k >= 1.0 AND 0 < brake.a_mps2 <= max_decel AND
    t_lat_s >= 0.4.

    Three sub-conditions checked in order; first failure raises. The
    order is set-alphabetical (k, a_mps2, t_lat_s) so the failure log
    is deterministic across runs.

    Why three sub-checks in one function: they share source (all
    common.safety.brake.* and common.safety.t_lat_s) and share
    consequence (all three feed speed_gate.f() and any one wrong
    over-authorises speed). Splitting them across three functions
    would fragment the "why" comments; keeping them together makes
    the failure-mode analysis easier to audit.
    """
    # Pull all four operands up front. brake sub-tree is optional in
    # dev checkouts but M refuses in production if these are missing.
    # Batch pulls keep the pull site clean; each sub-check tests
    # its own operands' presence + type.
    k = _get(tree, "common.safety.brake.k")
    a = _get(tree, "common.safety.brake.a_mps2")
    t_lat = _get(tree, "common.safety.t_lat_s")
    max_decel = _get(tree, "common.spec.max_decel_mps2")

    # brake.k: must be >= 1.0 if present.
    # k < 1 makes the brake model non-conservative -- the actual
    # stop distance is longer than the model estimates, so
    # speed_gate.f() authorises speeds the physical brake can't stop.
    # Also isinstance guard: bool is subclass of int in Python, so
    # True/False would slip past isinstance((int,float)). We accept
    # that risk here because a boolean value in a k slot is a shape
    # defect for another check to report.
    # Standard pattern: value must be numeric-shaped AND fail-limit,
    # both true, before raising.
    if isinstance(k, (int, float)) and k < _SP5_BRAKE_K_MIN:
        _fail("SP-5", "common.safety.brake.k", k,
              ">= %s" % _SP5_BRAKE_K_MIN,
              reason="k < 1 makes the brake model non-conservative; "
                     "speed_gate.f() over-estimates authorised speed")
    # brake.a_mps2: must be > 0 and <= max_decel_mps2 (if both present).
    # Two checks in one arm: >0 (physical requirement) and
    # <=max_decel (consistency with spec).
    # Nested `if isinstance...` because both sub-checks depend on
    # `a` being numeric; a single guard at the top saves the two
    # inner checks from repeating the type test.
    if isinstance(a, (int, float)):
        # a_mps2 > 0: physical requirement; a 0 or negative brake
        # deceleration is meaningless (would make d_stop infinite).
        # Zero as a value passes A but fails SP-5 here -- same shape
        # of hazard as SP-1's zero-in-spec.
        if a <= 0:
            # Zero or negative brake deceleration = infinite stopping
            # distance in the model -- catch here before the runtime
            # divides by zero.
            _fail("SP-5", "common.safety.brake.a_mps2", a, "> 0")
        # a_mps2 <= max_decel: consistency with spec. Writing an
        # a_mps2 above spec means the config claims more braking
        # power than the vehicle spec says exists.
        # Skip pair-wise if max_decel absent -- can't compare
        # against nothing.
        if isinstance(max_decel, (int, float)) and a > max_decel:
            # limit includes the spec value so the operator sees
            # both operands of the inequality inline.
            _fail("SP-5", "common.safety.brake.a_mps2", a,
                  "<= spec.max_decel_mps2 = %r" % max_decel)
    # t_lat_s: must be >= 0.4. This is CFG-FZ-7 variant (3) target.
    # t_lat=0.2 low-estimates d(2.0) from 1.60 m to 1.20 m, so
    # speed_gate.f() over-authorises speed AND fence.d_stop under-
    # reports required stopping distance -- both simultaneously
    # push the machine toward faster + closer to boundary.
    # Double-fault mode: writing t_lat=0.2 wrongly makes the machine
    # BOTH faster than sustained-braking allows AND closer to fences
    # than fail-safe would demand. The failure surface is worst-case.
    if isinstance(t_lat, (int, float)) and t_lat < _SP5_T_LAT_MIN_S:
        _fail("SP-5", "common.safety.t_lat_s", t_lat,
              ">= %s" % _SP5_T_LAT_MIN_S,
              reason="t_lat=0.2 low-estimates d(2.0) from 1.60 m to "
                     "1.20 m and over-authorises speed in f()")


def _check_sp11_gpu_throttle(tree: Dict[str, Any]) -> None:
    """SP-11: gateway.gpu_token.throttle_speed_mps < spec.max_vx_mps.

    The throttle is a downstream cap; if it equals or exceeds the
    spec cap, it either has no effect (equal) or contradicts the
    spec (greater), both meaningless.

    Why strict < (not <=): equality is silent no-op. An operator who
    writes throttle == spec probably intended to disable the throttle;
    the correct way is to remove the throttle config key entirely,
    not to set it to the spec value. Strict inequality forces the
    intent to be explicit.

    This is CFG-FZ-7 variant (1) target.
    """
    # Pull both operands; both optional (SP-11 only fires when both
    # are configured -- the throttle is a gateway feature, not a
    # required core parameter).
    # gpu_token is a P4 gateway module; deploys without a GPU-bound
    # LLM don't configure it at all.
    throttle = _get(tree, "common.gateway.gpu_token.throttle_speed_mps")
    max_vx = _get(tree, "common.spec.max_vx_mps")
    if throttle is None or max_vx is None:
        # Either operand missing -> skip. Not a defect here; the
        # throttle is deploy-optional. spec.max_vx would fail M
        # instead if required and missing.
        return
    if not isinstance(throttle, (int, float)) or not isinstance(max_vx, (int, float)):
        # Shape defect -- skip. Type validation is another check's
        # job; SP-11 only cares about the range.
        return
    # Strict >= comparison (violates the strict < contract).
    # SP-11's rule is "strictly less than" so >= (both == and >)
    # is the failure surface.
    if throttle >= max_vx:
        # Report with reason so the operator sees both operands and
        # the rationale in one message.
        # limit text spells out the spec value inline so an operator
        # doesn't have to look it up.
        _fail("SP-11", "common.gateway.gpu_token.throttle_speed_mps",
              throttle, "< spec.max_vx_mps = %r" % max_vx,
              reason="throttle at or above spec is either a no-op or a "
                     "contradiction; the throttle exists to cap BELOW spec")


# AS-7 keys: three timeout knobs, all bounded by 5.0 s.
# One tuple, one bound -- adding a fourth voice subsystem would
# extend this tuple and no other change.
_AS7_KEYS = (
    "common.gateway.asr.timeout_s",
    "common.gateway.llm.timeout_s",
    "common.gateway.tts.timeout_s",
)


def _check_as7_voice_timeouts(tree: Dict[str, Any]) -> None:
    """AS-7: ASR / LLM / TTS timeout_s <= 5.0 s each.

    11 S8.13.1 verbatim upper bound. Voice pipeline latency > 5 s is a
    UX failure -- the user gives up before the response arrives.

    Three keys share the same bound; walk them in order and raise on
    the first that exceeds. Order fixed by _AS7_KEYS declaration
    (asr, llm, tts) for reproducible failure logs.

    This is CFG-FZ-7 variant (2) target.
    """
    # Iterate each key; first over-limit raises.
    # Order fixed by _AS7_KEYS tuple: asr, llm, tts. Reproducible
    # failure log across runs when the same key is misconfigured.
    for key in _AS7_KEYS:
        val = _get(tree, key)
        if val is None:
            # Key optional per subsystem -- if the subsystem isn't
            # configured, its timeout does not need to be either.
            # Fresh deploys may not have configured every voice
            # module yet.
            continue
        if not isinstance(val, (int, float)):
            # Shape defect (e.g. string "5s") -- skip so we don't
            # raise TypeError comparing str > float. Another
            # schema-shape check should report this defect; AS-7's
            # job is only the range, not the type.
            continue
        # Strict > (not >=): equality is OK because 5.0 is exactly
        # the allowed bound.
        if val > _AS7_TIMEOUT_UPPER_S:
            # limit interpolates the actual 5.0 bound so the message
            # is standalone (an operator does not need to look up
            # _AS7_TIMEOUT_UPPER_S).
            _fail("AS-7", key, val, "<= %s" % _AS7_TIMEOUT_UPPER_S,
                  reason="voice-pipeline latency above 5 s is a UX failure")


# ---------------------------------------------------------------------------
# The registry
# ---------------------------------------------------------------------------

# Adding a new SP-N check = one row here + one _check_* function.
# Order runs sequentially; first-fail raises. Alphabetical within a
# rule family so the traversal is reproducible.
# The registry, in traversal order. Order = numeric within SP-*,
# then AS-* last. Traversal is sequential (not parallel) because
# each check is fast and the assertion contract is "one failure at
# a time, name the rule".
_REGISTRY: List[_CheckRow] = [
    # SP-1: five spec keys must be > 0.
    _CheckRow("SP-1", "spec.max_* all > 0", _check_sp1_spec_positive),
    # SP-2: spec covers the fastest profile.
    _CheckRow("SP-2", "max_vx >= max(profiles)", _check_sp2_max_vx_covers_profiles),
    # SP-5: three-way brake + latency invariant.
    _CheckRow("SP-5", "brake.k / a / t_lat_s within bounds",
              _check_sp5_brake_and_latency),
    # SP-11: throttle strictly below spec.
    _CheckRow("SP-11", "gpu_token.throttle < spec.max_vx",
              _check_sp11_gpu_throttle),
    # AS-7: three voice-timeout keys bounded by 5 s each.
    _CheckRow("AS-7", "voice timeouts <= 5 s",
              _check_as7_voice_timeouts),
]


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion G. Replaces registry's stub for G.

    Reuses ctx["overlay"] from A when available; falls back to
    fresh load + build_overlay when called in isolation. Walks
    _REGISTRY in declaration order; first-fail raises.

    Return dict includes both the count of checks run and the list
    of rules so a MANIFEST diff between runs surfaces a change in
    coverage (someone silently dropping a rule from _REGISTRY).
    """
    # Same wiring guard as J / A / M / B / C / D / F.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion G requires ctx['config_root']; caller did not "
            "populate it"
        )
    # Prefer A's cached overlay; fall back for isolated callers.
    # Same pattern as C / D / F.
    overlay = ctx.get("overlay")
    if overlay is None:
        # Fresh load path -- for isolated unit tests that skip A.
        layer_trees = load_layers(ctx["config_root"])
        overlay = build_overlay(layer_trees)
        # Populate ctx so downstream assertions in the same pass
        # do not re-load.
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees
    tree = overlay.tree

    # Walk the registry; first-fail raises. Sequential (not parallel)
    # because each check is fast and the assertion contract is "one
    # failure at a time, name the rule". A parallel walk would report
    # multiple violations concurrently, and G's failure surface is
    # designed to point at ONE thing to fix.
    # Row order = _REGISTRY declaration order (SP-1, SP-2, SP-5,
    # SP-11, AS-7) for reproducible triage.
    for row in _REGISTRY:
        # row.check raises XbrainError on failure; success is silent.
        # No collection of results -- assertion contract says one
        # raise at a time.
        row.check(tree)

    # Success return: count + rule list. rules list is stable-sorted
    # because _REGISTRY iteration is declaration-ordered.
    # Reporting rules explicitly means a future edit that dropped
    # SP-5 (say) surfaces as a MANIFEST diff between runs -- silent
    # coverage shrink is impossible.
    return {
        "status": "pass",
        "assertion": "G",
        "checks_run": len(_REGISTRY),
        "rules": [row.rule for row in _REGISTRY],
    }
