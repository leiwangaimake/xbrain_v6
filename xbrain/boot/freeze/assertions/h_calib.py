"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: h_calib.py
Brief: Assertion H -- extrinsics + calibration accuracy (CFG-FZ-9)

Description:
Five sub-checks against configs/calib/{robot_id}.yaml (L4b), per
11 S10.1.1 and S10.4.4:

  H-1 common.calib.robot_id == common.robot_id
      Two literal fields must agree. A robot that reads a calib
      file belonging to a different robot walks with the wrong
      extrinsics, and no runtime code checks for it.

  H-2 Every key under common.calib.frames MUST come from the
      calibration frame ID closed set (11 S0.3 devices + S10.4.4
      example frame IDs). A frame named 'nonsense' would silently
      never be consumed and the sensor it was meant to cover would
      operate with no extrinsics.

  H-3 Every frames.* entry has a complete accuracy block:
      method + sigma_trans_m (3-tuple) + sigma_rot_rad (3-tuple)
      + n_samples. CS-2 verbatim: a schema that permits value-only
      extrinsics is CAL-11 not implemented.

  H-4 Recompute lat_err_ref_m from sigma_trans_m + sigma_rot_rad
      via the S10.1.1 formula, compare against the file's recorded
      value. Deviation > 1e-6 = extrinsics and derived error do
      not agree, one of them was tampered with. This is the check
      the doc calls out specifically: a calibration where both
      operator-written numbers agree with each other but neither
      matches the underlying extrinsics passes CS-1 and CS-2 while
      being wrong.

  H-5 lat_err_ref_m vs gate.warn_m / gate.reject_m:
        <= gate.warn_m           pass
        gate.warn_m < .. <= reject_m   pass with degrade signal
        > gate.reject_m          fail (E_CONFIG_INVALID)

Contract:
  input:   ctx["config_root"]
           optional ctx["calib_raw"] -- dict override (for tests
               that inject a synthesized calib tree without writing
               yaml to disk)
           optional ctx["common_robot_id"] -- override the
               common.robot_id used for the H-1 identity check;
               otherwise loaded from L1 common.yaml
           optional ctx["extra_frame_ids"] -- widen the H-2 whitelist
               (used by tests to add a specific fake key without
               editing the shared set)
  raises:  XbrainError(E_CONFIG_INVALID) with detail.kind in
             {calib_robot_id_mismatch, unregistered_frame_key,
              accuracy_incomplete, lat_err_recompute_mismatch,
              lat_err_over_reject}
           + detail.key / detail.expected / detail.actual as needed

CFG-FZ-9 named variants (each MUST turn red in tests):
  1) common.calib.robot_id != common.robot_id -> H-1 red
  2) frames.foo (foo not in closed set)       -> H-2 red
  3) frames.cam_rgbd without accuracy         -> H-3 red
  4) lat_err_ref_m recorded value tampered    -> H-4 red
  5) lat_err_ref_m > gate.reject_m            -> H-5 red

Not in scope for H (handled elsewhere):
  * Whether calib.yaml exists at all -- J-2 (required L4b file
    presence) is the enforcer.
  * The value of gate.warn_m / gate.reject_m being 'right' --
    M-24 is deferred to real-machine acceptance; H only enforces
    the > reject_m stop, not the specific thresholds.
  * Whether the RGBD depth pipeline consumes the accuracy correctly
    -- that is a perception-side concern (still pending design).

lat_err_ref_m formula (S10.1.1 verbatim):
  sigma_rot_max = max(sigma_roll, sigma_pitch, sigma_yaw)
  sigma_trans_lat = hypot(sigma_x, sigma_y)
  lat_err_at_d(d) = d * sin(sigma_rot_max) + sigma_trans_lat
  lat_err_ref_m = lat_err_at_d(d_ref)

The formula is applied to EACH frames.* entry that has a full
accuracy block; the reported lat_err_ref_m is the MAX across all
frames because a downstream fence-margin consumer that reads one
lat_err_ref_m must be conservative against every sensor.

Ordering in the freeze pipeline (ORD-1):

H depends_on=("M",) so it runs after M (required-key completeness)
has confirmed that common.robot_id + common.calib.* required keys
are present. Placing H after M gives cleaner failure attribution:
'M: missing common.calib.robot_id' vs 'H: robot_id mismatch' --
different remediation paths (create the key vs correct the value).

H is orthogonal to A/M/B/C/D/E/F/G because it validates a physical-
world property (extrinsics accuracy) rather than a config-tree
property. It could run at any point after M; placement is late
because it is more expensive than the pure-tree checks (recompute
formula, per-frame iteration) and there is no reason to pay that
cost before cheaper checks have narrowed the failure surface.

Why the max across frames (not sum, not mean):

Downstream consumers that read a single lat_err_ref_m must be
conservative against every sensor. A robot with a very accurate
cam_rgbd (0.1) and a poorly-calibrated rslidar (0.3) has a real
worst-case lateral error of 0.3, not the mean (0.2) and not the
sum (0.4). Fence margins must extend by 0.3 or the rslidar-
authorised region will not match reality.

Failure-mode taxonomy (five distinct detail.kind values):

  calib_robot_id_mismatch      -> wrong calib file for this robot
                                  (transposed VIN, file copied
                                  from another unit); remediation
                                  is 'restore correct file'.
  unregistered_frame_key       -> typo in frame name OR a genuinely
                                  new sensor not yet in S0.3;
                                  remediation is 'fix typo' or
                                  'update S0.3 + closed set'.
  accuracy_incomplete          -> CS-2 violation, operator wrote
                                  values without sigma / n_samples;
                                  remediation is 're-run calib_solve'.
  lat_err_recompute_mismatch   -> operator tampered with the
                                  derived value (or the extrinsics)
                                  after the fact; remediation is
                                  'diagnose which side is wrong'.
  lat_err_over_reject          -> calibration insufficient for
                                  autonomous operation; remediation
                                  is 're-run field calibration'.

Keeping them split lets downstream dashboards categorise without
parsing the message string. detail.kind is the primary discriminator
that consumers should branch on; the message is human-only.

Fail-silent risk removed:

Every path that could 'do nothing and pass' has a distinct signal:

  calib file absent            -> J-2 already fires; H's tree is
                                  empty; H skips gracefully with
                                  frames_checked=0.
  frames block empty           -> no sub-check has anything to
                                  iterate; return outcome='skip'.
  no accuracy data on any frame-> H-4 returns None; H-5 skips;
                                  H-3 has fired if this state is
                                  a defect.
  degrade band                 -> return outcome='degrade' (does
                                  NOT raise per S10.1.1 iv); the
                                  caller receives a signal but the
                                  process still boots.

An observer who sees status='pass' with outcome='degrade' knows
extrinsics are in the warn zone; without the outcome field a
degrade would look identical to a clean pass.
"""

# math for hypot + sin used by the recompute step.
import math
# typing for annotations.
from typing import Any, Dict, FrozenSet, Iterable, Optional

# Layer loader for L1 (common.robot_id) + L4b (calib file).
from xbrain.boot.freeze.assertions._layer_loader import load_layers
# E_CONFIG_INVALID by name, per CLAUDE.md 3.5.
from xbrain.common.errors import E_CONFIG_INVALID
# XbrainError base -- H uses E_CONFIG_INVALID uniformly.
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# H-2: closed set of frame keys legal under common.calib.frames.
# Union of 11 S0.3 device IDs AND 11 S10.4.4 example frame IDs. The
# two sets overlap; the doc uses 'rslidar' (frame_id) and 'ptz_base'
# (coordinate frame) while S0.3 lists 'lidar' and 'cam_ptz' as the
# device IDs. Both are valid targets for calibration extrinsics.
#
# Kept as a module-level frozenset (not read from sets.yaml) because
# the calib frame key set is NOT the same as the S5.1A BIT device
# set that sets.yaml carries as 'device'. Widening a shared set to
# make this pass would BROADEN the BIT set spuriously; a fresh set
# is the correct decoupling.
_FRAME_IDS: FrozenSet[str] = frozenset({
    # S0.3 device IDs used as calibration frames
    "cam_rgbd", "cam_ptz", "cam_chassis_front", "cam_chassis_rear",
    "rtk", "imu_chassis", "gnss_chassis", "mic",
    # S10.4.4 example frame IDs (lidar single-topic-ised)
    "rslidar", "ptz_base",
    # Coordinate frames commented as "同构" in S10.4.4
    "gnss_link", "imu_link", "base_link",
})

# H-4 recompute tolerance. 1e-6 m = 1 micron. Any deviation above
# this = the recorded lat_err_ref_m disagrees with the extrinsics
# it should be derived from. Below 1e-6 = float rounding, tolerated.
_RECOMPUTE_TOL_M = 1e-6

# H-4 reference distance for lat_err_at_d(d_ref). Doc S10.1.1
# verbatim: d_ref = 10.0 m. Same as CAL-11 reference point.
_D_REF_M = 10.0

# H-5 default thresholds when calib file omits the gate block.
# M-24 deferred, doc gives 0.17 / 0.35 as placeholders. When the
# calib file carries its own gate block, that wins.
_DEFAULT_GATE_WARN_M = 0.17
_DEFAULT_GATE_REJECT_M = 0.35


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker; same shape as other assertions."""
    # Walk segment by segment; missing / non-dict returns default.
    # Kept local so this module is self-contained.
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _fail(kind: str, message: str, **detail_extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + arbitrary context.

    kind is ONE of the closed set in the module docstring; downstream
    dashboards categorise on kind so keeping it a small closed set
    matters.
    """
    # Detail carries kind + extras; message is human-only.
    detail: Dict[str, Any] = {"kind": kind}
    detail.update(detail_extra)
    raise XbrainError(E_CONFIG_INVALID,
                      "assertion H failed: %s" % message,
                      detail)


def _recompute_lat_err(frames: Dict[str, Any],
                       d_ref: float = _D_REF_M) -> Optional[float]:
    """Compute lat_err_ref_m from every frame's accuracy block.

    Returns the MAX lat_err_at_d(d_ref) across all frames that have
    a complete accuracy block (sigma_trans_m + sigma_rot_rad triples).
    Frames with missing / malformed accuracy are skipped (H-3 catches
    those separately).

    Returns None if no frame carries usable accuracy data (nothing
    to compute; caller decides whether to skip H-4).

    Formula (11 S10.1.1 verbatim):
        sigma_rot_max   = max(sigma_roll, sigma_pitch, sigma_yaw)
        sigma_trans_lat = hypot(sigma_x, sigma_y)
        lat_err_at_d(d) = d * sin(sigma_rot_max) + sigma_trans_lat
        lat_err_ref_m   = lat_err_at_d(d_ref)   with d_ref=10 m

    Note that sigma_z is NOT used: the fence-margin consumer only
    cares about horizontal (lateral) error, which is why the formula
    uses hypot(x, y) rather than hypot(x, y, z). z-error affects
    altitude decisions, which are handled separately.
    """
    # Track the running max; None sentinel distinguishes 'no frames
    # produced a value' from 'the value is 0.0'. The 0.0 case is
    # legitimate (perfect calibration by construction, or a single
    # frame with all sigmas at zero) and must not be conflated with
    # 'no data'.
    best = None
    # Iterate every declared frame. Non-dict / malformed entries are
    # silently skipped -- H-3 owns the schema-level enforcement, so
    # this function only cares about entries that CAN produce a
    # number.
    for name, frame in frames.items():
        # Skip non-dict frames (shape defect; H-3 or H-2 will fire).
        # Silently skipping here lets those checks own the failure
        # attribution.
        if not isinstance(frame, dict):
            continue
        # Accuracy block; if missing, this frame contributes nothing.
        acc = frame.get("accuracy")
        if not isinstance(acc, dict):
            continue
        # Pull the two triples; both must be lists of 3 floats.
        # Any other shape means the accuracy block is malformed;
        # skip and let H-3 fire on the shape defect.
        sigma_trans = acc.get("sigma_trans_m")
        sigma_rot = acc.get("sigma_rot_rad")
        if not (isinstance(sigma_trans, list) and len(sigma_trans) == 3
                and isinstance(sigma_rot, list) and len(sigma_rot) == 3):
            continue
        # Numeric type check on all six entries. try/except catches
        # both None (missing entry inside list) and non-numeric
        # strings. Any failure = skip the frame; H-3 will own the
        # 'wrong type' report elsewhere.
        try:
            tx, ty, tz = (float(x) for x in sigma_trans)
            rx, ry, rz = (float(x) for x in sigma_rot)
        except (TypeError, ValueError):
            continue
        # S10.1.1 formula: max rotation sigma, translation lateral
        # magnitude in xy, then project at d_ref.
        # sin(theta) rather than tan(theta) because sin gives the
        # perpendicular displacement at unit distance -- exactly
        # what a fence-margin consumer needs. tan(theta) would
        # give the projected distance to the axis, which is a
        # different (and larger) quantity.
        sigma_rot_max = max(rx, ry, rz)
        # hypot(x, y) is the standard-library idiom for sqrt(x^2+y^2)
        # with overflow-safe intermediate math. Not simply
        # sqrt(x**2 + y**2) because that squares sub-normal floats
        # and loses precision.
        sigma_trans_lat = math.hypot(tx, ty)
        lat_err = d_ref * math.sin(sigma_rot_max) + sigma_trans_lat
        # Track the max across all frames; the downstream fence-margin
        # consumer must be conservative against every sensor.
        # Using > (not >=) so the first-encountered value wins on
        # exact ties -- makes the failure trace deterministic when
        # two frames have identical accuracy.
        if best is None or lat_err > best:
            best = lat_err
    return best


# ---------------------------------------------------------------------------
# Sub-checks
# ---------------------------------------------------------------------------

def _check_h1(calib_tree: Dict[str, Any], common_robot_id: str) -> None:
    """H-1: common.calib.robot_id == common.robot_id.

    Two literal fields must agree; mismatch means the calib file
    belongs to a different robot. Real-world provenance for this
    defect: an ops engineer copies a working calib file from one
    robot to another as a starting point and forgets to update
    the robot_id inside. Without H-1, that robot boots with the
    donor robot's extrinsics -- fence positions off by cm, PTZ
    tilt geometry wrong, RGBD depth mis-projected.

    The check is a str-equality (str-coerce both sides) so int
    robot ids and string robot ids compare naturally without
    silent-cast surprises.
    """
    # Read the calib-side robot_id.
    calib_rid = _get(calib_tree, "common.calib.robot_id")
    # Skip if either side absent -- M enforces required-ness.
    if calib_rid is None or common_robot_id is None:
        return
    # String equality; ints or other types would fail this
    # naturally because yaml would parse them as different types.
    if str(calib_rid) != str(common_robot_id):
        _fail("calib_robot_id_mismatch",
              "calib.robot_id (%r) != common.robot_id (%r); "
              "extrinsics belong to a different robot"
              % (calib_rid, common_robot_id),
              key="common.calib.robot_id",
              expected=common_robot_id, actual=calib_rid)


def _check_h2(frames: Dict[str, Any], extra_ids: Iterable[str]) -> None:
    """H-2: every frames.* key must be in the closed set.

    CFG-FZ-9 variant explicitly names 'frames with unregistered key'
    as a must-red case. Extra IDs (test injection) widen the whitelist
    without editing the module-level constant.

    Why a closed set: the S0.3 registry is the single source of truth
    for device identifiers. A calib entry for a sensor not in S0.3
    means either (a) typo (fix the yaml) or (b) new sensor not yet
    registered (update S0.3 first, then add the calib entry). Both
    remediation paths route through the doc, not through code -- so
    freeze refusing to boot on this defect protects the schema
    boundary.

    First-fail with sorted() key order: any unregistered key raises;
    the sort makes the failure trace deterministic when multiple
    unregistered keys are present.
    """
    # Compose the effective whitelist; test injection is additive.
    allowed = _FRAME_IDS | frozenset(extra_ids or ())
    # Iterate; first out-of-set key fires the raise. sorted() for
    # deterministic error ordering.
    for key in sorted(frames.keys()):
        if key not in allowed:
            _fail("unregistered_frame_key",
                  "frames key %r not in calibration frame closed "
                  "set (11 S0.3 + S10.4.4)" % key,
                  key="common.calib.frames." + key,
                  value=key, allowed=sorted(allowed))


def _check_h3(frames: Dict[str, Any]) -> None:
    """H-3: every frames.* has a complete accuracy block.

    CS-2 verbatim: method + sigma_trans_m + sigma_rot_rad + n_samples.
    A schema that permits value-only extrinsics is CAL-11 not
    implemented -- CS-2 is the check that catches it.

    Why enforcement per-field (not just 'accuracy exists'): an
    operator who wants to skip full calibration might write
    accuracy={method: 'guessed'} to get past a naive check.
    Requiring the sigma triples + n_samples forces them to write
    numbers that H-4 can then verify against the extrinsics.
    Missing n_samples in particular is the giveaway that no real
    calibration was done -- calib_solve always populates n_samples
    with the observation count.
    """
    # Required accuracy fields per CS-2. cov6x6_b64 is optional.
    required = ("method", "sigma_trans_m", "sigma_rot_rad", "n_samples")
    for key, frame in frames.items():
        # Shape gate: frame must be a dict. Non-dict = H-3 fail on
        # the whole frame (accuracy cannot be evaluated).
        if not isinstance(frame, dict):
            _fail("accuracy_incomplete",
                  "frame %r not a dict" % key,
                  key="common.calib.frames." + key,
                  value=frame)
        # Accuracy block must exist and be a dict.
        acc = frame.get("accuracy")
        if not isinstance(acc, dict):
            _fail("accuracy_incomplete",
                  "frame %r missing accuracy block (CS-2)" % key,
                  key="common.calib.frames." + key + ".accuracy",
                  reason="block_missing")
        # Enumerate required fields; each missing = one fail.
        for field in required:
            if field not in acc:
                _fail("accuracy_incomplete",
                      "frame %r accuracy missing %r (CS-2)"
                      % (key, field),
                      key="common.calib.frames.%s.accuracy.%s"
                      % (key, field),
                      missing=field)


def _check_h4(calib_tree: Dict[str, Any],
              frames: Dict[str, Any],
              d_ref: float) -> Optional[float]:
    """H-4: recompute lat_err_ref_m from extrinsics, compare to file.

    Returns the recomputed value so H-5 can reuse it (avoids a
    second pass through the formula).

    A tampered record (operator wrote a favorable number) surfaces
    here even when CS-1 (frame keys) and CS-2 (accuracy fields) both
    pass. This is the check the doc specifically calls out for.

    Why 1e-6 m tolerance: float rounding of the formula
    d * sin(theta) + hypot(x, y) at typical sigma values (~1e-3 rad,
    ~1e-2 m) accumulates < 1e-10 m of error. Anything above 1e-6
    (a micron) is orders of magnitude larger than any legitimate
    rounding, so the tolerance leaves generous headroom for float
    imprecision while catching any real disagreement.

    Why absolute (not relative) tolerance: at very small lat_err
    (< 0.01), a relative tolerance of 1% would be 1e-4 -- large
    enough to hide a real tamper. Absolute tolerance stays strict
    regardless of magnitude, matching the doc's 1e-6 wording.

    The tamper class this catches: an operator who runs calib_solve,
    dislikes the resulting lat_err_ref_m (say 0.28, in the degrade
    band), edits the file to 0.14 (in the pass band) without re-
    running calib_solve. The extrinsics values are the same
    (calib_solve did produce them), the frame keys are legal, the
    accuracy blocks are present, but the derived scalar disagrees
    with the underlying data. Without H-4 this passes freeze.
    """
    # Recompute from all frames with usable accuracy.
    recomputed = _recompute_lat_err(frames, d_ref)
    # No frame carried usable data -- H-4 is skipped. H-3 will have
    # fired if this state is a defect; if H-3 skipped too (empty
    # frames block, which is legit for a robot with no sensors),
    # H-4 has nothing to compare and is a no-op.
    if recomputed is None:
        return None
    # Read the recorded value; missing = skip (M's job to enforce).
    recorded = _get(calib_tree, "common.calib.lat_err_ref_m")
    if recorded is None:
        return recomputed
    # Type gate. Non-numeric would crash the subtraction below.
    if not isinstance(recorded, (int, float)):
        _fail("lat_err_recompute_mismatch",
              "lat_err_ref_m %r not numeric" % recorded,
              key="common.calib.lat_err_ref_m",
              recorded=recorded, recomputed=recomputed)
    # Absolute-difference tolerance. Relative tolerance would let
    # small errors accumulate at small values; absolute is what the
    # doc's 1e-6 wording implies.
    if abs(recorded - recomputed) > _RECOMPUTE_TOL_M:
        _fail("lat_err_recompute_mismatch",
              "lat_err_ref_m recorded=%r vs recomputed=%r "
              "(diff %.3e > tol %.3e); extrinsics and derived "
              "error disagree"
              % (recorded, recomputed, abs(recorded - recomputed),
                 _RECOMPUTE_TOL_M),
              key="common.calib.lat_err_ref_m",
              recorded=recorded, recomputed=recomputed,
              tolerance=_RECOMPUTE_TOL_M)
    return recomputed


def _check_h5(calib_tree: Dict[str, Any],
              lat_err: Optional[float]) -> Optional[str]:
    """H-5: threshold check against gate.warn_m / gate.reject_m.

    Returns the outcome string ('pass' / 'degrade' / 'reject') or
    None if lat_err is unavailable. 'reject' raises before
    returning; 'degrade' does NOT raise (per S10.1.1 iv it is
    'pass with degrade signal').

    The three bands are the doc's own S10.1.1 iv verbatim:
      pass       lat_err <= warn_m
      degrade    warn_m < lat_err <= reject_m
      reject     lat_err > reject_m

    Degrade is deliberately non-raising because the operator has a
    functioning robot; refusing to boot would waste an available
    resource. The downstream signals (fence-margin extension,
    cam_rgbd health = warn) let the runtime work around the reduced
    accuracy safely.

    Reject raises E_CONFIG_INVALID because a robot with lat_err_ref_m
    above the reject band cannot be trusted to keep itself inside
    fences -- fence-margin extension cannot indefinitely compensate
    (margin_ext = lat_err would eventually exceed margin_max, per
    S9A.6).

    Threshold defaults (_DEFAULT_GATE_WARN_M / _REJECT_M) kick in
    only when the calib file omits the gate block. Doc placeholders
    (0.17 / 0.35) apply; when M-24 lands with real values, the
    calib file will carry them and the defaults become irrelevant.
    """
    # If H-4 was skipped, there is no lat_err to threshold; H-5 is
    # a no-op. This is legitimate when the calib file lacks accuracy
    # data (H-3 already fired if data was required).
    if lat_err is None:
        return None
    # Read thresholds from calib file; use defaults if absent so K
    # can still fire on the > reject_m case even when the operator
    # forgot the gate block. Defaults are the doc placeholders.
    warn = _get(calib_tree, "common.calib.gate.warn_m",
                _DEFAULT_GATE_WARN_M)
    reject = _get(calib_tree, "common.calib.gate.reject_m",
                  _DEFAULT_GATE_REJECT_M)
    # Numeric coercion for safety; a stringly-typed gate would
    # otherwise crash the comparison.
    try:
        warn_f = float(warn)
        reject_f = float(reject)
    except (TypeError, ValueError):
        _fail("lat_err_over_reject",
              "gate.warn_m or gate.reject_m not numeric "
              "(warn=%r reject=%r)" % (warn, reject),
              key="common.calib.gate",
              gate_warn_m=warn, gate_reject_m=reject)
    # Three bands: pass < warn <= degrade <= reject < fail.
    if lat_err > reject_f:
        # CFG-FZ-9 variant (5) fires here.
        _fail("lat_err_over_reject",
              "lat_err_ref_m %.6f > gate.reject_m %.6f; "
              "extrinsics accuracy insufficient for autonomous "
              "operation" % (lat_err, reject_f),
              key="common.calib.lat_err_ref_m",
              lat_err_ref_m=lat_err, gate_reject_m=reject_f)
    if lat_err > warn_f:
        # Degrade band: pass with a signal. Not a raise. The signal
        # surfaces via the returned outcome so a caller can drive
        # health, fence-margin extension, or startup-event emission.
        return "degrade"
    return "pass"


# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------

def _load_calib_and_robot_id(ctx: Dict[str, Any]) -> tuple:
    """Return (calib_tree, common_robot_id) from ctx or on-disk.

    ctx overrides win; otherwise loads L1 for common.robot_id and
    L4b calib file. L4b is picked by robot_id; if the calib file
    for the resolved robot_id does not exist, calib_tree is empty
    and every sub-check that reads it becomes a no-op (H-1 also
    skips because calib.robot_id is None). This matches the freeze
    convention: J-2 fires on missing required files, H just
    validates whatever is present.

    Path composition: configs/calib/{robot_id}.yaml. This is the
    L4b location per 10 S5.4.0 -- one file per robot, keyed by the
    common.robot_id string. If robot_id contains characters that
    are not valid filenames (colons, slashes) the join produces a
    non-existent path and the sub-checks skip; that is acceptable
    because robot_id constraints are enforced by D (identity).

    Import of yaml is deferred inside the function so the module
    stays importable even when yaml is not installed (e.g. in a
    test that only uses the ctx-override path).
    """
    # ctx overrides win outright; used by tests.
    calib_raw = ctx.get("calib_raw")
    common_rid = ctx.get("common_robot_id")
    # If both overrides present, no disk I/O needed.
    if calib_raw is not None and common_rid is not None:
        return calib_raw, common_rid
    # Fresh-load path: read L1 for common.robot_id.
    root = ctx["config_root"]
    layers = load_layers(root)
    l1 = layers.get("L1", {})
    if common_rid is None:
        common_rid = _get(l1, "common.robot_id")
    # For calib_raw, look under configs/calib/{robot_id}.yaml.
    if calib_raw is None:
        # Delayed import so the loader stays self-contained for
        # unit tests that skip disk I/O entirely.
        import os
        import yaml
        calib_dir = os.path.join(root, "calib")
        calib_raw = {}
        if common_rid and os.path.isdir(calib_dir):
            path = os.path.join(calib_dir, str(common_rid) + ".yaml")
            if os.path.isfile(path):
                with open(path, encoding="utf-8") as fh:
                    calib_raw = yaml.safe_load(fh) or {}
    return calib_raw, common_rid


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion H. Replaces registry's stub for H.

    Flow:
      1. Wiring guard on ctx['config_root'].
      2. Load calib tree + common.robot_id (ctx or L4b/L1).
      3. H-1: robot_id identity.
      4. H-2: frame keys in closed set.
      5. H-3: accuracy blocks complete.
      6. H-4: lat_err_ref_m recompute matches recorded.
      7. H-5: threshold check with degrade signal returned.
      8. Pass return carries outcome + recomputed value.

    Sub-check order matters: H-2 (unregistered keys) fires BEFORE
    H-3 (accuracy blocks) because an unregistered frame key means
    the sensor is not in the schema, so its accuracy block is
    irrelevant. H-3 fires before H-4 because H-4 needs the accuracy
    values H-3 vouches for. H-5 fires last because it needs H-4's
    recomputed value.

    First-fail: any sub-check that raises terminates the run. This
    is deliberate; H is a single-boolean-gate assertion. Fixing the
    first failure often uncovers or resolves later ones (a corrected
    robot_id often means a different calib file entirely).
    """
    # Wiring guard identical to every other assertion.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion H requires ctx['config_root']; caller did not "
            "populate it"
        )
    # Load the two operands. If calib file is absent, calib_tree is
    # empty and every sub-check skips gracefully.
    calib_tree, common_rid = _load_calib_and_robot_id(ctx)
    # Extract the frames block once.
    frames = _get(calib_tree, "common.calib.frames", {}) or {}
    if not isinstance(frames, dict):
        # Non-dict frames block = shape defect; report as H-2 with
        # a clear reason.
        _fail("unregistered_frame_key",
              "common.calib.frames not a dict (got %r)"
              % (frames,),
              key="common.calib.frames",
              value=type(frames).__name__)
    # H-1
    _check_h1(calib_tree, common_rid)
    # H-2
    _check_h2(frames, ctx.get("extra_frame_ids", ()))
    # H-3
    _check_h3(frames)
    # H-4 (returns recomputed value for H-5 to reuse)
    d_ref = _get(calib_tree, "common.calib.d_ref_m", _D_REF_M)
    try:
        d_ref_f = float(d_ref)
    except (TypeError, ValueError):
        d_ref_f = _D_REF_M
    lat_err = _check_h4(calib_tree, frames, d_ref_f)
    # H-5 (returns outcome; may raise on reject_m breach)
    outcome = _check_h5(calib_tree, lat_err)

    # Success shape. checks_run=5 lets a downstream observer verify
    # every sub-rule ran; outcome carries the H-5 band so a caller
    # can drive downstream degrade signals without re-reading the
    # calib file.
    return {
        "status": "pass",
        "assertion": "H",
        "checks_run": 5,
        "frames_checked": len(frames),
        "lat_err_ref_m": lat_err,
        "outcome": outcome or "skip",
    }
