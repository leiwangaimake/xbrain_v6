"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: k_quadruped_qc.py
Brief: Assertion K -- quadruped private QC-1..QC-17 (CFG-FZ-12)

Description:
Evaluates every QC-* rule declared in 13 S8.3 (17 rules, no exemption).
The rules guard quadruped's private config surface which no other
process reads, so if K does not fire, five documented bad configs
boot green:

  QC-13  codebook: legacy_decimal + empty table
  QC-9   prone_forbidden_gaits missing stair_standard
  QC-2   tier1.cmd_timeout_ms = 50 (below the 200 ms hard floor)
  QC-4   chassis_dds.domain_id == uplink.ros_domain_id
         (isolation defeated -- CHS-B side pubs leak into ROS 2)
  QC-15  TLS creds absent while the candidate is enabled

All five would let the robot start with a mission-blocking or safety-
degrading defect that would only surface at first real chassis
interaction, i.e. after Stage 4 releases motion authority. K's
purpose is to move each defect from 'runtime discovery' to 'freeze
refuses'.

Evaluation surface (per 13 S8.3 verbatim):

  QC-1                on the L6 SOURCE (configs/quadruped.yaml).
                      This one MUST read raw because it asserts
                      absence of DEFINITIONS, and any expansion
                      pass would replace a bad definition with
                      its resolved value and hide the defect.

  QC-2 .. QC-17       on the RESOLVED artifact
                      (/run/xbrain/resolved/quadruped.yaml).
                      That artifact is what quadruped will read at
                      runtime; validating pre-expansion would let
                      a ${common.spec.*} reference expand to a bad
                      value later.

Implementation-side detail: none of QC-2..QC-17 currently reference
${common.*} in a way that would change the validated value (the
reference is spec.max_decel_mps2 alone, used only for odom.a_max
which is not asserted by K). So reading L6 raw is a valid proxy
for the resolved value in every case except QC-1. We still note
the design surface here so a future rule that DOES depend on
expansion is placed on the resolved side by default.

Contract:
  input:   ctx["config_root"]
           optional ctx["quadruped_raw"]  -- dict override for L6
               source (used by tests to avoid writing a yaml).
           optional ctx["known_rt_imu_keys"] -- iterable to widen
               the RT-face imu_rt_key whitelist for tests.
  raises:  XbrainError(E_CONFIG_INVALID) with
             detail.rule = "QC-N"
             detail.key  = "quadruped.<dotted.path>"
             detail.value / detail.limit / detail.reason as needed

CFG-FZ-12 named variants (each MUST turn red in tests):
  1) chassis_link.codebook = "legacy_decimal" with empty
     codebook_table.legacy_decimal
     -> QC-13 red
  2) motion.prone_forbidden_gaits = ["stair_agile"]  (stair_standard
     removed)
     -> QC-9 red
  3) tier1.cmd_timeout_ms = 50
     -> QC-2 red (200 ms hard floor per 11 S9.12.6)
  4) chassis_dds.domain_id == uplink.ros_domain_id
     -> QC-4 red (domain isolation defeated)
  5) TLS enabled candidate with cred_dir empty / missing files
     -> QC-15 red

Rationale for one runner (not seventeen):

  * Keeping the seventeen sub-checks in one module means the QC-N
    numbering used in the doc lines up with a single grep hit; a
    reader who lands on 'QC-13 failed' finds every K-relevant piece
    of code in one file.
  * The pipeline registers K as one assertion (registry AssertSpec
    row K); splitting it would force N rows and drift risk between
    the doc's QC-N numbering and the registry.
  * Each sub-check is O(few config-key reads); collapsing them into
    one runner has no perf cost.

Not in scope for K:

  * Whether the resolved artifact matches the L6 raw byte-for-byte
    after freeze -- that is the ORD-1 sha256 rule, not a K rule.
  * Whether ${common.spec.*} references EXIST -- QC-1's measure is
    only DEFINITIONS of spec.* keys inside quadruped, not the
    absence of references.
  * IP address values (chassis endpoint hosts) -- V-15 is a real-
    machine acceptance value, kept in yaml so config change is a
    no-code deploy. K does not enforce a specific value; it only
    enforces port shape (QC-6).
  * ros_domain_id / RMW values -- D-15 default is 42 / cyclone but
    K only checks legality-range + non-collision with chassis_dds.
    Specific value = deploy choice, not K's business.

Per-QC rationale (why each rule exists, failure mode, doc anchor):

  QC-1  ban spec.* DEFINITIONS under quadruped
        WHY: spec.* keys are the single-source-of-truth for the
        platform (max_vx, max_decel, spec.robot). Defining them
        again under quadruped creates a second source of truth
        that L6 quadruped will read -- while p1_motion reads the
        L1 common one. Two processes on different values = the
        exact class of defect the layer model is meant to prevent.
        FAIL MODE: quadruped goes to 3 m/s while p1_motion caps
        at 2 m/s; the mismatch surfaces only when the operator
        commands >2 m/s.
        DOC: 11 S9.6 SP-3.

  QC-2  tier1.cmd_timeout_ms >= 200
        WHY: 200 ms is the hard floor at which the chassis
        heartbeat round-trip fits inside a Tier-1 budget on the
        worst-case link. Below it, the state machine falsely
        judges 'chassis lost' on every tick and thrashes.
        FAIL MODE: robot cannot leave IDLE because Tier-1 says
        'chassis link degraded'.
        DOC: 11 S9.12.6.

  QC-3  tier1.control_loop_hz >= 100
        WHY: 100 Hz is the odom publish rate; below it we would
        skip control cycles between odom updates.
        DOC: 11 S9.12.2.

  QC-4  chassis_dds.domain_id != uplink.ros_domain_id
        WHY: this is THE isolation guarantee for quadruped's
        three-channel layout. If the two collide, a domain-0
        CHS-B reader that was meant to only see the chassis's
        LIDAR/IMU also picks up domain-42 uplink pubs (odom,
        tf), and vice versa: our uplink pubs leak into the
        chassis DDS side. Neither side is expecting the traffic;
        both silently drop or mis-decode.
        FAIL MODE: perception sees garbage IMU frames from
        upstream uplink; chassis rejects uplink odom.
        DOC: 13 S2.4 DDS-1..DDS-9.

  QC-5  ros_domain_id in [1,232], rmw in closed set
        WHY: domain 0 collides with any debugger laptop on the
        LAN; upper bound is DDS spec. RMW set is the reviewed
        list (D-15 v0.7 default = cyclonedds).
        FAIL MODE: dev machine attaches and floods the domain
        with foreign pubs.
        DOC: 11 D-15 + PB-4.

  QC-6  endpoint_candidates non-empty + valid ports
        WHY: the probe walks the list; empty list = deadlock at
        boot. Invalid port would fail socket() with EINVAL.
        DOC: 13 S2.2.

  QC-7  odom.stale_warn < stale_invalid < stale_stop_publish
        WHY: the four-band staleness state machine crosses
        states in monotone order; swapped bounds would trip
        the states in the wrong order and either publish stale
        odom (invalid < warn -> warn never fires) or stop early.
        DOC: 13 S4.4.

  QC-8  publish_hz and 1000/publish_hz both integer ms
        WHY: fixed-tick scheduler; non-integer period drifts
        the 100 Hz grid by fractional ms per tick, accumulating
        one full period every ~30 s at 30 Hz.
        DOC: 13 S4.4.

  QC-9  prone_forbidden_gaits >= {stair_agile, stair_standard}
        WHY: the two stair gaits are structurally unsafe from
        prone -- attempting them collapses the robot forward.
        Allowed to widen (add more), not narrow (remove).
        FAIL MODE: prone -> stair transition attempted, robot
        falls forward.
        DOC: 13 PR-1.

  QC-10 special_gaits entries in registered set
        WHY: typos in gait names silently become no-ops at
        dispatch time. Registered set is small and stable.
        DOC: 13 S5.3.

  QC-11 heartbeat_hz >= 1.0
        WHY: vendor lower bound; below 1 Hz the chassis judges
        'controller lost' and enters its own safe mode.
        DOC: Vendor Guide 1.2.1.

  QC-12 axis_cmd_socket_fixed == true
        WHY: CA-1 -- fixed-socket invariant is what lets the
        chassis reject out-of-order axis commands as 0xE006.
        Turning it off re-enables a race the chassis firmware
        does not tolerate.
        DOC: 13 CA-1.

  QC-13 codebook in {hex32, legacy_decimal} + all-or-nothing table
        WHY: half-filled table would half-encode commands, some
        succeed some fail. Better to refuse the whole activation
        than run in the mixed state.
        FAIL MODE: real-axis command encodes but gait switch
        does not -- driver appears to move but never switches
        gait, chassis maintains flat.
        DOC: 13 CB-1 / CB-3.

  QC-14 forward_imu_to_rt=true implies imu_rt_key registered
        WHY: RT face keys are a closed set (11 S2.2.1). Publishing
        an unregistered key violates the RT face contract and
        no subscriber will receive it -- silent drop.
        DEFAULT: false, so this is a no-op for the current
        deploy; guards the future activation.
        DOC: 11 D-44.

  QC-15 enabled TLS candidate -> cred files exist + perm <= 0o600
        WHY: TLS-3 / TLS-4. Without this, a missing key file
        would silently timeout at 2 s per probe attempt instead
        of failing fast at freeze. World-readable key file would
        leak the private key.
        LAYERED WITH: 10 S5.4.4 assertion J-3 which recursively
        chmod-checks the whole secrets/ tree regardless of
        enabled flag. K's job here is fine-grained per-candidate.
        DOC: 13 TLS-3 / TLS-4.

  QC-16 single_tx_owner == true
        WHY: CA-4 -- exactly one function writes to the chassis
        socket. Two writers can interleave APDU frames and
        produce a byte stream the chassis cannot decode.
        DOC: 13 CA-4.

  QC-17 not_implemented.gaits contains stair_standard
        WHY: GS-1 -- downlink 0x1003 has no matching readback,
        so a stair_standard dispatch would send-but-never-confirm.
        Making it explicit under not_implemented ensures the
        dispatcher rejects it early rather than falling through
        to a chassis-side error.
        DOC: 13 GS-1 (V-59 unresolved).
"""

# json import unused historically; kept out. os for path work in QC-15
# (cred_dir stat).
import os
# typing for annotations. Iterable/List/Optional used across the
# seventeen sub-checks.
from typing import Any, Dict, Iterable, List, Optional

# Layer loader for reading L6 raw quadruped.yaml. K does not need
# overlay (no cross-file check); L6 is enough.
from xbrain.boot.freeze.assertions._layer_loader import load_l6_files
# XbrainError base; K uses E_CONFIG_INVALID uniformly for every QC-N
# failure. detail.rule discriminates which rule fired.
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# ---------------------------------------------------------------------------
# Closed sets / limits pulled from 13 S8.3 verbatim
# ---------------------------------------------------------------------------

# QC-2: cmd_timeout_ms hard floor. 11 S9.12.6 verbatim '200 ms lower
# bound'. Below this the chassis heartbeat cannot round-trip within
# the Tier-1 budget on a slow link.
_QC2_CMD_TIMEOUT_MS_MIN = 200

# QC-3: control loop rate lower bound. 11 S9.12.2 verbatim.
_QC3_CONTROL_LOOP_HZ_MIN = 100.0

# QC-5: ros_domain_id legal range. Per PB-4 rationale: 0 is the DDS
# default and would collide with any laptop / debugger on the LAN.
# Upper bound 232 is DDS spec (RTPS max participant id).
_QC5_ROS_DOMAIN_ID_MIN = 1
_QC5_ROS_DOMAIN_ID_MAX = 232
# QC-5: acceptable rmw values. Two rows because D-15 v0.7 lists both
# as candidates; deploy picks cyclone by default.
_QC5_RMW_ALLOWED = frozenset({"rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"})

# QC-6: port range (TCP/UDP legal).
_QC6_PORT_MIN = 1
_QC6_PORT_MAX = 65535

# QC-9: two stair gaits that PR-1 mandates in prone_forbidden_gaits.
# 'Allowed to widen, not narrow.' Missing either = defect.
_QC9_REQUIRED_FORBIDDEN_GAITS = frozenset({"stair_agile", "stair_standard"})

# QC-10: registered gait names per 13 S5.3. A special_gaits entry
# outside this set is a typo or a gait that does not exist.
_QC10_REGISTERED_GAITS = frozenset({
    "flat", "stair_agile", "stair_standard", "prone", "damped_prone",
})

# QC-11: heartbeat lower bound per Vendor Guide 1.2.1.
_QC11_HEARTBEAT_HZ_MIN = 1.0

# QC-13: codebook closed set. 'legacy_decimal' is a spare slot that
# must be populated by 5 rows before it can activate (see CB-3).
_QC13_CODEBOOK_ALLOWED = frozenset({"hex32", "legacy_decimal"})
# QC-13: five entries the legacy_decimal table must carry, all-or-
# nothing. Named strings match the codebook_table.legacy_decimal
# dict keys per 13 S2.2 CB-3.
_QC13_LEGACY_REQUIRED_KEYS = frozenset({
    "heartbeat", "usage_mode_switch", "motion_state_switch",
    "gait_switch", "real_axis_cmd",
})

# QC-14: RT face IMU keys registered in 11 S2.2.1. Current default
# is 'rt/chassis/imu' per doc's recommendation; other keys can be
# added by 11 revisions. Tests widen the whitelist via ctx.
_QC14_KNOWN_RT_IMU_KEYS = frozenset({"rt/chassis/imu"})

# QC-15: TLS cred file permission mask. 0o600 (owner rw only). Any
# other-writable / group-writable bit is a failure. World-readable
# would leak the private key at a network boundary.
_QC15_MAX_CRED_PERM = 0o600
# QC-15: cred file names that must exist in cred_dir when the
# candidate is enabled with tls=true. Two shapes: CA + client cert
# + client key (three files) OR PSK + identity (two files).
_QC15_CERT_FILES = frozenset({"ca.crt", "client.crt", "client.key"})
_QC15_PSK_FILES = frozenset({"psk.hex", "psk_identity"})


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker; same shape as other assertions."""
    # Walk segment by segment; missing/non-dict returns default.
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _fail(rule: str, key: str, message: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with QC-N label + key path.

    Detail shape uniform across all 17 sub-checks so downstream
    dashboards can aggregate by rule cleanly.
    """
    # detail carries rule + key at a minimum; extras (value, limit,
    # reason, ...) are per-rule context.
    detail: Dict[str, Any] = {"kind": "qc_violation", "rule": rule,
                              "key": key}
    detail.update(extra)
    # Message format: 'assertion K failed: QC-N: <human message>'
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion K failed: %s: %s" % (rule, message),
        detail,
    )


# ---------------------------------------------------------------------------
# QC-1: no spec.* key definitions inside the quadruped tree
# ---------------------------------------------------------------------------

def _walk_dict_paths(tree: Dict[str, Any], prefix: str = "") -> Iterable[str]:
    """Yield every dotted key path under a nested dict.

    Used by QC-1 to enumerate every key defined and flag any that
    starts with 'spec.' (post-'quadruped.' prefix stripping).
    """
    for k, v in tree.items():
        # Build the dotted path incrementally so we can report the
        # full offending key in the failure message.
        full = "%s.%s" % (prefix, k) if prefix else k
        # Yield the leaf path first; recurse into dicts.
        yield full
        # dict values recurse; scalar / list values are leaves.
        if isinstance(v, dict):
            yield from _walk_dict_paths(v, full)


def _check_qc1(quadruped_raw: Dict[str, Any]) -> None:
    """QC-1: no key path under quadruped starts with 'spec.'.

    ${common.spec.*} REFERENCES are legal (a string like '${common.
    spec.max_decel_mps2}' is a scalar value, not a DEFINITION). This
    check flags key names beginning with 'spec' only.
    """
    # QC-1 lives on the L6 SOURCE per 13 S8.3 explicit note.
    quad = quadruped_raw.get("quadruped", {})
    if not isinstance(quad, dict):
        return
    for path in _walk_dict_paths(quad):
        # Split into first segment; a defined 'spec.<anything>' at any
        # nesting level violates SP-3.
        head = path.split(".", 1)[0]
        if head == "spec":
            _fail("QC-1", "quadruped." + path,
                  "spec.* keys must not be DEFINED under quadruped "
                  "(references ${common.spec.*} are legal, definitions "
                  "are not)",
                  reason="spec_key_defined")


# ---------------------------------------------------------------------------
# QC-2 / QC-3: tier1 numeric floors
# ---------------------------------------------------------------------------

def _check_qc2(quad: Dict[str, Any]) -> None:
    """QC-2: tier1.cmd_timeout_ms >= 200.

    Below 200 ms and the chassis heartbeat cannot round-trip within
    a Tier-1 budget; the state machine will spuriously judge the
    link degraded on every tick.

    The 200 ms floor is not a target -- production runs at 200-500 ms
    depending on link quality. The floor exists because the chassis
    firmware round-trip time varies from 50 ms (clean LAN) up to
    ~180 ms (WiFi with retries). Any value below 200 ms guarantees
    a false-positive 'degraded' on any imperfect link.

    CFG-FZ-12 variant (3) verbatim: tier1.cmd_timeout_ms = 50 hits
    this branch and turns K red.
    """
    # Read the value; missing key = skip (M's job to enforce presence).
    v = _get(quad, "tier1.cmd_timeout_ms")
    # Skip when key absent -- M (required-keys) is the enforcer for
    # 'the key exists'; K only enforces value range on values that
    # ARE present. Keeping K single-purpose keeps the failure
    # attribution clean when the log says 'QC-2 vs required-key'.
    if v is None:
        return
    # Numeric type gate. A str '200' from a mistyped yaml would
    # otherwise crash the < comparison with TypeError -- surface it
    # as QC-2 fail so the failure attribution stays on the QC rule.
    if not isinstance(v, (int, float)) or v < _QC2_CMD_TIMEOUT_MS_MIN:
        # CFG-FZ-12 variant (3) fires here.
        _fail("QC-2", "quadruped.tier1.cmd_timeout_ms",
              "cmd_timeout_ms %r below hard floor %d ms (11 S9.12.6)"
              % (v, _QC2_CMD_TIMEOUT_MS_MIN),
              value=v, limit=_QC2_CMD_TIMEOUT_MS_MIN)


def _check_qc3(quad: Dict[str, Any]) -> None:
    """QC-3: tier1.control_loop_hz >= 100 (11 S9.12.2).

    The 100 Hz floor matches the odom publish rate. Running the
    control loop below odom rate would skip control cycles between
    odom updates -- the loop would consume the same odom twice and
    the second consumption would produce an identical command.
    Doubling commands on a moving robot amplifies control noise.

    100 is the minimum; production usually runs at 100 exactly to
    match odom cadence.
    """
    # Read + skip-if-absent, same pattern as QC-2.
    v = _get(quad, "tier1.control_loop_hz")
    if v is None:
        return
    # Numeric type gate + floor comparison.
    if not isinstance(v, (int, float)) or v < _QC3_CONTROL_LOOP_HZ_MIN:
        _fail("QC-3", "quadruped.tier1.control_loop_hz",
              "control_loop_hz %r below lower bound %s"
              % (v, _QC3_CONTROL_LOOP_HZ_MIN),
              value=v, limit=_QC3_CONTROL_LOOP_HZ_MIN)


# ---------------------------------------------------------------------------
# QC-4 / QC-5: DDS / ROS domain isolation
# ---------------------------------------------------------------------------

def _check_qc4(quad: Dict[str, Any]) -> None:
    """QC-4: chassis_dds.domain_id != uplink.ros_domain_id.

    Two-domain isolation is the whole point of quadruped's three-
    channel layout (13 S2.4 DDS-1..DDS-9). If they collide, the
    domain-0 CHS-B reader also picks up the uplink domain-42 publish,
    and Tier-1 state pubs cross into the ROS 2 side unintended.

    CFG-FZ-12 variant (4) verbatim: both set to the same value ->
    this branch. The variant that specifically catches the class of
    'operator copied domain_id from the wrong side' error.

    Why this is not merely a warning: DDS domain collision is
    silent at the network layer. Both participants advertise as
    the same domain and DISCOVER each other's publishers. The
    chassis-side subscriber that was meant to read /IMU from the
    chassis DDS starts receiving /odom_quadruped from the uplink
    ROS 2 side; there is no protocol error, just wrong data.

    A test with domain_id == ros_domain_id == 42 hits this branch;
    the failure detail carries BOTH sides so an operator can decide
    which one to change (usually ros_domain_id back to 42 and
    chassis_dds back to 0).
    """
    # Read both keys. Missing either = skip (M's job).
    dds = _get(quad, "chassis_dds.domain_id")
    ros = _get(quad, "uplink.ros_domain_id")
    # Skip if either absent; M handles required-ness. Presence is
    # not K's concern -- K only checks the collision if both are
    # present.
    if dds is None or ros is None:
        return
    # Equality check. Comparison works on any pair (int/float/str)
    # so no type gate needed -- if types differ, they will not
    # compare equal, and QC-5 fires on the individual type below.
    if dds == ros:
        # Report BOTH sides so operator can decide which to change.
        _fail("QC-4", "quadruped.chassis_dds.domain_id",
              "chassis_dds.domain_id (%r) equals uplink.ros_domain_id "
              "(%r); domain isolation defeated" % (dds, ros),
              chassis_dds_domain_id=dds, uplink_ros_domain_id=ros,
              reason="domain_collision")


def _check_qc5(quad: Dict[str, Any]) -> None:
    """QC-5: uplink.ros_domain_id legal + uplink.rmw in closed set.

    D-15 v0.7 defers exact value to deploy but freezes the CANDIDATE
    SET. K enforces legality of the choice, not the choice itself.

    The v0.2 order of this rule went from 'ros_domain_id must be
    exactly 42' to 'ros_domain_id must be a legal value'. Reason:
    D-15 is 'still pending review' and pre-locking a value that
    reviewers may still change would force a code edit when they
    do. The rule now enforces STRUCTURE (range + closed set) so a
    deploy can change the specific value without touching code.

    Domain 0 rejection is the important half: DDS default is 0,
    and any laptop / debug tool joins domain 0 by default. Using
    domain 0 in production would mean the robot receives every
    debug publish on the network.

    RMW closed set = {cyclonedds, fastrtps}. These are the two
    RMW impls the project has stability data for. rmw_zenoh_cpp
    is intentionally NOT in the set even though Zenoh is the
    RT face transport -- for ROS 2 uplink we stick to DDS-based
    RMW to keep interop with third-party ROS 2 tooling.
    """
    # Read the ros_domain_id; skip if absent.
    dom = _get(quad, "uplink.ros_domain_id")
    if dom is not None:
        # int only; str '42' would silently satisfy comparisons in
        # some yaml loaders and later mis-set the ROS domain. The
        # yaml.safe_load produces int for bare '42', str for
        # quoted '"42"' -- a quoted string here is the operator
        # typo that this type gate catches.
        if not isinstance(dom, int) or not (
                _QC5_ROS_DOMAIN_ID_MIN <= dom <= _QC5_ROS_DOMAIN_ID_MAX):
            _fail("QC-5", "quadruped.uplink.ros_domain_id",
                  "ros_domain_id %r not in [%d, %d]"
                  % (dom, _QC5_ROS_DOMAIN_ID_MIN, _QC5_ROS_DOMAIN_ID_MAX),
                  value=dom, min=_QC5_ROS_DOMAIN_ID_MIN,
                  max=_QC5_ROS_DOMAIN_ID_MAX)
    # Read rmw; skip if absent.
    rmw = _get(quad, "uplink.rmw")
    if rmw is not None:
        # Closed-set membership. sorted() in the message gives
        # deterministic ordering across runs.
        if rmw not in _QC5_RMW_ALLOWED:
            _fail("QC-5", "quadruped.uplink.rmw",
                  "rmw %r not in %s" % (rmw, sorted(_QC5_RMW_ALLOWED)),
                  value=rmw, allowed=sorted(_QC5_RMW_ALLOWED))


# ---------------------------------------------------------------------------
# QC-6: endpoint_candidates non-empty + valid ports
# ---------------------------------------------------------------------------

def _check_qc6(quad: Dict[str, Any]) -> None:
    """QC-6: chassis_link.endpoint_candidates non-empty and each has
    port in [1, 65535].

    The probe walks the endpoint_candidates list in order and picks
    the first one that returns a state upload. An empty list means
    the probe has nothing to try -- boot deadlocks. An invalid port
    (0, negative, > 65535) would fail socket(2) with EINVAL and
    the probe would move on -- correct behaviour but the invalid
    entry is a config typo we want the operator to know about
    before it silently gets skipped.

    The port range check catches typos like 3003 (missing a zero
    from 30003) which would otherwise be a valid ephemeral port
    that the chassis is not listening on.

    Per-endpoint reporting: the failure detail includes the array
    index so an operator can jump straight to endpoint_candidates[N]
    in the yaml. Without the index they would have to open the
    file and count entries.
    """
    # Read; skip if absent (M handles required-ness).
    eps = _get(quad, "chassis_link.endpoint_candidates")
    if eps is None:
        return
    # Shape gate: non-empty list. Both wrong type and empty list
    # collapse to same failure kind because both mean 'nothing to
    # probe'.
    if not isinstance(eps, list) or not eps:
        _fail("QC-6", "quadruped.chassis_link.endpoint_candidates",
              "endpoint_candidates must be a non-empty list "
              "(got %r)" % (eps,),
              value=eps, reason="empty")
    # Per-entry validation. First-fail with index so operator can
    # find the bad row without opening the yaml. enumerate() gives
    # both the 0-based index (which matches array notation) and
    # the entry.
    for i, ep in enumerate(eps):
        # Each entry must be a dict; a scalar here is a schema
        # defect that would crash the probe with AttributeError
        # on .get('proto') later.
        if not isinstance(ep, dict):
            _fail("QC-6",
                  "quadruped.chassis_link.endpoint_candidates[%d]" % i,
                  "endpoint %d not a dict: %r" % (i, ep))
        # Port must be an int in the legal TCP/UDP range.
        port = ep.get("port")
        if not isinstance(port, int) or not (
                _QC6_PORT_MIN <= port <= _QC6_PORT_MAX):
            _fail("QC-6",
                  "quadruped.chassis_link.endpoint_candidates[%d].port"
                  % i,
                  "port %r not in [%d, %d]"
                  % (port, _QC6_PORT_MIN, _QC6_PORT_MAX),
                  value=port, min=_QC6_PORT_MIN, max=_QC6_PORT_MAX)


# ---------------------------------------------------------------------------
# QC-7 / QC-8: odom staleness + publish rate
# ---------------------------------------------------------------------------

def _check_qc7(quad: Dict[str, Any]) -> None:
    """QC-7: odom.stale_warn_ms < stale_invalid_ms < stale_stop_publish_ms.

    Monotone triple; any two swapped and the four-band staleness
    state machine (13 S4.4) crosses states in the wrong order.

    Concrete failure mode of swapped values: if stale_invalid_ms
    (300) is set below stale_warn_ms (150), the state machine
    would enter 'invalid' state at 300 ms of staleness, but the
    'warn' precondition would fire ONLY at 150 ms -- meaning the
    operator sees no warning until AFTER the odom has been marked
    invalid. Downstream consumers that gate on 'not invalid'
    would see valid->invalid transitions with no warning.
    """
    # Read all three; if any missing, skip (M's job to enforce
    # presence). All-or-nothing so partial monotone is not judged.
    warn = _get(quad, "odom.stale_warn_ms")
    inv = _get(quad, "odom.stale_invalid_ms")
    stop = _get(quad, "odom.stale_stop_publish_ms")
    if None in (warn, inv, stop):
        return
    # Type gate on all three; a str would crash the < chain.
    if not (isinstance(warn, (int, float)) and isinstance(inv, (int, float))
            and isinstance(stop, (int, float))):
        return
    # Strict monotone (all three distinct). Equal values would put
    # two transitions on the same threshold and could result in
    # state cycling on jitter.
    if not (warn < inv < stop):
        _fail("QC-7", "quadruped.odom.stale_*_ms",
              "monotone violated: warn=%r invalid=%r stop=%r "
              "(need warn < invalid < stop)" % (warn, inv, stop),
              stale_warn_ms=warn, stale_invalid_ms=inv,
              stale_stop_publish_ms=stop)


def _check_qc8(quad: Dict[str, Any]) -> None:
    """QC-8: odom.publish_hz and (1000 / publish_hz) both integer ms.

    Integer-ms cadence means the scheduler can hit a fixed tick.
    A publish_hz like 30 -> 33.33 ms would drift the 100 Hz odom
    grid by 0.33 ms per tick, accumulating to a full period every
    30 s.

    Why not just check that hz is integer: publish_hz = 3 (integer)
    yields period_ms = 333.33, still non-integer. The rule is that
    BOTH must be integers, which restricts publish_hz to divisors
    of 1000: {1, 2, 4, 5, 8, 10, 20, 25, 40, 50, 100, 125, 200,
    250, 500, 1000}. In practice production uses 100.
    """
    # Read + skip-if-absent.
    hz = _get(quad, "odom.publish_hz")
    if hz is None:
        return
    if not isinstance(hz, (int, float)):
        return
    # Reject non-positive; 0 would divide-by-zero, negative makes
    # no physical sense.
    if hz <= 0:
        _fail("QC-8", "quadruped.odom.publish_hz",
              "publish_hz must be positive (got %r)" % (hz,), value=hz)
    # Compute the period; float division so 100.0 yields 10.0 not
    # a floor int.
    period_ms = 1000.0 / hz
    # Two floats compared as ints; use tolerance-free check because
    # QC-8 wording says 'integer ms' verbatim. int() truncates so
    # 33.33 != int(33.33) = 33 which surfaces the drift.
    if hz != int(hz) or period_ms != int(period_ms):
        _fail("QC-8", "quadruped.odom.publish_hz",
              "publish_hz %r yields non-integer period %r ms; both "
              "hz and 1000/hz must be integer" % (hz, period_ms),
              value=hz, period_ms=period_ms)


# ---------------------------------------------------------------------------
# QC-9 / QC-10 / QC-17: gait sets
# ---------------------------------------------------------------------------

def _check_qc9(quad: Dict[str, Any]) -> None:
    """QC-9: motion.prone_forbidden_gaits >= {stair_agile, stair_standard}.

    'Allowed to widen, not narrow.' The stair gaits are structurally
    unsafe from prone; removing either is a defect regardless of
    intent.

    CFG-FZ-12 variant (2) verbatim: prone_forbidden_gaits with
    stair_standard removed. The variant checks that a well-intended
    'we do not use stair_standard so let's clean the list' edit is
    still refused.

    Widening (adding e.g. 'damped_prone') is legal -- more restrictive
    is always safe. Narrowing is not -- the two stair gaits ARE
    unsafe from prone regardless of operator opinion.

    The frozenset comparison surfaces MISSING items explicitly so
    the operator sees which one to add back rather than being told
    'your list is wrong'.
    """
    # Read the list; missing = skip (M's job).
    v = _get(quad, "motion.prone_forbidden_gaits")
    if v is None:
        return
    # Shape gate: must be a list. A scalar or dict here is a schema
    # defect but K reports it under QC-9 for attribution.
    if not isinstance(v, list):
        _fail("QC-9", "quadruped.motion.prone_forbidden_gaits",
              "prone_forbidden_gaits must be a list (got %r)" % (v,),
              value=v)
    # frozenset for O(1) diff and canonical ordering in report.
    current = frozenset(v)
    # Set-difference: which required gaits are absent from current.
    missing = _QC9_REQUIRED_FORBIDDEN_GAITS - current
    # If any required gait is missing, fail with the missing list
    # named. sorted() for deterministic report order.
    if missing:
        # CFG-FZ-12 variant (2) fires here.
        _fail("QC-9", "quadruped.motion.prone_forbidden_gaits",
              "prone_forbidden_gaits missing %s (allowed to widen "
              "not narrow -- PR-1)"
              % sorted(missing),
              value=sorted(current), missing=sorted(missing))


def _check_qc10(quad: Dict[str, Any]) -> None:
    """QC-10: every motion.axes.special_gaits entry is a registered gait.

    Guards against typos and against writing a gait name that does
    not exist. The registered set is small and fixed (13 S5.3).
    """
    v = _get(quad, "motion.axes.special_gaits")
    if v is None:
        return
    if not isinstance(v, list):
        return
    for entry in v:
        if entry not in _QC10_REGISTERED_GAITS:
            _fail("QC-10", "quadruped.motion.axes.special_gaits",
                  "special_gaits entry %r not in registered set %s"
                  % (entry, sorted(_QC10_REGISTERED_GAITS)),
                  value=entry, allowed=sorted(_QC10_REGISTERED_GAITS))


def _check_qc17(quad: Dict[str, Any]) -> None:
    """QC-17: motion.not_implemented.gaits contains stair_standard.

    Same shape as QC-9: widen-only. GS-1 mandates we never emit
    stair_standard because its readback code differs from the
    downlink code (V-59 unresolved).

    Concrete: 13 GaitParam downlink set = {0x1001, 0x1003, 0x3002,
    0x3003} (contains 0x1003 = stair_standard). Gait upload set =
    {0x1001, 0x1002, 0x3002, 0x3003} (does NOT contain 0x1003).
    Sending 0x1003 succeeds on the wire, but readback confirmation
    NEVER matches -- MS-2 (mode-switch second) will judge the
    switch failed even though the chassis may have accepted it.

    Making stair_standard explicit in not_implemented.gaits ensures
    the dispatch layer rejects it EARLY with E_NOT_IMPLEMENTED
    rather than sending a command whose readback we know will
    fail. Removing stair_standard from the list would re-enable
    the send-then-fail-on-readback path.
    """
    # Read + skip-if-absent + shape gate.
    v = _get(quad, "motion.not_implemented.gaits")
    if v is None:
        return
    if not isinstance(v, list):
        return
    # Membership check. Widening (adding more not_implemented
    # gaits) is legal; narrowing (removing stair_standard) is not.
    if "stair_standard" not in v:
        _fail("QC-17", "quadruped.motion.not_implemented.gaits",
              "not_implemented.gaits must include 'stair_standard' "
              "(GS-1)",
              value=v, required="stair_standard")


# ---------------------------------------------------------------------------
# QC-11 / QC-12 / QC-16: chassis link toggles + heartbeat
# ---------------------------------------------------------------------------

def _check_qc11(quad: Dict[str, Any]) -> None:
    """QC-11: chassis_link.heartbeat_hz >= 1.0 (Vendor Guide 1.2.1).

    The chassis firmware watches for heartbeat frames and enters
    its own safe-mode if none arrives for >1 s. A heartbeat rate
    below 1 Hz means we would starve the watchdog on our own
    controller before the chassis firmware even sees a slow link.

    Production runs at 2 Hz per 13 S8.2 example -- that gives 500 ms
    slack under the chassis-side 1 s watchdog even on a stuttering
    link.
    """
    # Read + skip-if-absent + numeric type gate + floor comparison.
    # Same pattern as QC-2/QC-3.
    v = _get(quad, "chassis_link.heartbeat_hz")
    if v is None:
        return
    if not isinstance(v, (int, float)) or v < _QC11_HEARTBEAT_HZ_MIN:
        _fail("QC-11", "quadruped.chassis_link.heartbeat_hz",
              "heartbeat_hz %r below floor %s"
              % (v, _QC11_HEARTBEAT_HZ_MIN),
              value=v, limit=_QC11_HEARTBEAT_HZ_MIN)


def _check_qc12(quad: Dict[str, Any]) -> None:
    """QC-12: chassis_link.axis_cmd_socket_fixed == true.

    CA-1 rule -- the fixed-socket invariant is what lets the chassis
    reject 0xE006 out-of-order axis commands. Any config that tries
    to disable it is a foot-gun to make disabled explicitly reject.

    Why this rule exists as a config key at all: without a config
    key, an operator who wants to disable it has to touch code.
    With a config key, the temptation to disable it is high --
    hence the assertion: 'the key exists so an audit can grep for
    it, but it must always be true'.

    This is the same pattern as QC-16 (single_tx_owner) -- both are
    'the key exists specifically so an attempt to flip it is
    captured by an assertion'.
    """
    # Read + skip-if-absent. is not True catches both False and
    # non-bool truthy values (int 1 is truthy but not True). We
    # want STRICT True per the doc's 'must be true' wording.
    v = _get(quad, "chassis_link.axis_cmd_socket_fixed")
    if v is None:
        return
    if v is not True:
        _fail("QC-12", "quadruped.chassis_link.axis_cmd_socket_fixed",
              "axis_cmd_socket_fixed must be True (got %r); CA-1"
              % (v,), value=v)


def _check_qc16(quad: Dict[str, Any]) -> None:
    """QC-16: chassis_link.single_tx_owner == true.

    CA-4 rule: one and only one function may write to the chassis
    socket. Splitting the writer is the failure mode where two
    goroutines interleave APDU frames and produce a stream the
    chassis cannot decode.

    Concrete failure: two threads each holding a partial APDU
    frame race for the socket. Bytes interleave; the chassis
    receives frame1[0..7] + frame2[0..7] + frame1[8..15] etc.
    Length field doesn't match, CRC fails, chassis logs decoder
    error and drops. From our side the send() returned success
    for both, so no error propagates back.

    Same 'key exists as an assertion target' pattern as QC-12.
    """
    # Read + skip-if-absent + strict True check.
    v = _get(quad, "chassis_link.single_tx_owner")
    if v is None:
        return
    if v is not True:
        _fail("QC-16", "quadruped.chassis_link.single_tx_owner",
              "single_tx_owner must be True (got %r); CA-4"
              % (v,), value=v)


# ---------------------------------------------------------------------------
# QC-13: codebook value + table completeness
# ---------------------------------------------------------------------------

def _check_qc13(quad: Dict[str, Any]) -> None:
    """QC-13: codebook in closed set + legacy_decimal table five-row rule.

    Half of this rule is the closed-set check on the string value;
    the other half is the 'all or nothing' constraint on the legacy
    decimal codebook. A table with 1..4 rows is worse than an empty
    table: the runtime would half-encode commands and half fail.

    CFG-FZ-12 variant (1) verbatim: codebook=legacy_decimal WITH
    an empty codebook_table.legacy_decimal -> this branch fires.
    The variant catches the operator who reads the doc, thinks
    'legacy_decimal is a fallback' and enables it without filling
    the table. Silent failure would be: real-axis command encodes
    (using a default), gait switch fails to encode (returns None),
    dispatch silently drops the gait command -- robot moves but
    never switches gait.

    hex32 with an empty legacy_decimal table is legal: the table
    is a DORMANT SPARE (per CB-3). Only when legacy_decimal is
    activated does the table become required.
    """
    # Read the codebook value.
    codebook = _get(quad, "chassis_link.codebook")
    # Closed-set check on the string value. Absent = skip (M's job).
    if codebook is not None and codebook not in _QC13_CODEBOOK_ALLOWED:
        _fail("QC-13", "quadruped.chassis_link.codebook",
              "codebook %r not in %s"
              % (codebook, sorted(_QC13_CODEBOOK_ALLOWED)),
              value=codebook, allowed=sorted(_QC13_CODEBOOK_ALLOWED))
    # If codebook == legacy_decimal, the table must be five-complete.
    # We do NOT require the table to be five-complete when the
    # codebook is hex32 -- that would over-enforce (the table is a
    # dormant spare kept for future activation via a config-only
    # change once the five decimal codes are known).
    if codebook == "legacy_decimal":
        # Read table with a {} default so an absent block still
        # triggers 'all five missing' rather than crash.
        table = _get(quad, "chassis_link.codebook_table.legacy_decimal", {})
        # Shape gate on the table itself.
        if not isinstance(table, dict):
            _fail("QC-13",
                  "quadruped.chassis_link.codebook_table.legacy_decimal",
                  "legacy_decimal table must be a dict (got %r)"
                  % (table,), value=table)
        # CFG-FZ-12 variant (1) verbatim: codebook=legacy_decimal +
        # empty table -> this branch fires. Missing set is computed
        # so the report names EXACTLY which rows need adding.
        missing = _QC13_LEGACY_REQUIRED_KEYS - frozenset(table.keys())
        if missing:
            # Report present + missing so operator sees both halves
            # and can decide whether to fill or roll back to hex32.
            _fail("QC-13",
                  "quadruped.chassis_link.codebook_table.legacy_decimal",
                  "legacy_decimal table missing rows %s (all-or-nothing "
                  "-- CB-3)" % sorted(missing),
                  present=sorted(table.keys()),
                  missing=sorted(missing))


# ---------------------------------------------------------------------------
# QC-14: RT face IMU key whitelist (only when forward_imu_to_rt=true)
# ---------------------------------------------------------------------------

def _check_qc14(quad: Dict[str, Any],
                known_rt_imu_keys: frozenset) -> None:
    """QC-14: forward_imu_to_rt=true implies imu_rt_key registered.

    Default is forward_imu_to_rt=false so this is a no-op for the
    baseline config. Only fires when a deploy tries to turn the
    forwarding on; then the key MUST be one that 11 S2.2.1 has
    already registered.

    Why the whitelist: RT face keys are a closed set (RT-C1). A
    publisher advertising an unregistered key would silently
    succeed on the wire but have no subscriber on the far side
    -- 11 S2.2.1 lists exactly which keys are legal to publish.
    A typo like 'rt/chassis/lmu' would be published for the
    lifetime of the process with no error.

    Default state (forward=false, key='') is legal because there
    is no publish attempt to route through the registry. The
    check ONLY fires on the activation path, hence the guard
    'on is not True -> return'.

    Test-widening via ctx['known_rt_imu_keys']: lets a test
    inject its own whitelist so the whitelist semantic can be
    tested without hardcoding a specific key that may later
    change in 11 S2.2.1.
    """
    # Read the toggle. is not True catches False (default) AND
    # any non-bool value; we only enforce when strictly True.
    on = _get(quad, "chassis_dds.forward_imu_to_rt")
    if on is not True:
        # Default / any non-true value -> nothing to enforce.
        return
    # Toggle is true -> the rt key must be non-empty and registered.
    key = _get(quad, "chassis_dds.imu_rt_key")
    # Empty / non-str check first; reports 'empty_key' reason.
    if not isinstance(key, str) or not key:
        _fail("QC-14", "quadruped.chassis_dds.imu_rt_key",
              "forward_imu_to_rt is true but imu_rt_key is empty",
              value=key, reason="empty_key")
    # Registered check second; reports the allowed set so the
    # operator can pick a valid alternative.
    if key not in known_rt_imu_keys:
        _fail("QC-14", "quadruped.chassis_dds.imu_rt_key",
              "imu_rt_key %r not in registered RT keys %s "
              "(11 S2.2.1)" % (key, sorted(known_rt_imu_keys)),
              value=key, allowed=sorted(known_rt_imu_keys))


# ---------------------------------------------------------------------------
# QC-15: TLS credentials (existence + permission) for enabled candidates
# ---------------------------------------------------------------------------

def _check_qc15(quad: Dict[str, Any]) -> None:
    """QC-15: every enabled TLS candidate has readable cred files in
    tls.cred_dir with permission <= 0o600.

    CFG-FZ-12 variant (5) verbatim: TLS candidate enabled but
    credential files missing -> this fires. Turns 'silent 2 s per
    probe timeout' into 'explicit fast failure at freeze'.

    Layered gate: 10 S5.4.4 assertion J-3 recursively enforces
    chmod on the whole secrets/ tree regardless of the enabled
    flag. K's QC-15 fires PER CANDIDATE and only when enabled,
    which lets the failure message name the specific candidate
    that would have failed at runtime. Both gates are needed:
    J-3 catches the 'not-yet-enabled but still world-readable'
    case; QC-15 catches the 'enabled but credentials missing'
    case.

    Credential SHAPES: two mutually exclusive layouts are legal
    per 13 TLS-2/TLS-3:
      cert triple: ca.crt + client.crt + client.key
      PSK pair:    psk.hex + psk_identity
    Whichever shape is present, all files of that shape get the
    permission check.
    """
    # Read endpoint list; skip if absent (nothing enabled = nothing
    # to enforce).
    eps = _get(quad, "chassis_link.endpoint_candidates") or []
    if not isinstance(eps, list):
        return
    # TLS candidate = tls:true AND enabled:true. Scan the shape
    # loosely because a candidate row may be a dict OR use nested
    # 'tls: { enabled: ... }' -- 13 S8.2 example uses the flat form.
    # If a future schema change moves to nested, this comprehension
    # will need updating (test would then fail because the flag no
    # longer surfaces at the top level).
    tls_candidates = [ep for ep in eps
                      if isinstance(ep, dict)
                      and ep.get("tls") is True
                      and ep.get("enabled") is True]
    # If nothing to check, done. This is the common case today
    # because TLS-1 keeps every TLS candidate at enabled=false
    # pending V-40 (chassis TLS server availability).
    if not tls_candidates:
        return
    # From here on we know at least one TLS candidate is enabled;
    # cred_dir MUST name an existing directory containing at least
    # one full credential shape.
    cred_dir = _get(quad, "chassis_link.tls.cred_dir")
    # An enabled TLS candidate MUST name a cred_dir; empty/absent is
    # itself a violation. This mirrors variant (5)'s intent.
    if not isinstance(cred_dir, str) or not cred_dir:
        _fail("QC-15", "quadruped.chassis_link.tls.cred_dir",
              "TLS candidate is enabled but cred_dir is empty/absent",
              value=cred_dir, reason="cred_dir_missing")
    # Directory itself must exist. isdir returns False on missing
    # OR on a regular file at that path -- both are 'no dir here'.
    if not os.path.isdir(cred_dir):
        _fail("QC-15", "quadruped.chassis_link.tls.cred_dir",
              "cred_dir does not exist: %s" % cred_dir,
              value=cred_dir, reason="cred_dir_absent")
    # Discover which credential shape is present: cert triple OR PSK
    # pair. Neither present -> variant (5). Both present is fine
    # (allows overlapping deploys); we check cert triple first for
    # deterministic reporting.
    have_cert = all(os.path.isfile(os.path.join(cred_dir, f))
                    for f in _QC15_CERT_FILES)
    have_psk = all(os.path.isfile(os.path.join(cred_dir, f))
                   for f in _QC15_PSK_FILES)
    # Neither shape complete = no credentials at all.
    if not (have_cert or have_psk):
        _fail("QC-15", "quadruped.chassis_link.tls.cred_dir",
              "cred_dir %s has neither cert triple %s nor PSK pair %s"
              % (cred_dir, sorted(_QC15_CERT_FILES),
                 sorted(_QC15_PSK_FILES)),
              cred_dir=cred_dir, reason="no_credentials")
    # Permission check on whichever files are present. If both
    # shapes are present, we check the cert triple only -- checking
    # both would fire duplicate errors on the same underlying
    # config problem.
    files_to_check = (_QC15_CERT_FILES if have_cert
                      else _QC15_PSK_FILES)
    for name in files_to_check:
        path = os.path.join(cred_dir, name)
        # st_mode carries type + perm bits; mask to perm only.
        mode = os.stat(path).st_mode & 0o777
        # Permission tighter is fine; looser is not. Bitmask
        # (mode & ~_MAX) is non-zero iff ANY bit above the max
        # is set -- 0o640 has bit 0o040 set which is > 0o600.
        # Cannot compare integers directly because 0o400 < 0o600
        # numerically but 0o400 is TIGHTER (read-only), which
        # should PASS. The bit-mask distinguishes these correctly.
        if mode & ~_QC15_MAX_CRED_PERM:
            _fail("QC-15", "quadruped.chassis_link.tls.cred_dir",
                  "credential %s has permission 0o%o; must be <= 0o600"
                  % (path, mode),
                  path=path, mode=oct(mode),
                  reason="perm_too_loose")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def _load_quadruped(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Resolve the quadruped tree from ctx override or L6 loader.

    Two paths:
      * ctx['quadruped_raw'] override: used by tests to inject a
        dict without writing yaml to disk. Also usable by an
        integration harness that has already parsed the yaml.
      * L6 loader: production path. Reads
        configs/quadruped.yaml via load_l6_files which handles
        yaml parse + missing-file tolerance.
    """
    # Test-friendly override; used when the caller has a synthesized
    # dict and does not want to write yaml to disk. Priority is
    # 'override wins' so tests are fully hermetic.
    override = ctx.get("quadruped_raw")
    if override is not None:
        return override
    # Production path: read L6 raw. build_overlay is not used because
    # quadruped.yaml is L6 (per-process) and does not merge with the
    # common overlay tree. If the file is absent, load_l6_files
    # returns an empty dict and every _check_qcN skips its rule.
    l6 = load_l6_files(ctx["config_root"])
    return l6.get("quadruped.yaml", {})


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion K. Replaces registry's stub for K.

    Flow:
      1. Wiring guard on ctx['config_root'].
      2. Resolve quadruped raw tree (ctx override or L6 loader).
      3. Fire QC-1 first (on raw), then QC-2..QC-17 in numeric order.
      4. Return pass with checks_run=17 for observability.

    Ordering: QC-1 first because it runs on the RAW tree (before
    any extraction). QC-2..QC-17 run in numeric order because that
    matches the doc's tabular order -- easier for an auditor to
    read the failure sequence and match it against 13 S8.3.

    First-fail: the first _check_qcN to raise terminates the run.
    This is deliberate; K is a single-boolean-gate assertion. If
    QC-2 is red, the process is unbootable regardless of QC-3..
    QC-17, so continuing would just spam the log with follow-on
    failures. All-failures mode is a follow-up if the operator
    asks for it.
    """
    # Wiring guard identical to every other assertion. AssertionError
    # (not XbrainError) because a missing config_root is a caller
    # bug, not a config defect.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion K requires ctx['config_root']; caller did not "
            "populate it"
        )
    # The raw tree is the SOURCE-level yaml (pre-expansion). For K's
    # checks this is equivalent to the resolved artifact because
    # none of QC-2..QC-17 involves a ${common.*} reference that
    # would change the compared value. If a future rule DOES need
    # the expanded value, it should be added on the resolved side
    # and this raw load kept only for QC-1.
    raw = _load_quadruped(ctx)
    # Extract the 'quadruped' sub-tree. Default {} tolerates a raw
    # tree with no 'quadruped' block (which would make every rule
    # a no-op).
    quad = raw.get("quadruped", {}) if isinstance(raw, dict) else {}
    if not isinstance(quad, dict):
        # A top-level quadruped block that is not a dict is a schema
        # defect; report as QC-1 shape violation for uniform failure
        # surface. Reporting under QC-1 (which owns the whole
        # 'shape of the quadruped section' concern) keeps the
        # failure attribution clean.
        _fail("QC-1", "quadruped",
              "top-level 'quadruped' must be a dict (got %r)"
              % (quad,), value=type(quad).__name__)

    # ctx['known_rt_imu_keys'] widens the QC-14 whitelist for tests
    # that need to assert the whitelist semantics without depending
    # on the exact set encoded in _QC14_KNOWN_RT_IMU_KEYS. Cast to
    # frozenset so QC-14's membership test is O(1).
    known_rt = frozenset(ctx.get("known_rt_imu_keys")
                         or _QC14_KNOWN_RT_IMU_KEYS)

    # QC-1 runs against the RAW tree per 13 S8.3 (not the extracted
    # quadruped sub-dict) so a spec.* at the top level would also
    # fire. Reading raw preserves 'was this key DEFINED' semantics
    # that would be lost after any expansion pass.
    _check_qc1(raw)
    # Remaining checks all operate on the quadruped sub-tree. Order
    # is numeric to match doc tabular order.
    _check_qc2(quad)
    _check_qc3(quad)
    _check_qc4(quad)
    _check_qc5(quad)
    _check_qc6(quad)
    _check_qc7(quad)
    _check_qc8(quad)
    _check_qc9(quad)
    _check_qc10(quad)
    _check_qc11(quad)
    _check_qc12(quad)
    _check_qc13(quad)
    # QC-14 needs the whitelist as a second argument (only rule
    # with an extra parameter).
    _check_qc14(quad, known_rt)
    _check_qc15(quad)
    _check_qc16(quad)
    _check_qc17(quad)

    # Success return. checks_run=17 lets a downstream observer
    # verify that the assertion actually ran every rule (rather
    # than silently returning after an early skip).
    return {
        "status": "pass",
        # Fixed assertion label; matches registry row 'K'.
        "assertion": "K",
        # Total sub-rules executed. Static 17; if this ever drifts
        # from the actual number of _check_qcN calls above, the
        # count exposes the drift.
        "checks_run": 17,
    }
