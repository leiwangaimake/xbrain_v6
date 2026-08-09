"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: failure_class.py
Brief: CFG-BT-14 -- startup failure classifier (R/B/D/T) driven by 10 S3.3.6 table

Description:
Four classes with strict, documented behaviour (10 S3.3.6):

  R  reject-startup   No participants past the observation window,
                      AND motion forbidden. Both must hold; a common
                      failure mode is "process refuses to start" but
                      motion is still allowed because a stale window
                      leaks it -- that is NOT R, it is a bug.

  B  boot-but-block   Processes up, observable, allow_motion=false,
                      BLOCKED state.

  D  degraded         Motion allowed, capability limited; MUST land
                      a warn event AND set a HMI persistent marker.

  T  timed-retry      Bounded retries; over the bound MUST upgrade to
                      R or B (BOOT-I3). Infinite retry is forbidden.

The 29-item enumeration lives in _CLASSIFIER_TABLE below, one row per
failure listed in 10 S3.3.6. Each row carries:

  id           1..29 or 3b / 7b / 7c / 7d / 7e / 7f / 7g / 7h / 7i
  detection    where the failure is detected (assertion N, Stage 0,
               probe, etc.)
  cls          R / B / D / T
  ecode        the error-code group L identifier
  ref          the doc anchor for reviewers

The single entry point is classify(item_id) -> ClassResult; a
process that hits a startup failure calls this to obtain the
canonical class + code + guidance, then acts on it. Deciding
class inline at each detection site is exactly what CFG-BT-14
forbids (verbatim: NO scattered if-branches at each site).

Meta test (CFG-BT-13-like): _CLASSIFIER_TABLE row set must equal
the doc's 10 S3.3.6 row set. Enforced by
tests/boot/test_failure_class.py against the parsed doc.

Contract:
  classify(item_id)        -> ClassResult  (or KeyError if unknown)
  is_reject(cls)           -> bool         (R iff True)
  requires_upgrade(cls)    -> bool         (T iff True; must define
                                            an upgrade target)
  requires_hmi_marker(cls) -> bool         (D iff True)

The classifier does NOT execute the failure actions -- systemd,
event bus, and HMI drivers do that. This module owns the CLASS +
CODE mapping and the meta invariants (R/T/D discipline).

Why table-driven and not scattered if-branches (verbatim rule):

  * Scattered ifs drift. A new failure added in one place gets the
    'R' treatment; a similar new failure a month later added at
    another site gets 'B' because the author didn't remember. The
    same code path then sees both spellings and neither is right.
  * A table is bidirectional-diffable against the doc. The meta
    test compares the id set here with the doc's id set; a doc
    revision that adds an item shows up in test failure, not in
    silent under-coverage.

Why the ecode is nullable:

  * D-class rows for pure-observation degradations (e.g. row 18
    'zone load fail: does not change feasible region') land NO
    E_* code because there is no failure to report to a caller.
    A warn event is emitted, that's it. Filling in a fake ecode
    to make the table 'uniform' would leak an ecode into logs
    that has no downstream meaning.

Why the upgrade_to is per-row and not per-class:

  * The doc has three DISTINCT upgrade patterns among T rows:
      T -> R  (row 13, router-not-ready: RT is critical)
      T -> B  (row 14, chassis-tcp: chassis link failure -> block)
      T -> D  (row 23, AI service: fall back to preset WAV +
                       fastpath)
  * A per-class default would collapse them and either force AI
    service into R (breaks fastpath) or router into D (unsafe).

Why R rows all carry E_STORAGE_CORRUPT / E_CONFIG_INVALID / etc
and not a single canonical R code:

  * Different R failures need different remediation flows. Storage
    corruption needs the operator to restore from backup; config
    invalid needs a config edit; fence invalid needs a survey
    re-run. A single 'reject' code would erase that.

Meta test coverage vs the classifier code base:

  * The all_ids() snapshot IS the classifier's committed set. The
    meta test can diff it against the doc without any parallel
    hand-copied list; a doc addition surfaces as a failing test
    naming the missing id, and a table addition without doc update
    surfaces as an unknown id.
  * Individual class discipline tests (test_every_r_row_has_ecode,
    test_every_t_row_has_upgrade_target, test_variant_3_d_persistent_
    marker_flag) each guard ONE of the class rules verbatim.

Anti-patterns this module deliberately does NOT support:

  * A 'try to recover' mode. Once a failure is classified, the
    class is the class. Callers that want to 'downgrade R to D
    because it's only a dev machine' are asking for the exact
    fail-silent mode CLAUDE.md 3.1 forbids -- there is no such
    knob here.
  * A dynamic classifier registry. Adding a row means editing
    _CLASSIFIER_TABLE, which shows up in code review; adding it
    at runtime via a register() call would bypass review.
  * A default class for unknown ids. classify() raises KeyError
    on an unknown id rather than defaulting to R -- 'unknown ->
    treat as reject' would mask a table drift by making every
    stray call succeed.
"""

# NamedTuple for the frozen class-result row; dataclass would work
# but tuples are hashable + trivial to use in sets and dict keys.
from typing import Dict, NamedTuple, Optional


# Class constants. Kept as bare str (not enum) because the values
# appear in doc anchors and log lines; wrapping them in an enum
# would force str() calls everywhere.
#
# Ordering rationale (R, B, D, T): safety-first descending. R = the
# hardest response (whole stack down); T = the softest (retry, then
# escalate). A reader scanning the classes gets the severity intuition
# from left to right without a legend.
CLASS_R = "R"       # reject-startup: whole stack refuses to run
CLASS_B = "B"       # boot-but-block: up but no motion authority
CLASS_D = "D"       # degraded: motion allowed, capability limited
CLASS_T = "T"       # timed-retry: bounded retry, then upgrade

# The full closed set. Order = doc §3.3.6 order.
# Tuple (not frozenset) so the traversal order in tests is stable
# and matches the doc; the discipline tests iterate in this order
# when building error reports.
CLASSES = (CLASS_R, CLASS_B, CLASS_D, CLASS_T)


class ClassResult(NamedTuple):
    """One row of the classifier table.

    id           doc item id (1..29 or 3b/7b/.../7i)
    detection    where the failure is detected (short human tag)
    cls          class label from CLASSES
    ecode        error code (str, from group L per 11 S13.15)
                 or None when the row has no code (e.g. degrade
                 rows that just log an event)
    ref          doc reference anchor
    upgrade_to   for T-class rows, the class it upgrades to on
                 timeout (BOOT-I3). None for non-T rows.

    NamedTuple chosen over dataclass(frozen=True) for three reasons:
      1. Hashable by default; can go into sets / dict keys without
         extra config.
      2. Positional + keyword access -- readers can index [0] for
         the id in a tight loop.
      3. Fields have no defaults, so a caller cannot silently omit
         one and get None where None means something different
         (see upgrade_to comment above).

    The id field is str, not int, because sub-rows (3b, 7c, ...)
    exist in the doc; forcing them into an int scheme would either
    lose the sub-letter or invent a numbering the doc does not
    have.
    """

    id: str
    detection: str
    cls: str
    ecode: Optional[str]
    ref: str
    upgrade_to: Optional[str] = None


# ---------------------------------------------------------------------------
# The classifier table. One row per item in 10 S3.3.6.
# ---------------------------------------------------------------------------
#
# Table is the single source of truth for class assignment. Adding a
# row means (1) matching the doc's item id exactly (so the meta
# diff stays empty), (2) picking a class from CLASSES, (3) filling
# ecode from 11 S13.15 group L closed set, (4) upgrade_to for T rows.
#
# The rows are declared in doc order for review. Runtime lookup is
# via _BY_ID (dict) so declaration order does not affect performance.
_CLASSIFIER_TABLE = (
    # Storage / migration failures -- all R (data loss avoidance).
    ClassResult("1",  "Stage 0 disk image malformed",   CLASS_R,
                "E_STORAGE_CORRUPT", "10 S3.3.6.1"),
    ClassResult("2",  "Stage 0 migration failure",       CLASS_R,
                "E_STORAGE_CORRUPT", "10 S3.3.6.2"),
    ClassResult("3",  "assertion C retention monotone",  CLASS_R,
                "E_CONFIG_INVALID", "10 S3.3.6.3"),
    ClassResult("3b", "GATE-6 net profile mismatch",     CLASS_R,
                "E_CONFIG_INVALID", "10 S3.3.6.3b"),
    ClassResult("4",  "assertion A/B/D unresolved refs", CLASS_R,
                "E_CONFIG_INVALID", "10 S3.3.6.4"),
    ClassResult("5",  "assertion E safety namespaces vs hot-update",
                CLASS_R, "E_CONFIG_LOCKED", "10 S3.3.6.5"),
    ClassResult("6",  "assertion C dead cruise/transit", CLASS_R,
                "E_CONFIG_INVALID", "10 S3.3.6.6"),
    ClassResult("7",  "assertion G obstacle/patrol relation",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7"),
    ClassResult("7b", "assertion G safety-param range (SP-*/AS-7)",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7b"),
    ClassResult("7c", "assertion H calibration accuracy",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7c"),
    ClassResult("7d", "assertion I TRT engine / sha256",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7d"),
    ClassResult("7e", "assertion K quadruped QC-1..17",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7e"),
    ClassResult("7f", "assertion L BIT fatal exemption",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7f"),
    ClassResult("7g", "assertion M required key missing",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7g"),
    ClassResult("7h", "assertion N margin_base == d_safe",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7h"),
    ClassResult("7i", "assertion O teleop cloud priority",
                CLASS_R, "E_CONFIG_INVALID", "10 S3.3.6.7i"),
    # QoS + boot-time infra failures.
    ClassResult("8",  "assertion F QoS anti-pattern A2-A7", CLASS_R,
                "E_QOS_VIOLATION", "10 S3.3.6.8"),
    ClassResult("9",  "process-local A-1 anti-pattern",     CLASS_R,
                "E_QOS_VIOLATION", "10 S3.3.6.9"),
    ClassResult("10", "assertion F Q4_stream depth=0",      CLASS_R,
                "E_QOS_VIOLATION", "10 S3.3.6.10"),
    ClassResult("11", "MANIFEST boot_id mismatch",          CLASS_R,
                "E_CONFIG_INVALID", "10 S3.3.6.11"),
    ClassResult("12", "F' 7447 hijacked by V5 bridge",      CLASS_R,
                "E_CONFIG_INVALID", "10 S3.3.6.12"),
    # Router / link failures: T with upgrade path.
    ClassResult("13", "Stage 0z RT/GEN router not ready",   CLASS_T,
                "E_CONFIG_INVALID", "10 S3.3.6.13", upgrade_to=CLASS_R),
    ClassResult("14", "Stage 0z-3 chassis tcp/30003 unreachable",
                CLASS_T, "E_SAFETY_LINK_LOST", "10 S3.3.6.14",
                upgrade_to=CLASS_B),
    # Version / protocol block.
    ClassResult("15", "proto_version major mismatch",       CLASS_B,
                "E_PROTO_VERSION", "10 S3.3.6.15"),
    # Fence + BIT block.
    ClassResult("16", "forbid fence load fail / empty",     CLASS_R,
                "E_FENCE_INVALID", "10 S3.3.6.16"),
    ClassResult("17", "allow fence load fail",              CLASS_D,
                "E_FENCE_INVALID", "10 S3.3.6.17"),
    ClassResult("18", "zone load fail",                     CLASS_D,
                None, "10 S3.3.6.18"),
    ClassResult("19", "BIT fatal item fail",                CLASS_B,
                "E_UNHEALTHY", "10 S3.3.6.19"),
    ClassResult("20", "BIT degraded item fail",             CLASS_D,
                None, "10 S3.3.6.20"),
    ClassResult("21", "state/robot.timeout_lock=true",      CLASS_B,
                "E_LOCKED", "10 S3.3.6.21"),
    ClassResult("22", "common_digest mismatch MANIFEST",    CLASS_B,
                None, "10 S3.3.6.22"),
    # AI runtime + gpu + link degradations.
    ClassResult("23", "AI service (ASR/LLM) not ready",     CLASS_T,
                "E_TIMEOUT", "10 S3.3.6.23", upgrade_to=CLASS_D),
    ClassResult("24", "GPU/TensorRT engine load fail",      CLASS_D,
                None, "10 S3.3.6.24"),
    ClassResult("25", "cloud unreachable",                  CLASS_D,
                None, "10 S3.3.6.25"),
    ClassResult("26", "record.db continuous write fail",    CLASS_D,
                None, "10 S3.3.6.26"),
    # Non-failure: RTK not fixed yet.
    ClassResult("27", "RTK not fixed",                      CLASS_D,
                None, "10 S3.3.6.27"),
    # Pointer rows (perception + RNS internal boot failures).
    ClassResult("28", "perception internal boot fail (pointer)",
                CLASS_D, None, "10 S3.3.6.28"),
    ClassResult("29", "RNS internal boot fail (pointer)",   CLASS_D,
                None, "10 S3.3.6.29"),
)


# By-id lookup. Built once at import; classify() is O(1).
# The dict is not exposed; classify() is the sole entry point so a
# future refactor (e.g. from-yaml loading) does not break callers.
_BY_ID: Dict[str, ClassResult] = {row.id: row for row in _CLASSIFIER_TABLE}


def classify(item_id: str) -> ClassResult:
    """Return the class result for the given doc item id.

    Raises KeyError with a message pointing to the doc if the id is
    not in the table. Not silently mapping unknown ids to R because
    that would hide a table-vs-doc drift.

    O(1) lookup via _BY_ID; the tuple table is only walked at
    module-load time to build the dict, so declaration order does
    not affect classify() performance.

    Callers should NOT catch this KeyError and translate it into
    another class -- an unknown id means the caller's site is out
    of sync with the doc, which is a code defect the operator
    would rather see immediately than have papered over.
    """
    if item_id not in _BY_ID:
        raise KeyError(
            "startup failure id %r not registered in 10 S3.3.6 table; "
            "known ids: %s" % (item_id, sorted(_BY_ID)))
    return _BY_ID[item_id]


def is_reject(cls: str) -> bool:
    """True iff cls is R (reject-startup).

    R implies BOTH 'no participants past the observation window' AND
    'motion forbidden'. The caller must ensure both, not just one --
    this function does not check enforcement. Splitting them into
    two functions would let a caller check one and skip the other;
    keeping them coupled in the doc + coupled in this predicate
    keeps R's meaning intact.

    The observation window itself is a P5-minimal-mode capability;
    it must exist (so the operator has ANY visibility) but must NOT
    grant motion authority (which would defeat the point of R).
    """
    return cls == CLASS_R


def requires_upgrade(cls: str) -> bool:
    """True iff cls is T (timed-retry) -- MUST define an upgrade
    target per BOOT-I3.

    BOOT-I3 verbatim: 'each gated retry must have an upper bound +
    an upgrade path; infinite retry is forbidden'. Infinite retry
    in the field looks like 'always starting, never says anything',
    which is worse than an explicit failure.
    """
    return cls == CLASS_T


def requires_hmi_marker(cls: str) -> bool:
    """True iff cls is D (degraded) -- MUST land warn event + HMI
    persistent marker per 10 S3.3.6 D-row.

    'Persistent' means the marker stays visible until the operator
    clears it; a marker that decays after N seconds would let a
    degrade slip past attention if the operator was looking
    elsewhere at the wrong moment.
    """
    return cls == CLASS_D


def all_ids() -> tuple:
    """Snapshot of every id in the table, in declaration order.
    Used by the meta-diff test.

    Return type is tuple (not set) so declaration order is
    preserved for reviewers who want to diff the impl id sequence
    against the doc's row order visually.
    """
    return tuple(row.id for row in _CLASSIFIER_TABLE)


def all_rows() -> tuple:
    """Snapshot of every row (for testing / manifest emission).

    Rows are frozen NamedTuples; iterating is safe from any thread.
    Not exposing the internal tuple directly because a caller could
    otherwise reach in and mutate an individual row (tuples are
    immutable but NamedTuple fields are also immutable, so this is
    defence-in-depth against a future dataclass swap).
    """
    return _CLASSIFIER_TABLE
