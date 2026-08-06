"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: envelope.py
Brief: The nine-field Zenoh envelope -- decode with E_SCHEMA, encode back out

Description:
What this solves. Every JSON payload on either Zenoh plane is wrapped in the
outer envelope of 11 S3.0: v / rid / ts / mono / boot / seq / src / ts_sync /
data. A consumer that hand-reads those keys, each with its own idea of which are
required and what an absent ts_sync means, is how two processes end up disagreeing
about whether a message is even well formed -- and the split only shows up during
integration, as one process rejecting frames another accepts. This module is the
single decode point, so the presence rules and their two documented exceptions
live in exactly one place.

Where the rules come from, and why the TODO shorthand is not the authority.
INF-CM-2's criterion one says "nine fields, missing any -> E_SCHEMA, ts_sync
missing -> false". Read literally that is self-contradictory (ts_sync is one of
the nine, yet its missing case is false rather than E_SCHEMA) and it also
collides with criterion two, which treats a cloud packet with mono absent as a
normal fallback rather than a rejection. The authority is 11 S3.0's field table
read together with its own inline notes, plus the frozen structure statement
F-2 (11 S14.2) and CLK-C4 (11 S0.2.1). Those three agree, and this module
follows them:

  * v rid ts seq src data  -- unconditionally required. Absent or wrong type is
    a structural violation -> E_SCHEMA (S13.6 lists 必填字段缺失 under E_SCHEMA).
    v must additionally be a KNOWN schema version; S3.0 says an unrecognised v
    must be rejected, not guessed.
  * ts_sync  -- F-2 verbatim: 缺省/缺失一律 false. NOT rejected when absent. And
    it is trusted true ONLY when it is literally boolean true: rtk_driver is the
    sole authority (CLK-A1), a consumer never upgrades the flag, and every other
    shape (absent, false, a string, a number) is the fail-safe false.
  * mono  -- OPTIONAL. CLK-C4 requires cross-host publishers (cloud / HMI /
    WeChat) to OMIT it, so its absence is the ordinary cloud case, and the age
    layer falls back to receive time. Present-and-null is treated the same as
    absent, because the S3.0.1 pseudocode branches on "mono != null" and a cloud
    packet carries no monotonic reading either way.
  * boot  -- required exactly when mono is present (S3.0: mono 存在时必填). mono
    present with boot absent is a malformed on-host envelope -> E_SCHEMA. boot
    present without mono is harmless and ignored: boot only means anything as the
    validity domain of a mono reading (CLK-C4), and there is no mono to validate.
  * unknown fields  -- ignored, per F-2 (未知字段必须忽略). Reading only the known
    keys is what makes that true; do NOT add a strict-unknown-key rejection, it
    would reject any forward-compatible producer the moment a field is added.

What this module deliberately does NOT do:
  * It does not compute message age. That is age.py, because age has four
    branches that need the receiver's rx_mono and local boot id, none of which
    are envelope fields (11 S3.0.1).
  * It does not apply the tightening / loosening fail-safe. That is
    directionality.py: whether a decode failure becomes a stop or a rejection is
    a property of the KEY the message arrived on, which this layer does not know.
    decode() here is the strict path both the loosening rule and the general rule
    use; the tightening exemption wraps it.
  * It does not rewrite ts_sync for inbound cloud messages. CLK-A4 gives that to
    p5_gateway alone. This module records what the wire said (fail-safe); it does
    not second-guess it upward.
  * It does not validate rid against a robot-id registry, nor src against the
    publisher-binding table. Those are separate contracts (S8.8.1 publisher
    binding, V-3 cross-rid comparison) and belong to their owners.

Traps that look right and are not:
  1. Defaulting ts_sync to true when absent. It reads like the friendly choice
     and it is the exact fail-open S1.5.4 changed away from: an unsynced peer
     then presents as synced, and the clock hard-cap that should drop the robot
     to 0.5 m/s never engages. Missing is false, always.
  2. Rejecting a message because mono is absent. That rejects every cloud task,
     because CLK-C4 makes cloud OMIT mono. Absence of mono is legal; absence of
     boot WHEN mono is present is not.
  3. Using 0.0 or "" as an absent marker while reading raw keys. A real seq can
     be 0 (S3.0: 进程重启从 0 开始) and a real reading can be small, so a falsy
     value is indistinguishable from a missing one. _MISSING is a distinct object
     for exactly the reason config/merge.py keeps its own: absent must be
     tellable from present-and-falsy, or a required-field check passes on a field
     that was never sent.
"""

from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Optional

# E_SCHEMA is imported as a NAME, never written as the literal string. CLAUDE.md
# 3.5 forbids the literal outside common/errors/, and the payoff is immediate: a
# misspelling is an ImportError here rather than a code the cloud client silently
# fails to branch on. XbrainError is the family base so "except XbrainError"
# still catches an envelope failure alongside a closed-set one.
from ..errors import E_SCHEMA
from ..errors.exceptions import XbrainError

# A sentinel that is neither None nor any JSON value. object() has no equal but
# itself, so raw.get(key, _MISSING) is _MISSING answers "the key was absent"
# without colliding with a present null, a present 0, or a present "". This is
# the same device config/merge.py exports as MISSING; a local one is used rather
# than importing that module so the envelope layer does not depend on the config
# loader for a one-object sentinel.
_MISSING: Any = object()

# The known schema versions. S3.0 states v = 1 is the first and current version
# ("变更并入 v = 1 首版"), so the set is exactly {1} today. It is a frozenset and
# not a bare "== 1" so that adding v = 2 after a freeze is a one-line data change
# with an obvious diff, and so the reject-unknown-v rule reads as membership
# rather than as an inequality a later edit could loosen to "v >= 1".
#
# !! This is a schema version, taken verbatim from S3.0, NOT a tunable safety
# parameter, so hardcoding {1} is not the CLAUDE.md 3.1 defect: there is no
# calibration pending and no configuration key for it.
KNOWN_VERSIONS: FrozenSet[int] = frozenset({1})

# The unconditionally required keys, named once. ts_sync, mono and boot are
# deliberately ABSENT from this tuple -- each carries a documented exception
# handled explicitly in decode(), and folding them in here would reintroduce the
# contradiction the module docstring untangles.
_REQUIRED: tuple = ("v", "rid", "ts", "seq", "src", "data")


class EnvelopeSchemaError(XbrainError):
    """A payload that does not satisfy the 11 S3.0 envelope structure.

    Carries code E_SCHEMA -- S13.6's meaning row lists v 不认识, 必填字段缺失 and
    枚举值越界 together, which is exactly this class of failure. A named type
    rather than a bare XbrainError so a caller can catch precisely the envelope
    case; CLAUDE.md 4.5 forbids raising the bare base.
    """

    # detail is passed straight through to XbrainError, which keeps it by
    # reference. The field name that failed goes in the message rather than in a
    # structured detail dict, because S13.6 marks E_SCHEMA's detail requirement
    # "unspecified" (see codes.yaml) -- inventing a required detail schema here
    # would be this module deciding a contract question that is not its to decide.
    def __init__(self, message: str, detail: Optional[dict] = None):
        super().__init__(E_SCHEMA, message, detail)


# frozen so a decoded envelope cannot be edited after the fact. A consumer that
# mutated env.ts_sync would be doing CLK-A4's job (rewriting the sync flag) in the
# wrong place and without the authority; freezing makes that a runtime error
# rather than a silent one. eq is left on (dataclass default) so golden tests can
# compare two decodes for equality.
@dataclass(frozen=True)
class Envelope:
    """One decoded 11 S3.0 envelope.

    mono and boot are Optional because a cloud packet legitimately carries
    neither (CLK-C4). Every other field is always present on a decoded envelope
    -- decode() would have raised otherwise.
    """

    v: int                  # schema version; only a KNOWN_VERSIONS value decodes
    rid: str                # robot id; the only cross-host discriminator (V-3)
    ts: float               # wall clock, S3.0: alignment / recording / latency ONLY
    mono: Optional[float]   # monotonic; None for a cross-host (cloud) publisher
    boot: Optional[str]     # boot-id domain of mono; present iff mono is present
    seq: int                # per-key uint64, restart-relative; the resend cursor (U18)
    src: str                # producer process name; the publisher-binding key (S8.8.1)
    ts_sync: bool           # trusted true only when the wire said boolean true
    data: Dict[str, Any]    # the payload; every S3.x body is a JSON object


def _require(raw: Dict[str, Any], key: str) -> Any:
    """Fetch a required key or raise EnvelopeSchemaError naming it.

    The message names the field AND the section, because "envelope rejected" with
    no field is the kind of log line that sends an operator reading a 25000-line
    contract by hand. Present-and-null counts as absent for a required field: a
    null v or null src is not a usable value, and S13.6 groups a missing required
    field with a malformed one under the same E_SCHEMA.
    """
    value = raw.get(key, _MISSING)
    if value is _MISSING or value is None:
        raise EnvelopeSchemaError(
            "envelope missing required field %r (11 S3.0)" % key
        )
    return value


def decode(raw: Dict[str, Any]) -> Envelope:
    """Validate a parsed JSON object against 11 S3.0 and return an Envelope.

    Raises EnvelopeSchemaError (code E_SCHEMA) on any structural violation. This
    is the STRICT path: it makes no allowance for direction. The tightening
    fail-safe of S3.0.1 wraps this function in directionality.py rather than
    living inside it, because whether a rejection should instead become a stop
    depends on the key, which this layer does not see.
    """
    # A non-object payload is a structural failure, not a crash. json.loads can
    # return a list, a number or a string for perfectly valid JSON that is simply
    # not an envelope; naming that as E_SCHEMA keeps it in the same failure class
    # a consumer already handles, rather than surfacing as a TypeError from the
    # first .get below.
    if not isinstance(raw, dict):
        raise EnvelopeSchemaError(
            "envelope must be a JSON object, got %s (11 S3.0)"
            % type(raw).__name__
        )

    # v first, and checked for KNOWN membership rather than mere presence. S3.0:
    # 接收方遇到不认识的 v 必须拒绝并告警, 不得猜测解析. An out-of-set v is rejected
    # here for the general/loosening path; the tightening wrapper is what turns
    # that rejection into a stop for cmd/estop, and it can only do so because this
    # raises rather than guessing.
    v = _require(raw, "v")
    # bool excluded before int: True is an int in Python, and an envelope with
    # "v": true must not be read as version 1. isinstance(v, bool) would let it
    # through the int branch below, so it is rejected first.
    if isinstance(v, bool) or not isinstance(v, int) or v not in KNOWN_VERSIONS:
        raise EnvelopeSchemaError(
            "envelope carries unknown schema version %r; known: %s (11 S3.0)"
            % (v, sorted(KNOWN_VERSIONS))
        )

    # rid and src are identifiers; ts is a wall-clock float; seq is a uint64
    # counter. Each is required and type-checked, because S13.6 puts a wrong-typed
    # required field in the same E_SCHEMA class as a missing one, and a consumer
    # subtracting a string ts downstream would fail far from here.
    rid = _require(raw, "rid")
    if not isinstance(rid, str):
        raise EnvelopeSchemaError("envelope rid must be a string (11 S3.0)")

    ts = _require(raw, "ts")
    # bool before (int, float) for the same reason as v. A numeric ts is required;
    # int is accepted because a whole-second timestamp is a legal JSON integer and
    # rejecting it would reject a valid producer.
    if isinstance(ts, bool) or not isinstance(ts, (int, float)):
        raise EnvelopeSchemaError("envelope ts must be a number (11 S3.0)")

    seq = _require(raw, "seq")
    # seq is uint64 and starts at 0 after a restart (S3.0), so 0 is a real value
    # and _require's present-and-null guard is what rejects a null seq without
    # rejecting a zero one. Negative is out of range for a uint64 counter.
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise EnvelopeSchemaError("envelope seq must be a non-negative integer (11 S3.0)")

    src = _require(raw, "src")
    if not isinstance(src, str):
        raise EnvelopeSchemaError("envelope src must be a string (11 S3.0)")

    data = _require(raw, "data")
    # data is the payload object. Empty is fine ({} is a valid, if unusual,
    # payload); non-object is not, because every S3.x message body is a JSON
    # object and a list or scalar here means the envelope was assembled wrong.
    if not isinstance(data, dict):
        raise EnvelopeSchemaError("envelope data must be a JSON object (11 S3.0)")

    # ts_sync: the one required-column field that is NOT rejected when absent.
    # F-2 verbatim: 缺省/缺失一律 false. The missing branch defaults to False on
    # its own line so the fail-open mutation (defaulting to True) is a single
    # visible edit -- that is INF-CM-2's mutation two.
    present = raw.get("ts_sync", _MISSING)
    if present is _MISSING:
        ts_sync = False
    elif isinstance(present, bool):
        # Trusted exactly as sent when it is a real boolean. This is the only path
        # that can yield True, which is the point: only rtk_driver's copied-through
        # boolean reaches here as true (CLK-A2), never a coerced truthy value.
        ts_sync = present
    else:
        # A non-boolean ts_sync (a string "true", a 1) is not trusted. S1.5.2's
        # fail-safe direction says an ambiguous sync flag is false, never a
        # best-effort true, so a malformed producer degrades safe rather than open.
        ts_sync = False

    # mono / boot travel together. mono absent (or JSON null) is the cloud case:
    # store None and let age.py fall back to receive time. mono present must be a
    # number AND must be accompanied by boot (S3.0: boot required when mono
    # present); the pair is what CLK-C4 needs to decide the reading is comparable.
    raw_mono = raw.get("mono", _MISSING)
    if raw_mono is _MISSING or raw_mono is None:
        mono: Optional[float] = None
        # boot without mono is not an error -- boot is only the validity domain of
        # a mono reading, and there is none. It is read but ignored, so a cloud
        # packet that happens to carry a stray boot decodes cleanly.
        boot: Optional[str] = None
    else:
        if isinstance(raw_mono, bool) or not isinstance(raw_mono, (int, float)):
            raise EnvelopeSchemaError("envelope mono must be a number when present (11 S3.0)")
        mono = float(raw_mono)
        raw_boot = raw.get("boot", _MISSING)
        # boot is mandatory here and must be a string. Without it the receiver
        # cannot tell whether mono is from this boot (CLK-C4), so a mono with no
        # boot is a structural violation rather than a treat-as-cloud fallback:
        # falling back would silently discard a monotonic reading that WAS sent.
        if raw_boot is _MISSING or raw_boot is None or not isinstance(raw_boot, str):
            raise EnvelopeSchemaError(
                "envelope carries mono but no string boot; boot is required when "
                "mono is present (11 S3.0)"
            )
        boot = raw_boot

    return Envelope(
        v=v, rid=rid, ts=float(ts), mono=mono, boot=boot,
        seq=seq, src=src, ts_sync=ts_sync, data=data,
    )


def encode(env: Envelope) -> Dict[str, Any]:
    """Serialise an Envelope back to a plain dict ready for JSON.

    The inverse of decode() for the cases decode() accepts. mono and boot are
    emitted only when present, so a cloud envelope round-trips WITHOUT them and an
    on-host envelope round-trips WITH the pair -- matching CLK-C4 rather than
    forcing a null mono onto a cross-host message, which CLK-C4 says to omit.

    rx_mono is deliberately never emitted: it is a receiver-side annotation the
    age layer stamps locally, not a wire field (S3.0.1), so it has no place in the
    encoded envelope.
    """
    # ts_sync is always written -- it is a required field and its wire presence is
    # what lets the next hop apply the same missing-is-false rule this module
    # does. Writing it unconditionally is not the same as trusting it: the value
    # emitted is whatever decode() recorded, fail-safe included.
    #
    # The seven always-present fields are written first; mono / boot follow only
    # when present. Order in the dict does not matter for JSON, but it is written
    # to mirror the S3.0 field table so a reader diffing the encoded form against
    # the contract sees the same sequence.
    out: Dict[str, Any] = {
        "v": env.v,
        "rid": env.rid,
        "ts": env.ts,
        "seq": env.seq,
        "src": env.src,
        "ts_sync": env.ts_sync,
        "data": env.data,
    }
    # Emitted as a pair or not at all. The Envelope invariant from decode() is
    # that boot is present whenever mono is; this mirrors it on the way out so an
    # encode/decode round trip is stable.
    if env.mono is not None:
        out["mono"] = env.mono
        out["boot"] = env.boot
    return out


__all__ = ["Envelope", "EnvelopeSchemaError", "decode", "encode",
           "KNOWN_VERSIONS"]
