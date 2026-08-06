"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_envelope.py
Brief: INF-CM-2 criterion one -- nine-field decode, ts_sync fail-safe, encode

Description:
Holds the decode / encode half of INF-CM-2 against 11 S3.0 and F-2 (11 S14.2):
a missing unconditionally-required field is E_SCHEMA; ts_sync missing is false and
NEVER true (mutation two lives here); a cloud packet with no mono / boot decodes
rather than being rejected; unknown fields are ignored; an unknown schema version
is rejected. Every test names the behaviour it pins so a reader can see which
mutation it would catch.
"""

import os
import sys

import pytest

# Four dirnames up from tests/common/envelope/ is the repo root, matching the
# sys.path dance the sibling suites use. tests/ carries no __init__.py, so the
# package is imported by putting the root on the path rather than by relative
# import -- the same choice test_digest.py makes.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.envelope import (  # noqa: E402
    Envelope,
    EnvelopeSchemaError,
    KNOWN_VERSIONS,
    decode,
    encode,
)


def _onhost():
    """A complete, valid on-host envelope -- mono and boot present.

    Built fresh per call so a test that mutates its copy cannot leak into the
    next. This is the shape every required-field test strips one key from.
    """
    return {
        "v": 1, "rid": "dog-01", "ts": 1753660800.123456,
        "mono": 4821.337215, "boot": "9f2c1a44", "seq": 12345,
        "src": "perception", "ts_sync": True, "data": {"min_dist_m": 3.2},
    }


def _cloud():
    """A valid cloud envelope -- mono and boot deliberately absent (CLK-C4)."""
    # A cross-host publisher MUST omit mono; this is the packet p5_gateway
    # forwards after filling seq / src / ts_sync. It has to decode, not reject.
    return {
        "v": 1, "rid": "dog-01", "ts": 1753660800.5, "seq": 2,
        "src": "p5_gateway", "ts_sync": False, "data": {},
    }


def test_a_complete_onhost_envelope_decodes():
    """The positive control: a well-formed envelope round-trips its fields.

    Without this, every rejection test below could pass on a decoder that
    rejects everything -- the always-red failure mode CLAUDE.md 3.2 names.
    """
    env = decode(_onhost())
    # Spot-check the fields whose types the decoder coerces or validates.
    assert env.v == 1 and env.rid == "dog-01"
    assert env.mono == 4821.337215 and env.boot == "9f2c1a44"
    assert env.ts_sync is True
    assert env.data == {"min_dist_m": 3.2}
    # The result is frozen: a consumer cannot rewrite ts_sync (that is CLK-A4's
    # job, at the gateway, not a consumer's). Assigning raises FrozenInstanceError.
    with pytest.raises(Exception):
        env.ts_sync = False  # type: ignore[misc]


@pytest.mark.parametrize("field", ["v", "rid", "ts", "seq", "src", "data"])
def test_missing_unconditionally_required_field_is_e_schema(field):
    """Criterion one: dropping any of the six always-required fields -> E_SCHEMA.

    mono / boot / ts_sync are excluded from this list on purpose -- each carries
    a documented exception tested separately -- so this parametrization pins the
    fields that have NO exception.
    """
    raw = _onhost()
    del raw[field]
    # The code carried is E_SCHEMA (S13.6). Asserting on .code rather than on the
    # message keeps the test stable against wording changes.
    with pytest.raises(EnvelopeSchemaError) as exc:
        decode(raw)
    assert exc.value.code == "E_SCHEMA"


@pytest.mark.parametrize("field", ["v", "rid", "ts", "seq", "src", "data"])
def test_present_but_null_required_field_is_e_schema(field):
    """A present-but-null required field is rejected too, not read as a value.

    _require treats null the same as absent for a required field: a null src is
    not a usable producer name. This is why _MISSING and None are both caught.
    """
    raw = _onhost()
    raw[field] = None
    with pytest.raises(EnvelopeSchemaError):
        decode(raw)


def test_ts_sync_missing_is_false_never_true():
    """*** Mutation two: ts_sync absent must be false; the mutant defaults true.

    F-2 verbatim: 缺省/缺失一律 false. The mutation named by INF-CM-2 changes the
    missing-branch default in envelope.decode from False to True. This asserts
    False, so that edit turns this red. The fail-open it prevents (S1.5.4) is an
    unsynced peer presenting as synced, after which the clock speed cap never
    engages.
    """
    raw = _onhost()
    del raw["ts_sync"]
    assert decode(raw).ts_sync is False


@pytest.mark.parametrize("bad", [False, "true", 1, 0, "yes", None])
def test_ts_sync_is_true_only_when_the_wire_says_boolean_true(bad):
    """Only a literal boolean true is trusted; every other shape is false.

    S1.5.2's fail-safe direction: an ambiguous or non-boolean sync flag degrades
    to false, never to a best-effort true. A string "true" and an int 1 are the
    tempting truthy values that must NOT read as synced.
    """
    raw = _onhost()
    raw["ts_sync"] = bad
    assert decode(raw).ts_sync is False


def test_ts_sync_present_true_is_kept():
    """The one path that yields True: an actual boolean true survives."""
    # Balances the test above so the rule is not merely "always false" -- a
    # decoder hardwired to false would pass every case above and be wrong.
    raw = _onhost()
    raw["ts_sync"] = True
    assert decode(raw).ts_sync is True


def test_cloud_packet_without_mono_decodes_and_has_none_mono():
    """Criterion two support: mono absent is the cloud case, not a rejection.

    CLK-C4 requires cross-host publishers to omit mono. A decoder that required
    it would reject every cloud task; instead mono and boot come back None and
    the age layer falls back to receive time.
    """
    env = decode(_cloud())
    assert env.mono is None and env.boot is None


def test_mono_present_without_boot_is_e_schema():
    """boot is required exactly when mono is present (S3.0).

    A mono with no boot cannot be judged comparable (CLK-C4), and silently
    treating it as a cloud fallback would DISCARD a monotonic reading that was
    actually sent -- so it is a structural violation, not a fallback.
    """
    raw = _onhost()
    del raw["boot"]
    with pytest.raises(EnvelopeSchemaError):
        decode(raw)


def test_boot_without_mono_is_ignored_not_rejected():
    """A stray boot on a cloud packet is harmless: boot only qualifies a mono.

    The contract makes boot meaningful only as mono's validity domain, so a
    cloud packet that happens to carry boot but no mono still decodes, with mono
    None. Rejecting it would invent a rule S3.0 does not state.
    """
    raw = _cloud()
    raw["boot"] = "9f2c1a44"
    env = decode(raw)
    assert env.mono is None


@pytest.mark.parametrize("bad_v", [0, 2, 99, "1", 1.0, True])
def test_unknown_or_mistyped_schema_version_is_rejected(bad_v):
    """S3.0: an unrecognised v must be rejected, not guessed.

    Covers out-of-set ints (0, 2, 99), a string "1", a float 1.0, and True --
    True is an int in Python and must not sneak through as version 1, which is
    why the decoder checks bool before int.
    """
    raw = _onhost()
    raw["v"] = bad_v
    with pytest.raises(EnvelopeSchemaError):
        decode(raw)
    # Guard the guard: 1 really is in the known set, so the rejections above are
    # about the bad values and not about an empty set that rejects everything.
    assert 1 in KNOWN_VERSIONS


def test_unknown_fields_are_ignored():
    """F-2: 未知字段必须忽略. An extra key does not reject and does not appear.

    A forward-compatible producer that adds a field must still decode on an older
    consumer; a strict-unknown-key rejection would break that the day any field
    is added.
    """
    raw = _onhost()
    raw["orig_ts"] = 1753660800.0   # a real forwarding field (RT-C3.e), not modelled here
    raw["some_future_field"] = 42
    env = decode(raw)
    # It decoded, and the Envelope has no attribute for the extras -- they were
    # dropped, not smuggled onto data.
    assert env.v == 1
    assert "some_future_field" not in env.data


def test_non_object_payload_is_e_schema():
    """A valid-JSON non-object (list, number, string) is not an envelope."""
    # json.loads can return any of these for perfectly valid JSON; each must be
    # E_SCHEMA rather than a TypeError from the first field access.
    for payload in ([1, 2, 3], 7, "hello", None):
        with pytest.raises(EnvelopeSchemaError):
            decode(payload)  # type: ignore[arg-type]


def test_encode_is_the_inverse_for_an_onhost_envelope():
    """encode(decode(x)) reproduces every field of a well-formed on-host packet."""
    raw = _onhost()
    # Round-trip equals the input EXCEPT that decode coerces ts to float and
    # drops unknown keys; _onhost has none, and its ts is already a float, so the
    # dicts compare equal here.
    assert encode(decode(raw)) == raw


def test_encode_omits_mono_and_boot_for_a_cloud_envelope():
    """A cloud envelope round-trips WITHOUT mono / boot, per CLK-C4.

    encode must not force a null mono onto a cross-host message; CLK-C4 says to
    omit it. So the encoded cloud packet has neither key.
    """
    out = encode(decode(_cloud()))
    assert "mono" not in out and "boot" not in out
    # And ts_sync is always written -- it is required on the wire so the next hop
    # can apply the same missing-is-false rule.
    assert out["ts_sync"] is False


def test_encode_emits_mono_and_boot_as_a_pair():
    """When mono is present, encode writes both mono and boot together."""
    out = encode(decode(_onhost()))
    assert out["mono"] == 4821.337215 and out["boot"] == "9f2c1a44"


def test_envelope_is_constructible_directly_for_downstream_producers():
    """The frozen dataclass can be built directly, e.g. by an encoder-side test.

    Not every Envelope comes from decode(); a producer builds one to encode. This
    pins that the field order and names are the public shape they are documented
    as, so encode() below is exercised on a hand-built value too.
    """
    env = Envelope(v=1, rid="dog-01", ts=1.0, mono=2.0, boot="9f2c1a44",
                   seq=0, src="p1_motion", ts_sync=True, data={})
    # seq 0 is a real value (S3.0: restart starts at 0), so it must survive encode
    # rather than being mistaken for absent.
    assert encode(env)["seq"] == 0
