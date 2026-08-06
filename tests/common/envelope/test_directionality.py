"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_directionality.py
Brief: INF-CM-2 criterion three -- tightening executes, loosening rejects

Description:
Pins the S3.0.1 fail-safe: a malformed message on a collapse-safe (tightening)
key is executed, not rejected, while a malformed loosening message is rejected
with E_SCHEMA. The two directions are asserted separately, per the criterion.
Mutation five -- a tightening parse failure rejecting instead of stopping -- lives
here, together with the U75 point that the exemption is collapse-safety and not a
particular key.
"""

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.envelope import (  # noqa: E402
    Direction,
    Disposition,
    EnvelopeSchemaError,
    guarded_decode,
    is_collapse_safe,
)

# A valid on-host envelope, as bytes, for the positive control. A cmd/estop
# payload would carry action "stop" in data; the envelope layer does not read
# data, so an empty data is enough to exercise the accept path.
_VALID_BYTES = (
    b'{"v":1,"rid":"dog-01","ts":1.0,"mono":2.0,"boot":"9f2c1a44",'
    b'"seq":1,"src":"p5_gateway","ts_sync":true,"data":{}}'
)

# Every flavour of decode failure S3.0.1 names, so the fail-safe is tested against
# all of them and not just one. Truncated JSON and a bad UTF-8 byte are the
# "JSON 截断 / 编码错误" cases; the others are schema violations.
_MALFORMED = [
    b'{ this is not json',                              # truncated / unparsable
    b'\xff\xfe not utf-8 at all',                       # encoding error
    b'{"v":99,"rid":"d","ts":1.0,"seq":1,"src":"s","ts_sync":true,"data":{}}',  # unknown v
    b'{"v":1,"rid":"d","ts":1.0,"seq":1,"ts_sync":true,"data":{}}',             # missing src
    b'[1,2,3]',                                         # valid JSON, not an object
]


def test_is_collapse_safe_is_true_only_for_tightening():
    """The one-line decision primitive: TIGHTENING collapses safe, LOOSENING not.

    Asserting both directions pins that the function is a real discriminator and
    not a constant. Mutation five flips the TIGHTENING result to False, which
    this catches directly.
    """
    assert is_collapse_safe(Direction.TIGHTENING) is True
    assert is_collapse_safe(Direction.LOOSENING) is False


@pytest.mark.parametrize("raw", _MALFORMED, ids=lambda b: repr(b[:20]))
def test_tightening_parse_failure_is_collapse_safe_not_reject(raw):
    """*** Mutation five: a malformed collapse-safe message executes, not rejects.

    For a tightening key (cmd/estop, behavior/request cancel), every kind of
    decode failure -- unparsable, mis-encoded, unknown v, missing field, wrong
    root type -- must yield a COLLAPSE_SAFE disposition so the caller applies its
    stop / cancel. The mutation makes the tightening branch reject instead; then
    guarded_decode would RAISE here, and pytest.raises is not wrapping this call,
    so the raise fails the test. That is the red.
    """
    result = guarded_decode(raw, Direction.TIGHTENING)
    assert result.disposition is Disposition.COLLAPSE_SAFE
    # No envelope is produced on the collapse-safe path: the bytes never parsed,
    # so there is nothing to hand back, and the caller acts on the disposition.
    assert result.envelope is None


@pytest.mark.parametrize("raw", _MALFORMED, ids=lambda b: repr(b[:20]))
def test_loosening_parse_failure_is_rejected_with_e_schema(raw):
    """The other half of criterion three: a malformed loosening message rejects.

    enable, mode switch, any release -- E-3 and S3.0.1 require full validation and
    a rejection on failure, never a best-effort execute. The rejection carries
    E_SCHEMA whether the failure was a raw parse error (wrapped) or a schema
    violation (re-raised).
    """
    with pytest.raises(EnvelopeSchemaError) as exc:
        guarded_decode(raw, Direction.LOOSENING)
    assert exc.value.code == "E_SCHEMA"


def test_a_valid_envelope_is_accepted_on_both_directions():
    """The positive control: well-formed bytes decode and are ACCEPTED.

    Without this, both parametrized suites above could pass on an implementation
    that never accepts anything -- collapse-safe-everything on one side, reject-
    everything on the other. A valid envelope must come back ACCEPTED with the
    parsed Envelope, regardless of direction, because direction only changes the
    FAILURE handling.
    """
    for direction in (Direction.TIGHTENING, Direction.LOOSENING):
        result = guarded_decode(_VALID_BYTES, direction)
        assert result.disposition is Disposition.ACCEPTED
        assert result.envelope is not None
        assert result.envelope.src == "p5_gateway"


def test_guarded_decode_accepts_an_already_parsed_dict():
    """A caller that already has a dict can pass it; only bytes / str are parsed.

    guarded_decode routes a dict straight to decode and only json.loads a
    bytes / str, so a consumer holding a parsed object need not re-serialise it.
    """
    obj = {"v": 1, "rid": "dog-01", "ts": 1.0, "seq": 1, "src": "cloud",
           "ts_sync": False, "data": {}}   # a cloud dict, no mono
    result = guarded_decode(obj, Direction.LOOSENING)
    assert result.disposition is Disposition.ACCEPTED
    assert result.envelope.mono is None


def test_a_programming_defect_is_not_swallowed_by_the_tightening_path():
    """The tightening branch catches decode FAILURES, not arbitrary bugs.

    CLAUDE.md 4.5: a real defect must reach the fault path untouched. Passing an
    int (neither dict nor bytes / str) makes json.loads raise TypeError, which is
    NOT in guarded_decode's except list, so it propagates even under TIGHTENING --
    it is not silently turned into a collapse-safe stop.
    """
    with pytest.raises(TypeError):
        guarded_decode(12345, Direction.TIGHTENING)  # type: ignore[arg-type]
