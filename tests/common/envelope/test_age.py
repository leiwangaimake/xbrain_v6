"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_age.py
Brief: INF-CM-2 criterion two -- message_age_s four branches and CLK-C5 event

Description:
Pins the S3.0.1 age computation branch by branch against the shared golden
vectors, and carries three of INF-CM-2's five named mutations: age computed from
ts instead of mono (mutation one), the mono branch kept when boot mismatches
(mutation three), and a negative age clamped WITHOUT emitting the event (mutation
four). read_local_boot_id is exercised against a fixture UUID rather than the real
/proc so the test is deterministic.
"""

import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

from xbrain.common.envelope import (  # noqa: E402
    BRANCH_PRODUCED,
    BRANCH_RX_FALLBACK,
    NEGATIVE_AGE_CAT,
    NEGATIVE_AGE_KIND,
    NEGATIVE_AGE_SEV,
    compute_age,
    decode,
    message_age_s,
    read_local_boot_id,
)

# The one shared source of truth for both the Python branch tests here and the
# C++ cross-language test next door. Loaded once at module import.
GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden",
                      "message_age_vectors.json")


def _vectors():
    """The golden vectors as a list of dicts."""
    # Read fresh rather than cached in a global so a test cannot mutate the shared
    # list out from under another.
    with open(GOLDEN, encoding="utf-8") as fh:
        return json.load(fh)["vectors"]


def _fmt(value):
    """Render a computed age the way the golden pins it: %.17g.

    The same rule canonical.py uses and the C++ FormatAge mirrors, so a Python
    age and a golden string compare as text without a tolerance -- the point being
    that the value is exact under the chosen dyadic inputs.
    """
    return "%.17g" % value


@pytest.mark.parametrize("vec", _vectors(), ids=lambda v: v["name"])
def test_message_age_matches_the_golden_vector(vec):
    """Every branch of S3.0.1, driven from the golden file.

    Decodes the vector's real JSON envelope (so decode and compute are exercised
    together, honouring "same JSON" of criterion four) and checks the branch, the
    negative flag, the raw age and the clamped age against the hand-verified
    expectations.
    """
    env = decode(vec["envelope"])
    result = compute_age(env, rx_mono=vec["rx_mono"], now_mono=vec["now_mono"],
                         local_boot_id=vec["local_boot_id"])
    # Branch first: a wrong branch is the root cause a wrong number is a symptom
    # of, so naming it makes a failure diagnosable.
    assert result.branch == vec["expected_branch"]
    assert result.was_negative is vec["expected_negative"]
    assert _fmt(result.raw_age_s) == vec["expected_raw"]
    assert _fmt(result.age_s) == vec["expected_age"]


def test_every_golden_vector_was_actually_run():
    """Guards the guard: an empty golden file would leave zero cases and go green.

    pytest reports no tests as no failures, so a generator or path bug that
    yielded nothing would hide behind a green run. This asserts the file is
    non-empty and names the branches it must cover.
    """
    vectors = _vectors()
    assert vectors, "the golden file yielded no vectors"
    branches = {v["expected_branch"] for v in vectors}
    # Both branches must be present, or the produced/fallback split is untested.
    assert branches == {BRANCH_PRODUCED, BRANCH_RX_FALLBACK}
    # And at least one negative vector, or CLK-C5's clamp is unexercised here.
    assert any(v["expected_negative"] for v in vectors)


def test_produced_age_uses_mono_not_ts():
    """*** Mutation one: the produced branch subtracts mono, never ts.

    Constructs an envelope whose ts (wall clock, ~1.75e9) and mono (~4821) are
    wildly different. The correct age from mono is ~0.5 s. The mutation swaps
    env.mono for env.ts in compute_age; using ts would give now_mono - ts, a
    huge NEGATIVE number that clamps to 0 with was_negative True. So this asserts
    a small positive non-negative age, which the mutant cannot produce.
    """
    raw = {"v": 1, "rid": "dog-01", "ts": 1753660800.25, "mono": 4821.25,
           "boot": "9f2c1a44", "seq": 1, "src": "perception",
           "ts_sync": True, "data": {}}
    env = decode(raw)
    result = compute_age(env, rx_mono=4821.5, now_mono=4821.75,
                         local_boot_id="9f2c1a44")
    # mono path: 4821.75 - 4821.25 = 0.5, positive, not negative.
    assert result.branch == BRANCH_PRODUCED
    assert result.was_negative is False
    assert _fmt(result.age_s) == "0.5"
    # The discriminating assertion: what the mutant WOULD produce, computed here,
    # must differ from the real result -- so the test cannot pass on the ts path.
    mutant_raw = 4821.75 - 1753660800.25   # now_mono - ts, the mutated arithmetic
    assert result.raw_age_s != mutant_raw


def test_boot_mismatch_falls_back_to_rx_not_mono():
    """*** Mutation three: a mono from another boot must NOT be used.

    Same envelope, but local_boot_id differs from env.boot. The correct behaviour
    is the receive-time fallback (now_mono - rx_mono = 0.25). The mutation drops
    the boot half of the condition and uses mono anyway (now_mono - mono = 0.5).
    This asserts the fallback branch AND the 0.25 value, so keeping mono on a
    mismatch turns it red.
    """
    raw = {"v": 1, "rid": "dog-01", "ts": 1.0, "mono": 4821.25,
           "boot": "9f2c1a44", "seq": 1, "src": "perception",
           "ts_sync": True, "data": {}}
    env = decode(raw)
    result = compute_age(env, rx_mono=4821.5, now_mono=4821.75,
                         local_boot_id="deadbeef")
    assert result.branch == BRANCH_RX_FALLBACK
    assert _fmt(result.age_s) == "0.25"
    # The mono-branch value the mutant would yield, to prove the two differ.
    assert _fmt(4821.75 - 4821.25) == "0.5"
    assert result.age_s != (4821.75 - 4821.25)


def test_negative_age_emits_event_and_clamps():
    """*** Mutation four: a negative age must BOTH clamp and emit the event.

    message_age_s is called with now_mono < mono on the produced branch. The
    return must be 0 (the clamp) AND the injected sink must have received exactly
    one negative_age event with the right sev / cat / detail. The mutation removes
    the emit call: the clamp still returns 0, so ONLY the missing event is
    observable, which is what this recorder asserts.
    """
    raw = {"v": 1, "rid": "dog-01", "ts": 1.0, "mono": 4821.75,
           "boot": "9f2c1a44", "seq": 1, "src": "perception",
           "ts_sync": True, "data": {}}
    env = decode(raw)
    # A list append is the whole sink: it records every event for inspection. This
    # is the injection CLAUDE.md 7.1 calls the most valuable test class.
    events = []
    age = message_age_s(env, rx_mono=4821.5, now_mono=4821.25,
                        local_boot_id="9f2c1a44", on_negative_age=events.append)
    # The clamp: the returned age is 0 regardless of the mutation.
    assert age == 0.0
    # The event: exactly one, and its fields match CLK-C5 and the S3.0.1
    # pseudocode. detail.age_s carries the RAW negative value (-0.5), not the
    # clamped 0 -- the operator needs to see how far back it went.
    assert len(events) == 1
    event = events[0]
    assert event.sev == NEGATIVE_AGE_SEV      # "warn"
    assert event.cat == NEGATIVE_AGE_CAT      # "system", validated against the closed set
    assert event.detail["kind"] == NEGATIVE_AGE_KIND
    assert event.detail["src"] == "perception"
    assert _fmt(event.detail["age_s"]) == "-0.5"


def test_a_non_negative_age_emits_no_event():
    """The balancing case: a normal age does not fire the sink.

    Without this, a sink that fired on EVERY call would pass the mutation-four
    test and be wrong. Here the age is positive, so the recorder must stay empty.
    """
    raw = {"v": 1, "rid": "dog-01", "ts": 1.0, "mono": 4821.25,
           "boot": "9f2c1a44", "seq": 1, "src": "perception",
           "ts_sync": True, "data": {}}
    env = decode(raw)
    events = []
    age = message_age_s(env, rx_mono=4821.5, now_mono=4821.75,
                        local_boot_id="9f2c1a44", on_negative_age=events.append)
    assert age == 0.5
    assert events == []


def test_on_negative_age_is_a_required_argument():
    """The sink has no default: forgetting it is a TypeError, not a silent drop.

    A None-defaulted sink would let a caller compute ages while silently dropping
    every CLK-C5 event -- the fail-silent this design refuses. Calling without the
    keyword must therefore fail loudly.
    """
    env = decode({"v": 1, "rid": "d", "ts": 1.0, "mono": 1.0, "boot": "9f2c1a44",
                  "seq": 1, "src": "s", "ts_sync": True, "data": {}})
    with pytest.raises(TypeError):
        # Intentionally omit on_negative_age.
        message_age_s(env, rx_mono=1.0, now_mono=1.0, local_boot_id="9f2c1a44")  # type: ignore[call-arg]


def test_read_local_boot_id_takes_first_eight_hex_lowercased(tmp_path):
    """read_local_boot_id: first 8 hex of the boot_id file, dashes gone, lower case.

    Points the reader at a fixture file rather than /proc so the value is fixed.
    A standard UUID's first group is exactly the first 8 hex; an upper-case id
    must come back lower so a wire comparison is case-stable.
    """
    f = tmp_path / "boot_id"
    # A realistic boot_id UUID, upper-cased to prove the lowercasing.
    f.write_text("9F2C1A44-1B2C-3D4E-5F60-708090A0B0C0\n", encoding="ascii")
    assert read_local_boot_id(str(f)) == "9f2c1a44"


def test_read_local_boot_id_rejects_a_short_file(tmp_path):
    """A corrupt (too short) boot_id fails loudly rather than padding.

    A short id would compare unequal to every real wire boot and quietly force
    the fallback branch everywhere -- a silent system-wide degrade. Failing here
    keeps it loud.
    """
    f = tmp_path / "boot_id"
    f.write_text("abc\n", encoding="ascii")
    with pytest.raises(ValueError):
        read_local_boot_id(str(f))
