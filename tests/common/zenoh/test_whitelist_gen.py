"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_whitelist_gen.py
Brief: INF-ZN-7 -- extractor tallies match the doc; committed whitelists.py
       matches the extractor; each of the four criterion mutations behaves

Description:
Covers the whitelist generator (scripts/doccheck/whitelist_gen.py) end-to-end:
  1. extracted counts match the S1.1.6 hand-tables (5 cross-plane processes);
  2. committed xbrain/common/zenoh/whitelists.py matches what --check would
     compare it against RIGHT NOW -- the drift gate that keeps the two
     synchronised;
  3. each of the four criterion mutations produces the named behaviour, so
     none of the checks is a shell.
"""

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "scripts", "doccheck"))

from whitelist_gen import extract_all, check as gen_check      # noqa: E402
from xbrain.common.zenoh import whitelists as W                # noqa: E402

DOC = os.path.join(ROOT, "docs", "11-接口契约.md")


@pytest.fixture(scope="module")
def doc_text():
    return open(DOC, encoding="utf-8").read()


@pytest.fixture(scope="module")
def extracted(doc_text):
    return extract_all(doc_text)


# --------------------------------------------------------------------------
# The extractor's read of the committed doc
# --------------------------------------------------------------------------

def test_five_processes_present(extracted):
    """The five cross-plane processes S1.1.3 v0.6 fixed."""
    assert set(extracted) == {"perception", "p1_motion", "chassis_relay",
                              "p2_core", "p4_agent"}


def test_perception_has_two_pub_zero_sub(extracted):
    """Table ① is small enough to name verbatim."""
    assert set(extracted["perception"]["pub"]) == {
        "state/targets", "event/{severity}/perception"}
    assert extracted["perception"]["sub"] == []


def test_p1_motion_direction_dash_row_not_counted(extracted):
    """P1-2 (state/robot direction '--') must NOT enter either whitelist."""
    assert "state/robot" not in extracted["p1_motion"]["pub"]
    assert "state/robot" not in extracted["p1_motion"]["sub"]


def test_chassis_relay_direction_arrow_mapping(extracted):
    """GEN -> RT rows land in SUB, RT -> GEN rows land in PUB."""
    # CR-1 (cmd/estop) is GEN -> RT.
    assert "cmd/estop" in extracted["chassis_relay"]["sub"]
    # CR-4 (state/robot proxied to state/robot on general plane) is RT -> GEN.
    # The general-plane key is state/robot? No -- chassis_relay's general side
    # for CR-4 is state/robot. Confirmed by _clean_direction mapping.
    assert "rt/chassis/state" in extracted["chassis_relay"]["pub"]


def test_no_wildcard_in_any_whitelist(extracted):
    """WL-G3 refuses wildcards; a whitelist that ever contains one is a
    silent generic-forwarding vector."""
    for proc, data in extracted.items():
        for kind in ("pub", "sub"):
            for key in data[kind]:
                assert "*" not in key, (proc, kind, key)


# --------------------------------------------------------------------------
# The drift gate: committed constants match extractor output
# --------------------------------------------------------------------------

def test_committed_whitelists_match_extractor(extracted):
    """*** The gate S1.1.6 asks for: xbrain/common/zenoh/whitelists.py must
    equal what the extractor derives right now. A doc edit that grows a
    whitelist without regenerating this file fails here."""
    committed = {p: {"pub": set(W.WHITELISTS[p]["pub"]),
                     "sub": set(W.WHITELISTS[p]["sub"])}
                 for p in W.WHITELISTS}
    deltas = gen_check(extracted, committed)
    assert deltas == [], deltas


# --------------------------------------------------------------------------
# The four criterion mutations -- run through the same public API
# --------------------------------------------------------------------------

def test_mutation_1_drop_p1_21(doc_text, extracted):
    """① delete the P1-21 row -> p1_motion total falls by exactly 1."""
    import re
    baseline = len(extracted["p1_motion"]["pub"]) + len(extracted["p1_motion"]["sub"])
    # Same permissive prefix as the self-test uses: the row is bold-wrapped
    # with 中文星号, so ASCII-only star matching misses it.
    mut = re.sub(r"^\| [^|\n]*P1-21[^\n]*\n", "", doc_text, count=1,
                 flags=re.M)
    r = extract_all(mut)
    got = len(r["p1_motion"]["pub"]) + len(r["p1_motion"]["sub"])
    assert got == baseline - 1


def test_mutation_2_flip_p1_2_to_sub(doc_text, extracted):
    """② flip P1-2 '--' -> 'sub' -> p1_motion total rises by exactly 1."""
    import re
    baseline = len(extracted["p1_motion"]["pub"]) + len(extracted["p1_motion"]["sub"])
    # Doc uses em-dash (U+2014); kept explicit so the ASCII cleanup pass
    # over prose does not accidentally break this string match.
    mut = re.sub(r"(\| P1-2 \| `state/robot` \| )—", r"\1sub", doc_text)
    r = extract_all(mut)
    got = len(r["p1_motion"]["pub"]) + len(r["p1_motion"]["sub"])
    assert got == baseline + 1
    assert "state/robot" in r["p1_motion"]["sub"]


def test_mutation_3_wildcard_sub_reported_as_wl_g3(doc_text):
    """③ inject an event/** row that names p4_agent as subscriber -> WL-G3
    error for p4_agent, and the wildcard MUST NOT enter the sub set."""
    import re
    # Bold-wrapped key in the committed doc; permit the ** markers.
    row = re.search(r"^\| \*?\*?`xbrain/\{rid\}/event/\{severity\}/"
                    r"\{category\}`\*?\*?[^\n]*\n", doc_text, re.M)
    assert row is not None
    injection = ("| `xbrain/{rid}/event/**` | X | `p4_agent` | e | Q3 | z |\n")
    mut = doc_text[:row.end()] + injection + doc_text[row.end():]
    r = extract_all(mut)
    wl_g3 = r["p4_agent"]["wl_g3_errors"]
    assert any(key == "event/**" for _p, key, _k in wl_g3)
    assert "event/**" not in r["p4_agent"]["sub"]


def test_mutation_4_p1_22_flipped_triggers_direction_policy(doc_text):
    """④ flip P1-22 direction from pub to sub -> DIRECTION_POLICY refusal
    (S7A.8: state/arb/motion must be pub-only)."""
    import re
    # P1-22 row has 三星 wrapping around 'pub'. Match either wrapped or bare.
    m = re.search(r"(P1-22[^\n]*?\| ★+ ?)\*?\*?\*?pub\*?\*?\*?", doc_text)
    if m is None:
        m = re.search(r"(P1-22[^\n]*?\| )pub", doc_text)
    assert m is not None
    mut = doc_text[:m.start()] + m.group(1) + "sub" + doc_text[m.end():]
    r = extract_all(mut)
    policy = r["p1_motion"]["policy_errors"]
    assert policy, "P1-22 flipped to sub but DIRECTION_POLICY did not fire"
