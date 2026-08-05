"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_closed_sets.py
Brief: Metatest binding xbrain/common/enums/sets.yaml to 11, one case per set

Description:
CFG-CM-4. Same shape as the error-code metatest: the library and the contract are
maintained separately and only this test keeps them equal.

★★★ Symmetric difference, per set. One-directional containment is the shape this
project keeps catching -- "every value the library exports is in the contract"
stays green while the library is missing a value the contract added, which is
exactly how Event.category sat at 21 after the contract moved to 23.

★ Two sets are ordered, not just membership: gate_limiter's order is the
attribution priority and stop_reason's is the decision order of S9.12.2. They get
a sequence assertion too -- a set-only check would stay green while the reported
cause silently changed.

★ Every extractor below names the exact table header it keys off, and the parser
raises rather than skipping rows it does not understand. Both matter: an
extractor that silently finds fewer rows makes the diff look empty. That is not
hypothetical -- the first version of this parser treated an empty leading cell
as a separator row and dropped nine of task_state's twelve values.
"""

import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import enums  # noqa: E402
from xbrain.common.errors.exceptions import ClosedSetViolation  # noqa: E402

CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")
_LINES = open(CONTRACT, encoding="utf-8").read().split("\n")


def _cells(line):
    """Split on unescaped pipes only -- a cell containing \\| is content."""
    out, cur, i = [], "", 0
    while i < len(line):
        if line[i] == "\\" and i + 1 < len(line) and line[i + 1] == "|":
            cur += "|"
            i += 2
            continue
        if line[i] == "|":
            out.append(cur)
            cur = ""
            i += 1
            continue
        cur += line[i]
        i += 1
    out.append(cur)
    return [c.strip() for c in out[1:-1]] if len(out) > 2 else []


def _is_separator(c):
    """A separator row is |---|---|. An EMPTY leading cell is a continuation row.

    Treating empty as a separator drops every row whose first column is a
    rowspan-style blank, which in 11 S4.4 is nine of the twelve task states.
    """
    return bool(c) and c[0] != "" and set(c[0]) <= set("-: ")


def _table_column(header_pred, colno, what):
    """Values in column `colno` of the contiguous table whose header matches."""
    for i, line in enumerate(_LINES):
        if not line.startswith("|") or not header_pred(_cells(line)):
            continue
        vals = []
        for row in _LINES[i + 1:]:
            if not row.startswith("|"):
                break
            c = _cells(row)
            if not c or _is_separator(c) or len(c) <= colno:
                continue
            m = re.search(r"`([a-z_]+)`", c[colno])
            if m:
                vals.append(m.group(1))
        assert vals, f"{what}: header matched but no values parsed"
        return list(dict.fromkeys(vals))
    raise AssertionError(f"{what}: table header not found in the contract -- "
                         "the heading or column names changed")


def _inline_enum(anchor, colno, what):
    """Values from a pipe-separated enumeration inside one table cell.

    Takes the first sentence only and strips parenthesised prose: 11 S4.1's
    stop_reason cell explains mode_mismatch inline, and that explanation contains
    two further backticked identifiers which are not members of the set.
    """
    for line in _LINES:
        if not line.startswith("|") or anchor not in line:
            continue
        c = _cells(line)
        if len(c) <= colno:
            continue
        text = re.sub(r"（[^）]*）", "", c[colno].split("。")[0])
        vals = list(dict.fromkeys(re.findall(r"`([a-z_]+)`", text)))
        assert vals, f"{what}: anchor matched but no values parsed"
        return vals
    raise AssertionError(f"{what}: anchor {anchor!r} not found in the contract")


# One extractor per set. Each names the exact header/anchor it keys off, so a
# document change breaks the test loudly instead of shrinking the set quietly.
EXTRACTORS = {
    "plane": lambda: _inline_enum("| `plane` |", 1, "plane"),
    "domain": lambda: _table_column(
        lambda c: len(c) > 1 and c[0] == "域" and "`domain`" in c[1], 1, "domain"),
    "event_category": lambda: _table_column(
        lambda c: len(c) > 2 and c[0] == "category" and "channel" in c[2], 0, "event_category"),
    "gate_limiter": lambda: _table_column(
        lambda c: len(c) > 1 and c[0] == "优先级" and "`limiter`" in c[1], 1, "gate_limiter"),
    "stop_reason": lambda: _inline_enum("`stop_reason` |", 1, "stop_reason"),
    "task_state": lambda: _table_column(
        lambda c: len(c) > 1 and c[0] == "分组" and "`state`" in c[1], 1, "task_state"),
}


def test_every_exported_set_has_an_extractor():
    """Guards the guard.

    A set added to sets.yaml without an extractor here would never be compared
    against the contract, and the suite would still be green -- an assertion that
    covers less than it appears to.
    """
    assert set(EXTRACTORS) == set(enums.SET_NAMES), (
        f"extractors {sorted(EXTRACTORS)} vs exported sets {sorted(enums.SET_NAMES)}"
    )


@pytest.mark.parametrize("name", sorted(EXTRACTORS))
def test_symmetric_difference_is_empty(name):
    """★★★ The core assertion of CFG-CM-4, one case per set."""
    contract = set(EXTRACTORS[name]())
    lib = set(enums.get(name))
    only_contract = sorted(contract - lib)
    only_lib = sorted(lib - contract)
    assert not only_contract, (
        f"{name}: contract has values the library is missing: {only_contract} -- "
        "regenerate xbrain/common/enums/sets.yaml"
    )
    assert not only_lib, (
        f"{name}: library exports values the contract does not define: {only_lib} -- "
        "closed sets are not extended in code; amend 11 first"
    )


@pytest.mark.parametrize("name", ["gate_limiter", "stop_reason"])
def test_order_matches_the_contract(name):
    """These two are decision orders, not just memberships.

    gate_limiter's sequence is the attribution priority (estop must win); the
    contract states stop_reason's is 'the decision order of S9.12.2'. A
    membership-only check stays green while the reported cause changes.
    """
    assert enums.get(name).ordered, f"{name} must be declared ordered in sets.yaml"
    assert list(enums.get(name)) == EXTRACTORS[name](), (
        f"{name}: order differs from the contract"
    )


def test_estop_outranks_everything_in_the_limiter_order():
    """A positive case beside the negative ones.

    Without it, a sets.yaml whose gate_limiter happened to be empty, or an index()
    that always returned 0, would pass every other test here. That empty-shell
    pass is CLAUDE.md 3.2 form 1, and this project has caught it live.
    """
    assert enums.GATE_LIMITER.index("estop") == 0
    assert enums.GATE_LIMITER.index("estop") < enums.GATE_LIMITER.index("free_space")
    assert enums.STOP_REASON.index("none") < enums.STOP_REASON.index("no_source")


def test_out_of_set_value_raises():
    """11 S13.6: raise, never pass through and never degrade to a nearby value."""
    with pytest.raises(ClosedSetViolation):
        enums.parse_enum("plane", "telemetry")
    with pytest.raises(ClosedSetViolation):
        enums.EVENT_CATEGORY.parse("ptz_boost")
    # An unknown SET name must also raise -- not return an empty set that then
    # accepts everything.
    with pytest.raises(ClosedSetViolation):
        enums.get("no_such_set")


def test_legal_values_pass_through_unchanged():
    """Pairs with the test above: an implementation that raises on everything
    would satisfy the negative case alone."""
    assert enums.parse_enum("plane", "rt") == "rt"
    assert enums.EVENT_CATEGORY.parse("ptz") == "ptz"
    assert enums.TASK_STATE.parse("suspended") == "suspended"


def test_index_refuses_unordered_sets():
    """Asking a plane for its priority is a misunderstanding; answering hides it."""
    with pytest.raises(ValueError):
        enums.PLANE.index("rt")


def test_no_count_is_written_into_the_yaml():
    """CLAUDE.md 3.7. The contract's own tallies have already rotted once."""
    text = open(os.path.join(ROOT, "xbrain", "common", "enums", "sets.yaml"),
                encoding="utf-8").read()
    offenders = [ln.strip() for ln in text.split("\n")
                 if re.search(r"(合计|共\s*\d+\s*(个|值|条)|total\s*[:=]\s*\d+)", ln)]
    assert not offenders, f"sets.yaml must not carry a tally: {offenders}"
