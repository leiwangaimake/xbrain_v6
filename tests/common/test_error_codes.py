"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_error_codes.py
Brief: Metatest binding xbrain/common/errors/codes.yaml to 11 S13.4~S13.15, both directions

Description:
This is the whole point of CFG-CM-1. The shared library and the contract are two
documents maintained by different people at different times; nothing but this
test keeps them equal.

★★★ The assertion is a SYMMETRIC difference, not containment. One-directional
containment is the shape this project keeps catching: "every code in the library
appears in the contract" passes happily while the library is missing a code the
contract added -- which is exactly the E_STORAGE_CORRUPT case from 2026-08-04.

Both mutation cases below have been run and both go red (CLAUDE.md 3.3: an
assertion that has never been red has not been written).

★ Scan surface is asserted, not assumed: the parser reports which S13.x sections
it found, and a test fails if that set shrinks. A parser that silently stops
finding a group would make the closed set look smaller and the diff look empty.
"""

import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import errors  # noqa: E402
from xbrain.common.errors.exceptions import UnknownErrorCode  # noqa: E402

CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")

# The groups the contract defines. Named explicitly so a parser that stops
# finding one fails loudly instead of quietly returning a smaller set.
EXPECTED_GROUPS = frozenset("ABCDEFGHIJKL")


def _split_row(line):
    """Split a markdown row on unescaped pipes only.

    A cell containing \\| is content, not a delimiter. Splitting on bare "|"
    shifts every later column of that row -- which silently corrupted the
    retryable value of four codes when this parser was first written.
    """
    cells, cur, i = [], "", 0
    while i < len(line):
        if line[i] == "\\" and i + 1 < len(line) and line[i + 1] == "|":
            cur += "|"
            i += 2
            continue
        if line[i] == "|":
            cells.append(cur)
            cur = ""
            i += 1
            continue
        cur += line[i]
        i += 1
    cells.append(cur)
    return [c.strip() for c in cells[1:-1]] if len(cells) > 2 else []


def parse_contract():
    """{code: group} from the S13.4~S13.15 table bodies, plus the groups found.

    Reads only the first cell of each data row. The wider section prose mentions
    codes too (tombstones, examples, cross-references); mining those would let an
    unrelated sentence keep a deleted code alive in the set.
    """
    lines = open(CONTRACT, encoding="utf-8").read().split("\n")
    heads = sorted(
        (int(m.group(1)), m.group(2), i)
        for i, ln in enumerate(lines)
        if (m := re.match(r"^#{3,5}\s+13\.(\d+)\s+([A-L])\s*·", ln))
    )
    assert heads, "S13.4~S13.15 group headings not found -- parser or document changed"
    end = next(
        i for i, ln in enumerate(lines)
        if i > heads[-1][2] and re.match(r"^#{1,4}\s+13\.1[6-9]|^#{1,3}\s+1[4-9]\.", ln)
    )

    codes, groups = {}, set()
    for k, (_num, grp, start) in enumerate(heads):
        stop = heads[k + 1][2] if k + 1 < len(heads) else end
        groups.add(grp)
        for ln in lines[start:stop]:
            if not ln.startswith("|"):
                continue
            cells = _split_row(ln)
            if len(cells) < 5 or cells[0].strip() in ("码", "---"):
                continue
            m = re.search(r"`(OK|E_[A-Z0-9_]+)`", cells[0])
            if m:
                codes[m.group(1)] = grp
    return codes, groups


def test_scan_surface_intact():
    """All twelve groups are still reachable by the parser.

    Guards the guard: if a heading is renamed and the parser stops seeing group
    F, every F code vanishes from the contract side and the symmetric-difference
    test below would report them as library-only rather than as a parse failure.
    """
    _codes, groups = parse_contract()
    assert groups == EXPECTED_GROUPS, (
        f"groups found {sorted(groups)}, expected {sorted(EXPECTED_GROUPS)} -- "
        "a S13.x heading changed, or the parser no longer matches it"
    )


def test_closed_set_symmetric_difference_is_empty():
    """★★★ The core assertion of CFG-CM-1."""
    contract, _groups = parse_contract()
    lib = set(errors.ALL_CODES)
    only_contract = sorted(set(contract) - lib)
    only_lib = sorted(lib - set(contract))
    assert not only_contract, (
        f"contract has codes the library is missing: {only_contract} -- "
        "add them to xbrain/common/errors/codes.yaml"
    )
    assert not only_lib, (
        f"library exports codes the contract does not define: {only_lib} -- "
        "EC-2 forbids inventing codes; remove them or amend 11 S13"
    )


def test_group_assignment_matches():
    """Every code sits in the same group on both sides.

    Without this, a code could be moved between groups in the contract and the
    set-level test would stay green -- the set is unchanged, only the grouping
    moved, and the group is what determines the prefix convention.
    """
    contract, _ = parse_contract()
    wrong = [(c, g, errors.info(c).group) for c, g in contract.items()
             if errors.info(c).group != g]
    assert not wrong, f"group mismatch (code, contract, library): {wrong}"


def test_every_code_has_a_valid_retryability():
    """11 S13.2 -- and that section calls this column a safety item.

    A code with a wrong retryable makes a cloud client resend motion commands at
    10 Hz when E_LOCKED (HES not cleared) comes back.
    """
    valid = {errors.RETRY_YES, errors.RETRY_CONDITIONAL, errors.RETRY_NO, errors.RETRY_NA}
    bad = [c for c in errors.ALL_CODES if errors.retryable(c) not in valid]
    assert not bad, f"codes with an invalid retryable: {bad}"


def test_ok_is_not_a_failure_and_locked_is():
    """A positive case beside the negative ones.

    Without this, an is_failure that returns False for everything -- or a
    retryable that returns n_a for everything -- passes every other test here.
    That empty-shell implementation is exactly what CLAUDE.md 3.2 form 1 warns
    about, and this project has caught it live once already.
    """
    assert not errors.is_failure("OK")
    assert not errors.is_failure("E_DUPLICATE"), "idempotent hit is not a failure"
    assert errors.is_failure("E_LOCKED")
    assert errors.retryable("E_LOCKED") != errors.RETRY_YES, (
        "E_LOCKED marked retryable would make the cloud resend motion at 10 Hz"
    )


def test_out_of_set_value_raises():
    """11 S13.6: out-of-set values must raise -- no pass-through, no degrade."""
    with pytest.raises(UnknownErrorCode):
        errors.info("E_TOTALLY_MADE_UP")
    with pytest.raises(UnknownErrorCode):
        errors.retryable("")


def test_codes_are_importable_as_names():
    """CLAUDE.md 3.5: callers write E_TIMEOUT, not the string "E_TIMEOUT"."""
    assert errors.E_TIMEOUT == "E_TIMEOUT"
    assert errors.OK == "OK"
    assert errors.E_STORAGE_CORRUPT == "E_STORAGE_CORRUPT"


def test_no_count_is_written_into_the_yaml():
    """CLAUDE.md 3.7 -- and the contract itself proves why.

    11 S13.3 carries a per-group count and a total. Those numbers are already
    wrong: the table says group L holds 2 codes and the total is 40, while S13.15
    now holds 3. A number a human copies is a number that rots, so codes.yaml
    must not grow one.
    """
    text = open(os.path.join(ROOT, "xbrain", "common", "errors", "codes.yaml"), encoding="utf-8").read()
    offenders = [ln.strip() for ln in text.split("\n")
                 if re.search(r"(合计|共\s*\d+\s*(个|条|码)|total\s*[:=]\s*\d+)", ln)]
    assert not offenders, f"codes.yaml must not carry a tally: {offenders}"
