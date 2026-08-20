"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_closed_sets.py
Brief: Metatest binding xbrain/common/enums/sets.yaml to 11, one case per set

Description:
CFG-CM-4. Same shape as the error-code metatest: the library and the contract are
maintained separately and only this test keeps them equal.

*** Symmetric difference, per set. One-directional containment is the shape this
project keeps catching -- "every value the library exports is in the contract"
stays green while the library is missing a value the contract added, which is
exactly how Event.category sat at 21 after the contract moved to 23.

* Two sets are ordered, not just membership: gate_limiter's order is the
attribution priority and stop_reason's is the decision order of S9.12.2. They get
a sequence assertion too -- a set-only check would stay green while the reported
cause silently changed.

* Every extractor below names the exact table header it keys off, and the parser
raises rather than skipping rows it does not understand. Both matter: an
extractor that silently finds fewer rows makes the diff look empty. That is not
hypothetical -- the first version of this parser treated an empty leading cell
as a separator row and dropped nine of task_state's twelve values.

* Two sets are not whole tables and each brings its own way to be wrong.
  cls is prose: S3.1.1 states it as one line of names under a heading, so it is
  read by heading and the note under it is skipped, because prose about a set is
  not the set. device is a SUBSET of a table: the S5.1A rows whose kind column
  reads device, cap rows excluded. A subset extractor that loses its filter still
  produces a plausible-looking list, so it gets a named case of its own rather
  than relying on the difference to be read correctly.

*** The left operand is not always 11. charge_stage is defined in 15 S8.5: 11
states that its own charging section 定接口 while 15 S8.5 定判据与阈值, and that
两处不得各写一份阈值 (CFG-40). 11's charging table carries a charge_stage
synchronisation column, and reading THAT would look entirely reasonable and be
wrong -- it lists only the stages an event announces and has no row for
to_handover, so the library would end up rejecting the stage the robot is in
while it drives to the handover point. Each extractor below therefore names the
volume it reads, and test_every_set_is_read_from_the_volume_it_declares pins that
choice against the source: field the library exports.

*** Three of these tables appear TWICE in 11, because 11 carries a section of
merge-ready patch blocks that restate whole tables from the volumes they amend.
Taking the first match is right -- the body text comes first -- but "the first
one" quietly stops meaning "the current one" if the two ever disagree, which is
the failure mode this project has already recorded against 11 as a patch queue:
a whole-table replacement rolls back silently because nobody compares the copies.
So the tables are extracted from EVERY matching header and asserted equal, and a
companion case pins that the duplicates really exist, so the comparison cannot
degrade into a comparison of one thing with itself.
"""

import os
import re
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import enums  # noqa: E402
from xbrain.common.errors.exceptions import ClosedSetViolation  # noqa: E402

CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")
_LINES = open(CONTRACT, encoding="utf-8").read().split("\n")

#: Every volume an extractor below reads, keyed by the volume number the library
#: writes in its source: field. Keyed rather than held in separate globals so
#: test_every_set_is_read_from_the_volume_it_declares can compare the two
#: mechanically: an extractor pointed at the wrong volume produces a set that
#: still parses and is still plausible, and nothing else in this file would
#: notice.
_DOCS = {
    "11": _LINES,
    "15": open(os.path.join(ROOT, "docs", "15-P3任务与充电详细设计.md"),
               encoding="utf-8").read().split("\n"),
}

#: The ordinary shape of a value in a contract table: a backticked lower-case
#: identifier. Named rather than repeated at each call site so the two extractors
#: that need a different shape stand out as deliberate.
VALUE_RE = r"`([a-z_]+)`"

#: 11 S7A.6.5 writes its values as backticked JSON strings -- `"soft_estop"` --
#: because the column shows what the field literally holds. VALUE_RE finds
#: nothing in that cell, and "finds nothing" is indistinguishable from "the table
#: moved" unless the difference is deliberate, which is why this is a separate
#: named pattern rather than a quote-tolerant widening of VALUE_RE. Widening it
#: would also let a quoted example anywhere else in a cell arrive as a member.
QUOTED_VALUE_RE = r"`\"([a-z_]+)\"`"


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


def _all_tables_column(lines, header_pred, colno, row_pred, what, value_re,
                       take_all):
    """One value list per table in `lines` whose header matches, in document order.

    Every caller goes through here, including the single-table ones, so there is
    exactly one row loop to keep correct. The row loop is where this file has
    already been wrong once (see the module docstring on task_state).

    row_pred exists for the sets that are a SUBSET of a table rather than the
    whole of it -- device is the rows of 11 S5.1A whose kind column reads device,
    with the cap rows deliberately left out. It is a separate argument instead of
    a filter applied to the result because the decision needs the whole row: the
    item name alone cannot tell you that heading is a cap and rtk is a device.

    take_all decides whether a cell may contribute more than one value, and it is
    passed explicitly at every call site rather than defaulted, because both
    answers are silently wrong for the other kind of table. 11 S4.4's
    suspend_reason table puts TWO reasons in one cell where they share a
    suspend_kind, so taking the first would drop energy_unreachable and
    estop_hes -- values real code paths raise -- and the resulting set would look
    perfectly ordinary. Taking all from a table like S5.1A, whose cells carry
    cross-references in backticks, would sweep those references in as members.
    """
    tables = []
    for i, line in enumerate(lines):
        if not line.startswith("|") or not header_pred(_cells(line)):
            continue
        vals = []
        for row in lines[i + 1:]:
            if not row.startswith("|"):
                break
            c = _cells(row)
            if not c or _is_separator(c) or len(c) <= colno:
                continue
            # row_pred is consulted before the value is extracted, not after.
            # After would mean deciding on the extracted name, and the name is
            # the one part of the row that cannot answer the question: rtk and
            # heading are both plausible item names and only the kind column
            # separates them.
            if not row_pred(c):
                continue
            # The cell also carries severity markers and bold wrapping around the
            # name in several of these tables, so the name cannot be taken as the
            # whole cell.
            names = re.findall(value_re, c[colno])
            vals.extend(names if take_all else names[:1])
        assert vals, f"{what}: header matched but no values parsed"
        tables.append(list(dict.fromkeys(vals)))
    if not tables:
        raise AssertionError(f"{what}: table header not found in the contract -- "
                             "the heading or column names changed")
    return tables


def _table_column_where(lines, header_pred, colno, row_pred, what, value_re,
                        take_all):
    """The FIRST matching table's column. Body text precedes the patch blocks.

    Where a header occurs more than once, taking the first is what selects the
    body text over 11's merge-ready patch blocks. That the copies agree is a
    separate claim and is asserted separately, because "first" silently stops
    meaning "current" the moment they diverge.
    """
    return _all_tables_column(lines, header_pred, colno, row_pred, what,
                              value_re, take_all)[0]


def _table_column(lines, header_pred, colno, what, value_re, take_all):
    """The whole-table case, expressed as the no-op row filter."""
    return _table_column_where(lines, header_pred, colno, lambda c: True, what,
                               value_re, take_all)


def _inline_enum(lines, anchor, colno, what):
    """Values from a pipe-separated enumeration inside one table cell.

    Takes the first sentence only and strips parenthesised prose: 11 S4.1's
    stop_reason cell explains mode_mismatch inline, and that explanation contains
    two further backticked identifiers which are not members of the set.
    """
    for line in lines:
        if not line.startswith("|") or anchor not in line:
            continue
        c = _cells(line)
        if len(c) <= colno:
            continue
        text = re.sub(r"（[^）]*）", "", c[colno].split("。")[0])
        vals = list(dict.fromkeys(re.findall(VALUE_RE, text)))
        assert vals, f"{what}: anchor matched but no values parsed"
        return vals
    raise AssertionError(f"{what}: anchor {anchor!r} not found in the contract")


def _section_enum(lines, heading, what):
    """Values from the prose line under a section heading, for sets not in a table.

    11 S3.1.1 states the cls set as one line of backticked names under its own
    heading -- no table, no cell, so neither extractor above reaches it. Keying
    off the heading rather than off the value line itself is what makes a
    deletion loud: if the values line is removed, the next thing seen is the next
    heading and this raises, whereas keying off the values line would just report
    "anchor not found" whether the section moved, shrank or vanished.

    Blockquote lines are skipped rather than parsed. The note under this
    particular heading says new classes need review and explains unknown, and
    prose about a set is not the set. Headings terminate the search for the same
    reason: reading on past the section boundary would let a neighbouring
    section's backticked identifiers arrive as members.
    """
    for i, line in enumerate(lines):
        # Equality against the whole stripped line, not a substring test. The
        # section number also appears inside a prose table row elsewhere in the
        # contract, and matching that row would start reading the enumeration out
        # of an unrelated table.
        if line.strip() != heading:
            continue
        for row in lines[i + 1:]:
            s = row.strip()
            # Blank lines are layout. Blockquotes are the editorial note that
            # follows this heading; it names two members while talking ABOUT
            # them, so parsing it would work today and quietly go wrong the day
            # the note mentions a class that was removed.
            if not s or s.startswith(">"):
                continue
            # The next heading ends the section. Reaching it means the value line
            # is gone, which must raise rather than fall through to whatever the
            # following section happens to contain.
            if s.startswith("#"):
                break
            # dict.fromkeys and not set(): document order is preserved so the
            # ordered-set comparison in this file can use the same extractors
            # unchanged if this set is ever given an order.
            vals = list(dict.fromkeys(re.findall(VALUE_RE, s)))
            assert vals, f"{what}: the line under {heading!r} parsed to nothing"
            return vals
        # Only the first matching heading is considered. Falling through to look
        # for a second one would hide a duplicated section number instead of
        # reporting it.
        break
    raise AssertionError(f"{what}: no value line under heading {heading!r} -- "
                         "the section was renamed, moved or emptied")


def _row_backticked(lines, anchor, what, exclude=()):
    """Backticked values from the WHOLE row containing `anchor`, minus `exclude`.

    reason and control_mode (CFG-CM-16) live in the 11 S R2.6-e / R2.6-f patch
    rows, not in a S13 table (that table is not written yet -- the CFG-CM-16
    criterion calls the doc-diff a 空位/placeholder for exactly this reason). Two
    shapes _inline_enum cannot handle:
      * reason's four values sit INSIDE parentheses, and _inline_enum strips
        parenthesised prose (it is built to drop an explanatory aside);
      * control_mode's row backticks the SET NAME (`control_mode`) as well as the
        value (`jog`), so the name must be excluded.
    So this reads every backticked lower_snake token on the row and drops the
    excluded ones. `common/errors/` and friends carry slashes and VALUE_RE (which
    requires the whole backtick content to be [a-z_]+) does not match them.
    """
    for line in lines:
        if not line.startswith("|") or anchor not in line:
            continue
        vals = [v for v in re.findall(VALUE_RE, line) if v not in exclude]
        vals = list(dict.fromkeys(vals))
        assert vals, "%s: anchor matched but no values parsed" % what
        return vals
    raise AssertionError("%s: anchor %r not found in the contract" % (what, anchor))


#: Which volume each set is read from, kept as data so it can be compared with
#: the source: field the library exports. An extractor aimed at the wrong volume
#: still produces a plausible set -- 11 has a charge_stage column of its own --
#: and nothing else here would notice.
SET_DOC = {
    "plane": "11", "domain": "11", "event_category": "11", "gate_limiter": "11",
    "stop_reason": "11", "task_state": "11", "cls": "11", "device": "11",
    "release_reason": "11", "arb_suspended": "11", "gate_reason": "11",
    "suspend_kind": "11", "suspend_reason": "11", "severity": "11",
    "reason": "11", "control_mode": "11", "heading_source": "11",
    "fence_role": "11",
    # The one set defined outside the contract. See the module docstring.
    "charge_stage": "15",
}

#: The table-shaped sets, declared rather than written out as closures, so the
#: symmetric-difference cases and the duplicate-table case drive the same
#: predicates. Two extractors reading a table through slightly different
#: predicates is how one of them silently stops matching.
#:
#: Each entry is (header_pred, colno, row_pred, value_re, take_all).
TABLE_SETS = {
    "domain": (
        lambda c: len(c) > 1 and c[0] == "域" and "`domain`" in c[1],
        1, lambda c: True, VALUE_RE, False),
    "event_category": (
        lambda c: len(c) > 2 and c[0] == "category" and "channel" in c[2],
        0, lambda c: True, VALUE_RE, False),
    "gate_limiter": (
        lambda c: len(c) > 1 and c[0] == "优先级" and "`limiter`" in c[1],
        1, lambda c: True, VALUE_RE, False),
    "task_state": (
        lambda c: len(c) > 1 and c[0] == "分组" and "`state`" in c[1],
        1, lambda c: True, VALUE_RE, False),
    # device reads column 0 of the S5.1A spec table but keeps only the rows whose
    # kind column is device. The kind cell is compared with == and not with `in`:
    # the neighbouring value is cap, so a substring test would behave for now and
    # start swallowing rows the moment a kind is named that contains "device".
    "device": (
        lambda c: len(c) > 1 and c[0] == "项" and "`kind`" in c[1],
        0, lambda c: c[1] == "device", VALUE_RE, False),
    # 11 S7A.6.5, the table that separates the three kinds of "stop". Keyed off
    # the suspended column being present in the header rather than off the
    # section number, because the section numbering of 11 moves and the column
    # names are what the extractor actually depends on.
    #
    # QUOTED_VALUE_RE: this column shows the literal field contents, so its cells
    # read `"soft_estop"` and not `soft_estop`.
    "arb_suspended": (
        lambda c: len(c) > 3 and c[0] == "类型" and "`suspended`" in c[3],
        3, lambda c: True, QUOTED_VALUE_RE, False),
    # 11 S8.9.2's legend for AsrGate.reason. Whole-cell equality on both header
    # cells: "含义" alone appears in dozens of headers in 11, and a looser
    # predicate would match the first of them instead.
    "gate_reason": (
        lambda c: len(c) > 1 and c[0] == "`reason` 取值" and c[1] == "含义",
        0, lambda c: True, VALUE_RE, False),
    # 11 S4.4's suspend_kind legend. len(c) > 3 is load-bearing rather than
    # defensive: 15 quotes a three-column version of this same table, 11 carries
    # that quotation in its patch section, and matching it would read the set out
    # of a restatement instead of out of the definition.
    "suspend_kind": (
        lambda c: len(c) > 3 and c[0] == "`suspend_kind`" and c[1] == "触发",
        0, lambda c: True, VALUE_RE, False),
    # *** take_all is True here and nowhere else. Two rows of this table carry a
    # PAIR of reasons that share one suspend_kind, so first-per-row would drop
    # energy_unreachable and estop_hes. Both are raised by real paths -- an
    # unreachable charger and a hardware e-stop -- and the shrunken set would
    # look entirely ordinary while making the task machine reject its own
    # suspension reason.
    "suspend_reason": (
        lambda c: len(c) > 1 and c[0] == "`suspend_reason` 闭集"
        and c[1] == "`suspend_kind`",
        0, lambda c: True, VALUE_RE, True),
    # 15 S8.5. The header is three generic Chinese words, which would be far too
    # weak a key in 11; in 15 it occurs once, and the case below pins that.
    "charge_stage": (
        lambda c: c == ["取值", "含义", "退出条件"],
        0, lambda c: True, VALUE_RE, False),
}


def _table_extractor(name):
    """Bind one TABLE_SETS entry to its volume. Kept out of the dict literal so
    the loop variable is captured by value rather than by reference -- a late
    binding here would point every extractor at the last set defined."""
    header_pred, colno, row_pred, value_re, take_all = TABLE_SETS[name]
    return lambda: _table_column_where(_DOCS[SET_DOC[name]], header_pred, colno,
                                       row_pred, name, value_re, take_all)


# One extractor per set. Each names the exact header/anchor it keys off, so a
# document change breaks the test loudly instead of shrinking the set quietly.
EXTRACTORS = {name: _table_extractor(name) for name in TABLE_SETS}

# The four sets that are not whole tables. Written out because each is a
# different shape and there is nothing to share between them.
EXTRACTORS.update({
    "plane": lambda: _inline_enum(_DOCS["11"], "| `plane` |", 1, "plane"),
    "stop_reason": lambda: _inline_enum(_DOCS["11"], "`stop_reason` |", 1,
                                        "stop_reason"),
    # cls is the one set the contract states as prose rather than as a table.
    "cls": lambda: _section_enum(_DOCS["11"], "#### 3.1.1 目标类别取值", "cls"),
    # release_reason is an enumeration inside the 说明 cell of the S7A.1 field
    # table. The anchor carries the type column as well as the field name: the
    # op legend a few rows above mentions release_reason in prose, and an anchor
    # of just the field name would match that row first.
    "release_reason": lambda: _inline_enum(
        _DOCS["11"], "| `release_reason` | string |", 4, "release_reason"),
    # reason / control_mode are the CFG-CM-16 sets, read from the 11 S R2.6-e /
    # R2.6-f patch rows (the S13 table is not written yet -- see _row_backticked).
    # The anchors are the phrase unique to each row; control_mode excludes its own
    # backticked set name so only the value `jog` survives.
    "reason": lambda: _row_backticked(
        _DOCS["11"], "task/progress 的 reason 闭集", "reason"),
    "control_mode": lambda: _row_backticked(
        _DOCS["11"], "闭集章补 `control_mode` 一项",
        "control_mode", exclude=("control_mode",)),
    # heading_source (GnssHeading.source, 11 S3.3) is stated inline in one field
    # cell: `dual_antenna` \| `cog` \| `none`. The anchor is the field-name cell,
    # unique to the one "| `heading_source` | string |" row (the S3.3 field table
    # writes `source` as the field name and lists motion_derived as retired; that
    # OTHER row would drag the retired value in, so we anchor the clean row and
    # exclude the set's own backticked name).
    "heading_source": lambda: _row_backticked(
        _DOCS["11"], "| `heading_source` | string |",
        "heading_source", exclude=("heading_source",)),
    # severity is the {severity} key segment of event/{severity}/{category}
    # (S2.2.11 W-5), stated inline in one cell as info / warn / alarm / fault.
    # The anchor carries the value column too (`{severity}` | `info`):
    # the field name `{severity}` alone appears in several prose rows (S1.1.6,
    # V-2), and an anchor of just that would match one of them first.
    "severity": lambda: _inline_enum(
        _DOCS["11"], "`{severity}` | `info`", 2, "severity"),
    # fence_role -- the S9A.2 FenceSet field-table row. VALUE_RE only matches
    # backticked [a-z_]+ so `polygons[].role` (has [].) is skipped; the row's own
    # 旧名 zone note backticks `zone`, which is NOT a role value, so exclude it.
    "fence_role": lambda: _row_backticked(
        _DOCS["11"], "`polygons[].role` | string", "fence_role",
        exclude=("zone",)),
})


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
    """*** The core assertion of CFG-CM-4, one case per set."""
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


def test_device_excludes_the_capability_items():
    """A tripwire on the WRONG FIX, not on the extractor.

    Losing the kind filter is the natural way to write the device extractor
    wrong, and the symmetric difference does go red for it -- but with the
    message "contract has values the library is missing", which reads as
    "sets.yaml is out of date" and invites exactly one repair: paste the cap rows
    into sets.yaml. That repair turns the suite green again with a device set
    that is not a device set. This case is what makes it impossible: it fails on
    the library side and names the rows that do not belong.

    heading and clock lead the list on purpose. Both are fatal, so an
    implementation that filtered on the level column instead of the kind column
    would still be caught here.
    """
    # Two positive members first, so an empty device set cannot satisfy the
    # exclusions below by holding nothing at all -- CLAUDE.md 3.2 form 1.
    # cam_rgbd and payload_light are chosen from opposite ends of the table:
    # a truncated parse that stopped early would drop the second one.
    assert "cam_rgbd" in enums.DEVICE
    assert "payload_light" in enums.DEVICE
    # The exclusions. Each of these is a real row of the same table with kind
    # cap, so they are exactly the values a filter-less extractor produces.
    for cap_item in ("heading", "clock", "compute", "gpu", "network"):
        assert cap_item not in enums.DEVICE, (
            f"{cap_item} is a kind=cap row of 11 S5.1A; the device set is not "
            "the whole self-test table, it is the kind=device rows of it"
        )
    # Membership above says the value is absent; this says the boundary rejects
    # it. They are not the same claim: a parse() with a fallback would satisfy
    # the first and quietly accept a cap item at a decode site.
    with pytest.raises(ClosedSetViolation):
        enums.DEVICE.parse("heading")


def test_cls_rejects_a_plausible_neighbour_of_a_member():
    """Pairs with the symmetric difference, and aims at the tempting failure.

    11 S13.6 forbids degrading an unrecognised value to something close, and cls
    is where that temptation is strongest: a detector emitting "human" or "car"
    instead of person or vehicle is obviously "the same thing", and a decoder
    that quietly accepts it turns a perception contract break into a silently
    mislabelled target on the HMI. The values below must raise, not map.
    """
    # unknown is asserted as a legal member and not as the fallback it looks
    # like. It is a value perception is entitled to emit -- detected but not
    # classifiable -- so a decoder must never manufacture it from a class it did
    # not recognise: doing so makes a contract break read as an honest
    # low-confidence detection, which is the one reading nobody investigates.
    assert enums.CLS.parse("person") == "person"
    assert enums.CLS.parse("unknown") == "unknown"
    # Three kinds of near miss on purpose: a synonym, a different word for the
    # same object, and two case variants. Case is listed because parse() must
    # stay a validator; the day it learns to case-fold, two spellings of one
    # class survive across processes until they have to match each other.
    for near_miss in ("human", "car", "Person", "PERSON", "pedestrian"):
        with pytest.raises(ClosedSetViolation):
            enums.CLS.parse(near_miss)


def test_the_two_added_sets_carry_no_order():
    """Neither contract site states an order, so neither may hand one out.

    Marking either ordered in sets.yaml costs nothing at import and gives index()
    a straight face while it reports a rank that came from a table's typography.
    For device that rank would look like a self-test priority, which the contract
    expresses through the level column instead.
    """
    for name in ("cls", "device"):
        # The declaration in sets.yaml.
        assert not enums.get(name).ordered
        # And the behaviour that must follow from it. Checking only the flag
        # would leave an index() that answered anyway, which is the half that
        # actually reaches a caller. The member asked for is taken from the set
        # itself so this stays correct when the contract adds a class or a box.
        with pytest.raises(ValueError):
            enums.get(name).index(next(iter(enums.get(name))))


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


# ---------------------------------------------------------------------------
# BIZ-CM-0. The item adds six sets to this package and words three criteria; the
# first is the E_* literal scan and lives with the lint, in
# tests/common/test_no_literal_ecode.py. The second and third are here.
# ---------------------------------------------------------------------------

def test_every_set_is_read_from_the_volume_it_declares():
    """*** Guards the guard, on the axis the symmetric difference cannot see.

    An extractor aimed at the wrong volume still produces a set, and nothing
    else here would notice. Measured, not assumed: pointing charge_stage's
    SET_DOC entry at 11 leaves the symmetric difference GREEN, because 11's
    patch section carries a verbatim restatement of the 15 S8.5 table and the
    same header predicate finds it. So the wrong volume, the wrong section, and a
    copy that nobody promises to keep current -- and every value still matches.

    That is the whole reason this case exists. Mutation: set SET_DOC's
    charge_stage entry to 11 => red here and green everywhere else, which was run
    and is what the paragraph above records.

    Once 11's patch section is merged and removed, reading 11 would fall through
    to its charging event table -- the one headed with a charge_stage
    synchronisation column, which has no to_handover row and would silently
    shorten the set. test_charge_stage_is_not_read_from_the_contracts_
    synchronisation_column is what stands in front of that second route.
    """
    for name in sorted(enums.SET_NAMES):
        declared = enums.get(name).source.split()[0]
        assert SET_DOC[name] == declared, (
            f"{name}: the extractor reads volume {SET_DOC[name]} but the library "
            f"cites {declared!r}; one of the two is pointed at the wrong table"
        )


def test_the_repeated_tables_agree_with_the_body_text():
    """*** 11 carries merge-ready patch blocks that restate whole tables.

    Taking the first match selects the body text, which is right. What it does
    NOT establish is that the restatement still says the same thing -- and a
    silent rollback through a whole-table replacement is a failure this project
    has already recorded against 11. Every occurrence of each table header is
    therefore parsed and compared.

    Mutation: change one value inside the later copy of the suspend_reason table
    => red. Run against an in-memory copy of the document below, so the mutation
    needs no edit to a tracked file.
    """
    for name, (header_pred, colno, row_pred, value_re, take_all) in TABLE_SETS.items():
        tables = _all_tables_column(_DOCS[SET_DOC[name]], header_pred, colno,
                                    row_pred, name, value_re, take_all)
        for i, later in enumerate(tables[1:], start=1):
            assert later == tables[0], (
                f"{name}: occurrence {i} of this table disagrees with the body "
                f"text: {tables[0]} vs {later} -- a patch block and the section "
                "it amends have diverged"
            )


def test_the_repeated_tables_really_are_repeated():
    """What keeps the case above from comparing one thing with itself.

    If 11 ever stops repeating these tables the comparison becomes vacuous while
    staying green, which is CLAUDE.md 3.2 form 1. This case fails instead, with
    instructions: check whether the duplicate was resolved on purpose before
    deleting either case.
    """
    for name in ("suspend_kind", "suspend_reason", "task_state"):
        header_pred, colno, row_pred, value_re, take_all = TABLE_SETS[name]
        tables = _all_tables_column(_DOCS[SET_DOC[name]], header_pred, colno,
                                    row_pred, name, value_re, take_all)
        assert len(tables) > 1, (
            f"{name}: 11 no longer restates this table. If the patch section was "
            "merged and removed on purpose, drop this set from the list here; do "
            "not delete test_the_repeated_tables_agree_with_the_body_text, which "
            "still guards the sets that are restated"
        )


def test_the_mutated_patch_block_is_detected():
    """*** The mutation for the case above, run rather than described.

    A copy of 11 with one value changed inside the LATER copy of the
    suspend_reason table. The body text is untouched, so the symmetric difference
    stays green and only the agreement check can see it -- which is the whole
    point: this is what a silent rollback looks like.
    """
    header_pred, colno, row_pred, value_re, take_all = TABLE_SETS["suspend_reason"]
    lines = list(_DOCS["11"])
    # Find the row in the LAST occurrence of the table, not the first: mutating
    # the body text would be caught by the symmetric difference and would prove
    # nothing about the agreement check.
    hits = [i for i, ln in enumerate(lines)
            if ln.startswith("|") and header_pred(_cells(ln))]
    assert len(hits) > 1, "this mutation needs the table to be restated"
    for i in range(hits[-1], len(lines)):
        if "`preempted`" in lines[i]:
            lines[i] = lines[i].replace("`preempted`", "`yielded_to`")
            break
    else:
        raise AssertionError("preempted row not found under the last occurrence")

    tables = _all_tables_column(lines, header_pred, colno, row_pred,
                                "suspend_reason", value_re, take_all)
    assert tables[0] != tables[-1], (
        "the mutated copy parsed identically to the body text, so the agreement "
        "check could not have detected it"
    )


@pytest.mark.parametrize("name", sorted(enums.SET_NAMES))
def test_out_of_set_value_raises_for_every_set(name):
    """*** BIZ-CM-0 criterion 3, one case per set: raise, never pass through.

    11 S13.6 forbids both silent pass-through and degrading an unknown value to
    something close. The probe below is built from the set's own name so it
    cannot accidentally BE a member, and three properties are asserted rather
    than one:

      * the type is ClosedSetViolation, so a caller catching the family catches
        it. A bare ValueError would slip past the handler written for this case.
      * the offending value appears in the message. Without it the log says a
        value was rejected and not which, and the field it came from is usually
        one of several on the same message.
      * the set name appears too, for the same reason from the other side.

    Mutation the item names: make parse() return None on a miss => red here for
    every set at once, because pytest.raises fails when nothing is raised. Run
    against the real module rather than assumed; see the note in the item report.
    """
    bogus = f"not_a_{name}_value"
    assert bogus not in enums.get(name), "the probe must not be a real member"
    with pytest.raises(ClosedSetViolation) as caught:
        enums.get(name).parse(bogus)
    assert bogus in str(caught.value)
    assert name in str(caught.value)
    # And by the by-name route as well. Two entry points reach the same lookup,
    # and a fallback added to one of them would be invisible from the other.
    with pytest.raises(ClosedSetViolation):
        enums.parse_enum(name, bogus)


@pytest.mark.parametrize("name", sorted(enums.SET_NAMES))
def test_every_member_of_every_set_passes_through_unchanged(name):
    """*** The positive half, without which "raises on everything" passes.

    CLAUDE.md 3.2 form 1: an implementation that rejected every value would
    satisfy the case above for all fourteen sets and be catastrophically wrong --
    every boundary decode in the system would fail. Identity is asserted, not
    just absence of a raise, because a parse() that normalised its input would
    also return without raising while quietly creating a second spelling.
    """
    for value in enums.get(name):
        assert enums.get(name).parse(value) == value
        assert enums.parse_enum(name, value) == value


@pytest.mark.parametrize("name", ["release_reason", "arb_suspended", "gate_reason",
                                  "suspend_kind", "suspend_reason", "charge_stage"])
def test_the_six_added_sets_carry_no_order(name):
    """None of the six contract sites states an order, so none may hand one out.

    charge_stage is the one that looks most like a sequence and is not: the same
    table sends charging BACK to waiting_plug on an unplug confirmation, so a
    rank read off row position would run backwards while looking like progress.
    """
    assert not enums.get(name).ordered
    # The behaviour, not only the flag. A flag checked alone leaves an index()
    # that answered anyway, and that half is the one a caller reaches.
    with pytest.raises(ValueError):
        enums.get(name).index(next(iter(enums.get(name))))


def test_release_reason_does_not_carry_mode_switch():
    """*** Pins a decision that would otherwise be re-made wrongly every time.

    14 S13 CR-1 asks 11 to add mode_switch to this enumeration, and 14 S3.2.1
    reads as though it already has one ("v0.7 已含"). It does not: grep 11 for
    release_reason and four values come back. Anyone reading 14 alone will want
    to add the fifth value here, and that would make the symmetric difference red
    against the contract as it stands.

    14 supplies the transitional instruction in so many words -- emit mode_exit
    and mark detail.note self_held_handover -- so nothing is blocked by the
    absence. This case exists to make the reason findable at the moment somebody
    reaches for the edit, rather than as a preference about the value.
    """
    assert "mode_switch" not in enums.RELEASE_REASON, (
        "mode_switch is not in 11 S7A.1 today. 14 S3.2.1 gives the transitional "
        "form: release with mode_exit and mark detail.note self_held_handover. "
        "Amend 11 first, then this set"
    )
    # mode_exit itself must be present, or the transitional form has nothing to
    # travel on and the assertion above would be satisfied by an empty set.
    assert "mode_exit" in enums.RELEASE_REASON


def test_arb_suspended_rejects_the_absent_case():
    """null is not a member: absence is not a value.

    ArbDomainState.suspended is string|null, and null means the domain is armed.
    A decoder must branch on the null BEFORE calling parse(), so every spelling
    of "nothing here" has to raise. If any of them were accepted, the normal case
    and the disarmed case would validate through one call and stop being
    distinguishable -- and 11 G-10 turns on exactly that distinction: a motion
    domain disarmed with soft_estop still forwards commands rather than
    answering E_ARB_DISARMED.
    """
    for absent in ("null", "none", "None", "", "normal", "armed"):
        with pytest.raises(ClosedSetViolation):
            enums.ARB_SUSPENDED.parse(absent)
    # The positive half: all three real disarm reasons pass.
    for real in ("soft_estop", "hes", "cmd_timeout"):
        assert enums.ARB_SUSPENDED.parse(real) == real


def test_suspend_reason_keeps_both_values_of_the_shared_rows():
    """*** A tripwire on the WRONG FIX, not on the extractor.

    Two rows of the 11 S4.4 table carry a pair of reasons sharing one
    suspend_kind. Losing take_all drops the second of each pair, and the
    symmetric difference does go red -- but with the message "contract has values
    the library is missing", which reads as "sets.yaml is out of date" and
    invites the repair that deletes them from sets.yaml instead. This case fails
    on the library side and names them.

    energy_unreachable and estop_hes are not decorative: the first is raised when
    no reachable charger has enough energy budget and the second by a hardware
    e-stop, so a task machine that could not spell them would have to report a
    hardware stop as something else.
    """
    for second_of_a_pair in ("energy_unreachable", "estop_hes"):
        assert second_of_a_pair in enums.SUSPEND_REASON, (
            f"{second_of_a_pair} shares a table row with another reason in 11 "
            "S4.4; an extractor taking one value per row silently drops it"
        )


def test_charge_stage_is_not_read_from_the_contracts_synchronisation_column():
    """*** A tripwire on the other wrong fix.

    11 has a charge_stage column of its own, in the S9.11.2 charging event table.
    Reading it is the natural mistake -- 11 is the contract, and the column is
    literally headed charge_stage -- and the result would be a set that parses
    and looks right. It is not right: that column names the value each EVENT
    synchronises to, and no event is emitted while the robot is still driving, so
    to_handover has no row. A library built from it would reject the stage the
    task is actually in for the whole first leg of a charging task.

    Asserted through the document rather than through the library, so this stays
    a statement about 11 rather than a restatement of sets.yaml.
    """
    # Read through the same machinery every other set uses, so this measures 11
    # rather than measuring a second parser written for one case. The header cell
    # naming the column is the key; it raises if the table moves, which is the
    # right outcome -- an empty result must not read as "no to_handover here".
    sync = _table_column(
        _DOCS["11"],
        lambda c: len(c) > 3 and "`charge_stage`" in c[3] and "同步值" in c[3],
        3, "charge_stage_sync_column", VALUE_RE, False)
    assert "to_handover" not in sync, (
        "11's synchronisation column now carries to_handover. Check whether 11 "
        "took over the definition from 15 S8.5 before changing SET_DOC"
    )
    # And the consequence, so the case pins the size of the gap rather than only
    # its existence: the library must hold the value the contract column lacks.
    assert "to_handover" in enums.CHARGE_STAGE
    assert set(sync) < set(enums.CHARGE_STAGE), (
        "the synchronisation column is no longer a strict subset of the stage "
        "set; the two documents have moved relative to each other"
    )


# ---------------------------------------------------------------------------
# The deployed C++17 header. BIZ-CM-0 asks for one so chassis_relay can reuse
# these sets, and CLAUDE.md 5.3 says why that consumer decides the shape: it is
# on the emergency-stop path, so nothing under common/ may reach for rclcpp.
#
# *** Be exact about what these cases establish. The header is GENERATED from
# sets.yaml, so comparing it with sets.yaml is true by construction and proves
# nothing about the values -- CLAUDE.md 3.2 form 7. What it does catch is a hand
# edit to the generated file, which is the realistic defect and the one that
# would leave the two languages disagreeing. The values themselves are held
# correct by the symmetric-difference cases above, against the design volumes.
# Two different assertions, on purpose, in that order of importance.
# ---------------------------------------------------------------------------

GEN_SCRIPT = os.path.join(ROOT, "scripts", "gen", "closed_sets_h.py")
CXX_HEADER = os.path.join(ROOT, "common", "include", "xbrain", "enums",
                          "closed_sets.h")


def _cxx_sets(text):
    """{set name: [values]} parsed back out of the generated header.

    Parsed from the text rather than imported from the generator, so this reads
    what a C++ compiler would read. Asking the generator would compare the
    generator with itself.
    """
    found, current = {}, None
    for line in text.split("\n"):
        stripped = line.strip()
        # The citation comment carries the set name; the array declaration that
        # follows carries the C++ identifier, which is derived and therefore not
        # a second name to keep in step.
        marker = re.match(r"^// ([a-z_]+) -- ", stripped)
        if marker:
            current = marker.group(1)
            found[current] = []
            continue
        value = re.match(r'^"([a-z_]+)",$', stripped)
        if value and current:
            found[current].append(value.group(1))
        elif stripped == "};":
            current = None
    return found


def test_the_committed_cxx_header_matches_a_fresh_render():
    """*** The drift gate, run as CI would run it.

    Mutation: add a value to the committed header by hand => red. Run; see the
    case below, which performs exactly that edit against a copy.
    """
    proc = subprocess.run([sys.executable, GEN_SCRIPT, "--check"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_a_hand_edit_to_the_generated_header_is_detected(tmp_path):
    """*** The mutation for the case above, performed rather than described.

    A copy of the tree with one value added to the header by hand. The generator
    is pointed at the copy through its module globals, so the committed file is
    never touched -- a mutation that edits a tracked file and restores it leaves
    the tree broken whenever it is interrupted.
    """
    victim = tmp_path / "closed_sets.h"
    text = open(CXX_HEADER, encoding="utf-8").read()
    # Inserted into a real array rather than appended at the end of the file, so
    # the mutation is the one somebody would actually make: a value quietly added
    # on the C++ side only.
    mutated = text.replace('    "soft_estop",\n',
                           '    "soft_estop",\n    "power_cut",\n', 1)
    assert mutated != text, "the anchor line moved; re-derive this mutation"
    victim.write_text(mutated, encoding="utf-8")

    # A subprocess with OUT_PATH redirected, rather than a monkeypatch in this
    # process: main() is what CI invokes, so main() is what gets exercised, and
    # the committed header is never in reach of the mutation.
    driver = (
        "import sys\n"
        "sys.path.insert(0, %r)\n"
        "import closed_sets_h as g\n"
        "g.OUT_PATH = %r\n"
        "sys.exit(g.main())\n" % (os.path.join(ROOT, "scripts", "gen"), str(victim))
    )
    proc = subprocess.run([sys.executable, "-c", driver, "--check"],
                          capture_output=True, text=True)
    assert proc.returncode == 1, (
        "a hand-added C++ value went undetected:\n" + proc.stdout + proc.stderr)
    assert "DRIFT" in proc.stdout
    # And the control: the same driver against the UNMUTATED file must pass, or
    # the red above could be coming from the redirection rather than the edit.
    control = tmp_path / "control.h"
    control.write_text(text, encoding="utf-8")
    ok = subprocess.run(
        [sys.executable, "-c", driver.replace(str(victim), str(control)),
         "--check"], capture_output=True, text=True)
    assert ok.returncode == 0, ok.stdout + ok.stderr


def test_the_two_languages_export_the_same_sets():
    """The tripwire a reader will look for, with its limit stated.

    Set for set and value for value, order included. This follows from the file
    being generated, so it is not independent evidence -- it is what makes the
    breakage LEGIBLE when the header has been edited by hand: the failure names
    the set and the value rather than printing a diff of a whole file.
    """
    parsed = _cxx_sets(open(CXX_HEADER, encoding="utf-8").read())
    assert set(parsed) == set(enums.SET_NAMES), (
        "the deployed header and the Python package do not carry the same sets: "
        f"{sorted(set(parsed) ^ set(enums.SET_NAMES))}"
    )
    for name in sorted(enums.SET_NAMES):
        # Sequence, not set equality. gate_limiter's order IS the attribution
        # priority, and a reordered array holds exactly the same strings.
        assert parsed[name] == list(enums.get(name)), name


def test_the_deployed_header_reaches_for_no_ros_type():
    """CLAUDE.md 5.3, asserted here as well as through the build.

    tests/common/link_no_ros/ is the real gate: it compiles every header under
    common/include with no ROS include directory on the command line. This case
    is the cheap one that names the file, so a ROS include added here is
    reported against this header rather than as a compile error in an aggregate
    nobody wrote.
    """
    text = open(CXX_HEADER, encoding="utf-8").read()
    for forbidden in ("rclcpp", "sensor_msgs", "std_msgs", "rcl_", "ament_"):
        for line in text.split("\n"):
            if line.startswith("#include") and forbidden in line:
                raise AssertionError(
                    f"the deployed header includes {forbidden}: {line!r}")
    # The positive half: it really does include something, so a header emptied
    # by a broken render would not pass the exclusions above by holding nothing.
    assert "#include <string_view>" in text
