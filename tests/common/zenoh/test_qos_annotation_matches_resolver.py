"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_qos_annotation_matches_resolver.py
Brief: T35 -- the 11 S2.2 key-table QoS annotations agree with the resolver

Description:
What this case is worth, and why it is separate from test_qos_resolve.py.

The sibling file test_qos_resolve.py already pins the RESOLUTION side: given the
S2.4.7 bindings, each key resolves to one profile, and test_fixture_bindings_are_
verbatim keeps that fixture from drifting away from the contract. What it does NOT
check is the OTHER copy of the same fact -- the human-written QoS column in the
S2.2 key table. Those cells carried a prediction, "Q3-rt <- should be Q0", from
back when the bindings had not yet been fixed. The v0.3 / v1.8 binding repairs
made the prediction come true, but the cells kept saying "should be" for months
(T35). This case ties the cell to the resolver so the two cannot drift apart
again: either the binding changes and the cell no longer matches, or someone
reverts a cell, and one of the two assertions below goes red.

Why these nine keys and not the whole table. The nine are exactly the rows T35
reconciled -- the ones whose cell used to read "Qx <- should be Y". They are
listed by hand, the same way test_qos_resolve.py lists its golden vectors,
because a test that discovered its own key list from the table would agree with
the table by construction (CLAUDE.md 3.2 form 7) and could not catch a row that
was quietly reverted. A new key added later with a "should be" marker is out of
scope on purpose; it is not one of the rows this item closed.

How a row is found without ambiguity. The S2.2 key table writes keys in the
full-prefixed, back-ticked form `xbrain/{rid}/rt/safety/estop`. The S2.2.12 gap
table (kept as a historical record) writes the SAME keys in the bare form
`rt/safety/estop`, with no prefix. The needle here carries the `xbrain/{rid}/`
prefix and the trailing back-tick, so it matches only the key-table row and not
the gap table, and `.../estop` does not match `.../estop/ack` because the
trailing back-tick ends the token.

*** What this case does NOT establish, so a green run is not over-read:
  * It does not open a Zenoh session. The knobs being applied as described is
    QoS-T1..T8 (11 S2.4.9), pending T7. This only checks the DOCUMENT agrees
    with the resolver.
  * It does not check keys outside the nine. The S2.2.12 gap table still
    contains the word this case forbids on a key line, as a column header and in
    its historical rows written with the bare key form -- neither is one of the
    nine prefixed needles, so neither is touched.
  * It does not re-derive the bindings. It trusts test_fixture_bindings_are_
    verbatim (same directory) to hold the fixture honest against 11 S2.4.7.

MUTANTS (run 2026-10-15, both red):
  * revert one cell's profile token (Q0_safety -> Q3_cmd): the profile-match
    assertion goes red for that key.
  * revert one cell to the old "Qx <- should be Y" wording: the stale-marker
    assertion goes red, because the marker returns to the key line.
"""

import json
import os

from xbrain.common.zenoh.qos import load_qos_table

# The project root, four levels up from tests/common/zenoh/<this file>.
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

# The contract under test. Read as text, not parsed: this case asserts on the
# exact cell wording a human reads, which a markdown table parser would strip.
_CONTRACT = os.path.join(_ROOT, "docs", "11-接口契约.md")

# The S2.4.7 document, loaded from the same golden fixture the sibling file
# holds verbatim-honest against the contract. Loading it here rather than
# re-parsing 11 keeps a single source of truth for the bindings.
_GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden",
                       "qos_bindings_vectors.json")

# The stale-prediction marker, spelled as unicode escapes so this source file
# stays pure ASCII (CLAUDE.md 2.2). It is the two Han characters meaning
# "should be" -- the exact token the pre-T35 cells used, e.g. "Q3-rt <- X".
# Its presence on one of the nine key lines means a cell was reverted to the
# prediction form it had before the bindings were fixed.
_STALE_MARKER = "应为"

# The nine keys T35 reconciled, as key-suffixes under xbrain/<rid>/. The
# resolver needs a concrete rid; the contract writes the literal "{rid}". Both
# forms are derived from this one list so they cannot fall out of step.
_RECONCILED_SUFFIXES = [
    "rt/safety/estop",
    "rt/safety/probe/ping",
    "rt/safety/probe/pong",
    "rt/safety/estop/ack",
    "rt/audio/mic",
    "rt/audio/play",
    "rt/teleop/input",
    "probe/estop/ping",
    "probe/estop/pong",
]


def _table():
    """A QosTable over the same fixture the sibling resolver test uses."""
    with open(_GOLDEN, encoding="utf-8") as handle:
        document = json.load(handle)["qos"]
    return load_qos_table(document)


def _key_table_lines(contract_text, suffix):
    """The S2.2 key-table line(s) naming this key in full-prefixed form.

    The needle is the back-ticked `xbrain/{rid}/<suffix>` token. The trailing
    back-tick is load-bearing: without it `rt/safety/estop` would also match the
    `rt/safety/estop/ack` row. The bare form used by the S2.2.12 gap table has
    no prefix, so it is excluded.
    """
    needle = "`xbrain/{rid}/" + suffix + "`"
    return [ln for ln in contract_text.splitlines() if needle in ln]


def test_s22_qos_annotations_match_the_resolver():
    """Every reconciled S2.2 cell names the profile the resolver computes.

    Two assertions per key, with opposite failure meanings. The first ties the
    cell to the resolver: the row must name the profile that xbrain.common.zenoh
    .qos resolves the key to today. The second guards the reconciliation itself:
    the row must no longer carry the "should be" prediction marker, which is the
    signature of the pre-T35 state. A row can fail either way independently --
    a binding change breaks the first, a reverted cell the second -- so they are
    checked and reported separately.
    """
    with open(_CONTRACT, encoding="utf-8") as handle:
        contract_text = handle.read()
    table = _table()

    # Two accumulators rather than a fail-fast assert inside the loop. A loop
    # that raised on the first bad key would report one key and hide the rest,
    # and the two failure kinds have opposite fixes (a binding moved vs a cell
    # reverted) -- collecting them lets the message say which keys and which
    # kind, so the reader is not sent back to re-run nine times.
    missing_profile = []
    still_stale = []
    for suffix in _RECONCILED_SUFFIXES:
        # A concrete rid is required because the resolver matches on a
        # fully-qualified key; "dev" is arbitrary -- the bindings all match on
        # "xbrain/*/..." so any single rid segment resolves identically, and
        # the golden vectors use their own ("dog-01") for the same reason.
        # .profile_name, not .profile: QosResolution names the field after the
        # profile it matched BEFORE any set clause, which is the profile the
        # cell states (the ring-depth override is a separate detail, not the
        # name).
        resolved = table.resolve("xbrain/dev/" + suffix).profile_name
        # The contract may name a key in more than one place (the key-table
        # row, plus prose cross-references in S7/S8). Collect every line and
        # test the set: at least one is the key-table row and must carry the
        # profile, and none of them may carry the stale marker.
        lines = _key_table_lines(contract_text, suffix)
        # A key with no key-table row at all is a regression of its own: the
        # row was renamed or deleted and this case would otherwise check
        # nothing for it (CLAUDE.md 3.2 form 1). Treat it as a profile miss.
        if not lines:
            missing_profile.append("%s (no key-table row found)" % suffix)
            continue
        # The profile token must appear on the row. Checking every matching
        # line rather than assuming one keeps the case correct if the key also
        # appears in prose elsewhere -- at least one of them is the key-table
        # row and must carry the resolved profile.
        if not any(resolved in ln for ln in lines):
            missing_profile.append("%s (want %s)" % (suffix, resolved))
        # No matching line may carry the stale marker. After T35 the cell reads
        # the resolved profile with a struck-through history that does NOT use
        # the marker word, so its reappearance means a revert.
        if any(_STALE_MARKER in ln for ln in lines):
            still_stale.append(suffix)

    # Two asserts, not one combined. The messages have to steer to opposite
    # fixes: a profile mismatch means the DOCUMENT is behind the resolver (fix
    # the cell, or accept a deliberate binding change), while a stale marker
    # means a reconciled cell was reverted. Merging them into one assert would
    # force a reader to work out which of the two happened from the key list.
    assert not missing_profile, (
        "S2.2 QoS cells disagree with the S2.4.7 resolver (profile not named "
        "on the key row): %s -- the resolver is the source of truth; update "
        "the cell, or if a binding changed, see 11 S2.2.12" % missing_profile)
    # The stale marker is the pre-T35 signature; its return is always a revert,
    # never a legitimate state, for these nine keys -- the binding now delivers
    # the profile, so the cell has nothing left to predict.
    assert not still_stale, (
        "S2.2 QoS cells carry the pre-T35 prediction marker again: %s -- the "
        "binding now delivers the profile, so the cell must state it, not "
        "predict it (11 S2.2.12 records the reconciliation)" % still_stale)
