"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_common_skeleton.py
Brief: CFG-CF-2 -- configs/common.yaml declares every shared key position

Description:
WHAT THIS GUARDS. 10 S5.4.4 assertion M (anchor: 必填键完备性) says the required
key list IS the whole left column of the 10 S5.4.5 comparison table (anchor:
共享参数唯一定义处对照表). configs/common.yaml is the L1 file that has to declare
those positions. If a position is simply absent, no L1~L6 layer provides the key,
an L0 dataclass default fills it, and the stack starts on a value nobody chose --
with assertion A silent, because A inspects VALUES and a key nobody declared has
no value to inspect. That failure chain is written out in the assertion M row and
is the reason this file exists.

*** THE TWO OPERANDS ARE KEPT INDEPENDENT ON PURPOSE. The required list is PARSED
out of docs/10 at test time; it is not transcribed into this file. A transcribed
copy would make the test agree with itself: whoever forgot a key in the yaml is
the same person who would have forgotten it in the copy, and the check would pass
while the design table and the shipped file disagreed. Parsing means adding a row
to the table turns this suite red until someone declares the key -- which is the
direction that has to work. (CLAUDE.md 3.2 form 7: a criterion that defines its
own premise cannot have a counterexample.)

DECLARED SCAN FACES, so no result here reads as broader than it is:

  required key list   docs/10-顶层设计.md, the rows of the 5.4.5 comparison table
                      ONLY -- from the 对照表 marker down to the 别名表 heading.
                      The alias table below it is deliberately OUTSIDE: its rows
                      are a blacklist of forbidden private key names plus one
                      reverse entry, so a path there is not a key position L1 owes
                      anybody. Feeding it in would demand a per-vehicle calibration
                      leaf (the ptz_base camera height) in the shared L1 file.
  key slot check      the parsed yaml tree of configs/common.yaml, internal nodes
                      included, so a namespace entry (calib.*) is satisfied by any
                      declared child.
  zero-placeholder    the RAW text of configs/common.yaml, comments included. The
                      strictest available surface: it also catches a zero written
                      inside a comment as an example, which is how a value gets
                      copied into the key below it.

*** WHAT THIS DOES NOT ESTABLISH. It does not run the freeze service and it does
not evaluate assertion M. Nothing in the repository implements assertion M today,
so "the whole stack refuses to start and prints the key path" cannot be evaluated
from here -- that is CFG-CF-9 / the freeze items, and this file must not be read
as evidence for it. What is evaluated here is the half assertion M rests on: the
L1 file really does declare every position the table names, and the detector below
names the missing path AND the layer it is missing from, which is the message
assertion M has to be able to produce.

A NOTE ON THE MUTATION THE CRITERION NAMES. Deleting the common.safety.t_lat_s
line from configs/common.yaml turns test_key_slot_exists red here. It does NOT by
itself make a merged L0~L5 product lose the key: configs/safety/brake.yaml is the
unique source for that leaf and L3 still supplies it. So the criterion's mutation
is decisive for the L1 skeleton, and only the L1 skeleton -- see the notes handed
back with this task.
"""

import os
import re
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config import layers, merge  # noqa: E402

# INF-TS-1 三档 marker. 本文件是纯静态/元检查(读文件与仓库状态),
# 不碰任何硬件, 故 no_device -- 2026-08-23 从 legacy 未标记名单迁出.
pytestmark = pytest.mark.no_device

# tests/ sits outside the scan surface of scripts/lint/no_config_source_read.py
# (SCAN_DIRS is xbrain, ros2_ws, services) precisely so a test may name the config
# SOURCE. This file has to: the artifact under test is the source file, not the
# resolved snapshot, and the snapshot does not exist before freeze has ever run.
COMMON_YAML = os.path.join(ROOT, "configs", "common.yaml")

# The design volume is addressed by section anchor, never by line number (NUM-4):
# the volumes keep growing and a measured round of this project had three
# line-number references all drift onto blank lines while the citing side believed
# it had checked them.
TOP_DESIGN = os.path.join(ROOT, "docs", "10-顶层设计.md")

# Both anchors are plain words, chosen because they carry no punctuation and can
# be grepped by hand to check this parser is looking where it claims.
TABLE_START_ANCHOR = "**对照表**"
TABLE_END_ANCHOR = "别名表"

# Full-width parentheses, written as codepoints rather than as characters.
# charset_lint.py sets the precedent and gives the reason: a file that spells out
# the characters it is scanning for reports itself, and the usual repair is to
# loosen the check. Content inside these parentheses is stripped before parsing
# because it holds EXAMPLES of children (the profile table names one leaf of the
# two profiles, the pragma row ends in an ellipsis), and treating an example as a
# required path would demand keys the table never promised.
_LPAREN = chr(0xFF08)
_RPAREN = chr(0xFF09)
PARENTHETICAL_RE = re.compile(_LPAREN + "[^" + _RPAREN + "]*" + _RPAREN)

# A backticked span in the table cell. Everything the table means as a key path is
# in backticks; everything not in backticks is prose.
BACKTICKED_RE = re.compile(r"`([^`]+)`")

# A fully qualified path. Anchored at common. so a value such as an endpoint or a
# volume number can never be mistaken for one.
QUALIFIED_RE = re.compile(r"^common(?:\.[A-Za-z_][A-Za-z0-9_]*)+$")

# A bare leaf name written as shorthand for a sibling of the previous qualified
# path, which is how the table writes the five limits, the four databases and the
# three retention periods. Lower case only: that is what every leaf in the tree
# looks like, and it keeps prose tokens out.
BARE_LEAF_RE = re.compile(r"^[a-z_][a-z0-9_]*$")

# The brace form the fence row uses for its four fix qualities.
BRACE_RE = re.compile(r"^(common(?:\.[A-Za-z_][A-Za-z0-9_]*)+)\.\{([^}]*)\}$")

# The layer this file belongs to, taken from the shared loader rather than spelled
# out again. A second hand-kept copy of the layer's name and allowance is the
# seven-parallel-lists failure this project has already measured, and it would go
# stale in the direction that reads as correct.
L1 = next(layer for layer in layers.LAYERS if layer.name == "L1")
L1_LABEL = "%s (%s, %s)" % (L1.name, L1.what, COMMON_YAML)


def _table_rows():
    """The rows of the 5.4.5 comparison table, as raw markdown lines.

    Raises rather than returning empty if either anchor moves. An empty row list
    would make every coverage case below pass vacuously, which is the shape of
    failure that looks like a green suite (CLAUDE.md 3.2 form 1).
    """
    with open(TOP_DESIGN, encoding="utf-8") as handle:
        text = handle.read()
    # The start anchor is searched from the top of the volume rather than from
    # the section heading. Both are unique, and the table marker is the one that
    # actually bounds the rows, so anchoring on it means a renumbered section
    # heading cannot silently shift the surface.
    start = text.find(TABLE_START_ANCHOR)
    if start < 0:
        raise AssertionError("anchor %r not found in %s" % (TABLE_START_ANCHOR, TOP_DESIGN))
    # The end anchor is searched FROM the start, not from the top: the same word
    # appears elsewhere in the volume, and a global search would cut the table
    # short or run past it depending on which occurrence came first.
    end = text.find(TABLE_END_ANCHOR, start)
    if end < 0:
        raise AssertionError("anchor %r not found after the table" % TABLE_END_ANCHOR)
    # Every markdown table row starts with a pipe; the header and separator rows
    # survive this filter and are harmless, because neither carries a backticked
    # path that can pass QUALIFIED_RE.
    return [line for line in text[start:end].split("\n") if line.startswith("|")]


def _paths_of_token(token):
    """(paths, is_namespace) for one backticked token, or ([], False) if it is prose.

    Three spellings appear in the left column and each means the same thing --
    "declare a position here":

      common.zenoh.gen_listen[]   the [] marks the VALUE shape as a list. NET-C9
                                  forbids a wildcard bind, so the endpoint is one
                                  address per interface; the key itself is
                                  common.zenoh.gen_listen either way.
      common.calib.*              a namespace: the vehicle-specific leaves live in
                                  L4b, and L1 owes the branch, not each leaf.
      common.fence.margin_by_fix.{a,b,c,d}   a brace group over four sibling leaves.
    """
    token = token.strip()
    # Value shape first: the brackets say "this key holds a list", which is a
    # statement about what may be written there, not part of the key name.
    if token.endswith("[]"):
        token = token[:-2]
    # Then the namespace marker. Kept as a separate return value rather than
    # folded into the path set, because the two are checked differently: a path
    # must exist, a namespace must exist AND have a child.
    is_namespace = token.endswith(".*")
    if is_namespace:
        token = token[:-2]
    brace = BRACE_RE.match(token)
    if brace:
        head, group = brace.groups()
        members = [head + "." + name.strip() for name in group.split(",") if name.strip()]
        # Filtered through QUALIFIED_RE as well: a malformed group must yield
        # nothing rather than a plausible-looking path nobody ever wrote.
        return [m for m in members if QUALIFIED_RE.match(m)], is_namespace
    if QUALIFIED_RE.match(token):
        return [token], is_namespace
    # Everything else is a value, a volume number or prose. Returning an empty
    # list rather than raising is deliberate -- the left column legitimately
    # contains endpoints and example values, and a parser that refused them would
    # be permanently red and would end up loosened.
    return [], is_namespace


def parse_left_column():
    """(required_paths, required_namespaces) parsed from the 5.4.5 left column.

    The shorthand rule is what makes the parse faithful. The table writes a row as
    `common.db.task_db` . `record_db` . `fence_db` . `geo_db`, so a bare name is a
    SIBLING of the qualified path before it, and the parent is carried within one
    row and reset at the next. Reading a bare name as a child instead would demand
    common.db.task_db.record_db, and the coverage check would then be permanently
    red -- which in this project's measured experience is how a check ends up
    loosened into a permanently green one.
    """
    paths, namespaces = set(), set()
    for line in _table_rows():
        # Cell 0 only. The other cells name the unique definition FILE, the
        # readers and the authoritative volume; they are full of paths that are
        # not key positions L1 owes (models/m20s.yaml, sites/{site_id}.yaml).
        cell = line.split("|")[1]
        cell = PARENTHETICAL_RE.sub(" ", cell)
        # Reset per row. Carrying the parent across rows is the one mistake that
        # would still look like it worked: the counterexample row would inherit
        # the previous row's parent and start contributing paths, and the row
        # above it would silently gain leaves it never named.
        parent = None
        for token in BACKTICKED_RE.findall(cell):
            found, is_namespace = _paths_of_token(token)
            if found:
                (namespaces if is_namespace else paths).update(found)
                # The parent of the LAST member, which is the same for every
                # member of a brace group.
                parent = found[-1].rsplit(".", 1)[0]
                continue
            # A bare leaf only means something after a qualified path in the same
            # cell. Before one there is no parent to attach it to, and the
            # counterexample row (the P1 arbitration priorities, which stay in the
            # process-private file) is exactly that case: it must contribute
            # nothing.
            if parent and BARE_LEAF_RE.match(token.strip()):
                paths.add(parent + "." + token.strip())
    return paths, namespaces


REQUIRED_PATHS, REQUIRED_NAMESPACES = parse_left_column()


def load_common():
    """The parsed configs/common.yaml tree.

    Re-read per case rather than parsed once at import. It costs a few
    milliseconds and it means a mutation applied to the file between cases is
    seen by all of them -- which is how the mutation runs for this item were
    driven.
    """
    with open(COMMON_YAML, encoding="utf-8") as handle:
        # safe_load, so a tag in the file cannot construct a Python object. The
        # freeze service parses this same file; a loader that could be talked
        # into instantiating something would make the config root an execution
        # surface.
        return yaml.safe_load(handle)


def declared_paths(tree):
    """Every dotted path present in the tree, internal nodes included.

    merge.flatten gives leaf paths; the ancestors are added back because a
    namespace requirement is satisfied by the branch existing, and because a row
    such as the pragma block names the branch itself as the required position.
    """
    # merge.flatten is the loader's own flattener, reused rather than reimplemented
    # here. It treats an empty map as a LEAF on purpose, so a branch someone
    # emptied out still appears in the key set instead of vanishing from every
    # check that walks it.
    leaves = merge.flatten(tree)
    out = set(leaves)
    for path in leaves:
        parts = path.split(".")
        # range starts at 1: the ancestors, never the leaf itself, which is
        # already in the set.
        for i in range(1, len(parts)):
            out.add(".".join(parts[:i]))
    return out


def missing_slots(tree, paths, namespaces):
    """[(key_path, layer_label)] for every required position with no key slot.

    Returning the layer label alongside the path is not decoration. Assertion M is
    required to print the key path AND which layer it is missing from, because the
    operator reading it has to know which FILE to open; a bare key path sends them
    to the wrong one whenever a leaf is declared in L1 and valued in L3.
    """
    present = declared_paths(tree)
    # sorted() so the report is stable between runs. An unordered report of the
    # same defect reads as a different defect each time, and the first thing that
    # gets doubted is the checker.
    missing = [(p, L1_LABEL) for p in sorted(paths) if p not in present]
    for namespace in sorted(namespaces):
        # A namespace needs a child, not just the branch node: an empty branch
        # declares ownership of nothing and would satisfy a naive membership test.
        if not any(p.startswith(namespace + ".") for p in present):
            missing.append((namespace + ".*", L1_LABEL))
    return missing


# ---------------------------------------------------------------------------
# The parser itself, before anything is concluded from it
# ---------------------------------------------------------------------------

def test_parser_is_not_vacuous():
    """A silently empty parse would make every coverage case below pass.

    The sentinels are chosen one per parse shape, so a regression in any single
    branch is caught rather than averaged away:

      common.safety.t_lat_s              plain qualified path -- and the key the
                                         criterion's mutation targets
      common.spec.max_decel_mps2         bare-sibling shorthand
      common.fence.margin_by_fix.single   brace-group expansion
      common.calib                       namespace form
      common.recording.fence_close_tol_m  the last row-internal sibling, so a
                                         parser that stopped at the first token
                                         of a row is caught

    Mutation run: point TABLE_END_ANCHOR at the section heading instead, so the
    alias table falls inside the scan face => the calibration leaf
    common.calib.frames.ptz_base.h_camera_m enters REQUIRED_PATHS and
    test_key_slot_exists goes red for a key L1 must not own.
    """
    assert "common.safety.t_lat_s" in REQUIRED_PATHS
    assert "common.spec.max_decel_mps2" in REQUIRED_PATHS
    assert "common.fence.margin_by_fix.single" in REQUIRED_PATHS
    assert "common.recording.fence_close_tol_m" in REQUIRED_PATHS
    assert "common.calib" in REQUIRED_NAMESPACES
    # The counterexample row must contribute nothing at all: those priorities have
    # one reader and stay in the process-private file. If it ever leaks in, the
    # shorthand rule has started attaching bare names to the wrong parent.
    assert not any("arbitration" in p for p in REQUIRED_PATHS | REQUIRED_NAMESPACES)


# ---------------------------------------------------------------------------
# The criterion: every left-column position has a key slot
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("path", sorted(REQUIRED_PATHS))
def test_key_slot_exists(path):
    """Every common.* path in the 5.4.5 left column is declared in common.yaml.

    Mutation run (the one CFG-CF-2 names verbatim): delete the common.safety.t_lat_s
    LINE from configs/common.yaml -- delete it, do not set it to null -- and this
    case goes red naming the path and the layer.

    Why deletion and null are not the same failure, which is the whole point of
    the mutation being written that way: null is reported by assertion A as an
    unassigned key path, while an absent key is reported by assertion M as a key
    no layer provides. A file that answered the criterion by writing null
    everywhere and dropping the position entirely would still start on an L0
    default with nothing logged.
    """
    tree = load_common()
    # One path per case rather than one case for the whole set: pytest then names
    # the missing key in the test id, so the failure is readable from the summary
    # line without opening the report.
    missing = dict(missing_slots(tree, {path}, set()))
    assert path not in missing, (
        "no key slot for %s in %s; declare it with a null value" % (path, L1_LABEL)
    )


@pytest.mark.parametrize("namespace", sorted(REQUIRED_NAMESPACES))
def test_namespace_has_at_least_one_declared_child(namespace):
    """A namespace row is owed a branch with something under it.

    Mutation run: replace the whole calib block with `calib: null` => red. A bare
    null branch reads as "position declared" to a membership test while declaring
    no position at all, which is the same emptiness this project has already been
    bitten by in the layer allowance check.
    """
    tree = load_common()
    # The namespace rows are the ones whose leaves belong to a LOWER file: the
    # calibration branch is filled per vehicle in L4b and the clock branch in the
    # L3 safety file. L1 still owes the branch, because a branch nobody declares
    # is a branch assertion M cannot report on.
    missing = dict(missing_slots(tree, set(), {namespace}))
    assert (namespace + ".*") not in missing, (
        "namespace %s.* has no declared child in %s" % (namespace, L1_LABEL)
    )


def test_missing_slot_is_reported_with_key_path_and_layer():
    """The detector must NAME what is missing, not merely fail.

    This is the positive case for the detector itself. Without it, an
    implementation of missing_slots that returned [] for everything would make
    every case above green -- the shell-implementation failure mode from
    CLAUDE.md 3.3, where three negative assertions all passed an implementation
    that had no state machine at all.

    Mutation run: make missing_slots return [] unconditionally => this goes red
    while nothing else does.
    """
    tree = load_common()
    # Remove exactly the line the criterion names, in memory, so the real file is
    # never touched by the test run. An in-memory mutation is not a substitute for
    # running the criterion's mutation against the file itself -- that was run
    # separately and is reported with this task -- it is what keeps the detector
    # honest on every future run, when nobody is re-running mutations by hand.
    #
    # pop with a default rather than del: this case is about the DETECTOR, and it
    # must keep answering the same way while the file itself is being mutated by
    # hand. With del, running the criterion's mutation on the file would make this
    # case error out on a KeyError and bury the failure it is supposed to leave
    # standing alone.
    tree["common"]["safety"].pop("t_lat_s", None)
    missing = missing_slots(tree, {"common.safety.t_lat_s"}, set())
    assert missing, "deleting a required key must be detected"
    path, layer = missing[0]
    assert path == "common.safety.t_lat_s"
    # Both halves of the message assertion M owes the operator.
    assert "L1" in layer and "common.yaml" in layer


# ---------------------------------------------------------------------------
# The shape of the file
# ---------------------------------------------------------------------------

def test_top_level_key_is_only_common():
    """10 S5.4.3 L1 row: only common.*, any other top-level key is refused.

    Mutation run: add a second top-level block to configs/common.yaml => red.
    """
    tree = load_common()
    assert set(tree) == {"common"}, (
        "L1 may only write common.*; found top-level keys %s" % sorted(tree)
    )


def test_layer_allowance_holds_for_the_real_file():
    """The same rule, evaluated by the loader instead of by this file.

    Reusing check_namespace rather than restating the allowance is what stops the
    two from drifting: if the L1 allowance is ever changed in layers.py, this case
    follows it, and a test carrying its own copy would keep passing against a rule
    that no longer exists.

    Mutation run: put a non-common top-level key in the tree => ConfigLayerError,
    proving the check is actually reached rather than trivially satisfied.
    """
    tree = load_common()
    # flatten, not the tree: check_namespace matches on the full dotted path, and
    # the cheaper "is the top-level key common" test would pass a file that wrote
    # a safety key it has no business owning.
    layers.check_namespace(L1, merge.flatten(tree))
    # And the negative half in the same case. A check_namespace that raised on
    # nothing would satisfy the line above on its own, which is the shell
    # implementation this project keeps measuring.
    with pytest.raises(layers.ConfigLayerError):
        layers.check_namespace(L1, merge.flatten({"speed_profiles": {"patrol": 1}}))


def test_counterexample_key_is_not_here():
    """10 S5.4.5 counterexample row: the P1 arbitration priorities stay private.

    They have one reader, and 14 S11's resource-domain arbitration is a different
    thing wearing a similar name; merging the two into common is the outcome that
    row exists to prevent. Scan face is the PARSED key set, not the file text, so
    the sentence above -- which names the key in prose -- cannot make its own
    criterion unreachable (CLAUDE.md 3.2 form 3).

    Mutation run: add an arbitration.priorities block under common => red.
    """
    declared = declared_paths(load_common())
    # Substring, not equality: the name must not appear at ANY depth. Checking
    # only for a top-level block would miss the realistic version of this
    # mistake, which is somebody tucking the priorities under the shared
    # arbitration-looking branch a level down.
    assert not [p for p in declared if "arbitration" in p]


# ---------------------------------------------------------------------------
# * The placeholder rules
# ---------------------------------------------------------------------------

def test_no_zero_placeholder_anywhere_in_the_file():
    """*** An uncalibrated safety parameter is null, never a zero.

    10 S5.4.5 spells out the failure: a zero passes assertion A as "assigned" and
    then limits v_max to min(..., 0) = 0 at run time with nothing logged. That is
    fail-silent, one grade worse than fail-safe, and it is why this case scans the
    RAW file including comments -- a zero written in a comment as an example is
    how the value ends up on the key underneath it.

    Mutation run: set common.safety.t_lat_s to a zero => red.
    """
    with open(COMMON_YAML, encoding="utf-8") as handle:
        raw = handle.read()
    # Built from parts so this criterion is not inside its own scan face: a check
    # whose text contains the string it greps for can never reach zero hits.
    zero = "0" + "." + "0"
    assert zero not in raw, "zero placeholder found in %s" % COMMON_YAML

    # The raw scan catches the spelling; this catches the value however it was
    # spelled, including a plain 0 and an exponent form.
    for path, value in sorted(merge.flatten(load_common()).items()):
        # bool is a subclass of int in Python and False == 0, so without this
        # guard a legitimately configured false would be reported as a zero
        # placeholder -- and the clock override flag is exactly such a key.
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            assert value != 0, "%s is assigned a zero; an unset value is null" % path


#: Leaves deliberately LANDED, each with the reason it is not a skeleton hole.
#: Adding to this dict is the "deliberate act" the case below describes; it is
#: NOT a way to silence the check, because everything outside it still fails.
_LANDED = {
    # 2026-08-23: landed by 473dd81 (站点显示时区), which per this case's own
    # instructions should have updated the test in the same commit and did not.
    # Authorised as a landed value by 16 S(p4 config header) and 17 S6.10.2,
    # both verbatim: it is a DISPLAY value only -- the HMI clock, the G24 spoken
    # time and the CHS-A Time field -- while every timeout/period uses the
    # monotonic clock (CLK-C1) and the internal wall clock is always UTC.
    # "本值[不影响安全]". A null here would break G24 instead of protecting
    # anything, which is why it is landed rather than left unassigned.
    "common.timezone": "Asia/Shanghai",
}


def test_every_declared_leaf_is_null():
    """CFG-CF-2 delivers key POSITIONS; every value is still unlanded.

    Read this case as a statement about today's file, not as a ban on ever landing
    a value. Landing one is a deliberate act that cites the row of 5.4.5
    authorising it, and it updates this case in the same commit -- by adding the
    leaf to _LANDED above with its reason. What must NOT happen is the assertion
    being deleted to let a value through unremarked -- that turns the one check
    that knows this file is a skeleton into nothing.

    Mutation run: give any OTHER leaf a value => red, naming the path; change a
    landed value without updating _LANDED => red too, because the expected value
    is pinned, not just the key.
    """
    for path, value in sorted(merge.flatten(load_common()).items()):
        if path in _LANDED:
            # Pinned by VALUE, not merely exempted by key: an exemption that
            # only named the path would let the timezone be silently changed
            # to anything at all.
            assert value == _LANDED[path], (
                "%s is a landed leaf but its value changed to %r; update "
                "_LANDED (with the reason) in the same commit" % (path, value)
            )
            continue
        assert value is None, (
            "%s carries a value; CFG-CF-2 is a key-position skeleton and every "
            "landed value needs the 10 S5.4.5 row that authorises it" % path
        )


def test_empty_container_is_not_used_as_a_placeholder():
    """An empty list or map is "assigned", exactly like a zero is.

    This is the same trap in the shape the endpoint list invites: gen_listen is a
    list of one address per interface, so writing [] looks like "declared, nothing
    yet" while actually meaning "listen on no interface at all" -- a router that
    starts and that nobody can reach, reporting nothing.

    Mutation run: write gen_listen as an empty list => red. Note merge.flatten
    keeps an empty map as a LEAF for this very reason, so both spellings are seen.
    """
    for path, value in sorted(merge.flatten(load_common()).items()):
        assert value != [], "%s uses an empty list as a placeholder; use null" % path
        assert value != {}, "%s uses an empty map as a placeholder; use null" % path
