"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_retention_single_definition.py
Brief: common.retention.* has ONE definition site (L4) and consumers reference it

Description:
10 S5.4.5 lists the "sole definition site" for
common.retention.{task_days, event_days, command_log_days} verbatim as
"sites/{site_id}.yaml . retention (key position in common.yaml)" -- L4.
This file holds that shape down from three directions, because every one
of them was actually broken on 2026-10-08 and none of them was red:

  (1) L1 declares the key POSITION only (null). A value here is a second
      definition of the same quantity.
  (2) The L1 variant (common_sim.yaml) must not supply them either. It
      did, and that is why "freeze ran green with --variant sim"
      (NEXT S8.8) was not evidence about L4 at all: load_layers did not
      read L4, the variant had filled the values at L1, and the L4 path
      was never exercised. A layer that silently writes nothing is the
      hardest kind of hole to notice -- nothing fails.
  (3) Consumers REFERENCE the key, they do not restate the number.
      configs/p5_gateway.yaml carried `retention_days: 90` twice as a
      literal. Changing the site file moved nothing, and assertion C-1
      (which walks the overlay) cannot see an L6 literal at all.

What this file does NOT prove: that the retention values are correct, or
that anything actually deletes rows on that schedule. P3-side cleanup
has no config key yet (configs/p3_task.yaml has zero retention keys
today) -- see docs/NEXT.md. These are placement criteria only.

Why static text checks and not a freeze run: the freeze line needs a
variant to get past the production nulls (iron rule 3), so a test that
ran it would only ever exercise the sim path. Reading the files directly
covers the production layer too, which is the one that matters.

MUTANTS THAT MUST KILL THESE CASES (CLAUDE.md S3.3; all eight were run
on 2026-10-08 and all eight went red):

  load_layers stops merging L4            -> provenance case
  resolve_site_id stops consulting L5     -> inventory override case
  the containment guard is removed        -> separator case
  p5_gateway.yaml goes back to a literal  -> L6-reference case
  common_sim.yaml fills retention again   -> variant case
  sites/sim.yaml drops one key            -> site-file case
  common.yaml fills one in                -> L1 position case

IF ONE OF THESE GOES RED, LOOK HERE FIRST. The most likely cause is not
a code change but a config edit: somebody filling a value at the layer
that happens to be convenient. The failure message names the layer and
the key, so the fix is almost always "move it to sites/{site_id}.yaml",
not "loosen the test". The one case where loosening IS right: a second
site file appears and the question "must every site set all three"
gets decided -- that decision belongs in 10 S5.4.5, and this file
follows it rather than leading it.

WHAT NONE OF THIS GUARANTEES, stated so the green is not over-read:
  - that 30 / 90 / 180 are the right numbers (they come from 15 S9.11;
    this file never looks at the values except to check their type)
  - that any row is ever actually deleted (P3-side cleanup has no
    config key at all -- NEXT RT-1 / RT-2)
  - that the freeze line runs. It does today with --variant sim, and
    that is a separate observation recorded in the commit, not a
    criterion here.
"""

# os for path joins; re to spot a reference vs a literal; yaml to read
# the layer files the same way the loader does.
import os
import re

import pytest
import yaml

from xbrain.boot.freeze.assertions._layer_loader import load_layers
from xbrain.common.config import build_overlay

#: Every case here reads committed files or builds an overlay in memory.
#: Nothing touches the chassis, a camera, or the network, so the whole
#: module is no_device (tests/meta/test_marker_coverage.py requires the
#: declaration rather than inferring it -- an inferred marker would
#: silently reclassify a file the day someone adds a socket to it).
pytestmark = pytest.mark.no_device

#: Repo config root. Tests run from the repo root (conftest ensures it),
#: so a relative path would silently read the wrong tree if that ever
#: changed -- derive it from this file's location instead.
#: Three dirname() calls: this file is tests/configs/<name>.py, so the
#: third one lands on the repo root. Spelled out rather than using a
#: pathlib .parents[2] chain because an off-by-one there reads a
#: DIFFERENT configs/ directory (the sshfs mount has the repo twice in
#: some checkouts) and every case would then pass against the wrong
#: tree while looking entirely healthy.
CONFIGS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "configs")

#: The three keys, as 10 S5.4.5 spells them. Written once here; every
#: case below iterates this tuple rather than repeating the names, so a
#: fourth retention key cannot be added to configs/ and covered by none
#: of the cases just because someone forgot one list.
RETENTION_KEYS = ("task_days", "event_days", "command_log_days")


def _read(path):
    """Read one YAML file into a dict; missing / empty file -> {}.

    Returning {} for a missing file rather than raising: every caller
    then reports "the key is not there", which is the finding, instead
    of a FileNotFoundError that names a path and not a rule. A file
    that genuinely must exist is assertion J's business, not this
    module's.
    """
    # Not reusing _layer_loader._read_yaml: that one raises XbrainError on
    # a parse failure, which would make a broken file look like a
    # placement violation here instead of what it is.
    if not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh.read()) or {}


def test_l1_declares_the_key_position_and_no_value():
    """(1) common.yaml carries the three keys, all null.

    Both halves matter. Missing keys would mean the position is not
    declared (10 S5.4.5 puts the position in common.yaml); a non-null
    value would be the second definition site.
    """
    # Reading common.yaml directly, not through load_layers: the loader
    # applies the variant overlay, and this case is about what the REAL
    # file holds. Going through the loader would make the case pass as
    # soon as a variant happened to leave the keys null.
    tree = _read(os.path.join(CONFIGS, "common.yaml"))
    retention = (tree.get("common") or {}).get("retention")
    assert isinstance(retention, dict), \
        "common.yaml must declare the retention key position (10 S5.4.5)"
    for key in RETENTION_KEYS:
        assert key in retention, \
            "common.yaml is missing the key position for %s" % key
        assert retention[key] is None, (
            "common.yaml must not hold a VALUE for %s -- the sole "
            "definition site is sites/{site_id}.yaml (10 S5.4.5)" % key)


def test_the_l1_variant_does_not_supply_retention_either():
    """(2) common_sim.yaml must not fill these three.

    A variant may only fill keys the real file left null (CLAUDE.md
    S3.6) -- which these are, so the variant mechanism alone does not
    forbid it. What forbids it is the definition site: filling them at
    L1 makes the overlay correct and the provenance wrong, and it hides
    whether L4 is wired at all.
    """
    # `or {}` on both levels: an absent `common:` or an absent
    # `retention:` is the state we want, and treating either as a
    # failure would force the variant to carry an empty stub.
    tree = _read(os.path.join(CONFIGS, "common_sim.yaml"))
    retention = (tree.get("common") or {}).get("retention") or {}
    assert retention == {}, (
        "common_sim.yaml supplies %s; move them to configs/sites/sim.yaml "
        "(L4), which is the sole definition site" % sorted(retention))


def test_the_sim_site_file_supplies_all_three():
    """(3) sites/sim.yaml must carry all three, or the sim freeze stops.

    Paired with the case above: once the variant stops supplying them,
    the site file is the only source left, and assertion A refuses the
    whole boot on one null (it is all-or-nothing). So "the variant does
    not supply them" is only safe while this holds.
    """
    # Only the sim site file is checked. There is exactly one site file
    # in the repo today, and a case that iterated sites/*.yaml would
    # start demanding retention from every future site file -- which is
    # a real question (does a site have to set all three, or may it
    # inherit?) that 10 S5.4.5 does not answer, so this case does not
    # answer it either. If a second site file appears, that question
    # gets decided then, not by a glob written today.
    tree = _read(os.path.join(CONFIGS, "sites", "sim.yaml"))
    retention = (tree.get("common") or {}).get("retention") or {}
    for key in RETENTION_KEYS:
        # isinstance(x, int) is deliberately not `is not None`: a string
        # "30" would satisfy the looser form and then be compared with
        # `<=` against an int by assertion C-1, which in Python 3 raises
        # TypeError at freeze time rather than refusing with a key path.
        assert isinstance(retention.get(key), int), (
            "configs/sites/sim.yaml must supply an integer %s -- it is "
            "the only layer left that defines it" % key)


def test_the_overlay_gets_all_three_from_l4(monkeypatch):
    """load_layers must actually merge L4; provenance proves which layer won.

    This is the criterion for the 2026-10-08 change (CFG-A6 / NEXT T18
    "B12 二次合并 L4"). Asserting the VALUES would stay green with L4
    unmerged for as long as some other layer happened to supply the same
    numbers -- which is exactly the state this change replaced. The
    provenance is what distinguishes "right answer" from "right answer
    for the right reason".
    """
    # resolve_site_id consults L5, and load_layers takes no env
    # parameter (its 14 assertion callers want the real environment).
    # So a developer with XBRAIN_SITE_ID set in their shell would get a
    # different site file and this case would go red for a reason that
    # has nothing to do with the code. Clear it for the duration.
    monkeypatch.delenv("XBRAIN_SITE_ID", raising=False)
    overlay = build_overlay(load_layers(CONFIGS, variant="sim"))
    for key in RETENTION_KEYS:
        dotted = "common.retention.%s" % key
        assert overlay.provenance.get(dotted) == "L4", (
            "%s came from %r, expected L4 -- load_layers is not merging "
            "the site layer" % (dotted, overlay.provenance.get(dotted)))


#: Matches a `retention_days:` entry whose value is a bare number.
#: Deliberately NOT matching "any value": a key set to a reference
#: (${common.retention.event_days}) is the shape we want, and a key set
#: to null is a position declaration, which is somebody else's problem.
_LITERAL_RETENTION = re.compile(
    r"^\s*retention_days:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$",
    re.MULTILINE)


def test_l6_consumers_reference_the_key_instead_of_restating_the_number():
    """No L6 file may hold a numeric literal for retention_days.

    Scope is every *.yaml directly under configs/ except the shared
    layer and its variant (those are L1, covered above) -- i.e. the
    per-process L6 files. Declared here rather than left implicit,
    because "the number is in some file this test did not read" is the
    failure mode CLAUDE.md S3.2 calls "scan surface not declared".

    Failure this prevents concretely: p5_gateway.yaml said
    `retention_days: 90` twice. Editing sites/sim.yaml's event_days
    changed neither, and assertion C-1 compares overlay values, so it
    never saw the L6 copies. The number agreed with L4 by coincidence
    and would have diverged the first time a site used anything else.
    """
    # Collect ALL offenders before asserting, rather than failing on the
    # first: an operator fixing them one per test run is the pattern
    # assertion A's own "keys" field was changed to avoid.
    offenders = []
    # sorted() so the message is stable across filesystems -- an
    # unstable order makes two runs look like two different findings.
    for name in sorted(os.listdir(CONFIGS)):
        if not name.endswith(".yaml"):
            continue
        # common.yaml / common_sim.yaml are L1, not L6.
        if name in ("common.yaml", "common_sim.yaml"):
            continue
        path = os.path.join(CONFIGS, name)
        text = open(path, encoding="utf-8").read()
        for match in _LITERAL_RETENTION.finditer(text):
            # Report file + value; the line number would drift, the
            # value is what the reader greps for.
            offenders.append("%s: retention_days: %s" % (name, match.group(1)))
    assert offenders == [], (
        "these L6 entries restate a retention number instead of "
        "referencing ${common.retention.*}: %s" % offenders)


def test_a_site_id_with_a_separator_cannot_reach_outside_sites(
        tmp_path, monkeypatch):
    """load_layers must not read an L4 file from outside configs/sites/.

    site_id is joined into a path, so a value carrying a separator
    ("../evil") would name a file one directory up and that file's keys
    would land in the overlay under L4's namespace allowance. Nothing
    downstream re-checks where the L4 tree came from -- build_overlay
    merges whatever tree it is handed.

    Why the loader returns {} instead of raising: assertion D
    (10 S5.4.4) already requires site_id to equal the L4 file name and
    reports the mismatch with both names in its detail. Raising here
    would give two different verdicts for one fact, and the earlier one
    (this function, which runs during loading) carries less context.
    An empty L4 makes D fire on exactly the case D exists for.

    This case lives in this file rather than with the other loader tests
    because it guards the same L4 selection path as the provenance case
    above -- they fail together or not at all.
    """
    root = tmp_path / "configs"
    root.mkdir()
    # The file the escape would reach: a sibling of sites/, carrying a
    # key no site file should ever be able to inject.
    (root / "evil.yaml").write_text(
        "common:\n  retention:\n    task_days: 1\n", encoding="utf-8")
    (root / "sites").mkdir()
    # A legitimate site file exists too, so a loader that simply failed
    # to find anything would not look like a pass.
    (root / "sites" / "real.yaml").write_text(
        "common:\n  retention:\n    task_days: 30\n", encoding="utf-8")
    (root / "common.yaml").write_text(
        'common:\n  site_id: "../evil"\n', encoding="utf-8")

    # A real XBRAIN_SITE_ID in the developer's shell would decide this
    # case instead of the file under test (L5 wins over L1).
    monkeypatch.delenv("XBRAIN_SITE_ID", raising=False)
    trees = load_layers(str(root))
    assert trees["L4"] == {}, (
        "site_id '../evil' reached outside configs/sites/ and merged %r"
        % trees["L4"])
    # And the guard must be about containment, not about rejecting every
    # site_id: the plain name still loads.
    (root / "common.yaml").write_text(
        "common:\n  site_id: real\n", encoding="utf-8")
    trees = load_layers(str(root))
    assert trees["L4"]["common"]["retention"]["task_days"] == 30
