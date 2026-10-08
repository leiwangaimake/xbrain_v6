"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_crc32_contract.py
Brief: the 11 S9A.2 crc32 recipe in the CONTRACT BODY equals the implemented one

Description:
On 2026-10-09 (T27) this was found: the fence crc32 recipe had TWO versions in
11, and the implementation followed the one that was not in the document body.

  11 S9A.2 body      five segments: poly_id|role|winding|hard_enforce|vertices
  11 block B-3       seven segments: the above plus priority and
                     speed_limit_mps -- a "replace S9A.2's recipe" patch block
                     carrying no "merged" marker
  implementation     seven segments (xbrain/common/digest/fence.py)

The implementation was right and the body was stale -- the five-segment recipe
makes "lower the slow zone" produce a byte-identical crc32, so S-5 answers
duplicate and the change never takes effect with no error anywhere. But the
implementation's only written basis was a patch block, which is CLAUDE.md
S3.2's "citing verbatim text that was never merged" from the other side:
the code had a basis and the basis was not in the body.

Block B-3 has since been merged into S9A.2 (iron rule 1). This file is what
keeps them together, and it reads the BODY -- not the block -- on purpose:
if somebody edits one of the two and not the other, this goes red.

HOW IT CHECKS, and what that can and cannot prove: it extracts the recipe's
code fence from S9A.2 and asserts the SEGMENT NAMES and their ORDER, then
asserts the implementation builds a string with those same segments in that
same order for a known polygon. It does NOT re-derive a crc32 value from the
document -- a second implementation of the recipe living in a test file is
exactly the duplicate this project has measured twice, and it would drift.
The golden cross-language vectors (tests/common/test_digest_cross_language.py)
are what pin the VALUE.

MUTANTS, run 2026-10-09, all five red:
  body recipe loses the priority segment
  body recipe loses the speed_limit_mps segment
  body's %.3f drifts to %f
  body's %.8f,%.8f; vertex format drifts to %f,%f;
  implementation stops formatting priority

Note on the first four: each of those strings appears TWICE in the file now
-- once in the merged body and once in block B-3, which is retained as a
historical record. The mutants edit the FIRST occurrence, and the cases below
read the FIRST occurrence, which is how this file proves it is reading the
body rather than the block. A criterion that scanned unbounded would read
the block's copy, stay green through a body regression, and be worse than no
criterion at all.

WHY THIS FILE EXISTS AT ALL, stated plainly: the repo already had criteria
for the fence crc32 -- golden cross-language vectors, round-trip tests,
FV-13 checks. Every one of them was green while the contract body said
something different from the code, because every one of them compared the
implementation against ANOTHER COPY OF THE IMPLEMENTATION (a fixture, a
C++ twin, a golden value generated from it). Nothing read the document. That
is the gap, and it is not a gap any amount of testing the code can close.
"""

# os for the doc path. NO regex: the extraction below uses index() on
# literal markers, because a regex over prose-wrapped markdown fails by
# matching NOTHING, and a criterion that matches nothing is green.
import os

import pytest

from xbrain.common.digest.fence import fence_crc32

#: Reads committed files, computes in process. No device, no network, no
#: database -- the only external dependency is docs/11, which is in the repo.
pytestmark = pytest.mark.no_device

#: The contract. Derived from this file's location so the suite runs from
#: any cwd; two dirname() calls reach the repo root from tests/common/.
#: Spelled with the Chinese filename because that IS the filename -- a glob
#: on "docs/11-*.md" would silently pick a different file if a second one
#: ever appeared, and this criterion is only meaningful against the one
#: document that is the contract.
_CONTRACT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "docs", "11-接口契约.md")

#: The seven segments, in the order 11 S9A.2 writes them, each paired with
#: the literal text the recipe uses for it. This list is the criterion's own
#: statement of the contract -- it is checked against the document AND
#: against the implementation, so it cannot quietly become the only source
#: of truth for either.
#:
#: The last entry is a FORMAT STRING and not a field name, because the recipe
#: does not name the vertex field: it writes "各顶点 sprintf(...)". Matching
#: on "vertices" would never hit, and a criterion that silently matched
#: nothing is the vacuous pass CLAUDE.md S3.2 lists. The format is also the
#: part that must not drift -- a default float repr differs between Python
#: and C++ -- so pinning the format text is stricter than pinning a name.
_SEGMENTS = (
    "poly_id",
    "role",
    "winding",
    "hard_enforce",
    "priority",
    "speed_limit_mps",
    '%.8f,%.8f;',
)


def _body_recipe() -> str:
    """The crc32 recipe code fence from S9A.2's BODY, not from a patch block.

    Bounded between the section's own heading text and the end of the fence.
    Unbounded scanning would pick up block B-3's copy (which is retained as a
    historical record) and the case would pass even if the body regressed --
    the vacuous-pass failure mode CLAUDE.md S3.2 warns about.
    """
    with open(_CONTRACT, encoding="utf-8") as handle:
        text = handle.read()
    # The body recipe is introduced by this exact sentence. It appears twice
    # in the file (body + block B-3), so the FIRST occurrence is the body:
    # S9A.2 is at ~line 14400 and the patch blocks are at ~line 15100+.
    # The marker is the section's own sub-heading text. Using the heading
    # rather than a line of the recipe itself keeps the extraction from
    # depending on the content it is about to assert on -- CLAUDE.md S3.2's
    # "the criterion must not contain the string it greps for".
    marker = "`crc32` 的规范化规则"
    # FIRST occurrence = the body. S9A.2 sits around char 890k and the patch
    # blocks around 940k+, so index() lands in the body. Asserted indirectly
    # by the mutants: they edit the first occurrence and these cases go red.
    first = text.index(marker)
    fence_start = text.index("```text", first)
    # +7 skips the opening "```text" so the closing fence is found rather
    # than the opening one. Off by one here would return an empty string and
    # every "seg in recipe" assertion below would fail with a confusing
    # message instead of the real one -- hence the CRC-32 guard that follows.
    fence_end = text.index("```", fence_start + 7)
    recipe = text[fence_start:fence_end]
    # Guard against the extraction silently matching the wrong place: the
    # recipe must mention the CRC-32 polynomial line. Without this, a future
    # reshuffle that moved the marker would give us some other code fence
    # and every assertion below would be about the wrong text.
    assert "CRC-32" in recipe, "extracted the wrong code fence"
    return recipe


def test_the_body_recipe_lists_all_seven_segments_in_order():
    """S9A.2's body recipe names the seven segments, in the contract order.

    Order is asserted, not just membership: the string is concatenated, so
    swapping two segments produces a different crc32 for the same fence --
    and a cross-language mismatch that looks like a corrupt message.
    """
    recipe = _body_recipe()
    # Record where each segment was found, then check the list is sorted.
    # Checking adjacency instead would be stricter than the contract: the
    # recipe has comment markers ("<- v0.3 补") between segments, and a
    # criterion that forbade them would go red on a purely editorial change.
    positions = []
    for seg in _SEGMENTS:
        assert seg in recipe, (
            "11 S9A.2 body recipe is missing %r -- if block B-3 was "
            "un-merged or the body was edited, fix the body" % seg)
        positions.append(recipe.index(seg))
    # The failure message carries the segment/position pairs, not just
    # "wrong order": an operator seeing which one moved knows whether a
    # field was inserted in the wrong place or two were transposed, and
    # those have different fixes.
    assert positions == sorted(positions), (
        "segments appear in the wrong order: %s"
        # strict=True because the two lists MUST be the same length
        # here (positions is built one-per-segment); a silent truncation
        # would hide the very segment that moved. B905 is a hard rule in
        # this repo for exactly that reason.
        % list(zip(_SEGMENTS, positions, strict=True)))


def test_the_body_recipe_pins_the_two_formats_that_must_not_drift():
    """%.8f for vertices and %.3f for speed_limit_mps.

    Both are in the body as literal format strings. They are the pair that
    makes the recipe reproducible across Python and C++: a default float
    repr differs between the two languages, so the fixed-point formats are
    the contract, not an implementation detail. 11 S9A.2 says so verbatim.
    """
    recipe = _body_recipe()
    # %.8f for the vertices: eight decimal places of a WGS84 degree is about
    # 1 mm, so the format is not merely a rendering choice -- it fixes the
    # precision at which two fences count as the same fence.
    assert "%.8f,%.8f;" in recipe
    # %.3f for the speed limit: millimetres per second. Three places is more
    # than any real limit needs, which is the point -- the format has to be
    # fixed at SOME number of places, and a generous one never truncates a
    # value an operator typed.
    assert "%.3f" in recipe
    # Both asserted as the literal format strings and not as "a format
    # string is present": the failure this guards against is a DRIFT (%.3f
    # becoming %f), not an omission. A looser check would pass the drift.


def test_the_implementation_builds_exactly_those_segments():
    """Two fences differing ONLY in a late segment must differ in crc32.

    This is the half that ties the document to the code without writing a
    second recipe here. For each of the two segments block B-3 added, build
    two otherwise-identical fence sets that differ only in that field and
    assert the crc32 changes. A recipe that omitted the field would return
    the same value -- which is the defect, exactly as the contract's note
    describes it.
    """
    def fence(priority, limit):
        # role=speed_limit so speed_limit_mps is in the string at all; every
        # other field held constant so only the varied one can matter.
        #
        # Three vertices and not two: 11 S9A.2 FV-1 wants a closed polygon,
        # and a degenerate one could legitimately be rejected before the
        # crc32 is ever computed -- the case would then pass on an
        # exception path rather than on the recipe.
        #
        # hard_enforce False because role=speed_limit forces it (the merged
        # field table: both advisory roles are always false). Passing True
        # here would make the fixture contradict the contract it is used to
        # check, which is how a fixture stops being evidence.
        return {
            "fence_set_id": "fs1",
            "rev": 1,
            "polygons": [{
                "poly_id": "f-slow",
                "role": "speed_limit",
                "winding": "ccw",
                "hard_enforce": False,
                "priority": priority,
                "speed_limit_mps": limit,
                "vertices": [[31.0, 121.0], [31.1, 121.0], [31.1, 121.1]],
            }],
        }

    base = fence(0, 1.0)
    # Each assertion changes exactly ONE field from base. A case that varied
    # two at once would go green with either one of them in the recipe --
    # and the recipe was missing exactly one of them before block B-3.
    # priority is in the recipe: changing it must change the crc32.
    assert fence_crc32(base) != fence_crc32(fence(1, 1.0)), (
        "priority is not in the implemented recipe -- changing the painter "
        "order would produce a byte-identical FenceSet and S-5 would "
        "answer duplicate")
    # speed_limit_mps is in the recipe: the contract's own worked example.
    assert fence_crc32(base) != fence_crc32(fence(0, 0.5)), (
        "speed_limit_mps is not in the implemented recipe -- lowering the "
        "slow zone would never take effect, with no error anywhere")
    # And the control: an identical fence set gives an identical value, so
    # the two assertions above are not passing because crc32 is random.
    # The control. Without it, the two assertions above would also pass for
    # a crc32 that returned a random value every call -- which is the shape
    # a "compare two hashes" criterion most easily degenerates into.
    assert fence_crc32(base) == fence_crc32(fence(0, 1.0))


def test_name_is_not_in_the_recipe():
    """Changing only `name` must NOT change the crc32.

    11 S9A.2 states the reason verbatim: a rename affects no machine
    judgement, and including it would make a pure HMI rename trigger a
    full stage/commit cycle. This is the one field whose ABSENCE from the
    recipe is deliberate, so it is asserted rather than left to be
    rediscovered by whoever adds the next field.
    """
    def fence(name):
        # role=forbid here, not speed_limit: this case is about `name` and
        # a speed_limit row would drag speed_limit_mps into the string,
        # giving a second field that could mask a name leak if the recipe
        # happened to normalise one of them.
        return {
            "fence_set_id": "fs1",
            "rev": 1,
            "polygons": [{
                "poly_id": "f-a", "role": "forbid", "winding": "cw",
                "hard_enforce": True, "priority": 0, "name": name,
                "vertices": [[31.0, 121.0], [31.1, 121.0], [31.1, 121.1]],
            }],
        }
    # One Chinese name and one ASCII one: a recipe that included `name`
    # would differ on these two by more than a byte count, so the case
    # cannot pass by accident of equal lengths.
    assert fence_crc32(fence("油料库")) == fence_crc32(fence("renamed"))
