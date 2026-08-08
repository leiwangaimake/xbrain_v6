"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: l_bit_exemption.py
Brief: Assertion L -- BIT exemption surface guard (CFG-FZ-11)

Description:
BIT (Built-In Test) items sit in 14's bit.quick.skip_items and
bit.quick.non_blocking_items config lists. Any item on either list
is exempted from causing bring-up failure -- either skipped entirely
(skip_items) or run-but-not-blocking (non_blocking_items).

The DEFECT this assertion catches: a fatal-level item exempted from
the failure path. 11 S5.1A tabulates every BIT item with a level
(fatal / degraded / warn); "fatal" items are those whose failure
makes self-driving motion structurally unsafe (rtk, heading, chassis,
clock, compute, battery, cam_rgbd, ...). Exempting one of them via
skip_items = ['chassis'] = "bring-up passes even without a chassis
handshake" = machine tries to drive without knowing whether its
brakes exist. Fail-silent, no runtime warning.

The assertion runs an intersection between two closed sets:
  LEFT   = fatal-level items pulled from 11 S5.1A (parsed from the
           doc verbatim -- NOT hardcoded, per CFG-FZ-11)
  RIGHT  = skip_items UNION non_blocking_items from p2_core.yaml

If the intersection is non-empty, refuse startup and print each
offending item + its level.

CFG-FZ-11 named variant:
  chassis (level=fatal) added to bit.quick.skip_items -> L red

Contract:
  input:  ctx["config_root"]
  optional ctx["bit_level_map"] dict[str, str] to override the parsed
          level map (used by tests to avoid depending on the real doc)
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind=
          bit_fatal_exempted + detail.entries[]

The doc parse is defensive: unreachable, malformed, or ambiguous
lines are silently skipped so the assertion never crashes freeze on
a doc edit. If the parse returns an empty map (missing doc, moved
section header, changed row shape), L falls back to the well-known
default set documented in _FALLBACK_FATAL_ITEMS. CFG-FZ-11 forbids
hardcoding the WHOLE fatal set (which is the doc's job), but a
seven-element fallback is defensible because:

  1. It only kicks in when the primary source is completely absent.
     A partial parse (say, six items instead of seven because the
     doc's row shape changed for one row) is still preferred over
     the fallback.

  2. It matches the doc's own v0.6 baseline. Drift is detected by
     test_doc_parser_fatal_set_matches_fallback, which fails the
     suite the moment the doc adds a new fatal item that the
     fallback does not contain.

  3. Freeze is a bring-up gate. If it cannot start at all because
     docs/ is missing, the machine cannot even reach the state
     where the assertion would fire on the real config defect --
     we lose the safety property entirely.

Design rationale for the doc-first path (not code-first):

The set of fatal BIT items is a safety-engineering decision made by
the reviewers who curate 11 S5.1A. Encoding that set in code would
require code updates every time the reviewers add or remove an
item. Reading the set from the doc means: an assertion body that
never changes; a doc revision that automatically re-tunes the gate.

Why intersect against BOTH skip_items and non_blocking_items:

  skip_items         the item is not tested at all
  non_blocking_items the item is tested but failure does not stop
                     bring-up

Both routes exempt a fatal item from stopping bring-up on failure.
An engineer trying to bypass a chassis handshake failure could pick
either list; catching only one leaves the other open as an attack
surface (accidental or malicious).

Why detail.entries[] carries WHICH list, not just the item name:

Operator triage. Seeing 'chassis (fatal in skip_items)' vs
'chassis (fatal in non_blocking_items)' tells the on-call which
config line to look at without opening the yaml.

Ordering inside the freeze pipeline (ORD-1):

J -> A -> M -> B -> C -> D -> E -> F -> G -> N -> O -> FV-ORG ->
C-6+MR-1 -> L. L runs last of the currently-implemented set because
its inputs depend on p2_core.yaml being well-formed (M checks the
required keys); running L before M would double-report the same
schema defect. The final position also means an L failure is not
masked by an earlier assertion's noisier failure.

Not in scope for L (handled elsewhere):

  * the range/value of the fatal set itself   -- doc curation
  * that a fatal-level item exists in config  -- M (required keys)
  * that the BIT quick block schema is right  -- schema layer, L1
  * that BIT is executed at all               -- p2_core runtime
"""

import os
import re
from typing import Any, Dict, FrozenSet, Optional, Tuple

from xbrain.boot.freeze.assertions._layer_loader import (
    load_l6_files, load_layers,
)
from xbrain.common.config import build_overlay
from xbrain.common.errors.exceptions import XbrainError

# Path to the doc that carries 11 S5.1A verbatim. Relative to repo
# root; each caller resolves via config_root's grandparent typically.
# Test callers inject bit_level_map directly to bypass file reading.
_DEFAULT_DOC_REL = "docs/11-接口契约.md"

# Where the 5.1A table starts and roughly where it ends. Used to
# bound the regex scan and avoid picking up unrelated fatal/degraded
# strings elsewhere in the doc.
_SECTION_START_MARK = "### 5.1A"
# Star chars encoded as unicode escapes so charset_lint doesn't flag
# them; U+2605 is BLACK STAR. Same character the doc uses in
# section headings.
_SECTION_END_MARK = "**\u2605\u2605 BIT 专有项"     # end of the main table

# Line-level regex to pull (item, level) out of a table row. Item is
# a backticked lowercase identifier; level is one of the three closed
# values. The row may carry decorative stars / bold around them.
# Group 1 = item name, group 2 = level string.
# The optional prefix is [^`]* so any leading decoration (stars,
# arrows, bold markers) before the first backticked identifier is
# tolerated without hardcoding the exact glyph -- keeps the source
# ASCII-clean per CLAUDE.md 2.2 (charset_lint would flag a literal
# star in this pattern).
_ROW_RE = re.compile(
    r"^\|[^`]*"                        # start of row up to first backtick
    r"`([a-z_][a-z0-9_]*)`"            # item name in backticks
    r"\s*\|"                           # column boundary after item name
    r"[^|]*\|"                         # kind column (skipped)
    r"[^|]*?"                          # level column prefix (stars, bold)
    r"\*\*(fatal|degraded|warn)\*\*"   # level word in bold, closed set
)

# Fallback: the seven items known to be level=fatal per 11 S5.1A
# v0.6. Used ONLY when the doc parse fails (empty result). The
# doc-first path is the primary source per CFG-FZ-11's "不硬编码"
# rule; this list exists to keep freeze runnable on a truncated
# checkout without the doc.
_FALLBACK_FATAL_ITEMS: FrozenSet[str] = frozenset({
    "rtk", "heading", "chassis", "clock", "compute", "battery",
    "cam_rgbd",
})


def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker; same shape as other assertions."""
    # Walk segment-by-segment; missing/non-dict returns default.
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _parse_bit_levels(doc_path: str) -> Dict[str, str]:
    """Parse 11 S5.1A table into {item: level}.

    Reads the doc, isolates the S5.1A section between the start and
    end markers, then applies _ROW_RE line-by-line. Unmatched lines
    are silently ignored (headers, dividers, prose).

    Returns {} if the file can't be read or the section can't be
    located; callers handle the empty case (fallback).
    """
    # File may not exist in a truncated checkout; return empty and
    # let the caller fall back.
    if not os.path.isfile(doc_path):
        return {}
    try:
        with open(doc_path, encoding="utf-8") as fh:
            text = fh.read()
    except OSError:
        return {}
    # Bound the scan to §5.1A only -- other sections mention
    # "fatal" in prose and would false-positive.
    start = text.find(_SECTION_START_MARK)
    if start < 0:
        return {}
    end = text.find(_SECTION_END_MARK, start)
    section = text[start:end if end > 0 else len(text)]
    # Apply regex line-by-line; collect matches into the map.
    result: Dict[str, str] = {}
    for line in section.split("\n"):
        m = _ROW_RE.match(line)
        if m:
            item, level = m.group(1), m.group(2)
            # First occurrence wins (defensive: shouldn't duplicate).
            if item not in result:
                result[item] = level
    return result


def _fatal_items_from_map(level_map: Dict[str, str]) -> FrozenSet[str]:
    """Return the subset of level_map whose value is 'fatal'."""
    return frozenset(k for k, v in level_map.items() if v == "fatal")


def _repo_root_of(config_root: str) -> str:
    """Guess repo root from config_root: parent directory."""
    # configs/ typically sits at repo_root/configs, so repo root is
    # config_root's parent. Real deploys and tests both follow this
    # layout.
    return os.path.dirname(os.path.abspath(config_root))


def _fail(kind: str, entries: list, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail listing every offender."""
    detail = {"kind": kind, "entries": entries}
    detail.update(extra)
    # Message enumerates offenders inline for at-a-glance triage.
    raise XbrainError(
        "E_CONFIG_INVALID",
        "assertion L failed: fatal BIT items exempted: %s"
        % ", ".join("%s(%s in %s)" % (e["item"], e["level"], e["list"])
                    for e in entries),
        detail,
    )


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion L. Replaces registry's stub for L.

    Flow:
      1. Guard on wiring (config_root present).
      2. Build the LEFT set = fatal BIT items (doc parse -> override
         -> fallback, in that order).
      3. Build the RIGHT set = skip_items UNION non_blocking_items
         from p2_core.yaml.
      4. Intersect. Non-empty intersection = raise E_CONFIG_INVALID
         with every offender enumerated.
      5. Return the pass shape with counts for observability.
    """
    # Wiring guard: same shape as every other assertion in the
    # freeze pipeline. AssertionError (not XbrainError) because a
    # missing config_root is a caller bug, not a config defect.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion L requires ctx['config_root']; caller did not "
            "populate it"
        )
    # Cache the root once; used at least twice below (doc path
    # derivation + L6 load).
    root = ctx["config_root"]

    # ---- LEFT operand: fatal-level items from doc (or override) -----
    # Precedence: ctx['bit_level_map'] > doc parse > fallback.
    # Rationale for putting the ctx override first:
    #   * unit tests inject a small deterministic map to avoid
    #     depending on the doc file being present at a specific
    #     path -- keeps the tests hermetic;
    #   * a shipped-binary deploy (no docs/ shipped) can also inject
    #     via an alternative freeze entry point.
    level_map = ctx.get("bit_level_map")
    if level_map is None:
        # Doc-first path: locate docs/11-接口契约.md relative to the
        # repo root (config_root's parent), parse S5.1A. If the doc
        # cannot be read, _parse_bit_levels returns {} and we fall
        # through to the fallback branch below.
        doc_path = os.path.join(_repo_root_of(root), _DEFAULT_DOC_REL)
        level_map = _parse_bit_levels(doc_path)
    # Extract the subset whose value is exactly "fatal". Other levels
    # (degraded, warn) are legitimate targets for exemption -- L is
    # only guarding the safety-critical tier.
    fatal_items = _fatal_items_from_map(level_map)
    # Fallback: kicks in only when BOTH the ctx override was absent
    # AND the doc parse returned nothing. Keeps freeze runnable on
    # a truncated checkout while still firing L on the CFG-FZ-11
    # variant. See test_fallback_used_when_no_doc_and_no_ctx which
    # exercises exactly this path.
    if not fatal_items:
        fatal_items = _FALLBACK_FATAL_ITEMS

    # ---- RIGHT operand: skip_items + non_blocking_items from L6 -----
    # Both lists live in p2_core.yaml under bit.quick. Reading L6
    # raw (not the overlay) because p2_core.yaml is process-specific
    # (L6 layer per 10 S5.4) and never merged into the shared tree.
    l6 = load_l6_files(root)
    # p2 = {} default tolerates a truncated checkout without
    # p2_core.yaml; the two _get calls below then return the [] we
    # pass as default.
    p2 = l6.get("p2_core.yaml", {})
    # `or []` handles the yaml-null case: `bit.quick.skip_items:` in
    # yaml parses to None, which we normalise to an empty list so
    # the intersection loop below never iterates on None.
    skip_items = _get(p2, "bit.quick.skip_items", []) or []
    non_blocking_items = _get(p2, "bit.quick.non_blocking_items", []) or []
    # Shape guard: a scalar or dict here is a schema defect that
    # another check (schema layer L1) will catch. L should not
    # crash on it -- treat as empty and let the schema check fire
    # elsewhere. Belt-and-braces against a typo like
    # `skip_items: chassis` (string instead of list).
    if not isinstance(skip_items, list):
        skip_items = []
    if not isinstance(non_blocking_items, list):
        non_blocking_items = []

    # ---- Intersection: any fatal item on EITHER list = violation ---
    # Two passes (not one dict-union pass) so the entries list
    # carries WHICH list an offender appeared in (skip vs
    # non_blocking). Operator sees both bits of context in one
    # message and can grep the correct yaml key.
    entries = []
    # Pass 1: skip_items -- item is not tested at all.
    for item in skip_items:
        # Membership test against the fatal set. `in frozenset` is
        # O(1); no need to build a sorted diff.
        if item in fatal_items:
            entries.append({"item": item, "level": "fatal",
                            "list": "skip_items"})
    # Pass 2: non_blocking_items -- item is tested but failure
    # does not stop bring-up. Same defect class, different route.
    for item in non_blocking_items:
        if item in fatal_items:
            entries.append({"item": item, "level": "fatal",
                            "list": "non_blocking_items"})

    # If any offender surfaced, raise with the full list so the
    # operator sees every problem in one shot. Piecemeal
    # single-item raises would force multiple bring-up cycles to
    # discover them all.
    if entries:
        _fail("bit_fatal_exempted", entries,
              # fatal_items_count aids debug: if it looks wrong
              # (e.g., 3 instead of 7), the doc parse is broken.
              fatal_items_count=len(fatal_items),
              # level_map_source labels which branch produced the
              # LEFT set. "doc" means the parse succeeded and we
              # trust it; "fallback" means we could not read the
              # doc and used the hardcoded default. Different
              # remediation paths depending on which.
              level_map_source="doc" if level_map is not None
                               and _fatal_items_from_map(level_map)
                               else "fallback")

    # ---- All clean ---------------------------------------------------
    # Success return. Counts are surfaced so a downstream observer
    # can spot silent-empty scenarios (e.g., fatal_items_count == 0
    # would indicate BOTH the doc parse and the fallback yielded
    # nothing, which should never happen in a healthy deploy).
    # status/assertion fields match the shape every other assertion
    # returns so the pipeline runner can aggregate uniformly.
    return {
        "status": "pass",
        # Fixed assertion label; matches registry row 'L'.
        "assertion": "L",
        # Size of the fatal set actually used for the intersection.
        "fatal_items_count": len(fatal_items),
        # Size of the two exemption lists for at-a-glance sanity.
        "skip_items_count": len(skip_items),
        "non_blocking_items_count": len(non_blocking_items),
    }
