"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: refuse_to_boot.py
Brief: CFG-CF-9 milestone -- empty configs => refuse to boot with key-paths listed

Description:
The 10 S5.4.5 verbatim rule: configs/ null placeholders MUST make
freeze refuse to boot, listing:

  * missing file absolute paths (assertion J)
  * unassigned key paths          (assertion A)
  * layers where a required key is missing (assertion M)

WHERE THIS IS WIRED, and what it replaced
  xbrain/boot/freeze/__main__.py catches the XbrainError an assertion
  runner raises out of run_freeze and renders it through
  verdict_from_error() below. Before that wiring existed the module had
  NO caller at all, and the observable behaviour of `python -m
  xbrain.boot.freeze` on the current tree was a raw Python traceback on
  stderr ending in

    xbrain.common.errors.exceptions.XbrainError: E_CONFIG_INVALID:
    assertion A failed at key 'common.calib.calib_rev': null_unassigned

  -- exit code 1, so CFG-CF-9 criterion (1) was met by accident, while
  criterion (2) "stdout lists the key paths" was met by nothing. A
  traceback is not the listing: it names the raise site in this
  repository rather than the file an operator has to edit, and the
  p5_gateway minimal-mode observation window (which renders the failing
  assertion letter + key paths, see
  xbrain/p5_gateway/minimal/observation_window.py) had no structured
  input to render at all.

WHAT THIS MODULE IS NOT
  It does not decide whether the freeze fails -- the assertion runners
  in xbrain/boot/freeze/assertions/ do that, and they are the ONLY
  place a config verdict is reached. This module turns one already
  raised failure into operator-visible text. Putting any judgement here
  would create a second opinion about whether a config is startable.

  It also does not fill, default, or soften anything. CLAUDE.md iron
  rule 3: an uncalibrated key stays null and the whole stack refuses to
  start; that refusal is the designed behaviour, and this module exists
  to make it READABLE, never to get past it.

A TRAP WORTH NAMING
  Bucketing by detail.kind means a kind this module does not know about
  would compose to an EMPTY listing -- a refusal that prints nothing,
  which is strictly worse than the traceback it replaced. So the
  fallback branch in verdict_from_error is not defensive padding: it is
  the half that keeps an unmapped assertion visible. Do not delete it
  because "every kind is covered" -- assertions B/C/D/E/F/G/H/I/K/L/N/O/S
  all raise kinds that are deliberately NOT in the three CFG-CF-9
  buckets.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from xbrain.common.errors.exceptions import XbrainError

# detail.kind values assertion J raises (xbrain/boot/freeze/assertions/
# j_config_root.py _fail sites). All of them are about a FILE, so all of
# them land in the missing-files section and carry detail.path.
_J_KINDS = frozenset({
    "config_root_missing",
    "config_file_missing",
    "config_file_obsolete",
    "config_path_escape",
    "config_perm_bad",
})

# detail.kind values assertion A raises (a_references.py _fail). These are
# about a KEY whose value is absent or unresolvable, so they are the
# "unassigned key path" section.
_A_KINDS = frozenset({
    "null_unassigned",
    "unresolved_ref",
    "layer_load_failed",
})

# detail.kind values assertion M raises (m_required.py _fail). A required
# key that no layer supplies; the row names key AND layer, which is why
# it is its own section rather than folded into A's.
_M_KINDS = frozenset({
    "required_key_missing",
    "required_key_only_l0",
})


@dataclass(frozen=True)
class FreezeVerdict:
    exit_code: int
    stdout_lines: List[str]


def compose_stdout_lines(
        missing_files: List[str],
        unassigned_keys: List[str],
        missing_layer_keys: List[str]) -> List[str]:
    """Assemble the operator-visible failure listing. Every listed
    item MUST include enough info to fix it: file paths are
    absolute; key paths are dotted; layer-missing rows name both
    key AND layer.

    The listing is what the HMI's observation window renders."""
    lines: List[str] = []
    if missing_files:
        lines.append("assertion J: config files missing")
        for f in sorted(missing_files):
            lines.append(f"  missing_file: {f}")
    if unassigned_keys:
        lines.append("assertion A: keys unassigned (null placeholder)")
        for k in sorted(unassigned_keys):
            lines.append(f"  unassigned_key: {k}")
    if missing_layer_keys:
        lines.append("assertion M: keys missing from required layer")
        for k in sorted(missing_layer_keys):
            lines.append(f"  missing_layer_key: {k}")
    return lines


def verdict(missing_files: List[str],
            unassigned_keys: List[str],
            missing_layer_keys: List[str]) -> FreezeVerdict:
    """Produce a FreezeVerdict. Exit code:
       0 -> nothing failed
       1 -> at least one assertion failed
    We deliberately reject 'partial success': ANY failure aborts."""
    lines = compose_stdout_lines(
        missing_files, unassigned_keys, missing_layer_keys)
    if lines:
        return FreezeVerdict(exit_code=1, stdout_lines=lines)
    return FreezeVerdict(exit_code=0, stdout_lines=[])


def _layer_suffix(detail: dict) -> str:
    """Render the ' (layer: Lx)' tail for an M row, or empty.

    Kept separate so the caller reads as a bucket dispatch rather than
    as string assembly, and so a detail dict WITHOUT a layer field still
    produces a usable row instead of a KeyError at the exact moment the
    operator needs the message.
    """
    layer = detail.get("layer")
    return "" if layer is None else " (layer: %s)" % layer


def verdict_from_error(exc: XbrainError) -> FreezeVerdict:
    """Turn ONE assertion failure into the CFG-CF-9 listing.

    The freeze chain is fail-fast by construction -- every runner in
    assertions/ raises at its first violation rather than collecting --
    so this receives one failure per run, and the listing has one row.
    That is the honest shape: the operator fixes that key, re-runs, and
    gets the next one. A collector here would have to re-implement every
    runner's walk to find the rest, and the second implementation would
    be the one that drifts.

    An exception whose detail.kind is not one of the three CFG-CF-9
    buckets is NOT dropped: it gets an 'assertion (unclassified)' block
    carrying the code, the message and the raw detail. Printing an empty
    listing for it would be a refusal that says nothing at all.
    """
    detail = exc.detail or {}
    kind = detail.get("kind")
    if kind in _J_KINDS:
        return verdict([str(detail.get("path", exc))], [], [])
    if kind in _A_KINDS:
        return verdict([], [str(detail.get("key", exc))], [])
    if kind in _M_KINDS:
        row = "%s%s" % (detail.get("key", exc), _layer_suffix(detail))
        return verdict([], [], [row])
    # Unclassified: still a refusal, still exit 1, and the operator still
    # gets the code plus whatever structure the raiser attached.
    lines = [
        "assertion (unclassified): %s" % exc.code,
        "  message: %s" % (exc.args[0] if exc.args else ""),
    ]
    for name in sorted(detail):
        lines.append("  %s: %s" % (name, detail[name]))
    return FreezeVerdict(exit_code=1, stdout_lines=lines)
