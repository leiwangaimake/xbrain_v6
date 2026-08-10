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

Any 'default-values-in-code as fallback' pattern falsifies this
rule and is refused by the CLAUDE.md 3.1 lint (no_safety_default.py).

Even in the refuse-to-boot state, p5_gateway MINIMAL MODE still
starts (see xbrain/p5_gateway/minimal/observation_window.py) so
the HMI can display the failing assertion letter + key paths.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


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


class DefaultFallbackForbidden(Exception):
    """CFG-CF-9 variant 1 guard: a fallback that uses code
    defaults 'as a backup' when a key is missing is refused."""


def refuse_code_default(key_path: str) -> None:
    """Called from any freeze-time code path that would otherwise
    reach for a hardcoded fallback."""
    raise DefaultFallbackForbidden(
        f"key {key_path!r} unassigned; refusing to fall back to a "
        f"code default (CLAUDE.md 3.1, CFG-CF-9)")


def safety_zero_still_fails_g(key_path: str, value) -> None:
    """CFG-CF-9 variant 2 guard: even if a safety parameter is
    filled with 0.0 (passing assertion A), assertion G still
    reddens on the SP-5 rule. Filling with 0.0 -- classic
    fail-silent -- must not bypass the entire freeze chain."""
    if key_path.startswith("common.safety.") and value == 0.0:
        raise DefaultFallbackForbidden(
            f"safety key {key_path!r} = 0.0 refused; SP-5 requires "
            f"positive value (CLAUDE.md 3.1 zero-mask guard)")
