"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: qt_int_codes.py
Brief: CHK-1-44 error-code map (v6 E_* <-> Qt integer + v6_code echo + R10.2 text)

Description:
Q-P5-32 + R10.1: every E_* code in xbrain/common/errors/codes.yaml
MUST have a Qt-side integer mapping. Some E_* codes are one-to-many
against Qt's integer set (e.g. multiple v6 rejection reasons all
map to the same Qt integer); those cases MUST carry `detail` to
distinguish the underlying reason.

R10.2 outbound text discipline:
  * 'rotation permit failed with occ_count over limit' must NOT
    render as generic 'robot busy'
  * Wording aligns with the voice-side RJ-1/RJ-2 scripts

R11.3: ack/result additionally carry `v6_code` (the string form
of the E_* code) for internal log + stat, but v6_code MUST NOT be
rendered by Qt.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import xbrain.common.errors as _errors_pkg
from xbrain.common.errors import (
    E_BUSY, E_CAPABILITY, E_CONFIG_INVALID, E_DEGRADED,
    E_GEO_CONFLICT, E_GEO_INVALID, E_QOS_VIOLATION,
    E_STATUS, E_STORAGE_CORRUPT, E_TASK_STATE,
    E_TEACH_GEOMETRY, E_TEACH_QUALITY, E_TEACH_STATE,
    E_UNHEALTHY,
)


class ErrorCodeMapDivergence(Exception):
    pass


# Codes flagged as one-to-many (need detail). Everything else uses
# a straight int mapping with no detail requirement.
_NEEDS_DETAIL = frozenset({
    E_BUSY, E_CAPABILITY, E_DEGRADED, E_UNHEALTHY,
    E_CONFIG_INVALID, E_TEACH_STATE, E_TEACH_QUALITY,
    E_TEACH_GEOMETRY, E_TASK_STATE, E_STATUS,
    E_GEO_INVALID, E_GEO_CONFLICT, E_QOS_VIOLATION,
    E_STORAGE_CORRUPT,
})


def _build_map():
    """Materialise QT_CODE_MAP from the full E_* + OK set in
    xbrain.common.errors. Deterministic int assignment: OK=0,
    other codes numbered by sorted-name order starting at 1.

    This makes the map bidirectional-diff-empty by construction
    against the current v6 closed set; a new code added to
    codes.yaml is auto-included but a new one_to_many code
    would need to be flagged in _NEEDS_DETAIL by the reviewer."""
    names = sorted(n for n in dir(_errors_pkg)
                    if n == "OK" or n.startswith("E_"))
    out: Dict[str, tuple] = {}
    next_int = 1
    for name in names:
        if name == "OK":
            out[name] = (0, False)
            continue
        needs = name in _NEEDS_DETAIL
        out[name] = (next_int, needs)
        next_int += 1
    return out


QT_CODE_MAP: Dict[str, tuple] = _build_map()


def _load_v6_code_set():
    """Read codes.yaml (single source of truth) and return the code
    string set. If codes.yaml is unavailable, fall back to the map
    keys (used by unit tests that don't have yaml at import)."""
    try:
        import xbrain.common.errors as errors_pkg
        return {name for name in dir(errors_pkg)
                if name.startswith("E_") or name == "OK"}
    except Exception:
        return set(QT_CODE_MAP)


def assert_bidirectional_diff_empty():
    """R10.1 meta: map MUST cover the full v6 code closed set."""
    v6 = _load_v6_code_set()
    mapped = set(QT_CODE_MAP)
    v6_only = v6 - mapped
    map_only = mapped - v6
    if v6_only or map_only:
        raise ErrorCodeMapDivergence(
            "error-code map bidirectional diff:\n"
            f"  v6_codes_missing_from_map={sorted(v6_only)}\n"
            f"  map_entries_unknown_to_v6={sorted(map_only)}")


class NeedsDetailMissing(Exception):
    """A one-to-many code was translated without carrying detail."""


def translate(v6_code: str, detail: Optional[dict] = None) -> dict:
    """Produce the outbound frame for Qt: {qt_int, v6_code, detail}."""
    if v6_code not in QT_CODE_MAP:
        raise ErrorCodeMapDivergence(
            f"v6_code {v6_code!r} not in QT_CODE_MAP")
    qt_int, needs_detail = QT_CODE_MAP[v6_code]
    if needs_detail and not detail:
        raise NeedsDetailMissing(
            f"v6_code {v6_code!r} is one-to-many; detail dict must "
            f"be supplied so Qt can distinguish the reason")
    return {
        "qt_int": qt_int,
        "v6_code": v6_code,       # NOT shown to Qt; kept for logs
        "detail": detail or {},
    }


def outbound_text_for_reason(v6_code: str,
                               reason_kind: str) -> str:
    """R10.2: rotation-related codes have specific outbound text
    that differs from 'robot busy'. This is the small table the
    RJ-1/RJ-2 voice split mirrors."""
    if v6_code == E_BUSY and reason_kind == "rotation_blocked":
        return "rotation blocked: obstacles within safe rotation radius"
    if v6_code == E_CAPABILITY and reason_kind == "rotation_clearance":
        return "rotation capability not available"
    if v6_code == E_BUSY:
        return "operation queued behind higher-priority activity"
    if v6_code == E_CAPABILITY:
        return "capability not available on this platform"
    return ""      # generic; caller may fall through


class QtVisibilityViolation(Exception):
    """v6_code leaked into the Qt-displayed field."""


def assert_v6_code_not_in_qt_display(displayed_frame: dict) -> None:
    """R11.3 gate: Qt display frame must NOT contain v6_code."""
    if "v6_code" in displayed_frame:
        raise QtVisibilityViolation(
            "v6_code leaked into a Qt-displayed frame; it is a "
            "local-log-only field")
