"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: correction_log.py
Brief: GWY-P4-04 -- asr_correction table + accepted flag semantics

Description:
16 S3.5 asr_correction table -- persistent log of every L1/L2/L3
edit for later dict tuning and false-positive study.

Schema:
  asr_correction(id, ts, raw, corrected, layer, score,
                  session_id, accepted)

  layer: which of {L1, L2, L3} produced the correction
  accepted: True initially; user can flip to False (rejected as
            wrong correction). accepted=False rows form the negative
            sample set for L1 dict curation.

* CLAUDE.md 4.1: persistence layer MUST use aiosqlite (not sqlite3).
This module offers a SYNC facade for tests + writes; the
production adapter (P3 owns the DB actually) uses aiosqlite.
The actual DB opens live in xbrain/persistence/ per DAO discipline.

* CLAUDE.md 3.4: ts is for DISPLAY only. Use monotonic clock for
any age / timeout logic; never derive it from ts. Documented here
because a naive `age_s = now - row.ts` would be a wall-clock
violation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class CorrectionLayer(str, Enum):
    L1 = "L1"
    L2 = "L2"
    L3 = "L3"


@dataclass(frozen=True)
class CorrectionRow:
    """One asr_correction row."""
    ts_iso: str            # display only (CLAUDE.md 3.4)
    raw: str
    corrected: str
    layer: str             # CorrectionLayer value
    score: float
    session_id: str
    accepted: bool = True


def build_row(ts_iso: str, raw: str, corrected: str,
              layer: CorrectionLayer, score: float,
              session_id: str) -> CorrectionRow:
    return CorrectionRow(
        ts_iso=ts_iso, raw=raw, corrected=corrected,
        layer=layer.value, score=score,
        session_id=session_id, accepted=True,
    )


def reject_row(row: CorrectionRow) -> CorrectionRow:
    """User rejected the correction as wrong. Returns a new row
    with accepted=False (rows are frozen)."""
    return CorrectionRow(
        ts_iso=row.ts_iso, raw=row.raw, corrected=row.corrected,
        layer=row.layer, score=row.score,
        session_id=row.session_id, accepted=False,
    )


def negative_samples(rows: List[CorrectionRow]) -> List[CorrectionRow]:
    """The corpus of user-rejected corrections. Fed to L1 dict
    curation as anti-entries (do NOT add these to L1)."""
    return [r for r in rows if not r.accepted]
