"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cmdset_extractor.py
Brief: GWY-P4-09 -- cmdset_18.json extractor from 18 S13.1 (machine, not hand)

Description:
16 requires cmdset_18.json (the 128-intent closed set) to be
MACHINE-EXTRACTED from 18 S13.1 rather than hand-transcribed --
hand-transcription would drift silently as 18 revises.

This module implements the extractor:
  * scan 18-*.md for lines matching intent rows in S13.1
  * parse (id, intent_name, route, auth) from each row
  * write cmdset_18.json to configs/generated/

The output is CONSUMED by intents.yaml loading (CS-A1/CS-A2 gate).
"""

from __future__ import annotations

import re
from typing import Dict, List


# 18 S13.1 row pattern. Rows look like:
#   | A05 | move_forward | fastpath | L1a |
#   | E01 | ptz_move    | fastpath | L1a |
# Robust to whitespace variation; commit to strict shape (no free text).
_ROW_RE = re.compile(
    r"^\|\s*"
    r"([A-Z][0-9]{2})\s*"                # id, e.g. A05
    r"\|\s*"
    r"([a-z][a-z0-9_]+)\s*"              # intent name
    r"\|\s*"
    r"(fastpath|llm|bypass|fastpath_then_llm)\s*"    # route
    r"\|\s*"
    r"(L0|L1a|L1b|L2|L3)\s*"             # auth
    r"\|"
)


def extract_rows(md_text: str) -> List[dict]:
    """Scan md text; return list of {id, intent, route, auth} dicts."""
    out: List[dict] = []
    for line in md_text.splitlines():
        m = _ROW_RE.match(line)
        if not m:
            continue
        out.append({
            "id": m.group(1),
            "intent": m.group(2),
            "route": m.group(3),
            "auth": m.group(4),
        })
    return out


def build_cmdset_json(md_text: str) -> Dict[str, list]:
    """Return the dict to serialize as cmdset_18.json.

    Shape:
      {
        "version": 1,
        "intents": [
          {"id": "A05", "intent": "move_forward", "route": "fastpath",
           "auth": "L1a"},
          ...
        ]
      }
    """
    return {
        "version": 1,
        "intents": extract_rows(md_text),
    }
