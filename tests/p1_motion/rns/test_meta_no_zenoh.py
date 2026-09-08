"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_meta_no_zenoh.py
Brief: RNS-M-4/M-5 meta -- module reads no Zenoh session (P2.7 -- 20 S1.1)

Description:
RNS-M-4/M-5 (20 S1.1): the module consumes the tick SNAPSHOT and never reads a
Zenoh session, opens no subscriber, spawns no thread, does no async I/O. This
meta-test scans every rns/ file (excluding _legacy) for those imports/calls. The
mutant it catches: any rns module that imports zenoh or declare_subscriber ->
reddens. It is the machine form of RNS-M-4/M-5 (avoids the strong-ref trap of
CLAUDE.md 4.3 by structurally never holding a session).
"""

from __future__ import annotations

import re
from pathlib import Path

RNS = Path(__file__).resolve().parents[3] / "xbrain" / "p1_motion" / "rns"

# forbidden in the RNS module body (RNS-M-4/M-5): a Zenoh session, a subscriber,
# a thread, or async. The snapshot is handed IN; RNS never fetches it.
_FORBIDDEN = re.compile(
    r"\b(import\s+zenoh|from\s+zenoh|declare_subscriber|declare_publisher|"
    r"zenoh\.open|threading\.Thread|asyncio|async\s+def|await\s)")


def test_no_rns_module_reads_zenoh_or_threads():
    offenders = {}
    for f in RNS.glob("*.py"):
        if f.name == "__init__.py":
            continue
        hits = _FORBIDDEN.findall(f.read_text(encoding="utf-8"))
        if hits:
            offenders[f.name] = hits
    assert not offenders, (
        "RNS module files hold a Zenoh session / thread / async (RNS-M-4/M-5 "
        "forbids -- input is the tick snapshot, RNS never reads a session): %s"
        % offenders)
