"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: session_ctx.py
Brief: Zenoh session lifecycle context helpers (open + graceful close)

Description:
Wraps xbrain.common.zenoh.session_factory.build_session_config into a
context manager that opens the plane session, guarantees close() on
scope exit, and lets a caller use `with open_planes(('gen',)) as sess:`
or `with open_planes(('gen', 'rt')) as (gen, rt):`.

Why a helper instead of raw zenoh.open() at every __main__:
  * Every P-process boots the same way -- open one or two sessions,
    register subs/pubs, spin an asyncio loop, on shutdown call close.
  * Repeating the try/finally + double-plane handling in five main.py
    files invites at least one to skip the close on error and leak a
    router-side session -- exactly the RT-C3.a defect (multi-session
    reuse) the session_factory doc warns against.

This helper is DELIBERATELY thin: no auto-retry, no reconnection.
Zenoh's own reliability semantics cover packet loss; a router that
went away is a systemd-level failure the caller must decide about.
"""

from __future__ import annotations

import contextlib
from typing import Iterable, List, Tuple


@contextlib.contextmanager
def open_planes(planes: Iterable[str]):
    """Open one or more Zenoh sessions. Yields either a single
    session (planes == ('gen',) or ('rt',)) or a tuple aligned to
    the order in `planes`.

    Callers get a guaranteed close() on any exit path."""
    import zenoh
    from xbrain.common.zenoh.session_factory import build_session_config

    plane_list: List[str] = list(planes)
    if not plane_list:
        raise ValueError("open_planes: at least one plane required")

    sessions: List[Tuple[str, "zenoh.Session"]] = []
    try:
        for plane in plane_list:
            cfg = build_session_config(plane)
            sess = zenoh.open(cfg)
            sessions.append((plane, sess))
        if len(sessions) == 1:
            yield sessions[0][1]
        else:
            yield tuple(s for _plane, s in sessions)
    finally:
        # Close in REVERSE open order so a plane that depends on
        # another is torn down first.
        for _plane, sess in reversed(sessions):
            try:
                sess.close()
            except Exception:      # noqa: BLE001 -- best-effort on shutdown
                pass
