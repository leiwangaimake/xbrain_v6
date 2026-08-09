"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: p2_publisher.py
Brief: BIZ-P2-0 -- thread-checked publisher wrapper for p2_core

Description:
Wraps a Zenoh session's publisher declaration with two guarantees:

  1. Whitelist enforcement at declare-time. A key outside P2_CORE_PUB
     (literal or template-matched) is refused BEFORE the underlying
     session sees it. Without this, a stray publisher.declare would
     put an unregistered key on the wire and 11 S1.1.6 whitelist
     drift would only be caught at cross-plane audit time.

  2. Thread-affinity check at put-time. Direct publishes are
     permitted only from an allowed thread (P2's main / fast / tx
     threads). A Zenoh subscriber callback runs on a Rust-owned
     thread; publishing from THERE is CLAUDE.md 4.2 forbidden and
     would race the event loop. Use publish_threadsafe() instead
     (which posts back onto the event loop).

This class is DELIBERATELY session-agnostic: the caller injects the
session object. In production the injection is a real zenoh.Session;
in tests it is a FakeSession that records puts. Testability without
requiring a live zenoh install is the point.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, FrozenSet, Optional, Protocol

from xbrain.p2_core.messaging.whitelist_gate import (
    WhitelistViolation, check_pub_keys,
)


class ThreadAffinityError(RuntimeError):
    """Raised when publish() is called from a thread that is not on
    the allowed set. CLAUDE.md 4.2: Zenoh callback threads MUST post
    back via publish_threadsafe -- direct publish races the loop."""


class _PublisherLike(Protocol):
    """Structural type: the object a Zenoh session returns from
    declare_publisher. Tests inject a fake with the same shape."""

    def put(self, payload: bytes) -> None: ...


class _SessionLike(Protocol):
    """Structural type for the Zenoh session. Real: zenoh.Session.
    Test: FakeSession implementing the same call shape."""

    def declare_publisher(self, key_expr: str) -> _PublisherLike: ...


class P2Publisher:
    """P2's publisher facade.

    Usage:
        pub = P2Publisher(session=zenoh_session, allowed_threads={main_tid})
        pub.declare("state/mode")
        pub.declare("state/arb/motion")   # template match against P2_CORE_PUB
        pub.put("state/mode", b'{"mode":"idle"}')     # OK from main thread
        # From a Zenoh callback (different thread) instead:
        pub.publish_threadsafe(loop, "state/mode", payload)
    """

    def __init__(
        self,
        session: _SessionLike,
        allowed_threads: Optional[set] = None,
    ) -> None:
        self._session = session
        # Set of thread ids allowed to call put() directly. Empty set
        # means "no thread check" (only for tests that don't want the
        # affinity guard). Production callers ALWAYS pass main+fast+tx.
        self._allowed_threads = allowed_threads if allowed_threads is not None else set()
        # Registered publisher handles, keyed by literal declared key.
        # Kept as attribute so the objects live long enough for Zenoh
        # to keep the declaration alive.
        self._publishers: Dict[str, _PublisherLike] = {}

    def declare(self, key: str) -> None:
        """Declare a Zenoh publisher for `key`. Whitelist-checked.

        Raises WhitelistViolation if `key` is not in P2_CORE_PUB
        (literal or template-matched). On success the publisher
        object is stored on this instance so the caller does not
        have to remember to keep a reference alive.
        """
        # Whitelist-check exactly this key (subset-of-1).
        check_pub_keys([key])
        # Ask the session to create the publisher; the return value
        # MUST be stored (session -> tokio thread will discard the
        # declaration otherwise -- same failure mode as
        # SubscriberRegistry's strong-ref rule).
        self._publishers[key] = self._session.declare_publisher(key)

    def put(self, key: str, payload: bytes) -> None:
        """Publish payload on a previously-declared key.

        Thread-checked: caller must be on allowed_threads (or the
        check is disabled by passing an empty set at construction)."""
        if self._allowed_threads:
            tid = threading.get_ident()
            if tid not in self._allowed_threads:
                raise ThreadAffinityError(
                    "put(%r) called from thread %d which is not in "
                    "allowed_threads %s -- use publish_threadsafe "
                    "from a Zenoh callback (CLAUDE.md 4.2)"
                    % (key, tid, sorted(self._allowed_threads)))
        pub = self._publishers.get(key)
        if pub is None:
            raise KeyError(
                "publisher for %r was never declared; call declare() first"
                % key)
        pub.put(payload)

    def publish_threadsafe(
        self,
        loop_call_soon_threadsafe: Callable,
        key: str,
        payload: bytes,
    ) -> None:
        """Post a publish back onto the event loop.

        Accepts the loop's call_soon_threadsafe rather than the loop
        itself so this class does not need to import asyncio in
        tests that do not have a loop. Called from a Zenoh callback
        thread to hand the publish across the loop boundary safely."""
        loop_call_soon_threadsafe(self.put, key, payload)

    @property
    def declared_keys(self) -> FrozenSet[str]:
        """Snapshot of keys currently declared. For selfcheck."""
        return frozenset(self._publishers.keys())
