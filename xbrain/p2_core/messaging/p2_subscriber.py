"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: p2_subscriber.py
Brief: BIZ-P2-0 -- subscriber wrapper enforcing P2_CORE_SUB whitelist

Description:
Wraps xbrain/common/zenoh/subscriber_registry.SubscriberRegistry with
one added policy: the key expression MUST be in P2_CORE_SUB (literal
or template-matched) before the declaration reaches the session.
Without this policy check, a typo like `state/robots` (extra 's')
would silently declare an unrelated subscription that never fires.

The strong-reference discipline (BIZ-P2-0 doc #1: 'declare_subscriber
must land on self.x= / list.append / SubscriberRegistry.declare') is
inherited from SubscriberRegistry: every declare stores the returned
handle in the registry's internal list, so nothing here loses the
reference.

Static CI grep (CLAUDE.md 4.3 + BIZ-P2-0 spec #1) enforces "no bare
`session.declare_subscriber` outside this file". That grep lives in
scripts/lint/no_dangling_subscriber.py (already landed); this module
is the ONLY authorised call site inside xbrain/p2_core/.
"""

from __future__ import annotations

from typing import Any, Callable, FrozenSet

from xbrain.common.zenoh.subscriber_registry import SubscriberRegistry
from xbrain.p2_core.messaging.whitelist_gate import check_sub_keys


class P2Subscriber:
    """P2's subscriber facade.

    Usage:
        subs = P2Subscriber(registry=SubscriberRegistry(bus=event_bus))
        subs.declare(session, "cmd/motion/intent", on_intent)
        subs.declare(session, "cmd/estop", on_estop)
    """

    def __init__(self, registry: SubscriberRegistry) -> None:
        self._registry = registry
        # Track keys the caller declared so the startup selfcheck has
        # a snapshot to compare against P2_CORE_SUB.
        self._declared: set = set()

    def declare(
        self,
        session: Any,
        key_expr: str,
        handler: Callable,
    ) -> None:
        """Whitelist-check `key_expr` against P2_CORE_SUB, then hand
        off to the underlying SubscriberRegistry which keeps a strong
        reference to the resulting subscriber object."""
        check_sub_keys([key_expr])
        self._registry.declare(session, key_expr, handler)
        self._declared.add(key_expr)

    @property
    def declared_keys(self) -> FrozenSet[str]:
        """Snapshot of subscribed keys. For selfcheck / diagnostics."""
        return frozenset(self._declared)

    def close(self) -> None:
        """Shut down every subscription. Delegates to the registry."""
        self._registry.close()
