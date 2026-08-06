"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: event_bus.py
Brief: The one door from a Zenoh callback thread into the process event loop

Description:
What problem this solves. zenoh-python runs subscriber callbacks on threads the
Rust runtime owns, never on the process event loop. CLAUDE.md 4.2 states that and
then lists what such a callback may NOT do: asyncio.create_task, an asyncio
Queue put_nowait, any await, and a direct event_bus.publish. The reason those are
banned is not style: asyncio structures are not thread-safe, and the way they
fail off-loop is not a crash. Work queued from a foreign thread lands in a loop
that is asleep in epoll and has not been woken for it, so the message is late, or
lost, or arrives interleaved with loop state that was half-updated. Every one of
those reads downstream as a flaky sensor or a flaky link, and the search starts
at the sensor. This module gives the correct path one short name --
publish_threadsafe -- and makes the incorrect path raise the first time it is
used rather than work nine times out of ten.

Where the design comes from, said plainly because it is thinner than usual for
this project. EventBus and publish_threadsafe appear in NO volume: grep 10, 11
and 20 for either name and both come back empty. The only authority is
CLAUDE.md 4.2, which names exactly two permitted mechanisms
(event_bus.publish_threadsafe and loop.call_soon_threadsafe) and forbids the
rest. So this file implements that rule and nothing built on top of it. Delivery
guarantees, replay, cross-topic ordering, a topic taxonomy -- none of that is
written down anywhere, and inventing it here would bury a contract in a helper
module where no reviewer would look for one. What crosses a process boundary is
Zenoh, and Zenoh's guarantees are frozen in 11 S2.4.2; this bus never leaves one
process and adds none of its own.

What this module deliberately does NOT do:
  * It does not import zenoh, and must not start. Keeping it client-free is what
    lets every case in tests/common/zenoh/test_subscriber_registry.py run on a
    machine with no eclipse-zenoh, and a test that skips because an unrelated
    dependency is missing reports as green.
  * It does not own subscribers or their lifetime. That is
    subscriber_registry.py, for the reason CLAUDE.md 4.3 gives.
  * It does not persist, replay or number anything. Durable events with a resend
    cursor are EVT-13 in 11 S2.4.5, and they are the gateway's job over Zenoh
    Q3, not an in-process fan-out helper's.
  * It does not decide QoS. That is INF-ZN-3 against 11 S2.4.7.
  * It has no unsubscribe and no close. Nothing needs them today and CLAUDE.md
    9.3 forbids reserving an interface for later; the registry's close() takes
    the transport down, which is what actually stops delivery.

Traps that look correct and are not:
  * Defaulting the loop argument to asyncio.get_event_loop(). On 3.10 that
    either returns a loop nobody runs or creates one, and either way
    publish_threadsafe then queues callbacks into a loop that never turns. There
    is no exception and no log line -- the messages simply never arrive. Hence
    the loop is required at construction.
  * "Optimising" publish_threadsafe into a direct call when it notices it is
    already on the loop thread. It changes ordering under the caller (a direct
    call runs ahead of work already queued) and it makes the foreign-thread path
    -- the only one that is hard to get right -- the one that stops being
    exercised.
  * Letting publish() forward to publish_threadsafe when called off-loop, to be
    helpful. That deletes the only enforcement CLAUDE.md 4.2 has: the caller
    that meant to run on the loop never finds out it did not, and the
    prohibition becomes advice.
  * Wrapping handler dispatch in "except Exception: pass". A subscriber that has
    been broken for a week then looks exactly like a subscriber with nothing to
    do. Exceptions are left to propagate -- to the caller for publish(), to the
    loop exception handler for publish_threadsafe -- and the cost is stated in
    the comment on _dispatch.
  * Registering an "async def" handler. It would be called, its coroutine
    dropped on the floor, and nothing would run: no error, no delivery. It is
    rejected at subscribe() time; see the comment there for which shapes that
    check can and cannot catch.
"""

import asyncio
import threading
from typing import Any, Callable, Dict, List

# Imported as names, never written as string literals. CLAUDE.md 3.5 forbids the
# literal and the payoff is immediate: a misspelled name is an ImportError here,
# whereas a misspelled literal travels to whoever branches on the code.
from ..errors import E_INTERNAL
from ..errors.exceptions import XbrainError

#: The callable a subscriber registers. It takes the payload and returns nothing;
#: it runs on the loop thread, always, which is the entire point of this module.
Handler = Callable[[Any], None]


# Both error types below carry E_INTERNAL rather than a more specific row, and
# the choice is deliberate. E_SCHEMA is about a payload that arrived malformed
# from a peer; E_QOS_VIOLATION is the startup self-check in 11 S2.4.8. Neither
# describes this condition, which is our own process calling its own API from the
# wrong thread -- a defect on this side of the wire. K's row in codes.yaml reads
# 本节所有其他码都不适用时的兜底, which is exactly the case.
#
# They live here rather than in common/errors/exceptions.py for the same reason
# ZenohPlaneConfigError does next door: the family base stays in the lowest
# layer so that "except XbrainError" keeps meaning what it says, while the
# specific type stays beside the module that raises it. CLAUDE.md 4.5 bans
# raising bare Exception, and that is what the base type is for.
class ThreadAffinityError(XbrainError):
    """A loop-affine call was made from a thread that is not running the loop."""

    def __init__(self, message: str):
        super().__init__(E_INTERNAL, message)


class HandlerContractError(XbrainError):
    """A handler was registered that cannot be delivered to synchronously."""

    def __init__(self, message: str):
        super().__init__(E_INTERNAL, message)


class EventBus:
    """Synchronous fan-out on the loop thread, with one thread-safe entry point.

    Three methods, matching the three things CLAUDE.md 4.2 has an opinion about:
    subscribe (register work that must run on the loop), publish (fan out, only
    legal on the loop thread), publish_threadsafe (the single door a Zenoh
    callback thread may use).
    """

    # The loop is a required argument. See the first trap in the header for what
    # a default would cost: a bus wired to a loop nobody runs delivers nothing
    # and says nothing about it.
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        # topic -> handlers, in registration order. A dict of lists rather than a
        # dict of sets: order is what a reader will assume when two handlers on
        # one topic have to run in a particular sequence, and a set would make
        # that assumption silently false.
        self._handlers: Dict[str, List[Handler]] = {}

    # subscribe -- register a handler for one topic.
    #
    # Callable from any thread, on purpose. Processes declare their subscriptions
    # during startup, which for several of them happens before the loop is
    # running at all, so requiring loop affinity here would ban the normal
    # sequence. CPython's list append and dict insert are atomic under the GIL,
    # so a dispatch running concurrently on the loop thread sees either the old
    # list or the new one and never a torn one. What is NOT ordered is delivery:
    # a handler registered after a sample was already queued does not receive
    # that sample. That is precisely why SubscriberRegistry.declare() calls this
    # BEFORE it declares the Zenoh subscriber -- see the comment there.
    def subscribe(self, topic: str, handler: Handler) -> None:
        """Register handler for topic. Handlers run on the loop thread."""
        # An async def handler would be called, would return a coroutine, and the
        # coroutine would be discarded -- no delivery, no exception, at most a
        # RuntimeWarning nobody reads. That is the fail-silent shape this whole
        # file exists to close, so it is rejected at registration.
        #
        # Scope of the check, stated so nobody reads more into it: it catches the
        # plain "async def handler" shape. A functools.partial around a coroutine
        # function, or an object with an async __call__, passes it. The right way
        # for a handler to reach async code is to be a normal function that calls
        # loop.create_task(...) -- which is legal, because by then it is running
        # on the loop thread.
        if asyncio.iscoroutinefunction(handler):
            raise HandlerContractError(
                "handler for topic %r is a coroutine function; EventBus "
                "dispatches synchronously on the loop thread, so its coroutine "
                "would never be awaited. Register a plain function that calls "
                "loop.create_task() instead (CLAUDE.md 4.2)" % topic
            )
        # setdefault, not get-with-default followed by an assignment: the two-step
        # version has a window in which a concurrent subscribe on another thread
        # loses its handler, and the loss is invisible.
        self._handlers.setdefault(topic, []).append(handler)

    # publish -- fan out now, on this thread, which must be the loop thread.
    #
    # This is the method CLAUDE.md 4.2 names as forbidden inside a Zenoh
    # callback. It is not removed, because on-loop code has a legitimate use for
    # a synchronous fan-out with no scheduling hop; it is guarded, so that the
    # forbidden use fails loudly at its first call instead of corrupting loop
    # state on a schedule nobody can reproduce.
    def publish(self, topic: str, payload: Any) -> None:
        """Deliver payload to topic's handlers. Loop thread only."""
        self._require_loop_thread("publish")
        self._dispatch(topic, payload)

    # publish_threadsafe -- the one door from a foreign thread.
    #
    # loop.call_soon_threadsafe is the mechanism CLAUDE.md 4.2 permits by name.
    # It is also correct when called from the loop thread itself, and it is NOT
    # special-cased for that: see the second trap in the header -- shortcutting
    # would reorder delivery relative to work already queued and would leave the
    # foreign-thread path untested.
    #
    # Two failure modes are left to propagate rather than swallowed. A closed
    # loop makes call_soon_threadsafe raise RuntimeError, which is the truth: the
    # process is shutting down and this message is not going to be delivered. A
    # loop that exists but is not running takes the callback and never runs it,
    # which this module cannot detect and does not pretend to -- the symptom is
    # a bus with no deliveries at all, and the cause is a caller that never
    # started its loop.
    def publish_threadsafe(self, topic: str, payload: Any) -> None:
        """Schedule delivery on the loop. Safe from any thread."""
        self._loop.call_soon_threadsafe(self._dispatch, topic, payload)

    # _dispatch -- the actual fan-out. Always runs on the loop thread: either
    # directly from publish(), which has just checked, or as a loop callback
    # scheduled by publish_threadsafe.
    def _dispatch(self, topic: str, payload: Any) -> None:
        """Call every handler registered for topic, in registration order."""
        # Explicit membership test rather than a get() with a default. A handler
        # list is not a safety parameter, so CLAUDE.md 3.1 does not literally
        # apply, but writing the absent case out costs one line and leaves a
        # reviewer nothing to rule out.
        #
        # A topic with no handlers is not an error. Subscribers are allowed to be
        # absent, and for the path that matters it cannot happen anyway:
        # SubscriberRegistry.declare() registers the handler before it declares
        # the subscriber that produces this topic.
        if topic not in self._handlers:
            return
        # Iterate over a snapshot. A handler that subscribes during dispatch --
        # the shape a lazily-wired module has -- would otherwise mutate the list
        # being iterated and raise RuntimeError from deep inside the loop
        # callback, where the traceback names this line and not the caller.
        for handler in tuple(self._handlers[topic]):
            # No try/except around this call. See the fourth trap in the header:
            # swallowing here makes a broken subscriber indistinguishable from an
            # idle one. The cost is stated rather than hidden -- if one handler
            # raises, later handlers for the SAME message do not run, and the
            # exception surfaces to the caller (publish) or to the loop exception
            # handler (publish_threadsafe). Both are loud, which is the point.
            handler(payload)

    # _require_loop_thread -- the affinity guard behind publish().
    #
    # It asks asyncio which loop is running on THIS thread, which is the precise
    # question: a thread with no loop and a thread running a different loop are
    # both wrong, and comparing thread identity against a remembered one would
    # not distinguish them before the loop has started.
    def _require_loop_thread(self, api: str) -> None:
        """Raise ThreadAffinityError unless this thread runs the bus's loop."""
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            # Named type, never a bare except (CLAUDE.md 4.5). RuntimeError is
            # what asyncio raises for "no running event loop in this thread",
            # which is the ordinary case for a Zenoh callback thread and not an
            # error in itself -- it becomes one only because publish() is
            # loop-affine.
            running = None
        # Identity, not equality: two different loop objects can compare equal
        # under a subclass that defines __eq__, and delivering into the wrong
        # loop is the failure this guard exists to prevent.
        if running is not self._loop:
            raise ThreadAffinityError(
                "%s() is loop-affine but was called from thread %r, which is "
                "not running this bus's event loop. A Zenoh subscriber callback "
                "runs on a Rust-owned thread and must use publish_threadsafe() "
                "(CLAUDE.md 4.2)" % (api, threading.current_thread().name)
            )


# Exported names. The two error types are public because a caller has to be able
# to catch exactly the failure it can handle; _dispatch and _require_loop_thread
# are not, because neither is meaningful outside the object that owns them.
__all__ = ["EventBus", "Handler", "HandlerContractError", "ThreadAffinityError"]
