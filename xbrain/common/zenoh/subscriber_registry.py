"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: subscriber_registry.py
Brief: The long-lived container every Zenoh subscriber handle must land in

Description:
What problem this solves. In zenoh-python the object returned by
declare_subscriber IS the subscription: when Python collects it, the Rust side
undeclares, and nothing is logged on either side. CLAUDE.md 4.3 calls this the
number one trap of the client and gives the three shapes that lose the handle --
a bare call, an explicit "_ =", and a local variable that goes out of scope. The
symptom afterwards is a subscriber that receives nothing, which is
indistinguishable from a publisher that never started, from a QoS mismatch, and
from a router the process never connected to. Days go into the network before
anyone looks at the reference count. This module makes the handle land in an
instance attribute by construction, so losing it takes a deliberate edit instead
of a moment of inattention.

The second half of the same problem, and why the two live in one class. The
callback zenoh-python invokes runs on a Rust-owned thread (CLAUDE.md 4.2), so
whatever the handle survives to receive still cannot touch the event loop
directly. A registry that handed the raw sample to a caller-supplied callback
would put that back on every caller. Instead declare() takes a handler that runs
on the LOOP, and the only thing that runs on the Rust thread is on_message,
which does one thing: EventBus.publish_threadsafe. The rule and its enforcement
sit in the same place a process actually declares its subscriptions.

Where the design comes from:
  * CLAUDE.md 4.3, verbatim, for the strong reference: declare_subscriber(...)
    的返回值必须接住到 long-lived 容器. Its own example names this class.
  * CLAUDE.md 4.2 for the threading rule and for the two permitted mechanisms.
  * 11 S2.4.8, the anti-pattern table the startup self-check must reject, for
    what this module is NOT allowed to decide -- see below.
  * The API shape is checked against the installed client: Session.
    declare_subscriber(key_expr, handler=None, *, allowed_origin=None) and
    Subscriber.undeclare(), which is why declare() passes the callback
    positionally and close() calls undeclare().

What this module deliberately does NOT do:
  * It does not choose QoS, and it does not pass a channel handler. 11 S2.4.2
    freezes a 订阅端 handler per profile (Ring(1) for RT periodic keys, FIFO for
    events) and 11 S2.4.8 A-3 makes the wrong choice a startup-refusal
    condition. In zenoh-python a callback and a channel are alternative values of
    the SAME handler argument -- the installed client exports Callback,
    DefaultHandler, FifoChannel and RingChannel side by side -- so the profile's
    queue discipline cannot simply be added next to a callback, and which of the
    two shapes each key uses is not something this item may settle on its own.
    That decision belongs to INF-ZN-3 (11 S2.4.7 bindings) and INF-ZN-5 (the
    startup self-check), and it is recorded here rather than quietly made.
  * It does not check the key against a registry or a cross-plane whitelist.
    11 S2.2 holds the key table and 11 S1.1.6 the whitelist; enforcing them is
    INF-ZN-5 and INF-ZN-7.
  * It does not open sessions. Session config is session_factory.py; the session
    object is passed in, and it is duck-typed on purpose so the whole suite runs
    without eclipse-zenoh installed.
  * It does not import zenoh, for the same reason event_bus.py does not.

Traps that look correct and are not:
  * Returning the handle from declare() "so the caller can keep it too". A
    second owner is how a handle ends up undeclared by whoever drops it first,
    and it re-teaches exactly the habit CLAUDE.md 4.3 is trying to remove. The
    method returns None.
  * Declaring the subscriber first and registering the handler afterwards. A
    sample can arrive on the Rust thread between those two statements, and a
    sample dispatched to an empty handler list is dropped with no trace. The
    order in declare() is therefore fixed and commented at the call.
  * Storing handles anywhere other than an instance attribute -- a module-level
    list, a local, a WeakSet. The first two are covered by CLAUDE.md 4.3; a
    WeakSet is worse than either, because it looks like a container while
    holding nothing.
  * Testing this with a fake whose subscriber does nothing when collected. Then
    "no undeclare after gc" is true of every implementation, including the
    broken one -- CLAUDE.md 3.2 form 1, an assertion a do-nothing implementation
    passes. The fake in tests/common/zenoh/test_subscriber_registry.py
    undeclares from __del__ for exactly this reason, and one case exists purely
    to prove that it does.
  * Swallowing an undeclare failure during close() to make shutdown look clean.
    close() pops each handle before undeclaring it, so a raise leaves the
    registry honest about what is still open rather than silently full.
"""

import functools
from typing import Any, List

from ..errors import E_INTERNAL
from ..errors.exceptions import XbrainError
from .event_bus import EventBus, Handler


# E_INTERNAL for the same reason event_bus.py gives: declaring on a closed
# registry is our own process misusing its own API, which is what K's fallback
# row covers. It is a named type rather than a bare Exception (CLAUDE.md 4.5) so
# that a caller doing orderly shutdown can catch this specific race and nothing
# else.
class RegistryClosedError(XbrainError):
    """declare() was called on a registry that has already been closed."""

    def __init__(self, message: str):
        super().__init__(E_INTERNAL, message)


class SubscriberRegistry:
    """Owns every Zenoh subscriber handle a process declares, for its lifetime.

    One per process is the intended shape, held by whatever owns the session.
    The bus is required at construction rather than passed per declaration:
    every subscription in a process delivers into the same loop, and an optional
    bus would be an invitation to declare one subscription without it, which is
    the case that then bypasses the threading rule.
    """

    def __init__(self, bus: EventBus) -> None:
        self._bus = bus
        # *** This attribute is the entire point of the class (CLAUDE.md 4.3).
        # It must stay an instance attribute: a local, a module global or a weak
        # container all let the handles be collected while the registry lives,
        # and the Rust side then undeclares with nothing logged anywhere.
        self._handles: List[Any] = []
        # Whether close() has completed. Not "whether close() was called": if an
        # undeclare raises, the registry has NOT finished closing and must not
        # pretend it has -- see close().
        self._closed = False

    # declare -- the only sanctioned way to create a subscription.
    #
    # session is duck-typed (Any) rather than annotated as zenoh.Session. That is
    # what lets the tests drive this with a fake that records declare/undeclare,
    # and what keeps this module importable where eclipse-zenoh is absent. The
    # cost is that a wrong object fails at the call rather than at type-check
    # time, which is acceptable: there is exactly one call and its AttributeError
    # names declare_subscriber.
    def declare(self, session: Any, key_expr: str, handler: Handler) -> None:
        """Declare a subscriber for key_expr and keep its handle alive."""
        # Refuse rather than declare a subscription nobody will ever undeclare.
        # A subscriber created after close() would also start delivering into a
        # loop the owner is in the middle of shutting down, and the samples would
        # arrive looking perfectly normal.
        if self._closed:
            raise RegistryClosedError(
                "registry is closed; declaring %r now would create a "
                "subscription no close() will undeclare" % key_expr
            )
        # Handler first, subscriber second. See the second trap in the header:
        # between declare_subscriber returning and a later subscribe() call there
        # is a window in which a sample arrives, finds no handler, and is dropped
        # silently. Moving this statement below the declaration at the end of
        # this method reintroduces that window, which is why the ordering has a
        # case of its own rather than only a comment.
        self._bus.subscribe(key_expr, handler)
        # functools.partial rather than a lambda closing over key_expr. In a loop
        # over several keys a lambda captures the variable, not its value, so
        # every subscriber would report the last key -- and the failure is not a
        # crash, it is messages delivered under the wrong topic.
        callback = functools.partial(self.on_message, key_expr)
        # *** The declaration and the append are ONE statement, and that is not
        # cosmetic. This is the line CLAUDE.md 4.3 exists for -- without the
        # append every assertion about this class still passes and every
        # subscription dies at the next collection -- and the CI rule in
        # CLAUDE.md 8.2 accepts exactly three shapes at a declare_subscriber call
        # site: self.x =, list.append, or SubscriberRegistry.declare. Writing it
        # as "subscriber = session.declare_subscriber(...)" followed by an append
        # is none of them; it is 落到局部变量, the shape the rule rejects, and it
        # stays correct only until someone inserts an early return between the
        # two lines. So the handle never exists as a bare local at all.
        self._handles.append(session.declare_subscriber(key_expr, callback))

    # on_message -- everything that runs on the Rust-owned callback thread.
    #
    # It is deliberately one statement. Anything added here runs off-loop, and
    # CLAUDE.md 4.2 forbids awaiting, creating tasks and touching asyncio queues
    # from this thread. Filtering, decoding and age checks all belong in the
    # handler, which runs on the loop.
    #
    # It is public because the criterion for this item drives it directly from a
    # foreign thread, and because being able to inject a sample without a live
    # Zenoh runtime is what makes the cross-thread behaviour testable at all.
    def on_message(self, key_expr: str, sample: Any) -> None:
        """Hand a sample to the loop. Called on a Zenoh callback thread."""
        # publish_threadsafe, never publish: publish is loop-affine and raises
        # here by design, which is how the prohibition in CLAUDE.md 4.2 gets
        # teeth instead of staying advice.
        self._bus.publish_threadsafe(key_expr, sample)

    # alive -- how many subscriptions this registry is currently holding open.
    #
    # A count, not the handles themselves: handing out the list would give a
    # caller a way to keep a handle after close(), which is the ownership split
    # this class exists to prevent. The count is what a startup self-check and a
    # test can both assert on.
    def alive(self) -> int:
        """Number of subscriber handles currently held."""
        return len(self._handles)

    # close -- undeclare everything, once.
    #
    # Pop-then-undeclare rather than iterate-then-clear. If one undeclare raises,
    # the handles that were already released are gone from the container and the
    # ones still open remain in it, so alive() keeps telling the truth and the
    # exception reaches the caller instead of being smoothed over. The popped
    # handle is dropped, which in the real client is itself an undeclare, so
    # nothing leaks on the Rust side either.
    #
    # Idempotent by construction: the second call finds an empty container and
    # undeclares nothing. That matters because shutdown paths get called twice --
    # once by the owner, once by a context manager or a signal handler -- and a
    # double undeclare on a real Subscriber is a second Rust-side call on a
    # handle that has already been consumed.
    def close(self) -> None:
        """Undeclare every held subscription. Safe to call more than once."""
        while self._handles:
            handle = self._handles.pop()
            handle.undeclare()
        # Set only after the loop completes. If an undeclare raised, the registry
        # is not closed and declare() must keep working, because the owner's
        # recovery path may well be to finish closing.
        self._closed = True

    # No __del__, on purpose. A finaliser that undeclared handles would run at an
    # unpredictable point during interpreter shutdown, on a thread that may no
    # longer have a working loop, and it would hide the very defect this class is
    # built to expose: a registry nobody kept a reference to. Shutdown is the
    # owner's job and close() is the one way to do it.


__all__ = ["RegistryClosedError", "SubscriberRegistry"]
