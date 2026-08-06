"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_subscriber_registry.py
Brief: INF-ZN-2 -- handles stay held, and callbacks cross into the loop safely

Description:
What these cases are worth. Both halves of INF-ZN-2 fail silently when they are
wrong, which is the only reason they are worth testing at all. A subscriber
handle that nobody holds is undeclared by the garbage collector with no log line
on either side (CLAUDE.md 4.3), and the process afterwards looks exactly like one
whose publisher never started, or whose QoS never matched, or whose router was
never reached -- four causes, one symptom, and the reference count is the last
place anybody looks. A callback that touches the event loop from a Rust-owned
thread (CLAUDE.md 4.2) corrupts loop state on a schedule nobody can reproduce,
and what comes back from the field is "the link drops messages sometimes".

*** The one thing that makes the first half a real test rather than a formality.
The criterion is "after gc.collect(), undeclare calls == 0". That sentence is
also true of a fake whose subscriber does nothing when it is collected -- indeed
of a fake that never undeclares under any circumstances -- which is CLAUDE.md
3.2 form 1, an assertion a do-nothing implementation passes. So FakeSubscriber
undeclares from __del__, the way the real client does when Python drops the
handle, and test_a_local_variable_loses_the_subscription exists purely to prove
that the mechanism fires. Delete that case and the rest of this file becomes
decoration that cannot fail.

Deliberately NOT used as a criterion anywhere here: whether a weakref to the
handle is still alive. Liveness is an implementation detail of the interpreter,
and it says nothing at all about the Rust side, which is where the subscription
actually lives -- a handle can be alive in Python while its subscription has been
torn down, and a test that watched liveness would report the wrong one green.
Every assertion below counts undeclare calls recorded on the fake instead. Weak
references appear exactly once, inside DeclareLog, and only so that a case can
reach a subscriber to inject a sample WITHOUT keeping it alive; if the log held
those references strongly, no handle could ever be collected and the mutation
case above could never go red. That failure is itself covered -- see the
log_holds_strong_refs mutation in the list at the end of this header.

The fake session is duck-typed on the two members the registry uses:
declare_subscriber(key_expr, handler) and Subscriber.undeclare(). Both were
checked against the installed eclipse-zenoh -- Session.declare_subscriber has
signature (key_expr, handler=None, *, allowed_origin=None) and Subscriber
exports undeclare -- so a registry that called either the wrong way would fail
here and not only against a live router. Nothing in this file imports zenoh: the
behaviour under test has to be verifiable on a machine without the client, and a
suite that skips reports as green.

Mutations these cases were verified against, per CLAUDE.md 3.3. Each was injected
into the file named, run, confirmed red, and reverted:
  * subscriber_registry.py -- handles appended to a function local instead of
    self._handles (the mutation the criterion names);
  * subscriber_registry.py -- close() iterating without emptying the container,
    so a second close undeclares everything twice;
  * subscriber_registry.py -- on_message calling bus.publish instead of
    publish_threadsafe;
  * subscriber_registry.py -- the subscriber declared before the handler is
    registered;
  * subscriber_registry.py -- the closed check moved after the declaration;
  * event_bus.py -- the coroutine-function check removed from subscribe();
  * event_bus.py -- the thread-affinity guard neutered, and separately made to
    fire unconditionally (the always-red shape, which the positive case catches);
  * event_bus.py -- handler dispatch wrapped in except-Exception-pass;
  * this file -- FakeSubscriber.__del__ removed, and DeclareLog holding its
    subscribers strongly. Both make the criterion unfalsifiable, and both turn
    test_a_local_variable_loses_the_subscription red.

How to read a failure here, since the two halves fail very differently. A
counting failure -- undeclare_calls off by one, alive() off by one -- is about
ownership: some object is holding, or has stopped holding, a handle. A timeout
failure -- one of the "never reached the loop" messages -- is about the thread
hop: either nothing was scheduled onto the loop, or the loop is not running. The
two never mean the same thing, which is why no case in this file asserts a count
that could also be satisfied by a timeout, and why every wait() is asserted
rather than left as a bare call whose False return would be discarded.

What is NOT covered here, so nobody mistakes a green run for more than it is.
This file never opens a Zenoh session, so it says nothing about whether the
declaration reaches a router, nothing about QoS -- 11 S2.4.2 freezes a
subscriber-side handler per profile and choosing it is INF-ZN-3 -- and nothing
about whether a key is registered or allowed across planes (11 S2.2, 11 S1.1.6,
enforced by INF-ZN-5 and INF-ZN-7). What it does establish is the property that
has to hold before any of those can matter: the handle is still there, and the
sample it carries reaches the loop on the loop's own thread.
"""

import asyncio
import gc
import os
import sys
import threading
import weakref

import pytest

# gc is imported for one reason only -- the criterion names gc.collect() -- and
# weakref for one reason only: letting DeclareLog observe a subscriber without
# keeping it. Neither appears anywhere in the module under test, and neither
# should migrate there: a registry that reached for either would be managing
# lifetime by inference instead of by holding a reference, which is the defect.

# The repository root, four levels up from tests/common/zenoh/. Computed rather
# than assumed so the file runs under pytest from any working directory; the
# suite has no conftest.py to do it centrally.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, ROOT)

# Imported from the package, not from the modules inside it. That is the surface
# a consumer actually writes against, so importing it here also checks that the
# package re-exports what it claims to.
from xbrain.common.zenoh import (EventBus, HandlerContractError,  # noqa: E402
                                 RegistryClosedError, SubscriberRegistry,
                                 ThreadAffinityError)

#: Three keys, because the criterion says three subscriptions. They are written
#: in the full xbrain/{rid}/... form of 11 S2.2 only so that they read like real
#: keys; the registry never parses a key, and checking one against the registered
#: table or the cross-plane whitelist is INF-ZN-5 / INF-ZN-7 and not this item.
#: Three different keys rather than one key three times, because a registry that
#: keyed its container by key expression would collapse duplicates and still pass
#: a count of three.
THREE_KEYS = (
    "xbrain/testrobot/rt/motion/cmd_vel",
    "xbrain/testrobot/rt/lidar/grid",
    "xbrain/testrobot/state/targets",
)

#: Upper bound on how long a case waits for the loop thread to run a callback.
#: Generous on purpose: this is a deadlock guard, not a latency assertion. The
#: real budget for anything on this path lives in 10 S11.1 and is measured by
#: INF-OB-1; asserting a number here would put a second, drifting copy of it in a
#: place nobody maintains. threading.Event.wait is monotonic internally, so no
#: case in this file touches a wall clock (CLAUDE.md 3.4).
WAIT_S = 5.0

#: How many samples the delivery case pushes through one subscriber. It matches
#: the "send 100, receive 100" shape CFG-CM-17 gives for the integration case, so
#: the unit-level version fails for the same reason the integration one would:
#: under the local-variable mutation the handle has been collected by then and
#: there is nothing left to deliver through.
SAMPLE_COUNT = 100


# ---------------------------------------------------------------------------
# The fake session. Its fidelity is the whole argument of this file.
#
# Why a fake at all, rather than a real session against a real router. Three
# reasons, each sufficient on its own. First, the criterion asks for undeclare
# CALLS to be counted, and a real Subscriber gives no way to observe them --
# the undeclare happens on the Rust side and is not reported back. Second, the
# defect under test is a garbage-collection event, which needs the handle's
# lifetime under the test's control and nobody else's. Third, a case that needs
# a router running is a case that gets skipped, and a skipped isolation check
# reads identically to a passing one in a summary line -- this project has
# already been caught reading one as the other.
#
# The corresponding obligation is that the fake must model the ONE behaviour the
# criterion depends on: dropping a handle undeclares. That is what __del__ below
# is for, and one whole case exists to prove it still fires.
# ---------------------------------------------------------------------------

class DeclareLog:
    """Records declare and undeclare calls, and holds NO subscriber strongly.

    The no-strong-reference part is load-bearing rather than tidy. A log that
    kept the objects it handed back would keep every subscription alive for the
    length of the case, "undeclare == 0" would then hold for any implementation
    whatsoever -- including one that never stores a handle at all -- and the
    mutation named in the criterion could never turn anything red.
    """

    def __init__(self):
        # Key per declare_subscriber call, in call order. Order is kept because
        # one case asserts the registry declared exactly the keys it was asked
        # for, and a set would lose both order and duplicates.
        self.declared = []
        # (key, cause) per undeclare, where cause is "explicit" for a call to
        # undeclare() and "gc" for a handle that was collected. The two are kept
        # apart because they mean opposite things: one is an orderly shutdown,
        # the other is precisely the silent failure CLAUDE.md 4.3 describes. A
        # single counter would let a registry that dropped all its handles and
        # then reported a clean close() reach the same total.
        self.undeclared = []
        # Weak, never strong. See the class docstring, and the header on why
        # liveness is not used as a criterion anywhere in this file.
        self.subscriber_refs = []

    @property
    def undeclare_calls(self):
        """The count the criterion is literally written in terms of."""
        return len(self.undeclared)

    def causes(self):
        """Just the causes, for the cases that care HOW a subscription ended."""
        return [cause for _key, cause in self.undeclared]


class FakeSubscriber:
    """Stands in for zenoh.Subscriber, including what happens when it is dropped.

    *** __del__ is not decoration here. In zenoh-python the returned object IS
    the subscription: collecting it undeclares on the Rust side, and neither side
    logs anything. A fake without this method turns every assertion in this file
    into one that a broken registry also passes, which is the exact failure mode
    CLAUDE.md 3.2 lists first.
    """

    def __init__(self, log, key_expr, callback):
        self._log = log
        self.key_expr = key_expr
        # The registry's callback, kept so that a case can inject a sample the
        # way the Rust runtime would. The real client holds it too, which is why
        # keeping it here does not make the fake less faithful.
        self.callback = callback
        # Guards against counting one subscription twice: an explicit undeclare()
        # followed later by collection must record one undeclare and not two,
        # which is also how the real client behaves. Without the flag,
        # test_declared_handles_survive_gc_and_close_undeclares_them would pass
        # or fail depending on when the collector happened to run.
        self._undeclared = False

    def undeclare(self):
        """The orderly path, called by SubscriberRegistry.close()."""
        self._record("explicit")

    def __del__(self):
        """The silent path: Python dropped the handle, so the subscription ends."""
        self._record("gc")

    def _record(self, cause):
        # First cause wins. See the comment on _undeclared for why a second
        # record would make the criterion timing-dependent.
        if self._undeclared:
            return
        self._undeclared = True
        self._log.undeclared.append((self.key_expr, cause))

    def deliver(self, sample):
        """Invoke the registry's callback, as the Zenoh runtime would.

        Called from a foreign thread in the delivery cases, because that is where
        the real one runs (CLAUDE.md 4.2). Calling it from the test thread would
        still exercise the registry, but it would exercise the one situation that
        never happens in production and would leave the thread hop unverified.
        """
        self.callback(sample)


class FakeSession:
    """Duck-typed stand-in for zenoh.Session, exactly one method deep.

    Only declare_subscriber is modelled, because that is the only member the
    registry touches. Adding more would be inventing behaviour: the contract
    between this project and the client is fixed by the client's own signature,
    and a fake that grows features the code under test never calls is a second
    implementation nobody maintains.
    """

    def __init__(self, log, sample_on_declare=None):
        self._log = log
        # When set, a sample is delivered from INSIDE declare_subscriber. That
        # reproduces the window a real subscriber has: traffic can arrive before
        # the caller's next statement runs. One case uses it to pin the statement
        # order inside SubscriberRegistry.declare(), which is otherwise invisible
        # -- both orders produce a working system on a quiet bus.
        self._sample_on_declare = sample_on_declare

    def declare_subscriber(self, key_expr, handler):
        self._log.declared.append(key_expr)
        subscriber = FakeSubscriber(self._log, key_expr, handler)
        # Weak reference only. A strong one here would defeat the mutation case;
        # see DeclareLog, and the log_holds_strong_refs mutation in the header.
        self._log.subscriber_refs.append(weakref.ref(subscriber))
        if self._sample_on_declare is not None:
            # Deliberately before returning, that is, before the registry has had
            # any chance to do anything else with the handle. Delivering after
            # the return would test nothing: by then both statement orders look
            # the same.
            subscriber.deliver(self._sample_on_declare)
        return subscriber


# ---------------------------------------------------------------------------
# A real event loop, on a real second thread.
#
# The shape below mirrors production rather than a testing convenience. In every
# P1..P5 process the loop belongs to the process and the Zenoh callbacks arrive
# from threads the client owns; here the loop belongs to LoopThread and the
# "callback" arrives from the pytest thread or from run_off_loop. The direction
# of the hop is the same, which is what makes the assertions transferable.
#
# Why not pytest-asyncio and an async test function. The property under test is
# what happens when a NON-loop thread hands work to a loop; an async test runs
# the case body ON the loop, so the interesting thread boundary would not exist
# and every case would pass trivially. The dependency is also not configured in
# this repository, and adding one to make a test easier is how a suite acquires
# a skip that later hides a real failure.
# ---------------------------------------------------------------------------

class LoopThread:
    """An asyncio loop running in its own thread, as every P1..P5 process has.

    A real loop and a real thread, not a mock. The property under test is that
    work crosses a thread boundary into a loop that is otherwise asleep in epoll;
    a stub loop that recorded calls would confirm the calls were made and prove
    nothing whatsoever about the crossing, which is the only hard part.
    """

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self._running = threading.Event()
        # daemon=True so a case that fails mid-way cannot wedge the whole run;
        # the fixture still joins the thread explicitly, so this is a backstop
        # and not the normal path.
        self._thread = threading.Thread(target=self._run, name="test-loop",
                                        daemon=True)
        # Exceptions the loop reports through its exception handler, so a case
        # can assert that a handler failure surfaced somewhere instead of
        # vanishing. Without this list, "the exception was not swallowed" would
        # have no observable form on the threadsafe path.
        self.loop_errors = []

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.set_exception_handler(self._on_loop_error)
        # Set from INSIDE the loop, so start() returns only once the loop is
        # actually turning. Setting it before run_forever would let a case queue
        # work into a loop that has not started -- which is the one failure the
        # bus cannot detect, and it must not be confused with the failures that
        # are under test here.
        self.loop.call_soon(self._running.set)
        self.loop.run_forever()

    def _on_loop_error(self, loop, context):
        # asyncio hands the handler a context dict, not an exception, and the
        # exception may be absent from it -- a callback that never ran, a task
        # that was destroyed pending, and a raised exception all arrive here in
        # different shapes. The whole dict is kept so the assertion can look for
        # the one key it cares about instead of this method deciding for it.
        self.loop_errors.append(context)

    @property
    def ident(self):
        """Thread id of the loop thread, for the "which thread ran it" cases."""
        return self._thread.ident

    def start(self):
        self._thread.start()
        # Asserted, not merely waited on. A wait that timed out would otherwise
        # let every later case fail with a confusing timeout of its own instead
        # of naming the real cause here.
        assert self._running.wait(WAIT_S), "loop thread did not start"

    def stop(self):
        # call_soon_threadsafe is the only legal way to reach a running loop from
        # another thread -- the same rule the code under test lives by.
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(WAIT_S)
        assert not self._thread.is_alive(), "loop thread did not stop"
        # Closed after the thread is confirmed gone. Closing a loop that is still
        # running raises, and the raise would land in teardown where it masks
        # whatever the case was actually reporting.
        self.loop.close()

    def drain(self):
        """Block until work queued before this call has run.

        Used by the cases that assert something did NOT happen. Without it,
        "nothing was delivered" could equally mean "not delivered yet", and the
        case would be green for the wrong reason -- the same shape as an
        assertion that can never fail.
        """
        done = threading.Event()
        self.loop.call_soon_threadsafe(done.set)
        assert done.wait(WAIT_S), "loop did not drain"


@pytest.fixture
def loop_thread():
    """A started loop thread, stopped even when a case fails.

    try/finally rather than a bare yield: a case that fails after queueing work
    would otherwise leave a running loop and a live thread behind, and the next
    case would inherit them.
    """
    thread = LoopThread()
    thread.start()
    try:
        yield thread
    finally:
        thread.stop()


class Collector:
    """A handler that records what it received and which thread ran it."""

    # expected defaults to 1 because all but one case sends a single sample.
    # This is not the kind of default CLAUDE.md 3.1 bans: that rule covers
    # calibrated safety values under common.safety.* and common.spec.*, where a
    # plausible-looking default silently replaces a value nobody measured. A test
    # helper's message count is neither calibrated nor safety-bearing, and the
    # one case that needs another value passes it explicitly.
    def __init__(self, expected=1):
        self.payloads = []
        self.threads = set()
        # Set once the expected number of payloads has arrived, so a case waits
        # on an event rather than sleeping for a guessed interval. A sleep would
        # make this file slow when it passes and flaky when it fails.
        self.done = threading.Event()
        self._expected = expected

    def __call__(self, payload):
        # No lock, deliberately. Every call is supposed to arrive on the loop
        # thread -- which is exactly what the assertions below check -- so there
        # is no concurrent mutation to protect against. Adding a lock would hide
        # the defect rather than guard it: an off-loop delivery would then be
        # safe to record, and the case that exists to catch it would pass.
        self.payloads.append(payload)
        self.threads.add(threading.get_ident())
        if len(self.payloads) >= self._expected:
            self.done.set()


def run_off_loop(func, *args):
    """Run func on a thread that has no running loop; return {result, error}.

    This is the stand-in for a Zenoh callback thread. What matters is not that
    Rust owns it but that it is not the thread running the bus's loop, which is
    the precise condition the affinity guard tests for.
    """
    # A dict rather than two nonlocal variables: the worker writes it and the
    # caller reads it after join(), so the assignment has to be visible on an
    # object both frames share. Both keys are pre-set to None so that a case can
    # distinguish "func returned None" from "the worker never ran" by looking at
    # whether the thread finished, rather than by a missing key.
    box = {"result": None, "error": None}

    def target():
        try:
            box["result"] = func(*args)
        except Exception as exc:
            # Typed, never bare (CLAUDE.md 4.5). The point of this helper is to
            # carry an exception back across the thread boundary so a case can
            # assert on its type; letting it die in the worker thread would turn
            # every "must raise" case into a silent pass, since an exception in a
            # thread nobody joins is only printed to stderr.
            box["error"] = exc

    thread = threading.Thread(target=target, name="fake-zenoh-callback")
    thread.start()
    thread.join(WAIT_S)
    # A worker still alive here means func blocked, which no path in this module
    # is allowed to do; reporting it as a failed join names the cause directly.
    assert not thread.is_alive(), "off-loop thread did not finish"
    return box


# ---------------------------------------------------------------------------
# The criterion: three declarations survive collection, close undeclares them.
# ---------------------------------------------------------------------------

def test_declared_handles_survive_gc_and_close_undeclares_them(loop_thread):
    """gc.collect() must not cost a subscription; close() must end all three.

    This is INF-ZN-2's criterion verbatim, and it is the case the whole item
    exists for: a registry that forgets its handles produces a process that
    starts cleanly, holds an open session, and receives nothing on any key.

    Read it together with test_a_local_variable_loses_the_subscription, which
    proves the fake WOULD have reported an undeclare had one happened. On its
    own, "undeclare == 0" is satisfied by a fake that never undeclares anything,
    and this project has shipped that shape before.

    Turned red by: handles stored in a function local (the criterion's mutation).
    """
    log = DeclareLog()
    session = FakeSession(log)
    # The bus is built on the fixture's loop rather than on a loop of this
    # case's own, so that the registry under test is wired exactly as it is in
    # the delivery cases below. A registry whose bus pointed at a loop nobody
    # runs would still pass every counting assertion here, which is worth knowing
    # when reading this case: it establishes ownership, not delivery.
    bus = EventBus(loop_thread.loop)
    registry = SubscriberRegistry(bus)

    # A fresh Collector per key, and the return value of declare() deliberately
    # ignored -- declare() returns None precisely so that a caller cannot end up
    # as a second owner of the handle.
    for key in THREE_KEYS:
        registry.declare(session, key, Collector())

    # Every handle is unreferenced by the caller at this point: declare() returns
    # None precisely so that it is, and the only thing keeping the subscriptions
    # alive is registry._handles. That is the state a real process spends its
    # whole life in.
    gc.collect()

    # The declarations reached the session at all, and reached it once per key.
    # Without this, a registry that quietly skipped declare_subscriber would pass
    # every remaining assertion in this case: zero handles, zero undeclares.
    assert log.declared == list(THREE_KEYS)
    # The criterion's first half. The failure message prints WHICH subscription
    # was lost, because with three keys "0 != 1" alone sends the reader back to
    # the file to work out which one.
    assert log.undeclare_calls == 0, (
        "a subscription was undeclared by collection: %r" % (log.undeclared,))
    assert registry.alive() == len(THREE_KEYS)

    registry.close()

    # The criterion's second half.
    assert log.undeclare_calls == len(THREE_KEYS)
    # Cause, not just count. A registry that dropped its handles and then
    # reported a clean close() would also reach a total of three, and the
    # difference between the two situations is the entire subject of this file.
    assert log.causes() == ["explicit"] * len(THREE_KEYS)
    # And the container really is empty afterwards, so a later declare() cannot
    # find a stale handle and a second close() has nothing to repeat.
    assert registry.alive() == 0


def test_close_is_idempotent(loop_thread):
    """A second close() must not undeclare anything a second time.

    Shutdown paths get called twice in practice -- once by the owner, once by a
    signal handler or a context manager -- and on a real Subscriber the second
    undeclare is a Rust-side call on a handle that has already been consumed.
    Idempotence here is not politeness; it is what keeps an orderly shutdown from
    ending in an exception that hides whatever caused the shutdown.

    Turned red by: close() iterating over the container without emptying it.
    """
    log = DeclareLog()
    session = FakeSession(log)
    registry = SubscriberRegistry(EventBus(loop_thread.loop))
    for key in THREE_KEYS:
        registry.declare(session, key, Collector())

    # Twice in a row, with nothing in between. The realistic sequence has work
    # between the two calls, but that only adds ways for the second call to be
    # reached and none for it to behave differently -- close() takes no argument
    # and reads no state other than its own container.
    registry.close()
    registry.close()

    # Still three, not six. The fake's own double-count guard cannot mask this:
    # it suppresses a second record for the SAME subscriber, which is exactly
    # what the mutation produces, so the count would still be three if the guard
    # were doing the work -- hence the alive() assertion below, which the
    # mutation also fails.
    assert log.undeclare_calls == len(THREE_KEYS)
    assert registry.alive() == 0


def test_declare_after_close_raises(loop_thread):
    """A subscription created after close() would never be undeclared.

    It would also start delivering into a loop the owner is in the middle of
    shutting down, and the samples would look entirely normal on arrival -- there
    is no field in a sample that says "your process is stopping". So declare()
    raises rather than quietly succeeding.

    Turned red by: moving the closed check after the declaration.
    """
    log = DeclareLog()
    session = FakeSession(log)
    registry = SubscriberRegistry(EventBus(loop_thread.loop))
    # Closed while empty, on purpose. Closing a registry that never held anything
    # is the harder case for the implementation: there is no handle whose absence
    # could accidentally make the later declare() fail, so the refusal below can
    # only come from the closed flag itself.
    registry.close()

    # The specific type, not XbrainError. A caller doing orderly shutdown needs
    # to tell "you are too late" apart from every other failure the family
    # carries, and a broad catch here would pass even if declare() had raised for
    # an unrelated reason such as a missing session method.
    with pytest.raises(RegistryClosedError):
        registry.declare(session, THREE_KEYS[0], Collector())

    # Nothing was declared on the way to the exception. Without this assertion an
    # implementation that declared first and checked afterwards would leave a
    # live subscription behind and still satisfy the raises() above -- the worst
    # possible outcome, since the caller now believes nothing happened.
    assert log.declared == []


# ---------------------------------------------------------------------------
# The mutation named in the criterion, kept as a permanent case.
# ---------------------------------------------------------------------------

def test_a_local_variable_loses_the_subscription(loop_thread):
    """Storage in a function local => the handle is collected => undeclare > 0.

    This is the mutation INF-ZN-2 names, written out as a permanent case rather
    than performed once by hand, because it is what makes every "undeclare == 0"
    assertion in this file falsifiable. It is also, verbatim, the third of the
    three broken shapes CLAUDE.md 4.3 lists: def foo(): sub = declare_subscriber.

    What it proves is a property of the FAKE, not of the module under test: that
    dropping a handle really does record an undeclare here. Everything else in
    this file leans on that. If someone removes FakeSubscriber.__del__, or makes
    DeclareLog hold its subscribers strongly, this case is the only one that goes
    red -- and both of those were injected and confirmed.
    """
    log = DeclareLog()
    session = FakeSession(log)

    # No SubscriberRegistry in this case at all. The mutant is written out as a
    # plain function rather than as a subclass overriding declare(), because a
    # subclass would still inherit __init__ and its container, and the reader
    # would have to work out which of the two storages the handles ended up in.
    def declare_into_a_local(key):
        # The mutant body of SubscriberRegistry.declare: identical to the real
        # one except that the container is a local instead of self._handles. It
        # dies at return, and with it the subscription.
        #
        # Worth noting for whoever writes the static scan in CFG-DC-3: these two
        # lines are a genuine defect that a line-based rule would call clean,
        # because the call site does land in a list.append -- one of the three
        # accepted shapes. Only the container's lifetime distinguishes it from
        # the correct code, and lifetime is not visible to a grep. The scan is
        # therefore a floor, not a proof, and this case is what actually holds
        # the property.
        handles = []
        handles.append(session.declare_subscriber(key, lambda sample: None))

    for key in THREE_KEYS:
        declare_into_a_local(key)

    # CPython would collect these on the refcount alone, before this line. The
    # explicit collect is here because the criterion names it, and because a
    # build with a different collector must still reach the same conclusion --
    # the assertion below has to hold for a reason stronger than refcounting.
    gc.collect()

    assert log.undeclare_calls == len(THREE_KEYS)
    # All three ended by collection, not because anybody asked. That is the
    # silent failure in full: the process still runs, the session is still open,
    # every subscription is gone, and nothing anywhere says so.
    assert log.causes() == ["gc"] * len(THREE_KEYS)


# ---------------------------------------------------------------------------
# The cross-thread half.
#
# Everything below rests on one distinction that is easy to lose: a message
# ARRIVING and a message arriving ON THE RIGHT THREAD are different properties,
# and only the first one is visible to a consumer. An implementation that called
# the handler inline on the Zenoh thread would deliver every sample, in order,
# with the right payload, and would corrupt loop state on whatever schedule the
# handler happened to touch it. That is why every case here asserts the thread id
# as well as the payload, and why Collector records ids in a set: two threads
# leave a set of two, whereas a single recorded value would show only the last
# one and could easily be the right one by luck.
# ---------------------------------------------------------------------------

def test_on_message_from_a_foreign_thread_lands_on_the_loop(loop_thread):
    """A sample injected off-loop is handled ON the loop thread, and nowhere else.

    Both halves are asserted and they catch different defects. That the payload
    arrives at all shows the hop happened; that the recorded thread id is the
    loop's shows it happened THROUGH the loop rather than by the handler being
    run inline on the caller's thread. A registry that skipped publish_threadsafe
    and called the handler directly would deliver every message correctly, on the
    wrong thread, and this file's other cases would not notice.

    Turned red by: on_message calling bus.publish instead of publish_threadsafe.
    """
    log = DeclareLog()
    session = FakeSession(log)
    bus = EventBus(loop_thread.loop)
    registry = SubscriberRegistry(bus)
    collector = Collector()
    registry.declare(session, THREE_KEYS[0], collector)

    # A bare object() rather than a string or a number: it has no equal-but-not-
    # identical twin, so the identity assertion below cannot pass by accident.
    sample = object()
    # on_message is called directly rather than through a subscriber, because
    # this case is about the hop and not about the wiring -- the wiring has its
    # own case below. Calling it directly also means the assertion holds for any
    # future caller of on_message, not only for one reached through the fake.
    box = run_off_loop(registry.on_message, THREE_KEYS[0], sample)

    # The callback thread itself must see no error. An exception raised there
    # would surface in the Rust runtime's own log if anywhere at all, which in
    # practice means nowhere anybody reads.
    assert box["error"] is None
    assert collector.done.wait(WAIT_S), "sample never reached the loop"
    assert collector.payloads == [sample]
    # Identity, not equality: the handler must receive the very object the
    # callback was handed, not a copy, not a repr, not a re-serialised form.
    assert collector.payloads[0] is sample
    # Exactly one thread ran the handler, and it was the loop's. A set rather
    # than a single value, so a handler that ran on two threads shows up as a set
    # of two rather than as whichever id happened to be recorded last.
    assert collector.threads == {loop_thread.ident}


def test_every_sample_from_the_zenoh_callback_is_delivered(loop_thread):
    """Send SAMPLE_COUNT through the declared callback, receive all, in order.

    Delivery is driven through the handle the registry is holding, which makes
    this the unit-level form of CFG-CM-17's "send 100, receive 100": under the
    local-variable mutation the handle has been collected by the time this runs
    and there is nothing left to deliver through, so the case fails for exactly
    the reason the integration one would.

    Order is asserted as well as count, because call_soon_threadsafe preserves
    the order of calls made from a single thread. A reordering here would mean
    something else is queueing on the way in, which would matter a great deal for
    RT keys whose subscriber-side discipline 11 S2.4.2 freezes.

    Turned red by: the local-variable mutation, and by on_message publishing
    directly.
    """
    log = DeclareLog()
    session = FakeSession(log)
    registry = SubscriberRegistry(EventBus(loop_thread.loop))
    collector = Collector(expected=SAMPLE_COUNT)
    registry.declare(session, THREE_KEYS[0], collector)

    # Reached through the weak reference the log kept, so the registry remains
    # the only strong owner -- exactly as it is in a running process. Resolving
    # the weakref does create a strong reference for the rest of this case, which
    # is fine here: this case is about delivery, not about collection.
    subscriber = log.subscriber_refs[0]()
    # A None here is not "the weakref expired, therefore the registry is broken"
    # -- it is "there is nothing to send through, so the rest of this case cannot
    # run". The message says so, because a bare assertion on a weakref invites
    # exactly the liveness-as-criterion reading this file avoids everywhere else.
    assert subscriber is not None, "the registry did not keep its handle"

    # Integers as payloads here, unlike the identity case above, because what
    # this case checks is ORDER: 0..99 makes a reordering readable in the failure
    # output, whereas a hundred anonymous object() reprs would not be.
    def push_all():
        for index in range(SAMPLE_COUNT):
            subscriber.deliver(index)

    box = run_off_loop(push_all)
    assert box["error"] is None
    assert collector.done.wait(WAIT_S), "not every sample reached the loop"
    # The whole list, not its length. A count alone would pass for a delivery
    # path that duplicated one sample and dropped another.
    assert collector.payloads == list(range(SAMPLE_COUNT))
    assert collector.threads == {loop_thread.ident}


def test_declare_registers_the_handler_before_the_subscriber(loop_thread):
    """A sample arriving during declare_subscriber must not be dropped.

    The window is real: the Rust side can deliver before the caller's next
    statement runs. If declare() registered the handler after declaring the
    subscriber, this sample would find an empty handler list and disappear with
    no error anywhere -- and it would only ever happen on a busy bus, which is to
    say never in a lab and often in the field. So the case injects a sample from
    inside declare_subscriber, where a quiet bus would never produce one.

    Turned red by: reversing the two statements in declare().
    """
    log = DeclareLog()
    sample = object()
    # The only case that uses sample_on_declare. It is not a default on
    # FakeSession because delivering during declaration is the unusual event; a
    # fake that always did it would make every other case depend on behaviour
    # that only one of them is about.
    session = FakeSession(log, sample_on_declare=sample)
    registry = SubscriberRegistry(EventBus(loop_thread.loop))
    collector = Collector()

    # One statement, and the whole case is inside it: the sample is delivered
    # from within this call, before it returns.
    registry.declare(session, THREE_KEYS[0], collector)

    assert collector.done.wait(WAIT_S), "the sample delivered during declare was lost"
    assert collector.payloads == [sample]


# ---------------------------------------------------------------------------
# The forbidden shapes must be loud, not silent.
#
# CLAUDE.md 4.2 lists four things a Zenoh callback may not do. Two of them are
# checkable at run time from inside this package and are checked below: a direct
# publish, and a coroutine handler that would be dispatched into nothing. The
# other two -- an await, and an asyncio queue put_nowait -- are not reachable
# through this module's surface at all, since neither goes through the bus; the
# static scan in CFG-DC-3 is what covers those, and pretending otherwise here
# would be a case that passes because it cannot fail.
#
# The pairing rule for this section: every "must raise" case is accompanied by an
# assertion that nothing was delivered, and the guard has a positive case of its
# own. Without the first, an implementation that raised after acting would pass;
# without the second, a guard that always raised would pass everything here while
# making the bus useless.
# ---------------------------------------------------------------------------

def test_direct_publish_from_a_foreign_thread_raises(loop_thread):
    """CLAUDE.md 4.2 forbids a direct event_bus.publish inside a callback.

    A prohibition with no enforcement is advice, and advice is what this project
    keeps finding at the bottom of an incident. The guard makes the forbidden
    call fail on its first use, on the thread that made it, instead of leaving
    loop structures touched from two threads and the damage to surface later
    somewhere unrelated.

    Turned red by: neutering the affinity guard in _require_loop_thread.
    """
    # No registry in this case: publish() is a bus method and the defect is a bus
    # defect. Going through a registry would add a second thing that could be
    # wrong without making the assertion any stronger.
    bus = EventBus(loop_thread.loop)
    collector = Collector()
    # A subscriber is registered even though nothing should reach it. That is the
    # point: with no handler at all, "nothing was delivered" would be true by
    # construction and the second assertion below would prove nothing.
    bus.subscribe(THREE_KEYS[0], collector)

    box = run_off_loop(bus.publish, THREE_KEYS[0], object())

    # The specific type, not just "something raised". A caller has to be able to
    # distinguish this from a handler that failed, and catching the wrong one is
    # how a guard ends up being caught and ignored by the code it guards.
    assert isinstance(box["error"], ThreadAffinityError)
    # And the message did not sneak through anyway. Without this the case would
    # still pass for an implementation that raised AFTER dispatching, which is
    # the worst of both worlds: the off-loop delivery happens and the caller is
    # told it did not.
    loop_thread.drain()
    assert collector.payloads == []


def test_publish_on_the_loop_thread_is_allowed(loop_thread):
    """The positive half of the same guard: an on-loop publish must work.

    Required by CLAUDE.md 3.3, and by 3.2's second form. A guard that raised
    unconditionally would pass every "must raise" case in this file while making
    the bus useless, and the way that defect usually surfaces is not as a test
    failure -- it is as the guard being deleted months later by someone who found
    it broken and had no case telling them what it was for.

    Turned red by: making the affinity guard fire unconditionally.
    """
    bus = EventBus(loop_thread.loop)
    collector = Collector()
    bus.subscribe(THREE_KEYS[0], collector)
    sample = object()

    # Note what this case does NOT do: it never calls publish() from the test
    # thread expecting success. There is no such thing as a legal off-loop
    # publish, so a case shaped that way would be asserting the opposite of the
    # rule. The only way to exercise the permitted path is to get onto the loop
    # thread first, which is what the next line is for.

    # Run publish() ON the loop thread by scheduling it there.
    # call_soon_threadsafe is the only way to reach that thread from here, and it
    # is one of the two mechanisms CLAUDE.md 4.2 permits.
    loop_thread.loop.call_soon_threadsafe(bus.publish, THREE_KEYS[0], sample)

    assert collector.done.wait(WAIT_S), "on-loop publish delivered nothing"
    assert collector.payloads == [sample]
    assert collector.threads == {loop_thread.ident}


def test_create_task_in_a_callback_raises(loop_thread):
    """The naive implementation CLAUDE.md 4.2 bans must fail loudly, not quietly.

    asyncio.create_task off-loop raises RuntimeError, and this case pins that
    behaviour: whoever writes the forbidden line finds out at once. What it must
    NOT be is silent. So the case also asserts that nothing was delivered -- a
    version that appeared to work would cost far more than one that raises,
    because it would be adopted everywhere before anyone noticed the messages it
    loses under load.

    This case tests the language, not our module, and that is deliberate: the
    rule in 4.2 rests on this behaviour, so a Python release that changed it
    would invalidate the rule and should be caught here rather than in the field.
    """
    bus = EventBus(loop_thread.loop)
    collector = Collector()
    bus.subscribe(THREE_KEYS[0], collector)

    # The coroutine a naive implementation would schedule. Its body publishes,
    # so that if create_task ever DID succeed from a foreign thread, the delivery
    # assertion at the end of this case would catch it rather than the case
    # passing on a technicality.
    async def deliver_later():
        # Never awaited, on purpose; it exists only to be handed to create_task.
        bus.publish(THREE_KEYS[0], object())

    coro = deliver_later()
    try:
        box = run_off_loop(asyncio.create_task, coro)
        assert isinstance(box["error"], RuntimeError)
        # Not our own error type. This one comes from asyncio itself, which is
        # the point: the forbidden call cannot even be made from that thread, so
        # it never reaches any guard of ours.
        assert not isinstance(box["error"], ThreadAffinityError)
    finally:
        # Closing the coroutine keeps a "never awaited" RuntimeWarning out of the
        # run. A warning attached to a passing case is noise, and noise attached
        # to passing cases is how real warnings come to be ignored.
        coro.close()

    loop_thread.drain()
    assert collector.payloads == []


def test_a_coroutine_handler_is_rejected(loop_thread):
    """An async def handler would be called and its coroutine dropped.

    No exception, no delivery, at most a warning nobody reads -- the fail-silent
    shape this module exists to close. Registration refuses it, and refuses it
    before the subscriber is declared: otherwise the process would be left
    holding a live subscription whose handler can never run, which is strictly
    worse than having no subscription at all.

    Turned red by: removing the iscoroutinefunction check from subscribe().
    """
    log = DeclareLog()
    session = FakeSession(log)
    registry = SubscriberRegistry(EventBus(loop_thread.loop))

    # Driven through registry.declare() rather than bus.subscribe() directly,
    # because declare() is where a consumer meets this rule and because the
    # ordering claim -- rejected BEFORE the subscriber exists -- can only be
    # checked on the path that declares one.
    async def async_handler(payload):
        return payload

    with pytest.raises(HandlerContractError):
        registry.declare(session, THREE_KEYS[0], async_handler)

    # No subscription and no handle. Both are asserted because they fail
    # separately: declaring then raising leaves the first, and appending then
    # raising leaves the second.
    assert log.declared == []
    assert registry.alive() == 0


def test_a_failing_handler_is_not_swallowed(loop_thread):
    """A handler that raises must be reported, on both delivery paths.

    Wrapping dispatch in except-and-continue is the change a reviewer will
    suggest in the name of robustness. It makes a subscriber that has been broken
    for a week look exactly like one with nothing to do, which is the direction
    this project keeps closing off. The cost of not swallowing is real and is
    stated in event_bus._dispatch: later handlers for that one message do not
    run. That is a price worth paying once, loudly.

    Turned red by: wrapping the handler call in except-Exception-pass.
    """
    bus = EventBus(loop_thread.loop)

    # ValueError rather than a project exception type. It has to be something the
    # bus has no reason to recognise: if the handler raised XbrainError, a bus
    # that special-cased its own family could swallow it and this case would
    # still pass on the ValueError it never saw.
    def broken(payload):
        raise ValueError("handler failed on purpose")

    bus.subscribe(THREE_KEYS[0], broken)

    # Path one, on-loop: the exception must reach the caller of publish(). The
    # whole sequence has to run on the loop thread, so the call is scheduled
    # there and the result carried back through a box and an event.
    done = threading.Event()
    box = {"error": None}

    def on_loop():
        try:
            bus.publish(THREE_KEYS[0], object())
        except ValueError as exc:
            # The handler's own exception type, not the bus's. Catching
            # XbrainError here would pass even if the bus had replaced the
            # handler's failure with one of its own, which would lose the
            # traceback that names the broken subscriber.
            box["error"] = exc
        finally:
            done.set()

    loop_thread.loop.call_soon_threadsafe(on_loop)
    assert done.wait(WAIT_S)
    assert isinstance(box["error"], ValueError)

    # Path two, threadsafe: nobody is left to catch it, so it must reach the
    # loop's exception handler. Asserting on that is the difference between
    # "loud" and "raised into a void" -- and the void is what an unobserved
    # exception in a loop callback would otherwise be.
    #
    # The publish is made from a foreign thread, which is the only way the
    # threadsafe path is reached in production. The return value is discarded on
    # purpose: publish_threadsafe hands the work to the loop and returns before
    # the handler runs, so there is nothing to assert on it -- the evidence
    # arrives later, in loop_errors, which is why drain() has to come first.
    run_off_loop(bus.publish_threadsafe, THREE_KEYS[0], object())
    loop_thread.drain()
    assert any(isinstance(ctx.get("exception"), ValueError)
               for ctx in loop_thread.loop_errors), (
        "a handler exception vanished on the publish_threadsafe path")
