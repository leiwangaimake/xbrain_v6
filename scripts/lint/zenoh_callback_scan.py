#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: zenoh_callback_scan.py
Brief: Static scan for loop-affine work inside a Zenoh subscriber callback

Description:
What problem this solves. CLAUDE.md 4.2 states that a zenoh-python subscriber
callback runs on a Rust-owned thread, never on the process event loop, and then
lists four things such a callback may NOT do: asyncio.create_task, an asyncio
Queue put_nowait, any await, and a direct event_bus.publish. The reason those
are forbidden is not style. asyncio structures are not thread-safe, and the way
they fail from a foreign thread is not a crash: work queued into a loop that is
asleep in epoll and was never woken for it is late, or lost, or interleaved with
half-updated loop state. Every one of those reads downstream as a flaky sensor
or a flaky link, and the search starts at the sensor, days from the real cause.
The runtime half of the rule is already enforced -- EventBus.publish raises off
the loop thread and rejects coroutine handlers -- but a callback that reaches the
loop WITHOUT going through the bus (a create_task, a queue put_nowait, an await
in an async callback) never touches any guard we own. This script is the static
half: it reads the source, finds the function each declare_subscriber hands to
the Rust runtime, and fails if that function's own body contains one of the four.

Where the design comes from:
  * CLAUDE.md 4.2 verbatim, for the four forbidden operations and for the two
    permitted mechanisms (event_bus.publish_threadsafe and
    loop.call_soon_threadsafe). That is the whole authority; grep 10, 11 and 20
    for publish_threadsafe and it appears in no volume.
  * 11 S2.4.8 for the anti-pattern posture the deployment self-check takes -- a
    forbidden shape means the process refuses to start. This lint is the
    source-level companion to that runtime check, not a substitute for it: the
    A-7 "publisher without explicit QoS" self-check named by CFG-CM-17's third
    mutation is a running-process check and belongs to INF-ZN-5 / INF-ZN-6, which
    is why it is NOT attempted here (see the note in xbrain/common/zenoh).
  * The zenoh-python subscriber API, Session.declare_subscriber(key_expr,
    handler=None, ...): the handler is the second positional argument or the
    keyword handler, and that is the argument this scan follows to its function.

Why a FULL-TEXT scan of whole trees, not a diff. CLAUDE.md 3.2 form 5 (a lint
that runs over the diff and not the file) is a failure this project has measured:
a self-check reported "no unescaped pipes" after reading thirteen added lines
while the offending line sat outside the diff. A diff-scoped version of this
check would go green on every pull request that does not touch the offending
callback, which is all of them after the one that introduced it. So the surface
is every .py file in the runtime Python trees, every run.

How a callback is identified, and the honest limits of that. For each
declare_subscriber call this scan resolves the handler argument to a function
definition IN THE SAME FILE, following three shapes: a bare name or a method
attribute (self.on_message), a functools.partial wrapping one of those, and a
local variable assigned from one of those inside the same function -- which is
exactly the shape SubscriberRegistry.declare uses, so the registry's on_message
is reached and scanned. A lambda handler is scanned in place. What it CANNOT
follow is a callback passed in as a parameter, imported from another module,
stored on an object elsewhere, or selected dynamically; those are reported as
UNRESOLVED rather than silently treated as clean, because "found nothing to
scan" and "scanned something and it was clean" must not print as the same zero
(CLAUDE.md 3.2 form 1). Only the callback's OWN statements are scanned: the scan
does not descend into a nested function or a coroutine the callback defines,
because those run in a different scope -- a create_task IS in the callback body,
but the coroutine it schedules is not, and flagging the latter would make the
check red on the correct shape.

What this script deliberately does NOT do:
  * It does not enforce the strong-reference rule (the declare_subscriber return
    value landing in self.x, a list append, or SubscriberRegistry.declare). That
    is CLAUDE.md 8.2 and CFG-DC-3, a different check with a different mutation.
  * It does not check QoS, the key table, or the cross-plane whitelist. Those are
    INF-ZN-3, INF-ZN-5 and INF-ZN-7.
  * It does not run the process. The A-7 QoS self-check and the A-1 thread/pub
    binding self-check are runtime checks over live declarations; a source scan
    cannot see which thread a publisher ends up on.
  * It does not scan itself or the tests. Both would be self-harm in the
    CLAUDE.md 3.2 form 3 sense: this file must name the forbidden calls to look
    for them, and a fixture must be free to write a violating callback on
    purpose, so including either tree would make the surface unable to reach zero
    and the repair anyone reaches for is to loosen the criterion.

Traps that look correct and are not:
  * Flagging every asyncio.create_task in the tree. Most are legal: a handler
    that runs ON the loop is expected to call loop.create_task, and the event_bus
    header says exactly that. The op is forbidden only inside the Rust-thread
    callback, which is why this scan resolves callbacks first and reads bodies
    second. A whole-tree grep for create_task would be red on correct code, and a
    check red on correct code gets switched off (CLAUDE.md 3.2 form 2).
  * Matching publish by text. publish_threadsafe contains publish; a substring
    match would flag the one call the rule REQUIRES. The match is on the AST
    attribute name being exactly publish, so publish_threadsafe never matches.
  * Descending into a coroutine the callback defines and handing to create_task.
    Its body runs on the loop if it ever runs at all; the defect is the
    create_task, not the publish inside the coroutine. The scan stops at nested
    scope boundaries so the coroutine's body is out of surface.
"""

import ast
import os
import sys
from typing import Dict, List, NamedTuple, Optional, Tuple

ROOT = "/opt/xbrain_v6"

#: The scan surface: the Python runtime trees. A zenoh-python callback and the
#: asyncio operations it must not perform are Python, so C++ extensions are not
#: in scope here (the C++ side of the same isolation is a separate concern).
#: scripts/ and tests/ are deliberately absent -- see the Description and the
#: metatest in tests/common/test_zenoh_callback_scan.py, which pins that absence
#: rather than leaving it to whoever next edits this tuple.
SCAN_DIRS = ("xbrain", "ros2_ws", "services")

#: Directories never worth reading. Build outputs hold generated copies of
#: sources, and a hit reported in one sends the reader to a file that will be
#: overwritten instead of to the source that produced it. model* trees are
#: vendored ASR payloads, hundreds of megabytes with no source in them.
SKIP_DIRS = {"__pycache__", ".git", "node_modules", "build", "install", "log"}


class Banned(NamedTuple):
    """One forbidden operation inside a Rust-thread callback."""

    name: str  # what the report prints, in the spelling a human greps for
    why: str   # what breaks if this one runs off-loop, in one sentence


# The four operations CLAUDE.md 4.2 forbids inside a subscriber callback, plus
# the async-callback shape that is the same defect one level up: a coroutine the
# runtime calls and then drops unrun. Each carries the consequence rather than a
# rule number, because the report should teach at the point of failure -- the
# person who wrote the line is the person reading it.
BANNED_CREATE_TASK = Banned(
    "asyncio.create_task",
    "schedules a coroutine on the loop from a foreign thread; create_task is not "
    "thread-safe and raises off-loop, and the work never runs")
BANNED_PUT_NOWAIT = Banned(
    "asyncio.Queue.put_nowait",
    "an asyncio queue is not thread-safe; a put from the Rust thread lands in a "
    "loop that was never woken for it, so the item is seen late or not at all")
BANNED_AWAIT = Banned(
    "await",
    "the callback is a coroutine and the Rust runtime does not await it; the "
    "await never runs and the returned coroutine is discarded")
BANNED_PUBLISH = Banned(
    "direct event_bus.publish",
    "publish is loop-affine and raises off the loop thread; the callback must "
    "use publish_threadsafe (CLAUDE.md 4.2)")
BANNED_ASYNC_DEF = Banned(
    "async def callback",
    "a coroutine function handed to declare_subscriber is called by the runtime, "
    "returns a coroutine, and that coroutine is dropped unrun -- no delivery, no "
    "error")


# Scope-defining nodes. The scan must not cross INTO one of these from an outer
# body: a nested function or a lambda is a different scope that does not run on
# the Rust thread, so an operation inside it is not the callback's operation.
_SCOPE_NODES = (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)


def _walk_no_nested(roots):
    """Yield roots and their descendants without crossing a nested scope.

    The starting roots are statements already inside one scope (a function body,
    a lambda expression, or the module body). A nested scope node encountered
    along the way is yielded -- so a caller can notice it -- but its children are
    not visited, because they belong to a different scope. This is what keeps the
    banned-op scan on the callback's own statements and off the body of a
    coroutine the callback happens to define.
    """
    stack = list(roots)
    while stack:
        node = stack.pop()
        yield node
        # Do not descend through a nested def or lambda into its body. Its
        # statements run in their own scope, not on the Rust callback thread.
        if isinstance(node, _SCOPE_NODES):
            continue
        stack.extend(ast.iter_child_nodes(node))


def _defs_by_name(module: ast.AST) -> Dict[str, List[ast.AST]]:
    """Every function and method in the file, indexed by its bare name.

    A full walk, so a method defined in any class and a module-level function are
    both reachable. Resolution by bare name is what lets self.on_message be
    matched to the method named on_message; it is approximate (two methods in two
    classes can share a name), and the approximation is toward scanning MORE
    rather than fewer bodies, which is the safe direction for a defect check.
    """
    out: Dict[str, List[ast.AST]] = {}
    for node in ast.walk(module):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out.setdefault(node.name, []).append(node)
    return out


def _local_assigns(roots) -> Dict[str, ast.AST]:
    """name -> assigned expression, for simple-name targets in one scope.

    Collected without crossing into nested scopes, so an assignment inside a
    coroutine the function defines does not shadow the function's own. This is
    the map that resolves the callback local in SubscriberRegistry.declare:
    callback = functools.partial(self.on_message, key_expr).
    """
    out: Dict[str, ast.AST] = {}
    for node in _walk_no_nested(roots):
        if isinstance(node, ast.Assign):
            # Only simple Name targets. A tuple- or attribute-target assignment
            # is not a callback binding this scan claims to resolve, and pinning
            # its meaning would be guessing.
            for target in node.targets:
                if isinstance(target, ast.Name):
                    out[target.id] = node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            if isinstance(node.target, ast.Name):
                out[node.target.id] = node.value
    return out


def _is_partial(call: ast.Call) -> bool:
    """True if this call is functools.partial(...) or partial(...).

    Matched by the callee name only. The point of a partial handler is that the
    real callback is its first argument, so resolving that argument is what
    reaches the function whose body must be scanned.
    """
    func = call.func
    if isinstance(func, ast.Attribute):
        return func.attr == "partial"
    if isinstance(func, ast.Name):
        return func.id == "partial"
    return False


def _resolve_handler(expr, local_assigns, defs_by_name, seen=None):
    """Resolve a handler expression to the function nodes to scan.

    Returns (targets, resolved). targets is a list of FunctionDef /
    AsyncFunctionDef / Lambda nodes. resolved is False when the expression could
    not be followed to a definition in this file -- reported as a NOTE, never as
    a clean result, so an unreadable callback cannot pass as a scanned one.
    """
    # Guard against a self-referential assignment (x = x) sending this into a
    # loop. Identity of the visited nodes is enough; the graph is tiny.
    if seen is None:
        seen = set()
    if id(expr) in seen:
        return [], False
    seen.add(id(expr))

    # A lambda handler is scanned in place: its body IS the callback.
    if isinstance(expr, ast.Lambda):
        return [expr], True

    # A bare name: first a local binding in the same scope (the registry's
    # callback variable), then a function or method defined in this file.
    if isinstance(expr, ast.Name):
        if expr.id in local_assigns:
            return _resolve_handler(local_assigns[expr.id], local_assigns,
                                    defs_by_name, seen)
        if expr.id in defs_by_name:
            return list(defs_by_name[expr.id]), True
        return [], False

    # An attribute (self.on_message, obj.cb): matched to a method by its bare
    # name. The receiver is ignored on purpose -- within one file the method name
    # is the reliable part, and following the receiver's type is beyond a source
    # scan.
    if isinstance(expr, ast.Attribute):
        if expr.attr in defs_by_name:
            return list(defs_by_name[expr.attr]), True
        return [], False

    # functools.partial(cb, ...): the callback is the first positional argument.
    if isinstance(expr, ast.Call) and _is_partial(expr):
        if expr.args:
            return _resolve_handler(expr.args[0], local_assigns,
                                    defs_by_name, seen)
        return [], False

    # Anything else (a call returning a callable, a subscript, a conditional
    # expression) cannot be followed statically.
    return [], False


def _handler_arg(call: ast.Call) -> Optional[ast.AST]:
    """The handler expression of a declare_subscriber call, or None.

    zenoh-python: declare_subscriber(key_expr, handler=None, ...). The handler is
    the second positional argument or the keyword handler. A call with only the
    key and no handler is pull-mode -- there is no callback to scan, and reading
    one into existence would invent a violation.
    """
    if len(call.args) >= 2:
        return call.args[1]
    for kw in call.keywords:
        if kw.arg == "handler":
            return kw.value
    return None


def _scan_body(target: ast.AST) -> List[Tuple[int, Banned]]:
    """(lineno, Banned) for every forbidden operation in one callback body.

    target is a FunctionDef, AsyncFunctionDef or Lambda. Only the target's own
    statements are read; nested scopes are not entered (see _walk_no_nested).
    """
    hits: List[Tuple[int, Banned]] = []

    # An async callback is broken by its shape alone: the runtime calls it, gets
    # a coroutine, and drops it. Reported once, at the def, in addition to any
    # await inside -- both facts are true and both point at the same fix.
    if isinstance(target, ast.AsyncFunctionDef):
        hits.append((target.lineno, BANNED_ASYNC_DEF))

    # Roots: the body statements of a def, or the single expression of a lambda.
    if isinstance(target, ast.Lambda):
        roots = [target.body]
    else:
        roots = list(target.body)

    for node in _walk_no_nested(roots):
        # An await can only appear in an async callback, and it is the operation
        # the runtime never drives. Reported at its own line for specificity.
        if isinstance(node, ast.Await):
            hits.append((node.lineno, BANNED_AWAIT))
            continue
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        # create_task and put_nowait are distinctive asyncio names; match them
        # whether reached as an attribute (asyncio.create_task, loop.create_task,
        # queue.put_nowait) or as a bare name imported directly.
        if isinstance(func, ast.Attribute):
            if func.attr == "create_task":
                hits.append((node.lineno, BANNED_CREATE_TASK))
            elif func.attr == "put_nowait":
                hits.append((node.lineno, BANNED_PUT_NOWAIT))
            elif func.attr == "publish":
                # Exactly publish, not publish_threadsafe: an AST attribute name
                # is the whole token, so the permitted call never matches here.
                hits.append((node.lineno, BANNED_PUBLISH))
        elif isinstance(func, ast.Name):
            if func.id == "create_task":
                hits.append((node.lineno, BANNED_CREATE_TASK))
            elif func.id == "put_nowait":
                hits.append((node.lineno, BANNED_PUT_NOWAIT))
    return hits


def scan_source(text: str, shown: str = "<source>"):
    """(violations, notes) for one unit of Python source.

    violations: (shown, lineno, op_name, why) at the forbidden line inside a
      resolved callback body.
    notes: (shown, lineno, detail) for a declare_subscriber whose handler could
      not be resolved to a definition in this file -- reported, not failed, so
      the report never conflates "nothing to scan" with "scanned and clean".
    """
    violations: List[Tuple[str, int, str, str]] = []
    notes: List[Tuple[str, int, str]] = []
    try:
        module = ast.parse(text)
    except SyntaxError as exc:
        # A file that does not parse is a problem for some other gate, not this
        # one. Failing the whole lint on it would take the check down over an
        # unrelated defect, so it is a note and the scan continues.
        notes.append((shown, getattr(exc, "lineno", 0) or 0,
                      "could not parse: %s" % exc.msg))
        return violations, notes

    defs_by_name = _defs_by_name(module)

    # Each scope contributes its own local assignments and its own
    # declare_subscriber calls. A call is attributed to the innermost scope that
    # contains it, because _walk_no_nested over an outer body stops at the nested
    # def and never reaches the call there.
    scopes: List[ast.AST] = [module]
    for node in ast.walk(module):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            scopes.append(node)

    for scope in scopes:
        # Each scope is analysed against ITS OWN local assignments. The registry's
        # callback variable is local to declare(), so resolving it needs the map
        # built from declare()'s body, not the module's -- computing local per
        # scope is what makes the partial trace work.
        roots = list(scope.body)
        local = _local_assigns(roots)
        for node in _walk_no_nested(roots):
            # Two things are collected in this one walk: the declare_subscriber
            # calls (below) and, already, the local assignments (above). Walking
            # once keeps the two views of the same scope consistent.
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            # declare_subscriber is always an attribute call: session.declare_
            # subscriber(...). A bare name of that spelling would not be the
            # zenoh API and is not what the rule is about.
            if not isinstance(func, ast.Attribute):
                continue
            if func.attr != "declare_subscriber":
                continue
            handler = _handler_arg(node)
            if handler is None:
                # Pull-mode subscriber: no callback thread, nothing to scan.
                continue
            targets, resolved = _resolve_handler(handler, local, defs_by_name)
            if not resolved:
                # NOT a violation and NOT a clean pass: a body that was never read.
                # The note is what stops a callback the scan cannot follow from
                # counting as evidence of safety (CLAUDE.md 3.2 form 1). The line
                # reported is the declare call, since that is what a reader greps.
                notes.append((shown, node.lineno,
                              "declare_subscriber handler could not be resolved "
                              "to a function in this file; its body is NOT "
                              "scanned"))
                continue
            # targets can hold more than one node when a bare name matched several
            # defs of the same name across classes -- every match is scanned, so
            # the conservative direction is "scan more bodies", never fewer.
            for target in targets:
                for lineno, banned in _scan_body(target):
                    violations.append((shown, lineno, banned.name, banned.why))
    return violations, notes


def scan_file(path: str, rel_path: Optional[str] = None):
    """(violations, notes) for one file on disk."""
    shown = rel_path if rel_path else path
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except OSError as exc:
        return [], [(shown, 0, "cannot read: %s" % exc)]
    return scan_source(text, shown)


def _files_under(base: str):
    """Every .py file below one directory, pruning build and vendor trees."""
    for dirpath, dirnames, filenames in os.walk(base):
        # Pruned in place so os.walk does not descend. model* trees hold vendored
        # ASR payloads -- data, not source.
        dirnames[:] = [d for d in dirnames
                       if d not in SKIP_DIRS and not d.startswith("model")]
        for name in filenames:
            if name.endswith(".py"):
                yield os.path.join(dirpath, name)


def iter_sources():
    """Every file in the scan surface.

    An absent directory is skipped here and REPORTED by main(): zero hits from a
    tree that was never read is indistinguishable from zero hits from a clean
    one, and ros2_ws/ holds no Python file today, so this is not hypothetical.
    """
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            continue
        for path in _files_under(base):
            yield path


#: Self-test cases. Each is (name, source, expected_violation_count, why). The
#: table is in three groups and every group is load-bearing:
#:   * violating samples, one per forbidden operation and one per resolution
#:     shape (registry partial, direct method, lambda). Without them a mistyped
#:     matcher would sit here looking authoritative and catch nothing -- the
#:     do-nothing implementation CLAUDE.md 3.2 form 1 warns of.
#:   * compliant samples. These stop the check drifting into "flag anything with
#:     create_task", which would be red on the code the event_bus header
#:     requires and would end with the check switched off (form 2).
#:   * the unresolved sample, which must produce a NOTE and zero violations, so a
#:     callback the scan cannot follow is never counted as clean.
#
# The forbidden spellings appear here as plain source. They are safe: the scan is
# AST-based and only reads bodies reached from a declare_subscriber handler, and
# scripts/ is outside SCAN_DIRS in any case, so this file never scans itself.
_REGISTRY_CLEAN = (
    "import functools\n"
    "class R:\n"
    "    def declare(self, session, key, handler):\n"
    "        cb = functools.partial(self.on_message, key)\n"
    "        self._h.append(session.declare_subscriber(key, cb))\n"
    "    def on_message(self, key, sample):\n"
    "        self._bus.publish_threadsafe(key, sample)\n"
)
_REGISTRY_MUTATED = (
    "import asyncio, functools\n"
    "class R:\n"
    "    def declare(self, session, key, handler):\n"
    "        cb = functools.partial(self.on_message, key)\n"
    "        self._h.append(session.declare_subscriber(key, cb))\n"
    "    def on_message(self, key, sample):\n"
    "        asyncio.create_task(self._deliver(sample))\n"
)

SELF_TEST_CASES: Tuple[Tuple[str, str, int, str], ...] = (
    # Group 1 -- the named mutation and its compliant twin. These two must be
    # read together: the mutation proves the scanner FIRES, the twin proves it
    # DISCRIMINATES. Either alone is satisfied by a broken scanner (one that
    # never fires, or one that always fires), so neither is dropped.
    #
    # *** The mutation CFG-CM-17 names: create_task inside the callback, reached
    # through the registry's functools.partial -> on_message. Resolution path
    # exercised: local var (cb) -> functools.partial -> attribute (self.on_
    # message) -> method def. This is the longest path the resolver walks, and it
    # is the one the real code uses, so it is the one most worth pinning.
    ("registry_create_task.py", _REGISTRY_MUTATED, 1,
     "create_task in on_message, resolved through the registry partial, is the "
     "named mutation and must be flagged"),
    # The twin. Same resolution path, clean body. If this reported a violation the
    # scanner would be matching publish as a substring of publish_threadsafe, or
    # flagging every resolved body -- both are form 2 (red on correct code).
    ("registry_clean.py", _REGISTRY_CLEAN, 0,
     "the same registry with publish_threadsafe is clean; publish_threadsafe is "
     "not publish"),
    # Group 2 -- one violating sample per resolution shape and per forbidden op.
    # Resolution path: bare name (cb) -> module-level def. Op: create_task.
    ("direct_create_task.py",
     "def cb(sample):\n"
     "    import asyncio\n"
     "    asyncio.create_task(handle(sample))\n"
     "session.declare_subscriber(key, cb)\n", 1,
     "create_task in a callback passed directly by name is a violation"),
    # Resolution path: lambda scanned in place (no name to resolve). Op:
    # create_task inside the lambda's single expression.
    ("lambda_create_task.py",
     "session.declare_subscriber(key, lambda s: asyncio.create_task(go(s)))\n", 1,
     "create_task inside a lambda handler is a violation"),
    # Op: put_nowait, the asyncio queue operation that is the quietest of the
    # four -- it neither raises nor logs off-loop, so a static catch is the only
    # catch it gets.
    ("put_nowait.py",
     "def cb(sample):\n"
     "    q.put_nowait(sample)\n"
     "session.declare_subscriber(key, cb)\n", 1,
     "an asyncio queue put_nowait from the callback is a violation"),
    # Op: await, which forces the callback to be async def. Expect TWO hits -- the
    # async-def shape and the await itself -- because both are true and both point
    # a reader at the same fix. The count is pinned so a change that stopped
    # reporting one of them is caught here.
    ("await_callback.py",
     "async def cb(sample):\n"
     "    await sink.write(sample)\n"
     "session.declare_subscriber(key, cb)\n", 2,
     "an async callback is flagged for its shape and again for the await"),
    # Op: direct publish. This is the case that pins the exact-attribute match:
    # bus.publish must fire, and the compliant twin above proves bus.publish_
    # threadsafe does not.
    ("direct_publish.py",
     "def cb(sample):\n"
     "    bus.publish(key, sample)\n"
     "session.declare_subscriber(key, cb)\n", 1,
     "a direct event_bus.publish from the callback is a violation"),
    # Resolution path: keyword handler=cb rather than the second positional. The
    # zenoh API accepts both spellings, so the scanner must follow both or a
    # callback written the keyword way would escape unread.
    ("handler_kw.py",
     "def cb(sample):\n"
     "    asyncio.create_task(go(sample))\n"
     "session.declare_subscriber(key, handler=cb)\n", 1,
     "the handler passed by keyword is followed too"),
    # Group 3 -- compliant counter-examples, one per way the scanner could
    # over-reach. A create_task that is NOT in a callback is legal everywhere on
    # the loop, and a checker that flagged it would be red on correct code.
    ("create_task_not_a_callback.py",
     "async def worker():\n"
     "    asyncio.create_task(go())\n", 0,
     "create_task outside any subscriber callback is legal and must not flag"),
    # The loop-side handler. registry.declare(...) registers a handler that runs
    # ON the loop (via publish_threadsafe), so create_task there is legal. This
    # scan follows declare_subscriber, NOT declare, precisely so it does not flag
    # the loop-side handler and become red on the intended pattern.
    ("loop_side_handler_create_task.py",
     "def on_targets(sample):\n"
     "    loop.create_task(process(sample))\n"
     "registry.declare(session, key, on_targets)\n", 0,
     "a handler registered through SubscriberRegistry.declare runs on the loop; "
     "this scan only follows declare_subscriber, not declare"),
    # Pull mode: declare_subscriber called with only the key, no handler. There is
    # no callback thread, so reading a violation into it would invent one.
    ("pull_mode.py",
     "sub = session.declare_subscriber(key)\n", 0,
     "a pull-mode subscriber has no callback and nothing to scan"),
    # Group 4 -- the unresolved case. A callback this scan cannot follow (here an
    # attribute on an imported module, with no matching def in the file) must
    # produce a NOTE, not a violation and not a silent clean pass. self_test()
    # asserts the note separately, so a scanner that stopped noting is caught.
    ("unresolved.py",
     "session.declare_subscriber(key, some_module.callback)\n", 0,
     "an unresolvable handler produces a note, not a violation and not a clean "
     "pass"),
)


def self_test() -> int:
    """Prove the checker reacts to each case rather than trusting that it does.

    A lint reporting zero on a clean tree is indistinguishable from a lint that
    reports zero on everything. comment_ratio.py in this repository was silently a
    no-op through an entire fix because nothing exercised it; these probes are the
    cheapest defence against a repeat, and the metatest in
    tests/common/test_zenoh_callback_scan.py runs them as part of the suite.
    """
    ok = True
    n_notes = 0
    for name, source, want, why in SELF_TEST_CASES:
        violations, notes = scan_source(source, name)
        got = len(violations)
        if got != want:
            ok = False
        # The unresolved case is the one that must produce a note; count them so
        # a matcher that stopped noting (and started silently passing) is caught.
        if name == "unresolved.py" and not notes:
            ok = False
        n_notes += len(notes)
        print("  %s %-70s want %d got %d"
              % ("ok " if got == want else "FAIL", why, want, got))
    print("")
    print("  notes emitted across cases: %d (the unresolved case must emit one)"
          % n_notes)
    print("self-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def _print_surface() -> None:
    """Print the scan surface with a per-directory file count.

    *** Printed on every run, not only on failure. A tree that is absent, or
    present and empty, contributes zero hits, and zero hits from an unread tree
    is indistinguishable from zero hits from a clean one -- CLAUDE.md 3.2 form 6
    (undeclared scan surface). ros2_ws/ holds no Python file today, which is
    exactly the case that would otherwise let this report read as "three trees
    checked, all clean". The counts are derived here and written down nowhere,
    per CLAUDE.md 3.7.
    """
    print("scan surface: " + ", ".join(SCAN_DIRS))
    for top in SCAN_DIRS:
        base = os.path.join(ROOT, top)
        if not os.path.isdir(base):
            print("  NOTE %s/ does not exist; it contributed nothing to this scan"
                  % top)
            continue
        n = sum(1 for _ in _files_under(base))
        if n == 0:
            print("  NOTE %s/ exists but holds no Python file; it contributed "
                  "nothing to this scan" % top)
        else:
            print("  %-10s %d file(s) scanned" % (top + "/", n))
    print("scan mode: FULL TEXT of every file counted above, on every run --")
    print("  not a diff and not a changed-files list (CLAUDE.md 3.2 form 5)")


def _print_rules() -> None:
    """Print the forbidden operations, so the surface listing is self-describing."""
    print("")
    print("forbidden inside a Zenoh subscriber callback (CLAUDE.md 4.2):")
    for banned in (BANNED_CREATE_TASK, BANNED_PUT_NOWAIT, BANNED_AWAIT,
                   BANNED_PUBLISH, BANNED_ASYNC_DEF):
        print("  %-26s %s" % (banned.name, banned.why))


def _print_criterion() -> None:
    """State what a zero here does and does not establish.

    CLAUDE.md 3.2 form 7 is a conclusion defined into its premise. A green run of
    a source scan is easy to read as "no callback ever touches the loop", which is
    not decidable this way, so the boundary is printed next to the number every
    time rather than living in a document nobody opens alongside the output.
    """
    print("")
    print("criterion: violations == 0")
    print("  This establishes: no resolved subscriber callback body contains a")
    print("  listed operation. It does NOT establish that every callback is safe.")
    print("  Four things it cannot see:")
    print("    1. a callback it could not resolve -- passed in, imported, stored")
    print("       elsewhere, or chosen dynamically. Those are printed as NOTES;")
    print("       a note is a body that was NOT read, not a body that was clean.")
    print("    2. a helper the callback calls. Only the callback's own statements")
    print("       are scanned, not the functions it invokes.")
    print("    3. the runtime anti-patterns. A-7 (publisher without explicit QoS)")
    print("       and A-1 (one thread on Q0/Q1 and Q3) are 11 S2.4.8 checks over a")
    print("       running process; INF-ZN-5 / INF-ZN-6 own them.")
    print("    4. the strong-reference rule for the declare_subscriber handle,")
    print("       which is CFG-DC-3, a separate check.")


def main() -> int:
    if "--self-test" in sys.argv:
        return self_test()
    verbose = "-v" in sys.argv or "--verbose" in sys.argv

    _print_surface()
    _print_rules()
    print("")

    violations: List[Tuple[str, int, str, str]] = []
    notes: List[Tuple[str, int, str]] = []
    for path in sorted(iter_sources()):
        v, n = scan_file(path, os.path.relpath(path, ROOT))
        violations.extend(v)
        notes.extend(n)

    for shown, lineno, name, why in violations:
        print("  BAD  %s:%d  %s -- %s" % (shown, lineno, name, why))

    if notes and verbose:
        print("")
        print("  unresolved / unread callbacks (not failed; see criterion):")
        for shown, lineno, detail in notes:
            print("    %s:%d  %s" % (shown, lineno, detail))

    print("")
    print("  violations: %d" % len(violations))
    print("  notes:      %d%s"
          % (len(notes), "" if verbose else "   (-v to list)"))
    _print_criterion()
    return 1 if violations else 0


if __name__ == "__main__":
    sys.exit(main())
