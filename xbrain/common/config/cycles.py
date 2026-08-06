"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
Shanghai Hachist Intelligent Ship Technology Co., Ltd.
File: cycles.py
Brief: Three-colour DFS over the ${common.*} alias graph, reporting the full cycle

Description:
What problem this solves. The reference axis lets one common.* key alias another
(R-4, chain length at most 3). Aliases can close a loop, and a loader that merely
notices "something is circular" leaves the operator with nothing to act on: the
tree has hundreds of keys and the loop could be anywhere in it. 10 S5.4.3 is
explicit that the error MUST print the whole path, and it gives the exact shape:

    E_CONFIG_INVALID: reference cycle
      common.recording.fence_close_tol_m
        -> common.recording.min_dist_m
        -> common.recording.fence_close_tol_m

That requirement is the same one behind startup assertion J, which insists on
printing absolute paths rather than "a file was missing". A message the reader
cannot act on is close to no message.

Where this sits. It runs BEFORE the freeze line and scans only the common.*
subtree. 10 S5.4.3 states why that is sufficient: L6 references are one-way by
R-2 (a process config may only point at common.*, never at another process), so
a cycle cannot involve L6 at all. Cycles are possible only among the aliases R-4
permits inside common.* itself.

What this module deliberately does NOT do:
  * it does not expand references -- that is refs.resolve
  * it does not check reference SHAPE (R-1, R-2, R-3) -- that is refs.validate_shape
  * it does not merge layers -- that is the overlay axis, which must have finished
    before any of this runs

A shape worth naming. The obvious implementation is to resolve recursively and
catch "I have seen this key before". That does terminate, but the chain it can
report is the path from wherever resolution happened to start, not the cycle --
so the same defect prints a different, longer path depending on which key the
loader happened to visit first. Three-colour DFS reports the cycle itself, which
is stable and is what the contract asks for.
"""

from typing import Any, Dict, List, Optional, Sequence

from .layers import ConfigLayerError
from .merge import flatten

# Node colours of the classic three-colour DFS, named as 10 S5.4.3 names them.
# WHITE  not yet visited
# GREY   on the current DFS stack; meeting one again closes a cycle
# BLACK  fully explored, and known not to lead into a cycle
# The three colours, named as 10 S5.4.3 names them. The invariant each one
# carries is what makes the walk correct, so it is worth stating rather than
# assuming the reader remembers the textbook algorithm:
#
#   WHITE  never visited. Every node starts here, and a WHITE node reached from
#          anywhere is safe to descend into.
#   GREY   on the stack of the walk currently running. Reaching a GREY node is
#          the definition of a cycle: the only way back to a node still on the
#          stack is around a loop.
#   BLACK  fully explored, and the walk that explored it did not stay inside a
#          loop. Reaching BLACK means "already accounted for, stop" -- and this
#          is the property that removes the need for any de-duplication pass.
#
# The GREY/BLACK distinction is the whole algorithm. A single "visited" flag
# would conflate "on my current path" with "seen earlier", and a second alias
# pointing into a loop explored a moment ago would be reported as a new cycle.
WHITE, GREY, BLACK = 0, 1, 2

#: Alias chain limit, per R-4. The contract gives two reasons and both matter:
#: it stops pathological nesting, and past three hops a human can no longer hold
#: the chain in their head -- so a longer chain is a defect even when it is
#: acyclic and resolves correctly.
# R-4 gives two independent reasons for this limit and both are worth keeping in
# view, because they fail differently:
#
#   * pathological nesting -- a deep alias graph makes resolution cost and error
#     messages grow without bound, and nothing in the configuration needs it
#   * readability -- past three hops a person can no longer hold the chain in
#     their head, so a four-hop alias is a defect even when it is acyclic and
#     resolves to exactly the right value
#
# The second reason is why this is not merely a recursion guard. A guard would
# be satisfied by any large number; this limit is deliberately small because its
# purpose is that a human can follow it.
MAX_CHAIN = 3


class CycleError(ConfigLayerError):
    """A reference cycle, or a chain longer than R-4 allows.

    Both conditions share a type because both are the same repair from the
    operator's side: open the loop or shorten the chain, in the same file, using
    the same printed path. Splitting them would make a caller catch two things
    to handle one situation.

    The path rides on the exception as data, not only inside the formatted
    message. A caller that wants to report it another way -- a structured event,
    an HMI panel -- should not have to parse the message text back apart, and a
    parser of your own error strings is a thing that silently breaks the next
    time somebody improves the wording.

    Inherits E_CONFIG_INVALID from ConfigLayerError, which is the code 10 S5.4.3
    names for this. NEVER give it its own code: the closed set is closed, and
    EC-2 forbids inventing one for a case that already has an entry.
    """

    def __init__(self, message: str, path: Sequence[str]):
        self.path = list(path)
        super().__init__(message)


def format_cycle(path: Sequence[str]) -> str:
    """Render a cycle in the exact shape 10 S5.4.3 prescribes.

    `path` is expected to start and end on the same key, which is what makes the
    loop visible to a reader scanning the output -- the last line repeating the
    first is the whole point of the format.

    The indentation is not decoration. The first key sits at two spaces and every
    hop at four, so a cycle inside a wall of other startup output still reads as
    one block.
    """
    # An empty path should be unreachable -- find_cycles never produces one --
    # but formatting is also called from tests and future tooling, and a
    # formatter that indexes into an empty list would raise IndexError from
    # inside error reporting. An error path that itself errors is how a real
    # fault ends up displayed as a traceback nobody can read.
    if not path:
        return "E_CONFIG_INVALID: reference cycle (empty path)"
    lines = ["E_CONFIG_INVALID: reference cycle", "  " + path[0]]
    lines.extend("    -> " + node for node in path[1:])
    return "\n".join(lines)


def _reference_target(value: Any) -> Optional[str]:
    """The common.* path this value references, or None if it is a plain value.

    Deliberately a narrow, local test rather than a call into refs.classify.
    Shape validation belongs to refs and raises on illegal forms; this module
    runs first and must not raise on a malformed reference, because doing so
    would report a shape problem while claiming to be the cycle detector and
    send the reader to the wrong section of the contract.

    So anything that is not exactly ${common....} is treated as "not a
    reference" here and left for refs.validate_shape to reject afterwards.
    """
    # Only strings can carry a reference. Ints, floats, None and lists reach
    # here constantly and returning early keeps the intent obvious: this
    # function answers one question and does not validate anything.
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text.startswith("${") or not text.endswith("}"):
        return None
    inner = text[2:-1]
    # A path with a space, an operator or a default marker is malformed rather
    # than circular. Return None and let the shape validator name the real rule.
    if not inner.startswith("common.") or any(c in inner for c in " +-*/%():?|"):
        return None
    return inner


def build_graph(tree: Dict[str, Any]) -> Dict[str, str]:
    """{key: referenced_key} for every alias in the common.* subtree.

    Keys that hold ordinary values do not appear at all. The result is therefore
    a functional graph: every node has at most one outgoing edge, because R-1
    requires a reference to occupy the whole scalar node, so a key cannot
    reference two things.

    That single-edge property is what lets the walk below stay simple. If R-1
    were ever relaxed to allow interpolation, this would have to become a
    multi-edge graph and the DFS would need a real adjacency list.
    """
    graph: Dict[str, str] = {}
    # flatten() is what keeps lists out of the graph: merge.flatten treats a list
    # as a leaf, so a list of strings arrives as one value and no element of it
    # is ever inspected. Walking the nested tree instead would make it natural to
    # descend into list elements, and a reference sitting inside a list would
    # then become a graph edge -- which R-1 forbids in the first place, so the
    # edge would represent something the loader must reject, not resolve.
    for key, value in flatten(tree).items():
        target = _reference_target(value)
        if target is not None:
            graph[key] = target
    return graph


def find_cycles(tree: Dict[str, Any]) -> List[List[str]]:
    """Every distinct cycle in the alias graph, each as a path that closes.

    Returns rather than raises, so tooling can list all of them at once. The
    loader path (detect) raises on the first, because a loader that continues
    past a cycle would hand a process a configuration it must not accept.

    Each returned path starts and ends on the same key.

    No de-duplication step is needed, and an earlier draft carried one that was
    dead. Marking the whole stack BLACK on the way out is what prevents a repeat:
    a second alias reaching into an already-explored loop meets a BLACK node, not
    a GREY one, so the loop is never rediscovered. A mutation test proved that
    branch unreachable -- turning it off changed nothing -- and an unreachable
    guard is worse than none, because the next reader assumes it is load-bearing.
    """
    graph = build_graph(tree)
    colour: Dict[str, int] = {node: WHITE for node in graph}
    cycles: List[List[str]] = []

    for start in graph:
        if colour[start] != WHITE:
            continue
        # Iterative walk with an explicit stack. Recursion would be shorter, but
        # a deeply chained (though legal) alias graph would then depend on the
        # interpreter's recursion limit, and the failure would look like a crash
        # rather than a configuration error.
        # One walk per unexplored start. `stack` holds the GREY nodes of THIS
        # walk in order, which is what lets the cycle be sliced out of it below:
        # the loop is exactly the tail of the stack from the repeated node on.
        stack: List[str] = []
        node = start
        while node is not None and colour.get(node, BLACK) == WHITE:
            colour[node] = GREY
            stack.append(node)
            node = graph.get(node)

        # Three ways the walk above can stop, and only one of them is a cycle:
        #   node is None            the chain ran off the end into a plain value
        #   colour[node] is BLACK   it joined a path explored by an earlier walk
        #   colour[node] is GREY    it came back to a node still on this stack
        # Only the third is a loop. Testing merely "did we stop early" would
        # report the second case too, and every alias pointing into an already
        # explored chain would look circular.
        if node is not None and colour.get(node) == GREY:
            # Closed a loop. The cycle is the stack from the repeated node
            # onward, plus that node again so the printed path visibly closes.
            first = stack.index(node)
            cycles.append(stack[first:] + [node])

        # Mark the whole walk BLACK, including the nodes inside a loop we just
        # reported. This is load-bearing, not housekeeping: it is what stops the
        # next alias that points into this loop from rediscovering it. Removing
        # it turns one defect into as many reports as there are entry points,
        # and the operator repairs one, re-runs, and meets the same loop again.
        for visited in stack:
            colour[visited] = BLACK

    return cycles


def find_overlong_chains(tree: Dict[str, Any]) -> List[List[str]]:
    """Acyclic alias chains longer than R-4 permits.

    Kept separate from find_cycles because the two are different defects with
    different repairs: a cycle has no correct value at all, while an overlong
    chain resolves fine and is rejected because nobody can follow it. Reporting
    them under one name would make the second look more alarming than it is and
    the first less so.

    Only maximal chains are reported. Every suffix of a too-long chain is also
    too long, and listing all of them would bury the one the operator has to
    shorten.
    """
    graph = build_graph(tree)
    # Nodes already reported as part of a loop are excluded twice over: they are
    # not used as chain heads, and the walk stops when it reaches one. Without
    # that, a chain feeding into a cycle would walk forever, and the length
    # check would fire with a path that is mostly the loop repeated -- naming a
    # rule (R-4) that is not the actual defect.
    in_cycle = {node for path in find_cycles(tree) for node in path}
    # A chain head is a node nothing else points at. Everything else is the
    # middle or the tail of some chain, and starting there would re-report the
    # same chain shortened by one hop each time.
    #
    # This filter only becomes observable once a chain is long enough that its
    # own suffix is ALSO over the limit -- six keys, not five. The first test
    # written for it used five and passed against an implementation with the
    # filter removed; a case that cannot fail is not a test.
    targets = set(graph.values())
    long_chains: List[List[str]] = []

    for start in graph:
        # Start only from chain heads: a node nothing else points at. Starting
        # everywhere would report the same chain once per node in it.
        if start in targets or start in in_cycle:
            continue
        path = [start]
        node = graph.get(start)
        while node is not None and node not in in_cycle and len(path) <= MAX_CHAIN + 1:
            path.append(node)
            node = graph.get(node)
        # A path of N keys contains N-1 hops. R-4 counts hops, so the limit is
        # exceeded once the path holds more than MAX_CHAIN + 1 keys.
        # A path of N keys contains N-1 hops, and R-4 counts hops, so the limit
        # is crossed once the path holds more than MAX_CHAIN + 1 keys. The
        # off-by-one here fails in opposite directions: too strict rejects a
        # legal three-hop chain, which stops the stack and gets noticed at once;
        # too loose accepts a four-hop chain, which resolves fine and is only
        # noticed when someone tries to read it. The silent direction is the one
        # to guard, hence a boundary case for exactly three hops in the suite.
        if len(path) > MAX_CHAIN + 1:
            long_chains.append(path)

    return long_chains


def detect(tree: Dict[str, Any]) -> None:
    """Raise CycleError on the first cycle, or on an overlong chain.

    This is the loader entry point. It is called before references are expanded
    so the message names the cycle rather than surfacing as a missing key three
    hops into resolution -- which is what the reader would otherwise see, and it
    points at the wrong place.

    Cycles are reported before overlong chains. A cycle is unconditionally wrong
    while a long chain is merely unreadable, and when a tree has both, fixing the
    cycle often shortens the chain as a side effect.
    """
    # First cycle only. Unlike find_cycles, which returns everything so tooling
    # can list it, the loader stops here: continuing past a cycle would mean
    # handing a process a configuration that has no correct value for at least
    # one key, and every later assertion would then be reasoning about a tree
    # that cannot exist.
    cycles = find_cycles(tree)
    if cycles:
        raise CycleError(format_cycle(cycles[0]), cycles[0])

    long_chains = find_overlong_chains(tree)
    if long_chains:
        path = long_chains[0]
        # Same layout as a cycle so the two read alike, with the rule named
        # because the reader's next question is "how long is allowed".
        rendered = "\n".join(
            ["E_CONFIG_INVALID: alias chain longer than %d hops (R-4)" % MAX_CHAIN,
             "  " + path[0]] + ["    -> " + node for node in path[1:]])
        raise CycleError(rendered, path)
