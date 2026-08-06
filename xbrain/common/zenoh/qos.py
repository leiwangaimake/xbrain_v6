"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: qos.py
Brief: The frozen QoS profile table plus the first-match binding resolver

Description:
What problem this solves. 11 S2.4.0 (grep "为什么 QoS 必须写进契约") opens by
stating that v0.1 of the contract carried no QoS at all, which handed every key's
real-time behaviour to the binding defaults -- block, data, reliable, FIFO(256).
That set is right for one cloud task and wrong for every control loop here, and
the section then names the consequence of each: cmd_vel on block breaks a tick,
quadruped sees cmd_age over 200 ms and Tier 1 locks the machine. So every
publisher and subscriber must set QoS explicitly, and the values must come from
one table rather than from five processes each reading the same prose.

That table is 11 S2.4.2 (grep "档位表(冻结)") and its machine form is the json5
block in 11 S2.4.7 (grep "key 前缀 -> 档位"). This module is the one place either
is turned into values, and the one place a key is turned into the QoS it carries.

The three pieces, and why each is where it is:
  * FROZEN_PROFILES -- transcribed from S2.4.2 / S2.4.7. Frozen face F-11 (14.2,
    grep "QoS 档位定义与绑定规则"), so a deployment may not redefine a profile;
    load_qos_table audits the supplied profiles against this table and refuses a
    divergent one.
  * bindings -- NOT transcribed. They arrive from the deployment document. If
    they were a constant here, the ordering that the whole design rests on would
    be untestable: a resolver that ignores its input passes every vector no
    matter what order the input is in, which is CLAUDE.md 3.2 form 1.
  * RT_OVERRIDE -- hard-coded below. The S2.4.7 field table says so in as many
    words: 硬编码在实现中, 配置文件里的值仅供审计比对. A deployment that deletes
    it changes nothing; one that contradicts it is refused.

*** Why first-match ordering is a safety property and not a style choice. The
comment above the bindings array in S2.4.7 records the v0.3 repair verbatim:
rt/safety/estop and probe/estop/{ping,pong} had been left to fall through into
the rt/** and xbrain/*/** fallbacks, so the emergency-stop path lost real_time
and express on one plane and acquired block on the other. Nothing about that
failure is visible in operation -- estop still arrives, just from a lower queue
and behind an unbounded wait. Ordering IS the mechanism, so the resolver reports
which binding matched and the golden vectors assert on it.

*** handler.depth = 0 means NOT SUPPLIED, not "a queue of length zero". The
S2.4.7 field table row for handler.depth reads 0 = 无默认值, 必须由部署配置显式
给出, 否则拒绝启动, and 11 S13.15 lists that exact condition under
E_QOS_VIOLATION (grep "handler.depth = 0 未由部署配置显式给出"). Q4_stream is the
profile that carries it today. So a depth of 0 is loaded as MISSING and
QosResolution.require_handler_depth() raises. It is deliberately NOT filled in
here: S2.4.2 does now compute N = 10 for Q4 from a 200 ms jitter buffer over
20 ms chunks, but the same paragraph keeps the value pending QoS-T8, and a number
written into code cannot be told apart later from a number that was calibrated.
CLAUDE.md 3.1 is the general form of this and its worst case is exactly what a 0
would produce here: a value that passes the "is it assigned" test and then sizes
a queue to nothing with nothing logged.

What this module deliberately does NOT do:
  * It does not read configuration from disk. Run-time processes read the
    resolved artefacts under /run/xbrain/resolved (CLAUDE.md 3.6, 10 S5.4.1), and
    configs/common.yaml carries qos.profiles and qos.bindings as declared-but-
    null key positions today, so the stack refuses to start until they are
    supplied. That refusal is the design working, not a gap to paper over.
  * It does not check declarations against the anti-pattern table S2.4.8 A-1 to
    A-7. That is the startup self-check, INF-ZN-5 and INF-ZN-6, and it consumes
    this module rather than living inside it.
  * It does not check that a key is registered in the S2.2 key table. That is
    INF-ZN-4's registry. This module rejects a MALFORMED key, which is a
    different claim: xbrain/dog-01/cmd/nonsense is well formed and unregistered,
    and only the registry can say so.
  * It does not take the QoS column of the S2.2 key tables as truth. That column
    still carries flagged rows meaning "the bindings resolve this key somewhere
    its semantics do not fit", and S2.2.12 第 7 条 has already ruled 以 S1.1.6
    为准 for the one key where the two disagree outright. The bindings are the
    golden source for resolution; the key table is a reader's aid.

Traps that look correct and are not:
  * Falling back to a default profile when no binding matches. That is precisely
    the S2.4.0 failure the whole section exists to close, and it is silent: the
    key still publishes, from the wrong queue. Resolution raises instead.
  * Accepting a bare key such as cmd/estop. Every binding in S2.4.7 is written
    with the xbrain/*/ root because S2.1 fixes the key space at
    xbrain/{robot_id}/{plane}/{domain}/{name}. A bare key matches none of the
    specific rules and lands in the xbrain/*/** fallback -- except that fallback
    does not match it either, so it would resolve to nothing at all. Either way
    the answer would be wrong while looking ordinary, so a key that is not
    rooted is rejected at the door.
  * Letting a binding's set clause change congestion_control or reliability. The
    S2.4.7 field table forbids both by name and gives the reason: 这两项是安全
    语义. A set that flipped Q3_cmd to drop would delete the back-pressure the
    event pipeline's replay cursor depends on (S2.4.5 第 6 条), and the loss
    shows up as events that were marked sent and never arrived.
  * Applying the rt override before the set clause. The override must win: the
    field table says 实现必须无条件应用, 且不可由部署配置关闭, and a set clause
    applied afterwards would be exactly the deployment-side disable it forbids.
  * Matching key expressions with fnmatch or a regex built from the pattern.
    Zenoh's ** spans whole chunks and may span zero of them, and * never crosses
    a /. fnmatch's * crosses separators, so xbrain/*/cmd/estop would swallow
    xbrain/dog-01/cmd/estop/ack and the ack would silently inherit the wrong
    row. The matcher below is written for the two wildcards the contract uses and
    rejects any other wildcard syntax rather than guessing;
    tests/common/zenoh/test_qos_resolve.py cross-checks every pattern-key pair
    against zenoh.KeyExpr where the client is installed.

How a key becomes QoS, end to end. Worth reading once, because two of the
nineteen keys in the golden vectors resolve somewhere no reader guesses:

    load_qos_table(document)
      |  parse and validate profiles         -> refuses an unknown knob value
      |  audit profiles against S2.4.2       -> refuses a redefined profile
      |  parse bindings IN ORDER             -> refuses a dead or unrooted rule
      |  audit rt_override if present        -> refuses a contradictory one
      v
    table.resolve("xbrain/dog-01/rt/behavior/request")
      |  parse_full_key                      -> root, robot_id, plane, no wildcard
      |  first binding whose match covers it -> "xbrain/*/rt/**" wins
      |  profile Q3_cmd, then the set clause -> no set clause on that rule
      |  QOS-C1: plane is rt and cc is block -> rewritten to drop / FIFO(32)
      v
    QosResolution(profile_name="Q3_cmd", rt_override_applied=True, ...)

The second surprise is xbrain/dog-01/rt/chassis/state. TWO rules cover it --
xbrain/*/rt/**/state and xbrain/*/rt/chassis/** -- and they give the same profile
today, so only the reported binding index shows which one won. S2.4.7's own
comment explains why the more general rule is deliberately written first: it is
the catch-all that stops ANY future rt state key from falling into Q3-rt, and
deleting it 会让新增 key 再漏一次.

When a key resolves to something surprising, in the order worth checking:
  1. Which binding won. resolve() reports the index and the expression; a rule
     written earlier than you expected is the usual answer, and it is the
     documented answer -- first match wins, not most specific match wins.
  2. Whether QOS-C1 fired. rt_override_applied distinguishes "Q3 on the general
     plane" from "Q3 on the RT plane", which have different congestion control
     and different handlers while sharing a profile name.
  3. Whether a set clause moved priority or depth. The profile name in the
     result is the profile BEFORE the clause, because the name is what the
     document says and the knobs are what the process gets.

What changes if the contract changes, so nobody edits half of it:
  * A new or altered profile means editing FROZEN_PROFILES here, the C++ table
    in common/include/xbrain/zenoh/qos_profiles.h, and the transcription checks
    in tests/common/zenoh/test_qos_resolve.py -- which will already be red,
    because they grep 11 rather than trusting either table.
  * A new binding means editing the deployment document and the golden vectors.
    Nothing in this module changes; that is the point of not transcribing them.
  * A change to rt_override means editing RT_OVERRIDE and its C++ twin. A
    deployment cannot make that change, by design.

*** One tension inside S2.4.7 that this module does NOT resolve on its own, per
CLAUDE.md 9.1. The field table row for bindings[].set reads 只允许覆写 priority
与 handler.depth, while the frozen bindings array in the same section writes
set: { handler: { kind: "ring", depth: 4 } } on rt/audio/mic, rt/audio/play and
cmd/teleop -- handler.kind, which that row does not list. Read the row as a
strict key whitelist and the section's own bindings are unloadable. Every kind
written in those three clauses equals the kind already in the referenced profile,
so none of them changes anything, and this module therefore accepts a kind that
RESTATES the profile's kind and refuses one that would CHANGE it. That is the
only reading under which both sentences hold, but it is a reading: if a later
revision means to allow a real kind change, this is the check to revisit.
"""

# Standard library only, and only these two. Every runtime process imports this
# package indirectly through xbrain.common.zenoh, so a third-party import here
# would turn one missing wheel into a startup failure for all of them at once --
# the same argument the errors package makes for hand-parsing its own table.
# Notably absent: no yaml and no json reader. The document arrives already
# parsed, because the process that reads /run/xbrain/resolved owns that choice.
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

# MISSING comes from the configuration package rather than being defined again
# here. Its own comment gives the reason a second sentinel would break: identity
# is the entire contract, so two objects both meaning "absent" compare unequal
# and a caller testing against the wrong one silently sees "present".
#
# merge.py warns never to let MISSING into a tree that is merged or written to
# /run/xbrain/resolved. Nothing here does: it appears only inside HandlerSpec,
# which is a returned value object, never an input to deep_merge and never
# serialised.
from ..config import MISSING
# PLANE is the eight-value closed set from 11 S2.1, exported by the shared
# library. Written as a literal here it would be a private copy that drifts;
# imported, an out-of-set plane raises ClosedSetViolation with the set named.
from ..enums import PLANE
# CLAUDE.md 3.5: the codes are imported, never spelled. A misspelt import is an
# ImportError in this file; a misspelt literal travels to whoever branches on it.
from ..errors import E_CONFIG_INVALID, E_QOS_VIOLATION
from ..errors.exceptions import XbrainError

# ---------------------------------------------------------------------------
# The closed sets of 11 S2.4.1 and the S2.4.7 field table.
#
# Tuples and not lists: a module-level list is one append away from a value the
# frozen table has no row for, and the failure would surface as a profile that
# loads and then configures nothing.
# ---------------------------------------------------------------------------

#: S2.4.1 congestion_control. Two values, and the difference between them is the
#: difference between losing one message and stalling the calling thread for a
#: time nobody bounds.
CONGESTION_CONTROLS: Tuple[str, ...] = ("drop", "block")

#: block and drop are named because three places below have to talk about the
#: values themselves: the QOS-C1 trigger, the set-clause prohibition and the
#: messages. Repeating the literal is how one of the three ends up spelled
#: differently, and a comparison against a misspelt literal is always False --
#: which here means the override silently never fires.
BLOCK = "block"
DROP = "drop"

#: S2.4.7 field table, priority row, in the order it writes them. That order is
#: also Zenoh's 1..7 ranking, so the sequence itself carries the priority order
#: and a reader does not have to look it up somewhere else.
PRIORITIES: Tuple[str, ...] = ("real_time", "interactive_high", "interactive_low",
                               "data_high", "data", "data_low", "background")

#: S2.4.1 reliability. The section's own note matters to anyone tempted to fix a
#: dropped message by making the key reliable: the two knobs are orthogonal, and
#: reliable does not rescue a message the send queue already discarded.
RELIABILITIES: Tuple[str, ...] = ("reliable", "best_effort")

#: S2.4.1 handler, subscriber side. ring drops the oldest and fifo
#: back-pressures the publisher; S2.4.6 is the argument for why that difference
#: decides whether lag stays bounded, and why a FIFO under a periodic key also
#: makes the frame-age check vacuous rather than merely late.
HANDLER_KINDS: Tuple[str, ...] = ("ring", "fifo")

#: The depth value the S2.4.7 field table gives the meaning "no default, must be
#: supplied by deployment configuration, otherwise refuse to start". Loaded as
#: MISSING everywhere it appears, which is what stops it sizing a queue.
#:
#: Named rather than compared against a bare 0, because a bare 0 in this
#: position reads as a queue length and that is precisely the misreading the
#: sentinel has to survive. It is also why the translation happens once, in
#: _parse_depth, instead of at each consumer: a consumer that forgot would build
#: a ring buffer holding nothing, drop every message, and log not one word.
_DEPTH_NOT_SUPPLIED = 0

#: First chunk of every key in S2.1 and of every binding in S2.4.7. A key that
#: does not start here belongs to no plane at all.
KEY_ROOT = "xbrain"

#: The robot_id charset, S2.1 逐字 [a-z0-9_-]{1,32}. Checked with a set
#: membership loop rather than a regex, so this module keeps the small
#: standard-library surface the error package next door argues every
#: universally-imported layer should keep.
_RID_CHARS = frozenset("abcdefghijklmnopqrstuvwxyz0123456789_-")
_RID_MAX_LEN = 32

#: The two wildcards S2.4.7 actually uses. A single * covers exactly one chunk
#: and never crosses a separator; ** covers any number of chunks, zero included.
#: That zero is not a detail -- it is what lets the safety rule
#: xbrain/*/rt/safety/** cover xbrain/{rid}/rt/safety itself.
#:
#: Any other wildcard syntax a Zenoh
#: version may accept (the partial-chunk form, for one) is refused rather than
#: approximated -- a pattern this matcher reads differently from the router is
#: worse than a pattern it rejects, because the disagreement is invisible from
#: both sides.
_WILD_CHUNK = "*"
_WILD_ANY = "**"

#: The plane whose keys QOS-C1 governs, taken through the closed set so that a
#: typo fails at import time. Spelled as a literal it would fail at run time by
#: never matching, which disables the override for every key and logs nothing.
_RT_PLANE = PLANE.parse("rt")


# ---------------------------------------------------------------------------
# Value types.
# ---------------------------------------------------------------------------

# frozen=True per CLAUDE.md 4.5, and it earns its keep beyond style here: a
# resolution is handed to a publisher factory and to a startup self-check, and a
# mutable one would let the second caller see what the first one adjusted.
@dataclass(frozen=True)
class HandlerSpec:
    """A subscriber-side queue: kind plus depth, where depth may be absent.

    Subscriber-side only, which is worth stating because four of the five knobs
    in a profile are publisher-side. 11 S2.4.1 puts the handler in its own row
    for that reason, and 11 S2.4.8 A-3 -- an RT periodic key given a FIFO -- is
    a defect a publisher cannot commit and cannot detect.
    """

    #: ring or fifo, from HANDLER_KINDS.
    kind: str
    #: int, or MISSING when the deployment has not supplied it. Typed Any and
    #: not Optional[int], because None is a value a document can legitimately
    #: carry and MISSING is not -- that distinction is the whole reason the
    #: configuration package has a sentinel at all. A caller must go through
    #: QosResolution.require_handler_depth rather than do arithmetic on this.
    depth: Any


@dataclass(frozen=True)
class QosProfile:
    """One row of the S2.4.2 frozen table."""

    #: The profile's own name, carried inside the row as well as being its key
    #: in FROZEN_PROFILES. Duplicated on purpose: a resolution reports the name,
    #: and passing the row around without it would mean threading the key
    #: separately through every call that touches a profile.
    name: str
    #: drop or block, from CONGESTION_CONTROLS. The knob QOS-C1 is about.
    congestion_control: str
    #: One of PRIORITIES. Zenoh keeps one send queue per priority, so this is
    #: which queue the key competes in, not how fast it is.
    priority: str
    #: reliable or best_effort. Orthogonal to congestion_control: it covers
    #: link-layer loss and cannot rescue a message a full queue discarded.
    reliability: str
    #: True skips the batching timer, trading packet overhead for latency.
    #: Only Q0 and Q1 carry it, and S2.4.5 第 7 条 is the argument for Q0.
    express: bool
    #: The SUBSCRIBER-side queue. It travels with the profile because a
    #: publisher and a subscriber on one key must be configured from the same
    #: row -- 11 S2.4.8 A-3 is a defect that exists only on the subscriber side.
    handler: HandlerSpec


@dataclass(frozen=True)
class RtOverride:
    """QOS-C1, the three fields S2.4.7 writes into rt_override.

    reliability and express are absent on purpose: the override block in S2.4.7
    does not carry them, so a key it applies to keeps the reliability and express
    of the profile it matched. Adding them here would silently change Q3-rt away
    from the S2.4.2 row, which reads reliable and express false.
    """

    congestion_control: str
    priority: str
    handler: HandlerSpec


# ---------------------------------------------------------------------------
# Errors.
#
# Two types, because the two failures need different reader actions. A bad
# document is a deployment defect fixed by editing configuration; a bad key or an
# unresolvable one is a defect in the process that declared it. Both refuse to
# start -- 11 S13.15 gives both rows retryable = no -- but a single type would
# make the two indistinguishable at the catch site, and the two go to different
# people.
# ---------------------------------------------------------------------------

class QosConfigError(XbrainError):
    """The qos document violates the frozen definition in 11 S2.4.2 / S2.4.7.

    Every raise site names the path inside the document -- qos.profiles.Q1_rt,
    qos.bindings[7].set -- because the person who has to fix it is holding a
    file with twenty-five bindings in it and a message saying only what rule was
    broken leaves them to find where. 11 S13.15 gives this row retryable = no,
    which is right: retrying with the same document changes nothing.
    """

    def __init__(self, message: str):
        # E_CONFIG_INVALID: codes.yaml reads 配置文件自校验不通过, 启动自检拒绝
        # 放行, which is this condition exactly. The code is written once, at
        # the constructor, so that no raise site below can drift in case or in
        # prefix -- which is the failure CLAUDE.md 3.5 describes.
        super().__init__(E_CONFIG_INVALID, message)


class QosViolation(XbrainError):
    """A key cannot be given a QoS the frozen table guarantees.

    Raised for a malformed key, for a key no binding covers, and for a handler
    depth the deployment never supplied. All three share one property: the
    process cannot declare that key at all, so there is nothing to degrade to.
    """

    def __init__(self, message: str):
        # E_QOS_VIOLATION: 11 S13.15 words the row as QoS 配置违反冻结约束, 启动
        # 自检拒绝放行, and lists the unsupplied handler.depth under it by name.
        # The same row says 不得降级放行 -- there is deliberately no warn level
        # for this, because a warn is how fail-safe degrades into fail-silent.
        super().__init__(E_QOS_VIOLATION, message)


@dataclass(frozen=True)
class QosResolution:
    """What one key resolves to, plus enough provenance to test the ordering.

    binding_index and binding_match are part of the result rather than debug
    output. The v0.3 repair recorded in S2.4.7 was an ORDERING defect: the keys
    involved still resolved to a valid profile, just the wrong one. An assertion
    that only checked the resulting knobs would pass for any binding that
    happened to carry the same profile, so the golden vectors assert on which
    rule matched.
    """

    key: str
    #: Position of the winning rule in the bindings array, and the expression it
    #: was written with. Both, not one: an index alone says nothing to a reader
    #: of a failure, and a match expression alone does not prove the FIRST such
    #: rule won when two of them can cover the same key.
    binding_index: int
    binding_match: str
    profile_name: str
    congestion_control: str
    priority: str
    reliability: str
    express: bool
    handler: HandlerSpec
    #: True when QOS-C1 rewrote this resolution. Recorded because the override is
    #: otherwise invisible in the result: Q3-rt has no name of its own in the
    #: document and reads exactly like an ordinary profile.
    rt_override_applied: bool

    # An accessor and not a plain field read, so that "the deployment never
    # supplied this depth" cannot be turned into a queue size by accident.
    # Returning a default here instead would be the CLAUDE.md 3.1 failure in its
    # exact shape: the caller sizes a ring buffer from a number nobody
    # calibrated, and nothing in the log says which number or where it came from.
    def require_handler_depth(self) -> int:
        """The subscriber queue depth, or raise if deployment never supplied it."""
        if self.handler.depth is MISSING:
            # The message names the key AND the profile, because the reader has
            # to edit a profile in the document while holding a key that failed.
            raise QosViolation(
                "handler.depth for key %r (profile %s) not supplied by "
                "deployment, see 11 S2.4.7 and S13.15"
                % (self.key, self.profile_name))
        # int() rather than returning the field directly: the value has already
        # been checked to be an int at load, so this is a no-op that documents
        # the type for a reader who arrives here from the Any-typed field.
        return int(self.handler.depth)


# ---------------------------------------------------------------------------
# The frozen table, transcribed from 11 S2.4.2 and the S2.4.7 profiles block.
#
# Written as literals rather than parsed out of the document at import time. The
# contract is markdown with prose around the block, and a parser for it would be
# a second thing to keep correct; instead tests/common/zenoh/test_qos_resolve.py
# greps 11 to prove each row is still what the section says -- the same
# construction the session factory uses for its endpoints, for the same reason.
# A transcription nobody compares against its source is a constant that drifts.
# ---------------------------------------------------------------------------

FROZEN_PROFILES: Dict[str, QosProfile] = {
    # Q0. drop + real_time + express is the emergency-stop budget argued in
    # S2.4.5 第 7 条: Q3's block would let a full queue turn a pressed estop into
    # "pressed and nothing happened", so the residual loss is covered by the
    # 10 Hz resend in S2.3 instead of by back-pressure. Ring(8) on the
    # subscriber side is deep enough to hold a resend burst and shallow enough
    # that nothing stale survives to be acted on.
    "Q0_safety": QosProfile("Q0_safety", DROP, "real_time", "reliable", True,
                            HandlerSpec("ring", 8)),
    # Q1. best_effort because a 50 ms control period makes a retransmitted frame
    # worthless -- there is a newer one already -- and Ring(1) because FIFO turns
    # one late tick into permanent lag AND silently defeats the frame-age check
    # that is supposed to notice (S2.4.6).
    "Q1_rt": QosProfile("Q1_rt", DROP, "real_time", "best_effort", True,
                        HandlerSpec("ring", 1)),
    # Q2. reliable and drop together, which reads contradictory and is not: they
    # are orthogonal knobs (S2.4.5 第 4 条), reliable covering link loss and drop
    # covering a full queue. Ring(4) rather than Ring(1) because state keys carry
    # transitions, and depth 1 loses the intermediate state between two of them,
    # which breaks state-machine reconstruction downstream (第 5 条).
    "Q2_state": QosProfile("Q2_state", DROP, "data_high", "reliable", False,
                           HandlerSpec("ring", 4)),
    # Q3. The only profile with block, and the only one where block is correct:
    # S2.4.5 第 6 条 shows that dropping an event silently advances the replay
    # cursor past it, so the event is marked sent and lost with nobody informed.
    # Back-pressure on the producer is the lesser harm.
    "Q3_cmd": QosProfile("Q3_cmd", BLOCK, "data", "reliable", False,
                         HandlerSpec("fifo", 256)),
    # Q4. depth is MISSING, which is how S2.4.7's literal 0 is read. See the
    # header for why the now-computable N is deliberately not written here.
    "Q4_stream": QosProfile("Q4_stream", DROP, "interactive_high", "best_effort",
                            False, HandlerSpec("ring", MISSING)),
}

#: QOS-C1, hard-coded. S2.4.7 field table: 硬编码在实现中, 配置文件里的值仅供审计
#: 比对. That is the whole point of the row -- a deployment must not be able to
#: switch it off, because switching it off restores block on the RT plane and
#: S2.4.3 gives block's blocking time no upper bound at all. One blocked put
#: swallows an entire 50 ms control period.
RT_OVERRIDE = RtOverride(DROP, "interactive_high", HandlerSpec("fifo", 32))


# ---------------------------------------------------------------------------
# Key-expression matching.
# ---------------------------------------------------------------------------

# key_expr_matches -- does a S2.4.7 binding pattern cover this concrete key.
#
# The algorithm is the standard two-dimensional wildcard match over CHUNKS, not
# over characters: ok[i][j] is "the first i pattern chunks cover the first j key
# chunks". ** either consumes nothing (ok[i-1][j]) or one more key chunk
# (ok[i][j-1]); * and a literal consume exactly one. That is what makes ** able
# to span zero chunks, which the contract relies on -- xbrain/*/rt/safety/**
# covers xbrain/dog-01/rt/safety itself, not only its children, and a matcher
# that required at least one chunk would drop rt/safety past the safety group.
#
# Verified against the real matcher rather than reasoned about: the test module
# runs every pattern in the golden document against every golden key through
# zenoh.KeyExpr.includes and requires the two to agree.
def key_expr_matches(pattern: str, key: str) -> bool:
    """True when a S2.4.7 match expression covers this concrete key.

    Cost is O(pattern chunks * key chunks), which for this contract is at most
    about thirty boolean cells. It runs once per key at declare time, never
    inside a control loop, so the table is built plainly rather than cached --
    a cache keyed on the pattern would be one more thing to invalidate when a
    document is reloaded, for a saving nothing measures.
    """
    # Both sides are split into chunks up front. Zenoh key expressions are
    # defined over chunks, not characters, and every wildcard rule below is a
    # statement about how many chunks a pattern element consumes -- so working
    # on the strings directly would mean re-deriving the chunk boundaries
    # inside the loop, which is where an off-by-one hides best.
    p: List[str] = pattern.split("/")
    k: List[str] = key.split("/")
    n, m = len(p), len(k)
    # Row 0 is "no pattern chunks left". Only an empty remainder of the key
    # matches it, so every column but the first stays False -- that is what
    # stops a short pattern from covering a longer key.
    ok: List[List[bool]] = [[False] * (m + 1) for _ in range(n + 1)]
    ok[0][0] = True
    for i in range(1, n + 1):
        chunk = p[i - 1]
        # j runs from 0 so that a ** can be satisfied having consumed nothing.
        # Starting at 1 would be the off-by-one that removes the zero-chunk case
        # and, with it, the safety group's coverage of rt/safety itself.
        for j in range(0, m + 1):
            if chunk == _WILD_ANY:
                ok[i][j] = ok[i - 1][j] or (j > 0 and ok[i][j - 1])
            elif j > 0:
                # A single-chunk * matches any one chunk but never crosses a /.
                # This is the line fnmatch would get wrong; see the header.
                consumed = chunk == _WILD_CHUNK or chunk == k[j - 1]
                ok[i][j] = consumed and ok[i - 1][j - 1]
    return ok[n][m]


# parse_full_key -- the door check, and the reason mutant 4 in the item's
# criterion is meaningful.
#
# It rejects rather than normalises. A bare cmd/estop could be "helpfully"
# prefixed with a wildcard rid and resolved, and the result would even look
# right -- but the caller that passed a bare key is a caller that is about to
# declare one, and a publisher on cmd/estop is on no plane any subscriber is
# listening to. Rejecting puts the error where the mistake is, which is the only
# place it can be fixed.
def parse_full_key(key: str) -> Tuple[str, ...]:
    """Split a fully-qualified key into chunks, or raise QosViolation."""
    # Type first. A None or a bytes key would otherwise fail inside split() with
    # an AttributeError naming neither the key nor the caller.
    if not isinstance(key, str) or not key:
        raise QosViolation("key must be a non-empty string, got %r" % (key,))
    chunks = tuple(key.split("/"))
    # An empty chunk means a leading, trailing or doubled slash. Zenoh rejects
    # those too, but at declare time and in a message about the key expression
    # rather than about the process that built it.
    if any(c == "" for c in chunks):
        raise QosViolation("key %r has an empty chunk, see 11 S2.1" % (key,))
    # Wildcards are refused because 11 S2.2 W-4 states it outright: the * and **
    # in the key tables come from S2.4.7 match expressions or from prose
    # shorthand, and 二者都不构成一个可发布/可订阅的 key. A wildcard key handed
    # to this resolver would get one answer for a whole family of keys, which is
    # exactly the kind of single answer that later turns out to fit none of them.
    if any(c in (_WILD_CHUNK, _WILD_ANY) for c in chunks):
        raise QosViolation("key %r has a wildcard, see 11 S2.2 W-4" % (key,))
    # xbrain/{robot_id}/{plane}/... -- at least four chunks, because even the
    # shortest registered keys (health/bit, cmd/estop) carry a name after the
    # plane. Three would mean a key naming a plane and nothing on it.
    if len(chunks) < 4 or chunks[0] != KEY_ROOT:
        raise QosViolation("key %r is not rooted at %s/{robot_id}/{plane}/..., "
                           "see 11 S2.1" % (key, KEY_ROOT))
    rid = chunks[1]
    # The charset check catches the other half of the same mistake: a key built
    # by string concatenation with an unset robot_id yields something like
    # xbrain/None/cmd/estop, which is rooted, well shaped, and addressed to a
    # robot that does not exist. Nothing downstream would notice.
    if len(rid) > _RID_MAX_LEN or any(c not in _RID_CHARS for c in rid):
        raise QosViolation("robot_id %r in key %r is outside the 11 S2.1 "
                           "charset" % (rid, key))
    # The plane check is last, after the shape checks. Order matters for the
    # reader: a key with a doubled slash would otherwise be reported as having
    # an unknown plane, sending them to look at the eight-value closed set for
    # a defect that is a stray character.
    #
    # Closed set, from the shared library. An out-of-set plane raises
    # ClosedSetViolation naming the set, which is what 11 S13.6 requires: no
    # silent pass-through and no interpreting an unknown value as something
    # nearby. The return value is discarded because the raise is the point.
    PLANE.parse(chunks[2])
    return chunks


# ---------------------------------------------------------------------------
# Document parsing.
# ---------------------------------------------------------------------------

# _require_mapping / _require_exact_keys -- shape checks used by both halves.
#
# They exist so a malformed document produces a message naming the path inside
# it. The alternative is a KeyError or a TypeError raised deep in the parse,
# which names a Python expression and leaves the reader to work out which of
# twenty-five bindings it came from.
def _require_mapping(value: Any, where: str) -> Mapping[str, Any]:
    """Return value as a mapping, or raise QosConfigError naming where."""
    # Mapping and not dict. The resolved artefacts are loaded by whichever YAML
    # or JSON reader the process already has, and several of them hand back a
    # mapping type that is not a dict subclass. An isinstance(dict) check would
    # reject a perfectly good document for a reason that has nothing to do with
    # its contents, and the reader would go looking in the wrong file.
    if not isinstance(value, Mapping):
        raise QosConfigError("%s must be a mapping, got %r" % (where, value))
    return value


def _require_exact_keys(got: Mapping[str, Any], want: Sequence[str],
                        where: str) -> None:
    """Raise unless got carries exactly the keys in want.

    Exactly, not "at least". An unexpected key is how a misspelt field ends up
    ignored -- congestion_controll would leave the real one at its frozen value
    and read, to anyone editing the file, as though it had been set.

    want is a sequence rather than a set so the message lists the fields in the
    order 11 S2.4.7 writes them. A set would print them in whatever order the
    interpreter chose, and the reader comparing the message with the section
    would have to sort it themselves.
    """
    missing = [k for k in want if k not in got]
    extra = [k for k in got if k not in want]
    if missing or extra:
        # Both lists are reported, because the usual cause is a rename and the
        # reader needs to see the pair to recognise it as one.
        raise QosConfigError("%s wants exactly %s, missing %s, extra %s"
                             % (where, list(want), missing, extra))


def _parse_depth(value: Any, where: str) -> Any:
    """A handler depth: a non-negative int, with 0 read as MISSING.

    isinstance(True, int) is True in Python, so bool is excluded explicitly.
    Without that, handler.depth: true loads as depth 1 -- a ring of one, which is
    a plausible-looking queue and the wrong one for every profile but Q1.
    """
    if isinstance(value, bool) or not isinstance(value, int):
        raise QosConfigError("%s must be an int, got %r" % (where, value))
    # Negative is refused rather than clamped. A clamp would turn a typo into a
    # working queue of some other size, and the operator would never learn that
    # the number they wrote was not the number in use.
    if value < 0:
        raise QosConfigError("%s must be >= 0, got %d" % (where, value))
    # The sentinel translation, in one place. See the header: 0 is 11 S2.4.7's
    # spelling for "deployment must supply this", and turning it into MISSING
    # here is what makes the failure land at require_handler_depth() instead of
    # sizing a queue to nothing.
    return MISSING if value == _DEPTH_NOT_SUPPLIED else value


def _parse_handler(raw: Any, where: str) -> HandlerSpec:
    """A handler block: kind from the closed set, depth through _parse_depth."""
    block = _require_mapping(raw, where)
    # Exactly kind and depth. A handler block with a third field is a document
    # written against some other version of the contract, and quietly ignoring
    # that field would let the person who wrote it believe it took effect.
    _require_exact_keys(block, ("kind", "depth"), where)
    kind = block["kind"]
    # Closed set. An unrecognised kind must not pass through as a string that
    # some consumer later compares against "ring" and quietly treats as fifo.
    if kind not in HANDLER_KINDS:
        raise QosConfigError("%s.kind %r is outside %s"
                             % (where, kind, HANDLER_KINDS))
    return HandlerSpec(kind, _parse_depth(block["depth"], where + ".depth"))


def _parse_profile(name: str, raw: Any) -> QosProfile:
    """One profiles.<name> block, fully validated against the closed sets."""
    where = "qos.profiles.%s" % name
    block = _require_mapping(raw, where)
    _require_exact_keys(block, ("congestion_control", "priority", "reliability",
                                "express", "handler"), where)
    # Each of the three enumerations is checked against its own tuple rather
    # than against "whatever the frozen profile has". The audit further down
    # compares with the frozen table; this compares with the contract's value
    # space, so a typo is reported as a typo and not as a divergence from Q2 --
    # two different edits with two different fixes.
    for field, allowed in (("congestion_control", CONGESTION_CONTROLS),
                           ("priority", PRIORITIES),
                           ("reliability", RELIABILITIES)):
        if block[field] not in allowed:
            raise QosConfigError("%s.%s %r is outside %s"
                                     % (where, field, block[field], allowed))
    # express must be a real bool. A string "false" is truthy in Python, so this
    # is the field where a json5 quoting slip would silently enable express on
    # every state key and change nothing visible except the packet rate.
    if not isinstance(block["express"], bool):
        raise QosConfigError("%s.express must be a bool, got %r"
                             % (where, block["express"]))
    return QosProfile(name, block["congestion_control"], block["priority"],
                      block["reliability"], block["express"],
                      _parse_handler(block["handler"], where + ".handler"))


# _audit_against_frozen -- a deployment may supply the table, not redefine it.
#
# Why this is here at all. 14.2 registers F-11 (grep "QoS 档位定义与绑定规则") as
# a frozen face, and S2.4.7's profiles block is labelled 档位定义(冻结, 见
# S2.4.2). Without this check the word frozen has no mechanism behind it: a
# deployment could set Q0_safety to block and every golden vector in this project
# would still pass, because the vectors assert what the document says.
#
# The one thing a deployment MAY do is supply a depth the frozen table leaves
# unsupplied. That is not a redefinition -- it is the obligation the 0 sentinel
# creates, and Q4_stream sits in that state today.
def _audit_against_frozen(profiles: Mapping[str, QosProfile]) -> None:
    """Raise unless profiles matches FROZEN_PROFILES, depth exception aside."""
    # Both directions of the difference, and each one separately. A profile the
    # contract does not define routes around every argument in S2.4.5, since
    # none of them covers it; a missing one means some binding will fail to
    # resolve later, far from the document that caused it.
    unknown = sorted(set(profiles) - set(FROZEN_PROFILES))
    if unknown:
        raise QosConfigError("qos.profiles defines %s, absent from 11 S2.4.2"
                             % unknown)
    absent = sorted(set(FROZEN_PROFILES) - set(profiles))
    if absent:
        raise QosConfigError("qos.profiles omits %s, defined by 11 S2.4.2"
                             % absent)
    for name, frozen in FROZEN_PROFILES.items():
        got = profiles[name]
        # The four scalar knobs, compared by name through getattr so that adding
        # a field to QosProfile without adding it here is visible as an omission
        # in one list rather than as a missing if-statement among four.
        for field in ("congestion_control", "priority", "reliability", "express"):
            if getattr(got, field) != getattr(frozen, field):
                raise QosConfigError(
                    "qos.profiles.%s.%s is %r, 11 S2.4.2 freezes it at %r"
                    % (name, field, getattr(got, field), getattr(frozen, field)))
        # kind is frozen; only depth has the supply exception below.
        if got.handler.kind != frozen.handler.kind:
            raise QosConfigError(
                "qos.profiles.%s.handler.kind is %r, 11 S2.4.2 freezes it at %r"
                % (name, got.handler.kind, frozen.handler.kind))
        # The depth rule, written as the two cases it actually has. A frozen
        # depth must be kept. An unsupplied one may be filled in or left
        # unsupplied, and leaving it unsupplied is what makes
        # require_handler_depth() refuse later rather than here -- the document
        # is not wrong, the deployment is incomplete, and only the key that
        # needs the depth is in a position to say so.
        if frozen.handler.depth is not MISSING:
            if got.handler.depth != frozen.handler.depth:
                raise QosConfigError(
                    "qos.profiles.%s.handler.depth is %r, 11 S2.4.2 freezes it "
                    "at %r" % (name, got.handler.depth, frozen.handler.depth))


# _parse_set -- the binding-level override clause, and the one safety check the
# item's criterion names by name.
#
# congestion_control and reliability are refused with the field table's own
# reason. What a set clause could otherwise do: flip Q3_cmd on event/** to drop,
# and the replay cursor described in S2.4.5 第 6 条 starts advancing past events
# that never left the machine. Nothing logs it; the events are simply gone, and
# the ones lost are the ones produced when the system was most abnormal.
def _parse_set(raw: Any, profile: QosProfile, where: str) -> Tuple[str, Any]:
    """Return (priority, depth) after applying a set clause to profile."""
    block = _require_mapping(raw, where)
    # Start from the profile, so a clause that mentions neither field is a no-op
    # rather than an implicit reset to something else.
    priority = profile.priority
    depth = profile.handler.depth
    for field in block:
        # The two forbidden fields are named individually rather than caught by
        # "not in the allowed list", so the message can carry the contract's own
        # reason instead of a bare list of what was permitted.
        if field in ("congestion_control", "reliability"):
            raise QosConfigError("%s overrides %s, forbidden by 11 S2.4.7"
                                     % (where, field))
        if field not in ("priority", "handler"):
            raise QosConfigError("%s has unknown field %r, see 11 S2.4.7"
                                     % (where, field))
    if "priority" in block:
        # Checked against the closed set here as well as in the profile parser:
        # a set clause is a second door into the same field, and a door nobody
        # checks is the one that gets used.
        if block["priority"] not in PRIORITIES:
            raise QosConfigError("%s.priority %r is outside %s"
                                     % (where, block["priority"], PRIORITIES))
        priority = block["priority"]
    if "handler" in block:
        handler = _require_mapping(block["handler"], where + ".handler")
        _require_exact_keys(handler, ("kind", "depth"), where + ".handler")
        # kind may RESTATE the profile's kind and may not CHANGE it. The header
        # records why this reading and not a strict whitelist: the frozen
        # bindings themselves write kind, and every one of them writes the kind
        # the profile already has, so a whitelist would make the contract's own
        # bindings unloadable.
        if handler["kind"] != profile.handler.kind:
            raise QosConfigError(
                "%s.handler.kind would change %s from %r to %r, see 11 S2.4.7"
                % (where, profile.name, profile.handler.kind, handler["kind"]))
        depth = _parse_depth(handler["depth"], where + ".handler.depth")
    return priority, depth


# _Binding -- one parsed row. Private, because a caller holding one could pass
# it back into a table it did not come from and pair a match expression with a
# profile the audit never saw.
@dataclass(frozen=True)
class _Binding:
    """One parsed row of the ordered bindings array."""

    #: Position in the document. Carried so a resolution can report it, which is
    #: what makes the ordering itself assertable.
    index: int
    #: The expression as written in the document, kept verbatim. Normalising it
    #: -- collapsing a ** or reordering anything -- would make a resolution
    #: report a rule the reader cannot find by grepping the file they edited.
    match: str
    #: The profile row itself, not its name. Resolving through the name on every
    #: call would mean a dictionary lookup that can fail, in a path where
    #: failure has already been ruled out at load; holding the row makes that
    #: impossible rather than merely unlikely.
    profile: QosProfile
    #: priority and depth after the set clause, resolved once at load rather
    #: than on every resolve call. Resolution runs at declare time on every key
    #: a process owns, and re-deriving the clause each time would be work whose
    #: only possible outcome is the same answer.
    priority: str
    depth: Any


def _parse_binding(index: int, raw: Any,
                   profiles: Mapping[str, QosProfile]) -> _Binding:
    """One bindings[i] entry, validated against the profiles it may name."""
    where = "qos.bindings[%d]" % index
    block = _require_mapping(raw, where)
    # Unknown fields first, so a typo such as "profil" is reported as an unknown
    # field rather than as a missing one -- the two send the reader to different
    # halves of the line.
    for field in block:
        if field not in ("match", "profile", "set"):
            raise QosConfigError("%s carries unknown field %r" % (where, field))
    for field in ("match", "profile"):
        if field not in block:
            raise QosConfigError("%s is missing %r" % (where, field))
    match = block["match"]
    if not isinstance(match, str) or not match:
        raise QosConfigError("%s.match must be a non-empty string" % where)
    _check_match_expression(match, where)
    name = block["profile"]
    # A binding naming a profile that does not exist would otherwise fail at
    # resolve time, for whichever key happened to hit it first -- which may be
    # long after startup and on a machine nobody is watching.
    if name not in profiles:
        raise QosConfigError("%s.profile %r is not in qos.profiles"
                             % (where, name))
    profile = profiles[name]
    # The profile's own values are the starting point, so a binding without a
    # set clause carries the profile unchanged. Writing None here and resolving
    # it later would give the resolver a second "absent" to distinguish from
    # MISSING, and two sentinels for two different kinds of absence is how a
    # consumer ends up testing the wrong one.
    priority, depth = profile.priority, profile.handler.depth
    if "set" in block:
        priority, depth = _parse_set(block["set"], profile, where + ".set")
    return _Binding(index, match, profile, priority, depth)


# _check_match_expression -- refuse a pattern that can never match, and refuse
# one this matcher would read differently from Zenoh.
#
# The dead-pattern case is not hypothetical: 11 S2.2.12 already records a binding
# left behind under a renamed key and calls it 死配置, 应删除. A pattern that
# matches nothing costs nothing at run time and is invisible in every test, which
# is exactly why it survives -- so load time is the only moment anything can
# notice it.
def _check_match_expression(match: str, where: str) -> None:
    """Raise unless match is rooted at xbrain and uses only * and **."""
    chunks = match.split("/")
    if any(c == "" for c in chunks):
        raise QosConfigError("%s.match %r has an empty chunk" % (where, match))
    if chunks[0] != KEY_ROOT:
        raise QosConfigError("%s.match %r is not rooted at %s, so it "
                             "matches no key" % (where, match, KEY_ROOT))
    for chunk in chunks:
        if chunk in (_WILD_CHUNK, _WILD_ANY):
            continue
        # Any other wildcard character means a syntax this matcher does not
        # implement. Approximating it would make the resolver and the router
        # disagree about which rule wins, and the symptom would be a key
        # carrying different QoS than the golden vectors say it does -- with
        # both sides insisting they are right.
        if any(ch in chunk for ch in "*$?"):
            raise QosConfigError(
                "%s.match %r uses a wildcard beyond * and **, see 11 S2.4.7"
                % (where, match))


# ---------------------------------------------------------------------------
# The table.
# ---------------------------------------------------------------------------

# A class and not a closure over the parsed document. The two would behave the
# same, and the class is what lets a caller hold the table, inspect it, and hand
# the same instance to a startup self-check -- INF-ZN-5 walks every declaration a
# process makes against the table it will actually use, and a closure would give
# it a function with no way to ask what is inside.
class QosTable:
    """Profiles plus ordered bindings, resolving a key to the QoS it must carry.

    Built through load_qos_table, never by calling this directly with unchecked
    data: every invariant the resolver relies on is established by the loader,
    and a constructor that accepted raw dicts would give a caller a way past all
    of them at once.
    """

    def __init__(self, profiles: Mapping[str, QosProfile],
                 bindings: Sequence[_Binding],
                 rt_override_in_config: Optional[Mapping[str, Any]]):
        # dict() and tuple() copies, not the caller's containers. The loader is
        # the only caller today, but a table that aliased its input would let a
        # later edit to that document change the QoS of a running process --
        # and QoS is a frozen face precisely because it is not hot-updatable
        # (11 S7.6 returns E_CONFIG_LOCKED for the whole qos section).
        self._profiles: Dict[str, QosProfile] = dict(profiles)
        self._bindings: Tuple[_Binding, ...] = tuple(bindings)
        #: What the document said about rt_override, for audit only. Kept so a
        #: caller can report "the deployment did not carry it" without this
        #: module deciding what that means; resolution never reads it, which is
        #: the property mutant 3 in the item's criterion exists to prove.
        self.rt_override_in_config = rt_override_in_config

    @property
    def profiles(self) -> Dict[str, QosProfile]:
        """A copy of the profile table, so a caller cannot edit ours.

        A copy on every call, which is wasteful and deliberate: the alternative
        is handing out the live mapping, and a consumer that adjusted one entry
        would change the QoS every later resolve returns, from a call site that
        looks like a read. The profile rows themselves are frozen dataclasses,
        so the copy only has to be one level deep.
        """
        return dict(self._profiles)

    @property
    def match_expressions(self) -> Tuple[str, ...]:
        """The bindings' match expressions, in document order.

        Order preserved, because a caller inspecting them is almost always
        checking the ordering -- a set or a sorted list would answer a question
        nobody asked.
        """
        return tuple(b.match for b in self._bindings)

    # resolve -- first match wins, and no match is an error.
    #
    # The loop returns on the first hit rather than collecting candidates: 11
    # S2.4.7 says 首个匹配生效 and the ordering comment gives the reason in the
    # form of the v0.3 estop repair. Scoring the candidates by specificity would
    # be a different algorithm that happens to agree on most keys -- including,
    # unhelpfully, on all of the ones a reader would check by hand.
    def resolve(self, key: str) -> QosResolution:
        """The QoS for one fully-qualified key, or raise.

        Read-only and free of shared state, so it is safe to call from more
        than one thread. That is not incidental: 11 S2.4.8 A-1 pushes event
        publication onto its own thread precisely so a Q3 block cannot stall a
        Q1 publisher, and both threads have to be able to ask what QoS their
        keys carry. A table that memoised resolutions into a dict would put a
        mutation back in the middle of that.
        """
        # The chunks are computed once and passed down to _apply, which needs
        # the plane for the QOS-C1 test. Splitting the key twice would be
        # cheap; the reason not to is that the second split would not be behind
        # the validation, so a later edit could end up testing the plane of a
        # key parse_full_key had already rejected.
        chunks = parse_full_key(key)
        for binding in self._bindings:
            if key_expr_matches(binding.match, key):
                return self._apply(key, chunks, binding)
        # No fallback. See the header: a default here is the S2.4.0 failure the
        # section exists to close, and it is silent -- the key publishes anyway,
        # out of whichever queue the binding default picked.
        raise QosViolation("no qos.bindings rule matches key %r, and 11 S2.4.0 "
                           "forbids the Zenoh defaults" % (key,))

    # _apply -- profile, then set clause, then QOS-C1, in that order.
    #
    # The order is load-bearing. S2.4.7 says the override is 无条件应用, 且不可由
    # 部署配置关闭; applying it before the set clause would let a set clause
    # restore priority or handler on an RT key, which is that disable by another
    # name and would look like ordinary configuration while doing it.
    def _apply(self, key: str, chunks: Tuple[str, ...],
               binding: _Binding) -> QosResolution:
        """Build the resolution for a key that matched this binding."""
        profile = binding.profile
        congestion = profile.congestion_control
        priority = binding.priority
        # The kind comes from the profile and the depth from the binding: the
        # set clause may supply a depth, and _parse_set has already refused any
        # clause that would have changed the kind.
        handler = HandlerSpec(profile.handler.kind, binding.depth)
        # QOS-C1's trigger, written as the rule itself rather than as "the
        # profile is Q3_cmd". The two select the same keys -- Q3_cmd is the only
        # frozen profile carrying block, and _audit_against_frozen keeps it that
        # way -- but S2.4.3 states the rule as "rt/ 前缀下的任何 key ... 不得为
        # block", and writing it that way leaves no branch that can never be
        # reached and no way for a future block-carrying profile to slip onto
        # the RT plane by not having that particular name.
        applied = chunks[2] == _RT_PLANE and congestion == BLOCK
        if applied:
            congestion = RT_OVERRIDE.congestion_control
            priority = RT_OVERRIDE.priority
            # The whole handler, kind included: S2.4.2's Q3-rt row is FIFO(32),
            # so keeping the matched profile's FIFO(256) here would leave the
            # depth wrong while the rest of the row looked overridden.
            handler = RT_OVERRIDE.handler
        # reliability and express come from the profile in every case, override
        # or not. S2.4.7's rt_override block carries neither field, and S2.4.2's
        # Q3-rt row reads reliable with express false -- exactly Q3_cmd's own
        # values. Copying them from the override would mean inventing two
        # values the contract does not state, which is how a table acquires a
        # number nobody can trace to a section.
        return QosResolution(
            key=key, binding_index=binding.index, binding_match=binding.match,
            profile_name=profile.name, congestion_control=congestion,
            priority=priority, reliability=profile.reliability,
            express=profile.express, handler=handler,
            rt_override_applied=applied)


# load_qos_table -- the only supported way to build a QosTable.
#
# It takes a document, never a path. Run-time processes read the resolved
# artefacts under /run/xbrain/resolved (10 S5.4.1, whose own text calls the
# once-at-freeze-time expansion 最关键的一条结构性决定), and a loader that opened
# configs/ itself would be the second place references get expanded -- which is
# the thing that section forbids, because two expansions produce two answers and
# neither side can tell which one the other used.
def load_qos_table(document: Any) -> QosTable:
    """Build a QosTable from a resolved qos document (the S2.4.7 shape)."""
    doc = _require_mapping(document, "qos")
    # Unknown top-level fields are refused rather than ignored. A section
    # someone added expecting it to take effect is worse than one that is
    # rejected: it reads, to the next person, as evidence that the feature
    # exists.
    for field in doc:
        if field not in ("profiles", "bindings", "rt_override"):
            raise QosConfigError("qos carries unknown field %r" % (field,))
    # profiles and bindings are reported by name when absent or null, because
    # null is their state in configs/common.yaml today: the file declares both
    # key positions and assigns neither, so the stack refuses to start until a
    # deployment supplies them. The message has to name the key path, or the
    # operator is left to work out which of nineteen files is incomplete.
    for field in ("profiles", "bindings"):
        if doc.get(field) is None:
            raise QosConfigError("qos.%s is absent or null, see 11 S2.4.7"
                                 % field)
    raw_profiles = _require_mapping(doc["profiles"], "qos.profiles")
    # Every profile is parsed, not only the ones some binding names. An unused
    # profile with a broken knob would otherwise sit in the document until the
    # day a binding started pointing at it, and the failure would then be
    # attributed to the binding that was just added rather than to the profile
    # that had been wrong all along.
    profiles = {name: _parse_profile(name, block)
                for name, block in raw_profiles.items()}
    # The audit runs before the bindings are parsed, so a document with a
    # redefined profile is refused for that reason and not for whatever the
    # bindings then do with it.
    _audit_against_frozen(profiles)
    raw_bindings = doc["bindings"]
    # A list specifically, not any sequence: a mapping would iterate its keys
    # and a string would iterate its characters, and both would "work" far
    # enough to produce a confusing error later. Order is the whole semantics
    # here, so the container has to be one that has an order.
    if not isinstance(raw_bindings, list) or not raw_bindings:
        raise QosConfigError("qos.bindings must be a non-empty ordered list, "
                             "see 11 S2.4.7")
    bindings = [_parse_binding(i, raw, profiles)
                for i, raw in enumerate(raw_bindings)]
    rt_override = doc.get("rt_override")
    # Absent is tolerated and contradictory is refused; see _audit_rt_override.
    if rt_override is not None:
        _audit_rt_override(rt_override)
    return QosTable(profiles, bindings, rt_override)


# _audit_rt_override -- compare, never consume.
#
# S2.4.7's field table says the configured value is 仅供审计比对. So a document
# that agrees is accepted, one that disagrees is refused, and one that omits the
# block resolves exactly the same as one that carries it -- which is the only way
# the claim "hard-coded in the implementation" can be tested at all. If the
# loader consumed the configured value instead, a deployment could switch QOS-C1
# off by editing it, and the RT plane would be back on block with nothing to say
# so.
#
# The field table does mark rt_override 必填. That obligation belongs to the
# freeze-time configuration check, which inspects the document as a document;
# this function must not enforce it, because a loader that refused an absent
# block would make the hard-coding claim unverifiable -- both a hard-coded and a
# consumed implementation would raise.
def _audit_rt_override(raw: Any) -> None:
    """Raise unless the document's rt_override equals the hard-coded one."""
    block = _require_mapping(raw, "qos.rt_override")
    _require_exact_keys(block, ("congestion_control", "priority", "handler"),
                        "qos.rt_override")
    handler = _parse_handler(block["handler"], "qos.rt_override.handler")
    got = RtOverride(block["congestion_control"], block["priority"], handler)
    if got != RT_OVERRIDE:
        # Both values go in the message. A difference is a belief mismatch, and
        # the person commissioning the machine acts on the document they wrote,
        # so they need to see what the stack actually applies beside it.
        raise QosConfigError(
            "qos.rt_override is %r but the implementation applies %r, "
            "see 11 S2.4.7" % (got, RT_OVERRIDE))


# Exported names, in four groups.
#
#   * the closed sets and KEY_ROOT, so a caller validating a value elsewhere
#     compares against this table rather than a private copy of it;
#   * the value types, so a consumer can annotate what it holds;
#   * FROZEN_PROFILES and RT_OVERRIDE, for the cross-language metatest and for
#     tooling that dumps the table -- runtime code should be resolving a key,
#     not reading a profile directly, because the profile is only half the
#     answer once a set clause or QOS-C1 is involved;
#   * load_qos_table, resolve's two error types, and the two pure functions.
#
# The private parse helpers stay private: a caller that wants to validate a
# fragment should hand the whole document to load_qos_table, because the
# interesting checks (the frozen-table audit, the first-match ordering) are
# properties of the document as a whole and not of any one field. Exporting
# _parse_profile would let a consumer validate a profile in isolation and
# conclude the document is sound when the audit would have refused it.
__all__ = ["CONGESTION_CONTROLS", "PRIORITIES", "RELIABILITIES", "HANDLER_KINDS",
           "KEY_ROOT", "BLOCK", "DROP", "HandlerSpec", "QosProfile",
           "RtOverride", "FROZEN_PROFILES", "RT_OVERRIDE", "QosConfigError",
           "QosViolation", "QosResolution", "QosTable", "key_expr_matches",
           "parse_full_key", "load_qos_table"]
