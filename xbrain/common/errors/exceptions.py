"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: exceptions.py
Brief: Exception types for the shared error-code layer

Description:
CLAUDE.md 4.5 forbids raising bare Exception and forbids bare except. Every
failure this package can produce gets a named type so callers catch exactly what
they mean to handle.

XbrainError carries the closed-set code rather than a free-form string, so an
exception crossing a process boundary still maps onto 11 S13 without anyone
re-deriving it.

Why the named types earn their keep. The shape a reviewer will propose instead
is one generic error with a code field, caught by "except Exception". That
collapses two failures needing opposite handling: a contract failure, which the
caller reports and may retry, and a programming defect (AttributeError,
TypeError) which must reach the process fault path untouched. Under CLAUDE.md
4.4 a raise inside the P1 loop is supposed to produce zero speed for that tick
plus a fault record, with the loop still ticking on the next one. A handler wide
enough to swallow the defect leaves the loop running on assumptions that have
already failed, which is the fail-silent direction this project keeps closing.

What this module does NOT do, and must not grow into:

  1. It does not map codes onto HTTP status. 11 S8.13.5 is titled
     "错误映射(网关唯一实现点)" and the left column of its table is HTTP status:
     the gateway is the single implementation point. A to_http() helper here
     would give the system a second mapping to keep in step with the first.
  2. It does not validate the code it is handed. Reasons under XbrainError.
  3. It imports nothing outside the standard library, and nothing outside
     typing today. The package __init__ next door states why: this layer is
     imported by every runtime process, including ones that start before any
     virtualenv is guaranteed. An import failure here happens before the
     machinery that would have described the failure exists.

Why bare E_* literals are tolerated in this one file. CLAUDE.md 3.5 forbids E_*
string literals outside common/errors/, and this file is inside it. The
mechanical reason it could not follow the rule even if asked: the package
__init__ does "from .exceptions import ClosedSetViolation, UnknownErrorCode,
XbrainError" while it is still executing, so importing the generated names back
out of the package would be an import cycle. Keep the count of literals here
small, because each one is a name the CI static rule cannot check on our behalf.

Since 2026-08-06 that tolerance is DECLARED rather than assumed. The CI rule is
scripts/lint/no_literal_ecode.py and it fails on every E_* literal in the scanned
trees; the two here carry a line-level ECODE-OK(cycle) marker giving the reason
above, on the same declarative mechanism no_config_source_read.py uses for the
one place that is allowed to name the configuration source. An exemption written
down can be argued with in review; one expressed as a path check inside a scanner
cannot be found at all.

Cross-language note. These types are Python only. What crosses a process or a
language boundary is the code string itself, which is why code is typed str and
not an enum or a NewType: a wrapper would have to be unwrapped at every
serialisation point, and the value still has to be checked against the closed
set on the way in regardless.
"""

from typing import Optional, Sequence


# The base type exists so one clause, "except XbrainError", catches the whole
# family of failures this system raises on purpose -- including subclasses
# declared elsewhere, such as ConfigLayerError in common/config/layers.py --
# without also catching the bugs that must not be caught.
class XbrainError(Exception):
    """Base for every error this system raises deliberately.

    * 11 S8.13.5 is titled "错误映射(网关唯一实现点)" -- mapping a failure onto a
    closed-set code is the gateway's job, so AI services returning free-form
    detail are not in violation. Do not push E_* down into services/ to satisfy
    this type; construct it at the gateway instead.
    """

    # Argument semantics, since none of the three is obvious from its type:
    #   code     a closed-set value from 11 S13.4~S13.15, or OK. NOT checked
    #            here, see the comment on the super().__init__ call below.
    #   message  English, one line. It ends up in log output and in the string
    #            form of the exception, so CLAUDE.md 2.1 forbids Chinese in it.
    #            Empty is a normal value and means the code says it all.
    #   detail   the structured payload EC-3 talks about. A code's row in
    #            codes.yaml can carry detail: required. This class cannot enforce
    #            that, because deciding it needs the code's row, which lives in
    #            the package that imports this module. Enforcement belongs at
    #            the serialisation boundary, beside errors.detail_requirement().
    def __init__(self, code: str, message: str = "", detail: Optional[dict] = None):
        self.code = code
        # "or {}" here is NOT the default-fallback CLAUDE.md 3.1 bans. That rule
        # covers safety numbers under common.safety.* and common.spec.*, where a
        # falsy 0.0 is a real calibrated value and replacing it with a default is
        # how a robot ends up limited to v_max = 0 with nothing logged. detail has
        # no falsy-but-real value: None and {} both mean "nothing structured to
        # add", so collapsing them discards nothing.
        # NOTE a non-empty detail is kept by reference, not copied. A caller that
        # goes on mutating that dict after raising changes what finally gets
        # logged, which reads as the exception reporting something it never saw.
        self.detail = detail or {}
        # The code is prefixed onto the string form because the overwhelmingly
        # common handler prints str(e) and nothing else. Without the prefix the
        # one part a machine downstream can branch on stays locked in .code and
        # never reaches the log line a human is actually reading.
        # Deliberately no membership check of code against ALL_CODES. Two
        # reasons, either sufficient: it would need the package that imports this
        # module (cycle again), and a constructor able to raise would replace the
        # original failure with a second one at the exact moment the first one
        # matters most. Codes get checked where they enter or leave the process,
        # by errors.info(), which is also the only place that has the table.
        super().__init__(f"{code}: {message}" if message else code)


# Read this together with the docstring below, which forbids "mapping it to
# E_INTERNAL" -- while this class raises WITH code E_INTERNAL. Those are opposite
# moves and the difference is whether execution continues: what 11 S13.6 forbids
# is answering an unrecognised code with E_INTERNAL and carrying on, which turns
# a contract violation into an ordinary-looking log line. Raising stops the
# caller, and E_INTERNAL is then the right row, because reaching this type means
# our own process used a code absent from our own generated table -- a stale
# build or a hand-typed string, a defect on this side of the wire. E_SCHEMA would
# send the operator off to read the peer's logs for a fault the peer never had.
class UnknownErrorCode(XbrainError):
    """Raised when a value outside the closed set is used as a code.

    *** 11 S13.6 requires this to raise rather than degrade. Two tempting
    implementations are both forbidden: passing the unknown value through, and
    mapping it to E_INTERNAL. Either one converts a contract violation into
    something that looks like an ordinary failure, and the cloud client -- which
    branches on the code -- can no longer tell them apart.
    """

    def __init__(self, bad: str, known: Sequence[str]):
        self.bad = bad
        # The message names where the closed set lives so a reader can go look.
        # It deliberately does not print the set: that would put a count into
        # logs, and counts are what rots (CLAUDE.md 3.7).
        # known is accepted and then dropped on purpose. It stays in the
        # signature so the authoritative set sits on this stack frame, readable
        # from a debugger stopped here, while never reaching a log line. Do NOT
        # "put it to use" by formatting it into the message -- that writes the
        # set, and its size, into exactly the place 3.7 keeps them out of.
        # NEVER grow this into a "did you mean E_TIMEOUT" suggestion. 11 S13.6
        # requires an out-of-set value to raise with no pass-through and no
        # nearest-neighbour interpretation, and a printed candidate is the first
        # step towards a later patch that quietly applies it.
        #
        # On the exemption marker below: the package __init__ does "from
        # .exceptions import ..." while it is still executing, so importing the
        # generated code name back into this module would be an import cycle.
        # This is one of only two literals the CI rule
        # scripts/lint/no_literal_ecode.py tolerates in the whole tree, and it is
        # tolerated by declaration, not by silence. The marker sits on the
        # literal's own line rather than in this block because the block is not
        # contiguous with it -- super().__init__( intervenes -- and the lint only
        # searches an unbroken run of comment lines, so that a justification
        # written for one occurrence cannot drift onto a later one.
        super().__init__(
            "E_INTERNAL",  # ECODE-OK(cycle): import cycle, see the block above

            f"value {bad!r} is not in the E_* closed set defined by "
            f"11 S13.4~S13.15; see xbrain/common/errors/codes.yaml",
        )


# Scope boundary: this covers every closed set EXCEPT the error codes. Codes are
# a closed set too, but they raise UnknownErrorCode above, so a handler can still
# tell "our own code table is wrong" (a build or deploy defect) apart from "a
# field in this payload is out of range" (a bad message from a peer).
#
# It lives in errors/ rather than in enums/, even though enums/ is the only thing
# raising it today, because xbrain/common/enums/__init__.py does
# "from ..errors.exceptions import ClosedSetViolation" -- the family base has to
# stay in the lowest layer for "except XbrainError" to keep meaning what it says.
class ClosedSetViolation(XbrainError):
    """Raised when any closed set -- not just error codes -- gets a value outside it.

    Shared with xbrain/common/enums/ so callers catch one type for the family.
    """

    def __init__(self, set_name: str, bad: str):
        # 11 S13.6 gives the worked example of the wrong repair: reading a
        # retired cruise/transit gear value as patrol. Substituting the nearest
        # legal value hides the defect that actually happened -- an upstream is
        # still emitting a retired enum -- and the section also forbids resending
        # the offending payload as-is: it must be corrected and resubmitted under
        # a new cmd_id. set_name and bad are fields, not just text inside the
        # message, so a handler can branch on which set failed.
        self.set_name = set_name
        self.bad = bad
        # E_SCHEMA is the correct row rather than a generic internal error: its
        # meaning in codes.yaml lists 枚举值越界 alongside missing required fields
        # and checksum mismatch, which is exactly this condition.
        #
        # ECODE-OK(cycle): same import cycle as UnknownErrorCode above -- this
        # module is imported BY the package that exports the code names, so it
        # cannot import them. Keep the count of literals in this file at two.
        super().__init__("E_SCHEMA", f"value {bad!r} is outside closed set {set_name!r}")
