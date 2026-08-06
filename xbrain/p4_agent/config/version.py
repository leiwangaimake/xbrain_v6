"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: version.py
Brief: Ordering for cmdset.min_agent_version, with every guess refused

Description:
What problem this solves. 16 S12.4's version table states the rule in one line
-- grep 启动校验 -- : P4 checks min_agent_version at startup and refuses to start
with a clear error when it is not satisfied. The rule is one line; deciding what
"satisfied" means is not, and this file exists so that decision is made once,
visibly, with the parts the documents do not answer marked as unanswered rather
than papered over.

Why the check matters at all. 16 S12.4 gives the increment rules: adding an
intent bumps minor and stays backward compatible, while changing semantics or
deleting an intent bumps major and needs review. So a cmdset whose
min_agent_version is above the running implementation is one that contains
instructions this implementation will route wrongly -- not instructions it
merely does not know. Silently continuing means the robot executes a command
that means something else now. That is why the failure is a refusal to start.

What this module does NOT decide, stated so nobody later cites it as settled:

  * *** The implementation's OWN version is not defined in any volume. A grep
    for min_agent_version and agent_version across docs/ finds only 16 S12.4 and
    16 S14 -- both of them the REQUIREMENT side. Nothing anywhere states what
    version this implementation is. So this module never supplies one: the
    caller passes it in, and a caller that has none cannot call the function at
    all. Inventing a constant here would have made the check pass by
    construction, which is CLAUDE.md 3.2 form 1 -- an assertion a do-nothing
    implementation satisfies.

  * *** Whether a HIGHER major satisfies a lower minimum is not stated either.
    The literal reading of "min" is numeric ordering, and that is what is
    implemented; a cmdset requiring 1.0 is accepted by an agent claiming 2.0.
    The other reading -- major changes break compatibility both ways, so 2.0
    should refuse 1.0's cmdset -- is defensible and would follow from 16 S12.4's
    "改语义/删指令 major", but it is about cmdset.version, not about the agent,
    and no volume connects the two. Choosing it here would be resolving a
    silence, which CLAUDE.md 9.1 forbids. Recorded, not decided.

*** Malformed input raises. It never compares as zero and never compares as
"satisfied". Both fallbacks have the same shape of failure: "1.0-rc1" or a
version read from the wrong key would be silently treated as some number, the
check would pass, and the mismatch would surface as an intent being routed to
the wrong handler at run time. A parse failure is a configuration defect and the
configuration layer's whole contract is to refuse those at startup.

Scope. This module is ordering and parsing only. It does not read
configuration, does not know which key the value came from, and raises
ValueError rather than a configuration error type -- loader.py owns the
vocabulary of "which key path, in which snapshot", and duplicating that here
would create a second place where the message is composed.
"""

from typing import Tuple

#: The most components a version string may carry: major.minor.patch.
#:
#: Bounded rather than open-ended because 16 S12.4 names exactly three levels
#: (major / minor / patch) and nothing in the volume produces a fourth. An
#: unbounded parser would accept "1.0.0.0.7" and order it against "1.0.0" in a
#: way nobody has ever specified -- accepting input whose meaning is undefined
#: is how a checker starts answering questions it was not asked.
MAX_COMPONENTS = 3


def parse_version(text: str, what: str = "version") -> Tuple[int, ...]:
    """Dotted numeric version to a tuple of ints, or raise ValueError.

    `what` names the thing being parsed so the message says which of the two
    versions was malformed. With one shared message a reader cannot tell
    "the configuration is wrong" from "the implementation passed me rubbish",
    and those need different people to fix them.

    Deliberately strict about every shape this project could plausibly meet:
    a leading "v", a pre-release suffix, whitespace, an empty component. Each
    of them is refused rather than tolerated, because tolerance here means
    guessing the intended ordering of something the documents never described.
    """
    # A non-string is refused before anything else. YAML happily produces a
    # float for an unquoted 1.0, and float("1.0") stringifies back to "1.0" --
    # so an implementation that accepted numbers would work on 1.0 and then
    # order 1.10 BELOW 1.9, because float comparison is not version
    # comparison. Refusing the type makes that impossible instead of subtle.
    if not isinstance(text, str):
        raise ValueError(
            "%s must be a string, got %r of type %s; an unquoted 1.10 in YAML "
            "parses as the float 1.1 and would then order below 1.9"
            % (what, text, type(text).__name__))
    # Compared against the raw text, not a stripped copy. " 1.0" almost
    # certainly means 1.0, and accepting it would be harmless exactly once --
    # after which the tolerance is the reason nobody notices that the value is
    # being assembled by string concatenation somewhere upstream.
    if text != text.strip() or not text:
        raise ValueError(
            "%s is %r; a version must be non-empty and carry no surrounding "
            "whitespace" % (what, text))

    parts = text.split(".")
    if not 1 <= len(parts) <= MAX_COMPONENTS:
        raise ValueError(
            "%s is %r, which has %d dot-separated components; 16 S12.4 defines "
            "major, minor and patch and nothing beyond them"
            % (what, text, len(parts)))

    numbers = []
    for part in parts:
        # str.isdigit() rather than int(): int() accepts "+1", "-1", "1_0" and
        # a handful of Unicode digit forms, every one of which would parse to
        # something plausible and order somewhere unintended.
        if not part.isdigit():
            raise ValueError(
                "%s is %r; component %r is not a plain decimal number. "
                "Pre-release and build suffixes are refused rather than "
                "ignored -- their ordering is not defined by any volume"
                % (what, text, part))
        numbers.append(int(part))
    return tuple(numbers)


def satisfies(agent_version: str, minimum: str) -> bool:
    """True when the agent version is at least the minimum.

    Both sides are parsed first, so a malformed value on EITHER side raises
    instead of producing a verdict. A version check that returns False on
    garbage looks safe -- it refuses to start -- but it is the permanently-red
    shape, and the repair somebody reaches for is to make the parse failure
    return True.

    Comparison is component-wise with the shorter side zero-extended, which is
    what makes 16 S14's pair work: cmdset.min_agent_version is written "1.0"
    with two components while an implementation may call itself "1.0.0". Under
    plain tuple comparison (1, 0) < (1, 0, 0) is True, so an agent at exactly
    1.0.0 would FAIL a minimum of 1.0 -- a check that rejects the very pair the
    document shows. Zero-extension is the fix and it is not cosmetic.
    """
    got = parse_version(agent_version, "agent version")
    want = parse_version(minimum, "cmdset.min_agent_version")
    width = max(len(got), len(want))
    # Padding both sides to the same width. See the docstring for the exact
    # pair this exists for.
    got_padded = got + (0,) * (width - len(got))
    want_padded = want + (0,) * (width - len(want))
    # >= and not >. The minimum is inclusive: 16 S14 pairs a cmdset whose
    # min_agent_version is "1.0" with an implementation that is meant to run it,
    # so the boundary case is the normal case rather than an edge.
    return got_padded >= want_padded
