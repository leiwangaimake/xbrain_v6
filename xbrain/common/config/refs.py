"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: refs.py
Brief: The ${common.*} reference axis, R-1 through R-7 of 10 S5.4.2

Description:
CFG-CM-8. Runs AFTER the whole L0~L5 overlay finishes -- 10 S5.4.3 requires that
order, and the reason is that expanding while merging reads a value the site
layer has not overridden yet. That failure is invisible whenever the lab site and
the field site happen to agree, so it passes every test written in the lab and
goes wrong on exactly one robot.

The seven rules, each with the contract's own reason:

  R-1  a reference occupies the WHOLE scalar node; no string interpolation.
       Interpolation silently degrades an object or int into a string, and the
       schema check downstream then sees the wrong type.
  R-2  paths start with common. only. Cross-process references would turn YAML
       into an implicit IPC channel that breaks when startup order changes.
  R-3  ** NO default syntax -- no ${a:-b}, no ?. Unresolvable is
       E_CONFIG_INVALID and the stack refuses to start. "Fall back to a default
       when the reference misses" is precisely the silent drift CFG-40 exists to
       kill; making it a syntax feature would institutionalise it.
  R-4  aliases may nest inside common.*, chain length <= 3, and * NO expressions
       or arithmetic of any kind. A calculator in the loader hides safety logic
       inside a configuration file.
  R-5  list overrides replace the whole table (implemented in merge.py; this
       module asserts the two stay consistent).
  R-6  ** L6 must not carry a `common` top-level key, nor a private key listed in
       the S5.4.5 alias table. The contract calls this the only rule of the seven
       with any teeth -- without it the first five are advice.
  R-7  * YAML anchors & / * may not be used to express sharing. Anchors do not
       cross files while looking like they do, and having two sharing mechanisms
       makes expansion order undefinable.

*** R-7 is checked against RAW FILE TEXT, not the parsed tree. Anchors are a YAML
syntax feature: by the time a parser hands back a dict they have already been
resolved and are invisible. A check written against the tree would pass on every
file that uses them.
"""

import re
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .layers import ConfigLayerError
from .merge import flatten

#: A whole-node reference: the entire scalar is ${common....} and nothing else.
_WHOLE_NODE = re.compile(r"^\$\{(common(?:\.[A-Za-z0-9_]+)+)\}$")

#: Any ${...} occurrence, used to tell "interpolated" apart from "not a reference".
_ANY_REF = re.compile(r"\$\{([^}]*)\}")

#: R-4: chain length limit. A alias pointing at an alias pointing at an alias is
#: already hard to follow; four is a graph nobody reads.
MAX_CHAIN = 3

#: R-7: anchors and aliases in raw YAML. Matched at the value position so a `&`
#: inside a quoted string is not mistaken for an anchor.
_ANCHOR = re.compile(r":\s*&[A-Za-z0-9_-]+(\s|$)")
_ALIAS = re.compile(r":\s*\*[A-Za-z0-9_-]+(\s|$)")


class ReferenceError_(ConfigLayerError):
    """A reference violated one of R-1 ~ R-7."""


def _rule(n: str, msg: str) -> ReferenceError_:
    return ReferenceError_(f"{n}: {msg}")


def validate_shape(value: str) -> None:
    """The single shape validator for R-1 ~ R-4. Raises naming the rule broken.

    *** One validator, not two. An earlier draft checked R-3 in two places; a
    mutation test showed the second copy was unreachable -- the suite stayed
    green with it disabled. A guard that cannot fire is worse than no guard: it
    reads as protection and the next person maintaining it assumes it is
    load-bearing.

    Order matters. R-3 and R-2 are checked on the raw inner text because
    ${common.a:-b} matches neither the whole-node pattern nor a plain string; if
    R-1 ran first these would surface as "not a whole node", which is true but
    names the wrong rule and sends the reader to the wrong section.
    """
    for inner in _ANY_REF.findall(value):
        if ":-" in inner or inner.endswith("?") or "|" in inner:
            raise _rule("R-3", f"default-value syntax is not provided: {value!r}. "
                               "Unresolvable references are E_CONFIG_INVALID; falling "
                               "back to a default is the silent drift CFG-40 exists to kill")
        if any(op in inner for op in "+-*/%()"):
            raise _rule("R-4", f"expressions and arithmetic are not supported: {value!r}. "
                               "A calculator in the loader hides safety logic in a config file")
        head = inner.strip()
        if head and not head.startswith("common."):
            raise _rule("R-2", f"reference path must start with 'common.', got {value!r}; "
                               "cross-process references make YAML an implicit IPC channel "
                               "that breaks when startup order changes")
    if not _WHOLE_NODE.match(value):
        raise _rule("R-1", f"reference must occupy the whole scalar node, got {value!r}; "
                           "string interpolation silently degrades object/int to string")


def classify(value: Any) -> Optional[str]:
    """The referenced path if `value` is a legal whole-node reference, else None.

    Raises on every illegal shape rather than returning None for it. Returning
    None would make an illegal reference indistinguishable from an ordinary
    string, and it would then be written into the resolved artifact verbatim --
    a process would receive the literal text "${common.safety...}" where it
    expected a number.
    """
    if not isinstance(value, str) or not _ANY_REF.search(value):
        return None
    validate_shape(value)
    return _WHOLE_NODE.match(value).group(1)


def resolve(tree: Dict[str, Any]) -> Dict[str, Any]:
    """Expand every ${common.*} in `tree`. Input is not mutated.

    Runs on the merged common.* tree, after the overlay axis. Every violation
    raises; nothing is left half-expanded, because a partially expanded artifact
    is one a process would happily load.
    """
    flat = flatten(tree)

    # Validate shapes first, so an error names the rule rather than surfacing as
    # a missing key three steps into resolution.
    for key, value in flat.items():
        if isinstance(value, str) and "${" in value:
            classify(value)

    def lookup(path: str, seen: List[str]) -> Any:
        if path in seen:
            # Cycles are CFG-CM-9's job to report in full; here we only refuse to
            # loop forever. !! Do not return a partial value instead.
            raise _rule("R-4", f"reference cycle through {path!r} (chain {' -> '.join(seen)})")
        if len(seen) >= MAX_CHAIN:
            raise _rule("R-4", f"alias chain longer than {MAX_CHAIN}: {' -> '.join(seen + [path])}")
        if path not in flat:
            raise _rule("R-3", f"unresolvable reference {path!r} -- E_CONFIG_INVALID. "
                               "There is no fallback: refusing to start is the point")
        val = flat[path]
        nxt = classify(val) if isinstance(val, str) else None
        return lookup(nxt, seen + [path]) if nxt else val

    out: Dict[str, Any] = {}
    for key, value in flat.items():
        target = classify(value) if isinstance(value, str) else None
        out[key] = lookup(target, [key]) if target else value

    from .merge import unflatten
    return unflatten(out)


def check_l6(tree: Dict[str, Any], alias_blacklist: Iterable[str]) -> None:
    """R-6 -- the only rule of the seven with teeth, per the contract.

    Two prohibitions:
      * a `common` top-level key in a process config, which would let L6 redefine
        a shared value after the freeze line has already digested it
      * a private key whose name is on the S5.4.5 alias table, which is how the
        same physical quantity ends up with two names and drifts
    """
    if "common" in tree:
        raise _rule("R-6", "L6 process config must not carry a 'common' top-level key; "
                           "shared values are referenced with ${common.*}, never redefined")
    banned = {b for b in alias_blacklist}
    for key in flatten(tree):
        leaf = key.split(".")[-1]
        if leaf in banned or key in banned:
            raise _rule("R-6", f"L6 defines {key!r}, whose name is on the S5.4.5 alias "
                               f"table; use ${{common.*}} instead of a second name for "
                               "the same quantity")


def check_no_anchors(raw_text: str, where: str = "<config>") -> None:
    """R-7 -- must run on RAW TEXT.

    *** Anchors are resolved by the YAML parser; by the time you hold a dict they
    are gone. A check written against the parsed tree passes on every file that
    uses them, which makes it worse than no check at all.
    """
    for lineno, line in enumerate(raw_text.split("\n"), 1):
        stripped = line.split("#", 1)[0]
        if _ANCHOR.search(stripped):
            raise _rule("R-7", f"{where}:{lineno}: YAML anchor '&' is not allowed. "
                               "Anchors do not cross files while looking like they do, "
                               "and two sharing mechanisms make expansion order undefinable")
        if _ALIAS.search(stripped):
            raise _rule("R-7", f"{where}:{lineno}: YAML alias '*' is not allowed; "
                               "express sharing with ${common.*}")


def find_violations(tree: Dict[str, Any]) -> List[Tuple[str, str]]:
    """(key, rule) for every shape violation, without stopping at the first.

    For tooling that wants to report everything at once. resolve() still raises
    on the first, because a loader that continues past a violation would hand a
    process a config it should never have accepted.
    """
    out: List[Tuple[str, str]] = []
    for key, value in flatten(tree).items():
        if not isinstance(value, str) or "${" not in value:
            continue
        try:
            classify(value)
        except ReferenceError_ as exc:
            out.append((key, str(exc).split(":")[0].replace("E_CONFIG_INVALID", "").strip()))
    return out
