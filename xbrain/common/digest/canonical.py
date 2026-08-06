"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: canonical.py
Brief: Canonical JSON serialisation, the one both common_digest and crc32 rest on

Description:
What problem this solves. 10 S5.4.4 requires common_digest to come out identical
in Python and in C++, and 11 S9A.2 requires the same of the fence crc32. Neither
is achievable unless the bytes fed to the hash are fully determined by the data,
so every degree of freedom an ordinary JSON writer enjoys has to be nailed down:
key order, whitespace, escaping, and the printed form of numbers.

The rules, verbatim from 10 S5.4.4:
  * keys sorted by UTF-8 byte order, ascending
  * no whitespace anywhere
  * UTF-8 emitted directly; non-ASCII is NOT escaped to \\u
  * integers carry no decimal point; floats print via %.17g with trailing zeros
    removed
  * lists keep their order, because order is part of the meaning (R-5)

On "UTF-8 byte order", and what it does and does not rule out. UTF-8 is
order-preserving: comparing two strings by their UTF-8 bytes gives the same
answer as comparing them by Unicode code point, for every pair, always. So the
rule does NOT distinguish this implementation from one that sorted on the str
directly, and a mutation that switches to code point order is a no-op -- stated
here because it is easy to read the rule as forbidding something it does not.

What the rule does rule out is UTF-16 code unit order, which is the natural
ordering in Java, JavaScript, C# and any C++ port keyed on std::wstring under
Windows. There a character above the Basic Multilingual Plane is a surrogate
pair beginning at 0xD800, so it sorts BELOW the fullwidth Latin block instead of
above it. Sorting on the encoded bytes here makes that difference impossible to
introduce by accident. tests/common/test_digest.py runs that mutation and pins
the premise.

What this module deliberately does NOT do:
  * it does not hash anything -- see digest.py
  * it does not decide WHICH subtree to serialise; common_digest covers common.*
    only, and the caller is responsible for passing that subtree

A note on numbers. Under these rules 1 and 1.0 serialise identically, because
%.17g on an integral float yields "1". That is intended: the digest tracks
values, not the Python types they happen to arrive in, and a YAML loader that
returns 1 where another returns 1.0 must not change the digest.
"""

from typing import Any

#: Marker written for JSON null. Spelled out rather than inlined so the three
#: literal spellings (null, true, false) sit together and cannot drift apart
#: from the C++ side by a stray capital letter.
_NULL = "null"
_TRUE = "true"
_FALSE = "false"


def format_number(value: Any) -> str:
    """One number in canonical form.

    bool is checked before int on purpose. In Python bool IS a subclass of int,
    so an unguarded isinstance(value, int) would render True as "1" and the C++
    side, where the two types are unrelated, would render it as "true" -- a
    cross-language mismatch that no single-language test can see.
    """
    if isinstance(value, bool):
        return _TRUE if value else _FALSE
    if isinstance(value, int):
        # Integers print as themselves. No decimal point, no exponent, and no
        # thousands separator regardless of locale -- str() on an int is
        # locale-independent in Python, and the C++ side must use a plain
        # integer conversion rather than a locale-aware stream.
        return str(value)
    if isinstance(value, float):
        # %.17g is the shortest format that round-trips every IEEE 754 double,
        # which is what makes the two languages agree: both are printing the
        # same bits under the same rule rather than each choosing a "nice"
        # rendering.
        text = "%.17g" % value
        # The strip below is what S5.4.4 asks for second, and under %g it is
        # UNREACHABLE: C99 7.21.6.1 already removes trailing zeros from the
        # fractional part. Kept for fidelity to the section and so the rule
        # survives a change of format, but no golden vector covers it and no
        # mutation of it can be observed going red. See
        # test_the_trailing_zero_strip_is_unreachable_under_the_current_format,
        # which asserts the premise so this comment cannot quietly go stale.
        if "." in text and "e" not in text and "E" not in text:
            text = text.rstrip("0").rstrip(".")
        return text
    raise TypeError("not a number: %r" % (value,))


def canonical_json(node: Any) -> str:
    """Serialise a parsed tree to the canonical form.

    Recursive by structure rather than by a generic encoder, because the
    encoder settings needed here (sort_keys, separators, ensure_ascii, and a
    custom float repr) cannot all be expressed through json.dumps: it offers no
    hook for float formatting that also applies inside containers.
    """
    if node is None:
        return _NULL
    if isinstance(node, (int, float)):
        # bool arrives here too, because it subclasses int -- and that is fine:
        # format_number checks for it first and returns true/false. An earlier
        # draft repeated that check here; a mutation test showed deleting the
        # copy changed nothing, so it was dead code masked by the real guard and
        # it was removed. One guard that is exercised beats two where only one
        # can ever fire.
        return format_number(node)
    if isinstance(node, str):
        return _quote(node)
    if isinstance(node, (list, tuple)):
        # tuple as well as list. A YAML loader configured for immutability
        # returns tuples, and rejecting one would make the digest depend on
        # which loader the caller happened to use rather than on the data.
        # Order preserved. R-5 makes a list a unit that is replaced whole, and
        # qos.bindings is first-match-wins, so reordering changes behaviour --
        # the digest has to notice it.
        return "[" + ",".join(canonical_json(item) for item in node) + "]"
    if isinstance(node, dict):
        # Sort on the UTF-8 BYTES of the key. Sorting on the str itself would
        # give the same answer -- UTF-8 is order-preserving, see the module
        # docstring -- but encoding first is what the section says and what
        # matches std::map on the C++ side, and it is the spelling that cannot
        # accidentally become UTF-16 order in a future port.
        items = sorted(node.items(), key=lambda kv: kv[0].encode("utf-8"))
        return "{" + ",".join(_quote(k) + ":" + canonical_json(v) for k, v in items) + "}"
    raise TypeError("cannot serialise %r of type %s" % (node, type(node).__name__))


def _quote(text: str) -> str:
    """A JSON string with the minimum escaping the rules allow.

    Only the characters JSON itself requires are escaped. Non-ASCII goes out as
    UTF-8 directly, which is what "非 ASCII 不做 \\u 转义" asks for -- and it is
    also the only choice that keeps the two languages aligned without agreeing
    on a \\u case convention (\\u00e9 versus \\u00E9 hash differently).
    """
    out = ['"']
    for ch in text:
        # The five short escapes JSON defines, in the order RFC 8259 lists them
        # minus the two optional ones. \b and \f are deliberately absent: they
        # fall through to the numeric branch below, which is legal JSON and one
        # rule fewer for the C++ side to reproduce. Adding them later would
        # change the bytes and therefore every digest, so they stay out.
        if ch == '"':
            out.append('\\"')
        elif ch == "\\":
            out.append("\\\\")
        elif ch == "\n":
            out.append("\\n")
        elif ch == "\r":
            out.append("\\r")
        elif ch == "\t":
            out.append("\\t")
        elif ord(ch) < 0x20:
            # The remaining control characters have no short form and must be
            # escaped numerically. Lower case hex, fixed at four digits, so the
            # C++ side has exactly one form to match.
            out.append("\\u%04x" % ord(ch))
        else:
            # Everything else, including every character of a multi-byte UTF-8
            # sequence, goes out untouched. The encode to UTF-8 happens once at
            # the end, on the whole string, so a character is never split.
            out.append(ch)
    out.append('"')
    return "".join(out)
