"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: units.py
Brief: Strong scalar unit types (Mps / Factor / Mps2 / Seconds) that turn a
       dimension mix-up into a type error instead of a silent wrong number

Description:
CFG-CM-18. The speed gate (11 S9.6.2, 12 S6) multiplies a dimensionless factor
into a speed and takes min() across several speeds. When both a speed and a
factor are plain float, mixing them type-checks and runs, and the result is a
number smaller than intended -- the robot silently crawls, with no error
anywhere. That is the CLAUDE.md 3.1 fail-silent failure mode, one dimension off.
This module makes that slip loud.

What it does NOT do. It is not a units library -- no metre, no kg, no
dimensional-analysis engine, and no arithmetic operators at all yet. It carries
the four scalars the motion stack passes around and nothing speculative
(CLAUDE.md 9.3). A caller that needs Mps * Factor -> Mps adds that one method
when it lands, with its own test. Because NO __mul__/__add__/__sub__ exists,
every cross-unit (and unit-with-bare-float) arithmetic expression is a static
error today -- which is precisely the protection, see below.

*** What is and is not caught, stated exactly (CLAUDE.md 3.2 -- no pretending a
guarantee we do not have). Verified in tests/common/test_units.py:

  caught by mypy AND at runtime:
    - a DIRECT cross-unit comparison, `Mps(2) < Factor(3)`: the comparison
      operators are typed to the SAME unit (self-type), so mypy rejects a
      foreign operand, and the isinstance guard raises TypeError at runtime.
    - cross-unit ARITHMETIC, `Mps * Factor`: no such operator exists -> mypy
      [operator] error. NewType(float) would silently degrade this to a float;
      that contrast is the discriminator the item asked for.
    - a Factor passed where a Mps parameter is declared: [arg-type] error.

  caught ONLY at runtime, a real mypy blind spot:
    - `min(Mps(2), Factor(3))`. The criterion literally asked for this to be a
      mypy error; it CANNOT be. typeshed bounds builtins.min with
      SupportsDunderLT[Any], and that `Any` lets any type carrying a __lt__
      satisfy the bound regardless of what its __lt__ accepts -- so as long as
      min(Mps, Mps) is legal, min(Mps, Factor) is accepted too (reveal_type
      collapses it to the bound). NewType(float) has the same hole. The runtime
      isinstance guard is the compensating control, and the gap is itself an
      asserted test case so it is never mistaken for covered.

*** functools.total_ordering is BANNED here. The methods it synthesises are
typed to accept `object`, which would re-open the arg-type and direct-comparison
holes above. Ordering is written out once, typed to the self type.
"""

from __future__ import annotations

from typing import TypeVar

# Self-type variable: bound to _Scalar so each comparison method below is typed
# to the CONCRETE subclass of `self`, not to the shared base. That is what makes
# `Mps(2) < Factor(3)` a mypy error while `Mps(2) < Mps(3)` is fine.
_S = TypeVar("_S", bound="_Scalar")


class _Scalar:
    """Shared storage plus same-unit ordering. Subclasses add only a name and a
    docstring; the behaviour is here so there is one copy to reason about.

    The ordering methods take `other: _S` where _S is bound to the self type, so
    the operand must be the same concrete unit. Each also guards with isinstance
    and returns NotImplemented for a foreign type, so the restriction holds at
    runtime too (annotations are erased; without the guard, `Factor(0.3) <
    Mps(2.0)` would quietly compute 0.3 < 2.0 and return True).
    """

    # __slots__: these are constructed inside the 20 Hz motion loop; no per-
    # instance dict.
    __slots__ = ("value",)

    def __init__(self, value: float) -> None:
        # Coerce once so a caller may pass an int literal.
        self.value = float(value)

    def __repr__(self) -> str:
        # e.g. Mps(2.0) -- the concrete class name, so a mixed list is readable.
        return "%s(%r)" % (type(self).__name__, self.value)

    def __eq__(self, other: object) -> bool:
        # Equality is total (Python requires __eq__ not to raise for use in
        # dict/set), so a foreign type is simply not-equal rather than an error.
        # Only ORDERING raises -- that is the operation a dimension bug abuses.
        if type(other) is not type(self):
            return NotImplemented
        return self.value == other.value

    def __hash__(self) -> int:
        # Consistent with __eq__: same concrete type + same value -> same hash.
        return hash((type(self).__name__, self.value))

    # Only __lt__ and __gt__ are defined: they are what min() / max() / sorted()
    # and a direct `v < limit` need. __le__ / __ge__ are intentionally absent --
    # no caller uses `<=` on a unit yet, and adding operators before a consumer
    # exists is the speculative-hook pattern CLAUDE.md 9.3 forbids. A caller that
    # needs `v <= x` adds __le__ here, same shape, with its own test.
    def __lt__(self: _S, other: _S) -> bool:
        # Same-unit only; foreign operand -> NotImplemented -> Python raises
        # TypeError (after trying the reflected __gt__, which also refuses).
        if type(other) is not type(self):
            return NotImplemented
        return self.value < other.value

    def __gt__(self: _S, other: _S) -> bool:
        # Mirror of __lt__ for max(); same self-type typing and runtime guard.
        if type(other) is not type(self):
            return NotImplemented
        return self.value > other.value


class Mps(_Scalar):
    """Speed, metres per second. Output of the speed gate and the only unit the
    single velocity exit (p1_motion) is allowed to emit."""

    __slots__ = ()


class Factor(_Scalar):
    """Dimensionless multiplier in [0, 1] (a speed-gate band coefficient, a
    boost factor). Must never reach a min() that is choosing a speed."""

    __slots__ = ()


class Mps2(_Scalar):
    """Acceleration / deceleration, metres per second squared (a brake limit,
    common.spec.max_decel_mps2)."""

    __slots__ = ()


class Seconds(_Scalar):
    """A duration in seconds -- a monotonic-clock delta or a timeout budget, NOT
    a wall-clock timestamp (those stay plain float ts; see CLK-C1)."""

    __slots__ = ()
