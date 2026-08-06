"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_units.py
Brief: CFG-CM-18 -- prove a dimension mix-up is caught at runtime, by mypy, and
       by the C++ compiler, and that each check would go green under the mutant

Description:
The criterion (CFG-CM-18) literally asked for min(Mps(2.0), Factor(0.3)) to be a
mypy error. It cannot be -- typeshed bounds builtins.min with
SupportsDunderLT[Any], so any comparable type slips through, NewType(float)
included (reveal_type confirms it). Rather than fake a green, this file records
that gap explicitly and relocates the static guarantee to where it IS reachable:
cross-unit ARITHMETIC (Mps * Factor) is a mypy error, which NewType(float) does
NOT catch. The min() case is then covered at RUNTIME, where the comparison
genuinely raises.

Each assertion is paired with the mutant that would defeat it, per CLAUDE.md
3.3:
  - mypy:    the control snippet rebuilds Mps/Factor as NewType(float) and shows
             Mps * Factor is then accepted -- so the real error is due to the
             non-float class design, the exact discrimination the item wanted.
  - runtime: comparing Mps to Factor must raise; a shared ordering base (the
             design mistake the module warns about) would make it return a bool.
  - C++:     the must-not-compile snippet, under `using Mps = double`, WOULD
             compile -- so this test compiling-green is a real signal.

*** The min() mypy gap is itself an asserted case
(test_min_across_units_is_a_mypy_blind_spot_and_runtime_is_the_guard), not a
comment, so it cannot be quietly forgotten.
"""

import os
import shutil
import subprocess
import sys
import textwrap

import pytest

from xbrain.common.types import Factor, Mps, Mps2, Seconds

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CXX_HEADER_DIR = os.path.join(ROOT, "common", "include")


# --------------------------------------------------------------------------
# Runtime half -- holds even in a process where mypy never ran.
# --------------------------------------------------------------------------

def test_same_unit_min_returns_the_smaller():
    """min over one unit is normal and keeps the type."""
    result = min(Mps(2.0), Mps(0.3))
    assert result.value == 0.3
    assert isinstance(result, Mps)


def test_min_across_units_raises_at_runtime():
    """*** The runtime guarantee.

    min(Mps, Factor) compares the two, which must raise TypeError. Mutation: if
    the classes shared an ordering base (or dropped the isinstance guard), the
    comparison would execute 0.3 < 2.0 and return a bool -- no raise -- and this
    goes red.
    """
    with pytest.raises(TypeError):
        min(Mps(2.0), Factor(0.3))


@pytest.mark.parametrize("a,b", [
    (Mps(1.0), Factor(1.0)),
    (Mps(1.0), Mps2(1.0)),
    (Seconds(1.0), Mps(1.0)),
    (Factor(1.0), Seconds(1.0)),
])
def test_cross_unit_ordering_raises(a, b):
    """Any ordering across two different units raises, not just the min case."""
    with pytest.raises(TypeError):
        _ = a < b


def test_equality_across_units_is_false_not_error():
    """Equality is total (Python requires it), so it returns False for a foreign
    type rather than raising -- only ORDERING raises. Guards against someone
    'fixing' __eq__ to raise, which would break dict/set use."""
    assert (Mps(1.0) == Factor(1.0)) is False
    assert (Mps(1.0) == Mps(1.0)) is True


# --------------------------------------------------------------------------
# mypy half -- the criterion as written.
# --------------------------------------------------------------------------

def _run_mypy(source):
    """Type-check `source` as a standalone file under --strict, cwd=repo root so
    `import xbrain...` resolves. Returns (returncode, output). Skips if mypy is
    absent rather than passing vacuously."""
    if shutil.which("mypy") is None:
        pytest.skip("mypy not installed; the runtime and C++ halves still run")
    tmp = os.path.join(ROOT, "tests", "common", "_mypy_probe_tmp.py")
    with open(tmp, "w") as f:
        f.write(source)
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "mypy", "--strict", "--no-error-summary",
             "--no-incremental", tmp],
            cwd=ROOT, capture_output=True, text=True)
        return proc.returncode, proc.stdout + proc.stderr
    finally:
        os.remove(tmp)


def test_mypy_rejects_arithmetic_across_units():
    """*** The real static win. Mps * Factor must be a mypy error.

    This is what the non-float class buys that neither plain float nor NewType
    does: no cross-unit __mul__ is defined, so the expression has no operator.
    The paired control below shows NewType(float) lets the same line through.
    """
    rc, out = _run_mypy(textwrap.dedent("""
        from xbrain.common.types import Mps, Factor
        bad = Mps(2.0) * Factor(0.3)
    """))
    assert rc != 0, "mypy accepted Mps * Factor; the arithmetic barrier is gone\n" + out


def test_mypy_control_newtype_would_allow_arithmetic_mix():
    """*** The mutant, made executable.

    Rebuild Mps/Factor as NewType(float) and show Mps * Factor then type-checks,
    because both are floats. This proves the error above is caused specifically
    by the non-float class design -- the exact discrimination CFG-CM-18 wanted,
    relocated from min() (unreachable, see below) to arithmetic (reachable).
    """
    rc, out = _run_mypy(textwrap.dedent("""
        from typing import NewType
        Mps = NewType("Mps", float)
        Factor = NewType("Factor", float)
        legal = Mps(2.0) * Factor(0.3)
    """))
    assert rc == 0, ("the NewType(float) control should type-check Mps*Factor; if "
                     "not, the mypy setup itself is broken\n" + out)


def test_mypy_rejects_direct_comparison_across_units():
    """A direct `Mps < Factor` (not via min) IS a mypy error, because the
    ordering operators are typed to the self unit. This is stronger than the
    min() case and worth pinning separately."""
    rc, out = _run_mypy(textwrap.dedent("""
        from xbrain.common.types import Mps, Factor
        bad = Mps(2.0) < Factor(0.3)
    """))
    assert rc != 0, "mypy accepted a direct Mps < Factor comparison\n" + out


def test_mypy_rejects_factor_where_speed_expected():
    """Passing a Factor into a Mps-typed parameter is a mypy error -- the most
    common real slip (a gain handed to something expecting a speed)."""
    rc, out = _run_mypy(textwrap.dedent("""
        from xbrain.common.types import Mps, Factor
        def gate(v: Mps) -> Mps: return v
        bad = gate(Factor(0.3))
    """))
    assert rc != 0, "mypy accepted a Factor where a Mps was required\n" + out


def test_mypy_accepts_min_within_one_unit():
    """The types must stay usable: min(Mps, Mps) must NOT error, or the barrier
    is useless (everything red is the same as everything green -- 3.2 form 2)."""
    rc, out = _run_mypy(textwrap.dedent("""
        from xbrain.common.types import Mps
        ok = min(Mps(2.0), Mps(0.3))
    """))
    assert rc == 0, "mypy rejected min(Mps, Mps), which must be legal\n" + out


def test_min_across_units_is_a_mypy_blind_spot_and_runtime_is_the_guard():
    """*** The honest record of what mypy CANNOT do here (CLAUDE.md 3.2).

    CFG-CM-18 literally asked for min(Mps, Factor) to be a mypy error. It cannot
    be: typeshed bounds builtins.min with SupportsDunderLT[Any], and that `Any`
    lets any type with a __lt__ satisfy the bound no matter what its __lt__
    accepts -- so while min(Mps, Mps) is legal, min(Mps, Factor) is accepted too.
    NewType(float) has the identical hole. This case asserts BOTH facts so the
    gap is documented and permanent knowledge, not silently assumed covered:
      (1) mypy really does accept it (the limitation), and
      (2) the runtime guard really does catch it (the compensating control,
          which is the actual protection -- see test_min_across_units_raises).
    If a future mypy/typeshed tightens min() so (1) starts failing, that is good
    news and this case should be revisited, not deleted.
    """
    rc, out = _run_mypy(textwrap.dedent("""
        from xbrain.common.types import Mps, Factor
        slips_past_mypy = min(Mps(2.0), Factor(0.3))
    """))
    assert rc == 0, ("mypy now flags min(Mps, Factor) -- the typeshed limitation "
                     "may have been fixed; revisit this case\n" + out)
    # (2) the compensating control: the same expression raises at runtime.
    with pytest.raises(TypeError):
        min(Mps(2.0), Factor(0.3))


# --------------------------------------------------------------------------
# C++ half -- the strong typedef header.
# --------------------------------------------------------------------------

def _compile_cxx(body):
    """Compile a translation unit that includes units.h and runs `body` in
    main(). Returns (returncode, output). Skips if no g++."""
    gxx = shutil.which("g++") or shutil.which("clang++")
    if gxx is None:
        pytest.skip("no C++ compiler; runtime and mypy halves still run")
    src = os.path.join(ROOT, "tests", "common", "_units_probe_tmp.cc")
    with open(src, "w") as f:
        f.write("#include <algorithm>\n")
        f.write("#include \"xbrain/units/units.h\"\n")
        f.write("using namespace xbrain::units;\n")
        f.write("int main() {\n" + body + "\n  return 0;\n}\n")
    try:
        proc = subprocess.run(
            [gxx, "-std=c++17", "-c", src, "-o", os.devnull,
             "-I", CXX_HEADER_DIR],
            capture_output=True, text=True)
        return proc.returncode, proc.stdout + proc.stderr
    finally:
        os.remove(src)


def test_cxx_min_within_one_unit_compiles():
    """std::min(Mps, Mps) must compile -- the types have to be usable."""
    rc, out = _compile_cxx("  Mps r = std::min(Mps{2.0}, Mps{0.3}); (void)r;")
    assert rc == 0, "std::min(Mps, Mps) failed to compile\n" + out


def test_cxx_min_across_units_does_not_compile():
    """*** The C++ guarantee. std::min(Mps, Factor) must fail to compile.

    Mutation: replace the struct with `using Mps = double;` and this snippet
    compiles, so a green here is a real signal, not a vacuous one.
    """
    rc, out = _compile_cxx("  auto r = std::min(Mps{2.0}, Factor{0.3}); (void)r;")
    assert rc != 0, ("std::min(Mps, Factor) compiled; the C++ strong typedef is "
                     "not strong (did units.h become a type alias?)\n" + out)
