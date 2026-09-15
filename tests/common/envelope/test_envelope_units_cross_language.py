"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_envelope_units_cross_language.py
Brief: The C++ envelope must decode with the Python decoder, seconds and all

Description:
This exists because of a defect that lived for two days without a single test
turning red. 11 S3.0 states `ts` as a float64 Unix SECOND count and `mono` as a
CLOCK_MONOTONIC reading "单位秒"; the Qt-facing spec
(docs/MISSON/任务枚举_qt端v2.0.md S1 item 5) freezes `ts` the same way and
forbids millisecond integers by name. Python was corrected on 2026-09-13. The
C++ EnvelopeWriter went on stamping int64 MILLISECONDS, and rtk_driver went on
publishing them.

Nothing caught it, and the reason is worth stating: the C++ test asserted only
ts_sync semantics, so the UNIT was never an assertion at all -- it was an
assumption that happened to be wrong. Changing the header from milliseconds to
seconds did not turn a single existing case red, in either language.

So this test does not check the unit by reading the source or by trusting a
comment. It compiles the real header, stamps an envelope with values chosen so
that a millisecond implementation cannot produce the same digits, prints the
JSON the shared serialiser emits, and hands that text to the PYTHON decoder. It
passes only when:

  * the text is valid JSON that xbrain.common.envelope.decode accepts;
  * ts and mono come back as the second values that went in, not scaled;
  * ts carries sub-second precision, which a millisecond integer cannot;
  * mono is not an integer multiple of the input, which is what a stray *1000
    or /1000 would leave behind.

A missing compiler SKIPS LOUDLY rather than passing: a silent skip on a
cross-language test turns its whole claim into an empty green tick.
"""

import json
import os
import shutil
import subprocess

import pytest

pytestmark = pytest.mark.no_device

#: Four levels up: this file sits at tests/common/envelope/, so three dirnames
#: land on tests/ and not on the repository root.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
INCLUDE = os.path.join(ROOT, "common", "include")
CXX = shutil.which("g++") or shutil.which("clang++")

#: Values picked so that a millisecond implementation produces visibly different
#: digits. The wall time carries six decimals, which an integer millisecond
#: field cannot represent; the monotonic reading is deliberately small, because
#: a *1000 there turns 12.5 into 12500 and a naive "looks like a number" check
#: would accept both.
WALL_S = 1789455340.123456
MONO_S = 12.5
SYNC_TIMEOUT_S = 5.0

PROBE = r"""
#include "xbrain/envelope/envelope_writer.h"

#include <cstdio>

int main() {
  using hachist::xbrain::envelope::EnvelopeWriter;
  using hachist::xbrain::envelope::StampedEnvelope;
  using hachist::xbrain::envelope::WriteEnvelopeJson;

  EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", %(sync)r);
  w.note_clock_status(true, %(mono)r);
  const StampedEnvelope e = w.stamp(%(wall)r, %(mono)r);

  char buf[1024];
  const std::size_t n = WriteEnvelopeJson(e, "{\"probe\":1}", buf, sizeof(buf));
  if (n == 0) {
    std::printf("SERIALISE_FAILED\n");
    return 1;
  }
  std::printf("%%s\n", buf);
  return 0;
}
""" % {"sync": SYNC_TIMEOUT_S, "mono": MONO_S, "wall": WALL_S}


def _run_probe(tmp_path):
    src = os.path.join(str(tmp_path), "envelope_units_probe.cc")
    with open(src, "w", encoding="utf-8") as f:
        f.write(PROBE)
    exe = os.path.join(str(tmp_path), "envelope_units_probe")
    # -Werror is part of the assertion: 13 S5.6 requires it package-wide, so a
    # header that warns here would fail the build of every consumer.
    build = subprocess.run(
        [CXX, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
         "-I", INCLUDE, "-o", exe, src],
        capture_output=True, text=True, timeout=180)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([exe], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    return run.stdout.strip()


@pytest.mark.skipif(CXX is None, reason="no C++ compiler: the cross-language "
                                        "envelope unit check did NOT run")
def test_cxx_envelope_decodes_in_python_with_second_units(tmp_path):
    from xbrain.common.envelope.envelope import decode

    text = _run_probe(tmp_path)
    raw = json.loads(text)          # must be valid JSON before anything else
    env = decode(raw)               # ...and must satisfy the Python schema

    # The identity fields, so a probe that silently wrote a different envelope
    # cannot pass the numeric checks by accident.
    assert env.rid == "gj-001"
    assert env.src == "quadruped"
    assert env.boot == "9f2c1a44"
    assert env.seq == 1
    assert env.ts_sync is True

    # *** The unit. A millisecond implementation would return 1789455340123.456
    # here, which is off by three orders of magnitude and is exactly what was
    # on the wire before 2026-09-15.
    assert env.ts == pytest.approx(WALL_S, abs=1e-6)
    assert env.mono == pytest.approx(MONO_S, abs=1e-9)

    # Sub-second precision survives. An integer field cannot carry it, so this
    # fails on any implementation that rounds to whole milliseconds or seconds.
    assert env.ts != int(env.ts)
    assert abs(env.ts - int(env.ts) - 0.123456) < 1e-6

    # And the raw text is a JSON number, not a string. A serialiser that quoted
    # the value would still decode through float() in some readers and would
    # violate the frozen "JSON number" wording of the Qt spec.
    assert isinstance(raw["ts"], float)
    assert isinstance(raw["mono"], float)


@pytest.mark.skipif(CXX is None, reason="no C++ compiler: the cross-language "
                                        "envelope unit check did NOT run")
def test_cxx_envelope_field_set_matches_the_contract(tmp_path):
    """The eight outer fields, no more and no fewer (11 S3.0).

    A missing field is caught by the Python decoder; an EXTRA one is not, and an
    extra outer field is how a publisher starts carrying private state in a
    shared envelope.
    """
    text = _run_probe(tmp_path)
    raw = json.loads(text)
    assert set(raw) == {"v", "rid", "ts", "mono", "boot", "seq", "src",
                        "ts_sync", "data"}
    assert raw["v"] == 1
    assert raw["data"] == {"probe": 1}


@pytest.mark.skipif(CXX is None, reason="no C++ compiler: the truncation check "
                                        "did NOT run")
def test_a_buffer_that_is_too_small_yields_nothing(tmp_path):
    """Half an envelope is valid-looking JSON that decodes to the wrong thing.

    The serialiser reports 0 rather than a partial object, and this compiles a
    probe with a buffer deliberately one byte short of the result.
    """
    src = os.path.join(str(tmp_path), "trunc_probe.cc")
    with open(src, "w", encoding="utf-8") as f:
        f.write(r"""
#include "xbrain/envelope/envelope_writer.h"
#include <cstdio>
int main() {
  using namespace hachist::xbrain::envelope;
  EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", 5.0);
  const StampedEnvelope e = w.stamp(1789455340.123456, 12.5);
  char big[1024];
  const std::size_t full = WriteEnvelopeJson(e, "{\"probe\":1}", big, sizeof(big));
  if (full == 0) { std::printf("UNEXPECTED_ZERO\n"); return 1; }
  // One byte short of what it needs, counting the terminator.
  char small[1024];
  const std::size_t n = WriteEnvelopeJson(e, "{\"probe\":1}", small, full);
  std::printf("%zu\n", n);
  return 0;
}
""")
    exe = os.path.join(str(tmp_path), "trunc_probe")
    build = subprocess.run(
        [CXX, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
         "-I", INCLUDE, "-o", exe, src],
        capture_output=True, text=True, timeout=180)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([exe], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip() == "0", (
        "a buffer one byte short returned %r instead of 0, so a truncated "
        "envelope can reach the wire" % run.stdout.strip())
