"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_boot_greeting.py
Brief: p2 boot greeting -- one line after startup, lowest priority, retried only while the speaker is down

Description:
The feature (user ruling 2026-10-25): p2 speaks "你好,我是海卫风息!" once after
it starts. Design placement and limits are in xbrain/p2_core/speech/
boot_greeting.py: config lives in p2_core.yaml bit.announce (14 S11), the
speaker source is bit_announce (14 S4.1, priority 100), a busy speaker skips
the line and never replays it (14 S11 "不重播"), and the text makes no
readiness claim because nothing verifies readiness first (CLAUDE.md 3.2).

What these cases pin:
  * config parsing: null = off; an ABSENT key refuses startup (freeze line
    not re-run); an empty text or a non-positive / boolean wait is refused,
    naming the key path;
  * the shipped config holds the ruled text, ASCII punctuation, the party-A
    full name, and NO "启动完毕" (the readiness claim the user removed);
  * the loop: spoken on the first good ack; retried while the speaker is
    unhealthy (payload-service not up yet -- p2 starts before it); given up
    at the wait bound; a busy speaker is skipped WITHOUT a retry; a shutdown
    ends the wait at once; the source passed is bit_announce every time.

Why each rule needs its own case. The loop has five exits and each one is the
easy thing to get wrong in a different direction:
  * busy -> skip: the natural "just retry until it works" implementation
    would talk over an operator-facing source the moment it finished, which
    is exactly what priority 100 is supposed to prevent;
  * unhealthy -> retry: without it the greeting would never be heard on a
    normal boot, because payload-service comes up after p2;
  * deadline -> give up: without it a robot whose payload box is unplugged
    would retry forever (harmless for the speaker, but a thread and a log
    line every two seconds for the life of the process);
  * stop -> stopped: without it p2's exit could be followed by one more TTS
    call into a speaker that is being torn down;
  * null -> off vs absent -> refuse: the two look alike from the outside
    (no greeting) and mean opposite things (deliberate vs stale snapshot).

How the time is controlled. The loop reads a monotonic clock and waits on
the stop event between tries. Here the clock is a counter that advances one
second per read and the retry spacing is set to zero, so every case runs in
microseconds and the number of tries is a deterministic function of the
scripted acks -- the call count is then a reliable assertion.

Mutants this file kills (run by hand, see the batch commit):
  * busy retried instead of skipped            -> busy case (call count)
  * no retry on unhealthy                      -> retry case
  * deadline ignored (retries forever)         -> give-up case (bounded loop)
  * source not bit_announce                    -> source assertion
  * null text treated as an error              -> off case
  * absent key read as off                     -> absent-key case

Boundaries: the speaker itself is a stub; SpeakerDomain.handle_speak's
half-duplex mute is tested in tests/p2_core (speaker wiring). That the line
is actually heard on the robot is checked live on .1.26, not here.
"""
from __future__ import annotations

import threading

import pytest
import yaml

# The module is imported as a module (not names from it) so the fixture below
# can monkeypatch RETRY_INTERVAL_S where the loop reads it.
import xbrain.p2_core.speech.boot_greeting as bg
# The same closed-set constants handle_speak returns: the stub speaks the
# real codes, so a renamed code breaks these cases instead of passing them.
from xbrain.common.errors import E_BUSY, E_UNHEALTHY

# Pure unit cases: no zenoh, no payload-service, no speaker hardware.
# Nothing here opens a socket or plays a sound; the speaker is _Speaker below.
pytestmark = pytest.mark.no_device

#: Stand-in for resolved.MISSING: any unique object works, the parser only
#: compares identity. Passed explicitly to parse_boot_greeting, exactly as the
#: wiring passes the real sentinel, so the parser never imports the resolved
#: module and these cases need no snapshot on disk.
_MISSING = object()


def _getter(values):
    """A snapshot accessor over a flat {dotted.path: value} dict.

    Same contract as common.config.resolved's get: the value for a present
    key (which may be None), and the MISSING sentinel for an absent one.
    """
    return lambda path: values.get(path, _MISSING)


class _Speaker:
    """A handle_speak stub that replays a scripted list of acks.

    Records (text, source) of every call: the call count is how the busy and
    retry rules are told apart, and the source is how the priority rule is.
    """

    def __init__(self, acks):
        # Scripted acks, consumed in order; the last one repeats.
        self.acks = list(acks)
        # Every (text, source) the greeting tried to speak.
        self.calls = []

    def __call__(self, text, source):
        # Same signature as SpeakerDomain.handle_speak(text, source).
        self.calls.append((text, source))
        # Pop until one is left, then keep answering with it (a speaker that
        # stays down, or stays up).
        return self.acks.pop(0) if len(self.acks) > 1 else self.acks[0]


class _Clock:
    """A monotonic clock that advances one second per read.

    The loop reads it once to set the deadline and once after each failed
    try, so "seconds" here are really "reads"; the cases only rely on the
    deadline being reached after a bounded number of tries.
    """

    def __init__(self):
        # Starts at zero; only differences matter to the loop.
        self.t = 0.0

    def __call__(self):
        # Advance, then report: strictly increasing, like time.monotonic().
        self.t += 1.0
        return self.t


@pytest.fixture(autouse=True)
def _no_real_wait(monkeypatch):
    """Retry spacing 0 s: the loop logic is under test, not the 2 s sleep.

    stop.wait(0) returns at once, so a case with several retries still runs
    in microseconds; the deadline is driven by the fake clock instead.
    """
    monkeypatch.setattr(bg, "RETRY_INTERVAL_S", 0.0)


# --- config ----------------------------------------------------------------
# The parser is the startup gate: everything it refuses, p2 refuses to boot
# with, naming the key path.

def test_null_text_switches_the_greeting_off():
    """null is the documented off switch, not a refusal (content, not safety).

    CLAUDE.md 3.1's "null refuses startup" governs common.safety / spec
    values; a spoken line is content, and a site that wants a silent boot
    (night shift, residential neighbour) sets it to null.
    """
    got = bg.parse_boot_greeting(_getter({
        "bit.announce.boot_greeting": None,
        "bit.announce.boot_greeting_wait_s": 120}), _MISSING)
    # Off is "no greeting object", which start_boot_greeting turns into
    # "no thread" (see the last case of this file).
    assert got is None


@pytest.mark.parametrize("values, key", [
    # The text key absent: the freeze line was not re-run after the edit.
    ({"bit.announce.boot_greeting_wait_s": 120}, "bit.announce.boot_greeting"),
    # The wait key absent.
    ({"bit.announce.boot_greeting": "你好"}, "boot_greeting_wait_s"),
])
def test_an_absent_key_refuses_startup(values, key):
    """Absent is not off: reading it as off would hide a stale snapshot.

    The message must name the key, because the operator's fix is to re-run
    the freeze line for exactly that file; "config error" alone would send
    them looking in the wrong place.
    """
    # The other key is present with a valid value in each row, so the
    # refusal can only come from the missing one.
    # Matched on the path AND the word "absent": the malformed-value error
    # names the same path, so the path alone could not tell "absent" from
    # "present but wrong" -- and that difference is the point of this case.
    with pytest.raises(bg.BootGreetingConfigError,
                       match=key.strip() + " absent"):
        bg.parse_boot_greeting(_getter(values), _MISSING)


@pytest.mark.parametrize("text, wait", [
    ("", 120),          # empty line
    ("   ", 120),       # blank line
    (3, 120),           # not text at all
    ("你好", 0),         # zero wait: would give up before the first retry
    ("你好", -5),        # negative wait
    ("你好", True),      # bool is an int subclass; True would read as 1 s
])
def test_a_malformed_value_is_refused(text, wait):
    """Each malformed value refuses startup instead of a silent no-greeting.

    Each row is a value that, accepted, would produce a robot that boots in
    silence with a log line blaming the speaker: a blank line fails every
    TTS call; a zero or negative wait gives up before payload-service is up;
    True is a typo that Python would happily treat as one second.
    """
    # Both keys present (so the absent-key branch is not what fires); only
    # the value under test is wrong.
    with pytest.raises(bg.BootGreetingConfigError):
        bg.parse_boot_greeting(_getter({
            "bit.announce.boot_greeting": text,
            "bit.announce.boot_greeting_wait_s": wait}), _MISSING)


def test_the_shipped_config_holds_the_ruled_line():
    """configs/p2_core.yaml carries the line as ruled on 2026-10-25.

    The full name 海卫风息 (18 J02 party-A identity), ASCII punctuation in the
    data value (CLAUDE.md 2.2), and no "启动完毕": the user removed the
    readiness claim because nothing verifies readiness before it is spoken.
    The SOURCE file is read (not the resolved snapshot), so this does not
    depend on whichever freeze happens to be on disk.
    """
    # The absolute path matches the rest of tests/p2_core (the suite runs on
    # the robot from /opt/xbrain_v6; there is no relocatable checkout).
    cfg = yaml.safe_load(open("/opt/xbrain_v6/configs/p2_core.yaml",
                              encoding="utf-8"))
    ann = cfg["bit"]["announce"]
    # Exactly the ruled line, punctuation included.
    assert ann["boot_greeting"] == "你好,我是海卫风息!"
    # The readiness claim stays out until quick BIT can back it.
    assert "启动完毕" not in ann["boot_greeting"]
    # A positive wait bound, parsed the way p2 parses it.
    got = bg.parse_boot_greeting(_getter({
        "bit.announce.boot_greeting": ann["boot_greeting"],
        "bit.announce.boot_greeting_wait_s": ann["boot_greeting_wait_s"]}),
        _MISSING)
    assert got is not None and got.wait_s > 0


# --- the loop ----------------------------------------------------------------
# One greeting object for every loop case: 10 "seconds" of fake clock is a
# handful of tries, enough to see a retry and to reach the bound.
# Every case passes a fresh threading.Event and a fresh _Clock, so no case
# can leak a set stop flag or an advanced clock into the next one.
# The outcome constants (SPOKEN, SKIPPED_BUSY, GAVE_UP, STOPPED) are what p2
# logs once per boot; asserting on them pins the log line an operator reads.

_G = bg.BootGreeting(text="你好,我是海卫风息!", wait_s=10.0)


def test_spoken_on_the_first_good_ack_with_the_lowest_priority_source():
    """One call, the right text, source bit_announce (14 S4.1 priority 100).

    The source is what puts the greeting at the bottom of the speaker
    domain: with any higher source (tts_local 400, say) it would win over a
    D-mode warning that happened to start in the same second.
    """
    # A speaker that answers at once.
    spk = _Speaker([{"ok": True}])
    out = bg.run_boot_greeting(_G, spk, threading.Event(), now=_Clock())
    # Played on the first try.
    assert out == bg.SPOKEN
    # Exactly one call, with the configured text and the lowest source.
    assert spk.calls == [("你好,我是海卫风息!", "bit_announce")]


def test_an_unhealthy_speaker_is_retried_until_it_answers():
    """p2 starts before payload-service: the first tries fail, a later one plays.

    This is the normal boot, not an edge case: run_stack_cloud.sh starts the
    AI services (payload included) after p2..p5, so the first TTS call meets
    a refused connection.
    """
    # Three refusals (payload-service still starting), then it answers.
    spk = _Speaker([{"ok": False, "code": E_UNHEALTHY}] * 3 + [{"ok": True}])
    out = bg.run_boot_greeting(_G, spk, threading.Event(), now=_Clock())
    # Within the 10 s bound, so the fourth try plays.
    assert out == bg.SPOKEN
    # Three refused tries, then the one that played.
    assert len(spk.calls) == 4
    # Every try carries the lowest-priority source, not just the first.
    assert {s for _, s in spk.calls} == {"bit_announce"}


def test_the_greeting_gives_up_at_the_wait_bound():
    """A speaker that never answers ends in GAVE_UP, not an endless loop.

    The fake clock advances one second per read, so a 10 s bound allows only
    a handful of tries; a loop that ignored the deadline would not return
    (the test would hang and be killed), which is the failure this pins.
    """
    # One scripted ack that repeats forever: payload-service never comes up.
    spk = _Speaker([{"ok": False, "code": E_UNHEALTHY}])
    out = bg.run_boot_greeting(_G, spk, threading.Event(), now=_Clock())
    # The loop came back at all (a deadline-blind loop would hang here), and
    # with the give-up outcome rather than a false SPOKEN.
    assert out == bg.GAVE_UP
    # Bounded: well under the bound in seconds, since the clock moves per read.
    assert 1 <= len(spk.calls) <= 10


def test_a_busy_speaker_is_skipped_not_retried():
    """Someone else is talking: give way, do not replay (14 S11 "不重播").

    The second scripted ack is a success on purpose: an implementation that
    retried on busy would reach it and report SPOKEN, so the outcome AND the
    call count both catch that mutant.
    """
    # Busy first (an operator's TTS reply is playing), free afterwards.
    spk = _Speaker([{"ok": False, "code": E_BUSY}, {"ok": True}])
    out = bg.run_boot_greeting(_G, spk, threading.Event(), now=_Clock())
    # Gave way: the greeting is not worth interrupting anyone for.
    assert out == bg.SKIPPED_BUSY
    # Exactly one try: a retry would talk over the operator-facing source.
    assert len(spk.calls) == 1


def test_a_shutdown_ends_the_greeting_without_speaking():
    """p2 shutting down before the line played: STOPPED, nothing spoken.

    The stop event is set before the first pass, the strongest form: the
    loop must check it before the first attempt, not only between retries.
    """
    # Shutdown already requested when the loop starts.
    stop = threading.Event()
    stop.set()
    # A speaker that WOULD play: the only thing stopping it is the event.
    spk = _Speaker([{"ok": True}])
    out = bg.run_boot_greeting(_G, spk, stop, now=_Clock())
    assert out == bg.STOPPED
    # Not even one attempt reached the speaker.
    assert spk.calls == []


def test_start_returns_no_thread_when_off():
    """Off means no thread at all, not a thread that does nothing.

    A do-nothing thread would still log an outcome line ("gave_up" or the
    like) on every boot of a site that switched the greeting off, which is
    noise in exactly the log an operator reads when something is wrong.
    """
    # None is what parse_boot_greeting returns for a null text.
    assert bg.start_boot_greeting(None, _Speaker([{"ok": True}]),
                                  threading.Event()) is None
