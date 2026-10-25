"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: boot_greeting.py
Brief: Boot greeting -- one spoken line after p2 starts, once the speaker answers

Description:
What this adds. The robot says one fixed line when it comes up (user ruling
2026-10-25: "你好,我是海卫风息!"), so a person standing next to it hears that
the voice side is alive. The line and its wait bound live in p2_core.yaml
bit.announce (14 S11: ALL boot-announce configuration is in the bit: segment,
there is no separate file).

Where it sits in the design, and where it deliberately does not.
  * The design's boot speech is the quick-BIT announce (14 S11 announce.policy,
    11 S8.8 BA-1..BA-4): preset_seq built from the BIT result. Quick BIT is
    not wired in p2 today (bit/report.py, lifecycle/sm.py and
    speech/presets.build_announce_sequence have no runtime caller). This
    greeting is NOT that announce and makes no claim about the BIT result.
  * That is why the line carries no readiness guarantee: the user chose
    (2026-10-25) to drop "系统启动完毕" from it, because nothing verifies the
    system is ready before it is spoken (CLAUDE.md 3.2 -- an unverified
    "started OK" is a false guarantee). When quick BIT is wired, the "ready"
    wording belongs in its bit_pass / bit_tail_continue presets.
  * Speaker domain source is bit_announce (14 S4.1, priority 100, the
    lowest): the greeting never competes with an operator's request. If the
    speaker is busy when the greeting is tried, it is skipped, not queued --
    the same rule 14 S11 gives a preempted BIT announce ("不重播").

Why it waits and retries. p2 starts before payload-service in the stack
order (run_stack_cloud.sh stage 6 starts the AI services after p2..p5), so the
first TTS call at p2 startup normally fails with "connection refused". The
greeting retries on that kind of failure (E_UNHEALTHY from handle_speak) every
RETRY_INTERVAL_S until boot_greeting_wait_s has passed, then gives up with a
warning. A busy speaker is not retried (see above).

What this does NOT do: it does not run, wait for, or report BIT; it does not
publish health/bit or any event; it does not touch the mic gate itself
(SpeakerDomain.handle_speak does the half-duplex mute around playback, so the
greeting cannot be heard by the robot's own ASR).

Wrong-looking-but-right: a greeting set to null is "off", not "uncalibrated".
This is a content line, not a common.safety / common.spec value, so CLAUDE.md
3.1's "null refuses startup" rule does not apply; an ABSENT key, however,
still refuses startup, because it means the freeze line was not re-run.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

# The two ack codes the loop decides on, from the closed set (11 S13): the
# same constants SpeakerDomain.handle_speak returns, never string literals
# (CLAUDE.md 3.5), so a renamed code breaks here at import, not silently.
from xbrain.common.errors import E_BUSY, E_UNHEALTHY

# Own logger name so the one outcome line per boot is easy to grep for.
_logger = logging.getLogger("xbrain.p2.boot_greeting")

#: Speaker domain source for the greeting (14 S4.1 register: bit_announce,
#: priority 100). Shared with the future quick-BIT announce on purpose: both
#: are boot speech, both must lose to every operator-facing source.
GREETING_SOURCE = "bit_announce"

#: Module-level (not a parameter) so tests can set it to 0 with monkeypatch.
#: Seconds between attempts while payload-service is not answering. Not a
#: safety value: it only spaces out retries of one spoken line. Two seconds
#: keeps the greeting within a breath of payload-service coming up without
#: spamming its log with refused connections.
RETRY_INTERVAL_S = 2.0

#: Outcome strings, logged once per boot (the field the log reader greps).
SPOKEN = "spoken"            # the line was played
SKIPPED_BUSY = "skipped_busy"  # someone else held the speaker; not replayed
GAVE_UP = "gave_up"          # speaker never answered within the wait bound
STOPPED = "stopped"          # p2 is shutting down before the line was played


class BootGreetingConfigError(ValueError):
    """bit.announce.boot_greeting* is absent or malformed in the snapshot.

    A ValueError subclass: every case is a bad value. Raised at p2 startup,
    from the wiring, so the process refuses to start with the key path in the
    message -- the same shape as the rel_move.* checks next to it.
    """


@dataclass(frozen=True)
class BootGreeting:
    """The resolved greeting: what to say, and how long to wait for the speaker.

    Frozen: it is built once at startup from the snapshot and then read from
    the greeting thread; nothing may change it under that thread. Only a
    validated instance exists -- parse_boot_greeting is the one constructor
    the wiring uses, so the loop never has to re-check its fields.
    """
    # The line itself, stripped; never empty (parse refuses that).
    text: str
    # Seconds from the first try after which an unanswered speaker is given
    # up on; always > 0 (parse refuses 0, negatives and bools).
    wait_s: float


def parse_boot_greeting(get: Callable[[str], object],
                        missing: object) -> Optional[BootGreeting]:
    """Read the two keys from the resolved p2_core snapshot.

    get is the snapshot accessor (resolved.get); missing is its MISSING
    sentinel. Returns None when the greeting is switched off (text null).
    Raises BootGreetingConfigError naming the key path otherwise, at startup
    -- not as a silent "no greeting" that looks like a speaker fault.
    """
    # Both keys are read before either is judged, so one error message can
    # name the first offending path in a fixed order (text, then wait).
    text = get("bit.announce.boot_greeting")
    wait = get("bit.announce.boot_greeting_wait_s")
    # Absent = the source gained the key but the freeze line was not re-run
    # (10 S5.4.1: processes read the product, never the source).
    # Reading an absent key as "off" would make a stale snapshot look like a
    # deliberate choice: the robot would boot silently and nobody would know
    # the edit to configs/p2_core.yaml never reached it. So it refuses, and
    # the message says what to do (re-run the freeze line), which is the
    # same remedy rel_move.* gives for the same condition.
    for path, value in (("bit.announce.boot_greeting", text),
                        ("bit.announce.boot_greeting_wait_s", wait)):
        if value is missing:
            raise BootGreetingConfigError(
                "%s absent from the resolved snapshot (re-run the freeze line)"
                % path)
    # null text is the documented "off" switch (see module docstring).
    if text is None:
        return None
    # A blank line would make the TTS call fail on every try until the wait
    # bound, which reads as a payload-service fault; refuse it at startup
    # instead, where the cause is obvious.
    if not isinstance(text, str) or not text.strip():
        raise BootGreetingConfigError(
            "bit.announce.boot_greeting must be a non-empty string or null, "
            "got %r" % (text,))
    # bool is an int subclass; True would read as one second.
    # Zero or negative would give up before the first retry, so the greeting
    # would never survive the normal startup order (p2 before payload).
    if isinstance(wait, bool) or not isinstance(wait, (int, float)) or wait <= 0:
        raise BootGreetingConfigError(
            "bit.announce.boot_greeting_wait_s must be a number > 0, got %r"
            % (wait,))
    # Stripped so a stray space in the yaml does not reach the TTS engine;
    # wait coerced to float so an integer in yaml and a float compare alike.
    return BootGreeting(text=text.strip(), wait_s=float(wait))


def run_boot_greeting(greeting: BootGreeting,
                      speak: Callable[[str, Optional[str]], dict],
                      stop: threading.Event,
                      now: Callable[[], float] = time.monotonic) -> str:
    """Speak the greeting once, retrying only while the speaker is not up.

    Decision table, one row per ack from the speaker:
        ok                  -> SPOKEN        (done)
        code == E_BUSY      -> SKIPPED_BUSY  (give way, never replay)
        anything else       -> retry after RETRY_INTERVAL_S, until the bound
        bound passed        -> GAVE_UP
        stop set            -> STOPPED       (checked before every attempt)
    Returns one of the outcome constants above; the caller logs it.

    speak is SpeakerDomain.handle_speak (text, source) -> ack dict. stop is
    p2's shutdown event; waiting on it (rather than time.sleep) lets a
    shutdown during the wait end the thread at once. now is the monotonic
    clock (CLAUDE.md 3.4), injectable for tests.
    """
    # The bound is measured from the first try, on the monotonic clock: a
    # wall-clock step (NTP / RTK time sync at boot) must not stretch or cut it.
    deadline = now() + greeting.wait_s
    while True:
        # Checked first on every pass: a shutdown that arrived during the
        # previous wait must not be followed by one more spoken attempt.
        if stop.is_set():
            return STOPPED
        # One attempt. handle_speak blocks for the playback length when it
        # succeeds (it holds the speaker until the device has gone quiet).
        ack = speak(greeting.text, GREETING_SOURCE)
        if ack.get("ok"):
            return SPOKEN
        # Busy = another source is talking. Lowest priority: give way and do
        # not replay (14 S11 "不重播" for a preempted boot announce).
        if ack.get("code") == E_BUSY:
            return SKIPPED_BUSY
        # Anything else (E_UNHEALTHY: payload-service not answering yet) is
        # retried until the wait bound. An unknown code is treated the same
        # way: retrying a spoken line is harmless, giving up early is not.
        # What E_UNHEALTHY looks like in practice at boot: the TTS client gets
        # "connection refused" from payload-service, handle_speak unmutes the
        # mic again and returns the ack. Each retry repeats that cycle, so the
        # mic is muted only for the length of a failed HTTP call each time,
        # never across the whole wait.
        # An unexpected code is logged (once per occurrence) because it means
        # handle_speak grew a new failure the greeting does not know about.
        if ack.get("code") != E_UNHEALTHY:
            _logger.warning("boot greeting: unexpected speaker ack %r", ack)
        # The bound is checked AFTER a failed try, so even a bound shorter
        # than one retry interval gets one attempt.
        if now() >= deadline:
            return GAVE_UP
        # Event.wait returns early on shutdown; the loop head then exits.
        # Not time.sleep: a sleeping thread would delay p2's exit by up to
        # RETRY_INTERVAL_S, and a shutdown is when that matters most.
        stop.wait(RETRY_INTERVAL_S)


def start_boot_greeting(greeting: Optional[BootGreeting],
                        speak: Callable[[str, Optional[str]], dict],
                        stop: threading.Event) -> Optional[threading.Thread]:
    """Run the greeting on a daemon thread; never block p2's startup.

    Returns the thread (tests join it), or None when the greeting is off.
    The outcome is logged once: SPOKEN at INFO, anything else at WARNING so a
    silent boot is explained in the log.
    """
    if greeting is None:
        # Logged so "why did it not greet" has an answer in the log.
        _logger.info("boot greeting: off (bit.announce.boot_greeting is null)")
        return None

    def _run() -> None:
        # Runs on the greeting thread. run_boot_greeting does not raise on a
        # speaker fault (those come back as acks), so the only exceptions here
        # would be programming errors; they surface in the thread's traceback.
        # One line of log per boot, whatever happened.
        outcome = run_boot_greeting(greeting, speak, stop)
        log = _logger.info if outcome == SPOKEN else _logger.warning
        log("boot greeting: %s (wait bound %.0fs)", outcome, greeting.wait_s)

    # Daemon: a greeting still waiting for payload-service must never hold
    # p2's exit; the wiring also sets `stop` in its shutdown path so the
    # thread ends promptly rather than being killed mid-call.
    t = threading.Thread(target=_run, name="p2.boot_greeting", daemon=True)
    t.start()
    return t
