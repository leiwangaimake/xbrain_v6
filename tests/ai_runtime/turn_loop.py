"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: turn_loop.py
Brief: The 功能1 near-field dialogue loop -- Listen, Think, Speak, and back to Listen.

Description:
  This is the orchestrator the other AI_runtime modules exist to serve. It implements the
  turn described in plan section 6: read 20 ms frames from a microphone, let the VAD cut
  them into utterances, transcribe one, generate a reply, speak it through the device, and
  resume listening once the device has stopped talking. Everything it does is a
  composition of the modules around it -- it owns no protocol, no socket and no audio
  processing of its own.

  Which microphone is a configuration choice, not a structural one (open_mic below): the
  payload device's own mic over WS /mic, or one attached to the Orin. Both present the
  same 640-byte frames, so everything below this paragraph is written against the frames
  and is identical either way.

  The one hard problem here is the half-duplex boundary, and it drives the whole design.
  Whatever the robot says is heard back -- by the device's own mic, which shares a housing
  with its speaker, or by a local mic standing in the same room as it. If those frames
  reached the VAD the loop would transcribe its own reply and answer itself, forever. The
  fix is a GATE, and the gate has a specific shape (plan section 6 step 4): while a turn is
  in flight the frames are read and DISCARDED, not left unread and not stopped at the
  source. That distinction is the reason this file is structured around a background task
  instead of a straight-line sequence:

    - Not stopped at the source: closing the WS mic socket would make payload-service
      disarm recording, and re-arming costs about 98 ms of speech at the head of the next
      utterance (plan section 7); restarting a local capture costs an ALSA open. Either
      way the person is cut off mid-first-syllable every turn.
    - Not left unread: if the loop simply stopped reading while it thought and spoke, the
      frames would pile up in the socket or pipe buffers -- and the backlog would be
      exactly the robot's own speech, delivered in a burst the moment the gate lifted.
      That is the echo the gate exists to prevent, merely delayed. Frames must be drained
      at real time and thrown away as they arrive.

  So the turn runs as a separate asyncio Task while the frame loop keeps draining. The
  loop is a plain `async for` that drops every frame while that task is alive, which
  makes "gated" and "a turn is in flight" the same fact rather than two pieces of state
  that could disagree.

  The gate opens EARLIER than the plan's step 4 -- at the moment the VAD closes an
  utterance, rather than just before POST /tts. Gating through the Think phase as well is
  deliberate: the loop can only carry one turn at a time (there is one device and one
  speaker), so audio arriving while the model is generating could not be answered anyway,
  and admitting it would only queue up a second reply for a question the person has
  already stopped waiting on.

  Failure policy: an ASR, LLM or payload fault ends THAT turn and nothing more. Each
  client already narrows its faults to one exception type for exactly this reason -- the
  person can simply repeat themselves, and a bring-up session that died on the first
  timeout would be far less useful than one that logs it and keeps listening. Anything
  outside those three types is a bug rather than an operational fault, so it is allowed to
  propagate and stop the session instead of being retried in a loop forever.
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, AsyncIterator, Optional

from . import asr_client, llm_client, payload_client
from .asr_client import AsrClientError
from .config import AiRuntimeConfig
from .llm_client import LlmClientError
from .local_mic import LocalMicError, LocalMicStream
from .payload_client import MODE_FUNC1, MODE_IDLE, MicStream, PayloadClientError
from .vad import Utterance, VoiceActivityDetector, build_speech_detector

logger = logging.getLogger("ai_runtime.turn")

# Keys read out of payload-service's /status document during the startup precondition
# check. Spelled as constants because they are the published wire contract of another
# process, not local field names.
_STATUS_DEVICE = "device"
_STATUS_AUDIO_CONNECTED = "audio_connected"

# The two accepted values of config.mic_source. Named constants rather than bare strings
# for the same reason as vad's BACKEND_*: the value is compared here and documented in
# config, and a typo in either place should not be able to change behaviour silently.
MIC_SOURCE_LOCAL = "local"
MIC_SOURCE_DEVICE = "device"


def open_mic(config: AiRuntimeConfig) -> Any:
    """Build the mic stream named by config.mic_source.

    Args:
        config: supplies mic_source and whichever source's settings it names.

    Returns:
        An unopened async context manager that iterates 640-byte 20 ms frames -- either a
        LocalMicStream (a microphone on the Orin) or a MicStream (the payload device's own
        mic over WS /mic). The two are interchangeable from here on.

    Raises:
        PayloadClientError: if mic_source is not a known name.

    An unknown name is refused rather than defaulted, matching build_speech_detector: a
    silent fallback to the other source would produce a session that runs normally while
    listening in the wrong place, and nothing downstream could detect that.
    """
    if config.mic_source == MIC_SOURCE_LOCAL:
        return LocalMicStream(config)
    if config.mic_source == MIC_SOURCE_DEVICE:
        return MicStream(config)
    raise PayloadClientError(
        f"unknown mic_source {config.mic_source!r}, "
        f"expected {MIC_SOURCE_LOCAL!r} or {MIC_SOURCE_DEVICE!r}"
    )


class TurnLoop:
    """Runs 功能1 dialogue turns until the mic stream ends.

    One instance owns one session: the mode it put payload-service into, the mic socket,
    the VAD state and the in-flight turn. Nothing is shared between instances, so a
    session that fails leaves no state behind for the next one to inherit.
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Build a loop; no device or service is touched until run() is awaited.

        Raises:
            VadError: if the configured VAD backend cannot be loaded.

        The speech detector is built HERE, before run() is awaited, so a missing silero
        model is reported at startup beside the configuration that named it -- rather than
        on the first frame, minutes into a bring-up run, as a microphone that hears nobody.
        """
        self._config = config
        self._vad = VoiceActivityDetector(config, build_speech_detector(config))
        # The in-flight turn, or None while listening. This single field IS the gate:
        # not-None means frames are dropped. Keeping the gate and the task as one value
        # makes it impossible for them to disagree, which two booleans would allow.
        self._turn: Optional[asyncio.Task] = None
        # Counts completed turns for the session summary line; a bring-up run is judged
        # partly on how many turns it survived.
        self._turns = 0

    async def run(self) -> None:
        """Enter func1, stream the mic, and serve turns until the stream ends.

        Returns:
            None, when payload-service closes the mic stream cleanly.

        Raises:
            PayloadClientError: if the mode switch, the precondition check or the mic
                stream itself fails -- these end the session, unlike a per-turn fault.
            LocalMicError: if the local mic source is selected and capture fails.

        The mode is switched even when the frames come from a local microphone. It is not
        there to arm the device mic -- payload-service ties recording to the WS /mic socket,
        which a local session never opens -- but to hold invariant R3, so nothing else can
        drive the device into another mode while a dialogue is in progress.

        The mode is restored on the way out ONLY if this process was the one that changed
        it. A run that found the service already in func1 was sharing a session someone
        else set up, and resetting to idle on exit would tear down that other operator's
        state as a side effect of this process stopping.
        """
        await self._check_preconditions()
        switched = await asyncio.to_thread(payload_client.ensure_mode, self._config, MODE_FUNC1)
        try:
            async with open_mic(self._config) as mic:
                await self._serve(mic)
        finally:
            # The in-flight turn holds the device's speaker and must not outlive the
            # session; cancelling before the mode reset keeps the two in order.
            await self._cancel_turn()
            if switched:
                await asyncio.to_thread(payload_client.ensure_mode, self._config, MODE_IDLE)
            logger.info("session ended after %d turns", self._turns)

    async def _check_preconditions(self) -> None:
        """Fail early and clearly if the device audio link is down.

        Raises:
            PayloadClientError: if payload-service reports the audio socket disconnected.

        Checked regardless of the mic source, because the link carries the reply as well as
        the audio: a local-mic session still speaks through the device over [31], so a dead
        audio socket means a dialogue that hears everything and can answer nothing.
        Without this the same fault appears much later and much less legibly, as a WS
        /mic that opens and is immediately closed with code 1011. The cable being unplugged
        is a routine bring-up condition, so it is worth one status call to name it.
        """
        status = await asyncio.to_thread(payload_client.get_status, self._config)
        device = status.get(_STATUS_DEVICE) or {}
        if not device.get(_STATUS_AUDIO_CONNECTED):
            raise PayloadClientError("payload reports the device audio link is down")
        logger.info("payload ready: mode=%s device=%s", status.get("mode"), device)

    async def _serve(self, mic: AsyncIterator[bytes]) -> None:
        """Drain mic frames forever: feed the VAD while listening, discard while gated.

        Args:
            mic: an open mic stream yielding 640-byte 20 ms frames. Typed by what this
                method actually needs -- iteration -- so either source satisfies it.

        Returns:
            None, when the stream ends.

        Raises:
            Exception: whatever an in-flight turn failed with, if it was not one of the
                three operational client faults that turn handling absorbs. Such a fault
                is a bug, and stopping is more useful than looping on it.

        This coroutine must never block on anything slow. Every expensive step of a turn
        happens in the task created below, so the `async for` here keeps consuming frames
        at real time and the socket never backs up (module docstring).
        """
        async for frame in mic:
            if self._turn is not None:
                if not self._turn.done():
                    # Gated: a turn is in flight. The frame is read (so the stream stays
                    # at real time) and thrown away (so the robot never hears itself).
                    continue
                # The turn has finished. result() re-raises an unexpected fault; the three
                # operational ones were already handled inside _run_turn.
                self._turn.result()
                self._turn = None
                # Start the next listening window from silence. The VAD's preroll ring
                # still holds frames captured while the robot was speaking, and carrying
                # them into the next utterance would prepend the tail of the device's own
                # voice to what the person says next.
                self._vad.reset()
                # This frame arrived during the gate, so it is dropped with the rest.
                continue
            utterance = self._vad.push(frame)
            if utterance is None:
                continue
            # An utterance closed: gate immediately and hand the turn to a task, so this
            # loop returns to draining frames on the very next iteration.
            self._turn = asyncio.create_task(self._run_turn(utterance))

    async def _run_turn(self, utterance: Utterance) -> None:
        """Transcribe, answer and speak one utterance; absorb operational faults.

        Args:
            utterance: the audio the VAD segmented.

        Returns:
            None. Returning is what lifts the gate, whether the turn produced speech, was
            abandoned as silence, or failed.

        Every blocking client call is dispatched with asyncio.to_thread. All three are
        synchronous by design (they use requests), and running them inline would stall the
        event loop -- which here means stalling the frame drain, the one thing that must
        keep running.

        The three client errors are caught and logged rather than propagated: each client
        narrows its faults to a single type precisely so this handler can treat them
        alike, and the recovery for all of them is the same -- drop this turn, keep the
        session, let the person repeat themselves.
        """
        started = time.perf_counter()
        try:
            heard = await asyncio.to_thread(
                asr_client.transcribe, self._config, utterance.pcm
            )
            if not heard.strip():
                # A VAD false trigger -- a door, a cough, a burst of room noise. This is a
                # normal event, not a failure, so it is logged at debug and the turn ends
                # without the robot saying anything. Speaking a "sorry, I did not catch
                # that" here would make the device respond to every passing noise.
                logger.debug("empty transcript after %.1fs of audio", utterance.duration_s)
                return
            asr_ms = (time.perf_counter() - started) * 1000.0
            logger.info("heard (%.1fs): %s", utterance.duration_s, heard)

            reply = await asyncio.to_thread(llm_client.complete, self._config, heard)
            llm_ms = (time.perf_counter() - started) * 1000.0 - asr_ms
            logger.info("reply: %s", reply)

            est_ms = await asyncio.to_thread(payload_client.speak, self._config, reply)
        except (AsrClientError, LlmClientError, PayloadClientError) as exc:
            logger.warning("turn dropped: %s", exc)
            return

        self._turns += 1
        # The whole reason the loop knows when to listen again. The device sends no
        # "TTS finished" event (plan section 9), so the end of speech can only be
        # ESTIMATED: payload-service's est_ms for the text, plus a configured margin that
        # absorbs the error in that estimate and the device's own start-up delay. The
        # margin is the number the on-device timing calibration exists to establish.
        gate_ms = est_ms + self._config.tts_gate_margin_ms
        logger.info(
            "turn %d: asr_ms=%.0f llm_ms=%.0f est_ms=%d gate_ms=%d think_ms=%.0f",
            self._turns,
            asr_ms,
            llm_ms,
            est_ms,
            gate_ms,
            (time.perf_counter() - started) * 1000.0,
        )
        # Sleeping here, inside the turn task, is what holds the gate shut for the
        # duration of the device's speech -- the frame loop keeps draining and discarding
        # throughout, and only sees the turn as finished once this returns.
        await asyncio.sleep(gate_ms / 1000.0)

    async def _cancel_turn(self) -> None:
        """Cancel any in-flight turn and wait for it, so none outlives the session.

        A turn that is still sleeping out a TTS gate when the session ends has nothing
        left to do, and one still waiting on a client call cannot deliver its reply to a
        closing session either. Awaiting the cancellation (rather than firing and
        forgetting) is what keeps asyncio from reporting a destroyed pending task, and
        guarantees the worker threads are no longer being awaited when run() returns.

        A turn that has ALREADY finished is dropped without awaiting it: the frame loop
        raises out through this cleanup precisely when a turn failed unexpectedly, and
        awaiting the same failed task again here would raise a second copy of that
        exception during teardown and bury the original.
        """
        if self._turn is None or self._turn.done():
            self._turn = None
            return
        self._turn.cancel()
        try:
            await self._turn
        except asyncio.CancelledError:
            pass
        self._turn = None
