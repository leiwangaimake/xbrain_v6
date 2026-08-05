"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: deter.py
Brief: 功能3 驱离 (deter) loop -- red/blue flash + siren + male-voice warning, self-running.

Description:
  Runs deter mode (development plan sections 1 and 5.1): the payload-service-internal
  loop that drives the device's red/blue warning light, the synthesised siren, and a
  looping male-voice spoken warning to drive an intruder away. It is started and stopped
  by the SessionManager (core/session.py) while the service is in deter mode, and it is
  the only thing that speaks or plays audio in that mode -- there is no /mic, /play or
  external /tts in deter (invariant R3, one mode owns the device at a time).

  From scratch (test-system rule R0): this loop is written from the specification. It
  reuses the project's OWN from-scratch building blocks -- core.siren for the waveform,
  codec.opus_stream for Opus framing, protocol.audio_8519 / protocol.lights_8529 for the
  wire frames -- but imports nothing from tools/ (patrol_deter, siren_gen) and copies no
  code from them. Composing our own modules is exactly what R0 intends.

  Why a self-contained loop inside payload-service (rather than AI_runtime driving each
  siren/TTS/flash tick over the API): deter is a fixed, autonomous behaviour with no
  perception input -- it just needs to keep flashing and warning until told to stop. Its
  cadence is tight (siren playback, then spoken warnings, repeat) and would be fragile if
  every tick had to round-trip the loopback API. Keeping the loop next to the socket owner
  makes it robust to a momentary AI_runtime stall and keeps the whole deterrent as one
  cancellable unit the session can stop atomically.

  Lights are set ONCE at start, not re-sent every cycle. The 8529 red/blue and strobe
  patterns run autonomously in the device firmware once selected (protocol doc: control
  frames are fire-and-forget, the firmware animates the pattern), so re-sending them each
  loop would be pure noise on the wire. The loop body therefore only drives the AUDIO
  deterrent (siren then warning), which the device does NOT animate on its own.

  Audio deterrent path mirrors the /play hail path: the siren is 8 kHz mono PCM straight
  from core.siren (already the device's hail rate, so NO resample), Opus-encoded to
  480-sample frames and sent as [42] hails; the spoken warning goes out as a [31] TTS
  frame in the male voice (section 1 specifies 男声). Because the device emits no
  "TTS finished" event, each warning is paced by config.estimate_tts_ms plus a margin --
  the same estimate POST /tts returns -- since there is nothing to wait on but the clock.

  Resilience to an audio-link hiccup: if a siren/TTS send fails mid-loop, the loop does
  NOT end. The lights (a separate 8529 socket, set once at start) keep flashing from
  firmware and are the primary VISUAL deterrent, so the loop logs the audio fault, paces,
  and retries the audio deterrent next cycle. Only an explicit stop() ends the loop.

  Teardown is the contract that matters most (development plan section 13 acceptance:
  灯全灭收尾 -- lights fully off, red/blue reset, siren/TTS stopped). stop() cancels the
  loop task and then, unconditionally, turns off every device aspect that EMITS: strobe
  off, searchlight off, red/blue off (mode 0), and a hail-stop [11] so no siren tail is
  left playing.

  ★ Brightness is deliberately NOT reset, and it is the one register deter leaves changed:
  after a deter run the lamp is off but its stored level is still 30. Zeroing it would be
  worse, not better -- brightness 0 means the lamp draws no current, so a later caller that
  sends only searchlight-on would get a lamp that lights up dark, which is a stranger
  failure than one that lights up bright. The consequence is real and worth knowing: the
  next bare searchlight-on after a deter run comes up at full glare. A caller that needs a
  specific level should send it, which POST /lights lets it do in the same request.

  Each cleanup send swallows a link-down error, because cleanup runs precisely when the
  session is ending (often because the link itself dropped) and a secondary "could not
  clean up" error would only mask the real reason.
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import List

# Own from-scratch building blocks (R0): siren waveform, Opus framing, and the two wire
# protocols. Nothing here comes from tools/ -- this module only composes our own layers.
from ..codec.opus_stream import OpusEncoderStream
from ..config import PayloadConfig
from ..core.device_link import DeviceLink, DeviceLinkError
from ..core.siren import SirenSpec, synthesize
from ..protocol.audio_8519 import VOICE_MALE, build_hail, build_hail_stop, build_tts
from ..protocol.lights_8529 import (
    BRIGHT_MAX,
    REDBLUE_MAX,
    REDBLUE_MIN,
    build_brightness,
    build_redblue,
    build_searchlight,
    build_strobe,
)

# Namespaced under "payload." like the rest of the service so an operator can tune the
# verbosity of just the deter loop without touching the REST or transport logs.
logger = logging.getLogger("payload.deter")

# Device hail format for the siren stream: 8 kHz Opus in 480-sample (60 ms) frames -- the
# exact rate/frame the 8519 [42] path was verified with, and (crucially) the rate
# core.siren already synthesises at, so the siren feeds the encoder with no resample.
_HAIL_FS = 8000  # device hail sample rate; also exactly what core.siren emits (no resample)
_HAIL_FRAME = 480  # 60 ms @ 8 kHz -- the [42] frame size verified on real hardware
_HAIL_FRAME_S = _HAIL_FRAME / _HAIL_FS  # 0.06 s of audio per [42] frame

# Siren streaming pace (14 §7.3 按节流 streaming): ONE frame per frame-period, each sent
# at the moment it is due. See _play_siren for why neither bursting nor chunking works on
# this device.

# Deter light defaults. red/blue mode 1 is a valid built-in flash pattern (1..16); which
# exact pattern is "most deterrent" is a firmware detail we cannot introspect, so 1 is the
# documented default and POST /deter's mode overrides it. Full searchlight brightness (30)
# is used because deter wants maximum visual presence -- and because the searchlight is
# STEADY here (14 GL-2, see _set_deter_lights), brightness is the only lever it has.
_DEFAULT_REDBLUE_MODE = 1  # first valid built-in flash pattern (0 = off is disallowed)
_DEFAULT_SIREN_LEVEL = 0.45  # master amplitude 0..1; matches SirenSpec's own default
_DEFAULT_TTS_REPS = 2  # warnings spoken per cycle before the siren plays again
_DETER_BRIGHT = BRIGHT_MAX  # 30: full glare; the searchlight is steady, so this is its only lever

# Loop timing. _TTS_MARGIN_S is added to the per-utterance estimate because the estimate
# is a floor, not an exact playback time (the device has real start/stop latency and gives
# no done event). _AUDIO_RETRY_GAP_S paces the loop after an audio-send failure so a
# persistently dead audio link does not spin the loop hot while the lights keep flashing.
_TTS_MARGIN_S = 0.3  # slack over est_ms per utterance (the estimate is a floor, not exact)
_AUDIO_RETRY_GAP_S = 0.5  # pace after an audio-send failure so the loop cannot spin hot

# How long stop() waits for the cancelled loop task to unwind before giving up and
# proceeding to the light/audio reset anyway. Generous because a loop parked in a
# to_thread send returns quickly, but a wedged send should not block teardown forever.
_STOP_JOIN_TIMEOUT_S = 5.0  # cap on the join before the reset runs regardless


class DeterParamError(ValueError):
    """Raised when DeterParams holds a value the deter loop cannot use.

    House rule bans bare Exception; a dedicated type lets the POST /deter handler tell an
    illegal-parameter fault (which is a 4xx the client can fix) apart from a device fault.
    Subclassing ValueError keeps it semantically a bad-value error: every case it covers
    -- a red/blue mode outside 1..16, a siren level outside 0..1, negative repeats -- is a
    bad input value, not a runtime condition.
    """


@dataclass(frozen=True)
class DeterParams:
    """The three tunables POST /deter carries into one deter run (development plan 5.1).

    Frozen because a run's parameters are a fixed snapshot: to change them you stop the
    loop and start a new one with fresh params, which is exactly what SessionManager does,
    so there is never a reason to mutate a live params object. The utterance itself is NOT
    here -- it comes from PayloadConfig.deter_tts_text, since POST /deter has no text
    field -- so this DTO mirrors the request body one-to-one and nothing more.

    Fields:
        redblue_mode: which built-in red/blue flash pattern to run, 1..REDBLUE_MAX. 0
            (off) is deliberately NOT allowed here: a deter run with the warning light off
            is not a deter run, so the floor is 1, unlike the general /lights route.
        siren_level: master siren amplitude 0..1, passed straight to SirenSpec.level.
        tts_reps: how many times to speak the warning per cycle; 0 means siren-and-lights
            only (a valid, quieter deterrent), so the floor is 0, not 1.
    """

    redblue_mode: int = _DEFAULT_REDBLUE_MODE
    siren_level: float = _DEFAULT_SIREN_LEVEL
    tts_reps: int = _DEFAULT_TTS_REPS

    def validate(self) -> None:
        """Reject params that cannot drive a valid deter run, naming the bad field.

        Raises:
            DeterParamError: on a red/blue mode outside 1..REDBLUE_MAX, a siren level
                outside 0..1, or a negative repeat count.

        Validation lives on the DTO (not only at the HTTP boundary) so that any caller
        -- the REST handler, a test, a future internal trigger -- gets the same check,
        and DeterController can assume its params are sane at construction time.
        """
        # 1..MAX, not 0..MAX: a deter run with the red/blue light off is meaningless.
        if not 1 <= self.redblue_mode <= REDBLUE_MAX:
            raise DeterParamError(
                f"redblue_mode {self.redblue_mode} out of range 1..{REDBLUE_MAX}"
            )
        # Level scales the final waveform; core.siren also checks this, but catching it
        # here names siren_level specifically instead of surfacing as a SirenError later.
        if not 0.0 <= self.siren_level <= 1.0:
            raise DeterParamError(
                f"siren_level {self.siren_level} out of range 0..1"
            )
        # 0 repeats is allowed (siren + lights only); a negative count is a caller mistake.
        if self.tts_reps < 0:
            raise DeterParamError(f"tts_reps {self.tts_reps} must be >= 0")


class DeterController:
    """Owns one deter run: sets the lights, loops siren + warning, resets on stop.

    Lifecycle is start() then stop(), each called once by SessionManager. start() renders
    the siren, sets the lights, and spawns the loop task; stop() cancels that task and
    resets every device aspect deter touched. The controller borrows -- never owns -- the
    device sockets through the injected DeviceLink, honouring invariant R1 (one socket
    owner). Every blocking device write is pushed off the event loop with asyncio.to_thread
    so the loop never stalls the server, the same discipline the REST and WS planes use.
    """

    def __init__(self, link: DeviceLink, config: PayloadConfig, params: DeterParams) -> None:
        """Validate params and capture the collaborators; do no device I/O yet.

        Args:
            link: the shared socket owner deter borrows for its sends.
            config: supplies the deter utterance and the TTS timing estimate.
            params: the validated run parameters (validate() is called here so an illegal
                request fails at construction, before any light or sound is produced).

        Rendering the siren and touching the device are deliberately NOT done here -- they
        belong to start() -- so constructing a controller is cheap and side-effect-free,
        which keeps "build the object" and "start driving the hardware" separable and
        testable.
        """
        params.validate()  # fail fast: an illegal request raises here, before any device I/O
        self._link = link  # borrowed, never owned -- honours invariant R1 (one socket owner)
        self._config = config
        self._params = params
        # The running loop task, or None before start()/after stop(). Guarded so stop()
        # is safe to call even if start() never ran or already stopped.
        self._task: "asyncio.Task[None] | None" = None
        # Pre-rendered siren [42] frames and the clip's playback seconds, filled by
        # start()'s render step so the loop replays a ready buffer with no per-cycle synth.
        self._siren_frames: List[bytes] = []
        self._siren_seconds = 0.0

    async def start(self) -> None:
        """Render the siren, set the deter lights, and spawn the loop task.

        Rendering runs on a worker thread (numpy synth + Opus encode is CPU work) so it
        never blocks the event loop; the lights are then set ONCE (the firmware animates
        the pattern thereafter); finally the audio loop task is created. Ordering matters:
        lights come up immediately so the visual deterrent is present the instant deter
        starts, even before the first siren frame is encoded and sent.
        """
        await asyncio.to_thread(self._render_siren)
        await self._set_deter_lights()
        self._task = asyncio.create_task(self._run())  # spawn the loop; start() returns now
        logger.info(
            "deter started: redblue=%d siren_level=%.2f tts_reps=%d",
            self._params.redblue_mode,
            self._params.siren_level,
            self._params.tts_reps,
        )

    async def stop(self) -> None:
        """Cancel the loop and reset every device aspect deter touched.

        Idempotent and always safe: if the loop never started, or already stopped, the
        cancel step is skipped and only the (harmless, best-effort) device reset runs. The
        reset is unconditional because it is the deter acceptance contract (development
        plan section 13): lights fully off, red/blue reset, siren/TTS stopped, whatever
        state the loop was in when it was cancelled.
        """
        task = self._task
        self._task = None  # null before awaiting so a re-entrant stop() finds nothing to cancel
        if task is not None:
            # Cancel the loop and wait for it to unwind so no orphan keeps sending after
            # we begin the reset. The task raises CancelledError out of whatever await it
            # was parked on; that is expected and swallowed. A wedged send that overruns
            # the join timeout must not block teardown, so we log and reset anyway.
            task.cancel()
            try:
                await asyncio.wait_for(task, timeout=_STOP_JOIN_TIMEOUT_S)
            except asyncio.CancelledError:
                pass
            except asyncio.TimeoutError:
                logger.warning("deter loop did not unwind within %.1fs", _STOP_JOIN_TIMEOUT_S)
        await self._reset_device()  # unconditional: the teardown contract runs regardless
        logger.info("deter stopped: lights reset, audio silenced")

    def _render_siren(self) -> None:
        """Synthesise the siren once and cache it as ready-to-send [42] hail frames.

        Runs on a worker thread (start() calls it via asyncio.to_thread) because both the
        numpy waveform synthesis and the Opus encode are CPU-bound work that must not run
        on the event loop. The clip is rendered ONCE here and replayed every loop cycle,
        so the steady-state per-cycle cost is only the socket send -- there is no repeated
        synthesis and no per-cycle allocation of the frame list.

        The siren comes out of core.siren already at the device hail rate (8 kHz mono), so
        unlike the /play path there is NO resample step: the PCM feeds the Opus encoder
        directly. flush() is called after encode() so the encoder's final partial frame is
        emitted too and the cached clip is whole rather than clipped to a frame boundary.
        """
        # Amplitude is the only per-run knob; the rest of the siren shape is the core.siren
        # default. siren_level was already range-checked by DeterParams.validate() in
        # __init__, so constructing the spec here cannot raise a SirenError for a bad level.
        spec = SirenSpec(level=self._params.siren_level)
        # Synthesise to interleaved 16-bit PCM bytes at the hail rate -- no resample needed
        # because core.siren already emits at _HAIL_FS, the device's own hail sample rate.
        pcm = synthesize(spec).tobytes()
        # Encode the PCM to 480-sample (60 ms) Opus packets and wrap each as a [42] hail.
        encoder = OpusEncoderStream(_HAIL_FS, _HAIL_FRAME)
        frames = [build_hail(packet) for packet in encoder.encode(pcm)]
        # flush() drains the encoder's final partial frame so the cached clip is complete.
        frames += [build_hail(packet) for packet in encoder.flush()]
        self._siren_frames = frames
        # Cache the clip length so _play_siren knows how long to let it play before [11].
        self._siren_seconds = spec.seconds

    async def _set_deter_lights(self) -> None:
        """Raise the full visual deterrent in one batched 8529 control group.

        The four frames are handed to send_lights_control together (one call) so
        device_link writes them back-to-back under its send lock, with no other sender's
        frames able to slip between them -- the searchlight, its brightness, its strobe,
        and the red/blue pattern all take effect as one command rather than four racing
        ones that could momentarily show a half-configured light state.

        Sent ONCE per run, not per loop cycle: 8529 control frames are fire-and-forget and
        the device firmware animates the strobe and red/blue patterns autonomously once
        selected, so re-sending them every cycle would be pure noise on the wire. The send
        is pushed to a worker thread because send_lights_control does blocking socket I/O
        that must never run on the event loop.
        """
        frames = [
            # ★ Strobe OFF, explicitly and FIRST. 14 GL-2 forbids ever sending
            # MSG_STROBE[1] to the searchlight, and 14 §4.3.0 requires deter mode in
            # particular to send MSG_STROBE[0] at start rather than merely refrain --
            # the lamp may have been left flashing by something else, and deter is the
            # mode that must not inherit it.
            #
            # The reason is not aesthetic. The searchlight is the fill light night
            # perception depends on (PER-42): while it strobes, the camera only images
            # during the flashes, so exactly when the robot has found something worth
            # deterring, it stops being able to see it. The warning FLASH belongs to the
            # red/blue lamp below; the searchlight's job here is steady illumination.
            build_strobe(False),
            # Searchlight ON, then full brightness -- 00 VOI-43 spells out the D-mode
            # combination as siren + red/blue flashing + searchlight STEADY at maximum.
            build_searchlight(True),
            build_brightness(_DETER_BRIGHT),
            # Red/blue warning pattern (already validated to 1..REDBLUE_MAX by the DTO).
            # This is the 爆闪 of the requirements (00 PAY-02c): the firmware animates the
            # chosen pattern on its own, which is why lights are set once and not driven.
            build_redblue(self._params.redblue_mode),
        ]
        # One batched, blocking socket write -> worker thread so the loop never stalls.
        await asyncio.to_thread(self._link.send_lights_control, frames)

    async def _run(self) -> None:
        """Drive the audio deterrent forever: siren, then spoken warnings, repeat.

        Only the AUDIO deterrent is driven in the loop body; the lights were set once by
        start() and the firmware animates them, so there is nothing about them to re-send
        here. The loop runs until the task is cancelled by stop().

        Two exceptions are treated very differently, and the difference is the point:
          - CancelledError (raised into the loop by stop()) is deliberately NOT caught. It
            propagates out to end the loop, and stop() -- not this method -- performs the
            device reset. Catching it here would defeat cancellation and strand the loop.
          - DeviceLinkError is caught and swallowed with a pace. A transient audio-link
            drop must not kill the whole deterrent, because the lights (a separate 8529
            socket, set once at start) keep flashing as the primary visual deterrent. The
            loop logs the fault, waits _AUDIO_RETRY_GAP_S so a persistently dead link
            cannot spin the loop hot, and retries the audio next cycle.
        """
        while True:
            try:
                # One deterrent cycle: play the whole siren clip, then speak the warning(s).
                await self._play_siren()
                await self._speak_warnings()
            except DeviceLinkError as exc:
                # Audio hiccup only: the lights keep flashing from firmware, so pace+retry.
                logger.warning("deter: audio send failed, lights continue: %s", exc)
                await asyncio.sleep(_AUDIO_RETRY_GAP_S)

    async def _play_siren(self) -> None:
        """Stream the cached siren clip once at playback rate, then send the [11] stop.

        ★ ONE frame per frame-period, each sent when it is due. Both of the obvious
        alternatives were tried on the real device and both are audibly wrong.

        BURSTING the whole clip (the original implementation) rests on the premise that
        "the device buffers it and plays it out". That premise is false: of a 6.00 s clip
        the device produced 3.47 s, 0.35 s, 0.27 s and nothing at all on four consecutive
        runs. Its audio buffer holds whatever it holds at that instant and the rest is
        discarded, so the deterrent's primary audible element played a fraction of a
        second, non-deterministically.

        CHUNKING -- sending 240 ms at a time while keeping half a second queued ahead --
        fixes the truncation but sounds choppy, because this device plays close to as it
        receives: four frames arriving within a millisecond and then nothing for 240 ms is
        heard as a stutter, not as a buffer being topped up.

        What works is the pace the deterrent prototype has always used and 14 §7.3
        specifies as 按节流 streaming: hand over one 60 ms frame every 60 ms, so delivery
        matches playback and the device is never asked to buffer anything.

        Every await is a cancellation point, so stop() unwinds mid-siren within a frame
        rather than after the clip finishes.
        """
        # Nothing to play if the render produced no frames (defensive: a zero-length clip).
        if not self._siren_frames:
            return
        started = time.monotonic()
        for index, frame in enumerate(self._siren_frames):
            # device_link serialises this under the audio send lock.
            await asyncio.to_thread(self._link.send_audio, [frame])
            # Sleep until this frame's playback slot ends, computed as an ABSOLUTE deadline
            # from the clip's start rather than as a fixed per-frame delay. Accumulating
            # delays would let each send's latency add up and drift the stream late over a
            # hundred frames; anchoring to `started` keeps every frame on its own slot.
            due_s = started + (index + 1) * _HAIL_FRAME_S
            delay_s = due_s - time.monotonic()
            if delay_s > 0:
                await asyncio.sleep(delay_s)
        # Clean end to the hail so no siren tail is left ringing before the next cycle.
        await asyncio.to_thread(self._link.send_audio, [build_hail_stop()])

    async def _speak_warnings(self) -> None:
        """Speak the configured warning line in the male voice, tts_reps times.

        The utterance is config.deter_tts_text (POST /deter carries no text field, so the
        line is configuration, per 99 Q-C29-1) and the voice is male per development plan
        section 1 (男声). Because the device emits no "TTS finished" event, every wait here
        is paced by the SAME estimate POST /tts returns -- estimate_tts_ms plus a margin for
        the device's real start/stop latency -- since there is nothing to wait on but a clock.

        ★ The line is split on "|" into SEGMENTS, each sent as its own [31]. That is how the
        pause after 警告 is produced: we wait out the segment's estimate plus a deliberate
        deter_tts_pause_ms before sending the next one. The alternative -- one [31] with
        punctuation and hoping the device's TTS pauses on it -- was not chosen because
        nothing in the protocol notes or the device's observed behaviour says it treats
        punctuation as timing, so the pause would be unverifiable and vendor-dependent.

        The gap AFTER the last segment is the larger _TTS_MARGIN_S one, because that gap
        separates whole repetitions rather than the parts of one sentence.

        A tts_reps of 0, or a line with no speakable segment, means "no spoken warning this
        cycle": the method returns at once, leaving a valid siren-and-lights-only deterrent.
        Each sleep is a cancellation point, so a stop() mid-warning unwinds promptly -- and
        because segments are separate sends, a stop lands between them at worst one segment
        late rather than one whole sentence late.
        """
        # Split into segments and drop any empty ones, so a stray "|" or trailing separator
        # cannot turn into a zero-length utterance the device would reject.
        segments = [part for part in self._config.deter_tts_text.split("|") if part.strip()]
        # Zero repeats or no line: a valid siren-only deterrent, so nothing to speak.
        if self._params.tts_reps <= 0 or not segments:
            return
        # Build each [31] male-voice frame once (the text is fixed for the whole run) and
        # pair it with the wait that follows it. The last segment ends a whole repetition,
        # so it takes the mic-gate estimate plus the repetition margin; every earlier
        # segment takes the tighter mid-sentence gap, which is audible rather than a gate.
        last_index = len(segments) - 1
        plan = []
        for index, text in enumerate(segments):
            if index == last_index:
                wait_s = self._config.estimate_tts_ms(text) / 1000.0 + _TTS_MARGIN_S
            else:
                wait_s = self._config.estimate_segment_gap_ms(text) / 1000.0
            plan.append((build_tts(VOICE_MALE, text), wait_s))
        for _ in range(self._params.tts_reps):
            for frame, wait_s in plan:
                await asyncio.to_thread(self._link.send_audio, [frame])
                await asyncio.sleep(wait_s)

    async def _reset_device(self) -> None:
        """Best-effort, unconditional reset of every device aspect deter touched.

        This is the deter acceptance contract (development plan section 13): 灯全灭收尾 --
        searchlight off, strobe off, red/blue reset to off, and any siren tail silenced. It
        runs on every stop() regardless of how the loop ended (clean cancel, wedged send,
        or a dropped link), so the device is always left idle rather than stuck flashing.

        Lights and audio are reset in two SEPARATE try blocks so a failure resetting one
        still attempts the other -- a dead lights link must not prevent the hail-stop, and
        a dead audio link must not prevent the lights-off. A DeviceLinkError is swallowed
        (logged only) on each block because this reset runs precisely when the session is
        already ending, often because the link itself dropped; re-raising a "could not
        clean up" error here would only mask the real reason the session is stopping.
        """
        # All three visual aspects to their off state in one group: strobe, searchlight,
        # and red/blue -> REDBLUE_MIN (0 = off). Sent as one batch like _set_deter_lights.
        lights_off = [
            build_strobe(False),
            build_searchlight(False),
            build_redblue(REDBLUE_MIN),
        ]
        try:
            # Reset the lights first; swallow a link-down so the audio reset still runs.
            await asyncio.to_thread(self._link.send_lights_control, lights_off)
        except DeviceLinkError as exc:
            logger.warning("deter reset: lights-off failed: %s", exc)
        try:
            # Final [11] silences any siren burst still buffered or playing on the device.
            await asyncio.to_thread(self._link.send_audio, [build_hail_stop()])
        except DeviceLinkError as exc:
            logger.warning("deter reset: hail-stop failed: %s", exc)
