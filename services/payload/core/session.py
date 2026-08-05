"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: session.py
Brief: Mode state machine plus the R2/R3 gate that serialises all device audio use.

Description:
  SessionManager is the single authority for "which mode is the payload in, and what is
  therefore allowed to touch the device audio path right now". It is where test-system
  invariants R2 and R3 are physically enforced (development plan sections 4 and 6):
    - R3 (one mode at a time): the service is always in exactly one Mode -- IDLE, FUNC1
      (near-field AI dialogue), FUNC2 (far-field intercom), or DETER (驱离). AI_runtime
      DECIDES the mode by calling POST /mode; this class ENFORCES it, so a mode is never
      half-changed and two modes never run at once.
    - R2 (single audio client): within a mode, each audio kind (/mic uplink, /play hail)
      may have at most ONE active WebSocket session. A second connection of the same kind
      is refused rather than allowed to hijack the first, because the intended sole client
      is AI_runtime.

  Mode graph (why it is fully connected, not hub-and-spoke): section 13 acceptance requires
  that with FUNC1 active, POST /mode FUNC2 SUCCEEDS and the old /mic session is torn down.
  So a direct FUNC1 -> FUNC2 transition must be legal; the graph is therefore fully
  connected -- any mode may switch to any OTHER mode. The one illegal transition is
  switching to the mode already active (ModeTransitionError -> HTTP 409): re-entering a
  mode is a conflict, not a silent no-op, so the caller learns nothing was reset.

  Every cross-mode transition tears the old mode down first (see _teardown): it stops a
  running deter loop and closes every open audio session, and only then adopts the new
  mode. That ordering is what makes "mode switch" atomic from a client's view -- the old
  mode's device activity is fully quiesced before the new mode can arm anything.

  The audio-session teardown handshake (the subtle part): a WS handler does not own the
  right to keep streaming across a mode change, so closing it must be cooperative. Each
  AudioSession carries two asyncio.Events: `close` (SessionManager -> handler: "stop now")
  and `done` (handler -> SessionManager: "I have fully unwound"). _teardown sets `close`
  on every session, then awaits each `done` (bounded by _AUDIO_TEARDOWN_TIMEOUT_S). The WS
  handler, which is separately awaiting `close`, reacts by disarming its device use and
  calling finish_audio, which sets `done`. finish_audio deliberately takes NO lock and does
  NO await, so it completes atomically on the event loop while _teardown holds the mode
  lock and waits -- there is no lock-ordering deadlock between the two.

  Deter ownership: DETER mode is entered by POST /mode like any other mode, but entering it
  does NOT itself start the deterrent -- POST /deter {on:true} does, via start_deter, which
  requires the service to already be in DETER (else ModeStateError -> 409). This keeps mode
  selection (AI_runtime's job) separate from deterrent parameters (the /deter body), and
  leaving DETER via any transition stops the loop through the same _teardown path. The
  controller is built through an injected factory (default DeterController) so a test can
  substitute a fake and drive SessionManager without real hardware.

  Concurrency model: a single asyncio.Lock serialises every state-changing operation
  (transition, start/stop deter, opening an audio session, shutdown). Read-only predicates
  (current_mode, allows_mic/play/tts) take no lock: a Mode assignment is atomic on the
  single-threaded event loop, so a status read cannot observe a torn value, and a benign
  one-tick-stale answer is acceptable for a gate check that the lock-held operations
  re-validate anyway.
"""
from __future__ import annotations

import asyncio
import logging
from enum import Enum
from typing import Callable, Optional, Set

from ..config import PayloadConfig
from ..core.device_link import DeviceLink, DeviceLinkError
from ..core.deter import DeterController, DeterParams

# Namespaced under "payload." like the rest of the service so mode transitions and audio
# gate refusals can be tuned in the log independently of the transport and device layers.
logger = logging.getLogger("payload.session")

# How long _teardown waits for a single audio session to acknowledge its close (set its
# `done` Event) before giving up on it and proceeding. Generous because a well-behaved WS
# handler acknowledges within a couple of event-loop ticks, but a wedged or slow handler
# must never block a mode change forever -- teardown drops it and moves on after this.
_AUDIO_TEARDOWN_TIMEOUT_S = 5.0

# The two audio kinds a session can have. Declared as constants (not bare string literals
# at each call site) so a typo cannot silently create a third, never-gated "kind": the mic
# uplink (/mic, [40] record) and the hail playback (/play, [42]).
_KIND_MIC = "mic"
_KIND_PLAY = "play"


class Mode(str, Enum):
    """The four mutually exclusive payload modes (development plan section 6).

    Subclasses str so the enum member IS its wire value -- FastAPI/pydantic can accept and
    emit the plain strings ("idle"/"func1"/"func2"/"deter") in the POST /mode body and
    response without any extra conversion, and a log line prints the readable value. The
    string values are ASCII (house rule: no Chinese in code or API keys) even though the
    user-facing names elsewhere are 功能1/功能2/功能3.
    """

    IDLE = "idle"
    FUNC1 = "func1"
    FUNC2 = "func2"
    DETER = "deter"


# Gate tables: which modes permit each audio activity. Kept as module-level frozensets so
# the permission rule lives in exactly one place and is a cheap membership test.
#   - /mic and /play are the two-way audio of the interactive modes, so both are allowed in
#     FUNC1 (AI dialogue) and FUNC2 (intercom), and in neither IDLE (nothing running) nor
#     DETER (the deter loop owns the audio path exclusively -- R3).
#   - /tts (POST /tts, a one-shot spoken line) is allowed in IDLE (a simple announcement
#     while otherwise quiet) and FUNC1 (the AI speaks its reply as device TTS), but NOT in
#     FUNC2 (a live human intercom should not be interrupted by injected TTS) nor DETER
#     (the loop speaks its own male-voice warning; an external line would collide).
_MIC_MODES = frozenset({Mode.FUNC1, Mode.FUNC2})
_PLAY_MODES = frozenset({Mode.FUNC1, Mode.FUNC2})
_TTS_MODES = frozenset({Mode.IDLE, Mode.FUNC1})


class ModeError(RuntimeError):
    """Base for mode/gate faults so callers can catch the whole family at once.

    Subclasses RuntimeError, not ValueError: the value a caller supplies (a valid Mode, a
    well-formed /deter body) is fine in isolation; what fails is the runtime STATE -- the
    service is in the wrong mode, or already in the requested one -- so these are runtime
    conflicts, not bad values. House rule bans bare Exception, and a shared base lets the
    REST and WS layers translate any mode fault without importing each leaf type.
    """


class ModeTransitionError(ModeError):
    """Raised by transition() when the requested mode equals the current one.

    Re-entering the active mode is treated as a conflict (the REST layer maps it to HTTP
    409) rather than a silent success, so a client that expected a reset learns that
    nothing was torn down or re-armed. Every OTHER target mode is a legal transition.
    """


class ModeStateError(ModeError):
    """Raised when an operation is not permitted in the current mode.

    Covers three cases, all state conflicts: opening a /mic or /play session in a mode that
    does not allow it (the WS layer maps this to a 403-style close), starting or stopping
    the deterrent while not in DETER, and opening a second audio session of a kind that is
    already active (R2). The REST layer maps it to HTTP 409.
    """


# Signature of the deter-controller factory SessionManager calls to build one run. Default
# is the real DeterController; a test injects a stand-in with the same shape to exercise the
# manager's start/stop/teardown wiring without touching hardware.
DeterFactory = Callable[[DeviceLink, PayloadConfig, DeterParams], DeterController]


class AudioSession:
    """A single live audio activity (one /mic or one /play WebSocket) and its close signals.

    Identity, not value, is what matters: instances are held in a set and compared by
    object identity (the default), so this is a plain class rather than a dataclass -- two
    distinct /mic sessions must never be considered equal even though their fields match.

    The two Events are the cooperative-close handshake described in the module header:
        close: set by SessionManager (_teardown) to tell the owning handler to stop now.
        done:  set by the handler (via finish_audio) once it has fully unwound, which is
               what _teardown awaits so it knows the device activity has actually ceased.
    """

    def __init__(self, kind: str) -> None:
        """Capture the kind and create the two (initially unset) handshake Events.

        Args:
            kind: _KIND_MIC or _KIND_PLAY -- which audio activity this session represents.
                It selects the gate the session was admitted through and lets _teardown log
                which kind it is closing.
        """
        self.kind = kind
        # Both Events start clear: the session is open (not asked to close) and not yet
        # done. SessionManager sets close; the WS handler sets done via finish_audio.
        self.close = asyncio.Event()
        self.done = asyncio.Event()


class SessionManager:
    """Holds the current Mode, gates audio use, and drives deter start/stop and teardown.

    One instance lives for the process lifetime (built in the app lifespan, closed via
    aclose on shutdown). It borrows -- never owns -- the DeviceLink (invariant R1: the link
    is the sole socket owner) and uses the injected config for the deter utterance and TTS
    timing. All mutating methods run under a single asyncio.Lock so mode changes, deter
    control, and session opens are strictly serialised; the read-only predicates are
    lock-free point-in-time snapshots.
    """

    def __init__(
        self,
        link: DeviceLink,
        config: PayloadConfig,
        deter_factory: DeterFactory = DeterController,
    ) -> None:
        """Capture collaborators and start in IDLE with no audio and no deter running.

        Args:
            link: the shared device socket owner deter and (indirectly) the audio planes
                use; borrowed, never owned, per R1.
            config: supplies deter_tts_text and the TTS estimate for the deter loop.
            deter_factory: builds a DeterController from (link, config, params). Defaults to
                the real controller; injectable so a test can drive the manager with a fake.

        No device I/O happens here: constructing the manager only sets up in-memory state,
        so the app lifespan can build it before the links are even up. The lock is created
        here; on Python 3.10+ an asyncio.Lock does not bind to a loop at construction, and
        in practice this runs inside the lifespan's running loop anyway.
        """
        self._link = link
        self._config = config
        self._deter_factory = deter_factory
        # Start IDLE: the hub mode, with nothing using the device audio path.
        self._mode = Mode.IDLE
        # Serialises every state-changing method so R3 (one mode at a time) holds even
        # under concurrent requests -- two transitions can never interleave.
        self._lock = asyncio.Lock()
        # The set of currently-open audio sessions (0..1 mic and 0..1 play). A set because
        # membership and identity-removal are what teardown and finish_audio need.
        self._audio: Set[AudioSession] = set()
        # The running deter controller, or None when the deterrent is not running (which is
        # always the case outside DETER mode, and the default even within it until /deter).
        self._deter: Optional[DeterController] = None

    # -- read-only state (lock-free) --------------------------------------

    def current_mode(self) -> Mode:
        """Return the active mode (lock-free point-in-time read).

        Reading a single attribute is atomic on the single-threaded event loop, so no lock
        is needed; the value may change the instant after it is read, which is fine for
        GET /status and the POST /mode response's `previous` field.
        """
        return self._mode

    def allows_mic(self) -> bool:
        """Whether the current mode permits a /mic uplink session (FUNC1 or FUNC2)."""
        return self._mode in _MIC_MODES

    def allows_play(self) -> bool:
        """Whether the current mode permits a /play hail session (FUNC1 or FUNC2)."""
        return self._mode in _PLAY_MODES

    def allows_tts(self) -> bool:
        """Whether the current mode permits a one-shot POST /tts (IDLE or FUNC1).

        POST /tts is a fire-and-forget one-shot, so the REST handler checks this lock-free
        predicate and sends immediately; a mode flip in the one-tick race window is benign
        (at worst one stray TTS frame as the mode changes, which the new mode's teardown
        does not need to reap). The exclusive audio activities (/mic, /play, deter) instead
        go through the lock-held open_audio / start_deter, which cannot race a transition.
        """
        return self._mode in _TTS_MODES

    # -- audio session registry -------------------------------------------

    async def open_audio(self, kind: str) -> AudioSession:
        """Admit a new audio session for kind, atomically against mode changes.

        Args:
            kind: _KIND_MIC or _KIND_PLAY.

        Returns:
            A registered AudioSession the caller must later pass to finish_audio (in its
            finally) and whose `close` Event it must watch to react to a teardown.

        Raises:
            ModeStateError: if the current mode does not permit this kind, or if a session
                of this kind is already active (R2: one audio client per kind).
            ValueError: if kind is neither known constant (a programming error in a caller).

        The gate check and the registration happen under the same lock, so the admitted
        session cannot slip into a mode that a concurrent transition is just leaving: either
        this runs first (and the later transition tears the session down) or the transition
        runs first (and this sees the new mode and admits or refuses accordingly).
        """
        async with self._lock:
            gate = self._gate_for(kind)
            if self._mode not in gate:
                # Wrong mode for this activity: the WS layer turns this into a 403-close.
                raise ModeStateError(f"{kind} not allowed in mode {self._mode.value}")
            # R2: refuse a second client of the same kind rather than let it hijack the
            # first's device use. The sole intended client is AI_runtime, so a duplicate is
            # anomalous and the existing, working session is preserved.
            if any(sess.kind == kind for sess in self._audio):
                raise ModeStateError(f"a {kind} session is already active")
            sess = AudioSession(kind)
            self._audio.add(sess)
            logger.info("opened %s session in mode %s", kind, self._mode.value)
            return sess

    def finish_audio(self, sess: AudioSession) -> None:
        """Deregister a session and signal its close is complete (no lock, no await).

        Args:
            sess: the session returned by open_audio.

        Called by the WS handler when it has fully unwound -- whether that was triggered by
        a client disconnect or by a teardown's `close`. It is deliberately synchronous and
        lock-free: discarding from a set and setting an Event are atomic on the event loop,
        so this completes in one step even while _teardown holds the mode lock and awaits
        `done`. Requiring the lock here would deadlock exactly that teardown. Idempotent:
        discarding an already-removed session and re-setting an already-set Event are no-ops.
        """
        self._audio.discard(sess)
        sess.done.set()

    def _gate_for(self, kind: str) -> frozenset:
        """Return the set of modes that permit the given audio kind.

        Args:
            kind: _KIND_MIC or _KIND_PLAY.

        Raises:
            ValueError: for any other kind -- an internal programming error (only ws.py
                calls this, with the two constants), surfaced loudly rather than masked as a
                mode fault.
        """
        if kind == _KIND_MIC:
            return _MIC_MODES
        if kind == _KIND_PLAY:
            return _PLAY_MODES
        raise ValueError(f"unknown audio kind {kind!r}")

    # -- mode transitions -------------------------------------------------

    async def transition(self, new: Mode) -> Mode:
        """Switch to a different mode, tearing the old one down first; return the previous.

        Args:
            new: the mode to enter.

        Returns:
            The mode that was active before the switch (the POST /mode response's
            `previous`).

        Raises:
            ModeTransitionError: if new equals the current mode (a 409 conflict).

        The teardown-then-adopt order is what makes a switch atomic to a client: the old
        mode's deter loop and audio sessions are fully quiesced before self._mode changes,
        so nothing from the old mode can still be driving the device once the new mode is
        reported active. Entering DETER only sets the mode; the deterrent itself is armed
        later by start_deter.
        """
        async with self._lock:
            previous = self._mode
            if new == previous:
                # Re-entering the active mode is a conflict, not a silent reset.
                raise ModeTransitionError(f"already in mode {new.value}")
            # Quiesce everything the old mode was running before adopting the new one.
            await self._teardown()
            self._mode = new
            logger.info("mode transition %s -> %s", previous.value, new.value)
            return previous

    async def _teardown(self) -> None:
        """Stop the deter loop and cooperatively close every open audio session.

        Assumes the caller holds self._lock (transition and aclose both do). First stops a
        running deter controller (which resets the lights and silences audio -- section 13),
        then signals `close` on every audio session and awaits each `done`, bounded per
        session by _AUDIO_TEARDOWN_TIMEOUT_S so one wedged handler cannot stall the switch
        forever. Each session is discarded from the registry whether or not it acknowledged
        in time, so the registry is empty when this returns.

        Awaiting `done` here does not deadlock against finish_audio: finish_audio takes no
        lock and does no await, so the WS handler can set `done` while this coroutine holds
        the mode lock and waits on it.
        """
        # Stop the deterrent first: swap the handle out before awaiting so a concurrent
        # observer never sees a half-stopped controller, then let stop() reset the device.
        if self._deter is not None:
            deter = self._deter
            self._deter = None
            await deter.stop()
        # Snapshot the sessions before signalling: finish_audio mutates self._audio, and
        # iterating a snapshot keeps this stable even as sessions remove themselves.
        sessions = list(self._audio)
        # Signal every session to close first, so they all begin unwinding in parallel
        # rather than one-at-a-time behind each await below.
        for sess in sessions:
            sess.close.set()
        # Then await each acknowledgement, bounding the wait so a stuck handler is dropped.
        for sess in sessions:
            try:
                await asyncio.wait_for(sess.done.wait(), timeout=_AUDIO_TEARDOWN_TIMEOUT_S)
            except asyncio.TimeoutError:
                # Handler did not acknowledge in time: log and drop it so the switch can
                # complete. Its own finally will still run finish_audio harmlessly later.
                logger.warning(
                    "audio %s session did not close within %.1fs; dropping",
                    sess.kind,
                    _AUDIO_TEARDOWN_TIMEOUT_S,
                )
            # Remove regardless of whether it acknowledged, so the registry is clean.
            self._audio.discard(sess)

    # -- deter control ----------------------------------------------------

    async def start_deter(self, params: DeterParams) -> None:
        """Start (or restart) the deterrent loop; requires DETER mode.

        Args:
            params: the validated run parameters from the POST /deter body.

        Raises:
            ModeStateError: if the service is not in DETER mode (a 409 -- the caller must
                POST /mode deter first).
            DeterParamError: if params are invalid (the factory validates on construction).
            DeviceLinkError: if the controller cannot arm the device (link down); any
                partially-raised lights are reset before this propagates.

        Restart semantics: if a run is already active, it is stopped (device reset) before
        the new one starts, so POST /deter with fresh params cleanly re-arms rather than
        stacking two loops. self._deter is only stored after start() succeeds, so a failed
        start leaves no half-live controller behind.
        """
        async with self._lock:
            if self._mode is not Mode.DETER:
                raise ModeStateError(
                    f"deter runs only in deter mode, current={self._mode.value}"
                )
            # Restart: stop any existing run first so its lights/audio are reset before the
            # new params take over. Swap the handle out before awaiting stop().
            if self._deter is not None:
                previous = self._deter
                self._deter = None
                await previous.stop()
            # Build via the factory (validates params) and arm the device.
            controller = self._deter_factory(self._link, self._config, params)
            try:
                await controller.start()
            except DeviceLinkError:
                # start() failed partway (e.g. link dropped during the light-set): reset
                # any lights it already raised, then propagate so the caller sees the fault.
                await controller.stop()
                raise
            self._deter = controller
            logger.info("deter armed via session")

    async def stop_deter(self) -> None:
        """Stop the deterrent loop; requires DETER mode. Idempotent within DETER.

        Raises:
            ModeStateError: if the service is not in DETER mode (a 409).

        POST /deter {on:false} maps here. If no run is active (never started, or already
        stopped) this is a harmless no-op that still returns success, so a client can stop
        twice without error. Leaving DETER entirely goes through _teardown instead, which
        does not require the mode check because it runs as part of the transition.
        """
        async with self._lock:
            if self._mode is not Mode.DETER:
                raise ModeStateError(
                    f"deter runs only in deter mode, current={self._mode.value}"
                )
            if self._deter is not None:
                deter = self._deter
                self._deter = None
                await deter.stop()
            logger.info("deter disarmed via session")

    # -- shutdown ---------------------------------------------------------

    async def aclose(self) -> None:
        """Tear everything down and reset to IDLE for an orderly service shutdown.

        Called from the app lifespan before the DeviceLink is stopped. Runs the same
        _teardown as a mode switch (stopping deter, closing audio sessions) and then sets
        IDLE, so the process leaves the device idle and no background loop outlives the app.
        """
        async with self._lock:
            await self._teardown()
            self._mode = Mode.IDLE
        logger.info("session manager closed: mode reset to idle, audio torn down")
