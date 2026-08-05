"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: intercom.py
Brief: 功能2 远场对讲 -- half-duplex PTT routing between office-client and the robot.

Description:
  The 功能2 side of AI_runtime (plan sections 5.5 and 6): a WebSocket server that
  office-client connects to, and which routes raw voice between the office and the robot
  in ONE direction at a time. There is no ASR and no LLM on this path -- 功能2 carries
  speech as speech.

  Two directions, and they are no longer symmetric in how they reach the hardware:

    talk   office microphone -> here -> payload-service WS /play -> device [42] speaker
    listen microphone ON THE ROBOT -> here -> office speaker

  The listen direction does NOT use the device's own microphone. Issue A1 measured the
  payload's [40] uplink at 0.235x realtime with the missing three quarters never sent, so
  the test microphone is mounted on the dog and stands in for it (local_mic.py). That is a
  decision about WHERE the microphone is, not a workaround inside the audio path: the
  frames are ordinary 16 kHz PCM either way.

  Why half-duplex, and why it is now a physical requirement rather than a simplification:
  the microphone and the loudspeaker are both on the dog, a short distance apart, so
  anything the office operator says comes out of the robot's speaker and straight back
  into the robot's microphone. With no AEC anywhere in the system (issue A5) that is a
  loop, not an artefact. Full duplex needs echo cancellation at both ends and is explicitly
  out of scope; PTT sidesteps it by never having both directions open at once.

  The gate is the same shape as the 功能1 gate in turn_loop, and for the same reason: the
  microphone is read CONTINUOUSLY in every PTT state and its frames are dropped when they
  are not wanted. Not stopped at the source, because restarting capture costs the front of
  the next utterance. Not left unread, because an unread pipe fills, and then the reader is
  behind by however long it stopped listening -- an intercom would drift further out of
  realtime with every transmission.

  Turnaround guard: releasing PTT does not silence the robot instantly. Closing the /play
  socket makes payload-service flush and send [11], but the sound already in the air still
  reaches the microphone. Forwarding immediately on release would send the operator the
  tail of their own voice, so listen frames stay gated for intercom_turnaround_ms after
  leaving talk. Like tts_gate_margin_ms in 功能1 this is a timing margin that has to be
  calibrated on the real device; the default is a starting point, not a measurement.

  One office-client at a time. Invariant R2 gives payload-service a single audio client,
  and a second office-client would be a second voice competing for the same loudspeaker,
  so a further connection is closed immediately rather than queued -- an intercom that
  silently mixes two operators is worse than one that refuses the second.

  Written from scratch per R0; websockets is the whitelisted transport underneath.
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Optional

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed, WebSocketException

from .config import AiRuntimeConfig
from .local_mic import LocalMicError, LocalMicStream
from .payload_client import (
    MODE_FUNC2,
    MODE_IDLE,
    PayloadClientError,
    PlayStream,
    ensure_mode,
)

logger = logging.getLogger("ai_runtime.intercom")

# The three PTT positions office-client can select (plan section 5.5). "idle" is a real
# state, not the absence of one: it means the operator is holding the channel without
# routing either way, which is how they stop the robot transmitting without starting to
# talk themselves.
PTT_TALK = "talk"
PTT_LISTEN = "listen"
PTT_IDLE = "idle"
_PTT_STATES = frozenset({PTT_TALK, PTT_LISTEN, PTT_IDLE})

# Format of the PCM crossing this socket in BOTH directions, announced to office-client on
# connect. It matches the mic geometry the rest of the system is built around (plan section
# 7) so no reblocking or resampling happens here -- frames pass through untouched.
_HEADER = {
    "encoding": "s16le",
    "sample_rate": 16000,
    "channels": 1,
    "frame_bytes": 640,
}

# WebSocket close codes used when refusing or ending a session. 1013 (try again later) says
# the refusal is temporary -- another operator holds the channel -- which is true and is
# what a client should retry on, unlike a protocol error.
_WS_CLOSE_BUSY = 1013
_WS_CLOSE_FAULT = 1011


class IntercomError(RuntimeError):
    """Raised when the intercom cannot run: bad control traffic, or a routing fault.

    House rule bans bare Exception. Distinct from PayloadClientError and LocalMicError
    (which name a specific service or device fault) because this one covers the intercom's
    OWN protocol with office-client -- a malformed control message, an unknown PTT state --
    where the fix is on the client side rather than in a service.
    """


def _parse_control(raw: str) -> str:
    """Extract the requested PTT state from an office-client control message.

    Args:
        raw: the text frame received, expected to be {"ptt": "talk"|"listen"|"idle"}.

    Returns:
        The requested state, one of the PTT_* constants.

    Raises:
        IntercomError: if the message is not that object, or names a state this server
            does not implement.

    An unknown state is refused rather than mapped onto idle. Guessing would mean a
    client typo silently produces a channel that looks connected and carries nothing,
    which is the hardest kind of intercom fault to diagnose from the operator's end.
    """
    try:
        message = json.loads(raw)
    except ValueError as exc:
        raise IntercomError(f"control message is not valid json: {raw[:200]}") from exc
    if not isinstance(message, dict):
        raise IntercomError(f"control message is not a json object: {raw[:200]}")
    state = message.get("ptt")
    if state not in _PTT_STATES:
        raise IntercomError(f"unknown ptt state {state!r}, expected one of {sorted(_PTT_STATES)}")
    return state


class IntercomSession:
    """Routes audio for ONE connected office-client until it disconnects or faults.

    Holds the two things a session owns -- the microphone on the robot and, while
    transmitting, a /play socket to the device loudspeaker -- and the PTT state that
    decides which way audio moves. Both pumps run as tasks raced against each other, so
    whichever ends first (the client disconnecting, the microphone dying) ends the session
    and the other is cancelled.
    """

    def __init__(self, config: AiRuntimeConfig, connection: Any) -> None:
        """Capture the collaborators; open nothing yet.

        Args:
            config: supplies the payload URL, the microphone source and the turnaround.
            connection: the accepted office-client websocket.
        """
        self._config = config
        self._connection = connection
        # Start idle: a freshly connected client has not pressed anything, and defaulting
        # to either direction would put audio on the wire nobody asked for.
        self._ptt = PTT_IDLE
        # The /play socket, held only while transmitting. None whenever PTT is not talk,
        # because closing it is what ends the hail cleanly (see PlayStream's docstring).
        self._play: Optional[PlayStream] = None
        # Monotonic deadline before which listen frames are still dropped. Set when leaving
        # talk so the operator does not hear the tail of their own voice come back.
        self._gate_until = 0.0
        self._turnaround_s = config.intercom_turnaround_ms / 1000.0

    async def run(self) -> None:
        """Serve this client: announce the format, then route until something ends it.

        Raises:
            LocalMicError: if the microphone on the robot cannot be opened or stops.
            PayloadClientError: if the device loudspeaker path fails.
            IntercomError: if the client sends control traffic this server cannot honour.

        The microphone is opened BEFORE the header is sent, so a client that gets a header
        knows the robot can actually hear -- otherwise the operator would connect, press
        listen, and hear nothing, with no way to tell a silent room from a dead capture.
        """
        async with LocalMicStream(self._config) as mic:
            await self._connection.send(json.dumps(_HEADER))
            logger.info("office-client session open")
            # Two independent pumps: one draining the client, one draining the microphone.
            # Raced rather than awaited in sequence because either can end the session and
            # neither returns while things are healthy.
            office = asyncio.create_task(self._pump_office())
            robot = asyncio.create_task(self._pump_robot(mic))
            try:
                done, pending = await asyncio.wait(
                    {office, robot}, return_when=asyncio.FIRST_COMPLETED
                )
                # Cancel the loser and let it unwind before the microphone context exits,
                # so no orphan task is still reading a stream that is being closed.
                for task in pending:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
                # Re-raise whatever ended the winner. A pump that returned normally (the
                # client disconnected) has no exception and simply ends the session.
                for task in done:
                    error = task.exception()
                    if error is not None:
                        raise error
            finally:
                # Always end any transmission in progress: the session may be ending in the
                # middle of a talk burst, and an abandoned /play socket would leave the
                # robot's loudspeaker parked on the last frame it was handed.
                await self._stop_talking()

    async def _pump_office(self) -> None:
        """Receive from office-client: control messages and, while talking, PCM.

        Returns:
            None when the client disconnects, which is the normal end of a session.

        Raises:
            IntercomError: on control traffic that cannot be honoured.
            PayloadClientError: if forwarding to the device loudspeaker fails.

        PCM that arrives when PTT is not talk is DROPPED rather than forwarded. That is the
        server-side half of half-duplex: a client whose microphone keeps streaming after
        release must not be able to keep the robot talking, because the enforcement that
        matters is the one at the end that owns the loudspeaker.
        """
        while True:
            try:
                message = await self._connection.recv()
            except ConnectionClosed:
                # Client hung up. Not a fault -- the operator closed the intercom.
                logger.info("office-client disconnected")
                return
            if isinstance(message, str):
                await self._set_ptt(_parse_control(message))
                continue
            # Binary: the operator's voice. Only forwarded while actually transmitting.
            if self._play is not None:
                await self._play.send(message)

    async def _pump_robot(self, mic: LocalMicStream) -> None:
        """Read the robot's microphone continuously; forward only while listening.

        Args:
            mic: the open capture stream on the robot.

        Raises:
            LocalMicError: if capture stops, which ends the session -- an intercom whose
                robot end has gone deaf has nothing left to do.

        Every frame is read in every PTT state. Dropping here rather than pausing capture is
        the whole gate discipline (module docstring): a paused source loses the front of the
        next transmission, and an unread one falls behind realtime.
        """
        async for frame in mic:
            # Gate: forward only while listening AND past the turnaround deadline, so the
            # operator never receives the decaying tail of their own transmission.
            if self._ptt != PTT_LISTEN or asyncio.get_running_loop().time() < self._gate_until:
                continue
            try:
                await self._connection.send(frame)
            except ConnectionClosed:
                # The client vanished mid-transmission; ending the pump ends the session.
                logger.info("office-client disconnected while listening")
                return

    async def _set_ptt(self, state: str) -> None:
        """Apply a PTT change, opening or closing the device loudspeaker path with it.

        Args:
            state: the requested state, already validated by _parse_control.

        Raises:
            PayloadClientError: if the loudspeaker path cannot be opened.

        A repeat of the current state is ignored rather than re-applied: re-entering talk
        would tear down and rebuild a working /play socket mid-sentence, cutting the very
        audio the operator is in the middle of speaking.
        """
        if state == self._ptt:
            return
        logger.info("ptt %s -> %s", self._ptt, state)
        # Leaving talk: stop the hail first, and start the turnaround so the microphone
        # stays gated while the sound already in the room dies away.
        if self._ptt == PTT_TALK:
            await self._stop_talking()
            self._gate_until = asyncio.get_running_loop().time() + self._turnaround_s
        self._ptt = state
        if state == PTT_TALK:
            # Entering talk: open the loudspeaker path. Opened here rather than held for
            # the whole session because closing it is what makes the hail end cleanly.
            play = PlayStream(self._config)
            await play.__aenter__()
            self._play = play

    async def _stop_talking(self) -> None:
        """Close the loudspeaker path if one is open; safe to call when none is.

        Idempotent because it runs both on a PTT change and in the session teardown, and on
        a release-then-disconnect it runs twice. Nulling the handle before awaiting the
        close means a second call finds nothing to do even if the first is still unwinding.
        """
        play = self._play
        self._play = None
        if play is not None:
            await play.__aexit__(None, None, None)


class IntercomServer:
    """Listens for office-client, holds the payload in func2, serves one session at a time.

    Owns the process-level concerns: the mode (set once for the whole run, not per client,
    so a reconnecting operator does not cycle the device through idle) and the single-client
    rule. The per-connection routing belongs to IntercomSession.
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Capture the config; bind nothing until run()."""
        self._config = config
        # True while a session is being served. Checked and set with no await in between,
        # so on a single-threaded event loop it is a sufficient mutual exclusion.
        self._busy = False

    async def run(self) -> None:
        """Put the payload in func2, serve office-clients until cancelled, then restore.

        Raises:
            PayloadClientError: if payload-service will not enter func2 -- without it the
                /play route is refused, so there is no intercom to run.

        The mode is restored only if THIS process switched it, matching turn_loop: a
        process that found the payload already in func2 did not take it and should not
        give it back, because something else is relying on it.
        """
        # func2 is required before /play will be accepted at all (payload-service's gate),
        # so it is established before the listener opens rather than at the first press.
        switched = await asyncio.to_thread(ensure_mode, self._config, MODE_FUNC2)
        logger.info(
            "intercom listening on %s:%d (payload mode %s)",
            self._config.intercom_host,
            self._config.intercom_port,
            MODE_FUNC2,
        )
        try:
            async with serve(
                self._handle,
                self._config.intercom_host,
                self._config.intercom_port,
                # compression off: this is s16le PCM at 50 frames a second, which deflate
                # cannot usefully shrink but would spend CPU on at both ends.
                compression=None,
            ) as server:
                # Serve until cancelled; the process entry point owns the stop signal.
                await server.serve_forever()
        finally:
            if switched:
                # Hand the device back the way it was found. Runs even on a fault, because
                # leaving the payload parked in func2 would block the next mode a different
                # process wants (R3).
                await asyncio.to_thread(ensure_mode, self._config, MODE_IDLE)

    async def _handle(self, connection: Any) -> None:
        """Serve one office-client connection, refusing a second concurrent one.

        Args:
            connection: the accepted websocket.

        Faults are reported to the client with a close code and logged here rather than
        propagated, because one client's dead microphone or dropped device link must not
        take down the listener -- the operator's remedy is to fix it and reconnect, and a
        server that exited would leave them nothing to reconnect to.
        """
        if self._busy:
            # R2: one audio client. Refuse with "try again later" -- the channel is held,
            # not broken, so retrying is the correct client behaviour.
            logger.warning("refusing second office-client: channel already in use")
            await connection.close(code=_WS_CLOSE_BUSY, reason="intercom already in use")
            return
        self._busy = True
        try:
            await IntercomSession(self._config, connection).run()
        except (IntercomError, LocalMicError, PayloadClientError) as exc:
            # A fault on this session only. Tell the client why before dropping it, so the
            # operator sees a reason rather than a silent disconnection.
            logger.error("intercom session failed: %s", exc)
            await connection.close(code=_WS_CLOSE_FAULT, reason=str(exc)[:120])
        except WebSocketException as exc:
            # Transport-level trouble on this socket; the session is over either way.
            logger.warning("office-client connection error: %s", exc)
        finally:
            # Released whatever ended the session, so the next operator can connect.
            self._busy = False
