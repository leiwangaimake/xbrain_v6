"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: ws.py
Brief: payload-service WebSocket data plane -- WS /mic (record) and WS /play (hail).

Description:
  The binary audio side of payload-service (development plan section 5.2). Where the
  REST plane (api/rest.py) carries commands, this plane streams PCM: /mic pushes the
  device's recorded audio up to the local client, and /play accepts the client's audio
  and streams it out of the device as a realtime hail. Both follow the same wire
  convention: on connect the server first sends ONE JSON text frame declaring the PCM
  format, and every frame after that is binary -- one mono s16le PCM block each. The
  JSON-first handshake exists because raw PCM is format-blind: a receiver cannot tell
  8 kHz from 16 kHz, or where a frame starts, by looking at the bytes. Declaring the
  geometry once up front (rather than tagging every frame) keeps the steady-state path
  pure binary with zero per-frame overhead. The direction of that header differs by
  route, matching who produces the audio: on /mic the SERVER sends it (the server
  produces the PCM, so it declares the format), while on /play the CLIENT optionally
  sends it (the client produces the PCM, so it declares its own input rate).

  This module is the codec-driving layer. device_link stays codec-free (it owns only
  the transport and the [40] frame boundaries); here is where the Opus encode/decode
  and the 8k<->16k resample actually run, wiring the codec/ helpers to the WebSocket:
    - WS /mic  (server -> client): arm recording on the device, take the raw Opus
      packets device_link hands back, DECODE them to 16 kHz PCM, re-slice into fixed
      20 ms / 640-byte frames, and send each as a binary frame. The device's Opus frame
      size need not equal 20 ms, so decoded PCM is buffered and cut to a fixed frame,
      which is why a decoder-plus-buffer sits here rather than a 1:1 passthrough. A
      fixed 20 ms cadence is what downstream VAD/ASR stages expect, so normalising it
      at the source spares every later stage from re-framing.
    - WS /play (client -> server): accept 16 kHz (or client-declared 8 kHz) s16le,
      resample to the device's 8 kHz, ENCODE to 480-sample Opus, wrap each packet as a
      [42] hail frame, and send it. On close, flush the encoder's final partial frame
      and then send [11] so the device leaves the hail cleanly instead of on a silence.

  Threading bridge (the one genuinely cross-thread part): device_link invokes the mic
  sink on ITS supervisor thread, but the WebSocket lives on the asyncio event loop. The
  sink therefore does the minimum thread-safe thing -- loop.call_soon_threadsafe to drop
  the packet on an asyncio.Queue -- and all decoding and sending happen back on the loop
  in the pump coroutine. That keeps every WebSocket touch on the loop thread and every
  device-socket touch on device_link's threads, with a plain queue as the one hand-off.
  Concentrating the whole thread boundary at that single queue is deliberate: it is the
  only place a data race could live, so it is the only place that needs to be reasoned
  about, instead of scattering locks across the handler. The reverse direction (/play)
  needs no such bridge going down, but its blocking socket writes are still pushed off
  the loop with asyncio.to_thread, the same discipline the REST plane uses for its sends.

  Codec ordering is not symmetric between the two routes, and each order is forced by
  Opus. /mic must DECODE the device's Opus to PCM before it can re-slice to 20 ms frames
  -- compressed frames expose no sample-addressable boundary to cut on. /play must
  RESAMPLE to 8 kHz before it ENCODES, because the encoder is configured for 8 kHz and
  measures a frame in samples; handing it 16 kHz PCM would make every frame count as
  half its true duration and speak too fast. Both orders are what the codec/ helpers
  assume, so the wiring here has to respect them.

  Why these two are WebSocket while the control routes are REST: /mic and /play are
  continuous byte streams with no natural request/response boundary, which is exactly
  what WebSocket frames model and what a request-per-chunk REST call would model badly
  (a POST per 20 ms frame is absurd overhead). The commands in api/rest.py, by contrast,
  are discrete one-shot actions with a single reply, so they stay REST. The split is by
  traffic shape, not by module preference.

  Ownership boundary (invariant R1): like the REST handlers, these routes hold no socket
  and contain no transport logic. They borrow device_link's one audio socket through its
  send_audio / start_recording / stop_recording methods and let it serialise every write
  under its own send lock. The codec state (one decoder per /mic, one encoder per /play)
  lives on the handler's stack for the connection's lifetime and is discarded with it, so
  two connections can never share a half-updated codec.

  Single-client assumption (invariant R2): the whole audio plane is loopback-only and
  serves exactly one client at a time (AI_runtime). That is why the mic queue is left
  unbounded and recording is armed for the raw lifetime of one /mic socket -- there is
  no second consumer to starve and no fan-out to coordinate. If that ever changes it is
  a new milestone with its own backpressure design, not a flag bolted on here.

  Close-code taxonomy (why the two codes below are distinct): when the server ends a
  socket on its own initiative it picks a close code that tells the client what KIND of
  thing went wrong, so the client can react correctly. A device audio link that is down
  or drops mid-stream is 1011 (internal error): the request was fine, the hardware is
  simply not reachable, and retrying THIS socket will not help until the link returns. A
  /play opening header the server cannot parse is 1003 (unsupported data): the client
  sent something wrong and can fix it and reconnect. Splitting them mirrors the REST
  plane's "4xx you got it wrong / 503 device unreachable" split, so a client faces the
  same two failure kinds whether it is talking HTTP or WebSocket.

  Teardown philosophy: both routes do their real cleanup in a finally that always runs
  -- /mic disarms recording, /play flushes and stops the hail -- and both swallow a
  link-down error during that cleanup. Teardown runs precisely when something already
  ended the session (often the link itself), so raising there would replace the real
  cause with a secondary "could not clean up" error and tell the operator nothing useful.

  Mode gate (development plan section 5.2): both routes are valid only inside an
  interactive mode (func1/func2) and are refused in idle or deter, where the audio path is
  either unused or owned exclusively by the deter loop (invariant R3). The SessionManager on
  app.state is the single authority for that rule: each route calls open_audio(kind) BEFORE
  accepting the handshake, so a refusal (ModeStateError -- wrong mode, or a second client of
  the same kind under R2) closes the socket before accept, which the ASGI server turns into
  an HTTP 403. Only once open_audio admits the session is the handshake accepted, so a client
  never sees an opened-then-dropped socket for a mode refusal. That is deliberately distinct
  from a link-down (1011) or bad-header (1003) failure, which can only be detected AFTER the
  socket is up and so close with a WS code rather than a handshake rejection.

  Cooperative teardown (the tie to the mode machine): open_audio hands back an AudioSession
  carrying a `close` Event that SessionManager sets when a mode transition tears this mode
  down (section 13: with func1 active, POST /mode func2 closes the old /mic session). Each
  route therefore races that `close` Event alongside its normal work, and on either a client
  disconnect or a mode-driven close it ends the stream, disarms its device use, and calls
  finish_audio(sess) in a finally -- which sets the session's `done` Event so the waiting
  transition knows the device activity has actually ceased. finish_audio runs on every exit
  path (including a refused arm or a failed send), so a session is never left registered
  behind a dead socket, which would otherwise wedge that kind's single R2 slot.
"""
from __future__ import annotations

import asyncio
import json
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

# Three import groups, one per layer this plane sits on top of. The codec/ helpers do
# the Opus encode/decode and the 8k<->16k resample -- all the sample-level work that
# device_link deliberately does NOT do. device_link is the transport whose single audio
# socket both routes borrow (they never open one of their own). The 8519 builders turn a
# raw Opus packet into the [42]/[11] hail frames the device expects on the wire. Keeping
# the layers as separate imports is the same "each layer owns exactly its job" split the
# REST plane uses: this module only decides WHEN to encode/decode and WHERE the bytes go.
from ..codec.opus_stream import OpusDecoderStream, OpusEncoderStream
from ..codec.resample import resample_linear
from ..core.device_link import DeviceLink, DeviceLinkError
# SessionManager is the mode/gate authority both routes consult before accepting a socket
# (invariants R2/R3). ModeStateError is the refusal it raises; the two _KIND_* constants are
# session's canonical audio-kind names, imported here (rather than re-spelled as literals) so
# the gate key a route opens with can never drift from the set of kinds session actually
# gates on -- the single-source-of-truth reason those constants exist.
from ..core.session import ModeStateError, SessionManager, _KIND_MIC, _KIND_PLAY
from ..protocol.audio_8519 import build_hail, build_hail_stop

# Namespaced under "payload." like the rest of the service so an operator can raise or
# lower log verbosity for just the WebSocket plane without touching REST or device logs.
logger = logging.getLogger("payload.ws")

# Router mounted by app.py alongside the REST router. Kept module-level so the routes
# register by import, and free of any device handle so importing this module has no side
# effect on the hardware -- the DeviceLink it drives is reached at request time through
# app.state, never constructed or touched at import.
router = APIRouter()

# -- /mic format (development plan section 5.2 and 7) -----------------------
# The device records 16 kHz mono. Downstream consumers (VAD, then ASR) want a steady
# 20 ms cadence, so the client is fed fixed 20 ms frames. The byte geometry is fully
# determined by those two facts: 20 ms at 16 kHz is 320 samples, and s16le is 2 bytes a
# sample, so 640 bytes a frame. The numbers are spelled out as named constants (not
# recomputed at each use) so the JSON header the client reads and the slicer the server
# cuts with are guaranteed to describe the same frame -- they cannot drift apart.
_MIC_FS = 16000
_MIC_FRAME_BYTES = 640
# The one JSON text frame sent before any binary /mic frame, declaring the PCM format so
# the client never has to assume frame geometry. encoding/channels are constant across
# the whole audio plane; sample_rate/frame_ms/frame_bytes are echoed from the constants
# above so a reader of a packet capture can decode the stream from the header alone.
_MIC_HEADER = {
    "encoding": "s16le",
    "sample_rate": _MIC_FS,
    "channels": 1,
    "frame_ms": 20,
    "frame_bytes": _MIC_FRAME_BYTES,
}

# -- /play format -----------------------------------------------------------
# The device's hail input wants 8 kHz Opus in 480-sample (60 ms) frames -- those are the
# rate and frame size the 8519 [42] protocol was verified with on real hardware. Client
# PCM arrives at 16 kHz by default (the natural rate of a mic/ASR pipeline); the client
# MAY instead declare 8 kHz in its opening JSON frame, in which case the resample is a
# no-op. Only those two input rates are accepted: anything else would have to be guessed
# at, and a wrong guess plays the hail back at the wrong pitch and speed.
_HAIL_FS = 8000
_HAIL_FRAME = 480
_PLAY_DEFAULT_FS = 16000
_PLAY_ALLOWED_FS: set[int] = {8000, 16000}

# WebSocket close codes used when the SERVER ends a connection on its own initiative
# (RFC 6455 section 7.4.1). 1011 ("internal error") = the server hit a condition that
# stops it fulfilling the request, which here means the device audio link is down and no
# amount of client retry on this socket will help. 1003 ("unsupported data") = the
# endpoint received data it cannot accept, used for a /play opening header the server
# cannot parse. Distinct codes let the client log or branch on the two failure kinds.
_WS_CLOSE_LINK_DOWN = 1011
_WS_CLOSE_BAD_DATA = 1003


class WsProtocolError(ValueError):
    """Raised when a client's opening /play header cannot be understood.

    House rule bans bare Exception; a dedicated type lets the /play handler catch just
    the header-parse failure (bad JSON, or a sample rate outside the allowed pair) and
    turn it into a clean 1003 close, without also swallowing an unrelated programming
    error that happens to fly through the same try block. Subclassing ValueError keeps
    it semantically a "bad value" while still being catchable as this specific type.
    """


def _link_of(websocket: WebSocket) -> DeviceLink:
    # The single socket owner is stashed on app.state at lifespan startup; both routes
    # reach it the same way. Pulled into a one-line helper so the (slightly indirect)
    # access path is written once and both handlers read identically -- if the attribute
    # ever moves, it moves here and nowhere else.
    return websocket.app.state.device_link


def _session_of(websocket: WebSocket) -> SessionManager:
    # The mode/gate authority, stashed on app.state at lifespan startup alongside the device
    # link. Both audio routes reach it the same way -- to gate the handshake (open_audio) and
    # to register/deregister their AudioSession (finish_audio). A one-line helper for the same
    # reason as _link_of: the indirect access path is written once, so if the attribute ever
    # moves it moves here and nowhere else.
    return websocket.app.state.session


@router.websocket("/mic")
async def ws_mic(websocket: WebSocket) -> None:
    """Stream the device's recorded audio to the client as 20 ms 16 kHz PCM frames.

    Args:
        websocket: the client connection; app.state on it carries the shared DeviceLink.

    Returns:
        None. The coroutine returns when the mode gate refuses the connection (before the
        handshake is even accepted), when the client disconnects, when a send fails because
        the client went away, when a mode transition closes the session, or when recording
        could not be armed at all; in every accepted case the finally below has already
        disarmed the device mic and deregistered the session.

    Raises:
        Exception: a genuine decode or send fault from _pump -- anything other than a
            client WebSocketDisconnect -- is re-raised so FastAPI can close the socket
            and surface it, instead of being silently swallowed during teardown.

    Protocol: accept, send the JSON format header, then binary frames until the client
    disconnects. Recording is armed on the device for exactly the connection's lifetime
    -- [40] on entry, [41] on exit -- so a /mic socket both starts and stops the mic.
    Tying the device mic state to one socket's lifetime (rather than to separate REST
    calls) means there is no way to leave the device recording after the consumer is
    gone: the finally always disarms, whatever ends the session.

    Concurrency model: the device hands raw Opus up through device_link's mic sink on a
    device_link thread; this handler decodes it, re-slices to fixed 640-byte frames, and
    sends them. Three concurrent coroutines run: _pump does queue -> decode -> send,
    _watch_disconnect waits for the client to close, and a waiter on the session's `close`
    Event fires when a mode transition tears this mode down. asyncio.wait(FIRST_COMPLETED)
    ends the session as soon as ANY finishes -- the client closed (watch returns), a send
    failed because it closed (pump raises WebSocketDisconnect), or a /mode switch asked this
    session to close (the close waiter returns). The multi-task shape exists because the
    device may briefly send nothing (a silent room), so waiting only on the queue could sit
    blocked through a disconnect (or a mode change) it never notices; the dedicated watchers
    see both even during total audio silence. The loser tasks are then cancelled and awaited
    so no orphan coroutine keeps touching a closing socket.

    Threading: the sink runs on a device_link thread and does only a thread-safe hand-off
    (call_soon_threadsafe + put_nowait); all decode and send work happens back on the
    loop in _pump. Backpressure: the queue is unbounded on purpose (invariant R2, single
    loopback consumer) -- there is no second reader to starve, and a bounded queue would
    only force a "what to drop" decision that has no good answer for live audio.
    """
    link = _link_of(websocket)
    session = _session_of(websocket)
    # Mode gate BEFORE accept (development plan section 5.2): /mic is valid only in
    # func1/func2. open_audio refuses idle/deter (wrong mode) and a second mic client (R2),
    # and registers the session under the mode lock so the admission cannot race a concurrent
    # transition. Closing before accept makes the ASGI server reject the handshake with an
    # HTTP 403 (the WS code is moot pre-accept), so a mode refusal never opens a socket at all
    # -- unlike the link-down and decode faults below, which can only be reported after the
    # handshake is up.
    try:
        sess = await session.open_audio(_KIND_MIC)
    except ModeStateError as exc:
        logger.warning("mic: refused by mode gate: %s", exc)
        await websocket.close()
        return

    # The session is registered now, so from here EVERY exit path -- a failed accept, a
    # refused arm, a client disconnect, a mode-driven close, or a propagating fault -- must
    # deregister it. The whole body therefore runs under a finally that calls finish_audio,
    # which discards the session and fires its `done` Event; finish_audio is sync and
    # lock-free, so calling it here can never deadlock a teardown that is holding the mode
    # lock and awaiting that same `done`.
    try:
        # ASGI requires accept() to complete the handshake before ANY send; only after it may
        # the format header (and later the binary frames) go out. The gate above already
        # admitted this session, so the accept here is unconditional.
        await websocket.accept()
        # Declare the format up front so the client does not have to assume frame geometry;
        # this is the single text frame, everything after it on this socket is binary PCM.
        await websocket.send_json(_MIC_HEADER)

        # Capture the running loop so the (cross-thread) sink can schedule work back onto it.
        loop = asyncio.get_running_loop()
        # Raw Opus packets from the device cross the thread boundary through this queue. It is
        # intentionally unbounded: under R2 there is a single loopback consumer (the /mic
        # client) draining it as fast as it can decode, so growth is self-limiting, and a
        # bounded queue would only add a "what do we drop" decision that has no right answer
        # for live audio. If multi-client ever lands, that is where backpressure gets designed.
        queue: "asyncio.Queue[bytes]" = asyncio.Queue()

        def sink(opus_packet: bytes) -> None:
            # Runs on a device_link supervisor thread, NOT the event loop. It therefore does
            # the minimum thread-safe thing and hands the packet to the loop: call_soon_
            # threadsafe is the only sanctioned way to poke loop state from another thread,
            # and put_nowait on an unbounded queue cannot block -- both together honour the
            # device_link contract that a mic sink must never block or re-enter the link.
            loop.call_soon_threadsafe(queue.put_nowait, opus_packet)

        try:
            # Arm recording ([40]) on a worker thread: start_recording does a blocking socket
            # write under device_link's send lock, so to_thread keeps that off the loop. If
            # the audio link is down this raises before any frame is sent, and start_recording
            # has already rolled back its own partial arm state.
            await asyncio.to_thread(link.start_recording, sink)
        except DeviceLinkError as exc:
            # Nothing to stream if the mic never armed: tell the client with the link-down
            # code and return. No mic disarm is needed because arming did not take, but the
            # outer finally still deregisters the session on the way out.
            logger.warning("mic: cannot start recording: %s", exc)
            await websocket.close(code=_WS_CLOSE_LINK_DOWN)
            return

        # One decoder for the whole session because Opus decode is stateful (it carries
        # inter-frame prediction); a fresh decoder per packet would corrupt the audio. The
        # rolling PCM buffer is what lets a device Opus frame of any size be recut to 20 ms;
        # it is a bytearray (not bytes) so a sent frame can be dropped with an in-place del
        # instead of rebuilding the whole remainder on every frame.
        decoder = OpusDecoderStream(_MIC_FS)
        pcm_buf = bytearray()

        async def _pump() -> None:
            # Drain the queue forever: decode each packet, buffer the PCM, and emit every
            # whole 20 ms frame that has accumulated. Runs on the loop thread, so it is safe
            # to touch the WebSocket here. Loops on the buffer (not "one frame per packet")
            # because a single decoded Opus packet can yield more or less than one 20 ms frame.
            while True:
                # await queue.get() suspends _pump and yields the loop while the room is
                # silent, so the handler never busy-waits; the sink's call_soon_threadsafe
                # is what re-schedules this coroutine when the next packet actually arrives.
                opus_packet = await queue.get()
                pcm = decoder.decode(opus_packet)
                if not pcm:
                    # A corrupt/undecodable packet comes back empty from the codec layer;
                    # skip it rather than break the whole stream over one lost frame.
                    continue
                # Accumulate this packet's PCM onto whatever sub-frame tail is left over from
                # the previous packet, so frame boundaries stay independent of packet ones.
                pcm_buf.extend(pcm)
                while len(pcm_buf) >= _MIC_FRAME_BYTES:
                    # Send exactly one 20 ms frame, then drop it from the front; the remainder
                    # (a sub-frame tail) stays buffered for the next packet to complete. The
                    # bytes(...) wrap makes an immutable snapshot of the slice: send_bytes may
                    # keep a reference past this line, and the very next del mutates pcm_buf in
                    # place, so handing over the raw bytearray slice could let that mutation
                    # race the send. The copy is one 640-byte frame, so it is negligibly cheap.
                    await websocket.send_bytes(bytes(pcm_buf[:_MIC_FRAME_BYTES]))
                    del pcm_buf[:_MIC_FRAME_BYTES]

        async def _watch_disconnect() -> None:
            # Return as soon as the client goes away. At this low level receive() yields a
            # disconnect message rather than raising, so this coroutine unblocks on the
            # disconnect even while _pump is parked waiting on a silent queue -- which is the
            # whole reason it exists as a separate task instead of folding into _pump.
            while True:
                message = await websocket.receive()
                if message["type"] == "websocket.disconnect":
                    return

        # Starlette forbids two coroutines calling receive() (or send) concurrently on one
        # socket. That invariant holds here by construction: _pump is the ONLY sender and
        # _watch_disconnect is the ONLY receiver, so the two run concurrently yet never touch
        # the same direction at once. The close-event waiter below touches neither direction.
        try:
            pump_task = asyncio.create_task(_pump())
            watch_task = asyncio.create_task(_watch_disconnect())
            # Third waiter: the session's `close` Event, set by SessionManager when a mode
            # transition tears this mode down (section 13). Racing it here means a /mode
            # switch ends this /mic socket promptly, the same way a client disconnect does;
            # Event.wait() returns True and never raises, so it is a clean loser or winner.
            close_task = asyncio.create_task(sess.close.wait())
            # Wait for the FIRST of the three to finish -- the session-ending event: the
            # client closed (watch returns), a send failed because it closed (pump raises
            # WebSocketDisconnect), or a mode switch asked this session to close (close_task
            # returns). There is no normal case where more than one runs to completion.
            done, pending = await asyncio.wait(
                {pump_task, watch_task, close_task}, return_when=asyncio.FIRST_COMPLETED
            )
            # Cancel and await the losers so no orphan task keeps running past this request
            # and keeps touching a closing socket. Awaiting the cancel is required, not
            # optional: a cancelled-but-never-awaited task logs "task was destroyed but it is
            # pending" noise, so the CancelledError is awaited and swallowed here.
            for task in pending:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            # A clean client disconnect surfaces as WebSocketDisconnect (or a plain return)
            # and is the expected way this ends, so it is swallowed; a close_task win returns
            # True (no exception). Anything else is a real fault (a decode or send bug) that
            # must propagate rather than be lost in teardown.
            for task in done:
                error = task.exception()
                if error is not None and not isinstance(error, WebSocketDisconnect):
                    raise error
        finally:
            # Disarm recording ([41]) no matter how the session ended, so the device is never
            # left recording with no consumer. stop_recording swallows a link-down error
            # itself (there is nothing to stop on a dead link), so this line never raises, and
            # the to_thread keeps its blocking [41] write off the loop just like the arm did.
            # This runs BEFORE the outer finally's finish_audio, so the device mic is already
            # disarmed by the time a waiting transition is released to arm the new mode.
            await asyncio.to_thread(link.stop_recording)
    finally:
        # Deregister the session and fire its `done` Event on every exit path, so no dead
        # socket is ever left occupying this kind's single R2 slot and any transition awaiting
        # this session's teardown is released. Idempotent, so overlapping with an earlier
        # return path is harmless.
        session.finish_audio(sess)


@router.websocket("/play")
async def ws_play(websocket: WebSocket) -> None:
    """Stream client PCM out of the device as a realtime [42] hail.

    Args:
        websocket: the client connection; app.state on it carries the shared DeviceLink.

    Returns:
        None. The coroutine returns when the mode gate refuses the connection (before the
        handshake is accepted), when the client disconnects, when a mode transition closes
        the session, when the audio link drops mid-stream, or when the opening header cannot
        be parsed; in every accepted case the finally has flushed the encoder's tail and sent
        [11], then deregistered the session, before it returns (on the bad-header path no
        audio was ever buffered, so the flush emits nothing).

    Protocol: gate then accept; the client MAY send one opening JSON text frame declaring its
    sample rate (8000 or 16000), after which every frame is binary s16le PCM. Each PCM chunk
    is resampled to the device's 8 kHz, encoded to 480-sample Opus, and sent as a [42] frame.
    On disconnect the encoder's final partial frame is flushed as one last [42] and then [11]
    is sent, so the device ends the hail in a known idle state rather than staying parked on a
    stream that merely stopped arriving.

    Concurrency model: the receive/encode loop runs as _pump, raced by asyncio.wait against a
    waiter on the session's `close` Event. Whichever finishes first ends the session -- a
    client disconnect, a bad header, or a link drop ends _pump, while a mode transition sets
    `close`; the loser is then cancelled and awaited so no orphan task touches the closing
    socket.

    Buffering encoder: chunk sizes from the client are arbitrary (section 5.2) -- a client
    may push 100 ms blocks or 7 ms blocks. That is exactly why the encoder here is the
    buffering OpusEncoderStream: it absorbs any-length input and emits only whole
    480-sample frames, so odd-sized client buffers still produce valid 60 ms hail packets
    with no clicks at the seams and no partial frame sent mid-stream. One encoder serves
    the whole connection so a sub-frame tail carries across chunk boundaries.

    Header window: only the FIRST frame seen is treated as the optional format header.
    Any binary frame, or a second text frame, closes that window so the input rate cannot
    change mid-stream -- switching rates would desynchronise the stateful encoder. A
    header that fails to parse is a client mistake (1003 close), distinct from a link
    that drops (1011 close); either way the finally still runs.
    """
    link = _link_of(websocket)
    session = _session_of(websocket)
    # Mode gate BEFORE accept, mirroring /mic (development plan section 5.2): /play is valid
    # only in func1/func2, and a second play client is refused under R2. A refusal closes
    # before accept, which the ASGI server turns into an HTTP 403 (the WS code is moot
    # pre-accept); only an admitted session goes on to accept the handshake below.
    try:
        sess = await session.open_audio(_KIND_PLAY)
    except ModeStateError as exc:
        logger.warning("play: refused by mode gate: %s", exc)
        await websocket.close()
        return

    # One encoder for the whole stream so leftover sub-frame PCM carries across chunks (Opus
    # encode is stateful, same as decode) and no frame boundary is ever fabricated. It lives
    # in the handler scope, not inside _pump, because the finally must flush it after the pump
    # has ended -- _pump only reads it, so a closure read suffices there.
    encoder = OpusEncoderStream(_HAIL_FS, _HAIL_FRAME)

    async def _pump() -> None:
        # The receive/encode loop, run as a task so the handler can race it against the
        # session's `close` Event: a mode transition then ends /play the same way a client
        # disconnect does. in_fs and the header-window flag are locals here because nothing
        # outside this loop reads them.
        #
        # Input sample rate: 16 kHz until/unless the opening JSON header declares 8 kHz. The
        # default is 16 kHz because that is the natural rate of the mic/ASR pipeline feeding
        # /play; a client already producing 8 kHz declares it and skips the resample entirely.
        in_fs = _PLAY_DEFAULT_FS
        # The format header is honoured only as the FIRST frame seen. Any binary frame, or a
        # second text frame, flips this true; later text frames are then treated as noise
        # rather than allowed to change the rate mid-stream (which would break the encoder).
        header_seen = False
        while True:
            # receive() yields an ASGI event dict: {"type": ...} plus either "text" or
            # "bytes" depending on the frame kind. We branch on which key is present,
            # reading with .get so the absent key is simply None, not a KeyError.
            message = await websocket.receive()
            if message["type"] == "websocket.disconnect":
                # Client closed: end the pump normally so the handler's finally can flush.
                return
            # Binary PCM is the norm on /play; a text frame is the exception, carrying at
            # most the one optional format header. .get returns None for whichever of
            # text/bytes this frame is not, cleanly distinguishing the two frame kinds.
            text = message.get("text")
            if text is not None:
                # A text frame: treat only the very first one as the optional format
                # header. A stray text frame later in the stream is ignored on purpose.
                if not header_seen:
                    try:
                        in_fs = _parse_play_sample_rate(text)
                    except WsProtocolError as exc:
                        # An unparseable header is a client mistake, not a link fault:
                        # close with the bad-data code so the client can fix and retry.
                        logger.warning("play: bad header: %s", exc)
                        await websocket.close(code=_WS_CLOSE_BAD_DATA)
                        return
                header_seen = True
                continue
            # Any binary frame also closes the header window: once audio has started the
            # encoder is mid-stream and stateful, so a late header trying to switch rates
            # could not be honoured without a click and is refused implicitly from here on.
            header_seen = True
            data = message.get("bytes")
            if not data:
                # An empty binary frame carries no audio (some clients send keep-alives
                # this way); ignore it rather than feed the codec a zero-length chunk.
                continue
            # Resample to the device rate BEFORE encoding (a no-op if the client already
            # declared 8 kHz). Order matters: the Opus encoder is configured for 8 kHz and
            # counts a frame in samples, so feeding it 16 kHz PCM would treat each frame as
            # half its true duration and speak too fast. Only whole frames come back from
            # encode(); the remainder stays buffered in the encoder for the next chunk.
            pcm_8k = resample_linear(data, in_fs, _HAIL_FS)
            packets = encoder.encode(pcm_8k)
            if not packets:
                # This chunk did not complete a 480-sample frame yet; nothing to send, so
                # wait for more input rather than emitting a short (clicky) partial frame.
                continue
            try:
                # Send all whole frames from this chunk as [42] hails in one borrowed-
                # socket batch. to_thread keeps the blocking socket write off the loop,
                # and device_link serialises the batch under its own send lock so no
                # other writer can interleave between these frames.
                await asyncio.to_thread(
                    link.send_audio, [build_hail(packet) for packet in packets]
                )
            except DeviceLinkError as exc:
                # Audio link went down mid-stream: nothing more can be sent on it, so end
                # the session with the link-down code instead of spinning on failures.
                logger.warning("play: audio send failed: %s", exc)
                await websocket.close(code=_WS_CLOSE_LINK_DOWN)
                return

    try:
        # Complete the handshake before anything else (ASGI rule). Unlike /mic there is no
        # header to send back, because /play only ever receives from the client.
        await websocket.accept()
        # Race the receive/encode loop against the session's `close` Event so a mode
        # transition (section 13) tears this /play socket down promptly, just like a client
        # disconnect. Event.wait() returns True and never raises, so close_task is a clean
        # winner or loser.
        pump_task = asyncio.create_task(_pump())
        close_task = asyncio.create_task(sess.close.wait())
        # Wait for whichever ends first: _pump (client disconnect, bad header, or link drop)
        # or close_task (a mode transition). There is no normal case where both complete.
        done, pending = await asyncio.wait(
            {pump_task, close_task}, return_when=asyncio.FIRST_COMPLETED
        )
        # Cancel and await the loser so no orphan task keeps touching a closing socket; the
        # CancelledError from awaiting a cancelled task is expected and swallowed.
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        # A clean disconnect (or a bad-header/link-down close) ends _pump normally, and a
        # close_task win returns True -- neither is an error. Anything else is a real fault
        # that must propagate rather than be lost in teardown.
        for task in done:
            error = task.exception()
            if error is not None and not isinstance(error, WebSocketDisconnect):
                raise error
    finally:
        # Flush the trailing partial frame and stop the hail, whatever ended the session --
        # a clean disconnect, a mode-driven close, a link drop, or an error -- so the device
        # never keeps playing the last buffered audio after the client stopped feeding it.
        # _finish_play runs BEFORE finish_audio, so the hail is quiesced ([11] sent) before a
        # transition awaiting this session is released to arm the new mode; then finish_audio
        # deregisters the session and fires its `done` Event.
        await _finish_play(link, encoder)
        session.finish_audio(sess)


async def _finish_play(link: DeviceLink, encoder: OpusEncoderStream) -> None:
    """Flush the encoder's final partial frame, then send [11] to end the hail.

    Args:
        link: the device link to write the final frames on.
        encoder: the /play session encoder, possibly holding a sub-frame tail.

    Returns:
        None. This is a fire-and-forget teardown helper; a link-down failure here is
        logged and swallowed rather than raised (see below).

    The device has no "hail finished" signal, so if the stream just stops it keeps the
    speaker parked on the last frame it received. Sending [11] (build_hail_stop) is what
    actually returns it to idle, so the trailing flush + [11] is the clean end of a hail.
    A link-down error here is swallowed: if the audio link is already gone there is
    nothing to flush to, and this runs on the teardown path where re-raising would only
    mask the real reason the session ended (see the module's teardown philosophy note).
    """
    try:
        # flush() zero-pads and emits any buffered sub-frame so no captured audio is
        # dropped at the tail. Order matters: the flushed audio frames are appended BEFORE
        # [11], so the device plays the final samples and only then returns to idle --
        # sending [11] first would clip the tail of the hail.
        frames = [build_hail(packet) for packet in encoder.flush()]
        frames.append(build_hail_stop())
        await asyncio.to_thread(link.send_audio, frames)
    except DeviceLinkError as exc:
        logger.warning("play: could not flush/stop hail: %s", exc)


def _parse_play_sample_rate(text: str) -> int:
    """Parse the optional /play opening header and return its declared input rate.

    Args:
        text: the JSON text of the client's first frame.

    Returns:
        The declared sample rate (8000 or 16000), or the 16000 default when the header
        is a valid object that omits sample_rate.

    Raises:
        WsProtocolError: if the text is not a JSON object, or declares a sample rate
            outside the allowed pair -- both are client mistakes the caller turns into a
            clean close rather than guessing a rate and speaking at the wrong pitch.

    Kept as a pure, side-effect-free function (no socket, no device) so the header
    contract can be unit-tested on its own, without standing up a WebSocket or a link.
    """
    try:
        obj = json.loads(text)
    except (json.JSONDecodeError, ValueError) as exc:
        # json.loads raises JSONDecodeError (itself a ValueError subclass); catching both
        # names is belt-and-suspenders so the failure is always reported as our typed
        # error naming the underlying cause, never as a raw json exception.
        raise WsProtocolError(f"play header is not valid JSON: {exc}") from exc
    if not isinstance(obj, dict):
        # A bare JSON scalar or array is valid JSON but not a header object; reject it so
        # the obj.get below cannot fail obscurely on, say, a list or an int.
        raise WsProtocolError("play header must be a JSON object")
    # Absent sample_rate is allowed and means "use the 16 kHz default"; a PRESENT but
    # unsupported rate is an error, because silently defaulting it would play the hail at
    # the wrong pitch -- the one failure this whole check exists to prevent.
    sample_rate = obj.get("sample_rate", _PLAY_DEFAULT_FS)
    if sample_rate not in _PLAY_ALLOWED_FS:
        raise WsProtocolError(
            f"sample_rate must be one of {sorted(_PLAY_ALLOWED_FS)}, got {sample_rate!r}"
        )
    # The value already cleared the membership check above, so it is one of the two ints;
    # int() only narrows the declared return type and never actually converts here.
    return int(sample_rate)
