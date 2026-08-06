"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: rest.py
Brief: payload-service REST control plane (GET /status; POST /lights /tts /mode /deter).

Description:
  Defines the HTTP control-plane routes of payload-service as described in the
  development plan section 5.1. Milestone M1 shipped GET /status, which reports
  per-link connectivity and the latest parsed lights state; milestone M2 added
  POST /lights, which drives the searchlight and red/blue warning light; milestone M3
  added POST /tts, which makes the device speak a line and returns a playback-time
  estimate; milestone M4 (group A) adds POST /mode -- the mode state machine that
  enforces R2/R3 -- and POST /deter, which arms and disarms the 驱离 deterrent, and
  gates POST /tts behind the mode. This is the boundary the rest of the test system
  talks to: instead of every module knowing the 8519/8529 wire protocol, they call this
  loopback HTTP surface and let payload-service own the hardware conversation.

  Scope discipline (house V1/V2 rule): the one remaining control route -- /volume -- is
  deliberately NOT stubbed here. It arrives in the milestone that first implements its
  behaviour, so this file never carries an empty interface that merely reserves a URL for
  the future. An unimplemented route that 404s is more honest than a stub that 200s with
  a fabricated body, because a caller can detect the former and cannot detect the latter.
  POST /mode and POST /deter appear now precisely because M4 implements their behaviour
  end to end (the streaming /mic and /play audio routes live in api/ws.py, since they are
  WebSocket, not REST); they are not placeholders.

  Mode gate (invariant R3): POST /mode drives the SessionManager on app.state, which owns
  the single active mode and tears the old mode's audio/deter down on every switch. The
  other audio-touching routes consult it before acting: POST /tts is refused with 409
  unless the mode allows a one-shot spoken line, and POST /deter is refused with 409
  unless the service is in DETER. A re-entry of the active mode is also a 409. These 409s
  say "your request conflicts with the current mode", distinct from a 4xx bad request or
  a 503 unreachable device.

  Ownership boundary (invariant R1): these handlers contain no device logic and hold
  no socket. GET /status reads the DeviceLink that app.py stashes on app.state and
  returns its cached snapshot; POST /lights encodes control frames and hands the
  blocking socket write to DeviceLink.send_lights_control via asyncio.to_thread. Either
  way all socket ownership stays inside core.device_link, and the coroutine itself
  never blocks the event loop -- the read path only touches memory, and the write path
  offloads the blocking send to a worker thread rather than doing it inline. That
  offload is exactly the "hand blocking device I/O to a thread" discipline this module
  requires of any route that must talk to the hardware.

  Wire-model split (why the pydantic classes below mirror, but do not reuse, the
  internal types): the REST response is a published contract with external clients,
  whereas LightsStatus is an internal decode structure. Keeping them as separate
  classes means an internal field rename or a protocol-layer change cannot silently
  alter the JSON a client already depends on -- the mapping between them is written
  out explicitly in the handler, so any divergence is a compile-visible edit here.

  Error taxonomy for POST /lights (three distinct failure codes, each telling the
  client a different thing): a malformed or out-of-range field is rejected by pydantic
  as 422 at the boundary, before any frame is built; a syntactically valid but EMPTY
  command (no field set) is a 400, because it would change nothing yet a success would
  hide the caller's near-certain mistake; and a well-formed command that cannot be
  delivered -- the lights link is down, or it breaks mid-send -- is a 503. Splitting
  these lets a client distinguish "I sent something wrong" (4xx, fix the request) from
  "the device is not reachable right now" (503, retry later), which are different
  reactions. Note the 422 and the encoders' own range check are redundant on purpose:
  they share the MIN/MAX constants, so a value that clears pydantic can never fail the
  builder, and the builder's LightsProtocolError is therefore never expected on this
  path (it would signal a bounds-constant mismatch bug, not ordinary bad input).
"""
from __future__ import annotations

import asyncio
from typing import List, Optional

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

from ..config import PayloadConfig
from ..core.device_link import DeviceLink, DeviceLinkError

# POST /deter turns an HTTP body into one deter run: the DeterParams DTO carries the
# body's tunables into SessionManager, which builds and drives the controller.
from ..core.deter import DeterParams

# POST /mode and the /tts and /deter gates all go through the SessionManager on app.state.
# Mode is imported so the request/response bodies are typed by the enum (pydantic then
# validates the four values and rejects any other string with a 422); the two mode-fault
# types are imported so this layer can translate a state conflict into an HTTP 409.
from ..core.session import (
    Mode,
    ModeStateError,
    ModeTransitionError,
    SessionManager,
)

# POST /tts turns an HTTP request into a [31] audio frame, so it imports the 8519 TTS
# builder and its voice-selector constants -- the same "REST decides WHICH frame, the
# protocol layer decides HOW to encode it" split used for lights. The VOICE_MALE/FEMALE
# constants source the request's voice range so the Field bound and the builder's own
# check validate against one shared pair of values, not two copied literals.
from ..protocol.audio_8519 import (
    VOICE_FEMALE,
    VOICE_MALE,
    build_tts,
)

# POST /lights is where an HTTP command becomes wire bytes, so this module imports the
# 8529 builders and their value bounds directly. Importing the encoders (rather than
# re-implementing frame layout here) keeps protocol/lights_8529.py as the single place
# frame structure and CRC coverage live; the REST layer only decides WHICH frames a
# request implies, never HOW a frame is byte-encoded. The MIN/MAX constants are pulled
# in so the Field range checks below and the encoders validate against one shared
# source of truth instead of two hand-copied numeric literals that could drift apart.
from ..protocol.lights_8529 import (
    BRIGHT_MAX,
    BRIGHT_MIN,
    REDBLUE_MAX,
    REDBLUE_MIN,
    build_brightness,
    build_redblue,
    build_searchlight,
    build_strobe,
)

# Router that app.py mounts onto the FastAPI application. Kept module-level so the
# routes register by import, and free of any device handle so importing this module
# has no side effect on the device.
router = APIRouter()


class DeviceFlags(BaseModel):
    """Per-link connectivity block of the /status response.

    Two separate booleans (rather than one "connected") so a caller can tell which
    physical link is down -- the audio (8519) and lights (8529) sockets fail
    independently, and diagnosing the device benefits from knowing which one. A single
    rolled-up flag would hide the common real-world case where one cable or port is
    fine and the other is not, forcing an operator to guess which half to check.
    """

    audio_connected: bool
    lights_connected: bool


class LightsView(BaseModel):
    """Lights state block of the /status response, decoded from the 0x25 report.

    searchlight is on/off only; brightness is a SEPARATE field rather than folded
    into searchlight, because the device retains the brightness value even while the
    lamp is off (development plan section 9). Modelling them separately lets a client
    show "off, but remembers 30" faithfully. Folding brightness into the on/off state
    (e.g. reporting 0 when off) would destroy that retained value and mislead any UI
    that shows the level the lamp will resume at. redblue is 0 (off) or 1..16 (a
    built-in flash pattern), passed through verbatim from the decoded status so the
    sixteen patterns remain individually addressable by a client.
    """

    searchlight: bool
    bright: int
    redblue: int


class StatusResponse(BaseModel):
    """Body of GET /status (development plan section 5.1).

    volume and temperature are Optional and default to None because THIS three-in-one
    unit has no [99] auto-report, so those readings are genuinely unknown; they must
    serialize as JSON null rather than a fabricated 0 that a client could mistake for
    a real measurement (development plan section 9). The distinction matters: 0 is a
    legitimate volume, so emitting 0 for "unknown" would be indistinguishable from a
    real silent setting, whereas null is unambiguously "no reading available on this
    hardware." lights is Optional for a different reason: no valid 0x25 status may have
    arrived yet in the brief window just after startup, and null communicates "not
    known yet" honestly rather than inventing a default lamp state. mode is the live
    session mode as its wire string ("idle"/"func1"/"func2"/"deter"): kept a plain str
    (not the Mode enum) so the published /status contract is a stable string that cannot
    break a client if the internal enum is ever renamed, mirroring the wire-model split
    used for the lights view.
    """

    mode: str
    device: DeviceFlags
    volume: Optional[int] = None
    temperature: Optional[float] = None
    lights: Optional[LightsView] = None


@router.get("/status", response_model=StatusResponse)
async def get_status(request: Request) -> StatusResponse:
    """Report device connectivity and the latest cached lights state.

    Args:
        request: the incoming request, used only to reach app.state.device_link. The
            handler takes the Request purely as the documented FastAPI way to reach
            app.state; it reads no headers, query, or body, because /status is a pure
            GET with no inputs.

    Returns:
        A StatusResponse assembled entirely from cached state; this handler never
        performs device I/O.

    The snapshot is read exactly once and then mapped, so the connectivity flags and
    the lights view all describe the same instant rather than being sampled at
    slightly different times across the response. The None-vs-populated branch for
    lights is written as an explicit conditional rather than a clever expression so
    the "no status yet -> null" path is obvious to a future reader and cannot be lost
    in a refactor. Because every value returned here comes from memory the coroutine
    never awaits or blocks, which is what keeps it safe to run directly on the event
    loop under the R1 no-socket-in-the-handler rule.
    """
    # Reach the shared DeviceLink placed on app.state during lifespan startup. The
    # handler only ever reads its cached snapshot and connectivity flags.
    link: DeviceLink = request.app.state.device_link
    # Reach the session manager for the live mode (a lock-free point-in-time read).
    session: SessionManager = request.app.state.session
    # Pull the latest lights snapshot once. It may be None (no status seen yet).
    snapshot = link.snapshot_lights()
    # Map the internal LightsStatus onto the wire model, or leave it null when there
    # is no snapshot. Kept as an explicit conditional so the null case is obvious.
    lights = (
        None
        if snapshot is None
        else LightsView(
            # Field names are translated on purpose: the internal decode type uses
            # searchlight_on / redblue_mode, while the wire model uses the shorter
            # searchlight / redblue. Doing the rename here, in one visible place, is
            # what lets the two evolve independently (see the wire-model split note in
            # the module docstring). strobe is intentionally not surfaced in M1.
            searchlight=snapshot.searchlight_on,
            bright=snapshot.bright,
            redblue=snapshot.redblue_mode,
        )
    )
    return StatusResponse(
        # The live mode the SessionManager currently enforces (development plan section 6),
        # as its wire string. This is now a real value driven by POST /mode, not a
        # baseline: idle / func1 / func2 / deter.
        mode=session.current_mode().value,
        # Connectivity is read here from the two atomic Events on the link. They are
        # lock-free point-in-time reads, so each simply reflects whether that socket
        # was connected at the moment the response was built (see DeviceLink's
        # audio_connected / lights_connected properties for the atomicity contract).
        device=DeviceFlags(
            audio_connected=link.audio_connected,
            lights_connected=link.lights_connected,
        ),
        # Null on this unit: it emits no [99] volume/temperature auto-report, so
        # there is nothing truthful to put here (development plan section 9).
        volume=None,
        temperature=None,
        lights=lights,
    )


class LightsCommand(BaseModel):
    """Body of POST /lights: a PARTIAL lights command (development plan section 5.1).

    Every field is Optional and defaults to None, and None means "leave this aspect of
    the lamp unchanged". That is the whole semantic of the route: a client sends only
    the fields it wants to alter, and the handler emits exactly one 8529 control frame
    per present field. Modelling absence as None (rather than, say, a sentinel value)
    is what lets "turn the searchlight on but do not touch brightness" be expressed
    without the client having to know or restate the current brightness.

    The numeric ranges are enforced HERE at the HTTP boundary via Field ge/le, sourced
    from the same BRIGHT_MIN/MAX and REDBLUE_MIN/MAX constants the encoders use, so an
    out-of-range value is rejected with a 422 before any frame is built. The builders
    re-check the identical bounds, but validating at the edge gives the client a clear
    422 instead of surfacing as a 400/500 deeper in; the two layers agree because they
    read one shared source of truth, not two hand-copied literals.
    """

    # None => do not change the searchlight on/off state.
    searchlight: Optional[bool] = None
    # None => do not change brightness; otherwise BRIGHT_MIN..BRIGHT_MAX inclusive.
    bright: Optional[int] = Field(default=None, ge=BRIGHT_MIN, le=BRIGHT_MAX)
    # None => do not change the strobe on/off state.
    #
    # ** THIS IS THE SEARCHLIGHT'S OWN STROBE (8529 0x03), AND THE PROJECT DOES NOT USE IT.
    # 00 PAY-02c settles the word 爆闪 for the whole requirement set: it means the red/blue
    # warning lamp, MSG_REDBLUE below, and NOT this. The field exists because the device
    # has the command and this service is the protocol's complete surface, not because
    # anything upstream should call it.
    #
    # Reading 爆闪 as this field is a specific, already-diagnosed mistake (00 VOI-43): in D
    # mode the searchlight is required to be STEADY at full brightness while the red/blue
    # flashes, so implementing "开爆闪" here makes exactly the wrong lamp flash -- and the
    # searchlight going dark half the time is what night perception depends on it not
    # doing (PER-42). The failure is silent: the robot still looks like it is alarming.
    strobe: Optional[bool] = None
    # None => do not change the red/blue mode; otherwise REDBLUE_MIN..REDBLUE_MAX.
    #
    # * This is what 爆闪 means everywhere in the requirements (00 PAY-02c). The value is a
    # PATTERN SELECTOR, not a brightness and not a rate: 0 is off and 1..16 choose among
    # the firmware's sixteen built-in patterns. The device exposes no red/blue brightness
    # and no flash-rate parameter at all -- 14 GL-4d records that the payload is a single
    # mode byte -- so a caller wanting "faster flashing" must pick a different pattern.
    # ! And it must NOT be synthesised by toggling from the host: 14 GL-4a rules that out
    # because P2 reaches this lamp only through POST /lights, whose 300 ms timeout is an
    # order of magnitude longer than the 70-90 ms pulses a double-flash needs.
    redblue: Optional[int] = Field(default=None, ge=REDBLUE_MIN, le=REDBLUE_MAX)


class LightsAck(BaseModel):
    """Body of a successful POST /lights: a bare {"ok": true} acknowledgement.

    The route has no meaningful data to return -- it is a command, not a query -- so
    the ack carries a single boolean rather than echoing the applied state. Echoing
    state would be misleading: the authoritative lamp state is whatever the next 0x25
    status report says, which GET /status already exposes, so re-deriving it here from
    the request could disagree with the hardware. A minimal ack keeps the one source of
    truth for lamp state on the status path.
    """

    ok: bool


@router.post("/lights", response_model=LightsAck)
async def post_lights(command: LightsCommand, request: Request) -> LightsAck:
    """Apply a partial lights command by sending one 8529 control frame per field.

    Args:
        command: the partial command; only its non-None fields produce a frame.
        request: the incoming request, used only to reach app.state.device_link.

    Returns:
        LightsAck(ok=True) once every built frame has been handed to the device.

    Raises:
        HTTPException(400): if the body sets no field at all, since an empty command
            would send nothing yet report success -- a client almost certainly meant to
            change something, so this fails loud rather than lying with ok=true.
        HTTPException(503): if the lights link is down, or the send fails because the
            link broke mid-write; DeviceLinkError from the socket owner is mapped to
            Service Unavailable so the client can distinguish "device not reachable"
            from a request it got wrong (which would be a 4xx).

    Frames are built in a fixed field order and passed to send_lights_control as one
    group, so DeviceLink writes them back to back under its send lock and no other
    request can interleave between them. The blocking socket write is dispatched with
    asyncio.to_thread so the event loop stays free while DeviceLink does the I/O -- the
    handler thus performs device I/O without ever owning a socket or blocking the loop,
    which is how invariant R1 and the no-blocking-on-the-loop rule are both kept.

    Why to_thread rather than an async socket: send_lights_control is a synchronous,
    blocking call by design -- DeviceLink is a plain threaded class that must stay
    usable with no event loop present (from a unit test, or a future non-async caller).
    Bridging it with asyncio.to_thread runs that blocking call on the default worker
    pool and awaits the result, so the offload lives here at the async boundary and the
    socket owner never has to grow an event loop of its own or couple itself to FastAPI.
    """
    # Reach the single socket owner placed on app.state during lifespan startup.
    link: DeviceLink = request.app.state.device_link
    # Encode one control frame per field the client actually set. A field left as None
    # is deliberately skipped, which is what makes absent fields leave the lamp
    # unchanged. Field order here (searchlight, bright, strobe, redblue) is fixed and
    # matches the control message-id order; the aspects are independent device
    # registers, so the order among them is not otherwise significant.
    frames: List[bytes] = []
    if command.searchlight is not None:
        frames.append(build_searchlight(command.searchlight))
    if command.bright is not None:
        frames.append(build_brightness(command.bright))
    if command.strobe is not None:
        frames.append(build_strobe(command.strobe))
    if command.redblue is not None:
        frames.append(build_redblue(command.redblue))
    # An empty command is treated as a client error rather than a silent no-op: it
    # almost always means the caller forgot to set a field, and answering ok=true would
    # hide that mistake behind a success. This mirrors the module's "honest failure
    # beats a fabricated 200" stance.
    if not frames:
        raise HTTPException(status_code=400, detail="no lights field provided")
    try:
        # Offload the blocking socket write to a worker thread so the event loop is
        # never blocked; DeviceLink serialises the send under its own lock.
        await asyncio.to_thread(link.send_lights_control, frames)
    except DeviceLinkError as exc:
        # Link down or the write failed: the device is not reachable right now, which
        # is a 503 (Service Unavailable), not a client error.
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    # Every frame was accepted by the kernel send buffer; acknowledge success. The
    # authoritative post-command lamp state is the next 0x25 report exposed by
    # GET /status, so this ack deliberately carries no echoed state to disagree with.
    return LightsAck(ok=True)


class TtsCommand(BaseModel):
    """Body of POST /tts: which voice speaks what text (development plan section 5.1).

    Both fields are required -- unlike the partial /lights command there is no "leave
    unchanged" here, because speaking is a one-shot action, not a set of independently
    toggled registers. voice selects the device's built-in male/female TTS voice and is
    range-checked at the boundary against the same VOICE_MALE/VOICE_FEMALE the [31]
    builder uses, so an out-of-range selector is a 422 before any frame is built. text
    is constrained non-empty (min_length=1) so an empty utterance is rejected at the
    edge rather than reaching the builder, which would refuse it anyway -- the two checks
    agree on purpose. There is deliberately NO "loop" field: the [31] TTS frame has no
    repeat concept in the protocol, so inventing one here would be a stub for behaviour
    the device does not implement (house V1/V2 discipline).
    """

    # 0 = male, 1 = female; the device's first [31] payload byte.
    voice: int = Field(ge=VOICE_MALE, le=VOICE_FEMALE)
    # The sentence to speak; must be non-empty (an empty utterance is a caller mistake).
    text: str = Field(min_length=1)


class TtsAck(BaseModel):
    """Body of a successful POST /tts: {"ok": true, "est_ms": int}.

    est_ms is the estimated playback duration and is the whole point of the response.
    The device emits no "TTS finished" event (development plan section 9), so the caller
    (AI_runtime) cannot wait for completion; it instead waits est_ms plus its own margin
    before reopening the mic gate. Returning the estimate here is what lets the caller
    time that gate without a completion signal from the hardware.
    """

    ok: bool
    est_ms: int


@router.post("/tts", response_model=TtsAck)
async def post_tts(command: TtsCommand, request: Request) -> TtsAck:
    """Make the device speak text, and return the estimated playback duration.

    Args:
        command: the voice selector and the text to speak.
        request: the incoming request, used only to reach app.state (the device link
            and the config).

    Returns:
        TtsAck(ok=True, est_ms=...) once the [31] frame has been handed to the device,
        where est_ms is config.estimate_tts_ms(text).

    Raises:
        HTTPException(409): if the current mode does not permit a one-shot spoken line
            (allowed only in IDLE and FUNC1). In FUNC2 a live intercom must not be
            interrupted, and in DETER the deter loop owns the audio path, so an external
            /tts there is a state conflict the caller must resolve by changing mode first.
        HTTPException(503): if the audio link is down, or the send fails because the
            link broke mid-write; DeviceLinkError from the socket owner is mapped to
            Service Unavailable so the client can tell "device not reachable" from a
            request it got wrong (a 4xx, which pydantic already returns for a bad voice
            or empty text before this handler even runs).

    The shape mirrors POST /lights exactly: build the wire frame from the validated
    request, then offload the blocking socket write to a worker thread with
    asyncio.to_thread so the event loop is never blocked and this handler never owns a
    socket (invariant R1). est_ms is a pure function of the text and the config knobs, so
    it is computed with no I/O; it is computed only after a successful send so a 503
    carries no misleading duration for audio the device never received.
    """
    # Reach the three things stashed on app.state during lifespan startup: the single
    # socket owner (for the send), the config (for the estimate formula), and the session
    # (for the mode gate).
    link: DeviceLink = request.app.state.device_link
    config: PayloadConfig = request.app.state.config
    session: SessionManager = request.app.state.session
    # Mode gate (R3): a one-shot TTS is allowed only where allows_tts() says so. This is a
    # lock-free check; a one-tick race with a concurrent mode flip is benign for a
    # fire-and-forget frame (the new mode's teardown does not need to reap it).
    if not session.allows_tts():
        raise HTTPException(
            status_code=409,
            detail=f"tts not allowed in mode {session.current_mode().value}",
        )
    # Encode the [31] TTS frame. The builder re-validates voice/text; that is redundant
    # with the Field checks above (which already 422'd bad input), so on this path it is
    # never expected to raise -- a raise would signal a bounds mismatch bug, not input.
    frame = build_tts(command.voice, command.text)
    try:
        # Offload the blocking socket write; DeviceLink serialises it under its send lock.
        await asyncio.to_thread(link.send_audio, [frame])
    except DeviceLinkError as exc:
        # Audio link down or the write failed: the device is not reachable right now,
        # which is a 503 (Service Unavailable), not a client error.
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    # The frame is on its way; return the playback estimate the caller needs to time its
    # mic gate. Computed here (post-send) so a failed send never returns a stale est_ms.
    return TtsAck(ok=True, est_ms=config.estimate_tts_ms(command.text))


class ModeCommand(BaseModel):
    """Body of POST /mode: the mode to switch to (development plan section 5.1).

    The single field is typed as the Mode enum, so pydantic accepts only the four wire
    strings ("idle"/"func1"/"func2"/"deter") and rejects anything else with a 422 before
    the handler runs -- the route never has to validate the mode name itself.
    """

    mode: Mode


class ModeAck(BaseModel):
    """Body of a successful POST /mode: {"ok", "mode", "previous"}.

    Echoes both the newly-active mode and the one it replaced (development plan section
    5.1), so a client gets a definitive before/after without a second GET /status. Both
    are typed as Mode and serialize to their wire strings. A re-entry of the active mode
    is rejected with 409 rather than acked, so in a successful response previous always
    differs from mode.
    """

    ok: bool
    mode: Mode
    previous: Mode


@router.post("/mode", response_model=ModeAck)
async def post_mode(command: ModeCommand, request: Request) -> ModeAck:
    """Switch the payload mode, tearing down the mode being left, and echo before/after.

    Args:
        command: the target mode.
        request: the incoming request, used only to reach app.state.session.

    Returns:
        ModeAck(ok=True, mode=<new>, previous=<old>) once the switch has completed --
        which includes the old mode's deter loop and audio sessions being fully torn down.

    Raises:
        HTTPException(409): if the target mode is the one already active. Re-entering a
            mode is treated as a conflict (nothing would be reset) rather than a silent
            success, so the client learns its request had no effect.

    The whole transition -- quiesce the old mode, then adopt the new one -- happens inside
    SessionManager.transition under its lock, so this handler is a thin translation layer:
    it forwards the request and maps the one expected state conflict to 409. Any device
    reset needed to leave the old mode is done by that teardown, not here.
    """
    # Reach the mode authority placed on app.state during lifespan startup.
    session: SessionManager = request.app.state.session
    try:
        # transition tears down the old mode and adopts the new one atomically, returning
        # the previous mode. It raises only when the target equals the current mode.
        previous = await session.transition(command.mode)
    except ModeTransitionError as exc:
        # Re-entry of the active mode: a conflict, not a bad request or a device fault.
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return ModeAck(ok=True, mode=command.mode, previous=previous)


class DeterCommand(BaseModel):
    """Body of POST /deter: arm or disarm the 驱离 deterrent (development plan section 5.1).

    on is the only required field: true arms (or re-arms) the loop, false disarms it. The
    three optional tunables are applied only when arming; each defaults to None meaning
    "use the deter default", so a bare {"on": true} runs the deterrent with its configured
    defaults. Ranges are enforced here at the boundary (a 422 before any run starts) and
    mirror DeterParams.validate: mode is the red/blue pattern 1..REDBLUE_MAX (0/off is not
    a deterrent, so the floor is 1, unlike the general /lights route), siren_level is the
    0..1 amplitude, and tts_reps is the non-negative warning repeat count. There is no text
    field: the spoken line is configuration (config.deter_tts_text), not request data.
    """

    on: bool
    # None => use the deter default red/blue pattern; else 1..REDBLUE_MAX (0=off disallowed).
    mode: Optional[int] = Field(default=None, ge=1, le=REDBLUE_MAX)
    # None => use the deter default siren amplitude; else 0..1 inclusive.
    siren_level: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    # None => use the deter default repeat count; else a non-negative number of warnings.
    tts_reps: Optional[int] = Field(default=None, ge=0)


class DeterAck(BaseModel):
    """Body of a successful POST /deter: a bare {"ok": true} acknowledgement.

    Like /lights, this is a command not a query, so there is no state to echo -- the
    authoritative "is the deterrent running" is derivable from the mode and the operator's
    own on/off, and re-deriving it here could disagree. A minimal ack keeps one source of
    truth.
    """

    ok: bool


@router.post("/deter", response_model=DeterAck)
async def post_deter(command: DeterCommand, request: Request) -> DeterAck:
    """Arm or disarm the deterrent loop; requires the service to be in DETER mode.

    Args:
        command: on plus the optional arm-time tunables.
        request: the incoming request, used only to reach app.state.session.

    Returns:
        DeterAck(ok=True) once the loop has been armed (on=true) or disarmed (on=false).

    Raises:
        HTTPException(409): if the service is not in DETER mode. Mode selection is done
            through POST /mode; /deter only controls the deterrent WITHIN that mode, so a
            call outside DETER is a state conflict (the client must POST /mode deter first).
        HTTPException(503): if arming cannot reach the device (audio or lights link down);
            SessionManager resets any partially-raised lights before the error propagates.

    Arming builds a DeterParams from the present fields (absent ones take the deter
    defaults) and hands it to SessionManager.start_deter, which validates, resets any prior
    run, and starts the loop under its lock. Disarming forwards to stop_deter. This handler
    stays a thin translator: it never touches the device or the loop directly (R1).
    """
    # Reach the mode/deter authority placed on app.state during lifespan startup.
    session: SessionManager = request.app.state.session
    if command.on:
        # Build the run parameters from only the fields the client set, so each omitted
        # field falls back to the DeterParams default rather than being forced to None.
        overrides: dict = {}
        if command.mode is not None:
            overrides["redblue_mode"] = command.mode
        if command.siren_level is not None:
            overrides["siren_level"] = command.siren_level
        if command.tts_reps is not None:
            overrides["tts_reps"] = command.tts_reps
        params = DeterParams(**overrides)
        try:
            # start_deter requires DETER mode and arms the device; it resets any partial
            # light state itself if the device link drops mid-arm.
            await session.start_deter(params)
        except ModeStateError as exc:
            # Not in DETER mode: a state conflict the client resolves via POST /mode deter.
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except DeviceLinkError as exc:
            # Device unreachable while arming: Service Unavailable, not a client error.
            raise HTTPException(status_code=503, detail=str(exc)) from exc
    else:
        try:
            # Disarm; also requires DETER mode. A no-op if nothing was running.
            await session.stop_deter()
        except ModeStateError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    return DeterAck(ok=True)
