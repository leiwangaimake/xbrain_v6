"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: device_link.py
Brief: Sole owner of the GZH-2 device TCP links (8519 audio, 8529 lights).

Description:
  Test-system invariant R1 states that exactly one process -- payload-service -- may
  touch the device's raw TCP ports, and every other module must reach the device
  through this service. DeviceLink is where that ownership physically lives: it is
  the only object in the whole system that opens a socket to 8519 or 8529. Everything
  above it (the REST layer now, and in later milestones AI_runtime and office-client)
  reads device state through this class instead of dialing the hardware directly, so
  there is a single, auditable choke point for all device I/O.

  Responsibilities in milestone M1:
    1. Keep both device sockets connected, reconnecting automatically whenever the
       device drops, refuses, or resets a link.
    2. Run a background reader on 8529 that parses the 1 Hz 0x25 status stream into
       a single cached LightsStatus, which GET /status serves without ever touching
       a socket itself.

  ★ On the status cadence: the GZH-2 protocol document states 500 ms, and every comment
  in this file used to repeat it. Measured against the real unit on 2026-08-03 it is
  1 Hz -- six reports in six seconds, taken straight off the socket. The consequence is
  not cosmetic: a command's effect can only be confirmed by the NEXT report, so the
  observed confirmation latency after POST /lights is about 980 ms with a worst case of
  1.34 s, against the 500 ms that 11 T-PAY-2 budgets for exactly that confirmation. Any
  timeout derived from the documented figure is therefore unreachable, and a caller that
  reads the status back immediately will see the old value.
    3. Flush-close each socket on shutdown: drain unread receive data and wait
       close_flush_ms before close(), because close() with unread rx makes Linux emit
       a TCP RST, and on real hardware that RST dropped the device's final frame
       (leaving the searchlight stuck on). See development plan sections 8 and 9.

  Milestone M2 adds one responsibility on top of that M1 set:
    4. Send outbound 8529 control frames (searchlight, brightness, strobe, red/blue)
       on the SAME socket the lights supervisor already owns, driven from the REST
       thread via send_lights_control. TCP is full-duplex, so a send from the REST
       thread runs safely alongside the supervisor's recv() on the other direction of
       the one socket; a dedicated send lock serialises concurrent senders and guards
       the single published handle to that socket. This is what keeps invariant R1
       intact for writes as well as reads -- the sender never opens its own socket, it
       borrows the one the supervisor owns, so there is still exactly one socket owner.

  Milestone M3 turns the 8519 audio link from an M1 keep-alive into a full duplex
  audio path, using the very same borrow-the-owned-socket discipline as M2's lights
  send:
    5. Send outbound 8519 audio frames ([42] hail, [11] stop, [31] TTS) on the socket
       the audio supervisor owns, via send_audio, guarded by its own _audio_send_lock.
       This is the exact mirror of send_lights_control, one link over.
    6. Record uplink: start_recording sends [40] and arms an inbound path; while armed,
       the audio supervisor feeds each recv() chunk to an AudioUplinkFramer and hands
       the raw Opus packets it yields to a caller-registered mic sink; stop_recording
       sends [41] and disarms. device_link stays CODEC-FREE -- it owns the framer (the
       "where do frames begin/end" job) exactly as it owns the 8529 framer, and leaves
       Opus decode and resample to the codec layer above it. A dropped audio link
       disarms recording, so "audio link up" is a precondition of "recording".

  Lifecycle sequence (the states each thread walks through):
    construct -> start() spawns two daemon threads -> each thread loops
      { _connect -> set up_flag -> read until drop/stop -> clear up_flag ->
        _flush_close } -> stop() sets the stop Event and joins both threads.
    The loop body has the same shape for both links; only the read phase differs
    (lights parses frames, audio merely keeps the link warm in M1). A connect failure
    short-circuits straight back to the top of the loop after a backoff, so the
    "connected" state is only ever entered through _connect and only ever left
    through _flush_close -- there is exactly one door in and one door out.

  Concurrency contract (who touches what, and what makes it safe):
    - _latest_lights is written ONLY by the lights thread and read ONLY by the REST
      thread; the single mutex _lights_lock guards that one-slot value and is held for
      just the pointer read or write, so neither side ever waits on the other for a
      meaningful span.
    - _audio_up / _lights_up are threading.Events, not plain bools, because their
      set / clear / is_set operations are atomic; that lets the REST thread read
      connectivity with no lock at all.
    - _stop_evt is set by stop() and observed by both threads. It does double duty as
      an interruptible sleep for the reconnect backoff and the pre-close flush wait,
      so a shutdown is never blocked waiting for a timer to finish counting down.
    - No socket's LIFECYCLE is shared across threads: each supervisor thread alone
      opens, reads from, and closes its own socket. That single-owner rule for
      open/close is precisely what lets _flush_close run on the owning thread without
      racing an in-flight recv() on another thread.
    - The one cross-thread sharing is the M2 lights SEND path, and it is deliberately
      narrow. _lights_sock holds the live lights socket (or None when the link is
      down); the lights supervisor publishes it under _lights_send_lock right after a
      successful connect and clears it under the same lock in its finally, BEFORE the
      flush-close. send_lights_control takes that lock and, only while holding it,
      reads the handle and sendall()s -- so a send can never touch a socket the
      supervisor is about to close, and two senders can never interleave their frames.
      A send is safe alongside the supervisor's recv() because TCP is full-duplex: the
      two threads touch opposite directions of the one connection, never a shared
      buffer.

  Reconnect semantics:
    A supervisor never gives up. A refused or reset link is an expected operational
    event (device rebooting, cable pulled, adapter cycled), so the loop retries
    forever behind a fixed backoff rather than letting the thread die. The 8529 framer
    is created ONCE and reused across reconnects; its resync-to-0x8D logic discards any
    stale partial bytes left over from the previous connection when the new stream
    begins, so a disconnect in the middle of a frame cannot corrupt the next session.

  Failure taxonomy and log levels (so an operator can skim the log meaningfully):
    - info    : a link came up (a connect succeeded) -- a low-volume milestone event.
    - warning : an EXPECTED transient fault: a failed connect, a recv error, or a
      clean EOF from the device. Each is self-healing via reconnect, so it is not an
      error, but a storm of them is the signal that the device or cabling is unwell.
    - debug   : a benign, self-correcting data glitch -- a single malformed 0x25
      frame. The next frame a second later almost always parses, so it stays off by
      default.

  Threading model (chosen deliberately):
    - One daemon thread per socket. The two links are independent -- 8529 streams
      status continuously while 8519 is idle until recording starts -- so giving each
      its own supervisor keeps a stall on one link from blocking the other.
    - Each thread uses a blocking recv() with a SHORT poll timeout. That timeout is
      the mechanism by which the loop notices a stop request: it wakes roughly twice
      a second to recheck the stop Event. This is why no socket is ever closed from
      another thread -- each thread owns its socket for that socket's entire life and
      flush-closes it itself on the way out, so there is no cross-thread close racing
      an in-flight recv().
    - The audio socket has no inbound parsing in M1. It is held open purely so
      GET /status can report audio_connected truthfully and so the link is already
      warm (no ~98 ms first-frame cost) for the M3 audio milestone.

  What this module still deliberately leaves out (and where each part lands):
    - Opus decode and 8k<->16k resample: they live in the codec layer that the WS
      handlers drive, NOT here -- device_link owns transport and framing only, so the
      audio uplink framer hands out raw Opus and never decodes it -> codec/, api/ws.py.
    - The 驱离 (deter) siren+TTS sequence composed on top of send_audio -> a later step.
    - The mode session-gate that enforces the R2/R3 mutual-exclusion rules -> milestone
      M4; until it exists the service reports the idle baseline (see api/rest.py) and
      send_audio / start_recording are callable without a mode check. Leaving these out
      now is house V1/V2 discipline: the code carries no stub that merely reserves a
      shape for work not yet being done.
"""
from __future__ import annotations

import logging
import socket
import threading
from typing import Callable, List, Optional

from ..config import PayloadConfig
from ..protocol.audio_8519 import (
    AudioUplinkFramer,
    build_record_start,
    build_record_stop,
)
from ..protocol.lights_8529 import (
    Lights8529Framer,
    LightsStatus,
    MSG_ID_STATUS,
    STATUS_PAYLOAD_LEN,
    parse_status_payload,
    status_crc_ok,
)

# Type of the mic sink a recording caller registers: it receives one raw Opus packet
# per uplink record frame. The contract (see start_recording) is that it must return
# promptly and not block, and must not call back into DeviceLink -- the audio supervisor
# thread invokes it inline, so a slow or reentrant sink would stall device reads. A
# typical sink schedules the packet onto an asyncio loop and returns at once.
MicSink = Callable[[bytes], None]

# Module logger. House rule: logs and exception messages are English even though
# user-facing dialogue elsewhere in the system is Chinese.
logger = logging.getLogger("payload.device_link")

# recv() poll timeout (seconds) while a socket is connected. This is the shutdown
# responsiveness knob: the read loop only rechecks the stop Event when recv() returns
# or times out, so a value near half a second bounds how long stop() waits for a
# thread to notice, while staying long enough not to spin the CPU. It is intentionally
# NOT a config field -- it is an internal responsiveness detail, not something an
# operator should tune (house V1/V2 discipline: no speculative config switches).
_RECV_POLL_S = 0.5

# Wait (seconds) between reconnect attempts after a connect fails. Without it, a
# powered-off device would turn the supervisor loop into a busy reconnect spin that
# burns CPU and floods the log; one second is a calm retry cadence for a LAN device.
_RECONNECT_BACKOFF_S = 1.0

# recv() chunk size (bytes). A single 0x25 status frame is 28 bytes on the wire, so
# 4096 comfortably absorbs a burst of several frames per read without allocating an
# oversized buffer.
_RECV_CHUNK = 4096


class DeviceLinkError(RuntimeError):
    """Raised when an outbound device command cannot be delivered.

    Currently the sole raiser is send_lights_control, in two cases: the lights link is
    down at send time (nothing to write to), or the sendall itself fails because the
    link broke mid-write. A dedicated type (house rule bans bare Exception) lets the
    REST layer translate a delivery failure into a 503 while leaving genuine
    programming errors -- a mistyped attribute, a bad argument -- to surface as their
    own exception types instead of being masked as a device fault. It subclasses
    RuntimeError, not ValueError, because the value handed in was well-formed; what
    failed was the runtime act of delivering it over a link that was not available.
    """


class DeviceLink:
    """Owns and supervises the two device sockets and exposes cached read state.

    Construct once with a PayloadConfig, call start() to bring the links up, and
    call stop() for an orderly, frame-preserving teardown. All state readers
    (audio_connected, lights_connected, snapshot_lights) are safe to call from the
    ASGI request thread while the reader threads run.
    """

    def __init__(self, config: PayloadConfig) -> None:
        """Initialize link state; does NOT open any socket (start() does that).

        Args:
            config: the immutable service configuration providing device host/ports
                and the close-flush grace period.

        Deliberately side-effect-free: constructing a DeviceLink allocates only the
        stop Event, the lights lock and its one-slot cache, the two connectivity
        Events, and an empty thread list. No socket is dialed and no thread is spawned
        until start() runs. That separation lets the FastAPI lifespan build the object
        during app construction and defer all device contact to the serving phase, so
        importing or unit-testing the app has no effect on the real hardware.
        """
        self._cfg = config
        # Single stop signal for both reader threads. It also serves double duty as
        # an interruptible sleep: reconnect backoff and the pre-close flush wait both
        # use stop_evt.wait(...), so a stop request cuts those waits short instead of
        # forcing shutdown to block for their full duration.
        self._stop_evt = threading.Event()
        # Latest parsed lights status plus the lock guarding it. The reader thread is
        # the sole writer and the REST handler is a reader; a single mutex around a
        # one-slot value is the simplest correct sharing and cannot deadlock.
        self._lights_lock = threading.Lock()
        self._latest_lights: Optional[LightsStatus] = None
        # Connectivity flags for GET /status. threading.Event is used (rather than a
        # bare bool) because its set()/clear()/is_set() are atomic, so the request
        # thread can read connectivity without taking a lock.
        self._audio_up = threading.Event()
        self._lights_up = threading.Event()
        # M2 outbound-send state for the 8529 lights link. _lights_send_lock does two
        # jobs at once: it guards the _lights_sock handle (published by the lights
        # supervisor on connect, cleared before flush-close) AND serialises concurrent
        # senders so two control commands can never interleave their frames on the
        # wire. _lights_sock is None whenever the link is down, which is the single
        # condition send_lights_control checks to fail fast with DeviceLinkError.
        self._lights_send_lock = threading.Lock()
        self._lights_sock: Optional[socket.socket] = None
        # M3 outbound-send state for the 8519 audio link -- the exact mirror of the
        # lights send state one link over. _audio_send_lock guards the _audio_sock
        # handle (published by the audio supervisor on connect, cleared before
        # flush-close) and serialises concurrent audio senders (a /play [42] burst vs a
        # /tts [31] frame). _audio_sock is None whenever the audio link is down.
        self._audio_send_lock = threading.Lock()
        self._audio_sock: Optional[socket.socket] = None
        # M3 record-uplink state. _record_lock guards all three fields together, because
        # start/stop (an ASGI thread) writes them while the audio supervisor thread reads
        # them on every recv. _recording is the single arm/disarm flag the supervisor
        # checks before framing; _mic_sink is where framed Opus packets go while armed;
        # _uplink_framer reassembles [40] frames from the raw byte stream and is created
        # once and reset per recording session (like the lights framer is reused across
        # reconnects).
        self._record_lock = threading.Lock()
        self._recording = False
        self._mic_sink: Optional[MicSink] = None
        self._uplink_framer = AudioUplinkFramer()
        # Handles to the two supervisor threads, populated by start(), joined by
        # stop(). Empty until start() runs.
        self._threads: List[threading.Thread] = []

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        """Spawn the two per-socket supervisor threads and return immediately.

        The threads are daemon threads so that a hard process exit can never hang
        waiting on them; an orderly shutdown still goes through stop(), which is the
        path that guarantees the frame-preserving flush-close. Daemon status is a
        safety net for the abnormal exit, NOT the normal teardown route: relying on it
        for a clean stop would skip the flush-close and risk the very RST-drops-final-
        frame bug this module exists to avoid, so the healthy path always calls stop().

        start() returns as soon as the threads are launched; it does not wait for
        either link to come up. A caller that needs to know the links are live reads
        audio_connected / lights_connected, which flip true once each _connect
        succeeds. This non-blocking start is what lets the ASGI lifespan finish startup
        promptly even when the device is momentarily unreachable.
        """
        self._threads = [
            # 8529 lights link: connect, read and parse the 0x25 status stream.
            threading.Thread(target=self._serve_lights, name="lights-8529", daemon=True),
            # 8519 audio link: connect and hold open (M1 keep-alive only).
            threading.Thread(target=self._serve_audio, name="audio-8519", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        """Signal both reader threads to wind down and wait for them to finish.

        Each thread flush-closes its OWN socket as it exits, so stop() never closes a
        socket out from under an in-flight recv() (see the module header threading
        note). stop() only signals and joins; it touches no socket itself.

        The join budget below is sized from first principles rather than guessed. A
        healthy thread, once it sees the stop Event, needs at most: up to one recv poll
        timeout to return from a blocking recv(), plus the close_flush_ms grace the
        flush-close waits, plus a small constant slack for the drain-and-close itself.
        Summing those bounds the time an orderly exit can take, so the join almost
        always completes well inside it. If a thread is pathologically wedged the join
        times out, but because the thread is a daemon that still cannot block process
        exit -- we simply stop waiting on it and let the interpreter reap it.
        """
        # Ask both loops to stop. Any thread parked in a backoff or flush wait wakes
        # immediately because those waits are on this same Event.
        self._stop_evt.set()
        # Join budget: comfortably longer than one recv poll timeout plus the flush
        # wait, so a healthy thread always exits within it. A pathologically wedged
        # thread is a daemon, so even if the join times out it cannot block process
        # exit -- we simply stop waiting on it.
        join_budget_s = _RECV_POLL_S + self._cfg.close_flush_ms / 1000.0 + 2.0
        for thread in self._threads:
            thread.join(timeout=join_budget_s)

    # -- state readers ----------------------------------------------------

    @property
    def audio_connected(self) -> bool:
        """Whether the 8519 audio socket is currently connected (lock-free read).

        Reads the underlying Event with is_set(), which is atomic, so the REST thread
        needs no lock. The value is a point-in-time snapshot: it may flip the instant
        after it is read if the link drops, which is fine for a status report -- the
        next poll simply reflects the new state.
        """
        return self._audio_up.is_set()

    @property
    def lights_connected(self) -> bool:
        """Whether the 8529 lights socket is currently connected (lock-free read).

        Same contract as audio_connected: an atomic, lock-free, point-in-time read of
        the connectivity Event that the lights supervisor sets on connect and clears in
        its finally on any exit.
        """
        return self._lights_up.is_set()

    def snapshot_lights(self) -> Optional[LightsStatus]:
        """Return the most recently parsed lights status, or None if none yet.

        Returns:
            The last valid LightsStatus decoded from a 0x25 frame, or None when no
            valid status has arrived yet (for example in the brief window right after
            startup). GET /status tolerates the None and serializes it as null, per
            development plan section 9.

        Latest-wins, not a queue: the device re-sends full state about once a second, so a
        single overwritable slot is the right model -- an old status is never useful
        once a newer one exists, and buffering them would only add staleness and
        memory. The returned value is an immutable LightsStatus, so the caller may read
        its fields after releasing the lock without any risk of the reader thread
        mutating it underneath them.
        """
        # Hold the lock only for the pointer read so the reader thread is never
        # blocked writing a new snapshot for longer than a single assignment.
        with self._lights_lock:
            return self._latest_lights

    # -- outbound control -------------------------------------------------

    def send_lights_control(self, frames: List[bytes]) -> None:
        """Send one or more prebuilt 8529 control frames on the live lights socket.

        Args:
            frames: the encoded control frames to write, in order. The REST layer
                builds one frame per field present in a POST /lights body (for example
                a brightness frame and a red/blue frame) and passes them together so
                the whole command is delivered as one uninterrupted group rather than
                racing another request's frames between them.

        Raises:
            DeviceLinkError: if the lights link is down at send time, or if the write
                fails because the link broke mid-send. Either way the REST handler
                turns it into a 503, and the supervisor's own recv loop independently
                notices the broken link and reconnects -- send does not drive reconnect.

        Concurrency: the whole read-handle-then-write sequence runs under
        _lights_send_lock. Holding the lock across the sendall is what makes the send
        atomic against teardown -- the supervisor clears _lights_sock under this SAME
        lock before flush-closing, so while this method holds it the socket cannot be
        closed underneath the write -- and atomic against other senders, so two
        concurrent commands cannot interleave their bytes on the wire. This lock is NOT
        _lights_lock (which guards the status cache); they are distinct mutexes over
        distinct state and are never held nested, so no deadlock is possible. Each
        control frame is five bytes, so sendall returns effectively instantly and the
        lock is held for a negligible span even with the socket's recv timeout in force.
        """
        # Serialise with teardown and with other senders (see the docstring). Reading
        # the handle and writing to it MUST sit in one critical section, or the
        # supervisor could null-and-close the socket between the None-check and sendall.
        with self._lights_send_lock:
            sock = self._lights_sock
            if sock is None:
                # Link down (never connected, or already torn down): nothing to write.
                raise DeviceLinkError("lights link is down; control frame not sent")
            try:
                # sendall loops internally until every byte is accepted by the kernel,
                # so a short write cannot silently truncate a control frame.
                for frame in frames:
                    sock.sendall(frame)
            except OSError as exc:
                # Link broke mid-send. Report a delivery failure; the supervisor's recv
                # loop will observe the same break and handle the reconnect.
                raise DeviceLinkError(f"lights control send failed: {exc}") from exc

    def send_audio(self, frames: List[bytes]) -> None:
        """Send one or more prebuilt 8519 audio frames on the live audio socket.

        Args:
            frames: the encoded audio frames to write, in order -- for example a run of
                [42] hail packets from /play, or a single [31] TTS frame from POST /tts.
                Passing a group together writes them back-to-back with no other sender's
                frames able to slip between them.

        Raises:
            DeviceLinkError: if the audio link is down at send time, or if the write
                fails because the link broke mid-send. The caller (a WS or REST handler)
                turns it into a failure response; the supervisor's recv loop
                independently notices the break and reconnects -- send does not.

        This is the byte-for-byte analogue of send_lights_control on the audio link:
        the whole read-handle-then-write runs under _audio_send_lock so the socket
        cannot be closed underneath the write (the supervisor clears _audio_sock under
        the same lock before flush-close) and two senders cannot interleave their bytes.
        It is a dumb writer -- it does not know [42] from [31]; the typed builders in
        protocol/audio_8519 produce the bytes, keeping this method a pure transport.
        """
        # One critical section for the handle read and the write, so the supervisor
        # cannot null-and-close _audio_sock between the None-check and the sendall.
        with self._audio_send_lock:
            sock = self._audio_sock
            if sock is None:
                # Audio link down (never connected, or already torn down).
                raise DeviceLinkError("audio link is down; frame not sent")
            try:
                # sendall loops until the kernel accepts every byte, so a large [42]
                # burst cannot be silently truncated by a short write.
                for frame in frames:
                    sock.sendall(frame)
            except OSError as exc:
                raise DeviceLinkError(f"audio send failed: {exc}") from exc

    def start_recording(self, sink: MicSink) -> None:
        """Arm the record uplink: register a mic sink, then ask the device to record.

        Args:
            sink: called once per uplink record frame with that frame's raw Opus bytes.
                It runs INLINE on the audio supervisor thread, so it must return promptly
                and must not block or call back into DeviceLink; a well-behaved sink just
                hands the packet to an asyncio loop (loop.call_soon_threadsafe) and
                returns. Decoding the Opus is the sink side's job -- device_link stays
                codec-free.

        Raises:
            DeviceLinkError: if the [40] start command cannot be sent (audio link down).
                The recording state is rolled back so a failed start leaves nothing armed.

        Order matters: the framer is reset and the sink armed FIRST, under _record_lock,
        so that the instant the device begins streaming there is somewhere to route the
        frames; only then is [40] sent. If that send fails the arm is undone, because a
        device that never got [40] will never stream and a half-armed state would only
        mislead a later reconnect.
        """
        with self._record_lock:
            # Fresh framer per session: drop any trailing partial from a previous
            # recording so it cannot be prepended to this session's first frame.
            self._uplink_framer.reset()
            self._mic_sink = sink
            self._recording = True
        try:
            # Tell the device to start streaming mic audio. Sent outside _record_lock so
            # the send lock and the record lock are never held nested (no lock-order
            # coupling between the two paths).
            self.send_audio([build_record_start()])
        except DeviceLinkError:
            # The device never got [40], so undo the arm rather than leaving a sink wired
            # to a stream that will never come.
            self._clear_recording()
            raise

    def stop_recording(self) -> None:
        """Disarm the record uplink and ask the device to stop recording.

        Disarms FIRST (so no further frames reach the sink the caller is abandoning),
        then sends [41] as a best effort. A send failure is swallowed: if the audio link
        is already down the device has nothing to stop, and the disarm -- the part that
        matters for this process -- has already happened.
        """
        # Stop routing frames immediately; a late packet in flight is simply dropped.
        self._clear_recording()
        try:
            self.send_audio([build_record_stop()])
        except DeviceLinkError:
            # Link already down: the device is not streaming anyway, so nothing to stop.
            logger.warning("stop_recording: audio link down, [41] not sent")

    def _clear_recording(self) -> None:
        """Disarm recording under the record lock (idempotent).

        Sets _recording false and drops the sink reference in one locked step. Called
        both by stop_recording and by the audio supervisor's teardown, so that a dropped
        audio link also ends any recording -- making "audio link up" a precondition of
        "recording" and guaranteeing the supervisor never dispatches frames from a fresh
        reconnect on which the device is not actually recording. A no-op when already
        disarmed, so calling it on every audio teardown is harmless.
        """
        with self._record_lock:
            self._recording = False
            self._mic_sink = None

    # -- endpoint supervision loops ---------------------------------------

    def _serve_lights(self) -> None:
        """Supervisor loop for the 8529 lights link: connect, read, repeat.

        Runs on its own thread until stop() is requested. On each iteration it
        (re)connects, reads the status stream until the link drops, then loops to
        reconnect. The framer is created once and reused across reconnects; its
        resync-to-header logic naturally discards any stale partial bytes from a
        previous connection when the new stream starts.

        The try/finally is the invariant that keeps connectivity honest: no matter how
        the read phase ends -- device disconnect, recv error, or a stop request -- the
        finally clears _lights_up and flush-closes the socket before the loop either
        reconnects or exits. That guarantees GET /status never reports lights_connected
        true for a socket that has actually gone away, and that every socket this loop
        opens is eventually flush-closed exactly once.
        """
        framer = Lights8529Framer()
        while not self._stop_evt.is_set():
            sock = self._connect(self._cfg.port_lights, self._lights_up)
            if sock is None:
                # Connect failed; _connect already applied the backoff wait. Loop to
                # retry (or exit if the backoff was cut short by a stop request).
                continue
            # Publish the live socket so the REST thread's send_lights_control can
            # write control frames on it. Done under the send lock so a concurrent
            # sender sees either the fully-established socket or (before this) None,
            # never a half-open handle mid-assignment.
            with self._lights_send_lock:
                self._lights_sock = sock
            try:
                self._read_lights(sock, framer)
            finally:
                # Retract the send handle FIRST, under the send lock, so no send can
                # start on the socket we are about to tear down; a sender arriving
                # after this sees None and fails fast with DeviceLinkError instead of
                # writing into a closing socket. The clear MUST precede _flush_close,
                # which switches the socket to non-blocking and closes it -- a state a
                # send must never observe. Only after retracting do we mark the link
                # down and flush-close it, before reconnecting or exiting.
                with self._lights_send_lock:
                    self._lights_sock = None
                self._lights_up.clear()
                self._flush_close(sock)

    def _serve_audio(self) -> None:
        """Supervisor loop for the 8519 audio link: connect, publish, read, repeat.

        Now the exact structural twin of _serve_lights. On connect it publishes the live
        socket as _audio_sock (under _audio_send_lock) so send_audio can write [42]/[31]
        frames on it, then reads via _read_audio -- which discards inbound bytes when
        idle and reframes them to the mic sink while recording.

        The finally does three things, in the order that keeps every invariant honest:
        retract the send handle first (under the send lock, so no send touches a socket
        about to close), then disarm recording (so a link drop ends any /mic session and
        a reconnect never dispatches frames the device is not really sending), then mark
        the link down and flush-close. Sharing the loop shape with the lights supervisor
        keeps one reconnect and one teardown policy across both links.
        """
        while not self._stop_evt.is_set():
            sock = self._connect(self._cfg.port_audio, self._audio_up)
            if sock is None:
                continue
            # Publish the live socket so send_audio can borrow it, under the send lock so
            # a concurrent sender sees either the established socket or (before) None.
            with self._audio_send_lock:
                self._audio_sock = sock
            try:
                self._read_audio(sock)
            finally:
                # Retract the send handle first so no send starts on a socket we are
                # about to tear down; a sender arriving after this fails fast instead.
                with self._audio_send_lock:
                    self._audio_sock = None
                # A dropped audio link ends recording: disarm so the next reconnect does
                # not feed the framer for a device that is no longer streaming.
                self._clear_recording()
                self._audio_up.clear()
                self._flush_close(sock)

    # -- socket helpers ---------------------------------------------------

    def _connect(self, port: int, up_flag: threading.Event) -> Optional[socket.socket]:
        """Attempt one connection to the device, with backoff on failure.

        Args:
            port: the device TCP port to dial (8519 or 8529).
            up_flag: the connectivity Event to set once connected.

        Returns:
            A connected socket already carrying the short recv poll timeout, or None
            if the connect failed (the caller loop then retries) or a stop was
            requested during the backoff wait.

        A failed connect is expected operationally (device rebooting, cable pulled),
        so it is logged at warning and followed by an interruptible backoff rather
        than raising -- the supervisor loop is designed to keep retrying forever.

        Two timeouts are in play and the swap between them is the subtle part. The
        connect attempt uses the longer socket_timeout_s so a dead host fails fast
        instead of hanging the supervisor for minutes; once connected, the socket is
        immediately re-armed with the much shorter _RECV_POLL_S so the read loop wakes
        about twice a second to observe the stop Event. up_flag is set only after the
        connection succeeds and only cleared by the supervisor's finally, so the flag's
        lifetime brackets exactly the interval the socket is usable. Returning None (on
        connect failure or a stop cutting the backoff short) is the loop's signal to
        retry, so this method never raises into its caller.
        """
        try:
            # create_connection applies socket_timeout_s to the connect attempt so a
            # dead device fails fast instead of blocking the supervisor for minutes.
            sock = socket.create_connection(
                (self._cfg.device_host, port), timeout=self._cfg.socket_timeout_s
            )
        except OSError as exc:
            # Connect refused/unreachable/timed out: back off, then let the caller
            # retry. wait() returns early if stop() fires, so shutdown stays snappy.
            logger.warning("connect %s:%d failed: %s", self._cfg.device_host, port, exc)
            self._stop_evt.wait(_RECONNECT_BACKOFF_S)
            return None
        # Swap the (longer) connect timeout for the (shorter) recv poll timeout, so
        # the subsequent read loop rechecks the stop Event about twice a second.
        sock.settimeout(_RECV_POLL_S)
        up_flag.set()
        logger.info("connected %s:%d", self._cfg.device_host, port)
        return sock

    def _read_lights(self, sock: socket.socket, framer: Lights8529Framer) -> None:
        """Read the 8529 stream until stop or disconnect, caching valid 0x25 frames.

        Args:
            sock: the connected lights socket.
            framer: the reused stream framer that reassembles whole frames.

        Returns when the link drops or a stop is requested, so the supervisor can
        flush-close and reconnect. A recv() timeout is NOT an error here: status
        arrives only about once a second, so a timeout is normal idle and simply gives the
        loop a chance to recheck the stop Event.

        The three ways this loop ends are handled distinctly and on purpose:
          - socket.timeout  -> normal idle, continue (do NOT reconnect on idle);
          - OSError         -> the link broke, log at warning and return to reconnect;
          - empty recv()    -> the peer closed cleanly (EOF), log and return.
        Only the last two end the read phase; conflating an idle timeout with a
        disconnect would tear the link down and reconnect it needlessly twice a second.
        Every non-empty chunk is handed straight to the framer, which is the sole
        component that understands frame boundaries -- this loop never inspects bytes.
        """
        while not self._stop_evt.is_set():
            try:
                data = sock.recv(_RECV_CHUNK)
            except socket.timeout:
                # Normal idle between the 500 ms status frames: loop and recheck stop.
                continue
            except OSError as exc:
                # Broken link (reset, adapter down, ...): return to trigger reconnect.
                logger.warning("lights recv error, will reconnect: %s", exc)
                return
            if not data:
                # recv() returning empty means the peer closed cleanly (EOF).
                logger.warning("lights link closed by device, will reconnect")
                return
            # Feed the chunk to the framer and act on every whole frame it yields.
            for msg_id, payload, crc in framer.feed(data):
                self._handle_lights_frame(msg_id, payload, crc)

    def _handle_lights_frame(self, msg_id: int, payload: bytes, crc: int) -> None:
        """Validate and cache a single decoded 8529 frame.

        Args:
            msg_id: the frame's message id.
            payload: the frame's payload bytes.
            crc: the trailing CRC byte the framer extracted.

        In M1 only the 0x25 status report is consumed; any other message id (for
        example a future control reply) is ignored here. The frame is trusted only
        after both the expected payload length and the PAYLOAD-ONLY CRC check pass
        (the coverage asymmetry, development plan section 9), so a corrupt or
        misaligned frame can never publish a bogus light state.

        Validation order is length-then-CRC, and the two are combined in a single
        guard so either failure drops the frame the same way. Checking length first is
        not just tidiness: status_crc_ok / parse_status_payload read fixed byte offsets
        inside the payload, so a short payload must be rejected before those offsets
        are touched. Publishing happens only on the success path, and the lock wraps
        just the assignment so the reader thread hands the REST thread a fully-formed,
        already-validated LightsStatus every time -- a reader can never observe a
        half-written or unchecked value.
        """
        if msg_id != MSG_ID_STATUS:
            # Not a status frame: nothing else is meaningful to M1, so drop it.
            return
        if len(payload) != STATUS_PAYLOAD_LEN or not status_crc_ok(payload, crc):
            # Wrong length or failed payload-only CRC: almost certainly a rare framing
            # glitch. Debug (not warning) because it is benign and self-correcting --
            # the next 500 ms frame will very likely be clean.
            logger.debug("dropping invalid 0x25 frame (payload_len=%d)", len(payload))
            return
        # Decode and publish. The lock is held only for the single-slot store.
        status = parse_status_payload(payload)
        with self._lights_lock:
            self._latest_lights = status

    def _read_audio(self, sock: socket.socket) -> None:
        """Read the 8519 stream until stop or disconnect, dispatching while recording.

        Args:
            sock: the connected audio socket.

        Returns when the link drops or a stop is requested, so the supervisor can
        flush-close and reconnect. A recv() timeout is the normal idle case: the device
        sends nothing on 8519 until [40] recording is armed, so a timeout just lets the
        loop recheck the stop Event.

        The three terminations mirror the lights reader exactly:
          - socket.timeout -> normal idle, continue (do NOT reconnect on idle);
          - OSError        -> the link broke, log at warning and return to reconnect;
          - empty recv()   -> the peer closed cleanly (EOF), log and return.
        Every non-empty chunk goes to _dispatch_audio, which decides -- under the record
        lock -- whether the bytes are recording uplink to reframe or idle bytes to drop.
        Reading even when not recording still matters (as it did for the M1 keep-alive):
        it detects a silent disconnect quickly and keeps unread bytes from piling up and
        turning the eventual close() into an RST, the hazard _flush_close guards against.
        """
        while not self._stop_evt.is_set():
            try:
                data = sock.recv(_RECV_CHUNK)
            except socket.timeout:
                # Expected idle: nothing streams on 8519 until recording is armed.
                continue
            except OSError as exc:
                logger.warning("audio recv error, will reconnect: %s", exc)
                return
            if not data:
                logger.warning("audio link closed by device, will reconnect")
                return
            self._dispatch_audio(data)

    def _dispatch_audio(self, data: bytes) -> None:
        """Route one recv chunk: reframe to the mic sink while recording, else discard.

        Args:
            data: the raw bytes just read from the audio socket.

        The arm check, the framing, and the sink snapshot all happen under _record_lock
        so they see a consistent (recording, sink, framer) triple even as start/stop run
        on another thread. The sink is then invoked OUTSIDE the lock: the framer is
        mutable shared state that must be touched under the lock, but the sink call is an
        external callback that should not run while a mutex is held. Snapshotting the
        sink locally means a stop_recording that nulls _mic_sink between the unlock and
        the call cannot turn it into a None call -- the packets from this chunk are simply
        delivered to the sink that was armed when they were framed.

        When not recording the bytes are dropped, exactly as the M1 keep-alive did: they
        are read (to detect disconnects and keep the buffer clear) but not interpreted.
        """
        with self._record_lock:
            if not self._recording:
                # Idle bytes: read to keep the link healthy, but nothing to interpret.
                return
            # Reframe under the lock (the framer is shared mutable state), and snapshot
            # the sink so the call below cannot race a concurrent disarm to None.
            packets = self._uplink_framer.feed(data)
            sink = self._mic_sink
        if sink is None:
            # Defensive: recording was true so a sink should be set, but never call None.
            return
        # Hand each raw Opus packet to the sink outside the lock. The sink is contracted
        # to be prompt and non-reentrant, so this cannot stall the read loop for long.
        for packet in packets:
            sink(packet)

    def _flush_close(self, sock: socket.socket) -> None:
        """Close a device socket without losing the device's final frame.

        Args:
            sock: the socket to close.

        The hazard: the device keeps pushing 0x25 status about once a second, so a socket
        almost always has unread receive data. On Linux, calling close() with unread
        rx sends a TCP RST rather than a clean FIN, and on real hardware that RST was
        observed to drop the device's last in-flight frame -- which for a control
        socket meant a final "searchlight off" command never landed. The fix
        (development plan section 9) is to wait close_flush_ms so any in-flight bytes
        arrive, drain the receive buffer, and only then close, so the teardown is a
        clean FIN with nothing unread.

        The drain is deliberately non-blocking. After the grace wait the socket is put
        in non-blocking mode and read in a loop that treats BlockingIOError (buffer now
        empty) as the ordinary, expected exit -- so draining cannot itself hang if the
        device happens to keep sending. An empty recv() (EOF) or any OSError also ends
        the drain, because in both cases there is nothing left worth reading. The whole
        drain sits in a try whose finally always calls close(): even if draining
        raises, the descriptor is released, and a close() that itself fails (double
        close, already-reset peer) is swallowed because at teardown it is harmless.
        Using stop_evt.wait for the grace period rather than time.sleep means that
        during shutdown -- when the Event is already set -- the wait returns instantly,
        while during a mid-run reconnect (Event not set) it still honors the full
        close_flush_ms; one primitive serves both cases correctly.
        """
        try:
            # Give in-flight bytes time to arrive. Using stop_evt.wait means this is
            # not an unconditional sleep -- but during shutdown stop is already set,
            # so it returns at once; the value still bounds the wait during a
            # mid-run reconnect where stop is NOT set.
            self._stop_evt.wait(self._cfg.close_flush_ms / 1000.0)
            # Drain whatever is now buffered without blocking: switch to non-blocking
            # and read until the buffer reports empty (or the peer is gone).
            sock.setblocking(False)
            while True:
                try:
                    if not sock.recv(_RECV_CHUNK):
                        # EOF: peer already closed, nothing left to drain.
                        break
                except (BlockingIOError, socket.timeout):
                    # Receive buffer is empty -- the intended, common exit.
                    break
                except OSError:
                    # Socket already broken; there is nothing to drain, fall through.
                    break
        finally:
            # Always close, even if draining raised, so we never leak a descriptor.
            try:
                sock.close()
            except OSError:
                # A double close or already-reset socket is harmless at this point.
                pass
