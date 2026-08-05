"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_device_link_audio.py
Brief: Unit tests for the M3 8519 audio send + record path on DeviceLink.

Description:
  Exercises the parts of the audio path that need neither a real device nor the ASGI
  server: send_audio's borrow-the-owned-socket write and its down-link failure, the
  start/stop recording arm/disarm plus their [40]/[41] side effects and rollback, and
  the inbound dispatch that reframes [40] uplink to the mic sink only while armed. A
  fake socket stands in for the device connection and a fake config avoids any real
  network, so no supervisor thread is started -- the socket handle and recording flags
  are driven directly, which is exactly the state the supervisor would publish. Run
  from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_device_link_audio.py
"""
from __future__ import annotations

from typing import List

import pytest

from services.payload.config import PayloadConfig
from services.payload.core.device_link import DeviceLink, DeviceLinkError

_MARKER = b"[40]"
_A = b"\x11\x22\x33"
_B = b"\xaa\xbb"


class _FakeSock:
    # Records everything written so a test can assert the exact frames sent; sendall
    # never blocks or fails, standing in for a healthy connected device socket.
    def __init__(self) -> None:
        self.sent: List[bytes] = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(bytes(data))


class _BrokenSock(_FakeSock):
    # A socket whose write always fails, to drive the mid-send OSError -> DeviceLinkError
    # path without needing a real link that breaks.
    def sendall(self, data: bytes) -> None:
        raise OSError("broken pipe")


def _link() -> DeviceLink:
    # A DeviceLink on default config with NO threads started; the tests set _audio_sock
    # and the recording flags by hand to mimic what the supervisor would have published.
    return DeviceLink(PayloadConfig())


def test_send_audio_writes_frames_in_order() -> None:
    # send_audio must write each frame back-to-back on the borrowed socket, in order.
    link = _link()
    sock = _FakeSock()
    link._audio_sock = sock  # type: ignore[attr-defined]  # simulate a connected link
    link.send_audio([b"[42]\x01", b"[42]\x02", b"[11]"])
    assert sock.sent == [b"[42]\x01", b"[42]\x02", b"[11]"]


def test_send_audio_raises_when_link_down() -> None:
    # With no published socket the link is down, so send must fail fast with the typed
    # error the handlers translate to a failure response -- never a silent drop.
    with pytest.raises(DeviceLinkError):
        _link().send_audio([b"[42]\x01"])


def test_send_audio_raises_on_write_failure() -> None:
    # A write that fails mid-send must surface as DeviceLinkError, not a raw OSError.
    link = _link()
    link._audio_sock = _BrokenSock()  # type: ignore[attr-defined]
    with pytest.raises(DeviceLinkError):
        link.send_audio([b"[42]\x01"])


def test_start_recording_arms_and_sends_40() -> None:
    # start_recording must register the sink, flip the recording flag, and emit exactly
    # the [40] start command on the wire.
    link = _link()
    sock = _FakeSock()
    link._audio_sock = sock  # type: ignore[attr-defined]
    sink = lambda pkt: None
    link.start_recording(sink)
    assert link._recording is True  # type: ignore[attr-defined]
    assert link._mic_sink is sink  # type: ignore[attr-defined]
    assert sock.sent == [_MARKER]  # [40] == the record marker bytes


def test_start_recording_rolls_back_when_send_fails() -> None:
    # If [40] cannot be sent (link down), the arm must be undone so nothing is left
    # wired to a stream that will never arrive, and the caller sees the error.
    link = _link()  # no _audio_sock -> send_audio raises inside start_recording
    with pytest.raises(DeviceLinkError):
        link.start_recording(lambda pkt: None)
    assert link._recording is False  # type: ignore[attr-defined]
    assert link._mic_sink is None  # type: ignore[attr-defined]


def test_stop_recording_disarms_and_sends_41() -> None:
    # stop_recording must clear the recording state and emit [41]; the disarm must hold
    # even though [41] is only best-effort.
    link = _link()
    sock = _FakeSock()
    link._audio_sock = sock  # type: ignore[attr-defined]
    link.start_recording(lambda pkt: None)
    sock.sent.clear()  # drop the [40] so we assert only the stop side effect
    link.stop_recording()
    assert link._recording is False  # type: ignore[attr-defined]
    assert link._mic_sink is None  # type: ignore[attr-defined]
    assert sock.sent == [b"[41]"]


def test_stop_recording_swallows_send_failure() -> None:
    # If the link is already down, [41] cannot be sent but the disarm must still succeed
    # without raising -- there is nothing to stop on a dead link.
    link = _link()
    link._recording = True  # type: ignore[attr-defined]  # armed, but link now down
    link._mic_sink = lambda pkt: None  # type: ignore[attr-defined]
    link.stop_recording()  # must not raise
    assert link._recording is False  # type: ignore[attr-defined]


def test_dispatch_routes_frames_to_sink_while_recording() -> None:
    # While armed, an inbound [40]-framed stream must be reassembled and each Opus packet
    # handed to the sink; the final open packet stays buffered (needs a following marker).
    link = _link()
    got: List[bytes] = []
    link._recording = True  # type: ignore[attr-defined]
    link._mic_sink = got.append  # type: ignore[attr-defined]
    link._dispatch_audio(_MARKER + _A + _MARKER + _B + _MARKER)  # type: ignore[attr-defined]
    assert got == [_A, _B]


def test_dispatch_discards_when_not_recording() -> None:
    # When not armed the same bytes must be dropped, not framed -- idle reads exist only
    # to keep the link healthy, exactly as the M1 keep-alive discarded them.
    link = _link()
    got: List[bytes] = []
    link._mic_sink = got.append  # type: ignore[attr-defined]  # sink set but not armed
    link._dispatch_audio(_MARKER + _A + _MARKER)  # type: ignore[attr-defined]
    assert got == []


def test_clear_recording_ends_session() -> None:
    # _clear_recording (called on stop and on an audio-link teardown) must disarm so a
    # dropped link ends recording -- making "link up" a precondition of "recording".
    link = _link()
    link._recording = True  # type: ignore[attr-defined]
    link._mic_sink = lambda pkt: None  # type: ignore[attr-defined]
    link._clear_recording()  # type: ignore[attr-defined]
    assert link._recording is False  # type: ignore[attr-defined]
    assert link._mic_sink is None  # type: ignore[attr-defined]
