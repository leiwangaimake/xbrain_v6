"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_ctrl_stamp.py
Brief: p4's cmd/chassis/ctrl body -> a full 11 S3.0 envelope, at the one publish point

Description:
The voice bypass postures (A02 prone / A03 stand) leave P4 on
cmd/chassis/ctrl (11 MI-5, S9.3.4). That key is a LOOSENING command and
S9.3.4 E-3 says it must pass the full S3.0 envelope check, no exemption.
quadruped enforces it: read_envelope refuses a frame without rid, src, seq,
ts, ts_sync or data, and acks E_SCHEMA with cmd_id "anonymous".

Why P4 must build the envelope itself rather than leave it to the relay:
chassis_relay CR-11 rebuilds envelopes, but for a BARE body it can only wrap
it -- it has no original envelope to copy rid from and deliberately does not
invent one (envelope_rebuild.cc wrap_bare: fabricating provenance would be a
payload judgement CRL-1 forbids). So a bare body goes through the relay and
is refused one hop later. Measured 2026-10-26 on .1.26: a bare
{v, cmd_id, action, origin} came out on rt/chassis/ctrl as
{v, ts, seq, src:"chassis_relay", data:{...}} -- no rid, no ts_sync -- and
quadruped answered rejected / E_SCHEMA. dcd44c7 had moved the posture to
the right key but with that bare body, so "站起来" was still refused.

What it does: wraps the body (the S9.3.4 fields cmd_id / action / origin)
as `data` in an envelope built by the shared encoder
(xbrain.common.envelope.encode), with this process's rid and boot, a
per-key seq from 0, src "p4_agent" (the only publisher 11 S2.2 lists for
the voice bypass on this key) and ts_sync false: P4 holds no clock
verdict, and false is S3.0's fail-safe value.

Boundaries:
  * only cmd/chassis/ctrl. Other keys keep their current shapes; changing
    them is each consumer's question, not this module's.
  * no rid -> no envelope -> the frame is NOT published (stamp returns
    None). Publishing it bare would only be refused by quadruped, while
    looking like it went out.
  * the body is not validated here: decision_to_publishes builds it from a
    closed set and quadruped checks it again.
"""
from __future__ import annotations

import json  # wire bytes
import threading  # seq is shared by the mic and cloud paths
import time  # ts (wall, S3.0 field) and mono
from typing import Any, Dict, Optional  # annotations

from xbrain.common.envelope.envelope import Envelope, encode  # shared encoder

#: The envelope src for this key: p4 is its voice-bypass publisher (11 S2.2).
SRC = "p4_agent"


class ChassisCtrlStamper:
    """Wraps cmd/chassis/ctrl bodies into S3.0 envelopes, one seq per key.

    Built once at startup with the process rid (None when XBRAIN_ROBOT_ID is
    unset) and the local boot id; stamp() is called from the publish point.
    """

    def __init__(self, rid: Optional[str], boot: str) -> None:
        """rid None disables the key (stamp returns None); boot is S3.0's
        first 8 hex of the kernel boot id, the domain of mono (CLK-C4)."""
        self._rid = rid or None  # empty string counts as unset
        self._boot = boot  # pairs with mono
        self._seq = 0  # S3.0 seq starts at 0 per key per process
        self._lock = threading.Lock()  # mic worker and cloud jobs both publish

    def stamp(self, body_bytes: bytes) -> Optional[bytes]:
        """The envelope bytes for one body, or None when it cannot be sent.

        None for: no rid, or a body that is not a JSON object (a caller bug;
        the envelope's data must be an object, 11 S3.0).
        """
        if self._rid is None:  # no robot id: quadruped would refuse it anyway
            return None
        try:  # the body decision_to_publishes serialised
            body: Dict[str, Any] = json.loads(body_bytes.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):  # not JSON
            return None
        if not isinstance(body, dict):  # data must be an object
            return None
        with self._lock:  # seq is per key, not per caller
            seq = self._seq  # this frame's seq
            self._seq += 1  # next frame's
        env = Envelope(v=1, rid=self._rid,  # S3.0 version, our robot
                       # WALL-CLOCK-OK(align): the S3.0 ts field; no interval reads it.
                       ts=time.time(),
                       mono=time.monotonic(), boot=self._boot,  # CLK-C4 pair
                       seq=seq, src=SRC, ts_sync=False, data=body)  # fail-safe sync
        return json.dumps(encode(env), ensure_ascii=False,
                          separators=(",", ":")).encode("utf-8")  # wire bytes
