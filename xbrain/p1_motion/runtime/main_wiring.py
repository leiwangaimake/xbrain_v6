"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p1_motion voice-loop MVP wiring -- CHS-A client to chassis_stub :30004

Description:
Minimum-viable p1 for the voice-loop smoke test:

  * open RT + GEN sessions
  * subscribe cmd/motion/intent from p4
  * subscribe cmd/motion/factor (would come from p2 arbiter; MVP
    just observes)
  * on each cmd/motion/intent, produce a single-frame cmd_vel and
    forward it to chassis_stub :30004 as a CHS-A APDU frame
    (16-byte header + JSON ASDU)

Real 20 Hz ctrl_loop + rns_avoid + speed_gate + rotation_permit
live in xbrain/p1_motion/{ctrl_loop.py,rns/,gate/,rotation/} and
stay untouched by this MVP. The purpose here is: 'when p4 says
向前 3 米, p1 emits a CHS-A frame the chassis_stub prints.'
"""

from __future__ import annotations

import json
import logging
import os
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional


_logger = logging.getLogger("xbrain.p1.wiring")


CMD_MOTION_INTENT_TOPIC = "cmd/motion/intent"
CMD_MOTION_FACTOR_TOPIC = "cmd/motion/factor"
# 11 S2.2 / S12A.9.7: P1 arbitrates the local teleop inputs and is the sole
# publisher of state/teleop. cmd/teleop carries the HMI virtual stick (S12A.9.5);
# gamepad / local keyboard arrive from teleop_input, which is not built yet, so
# those two simply never appear in sources[].
CMD_TELEOP_TOPIC = "cmd/teleop"
CMD_ESTOP_TOPIC = "cmd/estop"            # P1-21: soft-estop latch (14 S3.7)
STATE_TELEOP_TOPIC = "state/teleop"
TELEOP_PUBLISH_PERIOD_S = 1.0


@dataclass
class ChassisClientConfig:
    """All fields required (CLAUDE.md 3.1)."""
    host: str
    port: int
    connect_timeout_s: float
    retry_delay_s: float


class ChassisClient:
    """Simple CHS-A frame sender to chassis_stub. Auto-reconnect
    on send failure (chassis_stub may not be up when p1 starts)."""

    def __init__(self, cfg: ChassisClientConfig) -> None:
        self._cfg = cfg
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.frames_sent = 0
        self.connect_attempts = 0
        self.connect_failures = 0

    def _connect(self) -> bool:
        try:
            self.connect_attempts += 1
            s = socket.create_connection(
                (self._cfg.host, self._cfg.port),
                timeout=self._cfg.connect_timeout_s)
            self._sock = s
            _logger.info("p1 chassis connected %s:%d",
                         self._cfg.host, self._cfg.port)
            return True
        except OSError as exc:
            self.connect_failures += 1
            _logger.warning("p1 chassis connect fail: %s", exc)
            self._sock = None
            return False

    def send_apdu(self, asdu_json_dict: dict) -> bool:
        """Send one APDU frame. On failure, drop the socket and
        return False; caller retries on next intent."""
        with self._lock:
            if self._sock is None and not self._connect():
                return False
            asdu = json.dumps(asdu_json_dict,
                                ensure_ascii=False).encode("utf-8")
            header = b"\x00" * 12 + struct.pack(">I", len(asdu))
            try:
                self._sock.sendall(header + asdu)
                self.frames_sent += 1
                return True
            except OSError as exc:
                _logger.warning("p1 chassis send fail: %s", exc)
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
                return False

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None


def intent_to_apdu(intent_payload: dict) -> dict:
    """Compose a CHS-A ASDU JSON dict wrapping the intent. Real
    CHS-A ASDU carries {PatrolDevice: {Time, ...}} per 13 §2.2;
    this MVP wraps the p4 intent envelope inside it."""
    intent_id = intent_payload.get("intent_id", "?")
    text = intent_payload.get("text", "")
    return {
        "PatrolDevice": {
            "Time": time.strftime("%Y-%m-%d %H:%M:%S",
                                   time.localtime()),
            "Op": "IntentForward",
            "IntentId": intent_id,
            "Text": text,
        },
    }


def run_voice_loop_wiring(chassis_cfg: ChassisClientConfig,
                            stop_flag: dict,
                            heartbeat_period_s: float = 5.0) -> int:
    """Block until stop_flag truthy. Returns 0 on clean shutdown."""
    from xbrain.common.runtime.session_ctx import open_planes

    _logger.info("p1 wiring: opening RT + GEN sessions")
    client = ChassisClient(chassis_cfg)
    with open_planes(("rt", "gen")) as (rt, gen):
        _rt = rt   # keep RT session alive for future cmd_vel pub

        # P1-21 soft-estop latch. Its primary consumer is the 20 Hz ctrl_loop
        # (estop -> zero cmd_vel + stop_reason=soft_estop); here in the running
        # relative-move path the latch records the soft-estop and drives re-arm.
        from xbrain.p1_motion.runtime.estop_latch import P1EstopLatch
        estop_latch = P1EstopLatch()

        def _on_intent(sample) -> None:
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                _logger.warning("p1 malformed cmd/motion/intent")
                return
            # A new motion intent is the soft-estop re-arm key (14 S3.7 / U35:
            # estop, then "go forward 2m", then go). gate_intent clears the
            # latch and returns True; refusing motion here would block that
            # documented field behaviour.
            estop_latch.gate_intent()
            apdu = intent_to_apdu(d)
            ok = client.send_apdu(apdu)
            _logger.info("p1 forwarded intent -> chassis (ok=%s intent=%s)",
                         ok, d.get("intent_id"))

        intent_sub = gen.declare_subscriber(
            CMD_MOTION_INTENT_TOPIC, _on_intent)

        def _on_estop(sample) -> None:
            # RUST THREAD (CLAUDE.md 4.2): latch only, no publish/await. The
            # cmd/estop callback must return fast; the latch is read by the
            # ctrl_loop (when active) and the intent path above.
            estop_latch.on_estop(bytes(sample.payload))

        estop_sub = gen.declare_subscriber(CMD_ESTOP_TOPIC, _on_estop)
        _logger.info("p1 wiring: subscribed %s (P1-21 soft-estop latch)",
                     CMD_ESTOP_TOPIC)

        # Also subscribe cmd/motion/factor (log only for MVP).
        def _on_factor(sample) -> None:
            _logger.debug("p1 obs cmd/motion/factor (%d bytes)",
                          len(bytes(sample.payload)))
        factor_sub = gen.declare_subscriber(
            CMD_MOTION_FACTOR_TOPIC, _on_factor)

        # --- 11 S12A.9.7 teleop arbitration + state/teleop -------------------
        # P1 owns the teleop behaviour source, so it is the only process that
        # can say which input is driving. P3 reads sources[] as criterion 1 of
        # the recording arming gate (S12A.3), so this is published whether or
        # not any input exists -- "nobody is at the controls" is the answer that
        # gate needs, and silence would be indistinguishable from a lost link.
        from xbrain.p1_motion.teleop.state import TeleopTracker

        teleop_tracker = TeleopTracker()
        teleop_pub = gen.declare_publisher(STATE_TELEOP_TOPIC)

        def _on_teleop(sample) -> None:
            # RUST THREAD: decode and record only. The tracker holds plain
            # fields and the publish happens on the loop below (CLAUDE.md 4.2).
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                _logger.warning("p1 malformed cmd/teleop")
                return
            # S12A.9.5: the HMI sends cmd/teleop. `device` names the physical
            # input when the sender knows it; a sender that only says
            # source=hmi is recorded as keyboard_hmi, which is the S12A.9.7
            # name for that path. A cloud teleop frame is NOT one of the four
            # arbitrated sources -- it drives the separate teleop_cloud
            # behaviour source (550) -- so it is ignored here rather than
            # promoted into the local arbitration.
            device = d.get("device")
            if device is None and d.get("source") == "hmi":
                device = "keyboard_hmi"
            if device is None:
                return
            try:
                teleop_tracker.observe(
                    device, now_mono_ms=int(time.monotonic() * 1000),
                    deadman=bool(d.get("deadman", False)),
                    axes=d.get("axes") if isinstance(d.get("axes"), dict)
                    else None,
                    mark_edge=bool(d.get("mark", False)))
            except ValueError as exc:
                # An off-set device name: refused rather than arbitrated (the
                # S12A.9.7 set is closed and carries per-device timeouts).
                _logger.warning("p1 cmd/teleop: %s", exc)

        teleop_sub = gen.declare_subscriber(CMD_TELEOP_TOPIC, _on_teleop)
        _logger.info("p1 wiring: subscribed %s, publishing %s at %.0f Hz",
                     CMD_TELEOP_TOPIC, STATE_TELEOP_TOPIC,
                     1.0 / TELEOP_PUBLISH_PERIOD_S)

        # --- RTK GNSS cross-plane bridge (11 S1.1.6: p1 订 rt/gnss/*, 发 state/pose) ---
        # rtk_driver publishes FULL RT keys (xbrain/{rid}/rt/gnss/*); the general
        # plane uses bare keys (state/pose). p1 IS the cross-plane point, so it
        # subscribes the full RT key and republishes the bare GEN key. Needs rid;
        # if XBRAIN_ROBOT_ID is unset the bridge is skipped (the MVP voice loop
        # still runs) rather than forming an invalid key.
        from xbrain.common.envelope import read_local_boot_id
        from xbrain.p1_motion.path import gnss_pose

        rid = os.environ.get("XBRAIN_ROBOT_ID", "")
        boot = ""
        gnss_subs = []
        pose_pub = None
        clock_pub = None
        # Rust-thread callbacks store only; a dict assignment is atomic (CLAUDE.md
        # 4.2 forbids await/publish in a Zenoh callback). Publishing is done by the
        # loop thread below.
        gnss_cache = {"data": None}
        fix_cache = {"data": None}
        clock_cache = {"data": None}
        pose_seq = {"n": 0}
        clock_seq = {"n": 0}
        if rid:
            boot = read_local_boot_id()

            def _on_gnss(sample) -> None:
                try:
                    msg = json.loads(bytes(sample.payload).decode("utf-8"))
                    gnss_cache["data"] = msg.get("data")
                except Exception:      # noqa: BLE001
                    _logger.warning("p1 malformed rt/gnss/heading")

            def _on_fix(sample) -> None:
                try:
                    msg = json.loads(bytes(sample.payload).decode("utf-8"))
                    fix_cache["data"] = msg.get("data")
                except Exception:      # noqa: BLE001
                    _logger.warning("p1 malformed rt/gnss/fix")

            def _on_clock(sample) -> None:
                try:
                    msg = json.loads(bytes(sample.payload).decode("utf-8"))
                    clock_cache["data"] = msg.get("data")
                except Exception:      # noqa: BLE001
                    _logger.warning("p1 malformed rt/clock/status")

            # Hold sub handles in a long-lived list (CLAUDE.md 4.3: a dropped
            # declare_subscriber return silently unsubscribes on GC).
            gnss_subs.append(rt.declare_subscriber(
                "xbrain/%s/rt/gnss/heading" % rid, _on_gnss))
            gnss_subs.append(rt.declare_subscriber(
                "xbrain/%s/rt/gnss/fix" % rid, _on_fix))
            gnss_subs.append(rt.declare_subscriber(
                "xbrain/%s/rt/clock/status" % rid, _on_clock))
            pose_pub = gen.declare_publisher("state/pose")
            clock_pub = gen.declare_publisher("state/clock")
            _logger.info("p1 gnss bridge on: rid=%s (rt/gnss/heading -> state/pose)",
                         rid)
        else:
            _logger.warning("p1 gnss bridge OFF: XBRAIN_ROBOT_ID unset")

        try:
            last_hb = time.monotonic()
            last_clock = 0.0
            last_teleop = 0.0
            while not stop_flag.get("stop"):
                now = time.monotonic()
                if pose_pub is not None:
                    # 10 Hz state/pose from the latest GnssHeading. When heading is
                    # invalid the cache still holds the last L3 frame (H-2 freeze),
                    # so the published pose degrades gracefully, never goes silent.
                    ts_sync = bool((clock_cache["data"] or {}).get("sync", False))
                    pose = gnss_pose.assemble_pose(gnss_cache["data"], fix_cache["data"])
                    penv = gnss_pose.stamp_envelope(
                        pose, rid=rid, boot=boot, seq=pose_seq["n"],
                        src="p1_motion", ts_sync=ts_sync)
                    pose_pub.put(json.dumps(penv, ensure_ascii=False).encode("utf-8"))
                    pose_seq["n"] += 1
                    if now - last_clock >= 1.0:   # 1 Hz state/clock mirror (P1-13)
                        cs = gnss_pose.mirror_clock(clock_cache["data"])
                        cenv = gnss_pose.stamp_envelope(
                            cs, rid=rid, boot=boot, seq=clock_seq["n"],
                            src="p1_motion", ts_sync=cs["sync"])
                        clock_pub.put(
                            json.dumps(cenv, ensure_ascii=False).encode("utf-8"))
                        clock_seq["n"] += 1
                        last_clock = now
                # 11 S12A.9.7: state/teleop at 1 Hz (plus on change, which the
                # arbitration makes visible through active_source).
                if now - last_teleop >= TELEOP_PUBLISH_PERIOD_S:
                    try:
                        teleop_pub.put(json.dumps(
                            teleop_tracker.build_state(int(now * 1000)),
                            ensure_ascii=False).encode("utf-8"))
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p1 teleop state publish failed: %s", exc)
                    last_teleop = now
                if now - last_hb >= heartbeat_period_s:
                    _logger.info(
                        "p1 alive; chassis_frames=%d chassis_reconnects=%d "
                        "chassis_fails=%d pose_pub=%d",
                        client.frames_sent, client.connect_attempts,
                        client.connect_failures, pose_seq["n"])
                    last_hb = now
                time.sleep(0.1)
        finally:
            try:
                intent_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                estop_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            try:
                factor_sub.undeclare()
            except Exception:      # noqa: BLE001
                pass
            for _s in gnss_subs:
                try:
                    _s.undeclare()
                except Exception:      # noqa: BLE001
                    pass
            client.close()
    return 0
