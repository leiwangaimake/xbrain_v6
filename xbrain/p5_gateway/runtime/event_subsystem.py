"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: event_subsystem.py
Brief: async event subsystem (record.db + pipeline + uplink) behind a sync facade

Description:
The running p5 (voice-loop wiring) is sync/threaded: Zenoh subscriber callbacks
run on the Rust thread pool and the main loop is a blocking heartbeat. But the
record.db DAO is async (aiosqlite, 15 S9.1 S-1 forbids sync sqlite3). This class
bridges the two: it owns a dedicated asyncio loop on a daemon thread, opens
record.db + builds the pipeline/uplink there, and exposes SYNC submit methods that
the Zenoh callbacks call. Each submit is a run_coroutine_threadsafe fire-and-
forget, so the callback returns immediately (4.2: no await, no blocking in a Zenoh
callback) and the persist happens on the event thread.

ADDITIVE, never a precondition (like the HMI web server): if record.db cannot be
opened (missing path, disk error), the subsystem logs and stays DISABLED -- every
submit becomes a no-op, the in-memory HMI ring in the wiring keeps working, and
the voice loop is untouched. A broken event store must not take down motion/voice.

What it does NOT do: it is not the live event publisher (17 S3.5.0 -- producers put
straight to event/{sev}/{cat}). It persists what arrives, marks need_ack=0 events
delivered when the link was up, tracks acks, and runs a backfill pass when told the
cloud link came back. The Zenoh I/O (subscribe event/**, event/ack, publish
event/replay/**) is wired by the caller; this class supplies the coroutines.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Callable, Optional

_logger = logging.getLogger("xbrain.p5.event_subsystem")


class EventSubsystem:
    """Sync facade over the async event pipeline + record.db + uplink."""

    def __init__(self, rid: str, record_db_path: str, jsonl_path: str,
                 now_iso: Callable[[], str], now_mono: Callable[[], float],
                 backfill_rate_eps: float = 20.0) -> None:
        self._rid = rid
        self._db_path = record_db_path
        self._jsonl_path = jsonl_path
        self._now_iso = now_iso
        self._now_mono = now_mono
        self._backfill_rate_eps = backfill_rate_eps

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()
        self._enabled = False
        # Built inside the loop thread by _init.
        self._conns: list = []
        self._dao = None
        self._pipeline = None
        self._marker = None
        self._acks = None
        self._rate = None
        self._runner = None
        self._recon = None

    @property
    def enabled(self) -> bool:
        return self._enabled

    # -- lifecycle ------------------------------------------------------------

    def start(self, init_timeout_s: float = 5.0) -> bool:
        """Start the loop thread and open record.db. Returns True if the store
        opened (subsystem enabled), False if it degraded to no-op. Never raises --
        a store failure must not stop the caller."""
        self._thread = threading.Thread(
            target=self._run_loop, name="p5-event-loop", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=init_timeout_s)
        if not self._enabled:
            _logger.warning(
                "event subsystem DISABLED (record.db not opened at %s) -- "
                "persistence off, HMI ring unaffected", self._db_path)
        return self._enabled

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._init())
            self._enabled = True
        except Exception as exc:  # noqa: BLE001 -- degrade, never crash the caller
            _logger.warning("event subsystem init failed: %s: %s",
                            type(exc).__name__, exc)
            self._enabled = False
        finally:
            self._ready.set()
        if self._enabled:
            self._loop.run_forever()

    async def _init(self) -> None:
        # Imported here (inside the loop thread's first coroutine) so a machine
        # without aiosqlite still imports this module; only start() needs it.
        from xbrain.p5_gateway.event.pipeline import EventPipeline
        from xbrain.p5_gateway.persistence.base import (
            open_record_reader, open_record_writer,
        )
        from xbrain.p5_gateway.persistence.record_dao import RecordDao
        from xbrain.p5_gateway.persistence.schema_record import (
            ALL_RECORD_STATEMENTS,
        )
        from xbrain.p5_gateway.reconnect.replay import RateLimiter
        from xbrain.p5_gateway.uplink.cloud import (
            AckTracker, BackfillRunner, DeliveryMarker, ReconRunner,
        )

        # writer_normal creates the schema; writer_full + reader open the same
        # file (WAL lets three connections coexist, 15 S9.1 S-2).
        wn = await open_record_writer(self._db_path, full=False,
                                      ddl_statements=ALL_RECORD_STATEMENTS)
        wf = await open_record_writer(self._db_path, full=True)
        rd = await open_record_reader(self._db_path)
        self._conns = [wn, wf, rd]
        self._dao = RecordDao(wn, wf, rd, self._jsonl_path)
        await self._dao.init_cursors_from_table()   # SEQ-3: resume ch_seq
        self._pipeline = EventPipeline(self._dao)
        self._marker = DeliveryMarker(self._dao, self._now_iso)
        self._acks = AckTracker(self._dao, self._now_iso)
        # ONE rate limiter shared by backfill AND recon (RC-4): the two together
        # must not exceed rate_eps_total, so they draw from the same bucket.
        self._rate = RateLimiter(self._backfill_rate_eps)
        self._runner = BackfillRunner(
            self._dao, self._rid, self._put_replay, self._rate, self._now_iso)
        self._recon = ReconRunner(
            self._dao, self._rid, self._put_replay, self._rate, self._now_iso)

    def stop(self, timeout_s: float = 3.0) -> None:
        """Stop the loop + close connections. Best-effort; safe to call when never
        started or already disabled."""
        if self._loop is None:
            return
        if self._enabled:
            fut = asyncio.run_coroutine_threadsafe(self._close(), self._loop)
            try:
                fut.result(timeout=timeout_s)
            except Exception:  # noqa: BLE001
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)

    async def _close(self) -> None:
        for c in self._conns:
            try:
                await c.conn.close()
            except Exception:  # noqa: BLE001
                pass

    # -- sync submit facade (called from Zenoh callbacks) --------------------

    def submit_event(self, ev: dict, link_connected: bool):
        """Persist one already-normalised event (fire-and-forget). Returns the
        concurrent future (the wiring ignores it; a test may wait on it), or None
        when disabled. The caller still owns the HMI ring; this only persists +
        marks delivery."""
        if not self._enabled or self._loop is None:
            return None
        return asyncio.run_coroutine_threadsafe(
            self._process(ev, link_connected), self._loop)

    async def _process(self, ev: dict, link_connected: bool) -> None:
        try:
            out = await self._pipeline.process(ev)
            # need_ack=0 events delivered now iff the link was up at insert
            # (S3.5.1). Merged/degraded/dropped events are not marked here.
            if out.persisted and not out.merged and not out.degraded:
                await self._marker.after_persist(
                    ev.get("eid"), out.need_ack, link_connected)
        except Exception as exc:  # noqa: BLE001 -- a bad event must not kill the loop
            _logger.warning("event process error (%s): %s",
                            type(exc).__name__, exc)

    def submit_ack(self, eid: str, result: str) -> None:
        """An event/ack from the cloud -> mark the eid delivered. No-op disabled."""
        if not self._enabled or self._loop is None or not eid:
            return
        asyncio.run_coroutine_threadsafe(
            self._acks.on_ack(eid, result), self._loop)

    def trigger_backfill(self) -> None:
        """Cloud link came back -> run one backfill pass. No-op disabled."""
        if not self._enabled or self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._run_backfill(), self._loop)

    async def _run_backfill(self) -> None:
        try:
            res = await self._runner.run(now_mono=self._now_mono,
                                         sleep=asyncio.sleep)
            _logger.info("backfill pass %s: sent=%s", res["batch_id"],
                         res["sent"])
        except Exception as exc:  # noqa: BLE001
            _logger.warning("backfill error (%s): %s",
                            type(exc).__name__, exc)

    def send_recon_reqs(self) -> None:
        """Periodic recon (17 S3Y.3): build + publish one recon/req per channel so
        the cloud can report holes. No-op disabled or until the req publisher is
        wired (set_recon_req_publisher)."""
        if not self._enabled or self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._send_recon_reqs(), self._loop)

    async def _send_recon_reqs(self) -> None:
        try:
            reqs = await self._recon.build_reqs(self._now_mono)
            if self._recon_req_put is not None:
                for req in reqs:
                    self._recon_req_put("event/recon/req", req)
        except Exception as exc:  # noqa: BLE001
            _logger.warning("recon req error (%s): %s",
                            type(exc).__name__, exc)

    def submit_recon_rsp(self, rsp: dict) -> None:
        """A cloud event/recon/rsp -> compute the gap and resend it (17 S3Y.3).
        No-op disabled. A rsp with a stale/foreign req_id is discarded inside the
        ReconRunner, not here."""
        if not self._enabled or self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._on_recon_rsp(rsp), self._loop)

    async def _on_recon_rsp(self, rsp: dict) -> None:
        try:
            res = await self._recon.on_rsp(
                rsp, now_mono=self._now_mono, sleep=asyncio.sleep)
            _logger.info("recon rsp ch=%s: %s", rsp.get("channel"), res)
        except Exception as exc:  # noqa: BLE001
            _logger.warning("recon rsp error (%s): %s",
                            type(exc).__name__, exc)

    # The recon/req publisher is injected by the wiring (Zenoh put on
    # event/recon/req). Until wired, recon reqs go nowhere.
    _recon_req_put: Optional[Callable[[str, dict], object]] = None

    def set_recon_req_publisher(
            self, put_fn: Callable[[str, dict], object]) -> None:
        """Wire the Zenoh publisher for event/recon/req. put_fn(key, data) is a SYNC
        call invoked from the event loop thread."""
        self._recon_req_put = put_fn

    # The replay publisher is injected by the wiring (Zenoh put). Until wired,
    # a backfill's puts go nowhere -- set by set_replay_publisher.
    _replay_put: Optional[Callable[[str, dict], object]] = None

    def set_replay_publisher(self, put_fn: Callable[[str, dict], object]) -> None:
        """Wire the Zenoh publisher for event/replay/**. put_fn(key, data) is a
        SYNC call (the wiring's declare_publisher().put); it is invoked from the
        event loop thread."""
        self._replay_put = put_fn

    async def _put_replay(self, key: str, data: dict) -> None:
        # BackfillRunner awaits this; the actual Zenoh put is sync, so just call it.
        if self._replay_put is not None:
            self._replay_put(key, data)
