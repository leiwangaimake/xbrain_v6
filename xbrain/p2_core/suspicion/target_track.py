"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: target_track.py
Brief: BIZ-P2-15 -- target tracking ledger + lost-confirm frames

Description:
14 S6.3 target ledger. Each tracked object has:
  * track_id (int)
  * last_hit_mono_ms (monotonic clock; used for TTL)
  * seen_frames_since_lost (counter; when a track re-appears)
  * lost_confirm_frames (14 S6.3 default 15; consecutive absent frames
    before track is confirmed lost)

TTL rule: track_ttl_s (default 30 s) is the max wall a lost track
remains in the ledger; after TTL passes it is EVICTED (its next
appearance would be a new track_id).

Fence intersection (14 S6.5) is a separate geometry module; this
ledger only tracks presence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Set


@dataclass
class TrackEntry:
    track_id: int
    last_hit_mono_ms: int         # monotonic ms of last positive frame
    consecutive_absent: int = 0   # frames since last hit
    confirmed_lost: bool = False


@dataclass
class TargetLedger:
    """The per-P2 tracking ledger. Not thread-safe (main-thread only)."""
    lost_confirm_frames: int      # doc default 15
    track_ttl_ms: int             # doc default 30_000
    tracks: Dict[int, TrackEntry] = field(default_factory=dict)

    def observe(self, track_id: int, now_mono_ms: int) -> None:
        """Register a positive detection for track_id."""
        entry = self.tracks.get(track_id)
        if entry is None:
            self.tracks[track_id] = TrackEntry(
                track_id=track_id,
                last_hit_mono_ms=now_mono_ms,
                consecutive_absent=0,
                confirmed_lost=False,
            )
            return
        entry.last_hit_mono_ms = now_mono_ms
        entry.consecutive_absent = 0
        # If it was confirmed lost, re-appearance clears the flag --
        # the caller may want to emit a re-acquired event.
        entry.confirmed_lost = False

    def tick_frame_absent(self, absent_ids: List[int]) -> List[int]:
        """A frame arrived; the given track_ids were NOT in it.

        Increments consecutive_absent for each. Returns the list of
        track_ids that just crossed the lost_confirm_frames threshold
        (i.e., newly confirmed lost this tick). Caller emits any
        target_lost events for them."""
        newly_lost: List[int] = []
        for tid in absent_ids:
            entry = self.tracks.get(tid)
            if entry is None:
                continue
            if entry.confirmed_lost:
                continue
            entry.consecutive_absent += 1
            if entry.consecutive_absent >= self.lost_confirm_frames:
                entry.confirmed_lost = True
                newly_lost.append(tid)
        return newly_lost

    def evict_ttl(self, now_mono_ms: int) -> List[int]:
        """Remove tracks whose last_hit_mono_ms is > track_ttl_ms ago
        (regardless of confirmed_lost status). Returns evicted IDs."""
        evicted = []
        threshold = now_mono_ms - self.track_ttl_ms
        for tid, entry in list(self.tracks.items()):
            if entry.last_hit_mono_ms < threshold:
                del self.tracks[tid]
                evicted.append(tid)
        return evicted

    def active_ids(self) -> Set[int]:
        return {tid for tid, e in self.tracks.items() if not e.confirmed_lost}
