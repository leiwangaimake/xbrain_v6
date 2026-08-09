"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: vsftpd.py
Brief: GWY-P5-09/24 disk-watermark FTP + vsftpd (chroot RO + rate limit + QoS DSCP)

Description:
17 S21 disk-based deliverables (media / logs) are served over FTP.
Discipline:

  * vsftpd binds each port explicitly (NET-C9 discipline)
  * chroot to /var/xbrain/deliver so an FTP client cannot walk out
  * READ-ONLY (no PUT / DELETE / RENAME)
  * per-connection rate limit (bytes per second)
  * disk watermark: soft(80%) / hard(90%). At soft, refuse new writes
    from p5 side. At hard, refuse new deliverables AND emit
    health_critical.

17 S22 QoS: three planes get DSCP tags:
  control -> EF (46)
  data    -> AF41 (34)
  media   -> AF31 (26)
DSCP set at socket create; do NOT rely on downstream routers.
"""

from __future__ import annotations

from dataclasses import dataclass


DSCP_EF   = 46   # control
DSCP_AF41 = 34   # data
DSCP_AF31 = 26   # media


DSCP_BY_PLANE = {
    "control": DSCP_EF,
    "data":    DSCP_AF41,
    "media":   DSCP_AF31,
}


class UnknownPlane(Exception):
    pass


class FtpWriteForbidden(Exception):
    """Attempted a write verb on read-only FTP."""


READ_ONLY_VERBS = frozenset({"USER", "PASS", "LIST", "RETR",
                              "CWD", "PWD", "TYPE", "QUIT", "MDTM"})


def dscp_for(plane: str) -> int:
    if plane not in DSCP_BY_PLANE:
        raise UnknownPlane(plane)
    return DSCP_BY_PLANE[plane]


def check_verb_read_only(verb: str) -> None:
    """Refuse write verbs; enforce read-only chroot policy."""
    if verb not in READ_ONLY_VERBS:
        raise FtpWriteForbidden(
            f"verb {verb!r} not allowed on read-only FTP")


@dataclass(frozen=True)
class DiskWatermarkPolicy:
    """Percent-of-capacity thresholds. soft < hard."""
    soft_pct: float
    hard_pct: float

    def __post_init__(self) -> None:
        if not (0 < self.soft_pct < self.hard_pct <= 100):
            raise ValueError(
                f"invalid watermark: soft={self.soft_pct}, "
                f"hard={self.hard_pct}")


class DiskWatermark:
    """Serves as a decision oracle: 'may I write a new file?'."""

    def __init__(self, policy: DiskWatermarkPolicy) -> None:
        self.policy = policy

    def allow_write(self, used_pct: float) -> bool:
        return used_pct < self.policy.soft_pct

    def health_critical(self, used_pct: float) -> bool:
        return used_pct >= self.policy.hard_pct
