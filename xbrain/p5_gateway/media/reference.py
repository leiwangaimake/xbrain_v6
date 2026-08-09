"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: reference.py
Brief: GWY-P5-23 media reference (event.media[] + lifecycle decoupled from delivery)

Description:
17 S20 events carry a media[] array of URIs (mostly to captured
photos / short clips). These media files live on the local
filesystem AND may be uploaded to cloud via FTP (see GWY-P5-09).

Lifecycle rules:
  * media file is CREATED before the event that references it is
    submitted -- otherwise HMI shows a broken link
  * media file's local lifecycle is INDEPENDENT of event delivery;
    a photo may live 30 days locally even though its event was
    delivered 5 minutes after capture
  * cloud upload is best-effort; failure does NOT block delivery
"""

from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class MediaRef:
    """A single media reference. path is local; url is optional
    cloud location filled in after upload."""
    path: str
    kind: str        # 'photo' / 'video' / 'audio'
    bytes_size: int
    url: str = ""


class MediaFileMissing(Exception):
    """File was expected to exist before the event was submitted."""


def verify_file_exists_before_submit(ref: MediaRef) -> None:
    """Precondition check: HMI should never see a broken link."""
    if not os.path.exists(ref.path):
        raise MediaFileMissing(ref.path)


VALID_MEDIA_KINDS = frozenset({"photo", "video", "audio"})


def validate_kind(kind: str) -> None:
    if kind not in VALID_MEDIA_KINDS:
        raise ValueError(f"kind {kind!r} not in {sorted(VALID_MEDIA_KINDS)}")
