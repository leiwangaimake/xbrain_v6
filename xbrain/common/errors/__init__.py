"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The E_* closed set, exported for every XBRAIN runtime process

Description:
CLAUDE.md 3.5 forbids E_* string literals outside this package. The reason is
concrete: a code spelled slightly differently in two processes compiles, runs,
and only surfaces during integration, when the cloud client cannot branch on it.

Every name here is derived from codes.yaml at import time, which is itself
generated from 11 S13.4~S13.15. Nothing is typed twice. The deployed C++ header
common/errors/errors.h comes from the same file (CFG-CM-3), so the two languages
cannot drift apart either.

*** Out-of-set values raise. 11 S13.6 requires it in so many words: no silent
pass-through, no "interpret the unknown value as something close". A parser that
falls back to E_INTERNAL on an unrecognised code turns a contract violation into
a plausible-looking log line, and the cloud can no longer tell them apart.
"""

import os
from typing import Dict, FrozenSet, NamedTuple

from .exceptions import UnknownErrorCode

_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "codes.yaml")

# retryable, per 11 S13.2. That section states the column is a safety item, not a
# UX one -- treating E_LOCKED (HES not cleared) as retryable makes a cloud client
# resend motion commands at 10 Hz.
RETRY_YES = "yes"
RETRY_CONDITIONAL = "conditional"
RETRY_NO = "no"
RETRY_NA = "n_a"
_RETRY_VALUES = frozenset({RETRY_YES, RETRY_CONDITIONAL, RETRY_NO, RETRY_NA})

# detail, per EC-3.
_DETAIL_VALUES = frozenset({"required", "implied", "unspecified"})


class ErrorCode(NamedTuple):
    """One row of 11 S13.4~S13.15."""

    code: str
    group: str
    retryable: str
    detail: str
    meaning: str


def _build(d: Dict[str, str], lineno: int) -> ErrorCode:
    if d.get("retryable") not in _RETRY_VALUES:
        raise ValueError(f"{_YAML}:{lineno}: {d.get('code')} has retryable="
                         f"{d.get('retryable')!r}, not one of {sorted(_RETRY_VALUES)}")
    if d.get("detail") not in _DETAIL_VALUES:
        raise ValueError(f"{_YAML}:{lineno}: {d.get('code')} has detail="
                         f"{d.get('detail')!r}, not one of {sorted(_DETAIL_VALUES)}")
    return ErrorCode(code=d["code"], group=d["group"], retryable=d["retryable"],
                     detail=d["detail"], meaning=d.get("meaning", ""))


def _load() -> Dict[str, ErrorCode]:
    """Parse codes.yaml without a yaml dependency.

    This package is imported by every runtime process, including ones that start
    before any virtualenv is guaranteed, so it deliberately has no third-party
    imports. The format is fixed and generated, so a five-key block reader is
    enough -- and it raises on any line it does not recognise rather than
    skipping it. A loader that skips what it cannot parse silently shrinks the
    closed set, which is the one failure this module exists to prevent.
    """
    out: Dict[str, ErrorCode] = {}
    cur: Dict[str, str] = {}
    in_codes = False
    with open(_YAML, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = "" if raw.lstrip().startswith("#") else raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            if line.rstrip() == "codes:":
                in_codes = True
                continue
            if not in_codes:
                continue
            if line.lstrip().startswith("- code:"):
                if cur:
                    out[cur["code"]] = _build(cur, lineno)
                cur = {"code": line.split(":", 1)[1].strip()}
                continue
            if ":" not in line:
                raise ValueError(f"{_YAML}:{lineno}: unparsable line {line!r}")
            key, val = line.split(":", 1)
            cur[key.strip()] = val.strip().strip('"')
    if cur:
        out[cur["code"]] = _build(cur, -1)
    if not out:
        raise ValueError(f"{_YAML}: no codes parsed")
    return out


_CODES: Dict[str, ErrorCode] = _load()

#: The closed set itself. !! Never write its size into code or comments (3.7).
ALL_CODES: FrozenSet[str] = frozenset(_CODES)

# Bind every code as a module attribute so callers write E_TIMEOUT, not "E_TIMEOUT".
globals().update({c: c for c in _CODES})

__all__ = ["ALL_CODES", "ErrorCode", "UnknownErrorCode", "info", "retryable",
           "detail_requirement", "is_failure", "RETRY_YES", "RETRY_CONDITIONAL",
           "RETRY_NO", "RETRY_NA", *sorted(_CODES)]


def info(code: str) -> ErrorCode:
    """The full row for a code. Raises UnknownErrorCode outside the set."""
    try:
        return _CODES[code]
    except KeyError:
        raise UnknownErrorCode(code, sorted(ALL_CODES)) from None


def retryable(code: str) -> str:
    """One of RETRY_YES / RETRY_CONDITIONAL / RETRY_NO / RETRY_NA."""
    return info(code).retryable


def detail_requirement(code: str) -> str:
    """One of 'required' / 'implied' / 'unspecified' -- EC-3."""
    return info(code).detail


def is_failure(code: str) -> bool:
    """True for every code that is not a success.

    EC-1: a rejected result must carry code != OK. E_DUPLICATE is an idempotent
    hit -- the contract says to treat it as success, so it is not a failure here
    either, and its retryable is n_a for the same reason.
    """
    return info(code).retryable != RETRY_NA
