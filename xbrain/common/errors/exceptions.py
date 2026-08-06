"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: exceptions.py
Brief: Exception types for the shared error-code layer

Description:
CLAUDE.md 4.5 forbids raising bare Exception and forbids bare except. Every
failure this package can produce gets a named type so callers catch exactly what
they mean to handle.

XbrainError carries the closed-set code rather than a free-form string, so an
exception crossing a process boundary still maps onto 11 S13 without anyone
re-deriving it.
"""

from typing import Optional, Sequence


class XbrainError(Exception):
    """Base for every error this system raises deliberately.

    * 11 S8.13.5 is titled "错误映射(网关唯一实现点)" -- mapping a failure onto a
    closed-set code is the gateway's job, so AI services returning free-form
    detail are not in violation. Do not push E_* down into services/ to satisfy
    this type; construct it at the gateway instead.
    """

    def __init__(self, code: str, message: str = "", detail: Optional[dict] = None):
        self.code = code
        self.detail = detail or {}
        super().__init__(f"{code}: {message}" if message else code)


class UnknownErrorCode(XbrainError):
    """Raised when a value outside the closed set is used as a code.

    *** 11 S13.6 requires this to raise rather than degrade. Two tempting
    implementations are both forbidden: passing the unknown value through, and
    mapping it to E_INTERNAL. Either one converts a contract violation into
    something that looks like an ordinary failure, and the cloud client -- which
    branches on the code -- can no longer tell them apart.
    """

    def __init__(self, bad: str, known: Sequence[str]):
        self.bad = bad
        # The message names where the closed set lives so a reader can go look.
        # It deliberately does not print the set: that would put a count into
        # logs, and counts are what rots (CLAUDE.md 3.7).
        super().__init__(
            "E_INTERNAL",
            f"value {bad!r} is not in the E_* closed set defined by "
            f"11 S13.4~S13.15; see xbrain/common/errors/codes.yaml",
        )


class ClosedSetViolation(XbrainError):
    """Raised when any closed set -- not just error codes -- gets a value outside it.

    Shared with xbrain/common/enums/ so callers catch one type for the family.
    """

    def __init__(self, set_name: str, bad: str):
        self.set_name = set_name
        self.bad = bad
        super().__init__("E_SCHEMA", f"value {bad!r} is outside closed set {set_name!r}")
