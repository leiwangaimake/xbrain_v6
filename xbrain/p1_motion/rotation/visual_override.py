"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: visual_override.py
Brief: CHK-1-20 rotation permit unique bypass allow_visual_override + RC-D7 no-toggle guard

Description:
12 §6A.7 defines the SINGLE bypass path for the rotation permit:
`allow_visual_override`. All FOUR conditions must be true:

  V-1  operator has explicit RTSP camera feed
  V-2  operator's session is authenticated + not restricted
  V-3  no active alarm state (e.g. fire event) that requires
       the robot to stay put
  V-4  the bypass command carries a distinct detail.kind =
       'rotation_visual_override' (never conflated with 'user'
       teleop)

RC-D7 is the accompanying static guard: NO OTHER 'skip permit'
toggle may exist. In particular, a `rotation_clearance.enabled=false`
config knob is FORBIDDEN. This is a CLAUDE.md 3.6 rule -- disabling
a safety check via config is not a supported operation.
"""

from __future__ import annotations


DETAIL_KIND = "rotation_visual_override"


class VisualOverrideDenied(Exception):
    """One of V-1..V-4 was not satisfied."""


def check_all_four(operator_has_rtsp: bool,
                    session_authenticated_and_normal: bool,
                    no_active_alarm: bool,
                    detail_kind: str) -> None:
    """Any one of V-1..V-4 missing -> raise. All four -> ok."""
    if not operator_has_rtsp:
        raise VisualOverrideDenied("V-1 no RTSP feed")
    if not session_authenticated_and_normal:
        raise VisualOverrideDenied("V-2 session not authenticated / restricted")
    if not no_active_alarm:
        raise VisualOverrideDenied("V-3 active alarm blocks rotation")
    if detail_kind != DETAIL_KIND:
        raise VisualOverrideDenied(
            f"V-4 detail.kind must be {DETAIL_KIND!r}, got {detail_kind!r}")


class ForbiddenClearanceToggle(Exception):
    """RC-D7: config carries a forbidden 'disable rotation permit' key."""


FORBIDDEN_KEYS = frozenset({
    "rotation_clearance.enabled",
    "rotation.permit.disabled",
    "rotation.clearance.enabled",
})


def rc_d7_scan(config: dict) -> None:
    """Walk the config; if any FORBIDDEN_KEYS is present at any
    depth, raise. This is a startup check, not a runtime one."""

    def _walk(obj, path=""):
        if isinstance(obj, dict):
            for k, v in obj.items():
                new_path = f"{path}.{k}" if path else k
                # RC-D7 keys may appear at any nesting depth; check
                # every dot-suffix of the current path.
                parts = new_path.split(".")
                for i in range(len(parts)):
                    suffix = ".".join(parts[i:])
                    if suffix in FORBIDDEN_KEYS:
                        raise ForbiddenClearanceToggle(
                            f"config key {new_path!r} is forbidden (RC-D7)")
                _walk(v, new_path)
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                _walk(v, f"{path}[{i}]")

    _walk(config)
