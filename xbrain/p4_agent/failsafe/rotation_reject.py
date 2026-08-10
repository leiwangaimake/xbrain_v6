"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rotation_reject.py
Brief: CHK-1-31 A09-A12/C07 rotation refusal RJ-1 vs RJ-2 script split

Description:
When a rotation intent (A09-A12 spin-in-place / C07 look) is
rejected by the motion arbiter, the voice reply MUST split by
which E_ code came back:

  RJ-1  E_CAPABILITY (rotation_clearance not fitted)
        -> say 'rotation not available' + one actionable next step
  RJ-2  E_BUSY  (rotation_blocked; occ_count + r_check_m detail)
        -> quote occ_count and r_check_m literally from detail,
           never made up

The two scripts MUST be textually distinct so the operator can
tell them apart (assert a != b).

Pre-check: state/pose.yaw_capable == False -> route straight to
RJ-1 script WITHOUT publishing any cmd/motion/intent (the arbiter
would only reject anyway; publishing wastes a round-trip and
pollutes the audit log).

Audit: every rejection lands ONE warn-level row in commands table;
the rejection MUST NOT mutate pose / gait / task / PTZ state.
"""

from __future__ import annotations

from dataclasses import dataclass

from xbrain.common.errors import E_BUSY, E_CAPABILITY


RJ_1_TEMPLATE = ("旋转能力不可用: 建议改为原地拍照或申请人工现场维护")
RJ_2_TEMPLATE = ("旋转已阻塞: 检测到 {occ_count} 个障碍物在 "
                   "{r_check_m:.2f} 米内, 请让开或改为直行")


class RotationRejectShapeError(Exception):
    pass


@dataclass(frozen=True)
class RotationRejectResponse:
    """Voice reply + a flag telling the caller whether an audit row
    should land."""
    script: str
    script_kind: str        # 'RJ-1' or 'RJ-2'
    emit_cmd_motion_intent: bool
    audit_level: str        # 'warn'


def refuse_from_ack(ack_code: str, ack_detail: dict) -> RotationRejectResponse:
    """Given a rejection ack from the motion arbiter, produce the
    voice-side response. Missing detail fields raise so no template
    silently substitutes zero (which would look like a real reading)."""
    if ack_code == E_CAPABILITY:
        return RotationRejectResponse(
            script=RJ_1_TEMPLATE, script_kind="RJ-1",
            emit_cmd_motion_intent=False, audit_level="warn")
    if ack_code == E_BUSY:
        for k in ("occ_count", "r_check_m"):
            if k not in ack_detail:
                raise RotationRejectShapeError(
                    f"RJ-2 requires ack_detail[{k!r}] literally from arbiter")
        return RotationRejectResponse(
            script=RJ_2_TEMPLATE.format(
                occ_count=int(ack_detail["occ_count"]),
                r_check_m=float(ack_detail["r_check_m"])),
            script_kind="RJ-2",
            emit_cmd_motion_intent=False,
            audit_level="warn")
    raise RotationRejectShapeError(
        "unhandled rejection code %r; RJ-1/RJ-2 script split only "
        "covers %r + %r" % (ack_code, E_CAPABILITY, E_BUSY))


def precheck_yaw_capable(yaw_capable: bool) -> RotationRejectResponse:
    """When state/pose.yaw_capable == False, short-circuit BEFORE
    publishing cmd/motion/intent. The response mimics an RJ-1
    (capability-absent) so downstream logging looks the same."""
    if yaw_capable:
        raise RotationRejectShapeError(
            "precheck_yaw_capable called with yaw_capable=True; "
            "caller should not short-circuit in that case")
    return RotationRejectResponse(
        script=RJ_1_TEMPLATE, script_kind="RJ-1",
        emit_cmd_motion_intent=False, audit_level="warn")


def scripts_are_distinct() -> bool:
    """CHK-1-31 meta-check: RJ-1 and RJ-2 templates must differ
    textually so operators can tell them apart in the log."""
    return RJ_1_TEMPLATE != RJ_2_TEMPLATE
