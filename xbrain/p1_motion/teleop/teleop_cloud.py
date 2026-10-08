"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: teleop_cloud.py
Brief: MOT-PM-22 teleop_cloud (550) exit-layer veto + zero-vy + link-down zero-vel

Description:
The exit-layer rules for cloud tel-op (12 S4.1 row 7b, S4.7.4, S6A.4.2).

*** THIS MODULE HAS NO PRODUCTION CALLER, AND THAT IS DELIBERATE.
12 S12 carries teleop.cloud.enabled = null with the note "本期是否放行云端
遥控, 待甲方接口评审纪要", and the 终审 S33 ruling is literal: "评审未过则该
通路不存在". So the rules are written and tested, the channel is not wired,
and wiring it before the review would be 9.3's "为将来留口子" plus a safety
parameter invented by us (铁律 3). NEXT S11 carries it as a C-group item.
*** Do NOT read "no caller" as an oversight -- the repo has caught three
cases of "能力都在, 最后一段没接" (SW-28), and this is the other kind: a
documented planned deferral. The thing to check before wiring is the 纪要,
not this file.

Priority: 550. The full 12 S4.1 order is fence_guard 1000, rns_avoid 900,
teleop 600, teleop_cloud 550, relative_move 500. Below local teleop on
purpose -- S4.1 row 7b verbatim: "本地遥控与云端遥控同时在时, 本地赢(现场的
人看得见, 云端的人隔着视频)".
*** The numbers do NOT decide preemption against rns_avoid(900): that is
TR-RNS-1 (12 S4.2c.5), whose own words are "优先级数字管同时活跃谁赢, 不管人
来了让不让". A reader who reasons "550 < 900 so cloud can never take over"
has the wrong model.

*** 2026-10-08 订正(铁律 1): the paragraph above replaced one that argued from
nav2_proxy=600 and rns=700. Both are gone -- rns_avoid moved to 900 in v0.8
(2026-09-08, #20-1) and nav2_proxy was removed entirely in v0.9 (2026-09-29,
12 S4.6 whole-section tombstone; BehaviorSource no longer carries it). The old
text also contained the author's unresolved thinking out loud ("600 - typo?
Actually ... Wait --"), which read as if the ordering were still in doubt.
依据: BehaviorSource closed set (sources/arbiter_p1.py) and the 12 S4.1 table.

Three exit-layer rules DIFFERENT from local teleop:
  * wz clipped to wz_blind (bounded even for cloud tel-op that has
    no local situational awareness); LOCAL teleop's wz limited but
    NOT clipped to wz_blind -- the variant catches merging the two
    paths into one limiter. 12 S6A.4.2 is where the two sources part
    company, and rcg.BLIND_CLAMP_EXEMPT_SOURCES is the other half of it.
  * vy REJECTED (cloud has no side awareness of obstacles); local
    teleop may issue vy. 12 S4.1 row 7b verbatim: "本期不接受 vy(横移)".
  * cloud link down (last frame > 1 s ago) -> ZERO velocity IMMEDIATELY
    (not gradual decel; fail-safe direction)
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CloudTeleopFrame:
    vx_mps: float
    vy_mps: float
    wz_radps: float
    arrived_mono_ms: int


class CloudTeleopReject(RuntimeError):
    """A cloud teleop frame violated a hard constraint."""


def clamp_and_check(
    frame: CloudTeleopFrame,
    now_mono_ms: int,
    obstacle_avoid_max_mps: float = 0.5,
    wz_blind_radps: float = 0.5,
    link_timeout_ms: int = 1000,
) -> tuple:
    """Apply cloud-specific rules; return (vx, vy, wz).

    * vy != 0 -> reject entire frame (cloud lacks side-awareness)
    * vx clamped to obstacle_avoid.max_mps (no fast cloud motion)
    * wz CLIPPED to wz_blind (not limited: cloud is blind)
    * link stale -> zero-vel

    Variant: any impl that lets cloud vy > 0 through, or clips
    local-teleop wz to wz_blind (which is a cloud-only rule),
    breaks the separation."""
    # Link-down check FIRST.
    if now_mono_ms - frame.arrived_mono_ms > link_timeout_ms:
        return (0.0, 0.0, 0.0)

    if frame.vy_mps != 0.0:
        raise CloudTeleopReject(
            "cloud vy=%f rejected; cloud has no side-obstacle awareness"
            % frame.vy_mps)

    vx = max(-obstacle_avoid_max_mps,
             min(obstacle_avoid_max_mps, frame.vx_mps))
    wz = max(-wz_blind_radps, min(wz_blind_radps, frame.wz_radps))
    return (vx, 0.0, wz)
