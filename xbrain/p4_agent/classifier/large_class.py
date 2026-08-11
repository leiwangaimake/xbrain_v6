"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: large_class.py
Brief: 16 S5.2 layer-4 large-class + intra-class rules for device families

Description:
16 S5.2 layer 4 is "大类 + 类内规则: A-J 十大类 -> 类内意图". Layer 2
(keyword_matcher) only fires on an EXACT long-phrase keyword; anything the
operator phrases even slightly differently ("云台朝左", "把照明灯搞亮点",
"镜头往左转") misses every keyword and, with no layer-4 router, falls straight
to the layer-6 LLM (today a stub) -- so the command does nothing. That is the
"死板" the operator hit on the ORIN: only the verbatim phrase works.

This module fills layer 4 for the two DEVICE closed-classes whose intra-class
selection is DETERMINISTIC (no LLM needed, 16 S5.2 line "类内规则"):

  * E class (PTZ, 18-B): pan/tilt/zoom/scan/speed.
  * D class (payload, 18-A): lights on/off/brightness, strobe on/off/mode,
    volume, siren.

It runs ONLY after layer 2 missed (priority_chain layer 4), so it never
overrides an exact keyword decision -- it only rescues the misses. It returns
an intent id (the slots are filled downstream by the same ptz_slots /
payload_slots parsers), or None when the text carries no device signal (then
the chain continues to layer 5/6).

Disambiguation rule (operator's 2026-08-11 ruling, the core of this module):
  * A PTZ SUBJECT / prefix ("云台" and its aliases 镜头/摄像头/布控球, plus the
    operator-confirmed ASR mishear "平台") routes an overlapping direction word
    (左/右/转) to PTZ. WITHOUT that prefix the same word is the CHASSIS's
    (A09 turn_left / A10 turn_right / A13 speed all share 左转/右转/快一点) --
    so a bare direction is left to the chassis, never stolen by PTZ.
  * PTZ-ONLY vocabulary has no chassis twin, so it needs no prefix: tilt
    up/down, zoom (变焦/拉近/放大), scan (环视/扫), speed ("转速"). These route
    to PTZ even bare.

What this module does NOT do:
  * It does not touch layer 2 (exact keywords still win first).
  * It does not guess free-text intents (D08/D09 speak, out-of-family motion).
    Those stay with the keyword table / LLM.
  * It is NOT a safety gate. It only chooses an intent id; the auth/confirm
    gate and the T-PTZ capability reject still run downstream in the
    orchestrator, unchanged.

Pitfalls this file was written to avoid (each has a mutation test):
  * Bare "大一点/小一点" is ambiguous between PTZ zoom and volume -- the
    ambiguous zoom words require a PTZ subject; volume requires an audio
    subject; a bare "大一点" is claimed by neither.
  * "灯" substring-matches 爆闪灯 / 警灯 / 底盘灯 -- so strobe and siren are
    resolved BEFORE the bare-"灯" light branch.
  * Bare "快一点/慢一点" is the chassis A13, NOT ptz speed -- ptz speed
    requires the PTZ-only word "转速" (or a ptz subject).
"""
from __future__ import annotations

from typing import Optional

from xbrain.p4_agent.slots.ptz_slots import (
    parse_ptz_direction, parse_ptz_speed_level, parse_zoom_direction,
)


# ---------------------------------------------------------------------------
# E class (PTZ)
# ---------------------------------------------------------------------------

# PTZ subject / prefix cues. 云台 is primary; 镜头/摄像头/布控球 are the same
# device; 平台 is the operator-confirmed ASR mishear of 云台 on this deployment
# (2026-08-11 ORIN: "云台" is frequently transcribed "平台"). Presence of any
# of these is what promotes an OVERLAPPING direction word to PTZ.
# Why a curated tuple and not "contains 云*": the mishear set is empirical
# (平台), not derivable -- a fuzzy rule would also swallow unrelated words.
_PTZ_SUBJECT = ("云台", "镜头", "摄像头", "布控球", "平台")

# Zoom words with NO chassis twin -> route to PTZ even without a subject.
# 放大/缩小 a light makes no sense; these are unambiguously the camera. The
# AMBIGUOUS zoom words (大一点/小一点/近一点/远一点, which also mean volume)
# are deliberately excluded here -- they are only zoom WITH a ptz subject
# (handled by parse_zoom_direction under the subj gate below).
_ZOOM_UNAMBIGUOUS = ("变焦", "拉近", "拉远", "推近", "推远", "放大", "缩小")

# Scan vocabulary (PTZ-only; the chassis has no "环视/扫"), so it needs no
# subject. 环视/一圈/一周 name a full single-direction turn; the bare "扫" is
# a bounded left-right sweep. Both resolve to E07 (p2 reads scan_mode to pick
# orbit vs sweep) -- the class router only needs to know it is E07.
_SCAN_ORBIT = ("环视", "一圈", "一周")


def has_ptz_subject(text: str) -> bool:
    # A plain substring test is enough: the subject set is short and the words
    # do not appear as fragments of unrelated commands in the 18 command set
    # (there is no chassis/task word containing 云台/镜头/布控球). "平台" is the
    # one risk word, accepted because operators do not issue "平台"-anything
    # else by voice here. Public because the orchestrator's PTZ-prefix reclaim
    # (a 云台-prefixed command a chassis keyword stole) also needs it.
    return any(s in text for s in _PTZ_SUBJECT)


def resolve_ptz(text: str) -> Optional[str]:
    """Intra-class rule for the E (PTZ) family. Returns E01/E06/E07/E09 or
    None. Order matters: the PTZ-ONLY vocabularies (speed/scan/zoom, and
    tilt up/down) are checked before the OVERLAPPING left/right, and a
    left/right move needs a PTZ subject (else the chassis owns it)."""
    text = text or ""
    subj = has_ptz_subject(text)

    # The order below is load-bearing: the PTZ-only vocabularies (speed, scan,
    # unambiguous zoom, tilt up/down) are tried before the OVERLAPPING
    # left/right, and scan is tried before direction so a phrase carrying both
    # (云台向左环视一周) resolves as an orbit rather than a one-shot move.

    # E09 set_ptz_speed. "转速" is PTZ-only wording; the chassis speed intent
    # (A13) is worded 快一点/慢一点/全速 and never contains "转速". A ptz
    # subject also admits a speed phrase ("云台转慢点"). Bare 快一点 stays
    # chassis -- it is not seen here anyway (A13 catches it at layer 2).
    if "转速" in text or (subj and parse_ptz_speed_level(text) is not None):
        return "E09"

    # E07 ptz_scan. 环视/一圈/一周 -> a full orbit; 扫 -> a bounded sweep.
    # Both are PTZ-only, so no subject is required. (Checked before the
    # direction rule so "云台向左环视一周" is an orbit, not a left move.)
    if any(w in text for w in _SCAN_ORBIT) or "扫" in text:
        return "E07"

    # E06 ptz_zoom. Unambiguous zoom words route bare; the ambiguous ones
    # (大一点/小一点/近一点/远一点, which collide with volume) require a
    # ptz subject.
    if any(w in text for w in _ZOOM_UNAMBIGUOUS):
        return "E06"
    if subj and parse_zoom_direction(text) is not None:
        return "E06"

    # E01 ptz_move. allow_bare=False: a single-char 左/右/上/下 is too weak to
    # CLASSIFY (it fires on 一下 / 楼下 / 上午 ...). Only a multi-char cue
    # (向下/往上/朝左/左转/低头/抬高 ...) decides the class. The overlapping
    # direction words then split:
    #   * up/down have NO chassis twin -> route even without a subject.
    #   * left/right ARE the chassis (A09/A10) -> only route with a ptz
    #     subject; a bare-of-subject 向左 is left to the chassis keyword.
    direction = parse_ptz_direction(text, allow_bare=False)
    if direction is not None:
        if subj:
            return "E01"
        if direction in ("up", "down"):
            return "E01"
        # left/right without a subject: not PTZ (operator's rule).
    return None


# ---------------------------------------------------------------------------
# D class (payload: lights / strobe / siren / audio)
# ---------------------------------------------------------------------------

# Audio subject cues (D10 volume). "音量" is unambiguous; a bare "声" needs a
# magnitude word to count (so "说一声" is not a volume command). This anchor
# is what keeps the ambiguous "大一点/小一点" out of volume unless the operator
# actually named the audio (声音大一点), mirroring the zoom subject gate.
_AUDIO_SUBJECT = ("音量", "嗓门", "喇叭", "喊话器")
_AUDIO_MAG = ("大", "小", "调", "高", "低", "静音")

# Strobe / warning-light subject cues (D06/D07/D18). Checked BEFORE the bare
# "灯" light branch because "爆闪灯"/"警灯" both contain "灯" -- if the light
# branch ran first it would resolve "关爆闪灯" to D02 (light off) instead of
# D07 (strobe off). 红蓝 is the M20S red-blue warning pattern (00 S5.1.1).
_STROBE_SUBJECT = ("爆闪", "双闪", "警灯", "警示灯", "红蓝")
# Mode-switch cues take precedence over on/off within the strobe branch: the
# operator changing the pattern (换一种/第三种/样式) is D18, not a re-open.
_STROBE_MODE = ("换", "样式", "第三种", "另一种", "切换", "模式")

# Siren cues (D04/D05). A distinct device from the strobe light (the siren is
# sound, the strobe is light), so it gets its own subject set and branch.
_SIREN_SUBJECT = ("警笛", "警报")

# Illumination-light subject cues (D01/D02/D17). Bare "灯" is last-resort and
# only reached after strobe + siren are excluded above, so it never shadows
# them; 照明/补光灯/探照灯/前灯 are the explicit illumination devices.
_LIGHT_SUBJECT = ("照明", "补光灯", "探照灯", "前灯")

# Action verb groups shared across the light/strobe/siren branches. Kept as
# small closed sets (not a fuzzy match) so an unrelated verb cannot flip a
# device on/off -- the branch only fires when subject AND action co-occur.
_ACT_OFF = ("关", "灭", "熄", "别")
_ACT_ON = ("开", "打开", "启", "亮起", "来一")
# Brightness (D17) is checked before on/off: "太暗了"/"亮一点" is a level
# change on an already-on light, not an on/off toggle.
_ACT_BRIGHT = ("亮", "刺眼", "晃眼")       # brighter / too-bright (D17)
_ACT_DARK = ("暗",)                        # darker (D17)


def resolve_payload(text: str) -> Optional[str]:
    """Intra-class rule for the D (payload) family. Returns a D-intent id or
    None. Ordered audio -> strobe -> siren -> light so the broad "灯"
    substring does not shadow the strobe/警灯 devices."""
    text = text or ""

    # Branch order is the whole correctness story here (see the subject-set
    # comments above): audio first (its subjects are disjoint from lights),
    # then strobe and siren, and only last the bare-"灯" light branch -- so a
    # warning-light command is never mis-read as an illumination command.

    # D10 set_volume. Requires an audio subject, OR "声" plus a magnitude
    # word (so "声音大一点"/"声音小一点" count but "喊一声" does not). The
    # bare ambiguous "大一点" (also a zoom word) is intentionally NOT a
    # volume command without one of these audio anchors.
    if any(w in text for w in _AUDIO_SUBJECT):
        return "D10"
    if "声" in text and any(m in text for m in _AUDIO_MAG):
        return "D10"

    # Strobe / warning light. Mode switch first (换一种/第三种/样式), then
    # off, then on.
    if any(w in text for w in _STROBE_SUBJECT):
        if any(w in text for w in _STROBE_MODE):
            return "D18"
        if any(w in text for w in _ACT_OFF):
            return "D07"
        if any(w in text for w in _ACT_ON):
            return "D06"
        return None                        # strobe named but action unclear

    # Siren.
    if any(w in text for w in _SIREN_SUBJECT):
        if any(w in text for w in _ACT_OFF):
            return "D05"
        if any(w in text for w in ("开", "响", "拉", "来")):
            return "D04"
        return None

    # Illumination light. Brightness (D17) before on/off, because "太暗了"
    # / "亮一点" are a brightness change, not an on/off.
    light_subj = any(w in text for w in _LIGHT_SUBJECT) or ("灯" in text)
    if light_subj:
        if any(w in text for w in _ACT_BRIGHT) or any(
                w in text for w in _ACT_DARK):
            return "D17"
        if any(w in text for w in _ACT_OFF):
            return "D02"
        if any(w in text for w in _ACT_ON):
            return "D01"
        return None                        # light named but action unclear

    return None


# ---------------------------------------------------------------------------
# top-level layer-4 entry
# ---------------------------------------------------------------------------

def resolve_large_class(text: str) -> Optional[str]:
    """16 S5.2 layer-4 resolver for the device families. Tries the PTZ (E)
    rule, then the payload (D) rule. Returns an intent id, or None when the
    text carries no device signal (the chain then continues to layer 5/6).

    PTZ is tried first: its subject set (云台/镜头/...) and PTZ-only vocab do
    not overlap the payload subjects (灯/音量/警笛), so the order only settles
    degenerate inputs that name both, which do not occur in practice."""
    return resolve_ptz(text) or resolve_payload(text)
