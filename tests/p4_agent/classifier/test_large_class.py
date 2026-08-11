"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_large_class.py
Brief: layer-4 device-family router (E ptz / D payload) + its mutation guards

Description:
Tests classifier.large_class -- the 16 S5.2 layer-4 "大类 + 类内规则" for the
two deterministic device closed-classes. The operator's 2026-08-11 ruling is
the spec under test:

  * a PTZ subject (云台 and its aliases, incl. the ASR mishear 平台) promotes an
    OVERLAPPING direction word to PTZ; WITHOUT the subject the same word is the
    chassis's (A09/A10/A13), so the router must NOT claim it;
  * PTZ-only vocab (tilt up/down, zoom, scan, 转速) routes with no subject;
  * payload subjects (灯/爆闪/警笛/音量) route lights/strobe/siren/volume, with
    strobe+siren resolved before the broad "灯" branch.

Every behavioural assertion is paired with a MUTATION note (CLAUDE.md 3.3):
the comment states the code change that would make the assertion go red, so
an empty / mis-ordered implementation cannot pass. The two most important
mutations -- "route bare 左/右 to PTZ" (steals the chassis) and "match bare
下 in 一下" -- have dedicated negative tests.
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.classifier.large_class import (
    resolve_large_class, resolve_payload, resolve_ptz,
)


# -- E class: subject promotes overlapping directions -------------------------

@pytest.mark.parametrize("text,intent", [
    ("云台朝左", "E01"),        # 朝左 -> left, subject present
    ("镜头向左", "E01"),        # alias subject 镜头
    ("云台左转", "E01"),        # 左转 (shared with chassis A09) + subject
    ("云台向右转", "E01"),
    ("平台往右", "E01"),        # 平台 = ASR mishear of 云台 (operator-confirmed)
])
def test_ptz_direction_with_subject(text, intent):
    # MUTATION: drop the subject gate (route bare left/right) -> 向左转/左转
    # below would ALSO become E01 and steal the chassis. That regression is
    # caught by test_bare_leftright_is_not_ptz.
    assert resolve_ptz(text) == intent


@pytest.mark.parametrize("text", ["向左转", "左转", "向右转", "右转"])
def test_bare_leftright_is_not_ptz(text):
    """Operator rule: a bare (no-subject) left/right is the CHASSIS (A09/A10),
    never PTZ. MUTATION: routing left/right without a subject returns E01
    here and silently overrides the chassis turn."""
    assert resolve_ptz(text) is None
    assert resolve_large_class(text) is None


@pytest.mark.parametrize("text,intent", [
    ("向下看", "E01"),          # tilt down: no chassis twin -> route bare
    ("往上看", "E01"),          # tilt up
    ("低头", "E01"),
    ("抬高一点", "E01"),
])
def test_ptz_tilt_updown_needs_no_subject(text, intent):
    # MUTATION: require a subject for up/down too -> these (which no operator
    # prefixes with 云台) would stop working, the exact 死板 we are removing.
    assert resolve_ptz(text) == intent


def test_bare_char_in_yixia_is_not_a_direction():
    """The '下' inside the amount word '一下' (and 楼下 / 上午) must NOT read as
    a tilt-down. MUTATION: classify with allow_bare=True (or match single
    chars) -> '照明灯开一下' becomes E01 tilt-down instead of D01 light-on."""
    assert resolve_ptz("云台看一下") is None      # subject present, still no dir
    assert resolve_ptz("楼下有人") is None
    assert resolve_large_class("照明灯开一下") == "D01"
    assert resolve_large_class("开一下爆闪灯") == "D06"


# -- E class: PTZ-only vocab (zoom / scan / speed) ----------------------------

@pytest.mark.parametrize("text,intent", [
    ("变焦", "E06"), ("拉近一点", "E06"), ("云台放大", "E06"),
    ("左右扫一遍", "E07"), ("环视一周", "E07"), ("云台向左环视一周", "E07"),
    ("转速最快", "E09"), ("云台转速调到最慢", "E09"),
])
def test_ptz_only_vocab_routes(text, intent):
    assert resolve_ptz(text) == intent


def test_scan_checked_before_direction():
    """'云台向左环视一周' has BOTH a direction (左) and a scan word: it must be
    E07 (orbit), not E01. MUTATION: check direction before scan -> returns
    E01 and the head does a small move instead of a full turn."""
    assert resolve_ptz("云台向左环视一周") == "E07"


def test_speed_needs_zhuansu_or_subject():
    """'转速' is PTZ-only; a bare 快一点/慢一点 is the chassis (A13) and must
    NOT become E09. MUTATION: route any speed-level word to E09 -> bare
    快一点 is stolen from the chassis."""
    assert resolve_ptz("转速调到最慢") == "E09"     # 转速 present
    assert resolve_ptz("云台转慢点") == "E09"        # subject present
    assert resolve_ptz("快一点") is None             # neither -> chassis
    assert resolve_ptz("慢一点") is None


def test_ambiguous_zoom_needs_subject():
    """大一点/小一点 are shared with volume: they route to PTZ zoom ONLY with a
    subject. MUTATION: route ambiguous zoom words bare -> a bare '大一点'
    becomes E06 and collides with volume."""
    assert resolve_ptz("云台大一点") == "E06"        # subject -> zoom
    assert resolve_ptz("大一点") is None             # bare -> neither


# -- D class: payload ---------------------------------------------------------

@pytest.mark.parametrize("text,intent", [
    ("把照明灯搞亮点", "D17"), ("灯光太暗了", "D17"),
    ("照明灯开一下", "D01"), ("把灯关了", "D02"),
    ("声音大一点", "D10"), ("音量小一点", "D10"),
    ("把爆闪关掉", "D07"), ("开一下爆闪灯", "D06"), ("警灯换一种", "D18"),
    ("拉响警报", "D04"), ("关警笛", "D05"),
])
def test_payload_family_routes(text, intent):
    assert resolve_payload(text) == intent


def test_strobe_resolved_before_bare_light():
    """'爆闪灯'/'警灯' both contain '灯'; strobe/siren MUST be resolved before
    the broad light branch. MUTATION: put the '灯' branch first -> '把爆闪关掉'
    becomes D02 (light off) instead of D07 (strobe off)."""
    assert resolve_payload("把爆闪关掉") == "D07"
    assert resolve_payload("警灯换一种") == "D18"


def test_volume_needs_audio_anchor():
    """Volume needs 音量/声+magnitude; a bare 大一点 is not a volume command.
    MUTATION: drop the audio-anchor requirement -> '大一点' becomes D10 and
    fights the zoom path."""
    assert resolve_payload("声音大一点") == "D10"
    assert resolve_payload("大一点") is None
    assert resolve_payload("说一声") is None          # 声 without magnitude


# -- non-device text is never claimed ----------------------------------------

@pytest.mark.parametrize("text", [
    "去充电", "开始巡逻", "你好", "前进", "今天天气不错", "",
])
def test_non_device_returns_none(text):
    assert resolve_large_class(text) is None
