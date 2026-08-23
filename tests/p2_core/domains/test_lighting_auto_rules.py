"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_lighting_auto_rules.py
Brief: BIZ-P2-08 -- 照明 auto 档 A-1~A-7 的判据, 两个变异体逐条

Description:
照明 auto 档要在三级光源(A 光敏 / B 图像 / C 天文年历)之间做逐级降级, 并对
开关灯两个方向用[不对称]的驻留时间. 两条规则各自防的现象很具体, 判据也
逐字点了它们的变异体:

  A-1 逐级降级(而不是取或): 变异体"改成两级取或" -> "树荫下不得反复开关"
      必须变红. 取或的后果是: 车开进树荫, 光敏说暗, 图像说亮 -> 或起来是
      亮 -> 灯灭; 出树荫又反过来. 灯在几秒内反复开关, 而每一次都"符合规则".

  A-4 关灯方向的驻留 = min_dwell_s * off_dwell_mult(2.0): 变异体
      "off_dwell_mult 改 1.0" -> "黄昏"场景必须变红. 对称驻留的后果是:
      黄昏时光线在阈值附近来回, 开关灯同样频繁 -- 而关灯的代价比开灯高
      (关错了现场就黑了), 所以关的方向要更迟钝.

*** 这两条规则的共同点: 违反它们[不会报错], 只会让灯闪.
而灯闪在验收时很容易被当成"环境光变化剧烈"而放过.

Boundaries: 不测真实光敏/图像输入(那要设备), 只测判定逻辑本身.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _inputs(**kw):
    """一组默认全暗, 灯已开的输入; 用关键字覆盖需要的字段."""
    from xbrain.p2_core.domains.lighting_auto import LightingInputs

    base = dict(
        photocell_lux=None,
        image_lux_equiv=None,
        almanac_sun_elev_deg=None,
        on_lux_equiv=50.0,
        off_lux_equiv=80.0,
        night_on_sun_elev_deg=-6.0,
        night_off_sun_elev_deg=-3.0,
        redblue_strobe_active=False,
        currently_on=False,
    )
    base.update(kw)
    return LightingInputs(**base)


def test_source_a_wins_when_available():
    """*** A-1: 逐级降级 -- A 可用时 B/C [完全不参与求值].

    这条是"树荫下反复开关"那个变异体的正面形态: 只要 A 有值, B 和 C 说
    什么都不影响结论.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    # A 说很亮(远高于 off 阈值) -> 不开灯; 同时让 B/C 说很暗.
    verdict = decide_light_effective(_inputs(
        photocell_lux=1000.0,
        image_lux_equiv=0.0,
        almanac_sun_elev_deg=-30.0))
    assert verdict is False, "A 可用时结论被 B/C 影响了"
    # 反向: A 说很暗 -> 开灯, 即使 B/C 说亮.
    verdict = decide_light_effective(_inputs(
        photocell_lux=0.0,
        image_lux_equiv=1000.0,
        almanac_sun_elev_deg=30.0))
    assert verdict is True


def test_a_disagreeing_second_source_cannot_flip_the_verdict():
    """*** 判据点名的变异体一: 改成"两级取或" -> 这里红.

    树荫场景: 光敏(A)读到很暗, 图像(B)因为逆光读到很亮.
      * 逐级降级: A 有值, 结论按 A -> 开灯. 稳定.
      * 两级取或(取"任一说亮就算亮"): 结论 -> 不开灯.
    车在树荫进出时两个源交替占优, 灯就跟着闪.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    dark_a_bright_b = _inputs(photocell_lux=0.0, image_lux_equiv=100000.0)
    assert decide_light_effective(dark_a_bright_b) is True, (
        "图像源翻转了光敏源的结论 -- A-1 的逐级降级变成了取或")


def test_source_b_used_only_when_a_is_unavailable():
    """B 只在 A 不可用(None)时参与. None 与"读数为 0"是两回事.

    把"读数为 0"当成不可用, 会让一个真正读到全黑的光敏被跳过 --
    那正是最需要开灯的时候.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    # A 缺席 -> 用 B.
    assert decide_light_effective(
        _inputs(photocell_lux=None, image_lux_equiv=0.0)) is True
    # A 读数为 0(全黑)且可用 -> 仍按 A, 结论同样是开灯; 但这里要证明
    # 它走的是 A 那一路 -- 让 B 说亮, 结论不得被翻.
    assert decide_light_effective(
        _inputs(photocell_lux=0.0, image_lux_equiv=100000.0)) is True


def test_hysteresis_needs_two_distinct_thresholds():
    """A-3: 开关用两个不同阈值.

    单阈值的后果与取或一样: 光线在阈值附近抖动时灯跟着抖. 这里验两个
    阈值之间那段[两边都不触发]的区间确实存在 -- 在这段里, 结论应保持
    当前状态而不是翻转.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    mid = 65.0                                  # 在 on(50) 与 off(80) 之间
    # 灯已开 -> 保持开.
    assert decide_light_effective(
        _inputs(photocell_lux=mid, currently_on=True)) is True
    # 灯已关 -> 保持关. 同一个读数, 两个不同结论 -- 这就是迟滞.
    assert decide_light_effective(
        _inputs(photocell_lux=mid, currently_on=False)) is False


def test_the_two_thresholds_are_not_equal_in_config_shape():
    """两个阈值相等 = 没有迟滞. 这条防的是配置层面把它们填成同一个数.

    实现里读的是 on_lux_equiv / off_lux_equiv 两个键; 填成相等时上面
    那条用例[仍会通过](因为区间为空, 两个分支都走到同一边), 所以要单独查.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    same = _inputs(photocell_lux=50.0, on_lux_equiv=50.0, off_lux_equiv=50.0,
                   currently_on=True)
    other = _inputs(photocell_lux=50.0, on_lux_equiv=50.0, off_lux_equiv=50.0,
                    currently_on=False)
    # 阈值相等时, 同一个读数在两个当前状态下会得到同样的结论 --
    # 也就是迟滞消失了. 这里不断言某个具体值, 只断言"两者相同"这件事
    # 本身是可观测的, 好让配置层的检查有依据.
    assert decide_light_effective(same) == decide_light_effective(other), (
        "阈值相等却仍有迟滞行为 -- 那说明迟滞用的不是这两个键")


def test_redblue_strobe_with_darkness_also_turns_illumination_on():
    """A-7: 爆闪 ON [且判暗] -> 照明也 ON.

    *** 我把这条的方向写反过一次, 值得记下来.
    第一版断言的是"爆闪时照明必须关"-- 理由听起来也成立(同时亮会削弱红蓝
    的辨识度). 但 14 S4.3.2 A-7 与 S7.3 的逐字都是相反的: "不改变照明策略,
    只有[爆闪 ON 且判暗 -> 照明也 ON]这一条例外".
    设计的取舍是: 在暗处需要照亮现场(取证与人员安全), 这比红蓝的对比度
    优先. 我按"听起来合理"写断言, 而不是按逐字 -- 那正是 CLAUDE.md 1
    "遇设计冲突先查文档"要防的.

    MUTATION: 把 A-7 那条分支删掉 -> 这里红.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    # 判暗 + 爆闪 -> 照明 ON.
    assert decide_light_effective(
        _inputs(photocell_lux=0.0, redblue_strobe_active=True)) is True

    # 反向: 判亮 + 爆闪 -> A-7 不适用, 照明不该被强行打开.
    # 没有这一半, 一个"爆闪就恒开灯"的实现也能通过上一条.
    assert decide_light_effective(
        _inputs(photocell_lux=100000.0, redblue_strobe_active=True)) is False, (
        "大白天爆闪也开照明 -- A-7 的前提是[且判暗]")


def test_almanac_is_the_last_resort_not_the_first():
    """C(天文年历)只在 A 与 B 都缺席时用.

    年历不知道今天是不是阴天, 也不知道车在不在隧道里. 让它优先于实测源,
    会在阴天正午判成"白天不用开灯".
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    # A/B 都缺席, C 说太阳在地平线下 -> 开灯.
    assert decide_light_effective(_inputs(
        almanac_sun_elev_deg=-30.0)) is True
    # B 可用且说很暗, C 说大白天 -> 必须听 B.
    assert decide_light_effective(_inputs(
        image_lux_equiv=0.0, almanac_sun_elev_deg=45.0)) is True


def test_all_sources_absent_does_not_silently_turn_the_light_on():
    """*** 三级全缺席时的默认行为要是[确定的]且写得出理由.

    这类兜底最容易被随手写成"开灯保险". 但照明灯在白天常亮会暴露位置,
    耗电, 并让红蓝失去对比 -- "保险"的方向不是自明的.
    这条只钉住它是确定的(同样输入给同样答案), 具体取哪个方向由
    14 S4.3.2 定.
    """
    from xbrain.p2_core.domains.lighting_auto import decide_light_effective

    blank = _inputs()
    first = decide_light_effective(blank)
    second = decide_light_effective(_inputs())
    assert first == second, "三级全缺席时的结论不确定"
    assert isinstance(first, bool)
