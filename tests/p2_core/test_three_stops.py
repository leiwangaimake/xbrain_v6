"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_three_stops.py
Brief: BIZ-P2-21 三停统一处理 + 域1 缴械的判据 (soft_estop/hes/cmd_timeout)

Description:
three_stops.apply_stop 写好后一直[没有测试也没有调用者](与批59查出的
同型). 本文件先把既有逻辑钉住, 再由 p2 接线(test_p2_estop_wiring)用它.

*** 本文件的重心是 14 S3.7 那句"只停车, 不锁机"的精确表达:
  域1 (motion) 缴械 -> suspended() == "soft_estop"
  域2/3/4/5 一个都不缴械 (siren 继续响, 语音继续工作)
  域4 强制爆闪(SE-1) -- 但用 arb_suspend? 不, 是 strobe flag, 域4 不缴械
一个把域4 也缴械的实现会让红蓝爆闪的 mode_driver 丢掉持有 -- 车停在路中间
反而不闪了, 而 14 S3.7 逐字"急停后车停在路中间, 必须被看见".

*** 恢复方式是断言里最容易写错的一处.
14 S3.7 表逐字: soft_estop 恢复 = "新运动指令即可, 无需显式解除". 所以
apply_rearm 由[新运动命令]触发, 不是由一条"解除急停"的命令 -- 后者不存在.

Boundaries: 纯逻辑. 与 cmd/estop 订阅, state/arb/motion 广播的接线在
test_p2_estop_wiring.py.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _motion_arb():
    """真 Arbiter, 域名 motion. DOMAIN.parse 会校验 motion 在闭集里 --
    构造不抛就证明域1 是合法域名(否则本文件根本建不出来)."""
    from xbrain.common.arbiter.core import Arbiter

    # wait_atomic_timeout 对 estop 缴械不参与判定(arb_suspend 不碰它),
    # 给一个名义值即可; 真实接线从 arbiter.wait_atomic_timeout_s 读.
    return Arbiter("motion", wait_atomic_timeout_ms=3000)


def _harness():
    from xbrain.p2_core.three_stops import ForceStrobeState

    arb = _motion_arb()
    strobe = ForceStrobeState()
    events = []
    return arb, strobe, events.append, events


# --- 缴械 -------------------------------------------------------------

def test_soft_estop_disarms_domain1():
    """*** 域1 缴械 = suspended() 变 soft_estop.

    这是软急停在框架里的落点(14 S3.7.1: 作用于仲裁器本身的一次性操作,
    不是加一个源). p1 读 state/arb/motion.suspended 后零速 hold.

    MUTATION: apply_stop 里删掉 arb_suspend 调用 -> 这里红.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_stop)

    arb, strobe, emit, _ev = _harness()
    assert arb.suspended() is None                       # 基线: 未缴械

    apply_stop(StopEvent(StopReason.SOFT_ESTOP, "e-1", 1000), arb, strobe, emit)

    assert arb.suspended() == "soft_estop", (
        "域1 没有被缴械 -- p1 读不到 suspended, 软急停对运动无效")


def test_the_strobe_is_forced_on_but_domain4_is_not_disarmed():
    """*** 域4 强制爆闪靠 flag, NO 不靠 arb_suspend.

    14 S3.7: 域4 不缴械(mode_driver 保持持有), 只是把爆闪强制 ON. 缴械域4
    会让爆闪的持有者被撤 -- 车停在路中间反而不闪了, 与"必须被看见"相反.

    MUTATION: apply_stop 里删掉 strobe_state.active = True -> 这里红.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_stop)

    arb, strobe, emit, _ev = _harness()
    assert strobe.active is False

    apply_stop(StopEvent(StopReason.SOFT_ESTOP, "e-1", 1000), arb, strobe, emit)

    assert strobe.active is True, "红蓝爆闪没有被强制打开 -- 停在路中间看不见"


def test_a_stop_emits_one_audit_event_naming_the_reason():
    """三停统一, reason 只调 detail. 审计事件必须点名是哪一停.

    MUTATION: apply_stop 里 emit 的 detail.reason 写死 -> 三停区分那条红.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_stop)

    arb, strobe, emit, events = _harness()

    apply_stop(StopEvent(StopReason.SOFT_ESTOP, "e-9", 1000), arb, strobe, emit)

    assert len(events) == 1
    assert events[0]["kind"] == "estop"
    assert events[0]["detail"]["reason"] == "soft_estop"
    assert events[0]["detail"]["cmd_id"] == "e-9"


def test_all_three_stops_share_one_branch_differing_only_in_reason():
    """*** BIZ-P2-21 逐字"三停处理分支数 == 1".

    hes 与 cmd_timeout 走同一个 apply_stop, 结果只有 detail.reason 不同 --
    域1 都缴械, 域4 都爆闪. 一个为 hes 单开分支的实现会在两条路径间漂移.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_stop)

    for reason, want in ((StopReason.HES, "hes"),
                         (StopReason.CMD_TIMEOUT, "cmd_timeout")):
        arb, strobe, emit, events = _harness()
        apply_stop(StopEvent(reason, "c-1", 1000), arb, strobe, emit)
        assert arb.suspended() == want
        assert strobe.active is True
        assert events[0]["detail"]["reason"] == want


def test_a_repeated_estop_under_the_same_cmd_id_is_idempotent():
    """*** 同一 cmd_id 二次到达不重复缴械(BIZ-CM-3 幂等).

    同一软急停经 cmd/estop 与 state/robot 两条路径到达, 共用一个 cmd_id.
    第二次已缴械, arb_suspend 返回 None(不再发第二条 suspend 审计).
    验证 apply_stop 不因二次调用而改变缴械态或造出矛盾.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_stop)

    arb, strobe, emit, _ev = _harness()
    apply_stop(StopEvent(StopReason.SOFT_ESTOP, "e-1", 1000), arb, strobe, emit)
    apply_stop(StopEvent(StopReason.SOFT_ESTOP, "e-1", 1100), arb, strobe, emit)

    assert arb.suspended() == "soft_estop"       # 仍缴械, 不翻转


# --- 恢复 -------------------------------------------------------------

def test_rearm_clears_the_disarm_and_the_strobe():
    """*** 新运动指令解除缴械(14 S3.7: 无需显式解除命令).

    MUTATION: apply_rearm 里删掉 arb_rearm -> suspended 那条红.
    MUTATION: apply_rearm 里删掉 strobe.active = False -> strobe 那条红.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_rearm,
                                            apply_stop)

    arb, strobe, emit, events = _harness()
    apply_stop(StopEvent(StopReason.SOFT_ESTOP, "e-1", 1000), arb, strobe, emit)
    assert arb.suspended() == "soft_estop" and strobe.active is True

    apply_rearm("m-2", 2000, arb, strobe, emit)

    assert arb.suspended() is None, "新运动指令没有解除缴械 -- 机器人再也不动"
    assert strobe.active is False, "爆闪没有随 re-arm 关掉"
    assert events[-1]["kind"] == "estop_rearm"


def test_rearm_on_an_armed_domain_is_a_noop():
    """未缴械时 re-arm 是空操作(arb_rearm 幂等返回 None).

    没有这条, 一个"每条运动指令都 rearm"的实现会在正常运行时不停发
    estop_rearm 审计事件 -- 而根本没发生过急停.
    """
    from xbrain.p2_core.three_stops import apply_rearm

    arb, strobe, emit, events = _harness()
    apply_rearm("m-1", 1000, arb, strobe, emit)   # 从未缴械

    assert arb.suspended() is None
    assert strobe.active is False
    # apply_rearm 仍 emit 一条(它不查 arb_rearm 的返回); 但缴械态不变.
    # 真实接线只在[确实处于缴械]时才调 apply_rearm, 见 test_p2_estop_wiring.
